//! End-to-end tests against the shipped maps.
//!
//! The unit tests in `src/movement.rs` cover the resolver in isolation. These
//! cover the properties that only show up once the whole tick loop runs, and
//! that a chaotic system makes easy to break silently: a policy change that
//! gridlocks the fab still "passes" every unit test.

use std::fs;

use ohtsim::model::VehState;
use ohtsim::{load_map, load_policy, load_scenario, MapConfig, Policy, ScenarioConfig, World};

/// Maps, scenarios and policies live at the repo root, not inside the crate --
/// they are project content shared with the web UI, not test fixtures.
const ROOT: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../..");

/// A scenario is written against one map, so the pair travels together. Same
/// pairs the UI's fab picker offers.
const DEMO: (&str, &str) = ("maps/demo_loop.json", "scenarios/baseline.json");
const FAB: (&str, &str) = ("maps/fab.json", "scenarios/fab.json");

fn read(rel: &str) -> String {
    fs::read_to_string(format!("{ROOT}/{rel}")).unwrap()
}

fn fixtures_of(scene: (&str, &str), policy: &str) -> (MapConfig, ScenarioConfig, Policy) {
    let map = load_map(&read(scene.0)).unwrap();
    let scen = load_scenario(&read(scene.1), &map.grid).unwrap();
    let pol = load_policy(&read(&format!("policies/{policy}.json"))).unwrap();
    (map, scen, pol)
}

fn fixtures(policy: &str) -> (MapConfig, ScenarioConfig, Policy) {
    fixtures_of(DEMO, policy)
}

fn world(policy: &str) -> World {
    let (m, s, p) = fixtures(policy);
    World::new(m, s, p)
}

/// Policy comparison is meaningless unless two runs see an identical job
/// stream, so determinism is a correctness property, not a nicety.
#[test]
fn runs_are_deterministic() {
    let mut a = world("starvation_biased");
    let mut b = world("starvation_biased");
    a.run(3_000);
    b.run(3_000);
    assert_eq!(a.metrics.lots_completed, b.metrics.lots_completed);
    assert_eq!(a.metrics.cycle_times, b.metrics.cycle_times);
    assert_eq!(a.metrics.stuck_vehicle_events, b.metrics.stuck_vehicle_events);
}

/// Two vehicles committing to the same empty spur is the bug that produced a
/// permanently blocked main line: the loser arrives, finds the spur taken, and
/// re-decides from wherever it is standing, which is on the loop.
#[test]
fn no_two_vehicles_claim_the_same_spur() {
    for policy in ["default", "starvation_biased"] {
        let mut w = world(policy);
        for _ in 0..3_000 {
            w.tick();
            let mut claims: Vec<usize> = w
                .vehicles
                .iter()
                .filter(|v| v.state == VehState::Repositioning)
                .filter_map(|v| v.route.last().copied())
                .filter(|c| w.parking.contains(c))
                .collect();
            let before = claims.len();
            claims.sort_unstable();
            claims.dedup();
            assert_eq!(
                before,
                claims.len(),
                "{policy}: two vehicles are repositioning to the same spur at tick {}",
                w.tick_count
            );
        }
    }
}

/// A vehicle idling on the main line blocks everything behind it, and with no
/// overtaking that congestion propagates backward until the fab gridlocks.
///
/// It cannot be driven to zero: a vehicle that finishes a job is Idle for the
/// one tick before it re-decides where to go. But it should never *stay* there
/// -- a vehicle with no free spur circulates instead of stopping, so the count
/// is now bounded by that handoff rather than by how many spurs are free.
#[test]
fn idle_vehicles_do_not_camp_on_the_main_line() {
    for (policy, limit) in [("default", 250), ("starvation_biased", 250)] {
        let mut w = world(policy);
        let mut blocked_ticks = 0;
        for _ in 0..3_000 {
            w.tick();
            let camping = w
                .vehicles
                .iter()
                .any(|v| v.state == VehState::Idle && !w.parking.contains(&v.cell));
            if camping {
                blocked_ticks += 1;
            }
        }
        assert!(
            blocked_ticks < limit,
            "{policy}: idle vehicle blocking the main line on {blocked_ticks}/3000 ticks \
             (limit {limit}) -- prepositioning is probably stranding vehicles again"
        );
    }
}

/// Both shipped policies must actually run the fab. The failure this catches is
/// not a crash but a quiet collapse: the source backs up, arrivals stop, and
/// the run still reports clean metrics over a fab that did almost nothing.
#[test]
fn shipped_policies_do_not_gridlock() {
    for policy in ["default", "starvation_biased"] {
        let mut w = world(policy);
        w.run(10_000);
        assert_eq!(
            w.metrics.deadlock_events, 0,
            "{policy}: deadlocked {} times",
            w.metrics.deadlock_events
        );
        assert!(
            w.metrics.lots_completed >= 30,
            "{policy}: only {} lots completed in 10k ticks -- the fab is jammed",
            w.metrics.lots_completed
        );
    }
}


/// The invariant behind circulation: a vehicle standing still on the main line
/// is a wall, because rails are one-way and nothing can overtake it.
///
/// Blocking is legitimate and expected -- a 20-tick hoist blocks everything
/// behind it, and that is the congestion the game is about. What must not
/// happen is a vehicle blocking others because it *parked on the loop*: it has
/// no job, it is not hoisting, it is simply stopped somewhere it should never
/// have stopped. Before vehicles circulated, that accounted for over nine
/// thousand vehicle-ticks of blocking per twenty thousand ticks.
#[test]
fn vehicles_do_not_stop_on_the_main_line_and_block_others() {
    for policy in ["default", "starvation_biased"] {
        let mut w = world(policy);
        let parking = w.parking.clone();
        let mut blocked = 0u64;
        let mut worst = 0u32;
        for _ in 0..20_000 {
            w.tick();
            for v in &w.vehicles {
                let Some(next) = v.next_cell() else { continue };
                let Some(other) = w.occupancy[next] else { continue };
                let blocker = &w.vehicles[other];
                if blocker.state != VehState::Idle || parking.contains(&blocker.cell) {
                    continue;
                }
                blocked += 1;
                worst = worst.max(v.blocked_ticks);
            }
        }
        assert!(
            blocked < 1_500,
            "{policy}: {blocked} vehicle-ticks spent blocked behind a vehicle stopped \
             on the main line (worst streak {worst}) -- unparkable vehicles should be \
             circulating, not halting on the loop"
        );
    }
}

/// The other kind of deadlock, and the one the demo map actually hits.
///
/// Recipes are reentrant -- litho, etch, cmp, litho -- and every place a lot can
/// rest is finite: input ports, chambers, output ports, buffer slots. Admit
/// enough lots to fill them all and the tools end up holding finished lots for
/// each other in a cycle, litho waiting on etch, etch on cmp, cmp on litho, with
/// nothing able to move. Uncapped it takes about 90,000 ticks to close and it is
/// permanent: the fab completes 647 lots and then nothing, forever.
///
/// The fix is release control, not storage. Capping work in progress keeps the
/// fab off the part of the curve where that can happen.
#[test]
fn a_wip_cap_prevents_the_resource_deadlock() {
    // Well past the ~90k where the uncapped fab dies.
    const HORIZON: u64 = 100_000;

    let mut capped_completed = 0;
    for policy in ["default", "starvation_biased"] {
        let mut w = world(policy);
        w.run(HORIZON);
        assert_eq!(
            w.metrics.resource_deadlock_events, 0,
            "{policy}: deadlocked at the shipped WIP cap ({} ticks stuck)",
            w.metrics.resource_deadlock_ticks
        );
        if policy == "default" {
            capped_completed = w.metrics.lots_completed;
        }
    }

    // The other half: without the cap it dies. Asserting only the first half
    // would also pass with a detector that never fires.
    let (map, mut scen, pol) = fixtures("default");
    scen.wip_cap = 0;
    let mut uncapped = World::new(map, scen, pol);
    uncapped.run(HORIZON);
    assert!(
        uncapped.metrics.resource_deadlock_events > 0,
        "removing the WIP cap should reopen the deadlock, but the detector saw \
         none -- detector or release control is broken"
    );
    // Terminally stuck, not briefly stalled: it froze around tick 90,000 and
    // stays frozen, so the longer the horizon the wider the gap. Asserting the
    // stuck time rather than a completion ratio, because at this horizon the
    // uncapped fab has only just died and the ratio is still close.
    assert!(
        uncapped.metrics.resource_deadlock_ticks > 5_000,
        "uncapped should be stuck for good, not momentarily: {} ticks",
        uncapped.metrics.resource_deadlock_ticks
    );
    assert!(
        uncapped.metrics.lots_completed < capped_completed,
        "the cap should complete more lots: {} uncapped against {} capped",
        uncapped.metrics.lots_completed,
        capped_completed
    );
}


/// Every vehicle must be able to reach the fab from wherever it parks.
///
/// Spurs on the demo map are two cells deep, and routing refuses to path
/// *through* a spur so a loaded vehicle never ends up behind a parked one. Taken
/// literally that also forbids driving *out* of one, and the four vehicles that
/// happened to start on inner spur cells could not route anywhere at all: they
/// sat still for the entire run and were never assigned a single job. Half the
/// fleet was dead and every metric was quietly measuring the other half.
///
/// Nothing caught it because a fab with half a fleet still completes lots. So
/// this asserts the thing that was actually false: every vehicle does some work.
#[test]
fn every_vehicle_gets_used() {
    for policy in ["default", "starvation_biased"] {
        let mut w = world(policy);
        let n = w.vehicles.len();
        let mut worked = vec![false; n];
        for _ in 0..20_000 {
            w.tick();
            for (i, v) in w.vehicles.iter().enumerate() {
                if matches!(v.state, VehState::ToPickup(_) | VehState::ToDropoff(_)) {
                    worked[i] = true;
                }
            }
        }
        let idle: Vec<usize> = (0..n).filter(|&i| !worked[i]).collect();
        assert!(
            idle.is_empty(),
            "{policy}: vehicles {idle:?} never carried a job in 20k ticks -- \
             they are probably unable to route off their parking spur"
        );
    }
}

/// The busiest and least busy vehicle should be doing comparable amounts.
///
/// A weaker version of the above that also catches a dispatcher that merely
/// *prefers* the same few vehicles, rather than one that strands the rest.
#[test]
fn work_is_spread_across_the_fleet() {
    let mut w = world("default");
    let n = w.vehicles.len();
    let mut busy = vec![0u64; n];
    for _ in 0..40_000 {
        w.tick();
        for (i, v) in w.vehicles.iter().enumerate() {
            if !v.is_idle() {
                busy[i] += 1;
            }
        }
    }
    let hi = *busy.iter().max().unwrap();
    let lo = *busy.iter().min().unwrap();
    assert!(
        lo * 3 >= hi,
        "fleet load is lopsided: busiest vehicle {hi} ticks, least busy {lo} ({busy:?})"
    );
}

// ---------------------------------------------------------------------------
// The fab map
// ---------------------------------------------------------------------------

/// The generated map has to satisfy the same rules a hand-drawn one does.
///
/// `reference/gen_fab.py` has its own checks, and they passed on a map this
/// validator rejected: the generator left exit bits pointing off the edge of
/// the grid, which its own connectivity check never looked at. Generated
/// content is not exempt from validation, it is exactly what needs it.
#[test]
fn the_shipped_maps_validate() {
    for scene in [DEMO, FAB] {
        let map = load_map(&read(scene.0)).unwrap();
        let problems = ohtsim::validate(&map);
        assert!(
            problems.is_empty(),
            "{}: {}",
            scene.0,
            problems
                .iter()
                .map(|p| p.to_string())
                .collect::<Vec<_>>()
                .join("; ")
        );
    }
}

/// The fab's WIP cap has to hold across job streams, not just the shipped seed.
///
/// The cap is a cliff, not a slope. At 36 the fab ran clean for a million ticks
/// on five seeds out of six and wedged permanently on the sixth -- at tick
/// 13,294, during the fill transient, which is when the reentrant flow is most
/// able to close a circular port dependency. A cap tuned on one seed is a cap
/// tuned on one coin flip, so this runs several and covers the transient.
#[test]
fn the_fab_cap_survives_its_fill_transient() {
    // Comfortably past the fill: the fab reaches its cap around 5k and the
    // known failure lands at 13k.
    const HORIZON: u64 = 40_000;

    for i in 0..6u64 {
        let (map, mut scen, pol) = fixtures_of(FAB, "default");
        let cap = scen.wip_cap;
        scen.seed = 20260903 + i * 7919;
        let seed = scen.seed;
        let mut w = World::new(map, scen, pol);
        w.run(HORIZON);
        assert_eq!(
            w.metrics.resource_deadlock_events, 0,
            "seed {seed} deadlocked at the shipped cap of {cap} ({} ticks stuck)",
            w.metrics.resource_deadlock_ticks
        );
        assert!(
            w.metrics.lots_completed > 0,
            "seed {seed} completed nothing in {HORIZON} ticks"
        );
    }
}

/// The reason the fab map exists: dispatch has to have something to choose
/// between.
///
/// On the demo map the mean number of assignable vehicles was 0.24 and dispatch
/// ranked a mean of 2.3 candidates when it planned at all. A weighted scoring
/// function cannot express anything with one candidate, so the whole
/// configuration space was inert -- ten of twelve knobs moved throughput less
/// than the seed-to-seed noise. Asserting it here because it is a property of
/// the map and the scenario together, and either can be edited back into
/// inertness without anything else failing.
///
/// Both numbers below are sampled after the tick, so both are lower bounds:
/// dispatch runs inside the tick, and the work it managed to assign is gone by
/// the time this looks. The second one is a weak signal for that reason -- the
/// fab measures 15.3% against the demo map's 2.4% -- and the first is the
/// telling one: 14.6 assignable vehicles against 0.24.
#[test]
fn the_fab_gives_dispatch_a_real_choice() {
    const HORIZON: u64 = 60_000;

    let (map, scen, pol) = fixtures_of(FAB, "default");
    let mut w = World::new(map, scen, pol);
    let (mut idle_sum, mut choice) = (0u64, 0u64);
    for _ in 0..HORIZON {
        w.tick();
        let idle = w
            .vehicles
            .iter()
            .filter(|v| v.is_idle() || v.state == VehState::Repositioning)
            .count() as u64;
        idle_sum += idle;
        if idle > 1 && w.pending_len() > 1 {
            choice += 1;
        }
    }
    let mean_idle = idle_sum as f64 / HORIZON as f64;
    let choice_pct = choice as f64 / HORIZON as f64 * 100.0;
    assert!(
        mean_idle > 5.0,
        "only {mean_idle:.2} assignable vehicles on average; dispatch has \
         nothing to pick from"
    );
    assert!(
        choice_pct > 10.0,
        "leftover work and free vehicles coincide on only {choice_pct:.1}% of \
         ticks; the fab measures 15.3% and the demo map 2.4%"
    );
}
