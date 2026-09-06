//! End-to-end tests against the demo map.
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

fn fixtures(policy: &str) -> (MapConfig, ScenarioConfig, Policy) {
    let read = |rel: String| fs::read_to_string(format!("{ROOT}/{rel}")).unwrap();
    let map = load_map(&read("maps/demo_loop.json".into())).unwrap();
    let scen = load_scenario(&read("scenarios/baseline.json".into()), &map.grid).unwrap();
    let pol = load_policy(&read(format!("policies/{policy}.json"))).unwrap();
    (map, scen, pol)
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

/// What buffers are actually for, stated precisely.
///
/// They are *not* what prevents the deadlock -- at the shipped cap the fab
/// completes the same number of lots with them and without. What they buy is
/// tolerance: they widen the range of WIP settings that stay safe, so a cap set
/// too high degrades instead of killing the fab. At a cap of 24 that is the
/// difference between running and stopping.
#[test]
fn buffers_widen_the_safe_wip_range() {
    let build = |cap: usize, buffers: bool| {
        let (mut map, mut scen, pol) = fixtures("default");
        if !buffers {
            map.machines.retain(|m| !m.is_buffer());
        }
        scen.wip_cap = cap;
        let mut w = World::new(map, scen, pol);
        w.run(30_000);
        w
    };

    // At a cap set too high, buffers are the difference between running and not.
    let loose_with = build(24, true);
    let loose_without = build(24, false);
    assert_eq!(loose_with.metrics.resource_deadlock_events, 0);
    assert!(
        loose_without.metrics.resource_deadlock_events > 0,
        "without buffers a cap of 24 should still deadlock; if it no longer \
         does, the safe range moved and this test is measuring nothing"
    );

    // At the shipped cap they are not load-bearing, and saying so keeps anyone
    // from mistaking them for the fix.
    let tight_with = build(16, true);
    let tight_without = build(16, false);
    assert_eq!(tight_with.metrics.resource_deadlock_events, 0);
    assert_eq!(tight_without.metrics.resource_deadlock_events, 0);
}
