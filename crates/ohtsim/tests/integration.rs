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
/// Recipes are reentrant -- litho, etch, cmp, litho -- so with only tool ports
/// to put lots on, the tools can end up holding finished lots for each other in
/// a cycle: litho waits on etch, etch on cmp, cmp on litho, and nothing can ever
/// move. Under the default policy it closed at around tick 12,000 and the fab
/// stopped, with metrics that looked merely disappointing rather than broken.
///
/// Buffers break it by giving a finished lot somewhere to go that is not another
/// tool's port.
#[test]
fn buffers_prevent_the_resource_deadlock() {
    for policy in ["default", "starvation_biased"] {
        let mut with = world(policy);
        with.run(20_000);
        assert_eq!(
            with.metrics.resource_deadlock_events, 0,
            "{policy}: deadlocked despite buffers ({} ticks stuck)",
            with.metrics.resource_deadlock_ticks
        );

        let (mut map, scen, pol) = fixtures(policy);
        map.machines.retain(|m| !m.is_buffer());
        let mut without = World::new(map, scen, pol);
        without.run(20_000);
        assert!(
            without.metrics.lots_completed < with.metrics.lots_completed,
            "{policy}: buffers should raise throughput ({} without, {} with)",
            without.metrics.lots_completed,
            with.metrics.lots_completed
        );

        // Only `default` is known to close the cycle inside 20k ticks;
        // starvation_biased keeps the tools drained enough that it does not,
        // at least on this map. Asserting it for both would be asserting
        // something that is not true. This half matters because "no deadlock
        // with buffers" would also pass with a detector that never fires.
        if policy == "default" {
            assert!(
                without.metrics.resource_deadlock_events > 0,
                "removing the buffers should reopen the deadlock under default, \
                 but the detector saw none -- detector or fallback is broken"
            );
        }
    }
}
