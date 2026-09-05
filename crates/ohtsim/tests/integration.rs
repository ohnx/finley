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
/// It cannot be driven to zero: when every spur is genuinely taken a vehicle
/// has nowhere to go and stops where it stands. The threshold guards the
/// regression that mattered -- prepositioning collapsing its whole target set
/// to one arbitrary cell, which stranded vehicles on the loop for 43% of the
/// run.
#[test]
fn idle_vehicles_do_not_camp_on_the_main_line() {
    for (policy, limit) in [("default", 900), ("starvation_biased", 700)] {
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
