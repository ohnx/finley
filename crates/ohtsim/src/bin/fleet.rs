//! Throughput against fleet size: where does transport start to bind?
//!
//! The policy weights can only change throughput in a regime where transport
//! is the constraint. At the shipped 30 vehicles the fab map is neither
//! transport-limited nor tool-limited -- the WIP cap sets the rate, and no
//! dispatch order can argue with release control, which is why the sensitivity
//! sweep comes back flat there. This finds the fleet sizes where it does bind,
//! so a sweep can be run somewhere it might mean something.
//!
//! Read the two utilisation columns together. Vehicles near 100% with litho
//! well below it means the fleet is the bottleneck; the two converging means
//! neither is, and the cap is.
//!
//!   cargo run --release --bin fleet [ticks]
//!
//! `OHT_MAP` and `OHT_SCENARIO` override what it scans.

use ohtsim::{load_map, load_policy, load_scenario, World};
use std::fs;

fn env_or(key: &str, fallback: &str) -> String {
    std::env::var(key).unwrap_or_else(|_| fallback.to_string())
}

fn main() {
    let ticks: u64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(300_000);
    let map_path = env_or("OHT_MAP", "maps/fab.json");
    let scen_path = env_or("OHT_SCENARIO", "scenarios/fab.json");

    println!("{map_path} + {scen_path}, {ticks} ticks");
    println!(
        "{:>4} {:>8} {:>8} {:>8} {:>8} {:>6}",
        "veh", "per1k", "p95", "veh%", "litho%", "dead"
    );
    for n in [10usize, 14, 18, 22, 26, 30] {
        let map = load_map(&fs::read_to_string(&map_path).unwrap()).unwrap();
        let mut scen = load_scenario(&fs::read_to_string(&scen_path).unwrap(), &map.grid).unwrap();
        scen.vehicles = n;
        // Start cells are listed per vehicle, so a shorter fleet needs a
        // shorter list -- otherwise the extras are placed and then ignored.
        scen.vehicle_start_cells.truncate(n);
        let pol = load_policy(&fs::read_to_string("policies/default.json").unwrap()).unwrap();
        let mut w = World::new(map, scen, pol);
        w.run(ticks);

        // The bottleneck tool, by name, so the comparison survives a map edit.
        let litho: Vec<usize> = w
            .machines
            .iter()
            .enumerate()
            .filter(|(_, m)| m.kind == "litho")
            .map(|(i, _)| i)
            .collect();
        let busy = if litho.is_empty() {
            f64::NAN
        } else {
            let idle: u64 = litho.iter().map(|&i| w.metrics.machine_idle_ticks[i]).sum();
            100.0 - idle as f64 / (litho.len() as u64 * ticks) as f64 * 100.0
        };

        println!(
            "{:>4} {:>8.2} {:>8} {:>7.1}% {:>7.1}% {:>6}",
            n,
            w.metrics.throughput_per_1k_ticks(),
            w.metrics.p95_cycle_time(),
            w.metrics.utilisation() * 100.0,
            busy,
            w.metrics.resource_deadlock_events
        );
    }
}
