//! Tick-rate benchmark. Reports throughput per window, so a cost that grows
//! with the length of the run shows up as a falling rate rather than an
//! average that hides it.
use ohtsim::{load_map, load_policy, load_scenario, World};
use std::fs;
use std::time::Instant;

fn main() {
    let total: u64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(400_000);
    let window = total / 8;
    let map = load_map(&fs::read_to_string("maps/demo_loop.json").unwrap()).unwrap();
    let scen = load_scenario(&fs::read_to_string("scenarios/baseline.json").unwrap(), &map.grid).unwrap();
    let pol = load_policy(&fs::read_to_string("policies/default.json").unwrap()).unwrap();
    let mut w = World::new(map, scen, pol);

    println!("{:>10} {:>12} {:>10}", "ticks", "ticks/sec", "jobs");
    let start = Instant::now();
    for k in 0..8 {
        let t0 = Instant::now();
        for _ in 0..window {
            w.tick();
        }
        println!("{:>10} {:>12.0} {:>10}",
                 (k + 1) * window,
                 window as f64 / t0.elapsed().as_secs_f64(),
                 w.jobs_len());
    }
    println!("\noverall {:.0} ticks/sec, completed {}",
             total as f64 / start.elapsed().as_secs_f64(), w.metrics.lots_completed);
}
