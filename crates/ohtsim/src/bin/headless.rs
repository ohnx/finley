//! Headless runner.
//!
//!   cargo run --release --bin headless -- maps/demo_loop.json \
//!       scenarios/baseline.json policies/default.json 20000
//!
//! With two policy files, runs both on the identical job stream and prints the
//! comparison. Same seed, same arrivals — otherwise the comparison is noise.

use std::env;
use std::fs;

use ohtsim::{load_map, load_policy, load_scenario, World};

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 4 {
        eprintln!(
            "usage: headless <map.json> <scenario.json> <policy.json> [ticks] [policy_b.json]"
        );
        std::process::exit(2);
    }

    let map_src = read(&args[1]);
    let scen_src = read(&args[2]);
    let pol_src = read(&args[3]);
    let ticks: u64 = args
        .get(4)
        .and_then(|s| s.parse().ok())
        .unwrap_or(20_000);

    let map = load_map(&map_src).unwrap_or_else(|e| fail("map", &e));
    let scenario = load_scenario(&scen_src, &map.grid).unwrap_or_else(|e| fail("scenario", &e));
    let policy = load_policy(&pol_src).unwrap_or_else(|e| fail("policy", &e));

    // Validate before simulating. A map that strands vehicles produces a run
    // that looks merely disappointing rather than broken, and diagnosing that
    // from metrics alone is miserable.
    let problems = ohtsim::validate(&map);
    if !problems.is_empty() {
        eprintln!("map validation failed ({} problems):", problems.len());
        for p in &problems {
            eprintln!("  - {}", p);
        }
        std::process::exit(1);
    }

    println!("map        {} ({}x{})", map.name, map.grid.w, map.grid.h);
    println!("vehicles   {}", scenario.vehicles);
    println!("seed       {}", scenario.seed);
    println!();

    let mut w = World::new(map.clone(), scenario.clone(), policy);
    w.run(ticks);
    println!("--- policy A: {} ---", args[3]);
    println!("{}", w.metrics.report());

    if let Some(pb) = args.get(5) {
        let pol_b = load_policy(&read(pb)).unwrap_or_else(|e| fail("policy B", &e));
        let mut w2 = World::new(map, scenario, pol_b);
        w2.run(ticks);
        println!("--- policy B: {} ---", pb);
        println!("{}", w2.metrics.report());

        println!("--- delta (B - A) ---");
        println!(
            "throughput   {:+.2} lots / 1000 ticks",
            w2.metrics.throughput_per_1k_ticks() - w.metrics.throughput_per_1k_ticks()
        );
        println!(
            "mean cycle   {:+.1} ticks",
            w2.metrics.mean_cycle_time() - w.metrics.mean_cycle_time()
        );
        println!(
            "p95 cycle    {:+} ticks",
            w2.metrics.p95_cycle_time() as i64 - w.metrics.p95_cycle_time() as i64
        );
        println!(
            "utilisation  {:+.1}%",
            (w2.metrics.utilisation() - w.metrics.utilisation()) * 100.0
        );
    }
}

fn read(path: &str) -> String {
    fs::read_to_string(path).unwrap_or_else(|e| {
        eprintln!("cannot read {}: {}", path, e);
        std::process::exit(1);
    })
}

fn fail(what: &str, e: &str) -> ! {
    eprintln!("{} config error: {}", what, e);
    std::process::exit(1);
}
