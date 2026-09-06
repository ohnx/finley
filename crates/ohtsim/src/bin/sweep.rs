//! One-at-a-time sensitivity sweep over the policy knobs.
//!
//! The design premise is that the interesting behaviour lives in the *basis* of
//! weighted criteria rather than in a menu of presets. That is only true if the
//! weights actually move the fab, so this measures how far each one does, from
//! the shipped default, holding everything else fixed.
//!
//! Averaged over several seeds. The fab is chaotic -- one blocked vehicle
//! changes who gets the next job -- so a single run's difference between two
//! settings is mostly the job stream, not the setting. The spread across seeds
//! is printed so a change can be read against the noise it has to beat.
//!
//!   cargo run --release --bin sweep [ticks] [seeds]

use ohtsim::policy::{IdleMode, Policy};
use ohtsim::{load_map, load_policy, load_scenario, World};
use std::fs;

struct Run {
    throughput: f64,
    p95: f64,
    mean_cycle: f64,
    util: f64,
    deadlocks: u64,
}

fn run(policy: &Policy, ticks: u64, seed: u64) -> Run {
    let map = load_map(&fs::read_to_string("maps/demo_loop.json").unwrap()).unwrap();
    let mut scen =
        load_scenario(&fs::read_to_string("scenarios/baseline.json").unwrap(), &map.grid).unwrap();
    scen.seed = seed;
    // Optional override, so the same sweep can be run at a different fleet size:
    // weights can only matter when dispatch has more than one way to assign the
    // work, and how often that happens depends on how much slack the fleet has.
    if let Some(n) = std::env::var("OHT_VEHICLES").ok().and_then(|v| v.parse().ok()) {
        scen.vehicles = n;
        scen.vehicle_start_cells.truncate(n);
    }
    let mut w = World::new(map, scen, policy.clone());
    w.run(ticks);
    Run {
        throughput: w.metrics.throughput_per_1k_ticks(),
        p95: w.metrics.p95_cycle_time() as f64,
        mean_cycle: w.metrics.mean_cycle_time(),
        util: w.metrics.utilisation() * 100.0,
        deadlocks: w.metrics.resource_deadlock_events,
    }
}

/// Mean of `f` across seeds, plus the half-range, as a noise yardstick.
fn measure(policy: &Policy, ticks: u64, seeds: &[u64]) -> (Run, f64) {
    let runs: Vec<Run> = seeds.iter().map(|&s| run(policy, ticks, s)).collect();
    let n = runs.len() as f64;
    let mean = Run {
        throughput: runs.iter().map(|r| r.throughput).sum::<f64>() / n,
        p95: runs.iter().map(|r| r.p95).sum::<f64>() / n,
        mean_cycle: runs.iter().map(|r| r.mean_cycle).sum::<f64>() / n,
        util: runs.iter().map(|r| r.util).sum::<f64>() / n,
        deadlocks: runs.iter().map(|r| r.deadlocks).sum(),
    };
    let hi = runs.iter().map(|r| r.throughput).fold(f64::MIN, f64::max);
    let lo = runs.iter().map(|r| r.throughput).fold(f64::MAX, f64::min);
    (mean, (hi - lo) / 2.0)
}

/// One value of a knob: how to label it, and how to apply it to a policy.
type Setting = (String, Box<dyn Fn(&mut Policy)>);
/// A knob and every value to try for it.
type Knob = (&'static str, Vec<Setting>);

fn scale(values: &[f32], set: fn(&mut Policy, f32)) -> Vec<Setting> {
    values
        .iter()
        .map(|&v| {
            let label = format!("{v}");
            let f: Box<dyn Fn(&mut Policy)> = Box::new(move |p: &mut Policy| set(p, v));
            (label, f)
        })
        .collect()
}

fn knobs() -> Vec<Knob> {
    let w = [0.0f32, 0.5, 1.0, 2.0, 4.0, 16.0];
    vec![
        ("dispatch.travel_to_pickup", scale(&w, |p, v| p.profiles[0].dispatch.travel_to_pickup = v)),
        ("dispatch.lot_wait", scale(&[0.0, 0.05, 0.2, 1.0, 4.0], |p, v| p.profiles[0].dispatch.lot_wait = v)),
        ("dispatch.lot_priority", scale(&[0.0, 10.0, 50.0, 200.0], |p, v| p.profiles[0].dispatch.lot_priority = v)),
        ("dispatch.dest_starvation", scale(&[0.0, 0.02, 1.0, 10.0, 50.0], |p, v| p.profiles[0].dispatch.dest_starvation = v)),
        ("dispatch.dest_queue", scale(&[0.0, 1.0, 4.0, 16.0, 64.0], |p, v| p.profiles[0].dispatch.dest_queue = v)),
        ("dispatch.dest_congestion", scale(&w, |p, v| p.profiles[0].dispatch.dest_congestion = v)),
        ("dispatch.steps_remaining", scale(&[-4.0, -1.0, 0.0, 1.0, 4.0], |p, v| p.profiles[0].dispatch.steps_remaining = v)),
        ("route.curve", scale(&[0.0, 1.0, 2.0, 4.0, 8.0], |p, v| p.profiles[0].route.curve = v)),
        ("route.congestion", scale(&[0.0, 1.0, 3.0, 8.0, 24.0], |p, v| p.profiles[0].route.congestion = v)),
        ("congestion_decay", scale(&[0.8, 0.95, 0.98, 0.995], |p, v| p.congestion_decay = v)),
        ("idle.dwell_before_move", scale(&[0.0, 5.0, 20.0, 100.0], |p, v| p.idle.dwell_before_move = v as u32)),
        (
            "idle.mode",
            [IdleMode::StayPut, IdleMode::NearestPark, IdleMode::Preposition]
                .into_iter()
                .map(|m| {
                    let label = format!("{m:?}");
                    let f: Box<dyn Fn(&mut Policy)> = Box::new(move |p: &mut Policy| p.idle.mode = m);
                    (label, f)
                })
                .collect(),
        ),
    ]
}

fn main() {
    let ticks: u64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(200_000);
    let n_seeds: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(4);
    let seeds: Vec<u64> = (0..n_seeds as u64).map(|i| 20260903 + i * 7919).collect();

    let base = load_policy(&fs::read_to_string("policies/default.json").unwrap()).unwrap();
    let (b, noise) = measure(&base, ticks, &seeds);
    println!("baseline (policies/default.json), {ticks} ticks x {n_seeds} seeds");
    println!(
        "  throughput {:.2} /1k   p95 {:.0}   mean cycle {:.0}   util {:.1}%",
        b.throughput, b.p95, b.mean_cycle, b.util
    );
    println!("  seed-to-seed spread in throughput: +/- {noise:.2}  <- anything smaller is noise\n");

    println!(
        "{:<26} {:>8} {:>9} {:>7} {:>8} {:>7} {:>6}",
        "knob = value", "thru/1k", "vs base", "p95", "cycle", "util%", "dead"
    );
    for (name, settings) in knobs() {
        println!("{}", "-".repeat(76));
        let mut best = f64::MIN;
        let mut worst = f64::MAX;
        for (label, apply) in settings {
            let mut p = base.clone();
            apply(&mut p);
            let (r, _) = measure(&p, ticks, &seeds);
            best = best.max(r.throughput);
            worst = worst.min(r.throughput);
            let delta = r.throughput - b.throughput;
            let mark = if delta.abs() > noise * 2.0 { "*" } else { " " };
            println!(
                "{:<26} {:>8.2} {:>8.2}{} {:>7.0} {:>8.0} {:>7.1} {:>6}",
                format!("{name} = {label}"),
                r.throughput,
                delta,
                mark,
                r.p95,
                r.mean_cycle,
                r.util,
                r.deadlocks
            );
        }
        println!("{:<26} range {:.2}", format!("  [{name}]"), best - worst);
    }
    println!("\n* = change larger than twice the seed-to-seed spread");
}
