//! Divergence tracer. Emits one line of world state per tick so a run can be
//! diffed against `reference/reference_sim.py`, which is the behavioural
//! ground truth for the port.
//!
//!   cargo run --release --bin trace -- maps/demo_loop.json \
//!       scenarios/baseline.json policies/starvation_biased.json 2000

use std::env;
use std::fs;

use ohtsim::model::VehState;
use ohtsim::{load_map, load_policy, load_scenario, World};

fn main() {
    let args: Vec<String> = env::args().collect();
    let map = load_map(&fs::read_to_string(&args[1]).unwrap()).unwrap();
    let scen = load_scenario(&fs::read_to_string(&args[2]).unwrap(), &map.grid).unwrap();
    let pol = load_policy(&fs::read_to_string(&args[3]).unwrap()).unwrap();
    let ticks: u64 = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(2000);

    let mut w = World::new(map, scen, pol);
    for _ in 0..ticks {
        w.tick();
        let veh: Vec<String> = w
            .vehicles
            .iter()
            .map(|v| {
                let s = match v.state {
                    VehState::Idle => "i",
                    VehState::ToPickup(_) => "p",
                    VehState::Loading(_) => "L",
                    VehState::ToDropoff(_) => "d",
                    VehState::Unloading(_) => "U",
                    VehState::Repositioning => "r",
                };
                format!("{}{}:{}:{}", s, v.cell, v.ready_in, v.route.len())
            })
            .collect();
        println!(
            "{} prof={} pend={} lots={} done={} | {}",
            w.tick_count,
            w.active_profile,
            w.pending.len(),
            w.lots.len(),
            w.metrics.lots_completed,
            veh.join(" ")
        );
    }
}
