//! Configuration loading.
//!
//! Four documents, not one: machine library and map (what the fab is),
//! scenario (what work arrives), policy (how it is dispatched). Splitting them
//! is what lets you run one map against many scenarios and one scenario
//! against many policies, which is the whole point of the exercise.

use crate::geom::{CellId, Grid};
use crate::json::{parse, Json};
use crate::model::{Machine, Port, PortKind};
use crate::policy::{
    DispatchWeights, IdleMode, IdlePolicy, Kinematics, Policy, Profile, RouteWeights, Trigger,
};

#[derive(Clone, Debug)]
pub struct MapConfig {
    pub name: String,
    pub grid: Grid,
    pub machines: Vec<Machine>,
    pub parking: Vec<CellId>,
}

#[derive(Clone, Debug)]
pub struct RecipeSpec {
    pub steps: Vec<String>,
    pub weight: f32,
}

#[derive(Clone, Debug)]
pub struct ScenarioConfig {
    pub seed: u64,
    pub vehicles: usize,
    pub vehicle_start_cells: Vec<CellId>,
    /// Expected lot releases per 1000 ticks.
    pub arrival_per_1000: f32,
    /// Share of lots flagged hot.
    pub hot_fraction: f32,
    /// Release control: the most lots allowed in the fab at once. A lot that
    /// would arrive while the fab is at its cap is simply not released -- it
    /// waits outside the line, which is what a real fab does.
    ///
    /// This is not a tuning nicety, it is what keeps the fab from deadlocking.
    /// Every place a lot can rest is finite, and if enough lots are admitted to
    /// fill all of them, the reentrant flow closes a cycle no amount of
    /// buffering can open. `0` means no cap, and means the fab will eventually
    /// stop.
    pub wip_cap: usize,
    pub recipes: Vec<RecipeSpec>,
}

impl Default for ScenarioConfig {
    fn default() -> Self {
        ScenarioConfig {
            seed: 1,
            vehicles: 8,
            vehicle_start_cells: Vec::new(),
            arrival_per_1000: 40.0,
            wip_cap: 0,
            hot_fraction: 0.05,
            recipes: Vec::new(),
        }
    }
}

pub fn load_map(src: &str) -> Result<MapConfig, String> {
    let j = parse(src)?;
    let w = j.usize_at("width").ok_or("map: missing width")?;
    let h = j.usize_at("height").ok_or("map: missing height")?;
    let name = j.str_at("name").unwrap_or_else(|| "unnamed".to_string());

    let mut grid = Grid::new(w, h);
    let rows = j
        .get("tracks")
        .and_then(|v| v.arr())
        .ok_or("map: missing tracks")?;
    if rows.len() != h {
        return Err(format!("map: tracks has {} rows, expected {}", rows.len(), h));
    }
    for (y, row) in rows.iter().enumerate() {
        let cells = row.arr().ok_or("map: track row is not an array")?;
        if cells.len() != w {
            return Err(format!("map: track row {} has {} cells, expected {}", y, cells.len(), w));
        }
        for (x, c) in cells.iter().enumerate() {
            let v = c.num().ok_or("map: track cell is not a number")? as i64;
            if !(0..=15).contains(&v) {
                return Err(format!("map: track value {} at ({},{}) out of range 0..15", v, x, y));
            }
            grid.track[y * w + x] = v as u8;
        }
    }

    let mut parking = Vec::new();
    if let Some(arr) = j.get("parking").and_then(|v| v.arr()) {
        for p in arr {
            let (x, y) = p.pair().ok_or("map: parking entry must be [x,y]")?;
            parking.push(grid.idx(x, y));
        }
    }

    let mut machines = Vec::new();
    let ms = j
        .get("machines")
        .and_then(|v| v.arr())
        .ok_or("map: missing machines")?;
    for (i, m) in ms.iter().enumerate() {
        let mname = m.str_at("name").unwrap_or_else(|| format!("m{}", i));
        let kind = m.str_at("kind").ok_or("machine: missing kind")?;
        let mut ports = Vec::new();
        if let Some(ps) = m.get("ports").and_then(|v| v.arr()) {
            for p in ps {
                let k = p.str_at("kind").unwrap_or_else(|| "in".to_string());
                let kind = match k.as_str() {
                    "in" | "input" => PortKind::Input,
                    "out" | "output" => PortKind::Output,
                    other => return Err(format!("machine {}: bad port kind `{}`", mname, other)),
                };
                let (px, py) = p
                    .get("cell")
                    .and_then(|c| c.pair())
                    .ok_or("port: missing cell [x,y]")?;
                let cell = grid.idx(px, py);
                if !grid.has_track(cell) {
                    return Err(format!(
                        "machine {}: port cell ({},{}) has no track above it",
                        mname, px, py
                    ));
                }
                ports.push(Port {
                    kind,
                    cell,
                    lot: None,
                    reserved_by: None,
                });
            }
        }
        machines.push(Machine {
            id: i,
            name: mname,
            kind,
            x: m.usize_at("x").unwrap_or(0),
            y: m.usize_at("y").unwrap_or(0),
            w: m.usize_at("w").unwrap_or(1),
            h: m.usize_at("h").unwrap_or(1),
            process_ticks: m.u32_at("process_ticks").unwrap_or(60),
            capacity: m.usize_at("capacity").unwrap_or(1),
            ports,
            in_process: Vec::new(),
            idle_ticks: 0,
            starvation: 0.0,
        });
    }

    Ok(MapConfig {
        name,
        grid,
        machines,
        parking,
    })
}

pub fn load_scenario(src: &str, grid: &Grid) -> Result<ScenarioConfig, String> {
    let j = parse(src)?;
    let mut s = ScenarioConfig::default();
    if let Some(v) = j.get("seed").and_then(|v| v.num()) {
        s.seed = v as u64;
    }
    if let Some(v) = j.usize_at("vehicles") {
        s.vehicles = v;
    }
    if let Some(v) = j.usize_at("wip_cap") {
        s.wip_cap = v;
    }
    if let Some(v) = j.f32_at("arrival_per_1000") {
        s.arrival_per_1000 = v;
    }
    if let Some(v) = j.f32_at("hot_fraction") {
        s.hot_fraction = v;
    }
    if let Some(arr) = j.get("vehicle_start_cells").and_then(|v| v.arr()) {
        for p in arr {
            let (x, y) = p.pair().ok_or("scenario: start cell must be [x,y]")?;
            s.vehicle_start_cells.push(grid.idx(x, y));
        }
    }
    if let Some(arr) = j.get("recipes").and_then(|v| v.arr()) {
        for r in arr {
            let steps: Vec<String> = r
                .get("steps")
                .and_then(|v| v.arr())
                .ok_or("recipe: missing steps")?
                .iter()
                .filter_map(|s| s.str().map(|x| x.to_string()))
                .collect();
            let weight = r.f32_at("weight").unwrap_or(1.0);
            s.recipes.push(RecipeSpec { steps, weight });
        }
    }
    Ok(s)
}

fn dispatch_from(j: &Json) -> DispatchWeights {
    let d = DispatchWeights::default();
    DispatchWeights {
        travel_to_pickup: j.f32_at("travel_to_pickup").unwrap_or(d.travel_to_pickup),
        lot_wait: j.f32_at("lot_wait").unwrap_or(d.lot_wait),
        lot_priority: j.f32_at("lot_priority").unwrap_or(d.lot_priority),
        dest_starvation: j.f32_at("dest_starvation").unwrap_or(d.dest_starvation),
        dest_queue: j.f32_at("dest_queue").unwrap_or(d.dest_queue),
        dest_congestion: j.f32_at("dest_congestion").unwrap_or(d.dest_congestion),
        steps_remaining: j.f32_at("steps_remaining").unwrap_or(d.steps_remaining),
    }
}

fn route_from(j: &Json) -> RouteWeights {
    let d = RouteWeights::default();
    RouteWeights {
        length: j.f32_at("length").unwrap_or(d.length),
        curve: j.f32_at("curve").unwrap_or(d.curve),
        congestion: j.f32_at("congestion").unwrap_or(d.congestion),
    }
}

fn trigger_from(j: &Json) -> Result<Trigger, String> {
    let t = j.str_at("type").unwrap_or_else(|| "always".to_string());
    match t.as_str() {
        "always" => Ok(Trigger::Always),
        "queue_depth_above" => Ok(Trigger::QueueDepthAbove {
            kind: j.str_at("kind").ok_or("trigger: missing kind")?,
            n: j.usize_at("n").unwrap_or(3),
        }),
        "starvation_above" => Ok(Trigger::StarvationAbove {
            kind: j.str_at("kind").ok_or("trigger: missing kind")?,
            ticks: j.get("ticks").and_then(|v| v.num()).unwrap_or(500.0) as u64,
        }),
        "backlog_above" => Ok(Trigger::BacklogAbove {
            n: j.usize_at("n").unwrap_or(5),
        }),
        other => Err(format!("unknown trigger `{}`", other)),
    }
}

pub fn load_policy(src: &str) -> Result<Policy, String> {
    let j = parse(src)?;
    let mut p = Policy::default();

    if let Some(v) = j.f32_at("congestion_decay") {
        p.congestion_decay = v;
    }
    if let Some(v) = j.u32_at("stuck_threshold") {
        p.stuck_threshold = v;
    }
    if let Some(k) = j.get("kinematics") {
        let d = Kinematics::default();
        p.kinematics = Kinematics {
            straight_ticks: k.u32_at("straight_ticks").unwrap_or(d.straight_ticks),
            curve_ticks: k.u32_at("curve_ticks").unwrap_or(d.curve_ticks),
            hoist_ticks: k.u32_at("hoist_ticks").unwrap_or(d.hoist_ticks),
        };
    }
    if let Some(i) = j.get("idle") {
        let mode = match i.str_at("mode").unwrap_or_else(|| "nearest_park".to_string()).as_str() {
            "stay_put" => IdleMode::StayPut,
            "nearest_park" => IdleMode::NearestPark,
            "preposition" => IdleMode::Preposition,
            other => return Err(format!("unknown idle mode `{}`", other)),
        };
        p.idle = IdlePolicy {
            mode,
            dwell_before_move: i.u32_at("dwell_before_move").unwrap_or(5),
        };
    }
    if let Some(arr) = j.get("profiles").and_then(|v| v.arr()) {
        let mut profiles = Vec::new();
        for pr in arr {
            profiles.push(Profile {
                name: pr.str_at("name").unwrap_or_else(|| "profile".to_string()),
                trigger: match pr.get("trigger") {
                    Some(t) => trigger_from(t)?,
                    None => Trigger::Always,
                },
                dispatch: match pr.get("dispatch") {
                    Some(d) => dispatch_from(d),
                    None => DispatchWeights::default(),
                },
                route: match pr.get("route") {
                    Some(r) => route_from(r),
                    None => RouteWeights::default(),
                },
            });
        }
        if !profiles.is_empty() {
            // Conditional profiles are checked in order and the first match
            // wins, so `always` must sit last or it shadows everything.
            profiles.sort_by_key(|pr| matches!(pr.trigger, Trigger::Always));
            p.profiles = profiles;
        }
    }
    Ok(p)
}
