//! Browser shim for the ohtsim core.
//!
//! Deliberately a raw C ABI with no wasm-bindgen. The core has no dependencies
//! and this crate keeps it that way: the whole bundle is the simulation and
//! nothing else, and there is no JS toolchain to install or keep current.
//!
//! The contract with `web/app.js`:
//!
//! * **Static map geometry never crosses this boundary.** JS already fetches
//!   the map JSON to construct the world, so it reads track bits, machine
//!   rectangles and port cells straight out of that. Serialising them here
//!   would be a second source of truth for no gain.
//! * **Per-frame data is read as typed arrays over wasm linear memory.** The
//!   accessors return pointers into flat `Vec`s that the core refills in place.
//!   Nothing is serialised per frame -- doing that would eat most of the reason
//!   for putting the simulation in Rust at all.
//!
//! Two rules JS must follow, both of which `web/app.js` documents at its call
//! sites: re-derive typed-array views from `memory.buffer` every frame, because
//! growing wasm memory detaches the old ones; and treat every pointer as valid
//! only until the next call that mutates the world.

use std::alloc::{alloc, dealloc, Layout};
use std::cell::RefCell;

use ohtsim::model::{LotState, VehState};
use ohtsim::world::Snapshot;
use ohtsim::{load_map, load_policy, load_scenario, World};

thread_local! {
    /// Last failure, so JS can show why `oht_new` returned null. Constructing a
    /// world is the only fallible call here, and a bad config is the common
    /// case: it is worth a real message rather than a null and a shrug.
    static LAST_ERROR: RefCell<String> = const { RefCell::new(String::new()) };
}

fn set_error(msg: String) {
    LAST_ERROR.with(|e| *e.borrow_mut() = msg);
}

/// Number of `f64` slots in the metrics block. Keep in sync with `METRIC_*`
/// below and with the reader in `web/app.js`.
pub const METRIC_COUNT: usize = 16;

/// Where a lot currently sits. `LOT_AT_PORT` carries a machine and a port in
/// `lot_a`/`lot_b`; the other two carry a machine or a vehicle in `lot_a`.
pub const LOT_AT_PORT: u8 = 0;
pub const LOT_IN_TRANSIT: u8 = 1;
pub const LOT_PROCESSING: u8 = 2;
/// Finished processing but still inside the tool, because every out-port is
/// full. The core models this as `Processing` with nothing left on the clock;
/// it is a distinct thing to watch, because it is where backpressure starts --
/// the tool cannot take new work until this lot gets out.
pub const LOT_BLOCKED: u8 = 3;

/// Per-frame view of the lots still in the fab. Done lots are dropped: the
/// world keeps every lot it ever made, but only the live ones are worth
/// drawing, and there are around twenty at a time on the demo map.
#[derive(Default)]
struct Lots {
    id: Vec<u32>,
    recipe: Vec<u8>,
    step: Vec<u8>,
    steps_total: Vec<u8>,
    place: Vec<u8>,
    a: Vec<u16>,
    b: Vec<u16>,
    wait: Vec<u32>,
    priority: Vec<f32>,
}

/// Per-frame view of what each vehicle is working on. `-1` means none.
#[derive(Default)]
struct Targets {
    machine: Vec<i16>,
    port: Vec<i16>,
    lot: Vec<i32>,
}

pub struct Sim {
    world: World,
    snap: Snapshot,
    metrics: [f64; METRIC_COUNT],
    lots: Lots,
    targets: Targets,
    /// Lot sitting on each port, `-1` for empty, flattened machine-major in the
    /// same order the map JSON lists them so JS can index it directly.
    port_lot: Vec<i32>,
    /// Cumulative ticks each machine spent with nothing in process. The UI
    /// turns this into a utilisation percentage.
    machine_idle: Vec<u32>,
    /// Completed lots' cycle times, as f64 to spare JS the BigInt dance.
    cycles: Vec<f64>,
}

impl Sim {
    fn refresh(&mut self) {
        self.world.snapshot_into(&mut self.snap);
        self.refresh_lots();
        self.refresh_targets();
        self.refresh_ports();

        self.machine_idle.clear();
        self.machine_idle
            .extend(self.world.machines.iter().map(|m| m.idle_ticks as u32));

        // Rebuilt only when a lot has completed. Copying the whole series
        // every frame is work proportional to the length of the run, for a
        // histogram that changes a few times a minute.
        if self.cycles.len() != self.world.metrics.cycle_times.len() {
            self.cycles.clear();
            self.cycles
                .extend(self.world.metrics.cycle_times.iter().map(|&c| c as f64));
        }

        let m = &self.world.metrics;
        self.metrics = [
            self.world.tick_count as f64,
            m.lots_created as f64,
            m.lots_completed as f64,
            m.throughput_per_1k_ticks(),
            m.mean_cycle_time(),
            m.p95_cycle_time() as f64,
            m.utilisation(),
            m.mean_backlog(),
            m.deadlock_events as f64,
            m.stuck_vehicle_events as f64,
            m.cycles_rotated as f64,
            self.world.pending.len() as f64,
            self.world
                .vehicles
                .iter()
                .filter(|v| !v.is_idle())
                .count() as f64,
            self.lots.id.len() as f64,                       // WIP now
            self.world.scenario.wip_cap as f64,              // release cap
            m.arrivals_deferred as f64,
        ];
    }

    fn refresh_lots(&mut self) {
        let l = &mut self.lots;
        l.id.clear();
        l.recipe.clear();
        l.step.clear();
        l.steps_total.clear();
        l.place.clear();
        l.a.clear();
        l.b.clear();
        l.wait.clear();
        l.priority.clear();

        let now = self.world.tick_count;
        for &lot_id in self.world.active_lots() {
            let lot = &self.world.lots[lot_id];
            let (place, a, b) = match lot.state {
                LotState::Done => continue, // retired between refreshes
                LotState::AtPort(m, p) => (LOT_AT_PORT, m as u16, p as u16),
                LotState::InTransit(v) => (LOT_IN_TRANSIT, v as u16, 0),
                LotState::Processing(m) => {
                    // Zero ticks left on the clock means the recipe step is
                    // finished and the lot is only still here because it has
                    // nowhere to go.
                    let done = self.world.machines[m]
                        .in_process
                        .iter()
                        .find(|(id, _)| *id == lot.id)
                        .is_some_and(|(_, remaining)| *remaining == 0);
                    (if done { LOT_BLOCKED } else { LOT_PROCESSING }, m as u16, 0)
                }
            };
            l.id.push(lot.id as u32);
            l.recipe.push(lot.recipe_id as u8);
            l.step.push(lot.step.min(255) as u8);
            l.steps_total.push(lot.recipe.len().min(255) as u8);
            l.place.push(place);
            l.a.push(a);
            l.b.push(b);
            // A lot in process or in transit is not waiting; the clock only
            // means something for one parked on a port.
            // Only meaningful for a lot parked on a port. `waiting_since` is
            // not reset when a lot enters a tool, so for one blocked inside it
            // would read as time since it arrived at the *input* port, whole
            // processing run included -- a number that looks like a wait and
            // is not one.
            l.wait.push(if place == LOT_AT_PORT {
                lot.wait_ticks(now).min(u32::MAX as u64) as u32
            } else {
                0
            });
            l.priority.push(lot.priority);
        }
    }

    fn refresh_targets(&mut self) {
        let t = &mut self.targets;
        t.machine.clear();
        t.port.clear();
        t.lot.clear();
        for v in &self.world.vehicles {
            // Which port this vehicle is driving at: the pickup while it is
            // still empty, the destination once it is loaded.
            let job = match v.state {
                VehState::ToPickup(j) | VehState::Loading(j) => Some((j, false)),
                VehState::ToDropoff(j) | VehState::Unloading(j) => Some((j, true)),
                VehState::Idle | VehState::Repositioning => None,
            };
            let dest = job.and_then(|(j, loaded)| {
                let job = &self.world.jobs[j];
                if loaded {
                    job.to
                } else {
                    Some(job.from)
                }
            });
            match dest {
                Some((m, p)) => {
                    t.machine.push(m as i16);
                    t.port.push(p as i16);
                }
                None => {
                    t.machine.push(-1);
                    t.port.push(-1);
                }
            }
            t.lot.push(v.carrying.map(|l| l as i32).unwrap_or(-1));
        }
    }

    fn refresh_ports(&mut self) {
        self.port_lot.clear();
        for m in &self.world.machines {
            for p in &m.ports {
                self.port_lot.push(p.lot.map(|l| l as i32).unwrap_or(-1));
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Memory, for handing config text in
// ---------------------------------------------------------------------------

/// Allocate `len` bytes for JS to write into. Pair every call with
/// `oht_free_buf` using the same length.
///
/// # Safety
/// Caller must not request a zero-length or absurdly large block.
#[no_mangle]
pub unsafe extern "C" fn oht_alloc(len: usize) -> *mut u8 {
    if len == 0 {
        return std::ptr::null_mut();
    }
    match Layout::from_size_align(len, 1) {
        Ok(l) => alloc(l),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
/// `ptr` must come from `oht_alloc` with the same `len`, and must not have been
/// freed already.
#[no_mangle]
pub unsafe extern "C" fn oht_free_buf(ptr: *mut u8, len: usize) {
    if ptr.is_null() || len == 0 {
        return;
    }
    if let Ok(l) = Layout::from_size_align(len, 1) {
        dealloc(ptr, l);
    }
}

/// # Safety
/// `ptr` must point to `len` initialised bytes.
unsafe fn as_str<'a>(ptr: *const u8, len: usize) -> Result<&'a str, String> {
    if ptr.is_null() {
        return Err("null config pointer".to_string());
    }
    std::str::from_utf8(std::slice::from_raw_parts(ptr, len))
        .map_err(|_| "config is not valid UTF-8".to_string())
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

/// Build a world from map, scenario and policy JSON. Returns null on failure;
/// read `oht_error_ptr`/`oht_error_len` for why.
///
/// The map is validated first, so a map that would strand vehicles is rejected
/// here rather than silently producing a fab that does nothing.
///
/// # Safety
/// All three pointers must point to that many initialised bytes.
#[no_mangle]
pub unsafe extern "C" fn oht_new(
    map_ptr: *const u8,
    map_len: usize,
    scen_ptr: *const u8,
    scen_len: usize,
    pol_ptr: *const u8,
    pol_len: usize,
) -> *mut Sim {
    let build = || -> Result<Sim, String> {
        let map = load_map(as_str(map_ptr, map_len)?)?;
        let problems = ohtsim::validate(&map);
        if !problems.is_empty() {
            let list: Vec<String> = problems.iter().map(|p| p.to_string()).collect();
            return Err(format!("map validation failed: {}", list.join("; ")));
        }
        let scenario = load_scenario(as_str(scen_ptr, scen_len)?, &map.grid)?;
        let policy = load_policy(as_str(pol_ptr, pol_len)?)?;
        Ok(Sim {
            world: World::new(map, scenario, policy),
            snap: Snapshot::default(),
            metrics: [0.0; METRIC_COUNT],
            lots: Lots::default(),
            targets: Targets::default(),
            port_lot: Vec::new(),
            machine_idle: Vec::new(),
            cycles: Vec::new(),
        })
    };
    match build() {
        Ok(mut sim) => {
            sim.refresh();
            Box::into_raw(Box::new(sim))
        }
        Err(e) => {
            set_error(e);
            std::ptr::null_mut()
        }
    }
}

/// # Safety
/// `sim` must come from `oht_new` and must not be used afterwards.
#[no_mangle]
pub unsafe extern "C" fn oht_drop(sim: *mut Sim) {
    if !sim.is_null() {
        drop(Box::from_raw(sim));
    }
}

#[no_mangle]
pub extern "C" fn oht_error_ptr() -> *const u8 {
    LAST_ERROR.with(|e| e.borrow().as_ptr())
}

#[no_mangle]
pub extern "C" fn oht_error_len() -> usize {
    LAST_ERROR.with(|e| e.borrow().len())
}

// ---------------------------------------------------------------------------
// Stepping
// ---------------------------------------------------------------------------

/// Advance `n` ticks, then refresh the snapshot and metrics once. Batching the
/// refresh is the point: at high speed multipliers the UI runs hundreds of
/// ticks per frame but only ever draws the last one.
///
/// # Safety
/// `sim` must be a live pointer from `oht_new`.
#[no_mangle]
pub unsafe extern "C" fn oht_tick(sim: *mut Sim, n: u32) {
    let Some(sim) = sim.as_mut() else { return };
    for _ in 0..n {
        sim.world.tick();
    }
    sim.refresh();
}

// ---------------------------------------------------------------------------
// Per-frame accessors
//
// Every pointer below is valid only until the next call that mutates the world.
// ---------------------------------------------------------------------------

macro_rules! accessor {
    ($name:ident, $ty:ty, $get:expr) => {
        /// # Safety
        /// `sim` must be a live pointer from `oht_new`. The returned pointer is
        /// valid until the next call that mutates the world.
        #[no_mangle]
        pub unsafe extern "C" fn $name(sim: *const Sim) -> $ty {
            match sim.as_ref() {
                Some(s) => {
                    let f: fn(&Sim) -> $ty = $get;
                    f(s)
                }
                None => Default::default(),
            }
        }
    };
}

accessor!(oht_veh_count, usize, |s| s.snap.veh_x.len());
accessor!(oht_veh_x, *const u16, |s| s.snap.veh_x.as_ptr());
accessor!(oht_veh_y, *const u16, |s| s.snap.veh_y.as_ptr());
accessor!(oht_veh_heading, *const u8, |s| s.snap.veh_heading.as_ptr());
accessor!(oht_veh_carrying, *const u8, |s| s.snap.veh_carrying.as_ptr());
accessor!(oht_veh_state, *const u8, |s| s.snap.veh_state.as_ptr());

accessor!(oht_machine_count, usize, |s| s.snap.machine_load.len());
accessor!(oht_machine_load, *const u16, |s| s
    .snap
    .machine_load
    .as_ptr());
accessor!(oht_machine_starvation, *const f32, |s| s
    .snap
    .machine_starvation
    .as_ptr());

accessor!(oht_cell_count, usize, |s| s.world.congestion.len());
accessor!(oht_congestion, *const f32, |s| s.world.congestion.as_ptr());

accessor!(oht_metrics, *const f64, |s| s.metrics.as_ptr());

#[no_mangle]
pub extern "C" fn oht_metric_count() -> usize {
    METRIC_COUNT
}

// ---------------------------------------------------------------------------
// Lots, targets, ports, machine utilisation, cycle times
// ---------------------------------------------------------------------------

accessor!(oht_lot_count, usize, |s| s.lots.id.len());
accessor!(oht_lot_id, *const u32, |s| s.lots.id.as_ptr());
accessor!(oht_lot_recipe, *const u8, |s| s.lots.recipe.as_ptr());
accessor!(oht_lot_step, *const u8, |s| s.lots.step.as_ptr());
accessor!(oht_lot_steps_total, *const u8, |s| s.lots.steps_total.as_ptr());
accessor!(oht_lot_place, *const u8, |s| s.lots.place.as_ptr());
accessor!(oht_lot_a, *const u16, |s| s.lots.a.as_ptr());
accessor!(oht_lot_b, *const u16, |s| s.lots.b.as_ptr());
accessor!(oht_lot_wait, *const u32, |s| s.lots.wait.as_ptr());
accessor!(oht_lot_priority, *const f32, |s| s.lots.priority.as_ptr());

accessor!(oht_veh_target_machine, *const i16, |s| s.targets.machine.as_ptr());
accessor!(oht_veh_target_port, *const i16, |s| s.targets.port.as_ptr());
accessor!(oht_veh_lot, *const i32, |s| s.targets.lot.as_ptr());

accessor!(oht_port_count, usize, |s| s.port_lot.len());
accessor!(oht_port_lot, *const i32, |s| s.port_lot.as_ptr());

accessor!(oht_machine_idle_ticks, *const u32, |s| s.machine_idle.as_ptr());

accessor!(oht_cycle_count, usize, |s| s.cycles.len());
accessor!(oht_cycle_times, *const f64, |s| s.cycles.as_ptr());

/// The planned route of one vehicle, as cell ids, next hop first. Separate from
/// the bulk accessors because it is per-vehicle and only the selected one is
/// ever drawn.
///
/// # Safety
/// `sim` must be live; the pointer is valid until the next call that mutates
/// the world.
#[no_mangle]
pub unsafe extern "C" fn oht_veh_route(sim: *const Sim, v: usize) -> *const usize {
    match sim.as_ref() {
        Some(s) => s
            .world
            .vehicles
            .get(v)
            .map(|veh| veh.route.as_ptr())
            .unwrap_or(std::ptr::null()),
        None => std::ptr::null(),
    }
}

/// # Safety
/// `sim` must be live.
#[no_mangle]
pub unsafe extern "C" fn oht_veh_route_len(sim: *const Sim, v: usize) -> usize {
    match sim.as_ref() {
        Some(s) => s.world.vehicles.get(v).map(|veh| veh.route.len()).unwrap_or(0),
        None => 0,
    }
}
