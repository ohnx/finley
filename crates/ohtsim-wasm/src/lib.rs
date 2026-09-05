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
pub const METRIC_COUNT: usize = 13;

pub struct Sim {
    world: World,
    snap: Snapshot,
    metrics: [f64; METRIC_COUNT],
}

impl Sim {
    fn refresh(&mut self) {
        self.world.snapshot_into(&mut self.snap);
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
        ];
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
