//! The simulation world and its tick loop.
//!
//! Deterministic and headless by construction: a fixed integer tick, a seeded
//! RNG, and no rendering imports. Two runs with the same seed and policy
//! produce byte-identical metrics, which is the only way policy comparison
//! means anything. Rendering is a pure consumer of `snapshot()`.

use crate::config::{MapConfig, ScenarioConfig};
use crate::dispatch;
use crate::geom::{CellId, Dir, Grid};
use crate::metrics::Metrics;
use crate::model::{
    Job, JobId, Lot, LotId, LotState, Machine, MachineId, PortKind, VehState, Vehicle, VehicleId,
};
use crate::policy::{Policy, Trigger};
use crate::routing::Router;

/// xorshift64*. Small, fast, and reproducible across platforms — which matters
/// more here than statistical quality.
#[derive(Clone, Debug)]
pub struct Rng {
    state: u64,
}

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng {
            state: if seed == 0 { 0x9E3779B97F4A7C15 } else { seed },
        }
    }

    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545F4914F6CDD1D)
    }

    pub fn next_f32(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u32 << 24) as f32
    }
}

pub struct World {
    pub grid: Grid,
    pub machines: Vec<Machine>,
    pub lots: Vec<Lot>,
    pub vehicles: Vec<Vehicle>,
    pub jobs: Vec<Job>,
    pub pending: Vec<JobId>,
    pub parking: Vec<CellId>,
    pub occupancy: Vec<Option<VehicleId>>,
    pub congestion: Vec<f32>,
    pub policy: Policy,
    pub scenario: ScenarioConfig,
    pub rng: Rng,
    pub metrics: Metrics,
    pub router: Router,
    pub tick_count: u64,
    /// Index into `policy.profiles` currently in force.
    pub active_profile: usize,
    /// Job currently attached to each lot, if any.
    pub(crate) lot_job: Vec<Option<JobId>>,
    /// Lots still in the fab. `lots` keeps every lot ever made, so anything
    /// that scans it once per tick gets slower the longer the run goes -- the
    /// tick rate used to halve over 400k ticks for exactly that reason.
    pub(crate) active_lots: Vec<LotId>,
    pub(crate) idle_ticks_of: Vec<u32>,
    /// Whether the previous tick was already inside a resource deadlock, so an
    /// episode is counted once rather than every tick it persists.
    in_resource_deadlock: bool,
    /// Consecutive ticks with nothing routable, before it counts as a deadlock.
    resource_stall_ticks: u64,
}

impl World {
    pub fn new(map: MapConfig, scenario: ScenarioConfig, policy: Policy) -> World {
        let grid = map.grid;
        let n_cells = grid.len();
        let mut router = Router::new(&grid);
        // Spur cells are destination-only, so no through-route can be blocked
        // by a parked vehicle.
        let mut avoid = vec![false; n_cells];
        for &c in &map.parking {
            if c < n_cells {
                avoid[c] = true;
            }
        }
        router.set_avoid(avoid);

        let machines: Vec<Machine> = map.machines;
        let mut occupancy = vec![None; n_cells];

        // Place vehicles: declared start cells first, then parking in map
        // order, then any track cell. Walked in order rather than strided --
        // parking cells are listed as spur pairs, so taking them in order
        // spreads vehicles over the map the way the map author intended, and
        // it is what reference/reference_sim.py does.
        let mut starts: Vec<CellId> = scenario.vehicle_start_cells.clone();
        if starts.len() < scenario.vehicles {
            let pool: Vec<CellId> = map
                .parking
                .iter()
                .copied()
                .chain((0..n_cells).filter(|&c| grid.has_track(c)))
                .collect();
            for c in pool {
                if starts.len() >= scenario.vehicles {
                    break;
                }
                if !starts.contains(&c) {
                    starts.push(c);
                }
            }
        }

        let mut vehicles = Vec::new();
        for (id, &cell) in starts.iter().take(scenario.vehicles).enumerate() {
            let heading = grid
                .exits(cell)
                .first()
                .map(|(d, _)| *d)
                .unwrap_or(Dir::East);
            occupancy[cell] = Some(id);
            vehicles.push(Vehicle {
                id,
                cell,
                heading,
                ready_in: 0,
                state: VehState::Idle,
                route: Vec::new(),
                carrying: None,
                busy_ticks: 0,
                blocked_ticks: 0,
            });
        }

        let metrics = Metrics::for_machines(machines.iter().map(|m| m.name.clone()).collect());

        let n_veh = vehicles.len();
        World {
            grid,
            machines,
            lots: Vec::new(),
            vehicles,
            jobs: Vec::new(),
            pending: Vec::new(),
            parking: map.parking,
            occupancy,
            congestion: vec![0.0; n_cells],
            rng: Rng::new(scenario.seed),
            policy,
            scenario,
            metrics,
            router,
            tick_count: 0,
            active_profile: 0,
            lot_job: Vec::new(),
            active_lots: Vec::new(),
            idle_ticks_of: vec![0; n_veh],
            in_resource_deadlock: false,
            resource_stall_ticks: 0,
        }
    }

    // -----------------------------------------------------------------------
    // Tick
    // -----------------------------------------------------------------------

    pub fn tick(&mut self) {
        self.tick_count += 1;
        self.metrics.ticks = self.tick_count;

        self.update_congestion();
        self.run_machines();
        self.spawn_lots();
        self.create_jobs();
        self.detect_resource_deadlock();
        self.select_profile();
        self.assign_jobs();
        self.advance_vehicles();
        self.move_vehicles();
        self.collect_metrics();
    }

    pub fn run(&mut self, ticks: u64) {
        for _ in 0..ticks {
            self.tick();
        }
    }

    // -----------------------------------------------------------------------

    fn update_congestion(&mut self) {
        let d = self.policy.congestion_decay;
        for c in 0..self.congestion.len() {
            let occupied = if self.occupancy[c].is_some() { 1.0 } else { 0.0 };
            self.congestion[c] = self.congestion[c] * d + occupied * (1.0 - d);
        }
    }

    fn run_machines(&mut self) {
        let now = self.tick_count;
        for m_id in 0..self.machines.len() {
            if self.machines[m_id].is_source() {
                continue;
            }

            // Pull waiting lots off input ports while capacity allows.
            loop {
                let m = &self.machines[m_id];
                if m.in_process.len() >= m.capacity {
                    break;
                }
                let slot = m
                    .ports
                    .iter()
                    .enumerate()
                    .find(|(_, p)| p.kind == PortKind::Input && p.lot.is_some())
                    .map(|(i, p)| (i, p.lot.unwrap()));
                let (port_id, lot_id) = match slot {
                    Some(s) => s,
                    None => break,
                };
                let ticks = self.machines[m_id].process_ticks;
                self.machines[m_id].ports[port_id].lot = None;
                self.machines[m_id].in_process.push((lot_id, ticks));
                self.lots[lot_id].state = LotState::Processing(m_id);
            }

            // Advance work in progress.
            for entry in self.machines[m_id].in_process.iter_mut() {
                if entry.1 > 0 {
                    entry.1 -= 1;
                }
            }

            // Retire completed lots. A lot whose next hop has no free output
            // port stays in the machine — that backpressure is the point.
            let mut still: Vec<(LotId, u32)> = Vec::new();
            let finished: Vec<(LotId, u32)> = self.machines[m_id].in_process.clone();
            self.machines[m_id].in_process.clear();

            for (lot_id, remaining) in finished {
                if remaining > 0 {
                    still.push((lot_id, remaining));
                    continue;
                }
                let step_now = self.lots[lot_id].step + 1;
                self.lots[lot_id].step = step_now;

                if step_now >= self.lots[lot_id].recipe.len() {
                    self.lots[lot_id].state = LotState::Done;
                    self.active_lots.retain(|&l| l != lot_id);
                    self.metrics.lots_completed += 1;
                    let created = self.lots[lot_id].created_tick;
                    self.metrics.record_cycle(now.saturating_sub(created));
                    continue;
                }

                match self.machines[m_id].free_port(PortKind::Output) {
                    Some(p) => {
                        self.machines[m_id].ports[p].lot = Some(lot_id);
                        self.lots[lot_id].state = LotState::AtPort(m_id, p);
                        self.lots[lot_id].waiting_since = now;
                    }
                    None => {
                        // Output blocked; hold it and retry next tick.
                        self.lots[lot_id].step = step_now - 1;
                        still.push((lot_id, 0));
                    }
                }
            }
            self.machines[m_id].in_process = still;

            // Starvation signal.
            let m = &self.machines[m_id];
            let starved = m.in_process.is_empty() && !m.is_sink();
            let decay = 0.99f32;
            let s = m.starvation * decay + if starved { 1.0 - decay } else { 0.0 };
            self.machines[m_id].starvation = s;
            if starved {
                self.machines[m_id].idle_ticks += 1;
                self.metrics.machine_idle_ticks[m_id] += 1;
            }
        }
    }

    fn spawn_lots(&mut self) {
        if self.scenario.recipes.is_empty() {
            return;
        }
        let p = self.scenario.arrival_per_1000 / 1000.0;
        if self.rng.next_f32() >= p {
            return;
        }

        // Release control. Drawn *after* the arrival roll on purpose, so the
        // random stream does not depend on how full the fab is: two runs with
        // different caps still see the same demand at the same ticks, which is
        // the only way a cap sweep compares like with like.
        if self.scenario.wip_cap > 0 && self.active_lots.len() >= self.scenario.wip_cap {
            self.metrics.arrivals_deferred += 1;
            return;
        }

        let source_ids: Vec<MachineId> = self
            .machines
            .iter()
            .enumerate()
            .filter(|(_, m)| m.is_source())
            .map(|(i, _)| i)
            .collect();
        if source_ids.is_empty() {
            return;
        }
        let src = source_ids[(self.rng.next_u64() as usize) % source_ids.len()];
        let port = match self.machines[src].free_port(PortKind::Output) {
            Some(p) => p,
            None => return, // source blocked: no free port. Realistic.
        };

        let total: f32 = self.scenario.recipes.iter().map(|r| r.weight).sum();
        let mut pick = self.rng.next_f32() * total.max(f32::EPSILON);
        let mut chosen = 0usize;
        for (i, r) in self.scenario.recipes.iter().enumerate() {
            pick -= r.weight;
            if pick <= 0.0 {
                chosen = i;
                break;
            }
        }

        let hot = self.rng.next_f32() < self.scenario.hot_fraction;
        let lot_id = self.lots.len();
        self.lots.push(Lot {
            id: lot_id,
            recipe: self.scenario.recipes[chosen].steps.clone(),
            recipe_id: chosen,
            step: 0,
            state: LotState::AtPort(src, port),
            created_tick: self.tick_count,
            waiting_since: self.tick_count,
            priority: if hot { 1.0 } else { 0.0 },
        });
        self.lot_job.push(None);
        self.active_lots.push(lot_id);
        self.machines[src].ports[port].lot = Some(lot_id);
        self.metrics.lots_created += 1;
    }

    fn create_jobs(&mut self) {
        for idx in 0..self.active_lots.len() {
            let lot_id = self.active_lots[idx];
            if self.lot_job[lot_id].is_some() {
                continue;
            }
            let (m, p) = match self.lots[lot_id].state {
                LotState::AtPort(m, p) => (m, p),
                _ => continue,
            };
            if self.machines[m].ports[p].kind != PortKind::Output {
                continue;
            }
            if self.lots[lot_id].next_kind().is_none() {
                continue;
            }
            let jid = self.jobs.len();
            self.jobs.push(Job {
                id: jid,
                lot: lot_id,
                from: (m, p),
                to: None,
                assigned: None,
                created_tick: self.tick_count,
            });
            self.pending.push(jid);
            self.lot_job[lot_id] = Some(jid);
        }
    }

    /// Resource deadlock: every pending job is unroutable and nothing in
    /// flight can change that.
    ///
    /// The movement resolver's deadlock detection is about vehicles blocking
    /// each other on the track. This is the other kind, and the one the demo
    /// map actually hits: tools holding finished lots that cannot move because
    /// the tool they need is itself full of finished lots. Without buffers it
    /// closed a litho -> etch -> cmp -> litho cycle at around tick 12,000 and
    /// the fab simply stopped, with metrics that looked merely disappointing.
    fn detect_resource_deadlock(&mut self) {
        let stuck = !self.pending.is_empty()
            && self
                .vehicles
                .iter()
                .all(|v| v.carrying.is_none())
            && self.pending.iter().all(|&jid| {
                let lot = &self.lots[self.jobs[jid].lot];
                match lot.next_kind() {
                    None => true,
                    Some(kind) => !self
                        .machines
                        .iter()
                        .any(|m| m.kind == kind && m.free_port(PortKind::Input).is_some()),
                }
            });

        // A tick where nothing happens to be routable is not a deadlock -- a
        // tool finishing frees a port and it clears. Only a state that persists
        // past a whole tool cycle is genuinely stuck, so the count means
        // "the fab stopped" rather than "the fab paused".
        const PERSIST_TICKS: u64 = 300;

        if stuck {
            self.resource_stall_ticks += 1;
            if self.resource_stall_ticks >= PERSIST_TICKS {
                self.metrics.resource_deadlock_ticks += 1;
                if !self.in_resource_deadlock {
                    self.metrics.resource_deadlock_events += 1;
                    self.in_resource_deadlock = true;
                }
            }
        } else {
            self.resource_stall_ticks = 0;
            self.in_resource_deadlock = false;
        }
    }

    fn select_profile(&mut self) {
        let mut chosen = 0usize;
        for (i, prof) in self.policy.profiles.iter().enumerate() {
            let hit = match &prof.trigger {
                Trigger::Always => {
                    chosen = i;
                    false
                }
                Trigger::QueueDepthAbove { kind, n } => self
                    .machines
                    .iter()
                    .any(|m| &m.kind == kind && m.load() >= *n),
                Trigger::StarvationAbove { kind, ticks } => self
                    .machines
                    .iter()
                    .any(|m| &m.kind == kind && m.idle_ticks >= *ticks),
                Trigger::BacklogAbove { n } => self.pending.len() > *n,
            };
            if hit {
                chosen = i;
                break;
            }
        }
        self.active_profile = chosen;
    }

    fn assign_jobs(&mut self) {
        if self.pending.is_empty() {
            return;
        }
        let prof = &self.policy.profiles[self.active_profile];
        let assignments = dispatch::plan(
            &self.grid,
            &self.machines,
            &self.lots,
            &self.vehicles,
            &self.jobs,
            &self.pending,
            &self.congestion,
            &prof.dispatch,
            &prof.route,
            self.tick_count,
            &mut self.router,
            &self.metrics,
        );

        for a in assignments {
            self.jobs[a.job].assigned = Some(a.vehicle);
            self.jobs[a.job].to = Some(a.dest);
            // Reserve both ends so nothing else claims them.
            self.machines[a.dest.0].ports[a.dest.1].reserved_by = Some(a.vehicle);
            let from = self.jobs[a.job].from;
            self.machines[from.0].ports[from.1].reserved_by = Some(a.vehicle);

            self.vehicles[a.vehicle].state = VehState::ToPickup(a.job);
            self.vehicles[a.vehicle].route = a.path_to_pickup;
            self.vehicles[a.vehicle].blocked_ticks = 0;
            self.pending.retain(|&j| j != a.job);
        }
    }

    fn collect_metrics(&mut self) {
        self.metrics.record_backlog(self.pending.len());
        self.metrics.vehicle_tick_capacity += self.vehicles.len() as u64;
        for v in &mut self.vehicles {
            if v.state != VehState::Idle {
                v.busy_ticks += 1;
                self.metrics.vehicle_busy_ticks += 1;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Render feed
    // -----------------------------------------------------------------------

    /// Struct-of-arrays snapshot for the renderer. Kept flat deliberately: the
    /// wasm build should expose pointers into these and let JS read linear
    /// memory as typed arrays, rather than serialising state every frame.
    /// Total jobs ever created. Useful for spotting per-tick work that scales
    /// with the length of the run rather than the size of the fab.
    pub fn jobs_len(&self) -> usize {
        self.jobs.len()
    }

    /// Lots still in the fab, in creation order. Everything that walks lots
    /// every tick should walk this, not `lots`.
    pub fn active_lots(&self) -> &[LotId] {
        &self.active_lots
    }

    pub fn snapshot(&self) -> Snapshot {
        let mut s = Snapshot::default();
        self.snapshot_into(&mut s);
        s
    }

    /// Refill an existing `Snapshot` instead of allocating a new one.
    ///
    /// The browser shim calls this every frame and hands JS pointers into these
    /// arrays. Reusing the buffers keeps their capacity, so the pointers stay
    /// put between frames -- though JS must still re-derive its typed-array
    /// views each frame, because growing wasm memory detaches them.
    pub fn snapshot_into(&self, s: &mut Snapshot) {
        s.veh_x.clear();
        s.veh_y.clear();
        s.veh_heading.clear();
        s.veh_carrying.clear();
        s.veh_state.clear();
        s.machine_load.clear();
        s.machine_starvation.clear();
        for v in &self.vehicles {
            let (x, y) = self.grid.xy(v.cell);
            s.veh_x.push(x as u16);
            s.veh_y.push(y as u16);
            s.veh_heading.push(v.heading.index() as u8);
            s.veh_carrying.push(if v.carrying.is_some() { 1 } else { 0 });
            s.veh_state.push(match v.state {
                VehState::Idle => 0,
                VehState::ToPickup(_) => 1,
                VehState::Loading(_) => 2,
                VehState::ToDropoff(_) => 3,
                VehState::Unloading(_) => 4,
                VehState::Repositioning => 5,
            });
        }
        for m in &self.machines {
            s.machine_load.push(m.load() as u16);
            s.machine_starvation.push(m.starvation);
        }
        s.tick = self.tick_count;
    }
}

#[derive(Default, Clone, Debug)]
pub struct Snapshot {
    pub tick: u64,
    pub veh_x: Vec<u16>,
    pub veh_y: Vec<u16>,
    pub veh_heading: Vec<u8>,
    pub veh_carrying: Vec<u8>,
    pub veh_state: Vec<u8>,
    pub machine_load: Vec<u16>,
    pub machine_starvation: Vec<f32>,
}

pub(crate) fn heading_between(grid: &Grid, from: CellId, to: CellId) -> Option<Dir> {
    let (fx, fy) = grid.xy(from);
    let (tx, ty) = grid.xy(to);
    let dx = tx as i32 - fx as i32;
    let dy = ty as i32 - fy as i32;
    match (dx, dy) {
        (0, -1) => Some(Dir::North),
        (1, 0) => Some(Dir::East),
        (0, 1) => Some(Dir::South),
        (-1, 0) => Some(Dir::West),
        _ => None,
    }
}
