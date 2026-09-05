//! The simulation world and its tick loop.
//!
//! Deterministic and headless by construction: a fixed integer tick, a seeded
//! RNG, and no rendering imports. Two runs with the same seed and policy
//! produce byte-identical metrics, which is the only way policy comparison
//! means anything. Rendering is a pure consumer of `snapshot()`.

use crate::config::{MapConfig, ScenarioConfig};
use crate::dispatch;
use crate::geom::{manoeuvre, CellId, Dir, Grid, Manoeuvre};
use crate::metrics::Metrics;
use crate::model::{
    Job, JobId, Lot, LotId, LotState, Machine, MachineId, PortKind, VehState, Vehicle, VehicleId,
};
use crate::policy::{IdleMode, Policy, Trigger};
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
    lot_job: Vec<Option<JobId>>,
    idle_ticks_of: Vec<u32>,
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

        let mut metrics = Metrics::default();
        metrics.machine_idle_ticks = vec![0; machines.len()];
        metrics.machine_names = machines.iter().map(|m| m.name.clone()).collect();

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
            idle_ticks_of: vec![0; n_veh],
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
                    self.metrics.lots_completed += 1;
                    let created = self.lots[lot_id].created_tick;
                    self.metrics.cycle_times.push(now.saturating_sub(created));
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
            step: 0,
            state: LotState::AtPort(src, port),
            created_tick: self.tick_count,
            waiting_since: self.tick_count,
            priority: if hot { 1.0 } else { 0.0 },
        });
        self.lot_job.push(None);
        self.machines[src].ports[port].lot = Some(lot_id);
        self.metrics.lots_created += 1;
    }

    fn create_jobs(&mut self) {
        for lot_id in 0..self.lots.len() {
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

    /// State transitions for vehicles that are free to act.
    fn advance_vehicles(&mut self) {
        let now = self.tick_count;
        let hoist = self.policy.kinematics.hoist_ticks;

        for v in 0..self.vehicles.len() {
            if self.vehicles[v].ready_in > 0 {
                self.vehicles[v].ready_in -= 1;
                continue;
            }

            let state = self.vehicles[v].state.clone();
            match state {
                VehState::ToPickup(job) => {
                    if self.vehicles[v].route.is_empty() {
                        self.vehicles[v].state = VehState::Loading(job);
                        self.vehicles[v].ready_in = hoist;
                    }
                }
                VehState::Loading(job) => {
                    let (m, p) = self.jobs[job].from;
                    let lot = self.jobs[job].lot;
                    self.machines[m].ports[p].lot = None;
                    self.machines[m].ports[p].reserved_by = None;
                    self.lots[lot].state = LotState::InTransit(v);
                    self.vehicles[v].carrying = Some(lot);

                    let dest = match self.jobs[job].to {
                        Some(d) => d,
                        None => {
                            self.lots[lot].state = LotState::AtPort(m, p);
                            self.machines[m].ports[p].lot = Some(lot);
                            self.vehicles[v].carrying = None;
                            self.vehicles[v].state = VehState::Idle;
                            self.lot_job[lot] = None;
                            continue;
                        }
                    };
                    let dest_cell = self.machines[dest.0].ports[dest.1].cell;
                    let prof = &self.policy.profiles[self.active_profile];
                    let path = self.router.route(
                        &self.grid,
                        &self.congestion,
                        &prof.route,
                        self.vehicles[v].cell,
                        self.vehicles[v].heading,
                        &[dest_cell],
                    );
                    match path {
                        Some(r) => {
                            self.vehicles[v].route = r.path;
                            self.vehicles[v].state = VehState::ToDropoff(job);
                        }
                        None => {
                            // Unreachable destination: put the lot back and
                            // release its job, or create_jobs will never
                            // re-offer it and the lot strands forever.
                            self.machines[m].ports[p].lot = Some(lot);
                            self.lots[lot].state = LotState::AtPort(m, p);
                            self.lots[lot].waiting_since = now;
                            self.vehicles[v].carrying = None;
                            self.vehicles[v].state = VehState::Idle;
                            if let Some(d) = self.jobs[job].to {
                                self.machines[d.0].ports[d.1].reserved_by = None;
                            }
                            self.lot_job[lot] = None;
                        }
                    }
                }
                VehState::ToDropoff(job) => {
                    if self.vehicles[v].route.is_empty() {
                        self.vehicles[v].state = VehState::Unloading(job);
                        self.vehicles[v].ready_in = hoist;
                    }
                }
                VehState::Unloading(job) => {
                    let dest = match self.jobs[job].to {
                        Some(d) => d,
                        None => {
                            self.vehicles[v].state = VehState::Idle;
                            continue;
                        }
                    };
                    let lot = self.jobs[job].lot;
                    self.machines[dest.0].ports[dest.1].lot = Some(lot);
                    self.machines[dest.0].ports[dest.1].reserved_by = None;
                    self.lots[lot].state = LotState::AtPort(dest.0, dest.1);
                    self.lots[lot].waiting_since = now;
                    self.metrics
                        .delivery_waits
                        .push(now.saturating_sub(self.jobs[job].created_tick));
                    self.vehicles[v].carrying = None;
                    self.vehicles[v].state = VehState::Idle;
                    self.lot_job[lot] = None;
                }
                VehState::Idle | VehState::Repositioning => {
                    self.maybe_reposition(v);
                }
            }
        }
    }

    fn maybe_reposition(&mut self, v: VehicleId) {
        if self.vehicles[v].state == VehState::Repositioning
            && !self.vehicles[v].route.is_empty()
        {
            return;
        }
        self.idle_ticks_of[v] = self.idle_ticks_of[v].saturating_add(1);
        if self.idle_ticks_of[v] < self.policy.idle.dwell_before_move {
            return;
        }
        if self.parking.is_empty() || self.policy.idle.mode == IdleMode::StayPut {
            self.vehicles[v].state = VehState::Idle;
            return;
        }

        // Only ever aim at parking that is actually free. Targeting an
        // occupied spur leaves the vehicle waiting on the main line, which is
        // the blockage parking was supposed to avoid.
        //
        // "Free" has to include spurs another vehicle is already driving to,
        // not just spurs that are occupied right now. Two vehicles that pick
        // the same empty spur both commit to it; the loser arrives, finds it
        // taken, and re-decides from wherever it is standing -- which is on
        // the main line.
        let claimed: Vec<CellId> = self
            .vehicles
            .iter()
            .filter(|o| o.id != v && o.state == VehState::Repositioning)
            .filter_map(|o| o.route.last().copied())
            .collect();
        let free_parking: Vec<CellId> = self
            .parking
            .iter()
            .copied()
            .filter(|&c| self.occupancy[c].is_none() || self.occupancy[c] == Some(v))
            .filter(|c| !claimed.contains(c))
            .collect();
        if free_parking.is_empty() {
            self.vehicles[v].state = VehState::Idle;
            return;
        }

        let targets: Vec<CellId> = match self.policy.idle.mode {
            IdleMode::NearestPark => free_parking.clone(),
            IdleMode::Preposition => {
                // Park nearest the hungriest tool.
                // First-wins on ties, not last. Early in a run every tool is
                // equally starved, so the tie-break decides where the whole
                // fleet prepositions -- `max_by` would silently pick the last
                // machine in map order instead of the first.
                let mut hungriest: Option<MachineId> = None;
                for (i, m) in self.machines.iter().enumerate() {
                    if m.is_source() || m.is_sink() {
                        continue;
                    }
                    match hungriest {
                        Some(b) if !(m.starvation > self.machines[b].starvation) => {}
                        _ => hungriest = Some(i),
                    }
                }
                match hungriest {
                    Some(mi) => {
                        let target_cell = self.machines[mi]
                            .ports
                            .first()
                            .map(|p| p.cell)
                            .unwrap_or(self.vehicles[v].cell);
                        let (tx, ty) = self.grid.xy(target_cell);
                        let d2 = |c: CellId| -> i64 {
                            let (x, y) = self.grid.xy(c);
                            let dx = x as i64 - tx as i64;
                            let dy = y as i64 - ty as i64;
                            dx * dx + dy * dy
                        };
                        // Keep every spur tied for nearest, not an arbitrary
                        // one of them. Straight-line distance to a tool is a
                        // crude proxy on a directed track graph and ties are
                        // common on a symmetric map; collapsing to one cell
                        // makes a coin-flip decide where the fleet waits, and
                        // strands the vehicle entirely if that one cell turns
                        // out to be unreachable. The router picks among the
                        // tied cells by real route cost, which is the quantity
                        // the vehicle actually pays.
                        let best = free_parking.iter().map(|&c| d2(c)).min().unwrap_or(0);
                        free_parking
                            .iter()
                            .copied()
                            .filter(|&c| d2(c) == best)
                            .collect()
                    }
                    None => free_parking.clone(),
                }
            }
            IdleMode::StayPut => Vec::new(),
        };

        if targets.contains(&self.vehicles[v].cell) {
            self.vehicles[v].state = VehState::Idle;
            self.idle_ticks_of[v] = 0;
            return;
        }

        // Falling back to any free spur matters more than honouring the
        // starvation bias: an idle vehicle left standing on the main line
        // blocks the loop for everything behind it, and no-overtaking means
        // that congestion propagates backward until the fab gridlocks.
        // Prepositioning is a preference, keeping the line clear is not.
        let prof = self.policy.profiles[self.active_profile].route.clone();
        let (cell, heading) = (self.vehicles[v].cell, self.vehicles[v].heading);
        let route = self
            .router
            .route(&self.grid, &self.congestion, &prof, cell, heading, &targets)
            .or_else(|| {
                if targets.len() == free_parking.len() {
                    None
                } else {
                    self.router.route(
                        &self.grid,
                        &self.congestion,
                        &prof,
                        cell,
                        heading,
                        &free_parking,
                    )
                }
            });
        if let Some(r) = route {
            self.vehicles[v].route = r.path;
            self.vehicles[v].state = VehState::Repositioning;
        } else {
            self.vehicles[v].state = VehState::Idle;
        }
    }

    fn move_vehicles(&mut self) {
        let n = self.vehicles.len();
        let mut proposals: Vec<Option<CellId>> = vec![None; n];
        let mut priority: Vec<i64> = vec![0; n];

        for v in 0..n {
            let veh = &self.vehicles[v];
            // Loaded and long-waiting vehicles win contested cells. This is the
            // merge rule; swapping it for pure FIFO changes fab behaviour a lot.
            priority[v] = -(veh.blocked_ticks as i64) * 10
                - if veh.carrying.is_some() { 5 } else { 0 };
            if veh.ready_in > 0 {
                continue;
            }
            if let Some(next) = veh.next_cell() {
                proposals[v] = Some(next);
            }
        }

        let result = crate::movement::resolve(&self.occupancy, &proposals, &priority);
        self.metrics.cycles_rotated += result.cycles_rotated as u64;

        for (v, dest) in &result.moves {
            let src = self.vehicles[*v].cell;
            let old_heading = self.vehicles[*v].heading;
            let new_heading = heading_between(&self.grid, src, *dest).unwrap_or(old_heading);
            let cost = match manoeuvre(old_heading, new_heading) {
                Manoeuvre::Straight => self.policy.kinematics.straight_ticks,
                Manoeuvre::Curve => self.policy.kinematics.curve_ticks,
                Manoeuvre::Reverse => self.policy.kinematics.curve_ticks,
            };
            self.occupancy[src] = None;
            self.occupancy[*dest] = Some(*v);
            self.vehicles[*v].cell = *dest;
            self.vehicles[*v].heading = new_heading;
            self.vehicles[*v].ready_in = cost.saturating_sub(1);
            self.vehicles[*v].blocked_ticks = 0;
            if !self.vehicles[*v].route.is_empty() {
                self.vehicles[*v].route.remove(0);
            }
        }

        for v in &result.stalled {
            self.vehicles[*v].blocked_ticks = self.vehicles[*v].blocked_ticks.saturating_add(1);
        }

        if !result.deadlocks.is_empty() {
            self.metrics.deadlock_events += result.deadlocks.len() as u64;
        }

        // Recovery: a vehicle blocked far too long gets its route thrown away
        // and recomputed, which lets the congestion term route it elsewhere.
        let threshold = self.policy.stuck_threshold;
        for v in 0..n {
            if self.vehicles[v].blocked_ticks > threshold {
                self.metrics.stuck_vehicle_events += 1;
                self.vehicles[v].blocked_ticks = 0;
                self.vehicles[v].route.clear();
                let state = self.vehicles[v].state.clone();
                match state {
                    VehState::ToPickup(job) => {
                        let (m, p) = self.jobs[job].from;
                        let cell = self.machines[m].ports[p].cell;
                        self.reroute(v, cell);
                    }
                    VehState::ToDropoff(job) => {
                        if let Some(d) = self.jobs[job].to {
                            let cell = self.machines[d.0].ports[d.1].cell;
                            self.reroute(v, cell);
                        }
                    }
                    _ => {
                        self.vehicles[v].state = VehState::Idle;
                    }
                }
            }
        }
    }

    fn reroute(&mut self, v: VehicleId, target: CellId) {
        let prof = &self.policy.profiles[self.active_profile];
        if let Some(r) = self.router.route(
            &self.grid,
            &self.congestion,
            &prof.route,
            self.vehicles[v].cell,
            self.vehicles[v].heading,
            &[target],
        ) {
            self.vehicles[v].route = r.path;
        }
    }

    fn collect_metrics(&mut self) {
        self.metrics.backlog_samples.push(self.pending.len());
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
    pub fn snapshot(&self) -> Snapshot {
        let mut s = Snapshot::default();
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
        s
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

fn heading_between(grid: &Grid, from: CellId, to: CellId) -> Option<Dir> {
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
