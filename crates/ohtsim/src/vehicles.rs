//! What vehicles do each tick: advancing a job, deciding where to wait, and
//! resolving contention for cells.
//!
//! Split out of `world.rs` because it is the half of the tick loop with its own
//! rules -- no overtaking, one vehicle per cell, spurs are for parking and not
//! for driving through -- and those rules are easier to hold in mind without
//! the machine, lot and job phases interleaved. Still `impl World`: the phases
//! share too much state for anything else to be honest.

use crate::geom::{manoeuvre, CellId, Manoeuvre};
use crate::model::{LotState, MachineId, VehState, VehicleId};
use crate::policy::IdleMode;
use crate::world::{heading_between, World};

impl World {
    /// State transitions for vehicles that are free to act.
    pub(crate) fn advance_vehicles(&mut self) {
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
        // Dwell exists to stop a parked vehicle twitching between spurs on
        // every small change in starvation. It has no business delaying a
        // vehicle that is standing on the main line, where waiting is not
        // patience, it is a roadblock.
        let on_spur = self.parking.contains(&self.vehicles[v].cell);
        if on_spur && self.idle_ticks_of[v] < self.policy.idle.dwell_before_move {
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
            self.circulate(v);
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
                    let better = match hungriest {
                        // Strictly greater, so the *first* maximum wins.
                        Some(b) => m.starvation > self.machines[b].starvation,
                        None => true,
                    };
                    if better {
                        hungriest = Some(i);
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
            self.circulate(v);
        }
    }

    /// Keep a vehicle that cannot park moving, rather than stopping it where it
    /// stands.
    ///
    /// Rails are one-way and vehicles cannot overtake, so a vehicle halted on
    /// the main line is a wall: everything behind it queues until something
    /// else breaks the jam. Real OHT fleets keep circulating for exactly this
    /// reason -- a vehicle with nowhere to go loiters around the loop instead
    /// of parking on it. Spurs exist so that stopping is always off the line,
    /// so if no spur is available the answer is to keep moving, not to stop.
    ///
    /// One hop at a time: the vehicle re-decides on arrival, so it parks the
    /// moment a spur frees up rather than committing to a lap.
    fn circulate(&mut self, v: VehicleId) {
        let cell = self.vehicles[v].cell;
        let heading = self.vehicles[v].heading;
        let exits = self.grid.exits(cell);

        // Straight on where possible: a curve costs three ticks and there is no
        // destination here worth paying for. Spurs are skipped -- an empty one
        // is either claimed by another vehicle or would have been parked in
        // already, and driving in would strand this one behind a dead end.
        let onward = exits
            .iter()
            .find(|(d, n)| *d == heading && !self.parking.contains(n))
            .or_else(|| exits.iter().find(|(_, n)| !self.parking.contains(n)))
            .map(|(_, n)| *n);

        match onward {
            Some(next) => {
                self.vehicles[v].route = vec![next];
                self.vehicles[v].state = VehState::Repositioning;
            }
            // Nowhere legal to go at all. Validation rejects maps with dead
            // ends, so this only happens if every exit is a spur.
            None => self.vehicles[v].state = VehState::Idle,
        }
    }

    pub(crate) fn move_vehicles(&mut self) {
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
}
