//! Job assignment.
//!
//! Scores every (job, vehicle, destination-tool) triple as a weighted sum of
//! named criteria and assigns greedily from best to worst. Picking *which* tool
//! of the right kind receives the lot is part of the decision, not a separate
//! step — that is where load balancing lives.

use crate::geom::{Grid, ALL_DIRS};
use crate::metrics::Metrics;
use crate::model::{Job, JobId, Lot, Machine, MachineId, PortId, PortKind, Vehicle, VehicleId};
use crate::policy::{DispatchWeights, RouteWeights};
use crate::routing::{state_index, Router};

#[derive(Clone, Debug)]
pub struct Assignment {
    pub vehicle: VehicleId,
    pub job: JobId,
    pub dest: (MachineId, PortId),
    /// Cells from the vehicle to the pickup port. Precomputed here so the
    /// world does not have to route again.
    pub path_to_pickup: Vec<usize>,
}

struct Candidate {
    score: f32,
    vehicle: VehicleId,
    job: JobId,
    /// Position of `job` within `pending`, so the "already assigned" mask can
    /// be sized by the pending queue instead of by every job ever created.
    slot: usize,
    dest: (MachineId, PortId),
}

#[allow(clippy::too_many_arguments)]
pub fn plan(
    grid: &Grid,
    machines: &[Machine],
    lots: &[Lot],
    vehicles: &[Vehicle],
    jobs: &[Job],
    pending: &[JobId],
    congestion: &[f32],
    dw: &DispatchWeights,
    rw: &RouteWeights,
    now: u64,
    router: &mut Router,
    _metrics: &Metrics,
) -> Vec<Assignment> {
    let idle: Vec<VehicleId> = vehicles
        .iter()
        .filter(|v| v.is_idle() || v.state == crate::model::VehState::Repositioning)
        .map(|v| v.id)
        .collect();

    if idle.is_empty() || pending.is_empty() {
        return Vec::new();
    }

    // One distance field per distinct pickup cell, searched backwards: a single
    // pass then gives every idle vehicle's cost to reach that pickup. A
    // WIP-capped fab runs at a couple of pending jobs and a dozen free
    // vehicles, so this is one search where a field per vehicle was a dozen.
    struct Pickup {
        cell: usize,
        /// Indexed by `(cell, heading)` state rather than by cell: a vehicle
        /// has a definite heading, and a per-cell minimum would quietly hand it
        /// the cost of a turn it cannot make.
        approach: Vec<f32>,
    }
    let mut pickup_fields: Vec<Pickup> = Vec::new();
    for &jid in pending {
        let job = &jobs[jid];
        let cell = machines[job.from.0].ports[job.from.1].cell;
        if pickup_fields.iter().any(|p| p.cell == cell) {
            continue;
        }
        pickup_fields.push(Pickup {
            cell,
            approach: router.rev_dist_field(congestion, rw, cell),
        });
    }

    let mut candidates: Vec<Candidate> = Vec::new();

    for (slot, &jid) in pending.iter().enumerate() {
        let job = &jobs[jid];
        let lot = &lots[job.lot];
        let pickup_cell = machines[job.from.0].ports[job.from.1].cell;

        let kind = match lot.next_kind() {
            Some(k) => k,
            None => continue, // finished; should already be Done
        };

        let approach = match pickup_fields.iter().find(|p| p.cell == pickup_cell) {
            Some(p) => &p.approach,
            None => continue,
        };
        // Delivery cost is wanted at a handful of cells -- the free input ports
        // of the tools that run this step -- so it is asked for one target at a
        // time. A search that stops when it reaches its target is an order of
        // magnitude cheaper than one that fills the map, and filling the map to
        // read three cells of it was most of what dispatch cost.
        //
        // Seeded with an arbitrary legal heading out of the pickup, so a
        // delivery cost may be off by at most one curve -- immaterial for
        // ranking.
        let deliver_heading = grid
            .exits(pickup_cell)
            .first()
            .map(|(d, _)| *d)
            .unwrap_or(ALL_DIRS[0]);

        // Terms independent of vehicle and destination.
        let wait = lot.wait_ticks(now) as f32;
        let steps_left = lot.recipe.len().saturating_sub(lot.step) as f32;
        let lot_term = -dw.lot_wait * wait - dw.lot_priority * lot.priority
            + dw.steps_remaining * steps_left;

        for (m_id, m) in machines.iter().enumerate() {
            if m.kind != kind {
                continue;
            }
            let port = match m.free_port(PortKind::Input) {
                Some(p) => p,
                None => continue,
            };
            let dest_cell = m.ports[port].cell;
            let deliver_cost =
                match router.cost_to(congestion, rw, pickup_cell, deliver_heading, dest_cell) {
                    Some(c) => c,
                    None => continue,
                };

            let dest_term = -dw.dest_starvation * m.starvation
                + dw.dest_queue * m.load() as f32
                + dw.dest_congestion * deliver_cost;

            for &v in &idle {
                let veh = &vehicles[v];
                let pickup_cost = approach[state_index(veh.cell, veh.heading)];
                if !pickup_cost.is_finite() {
                    continue;
                }
                let score = dw.travel_to_pickup * pickup_cost + lot_term + dest_term;
                candidates.push(Candidate {
                    score,
                    vehicle: v,
                    job: jid,
                    slot,
                    dest: (m_id, port),
                });
            }
        }
    }

    // Deterministic ordering: score, then ids as tiebreak. Without the tiebreak
    // two runs with identical inputs could diverge on float ties.
    candidates.sort_by(|a, b| {
        a.score
            .partial_cmp(&b.score)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a.vehicle.cmp(&b.vehicle))
            .then_with(|| a.job.cmp(&b.job))
            .then_with(|| a.dest.0.cmp(&b.dest.0))
    });

    let mut used_veh: Vec<bool> = vec![false; vehicles.len()];
    // Indexed by position within `pending`, not by job id: job ids run to
    // every job the fab has ever created, so sizing by them made this
    // allocation grow without bound over a long run.
    let mut used_job: Vec<bool> = vec![false; pending.len()];
    let mut used_port: Vec<(MachineId, PortId)> = Vec::new();
    let mut out = Vec::new();

    for c in candidates {
        if used_veh[c.vehicle] || used_job[c.slot] {
            continue;
        }
        if used_port.contains(&c.dest) {
            continue;
        }
        let veh = &vehicles[c.vehicle];
        let pickup_cell = machines[jobs[c.job].from.0].ports[jobs[c.job].from.1].cell;
        let path = match router.route(
            congestion,
            rw,
            veh.cell,
            veh.heading,
            &[pickup_cell],
        ) {
            Some(r) => r.path,
            None => continue,
        };
        used_veh[c.vehicle] = true;
        used_job[c.slot] = true;
        used_port.push(c.dest);
        out.push(Assignment {
            vehicle: c.vehicle,
            job: c.job,
            dest: c.dest,
            path_to_pickup: path,
        });
    }

    out
}
