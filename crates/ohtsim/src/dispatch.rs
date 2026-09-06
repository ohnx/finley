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
use crate::routing::Router;

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

    // One distance field per idle vehicle: cost from that vehicle to any cell.
    let mut veh_field: Vec<(VehicleId, Vec<f32>)> = Vec::with_capacity(idle.len());
    for &v in &idle {
        let veh = &vehicles[v];
        let f = router.dist_field(grid, congestion, rw, veh.cell, veh.heading);
        veh_field.push((v, f));
    }

    // One distance field per distinct pickup cell: cost onward to any
    // destination. Seeded with an arbitrary legal heading, so the delivery cost
    // may be off by at most one curve — immaterial for ranking.
    let mut pickup_fields: Vec<(usize, Vec<f32>)> = Vec::new();
    for &jid in pending {
        let job = &jobs[jid];
        let cell = machines[job.from.0].ports[job.from.1].cell;
        if pickup_fields.iter().any(|(c, _)| *c == cell) {
            continue;
        }
        let heading = grid
            .exits(cell)
            .first()
            .map(|(d, _)| *d)
            .unwrap_or(ALL_DIRS[0]);
        let f = router.dist_field(grid, congestion, rw, cell, heading);
        pickup_fields.push((cell, f));
    }

    let mut candidates: Vec<Candidate> = Vec::new();

    for &jid in pending {
        let job = &jobs[jid];
        let lot = &lots[job.lot];
        let pickup_cell = machines[job.from.0].ports[job.from.1].cell;

        let kind = match lot.next_kind() {
            Some(k) => k,
            None => continue, // finished; should already be Done
        };

        let deliver_field = match pickup_fields.iter().find(|(c, _)| *c == pickup_cell) {
            Some((_, f)) => f,
            None => continue,
        };

        // Terms independent of vehicle and destination.
        let wait = lot.wait_ticks(now) as f32;
        let steps_left = lot.recipe.len().saturating_sub(lot.step) as f32;
        let lot_term = -dw.lot_wait * wait - dw.lot_priority * lot.priority
            + dw.steps_remaining * steps_left;

        // Buffering is for unblocking a tool, not for parking work.
        //
        // Three conditions, and all of them earn their place. No tool of the
        // required kind can take the lot, or storage would be used while a tool
        // stood free. The lot is not already in a buffer, or two buffers would
        // pass it back and forth while it made no progress. And the tool it is
        // sitting on is actually blocked -- it has finished a lot that cannot
        // reach an output port -- because moving this one frees that port.
        //
        // Without the last condition a buffered lot costs two transport moves
        // where one would do, and on a fleet already near saturation that is
        // pure loss: it cost starvation_biased 30 lots and doubled its p95.
        let source = &machines[job.from.0];
        let tool_free = machines
            .iter()
            .any(|m| m.kind == kind && m.free_port(PortKind::Input).is_some());
        let source_blocked = source.in_process.iter().any(|(_, remaining)| *remaining == 0);
        let from_buffer = source.is_buffer();

        for (m_id, m) in machines.iter().enumerate() {
            let usable = if m.kind == kind {
                true
            } else {
                m.is_buffer() && !tool_free && !from_buffer && source_blocked
            };
            if !usable {
                continue;
            }
            let port = match m.free_inbound_port() {
                Some(p) => p,
                None => continue,
            };
            let dest_cell = m.ports[port].cell;
            let deliver_cost = deliver_field[dest_cell];
            if !deliver_cost.is_finite() {
                continue;
            }

            let dest_term = -dw.dest_starvation * m.starvation
                + dw.dest_queue * m.load() as f32
                + dw.dest_congestion * deliver_cost;

            for (v, field) in &veh_field {
                let pickup_cost = field[pickup_cell];
                if !pickup_cost.is_finite() {
                    continue;
                }
                let score = dw.travel_to_pickup * pickup_cost + lot_term + dest_term;
                candidates.push(Candidate {
                    score,
                    vehicle: *v,
                    job: jid,
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
    let mut used_job: Vec<bool> = vec![false; jobs.len()];
    let mut used_port: Vec<(MachineId, PortId)> = Vec::new();
    let mut out = Vec::new();

    for c in candidates {
        if used_veh[c.vehicle] || used_job[c.job] {
            continue;
        }
        if used_port.contains(&c.dest) {
            continue;
        }
        let veh = &vehicles[c.vehicle];
        let pickup_cell = machines[jobs[c.job].from.0].ports[jobs[c.job].from.1].cell;
        let path = match router.route(
            grid,
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
        used_job[c.job] = true;
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
