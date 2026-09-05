//! One-vehicle-per-cell movement resolution.
//!
//! Naive per-vehicle iteration breaks trains: six vehicles nose-to-tail all
//! wanting to advance would see the cell ahead as occupied and stall, so the
//! train would creep one vehicle per tick and produce phantom congestion. This
//! resolves the whole tick as a unit instead.
//!
//! Note on deadlock: a fully packed cycle where every member wants the next
//! cell is a legal *rotation*, not a deadlock, and is moved atomically. A chain
//! terminating in a hoisting vehicle is a transient stall, not a deadlock
//! either. True movement deadlock only appears here as a cycle whose members
//! are not all proposing; genuine resource deadlock (vehicles waiting on ports
//! that will never free) is detected separately, in `world`.

use crate::geom::CellId;
use crate::model::VehicleId;

#[derive(Debug, Default)]
pub struct MoveResult {
    pub moves: Vec<(VehicleId, CellId)>,
    pub stalled: Vec<VehicleId>,
    pub cycles_rotated: usize,
    pub deadlocks: Vec<Vec<VehicleId>>,
}

/// `occupancy` is indexed by cell, `proposals` and `priority` by vehicle id.
/// A `None` proposal means the vehicle is not attempting to move this tick.
/// Lower `priority` wins a contested cell.
pub fn resolve(
    occupancy: &[Option<VehicleId>],
    proposals: &[Option<CellId>],
    priority: &[i64],
) -> MoveResult {
    let n_cells = occupancy.len();
    let n_veh = proposals.len();
    let mut out = MoveResult::default();

    // Where each vehicle currently is.
    let mut pos: Vec<Option<CellId>> = vec![None; n_veh];
    for (cell, occ) in occupancy.iter().enumerate() {
        if let Some(v) = occ {
            if *v < n_veh {
                pos[*v] = Some(cell);
            }
        }
    }

    // --- Phase 1: arbitrate contested cells --------------------------------
    let mut winner: Vec<Option<VehicleId>> = vec![None; n_cells];
    for v in 0..n_veh {
        let tgt = match proposals[v] {
            Some(t) if t < n_cells => t,
            _ => continue,
        };
        match winner[tgt] {
            None => winner[tgt] = Some(v),
            Some(cur) => {
                if priority[v] < priority[cur] {
                    winner[tgt] = Some(v);
                }
            }
        }
    }

    // `active[v]` is the target a vehicle actually holds a claim on.
    let mut active: Vec<Option<CellId>> = vec![None; n_veh];
    for v in 0..n_veh {
        if let Some(tgt) = proposals[v] {
            if tgt < n_cells && winner[tgt] == Some(v) {
                active[v] = Some(tgt);
            } else {
                out.stalled.push(v);
            }
        }
    }

    // --- Phase 2: iteratively move anyone whose target is now free ---------
    let mut occ: Vec<Option<VehicleId>> = occupancy.to_vec();
    let mut moved: Vec<bool> = vec![false; n_veh];

    let mut progress = true;
    while progress {
        progress = false;
        for v in 0..n_veh {
            if moved[v] {
                continue;
            }
            let tgt = match active[v] {
                Some(t) => t,
                None => continue,
            };
            if occ[tgt].is_none() {
                if let Some(src) = pos[v] {
                    occ[src] = None;
                }
                occ[tgt] = Some(v);
                moved[v] = true;
                out.moves.push((v, tgt));
                progress = true;
            }
        }
    }

    // --- Phase 3: cycles among whoever is left -----------------------------
    // Wait-graph is functional: each vehicle waits on at most one blocker.
    let mut waits_on: Vec<Option<VehicleId>> = vec![None; n_veh];
    for v in 0..n_veh {
        if moved[v] {
            continue;
        }
        if let Some(tgt) = active[v] {
            if let Some(blocker) = occ[tgt] {
                if blocker != v {
                    waits_on[v] = Some(blocker);
                }
            }
        }
    }

    for cyc in find_cycles(&waits_on, n_veh) {
        // Rotatable only if every member is actively claiming a move.
        let rotatable = cyc.iter().all(|&v| active[v].is_some() && !moved[v]);
        if rotatable {
            for &v in &cyc {
                if let Some(tgt) = active[v] {
                    out.moves.push((v, tgt));
                    moved[v] = true;
                }
            }
            out.cycles_rotated += 1;
        } else {
            out.deadlocks.push(cyc);
        }
    }

    for v in 0..n_veh {
        if active[v].is_some() && !moved[v] {
            out.stalled.push(v);
        }
    }

    out
}

/// Cycles in a functional graph (each node has at most one outgoing edge).
fn find_cycles(waits_on: &[Option<VehicleId>], n: usize) -> Vec<Vec<VehicleId>> {
    const UNSEEN: u8 = 0;
    const ON_PATH: u8 = 1;
    const DONE: u8 = 2;

    let mut state = vec![UNSEEN; n];
    let mut cycles = Vec::new();

    for start in 0..n {
        if state[start] != UNSEEN || waits_on[start].is_none() {
            continue;
        }
        let mut path: Vec<VehicleId> = Vec::new();
        let mut node = start;
        loop {
            if state[node] == ON_PATH {
                // Found a cycle: the tail of `path` from the first sighting.
                if let Some(at) = path.iter().position(|&x| x == node) {
                    cycles.push(path[at..].to_vec());
                }
                break;
            }
            if state[node] == DONE {
                break;
            }
            state[node] = ON_PATH;
            path.push(node);
            match waits_on[node] {
                Some(next) => node = next,
                None => break,
            }
        }
        for &p in &path {
            state[p] = DONE;
        }
    }

    cycles
}

#[cfg(test)]
mod tests {
    use super::*;

    fn occ_from(pairs: &[(CellId, VehicleId)], n_cells: usize) -> Vec<Option<VehicleId>> {
        let mut o = vec![None; n_cells];
        for &(c, v) in pairs {
            o[c] = Some(v);
        }
        o
    }

    #[test]
    fn train_advances_fully_in_one_tick() {
        let occ = occ_from(&[(0, 0), (1, 1), (2, 2), (3, 3), (4, 4), (5, 5)], 16);
        let props = vec![Some(1), Some(2), Some(3), Some(4), Some(5), Some(6)];
        let pri: Vec<i64> = (0..6).collect();
        let r = resolve(&occ, &props, &pri);
        assert_eq!(r.moves.len(), 6);
        assert!(r.deadlocks.is_empty());
    }

    #[test]
    fn train_behind_hoist_does_not_move() {
        let occ = occ_from(&[(0, 0), (1, 1), (2, 2), (3, 3), (4, 4), (5, 5)], 16);
        let props = vec![Some(1), Some(2), Some(3), Some(4), Some(5), None];
        let pri: Vec<i64> = (0..6).collect();
        let r = resolve(&occ, &props, &pri);
        assert_eq!(r.moves.len(), 0);
        assert_eq!(r.stalled.len(), 5);
        assert!(r.deadlocks.is_empty(), "hoist stall is not a deadlock");
    }

    #[test]
    fn contested_cell_has_one_winner() {
        let occ = occ_from(&[(1, 0), (2, 1)], 16);
        let props = vec![Some(10), Some(10)];
        let pri = vec![5i64, 1i64];
        let r = resolve(&occ, &props, &pri);
        assert_eq!(r.moves, vec![(1, 10)]);
        assert!(r.stalled.contains(&0));
    }

    #[test]
    fn packed_ring_rotates() {
        let occ = occ_from(&[(0, 0), (1, 1), (2, 2), (3, 3)], 4);
        let props = vec![Some(1), Some(2), Some(3), Some(0)];
        let pri: Vec<i64> = (0..4).collect();
        let r = resolve(&occ, &props, &pri);
        assert_eq!(r.moves.len(), 4);
        assert_eq!(r.cycles_rotated, 1);
        assert!(r.deadlocks.is_empty());
    }

    #[test]
    fn packed_ring_with_hoist_stalls_without_false_deadlock() {
        let occ = occ_from(&[(0, 0), (1, 1), (2, 2), (3, 3)], 4);
        let props = vec![Some(1), Some(2), Some(3), None];
        let pri: Vec<i64> = (0..4).collect();
        let r = resolve(&occ, &props, &pri);
        assert_eq!(r.moves.len(), 0);
        assert!(r.deadlocks.is_empty());
    }
}
