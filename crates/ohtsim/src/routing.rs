//! Shortest-path routing over the directed track graph.
//!
//! Search states are (cell, heading) rather than plain cells, because curve
//! cost depends on the direction you arrived from. That makes the state space
//! 4x the cell count, which is still tiny.

use std::collections::BinaryHeap;

use crate::geom::{manoeuvre, CellId, Dir, Grid, Manoeuvre, ALL_DIRS};
use crate::policy::RouteWeights;

/// f32 has no Ord, so wrap the cost for the heap. Costs are always finite here.
#[derive(Copy, Clone, PartialEq)]
struct Cost(f32);

impl Eq for Cost {}

impl PartialOrd for Cost {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Cost {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Reversed: BinaryHeap is a max-heap and we want the min.
        other
            .0
            .partial_cmp(&self.0)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
struct Node {
    cost: Cost,
    state: usize,
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.cost
            .cmp(&other.cost)
            .then_with(|| other.state.cmp(&self.state))
    }
}

/// Sentinel for a cell that is not on any spur.
const NO_SPUR: u32 = u32::MAX;

pub struct Router {
    n_states: usize,
    dist: Vec<f32>,
    prev: Vec<usize>,
    heap: BinaryHeap<Node>,
    /// Cells that may only be entered as an explicit destination, never passed
    /// through. Parking spurs: routing a loaded vehicle through a spur would
    /// let a parked vehicle block it, which is the failure spurs exist to stop.
    avoid: Vec<bool>,
    /// Which spur each cell belongs to, or `NO_SPUR`. Spurs are the connected
    /// runs of avoided cells.
    spur: Vec<u32>,
    /// Track neighbours of each cell, either direction, for grouping spurs.
    neighbours: Vec<Vec<CellId>>,
}

fn state_of(cell: CellId, d: Dir) -> usize {
    cell * 4 + d.index()
}

fn cell_of(state: usize) -> CellId {
    state / 4
}

fn dir_of(state: usize) -> Dir {
    Dir::from_index(state % 4)
}

pub struct RouteResult {
    /// Cells to visit, excluding the start cell. Empty if already there.
    pub path: Vec<CellId>,
    pub cost: f32,
}

impl Router {
    pub fn new(grid: &Grid) -> Router {
        let n = grid.len() * 4;
        Router {
            n_states: n,
            dist: vec![f32::INFINITY; n],
            prev: vec![usize::MAX; n],
            heap: BinaryHeap::new(),
            avoid: vec![false; grid.len()],
            spur: vec![NO_SPUR; grid.len()],
            neighbours: {
                let mut adj = vec![Vec::new(); grid.len()];
                for c in 0..grid.len() {
                    for (_, n) in grid.exits(c) {
                        adj[c].push(n);
                        adj[n].push(c);
                    }
                }
                adj
            },
        }
    }

    /// Mark cells as destination-only. Length must equal the cell count.
    /// Cells no route may pass *through* -- the parking spurs. A spur exists so
    /// an idle vehicle can leave the main line; routing a loaded vehicle
    /// through one would put it behind whatever is parked there.
    ///
    /// "Through" is doing real work in that sentence. A vehicle sitting on a
    /// spur must still be able to drive *out* of one, and a vehicle heading for
    /// a spur must be able to drive *in*. Spurs on this map are two cells deep,
    /// so both of those cross a second avoided cell; forbidding that outright
    /// left the four vehicles parked on the inner cells unable to route
    /// anywhere at all, and therefore never assigned a job for the whole run.
    pub fn set_avoid(&mut self, mask: Vec<bool>) {
        // Label each connected run of avoided cells, so "this spur" and "some
        // other spur" are distinguishable. Two cells are in the same spur when
        // track runs between them in either direction.
        self.spur = vec![NO_SPUR; mask.len()];
        let mut next_id = 0u32;
        for start in 0..mask.len() {
            if !mask[start] || self.spur[start] != NO_SPUR {
                continue;
            }
            let id = next_id;
            next_id += 1;
            let mut stack = vec![start];
            self.spur[start] = id;
            while let Some(c) = stack.pop() {
                for n in self.neighbours[c].iter().copied() {
                    if mask.get(n).copied().unwrap_or(false) && self.spur[n] == NO_SPUR {
                        self.spur[n] = id;
                        stack.push(n);
                    }
                }
            }
        }
        self.avoid = mask;
    }

    /// Whether a step from `cell` into `next` is allowed given the avoid mask.
    ///
    /// Entering an avoided cell is allowed only within a single spur: either
    /// the step stays inside the spur we are already on (driving out), or it
    /// enters the spur we are trying to reach (driving in). Crossing some
    /// *other* spur stays forbidden, which is the whole point -- that is where
    /// a parked vehicle would be sitting in the way.
    fn passable(&self, cell: CellId, next: CellId, target_spur: u32) -> bool {
        let spur = self.spur_of(next);
        spur == NO_SPUR || spur == self.spur_of(cell) || spur == target_spur
    }

    fn spur_of(&self, cell: CellId) -> u32 {
        self.spur.get(cell).copied().unwrap_or(NO_SPUR)
    }

    /// Cheapest route from `start` (arriving with `heading`) to any cell in
    /// `targets`. Reusing one Router across calls avoids reallocating.
    pub fn route(
        &mut self,
        grid: &Grid,
        congestion: &[f32],
        w: &RouteWeights,
        start: CellId,
        heading: Dir,
        targets: &[CellId],
    ) -> Option<RouteResult> {
        if targets.is_empty() {
            return None;
        }

        // Computed once, not per edge. Targets are either all on one spur (a
        // parking move) or none of them are, so the first spur found is the one
        // the route is allowed to enter.
        let target_spur = targets
            .iter()
            .map(|&t| self.spur_of(t))
            .find(|&s| s != NO_SPUR)
            .unwrap_or(NO_SPUR);
        if targets.contains(&start) {
            return Some(RouteResult {
                path: Vec::new(),
                cost: 0.0,
            });
        }

        for i in 0..self.n_states {
            self.dist[i] = f32::INFINITY;
            self.prev[i] = usize::MAX;
        }
        self.heap.clear();

        let s0 = state_of(start, heading);
        self.dist[s0] = 0.0;
        self.heap.push(Node {
            cost: Cost(0.0),
            state: s0,
        });

        let mut goal: Option<usize> = None;

        while let Some(Node { cost, state }) = self.heap.pop() {
            if cost.0 > self.dist[state] {
                continue;
            }
            let cell = cell_of(state);
            if targets.contains(&cell) && state != s0 {
                goal = Some(state);
                break;
            }
            let from_dir = dir_of(state);
            for d in ALL_DIRS {
                let next = match grid.step(cell, d) {
                    Some(n) => n,
                    None => continue,
                };
                if !self.passable(cell, next, target_spur) {
                    continue;
                }
                let step_cost = match manoeuvre(from_dir, d) {
                    Manoeuvre::Straight => w.length,
                    Manoeuvre::Curve => w.length + w.curve,
                    // Reversing is impossible on a monorail; the track bitmask
                    // should already forbid it, but guard anyway.
                    Manoeuvre::Reverse => continue,
                };
                let cong = congestion.get(next).copied().unwrap_or(0.0);
                let total = cost.0 + step_cost + w.congestion * cong;
                let ns = state_of(next, d);
                if total < self.dist[ns] {
                    self.dist[ns] = total;
                    self.prev[ns] = state;
                    self.heap.push(Node {
                        cost: Cost(total),
                        state: ns,
                    });
                }
            }
        }

        let goal = goal?;
        let cost = self.dist[goal];

        let mut path = Vec::new();
        let mut cur = goal;
        while cur != s0 {
            path.push(cell_of(cur));
            cur = self.prev[cur];
            if cur == usize::MAX {
                return None;
            }
        }
        path.reverse();
        Some(RouteResult { path, cost })
    }

    /// Route cost only. Used for scoring candidates without keeping the path.
    pub fn cost_to(
        &mut self,
        grid: &Grid,
        congestion: &[f32],
        w: &RouteWeights,
        start: CellId,
        heading: Dir,
        target: CellId,
    ) -> Option<f32> {
        self.route(grid, congestion, w, start, heading, &[target])
            .map(|r| r.cost)
    }

    /// Cost from `start` to *every* reachable cell, in one pass.
    ///
    /// Dispatch scores every (job, vehicle, destination) triple, so computing a
    /// route per pair would mean thousands of Dijkstras per tick. One distance
    /// field per vehicle turns those into array lookups.
    pub fn dist_field(
        &mut self,
        grid: &Grid,
        congestion: &[f32],
        w: &RouteWeights,
        start: CellId,
        heading: Dir,
    ) -> Vec<f32> {
        for i in 0..self.n_states {
            self.dist[i] = f32::INFINITY;
            self.prev[i] = usize::MAX;
        }
        self.heap.clear();

        let s0 = state_of(start, heading);
        self.dist[s0] = 0.0;
        self.heap.push(Node {
            cost: Cost(0.0),
            state: s0,
        });

        while let Some(Node { cost, state }) = self.heap.pop() {
            if cost.0 > self.dist[state] {
                continue;
            }
            let cell = cell_of(state);
            let from_dir = dir_of(state);
            for d in ALL_DIRS {
                let next = match grid.step(cell, d) {
                    Some(n) => n,
                    None => continue,
                };
                // No targets here: a distance field is to everywhere. Steps
                // out of a spur are allowed so a parked vehicle has finite
                // costs; steps into one are not, since nothing routes through.
                if !self.passable(cell, next, NO_SPUR) {
                    continue;
                }
                let step_cost = match manoeuvre(from_dir, d) {
                    Manoeuvre::Straight => w.length,
                    Manoeuvre::Curve => w.length + w.curve,
                    Manoeuvre::Reverse => continue,
                };
                let cong = congestion.get(next).copied().unwrap_or(0.0);
                let total = cost.0 + step_cost + w.congestion * cong;
                let ns = state_of(next, d);
                if total < self.dist[ns] {
                    self.dist[ns] = total;
                    self.prev[ns] = state;
                    self.heap.push(Node {
                        cost: Cost(total),
                        state: ns,
                    });
                }
            }
        }

        // Collapse the (cell, heading) states down to per-cell minima.
        let mut field = vec![f32::INFINITY; grid.len()];
        for s in 0..self.n_states {
            let c = cell_of(s);
            if self.dist[s] < field[c] {
                field[c] = self.dist[s];
            }
        }
        field
    }
}
