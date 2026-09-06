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
    n_cells: usize,
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
    /// `succ[cell * 4 + d]` is the cell reached by leaving `cell` heading `d`,
    /// or `NO_CELL`. `pred` is the same table inverted. Both are `Grid::step`
    /// answers cached at construction: the search asks that question a few
    /// thousand times per call and it costs two divisions each time. The grid
    /// is fixed for the life of a Router, so an edited map needs a new one.
    succ: Vec<CellId>,
    pred: Vec<CellId>,
}

/// Sentinel for "no such cell" in the `succ`/`pred` tables.
const NO_CELL: CellId = usize::MAX;

fn state_of(cell: CellId, d: Dir) -> usize {
    cell * 4 + d.index()
}

/// Index of the `(cell, heading)` state in a field returned by
/// [`Router::rev_dist_field`]. Public because dispatch holds those fields and
/// looks vehicles up in them by their exact heading, not by cell alone.
pub fn state_index(cell: CellId, d: Dir) -> usize {
    state_of(cell, d)
}

fn cell_of(state: usize) -> CellId {
    state / 4
}

fn dir_of(state: usize) -> Dir {
    Dir::from_index(state % 4)
}

/// Cost of every arrival/departure direction pair, indexed `from * 4 + to`.
/// Negative marks the reversal that no monorail can make. Built once per
/// search rather than matched per edge.
fn step_costs(w: &RouteWeights) -> [f32; 16] {
    let mut t = [-1.0f32; 16];
    for from in ALL_DIRS {
        for to in ALL_DIRS {
            t[from.index() * 4 + to.index()] = match manoeuvre(from, to) {
                Manoeuvre::Straight => w.length,
                Manoeuvre::Curve => w.length + w.curve,
                Manoeuvre::Reverse => continue,
            };
        }
    }
    t
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
            n_cells: grid.len(),
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
            succ: {
                let mut t = vec![NO_CELL; n];
                for c in 0..grid.len() {
                    for d in ALL_DIRS {
                        if let Some(next) = grid.step(c, d) {
                            t[c * 4 + d.index()] = next;
                        }
                    }
                }
                t
            },
            pred: {
                let mut t = vec![NO_CELL; n];
                for c in 0..grid.len() {
                    for d in ALL_DIRS {
                        if let Some(next) = grid.step(c, d) {
                            t[next * 4 + d.index()] = c;
                        }
                    }
                }
                t
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

        self.dist.fill(f32::INFINITY);
        self.prev.fill(usize::MAX);
        self.heap.clear();
        let costs = step_costs(w);

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
            let from_dir = dir_of(state).index();
            for d in 0..4 {
                let next = self.succ[cell * 4 + d];
                if next == NO_CELL {
                    continue;
                }
                if !self.passable(cell, next, target_spur) {
                    continue;
                }
                let step_cost = costs[from_dir * 4 + d];
                // Reversing is impossible on a monorail; the track bitmask
                // should already forbid it, but guard anyway.
                if step_cost < 0.0 {
                    continue;
                }
                let cong = congestion.get(next).copied().unwrap_or(0.0);
                let total = cost.0 + step_cost + w.congestion * cong;
                let ns = next * 4 + d;
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
        congestion: &[f32],
        w: &RouteWeights,
        start: CellId,
        heading: Dir,
        target: CellId,
    ) -> Option<f32> {
        self.route(congestion, w, start, heading, &[target])
            .map(|r| r.cost)
    }

    /// Cost from `start` to *every* reachable cell, in one pass.
    ///
    /// Dispatch scores every (job, vehicle, destination) triple, so computing a
    /// route per pair would mean thousands of Dijkstras per tick. One distance
    /// field per vehicle turns those into array lookups.
    pub fn dist_field(
        &mut self,
        congestion: &[f32],
        w: &RouteWeights,
        start: CellId,
        heading: Dir,
    ) -> Vec<f32> {
        // `prev` is not read here, so it is not reset either.
        self.dist.fill(f32::INFINITY);
        self.heap.clear();
        let costs = step_costs(w);

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
            let from_dir = dir_of(state).index();
            for d in 0..4 {
                let next = self.succ[cell * 4 + d];
                if next == NO_CELL {
                    continue;
                }
                // No targets here: a distance field is to everywhere. Steps
                // out of a spur are allowed so a parked vehicle has finite
                // costs; steps into one are not, since nothing routes through.
                if !self.passable(cell, next, NO_SPUR) {
                    continue;
                }
                let step_cost = costs[from_dir * 4 + d];
                if step_cost < 0.0 {
                    continue;
                }
                let cong = congestion.get(next).copied().unwrap_or(0.0);
                let total = cost.0 + step_cost + w.congestion * cong;
                let ns = next * 4 + d;
                if total < self.dist[ns] {
                    self.dist[ns] = total;
                    self.heap.push(Node {
                        cost: Cost(total),
                        state: ns,
                    });
                }
            }
        }

        // Collapse the (cell, heading) states down to per-cell minima.
        let mut field = vec![f32::INFINITY; self.n_cells];
        for s in 0..self.n_states {
            let c = cell_of(s);
            if self.dist[s] < field[c] {
                field[c] = self.dist[s];
            }
        }
        field
    }

    /// Cost to reach `target` *from* every `(cell, heading)` state, in one pass.
    ///
    /// The mirror image of `dist_field`, searching the same edges backwards.
    /// Dispatch needs the cost from each idle vehicle to each pickup, and there
    /// are usually many more idle vehicles than distinct pickups -- a WIP-capped
    /// fab runs at roughly one pending job and a dozen free vehicles -- so one
    /// backwards search per pickup replaces one forwards search per vehicle.
    ///
    /// Indexed by state rather than by cell: a vehicle has a definite heading,
    /// and collapsing to a per-cell minimum here would quietly hand it the cost
    /// of a turn it cannot make.
    pub fn rev_dist_field(
        &mut self,
        congestion: &[f32],
        w: &RouteWeights,
        target: CellId,
    ) -> Vec<f32> {
        self.dist.fill(f32::INFINITY);
        self.heap.clear();
        let costs = step_costs(w);

        // Arrival heading at the target is free to choose, so every state on
        // the target cell is a source.
        for d in ALL_DIRS {
            let s = state_of(target, d);
            self.dist[s] = 0.0;
            self.heap.push(Node {
                cost: Cost(0.0),
                state: s,
            });
        }

        while let Some(Node { cost, state }) = self.heap.pop() {
            if cost.0 > self.dist[state] {
                continue;
            }
            // `state` was arrived at by stepping into `next` heading `d`, so
            // its predecessors all sit on the one cell that step came from.
            let next = cell_of(state);
            let d = state % 4;
            let cell = self.pred[next * 4 + d];
            if cell == NO_CELL {
                continue;
            }
            if !self.passable(cell, next, NO_SPUR) {
                continue;
            }
            let cong = congestion.get(next).copied().unwrap_or(0.0);
            for from_dir in 0..4 {
                let step_cost = costs[from_dir * 4 + d];
                if step_cost < 0.0 {
                    continue;
                }
                let total = cost.0 + step_cost + w.congestion * cong;
                let ps = cell * 4 + from_dir;
                if total < self.dist[ps] {
                    self.dist[ps] = total;
                    // A state nothing can arrive in is a dead end backwards:
                    // its cost is worth recording, since a vehicle may be
                    // sitting in it, but expanding it would find nothing. Most
                    // of the state space is like this on one-way track, and
                    // keeping it off the heap is most of what makes the
                    // backwards search cheaper than the forwards one.
                    if self.pred[ps] != NO_CELL {
                        self.heap.push(Node {
                            cost: Cost(total),
                            state: ps,
                        });
                    }
                }
            }
        }

        self.dist.clone()
    }
}
