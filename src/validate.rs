//! Map validation.
//!
//! Ported from `reference/gen_map2.py`, which generated the demo map and
//! checked it before writing it out. That check has to live in Rust before any
//! map editor exists: a hand-drawn map that strands vehicles fails in ways that
//! are miserable to diagnose from an isometric view, where all you see is that
//! the fab stopped.
//!
//! The load-bearing property is the last one. A parking spur exists so an idle
//! vehicle can leave the loop; if the author draws parking *on* the loop, one
//! parked vehicle severs it and the fab gridlocks with zero lots completed.
//! Checking that the main line stays strongly connected with every spur cell
//! removed is what catches that at edit time.

use crate::config::MapConfig;
use crate::geom::{CellId, Grid, ALL_DIRS};

/// A single validation failure. `cell` is set when the problem belongs to one
/// place on the map, so an editor can highlight it.
#[derive(Clone, Debug, PartialEq)]
pub struct Problem {
    pub cell: Option<CellId>,
    pub message: String,
}

impl Problem {
    fn at(cell: CellId, message: String) -> Problem {
        Problem {
            cell: Some(cell),
            message,
        }
    }

    fn global(message: &str) -> Problem {
        Problem {
            cell: None,
            message: message.to_string(),
        }
    }
}

impl std::fmt::Display for Problem {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.cell {
            Some(c) => write!(f, "cell {}: {}", c, self.message),
            None => write!(f, "{}", self.message),
        }
    }
}

/// Predecessors of every track cell. The forward graph is implicit in the exit
/// bitmask; strong connectivity needs the reverse graph too.
fn predecessors(grid: &Grid) -> Vec<Vec<CellId>> {
    let mut rev = vec![Vec::new(); grid.len()];
    for c in 0..grid.len() {
        if !grid.has_track(c) {
            continue;
        }
        for (_, n) in grid.exits(c) {
            rev[n].push(c);
        }
    }
    rev
}

/// Every cell in `nodes` reaches every other, travelling only through `nodes`.
fn strongly_connected(grid: &Grid, rev: &[Vec<CellId>], nodes: &[CellId]) -> bool {
    let Some(&start) = nodes.first() else {
        return true;
    };
    let mut member = vec![false; grid.len()];
    for &c in nodes {
        member[c] = true;
    }

    let reach = |forward: bool| -> usize {
        let mut seen = vec![false; grid.len()];
        seen[start] = true;
        let mut stack = vec![start];
        let mut n = 1;
        while let Some(c) = stack.pop() {
            if forward {
                for (_, next) in grid.exits(c) {
                    if member[next] && !seen[next] {
                        seen[next] = true;
                        n += 1;
                        stack.push(next);
                    }
                }
            } else {
                for &prev in &rev[c] {
                    if member[prev] && !seen[prev] {
                        seen[prev] = true;
                        n += 1;
                        stack.push(prev);
                    }
                }
            }
        }
        n
    };

    reach(true) == nodes.len() && reach(false) == nodes.len()
}

/// Cells in `from` that can reach some cell in `targets`, travelling forward
/// through track. Used to check that spurs rejoin the main line.
fn can_reach_main(grid: &Grid, start: CellId, is_main: &[bool]) -> bool {
    let mut seen = vec![false; grid.len()];
    seen[start] = true;
    let mut stack = vec![start];
    while let Some(c) = stack.pop() {
        for (_, n) in grid.exits(c) {
            if is_main[n] {
                return true;
            }
            if !seen[n] {
                seen[n] = true;
                stack.push(n);
            }
        }
    }
    false
}

fn reachable_from_main(grid: &Grid, rev: &[Vec<CellId>], start: CellId, is_main: &[bool]) -> bool {
    let mut seen = vec![false; grid.len()];
    seen[start] = true;
    let mut stack = vec![start];
    while let Some(c) = stack.pop() {
        for &p in &rev[c] {
            if is_main[p] {
                return true;
            }
            if !seen[p] {
                seen[p] = true;
                stack.push(p);
            }
        }
    }
    false
}

/// Check a map. An empty result means it is safe to simulate.
pub fn validate(map: &MapConfig) -> Vec<Problem> {
    let grid = &map.grid;
    let mut problems = Vec::new();

    let cells: Vec<CellId> = (0..grid.len()).filter(|&c| grid.has_track(c)).collect();
    if cells.is_empty() {
        problems.push(Problem::global("map has no track"));
        return problems;
    }

    // Dead ends and exit bits pointing at nothing. A dangling bit is the
    // commonest hand-editing slip: the cell claims an exit the neighbour
    // cannot accept, so routing silently treats it as a wall.
    for &c in &cells {
        if grid.exits(c).is_empty() {
            problems.push(Problem::at(c, "dead end: no legal exit".to_string()));
        }
        let (x, y) = grid.xy(c);
        for d in ALL_DIRS {
            if grid.track[c] & d.bit() == 0 {
                continue;
            }
            if grid.step(c, d).is_none() {
                let (dx, dy) = d.delta();
                problems.push(Problem::at(
                    c,
                    format!(
                        "exit {:?} from ({},{}) dangles: ({},{}) carries no track",
                        d,
                        x,
                        y,
                        x as i32 + dx,
                        y as i32 + dy
                    ),
                ));
            }
        }
    }

    let mut is_parking = vec![false; grid.len()];
    for &p in &map.parking {
        if p >= grid.len() {
            problems.push(Problem::global(&format!(
                "parking cell {p} is outside the grid"
            )));
            continue;
        }
        if !grid.has_track(p) {
            problems.push(Problem::at(p, "parking cell has no track".to_string()));
        }
        is_parking[p] = true;
    }

    let mut is_port = vec![false; grid.len()];
    for m in &map.machines {
        for port in &m.ports {
            if port.cell >= grid.len() {
                problems.push(Problem::global(&format!(
                    "{}: port cell {} is outside the grid",
                    m.name, port.cell
                )));
                continue;
            }
            if !grid.has_track(port.cell) {
                problems.push(Problem::at(
                    port.cell,
                    format!("{}: port has no track", m.name),
                ));
            }
            // A port on a spur is a trap: routing treats spurs as
            // destination-only, so the tool becomes unservable.
            if is_parking[port.cell] {
                problems.push(Problem::at(
                    port.cell,
                    format!("{}: port sits on a parking spur", m.name),
                ));
            }
            is_port[port.cell] = true;
        }
    }

    let rev = predecessors(grid);

    if !strongly_connected(grid, &rev, &cells) {
        problems.push(Problem::global(
            "track graph is not strongly connected: some cells cannot be reached, \
             or cannot get back",
        ));
    }

    // The check that matters. Rails are unidirectional and vehicles cannot
    // overtake, so a spur drawn on the loop turns one parked vehicle into a
    // severed fab.
    let main_line: Vec<CellId> = cells.iter().copied().filter(|&c| !is_parking[c]).collect();
    let mut is_main = vec![false; grid.len()];
    for &c in &main_line {
        is_main[c] = true;
    }
    if !strongly_connected(grid, &rev, &main_line) {
        problems.push(Problem::global(
            "main line is not strongly connected once parking spurs are excluded: \
             a parked vehicle would sever the fab",
        ));
    }

    for &p in &map.parking {
        if p >= grid.len() || !grid.has_track(p) {
            continue;
        }
        if !can_reach_main(grid, p, &is_main) {
            problems.push(Problem::at(
                p,
                "parking spur cannot rejoin the main line".to_string(),
            ));
        }
        if !reachable_from_main(grid, &rev, p, &is_main) {
            problems.push(Problem::at(
                p,
                "parking spur cannot be entered from the main line".to_string(),
            ));
        }
    }

    problems
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::load_map;

    fn demo() -> MapConfig {
        load_map(&std::fs::read_to_string("maps/demo_loop.json").unwrap()).unwrap()
    }

    #[test]
    fn demo_map_is_valid() {
        let problems = validate(&demo());
        assert!(
            problems.is_empty(),
            "demo map should validate, got: {:?}",
            problems.iter().map(|p| p.to_string()).collect::<Vec<_>>()
        );
    }

    #[test]
    fn dangling_exit_bit_is_caught() {
        let mut m = demo();
        // Point a cell at open space.
        let c = (0..m.grid.len()).find(|&c| m.grid.has_track(c)).unwrap();
        m.grid.track[c] = 0x0F;
        let problems = validate(&m);
        assert!(
            problems.iter().any(|p| p.message.contains("dangles")),
            "expected a dangling-exit problem, got {problems:?}"
        );
    }

    #[test]
    fn dead_end_is_caught() {
        let mut m = demo();
        // Leave a spur cell claiming a north exit, then blank the cell it aims
        // at. The spur can no longer be left at all.
        let target = m.parking[0];
        let (x, y) = m.grid.xy(target);
        assert!(y > 0);
        let north = m.grid.idx(x, y - 1);
        m.grid.track[target] = crate::geom::N;
        m.grid.track[north] = 0;
        let problems = validate(&m);
        assert!(
            problems.iter().any(|p| p.cell == Some(target)
                && (p.message.contains("dead end") || p.message.contains("dangles"))),
            "expected a dead-end or dangling problem at the spur, got {problems:?}"
        );
    }

    /// The regression the reference implementation actually hit: parking drawn
    /// on the main loop, which gridlocked the fab at zero lots completed.
    #[test]
    fn parking_on_the_main_loop_is_caught() {
        let mut m = demo();
        // Promote a plain loop cell -- one that is not already a spur and not a
        // port -- into parking. Removing it must break the main line.
        let port_cells: Vec<CellId> = m
            .machines
            .iter()
            .flat_map(|mm| mm.ports.iter().map(|p| p.cell))
            .collect();
        let victim = (0..m.grid.len())
            .find(|&c| {
                m.grid.has_track(c) && !m.parking.contains(&c) && !port_cells.contains(&c)
            })
            .unwrap();
        m.parking.push(victim);
        let problems = validate(&m);
        assert!(
            problems
                .iter()
                .any(|p| p.message.contains("main line is not strongly connected")),
            "parking on the loop should be rejected, got {problems:?}"
        );
    }

    #[test]
    fn port_on_a_spur_is_caught() {
        let mut m = demo();
        let spur = m.parking[0];
        m.machines[1].ports[0].cell = spur;
        let problems = validate(&m);
        assert!(
            problems.iter().any(|p| p.message.contains("parking spur")),
            "expected a port-on-spur problem, got {problems:?}"
        );
    }
}
