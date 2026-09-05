//! Grid, directions, and the directed track graph.
//!
//! A track cell holds the OR of its *allowed exit directions*:
//!   N=1  E=2  S=4  W=8      0 = no track
//! So a cell you may leave heading north or east is 1|2 = 3.
//! The directed graph falls out of this implicitly; nothing else stores edges.

pub const N: u8 = 1;
pub const E: u8 = 2;
pub const S: u8 = 4;
pub const W: u8 = 8;

pub type CellId = usize;

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Dir {
    North,
    East,
    South,
    West,
}

pub const ALL_DIRS: [Dir; 4] = [Dir::North, Dir::East, Dir::South, Dir::West];

impl Dir {
    pub fn bit(self) -> u8 {
        match self {
            Dir::North => N,
            Dir::East => E,
            Dir::South => S,
            Dir::West => W,
        }
    }

    pub fn index(self) -> usize {
        match self {
            Dir::North => 0,
            Dir::East => 1,
            Dir::South => 2,
            Dir::West => 3,
        }
    }

    pub fn from_index(i: usize) -> Dir {
        ALL_DIRS[i % 4]
    }

    /// (dx, dy) with y increasing southward, matching screen coordinates.
    pub fn delta(self) -> (i32, i32) {
        match self {
            Dir::North => (0, -1),
            Dir::East => (1, 0),
            Dir::South => (0, 1),
            Dir::West => (-1, 0),
        }
    }

    pub fn opposite(self) -> Dir {
        match self {
            Dir::North => Dir::South,
            Dir::East => Dir::West,
            Dir::South => Dir::North,
            Dir::West => Dir::East,
        }
    }
}

/// How a movement relates to the vehicle's current heading. Curves cost more
/// because real OHTs slow substantially through them; this is the single knob
/// that makes layout geometry matter.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Manoeuvre {
    Straight,
    Curve,
    Reverse,
}

pub fn manoeuvre(from: Dir, to: Dir) -> Manoeuvre {
    if from == to {
        Manoeuvre::Straight
    } else if from.opposite() == to {
        Manoeuvre::Reverse
    } else {
        Manoeuvre::Curve
    }
}

#[derive(Clone, Debug)]
pub struct Grid {
    pub w: usize,
    pub h: usize,
    /// Row-major, length w*h.
    pub track: Vec<u8>,
}

impl Grid {
    pub fn new(w: usize, h: usize) -> Grid {
        Grid {
            w,
            h,
            track: vec![0; w * h],
        }
    }

    pub fn idx(&self, x: usize, y: usize) -> CellId {
        y * self.w + x
    }

    pub fn xy(&self, c: CellId) -> (usize, usize) {
        (c % self.w, c / self.w)
    }

    pub fn len(&self) -> usize {
        self.w * self.h
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn has_track(&self, c: CellId) -> bool {
        c < self.track.len() && self.track[c] != 0
    }

    /// The cell reached by leaving `c` in direction `d`, if that exit is
    /// allowed and the destination carries track.
    pub fn step(&self, c: CellId, d: Dir) -> Option<CellId> {
        if !self.has_track(c) || (self.track[c] & d.bit()) == 0 {
            return None;
        }
        let (x, y) = self.xy(c);
        let (dx, dy) = d.delta();
        let nx = x as i32 + dx;
        let ny = y as i32 + dy;
        if nx < 0 || ny < 0 || nx >= self.w as i32 || ny >= self.h as i32 {
            return None;
        }
        let nc = self.idx(nx as usize, ny as usize);
        if self.has_track(nc) {
            Some(nc)
        } else {
            None
        }
    }

    /// All legal exits from a cell as (direction, destination).
    pub fn exits(&self, c: CellId) -> Vec<(Dir, CellId)> {
        let mut out = Vec::with_capacity(4);
        for d in ALL_DIRS {
            if let Some(n) = self.step(c, d) {
                out.push((d, n));
            }
        }
        out
    }

    /// Number of distinct exits. >1 means a diverge point.
    pub fn out_degree(&self, c: CellId) -> usize {
        self.exits(c).len()
    }
}
