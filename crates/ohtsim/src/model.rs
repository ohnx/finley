//! Domain model. Everything is referenced by index, never by reference, so the
//! borrow checker stays out of the way and state stays trivially serialisable.

use crate::geom::{CellId, Dir};

pub type VehicleId = usize;
pub type MachineId = usize;
pub type LotId = usize;
pub type JobId = usize;
pub type PortId = usize;

// ---------------------------------------------------------------------------
// Ports and machines
// ---------------------------------------------------------------------------

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum PortKind {
    Input,
    Output,
}

#[derive(Clone, Debug)]
pub struct Port {
    pub kind: PortKind,
    /// The track cell a vehicle occupies while hoisting to/from this port.
    pub cell: CellId,
    pub lot: Option<LotId>,
    /// Set while a vehicle is en route to claim this port, so two vehicles
    /// cannot be dispatched to the same slot.
    pub reserved_by: Option<VehicleId>,
}

impl Port {
    pub fn is_free(&self) -> bool {
        self.lot.is_none() && self.reserved_by.is_none()
    }
}

#[derive(Clone, Debug)]
pub struct Machine {
    pub id: MachineId,
    pub name: String,
    /// Recipe steps name a *kind*, not a specific tool, so the dispatcher gets
    /// to choose between identical tools. That choice is where load balancing
    /// lives.
    pub kind: String,
    pub x: usize,
    pub y: usize,
    pub w: usize,
    pub h: usize,
    pub process_ticks: u32,
    /// How many lots may be in process at once.
    pub capacity: usize,
    pub ports: Vec<Port>,
    /// (lot, ticks remaining)
    pub in_process: Vec<(LotId, u32)>,
    /// Ticks spent with no work available, cumulative. Reported as a metric.
    pub idle_ticks: u64,
    /// Exponentially-weighted recent idleness in [0,1]. This is what dispatch
    /// scores against — cumulative idleness would keep rewarding a tool long
    /// after it stopped being hungry.
    pub starvation: f32,
}

impl Machine {
    pub fn ports_of(&self, kind: PortKind) -> Vec<PortId> {
        self.ports
            .iter()
            .enumerate()
            .filter(|(_, p)| p.kind == kind)
            .map(|(i, _)| i)
            .collect()
    }

    pub fn free_port(&self, kind: PortKind) -> Option<PortId> {
        self.ports
            .iter()
            .enumerate()
            .find(|(_, p)| p.kind == kind && p.is_free())
            .map(|(i, _)| i)
    }

    /// Lots waiting at input ports plus lots in process. The queue-depth signal.
    pub fn load(&self) -> usize {
        let waiting = self
            .ports
            .iter()
            .filter(|p| p.kind == PortKind::Input && p.lot.is_some())
            .count();
        waiting + self.in_process.len()
    }

    pub fn is_source(&self) -> bool {
        self.kind == "source"
    }

    pub fn is_sink(&self) -> bool {
        self.kind == "sink"
    }
}

// ---------------------------------------------------------------------------
// Lots (FOUPs)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq)]
pub enum LotState {
    /// Sitting on a port, waiting to be collected or consumed.
    AtPort(MachineId, PortId),
    InTransit(VehicleId),
    Processing(MachineId),
    Done,
}

#[derive(Clone, Debug)]
pub struct Lot {
    pub id: LotId,
    /// Machine *kinds*, in order. Reentrant flows just repeat a kind.
    pub recipe: Vec<String>,
    /// Which scenario recipe this lot is running. The steps are already in
    /// `recipe`; this identifies *which* product route it is, which the UI
    /// needs to name the remaining steps without shipping strings per frame.
    pub recipe_id: usize,
    pub step: usize,
    pub state: LotState,
    pub created_tick: u64,
    /// When it arrived at its current resting place; drives the wait term.
    pub waiting_since: u64,
    /// Hot lots get a high value here.
    pub priority: f32,
}

impl Lot {
    pub fn next_kind(&self) -> Option<&str> {
        self.recipe.get(self.step).map(|s| s.as_str())
    }

    pub fn wait_ticks(&self, now: u64) -> u64 {
        now.saturating_sub(self.waiting_since)
    }
}

// ---------------------------------------------------------------------------
// Transport jobs
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct Job {
    pub id: JobId,
    pub lot: LotId,
    pub from: (MachineId, PortId),
    /// Chosen at assignment time, not at job creation — picking *which* tool of
    /// the right kind is a dispatch decision.
    pub to: Option<(MachineId, PortId)>,
    pub assigned: Option<VehicleId>,
    pub created_tick: u64,
}

// ---------------------------------------------------------------------------
// Vehicles
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq)]
pub enum VehState {
    Idle,
    ToPickup(JobId),
    Loading(JobId),
    ToDropoff(JobId),
    Unloading(JobId),
    Repositioning,
}

#[derive(Clone, Debug)]
pub struct Vehicle {
    pub id: VehicleId,
    pub cell: CellId,
    pub heading: Dir,
    /// Ticks before this vehicle may act again. Non-zero while traversing a
    /// cell or running a hoist cycle — one primitive covers both.
    pub ready_in: u32,
    pub state: VehState,
    /// Remaining cells to visit, in order. Front is the next hop.
    pub route: Vec<CellId>,
    pub carrying: Option<LotId>,
    /// Ticks spent not Idle. Drives the utilisation metric.
    pub busy_ticks: u64,
    /// Consecutive ticks unable to advance. Feeds livelock detection.
    pub blocked_ticks: u32,
}

impl Vehicle {
    pub fn is_idle(&self) -> bool {
        self.state == VehState::Idle
    }

    pub fn next_cell(&self) -> Option<CellId> {
        self.route.first().copied()
    }
}
