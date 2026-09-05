//! The configuration space.
//!
//! Policies are weighted scoring functions over *named criteria*, the way a fab
//! IE tunes an MCS. No policy is a preset the player selects: "nearest vehicle
//! first" is simply what you get when `distance` dominates. Every term names
//! something recognisable, and no simulation internals leak through.

/// Scoring a (job, vehicle, destination) triple. Lower score wins.
#[derive(Clone, Debug)]
pub struct DispatchWeights {
    /// Route cost from the vehicle to the pickup port.
    pub travel_to_pickup: f32,
    /// How long the lot has been sitting. Raise to reduce tail latency.
    pub lot_wait: f32,
    /// Multiplies the lot's own priority field. Raise for hot-lot bias.
    pub lot_priority: f32,
    /// Favours destination tools that have been starved. Raise this and
    /// vehicles stampede toward the hungriest tool — which congests the route
    /// to it, which starves it further.
    pub dest_starvation: f32,
    /// Penalises destination tools that already have a queue.
    pub dest_queue: f32,
    /// Penalises delivery routes running through congested track.
    pub dest_congestion: f32,
    /// Steps remaining in the recipe; favours nearly-finished lots.
    pub steps_remaining: f32,
}

impl Default for DispatchWeights {
    fn default() -> Self {
        DispatchWeights {
            travel_to_pickup: 1.0,
            lot_wait: 0.05,
            lot_priority: 10.0,
            dest_starvation: 0.02,
            dest_queue: 4.0,
            dest_congestion: 1.0,
            steps_remaining: 0.0,
        }
    }
}

/// Edge cost when routing. Turn the congestion term up too far and you get
/// oscillation: everyone reroutes onto the same alternate at once.
#[derive(Clone, Debug)]
pub struct RouteWeights {
    /// Cost per cell traversed.
    pub length: f32,
    /// Extra cost for a curve.
    pub curve: f32,
    /// Multiplies the cell's congestion estimate.
    pub congestion: f32,
}

impl Default for RouteWeights {
    fn default() -> Self {
        RouteWeights {
            length: 1.0,
            curve: 2.0,
            congestion: 3.0,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum IdleMode {
    /// Stop where you are. Simple, and blocks the main loop.
    StayPut,
    /// Head for the nearest declared parking cell.
    NearestPark,
    /// Head for parking near whichever tool is most starved.
    Preposition,
}

#[derive(Clone, Debug)]
pub struct IdlePolicy {
    pub mode: IdleMode,
    /// Ticks idle before repositioning kicks in. Prevents twitchy behaviour.
    pub dwell_before_move: u32,
}

impl Default for IdlePolicy {
    fn default() -> Self {
        IdlePolicy {
            mode: IdleMode::NearestPark,
            dwell_before_move: 5,
        }
    }
}

/// Physical constants. Not player-tunable in the game, but useful for
/// experiments and for matching a real fab's numbers.
#[derive(Clone, Debug)]
pub struct Kinematics {
    pub straight_ticks: u32,
    pub curve_ticks: u32,
    pub hoist_ticks: u32,
}

impl Default for Kinematics {
    fn default() -> Self {
        Kinematics {
            straight_ticks: 1,
            curve_ticks: 3,
            hoist_ticks: 20,
        }
    }
}

/// A conditional profile: when `trigger` holds, swap in these weights. This is
/// how fabs get policy structure without a scripting language.
#[derive(Clone, Debug)]
pub struct Profile {
    pub name: String,
    pub trigger: Trigger,
    pub dispatch: DispatchWeights,
    pub route: RouteWeights,
}

#[derive(Clone, Debug)]
pub enum Trigger {
    /// The fallback profile. Always matches; lowest precedence.
    Always,
    /// Any machine of this kind has load >= n.
    QueueDepthAbove { kind: String, n: usize },
    /// Any machine of this kind has been idle for >= n consecutive ticks.
    StarvationAbove { kind: String, ticks: u64 },
    /// More than n jobs are waiting unassigned.
    BacklogAbove { n: usize },
}

#[derive(Clone, Debug)]
pub struct Policy {
    pub profiles: Vec<Profile>,
    pub idle: IdlePolicy,
    pub kinematics: Kinematics,
    /// Decay factor for the per-cell congestion EMA, in [0,1).
    pub congestion_decay: f32,
    /// Consecutive blocked ticks before a vehicle is reported as stuck.
    pub stuck_threshold: u32,
}

impl Default for Policy {
    fn default() -> Self {
        Policy {
            profiles: vec![Profile {
                name: "default".to_string(),
                trigger: Trigger::Always,
                dispatch: DispatchWeights::default(),
                route: RouteWeights::default(),
            }],
            idle: IdlePolicy::default(),
            kinematics: Kinematics::default(),
            congestion_decay: 0.98,
            stuck_threshold: 200,
        }
    }
}
