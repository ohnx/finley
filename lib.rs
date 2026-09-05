//! Headless OHT / swarm-robotics simulation core.
//!
//! No rendering dependencies anywhere in this crate. The intended shape is:
//!   - native build: batch policy sweeps, thousands of runs
//!   - wasm build:   the same core behind a thin `#[wasm_bindgen]` shim
//!
//! For the wasm layer, do NOT serialise `Snapshot` to JSON each frame — that
//! will eat most of the speed. Expose pointers into the snapshot's flat arrays
//! and read wasm linear memory from JS as typed arrays instead.

pub mod config;
pub mod dispatch;
pub mod geom;
pub mod json;
pub mod metrics;
pub mod model;
pub mod movement;
pub mod policy;
pub mod routing;
pub mod world;

pub use config::{load_map, load_policy, load_scenario, MapConfig, RecipeSpec, ScenarioConfig};
pub use geom::{CellId, Dir, Grid};
pub use metrics::Metrics;
pub use policy::{DispatchWeights, IdleMode, Policy, RouteWeights};
pub use world::{Snapshot, World};
