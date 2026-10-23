//! Cellular non-manifold topology contract for smart-mesh (radial-edge model; orchestrator-owned).
//!
//! - [`ids`]: stable entity ids and structural handles for uses.
//! - [`model`]: records (`Body > Volume > Shell > FaceUse > Face > Loop > Coedge > Edge > Vertex`).
//! - [`view::TopologyView`]: read-only queries for meshers, semantics and tracking.
//! - [`edit::TopologyEdit`]: Euler-style edit operators with lineage.
//! - [`lineage`]: split / merge / delete records.
//! - [`overlay`]: virtual topology (composite / partition) that never changes host entities.
//! - [`validate()`] and [`view::state_hash`]: implementation-independent checks.
//! - [`mod@reference`]: in-memory reference implementation and fixtures.
//!
//! See `DESIGN.md` for the model, the tolerance decision and the tradeoffs.

pub mod edit;
pub mod error;
pub mod ids;
pub mod lineage;
pub mod model;
pub mod overlay;
pub mod reference;
pub mod validate;
pub mod view;

pub use edit::{EdgeSplit, FaceCut, FaceSplit, OpSummary, SplitAt, TopologyEdit};
pub use error::{TopoError, Violation, ViolationCode};
pub use ids::*;
pub use lineage::{LineageEvent, LineageLog, LineageRecord};
pub use model::*;
pub use overlay::{SimpleOverlay, VirtualEntity, VirtualKind, VirtualOverlay, visible_edges, visible_faces};
pub use reference::RefComplex;
pub use validate::validate;
pub use view::{TopologyView, coedge_ends, edge_use_tail, euler_characteristic, loop_use_edge_uses, state_hash};

#[cfg(test)]
mod tests;
