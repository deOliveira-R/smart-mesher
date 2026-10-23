//! Mesh database contract (layer "Meshes", docs/design/ir.md §5): a DMPlex-style Hasse diagram.
//!
//! * [`point`]: the unified point space, stratified vertex/edge/face/cell, with oriented
//!   cone/support incidences.
//! * [`traits`]: read-only query traits for consumers ([`MeshView`]) and the edit trait for
//!   meshers/modifiers ([`MeshEdit`]).
//! * [`metrics`], [`validate`]: the declared geometry rule and implementation-independent
//!   invariant checks and content hash.
//! * [`hierarchy`], [`instance`]: several meshes per model (fine/coarse maps) and the
//!   instanced view with deterministic `flatten`.
//! * [`reference`]: in-memory reference implementation.
//!
//! Orientation rule (addendum r2 §7.3): a face's vertex loop defines its normal by the
//! right-hand rule; `left` is the lower cell id; the normal points from `left` into `right`;
//! boundary faces have `right == None` and the normal points outward.

pub mod error;
pub mod hierarchy;
pub mod instance;
pub mod metrics;
pub mod point;
pub mod reference;
pub mod shape;
pub mod traits;
pub mod validate;

pub use error::MeshError;
pub use hierarchy::{FineToCoarse, MeshCollection, MeshHandle};
pub use instance::{CellOrigin, FlattenMap, Instance, InstancedView};
pub use metrics::MeshMetrics;
pub use point::{NONE, Orient, Point, Stratum};
pub use shape::{Basis, CellGeometry, CellShape, FaceShape, NodeRef};
pub use traits::{
    CellAdded, CellSpec, Classification, Cone, FaceLoop, FaceSides, GEOMETRY_RULE_DEFAULT, MeshClassification, MeshEdit,
    MeshGeometry, MeshLabels, MeshTopology, MeshView, Orients, Param, model_dim,
};
pub use validate::{Violation, content_hash, validate};
