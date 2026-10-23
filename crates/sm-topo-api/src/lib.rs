//! Cellular non-manifold topology contract for smart-mesh (OCCT-like).
//!
//! Model: shared entity handles ([`VertexId`] .. [`BodyId`]) + orientation ([`Sense`]) +
//! location ([`Location`]) + explicit use records ([`Coedge`], [`FaceUse`]) + per-entity
//! tolerances. [`TopologyView`] is the read-only query trait; [`TopologyEdit`] the Euler-style
//! edit trait; [`Complex`] the reference in-memory implementor; [`VirtualTopology`] the
//! composite/partition overlay hook. See `DESIGN.md` in this crate.

pub mod builders;
pub mod complex;
pub mod data;
pub mod edit;
pub mod error;
pub mod ids;
pub mod lineage;
pub mod location;
pub mod overlay;
pub mod validate;
pub mod view;

pub use complex::Complex;
pub use data::*;
pub use edit::{EdgeSpec, FaceSpec, SplitFaceSpec, TopoResult, TopologyEdit, WireOwner};
pub use error::TopoError;
pub use ids::{BodyId, EdgeId, FaceId, IdAllocator, InstanceId, OpId, Sense, VertexId, VolumeId};
pub use lineage::{LineageKind, LineageLog, LineageRecord};
pub use location::{InstanceData, Location, ShapeRef};
pub use overlay::{OverlayError, OverlayTable, VirtualKind, VirtualPiece, VirtualTopology, WithOverlay};
pub use validate::{TopoIssue, check_edge_geometry, topology_hash, validate};
pub use view::TopologyView;
