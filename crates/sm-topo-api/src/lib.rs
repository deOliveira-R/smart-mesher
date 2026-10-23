//! The cellular non-manifold complex contract for smart-mesh (frozen after W0c).
//!
//! Model: cells of five ranks ([`Dim`]) identified by stable `EntityId`s, **one** oriented
//! incidence relation with `Left`/`Right`/`Both` side tags ([`Incidence`], [`Side`]), geometry
//! attached as carriers resolved through `sm_geom_api::GeomStore`, an indexed lineage log, and a
//! global tolerance with sparse per-entity overrides. Loops, coedges, shells, face uses, face
//! sides and radial rings are *derived views* ([`TopologyViewExt`], [`TopologyView::radial_uses`]),
//! not stored objects.
//!
//! - Read: [`TopologyView`] (object-safe), including the geometric queries the model layer
//!   needs (`face_moments`, `volume_moments`, `face_edges`, `volume_faces`, `face_ordinal`,
//!   `coincident_faces`).
//! - Write: [`TopologyEdit`] (Euler-style operators inside `begin_op`/`end_op`).
//! - Reference implementation: [`CellComplex`]. Decomposition layer: [`Overlay`].
//! - Checks: [`validate()`], [`validate_geometry()`], [`euler_characteristic()`]. Test models:
//!   [`fixtures`].
//!
//! Instancing (prototypes, lattices, locations) is **not** here: it lives in `sm-model-api`.
//!
//! See `DESIGN.md` in this crate for the rationale.

pub mod complex;
pub mod derived;
pub mod edit;
pub mod error;
pub mod fixtures;
pub mod geometry;
pub mod lineage;
pub mod overlay;
pub mod types;
pub mod validate;
pub mod view;

pub use complex::CellComplex;
pub use derived::{Coedge, FaceClass, FaceSides, FaceUse, Loop, Shell, TopologyViewExt};
pub use edit::TopologyEdit;
pub use error::{QueryError, TopoError};
pub use geometry::{FaceMoments, Moments, PointClass};
pub use lineage::LineageLog;
pub use overlay::{Overlay, VirtualCut, VirtualOverlay};
pub use types::{
    CapFace, Carrier, CellFlags, CoedgeKey, CoedgeKind, CoedgeSpec, Coincidence, CornerKey, CurveCarrier, CutOutcome, Dim, EntityState, FaceCut, FacePair, FaceUseSpec, Incidence,
    InnerLoopAssign, Lineage, LineageEvent, LineageKind, LineageRecord, ModKind, OpId, OpSummary, Region, Sense, ShellAssign, ShellKind, ShellSpec, Side, SurfaceCarrier,
};
pub use validate::{Rule, Severity, TopoIssue, euler_characteristic, validate, validate_geometry};
pub use view::{EdgeUse, RADIAL_ANGLE_STEP, TopologyView};
