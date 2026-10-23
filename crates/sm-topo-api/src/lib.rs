//! The cellular non-manifold complex contract for smart-mesh.
//!
//! Model: cells of five ranks ([`Dim`]) identified by stable `EntityId`s, **one** oriented
//! incidence relation with `Left`/`Right`/`Both` side tags ([`Incidence`], [`Side`]), geometry
//! attached as carriers resolved through `sm_geom_api::GeometryStore`, a lineage log, and a
//! sparse tolerance model. Loops, coedges, shells, face uses and face sides are *derived views*
//! ([`TopologyViewExt`]), not stored objects.
//!
//! - Read: [`TopologyView`] (object-safe). Write: [`TopologyEdit`] (Euler-style operators).
//! - Reference implementation: [`CellComplex`]. Overlay hook: [`TopologyOverlay`] / [`Overlay`].
//! - Checks: [`validate()`], [`validate_geometry()`]. Test models: [`fixtures`].
//!
//! See `DESIGN.md` in this crate for the rationale.

pub mod complex;
pub mod derived;
pub mod edit;
pub mod error;
pub mod fixtures;
pub mod overlay;
pub mod types;
pub mod validate;
pub mod view;

pub use complex::CellComplex;
pub use derived::{Coedge, FaceSides, FaceUse, Loop, RadialUse, Shell, TopologyViewExt};
pub use edit::TopologyEdit;
pub use error::TopoError;
pub use overlay::{Overlay, TopologyOverlay};
pub use types::{
    Carrier, CellFlags, Coincidence, CoedgeSpec, CurveCarrier, Dim, FacePair, FaceUseSpec, Incidence, LineageKind, LineageRecord, OpId, Region, Sense, Side,
    SurfaceCarrier,
};
pub use validate::{Rule, Violation, validate, validate_geometry};
pub use view::TopologyView;
