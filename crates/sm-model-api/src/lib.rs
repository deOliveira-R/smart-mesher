//! Model-semantics and decomposition layers (ir.md section 5), as a selection algebra.
//!
//! Everything is a typed [`Selection`] with a [`Lifetime`] and invariants: colors, boundary
//! kinds, groups and decomposition pieces. Topology is reached only through [`TopologyView`].
//! See `DESIGN.md` for the model, tradeoffs and requirement mapping.

pub mod action;
pub mod boundary;
pub mod decomposition;
pub mod error;
pub mod hash;
pub mod instancing;
#[cfg(any(test, feature = "mock"))]
pub mod mock;
pub mod model;
pub mod selection;
pub mod symmetry;
pub mod topology;
pub mod validate;

#[cfg(test)]
mod tests;

pub use action::{DeckKind, Generator, PathStep, Placement, Plane, Slot};
pub use boundary::{BoundaryKind, Conformity, DeckPairing, DeckSide, InterfaceIntent, KindFilter, KindSet};
pub use decomposition::{CutSurface, Cutter, Decomposition, Part, Piece, PieceFace, Side, SweepHint};
pub use error::ModelError;
pub use instancing::{ColorMap, Fill, FlatInstance, HexOrientation, Instancing, Lattice, LatticeGeometry, Target};
pub use model::{GroupMembers, Model, Tolerance};
pub use selection::{Face, Lifetime, SelKind, Selection, Selector, Volume};
pub use symmetry::{CutSpec, DetectOptions, KeepSide, UnfoldOptions, Verification};
pub use topology::{FaceProps, FaceSides, MassProps, TopologyView};
pub use validate::{IssueCode, Report, Severity};
