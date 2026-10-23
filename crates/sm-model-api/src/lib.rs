//! Model-semantics and decomposition layers (docs/design/ir.md §5), frozen contract.
//!
//! Everything here is an **attribute table over topology**: typed tables keyed by
//! [`sm_core::EntityId`], read against an `sm_topo_api::TopologyView` (never mutating it).
//! Topology mutations (cuts) come back as data ([`sm_topo_api::CutOutcome`]) and are applied by
//! [`Model::apply_cut`].
//!
//! | Module | Content |
//! |---|---|
//! | [`color`] | colors with numeric labels, volume coloring (exact partition), face coloring, groups |
//! | [`boundary`] | decks, interface intents, per-face roles, the O(1) [`DeckLookup`] |
//! | [`symmetry`] | declare, [`plan_cut`] / [`Model::apply_cut`], point-group closure |
//! | [`detect`] | symmetry detection from colored invariants, staged verification |
//! | [`unfold`](mod@crate::unfold) | the unfolded model as a view of `(base, elem)` copies |
//! | [`instancing`] | prototypes, instances, lattices, recoloring, [`flatten`] |
//! | [`decomposition`] | pieces (`PartitionCell`) over volumes, split / composite / unsplit |
//! | [`validate`](mod@crate::validate) | the validator ([`validate()`](fn@crate::validate()), [`validate_unfolded`]) |
//!
//! Out of scope by design: material properties and response values (this layer colors; the
//! consumer resolves colors), mixed-material cells, detection of translation symmetry (declared,
//! not detected).
//!
//! Determinism: every map is a `BTreeMap`/`BTreeSet`, every output is in ascending id or
//! documented canonical order, and no randomness is used. Ids minted here
//! (`Color`, `Group`, `Boundary`, `Prototype`, `Lattice`, `Instance`, `PartitionCell`) come from
//! one per-kind allocator and are never reused.

pub mod boundary;
pub mod color;
pub mod decomposition;
pub mod detect;
pub mod error;
pub mod instancing;
pub mod model;
#[cfg(feature = "serde")]
pub(crate) mod serde_pairs;
pub mod symmetry;
pub mod unfold;
pub mod validate;
pub mod xf;

pub use boundary::{BoundaryKind, BoundaryRow, Deck, DeckHit, DeckLookup, DeckSide, FaceBoundary, FaceKindView, FaceRole, InterfaceIntent};
pub use color::{ColorRow, Domain, GroupRow};
pub use decomposition::{CutCells, CutGeometry, CutId, Decomposition, Piece, PieceFace, PieceRef, PieceShape, SplitSides};
pub use detect::{Candidate, CandidateKind, ColorInvariant, DetectionResult, Stage, Verification, detect_symmetry, verify_transform};
pub use error::{ModelError, Result};
pub use instancing::{ColorMap, FlatEntry, FlatModel, Fill, HexOrient, InstanceRow, InstancePath, Instancing, Lattice, Layout, PathOverride, PathStep, Placement, Prototype, Slot, SlotIx, flatten};
pub use model::{IdAlloc, Model};
pub use symmetry::{CutPlan, CutSpec, Keep, Provenance, Symmetry, plan_cut, point_group};
pub use unfold::{CopyRef, Glue, UnfoldSpec, UnfoldedModel, unfold};
pub use validate::{Issue, IssueCode, Report, Severity, validate, validate_unfolded};
pub use xf::{Axis, GROUP_TOL, Plane, Tolerance, Turns, transform_face_moments, transform_moments};
