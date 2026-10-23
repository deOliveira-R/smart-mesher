//! [`TopologyEdit`]: Euler-style operators for construct / boolean / imprint / merge.
//!
//! The edit surface is deliberately small. Booleans, webcut and imprint in `sm-model` are
//! compositions of these operators plus geometry computed outside this crate (new vertex
//! positions, chord curves, intersection edges).

use crate::error::TopoError;
use crate::types::{Carrier, CellFlags, Coincidence, CoedgeSpec, CurveCarrier, FaceUseSpec, OpId, Sense, SurfaceCarrier};
use crate::view::TopologyView;
use sm_core::{EntityId, Point3};
use sm_geom_api::Tolerance;

/// Mutating access to a cellular complex. Object-safe.
///
/// # Contracts shared by all operators
/// - **Atomic**: on `Err` nothing has changed.
/// - **Deterministic ids**: new cells take the next unused serial of their kind; when an
///   operator creates several cells it does so in the order documented on the method. Serials
///   are never reused, so ids replay exactly.
/// - **Lineage**: operators that replace identity (`split_*`, `merge_*`, `kill`, `reverse_face`,
///   `set_carrier`) append to the lineage log under the current op (`begin_op`).
/// - **Canonical rows**: after every operator all touched rows are back in canonical order
///   (see [`TopologyView::boundary`]).
pub trait TopologyEdit: TopologyView {
    /// Set the op that subsequent edits are attributed to. `OpId`s should be non-decreasing.
    fn begin_op(&mut self, op: OpId);

    /// Replace the global tolerance policy.
    fn set_model_tolerance(&mut self, tol: Tolerance) -> Result<(), TopoError>;

    // ---- construction (bottom-up) ----

    /// Create a vertex at `p`.
    fn make_vertex(&mut self, p: Point3) -> Result<EntityId, TopoError>;

    /// Create an edge from `start` to `end` (equal for a closed edge). `curve` is its carrier.
    fn make_edge(&mut self, start: EntityId, end: EntityId, curve: Option<CurveCarrier>) -> Result<EntityId, TopoError>;

    /// Create a face from loops of coedges (loop 0 is the outer loop). Each loop must be closed
    /// head-to-tail and is stored rotated to its canonical start. The face's left side is the
    /// side from which the loops run counter-clockwise.
    fn make_face(&mut self, loops: &[&[CoedgeSpec]], surface: Option<SurfaceCarrier>) -> Result<EntityId, TopoError>;

    /// Create a volume from shells of face uses (shell 0 is the outer shell). Fails with
    /// `SideOccupied` if a requested side of a face already has a volume. Shell closure is *not*
    /// required here (volumes may be built incrementally); `validate` checks it.
    fn make_volume(&mut self, shells: &[&[FaceUseSpec]]) -> Result<EntityId, TopoError>;

    /// Create a body grouping `members` (volumes, free faces, wires, points).
    fn make_body(&mut self, members: &[EntityId]) -> Result<EntityId, TopoError>;

    /// Add a face use to shell `ring` of `volume` (e.g. an internal baffle with `Side::Both`).
    fn add_face_use(&mut self, volume: EntityId, ring: u16, use_: FaceUseSpec) -> Result<(), TopoError>;

    /// Remove all uses of `face` from `volume`.
    fn remove_face_use(&mut self, volume: EntityId, face: EntityId) -> Result<(), TopoError>;

    /// Embed a lower-dimensional `child` (wire edge, point, free face) in the interior of
    /// `parent` (face, volume or body).
    fn embed(&mut self, parent: EntityId, child: EntityId) -> Result<(), TopoError>;

    // ---- Euler-style refinement (imprint) ----

    /// Split `edge` at an existing vertex `at` located at curve parameter `param`. Creates two
    /// edges in order `[start..at, at..end]` (edge direction), kills `edge`, and rewrites every
    /// parent row (a coedge is replaced by two in traversal order). Lineage: `Split`.
    fn split_edge(&mut self, edge: EntityId, at: EntityId, param: f64) -> Result<[EntityId; 2], TopoError>;

    /// Split `face` along `chord`, an existing edge joining two vertices of its single loop.
    /// Creates two faces in order `[first, second]`, where `first` is the loop segment that
    /// begins at the earlier loop position of the two chord ends. The chord gains a coedge in
    /// each child. Parent volumes get both children with the original side. The old face dies.
    /// Lineage: `Split`. Faces with more than one loop are `Unsupported`.
    fn split_face(&mut self, face: EntityId, chord: EntityId) -> Result<[EntityId; 2], TopoError>;

    // ---- merge ----

    /// Merge `drop` into `keep` (their points must coincide within the cells' tolerances).
    /// Every parent row now references `keep`. Lineage: `Merged`.
    fn merge_vertices(&mut self, keep: EntityId, drop: EntityId) -> Result<(), TopoError>;

    /// Merge edge `drop` into `keep`. Their end vertices must already be merged.
    /// `relative` is the direction of `drop` relative to `keep` (`Reversed` if it runs the
    /// other way). Lineage: `Merged`.
    fn merge_edges(&mut self, keep: EntityId, drop: EntityId, relative: Sense) -> Result<(), TopoError>;

    /// Merge face `drop` into `keep`. Their loops must consist of the same edges (merge the edges
    /// first). `relative` is the normal of `drop` relative to `keep`. The shared face keeps
    /// `keep`'s id; if two volumes now share it, the face is re-oriented so its left side is the
    /// lower volume id (docs/design/ir.md §6). Records `Coincidence::Merged`. Lineage: `Merged`.
    fn merge_faces(&mut self, keep: EntityId, drop: EntityId, relative: Sense) -> Result<(), TopoError>;

    // ---- modify / delete ----

    /// Reverse a face's orientation: loops run backwards, left and right swap in every parent,
    /// and the surface sense flips. Lineage: `Modified`.
    fn reverse_face(&mut self, face: EntityId) -> Result<(), TopoError>;

    /// Delete a cell that no non-body parent references. Lineage: `Deleted`.
    fn kill(&mut self, cell: EntityId) -> Result<(), TopoError>;

    /// Replace a cell's carrier (must match its rank). Lineage: `Modified`.
    fn set_carrier(&mut self, cell: EntityId, carrier: Carrier) -> Result<(), TopoError>;

    /// Set or clear the per-entity tolerance override (linear, must be positive and finite).
    fn set_tolerance(&mut self, cell: EntityId, tol: Option<f64>) -> Result<(), TopoError>;

    /// Replace flag bits.
    fn set_flags(&mut self, cell: EntityId, flags: CellFlags) -> Result<(), TopoError>;

    /// Record the imprint/merge state of a face pair (not `Merged`, which only `merge_faces`
    /// produces). `area` is the overlap area for `Overlapping`.
    fn record_coincidence(&mut self, a: EntityId, b: EntityId, status: Coincidence, area: Option<f64>) -> Result<(), TopoError>;
}
