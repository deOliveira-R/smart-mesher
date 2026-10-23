//! [`TopologyEdit`]: Euler-style operators for construct / boolean / imprint / merge / webcut.
//!
//! The edit surface is deliberately small. Booleans, webcut and imprint in `sm-model` are
//! compositions of these operators plus geometry computed outside this crate (new vertex
//! positions, chord curves, intersection edges), inserted through [`TopologyEdit::shapes_mut`].

use crate::error::TopoError;
use crate::types::{Carrier, Coincidence, CoedgeKey, CoedgeSpec, CurveCarrier, FaceCut, FaceUseSpec, InnerLoopAssign, OpId, OpSummary, Sense, ShellAssign, ShellSpec, Side, SurfaceCarrier};
use crate::view::TopologyView;
use sm_core::{EntityId, Point3};
use sm_geom_api::{CurveDef, GeomStoreMut, ModelTolerance, PCurve, SurfaceDef};
use smallvec::SmallVec;

/// Mutating access to a cellular complex. Object-safe.
///
/// # Contracts shared by all operators
/// - **Inside an operation**: every edit needs `begin_op` .. `end_op` (`NoOpenOp` otherwise).
///   Operation ids strictly increase; they are the caller's (the op engine's) numbers, so
///   lineage is replay-stable.
/// - **Atomic**: on `Err` the operator has changed nothing. `abort_op` undoes the whole
///   operation; `end_op` validates the touched cells, normalizes the orientation rule and aborts
///   if the result is invalid.
/// - **Deterministic ids**: new cells take the next unused serial of their kind, in the order
///   documented on the method. Serials are never reused, so ids replay exactly.
/// - **Lineage**: operators that replace identity or change an entity in place append to the
///   lineage log under the current op.
/// - **Lowest id survives**: `merge_*(keep, drop, ..)` require `keep < drop`; callers process
///   candidate pairs in ascending `(min id, max id)` order, so the result does not depend on
///   traversal order.
/// - **Canonical rows**: after every operator all touched rows are back in canonical order.
pub trait TopologyEdit: TopologyView {
    /// Open operation `op`. Fails if one is open or `op` does not exceed the previous one.
    fn begin_op(&mut self, op: OpId) -> Result<(), TopoError>;

    /// Commit the open operation. Re-applies the orientation rule to touched faces, runs the
    /// cheap invariant checks (local to the touched cells, not a full `validate`), rebuilds the
    /// radial rings of changed edges, and returns a summary. If a check fails the operation is
    /// rolled back and [`TopoError::OpInvalid`] is returned.
    fn end_op(&mut self) -> Result<OpSummary, TopoError>;

    /// Undo everything since `begin_op`, restoring the exact state (including the hash).
    fn abort_op(&mut self);

    /// The geometry store, for inserting curves and surfaces (tool geometry, chords, caps).
    /// Additions are rolled back with the operation.
    fn shapes_mut(&mut self) -> &mut dyn GeomStoreMut;

    /// Replace the global tolerance policy.
    fn set_model_tolerance(&mut self, tol: ModelTolerance) -> Result<(), TopoError>;

    // ---- construction (bottom-up) ----

    /// Create a vertex at `p`.
    fn make_vertex(&mut self, p: Point3) -> Result<EntityId, TopoError>;

    /// Create an edge from `start` to `end` (equal for a closed edge). `curve` is its carrier.
    fn make_edge(&mut self, start: EntityId, end: EntityId, curve: Option<CurveCarrier>) -> Result<EntityId, TopoError>;

    /// Create a face from loops of coedges (loop 0 is the outer loop). Each loop must be closed
    /// head-to-tail and is stored rotated to its canonical start; an `(edge, side)` may appear
    /// at most once per face. The loops run counter-clockwise seen from the face's **right**
    /// side, so the normal points from left to right (right-hand rule on the loop).
    fn make_face(&mut self, loops: &[&[CoedgeSpec]], surface: Option<SurfaceCarrier>) -> Result<EntityId, TopoError>;

    /// Create a volume from shells of face uses (shell 0 must be `Outer`). Fails with
    /// `SideOccupied` if a requested side of a face already has a volume, and with
    /// `BaffleInClosedShell` for a `Both` use outside an `Internal` shell. Shell closure is *not*
    /// required here (volumes may be built incrementally); `end_op` and `validate` check it.
    /// With no shells the volume has no topological boundary (e.g. a CSG volume).
    fn make_volume(&mut self, shells: &[ShellSpec<'_>]) -> Result<EntityId, TopoError>;

    /// Create a body grouping `members` (volumes, free faces, wires, points).
    fn make_body(&mut self, members: &[EntityId]) -> Result<EntityId, TopoError>;

    /// Append a shell to a volume; returns its ring index.
    fn add_shell(&mut self, volume: EntityId, shell: ShellSpec<'_>) -> Result<u16, TopoError>;

    /// Add a face use to an existing shell `ring` of `volume` (a baffle `Both` needs an
    /// `Internal` shell).
    fn add_face_use(&mut self, volume: EntityId, ring: u16, use_: FaceUseSpec) -> Result<(), TopoError>;

    /// Remove the use of `face` on `side` from `volume` (`Side::Both` removes every use). A
    /// baffle use loses one side when `Left` or `Right` is given. Re-applies the orientation
    /// rule to the face (a `reverse_face` is logged if it had to flip).
    fn remove_face_use(&mut self, volume: EntityId, face: EntityId, side: Side) -> Result<(), TopoError>;

    /// Embed a lower-dimensional `child` (wire edge, point, free face) in the interior of
    /// `parent` (face, volume or body).
    fn embed(&mut self, parent: EntityId, child: EntityId) -> Result<(), TopoError>;

    /// Inverse of [`Self::embed`].
    fn unembed(&mut self, parent: EntityId, child: EntityId) -> Result<(), TopoError>;

    // ---- Euler-style refinement (imprint, webcut) ----

    /// Split `edge` at an existing vertex `at` located at curve parameter `param`. Creates two
    /// edges in order `[start..at, at..end]` (edge direction), retires `edge`, and rewrites every
    /// parent row (a coedge is replaced by two in traversal order). Pcurves are copied to both
    /// halves (they share the curve parameter). Lineage: `Split`.
    fn split_edge(&mut self, edge: EntityId, at: EntityId, param: f64) -> Result<[EntityId; 2], TopoError>;

    /// Split `face` by `cut` (see [`FaceCut`]). Creates two faces in the order documented on
    /// each cut kind, gives both the original carrier, tolerance and flags, replaces the face in
    /// every parent with both children on the original side, moves pcurves and coincidence
    /// records to the children, and retires the face. Existing inner loops go to the children
    /// as `inner` says. Lineage: `Split`.
    fn split_face(&mut self, face: EntityId, cut: FaceCut<'_>, inner: InnerLoopAssign<'_>) -> Result<[EntityId; 2], TopoError>;

    /// Split `volume` along `cut` (free faces already separating it, e.g. the cap built from
    /// the chords of a webcut). The faces of the outer shell are partitioned by flood fill across
    /// shared edges, not crossing the cut faces' boundary edges; there must be exactly two
    /// parts. Children are `[part containing the lowest face id, other part]`; the cut faces
    /// are oriented so the first child is on their left (a `reverse_face` is logged if needed)
    /// and become interface faces. Non-outer shells go to the children as `assign` says. The
    /// volume is retired with `Split` lineage and replaced in bodies by both children.
    fn split_volume(&mut self, volume: EntityId, cut: &[EntityId], assign: ShellAssign<'_>) -> Result<[EntityId; 2], TopoError>;

    /// Add an inner loop to `face` (MEKR): returns its ring index. The loop is not required to
    /// lie inside the face geometrically; closure is checked at `end_op`.
    fn make_inner_loop(&mut self, face: EntityId, lp: &[CoedgeSpec]) -> Result<u16, TopoError>;

    /// Remove inner loop `ring` (KEMR); higher rings shift down by one. Ring 0 cannot be removed.
    fn kill_inner_loop(&mut self, face: EntityId, ring: u16) -> Result<(), TopoError>;

    // ---- merge ----

    /// Merge `drop` into `keep` (`keep < drop`). The points must coincide within the cells'
    /// tolerances (`ModelTolerance::coincident`), or within `max_gap` if given (an op-level
    /// Cubit-style merge tolerance, at most `linear_max`; the following edge and face merges
    /// of the operation inherit it). The survivor's tolerance grows to `max(own, gap + other)`
    /// when the gap exceeds the hash quantum. Lineage: `Merged { into: keep, gap }` on `drop`.
    fn merge_vertices(&mut self, keep: EntityId, drop: EntityId, max_gap: Option<f64>) -> Result<(), TopoError>;

    /// Merge edge `drop` into `keep`. Their end vertices must already be merged. `relative` is
    /// the direction of `drop` relative to `keep`. The sampled curve gap must be within
    /// tolerance or the inherited `max_gap`.
    fn merge_edges(&mut self, keep: EntityId, drop: EntityId, relative: Sense) -> Result<(), TopoError>;

    /// Merge face `drop` into `keep`. Their loops must correspond under `relative` (the normal
    /// of `drop` relative to `keep`): same rings, same coedges, edges merged first. Embedded
    /// members are carried over, the survivor's tolerance becomes the maximum of the two, and
    /// the measured gap is recorded. If two volumes now share the face it is re-oriented so
    /// its left side is the lower volume id. Records `Coincidence::Merged`.
    fn merge_faces(&mut self, keep: EntityId, drop: EntityId, relative: Sense) -> Result<(), TopoError>;

    // ---- modify / delete ----

    /// Reverse a face's orientation. Lineage: `Modified(Reversed)`.
    fn reverse_face(&mut self, face: EntityId) -> Result<(), TopoError>;

    /// Delete a cell that no non-body parent references. Lineage: `Deleted`.
    fn kill(&mut self, cell: EntityId) -> Result<(), TopoError>;

    /// Delete `cell` and every lower cell it leaves orphaned (no live non-body parent left),
    /// top-down: faces, then edges, then vertices, each ascending. Returns the killed cells in
    /// that order, `cell` first. Lineage: `Deleted` for each.
    fn kill_cascade(&mut self, cell: EntityId) -> Result<SmallVec<[EntityId; 16]>, TopoError>;

    /// Move a vertex. Lineage: `Modified(Point)`.
    fn set_vertex_point(&mut self, v: EntityId, p: Point3) -> Result<(), TopoError>;

    /// Replace a cell's carrier (must match its rank). Lineage: `Modified(Carrier)`.
    fn set_carrier(&mut self, cell: EntityId, carrier: Carrier) -> Result<(), TopoError>;

    /// Set or clear the pcurve of a coedge (which must exist in the face).
    fn set_pcurve(&mut self, key: CoedgeKey, pcurve: Option<PCurve>) -> Result<(), TopoError>;

    /// Replace a curve definition in the store (snap/heal). Lineage: `Modified(Curve)` on the
    /// curve handle.
    fn replace_curve(&mut self, id: EntityId, def: CurveDef) -> Result<(), TopoError>;

    /// Replace a surface definition in the store. Lineage: `Modified(Surface)` on the handle.
    fn replace_surface(&mut self, id: EntityId, def: SurfaceDef) -> Result<(), TopoError>;

    /// Raise the per-entity linear tolerance of `cell` to `tol`. Tolerances only grow and never
    /// exceed `linear_max`. Lineage: `Modified(Tolerance)`.
    fn raise_tolerance(&mut self, cell: EntityId, tol: f64) -> Result<(), TopoError>;

    /// Replace flag bits.
    fn set_flags(&mut self, cell: EntityId, flags: crate::types::CellFlags) -> Result<(), TopoError>;

    /// Record the imprint/merge state of a face pair (not `Merged`, which only `merge_faces`
    /// produces). Lineage: `Modified(Coincidence)` on the lower face.
    fn record_coincidence(&mut self, a: EntityId, b: EntityId, status: Coincidence) -> Result<(), TopoError>;

    // ---- lineage for composite operations ----

    /// Declare that `parent` (already retired in this operation) was replaced by `children`
    /// (live). Allowed only inside an operation and only for a parent retired in it; the
    /// parent's state becomes `Split(op)`.
    fn record_split(&mut self, parent: EntityId, children: &[EntityId]) -> Result<(), TopoError>;

    /// Record that face `cell` was generated on tool surface `from_surface`, so "the new cut
    /// face" can be found by a query.
    fn record_generated(&mut self, cell: EntityId, from_surface: EntityId) -> Result<(), TopoError>;

    /// Record that `cell` was created by a composite operation.
    fn record_created(&mut self, cell: EntityId) -> Result<(), TopoError>;
}
