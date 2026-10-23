//! The edit trait: Euler-style construct/split/merge operators.
//!
//! All edits happen inside an operation (`begin_op` .. `end_op`) tagged with a caller-supplied
//! [`OpId`], so every created, split, merged or deleted entity lands in the lineage log under
//! that op. Each operator is atomic (on `Err` nothing changed) and deterministic (ids come
//! from [`crate::IdAllocator`]; multi-entity results are created in the order documented).
//!
//! Operator pairs (inverse of each other): `split_edge`/`merge_edges` (the Euler operators
//! MEV/KEV on a coedge ring), `split_face`/`merge_faces` (MEF/KEF), `merge_vertices`.
//! Imprint and boolean algorithms are built from these by `sm-topo`; this trait fixes only
//! the bookkeeping they must go through.

use crate::data::*;
use crate::error::TopoError;
use crate::ids::*;
use crate::location::ShapeRef;
use crate::view::TopologyView;
use sm_core::{EntityId, Point3};
use sm_geom_api::{CurveId, Interval, PCurve, SurfaceId};

/// Result alias for edits.
pub type TopoResult<T> = Result<T, TopoError>;

/// Parameters of a new edge.
#[derive(Clone, Debug, PartialEq)]
pub struct EdgeSpec {
    /// 3D curve (`None` for degenerate edges).
    pub curve: Option<CurveId>,
    /// Parameter range from `start` to `end`.
    pub range: Interval,
    /// Start vertex.
    pub start: VertexId,
    /// End vertex (== `start` for closed edges).
    pub end: VertexId,
    /// Tolerance.
    pub tol: f64,
    /// Collapsed-to-a-point edge.
    pub degenerate: bool,
}

/// Parameters of a new face. Loops must be closed; the first must be `Outer`, the rest `Inner`.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceSpec {
    /// Supporting surface.
    pub surface: Option<SurfaceId>,
    /// Normal sense relative to the surface.
    pub surface_sense: Sense,
    /// Tolerance.
    pub tol: f64,
    /// Loops (first outer).
    pub loops: Vec<LoopData>,
    /// Embedded edges.
    pub embedded_edges: Vec<EdgeId>,
}

/// Parameters of [`TopologyEdit::split_face`].
#[derive(Clone, Debug, PartialEq)]
pub struct SplitFaceSpec {
    /// Face to split; must have exactly one (outer) loop.
    pub face: FaceId,
    /// Existing edge running from the start vertex of coedge `a` to the start vertex of coedge `b`.
    pub chord: EdgeId,
    /// Position of the coedge in the outer loop whose start vertex is the chord start.
    pub a: usize,
    /// Position of the coedge whose start vertex is the chord end.
    pub b: usize,
    /// Pcurves of the chord in `[child0, child1]` (child0 uses the chord `Reversed`, child1 `Forward`).
    pub pcurves: [Option<PCurve>; 2],
}

/// Where a wire is attached.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WireOwner {
    /// Embedded in a volume.
    Volume(VolumeId),
    /// Free in a body.
    Body(BodyId),
}

/// Editable topology. Extends the query trait, so an editor can also be passed as a view.
pub trait TopologyEdit: TopologyView {
    /// Open operation `op`. Fails if one is already open.
    fn begin_op(&mut self, op: OpId) -> TopoResult<()>;
    /// Close the current operation and refresh derived indices.
    fn end_op(&mut self) -> TopoResult<()>;

    /// New vertex. `tol` is raised to the policy minimum; above the policy maximum is an error.
    fn make_vertex(&mut self, point: Point3, tol: f64) -> TopoResult<VertexId>;
    /// New edge between existing vertices.
    fn make_edge(&mut self, spec: EdgeSpec) -> TopoResult<EdgeId>;
    /// New face from closed loops of existing edges.
    fn make_face(&mut self, spec: FaceSpec) -> TopoResult<FaceId>;
    /// New empty body.
    fn make_body(&mut self) -> TopoResult<BodyId>;
    /// New volume in `body` bounded by `shells` (first outer). Each use's face may then have at
    /// most one other use, of opposite sense.
    fn make_volume(&mut self, body: BodyId, shells: Vec<ShellData>) -> TopoResult<VolumeId>;
    /// Attach a wire.
    fn add_wire(&mut self, owner: WireOwner, coedges: Vec<Coedge>) -> TopoResult<()>;
    /// Insert an internal face (baffle) into the outer shell of `volume`: two uses of `face`,
    /// `Forward` then `Reversed`. The face must have no other use.
    fn add_baffle(&mut self, volume: VolumeId, face: FaceId) -> TopoResult<()>;
    /// New instance of a shared definition.
    fn make_instance(&mut self, target: ShapeRef, parent: Option<InstanceId>) -> TopoResult<InstanceId>;

    /// Split `edge` at `at` (parameter `t`, strictly inside the range). The edge is retired and
    /// replaced by `[start..at, at..end]` (children in that order) in every ring using it,
    /// respecting each coedge's sense. Pcurves are copied (same-parameter).
    fn split_edge(&mut self, edge: EdgeId, at: VertexId, t: f64) -> TopoResult<[EdgeId; 2]>;
    /// Split a face with an existing chord edge (see [`SplitFaceSpec`]). The face is retired;
    /// children are `[child0, child1]`, child0 holding coedges `a..b` (cyclic). Uses of the face
    /// in shells are replaced in place by uses of both children with the same sense.
    fn split_face(&mut self, spec: SplitFaceSpec) -> TopoResult<[FaceId; 2]>;

    /// Merge two vertices. The survivor is the lower id; its tolerance grows to cover the gap.
    fn merge_vertices(&mut self, a: VertexId, b: VertexId) -> TopoResult<VertexId>;
    /// Merge two edges with identical (possibly swapped) end vertices. Survivor is the lower id.
    /// Coedges of the absorbed edge are re-pointed (sense flipped if the edges were opposed) and
    /// lose their pcurves.
    fn merge_edges(&mut self, a: EdgeId, b: EdgeId) -> TopoResult<EdgeId>;
    /// Merge two faces whose boundary coedge multisets coincide (same or all-reversed). Survivor
    /// is the lower id; the absorbed face's shell uses are re-pointed with sense flipped if the
    /// normals were opposed. Records the pair as `Merged`.
    fn merge_faces(&mut self, a: FaceId, b: FaceId) -> TopoResult<FaceId>;

    /// Record the imprint status of a face pair (anything but `Merged`, which only `merge_faces` sets).
    fn set_coincidence(&mut self, a: FaceId, b: FaceId, status: ImprintStatus, overlap_area: Option<f64>) -> TopoResult<()>;
    /// Grow the tolerance of a vertex, edge or face (never shrinks, capped by the policy).
    fn raise_tolerance(&mut self, id: EntityId, tol: f64) -> TopoResult<()>;
    /// Record that `into` were generated from the surviving entity `from` (extrude, sweep).
    fn record_generated(&mut self, from: EntityId, into: &[EntityId]) -> TopoResult<()>;

    /// Delete a face, removing its shell uses.
    fn delete_face(&mut self, f: FaceId) -> TopoResult<()>;
    /// Delete an edge that no ring uses.
    fn delete_edge(&mut self, e: EdgeId) -> TopoResult<()>;
    /// Delete a vertex that no edge uses.
    fn delete_vertex(&mut self, v: VertexId) -> TopoResult<()>;
    /// Delete a volume (its faces stay, losing that use).
    fn delete_volume(&mut self, v: VolumeId) -> TopoResult<()>;
}
