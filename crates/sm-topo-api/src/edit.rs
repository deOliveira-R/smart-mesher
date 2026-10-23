//! The edit contract, [`TopologyEdit`]: Euler-style operators.
//!
//! Booleans, imprint, merge, webcut, extrude and healing are *compositions* of these primitives
//! (they live in `sm-model`). Keeping the primitive set small and closed under validity means any
//! op that ends with a valid complex is correct by construction, and every primitive records
//! lineage.
//!
//! Protocol: `begin_op(id)` ... edits ... `end_op()`. Ids are allocated in call order, so an op
//! must create entities in a canonical, data-defined order (ARCHITECTURE §3). Derived indices
//! (radial rings, coedge and vertex-use lists) are rebuilt by `end_op`, and are only guaranteed
//! fresh after it.

use crate::error::TopoError;
use crate::ids::*;
use crate::model::*;
use crate::view::TopologyView;
use sm_core::{EntityId, Point3};

/// Summary returned by `end_op`.
#[derive(Clone, Debug, PartialEq)]
pub struct OpSummary {
    /// The op.
    pub op: OpId,
    /// Entities created, in creation order.
    pub created: Vec<EntityId>,
    /// Entities retired (split, merged or deleted), in order.
    pub retired: Vec<EntityId>,
}

/// Where to cut an edge.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SplitAt {
    /// At curve parameter `t`; a new vertex is created at the curve point.
    NewVertex(f64),
    /// At curve parameter `t`, using an existing vertex that must lie on the curve there within resolution.
    Vertex {
        /// Parameter.
        t: f64,
        /// Existing vertex.
        vertex: VertexId,
    },
}

/// Result of `split_edge`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EdgeSplit {
    /// The vertex at the cut.
    pub vertex: VertexId,
    /// Piece from the original start to the cut.
    pub first: EdgeId,
    /// Piece from the cut to the original end.
    pub second: EdgeId,
}

/// A cut of a face by one existing edge joining two corners of its outer loop (Euler operator MEF).
#[derive(Clone, Debug, PartialEq)]
pub struct FaceCut {
    /// Corner index `i`: the tail vertex of outer-loop coedge `i` (in stored order). Must be `< to`.
    pub from: u32,
    /// Corner index `j`.
    pub to: u32,
    /// Edge joining the two corner vertices (either direction); already created.
    pub edge: EdgeId,
    /// Inner loops (by index) that go to the first piece; the others go to the second.
    pub inner_loops_to_first: Vec<u16>,
}

/// Result of `split_face`. `first` contains coedges `from..to`, `second` the rest.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaceSplit {
    /// Piece bounded by corners `from..to`.
    pub first: FaceId,
    /// The remaining piece.
    pub second: FaceId,
}

/// Mutating operations on a cell complex. Every method requires an open op, else `NoOpenOp`.
pub trait TopologyEdit: TopologyView {
    /// Open an op. `op` must be greater than every earlier op.
    fn begin_op(&mut self, op: OpId) -> Result<(), TopoError>;
    /// Close the op: rebuild derived indices and return what changed.
    fn end_op(&mut self) -> Result<OpSummary, TopoError>;

    /// New empty body.
    fn make_body(&mut self) -> Result<BodyId, TopoError>;
    /// New vertex.
    fn make_vertex(&mut self, point: Point3) -> Result<VertexId, TopoError>;
    /// New edge. The curve must resolve and its end points must match the vertices within resolution.
    fn make_edge(&mut self, spec: EdgeSpec) -> Result<EdgeId, TopoError>;
    /// New face with both uses exterior. Loops must be closed chains of active edges.
    fn make_face(&mut self, spec: FaceSpec) -> Result<FaceId, TopoError>;
    /// New volume in a body, with no shells yet.
    fn make_volume(&mut self, body: BodyId) -> Result<VolumeId, TopoError>;
    /// New shell in a volume.
    fn make_shell(&mut self, volume: VolumeId, kind: ShellKind) -> Result<ShellRef, TopoError>;
    /// Give an exterior face use to a shell (the shell's volume now sees that side of the face).
    fn bind_face_use(&mut self, fu: FaceUse, shell: ShellRef) -> Result<(), TopoError>;
    /// Return a face use to the exterior.
    fn release_face_use(&mut self, fu: FaceUse) -> Result<(), TopoError>;
    /// Add a wire edge to a shell (`Internal` shells hold free wires).
    fn add_wire_edge(&mut self, shell: ShellRef, edge: EdgeId) -> Result<(), TopoError>;
    /// Add an isolated vertex to a shell.
    fn add_lone_vertex(&mut self, shell: ShellRef, vertex: VertexId) -> Result<(), TopoError>;
    /// Register a face as a sheet of a body (both uses must be exterior).
    fn add_sheet(&mut self, body: BodyId, face: FaceId) -> Result<(), TopoError>;

    /// Split an edge in two (Euler MEV). The edge is retired (`Split`), and every coedge and wire
    /// that used it now uses both pieces in the right order and sense.
    fn split_edge(&mut self, edge: EdgeId, at: SplitAt) -> Result<EdgeSplit, TopoError>;
    /// Split a face in two along an existing edge (Euler MEF). The face is retired (`Split`);
    /// the shells that held its uses now hold both pieces on the same sides.
    fn split_face(&mut self, face: FaceId, cut: FaceCut) -> Result<FaceSplit, TopoError>;

    /// Merge `drop` into `keep` (vertex identification). Their points must be within resolution.
    /// `keep` survives; `drop` is retired (`Merged`) and every reference is redirected.
    fn merge_vertices(&mut self, keep: VertexId, drop: VertexId) -> Result<(), TopoError>;
    /// Merge `drop` into `keep` (edge identification). Endpoints must already be identical and the
    /// curves must coincide within resolution (checked at sample points). Senses of redirected
    /// coedges are composed so geometry is preserved.
    fn merge_edges(&mut self, keep: EdgeId, drop: EdgeId) -> Result<(), TopoError>;
    /// Merge face `drop` into `keep` (face identification, the "merge" of imprint/merge).
    /// Their loops must reference the same edges. `relative` says whether the normals agree
    /// (`Forward`) or oppose (`Reversed`). Each non-exterior use of `drop` moves to the matching
    /// side of `keep`, which must be exterior there. Records `Coincidence::Merged`.
    fn merge_faces(&mut self, keep: FaceId, drop: FaceId, relative: sm_geom_api::Sense) -> Result<(), TopoError>;

    /// Remove a face (both uses must be exterior).
    fn kill_face(&mut self, face: FaceId) -> Result<(), TopoError>;
    /// Remove an edge (no coedge or wire may use it).
    fn kill_edge(&mut self, edge: EdgeId) -> Result<(), TopoError>;
    /// Remove a vertex (no edge, pole or shell may use it).
    fn kill_vertex(&mut self, vertex: VertexId) -> Result<(), TopoError>;
    /// Remove a volume (its shells must be empty).
    fn kill_volume(&mut self, volume: VolumeId) -> Result<(), TopoError>;
    /// Remove a body (it must have no volumes, sheets or wires).
    fn kill_body(&mut self, body: BodyId) -> Result<(), TopoError>;

    /// Record the imprint/merge status of a pair of faces (replacing an earlier record for the
    /// same pair). `Merged` is set by `merge_faces`, not by this call.
    fn set_coincidence(&mut self, a: FaceId, b: FaceId, status: Coincidence, overlap_area: Option<f64>) -> Result<(), TopoError>;
}
