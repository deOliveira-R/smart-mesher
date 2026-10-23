//! Builder and edit traits for meshers and modifiers (separate from the read-only queries).
//!
//! * [`MeshBuild`] creates a new mesh. Handles it returns (`VertexIx`, `FaceIx`, `CellIx`) are
//!   *provisional*: `finish` normalises orientation, canonicalises the face order and assigns
//!   stable ids, and cell/vertex handles keep their position while face handles are renumbered.
//! * [`MeshEdit`] changes a finished mesh without changing topology (smoothing, snapping,
//!   reclassification, tagging). Topology-changing operations rebuild through [`MeshBuild`].

use crate::classify::Classification;
use crate::error::{MeshError, Result};
use crate::highorder::{Basis, HighOrderCell, NodeRef};
use crate::index::{CellFace, CellIx, FaceIx, MeshEntity, MeshEntityKind, Side, VertexIx};
use crate::read::MeshRead;
use crate::shape::{CellShape, LoopRelation};
use crate::sets::SetMembers;
use sm_core::{EntityId, EntityKind, Point3};
use std::collections::BTreeMap;

/// Deterministic per-kind serial allocator. Serials are never reused; one allocator is shared by
/// every mesh of a model so ids stay unique across hierarchy levels and instances.
#[derive(Clone, Debug, Default)]
pub struct SerialAlloc {
    next: BTreeMap<EntityKind, u64>,
}

impl SerialAlloc {
    /// A fresh allocator (all kinds start at serial 0).
    pub fn new() -> Self {
        Self::default()
    }
    /// Reserve `n` consecutive serials of `kind`; returns the first.
    pub fn take(&mut self, kind: EntityKind, n: usize) -> Result<u64> {
        let slot = self.next.entry(kind).or_insert(0);
        let first = *slot;
        let end = first.checked_add(n as u64).ok_or(MeshError::SerialOverflow)?;
        if end > EntityId::MAX_SERIAL {
            return Err(MeshError::SerialOverflow);
        }
        *slot = end;
        Ok(first)
    }
    /// Reserve one id.
    pub fn one(&mut self, kind: EntityKind) -> Result<EntityId> {
        let s = self.take(kind, 1)?;
        EntityId::new(kind, s).ok_or(MeshError::SerialOverflow)
    }
}

/// Construct a new face-first mesh.
///
/// Meshers normally use [`MeshBuild::add_cell_from_vertices`] (fast-path shapes) or
/// [`MeshBuild::add_polyhedron`] (general). Importers that already hold face-first data
/// (OpenFOAM, CGNS NFACE) use `add_face` + `add_cell`.
pub trait MeshBuild {
    /// The finished mesh type.
    type Output: MeshRead;

    /// Add a vertex.
    fn add_vertex(&mut self, p: Point3) -> Result<VertexIx>;
    /// Add a non-vertex node (high-order geometry); returns a reference usable in `set_high_order`.
    fn add_extra_node(&mut self, p: Point3) -> Result<NodeRef>;
    /// Number of vertices so far.
    fn vertex_count(&self) -> usize;
    /// Number of (provisional) faces so far.
    fn face_count(&self) -> usize;
    /// Number of cells so far.
    fn cell_count(&self) -> usize;
    /// Coordinates of vertex `v`.
    fn vertex_point(&self, v: VertexIx) -> Point3;
    /// Lowest-index vertex within `tol` of `p` (Chebyshev distance), for welding.
    fn find_vertex(&self, p: Point3, tol: f64) -> Option<VertexIx>;

    /// Add a face with the given loop (no deduplication). Needs >= 3 distinct, existing vertices.
    fn add_face(&mut self, vertex_loop: &[VertexIx]) -> Result<FaceIx>;
    /// Find an existing face with the same vertex cycle, and whether it runs the same way.
    fn find_face(&self, vertex_loop: &[VertexIx]) -> Option<(FaceIx, LoopRelation)>;
    /// Current loop of a face.
    fn face_loop(&self, f: FaceIx) -> &[VertexIx];
    /// Number of cells attached to a face so far.
    fn face_degree(&self, f: FaceIx) -> usize;

    /// Add a cell from oriented faces (`side` relative to each face's current loop; see
    /// [`CellFace`]). `ordered` is the template vertex order for fast-path cells.
    fn add_cell(&mut self, faces: &[CellFace], ordered: Option<&[VertexIx]>) -> Result<CellIx>;

    /// Classify a vertex, face or cell (edges are classified on the finished mesh).
    fn set_class(&mut self, e: MeshEntity, c: Classification) -> Result<()>;
    /// Give a cell high-order geometry. `nodes` are in reference-element order.
    fn set_high_order(&mut self, c: CellIx, degree: u8, basis: Basis, nodes: &[NodeRef]) -> Result<()>;
    /// Add a named set (face members use provisional face handles).
    fn add_set(&mut self, name: &str, members: SetMembers) -> Result<()>;
    /// Add a named integer column over all entities of `target` (length must match at `finish`).
    fn add_tag(&mut self, name: &str, target: MeshEntityKind, values: Vec<i64>) -> Result<()>;

    /// Orient, canonicalise, validate incidence, assign ids and return the finished mesh.
    fn finish(self, alloc: &mut SerialAlloc) -> Result<Self::Output>;

    /// Add a cell from outward-oriented face loops, reusing faces that already exist.
    /// `ordered` optionally records the fast-path vertex order.
    fn add_polyhedron_ordered(&mut self, outward_loops: &[Vec<VertexIx>], ordered: Option<&[VertexIx]>) -> Result<CellIx> {
        let mut faces = Vec::with_capacity(outward_loops.len());
        for l in outward_loops {
            match self.find_face(l) {
                Some((f, LoopRelation::Same)) => faces.push(CellFace { face: f, side: Side::Left }),
                Some((f, LoopRelation::Reversed)) => faces.push(CellFace { face: f, side: Side::Right }),
                None => {
                    let f = self.add_face(l)?;
                    faces.push(CellFace { face: f, side: Side::Left });
                }
            }
        }
        self.add_cell(&faces, ordered)
    }

    /// Add a general polyhedron from outward-oriented face loops.
    fn add_polyhedron(&mut self, outward_loops: &[Vec<VertexIx>]) -> Result<CellIx> {
        self.add_polyhedron_ordered(outward_loops, None)
    }

    /// Add a fast-path cell from vertices in the template order of `shape`.
    fn add_cell_from_vertices(&mut self, shape: CellShape, verts: &[VertexIx]) -> Result<CellIx> {
        if shape.vertex_count() != Some(verts.len()) {
            return Err(MeshError::State(format!("{shape:?} needs {:?} vertices, got {}", shape.vertex_count(), verts.len())));
        }
        let loops: Vec<Vec<VertexIx>> =
            shape.local_faces().iter().map(|lf| lf.iter().map(|&i| verts[i as usize]).collect()).collect();
        self.add_polyhedron_ordered(&loops, Some(verts))
    }
}

/// Topology-preserving edits of a finished mesh.
pub trait MeshEdit: MeshRead {
    /// Move a point (vertex or extra node).
    fn set_point(&mut self, i: usize, p: Point3) -> Result<()>;
    /// Reclassify a vertex, edge, face or cell.
    fn set_class(&mut self, e: MeshEntity, c: Classification) -> Result<()>;
    /// Replace or set a cell's high-order record (node indices must exist).
    fn set_high_order(&mut self, c: CellIx, h: HighOrderCell) -> Result<()>;
    /// Make a cell linear again.
    fn clear_high_order(&mut self, c: CellIx) -> Result<()>;
    /// Add a named set; its id comes from `alloc`.
    fn add_set(&mut self, name: &str, members: SetMembers, alloc: &mut SerialAlloc) -> Result<EntityId>;
    /// Add or replace a tag column.
    fn set_tag(&mut self, name: &str, target: MeshEntityKind, values: Vec<i64>) -> Result<()>;
    /// Derive and attach the edge table (no-op if present).
    fn ensure_edges(&mut self, alloc: &mut SerialAlloc) -> Result<()>;
    /// Detach the edge table.
    fn drop_edges(&mut self);
}
