//! Read-only query trait for consumers (DG sweeps, Monte Carlo mesh walking, exporters).
//!
//! # Orientation rule (declared, binding)
//! The vertex loop of a face defines its normal by the right-hand rule. `face_left(f)` is the
//! lower-id cell of the face; the normal points from left into right. A boundary face has no
//! right cell and its normal points outward. `cell_faces(c)` carries the matching [`Side`]:
//! `Left` = the normal is outward for `c`, `Right` = inward.
//!
//! # Canonical ordering
//! Ids ascend with dense index. Face loops start at their smallest vertex index. Faces are
//! ordered by `(left, right-or-none, loop)`. Cell->face lists ascend by face index.

use crate::classify::{ClassSet, Classification};
use crate::edges::EdgeTable;
use crate::highorder::{CellGeometry, HighOrderCell};
use crate::index::{CellFace, CellIx, FaceIx, Side, VertexIx};
use crate::sets::{MeshSet, TagColumn};
use crate::shape::{CellShape, FaceShape};
use sm_core::{EntityId, Point3};

/// Borrowed structure-of-arrays coordinates. The first `vertex_count` points are the vertices.
#[derive(Clone, Copy, Debug)]
pub struct Coords<'a> {
    /// x coordinates.
    pub x: &'a [f64],
    /// y coordinates.
    pub y: &'a [f64],
    /// z coordinates.
    pub z: &'a [f64],
}

impl Coords<'_> {
    /// Number of points (vertices plus high-order extras).
    pub fn len(&self) -> usize {
        self.x.len()
    }
    /// True if there are no points.
    pub fn is_empty(&self) -> bool {
        self.x.is_empty()
    }
    /// Point `i`.
    pub fn get(&self, i: usize) -> Point3 {
        Point3::new(self.x[i], self.y[i], self.z[i])
    }
}

/// Read-only access to one mesh. Object safe; all hot-path accessors return borrowed slices.
pub trait MeshRead: Send + Sync {
    /// Stable id of this mesh (`EntityKind::Mesh`).
    fn mesh_id(&self) -> EntityId;
    /// Number of vertices.
    fn vertex_count(&self) -> usize;
    /// Number of faces.
    fn face_count(&self) -> usize;
    /// Number of cells.
    fn cell_count(&self) -> usize;

    /// Stable id of a vertex.
    fn vertex_id(&self, v: VertexIx) -> EntityId;
    /// Stable id of a face.
    fn face_id(&self, f: FaceIx) -> EntityId;
    /// Stable id of a cell.
    fn cell_id(&self, c: CellIx) -> EntityId;
    /// Dense index of a vertex id (`O(log n)`).
    fn vertex_index(&self, id: EntityId) -> Option<VertexIx>;
    /// Dense index of a face id (`O(log n)`).
    fn face_index(&self, id: EntityId) -> Option<FaceIx>;
    /// Dense index of a cell id (`O(log n)`).
    fn cell_index(&self, id: EntityId) -> Option<CellIx>;

    /// Coordinates (vertices first, then high-order extra nodes).
    fn coords(&self) -> Coords<'_>;

    /// Outward-or-inward loop of a face: vertices in right-hand-normal order, starting at the smallest.
    fn face_vertices(&self, f: FaceIx) -> &[VertexIx];
    /// Left cell of a face (always present).
    fn face_left(&self, f: FaceIx) -> CellIx;
    /// Right cell of a face; `None` on the boundary.
    fn face_right(&self, f: FaceIx) -> Option<CellIx>;

    /// Oriented faces of a cell, ascending by face index.
    fn cell_faces(&self, c: CellIx) -> &[CellFace];
    /// Derived shape tag of a cell.
    fn cell_shape(&self, c: CellIx) -> CellShape;
    /// Ordered vertices in the shape template, present only for fast-path cells built from a vertex list.
    fn cell_ordered_vertices(&self, c: CellIx) -> Option<&[VertexIx]>;
    /// High-order record of a cell, `None` if linear.
    fn high_order_cell(&self, c: CellIx) -> Option<&HighOrderCell>;

    /// Classification of a vertex.
    fn vertex_class(&self, v: VertexIx) -> Classification;
    /// Classification of a face.
    fn face_class(&self, f: FaceIx) -> Classification;
    /// Classification of a cell.
    fn cell_class(&self, c: CellIx) -> Classification;
    /// All mesh entities classified on a model/partition entity, or `None` if there are none.
    fn classified_on(&self, e: EntityId) -> Option<&ClassSet>;
    /// Every model/partition entity that has mesh entities on it, ascending.
    fn classified_entities(&self) -> Vec<EntityId>;

    /// Named sets, ordered by id.
    fn sets(&self) -> &[MeshSet];
    /// Integer tag columns, in insertion order.
    fn tags(&self) -> &[TagColumn];
    /// Derived edge table, if present.
    fn edges(&self) -> Option<&EdgeTable>;

    // ---- provided -------------------------------------------------------------------------

    /// Point `i` of the coordinate arrays.
    fn point(&self, i: usize) -> Point3 {
        self.coords().get(i)
    }
    /// Shape tag of a face.
    fn face_shape(&self, f: FaceIx) -> FaceShape {
        FaceShape::from_vertex_count(self.face_vertices(f).len())
    }
    /// `(left, right)` cells of a face.
    fn face_cells(&self, f: FaceIx) -> (CellIx, Option<CellIx>) {
        (self.face_left(f), self.face_right(f))
    }
    /// True if the face has no right cell.
    fn is_boundary_face(&self, f: FaceIx) -> bool {
        self.face_right(f).is_none()
    }
    /// The cell on the other side of `cf` (as listed by the cell on `cf.side`), or `None` at the
    /// boundary. `O(1)`: this is the mesh-walking step.
    fn across(&self, cf: CellFace) -> Option<CellIx> {
        match cf.side {
            Side::Left => self.face_right(cf.face),
            Side::Right => Some(self.face_left(cf.face)),
        }
    }
    /// Distinct vertices of a cell, ascending (allocates; use ordered vertices on hot paths).
    fn cell_vertices_sorted(&self, c: CellIx) -> Vec<VertexIx> {
        let mut v: Vec<VertexIx> = self.cell_faces(c).iter().flat_map(|cf| self.face_vertices(cf.face).iter().copied()).collect();
        v.sort_unstable();
        v.dedup();
        v
    }
    /// Face-neighbour cells of a cell in face order (skips boundary faces).
    fn cell_neighbors(&self, c: CellIx) -> Vec<CellIx> {
        self.cell_faces(c).iter().filter_map(|&cf| self.across(cf)).collect()
    }
    /// All boundary faces, ascending.
    fn boundary_faces(&self) -> Vec<FaceIx> {
        (0..self.face_count()).map(FaceIx::new).filter(|&f| self.is_boundary_face(f)).collect()
    }
    /// Shape, degree and basis of a cell (linear Lagrange if not high order).
    fn cell_geometry(&self, c: CellIx) -> CellGeometry {
        match self.high_order_cell(c) {
            Some(h) => CellGeometry { shape: self.cell_shape(c), degree: h.degree, basis: h.basis },
            None => CellGeometry { shape: self.cell_shape(c), degree: 1, basis: crate::highorder::Basis::LagrangeEquispaced },
        }
    }
    /// Find a set by name.
    fn set_by_name(&self, name: &str) -> Option<&MeshSet> {
        self.sets().iter().find(|s| s.name == name)
    }
    /// Find a tag column by name.
    fn tag_by_name(&self, name: &str) -> Option<&TagColumn> {
        self.tags().iter().find(|t| t.name == name)
    }
}
