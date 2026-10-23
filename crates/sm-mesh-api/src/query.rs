//! Read-only query traits: what consumers (DG sweeps, Monte Carlo mesh walking, exporters) see.
//!
//! The traits are object safe: write consumers against `&dyn MeshRead`. Accessors that take a
//! dense index panic if the index did not come from this mesh (caller bug, as with slices).
//! Everything returned in a collection is in canonical (ascending index) order.

use sm_core::{EntityId, Point3, Vector3};
use smallvec::SmallVec;

use crate::classify::Classification;
use crate::index::{CellIx, Dim, EdgeIx, FaceIx, Sign, SignedFace, VertexIx};
use crate::rules;
use crate::schema::MeshSchema;
use crate::shape::{Basis, CellShape, CellSpec, FaceShape};
use crate::tag::{EntitySet, TagColumn, TagRow};

/// Coordinates as structure-of-arrays columns (f64).
#[derive(Clone, Copy, Debug)]
pub struct CoordsRef<'a> {
    /// x coordinates.
    pub x: &'a [f64],
    /// y coordinates.
    pub y: &'a [f64],
    /// z coordinates.
    pub z: &'a [f64],
}

/// A curved (high-order) face of a polyhedron: sparse side table entry (addendum §7.5).
#[derive(Clone, Copy, Debug)]
pub struct FaceCurve<'a> {
    /// Polynomial degree of the face map.
    pub degree: u8,
    /// Basis of the face map.
    pub basis: Basis,
    /// Node vertices of the face map (loop vertices first).
    pub nodes: &'a [VertexIx],
}

/// Face-first topology: counts, ids, and the stored oriented relations.
///
/// Orientation rule: see [`crate::orient`]. `face_left` always exists and is the lower cell id;
/// the stored loop normal points from `face_left` into `face_right`.
pub trait MeshTopology {
    /// The id of this mesh (kind `Mesh`).
    fn mesh_id(&self) -> EntityId;
    /// Number of entities of dimension `dim`.
    fn len(&self, dim: Dim) -> usize;
    /// True if the mesh has no cells.
    fn is_empty(&self) -> bool {
        self.len(Dim::Cell) == 0
    }
    /// Stable id of entity `ix`, or `None` if out of range.
    fn id_of(&self, dim: Dim, ix: u32) -> Option<EntityId>;
    /// Dense index of a stable id (dimension taken from the id's kind), by binary search.
    fn index_of(&self, id: EntityId) -> Option<u32>;
    /// Shape tag of a cell.
    fn cell_shape(&self, c: CellIx) -> CellShape;
    /// Oriented cell to face relation, in the cell's local face order.
    fn cell_faces(&self, c: CellIx) -> &[SignedFace];
    /// Reference-ordered corner vertices; `None` for polyhedra.
    fn cell_corners(&self, c: CellIx) -> Option<&[VertexIx]>;
    /// Face vertex loop (right-hand rule gives the normal), starting at the smallest vertex.
    fn face_vertices(&self, f: FaceIx) -> &[VertexIx];
    /// The `left` cell of a face (lower id).
    fn face_left(&self, f: FaceIx) -> CellIx;
    /// The `right` cell of a face; `None` on the boundary.
    fn face_right(&self, f: FaceIx) -> Option<CellIx>;
    /// Edge endpoints (`v0 < v1`). Meshes may have no edge table (`len(Edge) == 0`).
    fn edge_vertices(&self, e: EdgeIx) -> [VertexIx; 2];

    /// Fast-path face shape.
    fn face_shape(&self, f: FaceIx) -> FaceShape {
        FaceShape::from_len(self.face_vertices(f).len())
    }
    /// True if the face has no `right` cell.
    fn is_boundary_face(&self, f: FaceIx) -> bool {
        self.face_right(f).is_none()
    }
    /// The cell on the other side of `f` from `c`; `None` on the boundary.
    fn face_other_cell(&self, f: FaceIx, c: CellIx) -> Option<CellIx> {
        if self.face_left(f) == c { self.face_right(f) } else { Some(self.face_left(f)) }
    }
    /// Mesh walking: the cell reached by leaving `c` through its local face number `k`.
    fn next_cell(&self, c: CellIx, k: usize) -> Option<CellIx> {
        let sf = *self.cell_faces(c).get(k)?;
        self.face_other_cell(sf.face(), c)
    }
    /// Face-neighbour cells of `c` in local face order (boundary faces skipped).
    fn cell_neighbors(&self, c: CellIx) -> SmallVec<[CellIx; 8]> {
        self.cell_faces(c).iter().filter_map(|sf| self.face_other_cell(sf.face(), c)).collect()
    }
    /// All distinct vertices of a cell, ascending.
    fn cell_vertices(&self, c: CellIx) -> SmallVec<[VertexIx; 16]> {
        let mut v: SmallVec<[VertexIx; 16]> =
            self.cell_faces(c).iter().flat_map(|sf| self.face_vertices(sf.face()).iter().copied()).collect();
        v.sort_unstable();
        v.dedup();
        v
    }
    /// All boundary faces, ascending.
    fn boundary_faces(&self) -> Vec<FaceIx> {
        (0..self.len(Dim::Face) as u32).map(FaceIx).filter(|&f| self.is_boundary_face(f)).collect()
    }
}

/// Geometry: coordinates, per-cell `(shape, degree, basis)`, and derived geometry by the declared
/// rule (default [`rules::FAN_VERTEX_MEAN_V1`]).
pub trait MeshGeometry: MeshTopology {
    /// Coordinate columns.
    fn coords(&self) -> CoordsRef<'_>;
    /// Point of a vertex.
    fn point(&self, v: VertexIx) -> Point3 {
        let c = self.coords();
        Point3::new(c.x[v.index()], c.y[v.index()], c.z[v.index()])
    }
    /// Shape, degree and basis of a cell.
    fn cell_spec(&self, c: CellIx) -> CellSpec;
    /// Geometry nodes of a cell in basis order (corners first). Empty for polyhedra of degree 1.
    fn cell_nodes(&self, c: CellIx) -> &[VertexIx];
    /// Curvature record of a face, if it is curved.
    fn face_curve(&self, f: FaceIx) -> Option<FaceCurve<'_>>;
    /// Id of the geometry rule used by the provided derived-geometry methods.
    fn geometry_rule(&self) -> &str {
        rules::FAN_VERTEX_MEAN_V1
    }

    /// Area vector of a face in its stored orientation (points from `left` into `right`).
    fn face_area_vector(&self, f: FaceIx) -> Vector3 {
        rules::fan_face(&self.coords(), self.face_vertices(f)).area_vector
    }
    /// Face centre under the fan rule.
    fn face_centroid(&self, f: FaceIx) -> Point3 {
        rules::fan_face(&self.coords(), self.face_vertices(f)).centroid
    }
    /// Area vector of face `sf` pointing out of cell `c` (what an upwind test needs).
    fn outward_area_vector(&self, sf: SignedFace) -> Vector3 {
        self.face_area_vector(sf.face()) * sf.sign().as_f64()
    }
    /// Cell volume and centroid under the fan rule.
    fn cell_volume_centroid(&self, c: CellIx) -> (f64, Point3) {
        let coords = self.coords();
        let vs = self.cell_vertices(c);
        let n = vs.len().max(1) as f64;
        let (mut ox, mut oy, mut oz) = (0.0, 0.0, 0.0);
        for v in &vs {
            ox += coords.x[v.index()];
            oy += coords.y[v.index()];
            oz += coords.z[v.index()];
        }
        let o = Point3::new(ox / n, oy / n, oz / n);
        let (mut vol, mut mom) = (0.0, Vector3::default());
        for sf in self.cell_faces(c) {
            let (v, m) = rules::fan_cell_contribution(&coords, self.face_vertices(sf.face()), o);
            let s = match sf.sign() {
                Sign::Plus => 1.0,
                Sign::Minus => -1.0,
            };
            vol += s * v;
            mom = mom + m * s;
        }
        let centroid = if vol != 0.0 { Point3::new(mom.x / vol, mom.y / vol, mom.z / vol) } else { o };
        (vol, centroid)
    }
    /// Cell volume under the fan rule.
    fn cell_volume(&self, c: CellIx) -> f64 {
        self.cell_volume_centroid(c).0
    }
}

/// Classification of mesh entities onto model/partition entities.
pub trait MeshClassification {
    /// Classification of entity `ix`, if set.
    fn classification(&self, dim: Dim, ix: u32) -> Option<Classification>;
    /// Entities of `dim` classified on `model`, ascending.
    fn entities_on(&self, dim: Dim, model: EntityId) -> &[u32];
    /// Model entities that classify at least one `dim` entity, ascending by id.
    fn classified_models(&self, dim: Dim) -> Vec<EntityId>;
}

/// Tags and sets.
pub trait MeshTags {
    /// The declared schema.
    fn schema(&self) -> &MeshSchema;
    /// A tag column by dimension and name.
    fn tag(&self, dim: Dim, name: &str) -> Option<&TagColumn>;
    /// A set by name.
    fn set(&self, name: &str) -> Option<&EntitySet>;
    /// One tag row, if the tag and entity exist.
    fn tag_row(&self, dim: Dim, name: &str, ix: u32) -> Option<TagRow<'_>> {
        self.tag(dim, name)?.row(ix as usize)
    }
}

/// Everything a consumer needs; blanket-implemented for any type providing the four views.
pub trait MeshRead: MeshGeometry + MeshClassification + MeshTags {}
impl<T: MeshGeometry + MeshClassification + MeshTags + ?Sized> MeshRead for T {}
