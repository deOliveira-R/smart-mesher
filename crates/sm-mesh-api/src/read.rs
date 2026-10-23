//! Read-only query trait for consumers (DG sweeps, Monte Carlo mesh walking, exporters).
//!
//! # Orientation rule (declared, binding)
//! The vertex loop of a face defines its normal by the right-hand rule. `face_left(f)` is the
//! lower-id cell of the face; the normal points from left into right. A one-cell face (boundary,
//! non-conformal, partition cut) has no right cell and its normal points outward.
//! `cell_faces(c)` carries the matching [`SignedFace`]: `Left` / `+` = the normal is outward for
//! `c`, `Right` / `-` = inward. This makes OpenFOAM `owner`/`neighbour` and CGNS `ParentElements`
//! / NFACE exports a renumbering with no flips (the OpenFOAM face *order* is an exporter step:
//! interior faces by `(owner, neighbour)`, then boundary patches).
//!
//! # Canonical ordering
//! Ids ascend with dense index. Face loops start at their smallest vertex index. Faces are
//! ordered by `(left, right-or-none, loop)`. Cell->face lists ascend by face index.
//!
//! # Dimension
//! [`MeshRead::dim`] is the topological dimension ([`MeshDim`]). Everything above describes a
//! dimension-3 mesh; a dimension-2 mesh keeps the same invariants one level down: cells are
//! polygons, faces are two-vertex facets `[a, b]` (no rotation: the loop is directed), the left
//! cell traverses the facet `a -> b` and the right cell `b -> a`, and `cell_corners` is the
//! cell's vertex cycle in boundary order (present for every cell, polygons included).
//!
//! # Indices
//! Every method taking an index requires it to be below the matching `count`; a foreign or
//! out-of-range index panics (slice indexing) in the reference implementation, and other
//! implementations may do the same. Use [`MeshRead::index_of`] to translate ids.

use crate::classify::{ClassSet, Classification};
use crate::edges::EdgeTable;
use crate::error::{MeshError, Result};
use crate::highorder::{Basis, CellGeometry, FaceCurve, HighOrderCell};
use crate::index::{CellIx, FaceIx, FaceKind, MeshDim, MeshEntityKind, Side, SignedFace, VertexIx};
use crate::sets::{MeshSet, TagColumn};
use crate::shape::{CellShape, FaceShape};
use sm_core::{EntityId, Point3, Vector3};

/// Borrowed structure-of-arrays coordinates. The first `n_vertices` points are the vertices.
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

/// The raw face-first arrays (addendum 7.7 item 2) as whole slices, so hot loops can avoid
/// per-call dynamic dispatch. Offsets are `u64` (the on-disk width).
#[derive(Clone, Copy, Debug)]
pub struct TopologyColumns<'a> {
    /// CSR offsets into `face_vertices`, length `n_faces + 1`.
    pub face_vertex_offsets: &'a [u64],
    /// Concatenated face loops.
    pub face_vertices: &'a [VertexIx],
    /// Left cell of every face.
    pub face_left: &'a [CellIx],
    /// Right cell of every face, [`crate::NO_CELL`] if none.
    pub face_right: &'a [u32],
    /// CSR offsets into `cell_faces`, length `n_cells + 1`.
    pub cell_face_offsets: &'a [u64],
    /// Concatenated signed faces of every cell.
    pub cell_faces: &'a [SignedFace],
}

/// Cached face and cell geometry under a rule id (addendum 7.5 and 7.7 item 5). Derived data:
/// excluded from both content hashes and checked by `validate` against the mesh.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct GeometryCache {
    /// Geometry rule id the values were computed under.
    pub rule: String,
    /// [`crate::connectivity_hash`] of the mesh the cache was computed for.
    pub connectivity_hash: u64,
    /// [`crate::coordinate_hash`] of the mesh the cache was computed for.
    pub coordinate_hash: u64,
    /// Face area vectors (left -> right).
    pub face_area: Vec<Vector3>,
    /// Face centroids.
    pub face_centroid: Vec<Point3>,
    /// Face apexes (vertex means; persisted for non-planar faces).
    pub face_apex: Vec<Point3>,
    /// Cell volumes.
    pub cell_volume: Vec<f64>,
    /// Cell centroids.
    pub cell_centroid: Vec<Point3>,
}

/// Read-only access to one mesh. Object safe; all hot-path accessors return borrowed slices or
/// plain values and never allocate.
pub trait MeshRead: Send + Sync {
    /// Stable id of this mesh (`EntityKind::Mesh`).
    fn mesh_id(&self) -> EntityId;
    /// Topological dimension (default [`MeshDim::Three`]; a dimension-2 mesh overrides it).
    fn dim(&self) -> MeshDim {
        MeshDim::Three
    }
    /// Number of entities of kind `k` (edges: 0 without an edge table).
    fn count(&self, k: MeshEntityKind) -> usize;
    /// Stable id of entity `ix` of kind `k`; ids ascend with the dense index.
    fn id(&self, k: MeshEntityKind, ix: u32) -> EntityId;
    /// Dense index of an entity id (`O(log n)`); `None` if the id is not in this mesh.
    fn index_of(&self, id: EntityId) -> Option<u32>;

    /// Coordinates (vertices first, then high-order extra nodes).
    fn coords(&self) -> Coords<'_>;
    /// The raw face-first arrays as whole slices.
    fn columns(&self) -> TopologyColumns<'_>;

    /// Loop of a face: vertices in right-hand-normal order, starting at the smallest index.
    fn face_vertices(&self, f: FaceIx) -> &[VertexIx];
    /// Left cell of a face (always present).
    fn face_left(&self, f: FaceIx) -> CellIx;
    /// Right cell of a face; `None` for every kind except [`FaceKind::Interior`].
    fn face_right(&self, f: FaceIx) -> Option<CellIx>;
    /// Kind of a face.
    fn face_kind(&self, f: FaceIx) -> FaceKind;

    /// Signed faces of a cell, ascending by face index.
    fn cell_faces(&self, c: CellIx) -> &[SignedFace];
    /// Derived shape tag of a cell.
    fn cell_shape(&self, c: CellIx) -> CellShape;
    /// Corner vertices in the template order of the cell's shape; `Some` for every
    /// template-shaped (non-polyhedron) cell, and for every cell of a dimension-2 mesh (the
    /// vertex cycle in boundary order).
    fn cell_corners(&self, c: CellIx) -> Option<&[VertexIx]>;
    /// High-order record of a cell, `None` if linear.
    fn high_order_cell(&self, c: CellIx) -> Option<&HighOrderCell>;
    /// Curvature record of a face, `None` if flat.
    fn face_curve(&self, f: FaceIx) -> Option<FaceCurve<'_>>;
    /// Lineage: the parent of a refined cell, `None` for a root cell.
    fn cell_parent(&self, c: CellIx) -> Option<CellIx>;
    /// False for a parent retained inactive after refinement (addendum 7.2).
    fn cell_active(&self, c: CellIx) -> bool;

    /// Classification of entity `ix` of kind `k`. Edges need an edge table (without one the
    /// edge count is 0, so any edge index is out of range).
    fn classification(&self, k: MeshEntityKind, ix: u32) -> Classification;
    /// All mesh entities classified on a model/partition entity, or `None` if there are none.
    /// After refinement the set holds the faces of inactive parents as well as their children's;
    /// filter faces with [`MeshRead::face_active`] before integrating.
    fn classified_on(&self, model: EntityId) -> Option<&ClassSet>;
    /// Every model/partition entity that has mesh entities on it, ascending.
    fn classified_entities(&self) -> &[EntityId];

    /// Derived edge table, if present.
    fn edges(&self) -> Option<&EdgeTable>;
    /// Named sets, ordered by id.
    fn sets(&self) -> &[MeshSet];
    /// All tag columns, ordered by `(target, name)`.
    fn tags(&self) -> &[TagColumn];

    /// Id of the geometry rule that defines face and cell geometry for this mesh.
    fn geometry_rule(&self) -> &str {
        crate::geometry::FAN_VERTEX_MEAN_V1
    }
    /// Cached geometry, if the mesh carries it.
    fn geometry_cache(&self) -> Option<&GeometryCache> {
        None
    }

    // ---- provided, all allocation-free -------------------------------------------------------

    /// The cell on the other side of `sf` (as listed by the cell that owns `sf`), or `None` for
    /// a one-cell face. `O(1)`: this is the mesh-walking step. `sf` must come from the owning
    /// cell's [`MeshRead::cell_faces`]; use [`MeshRead::other_cell`] for a checked lookup.
    fn across(&self, sf: SignedFace) -> Option<CellIx> {
        let f = sf.face();
        debug_assert!(
            f.get() < self.count(MeshEntityKind::Face),
            "face index out of range"
        );
        match sf.side() {
            Side::Left => self.face_right(f),
            Side::Right => Some(self.face_left(f)),
        }
    }
    /// The cell opposite `c` across face `f`: `Ok(None)` for a one-cell face, an error if `c` is
    /// not incident to `f`.
    fn other_cell(&self, f: FaceIx, c: CellIx) -> Result<Option<CellIx>> {
        let (l, r) = (self.face_left(f), self.face_right(f));
        if l == c {
            Ok(r)
        } else if r == Some(c) {
            Ok(Some(l))
        } else {
            Err(MeshError::NotIncident(f, c))
        }
    }
    /// True for faces on the model boundary ([`FaceKind::Boundary`]).
    ///
    /// **Refinement warning:** an inactive (refined) parent keeps its boundary faces next to its
    /// children's, so summing over boundary faces counts that area twice. Filter with
    /// [`MeshRead::face_active`].
    fn is_boundary_face(&self, f: FaceIx) -> bool {
        self.face_kind(f) == FaceKind::Boundary
    }
    /// True if at least one cell of the face is active. Faces of a refined parent (all cells
    /// inactive) are not active; use this to skip them when integrating over a face set.
    fn face_active(&self, f: FaceIx) -> bool {
        self.cell_active(self.face_left(f))
            || self.face_right(f).is_some_and(|r| self.cell_active(r))
    }
    /// `(left, right)` cells of a face.
    fn face_cells(&self, f: FaceIx) -> (CellIx, Option<CellIx>) {
        (self.face_left(f), self.face_right(f))
    }
    /// Shape tag of a face.
    fn face_shape(&self, f: FaceIx) -> FaceShape {
        FaceShape::from_vertex_count(self.face_vertices(f).len())
    }
    /// Shape, degree and basis of a cell (linear Lagrange if not high order).
    fn cell_geometry(&self, c: CellIx) -> CellGeometry {
        match self.high_order_cell(c) {
            Some(h) => CellGeometry {
                shape: self.cell_shape(c),
                degree: h.degree,
                basis: h.basis,
            },
            None => CellGeometry {
                shape: self.cell_shape(c),
                degree: 1,
                basis: Basis::LagrangeEquispaced,
            },
        }
    }
    /// Point `i` of the coordinate arrays.
    fn point(&self, i: usize) -> Point3 {
        self.coords().get(i)
    }
    /// Number of vertices.
    fn vertex_count(&self) -> usize {
        self.count(MeshEntityKind::Vertex)
    }
    /// Number of faces.
    fn face_count(&self) -> usize {
        self.count(MeshEntityKind::Face)
    }
    /// Number of cells.
    fn cell_count(&self) -> usize {
        self.count(MeshEntityKind::Cell)
    }
    /// Stable id of a cell.
    fn cell_id(&self, c: CellIx) -> EntityId {
        self.id(MeshEntityKind::Cell, c.0)
    }
    /// Stable id of a face.
    fn face_id(&self, f: FaceIx) -> EntityId {
        self.id(MeshEntityKind::Face, f.0)
    }
    /// Stable id of a vertex.
    fn vertex_id(&self, v: VertexIx) -> EntityId {
        self.id(MeshEntityKind::Vertex, v.0)
    }
    /// Classification of a vertex.
    fn vertex_class(&self, v: VertexIx) -> Classification {
        self.classification(MeshEntityKind::Vertex, v.0)
    }
    /// Classification of a face.
    fn face_class(&self, f: FaceIx) -> Classification {
        self.classification(MeshEntityKind::Face, f.0)
    }
    /// Classification of a cell.
    fn cell_class(&self, c: CellIx) -> Classification {
        self.classification(MeshEntityKind::Cell, c.0)
    }
    /// A named set.
    fn set_by_name(&self, name: &str) -> Option<&MeshSet> {
        self.sets().iter().find(|s| s.name == name)
    }
    /// The tag column `(k, name)`.
    fn tag(&self, k: MeshEntityKind, name: &str) -> Option<&TagColumn> {
        self.tags().iter().find(|t| t.target == k && t.name == name)
    }
}
