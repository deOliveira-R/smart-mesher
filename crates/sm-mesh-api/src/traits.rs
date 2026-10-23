//! Read-only query traits (consumers) and the edit trait (meshers, modifiers).
//!
//! All read traits are object safe and return slices or owned vectors, so consumers can
//! hold `&dyn MeshView` and the columnar CSR layout is part of the contract.

use crate::error::MeshError;
use crate::point::{Orient, Point, Stratum};
use crate::shape::{Basis, CellGeometry, CellShape, NodeRef};
use sm_core::{EntityId, EntityKind, Point3};
use smallvec::SmallVec;

/// Per-incidence orientations of a [`Cone`]: one value for all, or one per entry.
#[derive(Clone, Copy, Debug)]
pub enum Orients<'a> {
    /// All incidences share this orientation (no storage).
    Uniform(Orient),
    /// One orientation per incidence.
    Slice(&'a [Orient]),
}

/// A cone or support: the points incident to a point one stratum away, with orientation.
///
/// Entries are `u32` indices into [`Cone::stratum`], stored contiguously (CSR row).
#[derive(Clone, Copy, Debug)]
pub struct Cone<'a> {
    /// Stratum all entries belong to.
    pub stratum: Stratum,
    /// Dense indices within `stratum`, in canonical order.
    pub indices: &'a [u32],
    /// Orientation of each incidence.
    pub orients: Orients<'a>,
}

impl<'a> Cone<'a> {
    /// An empty cone.
    pub const EMPTY: Cone<'static> =
        Cone { stratum: Stratum::Vertex, indices: &[], orients: Orients::Uniform(Orient::Fwd) };

    /// Number of incidences.
    pub fn len(&self) -> usize {
        self.indices.len()
    }
    /// True if there are no incidences.
    pub fn is_empty(&self) -> bool {
        self.indices.is_empty()
    }
    /// The `i`-th point. Panics if `i >= len()`.
    pub fn point(&self, i: usize) -> Point {
        Point::new(self.stratum, self.indices[i])
    }
    /// The `i`-th orientation. Panics if `i >= len()`.
    pub fn orient(&self, i: usize) -> Orient {
        match self.orients {
            Orients::Uniform(o) => o,
            Orients::Slice(s) => s[i],
        }
    }
    /// Iterate `(point, orientation)`.
    pub fn iter(&self) -> impl Iterator<Item = (Point, Orient)> + '_ {
        (0..self.len()).map(move |i| (self.point(i), self.orient(i)))
    }
}

/// The two cells of a face under the declared orientation rule.
///
/// The face normal (right-hand rule on its vertex loop) points from `left` into `right`;
/// `left` has the lower cell `EntityId`. Boundary faces have `right == None` and the normal
/// points out of `left`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaceSides {
    /// The cell the normal points out of.
    pub left: Point,
    /// The cell the normal points into; `None` on the boundary.
    pub right: Option<Point>,
}

/// Face-first Hasse topology (layer "Meshes" of docs/design/ir.md).
///
/// Strata: vertices (0), edges (1), faces (2), cells (3). Stored relations:
/// * `cone(cell)` = oriented faces; `cone(face)` = ordered vertex loop (it *skips* the edge
///   stratum, so polygons of any size need no edge decomposition); `cone(edge)` = 2 vertices.
/// * `support` is the transpose: `support(face)` = `[left, right?]`.
///
/// Methods taking a [`Point`] panic if the point is not in this mesh (check [`Self::contains`]).
pub trait MeshTopology {
    /// The mesh's own id (kind `Mesh`).
    fn mesh_id(&self) -> EntityId;
    /// Number of points in a stratum.
    fn count(&self, stratum: Stratum) -> u32;
    /// Stable id of a point.
    fn entity_id(&self, p: Point) -> EntityId;
    /// Inverse of [`Self::entity_id`].
    fn point_of(&self, id: EntityId) -> Option<Point>;
    /// Lower-dimension incidences (see trait docs for what each stratum stores).
    fn cone(&self, p: Point) -> Cone<'_>;
    /// Upper-dimension incidences. Vertex/edge: faces. Face: cells (`left` first). Cell: empty.
    fn support(&self, p: Point) -> Cone<'_>;
    /// Left/right cells of a face.
    fn face_sides(&self, face: Point) -> FaceSides;
    /// Shape tag of a cell.
    fn cell_shape(&self, cell: Point) -> CellShape;
    /// Edges of a face in loop order: edge `i` joins loop vertices `i` and `i+1`.
    /// The orientation is `Fwd` when the edge's stored direction agrees with the loop.
    fn face_edges(&self, face: Point) -> Cone<'_>;

    /// True if `p` exists in this mesh.
    fn contains(&self, p: Point) -> bool {
        p.index < self.count(p.stratum)
    }
    /// Iterate all points of a stratum in canonical order.
    fn points(&self, stratum: Stratum) -> Vec<Point> {
        Point::range(stratum, self.count(stratum)).collect()
    }
    /// The neighbour of `cell` across `face`, `None` on the boundary.
    fn across(&self, cell: Point, face: Point) -> Option<Point> {
        let s = self.face_sides(face);
        if s.left == cell { s.right } else { Some(s.left) }
    }
    /// All boundary faces (`right == None`) in canonical order.
    fn boundary_faces(&self) -> Vec<Point> {
        Point::range(Stratum::Face, self.count(Stratum::Face))
            .filter(|&f| self.face_sides(f).right.is_none())
            .collect()
    }
    /// Face loop of `face` with the given incidence orientation applied: `Fwd` keeps the
    /// stored (normal = left->right) loop, `Rev` reverses it. For a cell's own outward view
    /// pass the orientation from `cone(cell)`.
    fn outward_loop(&self, face: Point, orient: Orient) -> SmallVec<[u32; 8]> {
        let c = self.cone(face);
        match orient {
            Orient::Fwd => c.indices.iter().copied().collect(),
            Orient::Rev => c.indices.iter().rev().copied().collect(),
        }
    }
    /// Sorted distinct vertices of a point's closure.
    fn closure_vertices(&self, p: Point) -> Vec<u32> {
        let mut v: Vec<u32> = match p.stratum {
            Stratum::Vertex => vec![p.index],
            Stratum::Edge | Stratum::Face => self.cone(p).indices.to_vec(),
            Stratum::Cell => {
                let c = self.cone(p);
                c.indices.iter().flat_map(|&f| self.cone(Point::face(f)).indices.iter().copied()).collect()
            }
        };
        v.sort_unstable();
        v.dedup();
        v
    }
}

/// Coordinates and per-cell `(shape, degree, basis)`.
pub trait MeshGeometry {
    /// Vertex coordinates as an SoA triple `[x, y, z]`, indexed by vertex index.
    fn coords_soa(&self) -> [&[f64]; 3];
    /// Coordinates of a node (vertex or extra high-order node).
    fn node_coord(&self, n: NodeRef) -> Point3;
    /// `(degree, basis)` of a cell's geometry map.
    fn cell_geometry(&self, cell: Point) -> CellGeometry;
    /// High-order node list of a cell in the basis' reference order; empty for degree 1
    /// (the corners are then the cell's vertices).
    fn cell_nodes(&self, cell: Point) -> &[NodeRef];
    /// Id of the versioned face/cell geometry rule used by derived quantities.
    fn geometry_rule(&self) -> &str {
        GEOMETRY_RULE_DEFAULT
    }
    /// Coordinates of vertex `v`.
    fn vertex_coord(&self, v: u32) -> Point3 {
        let [x, y, z] = self.coords_soa();
        Point3::new(x[v as usize], y[v as usize], z[v as usize])
    }
}

/// Default geometry rule id (addendum r2 §7.5).
pub const GEOMETRY_RULE_DEFAULT: &str = "smartmesh.fan_vertex_mean.v1";

/// Parametric coordinates of a mesh entity on its classifying model entity.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum Param {
    /// No parametric data (e.g. classified on a vertex or a volume).
    #[default]
    None,
    /// Parameter `t` on a curve.
    Curve(f64),
    /// Parameters `(u, v)` on a surface.
    Surface([f64; 2]),
}

/// Classification of a mesh entity onto a model/partition entity.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Classification {
    /// The model entity: `Topo*` or `PartitionCell`.
    pub entity: EntityId,
    /// Parametric location on that entity, when known.
    pub param: Param,
}

impl Classification {
    /// Classification without parametric coordinates.
    pub const fn on(entity: EntityId) -> Self {
        Self { entity, param: Param::None }
    }
}

/// Dimension of a model entity id (`TopoVertex`=0 .. `TopoVolume`/`PartitionCell`=3).
pub fn model_dim(id: EntityId) -> Option<u8> {
    match id.kind() {
        EntityKind::TopoVertex => Some(0),
        EntityKind::TopoEdge => Some(1),
        EntityKind::TopoFace => Some(2),
        EntityKind::TopoVolume | EntityKind::PartitionCell => Some(3),
        _ => None,
    }
}

/// Mesh -> model/partition classification (the cross-layer key).
pub trait MeshClassification {
    /// Classification of a point, if set.
    fn classification(&self, p: Point) -> Option<Classification>;
    /// All points classified on `model`, in canonical order (inverse index).
    fn points_on(&self, model: EntityId) -> Vec<Point>;
    /// Distinct classifying model entities, sorted by id.
    fn classified_entities(&self) -> Vec<EntityId>;
}

/// Labels: named integer-valued secondary indexes over points (DMPlex `DMLabel`).
///
/// A *set* is a label stratum (`label_points(name, value)`); boundary sets are labels over
/// faces. Names are iterated in sorted order.
pub trait MeshLabels {
    /// Label names, sorted.
    fn label_names(&self) -> Vec<String>;
    /// Value of label `name` at `p`.
    fn label_value(&self, name: &str, p: Point) -> Option<i32>;
    /// Points with `value` under `name`, in canonical order.
    fn label_points(&self, name: &str, value: i32) -> Vec<Point>;
    /// All `(point, value)` entries of `name` in canonical order.
    fn label_entries(&self, name: &str) -> Vec<(Point, i32)>;
}

/// Everything a consumer can read from one mesh.
pub trait MeshView: MeshTopology + MeshGeometry + MeshClassification + MeshLabels {}
impl<T: MeshTopology + MeshGeometry + MeshClassification + MeshLabels + ?Sized> MeshView for T {}

/// One vertex loop of a face, wound so that its right-hand normal points *out of* the cell.
pub type FaceLoop = SmallVec<[Point; 4]>;

/// Description of a cell handed to [`MeshEdit::add_cell`].
#[derive(Clone, Debug)]
pub struct CellSpec {
    /// Declared shape tag; checked against the topology.
    pub shape: CellShape,
    /// One outward-wound vertex loop per face. Faces already present (same vertex cycle,
    /// opposite winding) are shared; this is how conformity is expressed.
    pub faces: SmallVec<[FaceLoop; 6]>,
    /// Classification of the cell.
    pub class: Option<Classification>,
}

/// Result of [`MeshEdit::add_cell`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CellAdded {
    /// The new cell.
    pub cell: Point,
    /// The faces of the cell, in the order of `CellSpec::faces`.
    pub faces: SmallVec<[Point; 6]>,
}

/// Build/edit interface for meshers and modifiers.
///
/// Contract: cells are appended with strictly increasing ids, so a face created by an earlier
/// cell has that cell as `left` and a later cell sharing it becomes `right`; callers only
/// supply outward-wound loops and the implementation applies the orientation rule.
pub trait MeshEdit {
    /// What [`MeshEdit::finish`] produces.
    type Output;
    /// Append a vertex.
    fn add_vertex(&mut self, pos: Point3, class: Option<Classification>) -> Point;
    /// Append a cell, creating or sharing its faces.
    fn add_cell(&mut self, spec: &CellSpec) -> Result<CellAdded, MeshError>;
    /// Append a high-order geometry node.
    fn add_node(&mut self, pos: Point3) -> NodeRef;
    /// Declare a cell's `(degree, basis)` and its node list.
    fn set_cell_order(&mut self, cell: Point, degree: u8, basis: Basis, nodes: &[NodeRef]) -> Result<(), MeshError>;
    /// Classify a vertex, face or cell (edges: use [`MeshEdit::classify_edge`]).
    fn classify(&mut self, p: Point, class: Classification) -> Result<(), MeshError>;
    /// Classify the edge joining two vertices (resolved at `finish`).
    fn classify_edge(&mut self, a: Point, b: Point, class: Classification) -> Result<(), MeshError>;
    /// Set label `name` at a vertex, face or cell.
    fn set_label(&mut self, name: &str, p: Point, value: i32) -> Result<(), MeshError>;
    /// Freeze into the output, building all derived adjacency in canonical order.
    fn finish(self) -> Result<Self::Output, MeshError>;
}
