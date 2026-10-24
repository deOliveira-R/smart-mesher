//! High-order geometry: sparse per-cell `(degree, basis)` records and per-face curvature records.
//!
//! The coordinate arrays hold the `n_vertices` vertices first, then any extra (non-vertex)
//! nodes; both kinds are addressed by [`PointIx`].
//!
//! # Node order `smartmesh.nodes.v1`
//! For template shapes up to degree 2 the node list follows Gmsh ordering: the corners in
//! template order, then one node per local edge in [`CellShape::local_edges`] order, then (for
//! Lagrange bases only) one node per quadrilateral face in [`CellShape::local_quads`] order, then
//! (hex only, Lagrange only) the cell centre. [`node_layout`] spells the list out.
//! [`Basis::Serendipity`] drops the face and centre nodes (HEX20 against HEX27). Every other
//! `(shape, degree)` is unsupported in v1: [`node_count`] returns `None` and builders refuse it.
//! GLL and equispaced Lagrange coincide at degree 2.
//!
//! Face curves ([`FaceCurve`]) list the face's loop vertices first (in loop order), then one node
//! per loop edge (edge `i` joins loop vertices `i` and `i+1`), then, for a Lagrange quad, the
//! centre.

use crate::index::PointIx;
use crate::shape::{CellShape, FaceShape};

/// Polynomial basis family of high-order geometry.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum Basis {
    /// Full Lagrange with equispaced nodes.
    LagrangeEquispaced,
    /// Full Lagrange with Gauss-Lobatto-Legendre nodes.
    LagrangeGll,
    /// Serendipity: corner and edge nodes only (HEX20, WEDGE15, PYRAMID13).
    Serendipity,
}

impl Basis {
    /// Stable small integer for hashing and codecs.
    pub const fn code(self) -> u8 {
        match self {
            Basis::LagrangeEquispaced => 0,
            Basis::LagrangeGll => 1,
            Basis::Serendipity => 2,
        }
    }
}

/// Geometric order of one cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct CellGeometry {
    /// Shape tag.
    pub shape: CellShape,
    /// Polynomial degree (1 = linear).
    pub degree: u8,
    /// Basis family (meaningful when `degree > 1`).
    pub basis: Basis,
}

/// What one node of a high-order cell sits on (`smartmesh.nodes.v1`). `#[non_exhaustive]`
/// (W0f) so triangle-face and higher-order edge slots can be added.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum NodeSlot {
    /// A template corner.
    Corner(u8),
    /// The midpoint of the local edge joining two local corners.
    Edge(u8, u8),
    /// The centre of the local quadrilateral face with these four local corners.
    Face([u8; 4]),
    /// The cell centre.
    Interior,
}

/// Number of nodes of a cell of `shape` at `degree` in `basis`; `None` if unsupported in v1
/// (polyhedra and polygons, degree 0 or above 2, serendipity at degree 1 is the linear
/// element). Dimension-2 templates follow Gmsh too: TRI6, QUAD8 (serendipity), QUAD9.
pub const fn node_count(shape: CellShape, degree: u8, basis: Basis) -> Option<usize> {
    let Some(corners) = shape.vertex_count() else {
        return None;
    };
    match degree {
        1 => Some(corners),
        2 => {
            let edges = shape.local_edges().len();
            Some(match basis {
                Basis::Serendipity => corners + edges,
                _ => {
                    corners
                        + edges
                        + shape.local_quads().len()
                        + matches!(shape, CellShape::Hex) as usize
                }
            })
        }
        _ => None,
    }
}

/// The slots of every node, in order, for a supported `(shape, degree, basis)`.
pub fn node_layout(shape: CellShape, degree: u8, basis: Basis) -> Option<Vec<NodeSlot>> {
    let n = node_count(shape, degree, basis)?;
    let mut v = Vec::with_capacity(n);
    v.extend((0..shape.vertex_count()? as u8).map(NodeSlot::Corner));
    if degree == 2 {
        v.extend(
            shape
                .local_edges()
                .iter()
                .map(|e| NodeSlot::Edge(e[0], e[1])),
        );
        if basis != Basis::Serendipity {
            v.extend(shape.local_quads().iter().map(|q| NodeSlot::Face(*q)));
            if shape == CellShape::Hex {
                v.push(NodeSlot::Interior);
            }
        }
    }
    debug_assert_eq!(v.len(), n);
    Some(v)
}

/// Number of nodes of a curved face with `loop_len` vertices; `None` if unsupported in v1.
///
/// Degree 2 only: triangle 6; quadrilateral 9 (Lagrange) or 8 (serendipity); polygon with
/// `n >= 5` vertices `2n` (serendipity only).
pub const fn face_node_count(loop_len: usize, degree: u8, basis: Basis) -> Option<usize> {
    if degree != 2 || loop_len < 3 {
        return None;
    }
    match (FaceShape::from_vertex_count(loop_len), basis) {
        (FaceShape::Tri, _) => Some(6),
        (FaceShape::Quad, Basis::Serendipity) => Some(8),
        (FaceShape::Quad, _) => Some(9),
        (FaceShape::Polygon, Basis::Serendipity) => Some(2 * loop_len),
        (FaceShape::Polygon, _) | (FaceShape::Segment, _) => None,
    }
}

/// Sparse high-order record of one cell. `nodes` lists every node (corners included, as
/// vertex-numbered points) in `smartmesh.nodes.v1` order; the first entries equal the cell's
/// [`crate::MeshRead::cell_corners`].
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct HighOrderCell {
    /// Polynomial degree (`>= 2` in a stored record).
    pub degree: u8,
    /// Basis family.
    pub basis: Basis,
    /// Node points, `smartmesh.nodes.v1` order.
    pub nodes: Vec<PointIx>,
}

impl HighOrderCell {
    /// A record of `degree` and `basis` with its nodes in `smartmesh.nodes.v1` order.
    pub fn new(degree: u8, basis: Basis, nodes: Vec<PointIx>) -> Self {
        Self { degree, basis, nodes }
    }
}

/// Borrowed curvature record of one face (the sparse side table of addendum 7.5).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaceCurve<'a> {
    /// Polynomial degree (2 in v1).
    pub degree: u8,
    /// Basis family.
    pub basis: Basis,
    /// Node points: loop vertices first, then edge nodes, then the centre (Lagrange quad).
    pub nodes: &'a [PointIx],
}

/// A node reference used while building, before final point numbering exists.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub enum NodeRef {
    /// An ordinary mesh vertex.
    Vertex(crate::index::VertexIx),
    /// The n-th extra node added to the builder.
    Extra(u32),
}
