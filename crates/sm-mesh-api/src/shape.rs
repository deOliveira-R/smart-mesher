//! Shape tags, bases and the local face templates behind the fast paths.
//!
//! Polyhedra are the general case; `Tet`/`Hex`/`Wedge`/`Pyramid` are *derived tags* that add a
//! corner ordering (and therefore cheap reference-element algorithms) on top of the same
//! face-first tables. Faces are `Tri`/`Quad` fast paths or general polygons.

use serde::{Deserialize, Serialize};

/// Cell shape tag.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum CellShape {
    /// Tetrahedron, 4 corners.
    Tet,
    /// Hexahedron, 8 corners (VTK order: bottom quad ccw, then top quad).
    Hex,
    /// Triangular prism, 6 corners (bottom tri, top tri).
    Wedge,
    /// Pyramid, 5 corners (base quad ccw seen from the side opposite the apex, then apex).
    Pyramid,
    /// General polyhedron: faces only, no corner ordering.
    Polyhedron,
}

// Local face loops, outward by the right-hand rule, on the reference corner order.
const TET_FACES: &[&[u8]] = &[&[0, 2, 1], &[0, 1, 3], &[1, 2, 3], &[0, 3, 2]];
const HEX_FACES: &[&[u8]] =
    &[&[0, 3, 2, 1], &[4, 5, 6, 7], &[0, 1, 5, 4], &[1, 2, 6, 5], &[2, 3, 7, 6], &[3, 0, 4, 7]];
const WEDGE_FACES: &[&[u8]] = &[&[0, 2, 1], &[3, 4, 5], &[0, 1, 4, 3], &[1, 2, 5, 4], &[2, 0, 3, 5]];
const PYRAMID_FACES: &[&[u8]] = &[&[0, 3, 2, 1], &[0, 1, 4], &[1, 2, 4], &[2, 3, 4], &[3, 0, 4]];

impl CellShape {
    /// Short lowercase name.
    pub const fn name(self) -> &'static str {
        match self {
            CellShape::Tet => "tet",
            CellShape::Hex => "hex",
            CellShape::Wedge => "wedge",
            CellShape::Pyramid => "pyramid",
            CellShape::Polyhedron => "polyhedron",
        }
    }

    /// Number of corner vertices; `None` for polyhedra.
    pub const fn num_corners(self) -> Option<usize> {
        match self {
            CellShape::Tet => Some(4),
            CellShape::Hex => Some(8),
            CellShape::Wedge => Some(6),
            CellShape::Pyramid => Some(5),
            CellShape::Polyhedron => None,
        }
    }

    /// Outward face loops in local corner indices; empty for polyhedra.
    pub const fn local_faces(self) -> &'static [&'static [u8]] {
        match self {
            CellShape::Tet => TET_FACES,
            CellShape::Hex => HEX_FACES,
            CellShape::Wedge => WEDGE_FACES,
            CellShape::Pyramid => PYRAMID_FACES,
            CellShape::Polyhedron => &[],
        }
    }

    /// Corner permutation `sigma` such that `corners'[i] = corners[sigma[i]]` turns a
    /// mirror-imaged (left-handed) cell back into a valid right-handed template.
    pub const fn mirror_permutation(self) -> Option<&'static [u8]> {
        match self {
            CellShape::Tet => Some(&[0, 2, 1, 3]),
            CellShape::Hex => Some(&[0, 3, 2, 1, 4, 7, 6, 5]),
            CellShape::Wedge => Some(&[0, 2, 1, 3, 5, 4]),
            CellShape::Pyramid => Some(&[0, 3, 2, 1, 4]),
            CellShape::Polyhedron => None,
        }
    }

    /// Number of Lagrange nodes at `degree >= 1`; `None` for polyhedra.
    pub const fn num_nodes(self, degree: u8) -> Option<usize> {
        let p = degree as usize;
        if p == 0 {
            return None;
        }
        match self {
            CellShape::Tet => Some((p + 1) * (p + 2) * (p + 3) / 6),
            CellShape::Hex => Some((p + 1) * (p + 1) * (p + 1)),
            CellShape::Wedge => Some((p + 1) * (p + 1) * (p + 2) / 2),
            CellShape::Pyramid => Some((p + 1) * (p + 2) * (2 * p + 3) / 6),
            CellShape::Polyhedron => None,
        }
    }
}

/// Face shape fast path, derived from the loop length.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum FaceShape {
    /// Triangle.
    Tri,
    /// Quadrilateral.
    Quad,
    /// General polygon with this many vertices.
    Polygon(u32),
}

impl FaceShape {
    /// Classify a loop of `n` vertices.
    pub const fn from_len(n: usize) -> Self {
        match n {
            3 => FaceShape::Tri,
            4 => FaceShape::Quad,
            _ => FaceShape::Polygon(n as u32),
        }
    }
}

/// Polynomial basis of a high-order cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize)]
pub enum Basis {
    /// Lagrange on equispaced points (degree 1 = the linear element).
    #[default]
    LagrangeEquispaced,
    /// Lagrange on Gauss-Lobatto-Legendre points.
    LagrangeGll,
    /// Bernstein-Bezier control points.
    Bernstein,
}

/// The `(shape, degree, basis)` triple of a cell (differentiator E).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CellSpec {
    /// Shape tag.
    pub shape: CellShape,
    /// Polynomial degree of the geometry map (1 = straight-sided).
    pub degree: u8,
    /// Basis of the geometry map.
    pub basis: Basis,
}

impl CellSpec {
    /// A straight-sided cell of `shape`.
    pub const fn linear(shape: CellShape) -> Self {
        Self { shape, degree: 1, basis: Basis::LagrangeEquispaced }
    }
}
