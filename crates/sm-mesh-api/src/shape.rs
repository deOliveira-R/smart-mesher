//! Shape tags, basis kinds and node references.
//!
//! Polyhedra are the general case. Tags exist so consumers can take a fast path; they are
//! always derivable from topology (see [`CellShape::detect`]) and validated against it.

/// Shape tag of a cell. [`CellShape::Polyhedron`] is the general case.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum CellShape {
    /// 4 vertices, 4 triangles.
    Tet = 0,
    /// 8 vertices, 6 quads.
    Hex = 1,
    /// Triangular prism: 6 vertices, 2 triangles + 3 quads.
    Wedge = 2,
    /// 5 vertices, 1 quad + 4 triangles.
    Pyramid = 3,
    /// Any closed polyhedron.
    Polyhedron = 4,
}

impl CellShape {
    /// `(vertices, faces)` of a standard shape, `None` for [`CellShape::Polyhedron`].
    pub const fn counts(self) -> Option<(usize, usize)> {
        match self {
            CellShape::Tet => Some((4, 4)),
            CellShape::Hex => Some((8, 6)),
            CellShape::Wedge => Some((6, 5)),
            CellShape::Pyramid => Some((5, 5)),
            CellShape::Polyhedron => None,
        }
    }

    /// Derive the tag from the number of distinct vertices and the face sizes.
    ///
    /// Counts alone identify the five shapes among closed polyhedra with these face
    /// multisets (a topological hex is any 8-vertex, 6-quad closed cell).
    pub fn detect(n_vertices: usize, face_sizes: &[usize]) -> Self {
        let tri = face_sizes.iter().filter(|&&n| n == 3).count();
        let quad = face_sizes.iter().filter(|&&n| n == 4).count();
        let nf = face_sizes.len();
        match (n_vertices, nf, tri, quad) {
            (4, 4, 4, 0) => CellShape::Tet,
            (8, 6, 0, 6) => CellShape::Hex,
            (6, 5, 2, 3) => CellShape::Wedge,
            (5, 5, 4, 1) => CellShape::Pyramid,
            _ => CellShape::Polyhedron,
        }
    }
}

/// Shape tag of a face, derived from its loop length.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum FaceShape {
    /// Triangle.
    Tri,
    /// Quadrilateral.
    Quad,
    /// General polygon with `n >= 5` vertices.
    Polygon,
}

impl FaceShape {
    /// Tag for a loop of `n` vertices.
    pub const fn from_len(n: usize) -> Self {
        match n {
            3 => FaceShape::Tri,
            4 => FaceShape::Quad,
            _ => FaceShape::Polygon,
        }
    }
}

/// Polynomial basis family of a high-order cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Basis {
    /// Lagrange with equispaced nodes.
    LagrangeEquispaced,
    /// Lagrange with Gauss-Lobatto-Legendre nodes.
    LagrangeGll,
    /// Bernstein-Bezier control points.
    Bernstein,
}

/// Per-cell geometric representation: `(degree, basis)`; the shape is stored separately.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct CellGeometry {
    /// Polynomial degree of the geometry map; 1 means straight-sided.
    pub degree: u8,
    /// Basis family (ignored when `degree == 1`).
    pub basis: Basis,
}

impl CellGeometry {
    /// Straight-sided cell (degree 1).
    pub const LINEAR: Self = Self { degree: 1, basis: Basis::LagrangeEquispaced };
}

/// Reference to a geometry node: either a mesh vertex or an extra (non-vertex) node.
///
/// Vertices keep their own coordinates; extra nodes carry high-order geometry. The
/// encoding is stable under appends.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct NodeRef(u32);

impl NodeRef {
    const EXTRA: u32 = 1 << 31;
    /// Reference to vertex `v`. Panics in debug builds if `v >= 2^31`.
    pub const fn vertex(v: u32) -> Self {
        debug_assert!(v < Self::EXTRA);
        Self(v)
    }
    /// Reference to extra node `i`. Panics in debug builds if `i >= 2^31`.
    pub const fn extra(i: u32) -> Self {
        debug_assert!(i < Self::EXTRA);
        Self(i | Self::EXTRA)
    }
    /// `Some(v)` if this is a vertex node.
    pub const fn as_vertex(self) -> Option<u32> {
        if self.0 & Self::EXTRA == 0 { Some(self.0) } else { None }
    }
    /// `Some(i)` if this is an extra node.
    pub const fn as_extra(self) -> Option<u32> {
        if self.0 & Self::EXTRA != 0 { Some(self.0 & !Self::EXTRA) } else { None }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detect_shapes() {
        assert_eq!(CellShape::detect(4, &[3, 3, 3, 3]), CellShape::Tet);
        assert_eq!(CellShape::detect(8, &[4; 6]), CellShape::Hex);
        assert_eq!(CellShape::detect(6, &[3, 3, 4, 4, 4]), CellShape::Wedge);
        assert_eq!(CellShape::detect(5, &[4, 3, 3, 3, 3]), CellShape::Pyramid);
        assert_eq!(CellShape::detect(10, &[5, 5, 4, 4, 4, 4, 4]), CellShape::Polyhedron);
    }

    #[test]
    fn node_ref_roundtrip() {
        assert_eq!(NodeRef::vertex(7).as_vertex(), Some(7));
        assert_eq!(NodeRef::extra(7).as_extra(), Some(7));
        assert_eq!(NodeRef::extra(7).as_vertex(), None);
    }
}
