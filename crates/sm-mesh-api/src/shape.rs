//! Shape tags (derived from topology) and the fast-path vertex templates.
//!
//! Polyhedra are the general case. A shape tag is a *derived* label checked by the validator;
//! it never carries information the faces do not. Fast-path cells additionally keep an ordered
//! vertex list (see [`crate::MeshRead::cell_ordered_vertices`]) in the templates below.
//!
//! Template conventions (faces are listed with OUTWARD loops, right-hand rule):
//! - Tet: 0,1,2 is the base, 3 the apex; with 3 above, 0,1,2 is counter-clockwise from +z.
//! - Hex: 0..3 bottom (counter-clockwise from +z), 4..7 the top ring above 0..3.
//! - Wedge: 0..2 bottom triangle (counter-clockwise from +z), 3..5 the top above them.
//! - Pyramid: 0..3 base (counter-clockwise from +z), 4 apex.

use crate::index::VertexIx;

/// Shape tag of a 3D cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum CellShape {
    /// 4 triangles, 4 vertices.
    Tet,
    /// 6 quads, 8 vertices.
    Hex,
    /// 2 triangles + 3 quads, 6 vertices.
    Wedge,
    /// 1 quad + 4 triangles, 5 vertices.
    Pyramid,
    /// Anything else (general closed polyhedron).
    Polyhedron,
}

/// Shape tag of a face.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum FaceShape {
    /// Triangle.
    Tri,
    /// Quadrilateral.
    Quad,
    /// General polygon with five or more vertices.
    Polygon,
}

impl FaceShape {
    /// Tag for a loop of `n` vertices (`n >= 3`).
    pub const fn from_vertex_count(n: usize) -> Self {
        match n {
            3 => FaceShape::Tri,
            4 => FaceShape::Quad,
            _ => FaceShape::Polygon,
        }
    }
}

const TET_FACES: &[&[u8]] = &[&[0, 2, 1], &[0, 1, 3], &[1, 2, 3], &[2, 0, 3]];
const HEX_FACES: &[&[u8]] =
    &[&[0, 3, 2, 1], &[4, 5, 6, 7], &[0, 1, 5, 4], &[1, 2, 6, 5], &[2, 3, 7, 6], &[3, 0, 4, 7]];
const WEDGE_FACES: &[&[u8]] = &[&[0, 2, 1], &[3, 4, 5], &[0, 1, 4, 3], &[1, 2, 5, 4], &[2, 0, 3, 5]];
const PYR_FACES: &[&[u8]] = &[&[0, 3, 2, 1], &[0, 1, 4], &[1, 2, 4], &[2, 3, 4], &[3, 0, 4]];

impl CellShape {
    /// Number of vertices of a fast-path shape; `None` for [`CellShape::Polyhedron`].
    pub const fn vertex_count(self) -> Option<usize> {
        match self {
            CellShape::Tet => Some(4),
            CellShape::Hex => Some(8),
            CellShape::Wedge => Some(6),
            CellShape::Pyramid => Some(5),
            CellShape::Polyhedron => None,
        }
    }

    /// Local outward face loops (indices into the ordered vertex list). Empty for polyhedra.
    pub const fn local_faces(self) -> &'static [&'static [u8]] {
        match self {
            CellShape::Tet => TET_FACES,
            CellShape::Hex => HEX_FACES,
            CellShape::Wedge => WEDGE_FACES,
            CellShape::Pyramid => PYR_FACES,
            CellShape::Polyhedron => &[],
        }
    }

    /// Permutation `new[k] = old[perm[k]]` that restores a right-handed ordering after an
    /// improper transform. `None` for polyhedra.
    pub const fn reflect_permutation(self) -> Option<&'static [u8]> {
        match self {
            CellShape::Tet => Some(&[0, 2, 1, 3]),
            CellShape::Hex => Some(&[0, 3, 2, 1, 4, 7, 6, 5]),
            CellShape::Wedge => Some(&[0, 2, 1, 3, 5, 4]),
            CellShape::Pyramid => Some(&[0, 3, 2, 1, 4]),
            CellShape::Polyhedron => None,
        }
    }

    /// Classify a cell from its face sizes and distinct-vertex count.
    pub fn detect(face_sizes: impl IntoIterator<Item = usize>, n_vertices: usize) -> Self {
        let (mut tris, mut quads, mut other) = (0usize, 0usize, 0usize);
        for s in face_sizes {
            match s {
                3 => tris += 1,
                4 => quads += 1,
                _ => other += 1,
            }
        }
        match (tris, quads, other, n_vertices) {
            (4, 0, 0, 4) => CellShape::Tet,
            (0, 6, 0, 8) => CellShape::Hex,
            (2, 3, 0, 6) => CellShape::Wedge,
            (4, 1, 0, 5) => CellShape::Pyramid,
            _ => CellShape::Polyhedron,
        }
    }

    /// Shape implied by an ordered vertex count (fast-path shapes only; 5 means pyramid).
    pub const fn from_ordered_len(n: usize) -> Option<Self> {
        match n {
            4 => Some(CellShape::Tet),
            5 => Some(CellShape::Pyramid),
            6 => Some(CellShape::Wedge),
            8 => Some(CellShape::Hex),
            _ => None,
        }
    }
}

/// How one closed vertex loop relates to another with the same vertex set.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoopRelation {
    /// Same cyclic order (same normal).
    Same,
    /// Opposite cyclic order (opposite normal).
    Reversed,
}

/// Compare two loops as cycles. `None` if they are not the same cycle in either direction.
pub fn loop_relation(a: &[VertexIx], b: &[VertexIx]) -> Option<LoopRelation> {
    let n = a.len();
    if n != b.len() || n == 0 {
        return None;
    }
    let s = b.iter().position(|&v| v == a[0])?;
    if (0..n).all(|i| a[i] == b[(s + i) % n]) {
        return Some(LoopRelation::Same);
    }
    if (0..n).all(|i| a[i] == b[(s + n - i) % n]) {
        return Some(LoopRelation::Reversed);
    }
    None
}
