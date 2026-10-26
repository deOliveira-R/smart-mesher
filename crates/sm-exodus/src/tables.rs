//! Element-type names and side numbering: our template face order to the Exodus side numbers.
//!
//! Our templates (`CellShape::local_faces`) already list nodes in Exodus corner order, so
//! connectivity needs no permutation; only faces are numbered differently. Sides are 1-based.
//!
//! | shape | our face index -> Exodus side | Exodus side -> nodes (1-based) |
//! |---|---|---|
//! | HEX8 | `[0,1,2,3,4,5] -> [5,6,1,2,3,4]` | 1: 1,2,6,5 / 2: 2,3,7,6 / 3: 3,4,8,7 / 4: 1,5,8,4 / 5: 1,4,3,2 / 6: 5,6,7,8 |
//! | TET4 | `[0,1,2,3] -> [4,1,2,3]` | 1: 1,2,4 / 2: 2,3,4 / 3: 1,4,3 / 4: 1,3,2 |
//! | WEDGE6 | `[0,1,2,3,4] -> [4,5,1,2,3]` | 1: 1,2,5,4 / 2: 2,3,6,5 / 3: 1,4,6,3 / 4: 1,3,2 / 5: 4,5,6 |
//! | PYRAMID5 | `[0,1,2,3,4] -> [5,1,2,3,4]` | 1: 1,2,5 / 2: 2,3,5 / 3: 3,4,5 / 4: 1,5,4 / 5: 1,4,3,2 |
//! | QUAD4, TRI3 | edge `i -> i + 1` | edge `i` runs from node `i + 1` to node `i + 2` (cyclic) |
//! | NFACED, NSIDED | 1-based position in the face (edge) list as written | |
//!
//! The right-hand column is the Exodus II manual's numbering; `tests/sides.rs` checks the table
//! against it by geometry, with the node lists typed independently of this module.

use sm_mesh_api::CellShape;

const HEX_SIDE: [u8; 6] = [5, 6, 1, 2, 3, 4];
const TET_SIDE: [u8; 4] = [4, 1, 2, 3];
const WEDGE_SIDE: [u8; 5] = [4, 5, 1, 2, 3];
const PYR_SIDE: [u8; 5] = [5, 1, 2, 3, 4];
const TRI_SIDE: [u8; 3] = [1, 2, 3];
const QUAD_SIDE: [u8; 4] = [1, 2, 3, 4];

const fn side_table(shape: CellShape) -> &'static [u8] {
    match shape {
        CellShape::Hex => &HEX_SIDE,
        CellShape::Tet => &TET_SIDE,
        CellShape::Wedge => &WEDGE_SIDE,
        CellShape::Pyramid => &PYR_SIDE,
        CellShape::Tri => &TRI_SIDE,
        CellShape::Quad => &QUAD_SIDE,
        _ => &[],
    }
}

/// Exodus side number (1-based) of local face `local_face` of a template shape
/// (`CellShape::local_faces` order). `None` for polyhedra and polygons (their side is the
/// position in the written face list) and for an out-of-range face.
pub const fn exodus_side(shape: CellShape, local_face: usize) -> Option<u8> {
    let t = side_table(shape);
    if local_face < t.len() { Some(t[local_face]) } else { None }
}

/// Inverse of [`exodus_side`]: the index into `CellShape::local_faces()` of Exodus side `side`.
pub const fn local_face(shape: CellShape, side: u8) -> Option<usize> {
    let t = side_table(shape);
    let mut i = 0;
    while i < t.len() {
        if t[i] == side {
            return Some(i);
        }
        i += 1;
    }
    None
}

/// Exodus spelling of the element type of a shape: `TET4`, `HEX8`, `WEDGE6`, `PYRAMID5`, `TRI3`,
/// `QUAD4`, `NSIDED` (polygon), `NFACED` (polyhedron). Internal and `.smsh` 1.1 names are
/// `POLYGON` / `POLYHEDRON`; those never reach the file.
pub const fn exodus_element_type(shape: CellShape) -> &'static str {
    match shape {
        CellShape::Tet => "TET4",
        CellShape::Hex => "HEX8",
        CellShape::Wedge => "WEDGE6",
        CellShape::Pyramid => "PYRAMID5",
        CellShape::Tri => "TRI3",
        CellShape::Quad => "QUAD4",
        CellShape::Polygon => "NSIDED",
        _ => "NFACED",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tables_are_bijections() {
        for s in [CellShape::Hex, CellShape::Tet, CellShape::Wedge, CellShape::Pyramid, CellShape::Tri, CellShape::Quad] {
            let n = s.local_faces().len();
            for k in 0..n {
                let side = exodus_side(s, k).expect("side");
                assert_eq!(local_face(s, side), Some(k));
            }
            assert_eq!(exodus_side(s, n), None);
            assert_eq!(local_face(s, 0), None);
            assert_eq!(local_face(s, n as u8 + 1), None);
        }
        assert_eq!(exodus_side(CellShape::Polyhedron, 0), None);
        assert_eq!(exodus_element_type(CellShape::Polyhedron), "NFACED");
    }
}
