//! Frozen code tables of `CellShape`, `FaceKind` and `Basis`.

use sm_mesh_api::{Basis, CellShape, FaceKind};

const SHAPES: [CellShape; 8] = [
    CellShape::Tet,
    CellShape::Hex,
    CellShape::Wedge,
    CellShape::Pyramid,
    CellShape::Polyhedron,
    CellShape::Tri,
    CellShape::Quad,
    CellShape::Polygon,
];
const KINDS: [FaceKind; 4] =
    [FaceKind::Interior, FaceKind::Boundary, FaceKind::NonConformal, FaceKind::PartitionCut];
const BASES: [Basis; 3] = [Basis::LagrangeEquispaced, Basis::LagrangeGll, Basis::Serendipity];

#[test]
fn shape_codes_are_the_frozen_table() {
    for (i, s) in SHAPES.into_iter().enumerate() {
        assert_eq!(s.code() as usize, i);
        assert_eq!(s.code() as u64, s as u64);
        assert_eq!(CellShape::from_code(s.code()), Some(s));
    }
    assert_eq!(CellShape::from_code(8), None);
    assert_eq!(CellShape::from_code(255), None);
}

#[test]
fn face_kind_codes_are_the_frozen_table() {
    for (i, k) in KINDS.into_iter().enumerate() {
        assert_eq!(k.code() as usize, i);
        assert_eq!(k.code() as u64, k as u64);
        assert_eq!(FaceKind::from_code(k.code()), Some(k));
    }
    assert_eq!(FaceKind::from_code(4), None);
}

#[test]
fn basis_codes_round_trip() {
    for (i, b) in BASES.into_iter().enumerate() {
        assert_eq!(b.code() as usize, i);
        assert_eq!(Basis::from_code(b.code()), Some(b));
    }
    assert_eq!(Basis::from_code(3), None);
}
