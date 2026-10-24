//! Decomposition into sweep pieces: colors preserved, undo restores ids, ids never reused.

mod common;

use common::*;
use sm_core::{EntityId, Point3, Vector3};
use sm_model_api::*;
use sm_topo_api::fixtures::{self, BrickIds, with_op};
use sm_topo_api::{CellComplex, OpId};

fn setup() -> (CellComplex, Model, BrickIds, EntityId) {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [2.0, 1.0, 1.0])).unwrap();
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[b.volume]).unwrap();
    finish_boundaries(&mut m, &t, 7, true);
    (t, m, b, mat)
}

fn cut_plane() -> CutGeometry {
    CutGeometry::Plane(Plane::new(Vector3::new(1.0, 0.0, 0.0), 1.0).unwrap())
}

#[test]
fn split_into_two_sweep_pieces_preserves_colors_and_validates() {
    let (t, mut m, b, mat) = setup();
    let colors_before = (m.volume_color(b.volume), m.face_color(b.faces[0]));
    assert_eq!(m.pieces_of_volume(b.volume).as_slice(), &[PieceRef::Whole(b.volume)], "implicit piece, nothing minted");
    // -x face is only on the negative part, +x only on the positive one; the four sides straddle.
    let neg = [b.faces[0], b.faces[2], b.faces[3], b.faces[4], b.faces[5]];
    let pos = [b.faces[1], b.faces[2], b.faces[3], b.faces[4], b.faces[5]];
    let [n, p] = m.split(PieceRef::Whole(b.volume), cut_plane(), &neg, &pos, &t).unwrap();
    assert!(n < p);
    assert_eq!((n.kind(), p.kind()), (sm_core::EntityKind::PartitionCell, sm_core::EntityKind::PartitionCell));
    assert_eq!(m.pieces_of_volume(b.volume).as_slice(), &[PieceRef::Cell(n), PieceRef::Cell(p)]);
    assert_eq!(m.pieces_of_face(b.faces[0]).as_slice(), &[PieceRef::Cell(n)]);
    assert_eq!(m.pieces_of_face(b.faces[2]).as_slice(), &[PieceRef::Cell(n), PieceRef::Cell(p)]);
    assert_eq!(m.pieces_of_face_in(b.faces[1], &t).as_slice(), &[PieceRef::Cell(p)]);
    // Piece faces: whole where one piece, Part where straddling, plus the cut face.
    let pn = m.decomposition().piece(n).unwrap();
    assert!(pn.faces.contains(&PieceFace::Whole(b.faces[0])));
    assert!(pn.faces.contains(&PieceFace::Part { face: b.faces[2], piece: n }));
    assert!(pn.faces.iter().any(|f| matches!(f, PieceFace::Cut { side: Keep::Negative, .. })));
    // Sweep from the -x face to the cut face, and from the cut face to the +x face.
    let cut = pn.faces.iter().find_map(|f| if let PieceFace::Cut { cut, .. } = f { Some(*cut) } else { None }).unwrap();
    m.set_shape(n, PieceShape::sweep(vec![PieceFace::Whole(b.faces[0])], vec![PieceFace::Cut { cut, side: Keep::Negative }])).unwrap();
    m.set_shape(p, PieceShape::sweep(vec![PieceFace::Cut { cut, side: Keep::Positive }], vec![PieceFace::Whole(b.faces[1])])).unwrap();
    // Colors: pieces read through the volume; nothing changed.
    assert_eq!(m.piece_color(PieceRef::Cell(n)), Some(mat));
    assert_eq!(m.piece_color(PieceRef::Cell(p)), Some(mat));
    assert_eq!(m.piece_volume(PieceRef::Cell(n)), b.volume);
    assert_eq!((m.volume_color(b.volume), m.face_color(b.faces[0])), colors_before);
    let r = validate(&m, &t, Tolerance::default());
    assert!(r.is_ok(), "{:#?}", r.issues);
    // A sweep whose source and target share an edge is rejected.
    m.set_shape(n, PieceShape::sweep(vec![PieceFace::Whole(b.faces[0])], vec![PieceFace::Part { face: b.faces[2], piece: n }])).unwrap();
    assert!(validate(&m, &t, Tolerance::default()).has(IssueCode::SweepSourceTargetAdjacent));
}

#[test]
fn split_arguments_are_checked() {
    let (t, mut m, b, _) = setup();
    // A face on neither side, a foreign face, a bad cut geometry.
    assert!(m.split(PieceRef::Whole(b.volume), cut_plane(), &b.faces[..3], &b.faces[3..5], &t).is_err());
    let alien = EntityId::new(sm_core::EntityKind::TopoFace, 99).unwrap();
    assert!(m.split(PieceRef::Whole(b.volume), cut_plane(), &[alien], &b.faces, &t).is_err());
    assert!(m.split(PieceRef::Whole(b.volume), CutGeometry::Surface(b.faces[0]), &b.faces, &b.faces, &t).is_err());
    assert!(m.decomposition().cells().next().is_none(), "failed splits changed nothing");
}

#[test]
fn undo_restores_the_parent_id_and_never_reuses_child_ids() {
    let (t, mut m, b, _) = setup();
    let all = b.faces.to_vec();
    let [n1, p1] = m.split(PieceRef::Whole(b.volume), cut_plane(), &all, &all, &t).unwrap();
    // Split the negative child again, then undo bottom-up.
    let [n2, p2] = m.split(PieceRef::Cell(n1), CutGeometry::Plane(Plane::new(Vector3::new(1.0, 0.0, 0.0), 0.5).unwrap()), &all, &all, &t).unwrap();
    assert!(n2 > p1 && p2 > n2, "ids increase");
    assert_eq!(m.pieces_of_volume(b.volume).as_slice(), &[PieceRef::Cell(p1), PieceRef::Cell(n2), PieceRef::Cell(p2)]);
    assert_eq!(m.unsplit(p1), Err(ModelError::Invalid("both children must be unsplit leaves".into())), "p1's parent still has split children");
    assert_eq!(m.unsplit(n2).unwrap(), PieceRef::Cell(n1), "the parent keeps its original id");
    assert_eq!(m.pieces_of_volume(b.volume).as_slice(), &[PieceRef::Cell(n1), PieceRef::Cell(p1)]);
    assert_eq!(m.unsplit(p1).unwrap(), PieceRef::Whole(b.volume));
    assert_eq!(m.pieces_of_volume(b.volume).as_slice(), &[PieceRef::Whole(b.volume)]);
    assert!(m.decomposition().cells().next().is_none());
    // Re-splitting mints new ids, never the retired ones.
    let [n3, p3] = m.split(PieceRef::Whole(b.volume), cut_plane(), &all, &all, &t).unwrap();
    assert!(n3 > p2 && p3 > n3, "{n3} {p3}");
    assert!(![n1, p1, n2, p2].contains(&n3) && ![n1, p1, n2, p2].contains(&p3));
    m.clear_decomposition();
    let [n4, _] = m.split(PieceRef::Whole(b.volume), cut_plane(), &all, &all, &t).unwrap();
    assert!(n4 > p3, "clearing keeps the id counters");
}

#[test]
fn composite_merges_same_colored_pieces_only() {
    let mut t = CellComplex::new();
    let a = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])).unwrap();
    let b = with_op(&mut t, OpId(2), |t| fixtures::brick(t, Point3::new(1.0, 0.0, 0.0), [1.0, 1.0, 1.0])).unwrap();
    let c = with_op(&mut t, OpId(3), |t| fixtures::brick(t, Point3::new(2.0, 0.0, 0.0), [1.0, 1.0, 1.0])).unwrap();
    let mut m = Model::new();
    let red = m.add_color(1, "red", Domain::Volume).unwrap();
    let blue = m.add_color(2, "blue", Domain::Volume).unwrap();
    m.color_volumes(red, &[a.volume, b.volume]).unwrap();
    m.color_volumes(blue, &[c.volume]).unwrap();
    finish_boundaries(&mut m, &t, 7, true);
    let comp = m.composite(&[PieceRef::Whole(a.volume), PieceRef::Whole(b.volume)], &t).unwrap();
    assert_eq!(m.pieces_of_volume(a.volume).as_slice(), &[PieceRef::Cell(comp)]);
    assert_eq!(m.pieces_of_volume(b.volume).as_slice(), &[PieceRef::Cell(comp)]);
    assert_eq!(m.piece_color(PieceRef::Cell(comp)), Some(red));
    assert!(m.composite(&[PieceRef::Cell(comp), PieceRef::Whole(c.volume)], &t).is_err(), "mixed colors");
    assert!(m.composite(&[PieceRef::Whole(a.volume), PieceRef::Whole(c.volume)], &t).is_err(), "a is covered already");
    assert!(m.split(PieceRef::Cell(comp), cut_plane(), &a.faces, &a.faces, &t).is_err(), "composites are not split");
    assert!(validate(&m, &t, Tolerance::default()).is_ok());
    m.uncomposite(comp).unwrap();
    assert_eq!(m.pieces_of_volume(a.volume).as_slice(), &[PieceRef::Whole(a.volume)]);
    // Recoloring one member afterwards makes a stale composite visible.
    let comp2 = m.composite(&[PieceRef::Whole(a.volume), PieceRef::Whole(b.volume)], &t).unwrap();
    m.color_volumes(blue, &[b.volume]).unwrap();
    assert!(validate(&m, &t, Tolerance::default()).has(IssueCode::DecompositionChangedColoring));
    assert_eq!(m.piece_color(PieceRef::Cell(comp2)), None);
}

#[test]
fn topology_changes_of_decomposed_volumes_are_refused() {
    let (t, mut m, b, _) = setup();
    let all = b.faces.to_vec();
    m.split(PieceRef::Whole(b.volume), cut_plane(), &all, &all, &t).unwrap();
    let lineage = [sm_topo_api::Lineage { old: b.volume, new: [b.volume.serial()].iter().map(|s| EntityId::new(sm_core::EntityKind::TopoVolume, *s + 10).unwrap()).collect() }];
    let before = m.clone();
    assert_eq!(m.apply_lineage(&lineage, &t), Err(ModelError::DecompositionActive(b.volume)));
    assert_eq!(m, before);
}
