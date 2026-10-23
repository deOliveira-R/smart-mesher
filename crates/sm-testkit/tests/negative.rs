//! Every checker fails on a deliberately broken input and passes on the good one.

use sm_core::Point3;
use sm_mesh_api::reference::ReferenceMesh;
use sm_mesh_api::{CellIx, FaceIx, MeshEntityKind, MeshRead, SerialAlloc};
use sm_testkit::broken::*;
use sm_testkit::check::*;
use sm_testkit::meshes::{CellSpec, build_mesh, hex_block_2x2x2};
use sm_testkit::topo::{self, cube};
use sm_mesh_api::CellShape;

fn block() -> ReferenceMesh {
    hex_block_2x2x2().unwrap().mesh
}

fn interior_face(m: &dyn MeshRead) -> usize {
    (0..m.face_count()).find(|&f| m.face_right(FaceIx::new(f)).is_some()).unwrap()
}

fn boundary_face(m: &dyn MeshRead) -> usize {
    (0..m.face_count()).find(|&f| m.face_right(FaceIx::new(f)).is_none()).unwrap()
}

fn all_mesh_checks(m: &dyn MeshRead) -> Vec<(&'static str, bool)> {
    vec![
        ("mesh_valid", mesh_valid(m).is_ok()),
        ("orientation_rule", orientation_rule(m).is_ok()),
        ("closed_cells", closed_cells(m).is_ok()),
        ("positive_volumes", positive_volumes(m).is_ok()),
        ("watertight", watertight(m).is_ok()),
        ("classification_complete", classification_complete(m).is_ok()),
    ]
}

fn failing(m: &dyn MeshRead) -> Vec<&'static str> {
    all_mesh_checks(m).into_iter().filter(|(_, ok)| !ok).map(|(n, _)| n).collect()
}

#[test]
fn an_untouched_copy_passes_every_check_and_hashes_the_same() {
    let m = block();
    let t = Tampered::of(&m);
    assert_eq!(failing(&t), Vec::<&str>::new());
    assert_eq!(mesh_hash(&t), mesh_hash(&m));
}

#[test]
fn swapped_left_right_is_caught() {
    let m = block();
    let t = Tampered::of(&m).swap_left_right(interior_face(&m));
    let f = failing(&t);
    assert!(f.contains(&"orientation_rule") && f.contains(&"mesh_valid"), "{f:?}");
    let e = orientation_rule(&t).unwrap_err();
    assert_eq!(e.check, "orientation_rule");
    assert!(e.to_string().contains("orientation_rule failed"));
}

#[test]
fn reversed_loop_is_caught() {
    let m = block();
    let f = failing(&Tampered::of(&m).reverse_loop(interior_face(&m)));
    assert!(f.contains(&"orientation_rule") && f.contains(&"closed_cells") && f.contains(&"mesh_valid"), "{f:?}");
}

#[test]
fn inverted_cell_is_caught() {
    let m = block();
    // Vertex (1,1,1) of the 3x3x3 lattice is the interior one; throw it out of the block.
    let t = Tampered::of(&m).move_vertex(13, Point3::new(-5.0, -5.0, -5.0));
    let f = failing(&t);
    assert!(f.contains(&"positive_volumes") && f.contains(&"mesh_valid"), "{f:?}");
}

#[test]
fn open_cell_is_caught() {
    let m = block();
    let f = failing(&Tampered::of(&m).remove_cell_face(0, 2));
    assert!(f.contains(&"closed_cells") && f.contains(&"mesh_valid"), "{f:?}");
}

#[test]
fn hole_in_the_boundary_is_caught() {
    let m = block();
    let f = failing(&Tampered::of(&m).set_face_right(boundary_face(&m), 7));
    assert!(f.contains(&"watertight"), "{f:?}");
}

#[test]
fn tampered_face_kinds_follow_the_owned_columns() {
    use sm_mesh_api::{FaceIx, FaceKind, MeshRead};
    let m = block();
    let bf = boundary_face(&m);
    let t = Tampered::of(&m).set_face_right(bf, 7);
    // The kind and the right cell agree, so every checker sees the same boundary.
    assert_eq!(t.face_kind(FaceIx::new(bf)), FaceKind::Interior);
    assert!(!t.is_boundary_face(FaceIx::new(bf)));
    for f in 0..t.face_count() {
        let fi = FaceIx::new(f);
        assert_eq!(t.is_boundary_face(fi), t.face_right(fi).is_none(), "face {f}");
    }
    assert_eq!(t.dim(), m.dim());
}

#[test]
fn missing_classification_is_caught() {
    let m = block();
    for (k, ix) in [(MeshEntityKind::Vertex, 3), (MeshEntityKind::Face, boundary_face(&m) as u32), (MeshEntityKind::Cell, 0)] {
        let f = failing(&Tampered::of(&m).declassify(k, ix));
        assert!(f.contains(&"classification_complete"), "{k:?}: {f:?}");
    }
}

#[test]
fn a_boundary_face_classified_on_a_volume_is_caught() {
    let m = block();
    let cell_class = m.classification(MeshEntityKind::Cell, 0);
    let mut t = Tampered::of(&m);
    t.class_override.insert((MeshEntityKind::Face, boundary_face(&m) as u32), cell_class);
    assert!(failing(&t).contains(&"classification_complete"));
}

#[test]
fn mesh_euler_distinguishes_one_ball_from_two() {
    let one = build_mesh(&UNIT, &[CellSpec::Template(CellShape::Hex, (0..8).collect())], &mut SerialAlloc::new()).unwrap();
    check_mesh_euler(&one, 1).unwrap();
    let mut pts = UNIT.to_vec();
    pts.extend(UNIT.iter().map(|p| [p[0] + 5.0, p[1], p[2]]));
    let two = build_mesh(
        &pts,
        &[CellSpec::Template(CellShape::Hex, (0..8).collect()), CellSpec::Template(CellShape::Hex, (8..16).collect())],
        &mut SerialAlloc::new(),
    )
    .unwrap();
    assert_eq!(mesh_euler(&two), 2);
    assert!(check_mesh_euler(&two, 1).is_err());
    // Each component is still watertight, and the whole is valid.
    watertight(&two).unwrap();
    mesh_valid(&two).unwrap();
}

const UNIT: [[f64; 3]; 8] = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];

#[test]
fn conformity_passes_on_the_block_and_fails_on_moved_or_wrong_carriers() {
    let m = block();
    let good = box_carriers([2, 2, 2], [0.5; 3]);
    conformity(&m, &good, 1e-12).unwrap();
    // A boundary vertex pushed off its face (and, as a corner, off its point).
    let t = Tampered::of(&m).move_vertex(0, Point3::new(-0.01, 0.0, 0.0));
    let e = conformity(&t, &good, 1e-6).unwrap_err();
    assert!(e.problems.iter().any(|p| p.contains("vertex 0")), "{e:?}");
    // An edge vertex that slides off its edge line.
    let t = Tampered::of(&m).move_vertex(1, Point3::new(0.5, 0.01, 0.0));
    assert!(conformity(&t, &good, 1e-6).is_err());
    // Wrong carriers for the same mesh.
    assert!(conformity(&m, &box_carriers([2, 2, 2], [0.6; 3]), 1e-6).is_err());
    // Missing carriers are reported.
    let none = Carriers::default();
    assert!(conformity(&m, &none, 1e-6).is_err());
}

#[test]
fn topology_checks_fail_on_broken_complexes() {
    let good = cube().unwrap();
    topo_valid(&good.complex).unwrap();
    manifold_edges(&good.complex).unwrap();
    check_topo_euler(&good.complex, 2).unwrap();

    let fin = brick_with_fin().unwrap();
    topo_valid(&fin.complex).unwrap();
    assert!(manifold_edges(&fin.complex).is_err());

    let baffle = brick_with_baffle().unwrap();
    topo_valid(&baffle.complex).unwrap();
    assert!(manifold_edges(&baffle.complex).is_err());

    let two = two_disjoint_bricks().unwrap();
    topo_valid(&two.complex).unwrap();
    assert!(check_topo_euler(&two.complex, 2).is_err());
    check_topo_euler(&two.complex, two.euler).unwrap();

    for f in [&fin, &baffle, &two] {
        use sm_topo_api::{TopologyView, TopologyViewExt};
        let area: f64 = f.complex.faces().iter().map(|x| f.complex.face_moments(*x).unwrap().area).sum();
        assert!((area - f.area).abs() < 1e-9, "{}: {area} vs {}", f.name, f.area);
    }
    for f in [&fin, &baffle, &two, &brick_with_moved_vertex().unwrap()] {
        let c = sm_topo_api::Dim::ALL.map(|d| sm_topo_api::TopologyView::cells(&f.complex, d).len());
        assert_eq!(c, f.counts, "{}", f.name);
        assert_eq!(sm_topo_api::euler_characteristic(&f.complex), f.euler, "{}", f.name);
    }

    let moved = brick_with_moved_vertex().unwrap();
    assert!(topo_valid(&moved.complex).is_err());
    // The merged pair has a three-face edge at its junction.
    assert!(manifold_edges(&topo::merged_bricks().unwrap().complex).is_err());
}

#[test]
fn model_check_fails_on_a_bad_mirror() {
    let bad = model_with_bad_mirror().unwrap();
    assert!(model_valid(&bad.model, &bad.complex).is_err());
    assert!(!topo::model_errors(&bad).is_empty());
    let ok = topo::pin_lattice_stub().unwrap();
    model_valid(&ok.model, &ok.complex).unwrap();
}

#[test]
fn tampered_cell_count_is_unchanged() {
    let m = block();
    assert_eq!(Tampered::of(&m).remove_cell_face(0, 0).cell_count(), 8);
    assert_eq!(Tampered::of(&m).cell_faces(CellIx::new(1)).len(), 6);
}
