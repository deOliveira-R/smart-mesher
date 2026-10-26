//! Acceptance 4 and 5: webcut with planes (and cylinders).

mod common;

use common::*;
use sm_boolean::{CutTool, KeepSide, WebcutOptions, webcut};
use sm_core::{Point3, Vector3};
use sm_topo_api::{CellComplex, Dim, LineageKind, TopologyView, TopologyViewExt};

fn cube() -> (CellComplex, sm_topo_api::fixtures::BrickIds) {
    let mut t = CellComplex::new();
    let b = brick(&mut t, 1, [0.0; 3], [1.0; 3]);
    (t, b)
}

fn plane_z(z: f64) -> CutTool {
    CutTool::Plane { origin: Point3::new(0.0, 0.0, z), normal: Vector3::new(0.0, 0.0, 1.0) }
}

#[test]
fn cube_cut_at_quarter() {
    let (mut t, b) = cube();
    let out = in_op(&mut t, 2, |t| webcut(t, &[b.body], &plane_z(0.25), &WebcutOptions::default())).unwrap();
    assert_eq!(counts(&t), [12, 20, 11, 2, 2]);
    assert_eq!(euler(&t), 3);
    assert!((total_area(&t) - 7.0).abs() < 1e-12);
    assert_valid(&t);
    assert_eq!(out.cut_faces.len(), 1);
    assert!((t.face_moments(out.cut_faces[0]).unwrap().area - 1.0).abs() < 1e-12);
    assert_eq!(out.negative.len(), 1);
    assert_eq!(out.positive.len(), 1);
    assert!(out.bodies.is_empty());
    let vol_of = |body| t.boundary(body)[0].cell;
    assert!((volume_of(&t, vol_of(out.negative[0])) - 0.25).abs() < 1e-12);
    assert!((volume_of(&t, vol_of(out.positive[0])) - 0.75).abs() < 1e-12);
    let rec = t.lineage().records().iter().find(|r| r.entity == out.cut_faces[0] && matches!(r.kind, LineageKind::Generated { .. }));
    assert!(rec.is_some());
    let _ = Dim::Face;
}

#[test]
fn reversed_normal_flips_the_roles_only() {
    let (mut t, b) = cube();
    let tool = CutTool::Plane { origin: Point3::new(0.0, 0.0, 0.25), normal: Vector3::new(0.0, 0.0, -1.0) };
    let out = in_op(&mut t, 2, |t| webcut(t, &[b.body], &tool, &WebcutOptions::default())).unwrap();
    let vol_of = |body| t.boundary(body)[0].cell;
    assert!((volume_of(&t, vol_of(out.positive[0])) - 0.25).abs() < 1e-12);
    assert!((volume_of(&t, vol_of(out.negative[0])) - 0.75).abs() < 1e-12);
}

#[test]
fn nomerge_gives_twelve_faces() {
    let (mut t, b) = cube();
    let o = WebcutOptions { merge: false, ..WebcutOptions::default() };
    let out = in_op(&mut t, 2, |t| webcut(t, &[b.body], &plane_z(0.25), &o)).unwrap();
    assert_eq!(counts(&t)[2], 12);
    assert_eq!(out.cut_faces.len(), 2);
    assert_valid(&t);
}

#[test]
fn section_keeps_one_half() {
    let (mut t, b) = cube();
    let o = WebcutOptions { keep: KeepSide::Positive, ..WebcutOptions::default() };
    let out = in_op(&mut t, 2, |t| webcut(t, &[b.body], &plane_z(0.25), &o)).unwrap();
    assert_eq!(out.positive.len(), 1);
    assert!(out.negative.is_empty());
    assert_eq!(counts(&t)[3], 1);
    let v = out.volumes[0];
    assert!((volume_of(&t, v) - 0.75).abs() < 1e-12);
    assert_valid(&t);
    // The cap is a boundary face of the kept piece.
    assert!(t.is_boundary_face(out.cut_faces[0]));
}

#[test]
fn a_plane_on_an_existing_face_changes_nothing_and_reports_the_side() {
    // Documented behaviour: the tool coincides with a face, so it does not cross the cube.
    let (mut t, b) = cube();
    let before = counts(&t);
    let out = in_op(&mut t, 2, |t| webcut(t, &[b.body], &plane_z(0.0), &WebcutOptions::default())).unwrap();
    assert_eq!((out.positive.len(), out.negative.len()), (1, 0));
    assert!(out.cut_faces.is_empty());
    assert_eq!(counts(&t), before);
    assert_valid(&t);
    let out = in_op(&mut t, 3, |t| webcut(t, &[b.body], &plane_z(1.0), &WebcutOptions::default())).unwrap();
    assert_eq!((out.positive.len(), out.negative.len()), (0, 1));
    assert_eq!(counts(&t), before);
}

#[test]
fn a_diagonal_plane_through_two_edges_and_four_vertices_halves_the_cube() {
    let (mut t, b) = cube();
    let tool = CutTool::Plane { origin: Point3::new(0.0, 0.0, 0.0), normal: Vector3::new(1.0, -1.0, 0.0) };
    let out = in_op(&mut t, 2, |t| webcut(t, &[b.body], &tool, &WebcutOptions::default())).unwrap();
    assert_valid(&t);
    assert_eq!((out.positive.len(), out.negative.len()), (1, 1));
    let vol_of = |body| t.boundary(body)[0].cell;
    assert!((volume_of(&t, vol_of(out.positive[0])) - 0.5).abs() < 1e-12);
    assert!((volume_of(&t, vol_of(out.negative[0])) - 0.5).abs() < 1e-12);
    assert!((t.face_moments(out.cut_faces[0]).unwrap().area - 2f64.sqrt()).abs() < 1e-12);
}
