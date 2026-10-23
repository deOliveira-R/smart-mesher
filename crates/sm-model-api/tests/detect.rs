//! Symmetry detection from colored invariants, with staged verification.

mod common;

use common::*;
use sm_core::{Point3, Vector3};
use sm_model_api::*;
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::{CellComplex, OpId};

fn three_bricks() -> (CellComplex, Model, [fixtures::BrickIds; 3]) {
    let mut t = CellComplex::new();
    let b0 = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])).unwrap();
    let b1 = with_op(&mut t, OpId(2), |t| fixtures::brick(t, Point3::new(1.0, 0.0, 0.0), [1.0, 1.0, 1.0])).unwrap();
    let b2 = with_op(&mut t, OpId(3), |t| fixtures::brick(t, Point3::new(2.0, 0.0, 0.0), [1.0, 1.0, 1.0])).unwrap();
    let mut m = Model::new();
    let a = m.add_color(1, "a", Domain::Volume).unwrap();
    let b = m.add_color(2, "b", Domain::Volume).unwrap();
    m.color_volumes(a, &[b0.volume, b2.volume]).unwrap();
    m.color_volumes(b, &[b1.volume]).unwrap();
    (t, m, [b0, b1, b2])
}

fn mirror_with_normal(r: &DetectionResult, n: Vector3) -> Option<&Candidate> {
    r.candidates.iter().find(|c| matches!(c.kind, CandidateKind::Mirror(p) if (p.normal.cross(n)).norm() < 1e-9))
}

#[test]
fn colored_bricks_have_a_verified_mirror_and_a_square_rotation() {
    let (t, m, _) = three_bricks();
    let r = detect_symmetry(&m, &t, Tolerance::default()).unwrap();
    assert_eq!(r.invariants.len(), 2);
    let ma = r.invariants[0];
    assert!((ma.volume - 2.0).abs() < 1e-12 && (ma.barycentre - Point3::new(1.5, 0.5, 0.5)).norm() < 1e-12, "{ma:?}");
    let mx = mirror_with_normal(&r, Vector3::new(1.0, 0.0, 0.0)).unwrap();
    assert!(mx.is_verified());
    let CandidateKind::Mirror(p) = mx.kind else { unreachable!() };
    assert!((p.offset.abs() - 1.5).abs() < 1e-12, "through the global barycentre x = 1.5: {p:?}");
    // Only the maximal order per axis: C4 about x, not C2 and C4 separately.
    let rx: Vec<&Candidate> = r.candidates.iter().filter(|c| matches!(c.kind, CandidateKind::Rotation { axis, .. } if axis.dir.cross(Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-9)).collect();
    assert_eq!(rx.len(), 1);
    assert!(matches!(rx[0].kind, CandidateKind::Rotation { order: 4, .. }) && rx[0].is_verified());
    // Elements through the middle brick: mirrors x, y, z + diagonals, order of D4h.
    assert_eq!(r.group_order, 16);
    assert!(r.complete);
    assert!(!r.generators.is_empty() && r.generators.len() <= 4, "a minimal generator set, not every element: {:?}", r.generators);
    // Invariants cannot tell directions apart inside the degenerate y-z plane, so some candidates
    // pass stage 0 and are rejected later; they are reported with their stage, never hidden.
    assert!(r.candidates.iter().any(|c| matches!(c.verification, Verification::Rejected { stage: Stage::Volumes | Stage::Faces, .. })));
    // The chamber of the x mirror is a CutSpec plannable as a mirror cut.
    let spec = mx.chamber(Keep::Positive);
    let plan = plan_cut(&spec).unwrap();
    assert_eq!(plan.half_spaces.len(), 1);
    assert!(plan.half_spaces[0].signed_distance(Point3::new(2.9, 0.5, 0.5)).abs() > 0.0);
}

#[test]
fn face_colors_and_kinds_break_candidates_at_the_faces_stage() {
    let (t, mut m, bricks) = three_bricks();
    let seam = m.add_color(7, "bc", Domain::Face).unwrap();
    m.color_faces(seam, &[bricks[0].faces[0]]).unwrap(); // only the outer -x face of the left brick
    let r = detect_symmetry(&m, &t, Tolerance::default()).unwrap();
    let mx = mirror_with_normal(&r, Vector3::new(1.0, 0.0, 0.0)).expect("it still passes the cheap invariants");
    match &mx.verification {
        Verification::Rejected { stage: Stage::Faces, reason, .. } => assert!(reason.contains("face"), "{reason}"),
        other => panic!("{other:?}"),
    }
    // A mirror in y maps that face to itself, so it still verifies.
    assert!(mirror_with_normal(&r, Vector3::new(0.0, 1.0, 0.0)).unwrap().is_verified());
    // Rejected candidates are refused unless forced; the provenance log is append-only.
    let mut m2 = m.clone();
    assert_eq!(m2.declare_detected(mx, Keep::Positive, false), Err(ModelError::Unverified));
    assert!(m2.provenance().is_empty());
    let spec = m2.declare_detected(mx, Keep::Positive, true).unwrap();
    assert!(matches!(m2.provenance(), [Provenance::Detected(Verification::Rejected { .. })]));
    assert!(matches!(spec.deck, Deck::Mirror { .. }));
    let ok = mirror_with_normal(&r, Vector3::new(0.0, 1.0, 0.0)).unwrap();
    m2.declare_detected(ok, Keep::Negative, false).unwrap();
    assert_eq!(m2.provenance().len(), 2);
    // verify_transform on a transform that is no symmetry at all.
    let shift = sm_core::Transform::translation(Vector3::new(0.1, 0.0, 0.0));
    assert!(matches!(verify_transform(&m, &t, &shift, Tolerance::default()).unwrap(), Verification::Rejected { stage: Stage::Invariants, .. }));
    // A color swap symmetry candidate fails at the volumes stage when colors differ.
    let mirror_x0 = Plane::new(Vector3::new(1.0, 0.0, 0.0), 1.0).unwrap().reflection();
    assert!(matches!(verify_transform(&m, &t, &mirror_x0, Tolerance::default()).unwrap(), Verification::Rejected { .. }));
}

#[test]
fn rotated_hex_prism_has_seven_mirrors_group_order_24_and_is_complete() {
    let mut t = CellComplex::new();
    let hex = regular_polygon(6, 1.0, 15f64.to_radians());
    let p = prism_op(&mut t, 1, &hex, 0.0, 1.0);
    let mut m = Model::new();
    let c = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(c, &[p.volume]).unwrap();
    let r = detect_symmetry(&m, &t, Tolerance::default()).unwrap();
    let verified_mirrors = r.candidates.iter().filter(|c| c.is_verified() && matches!(c.kind, CandidateKind::Mirror(_))).count();
    assert_eq!(verified_mirrors, 7, "{:#?}", r.candidates.iter().map(|c| (c.kind, c.is_verified())).collect::<Vec<_>>());
    assert_eq!(r.group_order, 24);
    assert!(r.complete, "face normals supply the in-plane directions of the degenerate eigenspace");
    let z = r.candidates.iter().find(|c| matches!(c.kind, CandidateKind::Rotation { axis, .. } if axis.dir.z.abs() > 0.999)).unwrap();
    assert!(matches!(z.kind, CandidateKind::Rotation { order: 6, .. }) && z.is_verified());
    // Only maximal orders are reported: six C2 axes in the plane, one C6 on z.
    let rots: Vec<u32> = r.candidates.iter().filter(|c| c.is_verified()).filter_map(|c| if let CandidateKind::Rotation { order, .. } = c.kind { Some(order) } else { None }).collect();
    assert_eq!(rots.iter().filter(|o| **o == 6).count(), 1);
    assert_eq!(rots.iter().filter(|o| **o == 2).count(), 6);
    assert_eq!(rots.len(), 7);
    // The 1/6 chamber of the C6 axis is a valid wedge plan.
    let spec = z.chamber(Keep::Positive);
    let plan = plan_cut(&spec).unwrap();
    assert_eq!(plan.half_spaces.len(), 2);
    let Deck::Rotation { angle, .. } = spec.deck else { panic!() };
    assert_eq!(angle, Turns::new(1, 6).unwrap());
}

#[test]
fn a_cube_has_the_full_octahedral_group_and_uncolored_models_are_errors() {
    // One cube: the inertia tensor is a multiple of the identity, but the faces supply directions.
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])).unwrap();
    let mut m = Model::new();
    let c = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(c, &[b.volume]).unwrap();
    let r = detect_symmetry(&m, &t, Tolerance::default()).unwrap();
    assert!(r.complete);
    assert_eq!(r.group_order, 48, "full octahedral group Oh");
    // An uncolored volume is an error, not a panic.
    let m2 = Model::new();
    assert!(detect_symmetry(&m2, &t, Tolerance::default()).is_err());
}
