//! Acceptance scenarios against the reference topology (`sm_topo_api::CellComplex`), not a mock.

mod common;

use common::*;
use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_model_api::*;
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::{CapFace, CellComplex, CutOutcome, OpId, Sense, TopologyEdit, TopologyView, TopologyViewExt};

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

fn tol() -> Tolerance {
    Tolerance::default()
}

fn close(a: Point3, b: Point3) -> bool {
    (a - b).norm() < 1e-9
}

fn brick_at(t: &mut CellComplex, op: u64, min: [f64; 3], size: [f64; 3]) -> fixtures::BrickIds {
    with_op(t, OpId(op), |t| fixtures::brick(t, p3(min[0], min[1], min[2]), size)).unwrap()
}

// ---------------------------------------------------------------- coloring

#[test]
fn two_materials_with_boundary_colors_and_an_uncolored_boundary_warning() {
    let mut t = CellComplex::new();
    let (a, b) = with_op(&mut t, OpId(1), fixtures::two_bricks).unwrap();
    let mut m = Model::new();
    let fuel = m.add_color(1, "fuel", Domain::Volume).unwrap();
    let water = m.add_color(2, "water", Domain::Volume).unwrap();
    let vac = m.add_color(7, "vacuum", Domain::Face).unwrap();
    let seam = m.add_color(8, "fuel-water", Domain::Face).unwrap();
    m.color_volumes(fuel, &[a.volume]).unwrap();
    m.color_volumes(water, &[b.volume]).unwrap();
    // Labels and names are looked up (linear scan over the small color table).
    assert_eq!(m.color_by_label(Domain::Face, 7), Some(vac));
    assert_eq!(m.color_by_label(Domain::Volume, 7), None, "labels are per domain");
    assert_eq!(m.color_by_name("water"), Some(water));
    assert_eq!(m.add_color(1, "other", Domain::Volume), Err(ModelError::LabelTaken { label: 1 }));
    assert_eq!(m.add_color(9, "fuel", Domain::Volume), Err(ModelError::NameTaken("fuel".into())));
    assert_eq!(m.color_faces(fuel, &[a.faces[0]]), Err(ModelError::ColorDomain(fuel)));

    // The touching faces are a non-conformal interface; everything else is response.
    let iface = m.add_boundary("fuel/water", BoundaryKind::Interface(InterfaceIntent::NonConformal { transform: None })).unwrap();
    m.assign_interface(iface, &[a.faces[1]], &[b.faces[0]]).unwrap();
    m.color_faces(seam, &[a.faces[1], b.faces[0]]).unwrap();
    let outer = m.add_boundary("outer", BoundaryKind::Response).unwrap();
    m.assign_response(outer, &[a.faces[0]]).unwrap();
    assert_eq!(m.assign_response(outer, &[a.faces[0]]), Err(ModelError::Duplicate(a.faces[0])), "a face has one kind");
    assert_eq!(m.assign_response(iface, &[a.faces[2]]), Err(ModelError::Invalid(format!("{iface} is not a response record"))));
}

#[test]
fn uncolored_boundary_face_is_a_warning_not_an_error() {
    let mut t = CellComplex::new();
    let (a, b) = with_op(&mut t, OpId(1), fixtures::two_bricks).unwrap();
    let mut m = Model::new();
    let fuel = m.add_color(1, "fuel", Domain::Volume).unwrap();
    let water = m.add_color(2, "water", Domain::Volume).unwrap();
    let vac = m.add_color(7, "vacuum", Domain::Face).unwrap();
    let seam = m.add_color(8, "seam", Domain::Face).unwrap();
    m.color_volumes(fuel, &[a.volume]).unwrap();
    m.color_volumes(water, &[b.volume]).unwrap();
    let iface = m.add_boundary("fuel/water", BoundaryKind::Interface(InterfaceIntent::NonConformal { transform: None })).unwrap();
    m.assign_interface(iface, &[a.faces[1]], &[b.faces[0]]).unwrap();
    m.color_faces(seam, &[a.faces[1], b.faces[0]]).unwrap();
    let outer = m.add_boundary("outer", BoundaryKind::Response).unwrap();
    let outer_faces = [a.faces[0], a.faces[2], a.faces[3], a.faces[4], a.faces[5], b.faces[1], b.faces[2], b.faces[3], b.faces[4], b.faces[5]];
    m.assign_response(outer, &outer_faces).unwrap();
    // Boundary color 7 on all outer faces except one.
    let colored: Vec<EntityId> = outer_faces.iter().copied().filter(|f| *f != b.faces[5]).collect();
    m.color_faces(vac, &colored).unwrap();
    let r = validate(&m, &t, tol());
    assert!(r.is_ok(), "{:#?}", r.issues);
    let w: Vec<&Issue> = r.warnings().collect();
    assert_eq!(w.len(), 1, "{w:#?}");
    assert_eq!(w[0].code, IssueCode::BoundaryFaceUncolored);
    assert_eq!(w[0].severity, Severity::Warning);
    assert_eq!(w[0].entities.as_slice(), &[b.faces[5]]);
    // At export time, when the target format needs boundary colors, the same finding is an error.
    let strict = r.clone().promote(&[IssueCode::BoundaryFaceUncolored]);
    assert!(!strict.is_ok() && strict.errors().count() == 1);
    // Without any kind on a boundary face it is an error.
    let mut m2 = m.clone();
    m2.unassign(&[b.faces[5]]);
    let r2 = validate(&m2, &t, tol());
    assert!(r2.has(IssueCode::BoundaryFaceNoKind) && !r2.is_ok());
    // A volume without a color breaks the partition.
    let mut m3 = Model::new();
    m3.add_color(1, "fuel", Domain::Volume).unwrap();
    assert!(validate(&m3, &t, tol()).has(IssueCode::ColoringNotPartition));
}

#[test]
fn groups_expand_colors_bodies_and_nested_groups() {
    let mut t = CellComplex::new();
    let (a, b) = with_op(&mut t, OpId(1), fixtures::two_bricks).unwrap();
    let mut m = Model::new();
    let c = m.add_color(1, "fuel", Domain::Volume).unwrap();
    let fc = m.add_color(1, "bc", Domain::Face).unwrap();
    m.color_volumes(c, &[a.volume]).unwrap();
    m.color_faces(fc, &[b.faces[3]]).unwrap();
    let g_color = m.add_group("by-color", &[c]).unwrap();
    let g_body = m.add_group("bodies", &[b.body]).unwrap();
    let g_faces = m.add_group("bc-faces", &[fc, a.faces[0]]).unwrap();
    let outer = m.add_group("all", &[g_color, g_body, g_faces]).unwrap();
    assert_eq!(m.group_volumes(outer, &t).unwrap(), vec![a.volume, b.volume]);
    let faces = m.group_faces(outer, &t).unwrap();
    assert!(faces.contains(&a.faces[0]) && faces.contains(&b.faces[3]) && faces.contains(&a.faces[5]) && faces.contains(&b.faces[0]));
    assert_eq!(faces.len(), 12, "all faces of both bricks");
    assert!(m.remove_color(c).is_err(), "still used by a volume");
    m.color_volumes(c, &[]).unwrap();
    m.remove_color(fc).unwrap();
    assert_eq!(m.face_color(b.faces[3]), None);
}

// ---------------------------------------------------------------- cut, unfold

/// A unit cube colored `mat`, every face response + colored, cut at `z = 0.5` keeping the lower
/// half through the real kernel. Returns everything the assertions need.
struct Cut {
    t: CellComplex,
    m: Model,
    keep: EntityId,
    cap: EntityId,
    mat: EntityId,
    cap_color: EntityId,
    b: fixtures::BrickIds,
    boundary: EntityId,
}

fn cut_cube() -> Cut {
    let mut t = CellComplex::new();
    let b = brick_at(&mut t, 1, [0.0; 3], [1.0, 1.0, 1.0]);
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[b.volume]).unwrap();
    finish_boundaries(&mut m, &t, 7, true);
    let cap_color = m.add_color(99, "reflect", Domain::Face).unwrap();
    let w = with_op(&mut t, OpId(2), |t| {
        let w = fixtures::webcut_brick(t, &b, 0.5)?;
        t.kill_cascade(w.volumes[1])?;
        Ok(w)
    })
    .unwrap();
    let out = t.lineage().cut_outcome(OpId(2));
    let spec = CutSpec { deck: Deck::Mirror { plane: Plane::new(Vector3::new(0.0, 0.0, 1.0), 0.5).unwrap() }, keep: Keep::Negative, origin: p3(0.0, 0.0, 0.5) };
    let plan = plan_cut(&spec).unwrap();
    let boundary = m.apply_cut(&plan, &out, Some(cap_color), &t).unwrap();
    Cut { keep: w.volumes[0], cap: w.cap, t, m, mat, cap_color, b, boundary }
}

#[test]
fn mirror_cut_keeps_the_chamber_and_tags_the_cap_as_a_mirror_deck() {
    let c = cut_cube();
    let (m, t) = (&c.m, &c.t);
    // Colors followed the lineage chain (split, then the other half deleted) onto the live child.
    assert_eq!(m.volume_color(c.keep), Some(c.mat));
    assert_eq!(m.volume_color(c.b.volume), None, "the retired parent is gone");
    // The cap is a self-paired mirror deck with the cap color.
    let fb = m.face_boundary(c.cap).unwrap();
    assert_eq!(fb.boundary, c.boundary);
    assert_eq!(fb.role, FaceRole::Deck { side: DeckSide::SelfPaired, mate: None });
    assert_eq!(m.face_color(c.cap), Some(c.cap_color));
    match m.face_kind(c.cap, t).unwrap() {
        FaceKindView::Deck { deck: Deck::Mirror { plane }, side: DeckSide::SelfPaired, mate: None, .. } => assert!((plane.offset - 0.5).abs() < 1e-12),
        other => panic!("{other:?}"),
    }
    // Symmetry and provenance were recorded.
    let sym = m.symmetry().unwrap();
    assert_eq!(sym.generators, vec![c.boundary]);
    assert!(sym.domain.contains(&c.keep));
    assert!(matches!(m.provenance(), [Provenance::Cut { .. }]));
    // The old side faces were re-keyed to their lower halves; the top face vanished.
    assert!(m.face_boundary(c.b.faces[5]).is_none());
    let r = validate(m, t, tol());
    assert!(r.is_ok(), "{:#?}", r.issues);
    assert_eq!(r.warnings().count(), 0, "{:#?}", r.issues);
}

#[test]
fn unfolding_the_cut_model_gives_identical_colors_and_orientation_flips() {
    let c = cut_cube();
    let (m, t) = (&c.m, &c.t);
    let u = unfold(m, t, &UnfoldSpec::default()).unwrap();
    assert_eq!(u.elements().len(), 2);
    assert!(!u.elements()[0].orientation_reversed && u.elements()[1].orientation_reversed);
    let vols: Vec<CopyRef> = u.volumes().collect();
    assert_eq!(vols.len(), 2);
    for v in &vols {
        assert_eq!(u.color_of(*v), m.volume_color(v.base), "unfolding never changes colors");
    }
    // The mirrored copy is the reflection of the base at z = 0.5.
    let x = u.elements()[1].transform;
    assert!(close(x.apply_point(p3(0.3, 0.4, 0.1)), p3(0.3, 0.4, 0.9)));
    // The cap is glued to itself and becomes interior; the others keep their kind.
    assert_eq!(u.glued().len(), 1);
    assert_eq!((u.glued()[0].a.base, u.glued()[0].b.base), (c.cap, c.cap));
    assert!(u.exposed().is_empty());
    assert_eq!(u.kind_of(CopyRef { base: c.cap, elem: 0 }), Some(FaceKindView::Interior));
    assert!(matches!(u.kind_of(CopyRef { base: c.b.faces[4], elem: 1 }), Some(FaceKindView::Response { .. })));
    assert_eq!(u.sense(CopyRef { base: c.cap, elem: 0 }), Sense::Forward);
    assert_eq!(u.sense(CopyRef { base: c.cap, elem: 1 }), Sense::Reversed, "improper copy reverses loops");
    // Pre-cut entities are found through the recorded lineage.
    let lower_side = t.lineage().children(c.b.faces[0]).first().copied().unwrap();
    assert_eq!(u.origin_of(CopyRef { base: lower_side, elem: 1 }), Some(c.b.faces[0]));
    assert_eq!(u.origin_of(CopyRef { base: c.keep, elem: 0 }), Some(c.b.volume));
    assert_eq!(u.refold(), m.symmetry().unwrap());
    let r = validate_unfolded(&u, m, t, tol());
    assert!(r.is_ok(), "{:#?}", r.issues);
    // The unfolded volume is exactly the cube: first moments add up.
    let mk = t.volume_moments(c.keep).unwrap();
    let mut total = mk;
    let img = transform_moments(&mk, &x);
    total.volume += img.volume;
    total.first = total.first + img.first;
    assert!((total.volume - 1.0).abs() < 1e-12 && (total.first.z / total.volume - 0.5).abs() < 1e-12);
}

#[test]
fn deck_lookup_is_a_dense_table_with_o1_access() {
    let c = cut_cube();
    let look = c.m.deck_lookup(&c.t);
    assert_eq!(look.len(), c.t.faces().len());
    assert_eq!(look.deck_face_count(), 1);
    let hit = look.hit(c.cap).unwrap();
    assert_eq!(hit.mate, c.cap);
    assert!(hit.improper);
    assert!(close(hit.to_mate.apply_point(p3(0.2, 0.3, 0.4)), p3(0.2, 0.3, 0.6)));
    let ord = c.t.face_ordinal(c.cap).unwrap();
    assert_eq!(look.hit_by_ordinal(ord), Some(hit));
    let other = c.t.faces().iter().copied().find(|f| *f != c.cap).unwrap();
    assert!(look.hit(other).is_none());
    assert!(look.hit_by_ordinal(10_000).is_none());
}

#[test]
fn apply_cut_rejects_outcomes_that_do_not_match_the_plan() {
    // Caps off the planned plane.
    let mut t = CellComplex::new();
    let b = brick_at(&mut t, 1, [0.0; 3], [1.0, 1.0, 1.0]);
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[b.volume]).unwrap();
    with_op(&mut t, OpId(2), |t| {
        let w = fixtures::webcut_brick(t, &b, 0.5)?;
        t.kill_cascade(w.volumes[1])?;
        Ok(w)
    })
    .unwrap();
    let out = t.lineage().cut_outcome(OpId(2));
    let mk = |offset: f64, keep: Keep| {
        plan_cut(&CutSpec { deck: Deck::Mirror { plane: Plane::new(Vector3::new(0.0, 0.0, 1.0), offset).unwrap() }, keep, origin: p3(0.0, 0.0, offset) }).unwrap()
    };
    let mut probe = m.clone();
    assert!(matches!(probe.apply_cut(&mk(0.25, Keep::Negative), &out, None, &t), Err(ModelError::CutInvalid(s)) if s.contains("off the planned")));
    // Kept volume on the wrong side.
    assert!(matches!(probe.apply_cut(&mk(0.5, Keep::Positive), &out, None, &t), Err(ModelError::CutInvalid(s)) if s.contains("outside")));
    assert_eq!(probe, m, "a rejected cut changes nothing");
    // The right plan works.
    m.apply_cut(&mk(0.5, Keep::Negative), &out, None, &t).unwrap();
    // A cap that is still an interior face (discarded half not removed) is rejected.
    let mut t2 = CellComplex::new();
    let b2 = brick_at(&mut t2, 1, [0.0; 3], [1.0, 1.0, 1.0]);
    let mut m2 = Model::new();
    let c2 = m2.add_color(1, "mat", Domain::Volume).unwrap();
    m2.color_volumes(c2, &[b2.volume]).unwrap();
    with_op(&mut t2, OpId(2), |t| fixtures::webcut_brick(t, &b2, 0.5)).unwrap();
    let out2 = t2.lineage().cut_outcome(OpId(2));
    assert!(matches!(m2.apply_cut(&mk(0.5, Keep::Negative), &out2, None, &t2), Err(ModelError::CutInvalid(s)) if s.contains("not a boundary face")));
}

#[test]
fn a_mirror_cut_through_a_periodic_pair_repairs_the_split_mates() {
    let mut t = CellComplex::new();
    let b = brick_at(&mut t, 1, [0.0; 3], [1.0, 1.0, 1.0]);
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[b.volume]).unwrap();
    let per = m.add_boundary("periodic-x", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(1.0, 0.0, 0.0) })).unwrap();
    m.assign_pairs(per, &[(b.faces[0], b.faces[1])]).unwrap();
    finish_boundaries(&mut m, &t, 7, true);
    assert!(validate(&m, &t, tol()).is_ok());
    with_op(&mut t, OpId(2), |t| {
        let w = fixtures::webcut_brick(t, &b, 0.5)?;
        t.kill_cascade(w.volumes[1])?;
        Ok(())
    })
    .unwrap();
    let out = t.lineage().cut_outcome(OpId(2));
    let plan = plan_cut(&CutSpec { deck: Deck::Mirror { plane: Plane::new(Vector3::new(0.0, 0.0, 1.0), 0.5).unwrap() }, keep: Keep::Negative, origin: p3(0.0, 0.0, 0.5) }).unwrap();
    m.apply_cut(&plan, &out, None, &t).unwrap();
    let pairs: Vec<_> = m.boundary_faces(per).collect();
    assert_eq!(pairs.len(), 2, "one lower half on each side");
    for (f, fb) in &pairs {
        let FaceRole::Deck { mate: Some(mt), .. } = fb.role else { panic!() };
        assert_eq!(m.face_boundary(mt).unwrap().role, FaceRole::Deck { side: if matches!(fb.role, FaceRole::Deck { side: DeckSide::A, .. }) { DeckSide::B } else { DeckSide::A }, mate: Some(*f) });
    }
    let r = validate(&m, &t, tol());
    assert!(r.is_ok(), "{:#?}", r.issues);
    // The generators are exactly the decks produced by cuts or declarations: the periodic pair
    // was never declared as a generator, so it is only a warning.
    assert!(r.has(IssueCode::DeckNotInSymmetry));
}

// ---------------------------------------------------------------- imported, per-face deck kinds

#[test]
fn imported_pre_cut_model_mirror_and_translation_pair() {
    let mut t = CellComplex::new();
    let b = brick_at(&mut t, 1, [0.0; 3], [1.0, 1.0, 1.0]);
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[b.volume]).unwrap();
    let per = m.add_boundary("periodic-x", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(1.0, 0.0, 0.0) })).unwrap();
    let mir = m.add_boundary("mirror-z", BoundaryKind::Deck(Deck::Mirror { plane: Plane::new(Vector3::new(0.0, 0.0, 1.0), 0.0).unwrap() })).unwrap();
    m.assign_pairs(per, &[(b.faces[0], b.faces[1])]).unwrap();
    m.assign_self_paired(mir, &[b.faces[4]]).unwrap();
    finish_boundaries(&mut m, &t, 7, true);
    let r = validate(&m, &t, tol());
    assert!(r.is_ok(), "{:#?}", r.issues);
    m.declare_imported_symmetry(&[per, mir], &t).unwrap();
    assert_eq!(m.provenance(), &[Provenance::Imported]);
    assert!(validate(&m, &t, tol()).is_ok());
    // A translation needs a window.
    assert_eq!(unfold(&m, &t, &UnfoldSpec::default()).unwrap_err(), ModelError::MissingRepeats(per));
    let mut spec = UnfoldSpec::default();
    spec.translation_repeats.insert(per, 3);
    let u = unfold(&m, &t, &spec).unwrap();
    assert_eq!(u.elements().len(), 6);
    // Translation: 2 point elements x 2 interior shifts = 4 glues; mirror: 3 shifts = 3 glues.
    assert_eq!(u.glued().len(), 7);
    assert_eq!(u.exposed().len(), 4, "the two ends of the window, A and B side, for both mirror images");
    let r = validate_unfolded(&u, &m, &t, tol());
    assert!(r.is_ok(), "{:#?}", r.issues);
    // Deck lookup for tracking: A -> B is the translation, B -> A its inverse.
    let look = m.deck_lookup(&t);
    let ha = look.hit(b.faces[0]).unwrap();
    assert_eq!(ha.mate, b.faces[1]);
    assert!(close(ha.to_mate.apply_point(p3(0.0, 0.5, 0.5)), p3(1.0, 0.5, 0.5)));
    let hb = look.hit(b.faces[1]).unwrap();
    assert!(close(hb.to_mate.apply_point(p3(1.0, 0.5, 0.5)), p3(0.0, 0.5, 0.5)) && !hb.improper);
    // The same sums analytically: the window covers x in [0, 3] and z in [-1, 1].
    let mk = t.volume_moments(b.volume).unwrap();
    let mut vol = 0.0;
    let mut first = Vector3::default();
    for e in u.elements() {
        let img = transform_moments(&mk, &e.transform);
        vol += img.volume;
        first = first + img.first;
    }
    assert!((vol - 6.0).abs() < 1e-12);
    assert!((first.x / vol - 1.5).abs() < 1e-12 && (first.z / vol).abs() < 1e-12);
}

#[test]
fn wrong_decks_are_caught_by_moments_not_by_corner_outlines() {
    let mut t = CellComplex::new();
    let (a, b) = with_op(&mut t, OpId(1), fixtures::two_bricks).unwrap();
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[a.volume, b.volume]).unwrap();
    // Same-side pair: translation takes A's -x face onto B's -x face, but both normals are -x.
    let per = m.add_boundary("same-side", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(1.0, 0.0, 0.0) })).unwrap();
    m.assign_pairs(per, &[(a.faces[0], b.faces[0])]).unwrap();
    let r = validate(&m, &t, tol());
    assert!(r.has(IssueCode::DeckOrientation), "{:#?}", r.issues);
    assert!(!r.has(IssueCode::DeckGeometry));
    // A proper pair with the wrong vector.
    let mut m2 = Model::new();
    let per2 = m2.add_boundary("wrong-vector", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(2.0, 0.0, 0.0) })).unwrap();
    m2.assign_pairs(per2, &[(a.faces[0], b.faces[1])]).unwrap();
    let r2 = validate(&m2, &t, tol());
    assert!(!r2.has(IssueCode::DeckGeometry) && !r2.has(IssueCode::DeckOrientation), "A -x to B +x by (2,0,0) is a valid pair: {:#?}", r2.issues);
    let mut m3 = Model::new();
    let per3 = m3.add_boundary("wrong-vector", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(1.5, 0.0, 0.0) })).unwrap();
    m3.assign_pairs(per3, &[(a.faces[0], b.faces[1])]).unwrap();
    assert!(validate(&m3, &t, tol()).has(IssueCode::DeckGeometry));
}

#[test]
fn equal_area_and_centroid_but_different_shape_is_a_deck_geometry_error() {
    let mut t = CellComplex::new();
    let a = brick_at(&mut t, 1, [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    // B: face -y/+x area 0.5 * 2 = 1, centered at (2, 0.5, 0.5) like A's -x face moved by 2.
    let b = brick_at(&mut t, 2, [1.0, 0.25, -0.5], [1.0, 0.5, 2.0]);
    let mut m = Model::new();
    let per = m.add_boundary("pair", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(2.0, 0.0, 0.0) })).unwrap();
    m.assign_pairs(per, &[(a.faces[0], b.faces[1])]).unwrap();
    let fa = t.face_moments(a.faces[0]).unwrap();
    let fb = t.face_moments(b.faces[1]).unwrap();
    assert!((fa.area - fb.area).abs() < 1e-12 && (fa.centroid().unwrap() + Vector3::new(2.0, 0.0, 0.0) - fb.centroid().unwrap()).norm() < 1e-12, "same area, same centroid");
    assert!(validate(&m, &t, tol()).has(IssueCode::DeckGeometry));
}

#[test]
fn reflective_unit_cell_with_mirrors_at_x0_and_x1_is_valid() {
    let mut t = CellComplex::new();
    let b = brick_at(&mut t, 1, [0.0; 3], [1.0, 1.0, 1.0]);
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[b.volume]).unwrap();
    let m0 = m.add_boundary("mirror-x0", BoundaryKind::Deck(Deck::Mirror { plane: Plane::new(Vector3::new(1.0, 0.0, 0.0), 0.0).unwrap() })).unwrap();
    let m1 = m.add_boundary("mirror-x1", BoundaryKind::Deck(Deck::Mirror { plane: Plane::new(Vector3::new(1.0, 0.0, 0.0), 1.0).unwrap() })).unwrap();
    m.assign_self_paired(m0, &[b.faces[0]]).unwrap();
    m.assign_self_paired(m1, &[b.faces[1]]).unwrap();
    finish_boundaries(&mut m, &t, 7, true);
    let r = validate(&m, &t, tol());
    assert!(r.is_ok(), "deck kinds are per face, no forced group closure: {:#?}", r.issues);
    // Declaring both as one symmetry fails up front: parallel mirrors compose to a translation.
    assert!(matches!(m.declare_symmetry(&[m0, m1], &t), Err(ModelError::GroupNotFinite { .. })));
    assert!(m.symmetry().is_none(), "nothing was recorded");
    m.declare_symmetry(&[m0], &t).unwrap();
    // The same plane x = 1 with a flipped normal and offset describes the same mirror.
    let mut flipped = Model::new();
    let f1 = flipped.add_boundary("mirror-x1-flipped", BoundaryKind::Deck(Deck::Mirror { plane: Plane::new(Vector3::new(-1.0, 0.0, 0.0), -1.0).unwrap() })).unwrap();
    flipped.assign_self_paired(f1, &[b.faces[1]]).unwrap();
    assert!(!validate(&flipped, &t, tol()).has(IssueCode::MirrorOffPlane));
    let f0 = flipped.add_boundary("mirror-x0-flipped", BoundaryKind::Deck(Deck::Mirror { plane: Plane::new(Vector3::new(-1.0, 0.0, 0.0), 0.0).unwrap() })).unwrap();
    flipped.assign_self_paired(f0, &[b.faces[0]]).unwrap();
    assert!(!validate(&flipped, &t, tol()).has(IssueCode::MirrorOffPlane));
    // A mirror face off its plane is an error.
    let mut bad = Model::new();
    let wrong = bad.add_boundary("m", BoundaryKind::Deck(Deck::Mirror { plane: Plane::new(Vector3::new(1.0, 0.0, 0.0), 0.5).unwrap() })).unwrap();
    bad.assign_self_paired(wrong, &[b.faces[0]]).unwrap();
    assert!(validate(&bad, &t, tol()).has(IssueCode::MirrorOffPlane));
}

// ---------------------------------------------------------------- rotation

fn wedge() -> (CellComplex, PrismIds) {
    let mut t = CellComplex::new();
    let poly = [(0.0, 0.0), (1.0, 0.0), (0.5, 3f64.sqrt() / 2.0)];
    let p = prism_op(&mut t, 1, &poly, 0.0, 1.0);
    (t, p)
}

fn wedge_model(t: &CellComplex, p: &PrismIds, angle: Turns) -> (Model, EntityId) {
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[p.volume]).unwrap();
    let axis = Axis::new(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
    let rot = m.add_boundary("rot", BoundaryKind::Deck(Deck::Rotation { axis, angle })).unwrap();
    m.assign_pairs(rot, &[(p.sides[0], p.sides[2])]).unwrap();
    finish_boundaries(&mut m, t, 7, true);
    (m, rot)
}

#[test]
fn one_sixth_rotation_pair_with_order_consistency() {
    let (t, p) = wedge();
    let (mut m, rot) = wedge_model(&t, &p, Turns::new(1, 6).unwrap());
    let r = validate(&m, &t, tol());
    assert!(r.is_ok(), "{:#?}", r.issues);
    m.declare_symmetry(&[rot], &t).unwrap();
    let u = unfold(&m, &t, &UnfoldSpec::default()).unwrap();
    assert_eq!(u.elements().len(), 6);
    assert_eq!(u.glued().len(), 6, "every A face meets one B face");
    assert!(u.exposed().is_empty());
    assert!(u.elements().iter().all(|e| !e.orientation_reversed));
    assert!(validate_unfolded(&u, &m, &t, tol()).is_ok());
    // The six wedges form the hexagon: area 6 * sqrt(3)/4, barycentre on the axis.
    let mw = t.volume_moments(p.volume).unwrap();
    let mut vol = 0.0;
    let mut first = Vector3::default();
    for e in u.elements() {
        let img = transform_moments(&mw, &e.transform);
        vol += img.volume;
        first = first + img.first;
    }
    assert!((vol - 6.0 * 3f64.sqrt() / 4.0).abs() < 1e-12);
    assert!(first.x.abs() < 1e-12 && first.y.abs() < 1e-12 && (first.z / vol - 0.5).abs() < 1e-12);
    // Order consistency: the same faces with a 1/3 or 5/6 rotation do not match.
    for bad in [Turns::new(1, 3).unwrap(), Turns::new(5, 6).unwrap(), Turns::new(1, 4).unwrap()] {
        let (m2, _) = wedge_model(&t, &p, bad);
        assert!(validate(&m2, &t, tol()).has(IssueCode::DeckGeometry), "{bad:?}");
    }
    // A non-primitive generator would not tile even with the right pairing geometry.
    let (m3, _) = wedge_model(&t, &p, Turns::new(2, 5).unwrap());
    assert!(validate(&m3, &t, tol()).has(IssueCode::RotationOrder));
    assert_eq!(Turns::new(2, 6).unwrap(), Turns { num: 1, den: 3 });
    assert_eq!(Turns::new(-1, 6).unwrap(), Turns { num: 5, den: 6 });
    assert_eq!(Turns::new(1, 6).unwrap().order(), 6);
}

#[test]
fn wedge_chamber_plan_has_two_planes_and_a_rotation_pair_of_caps() {
    let axis = Axis::new(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
    let spec = CutSpec { deck: Deck::Rotation { axis, angle: Turns::new(1, 6).unwrap() }, keep: Keep::Positive, origin: p3(1.0, 0.0, 0.0) };
    let plan = plan_cut(&spec).unwrap();
    assert_eq!(plan.half_spaces.len(), 2);
    assert_eq!(plan.roles.as_slice(), &[DeckSide::A, DeckSide::B]);
    // The 30 degree direction is inside both half-spaces; 90 and -30 are outside.
    let inside = |p: Point3| plan.half_spaces.iter().all(|h| h.signed_distance(p) >= -1e-12);
    let at = |deg: f64| p3(deg.to_radians().cos(), deg.to_radians().sin(), 0.0);
    assert!(inside(at(30.0)) && !inside(at(90.0)) && !inside(at(-30.0)) && inside(at(0.0)) && inside(at(60.0)));
    // Negative keeps the wedge clockwise from the start direction.
    let neg = plan_cut(&CutSpec { keep: Keep::Negative, ..spec }).unwrap();
    let inside_n = |p: Point3| neg.half_spaces.iter().all(|h| h.signed_distance(p) >= -1e-12);
    assert!(inside_n(at(-30.0)) && !inside_n(at(30.0)));
    // A half turn or more cannot be a wedge.
    assert!(plan_cut(&CutSpec { deck: Deck::Rotation { axis, angle: Turns::new(1, 2).unwrap() }, ..spec }).is_err());
    // Slab: thickness |v|, caps A then B.
    let slab = plan_cut(&CutSpec { deck: Deck::Translation { vector: Vector3::new(2.0, 0.0, 0.0) }, keep: Keep::Positive, origin: p3(1.0, 0.0, 0.0) }).unwrap();
    let inside_s = |p: Point3| slab.half_spaces.iter().all(|h| h.signed_distance(p) >= -1e-12);
    assert!(inside_s(p3(2.0, 5.0, 5.0)) && !inside_s(p3(0.5, 0.0, 0.0)) && !inside_s(p3(3.5, 0.0, 0.0)));
}

#[test]
fn point_group_closure_is_fast_and_rejects_infinite_groups() {
    let r = Transform::rotation(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0), core::f64::consts::TAU / 1000.0).unwrap();
    let g = point_group(&[r], 4096).unwrap();
    assert_eq!(g.len(), 1000);
    let m0 = Plane::new(Vector3::new(1.0, 0.0, 0.0), 0.0).unwrap().reflection();
    let m1 = Plane::new(Vector3::new(1.0, 0.0, 0.0), 1.0).unwrap().reflection();
    assert!(matches!(point_group(&[m0, m1], 4096), Err(ModelError::GroupNotFinite { .. })));
    let d = point_group(&[r.compose(&r), m0], 4096).unwrap();
    assert_eq!(d.len(), 1000, "even rotations of C1000 give C500, with a mirror (non-commuting dihedral) 1000");
    let tr = Transform::translation(Vector3::new(1.0, 0.0, 0.0));
    assert!(point_group(&[tr], 100).is_err());
}

// ---------------------------------------------------------------- interfaces

#[test]
fn interface_between_colors_reads_coincident_faces_and_non_conformal_counts() {
    // 1 face against 2: A's +x face (area 1) faces two half-width bricks.
    let mut t = CellComplex::new();
    let a = brick_at(&mut t, 1, [0.0; 3], [1.0, 1.0, 1.0]);
    let b1 = brick_at(&mut t, 2, [1.0, 0.0, 0.0], [1.0, 0.5, 1.0]);
    let b2 = brick_at(&mut t, 3, [1.0, 0.5, 0.0], [1.0, 0.5, 1.0]);
    let mut m = Model::new();
    let c1 = m.add_color(1, "left", Domain::Volume).unwrap();
    let c2 = m.add_color(2, "right", Domain::Volume).unwrap();
    m.color_volumes(c1, &[a.volume]).unwrap();
    m.color_volumes(c2, &[b1.volume, b2.volume]).unwrap();
    let i = m.add_boundary("slide", BoundaryKind::Interface(InterfaceIntent::NonConformal { transform: None })).unwrap();
    m.assign_interface(i, &[a.faces[1]], &[b1.faces[0], b2.faces[0]]).unwrap();
    finish_boundaries(&mut m, &t, 7, true);
    let r = validate(&m, &t, tol());
    assert!(r.is_ok(), "{:#?}", r.issues);
    assert!(!r.has(IssueCode::InterfaceSameColor));
}

#[test]
fn interface_faces_must_face_have_counterparts_and_not_be_merged() {
    let mut t = CellComplex::new();
    let (a, b) = with_op(&mut t, OpId(1), fixtures::two_bricks).unwrap();
    let base = || {
        let mut m = Model::new();
        let c1 = m.add_color(1, "left", Domain::Volume).unwrap();
        let c2 = m.add_color(2, "right", Domain::Volume).unwrap();
        m.color_volumes(c1, &[a.volume]).unwrap();
        m.color_volumes(c2, &[b.volume]).unwrap();
        m
    };
    // Both faces point the same way: not facing.
    let mut m = base();
    let i = m.add_boundary("i", BoundaryKind::Interface(InterfaceIntent::NonConformal { transform: None })).unwrap();
    m.assign_interface(i, &[a.faces[1]], &[b.faces[1]]).unwrap();
    assert!(validate(&m, &t, tol()).has(IssueCode::InterfaceNotFacing));
    // Missing side.
    let mut m = base();
    let i = m.add_boundary("i", BoundaryKind::Interface(InterfaceIntent::NonConformal { transform: None })).unwrap();
    m.assign_interface(i, &[a.faces[1]], &[]).unwrap();
    assert!(validate(&m, &t, tol()).has(IssueCode::InterfaceUnmatched));
    // Opposed normals but far apart: a face without a counterpart.
    let far = brick_at(&mut t, 2, [5.0, 5.0, 5.0], [1.0, 1.0, 1.0]);
    let mut m = base();
    m.color_volumes(m.color_by_name("right").unwrap(), &[far.volume]).unwrap();
    let i = m.add_boundary("i", BoundaryKind::Interface(InterfaceIntent::NonConformal { transform: None })).unwrap();
    m.assign_interface(i, &[a.faces[1]], &[far.faces[0]]).unwrap();
    assert!(validate(&m, &t, tol()).has(IssueCode::InterfaceUnmatched));
    // Same color on both sides warns; an explicit transform is intent and does not.
    let mut m = base();
    let c1 = m.color_by_name("left").unwrap();
    m.color_volumes(c1, &[b.volume]).unwrap();
    let i = m.add_boundary("i", BoundaryKind::Interface(InterfaceIntent::NonConformal { transform: None })).unwrap();
    m.assign_interface(i, &[a.faces[1]], &[b.faces[0]]).unwrap();
    let r = validate(&m, &t, tol());
    assert!(r.warnings().any(|w| w.code == IssueCode::InterfaceSameColor));
    let mut m = base();
    m.color_volumes(c1, &[b.volume]).unwrap();
    let i = m.add_boundary("i", BoundaryKind::Interface(InterfaceIntent::NonConformal { transform: Some(Transform::IDENTITY) })).unwrap();
    m.assign_interface(i, &[a.faces[1]], &[b.faces[0]]).unwrap();
    assert!(!validate(&m, &t, tol()).has(IssueCode::InterfaceSameColor));
}

#[test]
fn merged_faces_derive_a_conformal_interface_and_reject_a_non_conformal_declaration() {
    let mut t = CellComplex::new();
    let (a, b) = with_op(&mut t, OpId(1), fixtures::two_bricks).unwrap();
    with_op(&mut t, OpId(2), |t| fixtures::merge_touching_bricks(t, &a, &b)).unwrap();
    let mut m = Model::new();
    let c1 = m.add_color(1, "left", Domain::Volume).unwrap();
    let c2 = m.add_color(2, "right", Domain::Volume).unwrap();
    m.color_volumes(c1, &[a.volume]).unwrap();
    m.color_volumes(c2, &[b.volume]).unwrap();
    // The merged face is an interior face between two colors: derived Interface(Conformal).
    assert_eq!(m.face_kind(a.faces[1], &t), Some(FaceKindView::Interface { boundary: None, intent: InterfaceIntent::Conformal }));
    finish_boundaries(&mut m, &t, 7, true);
    let r = validate(&m, &t, tol());
    assert!(r.is_ok(), "an interior face between colors needs no explicit kind: {:#?}", r.issues);
    // Same color on both sides: plain interior face.
    m.color_volumes(c1, &[b.volume]).unwrap();
    assert_eq!(m.face_kind(a.faces[1], &t), Some(FaceKindView::Interior));
    // Declaring the merged face non-conformal is rejected.
    let i = m.add_boundary("i", BoundaryKind::Interface(InterfaceIntent::NonConformal { transform: None })).unwrap();
    m.assign_interface(i, &[a.faces[1]], &[]).unwrap();
    assert!(validate(&m, &t, tol()).has(IssueCode::InterfaceUnmatched));
    // Deck kinds on an interior face are rejected as well.
    let mut m2 = Model::new();
    let resp = m2.add_boundary("r", BoundaryKind::Response).unwrap();
    m2.assign_response(resp, &[a.faces[1]]).unwrap();
    assert!(validate(&m2, &t, tol()).has(IssueCode::KindOnInteriorFace));
}

#[test]
fn curved_periodic_disks_validate_by_integrated_moments() {
    let mut t = CellComplex::new();
    let c = with_op(&mut t, OpId(1), |t| fixtures::cylinder(t, 1.0, 3.0)).unwrap();
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[c.volume]).unwrap();
    let per = m.add_boundary("axial", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(0.0, 0.0, 3.0) })).unwrap();
    m.assign_pairs(per, &[(c.faces[0], c.faces[1])]).unwrap();
    let r = validate(&m, &t, tol());
    assert!(!r.has(IssueCode::DeckGeometry) && !r.has(IssueCode::DeckOrientation), "{:#?}", r.issues);
    // Rotating the top disk's pairing partner by a wrong shift is caught.
    let mut m2 = Model::new();
    let per2 = m2.add_boundary("axial", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(0.0, 0.0, 2.0) })).unwrap();
    m2.assign_pairs(per2, &[(c.faces[0], c.faces[1])]).unwrap();
    assert!(validate(&m2, &t, tol()).has(IssueCode::DeckGeometry));
}

#[test]
fn imported_geometry_already_cut_needs_only_the_caps() {
    // Nothing straddles the plane: the kernel returns no lineage, only the cap faces.
    let mut t = CellComplex::new();
    let b = brick_at(&mut t, 1, [0.0; 3], [1.0, 1.0, 1.0]);
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[b.volume]).unwrap();
    finish_boundaries(&mut m, &t, 7, true);
    let cap_color = m.add_color(9, "reflect", Domain::Face).unwrap();
    let surface = EntityId::new(sm_core::EntityKind::Surface, 0).unwrap();
    let out = CutOutcome { lineage: vec![], caps: vec![CapFace { face: b.faces[5], from_surface: surface }] };
    // The +z face lies on z = 1; the face was already a response face, so tagging it fails cleanly.
    let plan = plan_cut(&CutSpec { deck: Deck::Mirror { plane: Plane::new(Vector3::new(0.0, 0.0, 1.0), 1.0).unwrap() }, keep: Keep::Negative, origin: p3(0.0, 0.0, 1.0) }).unwrap();
    let mut probe = m.clone();
    assert_eq!(probe.apply_cut(&plan, &out, Some(cap_color), &t), Err(ModelError::Duplicate(b.faces[5])));
    assert_eq!(probe, m);
    m.unassign(&[b.faces[5]]);
    let boundary = m.apply_cut(&plan, &out, Some(cap_color), &t).unwrap();
    assert_eq!(m.face_boundary(b.faces[5]).unwrap().boundary, boundary);
    assert_eq!(m.face_color(b.faces[5]), Some(cap_color));
    assert!(validate(&m, &t, tol()).is_ok());
}
