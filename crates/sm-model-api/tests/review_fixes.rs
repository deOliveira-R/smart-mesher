//! Regression tests for the review of the final contract: closure on the cut path, panics from
//! public-field inputs, composites by face, interface coincidence, cut-face keys, the cut
//! pipeline end to end for every deck kind, and bounded inputs.

mod common;

use common::*;
use sm_core::{EntityId, EntityKind, Point3, Transform, Vector3};
use sm_model_api::*;
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::{CapFace, CellComplex, CutOutcome, Lineage, OpId, TopologyEdit, TopologyView};

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

fn tol() -> Tolerance {
    Tolerance::default()
}

fn brick_at(t: &mut CellComplex, op: u64, min: [f64; 3], size: [f64; 3]) -> fixtures::BrickIds {
    with_op(t, OpId(op), |t| fixtures::brick(t, p3(min[0], min[1], min[2]), size)).unwrap()
}

fn surface0() -> EntityId {
    EntityId::new(EntityKind::Surface, 0).unwrap()
}

fn colored_brick() -> (CellComplex, Model, fixtures::BrickIds) {
    let mut t = CellComplex::new();
    let b = brick_at(&mut t, 1, [0.0; 3], [1.0, 1.0, 1.0]);
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[b.volume]).unwrap();
    (t, m, b)
}

fn mirror_z(offset: f64, keep: Keep) -> CutPlan {
    plan_cut(&CutSpec { deck: Deck::Mirror { plane: Plane::new(Vector3::new(0.0, 0.0, 1.0), offset).unwrap() }, keep, origin: p3(0.0, 0.0, offset) }).unwrap()
}

// ------------------------------------------------------------- must fix 1: closure on cuts

#[test]
fn two_parallel_mirror_cuts_are_refused_like_two_parallel_mirror_declarations() {
    let (t, mut m, b) = colored_brick();
    let cap_lo = CutOutcome { lineage: vec![], caps: vec![CapFace { face: b.faces[4], from_surface: surface0() }] };
    let cap_hi = CutOutcome { lineage: vec![], caps: vec![CapFace { face: b.faces[5], from_surface: surface0() }] };
    m.apply_cut(&mirror_z(0.0, Keep::Positive), &cap_lo, None, &t).unwrap();
    let before = m.clone();
    // A second mirror parallel to the first composes to a translation: no finite group.
    let r = m.apply_cut(&mirror_z(1.0, Keep::Negative), &cap_hi, None, &t);
    assert!(matches!(r, Err(ModelError::GroupNotFinite { .. })), "{r:?}");
    assert_eq!(m, before, "a refused cut changes nothing");
    assert!(unfold(&m, &t, &UnfoldSpec::default()).is_ok());
}

// ------------------------------------------------------------- must fix 2: composite by face

#[test]
fn composites_of_whole_volumes_are_found_by_face() {
    let mut t = CellComplex::new();
    let a = brick_at(&mut t, 1, [0.0; 3], [1.0, 1.0, 1.0]);
    let b = brick_at(&mut t, 2, [1.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let mut m = Model::new();
    let red = m.add_color(1, "red", Domain::Volume).unwrap();
    m.color_volumes(red, &[a.volume, b.volume]).unwrap();
    let comp = m.composite(&[PieceRef::Whole(a.volume), PieceRef::Whole(b.volume)], &t).unwrap();
    for f in [a.faces[0], a.faces[3], b.faces[1], b.faces[5]] {
        assert_eq!(m.pieces_of_face(f).as_slice(), &[PieceRef::Cell(comp)], "{f}");
        assert_eq!(m.pieces_of_face_in(f, &t).as_slice(), &[PieceRef::Cell(comp)], "{f}");
    }
    m.uncomposite(comp).unwrap();
    assert!(m.pieces_of_face(a.faces[0]).is_empty());
    assert_eq!(m.pieces_of_face_in(a.faces[0], &t).as_slice(), &[PieceRef::Whole(a.volume)]);
}

// ------------------------------------------------------------- must fix 3: no panics

#[test]
fn hostile_turns_and_plans_are_errors_not_panics() {
    let axis = Axis::new(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
    for angle in [
        Turns { num: i32::MAX, den: u32::MAX },
        Turns { num: 2, den: 4 },
        Turns { num: 0, den: 3 },
        Turns { num: -1, den: 6 },
        Turns { num: 6, den: 6 },
        Turns { num: 1, den: 0 },
        Turns { num: 1, den: 1_000_000 },
    ] {
        let deck = Deck::Rotation { axis, angle };
        assert!(deck.check().is_err(), "{angle:?}");
        assert!(plan_cut(&CutSpec { deck, keep: Keep::Positive, origin: p3(1.0, 0.0, 0.0) }).is_err(), "{angle:?}");
        assert!(Model::new().add_boundary("r", BoundaryKind::Deck(deck)).is_err(), "{angle:?}");
    }
    assert!(Deck::Rotation { axis, angle: Turns::new(1, 6).unwrap() }.check().is_ok());
}

#[test]
fn a_hand_built_cut_plan_with_the_wrong_shape_is_refused() {
    let (t, mut m, b) = colored_brick();
    let spec = CutSpec { deck: Deck::Translation { vector: Vector3::new(0.0, 0.0, 1.0) }, keep: Keep::Positive, origin: p3(0.0, 0.0, 0.0) };
    let mut plan = plan_cut(&spec).unwrap();
    plan.half_spaces.pop();
    plan.roles.pop();
    let out = CutOutcome { lineage: vec![], caps: vec![CapFace { face: b.faces[4], from_surface: surface0() }] };
    let before = m.clone();
    assert!(matches!(m.apply_cut(&plan, &out, None, &t), Err(ModelError::Invalid(_))));
    // A mirror plan with a pair of half-spaces is refused as well.
    let mut mirror = mirror_z(0.0, Keep::Positive);
    mirror.half_spaces.push(mirror.half_spaces[0]);
    assert!(matches!(m.apply_cut(&mirror, &out, None, &t), Err(ModelError::Invalid(_))));
    assert_eq!(m, before);
}

#[test]
fn huge_translation_windows_fail_before_allocating() {
    let (t, mut m, b) = colored_brick();
    let px = m.add_boundary("px", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(1.0, 0.0, 0.0) })).unwrap();
    let py = m.add_boundary("py", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(0.0, 1.0, 0.0) })).unwrap();
    m.assign_pairs(px, &[(b.faces[0], b.faces[1])]).unwrap();
    m.assign_pairs(py, &[(b.faces[2], b.faces[3])]).unwrap();
    m.declare_symmetry(&[px, py], &t).unwrap();
    let mut spec = UnfoldSpec::default();
    spec.translation_repeats.insert(px, u32::MAX);
    spec.translation_repeats.insert(py, u32::MAX);
    assert!(matches!(unfold(&m, &t, &spec), Err(ModelError::GroupNotFinite { .. })));
    spec.translation_repeats.insert(px, 3);
    spec.translation_repeats.insert(py, 2);
    assert_eq!(unfold(&m, &t, &spec).unwrap().elements().len(), 6);
}

// ------------------------------------------------------------- must fix 4: coincidence is recorded

#[test]
fn bounds_never_stand_in_for_a_recorded_coincidence_without_a_transform() {
    let mut t = CellComplex::new();
    let a = brick_at(&mut t, 1, [0.0; 3], [1.0, 1.0, 1.0]);
    let b = brick_at(&mut t, 2, [1.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let mut m = Model::new();
    let c1 = m.add_color(1, "left", Domain::Volume).unwrap();
    let c2 = m.add_color(2, "right", Domain::Volume).unwrap();
    m.color_volumes(c1, &[a.volume]).unwrap();
    m.color_volumes(c2, &[b.volume]).unwrap();
    let i = m.add_boundary("slide", BoundaryKind::Interface(InterfaceIntent::NonConformal { transform: None })).unwrap();
    m.assign_interface(i, &[a.faces[1]], &[b.faces[0]]).unwrap();
    finish_boundaries(&mut m, &t, 7, true);
    assert_eq!(validate(&m, &t, tol()).with(IssueCode::InterfaceUnmatched).count(), 2, "touching but unrecorded");
    with_op(&mut t, OpId(3), |t| Ok(t.record_coincidence(a.faces[1], b.faces[0], sm_topo_api::Coincidence::CoincidentUnimprinted)?)).unwrap();
    let r = validate(&m, &t, tol());
    assert!(r.is_ok(), "{:#?}", r.issues);
}

// ------------------------------------------------------------- should fix 1: cut-face keys

#[test]
fn a_cut_face_that_straddles_a_later_cut_gets_distinct_part_keys() {
    let mut t = CellComplex::new();
    let b = brick_at(&mut t, 1, [0.0; 3], [2.0, 1.0, 1.0]);
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[b.volume]).unwrap();
    finish_boundaries(&mut m, &t, 7, true);
    let all = b.faces.to_vec();
    let plane = |x: f64| CutGeometry::Plane(Plane::new(Vector3::new(1.0, 0.0, 0.0), x).unwrap());
    let [n1, p1] = m.split(PieceRef::Whole(b.volume), plane(1.0), &all, &all, &t).unwrap();
    let cut0 = CutId(0);
    // Default: the parent's cut face is assumed to straddle the new cut.
    let [n2, p2] = m.split(PieceRef::Cell(n1), plane(0.5), &all, &all, &t).unwrap();
    let key = |c: EntityId| m.decomposition().piece(c).unwrap().faces.iter().copied().find(|f| matches!(f, PieceFace::CutPart { .. }));
    assert_eq!(key(n2), Some(PieceFace::CutPart { cut: cut0, side: Keep::Negative, piece: n2 }));
    assert_eq!(key(p2), Some(PieceFace::CutPart { cut: cut0, side: Keep::Negative, piece: p2 }));
    assert!(validate(&m, &t, tol()).is_ok());
    // The positive child p1 is untouched.
    assert!(m.decomposition().piece(p1).unwrap().faces.contains(&PieceFace::Cut { cut: cut0, side: Keep::Positive }));

    // Explicit classification: the old cut face lies wholly on the positive side of the new cut.
    m.unsplit(n2).unwrap();
    let neg_cuts: [(CutId, Keep); 0] = [];
    let pos_cuts = [(cut0, Keep::Negative)];
    let [n3, p3_] = m.split_sides(PieceRef::Cell(n1), plane(0.5), &SplitSides { neg: &all, pos: &all, neg_cuts: &neg_cuts, pos_cuts: &pos_cuts }, &t).unwrap();
    let has_old = |c: EntityId| m.decomposition().piece(c).unwrap().faces.contains(&PieceFace::Cut { cut: cut0, side: Keep::Negative });
    assert!(!has_old(n3) && has_old(p3_));
    assert!(validate(&m, &t, tol()).is_ok());
    // Unlisted or foreign cut faces are refused.
    m.unsplit(n3).unwrap();
    let none: [(CutId, Keep); 0] = [];
    assert!(m.split_sides(PieceRef::Cell(n1), plane(0.5), &SplitSides { neg: &all, pos: &all, neg_cuts: &none, pos_cuts: &none }, &t).is_err());
    let alien = [(CutId(77), Keep::Negative)];
    assert!(m.split_sides(PieceRef::Cell(n1), plane(0.5), &SplitSides { neg: &all, pos: &all, neg_cuts: &alien, pos_cuts: &pos_cuts }, &t).is_err());
}

// ------------------------------------------------------------- should fix 2: kept side by corners

#[test]
fn a_volume_the_kernel_failed_to_cut_is_caught_by_its_corners() {
    let (t, mut m, _b) = colored_brick();
    // The centroid z = 0.5 is inside z >= 0.3, but the volume reaches down to z = 0.
    let out = CutOutcome { lineage: vec![], caps: vec![] };
    let before = m.clone();
    let r = m.apply_cut(&mirror_z(0.3, Keep::Positive), &out, None, &t);
    assert!(matches!(r, Err(ModelError::CutInvalid(ref s)) if s.contains("corner")), "{r:?}");
    assert_eq!(m, before);
}

// ------------------------------------------------------------- should fix 3: face lineage

#[test]
fn a_changed_face_of_a_decomposed_volume_is_refused_even_if_the_volume_stays() {
    let (t, mut m, b) = colored_brick();
    let all = b.faces.to_vec();
    m.split(PieceRef::Whole(b.volume), CutGeometry::Plane(Plane::new(Vector3::new(1.0, 0.0, 0.0), 0.5).unwrap()), &all, &all, &t).unwrap();
    let new_face = EntityId::new(EntityKind::TopoFace, 900).unwrap();
    let lineage = [Lineage { old: b.faces[2], new: [new_face].into_iter().collect() }];
    let before = m.clone();
    assert_eq!(m.apply_lineage(&lineage, &t), Err(ModelError::DecompositionActive(b.volume)));
    assert_eq!(m, before);
}

// ------------------------------------------------------------- should fix 4: fingerprint

#[test]
fn quantized_fingerprint_ignores_noise_below_the_step_and_the_raw_one_does_not() {
    let (_t, mut m, b) = colored_brick();
    let proto = m.add_prototype(&[b.volume]).unwrap();
    m.add_instance(InstanceRow { of: Fill::Prototype(proto), at: Transform::translation(Vector3::new(0.5, 0.0, 0.0)), recolor: ColorMap::default(), clip: None }).unwrap();
    let flat = flatten(&m).unwrap();
    let mut noisy = flat.clone();
    noisy.placements[0].transform.t.x += 1e-15;
    assert_ne!(flat.fingerprint(), noisy.fingerprint());
    assert_eq!(flat.fingerprint_quantized(1e-8), noisy.fingerprint_quantized(1e-8));
    noisy.placements[0].transform.t.x += 1e-6;
    assert_ne!(flat.fingerprint_quantized(1e-8), noisy.fingerprint_quantized(1e-8));
}

// ------------------------------------------------------------- should fix 8: bounded layouts

#[test]
fn unbounded_layouts_are_refused() {
    let (_t, mut m, b) = colored_brick();
    let proto = m.add_prototype(&[b.volume]).unwrap();
    let lat = |layout| Lattice { layout, origin: Transform::IDENTITY, outer: Fill::Prototype(proto), slots: Default::default() };
    let huge = Layout::Cartesian { n: [u32::MAX; 3], pitch: Vector3::new(1.0, 1.0, 1.0) };
    assert!(m.add_lattice(lat(huge)).is_err());
    assert_eq!(huge.slot_count(), usize::MAX, "saturates instead of overflowing");
    assert!(huge.indices().is_empty());
    let hex = Layout::Hex { rings: u32::MAX, pitch: 1.0, orient: HexOrient::XRows, nz: u32::MAX, dz: 1.0 };
    assert!(m.add_lattice(lat(hex)).is_err());
    assert!(hex.indices().is_empty());
    assert!(m.add_lattice(lat(Layout::Hex { rings: 5, pitch: 1.0, orient: HexOrient::XRows, nz: 1, dz: 0.0 })).is_ok());
}

// ------------------------------------------------------------- should fix 9: unpaired deletion

#[test]
fn a_cut_that_deletes_one_face_of_a_translation_pair_says_so() {
    let (mut t, mut m, b) = colored_brick();
    let per = m.add_boundary("axial", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(0.0, 0.0, 1.0) })).unwrap();
    m.assign_pairs(per, &[(b.faces[4], b.faces[5])]).unwrap();
    with_op(&mut t, OpId(2), |t| {
        let w = fixtures::webcut_brick(t, &b, 0.5)?;
        t.kill_cascade(w.volumes[1])?;
        Ok(())
    })
    .unwrap();
    let out = t.lineage().cut_outcome(OpId(2));
    let before = m.clone();
    let r = m.apply_cut(&mirror_z(0.5, Keep::Negative), &out, None, &t);
    assert!(matches!(r, Err(ModelError::CutInvalid(ref s)) if s.contains("partner")), "{r:?}");
    assert_eq!(m, before);
}

// ------------------------------------------------------------- should fix 10: cut pipeline

#[test]
fn a_slab_cut_tags_a_translation_pair_of_caps() {
    let (t, mut m, b) = colored_brick();
    let spec = CutSpec { deck: Deck::Translation { vector: Vector3::new(0.0, 0.0, 1.0) }, keep: Keep::Positive, origin: p3(0.0, 0.0, 0.0) };
    let plan = plan_cut(&spec).unwrap();
    let out = CutOutcome { lineage: vec![], caps: vec![CapFace { face: b.faces[4], from_surface: surface0() }, CapFace { face: b.faces[5], from_surface: surface0() }] };
    let cap_color = m.add_color(9, "cap", Domain::Face).unwrap();
    let deck = m.apply_cut(&plan, &out, Some(cap_color), &t).unwrap();
    assert_eq!(m.face_boundary(b.faces[4]).unwrap().role, FaceRole::Deck { side: DeckSide::A, mate: Some(b.faces[5]) });
    assert_eq!(m.face_boundary(b.faces[5]).unwrap().role, FaceRole::Deck { side: DeckSide::B, mate: Some(b.faces[4]) });
    assert_eq!(m.face_boundary(b.faces[4]).unwrap().boundary, deck);
    assert_eq!(m.face_color(b.faces[5]), Some(cap_color));
    finish_boundaries(&mut m, &t, 7, true);
    let r = validate(&m, &t, tol());
    assert!(r.is_ok(), "{:#?}", r.issues);
    let mut ur = UnfoldSpec::default();
    ur.translation_repeats.insert(deck, 3);
    let u = unfold(&m, &t, &ur).unwrap();
    assert_eq!(u.glued().len(), 2);
    assert!(validate_unfolded(&u, &m, &t, tol()).is_ok());
    // Caps on the wrong planes are refused.
    let (t2, mut m2, b2) = colored_brick();
    let shifted = CutSpec { origin: p3(0.0, 0.0, 0.25), ..spec };
    let bad = m2.apply_cut(&plan_cut(&shifted).unwrap(), &CutOutcome { lineage: vec![], caps: out.caps.iter().map(|c| CapFace { face: if c.face == b.faces[4] { b2.faces[4] } else { b2.faces[5] }, ..*c }).collect() }, None, &t2);
    assert!(matches!(bad, Err(ModelError::CutInvalid(_))), "{bad:?}");
}

#[test]
fn a_wedge_cut_tags_a_rotation_pair_of_caps() {
    let mut t = CellComplex::new();
    let p = prism_op(&mut t, 1, &[(0.0, 0.0), (1.0, 0.0), (0.5, 3f64.sqrt() / 2.0)], 0.0, 1.0);
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[p.volume]).unwrap();
    let axis = Axis::new(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
    let spec = CutSpec { deck: Deck::Rotation { axis, angle: Turns::new(1, 6).unwrap() }, keep: Keep::Positive, origin: p3(1.0, 0.0, 0.0) };
    let plan = plan_cut(&spec).unwrap();
    let out = CutOutcome { lineage: vec![], caps: vec![CapFace { face: p.sides[0], from_surface: surface0() }, CapFace { face: p.sides[2], from_surface: surface0() }] };
    let deck = m.apply_cut(&plan, &out, None, &t).unwrap();
    assert_eq!(m.face_boundary(p.sides[0]).unwrap().role, FaceRole::Deck { side: DeckSide::A, mate: Some(p.sides[2]) });
    finish_boundaries(&mut m, &t, 7, true);
    let r = validate(&m, &t, tol());
    assert!(r.is_ok(), "{:#?}", r.issues);
    assert_eq!(m.symmetry().unwrap().generators, vec![deck]);
    let u = unfold(&m, &t, &UnfoldSpec::default()).unwrap();
    assert_eq!(u.elements().len(), 6);
    assert!(validate_unfolded(&u, &m, &t, tol()).is_ok());
}

#[test]
fn detect_declare_plan_cut_kernel_apply_chain() {
    let (mut t, mut m, b) = colored_brick();
    finish_boundaries(&mut m, &t, 7, true);
    let det = detect_symmetry(&m, &t, tol()).unwrap();
    let cand = det
        .verified()
        .find(|c| matches!(c.kind, CandidateKind::Mirror(p) if p.normal.z.abs() > 0.99))
        .expect("the cube has a verified z mirror")
        .clone();
    let spec = m.declare_detected(&cand, Keep::Negative, false).unwrap();
    let plan = plan_cut(&spec).unwrap();
    with_op(&mut t, OpId(2), |t| {
        let w = fixtures::webcut_brick(t, &b, 0.5)?;
        t.kill_cascade(w.volumes[1])?;
        Ok(())
    })
    .unwrap();
    let out = t.lineage().cut_outcome(OpId(2));
    let cap_color = m.add_color(99, "reflect", Domain::Face).unwrap();
    m.apply_cut(&plan, &out, Some(cap_color), &t).unwrap();
    assert!(matches!(m.provenance()[0], Provenance::Detected(Verification::Verified { .. })));
    assert!(matches!(m.provenance()[1], Provenance::Cut { .. }));
    let r = validate(&m, &t, tol());
    assert!(r.is_ok(), "{:#?}", r.issues);
    assert!(unfold(&m, &t, &UnfoldSpec::default()).is_ok());
}

// ------------------------------------------------------------- should fix 11: probing

#[test]
fn a_large_cyclic_group_closes_with_many_zero_entries() {
    // Rotations about axes aligned with the coordinate system have many exact zeros; the
    // closure must find every element (no misses from capped neighbour probing).
    let x = Transform::rotation(p3(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0), core::f64::consts::TAU / 4.0).unwrap();
    let z = Transform::rotation(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0), core::f64::consts::TAU / 4.0).unwrap();
    assert_eq!(point_group(&[x, z], 4096).unwrap().len(), 24, "the rotation group of the cube");
    let mx = Plane::new(Vector3::new(1.0, 0.0, 0.0), 0.0).unwrap().reflection();
    assert_eq!(point_group(&[x, z, mx], 4096).unwrap().len(), 48);
}
