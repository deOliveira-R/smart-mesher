//! Scenario tests for the model-semantics layer against the mock topology.

use sm_core::{EntityId, EntityKind, Point3, Transform, Vector3};
use sm_model_api::boundary::{BoundaryKind, DeckSide, DeckSides, InterfaceIntent};
use sm_model_api::coloring::{ColorRole, GroupKind};
use sm_model_api::decomposition::{CutGeometry, Decomposition, PieceFace, PieceShape, PieceSpec, SweepRoles};
use sm_model_api::detect::{DetectParams, RejectReason, Verification, detect};
use sm_model_api::group::{DeckElement, DeckGroup, DeckOp, ElemIx, GroupAction, new_generator};
use sm_model_api::ids::{IdAllocator, SequentialIds};
use sm_model_api::instancing::*;
use sm_model_api::mock::{BoxSpec, MockTopology};
use sm_model_api::symmetry::{CutReport, CutSpec, Glue, Lineage, Side, CapFace, plan_cut};
use sm_model_api::topo::{GeometryView, TopologyView, faces_on_plane};
use sm_model_api::validate::{IssueCode, validate};
use sm_model_api::xf::{Axis, Plane, Tolerance, Turns, approx_eq, compose, eigen_symmetric, inverse};
use sm_model_api::Model;

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}
fn bx(lo: [f64; 3], hi: [f64; 3]) -> BoxSpec {
    BoxSpec { lo, hi }
}
fn near(a: Point3, b: Point3) -> bool {
    (a - b).norm() < 1e-9
}
fn tol() -> Tolerance {
    Tolerance::default()
}

// ---------------------------------------------------------------- xf / group basics

#[test]
fn transform_algebra() {
    let plane = Plane::new(p(1.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap();
    let m = plane.reflection();
    assert!(m.is_improper());
    assert!(near(m.apply_point(p(3.0, 2.0, 1.0)), p(-1.0, 2.0, 1.0)));
    assert!(near(m.apply_point(p(1.0, 5.0, 5.0)), p(1.0, 5.0, 5.0)), "plane fixed pointwise");
    assert!(approx_eq(&compose(&m, &m), &Transform::IDENTITY, 1e-12));

    let rot = Axis::new(p(1.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap().rotation(Turns::one_over(4).unwrap().radians());
    assert!(near(rot.apply_point(p(2.0, 1.0, 0.0)), p(1.0, 2.0, 0.0)));
    let inv = inverse(&rot).unwrap();
    assert!(approx_eq(&compose(&rot, &inv), &Transform::IDENTITY, 1e-12));
    assert_eq!(Turns::new(2, 6).unwrap().order(), 3);
    assert!(Turns::new(0, 6).is_err());
}

#[test]
fn eigen_matches_analytic() {
    let a = [[2.0, 1.0, 0.0], [1.0, 2.0, 0.0], [0.0, 0.0, 5.0]];
    let (vals, vecs) = eigen_symmetric(&a);
    assert!((vals[0] - 5.0).abs() < 1e-12 && (vals[1] - 3.0).abs() < 1e-12 && (vals[2] - 1.0).abs() < 1e-12);
    for (l, v) in vals.iter().zip(vecs) {
        let av = Vector3::new(
            a[0][0] * v.x + a[0][1] * v.y + a[0][2] * v.z,
            a[1][0] * v.x + a[1][1] * v.y + a[1][2] * v.z,
            a[2][0] * v.x + a[2][1] * v.y + a[2][2] * v.z,
        );
        assert!((av - v * *l).norm() < 1e-12);
    }
}

#[test]
fn moments_transform_matches_analytic_box() {
    let b = bx([0.0, 0.0, 0.0], [1.0, 2.0, 3.0]);
    let shifted = bx([4.0, 0.0, 0.0], [5.0, 2.0, 3.0]);
    let t = sm_model_api::xf::translation(Vector3::new(4.0, 0.0, 0.0));
    assert!(b.moments().transformed(&t).approx_eq(&shifted.moments(), &tol()));
    let mir = Plane::new(p(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap().reflection();
    let mirrored = bx([-1.0, 0.0, 0.0], [0.0, 2.0, 3.0]);
    assert!(b.moments().transformed(&mir).approx_eq(&mirrored.moments(), &tol()));
}

#[test]
fn group_closure_orders() {
    let mut ids = SequentialIds::new();
    let z = Axis::new(p(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
    let c6 = new_generator(&mut ids, DeckOp::Rotation { axis: z, turns: Turns::one_over(6).unwrap() }).unwrap();
    let g = DeckGroup::generate(&[c6]).unwrap();
    assert_eq!(g.order(), 6);
    assert!(g.elements().iter().all(|e| !e.improper));
    // R^6 = identity through the group table
    let r = ElemIx(1);
    let mut acc = g.identity();
    for _ in 0..6 {
        acc = g.mul(acc, r).unwrap();
    }
    assert_eq!(acc, g.identity());
    assert_eq!(g.mul(r, g.inverse_of(r).unwrap()), Some(g.identity()));

    // D6: rotation + mirror through the x axis -> order 12, half improper
    let m = new_generator(&mut ids, DeckOp::Mirror { plane: Plane::new(p(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)).unwrap() }).unwrap();
    let d6 = DeckGroup::generate(&[c6, m]).unwrap();
    assert_eq!(d6.order(), 12);
    assert_eq!(d6.elements().iter().filter(|e| e.improper).count(), 6);

    // translation window x mirror
    let t = new_generator(&mut ids, DeckOp::Translation { vector: Vector3::new(2.0, 0.0, 0.0), copies: 3 }).unwrap();
    let mx = new_generator(&mut ids, DeckOp::Mirror { plane: Plane::new(p(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap() }).unwrap();
    assert_eq!(DeckGroup::generate(&[t, mx]).unwrap().order(), 6);

    // non-closing generator set -> typed error, not a hang
    let a = Axis::new(p(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
    let irr = new_generator(&mut ids, DeckOp::Rotation { axis: a, turns: Turns::new(1, 100_000).unwrap() }).unwrap();
    assert!(DeckGroup::generate(&[irr]).is_err());
}

// ---------------------------------------------------------------- 1. coloring

struct TwoMat {
    ids: SequentialIds,
    topo: MockTopology,
    model: Model,
    vols: Vec<EntityId>,
    fuel: EntityId,
    clad: EntityId,
    outer: EntityId,
}

fn two_material() -> TwoMat {
    let mut ids = SequentialIds::new();
    let (topo, vols) = MockTopology::from_boxes(&mut ids, &[bx([0.0; 3], [1.0, 1.0, 1.0]), bx([1.0, 0.0, 0.0], [2.0, 1.0, 1.0])]);
    let mut model = Model::new();
    let fuel = model.coloring.define_color(&mut ids, "fuel", ColorRole::Volume).unwrap();
    let clad = model.coloring.define_color(&mut ids, "clad", ColorRole::Volume).unwrap();
    let outer = model.coloring.define_color(&mut ids, "outer", ColorRole::Boundary).unwrap();
    model.coloring.assign_volume(vols[0], fuel).unwrap();
    model.coloring.assign_volume(vols[1], clad).unwrap();
    for f in topo.faces() {
        if topo.face_sides(f).unwrap().is_boundary() {
            model.boundaries.tag_response(&[f]).unwrap();
            model.coloring.assign_face(f, outer).unwrap();
        } else {
            model.boundaries.tag_interface(f, InterfaceIntent::BetweenColors).unwrap();
        }
    }
    TwoMat { ids, topo, model, vols, fuel, clad, outer }
}

#[test]
fn coloring_two_materials_with_boundary_colors() {
    let mut t = two_material();
    let rep = validate(&t.topo, &t.model, None, &tol());
    assert!(rep.is_ok(), "{:?}", rep.issues);
    assert_eq!(t.model.coloring.volumes_of(t.fuel), vec![t.vols[0]]);
    assert_eq!(t.model.coloring.faces_of(t.outer).len(), 10, "2 boxes: 12 faces - 2 merged = 11 faces, 1 interior");
    // role discipline: a boundary color cannot color a volume, a volume color cannot color a face
    assert!(t.model.coloring.assign_volume(t.vols[0], t.outer).is_err());
    let f = t.topo.faces()[0];
    assert!(t.model.coloring.assign_face(f, t.fuel).is_err());
    // duplicate names within a role are rejected; the same name in the other role is fine
    assert!(t.model.coloring.define_color(&mut t.ids, "fuel", ColorRole::Volume).is_err());
    assert!(t.model.coloring.define_color(&mut t.ids, "fuel", ColorRole::Boundary).is_ok());
    // recoloring keeps the partition exact: one color per volume
    t.model.coloring.assign_volume(t.vols[0], t.clad).unwrap();
    assert_eq!(t.model.coloring.volumes_of(t.clad).len(), 2);
    assert!(t.model.coloring.volumes_of(t.fuel).is_empty());
}

#[test]
fn validator_reports_missing_color_and_kind() {
    let mut t = two_material();
    let mut broken = t.model.clone();
    broken.coloring.retire(t.vols[1]);
    let first_boundary = t.topo.boundary_faces()[0];
    broken.boundaries.retire(first_boundary);
    let rep = validate(&t.topo, &broken, None, &tol());
    assert!(!rep.is_ok());
    assert!(rep.has(IssueCode::UncoloredVolume));
    assert!(rep.has(IssueCode::BoundaryFaceWithoutKind));
    // a BetweenColors interface between same-color volumes is only a warning
    t.model.coloring.assign_volume(t.vols[1], t.fuel).unwrap();
    let rep = validate(&t.topo, &t.model, None, &tol());
    assert!(rep.is_ok() && rep.has(IssueCode::BadInterface));
}

#[test]
fn named_groups_nest_and_resolve() {
    let mut t = two_material();
    let blk = t.model.coloring.define_group(&mut t.ids, "all_fuel", GroupKind::Block).unwrap();
    let asm = t.model.coloring.define_group(&mut t.ids, "assembly", GroupKind::Assembly).unwrap();
    t.model.coloring.add_to_group(blk, t.vols[0]).unwrap();
    t.model.coloring.add_to_group(asm, blk).unwrap();
    t.model.coloring.add_to_group(asm, t.vols[1]).unwrap();
    assert_eq!(t.model.coloring.expand_group(asm, EntityKind::TopoVolume).unwrap(), vec![t.vols[0], t.vols[1]]);
    assert!(t.model.coloring.add_to_group(blk, asm).is_err(), "cycle refused");
    assert!(t.model.coloring.add_to_group(asm, t.model.coloring.group_by_name("assembly").unwrap()).is_err());
}

// ---------------------------------------------------------------- 3. detection

fn d2h_boxes() -> Vec<BoxSpec> {
    vec![bx([-2.0, 0.0, 0.0], [-1.0, 1.0, 2.0]), bx([-1.0, 0.0, 0.0], [1.0, 1.0, 2.0]), bx([1.0, 0.0, 0.0], [2.0, 1.0, 2.0])]
}

fn color_boxes(ids: &mut SequentialIds, vols: &[EntityId], pattern: &[usize]) -> Model {
    let mut model = Model::new();
    let cols = [
        model.coloring.define_color(ids, "fuel", ColorRole::Volume).unwrap(),
        model.coloring.define_color(ids, "clad", ColorRole::Volume).unwrap(),
    ];
    for (v, &c) in vols.iter().zip(pattern) {
        model.coloring.assign_volume(*v, cols[c]).unwrap();
    }
    model
}

#[test]
fn detection_finds_exactly_the_d2h_elements() {
    let mut ids = SequentialIds::new();
    let (topo, vols) = MockTopology::from_boxes(&mut ids, &d2h_boxes());
    let model = color_boxes(&mut ids, &vols, &[1, 0, 1]);
    let rep = detect(&topo, &model.coloring, Some(&model.boundaries), &DetectParams::default()).unwrap();
    assert!(near(rep.centroid, p(0.0, 0.5, 1.0)));
    let verified: Vec<_> = rep.verified().collect();
    let mirrors = verified.iter().filter(|c| matches!(c.op, DeckOp::Mirror { .. })).count();
    let rots = verified.iter().filter(|c| matches!(c.op, DeckOp::Rotation { .. })).count();
    assert_eq!((mirrors, rots), (3, 3), "3 mirror planes + 3 C2 axes of a box stack: {:?}", verified);
    assert!(verified.iter().all(|c| matches!(c.op, DeckOp::Mirror { .. }) || c.order == 2));
    assert_eq!(rep.verified_group_order(&mut ids).unwrap(), 8);
    // the x mirror passes through the interface plane x = 0
    let has_x = verified.iter().any(|c| matches!(c.op, DeckOp::Mirror { plane } if plane.normal().x.abs() > 0.99 && plane.point().x.abs() < 1e-9));
    assert!(has_x);
}

#[test]
fn detection_rejects_when_screening_passes_but_volumes_differ() {
    let mut ids = SequentialIds::new();
    let boxes = vec![
        bx([-2.0, 0.0, 0.0], [-1.0, 1.0, 2.0]),
        bx([-1.0, 0.0, 0.0], [1.0, 1.0, 2.0]),
        bx([1.0, 0.0, 0.0], [1.5, 1.0, 2.0]),
        bx([1.5, 0.0, 0.0], [2.0, 1.0, 2.0]),
    ];
    let (topo, vols) = MockTopology::from_boxes(&mut ids, &boxes);
    let model = color_boxes(&mut ids, &vols, &[1, 0, 1, 1]);
    let rep = detect(&topo, &model.coloring, None, &DetectParams::default()).unwrap();
    let xm = rep.candidates.iter().find(|c| matches!(c.op, DeckOp::Mirror { plane } if plane.normal().x.abs() > 0.99)).expect("screened candidate present");
    assert!(matches!(xm.verification, Verification::Rejected(RejectReason::VolumeNotMapped(_))), "{:?}", xm.verification);
    // the transverse mirrors are still real symmetries
    assert!(rep.verified().any(|c| matches!(c.op, DeckOp::Mirror { plane } if plane.normal().y.abs() > 0.99)));
    // uncolored model: no symmetry statement possible
    assert!(detect(&topo, &sm_model_api::coloring::Coloring::new(), None, &DetectParams::default()).is_err());
}

// ---------------------------------------------------------------- 3. mirror cut keep chamber + unfold

fn full_model_for_cut() -> (SequentialIds, MockTopology, Vec<EntityId>, Model, EntityId, EntityId) {
    let mut ids = SequentialIds::new();
    let (topo, vols) = MockTopology::from_boxes(&mut ids, &d2h_boxes());
    let mut model = color_boxes(&mut ids, &vols, &[1, 0, 1]);
    let outer = model.coloring.define_color(&mut ids, "outer", ColorRole::Boundary).unwrap();
    for f in topo.faces() {
        if topo.face_sides(f).unwrap().is_boundary() {
            model.boundaries.tag_response(&[f]).unwrap();
            model.coloring.assign_face(f, outer).unwrap();
        } else {
            model.boundaries.tag_interface(f, InterfaceIntent::BetweenColors).unwrap();
        }
    }
    let fuel = model.coloring.color_by_name(ColorRole::Volume, "fuel").unwrap();
    (ids, topo, vols, model, fuel, outer)
}

#[test]
fn mirror_cut_keep_chamber_then_unfold_back() {
    let (mut ids, old, old_vols, mut model, fuel, outer) = full_model_for_cut();
    assert!(validate(&old, &model, None, &tol()).is_ok());
    let old_total = old.volumes().iter().fold(sm_model_api::topo::Moments::ZERO, |a, v| a.add(&old.volume_moments(*v).unwrap()));

    let plane = Plane::new(p(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap();
    let plan = plan_cut(&CutSpec::Mirror { plane, keep: Side::Positive }, &mut ids).unwrap();

    // What the topology engine would hand back: the chamber x >= 0.
    let (new, nv) = MockTopology::from_boxes(&mut ids, &[bx([0.0; 3], [1.0, 1.0, 2.0]), bx([1.0, 0.0, 0.0], [2.0, 1.0, 2.0])]);
    let of = |c: Point3| old.find_face(c, 1e-9).unwrap();
    let nf = |c: Point3| new.find_face(c, 1e-9).unwrap();
    let pairs = [
        (p(0.0, 0.0, 1.0), p(0.5, 0.0, 1.0)),
        (p(0.0, 1.0, 1.0), p(0.5, 1.0, 1.0)),
        (p(0.0, 0.5, 0.0), p(0.5, 0.5, 0.0)),
        (p(0.0, 0.5, 2.0), p(0.5, 0.5, 2.0)),
        (p(1.0, 0.5, 1.0), p(1.0, 0.5, 1.0)),
        (p(1.5, 0.0, 1.0), p(1.5, 0.0, 1.0)),
        (p(1.5, 1.0, 1.0), p(1.5, 1.0, 1.0)),
        (p(1.5, 0.5, 0.0), p(1.5, 0.5, 0.0)),
        (p(1.5, 0.5, 2.0), p(1.5, 0.5, 2.0)),
        (p(2.0, 0.5, 1.0), p(2.0, 0.5, 1.0)),
    ];
    let report = CutReport {
        volumes: vec![Lineage { old: old_vols[1], new: nv[0] }, Lineage { old: old_vols[2], new: nv[1] }],
        faces: pairs.iter().map(|&(o, n)| Lineage { old: of(o), new: nf(n) }).collect(),
        removed_volumes: vec![old_vols[0]],
        removed_faces: old.faces().into_iter().filter(|f| !pairs.iter().any(|&(o, _)| of(o) == *f)).collect(),
        caps: vec![CapFace { face: nf(p(0.0, 0.5, 1.0)), plane: 0 }],
    };
    let cap_color = model.coloring.define_color(&mut ids, "symmetry_plane", ColorRole::Boundary).unwrap();
    let deck = model.apply_cut(&plan, &report, Some(cap_color), &mut ids).unwrap();

    // Colors and kinds were inherited by the children; retired entities are gone.
    assert_eq!(model.coloring.volume_color(nv[0]), Some(fuel));
    assert!(model.coloring.volume_color(old_vols[1]).is_none());
    assert_eq!(model.coloring.face_color(nf(p(2.0, 0.5, 1.0))), Some(outer));
    assert!(matches!(model.boundaries.kind_of(nf(p(1.0, 0.5, 1.0))), Some(BoundaryKind::Interface(InterfaceIntent::BetweenColors))));
    // The cap is a self-paired deck boundary with its own boundary color.
    let cap = nf(p(0.0, 0.5, 1.0));
    assert!(matches!(model.boundaries.kind_of(cap), Some(BoundaryKind::Deck { side: DeckSide::SelfPaired, boundary }) if *boundary == deck));
    assert_eq!(model.coloring.face_color(cap), Some(cap_color));
    let rep = validate(&new, &model, None, &tol());
    assert!(rep.is_ok(), "{:?}", rep.issues);
    assert_eq!(faces_on_plane(&new, &plane, &tol()), vec![cap], "pre-cut import path finds the same face");

    // Unfold: 2 copies, the second is improper; colors pulled back; cap glued to itself in the mirror copy.
    let sym = model.symmetry.clone().unwrap();
    let cover = sym.unfold().unwrap();
    assert_eq!(cover.copies(), 2);
    assert!(!cover.reverses_orientation(ElemIx(0)) && cover.reverses_orientation(ElemIx(1)));
    let ents = cover.entities(&nv);
    assert_eq!(ents.len(), 4);
    assert_eq!(cover.volume_color(&model.coloring, ents[2]), Some(fuel), "mirror copy of the fuel half is fuel");
    assert_eq!(cover.glue(&model.boundaries, cap, ElemIx(0)), Glue::Glued { boundary: deck, element: ElemIx(1), side: DeckSide::SelfPaired });
    assert_eq!(cover.glue(&model.boundaries, cap, ElemIx(1)), Glue::Glued { boundary: deck, element: ElemIx(0), side: DeckSide::SelfPaired });
    assert_eq!(cover.glue(&model.boundaries, nf(p(2.0, 0.5, 1.0)), ElemIx(0)), Glue::NotDeck);
    assert!(cover.open_deck_faces(&model.boundaries).is_empty());
    // The unfolded model is the original: same measure, barycentre and second moments.
    assert!(cover.moments(&new).approx_eq(&old_total, &tol()));
    // Reversible: refolding returns the identical symmetry; a full-state symmetry cannot unfold.
    assert_eq!(cover.refold(), sym);
    let mut full = sym.clone();
    full.domain.state = sm_model_api::symmetry::DomainState::Full;
    assert!(full.unfold().is_err());
}

#[test]
fn imported_pre_cut_geometry_is_tagged_not_cut() {
    let (mut ids, ..) = full_model_for_cut();
    let (new, nv) = MockTopology::from_boxes(&mut ids, &[bx([0.0; 3], [1.0, 1.0, 2.0])]);
    let mut model = color_boxes(&mut ids, &nv, &[0]);
    let plane = Plane::new(p(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap();
    let cap: Vec<EntityId> = faces_on_plane(&new, &plane, &tol());
    assert_eq!(cap.len(), 1);
    let others: Vec<EntityId> = new.boundary_faces().into_iter().filter(|f| !cap.contains(f)).collect();
    model.boundaries.tag_response(&others).unwrap();
    model.declare_deck(&mut ids, DeckOp::Mirror { plane }, DeckSides::SelfPaired(cap.iter().copied().collect()), vec![]).unwrap();
    assert!(validate(&new, &model, None, &tol()).is_ok());
    // a mirror tag on a face that is NOT on the plane is an error
    let mut bad = model.clone();
    let wrong = others[0];
    bad.boundaries.retire(wrong);
    let sym_gen = bad.symmetry.as_ref().unwrap().generators[0].id;
    bad.boundaries.add_deck(&mut ids, sym_gen, DeckSides::SelfPaired([wrong].into_iter().collect())).unwrap();
    assert!(validate(&new, &bad, None, &tol()).has(IssueCode::MirrorFaceOffPlane));
}

// ---------------------------------------------------------------- 2. translation pair (periodic slab)

fn slab_case(b_x: f64) -> (SequentialIds, MockTopology, Model, sm_model_api::symmetry::CutPlan, CutReport, EntityId) {
    let mut ids = SequentialIds::new();
    let plane = Plane::new(p(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap();
    let plan = plan_cut(&CutSpec::Slab { plane, vector: Vector3::new(2.0, 0.0, 0.0), copies: 3 }, &mut ids).unwrap();
    let (topo, vols) = MockTopology::from_boxes(&mut ids, &[bx([0.0; 3], [b_x, 1.0, 1.0])]);
    let mut model = color_boxes(&mut ids, &vols, &[0]);
    let a = topo.find_face(p(0.0, 0.5, 0.5), 1e-9).unwrap();
    let b = topo.find_face(p(b_x, 0.5, 0.5), 1e-9).unwrap();
    let side: Vec<EntityId> = topo.boundary_faces().into_iter().filter(|f| *f != a && *f != b).collect();
    model.boundaries.tag_response(&side).unwrap();
    let report = CutReport { volumes: vec![], faces: vec![], removed_volumes: vec![], removed_faces: vec![], caps: vec![CapFace { face: a, plane: 0 }, CapFace { face: b, plane: 1 }] };
    (ids, topo, model, plan, report, a)
}

#[test]
fn translation_pair_validates_and_unfolds_with_open_ends() {
    let (mut ids, topo, mut model, plan, report, a) = slab_case(2.0);
    let deck = model.apply_cut(&plan, &report, None, &mut ids).unwrap();
    let rep = validate(&topo, &model, None, &tol());
    assert!(rep.is_ok(), "{:?}", rep.issues);
    let b = topo.find_face(p(2.0, 0.5, 0.5), 1e-9).unwrap();
    assert!(matches!(model.boundaries.kind_of(a), Some(BoundaryKind::Deck { side: DeckSide::A, .. })));
    assert!(matches!(model.boundaries.kind_of(b), Some(BoundaryKind::Deck { side: DeckSide::B, .. })));

    let cover = model.symmetry.as_ref().unwrap().unfold().unwrap();
    assert_eq!(cover.copies(), 3);
    for (i, e) in cover.group().elements().iter().enumerate() {
        assert!(near(e.xf.apply_point(p(0.0, 0.0, 0.0)), p(2.0 * i as f64, 0.0, 0.0)));
        assert!(!e.improper);
    }
    // B of copy k is glued to A of copy k+1 (B_k = A_{k+1}); the window ends stay open.
    assert_eq!(cover.glue(&model.boundaries, b, ElemIx(0)), Glue::Glued { boundary: deck, element: ElemIx(1), side: DeckSide::A });
    assert_eq!(cover.glue(&model.boundaries, a, ElemIx(2)), Glue::Glued { boundary: deck, element: ElemIx(1), side: DeckSide::B });
    assert_eq!(cover.glue(&model.boundaries, a, ElemIx(0)), Glue::Open);
    assert_eq!(cover.glue(&model.boundaries, b, ElemIx(2)), Glue::Open);
    assert_eq!(cover.open_deck_faces(&model.boundaries).len(), 2);
    assert!((cover.moments(&topo).measure - 6.0).abs() < 1e-12);
    // partner lookup by geometry
    let op = model.deck_op(model.boundaries.deck(deck).unwrap().generator).unwrap();
    let partner = sm_model_api::symmetry::deck_partner(&topo, &op, &model.boundaries.deck(deck).unwrap().sides, DeckSide::A, a, &tol());
    assert_eq!(partner, Some(b));
}

#[test]
fn translation_pair_with_wrong_period_is_flagged() {
    let (mut ids, topo, mut model, plan, report, _) = slab_case(3.0);
    model.apply_cut(&plan, &report, None, &mut ids).unwrap();
    assert!(validate(&topo, &model, None, &tol()).has(IssueCode::DeckPairUnmatched));
}

// ---------------------------------------------------------------- 2. one-sixth rotation wedge

struct Wedge {
    ids: SequentialIds,
    topo: MockTopology,
    model: Model,
    deck: EntityId,
    a: EntityId,
    b: EntityId,
    v: EntityId,
}

fn wedge(b_normal_sign: f64) -> Wedge {
    let mut ids = SequentialIds::new();
    let (r, h) = (1.0, 1.0);
    let th = std::f64::consts::FRAC_PI_3;
    let mut topo = MockTopology::new();
    let v = ids.alloc(EntityKind::TopoVolume);
    topo.add_volume(v, sm_model_api::topo::Moments { measure: 0.5 * th * r * r * h, centroid: p(0.6, 0.35, 0.5), second: [[0.0; 3]; 3] });
    let side = sm_model_api::topo::FaceSides { fwd: Some(v), rev: None };
    let a = ids.alloc(EntityKind::TopoFace);
    topo.add_face(a, side, p(r / 2.0, 0.0, h / 2.0), r * h, Vector3::new(0.0, -1.0, 0.0));
    let b = ids.alloc(EntityKind::TopoFace);
    topo.add_face(b, side, p(r / 2.0 * th.cos(), r / 2.0 * th.sin(), h / 2.0), r * h, Vector3::new(-th.sin(), th.cos(), 0.0) * b_normal_sign);
    let (o, t, bt) = (ids.alloc(EntityKind::TopoFace), ids.alloc(EntityKind::TopoFace), ids.alloc(EntityKind::TopoFace));
    topo.add_face(o, side, p(0.95, 0.5, 0.5), th * r * h, Vector3::new(0.866, 0.5, 0.0));
    topo.add_face(t, side, p(0.4, 0.3, h), 0.5 * th * r * r, Vector3::new(0.0, 0.0, 1.0));
    topo.add_face(bt, side, p(0.4, 0.3, 0.0), 0.5 * th * r * r, Vector3::new(0.0, 0.0, -1.0));
    let mut model = Model::new();
    let m1 = model.coloring.define_color(&mut ids, "m1", ColorRole::Volume).unwrap();
    model.coloring.assign_volume(v, m1).unwrap();
    model.boundaries.tag_response(&[o, t, bt]).unwrap();
    let z = Axis::new(p(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
    let plan = plan_cut(&CutSpec::Wedge { axis: z, n: 6, start: Vector3::new(1.0, 0.0, 0.0) }, &mut ids).unwrap();
    // the two planes bound the wedge 0..60 degrees
    assert!(plan.planes[0].keep.contains(p(0.8, 0.3, 0.0), 1e-12) && plan.planes[1].keep.contains(p(0.8, 0.3, 0.0), 1e-12));
    assert!(!plan.planes[0].keep.contains(p(0.8, -0.1, 0.0), 1e-12) && !plan.planes[1].keep.contains(p(0.2, 0.9, 0.0), 1e-12));
    let report = CutReport {
        volumes: vec![Lineage { old: v, new: v }],
        faces: vec![],
        removed_volumes: vec![],
        removed_faces: vec![],
        caps: vec![CapFace { face: a, plane: 0 }, CapFace { face: b, plane: 1 }],
    };
    let deck = model.apply_cut(&plan, &report, None, &mut ids).unwrap();
    Wedge { ids, topo, model, deck, a, b, v }
}

#[test]
fn one_sixth_rotation_wedge_closes_after_six_copies() {
    let w = wedge(1.0);
    let rep = validate(&w.topo, &w.model, None, &tol());
    assert!(rep.is_ok(), "{:?}", rep.issues);
    let cover = w.model.symmetry.as_ref().unwrap().unfold().unwrap();
    assert_eq!(cover.copies(), 6);
    assert!((0..6).all(|i| !cover.reverses_orientation(ElemIx(i))), "rotations are proper");
    assert!(cover.open_deck_faces(&w.model.boundaries).is_empty(), "no deck face stays open after 6 copies");
    // B of copy e == A of copy e+1: the element glued to B_0 is the rotation by +60 degrees.
    let r1 = sm_model_api::xf::Axis::new(p(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap().rotation(std::f64::consts::FRAC_PI_3);
    let Glue::Glued { element: eb, side: sb, .. } = cover.glue(&w.model.boundaries, w.b, ElemIx(0)) else { panic!("B must be glued") };
    assert_eq!(sb, DeckSide::A);
    assert!(approx_eq(&cover.element(eb).unwrap().xf, &r1, 1e-9));
    let Glue::Glued { element: ea, side: sa, boundary } = cover.glue(&w.model.boundaries, w.a, ElemIx(0)) else { panic!("A must be glued") };
    assert_eq!((sa, boundary), (DeckSide::B, w.deck));
    assert!(approx_eq(&cover.element(ea).unwrap().xf, &inverse(&r1).unwrap(), 1e-9));
    // 6 copies x 3 non-deck boundary faces, and the cover has the measure of the full cylinder.
    let non_deck = w.topo.boundary_faces().into_iter().filter(|&f| cover.glue(&w.model.boundaries, f, ElemIx(0)) == Glue::NotDeck).count();
    assert_eq!(non_deck * cover.copies(), 18);
    assert!((cover.moments(&w.topo).measure - std::f64::consts::PI).abs() < 1e-12);
    assert_eq!(cover.entities(&[w.v]).len(), 6);
}

#[test]
fn deck_pair_normal_sign_convention_is_enforced() {
    // n_B must equal -g(n_A). A wedge whose B normal points inward breaks it.
    let w = wedge(-1.0);
    let rep = validate(&w.topo, &w.model, None, &tol());
    assert!(rep.has(IssueCode::DeckNormalsNotOpposed), "{:?}", rep.issues);
    let _ = (w.ids, w.a);
}

// ---------------------------------------------------------------- 4. instancing

struct Pins {
    ids: SequentialIds,
    model: Model,
    inst: Instancing,
    fuel: EntityId,
    fuel2: EntityId,
    water: EntityId,
    v_fuel: EntityId,
    v_mod: EntityId,
    pin: EntityId,
}

fn pins() -> Pins {
    let mut ids = SequentialIds::new();
    let mut model = Model::new();
    let fuel = model.coloring.define_color(&mut ids, "fuel", ColorRole::Volume).unwrap();
    let fuel2 = model.coloring.define_color(&mut ids, "fuel2", ColorRole::Volume).unwrap();
    let water = model.coloring.define_color(&mut ids, "water", ColorRole::Volume).unwrap();
    let (v_fuel, v_mod) = (ids.alloc(EntityKind::TopoVolume), ids.alloc(EntityKind::TopoVolume));
    model.coloring.assign_volume(v_fuel, fuel).unwrap();
    model.coloring.assign_volume(v_mod, water).unwrap();
    let mut inst = Instancing::new();
    let pin = inst.add_prototype(&mut ids, "pin", [v_fuel, v_mod]).unwrap();
    Pins { ids, model, inst, fuel, fuel2, water, v_fuel, v_mod, pin }
}

fn recolor_fuel(from: EntityId, to: EntityId) -> Recolor {
    Recolor { by_volume: Default::default(), by_color: [(from, to)].into_iter().collect() }
}

#[test]
fn hex_lattice_of_pins_with_override_and_hole() {
    let mut s = pins();
    let pitch = 1.26;
    let geom = LatticeGeometry::Hex { rings: 2, layers: 1, pitch, layer_pitch: 1.0, orientation: HexOrientation::A1AlongX, origin: p(0.0, 0.0, 0.0) };
    let lat = s.inst.add_lattice(&mut s.ids, "pins", geom, Placed::of(Fill::Ref(s.pin))).unwrap();
    s.inst.set_slot(lat, SlotIndex::new(0, 0, 0), Placed { recolor: recolor_fuel(s.fuel, s.fuel2), ..Placed::of(Fill::Ref(s.pin)) }).unwrap();
    assert!(s.inst.set_slot(lat, SlotIndex::new(2, 0, 0), Placed::of(Fill::Empty)).is_err(), "outside the array");

    let flat = s.inst.flatten(lat, FlattenOptions::default()).unwrap();
    assert_eq!(flat.items.len(), 7);
    let order: Vec<(i32, i32)> = flat.items.iter().map(|i| { let st = i.path.0[0].slot.unwrap(); (st.row, st.col) }).collect();
    assert_eq!(order, vec![(-1, 0), (-1, 1), (0, -1), (0, 0), (0, 1), (1, -1), (1, 0)], "canonical (k, row, col) order");
    let centre = &flat.items[3];
    assert!(near(centre.xf.apply_point(p(0.0, 0.0, 0.0)), p(0.0, 0.0, 0.0)));
    for (i, it) in flat.items.iter().enumerate() {
        let c = it.xf.apply_point(p(0.0, 0.0, 0.0));
        if i != 3 {
            assert!(((c - p(0.0, 0.0, 0.0)).norm() - pitch).abs() < 1e-12, "six neighbours at one pitch");
        }
        let (want_fuel, want_mod) = if i == 3 { (s.fuel2, s.water) } else { (s.fuel, s.water) };
        assert_eq!(it.volume_color(&s.model.coloring, s.v_fuel), Some(want_fuel));
        assert_eq!(it.volume_color(&s.model.coloring, s.v_mod), Some(want_mod));
    }

    // sparse hole
    s.inst.set_slot(lat, SlotIndex::new(1, 0, 0), Placed::of(Fill::Empty)).unwrap();
    assert_eq!(s.inst.flatten(lat, FlattenOptions::default()).unwrap().items.len(), 6);

    // other orientation: a2 along +y
    let geom30 = LatticeGeometry::Hex { rings: 2, layers: 1, pitch, layer_pitch: 1.0, orientation: HexOrientation::A1At30, origin: p(0.0, 0.0, 0.0) };
    let lat30 = s.inst.add_lattice(&mut s.ids, "pins30", geom30, Placed::of(Fill::Ref(s.pin))).unwrap();
    let c = s.inst.lattices[&lat30].slot_center(SlotIndex::new(0, 1, 0));
    assert!(c.x.abs() < 1e-12 && (c.y - pitch).abs() < 1e-12);
    // the lattice is also a plain group action: one translation placement per slot
    assert_eq!(s.inst.lattices[&lat].placements().unwrap().len(), 7);
}

fn core(s: &mut Pins, reverse_fill: bool) -> (EntityId, EntityId) {
    let hex = LatticeGeometry::Hex { rings: 2, layers: 1, pitch: 1.26, layer_pitch: 1.0, orientation: HexOrientation::A1AlongX, origin: p(0.0, 0.0, 0.0) };
    let asm = s.inst.add_lattice(&mut s.ids, "assembly", hex, Placed::of(Fill::Ref(s.pin))).unwrap();
    s.inst.set_slot(asm, SlotIndex::new(0, 0, 0), Placed { recolor: recolor_fuel(s.fuel, s.fuel2), ..Placed::of(Fill::Ref(s.pin)) }).unwrap();
    let cart = LatticeGeometry::Cartesian { dims: [2, 2, 1], pitch: [10.0, 10.0, 1.0], origin: p(0.0, 0.0, 0.0) };
    let core = s.inst.add_lattice(&mut s.ids, "core", cart, Placed::of(Fill::Ref(asm))).unwrap();
    let fuel3 = s.model.coloring.define_color(&mut s.ids, "fuel3", ColorRole::Volume).unwrap();
    let rot = Axis::new(p(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap().rotation(std::f64::consts::FRAC_PI_2);
    let slots = [
        (SlotIndex::new(1, 0, 0), Placed { recolor: recolor_fuel(s.fuel2, fuel3), ..Placed::of(Fill::Ref(asm)) }),
        (SlotIndex::new(0, 1, 0), Placed { xf: rot, ..Placed::of(Fill::Ref(asm)) }),
    ];
    if reverse_fill {
        for (k, v) in slots.into_iter().rev() {
            s.inst.set_slot(core, k, v).unwrap();
        }
    } else {
        for (k, v) in slots {
            s.inst.set_slot(core, k, v).unwrap();
        }
    }
    (core, fuel3)
}

#[test]
fn nested_lattice_flatten_is_deterministic_and_recolors_compose_inner_first() {
    let mut s1 = pins();
    let (core1, fuel3) = core(&mut s1, false);
    let mut s2 = pins();
    let (core2, _) = core(&mut s2, true);
    let f1 = s1.inst.flatten(core1, FlattenOptions::default()).unwrap();
    assert_eq!(f1.items.len(), 28);
    assert_eq!(f1.fingerprint(), s1.inst.flatten(core1, FlattenOptions::default()).unwrap().fingerprint(), "run twice");
    assert_eq!(f1.fingerprint(), s2.inst.flatten(core2, FlattenOptions::default()).unwrap().fingerprint(), "slot insertion order is irrelevant");
    let mut paths: Vec<_> = f1.items.iter().map(|i| i.path.clone()).collect();
    paths.sort();
    paths.dedup();
    assert_eq!(paths.len(), 28, "paths are unique");
    assert!(f1.items.iter().all(|i| i.path.0.len() == 2));

    // Assembly in core slot (col 1, row 0): outer maps fuel2 -> fuel3 AFTER the inner fuel -> fuel2.
    for it in &f1.items {
        let outer = it.path.0[0].slot.unwrap();
        let inner = it.path.0[1].slot.unwrap();
        let c = it.volume_color(&s1.model.coloring, s1.v_fuel).unwrap();
        let centre = inner == SlotIndex::new(0, 0, 0);
        let want = match (outer == SlotIndex::new(1, 0, 0), centre) {
            (true, true) => fuel3,
            (_, true) => s1.fuel2,
            _ => s1.fuel,
        };
        assert_eq!(c, want);
    }
    // improper/proper bookkeeping and the rotated assembly
    assert!(f1.items.iter().all(|i| !i.improper));
    let rotated: Vec<_> = f1.items.iter().filter(|i| i.path.0[0].slot == Some(SlotIndex::new(0, 1, 0))).collect();
    assert_eq!(rotated.len(), 7);
    assert!(near(rotated[6].xf.apply_point(p(0.0, 0.0, 0.0)), p(0.0, 10.0, 0.0) + rotate_z90(p(1.26 * 0.5, 1.26 * 0.75f64.sqrt(), 0.0)).sub_origin()));

    // depth-limited flatten stops at the assemblies
    let shallow = s1.inst.flatten(core1, FlattenOptions { depth: Some(1) }).unwrap();
    assert_eq!(shallow.items.len(), 4);
    assert!(shallow.items.iter().all(|i| s1.inst.lattices.contains_key(&i.target)));
}

trait SubOrigin {
    fn sub_origin(self) -> Vector3;
}
impl SubOrigin for Point3 {
    fn sub_origin(self) -> Vector3 {
        self - p(0.0, 0.0, 0.0)
    }
}
fn rotate_z90(q: Point3) -> Point3 {
    p(-q.y, q.x, q.z)
}

#[test]
fn instancing_cycles_and_dangling_refs_are_errors() {
    let mut s = pins();
    let (c, _) = core(&mut s, false);
    assert!(s.inst.check_references().is_ok());
    s.inst.set_slot(c, SlotIndex::new(0, 0, 0), Placed::of(Fill::Ref(c))).unwrap();
    assert!(matches!(s.inst.flatten(c, FlattenOptions::default()), Err(sm_model_api::ModelError::Cycle(_))));
    assert!(matches!(s.inst.check_references(), Err(sm_model_api::ModelError::Cycle(_))));
    let ghost = s.ids.alloc(EntityKind::Instance);
    assert!(s.inst.add_instance(&mut s.ids, "x", Placed::of(Fill::Ref(ghost))).is_err());
}

#[test]
fn flatten_composes_with_symmetry_group_action() {
    // A mirror-reduced lattice: same machinery unfolds it. Improper copies are flagged.
    let mut s = pins();
    let (c, _) = core(&mut s, false);
    let flat = s.inst.flatten(c, FlattenOptions::default()).unwrap();
    let m = new_generator(&mut s.ids, DeckOp::Mirror { plane: Plane::new(p(-5.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap() }).unwrap();
    let g = DeckGroup::generate(&[m]).unwrap();
    let both = flat.apply_group(&g).unwrap();
    assert_eq!(both.items.len(), 56);
    assert_eq!(both.items.iter().filter(|i| i.improper).count(), 28);
    assert!(both.items[28].element.as_ref().is_some_and(|e: &DeckElement| e.point == 1));
    assert_eq!(both.fingerprint(), flat.apply_group(&g).unwrap().fingerprint());
}

// ---------------------------------------------------------------- 5. decomposition

#[test]
fn decomposition_into_two_sweep_pieces_preserves_coloring() {
    let mut ids = SequentialIds::new();
    let (topo, vols) = MockTopology::from_boxes(&mut ids, &[bx([0.0; 3], [1.0, 1.0, 2.0])]);
    let mut model = color_boxes(&mut ids, &vols, &[0]);
    model.boundaries.tag_response(&topo.boundary_faces()).unwrap();
    let before = model.coloring.clone();
    let mut dec = Decomposition::identity(&topo, &mut ids);
    assert_eq!(dec.pieces().count(), 1);
    let parent = dec.pieces().next().unwrap().id;

    let bottom = topo.find_face(p(0.5, 0.5, 0.0), 1e-9).unwrap();
    let top = topo.find_face(p(0.5, 0.5, 2.0), 1e-9).unwrap();
    let sides: Vec<EntityId> = topo.boundary_faces().into_iter().filter(|f| *f != top && *f != bottom).collect();
    let whole = |f: EntityId| PieceFace::Model { face: f, whole: true };
    let cut = dec.new_cut("mid", CutGeometry::Plane(Plane::new(p(0.0, 0.0, 1.0), Vector3::new(0.0, 0.0, 1.0)).unwrap()));
    let side_faces: Vec<PieceFace> = sides.iter().map(|&f| whole(f)).collect();
    let a = PieceSpec {
        faces: [vec![whole(bottom)], side_faces.clone()].concat(),
        shape: PieceShape::Sweep(SweepRoles { sources: vec![whole(bottom)], targets: vec![PieceFace::Virtual(cut)] }),
        measure: Some(1.0),
    };
    let b = PieceSpec {
        faces: [vec![whole(top)], side_faces].concat(),
        shape: PieceShape::Sweep(SweepRoles { sources: vec![PieceFace::Virtual(cut)], targets: vec![whole(top)] }),
        measure: Some(1.0),
    };
    let [pa, pb] = dec.split(parent, cut, a, b, &mut ids).unwrap();

    assert_eq!(dec.pieces_of_volume(vols[0]), vec![pa, pb]);
    assert_eq!(dec.volumes_of_piece(pa), vec![vols[0]]);
    assert_eq!(dec.pieces_of_face(sides[0]), vec![pa, pb], "side walls are shared (partial) by both pieces");
    assert_eq!(dec.pieces_of_face(bottom), vec![pa]);
    assert_eq!(dec.pieces_of_face(top), vec![pb]);
    assert!(dec.faces_of_piece(pa).iter().all(|&(f, whole)| whole == (f == bottom)));
    assert_eq!(dec.neighbors(pa), vec![pb]);
    assert_eq!(dec.sweep_pieces().len(), 2);
    let fuel = model.coloring.color_by_name(ColorRole::Volume, "fuel").unwrap();
    assert_eq!(dec.piece_volume_color(&model.coloring, pa), Some(fuel));
    assert_eq!(dec.piece_volume_color(&model.coloring, pb), Some(fuel));
    assert_eq!(model.coloring, before, "decomposition never touches coloring");
    let rep = validate(&topo, &model, Some(&dec), &tol());
    assert!(rep.is_ok(), "{:?}", rep.issues);

    // reversible: composite restores a single piece with whole faces; undo restores the parent.
    let merged = dec.composite(&[pa, pb], &model.coloring, &mut ids).unwrap();
    assert_eq!(dec.pieces().count(), 1);
    assert!(dec.faces_of_piece(merged).iter().all(|&(_, w)| w));
    assert!(dec.cut(cut).is_none());
    assert!(dec.undo());
    assert_eq!(dec.pieces_of_volume(vols[0]), vec![pa, pb]);
    assert!(dec.undo());
    assert_eq!(dec.pieces().next().unwrap().id, parent);
    assert!(!dec.undo() || dec.depth() == 0);
}

#[test]
fn decomposition_refuses_to_mix_colors_and_validator_catches_recoloring() {
    let mut t = two_material();
    let mut dec = Decomposition::identity(&t.topo, &mut t.ids);
    let ps: Vec<EntityId> = dec.pieces().map(|p| p.id).collect();
    assert!(dec.composite(&ps, &t.model.coloring, &mut t.ids).is_err(), "fuel+clad composite refused");
    // same-color composite is allowed ...
    t.model.coloring.assign_volume(t.vols[1], t.fuel).unwrap();
    let merged = dec.composite(&ps, &t.model.coloring, &mut t.ids).unwrap();
    assert_eq!(dec.volumes_of_piece(merged).len(), 2);
    assert!(validate(&t.topo, &t.model, Some(&dec), &tol()).is_ok());
    // ... until someone recolors a member: the validator flags the mixed composite.
    t.model.coloring.assign_volume(t.vols[1], t.clad).unwrap();
    assert!(validate(&t.topo, &t.model, Some(&dec), &tol()).has(IssueCode::CompositeMixedColors));
}

#[test]
fn decomposition_validator_checks_measures_and_cut_sharing() {
    let mut t = two_material();
    let mut dec = Decomposition::identity(&t.topo, &mut t.ids);
    assert!(validate(&t.topo, &t.model, Some(&dec), &tol()).is_ok());
    let first = dec.pieces().next().unwrap().id;
    let cut = dec.new_cut("c", CutGeometry::Unspecified);
    let faces = dec.piece(first).unwrap().faces.clone();
    let sp = |m: f64| PieceSpec { faces: faces.clone(), shape: PieceShape::Free, measure: Some(m) };
    // the volume has measure 1; the pieces claim 0.3 + 0.3
    let [pa, _] = dec.split(first, cut, sp(0.3), sp(0.3), &mut t.ids).unwrap();
    let rep = validate(&t.topo, &t.model, Some(&dec), &tol());
    assert!(rep.has(IssueCode::MeasureMismatch), "{:?}", rep.issues);
    assert!(!rep.has(IssueCode::CutNotShared));
    assert!(dec.undo());
    let [pa2, pb2] = dec.split(first, cut, sp(0.5), sp(0.5), &mut t.ids).unwrap();
    assert!(validate(&t.topo, &t.model, Some(&dec), &tol()).is_ok());
    // A sweep role that names a face the piece does not have is caught.
    let foreign = PieceFace::Model { face: t.topo.boundary_faces()[0], whole: true };
    let bad = PieceShape::Sweep(SweepRoles { sources: vec![PieceFace::Virtual(sm_model_api::decomposition::CutId(99))], targets: vec![foreign] });
    dec.set_shape(pb2, bad).unwrap();
    assert!(validate(&t.topo, &t.model, Some(&dec), &tol()).has(IssueCode::SweepRoleNotInPiece));
    let _ = pa;
    let _ = pa2;
}

// ---------------------------------------------------------------- determinism

fn pipeline_hash() -> u64 {
    use std::hash::{Hash, Hasher};
    let mut ids = SequentialIds::new();
    let (topo, vols) = MockTopology::from_boxes(&mut ids, &d2h_boxes());
    let model = color_boxes(&mut ids, &vols, &[1, 0, 1]);
    let rep = detect(&topo, &model.coloring, None, &DetectParams::default()).unwrap();
    let mut h = std::collections::hash_map::DefaultHasher::new(); // fixed keys, deterministic
    format!("{:?}", rep.candidates).hash(&mut h);
    let mut s = pins();
    let (c, _) = core(&mut s, false);
    s.inst.flatten(c, FlattenOptions::default()).unwrap().fingerprint().hash(&mut h);
    h.finish()
}

/// No rayon in this crate, so the 1-thread/N-thread comparison is vacuous by construction; we still
/// run twice and across spawned threads and compare hashes.
#[test]
fn determinism_run_twice_and_across_threads() {
    let a = pipeline_hash();
    let b = pipeline_hash();
    assert_eq!(a, b);
    let handles: Vec<_> = (0..4).map(|_| std::thread::spawn(pipeline_hash)).collect();
    for h in handles {
        assert_eq!(h.join().unwrap(), a);
    }
}
