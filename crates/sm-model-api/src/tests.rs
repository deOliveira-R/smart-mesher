//! Scenario tests against the mock topology. Numerical expectations are analytic.

use crate::action::{compose, enumerate_elements, inverse, rotation, EnumerateOptions};
use crate::hash::Fnv64;
use crate::instancing::{Fill, HexOrientation, LatticeGeometry, Target};
use crate::mock::TableMock;
use crate::symmetry::{GeneratorSpec, RejectStage, SymmetryOrigin, UnfoldedModel};
use crate::*;
use sm_core::{EntityId, EntityKind, Point3, Transform, Vector3};

type R = Result<(), ModelError>;

fn vol(n: u64) -> EntityId {
    TableMock::vid(n)
}
fn vsel(ns: &[u64]) -> Selection<Volume> {
    Selection::geometry(ns.iter().map(|n| vol(*n))).unwrap()
}
fn tol() -> Tolerance {
    Tolerance::default()
}
fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
}

/// Four boxes in a row along x: M2 | M1 | M1 | M2, already split at x = 0.
fn row() -> (TableMock, Model, EntityId, EntityId) {
    let b = |x0: f64, x1: f64| ([x0, 0.0, 0.0], [x1, 2.0, 2.0]);
    let topo = TableMock::from_boxes(&[b(-3.0, -1.0), b(-1.0, 0.0), b(0.0, 1.0), b(1.0, 3.0)]);
    let mut m = Model::new();
    let m1 = m.define_volume_color("material 1", vsel(&[2, 3])).unwrap();
    let m2 = m.define_volume_color("material 2", vsel(&[1, 4])).unwrap();
    (topo, m, m1, m2)
}

#[test]
fn coloring_two_materials_with_boundary_colors() -> R {
    let (topo, mut m, m1, m2) = row();
    assert_eq!(m.volume_color_of(vol(2)), Some(m1));
    assert_eq!(m.volume_color_of(vol(4)), Some(m2));
    // Exclusive: a volume cannot get a second color.
    assert!(matches!(m.define_volume_color("dup", vsel(&[1])), Err(ModelError::Conflict { .. })));
    assert!(matches!(m.define_volume_color("material 1", vsel(&[])), Err(ModelError::DuplicateName(_))));
    // Boundary colors: the two x-ends vs everything else on the outside.
    let ends = Selection::geometry(
        topo.faces_on_axis_plane(0, -3.0).into_iter().chain(topo.faces_on_axis_plane(0, 3.0)),
    )?;
    let ends_id = m.define_face_color("ends", ends.clone())?;
    let rest = Selector::<Face>::Diff(Box::new(Selector::DomainBoundary), Box::new(Selector::Color(ends_id))).resolve(&m, &topo)?;
    assert_eq!(rest.len(), 16);
    m.define_face_color("sides", rest)?;
    assert_eq!(m.face_color_of(ends.ids()[0]), Some(ends_id));
    // Interior faces carry no boundary color; before kinds are set every boundary face is flagged.
    let rep = m.validate(&topo, &tol());
    assert!(rep.has(IssueCode::BoundaryFaceWithoutKind));
    assert!(!rep.has(IssueCode::VolumeUncolored));
    // An abstract group (block/assembly) is just a named set.
    let blk = m.define_group("block A", GroupMembers::Volumes(vsel(&[2, 3])))?;
    let asm = m.define_group("assembly", GroupMembers::Groups(vec![blk]))?;
    assert_eq!(Selector::<Volume>::Named(asm).resolve(&m, &topo)?.ids(), vsel(&[2, 3]).ids());
    Ok(())
}

#[test]
fn uncolored_volume_is_an_error() {
    let (topo, _, _, _) = row();
    let mut m = Model::new();
    m.define_volume_color("only", vsel(&[1, 2, 3])).unwrap();
    let rep = m.validate(&topo, &tol());
    let missing: Vec<_> = rep.with_code(IssueCode::VolumeUncolored).flat_map(|i| i.entities.clone()).collect();
    assert_eq!(missing, vec![vol(4)]);
}

#[test]
fn mirror_cut_keep_chamber_then_unfold_and_refold() -> R {
    let (topo, mut m, m1, _m2) = row();
    let plane = Plane::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0))?;
    let rep = m.cut_keep_chamber(&topo, &CutSpec { generator: Generator::Mirror { plane }, keep: KeepSide::Positive, wedge_start: None }, &tol())?;
    assert_eq!(rep.kept, vec![vol(3), vol(4)]);
    assert_eq!(rep.tagged.len(), 1);
    let cut_face = rep.tagged[0];
    assert!(matches!(m.kind_of_face(cut_face), Some(BoundaryKind::Deck { side: DeckSide::SelfPaired, .. })));
    // The stored normal pointed into the kept volume; the outward normal is -x.
    let n = m.outward_normal(cut_face, &topo).unwrap();
    assert!(near(n.x, -1.0));
    // The rest of the outside is a response boundary; then the model validates.
    let rest = Selector::<Face>::Unassigned.resolve(&m, &topo)?;
    assert_eq!(rest.len(), 9); // x = 3 end + 8 side faces of the two kept boxes
    m.define_response("vacuum", rest)?;
    let report = m.validate(&topo, &tol());
    assert!(report.is_ok(), "{:?}", report.issues);
    assert_eq!(m.volume_color_of(vol(3)), Some(m1));

    let u = m.unfold(&topo, &UnfoldOptions::default())?;
    assert_eq!(u.elements.len(), 2);
    assert!(!u.elements[0].orientation_reversed && u.elements[1].orientation_reversed);
    assert!(near(u.elements[1].transform.det(), -1.0));
    // mirror o mirror = identity
    let mm = compose(&u.elements[1].transform, &u.elements[1].transform);
    assert!(crate::action::max_diff(&mm, &Transform::IDENTITY) < 1e-12);
    assert_eq!(u.volumes.len(), 4);
    // 11 faces in the domain, doubled, minus the merged mirror-plane face.
    assert_eq!(u.faces.len(), 21);
    let merged = u.faces.iter().find(|f| f.source == cut_face && f.copy == 0).unwrap();
    let v3_copy1 = u.volumes.iter().find(|v| v.source == vol(3) && v.copy == 1).unwrap().id;
    assert_eq!(merged.left, vol(3));
    assert_eq!(merged.right, Some(v3_copy1));
    assert!(merged.kind_set.is_none(), "merged deck face is interior now");
    assert!(!u.faces.iter().any(|f| f.source == cut_face && f.copy == 1));
    // Reflected copy: every surviving face has its loop reversed (the merged face is copy 0, stored inward).
    assert!(u.faces.iter().filter(|f| f.copy == 1).all(|f| f.loop_reversed));
    // Colors survive the copy.
    assert!(u.volumes.iter().filter(|v| v.source == vol(3)).all(|v| v.color == Some(m1)));
    // Reversible: derived ids map back; copy 0 is the fundamental domain.
    assert_eq!(u.source_of(v3_copy1), Some((vol(3), 1)));
    assert_eq!(u.refold(), u.fundamental_domain());
    assert_eq!(u.refold(), vec![vol(3), vol(4)]);
    Ok(())
}

#[test]
fn cut_failure_leaves_model_unchanged() {
    let (topo, mut m, _, _) = row();
    // A plane at x = 0.5 cuts through volume 3: the straddler is not pre-split, so no cut face exists.
    let plane = Plane::new(Point3::new(0.5, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap();
    let err = m.cut_keep_chamber(&topo, &CutSpec { generator: Generator::Mirror { plane }, keep: KeepSide::Positive, wedge_start: None }, &tol());
    assert!(matches!(err, Err(ModelError::NoCutFaces { .. })));
    assert!(m.domain().is_none() && m.symmetry().is_none());
}

#[test]
fn translation_pair_declared_checked_and_unfolded() -> R {
    let topo = TableMock::from_boxes(&[([0.0; 3], [1.0; 3])]);
    let mut m = Model::new();
    m.define_volume_color("fuel", vsel(&[1]))?;
    let a = topo.faces_on_axis_plane(0, 0.0)[0];
    let b = topo.faces_on_axis_plane(0, 1.0)[0];
    let g = Generator::Translation { origin: Point3::new(0.0, 0.0, 0.0), vector: Vector3::new(1.0, 0.0, 0.0) };
    let p = m.declare_deck(g, vec![(a, b)])?;
    let rest = Selector::<Face>::Unassigned.resolve(&m, &topo)?;
    m.define_response("vacuum", rest)?;
    assert!(m.validate(&topo, &tol()).is_ok());
    assert_eq!(m.pairing(p).unwrap().kind(), DeckKind::Translation);

    // Unfold needs a repeat count.
    m.declare_symmetry(vec![GeneratorSpec { generator: g, pairing: Some(p), repeat: None }], SymmetryOrigin::Imported)?;
    assert!(matches!(m.unfold(&topo, &UnfoldOptions::default()), Err(ModelError::UnboundedGenerator { index: 0 })));
    m.declare_symmetry(vec![GeneratorSpec { generator: g, pairing: Some(p), repeat: Some(3) }], SymmetryOrigin::Imported)?;
    let u = m.unfold(&topo, &UnfoldOptions::default())?;
    assert_eq!(u.elements.len(), 3);
    assert!(near(u.elements[2].transform.t.x, 2.0));
    // B@0 = A@1 and B@1 = A@2 are merged; B@2 and A@0 stay as deck faces.
    assert_eq!(u.faces.len(), 18 - 2);
    let a1 = u.faces.iter().find(|f| f.source == a && f.copy == 1).unwrap();
    assert!(a1.right.is_some() && a1.kind_set.is_none());
    let a0 = u.faces.iter().find(|f| f.source == a && f.copy == 0).unwrap();
    assert!(a0.kind_set.is_some() && a0.right.is_none(), "A@0 stays a deck face: the translation chain is open");
    Ok(())
}

#[test]
fn wrong_translation_is_caught_by_the_validator() -> R {
    let topo = TableMock::from_boxes(&[([0.0; 3], [1.0; 3])]);
    let mut m = Model::new();
    m.define_volume_color("fuel", vsel(&[1]))?;
    let a = topo.faces_on_axis_plane(0, 0.0)[0];
    let b = topo.faces_on_axis_plane(0, 1.0)[0];
    let g = Generator::Translation { origin: Point3::new(0.0, 0.0, 0.0), vector: Vector3::new(2.0, 0.0, 0.0) };
    m.declare_deck(g, vec![(a, b)])?;
    let rest = Selector::<Face>::Unassigned.resolve(&m, &topo)?;
    m.define_response("vacuum", rest)?;
    assert!(m.validate(&topo, &tol()).has(IssueCode::DeckPairGeometry));
    // Same-direction normals (A paired with itself under a translation) are also rejected.
    Ok(())
}

#[test]
fn translation_slab_cut_pairs_faces_automatically() -> R {
    let b = |x0: f64, x1: f64| ([x0, 0.0, 0.0], [x1, 1.0, 1.0]);
    let topo = TableMock::from_boxes(&[b(-1.0, 0.0), b(0.0, 1.0), b(1.0, 2.0), b(2.0, 3.0)]);
    let mut m = Model::new();
    m.define_volume_color("lattice cell", vsel(&[1, 2, 3, 4]))?;
    let g = Generator::Translation { origin: Point3::new(0.0, 0.0, 0.0), vector: Vector3::new(2.0, 0.0, 0.0) };
    let rep = m.cut_keep_chamber(&topo, &CutSpec { generator: g, keep: KeepSide::Positive, wedge_start: None }, &tol())?;
    assert_eq!(rep.kept, vec![vol(2), vol(3)]);
    let pairing = m.pairing(rep.pairing).unwrap();
    assert_eq!(pairing.pairs.len(), 1);
    assert!(near(topo.face_properties(pairing.pairs[0].0).unwrap().centroid.x, 0.0));
    assert!(near(topo.face_properties(pairing.pairs[0].1).unwrap().centroid.x, 2.0));
    let rest = Selector::<Face>::Unassigned.resolve(&m, &topo)?;
    m.define_response("vacuum", rest)?;
    assert!(m.validate(&topo, &tol()).is_ok());
    Ok(())
}

/// Six wedges around the z axis; wedge k spans [60k, 60(k+1)] degrees, unit radius, outer face at r = 2.
fn six_wedges() -> TableMock {
    let mut t = TableMock::new();
    let rad = |deg: f64| deg.to_radians();
    for k in 0..6u64 {
        let th = rad(60.0 * k as f64 + 30.0);
        t.add_volume(
            TableMock::vid(k + 1),
            MassProps { volume: 1.0, centroid: Point3::new(th.cos(), th.sin(), 0.5), second_moment: [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.02]] },
        );
        // Outer face of wedge k.
        t.add_face(
            TableMock::fid(10 + k),
            FaceSides { left: Some(TableMock::vid(k + 1)), right: None },
            FaceProps { centroid: Point3::new(2.0 * th.cos(), 2.0 * th.sin(), 0.5), normal: Vector3::new(th.cos(), th.sin(), 0.0), area: 1.0 },
        );
        // Plane face at angle 60k, between wedge k-1 and wedge k.
        let ph = rad(60.0 * k as f64);
        let e_phi = Vector3::new(-ph.sin(), ph.cos(), 0.0);
        let (prev, this) = (TableMock::vid((k + 5) % 6 + 1), TableMock::vid(k + 1));
        let (left, right, normal) = if k == 0 { (this, prev, -e_phi) } else { (prev, this, e_phi) };
        t.add_face(
            TableMock::fid(k + 1),
            FaceSides { left: Some(left), right: Some(right) },
            FaceProps { centroid: Point3::new(ph.cos(), ph.sin(), 0.5), normal, area: 1.0 },
        );
    }
    t
}

#[test]
fn one_sixth_rotation_cut_unfold_is_analytic() -> R {
    let topo = six_wedges();
    let mut m = Model::new();
    let mat = m.define_volume_color("assembly", vsel(&[1, 2, 3, 4, 5, 6]))?;
    let g = Generator::Rotation { origin: Point3::new(0.0, 0.0, 0.0), axis: Vector3::new(0.0, 0.0, 1.0), order: 6 };
    let rep = m.cut_keep_chamber(&topo, &CutSpec { generator: g, keep: KeepSide::Positive, wedge_start: None }, &tol())?;
    assert_eq!(rep.kept, vec![vol(1)]);
    let p = m.pairing(rep.pairing).unwrap();
    assert_eq!(p.pairs, vec![(TableMock::fid(1), TableMock::fid(2))]);
    let rest = Selector::<Face>::Unassigned.resolve(&m, &topo)?;
    assert_eq!(rest.len(), 1);
    m.define_response("vacuum", rest)?;
    let report = m.validate(&topo, &tol());
    assert!(report.is_ok(), "{:?}", report.issues);

    let u = m.unfold(&topo, &UnfoldOptions::default())?;
    assert_eq!(u.elements.len(), 6);
    assert!(u.elements.iter().all(|e| !e.orientation_reversed));
    // Element k is the rotation by 60k degrees (generated by repeated right-multiplication).
    let r60 = rotation(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0), 60f64.to_radians());
    let mut acc = Transform::IDENTITY;
    for e in &u.elements {
        assert!(crate::action::max_diff(&e.transform, &acc) < 1e-9);
        acc = compose(&acc, &r60);
    }
    assert!(crate::action::max_diff(&acc, &Transform::IDENTITY) < 1e-9, "six 60 degree turns close");
    assert_eq!(u.volumes.len(), 6);
    // 3 faces x 6 copies, 6 B faces glued onto A faces.
    assert_eq!(u.faces.len(), 12);
    let a_faces: Vec<_> = u.faces.iter().filter(|f| f.source == TableMock::fid(1)).collect();
    assert_eq!(a_faces.len(), 6);
    assert!(a_faces.iter().all(|f| f.right.is_some() && f.kind_set.is_none()), "full turn closes: no deck face remains");
    // A@0 is glued to the wedge of copy 5 (its B face is the A face of copy 0).
    let w5 = u.volumes.iter().find(|v| v.copy == 5).unwrap().id;
    assert_eq!(a_faces[0].right, Some(w5));
    assert!(u.volumes.iter().all(|v| v.color == Some(mat)));
    // Outer faces remain response boundaries in every copy.
    assert_eq!(u.faces.iter().filter(|f| f.kind_set.is_some()).count(), 6);
    Ok(())
}

#[test]
fn group_enumeration_and_inverse_are_analytic() {
    let r = rotation(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_2);
    let inv = inverse(&r).unwrap();
    assert!(crate::action::max_diff(&compose(&r, &inv), &Transform::IDENTITY) < 1e-12);
    // Rotation by 90 deg about (1,0,0) maps the origin to (1,-1,0).
    let p = r.apply_point(Point3::new(0.0, 0.0, 0.0));
    assert!(near(p.x, 1.0) && near(p.y, -1.0));
    // Two perpendicular mirrors generate the 4-element Klein group.
    let mx = Generator::Mirror { plane: Plane::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap() }.transform();
    let my = Generator::Mirror { plane: Plane::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)).unwrap() }.transform();
    let els = enumerate_elements(&[mx, my], &[], &EnumerateOptions::default()).unwrap();
    assert_eq!(els.len(), 4);
    assert_eq!(els.iter().filter(|e| e.orientation_reversed).count(), 2);
    let cap = enumerate_elements(&[mx, my], &[], &EnumerateOptions { max_elements: 3 });
    assert!(matches!(cap, Err(ModelError::GroupTooLarge { limit: 3 })));
    let singular = Transform { m: [[1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]], t: Vector3::new(0.0, 0.0, 0.0) };
    assert!(matches!(inverse(&singular), Err(ModelError::SingularTransform)));
}

#[test]
fn symmetry_detection_finds_verified_candidates_and_rejects_others() -> R {
    let (topo, mut m, _, _) = row();
    let res = m.detect_symmetry(&topo, &DetectOptions::default());
    let bary = res.barycentre.unwrap();
    assert!(bary.x.abs() < 1e-9 && near(bary.y, 1.0) && near(bary.z, 1.0));
    assert_eq!(res.invariants.len(), 2);
    let mirrors: Vec<_> = res.candidates.iter().filter_map(|c| match c.generator {
        Generator::Mirror { plane } => Some(plane),
        _ => None,
    }).collect();
    // x = 0, y = 1, z = 1 planes (the row is a symmetric 2x2 section).
    assert_eq!(mirrors.len(), 3);
    assert!(mirrors.iter().any(|p| near(p.normal.x.abs(), 1.0) && p.point.x.abs() < 1e-9));
    let rot_x = res.candidates.iter().find_map(|c| match c.generator {
        Generator::Rotation { axis, order, .. } if near(axis.x.abs(), 1.0) => Some(order),
        _ => None,
    });
    assert_eq!(rot_x, Some(4), "square section: 4-fold about the row axis");
    assert!(res.candidates.iter().all(|c| matches!(c.verification, Verification::Verified { .. })));
    // Detection is deterministic.
    assert_eq!(res, m.detect_symmetry(&topo, &DetectOptions::default()));

    // Adopt the x mirror, then cut with it.
    let cand = res.candidates.iter().find(|c| matches!(c.generator, Generator::Mirror { plane } if near(plane.normal.x.abs(), 1.0))).unwrap();
    m.declare_detected(cand, false)?;
    let Generator::Mirror { plane } = cand.generator else { unreachable!() };
    let rep = m.cut_keep_chamber(&topo, &CutSpec { generator: Generator::Mirror { plane }, keep: KeepSide::Positive, wedge_start: None }, &tol())?;
    assert_eq!(m.symmetry().unwrap().generators.len(), 1, "cut fills the detected generator's pairing");
    assert_eq!(m.symmetry().unwrap().generators[0].pairing, Some(rep.pairing));

    // Break the symmetry by recoloring: material 2 only on the left box.
    let mut broken = Model::new();
    broken.define_volume_color("a", vsel(&[1]))?;
    broken.define_volume_color("b", vsel(&[2, 3, 4]))?;
    let opts = DetectOptions { include_rejected: true, ..DetectOptions::default() };
    let res = broken.detect_symmetry(&topo, &opts);
    let xm = res.candidates.iter().find(|c| matches!(c.generator, Generator::Mirror { plane } if near(plane.normal.x.abs(), 1.0))).unwrap();
    assert!(matches!(xm.verification, Verification::Rejected { stage: RejectStage::ColorInvariants, .. }));
    assert!(matches!(broken.declare_detected(xm, false), Err(ModelError::NotVerified)));
    // y and z mirrors still hold.
    assert!(res.candidates.iter().any(|c| matches!((c.generator, c.verification), (Generator::Mirror { plane }, Verification::Verified { .. }) if near(plane.normal.y.abs(), 1.0))));
    Ok(())
}

fn pin_model() -> Result<(Model, EntityId, EntityId, EntityId, EntityId), ModelError> {
    let b = |x0: f64| ([x0, 0.0, 0.0], [x0 + 1.0, 1.0, 1.0]);
    let _ = TableMock::from_boxes(&[b(0.0), b(1.0)]);
    let mut m = Model::new();
    let fuel = m.define_volume_color("fuel", vsel(&[1]))?;
    let clad = m.define_volume_color("clad", vsel(&[2]))?;
    let f2 = m.define_volume_color("fuel B", Selection::empty())?;
    let pin = m.instancing_mut().add_prototype("pin", vsel(&[1, 2]))?;
    Ok((m, fuel, clad, f2, pin))
}

#[test]
fn hex_lattice_of_pins_with_override_and_sparse_slot() -> R {
    let (mut m, fuel, clad, fuel_b, pin) = pin_model()?;
    let lat = m.instancing_mut().add_lattice(
        "core",
        LatticeGeometry::Hex { rings: 2, layers: 1, pitch: 1.0, dz: 0.0, orientation: HexOrientation::Pointy },
        Transform::IDENTITY,
    )?;
    let plain = Fill::new(Target::Prototype(pin));
    m.instancing_mut().set_default_fill(lat, Some(plain.clone()))?;
    m.instancing_mut().set_slot(lat, Slot::new(0, 0, 0), Some(plain.clone().with_override(fuel, fuel_b)?))?;
    m.instancing_mut().set_slot(lat, Slot::new(1, -1, 0), None)?; // void
    assert!(m.instancing_mut().set_slot(lat, Slot::new(2, 0, 0), None).is_err(), "outside the hex");
    let shift = Transform { t: Vector3::new(10.0, 0.0, 0.0), ..Transform::IDENTITY };
    let inst = m.instancing_mut().add_instance("core at x=10", Fill::new(Target::Lattice(lat)), shift)?;
    let flat = m.instancing().flatten(inst, 1 << 40)?;
    assert_eq!(flat.len(), 6);
    let centre = flat.iter().find(|f| f.placement.path.last() == Some(&PathStep::Slot { lattice: lat, slot: Slot::new(0, 0, 0) })).unwrap();
    assert_eq!(centre.effective_color(fuel), fuel_b);
    assert_eq!(centre.effective_color(clad), clad);
    assert!(near(centre.placement.transform.t.x, 10.0));
    let others = flat.iter().filter(|f| f.id != centre.id);
    assert!(others.clone().count() == 5 && others.clone().all(|f| f.effective_color(fuel) == fuel));
    // Slot (1,0) of a pointy lattice sits one pitch along x; (0,1) at (1/2, sqrt(3)/2).
    let at = |s: Slot| flat.iter().find(|f| f.placement.path.last() == Some(&PathStep::Slot { lattice: lat, slot: s })).unwrap().placement.transform.t;
    assert!(near(at(Slot::new(1, 0, 0)).x, 11.0) && near(at(Slot::new(1, 0, 0)).y, 0.0));
    assert!(near(at(Slot::new(0, 1, 0)).x, 10.5) && near(at(Slot::new(0, 1, 0)).y, 3f64.sqrt() / 2.0));
    // Lexicographic slot order; ids follow emission order.
    let slots: Vec<_> = flat.iter().filter_map(|f| match f.placement.path.last() { Some(PathStep::Slot { slot, .. }) => Some(*slot), _ => None }).collect();
    assert!(slots.windows(2).all(|w| w[0] < w[1]));
    assert_eq!(flat[0].id.serial(), 1 << 40);
    // Overrides refer to real colors.
    assert!(m.validate(&TableMock::from_boxes(&[([0.0; 3], [1.0; 3]), ([1.0, 0.0, 0.0], [2.0, 1.0, 1.0])]), &tol()).with_code(IssueCode::InstancingInvalid).next().is_none());
    Ok(())
}

fn nested_model() -> Result<(Model, EntityId), ModelError> {
    let (mut m, fuel, _, fuel_b, pin) = pin_model()?;
    let fuel_c = m.define_volume_color("fuel C", Selection::empty())?;
    let inner = m.instancing_mut().add_lattice(
        "assembly",
        LatticeGeometry::Hex { rings: 2, layers: 1, pitch: 1.0, dz: 0.0, orientation: HexOrientation::Flat },
        Transform::IDENTITY,
    )?;
    // Inner: fuel -> fuel B on every pin. Outer slot: fuel B -> fuel C. Nearest the leaf applies first.
    m.instancing_mut().set_default_fill(inner, Some(Fill::new(Target::Prototype(pin)).with_override(fuel, fuel_b)?))?;
    let outer = m.instancing_mut().add_lattice(
        "core",
        LatticeGeometry::Cartesian { nx: 2, ny: 2, nz: 1, pitch: Vector3::new(10.0, 10.0, 1.0) },
        Transform::IDENTITY,
    )?;
    for (i, j) in [(0, 0), (0, 1), (1, 0), (1, 1)] {
        let fill = Fill::new(Target::Lattice(inner));
        let fill = if (i, j) == (1, 1) { fill.with_override(fuel_b, fuel_c)? } else { fill };
        m.instancing_mut().set_slot(outer, Slot::new(i, j, 0), Some(fill))?;
    }
    // A mirrored copy of the whole core.
    let mirror = Generator::Mirror { plane: Plane::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0))? }.transform();
    m.instancing_mut().add_instance("core instance", Fill::new(Target::Lattice(outer)), Transform::IDENTITY)?;
    m.instancing_mut().add_instance("core mirrored", Fill::new(Target::Lattice(outer)), mirror)?;
    Ok((m, fuel_c))
}

fn flat_hash(m: &Model) -> Result<u64, ModelError> {
    let mut h = Fnv64::new();
    for f in m.instancing().flatten_all(1 << 40)? {
        h.id(f.id);
        h.id(f.prototype);
        h.transform(&f.placement.transform);
        h.u64(u64::from(f.placement.orientation_reversed));
        for s in &f.placement.path {
            h.str(&format!("{s:?}"));
        }
        for (a, b) in f.overrides.iter() {
            h.id(a);
            h.id(b);
        }
    }
    Ok(h.finish())
}

#[test]
fn nested_lattice_flatten_is_deterministic_with_paths_and_override_order() -> R {
    let (m, fuel_c) = nested_model()?;
    let flat = m.instancing().flatten_all(1 << 40)?;
    assert_eq!(flat.len(), 2 * 4 * 7);
    assert!(flat.iter().all(|f| f.placement.path.len() == 3));
    // Mirrored instance: improper placements.
    assert_eq!(flat.iter().filter(|f| f.placement.orientation_reversed).count(), 28);
    // Slot (1,1) of the outer lattice: fuel -> fuel B (inner) -> fuel C (outer).
    let fuel = m.volume_colors().find(|c| c.name == "fuel").unwrap().id;
    let outer_11 = flat.iter().filter(|f| matches!(f.placement.path[1], PathStep::Slot { slot, .. } if slot == Slot::new(1, 1, 0)));
    assert!(outer_11.clone().count() == 14 && outer_11.clone().all(|f| f.effective_color(fuel) == fuel_c));
    // The same model built twice flattens identically; so does a clone.
    let (m2, _) = nested_model()?;
    assert_eq!(flat_hash(&m)?, flat_hash(&m2)?);
    assert_eq!(flat_hash(&m)?, flat_hash(&m.clone())?);
    // Unique derived ids.
    let mut ids: Vec<_> = flat.iter().map(|f| f.id).collect();
    ids.dedup();
    assert_eq!(ids.len(), flat.len());
    Ok(())
}

#[test]
fn lattice_cycle_is_reported() -> R {
    let (mut m, _, _, _, pin) = pin_model()?;
    let a = m.instancing_mut().add_lattice("a", LatticeGeometry::Cartesian { nx: 1, ny: 1, nz: 1, pitch: Vector3::new(1.0, 1.0, 1.0) }, Transform::IDENTITY)?;
    m.instancing_mut().set_slot(a, Slot::new(0, 0, 0), Some(Fill::new(Target::Prototype(pin))))?;
    let b = m.instancing_mut().add_lattice("b", LatticeGeometry::Cartesian { nx: 1, ny: 1, nz: 1, pitch: Vector3::new(1.0, 1.0, 1.0) }, Transform::IDENTITY)?;
    m.instancing_mut().set_slot(b, Slot::new(0, 0, 0), Some(Fill::new(Target::Lattice(a))))?;
    m.instancing_mut().set_slot(a, Slot::new(0, 0, 0), Some(Fill::new(Target::Lattice(b))))?;
    m.instancing_mut().add_instance("root", Fill::new(Target::Lattice(a)), Transform::IDENTITY)?;
    assert!(matches!(m.instancing().flatten_all(1 << 40), Err(ModelError::Cycle(_))));
    let topo = TableMock::from_boxes(&[([0.0; 3], [1.0; 3]), ([1.0, 0.0, 0.0], [2.0, 1.0, 1.0])]);
    assert!(m.validate(&topo, &tol()).has(IssueCode::InstancingInvalid));
    Ok(())
}

fn sweep_model() -> Result<(TableMock, Model, EntityId), ModelError> {
    let topo = TableMock::from_boxes(&[([0.0, 0.0, 0.0], [1.0, 1.0, 2.0])]);
    let mut m = Model::new();
    let c = m.define_volume_color("fuel", vsel(&[1]))?;
    Ok((topo, m, c))
}

#[test]
fn decomposition_into_two_sweep_pieces_preserves_colors() -> R {
    let (topo, mut m, color) = sweep_model()?;
    let before = m.coloring_fingerprint();
    m.decompose(&topo)?;
    let whole = m.decomposition().unwrap().pieces().next().unwrap().id;
    let bottom = topo.faces_on_axis_plane(2, 0.0);
    let top = topo.faces_on_axis_plane(2, 2.0);
    let cutter = Cutter {
        surface: CutSurface::Plane(Plane::new(Point3::new(0.0, 0.0, 1.0), Vector3::new(0.0, 0.0, 1.0))?),
        neg_only: Selection::geometry(bottom.clone())?,
        pos_only: Selection::geometry(top.clone())?,
    };
    let (lo, hi) = m.decomposition_mut().unwrap().split_piece(whole, cutter)?;
    let d = m.decomposition().unwrap();
    assert_eq!(d.pieces_of_volume(vol(1)), vec![lo, hi]);
    assert_eq!(d.volumes_of_piece(lo), vec![vol(1)]);
    assert_eq!(d.pieces_of_face(bottom[0], &topo), vec![lo]);
    assert_eq!(d.pieces_of_face(top[0], &topo), vec![hi]);
    // Lateral faces straddle the cut: both pieces own them.
    let lateral = topo.faces_on_axis_plane(0, 0.0)[0];
    assert_eq!(d.pieces_of_face(lateral, &topo), vec![lo, hi]);
    let faces_lo = d.faces_of_piece(lo, &topo);
    assert_eq!(faces_lo.len(), 5 + 1);
    assert!(faces_lo.contains(&PieceFace::Cut { cutter: 0, volume: vol(1), other: hi }));
    assert!(faces_lo.contains(&PieceFace::Model(bottom[0])) && !faces_lo.contains(&PieceFace::Model(top[0])));
    // Pieces carry no colors; they derive from volumes. Coloring is bit-identical.
    assert_eq!(m.piece_colors(lo), vec![color]);
    assert_eq!(m.piece_colors(hi), vec![color]);
    assert_eq!(m.coloring_fingerprint(), before);
    let rep = {
        m.define_response("vacuum", Selector::<Face>::DomainBoundary.resolve(&m, &topo)?)?;
        m.validate(&topo, &tol())
    };
    assert!(rep.is_ok(), "{:?}", rep.issues);
    // Selectors over pieces.
    assert_eq!(Selector::<Face>::Piece(lo).resolve(&m, &topo)?.len(), 5);
    // Reversible: unsplit restores a single whole-volume piece.
    let back = m.decomposition_mut().unwrap().unsplit(hi, lo)?;
    let d = m.decomposition().unwrap();
    assert_eq!(d.pieces().count(), 1);
    assert_eq!(d.piece(back).unwrap().parts.len(), 1);
    assert!(d.piece(back).unwrap().parts[0].path.is_empty() && d.cutters().is_empty());
    assert_eq!(m.coloring_fingerprint(), before);
    // Dropping the decomposition entirely leaves coloring alone too.
    m.clear_decomposition();
    assert_eq!(m.coloring_fingerprint(), before);
    Ok(())
}

#[test]
fn decomposition_tiling_composites_and_staleness() -> R {
    let (topo, mut m, _, _) = row();
    m.decompose(&topo)?;
    let ids: Vec<_> = m.decomposition().unwrap().pieces().map(|p| p.id).collect();
    assert_eq!(ids.len(), 4);
    // Composite of the two M1 boxes (one mappable chunk spanning two volumes).
    let chunk = m.decomposition_mut().unwrap().composite(&[ids[1], ids[2]])?;
    assert_eq!(m.decomposition().unwrap().volumes_of_piece(chunk), vec![vol(2), vol(3)]);
    assert_eq!(m.piece_colors(chunk).len(), 1);
    assert!(m.decomposition().unwrap().check_tiling(&m.domain_volumes(&topo)).is_empty());
    // A fifth volume is uncovered.
    let extra = TableMock::vid(99);
    let defects = m.decomposition().unwrap().check_tiling(&[vol(1), vol(2), vol(3), vol(4), extra]);
    assert_eq!(defects.len(), 1);
    assert_eq!(defects[0].0, extra);
    // Cut pieces cannot be composited; coloring edits make the decomposition stale (a warning).
    m.define_volume_color("late", Selection::empty())?;
    let rep = m.validate(&topo, &tol());
    assert!(rep.has(IssueCode::DecompositionStale));
    assert!(!rep.with_code(IssueCode::DecompositionStale).any(|i| i.severity == Severity::Error));
    Ok(())
}

#[test]
fn selection_algebra_and_lifetimes() -> R {
    let (topo, mut m, m1, m2) = row();
    let a = vsel(&[1, 2, 3]);
    let b = vsel(&[3, 4]);
    assert_eq!(a.union(&b)?.ids(), vsel(&[1, 2, 3, 4]).ids());
    assert_eq!(a.intersection(&b)?.ids(), vsel(&[3]).ids());
    assert_eq!(a.difference(&b)?.ids(), vsel(&[1, 2]).ids());
    assert_eq!(b.complement(&vsel(&[1, 2, 3, 4]))?.ids(), vsel(&[1, 2]).ids());
    assert!(a.contains(vol(2)) && !a.contains(vol(4)));
    // Kind check on construction.
    assert!(matches!(Selection::<Face>::geometry([vol(1)]), Err(ModelError::KindMismatch { .. })));
    // Mesh-bound selections come from lowering and cannot be mixed with geometry-bound ones.
    let mesh = EntityId::new(EntityKind::Mesh, 1).unwrap();
    let lowered = a.lower(mesh, |v| (0..2).map(|k| EntityId::new(EntityKind::MeshCell, v.serial() * 10 + k).unwrap()).collect())?;
    assert_eq!(lowered.len(), 6);
    assert_eq!(lowered.lifetime(), Lifetime::MeshBound { mesh });
    assert!(matches!(lowered.union(&Selection::empty()), Err(ModelError::LifetimeMismatch)));
    assert!(matches!(m.define_volume_color("mesh color", Selection::<Volume>::mesh(mesh, [])?), Err(ModelError::LifetimeMismatch)));
    assert!(lowered.lower(mesh, |_| vec![]).is_err());
    // Selectors.
    let sel = Selector::<Volume>::And(vec![Selector::All, Selector::Not(Box::new(Selector::Color(m1)))]);
    assert_eq!(sel.resolve(&m, &topo)?.ids(), vsel(&[1, 4]).ids());
    let or = Selector::<Volume>::Or(vec![Selector::Color(m1), Selector::Color(m2)]);
    assert_eq!(or.resolve(&m, &topo)?.len(), 4);
    let faces_of = Selector::<Face>::FacesOf(Box::new(Selector::Color(m2))).resolve(&m, &topo)?;
    assert_eq!(faces_of.len(), 12); // two boxes, 6 faces each, no shared face between them
    let back = Selector::<Volume>::VolumesOf(Box::new(Selector::Ids(faces_of))).resolve(&m, &topo)?;
    assert_eq!(back.ids(), vsel(&[1, 2, 3, 4]).ids());
    assert!(Selector::<Volume>::Kind(KindFilter::Interface).resolve(&m, &topo).is_err());
    assert!(Selector::<Volume>::Color(vol_as_color()).resolve(&m, &topo).is_err());
    // Kind filters.
    let all = Selector::<Face>::DomainBoundary.resolve(&m, &topo)?;
    m.define_response("vacuum", all.clone())?;
    assert_eq!(Selector::<Face>::Kind(KindFilter::Response(Some("vacuum".into()))).resolve(&m, &topo)?.len(), all.len());
    assert!(Selector::<Face>::Kind(KindFilter::Deck(None)).resolve(&m, &topo)?.is_empty());
    assert!(matches!(m.define_response("again", all), Err(ModelError::Conflict { .. })));
    Ok(())
}

fn vol_as_color() -> EntityId {
    EntityId::new(EntityKind::Color, 999).unwrap()
}

#[test]
fn interface_intent_is_checked_against_colors() -> R {
    let (topo, mut m, m1, m2) = row();
    // The face at x = -1 separates material 2 (left) from material 1.
    let f = topo.faces_on_axis_plane(0, -1.0);
    m.define_interface(InterfaceIntent { between: Some((m1, m2)), conformity: Conformity::Conformal }, Selection::geometry(f)?)?;
    // The face at x = 0 separates material 1 from material 1: declaring (m1, m2) there is wrong.
    let g = topo.faces_on_axis_plane(0, 0.0);
    m.define_interface(
        InterfaceIntent { between: Some((m1, m2)), conformity: Conformity::NonConformal { partner: None } },
        Selection::geometry(g.clone())?,
    )?;
    let rep = m.validate(&topo, &tol());
    let bad: Vec<_> = rep.with_code(IssueCode::InterfaceColorMismatch).flat_map(|i| i.entities.clone()).collect();
    assert!(bad.contains(&g[0]) && !bad.contains(&topo.faces_on_axis_plane(0, -1.0)[0]));
    // A deck kind on an interior face is an error.
    let (topo, mut m, _, _) = row();
    m.declare_deck(Generator::Mirror { plane: Plane::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0))? }, vec![(topo.faces_on_axis_plane(0, 0.0)[0], topo.faces_on_axis_plane(0, 0.0)[0])])?;
    assert!(m.validate(&topo, &tol()).has(IssueCode::DeckOnInteriorFace));
    Ok(())
}

/// Build the full scenario set and hash everything observable. Used for the determinism test.
fn scenario_hash() -> Result<u64, ModelError> {
    let mut h = Fnv64::new();
    let (topo, mut m, _, _) = row();
    let plane = Plane::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0))?;
    m.cut_keep_chamber(&topo, &CutSpec { generator: Generator::Mirror { plane }, keep: KeepSide::Positive, wedge_start: None }, &tol())?;
    let rest = Selector::<Face>::Unassigned.resolve(&m, &topo)?;
    m.define_response("vacuum", rest)?;
    let u: UnfoldedModel = m.unfold(&topo, &UnfoldOptions::default())?;
    for v in &u.volumes {
        h.id(v.id);
        h.id(v.source);
    }
    for f in &u.faces {
        h.id(f.id);
        h.u64(u64::from(f.loop_reversed));
        h.u64(f.right.map_or(0, EntityId::to_raw));
    }
    for e in &u.elements {
        h.transform(&e.transform);
    }
    let rep = m.validate(&topo, &tol());
    for i in &rep.issues {
        h.str(&i.message);
    }
    let det = m.detect_symmetry(&topo, &DetectOptions::default());
    for c in &det.candidates {
        h.transform(&c.generator.transform());
    }
    let (nm, _) = nested_model()?;
    h.u64(flat_hash(&nm)?);
    let t6 = six_wedges();
    let mut m6 = Model::new();
    m6.define_volume_color("a", vsel(&[1, 2, 3, 4, 5, 6]))?;
    let g = Generator::Rotation { origin: Point3::new(0.0, 0.0, 0.0), axis: Vector3::new(0.0, 0.0, 1.0), order: 6 };
    m6.cut_keep_chamber(&t6, &CutSpec { generator: g, keep: KeepSide::Positive, wedge_start: None }, &tol())?;
    for f in m6.unfold(&t6, &UnfoldOptions::default())?.faces {
        h.id(f.id);
    }
    Ok(h.finish())
}

#[test]
fn determinism_two_runs_hash_equal() -> R {
    // No parallel sections exist in this crate, so thread count cannot matter; two runs must agree.
    assert_eq!(scenario_hash()?, scenario_hash()?);
    Ok(())
}
