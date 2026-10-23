//! End-to-end scenarios on a mock topology. Numbers are analytic (known positions, counts, orders).

use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_model_api::action::{Axis, PathStep, Plane, approx_eq};
use sm_model_api::boundary::{BoundaryKind, Deck, InterfaceIntent, Side};
use sm_model_api::color::{ColorDomain, GroupKind};
use sm_model_api::decomposition::{CutSurface, PieceFace, PieceShape};
use sm_model_api::detect::{CandidateKind, DetectOptions, Verification, detect_symmetry};
use sm_model_api::instancing::{Fill, FlatContent, HexOrientation, Layout, Member, Override, Slot};
use sm_model_api::mock::MockTopology;
use sm_model_api::symmetry::{CapFace, ChamberCut, ChamberSpec, CutOutcome, Keep, SymmetryOrigin, UnfoldSpec};
use sm_model_api::validate::{IssueCode, validate, validate_unfolded};
use sm_model_api::{FaceSide, Lineage, Model, ModelError, Sense, Tolerance, TopologyView};
use smallvec::smallvec;
use std::collections::BTreeMap;

const TOL: Tolerance = Tolerance { linear: 1e-8, angular: 1e-9 };

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

/// Put every single-sided face that has no record yet into one response record.
fn tag_rest_response(m: &mut Model, topo: &MockTopology, name: &str) -> EntityId {
    let b = m.add_boundary(name, BoundaryKind::Response).unwrap();
    let rest: Vec<EntityId> = topo.faces().into_iter().filter(|f| topo.face_sides(*f).len() == 1 && !m.face_boundary.contains_key(f)).collect();
    m.assign_faces(b, &rest).unwrap();
    b
}

#[test]
fn two_materials_with_boundary_colors() {
    let mut t = MockTopology::new();
    let (fuel_v, _) = t.add_box(p(0.0, 0.0, 0.0), p(1.0, 1.0, 1.0));
    let (clad_v, cf) = t.add_box(p(1.0, 0.0, 0.0), p(2.0, 1.0, 1.0));
    let shared = t.faces().into_iter().find(|f| t.face_sides(*f).len() == 2).unwrap();

    let mut m = Model::new();
    let fuel = m.add_color("fuel", ColorDomain::Volume).unwrap();
    let clad = m.add_color("clad", ColorDomain::Volume).unwrap();
    let outer = m.add_color("outer", ColorDomain::Boundary).unwrap();
    m.color_volume(fuel_v, fuel).unwrap();
    m.color_volume(clad_v, clad).unwrap();
    m.color_face(cf[1], outer).unwrap(); // +x end of clad

    // Colors are domain-checked.
    assert_eq!(m.color_volume(fuel_v, outer), Err(ModelError::ColorDomain(outer)));
    assert_eq!(m.color_face(cf[1], fuel), Err(ModelError::ColorDomain(fuel)));

    // Interface between the two colors, response everywhere else.
    let iface = m.add_boundary("fuel|clad", BoundaryKind::Interface(InterfaceIntent::Conformal)).unwrap();
    m.assign_faces(iface, &[shared]).unwrap();
    tag_rest_response(&mut m, &t, "walls");

    // Named groups: block and assembly (nested).
    let blk = m.add_group("fuel-block", GroupKind::Block, &[fuel_v]).unwrap();
    let asm = m.add_group("assembly", GroupKind::Assembly, &[blk, clad_v]).unwrap();
    assert_eq!(m.group_volumes(asm).unwrap(), vec![fuel_v, clad_v]);
    assert_eq!(m.volumes_of_color(clad), vec![clad_v]);

    assert!(validate(&m, &t, &TOL).is_ok(), "{:?}", validate(&m, &t, &TOL));

    // Exact partition: an uncolored volume is reported.
    let mut bad = m.clone();
    bad.volume_color.remove(&clad_v);
    assert_eq!(validate(&bad, &t, &TOL).count(IssueCode::VolumeUncolored), 1);

    // Every boundary face needs a kind.
    let mut bad = m.clone();
    let victim = *bad.face_boundary.keys().find(|f| **f != shared).unwrap();
    bad.face_boundary.remove(&victim);
    assert_eq!(validate(&bad, &t, &TOL).count(IssueCode::BoundaryFaceWithoutKind), 1);

    // A response kind on an interior face is misplaced.
    let mut bad = m.clone();
    bad.face_boundary.remove(&shared);
    let resp = *bad.boundaries.iter().find(|(_, b)| b.kind == BoundaryKind::Response).unwrap().0;
    bad.assign_faces(resp, &[shared]).unwrap();
    assert_eq!(validate(&bad, &t, &TOL).count(IssueCode::KindMisplaced), 1);

    // A face belongs to one record only.
    let r2 = m.add_boundary("again", BoundaryKind::Response).unwrap();
    assert_eq!(m.assign_faces(r2, &[shared]), Err(ModelError::Duplicate(shared)));
}

#[test]
fn mirror_cut_keep_chamber_then_unfold_back() {
    // Before: one box x in [-1,1]. After the (mock) kernel cut: the x >= 0 half with a new cap at x = 0.
    let mut before = MockTopology::new();
    let (v0, f0) = before.add_box(p(-1.0, 0.0, 0.0), p(1.0, 1.0, 1.0));
    let mut after = MockTopology::new();
    let (v1, f1) = after.add_box(p(0.0, 0.0, 0.0), p(1.0, 1.0, 1.0));

    let mut m = Model::new();
    let fuel = m.add_color("fuel", ColorDomain::Volume).unwrap();
    let outer = m.add_color("outer", ColorDomain::Boundary).unwrap();
    let cap_color = m.add_color("symmetry-plane", ColorDomain::Boundary).unwrap();
    m.color_volume(v0, fuel).unwrap();
    m.color_face(f0[1], outer).unwrap();
    tag_rest_response(&mut m, &before, "walls");
    assert!(validate(&m, &before, &TOL).is_ok());

    let plane = Plane::new(p(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap();
    let spec = ChamberSpec { cut: ChamberCut::Mirror { plane, keep: Keep::Positive }, name: "mirror-x".into(), cap_color: Some(cap_color) };
    let mut lineage = vec![Lineage { old: v0, new: smallvec![v1] }, Lineage { old: f0[0], new: smallvec![] }];
    for k in 1..6 {
        lineage.push(Lineage { old: f0[k], new: smallvec![f1[k]] });
    }
    let outcome = CutOutcome { lineage, caps: vec![CapFace { face: f1[0], mate: None }] };

    // A failing outcome leaves the model untouched (transactional).
    let snapshot = m.clone();
    let bad = CutOutcome { caps: vec![], lineage: outcome.lineage.clone() };
    assert!(m.cut_and_keep_chamber(&ChamberSpec { cut: ChamberCut::Mirror { plane, keep: Keep::Positive }, name: "walls".into(), cap_color: None }, &bad).is_err());
    assert_eq!(m, snapshot);

    let b = m.cut_and_keep_chamber(&spec, &outcome).unwrap();
    assert_eq!(m.volume_color.get(&v1), Some(&fuel));
    assert_eq!(m.face_color.get(&f1[1]), Some(&outer), "face color follows lineage");
    assert_eq!(m.face_color.get(&f1[0]), Some(&cap_color));
    assert!(matches!(m.face_kind(f1[0]), Some(BoundaryKind::Deck(Deck::Mirror { .. }))));
    assert_eq!(m.symmetry.as_ref().unwrap().origin, SymmetryOrigin::Cut);
    assert_eq!(m.symmetry.as_ref().unwrap().domain, vec![v1]);
    let rep = validate(&m, &after, &TOL);
    assert!(rep.is_ok(), "{rep:?}");
    assert_eq!(m.boundary_pairs(b), vec![(f1[0], f1[0])]);

    // Unfold: 2 copies, the second improper (mirror), cap glued to itself across the plane.
    let snap = m.clone();
    let u = m.unfold(&UnfoldSpec::default()).unwrap();
    assert_eq!(m, snap, "unfold is a pure view");
    assert_eq!(u.copies.len(), 2);
    assert!(!u.copies[0].improper && u.copies[1].improper);
    assert!(approx_eq(&u.copies[0].transform, &Transform::IDENTITY, &TOL));
    let image = u.copies[1].transform.apply_point(p(0.25, 0.5, 0.5));
    assert!((image.x + 0.25).abs() < 1e-12 && (image.y - 0.5).abs() < 1e-12);
    assert_eq!(u.volumes.len(), 2);
    assert!(u.volumes.iter().all(|v| v.color == Some(fuel)));
    assert_eq!(u.glued.len(), 1);
    assert_eq!((u.glued[0].a.base, u.glued[0].b.base), (f1[0], f1[0]));
    assert!(u.exposed.is_empty());
    assert!(validate_unfolded(&after, &u, &TOL).is_ok());
    // Reversible: every unfolded entity folds to a base entity of F; copy 0 is F itself.
    assert!(u.volumes.iter().all(|v| v.at.base == v1 && u.fold(v.at) == v1));
    // Orientation flips for the improper copy.
    let s0 = u.own_face_sides(&after, 0, f1[1]);
    let s1 = u.own_face_sides(&after, 1, f1[1]);
    assert_eq!(s0[0].1, Sense::Forward);
    assert_eq!(s1[0].1, Sense::Reverse);
}

#[test]
fn translation_pair_validates_and_tiles() {
    let mut t = MockTopology::new();
    let (v, f) = t.add_box(p(0.0, 0.0, 0.0), p(1.0, 1.0, 1.0));
    let mut m = Model::new();
    let c = m.add_color("fuel", ColorDomain::Volume).unwrap();
    m.color_volume(v, c).unwrap();
    let deck = m.add_boundary("periodic-x", BoundaryKind::Deck(Deck::Translation { shift: Vector3::new(1.0, 0.0, 0.0) })).unwrap();
    m.assign_pairs(deck, &[(f[0], f[1])]).unwrap();
    tag_rest_response(&mut m, &t, "walls");
    m.declare_symmetry(&[deck], &[v], SymmetryOrigin::Declared).unwrap();
    assert!(validate(&m, &t, &TOL).is_ok());
    assert_eq!(m.boundary_faces(deck, Side::B), vec![f[1]]);

    // Wrong shift: B is not T(A).
    let mut bad = m.clone();
    bad.boundaries.get_mut(&deck).unwrap().kind = BoundaryKind::Deck(Deck::Translation { shift: Vector3::new(1.1, 0.0, 0.0) });
    assert_eq!(validate(&bad, &t, &TOL).count(IssueCode::DeckGeometry), 1);
    // Mates must be reciprocal.
    let mut bad = m.clone();
    bad.face_boundary.get_mut(&f[1]).unwrap().mate = Some(f[2]);
    assert!(validate(&bad, &t, &TOL).count(IssueCode::DeckPairing) >= 1);
    // Different face colors on mates.
    let mut bad = m.clone();
    let wall = bad.add_color("wall", ColorDomain::Boundary).unwrap();
    bad.color_face(f[0], wall).unwrap();
    assert_eq!(validate(&bad, &t, &TOL).count(IssueCode::DeckColor), 1);
    // Pairs only on paired kinds; deck kinds expose their transform.
    assert!(m.face_deck_transform(f[1]).unwrap().is_ok());

    // Unfolding an infinite group needs a repeat count.
    assert_eq!(m.unfold(&UnfoldSpec::default()), Err(ModelError::MissingRepeats(deck)));
    let spec = UnfoldSpec { repeats: BTreeMap::from([(deck, 3)]), ..UnfoldSpec::default() };
    let u = m.unfold(&spec).unwrap();
    assert_eq!(u.copies.len(), 3);
    assert_eq!(u.glued.len(), 2, "interior seams of a 3-tile strip");
    assert_eq!(u.exposed.len(), 2, "outer ends of the strip");
    assert!((u.copies[2].transform.t.x - 2.0).abs() < 1e-12);
    assert!(validate_unfolded(&t, &u, &TOL).is_ok());
}

fn wedge_topology() -> (MockTopology, EntityId, [EntityId; 5]) {
    let (c, s) = (0.5, 3f64.sqrt() / 2.0);
    let mut t = MockTopology::new();
    let v = t.add_volume(None);
    let fw = |v| [FaceSide { volume: v, sense: Sense::Forward }];
    let a = t.add_face(vec![p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(1.0, 0.0, 1.0), p(0.0, 0.0, 1.0)], &fw(v));
    let b = t.add_face(vec![p(0.0, 0.0, 0.0), p(c, s, 0.0), p(c, s, 1.0), p(0.0, 0.0, 1.0)], &fw(v));
    let outer = t.add_face(vec![p(1.0, 0.0, 0.0), p(c, s, 0.0), p(c, s, 1.0), p(1.0, 0.0, 1.0)], &fw(v));
    let bot = t.add_face(vec![p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(c, s, 0.0)], &fw(v));
    let top = t.add_face(vec![p(0.0, 0.0, 1.0), p(1.0, 0.0, 1.0), p(c, s, 1.0)], &fw(v));
    (t, v, [a, b, outer, bot, top])
}

#[test]
fn one_sixth_rotation_unfolds_to_six_copies() {
    let (t, v, f) = wedge_topology();
    let mut m = Model::new();
    let c = m.add_color("fuel", ColorDomain::Volume).unwrap();
    m.color_volume(v, c).unwrap();
    let axis = Axis::new(p(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
    let deck = m.add_boundary("wedge", BoundaryKind::Deck(Deck::Rotation { axis, order: 6 })).unwrap();
    m.assign_pairs(deck, &[(f[0], f[1])]).unwrap();
    tag_rest_response(&mut m, &t, "walls");
    m.declare_symmetry(&[deck], &[v], SymmetryOrigin::Declared).unwrap();
    let rep = validate(&m, &t, &TOL);
    assert!(rep.is_ok(), "{rep:?}");

    let u = m.unfold(&UnfoldSpec::default()).unwrap();
    assert_eq!(u.copies.len(), 6);
    assert_eq!(u.volumes.len(), 6);
    assert_eq!(u.glued.len(), 6, "every seam closes: the group is cyclic of order 6");
    assert!(u.exposed.is_empty());
    assert!(u.copies.iter().all(|c| !c.improper));
    assert!(validate_unfolded(&t, &u, &TOL).is_ok());
    // Copy 3 is the half turn: (1,0,0) -> (-1,0,0).
    let q = u.copies[3].transform.apply_point(p(1.0, 0.0, 0.0));
    assert!((q.x + 1.0).abs() < 1e-12 && q.y.abs() < 1e-12);

    // A wrong order breaks the pairing geometry.
    let mut bad = m.clone();
    bad.boundaries.get_mut(&deck).unwrap().kind = BoundaryKind::Deck(Deck::Rotation { axis, order: 5 });
    assert_eq!(validate(&bad, &t, &TOL).count(IssueCode::DeckGeometry), 1);

    // Wedge start direction must be perpendicular to the axis.
    let mut m2 = Model::new();
    let spec = ChamberSpec { cut: ChamberCut::Wedge { axis, order: 6, start: Vector3::new(0.0, 0.0, 1.0) }, name: "w".into(), cap_color: None };
    assert!(m2.cut_and_keep_chamber(&spec, &CutOutcome { lineage: vec![], caps: vec![] }).is_err());
}

fn pin_model(t: &mut MockTopology, m: &mut Model) -> (EntityId, EntityId, [EntityId; 4]) {
    let (fv, _) = t.add_box(p(-0.4, -0.4, 0.0), p(0.4, 0.4, 1.0));
    let (cv, _) = t.add_box(p(-0.5, -0.5, 0.0), p(-0.4, 0.5, 1.0));
    let fuel = m.add_color("fuel", ColorDomain::Volume).unwrap();
    let clad = m.add_color("clad", ColorDomain::Volume).unwrap();
    let absorber = m.add_color("absorber", ColorDomain::Volume).unwrap();
    let water = m.add_color("water", ColorDomain::Volume).unwrap();
    m.color_volume(fv, fuel).unwrap();
    m.color_volume(cv, clad).unwrap();
    (fv, cv, [fuel, clad, absorber, water])
}

#[test]
fn hex_lattice_of_pins_with_override() {
    let mut t = MockTopology::new();
    let mut m = Model::new();
    let (fv, cv, [fuel, clad, absorber, water]) = pin_model(&mut t, &mut m);
    tag_rest_response(&mut m, &t, "walls");
    let pin = m.add_prototype("pin", &[Member::Volume(fv), Member::Volume(cv)]).unwrap();

    let ov = vec![Override::ReplaceColor { from: fuel, to: absorber }];
    let layout = Layout::Hex { pitch: 1.5, axial_pitch: 1.0, rings: 2, layers: 1, orientation: HexOrientation::XRows };
    let mut slots = BTreeMap::new();
    for idx in layout.indices() {
        if idx == [-1, 0, 0] {
            continue; // sparse: left to `outer`
        }
        let overrides = if idx == [0, 0, 0] { ov.clone() } else { vec![] };
        slots.insert(idx, Slot { fill: Fill::Prototype(pin), transform: None, overrides });
    }
    let lat = m.add_lattice("core", layout, slots, Some(Fill::Homogeneous(water))).unwrap();
    // Slots outside the layout are rejected.
    assert!(m.add_lattice("bad", layout, BTreeMap::from([([5, 0, 0], Slot { fill: Fill::Prototype(pin), transform: None, overrides: vec![] })]), None).is_err());
    let root = m.add_instance(Fill::Lattice(lat), Transform::IDENTITY, vec![]).unwrap();
    m.add_root(root).unwrap();

    let rep = validate(&m, &t, &TOL);
    assert!(rep.is_ok(), "{rep:?}");
    let flat = m.flatten().unwrap();
    assert_eq!(flat.entries.len(), 6 * 2 + 1, "6 pins x 2 volumes + 1 homogeneous hole");
    assert_eq!(flat.ranges.len(), 7);

    let at = |idx: [i32; 3]| -> Vec<&sm_model_api::instancing::FlatEntry> {
        flat.entries.iter().filter(|e| e.placed.path.contains(&PathStep::Slot { lattice: lat, index: idx })).collect()
    };
    let center = at([0, 0, 0]);
    assert_eq!(center[0].content, FlatContent::Volume { volume: fv, color: Some(absorber) });
    assert_eq!(center[1].content, FlatContent::Volume { volume: cv, color: Some(clad) });
    let east = at([1, 0, 0]);
    assert_eq!(east[0].content, FlatContent::Volume { volume: fv, color: Some(fuel) }, "override is per slot");
    assert!((east[0].placed.transform.t.x - 1.5).abs() < 1e-12 && east[0].placed.transform.t.y.abs() < 1e-12);
    let ne = at([0, 1, 0]);
    assert!((ne[0].placed.transform.t.x - 0.75).abs() < 1e-12 && (ne[0].placed.transform.t.y - 1.5 * 3f64.sqrt() / 2.0).abs() < 1e-12);
    assert_eq!(at([-1, 0, 0])[0].content, FlatContent::Homogeneous { color: water });

    // Outer-level overrides are applied after (and so win over) inner ones.
    let root2 = m.add_instance(Fill::Lattice(lat), Transform::IDENTITY, vec![Override::ReplaceColor { from: absorber, to: clad }]).unwrap();
    m.instancing.roots = vec![root2];
    let flat2 = m.flatten().unwrap();
    let c2 = flat2.entries.iter().find(|e| e.placed.path.contains(&PathStep::Slot { lattice: lat, index: [0, 0, 0] })).unwrap();
    assert_eq!(c2.content, FlatContent::Volume { volume: fv, color: Some(clad) });
}

#[test]
fn nested_lattice_flatten_is_deterministic() {
    let build = || {
        let mut t = MockTopology::new();
        let mut m = Model::new();
        let (fv, cv, _) = pin_model(&mut t, &mut m);
        tag_rest_response(&mut m, &t, "walls");
        let pin = m.add_prototype("pin", &[Member::Volume(fv), Member::Volume(cv)]).unwrap();
        let pins = m
            .add_lattice(
                "pins",
                Layout::Cartesian { pitch: [1.0, 1.0, 1.0], dims: [3, 3, 1] },
                BTreeMap::new(),
                Some(Fill::Prototype(pin)),
            )
            .unwrap();
        let asm_proto = m.add_prototype("assembly", &[]).unwrap();
        let _ = asm_proto;
        let asm = m
            .add_lattice(
                "assemblies",
                Layout::Cartesian { pitch: [10.0, 10.0, 1.0], dims: [2, 2, 1] },
                BTreeMap::new(),
                Some(Fill::Lattice(pins)),
            )
            .unwrap();
        let mirrored = Transform { m: [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: Vector3::default() };
        let r1 = m.add_instance(Fill::Lattice(asm), Transform::IDENTITY, vec![]).unwrap();
        let r2 = m.add_instance(Fill::Lattice(asm), mirrored, vec![]).unwrap();
        m.add_root(r1).unwrap();
        m.add_root(r2).unwrap();
        (t, m, asm, pins, r1)
    };
    let (t, m, asm, pins, r1) = build();
    assert!(validate(&m, &t, &TOL).is_ok());
    let flat = m.flatten().unwrap();
    // 2 roots x 4 assemblies x 9 pins x 2 volumes
    assert_eq!(flat.entries.len(), 2 * 4 * 9 * 2);
    assert_eq!(flat.ranges.len(), 2 * 4 * 9);
    assert!(flat.ranges.iter().all(|(_, r)| r.len() == 2));
    // Assembly slot [1,1,0], pin slot [2,0,0] -> (10 + 2, 10 + 0) in the first root.
    let path: Vec<PathStep> = vec![
        PathStep::Instance(r1),
        PathStep::Slot { lattice: asm, index: [1, 1, 0] },
        PathStep::Slot { lattice: pins, index: [2, 0, 0] },
    ];
    let e = flat.entries.iter().find(|e| e.placed.path.as_slice() == path.as_slice()).unwrap();
    assert!((e.placed.transform.t.x - 12.0).abs() < 1e-12 && (e.placed.transform.t.y - 10.0).abs() < 1e-12);
    assert!(!e.placed.improper);
    // Mirrored root: x flips and every copy is improper.
    let mirrored: Vec<_> = flat.entries.iter().filter(|e| e.placed.improper).collect();
    assert_eq!(mirrored.len(), 4 * 9 * 2);
    // Determinism: repeated runs, a fresh build, and another thread give the same bits.
    let fp = flat.fingerprint();
    assert_eq!(m.flatten().unwrap().fingerprint(), fp);
    assert_eq!(build().1.flatten().unwrap().fingerprint(), fp);
    let from_thread = std::thread::scope(|s| s.spawn(|| m.flatten().unwrap().fingerprint()).join().unwrap());
    assert_eq!(from_thread, fp);
    assert_eq!(m.flatten().unwrap(), flat);
}

#[test]
fn instancing_cycles_are_errors() {
    let mut m = Model::new();
    let p1 = m.add_prototype("a", &[]).unwrap();
    let i = m.add_instance(Fill::Prototype(p1), Transform::IDENTITY, vec![]).unwrap();
    m.instancing.prototypes.get_mut(&p1).unwrap().members.push(Member::Instance(i));
    m.add_root(i).unwrap();
    assert_eq!(m.flatten(), Err(ModelError::Cycle(p1)));
    let t = MockTopology::new();
    assert_eq!(validate(&m, &t, &TOL).count(IssueCode::InstancingInvalid), 1);
}

#[test]
fn decomposition_into_two_sweep_pieces_preserves_colors() {
    let mut t = MockTopology::new();
    let (v, f) = t.add_box(p(0.0, 0.0, 0.0), p(1.0, 1.0, 1.0));
    let mut m = Model::new();
    let c = m.add_color("fuel", ColorDomain::Volume).unwrap();
    m.color_volume(v, c).unwrap();
    tag_rest_response(&mut m, &t, "walls");
    let before = m.clone();

    let cut = m.add_cut(v, CutSurface::Plane(Plane::new(p(0.0, 0.0, 0.5), Vector3::new(0.0, 0.0, 1.0)).unwrap())).unwrap();
    let lateral = [f[0], f[1], f[2], f[3]].map(PieceFace::Original);
    let mut lo = lateral.to_vec();
    lo.extend([PieceFace::Original(f[4]), PieceFace::Cut(cut)]);
    let mut hi = lateral.to_vec();
    hi.extend([PieceFace::Original(f[5]), PieceFace::Cut(cut)]);
    let p0 = m.add_piece("lower", &[v], &lo, PieceShape::Sweep { source: PieceFace::Original(f[4]), target: PieceFace::Cut(cut) }).unwrap();
    let p1 = m.add_piece("upper", &[v], &hi, PieceShape::Sweep { source: PieceFace::Cut(cut), target: PieceFace::Original(f[5]) }).unwrap();
    assert_eq!(p0.kind(), sm_core::EntityKind::PartitionCell);

    let rep = validate(&m, &t, &TOL);
    assert!(rep.is_ok(), "{rep:?}");
    let d = m.decomposition.as_ref().unwrap();
    assert_eq!(d.pieces_of_volume(v), vec![p0, p1]);
    assert_eq!(d.pieces_of_face(f[0]), vec![p0, p1]);
    assert_eq!(d.pieces_of_face(f[4]), vec![p0]);
    assert_eq!(d.pieces_of_face(f[5]), vec![p1]);
    assert_eq!(d.volumes_of_piece(p1), &[v]);
    assert_eq!(d.pieces_of_cut(cut), vec![p0, p1]);
    assert_eq!(m.piece_color(p0), Some(c));
    assert_eq!(m.piece_color(p1), Some(c));

    // Coloring is untouched; undoing restores the plain model exactly.
    assert_eq!(m.colors, before.colors);
    assert_eq!(m.volume_color, before.volume_color);
    assert_eq!(m.face_color, before.face_color);
    assert_eq!(m.face_boundary, before.face_boundary);
    m.undo_decomposition();
    assert_eq!(m.decomposition, None);
    assert_eq!(m.volume_color, before.volume_color);
}

#[test]
fn decomposition_validator_catches_color_changes_and_gaps() {
    let mut t = MockTopology::new();
    let (v1, _) = t.add_box(p(0.0, 0.0, 0.0), p(1.0, 1.0, 1.0));
    let (v2, _) = t.add_box(p(1.0, 0.0, 0.0), p(2.0, 1.0, 1.0));
    let mut m = Model::new();
    let a = m.add_color("a", ColorDomain::Volume).unwrap();
    let b = m.add_color("b", ColorDomain::Volume).unwrap();
    m.color_volume(v1, a).unwrap();
    m.color_volume(v2, b).unwrap();
    tag_rest_response(&mut m, &t, "walls");
    let shared = t.faces().into_iter().find(|f| t.face_sides(*f).len() == 2).unwrap();
    let iface = m.add_boundary("i", BoundaryKind::Interface(InterfaceIntent::Conformal)).unwrap();
    m.assign_faces(iface, &[shared]).unwrap();
    m.add_piece("composite", &[v1, v2], &[], PieceShape::Free).unwrap();
    let rep = validate(&m, &t, &TOL);
    assert_eq!(rep.count(IssueCode::PieceColorMixed), 1);
    assert_eq!(m.piece_color(*m.decomposition.as_ref().unwrap().pieces.keys().next().unwrap()), None);

    // Uncovered volume + dangling cut.
    let mut m2 = m.clone();
    m2.decomposition = None;
    m2.add_piece("only-first", &[v1], &[], PieceShape::Free).unwrap();
    m2.add_cut(v1, CutSurface::Plane(Plane::new(p(0.5, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap())).unwrap();
    let rep = validate(&m2, &t, &TOL);
    assert_eq!(rep.count(IssueCode::VolumeUncovered), 1);
    assert_eq!(rep.count(IssueCode::CutInvalid), 1);

    // Trivial decomposition is valid.
    let mut m3 = m.clone();
    m3.decomposition = None;
    m3.decompose_trivially(&t).unwrap();
    assert!(validate(&m3, &t, &TOL).is_ok());
}

/// Three boxes along x (clad | fuel | clad), square cross-section in y, z.
fn slab_stack(left_clad_same: bool) -> (MockTopology, Model) {
    let mut t = MockTopology::new();
    let (l, _) = t.add_box(p(-2.0, -1.0, -1.0), p(-1.0, 1.0, 1.0));
    let (c, _) = t.add_box(p(-1.0, -1.0, -1.0), p(1.0, 1.0, 1.0));
    let (r, _) = t.add_box(p(1.0, -1.0, -1.0), p(2.0, 1.0, 1.0));
    let mut m = Model::new();
    let fuel = m.add_color("fuel", ColorDomain::Volume).unwrap();
    let clad = m.add_color("clad", ColorDomain::Volume).unwrap();
    let clad2 = if left_clad_same { clad } else { m.add_color("clad-right", ColorDomain::Volume).unwrap() };
    m.color_volume(l, clad).unwrap();
    m.color_volume(c, fuel).unwrap();
    m.color_volume(r, clad2).unwrap();
    (t, m)
}

#[test]
fn detection_finds_verified_symmetries_and_rejects_false_candidates() {
    let (t, m) = slab_stack(true);
    let res = detect_symmetry(&m, &t, &DetectOptions::default()).unwrap();
    assert_eq!(res.invariants.len(), 2);
    assert!(res.global_barycentre.x.abs() < 1e-12);

    let verified_mirror = |n: Vector3| {
        res.verified().any(|c| matches!(c.kind, CandidateKind::Mirror { plane } if (plane.normal.dot(n).abs() - 1.0).abs() < 1e-9))
    };
    assert!(verified_mirror(Vector3::new(1.0, 0.0, 0.0)));
    assert!(verified_mirror(Vector3::new(0.0, 1.0, 0.0)));
    assert!(verified_mirror(Vector3::new(0.0, 0.0, 1.0)));

    let rot = |axis: Vector3, order: u32| {
        res.candidates.iter().find(|c| matches!(c.kind, CandidateKind::Rotation { axis: a, order: o } if o == order && (a.dir.dot(axis).abs() - 1.0).abs() < 1e-9))
    };
    let x = Vector3::new(1.0, 0.0, 0.0);
    // Square cross-section: the inertia tensor is isotropic in (y, z), so every order passes the invariants,
    // but only 2 and 4 are real symmetries.
    for order in [2, 4] {
        assert!(rot(x, order).unwrap().is_verified(), "order {order}");
    }
    for order in [3, 5, 8] {
        let c = rot(x, order).expect("passes the invariants");
        assert!(matches!(c.status, Verification::Rejected { .. }), "order {order} must be rejected by verification");
    }
    // About y the cross-section (x extent 4, z extent 2) is not isotropic: only the half turn survives.
    assert!(rot(Vector3::new(0.0, 1.0, 0.0), 2).unwrap().is_verified());
    assert!(rot(Vector3::new(0.0, 1.0, 0.0), 4).is_none());

    // A verified candidate yields a cut request.
    let mirror = res.verified().find(|c| matches!(c.kind, CandidateKind::Mirror { .. })).unwrap();
    assert!(matches!(mirror.chamber("m").cut, ChamberCut::Mirror { .. }));
    let wedge = rot(x, 4).unwrap().chamber("w");
    if let ChamberCut::Wedge { axis, start, .. } = wedge.cut {
        assert!(start.dot(axis.dir).abs() < 1e-12 && (start.norm() - 1.0).abs() < 1e-12);
    } else {
        panic!("expected wedge");
    }

    // Different colors left/right: the x mirror disappears, y and z remain.
    let (t2, m2) = slab_stack(false);
    let res2 = detect_symmetry(&m2, &t2, &DetectOptions::default()).unwrap();
    let has_x = res2.candidates.iter().any(|c| matches!(c.kind, CandidateKind::Mirror { plane } if plane.normal.dot(x).abs() > 1.0 - 1e-9));
    assert!(!has_x, "colors break the x mirror at the invariant stage");
    assert!(res2.verified().any(|c| matches!(c.kind, CandidateKind::Mirror { plane } if plane.normal.dot(Vector3::new(0.0, 1.0, 0.0)).abs() > 1.0 - 1e-9)));

    // Unverified mode only checks invariants.
    let res3 = detect_symmetry(&m, &t, &DetectOptions { verify: false, ..DetectOptions::default() }).unwrap();
    assert!(res3.candidates.iter().all(|c| c.status == Verification::Unverified));
    assert!(res3.candidates.len() > res.verified().count());
    // Deterministic.
    assert_eq!(detect_symmetry(&m, &t, &DetectOptions::default()).unwrap(), res);
}

#[test]
fn verify_rejects_a_shifted_plane() {
    use sm_model_api::action::reflection;
    use sm_model_api::detect::verify_transform;
    let (t, m) = slab_stack(true);
    let good = reflection(&Plane::new(p(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap());
    let bad = reflection(&Plane::new(p(0.3, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap());
    let vgood = verify_transform(&m, &t, &good, &TOL);
    assert!(matches!(vgood, Verification::Verified { max_deviation } if max_deviation < 1e-12), "{vgood:?}");
    assert!(matches!(verify_transform(&m, &t, &bad, &TOL), Verification::Rejected { .. }));
}

#[test]
fn mirror_face_off_plane_is_reported() {
    let mut t = MockTopology::new();
    let (v, f) = t.add_box(p(0.0, 0.0, 0.0), p(1.0, 1.0, 1.0));
    let mut m = Model::new();
    let c = m.add_color("fuel", ColorDomain::Volume).unwrap();
    m.color_volume(v, c).unwrap();
    let plane = Plane::new(p(0.1, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap();
    let mir = m.add_boundary("mirror", BoundaryKind::Deck(Deck::Mirror { plane })).unwrap();
    m.assign_faces(mir, &[f[0]]).unwrap();
    tag_rest_response(&mut m, &t, "walls");
    assert_eq!(validate(&m, &t, &TOL).count(IssueCode::DeckGeometry), 1);
    // Paired assignment on a self-paired kind is refused up front.
    let mut m2 = m.clone();
    let m3 = m2.add_boundary("m2", BoundaryKind::Deck(Deck::Mirror { plane })).unwrap();
    assert!(m2.assign_pairs(m3, &[(f[2], f[3])]).is_err());
}

#[test]
fn whole_pipeline_is_deterministic() {
    let run = || {
        let (t, m) = slab_stack(true);
        let det = detect_symmetry(&m, &t, &DetectOptions::default()).unwrap();
        let (wt, v, f) = wedge_topology();
        let mut w = Model::new();
        let c = w.add_color("c", ColorDomain::Volume).unwrap();
        w.color_volume(v, c).unwrap();
        let axis = Axis::new(p(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let d = w.add_boundary("w", BoundaryKind::Deck(Deck::Rotation { axis, order: 6 })).unwrap();
        w.assign_pairs(d, &[(f[0], f[1])]).unwrap();
        tag_rest_response(&mut w, &wt, "walls");
        w.declare_symmetry(&[d], &[v], SymmetryOrigin::Declared).unwrap();
        let u = w.unfold(&UnfoldSpec::default()).unwrap();
        format!("{det:?}|{u:?}|{:?}", validate(&w, &wt, &TOL))
    };
    let a = run();
    assert_eq!(a, run());
    assert_eq!(a, std::thread::scope(|s| s.spawn(run).join().unwrap()));
}
