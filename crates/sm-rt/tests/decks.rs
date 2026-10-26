//! Acceptance 6: mirror, translation and rotation decks, response boundaries, closed box.
mod common;
use common::*;
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_mesh_api::SerialAlloc;
use sm_mesh_api::reference::hex_block;
use sm_model_api::{Axis, BoundaryKind, Deck, DeckSide, FaceBoundary, FaceRole, Plane, Turns};
use sm_rt_api::{BoundaryAction, BoundaryDef, BoundarySelector, BoundarySpec, CellId, FaceAction, ResponseKind, ResponseMap, Scratch, StepEnd, Tracker};

fn face(n: u64) -> EntityId {
    EntityId::new(EntityKind::TopoFace, n).unwrap()
}
fn bnd(n: u64) -> EntityId {
    EntityId::new(EntityKind::Boundary, n).unwrap()
}
/// `hex_block` boundary faces: TopoFace#1..6 are -x,+x,-y,+y,-z,+z.
const MX: u64 = 1;
const PX: u64 = 2;
const PY: u64 = 4;

fn block(n: u32) -> sm_mesh_api::reference::ReferenceMesh {
    hex_block([n; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap()
}

#[test]
fn acc06_mirror_reflects_axis_aligned_directions_bit_exactly() {
    let mesh = block(4);
    let mut spec = BoundarySpec::default();
    let mirror = Deck::Mirror { plane: Plane { normal: Vector3::new(1.0, 0.0, 0.0), offset: 4.0 } };
    let fb = FaceBoundary { boundary: bnd(1), role: FaceRole::Deck { side: DeckSide::SelfPaired, mate: None } };
    spec.add_model_face(face(PX), &fb, &BoundaryKind::Deck(mirror), &ResponseMap::new()).unwrap();
    let t = build_mesh_tracker(&mesh, &spec, &opts_fail());
    let mut sc = Scratch::new();
    // Axis-aligned: exact sign flip.
    let mut st = t.init_state(Point3::new(1.5, 1.5, 1.5), Vector3::new(1.0, 0.0, 0.0), None, &mut sc).unwrap();
    let mut reflected = false;
    for _ in 0..20 {
        let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
        if let StepEnd::Boundary { action, .. } = s.end {
            assert_eq!(action, FaceAction::Reflect);
            assert_eq!(st.dir, Vector3::new(-1.0, 0.0, 0.0), "bit-exact reflection");
            assert_eq!(st.pos.x, 4.0);
            reflected = true;
            break;
        }
    }
    assert!(reflected);
    // Oblique: only the normal component changes, bit for bit.
    let d = Vector3::new(0.6, -0.48, 0.64);
    let mut st = t.init_state(Point3::new(3.5, 2.0, 2.0), d, None, &mut sc).unwrap();
    for _ in 0..20 {
        if matches!(t.step(&mut st, f64::INFINITY, &mut sc).unwrap().end, StepEnd::Boundary { .. }) {
            break;
        }
    }
    assert_eq!((st.dir.x, st.dir.y, st.dir.z), (-0.6, -0.48, 0.64));
    // The reflected particle keeps tracking (and now heads for the vacuum face at x = 0).
    let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    assert!(matches!(s.end, StepEnd::CellFace { .. } | StepEnd::Boundary { .. }));
}

fn translation_spec() -> BoundarySpec {
    let mut spec = BoundarySpec::default();
    let deck = Deck::Translation { vector: Vector3::new(-4.0, 0.0, 0.0) };
    // Side A is +x (g maps it to -x), side B is -x.
    let a = FaceBoundary { boundary: bnd(2), role: FaceRole::Deck { side: DeckSide::A, mate: Some(face(MX)) } };
    let b = FaceBoundary { boundary: bnd(2), role: FaceRole::Deck { side: DeckSide::B, mate: Some(face(PX)) } };
    spec.add_model_face(face(PX), &a, &BoundaryKind::Deck(deck), &ResponseMap::new()).unwrap();
    spec.add_model_face(face(MX), &b, &BoundaryKind::Deck(deck), &ResponseMap::new()).unwrap();
    spec
}

#[test]
fn acc06_translation_pair_wraps_position_and_enters_through_the_mate() {
    let mesh = block(4);
    let t = build_mesh_tracker(&mesh, &translation_spec(), &opts_fail());
    let mut sc = Scratch::new();
    let mut r = rng("acc06_translation");
    let mut wraps = 0;
    for _ in 0..200 {
        let o = Point3::new(2.0, 4.0 * (r.unit() + 1.0) / 2.0, 4.0 * (r.unit() + 1.0) / 2.0);
        let d = Vector3::new(0.8, 0.36, -0.48);
        let mut st = t.init_state(o, d, None, &mut sc).unwrap();
        for _ in 0..30 {
            let before = st.pos;
            let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
            match s.end {
                StepEnd::Boundary { face: f, action: FaceAction::Wrap { mate, transform }, .. } => {
                    let exit = Point3::new(before.x + d.x * s.dist, before.y + d.y * s.dist, before.z + d.z * s.dist);
                    let image = transform.apply_point(exit);
                    assert!((st.pos - image).norm() < 1e-12, "wrapped position");
                    assert_eq!(st.entered_via, Some(mate), "entered via the mate");
                    assert_ne!(f, mate);
                    assert_eq!(st.dir, d, "translation keeps the direction");
                    wraps += 1;
                    break;
                }
                StepEnd::Boundary { .. } | StepEnd::Lost(_) => break,
                _ => {}
            }
        }
    }
    assert!(wraps > 50, "{wraps}");
}

/// Quarter turn about the vertical axis through (2, 2): +x face <-> +y face of the 4x4 block.
fn rotation_spec() -> (BoundarySpec, Deck) {
    let mut spec = BoundarySpec::default();
    let deck = Deck::rotation(Axis { point: Point3::new(2.0, 2.0, 0.0), dir: Vector3::new(0.0, 0.0, 1.0) }, Turns { num: 1, den: 4 });
    let a = FaceBoundary { boundary: bnd(3), role: FaceRole::Deck { side: DeckSide::A, mate: Some(face(PY)) } };
    let b = FaceBoundary { boundary: bnd(3), role: FaceRole::Deck { side: DeckSide::B, mate: Some(face(PX)) } };
    spec.add_model_face(face(PX), &a, &BoundaryKind::Deck(deck), &ResponseMap::new()).unwrap();
    spec.add_model_face(face(PY), &b, &BoundaryKind::Deck(deck), &ResponseMap::new()).unwrap();
    (spec, deck)
}

#[test]
fn acc06_rotation_wrap_matches_the_analytic_image() {
    let mesh = block(4);
    let (spec, _) = rotation_spec();
    let t = build_mesh_tracker(&mesh, &spec, &opts_fail());
    let mut sc = Scratch::new();
    let mut r = rng("acc06_rotation");
    let mut wraps = 0;
    for _ in 0..300 {
        let o = Point3::new(1.0 + 2.0 * (r.unit() + 1.0) / 2.0, 1.0 + 2.0 * (r.unit() + 1.0) / 2.0, 2.0);
        let (d, _, _) = random_frame(&mut r);
        let d = Vector3::new(d[0], d[1], d[2] * 0.3);
        let d = d * (1.0 / d.norm());
        let mut st = t.init_state(o, d, None, &mut sc).unwrap();
        for _ in 0..40 {
            let before = (st.pos, st.dir);
            let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
            match s.end {
                StepEnd::Boundary { action: FaceAction::Wrap { mate, .. }, face: f, .. } => {
                    let exit = before.0 + before.1 * s.dist;
                    // Analytic quarter turn about (2, 2): (x, y) -> (2 - (y - 2), 2 + (x - 2)) when
                    // leaving +x; the inverse when leaving +y.
                    let (img, dir) = if (exit.x - 4.0).abs() < 1e-9 {
                        (Point3::new(4.0 - exit.y, exit.x, exit.z), Vector3::new(-before.1.y, before.1.x, before.1.z))
                    } else {
                        (Point3::new(exit.y, 4.0 - exit.x, exit.z), Vector3::new(before.1.y, -before.1.x, before.1.z))
                    };
                    assert!((st.pos - img).norm() < 1e-12, "position {:?} vs {:?}", st.pos, img);
                    assert!((st.dir - dir).norm() < 1e-15, "direction {:?} vs {:?}", st.dir, dir);
                    assert_eq!(st.entered_via, Some(mate));
                    assert_ne!(f, mate);
                    wraps += 1;
                    break;
                }
                StepEnd::Boundary { .. } | StepEnd::Lost(_) => break,
                _ => {}
            }
        }
    }
    assert!(wraps > 30, "{wraps}");
}

#[test]
fn acc06_response_boundaries_leave_the_state_on_the_face() {
    let mesh = block(4);
    for kind in [ResponseKind::Vacuum, ResponseKind::White, ResponseKind::Albedo(0.5), ResponseKind::Reflective] {
        let spec = BoundarySpec::new(BoundaryDef::Respond(kind), vec![]);
        let t = build_mesh_tracker(&mesh, &spec, &opts_fail());
        let mut sc = Scratch::new();
        let mut st = t.init_state(Point3::new(2.5, 2.5, 2.5), Vector3::new(0.0, 0.0, 1.0), None, &mut sc).unwrap();
        loop {
            let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
            if let StepEnd::Boundary { face, action, surface } = s.end {
                assert_eq!(action, FaceAction::Respond(kind));
                assert_eq!(st.pos.z, 4.0, "state on the face");
                assert!(st.cell.0 < 64 && t.surfaces().get(surface).is_some());
                let end = t.resolve_boundary(&mut st, face, kind, [0.25, 0.5]).unwrap();
                match kind {
                    ResponseKind::Vacuum => assert_eq!(end, BoundaryAction::Terminate),
                    _ => assert_eq!(end, BoundaryAction::Continue),
                }
                break;
            }
        }
    }
}

#[test]
fn acc06_closed_reflective_box_keeps_every_history_inside() {
    let mesh = block(4);
    // All six faces are mirrors (applied in step).
    let mut spec = BoundarySpec::default();
    for k in 1..=6u64 {
        spec.rules.push((BoundarySelector::Entity(face(k)), BoundaryDef::Reflect));
    }
    let t = build_mesh_tracker(&mesh, &spec, &opts_fail());
    let histories = if cfg!(debug_assertions) { 1_000 } else { 10_000 };
    let steps = if cfg!(debug_assertions) { 1_000 } else { 1_000 };
    let mut sc = Scratch::new();
    let mut r = rng("acc06_reflective_box");
    for h in 0..histories {
        let o = Point3::new(4.0 * (r.unit() + 1.0) / 2.0, 4.0 * (r.unit() + 1.0) / 2.0, 4.0 * (r.unit() + 1.0) / 2.0);
        let (d, _, _) = random_frame(&mut r);
        let mut st = t.init_state(o, v3(d), None, &mut sc).unwrap();
        for k in 0..steps {
            let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
            assert!(!matches!(s.end, StepEnd::Lost(_)), "history {h} step {k} lost: {:?}", s.end);
            assert!(!st.cell.is_none() && st.cell < CellId::new(64));
            for c in [st.pos.x, st.pos.y, st.pos.z] {
                assert!((-1e-9..=4.0 + 1e-9).contains(&c), "history {h} step {k} left the box: {:?}", st.pos);
            }
        }
    }
    assert_eq!(t.nudge_count(), 0);
}

#[test]
fn acc06_response_reflective_box_via_resolve_boundary() {
    let mesh = block(3);
    let spec = BoundarySpec::new(BoundaryDef::Respond(ResponseKind::Reflective), vec![]);
    let t = build_mesh_tracker(&mesh, &spec, &opts_fail());
    let mut sc = Scratch::new();
    let mut r = rng("acc06_resolve_box");
    for _ in 0..200 {
        let (d, _, _) = random_frame(&mut r);
        let mut st = t.init_state(Point3::new(1.5, 1.5, 1.5), v3(d), None, &mut sc).unwrap();
        for _ in 0..200 {
            let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
            match s.end {
                StepEnd::Boundary { face, action: FaceAction::Respond(k), .. } => {
                    assert_eq!(t.resolve_boundary(&mut st, face, k, [0.0, 0.0]).unwrap(), BoundaryAction::Continue);
                }
                StepEnd::Lost(l) => panic!("{l:?}"),
                _ => {}
            }
            assert!((-1e-9..=3.0 + 1e-9).contains(&st.pos.x) && (-1e-9..=3.0 + 1e-9).contains(&st.pos.y) && (-1e-9..=3.0 + 1e-9).contains(&st.pos.z));
        }
    }
}
