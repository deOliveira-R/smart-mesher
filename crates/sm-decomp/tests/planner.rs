//! The planner core on hand-built volume views: exact cut lists, refusals with reasons, determinism.

use sm_core::{EntityId, EntityKind};
use sm_decomp::{
    CutKind, CutStep, DecompError, FaceKind, FaceView, Role, Target, VolumeView, plan_cuts_view,
    render_script,
};
use sm_testkit::strategies::config;
use std::f64::consts::TAU;

const TOL: f64 = 1e-9;

fn id(kind: EntityKind, n: u64) -> EntityId {
    EntityId::new(kind, n).expect("id")
}

fn vol() -> EntityId {
    id(EntityKind::TopoVolume, 5)
}

struct B {
    faces: Vec<FaceView>,
    next: u64,
}

impl B {
    fn new() -> B {
        B {
            faces: Vec::new(),
            next: 1,
        }
    }
    fn push(&mut self, kind: FaceKind, points: Vec<[f64; 3]>, area: Option<f64>) {
        let f = FaceView {
            id: id(EntityKind::TopoFace, self.next),
            kind,
            points,
            area,
        };
        self.next += 1;
        self.faces.push(f);
    }
    /// A box `[-hx, hx] x [-hy, hy] x [z0, z1]` as six plane faces.
    fn brick(mut self, hx: f64, hy: f64, z0: f64, z1: f64) -> B {
        let corners = |z: f64| vec![[-hx, -hy, z], [hx, -hy, z], [hx, hy, z], [-hx, hy, z]];
        self.push(
            FaceKind::Plane {
                origin: [0.0, 0.0, z0],
                normal: [0.0, 0.0, 1.0],
            },
            corners(z0),
            None,
        );
        self.push(
            FaceKind::Plane {
                origin: [0.0, 0.0, z1],
                normal: [0.0, 0.0, 1.0],
            },
            corners(z1),
            None,
        );
        for (n, o, p) in [
            ([1.0, 0.0, 0.0], [hx, 0.0, 0.0], [[hx, -hy], [hx, hy]]),
            ([-1.0, 0.0, 0.0], [-hx, 0.0, 0.0], [[-hx, -hy], [-hx, hy]]),
            ([0.0, 1.0, 0.0], [0.0, hy, 0.0], [[-hx, hy], [hx, hy]]),
            ([0.0, -1.0, 0.0], [0.0, -hy, 0.0], [[-hx, -hy], [hx, -hy]]),
        ] {
            let pts = vec![
                [p[0][0], p[0][1], z0],
                [p[1][0], p[1][1], z0],
                [p[0][0], p[0][1], z1],
                [p[1][0], p[1][1], z1],
            ];
            self.push(
                FaceKind::Plane {
                    origin: o,
                    normal: n,
                },
                pts,
                None,
            );
        }
        self
    }
    /// A cavity wall (one whole cylinder face) from `a` to `b` at `(cx, cy)`.
    fn wall(&mut self, cx: f64, cy: f64, r: f64, a: f64, b: f64) {
        let pts = vec![[cx + r, cy, a], [cx + r, cy, b]];
        self.push(
            FaceKind::Cylinder {
                origin: [cx, cy, 0.0],
                axis: [0.0, 0.0, 1.0],
                radius: r,
                outward: false,
            },
            pts,
            Some(TAU * r * (b - a)),
        );
    }
    /// A disk face closing a cavity at height `z`.
    fn disk(&mut self, cx: f64, cy: f64, r: f64, z: f64) {
        self.push(
            FaceKind::Plane {
                origin: [cx, cy, z],
                normal: [0.0, 0.0, 1.0],
            },
            vec![[cx + r, cy, z]],
            None,
        );
    }
    /// A closed cavity from `a` to `b` strictly inside the host.
    fn enclosed(mut self, cx: f64, cy: f64, r: f64, a: f64, b: f64) -> B {
        self.wall(cx, cy, r, a, b);
        self.disk(cx, cy, r, a);
        self.disk(cx, cy, r, b);
        self
    }
    fn view(self) -> VolumeView {
        VolumeView {
            id: vol(),
            faces: self.faces,
        }
    }
}

fn plane_origin(s: &CutStep) -> [f64; 3] {
    match &s.cut {
        CutKind::Plane { plane } => plane.origin,
        other => panic!("not a plane: {other:?}"),
    }
}

fn is_plane(s: &CutStep) -> bool {
    matches!(s.cut, CutKind::Plane { .. })
}

fn reason(r: Result<Vec<CutStep>, DecompError>) -> String {
    match r {
        Err(DecompError::NoDecomposition { reason, .. }) => reason,
        other => panic!("expected NoDecomposition, got {other:?}"),
    }
}

#[test]
fn enclosed_pin_is_a_cylinder_and_two_planes_in_order() {
    let v = B::new()
        .brick(0.63, 0.63, -0.5, 0.5)
        .enclosed(0.0, 0.0, 0.4, -0.4, 0.4)
        .view();
    let steps = plan_cuts_view(&v, TOL).expect("plan");
    assert_eq!(steps.len(), 3);
    assert_eq!(
        steps[0].cut,
        CutKind::Cylinder {
            radius: 0.4,
            axis: [0.0, 0.0, 1.0],
            center: [0.0, 0.0, 0.0]
        }
    );
    assert_eq!(steps[0].targets, vec![Target::Id { id: vol() }]);
    assert_eq!(plane_origin(&steps[1]), [0.0, 0.0, -0.4]);
    assert_eq!(plane_origin(&steps[2]), [0.0, 0.0, 0.4]);
    // Each later step cuts the positive output of the one before (the outside of the cylinder, then above the plane).
    assert_eq!(
        steps[1].targets,
        vec![Target::Step {
            step: 0,
            role: Role::Positive
        }]
    );
    assert_eq!(
        steps[2].targets,
        vec![Target::Step {
            step: 1,
            role: Role::Positive
        }]
    );
    for (i, s) in steps.iter().enumerate() {
        assert_eq!(s.step, i);
    }
}

#[test]
fn script_text_and_json_of_the_enclosed_pin() {
    let v = B::new()
        .brick(0.63, 0.63, -0.5, 0.5)
        .enclosed(0.0, 0.0, 0.4, -0.4, 0.4)
        .view();
    let steps = plan_cuts_view(&v, TOL).expect("plan");
    let text = render_script(&steps, "Volume#5", "cut");
    assert_eq!(
        text,
        "let cut0 = model.webcut_cylinder(targets: Volume#5, radius: 0.4, axis: [0.0, 0.0, 1.0], center: [0.0, 0.0, 0.0])\n\
         let cut1 = model.webcut_plane(targets: cut0.positive, plane: {origin: [0.0, 0.0, -0.4], normal: [0.0, 0.0, 1.0]})\n\
         let cut2 = model.webcut_plane(targets: cut1.positive, plane: {origin: [0.0, 0.0, 0.4], normal: [0.0, 0.0, 1.0]})\n"
    );
    let j = serde_json::to_value(&steps).expect("json");
    assert_eq!(j[0]["op"], "model.webcut_cylinder");
    assert_eq!(j[0]["targets"][0]["id"], "TopoVolume#5");
    assert_eq!(j[1]["op"], "model.webcut_plane");
    assert_eq!(j[1]["plane"]["normal"], serde_json::json!([0.0, 0.0, 1.0]));
    assert_eq!(
        j[1]["targets"][0],
        serde_json::json!({"step": 0, "role": "positive"})
    );
}

#[test]
fn steps_round_trip_through_json_and_have_a_schema() {
    let v = B::new().brick(1.5, 0.75, -0.5, 0.5).enclosed(0.8, 0.0, 0.3, -0.1, 0.4).view();
    let steps = plan_cuts_view(&v, TOL).expect("plan");
    let text = serde_json::to_string(&steps).expect("json");
    let back: Vec<CutStep> = serde_json::from_str(&text).expect("parse");
    assert_eq!(back, steps);
    assert_eq!(serde_json::to_string(&back).expect("json"), text);
    let schema = serde_json::to_string(&schemars::schema_for!(Vec<CutStep>)).expect("schema");
    assert!(schema.contains("model.webcut_plane") && schema.contains("model.webcut_cylinder"), "{schema}");
    // `PlaneArg` is nameable.
    let _ = sm_decomp::PlaneArg { origin: [0.0; 3], normal: [0.0, 0.0, 1.0] };
}

#[test]
fn volumes_that_already_sweep_get_an_empty_plan() {
    // Brick: no cavity.
    assert!(
        plan_cuts_view(&B::new().brick(0.63, 0.63, -0.5, 0.5).view(), TOL)
            .expect("brick")
            .is_empty()
    );
    // Through hole with a one-piece wall.
    let mut b = B::new().brick(0.63, 0.63, -0.5, 0.5);
    b.wall(0.0, 0.0, 0.4, -0.5, 0.5);
    assert!(plan_cuts_view(&b.view(), TOL).expect("through").is_empty());
    // A convex cylinder (a pin's own fuel wall) is no cavity.
    let mut c = B::new();
    c.push(
        FaceKind::Cylinder {
            origin: [0.0; 3],
            axis: [0.0, 0.0, 1.0],
            radius: 0.4,
            outward: true,
        },
        vec![[0.4, 0.0, -0.4], [0.4, 0.0, 0.4]],
        Some(TAU * 0.4 * 0.8),
    );
    assert!(plan_cuts_view(&c.view(), TOL).expect("fuel").is_empty());
}

#[test]
fn through_hole_with_a_split_wall_needs_one_plane_and_no_cylinder() {
    let mut b = B::new().brick(0.63, 0.63, -0.5, 0.5);
    b.wall(0.0, 0.0, 0.4, -0.5, 0.1);
    b.wall(0.0, 0.0, 0.4, 0.1, 0.5);
    let steps = plan_cuts_view(&b.view(), TOL).expect("plan");
    assert_eq!(steps.len(), 1);
    assert!(is_plane(&steps[0]));
    assert_eq!(plane_origin(&steps[0]), [0.0, 0.0, 0.1]);
    assert_eq!(steps[0].targets, vec![Target::Id { id: vol() }]);
}

#[test]
fn blind_cavities_get_their_end_plane_and_a_cylinder_on_the_solid_slab() {
    // Open at the bottom, closed at 0.2: the plane, then the cylinder on the piece above it.
    let mut b = B::new().brick(0.63, 0.63, -0.5, 0.5);
    b.wall(0.0, 0.0, 0.3, -0.5, 0.2);
    b.disk(0.0, 0.0, 0.3, 0.2);
    let steps = plan_cuts_view(&b.view(), TOL).expect("plan");
    assert_eq!(steps.len(), 2);
    assert_eq!(plane_origin(&steps[0]), [0.0, 0.0, 0.2]);
    assert_eq!(steps[0].targets, vec![Target::Id { id: vol() }]);
    assert!(matches!(steps[1].cut, CutKind::Cylinder { radius, .. } if radius == 0.3));
    assert_eq!(steps[1].targets, vec![Target::Step { step: 0, role: Role::Positive }]);
    // Open at the top, closed at -0.2: the cylinder goes on the piece below the plane.
    let mut t = B::new().brick(0.63, 0.63, -0.5, 0.5);
    t.wall(0.0, 0.0, 0.3, -0.2, 0.5);
    t.disk(0.0, 0.0, 0.3, -0.2);
    let steps = plan_cuts_view(&t.view(), TOL).expect("plan");
    assert_eq!(steps.len(), 2);
    assert_eq!(plane_origin(&steps[0]), [0.0, 0.0, -0.2]);
    assert!(matches!(steps[1].cut, CutKind::Cylinder { .. }));
    assert_eq!(steps[1].targets, vec![Target::Step { step: 0, role: Role::Negative }]);
}

#[test]
fn two_cavities_at_different_heights_and_an_off_axis_one() {
    let v = B::new()
        .brick(1.5, 0.75, -0.5, 0.5)
        .enclosed(0.8, 0.0, 0.3, -0.1, 0.4)
        .enclosed(-0.8, 0.1, 0.4, -0.4, 0.2)
        .view();
    let steps = plan_cuts_view(&v, TOL).expect("plan");
    // Cylinders by centre (x ascending), then the four distinct heights ascending.
    assert_eq!(steps.len(), 6);
    assert!(
        matches!(steps[0].cut, CutKind::Cylinder { radius, center, .. } if radius == 0.4 && center == [-0.8, 0.1, 0.0])
    );
    assert!(
        matches!(steps[1].cut, CutKind::Cylinder { radius, center, .. } if radius == 0.3 && center == [0.8, 0.0, 0.0])
    );
    let z: Vec<f64> = steps[2..].iter().map(|s| plane_origin(s)[2]).collect();
    assert_eq!(z, vec![-0.4, -0.1, 0.2, 0.4]);
}

#[test]
fn a_through_cavity_beside_an_enclosed_one_is_not_cylinder_cut() {
    let mut b = B::new()
        .brick(1.5, 0.75, -0.5, 0.5)
        .enclosed(0.8, 0.0, 0.3, -0.1, 0.4);
    b.wall(-0.8, 0.0, 0.3, -0.5, 0.5);
    let steps = plan_cuts_view(&b.view(), TOL).expect("plan");
    assert_eq!(steps.iter().filter(|s| !is_plane(s)).count(), 1);
}

#[test]
fn unsupported_geometry_is_refused_with_a_reason() {
    // Stepped bore: two radii on one axis.
    let mut b = B::new().brick(0.63, 0.63, -0.5, 0.5);
    b.wall(0.0, 0.0, 0.4, -0.3, 0.2);
    b.wall(0.0, 0.0, 0.25, 0.2, 0.4);
    b.disk(0.0, 0.0, 0.4, -0.3);
    b.disk(0.0, 0.0, 0.25, 0.4);
    assert!(reason(plan_cuts_view(&b.view(), TOL)).contains("overlap or nest"));
    // Overlapping neighbours.
    let v = B::new()
        .brick(1.5, 0.75, -0.5, 0.5)
        .enclosed(0.0, 0.0, 0.4, -0.2, 0.2)
        .enclosed(0.5, 0.0, 0.3, -0.2, 0.2)
        .view();
    assert!(reason(plan_cuts_view(&v, TOL)).contains("overlap or nest"));
    // A cross hole has a different axis.
    let mut x = B::new()
        .brick(0.63, 0.63, -0.5, 0.5)
        .enclosed(0.0, 0.0, 0.2, -0.2, 0.2);
    x.push(
        FaceKind::Cylinder {
            origin: [0.0; 3],
            axis: [1.0, 0.0, 0.0],
            radius: 0.1,
            outward: false,
        },
        vec![[0.0, 0.1, -0.63], [0.0, 0.1, 0.63]],
        None,
    );
    assert!(reason(plan_cuts_view(&x.view(), TOL)).contains("different axis"));
    // A slot: the wall is half a cylinder.
    let mut s = B::new().brick(0.63, 0.63, -0.5, 0.5);
    s.push(
        FaceKind::Cylinder {
            origin: [0.0; 3],
            axis: [0.0, 0.0, 1.0],
            radius: 0.3,
            outward: false,
        },
        vec![[0.3, 0.0, -0.2], [0.3, 0.0, 0.2]],
        Some(0.5 * TAU * 0.3 * 0.4),
    );
    assert!(reason(plan_cuts_view(&s.view(), TOL)).contains("not a whole cylinder"));
    // A sphere face anywhere.
    let mut o = B::new()
        .brick(0.63, 0.63, -0.5, 0.5)
        .enclosed(0.0, 0.0, 0.2, -0.2, 0.2);
    o.push(FaceKind::Other, vec![[0.0, 0.0, 0.0]], None);
    assert!(reason(plan_cuts_view(&o.view(), TOL)).contains("not a plane or a cylinder"));
    // A tilted plane.
    let mut t = B::new()
        .brick(0.63, 0.63, -0.5, 0.5)
        .enclosed(0.0, 0.0, 0.2, -0.2, 0.2);
    let n = 0.5_f64.sqrt();
    t.push(
        FaceKind::Plane {
            origin: [0.0; 3],
            normal: [n, 0.0, n],
        },
        vec![[0.0, 0.0, 0.0]],
        None,
    );
    assert!(reason(plan_cuts_view(&t.view(), TOL)).contains("neither parallel nor normal"));
    // A step face inside the host that closes no cavity.
    let mut p = B::new()
        .brick(0.63, 0.63, -0.5, 0.5)
        .enclosed(0.0, 0.0, 0.2, -0.2, 0.2);
    p.push(
        FaceKind::Plane {
            origin: [0.5, 0.5, 0.3],
            normal: [0.0, 0.0, 1.0],
        },
        vec![[0.5, 0.5, 0.3], [0.6, 0.6, 0.3]],
        None,
    );
    assert!(reason(plan_cuts_view(&p.view(), TOL)).contains("does not close a round cavity"));
}

#[test]
fn the_plan_does_not_depend_on_face_order() {
    let base = B::new()
        .brick(1.5, 0.75, -0.5, 0.5)
        .enclosed(0.8, 0.0, 0.3, -0.1, 0.4)
        .enclosed(-0.8, 0.1, 0.4, -0.4, 0.2)
        .view();
    let want = plan_cuts_view(&base, TOL).expect("plan");
    let mut rev = base.clone();
    rev.faces.reverse();
    assert_eq!(plan_cuts_view(&rev, TOL).expect("plan"), want);
    for k in 1..base.faces.len() {
        let mut rot = base.clone();
        rot.faces.rotate_left(k);
        assert_eq!(
            plan_cuts_view(&rot, TOL).expect("plan"),
            want,
            "rotation {k}"
        );
    }
}

mod prop {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        #![proptest_config(config(64))]

        /// One enclosed cavity at a random place and height: always one cylinder and two ascending planes
        /// at exactly the cavity ends; the same plan for a shuffled face order.
        #[test]
        fn enclosed_cavity_plan_shape(cx in -0.2f64..0.2, cy in -0.2f64..0.2, r in 0.05f64..0.3, a in -0.4f64..0.0, len in 0.1f64..0.7, rot in 0usize..12) {
            let b = a + len;
            prop_assume!(b < 0.45);
            let v = B::new().brick(0.63, 0.63, -0.5, 0.5).enclosed(cx, cy, r, a, b).view();
            let steps = plan_cuts_view(&v, TOL).unwrap();
            prop_assert_eq!(steps.len(), 3);
            let first_ok = matches!(steps[0].cut, CutKind::Cylinder { radius, center, .. } if radius == r && center == [cx, cy, 0.0]);
            prop_assert!(first_ok);
            prop_assert_eq!(plane_origin(&steps[1])[2], a);
            prop_assert_eq!(plane_origin(&steps[2])[2], b);
            let mut w = v.clone();
            let n = w.faces.len();
            w.faces.rotate_left(rot % n);
            prop_assert_eq!(plan_cuts_view(&w, TOL).unwrap(), steps);
        }
    }
}

fn cylinders(steps: &[CutStep]) -> usize {
    steps.iter().filter(|s| !is_plane(s)).count()
}

fn plane_zs(steps: &[CutStep]) -> Vec<f64> {
    steps.iter().filter(|s| is_plane(s)).map(|s| plane_origin(s)[2]).collect()
}

#[test]
fn coaxial_blind_holes_from_both_faces_are_refused_not_merged() {
    // Blind -0.5..-0.2 and 0.2..0.5 on one axis in a -0.5..0.5 host: two cavities, not one through
    // cavity. The disc between them cannot be cut out, so the planner refuses (it never emits a
    // partial cut set).
    let mut b = B::new().brick(0.63, 0.63, -0.5, 0.5);
    b.wall(0.0, 0.0, 0.4, -0.5, -0.2);
    b.disk(0.0, 0.0, 0.4, -0.2);
    b.wall(0.0, 0.0, 0.4, 0.2, 0.5);
    b.disk(0.0, 0.0, 0.4, 0.2);
    let r = reason(plan_cuts_view(&b.view(), TOL));
    assert!(r.contains("stacked on one axis"), "{r}");
    // Enclosed from below and blind from above is the mirror case: refused too.
    let mut m = B::new().brick(0.63, 0.63, -0.5, 0.5).enclosed(0.0, 0.0, 0.4, -0.3, -0.1);
    m.wall(0.0, 0.0, 0.4, 0.3, 0.5);
    m.disk(0.0, 0.0, 0.4, 0.3);
    assert!(reason(plan_cuts_view(&m.view(), TOL)).contains("stacked on one axis"));
}

#[test]
fn coaxial_blind_then_enclosed_cavity_cuts_one_cylinder_above_the_blind_end() {
    let mut b = B::new().brick(0.63, 0.63, -0.5, 0.5).enclosed(0.0, 0.0, 0.4, -0.1, 0.3);
    b.wall(0.0, 0.0, 0.4, -0.5, -0.3);
    b.disk(0.0, 0.0, 0.4, -0.3);
    let steps = plan_cuts_view(&b.view(), TOL).expect("plan");
    // Plane at the blind end, one cylinder on the slab above it, then the enclosed run's planes on the ring.
    assert_eq!(steps.len(), 4);
    assert_eq!(plane_zs(&steps), vec![-0.3, -0.1, 0.3]);
    assert_eq!(cylinders(&steps), 1);
    assert!(matches!(steps[1].cut, CutKind::Cylinder { .. }));
    assert_eq!(steps[1].targets, vec![Target::Step { step: 0, role: Role::Positive }]);
    assert_eq!(steps[2].targets, vec![Target::Step { step: 1, role: Role::Positive }]);
}

#[test]
fn coaxial_enclosed_cavities_with_a_gap_and_touching_walls() {
    let two = B::new()
        .brick(0.63, 0.63, -0.5, 0.5)
        .enclosed(0.0, 0.0, 0.4, -0.3, -0.1)
        .enclosed(0.0, 0.0, 0.4, 0.1, 0.3);
    let steps = plan_cuts_view(&two.view(), TOL).expect("plan");
    assert_eq!(cylinders(&steps), 1);
    assert_eq!(steps.len(), 5);
    // Touching walls are one run.
    let mut t = B::new().brick(0.63, 0.63, -0.5, 0.5);
    t.wall(0.0, 0.0, 0.4, -0.3, 0.0);
    t.wall(0.0, 0.0, 0.4, 0.0, 0.3);
    t.disk(0.0, 0.0, 0.4, -0.3);
    t.disk(0.0, 0.0, 0.4, 0.3);
    let steps = plan_cuts_view(&t.view(), TOL).expect("plan");
    assert_eq!(cylinders(&steps), 1);
}
