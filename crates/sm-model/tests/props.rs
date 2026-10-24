//! Property tests: random solids under random chains of similarity transforms keep a valid,
//! outward-oriented model on both topology hosts, with the volume scaling as `f^3`.

mod common;

use common::*;
use proptest::prelude::*;
use sm_core::EntityKind;
use sm_model::{CreateBrick, CreateCylinder, Move, Reflect, Rotate, Scale};
use sm_op_api::{Op, Session};
use sm_topo_api::Dim;

#[derive(Clone, Debug)]
enum Step {
    Move([f64; 3]),
    Rotate([f64; 3], f64),
    Scale(f64),
    Reflect([f64; 3]),
}

fn vec3() -> impl Strategy<Value = [f64; 3]> {
    [-5.0f64..5.0, -5.0f64..5.0, -5.0f64..5.0]
}

fn axis() -> impl Strategy<Value = [f64; 3]> {
    // Never near zero: one component is forced to magnitude >= 1.
    (vec3(), 0usize..3, prop_oneof![Just(1.0f64), Just(-1.0)]).prop_map(|(mut v, k, s)| {
        v[k] = s * (1.0 + v[k].abs());
        v
    })
}

fn step() -> impl Strategy<Value = Step> {
    prop_oneof![
        vec3().prop_map(Step::Move),
        (axis(), -180.0f64..180.0).prop_map(|(a, t)| Step::Rotate(a, t)),
        (0.5f64..2.0).prop_map(Step::Scale),
        axis().prop_map(Step::Reflect),
    ]
}

fn solid() -> impl Strategy<Value = (u8, [f64; 3], [f64; 3])> {
    (0u8..2, [0.5f64..6.0, 0.5f64..6.0, 0.5f64..6.0], vec3())
}

fn op_of(step: &Step) -> Box<dyn Op> {
    let e = vec![id(EntityKind::TopoVolume, 1)];
    match step {
        Step::Move(d) => Box::new(Move { entities: e, delta: *d }),
        Step::Rotate(a, t) => Box::new(Rotate { entities: e, axis: *a, angle_deg: *t, origin: Some([1.0, -1.0, 0.5]) }),
        Step::Scale(f) => Box::new(Scale { entities: e, factor: *f, origin: Some([0.5, 0.5, 0.5]) }),
        Step::Reflect(n) => Box::new(Reflect { entities: e, normal: *n, origin: Some([0.25, 0.0, -1.0]) }),
    }
}

fn volume(s: &Session) -> f64 {
    let t = model(s).topo();
    t.volume_moments(t.cells(Dim::Volume)[0]).expect("moments").volume
}

proptest! {
    #![proptest_config(sm_testkit::strategies::config(48))]

    #[test]
    fn chains_of_transforms_keep_the_model_valid_and_agree_across_hosts((kind, size, center) in solid(), steps in proptest::collection::vec(step(), 1..5)) {
        let create: Box<dyn Op> = if kind == 0 {
            Box::new(CreateBrick { x: size[0], y: Some(size[1]), z: Some(size[2]), center: Some(center) })
        } else {
            Box::new(CreateCylinder { height: size[0], radius: 0.4 * size[1], center: Some(center) })
        };
        let mut sessions = [session(cell_complex), session(topo_store)];
        let mut want = {
            let mut s = session(cell_complex);
            run(&mut s, create.as_ref()).expect("create");
            volume(&s)
        };
        for s in &mut sessions {
            run(s, create.as_ref()).expect("create");
        }
        for st in &steps {
            if let Step::Scale(f) = st {
                want *= f * f * f;
            }
            let op = op_of(st);
            for s in &mut sessions {
                run(s, op.as_ref()).expect("transform");
                let v = volume(s);
                prop_assert!((v - want).abs() <= 1e-8 * want.abs().max(1.0), "volume {} vs {} after {:?}", v, want, st);
                prop_assert!(worst_outwardness(s) > 0.0, "outward after {:?}", st);
                prop_assert_eq!(issues(s), Vec::<String>::new(), "validate after {:?}", st);
            }
            prop_assert_eq!(
                sessions[0].state().layer_hash("model").expect("hash"),
                sessions[1].state().layer_hash("model").expect("hash"),
                "hosts agree after {:?}", st
            );
        }
    }
}
