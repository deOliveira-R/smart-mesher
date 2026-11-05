//! W3b-PRIM acceptance 9: property tests (fixed seed through `sm_testkit::strategies::config`).
//! Random valid sizes give clean models with the analytic volume; invalid sizes are refused and
//! leave the state hash unchanged.

mod common;

use common::*;
use core::f64::consts::{PI, TAU};
use proptest::prelude::*;
use sm_model::{CreateFrustum, CreatePrism, CreatePyramid, CreateSpherePart, CreateTorus, HalfSpace, Octants};
use sm_op_api::Op;

fn half() -> impl Strategy<Value = Option<HalfSpace>> {
    prop_oneof![Just(None), Just(Some(HalfSpace::Positive)), Just(Some(HalfSpace::Negative))]
}

proptest! {
    #![proptest_config(sm_testkit::strategies::config(24))]

    #[test]
    fn frustum_sizes(r in 0.5f64..5.0, b in 0.5f64..5.0, t in 0.0f64..5.0, h in 0.5f64..5.0, centre in [-3.0f64..3.0, -3.0f64..3.0, -3.0f64..3.0]) {
        prop_assume!((r - t).abs() > 1e-3);
        let mut s = session(topo_store);
        run(&mut s, &CreateFrustum { height: h, radius: r, minor: Some(b), top: Some(t), center: Some(centre) }).expect("frustum");
        clean(&s);
        let want = PI * b * h * (r * r + r * t + t * t) / (3.0 * r);
        prop_assert!(near(volumes(&s)[0], want, 1e-11), "{} vs {}", volumes(&s)[0], want);
        prop_assert!(worst_outwardness(&s) > 0.0);
    }

    #[test]
    fn prism_and_pyramid_sizes(n in 3u32..=12, r in 0.5f64..5.0, b in 0.5f64..5.0, t in 0.1f64..5.0, h in 0.5f64..5.0) {
        prop_assume!((r - t).abs() > 1e-3);
        let k = f64::from(n) / 2.0 * (TAU / f64::from(n)).sin();
        let mut s = session(topo_store);
        run(&mut s, &CreatePrism { height: h, sides: n, radius: r, minor: Some(b), center: None }).expect("prism");
        clean(&s);
        prop_assert!(near(volumes(&s)[0], h * k * r * b, 1e-11));
        prop_assert!(worst_outwardness(&s) > 0.0);
        let mut s = session(topo_store);
        run(&mut s, &CreatePyramid { height: h, sides: n, radius: r, top: Some(t), minor: Some(b), center: None }).expect("pyramid");
        clean(&s);
        // h/3 (A1 + A2 + sqrt(A1 A2)) with A1 = k r b and A2 = k t^2 b / r.
        let want = h / 3.0 * k * b * (r + t * t / r + t);
        prop_assert!(near(volumes(&s)[0], want, 1e-11), "{} vs {want}", volumes(&s)[0]);
        prop_assert!(worst_outwardness(&s) > 0.0);
        let mut s = session(topo_store);
        run(&mut s, &CreatePyramid { height: h, sides: n, radius: r, top: None, minor: Some(b), center: None }).expect("apex");
        clean(&s);
        prop_assert!(near(volumes(&s)[0], h / 3.0 * k * r * b, 1e-11));
    }

    #[test]
    fn torus_sizes(big in 1.5f64..8.0, f in 0.1f64..0.9) {
        let minor = (big - 0.2) * f;
        let mut s = session(topo_store);
        run(&mut s, &CreateTorus { major: big, minor, center: None }).expect("torus");
        clean(&s);
        prop_assert!(near(volumes(&s)[0], 2.0 * PI * PI * big * minor * minor, 1e-11));
    }

    #[test]
    fn sphere_part_sizes(r in 0.5f64..5.0, inner in prop_oneof![Just(None), (0.1f64..0.9).prop_map(Some)], x in half(), y in half(), z in half()) {
        let ri = inner.map(|f| f * r);
        let mut s = session(topo_store);
        let op = CreateSpherePart { radius: r, inner: ri, octants: Some(Octants { x, y, z }), center: Some([0.5, -0.5, 1.0]) };
        run(&mut s, &op).expect("sphere part");
        clean(&s);
        let k = [x, y, z].iter().flatten().count() as i32;
        let want = 4.0 / 3.0 * PI * (r.powi(3) - ri.unwrap_or(0.0).powi(3)) / 2f64.powi(k);
        prop_assert!(near(volumes(&s)[0], want, 1e-10), "{} vs {want}", volumes(&s)[0]);
        if k > 0 {
            prop_assert!(worst_outwardness(&s) > 0.0);
        }
    }

    #[test]
    fn invalid_sizes_are_refused_and_leave_the_hash_alone(bad in prop_oneof![Just(0.0f64), Just(-1.0), Just(-1e-3), Just(1e-7), Just(5e-324), Just(f64::NAN), Just(f64::INFINITY)], which in 0usize..9) {
        let mut s = session(cell_complex);
        let before = s.state().hash();
        let ops: Vec<Box<dyn Op>> = vec![
            Box::new(CreateFrustum { height: bad, radius: 1.0, minor: None, top: Some(0.5), center: None }),
            Box::new(CreateFrustum { height: 1.0, radius: bad, minor: None, top: Some(0.5), center: None }),
            Box::new(CreateFrustum { height: 1.0, radius: 1.0, minor: Some(bad), top: None, center: None }),
            Box::new(CreatePrism { height: bad, sides: 5, radius: 1.0, minor: None, center: None }),
            Box::new(CreatePrism { height: 1.0, sides: 5, radius: bad, minor: None, center: None }),
            Box::new(CreatePyramid { height: 1.0, sides: 5, radius: 1.0, top: None, minor: Some(bad), center: None }),
            Box::new(CreateTorus { major: bad, minor: 0.5, center: None }),
            Box::new(CreateTorus { major: 2.0, minor: bad, center: None }),
            Box::new(CreateSpherePart { radius: bad, inner: None, octants: None, center: None }),
        ];
        let e = run(&mut s, ops[which].as_ref());
        prop_assert!(e.is_err(), "{:?}", ops[which]);
        if bad.is_finite() {
            prop_assert!(format!("{e:?}").contains("Invalid"), "{e:?}");
        }
        prop_assert_eq!(s.state().hash(), before);
    }
}

#[test]
fn overflowing_sizes_are_invalid_and_leave_the_state_alone() {
    let m = f64::MAX;
    let mut s = session(cell_complex);
    let before = s.state().hash();
    let ops: Vec<Box<dyn Op>> = vec![
        Box::new(CreateFrustum { height: 1.0, radius: m, minor: None, top: Some(1.0), center: None }),
        Box::new(CreateFrustum { height: m, radius: m, minor: Some(1.0), top: None, center: None }),
        Box::new(CreateFrustum { height: 1.0, radius: 1.0, minor: None, top: Some(1.0), center: Some([m, 0.0, 0.0]) }),
        Box::new(CreateFrustum { height: 1.0, radius: 1.0, minor: None, top: Some(1e-300), center: None }),
        Box::new(CreatePrism { height: 1.0, sides: 4, radius: m, minor: None, center: None }),
        Box::new(CreatePrism { height: m, sides: 4, radius: 1.0, minor: None, center: None }),
        Box::new(CreatePyramid { height: 1.0, sides: 4, radius: 1.0, top: Some(m), minor: Some(m), center: None }),
        Box::new(CreateTorus { major: m, minor: 1.0, center: Some([m, 0.0, 0.0]) }),
        Box::new(CreateSpherePart { radius: m, inner: None, octants: Some(Octants { x: Some(HalfSpace::Positive), y: None, z: None }), center: None }),
        Box::new(CreateSpherePart { radius: 1.0, inner: Some(1e-300), octants: None, center: None }),
    ];
    for op in &ops {
        let e = run(&mut s, op.as_ref()).expect_err("refused");
        assert!(format!("{e:?}").contains("Invalid"), "{op:?}: {e:?}");
        assert_eq!(s.state().hash(), before, "{op:?}");
    }
}
