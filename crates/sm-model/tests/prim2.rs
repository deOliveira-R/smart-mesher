//! W3b-PRIM acceptance 1-4 (frustum, prism, pyramid, torus) on both topology hosts.

mod common;

use common::*;
use core::f64::consts::PI;
use sm_model::{CreateCylinder, CreateFrustum, CreatePrism, CreatePyramid, CreateTorus};
use sm_topo_api::TopologyView;

fn frustum(height: f64, radius: f64, minor: Option<f64>, top: Option<f64>, center: Option<[f64; 3]>) -> CreateFrustum {
    CreateFrustum { height, radius, minor, top, center }
}

#[test]
fn frustum_counts_volume_area_and_inversion() {
    both(|factory| {
        let (r, t, h) = (3.0, 1.0, 4.0);
        let want_v = PI * h * (r * r + r * t + t * t) / 3.0;
        let want_a = PI * (r + t) * (h * h + (r - t) * (r - t)).sqrt() + PI * (r * r + t * t);
        for (rr, tt) in [(r, t), (t, r)] {
            let mut s = session(factory);
            run(&mut s, &frustum(h, rr, None, Some(tt), None)).expect("frustum");
            assert_eq!(counts(&s), vec![2, 3, 3, 1, 1]);
            assert_eq!(euler_with_holes(&s), 2);
            assert!(near(volumes(&s)[0], want_v, 1e-12), "{} vs {want_v}", volumes(&s)[0]);
            assert!(near(total_area(&s), want_a, 1e-12), "{} vs {want_a}", total_area(&s));
            clean(&s);
            assert!(worst_outwardness(&s) > 0.0);
        }
    });
}

#[test]
fn frustum_with_top_equal_to_radius_is_the_cylinder() {
    both(|factory| {
        let mut a = session(factory);
        run(&mut a, &frustum(4.0, 1.5, None, Some(1.5), Some([1.0, 2.0, 3.0]))).expect("frustum");
        let mut b = session(factory);
        run(&mut b, &CreateCylinder { height: 4.0, radius: 1.5, center: Some([1.0, 2.0, 3.0]) }).expect("cylinder");
        assert_eq!(model(&a).topo().canonical_hash(), model(&b).topo().canonical_hash());
    });
}

#[test]
fn frustum_apex_is_the_cone_fixture() {
    let want = sm_testkit::topo::cone(2.0, 3.0).expect("fixture").complex.canonical_hash();
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &frustum(3.0, 2.0, None, None, Some([0.0, 0.0, 1.5]))).expect("cone");
        assert_eq!(model(&s).topo().canonical_hash(), want);
        assert_eq!(counts(&s), vec![2, 3, 2, 1, 1]);
        assert!(near(volumes(&s)[0], 4.0 * PI, 1e-12));
        let lateral = 2.0 * PI * 13f64.sqrt();
        assert!(near(total_area(&s), lateral + 4.0 * PI, 1e-12), "{}", total_area(&s));
        clean(&s);
    });
}

/// Lateral area of the elliptic frustum: `|p_u x p_s|` for `p(u, s) = (A(s) cos u, B(s) sin u,
/// h s)` by a periodic trapezoid in `u` (4096 points) and composite Simpson in `s`.
fn elliptic_area(a: f64, b: f64, t: f64, h: f64) -> f64 {
    let n = 4096;
    let mut total = 0.0;
    for i in 0..n {
        let u = core::f64::consts::TAU * i as f64 / n as f64;
        let f = |s: f64| {
            let (ka, kb) = (a + (t - a) * s, (b / a) * (a + (t - a) * s));
            let (da, db) = (t - a, (b / a) * (t - a));
            let pu = [-ka * u.sin(), kb * u.cos(), 0.0];
            let ps = [da * u.cos(), db * u.sin(), h];
            let c = [pu[1] * ps[2] - pu[2] * ps[1], pu[2] * ps[0] - pu[0] * ps[2], pu[0] * ps[1] - pu[1] * ps[0]];
            (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt()
        };
        let m = 200;
        let mut acc = 0.0;
        for k in 0..m {
            let (s0, s1) = (k as f64 / m as f64, (k + 1) as f64 / m as f64);
            acc += (s1 - s0) / 6.0 * (f(s0) + 4.0 * f(0.5 * (s0 + s1)) + f(s1));
        }
        total += acc;
    }
    total * core::f64::consts::TAU / n as f64
}

#[test]
fn elliptic_frustum_volume_and_area() {
    both(|factory| {
        let (a, b, t, h) = (3.0, 2.0, 1.5, 4.0);
        let mut s = session(factory);
        run(&mut s, &frustum(h, a, Some(b), Some(t), None)).expect("elliptic frustum");
        assert_eq!(counts(&s), vec![2, 3, 3, 1, 1]);
        clean(&s);
        let want_v = PI * h * b * (a * a + a * t + t * t) / (3.0 * a);
        assert!(near(want_v, 14.0 * PI, 1e-14));
        assert!(near(volumes(&s)[0], want_v, 1e-12), "{} vs {want_v}", volumes(&s)[0]);
        let top_b = t * b / a;
        let want_a = elliptic_area(a, b, t, h) + PI * a * b + PI * t * top_b;
        assert!(near(total_area(&s), want_a, 1e-9), "{} vs {want_a}", total_area(&s));
        assert!(worst_outwardness(&s) > 0.0);
        for p in vertex_points(&s) {
            let (ka, kb) = if p[2] < 0.0 { (a, b) } else { (t, top_b) };
            assert!(((p[0] / ka).powi(2) + (p[1] / kb).powi(2) - 1.0).abs() < 1e-14, "{p:?}");
        }
        let mut s = session(factory);
        run(&mut s, &frustum(h, a, Some(b), None, None)).expect("elliptic cone");
        assert_eq!(counts(&s), vec![2, 3, 2, 1, 1]);
        assert!(near(volumes(&s)[0], PI * a * b * h / 3.0, 1e-12), "{}", volumes(&s)[0]);
        clean(&s);
        let mut s = session(factory);
        run(&mut s, &frustum(h, a, Some(b), Some(a), None)).expect("elliptic cylinder");
        assert!(near(volumes(&s)[0], PI * a * b * h, 1e-12));
        clean(&s);
    });
}

#[test]
fn inverted_elliptic_frustum_is_clean() {
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &frustum(2.0, 1.0, Some(0.5), Some(2.0), Some([1.0, -1.0, 0.5]))).expect("inverted");
        clean(&s);
        let (a, t, h) = (1.0, 2.0, 2.0);
        let want = PI * h * 0.5 * (a * a + a * t + t * t) / (3.0 * a);
        assert!(near(volumes(&s)[0], want, 1e-12));
        assert!(worst_outwardness(&s) > 0.0);
    });
}

#[test]
fn prism_and_pyramid_analytic() {
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &CreatePrism { height: 3.0, sides: 6, radius: 2.0, minor: None, center: None }).expect("prism");
        assert_eq!(counts(&s), vec![12, 18, 8, 1, 1]);
        assert!(near(volumes(&s)[0], 18.0 * 3f64.sqrt(), 1e-12));
        assert!(near(total_area(&s), 36.0 + 12.0 * 3f64.sqrt(), 1e-12));
        clean(&s);
        assert!(worst_outwardness(&s) > 0.0);
        assert_eq!(vertex_points(&s)[0], [2.0, 0.0, -1.5]);

        let mut s = session(factory);
        run(&mut s, &CreatePrism { height: 1.0, sides: 4, radius: 2.0, minor: Some(1.0), center: None }).expect("elliptic prism");
        assert!(near(volumes(&s)[0], 4.0, 1e-12), "{}", volumes(&s)[0]);
        clean(&s);

        let r = 2f64.sqrt();
        let mut s = session(factory);
        run(&mut s, &CreatePyramid { height: 3.0, sides: 4, radius: r, top: None, minor: None, center: None }).expect("pyramid");
        assert_eq!(counts(&s), vec![5, 8, 5, 1, 1]);
        assert!(near(volumes(&s)[0], 4.0, 1e-12));
        assert!(near(total_area(&s), 4.0 + 4.0 * 10f64.sqrt(), 1e-12));
        clean(&s);
        assert!(worst_outwardness(&s) > 0.0);

        let mut s = session(factory);
        run(&mut s, &CreatePyramid { height: 3.0, sides: 4, radius: r, top: Some(r / 2.0), minor: None, center: None }).expect("truncated");
        assert_eq!(counts(&s), vec![8, 12, 6, 1, 1]);
        assert!(near(volumes(&s)[0], 7.0, 1e-12), "{}", volumes(&s)[0]);
        clean(&s);
        assert!(worst_outwardness(&s) > 0.0);

        let mut s = session(factory);
        run(&mut s, &CreatePyramid { height: 3.0, sides: 4, radius: r / 2.0, top: Some(r), minor: None, center: None }).expect("inverted");
        assert!(near(volumes(&s)[0], 7.0, 1e-12));
        clean(&s);
        assert!(worst_outwardness(&s) > 0.0);
    });
}

#[test]
fn polygon_sizes_are_validated() {
    let mut s = session(cell_complex);
    let before = s.state().hash();
    for sides in [0, 2, 1025, u32::MAX] {
        let e = run(&mut s, &CreatePrism { height: 1.0, sides, radius: 1.0, minor: None, center: None }).expect_err("sides");
        assert!(format!("{e:?}").contains("Invalid"), "{e:?}");
    }
    let e = run(&mut s, &CreatePyramid { height: 1.0, sides: 4, radius: 1.0, top: Some(1.0), minor: None, center: None }).expect_err("top == radius");
    assert!(format!("{e:?}").contains("create_prism"), "{e:?}");
    assert_eq!(s.state().hash(), before);
    run(&mut s, &CreatePrism { height: 1.0, sides: 3, radius: 1.0, minor: None, center: None }).expect("3");
    run(&mut s, &CreatePrism { height: 1.0, sides: 1024, radius: 1.0, minor: None, center: None }).expect("1024");
}

#[test]
fn torus_matches_the_fixture_and_closed_forms() {
    let want = sm_testkit::topo::torus(5.0, 1.0).expect("fixture").complex.canonical_hash();
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &CreateTorus { major: 5.0, minor: 1.0, center: None }).expect("torus");
        assert_eq!(counts(&s), vec![1, 2, 1, 1, 1]);
        assert_eq!(euler_with_holes(&s), 0);
        assert!(near(volumes(&s)[0], 10.0 * PI * PI, 1e-12), "{}", volumes(&s)[0]);
        assert!(near(total_area(&s), 20.0 * PI * PI, 1e-12));
        assert_eq!(model(&s).topo().canonical_hash(), want);
        clean(&s);
        for (major, minor) in [(1.0, 1.0), (1.0, 2.0)] {
            let before = s.state().hash();
            let e = run(&mut s, &CreateTorus { major, minor, center: None }).expect_err("spindle");
            assert!(format!("{e:?}").contains("Invalid"));
            assert_eq!(s.state().hash(), before);
        }
        let mut s = session(factory);
        run(&mut s, &CreateTorus { major: 5.0, minor: 1.0, center: Some([1.0, 2.0, 3.0]) }).expect("shifted");
        assert!(near(volumes(&s)[0], 10.0 * PI * PI, 1e-12));
        clean(&s);
    });
}
