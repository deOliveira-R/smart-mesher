//! W3b-PRIM acceptance 5: `model.create_sphere_part`.

mod common;

use common::*;
use core::f64::consts::PI;
use sm_model::{CreateSphere, CreateSpherePart, HalfSpace, Octants};

fn part(radius: f64, inner: Option<f64>, x: Option<HalfSpace>, y: Option<HalfSpace>, z: Option<HalfSpace>) -> CreateSpherePart {
    CreateSpherePart { radius, inner, octants: Some(Octants { x, y, z }), center: None }
}

const SIGNS: [Option<HalfSpace>; 3] = [None, Some(HalfSpace::Positive), Some(HalfSpace::Negative)];

/// `[V, E, F]` expected without a cavity for `k = (x cut, y cut, z cut)`.
fn expected_counts(x: bool, y: bool, z: bool) -> [usize; 3] {
    match (x, y, z) {
        (false, false, false) => [2, 3, 1],
        (false, false, true) => [2, 3, 2],
        (true, false, false) | (false, true, false) => [2, 4, 2],
        (true, true, false) => [2, 5, 3],
        (true, false, true) | (false, true, true) => [3, 5, 3],
        (true, true, true) => [4, 7, 4],
    }
}

#[test]
fn hemisphere_quarter_and_octant_closed_forms() {
    use HalfSpace::Positive as P;
    both(|factory| {
        let r: f64 = 2.0;
        let cases = [
            (part(r, None, None, None, Some(P)), 2.0 * PI * r.powi(3) / 3.0, 3.0 * PI * r * r),
            (part(r, None, Some(P), Some(P), None), PI * r.powi(3) / 3.0, 2.0 * PI * r * r),
            (part(r, None, Some(P), Some(P), Some(P)), PI * r.powi(3) / 6.0, 5.0 * PI * r * r / 4.0),
        ];
        for (op, v, a) in cases {
            let mut s = session(factory);
            run(&mut s, &op).expect("part");
            assert!(near(volumes(&s)[0], v, 1e-12), "{op:?}: {} vs {v}", volumes(&s)[0]);
            assert!(near(total_area(&s), a, 1e-12), "{op:?}: {} vs {a}", total_area(&s));
            clean(&s);
        }
    });
}

#[test]
fn all_26_cut_sets_are_clean_with_the_listed_counts() {
    both(|factory| {
        let mut octant_sum = 0.0;
        let mut n = 0;
        for x in SIGNS {
            for y in SIGNS {
                for z in SIGNS {
                    if x.is_none() && y.is_none() && z.is_none() {
                        continue;
                    }
                    n += 1;
                    let (r, k) = (2.0f64, [x, y, z].iter().flatten().count());
                    let mut s = session(factory);
                    run(&mut s, &part(r, None, x, y, z)).unwrap_or_else(|e| panic!("{x:?}{y:?}{z:?}: {e:?}"));
                    let want = expected_counts(x.is_some(), y.is_some(), z.is_some());
                    assert_eq!(&counts(&s)[..3], &want, "{x:?}{y:?}{z:?}");
                    assert_eq!(&counts(&s)[3..], &[1, 1]);
                    assert_eq!(euler_with_holes(&s), 2, "{x:?}{y:?}{z:?}");
                    clean(&s);
                    let v = volumes(&s)[0];
                    let want_v = 4.0 / 3.0 * PI * r.powi(3) / f64::from(1u32 << k);
                    assert!(near(v, want_v, 1e-12), "{x:?}{y:?}{z:?}: {v} vs {want_v}");
                    assert!(worst_outwardness(&s) > 0.0, "{x:?}{y:?}{z:?}");
                    if k == 3 {
                        octant_sum += v;
                    }
                }
            }
        }
        assert_eq!(n, 26);
        assert!(near(octant_sum, 4.0 / 3.0 * PI * 8.0, 1e-12), "8 octants sum to the ball: {octant_sum}");
    });
}

#[test]
fn octants_sit_in_their_own_octant() {
    both(|factory| {
        for x in &SIGNS[1..] {
            for y in &SIGNS[1..] {
                for z in &SIGNS[1..] {
                    let mut s = session(factory);
                    let mut op = part(2.0, None, *x, *y, *z);
                    op.center = Some([1.0, -2.0, 3.0]);
                    run(&mut s, &op).expect("octant");
                    let sg = |h: &Option<HalfSpace>| if *h == Some(HalfSpace::Positive) { 1.0 } else { -1.0 };
                    let signs = [sg(x), sg(y), sg(z)];
                    let c = [1.0, -2.0, 3.0];
                    for p in vertex_points(&s) {
                        for k in 0..3 {
                            assert!(signs[k] * (p[k] - c[k]) >= -1e-12, "vertex {p:?} outside its octant {signs:?}");
                        }
                    }
                    // The centroid of an octant lies strictly inside it.
                    let t = model(&s).topo();
                    use sm_topo_api::TopologyViewExt;
                    let cen = t.volume_moments(t.volumes()[0]).expect("moments").centroid().expect("centroid");
                    for (k, v) in [cen.x, cen.y, cen.z].iter().enumerate() {
                        assert!(signs[k] * (v - c[k]) > 0.1, "centroid {cen:?} for {signs:?}");
                    }
                }
            }
        }
    });
}

#[test]
fn hollow_sphere_and_hollow_octant() {
    use HalfSpace::Positive as P;
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &CreateSpherePart { radius: 2.0, inner: Some(1.0), octants: None, center: None }).expect("hollow");
        assert_eq!(counts(&s), vec![4, 6, 2, 1, 1]);
        assert_eq!(euler_with_holes(&s), 4);
        assert!(near(volumes(&s)[0], 28.0 * PI / 3.0, 1e-12), "{}", volumes(&s)[0]);
        assert!(near(total_area(&s), 20.0 * PI, 1e-12), "{}", total_area(&s));
        clean(&s);

        let mut s = session(factory);
        run(&mut s, &part(2.0, Some(1.0), Some(P), Some(P), Some(P))).expect("hollow octant");
        assert_eq!(euler_with_holes(&s), 2);
        assert_eq!(sm_topo_api::euler_characteristic(model(&s).topo()), 2);
        assert!(near(volumes(&s)[0], 7.0 * PI / 6.0, 1e-12), "{}", volumes(&s)[0]);
        assert!(near(total_area(&s), 19.0 * PI / 4.0, 1e-12), "{}", total_area(&s));
        clean(&s);
        assert!(worst_outwardness(&s) > 0.0);
    });
}

#[test]
fn every_hollow_cut_set_is_clean_with_analytic_volume() {
    both(|factory| {
        for x in SIGNS {
            for y in SIGNS {
                for z in SIGNS {
                    let k = [x, y, z].iter().flatten().count();
                    let mut s = session(factory);
                    run(&mut s, &part(2.0, Some(1.0), x, y, z)).unwrap_or_else(|e| panic!("{x:?}{y:?}{z:?}: {e:?}"));
                    clean(&s);
                    let want = 28.0 * PI / 3.0 / f64::from(1u32 << k);
                    assert!(near(volumes(&s)[0], want, 1e-12), "{x:?}{y:?}{z:?}: {} vs {want}", volumes(&s)[0]);
                    // Each cut plane leaves an annulus (area 3 pi) cut again by the other k - 1 planes.
                    let area = 20.0 * PI / f64::from(1u32 << k) + if k == 0 { 0.0 } else { k as f64 * 3.0 * PI / f64::from(1u32 << (k - 1)) };
                    assert!(near(total_area(&s), area, 1e-12), "{x:?}{y:?}{z:?}: {} vs {area}", total_area(&s));
                    if k > 0 {
                        assert_eq!(euler_with_holes(&s), 2, "{x:?}{y:?}{z:?}");
                        assert!(worst_outwardness(&s) > 0.0, "{x:?}{y:?}{z:?}");
                    }
                }
            }
        }
    });
}

#[test]
fn invalid_sphere_parts_are_refused_and_leave_the_state_alone() {
    let mut s = session(cell_complex);
    let before = s.state().hash();
    for op in [
        CreateSpherePart { radius: 2.0, inner: Some(2.0), octants: None, center: None },
        CreateSpherePart { radius: 2.0, inner: Some(3.0), octants: None, center: None },
        CreateSpherePart { radius: 2.0, inner: Some(0.0), octants: None, center: None },
        CreateSpherePart { radius: 2.0, inner: Some(1.999_999_9), octants: None, center: None },
        CreateSpherePart { radius: -1.0, inner: None, octants: None, center: None },
        CreateSpherePart { radius: f64::MAX, inner: None, octants: None, center: Some([f64::MAX, 0.0, 0.0]) },
    ] {
        let e = run(&mut s, &op).expect_err("refused");
        assert!(format!("{e:?}").contains("Invalid"), "{op:?}: {e:?}");
        assert_eq!(s.state().hash(), before);
    }
    // NaN never reaches `apply`: it is refused when the op is encoded.
    assert!(run(&mut s, &CreateSpherePart { radius: 2.0, inner: Some(f64::NAN), octants: None, center: None }).is_err());
    assert_eq!(s.state().hash(), before);
}

#[test]
fn plain_sphere_part_has_the_create_sphere_hash() {
    both(|factory| {
        let mut a = session(factory);
        run(&mut a, &CreateSpherePart { radius: 2.0, inner: None, octants: None, center: Some([1.0, 2.0, 3.0]) }).expect("part");
        let mut b = session(factory);
        run(&mut b, &CreateSphere { radius: 2.0, center: Some([1.0, 2.0, 3.0]) }).expect("sphere");
        assert_eq!(model(&a).topo().canonical_hash(), model(&b).topo().canonical_hash());
        assert_eq!(a.state().hash(), b.state().hash());
        let mut c = session(factory);
        run(&mut c, &CreateSpherePart { radius: 2.0, inner: None, octants: Some(Octants::default()), center: Some([1.0, 2.0, 3.0]) }).expect("empty octants");
        assert_eq!(c.state().hash(), b.state().hash());
    });
}
