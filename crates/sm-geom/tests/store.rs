//! `KernelStore` against the reference `MemoryStore` (differential), plus the testkit's analytic
//! fixtures resolved through the store.

use core::f64::consts::TAU;
use sm_core::{Point3, Vector3};
use sm_geom::{KernelStore, exact};
use sm_geom_api::{
    Circle, Cone, CurveDef, Cylinder, Ellipse, EllipticCone, EllipticCylinder, GeomAppend, GeomStore, GeomStoreMut, ImplicitDef, Interval, Line, MemoryStore, NurbsCurve, NurbsSurface, Plane, Quadric, Sphere, StoreData, SurfaceDef, Torus, Uv,
};
use sm_testkit::geom::{curvatures_from_jet, surface_fixtures, tilted_frame, world_frame};

fn curve_defs() -> Vec<CurveDef> {
    let f = tilted_frame();
    let periodic = NurbsCurve::new_periodic(
        3,
        (0..13).map(|i| i as f64 - 3.0).collect(),
        {
            let u = [Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.5, 0.2), Point3::new(3.0, 2.0, -0.3), Point3::new(1.5, 3.0, 0.4), Point3::new(-0.5, 2.0, 0.0), Point3::new(-1.0, 0.7, 0.1)];
            (0..9).map(|i| u[i % 6]).collect()
        },
        None,
    )
    .unwrap();
    vec![
        CurveDef::Line(Line::new(Point3::new(1.0, 2.0, 3.0), Vector3::new(1.0, -1.0, 0.5)).unwrap()),
        CurveDef::Circle(Circle::new(f, 1.75).unwrap()),
        CurveDef::Ellipse(Ellipse::new(f, 2.0, 1.0).unwrap()),
        CurveDef::Nurbs(exact::circle_to_nurbs(&Circle::new(world_frame(), 2.0).unwrap()).unwrap()),
        CurveDef::Nurbs(periodic),
    ]
}

fn surface_defs() -> Vec<SurfaceDef> {
    let mut v = Vec::new();
    for f in [world_frame(), tilted_frame()] {
        v.push(SurfaceDef::Plane(Plane { frame: f }));
        v.push(SurfaceDef::Cylinder(Cylinder::new(f, 1.5).unwrap()));
        v.push(SurfaceDef::Cone(Cone::new(f, 0.5, 0.4).unwrap()));
        v.push(SurfaceDef::Sphere(Sphere::new(f, 2.0).unwrap()));
        v.push(SurfaceDef::Torus(Torus::new(f, 3.0, 1.0).unwrap()));
        v.push(SurfaceDef::EllipticCylinder(EllipticCylinder::new(f, 2.0, 1.0).unwrap()));
        v.push(SurfaceDef::EllipticCone(EllipticCone::new(f, 2.0, 1.0, 0.3).unwrap()));
    }
    v.push(SurfaceDef::Implicit(ImplicitDef::Quadric(Quadric::sphere(&tilted_frame(), 1.3))));
    v.push(SurfaceDef::Nurbs(exact::sphere_to_nurbs(&Sphere::new(tilted_frame(), 2.0).unwrap()).unwrap()));
    v.push(SurfaceDef::Nurbs(exact::torus_to_nurbs(&Torus::new(world_frame(), 3.0, 1.0).unwrap()).unwrap()));
    let net: Vec<Point3> = (0..5).flat_map(|i| (0..4).map(move |j| Point3::new(i as f64, j as f64 * 1.3, ((i * 3 + j * 5) % 7) as f64 * 0.3))).collect();
    v.push(SurfaceDef::Nurbs(NurbsSurface::new(3, 2, vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0], vec![0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0], 5, 4, net, None).unwrap()));
    v
}

fn both() -> (MemoryStore, KernelStore, Vec<sm_core::EntityId>, Vec<sm_core::EntityId>) {
    let (mut m, mut k) = (MemoryStore::new(), KernelStore::new());
    let (mut cm, mut cs) = (Vec::new(), Vec::new());
    for d in curve_defs() {
        cm.push(m.add_curve(d.clone()).unwrap());
        cs.push(k.add_curve(d).unwrap());
    }
    let (mut sm_, mut ss) = (Vec::new(), Vec::new());
    for d in surface_defs() {
        sm_.push(m.add_surface(d.clone()).unwrap());
        ss.push(k.add_surface(d).unwrap());
    }
    assert_eq!(cm, cs);
    assert_eq!(sm_, ss);
    (m, k, cm, sm_)
}

#[test]
fn same_handles_defs_counts_and_round_trip() {
    let (m, k, cs, ss) = both();
    assert_eq!((m.curve_count(), m.surface_count()), (k.curve_count(), k.surface_count()));
    for id in &cs {
        assert_eq!(m.curve_def(*id), k.curve_def(*id));
    }
    for id in &ss {
        assert_eq!(m.surface_def(*id), k.surface_def(*id));
        assert_eq!(m.surface(*id).is_some(), k.surface(*id).is_some());
        assert_eq!(m.implicit(*id).is_some(), k.implicit(*id).is_some());
    }
    // Round trips through the persisted form.
    let data: StoreData = k.clone().into();
    let back = KernelStore::from_data(data.clone()).unwrap();
    assert_eq!(back, k);
    assert_eq!(MemoryStore::from(data), m);
    assert_eq!(k.to_memory_store(), m);
    // Foreign and unknown handles resolve to nothing, as in the reference.
    assert!(k.curve(ss[0]).is_none() && k.surface(cs[0]).is_none());
    assert!(k.curve_def(sm_core::EntityId::new(sm_core::EntityKind::Curve, 99).unwrap()).is_none());
}

#[test]
fn evaluation_matches_reference_store() {
    let (m, k, cs, ss) = both();
    for id in &cs {
        let (a, b) = (m.curve(*id).unwrap(), k.curve(*id).unwrap());
        assert_eq!(a.domain(), b.domain());
        let d = a.domain().range;
        let (lo, hi) = if d.is_bounded() { (d.lo, d.hi) } else { (-2.0, 3.0) };
        for i in 0..=40 {
            let t = lo + (hi - lo) * i as f64 / 40.0;
            let (ja, jb) = (a.jet(t), b.jet(t));
            assert!((ja.p - jb.p).norm() < 1e-12 && (ja.d1 - jb.d1).norm() < 1e-11 && (ja.d2 - jb.d2).norm() < 1e-10, "{id} t={t}");
        }
        let q = Point3::new(0.7, -1.1, 2.3);
        let (pa, pb) = (a.project(q, None).unwrap(), b.project(q, None).unwrap());
        assert!((pa.distance - pb.distance).abs() < 1e-9, "{id}: {pa:?} {pb:?}");
    }
    for id in &ss {
        let (Some(a), Some(b)) = (m.surface(*id), k.surface(*id)) else { continue };
        let (da, db) = (a.domain(), b.domain());
        assert_eq!(da.u, db.u);
        assert_eq!(da.v, db.v);
        assert_eq!(da.singularities.len(), db.singularities.len(), "{id}");
        let ur = if da.u.range.is_bounded() { da.u.range } else { Interval::new(-1.0, 2.0) };
        let vr = if da.v.range.is_bounded() { da.v.range } else { Interval::new(-1.0, 2.0) };
        for i in 1..9 {
            for j in 1..9 {
                let uv = Uv::new(ur.lo + ur.len() * i as f64 / 9.0, vr.lo + vr.len() * j as f64 / 9.0);
                let (ja, jb) = (a.jet(uv), b.jet(uv));
                assert!((ja.p - jb.p).norm() < 1e-12 && (ja.du - jb.du).norm() < 1e-10 && (ja.dv - jb.dv).norm() < 1e-10, "{id} {uv:?}");
                assert!((ja.duu - jb.duu).norm() < 1e-9 && (ja.duv - jb.duv).norm() < 1e-9 && (ja.dvv - jb.dvv).norm() < 1e-9, "{id} {uv:?}");
            }
        }
        let q = Point3::new(0.4, 5.0, 1.5);
        if let (Ok(pa), Ok(pb)) = (a.project(q, None), b.project(q, None)) {
            if matches!(k.surface_def(*id), Some(SurfaceDef::Nurbs(_))) {
                // The prepared projector is global (branch and bound): never worse than the
                // reference sampler, and not better than a dense grid scan by more than rounding.
                assert!(pb.distance <= pa.distance + 1e-9, "{id}: {} vs {}", pb.distance, pa.distance);
                let (ur, vr) = (da.u.range, da.v.range);
                let mut best = f64::INFINITY;
                for i in 0..=120 {
                    for j in 0..=120 {
                        let uv = Uv::new(ur.lo + ur.len() * i as f64 / 120.0, vr.lo + vr.len() * j as f64 / 120.0);
                        best = best.min((b.eval(uv) - q).norm());
                    }
                }
                assert!(pb.distance <= best + 1e-9, "{id}: {} vs grid {}", pb.distance, best);
            } else {
                assert!((pa.distance - pb.distance).abs() < 1e-8, "{id}: {} vs {}", pa.distance, pb.distance);
            }
        }
        // Implicit forms agree exactly.
        if let (Some(fa), Some(fb)) = (m.implicit(*id), k.implicit(*id)) {
            let p = Point3::new(0.3, 0.2, -0.9);
            assert_eq!(fa.value(p), fb.value(p));
        }
    }
}

/// Analytic kinds go through the contract evaluators in both stores, so their results are
/// bit-identical (only the prepared NURBS evaluator may differ in rounding).
#[test]
fn analytic_handles_are_bit_identical_to_the_reference_store() {
    let (m, k, cs, ss) = both();
    macro_rules! bits {
        ($p:expr) => {{
            let p = $p;
            [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]
        }};
    }
    let (mut nc, mut ns) = (0, 0);
    for id in &cs {
        if matches!(k.curve_def(*id), Some(CurveDef::Nurbs(_))) {
            continue;
        }
        let (a, b) = (m.curve(*id).unwrap(), k.curve(*id).unwrap());
        let d = a.domain().range;
        let (lo, hi) = if d.is_bounded() { (d.lo, d.hi) } else { (-2.0, 3.0) };
        for i in 0..=20 {
            let t = lo + (hi - lo) * i as f64 / 20.0;
            let (ja, jb) = (a.jet(t), b.jet(t));
            assert_eq!((bits!(ja.p), bits!(ja.d1), bits!(ja.d2)), (bits!(jb.p), bits!(jb.d1), bits!(jb.d2)), "{id} t={t}");
        }
        nc += 1;
    }
    for id in &ss {
        if matches!(k.surface_def(*id), Some(SurfaceDef::Nurbs(_))) {
            continue;
        }
        let (Some(a), Some(b)) = (m.surface(*id), k.surface(*id)) else { continue };
        let (da, db) = (a.domain(), b.domain());
        let ur = if da.u.range.is_bounded() { da.u.range } else { Interval::new(-1.0, 2.0) };
        let vr = if da.v.range.is_bounded() { da.v.range } else { Interval::new(-1.0, 2.0) };
        assert_eq!(da.u, db.u);
        for i in 1..6 {
            for j in 1..6 {
                let uv = Uv::new(ur.lo + ur.len() * i as f64 / 6.0, vr.lo + vr.len() * j as f64 / 6.0);
                let (ja, jb) = (a.jet(uv), b.jet(uv));
                assert_eq!((bits!(ja.p), bits!(ja.du), bits!(ja.dv), bits!(ja.duu)), (bits!(jb.p), bits!(jb.du), bits!(jb.dv), bits!(jb.duu)), "{id} {uv:?}");
            }
        }
        ns += 1;
    }
    assert!(nc > 0 && ns > 0, "the fixtures must include analytic kinds");
}

#[test]
fn replace_truncate_and_validation_match_reference() {
    let (mut m, mut k, cs, ss) = both();
    let new = CurveDef::Line(Line::new(Point3::default(), Vector3::new(0.0, 0.0, 1.0)).unwrap());
    assert_eq!(m.replace_curve(cs[0], new.clone()).unwrap(), k.replace_curve(cs[0], new).unwrap());
    let ns = SurfaceDef::Sphere(Sphere::new(world_frame(), 9.0).unwrap());
    assert_eq!(m.replace_surface(ss[3], ns.clone()).unwrap(), k.replace_surface(ss[3], ns).unwrap());
    assert!(m.to_data_eq(&k));
    // Invalid input is rejected identically.
    let bad = CurveDef::Circle(Circle { frame: world_frame(), radius: -1.0 });
    assert!(m.add_curve(bad.clone()).is_err() && k.add_curve(bad).is_err());
    assert_eq!(m.curve_count(), k.curve_count());
    assert!(k.replace_curve(ss[0], CurveDef::Line(Line::new(Point3::default(), Vector3::new(1.0, 0.0, 0.0)).unwrap())).is_err());
    m.truncate(2, 5);
    k.truncate(2, 5);
    assert_eq!((m.curve_count(), m.surface_count()), (k.curve_count(), k.surface_count()));
    assert!(k.surface(ss[10]).is_none() && k.implicit(ss[10]).is_none());
    // New handles continue from the truncated counts.
    let a = m.add_curve(CurveDef::Circle(Circle::new(world_frame(), 1.0).unwrap())).unwrap();
    let b = k.add_curve(CurveDef::Circle(Circle::new(world_frame(), 1.0).unwrap())).unwrap();
    assert_eq!(a, b);
}

trait SameData {
    fn to_data_eq(&self, k: &KernelStore) -> bool;
}
impl SameData for MemoryStore {
    fn to_data_eq(&self, k: &KernelStore) -> bool {
        *self == k.to_memory_store()
    }
}

#[test]
fn testkit_surface_fixtures_through_the_store() {
    // The fixtures list is, per frame: plane, cylinder 1.5, cone (0.5, 0.4), sphere 2, torus 3/1.
    let fixtures = surface_fixtures();
    let mut k = KernelStore::new();
    let mut ids = Vec::new();
    for f in [world_frame(), tilted_frame()] {
        for d in [
            SurfaceDef::Plane(Plane { frame: f }),
            SurfaceDef::Cylinder(Cylinder::new(f, 1.5).unwrap()),
            SurfaceDef::Cone(Cone::new(f, 0.5, 0.4).unwrap()),
            SurfaceDef::Sphere(Sphere::new(f, 2.0).unwrap()),
            SurfaceDef::Torus(Torus::new(f, 3.0, 1.0).unwrap()),
        ] {
            ids.push(k.add_surface(d).unwrap());
        }
    }
    assert_eq!(ids.len(), fixtures.len());
    for (fx, id) in fixtures.iter().zip(&ids) {
        let s = k.surface(*id).unwrap();
        for uv in fx.sample_uvs() {
            let n = s.normal(uv).unwrap();
            assert!((n - fx.exact_normal(uv)).norm() < 1e-12, "{} normal", fx.label());
            let (kk, h) = curvatures_from_jet(&s.jet(uv), n).unwrap();
            assert!((kk - fx.gaussian_curvature(uv)).abs() < 1e-9, "{} K", fx.label());
            assert!((h - fx.mean_curvature(uv)).abs() < 1e-9, "{} H", fx.label());
        }
        let q = Point3::new(1.3, -2.1, 0.77);
        if let (Some(want), Ok(got)) = (fx.closest_point(q), s.project(q, None)) {
            assert!((got.point - want).norm() < 1e-9, "{} closest", fx.label());
        }
    }
    let _ = TAU;
}
