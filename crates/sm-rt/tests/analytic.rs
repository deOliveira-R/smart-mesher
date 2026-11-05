//! W3b-RT-ANALYTIC acceptance 1-6 (kernels and point location) on exact chords.
//! Tolerance is 1e-12 absolute unless stated; fixtures are local (`common/csg.rs`).
#[path = "common/csg.rs"]
mod csg;

use csg::*;
use sm_core::Vector3;
use sm_geom_api::{Frame, Quadric};
use sm_rt_api::{CellId, Scratch, StepEnd, Tracker};

fn mean_se(xs: &[f64]) -> (f64, f64) {
    let n = xs.len() as f64;
    let m = xs.iter().sum::<f64>() / n;
    let var = xs.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / (n - 1.0);
    (m, (var / n).sqrt())
}

/// A uniformly distributed (IUR conditioned on hitting a ball of radius `rb`) offset and
/// direction: the line passes through a uniform disc of radius `rb` perpendicular to `d`.
fn iur(r: &mut SplitMix64, rb: f64) -> (Vector3, Vector3) {
    let d = random_dir(r);
    let e = perp_to(r, d);
    let f = d.cross(e);
    loop {
        let (u, v) = (r.unit() * rb, r.unit() * rb);
        if u * u + v * v <= rb * rb {
            return (d, e * u + f * v);
        }
    }
}

#[test]
fn sphere_chords_and_cauchy_mean() {
    let c0 = p3(12.0, -7.0, 20.0);
    let rad = 5.0;
    let mut fx = Fx::new(90.0);
    let s = fx.sphere(c0, rad);
    fx.cell1("ball", &[hs(s, true)]);
    let t = fx.build();
    assert_eq!(t.surface_kinds(), vec!["sphere"]);
    let mut sc = Scratch::new();
    let mut r = rng("sphere");
    for b in [0.0, 3.0, 4.9] {
        for _ in 0..200 {
            let d = random_dir(&mut r);
            let e = perp_to(&mut r, d);
            let tr = trace(&t, line(c0, d, e * b, 30.0), d, f64::INFINITY, &mut sc);
            let exact = 2.0 * (rad * rad - b * b).sqrt();
            assert!((tr.inside() - exact).abs() < 1e-12, "b {b}: {} vs {exact}", tr.inside());
            assert_eq!(tr.chords().len(), 1);
        }
    }
    let n = n_lines();
    let mut chords = Vec::with_capacity(n);
    for _ in 0..n {
        let (d, off) = iur(&mut r, rad);
        let tr = trace(&t, line(c0, d, off, 30.0), d, f64::INFINITY, &mut sc);
        chords.extend(tr.chords());
    }
    let (m, se) = mean_se(&chords);
    assert!((m - 4.0 * rad / 3.0).abs() < 4.0 * se, "mean {m} se {se}");
}

#[test]
fn cylinder_finite_tilted_and_annulus() {
    let mut r = rng("cylinder");
    let (rad, h) = (2.0, 7.0);
    let f = random_frame(&mut r, p3(5.0, -3.0, 4.0));
    let mut fx = Fx::new(90.0);
    let c = fx.cylinder(f, rad);
    let lo = fx.plane(f.origin, f.z);
    let hi = fx.plane(f.origin + f.z * h, f.z);
    fx.cell1("cyl", &[hs(c, true), hs(lo, false), hs(hi, true)]);
    let t = fx.build();
    assert_eq!(t.surface_kinds(), vec!["cylinder", "plane", "plane"]);
    let mut sc = Scratch::new();
    let mid = f.origin + f.z * (h / 2.0);
    for _ in 0..300 {
        // In the cross-section plane.
        let phi = std::f64::consts::PI * r.unit();
        let d = f.x * phi.cos() + f.y * phi.sin();
        let e = f.z.cross(d);
        let b = rad * 0.999 * r.unit().abs();
        let tr = trace(&t, mid + (e * b - d * 25.0), d, f64::INFINITY, &mut sc);
        assert!((tr.inside() - 2.0 * (rad * rad - b * b).sqrt()).abs() < 1e-12);
        // Through the axis at angle alpha to it, clipped by the planes.
        let alpha = 0.2 + 1.2 * r.unit().abs();
        let z0 = h * (0.5 + 0.4 * r.unit());
        let u = d;
        let dir = f.z * alpha.cos() + u * alpha.sin();
        let o = f.origin + f.z * z0;
        let tr = trace(&t, o + (-dir * 20.0), dir, f64::INFINITY, &mut sc);
        let (t0, t1) = ((-z0) / alpha.cos(), (h - z0) / alpha.cos());
        let (a, b2) = (t0.min(t1).max(-rad / alpha.sin()), t0.max(t1).min(rad / alpha.sin()));
        assert!((tr.inside() - (b2 - a).max(0.0)).abs() < 1e-12, "alpha {alpha}: {} vs {}", tr.inside(), b2 - a);
    }

    // Coaxial annulus: bore + two annular chords = outer chord; the concentric fast path applies.
    let mut fx = Fx::new(90.0);
    let (r1, r2) = (1.0, 2.0);
    let c1 = fx.cylinder(f, r1);
    let c2 = fx.cylinder(f, r2);
    let lo = fx.plane(f.origin, f.z);
    let hi = fx.plane(f.origin + f.z * h, f.z);
    fx.cell1("bore", &[hs(c1, true), hs(lo, false), hs(hi, true)]);
    fx.cell1("ring", &[hs(c1, false), hs(c2, true), hs(lo, false), hs(hi, true)]);
    let t = fx.build();
    assert!(t.has_concentric_fast_path());
    for _ in 0..200 {
        let phi = std::f64::consts::PI * r.unit();
        let d = f.x * phi.cos() + f.y * phi.sin();
        let e = f.z.cross(d);
        let b = r1 * 0.99 * r.unit().abs();
        let tr = trace(&t, mid + (e * b - d * 25.0), d, f64::INFINITY, &mut sc);
        assert!((tr.inside() - 2.0 * (r2 * r2 - b * b).sqrt()).abs() < 1e-12);
        assert_eq!(tr.chords().len(), 3);
    }
}

#[test]
fn cone_nappes_and_apex() {
    let mut r = rng("cone");
    let half = 30f64.to_radians();
    let tn = half.tan();
    let (z0, z1) = (1.0, 10.0);
    let f = random_frame(&mut r, p3(3.0, -2.0, 5.0));
    let mut fx = Fx::new(90.0);
    let cone = fx.cone(f, half);
    let up_lo = fx.plane(f.origin + f.z * z0, f.z);
    let up_hi = fx.plane(f.origin + f.z * z1, f.z);
    let dn_lo = fx.plane(f.origin + (-f.z * z1), f.z);
    let dn_hi = fx.plane(f.origin + (-f.z * z0), f.z);
    fx.cell("cone", vec![term(&[hs(cone, true), hs(up_lo, false), hs(up_hi, true)]), term(&[hs(cone, true), hs(dn_lo, false), hs(dn_hi, true)])]);
    let t = fx.build();
    assert_eq!(t.surface_kinds()[0], "cone");
    let mut sc = Scratch::new();
    for rho in [0.5, 1.5, 2.0, 4.0] {
        for _ in 0..50 {
            let e = perp_to(&mut r, f.z);
            let tr = trace(&t, f.origin + (e * rho - f.z * 30.0), f.z, f64::INFINITY, &mut sc);
            let exact = (z1 - z0.max(rho / tn)).max(0.0);
            let chords = tr.chords();
            assert_eq!(chords.len(), if exact > 0.0 { 2 } else { 0 }, "rho {rho}");
            for c in chords {
                assert!((c - exact).abs() < 1e-12, "rho {rho}: {c} vs {exact}");
            }
        }
    }
    // Along the axis through the apex (inside both nappes): chord z1 - z0 each, no event at the apex.
    let tr = trace(&t, f.origin + (-f.z * 30.0), f.z, f64::INFINITY, &mut sc);
    assert_eq!(tr.chords().len(), 2);
    assert!(tr.chords().iter().all(|c| (c - (z1 - z0)).abs() < 1e-12));
    // A line through the apex at 60 degrees to the axis stays outside both nappes: the apex is a
    // double root, so there is no event and no zero-length cell flip.
    let d = f.z * 60f64.to_radians().cos() + f.x * 60f64.to_radians().sin();
    let tr = trace(&t, f.origin + (-d * 20.0), d, f64::INFINITY, &mut sc);
    assert_eq!(tr.segs.len(), 1, "{:?}", tr.segs);
    assert!(matches!(tr.end, sm_rt_api::TraceEnd::Boundary { .. }));
}

#[test]
fn torus_chords_tangent_and_cauchy() {
    let mut r = rng("torus");
    let (big, small) = (3.0, 1.0);
    let f = random_frame(&mut r, p3(7.0, -4.0, 3.0));
    let mut fx = Fx::new(90.0);
    let s = fx.torus(f, big, small);
    fx.cell1("ring", &[hs(s, true)]);
    let t = fx.build();
    assert_eq!(t.surface_kinds(), vec!["torus"]);
    let mut sc = Scratch::new();
    for _ in 0..300 {
        // Axis-parallel at rho in (R - r, R + r).
        let rho = 2.0 + 2.0 * (0.5 + 0.5 * r.unit()).clamp(0.001, 0.999);
        let e = perp_to(&mut r, f.z);
        let tr = trace(&t, f.origin + (e * rho - f.z * 30.0), f.z, f64::INFINITY, &mut sc);
        let exact = 2.0 * (small * small - (rho - big).powi(2)).sqrt();
        assert_eq!(tr.chords().len(), 1);
        assert!((tr.inside() - exact).abs() < 1e-12, "rho {rho}: {} vs {exact}", tr.inside());
        // Equatorial lines at offset b.
        let phi = std::f64::consts::PI * r.unit();
        let d = f.x * phi.cos() + f.y * phi.sin();
        let eq = f.z.cross(d);
        let b = 4.0 * 0.999 * r.unit().abs();
        let tr = trace(&t, f.origin + (eq * b - d * 30.0), d, f64::INFINITY, &mut sc);
        let (so, si) = ((16.0 - b * b).sqrt(), if b < 2.0 { (4.0 - b * b).sqrt() } else { 0.0 });
        let ch = tr.chords();
        if b < 2.0 {
            assert_eq!(ch.len(), 2, "b {b}");
            assert!(ch.iter().all(|c| (c - (so - si)).abs() < 1e-12), "b {b}: {ch:?} vs {}", so - si);
        } else {
            assert_eq!(ch.len(), 1, "b {b}");
            assert!((ch[0] - 2.0 * so).abs() < 1e-12);
        }
    }
    // Tangent: no event, stays in the complement.
    let d = f.x;
    let eq = f.y;
    let tr = trace(&t, f.origin + (eq * 4.0 - d * 30.0), d, f64::INFINITY, &mut sc);
    assert_eq!(tr.segs.len(), 1);
    // Near tangent: the chord is found to the documented sqrt conditioning.
    let b = 4.0 - 1e-6;
    let tr = trace(&t, f.origin + (eq * b - d * 30.0), d, f64::INFINITY, &mut sc);
    assert!(matches!(tr.end, sm_rt_api::TraceEnd::Boundary { .. }));
    assert!((tr.inside() - 2.0 * (16.0 - b * b).sqrt()).abs() < 1e-9, "{}", tr.inside());
    // Cauchy: the mean chord over IUR lines, each chord counted separately, is 4V/S = 2r.
    let n = n_lines();
    let mut chords = Vec::with_capacity(2 * n);
    for _ in 0..n {
        let (d, off) = iur(&mut r, big + small);
        let tr = trace(&t, f.origin + (off - d * 30.0), d, f64::INFINITY, &mut sc);
        assert!(!matches!(tr.end, sm_rt_api::TraceEnd::Lost(_)));
        chords.extend(tr.chords());
    }
    let (m, se) = mean_se(&chords);
    assert!((m - 2.0 * small).abs() < 4.0 * se, "mean {m} se {se}");
}

fn ellipsoid(fx: &mut Fx, f: &Frame, ax: [f64; 3]) -> sm_core::EntityId {
    fx.quadric(Quadric::from_local(f, [1.0 / (ax[0] * ax[0]), 1.0 / (ax[1] * ax[1]), 1.0 / (ax[2] * ax[2])], [0.0; 3], -1.0))
}

#[test]
fn general_quadric_chords() {
    let mut r = rng("quadric");
    let ax = [4.0, 2.5, 1.5];
    let f = random_frame(&mut r, p3(10.0, 6.0, -8.0));
    let mut fx = Fx::new(90.0);
    let s = ellipsoid(&mut fx, &f, ax);
    fx.cell1("ell", &[hs(s, true)]);
    let t = fx.build();
    assert_eq!(t.surface_kinds(), vec!["quadric"]);
    let mut sc = Scratch::new();
    for _ in 0..500 {
        let d = random_dir(&mut r);
        let e = perp_to(&mut r, d);
        let o = f.origin + (e * (2.0 * r.unit().abs()) - d * 30.0);
        let tr = trace(&t, o, d, f64::INFINITY, &mut sc);
        // Independent closed form from the semi-axes.
        let u = [f.x.dot(d) / ax[0], f.y.dot(d) / ax[1], f.z.dot(d) / ax[2]];
        let q = o - f.origin;
        let w = [f.x.dot(q) / ax[0], f.y.dot(q) / ax[1], f.z.dot(q) / ax[2]];
        let (a, b, c) = (u.iter().map(|x| x * x).sum::<f64>(), u.iter().zip(&w).map(|(x, y)| x * y).sum::<f64>(), w.iter().map(|x| x * x).sum::<f64>() - 1.0);
        let disc = b * b - a * c;
        let exact = if disc > 0.0 { 2.0 * disc.sqrt() / a } else { 0.0 };
        assert!((tr.inside() - exact).abs() < 1e-12, "{} vs {exact}", tr.inside());
    }
    // Hyperboloid of one sheet x^2 + y^2 - z^2 = 1 cut by z = -3 and z = 3.
    let mut fx = Fx::new(90.0);
    let f0 = Frame::at(p3(1.0, 2.0, 3.0));
    let hyp = fx.quadric(Quadric::from_local(&f0, [1.0, 1.0, -1.0], [0.0; 3], -1.0));
    let lo = fx.plane(p3(1.0, 2.0, 0.0), v3(0.0, 0.0, 1.0));
    let hi = fx.plane(p3(1.0, 2.0, 6.0), v3(0.0, 0.0, 1.0));
    // Inside the hyperboloid (f <= 0) is |z| >= sqrt(rho^2 - 1) around the centre z = 3.
    fx.cell1("hyp", &[hs(hyp, true), hs(lo, false), hs(hi, true)]);
    let t = fx.build();
    let tr = trace(&t, p3(1.0 + 2.0, 2.0, -20.0), v3(0.0, 0.0, 1.0), f64::INFINITY, &mut sc);
    let ch = tr.chords();
    assert_eq!(ch.len(), 2);
    for c in ch {
        assert!((c - (3.0 - 3f64.sqrt())).abs() < 1e-12, "{c}");
    }
}

#[test]
fn point_location_exact_and_concentric_fast_path() {
    let mut r = rng("locate");
    let c0 = p3(2.0, -3.0, 4.0);
    let radii = [1.0, 2.0, 3.0, 4.0, 5.0];
    let mut fx = Fx::new(60.0);
    let sph: Vec<_> = radii.iter().map(|&q| fx.sphere(c0, q)).collect();
    fx.cell1("s0", &[hs(sph[0], true)]);
    for k in 1..5 {
        fx.cell1(&format!("s{k}"), &[hs(sph[k - 1], false), hs(sph[k], true)]);
    }
    // A torus and a cone far from the shells.
    let ft = random_frame(&mut r, p3(30.0, 0.0, 0.0));
    let tor = fx.torus(ft, 3.0, 1.0);
    fx.cell1("tor", &[hs(tor, true)]);
    let fc = random_frame(&mut r, p3(-30.0, 10.0, 0.0));
    let cone = fx.cone(fc, 0.5);
    let (a, b) = (fx.plane(fc.origin + fc.z * 1.0, fc.z), fx.plane(fc.origin + fc.z * 6.0, fc.z));
    fx.cell1("cone", &[hs(cone, true), hs(a, false), hs(b, true)]);
    let t = fx.build();
    assert!(t.has_concentric_fast_path());
    let mut sc = Scratch::new();
    let n = n_lines();
    let expect = |p: sm_core::Point3| -> (CellId, f64) {
        // (cell, smallest distance to any surface)
        let rr = (p - c0).norm();
        let mut dmin = radii.iter().map(|q| (rr - q).abs()).fold(f64::INFINITY, f64::min);
        let mut cell = CellId::COMPLEMENT;
        if rr < 5.0 {
            cell = CellId::new(radii.iter().position(|&q| rr < q).unwrap());
        }
        let q = p - ft.origin;
        let (x, y, z) = (ft.x.dot(q), ft.y.dot(q), ft.z.dot(q));
        let dt = ((x.hypot(y) - 3.0).hypot(z) - 1.0).abs();
        dmin = dmin.min(dt);
        if (x.hypot(y) - 3.0).hypot(z) < 1.0 {
            cell = CellId::new(5);
        }
        let q = p - fc.origin;
        let (rho, zz) = (fc.x.dot(q).hypot(fc.y.dot(q)), fc.z.dot(q));
        let (ca, sa) = (0.5f64.cos(), 0.5f64.sin());
        // distance to the cone surface is only used far from it (the margin is generous)
        dmin = dmin.min((rho * ca - zz.abs() * sa).abs()).min((zz - 1.0).abs()).min((zz - 6.0).abs());
        if rho < zz * 0.5f64.tan() && zz > 1.0 && zz < 6.0 {
            cell = CellId::new(6);
        }
        (cell, dmin)
    };
    let (mut checked, mut on_face_seen) = (0, 0);
    for i in 0..n {
        // Points near the shells, the torus and the cone, and uniform ones.
        let p = match i % 4 {
            0 => c0 + random_dir(&mut r) * (5.5 * (0.5 + 0.5 * r.unit())),
            1 => {
                let k = (r.unit().abs() * 5.0) as usize % 5;
                c0 + random_dir(&mut r) * (radii[k] + 4e-6 * r.unit())
            }
            2 => ft.origin + random_dir(&mut r) * 5.0,
            _ => p3(55.0 * r.unit(), 55.0 * r.unit(), 55.0 * r.unit()),
        };
        let l = t.locate(p, None, None, &mut sc).unwrap();
        let g = t.locate_generic(p, None, &mut sc).unwrap();
        assert_eq!(l, g, "fast path differs from the generic path at {p:?}");
        let (cell, dmin) = expect(p);
        if dmin > 2e-6 {
            assert_eq!(l.cell, cell, "{p:?}");
            assert!(l.on_face.is_none());
            checked += 1;
        } else if dmin <= 0.5e-6 {
            assert!(l.on_face.is_some(), "{p:?} within the band must report on_face");
            on_face_seen += 1;
        }
    }
    assert!(checked > n / 2 && on_face_seen > 0, "{checked} {on_face_seen}");

    // Points constructed exactly on a surface: the direction picks the side.
    let c1 = p3(0.0, 0.0, 0.0);
    let mut fx = Fx::new(60.0);
    let s = fx.sphere(c1, 4.0);
    fx.cell1("ball", &[hs(s, true)]);
    let t = fx.build();
    let on = p3(4.0, 0.0, 0.0);
    let inn = t.locate(on, Some(v3(-1.0, 0.0, 0.0)), None, &mut sc).unwrap();
    let out = t.locate(on, Some(v3(1.0, 0.0, 0.0)), None, &mut sc).unwrap();
    assert_eq!(inn.cell, CellId::new(0));
    assert_eq!(out.cell, CellId::COMPLEMENT);
    assert!(inn.on_face.is_some() && out.on_face.is_some());
    // dir = None is the hashed-direction rule, deterministic across calls.
    let h = sm_rt_api::hashed_direction(on);
    let a = t.locate(on, None, None, &mut sc).unwrap();
    assert_eq!(a, t.locate(on, Some(h), None, &mut sc).unwrap());
    assert_eq!(a, t.locate(on, None, None, &mut sc).unwrap());
}

#[test]
fn pin_cell_locate_work_counter() {
    // A 12-ring pin cell: the fast path costs one radius and one shell test.
    let mut fx = Fx::new(10.0);
    let f = Frame::at(p3(0.0, 0.0, 0.0));
    let cyl: Vec<_> = (1..=12).map(|k| fx.cylinder(f, 0.1 * k as f64)).collect();
    fx.cell1("r0", &[hs(cyl[0], true)]);
    for k in 1..12 {
        fx.cell1(&format!("r{k}"), &[hs(cyl[k - 1], false), hs(cyl[k], true)]);
    }
    fx.cell1("mod", &[hs(cyl[11], false)]);
    let t = fx.build();
    assert!(t.has_concentric_fast_path());
    let mut r = rng("pin");
    let bound = (12f64).log2().ceil() as u64 + 3;
    for _ in 0..2000 {
        let mut sc = Scratch::new();
        let p = p3(1.5 * r.unit(), 1.5 * r.unit(), 3.0 * r.unit());
        let l = t.locate(p, Some(v3(0.0, 0.0, 1.0)), None, &mut sc).unwrap();
        assert!(!l.cell.is_none());
        assert!(sc.counters.surface_evals <= bound, "{} > {bound}", sc.counters.surface_evals);
        assert_eq!(sc.counters.slow_locates, 0);
    }
}

#[test]
fn step_ends_report_cells_and_faces() {
    let mut fx = Fx::new(30.0);
    let a = fx.sphere(p3(0.0, 0.0, 0.0), 2.0);
    let b = fx.sphere(p3(0.0, 0.0, 0.0), 4.0);
    fx.cell1("in", &[hs(a, true)]);
    fx.cell1("shell", &[hs(a, false), hs(b, true)]);
    let t = fx.build();
    let mut sc = Scratch::new();
    let mut st = t.init_state(p3(-10.0, 0.0, 0.0), v3(1.0, 0.0, 0.0), None, &mut sc).unwrap();
    assert_eq!(st.cell, CellId::COMPLEMENT);
    let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    assert!((s.dist - 6.0).abs() < 1e-12);
    assert!(matches!(s.end, StepEnd::RegionBoundary { to_cell, .. } if to_cell == CellId::new(1)));
    let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    assert!((s.dist - 2.0).abs() < 1e-12);
    assert!(matches!(s.end, StepEnd::RegionBoundary { to_cell, .. } if to_cell == CellId::new(0)));
}
