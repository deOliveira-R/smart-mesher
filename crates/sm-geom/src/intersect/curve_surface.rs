//! Curve-plane and curve-surface intersection.

use super::bernstein;
use super::contact::{Gap, merge_contacts};
use super::interval;
use super::{CurveSurface, CurveSurfaceHit};
use crate::nurbs::curve::{SegPoles, bbox_of, hp_point};
use crate::nurbs::exact;
use crate::nurbs::project::split_patch;
use crate::nurbs::spline1::split_half_into;
use crate::nurbs::surface::PatchPoles;
use crate::nurbs::{PreparedCurve, PreparedSurface};
use crate::roots::{poly_roots, solve_dense};
use core::f64::consts::TAU;
use sm_core::{Point3, Vector3};
use sm_geom_api::{
    Crossing, CurveDef, GeomError, ImplicitDef, ImplicitSurface, Interval, ModelTolerance, NurbsCurve, ParametricCurve, ParametricSurface, Plane, Ray, RayHit, SurfaceDef, Uv,
};

type Raw = (f64, bool);

fn domain_range(curve: &CurveDef, range: Option<Interval>) -> Interval {
    let d = curve.as_curve().domain();
    match range {
        // A periodic analytic curve may be asked about any window of its (unwrapped) parameter.
        Some(r) if d.is_periodic() && !matches!(curve, CurveDef::Nurbs(_)) => r,
        Some(r) => Interval::new(r.lo.max(d.range.lo), r.hi.min(d.range.hi)),
        None => d.range,
    }
}

fn surface_uv(surface: &SurfaceDef, p: Point3) -> Option<Uv> {
    surface.as_surface().and_then(|s| s.project(p, None).ok()).map(|pr| pr.uv)
}

/// Sort by parameter and merge hits that are the same contact: parameters equal to rounding,
/// or neighbouring parameters whose points lie within `merge` of each other.
fn build_hits(curve: &CurveDef, surface: &SurfaceDef, mut raw: Vec<Raw>, merge: f64) -> Vec<CurveSurfaceHit> {
    let c = curve.as_curve();
    raw.sort_by(|a, b| a.0.total_cmp(&b.0));
    let span = {
        let r = c.domain().range;
        if r.is_bounded() { r.len() } else { 1.0 }
    };
    let mut merged: Vec<Raw> = Vec::new();
    for h in raw {
        match merged.last_mut() {
            Some(l) if (h.0 - l.0).abs() <= 1e-12 * (1.0 + h.0.abs()) || ((h.0 - l.0).abs() <= 1e-6 * span && (c.eval(h.0) - c.eval(l.0)).norm() <= merge) => l.1 |= h.1,
            _ => merged.push(h),
        }
    }
    merged
        .into_iter()
        .map(|(t, tangent)| {
            let point = c.eval(t);
            CurveSurfaceHit { t, point, uv: surface_uv(surface, point), tangent }
        })
        .collect()
}

/// Representatives of `t` (mod `period`) in the half-open `[range.lo, range.hi)`: the seam is
/// reported once, at its low end.
fn periodic_in_range(t: f64, period: f64, range: Interval) -> impl Iterator<Item = f64> {
    let k0 = ((range.lo - t) / period).ceil() as i64;
    let k1 = ((range.hi - t) / period).ceil() as i64 - 1;
    (k0..=k1).map(move |k| t + k as f64 * period)
}

// ------------------------------------------------------------------ curve - plane

/// Intersect a curve with a plane.
///
/// Closed forms for lines, circles and ellipses; Bernstein subdivision (exact root isolation of
/// the signed distance in homogeneous form) for NURBS. `range` restricts the curve parameter
/// (default: the curve's domain; a periodic curve is searched on its principal period). A curve
/// lying in the plane within `tol.linear` gives [`CurveSurface::Coincident`].
pub fn curve_plane(curve: &CurveDef, range: Option<Interval>, plane: &Plane, tol: &ModelTolerance) -> Result<CurveSurface, GeomError> {
    let n = plane.frame.z;
    let p0 = plane.frame.origin;
    let r = domain_range(curve, range);
    let surf = SurfaceDef::Plane(*plane);
    let raw: Vec<Raw> = match curve {
        CurveDef::Line(l) => {
            let dn = l.dir.dot(n);
            let dist0 = n.dot(l.origin - p0);
            if dn.abs() <= tol.angular {
                return Ok(if dist0.abs() <= tol.linear { CurveSurface::Coincident } else { CurveSurface::Points(Vec::new()) });
            }
            let t = -dist0 / dn;
            if r.contains(t, 0.0) { vec![(t, false)] } else { Vec::new() }
        }
        CurveDef::Circle(c) => match ellipse_plane(&c.frame, c.radius, c.radius, n, p0, tol, r) {
            Some(v) => v,
            None => return Ok(CurveSurface::Coincident),
        },
        CurveDef::Ellipse(e) => match ellipse_plane(&e.frame, e.a, e.b, n, p0, tol, r) {
            Some(v) => v,
            None => return Ok(CurveSurface::Coincident),
        },
        CurveDef::Nurbs(nc) => {
            let pc = PreparedCurve::new(nc)?;
            match nurbs_plane(&pc, n, p0, tol, r) {
                // A contact at a knot or end is found by the tolerance rule, not a sign change:
                // the angle between the tangent and the plane decides whether it grazes.
                Some(v) => {
                    let raw: Vec<Raw> = v
                        .into_iter()
                        .map(|(t, tan)| {
                            let d1 = pc.jet(t).d1;
                            (t, tan || n.dot(d1).abs() <= 1e-6 * d1.norm())
                        })
                        .collect();
                    // One hit per contact stretch, tangent by the sign rule (see `contact`).
                    let (lo, hi) = (pc.range().lo, pc.range().hi);
                    let closed = pc.domain().is_periodic() || (pc.eval(lo) - pc.eval(hi)).norm() <= tol.linear;
                    let signed = |t: f64| n.dot(pc.eval(t) - p0);
                    let h0 = raw.first().map_or(1.0, |h| 0.5 * tol.linear / pc.jet(h.0).d1.norm().max(1e-300));
                    let gap = Gap { f: &signed, lo, hi, closed, tol: tol.linear, h0 };
                    merge_contacts(&gap, &raw).into_iter().filter(|h| r.contains(h.0, 0.0)).collect()
                }
                None => return Ok(CurveSurface::Coincident),
            }
        }
        _ => return Err(GeomError::Unsupported("curve kind")),
    };
    Ok(CurveSurface::Points(build_hits(curve, &surf, raw, 5.0 * tol.linear)))
}

/// `None` when the ellipse lies in the plane; the roots are otherwise parameters of the
/// ellipse with semi-axes `a`, `b` in the frame `f`.
fn ellipse_plane(f: &sm_geom_api::Frame, a: f64, b: f64, n: Vector3, p0: Point3, tol: &ModelTolerance, r: Interval) -> Option<Vec<Raw>> {
    let c = n.dot(f.origin - p0);
    let (ca, sb) = (a * n.dot(f.x), b * n.dot(f.y));
    let amp = ca.hypot(sb);
    if amp <= 1e-12 * a.max(b) {
        return if c.abs() <= tol.linear { None } else { Some(Vec::new()) };
    }
    let k = -c / amp;
    let phi = sb.atan2(ca);
    let mut roots: Vec<Raw> = Vec::new();
    if k.abs() > 1.0 {
        if c.abs() - amp <= tol.linear {
            roots.push((phi + if k > 0.0 { 0.0 } else { core::f64::consts::PI }, true));
        }
    } else if amp - c.abs() <= tol.linear {
        // Cut by less than `tol.linear`: the curve stays within tolerance of the plane between
        // the two roots, which is one grazing contact (the same rule as for `|k| > 1`).
        roots.push((phi + if k > 0.0 { 0.0 } else { core::f64::consts::PI }, true));
    } else if 1.0 - k.abs() < 1e-12 {
        roots.push((phi + if k > 0.0 { 0.0 } else { core::f64::consts::PI }, true));
    } else {
        let d = k.acos();
        roots.push((phi + d, false));
        roots.push((phi - d, false));
    }
    let mut out = Vec::new();
    for (t, tan) in roots {
        let w = t.rem_euclid(TAU);
        for tt in periodic_in_range(w, TAU, r) {
            out.push((tt, tan));
        }
    }
    Some(out)
}

/// `None` when the curve lies in the plane.
fn nurbs_plane(pc: &PreparedCurve, n: Vector3, p0: Point3, tol: &ModelTolerance, r: Interval) -> Option<Vec<Raw>> {
    let c0 = n.dot(Vector3::new(p0.x, p0.y, p0.z));
    let mut all_flat = true;
    let mut out: Vec<Raw> = Vec::new();
    let periodic = pc.domain().is_periodic();
    for s in &pc.segs {
        let d: Vec<f64> = s.poles.iter().map(|h| n.x * h[0] + n.y * h[1] + n.z * h[2] - c0 * h[3]).collect();
        let wmin = s.poles.iter().fold(f64::INFINITY, |m, h| m.min(h[3]));
        if d.iter().zip(s.poles.iter()).any(|(d, h)| d.abs() > tol.linear * h[3]) {
            all_flat = false;
        }
        for br in bernstein::roots(&d, tol.linear * wmin) {
            let mut t = s.lo + br.s * (s.hi - s.lo);
            if periodic && t >= pc.range().hi {
                t = pc.range().lo;
            }
            if r.contains(t, 0.0) {
                out.push((t, br.tangent));
            }
        }
        // A crossing exactly at a knot can leave both neighbours without a sign change in the
        // rounded coefficients: take segment ends that lie on the plane within tolerance.
        for (k, t) in [(0usize, s.lo), (d.len() - 1, s.hi)] {
            let t = if periodic && t >= pc.range().hi { pc.range().lo } else { t };
            if d[k].abs() <= tol.linear * s.poles[k][3] && r.contains(t, 0.0) {
                out.push((t, false));
            }
        }
    }
    if all_flat { None } else { Some(out) }
}

// ------------------------------------------------------------------ curve - surface

/// Intersect a curve with a surface.
///
/// - plane: [`curve_plane`];
/// - lines: the contract's ray hook (closed forms) for analytic surfaces;
/// - circles and ellipses against quadrics and the torus: exact, through the trigonometric
///   polynomial of the restriction (`x = tan(t/2)` and polynomial roots);
/// - NURBS curves against implicit surfaces: Bezier subdivision with interval bounds, then
///   Newton;
/// - anything against a NURBS surface: pairwise Bezier box subdivision, then damped Newton.
///
/// Hits are ordered by curve parameter; a grazing contact is reported once with `tangent`.
/// Coincidence (the curve running along the surface) is detected for planes and, by sampling,
/// for implicit surfaces; for NURBS surfaces a coincident curve yields
/// [`GeomError::NoConvergence`] (documented limit).
///
/// A circle or ellipse against a quadric or torus treats a local extremum of the restriction
/// that stays within `tol.linear` of the surface as one tangent contact (like the other closed
/// forms); a NURBS surface merges neighbouring hits while the curve stays within `tol.linear`
/// of it between them.
///
/// **Resolution limits.** NURBS curve against an implicit surface subdivides to `20 tol.linear`
/// (contacts closer than a few `tol.linear`, or along a stretch that stays within `tol.linear`
/// of the surface, merge into one hit). Against a NURBS surface the box search stops at 1e-3 of
/// the larger bounding-box diagonal before the Newton polish, and hits within `20 tol.linear`
/// merge: two contacts closer than about that leaf size in one box may be reported as one.
pub fn curve_surface(curve: &CurveDef, range: Option<Interval>, surface: &SurfaceDef, tol: &ModelTolerance) -> Result<CurveSurface, GeomError> {
    match surface {
        SurfaceDef::Plane(p) => return curve_plane(curve, range, p, tol),
        SurfaceDef::Nurbs(n) => {
            let ps = PreparedSurface::new(n)?;
            return curve_prepared_surface(curve, range, &ps, tol);
        }
        _ => {}
    }
    let imp = surface.implicit_def().ok_or(GeomError::Unsupported("surface has no implicit form"))?;
    let r = domain_range(curve, range);
    let raw: Vec<Raw> = match curve {
        CurveDef::Line(l) => {
            let ray = Ray { origin: l.origin, dir: l.dir };
            let mut hits = Vec::new();
            match surface.as_surface() {
                Some(s) => s.ray_hits(&ray, r, &mut hits)?,
                None => imp.ray_hits(&ray, r, &mut hits)?,
            }
            hits.into_iter().map(|h| (h.t, h.crossing == Crossing::Tangent)).collect()
        }
        CurveDef::Circle(c) => match trig_vs_implicit(&c.frame, c.radius, c.radius, &imp, tol, r) {
            Some(v) => v,
            None => return Ok(CurveSurface::Coincident),
        },
        CurveDef::Ellipse(e) => match trig_vs_implicit(&e.frame, e.a, e.b, &imp, tol, r) {
            Some(v) => v,
            None => return Ok(CurveSurface::Coincident),
        },
        CurveDef::Nurbs(n) => {
            let pc = PreparedCurve::new(n)?;
            match nurbs_vs_implicit(&pc, &imp, tol, r)? {
                Some(v) => v,
                None => return Ok(CurveSurface::Coincident),
            }
        }
        _ => return Err(GeomError::Unsupported("curve kind")),
    };
    let c = curve.as_curve();
    let raw: Vec<Raw> = raw.into_iter().filter(|(t, _)| nappe_ok(surface, c.eval(*t), tol.linear)).collect();
    Ok(CurveSurface::Points(build_hits(curve, surface, raw, 5.0 * tol.linear)))
}

/// Cones are one nappe parametrically; their implicit forms are double cones.
fn nappe_ok(surface: &SurfaceDef, p: Point3, eps: f64) -> bool {
    match surface {
        SurfaceDef::Cone(c) => (p - c.apex()).dot(c.frame.z) >= -eps,
        SurfaceDef::EllipticCone(c) => (p - c.apex()).dot(c.frame.z) >= -eps,
        _ => true,
    }
}

// ---- circle / ellipse against a quadric or torus: trigonometric polynomial

/// `None` when the whole curve lies on the surface (to tolerance).
fn trig_vs_implicit(f: &sm_geom_api::Frame, a: f64, b: f64, imp: &ImplicitDef, tol: &ModelTolerance, r: Interval) -> Option<Vec<Raw>> {
    let at = |t: f64| -> Point3 {
        let (s, c) = t.sin_cos();
        f.origin + f.x * (a * c) + f.y * (b * s)
    };
    let g = |t: f64| imp.value(at(t));
    let d = match imp {
        ImplicitDef::Torus(_) => 4usize,
        _ => 2usize,
    };
    let vals: Vec<f64> = (0..16).map(|k| g(TAU * k as f64 / 16.0)).collect();
    let scale = vals.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    let on_surface = |t: f64| {
        let p = at(t);
        imp.value(p).abs() <= tol.linear * imp.gradient(p).norm().max(1e-300)
    };
    if (0..16).all(|k| on_surface(TAU * k as f64 / 16.0)) && on_surface(0.123) {
        return None;
    }
    if scale == 0.0 {
        return Some(Vec::new());
    }
    let gp = |t: f64| {
        let (s, c) = t.sin_cos();
        let p = at(t);
        imp.gradient(p).dot(f.x * (-a * s) + f.y * (b * c))
    };
    let ts = trig_poly_roots(&g, d);
    // Polish with Newton on the true restriction (the polynomial route loses a few digits).
    let dist = |t: f64| {
        let p = at(t);
        g(t).abs() / imp.gradient(p).norm().max(1e-300)
    };
    let mut roots: Vec<(f64, bool)> = Vec::new();
    for (t0, tan) in ts {
        let mut t = t0;
        if !tan {
            for _ in 0..4 {
                let (v, dv) = (g(t), gp(t));
                if dv.abs() < 1e-300 {
                    break;
                }
                let nt = t - v / dv;
                if (g(nt)).abs() <= v.abs() {
                    t = nt;
                } else {
                    break;
                }
            }
        }
        // Drop numerical roots that are not actually on the surface (conditioning of the
        // polynomial route): require the residual to be within the tolerance.
        if dist(t) <= if tan { 100.0 * tol.linear } else { 10.0 * tol.linear } {
            roots.push((t.rem_euclid(TAU), tan));
        }
    }
    // Near-tangent contacts: a local extremum of `g` that does not cross zero but lies within
    // `tol.linear` of the surface is a contact, like the gap rule of the other closed forms.
    // Roots on the same stretch (the curve stays within tolerance between them) collapse into it.
    let mut crit = trig_poly_roots(&gp, d);
    crit.sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut contacts: Vec<f64> = Vec::new();
    for (tc, _) in crit {
        let mut t = tc;
        for _ in 0..3 {
            // Newton on g' with a central difference for g''.
            let h = 1e-5;
            let g2 = (gp(t + h) - gp(t - h)) / (2.0 * h);
            if g2.abs() < 1e-300 {
                break;
            }
            let nt = t - gp(t) / g2;
            if (nt - t).abs() > 1e-3 {
                break;
            }
            t = nt;
        }
        if dist(t) <= tol.linear {
            contacts.push(t.rem_euclid(TAU));
        }
    }
    let within = |t0: f64, t1: f64| {
        // Shorter arc from t0 to t1; true when the curve stays within tolerance along it.
        let mut dt = (t1 - t0).rem_euclid(TAU);
        if dt > core::f64::consts::PI {
            dt -= TAU;
        }
        (0..=8).all(|i| dist(t0 + dt * i as f64 / 8.0) <= tol.linear)
    };
    for &tc in &contacts {
        roots.retain(|(t, _)| !within(tc, *t));
    }
    contacts.dedup_by(|a, b| within(*a, *b));
    roots.extend(contacts.into_iter().map(|t| (t, true)));
    let mut out = Vec::new();
    for (w, tan) in roots {
        for tt in periodic_in_range(w, TAU, r) {
            out.push((tt, tan));
        }
    }
    Some(out)
}

/// Real roots `(t, double)` in `[0, 2 pi)` of the trigonometric polynomial `g` of degree `d`
/// (`x = tan(t/2)` and polynomial roots). Coefficients come from 16 equispaced samples.
fn trig_poly_roots(g: &dyn Fn(f64) -> f64, d: usize) -> Vec<(f64, bool)> {
    // Exact trigonometric coefficients from 16 equispaced samples.
    const N: usize = 16;
    let vals: Vec<f64> = (0..N).map(|k| g(TAU * k as f64 / N as f64)).collect();
    let scale = vals.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    let mean = vals.iter().sum::<f64>() / N as f64;
    let mut am = vec![0.0; d + 1];
    let mut bm = vec![0.0; d + 1];
    am[0] = mean;
    for m in 1..=d {
        for (k, v) in vals.iter().enumerate() {
            let ang = TAU * (k * m) as f64 / N as f64;
            am[m] += 2.0 / N as f64 * v * ang.cos();
            bm[m] += 2.0 / N as f64 * v * ang.sin();
        }
    }
    if scale == 0.0 {
        return Vec::new();
    }
    // P(x) = sum over harmonics of (1+x^2)^(d-m) * (a_m Re (1+ix)^(2m) + b_m Im (1+ix)^(2m)).
    let poly = |m: usize| -> (Vec<f64>, Vec<f64>) {
        // (1 + i x)^(2m) as (re, im) coefficient vectors in ascending powers of x.
        let mut re = vec![1.0];
        let mut im = vec![0.0];
        for _ in 0..2 * m {
            let (mut nre, mut nim) = (vec![0.0; re.len() + 1], vec![0.0; re.len() + 1]);
            for k in 0..re.len() {
                // multiply by (1 + i x): (re + i im)(1 + i x)
                nre[k] += re[k];
                nim[k] += im[k];
                nre[k + 1] -= im[k];
                nim[k + 1] += re[k];
            }
            re = nre;
            im = nim;
        }
        (re, im)
    };
    let one_plus_x2 = |pow: usize| -> Vec<f64> {
        let mut c = vec![1.0];
        for _ in 0..pow {
            let mut n = vec![0.0; c.len() + 2];
            for (k, v) in c.iter().enumerate() {
                n[k] += v;
                n[k + 2] += v;
            }
            c = n;
        }
        c
    };
    let mul = |p: &[f64], q: &[f64]| -> Vec<f64> {
        let mut o = vec![0.0; p.len() + q.len() - 1];
        for (i, x) in p.iter().enumerate() {
            for (j, y) in q.iter().enumerate() {
                o[i + j] += x * y;
            }
        }
        o
    };
    let mut total = vec![0.0; 2 * d + 1];
    let base = one_plus_x2(d);
    for (k, v) in base.iter().enumerate() {
        total[k] += am[0] * v;
    }
    for m in 1..=d {
        let (re, im) = poly(m);
        let sc = one_plus_x2(d - m);
        let pr = mul(&sc, &re);
        let pi = mul(&sc, &im);
        for k in 0..pr.len() {
            total[k] += am[m] * pr[k] + bm[m] * pi[k];
        }
    }
    let mut ts: Vec<(f64, bool)> = Vec::new();
    for rt in poly_roots(&total, f64::NEG_INFINITY, f64::INFINITY) {
        ts.push((2.0 * rt.t.atan(), rt.double));
    }
    // The point at x = infinity (t = pi) is a root when the leading coefficient vanishes.
    let lead = total[2 * d].abs();
    let tmax = total.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    if lead <= 1e-12 * tmax && g(core::f64::consts::PI).abs() <= 1e-9 * scale {
        // Even multiplicity (the restriction keeps its sign on both sides) is a contact.
        let h = 1e-4;
        let tangent = g(core::f64::consts::PI - h) * g(core::f64::consts::PI + h) > 0.0;
        ts.push((core::f64::consts::PI, tangent));
    }
    ts
}

// ---- NURBS curve against an implicit surface

struct CNode {
    lo: f64,
    hi: f64,
    poles: SegPoles,
    depth: u32,
}

/// `None` = the curve lies on the surface (sampled).
fn nurbs_vs_implicit(c: &PreparedCurve, imp: &ImplicitDef, tol: &ModelTolerance, r: Interval) -> Result<Option<Vec<Raw>>, GeomError> {
    let bb = c.control_bbox();
    let scale = ((bb[3] - bb[0]).powi(2) + (bb[4] - bb[1]).powi(2) + (bb[5] - bb[2]).powi(2)).sqrt().max(1e-300);
    // Leaves are fixed by geometry, not by the curve's extent: small enough to separate two
    // contacts a few `tol.linear` apart, never finer than that (a coarser start for long
    // curves only costs depth, one level per halving).
    let leaf = (20.0 * tol.linear).min(1e-4 * scale).max(1e-12 * scale);
    let mut found: Vec<Raw> = Vec::new();
    let mut stack: Vec<CNode> = c.segs.iter().rev().filter(|s| s.hi >= r.lo && s.lo <= r.hi).map(|s| CNode { lo: s.lo, hi: s.hi, poles: s.poles.clone(), depth: 0 }).collect();
    let mut budget = 200_000usize;
    let mut leaves = 0usize;
    while let Some(n) = stack.pop() {
        if budget == 0 {
            break;
        }
        budget -= 1;
        let bx = bbox_of(n.poles.iter().map(|h| hp_point(*h)));
        let (flo, fhi) = interval::range(imp, &bx);
        let mid = 0.5 * (bx[0] + bx[3]);
        let _ = mid;
        let gscale = imp.gradient(Point3::new(0.5 * (bx[0] + bx[3]), 0.5 * (bx[1] + bx[4]), 0.5 * (bx[2] + bx[5]))).norm();
        let slack = tol.linear * gscale * 0.5;
        if flo > slack || fhi < -slack {
            continue;
        }
        let diam = ((bx[3] - bx[0]).powi(2) + (bx[4] - bx[1]).powi(2) + (bx[5] - bx[2]).powi(2)).sqrt();
        if diam <= leaf || n.depth >= 50 {
            leaves += 1;
            if let Some(h) = refine_implicit(c, imp, 0.5 * (n.lo + n.hi), tol, r) {
                found.push(h);
            }
            continue;
        }
        let (mut l, mut rr) = (SegPoles::from_elem([0.0; 4], n.poles.len()), SegPoles::from_elem([0.0; 4], n.poles.len()));
        split_half_into(&n.poles, &mut l, &mut rr);
        let m = 0.5 * (n.lo + n.hi);
        stack.push(CNode { lo: m, hi: n.hi, poles: rr, depth: n.depth + 1 });
        stack.push(CNode { lo: n.lo, hi: m, poles: l, depth: n.depth + 1 });
    }
    if budget == 0 {
        // Too many candidate pieces: the curve runs along the surface or the bounds are loose.
        let n = 33;
        let on = (0..n).all(|i| {
            let t = c.range().lo + c.range().len() * i as f64 / (n - 1) as f64;
            let p = c.eval(t);
            imp.value(p).abs() / imp.gradient(p).norm().max(1e-300) <= tol.linear
        });
        return if on { Ok(None) } else { Err(GeomError::NoConvergence("curve/surface subdivision budget exhausted")) };
    }
    let _ = leaves;
    // One hit per contact stretch (see `contact`): independent of where Newton stopped, of
    // knots and of the seam of a closed curve.
    let signed = |t: f64| {
        let p = c.eval(t);
        imp.value(p) / imp.gradient(p).norm().max(1e-300)
    };
    let (lo, hi) = (c.range().lo, c.range().hi);
    let closed = c.domain().is_periodic() || (c.eval(lo) - c.eval(hi)).norm() <= tol.linear;
    let h0 = |t: f64| 0.5 * tol.linear / c.jet(t).d1.norm().max(1e-300);
    let seed = found.first().map_or(lo, |h| h.0);
    let gap = Gap { f: &signed, lo, hi, closed, tol: tol.linear, h0: h0(seed) };
    let merged: Vec<Raw> = merge_contacts(&gap, &found).into_iter().filter(|h| r.contains(h.0, 0.0)).collect();
    Ok(Some(merged))
}

/// Newton on `g(t) = f(C(t))` from `t0`; accept when the point is within `tol` of the surface.
fn refine_implicit(c: &PreparedCurve, imp: &ImplicitDef, t0: f64, tol: &ModelTolerance, r: Interval) -> Option<Raw> {
    let dom = c.domain();
    let fix = |t: f64| if dom.is_periodic() { dom.wrap(t) } else { dom.range.clamp(t) };
    let mut t = fix(t0);
    let mut best = (f64::INFINITY, t);
    for _ in 0..80 {
        let j = c.jet(t);
        let gv = imp.value(j.p);
        let gr = imp.gradient(j.p);
        let gn = gr.norm().max(1e-300);
        let dist = gv.abs() / gn;
        if dist < best.0 {
            best = (dist, t);
        }
        if dist <= 1e-3 * tol.linear {
            break;
        }
        let gp = gr.dot(j.d1);
        if gp.abs() <= 1e-14 * gn * j.d1.norm() {
            break;
        }
        let nt = fix(t - gv / gp);
        if (nt - t).abs() <= 1e-16 * (1.0 + t.abs()) {
            break;
        }
        t = nt;
    }
    let t = best.1;
    if best.0 > tol.linear || !r.contains(t, 0.0) {
        return None;
    }
    let j = c.jet(t);
    let gr = imp.gradient(j.p);
    let tangent = gr.dot(j.d1).abs() <= 1e-6 * gr.norm() * j.d1.norm();
    Some((t, tangent))
}

// ---- curve against a NURBS surface

/// Intersect a curve with an already prepared NURBS surface (see [`curve_surface`]).
pub fn curve_prepared_surface(curve: &CurveDef, range: Option<Interval>, ps: &PreparedSurface, tol: &ModelTolerance) -> Result<CurveSurface, GeomError> {
    let r = domain_range(curve, range);
    // Express the curve as a NURBS: exact conversions for circles and ellipses; lines are
    // clipped to the surface box.
    let (pc, remap): (PreparedCurve, Option<&dyn ParametricCurve>) = match curve {
        CurveDef::Nurbs(n) => (PreparedCurve::new(n)?, None),
        CurveDef::Circle(c) => (PreparedCurve::new(&exact::circle_to_nurbs(c)?)?, Some(c as &dyn ParametricCurve)),
        CurveDef::Ellipse(e) => (PreparedCurve::new(&exact::ellipse_to_nurbs(e)?)?, Some(e as &dyn ParametricCurve)),
        CurveDef::Line(l) => {
            let Some((t0, t1)) = clip_to_box(l.origin, l.dir, &ps.control_bbox(), r, tol.linear) else { return Ok(CurveSurface::Points(Vec::new())) };
            let seg = NurbsCurve::new(1, vec![t0, t0, t1, t1], vec![l.origin + l.dir * t0, l.origin + l.dir * t1], None)?;
            (PreparedCurve::new(&seg)?, None)
        }
        _ => return Err(GeomError::Unsupported("curve kind")),
    };
    let hits = pair_hits(&pc, ps, if remap.is_some() || matches!(curve, CurveDef::Line(_)) { pc.range() } else { r }, tol)?;
    // One hit per contact stretch, before any periodic replication.
    let hits = classify_pair_hits(&pc, ps, hits, tol);
    let mut out: Vec<CurveSurfaceHit> = Vec::new();
    for h in hits {
        match remap {
            // The search ran over the principal period: replicate each hit over every period
            // that meets the caller's range (as `trig_vs_implicit` and `ellipse_plane` do).
            Some(a) => {
                let t = a.project(h.point, None).map(|p| p.t).unwrap_or(h.t).rem_euclid(TAU);
                for tt in periodic_in_range(t, TAU, r) {
                    out.push(CurveSurfaceHit { t: tt, point: h.point, uv: Some(h.uv), tangent: h.tangent });
                }
            }
            None => {
                if r.contains(h.t, 0.0) {
                    out.push(CurveSurfaceHit { t: h.t, point: h.point, uv: Some(h.uv), tangent: h.tangent });
                }
            }
        }
    }
    out.sort_by(|a, b| a.t.total_cmp(&b.t));
    Ok(CurveSurface::Points(out))
}

/// Signed gap from `q` (near the surface) to `s` along the surface normal, and the foot's
/// parameters. Local Gauss-Newton from `seed`, global projection if that does not converge.
fn signed_gap_surface(s: &PreparedSurface, q: Point3, seed: Uv) -> (Uv, f64) {
    let dom = s.domain();
    let fixu = |q: Uv| {
        let f = |d: &sm_geom_api::Domain1, x: f64| if d.is_periodic() { d.wrap(x) } else { d.range.clamp(x) };
        Uv::new(f(&dom.u, q.u), f(&dom.v, q.v))
    };
    let scale = diam(&s.control_bbox()).max(1e-300);
    let mut uv = fixu(seed);
    let mut ok = false;
    for _ in 0..16 {
        let j = s.jet(uv);
        let r = j.p - q;
        let (a, b, c) = (j.du.dot(j.du), j.du.dot(j.dv), j.dv.dot(j.dv));
        let (gu, gv) = (-r.dot(j.du), -r.dot(j.dv));
        let damp = 1e-12 * (a + c).max(1e-300);
        let det = (a + damp) * (c + damp) - b * b;
        if det.abs() < 1e-300 {
            break;
        }
        let (du, dv) = ((gu * (c + damp) - gv * b) / det, (gv * (a + damp) - gu * b) / det);
        let nuv = fixu(Uv::new(uv.u + du, uv.v + dv));
        let moved = (nuv.u - uv.u).abs() + (nuv.v - uv.v).abs();
        uv = nuv;
        if moved <= 1e-14 * (1.0 + uv.u.abs() + uv.v.abs()) {
            ok = true;
            break;
        }
    }
    let j = s.jet(uv);
    let r = q - j.p;
    // Converged to a foot point, near the query (a far result may be another sheet).
    let good = ok && r.norm() <= 1e-3 * scale && r.dot(j.du).abs() <= 1e-7 * r.norm().max(1e-12 * scale) * j.du.norm() + 1e-14 * scale * j.du.norm() && r.dot(j.dv).abs() <= 1e-7 * r.norm().max(1e-12 * scale) * j.dv.norm() + 1e-14 * scale * j.dv.norm();
    let (uv, j, r) = if good {
        (uv, j, r)
    } else {
        let pr = crate::nurbs::project::project_surface(s, q, Some(seed));
        let j = s.jet(pr.uv);
        (pr.uv, j, q - j.p)
    };
    let n = j.du.cross(j.dv);
    let nn = n.norm();
    let sd = if nn > 0.0 { r.dot(n) / nn } else { r.norm() };
    (uv, sd)
}

/// Replace the hits of `pair_hits` by one hit per contact stretch (see `contact`).
fn classify_pair_hits(c: &PreparedCurve, s: &PreparedSurface, hits: Vec<PairHit>, tol: &ModelTolerance) -> Vec<PairHit> {
    if hits.is_empty() {
        return hits;
    }
    let (lo, hi) = (c.range().lo, c.range().hi);
    let closed = c.domain().is_periodic() || (c.eval(lo) - c.eval(hi)).norm() <= tol.linear;
    let span = hi - lo;
    // Seed each gap evaluation from the nearest hit (circularly on a closed curve).
    let nearest = |t: f64| {
        let d = |h: &PairHit| {
            let x = (h.t - t).abs();
            if closed { x.min(span - x) } else { x }
        };
        hits.iter().min_by(|a, b| d(a).total_cmp(&d(b))).map_or(Uv::new(0.0, 0.0), |h| h.uv)
    };
    let signed = |t: f64| signed_gap_surface(s, c.eval(t), nearest(t)).1;
    let h0 = 0.5 * tol.linear / c.jet(hits[0].t).d1.norm().max(1e-300);
    let gap = Gap { f: &signed, lo, hi, closed, tol: tol.linear, h0 };
    let raw: Vec<Raw> = hits.iter().map(|h| (h.t, h.tangent)).collect();
    merge_contacts(&gap, &raw)
        .into_iter()
        .map(|(t, tangent)| {
            let point = c.eval(t);
            let (uv, _) = signed_gap_surface(s, point, nearest(t));
            PairHit { t, uv, point, tangent }
        })
        .collect()
}

/// Slab clip of `origin + t dir` against a box inflated by `pad`; `None` if empty or unbounded
/// in a way that cannot be clipped.
fn clip_to_box(origin: Point3, dir: Vector3, b: &[f64; 6], r: Interval, pad: f64) -> Option<(f64, f64)> {
    let (mut t0, mut t1) = (r.lo, r.hi);
    let o = [origin.x, origin.y, origin.z];
    let d = [dir.x, dir.y, dir.z];
    for k in 0..3 {
        let (lo, hi) = (b[k] - pad, b[k + 3] + pad);
        if d[k].abs() < 1e-300 {
            if o[k] < lo || o[k] > hi {
                return None;
            }
        } else {
            let (a, c) = ((lo - o[k]) / d[k], (hi - o[k]) / d[k]);
            t0 = t0.max(a.min(c));
            t1 = t1.min(a.max(c));
        }
    }
    (t0.is_finite() && t1.is_finite() && t1 >= t0).then_some((t0, if t1 > t0 { t1 } else { t0 + 1e-9 }))
}

/// A curve/NURBS-surface meeting point.
#[derive(Clone, Copy, Debug)]
pub(crate) struct PairHit {
    pub t: f64,
    pub uv: Uv,
    pub point: Point3,
    pub tangent: bool,
}

struct PairNode {
    c_lo: f64,
    c_hi: f64,
    c_poles: SegPoles,
    c_box: [f64; 6],
    u: Interval,
    v: Interval,
    s_poles: PatchPoles,
    s_box: [f64; 6],
    depth: u32,
}

fn overlap(a: &[f64; 6], b: &[f64; 6], pad: f64) -> bool {
    (0..3).all(|k| a[k] <= b[k + 3] + pad && b[k] <= a[k + 3] + pad)
}

fn diam(b: &[f64; 6]) -> f64 {
    ((b[3] - b[0]).powi(2) + (b[4] - b[1]).powi(2) + (b[5] - b[2]).powi(2)).sqrt()
}

/// All intersections of the curve (restricted to `cr`) with the surface.
pub(crate) fn pair_hits(c: &PreparedCurve, s: &PreparedSurface, cr: Interval, tol: &ModelTolerance) -> Result<Vec<PairHit>, GeomError> {
    let (pu, pv) = s.degrees();
    let scale = diam(&s.control_bbox()).max(diam(&c.control_bbox())).max(1e-300);
    let leaf = 1e-3 * scale;
    let pad = 1.01 * tol.linear;
    let mut stack: Vec<PairNode> = Vec::new();
    for cs in c.segs.iter().rev().filter(|x| x.hi >= cr.lo && x.lo <= cr.hi) {
        for p in s.patches.iter().rev() {
            if overlap(&cs.bbox, &p.bbox, pad) {
                stack.push(PairNode { c_lo: cs.lo, c_hi: cs.hi, c_poles: cs.poles.clone(), c_box: cs.bbox, u: p.u, v: p.v, s_poles: p.poles.clone(), s_box: p.bbox, depth: 0 });
            }
        }
    }
    let mut budget = 400_000usize;
    let mut found: Vec<PairHit> = Vec::new();
    while let Some(n) = stack.pop() {
        if budget == 0 {
            return Err(GeomError::NoConvergence("curve/surface subdivision budget exhausted"));
        }
        budget -= 1;
        let (dc, ds) = (diam(&n.c_box), diam(&n.s_box));
        if (dc <= leaf && ds <= leaf) || n.depth >= 60 {
            if let Some(h) = refine_pair(c, s, 0.5 * (n.c_lo + n.c_hi), Uv::new(0.5 * (n.u.lo + n.u.hi), 0.5 * (n.v.lo + n.v.hi)), tol)
                && cr.contains(h.t, 0.0)
            {
                found.push(h);
            }
            continue;
        }
        if dc >= ds {
            let (mut l, mut r) = (SegPoles::from_elem([0.0; 4], n.c_poles.len()), SegPoles::from_elem([0.0; 4], n.c_poles.len()));
            split_half_into(&n.c_poles, &mut l, &mut r);
            let m = 0.5 * (n.c_lo + n.c_hi);
            for (pl, lo, hi) in [(r, m, n.c_hi), (l, n.c_lo, m)] {
                let bx = bbox_of(pl.iter().map(|h| hp_point(*h)));
                if overlap(&bx, &n.s_box, pad) {
                    stack.push(PairNode { c_lo: lo, c_hi: hi, c_poles: pl, c_box: bx, u: n.u, v: n.v, s_poles: n.s_poles.clone(), s_box: n.s_box, depth: n.depth + 1 });
                }
            }
        } else {
            let along_u = n.depth % 2 == 0;
            let (l, r) = split_patch(&n.s_poles, pu, pv, along_u);
            let halves = if along_u {
                let m = 0.5 * (n.u.lo + n.u.hi);
                [(r, Interval::new(m, n.u.hi), n.v), (l, Interval::new(n.u.lo, m), n.v)]
            } else {
                let m = 0.5 * (n.v.lo + n.v.hi);
                [(r, n.u, Interval::new(m, n.v.hi)), (l, n.u, Interval::new(n.v.lo, m))]
            };
            for (pl, cu, cv) in halves {
                let bx = bbox_of(pl.iter().map(|h| hp_point(*h)));
                if overlap(&bx, &n.c_box, pad) {
                    stack.push(PairNode { c_lo: n.c_lo, c_hi: n.c_hi, c_poles: n.c_poles.clone(), c_box: n.c_box, u: cu, v: cv, s_poles: pl, s_box: bx, depth: n.depth + 1 });
                }
            }
        }
    }
    // Deduplicate hits found from neighbouring leaves.
    found.sort_by(|a, b| a.t.total_cmp(&b.t).then(a.uv.u.total_cmp(&b.uv.u)).then(a.uv.v.total_cmp(&b.uv.v)));
    let mut out: Vec<PairHit> = Vec::new();
    for h in found {
        match out.iter_mut().find(|o| (o.point - h.point).norm() <= 20.0 * tol.linear.max(1e-12 * scale)) {
            Some(o) => o.tangent |= h.tangent,
            None => out.push(h),
        }
    }
    // False crossings either side of a grazing contact: neighbouring hits are one contact when
    // the curve stays within `tol.linear` of the surface all along between them.
    out.sort_by(|a, b| a.t.total_cmp(&b.t));
    let near = |t: f64, uv: Uv| crate::nurbs::project::project_surface(s, c.eval(t), Some(uv)).distance <= tol.linear;
    let mut merged: Vec<PairHit> = Vec::new();
    for h in out {
        match merged.last_mut() {
            Some(l) if (l.point - h.point).norm() <= 1e-2 * scale && (1..8).all(|i| {
                let f = i as f64 / 8.0;
                near(l.t + (h.t - l.t) * f, Uv::new(l.uv.u + (h.uv.u - l.uv.u) * f, l.uv.v + (h.uv.v - l.uv.v) * f))
            }) =>
            {
                // Keep the exact tangent hit when there is one, else the better-centred one.
                if h.tangent && !l.tangent {
                    *l = h;
                }
                l.tangent = true;
            }
            _ => merged.push(h),
        }
    }
    Ok(merged)
}

/// Levenberg-Marquardt on `C(t) - S(u, v) = 0`.
fn refine_pair(c: &PreparedCurve, s: &PreparedSurface, t0: f64, uv0: Uv, tol: &ModelTolerance) -> Option<PairHit> {
    let cdom = c.domain();
    let sdom = s.domain();
    let fixt = |t: f64| if cdom.is_periodic() { cdom.wrap(t) } else { cdom.range.clamp(t) };
    let fixu = |q: Uv| {
        let f = |d: &sm_geom_api::Domain1, x: f64| if d.is_periodic() { d.wrap(x) } else { d.range.clamp(x) };
        Uv::new(f(&sdom.u, q.u), f(&sdom.v, q.v))
    };
    let (mut t, mut q) = (fixt(t0), fixu(uv0));
    let resid = |t: f64, q: Uv| c.eval(t) - s.eval(q);
    let mut lambda = 1e-6;
    let mut r = resid(t, q);
    let mut rn = r.norm();
    for _ in 0..80 {
        if rn <= 1e-3 * tol.linear {
            break;
        }
        let cj = c.jet(t);
        let sj = s.jet(q);
        let jac = [[cj.d1.x, -sj.du.x, -sj.dv.x], [cj.d1.y, -sj.du.y, -sj.dv.y], [cj.d1.z, -sj.du.z, -sj.dv.z]];
        let rv = [r.x, r.y, r.z];
        // Normal equations with damping.
        let mut a = [[0.0; 3]; 3];
        let mut g = [0.0; 3];
        for i in 0..3 {
            for j in 0..3 {
                a[i][j] = (0..3).map(|k| jac[k][i] * jac[k][j]).sum();
            }
            g[i] = -(0..3).map(|k| jac[k][i] * rv[k]).sum::<f64>();
        }
        let dscale = a[0][0].max(a[1][1]).max(a[2][2]).max(1e-300);
        let mut moved = false;
        for _ in 0..12 {
            let mut ad = a;
            for (i, row) in ad.iter_mut().enumerate() {
                row[i] += lambda * dscale;
            }
            if let Some(d) = solve_dense(ad, g) {
                let (nt, nq) = (fixt(t + d[0]), fixu(Uv::new(q.u + d[1], q.v + d[2])));
                let nr = resid(nt, nq);
                if nr.norm() < rn {
                    t = nt;
                    q = nq;
                    r = nr;
                    let step = d[0].abs() + d[1].abs() + d[2].abs();
                    rn = nr.norm();
                    lambda = (lambda * 0.3).max(1e-14);
                    moved = step > 1e-15;
                    break;
                }
            }
            lambda = (lambda * 10.0).min(1e12);
        }
        if !moved {
            break;
        }
    }
    if rn > tol.linear {
        return None;
    }
    let cj = c.jet(t);
    let tangent = s.normal(q).map(|n| cj.d1.dot(n).abs() <= 1e-6 * cj.d1.norm()).unwrap_or(true);
    Some(PairHit { t, uv: q, point: c.eval(t), tangent })
}

// ------------------------------------------------------------------ ray hook for NURBS surfaces

/// Ray against a prepared NURBS surface: the implementation behind
/// `PreparedSurface::ray_hits` (the contract's reference NURBS surface returns `Unsupported`).
/// Hits ascend in `t`; `normal` is the surface normal `du x dv` (zero at a pole); `crossing` is
/// relative to it.
pub(crate) fn ray_nurbs_surface(s: &PreparedSurface, ray: &Ray, range: Interval, out: &mut Vec<RayHit>) -> Result<(), GeomError> {
    let diag = diam(&s.control_bbox());
    let tol = ModelTolerance::for_extent(diag.max(1e-12));
    let Some((t0, t1)) = clip_to_box(ray.origin, ray.dir, &s.control_bbox(), range, tol.linear) else { return Ok(()) };
    let seg = NurbsCurve::new(1, vec![t0, t0, t1, t1], vec![ray.at(t0), ray.at(t1)], None)?;
    let pc = PreparedCurve::new(&seg)?;
    let mut hits = pair_hits(&pc, s, pc.range(), &tol)?;
    hits.sort_by(|a, b| a.t.total_cmp(&b.t));
    let dn = ray.dir.norm();
    for h in hits {
        if !range.contains(h.t, 0.0) {
            continue;
        }
        let normal = s.normal(h.uv).unwrap_or_default();
        let along = if dn > 0.0 { ray.dir.dot(normal) / dn } else { 0.0 };
        let crossing = if h.tangent || along.abs() <= 1e-12 {
            Crossing::Tangent
        } else if along > 0.0 {
            Crossing::AlongNormal
        } else {
            Crossing::AgainstNormal
        };
        out.push(RayHit::new(h.t, h.point, normal, Some(h.uv), crossing));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::PI;

    #[test]
    fn root_at_pi_from_a_vanishing_leading_coefficient_carries_its_multiplicity() {
        // 1 + cos t = 2 / (1 + x^2) in x = tan(t / 2): the polynomial is constant, its leading
        // coefficient vanishes, and t = pi is a double root (a contact).
        let r = trig_poly_roots(&|t: f64| 1.0 + t.cos(), 1);
        assert_eq!(r.len(), 1, "{r:?}");
        assert!((r[0].0 - PI).abs() < 1e-12 && r[0].1, "{r:?}");
        // The same with the opposite sign and a degree-2 polynomial: (1 + cos t)^2.
        let r = trig_poly_roots(&|t: f64| -(1.0 + t.cos()).powi(2), 2);
        assert!(r.iter().any(|(t, tan)| (t - PI).abs() < 1e-12 && *tan), "{r:?}");
        // sin t: simple roots at 0 and pi, both crossings.
        let r = trig_poly_roots(&|t: f64| t.sin(), 1);
        assert_eq!(r.len(), 2, "{r:?}");
        assert!(r.iter().all(|(_, tan)| !tan), "{r:?}");
        assert!(r.iter().any(|(t, _)| (t - PI).abs() < 1e-12), "{r:?}");
    }
}
