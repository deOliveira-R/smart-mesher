//! Plane against a quadric in closed form: the restriction is a conic, classified and returned
//! as an exact curve (line, circle, ellipse, or a rational NURBS chain for parabola and
//! hyperbola branches clipped to a window).

use super::ssi::{IntersectionCurve, SsiResult};
use crate::nurbs::exact::conic_arc;
use sm_geom_api::roots::{Root, solve_quadratic};
use core::f64::consts::TAU;
use sm_core::{Point3, Vector3};
use sm_geom_api::{Circle, CurveDef, Ellipse, Frame, GeomError, ImplicitSurface, Interval, Line, ModelTolerance, NurbsCurve, Plane, Quadric};

/// Classified conic in 3D.
#[derive(Clone, Debug)]
pub(crate) enum Section {
    Empty,
    /// The plane lies in the quadric (it is a plane quadric equal to it).
    Coincident,
    Point(Point3),
    Ellipse { center: Point3, x: Vector3, y: Vector3, a: f64, b: f64 },
    Parabola { vertex: Point3, perp: Vector3, axis: Vector3, focal: f64 },
    Hyperbola { center: Point3, main: Vector3, other: Vector3, a: f64, b: f64 },
    Lines(Vec<(Point3, Vector3)>),
}

fn v3(p: Point3) -> Vector3 {
    Vector3::new(p.x, p.y, p.z)
}

fn quad_form(q: &[[f64; 3]; 3], a: Vector3, b: Vector3) -> f64 {
    let qb = Vector3::new(q[0][0] * b.x + q[0][1] * b.y + q[0][2] * b.z, q[1][0] * b.x + q[1][1] * b.y + q[1][2] * b.z, q[2][0] * b.x + q[2][1] * b.y + q[2][2] * b.z);
    a.dot(qb)
}

/// Classify `plane ∩ {q = 0}`.
///
/// Tangent contacts are decided by a distance: the gap between the plane and the quadric at the
/// stationary point of the restriction (`|q| / |grad q|`, first order) is compared with
/// `tol.linear`. The stationary values are evaluated on the quadric itself at world points, not
/// from the plane-origin-relative expansion, so re-anchoring the plane origin does not change
/// the classification.
pub(crate) fn plane_quadric(plane: &Plane, q: &Quadric, tol: &ModelTolerance) -> Section {
    let gap_at = |p: Point3| q.value(p).abs() / q.gradient(p).norm().max(1e-300);
    let [a, b, c, d, e, f, g, h, j, k] = q.coeffs;
    let qm = [[a, d / 2.0, f / 2.0], [d / 2.0, b, e / 2.0], [f / 2.0, e / 2.0, c]];
    let l = Vector3::new(g, h, j);
    let fr = &plane.frame;
    let o = v3(fr.origin);
    let (x, y) = (fr.x, fr.y);
    // Restriction to the plane: A u^2 + B u v + C v^2 + D u + E v + F.
    let aa = quad_form(&qm, x, x);
    let bb = 2.0 * quad_form(&qm, x, y);
    let cc = quad_form(&qm, y, y);
    let dd = 2.0 * quad_form(&qm, x, o) + l.dot(x);
    let ee = 2.0 * quad_form(&qm, y, o) + l.dot(y);
    let ff = quad_form(&qm, o, o) + l.dot(o) + k;
    let qglob = [a, b, c, d, e, f].iter().fold(0.0f64, |m, v| m.max(v.abs()));
    let world = |u: f64, w: f64| fr.origin + x * u + y * w;
    let dir = |u: f64, w: f64| x * u + y * w;
    if aa.abs().max(bb.abs()).max(cc.abs()) <= 1e-12 * qglob.max(1e-300) {
        // Linear in the plane: a line, nothing, or the whole plane.
        let lin = dd.hypot(ee);
        let scale = lin.max(ff.abs());
        if lin <= 1e-12 * scale.max(1e-300) {
            return if ff.abs() <= 1e-12 * l.norm().max(qglob).max(1e-300) { Section::Coincident } else { Section::Empty };
        }
        let n2 = lin * lin;
        let p = world(-dd * ff / n2, -ee * ff / n2);
        return Section::Lines(vec![(p, dir(-ee / lin, dd / lin))]);
    }
    // Principal axes of the quadratic part, with |l1| >= |l2|.
    let mut th = 0.5 * bb.atan2(aa - cc);
    let eig = |th: f64| {
        let (s, co) = th.sin_cos();
        (aa * co * co + bb * co * s + cc * s * s, aa * s * s - bb * s * co + cc * co * co)
    };
    let (mut l1, mut l2) = eig(th);
    if l1.abs() < l2.abs() {
        th += core::f64::consts::FRAC_PI_2;
        (l1, l2) = eig(th);
    }
    let (s, co) = th.sin_cos();
    let dp = dd * co + ee * s;
    let ep = -dd * s + ee * co;
    let e1 = dir(co, s);
    let e2 = dir(-s, co);
    let at = |up: f64, wp: f64| -> Point3 { fr.origin + e1 * up + e2 * wp };
    let at_vec = |up: f64, wp: f64| -> Vector3 { e1 * up + e2 * wp };
    if l2.abs() <= 1e-10 * l1.abs() {
        // Parabolic type.
        let xi0 = -dp / (2.0 * l1);
        let on_line = at(xi0, 0.0);
        // Restriction value at (xi0, 0), evaluated on the quadric (no cancellation with ff).
        let gg = q.value(on_line);
        let reference = dp.abs().max(ep.abs()).max(l1.abs());
        if ep.abs() > 1e-9 * reference {
            let kappa = -l1 / ep;
            let vertex = at(xi0, -gg / ep);
            // eta' - eta_v = kappa * xi''^2 -> opens toward sign(kappa) * e2.
            let axis = if kappa > 0.0 { e2 } else { e2 * -1.0 };
            let focal = 1.0 / (4.0 * kappa.abs());
            return Section::Parabola { vertex, perp: e1, axis, focal };
        }
        let r2 = -gg / l1;
        if r2 < 0.0 {
            // Plane and quadric do not meet: tangent when the gap is within tolerance.
            return if gap_at(on_line) <= tol.linear { Section::Lines(vec![(on_line, e2)]) } else { Section::Empty };
        }
        let s = r2.sqrt();
        if s <= tol.linear {
            return Section::Lines(vec![(on_line, e2)]);
        }
        return Section::Lines(vec![(at(xi0 - s, 0.0), e2), (at(xi0 + s, 0.0), e2)]);
    }
    let (uc, wc) = (-dp / (2.0 * l1), -ep / (2.0 * l2));
    let center = at(uc, wc);
    // Restriction value at the centre, evaluated on the quadric itself.
    let fp = q.value(center);
    let gap = gap_at(center);
    if l1 * l2 > 0.0 {
        if -fp / l1 > 0.0 {
            let (a, b) = ((-fp / l1).sqrt(), (-fp / l2).sqrt());
            if a.max(b) <= tol.linear {
                return Section::Point(center);
            }
            return Section::Ellipse { center, x: dir(co, s), y: dir(-s, co), a, b };
        }
        // Imaginary ellipse: the plane misses the quadric; tangent if the gap is within tolerance.
        return if gap <= tol.linear { Section::Point(center) } else { Section::Empty };
    }
    if gap <= tol.linear {
        // Two lines through the centre: l1 u^2 + l2 w^2 = 0.
        let k = (-l1 / l2).sqrt();
        return Section::Lines(vec![(center, at_vec(1.0, k)), (center, at_vec(1.0, -k))]);
    }
    if -fp / l1 > 0.0 {
        Section::Hyperbola { center, main: dir(co, s), other: dir(-s, co), a: (-fp / l1).sqrt(), b: (fp / l2).sqrt() }
    } else {
        Section::Hyperbola { center, main: dir(-s, co), other: dir(co, s), a: (-fp / l2).sqrt(), b: (fp / l1).sqrt() }
    }
}

fn unit(v: Vector3) -> Vector3 {
    let n = v.norm();
    if n > 0.0 { v * (1.0 / n) } else { v }
}

fn in_box(w: &[f64; 6], p: Point3, pad: f64) -> bool {
    p.x >= w[0] - pad && p.x <= w[3] + pad && p.y >= w[1] - pad && p.y <= w[4] + pad && p.z >= w[2] - pad && p.z <= w[5] + pad
}

fn box_radius(w: &[f64; 6], from: Point3) -> f64 {
    let mut m = 0.0f64;
    for i in 0..8 {
        let c = Point3::new(if i & 1 == 0 { w[0] } else { w[3] }, if i & 2 == 0 { w[1] } else { w[4] }, if i & 4 == 0 { w[2] } else { w[5] });
        m = m.max((c - from).norm());
    }
    m
}

/// Maximal sub-intervals of `[lo, hi]` where `pt(t)` lies in the window (padded by `pad`), in
/// ascending order. `breaks` are the parameters where a coordinate crosses a window face (found
/// in closed form by the caller); the curve is inside or outside between consecutive breaks,
/// which a midpoint test decides. Adjacent inside pieces are merged.
fn window_ranges(pt: &dyn Fn(f64) -> Point3, mut breaks: Vec<f64>, lo: f64, hi: f64, w: &[f64; 6], pad: f64) -> Vec<(f64, f64)> {
    breaks.retain(|t| t.is_finite() && *t > lo && *t < hi);
    breaks.push(lo);
    breaks.push(hi);
    breaks.sort_by(|a, b| a.total_cmp(b));
    breaks.dedup();
    let mut out: Vec<(f64, f64)> = Vec::new();
    for pair in breaks.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        if b > a && in_box(w, pt(0.5 * (a + b)), pad) {
            match out.last_mut() {
                Some(l) if l.1 >= a => l.1 = b,
                _ => out.push((a, b)),
            }
        }
    }
    out
}

/// Real roots of `a t^2 + b t + c` (a double root once).
fn quadratic(a: f64, b: f64, c: f64) -> impl Iterator<Item = Root> {
    let mut out = [Root::default(); 2];
    let n = solve_quadratic(a, b, c, &mut out);
    out.into_iter().take(n)
}

/// Face-crossing parameters of the polynomial coordinates `c0 + c1 t + c2 t^2` against the
/// padded window (`k`-th coordinate between `w[k]` and `w[k + 3]`).
fn quad_breaks(coef: [[f64; 3]; 3], w: &[f64; 6], pad: f64) -> Vec<f64> {
    let mut v = Vec::new();
    for k in 0..3 {
        for val in [w[k] - pad, w[k + 3] + pad] {
            v.extend(quadratic(coef[k][2], coef[k][1], coef[k][0] - val).map(|r| r.t));
        }
    }
    v
}

/// Same for `c0 + A cosh t + B sinh t`, with `z = e^t`: `(A+B)/2 z^2 + (c0 - val) z + (A-B)/2`.
fn hyper_breaks(c: [f64; 3], a: [f64; 3], b: [f64; 3], w: &[f64; 6], pad: f64) -> Vec<f64> {
    let mut v = Vec::new();
    for k in 0..3 {
        for val in [w[k] - pad, w[k + 3] + pad] {
            v.extend(quadratic(0.5 * (a[k] + b[k]), c[k] - val, 0.5 * (a[k] - b[k])).filter(|r| r.t > 0.0).map(|r| r.t.ln()));
        }
    }
    v
}

fn nurbs_chain(arcs: Vec<NurbsCurve>) -> Result<NurbsCurve, GeomError> {
    // Merge consecutive quadratic Bezier arcs into one C0 NURBS: knots at the arc boundaries.
    let mut control: Vec<Point3> = Vec::new();
    let mut weights: Vec<f64> = Vec::new();
    let mut knots: Vec<f64> = Vec::new();
    let n = arcs.len();
    for (i, a) in arcs.iter().enumerate() {
        let (r, w) = (a.range(), a.weights.clone().unwrap_or_else(|| vec![1.0; 3]));
        let skip = usize::from(i > 0);
        control.extend(a.control[skip..].iter().copied());
        weights.extend(w[skip..].iter().copied());
        if i == 0 {
            knots.extend([r.lo; 3]);
        }
        if i + 1 < n {
            knots.extend([r.hi; 2]);
        } else {
            knots.extend([r.hi; 3]);
        }
    }
    NurbsCurve::new(2, knots, control, Some(weights))
}

/// A conic section as `SsiResult` curves. `window` clips open branches (required for parabola
/// and hyperbola); `keep` filters closed/open pieces by a point predicate (nappe selection).
pub(crate) fn section_to_result(sec: &Section, tol: &ModelTolerance, window: Option<&[f64; 6]>, keep: &dyn Fn(Point3) -> bool, ray_from: Option<(Point3, Vector3)>) -> Result<SsiResult, GeomError> {
    let curves = |v: Vec<IntersectionCurve>| if v.is_empty() { SsiResult::Empty } else { SsiResult::Curves(v) };
    match sec {
        Section::Empty => Ok(SsiResult::Empty),
        Section::Coincident => Ok(SsiResult::Coincident),
        Section::Point(p) => Ok(if keep(*p) { SsiResult::Point(*p) } else { SsiResult::Empty }),
        Section::Ellipse { center, x, y, a, b } => {
            let p = *center + *x * *a;
            if !keep(p) {
                return Ok(SsiResult::Empty);
            }
            let x = unit(*x);
            let z = x.cross(unit(*y));
            let frame = Frame { origin: *center, x, y: z.cross(x), z };
            let def = if (a - b).abs() <= 1e-9 * a.max(*b) { CurveDef::Circle(Circle::new(frame, 0.5 * (a + b))?) } else { CurveDef::Ellipse(Ellipse::new(frame, *a, *b)?) };
            Ok(curves(vec![IntersectionCurve::analytic(def, Interval::new(0.0, TAU), true)]))
        }
        Section::Lines(lines) => {
            let mut out = Vec::new();
            for (o, d) in lines {
                let d = unit(*d);
                let line = Line::new(*o, d)?;
                let mut range = Interval::REAL;
                if let Some((apex, axis)) = ray_from {
                    // Lines through the apex of a cone keep only their half on the nappe.
                    if (*o - apex).norm() <= tol.linear.max(1e-9 * (*o - apex).norm()) || ((*o - apex).cross(d)).norm() <= tol.linear {
                        let s = d.dot(axis);
                        let l = Line::new(apex, if s >= 0.0 { d } else { d * -1.0 })?;
                        out.push(IntersectionCurve::analytic(CurveDef::Line(l), Interval::new(0.0, f64::INFINITY), false));
                        continue;
                    }
                }
                if let Some(w) = window {
                    // Slab clip.
                    let (mut t0, mut t1) = (f64::NEG_INFINITY, f64::INFINITY);
                    let oo = [o.x, o.y, o.z];
                    let dd = [d.x, d.y, d.z];
                    for k in 0..3 {
                        let (lo, hi) = (w[k] - tol.linear, w[k + 3] + tol.linear);
                        if dd[k].abs() < 1e-300 {
                            if oo[k] < lo || oo[k] > hi {
                                t1 = f64::NEG_INFINITY;
                            }
                        } else {
                            let (u, v) = ((lo - oo[k]) / dd[k], (hi - oo[k]) / dd[k]);
                            t0 = t0.max(u.min(v));
                            t1 = t1.min(u.max(v));
                        }
                    }
                    if t1 < t0 {
                        continue;
                    }
                    range = Interval::new(t0, t1);
                }
                if keep(line.origin + line.dir * (if range.is_bounded() { 0.5 * (range.lo + range.hi) } else { 0.0 })) {
                    out.push(IntersectionCurve::analytic(CurveDef::Line(line), range, false));
                }
            }
            Ok(curves(out))
        }
        Section::Parabola { vertex, perp, axis, focal } => {
            if !keep(*vertex) {
                return Ok(SsiResult::Empty);
            }
            let w = window.ok_or(GeomError::Invalid("an open conic section needs a window"))?;
            let pt = |t: f64| *vertex + *perp * t + *axis * (t * t / (4.0 * focal));
            let reach = box_radius(w, *vertex);
            let tmax = 2.0 * reach + 1.0;
            let (v, p, ax) = ([vertex.x, vertex.y, vertex.z], [perp.x, perp.y, perp.z], [axis.x, axis.y, axis.z]);
            let coef = [0, 1, 2].map(|k| [v[k], p[k], ax[k] / (4.0 * focal)]);
            // One arc per stretch inside the window (a parabola can enter the box several times).
            let mut out = Vec::new();
            for (t0, t1) in window_ranges(&pt, quad_breaks(coef, w, tol.linear), -tmax, tmax, w, tol.linear) {
                let p1 = *vertex + *perp * (0.5 * (t0 + t1)) + *axis * (t0 * t1 / (4.0 * focal));
                let arc = NurbsCurve::new(2, vec![t0, t0, t0, t1, t1, t1], vec![pt(t0), p1, pt(t1)], None)?;
                out.push(IntersectionCurve::analytic(CurveDef::Nurbs(arc), Interval::new(t0, t1), false));
            }
            Ok(curves(out))
        }
        Section::Hyperbola { center, main, other, a, b } => {
            let w = window.ok_or(GeomError::Invalid("an open conic section needs a window"))?;
            let reach = box_radius(w, *center);
            let tau_max = ((reach / a.min(*b)).max(1.0) + 1.0).ln() + 2.0;
            let mut out = Vec::new();
            for sgn in [1.0f64, -1.0] {
                let pt = |t: f64| *center + *main * (sgn * a * t.cosh()) + *other * (b * t.sinh());
                if !keep(pt(0.0)) {
                    continue;
                }
                let (c, am, bm) = ([center.x, center.y, center.z], [main.x * sgn * a, main.y * sgn * a, main.z * sgn * a], [other.x * b, other.y * b, other.z * b]);
                for (t0, t1) in window_ranges(&pt, hyper_breaks(c, am, bm, w, tol.linear), -tau_max, tau_max, w, tol.linear) {
                    let n = (((t1 - t0) / 1.5).ceil() as usize).max(1);
                    let mut arcs = Vec::with_capacity(n);
                    for i in 0..n {
                        let (ta, tb) = (t0 + (t1 - t0) * i as f64 / n as f64, t0 + (t1 - t0) * (i + 1) as f64 / n as f64);
                        let (m, half) = (0.5 * (ta + tb), 0.5 * (tb - ta));
                        let ch = half.cosh();
                        let p1 = *center + *main * (sgn * a * m.cosh() / ch) + *other * (b * m.sinh() / ch);
                        let arc = conic_arc(pt(ta), p1, pt(tb), ch)?;
                        // Re-knot so the chain's knots are the conic parameter at the arc ends.
                        let reknot = NurbsCurve::new(2, vec![ta, ta, ta, tb, tb, tb], arc.control.clone(), arc.weights.clone())?;
                        arcs.push(reknot);
                    }
                    let chain = nurbs_chain(arcs)?;
                    out.push(IntersectionCurve::analytic(CurveDef::Nurbs(chain), Interval::new(t0, t1), false));
                }
            }
            Ok(curves(out))
        }
    }
}
