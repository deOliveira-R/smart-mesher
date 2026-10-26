//! Curve-curve intersection (private to the crate; `sm-geom` has curve-surface only).
//!
//! Line/line is closed form. Every other pair is solved on the curves' evaluators: the first
//! curve is sampled, the distance to the second (a closest-point projection restricted to its
//! range) is minimized around each local minimum by golden-section search, and minima within
//! the tolerance are reported as hits; runs of samples all within the tolerance are reported as
//! overlaps. This covers the line/circle, circle/circle and NURBS cases to the accuracy of the
//! evaluators (about 1e-12 for the analytic kinds). Promotion into `sm-geom` is a W3b item.

use sm_core::{Point3, Vector3};
use sm_geom_api::{CurveDef, Interval, ParametricCurve};

use crate::geo;

/// One curve-curve contact.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct CcHit {
    /// Parameter on the first curve.
    pub t1: f64,
    /// Parameter on the second curve.
    pub t2: f64,
    /// The contact point (on the first curve).
    pub p: Point3,
}

/// Result of [`curve_curve`].
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct CcResult {
    /// Isolated contacts.
    pub points: Vec<CcHit>,
    /// Parameter intervals of the first curve along which the curves coincide.
    pub overlaps: Vec<Interval>,
}

/// Distance from `p` to the part of `c` over `range`, with the parameter of the closest point.
pub(crate) fn dist_to_range(c: &dyn ParametricCurve, range: Interval, p: Point3) -> Option<(f64, f64)> {
    let mut best: Option<(f64, f64)> = None;
    let mut consider = |t: f64| {
        let d = geo::dist(c.eval(t), p);
        if best.is_none_or(|b| d < b.0) {
            best = Some((d, t));
        }
    };
    consider(range.lo);
    consider(range.hi);
    if let Ok(pr) = c.project(p, None) {
        let period = c.domain().period;
        let mut ts = vec![pr.t];
        if let Some(per) = period {
            ts.push(pr.t + per);
            ts.push(pr.t - per);
        }
        for t in ts {
            if t >= range.lo - 1e-12 * range.len().abs().max(1.0) && t <= range.hi + 1e-12 * range.len().abs().max(1.0) {
                consider(t.clamp(range.lo, range.hi));
            }
        }
    }
    best
}

fn line_parts(d: &CurveDef) -> Option<(Point3, Vector3)> {
    match d {
        CurveDef::Line(l) => Some((l.origin, l.dir)),
        _ => None,
    }
}

/// Intersect two curve segments within the distance `tol`.
pub(crate) fn curve_curve(d1: &CurveDef, r1: Interval, d2: &CurveDef, r2: Interval, tol: f64) -> CcResult {
    if let (Some((p1, a)), Some((p2, b))) = (line_parts(d1), line_parts(d2)) {
        return line_line(p1, a, r1, p2, b, r2, tol);
    }
    numeric(d1.as_curve(), r1, d2.as_curve(), r2, tol)
}

fn line_line(p1: Point3, a: Vector3, r1: Interval, p2: Point3, b: Vector3, r2: Interval, tol: f64) -> CcResult {
    let mut out = CcResult::default();
    let w = p1 - p2;
    let (aa, bb, ab) = (a.dot(a), b.dot(b), a.dot(b));
    let den = aa * bb - ab * ab;
    let eps1 = 1e-12 * r1.len().abs().max(1.0);
    let eps2 = 1e-12 * r2.len().abs().max(1.0);
    if den.abs() <= 1e-14 * aa * bb {
        // Parallel: collinear if the distance between the lines is within tol.
        let t_of = |p: Point3| (p - p1).dot(a) / aa;
        let off = {
            let q = p2 - p1;
            let along = q.dot(a) / aa;
            let perp = q - a * along;
            perp.norm()
        };
        if off > tol {
            return out;
        }
        // Overlap of r1 with the image of r2 on the first line's parameter.
        let (u0, u1) = (t_of(p2 + b * r2.lo), t_of(p2 + b * r2.hi));
        let (lo, hi) = (u0.min(u1).max(r1.lo), u0.max(u1).min(r1.hi));
        if hi - lo > tol / aa.sqrt() {
            out.overlaps.push(Interval::new(lo, hi));
        } else if hi - lo >= -tol / aa.sqrt() {
            let t = 0.5 * (lo + hi);
            let p = p1 + a * t;
            let s = (p - p2).dot(b) / bb;
            out.points.push(CcHit { t1: t, t2: s, p });
        }
        return out;
    }
    let (d, e) = (a.dot(w), b.dot(w));
    let t = (ab * e - bb * d) / den;
    let s = (aa * e - ab * d) / den;
    if t < r1.lo - eps1 || t > r1.hi + eps1 || s < r2.lo - eps2 || s > r2.hi + eps2 {
        return out;
    }
    let (q1, q2) = (p1 + a * t, p2 + b * s);
    if geo::dist(q1, q2) <= tol {
        out.points.push(CcHit { t1: t.clamp(r1.lo, r1.hi), t2: s.clamp(r2.lo, r2.hi), p: q1 });
    }
    out
}

fn numeric(c1: &dyn ParametricCurve, r1: Interval, c2: &dyn ParametricCurve, r2: Interval, tol: f64) -> CcResult {
    let mut out = CcResult::default();
    if !(r1.is_bounded() && r1.hi > r1.lo) {
        return out;
    }
    const N: usize = 160;
    let ts: Vec<f64> = (0..=N).map(|i| r1.lo + (r1.hi - r1.lo) * (i as f64 / N as f64)).collect();
    let dist = |t: f64| dist_to_range(c2, r2, c1.eval(t)).map(|d| d.0).unwrap_or(f64::INFINITY);
    let ds: Vec<f64> = ts.iter().map(|&t| dist(t)).collect();
    // Overlap runs: consecutive samples all within tol (and at least three of them).
    let mut i = 0;
    let mut in_overlap = vec![false; ts.len()];
    while i < ts.len() {
        if ds[i] <= tol {
            let mut j = i;
            while j + 1 < ts.len() && ds[j + 1] <= tol {
                j += 1;
            }
            if j - i >= 2 {
                for flag in in_overlap.iter_mut().take(j + 1).skip(i) {
                    *flag = true;
                }
                // Refine the run ends by bisection towards the neighbouring failing samples.
                let refine = |inside: f64, outside: f64| -> f64 {
                    let (mut a, mut b) = (inside, outside);
                    for _ in 0..60 {
                        let m = 0.5 * (a + b);
                        if dist(m) <= tol { a = m } else { b = m }
                    }
                    a
                };
                let lo = if i > 0 { refine(ts[i], ts[i - 1]) } else { ts[i] };
                let hi = if j + 1 < ts.len() { refine(ts[j], ts[j + 1]) } else { ts[j] };
                out.overlaps.push(Interval::new(lo, hi));
            }
            i = j + 1;
        } else {
            i += 1;
        }
    }
    // Isolated minima.
    for i in 0..ts.len() {
        if in_overlap[i] {
            continue;
        }
        let left = if i > 0 { ds[i - 1] } else { f64::INFINITY };
        let right = if i + 1 < ts.len() { ds[i + 1] } else { f64::INFINITY };
        if !(ds[i] <= left && ds[i] <= right) {
            continue;
        }
        // Golden-section search on [ts[i-1], ts[i+1]].
        let (mut a, mut b) = (ts[i.saturating_sub(1)], ts[(i + 1).min(ts.len() - 1)]);
        let g = 0.618_033_988_749_894_9;
        let (mut x1, mut x2) = (b - g * (b - a), a + g * (b - a));
        let (mut f1, mut f2) = (dist(x1), dist(x2));
        for _ in 0..80 {
            if f1 < f2 {
                b = x2;
                x2 = x1;
                f2 = f1;
                x1 = b - g * (b - a);
                f1 = dist(x1);
            } else {
                a = x1;
                x1 = x2;
                f1 = f2;
                x2 = a + g * (b - a);
                f2 = dist(x2);
            }
        }
        let t = 0.5 * (a + b);
        let mut best_t = t;
        let mut best_d = dist(t);
        for cand in [ts[i], r1.lo, r1.hi] {
            if (cand - t).abs() < (b - a).abs().max(1e-9) * 4.0 + (r1.hi - r1.lo) / N as f64 && dist(cand) < best_d {
                best_t = cand;
                best_d = dist(cand);
            }
        }
        if best_d <= tol {
            let p = c1.eval(best_t);
            if let Some((_, s)) = dist_to_range(c2, r2, p)
                && out.points.iter().all(|h| (h.t1 - best_t).abs() > 1e-9 * r1.len().abs().max(1.0)) {
                    out.points.push(CcHit { t1: best_t, t2: s, p });
                }
        }
    }
    out.points.sort_by(|a, b| a.t1.total_cmp(&b.t1));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_geom_api::{Circle, Frame, Line};

    fn line(o: [f64; 3], d: [f64; 3]) -> CurveDef {
        CurveDef::Line(Line::new(Point3::new(o[0], o[1], o[2]), Vector3::new(d[0], d[1], d[2])).unwrap())
    }

    #[test]
    fn crossing_lines() {
        let r = curve_curve(&line([0.0; 3], [1.0, 0.0, 0.0]), Interval::new(0.0, 2.0), &line([1.0, -1.0, 0.0], [0.0, 1.0, 0.0]), Interval::new(0.0, 2.0), 1e-9);
        assert_eq!(r.points.len(), 1);
        assert!((r.points[0].t1 - 1.0).abs() < 1e-12 && (r.points[0].t2 - 1.0).abs() < 1e-12);
    }

    #[test]
    fn skew_and_out_of_range_lines_miss() {
        let skew = curve_curve(&line([0.0; 3], [1.0, 0.0, 0.0]), Interval::new(0.0, 2.0), &line([1.0, -1.0, 0.5], [0.0, 1.0, 0.0]), Interval::new(0.0, 2.0), 1e-9);
        assert!(skew.points.is_empty());
        let far = curve_curve(&line([0.0; 3], [1.0, 0.0, 0.0]), Interval::new(0.0, 0.5), &line([1.0, -1.0, 0.0], [0.0, 1.0, 0.0]), Interval::new(0.0, 2.0), 1e-9);
        assert!(far.points.is_empty());
    }

    #[test]
    fn collinear_lines_overlap() {
        let r = curve_curve(&line([0.0; 3], [1.0, 0.0, 0.0]), Interval::new(0.0, 2.0), &line([1.0, 0.0, 0.0], [1.0, 0.0, 0.0]), Interval::new(0.0, 3.0), 1e-9);
        assert_eq!(r.overlaps.len(), 1);
        assert!((r.overlaps[0].lo - 1.0).abs() < 1e-12 && (r.overlaps[0].hi - 2.0).abs() < 1e-12);
    }

    #[test]
    fn line_through_circle() {
        let c = CurveDef::Circle(Circle::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0).unwrap());
        let r = curve_curve(&line([-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]), Interval::new(0.0, 4.0), &c, Interval::new(0.0, std::f64::consts::TAU), 1e-9);
        assert_eq!(r.points.len(), 2, "{r:?}");
        let ts: Vec<f64> = r.points.iter().map(|h| h.t1).collect();
        assert!((ts[0] - 1.0).abs() < 1e-9 && (ts[1] - 3.0).abs() < 1e-9, "{ts:?}");
    }

    #[test]
    fn tangent_line_touches_the_circle_once() {
        let c = CurveDef::Circle(Circle::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0).unwrap());
        let r = curve_curve(&line([-2.0, 1.0, 0.0], [1.0, 0.0, 0.0]), Interval::new(0.0, 4.0), &c, Interval::new(0.0, std::f64::consts::TAU), 1e-6);
        assert_eq!(r.points.len(), 1, "{r:?}");
        assert!((r.points[0].t1 - 2.0).abs() < 1e-5);
    }

    #[test]
    fn circles_meet_in_two_points() {
        let a = CurveDef::Circle(Circle::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0).unwrap());
        let b = CurveDef::Circle(Circle::new(Frame::at(Point3::new(1.0, 0.0, 0.0)), 1.0).unwrap());
        let r = curve_curve(&a, Interval::new(0.0, std::f64::consts::TAU), &b, Interval::new(0.0, std::f64::consts::TAU), 1e-9);
        assert_eq!(r.points.len(), 2, "{r:?}");
        for h in &r.points {
            assert!((h.p.x - 0.5).abs() < 1e-9 && (h.p.y.abs() - 0.75f64.sqrt()).abs() < 1e-9);
        }
    }

    #[test]
    fn equal_circles_overlap() {
        let a = CurveDef::Circle(Circle::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0).unwrap());
        let r = curve_curve(&a, Interval::new(0.0, 1.0), &a, Interval::new(0.5, 2.0), 1e-9);
        assert_eq!(r.overlaps.len(), 1, "{r:?}");
        assert!((r.overlaps[0].lo - 0.5).abs() < 1e-6 && (r.overlaps[0].hi - 1.0).abs() < 1e-6);
    }
}
