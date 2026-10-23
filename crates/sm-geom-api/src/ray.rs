//! Ray/surface intersection hook for tracking, plus a small real-root finder.

use crate::error::GeomError;
use crate::frame::unit;
use crate::tol::{Interval, TolerancePolicy};
use smallvec::SmallVec;
use sm_core::{Point3, Vector3};

/// A ray `origin + t * dir` with unit `dir`, so `t` is arc length.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Ray {
    /// Start point.
    pub origin: Point3,
    /// Unit direction.
    pub dir: Vector3,
}

impl Ray {
    /// Build a ray; `dir` is normalized.
    pub fn new(origin: Point3, dir: Vector3) -> Result<Self, GeomError> {
        Ok(Self { origin, dir: unit(dir).ok_or(GeomError::Invalid("zero ray direction"))? })
    }
    /// Point at distance `t`.
    pub fn at(&self, t: f64) -> Point3 {
        self.origin + self.dir * t
    }
}

/// One ray/surface crossing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    /// Ray parameter (distance).
    pub t: f64,
    /// Hit point.
    pub point: Point3,
    /// Unit surface normal at the hit, in the surface's own orientation (not flipped toward
    /// the ray). Dotting with `ray.dir` tells entering vs leaving for oriented faces.
    pub normal: Vector3,
    /// Surface parameters when the surface is parametric.
    pub uv: Option<(f64, f64)>,
    /// True if this is a tangent (double-root) contact within tolerance.
    pub grazing: bool,
}

/// Exact ray intersection. Object-safe so tracking can hold `&dyn RayIntersect` per face.
///
/// Hits are appended to `hits` sorted by increasing `t`, restricted to `t_range`. A tracker
/// excludes the surface it just crossed by raising `t_range.lo`. Roots closer than
/// `tol.linear_min` merge into one `grazing` hit. Shapes without an exact solver (NURBS)
/// return [`GeomError::Unsupported`] and the tracker falls back to facets.
pub trait RayIntersect: Send + Sync {
    /// Append all crossings in `t_range` to `hits`.
    fn intersect_ray(
        &self,
        ray: &Ray,
        t_range: Interval,
        tol: &TolerancePolicy,
        hits: &mut SmallVec<[RayHit; 4]>,
    ) -> Result<(), GeomError>;
}

fn eval_poly(c: &[f64], t: f64) -> f64 {
    c.iter().rev().fold(0.0, |acc, k| acc * t + k)
}

/// Real roots of the polynomial `c[0] + c[1] t + ...` in the finite interval `[lo, hi]`,
/// ascending. Isolates roots between the roots of the derivative (so degree <= 4 is
/// exact up to bisection). Double roots are reported once.
pub fn poly_roots_in(c: &[f64], lo: f64, hi: f64) -> SmallVec<[f64; 4]> {
    let mut out: SmallVec<[f64; 4]> = SmallVec::new();
    let scale = c.iter().fold(0.0f64, |m, k| m.max(k.abs()));
    if scale == 0.0 || lo.is_nan() || hi.is_nan() || lo > hi {
        return out;
    }
    let mut n = c.len();
    while n > 0 && c[n - 1].abs() <= 1e-14 * scale {
        n -= 1;
    }
    let c = &c[..n];
    match n {
        0 | 1 => {}
        2 => out.push(-c[0] / c[1]),
        3 => {
            let (a, b, k) = (c[2], c[1], c[0]);
            let disc = b * b - 4.0 * a * k;
            if disc.abs() <= 1e-14 * (b * b + (4.0 * a * k).abs()) {
                out.push(-b / (2.0 * a));
            } else if disc > 0.0 {
                let q = -0.5 * (b + b.signum() * disc.sqrt());
                let (r1, r2) = if q == 0.0 { (0.0, 0.0) } else { (q / a, k / q) };
                out.push(r1.min(r2));
                out.push(r1.max(r2));
            }
        }
        _ => {
            let d: SmallVec<[f64; 5]> = (1..n).map(|i| c[i] * i as f64).collect();
            let crit = poly_roots_in(&d, lo, hi);
            let mut pts: SmallVec<[f64; 6]> = SmallVec::new();
            pts.push(lo);
            pts.extend(crit.iter().copied());
            pts.push(hi);
            let fs: SmallVec<[f64; 6]> = pts.iter().map(|&t| eval_poly(c, t)).collect();
            let eps = 1e-12 * scale;
            for i in 0..pts.len() {
                if fs[i].abs() <= eps && i > 0 && i + 1 < pts.len() {
                    out.push(pts[i]); // tangent root at a critical point
                } else if i + 1 < pts.len() && fs[i] * fs[i + 1] < 0.0 {
                    let (mut a, mut b) = (pts[i], pts[i + 1]);
                    let fa_neg = fs[i] < 0.0;
                    for _ in 0..100 {
                        let m = 0.5 * (a + b);
                        if (eval_poly(c, m) < 0.0) == fa_neg { a = m } else { b = m }
                    }
                    out.push(0.5 * (a + b));
                } else if fs[i].abs() <= eps && (i == 0 || i + 1 == pts.len()) {
                    out.push(pts[i]);
                }
            }
        }
    }
    let mut r: SmallVec<[f64; 4]> = out.into_iter().filter(|t| *t >= lo && *t <= hi).collect();
    r.sort_by(|a, b| a.total_cmp(b));
    r.dedup_by(|a, b| (*a - *b).abs() <= 1e-12 * (1.0 + b.abs()));
    r
}

/// Cauchy bound `B` such that all real roots of `c` lie in `[-B, B]`.
pub fn root_bound(c: &[f64]) -> f64 {
    let scale = c.iter().fold(0.0f64, |m, k| m.max(k.abs()));
    let mut n = c.len();
    while n > 0 && c[n - 1].abs() <= 1e-14 * scale {
        n -= 1;
    }
    if n < 2 {
        return 0.0;
    }
    1.0 + c[..n - 1].iter().fold(0.0f64, |m, k| m.max((k / c[n - 1]).abs()))
}

/// Roots of `c` clipped to `t_range` (which may be unbounded), with near-coincident roots
/// merged; returns `(t, grazing)` pairs.
pub fn ray_roots(c: &[f64], t_range: Interval, tol: &TolerancePolicy) -> SmallVec<[(f64, bool); 4]> {
    let b = root_bound(c);
    let (lo, hi) = (t_range.lo.max(-b), t_range.hi.min(b));
    let raw = poly_roots_in(c, lo, hi);
    let scale = c.iter().fold(0.0f64, |m, k| m.max(k.abs()));
    let dc: SmallVec<[f64; 5]> = (1..c.len()).map(|i| c[i] * i as f64).collect();
    let mut out: SmallVec<[(f64, bool); 4]> = SmallVec::new();
    for t in raw {
        // A vanishing slope at the root means a tangent (double-root) contact.
        let tangent = eval_poly(&dc, t).abs() <= 1e-9 * scale * (1.0 + t.abs());
        match out.last_mut() {
            Some(last) if t - last.0 <= tol.linear_min => *last = (0.5 * (last.0 + t), true),
            _ => out.push((t, tangent)),
        }
    }
    out
}
