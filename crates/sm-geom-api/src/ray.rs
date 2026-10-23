//! Ray/surface intersection hook used by tracking structures (docs/design/ir.md §5.2).
//!
//! The hook intersects the *untrimmed* surface. Trimming against face loops is the caller's job
//! (it knows the topology). Hits are appended to `out` in ascending `t`, so results are
//! deterministic and independent of thread count.

use crate::ModelTolerance;
use sm_core::{Point3, Vector3};

/// A ray `origin + t * dir`. `dir` need not be unit; `t` is in units of `dir`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ray {
    /// Start point.
    pub origin: Point3,
    /// Direction.
    pub dir: Vector3,
}

impl Ray {
    /// Point at parameter `t`.
    pub fn at(&self, t: f64) -> Point3 {
        self.origin + self.dir * t
    }
}

/// Direction of travel relative to the surface normal at a hit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NormalCrossing {
    /// `dir . n > 0`: the ray leaves the side the normal points away from and enters the side it points into.
    AlongNormal,
    /// `dir . n < 0`.
    AgainstNormal,
    /// Grazing: `|dir . n|` below angular tolerance.
    Tangent,
}

impl NormalCrossing {
    /// Classify `dir` against unit normal `n`.
    pub fn classify(dir: Vector3, n: Vector3, tol: &ModelTolerance) -> Self {
        let dn = dir.norm();
        let c = if dn > 0.0 { dir.dot(n) / dn } else { 0.0 };
        if c.abs() <= tol.angular {
            Self::Tangent
        } else if c > 0.0 {
            Self::AlongNormal
        } else {
            Self::AgainstNormal
        }
    }
}

/// One ray/surface intersection.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    /// Ray parameter.
    pub t: f64,
    /// Hit point.
    pub point: Point3,
    /// Unit surface normal at the hit (surface orientation, not flipped toward the ray).
    pub normal: Vector3,
    /// Surface parameters when the surface is parametric.
    pub uv: Option<[f64; 2]>,
    /// Crossing direction relative to `normal`.
    pub crossing: NormalCrossing,
}

/// Ray intersection with an untrimmed surface.
pub trait RayIntersect: Send + Sync {
    /// Append all hits with `t_min <= t <= t_max` to `out`, ascending in `t`.
    /// A double root is reported once, with [`NormalCrossing::Tangent`].
    fn intersect_ray(&self, ray: &Ray, t_min: f64, t_max: f64, tol: &ModelTolerance, out: &mut Vec<RayHit>);
}

/// Real roots of `a t^2 + b t + c` in ascending order, as `(root, is_double)`.
/// Numerically stable (no cancellation); a near-zero discriminant counts as a double root.
pub fn solve_quadratic(a: f64, b: f64, c: f64) -> smallvec::SmallVec<[(f64, bool); 2]> {
    let mut r = smallvec::SmallVec::new();
    let scale = a.abs().max(b.abs()).max(c.abs());
    if scale == 0.0 {
        return r;
    }
    if a.abs() <= 1e-14 * scale {
        if b != 0.0 {
            r.push((-c / b, false));
        }
        return r;
    }
    let disc = b * b - 4.0 * a * c;
    let eps = 1e-14 * (b * b).max((4.0 * a * c).abs());
    if disc.abs() <= eps {
        r.push((-b / (2.0 * a), true));
    } else if disc > 0.0 {
        let q = -0.5 * (b + b.signum() * disc.sqrt());
        let (x0, x1) = (q / a, if q != 0.0 { c / q } else { q / a });
        r.push((x0.min(x1), false));
        r.push((x0.max(x1), false));
    }
    r
}

/// Real roots of a polynomial (coefficients ascending) in `[lo, hi]`, ascending.
/// Uses recursive bracketing by derivative roots, so the result is deterministic. A tangential
/// (double) root may be missed when the function touches zero without a sign change; callers that
/// need grazing hits use the closed-form quadratic paths.
pub fn poly_roots(coef: &[f64], lo: f64, hi: f64) -> Vec<f64> {
    let mut c: Vec<f64> = coef.to_vec();
    while c.len() > 1 && c.last() == Some(&0.0) {
        c.pop();
    }
    if c.len() <= 1 {
        return Vec::new();
    }
    if c.len() == 2 {
        let r = -c[0] / c[1];
        return if r >= lo && r <= hi { vec![r] } else { Vec::new() };
    }
    let d: Vec<f64> = c.iter().enumerate().skip(1).map(|(i, &v)| v * i as f64).collect();
    let mut pts = vec![lo];
    pts.extend(poly_roots(&d, lo, hi));
    pts.push(hi);
    let eval = |x: f64| c.iter().rev().fold(0.0, |acc, &k| acc * x + k);
    let mut roots = Vec::new();
    for w in pts.windows(2) {
        let (mut a, mut b) = (w[0], w[1]);
        let (fa, fb) = (eval(a), eval(b));
        if fa == 0.0 {
            roots.push(a);
            continue;
        }
        if fa * fb > 0.0 {
            continue;
        }
        if fb == 0.0 && w[1] == hi {
            roots.push(b);
            continue;
        }
        let mut fa = fa;
        for _ in 0..200 {
            let m = 0.5 * (a + b);
            let fm = eval(m);
            if fm == 0.0 || (b - a) <= 1e-16 * m.abs().max(1.0) {
                a = m;
                b = m;
                break;
            }
            if fa * fm < 0.0 {
                b = m;
            } else {
                a = m;
                fa = fm;
            }
        }
        roots.push(0.5 * (a + b));
    }
    roots.dedup_by(|x, y| (*x - *y).abs() < 1e-15);
    roots
}
