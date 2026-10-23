//! CSG shape providers: signed implicit surfaces (quadrics, torus), half-spaces and CSG cells.
//!
//! The convention everywhere: `value(p) > 0` is the *positive* side, the side the gradient (and
//! the analytic surface normal) points to; `value(p) < 0` is the negative side (for closed
//! shapes the inside).

use crate::{Crossing, Frame, GeomError, GeomStore, Interval, Ray, RayHit, util};
use sm_core::{EntityId, Point3, Vector3};
use smallvec::SmallVec;

/// A signed implicit surface `f(p) = 0`. Object-safe; the CSG front end and tracker use
/// `&dyn ImplicitSurface`.
pub trait ImplicitSurface: Send + Sync + core::fmt::Debug {
    /// The signed function value (negative inside for closed shapes).
    fn value(&self, p: Point3) -> f64;
    /// The gradient of `value` (not normalized); it is the outward normal direction.
    fn gradient(&self, p: Point3) -> Vector3;
    /// Append all crossings with `range.lo <= t <= range.hi` to `out`, ascending in `t`.
    /// A tangential (double-root) contact is reported once, with [`Crossing::Tangent`].
    /// `RayHit::normal` is the unit gradient (zero at a singular point of the surface).
    fn ray_hits(&self, ray: &Ray, range: Interval, out: &mut Vec<RayHit>) -> Result<(), GeomError>;
}

/// Which side of a signed surface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Sign {
    /// `f <= 0` (inside for closed shapes).
    Negative,
    /// `f >= 0`.
    Positive,
}

impl Sign {
    /// `-1.0` for [`Sign::Negative`], `+1.0` for [`Sign::Positive`].
    pub const fn factor(self) -> f64 {
        match self {
            Sign::Negative => -1.0,
            Sign::Positive => 1.0,
        }
    }
}

/// A half-space `sign * f >= 0` of a surface held in a [`GeomStore`] (kind `Surface`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct HalfSpace {
    /// Handle of the bounding surface (an `EntityKind::Surface` id).
    pub surface: EntityId,
    /// Which side is inside the half-space.
    pub sign: Sign,
}

impl HalfSpace {
    /// True if `p` is in the half-space, allowing `slack` in function-value units.
    pub fn contains(&self, f: &dyn ImplicitSurface, p: Point3, slack: f64) -> bool {
        let v = f.value(p);
        match self.sign {
            Sign::Negative => v <= slack,
            Sign::Positive => v >= -slack,
        }
    }

    /// First-order signed distance of `p` into the half-space: positive inside, negative
    /// outside, `|f| / |grad f|` in length units (a zero gradient falls back to `f`).
    pub fn depth(&self, f: &dyn ImplicitSurface, p: Point3) -> f64 {
        let v = f.value(p);
        let g = f.gradient(p).norm();
        let d = if g > 0.0 { v / g } else { v };
        self.sign.factor() * d
    }
}

/// Where a point lies relative to a [`CsgCell`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Containment {
    /// Strictly inside every half-space.
    Inside,
    /// Within tolerance of the boundary of the cell.
    On,
    /// Outside at least one half-space by more than the tolerance.
    Outside,
}

/// A CSG cell: the intersection of signed half-spaces. Surfaces are resolved through a
/// [`GeomStore`] (parametric surfaces via their implicit form, or `SurfaceDef::Implicit`), so a
/// general quadric or a torus quartic can bound a cell like a plane.
#[derive(Clone, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CsgCell {
    /// The half-spaces whose intersection is the cell.
    pub halfspaces: SmallVec<[HalfSpace; 4]>,
}

impl CsgCell {
    /// A cell from half-spaces.
    pub fn new(halfspaces: impl IntoIterator<Item = HalfSpace>) -> Self {
        Self { halfspaces: halfspaces.into_iter().collect() }
    }

    fn implicit_of<'a>(s: &'a dyn GeomStore, h: &HalfSpace) -> Result<&'a dyn ImplicitSurface, GeomError> {
        s.implicit(h.surface).ok_or(GeomError::Unsupported("half-space surface has no implicit form"))
    }

    /// Classify `p` against the cell with length tolerance `tol` (first-order distances).
    pub fn contains(&self, s: &dyn GeomStore, p: Point3, tol: f64) -> Result<Containment, GeomError> {
        let mut on = false;
        for h in &self.halfspaces {
            let d = h.depth(Self::implicit_of(s, h)?, p);
            if d < -tol {
                return Ok(Containment::Outside);
            }
            on |= d <= tol;
        }
        Ok(if on { Containment::On } else { Containment::Inside })
    }

    /// Append the crossings of the cell boundary with `range.lo <= t <= range.hi` to `out`,
    /// ascending in `t`. `normal` is the outward normal of the cell and `crossing` is relative to
    /// it (`AlongNormal` = the ray leaves the cell). A ray through an edge or corner of the cell
    /// reports one hit.
    pub fn ray_hits(&self, s: &dyn GeomStore, ray: &Ray, range: Interval, out: &mut Vec<RayHit>) -> Result<(), GeomError> {
        const TOL: f64 = 1e-9;
        let first = out.len();
        let dir_len = ray.dir.norm();
        let mut scratch: Vec<RayHit> = Vec::new();
        for (i, h) in self.halfspaces.iter().enumerate() {
            scratch.clear();
            Self::implicit_of(s, h)?.ray_hits(ray, range, &mut scratch)?;
            'hit: for hit in &scratch {
                for (j, other) in self.halfspaces.iter().enumerate() {
                    if j != i && other.depth(Self::implicit_of(s, other)?, hit.point) < -TOL {
                        continue 'hit;
                    }
                }
                // Outward from the cell: along the gradient for a negative-inside half-space.
                let normal = if h.sign == Sign::Negative { hit.normal } else { -hit.normal };
                let along = if dir_len > 0.0 { ray.dir.dot(normal) / dir_len } else { 0.0 };
                let crossing = if hit.crossing == Crossing::Tangent || along.abs() <= 1e-12 {
                    Crossing::Tangent
                } else if along > 0.0 {
                    Crossing::AlongNormal
                } else {
                    Crossing::AgainstNormal
                };
                out.push(RayHit { normal, crossing, uv: None, ..*hit });
            }
        }
        out[first..].sort_by(|a, b| a.t.total_cmp(&b.t));
        // Merge hits at the same parameter (edges and corners of the cell).
        let mut keep = first;
        for i in first..out.len() {
            if keep > first && (out[i].t - out[keep - 1].t).abs() <= TOL * (1.0 + out[i].t.abs()) {
                continue;
            }
            out.swap(keep, i);
            keep += 1;
        }
        out.truncate(keep);
        Ok(())
    }
}

/// General quadric `A x^2 + B y^2 + C z^2 + D xy + E yz + F zx + G x + H y + J z + K` (MCNP "GQ"
/// ordering). Coefficients may come from an improper (mirrored) transform; sign conventions are
/// whatever the coefficients say (negative inside for a closed quadric).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Quadric {
    /// Coefficients `[A, B, C, D, E, F, G, H, J, K]`.
    pub coeffs: [f64; 10],
}

impl Quadric {
    /// From the local form `sum d_i l_i^2 + sum lin_i l_i + k` with `l` the coordinates in
    /// `frame`. Covers planes, spheres, cylinders, cones, ellipsoids, paraboloids.
    pub fn from_local(frame: &Frame, d: [f64; 3], lin: [f64; 3], k: f64) -> Self {
        let e = [frame.x, frame.y, frame.z];
        let o = util::pos(frame.origin);
        let (mut a, mut b, mut c, mut dd, mut ee, mut ff) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        for i in 0..3 {
            a += d[i] * e[i].x * e[i].x;
            b += d[i] * e[i].y * e[i].y;
            c += d[i] * e[i].z * e[i].z;
            dd += 2.0 * d[i] * e[i].x * e[i].y;
            ee += 2.0 * d[i] * e[i].y * e[i].z;
            ff += 2.0 * d[i] * e[i].z * e[i].x;
        }
        let l = util::comb3(lin[0], e[0], lin[1], e[1], lin[2], e[2]);
        // A*o with the symmetric matrix [[a, dd/2, ff/2], [dd/2, b, ee/2], [ff/2, ee/2, c]].
        let ao = Vector3::new(a * o.x + 0.5 * dd * o.y + 0.5 * ff * o.z, 0.5 * dd * o.x + b * o.y + 0.5 * ee * o.z, 0.5 * ff * o.x + 0.5 * ee * o.y + c * o.z);
        let g = l.x - 2.0 * ao.x;
        let h = l.y - 2.0 * ao.y;
        let j = l.z - 2.0 * ao.z;
        let kk = k + o.dot(ao) - l.dot(o);
        Self { coeffs: [a, b, c, dd, ee, ff, g, h, j, kk] }
    }

    /// The plane through `frame.origin` with normal `frame.z`; positive side along `z`.
    pub fn plane(frame: &Frame) -> Self {
        Self::from_local(frame, [0.0; 3], [0.0, 0.0, 1.0], 0.0)
    }

    /// Sphere of radius `r` about the frame origin (negative inside).
    pub fn sphere(frame: &Frame, r: f64) -> Self {
        Self::from_local(frame, [1.0; 3], [0.0; 3], -r * r)
    }

    /// Infinite cylinder of radius `r` about the frame's `z` axis (negative inside).
    pub fn cylinder(frame: &Frame, r: f64) -> Self {
        Self::from_local(frame, [1.0, 1.0, 0.0], [0.0; 3], -r * r)
    }

    /// Double cone with apex at the frame origin, axis `z`, half-angle `half_angle` (negative
    /// inside).
    pub fn cone(frame: &Frame, half_angle: f64) -> Self {
        let t2 = half_angle.tan().powi(2);
        Self::from_local(frame, [1.0, 1.0, -t2], [0.0; 3], 0.0)
    }

    /// The image of this quadric under an affine map: `f'(p) = f(T^-1 p)`. Works for improper
    /// transforms. `None` if `t` is singular.
    pub fn transformed(&self, t: &sm_core::Transform) -> Option<Self> {
        let inv = t.inverse()?;
        let [a, b, c, d, e, f, g, h, j, k] = self.coeffs;
        // f(x) = x^T Q x + l.x + k with symmetric Q; substitute x = M y + s.
        let q = [[a, d / 2.0, f / 2.0], [d / 2.0, b, e / 2.0], [f / 2.0, e / 2.0, c]];
        let l = [g, h, j];
        let m = inv.m;
        let s = [inv.t.x, inv.t.y, inv.t.z];
        let mut qn = [[0.0; 3]; 3];
        for (i, row) in qn.iter_mut().enumerate() {
            for (jx, cell) in row.iter_mut().enumerate() {
                let mut sum = 0.0;
                for r in 0..3 {
                    for cc in 0..3 {
                        sum += m[r][i] * q[r][cc] * m[cc][jx];
                    }
                }
                *cell = sum;
            }
        }
        let mut qs = [0.0; 3];
        for (r, v) in qs.iter_mut().enumerate() {
            *v = (0..3).map(|cc| q[r][cc] * s[cc]).sum();
        }
        let mut ln = [0.0; 3];
        for (i, v) in ln.iter_mut().enumerate() {
            *v = (0..3).map(|r| 2.0 * m[r][i] * qs[r] + m[r][i] * l[r]).sum();
        }
        let kn = k + (0..3).map(|r| s[r] * qs[r] + l[r] * s[r]).sum::<f64>();
        Some(Self { coeffs: [qn[0][0], qn[1][1], qn[2][2], 2.0 * qn[0][1], 2.0 * qn[1][2], 2.0 * qn[0][2], ln[0], ln[1], ln[2], kn] })
    }
}

impl ImplicitSurface for Quadric {
    fn value(&self, p: Point3) -> f64 {
        let [a, b, c, d, e, f, g, h, j, k] = self.coeffs;
        a * p.x * p.x + b * p.y * p.y + c * p.z * p.z + d * p.x * p.y + e * p.y * p.z + f * p.z * p.x + g * p.x + h * p.y + j * p.z + k
    }

    fn gradient(&self, p: Point3) -> Vector3 {
        let [a, b, c, d, e, f, g, h, j, _] = self.coeffs;
        Vector3::new(2.0 * a * p.x + d * p.y + f * p.z + g, 2.0 * b * p.y + d * p.x + e * p.z + h, 2.0 * c * p.z + e * p.y + f * p.x + j)
    }

    fn ray_hits(&self, ray: &Ray, range: Interval, out: &mut Vec<RayHit>) -> Result<(), GeomError> {
        let [a, b, c, d, e, f, g, h, j, _] = self.coeffs;
        let (o, v) = (ray.origin, ray.dir);
        let qa = a * v.x * v.x + b * v.y * v.y + c * v.z * v.z + d * v.x * v.y + e * v.y * v.z + f * v.z * v.x;
        let qb = 2.0 * (a * o.x * v.x + b * o.y * v.y + c * o.z * v.z)
            + d * (o.x * v.y + o.y * v.x)
            + e * (o.y * v.z + o.z * v.y)
            + f * (o.z * v.x + o.x * v.z)
            + g * v.x
            + h * v.y
            + j * v.z;
        let qc = self.value(o);
        let mut roots = [Root::default(); 2];
        let n = solve_quadratic(qa, qb, qc, &mut roots);
        push_hits(self, ray, range, &roots[..n], out);
        Ok(())
    }
}

/// Torus about the frame's `z` axis as the quartic `(|l|^2 + R^2 - r^2)^2 - 4 R^2 (lx^2 + ly^2)`
/// in local coordinates `l` (negative inside the tube). Requires `major > minor > 0` (ring torus).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ImplicitTorus {
    /// Placement; `z` is the symmetry axis.
    pub frame: Frame,
    /// Major radius `R`.
    pub major: f64,
    /// Minor (tube) radius `r`.
    pub minor: f64,
}

impl ImplicitSurface for ImplicitTorus {
    fn value(&self, p: Point3) -> f64 {
        let l = self.frame.to_local(p);
        let (rr, k) = (self.major * self.major, self.major * self.major - self.minor * self.minor);
        let s = l.dot(l) + k;
        s * s - 4.0 * rr * (l.x * l.x + l.y * l.y)
    }

    fn gradient(&self, p: Point3) -> Vector3 {
        let l = self.frame.to_local(p);
        let (rr, k) = (self.major * self.major, self.major * self.major - self.minor * self.minor);
        let s = l.dot(l) + k;
        let g = Vector3::new(4.0 * s * l.x - 8.0 * rr * l.x, 4.0 * s * l.y - 8.0 * rr * l.y, 4.0 * s * l.z);
        self.frame.vec_from_local(g)
    }

    fn ray_hits(&self, ray: &Ray, range: Interval, out: &mut Vec<RayHit>) -> Result<(), GeomError> {
        let o = self.frame.to_local(ray.origin);
        let d = self.frame.to_local_vec(ray.dir);
        let rr = self.major * self.major;
        let k = rr - self.minor * self.minor;
        let (a, b, c) = (d.dot(d), o.dot(d), o.dot(o));
        let (e, g, h) = (d.x * d.x + d.y * d.y, o.x * d.x + o.y * d.y, o.x * o.x + o.y * o.y);
        let ck = c + k;
        let coef = [ck * ck - 4.0 * rr * h, 4.0 * b * ck - 8.0 * rr * g, 4.0 * b * b + 2.0 * a * ck - 4.0 * rr * e, 4.0 * a * b, a * a];
        let roots = real_roots(&coef, range.lo, range.hi);
        push_hits(self, ray, range, &roots, out);
        Ok(())
    }
}

/// Either implicit form of an analytic surface: a closed, serializable value. Returned by
/// [`crate::ParametricSurface::implicit`] and stored by [`crate::SurfaceDef::Implicit`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ImplicitDef {
    /// A (degree <= 2) quadric, including a general quadric.
    Quadric(Quadric),
    /// A ring torus (degree 4).
    Torus(ImplicitTorus),
}

impl ImplicitSurface for ImplicitDef {
    fn value(&self, p: Point3) -> f64 {
        match self {
            Self::Quadric(q) => q.value(p),
            Self::Torus(t) => t.value(p),
        }
    }
    fn gradient(&self, p: Point3) -> Vector3 {
        match self {
            Self::Quadric(q) => q.gradient(p),
            Self::Torus(t) => t.gradient(p),
        }
    }
    fn ray_hits(&self, ray: &Ray, range: Interval, out: &mut Vec<RayHit>) -> Result<(), GeomError> {
        match self {
            Self::Quadric(q) => q.ray_hits(ray, range, out),
            Self::Torus(t) => t.ray_hits(ray, range, out),
        }
    }
}

/// A real root and whether it is a double (tangential) root.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct Root {
    pub t: f64,
    pub double: bool,
}

fn push_hits<S: ImplicitSurface + ?Sized>(s: &S, ray: &Ray, range: Interval, roots: &[Root], out: &mut Vec<RayHit>) {
    let mut sorted = [Root::default(); 4];
    let n = roots.len().min(4);
    sorted[..n].copy_from_slice(&roots[..n]);
    sorted[..n].sort_by(|a, b| a.t.total_cmp(&b.t));
    let dir_len = ray.dir.norm();
    for r in &sorted[..n] {
        if r.t >= range.lo && r.t <= range.hi {
            let point = ray.at(r.t);
            let normal = util::unit(s.gradient(point)).unwrap_or_default();
            let along = if dir_len > 0.0 { ray.dir.dot(normal) / dir_len } else { 0.0 };
            let crossing = if r.double || along.abs() <= 1e-12 {
                Crossing::Tangent
            } else if along > 0.0 {
                Crossing::AlongNormal
            } else {
                Crossing::AgainstNormal
            };
            out.push(RayHit { t: r.t, point, normal, uv: None, crossing });
        }
    }
}

/// Real roots of `a t^2 + b t + c` (stable form). Returns the count written to `out`
/// (a double root counts once and is flagged). Degenerates to the linear case when `a` is
/// negligible.
pub(crate) fn solve_quadratic(a: f64, b: f64, c: f64, out: &mut [Root; 2]) -> usize {
    let scale = a.abs().max(b.abs()).max(c.abs());
    if scale == 0.0 {
        return 0;
    }
    if a.abs() <= 1e-14 * scale {
        if b.abs() <= 1e-14 * scale {
            return 0;
        }
        out[0] = Root { t: -c / b, double: false };
        return 1;
    }
    let disc = b * b - 4.0 * a * c;
    let dscale = b * b + (4.0 * a * c).abs();
    if disc < -1e-13 * dscale {
        return 0;
    }
    if disc <= 1e-13 * dscale {
        out[0] = Root { t: -b / (2.0 * a), double: true };
        return 1;
    }
    let sq = disc.sqrt();
    let q = -0.5 * (b + b.signum() * sq);
    let (r1, r2) = (q / a, if q != 0.0 { c / q } else { q / a });
    out[0] = Root { t: r1.min(r2), double: false };
    out[1] = Root { t: r1.max(r2), double: false };
    2
}

/// Real roots of the polynomial with ascending coefficients `c` inside `[lo, hi]` (either may
/// be infinite). Isolates monotone pieces with the roots of the derivative, then bisects;
/// double roots are detected from near-zero critical values and flagged. Sorted, deduplicated
/// (a simple root that coincides with a flagged critical root becomes that double root).
pub(crate) fn real_roots(c: &[f64], lo: f64, hi: f64) -> Vec<Root> {
    let maxabs = c.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    let mut n = c.len();
    while n > 1 && c[n - 1].abs() <= 1e-14 * maxabs {
        n -= 1;
    }
    let c = &c[..n];
    if n <= 1 {
        return Vec::new();
    }
    if n == 2 {
        let r = -c[0] / c[1];
        return if r >= lo && r <= hi { vec![Root { t: r, double: false }] } else { Vec::new() };
    }
    let lead = c[n - 1].abs();
    let bound = 1.0 + c[..n - 1].iter().fold(0.0f64, |m, v| m.max(v.abs())) / lead;
    let (lo, hi) = (lo.max(-bound), hi.min(bound));
    if lo > hi {
        return Vec::new();
    }
    let deriv: Vec<f64> = c.iter().enumerate().skip(1).map(|(i, v)| v * i as f64).collect();
    let crit: Vec<f64> = real_roots(&deriv, lo, hi).into_iter().map(|r| r.t).collect();
    let eval = |t: f64| c.iter().rev().fold(0.0, |acc, v| acc * t + v);
    let abs_eval = |t: f64| c.iter().rev().fold(0.0, |acc, v| acc * t.abs() + v.abs());
    let mut pts = Vec::with_capacity(crit.len() + 2);
    pts.push(lo);
    pts.extend(crit.iter().copied());
    pts.push(hi);
    let mut roots: Vec<Root> = Vec::new();
    for w in pts.windows(2) {
        let (mut a, mut b) = (w[0], w[1]);
        let (mut fa, fb) = (eval(a), eval(b));
        if fa == 0.0 {
            roots.push(Root { t: a, double: false });
        } else if fa * fb < 0.0 {
            for _ in 0..200 {
                let m = 0.5 * (a + b);
                if m <= a || m >= b {
                    break;
                }
                let fm = eval(m);
                if fm == 0.0 {
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
            roots.push(Root { t: 0.5 * (a + b), double: false });
        }
        if fb == 0.0 {
            roots.push(Root { t: b, double: false });
        }
    }
    for &t in &crit {
        if eval(t).abs() <= 1e-12 * abs_eval(t) {
            roots.push(Root { t, double: true });
        }
    }
    roots.sort_by(|a, b| a.t.total_cmp(&b.t));
    let mut merged: Vec<Root> = Vec::with_capacity(roots.len());
    for r in roots {
        match merged.last_mut() {
            Some(l) if (r.t - l.t).abs() <= 1e-9 * (1.0 + l.t.abs()) => {
                if r.double && !l.double {
                    l.t = r.t;
                }
                l.double |= r.double;
            }
            _ => merged.push(r),
        }
    }
    merged
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MemoryStore, SurfaceDef, store::GeomStoreMut};

    fn ray_x(x0: f64) -> Ray {
        Ray { origin: Point3::new(x0, 0.0, 0.0), dir: Vector3::new(1.0, 0.0, 0.0) }
    }

    #[test]
    fn sphere_ray_hits_analytic() {
        let q = Quadric::sphere(&Frame::at(Point3::new(0.0, 0.0, 0.0)), 2.0);
        let mut out = Vec::new();
        q.ray_hits(&ray_x(-5.0), Interval::new(0.0, 100.0), &mut out).unwrap();
        assert_eq!(out.len(), 2);
        assert!((out[0].t - 3.0).abs() < 1e-12 && (out[1].t - 7.0).abs() < 1e-12);
        assert!((out[0].normal.x + 1.0).abs() < 1e-12);
        assert_eq!(out[0].crossing, Crossing::AgainstNormal);
        assert_eq!(out[1].crossing, Crossing::AlongNormal);
        assert!(out[1].leaving(&ray_x(-5.0)));
    }

    #[test]
    fn tangent_and_miss() {
        let q = Quadric::sphere(&Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0);
        let mut out = Vec::new();
        let grazing = Ray { origin: Point3::new(-3.0, 1.0, 0.0), dir: Vector3::new(1.0, 0.0, 0.0) };
        q.ray_hits(&grazing, Interval::new(0.0, 10.0), &mut out).unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].crossing, Crossing::Tangent);
        assert!((out[0].t - 3.0).abs() < 1e-12);
        out.clear();
        let miss = Ray { origin: Point3::new(-3.0, 1.5, 0.0), dir: Vector3::new(1.0, 0.0, 0.0) };
        q.ray_hits(&miss, Interval::new(0.0, 10.0), &mut out).unwrap();
        assert!(out.is_empty());
    }

    #[test]
    fn plane_and_offset_frame() {
        let f = Frame::from_z(Point3::new(1.0, 2.0, 3.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let q = Quadric::plane(&f);
        assert!((q.value(Point3::new(9.0, 9.0, 5.0)) - 2.0).abs() < 1e-12);
        let r = Ray { origin: Point3::new(0.0, 0.0, 0.0), dir: Vector3::new(0.0, 0.0, 1.0) };
        let mut out = Vec::new();
        q.ray_hits(&r, Interval::new(0.0, 10.0), &mut out).unwrap();
        assert_eq!(out.len(), 1);
        assert!((out[0].t - 3.0).abs() < 1e-12);
        assert_eq!(out[0].crossing, Crossing::AlongNormal);
    }

    #[test]
    fn torus_ray_through_axis_plane_and_graze() {
        let t = ImplicitTorus { frame: Frame::at(Point3::new(0.0, 0.0, 0.0)), major: 3.0, minor: 1.0 };
        let mut out = Vec::new();
        t.ray_hits(&ray_x(-10.0), Interval::new(0.0, 100.0), &mut out).unwrap();
        let ts: Vec<f64> = out.iter().map(|h| h.t).collect();
        assert_eq!(ts.len(), 4);
        for (got, want) in ts.iter().zip([6.0, 8.0, 12.0, 14.0]) {
            assert!((got - want).abs() < 1e-9, "{got} vs {want}");
        }
        let crossings: Vec<Crossing> = out.iter().map(|h| h.crossing).collect();
        assert_eq!(crossings, vec![Crossing::AgainstNormal, Crossing::AlongNormal, Crossing::AgainstNormal, Crossing::AlongNormal]);
        // Tube top grazes at z = r: two double roots (x = -3 and x = 3), each reported once as Tangent.
        out.clear();
        let graze = Ray { origin: Point3::new(-10.0, 0.0, 1.0), dir: Vector3::new(1.0, 0.0, 0.0) };
        t.ray_hits(&graze, Interval::new(0.0, 100.0), &mut out).unwrap();
        assert_eq!(out.len(), 2);
        assert!(out.iter().all(|h| h.crossing == Crossing::Tangent));
        assert!((out[0].t - 7.0).abs() < 1e-6 && (out[1].t - 13.0).abs() < 1e-6, "{:?}", out.iter().map(|h| h.t).collect::<Vec<_>>());
        // Just below the graze height the contacts become four simple crossings.
        out.clear();
        let below = Ray { origin: Point3::new(-10.0, 0.0, 0.99), dir: Vector3::new(1.0, 0.0, 0.0) };
        t.ray_hits(&below, Interval::new(0.0, 100.0), &mut out).unwrap();
        assert_eq!(out.len(), 4);
        assert!(out.iter().all(|h| h.crossing != Crossing::Tangent));
    }

    #[test]
    fn halfspace_membership() {
        let q = Quadric::sphere(&Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0);
        let id = EntityId::new(sm_core::EntityKind::Surface, 0).unwrap();
        let inside = HalfSpace { surface: id, sign: Sign::Negative };
        assert!(inside.contains(&q, Point3::new(0.5, 0.0, 0.0), 0.0));
        assert!(!inside.contains(&q, Point3::new(2.0, 0.0, 0.0), 0.0));
        assert!((inside.depth(&q, Point3::new(0.5, 0.0, 0.0)) - 0.75).abs() < 1e-12, "first-order depth f/|grad f|");
    }

    #[test]
    fn polynomial_roots_and_multiplicity() {
        // (t-1)(t-2)(t+3) = t^3 - 7t + 6
        let r = real_roots(&[6.0, -7.0, 0.0, 1.0], f64::NEG_INFINITY, f64::INFINITY);
        assert_eq!(r.len(), 3);
        for (g, w) in r.iter().zip([-3.0, 1.0, 2.0]) {
            assert!((g.t - w).abs() < 1e-12 && !g.double);
        }
        // (t-1)^2 (t+2) = t^3 - 3t + 2
        let r = real_roots(&[2.0, -3.0, 0.0, 1.0], f64::NEG_INFINITY, f64::INFINITY);
        assert_eq!(r.len(), 2);
        assert!(!r[0].double && (r[0].t + 2.0).abs() < 1e-12);
        assert!(r[1].double && (r[1].t - 1.0).abs() < 1e-9);
    }

    #[test]
    fn quadric_transform_matches_pointwise() {
        let q = Quadric::from_local(&Frame::at(Point3::new(0.3, 0.1, -0.2)), [1.0, 2.0, 0.5], [0.2, -0.1, 0.4], -1.0);
        let t = sm_core::Transform::rotation(Point3::new(1.0, 0.0, 0.0), Vector3::new(1.0, 2.0, 3.0), 0.7).unwrap().compose(&sm_core::Transform::translation(Vector3::new(0.5, -1.0, 2.0)));
        let qt = q.transformed(&t).unwrap();
        for p in [Point3::new(0.1, 0.2, 0.3), Point3::new(-1.0, 2.0, 0.5)] {
            assert!((qt.value(t.apply_point(p)) - q.value(p)).abs() < 1e-12);
        }
        let mirror = sm_core::Transform::reflection(Vector3::new(1.0, 0.0, 0.0), 1.0).unwrap();
        let qm = q.transformed(&mirror).unwrap();
        let p = Point3::new(0.4, 0.5, 0.6);
        assert!((qm.value(mirror.apply_point(p)) - q.value(p)).abs() < 1e-12);
    }

    #[test]
    fn csg_cell_from_halfspaces_with_general_quadric() {
        // Cell: unit-radius sphere intersected with the half-space z >= 0 and a general quadric
        // (ellipsoid x^2/4 + y^2 + z^2 <= 1.5), the latter stored as SurfaceDef::Implicit.
        let mut store = MemoryStore::new();
        let sphere = store.add_surface(SurfaceDef::Sphere(crate::Sphere::new(Frame::at(Point3::default()), 1.0).unwrap())).unwrap();
        let plane = store.add_surface(SurfaceDef::Plane(crate::Plane { frame: Frame::at(Point3::default()) })).unwrap();
        let gq = Quadric { coeffs: [0.25, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.5] };
        let ell = store.add_surface(SurfaceDef::Implicit(ImplicitDef::Quadric(gq))).unwrap();
        let cell = CsgCell::new([
            HalfSpace { surface: sphere, sign: Sign::Negative },
            HalfSpace { surface: plane, sign: Sign::Positive },
            HalfSpace { surface: ell, sign: Sign::Negative },
        ]);
        assert_eq!(cell.contains(&store, Point3::new(0.0, 0.0, 0.5), 1e-9).unwrap(), Containment::Inside);
        assert_eq!(cell.contains(&store, Point3::new(0.0, 0.0, -0.5), 1e-9).unwrap(), Containment::Outside);
        assert_eq!(cell.contains(&store, Point3::new(0.0, 0.0, 1.0), 1e-9).unwrap(), Containment::On);
        assert_eq!(cell.contains(&store, Point3::new(0.5, 0.5, 0.0), 1e-9).unwrap(), Containment::On, "on the z = 0 face");
        // Ray along +z from below: enters through the plane at t=1 (z=0), leaves the sphere at z=1.
        let ray = Ray { origin: Point3::new(0.0, 0.0, -1.0), dir: Vector3::new(0.0, 0.0, 1.0) };
        let mut out = Vec::new();
        cell.ray_hits(&store, &ray, Interval::new(0.0, 10.0), &mut out).unwrap();
        assert_eq!(out.len(), 2, "{out:?}");
        assert!((out[0].t - 1.0).abs() < 1e-9 && out[0].crossing == Crossing::AgainstNormal);
        assert!((out[0].normal.z + 1.0).abs() < 1e-9, "outward normal of the z>=0 face points down");
        assert!((out[1].t - 2.0).abs() < 1e-9 && out[1].crossing == Crossing::AlongNormal);
        // The ellipsoid x-axis semi-axis is sqrt(6)=2.449 > 1: along x it is the sphere that bounds.
        out.clear();
        let rx = Ray { origin: Point3::new(-3.0, 0.0, 0.1), dir: Vector3::new(1.0, 0.0, 0.0) };
        cell.ray_hits(&store, &rx, Interval::new(0.0, 10.0), &mut out).unwrap();
        let expect = (1.0f64 - 0.01).sqrt();
        assert_eq!(out.len(), 2);
        assert!((out[0].point.x + expect).abs() < 1e-9 && (out[1].point.x - expect).abs() < 1e-9);
    }
}
