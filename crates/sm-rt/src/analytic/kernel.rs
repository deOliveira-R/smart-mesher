//! Surface kinds and their conditioned ray kernels.
//!
//! Each distinct surface is classified once at build from its `SurfaceDef` (not from quadric
//! coefficients). `hits` returns the ray parameters `t` in `(0, t_max]` ascending; tangent
//! (double-root) contacts give no event. Spheres, cylinders and cones use the closest-approach
//! form (the perpendicular offset is formed directly, never `b^2 - 4ac` on a far origin); general
//! quadrics evaluate their ray polynomial in double-double; tori solve the quartic about the
//! point of closest approach to the axis centre and polish each root with Newton steps on
//! compensated Horner residuals. No heap allocation on any path.

use crate::vec3::{add, cross, dot, madd, norm, scale, sub};
use sm_geom_api::{Frame, GeomStore, ImplicitDef, SurfaceDef};
use sm_core::EntityId;

pub(crate) type V3 = [f64; 3];

/// Relative width of the tangent band: a chord shorter than ~`1e-7` of the radius is no event.
const TANGENT: f64 = 1e-14;

fn pt(p: sm_core::Point3) -> V3 {
    [p.x, p.y, p.z]
}
fn vc(v: sm_core::Vector3) -> V3 {
    [v.x, v.y, v.z]
}

/// A classified surface. `sd` has the sign of the implicit function the half-spaces use.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Surf {
    Plane { o: V3, n: V3 },
    Sphere { c: V3, r: f64 },
    Cylinder { o: V3, a: V3, r: f64 },
    /// Double cone, apex `o`, axis `a`, `k2 = tan^2` of the half-angle.
    Cone { o: V3, a: V3, k2: f64, cos: f64, sin: f64 },
    Torus { o: V3, x: V3, y: V3, z: V3, big: f64, small: f64 },
    General { q: [f64; 10] },
}

/// Why a surface cannot be classified.
pub(crate) enum ClassifyError {
    Unknown,
    Degenerate(&'static str),
}

fn frame_torus(f: &Frame, big: f64, small: f64) -> Result<Surf, ClassifyError> {
    if !(small.is_finite() && small > 0.0 && big.is_finite() && big > small) {
        return Err(ClassifyError::Degenerate("torus needs major > minor > 0"));
    }
    Ok(Surf::Torus { o: pt(f.origin), x: vc(f.x), y: vc(f.y), z: vc(f.z), big, small })
}

impl Surf {
    /// Classify the surface `id` of `geom`.
    pub(crate) fn classify(geom: &dyn GeomStore, id: EntityId) -> Result<Self, ClassifyError> {
        let def = geom.surface_def(id).ok_or(ClassifyError::Unknown)?;
        let pos = |v: f64| v.is_finite() && v > 0.0;
        match def {
            SurfaceDef::Plane(p) => {
                let n = vc(p.frame.z);
                if norm(n).partial_cmp(&0.0) != Some(core::cmp::Ordering::Greater) {
                    return Err(ClassifyError::Degenerate("plane normal is zero"));
                }
                Ok(Surf::Plane { o: pt(p.frame.origin), n })
            }
            SurfaceDef::Sphere(s) if pos(s.radius) => Ok(Surf::Sphere { c: pt(s.frame.origin), r: s.radius }),
            SurfaceDef::Sphere(_) => Err(ClassifyError::Degenerate("sphere radius must be positive")),
            SurfaceDef::Cylinder(c) if pos(c.radius) => Ok(Surf::Cylinder { o: pt(c.frame.origin), a: vc(c.frame.z), r: c.radius }),
            SurfaceDef::Cylinder(_) => Err(ClassifyError::Degenerate("cylinder radius must be positive")),
            SurfaceDef::Cone(c) => {
                if !(c.half_angle > 0.0 && c.half_angle < core::f64::consts::FRAC_PI_2) || !c.radius.is_finite() || c.radius < 0.0 {
                    return Err(ClassifyError::Degenerate("cone half-angle must be in (0, pi/2)"));
                }
                let t = c.half_angle.tan();
                Ok(Surf::Cone { o: pt(c.apex()), a: vc(c.frame.z), k2: t * t, cos: c.half_angle.cos(), sin: c.half_angle.sin() })
            }
            SurfaceDef::Torus(t) => frame_torus(&t.frame, t.major, t.minor),
            other => match other.implicit_def() {
                Some(ImplicitDef::Torus(t)) => frame_torus(&t.frame, t.major, t.minor),
                Some(ImplicitDef::Quadric(q)) => {
                    if q.coeffs.iter().any(|c| !c.is_finite()) || q.coeffs[..9].iter().all(|&c| c == 0.0) {
                        return Err(ClassifyError::Degenerate("quadric has non-finite or all-zero coefficients"));
                    }
                    Ok(Surf::General { q: q.coeffs })
                }
                _ => Err(ClassifyError::Unknown),
            },
        }
    }

    /// Hash words that describe the surface by value.
    pub(crate) fn words(&self, out: &mut Vec<u64>) {
        let mut f = |v: &[f64]| out.extend(v.iter().map(|x| if *x == 0.0 { 0 } else { x.to_bits() }));
        match self {
            Surf::Plane { o, n } => {
                f(&[1.0]);
                f(o);
                f(n);
            }
            Surf::Sphere { c, r } => {
                f(&[2.0]);
                f(c);
                f(&[*r]);
            }
            Surf::Cylinder { o, a, r } => {
                f(&[3.0]);
                f(o);
                f(a);
                f(&[*r]);
            }
            Surf::Cone { o, a, k2, .. } => {
                f(&[4.0]);
                f(o);
                f(a);
                f(&[*k2]);
            }
            Surf::Torus { o, x, y, z, big, small } => {
                f(&[5.0]);
                f(o);
                f(x);
                f(y);
                f(z);
                f(&[*big, *small]);
            }
            Surf::General { q } => {
                f(&[6.0]);
                f(q);
            }
        }
    }

    /// Short kind name (for diagnostics).
    pub(crate) fn kind(&self) -> &'static str {
        match self {
            Surf::Plane { .. } => "plane",
            Surf::Sphere { .. } => "sphere",
            Surf::Cylinder { .. } => "cylinder",
            Surf::Cone { .. } => "cone",
            Surf::Torus { .. } => "torus",
            Surf::General { .. } => "quadric",
        }
    }

    /// Signed distance-like value, positive on the side the implicit function is positive.
    /// Exact for planes, spheres, cylinders, cones (both nappes) and tori; first order for a
    /// general quadric (`f / |grad f|`).
    pub(crate) fn sd(&self, p: V3) -> f64 {
        match self {
            Surf::Plane { o, n } => dot(*n, sub(p, *o)),
            Surf::Sphere { c, r } => norm(sub(p, *c)) - r,
            Surf::Cylinder { o, a, r } => norm(perp(sub(p, *o), *a)) - r,
            Surf::Cone { o, a, cos, sin, .. } => {
                let q = sub(p, *o);
                let z = dot(q, *a);
                norm(perp(q, *a)) * cos - z.abs() * sin
            }
            Surf::Torus { o, x, y, z, big, small } => {
                let q = sub(p, *o);
                let (lx, ly, lz) = (dot(q, *x), dot(q, *y), dot(q, *z));
                (lx.hypot(ly) - big).hypot(lz) - small
            }
            Surf::General { q } => {
                let f = quad_val(q, p);
                let g = norm(quad_grad(q, p));
                if g > 0.0 { f / g } else { f }
            }
        }
    }

    /// Unit gradient (the outward normal toward the positive side); zero at a singular point.
    pub(crate) fn normal(&self, p: V3) -> V3 {
        match self {
            Surf::Plane { n, .. } => *n,
            Surf::Sphere { c, .. } => unit_or(sub(p, *c), [1.0, 0.0, 0.0]),
            Surf::Cylinder { o, a, .. } => unit_or(perp(sub(p, *o), *a), any_perp(*a)),
            Surf::Cone { o, a, k2, .. } => {
                let q = sub(p, *o);
                let z = dot(q, *a);
                let g = sub(perp(q, *a), scale(*a, k2 * z));
                unit_or(g, [0.0; 3])
            }
            Surf::Torus { o, x, y, z, big, .. } => {
                let q = sub(p, *o);
                let (lx, ly, lz) = (dot(q, *x), dot(q, *y), dot(q, *z));
                let rho = lx.hypot(ly);
                let k = if rho > 0.0 { 1.0 - big / rho } else { 0.0 };
                let (gx, gy) = if rho > 0.0 { (lx * k, ly * k) } else { (-big, 0.0) };
                let g = add(add(scale(*x, gx), scale(*y, gy)), scale(*z, lz));
                unit_or(g, [0.0; 3])
            }
            Surf::General { q } => unit_or(quad_grad(q, p), [0.0; 3]),
        }
    }

    /// Ray parameters `t` in `(0, t_max]`, ascending, written to `out`; returns the count.
    /// `d` is unit. Tangent contacts are dropped.
    pub(crate) fn hits(&self, o: V3, d: V3, t_max: f64, out: &mut [f64; 4]) -> usize {
        let mut raw = [0.0f64; 4];
        let n = match self {
            Surf::Plane { o: p0, n } => {
                let den = dot(*n, d);
                if den == 0.0 {
                    0
                } else {
                    raw[0] = -dot(*n, sub(o, *p0)) / den;
                    1
                }
            }
            Surf::Sphere { c, r } => {
                let q = sub(o, *c);
                let tc = -dot(q, d);
                chord(&mut raw, tc, norm(madd(q, d, tc)), *r, 1.0)
            }
            Surf::Cylinder { o: p0, a, r } => {
                let q = perp(sub(o, *p0), *a);
                let dp = perp(d, *a);
                let aa = dot(dp, dp);
                if aa < 1e-28 {
                    0
                } else {
                    let tc = -dot(q, dp) / aa;
                    chord(&mut raw, tc, norm(madd(q, dp, tc)), *r, aa)
                }
            }
            Surf::Cone { o: apex, a, k2, .. } => cone_hits(&mut raw, sub(o, *apex), d, *a, *k2),
            Surf::Torus { o: c, x, y, z, big, small } => torus_hits(&mut raw, sub(o, *c), d, [*x, *y, *z], *big, *small, t_max),
            Surf::General { q } => general_hits(&mut raw, q, o, d),
        };
        let mut k = 0;
        for &t in &raw[..n] {
            if t > 0.0 && t <= t_max {
                out[k] = t;
                k += 1;
            }
        }
        out[..k].sort_by(f64::total_cmp);
        k
    }

    /// Exact closest point of the surface to `p`: `(distance, point)`. `None` for a general
    /// quadric.
    pub(crate) fn closest(&self, p: V3) -> Option<(f64, V3)> {
        match self {
            Surf::Plane { o, n } => {
                let s = dot(*n, sub(p, *o));
                Some((s.abs(), madd(p, *n, -s)))
            }
            Surf::Sphere { c, r } => {
                let q = sub(p, *c);
                let l = norm(q);
                let u = if l > 0.0 { scale(q, 1.0 / l) } else { [1.0, 0.0, 0.0] };
                Some(((l - r).abs(), madd(*c, u, *r)))
            }
            Surf::Cylinder { o, a, r } => {
                let q = sub(p, *o);
                let ax = dot(q, *a);
                let pp = perp(q, *a);
                let l = norm(pp);
                let u = if l > 0.0 { scale(pp, 1.0 / l) } else { any_perp(*a) };
                Some(((l - r).abs(), add(madd(*o, *a, ax), scale(u, *r))))
            }
            Surf::Cone { o, a, cos, sin, .. } => {
                let q = sub(p, *o);
                let z = dot(q, *a);
                let pp = perp(q, *a);
                let rho = norm(pp);
                let u = if rho > 0.0 { scale(pp, 1.0 / rho) } else { any_perp(*a) };
                // Meridian half-plane (rho, z): generators from the apex along (sin, +-cos).
                let mut best = (f64::INFINITY, [0.0; 3]);
                for sg in [1.0, -1.0] {
                    let (gr, gz) = (*sin, sg * cos);
                    let s = (rho * gr + z * gz).max(0.0);
                    let (qr, qz) = (s * gr, s * gz);
                    let dist = (rho - qr).hypot(z - qz);
                    if dist < best.0 {
                        best = (dist, add(madd(*o, *a, qz), scale(u, qr)));
                    }
                }
                Some(best)
            }
            Surf::Torus { o, x, y, z, big, small } => {
                let q = sub(p, *o);
                let (lx, ly, lz) = (dot(q, *x), dot(q, *y), dot(q, *z));
                let rho = lx.hypot(ly);
                let (ux, uy) = if rho > 0.0 { (lx / rho, ly / rho) } else { (1.0, 0.0) };
                let (dr, dz) = (rho - big, lz);
                let l = dr.hypot(dz);
                let (er, ez) = if l > 0.0 { (dr / l, dz / l) } else { (1.0, 0.0) };
                let (qr, qz) = (big + small * er, small * ez);
                let loc = [ux * qr, uy * qr, qz];
                let w = add(add(scale(*x, loc[0]), scale(*y, loc[1])), scale(*z, loc[2]));
                Some(((l - small).abs(), add(*o, w)))
            }
            Surf::General { .. } => None,
        }
    }
}

/// Component of `v` perpendicular to the unit `a`.
fn perp(v: V3, a: V3) -> V3 {
    madd(v, a, -dot(v, a))
}

fn unit_or(v: V3, fallback: V3) -> V3 {
    let l = norm(v);
    if l > 0.0 && l.is_finite() { scale(v, 1.0 / l) } else { fallback }
}

/// A deterministic unit vector perpendicular to the unit `a`.
fn any_perp(a: V3) -> V3 {
    let h = if a[0].abs() <= a[1].abs() && a[0].abs() <= a[2].abs() {
        [1.0, 0.0, 0.0]
    } else if a[1].abs() <= a[2].abs() {
        [0.0, 1.0, 0.0]
    } else {
        [0.0, 0.0, 1.0]
    };
    unit_or(cross(a, h), [1.0, 0.0, 0.0])
}

/// Two roots `tc -+ sqrt((r - m)(r + m) / aa)` of a chord at perpendicular offset `m`.
fn chord(out: &mut [f64; 4], tc: f64, m: f64, r: f64, aa: f64) -> usize {
    if r - m <= TANGENT * r {
        return 0;
    }
    let half = ((r - m) * (r + m) / aa).sqrt();
    out[0] = tc - half;
    out[1] = tc + half;
    2
}

/// Double cone about the apex: shift the origin to the closest approach to the apex, solve the
/// quadratic in the axial/radial split, shift back.
fn cone_hits(out: &mut [f64; 4], q0: V3, d: V3, a: V3, k2: f64) -> usize {
    let tc = -dot(q0, d);
    let q = madd(q0, d, tc);
    let (qa, da) = (dot(q, a), dot(d, a));
    let (qp, dp) = (madd(q, a, -qa), madd(d, a, -da));
    let aa = dot(dp, dp) - k2 * da * da;
    let bb = dot(qp, dp) - k2 * qa * da;
    let cc = dot(qp, qp) - k2 * qa * qa;
    let scale_a = dot(dp, dp) + k2 * da * da;
    if aa.abs() <= 1e-14 * scale_a {
        // Ray parallel to a generator: one crossing.
        if bb == 0.0 {
            return 0;
        }
        out[0] = tc - cc / (2.0 * bb);
        return 1;
    }
    let disc = bb * bb - aa * cc;
    if disc <= TANGENT * (bb * bb + (aa * cc).abs()) {
        return 0;
    }
    let sq = disc.sqrt();
    let qq = -(bb + bb.signum() * sq);
    let (r1, r2) = (qq / aa, if qq != 0.0 { cc / qq } else { qq / aa });
    out[0] = tc + r1.min(r2);
    out[1] = tc + r1.max(r2);
    2
}

// ---- double-double -------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct Dd(f64, f64);

fn two_sum(a: f64, b: f64) -> (f64, f64) {
    let s = a + b;
    let bb = s - a;
    (s, (a - (s - bb)) + (b - bb))
}

impl Dd {
    fn prod(a: f64, b: f64) -> Dd {
        let p = a * b;
        Dd(p, a.mul_add(b, -p))
    }
    fn mul(self, b: f64) -> Dd {
        let p = self.0 * b;
        let e = self.0.mul_add(b, -p) + self.1 * b;
        let (h, l) = two_sum(p, e);
        Dd(h, l)
    }
    fn add(self, o: Dd) -> Dd {
        let (s, e) = two_sum(self.0, o.0);
        let (h, l) = two_sum(s, e + self.1 + o.1);
        Dd(h, l)
    }
    fn val(self) -> f64 {
        self.0 + self.1
    }
}

fn quad_val(q: &[f64; 10], p: V3) -> f64 {
    let [a, b, c, d, e, f, g, h, j, k] = *q;
    let [x, y, z] = p;
    a * x * x + b * y * y + c * z * z + d * x * y + e * y * z + f * z * x + g * x + h * y + j * z + k
}

fn quad_grad(q: &[f64; 10], p: V3) -> V3 {
    let [a, b, c, d, e, f, g, h, j, _] = *q;
    let [x, y, z] = p;
    [2.0 * a * x + d * y + f * z + g, 2.0 * b * y + d * x + e * z + h, 2.0 * c * z + e * y + f * x + j]
}

/// General quadric: the ray polynomial `c0 + c1 t + c2 t^2` in double-double, then the stable
/// quadratic formula.
fn general_hits(out: &mut [f64; 4], q: &[f64; 10], o: V3, d: V3) -> usize {
    let [a, b, c, dd, e, f, g, h, j, k] = *q;
    let [x, y, z] = o;
    let [vx, vy, vz] = d;
    let t3 = |co: f64, u: f64, w: f64| Dd::prod(co, u).mul(w);
    let mut c0 = Dd(k, 0.0);
    for t in [t3(a, x, x), t3(b, y, y), t3(c, z, z), t3(dd, x, y), t3(e, y, z), t3(f, z, x), Dd::prod(g, x), Dd::prod(h, y), Dd::prod(j, z)] {
        c0 = c0.add(t);
    }
    let mut c1 = Dd(0.0, 0.0);
    for t in [
        t3(2.0 * a, x, vx),
        t3(dd, y, vx),
        t3(f, z, vx),
        Dd::prod(g, vx),
        t3(dd, x, vy),
        t3(2.0 * b, y, vy),
        t3(e, z, vy),
        Dd::prod(h, vy),
        t3(f, x, vz),
        t3(e, y, vz),
        t3(2.0 * c, z, vz),
        Dd::prod(j, vz),
    ] {
        c1 = c1.add(t);
    }
    let mut c2 = Dd(0.0, 0.0);
    for t in [t3(a, vx, vx), t3(b, vy, vy), t3(c, vz, vz), t3(dd, vx, vy), t3(e, vy, vz), t3(f, vz, vx)] {
        c2 = c2.add(t);
    }
    let (c0, c1, c2) = (c0.val(), c1.val(), c2.val());
    let scale_all = c0.abs().max(c1.abs()).max(c2.abs());
    if scale_all == 0.0 {
        return 0;
    }
    if c2.abs() <= 1e-14 * scale_all {
        if c1.abs() <= 1e-14 * scale_all {
            return 0;
        }
        out[0] = -c0 / c1;
        return 1;
    }
    let disc = c1 * c1 - 4.0 * c2 * c0;
    let ds = c1 * c1 + (4.0 * c2 * c0).abs();
    if disc <= TANGENT * ds {
        return 0;
    }
    let sq = disc.sqrt();
    let qq = -0.5 * (c1 + c1.signum() * sq);
    let (r1, r2) = (qq / c2, if qq != 0.0 { c0 / qq } else { qq / c2 });
    out[0] = r1.min(r2);
    out[1] = r1.max(r2);
    2
}

// ---- torus ---------------------------------------------------------------------------------

/// Compensated Horner (Graillat): value of the polynomial with ascending coefficients `c` at
/// `t`, as if computed in twice the working precision.
fn horner_comp(c: &[f64], t: f64) -> f64 {
    let n = c.len();
    let mut s = c[n - 1];
    let mut err = 0.0;
    for i in (0..n - 1).rev() {
        let p = s * t;
        let pe = s.mul_add(t, -p);
        let (sum, se) = two_sum(p, c[i]);
        s = sum;
        err = err * t + (pe + se);
    }
    s + err
}

fn horner_d(c: &[f64], t: f64) -> f64 {
    let n = c.len();
    let mut s = 0.0;
    for i in (1..n).rev() {
        s = s * t + c[i] * i as f64;
    }
    s
}

fn eval_plain(c: &[f64], t: f64) -> f64 {
    c.iter().rev().fold(0.0, |a, v| a * t + v)
}

fn eval_abs(c: &[f64], t: f64) -> f64 {
    c.iter().rev().fold(0.0, |a, v| a * t.abs() + v.abs())
}

/// Real roots of the polynomial `c` (ascending, degree <= 4) inside `[lo, hi]`; the flag marks
/// a double root. Fixed-size, allocation-free (the recursion isolates monotone pieces with the
/// roots of the derivative).
fn poly_roots(c: &[f64], lo: f64, hi: f64, out: &mut [(f64, bool); 4]) -> usize {
    let maxabs = c.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    let mut n = c.len();
    while n > 1 && c[n - 1].abs() <= 1e-14 * maxabs {
        n -= 1;
    }
    let c = &c[..n];
    if n <= 1 || lo > hi {
        return 0;
    }
    if n == 2 {
        let r = -c[0] / c[1];
        if r >= lo && r <= hi {
            out[0] = (r, false);
            return 1;
        }
        return 0;
    }
    let lead = c[n - 1].abs();
    let bound = 1.0 + c[..n - 1].iter().fold(0.0f64, |m, v| m.max(v.abs())) / lead;
    let (lo, hi) = (lo.max(-bound), hi.min(bound));
    if lo > hi {
        return 0;
    }
    let mut dc = [0.0f64; 4];
    for i in 1..n {
        dc[i - 1] = c[i] * i as f64;
    }
    let mut crit = [(0.0, false); 4];
    let nc = poly_roots(&dc[..n - 1], lo, hi, &mut crit);
    let mut pts = [0.0f64; 6];
    pts[0] = lo;
    for i in 0..nc {
        pts[1 + i] = crit[i].0;
    }
    pts[nc + 1] = hi;
    let np = nc + 2;
    let mut roots = [(0.0f64, false); 8];
    let mut nr = 0;
    for w in 0..np - 1 {
        let (mut a, mut b) = (pts[w], pts[w + 1]);
        let (mut fa, fb) = (eval_plain(c, a), eval_plain(c, b));
        if fa == 0.0 {
            roots[nr] = (a, false);
            nr += 1;
        } else if fa * fb < 0.0 {
            for _ in 0..200 {
                let m = 0.5 * (a + b);
                if m <= a || m >= b {
                    break;
                }
                let fm = eval_plain(c, m);
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
            roots[nr] = (0.5 * (a + b), false);
            nr += 1;
        }
        if fb == 0.0 {
            roots[nr] = (b, false);
            nr += 1;
        }
    }
    for &(t, _) in &crit[..nc] {
        if eval_plain(c, t).abs() <= 1e-12 * eval_abs(c, t) {
            let near = 1e-5 * (1.0 + t.abs());
            let mut k = 0;
            for j in 0..nr {
                if (roots[j].0 - t).abs() > near {
                    roots[k] = roots[j];
                    k += 1;
                }
            }
            nr = k;
            roots[nr] = (t, true);
            nr += 1;
        }
    }
    roots[..nr].sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut m = 0;
    let mut merged = [(0.0f64, false); 8];
    for r in &roots[..nr] {
        if m > 0 && (r.0 - merged[m - 1].0).abs() <= 1e-9 * (1.0 + merged[m - 1].0.abs()) {
            if r.1 && !merged[m - 1].1 {
                merged[m - 1].0 = r.0;
            }
            merged[m - 1].1 |= r.1;
        } else {
            merged[m] = *r;
            m += 1;
        }
    }
    let m = m.min(4);
    out[..m].copy_from_slice(&merged[..m]);
    m
}

/// Torus quartic about the closest approach to its centre, roots polished by Newton steps on
/// compensated residuals. `q0` is the origin relative to the torus centre (world axes).
fn torus_hits(out: &mut [f64; 4], q0: V3, d: V3, ax: [V3; 3], big: f64, small: f64, t_max: f64) -> usize {
    let lo_ = [dot(q0, ax[0]), dot(q0, ax[1]), dot(q0, ax[2])];
    let dl = [dot(d, ax[0]), dot(d, ax[1]), dot(d, ax[2])];
    let tc = -dot(lo_, dl);
    let q = madd(lo_, dl, tc);
    let rr = big * big;
    let k = rr - small * small;
    let (a, b, c) = (dot(dl, dl), dot(q, dl), dot(q, q));
    let (e, g, h) = (dl[0] * dl[0] + dl[1] * dl[1], q[0] * dl[0] + q[1] * dl[1], q[0] * q[0] + q[1] * q[1]);
    let ck = c + k;
    let coef = [ck * ck - 4.0 * rr * h, 4.0 * b * ck - 8.0 * rr * g, 4.0 * b * b + 2.0 * a * ck - 4.0 * rr * e, 4.0 * a * b, a * a];
    let mut roots = [(0.0, false); 4];
    let n = poly_roots(&coef, -tc, t_max - tc, &mut roots);
    let mut m = 0;
    for &(t, dbl) in &roots[..n] {
        if dbl {
            continue;
        }
        let mut t = t;
        for _ in 0..2 {
            let f = horner_comp(&coef, t);
            let fp = horner_d(&coef, t);
            if fp != 0.0 {
                let nt = t - f / fp;
                if nt.is_finite() && (nt - t).abs() <= 1e-6 * (1.0 + t.abs()) {
                    t = nt;
                }
            }
        }
        out[m] = tc + t;
        m += 1;
    }
    out[..m].sort_by(f64::total_cmp);
    m
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quartic_roots_and_double_root() {
        // (t-1)(t-2)(t-3)(t-4)
        let c = [24.0, -50.0, 35.0, -10.0, 1.0];
        let mut r = [(0.0, false); 4];
        let n = poly_roots(&c, -10.0, 10.0, &mut r);
        assert_eq!(n, 4);
        for (i, x) in r.iter().enumerate() {
            assert!((x.0 - (i as f64 + 1.0)).abs() < 1e-9 && !x.1);
        }
        // (t-1)^2 (t^2 + 1): a flagged double root
        let c = [1.0, -2.0, 2.0, -2.0, 1.0];
        let n = poly_roots(&c, -10.0, 10.0, &mut r);
        assert_eq!(n, 1);
        assert!(r[0].1 && (r[0].0 - 1.0).abs() < 1e-6);
    }

    #[test]
    fn sphere_chord_far_from_origin() {
        let s = Surf::Sphere { c: [40.0, -30.0, 20.0], r: 5.0 };
        let mut out = [0.0; 4];
        let n = s.hits([40.0, -30.0 + 3.0, 0.0], [0.0, 0.0, 1.0], 1e9, &mut out);
        assert_eq!(n, 2);
        assert!((out[1] - out[0] - 8.0).abs() < 1e-12);
    }
}
