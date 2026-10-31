//! Predicate wrappers and small vector helpers.
//!
//! **Conventions.** `ori(a,b,c,d) > 0` is a positive-volume tet in the `CellShape::Tet`
//! template (apex above the counter-clockwise base); it is `-sm_math::orient3d`, i.e. the sign
//! of `det(b-a, c-a, d-a)`. For such a tet, `sm_math::insphere` is *negative* when the fifth
//! point is inside the circumsphere; [`in_sphere`] hides that.

use sm_core::Point3;
use sm_math::Sign;
use sm_math::sos::{Tagged, insphere_sos, orient3d_sos};

/// A point.
pub type V3 = [f64; 3];

#[inline]
pub(crate) fn pt(p: V3) -> Point3 {
    Point3::new(p[0], p[1], p[2])
}

#[inline]
pub(crate) fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
#[inline]
pub(crate) fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
#[inline]
pub(crate) fn scale(a: V3, s: f64) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
#[inline]
pub(crate) fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
#[inline]
pub(crate) fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
#[inline]
pub(crate) fn norm2(a: V3) -> f64 {
    dot(a, a)
}
#[inline]
pub(crate) fn dist(a: V3, b: V3) -> f64 {
    norm2(sub(a, b)).sqrt()
}

/// Exact orientation: `1` for a positive-volume tet, `-1` for a negative one, `0` if the four
/// points are coplanar.
#[inline]
pub fn ori(a: V3, b: V3, c: V3, d: V3) -> i32 {
    -sm_math::orient3d(pt(a), pt(b), pt(c), pt(d)).as_i32()
}

/// True if `p` is strictly inside the circumsphere of the positive tet `a,b,c,d` (exact; `false`
/// on the sphere).
#[inline]
pub fn in_sphere(a: V3, b: V3, c: V3, d: V3, p: V3) -> bool {
    sm_math::insphere(pt(a), pt(b), pt(c), pt(d), pt(p)) == Sign::Negative
}

/// Point set with SoS tie-breaking by point index.
#[derive(Clone, Copy)]
pub(crate) struct Pts<'a>(pub &'a [V3]);

impl Pts<'_> {
    #[inline]
    pub fn p(&self, i: u32) -> V3 {
        self.0[i as usize]
    }
    #[inline]
    fn tag(&self, i: u32) -> Tagged<Point3> {
        Tagged::new(pt(self.0[i as usize]), u64::from(i))
    }
    /// Never-zero orientation of four distinct points (SoS by index); `1` = positive volume.
    #[inline]
    pub fn ori_s(&self, a: u32, b: u32, c: u32, d: u32) -> i32 {
        let s = orient3d_sos(self.tag(a), self.tag(b), self.tag(c), self.tag(d)).unwrap_or(Sign::Positive);
        -s.as_i32()
    }
    /// Never-zero "e is inside the sphere of positive tet a,b,c,d" (SoS by index).
    #[inline]
    pub fn in_sphere_s(&self, a: u32, b: u32, c: u32, d: u32, e: u32) -> bool {
        insphere_sos(self.tag(a), self.tag(b), self.tag(c), self.tag(d), self.tag(e))
            .map(|s| s == Sign::Negative)
            .unwrap_or(false)
    }
    /// Exact orientation (zero allowed).
    #[inline]
    pub fn ori(&self, a: u32, b: u32, c: u32, d: u32) -> i32 {
        ori(self.p(a), self.p(b), self.p(c), self.p(d))
    }
}

/// Six times the signed volume (floating point).
#[inline]
pub(crate) fn vol6(a: V3, b: V3, c: V3, d: V3) -> f64 {
    dot(sub(b, a), cross(sub(c, a), sub(d, a)))
}

/// Circumcenter and squared circumradius of a tet; `None` if it is (numerically) flat.
pub(crate) fn circumsphere(a: V3, b: V3, c: V3, d: V3) -> Option<(V3, f64)> {
    let ba = sub(b, a);
    let ca = sub(c, a);
    let da = sub(d, a);
    let det = dot(ba, cross(ca, da));
    if det == 0.0 || !det.is_finite() {
        return None;
    }
    let t = add(
        add(scale(cross(ca, da), norm2(ba)), scale(cross(da, ba), norm2(ca))),
        scale(cross(ba, ca), norm2(da)),
    );
    let o = scale(t, 1.0 / (2.0 * det));
    let r2 = norm2(o);
    if !r2.is_finite() {
        return None;
    }
    Some((add(a, o), r2))
}

/// Mean ratio `12 (3V)^(2/3) / sum(l^2)` of a tet (1 for the regular tet, 0 flat, negative if
/// inverted).
pub(crate) fn mean_ratio(a: V3, b: V3, c: V3, d: V3) -> f64 {
    let v = vol6(a, b, c, d) / 6.0;
    let s = norm2(sub(a, b)) + norm2(sub(a, c)) + norm2(sub(a, d)) + norm2(sub(b, c)) + norm2(sub(b, d)) + norm2(sub(c, d));
    if s <= 0.0 || !s.is_finite() {
        return 0.0;
    }
    let sgn = if v < 0.0 { -1.0 } else { 1.0 };
    sgn * 12.0 * (3.0 * v.abs()).powf(2.0 / 3.0) / s
}

/// Squared distance from `p` to triangle `a,b,c`.
pub(crate) fn dist2_point_tri(p: V3, a: V3, b: V3, c: V3) -> f64 {
    // Ericson, Real-Time Collision Detection, closest point on triangle.
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(p, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return norm2(ap);
    }
    let bp = sub(p, b);
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 {
        return norm2(bp);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return norm2(sub(p, add(a, scale(ab, v))));
    }
    let cp = sub(p, c);
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 {
        return norm2(cp);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return norm2(sub(p, add(a, scale(ac, w))));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return norm2(sub(p, add(b, scale(sub(c, b), w))));
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    norm2(sub(p, add(a, add(scale(ab, v), scale(ac, w)))))
}

/// Closest point of triangle `a,b,c` to `p`.
pub(crate) fn closest_point_tri(p: V3, a: V3, b: V3, c: V3) -> V3 {
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(p, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = sub(p, b);
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return add(a, scale(ab, d1 / (d1 - d3)));
    }
    let cp = sub(p, c);
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return add(a, scale(ac, d2 / (d2 - d6)));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return add(b, scale(sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6))));
    }
    let denom = 1.0 / (va + vb + vc);
    add(a, add(scale(ab, vb * denom), scale(ac, vc * denom)))
}

/// `true` if `x` is usable as a predicate coordinate (zero or within the exact range).
#[inline]
pub(crate) fn coord_ok(x: f64) -> bool {
    sm_math::predicates::in_exact_range(x)
}

/// Snap a candidate point into the exact predicate range (tiny values to zero); `None` if it
/// cannot be represented.
pub(crate) fn sanitize(mut p: V3) -> Option<V3> {
    for x in &mut p {
        if !x.is_finite() {
            return None;
        }
        if !coord_ok(*x) {
            if x.abs() < 1.0 {
                *x = 0.0;
            } else {
                return None;
            }
        }
    }
    Some(p)
}
