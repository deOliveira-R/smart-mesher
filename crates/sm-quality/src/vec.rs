//! Minimal `[f64; 3]` vector helpers used by the element kernels.

/// A 3-vector / point.
pub type V = [f64; 3];

/// Verdict's "large" sentinel: returned (instead of infinity) by metrics where large is bad.
pub const DBL_MAX: f64 = 1.0e30;
/// Absolute threshold below which a denominator is treated as degenerate. This is not Verdict's
/// `VERDICT_DBL_MIN` (2.22e-15, `verdict.h`), which most Verdict kernels apply after per-element
/// scaling and so act as a relative threshold. The crate's absolute 1e-30 makes cells with edges
/// below about 1e-10 read as degenerate and keeps relatively near-flat cells finite.
pub const TINY: f64 = 1.0e-30;

#[inline]
pub(crate) fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
#[inline]
pub(crate) fn add(a: V, b: V) -> V {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
#[inline]
pub(crate) fn scale(a: V, s: f64) -> V {
    [a[0] * s, a[1] * s, a[2] * s]
}
#[inline]
pub(crate) fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
#[inline]
pub(crate) fn cross(a: V, b: V) -> V {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
#[inline]
pub(crate) fn norm2(a: V) -> f64 {
    dot(a, a)
}
#[inline]
pub(crate) fn norm(a: V) -> f64 {
    norm2(a).sqrt()
}
/// Unit vector, or the zero vector if `a` is (numerically) zero.
#[inline]
pub(crate) fn unit(a: V) -> V {
    let n = norm(a);
    if n <= TINY { [0.0; 3] } else { scale(a, 1.0 / n) }
}
/// Scalar triple product `a . (b x c)`.
#[inline]
pub(crate) fn det3(a: V, b: V, c: V) -> f64 {
    dot(a, cross(b, c))
}
/// Clamp a "large is bad" value into `[-DBL_MAX, DBL_MAX]` (NaN stays NaN).
#[inline]
pub(crate) fn cap(x: f64) -> f64 {
    x.clamp(-DBL_MAX, DBL_MAX)
}
