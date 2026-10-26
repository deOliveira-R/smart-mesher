//! Tiny `[f64; 3]` helpers for the hot loops (no `Point3` conversions, all `#[inline]`).

#[inline]
pub(crate) fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
#[inline]
pub(crate) fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
#[inline]
pub(crate) fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}
#[inline]
pub(crate) fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
#[inline]
pub(crate) fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
#[inline]
pub(crate) fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}
/// `a + b * s`.
#[inline]
pub(crate) fn madd(a: [f64; 3], b: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s]
}
#[inline]
pub(crate) fn finite3(a: [f64; 3]) -> bool {
    a[0].is_finite() && a[1].is_finite() && a[2].is_finite()
}
#[inline]
pub(crate) fn linf(a: [f64; 3]) -> f64 {
    a[0].abs().max(a[1].abs()).max(a[2].abs())
}
