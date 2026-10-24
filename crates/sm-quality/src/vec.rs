//! Minimal `[f64; 3]` vector helpers used by the element kernels.

/// A 3-vector / point.
pub type V = [f64; 3];

/// Verdict's "large" sentinel: returned (instead of infinity) by metrics where large is bad.
pub const DBL_MAX: f64 = 1.0e30;
/// Threshold below which a denominator is treated as degenerate. The kernels apply it to
/// coordinates that `Scale` has normalised to a characteristic length in `[1, 2)`, so it acts
/// relative to the element size. This is not Verdict's `VERDICT_DBL_MIN` (2.22e-15, `verdict.h`),
/// which Verdict applies after its own per-element scaling.
pub const TINY: f64 = 1.0e-30;

/// Exact power-of-two scaling of an element: coordinates are multiplied by `2^-k` (exact in
/// floating point), `2^k` being the largest power of two not above the element's characteristic
/// length (its largest coordinate extent from the first corner). Dimensionless metrics are then
/// bit-identical for an element and any power-of-two multiple of it, and dimensional results
/// (length `^ d`) are scaled back by the same exact factor.
#[derive(Clone, Copy)]
pub(crate) struct Scale {
    /// `2^k` (1 if the element has no usable length).
    pub s: f64,
}

impl Scale {
    /// Scale of the point set `p` (identity for coincident or non-finite points).
    pub(crate) fn of(p: &[V]) -> Self {
        let Some(o) = p.first() else { return Self { s: 1.0 } };
        let mut l = 0.0f64;
        for q in p {
            for c in 0..3 {
                l = l.max((q[c] - o[c]).abs());
            }
        }
        if !(l.is_finite() && l > 0.0) {
            return Self { s: 1.0 };
        }
        // Biased exponent of `l` (subnormals: scale up first); `s = 2^floor(log2 l)`, clamped.
        let (x, extra) = if l < f64::MIN_POSITIVE { (l * 2f64.powi(100), 100) } else { (l, 0) };
        let e = ((x.to_bits() >> 52) & 0x7ff) as i32 - 1023 - extra;
        let e = e.clamp(-1000, 1000);
        Self { s: f64::from_bits(((e + 1023) as u64) << 52) }
    }
    /// True if scaling is the identity.
    pub(crate) fn is_identity(self) -> bool {
        self.s == 1.0
    }
    /// The normalised copy of `p`.
    pub(crate) fn down<A: smallvec::Array<Item = V>>(self, p: &[V]) -> smallvec::SmallVec<A> {
        let r = 1.0 / self.s;
        p.iter().map(|q| [q[0] * r, q[1] * r, q[2] * r]).collect()
    }
    /// Scale a quantity of dimension length`^d` from normalised back to original units.
    pub(crate) fn up(self, x: f64, d: i32) -> f64 {
        (0..d).fold(x, |a, _| a * self.s)
    }
    /// Scale a quantity of dimension length`^d` from original to normalised units.
    pub(crate) fn dn(self, x: f64, d: i32) -> f64 {
        let r = 1.0 / self.s;
        (0..d).fold(x, |a, _| a * r)
    }
}

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
