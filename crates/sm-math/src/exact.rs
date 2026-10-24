//! Error-free transformations and floating-point *expansions* (Shewchuk, "Adaptive Precision
//! Floating-Point Arithmetic and Fast Robust Geometric Predicates", 1997).
//!
//! An [`Expansion`] represents a real number exactly as a sum of non-overlapping `f64`
//! components stored in increasing order of magnitude, with no zero components (zero is the
//! empty expansion). Add, subtract and multiply are exact as long as no component overflows or
//! underflows; see [`crate::predicates`] for the input range this implies.
//!
//! Everything here is plain IEEE-754 binary64 arithmetic in round-to-nearest-even, with no fused
//! multiply-add (products are split with Veltkamp/Dekker), so results are bit-identical on every
//! platform Rust targets (x86-64, aarch64). Rust never contracts or reassociates float
//! operations, which these algorithms depend on.

use core::ops::{Add, Mul, Neg, Sub};
use smallvec::SmallVec;

/// `2^27 + 1`, the Veltkamp splitter for 53-bit significands.
const SPLITTER: f64 = 134_217_729.0;

/// Exact sum: returns `(x, y)` with `x = fl(a + b)` and `a + b = x + y` exactly.
#[inline]
pub fn two_sum(a: f64, b: f64) -> (f64, f64) {
    let x = a + b;
    let bv = x - a;
    let av = x - bv;
    (x, (a - av) + (b - bv))
}

/// Exact difference: returns `(x, y)` with `x = fl(a - b)` and `a - b = x + y` exactly.
#[inline]
pub fn two_diff(a: f64, b: f64) -> (f64, f64) {
    let x = a - b;
    let bv = a - x;
    let av = x + bv;
    (x, (a - av) + (bv - b))
}

/// [`two_sum`] when `|a| >= |b|` (or `a == 0`) is known; three flops instead of six.
#[inline]
pub fn fast_two_sum(a: f64, b: f64) -> (f64, f64) {
    let x = a + b;
    (x, b - (x - a))
}

/// Veltkamp split: `a = hi + lo` exactly, each part with at most 26 significant bits.
#[inline]
fn split(a: f64) -> (f64, f64) {
    let c = SPLITTER * a;
    let big = c - a;
    let hi = c - big;
    (hi, a - hi)
}

/// Exact product: returns `(x, y)` with `x = fl(a * b)` and `a * b = x + y` exactly.
///
/// Valid when `a`, `b` are finite, `|a|, |b| < 2^996`, and the error term does not underflow.
#[inline]
pub fn two_product(a: f64, b: f64) -> (f64, f64) {
    let x = a * b;
    let (ahi, alo) = split(a);
    let (bhi, blo) = split(b);
    let e1 = x - ahi * bhi;
    let e2 = e1 - alo * bhi;
    let e3 = e2 - ahi * blo;
    (x, alo * blo - e3)
}

/// Inline capacity of an [`Expansion`] before it spills to the heap.
const INLINE: usize = 24;

/// An exact real number stored as non-overlapping `f64` components, smallest magnitude first.
///
/// The value is the sum of the components. The zero expansion has no components.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Expansion {
    c: SmallVec<[f64; INLINE]>,
}

impl Expansion {
    /// The zero expansion.
    pub fn zero() -> Self {
        Self::default()
    }

    /// The exact value of `a` (which must be finite).
    pub fn from_f64(a: f64) -> Self {
        let mut e = Self::zero();
        if a != 0.0 {
            e.c.push(a);
        }
        e
    }

    /// The exact value of `a - b`.
    pub fn from_diff(a: f64, b: f64) -> Self {
        let (x, y) = two_diff(a, b);
        Self::from_pair(y, x)
    }

    /// The exact value of `a + b`.
    pub fn from_sum(a: f64, b: f64) -> Self {
        let (x, y) = two_sum(a, b);
        Self::from_pair(y, x)
    }

    /// The exact value of `a * b`.
    pub fn from_product(a: f64, b: f64) -> Self {
        let (x, y) = two_product(a, b);
        Self::from_pair(y, x)
    }

    fn from_pair(small: f64, big: f64) -> Self {
        let mut e = Self::zero();
        if small != 0.0 {
            e.c.push(small);
        }
        if big != 0.0 {
            e.c.push(big);
        }
        e
    }

    /// The components, smallest magnitude first.
    pub fn components(&self) -> &[f64] {
        &self.c
    }

    /// True if the value is exactly zero.
    pub fn is_zero(&self) -> bool {
        self.c.is_empty()
    }

    /// Exact sign: the sign of the largest-magnitude component.
    pub fn sign(&self) -> crate::Sign {
        match self.c.last() {
            Some(&top) => crate::Sign::of(top),
            None => crate::Sign::Zero,
        }
    }

    /// Floating-point approximation of the value (components summed small to large). Has the
    /// exact sign and is zero only for the zero expansion.
    pub fn estimate(&self) -> f64 {
        self.c.iter().sum()
    }

    /// Exact `self + other`.
    pub fn add(&self, other: &Expansion) -> Expansion {
        let (e, f) = (&self.c, &other.c);
        let mut out = Expansion::zero();
        if e.is_empty() {
            out.c.extend_from_slice(f);
            return out;
        }
        if f.is_empty() {
            out.c.extend_from_slice(e);
            return out;
        }
        // Merge by increasing magnitude, then carry an accumulator through two_sum.
        let (mut i, mut j) = (0, 0);
        let mut next = || -> Option<f64> {
            let take_e = match (e.get(i), f.get(j)) {
                (Some(x), Some(y)) => x.abs() <= y.abs(),
                (Some(_), None) => true,
                (None, Some(_)) => false,
                (None, None) => return None,
            };
            if take_e {
                i += 1;
                Some(e[i - 1])
            } else {
                j += 1;
                Some(f[j - 1])
            }
        };
        let mut q = next().unwrap_or(0.0);
        while let Some(g) = next() {
            let (qn, h) = two_sum(q, g);
            if h != 0.0 {
                out.c.push(h);
            }
            q = qn;
        }
        if q != 0.0 {
            out.c.push(q);
        }
        out
    }

    /// Exact `-self`.
    pub fn negated(&self) -> Expansion {
        Expansion { c: self.c.iter().map(|&x| -x).collect() }
    }

    /// Exact `self - other`.
    pub fn sub(&self, other: &Expansion) -> Expansion {
        self.add(&other.negated())
    }

    /// Exact `self * b` for a double `b`.
    pub fn scale(&self, b: f64) -> Expansion {
        let mut out = Expansion::zero();
        if self.c.is_empty() || b == 0.0 {
            return out;
        }
        let (mut q, h) = two_product(self.c[0], b);
        if h != 0.0 {
            out.c.push(h);
        }
        for &e in &self.c[1..] {
            let (p1, p0) = two_product(e, b);
            let (s, h) = two_sum(q, p0);
            if h != 0.0 {
                out.c.push(h);
            }
            let (qn, h) = fast_two_sum(p1, s);
            if h != 0.0 {
                out.c.push(h);
            }
            q = qn;
        }
        if q != 0.0 {
            out.c.push(q);
        }
        out
    }

    /// Exact `self * other`.
    pub fn mul(&self, other: &Expansion) -> Expansion {
        let (long, short) = if self.c.len() >= other.c.len() { (self, other) } else { (other, self) };
        let mut acc = Expansion::zero();
        for &f in &short.c {
            acc = acc.add(&long.scale(f));
        }
        acc
    }

    /// Exact `self * self`.
    pub fn square(&self) -> Expansion {
        self.mul(self)
    }

    /// Rewrite as a shorter expansion with the same value (Shewchuk's `compress`).
    pub fn compress(&self) -> Expansion {
        let n = self.c.len();
        if n < 2 {
            return self.clone();
        }
        let mut g: SmallVec<[f64; INLINE]> = SmallVec::from_elem(0.0, n);
        let mut bottom = n - 1;
        let mut q = self.c[n - 1];
        for idx in (0..n - 1).rev() {
            let (qn, e) = fast_two_sum(q, self.c[idx]);
            if e != 0.0 {
                g[bottom] = qn;
                bottom -= 1;
                q = e;
            } else {
                q = qn;
            }
        }
        let mut out = Expansion::zero();
        for &gi in &g[bottom + 1..] {
            let (qn, e) = fast_two_sum(gi, q);
            q = qn;
            if e != 0.0 {
                out.c.push(e);
            }
        }
        if q != 0.0 {
            out.c.push(q);
        }
        out
    }
}

impl From<f64> for Expansion {
    fn from(a: f64) -> Self {
        Expansion::from_f64(a)
    }
}

macro_rules! binop {
    ($tr:ident, $m:ident) => {
        impl $tr<&Expansion> for &Expansion {
            type Output = Expansion;
            fn $m(self, o: &Expansion) -> Expansion {
                Expansion::$m(self, o)
            }
        }
    };
}
binop!(Add, add);
binop!(Sub, sub);
binop!(Mul, mul);

impl Neg for &Expansion {
    type Output = Expansion;
    fn neg(self) -> Expansion {
        self.negated()
    }
}

/// Exact sign of the determinant of a small square matrix of expansions (Laplace expansion
/// along the first column). Intended for dimensions up to 5.
pub fn det_exact(m: &[Vec<Expansion>]) -> Expansion {
    let n = m.len();
    match n {
        0 => Expansion::from_f64(1.0),
        1 => m[0][0].clone(),
        _ => {
            let mut acc = Expansion::zero();
            for r in 0..n {
                if m[r][0].is_zero() {
                    continue;
                }
                let minor: Vec<Vec<Expansion>> =
                    (0..n).filter(|&i| i != r).map(|i| m[i][1..].to_vec()).collect();
                let term = m[r][0].mul(&det_exact(&minor));
                acc = if r % 2 == 0 { acc.add(&term) } else { acc.sub(&term) };
            }
            acc
        }
    }
}
