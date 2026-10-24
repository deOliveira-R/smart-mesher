//! Interval arithmetic with guaranteed enclosure and no rounding-mode changes.
//!
//! Rust has no stable access to directed rounding, and switching the FPU rounding mode is not
//! portable. Instead every operation computes the round-to-nearest result *and its exact error
//! term* (via [`crate::exact`]) which tells on which side of the computed value the true result
//! lies; the bound on that side is moved out by one ulp. This is bit-for-bit equivalent to
//! directed rounding for `+`, `-` and `*` away from overflow/underflow. Where the error term is
//! unavailable (division, `sqrt`, magnitudes outside `[2^-900, 2^900]`), both bounds are
//! conservatively widened by one ulp with [`f64::next_down`]/[`f64::next_up`], which is sound
//! because IEEE-754 `/` and `sqrt` are correctly rounded (error below one ulp).
//!
//! Invariants: `lo <= hi`, never NaN, `lo < +inf`, `hi > -inf`. Infinite bounds are allowed
//! (`[1, +inf)`, the whole line [`Interval::ENTIRE`]). Division by an interval containing
//! zero returns [`Interval::ENTIRE`] (use [`Interval::checked_div`] to get an error instead).
//! Transcendental functions are deliberately absent: `libm` results differ across platforms,
//! so they cannot be enclosed portably without a correctly rounded implementation.

use crate::Sign;
use crate::exact::{two_product, two_sum};
use core::ops::{Add, Div, Mul, Neg, Sub};

const SAFE_LO: f64 = f64::from_bits((1023 - 900) << 52);
const SAFE_HI: f64 = f64::from_bits((1023 + 900) << 52);

/// Errors from interval construction and checked operations.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum IntervalError {
    /// A bound was NaN, `lo > hi`, `lo == +inf` or `hi == -inf`.
    #[error("invalid interval bounds")]
    InvalidBounds,
    /// Division by an interval that contains zero.
    #[error("division by an interval containing zero")]
    DivisionByZero,
    /// Square root (or similar) of an interval entirely outside the function's domain.
    #[error("interval is outside the function's domain")]
    Domain,
}

/// A closed real interval `[lo, hi]` that encloses every value its computation could take.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Interval {
    lo: f64,
    hi: f64,
}

#[inline]
fn down(x: f64) -> f64 {
    if x == f64::NEG_INFINITY || x.is_nan() {
        x
    } else {
        x.next_down()
    }
}

#[inline]
fn up(x: f64) -> f64 {
    if x == f64::INFINITY || x.is_nan() {
        x
    } else {
        x.next_up()
    }
}

/// Bounds of the exact value `s + e` given the nearest-rounded `s` and exact error `e`.
#[inline]
fn pair_bounds(s: f64, e: f64) -> (f64, f64) {
    if e > 0.0 {
        (s, s.next_up())
    } else if e < 0.0 {
        (s.next_down(), s)
    } else {
        (s, s)
    }
}

/// `(lower, upper)` bounds of the real sum `a + b` for non-NaN `a`, `b` not forming `inf - inf`.
#[inline]
fn add_bounds(a: f64, b: f64) -> (f64, f64) {
    let (s, e) = two_sum(a, b);
    if a.is_infinite() || b.is_infinite() {
        (s, s)
    } else if s.is_infinite() {
        if s > 0.0 {
            (f64::MAX, s)
        } else {
            (s, -f64::MAX)
        }
    } else {
        pair_bounds(s, e)
    }
}

/// `(lower, upper)` bounds of the real product `a * b` with `0 * inf = 0`.
#[inline]
fn mul_bounds(a: f64, b: f64) -> (f64, f64) {
    if a == 0.0 || b == 0.0 {
        return (0.0, 0.0);
    }
    let p = a * b;
    if a.is_infinite() || b.is_infinite() {
        return (p, p);
    }
    if p.abs() >= SAFE_LO && p.abs() <= SAFE_HI && a.abs() < SAFE_HI && b.abs() < SAFE_HI {
        let (p, e) = two_product(a, b);
        pair_bounds(p, e)
    } else {
        (down(p), up(p))
    }
}

impl Interval {
    /// The whole real line.
    pub const ENTIRE: Interval = Interval {
        lo: f64::NEG_INFINITY,
        hi: f64::INFINITY,
    };

    /// The interval `[lo, hi]`; errors on NaN or reversed bounds.
    pub fn new(lo: f64, hi: f64) -> Result<Self, IntervalError> {
        if lo.is_nan() || hi.is_nan() || lo > hi || lo == f64::INFINITY || hi == f64::NEG_INFINITY {
            Err(IntervalError::InvalidBounds)
        } else {
            Ok(Self { lo, hi })
        }
    }

    /// The degenerate interval `[x, x]`. A NaN or infinite input gives [`Interval::ENTIRE`].
    pub fn point(x: f64) -> Self {
        if x.is_nan() || x.is_infinite() {
            Self::ENTIRE
        } else {
            Self { lo: x, hi: x }
        }
    }

    /// The smallest interval enclosing `a` and `b` (in either order). NaN gives [`Interval::ENTIRE`].
    pub fn hull_of(a: f64, b: f64) -> Self {
        if a.is_nan() || b.is_nan() {
            Self::ENTIRE
        } else {
            Self {
                lo: a.min(b),
                hi: a.max(b),
            }
        }
    }

    /// `[x - r, x + r]` (enclosing, rounded outward); `r` is taken in absolute value.
    pub fn around(x: f64, r: f64) -> Self {
        Self::point(x)
            + Self {
                lo: -r.abs(),
                hi: r.abs(),
            }
    }

    /// Lower bound.
    pub fn lo(&self) -> f64 {
        self.lo
    }

    /// Upper bound.
    pub fn hi(&self) -> f64 {
        self.hi
    }

    /// `hi - lo`, rounded up.
    pub fn width(&self) -> f64 {
        up(self.hi - self.lo)
    }

    /// A point inside the interval: the midpoint, or a finite endpoint/zero when unbounded.
    pub fn mid(&self) -> f64 {
        match (self.lo.is_finite(), self.hi.is_finite()) {
            (true, true) => self.lo * 0.5 + self.hi * 0.5,
            (true, false) => self.lo.max(0.0),
            (false, true) => self.hi.min(0.0),
            (false, false) => 0.0,
        }
    }

    /// True if `x` lies in the interval.
    pub fn contains(&self, x: f64) -> bool {
        self.lo <= x && x <= self.hi
    }

    /// True if zero lies in the interval.
    pub fn contains_zero(&self) -> bool {
        self.contains(0.0)
    }

    /// True if `self` is contained in `other`.
    pub fn is_subset_of(&self, other: &Interval) -> bool {
        other.lo <= self.lo && self.hi <= other.hi
    }

    /// True if the intervals share a point.
    pub fn overlaps(&self, other: &Interval) -> bool {
        self.lo <= other.hi && other.lo <= self.hi
    }

    /// Smallest interval containing both.
    pub fn hull(&self, other: &Interval) -> Interval {
        Interval {
            lo: self.lo.min(other.lo),
            hi: self.hi.max(other.hi),
        }
    }

    /// Intersection, or `None` if disjoint.
    pub fn intersection(&self, other: &Interval) -> Option<Interval> {
        let (lo, hi) = (self.lo.max(other.lo), self.hi.min(other.hi));
        (lo <= hi).then_some(Interval { lo, hi })
    }

    /// The sign shared by every point of the interval, or `None` if it straddles zero or
    /// touches zero without being exactly `[0, 0]`.
    pub fn sign(&self) -> Option<Sign> {
        if self.lo > 0.0 {
            Some(Sign::Positive)
        } else if self.hi < 0.0 {
            Some(Sign::Negative)
        } else if self.lo == 0.0 && self.hi == 0.0 {
            Some(Sign::Zero)
        } else {
            None
        }
    }

    /// `{|x| : x in self}`.
    pub fn abs(&self) -> Interval {
        if self.lo >= 0.0 {
            *self
        } else if self.hi <= 0.0 {
            -*self
        } else {
            Interval {
                lo: 0.0,
                hi: (-self.lo).max(self.hi),
            }
        }
    }

    /// `{x * x}`; tighter than `self * self` and never negative.
    pub fn sqr(&self) -> Interval {
        let a = self.abs();
        let lo = if self.contains_zero() {
            0.0
        } else {
            mul_bounds(a.lo, a.lo).0
        };
        Interval {
            lo,
            hi: mul_bounds(a.hi, a.hi).1,
        }
    }

    /// `{x^n}` for integer `n >= 0`, by repeated squaring (even powers use [`Interval::sqr`], so
    /// they are never negative).
    pub fn powi(&self, n: u32) -> Interval {
        match n {
            0 => Interval::point(1.0),
            1 => *self,
            _ if n.is_multiple_of(2) => self.powi(n / 2).sqr(),
            _ => self.powi(n - 1) * *self,
        }
    }

    /// Square root of the non-negative part of the interval; `Err(Domain)` if entirely negative.
    pub fn sqrt(&self) -> Result<Interval, IntervalError> {
        if self.hi < 0.0 {
            return Err(IntervalError::Domain);
        }
        let lo = if self.lo <= 0.0 {
            0.0
        } else {
            down(self.lo.sqrt()).max(0.0)
        };
        Ok(Interval {
            lo,
            hi: up(self.hi.sqrt()),
        })
    }

    /// Division that reports `Err(DivisionByZero)` if `other` contains zero.
    pub fn checked_div(&self, other: &Interval) -> Result<Interval, IntervalError> {
        if other.contains_zero() {
            return Err(IntervalError::DivisionByZero);
        }
        let q = [
            self.lo / other.lo,
            self.lo / other.hi,
            self.hi / other.lo,
            self.hi / other.hi,
        ];
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for v in q {
            if v.is_nan() {
                return Ok(Interval::ENTIRE); // inf / inf
            }
            lo = lo.min(v);
            hi = hi.max(v);
        }
        Ok(Interval {
            lo: down(lo),
            hi: up(hi),
        })
    }

    /// Elementwise minimum of two intervals.
    pub fn min(&self, other: &Interval) -> Interval {
        Interval {
            lo: self.lo.min(other.lo),
            hi: self.hi.min(other.hi),
        }
    }

    /// Elementwise maximum of two intervals.
    pub fn max(&self, other: &Interval) -> Interval {
        Interval {
            lo: self.lo.max(other.lo),
            hi: self.hi.max(other.hi),
        }
    }
}

impl Neg for Interval {
    type Output = Interval;
    fn neg(self) -> Interval {
        Interval {
            lo: -self.hi,
            hi: -self.lo,
        }
    }
}

impl Add for Interval {
    type Output = Interval;
    fn add(self, o: Interval) -> Interval {
        Interval {
            lo: add_bounds(self.lo, o.lo).0,
            hi: add_bounds(self.hi, o.hi).1,
        }
    }
}

impl Sub for Interval {
    type Output = Interval;
    fn sub(self, o: Interval) -> Interval {
        self + (-o)
    }
}

impl Mul for Interval {
    type Output = Interval;
    fn mul(self, o: Interval) -> Interval {
        let c = [
            mul_bounds(self.lo, o.lo),
            mul_bounds(self.lo, o.hi),
            mul_bounds(self.hi, o.lo),
            mul_bounds(self.hi, o.hi),
        ];
        let lo = c.iter().map(|b| b.0).fold(f64::INFINITY, f64::min);
        let hi = c.iter().map(|b| b.1).fold(f64::NEG_INFINITY, f64::max);
        Interval { lo, hi }
    }
}

impl Div for Interval {
    type Output = Interval;
    /// Division; an interval containing zero in the divisor yields [`Interval::ENTIRE`].
    fn div(self, o: Interval) -> Interval {
        self.checked_div(&o).unwrap_or(Interval::ENTIRE)
    }
}

impl Add<f64> for Interval {
    type Output = Interval;
    fn add(self, o: f64) -> Interval {
        self + Interval::point(o)
    }
}

impl Sub<f64> for Interval {
    type Output = Interval;
    fn sub(self, o: f64) -> Interval {
        self - Interval::point(o)
    }
}

impl Mul<f64> for Interval {
    type Output = Interval;
    fn mul(self, o: f64) -> Interval {
        self * Interval::point(o)
    }
}

impl From<f64> for Interval {
    fn from(x: f64) -> Self {
        Interval::point(x)
    }
}
