//! The three-valued [`Sign`] returned by every exact predicate.

use core::ops::{Mul, Neg};

/// Sign of an exactly evaluated quantity.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Sign {
    /// Strictly negative.
    Negative = -1,
    /// Exactly zero.
    Zero = 0,
    /// Strictly positive.
    Positive = 1,
}

impl Sign {
    /// Sign of `x`. NaN maps to [`Sign::Zero`]; callers feed finite data.
    #[inline]
    pub fn of(x: f64) -> Self {
        if x > 0.0 {
            Sign::Positive
        } else if x < 0.0 {
            Sign::Negative
        } else {
            Sign::Zero
        }
    }

    /// `-1`, `0` or `1`.
    #[inline]
    pub fn as_i32(self) -> i32 {
        self as i32
    }

    /// True for [`Sign::Zero`].
    #[inline]
    pub fn is_zero(self) -> bool {
        self == Sign::Zero
    }
}

impl Neg for Sign {
    type Output = Sign;
    #[inline]
    fn neg(self) -> Sign {
        match self {
            Sign::Negative => Sign::Positive,
            Sign::Zero => Sign::Zero,
            Sign::Positive => Sign::Negative,
        }
    }
}

impl Mul for Sign {
    type Output = Sign;
    #[inline]
    fn mul(self, o: Sign) -> Sign {
        match self.as_i32() * o.as_i32() {
            -1 => Sign::Negative,
            0 => Sign::Zero,
            _ => Sign::Positive,
        }
    }
}
