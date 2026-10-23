//! Parameter domains: intervals, periodicity (seams) and singular iso-lines (poles).

use sm_core::Point3;
use smallvec::SmallVec;

/// A closed parameter interval. Ends may be infinite (lines, planes).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Interval {
    /// Lower bound.
    pub lo: f64,
    /// Upper bound.
    pub hi: f64,
}

impl Interval {
    /// Build an interval (no ordering check; see [`Self::is_valid`]).
    pub const fn new(lo: f64, hi: f64) -> Self {
        Self { lo, hi }
    }
    /// The whole real line.
    pub const REAL: Self = Self { lo: f64::NEG_INFINITY, hi: f64::INFINITY };
    /// `hi - lo`.
    pub fn len(&self) -> f64 {
        self.hi - self.lo
    }
    /// True if `lo <= hi` and neither is NaN.
    pub fn is_valid(&self) -> bool {
        self.lo <= self.hi
    }
    /// True if `t` lies in the interval, widened by `eps`.
    pub fn contains(&self, t: f64, eps: f64) -> bool {
        t >= self.lo - eps && t <= self.hi + eps
    }
    /// Clamp `t` into the interval.
    pub fn clamp(&self, t: f64) -> f64 {
        t.max(self.lo).min(self.hi)
    }
    /// True if both ends are finite.
    pub fn is_bounded(&self) -> bool {
        self.lo.is_finite() && self.hi.is_finite()
    }
}

/// Domain of a one-parameter map: an interval plus an optional period.
///
/// If `period` is `Some(p)` then `p == range.len()` and the map satisfies `f(t) == f(t + p)`.
/// The interval ends are then the *seam*.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Domain1 {
    /// The principal interval.
    pub range: Interval,
    /// The period, if the map is periodic.
    pub period: Option<f64>,
}

impl Domain1 {
    /// A non-periodic domain.
    pub const fn open(lo: f64, hi: f64) -> Self {
        Self { range: Interval::new(lo, hi), period: None }
    }
    /// A periodic domain `[lo, lo + period]`.
    pub const fn periodic(lo: f64, period: f64) -> Self {
        Self { range: Interval::new(lo, lo + period), period: Some(period) }
    }
    /// True if periodic.
    pub fn is_periodic(&self) -> bool {
        self.period.is_some()
    }
    /// Map `t` into `[lo, hi)` if periodic; unchanged otherwise.
    pub fn wrap(&self, t: f64) -> f64 {
        match self.period {
            Some(p) => {
                let w = self.range.lo + (t - self.range.lo).rem_euclid(p);
                if w >= self.range.hi { self.range.lo } else { w }
            }
            None => t,
        }
    }
    /// The representative of `t` (mod period) closest to `reference`. Unchanged if not periodic.
    pub fn nearest_to(&self, t: f64, reference: f64) -> f64 {
        match self.period {
            Some(p) => t + p * ((reference - t) / p).round(),
            None => t,
        }
    }
}

/// A point in a surface parameter space.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Uv {
    /// First parameter.
    pub u: f64,
    /// Second parameter.
    pub v: f64,
}

impl Uv {
    /// Construct.
    pub const fn new(u: f64, v: f64) -> Self {
        Self { u, v }
    }
}

/// Which surface parameter is held constant along an iso-line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ParamAxis {
    /// The iso-line `u = const`.
    U,
    /// The iso-line `v = const`.
    V,
}

/// An iso-line that collapses to a single point (sphere pole, cone apex).
///
/// The first partial derivative across it vanishes, so the normal cannot be derived from
/// `du x dv` there; implementors return the exact limit normal instead.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Singularity {
    /// The held-constant parameter.
    pub along: ParamAxis,
    /// Its value.
    pub at: f64,
    /// The point the whole iso-line maps to.
    pub point: Point3,
}

/// Domain of a surface: two [`Domain1`] plus the singular iso-lines.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Domain2 {
    /// Domain of `u`.
    pub u: Domain1,
    /// Domain of `v`.
    pub v: Domain1,
    /// Collapsed iso-lines (poles), usually 0 to 2.
    pub singularities: SmallVec<[Singularity; 2]>,
}

impl Domain2 {
    /// A domain without singularities.
    pub fn regular(u: Domain1, v: Domain1) -> Self {
        Self { u, v, singularities: SmallVec::new() }
    }
    /// Wrap both parameters into their principal range.
    pub fn wrap(&self, uv: Uv) -> Uv {
        Uv::new(self.u.wrap(uv.u), self.v.wrap(uv.v))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::TAU;

    #[test]
    fn wrap_and_nearest() {
        let d = Domain1::periodic(0.0, TAU);
        assert!((d.wrap(-0.5) - (TAU - 0.5)).abs() < 1e-14);
        assert_eq!(d.wrap(TAU), 0.0);
        assert!((d.nearest_to(0.1, TAU - 0.1) - (TAU + 0.1)).abs() < 1e-14);
        assert_eq!(Domain1::open(0.0, 1.0).wrap(5.0), 5.0);
    }
}
