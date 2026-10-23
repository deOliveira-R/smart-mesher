//! Intervals and the tolerance policy.

/// A closed parameter interval `[lo, hi]`. Bounds may be infinite (lines, planes).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Interval {
    /// Lower bound.
    pub lo: f64,
    /// Upper bound.
    pub hi: f64,
}

impl Interval {
    /// The whole real line.
    pub const FULL: Self = Self { lo: f64::NEG_INFINITY, hi: f64::INFINITY };

    /// Construct an interval (no ordering check; see [`Interval::is_valid`]).
    pub const fn new(lo: f64, hi: f64) -> Self {
        Self { lo, hi }
    }

    /// True if `lo <= hi` and neither bound is NaN.
    pub fn is_valid(&self) -> bool {
        self.lo <= self.hi
    }

    /// True if both bounds are finite.
    pub fn is_finite(&self) -> bool {
        self.lo.is_finite() && self.hi.is_finite()
    }

    /// `hi - lo`.
    pub fn length(&self) -> f64 {
        self.hi - self.lo
    }

    /// Midpoint (meaningful only for finite intervals).
    pub fn mid(&self) -> f64 {
        0.5 * (self.lo + self.hi)
    }

    /// Membership with slack `eps` on both ends.
    pub fn contains(&self, t: f64, eps: f64) -> bool {
        t >= self.lo - eps && t <= self.hi + eps
    }

    /// Clamp `t` into the interval.
    pub fn clamp(&self, t: f64) -> f64 {
        t.max(self.lo).min(self.hi)
    }

    /// Intersection, or `None` if empty.
    pub fn intersect(&self, o: &Self) -> Option<Self> {
        let r = Self { lo: self.lo.max(o.lo), hi: self.hi.min(o.hi) };
        r.is_valid().then_some(r)
    }
}

/// Model-wide tolerance policy: the floor and ceiling for per-entity tolerances (tolerant
/// modeling, OCCT style) plus the angular and parametric resolutions.
///
/// Per-entity tolerances live on topology entities; this type only bounds them and defines
/// the "same place" predicate used by imprint/merge and by tracking.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TolerancePolicy {
    /// Smallest meaningful distance (resolution). Every entity tolerance is at least this.
    pub linear_min: f64,
    /// Largest tolerance an entity may grow to; beyond this, healing/rejection is required.
    pub linear_max: f64,
    /// Angular resolution in radians (parallel/tangent decisions).
    pub angular: f64,
    /// Floor for parametric tolerances derived from `linear_min / speed`.
    pub parametric_min: f64,
}

impl Default for TolerancePolicy {
    fn default() -> Self {
        Self { linear_min: 1e-7, linear_max: 1e-3, angular: 1e-9, parametric_min: 1e-12 }
    }
}

impl TolerancePolicy {
    /// True if the policy is internally consistent (positive, `min <= max`).
    pub fn is_valid(&self) -> bool {
        self.linear_min > 0.0
            && self.linear_min <= self.linear_max
            && self.angular > 0.0
            && self.parametric_min > 0.0
            && self.linear_max.is_finite()
    }

    /// Clamp an entity tolerance into `[linear_min, linear_max]`.
    pub fn clamp_linear(&self, tol: f64) -> f64 {
        tol.max(self.linear_min).min(self.linear_max)
    }

    /// Two entities with tolerances `ta`, `tb` whose gap is `dist` are "the same place"
    /// when `dist <= ta + tb` (OCCT rule: tolerance balls overlap).
    pub fn coincident(&self, dist: f64, ta: f64, tb: f64) -> bool {
        dist <= ta + tb
    }

    /// Tolerance an entity must grow to so that it also covers a gap `dist` to another
    /// entity of tolerance `other`: `max(own, dist + other)`, capped at `linear_max`
    /// by the caller (see [`TolerancePolicy::within_cap`]).
    pub fn grown(&self, own: f64, dist: f64, other: f64) -> f64 {
        own.max(dist + other)
    }

    /// True if `tol` does not exceed `linear_max`.
    pub fn within_cap(&self, tol: f64) -> bool {
        tol <= self.linear_max
    }

    /// Parametric tolerance for a curve with parametric speed `speed = |dC/dt|`.
    pub fn param_tol(&self, speed: f64) -> f64 {
        (self.linear_min / speed.abs().max(1e-300)).max(self.parametric_min)
    }
}
