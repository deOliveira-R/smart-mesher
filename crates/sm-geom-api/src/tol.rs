//! Tolerance policy and the cross-platform hash quantization rule.

use sm_core::Point3;

/// The tolerance policy of a model: one global value set plus (in topology) sparse per-entity
/// linear overrides.
///
/// All geometric "is it the same?" decisions use these numbers, never ad-hoc epsilons.
///
/// # Hash quantization rule (binding, cross-platform)
/// Content hashes (`sm-topo-api` `canonical_hash`, the mesh `geometry_hash`) never hash raw float
/// bits. A length-valued float `x` is replaced by the integer `round(x / q)` with
/// `q = linear * 2^-10` ([`Self::quantum`]); a parameter-valued float (curve parameters, pcurve
/// coordinates) uses `q = parametric * 2^-10` ([`Self::param_quantum`]). Rounding is
/// round-half-away-from-zero, `-0` and `+0` agree, NaN maps to `i64::MIN` and infinities
/// saturate ([`Self::quantize`]). Scale-free values (knots, weights, quadric coefficients, line
/// direction components) are quantized relatively instead ([`Self::quantize_scale_free`]): their
/// magnitude is unbounded (quadric terms scale like L^2), so a fixed absolute step would
/// saturate. Angles and unit-vector components (frame axes, cone half-angle) use
/// `q = angular * 2^-10` ([`Self::angular_quantum`]). Shape
/// definitions are hashed value by value through [`crate::CurveDef::for_each_value`] and
/// [`Self::quantize_shape`]. Two values within a small fraction of a step hash equal unless
/// they straddle a rounding boundary; platform noise (a few ulp) is far below one step for any
/// sane model, so the residual risk is a value that sits within noise of a half-step boundary.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct ModelTolerance {
    /// Model resolution: two points closer than this are the same point (model length units).
    pub linear: f64,
    /// Two unit directions closer than this angle (radians) are parallel.
    pub angular: f64,
    /// Resolution in parameter space for periodic wrap and seam matching.
    pub parametric: f64,
    /// Hard cap on any per-entity tolerance (and on the sum used by a coincidence test).
    pub linear_max: f64,
}

/// `2^-10`, the hash quantization fraction.
const QUANT_FRACTION: f64 = 1.0 / 1024.0;

/// `2^-40`, the relative step of [`ModelTolerance::quantize_scale_free`].
const SCALE_FREE_STEP: f64 = 1.0 / 1_099_511_627_776.0;

impl ModelTolerance {
    /// Default policy: 1e-6 length, 1e-9 rad, 1e-9 parameter, cap 1e-3 (a unit-size model).
    pub const DEFAULT: Self = Self { linear: 1e-6, angular: 1e-9, parametric: 1e-9, linear_max: 1e-3 };

    /// Policy scaled to a model of characteristic size `extent`: `linear = 1e-6 extent`,
    /// `linear_max = 1e-3 extent`. A non-positive or non-finite `extent` gives [`Self::DEFAULT`].
    pub fn for_extent(extent: f64) -> Self {
        if !(extent.is_finite() && extent > 0.0) {
            return Self::DEFAULT;
        }
        Self { linear: extent * 1e-6, angular: 1e-9, parametric: 1e-9, linear_max: extent * 1e-3 }
    }

    /// Coincidence test for two entities with tolerances `ta`, `tb` at distance `d`:
    /// `d <= ta + tb` and `ta + tb <= linear_max`.
    pub fn coincident(&self, d: f64, ta: f64, tb: f64) -> bool {
        let sum = ta + tb;
        d <= sum && sum <= self.linear_max
    }

    /// True if the two points coincide within [`Self::linear`] (global tolerance only).
    pub fn points_coincide(&self, a: Point3, b: Point3) -> bool {
        (a - b).norm() <= self.linear
    }

    /// True if `len` is below the linear resolution.
    pub fn is_zero_length(&self, len: f64) -> bool {
        len.abs() <= self.linear
    }

    /// True if all components are finite and strictly positive and `linear <= linear_max`.
    pub fn is_valid(&self) -> bool {
        [self.linear, self.angular, self.parametric, self.linear_max].iter().all(|v| v.is_finite() && *v > 0.0) && self.linear <= self.linear_max
    }

    /// Hash step for length-valued floats: `linear * 2^-10`.
    pub fn quantum(&self) -> f64 {
        self.linear * QUANT_FRACTION
    }

    /// Hash step for parameter-valued floats: `parametric * 2^-10`.
    pub fn param_quantum(&self) -> f64 {
        self.parametric * QUANT_FRACTION
    }

    /// Hash step for angles and direction components: `angular * 2^-10`.
    pub fn angular_quantum(&self) -> f64 {
        self.angular * QUANT_FRACTION
    }

    /// Quantize one value of a shape definition according to its class.
    pub fn quantize_shape(&self, v: crate::ShapeValue) -> i64 {
        use crate::ShapeValue as S;
        match v {
            S::Int(n) => n as i64,
            S::Length(x) => self.quantize(x),
            S::Angle(x) => Self::quantize_with(x, self.angular_quantum()),
            S::Unitless(x) => self.quantize_scale_free(x),
        }
    }

    /// Quantize one value of a shape definition whose largest scale-free magnitude is `scale`
    /// (see [`Self::quantize_scale_free_in`]). Content hashes use this through
    /// [`crate::CurveDef::quantized_values`] / [`crate::SurfaceDef::quantized_values`].
    pub fn quantize_shape_in(&self, v: crate::ShapeValue, scale: f64) -> i64 {
        match v {
            crate::ShapeValue::Unitless(x) => self.quantize_scale_free_in(x, scale),
            other => self.quantize_shape(other),
        }
    }

    /// [`Self::quantize_scale_free`] with the zero cutoff relative to `scale`, the largest
    /// scale-free magnitude of the same definition: `|x| < max(parametric * 2^-10, scale * 2^-40)`
    /// maps to 0. `2^-40` is the relative step of the mantissa rounding, so a value below one
    /// step of the definition's largest value (cancellation noise after a large translation, e.g.
    /// a cross term of a translated quadric) hashes as zero. A non-finite `scale` is ignored.
    pub fn quantize_scale_free_in(&self, x: f64, scale: f64) -> i64 {
        let rel = if scale.is_finite() { scale.abs() * SCALE_FREE_STEP } else { 0.0 };
        if x.is_finite() && x.abs() < rel {
            return 0;
        }
        self.quantize_scale_free(x)
    }

    /// Quantize `x` with step `q` (see the type-level rule).
    pub fn quantize_with(x: f64, q: f64) -> i64 {
        if x.is_nan() || q.is_nan() || q <= 0.0 {
            return i64::MIN;
        }
        // `as` saturates for out-of-range and infinite values.
        let r = (x / q).round();
        if r == 0.0 { 0 } else { r as i64 }
    }

    /// Quantize a scale-free value (knots, weights, quadric coefficients, line direction).
    ///
    /// Relative rule: values with `|x| < parametric * 2^-10` map to 0; otherwise the mantissa is
    /// rounded to 40 bits (relative step about 1e-12) and packed with the biased exponent and
    /// sign into an `i64`. The result never saturates for finite input. A few ulp of noise is far
    /// below one step, including just below a power of two (the rounding carry moves into the
    /// exponent). The zero cutoff here is absolute; content hashes use
    /// [`Self::quantize_scale_free_in`], which also zeroes values below one relative step of the
    /// largest scale-free value of the same definition. NaN maps to `i64::MIN`, `+-inf` to `+-i64::MAX`.
    pub fn quantize_scale_free(&self, x: f64) -> i64 {
        if x.is_nan() {
            return i64::MIN;
        }
        if x.is_infinite() {
            return if x > 0.0 { i64::MAX } else { -i64::MAX };
        }
        let a = x.abs();
        let exp_field = a.to_bits() >> 52;
        if a < self.param_quantum() || exp_field == 0 {
            return 0;
        }
        // Exponent and mantissa are contiguous in the bit pattern: dropping the low 13 bits with
        // round-half-up lets a mantissa carry increment the exponent, so a power of two and the
        // value one ulp below it share a code. Monotone; cannot overflow for finite input.
        let code = ((a.to_bits() + (1u64 << 12)) >> 13) as i64;
        if x < 0.0 { -code } else { code }
    }

    /// Quantize a length-valued float.
    pub fn quantize(&self, x: f64) -> i64 {
        Self::quantize_with(x, self.quantum())
    }

    /// Quantize a parameter-valued float.
    pub fn quantize_param(&self, x: f64) -> i64 {
        Self::quantize_with(x, self.param_quantum())
    }

    /// A copy with the linear tolerance replaced.
    pub fn with_linear(self, linear: f64) -> Self {
        Self { linear, ..self }
    }
}

impl Default for ModelTolerance {
    fn default() -> Self {
        Self::DEFAULT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coincidence_and_validity() {
        let t = ModelTolerance::default();
        assert!(t.is_valid());
        assert!(t.points_coincide(Point3::new(0.0, 0.0, 0.0), Point3::new(1e-7, 0.0, 0.0)));
        assert!(!t.points_coincide(Point3::new(0.0, 0.0, 0.0), Point3::new(1e-5, 0.0, 0.0)));
        assert!(t.coincident(1.5e-6, 1e-6, 1e-6));
        assert!(!t.coincident(3e-6, 1e-6, 1e-6));
        assert!(!t.coincident(1e-6, 1e-3, 1e-3), "sum above linear_max is never coincident");
        assert_eq!(ModelTolerance::for_extent(1.0), ModelTolerance::DEFAULT);
        assert_eq!(ModelTolerance::for_extent(-3.0), ModelTolerance::DEFAULT);
    }

    #[test]
    fn quantization_rule() {
        let t = ModelTolerance::DEFAULT;
        let q = t.quantum();
        assert_eq!(q, 1e-6 / 1024.0);
        assert_eq!(t.quantize(0.0), 0);
        assert_eq!(t.quantize(-0.0), 0);
        assert_eq!(t.quantize(0.4 * q), 0);
        assert_eq!(t.quantize(0.6 * q), 1);
        assert_eq!(t.quantize(-0.6 * q), -1);
        assert_eq!(t.quantize(1.0), (1.0 / q).round() as i64);
        assert_eq!(t.quantize(f64::NAN), i64::MIN);
        assert_eq!(t.quantize(f64::INFINITY), i64::MAX);
        assert_eq!(t.quantize(f64::NEG_INFINITY), i64::MIN);
        assert_eq!(t.quantize_param(1.0), (1.0f64 / (1e-9 / 1024.0)).round() as i64);
    }

    #[test]
    fn scale_free_stable_below_powers_of_two() {
        for t in [ModelTolerance::DEFAULT, ModelTolerance::for_extent(1000.0)] {
            for v in [0.5f64, 1.0, 2.0, 1024.0, 1.6e7] {
                let below = f64::from_bits(v.to_bits() - 1);
                let above = f64::from_bits(v.to_bits() + 1);
                assert_eq!(t.quantize_scale_free(v), t.quantize_scale_free(below), "{v}");
                assert_eq!(t.quantize_scale_free(v), t.quantize_scale_free(above), "{v}");
                assert_eq!(t.quantize_scale_free(-v), t.quantize_scale_free(-below), "{v}");
            }
            assert!(t.quantize_scale_free(1.0) < t.quantize_scale_free(1.0 + 1e-9));
        }
    }

    #[test]
    fn scale_free_cutoff_is_relative_to_the_definition() {
        let t = ModelTolerance::DEFAULT;
        // Cancellation noise next to a large coefficient hashes as zero ...
        let big = 4.0e12;
        let noise = big * f64::EPSILON * 8.0;
        assert_ne!(t.quantize_scale_free(noise), 0);
        assert_eq!(t.quantize_scale_free_in(noise, big), 0);
        assert_eq!(t.quantize_scale_free_in(-noise, big), 0);
        // ... while values above one relative step keep their code.
        assert_eq!(t.quantize_scale_free_in(10.0, big), t.quantize_scale_free(10.0));
        assert_eq!(t.quantize_scale_free_in(1.0, big), 0, "below one relative step of 4e12");
        assert_eq!(t.quantize_scale_free_in(big, big), t.quantize_scale_free(big));
        assert_eq!(t.quantize_scale_free_in(1e-3, 1.0), t.quantize_scale_free(1e-3));
        assert_eq!(t.quantize_scale_free_in(1e-3, f64::INFINITY), t.quantize_scale_free(1e-3));
    }

    #[test]
    fn scale_free_does_not_saturate() {
        for t in [ModelTolerance::DEFAULT, ModelTolerance::for_extent(1000.0)] {
            let a = t.quantize_scale_free(1.6e7 - 1.0);
            let b = t.quantize_scale_free(1.6e7 - 100.0);
            assert_ne!(a, b);
            assert_ne!(a, i64::MAX);
            assert_eq!(t.quantize_scale_free(-8000.0), -t.quantize_scale_free(8000.0));
            assert_eq!(t.quantize_scale_free(1e9), t.quantize_scale_free(1e9 * (1.0 + 4.0 * f64::EPSILON)));
            assert_ne!(t.quantize_scale_free(1e9), t.quantize_scale_free(1e9 * (1.0 + 1e-10)));
            assert_eq!(t.quantize_scale_free(1e-15), 0);
            assert_eq!(t.quantize_scale_free(-0.0), 0);
            assert_eq!(t.quantize_scale_free(f64::NAN), i64::MIN);
            assert_eq!(t.quantize_scale_free(f64::INFINITY), i64::MAX);
            assert_eq!(t.quantize_scale_free(f64::MAX), t.quantize_scale_free(f64::MAX));
        }
    }
}
