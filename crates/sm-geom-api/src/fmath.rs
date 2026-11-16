//! Transcendental helpers routed through the pure-Rust `libm` crate (pinned `=0.2.16`).
//!
//! Why (ruling C12): `f64::sin`/`cos`/... call the platform libm (Apple libm on macOS, glibc on
//! Linux), which differ by 1 ulp for some arguments. Curve and surface points built from them then
//! differ between platforms, and the exact predicates of the meshers (cocircular and cospherical
//! points) turn that into different meshes and hashes. `libm` is not correctly rounded, but it
//! returns the same bits on every platform, which is what the milestone hashes need. Bumping the
//! pinned version can move bits: `tests::pinned_bits` catches that.
//!
//! Every trig call in this crate goes through here. Other crates still use `std` trig (see
//! `docs/design/w4/int/c12.md`).

/// `(sin x, cos x)`.
pub fn sin_cos(x: f64) -> (f64, f64) {
    libm::sincos(x)
}

/// `sin x`.
pub fn sin(x: f64) -> f64 {
    libm::sin(x)
}

/// `cos x`.
pub fn cos(x: f64) -> f64 {
    libm::cos(x)
}

/// `tan x`.
#[allow(dead_code)]
pub fn tan(x: f64) -> f64 {
    libm::tan(x)
}

/// `atan x`.
#[allow(dead_code)]
pub fn atan(x: f64) -> f64 {
    libm::atan(x)
}

/// `asin x`.
pub fn asin(x: f64) -> f64 {
    libm::asin(x)
}

/// `atan2(y, x)`.
pub fn atan2(y: f64, x: f64) -> f64 {
    libm::atan2(y, x)
}

/// `sqrt(x^2 + y^2)` without undue overflow.
pub fn hypot(x: f64, y: f64) -> f64 {
    libm::hypot(x, y)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_3, PI, TAU};

    // Seam-like angles (multiples of pi/3 and pi/6, near 0 and 2 pi) and 26 and 34 degrees,
    // where the `libm` crate differs by 1 ulp from Apple's libm (so this fails if `fmath` ever
    // goes back to `std` on macOS).
    const ARGS: [f64; 8] =
        [1.0, FRAC_PI_3, 2.0 * FRAC_PI_3, 5.0 * PI / 6.0, 0.001, TAU - 1e-3, 26.0 * PI / 180.0, 34.0 * PI / 180.0];
    const SIN: [u64; 8] = [
        0x3feaed548f090cee,
        0x3febb67ae8584cab,
        0x3febb67ae8584caa,
        0x3fdfffffffffffff,
        0x3f50624da5218a62,
        0xbf50624da52194d0,
        0x3fdc0e45dabe05c8,
        0x3fe1e4e88411fd13,
    ];
    const COS: [u64; 8] = [
        0x3fe14a280fb5068c,
        0x3fdffffffffffffe,
        0xbfe0000000000002,
        0xbfebb67ae8584cab,
        0x3feffffef390876c,
        0x3feffffef390876c,
        0x3fecc2ebbb5638ca,
        0x3fea8779cda8eea4,
    ];

    #[test]
    fn pinned_bits() {
        for (i, &x) in ARGS.iter().enumerate() {
            let (s, c) = sin_cos(x);
            assert_eq!(s.to_bits(), SIN[i], "sin arg {i}");
            assert_eq!(c.to_bits(), COS[i], "cos arg {i}");
            assert_eq!(sin(x).to_bits(), s.to_bits());
            assert_eq!(cos(x).to_bits(), c.to_bits());
        }
    }

    #[test]
    fn analytic_values_within_an_ulp_or_two() {
        let ulp = f64::EPSILON;
        assert!((sin(1.0) - 0.841_470_984_807_896_5).abs() < ulp);
        assert!((cos(1.0) - 0.540_302_305_868_139_7).abs() < ulp);
        assert!((tan(1.0) - 1.557_407_724_654_902_3).abs() < 4.0 * ulp);
        assert!((atan(1.0) - PI / 4.0).abs() < ulp);
        assert!((asin(1.0) - PI / 2.0).abs() < ulp);
        assert!((atan2(1.0, -1.0) - 3.0 * PI / 4.0).abs() < 2.0 * ulp);
        assert_eq!(hypot(3.0, 4.0), 5.0);
    }
}
