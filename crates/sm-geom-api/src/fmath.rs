//! Transcendental helpers routed through pure-Rust `libm` (platform-independent bits).
#![allow(missing_docs)]
pub fn sin_cos(x: f64) -> (f64, f64) { libm::sincos(x) }
pub fn sin(x: f64) -> f64 { libm::sin(x) }
pub fn cos(x: f64) -> f64 { libm::cos(x) }
pub fn tan(x: f64) -> f64 { libm::tan(x) }
pub fn atan(x: f64) -> f64 { libm::atan(x) }
pub fn asin(x: f64) -> f64 { libm::asin(x) }
pub fn atan2(y: f64, x: f64) -> f64 { libm::atan2(y, x) }
pub fn hypot(x: f64, y: f64) -> f64 { libm::hypot(x, y) }
