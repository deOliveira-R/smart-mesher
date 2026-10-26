//! Small shared helpers.

/// Smallest power of two `>= x` (`x > 0`, finite, normal), computed exactly from the bits.
pub(crate) fn pow2_ceil(x: f64) -> f64 {
    let bits = x.to_bits();
    let exp = ((bits >> 52) & 0x7ff) as i64;
    if exp == 0 {
        return f64::MIN_POSITIVE;
    }
    let mant = bits & ((1u64 << 52) - 1);
    let e = if mant == 0 { exp } else { exp + 1 };
    f64::from_bits((e as u64) << 52)
}
