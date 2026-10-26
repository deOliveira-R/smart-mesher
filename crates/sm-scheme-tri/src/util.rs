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

/// True for a strictly positive number (false for NaN): the readable form of `!(x > 0.0)`'s
/// complement.
#[inline]
pub(crate) fn is_pos(x: f64) -> bool {
    x > 0.0
}
