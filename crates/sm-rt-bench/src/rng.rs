//! Seeded generators shared by every case: FNV-1a 64 of the case name seeds splitmix64.
//! Only IEEE-exact operations (`+ - * /`, `sqrt`) touch floats, so streams are bit-identical on
//! every platform.

use sha2::{Digest, Sha256};

/// FNV-1a 64 of the UTF-8 bytes of `s`.
pub fn fnv1a64(s: &str) -> u64 {
    let mut h: u64 = 0xCBF2_9CE4_8422_2325;
    for b in s.bytes() {
        h = (h ^ u64::from(b)).wrapping_mul(0x0000_0100_0000_01B3);
    }
    h
}

/// splitmix64.
#[derive(Clone, Debug)]
pub struct SplitMix64 {
    s: u64,
}

impl SplitMix64 {
    /// Stream seeded with `seed`.
    pub fn new(seed: u64) -> Self {
        Self { s: seed }
    }

    /// Stream seeded from the FNV-1a hash of a name.
    pub fn from_name(name: &str) -> Self {
        Self::new(fnv1a64(name))
    }

    /// Per-history stream: the seed mixed with the history index (no thread or time input).
    pub fn for_index(seed: u64, index: u64) -> Self {
        let mut m = Self::new(seed ^ index.wrapping_mul(0x9E37_79B9_7F4A_7C15));
        m.next_u64();
        m
    }

    /// Next 64 random bits.
    pub fn next_u64(&mut self) -> u64 {
        self.s = self.s.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.s;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)` with 53 bits.
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / 9_007_199_254_740_992.0)
    }

    /// Isotropic unit vector: rejection in the unit ball, then normalisation.
    pub fn direction(&mut self) -> [f64; 3] {
        loop {
            let x = 2.0 * self.unit() - 1.0;
            let y = 2.0 * self.unit() - 1.0;
            let z = 2.0 * self.unit() - 1.0;
            let n2 = x * x + y * y + z * z;
            if 1e-6 < n2 && n2 <= 1.0 {
                let n = n2.sqrt();
                return [x / n, y / n, z / n];
            }
        }
    }
}

/// Lower-case hex SHA-256 of `bytes`.
pub fn sha256_hex(bytes: &[u8]) -> String {
    let d = Sha256::digest(bytes);
    let mut s = String::with_capacity(64);
    for b in d {
        s.push_str(&format!("{b:02x}"));
    }
    s
}
