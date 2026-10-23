//! Tiny deterministic hasher (FNV-1a, 64 bit) used for content fingerprints and determinism tests.
//! Floats are quantized first so that fingerprints do not depend on the last bits.

use sm_core::{EntityId, Transform};

/// FNV-1a 64-bit streaming hasher.
#[derive(Clone, Debug)]
pub struct Fnv64(u64);

impl Default for Fnv64 {
    fn default() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
}

impl Fnv64 {
    /// A fresh hasher.
    pub fn new() -> Self {
        Self::default()
    }
    /// Feed raw bytes.
    pub fn bytes(&mut self, b: &[u8]) {
        for &x in b {
            self.0 ^= u64::from(x);
            self.0 = self.0.wrapping_mul(0x0100_0000_01b3);
        }
    }
    /// Feed a `u64`.
    pub fn u64(&mut self, v: u64) {
        self.bytes(&v.to_le_bytes());
    }
    /// Feed an entity id.
    pub fn id(&mut self, id: EntityId) {
        self.u64(id.to_raw());
    }
    /// Feed a string (length-prefixed).
    pub fn str(&mut self, s: &str) {
        self.u64(s.len() as u64);
        self.bytes(s.as_bytes());
    }
    /// Feed a float quantized to 1e-9.
    pub fn f64(&mut self, x: f64) {
        // Saturating cast; `+ 0.0` folds -0.0 into 0.0 before rounding.
        let q = ((x + 0.0) * 1e9).round() as i64;
        self.bytes(&q.to_le_bytes());
    }
    /// Feed a transform (quantized).
    pub fn transform(&mut self, t: &Transform) {
        for r in &t.m {
            for &x in r {
                self.f64(x);
            }
        }
        self.f64(t.t.x);
        self.f64(t.t.y);
        self.f64(t.t.z);
    }
    /// The digest.
    pub fn finish(&self) -> u64 {
        self.0
    }
}
