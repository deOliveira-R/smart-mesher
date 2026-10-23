//! Content hashing: a self-contained SHA-256, the [`Hash`] newtype and the canonical
//! [`StateHasher`] used for node ids and state hashes.

use crate::value::Value;
use core::fmt;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// A 256-bit content hash (SHA-256). Displays and serializes as 64 lowercase hex chars.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Hash([u8; 32]);

/// Error parsing a [`Hash`] from hex.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("invalid hash: expected 64 lowercase hex characters")]
pub struct HashParseError;

impl Hash {
    /// Hash of arbitrary bytes.
    pub fn of(bytes: &[u8]) -> Self {
        let mut s = Sha256::new();
        s.update(bytes);
        Hash(s.finalize())
    }

    /// Raw digest bytes.
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }

    /// Lowercase hex form.
    pub fn to_hex(&self) -> String {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        let mut s = String::with_capacity(64);
        for b in self.0 {
            s.push(HEX[(b >> 4) as usize] as char);
            s.push(HEX[(b & 15) as usize] as char);
        }
        s
    }

    /// Parse the lowercase hex form.
    pub fn from_hex(s: &str) -> Result<Self, HashParseError> {
        let b = s.as_bytes();
        if b.len() != 64 {
            return Err(HashParseError);
        }
        let nib = |c: u8| match c {
            b'0'..=b'9' => Ok(c - b'0'),
            b'a'..=b'f' => Ok(c - b'a' + 10),
            _ => Err(HashParseError),
        };
        let mut out = [0u8; 32];
        for (i, o) in out.iter_mut().enumerate() {
            *o = (nib(b[2 * i])? << 4) | nib(b[2 * i + 1])?;
        }
        Ok(Hash(out))
    }
}

impl fmt::Display for Hash {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.to_hex())
    }
}

impl fmt::Debug for Hash {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Hash({})", &self.to_hex()[..12])
    }
}

impl Serialize for Hash {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(&self.to_hex())
    }
}

impl<'de> Deserialize<'de> for Hash {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let s = String::deserialize(d)?;
        Hash::from_hex(&s).map_err(serde::de::Error::custom)
    }
}

/// Streaming SHA-256.
#[derive(Clone)]
pub struct Sha256 {
    h: [u32; 8],
    buf: [u8; 64],
    buf_len: usize,
    total: u64,
}

const K: [u32; 64] = [
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
];

impl Default for Sha256 {
    fn default() -> Self {
        Self::new()
    }
}

impl Sha256 {
    /// Fresh hasher.
    pub fn new() -> Self {
        Sha256 {
            h: [
                0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
                0x5be0cd19,
            ],
            buf: [0; 64],
            buf_len: 0,
            total: 0,
        }
    }

    fn block(&mut self, b: &[u8; 64]) {
        let mut w = [0u32; 64];
        for i in 0..16 {
            w[i] = u32::from_be_bytes([b[4 * i], b[4 * i + 1], b[4 * i + 2], b[4 * i + 3]]);
        }
        for i in 16..64 {
            let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
            let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16]
                .wrapping_add(s0)
                .wrapping_add(w[i - 7])
                .wrapping_add(s1);
        }
        let [mut a, mut bb, mut c, mut d, mut e, mut f, mut g, mut h] = self.h;
        for i in 0..64 {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let ch = (e & f) ^ (!e & g);
            let t1 = h
                .wrapping_add(s1)
                .wrapping_add(ch)
                .wrapping_add(K[i])
                .wrapping_add(w[i]);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let maj = (a & bb) ^ (a & c) ^ (bb & c);
            let t2 = s0.wrapping_add(maj);
            h = g;
            g = f;
            f = e;
            e = d.wrapping_add(t1);
            d = c;
            c = bb;
            bb = a;
            a = t1.wrapping_add(t2);
        }
        for (s, v) in self.h.iter_mut().zip([a, bb, c, d, e, f, g, h]) {
            *s = s.wrapping_add(v);
        }
    }

    /// Feed bytes.
    pub fn update(&mut self, mut data: &[u8]) {
        self.total += data.len() as u64;
        if self.buf_len > 0 {
            let take = (64 - self.buf_len).min(data.len());
            self.buf[self.buf_len..self.buf_len + take].copy_from_slice(&data[..take]);
            self.buf_len += take;
            data = &data[take..];
            if self.buf_len == 64 {
                let b = self.buf;
                self.block(&b);
                self.buf_len = 0;
            }
        }
        while data.len() >= 64 {
            let mut b = [0u8; 64];
            b.copy_from_slice(&data[..64]);
            self.block(&b);
            data = &data[64..];
        }
        if !data.is_empty() {
            self.buf[..data.len()].copy_from_slice(data);
            self.buf_len = data.len();
        }
    }

    /// Finish and return the digest.
    pub fn finalize(mut self) -> [u8; 32] {
        let bits = self.total * 8;
        let mut pad = vec![0x80u8];
        while (self.buf_len + pad.len()) % 64 != 56 {
            pad.push(0);
        }
        pad.extend_from_slice(&bits.to_be_bytes());
        self.update(&pad);
        let mut out = [0u8; 32];
        for (i, v) in self.h.iter().enumerate() {
            out[4 * i..4 * i + 4].copy_from_slice(&v.to_be_bytes());
        }
        out
    }
}

/// Canonical, unambiguous hashing of structured data. Layers feed their content through
/// this so that state hashes are platform independent (all integers little-endian,
/// strings length-prefixed, floats by IEEE bits with one canonical NaN).
pub struct StateHasher(Sha256);

impl Default for StateHasher {
    fn default() -> Self {
        Self::new()
    }
}

impl StateHasher {
    /// Fresh hasher.
    pub fn new() -> Self {
        StateHasher(Sha256::new())
    }

    /// Feed raw bytes (caller is responsible for framing).
    pub fn bytes(&mut self, b: &[u8]) {
        self.0.update(b);
    }

    /// Feed a `u64`.
    pub fn u64(&mut self, v: u64) {
        self.0.update(&v.to_le_bytes());
    }

    /// Feed a length-prefixed string.
    pub fn str(&mut self, s: &str) {
        self.u64(s.len() as u64);
        self.0.update(s.as_bytes());
    }

    /// Feed a hash.
    pub fn hash(&mut self, h: &Hash) {
        self.0.update(h.as_bytes());
    }

    /// Feed a [`Value`] in a tagged canonical encoding.
    pub fn value(&mut self, v: &Value) {
        match v {
            Value::Bool(b) => self.bytes(&[b'b', u8::from(*b)]),
            Value::Int(i) => {
                self.bytes(b"i");
                self.bytes(&i.to_le_bytes());
            }
            Value::Float(f) => {
                let bits = if f.is_nan() {
                    0x7ff8_0000_0000_0000
                } else {
                    f.to_bits()
                };
                self.bytes(b"f");
                self.u64(bits);
            }
            Value::Str(s) => {
                self.bytes(b"s");
                self.str(s);
            }
            Value::Array(a) => {
                self.bytes(b"a");
                self.u64(a.len() as u64);
                for x in a {
                    self.value(x);
                }
            }
            Value::Table(t) => {
                self.bytes(b"t");
                self.u64(t.len() as u64);
                for (k, x) in t {
                    self.str(k);
                    self.value(x);
                }
            }
        }
    }

    /// Finish.
    pub fn finish(self) -> Hash {
        Hash(self.0.finalize())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sha256_vectors() {
        assert_eq!(
            Hash::of(b"").to_hex(),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        );
        assert_eq!(
            Hash::of(b"abc").to_hex(),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
        let long = vec![b'a'; 1000];
        let mut s = Sha256::new();
        for c in long.chunks(37) {
            s.update(c);
        }
        assert_eq!(Hash(s.finalize()), Hash::of(&long));
    }

    #[test]
    fn hex_roundtrip() {
        let h = Hash::of(b"x");
        assert_eq!(Hash::from_hex(&h.to_hex()), Ok(h));
        assert!(Hash::from_hex("zz").is_err());
    }
}
