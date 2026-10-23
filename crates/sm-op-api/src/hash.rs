//! Content hashing: the [`Hash256`] id type and domain-separated helpers over SHA-256.

use core::fmt;
use core::str::FromStr;

use serde::de::{self, Deserialize, Deserializer};
use serde::{Serialize, Serializer};
use sha2::{Digest as _, Sha256};

use crate::codec::{CodecError, canonical_bytes, to_value};

/// A 32-byte SHA-256 digest used as a content address.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Hash256(pub [u8; 32]);

impl Hash256 {
    /// Hash a byte slice in one shot.
    pub fn of(bytes: &[u8]) -> Self {
        Self(Sha256::digest(bytes).into())
    }

    /// Hash `bytes` under a domain tag: `SHA-256(len(domain) ‖ domain ‖ bytes)`.
    pub fn of_domain(domain: &str, bytes: &[u8]) -> Self {
        let mut h = Sha256::new();
        h.update((domain.len() as u64).to_le_bytes());
        h.update(domain.as_bytes());
        h.update(bytes);
        Self(h.finalize().into())
    }

    /// Lowercase hex form (64 characters).
    pub fn to_hex(self) -> String {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        let mut s = String::with_capacity(64);
        for b in self.0 {
            s.push(HEX[(b >> 4) as usize] as char);
            s.push(HEX[(b & 15) as usize] as char);
        }
        s
    }
}

impl fmt::Display for Hash256 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.to_hex())
    }
}

impl fmt::Debug for Hash256 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Hash256({})", &self.to_hex()[..12])
    }
}

/// Error parsing a hash from hex.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("invalid hash: expected 64 lowercase hex characters")]
pub struct ParseHashError;

impl FromStr for Hash256 {
    type Err = ParseHashError;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let b = s.as_bytes();
        if b.len() != 64 {
            return Err(ParseHashError);
        }
        let nib = |c: u8| match c {
            b'0'..=b'9' => Ok(c - b'0'),
            b'a'..=b'f' => Ok(c - b'a' + 10),
            _ => Err(ParseHashError),
        };
        let mut out = [0u8; 32];
        for (i, o) in out.iter_mut().enumerate() {
            *o = (nib(b[2 * i])? << 4) | nib(b[2 * i + 1])?;
        }
        Ok(Self(out))
    }
}

impl Serialize for Hash256 {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.collect_str(self)
    }
}

impl<'de> Deserialize<'de> for Hash256 {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let s = <std::borrow::Cow<'de, str>>::deserialize(d)?;
        s.parse().map_err(de::Error::custom)
    }
}

impl schemars::JsonSchema for Hash256 {
    fn schema_name() -> std::borrow::Cow<'static, str> {
        "Hash256".into()
    }
    fn json_schema(_: &mut schemars::SchemaGenerator) -> schemars::Schema {
        schemars::json_schema!({ "type": "string", "pattern": "^[0-9a-f]{64}$", "description": "SHA-256, lowercase hex" })
    }
}

/// Hash any serializable value under `domain` through its canonical encoding
/// ([`canonical_bytes`]): independent of text formatting, key order and `-0.0`.
pub fn content_hash<T: Serialize + ?Sized>(domain: &str, v: &T) -> Result<Hash256, CodecError> {
    Ok(Hash256::of_domain(domain, &canonical_bytes(&to_value(v)?)))
}

/// Order-independent multiset accumulator: the sum mod 2^256 of `SHA-256(item)`.
/// O(1) per insert, independent of insertion order.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct MultisetHash([u64; 4]);

impl MultisetHash {
    pub(crate) fn add(&mut self, item: &[u8]) {
        let d: [u8; 32] = Sha256::digest(item).into();
        let mut carry = 0u128;
        for i in 0..4 {
            let limb = u64::from_le_bytes(d[8 * i..8 * i + 8].try_into().unwrap_or([0; 8]));
            let s = u128::from(self.0[i]) + u128::from(limb) + carry;
            self.0[i] = s as u64;
            carry = s >> 64;
        }
    }

    pub(crate) fn bytes(&self) -> [u8; 32] {
        let mut out = [0u8; 32];
        for (i, l) in self.0.iter().enumerate() {
            out[8 * i..8 * i + 8].copy_from_slice(&l.to_le_bytes());
        }
        out
    }
}

/// Streaming SHA-256 used by [`Digest`](crate::Digest) and internal hashing.
pub(crate) struct Hasher(Sha256);

impl Hasher {
    pub(crate) fn new(domain: &str) -> Self {
        let mut h = Sha256::new();
        h.update((domain.len() as u64).to_le_bytes());
        h.update(domain.as_bytes());
        Self(h)
    }
    pub(crate) fn update(&mut self, b: &[u8]) {
        self.0.update(b);
    }
    pub(crate) fn finish(self) -> Hash256 {
        Hash256(self.0.finalize().into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sha256_known_vector() {
        assert_eq!(
            Hash256::of(b"abc").to_hex(),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }

    #[test]
    fn hex_roundtrip() {
        let h = Hash256::of(b"x");
        assert_eq!(h.to_hex().parse::<Hash256>(), Ok(h));
        assert!("zz".parse::<Hash256>().is_err());
        assert!(h.to_hex().to_uppercase().parse::<Hash256>().is_err());
    }

    #[test]
    fn multiset_is_order_independent() {
        let (mut a, mut b) = (MultisetHash::default(), MultisetHash::default());
        for i in 0u64..50 {
            a.add(&i.to_le_bytes());
        }
        for i in (0u64..50).rev() {
            b.add(&i.to_le_bytes());
        }
        assert_eq!(a, b);
        b.add(&[1]);
        assert_ne!(a, b);
    }
}
