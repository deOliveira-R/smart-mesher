//! Content hashing: an in-crate SHA-256 and a canonical serde byte encoding.
//!
//! The hash of a value is `SHA-256(domain || canonical_bytes(value))`. The canonical encoding is
//! owned by this crate (it does not depend on TOML/JSON text formatting or on `std` hashers), so a
//! hash can always be recomputed from a parsed file and never changes between Rust versions.

use core::fmt;
use core::str::FromStr;

use serde::ser::{self, Serialize};

/// A 256-bit content hash, displayed and serialized as 64 lowercase hex characters.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Hash256(pub [u8; 32]);

impl Hash256 {
    /// Hex form of the hash (64 characters).
    pub fn to_hex(&self) -> String {
        let mut s = String::with_capacity(64);
        for b in self.0 {
            s.push(char::from_digit(u32::from(b >> 4), 16).unwrap_or('0'));
            s.push(char::from_digit(u32::from(b & 15), 16).unwrap_or('0'));
        }
        s
    }

    /// First 12 hex characters, for display only.
    pub fn short(&self) -> String {
        self.to_hex()[..12].to_string()
    }
}

impl fmt::Display for Hash256 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.to_hex())
    }
}

impl fmt::Debug for Hash256 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Hash256({})", self.short())
    }
}

/// Error parsing a [`Hash256`] from hex.
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
        let mut out = [0u8; 32];
        for (i, pair) in b.chunks(2).enumerate() {
            let hi = hex_val(pair[0]).ok_or(ParseHashError)?;
            let lo = hex_val(pair[1]).ok_or(ParseHashError)?;
            out[i] = (hi << 4) | lo;
        }
        Ok(Self(out))
    }
}

fn hex_val(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        _ => None,
    }
}

impl Serialize for Hash256 {
    fn serialize<S: ser::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(&self.to_hex())
    }
}

impl<'de> serde::Deserialize<'de> for Hash256 {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let s = String::deserialize(d)?;
        s.parse().map_err(serde::de::Error::custom)
    }
}

impl schemars::JsonSchema for Hash256 {
    fn schema_name() -> std::borrow::Cow<'static, str> {
        "Hash256".into()
    }
    fn json_schema(_: &mut schemars::SchemaGenerator) -> schemars::Schema {
        schemars::json_schema!({ "type": "string", "pattern": "^[0-9a-f]{64}$" })
    }
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

/// SHA-256 of `data` (FIPS 180-4).
pub fn sha256(data: &[u8]) -> Hash256 {
    let mut h: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];
    let mut msg = data.to_vec();
    msg.push(0x80);
    while msg.len() % 64 != 56 {
        msg.push(0);
    }
    msg.extend_from_slice(&((data.len() as u64).wrapping_mul(8)).to_be_bytes());
    for chunk in msg.as_chunks::<64>().0 {
        let mut w = [0u32; 64];
        let (words, _) = chunk.as_slice().as_chunks::<4>();
        for (slot, word) in w.iter_mut().zip(words) {
            *slot = u32::from_be_bytes(*word);
        }
        for i in 16..64 {
            let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
            let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16]
                .wrapping_add(s0)
                .wrapping_add(w[i - 7])
                .wrapping_add(s1);
        }
        let mut v = h;
        for i in 0..64 {
            let s1 = v[4].rotate_right(6) ^ v[4].rotate_right(11) ^ v[4].rotate_right(25);
            let ch = (v[4] & v[5]) ^ (!v[4] & v[6]);
            let t1 = v[7]
                .wrapping_add(s1)
                .wrapping_add(ch)
                .wrapping_add(K[i])
                .wrapping_add(w[i]);
            let s0 = v[0].rotate_right(2) ^ v[0].rotate_right(13) ^ v[0].rotate_right(22);
            let maj = (v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]);
            let t2 = s0.wrapping_add(maj);
            v = [
                t1.wrapping_add(t2),
                v[0],
                v[1],
                v[2],
                v[3].wrapping_add(t1),
                v[4],
                v[5],
                v[6],
            ];
        }
        for i in 0..8 {
            h[i] = h[i].wrapping_add(v[i]);
        }
    }
    let mut out = [0u8; 32];
    for (i, word) in h.iter().enumerate() {
        out[i * 4..i * 4 + 4].copy_from_slice(&word.to_be_bytes());
    }
    Hash256(out)
}

/// Error from canonical encoding.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum CanonError {
    /// NaN and infinities have no canonical form.
    #[error("non-finite float cannot be canonically encoded")]
    NonFinite,
    /// 128-bit integers are not supported.
    #[error("128-bit integers are not supported")]
    Int128,
    /// A custom serialize error.
    #[error("{0}")]
    Custom(String),
}

impl ser::Error for CanonError {
    fn custom<T: fmt::Display>(msg: T) -> Self {
        CanonError::Custom(msg.to_string())
    }
}

/// Canonical, self-describing byte encoding of any `Serialize` value.
///
/// Every value is prefixed by a type tag; strings and byte blobs by a little-endian `u64` length;
/// floats are encoded as `f64::to_bits` (with `-0.0` normalized to `+0.0`); struct fields and map
/// entries are written in serialization order, so maps must be ordered (`BTreeMap`, or an
/// `IndexMap` built deterministically).
pub fn canonical_bytes<T: Serialize + ?Sized>(value: &T) -> Result<Vec<u8>, CanonError> {
    let mut enc = Enc { out: Vec::new() };
    value.serialize(&mut enc)?;
    Ok(enc.out)
}

/// `SHA-256(domain-length || domain || canonical_bytes(value))`: a domain-separated content hash.
pub fn content_hash<T: Serialize + ?Sized>(domain: &str, value: &T) -> Result<Hash256, CanonError> {
    let mut buf = Vec::new();
    buf.extend_from_slice(&(domain.len() as u64).to_le_bytes());
    buf.extend_from_slice(domain.as_bytes());
    buf.extend_from_slice(&canonical_bytes(value)?);
    Ok(sha256(&buf))
}

struct Enc {
    out: Vec<u8>,
}

mod tag {
    pub const UNIT: u8 = 0;
    pub const FALSE: u8 = 1;
    pub const TRUE: u8 = 2;
    pub const INT: u8 = 3;
    pub const UINT: u8 = 4;
    pub const FLOAT: u8 = 5;
    pub const STR: u8 = 6;
    pub const BYTES: u8 = 7;
    pub const NONE: u8 = 8;
    pub const SOME: u8 = 9;
    pub const SEQ: u8 = 10;
    pub const STRUCT: u8 = 11;
    pub const MAP: u8 = 12;
    pub const UNIT_VARIANT: u8 = 13;
    pub const NEWTYPE_VARIANT: u8 = 14;
    pub const TUPLE_VARIANT: u8 = 15;
    pub const STRUCT_VARIANT: u8 = 16;
    pub const END: u8 = 255;
}

impl Enc {
    fn str(&mut self, t: u8, s: &[u8]) {
        self.out.push(t);
        self.out.extend_from_slice(&(s.len() as u64).to_le_bytes());
        self.out.extend_from_slice(s);
    }
    fn float(&mut self, v: f64) -> Result<(), CanonError> {
        if !v.is_finite() {
            return Err(CanonError::NonFinite);
        }
        let v = if v == 0.0 { 0.0 } else { v };
        self.out.push(tag::FLOAT);
        self.out.extend_from_slice(&v.to_bits().to_le_bytes());
        Ok(())
    }
    fn end(&mut self) -> Result<(), CanonError> {
        self.out.push(tag::END);
        Ok(())
    }
}

impl ser::Serializer for &mut Enc {
    type Ok = ();
    type Error = CanonError;
    type SerializeSeq = Self;
    type SerializeTuple = Self;
    type SerializeTupleStruct = Self;
    type SerializeTupleVariant = Self;
    type SerializeMap = Self;
    type SerializeStruct = Self;
    type SerializeStructVariant = Self;

    fn serialize_bool(self, v: bool) -> Result<(), CanonError> {
        self.out.push(if v { tag::TRUE } else { tag::FALSE });
        Ok(())
    }
    fn serialize_i8(self, v: i8) -> Result<(), CanonError> {
        self.serialize_i64(i64::from(v))
    }
    fn serialize_i16(self, v: i16) -> Result<(), CanonError> {
        self.serialize_i64(i64::from(v))
    }
    fn serialize_i32(self, v: i32) -> Result<(), CanonError> {
        self.serialize_i64(i64::from(v))
    }
    fn serialize_i64(self, v: i64) -> Result<(), CanonError> {
        self.out.push(tag::INT);
        self.out.extend_from_slice(&v.to_le_bytes());
        Ok(())
    }
    fn serialize_u8(self, v: u8) -> Result<(), CanonError> {
        self.serialize_u64(u64::from(v))
    }
    fn serialize_u16(self, v: u16) -> Result<(), CanonError> {
        self.serialize_u64(u64::from(v))
    }
    fn serialize_u32(self, v: u32) -> Result<(), CanonError> {
        self.serialize_u64(u64::from(v))
    }
    fn serialize_u64(self, v: u64) -> Result<(), CanonError> {
        self.out.push(tag::UINT);
        self.out.extend_from_slice(&v.to_le_bytes());
        Ok(())
    }
    fn serialize_i128(self, _: i128) -> Result<(), CanonError> {
        Err(CanonError::Int128)
    }
    fn serialize_u128(self, _: u128) -> Result<(), CanonError> {
        Err(CanonError::Int128)
    }
    fn serialize_f32(self, v: f32) -> Result<(), CanonError> {
        self.float(f64::from(v))
    }
    fn serialize_f64(self, v: f64) -> Result<(), CanonError> {
        self.float(v)
    }
    fn serialize_char(self, v: char) -> Result<(), CanonError> {
        let mut b = [0u8; 4];
        self.str(tag::STR, v.encode_utf8(&mut b).as_bytes());
        Ok(())
    }
    fn serialize_str(self, v: &str) -> Result<(), CanonError> {
        self.str(tag::STR, v.as_bytes());
        Ok(())
    }
    fn serialize_bytes(self, v: &[u8]) -> Result<(), CanonError> {
        self.str(tag::BYTES, v);
        Ok(())
    }
    fn serialize_none(self) -> Result<(), CanonError> {
        self.out.push(tag::NONE);
        Ok(())
    }
    fn serialize_some<T: Serialize + ?Sized>(self, v: &T) -> Result<(), CanonError> {
        self.out.push(tag::SOME);
        v.serialize(self)
    }
    fn serialize_unit(self) -> Result<(), CanonError> {
        self.out.push(tag::UNIT);
        Ok(())
    }
    fn serialize_unit_struct(self, _: &'static str) -> Result<(), CanonError> {
        self.serialize_unit()
    }
    fn serialize_unit_variant(
        self,
        _: &'static str,
        _: u32,
        v: &'static str,
    ) -> Result<(), CanonError> {
        self.str(tag::UNIT_VARIANT, v.as_bytes());
        Ok(())
    }
    fn serialize_newtype_struct<T: Serialize + ?Sized>(
        self,
        _: &'static str,
        v: &T,
    ) -> Result<(), CanonError> {
        v.serialize(self)
    }
    fn serialize_newtype_variant<T: Serialize + ?Sized>(
        self,
        _: &'static str,
        _: u32,
        variant: &'static str,
        v: &T,
    ) -> Result<(), CanonError> {
        self.str(tag::NEWTYPE_VARIANT, variant.as_bytes());
        v.serialize(self)
    }
    fn serialize_seq(self, _: Option<usize>) -> Result<Self, CanonError> {
        self.out.push(tag::SEQ);
        Ok(self)
    }
    fn serialize_tuple(self, len: usize) -> Result<Self, CanonError> {
        self.serialize_seq(Some(len))
    }
    fn serialize_tuple_struct(self, _: &'static str, len: usize) -> Result<Self, CanonError> {
        self.serialize_seq(Some(len))
    }
    fn serialize_tuple_variant(
        self,
        _: &'static str,
        _: u32,
        variant: &'static str,
        _: usize,
    ) -> Result<Self, CanonError> {
        self.str(tag::TUPLE_VARIANT, variant.as_bytes());
        Ok(self)
    }
    fn serialize_map(self, _: Option<usize>) -> Result<Self, CanonError> {
        self.out.push(tag::MAP);
        Ok(self)
    }
    fn serialize_struct(self, _: &'static str, _: usize) -> Result<Self, CanonError> {
        self.out.push(tag::STRUCT);
        Ok(self)
    }
    fn serialize_struct_variant(
        self,
        _: &'static str,
        _: u32,
        variant: &'static str,
        _: usize,
    ) -> Result<Self, CanonError> {
        self.str(tag::STRUCT_VARIANT, variant.as_bytes());
        Ok(self)
    }
}

impl ser::SerializeSeq for &mut Enc {
    type Ok = ();
    type Error = CanonError;
    fn serialize_element<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), CanonError> {
        v.serialize(&mut **self)
    }
    fn end(self) -> Result<(), CanonError> {
        Enc::end(self)
    }
}
impl ser::SerializeTuple for &mut Enc {
    type Ok = ();
    type Error = CanonError;
    fn serialize_element<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), CanonError> {
        v.serialize(&mut **self)
    }
    fn end(self) -> Result<(), CanonError> {
        Enc::end(self)
    }
}
impl ser::SerializeTupleStruct for &mut Enc {
    type Ok = ();
    type Error = CanonError;
    fn serialize_field<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), CanonError> {
        v.serialize(&mut **self)
    }
    fn end(self) -> Result<(), CanonError> {
        Enc::end(self)
    }
}
impl ser::SerializeTupleVariant for &mut Enc {
    type Ok = ();
    type Error = CanonError;
    fn serialize_field<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), CanonError> {
        v.serialize(&mut **self)
    }
    fn end(self) -> Result<(), CanonError> {
        Enc::end(self)
    }
}
impl ser::SerializeMap for &mut Enc {
    type Ok = ();
    type Error = CanonError;
    fn serialize_key<T: Serialize + ?Sized>(&mut self, k: &T) -> Result<(), CanonError> {
        k.serialize(&mut **self)
    }
    fn serialize_value<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), CanonError> {
        v.serialize(&mut **self)
    }
    fn end(self) -> Result<(), CanonError> {
        Enc::end(self)
    }
}
impl ser::SerializeStruct for &mut Enc {
    type Ok = ();
    type Error = CanonError;
    fn serialize_field<T: Serialize + ?Sized>(
        &mut self,
        key: &'static str,
        v: &T,
    ) -> Result<(), CanonError> {
        self.str(tag::STR, key.as_bytes());
        v.serialize(&mut **self)
    }
    fn end(self) -> Result<(), CanonError> {
        Enc::end(self)
    }
}
impl ser::SerializeStructVariant for &mut Enc {
    type Ok = ();
    type Error = CanonError;
    fn serialize_field<T: Serialize + ?Sized>(
        &mut self,
        key: &'static str,
        v: &T,
    ) -> Result<(), CanonError> {
        self.str(tag::STR, key.as_bytes());
        v.serialize(&mut **self)
    }
    fn end(self) -> Result<(), CanonError> {
        Enc::end(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sha256_known_answers() {
        assert_eq!(
            sha256(b"abc").to_hex(),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
        assert_eq!(
            sha256(b"").to_hex(),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        );
        assert_eq!(
            sha256(b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq").to_hex(),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"
        );
    }

    #[test]
    fn hex_roundtrip() {
        let h = sha256(b"x");
        assert_eq!(h.to_hex().parse::<Hash256>(), Ok(h));
        assert!("zz".parse::<Hash256>().is_err());
    }

    #[test]
    fn canonical_distinguishes_shapes_and_rejects_nan() {
        let a = canonical_bytes(&(1u32, "ab")).unwrap();
        let b = canonical_bytes(&(1u32, "a")).unwrap();
        assert_ne!(a, b);
        assert_eq!(
            canonical_bytes(&0.0f64).unwrap(),
            canonical_bytes(&-0.0f64).unwrap()
        );
        assert_eq!(canonical_bytes(&f64::NAN), Err(CanonError::NonFinite));
        assert_ne!(
            content_hash("a", &1u8).unwrap(),
            content_hash("b", &1u8).unwrap()
        );
    }
}
