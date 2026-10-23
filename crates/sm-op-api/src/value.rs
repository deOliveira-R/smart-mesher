//! The canonical data model: [`Value`], and serde bridges to and from it.
//!
//! Everything that is hashed or written to disk passes through [`Value`]. Tables are
//! `BTreeMap`s, so key order is canonical. `None` fields are omitted. There is no null.

use std::collections::BTreeMap;
use std::fmt;

use serde::de::{self, DeserializeOwned, IntoDeserializer, Visitor};
use serde::ser::{self, Serialize};

/// A canonical, TOML-shaped value tree.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    /// Boolean.
    Bool(bool),
    /// Signed 64-bit integer (TOML integers are `i64`; larger `u64` values are rejected).
    Int(i64),
    /// IEEE double. NaN is canonicalised to a single NaN on output.
    Float(f64),
    /// UTF-8 string.
    Str(String),
    /// Ordered list.
    Array(Vec<Value>),
    /// Table with sorted keys.
    Table(BTreeMap<String, Value>),
}

/// Error converting between Rust types and [`Value`] (or TOML text).
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("{0}")]
pub struct CodecError(pub String);

impl CodecError {
    /// Build from any message.
    pub fn new(msg: impl Into<String>) -> Self {
        Self(msg.into())
    }
}

impl ser::Error for CodecError {
    fn custom<T: fmt::Display>(msg: T) -> Self {
        Self(msg.to_string())
    }
}

impl de::Error for CodecError {
    fn custom<T: fmt::Display>(msg: T) -> Self {
        Self(msg.to_string())
    }
}

/// Serialize any `Serialize` type into a [`Value`].
pub fn to_value<T: Serialize + ?Sized>(v: &T) -> Result<Value, CodecError> {
    match v.serialize(Ser)? {
        Some(v) => Ok(v),
        None => Err(CodecError::new("top-level value is None")),
    }
}

/// Deserialize a [`Value`] into a type.
pub fn from_value<T: DeserializeOwned>(v: &Value) -> Result<T, CodecError> {
    T::deserialize(v.clone())
}

// ---------------------------------------------------------------- serializer

/// `Ok` is `None` for an absent value (`Option::None`), which tables skip.
struct Ser;

type SerOut = Option<Value>;

fn some(v: Value) -> Result<SerOut, CodecError> {
    Ok(Some(v))
}

fn present(v: SerOut) -> Result<Value, CodecError> {
    v.ok_or_else(|| CodecError::new("None is only allowed as a struct or map field"))
}

impl ser::Serializer for Ser {
    type Ok = SerOut;
    type Error = CodecError;
    type SerializeSeq = SeqSer;
    type SerializeTuple = SeqSer;
    type SerializeTupleStruct = SeqSer;
    type SerializeTupleVariant = VariantSeqSer;
    type SerializeMap = MapSer;
    type SerializeStruct = MapSer;
    type SerializeStructVariant = VariantMapSer;

    fn serialize_bool(self, v: bool) -> Result<SerOut, CodecError> {
        some(Value::Bool(v))
    }
    fn serialize_i8(self, v: i8) -> Result<SerOut, CodecError> {
        some(Value::Int(v.into()))
    }
    fn serialize_i16(self, v: i16) -> Result<SerOut, CodecError> {
        some(Value::Int(v.into()))
    }
    fn serialize_i32(self, v: i32) -> Result<SerOut, CodecError> {
        some(Value::Int(v.into()))
    }
    fn serialize_i64(self, v: i64) -> Result<SerOut, CodecError> {
        some(Value::Int(v))
    }
    fn serialize_u8(self, v: u8) -> Result<SerOut, CodecError> {
        some(Value::Int(v.into()))
    }
    fn serialize_u16(self, v: u16) -> Result<SerOut, CodecError> {
        some(Value::Int(v.into()))
    }
    fn serialize_u32(self, v: u32) -> Result<SerOut, CodecError> {
        some(Value::Int(v.into()))
    }
    fn serialize_u64(self, v: u64) -> Result<SerOut, CodecError> {
        i64::try_from(v)
            .map(|i| Some(Value::Int(i)))
            .map_err(|_| CodecError::new(format!("u64 {v} does not fit a TOML integer")))
    }
    fn serialize_f32(self, v: f32) -> Result<SerOut, CodecError> {
        some(Value::Float(v.into()))
    }
    fn serialize_f64(self, v: f64) -> Result<SerOut, CodecError> {
        some(Value::Float(v))
    }
    fn serialize_char(self, v: char) -> Result<SerOut, CodecError> {
        some(Value::Str(v.to_string()))
    }
    fn serialize_str(self, v: &str) -> Result<SerOut, CodecError> {
        some(Value::Str(v.to_owned()))
    }
    fn serialize_bytes(self, v: &[u8]) -> Result<SerOut, CodecError> {
        some(Value::Array(v.iter().map(|b| Value::Int((*b).into())).collect()))
    }
    fn serialize_none(self) -> Result<SerOut, CodecError> {
        Ok(None)
    }
    fn serialize_some<T: Serialize + ?Sized>(self, v: &T) -> Result<SerOut, CodecError> {
        v.serialize(Ser)
    }
    fn serialize_unit(self) -> Result<SerOut, CodecError> {
        some(Value::Table(BTreeMap::new()))
    }
    fn serialize_unit_struct(self, _: &'static str) -> Result<SerOut, CodecError> {
        self.serialize_unit()
    }
    fn serialize_unit_variant(
        self,
        _: &'static str,
        _: u32,
        variant: &'static str,
    ) -> Result<SerOut, CodecError> {
        some(Value::Str(variant.to_owned()))
    }
    fn serialize_newtype_struct<T: Serialize + ?Sized>(
        self,
        _: &'static str,
        v: &T,
    ) -> Result<SerOut, CodecError> {
        v.serialize(Ser)
    }
    fn serialize_newtype_variant<T: Serialize + ?Sized>(
        self,
        _: &'static str,
        _: u32,
        variant: &'static str,
        v: &T,
    ) -> Result<SerOut, CodecError> {
        let inner = present(v.serialize(Ser)?)?;
        some(Value::Table(BTreeMap::from([(variant.to_owned(), inner)])))
    }
    fn serialize_seq(self, _: Option<usize>) -> Result<SeqSer, CodecError> {
        Ok(SeqSer(Vec::new()))
    }
    fn serialize_tuple(self, n: usize) -> Result<SeqSer, CodecError> {
        self.serialize_seq(Some(n))
    }
    fn serialize_tuple_struct(self, _: &'static str, n: usize) -> Result<SeqSer, CodecError> {
        self.serialize_seq(Some(n))
    }
    fn serialize_tuple_variant(
        self,
        _: &'static str,
        _: u32,
        variant: &'static str,
        _: usize,
    ) -> Result<VariantSeqSer, CodecError> {
        Ok(VariantSeqSer(variant, Vec::new()))
    }
    fn serialize_map(self, _: Option<usize>) -> Result<MapSer, CodecError> {
        Ok(MapSer { map: BTreeMap::new(), key: None })
    }
    fn serialize_struct(self, _: &'static str, _: usize) -> Result<MapSer, CodecError> {
        self.serialize_map(None)
    }
    fn serialize_struct_variant(
        self,
        _: &'static str,
        _: u32,
        variant: &'static str,
        _: usize,
    ) -> Result<VariantMapSer, CodecError> {
        Ok(VariantMapSer(variant, BTreeMap::new()))
    }
}

struct SeqSer(Vec<Value>);

impl ser::SerializeSeq for SeqSer {
    type Ok = SerOut;
    type Error = CodecError;
    fn serialize_element<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), CodecError> {
        self.0.push(present(v.serialize(Ser)?)?);
        Ok(())
    }
    fn end(self) -> Result<SerOut, CodecError> {
        some(Value::Array(self.0))
    }
}

impl ser::SerializeTuple for SeqSer {
    type Ok = SerOut;
    type Error = CodecError;
    fn serialize_element<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), CodecError> {
        ser::SerializeSeq::serialize_element(self, v)
    }
    fn end(self) -> Result<SerOut, CodecError> {
        ser::SerializeSeq::end(self)
    }
}

impl ser::SerializeTupleStruct for SeqSer {
    type Ok = SerOut;
    type Error = CodecError;
    fn serialize_field<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), CodecError> {
        ser::SerializeSeq::serialize_element(self, v)
    }
    fn end(self) -> Result<SerOut, CodecError> {
        ser::SerializeSeq::end(self)
    }
}

struct VariantSeqSer(&'static str, Vec<Value>);

impl ser::SerializeTupleVariant for VariantSeqSer {
    type Ok = SerOut;
    type Error = CodecError;
    fn serialize_field<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), CodecError> {
        self.1.push(present(v.serialize(Ser)?)?);
        Ok(())
    }
    fn end(self) -> Result<SerOut, CodecError> {
        some(Value::Table(BTreeMap::from([(self.0.to_owned(), Value::Array(self.1))])))
    }
}

struct MapSer {
    map: BTreeMap<String, Value>,
    key: Option<String>,
}

fn key_string(v: Value) -> Result<String, CodecError> {
    match v {
        Value::Str(s) => Ok(s),
        Value::Int(i) => Ok(i.to_string()),
        Value::Bool(b) => Ok(b.to_string()),
        _ => Err(CodecError::new("map keys must be strings or integers")),
    }
}

impl ser::SerializeMap for MapSer {
    type Ok = SerOut;
    type Error = CodecError;
    fn serialize_key<T: Serialize + ?Sized>(&mut self, k: &T) -> Result<(), CodecError> {
        self.key = Some(key_string(present(k.serialize(Ser)?)?)?);
        Ok(())
    }
    fn serialize_value<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), CodecError> {
        let k = self.key.take().ok_or_else(|| CodecError::new("value without key"))?;
        if let Some(v) = v.serialize(Ser)? {
            self.map.insert(k, v);
        }
        Ok(())
    }
    fn end(self) -> Result<SerOut, CodecError> {
        some(Value::Table(self.map))
    }
}

impl ser::SerializeStruct for MapSer {
    type Ok = SerOut;
    type Error = CodecError;
    fn serialize_field<T: Serialize + ?Sized>(
        &mut self,
        k: &'static str,
        v: &T,
    ) -> Result<(), CodecError> {
        if let Some(v) = v.serialize(Ser)? {
            self.map.insert(k.to_owned(), v);
        }
        Ok(())
    }
    fn end(self) -> Result<SerOut, CodecError> {
        some(Value::Table(self.map))
    }
}

struct VariantMapSer(&'static str, BTreeMap<String, Value>);

impl ser::SerializeStructVariant for VariantMapSer {
    type Ok = SerOut;
    type Error = CodecError;
    fn serialize_field<T: Serialize + ?Sized>(
        &mut self,
        k: &'static str,
        v: &T,
    ) -> Result<(), CodecError> {
        if let Some(v) = v.serialize(Ser)? {
            self.1.insert(k.to_owned(), v);
        }
        Ok(())
    }
    fn end(self) -> Result<SerOut, CodecError> {
        some(Value::Table(BTreeMap::from([(self.0.to_owned(), Value::Table(self.1))])))
    }
}


impl ser::Serialize for Value {
    fn serialize<S: ser::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        use ser::{SerializeMap, SerializeSeq};
        match self {
            Value::Bool(b) => s.serialize_bool(*b),
            Value::Int(i) => s.serialize_i64(*i),
            Value::Float(f) => s.serialize_f64(*f),
            Value::Str(x) => s.serialize_str(x),
            Value::Array(a) => {
                let mut q = s.serialize_seq(Some(a.len()))?;
                for e in a {
                    q.serialize_element(e)?;
                }
                q.end()
            }
            Value::Table(t) => {
                let mut m = s.serialize_map(Some(t.len()))?;
                for (k, e) in t {
                    m.serialize_entry(k, e)?;
                }
                m.end()
            }
        }
    }
}

struct ValueVisitor;

impl<'de> Visitor<'de> for ValueVisitor {
    type Value = Value;
    fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("a TOML-shaped value")
    }
    fn visit_bool<E: de::Error>(self, v: bool) -> Result<Value, E> {
        Ok(Value::Bool(v))
    }
    fn visit_i64<E: de::Error>(self, v: i64) -> Result<Value, E> {
        Ok(Value::Int(v))
    }
    fn visit_u64<E: de::Error>(self, v: u64) -> Result<Value, E> {
        i64::try_from(v).map(Value::Int).map_err(|_| E::custom("integer out of i64 range"))
    }
    fn visit_f64<E: de::Error>(self, v: f64) -> Result<Value, E> {
        Ok(Value::Float(v))
    }
    fn visit_str<E: de::Error>(self, v: &str) -> Result<Value, E> {
        Ok(Value::Str(v.to_owned()))
    }
    fn visit_string<E: de::Error>(self, v: String) -> Result<Value, E> {
        Ok(Value::Str(v))
    }
    fn visit_seq<A: de::SeqAccess<'de>>(self, mut a: A) -> Result<Value, A::Error> {
        let mut out = Vec::new();
        while let Some(e) = a.next_element()? {
            out.push(e);
        }
        Ok(Value::Array(out))
    }
    fn visit_map<A: de::MapAccess<'de>>(self, mut a: A) -> Result<Value, A::Error> {
        let mut out = BTreeMap::new();
        while let Some((k, v)) = a.next_entry::<String, Value>()? {
            out.insert(k, v);
        }
        Ok(Value::Table(out))
    }
}

impl<'de> de::Deserialize<'de> for Value {
    fn deserialize<D: de::Deserializer<'de>>(d: D) -> Result<Value, D::Error> {
        d.deserialize_any(ValueVisitor)
    }
}

// -------------------------------------------------------------- deserializer

impl<'de> de::Deserializer<'de> for Value {
    type Error = CodecError;

    fn deserialize_any<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CodecError> {
        match self {
            Value::Bool(b) => visitor.visit_bool(b),
            Value::Int(i) => visitor.visit_i64(i),
            Value::Float(f) => visitor.visit_f64(f),
            Value::Str(s) => visitor.visit_string(s),
            Value::Array(a) => visitor.visit_seq(SeqDe(a.into_iter())),
            Value::Table(t) => visitor.visit_map(MapDe { it: t.into_iter(), val: None }),
        }
    }

    fn deserialize_option<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CodecError> {
        visitor.visit_some(self)
    }

    fn deserialize_unit<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CodecError> {
        visitor.visit_unit()
    }

    fn deserialize_unit_struct<V: Visitor<'de>>(
        self,
        _: &'static str,
        visitor: V,
    ) -> Result<V::Value, CodecError> {
        visitor.visit_unit()
    }

    fn deserialize_newtype_struct<V: Visitor<'de>>(
        self,
        _: &'static str,
        visitor: V,
    ) -> Result<V::Value, CodecError> {
        visitor.visit_newtype_struct(self)
    }

    fn deserialize_enum<V: Visitor<'de>>(
        self,
        _: &'static str,
        _: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value, CodecError> {
        match self {
            Value::Str(s) => visitor.visit_enum(s.into_deserializer()),
            Value::Table(t) if t.len() == 1 => match t.into_iter().next() {
                Some((variant, value)) => visitor.visit_enum(EnumDe { variant, value }),
                None => Err(CodecError::new("empty enum table")),
            },
            _ => Err(CodecError::new("expected string or single-key table for enum")),
        }
    }

    serde::forward_to_deserialize_any! {
        bool i8 i16 i32 i64 u8 u16 u32 u64 f32 f64 char str string bytes byte_buf
        seq tuple tuple_struct map struct identifier ignored_any i128 u128
    }
}

struct SeqDe(std::vec::IntoIter<Value>);

impl<'de> de::SeqAccess<'de> for SeqDe {
    type Error = CodecError;
    fn next_element_seed<T: de::DeserializeSeed<'de>>(
        &mut self,
        seed: T,
    ) -> Result<Option<T::Value>, CodecError> {
        self.0.next().map(|v| seed.deserialize(v)).transpose()
    }
    fn size_hint(&self) -> Option<usize> {
        Some(self.0.len())
    }
}

struct MapDe {
    it: std::collections::btree_map::IntoIter<String, Value>,
    val: Option<Value>,
}

impl<'de> de::MapAccess<'de> for MapDe {
    type Error = CodecError;
    fn next_key_seed<K: de::DeserializeSeed<'de>>(
        &mut self,
        seed: K,
    ) -> Result<Option<K::Value>, CodecError> {
        match self.it.next() {
            Some((k, v)) => {
                self.val = Some(v);
                seed.deserialize(KeyDe(k)).map(Some)
            }
            None => Ok(None),
        }
    }
    fn next_value_seed<V: de::DeserializeSeed<'de>>(
        &mut self,
        seed: V,
    ) -> Result<V::Value, CodecError> {
        match self.val.take() {
            Some(v) => seed.deserialize(v),
            None => Err(CodecError::new("value without key")),
        }
    }
}

/// Map keys are strings; integer-keyed maps parse them back on demand.
struct KeyDe(String);

impl KeyDe {
    fn int<'de, V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CodecError> {
        match self.0.parse::<i64>() {
            Ok(i) => visitor.visit_i64(i),
            Err(_) => visitor.visit_string(self.0),
        }
    }
}

impl<'de> de::Deserializer<'de> for KeyDe {
    type Error = CodecError;
    fn deserialize_any<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CodecError> {
        visitor.visit_string(self.0)
    }
    fn deserialize_i8<V: Visitor<'de>>(self, v: V) -> Result<V::Value, CodecError> {
        self.int(v)
    }
    fn deserialize_i16<V: Visitor<'de>>(self, v: V) -> Result<V::Value, CodecError> {
        self.int(v)
    }
    fn deserialize_i32<V: Visitor<'de>>(self, v: V) -> Result<V::Value, CodecError> {
        self.int(v)
    }
    fn deserialize_i64<V: Visitor<'de>>(self, v: V) -> Result<V::Value, CodecError> {
        self.int(v)
    }
    fn deserialize_u8<V: Visitor<'de>>(self, v: V) -> Result<V::Value, CodecError> {
        self.int(v)
    }
    fn deserialize_u16<V: Visitor<'de>>(self, v: V) -> Result<V::Value, CodecError> {
        self.int(v)
    }
    fn deserialize_u32<V: Visitor<'de>>(self, v: V) -> Result<V::Value, CodecError> {
        self.int(v)
    }
    fn deserialize_u64<V: Visitor<'de>>(self, v: V) -> Result<V::Value, CodecError> {
        self.int(v)
    }
    serde::forward_to_deserialize_any! {
        bool f32 f64 char str string bytes byte_buf option unit
        unit_struct newtype_struct seq tuple tuple_struct map struct enum identifier
        ignored_any i128 u128
    }
}

struct EnumDe {
    variant: String,
    value: Value,
}

impl<'de> de::EnumAccess<'de> for EnumDe {
    type Error = CodecError;
    type Variant = VariantDe;
    fn variant_seed<V: de::DeserializeSeed<'de>>(
        self,
        seed: V,
    ) -> Result<(V::Value, VariantDe), CodecError> {
        let v = seed.deserialize(self.variant.into_deserializer())?;
        Ok((v, VariantDe(self.value)))
    }
}

struct VariantDe(Value);

impl<'de> de::VariantAccess<'de> for VariantDe {
    type Error = CodecError;
    fn unit_variant(self) -> Result<(), CodecError> {
        Ok(())
    }
    fn newtype_variant_seed<T: de::DeserializeSeed<'de>>(
        self,
        seed: T,
    ) -> Result<T::Value, CodecError> {
        seed.deserialize(self.0)
    }
    fn tuple_variant<V: Visitor<'de>>(self, _: usize, visitor: V) -> Result<V::Value, CodecError> {
        de::Deserializer::deserialize_seq(self.0, visitor)
    }
    fn struct_variant<V: Visitor<'de>>(
        self,
        _: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value, CodecError> {
        de::Deserializer::deserialize_map(self.0, visitor)
    }
}

impl<'de> IntoDeserializer<'de, CodecError> for Value {
    type Deserializer = Value;
    fn into_deserializer(self) -> Value {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize, Debug, PartialEq)]
    enum Shape {
        Unit,
        Newtype(f64),
        Tuple(i32, i32),
        Struct { a: u8, b: Option<String> },
    }

    #[derive(Serialize, Deserialize, Debug, PartialEq)]
    struct Doc {
        name: String,
        opt: Option<i64>,
        list: Vec<Shape>,
        map: BTreeMap<u32, String>,
        unit: (),
    }

    #[test]
    fn roundtrip() {
        let d = Doc {
            name: "n".into(),
            opt: None,
            list: vec![
                Shape::Unit,
                Shape::Newtype(1.5),
                Shape::Tuple(1, -2),
                Shape::Struct { a: 3, b: Some("x".into()) },
                Shape::Struct { a: 4, b: None },
            ],
            map: BTreeMap::from([(2, "b".into()), (10, "a".into())]),
            unit: (),
        };
        let v = to_value(&d).expect("ser");
        let back: Doc = from_value(&v).expect("de");
        assert_eq!(back, d);
    }

    #[test]
    fn u64_overflow_rejected() {
        assert!(to_value(&u64::MAX).is_err());
    }
}
