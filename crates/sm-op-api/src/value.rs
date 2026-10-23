//! [`Value`]: the dynamic, canonical data model every op, patch and node is reduced to.
//!
//! It is exactly the TOML data model minus datetimes: bool, 64-bit signed int, finite
//! float, string, array, table with sorted string keys. Anything `Serialize` converts via
//! [`to_value`] and back via [`from_value`]. Absent optional fields are omitted (there is
//! no null), which makes the encoding canonical.

use serde::de::{self, DeserializeOwned, IntoDeserializer, Visitor};
use serde::ser::{self, Serialize};
use std::collections::BTreeMap;

/// A table: string keys in sorted order (this is what makes serialization canonical).
pub type Table = BTreeMap<String, Value>;

/// Dynamic value; see the [module docs](self).
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    /// Boolean.
    Bool(bool),
    /// 64-bit signed integer.
    Int(i64),
    /// Finite 64-bit float.
    Float(f64),
    /// UTF-8 string.
    Str(String),
    /// Heterogeneous array.
    Array(Vec<Value>),
    /// Table with sorted keys.
    Table(Table),
}

impl Value {
    /// Table contents, if this is a table.
    pub fn as_table(&self) -> Option<&Table> {
        if let Value::Table(t) = self {
            Some(t)
        } else {
            None
        }
    }

    /// String contents, if this is a string.
    pub fn as_str(&self) -> Option<&str> {
        if let Value::Str(s) = self {
            Some(s)
        } else {
            None
        }
    }

    /// Integer contents, if this is an integer.
    pub fn as_i64(&self) -> Option<i64> {
        if let Value::Int(i) = self {
            Some(*i)
        } else {
            None
        }
    }

    /// Array contents, if this is an array.
    pub fn as_array(&self) -> Option<&[Value]> {
        if let Value::Array(a) = self {
            Some(a)
        } else {
            None
        }
    }

    /// Look up a key in a table value.
    pub fn get(&self, key: &str) -> Option<&Value> {
        self.as_table()?.get(key)
    }

    /// Compact JSON text (sorted keys, so deterministic). Used for schemas and MCP payloads.
    pub fn to_json(&self) -> String {
        let mut out = String::new();
        self.write_json(&mut out);
        out
    }

    fn write_json(&self, out: &mut String) {
        match self {
            Value::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
            Value::Int(i) => out.push_str(&i.to_string()),
            Value::Float(f) => out.push_str(&format!("{f:?}")),
            Value::Str(s) => json_str(s, out),
            Value::Array(a) => {
                out.push('[');
                for (i, x) in a.iter().enumerate() {
                    if i > 0 {
                        out.push(',');
                    }
                    x.write_json(out);
                }
                out.push(']');
            }
            Value::Table(t) => {
                out.push('{');
                for (i, (k, x)) in t.iter().enumerate() {
                    if i > 0 {
                        out.push(',');
                    }
                    json_str(k, out);
                    out.push(':');
                    x.write_json(out);
                }
                out.push('}');
            }
        }
    }
}

fn json_str(s: &str, out: &mut String) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
}

impl Serialize for Value {
    fn serialize<S: ser::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        use ser::{SerializeMap, SerializeSeq};
        match self {
            Value::Bool(b) => s.serialize_bool(*b),
            Value::Int(i) => s.serialize_i64(*i),
            Value::Float(f) => s.serialize_f64(*f),
            Value::Str(x) => s.serialize_str(x),
            Value::Array(a) => {
                let mut q = s.serialize_seq(Some(a.len()))?;
                for x in a {
                    q.serialize_element(x)?;
                }
                q.end()
            }
            Value::Table(t) => {
                let mut m = s.serialize_map(Some(t.len()))?;
                for (k, x) in t {
                    m.serialize_entry(k, x)?;
                }
                m.end()
            }
        }
    }
}

struct ValueVisitor;

impl<'de> Visitor<'de> for ValueVisitor {
    type Value = Value;
    fn expecting(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("a TOML-like value")
    }
    fn visit_bool<E: de::Error>(self, v: bool) -> Result<Value, E> {
        Ok(Value::Bool(v))
    }
    fn visit_i64<E: de::Error>(self, v: i64) -> Result<Value, E> {
        Ok(Value::Int(v))
    }
    fn visit_u64<E: de::Error>(self, v: u64) -> Result<Value, E> {
        i64::try_from(v)
            .map(Value::Int)
            .map_err(|_| E::custom("integer exceeds i64::MAX"))
    }
    fn visit_f64<E: de::Error>(self, v: f64) -> Result<Value, E> {
        if v.is_finite() {
            Ok(Value::Float(v))
        } else {
            Err(E::custom("non-finite float"))
        }
    }
    fn visit_str<E: de::Error>(self, v: &str) -> Result<Value, E> {
        Ok(Value::Str(v.to_owned()))
    }
    fn visit_seq<A: de::SeqAccess<'de>>(self, mut a: A) -> Result<Value, A::Error> {
        let mut v = Vec::new();
        while let Some(x) = a.next_element()? {
            v.push(x);
        }
        Ok(Value::Array(v))
    }
    fn visit_map<A: de::MapAccess<'de>>(self, mut a: A) -> Result<Value, A::Error> {
        let mut t = Table::new();
        while let Some((k, x)) = a.next_entry::<String, Value>()? {
            t.insert(k, x);
        }
        Ok(Value::Table(t))
    }
}

impl<'de> serde::Deserialize<'de> for Value {
    fn deserialize<D: de::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        d.deserialize_any(ValueVisitor)
    }
}

/// Error converting between Rust values and [`Value`].
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("{0}")]
pub struct ValueError(pub String);

impl ser::Error for ValueError {
    fn custom<T: core::fmt::Display>(msg: T) -> Self {
        ValueError(msg.to_string())
    }
}

impl de::Error for ValueError {
    fn custom<T: core::fmt::Display>(msg: T) -> Self {
        ValueError(msg.to_string())
    }
}

/// Convert any `Serialize` value to a [`Value`].
///
/// `None` fields are omitted. Fails on non-finite floats, `u64` above `i64::MAX`, `None`
/// at the top level or inside an array, and non-string map keys.
pub fn to_value<T: Serialize + ?Sized>(v: &T) -> Result<Value, ValueError> {
    v.serialize(Ser)?
        .ok_or_else(|| ValueError("cannot represent `None` here (no null in Value)".into()))
}

/// Convert a [`Value`] back into a typed value.
pub fn from_value<T: DeserializeOwned>(v: &Value) -> Result<T, ValueError> {
    T::deserialize(De(v))
}

// ---------------- serializer ----------------

struct Ser;

type Out = Option<Value>;

struct SeqSer(Vec<Value>, Option<&'static str>);
struct MapSer(Table, Option<String>, Option<&'static str>);

fn some(v: Value) -> Result<Out, ValueError> {
    Ok(Some(v))
}

impl ser::Serializer for Ser {
    type Ok = Out;
    type Error = ValueError;
    type SerializeSeq = SeqSer;
    type SerializeTuple = SeqSer;
    type SerializeTupleStruct = SeqSer;
    type SerializeTupleVariant = SeqSer;
    type SerializeMap = MapSer;
    type SerializeStruct = MapSer;
    type SerializeStructVariant = MapSer;

    fn serialize_bool(self, v: bool) -> Result<Out, ValueError> {
        some(Value::Bool(v))
    }
    fn serialize_i8(self, v: i8) -> Result<Out, ValueError> {
        some(Value::Int(v.into()))
    }
    fn serialize_i16(self, v: i16) -> Result<Out, ValueError> {
        some(Value::Int(v.into()))
    }
    fn serialize_i32(self, v: i32) -> Result<Out, ValueError> {
        some(Value::Int(v.into()))
    }
    fn serialize_i64(self, v: i64) -> Result<Out, ValueError> {
        some(Value::Int(v))
    }
    fn serialize_u8(self, v: u8) -> Result<Out, ValueError> {
        some(Value::Int(v.into()))
    }
    fn serialize_u16(self, v: u16) -> Result<Out, ValueError> {
        some(Value::Int(v.into()))
    }
    fn serialize_u32(self, v: u32) -> Result<Out, ValueError> {
        some(Value::Int(v.into()))
    }
    fn serialize_u64(self, v: u64) -> Result<Out, ValueError> {
        i64::try_from(v)
            .map(Value::Int)
            .map(Some)
            .map_err(|_| ValueError(format!("integer {v} exceeds i64::MAX")))
    }
    fn serialize_f32(self, v: f32) -> Result<Out, ValueError> {
        self.serialize_f64(f64::from(v))
    }
    fn serialize_f64(self, v: f64) -> Result<Out, ValueError> {
        if v.is_finite() {
            some(Value::Float(v))
        } else {
            Err(ValueError("non-finite float is not representable".into()))
        }
    }
    fn serialize_char(self, v: char) -> Result<Out, ValueError> {
        some(Value::Str(v.to_string()))
    }
    fn serialize_str(self, v: &str) -> Result<Out, ValueError> {
        some(Value::Str(v.to_owned()))
    }
    fn serialize_bytes(self, v: &[u8]) -> Result<Out, ValueError> {
        some(Value::Array(
            v.iter().map(|b| Value::Int(i64::from(*b))).collect(),
        ))
    }
    fn serialize_none(self) -> Result<Out, ValueError> {
        Ok(None)
    }
    fn serialize_some<T: Serialize + ?Sized>(self, v: &T) -> Result<Out, ValueError> {
        v.serialize(self)
    }
    fn serialize_unit(self) -> Result<Out, ValueError> {
        some(Value::Table(Table::new()))
    }
    fn serialize_unit_struct(self, _: &'static str) -> Result<Out, ValueError> {
        self.serialize_unit()
    }
    fn serialize_unit_variant(
        self,
        _: &'static str,
        _: u32,
        v: &'static str,
    ) -> Result<Out, ValueError> {
        some(Value::Str(v.to_owned()))
    }
    fn serialize_newtype_struct<T: Serialize + ?Sized>(
        self,
        _: &'static str,
        v: &T,
    ) -> Result<Out, ValueError> {
        v.serialize(self)
    }
    fn serialize_newtype_variant<T: Serialize + ?Sized>(
        self,
        _: &'static str,
        _: u32,
        variant: &'static str,
        v: &T,
    ) -> Result<Out, ValueError> {
        let inner = to_value(v)?;
        some(Value::Table(Table::from([(variant.to_owned(), inner)])))
    }
    fn serialize_seq(self, len: Option<usize>) -> Result<SeqSer, ValueError> {
        Ok(SeqSer(Vec::with_capacity(len.unwrap_or(0)), None))
    }
    fn serialize_tuple(self, len: usize) -> Result<SeqSer, ValueError> {
        self.serialize_seq(Some(len))
    }
    fn serialize_tuple_struct(self, _: &'static str, len: usize) -> Result<SeqSer, ValueError> {
        self.serialize_seq(Some(len))
    }
    fn serialize_tuple_variant(
        self,
        _: &'static str,
        _: u32,
        v: &'static str,
        len: usize,
    ) -> Result<SeqSer, ValueError> {
        Ok(SeqSer(Vec::with_capacity(len), Some(v)))
    }
    fn serialize_map(self, _: Option<usize>) -> Result<MapSer, ValueError> {
        Ok(MapSer(Table::new(), None, None))
    }
    fn serialize_struct(self, _: &'static str, _: usize) -> Result<MapSer, ValueError> {
        self.serialize_map(None)
    }
    fn serialize_struct_variant(
        self,
        _: &'static str,
        _: u32,
        v: &'static str,
        _: usize,
    ) -> Result<MapSer, ValueError> {
        Ok(MapSer(Table::new(), None, Some(v)))
    }
}

impl SeqSer {
    fn push<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), ValueError> {
        self.0.push(to_value(v)?);
        Ok(())
    }
    fn done(self) -> Result<Out, ValueError> {
        let arr = Value::Array(self.0);
        match self.1 {
            Some(var) => some(Value::Table(Table::from([(var.to_owned(), arr)]))),
            None => some(arr),
        }
    }
}

impl ser::SerializeSeq for SeqSer {
    type Ok = Out;
    type Error = ValueError;
    fn serialize_element<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), ValueError> {
        self.push(v)
    }
    fn end(self) -> Result<Out, ValueError> {
        self.done()
    }
}
impl ser::SerializeTuple for SeqSer {
    type Ok = Out;
    type Error = ValueError;
    fn serialize_element<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), ValueError> {
        self.push(v)
    }
    fn end(self) -> Result<Out, ValueError> {
        self.done()
    }
}
impl ser::SerializeTupleStruct for SeqSer {
    type Ok = Out;
    type Error = ValueError;
    fn serialize_field<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), ValueError> {
        self.push(v)
    }
    fn end(self) -> Result<Out, ValueError> {
        self.done()
    }
}
impl ser::SerializeTupleVariant for SeqSer {
    type Ok = Out;
    type Error = ValueError;
    fn serialize_field<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), ValueError> {
        self.push(v)
    }
    fn end(self) -> Result<Out, ValueError> {
        self.done()
    }
}

impl MapSer {
    fn put<T: Serialize + ?Sized>(&mut self, k: String, v: &T) -> Result<(), ValueError> {
        if let Some(x) = v.serialize(Ser)? {
            self.0.insert(k, x);
        }
        Ok(())
    }
    fn done(self) -> Result<Out, ValueError> {
        let t = Value::Table(self.0);
        match self.2 {
            Some(var) => some(Value::Table(Table::from([(var.to_owned(), t)]))),
            None => some(t),
        }
    }
}

impl ser::SerializeMap for MapSer {
    type Ok = Out;
    type Error = ValueError;
    fn serialize_key<T: Serialize + ?Sized>(&mut self, k: &T) -> Result<(), ValueError> {
        match to_value(k)? {
            Value::Str(s) => {
                self.1 = Some(s);
                Ok(())
            }
            _ => Err(ValueError("map keys must be strings".into())),
        }
    }
    fn serialize_value<T: Serialize + ?Sized>(&mut self, v: &T) -> Result<(), ValueError> {
        let k = self
            .1
            .take()
            .ok_or_else(|| ValueError("value without key".into()))?;
        self.put(k, v)
    }
    fn end(self) -> Result<Out, ValueError> {
        self.done()
    }
}
impl ser::SerializeStruct for MapSer {
    type Ok = Out;
    type Error = ValueError;
    fn serialize_field<T: Serialize + ?Sized>(
        &mut self,
        k: &'static str,
        v: &T,
    ) -> Result<(), ValueError> {
        self.put(k.to_owned(), v)
    }
    fn end(self) -> Result<Out, ValueError> {
        self.done()
    }
}
impl ser::SerializeStructVariant for MapSer {
    type Ok = Out;
    type Error = ValueError;
    fn serialize_field<T: Serialize + ?Sized>(
        &mut self,
        k: &'static str,
        v: &T,
    ) -> Result<(), ValueError> {
        self.put(k.to_owned(), v)
    }
    fn end(self) -> Result<Out, ValueError> {
        self.done()
    }
}

// ---------------- deserializer ----------------

struct De<'a>(&'a Value);

impl<'de, 'a> de::Deserializer<'de> for De<'a> {
    type Error = ValueError;

    fn deserialize_any<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, ValueError> {
        match self.0 {
            Value::Bool(b) => visitor.visit_bool(*b),
            Value::Int(i) => visitor.visit_i64(*i),
            Value::Float(f) => visitor.visit_f64(*f),
            Value::Str(s) => visitor.visit_str(s),
            Value::Array(a) => visitor.visit_seq(SeqDe(a.iter())),
            Value::Table(t) => visitor.visit_map(MapDe {
                it: t.iter(),
                val: None,
            }),
        }
    }

    fn deserialize_option<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, ValueError> {
        visitor.visit_some(self)
    }

    fn deserialize_unit<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, ValueError> {
        visitor.visit_unit()
    }

    fn deserialize_newtype_struct<V: Visitor<'de>>(
        self,
        _: &'static str,
        visitor: V,
    ) -> Result<V::Value, ValueError> {
        visitor.visit_newtype_struct(self)
    }

    fn deserialize_enum<V: Visitor<'de>>(
        self,
        _: &'static str,
        _: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value, ValueError> {
        match self.0 {
            Value::Str(s) => visitor.visit_enum(s.as_str().into_deserializer()),
            Value::Table(t) if t.len() == 1 => {
                let (k, v) = t.iter().next().expect("len checked");
                visitor.visit_enum(EnumDe {
                    variant: k,
                    value: v,
                })
            }
            _ => Err(ValueError(
                "expected string or single-key table for enum".into(),
            )),
        }
    }

    serde::forward_to_deserialize_any! {
        bool i8 i16 i32 i64 i128 u8 u16 u32 u64 u128 f32 f64 char str string bytes byte_buf
        unit_struct seq tuple tuple_struct map struct identifier ignored_any
    }
}

struct SeqDe<'a>(core::slice::Iter<'a, Value>);

impl<'de, 'a> de::SeqAccess<'de> for SeqDe<'a> {
    type Error = ValueError;
    fn next_element_seed<T: de::DeserializeSeed<'de>>(
        &mut self,
        seed: T,
    ) -> Result<Option<T::Value>, ValueError> {
        self.0.next().map(|v| seed.deserialize(De(v))).transpose()
    }
}

struct MapDe<'a> {
    it: std::collections::btree_map::Iter<'a, String, Value>,
    val: Option<&'a Value>,
}

impl<'de, 'a> de::MapAccess<'de> for MapDe<'a> {
    type Error = ValueError;
    fn next_key_seed<K: de::DeserializeSeed<'de>>(
        &mut self,
        seed: K,
    ) -> Result<Option<K::Value>, ValueError> {
        match self.it.next() {
            Some((k, v)) => {
                self.val = Some(v);
                seed.deserialize(k.as_str().into_deserializer()).map(Some)
            }
            None => Ok(None),
        }
    }
    fn next_value_seed<V: de::DeserializeSeed<'de>>(
        &mut self,
        seed: V,
    ) -> Result<V::Value, ValueError> {
        let v = self
            .val
            .take()
            .ok_or_else(|| ValueError("value without key".into()))?;
        seed.deserialize(De(v))
    }
}

struct EnumDe<'a> {
    variant: &'a str,
    value: &'a Value,
}

impl<'de, 'a> de::EnumAccess<'de> for EnumDe<'a> {
    type Error = ValueError;
    type Variant = VariantDe<'a>;
    fn variant_seed<V: de::DeserializeSeed<'de>>(
        self,
        seed: V,
    ) -> Result<(V::Value, VariantDe<'a>), ValueError> {
        let v = seed.deserialize(self.variant.into_deserializer())?;
        Ok((v, VariantDe(self.value)))
    }
}

struct VariantDe<'a>(&'a Value);

impl<'de, 'a> de::VariantAccess<'de> for VariantDe<'a> {
    type Error = ValueError;
    fn unit_variant(self) -> Result<(), ValueError> {
        Ok(())
    }
    fn newtype_variant_seed<T: de::DeserializeSeed<'de>>(
        self,
        seed: T,
    ) -> Result<T::Value, ValueError> {
        seed.deserialize(De(self.0))
    }
    fn tuple_variant<V: Visitor<'de>>(self, _: usize, visitor: V) -> Result<V::Value, ValueError> {
        de::Deserializer::deserialize_any(De(self.0), visitor)
    }
    fn struct_variant<V: Visitor<'de>>(
        self,
        _: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value, ValueError> {
        de::Deserializer::deserialize_any(De(self.0), visitor)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize, Debug, PartialEq)]
    enum E {
        A,
        B(i32),
        C { x: f64, y: Option<String> },
        D(u8, u8),
    }

    #[derive(Serialize, Deserialize, Debug, PartialEq)]
    struct S {
        n: u32,
        v: Vec<E>,
        t: (i8, bool),
        o: Option<i64>,
        m: std::collections::BTreeMap<String, f64>,
    }

    #[test]
    fn roundtrip() {
        let s = S {
            n: 7,
            v: vec![
                E::A,
                E::B(-3),
                E::C { x: 1.5, y: None },
                E::C {
                    x: -0.0,
                    y: Some("q\"\n".into()),
                },
                E::D(1, 2),
            ],
            t: (-1, true),
            o: None,
            m: [("a".to_owned(), 1.0)].into(),
        };
        let v = to_value(&s).unwrap();
        assert!(v.get("o").is_none());
        assert_eq!(from_value::<S>(&v).unwrap(), s);
    }

    #[test]
    fn rejects_nonfinite_and_big() {
        assert!(to_value(&f64::NAN).is_err());
        assert!(to_value(&u64::MAX).is_err());
        assert!(to_value(&Option::<u8>::None).is_err());
    }
}
