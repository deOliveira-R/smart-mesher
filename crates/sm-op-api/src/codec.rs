//! Canonical parameter values and their canonical byte encoding.
//!
//! Op parameters are [`Value`] (`serde_json::Value`) restricted to what TOML can also
//! represent: no `null`, integers within `i64`, finite floats. Encoding goes through
//! `toml::Value`, which skips `None` struct fields and keeps `NaN` visible so it can be rejected.
//! Hashes use [`canonical_bytes`], never text, so TOML formatting, CRLF and key order cannot
//! change an id.

use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use serde_json::{Map, Number};

/// The parameter value model.
pub type Value = serde_json::Value;

/// Encoding or decoding of parameters failed.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error, Serialize, Deserialize)]
#[error("{0}")]
pub struct CodecError(pub String);

impl CodecError {
    /// Build an error from any message.
    pub fn new(msg: impl Into<String>) -> Self {
        Self(msg.into())
    }
}

#[derive(Serialize)]
struct Wrap<'a, T: ?Sized> {
    v: &'a T,
}

/// Encode any serializable value canonically (any top-level shape).
///
/// Rejects `NaN`/infinity, unsupported `None` inside sequences, non-string map keys and
/// integers beyond `i64`. `-0.0` becomes `+0.0`; `None` struct fields are omitted.
pub fn to_value<T: Serialize + ?Sized>(v: &T) -> Result<Value, CodecError> {
    let t = toml::Value::try_from(Wrap { v }).map_err(|e| CodecError(format!("encode: {e}")))?;
    let toml::Value::Table(mut t) = t else {
        return Err(CodecError::new("encode: internal wrapper"));
    };
    let inner = t
        .remove("v")
        .ok_or_else(|| CodecError::new("encode: value is null/None, which is not representable"))?;
    from_toml_value(inner)
}

/// Encode op parameters: like [`to_value`] but the result must be an object.
pub fn to_params<T: Serialize + ?Sized>(v: &T) -> Result<Value, CodecError> {
    let val = to_value(v)?;
    if val.is_object() {
        Ok(val)
    } else {
        Err(CodecError::new(
            "op parameters must serialize to a struct or map",
        ))
    }
}

/// Decode a [`Value`] into a typed value.
pub fn from_value<T: DeserializeOwned>(v: &Value) -> Result<T, CodecError> {
    T::deserialize(v).map_err(|e| CodecError(format!("decode: {e}")))
}

fn from_toml_value(v: toml::Value) -> Result<Value, CodecError> {
    Ok(match v {
        toml::Value::String(s) => Value::String(s),
        toml::Value::Integer(i) => Value::Number(i.into()),
        toml::Value::Float(f) => {
            if !f.is_finite() {
                return Err(CodecError::new(
                    "non-finite float (NaN or infinity) is not allowed in op parameters",
                ));
            }
            Value::Number(
                Number::from_f64(if f == 0.0 { 0.0 } else { f })
                    .ok_or_else(|| CodecError::new("bad float"))?,
            )
        }
        toml::Value::Boolean(b) => Value::Bool(b),
        toml::Value::Datetime(_) => {
            return Err(CodecError::new(
                "datetime values are not allowed in op parameters",
            ));
        }
        toml::Value::Array(a) => Value::Array(
            a.into_iter()
                .map(from_toml_value)
                .collect::<Result<_, _>>()?,
        ),
        toml::Value::Table(t) => {
            let mut m = Map::new();
            for (k, x) in t {
                m.insert(k, from_toml_value(x)?);
            }
            Value::Object(m)
        }
    })
}

/// Bring an externally supplied value (e.g. from a JSON client or a hand-built record) into
/// canonical form in place: null object entries are dropped (same meaning as an absent
/// `Option`), `-0.0` becomes `+0.0`. Fails on `null` elsewhere and on integers beyond `i64`.
pub fn canonicalize(v: &mut Value) -> Result<(), CodecError> {
    match v {
        Value::Null => Err(CodecError::new("null is not allowed in op parameters")),
        Value::Bool(_) | Value::String(_) => Ok(()),
        Value::Number(n) => {
            if n.is_f64() {
                if n.as_f64() == Some(0.0) {
                    *n = Number::from_f64(0.0).ok_or_else(|| CodecError::new("bad float"))?;
                }
                Ok(())
            } else if n.is_i64() {
                Ok(())
            } else {
                Err(CodecError(format!(
                    "integer {n} exceeds the i64 range supported by TOML"
                )))
            }
        }
        Value::Array(a) => a.iter_mut().try_for_each(canonicalize),
        Value::Object(m) => {
            m.retain(|_, x| !x.is_null());
            m.values_mut().try_for_each(canonicalize)
        }
    }
}

/// Canonical byte encoding of a value: type-tagged, length-framed, object keys sorted,
/// `-0.0` folded to `+0.0`. Stable across serde_json features and TOML/JSON text forms.
pub fn canonical_bytes(v: &Value) -> Vec<u8> {
    let mut out = Vec::new();
    write_canonical(v, &mut out);
    out
}

fn write_len(n: usize, out: &mut Vec<u8>) {
    out.extend_from_slice(&(n as u64).to_le_bytes());
}

fn write_canonical(v: &Value, out: &mut Vec<u8>) {
    match v {
        Value::Null => out.push(0),
        Value::Bool(false) => out.push(1),
        Value::Bool(true) => out.push(2),
        Value::Number(n) => {
            if let Some(i) = n.as_i64() {
                out.push(3);
                out.extend_from_slice(&i.to_le_bytes());
            } else if let Some(u) = n.as_u64() {
                out.push(4);
                out.extend_from_slice(&u.to_le_bytes());
            } else {
                let f = n.as_f64().unwrap_or(f64::NAN);
                out.push(5);
                out.extend_from_slice(&(if f == 0.0 { 0.0f64 } else { f }).to_bits().to_le_bytes());
            }
        }
        Value::String(s) => {
            out.push(6);
            write_len(s.len(), out);
            out.extend_from_slice(s.as_bytes());
        }
        Value::Array(a) => {
            out.push(7);
            write_len(a.len(), out);
            a.iter().for_each(|x| write_canonical(x, out));
        }
        Value::Object(m) => {
            out.push(8);
            write_len(m.len(), out);
            let mut keys: Vec<&String> = m.keys().collect();
            keys.sort();
            for k in keys {
                write_len(k.len(), out);
                out.extend_from_slice(k.as_bytes());
                write_canonical(&m[k], out);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[derive(Serialize)]
    struct P {
        a: f64,
        b: Option<u32>,
        c: Vec<i64>,
    }

    #[test]
    fn none_is_omitted_and_negzero_folded() {
        let v = to_params(&P {
            a: -0.0,
            b: None,
            c: vec![1, 2],
        })
        .unwrap();
        assert_eq!(v, json!({"a": 0.0, "c": [1, 2]}));
        assert!(v["a"].as_f64().unwrap().is_sign_positive());
    }

    #[test]
    fn nan_and_inf_rejected() {
        assert!(
            to_params(&P {
                a: f64::NAN,
                b: None,
                c: vec![]
            })
            .is_err()
        );
        assert!(
            to_params(&P {
                a: f64::INFINITY,
                b: None,
                c: vec![]
            })
            .is_err()
        );
    }

    #[test]
    fn big_u64_rejected() {
        assert!(to_value(&u64::MAX).is_err());
    }

    #[test]
    fn canonical_bytes_ignore_key_order_but_not_int_vs_float() {
        let a = json!({"x": 1, "y": [1, 2]});
        let b: Value = serde_json::from_str(r#"{"y":[1,2],"x":1}"#).unwrap();
        assert_eq!(canonical_bytes(&a), canonical_bytes(&b));
        assert_ne!(canonical_bytes(&json!(1)), canonical_bytes(&json!(1.0)));
        assert_eq!(canonical_bytes(&json!(-0.0)), canonical_bytes(&json!(0.0)));
    }

    #[test]
    fn canonicalize_drops_null_entries() {
        let mut v = json!({"a": null, "b": {"c": null, "d": 1}});
        canonicalize(&mut v).unwrap();
        assert_eq!(v, json!({"b": {"d": 1}}));
        assert!(canonicalize(&mut json!([null])).is_err());
    }
}
