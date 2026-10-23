//! Serde helper: serialize a `BTreeMap` with non-string keys as a sequence of pairs (JSON-safe).

use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::collections::BTreeMap;

pub(crate) fn serialize<S: Serializer, K: Serialize, V: Serialize>(m: &BTreeMap<K, V>, s: S) -> Result<S::Ok, S::Error> {
    s.collect_seq(m.iter())
}

pub(crate) fn deserialize<'de, D: Deserializer<'de>, K: Deserialize<'de> + Ord, V: Deserialize<'de>>(d: D) -> Result<BTreeMap<K, V>, D::Error> {
    let v: Vec<(K, V)> = Vec::deserialize(d)?;
    Ok(v.into_iter().collect())
}
