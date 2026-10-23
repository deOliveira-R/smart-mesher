//! Serialize a `BTreeMap` with non-string keys as a sequence of pairs (JSON/TOML need string
//! keys). Order is the map's ascending key order, so output is deterministic.

use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::collections::BTreeMap;

pub(crate) fn serialize<K: Serialize, V: Serialize, S: Serializer>(m: &BTreeMap<K, V>, s: S) -> Result<S::Ok, S::Error> {
    s.collect_seq(m.iter())
}

pub(crate) fn deserialize<'de, K: Deserialize<'de> + Ord, V: Deserialize<'de>, D: Deserializer<'de>>(d: D) -> Result<BTreeMap<K, V>, D::Error> {
    let v: Vec<(K, V)> = Vec::deserialize(d)?;
    Ok(v.into_iter().collect())
}
