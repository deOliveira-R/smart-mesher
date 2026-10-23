//! [`Eid`]: serde/schemars wrapper around `sm_core::EntityId`.

use schemars::{JsonSchema, Schema, SchemaGenerator, json_schema};
use serde::de::{self, Visitor};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use sm_core::EntityId;
use std::borrow::Cow;

/// An [`EntityId`] usable in op parameters: serializes as its raw `u64` (as a TOML integer).
///
/// `EntityId` itself carries no serde impls (frozen contract), so ops use this wrapper.
/// Kinds are below 128, so the raw value always fits `i64`.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct Eid(pub EntityId);

impl From<EntityId> for Eid {
    fn from(id: EntityId) -> Self {
        Eid(id)
    }
}

impl Serialize for Eid {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        let raw = i64::try_from(self.0.to_raw())
            .map_err(|_| serde::ser::Error::custom("entity id exceeds i64"))?;
        s.serialize_i64(raw)
    }
}

struct EidVisitor;

impl Visitor<'_> for EidVisitor {
    type Value = Eid;
    fn expecting(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("a raw entity id integer")
    }
    fn visit_i64<E: de::Error>(self, v: i64) -> Result<Eid, E> {
        let raw = u64::try_from(v).map_err(|_| E::custom("negative entity id"))?;
        EntityId::from_raw(raw)
            .map(Eid)
            .ok_or_else(|| E::custom("invalid entity id"))
    }
    fn visit_u64<E: de::Error>(self, v: u64) -> Result<Eid, E> {
        EntityId::from_raw(v)
            .map(Eid)
            .ok_or_else(|| E::custom("invalid entity id"))
    }
}

impl<'de> Deserialize<'de> for Eid {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        d.deserialize_i64(EidVisitor)
    }
}

impl JsonSchema for Eid {
    fn schema_name() -> Cow<'static, str> {
        Cow::Borrowed("EntityId")
    }
    fn json_schema(_: &mut SchemaGenerator) -> Schema {
        json_schema!({
            "type": "integer",
            "minimum": 1,
            "description": "Stable entity id: (kind << 56) | serial, as returned by the op that created the entity."
        })
    }
}
