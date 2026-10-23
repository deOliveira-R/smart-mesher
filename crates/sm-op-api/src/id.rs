//! Serializable entity ids and the deterministic allocator.
//!
//! `sm_core::EntityId` carries no serde impls, so ops use the [`Id`] newtype, which serializes to
//! the readable string `"TopoVolume#3"` (kind name, `#`, serial). The string form diffs well in
//! git and is the only form (JSON, TOML and the canonical hash all see it).

use core::fmt;
use core::str::FromStr;
use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};

use crate::error::OpError;

/// A serializable wrapper around [`EntityId`].
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct Id(pub EntityId);

impl From<EntityId> for Id {
    fn from(v: EntityId) -> Self {
        Self(v)
    }
}

impl From<Id> for EntityId {
    fn from(v: Id) -> Self {
        v.0
    }
}

impl Id {
    /// The entity kind.
    pub fn kind(self) -> EntityKind {
        self.0.kind()
    }
    /// The 56-bit serial.
    pub fn serial(self) -> u64 {
        self.0.serial()
    }
}

impl fmt::Debug for Id {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(self, f)
    }
}

impl fmt::Display for Id {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}#{}", self.0.kind(), self.0.serial())
    }
}

/// Error parsing an [`Id`] from its string form.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("invalid entity id {0:?}: expected `<Kind>#<serial>`")]
pub struct ParseIdError(pub String);

impl FromStr for Id {
    type Err = ParseIdError;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let bad = || ParseIdError(s.to_string());
        let (name, serial) = s.split_once('#').ok_or_else(bad)?;
        let serial: u64 = serial.parse().map_err(|_| bad())?;
        let kind = (0..=u8::MAX)
            .filter_map(EntityKind::from_u8)
            .find(|k| format!("{k:?}") == name)
            .ok_or_else(bad)?;
        EntityId::new(kind, serial).map(Id).ok_or_else(bad)
    }
}

impl Serialize for Id {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.collect_str(self)
    }
}

impl<'de> Deserialize<'de> for Id {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let s = String::deserialize(d)?;
        s.parse().map_err(serde::de::Error::custom)
    }
}

impl schemars::JsonSchema for Id {
    fn schema_name() -> std::borrow::Cow<'static, str> {
        "EntityId".into()
    }
    fn json_schema(_: &mut schemars::SchemaGenerator) -> schemars::Schema {
        schemars::json_schema!({
            "type": "string",
            "pattern": "^[A-Za-z]+#[0-9]+$",
            "description": "Stable entity id: `<Kind>#<serial>`, e.g. `TopoVolume#3`."
        })
    }
}

/// Deterministic, never-reusing id allocator, one serial counter per [`EntityKind`].
///
/// It lives inside the `State` (never in a static or atomic), is cloned for every op application
/// and committed only if the op succeeds. Ids are therefore a pure function of the op prefix: a
/// replay, or a fork from an old node, reassigns exactly the same ids.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize)]
pub struct IdAllocator {
    next: BTreeMap<u8, u64>,
}

impl IdAllocator {
    /// Allocate the next id of `kind`. Serials start at 1 and are never reused.
    pub fn alloc(&mut self, kind: EntityKind) -> Result<Id, OpError> {
        let slot = self.next.entry(kind as u8).or_insert(1);
        let id = EntityId::new(kind, *slot).ok_or(OpError::IdSpaceExhausted(kind))?;
        *slot += 1;
        Ok(Id(id))
    }

    /// How many ids of `kind` have ever been allocated.
    pub fn allocated(&self, kind: EntityKind) -> u64 {
        self.next.get(&(kind as u8)).map_or(0, |n| n - 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn string_roundtrip_and_errors() {
        let id = Id(EntityId::new(EntityKind::TopoVolume, 3).unwrap());
        assert_eq!(id.to_string(), "TopoVolume#3");
        assert_eq!("TopoVolume#3".parse::<Id>(), Ok(id));
        assert!("Nope#3".parse::<Id>().is_err());
        assert!("TopoVolume3".parse::<Id>().is_err());
    }

    #[test]
    fn allocator_counts_per_kind() {
        let mut a = IdAllocator::default();
        let v1 = a.alloc(EntityKind::TopoVolume).unwrap();
        let b1 = a.alloc(EntityKind::TopoBody).unwrap();
        let v2 = a.alloc(EntityKind::TopoVolume).unwrap();
        assert_eq!((v1.serial(), b1.serial(), v2.serial()), (1, 1, 2));
        assert_eq!(a.allocated(EntityKind::TopoVolume), 2);
    }
}
