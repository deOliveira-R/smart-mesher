//! Layer stores: the independent, pluggable pieces a [`State`](crate::State) is made of.
//!
//! The api crate knows nothing about what a layer holds. A concept crate defines a type
//! implementing [`Layer`] (model topology, partition, intent, meshes, ...), registers it
//! under a [`LayerId`], and mutates it only through *patches*. A patch is a serializable
//! value; applying one returns its **inverse**. That single rule is what makes every op's
//! delta reversible and persistable without per-layer undo code.

use crate::eid::Eid;
use crate::error::LayerError;
use crate::hash::StateHasher;
use crate::value::{Value, from_value, to_value};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use std::any::Any;
use std::collections::BTreeMap;

/// Name of a layer store. Well-known names are provided for the standard layers; any
/// concept may add its own.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct LayerId(pub &'static str);

impl LayerId {
    /// Model layer: topology, shape, semantics (CAD-like source of truth).
    pub const MODEL: LayerId = LayerId("model");
    /// Partition layer: virtual decomposition.
    pub const PARTITION: LayerId = LayerId("partition");
    /// Intent layer: sizing, schemes, requested meshing parameters.
    pub const INTENT: LayerId = LayerId("intent");
    /// Meshes layer: generated mesh snapshots.
    pub const MESHES: LayerId = LayerId("meshes");
    /// Derived layer name (used for cache keys of derived products, see
    /// [`DerivedCache`](crate::DerivedCache)). Derived data is never part of the state hash.
    pub const DERIVED: LayerId = LayerId("derived");

    /// The name.
    pub const fn as_str(self) -> &'static str {
        self.0
    }
}

/// A typed layer store.
///
/// Contract for implementors:
/// * `apply` is atomic: on `Err` the layer is unchanged.
/// * `apply` is deterministic and returns a patch that, applied next, restores the
///   exact prior content (so `apply(apply(p).inverse) == identity`).
/// * `hash_into` feeds a canonical encoding of the content (no `HashMap` order, no
///   pointer values; geometry should be quantized if platform independence is needed).
pub trait Layer: Clone + Send + Sync + 'static {
    /// The serializable mutation type.
    type Patch: Serialize + DeserializeOwned;

    /// Apply a patch, returning its inverse.
    fn apply(&mut self, patch: &Self::Patch) -> Result<Self::Patch, LayerError>;

    /// Feed canonical content into the hasher.
    fn hash_into(&self, h: &mut StateHasher);
}

/// Object-safe form of [`Layer`]; implemented automatically. Patches cross this boundary
/// as [`Value`]s.
pub trait DynLayer: Send + Sync {
    /// Clone into a box.
    fn clone_box(&self) -> Box<dyn DynLayer>;
    /// Downcast support.
    fn as_any(&self) -> &dyn Any;
    /// Downcast support.
    fn as_any_mut(&mut self) -> &mut dyn Any;
    /// Apply a patch given as a [`Value`]; returns the inverse patch as a [`Value`].
    fn apply_value(&mut self, patch: &Value) -> Result<Value, LayerError>;
    /// Feed canonical content into the hasher.
    fn hash_into(&self, h: &mut StateHasher);
}

impl<L: Layer> DynLayer for L {
    fn clone_box(&self) -> Box<dyn DynLayer> {
        Box::new(self.clone())
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
    fn apply_value(&mut self, patch: &Value) -> Result<Value, LayerError> {
        let p: L::Patch = from_value(patch)?;
        let inv = Layer::apply(self, &p)?;
        Ok(to_value(&inv)?)
    }
    fn hash_into(&self, h: &mut StateHasher) {
        Layer::hash_into(self, h);
    }
}

/// Reference layer: an ordered map from entity id to a record `T`. Covers most
/// "table of entities" layers; concept crates can use it directly or write their own.
#[derive(Clone, Debug, PartialEq)]
pub struct MapLayer<T> {
    items: BTreeMap<Eid, T>,
}

impl<T> Default for MapLayer<T> {
    fn default() -> Self {
        MapLayer {
            items: BTreeMap::new(),
        }
    }
}

/// One change in a [`MapPatch`]: set the record, or remove it when `value` is `None`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MapEntry<T> {
    /// Entity affected.
    pub id: Eid,
    /// New record; `None` removes the entity.
    pub value: Option<T>,
}

/// Patch for [`MapLayer`]: entries applied in order.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MapPatch<T> {
    /// Ordered changes.
    pub entries: Vec<MapEntry<T>>,
}

impl<T> MapPatch<T> {
    /// Empty patch.
    pub fn new() -> Self {
        MapPatch {
            entries: Vec::new(),
        }
    }
    /// Add "set `id` to `value`".
    pub fn set(mut self, id: impl Into<Eid>, value: T) -> Self {
        self.entries.push(MapEntry {
            id: id.into(),
            value: Some(value),
        });
        self
    }
    /// Add "remove `id`".
    pub fn remove(mut self, id: impl Into<Eid>) -> Self {
        self.entries.push(MapEntry {
            id: id.into(),
            value: None,
        });
        self
    }
}

impl<T> Default for MapPatch<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> MapLayer<T> {
    /// Record for `id`.
    pub fn get(&self, id: impl Into<Eid>) -> Option<&T> {
        self.items.get(&id.into())
    }
    /// Whether `id` exists.
    pub fn contains(&self, id: impl Into<Eid>) -> bool {
        self.items.contains_key(&id.into())
    }
    /// Number of records.
    pub fn len(&self) -> usize {
        self.items.len()
    }
    /// Whether empty.
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
    /// Iterate in id order.
    pub fn iter(&self) -> impl Iterator<Item = (Eid, &T)> {
        self.items.iter().map(|(k, v)| (*k, v))
    }
}

impl<T> Layer for MapLayer<T>
where
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    type Patch = MapPatch<T>;

    fn apply(&mut self, patch: &MapPatch<T>) -> Result<MapPatch<T>, LayerError> {
        let mut inverse = Vec::with_capacity(patch.entries.len());
        for e in &patch.entries {
            let old = match &e.value {
                Some(v) => self.items.insert(e.id, v.clone()),
                None => self.items.remove(&e.id),
            };
            inverse.push(MapEntry {
                id: e.id,
                value: old,
            });
        }
        inverse.reverse();
        Ok(MapPatch { entries: inverse })
    }

    fn hash_into(&self, h: &mut StateHasher) {
        h.u64(self.items.len() as u64);
        for (k, v) in &self.items {
            h.u64(k.0.to_raw());
            match to_value(v) {
                Ok(val) => h.value(&val),
                Err(e) => h.str(&e.0),
            }
        }
    }
}
