//! [`State`]: the composable document state, plus [`Delta`]s and the [`DerivedCache`].

use crate::error::{ApplyError, OpError};
use crate::hash::{Hash, StateHasher};
use crate::layer::{DynLayer, Layer, LayerId};
use crate::toml::{self, TomlError};
use crate::value::{Table, Value, from_value, to_value};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use std::any::Any;
use std::collections::BTreeMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};

/// Per-kind serial counters. Serial `n` of a kind is handed out exactly once; serial 0 is
/// never used. Counters are part of the state (and of every delta), so id assignment is a
/// pure function of the op sequence.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct IdCounters {
    last: BTreeMap<u8, u64>,
}

impl IdCounters {
    /// Last serial handed out for `kind` (0 if none).
    pub fn last(&self, kind: EntityKind) -> u64 {
        self.last.get(&(kind as u8)).copied().unwrap_or(0)
    }

    fn get_raw(&self, kind: u8) -> u64 {
        self.last.get(&kind).copied().unwrap_or(0)
    }

    fn set_raw(&mut self, kind: u8, v: u64) {
        if v == 0 {
            self.last.remove(&kind);
        } else {
            self.last.insert(kind, v);
        }
    }
}

struct Slot {
    layer: Box<dyn DynLayer>,
    hash: OnceLock<Hash>,
}

impl Clone for Slot {
    fn clone(&self) -> Self {
        let hash = OnceLock::new();
        if let Some(h) = self.hash.get() {
            let _ = hash.set(*h);
        }
        Slot {
            layer: self.layer.clone_box(),
            hash,
        }
    }
}

/// The whole document state: independent layer stores, id counters, and a derived cache.
///
/// Created by [`Registry::new_state`](crate::Registry::new_state) with one empty store per
/// registered layer. It can only be mutated by applying ops (through the registry) or by
/// replaying recorded [`Delta`]s.
#[derive(Clone)]
pub struct State {
    layers: BTreeMap<String, Slot>,
    ids: IdCounters,
    derived: Arc<DerivedCache>,
}

impl State {
    pub(crate) fn new(layers: Vec<(String, Box<dyn DynLayer>)>) -> Self {
        State {
            layers: layers
                .into_iter()
                .map(|(n, layer)| {
                    (
                        n,
                        Slot {
                            layer,
                            hash: OnceLock::new(),
                        },
                    )
                })
                .collect(),
            ids: IdCounters::default(),
            derived: Arc::new(DerivedCache::new(256)),
        }
    }

    /// Names of the layers present, sorted.
    pub fn layer_names(&self) -> impl Iterator<Item = &str> {
        self.layers.keys().map(String::as_str)
    }

    /// Typed read access to a layer.
    pub fn layer<L: Layer>(&self, id: LayerId) -> Option<&L> {
        self.layers
            .get(id.as_str())?
            .layer
            .as_any()
            .downcast_ref::<L>()
    }

    pub(crate) fn has_layer(&self, name: &str) -> bool {
        self.layers.contains_key(name)
    }

    /// Content hash of one layer (cached until the layer is next patched).
    pub fn layer_hash(&self, name: &str) -> Option<Hash> {
        let slot = self.layers.get(name)?;
        Some(*slot.hash.get_or_init(|| {
            let mut h = StateHasher::new();
            h.str(name);
            slot.layer.hash_into(&mut h);
            h.finish()
        }))
    }

    /// Hash of the entire state: all layers (sorted by name) and the id counters. The
    /// derived cache is excluded by design.
    pub fn hash(&self) -> Hash {
        let mut h = StateHasher::new();
        h.str("smesh-state-v1");
        h.u64(self.layers.len() as u64);
        for name in self.layers.keys() {
            if let Some(lh) = self.layer_hash(name) {
                h.hash(&lh);
            }
        }
        h.u64(self.ids.last.len() as u64);
        for (k, v) in &self.ids.last {
            h.u64(u64::from(*k));
            h.u64(*v);
        }
        h.finish()
    }

    /// Id counters.
    pub fn ids(&self) -> &IdCounters {
        &self.ids
    }

    /// The derived cache (shared by clones of this state; keys are content hashes so
    /// sharing is always sound).
    pub fn derived(&self) -> &DerivedCache {
        &self.derived
    }

    /// Cache key covering exactly the named layers' content. Use with
    /// [`DerivedCache::get_or_compute`] so a derived product is recomputed only when one of
    /// its inputs changed.
    pub fn derived_key(&self, inputs: &[LayerId]) -> Hash {
        let mut h = StateHasher::new();
        h.str("smesh-derived-key-v1");
        for id in inputs {
            h.str(id.as_str());
            match self.layer_hash(id.as_str()) {
                Some(lh) => h.hash(&lh),
                None => h.u64(0),
            }
        }
        h.finish()
    }

    /// Apply a patch value to a layer; returns the inverse patch value.
    pub(crate) fn apply_patch(&mut self, name: &str, patch: &Value) -> Result<Value, OpError> {
        let slot = self
            .layers
            .get_mut(name)
            .ok_or_else(|| OpError::NoSuchLayer(name.to_owned()))?;
        slot.hash = OnceLock::new();
        Ok(slot.layer.apply_value(patch)?)
    }

    pub(crate) fn alloc_id(
        &mut self,
        kind: EntityKind,
    ) -> Result<(EntityId, CounterChange), OpError> {
        let k = kind as u8;
        let before = self.ids.get_raw(k);
        let after = before + 1;
        let id = EntityId::new(kind, after)
            .ok_or_else(|| OpError::invalid("entity serial space exhausted"))?;
        self.ids.set_raw(k, after);
        Ok((
            id,
            CounterChange {
                kind: k,
                before,
                after,
            },
        ))
    }

    pub(crate) fn set_counter(&mut self, kind: u8, v: u64) {
        self.ids.set_raw(kind, v);
    }

    /// Move the state forward by replaying a recorded delta (no op code runs).
    ///
    /// Fails without side effects on the *first* patch only; a failure later leaves the
    /// state partially advanced, so callers must discard it (the workspace falls back to a
    /// full op replay).
    pub fn apply_delta(&mut self, d: &Delta) -> Result<(), ApplyError> {
        for c in &d.counters {
            if self.ids.get_raw(c.kind) != c.before {
                return Err(ApplyError::Delta(format!(
                    "id counter {} is not at {}",
                    c.kind, c.before
                )));
            }
        }
        for p in &d.patches {
            self.apply_patch(&p.layer, &p.forward)
                .map_err(|e| ApplyError::Delta(e.to_string()))?;
        }
        for c in &d.counters {
            self.set_counter(c.kind, c.after);
        }
        Ok(())
    }

    /// Move the state backward by undoing a recorded delta.
    pub fn revert_delta(&mut self, d: &Delta) -> Result<(), ApplyError> {
        for c in &d.counters {
            if self.ids.get_raw(c.kind) != c.after {
                return Err(ApplyError::Delta(format!(
                    "id counter {} is not at {}",
                    c.kind, c.after
                )));
            }
        }
        for p in d.patches.iter().rev() {
            self.apply_patch(&p.layer, &p.inverse)
                .map_err(|e| ApplyError::Delta(e.to_string()))?;
        }
        for c in &d.counters {
            self.set_counter(c.kind, c.before);
        }
        Ok(())
    }
}

/// An id counter movement inside a [`Delta`].
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CounterChange {
    /// Raw `EntityKind` byte.
    pub kind: u8,
    /// Last serial before the op.
    pub before: u64,
    /// Last serial after the op.
    pub after: u64,
}

/// One layer mutation with both directions recorded.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LayerPatch {
    /// Layer name.
    pub layer: String,
    /// The patch the op applied.
    pub forward: Value,
    /// The patch that undoes it.
    pub inverse: Value,
}

/// The reversible effect of one applied op: persisted next to the op, but *derived* data.
/// Ops remain the source of truth; a missing or stale delta is regenerated by re-running
/// the op.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Delta {
    /// State hash before the op.
    pub state_before: Hash,
    /// State hash after the op (used to verify replay).
    pub state_after: Hash,
    /// Id counters that moved.
    pub counters: Vec<CounterChange>,
    /// Layer patches in application order.
    pub patches: Vec<LayerPatch>,
}

impl Delta {
    /// Canonical TOML form.
    pub fn to_toml(&self) -> Result<String, crate::value::ValueError> {
        match to_value(self)? {
            Value::Table(t) => Ok(toml::to_string(&t)),
            _ => Err(crate::value::ValueError("delta is not a table".into())),
        }
    }

    /// Parse the TOML form.
    pub fn from_toml(s: &str) -> Result<Self, DeltaParseError> {
        let t: Table = toml::from_str(s)?;
        Ok(from_value(&Value::Table(t))?)
    }
}

/// Failure parsing a persisted delta.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum DeltaParseError {
    /// Syntax error.
    #[error(transparent)]
    Toml(#[from] TomlError),
    /// Shape error.
    #[error(transparent)]
    Value(#[from] crate::value::ValueError),
}

type CacheEntry = Arc<dyn Any + Send + Sync>;

/// Cache of derived products (tessellations, quality tables, spatial indices...) keyed by
/// `(kind, content hash of the inputs)`. Never part of the state hash or of deltas; entries
/// never need invalidation because a changed input changes the key. Bounded FIFO.
pub struct DerivedCache {
    map: Mutex<IndexMap<(String, Hash), CacheEntry>>,
    capacity: usize,
    hits: AtomicU64,
    misses: AtomicU64,
}

impl DerivedCache {
    /// Cache holding at most `capacity` entries.
    pub fn new(capacity: usize) -> Self {
        DerivedCache {
            map: Mutex::new(IndexMap::new()),
            capacity: capacity.max(1),
            hits: AtomicU64::new(0),
            misses: AtomicU64::new(0),
        }
    }

    /// Return the cached product for `(kind, key)` or compute and store it.
    pub fn get_or_compute<T: Any + Send + Sync>(
        &self,
        kind: &str,
        key: Hash,
        f: impl FnOnce() -> T,
    ) -> Arc<T> {
        let k = (kind.to_owned(), key);
        let found = self.lock().get(&k).cloned();
        if let Some(a) = found.and_then(|e| e.downcast::<T>().ok()) {
            self.hits.fetch_add(1, Ordering::Relaxed);
            return a;
        }
        self.misses.fetch_add(1, Ordering::Relaxed);
        let v = Arc::new(f());
        let mut m = self.lock();
        m.insert(k, v.clone());
        while m.len() > self.capacity {
            m.shift_remove_index(0);
        }
        v
    }

    /// `(hits, misses)` since creation.
    pub fn stats(&self) -> (u64, u64) {
        (
            self.hits.load(Ordering::Relaxed),
            self.misses.load(Ordering::Relaxed),
        )
    }

    /// Number of cached entries.
    pub fn len(&self) -> usize {
        self.lock().len()
    }

    /// Whether the cache is empty.
    pub fn is_empty(&self) -> bool {
        self.lock().is_empty()
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, IndexMap<(String, Hash), CacheEntry>> {
        self.map.lock().unwrap_or_else(|e| e.into_inner())
    }
}
