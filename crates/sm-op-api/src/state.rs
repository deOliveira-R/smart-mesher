//! The composable [`State`]: independent layer stores plus the entity table.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, OnceLock};

use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};

use crate::codec::{CodecError, Value, canonical_bytes};
use crate::error::OpError;
use crate::hash::{Hash256, Hasher, MultisetHash};
use crate::layer::{Layer, LayerRole, LayerType};

/// One layer plus its memoized content hash (cleared whenever the layer is borrowed mutably).
#[derive(Clone)]
struct Slot {
    layer: Arc<dyn Layer>,
    hash: OnceLock<Hash256>,
}

impl Slot {
    fn new(layer: Arc<dyn Layer>) -> Self {
        Self { layer, hash: OnceLock::new() }
    }

    fn hash(&self) -> Hash256 {
        *self.hash.get_or_init(|| self.layer.content_hash())
    }

    /// Unique mutable access (copy-on-write), invalidating the memo.
    fn make_mut(&mut self) -> &mut dyn Layer {
        self.hash = OnceLock::new();
        if Arc::get_mut(&mut self.layer).is_none() {
            self.layer = Arc::from(self.layer.clone_box());
        }
        // Just made unique above.
        Arc::get_mut(&mut self.layer).expect("unique after copy-on-write")
    }
}

/// Allocation state of one entity kind. Live set = `1..next` minus `retired`, so allocating a
/// million mesh cells costs nothing but a counter bump.
#[derive(Clone, Default)]
struct KindTable {
    next: u64,
    retired: BTreeSet<u64>,
    retired_acc: MultisetHash,
}

#[derive(Clone, Default)]
struct Entities {
    kinds: BTreeMap<EntityKind, KindTable>,
}

/// Persisted form of the allocation state of one entity kind.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct KindSnapshot {
    /// The entity kind.
    pub kind: EntityKind,
    /// Next serial to allocate.
    pub next: u64,
    /// Retired serials, ascending.
    pub retired: Vec<u64>,
}

/// A persistable copy of a [`State`], produced by [`State::snapshot`] and consumed by
/// `Registry::restore`. Only layers that implement the persisted-snapshot hook appear; derived
/// layers are omitted (rebuildable).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct StateSnapshot {
    /// Entity allocation state.
    pub kinds: Vec<KindSnapshot>,
    /// Layer name to snapshot bytes.
    pub layers: BTreeMap<String, Vec<u8>>,
}

/// The whole project state at one timeline node.
///
/// Cloning is cheap: layers and the entity table are `Arc`-shared and copied on first write, so
/// a clone is both a transaction scratchpad and a snapshot. Mutation is only possible through
/// [`ApplyCtx`](crate::ApplyCtx); readers use [`layer`](Self::layer).
#[derive(Clone, Default)]
pub struct State {
    layers: BTreeMap<&'static str, Slot>,
    entities: Arc<Entities>,
}

impl State {
    pub(crate) fn with_layers(layers: BTreeMap<&'static str, Arc<dyn Layer>>) -> Self {
        Self { layers: layers.into_iter().map(|(k, v)| (k, Slot::new(v))).collect(), entities: Arc::default() }
    }

    pub(crate) fn replace_layer(&mut self, name: &'static str, layer: Arc<dyn Layer>) {
        self.layers.insert(name, Slot::new(layer));
    }

    /// Read a layer by type. `None` if no such layer is registered.
    pub fn layer<L: LayerType>(&self) -> Option<&L> {
        self.layers.get(L::NAME)?.layer.as_any().downcast_ref::<L>()
    }

    /// Registered layer names, sorted.
    pub fn layer_names(&self) -> impl Iterator<Item = &'static str> + '_ {
        self.layers.keys().copied()
    }

    /// Content hash of one layer by name (memoized until the layer is next mutated).
    pub fn layer_hash(&self, name: &str) -> Result<Hash256, OpError> {
        self.layers.get(name).map(Slot::hash).ok_or_else(|| OpError::MissingLayer(name.to_owned()))
    }

    /// Copy-on-write access to a layer; clears its memoized hash.
    pub(crate) fn layer_mut<L: LayerType>(&mut self) -> Option<&mut L> {
        let slot = self.layers.get_mut(L::NAME)?;
        if !slot.layer.as_any().is::<L>() {
            return None;
        }
        slot.make_mut().as_any_mut().downcast_mut::<L>()
    }

    /// Run `on_retired` on non-empty authoritative layers in `order`. Returns the layers touched.
    pub(crate) fn notify_retired(&mut self, order: &[&'static str], ids: &[EntityId]) -> Vec<&'static str> {
        let mut touched = Vec::new();
        for name in order {
            if let Some(slot) = self.layers.get_mut(name)
                && slot.layer.role() == LayerRole::Authoritative
                && !slot.layer.is_empty()
            {
                slot.make_mut().on_retired(ids);
                touched.push(*name);
            }
        }
        touched
    }

    /// Whether `id` is a live entity.
    pub fn contains(&self, id: EntityId) -> bool {
        self.entities
            .kinds
            .get(&id.kind())
            .is_some_and(|t| id.serial() >= 1 && id.serial() < t.next && !t.retired.contains(&id.serial()))
    }

    /// The kind of `id` if it is live (the kind is also encoded in the id itself).
    pub fn kind_of(&self, id: EntityId) -> Option<EntityKind> {
        self.contains(id).then(|| id.kind())
    }

    /// Live entities in id order.
    pub fn entities(&self) -> impl Iterator<Item = EntityId> + '_ {
        self.entities.kinds.iter().flat_map(|(kind, t)| {
            (1..t.next).filter(|s| !t.retired.contains(s)).filter_map(move |s| EntityId::new(*kind, s))
        })
    }

    /// Number of live entities.
    pub fn entity_count(&self) -> usize {
        self.entities.kinds.values().map(|t| (t.next.saturating_sub(1) as usize) - t.retired.len()).sum()
    }

    /// The id the next `ApplyCtx::alloc` of `kind` would return.
    pub fn peek_next(&self, kind: EntityKind) -> Option<EntityId> {
        let n = self.entities.kinds.get(&kind).map_or(1, |t| t.next.max(1));
        EntityId::new(kind, n)
    }

    /// Reserve `n` consecutive serials; returns the first.
    pub(crate) fn alloc_range(&mut self, kind: EntityKind, n: u64) -> Result<EntityId, OpError> {
        let first = self.peek_next(kind).ok_or(OpError::IdExhausted)?;
        let end = first.serial().checked_add(n).ok_or(OpError::IdExhausted)?;
        if n > 0 && end - 1 > EntityId::MAX_SERIAL {
            return Err(OpError::IdExhausted);
        }
        if n > 0 {
            Arc::make_mut(&mut self.entities).kinds.entry(kind).or_default().next = end;
        }
        Ok(first)
    }

    pub(crate) fn alloc(&mut self, kind: EntityKind) -> Result<EntityId, OpError> {
        self.alloc_range(kind, 1)
    }

    pub(crate) fn retire(&mut self, id: EntityId) -> bool {
        if !self.contains(id) {
            return false;
        }
        let t = Arc::make_mut(&mut self.entities).kinds.entry(id.kind()).or_default();
        t.retired.insert(id.serial());
        t.retired_acc.add(&id.serial().to_le_bytes());
        true
    }

    /// Identity of the state: allocation state and every non-empty
    /// [`Authoritative`](LayerRole::Authoritative) layer in name order. Derived and empty layers
    /// do not contribute. O(kinds + layers) given memoized layer hashes; independent of platform
    /// as long as layers quantize floats in `digest`.
    pub fn hash(&self) -> Hash256 {
        let mut h = Hasher::new("smart-mesh/state/v1");
        for (kind, t) in &self.entities.kinds {
            if t.next == 0 {
                continue;
            }
            h.update(&[*kind as u8]);
            h.update(&t.next.to_le_bytes());
            h.update(&(t.retired.len() as u64).to_le_bytes());
            h.update(&t.retired_acc.bytes());
        }
        h.update(&[0xFF]);
        for (name, slot) in &self.layers {
            if slot.layer.role() == LayerRole::Authoritative && !slot.layer.is_empty() {
                h.update(&(name.len() as u64).to_le_bytes());
                h.update(name.as_bytes());
                h.update(&slot.hash().0);
            }
        }
        h.finish()
    }

    /// Key for a derived-cache entry: hash of the cache `kind`, the current content hash of each
    /// named input layer, and extra parameters. Equal inputs give equal keys on any replay.
    /// Fails with [`OpError::MissingLayer`] on an unknown input name.
    pub fn cache_key(&self, kind: &str, inputs: &[&str], params: &Value) -> Result<Hash256, OpError> {
        let mut h = Hasher::new("smart-mesh/cache-key/v1");
        h.update(&(kind.len() as u64).to_le_bytes());
        h.update(kind.as_bytes());
        h.update(&(inputs.len() as u64).to_le_bytes());
        for name in inputs {
            h.update(&(name.len() as u64).to_le_bytes());
            h.update(name.as_bytes());
            h.update(&self.layer_hash(name)?.0);
        }
        h.update(&canonical_bytes(params));
        Ok(h.finish())
    }

    /// Persistable copy of this state. Fails if a non-empty authoritative layer does not
    /// implement the persisted-snapshot hook ([`LayerType::snapshot`]).
    pub fn snapshot(&self) -> Result<StateSnapshot, CodecError> {
        let kinds = self
            .entities
            .kinds
            .iter()
            .filter(|(_, t)| t.next != 0)
            .map(|(k, t)| KindSnapshot { kind: *k, next: t.next, retired: t.retired.iter().copied().collect() })
            .collect();
        let mut layers = BTreeMap::new();
        for (name, slot) in &self.layers {
            if slot.layer.role() != LayerRole::Authoritative || slot.layer.is_empty() {
                continue;
            }
            let bytes = slot
                .layer
                .snapshot()?
                .ok_or_else(|| CodecError(format!("layer '{name}' has no persisted snapshot support")))?;
            layers.insert((*name).to_owned(), bytes);
        }
        Ok(StateSnapshot { kinds, layers })
    }

    /// Install allocation state from a snapshot (layers are restored by the registry).
    pub(crate) fn restore_entities(&mut self, kinds: &[KindSnapshot]) -> Result<(), CodecError> {
        let mut e = Entities::default();
        for k in kinds {
            let mut t = KindTable { next: k.next, ..KindTable::default() };
            if k.next == 0 || k.next - 1 > EntityId::MAX_SERIAL {
                return Err(CodecError::new("snapshot: bad next serial"));
            }
            for &s in &k.retired {
                if s == 0 || s >= k.next || !t.retired.insert(s) {
                    return Err(CodecError::new("snapshot: bad retired serial"));
                }
                t.retired_acc.add(&s.to_le_bytes());
            }
            if e.kinds.insert(k.kind, t).is_some() {
                return Err(CodecError::new("snapshot: duplicate kind"));
            }
        }
        self.entities = Arc::new(e);
        Ok(())
    }
}

impl std::fmt::Debug for State {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("State")
            .field("layers", &self.layers.keys().collect::<Vec<_>>())
            .field("entities", &self.entity_count())
            .field("hash", &self.hash())
            .finish()
    }
}
