//! The composable [`State`]: independent layer stores plus the entity table.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use sm_core::{EntityId, EntityKind};

use crate::error::OpError;
use crate::hash::{Hash256, Sha256};
use crate::layer::{Digest, Layer, LayerRole, LayerType};
use crate::toml::to_toml;
use crate::value::Value;

/// The whole project state at one timeline node.
///
/// Cloning is cheap: layers are `Arc`-shared and copied on first write, so a clone is both a
/// transaction scratchpad and a snapshot. Mutation is only possible through
/// [`ApplyCtx`](crate::ApplyCtx); readers use [`layer`](Self::layer).
#[derive(Clone, Default)]
pub struct State {
    layers: BTreeMap<&'static str, Arc<dyn Layer>>,
    /// Live entities. Serials of retired entities are never reused (`next_serial`).
    entities: BTreeSet<EntityId>,
    next_serial: BTreeMap<EntityKind, u64>,
}

impl State {
    pub(crate) fn with_layers(layers: BTreeMap<&'static str, Arc<dyn Layer>>) -> Self {
        Self { layers, ..Self::default() }
    }

    /// Read a layer by type. `None` if no such layer is registered.
    pub fn layer<L: LayerType>(&self) -> Option<&L> {
        self.layers.get(L::NAME)?.as_any().downcast_ref::<L>()
    }

    /// Registered layer names, sorted.
    pub fn layer_names(&self) -> impl Iterator<Item = &'static str> + '_ {
        self.layers.keys().copied()
    }

    /// Content hash of one layer by name.
    pub fn layer_hash(&self, name: &str) -> Option<Hash256> {
        self.layers.get(name).map(|l| l.content_hash())
    }

    /// Copy-on-write access to a layer.
    pub(crate) fn layer_mut<L: LayerType>(&mut self) -> Option<&mut L> {
        let slot = self.layers.get_mut(L::NAME)?;
        if Arc::get_mut(slot).is_none() {
            *slot = Arc::from(slot.clone_box());
        }
        Arc::get_mut(slot)?.as_any_mut().downcast_mut::<L>()
    }

    /// Whether `id` is a live entity.
    pub fn contains(&self, id: EntityId) -> bool {
        self.entities.contains(&id)
    }

    /// Live entities in id order.
    pub fn entities(&self) -> impl Iterator<Item = EntityId> + '_ {
        self.entities.iter().copied()
    }

    /// Number of live entities.
    pub fn entity_count(&self) -> usize {
        self.entities.len()
    }

    /// The id the next `ApplyCtx::alloc` of `kind` would return.
    pub fn peek_next(&self, kind: EntityKind) -> Option<EntityId> {
        EntityId::new(kind, self.next_serial.get(&kind).copied().unwrap_or(1))
    }

    pub(crate) fn alloc(&mut self, kind: EntityKind) -> Result<EntityId, OpError> {
        let id = self.peek_next(kind).ok_or(OpError::IdExhausted)?;
        self.next_serial.insert(kind, id.serial() + 1);
        self.entities.insert(id);
        Ok(id)
    }

    pub(crate) fn retire(&mut self, id: EntityId) -> bool {
        self.entities.remove(&id)
    }

    /// Identity of the state: entity table, serial counters and every
    /// [`Authoritative`](LayerRole::Authoritative) layer in name order. Derived layers do not
    /// contribute, so caches never change the hash.
    pub fn hash(&self) -> Hash256 {
        let mut d = Digest::new();
        d.str("smart-mesh/state/v1");
        d.u64(self.entities.len() as u64);
        for e in &self.entities {
            d.u64(e.to_raw());
        }
        d.u64(self.next_serial.len() as u64);
        for (k, n) in &self.next_serial {
            d.u64(*k as u64).u64(*n);
        }
        for (name, l) in &self.layers {
            if l.role() == LayerRole::Authoritative {
                d.str(name).hash(&l.content_hash());
            }
        }
        d.finish()
    }

    /// Key for a derived-cache entry: hash of the cache `kind`, the current content of the
    /// named input layers, and extra parameters. Equal inputs give equal keys on any replay.
    /// Returns `None` if an input layer is not registered.
    pub fn cache_key(&self, kind: &str, inputs: &[&str], params: &Value) -> Option<Hash256> {
        let mut h = Sha256::new();
        h.update(kind.as_bytes());
        for name in inputs {
            h.update(&[0]);
            h.update(name.as_bytes());
            h.update(&self.layer_hash(name)?.0);
        }
        h.update(&[1]);
        let wrapped = BTreeMap::from([("p".to_owned(), params.clone())]);
        h.update(to_toml(&wrapped).as_bytes());
        Some(h.finish())
    }
}

impl std::fmt::Debug for State {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("State")
            .field("layers", &self.layers.keys().collect::<Vec<_>>())
            .field("entities", &self.entities.len())
            .field("hash", &self.hash())
            .finish()
    }
}
