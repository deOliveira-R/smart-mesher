//! Deterministic id allocation for entities created by this layer.

use sm_core::{EntityId, EntityKind};
use std::collections::BTreeMap;

/// Source of fresh [`EntityId`]s. The engine supplies the real one (ids are assigned per operation
/// in canonical order); [`SequentialIds`] is the reference implementation.
pub trait IdAllocator {
    /// Allocate the next id of `kind`. Ids are never reused.
    ///
    /// Panics only if the 56-bit serial space of one kind is exhausted.
    fn alloc(&mut self, kind: EntityKind) -> EntityId;
}

/// Per-kind counters starting at 1. Deterministic by construction.
#[derive(Clone, Debug, Default)]
pub struct SequentialIds {
    next: BTreeMap<EntityKind, u64>,
}

impl SequentialIds {
    /// A fresh allocator.
    pub fn new() -> Self {
        Self::default()
    }
}

impl IdAllocator for SequentialIds {
    fn alloc(&mut self, kind: EntityKind) -> EntityId {
        let n = self.next.entry(kind).or_insert(1);
        let serial = *n;
        *n += 1;
        match EntityId::new(kind, serial) {
            Some(id) => id,
            None => panic!("serial space exhausted for {kind:?}"),
        }
    }
}
