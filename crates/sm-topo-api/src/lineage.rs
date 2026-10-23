//! Lineage: what happened to each entity, recorded per operation.
//!
//! Scope (ARCHITECTURE §3): P0-P1 only. This is a record of *what happened*, enough to
//! replay and to answer "where did entity X go". There are no selectors here.

use crate::data::IdList;
use crate::ids::OpId;
use indexmap::IndexMap;
use smallvec::SmallVec;
use sm_core::EntityId;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// What an operation did to some entities.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum LineageKind {
    /// A new entity appeared with no ancestor.
    Created {
        /// The new entity.
        entity: EntityId,
    },
    /// New entities generated from an existing one that stays alive (extrude: edge -> face).
    Generated {
        /// The generating entity.
        from: EntityId,
        /// The generated entities.
        into: IdList,
    },
    /// `from` was retired and replaced by `into`, in the canonical order documented by the
    /// splitting operation.
    Split {
        /// The retired entity.
        from: EntityId,
        /// Its pieces.
        into: IdList,
    },
    /// `from` entities were merged; `into` is the survivor (always one of `from`, the lowest id).
    Merged {
        /// All merged entities, ascending by id.
        from: IdList,
        /// The surviving entity.
        into: EntityId,
    },
    /// An entity's data changed in place (tolerance raised).
    Modified {
        /// The modified entity.
        entity: EntityId,
    },
    /// An entity was removed without replacement.
    Deleted {
        /// The removed entity.
        entity: EntityId,
    },
}

/// One lineage entry.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct LineageRecord {
    /// The operation that did it.
    pub op: OpId,
    /// What it did.
    pub kind: LineageKind,
}

/// Append-only lineage log with a forward index (entity -> records that retire it).
#[derive(Clone, Debug, Default)]
pub struct LineageLog {
    records: Vec<LineageRecord>,
    /// retired entity -> indices into `records` (Split/Merged/Deleted with that `from`).
    retired_by: IndexMap<EntityId, SmallVec<[u32; 2]>>,
}

impl LineageLog {
    /// Empty log.
    pub fn new() -> Self {
        Self::default()
    }

    /// Rebuild a log (and its index) from stored records.
    pub fn from_records(records: Vec<LineageRecord>) -> Self {
        let mut log = Self::default();
        for r in records {
            log.push(r);
        }
        log
    }

    /// All records in the order they were made.
    pub fn records(&self) -> &[LineageRecord] {
        &self.records
    }

    /// Append a record.
    pub fn push(&mut self, r: LineageRecord) {
        let idx = self.records.len() as u32;
        match &r.kind {
            LineageKind::Split { from, .. } | LineageKind::Deleted { entity: from } => {
                self.retired_by.entry(*from).or_default().push(idx);
            }
            LineageKind::Merged { from, into } => {
                for f in from.iter().filter(|f| *f != into) {
                    self.retired_by.entry(*f).or_default().push(idx);
                }
            }
            _ => {}
        }
        self.records.push(r);
    }

    /// True if some record retired `id` (split it, merged it away, or deleted it).
    pub fn is_retired(&self, id: EntityId) -> bool {
        self.retired_by.contains_key(&id)
    }

    /// The live entities `id` became: follows split/merge chains transitively. Empty if it was
    /// deleted; `[id]` if it was never retired. Ascending, de-duplicated.
    pub fn successors(&self, id: EntityId) -> Vec<EntityId> {
        let mut out = Vec::new();
        let mut stack = vec![id];
        let mut guard = 0usize;
        while let Some(cur) = stack.pop() {
            guard += 1;
            if guard > 1_000_000 {
                break;
            }
            match self.retired_by.get(&cur) {
                None => out.push(cur),
                Some(ixs) => {
                    for &i in ixs {
                        match &self.records[i as usize].kind {
                            LineageKind::Split { into, .. } => stack.extend(into.iter().copied()),
                            LineageKind::Merged { into, .. } => stack.push(*into),
                            _ => {}
                        }
                    }
                }
            }
        }
        out.sort();
        out.dedup();
        out
    }

    /// The entities `id` came from (one step): the `from` of the split/merge/generate record
    /// that produced it. Ascending, de-duplicated.
    pub fn parents(&self, id: EntityId) -> Vec<EntityId> {
        let mut out = Vec::new();
        for r in &self.records {
            match &r.kind {
                LineageKind::Split { from, into } | LineageKind::Generated { from, into } if into.contains(&id) => out.push(*from),
                LineageKind::Merged { from, into } if *into == id => out.extend(from.iter().copied().filter(|f| *f != id)),
                _ => {}
            }
        }
        out.sort();
        out.dedup();
        out
    }
}
