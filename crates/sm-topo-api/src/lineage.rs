//! Lineage: what happened to each entity (ARCHITECTURE §3, persistent naming P0-P1).
//!
//! Only ids and recorded events. There are no selectors; those are P2 and build on this log.

use crate::ids::OpId;
use sm_core::EntityId;

/// One thing that happened to entities.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum LineageEvent {
    /// `entity` was created.
    Created {
        /// The new entity.
        entity: EntityId,
    },
    /// `parent` was replaced by `children` (retired, not reused).
    Split {
        /// The retired entity.
        parent: EntityId,
        /// Replacements, in canonical order.
        children: Vec<EntityId>,
    },
    /// `absorbed` was merged into `survivor`. `gap` is the measured deviation between the two
    /// geometries that was discarded (audit trail for the global-tolerance model).
    Merged {
        /// Entity that keeps existing.
        survivor: EntityId,
        /// Entity that was retired.
        absorbed: EntityId,
        /// Max measured deviation between the two.
        gap: f64,
    },
    /// `entity` was removed.
    Deleted {
        /// The removed entity.
        entity: EntityId,
    },
}

/// A lineage event with its op and position.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LineageRecord {
    /// The op during which it happened.
    pub op: OpId,
    /// Position within the op (0-based), so order is total.
    pub seq: u32,
    /// What happened.
    pub event: LineageEvent,
}

/// Append-only lineage log with query helpers.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LineageLog {
    records: Vec<LineageRecord>,
}

impl LineageLog {
    /// Empty log.
    pub fn new() -> Self {
        Self::default()
    }

    /// Append an event for `op`; `seq` is assigned automatically.
    pub fn push(&mut self, op: OpId, event: LineageEvent) {
        let seq = self.records.iter().rev().take_while(|r| r.op == op).count() as u32;
        self.records.push(LineageRecord { op, seq, event });
    }

    /// All records in order.
    pub fn records(&self) -> &[LineageRecord] {
        &self.records
    }

    /// Direct replacements of `id`: split children, or the merge survivor.
    pub fn successors(&self, id: EntityId) -> Vec<EntityId> {
        let mut out = Vec::new();
        for r in &self.records {
            match &r.event {
                LineageEvent::Split { parent, children } if *parent == id => out.extend(children.iter().copied()),
                LineageEvent::Merged { survivor, absorbed, .. } if *absorbed == id => out.push(*survivor),
                _ => {}
            }
        }
        out
    }

    /// The live entities that `id` has become: follows splits and merges transitively.
    /// `[id]` if untouched, empty if deleted. Result is sorted and deduplicated.
    pub fn current_of(&self, id: EntityId) -> Vec<EntityId> {
        let mut out = Vec::new();
        let mut stack = vec![id];
        let mut seen = Vec::new();
        while let Some(e) = stack.pop() {
            if seen.contains(&e) {
                continue;
            }
            seen.push(e);
            let next = self.successors(e);
            if next.is_empty() {
                if !self.is_deleted(e) {
                    out.push(e);
                }
            } else {
                stack.extend(next);
            }
        }
        out.sort();
        out.dedup();
        out
    }

    /// Entities that `id` came from (split parents, merge absorbed partners), transitively.
    pub fn ancestors_of(&self, id: EntityId) -> Vec<EntityId> {
        let mut out: Vec<EntityId> = Vec::new();
        let mut stack = vec![id];
        while let Some(e) = stack.pop() {
            for r in &self.records {
                match &r.event {
                    LineageEvent::Split { parent, children } if children.contains(&e) => push_new(&mut out, &mut stack, *parent),
                    LineageEvent::Merged { survivor, absorbed, .. } if *survivor == e => push_new(&mut out, &mut stack, *absorbed),
                    _ => {}
                }
            }
        }
        out.sort();
        out
    }

    /// True if `id` was deleted (and not merely replaced).
    pub fn is_deleted(&self, id: EntityId) -> bool {
        self.records.iter().any(|r| matches!(&r.event, LineageEvent::Deleted { entity } if *entity == id))
    }

    /// All records that mention `id`, in order.
    pub fn history_of(&self, id: EntityId) -> Vec<&LineageRecord> {
        self.records
            .iter()
            .filter(|r| match &r.event {
                LineageEvent::Created { entity } | LineageEvent::Deleted { entity } => *entity == id,
                LineageEvent::Split { parent, children } => *parent == id || children.contains(&id),
                LineageEvent::Merged { survivor, absorbed, .. } => *survivor == id || *absorbed == id,
            })
            .collect()
    }
}

fn push_new(out: &mut Vec<EntityId>, stack: &mut Vec<EntityId>, id: EntityId) {
    if !out.contains(&id) {
        out.push(id);
        stack.push(id);
    }
}
