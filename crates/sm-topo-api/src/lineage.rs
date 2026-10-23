//! The append-only lineage log, indexed in both directions.

use crate::types::{CapFace, CutOutcome, Lineage, LineageKind, LineageRecord, OpId};
use sm_core::EntityId;
use smallvec::SmallVec;
use std::collections::BTreeMap;

/// Append-only log of what operations did to entities, in operation order, with an index by
/// subject (what happened to X?) and by target (where did Y come from?), so both lookups are
/// `O(log n + hits)` instead of scans.
///
/// Records are ordered by operation (operation ids strictly increase), so [`Self::for_op`] is a
/// binary search.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LineageLog {
    records: Vec<LineageRecord>,
    by_subject: BTreeMap<EntityId, SmallVec<[usize; 2]>>,
    by_target: BTreeMap<EntityId, SmallVec<[usize; 2]>>,
}

fn targets(kind: &LineageKind) -> SmallVec<[EntityId; 2]> {
    match kind {
        LineageKind::Split { children } => children.clone(),
        LineageKind::Merged { into, .. } => SmallVec::from_slice(&[*into]),
        _ => SmallVec::new(),
    }
}

impl LineageLog {
    /// An empty log.
    pub fn new() -> Self {
        Self::default()
    }

    /// Append a record (updates both indexes).
    pub fn push(&mut self, rec: LineageRecord) {
        let i = self.records.len();
        self.by_subject.entry(rec.entity).or_default().push(i);
        for t in targets(&rec.kind) {
            self.by_target.entry(t).or_default().push(i);
        }
        self.records.push(rec);
    }

    /// Drop every record with index `>= len` (rollback of an aborted operation).
    pub fn truncate(&mut self, len: usize) {
        while self.records.len() > len {
            let Some(rec) = self.records.pop() else { break };
            let i = self.records.len();
            if let Some(v) = self.by_subject.get_mut(&rec.entity) {
                v.retain(|k| *k != i);
                if v.is_empty() {
                    self.by_subject.remove(&rec.entity);
                }
            }
            for t in targets(&rec.kind) {
                if let Some(v) = self.by_target.get_mut(&t) {
                    v.retain(|k| *k != i);
                    if v.is_empty() {
                        self.by_target.remove(&t);
                    }
                }
            }
        }
    }

    /// All records in log order.
    pub fn records(&self) -> &[LineageRecord] {
        &self.records
    }

    /// Number of records.
    pub fn len(&self) -> usize {
        self.records.len()
    }

    /// True if there are no records.
    pub fn is_empty(&self) -> bool {
        self.records.is_empty()
    }

    /// Records whose subject is `id`, in log order.
    pub fn of(&self, id: EntityId) -> impl Iterator<Item = &LineageRecord> + '_ {
        self.by_subject.get(&id).into_iter().flatten().map(|&i| &self.records[i])
    }

    /// Records in which `id` appears as a result (a split child or a merge survivor).
    pub fn produced(&self, id: EntityId) -> impl Iterator<Item = &LineageRecord> + '_ {
        self.by_target.get(&id).into_iter().flatten().map(|&i| &self.records[i])
    }

    /// Direct predecessors of `id`: the subjects of the records that produced it, ascending.
    pub fn parents(&self, id: EntityId) -> SmallVec<[EntityId; 2]> {
        let mut v: SmallVec<[EntityId; 2]> = self.produced(id).map(|r| r.entity).filter(|p| *p != id).collect();
        v.sort();
        v.dedup();
        v
    }

    /// Direct successors of `id`: split children and merge survivors, in log order.
    pub fn children(&self, id: EntityId) -> SmallVec<[EntityId; 2]> {
        let mut out = SmallVec::new();
        for r in self.of(id) {
            for t in targets(&r.kind) {
                if t != id && !out.contains(&t) {
                    out.push(t);
                }
            }
        }
        out
    }

    /// The end of every succession chain starting at `id`: follow splits and merges until an
    /// entity has no outgoing record. A deleted entity has no successors. `id` itself is returned
    /// if nothing replaced it. Ascending, deduplicated; the caller filters by liveness.
    pub fn successors(&self, id: EntityId) -> Vec<EntityId> {
        let mut out = Vec::new();
        let mut stack = vec![id];
        let mut seen: Vec<EntityId> = Vec::new();
        while let Some(cur) = stack.pop() {
            if seen.contains(&cur) {
                continue;
            }
            seen.push(cur);
            let next = self.children(cur);
            if next.is_empty() {
                if !self.of(cur).any(|r| matches!(r.kind, LineageKind::Deleted)) {
                    out.push(cur);
                }
            } else {
                stack.extend(next);
            }
        }
        out.sort();
        out.dedup();
        out
    }

    /// The records of one operation (empty if none).
    pub fn for_op(&self, op: OpId) -> &[LineageRecord] {
        let lo = self.records.partition_point(|r| r.op < op);
        let hi = self.records.partition_point(|r| r.op <= op);
        &self.records[lo..hi]
    }

    /// Successions and cap faces of one operation, in the shape the model layer consumes.
    pub fn cut_outcome(&self, op: OpId) -> CutOutcome {
        let mut out = CutOutcome::default();
        for r in self.for_op(op) {
            match &r.kind {
                LineageKind::Split { children } => out.lineage.push(Lineage { old: r.entity, new: children.clone() }),
                LineageKind::Merged { into, .. } => out.lineage.push(Lineage { old: r.entity, new: SmallVec::from_slice(&[*into]) }),
                LineageKind::Deleted => out.lineage.push(Lineage { old: r.entity, new: SmallVec::new() }),
                LineageKind::Generated { from_surface } => out.caps.push(CapFace { face: r.entity, from_surface: *from_surface }),
                LineageKind::Created | LineageKind::Modified(_) => {}
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_core::EntityKind;

    fn f(n: u64) -> EntityId {
        EntityId::new(EntityKind::TopoFace, n).unwrap()
    }

    #[test]
    fn indexes_both_ways_and_truncates() {
        let mut log = LineageLog::new();
        log.push(LineageRecord { op: OpId(1), entity: f(3), kind: LineageKind::Split { children: SmallVec::from_slice(&[f(10), f(11)]) } });
        log.push(LineageRecord { op: OpId(2), entity: f(10), kind: LineageKind::Split { children: SmallVec::from_slice(&[f(12), f(13)]) } });
        log.push(LineageRecord { op: OpId(2), entity: f(11), kind: LineageKind::Merged { into: f(12), gap: 1e-7 } });
        log.push(LineageRecord { op: OpId(3), entity: f(13), kind: LineageKind::Deleted });
        assert_eq!(log.parents(f(12)).as_slice(), &[f(10), f(11)]);
        assert_eq!(log.children(f(3)).as_slice(), &[f(10), f(11)]);
        assert_eq!(log.successors(f(3)), vec![f(12)], "13 was deleted; 11 merged into 12");
        assert_eq!(log.for_op(OpId(2)).len(), 2);
        assert_eq!(log.for_op(OpId(9)).len(), 0);
        let co = log.cut_outcome(OpId(2));
        assert_eq!(co.lineage.len(), 2);
        log.truncate(2);
        assert_eq!(log.len(), 2);
        assert!(log.produced(f(12)).count() == 1 && log.of(f(11)).count() == 0);
        log.truncate(0);
        assert!(log.is_empty() && log.produced(f(10)).count() == 0);
    }
}
