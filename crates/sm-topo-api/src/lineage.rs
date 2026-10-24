//! The append-only lineage log, indexed in both directions.

use crate::types::{CapFace, CutOutcome, Lineage, LineageKind, LineageRecord, OpId};
use sm_core::EntityId;
use smallvec::SmallVec;
use std::collections::BTreeMap;

/// Serials below this are indexed densely; larger ones (never produced by the stores, which
/// allocate serials from 1) go to a sorted overflow map so a stray huge serial cannot allocate.
const DENSE_MAX: u64 = 1 << 22;

type Hits = SmallVec<[usize; 2]>;

/// Record indices per entity: a dense `Vec` per entity kind indexed by serial (grown on demand),
/// plus an overflow map for serials `>= DENSE_MAX`.
#[derive(Clone, Debug, Default)]
struct Index {
    dense: Vec<Vec<Hits>>,
    sparse: BTreeMap<EntityId, Hits>,
}

impl Index {
    fn get(&self, id: EntityId) -> &[usize] {
        let serial = id.serial();
        if serial < DENSE_MAX {
            self.dense.get(id.kind() as usize).and_then(|t| t.get(serial as usize)).map_or(&[], |h| h.as_slice())
        } else {
            self.sparse.get(&id).map_or(&[], |h| h.as_slice())
        }
    }

    fn slot(&mut self, id: EntityId) -> &mut Hits {
        let serial = id.serial();
        if serial < DENSE_MAX {
            let k = id.kind() as usize;
            if self.dense.len() <= k {
                self.dense.resize_with(k + 1, Vec::new);
            }
            let t = &mut self.dense[k];
            if t.len() <= serial as usize {
                t.resize_with(serial as usize + 1, Hits::new);
            }
            &mut t[serial as usize]
        } else {
            self.sparse.entry(id).or_default()
        }
    }

    fn push(&mut self, id: EntityId, i: usize) {
        self.slot(id).push(i);
    }

    /// Remove record `i` (the last one) from `id`'s hits.
    fn unpush(&mut self, id: EntityId, i: usize) {
        let serial = id.serial();
        if serial < DENSE_MAX {
            if let Some(h) = self.dense.get_mut(id.kind() as usize).and_then(|t| t.get_mut(serial as usize)) {
                h.retain(|k| *k != i);
            }
        } else if let Some(h) = self.sparse.get_mut(&id) {
            h.retain(|k| *k != i);
            if h.is_empty() {
                self.sparse.remove(&id);
            }
        }
    }
}

/// Append-only log of what operations did to entities, in operation order, with an index by
/// subject (what happened to X?) and by target (where did Y come from?), so both lookups are
/// `O(1 + hits)` instead of scans.
///
/// Records are ordered by operation (operation ids strictly increase), so [`Self::for_op`] is a
/// binary search. Equality compares the records only (the indexes are derived from them).
#[derive(Clone, Debug, Default)]
pub struct LineageLog {
    records: Vec<LineageRecord>,
    by_subject: Index,
    by_target: Index,
}

impl PartialEq for LineageLog {
    fn eq(&self, other: &Self) -> bool {
        self.records == other.records
    }
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
        self.by_subject.push(rec.entity, i);
        for t in targets(&rec.kind) {
            self.by_target.push(t, i);
        }
        self.records.push(rec);
    }

    /// Drop every record with index `>= len` (rollback of an aborted operation).
    pub fn truncate(&mut self, len: usize) {
        while self.records.len() > len {
            let Some(rec) = self.records.pop() else { break };
            let i = self.records.len();
            self.by_subject.unpush(rec.entity, i);
            for t in targets(&rec.kind) {
                self.by_target.unpush(t, i);
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
        self.by_subject.get(id).iter().map(|&i| &self.records[i])
    }

    /// Records in which `id` appears as a result (a split child or a merge survivor).
    pub fn produced(&self, id: EntityId) -> impl Iterator<Item = &LineageRecord> + '_ {
        self.by_target.get(id).iter().map(|&i| &self.records[i])
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
        let records = self.for_op(op);
        // A composite op kills a cell and then records its split: the Split/Merged record
        // supersedes the Deleted one, so each entity yields one row.
        let replaced = |e: EntityId| records.iter().any(|r| r.entity == e && matches!(r.kind, LineageKind::Split { .. } | LineageKind::Merged { .. }));
        for r in records {
            match &r.kind {
                LineageKind::Deleted if replaced(r.entity) => {}
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
        // Deleted then Split for the same entity in one op: one row, the split.
        let mut log2 = LineageLog::new();
        log2.push(LineageRecord { op: OpId(1), entity: f(0), kind: LineageKind::Deleted });
        log2.push(LineageRecord { op: OpId(1), entity: f(0), kind: LineageKind::Split { children: SmallVec::from_slice(&[f(1)]) } });
        let co2 = log2.cut_outcome(OpId(1));
        assert_eq!(co2.lineage, vec![Lineage { old: f(0), new: SmallVec::from_slice(&[f(1)]) }]);
        assert_eq!(log2.successors(f(0)), vec![f(1)]);
        log.truncate(2);
        assert_eq!(log.len(), 2);
        assert!(log.produced(f(12)).count() == 1 && log.of(f(11)).count() == 0);
        log.truncate(0);
        assert!(log.is_empty() && log.produced(f(10)).count() == 0);
    }

    /// Brute-force oracle: the old scan semantics over the record list.
    fn scan_of(log: &LineageLog, id: EntityId) -> Vec<&LineageRecord> {
        log.records().iter().filter(|r| r.entity == id).collect()
    }

    fn scan_produced(log: &LineageLog, id: EntityId) -> Vec<&LineageRecord> {
        // A record listing `id` twice (a split with a repeated child) is returned twice.
        log.records().iter().flat_map(|r| targets(&r.kind).into_iter().filter(|t| *t == id).map(move |_| r)).collect()
    }

    #[test]
    fn dense_index_matches_a_scan_across_kinds_push_and_truncate() {
        // A seeded pseudo-random history over three kinds, a huge serial (overflow map) and
        // truncations; queries agree with a linear scan and with a determinism hash of the answers.
        let ids: Vec<EntityId> = [EntityKind::TopoFace, EntityKind::TopoEdge, EntityKind::TopoVolume]
            .iter()
            .flat_map(|&k| (1..9u64).map(move |n| EntityId::new(k, n).unwrap()))
            .chain([EntityId::new(EntityKind::TopoFace, DENSE_MAX + 5).unwrap(), EntityId::new(EntityKind::TopoVertex, 1 << 40).unwrap()])
            .collect();
        let mut x = 0x9e37_79b9_7f4a_7c15u64;
        let mut next = move |n: usize| {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            (x % n as u64) as usize
        };
        let mut log = LineageLog::new();
        let mut digest = 0u64;
        for step in 0..400usize {
            if step % 37 == 36 {
                let keep = next(log.len() + 1);
                log.truncate(keep);
            } else {
                let kind = match next(3) {
                    0 => LineageKind::Deleted,
                    1 => LineageKind::Split { children: SmallVec::from_slice(&[ids[next(ids.len())], ids[next(ids.len())]]) },
                    _ => LineageKind::Merged { into: ids[next(ids.len())], gap: 0.0 },
                };
                log.push(LineageRecord { op: OpId(step as u64 / 3 + 1), entity: ids[next(ids.len())], kind });
            }
            for &id in &ids {
                let (a, b) = (log.of(id).collect::<Vec<_>>(), scan_of(&log, id));
                assert_eq!(a, b, "of({id:?}) at step {step}");
                let (a, b) = (log.produced(id).collect::<Vec<_>>(), scan_produced(&log, id));
                assert_eq!(a, b, "produced({id:?}) at step {step}");
                digest = digest.wrapping_mul(0x100_0000_01b3) ^ (a.len() as u64 + 31 * log.of(id).count() as u64);
            }
        }
        // Pinned: the answers of this fixed history never change with the index layout.
        assert_eq!(digest, 15_108_247_996_424_554_546, "determinism digest");
    }
}
