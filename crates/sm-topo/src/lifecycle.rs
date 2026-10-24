//! Operation lifecycle: `begin_op`, `end_op` (normalization, local checks, radial rebuild,
//! summary) and `abort_op`.

use crate::store::{OpState, TopoStore, Undo};
use rayon::prelude::*;
use smallvec::SmallVec;
use sm_core::{EntityId, EntityKind};
use sm_topo_api::{Dim, EntityState, OpId, OpSummary, TopoError, TopologyView};

/// Dirty edges at or above this count are rebuilt in parallel.
const PAR_RADIAL_MIN: usize = 256;

impl TopoStore {
    pub(crate) fn start_op(&mut self, op: OpId) -> Result<(), TopoError> {
        if self.open.is_some() {
            return Err(TopoError::OpAlreadyOpen);
        }
        if self.ops_run > 0 && op <= self.revision {
            return Err(TopoError::OpNotIncreasing { previous: self.revision.0, got: op.0 });
        }
        self.journal.clear();
        self.jdata.clear();
        self.dirty_edges.clear();
        self.orient_dirty.clear();
        self.closure_edges.clear();
        self.merge_gap = None;
        self.open = Some(OpState { op, prev_revision: self.revision, start: self.mark() });
        self.revision = op;
        Ok(())
    }

    pub(crate) fn cancel_op(&mut self) {
        if let Some(st) = self.open.take() {
            self.rollback(st.start);
            self.revision = st.prev_revision;
            self.merge_gap = None;
            self.dirty_edges.clear();
            self.orient_dirty.clear();
            self.closure_edges.clear();
            self.journal.clear();
            self.jdata.clear();
        }
    }

    /// Ids named by journal records of kinds Alloc / Splice / State / Carrier, ascending.
    fn touched_ids(&self) -> Vec<EntityId> {
        let mut v: Vec<EntityId> = Vec::new();
        for u in &self.journal {
            match u {
                Undo::Alloc(id) | Undo::Splice { id, .. } | Undo::State { id, .. } | Undo::Carrier { id, .. } => v.push(*id),
                _ => {}
            }
        }
        v.sort_unstable();
        v.dedup();
        v
    }

    pub(crate) fn finish_op(&mut self) -> Result<OpSummary, TopoError> {
        let st = self.open.ok_or(TopoError::NoOpenOp)?;
        // 1. Re-apply the orientation rule to every touched face.
        let touched = self.touched_ids();
        let mut faces: Vec<EntityId> = touched.iter().copied().filter(|id| id.kind() == EntityKind::TopoFace).collect();
        faces.extend(self.orient_dirty.take_sorted().into_iter().filter_map(|s| EntityId::new(EntityKind::TopoFace, u64::from(s) + 1)));
        faces.sort_unstable();
        faces.dedup();
        for f in faces {
            if let Err(e) = self.normalize_orientation(f) {
                self.cancel_op();
                return Err(e);
            }
        }
        // 1b. Drop the kinds of emptied non-outer shells and close the ring gaps, so equal
        // shapes hash equal whatever shells they once had.
        if let Err(e) = self.compact_shells() {
            self.cancel_op();
            return Err(e);
        }
        // 2. Cheap local checks on the touched cells and their parents.
        let mut check: Vec<EntityId> = self.touched_ids();
        let seeds = check.clone();
        for id in seeds {
            if self.state(id) != Some(EntityState::Live) {
                continue;
            }
            if matches!(id.kind(), EntityKind::TopoEdge | EntityKind::TopoFace) {
                for e in self.up(id) {
                    let p = e.cell;
                    check.push(p);
                    if id.kind() == EntityKind::TopoEdge {
                        check.extend(self.up(p).iter().map(|q| q.cell));
                    }
                }
            }
        }
        check.sort_unstable();
        check.dedup();
        check.retain(|id| self.state(*id) == Some(EntityState::Live));
        let open = self.open_shells();
        let issues = self.check_local(&check, &open);
        if self.paranoid {
            let full = crate::check::check_cells(self, &check, None);
            assert_eq!(issues, full, "incremental end_op checks differ from the full reference checks");
        }
        if !issues.is_empty() {
            self.cancel_op();
            return Err(TopoError::OpInvalid(issues));
        }
        // 3. Rebuild the radial rings of changed edges (independent per edge, so parallel and
        // deterministic: results are written back by slot).
        let dirty = self.dirty_edges.take_sorted();
        let this: &Self = self;
        let compute = |s: &u32| -> Option<(usize, SmallVec<[sm_topo_api::EdgeUse; 4]>)> {
            let slot = *s as usize;
            if slot >= this.t[Dim::Edge.index()].state.len() {
                return None;
            }
            let ring = if this.t[Dim::Edge.index()].state[slot] == EntityState::Live {
                let id = EntityId::new(EntityKind::TopoEdge, slot as u64 + 1)?;
                this.compute_radial(id)
            } else {
                SmallVec::new()
            };
            Some((slot, ring))
        };
        let rings: Vec<(usize, SmallVec<[sm_topo_api::EdgeUse; 4]>)> =
            if dirty.len() >= PAR_RADIAL_MIN { dirty.par_iter().filter_map(compute).collect() } else { dirty.iter().filter_map(compute).collect() };
        for (slot, ring) in rings {
            self.radial[slot] = ring;
        }
        // 4. Summary and commit.
        let mut created: Vec<EntityId> = Vec::new();
        let mut retired: Vec<EntityId> = Vec::new();
        for u in &self.journal {
            match u {
                Undo::Alloc(id) => created.push(*id),
                Undo::State { id, .. } if self.state(*id) != Some(EntityState::Live) => retired.push(*id),
                _ => {}
            }
        }
        created.sort();
        retired.sort();
        retired.dedup();
        let summary = OpSummary::new(st.op, created, retired, st.start.lineage, self.lineage.len() - st.start.lineage);
        self.open = None;
        self.ops_run += 1;
        self.journal.clear();
        self.jdata.clear();
        Ok(summary)
    }
}
