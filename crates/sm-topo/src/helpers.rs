//! Shared operator machinery: retirement, parent rewrites, orientation, shell compaction.

use crate::norm::{normalize, sort_merge};
use crate::store::{TopoStore, Undo};
use smallvec::SmallVec;
use sm_core::{EntityId, EntityKind};
use sm_topo_api::{Carrier, Coincidence, CoedgeKey, Dim, EntityState, FacePair, Incidence, LineageKind, ModKind, Side, SurfaceCarrier, TopoError, TopologyView};
use std::collections::BTreeMap;

/// Volume/body rows at least this long are edited by position instead of rebuilt.
const SORTED_ROW_MIN: usize = 24;

/// Positions of every entry naming `cell` in a row sorted by `(ring, cell, side)`.
fn positions_of(row: &[Incidence], cell: EntityId, out: &mut Vec<usize>) {
    let mut pos = 0;
    while pos < row.len() {
        let ring = row[pos].ring;
        let end = pos + row[pos..].partition_point(|e| e.ring == ring);
        let g = &row[pos..end];
        let lo = g.partition_point(|e| e.cell < cell);
        let hi = g.partition_point(|e| e.cell <= cell);
        out.extend((pos + lo)..(pos + hi));
        pos = end;
    }
}

impl TopoStore {
    /// Mark dead (with the given state) and clear the row. Parents must already be rewritten.
    pub(crate) fn retire(&mut self, id: EntityId, state: EntityState) -> Result<(), TopoError> {
        self.set_row(id, &[])?;
        self.set_state(id, state);
        self.set_tol_raw(id, None);
        if let Some((Dim::Face, _)) = self.locate(id) {
            let doomed: Vec<FacePair> = self.pairs_of_face(id).iter().copied().filter(|p| p.status != Coincidence::Merged).collect();
            for p in doomed {
                self.del_pair(p.a, p.b);
            }
        }
        if let Some((Dim::Volume, _)) = self.locate(id) {
            for (ring, _) in self.shell_kinds(id) {
                self.set_shell_raw((id, ring), None);
            }
        }
        self.release_rows(id);
        Ok(())
    }

    /// For every live volume whose row or shell kinds changed in the open op: drop the kind of
    /// each non-outer ring left without uses and renumber the remaining non-outer rings
    /// `1..n` in their old order (rows and kinds together).
    pub(crate) fn compact_shells(&mut self) -> Result<(), TopoError> {
        let mut vols: Vec<EntityId> = Vec::new();
        for u in &self.journal {
            match u {
                Undo::Splice { id, .. } if id.kind() == EntityKind::TopoVolume => vols.push(*id),
                Undo::Shell { volume, .. } => vols.push(*volume),
                _ => {}
            }
        }
        vols.sort_unstable();
        vols.dedup();
        for v in vols {
            if self.state(v) != Some(EntityState::Live) {
                continue;
            }
            let kinds = self.shell_kinds(v);
            // Distinct rings of the (ring-sorted) row.
            let mut used: SmallVec<[u16; 8]> = SmallVec::new();
            {
                let row = self.row(v);
                let mut pos = 0;
                while pos < row.len() {
                    let ring = row[pos].ring;
                    if ring != Incidence::EMBEDDED {
                        used.push(ring);
                    }
                    pos += row[pos..].partition_point(|e| e.ring == ring);
                }
            }
            // Old ring -> new ring for every kept non-outer ring.
            let mut map: SmallVec<[(u16, u16); 4]> = SmallVec::new();
            let mut next = 1u16;
            for (r, _) in &kinds {
                if *r == 0 {
                    continue;
                }
                if used.contains(r) {
                    map.push((*r, next));
                    next += 1;
                }
            }
            let target = |r: u16| map.iter().find(|(o, _)| *o == r).map(|(_, n)| *n);
            let changed = kinds.iter().any(|(r, _)| *r != 0 && target(*r) != Some(*r));
            if !changed {
                continue;
            }
            for (r, _) in &kinds {
                if *r != 0 {
                    self.set_shell_raw((v, *r), None);
                }
            }
            for (r, k) in &kinds {
                if let Some(n) = target(*r) {
                    self.set_shell_raw((v, n), Some(*k));
                }
            }
            let mut row: Vec<Incidence> = self
                .row(v)
                .iter()
                .map(|e| match target(e.ring) {
                    Some(n) => Incidence { ring: n, ..*e },
                    None => *e,
                })
                .collect();
            normalize(Dim::Volume, &mut row);
            if row.as_slice() != self.row(v) {
                self.set_row(v, &row)?;
            }
        }
        Ok(())
    }

    /// Rewrite every parent row of `old`, replacing each entry that references it with `f(entry)`.
    pub(crate) fn replace_in_parents(&mut self, old: EntityId, f: impl Fn(&Incidence) -> SmallVec<[Incidence; 2]>) -> Result<(), TopoError> {
        let mut parents: SmallVec<[EntityId; 4]> = self.up(old).iter().map(|e| e.cell).collect();
        parents.dedup();
        for parent in parents {
            let Some((pdim, _)) = self.locate(parent) else { continue };
            let long = matches!(pdim, Dim::Volume | Dim::Body) && self.row(parent).len() >= SORTED_ROW_MIN;
            if long {
                self.counters.sorted_parent_rewrites += 1;
                let expected = self.paranoid.then(|| self.generic_replaced(parent, pdim, old, &f));
                self.replace_sorted(parent, pdim, old, &f)?;
                if let Some(expected) = expected {
                    assert_eq!(self.row(parent), expected.as_slice(), "sorted parent rewrite differs from the full rewrite");
                }
            } else {
                self.counters.full_parent_rewrites += 1;
                let new = self.generic_replaced(parent, pdim, old, &f);
                self.set_row(parent, &new)?;
            }
        }
        Ok(())
    }

    /// The full-rewrite result of replacing `old` in the row of `parent`.
    fn generic_replaced(&self, parent: EntityId, pdim: Dim, old: EntityId, f: &impl Fn(&Incidence) -> SmallVec<[Incidence; 2]>) -> Vec<Incidence> {
        let mut new: Vec<Incidence> = Vec::with_capacity(self.row(parent).len() + 1);
        for e in self.row(parent) {
            if e.cell == old {
                new.extend(f(e));
            } else {
                new.push(*e);
            }
        }
        normalize(pdim, &mut new);
        new
    }

    /// Replace `old` in a long sorted (volume/body) row by position: remove the entries of `old`
    /// and of every cell it is replaced by, normalize those groups alone, insert them back.
    fn replace_sorted(&mut self, parent: EntityId, pdim: Dim, old: EntityId, f: &impl Fn(&Incidence) -> SmallVec<[Incidence; 2]>) -> Result<(), TopoError> {
        let row = self.row(parent);
        let mut pos: Vec<usize> = Vec::new();
        positions_of(row, old, &mut pos);
        let mut news: Vec<Incidence> = Vec::new();
        for &p in &pos {
            news.extend(f(&row[p]));
        }
        let mut others: SmallVec<[EntityId; 4]> = news.iter().map(|e| e.cell).filter(|c| *c != old).collect();
        others.sort_unstable();
        others.dedup();
        let mut cand: Vec<Incidence> = Vec::new();
        for &c in &others {
            let before = pos.len();
            positions_of(row, c, &mut pos);
            cand.extend(pos[before..].iter().map(|&p| row[p]));
        }
        cand.extend(news);
        sort_merge(pdim, &mut cand);
        pos.sort_unstable();
        pos.dedup();
        // Remove in descending position order, merging consecutive runs into one splice.
        let mut i = pos.len();
        while i > 0 {
            let end = pos[i - 1];
            let mut start_i = i - 1;
            while start_i > 0 && pos[start_i - 1] + 1 == pos[start_i] {
                start_i -= 1;
            }
            self.splice_row(parent, pos[start_i], end + 1 - pos[start_i], &[])?;
            i = start_i;
        }
        for e in cand {
            let key = (e.ring, e.cell, e.side);
            let at = self.row(parent).partition_point(|x| (x.ring, x.cell, x.side) < key);
            self.splice_row(parent, at, 0, &[e])?;
        }
        Ok(())
    }

    /// Copy flags and tolerance override of `from` onto `to`.
    pub(crate) fn inherit(&mut self, from: EntityId, to: EntityId) {
        if let Some((d, s)) = self.locate(from) {
            let fl = self.t[d.index()].flags[s];
            self.set_flags_raw(to, fl);
        }
        if let Some(t) = self.tol_raw(from) {
            self.set_tol_raw(to, Some(t));
        }
    }

    /// Which sides of a face are occupied by volumes.
    pub(crate) fn occupancy(&self, face: EntityId) -> (bool, bool) {
        let mut occ = (false, false);
        for e in self.up(face) {
            if Dim::of_real_kind(e.cell.kind()) == Some(Dim::Volume) {
                occ.0 |= e.side != Side::Right;
                occ.1 |= e.side != Side::Left;
            }
        }
        occ
    }

    pub(crate) fn check_occupancy(&self, uses: &[(EntityId, Side)]) -> Result<(), TopoError> {
        let mut occ: BTreeMap<EntityId, (bool, bool)> = BTreeMap::new();
        for &(face, side) in uses {
            let o = occ.entry(face).or_insert_with(|| self.occupancy(face));
            let (l, r) = *o;
            match side {
                Side::Left if l => return Err(TopoError::SideOccupied { face }),
                Side::Right if r => return Err(TopoError::SideOccupied { face }),
                Side::Both if l || r => return Err(TopoError::SideOccupied { face }),
                Side::Left => o.0 = true,
                Side::Right => o.1 = true,
                Side::Both => *o = (true, true),
            }
        }
        Ok(())
    }

    /// Reverse a face's orientation: loops run backwards, left and right swap in every parent,
    /// the carrier sense flips and pcurves follow their coedges. Lineage: `Modified(Reversed)`.
    pub(crate) fn reverse_inner(&mut self, face: EntityId) -> Result<(), TopoError> {
        let row = self.row(face).to_vec();
        let mut out: Vec<Incidence> = Vec::with_capacity(row.len());
        for g in row.chunk_by(|a, b| a.ring == b.ring) {
            if g[0].ring == Incidence::EMBEDDED {
                out.extend_from_slice(g);
            } else {
                out.extend(g.iter().rev().map(|e| Incidence { side: e.side.flip(), ..*e }));
            }
        }
        normalize(Dim::Face, &mut out);
        // Re-key pcurves (the uv curve of an edge does not change when the loop is reversed).
        let mut moved: Vec<(CoedgeKey, sm_geom_api::PCurve)> = Vec::new();
        for e in &row {
            if e.ring == Incidence::EMBEDDED {
                continue;
            }
            let k = CoedgeKey { face, edge: e.cell, side: e.side };
            if let Some(p) = self.pcurve_raw(k).cloned() {
                moved.push((k, p));
            }
        }
        for (k, _) in &moved {
            self.set_pcurve_raw(*k, None);
        }
        for (k, p) in moved {
            self.set_pcurve_raw(CoedgeKey { side: k.side.flip(), ..k }, Some(p));
        }
        self.replace_in_parents(face, |e| smallvec::smallvec![Incidence { side: e.side.flip(), ..*e }])?;
        self.set_row(face, &out)?;
        // The carrier sense flips, which changes the radial order around every edge of the face
        // (also embedded ones, whose coboundary entry does not change).
        for e in &row {
            if let Some(slot) = self.slot_in(e.cell, Dim::Edge) {
                self.dirty_edges.insert(slot);
            }
        }
        if let Some((_, s)) = self.locate(face)
            && let Carrier::Surface(sc) = self.t[Dim::Face.index()].carrier[s].clone()
        {
            self.set_carrier_raw(face, Carrier::Surface(SurfaceCarrier { sense: sc.sense.flip(), ..sc }));
        }
        self.note(face, LineageKind::Modified(ModKind::Reversed));
        Ok(())
    }

    /// Re-apply the orientation rule (ir.md §6) to one face: a lone volume sits on the left, two
    /// different volumes are `left < right`. Logs a `Modified(Reversed)` if it had to flip.
    pub(crate) fn normalize_orientation(&mut self, face: EntityId) -> Result<(), TopoError> {
        if self.live_slot(face, Dim::Face).is_err() {
            return Ok(());
        }
        let mut left = None;
        let mut right = None;
        for e in self.up(face) {
            if Dim::of_real_kind(e.cell.kind()) != Some(Dim::Volume) {
                continue;
            }
            match e.side {
                Side::Left => left = Some(e.cell),
                Side::Right => right = Some(e.cell),
                Side::Both => {}
            }
        }
        let flip = match (left, right) {
            (None, Some(_)) => true,
            (Some(l), Some(r)) => l > r,
            _ => false,
        };
        if flip { self.reverse_inner(face) } else { Ok(()) }
    }
}

