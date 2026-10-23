//! [`CellComplex`]: the reference in-memory implementor of [`TopologyView`] and [`TopologyEdit`].
//!
//! Storage (all that is stored, per rank): `state`, `born`, `carrier`, `flags`, one boundary row
//! (a "paged" CSR: `(start, len)` spans into one flat `Vec<Incidence>`), and incrementally
//! maintained derived data:
//! - the **coboundary** (transpose) rows, updated by every row write, ascending by parent id;
//! - the sorted **live lists**;
//! - **aux rows**: distinct edges of a face and distinct faces of a volume, ascending;
//! - the geometry-dependent **radial rings** of edges, rebuilt per changed edge at `end_op`.
//!
//! Every mutation goes through a few primitives (`alloc`, `set_row`, `retire`, carrier/flag/
//! tolerance/pcurve/pair setters) that append an inverse record to the op **journal**. That one
//! mechanism gives atomic operators (a failing operator rolls back to its mark), `abort_op`, and
//! the set of touched cells that `end_op` validates, so building N cells costs O(N log N) and
//! never rebuilds an index wholesale.

mod ops;
mod radial;

use crate::derived::TopologyViewExt;
use crate::error::TopoError;
use crate::lineage::LineageLog;
use crate::types::{
    Carrier, CellFlags, CoedgeKey, Dim, EntityState, FacePair, Incidence, LineageKind, LineageRecord, ModKind, OpId, OpSummary, ShellKind, Side, Sense, SurfaceCarrier,
};
use crate::validate;
use crate::view::{EdgeUse, NO_CARRIER, TopologyView};
use sm_core::EntityId;
use sm_geom_api::{CurveDef, GeomStore, GeomStoreMut, MemoryStore, ModelTolerance, PCurve, SurfaceDef};
use smallvec::SmallVec;
use std::collections::{BTreeMap, BTreeSet};

/// Paged CSR rows: row `slot` lives at `data[span.0 .. span.0 + span.1]`. Replacing a row appends
/// the new one and orphans the old; `compact` rewrites in slot order (deterministic). Compaction
/// only runs when no operation is open, so a rollback can always restore an old span.
#[derive(Clone, Debug, Default)]
struct Rows {
    span: Vec<(u32, u32)>,
    data: Vec<Incidence>,
    garbage: usize,
}

impl Rows {
    fn push_empty(&mut self) {
        self.span.push((0, 0));
    }

    fn row(&self, slot: usize) -> &[Incidence] {
        match self.span.get(slot) {
            Some(&(s, l)) => self.data.get(s as usize..(s + l) as usize).unwrap_or(&[]),
            None => &[],
        }
    }

    fn span_row(&self, span: (u32, u32)) -> &[Incidence] {
        self.data.get(span.0 as usize..(span.0 + span.1) as usize).unwrap_or(&[])
    }

    /// Replace row `slot`; returns the previous span.
    fn set(&mut self, slot: usize, entries: &[Incidence]) -> Result<(u32, u32), TopoError> {
        let start = u32::try_from(self.data.len()).map_err(|_| TopoError::IdExhausted)?;
        let len = u32::try_from(entries.len()).map_err(|_| TopoError::IdExhausted)?;
        let Some(sp) = self.span.get_mut(slot) else { return Err(TopoError::IdExhausted) };
        let old = *sp;
        self.garbage += old.1 as usize;
        *sp = (start, len);
        self.data.extend_from_slice(entries);
        Ok(old)
    }

    fn restore(&mut self, slot: usize, span: (u32, u32)) {
        if let Some(sp) = self.span.get_mut(slot) {
            self.garbage += sp.1 as usize;
            *sp = span;
        }
    }

    fn maybe_compact(&mut self) {
        if self.garbage <= 4096 || self.garbage * 2 <= self.data.len() {
            return;
        }
        let mut data = Vec::with_capacity(self.data.len() - self.garbage.min(self.data.len()));
        let mut span = Vec::with_capacity(self.span.len());
        for slot in 0..self.span.len() {
            let start = data.len() as u32;
            let row = self.row(slot);
            data.extend_from_slice(row);
            span.push((start, row.len() as u32));
        }
        self.data = data;
        self.span = span;
        self.garbage = 0;
    }
}

#[derive(Clone, Debug, Default)]
struct Table {
    state: Vec<EntityState>,
    born: Vec<OpId>,
    carrier: Vec<Carrier>,
    flags: Vec<CellFlags>,
    down: Rows,
    up: Vec<SmallVec<[Incidence; 4]>>,
    aux: Vec<SmallVec<[EntityId; 8]>>,
    live: Vec<EntityId>,
}

/// Inverse of one primitive mutation.
#[derive(Clone, Debug)]
enum Undo {
    Alloc(EntityId),
    Row { id: EntityId, old: (u32, u32) },
    State { id: EntityId, old: EntityState },
    Carrier { id: EntityId, old: Carrier },
    Flags { id: EntityId, old: CellFlags },
    Tol { id: EntityId, old: Option<f64> },
    PCurve { key: CoedgeKey, old: Option<PCurve> },
    Shell { key: (EntityId, u16), old: Option<ShellKind> },
    Pair { a: EntityId, b: EntityId, old: Option<FacePair> },
    ModelTol(ModelTolerance),
    Curve { id: EntityId, old: CurveDef },
    Surface { id: EntityId, old: SurfaceDef },
    Gap(Option<f64>),
}

/// A point to roll back to.
#[derive(Clone, Copy, Debug)]
struct Mark {
    journal: usize,
    lineage: usize,
    curves: usize,
    surfaces: usize,
}

/// State of the open operation.
#[derive(Clone, Copy, Debug)]
struct OpState {
    op: OpId,
    prev_revision: OpId,
    start: Mark,
}

/// The reference cellular non-manifold complex.
///
/// Implements [`TopologyView`] and [`TopologyEdit`]. It owns its geometry store
/// ([`TopologyView::shapes`] / [`TopologyEdit::shapes_mut`]). Equal edit sequences give equal ids
/// and equal [`TopologyView::canonical_hash`].
///
/// All edits must happen between `begin_op` and `end_op`; each operator is atomic and
/// `abort_op` restores the state at `begin_op`.
#[derive(Clone, Debug)]
pub struct CellComplex {
    tol: ModelTolerance,
    tables: [Table; 5],
    tol_override: BTreeMap<EntityId, f64>,
    pcurves: BTreeMap<CoedgeKey, PCurve>,
    shells: BTreeMap<(EntityId, u16), ShellKind>,
    pairs: Vec<FacePair>,
    pair_by_face: BTreeMap<EntityId, SmallVec<[FacePair; 2]>>,
    lineage: LineageLog,
    shapes: MemoryStore,
    radial: Vec<SmallVec<[EdgeUse; 4]>>,
    revision: OpId,
    open: Option<OpState>,
    ops_run: u64,
    merge_gap: Option<f64>,
    journal: Vec<Undo>,
    dirty_edges: BTreeSet<EntityId>,
}

impl Default for CellComplex {
    fn default() -> Self {
        Self::new()
    }
}

impl CellComplex {
    /// An empty complex with the default tolerance policy.
    pub fn new() -> Self {
        Self::with_tolerance(ModelTolerance::DEFAULT)
    }

    /// An empty complex with the given global tolerance policy.
    pub fn with_tolerance(tol: ModelTolerance) -> Self {
        Self {
            tol,
            tables: core::array::from_fn(|_| Table::default()),
            tol_override: BTreeMap::new(),
            pcurves: BTreeMap::new(),
            shells: BTreeMap::new(),
            pairs: Vec::new(),
            pair_by_face: BTreeMap::new(),
            lineage: LineageLog::new(),
            shapes: MemoryStore::new(),
            radial: Vec::new(),
            revision: OpId(0),
            open: None,
            ops_run: 0,
            merge_gap: None,
            journal: Vec::new(),
            dirty_edges: BTreeSet::new(),
        }
    }

    /// The geometry store (read access without going through the trait object).
    pub fn store(&self) -> &MemoryStore {
        &self.shapes
    }

    /// The operation currently open, if any.
    pub fn open_op(&self) -> Option<OpId> {
        self.open.as_ref().map(|o| o.op)
    }

    // ---------------------------------------------------------------- primitives

    fn locate(&self, id: EntityId) -> Option<(Dim, usize)> {
        let dim = Dim::of_real_kind(id.kind())?;
        let slot = usize::try_from(id.serial()).ok()?;
        (slot < self.tables[dim.index()].state.len()).then_some((dim, slot))
    }

    fn ensure_open(&self) -> Result<OpId, TopoError> {
        self.open.as_ref().map(|o| o.op).ok_or(TopoError::NoOpenOp)
    }

    fn mark(&self) -> Mark {
        Mark { journal: self.journal.len(), lineage: self.lineage.len(), curves: self.shapes.curve_count(), surfaces: self.shapes.surface_count() }
    }

    /// Run `f` as one atomic operator: on error everything it did is rolled back.
    fn atomic<T>(&mut self, f: impl FnOnce(&mut Self) -> Result<T, TopoError>) -> Result<T, TopoError> {
        self.ensure_open()?;
        let mark = self.mark();
        let r = f(self);
        if r.is_err() {
            self.rollback(mark);
        }
        r
    }

    fn rollback(&mut self, mark: Mark) {
        while self.journal.len() > mark.journal {
            let Some(u) = self.journal.pop() else { break };
            self.undo(u);
        }
        self.lineage.truncate(mark.lineage);
        self.shapes.truncate(mark.curves, mark.surfaces);
    }

    fn undo(&mut self, u: Undo) {
        match u {
            Undo::Alloc(id) => {
                if let Some(dim) = Dim::of_real_kind(id.kind()) {
                    let t = &mut self.tables[dim.index()];
                    t.state.pop();
                    t.born.pop();
                    t.carrier.pop();
                    t.flags.pop();
                    t.down.span.pop();
                    t.up.pop();
                    t.aux.pop();
                    if let Ok(i) = t.live.binary_search(&id) {
                        t.live.remove(i);
                    }
                    if dim == Dim::Edge {
                        self.radial.pop();
                    }
                }
            }
            Undo::Row { id, old } => {
                if let Some((dim, slot)) = self.locate(id) {
                    let cur: Vec<Incidence> = self.tables[dim.index()].down.row(slot).to_vec();
                    let prev: Vec<Incidence> = self.tables[dim.index()].down.span_row(old).to_vec();
                    self.relink(id, &cur, &prev);
                    self.tables[dim.index()].down.restore(slot, old);
                    self.refresh_aux(dim, slot);
                }
            }
            Undo::State { id, old } => {
                if let Some((dim, slot)) = self.locate(id) {
                    let t = &mut self.tables[dim.index()];
                    let was_live = t.state[slot] == EntityState::Live;
                    t.state[slot] = old;
                    if old == EntityState::Live && !was_live {
                        if let Err(i) = t.live.binary_search(&id) {
                            t.live.insert(i, id);
                        }
                    } else if old != EntityState::Live && was_live {
                        if let Ok(i) = t.live.binary_search(&id) {
                            t.live.remove(i);
                        }
                    }
                }
            }
            Undo::Carrier { id, old } => {
                if let Some((dim, slot)) = self.locate(id) {
                    self.tables[dim.index()].carrier[slot] = old;
                }
            }
            Undo::Flags { id, old } => {
                if let Some((dim, slot)) = self.locate(id) {
                    self.tables[dim.index()].flags[slot] = old;
                }
            }
            Undo::Tol { id, old } => match old {
                Some(t) => {
                    self.tol_override.insert(id, t);
                }
                None => {
                    self.tol_override.remove(&id);
                }
            },
            Undo::PCurve { key, old } => match old {
                Some(p) => {
                    self.pcurves.insert(key, p);
                }
                None => {
                    self.pcurves.remove(&key);
                }
            },
            Undo::Shell { key, old } => match old {
                Some(k) => {
                    self.shells.insert(key, k);
                }
                None => {
                    self.shells.remove(&key);
                }
            },
            Undo::Pair { a, b, old } => match old {
                Some(p) => self.raw_put_pair(p),
                None => self.raw_del_pair(a, b),
            },
            Undo::ModelTol(t) => self.tol = t,
            Undo::Curve { id, old } => {
                let _ = self.shapes.replace_curve(id, old);
            }
            Undo::Surface { id, old } => {
                let _ = self.shapes.replace_surface(id, old);
            }
            Undo::Gap(g) => self.merge_gap = g,
        }
    }

    fn remove_up(&mut self, child: EntityId, inc: Incidence) {
        if let Some((d, s)) = self.locate(child) {
            let r = &mut self.tables[d.index()].up[s];
            if let Ok(i) = r.binary_search(&inc) {
                r.remove(i);
            }
        }
    }

    fn insert_up(&mut self, child: EntityId, inc: Incidence) {
        if let Some((d, s)) = self.locate(child) {
            let r = &mut self.tables[d.index()].up[s];
            let i = match r.binary_search(&inc) {
                Ok(i) | Err(i) => i,
            };
            r.insert(i, inc);
        }
    }

    /// Move the coboundary links of `id` from the entries `from` to the entries `to`.
    fn relink(&mut self, id: EntityId, from: &[Incidence], to: &[Incidence]) {
        for e in from {
            self.remove_up(e.cell, Incidence { cell: id, side: e.side, ring: e.ring });
            if e.cell.kind() == sm_core::EntityKind::TopoEdge {
                self.dirty_edges.insert(e.cell);
            }
        }
        for e in to {
            self.insert_up(e.cell, Incidence { cell: id, side: e.side, ring: e.ring });
            if e.cell.kind() == sm_core::EntityKind::TopoEdge {
                self.dirty_edges.insert(e.cell);
            }
        }
    }

    fn refresh_aux(&mut self, dim: Dim, slot: usize) {
        if !matches!(dim, Dim::Face | Dim::Volume) {
            return;
        }
        let t = &self.tables[dim.index()];
        let mut v: SmallVec<[EntityId; 8]> = t.down.row(slot).iter().filter(|e| e.ring != Incidence::EMBEDDED).map(|e| e.cell).collect();
        v.sort();
        v.dedup();
        self.tables[dim.index()].aux[slot] = v;
    }

    fn alloc(&mut self, dim: Dim, carrier: Carrier) -> Result<EntityId, TopoError> {
        let op = self.ensure_open()?;
        let t = &mut self.tables[dim.index()];
        let id = EntityId::new(dim.kind(), t.state.len() as u64).ok_or(TopoError::IdExhausted)?;
        t.state.push(EntityState::Live);
        t.born.push(op);
        t.carrier.push(carrier);
        t.flags.push(CellFlags::NONE);
        t.down.push_empty();
        t.up.push(SmallVec::new());
        t.aux.push(SmallVec::new());
        t.live.push(id);
        if dim == Dim::Edge {
            self.radial.push(SmallVec::new());
        }
        self.journal.push(Undo::Alloc(id));
        Ok(id)
    }

    /// Check `id` is a live cell of rank `dim`; returns its slot.
    fn live_slot(&self, id: EntityId, dim: Dim) -> Result<usize, TopoError> {
        if Dim::of_real_kind(id.kind()) != Some(dim) {
            return Err(TopoError::WrongDim { id, expected: dim });
        }
        match self.locate(id) {
            None => Err(TopoError::NoSuchCell(id)),
            Some((_, slot)) if self.tables[dim.index()].state[slot] == EntityState::Live => Ok(slot),
            Some(_) => Err(TopoError::Dead(id)),
        }
    }

    fn live_any(&self, id: EntityId) -> Result<(Dim, usize), TopoError> {
        let (dim, slot) = self.locate(id).ok_or(TopoError::NoSuchCell(id))?;
        if self.tables[dim.index()].state[slot] == EntityState::Live { Ok((dim, slot)) } else { Err(TopoError::Dead(id)) }
    }

    fn set_row(&mut self, id: EntityId, entries: &[Incidence]) -> Result<(), TopoError> {
        let (dim, slot) = self.live_any(id)?;
        let prev: Vec<Incidence> = self.tables[dim.index()].down.row(slot).to_vec();
        let old = self.tables[dim.index()].down.set(slot, entries)?;
        self.journal.push(Undo::Row { id, old });
        self.relink(id, &prev, entries);
        self.refresh_aux(dim, slot);
        Ok(())
    }

    fn row(&self, id: EntityId) -> &[Incidence] {
        match self.locate(id) {
            Some((d, s)) => self.tables[d.index()].down.row(s),
            None => &[],
        }
    }

    fn up(&self, id: EntityId) -> &[Incidence] {
        match self.locate(id) {
            Some((d, s)) => &self.tables[d.index()].up[s],
            None => &[],
        }
    }

    fn note(&mut self, entity: EntityId, kind: LineageKind) {
        let op = self.open.as_ref().map_or(self.revision, |o| o.op);
        self.lineage.push(LineageRecord { op, entity, kind });
    }

    fn set_state(&mut self, id: EntityId, new: EntityState) {
        if let Some((dim, slot)) = self.locate(id) {
            let t = &mut self.tables[dim.index()];
            let old = t.state[slot];
            if old == new {
                return;
            }
            t.state[slot] = new;
            if old == EntityState::Live {
                if let Ok(i) = t.live.binary_search(&id) {
                    t.live.remove(i);
                }
            }
            self.journal.push(Undo::State { id, old });
        }
    }

    fn set_carrier_raw(&mut self, id: EntityId, c: Carrier) {
        if let Some((dim, slot)) = self.locate(id) {
            let old = core::mem::replace(&mut self.tables[dim.index()].carrier[slot], c);
            self.journal.push(Undo::Carrier { id, old });
        }
    }

    fn set_flags_raw(&mut self, id: EntityId, f: CellFlags) {
        if let Some((dim, slot)) = self.locate(id) {
            let old = core::mem::replace(&mut self.tables[dim.index()].flags[slot], f);
            self.journal.push(Undo::Flags { id, old });
        }
    }

    fn set_tol_raw(&mut self, id: EntityId, t: Option<f64>) {
        let old = match t {
            Some(v) => self.tol_override.insert(id, v),
            None => self.tol_override.remove(&id),
        };
        if old != t {
            self.journal.push(Undo::Tol { id, old });
        }
    }

    fn set_pcurve_raw(&mut self, key: CoedgeKey, pc: Option<PCurve>) {
        let old = match pc {
            Some(p) => self.pcurves.insert(key, p),
            None => self.pcurves.remove(&key),
        };
        self.journal.push(Undo::PCurve { key, old });
    }

    fn move_pcurve(&mut self, from: CoedgeKey, to: CoedgeKey) {
        if let Some(p) = self.pcurves.get(&from).cloned() {
            self.set_pcurve_raw(from, None);
            self.set_pcurve_raw(to, Some(p));
        }
    }

    fn set_shell_raw(&mut self, key: (EntityId, u16), k: Option<ShellKind>) {
        let old = match k {
            Some(v) => self.shells.insert(key, v),
            None => self.shells.remove(&key),
        };
        self.journal.push(Undo::Shell { key, old });
    }

    fn raw_put_pair(&mut self, p: FacePair) {
        match self.pairs.binary_search_by(|q| (q.a, q.b).cmp(&(p.a, p.b))) {
            Ok(i) => self.pairs[i] = p,
            Err(i) => self.pairs.insert(i, p),
        }
        for f in [p.a, p.b] {
            let v = self.pair_by_face.entry(f).or_default();
            match v.binary_search_by(|q| (q.a, q.b).cmp(&(p.a, p.b))) {
                Ok(i) => v[i] = p,
                Err(i) => v.insert(i, p),
            }
        }
    }

    fn raw_del_pair(&mut self, a: EntityId, b: EntityId) {
        if let Ok(i) = self.pairs.binary_search_by(|q| (q.a, q.b).cmp(&(a, b))) {
            self.pairs.remove(i);
        }
        for f in [a, b] {
            if let Some(v) = self.pair_by_face.get_mut(&f) {
                if let Ok(i) = v.binary_search_by(|q| (q.a, q.b).cmp(&(a, b))) {
                    v.remove(i);
                }
                if v.is_empty() {
                    self.pair_by_face.remove(&f);
                }
            }
        }
    }

    fn put_pair(&mut self, p: FacePair) {
        let old = self.pairs.binary_search_by(|q| (q.a, q.b).cmp(&(p.a, p.b))).ok().map(|i| self.pairs[i]);
        self.journal.push(Undo::Pair { a: p.a, b: p.b, old });
        self.raw_put_pair(p);
    }

    fn del_pair(&mut self, a: EntityId, b: EntityId) {
        let old = self.pairs.binary_search_by(|q| (q.a, q.b).cmp(&(a, b))).ok().map(|i| self.pairs[i]);
        if old.is_some() {
            self.journal.push(Undo::Pair { a, b, old });
            self.raw_del_pair(a, b);
        }
    }

    /// Mark dead (with the given state) and clear the row. Parents must already be rewritten.
    fn retire(&mut self, id: EntityId, state: EntityState) -> Result<(), TopoError> {
        self.set_row(id, &[])?;
        self.set_state(id, state);
        self.set_tol_raw(id, None);
        if let Some((Dim::Face, _)) = self.locate(id) {
            let doomed: Vec<FacePair> = self.pair_by_face.get(&id).map(|v| v.iter().copied().filter(|p| p.status != crate::types::Coincidence::Merged).collect()).unwrap_or_default();
            for p in doomed {
                self.del_pair(p.a, p.b);
            }
        }
        if let Some((Dim::Volume, _)) = self.locate(id) {
            let keys: Vec<(EntityId, u16)> = self.shells.range((id, 0)..=(id, u16::MAX)).map(|(k, _)| *k).collect();
            for k in keys {
                self.set_shell_raw(k, None);
            }
        }
        Ok(())
    }

    /// Normalize a row into canonical form for its rank.
    fn normalize(dim: Dim, row: &mut Vec<Incidence>) {
        match dim {
            Dim::Vertex => {}
            Dim::Edge => {
                let merged = match row.as_slice() {
                    [a, b] if a.cell == b.cell && a.side != Side::Both => Some(a.cell),
                    _ => None,
                };
                if let Some(cell) = merged {
                    *row = vec![Incidence::new(cell, Side::Both, 0)];
                }
            }
            Dim::Face => {
                let mut out = Vec::with_capacity(row.len());
                let mut embedded: Vec<Incidence> = Vec::new();
                for g in row.chunk_by(|a, b| a.ring == b.ring) {
                    if g[0].ring == Incidence::EMBEDDED {
                        embedded.extend_from_slice(g);
                    } else {
                        let mut ring = g.to_vec();
                        canonical_rotate(&mut ring);
                        out.extend(ring);
                    }
                }
                embedded.sort();
                embedded.dedup();
                out.extend(embedded);
                *row = out;
            }
            Dim::Volume | Dim::Body => {
                row.sort_by_key(|e| (e.ring, e.cell, e.side));
                row.dedup();
                // Left + Right of the same face in one shell is a baffle.
                let mut out: Vec<Incidence> = Vec::with_capacity(row.len());
                for e in row.iter() {
                    match out.last_mut() {
                        Some(l) if l.cell == e.cell && l.ring == e.ring && l.side != e.side && dim == Dim::Volume => l.side = Side::Both,
                        _ => out.push(*e),
                    }
                }
                *row = out;
            }
        }
    }

    /// Rewrite every parent row of `old`, replacing each entry that references it with `f(entry)`.
    fn replace_in_parents(&mut self, old: EntityId, f: impl Fn(&Incidence) -> SmallVec<[Incidence; 2]>) -> Result<(), TopoError> {
        let mut parents: Vec<EntityId> = self.up(old).iter().map(|e| e.cell).collect();
        parents.dedup();
        for parent in parents {
            let Some((pdim, _)) = self.locate(parent) else { continue };
            let mut new: Vec<Incidence> = Vec::with_capacity(self.row(parent).len() + 1);
            for e in self.row(parent) {
                if e.cell == old {
                    new.extend(f(e));
                } else {
                    new.push(*e);
                }
            }
            Self::normalize(pdim, &mut new);
            self.set_row(parent, &new)?;
        }
        Ok(())
    }

    fn effective_tol(&self, id: EntityId) -> f64 {
        TopologyView::tolerance_of(self, id)
    }

    fn inherit(&mut self, from: EntityId, to: EntityId) {
        if let Some((d, s)) = self.locate(from) {
            let fl = self.tables[d.index()].flags[s];
            self.set_flags_raw(to, fl);
        }
        if let Some(t) = self.tol_override.get(&from).copied() {
            self.set_tol_raw(to, Some(t));
        }
    }

    /// Which sides of a face are occupied by volumes.
    fn occupancy(&self, face: EntityId) -> (bool, bool) {
        let mut occ = (false, false);
        for e in self.up(face) {
            if Dim::of_real_kind(e.cell.kind()) == Some(Dim::Volume) {
                occ.0 |= e.side != Side::Right;
                occ.1 |= e.side != Side::Left;
            }
        }
        occ
    }

    fn check_occupancy(&self, uses: &[(EntityId, Side)]) -> Result<(), TopoError> {
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
    fn reverse_inner(&mut self, face: EntityId) -> Result<(), TopoError> {
        let row = self.row(face).to_vec();
        let mut out: Vec<Incidence> = Vec::with_capacity(row.len());
        for g in row.chunk_by(|a, b| a.ring == b.ring) {
            if g[0].ring == Incidence::EMBEDDED {
                out.extend_from_slice(g);
            } else {
                out.extend(g.iter().rev().map(|e| Incidence { side: e.side.flip(), ..*e }));
            }
        }
        Self::normalize(Dim::Face, &mut out);
        // Re-key pcurves (the uv curve of an edge does not change when the loop is reversed).
        let mut moved: Vec<(CoedgeKey, PCurve)> = Vec::new();
        for e in &row {
            if e.ring == Incidence::EMBEDDED {
                continue;
            }
            let k = CoedgeKey { face, edge: e.cell, side: e.side };
            if let Some(p) = self.pcurves.get(&k).cloned() {
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
        if let Some((_, s)) = self.locate(face) {
            if let Carrier::Surface(sc) = self.tables[Dim::Face.index()].carrier[s].clone() {
                self.set_carrier_raw(face, Carrier::Surface(SurfaceCarrier { sense: sc.sense.flip(), ..sc }));
            }
        }
        self.note(face, LineageKind::Modified(ModKind::Reversed));
        Ok(())
    }

    /// Re-apply the orientation rule (ir.md §6) to one face: a lone volume sits on the left, two
    /// different volumes are `left < right`. Logs a `Modified(Reversed)` if it had to flip.
    fn normalize_orientation(&mut self, face: EntityId) -> Result<(), TopoError> {
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

    fn edge_ends_raw(&self, edge: EntityId) -> Option<(EntityId, EntityId)> {
        match self.row(edge) {
            [only] if only.side == Side::Both => Some((only.cell, only.cell)),
            [a, b] if a.side == Side::Left && b.side == Side::Right => Some((a.cell, b.cell)),
            _ => None,
        }
    }

}

/// `a < b`, false if either is NaN (used by operators).
pub(crate) fn lt(a: f64, b: f64) -> bool {
    a.partial_cmp(&b) == Some(core::cmp::Ordering::Less)
}

/// Rotate a cyclic ring so it starts at its smallest `(edge, side)`; ties are broken by comparing
/// the whole rotated sequences, so the result is unique.
pub(crate) fn canonical_rotate(ring: &mut [Incidence]) {
    let n = ring.len();
    if n < 2 {
        return;
    }
    let key = |e: &Incidence| (e.cell, e.side);
    let Some(min) = ring.iter().map(key).min() else { return };
    let mut best: Option<usize> = None;
    for i in 0..n {
        if key(&ring[i]) != min {
            continue;
        }
        best = match best {
            None => Some(i),
            Some(b) => {
                let less = (0..n).map(|k| (key(&ring[(i + k) % n]), key(&ring[(b + k) % n]))).find(|(x, y)| x != y).is_some_and(|(x, y)| x < y);
                Some(if less { i } else { b })
            }
        };
    }
    if let Some(b) = best {
        ring.rotate_left(b);
    }
}

impl TopologyView for CellComplex {
    fn tolerance(&self) -> &ModelTolerance {
        &self.tol
    }

    fn shapes(&self) -> &dyn GeomStore {
        &self.shapes
    }

    fn dim_of(&self, id: EntityId) -> Option<Dim> {
        self.locate(id).map(|(d, _)| d)
    }

    fn state(&self, id: EntityId) -> Option<EntityState> {
        self.locate(id).map(|(d, s)| self.tables[d.index()].state[s])
    }

    fn cells(&self, dim: Dim) -> &[EntityId] {
        &self.tables[dim.index()].live
    }

    fn boundary(&self, id: EntityId) -> &[Incidence] {
        self.row(id)
    }

    fn coboundary(&self, id: EntityId) -> &[Incidence] {
        self.up(id)
    }

    fn carrier(&self, id: EntityId) -> &Carrier {
        match self.locate(id) {
            Some((d, s)) => &self.tables[d.index()].carrier[s],
            None => &NO_CARRIER,
        }
    }

    fn flags(&self, id: EntityId) -> CellFlags {
        match self.locate(id) {
            Some((d, s)) => self.tables[d.index()].flags[s],
            None => CellFlags::NONE,
        }
    }

    fn pcurve(&self, k: CoedgeKey) -> Option<&PCurve> {
        self.pcurves.get(&k)
    }

    fn shell_kind(&self, volume: EntityId, ring: u16) -> Option<ShellKind> {
        self.shells.get(&(volume, ring)).copied()
    }

    fn radial_uses(&self, edge: EntityId) -> &[EdgeUse] {
        match self.locate(edge) {
            Some((Dim::Edge, s)) => &self.radial[s],
            _ => &[],
        }
    }

    fn tolerance_override(&self, id: EntityId) -> Option<f64> {
        self.tol_override.get(&id).copied()
    }

    fn coincidences(&self) -> &[FacePair] {
        &self.pairs
    }

    fn coincident_faces(&self, f: EntityId) -> &[FacePair] {
        self.pair_by_face.get(&f).map_or(&[], |v| v.as_slice())
    }

    fn lineage(&self) -> &LineageLog {
        &self.lineage
    }

    fn born(&self, id: EntityId) -> Option<OpId> {
        self.locate(id).map(|(d, s)| self.tables[d.index()].born[s])
    }

    fn revision(&self) -> OpId {
        self.revision
    }

    fn volume_faces(&self, v: EntityId) -> &[EntityId] {
        match self.locate(v) {
            Some((Dim::Volume, s)) => &self.tables[Dim::Volume.index()].aux[s],
            _ => &[],
        }
    }

    fn face_edges(&self, f: EntityId) -> &[EntityId] {
        match self.locate(f) {
            Some((Dim::Face, s)) => &self.tables[Dim::Face.index()].aux[s],
            _ => &[],
        }
    }
}


const _: fn() = || {
    fn is_send_sync<T: Send + Sync>() {}
    is_send_sync::<CellComplex>();
};
