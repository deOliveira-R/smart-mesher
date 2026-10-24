//! [`TopoStore`]: storage, primitives, journal and the [`TopologyView`] implementation.
//!
//! Layout (all per rank, structure of arrays, dense `slot = serial - 1`):
//! - `state`, `born`, `carrier`, `flags`, `tol` columns;
//! - `down`: boundary rows in one paged CSR arena ([`Rows`]), edited in place by `splice`;
//! - `up`: the transposed (coboundary) rows in a second arena, kept in step by every splice;
//! - `aux`: distinct edges of a face / distinct faces of a volume (ascending), also in an arena;
//! - `live`: the ascending live list, a lazily rebuilt cache (`OnceLock`), extended on `alloc`.
//!
//! Sparse data is dense too: pcurves per face slot, shell kinds per volume slot, coincidence
//! records per face slot, radial rings per edge slot.
//!
//! Every mutation is a few primitives that append an inverse record to the op journal. Row edits
//! are journaled as *deltas* (`Splice`), so replacing one face of a million-face volume by two is
//! a binary search and a `memmove`, not a row rewrite, and rollback replays the deltas.

use crate::rows::Rows;
use smallvec::SmallVec;
use sm_core::{EntityId, EntityKind};
use sm_geom_api::{CurveDef, GeomStore, GeomStoreMut, MemoryStore, ModelTolerance, PCurve, SurfaceDef};
use sm_topo_api::{
    Carrier, CellFlags, CoedgeKey, Dim, EdgeUse, EntityState, FacePair, HashWord, Incidence, LineageKind, LineageLog, LineageRecord, OpId, ShellKind, Side, TopoError, TopologyView,
};
use std::sync::OnceLock;

/// Retirements per operation applied in place to a cached live list before it is dropped.
const LIVE_EDITS_MAX: u32 = 32;

/// Pads fresh arena blocks of id rows.
const FILL_ID: EntityId = match EntityId::new(EntityKind::TopoVertex, 1) {
    Some(id) => id,
    None => panic!("TopoVertex serial 1 is a valid id"),
};
const FILL_INC: Incidence = Incidence::new(FILL_ID, Side::Left, 0);

/// Bit set of slots plus the list in insertion order.
#[derive(Clone, Debug, Default)]
pub(crate) struct Marks {
    list: Vec<u32>,
    bits: Vec<u64>,
}

impl Marks {
    pub(crate) fn insert(&mut self, slot: usize) {
        let w = slot >> 6;
        if w >= self.bits.len() {
            self.bits.resize(w + 1, 0);
        }
        let m = 1u64 << (slot & 63);
        if self.bits[w] & m == 0 {
            self.bits[w] |= m;
            self.list.push(slot as u32);
        }
    }

    pub(crate) fn clear(&mut self) {
        for &s in &self.list {
            self.bits[(s >> 6) as usize] = 0;
        }
        self.list.clear();
    }

    /// The marked slots, ascending; clears the set.
    pub(crate) fn take_sorted(&mut self) -> Vec<u32> {
        let mut v = core::mem::take(&mut self.list);
        for &s in &v {
            self.bits[(s >> 6) as usize] = 0;
        }
        v.sort_unstable();
        v
    }
}

#[derive(Clone, Debug)]
pub(crate) struct Table {
    pub(crate) state: Vec<EntityState>,
    pub(crate) born: Vec<OpId>,
    pub(crate) carrier: Vec<Carrier>,
    pub(crate) flags: Vec<CellFlags>,
    /// Per-entity linear tolerance override; NaN = none.
    pub(crate) tol: Vec<f64>,
    pub(crate) down: Rows<Incidence>,
    pub(crate) up: Rows<Incidence>,
    pub(crate) aux: Rows<EntityId>,
    /// Ascending live ids; kept up to date for the first few retirements of an operation and
    /// rebuilt lazily after more.
    pub(crate) live: OnceLock<Vec<EntityId>>,
    /// Retirements applied to `live` in place since the open operation began.
    pub(crate) live_edits: u32,
}

impl Table {
    fn new() -> Self {
        Self {
            state: Vec::new(),
            born: Vec::new(),
            carrier: Vec::new(),
            flags: Vec::new(),
            tol: Vec::new(),
            down: Rows::new(FILL_INC),
            up: Rows::new(FILL_INC),
            aux: Rows::new(FILL_ID),
            live: OnceLock::new(),
            live_edits: 0,
        }
    }
}

/// Pcurves of one face, ascending by `(edge, side)`.
pub(crate) type PcList = Vec<(EntityId, Side, PCurve)>;

/// Inverse of one primitive mutation.
#[derive(Clone, Debug)]
pub(crate) enum Undo {
    Alloc(EntityId),
    /// A delta on the boundary row of `id`: `inserted_len` entries at `pos` replaced the
    /// `removed_len` entries saved in the journal data buffer at `removed_start`.
    Splice { id: EntityId, pos: u32, removed_start: u32, removed_len: u32, inserted_len: u32 },
    State { id: EntityId, old: EntityState },
    Carrier { id: EntityId, old: Box<Carrier> },
    Flags { id: EntityId, old: CellFlags },
    Tol { id: EntityId, old: f64 },
    PCurve { key: CoedgeKey, old: Option<Box<PCurve>> },
    Shell { volume: EntityId, ring: u16, old: Option<ShellKind> },
    Pair { a: EntityId, b: EntityId, old: Option<FacePair> },
    ModelTol(ModelTolerance),
    Curve { id: EntityId, old: Box<CurveDef> },
    Surface { id: EntityId, old: Box<SurfaceDef> },
    Gap(Option<f64>),
}

/// A point to roll back to.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Mark {
    pub(crate) journal: usize,
    pub(crate) lineage: usize,
    pub(crate) curves: usize,
    pub(crate) surfaces: usize,
}

/// State of the open operation.
#[derive(Clone, Copy, Debug)]
pub(crate) struct OpState {
    pub(crate) op: OpId,
    pub(crate) prev_revision: OpId,
    pub(crate) start: Mark,
}

/// Diagnostic counters of the shortcuts a [`TopoStore`] has taken.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct Counters {
    /// Parent rows (long volume/body rows) rewritten by position.
    pub sorted_parent_rewrites: u64,
    /// Parent rows rewritten whole.
    pub full_parent_rewrites: u64,
    /// Operations committed with the incremental shell-closure check.
    pub ops_committed: u64,
}

/// The production cellular non-manifold complex.
///
/// Implements [`TopologyView`] and [`sm_topo_api::TopologyEdit`] with the same observable
/// behaviour as the reference [`sm_topo_api::CellComplex`] (ids, lineage, validation results and
/// canonical hashes are identical), on a structure-of-arrays store with in-place delta edits.
/// All edits must happen between `begin_op` and `end_op`; each operator is atomic and
/// `abort_op` restores the state at `begin_op`.
#[derive(Clone, Debug)]
pub struct TopoStore {
    pub(crate) tol: ModelTolerance,
    pub(crate) t: [Table; 5],
    pub(crate) pcurves: Vec<PcList>,
    pub(crate) shells: Vec<SmallVec<[Option<ShellKind>; 4]>>,
    pub(crate) pairs: Vec<FacePair>,
    pub(crate) pair_by_face: Vec<Vec<FacePair>>,
    pub(crate) lineage: LineageLog,
    pub(crate) shapes: MemoryStore,
    pub(crate) radial: Vec<SmallVec<[EdgeUse; 4]>>,
    pub(crate) revision: OpId,
    pub(crate) open: Option<OpState>,
    pub(crate) ops_run: u64,
    pub(crate) merge_gap: Option<f64>,
    pub(crate) journal: Vec<Undo>,
    pub(crate) jdata: Vec<Incidence>,
    /// Edge slots whose radial ring must be rebuilt at `end_op`.
    pub(crate) dirty_edges: Marks,
    /// Face slots whose volume uses changed (orientation rule must be re-applied).
    pub(crate) orient_dirty: Marks,
    /// Edge slots whose shell net use may have changed (incremental closure check).
    pub(crate) closure_edges: Marks,
    /// Persistent across commits: edge slots with a non-zero net shell use at the last commit.
    /// The reference leaves volumes untouched by flag-only edits unchecked, so a committed state
    /// can have open shells that a later check of the volume must still report.
    pub(crate) open_edges: std::collections::BTreeSet<u32>,
    /// Entries inserted into volume/body rows in the open operation (for incremental row checks).
    pub(crate) ins_log: Vec<(EntityId, Incidence)>,
    /// Cross-check the incremental `end_op` shortcuts against the full reference algorithms.
    pub(crate) paranoid: bool,
    /// Volume/body rows at least this long are edited by position.
    pub(crate) sorted_row_min: usize,
    /// Volume/body rows at least this long are checked through their inserted entries only.
    pub(crate) long_row: usize,
    pub(crate) counters: Counters,
}

impl Default for TopoStore {
    fn default() -> Self {
        Self::new()
    }
}

impl TopoStore {
    /// An empty complex with the default tolerance policy.
    pub fn new() -> Self {
        Self::with_tolerance(ModelTolerance::DEFAULT)
    }

    /// An empty complex with the given global tolerance policy.
    pub fn with_tolerance(tol: ModelTolerance) -> Self {
        Self {
            tol,
            t: core::array::from_fn(|_| Table::new()),
            pcurves: Vec::new(),
            shells: Vec::new(),
            pairs: Vec::new(),
            pair_by_face: Vec::new(),
            lineage: LineageLog::new(),
            shapes: MemoryStore::new(),
            radial: Vec::new(),
            revision: OpId(0),
            open: None,
            ops_run: 0,
            merge_gap: None,
            journal: Vec::new(),
            jdata: Vec::new(),
            dirty_edges: Marks::default(),
            orient_dirty: Marks::default(),
            closure_edges: Marks::default(),
            open_edges: Default::default(),
            ins_log: Vec::new(),
            paranoid: false,
            sorted_row_min: 24,
            long_row: 128,
            counters: Counters::default(),
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

    /// Lengths from which volume/body rows are edited by position and checked incrementally.
    /// The defaults suit production; tests lower them to exercise those paths on small models.
    #[doc(hidden)]
    pub fn set_row_thresholds(&mut self, sorted_row_min: usize, long_row: usize) {
        self.sorted_row_min = sorted_row_min;
        self.long_row = long_row;
    }

    /// Diagnostic counters (how often each shortcut ran).
    pub fn counters(&self) -> Counters {
        self.counters
    }

    /// When on, `end_op` also runs the full reference algorithms (whole-shell closure walks,
    /// whole-row checks) and panics if they disagree with the incremental ones. For tests.
    #[doc(hidden)]
    pub fn set_paranoid(&mut self, on: bool) {
        self.paranoid = on;
    }

    // ---------------------------------------------------------------- lookup

    #[inline]
    pub(crate) fn locate(&self, id: EntityId) -> Option<(Dim, usize)> {
        let dim = Dim::of_real_kind(id.kind())?;
        // Serial = slot + 1: serial 0 is never a cell.
        let slot = usize::try_from(id.serial().checked_sub(1)?).ok()?;
        (slot < self.t[dim.index()].state.len()).then_some((dim, slot))
    }

    /// Slot of `id` if it is a cell of rank `dim` that exists (live or not).
    #[inline]
    pub(crate) fn slot_in(&self, id: EntityId, dim: Dim) -> Option<usize> {
        match self.locate(id) {
            Some((d, s)) if d == dim => Some(s),
            _ => None,
        }
    }

    pub(crate) fn ensure_open(&self) -> Result<OpId, TopoError> {
        self.open.as_ref().map(|o| o.op).ok_or(TopoError::NoOpenOp)
    }

    pub(crate) fn mark(&self) -> Mark {
        Mark { journal: self.journal.len(), lineage: self.lineage.len(), curves: self.shapes.curve_count(), surfaces: self.shapes.surface_count() }
    }

    /// Run `f` as one atomic operator: on error everything it did is rolled back.
    pub(crate) fn atomic<T>(&mut self, f: impl FnOnce(&mut Self) -> Result<T, TopoError>) -> Result<T, TopoError> {
        self.ensure_open()?;
        let mark = self.mark();
        let r = f(self);
        if r.is_err() {
            self.rollback(mark);
        }
        r
    }

    pub(crate) fn rollback(&mut self, mark: Mark) {
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
                    let d = dim.index();
                    let t = &mut self.t[d];
                    t.state.pop();
                    t.born.pop();
                    t.carrier.pop();
                    t.flags.pop();
                    t.tol.pop();
                    t.down.pop_slot();
                    t.up.pop_slot();
                    if matches!(dim, Dim::Face | Dim::Volume) {
                        t.aux.pop_slot();
                    }
                    if let Some(c) = t.live.get_mut()
                        && c.last() == Some(&id)
                    {
                        c.pop();
                    }
                    match dim {
                        Dim::Edge => {
                            self.radial.pop();
                        }
                        Dim::Face => {
                            self.pcurves.pop();
                            self.pair_by_face.pop();
                        }
                        Dim::Volume => {
                            self.shells.pop();
                        }
                        _ => {}
                    }
                }
            }
            Undo::Splice { id, pos, removed_start, removed_len, inserted_len } => {
                if let Some((dim, slot)) = self.locate(id) {
                    let (rs, rl) = (removed_start as usize, removed_len as usize);
                    let restored: SmallVec<[Incidence; 8]> = SmallVec::from_slice(&self.jdata[rs..rs + rl]);
                    // Restoring never needs more room than the row had before; errors cannot occur.
                    let _ = self.splice_down(dim, slot, id, pos as usize, inserted_len as usize, &restored);
                    self.jdata.truncate(rs);
                }
            }
            Undo::State { id, old } => {
                if let Some((dim, slot)) = self.locate(id) {
                    let t = &mut self.t[dim.index()];
                    let was_live = t.state[slot] == EntityState::Live;
                    t.state[slot] = old;
                    if was_live != (old == EntityState::Live) {
                        t.live.take();
                    }
                }
            }
            Undo::Carrier { id, old } => {
                if let Some((dim, slot)) = self.locate(id) {
                    self.t[dim.index()].carrier[slot] = *old;
                }
            }
            Undo::Flags { id, old } => {
                if let Some((dim, slot)) = self.locate(id) {
                    self.t[dim.index()].flags[slot] = old;
                }
            }
            Undo::Tol { id, old } => {
                if let Some((dim, slot)) = self.locate(id) {
                    self.t[dim.index()].tol[slot] = old;
                }
            }
            Undo::PCurve { key, old } => {
                self.pcurve_put(key, old.map(|b| *b));
            }
            Undo::Shell { volume, ring, old } => {
                self.shell_put(volume, ring, old);
            }
            Undo::Pair { a, b, old } => match old {
                Some(p) => self.raw_put_pair(p),
                None => self.raw_del_pair(a, b),
            },
            Undo::ModelTol(t) => self.tol = t,
            Undo::Curve { id, old } => {
                let _ = self.shapes.replace_curve(id, *old);
            }
            Undo::Surface { id, old } => {
                let _ = self.shapes.replace_surface(id, *old);
            }
            Undo::Gap(g) => self.merge_gap = g,
        }
    }

    // ---------------------------------------------------------------- rows

    #[inline]
    pub(crate) fn row(&self, id: EntityId) -> &[Incidence] {
        match self.locate(id) {
            Some((d, s)) => self.t[d.index()].down.row(s),
            None => &[],
        }
    }

    #[inline]
    pub(crate) fn up(&self, id: EntityId) -> &[Incidence] {
        match self.locate(id) {
            Some((d, s)) => self.t[d.index()].up.row(s),
            None => &[],
        }
    }

    /// Remove the coboundary link `parent -> entry.cell`.
    fn unlink_up(&mut self, parent: EntityId, e: &Incidence) {
        let Some((d, s)) = self.locate(e.cell) else { return };
        let inc = Incidence { cell: parent, side: e.side, ring: e.ring };
        let r = self.t[d.index()].up.row(s);
        if let Ok(i) = r.binary_search(&inc) {
            let _ = self.t[d.index()].up.splice(s, i, 1, &[]);
        }
        self.note_relink(d, s, parent);
    }

    /// Add the coboundary link `parent -> entry.cell`.
    fn link_up(&mut self, parent: EntityId, e: &Incidence) {
        let Some((d, s)) = self.locate(e.cell) else { return };
        let inc = Incidence { cell: parent, side: e.side, ring: e.ring };
        let r = self.t[d.index()].up.row(s);
        let i = match r.binary_search(&inc) {
            Ok(i) | Err(i) => i,
        };
        // Growth can fail only past 2^32 entries.
        let _ = self.t[d.index()].up.splice(s, i, 0, &[inc]);
        self.note_relink(d, s, parent);
    }

    #[inline]
    fn note_relink(&mut self, child_dim: Dim, child_slot: usize, parent: EntityId) {
        match child_dim {
            Dim::Edge => self.dirty_edges.insert(child_slot),
            // A volume's use of a face changed: the orientation rule must be re-applied.
            Dim::Face if parent.kind() == EntityKind::TopoVolume => self.orient_dirty.insert(child_slot),
            _ => {}
        }
    }

    /// Replace `row[pos .. pos + remove]` of cell `id` by `insert`, keeping the coboundary rows,
    /// the aux rows and the dirty marks in step. Not journaled.
    pub(crate) fn splice_down(&mut self, dim: Dim, slot: usize, id: EntityId, pos: usize, remove: usize, insert: &[Incidence]) -> Result<(), TopoError> {
        let removed: SmallVec<[Incidence; 8]> = SmallVec::from_slice(&self.t[dim.index()].down.row(slot)[pos..pos + remove]);
        self.t[dim.index()].down.splice(slot, pos, remove, insert)?;
        for r in &removed {
            self.unlink_up(id, r);
        }
        for i in insert {
            self.link_up(id, i);
        }
        match dim {
            Dim::Face => {
                for e in removed.iter().chain(insert) {
                    if e.ring != Incidence::EMBEDDED
                        && let Some(s) = self.slot_in(e.cell, Dim::Edge)
                    {
                        self.closure_edges.insert(s);
                    }
                }
                self.update_aux(dim, slot, &removed, insert);
            }
            Dim::Volume => {
                for e in removed.iter().chain(insert) {
                    if e.ring != Incidence::EMBEDDED
                        && let Some(fs) = self.slot_in(e.cell, Dim::Face)
                    {
                        let n = self.t[Dim::Face.index()].aux.row(fs).len();
                        for k in 0..n {
                            let edge = self.t[Dim::Face.index()].aux.row(fs)[k];
                            if let Some(s) = self.slot_in(edge, Dim::Edge) {
                                self.closure_edges.insert(s);
                            }
                        }
                    }
                }
                self.update_aux(dim, slot, &removed, insert);
            }
            _ => {}
        }
        Ok(())
    }

    /// True if the non-embedded part of the (post-edit) row of `dim`/`slot` still names `x`.
    fn row_names(&self, dim: Dim, slot: usize, x: EntityId) -> bool {
        let row = self.t[dim.index()].down.row(slot);
        if dim == Dim::Face || row.len() <= 16 {
            return row.iter().any(|e| e.cell == x && e.ring != Incidence::EMBEDDED);
        }
        // Volume rows are sorted by (ring, cell, side): one binary search per ring group.
        let mut pos = 0;
        while pos < row.len() {
            let ring = row[pos].ring;
            let end = pos + row[pos..].partition_point(|e| e.ring == ring);
            if ring != Incidence::EMBEDDED && row[pos..end].binary_search_by(|e| e.cell.cmp(&x)).is_ok() {
                return true;
            }
            pos = end;
        }
        false
    }

    fn rebuild_aux(&mut self, dim: Dim, slot: usize) {
        let mut v: Vec<EntityId> = self.t[dim.index()].down.row(slot).iter().filter(|e| e.ring != Incidence::EMBEDDED).map(|e| e.cell).collect();
        v.sort_unstable();
        v.dedup();
        let t = &mut self.t[dim.index()];
        let old = t.aux.row(slot).len();
        let _ = t.aux.splice(slot, 0, old, &v);
    }

    fn update_aux(&mut self, dim: Dim, slot: usize, removed: &[Incidence], inserted: &[Incidence]) {
        if removed.len() + inserted.len() > 48 {
            self.rebuild_aux(dim, slot);
            return;
        }
        for r in removed {
            if r.ring == Incidence::EMBEDDED || self.row_names(dim, slot, r.cell) {
                continue;
            }
            let a = self.t[dim.index()].aux.row(slot);
            if let Ok(i) = a.binary_search(&r.cell) {
                let _ = self.t[dim.index()].aux.splice(slot, i, 1, &[]);
            }
        }
        for i in inserted {
            if i.ring == Incidence::EMBEDDED {
                continue;
            }
            let a = self.t[dim.index()].aux.row(slot);
            if let Err(p) = a.binary_search(&i.cell) {
                let _ = self.t[dim.index()].aux.splice(slot, p, 0, &[i.cell]);
            }
        }
    }

    /// Journaled `splice_down`.
    pub(crate) fn splice_row(&mut self, id: EntityId, pos: usize, remove: usize, insert: &[Incidence]) -> Result<(), TopoError> {
        let (dim, slot) = self.live_any(id)?;
        let removed_start = u32::try_from(self.jdata.len()).map_err(|_| TopoError::IdExhausted)?;
        let removed_len = u32::try_from(remove).map_err(|_| TopoError::IdExhausted)?;
        let inserted_len = u32::try_from(insert.len()).map_err(|_| TopoError::IdExhausted)?;
        let pos32 = u32::try_from(pos).map_err(|_| TopoError::IdExhausted)?;
        self.jdata.extend_from_slice(&self.t[dim.index()].down.row(slot)[pos..pos + remove]);
        if let Err(e) = self.splice_down(dim, slot, id, pos, remove, insert) {
            self.jdata.truncate(removed_start as usize);
            return Err(e);
        }
        if matches!(dim, Dim::Volume | Dim::Body) {
            self.ins_log.extend(insert.iter().map(|e| (id, *e)));
        }
        self.journal.push(Undo::Splice { id, pos: pos32, removed_start, removed_len, inserted_len });
        Ok(())
    }

    /// Replace the whole row of `id`, touching only the differing middle.
    pub(crate) fn set_row(&mut self, id: EntityId, entries: &[Incidence]) -> Result<(), TopoError> {
        let (dim, slot) = self.live_any(id)?;
        let old = self.t[dim.index()].down.row(slot);
        let max = old.len().min(entries.len());
        let p = old.iter().zip(entries).take_while(|(a, b)| a == b).count();
        let s = old[p..].iter().rev().zip(entries[p..].iter().rev()).take(max - p).take_while(|(a, b)| a == b).count();
        let (remove, ins) = (old.len() - p - s, &entries[p..entries.len() - s]);
        // An identical row still journals an empty record: the reference's `set_row` always does,
        // and the journal decides which cells the `end_op` checks cover.
        self.splice_row(id, p, remove, ins)
    }

    // ---------------------------------------------------------------- cells

    pub(crate) fn alloc(&mut self, dim: Dim, carrier: Carrier) -> Result<EntityId, TopoError> {
        let op = self.ensure_open()?;
        let d = dim.index();
        let slot = self.t[d].state.len();
        let id = EntityId::new(dim.kind(), slot as u64 + 1).ok_or(TopoError::IdExhausted)?;
        let t = &mut self.t[d];
        t.state.push(EntityState::Live);
        t.born.push(op);
        t.carrier.push(carrier);
        t.flags.push(CellFlags::NONE);
        t.tol.push(f64::NAN);
        t.down.push_empty();
        t.up.push_empty();
        if matches!(dim, Dim::Face | Dim::Volume) {
            t.aux.push_empty();
        }
        if let Some(c) = t.live.get_mut() {
            c.push(id);
        }
        match dim {
            Dim::Edge => self.radial.push(SmallVec::new()),
            Dim::Face => {
                self.pcurves.push(Vec::new());
                self.pair_by_face.push(Vec::new());
            }
            Dim::Volume => self.shells.push(SmallVec::new()),
            _ => {}
        }
        self.journal.push(Undo::Alloc(id));
        Ok(id)
    }

    /// Check `id` is a live cell of rank `dim`; returns its slot.
    pub(crate) fn live_slot(&self, id: EntityId, dim: Dim) -> Result<usize, TopoError> {
        if Dim::of_real_kind(id.kind()) != Some(dim) {
            return Err(TopoError::WrongDim { id, expected: dim });
        }
        match self.locate(id) {
            None => Err(TopoError::NoSuchCell(id)),
            Some((_, slot)) if self.t[dim.index()].state[slot] == EntityState::Live => Ok(slot),
            Some(_) => Err(TopoError::Dead(id)),
        }
    }

    pub(crate) fn live_any(&self, id: EntityId) -> Result<(Dim, usize), TopoError> {
        let (dim, slot) = self.locate(id).ok_or(TopoError::NoSuchCell(id))?;
        if self.t[dim.index()].state[slot] == EntityState::Live { Ok((dim, slot)) } else { Err(TopoError::Dead(id)) }
    }

    pub(crate) fn note(&mut self, entity: EntityId, kind: LineageKind) {
        let op = self.open.as_ref().map_or(self.revision, |o| o.op);
        self.lineage.push(LineageRecord { op, entity, kind });
    }

    pub(crate) fn set_state(&mut self, id: EntityId, new: EntityState) {
        if let Some((dim, slot)) = self.locate(id) {
            let t = &mut self.t[dim.index()];
            let old = t.state[slot];
            if old == new {
                return;
            }
            t.state[slot] = new;
            if old == EntityState::Live && new != EntityState::Live && t.live_edits < LIVE_EDITS_MAX {
                // A few retirements: edit the cached list (a memmove); many: rebuild once on demand.
                t.live_edits += 1;
                if let Some(c) = t.live.get_mut()
                    && let Ok(i) = c.binary_search(&id)
                {
                    c.remove(i);
                }
            } else if (old == EntityState::Live) != (new == EntityState::Live) {
                t.live.take();
            }
            self.journal.push(Undo::State { id, old });
        }
    }

    pub(crate) fn set_carrier_raw(&mut self, id: EntityId, c: Carrier) {
        if let Some((dim, slot)) = self.locate(id) {
            let old = core::mem::replace(&mut self.t[dim.index()].carrier[slot], c);
            self.journal.push(Undo::Carrier { id, old: Box::new(old) });
        }
    }

    pub(crate) fn set_flags_raw(&mut self, id: EntityId, f: CellFlags) {
        if let Some((dim, slot)) = self.locate(id) {
            let old = core::mem::replace(&mut self.t[dim.index()].flags[slot], f);
            if dim == Dim::Edge && old.contains(CellFlags::DEGENERATE) != f.contains(CellFlags::DEGENERATE) {
                self.closure_edges.insert(slot);
            }
            self.journal.push(Undo::Flags { id, old });
        }
    }

    /// Tolerance override of `id`, if any.
    pub(crate) fn tol_raw(&self, id: EntityId) -> Option<f64> {
        let (d, s) = self.locate(id)?;
        let v = self.t[d.index()].tol[s];
        (!v.is_nan()).then_some(v)
    }

    pub(crate) fn set_tol_raw(&mut self, id: EntityId, t: Option<f64>) {
        if let Some((dim, slot)) = self.locate(id) {
            let new = t.unwrap_or(f64::NAN);
            let old = core::mem::replace(&mut self.t[dim.index()].tol[slot], new);
            let changed = if old.is_nan() { !new.is_nan() } else { old != new };
            if changed {
                self.journal.push(Undo::Tol { id, old });
            }
        }
    }

    // ---------------------------------------------------------------- pcurves

    pub(crate) fn pcurve_raw(&self, k: CoedgeKey) -> Option<&PCurve> {
        let s = self.slot_in(k.face, Dim::Face)?;
        let l = &self.pcurves[s];
        let i = l.binary_search_by(|(e, sd, _)| (*e, *sd).cmp(&(k.edge, k.side))).ok()?;
        Some(&l[i].2)
    }

    /// Put (or remove) a pcurve; returns the previous one.
    fn pcurve_put(&mut self, k: CoedgeKey, pc: Option<PCurve>) -> Option<PCurve> {
        let s = self.slot_in(k.face, Dim::Face)?;
        let l = &mut self.pcurves[s];
        match (l.binary_search_by(|(e, sd, _)| (*e, *sd).cmp(&(k.edge, k.side))), pc) {
            (Ok(i), Some(p)) => Some(core::mem::replace(&mut l[i].2, p)),
            (Ok(i), None) => Some(l.remove(i).2),
            (Err(i), Some(p)) => {
                l.insert(i, (k.edge, k.side, p));
                None
            }
            (Err(_), None) => None,
        }
    }

    pub(crate) fn set_pcurve_raw(&mut self, key: CoedgeKey, pc: Option<PCurve>) {
        if self.slot_in(key.face, Dim::Face).is_none() {
            return;
        }
        let old = self.pcurve_put(key, pc);
        self.journal.push(Undo::PCurve { key, old: old.map(Box::new) });
    }

    pub(crate) fn move_pcurve(&mut self, from: CoedgeKey, to: CoedgeKey) {
        if let Some(p) = self.pcurve_raw(from).cloned() {
            self.set_pcurve_raw(from, None);
            self.set_pcurve_raw(to, Some(p));
        }
    }

    // ---------------------------------------------------------------- shells

    pub(crate) fn shell_raw(&self, volume: EntityId, ring: u16) -> Option<ShellKind> {
        let s = self.slot_in(volume, Dim::Volume)?;
        self.shells[s].get(usize::from(ring)).copied().flatten()
    }

    /// The shell kinds of `volume` as `(ring, kind)`, ascending.
    pub(crate) fn shell_kinds(&self, volume: EntityId) -> SmallVec<[(u16, ShellKind); 4]> {
        match self.slot_in(volume, Dim::Volume) {
            Some(s) => self.shells[s].iter().enumerate().filter_map(|(r, k)| k.map(|k| (r as u16, k))).collect(),
            None => SmallVec::new(),
        }
    }

    fn shell_put(&mut self, volume: EntityId, ring: u16, k: Option<ShellKind>) -> Option<ShellKind> {
        let s = self.slot_in(volume, Dim::Volume)?;
        let v = &mut self.shells[s];
        let r = usize::from(ring);
        if v.len() <= r {
            k?;
            v.resize(r + 1, None);
        }
        let old = core::mem::replace(&mut v[r], k);
        while v.last() == Some(&None) {
            v.pop();
        }
        old
    }

    pub(crate) fn set_shell_raw(&mut self, key: (EntityId, u16), k: Option<ShellKind>) {
        let old = self.shell_put(key.0, key.1, k);
        self.journal.push(Undo::Shell { volume: key.0, ring: key.1, old });
    }

    // ---------------------------------------------------------------- pairs

    pub(crate) fn raw_put_pair(&mut self, p: FacePair) {
        match self.pairs.binary_search_by(|q| (q.a, q.b).cmp(&(p.a, p.b))) {
            Ok(i) => self.pairs[i] = p,
            Err(i) => self.pairs.insert(i, p),
        }
        for f in [p.a, p.b] {
            if let Some(s) = self.slot_in(f, Dim::Face) {
                let v = &mut self.pair_by_face[s];
                match v.binary_search_by(|q| (q.a, q.b).cmp(&(p.a, p.b))) {
                    Ok(i) => v[i] = p,
                    Err(i) => v.insert(i, p),
                }
            }
        }
    }

    pub(crate) fn raw_del_pair(&mut self, a: EntityId, b: EntityId) {
        if let Ok(i) = self.pairs.binary_search_by(|q| (q.a, q.b).cmp(&(a, b))) {
            self.pairs.remove(i);
        }
        for f in [a, b] {
            if let Some(s) = self.slot_in(f, Dim::Face) {
                let v = &mut self.pair_by_face[s];
                if let Ok(i) = v.binary_search_by(|q| (q.a, q.b).cmp(&(a, b))) {
                    v.remove(i);
                }
            }
        }
    }

    pub(crate) fn pair_of(&self, a: EntityId, b: EntityId) -> Option<FacePair> {
        self.pairs.binary_search_by(|q| (q.a, q.b).cmp(&(a, b))).ok().map(|i| self.pairs[i])
    }

    pub(crate) fn put_pair(&mut self, p: FacePair) {
        let old = self.pair_of(p.a, p.b);
        self.journal.push(Undo::Pair { a: p.a, b: p.b, old });
        self.raw_put_pair(p);
    }

    pub(crate) fn del_pair(&mut self, a: EntityId, b: EntityId) {
        let old = self.pair_of(a, b);
        if old.is_some() {
            self.journal.push(Undo::Pair { a, b, old });
            self.raw_del_pair(a, b);
        }
    }

    /// Pair records of face `f`.
    pub(crate) fn pairs_of_face(&self, f: EntityId) -> &[FacePair] {
        match self.slot_in(f, Dim::Face) {
            Some(s) => &self.pair_by_face[s],
            None => &[],
        }
    }

    // ---------------------------------------------------------------- queries used by operators

    pub(crate) fn effective_tol(&self, id: EntityId) -> f64 {
        self.tol_raw(id).unwrap_or(self.tol.linear)
    }

    pub(crate) fn edge_ends_raw(&self, edge: EntityId) -> Option<(EntityId, EntityId)> {
        match self.row(edge) {
            [only] if only.side == Side::Both => Some((only.cell, only.cell)),
            [a, b] if a.side == Side::Left && b.side == Side::Right => Some((a.cell, b.cell)),
            _ => None,
        }
    }

    /// Release the row storage of a retired cell.
    pub(crate) fn release_rows(&mut self, id: EntityId) {
        if let Some((d, s)) = self.locate(id) {
            let t = &mut self.t[d.index()];
            if t.down.row(s).is_empty() {
                t.down.release_row(s);
            }
            if t.up.row(s).is_empty() {
                t.up.release_row(s);
            }
            if matches!(d, Dim::Face | Dim::Volume) && t.aux.row(s).is_empty() {
                t.aux.release_row(s);
            }
        }
    }

    /// Rebuild the live list of `dim` (ascending ids of cells in state `Live`).
    fn collect_live(&self, dim: Dim) -> Vec<EntityId> {
        let t = &self.t[dim.index()];
        t.state.iter().enumerate().filter(|(_, st)| **st == EntityState::Live).filter_map(|(i, _)| EntityId::new(dim.kind(), i as u64 + 1)).collect()
    }
}

impl TopologyView for TopoStore {
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
        self.locate(id).map(|(d, s)| self.t[d.index()].state[s])
    }

    fn cells(&self, dim: Dim) -> &[EntityId] {
        self.t[dim.index()].live.get_or_init(|| self.collect_live(dim))
    }

    fn boundary(&self, id: EntityId) -> &[Incidence] {
        self.row(id)
    }

    fn coboundary(&self, id: EntityId) -> &[Incidence] {
        self.up(id)
    }

    fn carrier(&self, id: EntityId) -> &Carrier {
        match self.locate(id) {
            Some((d, s)) => &self.t[d.index()].carrier[s],
            None => &sm_topo_api::view::NO_CARRIER,
        }
    }

    fn flags(&self, id: EntityId) -> CellFlags {
        match self.locate(id) {
            Some((d, s)) => self.t[d.index()].flags[s],
            None => CellFlags::NONE,
        }
    }

    fn pcurve(&self, k: CoedgeKey) -> Option<&PCurve> {
        self.pcurve_raw(k)
    }

    fn shell_kind(&self, volume: EntityId, ring: u16) -> Option<ShellKind> {
        self.shell_raw(volume, ring)
    }

    fn radial_uses(&self, edge: EntityId) -> &[EdgeUse] {
        match self.locate(edge) {
            Some((Dim::Edge, s)) => &self.radial[s],
            _ => &[],
        }
    }

    fn tolerance_override(&self, id: EntityId) -> Option<f64> {
        self.tol_raw(id)
    }

    fn coincidences(&self) -> &[FacePair] {
        &self.pairs
    }

    fn coincident_faces(&self, f: EntityId) -> &[FacePair] {
        self.pairs_of_face(f)
    }

    fn lineage(&self) -> &LineageLog {
        &self.lineage
    }

    fn born(&self, id: EntityId) -> Option<OpId> {
        self.locate(id).map(|(d, s)| self.t[d.index()].born[s])
    }

    fn revision(&self) -> OpId {
        self.revision
    }

    fn volume_faces(&self, v: EntityId) -> &[EntityId] {
        match self.locate(v) {
            Some((Dim::Volume, s)) => self.t[Dim::Volume.index()].aux.row(s),
            _ => &[],
        }
    }

    fn face_edges(&self, f: EntityId) -> &[EntityId] {
        match self.locate(f) {
            Some((Dim::Face, s)) => self.t[Dim::Face.index()].aux.row(s),
            _ => &[],
        }
    }

    fn canonical_visit(&self, f: &mut dyn FnMut(HashWord)) {
        self.visit_words(&mut |w| f(w));
    }

    fn canonical_hash(&self) -> u64 {
        self.hash_words()
    }
}

const _: fn() = || {
    fn is_send_sync<T: Send + Sync>() {}
    is_send_sync::<TopoStore>();
};
