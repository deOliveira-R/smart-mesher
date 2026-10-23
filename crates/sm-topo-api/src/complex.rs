//! [`CellComplex`]: the reference in-memory implementor of [`TopologyView`] and [`TopologyEdit`].
//!
//! Storage (all that is stored):
//! - per rank, a table of cells indexed by serial: `alive`, `born`, `carrier`, `flags`, and one
//!   boundary row each (a "paged" CSR: `(start, len)` spans into one flat `Vec<Incidence>`);
//! - sparse tolerance overrides, face-pair records, the lineage log, the current op.
//!
//! The coboundary (transpose), live-cell lists and every loop/shell/use view are *derived*:
//! built lazily from the boundary rows into a `OnceLock` and dropped by every edit.

use crate::derived::TopologyViewExt;
use crate::edit::TopologyEdit;
use crate::error::TopoError;
use crate::types::{
    Carrier, CellFlags, Coincidence, CoedgeSpec, CurveCarrier, Dim, FacePair, FaceUseSpec, Incidence, LineageKind, LineageRecord, OpId, Region, Sense,
    Side, SurfaceCarrier,
};
use crate::view::TopologyView;
use sm_core::{EntityId, Point3};
use sm_geom_api::{Interval, Tolerance};
use smallvec::{SmallVec, smallvec};
use std::collections::BTreeMap;
use std::sync::OnceLock;

/// Paged CSR rows: row `slot` lives at `data[span.0 .. span.0 + span.1]`. Replacing a row appends
/// the new one and orphans the old; `compact` rewrites in slot order (deterministic).
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

    fn set(&mut self, slot: usize, entries: &[Incidence]) -> Result<(), TopoError> {
        let start = u32::try_from(self.data.len()).map_err(|_| TopoError::IdExhausted)?;
        let len = u32::try_from(entries.len()).map_err(|_| TopoError::IdExhausted)?;
        let Some(sp) = self.span.get_mut(slot) else { return Err(TopoError::IdExhausted) };
        self.garbage += sp.1 as usize;
        *sp = (start, len);
        self.data.extend_from_slice(entries);
        if self.garbage > 4096 && self.garbage * 2 > self.data.len() {
            self.compact()?;
        }
        Ok(())
    }

    fn compact(&mut self) -> Result<(), TopoError> {
        let mut data = Vec::with_capacity(self.data.len() - self.garbage.min(self.data.len()));
        let mut span = Vec::with_capacity(self.span.len());
        for slot in 0..self.span.len() {
            let start = u32::try_from(data.len()).map_err(|_| TopoError::IdExhausted)?;
            let row = self.row(slot);
            data.extend_from_slice(row);
            span.push((start, row.len() as u32));
        }
        self.data = data;
        self.span = span;
        self.garbage = 0;
        Ok(())
    }
}

#[derive(Clone, Debug, Default)]
struct Table {
    alive: Vec<bool>,
    born: Vec<OpId>,
    carrier: Vec<Carrier>,
    flags: Vec<CellFlags>,
    down: Rows,
}

/// Lazily derived indexes: live-cell lists and the transposed (coboundary) rows.
#[derive(Clone, Debug, Default)]
struct Derived {
    live: [Vec<EntityId>; 5],
    up: [Rows; 5],
}

/// The reference cellular non-manifold complex.
///
/// Implements [`TopologyView`] and [`TopologyEdit`]. Equal edit sequences give equal ids and equal
/// [`TopologyView::canonical_hash`].
#[derive(Clone, Debug)]
pub struct CellComplex {
    tol: Tolerance,
    tables: [Table; 5],
    tol_override: BTreeMap<EntityId, f64>,
    pairs: Vec<FacePair>,
    lineage: Vec<LineageRecord>,
    op: OpId,
    derived: OnceLock<Derived>,
}

impl Default for CellComplex {
    fn default() -> Self {
        Self::new()
    }
}

impl CellComplex {
    /// An empty complex with the default tolerance policy.
    pub fn new() -> Self {
        Self::with_tolerance(Tolerance::DEFAULT)
    }

    /// An empty complex with the given global tolerance policy.
    pub fn with_tolerance(tol: Tolerance) -> Self {
        Self {
            tol,
            tables: core::array::from_fn(|_| Table::default()),
            tol_override: BTreeMap::new(),
            pairs: Vec::new(),
            lineage: Vec::new(),
            op: OpId(0),
            derived: OnceLock::new(),
        }
    }

    // ---------------------------------------------------------------- internals

    fn locate(&self, id: EntityId) -> Option<(Dim, usize)> {
        let dim = Dim::of_kind(id.kind())?;
        let slot = usize::try_from(id.serial()).ok()?;
        (slot < self.tables[dim.index()].alive.len()).then_some((dim, slot))
    }

    fn derived(&self) -> &Derived {
        self.derived.get_or_init(|| self.build_derived())
    }

    fn invalidate(&mut self) {
        self.derived = OnceLock::new();
    }

    fn build_derived(&self) -> Derived {
        let mut d = Derived::default();
        for dim in Dim::ALL {
            let t = &self.tables[dim.index()];
            for (slot, alive) in t.alive.iter().enumerate() {
                if *alive && let Some(id) = EntityId::new(dim.kind(), slot as u64) {
                    d.live[dim.index()].push(id);
                }
            }
            d.up[dim.index()].span = vec![(0, 0); t.alive.len()];
        }
        // Parents are visited in ascending id order and the sort below is stable, so every
        // coboundary row comes out ascending by parent id.
        let mut tuples: Vec<(usize, usize, Incidence)> = Vec::new();
        for dim in Dim::ALL {
            for &parent in &d.live[dim.index()] {
                let Some((_, pslot)) = self.locate(parent) else { continue };
                for e in self.tables[dim.index()].down.row(pslot) {
                    if let Some((cd, cs)) = self.locate(e.cell) {
                        tuples.push((cd.index(), cs, Incidence { cell: parent, side: e.side, ring: e.ring }));
                    }
                }
            }
        }
        tuples.sort_by_key(|t| (t.0, t.1));
        for (cd, cs, inc) in tuples {
            let rows = &mut d.up[cd];
            let sp = &mut rows.span[cs];
            if sp.1 == 0 {
                sp.0 = rows.data.len() as u32;
            }
            sp.1 += 1;
            rows.data.push(inc);
        }
        d
    }

    fn alloc(&mut self, dim: Dim, carrier: Carrier) -> Result<EntityId, TopoError> {
        let t = &mut self.tables[dim.index()];
        let id = EntityId::new(dim.kind(), t.alive.len() as u64).ok_or(TopoError::IdExhausted)?;
        t.alive.push(true);
        t.born.push(self.op);
        t.carrier.push(carrier);
        t.flags.push(CellFlags::NONE);
        t.down.push_empty();
        self.invalidate();
        Ok(id)
    }

    /// Check `id` is a live cell of rank `dim`; returns its slot.
    fn live_slot(&self, id: EntityId, dim: Dim) -> Result<usize, TopoError> {
        if Dim::of_kind(id.kind()) != Some(dim) {
            return Err(TopoError::WrongDim { id, expected: dim });
        }
        match self.locate(id) {
            None => Err(TopoError::NoSuchCell(id)),
            Some((_, slot)) if self.tables[dim.index()].alive[slot] => Ok(slot),
            Some(_) => Err(TopoError::Dead(id)),
        }
    }

    fn live_any(&self, id: EntityId) -> Result<(Dim, usize), TopoError> {
        let (dim, slot) = self.locate(id).ok_or(TopoError::NoSuchCell(id))?;
        if self.tables[dim.index()].alive[slot] { Ok((dim, slot)) } else { Err(TopoError::Dead(id)) }
    }

    fn set_row(&mut self, id: EntityId, entries: &[Incidence]) -> Result<(), TopoError> {
        let (dim, slot) = self.live_any(id)?;
        self.tables[dim.index()].down.set(slot, entries)?;
        self.invalidate();
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
            Some((d, s)) => self.derived().up[d.index()].row(s),
            None => &[],
        }
    }

    fn note(&mut self, kind: LineageKind, from: &[EntityId], to: &[EntityId]) {
        self.lineage.push(LineageRecord { op: self.op, kind, from: from.iter().copied().collect(), to: to.iter().copied().collect() });
    }

    /// Mark dead and clear the row. Does not touch parents or lineage.
    fn retire(&mut self, id: EntityId) -> Result<(), TopoError> {
        self.set_row(id, &[])?;
        if let Some((d, s)) = self.locate(id) {
            self.tables[d.index()].alive[s] = false;
        }
        self.tol_override.remove(&id);
        self.pairs.retain(|p| p.status == Coincidence::Merged || (p.a != id && p.b != id));
        self.invalidate();
        Ok(())
    }

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
        TopologyView::tolerance(self, id)
    }

    fn inherit(&mut self, from: EntityId, to: EntityId) {
        if let Some((d, s)) = self.locate(from)
            && let Some((d2, s2)) = self.locate(to)
        {
            let fl = self.tables[d.index()].flags[s];
            self.tables[d2.index()].flags[s2] = fl;
        }
        if let Some(t) = self.tol_override.get(&from).copied() {
            self.tol_override.insert(to, t);
        }
    }

    fn raise_tolerance(&mut self, keep: EntityId, other_tol: f64) {
        let t = self.effective_tol(keep).max(other_tol);
        if t > self.tol.linear {
            self.tol_override.insert(keep, t);
        }
    }

    fn occupancy(&self, face: EntityId) -> (bool, bool) {
        let s = self.face_sides(face);
        (s.left != Region::Exterior, s.right != Region::Exterior)
    }

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
        self.replace_in_parents(face, |e| smallvec![Incidence { side: e.side.flip(), ..*e }])?;
        self.set_row(face, &out)?;
        if let Some((_, s)) = self.locate(face)
            && let Carrier::Surface(sc) = self.tables[Dim::Face.index()].carrier[s]
        {
            self.tables[Dim::Face.index()].carrier[s] = Carrier::Surface(SurfaceCarrier { sense: sc.sense.flip(), ..sc });
        }
        self.note(LineageKind::Modified, &[face], &[face]);
        Ok(())
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
}

/// `a < b`, false if either is NaN.
fn lt(a: f64, b: f64) -> bool {
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
    fn model_tolerance(&self) -> Tolerance {
        self.tol
    }

    fn dim_of(&self, id: EntityId) -> Option<Dim> {
        self.locate(id).map(|(d, _)| d)
    }

    fn is_alive(&self, id: EntityId) -> bool {
        self.locate(id).is_some_and(|(d, s)| self.tables[d.index()].alive[s])
    }

    fn cells(&self, dim: Dim) -> &[EntityId] {
        &self.derived().live[dim.index()]
    }

    fn boundary(&self, id: EntityId) -> &[Incidence] {
        if self.is_alive(id) { self.row(id) } else { &[] }
    }

    fn coboundary(&self, id: EntityId) -> &[Incidence] {
        if self.is_alive(id) { self.up(id) } else { &[] }
    }

    fn carrier(&self, id: EntityId) -> Carrier {
        match self.locate(id) {
            Some((d, s)) => self.tables[d.index()].carrier[s],
            None => Carrier::None,
        }
    }

    fn flags(&self, id: EntityId) -> CellFlags {
        match self.locate(id) {
            Some((d, s)) => self.tables[d.index()].flags[s],
            None => CellFlags::NONE,
        }
    }

    fn tolerance_override(&self, id: EntityId) -> Option<f64> {
        self.tol_override.get(&id).copied()
    }

    fn coincidences(&self) -> &[FacePair] {
        &self.pairs
    }

    fn lineage(&self) -> &[LineageRecord] {
        &self.lineage
    }

    fn born(&self, id: EntityId) -> Option<OpId> {
        self.locate(id).map(|(d, s)| self.tables[d.index()].born[s])
    }

    fn revision(&self) -> OpId {
        self.op
    }
}

impl TopologyEdit for CellComplex {
    fn begin_op(&mut self, op: OpId) {
        self.op = op;
    }

    fn set_model_tolerance(&mut self, tol: Tolerance) -> Result<(), TopoError> {
        if !tol.is_valid() {
            return Err(TopoError::InvalidParameter("tolerance must be finite and positive"));
        }
        self.tol = tol;
        Ok(())
    }

    fn make_vertex(&mut self, p: Point3) -> Result<EntityId, TopoError> {
        if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
            return Err(TopoError::InvalidParameter("non-finite vertex position"));
        }
        self.alloc(Dim::Vertex, Carrier::Point(p))
    }

    fn make_edge(&mut self, start: EntityId, end: EntityId, curve: Option<CurveCarrier>) -> Result<EntityId, TopoError> {
        self.live_slot(start, Dim::Vertex)?;
        self.live_slot(end, Dim::Vertex)?;
        if let Some(c) = curve
            && !lt(c.range.lo, c.range.hi)
        {
            return Err(TopoError::InvalidParameter("empty curve range"));
        }
        let id = self.alloc(Dim::Edge, curve.map_or(Carrier::None, Carrier::Curve))?;
        let row = if start == end { vec![Incidence::new(start, Side::Both, 0)] } else { vec![Incidence::new(start, Side::Left, 0), Incidence::new(end, Side::Right, 0)] };
        self.set_row(id, &row)?;
        Ok(id)
    }

    fn make_face(&mut self, loops: &[&[CoedgeSpec]], surface: Option<SurfaceCarrier>) -> Result<EntityId, TopoError> {
        if loops.is_empty() {
            return Err(TopoError::EmptyLoop);
        }
        if loops.len() >= usize::from(Incidence::EMBEDDED) {
            return Err(TopoError::InvalidParameter("too many loops"));
        }
        let ends_of = |c: &CoedgeSpec| -> Result<(EntityId, EntityId), TopoError> {
            self.live_slot(c.edge, Dim::Edge)?;
            let (s, e) = self.edge_ends(c.edge).ok_or(TopoError::WrongDim { id: c.edge, expected: Dim::Edge })?;
            Ok(if c.sense == Sense::Forward { (s, e) } else { (e, s) })
        };
        for (ri, l) in loops.iter().enumerate() {
            if l.is_empty() {
                return Err(TopoError::EmptyLoop);
            }
            for (i, c) in l.iter().enumerate() {
                let (_, head) = ends_of(c)?;
                let (next_tail, _) = ends_of(&l[(i + 1) % l.len()])?;
                if head != next_tail {
                    return Err(TopoError::OpenLoop { ring: ri, at: i });
                }
            }
        }
        let id = self.alloc(Dim::Face, surface.map_or(Carrier::None, Carrier::Surface))?;
        let mut row = Vec::new();
        for (ri, l) in loops.iter().enumerate() {
            row.extend(l.iter().map(|c| Incidence::new(c.edge, c.sense.side(), ri as u16)));
        }
        Self::normalize(Dim::Face, &mut row);
        self.set_row(id, &row)?;
        Ok(id)
    }

    fn make_volume(&mut self, shells: &[&[FaceUseSpec]]) -> Result<EntityId, TopoError> {
        if shells.len() >= usize::from(Incidence::EMBEDDED) {
            return Err(TopoError::InvalidParameter("too many shells"));
        }
        let mut uses = Vec::new();
        for s in shells {
            for u in *s {
                self.live_slot(u.face, Dim::Face)?;
                uses.push((u.face, u.side));
            }
        }
        self.check_occupancy(&uses)?;
        let id = self.alloc(Dim::Volume, Carrier::None)?;
        let mut row: Vec<Incidence> = shells.iter().enumerate().flat_map(|(ri, s)| s.iter().map(move |u| Incidence::new(u.face, u.side, ri as u16))).collect();
        Self::normalize(Dim::Volume, &mut row);
        self.set_row(id, &row)?;
        Ok(id)
    }

    fn make_body(&mut self, members: &[EntityId]) -> Result<EntityId, TopoError> {
        for &m in members {
            let (d, _) = self.live_any(m)?;
            if d == Dim::Body || self.up(m).iter().any(|e| self.dim_of(e.cell) == Some(Dim::Body)) {
                return Err(TopoError::InvalidIncidence { parent: m, child: m });
            }
        }
        let id = self.alloc(Dim::Body, Carrier::None)?;
        let mut row: Vec<Incidence> = members.iter().map(|&m| Incidence::new(m, Side::Both, 0)).collect();
        Self::normalize(Dim::Body, &mut row);
        self.set_row(id, &row)?;
        Ok(id)
    }

    fn add_face_use(&mut self, volume: EntityId, ring: u16, use_: FaceUseSpec) -> Result<(), TopoError> {
        self.live_slot(volume, Dim::Volume)?;
        self.live_slot(use_.face, Dim::Face)?;
        if ring == Incidence::EMBEDDED {
            return Err(TopoError::InvalidParameter("reserved ring"));
        }
        self.check_occupancy(&[(use_.face, use_.side)])?;
        let mut row = self.row(volume).to_vec();
        row.push(Incidence::new(use_.face, use_.side, ring));
        Self::normalize(Dim::Volume, &mut row);
        self.set_row(volume, &row)
    }

    fn remove_face_use(&mut self, volume: EntityId, face: EntityId) -> Result<(), TopoError> {
        self.live_slot(volume, Dim::Volume)?;
        let row: Vec<Incidence> = self.row(volume).iter().copied().filter(|e| e.cell != face).collect();
        if row.len() == self.row(volume).len() {
            return Err(TopoError::InvalidIncidence { parent: volume, child: face });
        }
        self.set_row(volume, &row)
    }

    fn embed(&mut self, parent: EntityId, child: EntityId) -> Result<(), TopoError> {
        let (pd, _) = self.live_any(parent)?;
        let (cd, _) = self.live_any(child)?;
        let ok = match pd {
            Dim::Face | Dim::Volume => cd < Dim::Face,
            Dim::Body => cd < Dim::Body,
            _ => false,
        };
        if !ok || cd >= pd {
            return Err(TopoError::InvalidIncidence { parent, child });
        }
        let ring = if pd == Dim::Body { 0 } else { Incidence::EMBEDDED };
        let mut row = self.row(parent).to_vec();
        row.push(Incidence::new(child, Side::Both, ring));
        Self::normalize(pd, &mut row);
        self.set_row(parent, &row)
    }

    fn unembed(&mut self, parent: EntityId, child: EntityId) -> Result<(), TopoError> {
        let (pd, _) = self.live_any(parent)?;
        let keep = |e: &Incidence| !(e.cell == child && (e.ring == Incidence::EMBEDDED || pd == Dim::Body));
        let row: Vec<Incidence> = self.row(parent).iter().copied().filter(keep).collect();
        if row.len() == self.row(parent).len() {
            return Err(TopoError::InvalidIncidence { parent, child });
        }
        self.set_row(parent, &row)
    }

    fn split_edge(&mut self, edge: EntityId, at: EntityId, param: f64) -> Result<[EntityId; 2], TopoError> {
        self.live_slot(edge, Dim::Edge)?;
        self.live_slot(at, Dim::Vertex)?;
        if self.flags(edge).contains(CellFlags::DEGENERATE) {
            return Err(TopoError::Unsupported("split of a degenerate edge"));
        }
        let (start, end) = self.edge_ends(edge).ok_or(TopoError::WrongDim { id: edge, expected: Dim::Edge })?;
        if at == start || at == end {
            return Err(TopoError::InvalidParameter("split vertex equals an end vertex"));
        }
        let (c1, c2) = match self.carrier(edge) {
            Carrier::Curve(c) => {
                if !(lt(c.range.lo, param) && lt(param, c.range.hi)) {
                    return Err(TopoError::InvalidParameter("split parameter outside edge range"));
                }
                let (lo, hi) = (Interval::new(c.range.lo, param), Interval::new(param, c.range.hi));
                let (r1, r2) = if c.sense == Sense::Forward { (lo, hi) } else { (hi, lo) };
                (Carrier::Curve(CurveCarrier { range: r1, ..c }), Carrier::Curve(CurveCarrier { range: r2, ..c }))
            }
            _ => (Carrier::None, Carrier::None),
        };
        let e1 = self.alloc(Dim::Edge, c1)?;
        let e2 = self.alloc(Dim::Edge, c2)?;
        self.set_row(e1, &[Incidence::new(start, Side::Left, 0), Incidence::new(at, Side::Right, 0)])?;
        self.set_row(e2, &[Incidence::new(at, Side::Left, 0), Incidence::new(end, Side::Right, 0)])?;
        self.inherit(edge, e1);
        self.inherit(edge, e2);
        self.replace_in_parents(edge, |e| match e.side {
            Side::Left => smallvec![Incidence { cell: e1, ..*e }, Incidence { cell: e2, ..*e }],
            Side::Right => smallvec![Incidence { cell: e2, ..*e }, Incidence { cell: e1, ..*e }],
            Side::Both => smallvec![Incidence { cell: e1, ..*e }, Incidence { cell: e2, ..*e }],
        })?;
        self.retire(edge)?;
        self.note(LineageKind::Split, &[edge], &[e1, e2]);
        Ok([e1, e2])
    }

    fn split_face(&mut self, face: EntityId, chord: EntityId) -> Result<[EntityId; 2], TopoError> {
        self.live_slot(face, Dim::Face)?;
        self.live_slot(chord, Dim::Edge)?;
        let row = self.row(face).to_vec();
        if row.iter().any(|e| e.ring == Incidence::EMBEDDED) {
            return Err(TopoError::Unsupported("split_face with embedded members"));
        }
        if row.iter().any(|e| e.ring != 0) {
            return Err(TopoError::Unsupported("split_face on a face with holes"));
        }
        let (cs, ce) = self.edge_ends(chord).ok_or(TopoError::WrongDim { id: chord, expected: Dim::Edge })?;
        if cs == ce {
            return Err(TopoError::ChordNotOnLoop);
        }
        let mut tails = Vec::with_capacity(row.len());
        for e in &row {
            let (s, t) = self.edge_ends(e.cell).ok_or(TopoError::WrongDim { id: e.cell, expected: Dim::Edge })?;
            tails.push(if e.side == Side::Right { t } else { s });
        }
        let pi = tails.iter().position(|v| *v == cs).ok_or(TopoError::ChordNotOnLoop)?;
        let pj = tails.iter().position(|v| *v == ce).ok_or(TopoError::ChordNotOnLoop)?;
        let (a, b) = (pi.min(pj), pi.max(pj));
        let (va, vb) = (tails[a], tails[b]);
        // Child 1 runs a..b then closes b -> a along the chord; child 2 runs b..a (wrapping) and closes a -> b.
        let chord_side_1 = if cs == vb && ce == va { Side::Left } else { Side::Right };
        let mut r1: Vec<Incidence> = row[a..b].to_vec();
        r1.push(Incidence::new(chord, chord_side_1, 0));
        let mut r2: Vec<Incidence> = row[b..].iter().chain(row[..a].iter()).copied().collect();
        r2.push(Incidence::new(chord, chord_side_1.flip(), 0));
        Self::normalize(Dim::Face, &mut r1);
        Self::normalize(Dim::Face, &mut r2);
        let carrier = self.carrier(face);
        let f1 = self.alloc(Dim::Face, carrier)?;
        let f2 = self.alloc(Dim::Face, carrier)?;
        self.set_row(f1, &r1)?;
        self.set_row(f2, &r2)?;
        self.inherit(face, f1);
        self.inherit(face, f2);
        self.replace_in_parents(face, |e| smallvec![Incidence { cell: f1, ..*e }, Incidence { cell: f2, ..*e }])?;
        self.retire(face)?;
        self.note(LineageKind::Split, &[face], &[f1, f2]);
        Ok([f1, f2])
    }

    fn merge_vertices(&mut self, keep: EntityId, drop: EntityId) -> Result<(), TopoError> {
        self.live_slot(keep, Dim::Vertex)?;
        self.live_slot(drop, Dim::Vertex)?;
        if keep == drop {
            return Err(TopoError::InvalidParameter("cannot merge a cell with itself"));
        }
        if let (Carrier::Point(a), Carrier::Point(b)) = (self.carrier(keep), self.carrier(drop))
            && (a - b).norm() > self.effective_tol(keep) + self.effective_tol(drop)
        {
            return Err(TopoError::NotCoincident("vertex positions differ"));
        }
        let other = self.effective_tol(drop);
        self.replace_in_parents(drop, |e| smallvec![Incidence { cell: keep, ..*e }])?;
        self.raise_tolerance(keep, other);
        self.retire(drop)?;
        self.note(LineageKind::Merged, &[keep, drop], &[keep]);
        Ok(())
    }

    fn merge_edges(&mut self, keep: EntityId, drop: EntityId, relative: Sense) -> Result<(), TopoError> {
        self.live_slot(keep, Dim::Edge)?;
        self.live_slot(drop, Dim::Edge)?;
        if keep == drop {
            return Err(TopoError::InvalidParameter("cannot merge a cell with itself"));
        }
        let (ks, ke) = self.edge_ends(keep).ok_or(TopoError::WrongDim { id: keep, expected: Dim::Edge })?;
        let (ds, de) = self.edge_ends(drop).ok_or(TopoError::WrongDim { id: drop, expected: Dim::Edge })?;
        let ok = if relative == Sense::Forward { ks == ds && ke == de } else { ks == de && ke == ds };
        if !ok {
            return Err(TopoError::NotCoincident("end vertices differ (merge vertices first)"));
        }
        let other = self.effective_tol(drop);
        self.replace_in_parents(drop, |e| smallvec![Incidence { cell: keep, side: if relative == Sense::Reversed { e.side.flip() } else { e.side }, ring: e.ring }])?;
        self.raise_tolerance(keep, other);
        self.retire(drop)?;
        self.note(LineageKind::Merged, &[keep, drop], &[keep]);
        Ok(())
    }

    fn merge_faces(&mut self, keep: EntityId, drop: EntityId, relative: Sense) -> Result<(), TopoError> {
        self.live_slot(keep, Dim::Face)?;
        self.live_slot(drop, Dim::Face)?;
        if keep == drop {
            return Err(TopoError::InvalidParameter("cannot merge a cell with itself"));
        }
        let edges = |f: EntityId| {
            let mut v: Vec<EntityId> = self.row(f).iter().map(|e| e.cell).collect();
            v.sort();
            v
        };
        if edges(keep) != edges(drop) {
            return Err(TopoError::NotCoincident("loops consist of different edges (imprint and merge edges first)"));
        }
        let (kl, kr) = self.occupancy(keep);
        let (dl, dr) = self.occupancy(drop);
        let (dl, dr) = if relative == Sense::Reversed { (dr, dl) } else { (dl, dr) };
        if (kl && dl) || (kr && dr) {
            return Err(TopoError::SideOccupied { face: keep });
        }
        self.replace_in_parents(drop, |e| smallvec![Incidence { cell: keep, side: if relative == Sense::Reversed { e.side.flip() } else { e.side }, ring: e.ring }])?;
        self.retire(drop)?;
        self.note(LineageKind::Merged, &[keep, drop], &[keep]);
        let (a, b) = if keep <= drop { (keep, drop) } else { (drop, keep) };
        let at = self.pairs.partition_point(|p| (p.a, p.b) < (a, b));
        self.pairs.insert(at, FacePair { a, b, status: Coincidence::Merged, overlap_area: None });
        let s = self.face_sides(keep);
        if let (Region::Volume(l), Region::Volume(r)) = (s.left, s.right)
            && l > r
        {
            self.reverse_inner(keep)?;
        }
        Ok(())
    }

    fn reverse_face(&mut self, face: EntityId) -> Result<(), TopoError> {
        self.live_slot(face, Dim::Face)?;
        self.reverse_inner(face)
    }

    fn kill(&mut self, cell: EntityId) -> Result<(), TopoError> {
        self.live_any(cell)?;
        let bodies: Vec<EntityId> = self.up(cell).iter().filter(|e| self.dim_of(e.cell) == Some(Dim::Body)).map(|e| e.cell).collect();
        if self.up(cell).iter().any(|e| self.dim_of(e.cell) != Some(Dim::Body)) {
            return Err(TopoError::StillReferenced(cell));
        }
        for b in bodies {
            let row: Vec<Incidence> = self.row(b).iter().copied().filter(|e| e.cell != cell).collect();
            self.set_row(b, &row)?;
        }
        self.retire(cell)?;
        self.note(LineageKind::Deleted, &[cell], &[]);
        Ok(())
    }

    fn set_carrier(&mut self, cell: EntityId, carrier: Carrier) -> Result<(), TopoError> {
        let (d, s) = self.live_any(cell)?;
        let ok = matches!(
            (d, carrier),
            (Dim::Vertex, Carrier::Point(_)) | (Dim::Edge, Carrier::Curve(_) | Carrier::None) | (Dim::Face, Carrier::Surface(_) | Carrier::None) | (Dim::Volume | Dim::Body, Carrier::None)
        );
        if !ok {
            return Err(TopoError::WrongDim { id: cell, expected: d });
        }
        self.tables[d.index()].carrier[s] = carrier;
        self.note(LineageKind::Modified, &[cell], &[cell]);
        Ok(())
    }

    fn set_tolerance(&mut self, cell: EntityId, tol: Option<f64>) -> Result<(), TopoError> {
        self.live_any(cell)?;
        match tol {
            Some(t) if !(t.is_finite() && t > 0.0) => return Err(TopoError::InvalidParameter("tolerance must be finite and positive")),
            Some(t) => {
                self.tol_override.insert(cell, t);
            }
            None => {
                self.tol_override.remove(&cell);
            }
        }
        Ok(())
    }

    fn set_flags(&mut self, cell: EntityId, flags: CellFlags) -> Result<(), TopoError> {
        let (d, s) = self.live_any(cell)?;
        self.tables[d.index()].flags[s] = flags;
        Ok(())
    }

    fn record_coincidence(&mut self, a: EntityId, b: EntityId, status: Coincidence, area: Option<f64>) -> Result<(), TopoError> {
        self.live_slot(a, Dim::Face)?;
        self.live_slot(b, Dim::Face)?;
        if a == b || status == Coincidence::Merged {
            return Err(TopoError::InvalidParameter("pair must be two faces and not Merged"));
        }
        let (a, b) = if a <= b { (a, b) } else { (b, a) };
        let rec = FacePair { a, b, status, overlap_area: area };
        match self.pairs.binary_search_by(|p| (p.a, p.b).cmp(&(a, b))) {
            Ok(i) => self.pairs[i] = rec,
            Err(i) => self.pairs.insert(i, rec),
        }
        Ok(())
    }
}
