//! The [`TopologyEdit`] implementation of [`TopoStore`]: operation lifecycle and operators.
//!
//! The operator algorithms follow the reference implementation in `sm-topo-api` step by step
//! (that is what keeps ids, lineage and hashes identical); what differs is the storage they run
//! on and the bookkeeping around them (delta row edits, incremental end-of-op checks).

use crate::norm::lt;
use sm_topo_api::{canonical_rotate, normalize_row};
use crate::store::{TopoStore, Undo};
use sm_core::{EntityId, EntityKind, Point3};
use sm_geom_api::{CurveDef, GeomAppend, GeomStore, GeomStoreMut, Interval, ModelTolerance, PCurve, SurfaceDef};
use sm_topo_api::{
    Carrier, CellFlags, CoedgeKey, CoedgeSpec, Coincidence, CornerKey, CurveCarrier, Dim, EntityState, FaceCut, FacePair, FaceUseSpec, Incidence, InnerLoopAssign, LineageKind, ModKind, OpId, OpSummary,
    PointClass, Sense, ShellAssign, ShellKind, ShellSpec, Side, SurfaceCarrier, TopoError, TopologyEdit, TopologyView, TopologyViewExt,
};
use smallvec::{SmallVec, smallvec};
use std::collections::{BTreeMap, BTreeSet};

type Row = Vec<Incidence>;

/// Group a face row into its non-embedded loops (assumed numbered `0..n` in order) and the
/// embedded members.
fn split_rings(row: &[Incidence]) -> (Vec<Row>, Row) {
    let mut rings: Vec<Row> = Vec::new();
    let mut embedded: Row = Vec::new();
    for g in row.chunk_by(|a, b| a.ring == b.ring) {
        if g[0].ring == Incidence::EMBEDDED {
            embedded.extend_from_slice(g);
        } else {
            rings.push(g.to_vec());
        }
    }
    (rings, embedded)
}

fn reversed_ring(r: &[Incidence]) -> Row {
    r.iter().rev().map(|e| Incidence { side: e.side.flip(), ..*e }).collect()
}

fn with_ring(r: &[Incidence], ring: u16) -> Row {
    r.iter().map(|e| Incidence { ring, ..*e }).collect()
}

impl TopoStore {
    fn tail_of(&self, e: &Incidence) -> Option<EntityId> {
        self.edge_ends_raw(e.cell).map(|(s, t)| if e.side == Side::Right { t } else { s })
    }

    fn head_of(&self, e: &Incidence) -> Option<EntityId> {
        self.edge_ends_raw(e.cell).map(|(s, t)| if e.side == Side::Right { s } else { t })
    }

    /// Validate a closed chain of coedges and return it as ring entries.
    fn chain_entries(&self, ring: usize, chain: &[CoedgeSpec]) -> Result<Row, TopoError> {
        if chain.is_empty() {
            return Err(TopoError::EmptyLoop);
        }
        let entries: Row = chain.iter().map(|c| Incidence::new(c.edge, c.sense.side(), ring as u16)).collect();
        for c in chain {
            self.live_slot(c.edge, Dim::Edge)?;
        }
        for (i, e) in entries.iter().enumerate() {
            let head = self.head_of(e).ok_or(TopoError::WrongDim { id: e.cell, expected: Dim::Edge })?;
            let next = &entries[(i + 1) % entries.len()];
            let tail = self.tail_of(next).ok_or(TopoError::WrongDim { id: next.cell, expected: Dim::Edge })?;
            if head != tail {
                return Err(TopoError::OpenLoop { ring, at: i });
            }
        }
        Ok(entries)
    }

    /// Validate an open chain of coedges (head to tail, not closed, distinct edges) and return
    /// it as ring-0 entries.
    fn open_chain_entries(&self, chain: &[CoedgeSpec]) -> Result<Row, TopoError> {
        if chain.is_empty() {
            return Err(TopoError::EmptyLoop);
        }
        for c in chain {
            self.live_slot(c.edge, Dim::Edge)?;
        }
        let entries: Row = chain.iter().map(|c| Incidence::new(c.edge, c.sense.side(), 0)).collect();
        let mut seen: Vec<EntityId> = entries.iter().map(|e| e.cell).collect();
        seen.sort();
        if seen.windows(2).any(|w| w[0] == w[1]) {
            return Err(TopoError::InvalidParameter("chain uses an edge twice"));
        }
        for (i, w) in entries.windows(2).enumerate() {
            let head = self.head_of(&w[0]).ok_or(TopoError::WrongDim { id: w[0].cell, expected: Dim::Edge })?;
            let tail = self.tail_of(&w[1]).ok_or(TopoError::WrongDim { id: w[1].cell, expected: Dim::Edge })?;
            if head != tail {
                return Err(TopoError::OpenLoop { ring: 0, at: i });
            }
        }
        Ok(entries)
    }

    fn bridge_loops_impl(&mut self, face: EntityId, edge: EntityId) -> Result<(), TopoError> {
        self.live_slot(face, Dim::Face)?;
        self.live_slot(edge, Dim::Edge)?;
        let (es, ee) = self.edge_ends_raw(edge).ok_or(TopoError::WrongDim { id: edge, expected: Dim::Edge })?;
        if es == ee {
            return Err(TopoError::InvalidParameter("bridge_loops needs an open edge"));
        }
        let row = self.row(face).to_vec();
        let (rings, embedded) = split_rings(&row);
        if rings.iter().flatten().any(|e| e.cell == edge) {
            return Err(TopoError::InvalidParameter("edge is already a coedge of the face"));
        }
        // The loop and position (tail vertex) of each end.
        let this: &Self = self;
        let find = |v: EntityId| -> Result<(usize, usize), TopoError> {
            let mut hits = Vec::new();
            for (r, ring) in rings.iter().enumerate() {
                for (i, e) in ring.iter().enumerate() {
                    if this.tail_of(e) == Some(v) {
                        hits.push((r, i));
                    }
                }
            }
            match hits[..] {
                [] => Err(TopoError::ChordNotOnLoop),
                [one] => Ok(one),
                _ => Err(TopoError::Unsupported("bridge end vertex appears more than once in the face's loops")),
            }
        };
        let (rs, ps) = find(es)?;
        let (re, pe) = find(ee)?;
        if rs == re {
            return Err(TopoError::InvalidParameter("bridge ends lie on the same loop: split the face instead"));
        }
        // Merged loop: ring rs from `es`, the edge forward to `ee`, ring re from `ee`, the edge back.
        let mut a = rings[rs].clone();
        a.rotate_left(ps);
        let mut b = rings[re].clone();
        b.rotate_left(pe);
        let mut merged: Row = a;
        merged.push(Incidence::new(edge, Side::Left, 0));
        merged.extend(b);
        merged.push(Incidence::new(edge, Side::Right, 0));
        let (lo, hi) = (rs.min(re), rs.max(re));
        let mut new: Row = Vec::with_capacity(row.len() + 2);
        for (r, ring) in rings.iter().enumerate() {
            let src: &[Incidence] = if r == lo {
                &merged
            } else if r == hi {
                continue;
            } else {
                ring
            };
            let idx = if r > hi { r - 1 } else { r };
            new.extend(with_ring(src, idx as u16));
        }
        new.extend(embedded.into_iter().filter(|e| e.cell != edge));
        normalize_row(Dim::Face, &mut new);
        self.set_row(face, &new)?;
        self.mark_dirty_around(face);
        self.note(face, LineageKind::Modified(ModKind::InnerLoop));
        Ok(())
    }

    fn drop_face_pcurves(&mut self, face: EntityId, row: &[Incidence]) {
        for e in row {
            if e.ring != Incidence::EMBEDDED && e.side != Side::Both {
                let k = CoedgeKey { face, edge: e.cell, side: e.side };
                if self.pcurve_raw(k).is_some() {
                    self.set_pcurve_raw(k, None);
                }
            }
        }
    }

    /// Mark the radial rings that depend on `id`'s geometry or incidences as stale.
    pub(crate) fn mark_dirty_around(&mut self, id: EntityId) {
        let Some((dim, _)) = self.locate(id) else { return };
        match dim {
            Dim::Edge => {
                if let Some(slot) = self.slot_in(id, Dim::Edge) {
                    self.dirty_edges.insert(slot);
                }
            }
            Dim::Face => {
                for k in 0..self.face_edges(id).len() {
                    if let Some(slot) = self.slot_in(self.face_edges(id)[k], Dim::Edge) {
                        self.dirty_edges.insert(slot);
                    }
                }
            }
            Dim::Vertex => {
                for k in 0..self.up(id).len() {
                    if let Some(slot) = self.slot_in(self.up(id)[k].cell, Dim::Edge) {
                        self.dirty_edges.insert(slot);
                    }
                }
            }
            _ => {}
        }
    }

    fn require_increasing(&self, keep: EntityId, drop: EntityId) -> Result<(), TopoError> {
        if keep >= drop { Err(TopoError::KeepNotLowest { keep, drop }) } else { Ok(()) }
    }

    fn gap_allowed(&self, gap: f64, ta: f64, tb: f64) -> bool {
        self.tol.coincident(gap, ta, tb) || self.merge_gap.is_some_and(|g| gap <= g)
    }

    /// Survivor tolerance after absorbing `other_tol` at distance `gap` (noise below the hash
    /// quantum does not grow it); sets an override and logs `Modified(Tolerance)` if it grew.
    fn grow_tolerance(&mut self, keep: EntityId, gap: f64, other_tol: f64) -> Result<(), TopoError> {
        let own = self.effective_tol(keep);
        let g = if gap > self.tol.quantum() { gap } else { 0.0 };
        let wanted = own.max(g + other_tol);
        if wanted > self.tol.linear_max {
            return Err(TopoError::ToleranceTooLarge { wanted, max: self.tol.linear_max });
        }
        if wanted > own && wanted > self.tol.linear {
            self.set_tol_raw(keep, Some(wanted));
            self.note(keep, LineageKind::Modified(ModKind::Tolerance));
        }
        Ok(())
    }

    fn param_at(cc: &CurveCarrier, s: f64) -> f64 {
        let Interval { lo, hi } = cc.range;
        if cc.sense == Sense::Forward { lo + s * (hi - lo) } else { hi - s * (hi - lo) }
    }

    /// The sampled distance between two edge curves. Two curve-less edges have nothing to
    /// measure (0); anything else that cannot be measured is an error, never a silent zero.
    fn curve_gap(&self, keep: EntityId, drop: EntityId, rel: Sense) -> Result<f64, TopoError> {
        const NOT: TopoError = TopoError::Unsupported("gap not measurable");
        let (ck, cd) = match (self.carrier(keep), self.carrier(drop)) {
            (Carrier::Curve(ck), Carrier::Curve(cd)) => (ck, cd),
            (Carrier::None, Carrier::None) => return Ok(0.0),
            _ => return Err(NOT),
        };
        let (Some(kc), Some(dc)) = (self.shapes.curve(ck.curve), self.shapes.curve(cd.curve)) else { return Err(NOT) };
        let mut gap: f64 = 0.0;
        for s in [0.25, 0.5, 0.75] {
            let pd = dc.eval(Self::param_at(cd, s));
            let hint = Self::param_at(ck, if rel == Sense::Forward { s } else { 1.0 - s });
            let pr = kc.project(pd, Some(hint)).map_err(|_| NOT)?;
            if !pr.distance.is_finite() {
                return Err(NOT);
            }
            gap = gap.max(pr.distance);
        }
        Ok(gap)
    }

    /// The distance between two face surfaces at the keep face's outline centroid. Two
    /// surface-less faces have nothing to measure (0); anything else that cannot be measured is
    /// an error.
    fn face_gap(&self, keep: EntityId, drop: EntityId) -> Result<f64, TopoError> {
        const NOT: TopoError = TopoError::Unsupported("gap not measurable");
        let (sk, sd) = match (self.carrier(keep), self.carrier(drop)) {
            (Carrier::Surface(sk), Carrier::Surface(sd)) => (sk, sd),
            (Carrier::None, Carrier::None) => return Ok(0.0),
            _ => return Err(NOT),
        };
        let (Some(ks), Some(ds)) = (self.shapes.surface(sk.surface), self.shapes.surface(sd.surface)) else { return Err(NOT) };
        let pts = self.face_outline(keep);
        if pts.is_empty() {
            return Err(NOT);
        }
        let n = pts.len() as f64;
        let c = Point3::new(pts.iter().map(|p| p.x).sum::<f64>() / n, pts.iter().map(|p| p.y).sum::<f64>() / n, pts.iter().map(|p| p.z).sum::<f64>() / n);
        let q = ks.project(c, None).map_err(|_| NOT)?;
        let d = ds.project(q.point, None).map_err(|_| NOT)?.distance;
        if d.is_finite() { Ok(d) } else { Err(NOT) }
    }

    /// Assign the existing inner loops of a split face to the two children.
    fn assign_inner(&mut self, face: EntityId, inner: &[Row], assign: InnerLoopAssign<'_>, children: [EntityId; 2], prefer_second: bool) -> Result<Vec<u8>, TopoError> {
        if inner.is_empty() {
            return Ok(Vec::new());
        }
        match assign {
            InnerLoopAssign::None => Err(TopoError::InvalidParameter("face has inner loops: give an inner-loop assignment")),
            InnerLoopAssign::Explicit(list) => inner
                .iter()
                .enumerate()
                .map(|(i, _)| {
                    let ring = (i + 1) as u16;
                    list.iter().find(|(r, _)| *r == ring).map(|(_, c)| *c).filter(|c| *c < 2).ok_or(TopoError::InvalidParameter("inner loop without assignment"))
                })
                .collect(),
            InnerLoopAssign::ByPointTest => {
                let Carrier::Surface(sc) = self.carrier(face).clone() else { return Err(TopoError::Unsupported("point test needs a surface carrier")) };
                let surf = self.shapes.surface(sc.surface).ok_or(TopoError::Unsupported("surface not in store"))?;
                let mut out = Vec::with_capacity(inner.len());
                for ring in inner {
                    let tail = ring.iter().find_map(|e| self.tail_of(e)).ok_or(TopoError::Unsupported("empty inner loop"))?;
                    let Carrier::Point(p) = self.carrier(tail) else { return Err(TopoError::Unsupported("vertex without point")) };
                    let uv = surf.project(*p, None)?.uv;
                    let order: [u8; 2] = if prefer_second { [1, 0] } else { [0, 1] };
                    let mut found = None;
                    for c in order {
                        if self.classify_point_on_face(children[usize::from(c)], uv).map_err(|_| TopoError::Unsupported("point test failed"))? == PointClass::In {
                            found = Some(c);
                            break;
                        }
                    }
                    out.push(found.ok_or(TopoError::Unsupported("inner loop not inside either child"))?);
                }
                Ok(out)
            }
            _ => Err(TopoError::Unsupported("inner-loop assignment kind")),
        }
    }

    fn remove_from_bodies_and_kill(&mut self, cell: EntityId) -> Result<(), TopoError> {
        let (dim, _) = self.live_any(cell)?;
        let bodies: Vec<EntityId> = self.up(cell).iter().filter(|e| Dim::of_real_kind(e.cell.kind()) == Some(Dim::Body)).map(|e| e.cell).collect();
        if self.up(cell).iter().any(|e| Dim::of_real_kind(e.cell.kind()) != Some(Dim::Body)) {
            return Err(TopoError::StillReferenced(cell));
        }
        for b in bodies {
            let row: Row = self.row(b).iter().copied().filter(|e| e.cell != cell).collect();
            self.set_row(b, &row)?;
        }
        if dim == Dim::Face {
            let row = self.row(cell).to_vec();
            self.drop_face_pcurves(cell, &row);
        }
        let op = self.ensure_open()?;
        self.retire(cell, EntityState::Deleted(op))?;
        self.note(cell, LineageKind::Deleted);
        Ok(())
    }

    fn split_face_impl(&mut self, face: EntityId, cut: FaceCut<'_>, inner: InnerLoopAssign<'_>) -> Result<[EntityId; 2], TopoError> {
        let op = self.ensure_open()?;
        self.live_slot(face, Dim::Face)?;
        let row = self.row(face).to_vec();
        let (rings, embedded) = split_rings(&row);
        if !embedded.is_empty() {
            return Err(TopoError::Unsupported("split_face with embedded members"));
        }
        let outer = rings.first().cloned().ok_or(TopoError::EmptyLoop)?;
        let inner_rings: Vec<Row> = rings[1..].to_vec();
        let tails: Vec<Option<EntityId>> = outer.iter().map(|e| self.tail_of(e)).collect();
        let positions = |v: EntityId| -> Vec<usize> { tails.iter().enumerate().filter(|(_, t)| **t == Some(v)).map(|(i, _)| i).collect() };
        let on_inner = |v: EntityId| -> bool { inner_rings.iter().any(|r| r.iter().any(|e| self.tail_of(e) == Some(v))) };

        // Outer loops of the two children and the loop (if any) that becomes a new inner loop of
        // the first child.
        // Loop position of a chord/chain end: from its corner if given, else the vertex's only
        // occurrence in the outer loop.
        let n_outer = outer.len();
        let end_position = |v: EntityId, corner: Option<CornerKey>, what: &'static str| -> Result<usize, TopoError> {
            match corner {
                Some(ck) => {
                    if ck.face != face || ck.incoming.face != face {
                        return Err(TopoError::InvalidParameter("corner of another face"));
                    }
                    let i = outer
                        .iter()
                        .position(|e| e.cell == ck.incoming.edge && e.side == ck.incoming.side)
                        .ok_or(TopoError::InvalidParameter("corner coedge is not on the outer loop"))?;
                    let p = (i + 1) % n_outer;
                    if tails[p] != Some(v) {
                        return Err(TopoError::InvalidParameter("corner is not at the cut's end vertex"));
                    }
                    Ok(p)
                }
                None => match positions(v)[..] {
                    [] => Err(TopoError::ChordNotOnLoop),
                    [one] => Ok(one),
                    _ => Err(TopoError::Unsupported(what)),
                },
            }
        };
        type CutParts<'c> = (Option<EntityId>, Option<&'c [CoedgeSpec]>, Option<(CornerKey, CornerKey)>);
        let (chord, chain, corners): CutParts<'_> = match cut {
            FaceCut::Chord { edge } => (Some(edge), None, None),
            FaceCut::ChordAt { edge, start, end } => (Some(edge), None, Some((start, end))),
            FaceCut::Chain { edges } => (None, Some(edges), None),
            FaceCut::ChainAt { edges, start, end } => (None, Some(edges), Some((start, end))),
            _ => (None, None, None),
        };
        let (c1_outer, c2_outer, new_inner, second_is_inside): (Row, Row, Option<Row>, bool) = match (chord, chain, cut) {
            (Some(edge), _, _) => {
                self.live_slot(edge, Dim::Edge)?;
                let (cs, ce) = self.edge_ends_raw(edge).ok_or(TopoError::WrongDim { id: edge, expected: Dim::Edge })?;
                if cs == ce {
                    return Err(TopoError::InvalidParameter("closed edge: cut with FaceCut::ClosedLoop"));
                }
                if on_inner(cs) || on_inner(ce) {
                    return Err(TopoError::Unsupported("chord ending on an inner loop (KEMR)"));
                }
                let what = "chord end vertex appears more than once in the loop (name its corner: FaceCut::ChordAt)";
                let pi = end_position(cs, corners.map(|c| c.0), what)?;
                let pj = end_position(ce, corners.map(|c| c.1), what)?;
                let (a, b) = (pi.min(pj), pi.max(pj));
                // r1 runs a..b and returns from tails[b] to tails[a] over the chord.
                let side1 = if pi > pj { Side::Left } else { Side::Right };
                let mut r1: Row = outer[a..b].to_vec();
                r1.push(Incidence::new(edge, side1, 0));
                let mut r2: Row = outer[b..].iter().chain(outer[..a].iter()).copied().collect();
                r2.push(Incidence::new(edge, side1.flip(), 0));
                (r1, r2, None, false)
            }
            (None, Some(edges), _) => {
                let chain = self.open_chain_entries(edges)?;
                let first = chain.first().ok_or(TopoError::EmptyLoop)?;
                let last = chain.last().ok_or(TopoError::EmptyLoop)?;
                let cs = self.tail_of(first).ok_or(TopoError::WrongDim { id: first.cell, expected: Dim::Edge })?;
                let ce = self.head_of(last).ok_or(TopoError::WrongDim { id: last.cell, expected: Dim::Edge })?;
                if cs == ce {
                    return Err(TopoError::InvalidParameter("closed chain: cut with FaceCut::ClosedLoop"));
                }
                if chain.iter().any(|n| row.iter().any(|e| e.cell == n.cell)) {
                    return Err(TopoError::InvalidParameter("chain edge already used by the face"));
                }
                if on_inner(cs) || on_inner(ce) {
                    return Err(TopoError::Unsupported("chain ending on an inner loop: bridge_loops first"));
                }
                for e in &chain[1..] {
                    let v = self.tail_of(e).ok_or(TopoError::WrongDim { id: e.cell, expected: Dim::Edge })?;
                    if !positions(v).is_empty() || on_inner(v) {
                        return Err(TopoError::InvalidParameter("chain interior vertex lies on a face loop"));
                    }
                }
                let what = "chain end vertex appears more than once in the loop (name its corner: FaceCut::ChainAt)";
                let pi = end_position(cs, corners.map(|c| c.0), what)?;
                let pj = end_position(ce, corners.map(|c| c.1), what)?;
                let (a, b) = (pi.min(pj), pi.max(pj));
                // r1 runs a..b (ending at vb) and must return vb -> va; r2 the opposite.
                let (back, fwd) = if pi < pj { (reversed_ring(&chain), chain.clone()) } else { (chain.clone(), reversed_ring(&chain)) };
                let mut r1: Row = outer[a..b].to_vec();
                r1.extend(back);
                let mut r2: Row = outer[b..].iter().chain(outer[..a].iter()).copied().collect();
                r2.extend(fwd);
                (r1, r2, None, false)
            }
            (None, None, FaceCut::ClosedLoop { edges }) => {
                let chain = self.chain_entries(0, edges)?;
                let chain_tails: Vec<EntityId> = chain.iter().filter_map(|e| self.tail_of(e)).collect();
                let touching: Vec<EntityId> = chain_tails.iter().copied().filter(|v| !positions(*v).is_empty() || on_inner(*v)).collect();
                if touching.is_empty() {
                    // Interior loop: new inner loop of the face, plus a disk bounded by the loop.
                    (outer.clone(), chain.clone(), Some(reversed_ring(&chain)), true)
                } else {
                    let mut vs = touching.clone();
                    vs.sort();
                    vs.dedup();
                    let [v] = vs[..] else { return Err(TopoError::Unsupported("closed loop touching the face loop at several vertices")) };
                    if on_inner(v) {
                        return Err(TopoError::Unsupported("closed loop attached to an inner loop"));
                    }
                    let pv = positions(v);
                    let [a, b] = pv[..] else { return Err(TopoError::Unsupported("closed loop vertex must appear exactly twice in the face loop (periodic seam)")) };
                    // Rotate the chain to start at v.
                    let start = chain.iter().position(|e| self.tail_of(e) == Some(v)).ok_or(TopoError::ChordNotOnLoop)?;
                    let mut ch = chain.clone();
                    ch.rotate_left(start);
                    let mut r1: Row = outer[a..b].to_vec();
                    r1.extend(ch.iter().copied());
                    let mut r2: Row = outer[b..].iter().chain(outer[..a].iter()).copied().collect();
                    r2.extend(reversed_ring(&ch));
                    (r1, r2, None, false)
                }
            }
            _ => return Err(TopoError::Unsupported("face cut kind")),
        };

        let carrier = self.carrier(face).clone();
        let f1 = self.alloc(Dim::Face, carrier.clone())?;
        let f2 = self.alloc(Dim::Face, carrier)?;
        let mut o1 = c1_outer.clone();
        let mut o2 = c2_outer.clone();
        for r in [&mut o1, &mut o2] {
            for e in r.iter_mut() {
                e.ring = 0;
            }
            normalize_row(Dim::Face, r);
        }
        self.set_row(f1, &o1)?;
        self.set_row(f2, &o2)?;
        let assignment = self.assign_inner(face, &inner_rings, inner, [f1, f2], second_is_inside)?;
        // Final rows: outer, then assigned inner loops in original order (renumbered), then the
        // new inner loop (first child, interior case).
        let mut rows = [c1_outer, c2_outer];
        for r in &mut rows {
            for e in r.iter_mut() {
                e.ring = 0;
            }
        }
        let mut next = [1u16, 1u16];
        for (ring, which) in inner_rings.iter().zip(&assignment) {
            let w = usize::from(*which);
            rows[w].extend(with_ring(ring, next[w]));
            next[w] += 1;
        }
        if let Some(n) = &new_inner {
            let r = with_ring(n, next[0]);
            rows[0].extend(r);
        }
        let [mut r1, mut r2] = rows;
        normalize_row(Dim::Face, &mut r1);
        normalize_row(Dim::Face, &mut r2);
        self.set_row(f1, &r1)?;
        self.set_row(f2, &r2)?;
        self.inherit(face, f1);
        self.inherit(face, f2);
        // Pcurves follow their coedges.
        for (child, r) in [(f1, &r1), (f2, &r2)] {
            for e in r.iter() {
                if e.ring != Incidence::EMBEDDED && e.side != Side::Both {
                    self.move_pcurve(CoedgeKey { face, edge: e.cell, side: e.side }, CoedgeKey { face: child, edge: e.cell, side: e.side });
                }
            }
        }
        // Coincidence records move to the children.
        let pairs: Vec<FacePair> = self.pairs_of_face(face).to_vec();
        for p in pairs {
            self.del_pair(p.a, p.b);
            if let Some(other) = p.other(face) {
                for c in [f1, f2] {
                    let (a, b) = if c <= other { (c, other) } else { (other, c) };
                    self.put_pair(FacePair::new(a, b, p.status, op));
                }
            }
        }
        self.replace_in_parents(face, |e| smallvec![Incidence { cell: f1, ..*e }, Incidence { cell: f2, ..*e }])?;
        let leftover = self.row(face).to_vec();
        self.drop_face_pcurves(face, &leftover);
        self.retire(face, EntityState::Split(op))?;
        self.note(face, LineageKind::Split { children: smallvec![f1, f2] });
        Ok([f1, f2])
    }

    fn split_volume_impl(&mut self, volume: EntityId, cut: &[EntityId], assign: ShellAssign<'_>) -> Result<[EntityId; 2], TopoError> {
        let op = self.ensure_open()?;
        self.live_slot(volume, Dim::Volume)?;
        let mut cut_faces: Vec<EntityId> = cut.to_vec();
        cut_faces.sort();
        cut_faces.dedup();
        if cut_faces.is_empty() {
            return Err(TopoError::InvalidParameter("a volume split needs cut faces"));
        }
        let vrow = self.row(volume).to_vec();
        let (outer_row, other_rows): (Row, Row) = vrow.iter().partition(|e| e.ring == 0);
        let mut cut_embedded = Vec::new();
        for e in &other_rows {
            if e.ring == Incidence::EMBEDDED {
                if cut_faces.contains(&e.cell) {
                    cut_embedded.push(e.cell);
                } else {
                    return Err(TopoError::Unsupported("split_volume with embedded members"));
                }
            }
        }
        for &c in &cut_faces {
            self.live_slot(c, Dim::Face)?;
            if self.occupancy(c) != (false, false) {
                return Err(TopoError::SideOccupied { face: c });
            }
            if outer_row.iter().any(|e| e.cell == c) {
                return Err(TopoError::InvalidParameter("cut face already bounds the volume"));
            }
        }
        let cut_edges: BTreeSet<EntityId> = cut_faces.iter().flat_map(|c| self.face_edges(*c).iter().copied()).collect();
        let outer_faces: Vec<EntityId> = {
            let mut v: Vec<EntityId> = outer_row.iter().map(|e| e.cell).collect();
            v.sort();
            v.dedup();
            v
        };
        let mut by_edge: BTreeMap<EntityId, SmallVec<[EntityId; 4]>> = BTreeMap::new();
        for &f in &outer_faces {
            for &e in self.face_edges(f) {
                if !cut_edges.contains(&e) {
                    by_edge.entry(e).or_default().push(f);
                }
            }
        }
        let mut comp_of: BTreeMap<EntityId, usize> = BTreeMap::new();
        let mut comps: Vec<Vec<EntityId>> = Vec::new();
        for &seed in &outer_faces {
            if comp_of.contains_key(&seed) {
                continue;
            }
            let ci = comps.len();
            let mut stack = vec![seed];
            comp_of.insert(seed, ci);
            let mut members = Vec::new();
            while let Some(f) = stack.pop() {
                members.push(f);
                for &e in self.face_edges(f) {
                    if let Some(ns) = by_edge.get(&e) {
                        for &n in ns {
                            if let std::collections::btree_map::Entry::Vacant(v) = comp_of.entry(n) {
                                v.insert(ci);
                                stack.push(n);
                            }
                        }
                    }
                }
            }
            members.sort();
            comps.push(members);
        }
        if comps.len() != 2 {
            return Err(TopoError::CutDoesNotSeparate(comps.len()));
        }
        // Orient each cut face so that part 0 is on its left.
        let use_side = |f: EntityId| outer_row.iter().find(|e| e.cell == f).map(|e| e.side);
        for &c in &cut_faces {
            let crow = self.row(c).to_vec();
            let mut verdict: Option<bool> = None; // part 0 on the cut face's left?
            'edges: for ce in crow.iter().filter(|e| e.ring != Incidence::EMBEDDED) {
                for (pi, comp) in comps.iter().enumerate() {
                    let sharing: Vec<EntityId> = comp.iter().copied().filter(|f| self.face_edges(*f).binary_search(&ce.cell).is_ok()).collect();
                    let [f] = sharing[..] else { continue };
                    let uses: Vec<&Incidence> = self.row(f).iter().filter(|e| e.cell == ce.cell && e.ring != Incidence::EMBEDDED).collect();
                    let [fe] = uses[..] else { continue };
                    let eff = if use_side(f) == Some(Side::Right) { fe.side.flip() } else { fe.side };
                    let part_on_left = ce.side != eff;
                    verdict = Some(if pi == 0 { part_on_left } else { !part_on_left });
                    break 'edges;
                }
            }
            match verdict {
                Some(true) => {}
                Some(false) => self.reverse_inner(c)?,
                None => return Err(TopoError::Unsupported("cut face shares no simple edge with the volume boundary")),
            }
        }
        // Remaining shells.
        let mut shell_rows: Vec<(u16, Row)> = Vec::new();
        for g in other_rows.chunk_by(|a, b| a.ring == b.ring) {
            if g[0].ring != Incidence::EMBEDDED {
                shell_rows.push((g[0].ring, g.to_vec()));
            }
        }
        let mut to_child: BTreeMap<u16, u8> = BTreeMap::new();
        match assign {
            ShellAssign::None if !shell_rows.is_empty() => return Err(TopoError::InvalidParameter("volume has non-outer shells: give a shell assignment")),
            ShellAssign::None => {}
            ShellAssign::Explicit(list) => {
                for (ring, _) in &shell_rows {
                    let c = list.iter().find(|(r, _)| r == ring).map(|(_, c)| *c).filter(|c| *c < 2).ok_or(TopoError::InvalidParameter("shell without assignment"))?;
                    to_child.insert(*ring, c);
                }
            }
            _ => return Err(TopoError::Unsupported("shell assignment kind")),
        }
        // Build the child rows.
        let mut rows: [Row; 2] = [Vec::new(), Vec::new()];
        let mut kinds: [Vec<ShellKind>; 2] = [vec![ShellKind::Outer], vec![ShellKind::Outer]];
        for e in &outer_row {
            let ci = comp_of.get(&e.cell).copied().unwrap_or(0);
            rows[ci].push(Incidence { ring: 0, ..*e });
        }
        for &c in &cut_faces {
            rows[0].push(Incidence::new(c, Side::Left, 0));
            rows[1].push(Incidence::new(c, Side::Right, 0));
        }
        for (ring, entries) in &shell_rows {
            let ci = usize::from(to_child[ring]);
            let new_ring = kinds[ci].len() as u16;
            kinds[ci].push(self.shell_raw(volume, *ring).unwrap_or(ShellKind::Internal));
            rows[ci].extend(with_ring(entries, new_ring));
        }
        for &c in &cut_embedded {
            let row: Row = self.row(volume).iter().copied().filter(|e| e.cell != c).collect();
            self.set_row(volume, &row)?;
        }
        let v0 = self.alloc(Dim::Volume, Carrier::None)?;
        let v1 = self.alloc(Dim::Volume, Carrier::None)?;
        for (v, (mut r, ks)) in [v0, v1].into_iter().zip(rows.into_iter().zip(kinds)) {
            normalize_row(Dim::Volume, &mut r);
            self.set_row(v, &r)?;
            for (i, k) in ks.into_iter().enumerate() {
                self.set_shell_raw((v, i as u16), Some(k));
            }
            self.inherit(volume, v);
        }
        self.replace_in_parents(volume, |e| smallvec![Incidence { cell: v0, ..*e }, Incidence { cell: v1, ..*e }])?;
        self.retire(volume, EntityState::Split(op))?;
        self.note(volume, LineageKind::Split { children: smallvec![v0, v1] });
        Ok([v0, v1])
    }
}

impl TopologyEdit for TopoStore {
    fn remove_shell(&mut self, volume: EntityId, ring: u16) -> Result<(), TopoError> {
        self.atomic(|s| {
            s.live_slot(volume, Dim::Volume)?;
            if ring == 0 || ring == Incidence::EMBEDDED {
                return Err(TopoError::InvalidParameter("only a non-outer shell can be removed"));
            }
            if s.shell_raw(volume, ring).is_none() {
                return Err(TopoError::InvalidParameter("no such shell"));
            }
            let (keep, drop): (Row, Row) = s.row(volume).iter().partition(|e| e.ring != ring);
            s.set_row(volume, &keep)?;
            s.set_shell_raw((volume, ring), None);
            let mut faces: Vec<EntityId> = drop.iter().map(|e| e.cell).collect();
            faces.dedup();
            for f in faces {
                s.normalize_orientation(f)?;
            }
            Ok(())
        })
    }

    fn begin_op(&mut self, op: OpId) -> Result<(), TopoError> {
        self.start_op(op)
    }

    fn end_op(&mut self) -> Result<OpSummary, TopoError> {
        self.finish_op()
    }

    fn abort_op(&mut self) {
        self.cancel_op();
    }

    fn shapes_mut(&mut self) -> Result<&mut dyn GeomAppend, TopoError> {
        self.ensure_open()?;
        Ok(&mut self.shapes)
    }

    fn set_model_tolerance(&mut self, tol: ModelTolerance) -> Result<(), TopoError> {
        self.atomic(|s| {
            if !tol.is_valid() {
                return Err(TopoError::InvalidParameter("tolerance must be finite, positive and linear <= linear_max"));
            }
            let old = s.tol;
            s.journal.push(Undo::ModelTol(old));
            s.tol = tol;
            Ok(())
        })
    }

    fn make_vertex(&mut self, p: Point3) -> Result<EntityId, TopoError> {
        self.atomic(|s| {
            if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
                return Err(TopoError::InvalidParameter("non-finite vertex position"));
            }
            s.alloc(Dim::Vertex, Carrier::Point(p))
        })
    }

    fn make_edge(&mut self, start: EntityId, end: EntityId, curve: Option<CurveCarrier>) -> Result<EntityId, TopoError> {
        self.atomic(|s| {
            s.live_slot(start, Dim::Vertex)?;
            s.live_slot(end, Dim::Vertex)?;
            if let Some(c) = curve {
                if !lt(c.range.lo, c.range.hi) {
                    return Err(TopoError::InvalidParameter("empty curve range"));
                }
                if c.curve.kind() != EntityKind::Curve {
                    return Err(TopoError::InvalidParameter("edge carrier must be a Curve handle"));
                }
            }
            let id = s.alloc(Dim::Edge, curve.map_or(Carrier::None, Carrier::Curve))?;
            let row = if start == end { vec![Incidence::new(start, Side::Both, 0)] } else { vec![Incidence::new(start, Side::Left, 0), Incidence::new(end, Side::Right, 0)] };
            s.set_row(id, &row)?;
            Ok(id)
        })
    }

    fn make_face(&mut self, loops: &[&[CoedgeSpec]], surface: Option<SurfaceCarrier>) -> Result<EntityId, TopoError> {
        self.atomic(|s| {
            if loops.is_empty() {
                return Err(TopoError::EmptyLoop);
            }
            if loops.len() >= usize::from(Incidence::EMBEDDED) {
                return Err(TopoError::InvalidParameter("too many loops"));
            }
            if let Some(sc) = surface
                && sc.surface.kind() != EntityKind::Surface
            {
                return Err(TopoError::InvalidParameter("face carrier must be a Surface handle"));
            }
            let mut row: Row = Vec::new();
            for (ri, l) in loops.iter().enumerate() {
                row.extend(s.chain_entries(ri, l)?);
            }
            let mut keys: Vec<(EntityId, Side)> = row.iter().map(|e| (e.cell, e.side)).collect();
            keys.sort();
            if keys.windows(2).any(|w| w[0] == w[1]) {
                return Err(TopoError::InvalidParameter("an (edge, side) may appear at most once per face"));
            }
            let id = s.alloc(Dim::Face, surface.map_or(Carrier::None, Carrier::Surface))?;
            normalize_row(Dim::Face, &mut row);
            s.set_row(id, &row)?;
            Ok(id)
        })
    }

    fn make_volume(&mut self, shells: &[ShellSpec<'_>]) -> Result<EntityId, TopoError> {
        self.atomic(|s| {
            if shells.len() >= usize::from(Incidence::EMBEDDED) {
                return Err(TopoError::InvalidParameter("too many shells"));
            }
            let mut uses = Vec::new();
            for (ri, sh) in shells.iter().enumerate() {
                if (ri == 0) != (sh.kind == ShellKind::Outer) {
                    return Err(TopoError::InvalidParameter("shell 0 must be Outer, and only shell 0"));
                }
                for u in sh.uses {
                    s.live_slot(u.face, Dim::Face)?;
                    if u.side == Side::Both && sh.kind != ShellKind::Internal {
                        return Err(TopoError::BaffleInClosedShell);
                    }
                    uses.push((u.face, u.side));
                }
            }
            s.check_occupancy(&uses)?;
            let id = s.alloc(Dim::Volume, Carrier::None)?;
            let mut row: Row = shells.iter().enumerate().flat_map(|(ri, sh)| sh.uses.iter().map(move |u| Incidence::new(u.face, u.side, ri as u16))).collect();
            normalize_row(Dim::Volume, &mut row);
            s.set_row(id, &row)?;
            for (ri, sh) in shells.iter().enumerate() {
                s.set_shell_raw((id, ri as u16), Some(sh.kind));
            }
            Ok(id)
        })
    }

    fn make_body(&mut self, members: &[EntityId]) -> Result<EntityId, TopoError> {
        self.atomic(|s| {
            for &m in members {
                let (d, _) = s.live_any(m)?;
                if d == Dim::Body || s.up(m).iter().any(|e| Dim::of_real_kind(e.cell.kind()) == Some(Dim::Body)) {
                    return Err(TopoError::InvalidIncidence { parent: m, child: m });
                }
            }
            let id = s.alloc(Dim::Body, Carrier::None)?;
            let mut row: Row = members.iter().map(|&m| Incidence::new(m, Side::Both, 0)).collect();
            normalize_row(Dim::Body, &mut row);
            s.set_row(id, &row)?;
            Ok(id)
        })
    }

    fn add_shell(&mut self, volume: EntityId, shell: ShellSpec<'_>) -> Result<u16, TopoError> {
        self.atomic(|s| {
            s.live_slot(volume, Dim::Volume)?;
            if shell.kind == ShellKind::Outer {
                return Err(TopoError::InvalidParameter("only shell 0 is Outer"));
            }
            let ring = s.row(volume).iter().filter(|e| e.ring != Incidence::EMBEDDED).map(|e| e.ring + 1).max().unwrap_or(0);
            let ring = ring.max(s.shell_kinds(volume).iter().map(|(r, _)| r + 1).max().unwrap_or(0));
            if ring == 0 {
                return Err(TopoError::InvalidParameter("volume has no outer shell yet"));
            }
            let mut uses = Vec::new();
            for u in shell.uses {
                s.live_slot(u.face, Dim::Face)?;
                if u.side == Side::Both && shell.kind != ShellKind::Internal {
                    return Err(TopoError::BaffleInClosedShell);
                }
                uses.push((u.face, u.side));
            }
            s.check_occupancy(&uses)?;
            let mut row = s.row(volume).to_vec();
            row.extend(shell.uses.iter().map(|u| Incidence::new(u.face, u.side, ring)));
            normalize_row(Dim::Volume, &mut row);
            s.set_row(volume, &row)?;
            s.set_shell_raw((volume, ring), Some(shell.kind));
            Ok(ring)
        })
    }

    fn add_face_use(&mut self, volume: EntityId, ring: u16, use_: FaceUseSpec) -> Result<(), TopoError> {
        self.atomic(|s| {
            s.live_slot(volume, Dim::Volume)?;
            s.live_slot(use_.face, Dim::Face)?;
            let kind = s.shell_raw(volume, ring).ok_or(TopoError::InvalidParameter("no such shell"))?;
            if use_.side == Side::Both && kind != ShellKind::Internal {
                return Err(TopoError::BaffleInClosedShell);
            }
            s.check_occupancy(&[(use_.face, use_.side)])?;
            let mut row = s.row(volume).to_vec();
            row.push(Incidence::new(use_.face, use_.side, ring));
            normalize_row(Dim::Volume, &mut row);
            s.set_row(volume, &row)
        })
    }

    fn remove_face_use(&mut self, volume: EntityId, face: EntityId, side: Side) -> Result<(), TopoError> {
        self.atomic(|s| {
            s.live_slot(volume, Dim::Volume)?;
            let mut found = false;
            let mut row: Row = Vec::new();
            for e in s.row(volume) {
                if e.cell != face || e.ring == Incidence::EMBEDDED {
                    row.push(*e);
                    continue;
                }
                match (side, e.side) {
                    (Side::Both, _) => found = true,
                    (x, y) if x == y => found = true,
                    (x, Side::Both) => {
                        found = true;
                        row.push(Incidence { side: x.flip(), ..*e });
                    }
                    _ => row.push(*e),
                }
            }
            if !found {
                return Err(TopoError::InvalidIncidence { parent: volume, child: face });
            }
            s.set_row(volume, &row)?;
            s.normalize_orientation(face)
        })
    }

    fn embed(&mut self, parent: EntityId, child: EntityId) -> Result<(), TopoError> {
        self.atomic(|s| {
            let (pd, _) = s.live_any(parent)?;
            let (cd, _) = s.live_any(child)?;
            let ok = match pd {
                Dim::Face | Dim::Volume => cd < Dim::Face || (pd == Dim::Volume && cd == Dim::Face),
                Dim::Body => cd < Dim::Body,
                _ => false,
            };
            if !ok || cd >= pd {
                return Err(TopoError::InvalidIncidence { parent, child });
            }
            let ring = if pd == Dim::Body { 0 } else { Incidence::EMBEDDED };
            let mut row = s.row(parent).to_vec();
            row.push(Incidence::new(child, Side::Both, ring));
            normalize_row(pd, &mut row);
            s.set_row(parent, &row)
        })
    }

    fn unembed(&mut self, parent: EntityId, child: EntityId) -> Result<(), TopoError> {
        self.atomic(|s| {
            let (pd, _) = s.live_any(parent)?;
            let keep = |e: &Incidence| !(e.cell == child && (e.ring == Incidence::EMBEDDED || pd == Dim::Body));
            let row: Row = s.row(parent).iter().copied().filter(keep).collect();
            if row.len() == s.row(parent).len() {
                return Err(TopoError::InvalidIncidence { parent, child });
            }
            s.set_row(parent, &row)
        })
    }

    fn split_edge(&mut self, edge: EntityId, at: EntityId, param: f64) -> Result<[EntityId; 2], TopoError> {
        self.atomic(|s| {
            let op = s.ensure_open()?;
            s.live_slot(edge, Dim::Edge)?;
            s.live_slot(at, Dim::Vertex)?;
            if s.flags(edge).contains(CellFlags::DEGENERATE) {
                return Err(TopoError::Unsupported("split of a degenerate edge"));
            }
            let (start, end) = s.edge_ends_raw(edge).ok_or(TopoError::WrongDim { id: edge, expected: Dim::Edge })?;
            if at == start || at == end {
                return Err(TopoError::InvalidParameter("split vertex equals an end vertex"));
            }
            let (c1, c2) = match s.carrier(edge).clone() {
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
            let e1 = s.alloc(Dim::Edge, c1)?;
            let e2 = s.alloc(Dim::Edge, c2)?;
            s.set_row(e1, &[Incidence::new(start, Side::Left, 0), Incidence::new(at, Side::Right, 0)])?;
            s.set_row(e2, &[Incidence::new(at, Side::Left, 0), Incidence::new(end, Side::Right, 0)])?;
            s.inherit(edge, e1);
            s.inherit(edge, e2);
            // Pcurves share the curve parameter, so both halves inherit the coedge's pcurve.
            let uses: Vec<Incidence> = s.up(edge).iter().copied().filter(|u| Dim::of_real_kind(u.cell.kind()) == Some(Dim::Face)).collect();
            for u in uses {
                let old = CoedgeKey { face: u.cell, edge, side: u.side };
                if let Some(p) = s.pcurve_raw(old).cloned() {
                    s.set_pcurve_raw(old, None);
                    s.set_pcurve_raw(CoedgeKey { edge: e1, ..old }, Some(p.clone()));
                    s.set_pcurve_raw(CoedgeKey { edge: e2, ..old }, Some(p));
                }
            }
            s.replace_in_parents(edge, |e| match e.side {
                Side::Right => smallvec![Incidence { cell: e2, ..*e }, Incidence { cell: e1, ..*e }],
                _ => smallvec![Incidence { cell: e1, ..*e }, Incidence { cell: e2, ..*e }],
            })?;
            s.retire(edge, EntityState::Split(op))?;
            s.note(edge, LineageKind::Split { children: smallvec![e1, e2] });
            Ok([e1, e2])
        })
    }

    fn split_face(&mut self, face: EntityId, cut: FaceCut<'_>, inner: InnerLoopAssign<'_>) -> Result<[EntityId; 2], TopoError> {
        self.atomic(|s| s.split_face_impl(face, cut, inner))
    }

    fn split_volume(&mut self, volume: EntityId, cut: &[EntityId], assign: ShellAssign<'_>) -> Result<[EntityId; 2], TopoError> {
        self.atomic(|s| s.split_volume_impl(volume, cut, assign))
    }

    fn bridge_loops(&mut self, face: EntityId, edge: EntityId) -> Result<(), TopoError> {
        self.atomic(|s| s.bridge_loops_impl(face, edge))
    }

    fn make_inner_loop(&mut self, face: EntityId, lp: &[CoedgeSpec]) -> Result<u16, TopoError> {
        self.atomic(|s| {
            s.live_slot(face, Dim::Face)?;
            let row = s.row(face).to_vec();
            let (rings, _) = split_rings(&row);
            let ring = rings.len();
            if ring >= usize::from(Incidence::EMBEDDED) {
                return Err(TopoError::InvalidParameter("too many loops"));
            }
            let entries = s.chain_entries(ring, lp)?;
            if entries.iter().any(|n| row.iter().any(|e| e.cell == n.cell && e.side == n.side && e.ring != Incidence::EMBEDDED)) {
                return Err(TopoError::InvalidParameter("an (edge, side) may appear at most once per face"));
            }
            let mut new = row;
            new.extend(entries);
            normalize_row(Dim::Face, &mut new);
            s.set_row(face, &new)?;
            s.note(face, LineageKind::Modified(ModKind::InnerLoop));
            Ok(ring as u16)
        })
    }

    fn kill_inner_loop(&mut self, face: EntityId, ring: u16) -> Result<(), TopoError> {
        self.atomic(|s| {
            s.live_slot(face, Dim::Face)?;
            if ring == 0 || ring == Incidence::EMBEDDED {
                return Err(TopoError::InvalidParameter("only inner loops (ring >= 1) can be removed"));
            }
            let row = s.row(face).to_vec();
            if !row.iter().any(|e| e.ring == ring) {
                return Err(TopoError::InvalidParameter("no such loop"));
            }
            let mut new: Row = Vec::with_capacity(row.len());
            for e in &row {
                match e.ring {
                    r if r == ring => {}
                    r if r != Incidence::EMBEDDED && r > ring => new.push(Incidence { ring: r - 1, ..*e }),
                    _ => new.push(*e),
                }
            }
            // Pcurves of the removed loop's coedges go with it.
            let gone: Row = row.iter().copied().filter(|e| e.ring == ring).collect();
            s.drop_face_pcurves(face, &gone);
            normalize_row(Dim::Face, &mut new);
            s.set_row(face, &new)?;
            s.note(face, LineageKind::Modified(ModKind::InnerLoop));
            Ok(())
        })
    }

    fn merge_vertices(&mut self, keep: EntityId, drop: EntityId, max_gap: Option<f64>) -> Result<(), TopoError> {
        self.atomic(|s| {
            let op = s.ensure_open()?;
            s.live_slot(keep, Dim::Vertex)?;
            s.live_slot(drop, Dim::Vertex)?;
            s.require_increasing(keep, drop)?;
            if let Some(g) = max_gap {
                if !(g.is_finite() && g >= 0.0) {
                    return Err(TopoError::InvalidParameter("max_gap must be finite and non-negative"));
                }
                if g > s.tol.linear_max {
                    return Err(TopoError::ToleranceTooLarge { wanted: g, max: s.tol.linear_max });
                }
            }
            let (Carrier::Point(a), Carrier::Point(b)) = (s.carrier(keep).clone(), s.carrier(drop).clone()) else {
                return Err(TopoError::InvalidParameter("vertex without a point"));
            };
            let d = (a - b).norm();
            let (tk, td) = (s.effective_tol(keep), s.effective_tol(drop));
            if !(s.tol.coincident(d, tk, td) || max_gap.is_some_and(|g| d <= g)) {
                return Err(TopoError::NotCoincident("vertex positions differ"));
            }
            s.journal.push(Undo::Gap(s.merge_gap));
            s.merge_gap = max_gap.or(s.merge_gap);
            s.grow_tolerance(keep, d, td)?;
            s.replace_in_parents(drop, |e| smallvec![Incidence { cell: keep, ..*e }])?;
            s.retire(drop, EntityState::Merged(op))?;
            s.note(drop, LineageKind::Merged { into: keep, gap: d });
            Ok(())
        })
    }

    fn merge_edges(&mut self, keep: EntityId, drop: EntityId, relative: Sense) -> Result<(), TopoError> {
        self.atomic(|s| {
            let op = s.ensure_open()?;
            s.live_slot(keep, Dim::Edge)?;
            s.live_slot(drop, Dim::Edge)?;
            s.require_increasing(keep, drop)?;
            let (ks, ke) = s.edge_ends_raw(keep).ok_or(TopoError::WrongDim { id: keep, expected: Dim::Edge })?;
            let (ds, de) = s.edge_ends_raw(drop).ok_or(TopoError::WrongDim { id: drop, expected: Dim::Edge })?;
            let ok = if relative == Sense::Forward { ks == ds && ke == de } else { ks == de && ke == ds };
            if !ok {
                return Err(TopoError::NotCoincident("end vertices differ (merge vertices first)"));
            }
            let (tk, td) = (s.effective_tol(keep), s.effective_tol(drop));
            let gap = s.curve_gap(keep, drop, relative)?;
            if !s.gap_allowed(gap, tk, td) {
                return Err(TopoError::NotCoincident("curves are farther apart than the tolerances allow"));
            }
            s.grow_tolerance(keep, gap, td)?;
            // The dropped edge's pcurves refer to its own curve parameter: drop them.
            let uses: Vec<Incidence> = s.up(drop).iter().copied().filter(|u| Dim::of_real_kind(u.cell.kind()) == Some(Dim::Face)).collect();
            for u in uses {
                let k = CoedgeKey { face: u.cell, edge: drop, side: u.side };
                if s.pcurve_raw(k).is_some() {
                    s.set_pcurve_raw(k, None);
                }
            }
            s.replace_in_parents(drop, |e| smallvec![Incidence { cell: keep, side: if relative == Sense::Reversed { e.side.flip() } else { e.side }, ring: e.ring }])?;
            s.retire(drop, EntityState::Merged(op))?;
            s.note(drop, LineageKind::Merged { into: keep, gap });
            Ok(())
        })
    }

    fn merge_faces(&mut self, keep: EntityId, drop: EntityId, relative: Sense) -> Result<(), TopoError> {
        self.atomic(|s| {
            let op = s.ensure_open()?;
            s.live_slot(keep, Dim::Face)?;
            s.live_slot(drop, Dim::Face)?;
            s.require_increasing(keep, drop)?;
            let krow = s.row(keep).to_vec();
            let drow = s.row(drop).to_vec();
            let (kr, k_emb) = split_rings(&krow);
            let (dr, d_emb) = split_rings(&drow);
            // Loop correspondence under `relative`.
            let canon = |rings: &[Row], flip: bool| -> Vec<Vec<(EntityId, Side)>> {
                let mut out: Vec<Vec<(EntityId, Side)>> = rings
                    .iter()
                    .map(|r| {
                        let mut v: Row = if flip { reversed_ring(r) } else { r.clone() };
                        canonical_rotate(&mut v);
                        v.iter().map(|e| (e.cell, e.side)).collect()
                    })
                    .collect();
                out.sort();
                out
            };
            if canon(&kr, false) != canon(&dr, relative == Sense::Reversed) {
                return Err(TopoError::NotCoincident("loops do not correspond (imprint and merge edges first)"));
            }
            let (kl, krr) = s.occupancy(keep);
            let (dl, drr) = s.occupancy(drop);
            let (dl, drr) = if relative == Sense::Reversed { (drr, dl) } else { (dl, drr) };
            if (kl && dl) || (krr && drr) {
                return Err(TopoError::SideOccupied { face: keep });
            }
            let (tk, td) = (s.effective_tol(keep), s.effective_tol(drop));
            let gap = s.face_gap(keep, drop)?;
            if !s.gap_allowed(gap, tk, td) {
                return Err(TopoError::NotCoincident("surfaces are farther apart than the tolerances allow"));
            }
            let wanted = tk.max(td);
            if wanted > s.tol.linear_max {
                return Err(TopoError::ToleranceTooLarge { wanted, max: s.tol.linear_max });
            }
            if wanted > tk && wanted > s.tol.linear {
                s.set_tol_raw(keep, Some(wanted));
                s.note(keep, LineageKind::Modified(ModKind::Tolerance));
            }
            // Carry over the absorbed face's embedded members.
            if d_emb.iter().any(|d| !k_emb.contains(d)) {
                let mut row = krow.clone();
                row.extend(d_emb.iter().copied().filter(|d| !k_emb.contains(d)));
                normalize_row(Dim::Face, &mut row);
                s.set_row(keep, &row)?;
            }
            // Other coincidence records of the dropped face now belong to the survivor.
            let pairs: Vec<FacePair> = s.pairs_of_face(drop).to_vec();
            for p in pairs {
                s.del_pair(p.a, p.b);
                if let Some(other) = p.other(drop)
                    && other != keep
                {
                    let (a, b) = if keep <= other { (keep, other) } else { (other, keep) };
                    s.put_pair(FacePair::new(a, b, p.status, op));
                }
            }
            s.replace_in_parents(drop, |e| smallvec![Incidence { cell: keep, side: if relative == Sense::Reversed { e.side.flip() } else { e.side }, ring: e.ring }])?;
            s.drop_face_pcurves(drop, &drow);
            s.retire(drop, EntityState::Merged(op))?;
            s.note(drop, LineageKind::Merged { into: keep, gap });
            s.put_pair(FacePair::new(keep, drop, Coincidence::Merged, op));
            s.normalize_orientation(keep)
        })
    }

    fn join_edges(&mut self, first: EntityId, second: EntityId) -> Result<EntityId, TopoError> {
        self.atomic(|s| {
            let op = s.ensure_open()?;
            let plan = sm_topo_api::plan_join_edges(&*s, first, second)?;
            for (k, p) in &plan.pcurves {
                if p.is_some() || s.pcurve_raw(*k).is_some() {
                    s.set_pcurve_raw(*k, p.clone());
                }
            }
            for (f, row) in &plan.face_rows {
                s.set_row(*f, row)?;
            }
            s.set_carrier_raw(plan.survivor, plan.carrier.clone());
            s.set_row(plan.survivor, &plan.row)?;
            if let Some(t) = plan.tolerance {
                s.set_tol_raw(plan.survivor, Some(t));
                s.note(plan.survivor, LineageKind::Modified(ModKind::Tolerance));
            }
            s.retire(plan.absorbed, EntityState::Merged(op))?;
            s.note(plan.absorbed, LineageKind::Merged { into: plan.survivor, gap: 0.0 });
            s.note(plan.survivor, LineageKind::Modified(ModKind::Carrier));
            s.remove_from_bodies_and_kill(plan.vertex)?;
            s.mark_dirty_around(plan.survivor);
            Ok(plan.survivor)
        })
    }

    fn join_faces(&mut self, keep: EntityId, drop: EntityId) -> Result<EntityId, TopoError> {
        self.atomic(|s| {
            let op = s.ensure_open()?;
            let plan = sm_topo_api::plan_join_faces(&*s, keep, drop)?;
            // Coincidence records of the absorbed face now belong to the survivor; where `keep`
            // already has a record with the same face, the survivor's record wins.
            let keep_partners: Vec<EntityId> = s.pairs_of_face(keep).iter().filter_map(|p| p.other(keep)).collect();
            let pairs: Vec<FacePair> = s.pairs_of_face(drop).to_vec();
            for p in pairs {
                s.del_pair(p.a, p.b);
                if let Some(other) = p.other(drop)
                    && other != keep
                    && !keep_partners.contains(&other)
                {
                    let (a, b) = if keep <= other { (keep, other) } else { (other, keep) };
                    s.put_pair(FacePair::new(a, b, p.status, op));
                }
            }
            s.replace_in_parents(drop, |_| smallvec![])?;
            for k in &plan.pcurve_drops {
                if s.pcurve_raw(*k).is_some() {
                    s.set_pcurve_raw(*k, None);
                }
            }
            for (from, to) in &plan.pcurve_moves {
                s.move_pcurve(*from, *to);
            }
            s.set_row(keep, &plan.keep_row)?;
            if let Some(t) = plan.tolerance {
                s.set_tol_raw(keep, Some(t));
                s.note(keep, LineageKind::Modified(ModKind::Tolerance));
            }
            s.retire(drop, EntityState::Merged(op))?;
            s.note(drop, LineageKind::Merged { into: keep, gap: 0.0 });
            if plan.inner_loops_changed {
                s.note(keep, LineageKind::Modified(ModKind::InnerLoop));
            }
            for e in &plan.chain_edges {
                s.remove_from_bodies_and_kill(*e)?;
            }
            for v in &plan.chain_vertices {
                s.remove_from_bodies_and_kill(*v)?;
            }
            Ok(keep)
        })
    }

    fn unmerge_vertex(&mut self, v: EntityId, edges: &[EntityId]) -> Result<EntityId, TopoError> {
        self.atomic(|s| {
            s.live_slot(v, Dim::Vertex)?;
            if edges.is_empty() {
                return Err(TopoError::InvalidParameter("unmerge_vertex needs edges"));
            }
            let mut sorted = edges.to_vec();
            sorted.sort();
            if sorted.windows(2).any(|w| w[0] == w[1]) {
                return Err(TopoError::InvalidParameter("an edge is listed twice"));
            }
            for &e in edges {
                s.live_slot(e, Dim::Edge)?;
                if !s.row(e).iter().any(|i| i.cell == v) {
                    return Err(TopoError::InvalidIncidence { parent: e, child: v });
                }
            }
            let carrier = s.carrier(v).clone();
            let copy = s.alloc(Dim::Vertex, carrier)?;
            s.inherit(v, copy);
            for &e in &sorted {
                let row: Row = s.row(e).iter().map(|i| if i.cell == v { Incidence { cell: copy, ..*i } } else { *i }).collect();
                s.set_row(e, &row)?;
                s.mark_dirty_around(e);
            }
            s.note(copy, LineageKind::Created);
            Ok(copy)
        })
    }

    fn unmerge_edge(&mut self, edge: EntityId, faces: &[EntityId]) -> Result<EntityId, TopoError> {
        self.atomic(|s| {
            s.live_slot(edge, Dim::Edge)?;
            if faces.is_empty() {
                return Err(TopoError::InvalidParameter("unmerge_edge needs faces"));
            }
            let mut sorted = faces.to_vec();
            sorted.sort();
            if sorted.windows(2).any(|w| w[0] == w[1]) {
                return Err(TopoError::InvalidParameter("a face is listed twice"));
            }
            for &f in &sorted {
                s.live_slot(f, Dim::Face)?;
                if !s.row(f).iter().any(|i| i.cell == edge && i.ring != Incidence::EMBEDDED) {
                    return Err(TopoError::InvalidIncidence { parent: f, child: edge });
                }
            }
            let carrier = s.carrier(edge).clone();
            let row = s.row(edge).to_vec();
            let copy = s.alloc(Dim::Edge, carrier)?;
            s.set_row(copy, &row)?;
            s.inherit(edge, copy);
            for &f in &sorted {
                let old = s.row(f).to_vec();
                for i in old.iter().filter(|i| i.cell == edge && i.ring != Incidence::EMBEDDED) {
                    s.move_pcurve(CoedgeKey { face: f, edge, side: i.side }, CoedgeKey { face: f, edge: copy, side: i.side });
                }
                let mut new: Row = old.iter().map(|i| if i.cell == edge && i.ring != Incidence::EMBEDDED { Incidence { cell: copy, ..*i } } else { *i }).collect();
                normalize_row(Dim::Face, &mut new);
                s.set_row(f, &new)?;
            }
            s.mark_dirty_around(edge);
            s.mark_dirty_around(copy);
            s.note(copy, LineageKind::Created);
            Ok(copy)
        })
    }

    fn unmerge_face(&mut self, face: EntityId, side: Side) -> Result<EntityId, TopoError> {
        self.atomic(|s| {
            let op = s.ensure_open()?;
            s.live_slot(face, Dim::Face)?;
            if side == Side::Both {
                return Err(TopoError::InvalidParameter("unmerge_face takes Left or Right"));
            }
            let uses: Vec<Incidence> = s.up(face).iter().copied().filter(|i| Dim::of_real_kind(i.cell.kind()) == Some(Dim::Volume)).collect();
            if uses.iter().any(|i| i.side == Side::Both) {
                return Err(TopoError::Unsupported("unmerge_face of a baffle"));
            }
            let Some(u) = uses.iter().find(|i| i.side == side).copied() else {
                return Err(TopoError::InvalidParameter("no volume uses that side of the face"));
            };
            let carrier = s.carrier(face).clone();
            let loops: Row = s.row(face).iter().copied().filter(|i| i.ring != Incidence::EMBEDDED).collect();
            let new = s.alloc(Dim::Face, carrier)?;
            s.set_row(new, &loops)?;
            s.inherit(face, new);
            for i in &loops {
                let k = CoedgeKey { face, edge: i.cell, side: i.side };
                if let Some(p) = s.pcurve_raw(k).cloned() {
                    s.set_pcurve_raw(CoedgeKey { face: new, ..k }, Some(p));
                }
            }
            let mut vrow: Row = s.row(u.cell).iter().map(|i| if i.cell == face && i.side == side && i.ring == u.ring { Incidence { cell: new, ..*i } } else { *i }).collect();
            normalize_row(Dim::Volume, &mut vrow);
            s.set_row(u.cell, &vrow)?;
            // The merge record no longer describes this face.
            let stale: Vec<FacePair> = s.pairs_of_face(face).iter().copied().filter(|p| p.status == Coincidence::Merged && p.other(face).is_some_and(|o| s.state(o) != Some(EntityState::Live))).collect();
            for p in stale {
                s.del_pair(p.a, p.b);
            }
            s.put_pair(FacePair::new(face.min(new), face.max(new), Coincidence::ImprintedNotMerged, op));
            s.note(face.min(new), LineageKind::Modified(ModKind::Coincidence));
            s.mark_dirty_around(face);
            s.mark_dirty_around(new);
            s.note(new, LineageKind::Created);
            Ok(new)
        })
    }

    fn split_degenerate_edge(&mut self, edge: EntityId, param: f64) -> Result<[EntityId; 2], TopoError> {
        self.atomic(|s| {
            let op = s.ensure_open()?;
            s.live_slot(edge, Dim::Edge)?;
            if !s.flags(edge).contains(CellFlags::DEGENERATE) {
                return Err(TopoError::InvalidParameter("not a degenerate edge: use split_edge"));
            }
            let (v, w) = s.edge_ends_raw(edge).ok_or(TopoError::WrongDim { id: edge, expected: Dim::Edge })?;
            if v != w {
                return Err(TopoError::InvalidParameter("a degenerate edge is closed"));
            }
            if !param.is_finite() {
                return Err(TopoError::InvalidParameter("non-finite split parameter"));
            }
            let mut checked = false;
            let (c1, c2) = match s.carrier(edge).clone() {
                Carrier::Curve(c) => {
                    if !(lt(c.range.lo, param) && lt(param, c.range.hi)) {
                        return Err(TopoError::InvalidParameter("split parameter outside edge range"));
                    }
                    checked = true;
                    let (lo, hi) = (Interval::new(c.range.lo, param), Interval::new(param, c.range.hi));
                    let (r1, r2) = if c.sense == Sense::Forward { (lo, hi) } else { (hi, lo) };
                    (Carrier::Curve(CurveCarrier { range: r1, ..c }), Carrier::Curve(CurveCarrier { range: r2, ..c }))
                }
                other => (other.clone(), other),
            };
            // Split every pole pcurve at `param` (its parameter is the edge parameter).
            let uses: Vec<Incidence> = s.up(edge).iter().copied().filter(|u| Dim::of_real_kind(u.cell.kind()) == Some(Dim::Face)).collect();
            let mut halves: Vec<(CoedgeKey, PCurve, PCurve)> = Vec::new();
            for u in &uses {
                let k = CoedgeKey { face: u.cell, edge, side: u.side };
                let Some(p) = s.pcurve_raw(k) else { continue };
                let PCurve::Line2(l) = p else { return Err(TopoError::Unsupported("split of a non-Line2 pole pcurve")) };
                if !(lt(l.range.lo, param) && lt(param, l.range.hi)) {
                    return Err(TopoError::InvalidParameter("split parameter outside a pole pcurve range"));
                }
                checked = true;
                let a = PCurve::Line2(sm_geom_api::Line2 { range: Interval::new(l.range.lo, param), ..*l });
                let b = PCurve::Line2(sm_geom_api::Line2 { range: Interval::new(param, l.range.hi), ..*l });
                halves.push((k, a, b));
            }
            if !checked {
                return Err(TopoError::InvalidParameter("nothing to split at the parameter: the edge has no curve and no pcurve"));
            }
            let e1 = s.alloc(Dim::Edge, c1)?;
            let e2 = s.alloc(Dim::Edge, c2)?;
            s.set_row(e1, &[Incidence::new(v, Side::Both, 0)])?;
            s.set_row(e2, &[Incidence::new(v, Side::Both, 0)])?;
            s.inherit(edge, e1);
            s.inherit(edge, e2);
            for (k, a, b) in halves {
                s.set_pcurve_raw(k, None);
                s.set_pcurve_raw(CoedgeKey { edge: e1, ..k }, Some(a));
                s.set_pcurve_raw(CoedgeKey { edge: e2, ..k }, Some(b));
            }
            s.replace_in_parents(edge, |e| match e.side {
                Side::Right => smallvec![Incidence { cell: e2, ..*e }, Incidence { cell: e1, ..*e }],
                _ => smallvec![Incidence { cell: e1, ..*e }, Incidence { cell: e2, ..*e }],
            })?;
            s.retire(edge, EntityState::Split(op))?;
            s.note(edge, LineageKind::Split { children: smallvec![e1, e2] });
            Ok([e1, e2])
        })
    }

    fn scale_tolerances(&mut self, cells: &[EntityId], factor: f64) -> Result<(), TopoError> {
        self.atomic(|s| {
            if !(factor.is_finite() && factor > 0.0) {
                return Err(TopoError::InvalidParameter("scale factor must be finite and positive"));
            }
            let mut ids = cells.to_vec();
            ids.sort();
            ids.dedup();
            for &c in &ids {
                s.live_any(c)?;
            }
            for c in ids {
                let Some(t) = s.tol_raw(c) else { continue };
                let scaled = (t * factor).min(s.tol.linear_max);
                let new = if scaled <= s.tol.linear { None } else { Some(scaled) };
                if new != Some(t) {
                    s.set_tol_raw(c, new);
                    s.note(c, LineageKind::Modified(ModKind::Tolerance));
                }
            }
            Ok(())
        })
    }

    fn reverse_face(&mut self, face: EntityId) -> Result<(), TopoError> {
        self.atomic(|s| {
            s.live_slot(face, Dim::Face)?;
            s.reverse_inner(face)
        })
    }

    fn kill(&mut self, cell: EntityId) -> Result<(), TopoError> {
        self.atomic(|s| {
            let was_face = Dim::of_real_kind(cell.kind()) == Some(Dim::Face);
            let vols: Vec<EntityId> = if was_face { s.up(cell).iter().map(|e| e.cell).collect() } else { Vec::new() };
            // A face used by a volume cannot be killed without first removing the uses.
            if vols.iter().any(|v| Dim::of_real_kind(v.kind()) == Some(Dim::Volume)) {
                return Err(TopoError::StillReferenced(cell));
            }
            s.remove_from_bodies_and_kill(cell)
        })
    }

    fn kill_cascade(&mut self, cell: EntityId) -> Result<SmallVec<[EntityId; 16]>, TopoError> {
        self.atomic(|s| {
            let (top, _) = s.live_any(cell)?;
            let mut cand: BTreeSet<EntityId> = s.row(cell).iter().map(|e| e.cell).collect();
            let mut killed: SmallVec<[EntityId; 16]> = SmallVec::new();
            s.remove_from_bodies_and_kill(cell)?;
            killed.push(cell);
            // Faces' lower cells are processed after the faces: volume -> face -> edge -> vertex.
            for dim in [Dim::Volume, Dim::Face, Dim::Edge, Dim::Vertex] {
                if dim >= top {
                    continue;
                }
                let level: Vec<EntityId> = cand.iter().copied().filter(|c| Dim::of_real_kind(c.kind()) == Some(dim)).collect();
                for c in level {
                    cand.remove(&c);
                    if s.state(c) != Some(EntityState::Live) || !s.up(c).is_empty() {
                        continue;
                    }
                    cand.extend(s.row(c).iter().map(|e| e.cell));
                    s.remove_from_bodies_and_kill(c)?;
                    killed.push(c);
                }
            }
            Ok(killed)
        })
    }

    fn set_vertex_point(&mut self, v: EntityId, p: Point3) -> Result<(), TopoError> {
        self.atomic(|s| {
            s.live_slot(v, Dim::Vertex)?;
            if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
                return Err(TopoError::InvalidParameter("non-finite vertex position"));
            }
            s.set_carrier_raw(v, Carrier::Point(p));
            s.mark_dirty_around(v);
            s.note(v, LineageKind::Modified(ModKind::Point));
            Ok(())
        })
    }

    fn set_carrier(&mut self, cell: EntityId, carrier: Carrier) -> Result<(), TopoError> {
        self.atomic(|s| {
            let (d, _) = s.live_any(cell)?;
            let ok = matches!(
                (d, &carrier),
                (Dim::Vertex, Carrier::Point(_)) | (Dim::Edge, Carrier::Curve(_) | Carrier::None) | (Dim::Face, Carrier::Surface(_) | Carrier::None) | (Dim::Volume, Carrier::Csg(_) | Carrier::None) | (Dim::Body, Carrier::None)
            );
            if !ok {
                return Err(TopoError::WrongDim { id: cell, expected: d });
            }
            let kinds_ok = match &carrier {
                Carrier::Curve(c) => c.curve.kind() == EntityKind::Curve,
                Carrier::Surface(sc) => sc.surface.kind() == EntityKind::Surface,
                Carrier::Csg(cell) => cell.halfspaces.iter().all(|h| h.surface.kind() == EntityKind::Surface),
                _ => true,
            };
            if !kinds_ok {
                return Err(TopoError::InvalidParameter("carrier handle has the wrong kind"));
            }
            s.set_carrier_raw(cell, carrier);
            s.mark_dirty_around(cell);
            s.note(cell, LineageKind::Modified(ModKind::Carrier));
            Ok(())
        })
    }

    fn set_pcurve(&mut self, key: CoedgeKey, pcurve: Option<PCurve>) -> Result<(), TopoError> {
        self.atomic(|s| {
            s.live_slot(key.face, Dim::Face)?;
            if !s.row(key.face).iter().any(|e| e.cell == key.edge && e.side == key.side && e.ring != Incidence::EMBEDDED) {
                return Err(TopoError::InvalidIncidence { parent: key.face, child: key.edge });
            }
            s.set_pcurve_raw(key, pcurve);
            Ok(())
        })
    }

    fn replace_curve(&mut self, id: EntityId, def: CurveDef) -> Result<(), TopoError> {
        self.atomic(|s| {
            let old = s.shapes.replace_curve(id, def)?;
            s.journal.push(Undo::Curve { id, old: Box::new(old) });
            let edges: Vec<EntityId> = s.cells(Dim::Edge).iter().copied().filter(|e| matches!(s.carrier(*e), Carrier::Curve(c) if c.curve == id)).collect();
            for e in edges {
                if let Some(slot) = s.slot_in(e, Dim::Edge) {
                    s.dirty_edges.insert(slot);
                }
            }
            s.note(id, LineageKind::Modified(ModKind::Curve));
            Ok(())
        })
    }

    fn replace_surface(&mut self, id: EntityId, def: SurfaceDef) -> Result<(), TopoError> {
        self.atomic(|s| {
            let old = s.shapes.replace_surface(id, def)?;
            s.journal.push(Undo::Surface { id, old: Box::new(old) });
            // Faces carried by the surface and CSG cells with a half-space on it.
            let uses = |c: &Carrier| match c {
                Carrier::Surface(c) => c.surface == id,
                Carrier::Csg(cell) => cell.halfspaces.iter().any(|h| h.surface == id),
                _ => false,
            };
            let mut hit: Vec<EntityId> = Vec::new();
            for dim in [Dim::Face, Dim::Volume] {
                hit.extend(s.cells(dim).iter().copied().filter(|c| uses(s.carrier(*c))));
            }
            for c in hit {
                s.mark_dirty_around(c);
            }
            s.note(id, LineageKind::Modified(ModKind::Surface));
            Ok(())
        })
    }

    fn raise_tolerance(&mut self, cell: EntityId, tol: f64) -> Result<(), TopoError> {
        self.atomic(|s| {
            s.live_any(cell)?;
            if !tol.is_finite() {
                return Err(TopoError::InvalidParameter("tolerance must be finite"));
            }
            if tol > s.tol.linear_max {
                return Err(TopoError::ToleranceTooLarge { wanted: tol, max: s.tol.linear_max });
            }
            if tol <= s.effective_tol(cell) {
                return Err(TopoError::InvalidParameter("tolerances only grow"));
            }
            s.set_tol_raw(cell, Some(tol));
            s.note(cell, LineageKind::Modified(ModKind::Tolerance));
            Ok(())
        })
    }

    fn set_flags(&mut self, cell: EntityId, flags: CellFlags) -> Result<(), TopoError> {
        self.atomic(|s| {
            s.live_any(cell)?;
            s.set_flags_raw(cell, flags);
            s.mark_dirty_around(cell);
            Ok(())
        })
    }

    fn record_coincidence(&mut self, a: EntityId, b: EntityId, status: Coincidence) -> Result<(), TopoError> {
        self.atomic(|s| {
            let op = s.ensure_open()?;
            s.live_slot(a, Dim::Face)?;
            s.live_slot(b, Dim::Face)?;
            if a == b || status == Coincidence::Merged {
                return Err(TopoError::InvalidParameter("pair must be two faces and not Merged"));
            }
            let (a, b) = if a <= b { (a, b) } else { (b, a) };
            if s.coincidence(a, b).is_some_and(|p| p.status == Coincidence::Merged) {
                return Err(TopoError::InvalidParameter("pair is already merged"));
            }
            s.put_pair(FacePair::new(a, b, status, op));
            s.note(a, LineageKind::Modified(ModKind::Coincidence));
            Ok(())
        })
    }

    fn record_split(&mut self, parent: EntityId, children: &[EntityId]) -> Result<(), TopoError> {
        self.atomic(|s| {
            let op = s.ensure_open()?;
            if children.is_empty() {
                return Err(TopoError::InvalidParameter("a split needs children"));
            }
            match s.state(parent) {
                Some(EntityState::Deleted(o)) if o == op => {}
                Some(EntityState::Live) => return Err(TopoError::InvalidParameter("the parent must be retired in this operation")),
                None => return Err(TopoError::NoSuchCell(parent)),
                Some(_) => return Err(TopoError::InvalidParameter("only a parent deleted in this operation can be recorded as split")),
            }
            let pd = Dim::of_real_kind(parent.kind());
            for &c in children {
                if Dim::of_real_kind(c.kind()) != pd {
                    return Err(TopoError::InvalidParameter("children must have the parent's rank"));
                }
                s.live_any(c)?;
            }
            s.set_state(parent, EntityState::Split(op));
            s.note(parent, LineageKind::Split { children: children.iter().copied().collect() });
            Ok(())
        })
    }

    fn record_merged(&mut self, drop: EntityId, into: EntityId, gap: f64) -> Result<(), TopoError> {
        self.atomic(|s| {
            let op = s.ensure_open()?;
            if !(gap.is_finite() && gap >= 0.0) {
                return Err(TopoError::InvalidParameter("the merge gap must be finite and non-negative"));
            }
            if drop == into {
                return Err(TopoError::InvalidParameter("a cell cannot merge into itself"));
            }
            match s.state(drop) {
                Some(EntityState::Deleted(o)) if o == op => {}
                Some(EntityState::Live) => return Err(TopoError::InvalidParameter("the merged cell must be retired in this operation")),
                None => return Err(TopoError::NoSuchCell(drop)),
                Some(_) => return Err(TopoError::InvalidParameter("only a cell deleted in this operation can be recorded as merged")),
            }
            if Dim::of_real_kind(into.kind()) != Dim::of_real_kind(drop.kind()) {
                return Err(TopoError::InvalidParameter("the survivor must have the merged cell's rank"));
            }
            s.live_any(into)?;
            s.set_state(drop, EntityState::Merged(op));
            s.note(drop, LineageKind::Merged { into, gap });
            Ok(())
        })
    }

    fn record_generated(&mut self, cell: EntityId, from_surface: EntityId) -> Result<(), TopoError> {
        self.atomic(|s| {
            s.live_slot(cell, Dim::Face)?;
            if from_surface.kind() != EntityKind::Surface {
                return Err(TopoError::InvalidParameter("from_surface must be a Surface handle"));
            }
            s.note(cell, LineageKind::Generated { from_surface });
            Ok(())
        })
    }

    fn record_created(&mut self, cell: EntityId) -> Result<(), TopoError> {
        self.atomic(|s| {
            s.live_any(cell)?;
            s.note(cell, LineageKind::Created);
            Ok(())
        })
    }
}
