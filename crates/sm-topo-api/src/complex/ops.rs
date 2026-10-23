//! The [`TopologyEdit`] implementation of [`CellComplex`]: operation lifecycle and operators.

use super::*;
use crate::edit::TopologyEdit;
use crate::geometry::PointClass;
use crate::types::{CellFlags, Coincidence, CoedgeSpec, CurveCarrier, FaceCut, FaceUseSpec, InnerLoopAssign, ShellAssign, ShellSpec};
use sm_core::{EntityKind, Point3};
use sm_geom_api::Interval;
use smallvec::smallvec;

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

impl CellComplex {
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
        Self::normalize(Dim::Face, &mut new);
        self.set_row(face, &new)?;
        self.mark_dirty_around(face);
        self.note(face, LineageKind::Modified(ModKind::InnerLoop));
        Ok(())
    }

    fn drop_face_pcurves(&mut self, face: EntityId, row: &[Incidence]) {
        for e in row {
            if e.ring != Incidence::EMBEDDED && e.side != Side::Both {
                let k = CoedgeKey { face, edge: e.cell, side: e.side };
                if self.pcurves.contains_key(&k) {
                    self.set_pcurve_raw(k, None);
                }
            }
        }
    }

    /// Mark the radial rings that depend on `id`'s geometry or incidences as stale.
    fn mark_dirty_around(&mut self, id: EntityId) {
        let Some((dim, _)) = self.locate(id) else { return };
        match dim {
            Dim::Edge => {
                self.dirty_edges.insert(id);
            }
            Dim::Face => {
                let es: Vec<EntityId> = self.face_edges(id).to_vec();
                self.dirty_edges.extend(es);
            }
            Dim::Vertex => {
                let es: Vec<EntityId> = self.up(id).iter().map(|e| e.cell).collect();
                self.dirty_edges.extend(es);
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

    fn curve_gap(&self, keep: EntityId, drop: EntityId, rel: Sense) -> f64 {
        let (Carrier::Curve(ck), Carrier::Curve(cd)) = (self.carrier(keep), self.carrier(drop)) else { return 0.0 };
        let (Some(kc), Some(dc)) = (self.shapes.curve(ck.curve), self.shapes.curve(cd.curve)) else { return 0.0 };
        let mut gap: f64 = 0.0;
        for s in [0.25, 0.5, 0.75] {
            let pd = dc.eval(Self::param_at(cd, s));
            let hint = Self::param_at(ck, if rel == Sense::Forward { s } else { 1.0 - s });
            if let Ok(pr) = kc.project(pd, Some(hint)) {
                gap = gap.max(pr.distance);
            }
        }
        gap
    }

    fn face_gap(&self, keep: EntityId, drop: EntityId) -> f64 {
        let (Carrier::Surface(sk), Carrier::Surface(sd)) = (self.carrier(keep), self.carrier(drop)) else { return 0.0 };
        let (Some(ks), Some(ds)) = (self.shapes.surface(sk.surface), self.shapes.surface(sd.surface)) else { return 0.0 };
        let pts = self.face_outline(keep);
        if pts.is_empty() {
            return 0.0;
        }
        let n = pts.len() as f64;
        let c = Point3::new(pts.iter().map(|p| p.x).sum::<f64>() / n, pts.iter().map(|p| p.y).sum::<f64>() / n, pts.iter().map(|p| p.z).sum::<f64>() / n);
        let Ok(q) = ks.project(c, None) else { return 0.0 };
        ds.project(q.point, None).map_or(0.0, |p| p.distance)
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
        let (c1_outer, c2_outer, new_inner, second_is_inside): (Row, Row, Option<Row>, bool) = match cut {
            FaceCut::Chord { edge } => {
                self.live_slot(edge, Dim::Edge)?;
                let (cs, ce) = self.edge_ends_raw(edge).ok_or(TopoError::WrongDim { id: edge, expected: Dim::Edge })?;
                if cs == ce {
                    return Err(TopoError::InvalidParameter("closed edge: cut with FaceCut::ClosedLoop"));
                }
                if on_inner(cs) || on_inner(ce) {
                    return Err(TopoError::Unsupported("chord ending on an inner loop (KEMR)"));
                }
                let (pi, pj) = (positions(cs), positions(ce));
                if pi.is_empty() || pj.is_empty() {
                    return Err(TopoError::ChordNotOnLoop);
                }
                if pi.len() != 1 || pj.len() != 1 {
                    return Err(TopoError::Unsupported("chord end vertex appears more than once in the loop"));
                }
                let (a, b) = (pi[0].min(pj[0]), pi[0].max(pj[0]));
                let (va, vb) = (tails[a], tails[b]);
                let side1 = if Some(cs) == vb && Some(ce) == va { Side::Left } else { Side::Right };
                let mut r1: Row = outer[a..b].to_vec();
                r1.push(Incidence::new(edge, side1, 0));
                let mut r2: Row = outer[b..].iter().chain(outer[..a].iter()).copied().collect();
                r2.push(Incidence::new(edge, side1.flip(), 0));
                (r1, r2, None, false)
            }
            FaceCut::Chain { edges } => {
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
                let (pi, pj) = (positions(cs), positions(ce));
                if pi.is_empty() || pj.is_empty() {
                    return Err(TopoError::ChordNotOnLoop);
                }
                if pi.len() != 1 || pj.len() != 1 {
                    return Err(TopoError::Unsupported("chain end vertex appears more than once in the loop"));
                }
                let (a, b) = (pi[0].min(pj[0]), pi[0].max(pj[0]));
                // r1 runs a..b (ending at vb) and must return vb -> va; r2 the opposite.
                let (back, fwd) = if pi[0] < pj[0] { (reversed_ring(&chain), chain.clone()) } else { (chain.clone(), reversed_ring(&chain)) };
                let mut r1: Row = outer[a..b].to_vec();
                r1.extend(back);
                let mut r2: Row = outer[b..].iter().chain(outer[..a].iter()).copied().collect();
                r2.extend(fwd);
                (r1, r2, None, false)
            }
            FaceCut::ClosedLoop { edges } => {
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
            Self::normalize(Dim::Face, r);
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
        Self::normalize(Dim::Face, &mut r1);
        Self::normalize(Dim::Face, &mut r2);
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
        let pairs: Vec<FacePair> = self.coincident_faces(face).to_vec();
        for p in pairs {
            self.del_pair(p.a, p.b);
            if let Some(other) = p.other(face) {
                for c in [f1, f2] {
                    let (a, b) = if c <= other { (c, other) } else { (other, c) };
                    self.put_pair(FacePair { a, b, status: p.status, op });
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
            kinds[ci].push(self.shells.get(&(volume, *ring)).copied().unwrap_or(ShellKind::Internal));
            rows[ci].extend(with_ring(entries, new_ring));
        }
        for &c in &cut_embedded {
            let row: Row = self.row(volume).iter().copied().filter(|e| e.cell != c).collect();
            self.set_row(volume, &row)?;
        }
        let v0 = self.alloc(Dim::Volume, Carrier::None)?;
        let v1 = self.alloc(Dim::Volume, Carrier::None)?;
        for (v, (mut r, ks)) in [v0, v1].into_iter().zip(rows.into_iter().zip(kinds)) {
            Self::normalize(Dim::Volume, &mut r);
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

impl TopologyEdit for CellComplex {
    fn remove_shell(&mut self, volume: EntityId, ring: u16) -> Result<(), TopoError> {
        self.atomic(|s| {
            s.live_slot(volume, Dim::Volume)?;
            if ring == 0 || ring == Incidence::EMBEDDED {
                return Err(TopoError::InvalidParameter("only a non-outer shell can be removed"));
            }
            if !s.shells.contains_key(&(volume, ring)) {
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
        if self.open.is_some() {
            return Err(TopoError::OpAlreadyOpen);
        }
        if self.ops_run > 0 && op <= self.revision {
            return Err(TopoError::OpNotIncreasing { previous: self.revision.0, got: op.0 });
        }
        self.journal.clear();
        self.dirty_edges.clear();
        self.orient_dirty.clear();
        self.merge_gap = None;
        self.open = Some(OpState { op, prev_revision: self.revision, start: self.mark() });
        self.revision = op;
        Ok(())
    }

    fn end_op(&mut self) -> Result<OpSummary, TopoError> {
        let st = self.open.ok_or(TopoError::NoOpenOp)?;
        // 1. Re-apply the orientation rule to every touched face.
        let mut touched: BTreeSet<EntityId> = BTreeSet::new();
        for u in &self.journal {
            match u {
                Undo::Alloc(id) | Undo::Row { id, .. } | Undo::State { id, .. } | Undo::Carrier { id, .. } => {
                    touched.insert(*id);
                }
                _ => {}
            }
        }
        let mut faces: BTreeSet<EntityId> = touched.iter().copied().filter(|id| id.kind() == EntityKind::TopoFace).collect();
        faces.extend(core::mem::take(&mut self.orient_dirty));
        for f in faces {
            if let Err(e) = self.normalize_orientation(f) {
                self.abort_op();
                return Err(e);
            }
        }
        // 1b. Drop the kinds of emptied non-outer shells and close the ring gaps, so equal
        // shapes hash equal whatever shells they once had.
        if let Err(e) = self.compact_shells() {
            self.abort_op();
            return Err(e);
        }
        // 2. Cheap local checks on the touched cells and their parents.
        let mut check: BTreeSet<EntityId> = BTreeSet::new();
        for u in &self.journal {
            if let Undo::Alloc(id) | Undo::Row { id, .. } | Undo::State { id, .. } | Undo::Carrier { id, .. } = u {
                check.insert(*id);
            }
        }
        let seeds: Vec<EntityId> = check.iter().copied().collect();
        for id in seeds {
            if self.state(id) != Some(EntityState::Live) {
                continue;
            }
            let ups: Vec<EntityId> = self.up(id).iter().map(|e| e.cell).collect();
            if matches!(id.kind(), EntityKind::TopoEdge | EntityKind::TopoFace) {
                for p in &ups {
                    check.insert(*p);
                    if id.kind() == EntityKind::TopoEdge {
                        for q in self.up(*p) {
                            check.insert(q.cell);
                        }
                    }
                }
            }
        }
        let ids: Vec<EntityId> = check.into_iter().filter(|id| self.state(*id) == Some(EntityState::Live)).collect();
        let issues = validate::check_cells(self, &ids);
        if !issues.is_empty() {
            self.abort_op();
            return Err(TopoError::OpInvalid(issues));
        }
        // 3. Rebuild the radial rings of changed edges.
        let dirty: Vec<EntityId> = core::mem::take(&mut self.dirty_edges).into_iter().collect();
        for e in dirty {
            if let Some((Dim::Edge, slot)) = self.locate(e) {
                if self.tables[Dim::Edge.index()].state[slot] == EntityState::Live {
                    let ring = self.compute_radial(e);
                    self.radial[slot] = ring;
                } else {
                    self.radial[slot] = SmallVec::new();
                }
            }
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
        let summary = OpSummary { op: st.op, created, retired, lineage_start: st.start.lineage, lineage_len: self.lineage.len() - st.start.lineage };
        self.open = None;
        self.ops_run += 1;
        self.journal.clear();
        for t in &mut self.tables {
            t.down.maybe_compact();
        }
        Ok(summary)
    }

    fn abort_op(&mut self) {
        if let Some(st) = self.open.take() {
            self.rollback(st.start);
            self.revision = st.prev_revision;
            self.merge_gap = None;
            self.dirty_edges.clear();
            self.orient_dirty.clear();
            self.journal.clear();
        }
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
                if !super::lt(c.range.lo, c.range.hi) {
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
            Self::normalize(Dim::Face, &mut row);
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
            Self::normalize(Dim::Volume, &mut row);
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
            Self::normalize(Dim::Body, &mut row);
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
            let ring = ring.max(s.shells.range((volume, 0)..=(volume, u16::MAX)).map(|(k, _)| k.1 + 1).max().unwrap_or(0));
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
            Self::normalize(Dim::Volume, &mut row);
            s.set_row(volume, &row)?;
            s.set_shell_raw((volume, ring), Some(shell.kind));
            Ok(ring)
        })
    }

    fn add_face_use(&mut self, volume: EntityId, ring: u16, use_: FaceUseSpec) -> Result<(), TopoError> {
        self.atomic(|s| {
            s.live_slot(volume, Dim::Volume)?;
            s.live_slot(use_.face, Dim::Face)?;
            let kind = s.shells.get(&(volume, ring)).copied().ok_or(TopoError::InvalidParameter("no such shell"))?;
            if use_.side == Side::Both && kind != ShellKind::Internal {
                return Err(TopoError::BaffleInClosedShell);
            }
            s.check_occupancy(&[(use_.face, use_.side)])?;
            let mut row = s.row(volume).to_vec();
            row.push(Incidence::new(use_.face, use_.side, ring));
            Self::normalize(Dim::Volume, &mut row);
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
            Self::normalize(pd, &mut row);
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
                    if !(super::lt(c.range.lo, param) && super::lt(param, c.range.hi)) {
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
                if let Some(p) = s.pcurves.get(&old).cloned() {
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
            Self::normalize(Dim::Face, &mut new);
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
            Self::normalize(Dim::Face, &mut new);
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
            let gap = s.curve_gap(keep, drop, relative);
            if !s.gap_allowed(gap, tk, td) {
                return Err(TopoError::NotCoincident("curves are farther apart than the tolerances allow"));
            }
            s.grow_tolerance(keep, gap, td)?;
            // The dropped edge's pcurves refer to its own curve parameter: drop them.
            let uses: Vec<Incidence> = s.up(drop).iter().copied().filter(|u| Dim::of_real_kind(u.cell.kind()) == Some(Dim::Face)).collect();
            for u in uses {
                let k = CoedgeKey { face: u.cell, edge: drop, side: u.side };
                if s.pcurves.contains_key(&k) {
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
            let gap = s.face_gap(keep, drop);
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
                Self::normalize(Dim::Face, &mut row);
                s.set_row(keep, &row)?;
            }
            // Other coincidence records of the dropped face now belong to the survivor.
            let pairs: Vec<FacePair> = s.coincident_faces(drop).to_vec();
            for p in pairs {
                s.del_pair(p.a, p.b);
                if let Some(other) = p.other(drop)
                    && other != keep
                {
                    let (a, b) = if keep <= other { (keep, other) } else { (other, keep) };
                    s.put_pair(FacePair { a, b, status: p.status, op });
                }
            }
            s.replace_in_parents(drop, |e| smallvec![Incidence { cell: keep, side: if relative == Sense::Reversed { e.side.flip() } else { e.side }, ring: e.ring }])?;
            s.drop_face_pcurves(drop, &drow);
            s.retire(drop, EntityState::Merged(op))?;
            s.note(drop, LineageKind::Merged { into: keep, gap });
            s.put_pair(FacePair { a: keep, b: drop, status: Coincidence::Merged, op });
            s.normalize_orientation(keep)
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
            s.journal.push(Undo::Curve { id, old });
            let edges: Vec<EntityId> = s.tables[Dim::Edge.index()].live.iter().copied().filter(|e| matches!(s.carrier(*e), Carrier::Curve(c) if c.curve == id)).collect();
            s.dirty_edges.extend(edges);
            s.note(id, LineageKind::Modified(ModKind::Curve));
            Ok(())
        })
    }

    fn replace_surface(&mut self, id: EntityId, def: SurfaceDef) -> Result<(), TopoError> {
        self.atomic(|s| {
            let old = s.shapes.replace_surface(id, def)?;
            s.journal.push(Undo::Surface { id, old });
            // Faces carried by the surface and CSG cells with a half-space on it.
            let uses = |c: &Carrier| match c {
                Carrier::Surface(c) => c.surface == id,
                Carrier::Csg(cell) => cell.halfspaces.iter().any(|h| h.surface == id),
                _ => false,
            };
            let mut hit: Vec<EntityId> = Vec::new();
            for dim in [Dim::Face, Dim::Volume] {
                hit.extend(s.tables[dim.index()].live.iter().copied().filter(|c| uses(s.carrier(*c))));
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
            s.put_pair(FacePair { a, b, status, op });
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
