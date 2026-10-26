//! Stage 4 of the kernel: apply pieces to the topology.
//!
//! Vertices are found or created (a [`VertexIndex`] per apply group), existing edges are split
//! where a vertex lies on them, every piece becomes sub-edges between consecutive vertices
//! (an existing edge is reused when one already connects the same vertices along the same
//! geometry), and each face is arranged: its cuts are inserted one chain at a time with
//! `split_face`, the remaining cuts following their children.

use std::collections::{BTreeMap, BTreeSet};

use sm_core::{EntityId, Point3};
use sm_geom_api::{CurveDef, Interval, Uv};
use sm_topo_api::{Carrier, CoedgeKey, CoedgeSpec, CornerKey, CurveCarrier, Dim, FaceCut, InnerLoopAssign, PointClass, Sense, Side, TopologyEdit, TopologyViewExt};

use crate::cc;
use crate::error::BooleanError;
use crate::faceclass;
use crate::geo;
use crate::pieces::{CurveSrc, Piece};
use crate::registry::VertexIndex;

/// One cut to insert into a face.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Cut {
    pub edge: EntityId,
    /// Vertex the edge starts at, and the one it ends at.
    pub tail: EntityId,
    pub head: EntityId,
    /// A point strictly inside the edge, used to decide which child of a split owns it.
    pub mid: Point3,
}

/// What [`apply`] did.
#[derive(Clone, Debug, Default)]
pub(crate) struct Applied {
    /// Original face -> the live faces it became (only faces that were cut).
    pub fragments: BTreeMap<EntityId, Vec<EntityId>>,
    /// Every edge a piece produced or reused, with the original faces it was inserted into.
    pub cut_edges: Vec<(EntityId, Vec<EntityId>)>,
    /// Edges created for pieces (not reused).
    pub new_edges: Vec<EntityId>,
    /// Vertices created.
    pub new_vertices: Vec<EntityId>,
}

/// The owned definition of a piece's curve.
fn piece_def<E: TopologyEdit + ?Sized>(topo: &E, src: &CurveSrc) -> Option<CurveDef> {
    match src {
        CurveSrc::New(d) => Some(d.clone()),
        CurveSrc::Existing(h) => topo.shapes().curve_def(*h).cloned(),
    }
}

struct Applier<'a, E: TopologyEdit + ?Sized> {
    topo: &'a mut E,
    lin: f64,
    out: Applied,
}

/// Apply `pieces` (all of one group) to the topology. `generated_from` is recorded on every new
/// edge's faces by the caller; here only geometry and topology change.
pub(crate) fn apply<E: TopologyEdit + ?Sized>(topo: &mut E, pieces: &[&Piece], lin: f64, face_order: &[EntityId]) -> Result<Applied, BooleanError> {
    let mut ap = Applier { topo, lin, out: Applied::default() };
    ap.run(pieces, face_order)?;
    Ok(ap.out)
}

impl<E: TopologyEdit + ?Sized> Applier<'_, E> {
    fn run(&mut self, pieces: &[&Piece], face_order: &[EntityId]) -> Result<(), BooleanError> {
        if pieces.is_empty() {
            return Ok(());
        }
        // Faces and the cells around them.
        let mut faces: BTreeSet<EntityId> = BTreeSet::new();
        for p in pieces {
            faces.extend(p.faces.iter().copied());
            faces.extend(p.on_boundary.iter().copied());
        }
        let mut edges: BTreeSet<EntityId> = BTreeSet::new();
        let mut index = VertexIndex::new(self.lin);
        let mut seen_v: BTreeSet<EntityId> = BTreeSet::new();
        for &f in &faces {
            for &e in self.topo.face_edges(f) {
                if !edges.insert(e) {
                    continue;
                }
                if let Some((s, t)) = self.topo.edge_ends(e) {
                    for v in [s, t] {
                        if seen_v.insert(v)
                            && let Some(p) = geo::vertex_point(&*self.topo, v)
                        {
                            index.insert(v, p);
                        }
                    }
                }
            }
        }
        // Curve handles for new curves, one per key.
        let mut handles: BTreeMap<usize, EntityId> = BTreeMap::new();
        for p in pieces {
            if let CurveSrc::New(def) = &p.src
                && !handles.contains_key(&p.key)
            {
                let h = self.topo.shapes_mut()?.add_curve(def.clone())?;
                handles.insert(p.key, h);
            }
        }
        // Vertices at piece ends.
        let mut ends: Vec<(EntityId, EntityId)> = Vec::with_capacity(pieces.len());
        let mut defs: Vec<CurveDef> = Vec::with_capacity(pieces.len());
        for p in pieces {
            let def = piece_def(&*self.topo, &p.src).ok_or(BooleanError::Unsupported("a piece's curve is not in the store"))?;
            let c = def.as_curve();
            let (a, b) = (c.eval(p.range.lo), c.eval(p.range.hi));
            let va = self.vertex_at(&mut index, a)?;
            let vb = if p.closed { va } else { self.vertex_at(&mut index, b)? };
            ends.push((va, vb));
            defs.push(def);
        }
        // Split existing edges where a vertex lies on them.
        let mut edge_children: BTreeMap<EntityId, Vec<EntityId>> = BTreeMap::new();
        for &e in &edges {
            let kids = self.split_edge_at_vertices(e, &index)?;
            if kids.len() > 1 {
                edge_children.insert(e, kids);
            }
        }
        // Sub-edges of every piece.
        let mut cuts: BTreeMap<EntityId, Vec<Cut>> = BTreeMap::new();
        for (pi, p) in pieces.iter().enumerate() {
            let handle = handles.get(&p.key).copied();
            let (va, vb) = ends[pi];
            let _ = handle;
            let c = defs[pi].as_curve();
            // Interior vertices on the piece.
            let mut mids: Vec<(f64, EntityId)> = Vec::new();
            for &(v, q) in index.all() {
                if v == va || v == vb {
                    continue;
                }
                if let Some((d, t)) = cc::dist_to_range(c, p.range, q)
                    && d <= self.lin
                    && t > p.range.lo
                    && t < p.range.hi
                {
                    mids.push((t, v));
                }
            }
            mids.sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
            let mut chain: Vec<(f64, EntityId)> = vec![(p.range.lo, va)];
            chain.extend(mids);
            chain.push((p.range.hi, vb));
            for w in chain.windows(2) {
                let ((ta, u), (tb, v)) = (w[0], w[1]);
                if u == v && !(p.closed && chain.len() == 2) {
                    continue;
                }
                let midp = c.eval(0.5 * (ta + tb));
                // Reuse an edge that already runs between the same vertices along the same curve.
                let target_faces: Vec<EntityId> = p.faces.iter().chain(p.on_boundary.iter()).copied().collect();
                let existing = self.find_edge(u, v, midp, &edges, &edge_children);
                let edge = match existing {
                    Some(e) => e,
                    None => {
                        let h = match &p.src {
                            CurveSrc::New(_) => handles[&p.key],
                            CurveSrc::Existing(h) => *h,
                        };
                        let e = self.topo.make_edge(u, v, Some(CurveCarrier { curve: h, range: Interval::new(ta, tb), sense: Sense::Forward }))?;
                        self.out.new_edges.push(e);
                        e
                    }
                };
                let (tail, head) = self.topo.edge_ends(edge).unwrap_or((u, v));
                let mut inserted_into = Vec::new();
                for &f in &p.faces {
                    // Skip a face whose boundary already holds the edge.
                    if self.topo.face_edges(f).binary_search(&edge).is_ok() {
                        continue;
                    }
                    cuts.entry(f).or_default().push(Cut { edge, tail, head, mid: midp });
                    inserted_into.push(f);
                }
                let _ = target_faces;
                self.out.cut_edges.push((edge, inserted_into));
            }
        }
        // Arrange each face.
        let order: Vec<EntityId> = if face_order.is_empty() { cuts.keys().copied().collect() } else { face_order.iter().copied().filter(|f| cuts.contains_key(f)).collect() };
        for f in order {
            let mut list = cuts.remove(&f).unwrap_or_default();
            list.sort_by_key(|c| c.edge);
            list.dedup_by_key(|c| c.edge);
            let frags = self.arrange(f, list)?;
            self.out.fragments.insert(f, frags);
        }
        // Faces that got cuts but were not in `face_order` (should not happen) are arranged too.
        let rest: Vec<EntityId> = cuts.keys().copied().collect();
        for f in rest {
            let mut list = cuts.remove(&f).unwrap_or_default();
            list.sort_by_key(|c| c.edge);
            list.dedup_by_key(|c| c.edge);
            let frags = self.arrange(f, list)?;
            self.out.fragments.insert(f, frags);
        }
        Ok(())
    }

    fn vertex_at(&mut self, index: &mut VertexIndex, p: Point3) -> Result<EntityId, BooleanError> {
        if let Some(v) = index.find(p) {
            return Ok(v);
        }
        let v = self.topo.make_vertex(p)?;
        index.insert(v, p);
        self.out.new_vertices.push(v);
        Ok(v)
    }

    /// Split `edge` at every indexed vertex lying strictly inside it; returns the live pieces in
    /// edge order (`[edge]` if nothing was split).
    fn split_edge_at_vertices(&mut self, edge: EntityId, index: &VertexIndex) -> Result<Vec<EntityId>, BooleanError> {
        let Some((vs, ve)) = self.topo.edge_ends(edge) else { return Ok(vec![edge]) };
        let Carrier::Curve(cc0) = self.topo.carrier(edge).clone() else { return Ok(vec![edge]) };
        let Some(curve) = self.topo.shapes().curve(cc0.curve) else { return Ok(vec![edge]) };
        let mut hits: Vec<(f64, EntityId)> = Vec::new();
        for &(v, q) in index.all() {
            if v == vs || v == ve {
                // A closed edge's own vertex is an end; nothing to split there.
                continue;
            }
            if let Some((d, t)) = cc::dist_to_range(curve, cc0.range, q)
                && d <= self.lin
                && t > cc0.range.lo
                && t < cc0.range.hi
            {
                hits.push((t, v));
            }
        }
        if hits.is_empty() {
            return Ok(vec![edge]);
        }
        // Edge direction order: increasing parameter for a forward carrier, decreasing otherwise.
        hits.sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
        if cc0.sense == Sense::Reversed {
            hits.reverse();
        }
        hits.dedup_by(|a, b| a.1 == b.1);
        let mut kids = Vec::with_capacity(hits.len() + 1);
        let mut cur = edge;
        for (t, v) in hits {
            let [first, second] = self.topo.split_edge(cur, v, t)?;
            kids.push(first);
            cur = second;
        }
        kids.push(cur);
        Ok(kids)
    }

    /// A live edge between `u` and `v` that runs through `mid` (and, for a closed edge, covers the
    /// range), among the cells of this apply.
    fn find_edge(&self, u: EntityId, v: EntityId, mid: Point3, edges: &BTreeSet<EntityId>, children: &BTreeMap<EntityId, Vec<EntityId>>) -> Option<EntityId> {
        let mut cand: Vec<EntityId> = Vec::new();
        for inc in self.topo.coboundary(u) {
            if Dim::of_real_kind(inc.cell.kind()) == Some(Dim::Edge) {
                cand.push(inc.cell);
            }
        }
        cand.sort();
        cand.dedup();
        for e in cand {
            if !self.topo.is_alive(e) {
                continue;
            }
            // Only edges of this apply: originals (or their pieces) and edges created here.
            let known = edges.contains(&e) || children.values().any(|k| k.contains(&e)) || self.out.new_edges.contains(&e);
            if !known {
                continue;
            }
            let Some((s, t)) = self.topo.edge_ends(e) else { continue };
            if !((s == u && t == v) || (s == v && t == u)) {
                continue;
            }
            let Carrier::Curve(cc0) = self.topo.carrier(e) else { continue };
            let Some(ec) = self.topo.shapes().curve(cc0.curve) else { continue };
            if let Some((d, _)) = cc::dist_to_range(ec, cc0.range, mid)
                && d <= self.lin
            {
                return Some(e);
            }
        }
        None
    }
}

// ---------------------------------------------------------------------------------------------
// Face arrangement
// ---------------------------------------------------------------------------------------------

/// A cut chain chosen for insertion.
struct Step {
    kind: StepKind,
    /// Coedges in traversal order.
    chain: Vec<CoedgeSpec>,
    /// Indices into the cut list that the chain consumes.
    used: Vec<usize>,
    /// Where the chain attaches: set when an end vertex occurs more than once in the face loop.
    ends: Option<(End, End)>,
}

/// One end of a chain on the face loop.
#[derive(Clone, Copy, Debug)]
enum End {
    /// A vertex that occurs once in the loop.
    One,
    /// A vertex that occurs several times (pole, seam): the corner is chosen from the direction
    /// the chain leaves in, given by a point of the chain close to the vertex.
    Multi(EntityId, Point3),
}

#[derive(Clone, Copy)]
enum StepKind {
    /// Open chain between two distinct vertices of the outer loop.
    Chain,
    /// Closed chain: interior (no vertex on the face) or through a vertex occurring twice.
    Closed,
    /// A single edge joining two different loops: bridge them.
    Bridge(EntityId),
}

impl<E: TopologyEdit + ?Sized> Applier<'_, E> {
    /// Insert `cuts` into `face`; returns the live fragments (ascending).
    pub(crate) fn arrange(&mut self, face: EntityId, cuts: Vec<Cut>) -> Result<Vec<EntityId>, BooleanError> {
        let mut work: Vec<(EntityId, Vec<Cut>)> = vec![(face, cuts)];
        let mut done: Vec<EntityId> = Vec::new();
        while let Some((f, cuts)) = work.pop() {
            if cuts.is_empty() {
                done.push(f);
                continue;
            }
            let step = self.next_step(f, &cuts)?;
            if let StepKind::Bridge(e) = step.kind {
                self.topo.bridge_loops(f, e)?;
                let rest: Vec<Cut> = cuts.iter().enumerate().filter(|(i, _)| !step.used.contains(i)).map(|(_, c)| *c).collect();
                work.push((f, rest));
                continue;
            }
            let has_inner = self.topo.loops(f).any(|l| l.ring > 0);
            let inner = if has_inner { InnerLoopAssign::ByPointTest } else { InnerLoopAssign::None };
            let corners = match step.ends {
                Some((a, b)) => {
                    let ca = self.resolve_corner(f, a)?;
                    let cb = self.resolve_corner(f, b)?;
                    Some((ca, cb))
                }
                None => None,
            };
            let cut = match (step.kind, corners) {
                (StepKind::Chain, Some((start, end))) if step.chain.len() == 1 => FaceCut::ChordAt { edge: step.chain[0].edge, start, end },
                (StepKind::Chain, Some((start, end))) => FaceCut::ChainAt { edges: &step.chain, start, end },
                (StepKind::Chain, None) if step.chain.len() == 1 => FaceCut::Chord { edge: step.chain[0].edge },
                (StepKind::Chain, None) => FaceCut::Chain { edges: &step.chain },
                (StepKind::Closed, _) => FaceCut::ClosedLoop { edges: &step.chain },
                (StepKind::Bridge(_), _) => unreachable!("handled above"),
            };
            let [c0, c1] = self.topo.split_face(f, cut, inner)?;
            let rest: Vec<Cut> = cuts.iter().enumerate().filter(|(i, _)| !step.used.contains(i)).map(|(_, c)| *c).collect();
            let (mut r0, mut r1) = (Vec::new(), Vec::new());
            let cache = faceclass::FaceCache::new();
            for c in rest {
                let (class, _) = faceclass::classify(&*self.topo, &cache, c0, c.mid, self.lin)?;
                if class == PointClass::In {
                    r0.push(c);
                } else {
                    let (class1, _) = faceclass::classify(&*self.topo, &cache, c1, c.mid, self.lin)?;
                    if class1 != PointClass::In {
                        return Err(BooleanError::Unsupported("a cut lies on neither side of the cut just inserted (crossing cuts inside a face)"));
                    }
                    r1.push(c);
                }
            }
            work.push((c1, r1));
            work.push((c0, r0));
        }
        done.sort();
        Ok(done)
    }

    /// Loop vertex positions of a face: vertex -> list of `(ring, index)`.
    fn loop_positions(&self, f: EntityId) -> BTreeMap<EntityId, Vec<(u16, usize)>> {
        let mut m: BTreeMap<EntityId, Vec<(u16, usize)>> = BTreeMap::new();
        for lp in self.topo.loops(f) {
            for (i, key) in lp.keys().enumerate() {
                let Some((s, t)) = self.topo.edge_ends(key.edge) else { continue };
                let tail = if key.side == Side::Right { t } else { s };
                m.entry(tail).or_default().push((lp.ring, i));
            }
        }
        m
    }

    /// Depth-first walk from `cur` along unused cuts to a vertex on a face loop. On success `path`
    /// holds `(cut index, vertex it is entered from)` and the loop vertex reached is returned.
    fn walk(cuts: &[Cut], incident: &BTreeMap<EntityId, Vec<usize>>, pos: &BTreeMap<EntityId, Vec<(u16, usize)>>, cur: EntityId, used: &mut Vec<usize>, path: &mut Vec<(usize, EntityId)>, depth: usize) -> Option<EntityId> {
        if pos.contains_key(&cur) {
            return Some(cur);
        }
        if depth > cuts.len() {
            return None;
        }
        let mut nexts: Vec<usize> = incident.get(&cur).into_iter().flatten().copied().filter(|j| !used.contains(j)).collect();
        nexts.sort_by_key(|&j| cuts[j].edge);
        for j in nexts {
            used.push(j);
            path.push((j, cur));
            let nxt = if cuts[j].tail == cur { cuts[j].head } else { cuts[j].tail };
            if let Some(end) = Self::walk(cuts, incident, pos, nxt, used, path, depth + 1) {
                return Some(end);
            }
            used.pop();
            path.pop();
        }
        None
    }

    fn chain_of(cuts: &[Cut], path: &[(usize, EntityId)]) -> Vec<CoedgeSpec> {
        path.iter().map(|&(j, from)| CoedgeSpec { edge: cuts[j].edge, sense: if cuts[j].tail == from { Sense::Forward } else { Sense::Reversed } }).collect()
    }

    fn next_step(&self, f: EntityId, cuts: &[Cut]) -> Result<Step, BooleanError> {
        let pos = self.loop_positions(f);
        let mut incident: BTreeMap<EntityId, Vec<usize>> = BTreeMap::new();
        for (i, c) in cuts.iter().enumerate() {
            incident.entry(c.tail).or_default().push(i);
            incident.entry(c.head).or_default().push(i);
        }
        let ring_of = |v: EntityId| -> Option<u16> { pos.get(&v).and_then(|ps| ps.iter().map(|p| p.0).min()) };
        let mut order: Vec<usize> = (0..cuts.len()).collect();
        order.sort_by_key(|&i| cuts[i].edge);
        // 1. Chains starting at a loop vertex.
        for &i in &order {
            for start in [cuts[i].tail, cuts[i].head] {
                if !pos.contains_key(&start) {
                    continue;
                }
                let mut used = vec![i];
                let mut path = vec![(i, start)];
                let cur = if cuts[i].tail == start { cuts[i].head } else { cuts[i].tail };
                let Some(end) = Self::walk(cuts, &incident, &pos, cur, &mut used, &mut path, 0) else {
                    // A dead end from this start; try the other end / next cut.
                    continue;
                };
                let chain = Self::chain_of(cuts, &path);
                if start == end {
                    let n = pos.get(&start).map_or(0, Vec::len);
                    if n != 2 {
                        return Err(BooleanError::Unsupported("a closed cut touches the face boundary at a vertex that is not a periodic seam vertex"));
                    }
                    let chain = self.orient_seam_loop(f, chain, &pos[&start])?;
                    return Ok(Step { kind: StepKind::Closed, chain, used, ends: None });
                }
                let (rs, re) = (ring_of(start), ring_of(end));
                if rs != re {
                    if used.len() == 1 {
                        return Ok(Step { kind: StepKind::Bridge(cuts[i].edge), chain, used, ends: None });
                    }
                    return Err(BooleanError::Unsupported("a multi-edge cut joins two different loops of a face (bridge_loops takes a single edge)"));
                }
                if rs != Some(0) {
                    return Err(BooleanError::Unsupported("a cut runs from an inner loop back to the same inner loop"));
                }
                let end_of = |v: EntityId, first: bool| -> End {
                    if pos.get(&v).map_or(0, Vec::len) <= 1 {
                        return End::One;
                    }
                    // A point of the chain near this end.
                    let (spec, near_tail) = if first { (chain[0], chain[0].sense == Sense::Forward) } else { (chain[chain.len() - 1], chain[chain.len() - 1].sense != Sense::Forward) };
                    let eps = 0.02;
                    let frac = if near_tail { eps } else { 1.0 - eps };
                    match geo::edge_sample(&*self.topo, spec.edge, frac) {
                        Some(p) => End::Multi(v, p),
                        None => End::One,
                    }
                };
                let ends = (end_of(start, true), end_of(end, false));
                let ends = if matches!(ends.0, End::One) && matches!(ends.1, End::One) { None } else { Some(ends) };
                return Ok(Step { kind: StepKind::Chain, chain, used, ends });
            }
        }
        // 2. A cycle strictly inside the face.
        let i0 = order[0];
        let start = cuts[i0].tail;
        let mut used = vec![i0];
        let mut path = vec![(i0, start)];
        let mut cur = cuts[i0].head;
        let mut guard = 0;
        while cur != start {
            guard += 1;
            if guard > cuts.len() + 2 {
                return Err(BooleanError::Unsupported("interior cut does not close"));
            }
            let mut nexts: Vec<usize> = incident.get(&cur).into_iter().flatten().copied().filter(|j| !used.contains(j)).collect();
            nexts.sort_by_key(|&j| cuts[j].edge);
            let Some(&j) = nexts.first() else { return Err(BooleanError::Unsupported("an interior cut is a dangling chain")) };
            used.push(j);
            path.push((j, cur));
            cur = if cuts[j].tail == cur { cuts[j].head } else { cuts[j].tail };
        }
        let mut chain = Self::chain_of(cuts, &path);
        // The disk child runs counter-clockwise seen from the face's right side.
        if self.loop_orientation(f, &chain)? < 0.0 {
            chain.reverse();
            for c in &mut chain {
                c.sense = c.sense.flip();
            }
        }
        Ok(Step { kind: StepKind::Closed, chain, used, ends: None })
    }

    /// The corner of the outer loop of `f` where a chain that leaves vertex `v` towards `toward`
    /// attaches. A pole vertex has degenerate coedges spanning a range of longitudes: the one that
    /// holds the chain's longitude is split there. Other repeated vertices (a periodic seam) pick
    /// the corner whose interior wedge contains the chain's direction.
    fn resolve_corner(&mut self, f: EntityId, end: End) -> Result<CornerKey, BooleanError> {
        let End::Multi(v, toward) = end else { return Err(BooleanError::Unsupported("internal: a single occurrence needs no corner")) };
        let Some((surf, sc)) = geo::face_surface(&*self.topo, f) else { return Err(BooleanError::Unsupported("a face has no surface")) };
        let dom = surf.domain();
        let entries: Vec<(EntityId, Side)> = self.topo.loops(f).find(|l| l.ring == 0).map(|l| l.entries.iter().map(|e| (e.cell, e.side)).collect()).unwrap_or_default();
        let n = entries.len();
        let tail = |t: &E, i: usize| -> Option<EntityId> {
            let (e, side) = entries[i];
            t.edge_ends(e).map(|(s, h)| if side == Side::Right { h } else { s })
        };
        let key = |i: usize| CoedgeKey { face: f, edge: entries[i].0, side: entries[i].1 };
        let positions: Vec<usize> = (0..n).filter(|&i| tail(&*self.topo, i) == Some(v)).collect();
        let prev = |i: usize| (i + n - 1) % n;
        // Pole: a degenerate coedge starts here.
        let deg: Vec<usize> = positions.iter().copied().filter(|&i| geo::is_degenerate(&*self.topo, entries[i].0)).collect();
        if !deg.is_empty() {
            let u0 = surf.project(toward, None)?.uv.u;
            let period = dom.u.period.unwrap_or(0.0);
            for i in deg {
                let (edge, side) = entries[i];
                let Some(sm_geom_api::PCurve::Line2(l)) = self.topo.pcurve(key(i)).cloned() else { return Err(BooleanError::Unsupported("a pole edge has no Line2 pcurve")) };
                let (ua, ub) = if side == Side::Left { (l.origin.u + l.range.lo * l.dir.u, l.origin.u + l.range.hi * l.dir.u) } else { (l.origin.u + l.range.hi * l.dir.u, l.origin.u + l.range.lo * l.dir.u) };
                let (lo, hi) = (ua.min(ub), ua.max(ub));
                let mid = 0.5 * (lo + hi);
                let u = if period > 0.0 { u0 + period * ((mid - u0) / period).round() } else { u0 };
                let tol_u = 1e-8;
                if u > lo + tol_u && u < hi - tol_u {
                    if l.dir.u == 0.0 {
                        return Err(BooleanError::Unsupported("a pole pcurve does not run along u"));
                    }
                    let t = (u - l.origin.u) / l.dir.u;
                    let [e1, e2] = self.topo.split_degenerate_edge(edge, t)?;
                    let incoming = if side == Side::Left { CoedgeKey { face: f, edge: e1, side } } else { CoedgeKey { face: f, edge: e2, side } };
                    return Ok(CornerKey { face: f, incoming });
                }
                if (u - ua).abs() <= tol_u {
                    return Ok(CornerKey { face: f, incoming: key(prev(i)) });
                }
                if (u - ub).abs() <= tol_u {
                    return Ok(CornerKey { face: f, incoming: key(i) });
                }
            }
            return Err(BooleanError::Unsupported("the chain meets a pole at a longitude outside the pole edge"));
        }
        // Repeated non-pole vertex: wedge test in the parameter plane.
        let pv = geo::vertex_point(&*self.topo, v).ok_or(BooleanError::Unsupported("vertex without point"))?;
        let c = surf.project(pv, None)?.uv;
        let near = |p: Point3| -> Result<(f64, f64), BooleanError> {
            let uv = surf.project(p, Some(c))?.uv;
            Ok((dom.u.nearest_to(uv.u, c.u) - c.u, dom.v.nearest_to(uv.v, c.v) - c.v))
        };
        let angle = |d: (f64, f64)| d.1.atan2(d.0);
        let a_chain = angle(near(toward)?);
        let eps = 0.02;
        let mut hits: Vec<usize> = Vec::new();
        for &i in &positions {
            let (e_out, s_out) = entries[i];
            let (e_in, s_in) = entries[prev(i)];
            // A point just after the corner on the outgoing coedge, just before it on the incoming.
            let f_out = if s_out == Side::Right { 1.0 - eps } else { eps };
            let f_in = if s_in == Side::Right { eps } else { 1.0 - eps };
            let (Some(po), Some(pi)) = (geo::edge_sample(&*self.topo, e_out, f_out), geo::edge_sample(&*self.topo, e_in, f_in)) else { continue };
            let (a_out, a_in) = (angle(near(po)?), angle(near(pi)?));
            let ccw = |from: f64, to: f64, x: f64| {
                let tau = core::f64::consts::TAU;
                let d = |a: f64| a.rem_euclid(tau);
                d(x - from) <= d(to - from)
            };
            let inside = if sc.sense == Sense::Forward { ccw(a_out, a_in, a_chain) } else { ccw(a_in, a_out, a_chain) };
            if inside {
                hits.push(i);
            }
        }
        match hits[..] {
            [i] => Ok(CornerKey { face: f, incoming: key(prev(i)) }),
            _ => Err(BooleanError::Unsupported("cannot tell which corner of a repeated vertex the cut attaches to")),
        }
    }

    /// Sample a chain in the parameter space of `f` (continuous in the periodic directions) with
    /// the matching 3D points.
    fn chain_uv(&self, f: EntityId, chain: &[CoedgeSpec], per_edge: usize) -> Result<Vec<(Uv, Point3)>, BooleanError> {
        let Some((surf, _)) = geo::face_surface(&*self.topo, f) else { return Err(BooleanError::Unsupported("a face has no surface")) };
        let dom = surf.domain();
        let mut pts: Vec<(Uv, Point3)> = Vec::new();
        let mut prev: Option<Uv> = None;
        for spec in chain {
            let Carrier::Curve(cc0) = self.topo.carrier(spec.edge) else { return Err(BooleanError::Unsupported("an edge has no curve")) };
            let Some(curve) = self.topo.shapes().curve(cc0.curve) else { return Err(BooleanError::Unsupported("curve not in store")) };
            for k in 0..per_edge {
                let frac = k as f64 / per_edge as f64;
                let t = geo::edge_param(&cc0, if spec.sense == Sense::Forward { frac } else { 1.0 - frac });
                let p = curve.eval(t);
                let pr = surf.project(p, prev)?;
                let uv = Uv::new(dom.u.nearest_to(pr.uv.u, prev.map_or(pr.uv.u, |q| q.u)), dom.v.nearest_to(pr.uv.v, prev.map_or(pr.uv.v, |q| q.v)));
                pts.push((uv, p));
                prev = Some(uv);
            }
        }
        Ok(pts)
    }

    /// Orient a closed chain through a periodic seam vertex so that the first child
    /// (`outer[a..b]` plus the chain) lies on the interior side of its own boundary.
    fn orient_seam_loop(&self, f: EntityId, chain: Vec<CoedgeSpec>, at: &[(u16, usize)]) -> Result<Vec<CoedgeSpec>, BooleanError> {
        let Some((surf, sc)) = geo::face_surface(&*self.topo, f) else { return Err(BooleanError::Unsupported("a face has no surface")) };
        let dom = surf.domain();
        let (a, b) = (at[0].1.min(at[1].1), at[0].1.max(at[1].1));
        let outer: Vec<CoedgeSpec> = self
            .topo
            .loops(f)
            .find(|l| l.ring == 0)
            .map(|l| l.entries.iter().map(|e| CoedgeSpec { edge: e.cell, sense: if e.side == Side::Right { Sense::Reversed } else { Sense::Forward } }).collect())
            .unwrap_or_default();
        if b <= a || b > outer.len() {
            return Ok(chain);
        }
        // A point of the first child's boundary segment (middle coedge, skipping pole edges).
        let seg: Vec<&CoedgeSpec> = outer[a..b].iter().filter(|c| !geo::is_degenerate(&*self.topo, c.edge)).collect();
        let Some(mid) = seg.get(seg.len() / 2) else { return Ok(chain) };
        let Some(q3) = geo::edge_sample(&*self.topo, mid.edge, 0.5) else { return Ok(chain) };
        let pts = self.chain_uv(f, &chain, 24)?;
        if pts.len() < 2 {
            return Ok(chain);
        }
        let (i, _) = pts.iter().enumerate().map(|(i, p)| (i, geo::dist(p.1, q3))).fold((0, f64::INFINITY), |m, x| if x.1 < m.1 { x } else { m });
        let j = (i + 1) % pts.len();
        let (c, t) = (pts[i].0, Uv::new(pts[j].0.u - pts[i].0.u, pts[j].0.v - pts[i].0.v));
        let qpr = surf.project(q3, Some(c))?;
        let (qu, qv) = (dom.u.nearest_to(qpr.uv.u, c.u), dom.v.nearest_to(qpr.uv.v, c.v));
        let cross = t.u * (qv - c.v) - t.v * (qu - c.u);
        if cross * sc.sense.sign() > 0.0 {
            Ok(chain)
        } else {
            let mut r: Vec<CoedgeSpec> = chain.into_iter().rev().collect();
            for c in &mut r {
                c.sense = c.sense.flip();
            }
            Ok(r)
        }
    }

    /// Signed area of a closed chain in the parameter space of `f`, times the carrier sense
    /// (positive: counter-clockwise seen from the face's right side).
    fn loop_orientation(&self, f: EntityId, chain: &[CoedgeSpec]) -> Result<f64, BooleanError> {
        let Some((surf, sc)) = geo::face_surface(&*self.topo, f) else { return Err(BooleanError::Unsupported("a face has no surface")) };
        let dom = surf.domain();
        let mut pts: Vec<Uv> = Vec::new();
        let mut prev: Option<Uv> = None;
        for spec in chain {
            let Carrier::Curve(cc0) = self.topo.carrier(spec.edge) else { return Err(BooleanError::Unsupported("an edge has no curve")) };
            let Some(curve) = self.topo.shapes().curve(cc0.curve) else { return Err(BooleanError::Unsupported("curve not in store")) };
            const N: usize = 16;
            for k in 0..N {
                let frac = k as f64 / N as f64;
                let along = spec.sense == Sense::Forward;
                let t = geo::edge_param(&cc0, if along { frac } else { 1.0 - frac });
                let pr = surf.project(curve.eval(t), prev)?;
                let uv = Uv::new(dom.u.nearest_to(pr.uv.u, prev.map_or(pr.uv.u, |p| p.u)), dom.v.nearest_to(pr.uv.v, prev.map_or(pr.uv.v, |p| p.v)));
                pts.push(uv);
                prev = Some(uv);
            }
        }
        let n = pts.len();
        let mut a = 0.0;
        for i in 0..n {
            let (p, q) = (pts[i], pts[(i + 1) % n]);
            a += p.u * q.v - q.u * p.v;
        }
        Ok(0.5 * a * sc.sense.sign())
    }
}

