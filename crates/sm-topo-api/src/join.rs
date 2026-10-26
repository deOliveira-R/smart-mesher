//! Planning of [`TopologyEdit::join_edges`](crate::TopologyEdit::join_edges) and
//! [`TopologyEdit::join_faces`](crate::TopologyEdit::join_faces).
//!
//! Both operators are the inverse of a split. Every decision (which cells survive, the new
//! rows, the pcurve moves) is computed here from a read-only [`TopologyView`], so the reference
//! complex and the production store apply one plan with their own raw mutators and agree on
//! ids, rows and lineage. A planner changes nothing; an `Err` from it is the operator's `Err`.

use crate::error::TopoError;
use crate::types::{Carrier, CellFlags, CoedgeKey, CurveCarrier, Dim, EntityState, Incidence, Sense, Side};
use crate::view::TopologyView;
use crate::complex::{canonical_rotate, normalize_row};
use sm_core::EntityId;
use sm_geom_api::{Interval, PCurve};
use std::collections::{BTreeMap, BTreeSet};

type Row = Vec<Incidence>;

fn live(view: &dyn TopologyView, id: EntityId, dim: Dim) -> Result<(), TopoError> {
    if Dim::of_real_kind(id.kind()) != Some(dim) {
        return Err(TopoError::WrongDim { id, expected: dim });
    }
    match view.state(id) {
        None => Err(TopoError::NoSuchCell(id)),
        Some(EntityState::Live) => Ok(()),
        Some(_) => Err(TopoError::Dead(id)),
    }
}

fn ends(view: &dyn TopologyView, edge: EntityId) -> Result<(EntityId, EntityId), TopoError> {
    match view.boundary(edge) {
        [only] if only.side == Side::Both => Ok((only.cell, only.cell)),
        [a, b] if a.side == Side::Left && b.side == Side::Right => Ok((a.cell, b.cell)),
        _ => Err(TopoError::WrongDim { id: edge, expected: Dim::Edge }),
    }
}

/// `(tail, head)` of a coedge in traversal order.
fn tail_head(view: &dyn TopologyView, e: &Incidence) -> Result<(EntityId, EntityId), TopoError> {
    let (s, t) = ends(view, e.cell)?;
    Ok(if e.side == Side::Right { (t, s) } else { (s, t) })
}

fn rings(row: &[Incidence]) -> Result<Vec<Row>, TopoError> {
    let mut out: Vec<Row> = Vec::new();
    for g in row.chunk_by(|a, b| a.ring == b.ring) {
        if g[0].ring == Incidence::EMBEDDED {
            return Err(TopoError::Unsupported("join with embedded members"));
        }
        out.push(g.to_vec());
    }
    Ok(out)
}

fn flatten(rings: &[Row]) -> Row {
    let mut row: Row = rings.iter().enumerate().flat_map(|(i, r)| r.iter().map(move |e| Incidence { ring: i as u16, ..*e })).collect();
    normalize_row(Dim::Face, &mut row);
    row
}

/// What [`TopologyEdit::join_edges`](crate::TopologyEdit::join_edges) does, decided.
#[derive(Clone, Debug, PartialEq)]
pub struct JoinEdgesPlan {
    /// The surviving edge (the lower id of the two); it takes the new carrier and row.
    pub survivor: EntityId,
    /// The edge absorbed into the survivor (`Merged` lineage).
    pub absorbed: EntityId,
    /// The degree-2 vertex that dissolves (`Deleted` lineage).
    pub vertex: EntityId,
    /// The survivor's carrier after the join.
    pub carrier: Carrier,
    /// The survivor's new boundary row (normalized).
    pub row: Vec<Incidence>,
    /// The survivor's new tolerance override, if the absorbed edge's was larger.
    pub tolerance: Option<f64>,
    /// New rows of the parent faces, ascending by face.
    pub face_rows: Vec<(EntityId, Vec<Incidence>)>,
    /// Pcurve writes to apply (a `None` removes the pcurve).
    pub pcurves: Vec<(CoedgeKey, Option<PCurve>)>,
}

fn adjacent(view: &dyn TopologyView, a: f64, b: f64) -> bool {
    (a - b).abs() <= view.tolerance().parametric
}

fn merge_pcurves(a: Option<PCurve>, b: Option<PCurve>, tol: f64) -> Result<Option<PCurve>, TopoError> {
    match (a, b) {
        (None, None) => Ok(None),
        (Some(x), Some(y)) if x == y => Ok(Some(x)),
        (Some(PCurve::Line2(x)), Some(PCurve::Line2(y))) if x.origin == y.origin && x.dir == y.dir => {
            let (lo, hi) = if (x.range.hi - y.range.lo).abs() <= tol {
                (x.range.lo, y.range.hi)
            } else if (y.range.hi - x.range.lo).abs() <= tol {
                (y.range.lo, x.range.hi)
            } else {
                return Err(TopoError::Unsupported("join_edges: pcurve ranges are not adjacent"));
            };
            Ok(Some(PCurve::Line2(sm_geom_api::Line2 { range: Interval::new(lo, hi), ..x })))
        }
        _ => Err(TopoError::Unsupported("join_edges: pcurves cannot be merged")),
    }
}

/// The result of joining at one candidate vertex.
struct EdgeJoin {
    m: EntityId,
    start: EntityId,
    end: EntityId,
    /// The absorbed edge runs against the survivor's direction.
    rev_o: bool,
    m_is_s_end: bool,
    carrier: Carrier,
}

fn edge_join_at(view: &dyn TopologyView, s: EntityId, o: EntityId, m: EntityId) -> Result<EdgeJoin, TopoError> {
    let (s0, s1) = ends(view, s)?;
    let (o0, o1) = ends(view, o)?;
    let m_is_s_end = s1 == m;
    let far_o = if o0 == m { o1 } else { o0 };
    let (start, end) = if m_is_s_end { (s0, far_o) } else { (far_o, s1) };
    let rev_o = if m_is_s_end { o1 == m } else { o0 == m };
    let carrier = match (view.carrier(s), view.carrier(o)) {
        (Carrier::None, Carrier::None) => Carrier::None,
        (Carrier::Curve(cs), Carrier::Curve(co)) => {
            if cs.curve != co.curve {
                return Err(TopoError::NotCoincident("edges lie on different curves"));
            }
            let inc_s = cs.sense == Sense::Forward;
            let inc_o = (co.sense == Sense::Forward) != rev_o;
            if inc_s != inc_o {
                return Err(TopoError::NotCoincident("edges run against each other along the curve"));
            }
            let (lo, hi) = match (m_is_s_end, inc_s) {
                (true, true) => (adjacent(view, cs.range.hi, co.range.lo), (cs.range.lo, co.range.hi)),
                (true, false) => (adjacent(view, cs.range.lo, co.range.hi), (co.range.lo, cs.range.hi)),
                (false, true) => (adjacent(view, co.range.hi, cs.range.lo), (co.range.lo, cs.range.hi)),
                (false, false) => (adjacent(view, co.range.lo, cs.range.hi), (cs.range.lo, co.range.hi)),
            };
            if !lo {
                return Err(TopoError::NotCoincident("curve ranges are not adjacent"));
            }
            Carrier::Curve(CurveCarrier { curve: cs.curve, range: Interval::new(hi.0, hi.1), sense: cs.sense })
        }
        _ => return Err(TopoError::NotCoincident("edge carriers differ (curve and no curve)")),
    };
    Ok(EdgeJoin { m, start, end, rev_o, m_is_s_end, carrier })
}

/// Plan `join_edges(first, second)`: see [`TopologyEdit::join_edges`](crate::TopologyEdit::join_edges).
///
/// # Errors
/// As documented on the operator; the view is never changed.
pub fn plan_join_edges(view: &dyn TopologyView, first: EntityId, second: EntityId) -> Result<JoinEdgesPlan, TopoError> {
    live(view, first, Dim::Edge)?;
    live(view, second, Dim::Edge)?;
    if first == second {
        return Err(TopoError::InvalidParameter("join_edges needs two different edges"));
    }
    let (s, o) = if first < second { (first, second) } else { (second, first) };
    for e in [s, o] {
        if view.flags(e).contains(CellFlags::DEGENERATE) {
            return Err(TopoError::Unsupported("join_edges of a degenerate edge"));
        }
    }
    if view.flags(s) != view.flags(o) {
        return Err(TopoError::NotCoincident("edges differ in flags"));
    }
    let (s0, s1) = ends(view, s)?;
    let (o0, o1) = ends(view, o)?;
    if s0 == s1 || o0 == o1 {
        return Err(TopoError::InvalidParameter("join_edges of a closed edge"));
    }
    let mut candidates: Vec<EntityId> = [s0, s1].into_iter().filter(|v| *v == o0 || *v == o1).collect();
    if candidates.is_empty() {
        return Err(TopoError::InvalidParameter("edges do not share a vertex"));
    }
    // The vertex dissolves, so nothing else may touch it.
    candidates.retain(|&m| {
        let cob = view.coboundary(m);
        cob.len() == 2 && cob.iter().any(|i| i.cell == s) && cob.iter().any(|i| i.cell == o)
    });
    if candidates.is_empty() {
        return Err(TopoError::InvalidParameter("shared vertex has other incident cells"));
    }
    // Two valid vertices (a ring of two edges): dissolve the higher id, the lower one survives
    // as the end of the closed result. The first vertex whose curve ranges are adjacent wins.
    candidates.sort_by(|a, b| b.cmp(a));
    let mut last_err = None;
    let mut found = None;
    for m in candidates {
        match edge_join_at(view, s, o, m) {
            Ok(j) => {
                found = Some(j);
                break;
            }
            Err(e) => last_err = Some(e),
        }
    }
    let j = match (found, last_err) {
        (Some(j), _) => j,
        (None, Some(e)) => return Err(e),
        (None, None) => return Err(TopoError::InvalidParameter("edges do not share a vertex")),
    };

    // Parent faces: only plain loop uses can be rewritten.
    let mut faces: BTreeSet<EntityId> = BTreeSet::new();
    for e in [s, o] {
        for u in view.coboundary(e) {
            if Dim::of_real_kind(u.cell.kind()) != Some(Dim::Face) || u.ring == Incidence::EMBEDDED {
                return Err(TopoError::Unsupported("join_edges: edge is embedded or a body member"));
            }
            faces.insert(u.cell);
        }
    }
    let mut face_rows = Vec::with_capacity(faces.len());
    let mut pcurves: Vec<(CoedgeKey, Option<PCurve>)> = Vec::new();
    let tol = view.tolerance().parametric;
    for &f in &faces {
        let mut out: Vec<Row> = Vec::new();
        for ring in rings(view.boundary(f))? {
            let n = ring.len();
            let mut keep = vec![true; n];
            for i in 0..n {
                if ring[i].cell != s {
                    continue;
                }
                let side = ring[i].side;
                let o_after = (side == Side::Left) == j.m_is_s_end;
                let k = if o_after { (i + 1) % n } else { (i + n - 1) % n };
                let o_side = if j.rev_o { ring[k].side.flip() } else { ring[k].side };
                if k == i || ring[k].cell != o || !keep[k] || o_side != side {
                    return Err(TopoError::InvalidParameter("join_edges: a loop does not pass the vertex through both edges"));
                }
                keep[k] = false;
                let ks = CoedgeKey { face: f, edge: s, side };
                let ko = CoedgeKey { face: f, edge: o, side: ring[k].side };
                let (ps, po) = (view.pcurve(ks).cloned(), view.pcurve(ko).cloned());
                if ps.is_some() || po.is_some() {
                    pcurves.push((ks, merge_pcurves(ps, po, tol)?));
                    pcurves.push((ko, None));
                }
            }
            if (0..n).any(|i| ring[i].cell == o && keep[i]) {
                return Err(TopoError::InvalidParameter("join_edges: a loop uses the second edge without the first"));
            }
            out.push(ring.iter().zip(&keep).filter(|(_, k)| **k).map(|(e, _)| *e).collect());
        }
        face_rows.push((f, flatten(&out)));
    }
    let mut row = vec![Incidence::new(j.start, Side::Left, 0), Incidence::new(j.end, Side::Right, 0)];
    normalize_row(Dim::Edge, &mut row);
    let (ts, to) = (view.tolerance_of(s), view.tolerance_of(o));
    let tolerance = (to > ts).then_some(to);
    Ok(JoinEdgesPlan { survivor: s, absorbed: o, vertex: j.m, carrier: j.carrier, row, tolerance, face_rows, pcurves })
}

/// What [`TopologyEdit::join_faces`](crate::TopologyEdit::join_faces) does, decided.
#[derive(Clone, Debug, PartialEq)]
pub struct JoinFacesPlan {
    /// The surviving face.
    pub keep: EntityId,
    /// The face absorbed into `keep` (`Merged` lineage).
    pub drop: EntityId,
    /// The survivor's new boundary row (normalized).
    pub keep_row: Vec<Incidence>,
    /// Coedge pcurves of the shared chain, on both faces: removed.
    pub pcurve_drops: Vec<CoedgeKey>,
    /// Pcurves of the absorbed face's remaining coedges: `(from, to)` re-keys onto the survivor.
    pub pcurve_moves: Vec<(CoedgeKey, CoedgeKey)>,
    /// The shared chain edges, ascending (`Deleted` lineage).
    pub chain_edges: Vec<EntityId>,
    /// Chain vertices left without any edge, ascending (`Deleted` lineage).
    pub chain_vertices: Vec<EntityId>,
    /// The survivor's new tolerance override, if the absorbed face's was larger.
    pub tolerance: Option<f64>,
    /// An inner loop disappeared (the interior-loop case).
    pub inner_loops_changed: bool,
}

type Uses = BTreeMap<EntityId, Vec<Side>>;

fn uses(rings: &[Row]) -> Uses {
    let mut m: Uses = BTreeMap::new();
    for e in rings.iter().flatten() {
        m.entry(e.cell).or_default().push(e.side);
    }
    m
}

/// The single run of non-shared coedges of a ring, in traversal order.
fn arc(ring: &[Incidence], shared: &BTreeSet<EntityId>) -> Result<Row, TopoError> {
    let n = ring.len();
    let starts: Vec<usize> = (0..n).filter(|&i| !shared.contains(&ring[i].cell) && shared.contains(&ring[(i + n - 1) % n].cell)).collect();
    let [i0] = starts[..] else { return Err(TopoError::Unsupported("join_faces: shared edges are not one connected chain")) };
    Ok((0..n).map(|k| ring[(i0 + k) % n]).take_while(|e| !shared.contains(&e.cell)).collect())
}

/// Plan `join_faces(keep, drop)`: see [`TopologyEdit::join_faces`](crate::TopologyEdit::join_faces).
///
/// # Errors
/// As documented on the operator; the view is never changed.
pub fn plan_join_faces(view: &dyn TopologyView, keep: EntityId, drop: EntityId) -> Result<JoinFacesPlan, TopoError> {
    live(view, keep, Dim::Face)?;
    live(view, drop, Dim::Face)?;
    if keep == drop {
        return Err(TopoError::InvalidParameter("join_faces needs two different faces"));
    }
    match (view.carrier(keep), view.carrier(drop)) {
        (Carrier::Surface(a), Carrier::Surface(b)) if a.surface == b.surface && a.sense == b.sense => {}
        (Carrier::None, Carrier::None) => {}
        _ => return Err(TopoError::NotCoincident("faces differ in surface carrier")),
    }
    if view.flags(keep) != view.flags(drop) {
        return Err(TopoError::NotCoincident("faces differ in flags"));
    }
    let volume_uses = |f: EntityId| -> Result<Vec<Incidence>, TopoError> {
        let mut v = Vec::new();
        for u in view.coboundary(f) {
            if Dim::of_real_kind(u.cell.kind()) != Some(Dim::Volume) {
                return Err(TopoError::Unsupported("join_faces: face is a body member or embedded"));
            }
            v.push(*u);
        }
        v.sort();
        Ok(v)
    };
    if volume_uses(keep)? != volume_uses(drop)? {
        return Err(TopoError::NotCoincident("faces differ in volume uses"));
    }
    let (kr, dr) = (rings(view.boundary(keep))?, rings(view.boundary(drop))?);
    if kr.is_empty() || dr.is_empty() {
        return Err(TopoError::EmptyLoop);
    }
    let (ku, du) = (uses(&kr), uses(&dr));
    let mut shared: BTreeSet<EntityId> = ku.keys().filter(|e| du.contains_key(e)).copied().collect();
    if shared.is_empty() {
        return Err(TopoError::InvalidParameter("faces do not share an edge"));
    }
    for e in &shared {
        let (a, b) = (&ku[e], &du[e]);
        if a.len() != 1 || b.len() != 1 {
            return Err(TopoError::Unsupported("join_faces: a shared edge is used more than once by a face"));
        }
        if a[0] == b[0] {
            return Err(TopoError::NotCoincident("faces run the same way along a shared edge"));
        }
        let cob = view.coboundary(*e);
        if cob.len() != 2 || cob.iter().any(|i| i.cell != keep && i.cell != drop) {
            return Err(TopoError::NotCoincident("a shared edge has other faces"));
        }
    }
    let count = |r: &Row| r.iter().filter(|e| shared.contains(&e.cell)).count();
    let partial = |rs: &[Row]| -> Vec<usize> { (0..rs.len()).filter(|&i| count(&rs[i]) > 0 && count(&rs[i]) < rs[i].len()).collect() };
    let full = |rs: &[Row]| -> Vec<usize> { (0..rs.len()).filter(|&i| count(&rs[i]) == rs[i].len()).collect() };
    let (pk, pd, fk, fd) = (partial(&kr), partial(&dr), full(&kr), full(&dr));

    let (new_rings, changed, endpoints): (Vec<Row>, bool, Vec<EntityId>) = if pk.len() == 1 && pd.len() == 1 && fk.is_empty() && fd.is_empty() {
        // A chord, chain or seam cut: the two outer loops fuse along the chain.
        if pk[0] != 0 || pd[0] != 0 {
            return Err(TopoError::Unsupported("join_faces: the chain must lie on the outer loops"));
        }
        let arcs = |sh: &BTreeSet<EntityId>| Ok::<_, TopoError>((arc(&kr[0], sh)?, arc(&dr[0], sh)?));
        let (ka, da) = match arcs(&shared) {
            Ok(a) => a,
            Err(e) if shared.len() == 2 => {
                // A periodic face cut by one chord: the pieces share the chord and the seam,
                // which are disconnected. The chord dissolves; the seam stays and is used twice
                // by the merged face.
                let [x, y] = [*shared.first().unwrap_or(&keep), *shared.last().unwrap_or(&keep)];
                let seam_like = |e: EntityId| match (view.pcurve(CoedgeKey { face: keep, edge: e, side: ku[&e][0] }), view.pcurve(CoedgeKey { face: drop, edge: e, side: du[&e][0] })) {
                    (Some(a), Some(b)) => a != b,
                    _ => false,
                };
                // The seam is the edge whose two pcurves differ (u = 0 against u = 2 pi); with no
                // pcurves to tell, the older edge is the seam.
                let seam = if seam_like(y) && !seam_like(x) { y } else { x };
                let chord = BTreeSet::from([if seam == x { y } else { x }]);
                let a = arcs(&chord).map_err(|_| e)?;
                shared = chord;
                a
            }
            Err(e) => return Err(e),
        };
        let (k_tail, k_head) = (tail_head(view, &ka[0])?.0, tail_head(view, &ka[ka.len() - 1])?.1);
        let (d_tail, d_head) = (tail_head(view, &da[0])?.0, tail_head(view, &da[da.len() - 1])?.1);
        if k_head != d_tail || d_head != k_tail {
            return Err(TopoError::InvalidParameter("join_faces: the arcs do not close into one loop"));
        }
        let mut out: Vec<Row> = vec![ka.into_iter().chain(da).collect()];
        out.extend(kr[1..].iter().cloned());
        out.extend(dr[1..].iter().cloned());
        (out, false, vec![k_tail, k_head])
    } else if pk.is_empty() && pd.is_empty() && fk.len() == 1 && fd.len() == 1 {
        // An interior loop: one face is the disk bounded by a hole of the other.
        let (ki, dj) = (fk[0], fd[0]);
        if (ki == 0) == (dj == 0) {
            return Err(TopoError::Unsupported("join_faces: shared loops must be an outer loop and a hole"));
        }
        let (mut a, mut b) = (kr[ki].clone(), dr[dj].clone());
        let rev: Row = b.iter().rev().map(|e| Incidence { side: e.side.flip(), ..*e }).collect();
        b = rev;
        canonical_rotate(&mut a);
        canonical_rotate(&mut b);
        if a.iter().map(|e| (e.cell, e.side)).ne(b.iter().map(|e| (e.cell, e.side))) {
            return Err(TopoError::NotCoincident("shared loops do not correspond"));
        }
        let mut out: Vec<Row> = Vec::new();
        if ki == 0 {
            // `keep` is the disk: the result is `drop` without the hole, plus the disk's holes.
            out.extend(dr.iter().enumerate().filter(|(i, _)| *i != dj).map(|(_, r)| r.clone()));
            out.extend(kr[1..].iter().cloned());
        } else {
            out.extend(kr.iter().enumerate().filter(|(i, _)| *i != ki).map(|(_, r)| r.clone()));
            out.extend(dr[1..].iter().cloned());
        }
        (out, true, Vec::new())
    } else {
        return Err(TopoError::Unsupported("join_faces: shared edges are not one connected chain"));
    };

    // A coedge may appear once per face.
    let mut seen: BTreeSet<(EntityId, Side)> = BTreeSet::new();
    for e in new_rings.iter().flatten() {
        if !seen.insert((e.cell, e.side)) {
            return Err(TopoError::InvalidParameter("join_faces: the result would use a coedge twice"));
        }
    }

    // Chain vertices that no other edge touches die with the chain; the chain ends stay.
    let mut chain_vertices: BTreeSet<EntityId> = BTreeSet::new();
    for &e in &shared {
        let (a, b) = ends(view, e)?;
        for v in [a, b] {
            if endpoints.contains(&v) {
                continue;
            }
            if view.coboundary(v).iter().any(|i| !shared.contains(&i.cell)) {
                return Err(TopoError::Unsupported("join_faces: a chain vertex has other incident cells"));
            }
            chain_vertices.insert(v);
        }
    }

    let in_shared = |e: &Incidence| shared.contains(&e.cell);
    let mut pcurve_drops = Vec::new();
    for e in shared.iter() {
        pcurve_drops.push(CoedgeKey { face: keep, edge: *e, side: ku[e][0] });
        pcurve_drops.push(CoedgeKey { face: drop, edge: *e, side: du[e][0] });
    }
    let pcurve_moves = dr.iter().flatten().filter(|e| !in_shared(e)).map(|e| (CoedgeKey { face: drop, edge: e.cell, side: e.side }, CoedgeKey { face: keep, edge: e.cell, side: e.side })).collect();
    let (tk, td) = (view.tolerance_of(keep), view.tolerance_of(drop));
    Ok(JoinFacesPlan {
        keep,
        drop,
        keep_row: flatten(&new_rings),
        pcurve_drops,
        pcurve_moves,
        chain_edges: shared.into_iter().collect(),
        chain_vertices: chain_vertices.into_iter().collect(),
        tolerance: (td > tk).then_some(td),
        inner_loops_changed: changed,
    })
}
