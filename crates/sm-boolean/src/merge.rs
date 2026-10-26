//! Cubit-style `merge` and its read-only twin `merge_check`.
//!
//! Merging works vertices, then edges, then faces. Candidate pairs are processed in ascending
//! `(min id, max id)` order and the lowest id survives, so the result does not depend on any
//! traversal order. A pair is *mergeable* when its measured gap is within the cells' own
//! tolerances (or the op-level `tol`), *near* when the gap is above that but within the model's
//! `linear_max`, *overlapping* for coincident faces whose boundaries differ, *conflict* when
//! merging would be invalid (both faces carry a volume on the side the merge would fill, or two
//! vertices bound the same edge), and *skipped* when a face is on the caller's skip list.

use std::collections::{BTreeMap, BTreeSet};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, Point3};
use sm_geom_api::{SurfaceDef, Uv};
use sm_topo_api::{Coincidence, Dim, EntityState, Sense, Side, TopologyEdit, TopologyView, TopologyViewExt};

use crate::error::BooleanError;
use crate::geo;
use crate::poly2;

/// Which cell ranks a merge touches.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct MergeKinds {
    /// Merge coincident vertices.
    pub vertex: bool,
    /// Merge coincident edges.
    pub edge: bool,
    /// Merge coincident faces.
    pub face: bool,
}

impl Default for MergeKinds {
    fn default() -> Self {
        Self { vertex: true, edge: true, face: true }
    }
}

/// Options of [`merge`] and [`merge_check`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct MergeOptions {
    /// Op-level merge tolerance (Cubit's `merge tolerance`); `None` uses the cells' own
    /// tolerances. Above `ModelTolerance::linear_max` it is refused.
    pub tol: Option<f64>,
    /// Cell ranks to merge.
    pub kinds: MergeKinds,
}

/// The rank of a merge candidate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum MergeKind {
    /// Two vertices.
    Vertex,
    /// Two edges.
    Edge,
    /// Two faces.
    Face,
}

/// What a candidate pair is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum MergeStatus {
    /// Coincident within tolerance: `merge` merges it.
    Mergeable,
    /// Closer than `linear_max` but not within tolerance: raise `tol` to merge it.
    Near,
    /// Coincident faces whose boundaries differ: imprint first.
    Overlapping,
    /// Merging would be invalid.
    Conflict,
    /// On the skip list, never merged.
    Skipped,
}

/// Why a pair is skipped.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub enum SkipReason {
    /// The face is an interface face of the model's boundary tables.
    Interface,
}

/// One candidate pair of a merge analysis.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct MergePair {
    /// Rank of the cells.
    pub kind: MergeKind,
    /// The lower id (the survivor of a merge).
    pub a: EntityId,
    /// The higher id.
    pub b: EntityId,
    /// Status.
    pub status: MergeStatus,
    /// Measured gap in model length units.
    pub gap: f64,
    /// For `Overlapping` planar faces: the area of the overlap, when it could be computed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub overlap_area: Option<f64>,
    /// Set when `status` is `Skipped`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reason: Option<SkipReason>,
    /// What to do about it.
    pub hint: String,
}

/// A merge that was performed.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct MergedCell {
    /// Rank.
    pub kind: MergeKind,
    /// The survivor.
    pub keep: EntityId,
    /// The absorbed cell (retired).
    pub drop: EntityId,
    /// The gap the merge discarded.
    pub gap: f64,
}

/// The result of an analysis or a merge.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct MergeReport {
    /// Every candidate pair found, vertices first, then edges, then faces, each ascending by
    /// `(a, b)`. For [`merge`] this is the pre-merge status: `Mergeable` pairs were merged.
    pub pairs: Vec<MergePair>,
    /// The merges [`merge`] performed (empty for [`merge_check`]).
    #[serde(default)]
    pub merged: Vec<MergedCell>,
    /// Set by [`merge_check`] when the options are refused (the read-only twin cannot return an
    /// error); [`merge`] returns it as `BooleanError::Invalid`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

impl MergeReport {
    /// Pairs of one status.
    pub fn with_status(&self, kind: MergeKind, status: MergeStatus) -> impl Iterator<Item = &MergePair> {
        self.pairs.iter().filter(move |p| p.kind == kind && p.status == status)
    }
}

struct Scope {
    vertices: Vec<EntityId>,
    edges: Vec<EntityId>,
    faces: Vec<EntityId>,
}

fn resolve_scope<V: TopologyView + ?Sized>(view: &V, scope: &[EntityId]) -> Result<Scope, BooleanError> {
    let mut cells: BTreeSet<EntityId> = BTreeSet::new();
    if scope.is_empty() {
        for d in [Dim::Vertex, Dim::Edge, Dim::Face] {
            cells.extend(view.cells(d).iter().copied());
        }
    } else {
        for &id in scope {
            if !view.is_alive(id) {
                return Err(BooleanError::Invalid(format!("{id} is not a live topology cell")));
            }
            cells.extend(view.closure(id));
        }
    }
    let pick = |d: Dim| cells.iter().copied().filter(|c| Dim::of_real_kind(c.kind()) == Some(d) && view.is_alive(*c)).collect::<Vec<_>>();
    Ok(Scope { vertices: pick(Dim::Vertex), edges: pick(Dim::Edge), faces: pick(Dim::Face) })
}

fn check_tol<V: TopologyView + ?Sized>(view: &V, o: &MergeOptions) -> Result<(), BooleanError> {
    if let Some(t) = o.tol {
        let max = view.tolerance().linear_max;
        if !(t.is_finite() && t >= 0.0) || t > max {
            return Err(BooleanError::Invalid(format!("merge tolerance {t:e} is outside [0, linear_max = {max:e}]; lower `tol` or raise the model's linear_max")));
        }
    }
    Ok(())
}

/// Classify a measured gap between two cells.
fn gap_status<V: TopologyView + ?Sized>(view: &V, a: EntityId, b: EntityId, gap: f64, o: &MergeOptions) -> Option<MergeStatus> {
    let tol = view.tolerance();
    if tol.coincident(gap, view.tolerance_of(a), view.tolerance_of(b)) || o.tol.is_some_and(|t| gap <= t) {
        Some(MergeStatus::Mergeable)
    } else if gap <= tol.linear_max {
        Some(MergeStatus::Near)
    } else {
        None
    }
}

fn near_hint(kind: MergeKind, gap: f64) -> String {
    format!("{kind:?} pair is {gap:.3e} apart, within linear_max but not within tolerance: pass tol >= {gap:.3e} to merge it")
}

/// Vertex candidate pairs by an x-sorted sweep, ascending `(a, b)`.
fn vertex_pairs<V: TopologyView + ?Sized>(view: &V, verts: &[EntityId], o: &MergeOptions) -> Vec<MergePair> {
    let reach = view.tolerance().linear_max;
    let mut pts: Vec<(f64, EntityId, Point3)> = verts.iter().filter_map(|&v| geo::vertex_point(view, v).map(|p| (p.x, v, p))).collect();
    pts.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    let mut out = Vec::new();
    for i in 0..pts.len() {
        for j in (i + 1)..pts.len() {
            if pts[j].0 - pts[i].0 > reach {
                break;
            }
            let d = geo::dist(pts[i].2, pts[j].2);
            let (a, b) = if pts[i].1 < pts[j].1 { (pts[i].1, pts[j].1) } else { (pts[j].1, pts[i].1) };
            let Some(status) = gap_status(view, a, b, d, o) else { continue };
            // Two ends of one edge must not merge: the edge would collapse.
            let joined = view.coboundary(a).iter().any(|e| Dim::of_real_kind(e.cell.kind()) == Some(Dim::Edge) && view.edge_ends(e.cell).is_some_and(|(s, t)| (s == b && t == a) || (s == a && t == b)));
            let (status, hint) = if joined {
                (MergeStatus::Conflict, format!("{a} and {b} bound one edge, which would collapse: the edge is shorter than the tolerance"))
            } else if status == MergeStatus::Near {
                (status, near_hint(MergeKind::Vertex, d))
            } else {
                (status, "coincident vertices".to_owned())
            };
            out.push(MergePair { kind: MergeKind::Vertex, a, b, status, gap: d, overlap_area: None, reason: None, hint });
        }
    }
    out.sort_by_key(|p| (p.a, p.b));
    out
}

/// Distance from `p` to the edge `e` (restricted to the edge's parameter range).
fn dist_point_edge<V: TopologyView + ?Sized>(view: &V, e: EntityId, p: Point3) -> f64 {
    let ends = view.edge_ends(e).and_then(|(s, t)| Some((geo::vertex_point(view, s)?, geo::vertex_point(view, t)?)));
    if let Some((c, cc)) = geo::edge_curve(view, e) {
        if let Ok(pr) = c.project(p, None) {
            let dom = c.domain();
            let mut t = pr.t;
            if let Some(period) = dom.period {
                while t < cc.range.lo {
                    t += period;
                }
                while t > cc.range.lo + period {
                    t -= period;
                }
            }
            if cc.range.contains(t, 1e-9 * cc.range.len().abs().max(1.0)) {
                return pr.distance;
            }
        }
        if let Some((a, b)) = ends {
            return geo::dist(p, a).min(geo::dist(p, b));
        }
        return f64::INFINITY;
    }
    match ends {
        Some((a, b)) => {
            let ab = b - a;
            let l2 = ab.dot(ab);
            let t = if l2 > 0.0 { ((p - a).dot(ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
            geo::dist(p, geo::lerp(a, b, t))
        }
        None => f64::INFINITY,
    }
}

/// The measured gap and relative sense of `drop` against `keep`, comparing both edges at nine
/// fractions by closest distance to the other edge.
fn edge_gap<V: TopologyView + ?Sized>(view: &V, keep: EntityId, drop: EntityId, rep: &dyn Fn(EntityId) -> EntityId) -> Option<(f64, Sense)> {
    let (sk, tk) = view.edge_ends(keep)?;
    let (sd, td) = view.edge_ends(drop)?;
    let (sk, tk, sd, td) = (rep(sk), rep(tk), rep(sd), rep(td));
    let closed = sk == tk;
    let rel = if !closed {
        if sd == sk && td == tk {
            Sense::Forward
        } else if sd == tk && td == sk {
            Sense::Reversed
        } else {
            return None;
        }
    } else {
        if sd != sk || td != tk {
            return None;
        }
        let q = geo::edge_sample(view, drop, 0.25)?;
        let (f, r) = (geo::edge_sample(view, keep, 0.25)?, geo::edge_sample(view, keep, 0.75)?);
        if geo::dist(q, f) <= geo::dist(q, r) { Sense::Forward } else { Sense::Reversed }
    };
    let mut gap = 0.0f64;
    for k in 0..=8 {
        let f = k as f64 / 8.0;
        let pd = geo::edge_sample(view, drop, f)?;
        let pk = geo::edge_sample(view, keep, f)?;
        let _ = pk;
        gap = gap.max(dist_point_edge(view, keep, pd));
        let pk = geo::edge_sample(view, keep, f)?;
        gap = gap.max(dist_point_edge(view, drop, pk));
    }
    Some((gap, rel))
}

struct EdgePair {
    pair: MergePair,
    rel: Sense,
}

fn edge_pairs<V: TopologyView + ?Sized>(view: &V, edges: &[EntityId], rep: &dyn Fn(EntityId) -> EntityId, o: &MergeOptions) -> Vec<EdgePair> {
    let mut buckets: BTreeMap<(EntityId, EntityId), Vec<EntityId>> = BTreeMap::new();
    for &e in edges {
        if geo::is_degenerate(view, e) {
            continue;
        }
        let Some((s, t)) = view.edge_ends(e) else { continue };
        let (s, t) = (rep(s), rep(t));
        buckets.entry((s.min(t), s.max(t))).or_default().push(e);
    }
    let mut out = Vec::new();
    for list in buckets.values() {
        for (i, &a) in list.iter().enumerate() {
            for &b in &list[i + 1..] {
                let Some((gap, rel)) = edge_gap(view, a, b, rep) else { continue };
                let Some(status) = gap_status(view, a, b, gap, o) else { continue };
                let hint = if status == MergeStatus::Near { near_hint(MergeKind::Edge, gap) } else { "coincident edges".to_owned() };
                out.push(EdgePair { pair: MergePair { kind: MergeKind::Edge, a, b, status, gap, overlap_area: None, reason: None, hint }, rel });
            }
        }
    }
    out.sort_by_key(|p| (p.pair.a, p.pair.b));
    out
}

struct FacePairInfo {
    pair: MergePair,
    rel: Sense,
}

/// Which volume sits on each side of a face.
fn face_occupancy<V: TopologyView + ?Sized>(view: &V, f: EntityId) -> (Option<EntityId>, Option<EntityId>) {
    let (mut l, mut r) = (None, None);
    for u in view.coboundary(f) {
        if Dim::of_real_kind(u.cell.kind()) != Some(Dim::Volume) {
            continue;
        }
        match u.side {
            Side::Left => l = Some(u.cell),
            Side::Right => r = Some(u.cell),
            Side::Both => {
                l = Some(u.cell);
                r = Some(u.cell);
            }
        }
    }
    (l, r)
}

/// Would merging `drop` into `keep` (normals `rel`) put two volumes on one side?
fn side_conflict<V: TopologyView + ?Sized>(view: &V, keep: EntityId, drop: EntityId, rel: Sense) -> bool {
    let (kl, kr) = face_occupancy(view, keep);
    let (dl, dr) = face_occupancy(view, drop);
    let (dl, dr) = if rel == Sense::Reversed { (dr, dl) } else { (dl, dr) };
    (kl.is_some() && dl.is_some()) || (kr.is_some() && dr.is_some())
}

/// Planar polygon (outer loop only) of a face in the plane frame `(origin, ex, ey)`.
fn planar_outline<V: TopologyView + ?Sized>(view: &V, f: EntityId, origin: Point3, ex: sm_core::Vector3, ey: sm_core::Vector3) -> Vec<poly2::P2> {
    let mut poly = Vec::new();
    for l in view.loops(f).filter(|l| l.ring == 0) {
        for en in l.entries {
            if geo::is_degenerate(view, en.cell) {
                continue;
            }
            for k in 0..8 {
                let frac = k as f64 / 8.0;
                let fr = if en.side == Side::Right { 1.0 - frac } else { frac };
                if let Some(p) = geo::edge_sample(view, en.cell, fr) {
                    let d = p - origin;
                    poly.push([d.dot(ex), d.dot(ey)]);
                }
            }
        }
    }
    poly
}

fn plane_of<V: TopologyView + ?Sized>(view: &V, f: EntityId) -> Option<(Point3, sm_core::Vector3, sm_core::Vector3, sm_core::Vector3)> {
    let sid = match view.carrier(f) {
        sm_topo_api::Carrier::Surface(sc) => sc.surface,
        _ => return None,
    };
    match view.shapes().surface_def(sid)? {
        SurfaceDef::Plane(p) => Some((p.frame.origin, p.frame.x, p.frame.y, p.frame.z)),
        _ => None,
    }
}

fn face_pairs<V: TopologyView + ?Sized>(view: &V, faces: &[EntityId], rep_e: &dyn Fn(EntityId) -> EntityId, skip: &BTreeSet<EntityId>, o: &MergeOptions) -> Vec<FacePairInfo> {
    let pad = view.tolerance().linear_max;
    let boxes: Vec<(EntityId, sm_core::Aabb)> = faces.iter().map(|&f| (f, geo::face_aabb(view, f, pad))).collect();
    let mut order: Vec<usize> = (0..boxes.len()).collect();
    order.sort_by(|&i, &j| boxes[i].1.lo.x.total_cmp(&boxes[j].1.lo.x).then(boxes[i].0.cmp(&boxes[j].0)));
    let mut out = Vec::new();
    for (oi, &i) in order.iter().enumerate() {
        for &j in &order[oi + 1..] {
            if boxes[j].1.lo.x > boxes[i].1.hi.x {
                break;
            }
            if !geo::aabb_overlap(&boxes[i].1, &boxes[j].1) {
                continue;
            }
            let (a, b) = if boxes[i].0 < boxes[j].0 { (boxes[i].0, boxes[j].0) } else { (boxes[j].0, boxes[i].0) };
            if let Some(info) = classify_face_pair(view, a, b, rep_e, skip, o) {
                out.push(info);
            }
        }
    }
    out.sort_by_key(|p| (p.pair.a, p.pair.b));
    out
}

fn classify_face_pair<V: TopologyView + ?Sized>(view: &V, a: EntityId, b: EntityId, rep_e: &dyn Fn(EntityId) -> EntityId, skip: &BTreeSet<EntityId>, o: &MergeOptions) -> Option<FacePairInfo> {
    let (sa, _) = geo::face_surface(view, a)?;
    let (sb, _) = geo::face_surface(view, b)?;
    let (da, db) = (view.shapes().surface_def(match view.carrier(a) { sm_topo_api::Carrier::Surface(s) => s.surface, _ => return None })?, view.shapes().surface_def(match view.carrier(b) { sm_topo_api::Carrier::Surface(s) => s.surface, _ => return None })?);
    if core::mem::discriminant(da) != core::mem::discriminant(db) {
        return None;
    }
    // Gap: worst distance of either boundary's samples to the other face's surface.
    let mut gap = 0.0f64;
    let pa = geo::face_boundary_samples(view, a, 4);
    let pb = geo::face_boundary_samples(view, b, 4);
    if pa.is_empty() || pb.is_empty() {
        return None;
    }
    for &p in &pb {
        gap = gap.max(sa.project(p, None).ok()?.distance);
    }
    for &p in &pa {
        gap = gap.max(sb.project(p, None).ok()?.distance);
    }
    if gap > view.tolerance().linear_max {
        return None;
    }
    // Normals at a common point must be (anti)parallel.
    let q = pb[0];
    let (na, _, _) = geo::face_normal_near(view, a, q)?;
    let (nb, _, _) = geo::face_normal_near(view, b, q)?;
    let c = na.dot(nb);
    if c.abs() < 0.999_999 {
        return None;
    }
    let rel = if c > 0.0 { Sense::Forward } else { Sense::Reversed };
    let ea: BTreeSet<EntityId> = view.face_edges(a).iter().map(|&e| rep_e(e)).collect();
    let eb: BTreeSet<EntityId> = view.face_edges(b).iter().map(|&e| rep_e(e)).collect();
    let nloops = |f: EntityId| view.loops(f).count();
    let same_edges = ea == eb && nloops(a) == nloops(b);
    let mut pair = MergePair { kind: MergeKind::Face, a, b, status: MergeStatus::Mergeable, gap, overlap_area: None, reason: None, hint: String::new() };
    if skip.contains(&a) || skip.contains(&b) {
        // Only report a skip where the faces would otherwise be a candidate.
        if same_edges || overlap_of(view, a, b, o).is_some() {
            pair.status = MergeStatus::Skipped;
            pair.reason = Some(SkipReason::Interface);
            pair.hint = "an interface face of the model's boundary tables is never merged".to_owned();
            return Some(FacePairInfo { pair, rel });
        }
        return None;
    }
    if same_edges {
        if side_conflict(view, a, b, rel) {
            pair.status = MergeStatus::Conflict;
            pair.hint = format!("{a} and {b} each carry a volume on the side the merge would fill: the volumes overlap, or the normals disagree");
            return Some(FacePairInfo { pair, rel });
        }
        match gap_status(view, a, b, gap, o)? {
            MergeStatus::Mergeable => pair.hint = "coincident faces".to_owned(),
            _ => {
                pair.status = MergeStatus::Near;
                pair.hint = near_hint(MergeKind::Face, gap);
            }
        }
        return Some(FacePairInfo { pair, rel });
    }
    let area = overlap_of(view, a, b, o)?;
    pair.status = MergeStatus::Overlapping;
    pair.overlap_area = area;
    pair.hint = format!("{a} and {b} overlap but their boundaries differ: imprint the bodies first (model.imprint), then merge");
    Some(FacePairInfo { pair, rel })
}

/// `Some(area)` when two coincident faces overlap with positive area (`Some(None)` when they
/// overlap but the area is not available), `None` when they only touch along their boundaries.
#[allow(clippy::option_option)]
fn overlap_of<V: TopologyView + ?Sized>(view: &V, a: EntityId, b: EntityId, o: &MergeOptions) -> Option<Option<f64>> {
    let tol = o.tol.unwrap_or(view.tolerance().linear).max(view.tolerance().linear);
    if let (Some((org, ex, ey, _)), Some(_)) = (plane_of(view, a), plane_of(view, b)) {
        let (pa, pb) = (planar_outline(view, a, org, ex, ey), planar_outline(view, b, org, ex, ey));
        if let Some(area) = poly2::intersection_area(&pa, &pb, tol * tol) {
            return (area > tol * tol * 100.0).then_some(Some(area));
        }
    }
    // Generic: a vertex or edge midpoint of one face strictly inside the other.
    let inside = |from: EntityId, into: EntityId| -> bool {
        let Some((s, _)) = geo::face_surface(view, into) else { return false };
        geo::face_boundary_samples(view, from, 2).into_iter().any(|p| {
            let Ok(pr) = s.project(p, None) else { return false };
            matches!(view.classify_point_on_face(into, Uv::new(pr.uv.u, pr.uv.v)), Ok(sm_topo_api::PointClass::In))
        })
    };
    (inside(a, b) || inside(b, a)).then_some(None)
}

/// Read-only merge analysis: every candidate pair of vertices, edges and faces in `scope`
/// (empty = all cells), with its status, measured gap and a hint. `skip_faces` are reported as
/// `Skipped` and never merged. Edge pairs are matched through the vertex pairs that are
/// mergeable or near, and face pairs through the edge pairs, so one call shows the whole chain
/// a [`merge`] would follow once `tol` is raised far enough. An invalid `tol` is returned in
/// [`MergeReport::error`].
pub fn merge_check(view: &dyn TopologyView, scope: &[EntityId], skip_faces: &[EntityId], o: &MergeOptions) -> MergeReport {
    match analyze(view, scope, skip_faces, o) {
        Ok(r) => r,
        Err(e) => MergeReport { error: Some(e.to_string()), ..MergeReport::default() },
    }
}

fn analyze<V: TopologyView + ?Sized>(view: &V, scope: &[EntityId], skip_faces: &[EntityId], o: &MergeOptions) -> Result<MergeReport, BooleanError> {
    check_tol(view, o)?;
    let sc = resolve_scope(view, scope)?;
    let skip: BTreeSet<EntityId> = skip_faces.iter().copied().collect();
    let mut report = MergeReport::default();
    // Virtual representatives: cluster by mergeable and near pairs.
    let vp = if o.kinds.vertex { vertex_pairs(view, &sc.vertices, o) } else { Vec::new() };
    let rep_v = representatives(vp.iter().filter(|p| matches!(p.status, MergeStatus::Mergeable | MergeStatus::Near)));
    let rv = |v: EntityId| *rep_v.get(&v).unwrap_or(&v);
    report.pairs.extend(vp.iter().cloned());
    let ep = if o.kinds.edge { edge_pairs(view, &sc.edges, &rv, o) } else { Vec::new() };
    let rep_e = representatives(ep.iter().map(|p| &p.pair));
    let re = |e: EntityId| *rep_e.get(&e).unwrap_or(&e);
    report.pairs.extend(ep.iter().map(|p| p.pair.clone()));
    if o.kinds.face {
        report.pairs.extend(face_pairs(view, &sc.faces, &re, &skip, o).into_iter().map(|p| p.pair));
    }
    Ok(report)
}

/// Map each cell of a pair set to the lowest id of its connected cluster.
fn representatives<'a>(pairs: impl Iterator<Item = &'a MergePair>) -> BTreeMap<EntityId, EntityId> {
    let mut parent: BTreeMap<EntityId, EntityId> = BTreeMap::new();
    fn find(parent: &mut BTreeMap<EntityId, EntityId>, x: EntityId) -> EntityId {
        let p = *parent.entry(x).or_insert(x);
        if p == x {
            return x;
        }
        let r = find(parent, p);
        parent.insert(x, r);
        r
    }
    for p in pairs {
        let (ra, rb) = (find(&mut parent, p.a), find(&mut parent, p.b));
        if ra != rb {
            let (lo, hi) = (ra.min(rb), ra.max(rb));
            parent.insert(hi, lo);
        }
    }
    let keys: Vec<EntityId> = parent.keys().copied().collect();
    keys.into_iter().map(|k| (k, find(&mut parent, k))).collect()
}

/// Merge coincident vertices, then edges, then faces in `scope` (empty = all cells), skipping
/// `skip_faces`. Runs inside an open operation; on `Err` the caller aborts it.
///
/// Candidate pairs are processed in ascending `(min id, max id)` order, the lowest id survives,
/// and the gaps are recorded by the `merge_*` operators (`Merged { gap }` lineage and the
/// `Coincidence::Merged` record). Pairs that are only `Near`, `Overlapping`, `Conflict` or
/// `Skipped` are reported but left alone.
///
/// # Errors
/// `Invalid` if `tol` is negative, not finite or above the model's `linear_max`; `Topo` if an
/// operator refuses a merge the analysis predicted valid.
pub fn merge<E: TopologyEdit + ?Sized>(topo: &mut E, scope: &[EntityId], skip_faces: &[EntityId], o: &MergeOptions) -> Result<MergeReport, BooleanError> {
    check_tol(&*topo, o)?;
    let sc = resolve_scope(&*topo, scope)?;
    let skip: BTreeSet<EntityId> = skip_faces.iter().copied().collect();
    let mut report = MergeReport::default();
    let alive = |t: &E, c: EntityId| t.state(c) == Some(EntityState::Live);

    if o.kinds.vertex {
        let vp = vertex_pairs(&*topo, &sc.vertices, o);
        report.pairs.extend(vp.iter().cloned());
        for p in vp.iter().filter(|p| p.status == MergeStatus::Mergeable) {
            if !alive(topo, p.a) || !alive(topo, p.b) {
                continue;
            }
            topo.merge_vertices(p.a, p.b, o.tol)?;
            report.merged.push(MergedCell { kind: MergeKind::Vertex, keep: p.a, drop: p.b, gap: p.gap });
        }
    }
    if o.kinds.edge {
        let edges: Vec<EntityId> = sc.edges.iter().copied().filter(|&e| alive(topo, e)).collect();
        let ident = |c: EntityId| c;
        let ep = edge_pairs(&*topo, &edges, &ident, o);
        report.pairs.extend(ep.iter().map(|p| p.pair.clone()));
        for p in ep.iter().filter(|p| p.pair.status == MergeStatus::Mergeable) {
            if !alive(topo, p.pair.a) || !alive(topo, p.pair.b) {
                continue;
            }
            topo.merge_edges(p.pair.a, p.pair.b, p.rel)?;
            report.merged.push(MergedCell { kind: MergeKind::Edge, keep: p.pair.a, drop: p.pair.b, gap: p.pair.gap });
        }
    }
    if o.kinds.face {
        let faces: Vec<EntityId> = sc.faces.iter().copied().filter(|&f| alive(topo, f)).collect();
        let ident = |c: EntityId| c;
        let fp = face_pairs(&*topo, &faces, &ident, &skip, o);
        report.pairs.extend(fp.iter().map(|p| p.pair.clone()));
        for p in fp.iter().filter(|p| p.pair.status == MergeStatus::Mergeable) {
            if !alive(topo, p.pair.a) || !alive(topo, p.pair.b) {
                continue;
            }
            // The pair goes imprinted-not-merged -> merged, as in `fixtures::merge_touching_bricks`.
            topo.record_coincidence(p.pair.a, p.pair.b, Coincidence::ImprintedNotMerged)?;
            topo.merge_faces(p.pair.a, p.pair.b, p.rel)?;
            report.merged.push(MergedCell { kind: MergeKind::Face, keep: p.pair.a, drop: p.pair.b, gap: p.pair.gap });
        }
    }
    Ok(report)
}
