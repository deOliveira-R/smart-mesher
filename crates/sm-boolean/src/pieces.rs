//! Stages 1-3 of the kernel: candidate face pairs, surface-surface intersection, and clipping of
//! the intersection curves to the faces. The output is a list of [`Piece`]s: curve segments with
//! the faces they must be inserted into. Nothing here edits the topology.

use rayon::prelude::*;
use smallvec::SmallVec;
use sm_core::{Aabb, EntityId, Point3};
use sm_geom::{CurveGeometry, CurveSurface, SsiOptions, SsiResult, curve_surface, intersect_surfaces};
use sm_geom_api::{CurveDef, Interval, ModelTolerance, SurfaceDef};
use sm_topo_api::{Carrier, PointClass, TopologyView, TopologyViewExt};

use crate::cc;
use crate::error::BooleanError;
use crate::faceclass;
use crate::geo;

/// What the faces of a pair are cut with.
#[derive(Clone, Debug)]
pub(crate) enum Other {
    /// A face of the complex.
    Face(EntityId),
    /// A bare tool surface (webcut): the pieces are clipped to the target face only.
    Surface(SurfaceDef),
}

/// One candidate pair: a face and what cuts it.
#[derive(Clone, Debug)]
pub(crate) struct PairSpec {
    pub a: EntityId,
    pub b: Other,
}

/// Where the geometry of a piece comes from.
#[derive(Clone, Debug)]
pub(crate) enum CurveSrc {
    /// A new curve definition (added to the store once per `(group, key)`).
    New(CurveDef),
    /// A curve handle already in the store (an existing edge's curve).
    Existing(EntityId),
}

/// A curve segment to be inserted into faces.
#[derive(Clone, Debug)]
pub(crate) struct Piece {
    pub src: CurveSrc,
    /// Pieces with the same `(group, key)` share one curve handle.
    pub key: usize,
    pub range: Interval,
    /// The piece is a whole closed curve (its start and end are one vertex).
    pub closed: bool,
    /// Faces the piece must be inserted into.
    pub faces: SmallVec<[EntityId; 2]>,
    /// Faces whose boundary the piece already runs along.
    pub on_boundary: SmallVec<[EntityId; 2]>,
    /// Apply group: pieces of different groups never share vertices or edges (imprint without
    /// merge keeps each body self-contained).
    pub group: usize,
    /// An existing edge this piece is a part of (a coincident-face imprint): in a shared apply
    /// its sub-edge is used instead of a new edge.
    pub source_edge: Option<EntityId>,
    /// Index of the pair that produced the piece.
    pub pair: usize,
}

/// How pieces are shared between the two faces of a pair.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Share {
    /// One piece, inserted into both faces; edges are shared.
    Shared,
    /// Two pieces (group 0 for `a`, group 1 for `b`): each face gets its own copy.
    Private,
}

/// Result of [`compute`].
#[derive(Clone, Debug, Default)]
pub(crate) struct Computed {
    pub pieces: Vec<Piece>,
    /// Pairs found coincident (same surface), as `(a, b)`.
    pub coincident: Vec<(EntityId, EntityId)>,
    /// Isolated tangent contact points.
    pub tangent_points: Vec<Point3>,
    /// True if some pair intersected in curves (even if clipped away).
    pub any_curves: bool,
}

pub(crate) fn surface_def_of<'a, V: TopologyView + ?Sized>(view: &'a V, f: EntityId) -> Option<&'a SurfaceDef> {
    match view.carrier(f) {
        Carrier::Surface(sc) => view.shapes().surface_def(sc.surface),
        _ => None,
    }
}

fn pad_box(b: Aabb, pad: f64) -> Aabb {
    Aabb { lo: Point3::new(b.lo.x - pad, b.lo.y - pad, b.lo.z - pad), hi: Point3::new(b.hi.x + pad, b.hi.y + pad, b.hi.z + pad) }
}

/// Candidate pairs `(a, b)` with `a` from `fa` and `b` from `fb` whose padded boxes overlap,
/// ascending by `(a, b)`. Boxes are computed in parallel and collected in index order.
pub(crate) fn candidate_pairs<V: TopologyView + ?Sized>(view: &V, fa: &[EntityId], fb: &[EntityId], pad: f64) -> Vec<(EntityId, EntityId)> {
    let boxes = |list: &[EntityId]| -> Vec<(EntityId, Aabb)> { list.par_iter().map(|&f| (f, geo::face_aabb(view, f, pad))).collect() };
    let (ba, bb) = (boxes(fa), boxes(fb));
    let mut out = Vec::new();
    for (a, boxa) in &ba {
        for (b, boxb) in &bb {
            if a != b && geo::aabb_overlap(boxa, boxb) {
                out.push((*a, *b));
            }
        }
    }
    out.sort();
    out
}

struct PairOut {
    pieces: Vec<Proto>,
    coincident: bool,
    tangent: Vec<Point3>,
    any_curves: bool,
    next_idx: usize,
}

/// A piece before group and key assignment.
struct Proto {
    src: CurveSrc,
    curve_idx: usize,
    range: Interval,
    closed: bool,
    faces: SmallVec<[EntityId; 2]>,
    on_boundary: SmallVec<[EntityId; 2]>,
    source_edge: Option<EntityId>,
    /// Faces of the pair that the piece lies in (both, for group assignment in private mode).
    both: (Option<EntityId>, Option<EntityId>),
}

/// Compute the pieces of `pairs`. The pairs are evaluated in parallel and joined in index order.
pub(crate) fn compute<V: TopologyView + ?Sized>(view: &V, pairs: &[PairSpec], share: Share, group_of: &dyn Fn(EntityId) -> usize, window_pad: f64) -> Result<Computed, BooleanError> {
    let outs: Vec<Result<PairOut, BooleanError>> = pairs.par_iter().map(|p| compute_pair(view, p, window_pad)).collect();
    let mut c = Computed::default();
    let mut key = 0usize;
    for (pi, out) in outs.into_iter().enumerate() {
        let out = out?;
        c.any_curves |= out.any_curves;
        c.tangent_points.extend(out.tangent);
        if out.coincident {
            if let Other::Face(b) = pairs[pi].b {
                c.coincident.push((pairs[pi].a, b));
            }
        }
        let base = key;
        let max_idx = out.next_idx;
        for pr in out.pieces {
            let k = base + pr.curve_idx;
            match share {
                Share::Shared => c.pieces.push(Piece { src: pr.src, key: k, range: pr.range, closed: pr.closed, faces: pr.faces, on_boundary: pr.on_boundary, group: 0, source_edge: pr.source_edge, pair: pi }),
                Share::Private => {
                    // One copy per face of the pair that needs it, each with its own curve.
                    for f in [pr.both.0, pr.both.1] {
                        let Some(f) = f else { continue };
                        let g = group_of(f);
                        let faces: SmallVec<[EntityId; 2]> = pr.faces.iter().copied().filter(|x| *x == f).collect();
                        if faces.is_empty() {
                            continue;
                        }
                        let on_boundary: SmallVec<[EntityId; 2]> = pr.on_boundary.iter().copied().filter(|x| *x == f).collect();
                        let src = match &pr.src {
                            CurveSrc::Existing(h) => view.shapes().curve_def(*h).map_or(CurveSrc::Existing(*h), |d| CurveSrc::New(d.clone())),
                            other => other.clone(),
                        };
                        c.pieces.push(Piece { src, key: k, range: pr.range, closed: pr.closed, faces, on_boundary, group: g, source_edge: None, pair: pi });
                    }
                }
            }
        }
        key = base + max_idx;
    }
    Ok(c)
}

fn unique_edges<V: TopologyView + ?Sized>(view: &V, f: EntityId) -> Vec<EntityId> {
    view.face_edges(f).iter().copied().filter(|&e| !geo::is_degenerate(view, e)).collect()
}

fn window_of<V: TopologyView + ?Sized>(view: &V, a: EntityId, b: &Other, pad: f64) -> [f64; 6] {
    let mut bx = geo::face_aabb(view, a, 0.0);
    if let Other::Face(bf) = b {
        bx = bx.union(&geo::face_aabb(view, *bf, 0.0));
    } else {
        // A bare tool surface: the window is the target face's box.
    }
    let diag = geo::dist(bx.lo, bx.hi);
    let bx = pad_box(bx, pad + 0.01 * diag);
    [bx.lo.x, bx.lo.y, bx.lo.z, bx.hi.x, bx.hi.y, bx.hi.z]
}

fn compute_pair<V: TopologyView + ?Sized>(view: &V, spec: &PairSpec, window_pad: f64) -> Result<PairOut, BooleanError> {
    let tol: ModelTolerance = *view.tolerance();
    let lin = tol.linear;
    let a = spec.a;
    let Some(sa) = surface_def_of(view, a) else { return Err(BooleanError::Unsupported("a face has no surface carrier")) };
    let (sb, bface): (SurfaceDef, Option<EntityId>) = match &spec.b {
        Other::Face(b) => match surface_def_of(view, *b) {
            Some(s) => (s.clone(), Some(*b)),
            None => return Err(BooleanError::Unsupported("a face has no surface carrier")),
        },
        Other::Surface(s) => (s.clone(), None),
    };
    let cache = faceclass::FaceCache::new();
    let window = window_of(view, a, &spec.b, window_pad);
    let opts = SsiOptions::new(tol).with_window(window).with_chord_tol(100.0 * lin);
    let bid = bface.unwrap_or(a);
    let res = intersect_surfaces(sa, &sb, &opts).map_err(|e| BooleanError::Ssi { a, b: bid, source: e })?;
    let mut out = PairOut { pieces: Vec::new(), coincident: false, tangent: Vec::new(), any_curves: false, next_idx: 0 };
    match res {
        SsiResult::Empty => {}
        SsiResult::Point(p) => out.tangent.push(p),
        SsiResult::Coincident => {
            out.coincident = true;
            if let Some(b) = bface {
                // Boundary of B inside A (target slot 0), boundary of A inside B (slot 1).
                coplanar_pieces(view, &cache, b, a, 0, lin, &mut out)?;
                coplanar_pieces(view, &cache, a, b, 1, lin, &mut out)?;
            }
        }
        SsiResult::Curves(curves) => {
            out.any_curves = !curves.is_empty();
            for (ci, ic) in curves.into_iter().enumerate() {
                let (def, range, closed) = match ic.geometry {
                    CurveGeometry::Analytic { curve, range } => (curve, range, ic.closed),
                    CurveGeometry::Marched(mc) => {
                        let nc = mc.to_nurbs().map_err(|e| BooleanError::Ssi { a, b: bid, source: e })?;
                        let closed = mc.closed;
                        let r = Interval::new(0.0, 1.0);
                        (CurveDef::Nurbs(nc), r, closed)
                    }
                    _ => return Err(BooleanError::Unsupported("unknown intersection curve kind")),
                };
                if !(range.lo.is_finite() && range.hi.is_finite()) {
                    return Err(BooleanError::Unsupported("an intersection curve is unbounded: the window did not clip it"));
                }
                clip_curve(view, &cache, a, bface, sa, &sb, &def, range, closed, ci, lin, &mut out)?;
                out.next_idx = out.next_idx.max(ci + 1);
            }
        }
        _ => return Err(BooleanError::Unsupported("unknown intersection result")),
    }
    Ok(out)
}

/// Breakpoints (curve parameter and point) where the boundary edges of `face` meet the surface
/// `other` and lie on the curve `c` within `reach`.
fn edge_breakpoints<V: TopologyView + ?Sized>(view: &V, face: EntityId, other: &SurfaceDef, c: &dyn sm_geom_api::ParametricCurve, reach: f64, tol: &ModelTolerance, out: &mut Vec<(f64, Point3)>) {
    for e in unique_edges(view, face) {
        let Carrier::Curve(cc) = view.carrier(e) else { continue };
        let Some(def) = view.shapes().curve_def(cc.curve) else { continue };
        let mut add = |p: Point3| {
            if let Ok(pr) = c.project(p, None)
                && pr.distance <= reach
            {
                out.push((pr.t, p));
            }
        };
        match curve_surface(def, Some(cc.range), other, tol) {
            Ok(CurveSurface::Points(hits)) => {
                for h in hits {
                    add(h.point);
                }
            }
            Ok(CurveSurface::Coincident) => {
                if let Some((s, t)) = view.edge_ends(e) {
                    for v in [s, t] {
                        if let Some(p) = geo::vertex_point(view, v) {
                            add(p);
                        }
                    }
                }
            }
            _ => {}
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn clip_curve<V: TopologyView + ?Sized>(
    view: &V,
    cache: &faceclass::FaceCache,
    a: EntityId,
    b: Option<EntityId>,
    sa: &SurfaceDef,
    sb: &SurfaceDef,
    def: &CurveDef,
    range: Interval,
    closed: bool,
    curve_idx: usize,
    lin: f64,
    out: &mut PairOut,
) -> Result<(), BooleanError> {
    let tol = *view.tolerance();
    let c = def.as_curve();
    let period = c.domain().period;
    let reach = 1e-3_f64.min(100.0 * lin).max(10.0 * lin);
    // Breakpoints: edges of A against surface B, edges of B against surface A.
    let mut bps: Vec<(f64, Point3)> = Vec::new();
    edge_breakpoints(view, a, sb, c, reach, &tol, &mut bps);
    if let Some(b) = b {
        edge_breakpoints(view, b, sa, c, reach, &tol, &mut bps);
    }
    // Normalize periodic parameters into [range.lo, range.lo + period).
    if let (Some(per), true) = (period, closed) {
        for bp in &mut bps {
            let mut t = bp.0;
            while t < range.lo {
                t += per;
            }
            while t >= range.lo + per {
                t -= per;
            }
            bp.0 = t;
        }
    } else {
        bps.retain(|bp| bp.0 >= range.lo - 1e-12 && bp.0 <= range.hi + 1e-12);
        for bp in &mut bps {
            bp.0 = bp.0.clamp(range.lo, range.hi);
        }
    }
    // A closed non-periodic curve (marched loop) gets its start as a breakpoint so that every
    // interval lies within the parameter range.
    if closed && period.is_none() {
        bps.push((range.lo, c.eval(range.lo)));
    }
    bps.sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut uniq: Vec<(f64, Point3)> = Vec::new();
    for bp in bps {
        if uniq.last().is_none_or(|l| geo::dist(l.1, bp.1) > lin) {
            uniq.push(bp);
        }
    }
    // Intervals.
    let mut intervals: Vec<(f64, f64, bool)> = Vec::new();
    match (closed, period) {
        (true, Some(per)) => {
            if uniq.is_empty() {
                intervals.push((range.lo, range.lo + per, true));
            } else if uniq.len() == 1 {
                intervals.push((uniq[0].0, uniq[0].0 + per, true));
            } else {
                for w in uniq.windows(2) {
                    intervals.push((w[0].0, w[1].0, false));
                }
                let (l, f) = (uniq[uniq.len() - 1].0, uniq[0].0);
                intervals.push((l, f + per, false));
            }
        }
        (true, None) => {
            // Start breakpoint at range.lo guarantees uniq[0] is the start; close at range.hi.
            for w in uniq.windows(2) {
                intervals.push((w[0].0, w[1].0, false));
            }
            if let Some(last) = uniq.last() {
                intervals.push((last.0, range.hi, false));
            }
        }
        (false, _) => {
            let mut prev = range.lo;
            for bp in &uniq {
                intervals.push((prev, bp.0, false));
                prev = bp.0;
            }
            intervals.push((prev, range.hi, false));
        }
    }
    for (t0, t1, whole) in intervals {
        if t1 - t0 <= 1e-14 * (1.0 + t0.abs()) {
            continue;
        }
        let mid = c.eval(0.5 * (t0 + t1));
        let ca = faceclass::classify(view, cache, a, mid, lin)?.0;
        let cb = match b {
            Some(b) => faceclass::classify(view, cache, b, mid, lin)?.0,
            None => PointClass::In,
        };
        if ca == PointClass::Out || cb == PointClass::Out {
            continue;
        }
        let mut faces: SmallVec<[EntityId; 2]> = SmallVec::new();
        let mut on_boundary: SmallVec<[EntityId; 2]> = SmallVec::new();
        if ca == PointClass::In {
            faces.push(a);
        } else {
            on_boundary.push(a);
        }
        if let Some(b) = b {
            if cb == PointClass::In {
                faces.push(b);
            } else {
                on_boundary.push(b);
            }
        }
        if faces.is_empty() && b.is_some() {
            // Runs along the boundary of both: nothing to insert.
            continue;
        }
        out.pieces.push(Proto { src: CurveSrc::New(def.clone()), curve_idx, range: Interval::new(t0, t1), closed: whole, faces, on_boundary, source_edge: None, both: (Some(a), b) });
    }
    Ok(())
}

/// Boundary edges of `src_face` clipped to the interior of `into` (both on one surface):
/// cut pieces for `into`.
fn coplanar_pieces<V: TopologyView + ?Sized>(view: &V, cache: &faceclass::FaceCache, src_face: EntityId, into: EntityId, slot: u8, lin: f64, out: &mut PairOut) -> Result<(), BooleanError> {
    let into_edges = unique_edges(view, into);
    for e in unique_edges(view, src_face) {
        let curve_idx = out.next_idx;
        out.next_idx += 1;
        let Carrier::Curve(cc) = view.carrier(e) else { continue };
        let Some(def) = view.shapes().curve_def(cc.curve) else { continue };
        let mut cuts: Vec<f64> = Vec::new();
        for ie in &into_edges {
            let Carrier::Curve(icc) = view.carrier(*ie) else { continue };
            let Some(idef) = view.shapes().curve_def(icc.curve) else { continue };
            let r = cc::curve_curve(def, cc.range, idef, icc.range, lin);
            cuts.extend(r.points.iter().map(|h| h.t1));
            for o in &r.overlaps {
                cuts.push(o.lo);
                cuts.push(o.hi);
            }
        }
        // Vertices of `into` lying on this edge are found as edge/edge contacts; add the ends.
        let mut ts: Vec<f64> = vec![cc.range.lo, cc.range.hi];
        ts.extend(cuts.into_iter().filter(|t| *t > cc.range.lo && *t < cc.range.hi));
        ts.sort_by(|x, y| x.total_cmp(y));
        ts.dedup_by(|x, y| geo::dist(def.as_curve().eval(*x), def.as_curve().eval(*y)) <= lin);
        for w in ts.windows(2) {
            let (t0, t1) = (w[0], w[1]);
            if t1 - t0 <= 1e-14 * (1.0 + t0.abs()) {
                continue;
            }
            let mid = def.as_curve().eval(0.5 * (t0 + t1));
            let class = faceclass::classify(view, cache, into, mid, lin)?.0;
            if class != PointClass::In {
                continue;
            }
            let whole = (t1 - t0 - (cc.range.hi - cc.range.lo)).abs() <= 1e-12 * (cc.range.hi - cc.range.lo).abs() && view.edge_ends(e).is_some_and(|(s, t)| s == t);
            out.pieces.push(Proto {
                src: CurveSrc::Existing(cc.curve),
                curve_idx,
                range: Interval::new(t0, t1),
                closed: whole,
                faces: SmallVec::from_slice(&[into]),
                on_boundary: SmallVec::new(),
                source_edge: Some(e),
                both: if slot == 0 { (Some(into), None) } else { (None, Some(into)) },
            });
        }
    }
    Ok(())
}
