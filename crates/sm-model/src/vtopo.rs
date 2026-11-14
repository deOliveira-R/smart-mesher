//! Virtual-topology ops on the layer's overlay (W4 VTOPO-OPS): composites, partitions, edge
//! collapse and realized decomposition splits.
//!
//! Every op goes through [`model_overlay_txn`], so the virtual ids an op mints are adopted in
//! the entity table and the ones it drops are retired. They record into THE layer's overlay
//! (`ModelLayer::overlay`), which is hashed, forked and replayed with the state. Meshing and
//! queries read the overlay through `topo()`; the real geometry is untouched (`host()`).
//!
//! A host edit that breaks the overlay is refused by `topo_txn` as
//! `LayerError::OverlayRebase(TopoError::Dead(id))` (the id names the dead cell); one that
//! does not touch it is rebased. `model.remove_virtual` restores the view exactly; the state
//! hash differs from the pre-op hash only by the overlay's id counters.

use std::collections::BTreeSet;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind, Vector3};
use sm_geom_api::{CurveDef, Interval, Line, SurfaceDef};
use sm_layer_api::{LayerResultExt, ModelLayer, model_overlay_txn};
use sm_model_api::{PieceRef, Plane};
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, TypedOp};
use sm_topo_api::{Carrier, Dim, Overlay, TopologyView, TopologyViewExt, VirtualCut, VirtualOp};

const FACE_ROLE: &[OutputRole] = &[OutputRole::new("face", &[EntityKind::PartitionFace], Cardinality::One, "The composite face.")];
const EDGE_ROLE: &[OutputRole] = &[OutputRole::new("edge", &[EntityKind::PartitionEdge], Cardinality::One, "The composite edge.")];
const SPLIT_EDGE_ROLES: &[OutputRole] = &[
    OutputRole::new("vertex", &[EntityKind::PartitionVertex], Cardinality::One, "The new virtual vertex."),
    OutputRole::new("edges", &[EntityKind::PartitionEdge], Cardinality::Many, "The two halves, in edge order."),
];
const SPLIT_FACE_ROLES: &[OutputRole] = &[
    OutputRole::new("chord", &[EntityKind::PartitionEdge], Cardinality::One, "The virtual chord."),
    OutputRole::new("faces", &[EntityKind::PartitionFace], Cardinality::Many, "The two child faces."),
];
const COLLAPSE_ROLES: &[OutputRole] = &[OutputRole::new("edges", &[EntityKind::PartitionEdge], Cardinality::Many, "Virtual copies of the edges that ended at the dropped vertex.")];
const PIECE_ROLES: &[OutputRole] = &[OutputRole::new("pieces", &[EntityKind::PartitionCell], Cardinality::Many, "The two pieces `[negative, positive]`.")];

fn kind_is(id: EntityId, kinds: &[EntityKind], what: &str) -> Result<(), OpError> {
    if kinds.contains(&id.kind()) {
        Ok(())
    } else {
        Err(OpError::Invalid(format!("{what}: {id} is a `{}`, expected one of {:?}", id.kind().name(), kinds.iter().map(|k| k.name()).collect::<Vec<_>>())))
    }
}

/// `model.composite_faces {faces}`: merge host faces into one virtual face (a `PartitionFace`).
///
/// The faces must be live host faces, connected through shared edges, with identical parents
/// (same volumes, sides and shells); their interior edges disappear from the overlay view. The
/// host is untouched. Refused with a located error otherwise (see `Overlay::composite_faces`).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CompositeFaces {
    /// The host faces to merge (at least two).
    #[schemars(extend("x-entity-kind" = ["TopoFace"]))]
    pub faces: Vec<EntityId>,
}

impl TypedOp for CompositeFaces {
    const NAME: &'static str = "model.composite_faces";
    const OUTPUTS: &'static [OutputRole] = FACE_ROLE;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        for &f in &self.faces {
            kind_is(f, &[EntityKind::TopoFace], "faces")?;
        }
        model_overlay_txn(ctx, |_, ov| ov.composite_faces(&self.faces).or_op(), |f, _| vec![("face", vec![*f])]).map(|_| ())
    }
}

/// `model.composite_edges {edges}`: merge a chain of host edges, joined at degree-2 vertices,
/// into one virtual edge (a `PartitionEdge`); the inner vertices disappear from the view.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CompositeEdges {
    /// The host edges to chain (at least two).
    #[schemars(extend("x-entity-kind" = ["TopoEdge"]))]
    pub edges: Vec<EntityId>,
}

impl TypedOp for CompositeEdges {
    const NAME: &'static str = "model.composite_edges";
    const OUTPUTS: &'static [OutputRole] = EDGE_ROLE;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        for &e in &self.edges {
            kind_is(e, &[EntityKind::TopoEdge], "edges")?;
        }
        model_overlay_txn(ctx, |_, ov| ov.composite_edges(&self.edges).or_op(), |e, _| vec![("edge", vec![*e])]).map(|_| ())
    }
}

/// `model.partition_edge {edge, param}`: add a virtual vertex on `edge` at curve parameter
/// `param`, replacing the edge by two virtual halves in every face that uses it. The halves'
/// lengths sum to the edge's length and the vertex lies on the curve.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PartitionEdge {
    /// The edge to split (host or virtual).
    #[schemars(extend("x-entity-kind" = ["TopoEdge", "PartitionEdge"]))]
    pub edge: EntityId,
    /// Curve parameter of the new vertex, strictly inside the edge's range.
    pub param: f64,
}

impl TypedOp for PartitionEdge {
    const NAME: &'static str = "model.partition_edge";
    const OUTPUTS: &'static [OutputRole] = SPLIT_EDGE_ROLES;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        kind_is(self.edge, &[EntityKind::TopoEdge, EntityKind::PartitionEdge], "edge")?;
        if !self.param.is_finite() {
            return Err(OpError::Invalid("`param` must be finite".into()));
        }
        model_overlay_txn(
            ctx,
            |_, ov| ov.partition_edge(self.edge, self.param).or_op(),
            |(v, h), _| vec![("vertex", vec![*v]), ("edges", h.to_vec())],
        )
        .map(|_| ())
    }
}

/// `model.partition_face {face, from, to}`: split a planar face by a straight virtual chord
/// between two vertices of its outer loop, into two child faces `[first, second]`.
///
/// Only faces on a plane are accepted: a chord on a curved face would need pcurves that
/// imported faces lack (vtopo risk R3), so a periodic or curved face is refused with the
/// face's id. A chord that leaves the face, touches or crosses its boundary away from its two
/// end vertices, or runs along an existing edge is refused with the face's id too.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PartitionFace {
    /// The face to split (host or virtual, one loop).
    #[schemars(extend("x-entity-kind" = ["TopoFace", "PartitionFace"]))]
    pub face: EntityId,
    /// Chord start: a vertex on the face's outer loop (host or virtual).
    #[schemars(extend("x-entity-kind" = ["TopoVertex", "PartitionVertex"]))]
    pub from: EntityId,
    /// Chord end: another vertex on the outer loop.
    #[schemars(extend("x-entity-kind" = ["TopoVertex", "PartitionVertex"]))]
    pub to: EntityId,
}

impl TypedOp for PartitionFace {
    const NAME: &'static str = "model.partition_face";
    const OUTPUTS: &'static [OutputRole] = SPLIT_FACE_ROLES;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        kind_is(self.face, &[EntityKind::TopoFace, EntityKind::PartitionFace], "face")?;
        kind_is(self.from, &[EntityKind::TopoVertex, EntityKind::PartitionVertex], "from")?;
        kind_is(self.to, &[EntityKind::TopoVertex, EntityKind::PartitionVertex], "to")?;
        model_overlay_txn(
            ctx,
            |_, ov| {
                let cut = chord(ov, self.face, self.from, self.to)?;
                let before: BTreeSet<EntityId> = ov.cells(Dim::Edge).iter().copied().collect();
                let halves = ov.partition_face(self.face, cut).or_op()?;
                // The chord is the one new edge shared by the two halves.
                let chord = ov.cells(Dim::Edge).iter().copied().find(|e| !before.contains(e) && e.kind() == EntityKind::PartitionEdge);
                Ok((chord, halves))
            },
            |(c, h), _| {
                let mut roles = vec![("faces", h.to_vec())];
                if let Some(c) = c {
                    roles.push(("chord", vec![*c]));
                }
                roles
            },
        )
        .map(|_| ())
    }
}

/// Collects the plane of `face`: a plane surface, or every member of a composite (recursively).
/// Any other carrier is refused with the id of `top`, the face the op was asked to cut.
///
/// The overlay view reports no carrier for a hidden cell, so a composite's members are read
/// from the host. A hidden virtual member (a composite of a partition child) has no carrier
/// to read and is refused.
fn face_planes(ov: &Overlay, top: EntityId, face: EntityId, out: &mut Vec<sm_geom_api::Plane>) -> Result<(), OpError> {
    let view: &dyn TopologyView = ov;
    let carrier = match view.carrier(face) {
        Carrier::None => ov.host().carrier(face),
        c => c,
    };
    match carrier {
        Carrier::Surface(sc) => match view.shapes().surface_def(sc.surface) {
            Some(SurfaceDef::Plane(p)) => {
                out.push(*p);
                Ok(())
            }
            _ => Err(OpError::Invalid(format!("face {top} is not planar: a virtual chord on a curved face needs pcurves (not available yet)"))),
        },
        Carrier::Composite(members) if !members.is_empty() => members.iter().try_for_each(|&m| face_planes(ov, top, m, out)),
        _ => Err(OpError::Invalid(format!("face {top} is not planar: it has no plane carrier"))),
    }
}

/// Polyline samples per edge when the loop is flattened for the chord checks.
const LOOP_SAMPLES: usize = 32;

/// Distance from `p` to the segment `a b`.
fn dist_pt_seg(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (d, w) = ([b[0] - a[0], b[1] - a[1]], [p[0] - a[0], p[1] - a[1]]);
    let l2 = d[0] * d[0] + d[1] * d[1];
    let t = if l2 > 0.0 { ((w[0] * d[0] + w[1] * d[1]) / l2).clamp(0.0, 1.0) } else { 0.0 };
    ((w[0] - t * d[0]).powi(2) + (w[1] - t * d[1]).powi(2)).sqrt()
}

/// Whether the open segments `a b` and `c d` properly cross.
fn crosses(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
    let o = |p: [f64; 2], q: [f64; 2], r: [f64; 2]| (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
    o(a, b, c) * o(a, b, d) < 0.0 && o(c, d, a) * o(c, d, b) < 0.0
}

/// Minimum distance between segments `a b` and `c d`.
fn dist_seg_seg(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> f64 {
    if crosses(a, b, c, d) {
        return 0.0;
    }
    dist_pt_seg(a, c, d).min(dist_pt_seg(b, c, d)).min(dist_pt_seg(c, a, b)).min(dist_pt_seg(d, a, b))
}

/// Refuses (with the face id) a chord that is not a proper interior chord of the face: one that
/// touches or crosses the loops anywhere but at its two end vertices, runs along an existing
/// edge, or whose midpoint is not strictly inside the face. The loops are flattened in the
/// plane's frame (edge samplers, so virtual and curved edges count).
fn check_chord_inside(view: &dyn TopologyView, face: EntityId, frame: &sm_geom_api::Frame, a3: sm_core::Point3, b3: sm_core::Point3) -> Result<(), OpError> {
    let tol = view.tolerance().linear;
    let proj = |p: sm_core::Point3| {
        let w = p - frame.origin;
        [w.dot(frame.x), w.dot(frame.y)]
    };
    let (a, b) = (proj(a3), proj(b3));
    let mut segs: Vec<([f64; 2], [f64; 2])> = Vec::new();
    for l in view.loops(face) {
        for e in l.entries {
            let Some(sm) = view.edge_sampler(e.cell) else { continue };
            let pts: Vec<[f64; 2]> = (0..=LOOP_SAMPLES).map(|i| proj(sm.eval(i as f64 / LOOP_SAMPLES as f64))).collect();
            segs.extend(pts.windows(2).map(|w| (w[0], w[1])));
        }
    }
    let bad = |why: &str| Err(OpError::Invalid(format!("face {face}: the chord {why}")));
    let near = |p: [f64; 2], q: [f64; 2]| ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2)).sqrt() <= tol;
    for &(p, q) in &segs {
        let at_end = near(p, a) || near(q, a) || near(p, b) || near(q, b);
        if at_end {
            // Touching at an end vertex is fine; running along the segment is not.
            let other = |v: [f64; 2]| if near(p, v) { q } else { p };
            let ends = [(a, b), (b, a)];
            for (v, w) in ends {
                if near(p, v) || near(q, v) {
                    let far = other(v);
                    if dist_pt_seg(far, a, b) <= tol || dist_pt_seg(w, p, q) <= tol {
                        return bad("runs along an existing edge of the face");
                    }
                }
            }
        } else if dist_seg_seg(a, b, p, q) <= tol {
            return bad("crosses or touches the face boundary away from its end vertices");
        }
    }
    // Midpoint strictly inside (even-odd over every loop).
    let m = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
    if segs.iter().any(|&(p, q)| dist_pt_seg(m, p, q) <= tol) {
        return bad("lies on the face boundary");
    }
    let inside = segs.iter().fold(false, |acc, &(p, q)| if (p[1] > m[1]) != (q[1] > m[1]) && m[0] < p[0] + (m[1] - p[1]) / (q[1] - p[1]) * (q[0] - p[0]) { !acc } else { acc });
    if !inside {
        return bad("leaves the face (its midpoint is outside)");
    }
    Ok(())
}

/// The straight chord `from -> to` as a [`VirtualCut`]; refuses a non-planar face or a vertex
/// without a point.
fn chord(ov: &Overlay, face: EntityId, from: EntityId, to: EntityId) -> Result<VirtualCut, OpError> {
    let view: &dyn TopologyView = ov;
    if !view.is_alive(face) {
        return Err(OpError::Invalid(format!("face {face} is not live in the overlay view")));
    }
    let mut planes = Vec::new();
    face_planes(ov, face, face, &mut planes)?;
    let tol = view.tolerance();
    let r = planes[0];
    for p in &planes[1..] {
        let tilt = r.frame.z.cross(p.frame.z).norm();
        let off = (p.frame.origin - r.frame.origin).dot(r.frame.z).abs();
        if tilt > tol.angular || off > tol.linear {
            return Err(OpError::Invalid(format!("face {face} is not planar: its members do not lie on one plane (a virtual chord needs pcurves on a bent face)")));
        }
    }
    let pt = |v: EntityId| match view.is_alive(v).then(|| view.carrier(v)) {
        Some(Carrier::Point(p)) => Ok(*p),
        _ => Err(OpError::Invalid(format!("vertex {v} is not a live vertex with a point"))),
    };
    let (a, b) = (pt(from)?, pt(to)?);
    let d: Vector3 = b - a;
    let len = d.norm();
    check_chord_inside(view, face, &r.frame, a, b)?;
    let line = Line::new(a, d).map_err(|e| OpError::Invalid(format!("chord {from} -> {to}: {e}")))?;
    Ok(VirtualCut { from, to, curve: CurveDef::Line(line), range: Interval::new(0.0, len) })
}

/// `model.collapse_edge {edge, into_vertex}`: collapse a short edge virtually. It disappears
/// from the view and its end vertices become one (`into_vertex`); other edges at the dropped
/// vertex get virtual copies. No host entity moves. The collapse gap (largest curve-end move)
/// is reported by `model.virtual_info` and is refused above the tolerance's `linear_max`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CollapseEdge {
    /// The edge to collapse (host or virtual).
    #[schemars(extend("x-entity-kind" = ["TopoEdge", "PartitionEdge"]))]
    pub edge: EntityId,
    /// The surviving end vertex of `edge`.
    #[schemars(extend("x-entity-kind" = ["TopoVertex", "PartitionVertex"]))]
    pub into_vertex: EntityId,
}

impl TypedOp for CollapseEdge {
    const NAME: &'static str = "model.collapse_edge";
    const OUTPUTS: &'static [OutputRole] = COLLAPSE_ROLES;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        kind_is(self.edge, &[EntityKind::TopoEdge, EntityKind::PartitionEdge], "edge")?;
        kind_is(self.into_vertex, &[EntityKind::TopoVertex, EntityKind::PartitionVertex], "into_vertex")?;
        model_overlay_txn(
            ctx,
            |_, ov| ov.collapse_edge(self.edge, self.into_vertex).or_op(),
            |c, _| {
                let copies: Vec<EntityId> = c.repointed.iter().map(|&(_, n)| n).collect();
                if copies.is_empty() { vec![] } else { vec![("edges", copies)] }
            },
        )
        .map(|_| ())
    }
}

/// `model.remove_virtual {id}`: undo the virtual operation that created `id` (any cell it
/// created; for a collapse also the collapsed edge). The hidden cells return and the virtual
/// ids it minted are retired and never reused. Refused (`StillReferenced`) while a later
/// virtual op uses one of its cells: remove those first.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RemoveVirtual {
    /// A virtual cell of the op to undo.
    #[schemars(extend("x-entity-kind" = ["PartitionVertex", "PartitionEdge", "PartitionFace", "TopoEdge"]))]
    pub id: EntityId,
}

impl TypedOp for RemoveVirtual {
    const NAME: &'static str = "model.remove_virtual";
    const NO_OUTPUTS: bool = true;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let known = ctx
            .layer::<ModelLayer>()?
            .overlay()
            .is_some_and(|o| o.ops().iter().any(|op| op.created().contains(&self.id) || matches!(op, VirtualOp::CollapseEdge { edge, .. } if *edge == self.id)));
        if !known {
            return Err(OpError::Invalid(format!("{} is not part of any virtual operation of the overlay", self.id)));
        }
        model_overlay_txn(ctx, |_, ov| ov.remove(self.id).or_op(), |_, _| vec![])
    }
}

fn piece_of(id: EntityId) -> Result<PieceRef, OpError> {
    match id.kind() {
        EntityKind::TopoVolume => Ok(PieceRef::Whole(id)),
        EntityKind::PartitionCell => Ok(PieceRef::Cell(id)),
        k => Err(OpError::Invalid(format!("piece: {id} is a `{}`, expected a TopoVolume or PartitionCell", k.name()))),
    }
}

/// `model.split_realized {piece, normal, offset}`: split a decomposition piece by the plane
/// `normal . x = offset` and realize the cut in the layer's overlay, so the persisted cut cell
/// ids resolve in the replayed view. Returns the two pieces `[negative, positive]`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SplitRealized {
    /// A volume (whole piece) or a `PartitionCell` made by an earlier realized split.
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "PartitionCell"]))]
    pub piece: EntityId,
    /// Plane normal (any non-zero vector, normalized for you).
    pub normal: [f64; 3],
    /// Plane offset along the normal (rescaled with it).
    pub offset: f64,
}

impl TypedOp for SplitRealized {
    const NAME: &'static str = "model.split_realized";
    const OUTPUTS: &'static [OutputRole] = PIECE_ROLES;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let piece = piece_of(self.piece)?;
        if !self.normal.iter().chain([&self.offset]).all(|x| x.is_finite()) {
            return Err(OpError::Invalid("`normal` and `offset` must be finite".into()));
        }
        let plane = Plane::new(Vector3::new(self.normal[0], self.normal[1], self.normal[2]), self.offset).or_op()?;
        model_overlay_txn(ctx, |m, ov| m.split_realized(piece, plane, ov).or_op(), |kids, _| vec![("pieces", kids.to_vec())]).map(|_| ())
    }
}

/// `model.unsplit_realized {piece}`: undo a realized split from either of its two pieces (both
/// must be unsplit leaves). The overlay cells of the cut are removed and retired.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UnsplitRealized {
    /// One of the two pieces of the split.
    #[schemars(extend("x-entity-kind" = ["PartitionCell"]))]
    pub piece: EntityId,
}

impl TypedOp for UnsplitRealized {
    const NAME: &'static str = "model.unsplit_realized";
    const NO_OUTPUTS: bool = true;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        kind_is(self.piece, &[EntityKind::PartitionCell], "piece")?;
        model_overlay_txn(ctx, |m, ov| m.unsplit_realized(self.piece, ov).or_op(), |_, _| vec![]).map(|_| ())
    }
}
