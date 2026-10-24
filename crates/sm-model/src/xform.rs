//! Similarity transforms of whole volumes: `model.move`, `model.rotate`, `model.scale`,
//! `model.reflect`.
//!
//! All four share [`apply_similarity`]. The selection (volumes, or bodies standing for their
//! members) is closed downwards; every cell of the closure moves, so a closure cell that an
//! unselected volume also uses (a face shared with a neighbour) refuses the op instead of tearing
//! the model. Inside one topology transaction it
//!
//! 1. moves vertices (`set_vertex_point`);
//! 2. replaces the curve and surface definitions with `CurveDef::transformed` /
//!    `SurfaceDef::transformed`, scaling the parameter range of line edges (a line's parameter
//!    is arc length) and mapping every `Line2`/`Polyline2` pcurve through the surface's
//!    parameter change (scale of `u`/`v`, `u -> 2 pi - u` under a reflection of a periodic
//!    angle);
//! 3. for an improper map, re-orients each face (see [`reorient`]);
//! 4. scales per-entity tolerances (`scale_tolerances`);
//! 5. checks that every pcurve still lies on its surface and curve;
//! 6. carries the model tables through (`Model::apply_transform`: decks, cut planes, symmetry).
//!
//! Meshed volumes are refused (decision W2-2: `mesh.delete` first).

use std::collections::{BTreeMap, BTreeSet};

use core::f64::consts::TAU;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind, Point3, Transform, Vector3};
use sm_geom_api::{CurveDef, Interval, Line2, PCurve, SurfaceDef, Uv};
use sm_layer_api::{LayerResultExt, MeshLayer, ModelLayer, TopoHost, commit_topology, topo_txn};
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, TypedOp};
use sm_topo_api::{
    Carrier, CellFlags, CoedgeKey, CurveCarrier, Dim, FaceUseSpec, Side, SurfaceCarrier, TopologyViewExt,
};

const OUTPUTS: &[OutputRole] = &[OutputRole::new("volumes", &[EntityKind::TopoVolume], Cardinality::Many, "The transformed volumes (they keep their ids).")];

/// Why a transform could not be applied.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum XformError {
    /// A parameter is not finite or out of range.
    #[error("{0}")]
    BadParam(String),
    /// The selection is empty.
    #[error("`entities` is empty")]
    EmptySelection,
    /// A selected volume (or one of its faces) is meshed.
    #[error("{0} is meshed by {1}; mesh.delete first")]
    Meshed(EntityId, EntityId),
    /// A cell of the selection is also used by something outside it.
    #[error("{cell} is shared with {other}, which is not part of the selection")]
    Shared {
        /// The shared cell.
        cell: EntityId,
        /// The unselected user.
        other: EntityId,
    },
    /// A curve or surface handle is used by cells outside the selection.
    #[error("shape {0} is shared with cells outside the selection")]
    SharedShape(EntityId),
    /// A shape or pcurve kind this transform cannot carry exactly.
    #[error("{0}")]
    Unsupported(String),
    /// A pcurve does not lie on its surface after the transform (an internal consistency check).
    #[error("pcurve of {key:?} is {gap:e} away from its edge after the transform")]
    PcurveDrift {
        /// The offending coedge.
        key: CoedgeKey,
        /// The measured distance.
        gap: f64,
    },
}

impl From<XformError> for OpError {
    fn from(e: XformError) -> Self {
        match e {
            XformError::BadParam(_) | XformError::EmptySelection | XformError::Meshed(..) | XformError::Shared { .. } | XformError::SharedShape(_) => OpError::Invalid(e.to_string()),
            _ => OpError::Failed(e.to_string()),
        }
    }
}

/// An affine map that is a similarity: `scale` is the uniform factor (1 for rigid maps).
#[derive(Clone, Copy, Debug)]
struct Xf {
    t: Transform,
    scale: f64,
}

impl Xf {
    fn improper(&self) -> bool {
        self.t.is_improper()
    }
}

fn finite_vec(name: &str, v: [f64; 3]) -> Result<Vector3, XformError> {
    if v.iter().all(|c| c.is_finite()) { Ok(Vector3::new(v[0], v[1], v[2])) } else { Err(XformError::BadParam(format!("`{name}` must be finite, got {v:?}"))) }
}

fn nonzero_vec(name: &str, v: [f64; 3]) -> Result<Vector3, XformError> {
    let w = finite_vec(name, v)?;
    if w.norm() > 0.0 { Ok(w) } else { Err(XformError::BadParam(format!("`{name}` must be non-zero"))) }
}

fn origin_of(o: Option<[f64; 3]>) -> Result<Point3, XformError> {
    let v = finite_vec("origin", o.unwrap_or([0.0; 3]))?;
    Ok(Point3::new(v.x, v.y, v.z))
}

/// `model.move`: translate volumes by `delta`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Move {
    /// Volumes, or bodies standing for their members.
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoBody"]))]
    pub entities: Vec<EntityId>,
    /// Translation vector.
    pub delta: [f64; 3],
}

impl TypedOp for Move {
    const NAME: &'static str = "model.move";
    const OUTPUTS: &'static [OutputRole] = OUTPUTS;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let d = finite_vec("delta", self.delta)?;
        apply_similarity(ctx, &self.entities, Xf { t: Transform::translation(d), scale: 1.0 })
    }
}

/// `model.rotate`: rotate volumes by `angle_deg` degrees (right-hand rule) about the axis
/// through `origin` (default the origin) with direction `axis`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Rotate {
    /// Volumes, or bodies standing for their members.
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoBody"]))]
    pub entities: Vec<EntityId>,
    /// Rotation axis direction (not necessarily unit).
    pub axis: [f64; 3],
    /// Angle in degrees.
    pub angle_deg: f64,
    /// A point on the axis (default the origin).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub origin: Option<[f64; 3]>,
}

impl TypedOp for Rotate {
    const NAME: &'static str = "model.rotate";
    const OUTPUTS: &'static [OutputRole] = OUTPUTS;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let axis = nonzero_vec("axis", self.axis)?;
        if !self.angle_deg.is_finite() {
            return Err(XformError::BadParam(format!("`angle_deg` must be finite, got {}", self.angle_deg)).into());
        }
        let o = origin_of(self.origin)?;
        let t = Transform::rotation(o, axis, self.angle_deg.to_radians()).ok_or_else(|| XformError::BadParam("`axis` must be non-zero".into()))?;
        apply_similarity(ctx, &self.entities, Xf { t, scale: 1.0 })
    }
}

/// `model.scale`: uniform scale of volumes by `factor` about `origin` (default the origin).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Scale {
    /// Volumes, or bodies standing for their members.
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoBody"]))]
    pub entities: Vec<EntityId>,
    /// Scale factor (`> 0`).
    pub factor: f64,
    /// Fixed point (default the origin).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub origin: Option<[f64; 3]>,
}

impl TypedOp for Scale {
    const NAME: &'static str = "model.scale";
    const OUTPUTS: &'static [OutputRole] = OUTPUTS;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let f = self.factor;
        if !(f.is_finite() && f > 0.0) {
            return Err(XformError::BadParam(format!("`factor` must be finite and positive, got {f}")).into());
        }
        let o = origin_of(self.origin)?;
        let ov = Vector3::new(o.x, o.y, o.z);
        let t = Transform { m: [[f, 0.0, 0.0], [0.0, f, 0.0], [0.0, 0.0, f]], t: ov * (1.0 - f) };
        apply_similarity(ctx, &self.entities, Xf { t, scale: f })
    }
}

/// `model.reflect`: mirror volumes in the plane through `origin` (default the origin) with
/// normal `normal`. Face orientation is repaired, so the volumes keep a positive volume and
/// outward normals.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Reflect {
    /// Volumes, or bodies standing for their members.
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoBody"]))]
    pub entities: Vec<EntityId>,
    /// Plane normal (not necessarily unit).
    pub normal: [f64; 3],
    /// A point on the plane (default the origin).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub origin: Option<[f64; 3]>,
}

impl TypedOp for Reflect {
    const NAME: &'static str = "model.reflect";
    const OUTPUTS: &'static [OutputRole] = OUTPUTS;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let n = nonzero_vec("normal", self.normal)?;
        let o = origin_of(self.origin)?;
        // `Transform::reflection(n, d)` takes the offset in the scale of the unnormalized `n`.
        let d = n.dot(Vector3::new(o.x, o.y, o.z));
        let t = Transform::reflection(n, d).ok_or_else(|| XformError::BadParam("`normal` must be non-zero".into()))?;
        apply_similarity(ctx, &self.entities, Xf { t, scale: 1.0 })
    }
}

/// The resolved selection: the volumes (ascending) and the downward closure (ascending, all
/// ranks, volumes included).
struct Selection {
    volumes: Vec<EntityId>,
    closure: Vec<EntityId>,
}

fn select(ctx: &ApplyCtx<'_>, entities: &[EntityId]) -> Result<Selection, OpError> {
    if entities.is_empty() {
        return Err(XformError::EmptySelection.into());
    }
    let topo = ctx.layer::<ModelLayer>()?.topo();
    let mut seeds: BTreeSet<EntityId> = BTreeSet::new();
    for &e in entities {
        ctx.require(e)?;
        match e.kind() {
            EntityKind::TopoVolume => {
                seeds.insert(e);
            }
            EntityKind::TopoBody => seeds.extend(topo.boundary(e).iter().map(|i| i.cell)),
            _ => return Err(OpError::WrongKind { id: e, expected: EntityKind::TopoVolume }),
        }
    }
    let mut closure: BTreeSet<EntityId> = BTreeSet::new();
    let mut stack: Vec<EntityId> = seeds.into_iter().collect();
    while let Some(c) = stack.pop() {
        if !closure.insert(c) {
            continue;
        }
        if c.kind() != EntityKind::TopoVertex {
            stack.extend(topo.boundary(c).iter().map(|i| i.cell));
        }
    }
    for &c in &closure {
        for p in topo.coboundary(c) {
            if p.cell.kind() != EntityKind::TopoBody && !closure.contains(&p.cell) {
                return Err(XformError::Shared { cell: c, other: p.cell }.into());
            }
        }
    }
    let volumes: Vec<EntityId> = closure.iter().copied().filter(|c| c.kind() == EntityKind::TopoVolume).collect();
    // Meshed volumes (or faces) are refused; a missing `meshes` layer is tolerated.
    match ctx.layer::<MeshLayer>() {
        Err(OpError::MissingLayer(_)) => {}
        Err(e) => return Err(e),
        Ok(layer) => {
            for &c in closure.iter().filter(|c| matches!(c.kind(), EntityKind::TopoVolume | EntityKind::TopoFace)) {
                if let Some(m) = layer.mesh_of(c) {
                    return Err(XformError::Meshed(c, m).into());
                }
            }
        }
    }
    Ok(Selection { volumes, closure: closure.into_iter().collect() })
}

fn apply_similarity(ctx: &mut ApplyCtx<'_>, entities: &[EntityId], xf: Xf) -> Result<(), OpError> {
    let sel = select(ctx, entities)?;
    let ((), txn) = topo_txn(ctx, |topo, model| {
        transform_cells(topo, &sel.closure, &xf)?;
        model.apply_transform(&xf.t, &sel.volumes, topo.as_view()).or_op()
    })?;
    commit_topology(ctx, &txn, &[("volumes", &sel.volumes)])
}

/// How a surface's `(u, v)` change under the transform: `u' = au * u + bu`, `v' = av * v`.
#[derive(Clone, Copy, Debug)]
struct UvMap {
    au: f64,
    bu: f64,
    av: f64,
}

impl UvMap {
    const IDENTITY: UvMap = UvMap { au: 1.0, bu: 0.0, av: 1.0 };

    fn is_identity(&self) -> bool {
        self.au == 1.0 && self.bu == 0.0 && self.av == 1.0
    }

    fn point(&self, p: Uv) -> Uv {
        Uv::new(self.au * p.u + self.bu, self.av * p.v)
    }
}

/// The parameter change of the image surface, or `None` where the kind is not covered.
fn uv_map(old: &SurfaceDef, xf: &Xf) -> Option<UvMap> {
    let (s, improper) = (xf.scale, xf.improper());
    // Under a reflection the angle of a surface of revolution runs backwards: `u' = 2 pi - u`.
    let angle = if improper { (-1.0, TAU) } else { (1.0, 0.0) };
    match old {
        SurfaceDef::Plane(_) => Some(UvMap { au: s, bu: 0.0, av: s }),
        SurfaceDef::Cylinder(_) => Some(UvMap { au: angle.0, bu: angle.1, av: s }),
        SurfaceDef::Sphere(_) => Some(UvMap { au: angle.0, bu: angle.1, av: 1.0 }),
        SurfaceDef::Nurbs(_) => Some(UvMap::IDENTITY),
        _ => None,
    }
}

fn map_pcurve(pc: &PCurve, m: &UvMap, c: f64) -> Result<PCurve, XformError> {
    Ok(match pc {
        PCurve::Line2(l) => {
            let o = m.point(l.origin);
            PCurve::Line2(Line2 { origin: o, dir: Uv::new(m.au * l.dir.u / c, m.av * l.dir.v / c), range: Interval::new(l.range.lo * c, l.range.hi * c) })
        }
        PCurve::Polyline2(pts) => PCurve::Polyline2(pts.iter().map(|&p| m.point(p)).collect()),
        PCurve::Nurbs2(_) if m.is_identity() && c == 1.0 => pc.clone(),
        _ => return Err(XformError::Unsupported("a NURBS pcurve cannot be carried through this transform".into())),
    })
}

fn transform_cells(topo: &mut dyn TopoHost, closure: &[EntityId], xf: &Xf) -> Result<(), OpError> {
    let inside: BTreeSet<EntityId> = closure.iter().copied().collect();
    let of_kind = |k: EntityKind| closure.iter().copied().filter(move |c| c.kind() == k);
    let vertices: Vec<EntityId> = of_kind(EntityKind::TopoVertex).collect();
    let edges: Vec<EntityId> = of_kind(EntityKind::TopoEdge).collect();
    let faces: Vec<EntityId> = of_kind(EntityKind::TopoFace).collect();

    // Shape handles: every user must be inside the selection.
    let mut curve_handles: BTreeMap<EntityId, bool> = BTreeMap::new();
    for &e in topo.cells(Dim::Edge) {
        if let Carrier::Curve(cc) = topo.carrier(e) {
            let ok = inside.contains(&e);
            curve_handles.entry(cc.curve).and_modify(|a| *a &= ok).or_insert(ok);
        }
    }
    let mut surface_handles: BTreeMap<EntityId, bool> = BTreeMap::new();
    for &f in topo.cells(Dim::Face) {
        if let Carrier::Surface(sc) = topo.carrier(f) {
            let ok = inside.contains(&f);
            surface_handles.entry(sc.surface).and_modify(|a| *a &= ok).or_insert(ok);
        }
    }
    let mut curves: BTreeSet<EntityId> = BTreeSet::new();
    for &e in &edges {
        if let Carrier::Curve(cc) = topo.carrier(e) {
            if curve_handles.get(&cc.curve) == Some(&false) {
                return Err(XformError::SharedShape(cc.curve).into());
            }
            curves.insert(cc.curve);
        }
    }
    let mut surfaces: BTreeSet<EntityId> = BTreeSet::new();
    for &f in &faces {
        if let Carrier::Surface(sc) = topo.carrier(f) {
            if surface_handles.get(&sc.surface) == Some(&false) {
                return Err(XformError::SharedShape(sc.surface).into());
            }
            surfaces.insert(sc.surface);
        }
    }

    // Old definitions, before anything is replaced.
    let old_curves: BTreeMap<EntityId, CurveDef> = curves.iter().filter_map(|&h| topo.shapes().curve_def(h).map(|d| (h, d.clone()))).collect();
    let old_surfaces: BTreeMap<EntityId, SurfaceDef> = surfaces.iter().filter_map(|&h| topo.shapes().surface_def(h).map(|d| (h, d.clone()))).collect();
    // Parameter factor of an edge's curve: a line's parameter is arc length.
    let curve_factor = |h: EntityId| if matches!(old_curves.get(&h), Some(CurveDef::Line(_))) { xf.scale } else { 1.0 };

    // 1. vertices
    for &v in &vertices {
        if let Carrier::Point(p) = topo.carrier(v).clone() {
            topo.set_vertex_point(v, xf.t.apply_point(p)).or_op()?;
        }
    }

    // 2. curves, then the edge ranges
    for (&h, def) in &old_curves {
        let new = def.transformed(&xf.t).map_err(|e| OpError::Failed(format!("curve {h}: {e}")))?;
        topo.replace_curve(h, new).or_op()?;
    }
    if xf.scale != 1.0 {
        for &e in &edges {
            if let Carrier::Curve(cc) = topo.carrier(e).clone() {
                let c = curve_factor(cc.curve);
                if c != 1.0 {
                    let range = Interval::new(cc.range.lo * c, cc.range.hi * c);
                    topo.set_carrier(e, Carrier::Curve(CurveCarrier { range, ..cc })).or_op()?;
                }
            }
        }
    }

    // 3. surfaces; remember the sense change of each handle
    let mut sense_change: BTreeMap<EntityId, sm_geom_api::Sense> = BTreeMap::new();
    for (&h, def) in &old_surfaces {
        let (new, sense) = def.transformed(&xf.t).map_err(|e| OpError::Failed(format!("surface {h}: {e}")))?;
        topo.replace_surface(h, new).or_op()?;
        sense_change.insert(h, sense);
    }

    // 4. pcurves (under their pre-reorientation keys)
    for &f in &faces {
        let Carrier::Surface(sc) = topo.carrier(f).clone() else { continue };
        let Some(old) = old_surfaces.get(&sc.surface) else { continue };
        let keys: Vec<CoedgeKey> = topo.coedges(f).map(|c| c.key).collect();
        for key in keys {
            let Some(pc) = topo.pcurve(key).cloned() else { continue };
            let c = match topo.carrier(key.edge) {
                Carrier::Curve(cc) => curve_factor(cc.curve),
                _ => 1.0,
            };
            let m = uv_map(old, xf).ok_or_else(|| XformError::Unsupported(format!("pcurves on a {} surface cannot be carried through this transform", surface_kind(old))))?;
            if m.is_identity() && c == 1.0 {
                continue;
            }
            let new = map_pcurve(&pc, &m, c)?;
            topo.set_pcurve(key, Some(new)).or_op()?;
        }
    }

    // 5. orientation under an improper map
    if xf.improper() {
        for &f in &faces {
            reorient(topo, f, &sense_change)?;
        }
    }

    // 6. tolerances
    if xf.scale != 1.0 {
        topo.scale_tolerances(closure, xf.scale).or_op()?;
    }

    // 7. consistency
    for &f in &faces {
        verify_pcurves(topo, f)?;
    }
    Ok(())
}

fn surface_kind(d: &SurfaceDef) -> &'static str {
    match d {
        SurfaceDef::Plane(_) => "plane",
        SurfaceDef::Cylinder(_) => "cylinder",
        SurfaceDef::Cone(_) => "cone",
        SurfaceDef::Sphere(_) => "sphere",
        SurfaceDef::Torus(_) => "torus",
        SurfaceDef::EllipticCylinder(_) => "elliptic cylinder",
        SurfaceDef::EllipticCone(_) => "elliptic cone",
        SurfaceDef::Nurbs(_) => "NURBS",
        SurfaceDef::Implicit(_) => "implicit",
        _ => "unknown",
    }
}

/// Restore the orientation rule for a mirrored face.
///
/// A reflection keeps every face's loops and its side tags, but turns the loop-derived normal
/// inwards. The loops must run the other way while the volumes stay on the same side tags. The
/// only primitive that reverses loops is `reverse_face`, which also swaps the sides of the
/// volume uses and flips the carrier sense; `end_op` would swap the sides straight back for a
/// lone volume. So: take the uses off, reverse, put the carrier sense right, and put the uses
/// back with their original sides and shells.
///
/// The right carrier sense is the pre-op sense composed with the sense change of the image
/// surface (`Reversed` for planes and NURBS under an improper map, `Forward` for surfaces of
/// revolution and implicit forms).
fn reorient(topo: &mut dyn TopoHost, face: EntityId, change: &BTreeMap<EntityId, sm_geom_api::Sense>) -> Result<(), OpError> {
    let mut uses: Vec<(EntityId, Side, u16)> = topo.coboundary(face).iter().filter(|i| i.cell.kind() == EntityKind::TopoVolume).map(|i| (i.cell, i.side, i.ring)).collect();
    // `remove_face_use` re-applies the orientation rule, which would reverse a face left with a
    // lone `Right` use and so spoil the parity of our own reversal. Taking the right-hand uses
    // off first never leaves a lone one; the loop below guards that no hidden flip happened.
    uses.sort_by_key(|&(v, side, ring)| (matches!(side, Side::Left) as u8 + 2 * matches!(side, Side::Both) as u8, v, ring));
    let row_before = topo.boundary(face).to_vec();
    for &(v, side, _) in &uses {
        topo.remove_face_use(v, face, side).or_op()?;
    }
    if topo.boundary(face) != row_before.as_slice() {
        return Err(OpError::Failed(format!("reflect: removing the volume uses of {face} reversed it (orientation rule)")));
    }
    uses.reverse();
    let before = topo.carrier(face).clone();
    topo.reverse_face(face).or_op()?;
    if let Carrier::Surface(sc) = before {
        let flips = change.get(&sc.surface).copied() == Some(sm_geom_api::Sense::Reversed);
        let want = if flips { sc.sense.flip() } else { sc.sense };
        topo.set_carrier(face, Carrier::Surface(SurfaceCarrier { surface: sc.surface, sense: want })).or_op()?;
    }
    for &(v, side, ring) in &uses {
        topo.add_face_use(v, ring, FaceUseSpec { face, side }).or_op()?;
    }
    Ok(())
}

/// After the transform every pcurve must still evaluate, on its surface, to the point of its
/// edge's curve (same parameter). Catches an uncovered surface/pcurve combination instead of
/// storing a silently wrong model.
fn verify_pcurves(topo: &dyn TopoHost, face: EntityId) -> Result<(), OpError> {
    let Carrier::Surface(sc) = topo.carrier(face) else { return Ok(()) };
    let shapes = topo.shapes();
    let Some(surf) = shapes.surface(sc.surface) else { return Ok(()) };
    for co in topo.coedges(face) {
        let Some(pc) = topo.pcurve(co.key) else { continue };
        if topo.flags(co.key.edge).contains(CellFlags::DEGENERATE) {
            continue;
        }
        let Carrier::Curve(cc) = topo.carrier(co.key.edge) else { continue };
        let Some(curve) = shapes.curve(cc.curve) else { continue };
        let r = pc.range();
        for k in 0..=2 {
            let t = r.lo + (r.hi - r.lo) * (k as f64 / 2.0);
            let (p, q) = (surf.eval(pc.eval(t)), curve.eval(t));
            let gap = (p - q).norm();
            let scale = 1.0 + q.x.abs().max(q.y.abs()).max(q.z.abs());
            if gap > 1e-9 * scale {
                return Err(XformError::PcurveDrift { key: co.key, gap }.into());
            }
        }
    }
    Ok(())
}
