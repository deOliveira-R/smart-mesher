//! Transform forms that resolve their parameters from the model: `model.move_to` (move a
//! selection's anchor to a point or an entity) and `model.rotate_about` (the axis from a world
//! axis, two vertices or a face).
//!
//! Both end in the same `apply_similarity` path as `model.move` and `model.rotate`, so they share
//! the refusals (meshed, shared cells) and the pcurve handling.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{Aabb, EntityId, EntityKind, Point3, Vector3};
use sm_geom_api::SurfaceDef;
use sm_layer_api::ModelLayer;
use sm_op_api::{ApplyCtx, OpError, TypedOp};
use sm_topo_api::{Carrier, Dim, Side, TopologyView};

use crate::queries::{edge_bbox, face_area, face_bbox, volume_value};
use crate::xform::{OUTPUTS, XformError, apply_similarity, move_xf, rotation_xf, select};

/// Where `model.move_to` puts the anchor of the selection.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum MoveTarget {
    /// Three coordinates.
    Coords([f64; 3]),
    /// A point given by name; an omitted coordinate is left unchanged.
    Point(PointSpec),
    /// An entity: a vertex (its point) or an edge, face, volume or body (its centroid).
    Entity(
        #[schemars(extend("x-entity-kind" = ["TopoVertex", "TopoEdge", "TopoFace", "TopoVolume", "TopoBody"]))]
        EntityId,
    ),
}

/// A partial point for `model.move_to`: the coordinates that are given move the anchor there,
/// the others keep the anchor's own value. (Op parameters cannot hold `null`, so an unchanged
/// component is an omitted field rather than a `null` array element.)
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PointSpec {
    /// Target x.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub x: Option<f64>,
    /// Target y.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub y: Option<f64>,
    /// Target z.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub z: Option<f64>,
}

/// The point of the selection that `model.move_to` moves to the target.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Anchor {
    /// The centroid of the selection's volumes (volume-weighted).
    #[default]
    Centroid,
    /// The centre of the bounding box of the selection's faces.
    BboxCenter,
}

/// `model.move_to`: translate the selection so that its anchor lands on `to`.
///
/// `delta = to - anchor`, then the same path as `model.move` (same refusals). `to` is
/// `[x, y, z]`, an object with any of `x`, `y`, `z` (an omitted coordinate is unchanged), or the
/// id of an entity. The anchor defaults to the centroid of the selected volumes.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MoveTo {
    /// Volumes, or bodies standing for their members.
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoBody"]))]
    pub entities: Vec<EntityId>,
    /// The target point or entity.
    pub to: MoveTarget,
    /// The selection's anchor (default `centroid`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub anchor: Option<Anchor>,
}

fn arr(p: Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}

fn q(e: impl std::fmt::Display) -> OpError {
    OpError::Failed(format!("measure: {e}"))
}

/// The point of a vertex.
pub(crate) fn vertex_point(topo: &dyn TopologyView, v: EntityId) -> Result<Point3, OpError> {
    if v.kind() != EntityKind::TopoVertex {
        return Err(OpError::WrongKind { id: v, expected: EntityKind::TopoVertex });
    }
    match topo.carrier(v) {
        Carrier::Point(p) if topo.is_alive(v) => Ok(*p),
        _ => Err(OpError::UnknownEntity(v)),
    }
}

/// Centroid of an edge's curve (arc-length weighted, composite Simpson over the parameter).
fn edge_centroid(topo: &dyn TopologyView, e: EntityId) -> Result<Point3, OpError> {
    let Carrier::Curve(cc) = topo.carrier(e) else { return Err(OpError::Failed(format!("{e} has no curve"))) };
    let curve = topo.shapes().curve(cc.curve).ok_or_else(|| OpError::Failed(format!("curve {} is not in the store", cc.curve)))?;
    const N: usize = 2048;
    let (mut len, mut acc) = (0.0, Vector3::new(0.0, 0.0, 0.0));
    for k in 0..=N {
        let w = if k == 0 || k == N { 1.0 } else if k % 2 == 1 { 4.0 } else { 2.0 };
        let j = curve.jet(cc.range.lo + (cc.range.hi - cc.range.lo) * (k as f64 / N as f64));
        let s = w * j.d1.norm();
        len += s;
        acc = acc + Vector3::new(j.p.x, j.p.y, j.p.z) * s;
    }
    if len > 0.0 { Ok(Point3::new(acc.x / len, acc.y / len, acc.z / len)) } else { Err(OpError::Failed(format!("{e} has zero length"))) }
}

/// Volume-weighted centroid of volumes.
fn volumes_centroid(topo: &dyn TopologyView, volumes: &[EntityId]) -> Result<Point3, OpError> {
    let (mut vol, mut first) = (0.0, [0.0; 3]);
    for &v in volumes {
        let (vv, c) = volume_value(topo, v)?;
        let Some(c) = c else { continue };
        vol += vv;
        for (k, x) in arr(c).iter().enumerate() {
            first[k] += vv * x;
        }
    }
    if vol.abs() > 0.0 { Ok(Point3::new(first[0] / vol, first[1] / vol, first[2] / vol)) } else { Err(OpError::Failed("the selection has no volume to take a centroid of".into())) }
}

fn volumes_of(topo: &dyn TopologyView, e: EntityId) -> Vec<EntityId> {
    if e.kind() == EntityKind::TopoBody { topo.boundary(e).iter().map(|i| i.cell).filter(|c| c.kind() == EntityKind::TopoVolume).collect() } else { vec![e] }
}

fn target_point(topo: &dyn TopologyView, e: EntityId) -> Result<Point3, OpError> {
    let dim = Dim::of_real_kind(e.kind()).ok_or(OpError::WrongKind { id: e, expected: EntityKind::TopoVertex })?;
    if !topo.is_alive(e) {
        return Err(OpError::UnknownEntity(e));
    }
    match dim {
        Dim::Vertex => vertex_point(topo, e),
        Dim::Edge => edge_centroid(topo, e),
        Dim::Face => face_area(topo, e)?.1.ok_or_else(|| q(format!("{e} has no centroid"))),
        Dim::Volume | Dim::Body => volumes_centroid(topo, &volumes_of(topo, e)),
    }
}

impl TypedOp for MoveTo {
    const NAME: &'static str = "model.move_to";
    const OUTPUTS: &'static [sm_op_api::OutputRole] = OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let sel = select(ctx, &self.entities)?;
        let delta = {
            let topo = ctx.layer::<ModelLayer>()?.host();
            let anchor = match self.anchor.unwrap_or_default() {
                Anchor::Centroid => volumes_centroid(topo, &sel.volumes)?,
                Anchor::BboxCenter => {
                    let mut bb = Aabb::empty();
                    for &c in sel.closure.iter().filter(|c| c.kind() == EntityKind::TopoFace) {
                        bb = bb.union(&face_bbox(topo, c));
                    }
                    for &c in sel.closure.iter().filter(|c| c.kind() == EntityKind::TopoEdge) {
                        bb = bb.union(&edge_bbox(topo, c));
                    }
                    if bb.is_empty() {
                        return Err(OpError::Failed("the selection has no geometry to bound".into()));
                    }
                    Point3::new((bb.lo.x + bb.hi.x) / 2.0, (bb.lo.y + bb.hi.y) / 2.0, (bb.lo.z + bb.hi.z) / 2.0)
                }
            };
            match self.to {
                MoveTarget::Coords(_) | MoveTarget::Point(_) => {
                    let p = match self.to {
                        MoveTarget::Coords(c) => [Some(c[0]), Some(c[1]), Some(c[2])],
                        MoveTarget::Point(p) => [p.x, p.y, p.z],
                        MoveTarget::Entity(_) => [None; 3],
                    };
                    let a = arr(anchor);
                    let mut d = [0.0; 3];
                    for k in 0..3 {
                        if let Some(t) = p[k] {
                            if !t.is_finite() {
                                return Err(XformError::BadParam(format!("`to` must be finite, got {p:?}")).into());
                            }
                            d[k] = t - a[k];
                        }
                    }
                    d
                }
                MoveTarget::Entity(e) => {
                    ctx.require(e)?;
                    let t = arr(target_point(topo, e)?);
                    let a = arr(anchor);
                    [t[0] - a[0], t[1] - a[1], t[2] - a[2]]
                }
            }
        };
        apply_similarity(ctx, &self.entities, move_xf(delta)?)
    }
}

/// A rotation axis given directly.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum AxisSpec {
    /// A direction vector (not necessarily unit).
    Vector([f64; 3]),
    /// A world axis.
    Name(AxisName),
}

/// A world axis by name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum AxisName {
    /// `+x`.
    X,
    /// `+y`.
    Y,
    /// `+z`.
    Z,
}

impl AxisSpec {
    fn vector(self) -> [f64; 3] {
        match self {
            AxisSpec::Vector(v) => v,
            AxisSpec::Name(AxisName::X) => [1.0, 0.0, 0.0],
            AxisSpec::Name(AxisName::Y) => [0.0, 1.0, 0.0],
            AxisSpec::Name(AxisName::Z) => [0.0, 0.0, 1.0],
        }
    }
}

/// `model.rotate_about`: rotate the selection by `angle_deg` (right-hand rule) about an axis given
/// by exactly one of three forms.
///
/// - `axis`: a vector or `"x" | "y" | "z"`; pivot `origin` (default the world origin).
/// - `vertices: [a, b]`: the axis runs from vertex `a` to vertex `b`, pivot `a`.
/// - `face`: the axis is the face's unit normal pointing out of its volume (the carrier sense,
///   flipped for a right-hand volume use), pivot the face centroid. Curved faces use the normal
///   at the surface point nearest the centroid, which is then the pivot.
///
/// `origin` overrides the pivot of the second and third form. Any other combination is `Invalid`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RotateAbout {
    /// Volumes, or bodies standing for their members.
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoBody"]))]
    pub entities: Vec<EntityId>,
    /// Angle in degrees.
    pub angle_deg: f64,
    /// The axis direction (form 1).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub axis: Option<AxisSpec>,
    /// A point on the axis (form 1: default the origin; forms 2 and 3: overrides the pivot).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub origin: Option<[f64; 3]>,
    /// Two vertices that define the axis (form 2).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoVertex"]))]
    pub vertices: Option<[EntityId; 2]>,
    /// A face whose normal is the axis (form 3).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoFace"]))]
    pub face: Option<EntityId>,
}

/// The rotation forms of `model.rotate_about` and of a rotating `model.copy`.
pub(crate) struct RotationForm {
    pub angle_deg: f64,
    pub axis: Option<AxisSpec>,
    pub origin: Option<[f64; 3]>,
    pub vertices: Option<[EntityId; 2]>,
    pub face: Option<EntityId>,
}

/// Resolve exactly one rotation form to a transform.
pub(crate) fn resolve_rotation(ctx: &ApplyCtx<'_>, f: &RotationForm) -> Result<crate::xform::Xf, OpError> {
    let given = usize::from(f.axis.is_some()) + usize::from(f.vertices.is_some()) + usize::from(f.face.is_some());
    if given != 1 {
        return Err(XformError::NeedExactlyOne(format!("exactly one of `axis`, `vertices` and `face` is required, got {given}")).into());
    }
    if let Some(a) = f.axis {
        return Ok(rotation_xf(a.vector(), f.angle_deg, f.origin)?);
    }
    let topo = ctx.layer::<ModelLayer>()?.host();
    let (axis, pivot) = if let Some([a, b]) = f.vertices {
        ctx.require(a)?;
        ctx.require(b)?;
        let (pa, pb) = (vertex_point(topo, a)?, vertex_point(topo, b)?);
        let d = pb - pa;
        let len = d.norm();
        if len.is_nan() || len <= topo.tolerance().linear {
            return Err(XformError::Degenerate(format!("vertices {a} and {b} coincide within the linear tolerance: no axis")).into());
        }
        ([d.x, d.y, d.z], pa)
    } else if let Some(face) = f.face {
        ctx.require(face)?;
        face_axis(topo, face)?
    } else {
        return Err(XformError::NeedExactlyOne("no rotation form given".into()).into());
    };
    Ok(rotation_xf(axis, f.angle_deg, Some(arr(f.origin.map_or(pivot, |o| Point3::new(o[0], o[1], o[2])))))?)
}

/// Outward unit normal and pivot of a face (see [`RotateAbout`]).
fn face_axis(topo: &dyn TopologyView, face: EntityId) -> Result<([f64; 3], Point3), OpError> {
    if face.kind() != EntityKind::TopoFace {
        return Err(OpError::WrongKind { id: face, expected: EntityKind::TopoFace });
    }
    if !topo.is_alive(face) {
        return Err(OpError::UnknownEntity(face));
    }
    let Carrier::Surface(sc) = topo.carrier(face).clone() else { return Err(OpError::Failed(format!("{face} has no surface"))) };
    let centroid = face_area(topo, face)?.1.ok_or_else(|| q(format!("{face} has no centroid")))?;
    let mut sign = sc.sense.sign();
    if let Some(inc) = topo.coboundary(face).iter().find(|i| i.cell.kind() == EntityKind::TopoVolume)
        && inc.side == Side::Right
    {
        sign = -sign;
    }
    let (n, pivot) = match topo.shapes().surface_def(sc.surface) {
        Some(SurfaceDef::Plane(p)) => (p.frame.z, centroid),
        _ => {
            let s = topo.shapes().surface(sc.surface).ok_or_else(|| OpError::Failed(format!("surface {} is not in the store", sc.surface)))?;
            let pr = s.project(centroid, None).map_err(q)?;
            (s.normal(pr.uv).map_err(q)?, pr.point)
        }
    };
    Ok(([sign * n.x, sign * n.y, sign * n.z], pivot))
}

impl TypedOp for RotateAbout {
    const NAME: &'static str = "model.rotate_about";
    const OUTPUTS: &'static [sm_op_api::OutputRole] = OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let xf = resolve_rotation(ctx, &RotationForm { angle_deg: self.angle_deg, axis: self.axis, origin: self.origin, vertices: self.vertices, face: self.face })?;
        apply_similarity(ctx, &self.entities, xf)
    }
}
