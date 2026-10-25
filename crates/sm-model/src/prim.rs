//! Primitive-creating ops: `model.create_brick`, `model.create_cylinder`, `model.create_sphere`.
//!
//! All are centred on `center` (default the origin), like Cubit. Each builds its cells through
//! one [`topo_txn`] and reports the new cells under the declared output roles.

use core::f64::consts::{FRAC_PI_2, TAU};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_geom_api::{Circle, CurveDef, Cylinder, Frame, Interval, Line, Line2, PCurve, Plane, Sphere, SurfaceDef, Uv};
use sm_layer_api::{LayerError, TopoHost, commit_topology, topo_txn};
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, TypedOp};
use sm_topo_api::fixtures::brick;
use sm_topo_api::{CellFlags, CoedgeKey, CoedgeSpec, CurveCarrier, FaceUseSpec, Sense, ShellKind, ShellSpec, Side, SurfaceCarrier};

/// Ids produced by a primitive builder, grouped by output role.
struct Built {
    body: EntityId,
    volume: EntityId,
    faces: Vec<EntityId>,
    edges: Vec<EntityId>,
    vertices: Vec<EntityId>,
}

const SOLID_OUTPUTS: &[OutputRole] = &[
    OutputRole::new("body", &[EntityKind::TopoBody], Cardinality::One, "The new body."),
    OutputRole::new("volume", &[EntityKind::TopoVolume], Cardinality::One, "The new volume."),
    OutputRole::new("faces", &[EntityKind::TopoFace], Cardinality::Many, "The faces of the solid."),
    OutputRole::new("edges", &[EntityKind::TopoEdge], Cardinality::Many, "The edges of the solid."),
    OutputRole::new("vertices", &[EntityKind::TopoVertex], Cardinality::Many, "The vertices of the solid."),
];

fn positive(name: &str, v: f64) -> Result<f64, OpError> {
    if v.is_finite() && v > 0.0 { Ok(v) } else { Err(OpError::Invalid(format!("`{name}` must be finite and positive, got {v}"))) }
}

fn finite3(name: &str, v: [f64; 3]) -> Result<Point3, OpError> {
    if v.iter().all(|c| c.is_finite()) { Ok(Point3::new(v[0], v[1], v[2])) } else { Err(OpError::Invalid(format!("`{name}` must be finite, got {v:?}"))) }
}

/// Require `lo < hi` and both finite: the derived coordinates must be representable and
/// distinct (an extent that underflows or vanishes next to the centre would give coincident
/// corners). Extents that are representable but below the model tolerance are accepted.
fn span(name: &str, lo: f64, hi: f64) -> Result<(), OpError> {
    if lo.is_finite() && hi.is_finite() && lo < hi {
        Ok(())
    } else {
        Err(OpError::Invalid(format!("`{name}` and `center` give a non-finite or degenerate range [{lo}, {hi}]")))
    }
}

/// Run `build` in a topology transaction and commit its ids under the solid roles.
///
/// `smallest` is the smallest feature size (an extent or a diameter); one at or below the model's
/// linear tolerance would merge points the tolerance calls identical, so it is refused.
fn create(ctx: &mut ApplyCtx<'_>, smallest: f64, build: impl FnOnce(&mut dyn TopoHost) -> Result<Built, LayerError>) -> Result<(), OpError> {
    let (b, txn) = topo_txn(ctx, |topo, _model| {
        let tol = topo.tolerance().linear;
        if smallest <= tol {
            return Err(OpError::Invalid(format!("size {smallest:e} is not above the model linear tolerance {tol:e}")));
        }
        build(topo).map_err(OpError::from)
    })?;
    commit_topology(
        ctx,
        &txn,
        &[("body", &[b.body]), ("volume", &[b.volume]), ("faces", &b.faces), ("edges", &b.edges), ("vertices", &b.vertices)],
    )
}

/// `model.create_brick`: an axis-aligned box centred on `center` (default the origin).
///
/// Extents `x`, `y`, `z`; `y` and `z` default to `x` (a cube). Every primitive refuses (`Invalid`) a size at or below the model's linear tolerance, and any size or centre whose derived coordinates are not finite and distinct. The minimum corner is
/// `center - extent / 2` and the others are `min + extent` (the `sm_topo_api::fixtures::brick`
/// construction), so for representable values such as `x = 10` the vertices are exactly `±5`.
/// Face order is `-x, +x, -y, +y, -z, +z`; vertex index `i + 2j + 4k`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CreateBrick {
    /// Extent along x.
    pub x: f64,
    /// Extent along y (default `x`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub y: Option<f64>,
    /// Extent along z (default `x`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub z: Option<f64>,
    /// Centre (default the origin).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub center: Option<[f64; 3]>,
}

impl TypedOp for CreateBrick {
    const NAME: &'static str = "model.create_brick";
    const OUTPUTS: &'static [OutputRole] = SOLID_OUTPUTS;
    const POSITIONAL: Option<&'static str> = Some("x");

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let x = positive("x", self.x)?;
        let y = positive("y", self.y.unwrap_or(x))?;
        let z = positive("z", self.z.unwrap_or(x))?;
        let c = finite3("center", self.center.unwrap_or([0.0; 3]))?;
        let min = Point3::new(c.x - x / 2.0, c.y - y / 2.0, c.z - z / 2.0);
        // The same expression the fixture uses for the opposite corner.
        span("x", min.x, min.x + x)?;
        span("y", min.y, min.y + y)?;
        span("z", min.z, min.z + z)?;
        create(ctx, x.min(y).min(z), |topo| {
            let b = brick(topo, min, [x, y, z])?;
            Ok(Built { body: b.body, volume: b.volume, faces: b.faces.to_vec(), edges: b.edges.to_vec(), vertices: b.vertices.to_vec() })
        })
    }
}

/// `model.create_cylinder`: a solid circular cylinder about the z axis through `center`.
///
/// Spans `z = center.z ± height / 2`. Topology as in `sm_topo_api::fixtures::cylinder`: 2
/// vertices on the seam, 3 edges (two circles and the seam, used twice by the lateral face), 3
/// faces `[bottom, top, lateral]`; the lateral face carries pcurves in `(angle, z)`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CreateCylinder {
    /// Height along z.
    pub height: f64,
    /// Radius.
    pub radius: f64,
    /// Centre (default the origin).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub center: Option<[f64; 3]>,
}

impl TypedOp for CreateCylinder {
    const NAME: &'static str = "model.create_cylinder";
    const OUTPUTS: &'static [OutputRole] = SOLID_OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let h = positive("height", self.height)?;
        let r = positive("radius", self.radius)?;
        let c = finite3("center", self.center.unwrap_or([0.0; 3]))?;
        let base = Point3::new(c.x, c.y, c.z - h / 2.0);
        span("height", base.z, base.z + h)?;
        span("radius", c.x - r, c.x + r)?;
        span("radius", c.y - r, c.y + r)?;
        create(ctx, h.min(2.0 * r), |topo| build_cylinder(topo, base, r, h))
    }
}

/// `model.create_sphere`: a solid sphere centred on `center` (default the origin).
///
/// Topology as in `sm_topo_api::fixtures::sphere`: 2 pole vertices, a meridian seam edge used
/// twice and two degenerate pole edges, one face.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CreateSphere {
    /// Radius.
    pub radius: f64,
    /// Centre (default the origin).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub center: Option<[f64; 3]>,
}

impl TypedOp for CreateSphere {
    const NAME: &'static str = "model.create_sphere";
    const OUTPUTS: &'static [OutputRole] = SOLID_OUTPUTS;
    const POSITIONAL: Option<&'static str> = Some("radius");

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let r = positive("radius", self.radius)?;
        let c = finite3("center", self.center.unwrap_or([0.0; 3]))?;
        span("radius", c.x - r, c.x + r)?;
        span("radius", c.y - r, c.y + r)?;
        span("radius", c.z - r, c.z + r)?;
        create(ctx, 2.0 * r, |topo| build_sphere(topo, c, r))
    }
}

fn curve(topo: &mut dyn TopoHost, def: CurveDef) -> Result<EntityId, LayerError> {
    Ok(topo.shapes_mut()?.add_curve(def)?)
}

fn surface(topo: &mut dyn TopoHost, def: SurfaceDef) -> Result<EntityId, LayerError> {
    Ok(topo.shapes_mut()?.add_surface(def)?)
}

fn line2(origin: (f64, f64), dir: (f64, f64), range: Interval) -> PCurve {
    PCurve::Line2(Line2 { origin: Uv::new(origin.0, origin.1), dir: Uv::new(dir.0, dir.1), range })
}

/// The `sm_topo_api::fixtures::cylinder` construction with its bottom-circle centre at `base`.
fn build_cylinder(topo: &mut dyn TopoHost, base: Point3, r: f64, h: f64) -> Result<Built, LayerError> {
    let top = Point3::new(base.x, base.y, base.z + h);
    let vb = topo.make_vertex(Point3::new(base.x + r, base.y, base.z))?;
    let vt = topo.make_vertex(Point3::new(base.x + r, base.y, base.z + h))?;
    let x = Vector3::new(1.0, 0.0, 0.0);
    let z = Vector3::new(0.0, 0.0, 1.0);
    let bottom_frame = Frame::from_zx(base, z, x)?;
    let top_frame = Frame::from_zx(top, z, x)?;
    let cb = curve(topo, CurveDef::Circle(Circle::new(bottom_frame, r)?))?;
    let ct = curve(topo, CurveDef::Circle(Circle::new(top_frame, r)?))?;
    let cs = curve(topo, CurveDef::Line(Line::new(Point3::new(base.x + r, base.y, base.z), z)?))?;
    let full = Interval::new(0.0, TAU);
    let e_bot = topo.make_edge(vb, vb, Some(CurveCarrier { curve: cb, range: full, sense: Sense::Forward }))?;
    let e_top = topo.make_edge(vt, vt, Some(CurveCarrier { curve: ct, range: full, sense: Sense::Forward }))?;
    let e_seam = topo.make_edge(vb, vt, Some(CurveCarrier { curve: cs, range: Interval::new(0.0, h), sense: Sense::Forward }))?;
    let s_bot = surface(topo, SurfaceDef::Plane(Plane { frame: Frame::from_zx(base, Vector3::new(0.0, 0.0, -1.0), x)? }))?;
    let s_top = surface(topo, SurfaceDef::Plane(Plane { frame: top_frame }))?;
    let s_lat = surface(topo, SurfaceDef::Cylinder(Cylinder::new(Frame::at(base), r)?))?;
    let f_bot = topo.make_face(&[&[CoedgeSpec { edge: e_bot, sense: Sense::Reversed }]], Some(SurfaceCarrier { surface: s_bot, sense: Sense::Forward }))?;
    let f_top = topo.make_face(&[&[CoedgeSpec { edge: e_top, sense: Sense::Forward }]], Some(SurfaceCarrier { surface: s_top, sense: Sense::Forward }))?;
    let lateral = [
        CoedgeSpec { edge: e_bot, sense: Sense::Forward },
        CoedgeSpec { edge: e_seam, sense: Sense::Forward },
        CoedgeSpec { edge: e_top, sense: Sense::Reversed },
        CoedgeSpec { edge: e_seam, sense: Sense::Reversed },
    ];
    let f_lat = topo.make_face(&[&lateral], Some(SurfaceCarrier { surface: s_lat, sense: Sense::Forward }))?;
    let key = |edge, side| CoedgeKey { face: f_lat, edge, side };
    topo.set_pcurve(key(e_bot, Side::Left), Some(line2((0.0, 0.0), (1.0, 0.0), full)))?;
    topo.set_pcurve(key(e_seam, Side::Left), Some(line2((TAU, 0.0), (0.0, 1.0), Interval::new(0.0, h))))?;
    topo.set_pcurve(key(e_top, Side::Right), Some(line2((0.0, h), (1.0, 0.0), full)))?;
    topo.set_pcurve(key(e_seam, Side::Right), Some(line2((0.0, 0.0), (0.0, 1.0), Interval::new(0.0, h))))?;
    let uses = [f_bot, f_top, f_lat].map(|face| FaceUseSpec { face, side: Side::Left });
    let volume = topo.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &uses }])?;
    let body = topo.make_body(&[volume])?;
    Ok(Built { body, volume, faces: vec![f_bot, f_top, f_lat], edges: vec![e_bot, e_top, e_seam], vertices: vec![vb, vt] })
}

/// The `sm_topo_api::fixtures::sphere` construction centred on `c`.
fn build_sphere(topo: &mut dyn TopoHost, c: Point3, r: f64) -> Result<Built, LayerError> {
    let vs = topo.make_vertex(Point3::new(c.x, c.y, c.z - r))?;
    let vn = topo.make_vertex(Point3::new(c.x, c.y, c.z + r))?;
    let frame = Frame::from_zx(c, Vector3::new(0.0, -1.0, 0.0), Vector3::new(1.0, 0.0, 0.0))?;
    let meridian = curve(topo, CurveDef::Circle(Circle::new(frame, r)?))?;
    let seam = topo.make_edge(vs, vn, Some(CurveCarrier { curve: meridian, range: Interval::new(-FRAC_PI_2, FRAC_PI_2), sense: Sense::Forward }))?;
    let pole_s = topo.make_edge(vs, vs, None)?;
    let pole_n = topo.make_edge(vn, vn, None)?;
    topo.set_flags(pole_s, CellFlags::DEGENERATE)?;
    topo.set_flags(pole_n, CellFlags::DEGENERATE)?;
    let surf = surface(topo, SurfaceDef::Sphere(Sphere::new(Frame::at(c), r)?))?;
    let lp = [
        CoedgeSpec { edge: pole_s, sense: Sense::Forward },
        CoedgeSpec { edge: seam, sense: Sense::Forward },
        CoedgeSpec { edge: pole_n, sense: Sense::Reversed },
        CoedgeSpec { edge: seam, sense: Sense::Reversed },
    ];
    let face = topo.make_face(&[&lp], Some(SurfaceCarrier { surface: surf, sense: Sense::Forward }))?;
    let key = |edge, side| CoedgeKey { face, edge, side };
    topo.set_pcurve(key(pole_s, Side::Left), Some(line2((0.0, -FRAC_PI_2), (1.0, 0.0), Interval::new(0.0, TAU))))?;
    topo.set_pcurve(key(seam, Side::Left), Some(line2((TAU, 0.0), (0.0, 1.0), Interval::new(-FRAC_PI_2, FRAC_PI_2))))?;
    topo.set_pcurve(key(pole_n, Side::Right), Some(line2((0.0, FRAC_PI_2), (1.0, 0.0), Interval::new(0.0, TAU))))?;
    topo.set_pcurve(key(seam, Side::Right), Some(line2((0.0, 0.0), (0.0, 1.0), Interval::new(-FRAC_PI_2, FRAC_PI_2))))?;
    let uses = [FaceUseSpec { face, side: Side::Left }];
    let volume = topo.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &uses }])?;
    let body = topo.make_body(&[volume])?;
    Ok(Built { body, volume, faces: vec![face], edges: vec![pole_s, seam, pole_n], vertices: vec![vs, vn] })
}
