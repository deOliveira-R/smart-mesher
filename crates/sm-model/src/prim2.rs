//! Cubit primitives beyond brick, cylinder and sphere: `model.create_frustum`,
//! `model.create_prism`, `model.create_pyramid`, `model.create_torus`.
//!
//! Same rules as `prim.rs`: one topology transaction, `SOLID_OUTPUTS`, exact carriers (no
//! fitting), and the shared size guard `create`.

use core::f64::consts::TAU;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{Circle, CurveDef, Cone, Ellipse, EllipticCone, EllipticCylinder, Frame, Interval, Line, Plane, SurfaceDef, Torus};
use sm_layer_api::{LayerError, TopoHost};
use sm_op_api::{ApplyCtx, OpError, OutputRole, TypedOp};
use sm_topo_api::{CellFlags, CoedgeKey, CoedgeSpec, CurveCarrier, FaceUseSpec, Sense, ShellKind, ShellSpec, Side, SurfaceCarrier};

use crate::prim::{Built, SOLID_OUTPUTS, build_cylinder, create, curve, finite3, line2, positive, span, surface};

/// Sizes above this make squared edge lengths overflow.
const TOO_LARGE: f64 = 1e150;

const F: Sense = Sense::Forward;
const R: Sense = Sense::Reversed;

fn nonneg(name: &str, v: f64) -> Result<f64, OpError> {
    if v.is_finite() && v >= 0.0 { Ok(v) } else { Err(OpError::Invalid(format!("`{name}` must be finite and not negative, got {v}"))) }
}

fn close_solid(topo: &mut dyn TopoHost, faces: &[EntityId]) -> Result<(EntityId, EntityId), LayerError> {
    let uses: Vec<FaceUseSpec> = faces.iter().map(|&face| FaceUseSpec { face, side: Side::Left }).collect();
    let volume = topo.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &uses }])?;
    let body = topo.make_body(&[volume])?;
    Ok((volume, body))
}

/// An edge along the straight segment `a -> b`, with a unit-speed line carrier.
fn line_edge(topo: &mut dyn TopoHost, a: (EntityId, Point3), b: (EntityId, Point3)) -> Result<EntityId, LayerError> {
    let d = b.1 - a.1;
    let c = curve(topo, CurveDef::Line(Line::new(a.1, d)?))?;
    Ok(topo.make_edge(a.0, b.0, Some(CurveCarrier { curve: c, range: Interval::new(0.0, d.norm()), sense: F }))?)
}

fn full() -> Interval {
    Interval::new(0.0, TAU)
}

// ------------------------------------------------------------------------- frustum

/// `model.create_frustum`: a circular or elliptic cone, cone frustum, cylinder or apex, about the
/// z axis through `center`.
///
/// `radius` is the bottom semi-axis along x, `minor` the bottom semi-axis along y (default
/// `radius`) and `top` the top semi-axis along x (default 0, a cone with its apex up). The top y
/// semi-axis is `top * minor / radius` (a homothetic section, as in Cubit). `top > radius`
/// builds the inverted frustum; `top == radius` is the cylinder (the same cells as
/// `model.create_cylinder` when circular). Spans `z = center.z +- height / 2`.
///
/// Circular cones carry a `Cone` lateral surface (frame at the larger end, `z` pointing away from
/// the other end), elliptic ones an `EllipticCone` or `EllipticCylinder` with `Ellipse` edges.
/// Cells: frustum faces `[bottom, top, lateral]`, vertices `[bottom, top]` on the seam at `u = 0`,
/// edges `[bottom, top, seam]`; an apex gives the `sm_topo_api::fixtures::cone` layout (faces
/// `[lateral, base]`, a degenerate pole edge).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CreateFrustum {
    /// Height along z.
    pub height: f64,
    /// Bottom semi-axis along x.
    pub radius: f64,
    /// Bottom semi-axis along y (default `radius`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub minor: Option<f64>,
    /// Top semi-axis along x (default 0, an apex).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub top: Option<f64>,
    /// Centre (default the origin).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub center: Option<[f64; 3]>,
}

impl TypedOp for CreateFrustum {
    const NAME: &'static str = "model.create_frustum";
    const OUTPUTS: &'static [OutputRole] = SOLID_OUTPUTS;
    const POSITIONAL: Option<&'static str> = Some("height");

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let h = positive("height", self.height)?;
        let r = positive("radius", self.radius)?;
        let b = positive("minor", self.minor.unwrap_or(r))?;
        let t = nonneg("top", self.top.unwrap_or(0.0))?;
        let c = finite3("center", self.center.unwrap_or([0.0; 3]))?;
        let top_y = if t == r { b } else { t * b / r };
        if !top_y.is_finite() || (t > 0.0 && top_y <= 0.0) {
            return Err(OpError::Invalid(format!("`top` {t:e} and `minor / radius` give a degenerate top section")));
        }
        let (lo, hi) = (c.z - h / 2.0, c.z - h / 2.0 + h);
        span("height", lo, hi)?;
        let (ax, by) = (r.max(t), b.max(top_y));
        if !(t == r && b == r) && h.max(ax).max(by) > TOO_LARGE {
            return Err(OpError::Invalid(format!("the solid is too large: sizes above {TOO_LARGE:e} overflow edge lengths")));
        }
        span("radius", c.x - ax, c.x + ax)?;
        span("minor", c.y - by, c.y + by)?;
        if t != r {
            let slant = (r - t).hypot(h);
            let half = (r - t).abs().atan2(h);
            if !slant.is_finite() || !(half > 0.0 && half < core::f64::consts::FRAC_PI_2) {
                return Err(OpError::Invalid(format!("`radius` {r:e} and `top` {t:e} differ too little (or too much) against `height` {h:e}")));
            }
        }
        let mut smallest = h.min(2.0 * r).min(2.0 * b);
        if t > 0.0 {
            smallest = smallest.min(2.0 * t).min(2.0 * top_y);
        }
        let sec = Section { r, b, t, top_y };
        create(ctx, smallest, |topo| {
            let base = Point3::new(c.x, c.y, lo);
            if t == r && b == r {
                build_cylinder(topo, base, r, h)
            } else {
                build_frustum(topo, base, h, &sec)
            }
        })
    }
}

/// Bottom and top semi-axes of a frustum.
struct Section {
    r: f64,
    b: f64,
    t: f64,
    top_y: f64,
}

/// One end of the lateral surface: centre, semi-axes.
fn ellipse_or_circle(frame: Frame, a: f64, b: f64) -> Result<CurveDef, LayerError> {
    Ok(if a == b { CurveDef::Circle(Circle::new(frame, a)?) } else { CurveDef::Ellipse(Ellipse::new(frame, a, b)?) })
}

fn build_frustum(topo: &mut dyn TopoHost, base: Point3, h: f64, s: &Section) -> Result<Built, LayerError> {
    let x = Vector3::new(1.0, 0.0, 0.0);
    let up = Vector3::new(0.0, 0.0, 1.0);
    let top_c = Point3::new(base.x, base.y, base.z + h);
    if s.t == s.r {
        return build_elliptic_cylinder(topo, base, s.r, s.b, h);
    }
    // The larger end carries the frame; `zdir` points away from the smaller end.
    let bottom_big = s.r > s.t;
    let (big_c, small_c, zdir) = if bottom_big { (base, top_c, -up) } else { (top_c, base, up) };
    let (a_big, b_big, a_small, b_small) = if bottom_big { (s.r, s.b, s.t, s.top_y) } else { (s.t, s.top_y, s.r, s.b) };
    let frame = Frame::from_zx(big_c, zdir, x)?;
    let da = a_big - a_small;
    let slant = da.hypot(h);
    let half = da.atan2(h);
    let circular = a_big == b_big;
    let lateral = if circular { SurfaceDef::Cone(Cone::new(frame, a_big, half)?) } else { SurfaceDef::EllipticCone(EllipticCone::new(frame, a_big, b_big, half)?) };
    let apex = a_small == 0.0;
    let big_end = Frame::from_zx(big_c, zdir, x)?;
    let big_pt = Point3::new(big_c.x + a_big, big_c.y, big_c.z);
    let small_pt = Point3::new(small_c.x + a_small, small_c.y, small_c.z);
    let gen_dir = small_pt - big_pt;
    let lateral_uv = |topo: &mut dyn TopoHost, face: EntityId, e_big: EntityId, e_seam: EntityId, e_small: EntityId, apex_v: Option<f64>| -> Result<(), LayerError> {
        let key = |edge, side| CoedgeKey { face, edge, side };
        let sl = Interval::new(0.0, slant);
        topo.set_pcurve(key(e_seam, Side::Right), Some(line2((TAU, 0.0), (0.0, -1.0), sl)))?;
        topo.set_pcurve(key(e_big, Side::Right), Some(line2((0.0, 0.0), (1.0, 0.0), full())))?;
        topo.set_pcurve(key(e_seam, Side::Left), Some(line2((0.0, 0.0), (0.0, -1.0), sl)))?;
        let v_small = apex_v.unwrap_or(-slant);
        topo.set_pcurve(key(e_small, Side::Left), Some(line2((0.0, v_small), (1.0, 0.0), full())))?;
        Ok(())
    };
    if apex {
        // The `fixtures::cone` construction, in its creation order.
        let vb = topo.make_vertex(big_pt)?;
        let vs = topo.make_vertex(small_c)?;
        let base_curve = curve(topo, ellipse_or_circle(big_end, a_big, b_big)?)?;
        let generator = curve(topo, CurveDef::Line(Line::new(big_pt, gen_dir)?))?;
        let e_big = topo.make_edge(vb, vb, Some(CurveCarrier { curve: base_curve, range: full(), sense: F }))?;
        let e_seam = topo.make_edge(vb, vs, Some(CurveCarrier { curve: generator, range: Interval::new(0.0, slant), sense: F }))?;
        let e_small = topo.make_edge(vs, vs, None)?;
        topo.set_flags(e_small, CellFlags::DEGENERATE)?;
        let apex_v = match &lateral {
            SurfaceDef::Cone(k) => k.apex_v(),
            SurfaceDef::EllipticCone(k) => k.apex_v(),
            _ => -slant,
        };
        let s_lat = surface(topo, lateral)?;
        let loop_lat = [
            CoedgeSpec { edge: e_small, sense: F },
            CoedgeSpec { edge: e_seam, sense: R },
            CoedgeSpec { edge: e_big, sense: R },
            CoedgeSpec { edge: e_seam, sense: F },
        ];
        let f_lat = topo.make_face(&[&loop_lat], Some(SurfaceCarrier { surface: s_lat, sense: F }))?;
        lateral_uv(topo, f_lat, e_big, e_seam, e_small, Some(apex_v))?;
        let s_disc = surface(topo, SurfaceDef::Plane(Plane { frame: big_end }))?;
        let f_big = topo.make_face(&[&[CoedgeSpec { edge: e_big, sense: F }]], Some(SurfaceCarrier { surface: s_disc, sense: F }))?;
        let (volume, body) = close_solid(topo, &[f_lat, f_big])?;
        // World order: bottom first.
        let (vertices, edges) = if bottom_big { (vec![vb, vs], vec![e_big, e_seam, e_small]) } else { (vec![vs, vb], vec![e_big, e_seam, e_small]) };
        return Ok(Built { body, volume, faces: vec![f_lat, f_big], edges, vertices });
    }
    let vb = topo.make_vertex(big_pt)?;
    let vs = topo.make_vertex(small_pt)?;
    let small_end = Frame::from_zx(small_c, zdir, x)?;
    let c_big = curve(topo, ellipse_or_circle(big_end, a_big, b_big)?)?;
    let c_small = curve(topo, ellipse_or_circle(small_end, a_small, b_small)?)?;
    let generator = curve(topo, CurveDef::Line(Line::new(big_pt, gen_dir)?))?;
    let e_big = topo.make_edge(vb, vb, Some(CurveCarrier { curve: c_big, range: full(), sense: F }))?;
    let e_small = topo.make_edge(vs, vs, Some(CurveCarrier { curve: c_small, range: full(), sense: F }))?;
    let e_seam = topo.make_edge(vb, vs, Some(CurveCarrier { curve: generator, range: Interval::new(0.0, slant), sense: F }))?;
    let s_lat = surface(topo, lateral)?;
    let s_big = surface(topo, SurfaceDef::Plane(Plane { frame: big_end }))?;
    let s_small = surface(topo, SurfaceDef::Plane(Plane { frame: Frame::from_zx(small_c, -zdir, x)? }))?;
    let f_big = topo.make_face(&[&[CoedgeSpec { edge: e_big, sense: F }]], Some(SurfaceCarrier { surface: s_big, sense: F }))?;
    let f_small = topo.make_face(&[&[CoedgeSpec { edge: e_small, sense: R }]], Some(SurfaceCarrier { surface: s_small, sense: F }))?;
    let loop_lat = [
        CoedgeSpec { edge: e_small, sense: F },
        CoedgeSpec { edge: e_seam, sense: R },
        CoedgeSpec { edge: e_big, sense: R },
        CoedgeSpec { edge: e_seam, sense: F },
    ];
    let f_lat = topo.make_face(&[&loop_lat], Some(SurfaceCarrier { surface: s_lat, sense: F }))?;
    lateral_uv(topo, f_lat, e_big, e_seam, e_small, None)?;
    let (f_bottom, f_top, e_bottom, e_top, v_bottom, v_top) = if bottom_big { (f_big, f_small, e_big, e_small, vb, vs) } else { (f_small, f_big, e_small, e_big, vs, vb) };
    let (volume, body) = close_solid(topo, &[f_bottom, f_top, f_lat])?;
    Ok(Built { body, volume, faces: vec![f_bottom, f_top, f_lat], edges: vec![e_bottom, e_top, e_seam], vertices: vec![v_bottom, v_top] })
}

/// The elliptic twin of `build_cylinder`: `EllipticCylinder` lateral, `Ellipse` edges, same layout.
fn build_elliptic_cylinder(topo: &mut dyn TopoHost, base: Point3, a: f64, b: f64, h: f64) -> Result<Built, LayerError> {
    let top = Point3::new(base.x, base.y, base.z + h);
    let vb = topo.make_vertex(Point3::new(base.x + a, base.y, base.z))?;
    let vt = topo.make_vertex(Point3::new(base.x + a, base.y, base.z + h))?;
    let x = Vector3::new(1.0, 0.0, 0.0);
    let z = Vector3::new(0.0, 0.0, 1.0);
    let bottom_frame = Frame::from_zx(base, z, x)?;
    let top_frame = Frame::from_zx(top, z, x)?;
    let cb = curve(topo, CurveDef::Ellipse(Ellipse::new(bottom_frame, a, b)?))?;
    let ct = curve(topo, CurveDef::Ellipse(Ellipse::new(top_frame, a, b)?))?;
    let cs = curve(topo, CurveDef::Line(Line::new(Point3::new(base.x + a, base.y, base.z), z)?))?;
    let e_bot = topo.make_edge(vb, vb, Some(CurveCarrier { curve: cb, range: full(), sense: F }))?;
    let e_top = topo.make_edge(vt, vt, Some(CurveCarrier { curve: ct, range: full(), sense: F }))?;
    let e_seam = topo.make_edge(vb, vt, Some(CurveCarrier { curve: cs, range: Interval::new(0.0, h), sense: F }))?;
    let s_bot = surface(topo, SurfaceDef::Plane(Plane { frame: Frame::from_zx(base, -z, x)? }))?;
    let s_top = surface(topo, SurfaceDef::Plane(Plane { frame: top_frame }))?;
    let s_lat = surface(topo, SurfaceDef::EllipticCylinder(EllipticCylinder::new(Frame::at(base), a, b)?))?;
    let f_bot = topo.make_face(&[&[CoedgeSpec { edge: e_bot, sense: R }]], Some(SurfaceCarrier { surface: s_bot, sense: F }))?;
    let f_top = topo.make_face(&[&[CoedgeSpec { edge: e_top, sense: F }]], Some(SurfaceCarrier { surface: s_top, sense: F }))?;
    let lateral = [CoedgeSpec { edge: e_bot, sense: F }, CoedgeSpec { edge: e_seam, sense: F }, CoedgeSpec { edge: e_top, sense: R }, CoedgeSpec { edge: e_seam, sense: R }];
    let f_lat = topo.make_face(&[&lateral], Some(SurfaceCarrier { surface: s_lat, sense: F }))?;
    let key = |edge, side| CoedgeKey { face: f_lat, edge, side };
    let hr = Interval::new(0.0, h);
    topo.set_pcurve(key(e_bot, Side::Left), Some(line2((0.0, 0.0), (1.0, 0.0), full())))?;
    topo.set_pcurve(key(e_seam, Side::Left), Some(line2((TAU, 0.0), (0.0, 1.0), hr)))?;
    topo.set_pcurve(key(e_top, Side::Right), Some(line2((0.0, h), (1.0, 0.0), full())))?;
    topo.set_pcurve(key(e_seam, Side::Right), Some(line2((0.0, 0.0), (0.0, 1.0), hr)))?;
    let (volume, body) = close_solid(topo, &[f_bot, f_top, f_lat])?;
    Ok(Built { body, volume, faces: vec![f_bot, f_top, f_lat], edges: vec![e_bot, e_top, e_seam], vertices: vec![vb, vt] })
}

// ------------------------------------------------------------------- prism, pyramid

/// `(cos, sin)` of `TAU * i / n`, exact at the quadrant points.
fn cos_sin(i: u32, n: u32) -> (f64, f64) {
    if (4 * u64::from(i)) % u64::from(n) == 0 {
        return [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)][(4 * u64::from(i) / u64::from(n)) as usize % 4];
    }
    let (s, c) = (TAU * f64::from(i) / f64::from(n)).sin_cos();
    (c, s)
}

fn sides_of(n: u32) -> Result<usize, OpError> {
    if (3..=1024).contains(&n) { Ok(n as usize) } else { Err(OpError::Invalid(format!("`sides` must be in 3..=1024, got {n}"))) }
}

/// Newell normal of a closed polygon (counter-clockwise seen from the tip gives the tip).
fn newell(pts: &[Point3]) -> Vector3 {
    let mut n = Vector3::new(0.0, 0.0, 0.0);
    for i in 0..pts.len() {
        let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
        n.x += (p.y - q.y) * (p.z + q.z);
        n.y += (p.z - q.z) * (p.x + q.x);
        n.z += (p.x - q.x) * (p.y + q.y);
    }
    n
}

/// A planar face from a loop, with the plane normal taken from the loop vertices.
fn plane_face(topo: &mut dyn TopoHost, specs: &[CoedgeSpec], pts: &[Point3]) -> Result<EntityId, LayerError> {
    let frame = Frame::from_zx(pts[0], newell(pts), pts[1] - pts[0])?;
    let s = surface(topo, SurfaceDef::Plane(Plane { frame }))?;
    Ok(topo.make_face(&[specs], Some(SurfaceCarrier { surface: s, sense: F }))?)
}

/// A regular `n`-gon prism or pyramid frustum with all-`Plane` faces.
///
/// `ring0` is the bottom circumradius, `ring1` the top (0 for an apex), `ratio` scales y
/// (`minor / radius`). Faces `[bottom, top?, laterals 0..n]`, lateral `i` spans vertex `i` to `i + 1`.
fn build_polygonal(topo: &mut dyn TopoHost, n: usize, c: Point3, h: f64, ring0: f64, ring1: f64, ratio: f64) -> Result<Built, LayerError> {
    let ring = |rad: f64, z: f64, i: usize| {
        let (cs, sn) = cos_sin(i as u32, n as u32);
        Point3::new(c.x + rad * cs, c.y + rad * ratio * sn, z)
    };
    let z0 = c.z - h / 2.0;
    let z1 = z0 + h;
    let apex = ring1 == 0.0;
    let pb: Vec<Point3> = (0..n).map(|i| ring(ring0, z0, i)).collect();
    let pt: Vec<Point3> = if apex { vec![Point3::new(c.x, c.y, z1)] } else { (0..n).map(|i| ring(ring1, z1, i)).collect() };
    let vb: Vec<EntityId> = pb.iter().map(|&p| topo.make_vertex(p)).collect::<Result<_, _>>()?;
    let vt: Vec<EntityId> = pt.iter().map(|&p| topo.make_vertex(p)).collect::<Result<_, _>>()?;
    let top_v = |i: usize| if apex { vt[0] } else { vt[i] };
    let top_p = |i: usize| if apex { pt[0] } else { pt[i] };
    let mut eb = Vec::with_capacity(n);
    for i in 0..n {
        eb.push(line_edge(topo, (vb[i], pb[i]), (vb[(i + 1) % n], pb[(i + 1) % n]))?);
    }
    let mut et = Vec::new();
    if !apex {
        for i in 0..n {
            et.push(line_edge(topo, (vt[i], pt[i]), (vt[(i + 1) % n], pt[(i + 1) % n]))?);
        }
    }
    let mut ev = Vec::with_capacity(n);
    for i in 0..n {
        ev.push(line_edge(topo, (vb[i], pb[i]), (top_v(i), top_p(i)))?);
    }
    let mut faces = Vec::with_capacity(n + 2);
    // Bottom: normal -z, so the loop runs against the ring order.
    let bottom: Vec<CoedgeSpec> = (0..n).rev().map(|i| CoedgeSpec { edge: eb[i], sense: R }).collect();
    let bottom_pts: Vec<Point3> = (0..n).rev().map(|i| pb[(i + 1) % n]).collect();
    faces.push(plane_face(topo, &bottom, &bottom_pts)?);
    if !apex {
        let top: Vec<CoedgeSpec> = (0..n).map(|i| CoedgeSpec { edge: et[i], sense: F }).collect();
        faces.push(plane_face(topo, &top, &pt)?);
    }
    for i in 0..n {
        let j = (i + 1) % n;
        let (spec, pts): (Vec<CoedgeSpec>, Vec<Point3>) = if apex {
            (
                vec![CoedgeSpec { edge: eb[i], sense: F }, CoedgeSpec { edge: ev[j], sense: F }, CoedgeSpec { edge: ev[i], sense: R }],
                vec![pb[i], pb[j], top_p(0)],
            )
        } else {
            (
                vec![CoedgeSpec { edge: eb[i], sense: F }, CoedgeSpec { edge: ev[j], sense: F }, CoedgeSpec { edge: et[i], sense: R }, CoedgeSpec { edge: ev[i], sense: R }],
                vec![pb[i], pb[j], pt[j], pt[i]],
            )
        };
        faces.push(plane_face(topo, &spec, &pts)?);
    }
    let (volume, body) = close_solid(topo, &faces)?;
    let mut vertices = vb;
    vertices.extend(vt);
    let mut edges = eb;
    edges.extend(et);
    edges.extend(ev);
    Ok(Built { body, volume, faces, edges, vertices })
}

/// Shortest edge of the polygonal solid, for the model-tolerance guard.
fn shortest_edge(n: usize, c: Point3, h: f64, ring0: f64, ring1: f64, ratio: f64) -> f64 {
    let p = |rad: f64, i: usize| {
        let (cs, sn) = cos_sin(i as u32, n as u32);
        (rad * cs, rad * ratio * sn)
    };
    let side = |rad: f64| {
        let (a, b) = (p(rad, 0), p(rad, 1));
        (a.0 - b.0).hypot(a.1 - b.1)
    };
    let _ = c;
    let mut m = h.min(side(ring0));
    if ring1 > 0.0 {
        m = m.min(side(ring1));
    }
    // Lateral edges: vertex 0 to its top counterpart.
    let top = if ring1 > 0.0 { p(ring1, 0) } else { (0.0, 0.0) };
    let b0 = p(ring0, 0);
    m.min((b0.0 - top.0).hypot(b0.1 - top.1).hypot(h))
}

/// `model.create_prism`: a regular `sides`-gon prism about the z axis through `center`.
///
/// `radius` is the circumradius (the polygon is inscribed in the circle or ellipse of that
/// radius); vertex 0 is on `+x` and the vertices run counter-clockwise seen from `+z`; `minor`
/// scales y (`y' = y * minor / radius`). Faces `[bottom, top, laterals 0..sides]`, `2n` vertices,
/// `3n` edges. `sides` is an integer in `3..=1024`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CreatePrism {
    /// Height along z.
    pub height: f64,
    /// Number of sides, `3..=1024`.
    pub sides: u32,
    /// Circumradius along x.
    pub radius: f64,
    /// Semi-axis along y (default `radius`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub minor: Option<f64>,
    /// Centre (default the origin).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub center: Option<[f64; 3]>,
}

impl TypedOp for CreatePrism {
    const NAME: &'static str = "model.create_prism";
    const OUTPUTS: &'static [OutputRole] = SOLID_OUTPUTS;
    const POSITIONAL: Option<&'static str> = Some("sides");

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        polygonal(ctx, self.height, self.sides, self.radius, self.radius, self.minor, self.center)
    }
}

/// `model.create_pyramid`: a regular `sides`-gon pyramid or truncated pyramid.
///
/// As [`CreatePrism`], with `top` the top circumradius (default 0, an apex). `top == radius` is
/// refused (use `model.create_prism`); `top > radius` builds the inverted pyramid frustum. An
/// apex gives `n + 1` vertices, `2n` edges and `n + 1` faces `[bottom, laterals]`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CreatePyramid {
    /// Height along z.
    pub height: f64,
    /// Number of sides, `3..=1024`.
    pub sides: u32,
    /// Bottom circumradius along x.
    pub radius: f64,
    /// Top circumradius (default 0, an apex).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub top: Option<f64>,
    /// Semi-axis along y at the bottom (default `radius`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub minor: Option<f64>,
    /// Centre (default the origin).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub center: Option<[f64; 3]>,
}

impl TypedOp for CreatePyramid {
    const NAME: &'static str = "model.create_pyramid";
    const OUTPUTS: &'static [OutputRole] = SOLID_OUTPUTS;
    const POSITIONAL: Option<&'static str> = Some("sides");

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let top = nonneg("top", self.top.unwrap_or(0.0))?;
        if top == self.radius {
            return Err(OpError::Invalid("`top` equals `radius`: that is a prism, use model.create_prism".into()));
        }
        polygonal(ctx, self.height, self.sides, self.radius, top, self.minor, self.center)
    }
}

fn polygonal(ctx: &mut ApplyCtx<'_>, height: f64, sides: u32, radius: f64, top: f64, minor: Option<f64>, center: Option<[f64; 3]>) -> Result<(), OpError> {
    let n = sides_of(sides)?;
    let h = positive("height", height)?;
    let r = positive("radius", radius)?;
    let b = positive("minor", minor.unwrap_or(r))?;
    let c = finite3("center", center.unwrap_or([0.0; 3]))?;
    let ratio = b / r;
    if !ratio.is_finite() || ratio <= 0.0 || !(top * ratio).is_finite() {
        return Err(OpError::Invalid("`minor / radius` is degenerate".into()));
    }
    let z0 = c.z - h / 2.0;
    span("height", z0, z0 + h)?;
    let big = r.max(top);
    span("radius", c.x - big, c.x + big)?;
    span("minor", c.y - big * ratio, c.y + big * ratio)?;
    if h.max(big).max(big * ratio) > TOO_LARGE {
        return Err(OpError::Invalid(format!("the solid is too large: sizes above {TOO_LARGE:e} overflow edge lengths")));
    }
    let smallest = shortest_edge(n, c, h, r, top, ratio);
    create(ctx, smallest, |topo| build_polygonal(topo, n, c, h, r, top, ratio))
}

// ---------------------------------------------------------------------------- torus

/// `model.create_torus`: a solid ring torus about the z axis through `center`.
///
/// `major > minor > 0` (spindle and horn tori are refused). Topology and pcurves are
/// `sm_topo_api::fixtures::torus` translated to `center`: 1 vertex at `(R + r, 0, 0)`, 2 closed
/// edges, 1 face, Euler characteristic 0.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CreateTorus {
    /// Major radius `R` (centre of the tube to the axis).
    pub major: f64,
    /// Minor radius `r` (tube).
    pub minor: f64,
    /// Centre (default the origin).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub center: Option<[f64; 3]>,
}

impl TypedOp for CreateTorus {
    const NAME: &'static str = "model.create_torus";
    const OUTPUTS: &'static [OutputRole] = SOLID_OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let r = positive("minor", self.minor)?;
        let big = positive("major", self.major)?;
        if big <= r {
            return Err(OpError::Invalid(format!("`major` {big} must exceed `minor` {r}: spindle and horn tori are not supported")));
        }
        let c = finite3("center", self.center.unwrap_or([0.0; 3]))?;
        let outer = big + r;
        span("major", c.x - outer, c.x + outer)?;
        span("major", c.y - outer, c.y + outer)?;
        span("minor", c.z - r, c.z + r)?;
        create(ctx, (2.0 * r).min(big - r), |topo| build_torus(topo, c, big, r))
    }
}

fn build_torus(topo: &mut dyn TopoHost, c: Point3, major: f64, minor: f64) -> Result<Built, LayerError> {
    let surface_def = Torus::new(Frame::at(c), major, minor)?;
    let v = topo.make_vertex(Point3::new(c.x + major + minor, c.y, c.z))?;
    let x = Vector3::new(1.0, 0.0, 0.0);
    let c_u = curve(topo, CurveDef::Circle(Circle::new(Frame::at(c), major + minor)?))?;
    // Meridian circle: x = R + r cos v, z = r sin v (normal -y so the angle runs from +x to +z).
    let c_v = curve(topo, CurveDef::Circle(Circle::new(Frame::from_zx(Point3::new(c.x + major, c.y, c.z), Vector3::new(0.0, -1.0, 0.0), x)?, minor)?))?;
    let e_u = topo.make_edge(v, v, Some(CurveCarrier { curve: c_u, range: full(), sense: F }))?;
    let e_v = topo.make_edge(v, v, Some(CurveCarrier { curve: c_v, range: full(), sense: F }))?;
    let surf = surface(topo, SurfaceDef::Torus(surface_def))?;
    let lp = [
        CoedgeSpec { edge: e_u, sense: F },
        CoedgeSpec { edge: e_v, sense: F },
        CoedgeSpec { edge: e_u, sense: R },
        CoedgeSpec { edge: e_v, sense: R },
    ];
    let face = topo.make_face(&[&lp], Some(SurfaceCarrier { surface: surf, sense: F }))?;
    let key = |edge, side| CoedgeKey { face, edge, side };
    topo.set_pcurve(key(e_u, Side::Left), Some(line2((0.0, 0.0), (1.0, 0.0), full())))?;
    topo.set_pcurve(key(e_v, Side::Left), Some(line2((TAU, 0.0), (0.0, 1.0), full())))?;
    topo.set_pcurve(key(e_u, Side::Right), Some(line2((0.0, TAU), (1.0, 0.0), full())))?;
    topo.set_pcurve(key(e_v, Side::Right), Some(line2((0.0, 0.0), (0.0, 1.0), full())))?;
    let (volume, body) = close_solid(topo, &[face])?;
    Ok(Built { body, volume, faces: vec![face], edges: vec![e_u, e_v], vertices: vec![v] })
}
