//! Single-face fixtures: polygonal planar faces (with holes), the circular annulus and a
//! latitude/longitude zone of a sphere.

use super::run;
use crate::loops2d::Region2;
use core::f64::consts::{PI, TAU};
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{Circle, CurveDef, Frame, Interval, Line, Line2, PCurve, Plane, Sphere, SurfaceDef, Uv};
use sm_topo_api::fixtures::FixtureError;
use sm_topo_api::{CellComplex, CoedgeKey, CoedgeSpec, CurveCarrier, Sense, Side, SurfaceCarrier, TopoError, TopologyEdit};

/// A complex holding one face (no volume) with the expectations of its construction.
#[derive(Clone, Debug)]
pub struct FaceFixture {
    /// Fixture name.
    pub name: &'static str,
    /// The complex (one face).
    pub complex: CellComplex,
    /// The face.
    pub face: EntityId,
    /// Vertex ids per loop (outer loop first), in loop order. A circle loop has one vertex.
    pub vertices: Vec<Vec<EntityId>>,
    /// Edge ids per loop, in loop order: edge `i` of a polygon loop runs from vertex `i` to
    /// vertex `i + 1`.
    pub edges: Vec<Vec<EntityId>>,
    /// Exact face area.
    pub area: f64,
    /// Topological Euler characteristic of the face: `1 - holes`. The combinatorial
    /// `euler_characteristic` of the complex (`V - E + F`) is 1 for every such face, because each
    /// loop has as many vertices as edges.
    pub euler: i64,
}

impl FaceFixture {
    /// Number of loops (`1 + holes`).
    pub fn loop_count(&self) -> usize {
        self.vertices.len()
    }
}

fn bad(msg: &'static str) -> FixtureError {
    FixtureError::Topo(TopoError::InvalidParameter(msg))
}

fn plane_z(z: f64) -> Result<SurfaceDef, FixtureError> {
    let n = Vector3::new(0.0, 0.0, 1.0);
    Ok(SurfaceDef::Plane(Plane { frame: Frame::from_zx(Point3::new(0.0, 0.0, z), n, Vector3::new(1.0, 0.0, 0.0))? }))
}

type LoopIds = Vec<Vec<EntityId>>;

/// The planar face of `region` at height `z`: one straight `Line` edge per segment, the outer
/// loop counter-clockwise and the holes clockwise as given (normal `+z`), `Plane` carrier.
///
/// Invariants: `loop_count() == 1 + holes`, face area equals `region.area` (to 1e-12 relative for
/// the shipped regions), `euler == 1 - holes`. Errors if a loop has fewer than three vertices or
/// a zero-length segment.
pub fn planar_face(region: &Region2, z: f64) -> Result<FaceFixture, FixtureError> {
    let mut t = CellComplex::new();
    let (face, vertices, edges) = run(&mut t, 1, |t| {
        let mut vertices: LoopIds = Vec::new();
        let mut edges: LoopIds = Vec::new();
        let mut loops = Vec::new();
        for l in region.loops() {
            if l.len() < 3 {
                return Err(bad("a loop needs at least three vertices"));
            }
            let pts: Vec<Point3> = l.iter().map(|p| Point3::new(p.0, p.1, z)).collect();
            let vs: Vec<EntityId> = pts.iter().map(|p| t.make_vertex(*p)).collect::<Result<_, _>>()?;
            let mut es = Vec::with_capacity(pts.len());
            for i in 0..pts.len() {
                let j = (i + 1) % pts.len();
                let d = pts[j] - pts[i];
                let curve = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(pts[i], d)?))?;
                es.push(t.make_edge(vs[i], vs[j], Some(CurveCarrier { curve, range: Interval::new(0.0, d.norm()), sense: Sense::Forward }))?);
            }
            loops.push(es.iter().map(|e| CoedgeSpec { edge: *e, sense: Sense::Forward }).collect::<Vec<_>>());
            vertices.push(vs);
            edges.push(es);
        }
        let surface = t.shapes_mut()?.add_surface(plane_z(z)?)?;
        let refs: Vec<&[CoedgeSpec]> = loops.iter().map(Vec::as_slice).collect();
        let face = t.make_face(&refs, Some(SurfaceCarrier { surface, sense: Sense::Forward }))?;
        Ok((face, vertices, edges))
    })?;
    Ok(FaceFixture { name: region.name, complex: t, face, vertices, edges, area: region.area, euler: 1 - region.holes.len() as i64 })
}

/// The planar annulus `r_in < r <= r_out` in the plane `z = 0` (normal `+z`): two circle edges
/// with one vertex each (`(r_out, 0, 0)` and `(r_in, 0, 0)`), the outer loop counter-clockwise
/// and the inner clockwise. Area `pi (r_out^2 - r_in^2)`, topological Euler 0.
pub fn annulus_face(r_in: f64, r_out: f64) -> Result<FaceFixture, FixtureError> {
    if !(r_in > 0.0 && r_out > r_in && r_out.is_finite()) {
        return Err(bad("annulus needs 0 < r_in < r_out"));
    }
    let mut t = CellComplex::new();
    let (face, vertices, edges) = run(&mut t, 1, |t| {
        let mut vs = Vec::new();
        let mut es = Vec::new();
        for r in [r_out, r_in] {
            let v = t.make_vertex(Point3::new(r, 0.0, 0.0))?;
            let frame = Frame::from_zx(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 0.0, 0.0))?;
            let curve = t.shapes_mut()?.add_curve(CurveDef::Circle(Circle::new(frame, r)?))?;
            es.push(t.make_edge(v, v, Some(CurveCarrier { curve, range: Interval::new(0.0, TAU), sense: Sense::Forward }))?);
            vs.push(v);
        }
        let outer = [CoedgeSpec { edge: es[0], sense: Sense::Forward }];
        let inner = [CoedgeSpec { edge: es[1], sense: Sense::Reversed }];
        let surface = t.shapes_mut()?.add_surface(plane_z(0.0)?)?;
        let face = t.make_face(&[&outer, &inner], Some(SurfaceCarrier { surface, sense: Sense::Forward }))?;
        Ok((face, vec![vec![vs[0]], vec![vs[1]]], vec![vec![es[0]], vec![es[1]]]))
    })?;
    Ok(FaceFixture { name: "annulus_face", complex: t, face, vertices, edges, area: PI * (r_out * r_out - r_in * r_in), euler: 0 })
}

/// A zone of the sphere of radius `r` about the origin: longitude `u = [u0, u1]` and latitude
/// `v = [v0, v1]` (radians), a four-edge face with `Line2` pcurves in the sphere's `(u, v)`
/// parametrization. The corners are `(u0, v0), (u1, v0), (u1, v1), (u0, v1)`; the edges are two
/// latitude arcs and two meridian arcs (ids in the order bottom, right, top, left), the normal
/// points outward. Area `r^2 (u1 - u0)(sin v1 - sin v0)`. Needs `0 < u1 - u0 < 2 pi` and
/// `-pi/2 < v0 < v1 < pi/2` (no pole, no seam).
pub fn sphere_zone(r: f64, u: [f64; 2], v: [f64; 2]) -> Result<FaceFixture, FixtureError> {
    let half = PI / 2.0;
    if !(r > 0.0 && u[1] > u[0] && u[1] - u[0] < TAU && v[0] > -half && v[1] < half && v[1] > v[0]) {
        return Err(bad("sphere zone needs r > 0, 0 < du < 2 pi, -pi/2 < v0 < v1 < pi/2"));
    }
    let mut t = CellComplex::new();
    let sph = |u: f64, v: f64| Point3::new(r * v.cos() * u.cos(), r * v.cos() * u.sin(), r * v.sin());
    let (face, vs, es) = run(&mut t, 1, |t| {
        let corners = [(u[0], v[0]), (u[1], v[0]), (u[1], v[1]), (u[0], v[1])];
        let vs: Vec<EntityId> = corners.iter().map(|c| t.make_vertex(sph(c.0, c.1))).collect::<Result<_, _>>()?;
        let x = Vector3::new(1.0, 0.0, 0.0);
        let z = Vector3::new(0.0, 0.0, 1.0);
        // Latitude circle: parameter = longitude. Meridian circle: parameter = latitude.
        let lat = |t: &mut CellComplex, v: f64| -> Result<EntityId, FixtureError> {
            let frame = Frame::from_zx(Point3::new(0.0, 0.0, r * v.sin()), z, x)?;
            Ok(t.shapes_mut()?.add_curve(CurveDef::Circle(Circle::new(frame, r * v.cos())?))?)
        };
        let mer = |t: &mut CellComplex, u: f64| -> Result<EntityId, FixtureError> {
            let frame = Frame::from_zx(Point3::new(0.0, 0.0, 0.0), Vector3::new(u.sin(), -u.cos(), 0.0), Vector3::new(u.cos(), u.sin(), 0.0))?;
            Ok(t.shapes_mut()?.add_curve(CurveDef::Circle(Circle::new(frame, r)?))?)
        };
        let (cb, ct, cr, cl) = (lat(t, v[0])?, lat(t, v[1])?, mer(t, u[1])?, mer(t, u[0])?);
        let ur = Interval::new(u[0], u[1]);
        let vr = Interval::new(v[0], v[1]);
        let carrier = |curve, range| Some(CurveCarrier { curve, range, sense: Sense::Forward });
        let e_bot = t.make_edge(vs[0], vs[1], carrier(cb, ur))?;
        let e_right = t.make_edge(vs[1], vs[2], carrier(cr, vr))?;
        let e_top = t.make_edge(vs[3], vs[2], carrier(ct, ur))?;
        let e_left = t.make_edge(vs[0], vs[3], carrier(cl, vr))?;
        let surface = t.shapes_mut()?.add_surface(SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), r)?))?;
        let lp = [
            CoedgeSpec { edge: e_bot, sense: Sense::Forward },
            CoedgeSpec { edge: e_right, sense: Sense::Forward },
            CoedgeSpec { edge: e_top, sense: Sense::Reversed },
            CoedgeSpec { edge: e_left, sense: Sense::Reversed },
        ];
        let face = t.make_face(&[&lp], Some(SurfaceCarrier { surface, sense: Sense::Forward }))?;
        let line2 = |o: (f64, f64), d: (f64, f64), range| PCurve::Line2(Line2 { origin: Uv::new(o.0, o.1), dir: Uv::new(d.0, d.1), range });
        let key = |edge, side| CoedgeKey { face, edge, side };
        t.set_pcurve(key(e_bot, Side::Left), Some(line2((0.0, v[0]), (1.0, 0.0), ur)))?;
        t.set_pcurve(key(e_right, Side::Left), Some(line2((u[1], 0.0), (0.0, 1.0), vr)))?;
        t.set_pcurve(key(e_top, Side::Right), Some(line2((0.0, v[1]), (1.0, 0.0), ur)))?;
        t.set_pcurve(key(e_left, Side::Right), Some(line2((u[0], 0.0), (0.0, 1.0), vr)))?;
        Ok((face, vs, [e_bot, e_right, e_top, e_left]))
    })?;
    Ok(FaceFixture {
        name: "sphere_zone",
        complex: t,
        face,
        vertices: vec![vs],
        edges: vec![es.to_vec()],
        area: r * r * (u[1] - u[0]) * (v[1].sin() - v[0].sin()),
        euler: 1,
    })
}
