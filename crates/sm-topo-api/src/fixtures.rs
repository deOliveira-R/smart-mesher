//! Reference fixtures built only through [`TopologyEdit`]: brick, touching bricks (and their
//! merge), webcut brick, cylinder with a periodic seam, sphere with poles, brick with an internal
//! baffle, brick with a non-manifold fin.
//!
//! They double as executable documentation of the orientation rules: every face normal points out
//! of its volume, so the volume sits on the face's `Left` side. All builders assume an operation
//! is open (`begin_op`); [`with_op`] wraps that.

use crate::edit::TopologyEdit;
use crate::error::TopoError;
use crate::types::{CellFlags, CoedgeKey, CoedgeSpec, Coincidence, CurveCarrier, FaceCut, FaceUseSpec, InnerLoopAssign, OpId, Sense, ShellAssign, ShellKind, ShellSpec, Side, SurfaceCarrier};
use core::f64::consts::{FRAC_PI_2, TAU};
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{Circle, CurveDef, Cylinder, Frame, GeomError, Interval, Line, Line2, PCurve, Plane, Sphere, SurfaceDef, Uv};

/// A fixture could not be built.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum FixtureError {
    /// A topology edit failed.
    #[error(transparent)]
    Topo(#[from] TopoError),
    /// A shape could not be built.
    #[error(transparent)]
    Geom(#[from] GeomError),
}

/// Run `f` inside operation `op`: `begin_op`, then `f`, then `end_op` (or `abort_op` if `f`
/// failed).
pub fn with_op<E: TopologyEdit + ?Sized, T>(topo: &mut E, op: OpId, f: impl FnOnce(&mut E) -> Result<T, FixtureError>) -> Result<T, FixtureError> {
    topo.begin_op(op)?;
    match f(topo) {
        Ok(v) => {
            topo.end_op()?;
            Ok(v)
        }
        Err(e) => {
            topo.abort_op();
            Err(e)
        }
    }
}

fn add_curve<E: TopologyEdit + ?Sized>(topo: &mut E, def: CurveDef) -> Result<EntityId, FixtureError> {
    Ok(topo.shapes_mut().add_curve(def)?)
}

fn add_surface<E: TopologyEdit + ?Sized>(topo: &mut E, def: SurfaceDef) -> Result<EntityId, FixtureError> {
    Ok(topo.shapes_mut().add_surface(def)?)
}

fn line_edge<E: TopologyEdit + ?Sized>(topo: &mut E, a: (EntityId, Point3), b: (EntityId, Point3)) -> Result<EntityId, FixtureError> {
    let d = b.1 - a.1;
    let curve = add_curve(topo, CurveDef::Line(Line::new(a.1, d)?))?;
    Ok(topo.make_edge(a.0, b.0, Some(CurveCarrier { curve, range: Interval::new(0.0, d.norm()), sense: Sense::Forward }))?)
}

/// Ids of a brick. Vertex index is `i + 2j + 4k` for the corner at `(i, j, k)`.
#[derive(Clone, Debug, PartialEq)]
pub struct BrickIds {
    /// Minimum corner.
    pub min: Point3,
    /// Edge lengths.
    pub size: [f64; 3],
    /// Eight corners.
    pub vertices: [EntityId; 8],
    /// Edges: x-edges `j + 2k`, then y-edges `4 + i + 2k`, then z-edges `8 + i + 2j`.
    pub edges: [EntityId; 12],
    /// Faces in the order `-x, +x, -y, +y, -z, +z`.
    pub faces: [EntityId; 6],
    /// The volume (`EntityId::from_raw(0)`-style placeholder is never used: bricks always have one).
    pub volume: EntityId,
    /// The body.
    pub body: EntityId,
}

const CORNER_CYCLES: [[usize; 4]; 6] = [[0, 4, 6, 2], [1, 3, 7, 5], [0, 1, 5, 4], [2, 6, 7, 3], [0, 2, 3, 1], [4, 5, 7, 6]];
const NORMALS: [[f64; 3]; 6] = [[-1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [0.0, 0.0, 1.0]];

struct BrickParts {
    vertices: [EntityId; 8],
    edges: [EntityId; 12],
    faces: [EntityId; 6],
}

fn arr<const N: usize>(v: Vec<EntityId>) -> Result<[EntityId; N], FixtureError> {
    v.try_into().map_err(|_| FixtureError::Topo(TopoError::IdExhausted))
}

fn brick_parts<E: TopologyEdit + ?Sized>(topo: &mut E, min: Point3, size: [f64; 3]) -> Result<BrickParts, FixtureError> {
    let corner = |idx: usize| Point3::new(min.x + (idx & 1) as f64 * size[0], min.y + ((idx >> 1) & 1) as f64 * size[1], min.z + ((idx >> 2) & 1) as f64 * size[2]);
    let vs: Vec<EntityId> = (0..8).map(|idx| topo.make_vertex(corner(idx))).collect::<Result<_, _>>()?;
    let vertices: [EntityId; 8] = arr(vs)?;
    // (start corner, end corner, axis)
    let mut spec: Vec<(usize, usize, usize)> = Vec::with_capacity(12);
    for k in 0..2 {
        for j in 0..2 {
            spec.push((2 * j + 4 * k, 1 + 2 * j + 4 * k, 0));
        }
    }
    for k in 0..2 {
        for i in 0..2 {
            spec.push((i + 4 * k, i + 2 + 4 * k, 1));
        }
    }
    for j in 0..2 {
        for i in 0..2 {
            spec.push((i + 2 * j, i + 2 * j + 4, 2));
        }
    }
    let mut edge_vec = Vec::with_capacity(12);
    for (a, b, axis) in &spec {
        let mut d = [0.0; 3];
        d[*axis] = 1.0;
        let curve = add_curve(topo, CurveDef::Line(Line::new(corner(*a), Vector3::new(d[0], d[1], d[2]))?))?;
        edge_vec.push(topo.make_edge(vertices[*a], vertices[*b], Some(CurveCarrier { curve, range: Interval::new(0.0, size[*axis]), sense: Sense::Forward }))?);
    }
    let edges: [EntityId; 12] = arr(edge_vec)?;
    let coedge = |a: usize, b: usize| -> CoedgeSpec {
        match spec.iter().position(|s| s.0 == a && s.1 == b) {
            Some(n) => CoedgeSpec { edge: edges[n], sense: Sense::Forward },
            None => {
                let n = spec.iter().position(|s| s.0 == b && s.1 == a).unwrap_or(0);
                CoedgeSpec { edge: edges[n], sense: Sense::Reversed }
            }
        }
    };
    let mut face_vec = Vec::with_capacity(6);
    for f in 0..6 {
        let cyc = CORNER_CYCLES[f];
        let l: Vec<CoedgeSpec> = (0..4).map(|i| coedge(cyc[i], cyc[(i + 1) % 4])).collect();
        let n = NORMALS[f];
        let frame = Frame::from_z(corner(cyc[0]), Vector3::new(n[0], n[1], n[2]))?;
        let surface = add_surface(topo, SurfaceDef::Plane(Plane { frame }))?;
        face_vec.push(topo.make_face(&[&l], Some(SurfaceCarrier { surface, sense: Sense::Forward }))?);
    }
    Ok(BrickParts { vertices, edges, faces: arr(face_vec)? })
}

/// Build an axis-aligned brick `[min, min + size]` (one op must be open).
pub fn brick<E: TopologyEdit + ?Sized>(topo: &mut E, min: Point3, size: [f64; 3]) -> Result<BrickIds, FixtureError> {
    let p = brick_parts(topo, min, size)?;
    let uses: Vec<FaceUseSpec> = p.faces.iter().map(|&face| FaceUseSpec { face, side: Side::Left }).collect();
    let volume = topo.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &uses }])?;
    let body = topo.make_body(&[volume])?;
    Ok(BrickIds { min, size, vertices: p.vertices, edges: p.edges, faces: p.faces, volume, body })
}

/// Two unit bricks `[0,1]^3` and `[1,2] x [0,1]^2` that touch on the plane `x = 1` but share
/// nothing yet. Their coincident faces (A `+x`, B `-x`) are recorded `CoincidentUnimprinted`.
pub fn two_bricks<E: TopologyEdit + ?Sized>(topo: &mut E) -> Result<(BrickIds, BrickIds), FixtureError> {
    let a = brick(topo, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])?;
    let b = brick(topo, Point3::new(1.0, 0.0, 0.0), [1.0, 1.0, 1.0])?;
    topo.record_coincidence(a.faces[1], b.faces[0], Coincidence::CoincidentUnimprinted)?;
    Ok((a, b))
}

/// Merge the touching face of `b` (`-x`) into that of `a` (`+x`): vertices, then edges, then the
/// face, always keeping the lower id (so `a` must have been built first). The pair goes
/// `CoincidentUnimprinted -> ImprintedNotMerged -> Merged`.
pub fn merge_touching_bricks<E: TopologyEdit + ?Sized>(topo: &mut E, a: &BrickIds, b: &BrickIds) -> Result<(), FixtureError> {
    merge_touching_bricks_gap(topo, a, b, None)
}

/// [`merge_touching_bricks`] with an op-level merge tolerance `max_gap` for the vertex merges.
pub fn merge_touching_bricks_gap<E: TopologyEdit + ?Sized>(topo: &mut E, a: &BrickIds, b: &BrickIds, max_gap: Option<f64>) -> Result<(), FixtureError> {
    for j in 0..2 {
        for k in 0..2 {
            topo.merge_vertices(a.vertices[1 + 2 * j + 4 * k], b.vertices[2 * j + 4 * k], max_gap)?;
        }
    }
    for k in 0..2 {
        topo.merge_edges(a.edges[5 + 2 * k], b.edges[4 + 2 * k], Sense::Forward)?;
    }
    for j in 0..2 {
        topo.merge_edges(a.edges[9 + 2 * j], b.edges[8 + 2 * j], Sense::Forward)?;
    }
    topo.record_coincidence(a.faces[1], b.faces[0], Coincidence::ImprintedNotMerged)?;
    topo.merge_faces(a.faces[1], b.faces[0], Sense::Reversed)?;
    Ok(())
}

/// Ids produced by [`webcut_brick`].
#[derive(Clone, Debug, PartialEq)]
pub struct WebcutIds {
    /// The four new vertices on the vertical edges (`i + 2j`).
    pub vertices: [EntityId; 4],
    /// The halves of the four vertical edges, each `[below, above]` (index `i + 2j`).
    pub split_edges: [[EntityId; 2]; 4],
    /// The four chord edges: `-x`, `+x`, `-y`, `+y` side.
    pub chords: [EntityId; 4],
    /// The side faces `-x, +x, -y, +y` split by the cut: `[below, above]` pieces.
    pub split_faces: [[EntityId; 2]; 4],
    /// The cap face generated on the cutting plane.
    pub cap: EntityId,
    /// The plane surface of the cut.
    pub plane: EntityId,
    /// The two volumes `[first, second]`.
    pub volumes: [EntityId; 2],
}

/// Cut a brick built by [`brick`] with the plane `z = min.z + frac * size.z` (webcut): the four
/// vertical edges are split, the four side faces are split by chords, the cap is created on the
/// plane (`record_generated`), and the volume is split in two.
pub fn webcut_brick<E: TopologyEdit + ?Sized>(topo: &mut E, b: &BrickIds, frac: f64) -> Result<WebcutIds, FixtureError> {
    let z = b.min.z + frac * b.size[2];
    let corner_xy = |k: usize| (b.min.x + (k & 1) as f64 * b.size[0], b.min.y + ((k >> 1) & 1) as f64 * b.size[1]);
    let mut vertices = Vec::new();
    let mut split_edges = Vec::new();
    let mut pts = Vec::new();
    for k in 0..4 {
        let (x, y) = corner_xy(k);
        let p = Point3::new(x, y, z);
        let m = topo.make_vertex(p)?;
        split_edges.push(topo.split_edge(b.edges[8 + k], m, z - b.min.z)?);
        vertices.push(m);
        pts.push(p);
    }
    let vertices: [EntityId; 4] = arr(vertices)?;
    let split_edges: [[EntityId; 2]; 4] = split_edges.try_into().map_err(|_| FixtureError::Topo(TopoError::IdExhausted))?;
    let pt = |k: usize| (vertices[k], pts[k]);
    // Chords: -x: m0->m2, +x: m1->m3, -y: m0->m1, +y: m2->m3.
    let c_mx = line_edge(topo, pt(0), pt(2))?;
    let c_px = line_edge(topo, pt(1), pt(3))?;
    let c_my = line_edge(topo, pt(0), pt(1))?;
    let c_py = line_edge(topo, pt(2), pt(3))?;
    let chords = [c_mx, c_px, c_my, c_py];
    let mut split_faces = Vec::new();
    for (face, chord) in [(b.faces[0], c_mx), (b.faces[1], c_px), (b.faces[2], c_my), (b.faces[3], c_py)] {
        split_faces.push(topo.split_face(face, FaceCut::Chord { edge: chord }, InnerLoopAssign::None)?);
    }
    let split_faces: [[EntityId; 2]; 4] = split_faces.try_into().map_err(|_| FixtureError::Topo(TopoError::IdExhausted))?;
    let plane = add_surface(topo, SurfaceDef::Plane(Plane { frame: Frame::from_z(Point3::new(b.min.x, b.min.y, z), Vector3::new(0.0, 0.0, 1.0))? }))?;
    let lp = [
        CoedgeSpec { edge: c_my, sense: Sense::Forward },
        CoedgeSpec { edge: c_px, sense: Sense::Forward },
        CoedgeSpec { edge: c_py, sense: Sense::Reversed },
        CoedgeSpec { edge: c_mx, sense: Sense::Reversed },
    ];
    let cap = topo.make_face(&[&lp], Some(SurfaceCarrier { surface: plane, sense: Sense::Forward }))?;
    let volumes = topo.split_volume(b.volume, &[cap], ShellAssign::None)?;
    topo.record_generated(cap, plane)?;
    Ok(WebcutIds { vertices, split_edges, chords, split_faces, cap, plane, volumes })
}

/// Ids of the cylinder fixture.
#[derive(Clone, Debug, PartialEq)]
pub struct CylinderIds {
    /// `[bottom, top]` seam-circle vertices (both at angle 0).
    pub vertices: [EntityId; 2],
    /// `[bottom circle, top circle, seam line]`.
    pub edges: [EntityId; 3],
    /// `[bottom cap, top cap, lateral]`.
    pub faces: [EntityId; 3],
    /// The volume.
    pub volume: EntityId,
    /// The body.
    pub body: EntityId,
    /// Handle of the cylinder surface.
    pub lateral_surface: EntityId,
}

/// A solid cylinder of radius `r` and height `h` about the z axis from `z = 0`: 2 vertices,
/// 3 edges (two closed circles and a seam used twice by the lateral face), 3 faces. The lateral
/// face carries pcurves in `(u, v) = (angle, z)`: the seam is `u = 2 pi` where it is traversed
/// forward (`Left`) and `u = 0` where it is traversed backward (`Right`).
pub fn cylinder<E: TopologyEdit + ?Sized>(topo: &mut E, r: f64, h: f64) -> Result<CylinderIds, FixtureError> {
    let vb = topo.make_vertex(Point3::new(r, 0.0, 0.0))?;
    let vt = topo.make_vertex(Point3::new(r, 0.0, h))?;
    let x = Vector3::new(1.0, 0.0, 0.0);
    let z = Vector3::new(0.0, 0.0, 1.0);
    let bottom_frame = Frame::from_zx(Point3::new(0.0, 0.0, 0.0), z, x)?;
    let top_frame = Frame::from_zx(Point3::new(0.0, 0.0, h), z, x)?;
    let cb = add_curve(topo, CurveDef::Circle(Circle::new(bottom_frame, r)?))?;
    let ct = add_curve(topo, CurveDef::Circle(Circle::new(top_frame, r)?))?;
    let cs = add_curve(topo, CurveDef::Line(Line::new(Point3::new(r, 0.0, 0.0), z)?))?;
    let full = Interval::new(0.0, TAU);
    let e_bot = topo.make_edge(vb, vb, Some(CurveCarrier { curve: cb, range: full, sense: Sense::Forward }))?;
    let e_top = topo.make_edge(vt, vt, Some(CurveCarrier { curve: ct, range: full, sense: Sense::Forward }))?;
    let e_seam = topo.make_edge(vb, vt, Some(CurveCarrier { curve: cs, range: Interval::new(0.0, h), sense: Sense::Forward }))?;
    let s_bot = add_surface(topo, SurfaceDef::Plane(Plane { frame: Frame::from_zx(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, -1.0), x)? }))?;
    let s_top = add_surface(topo, SurfaceDef::Plane(Plane { frame: top_frame }))?;
    let s_lat = add_surface(topo, SurfaceDef::Cylinder(Cylinder::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), r)?))?;
    let f_bot = topo.make_face(&[&[CoedgeSpec { edge: e_bot, sense: Sense::Reversed }]], Some(SurfaceCarrier { surface: s_bot, sense: Sense::Forward }))?;
    let f_top = topo.make_face(&[&[CoedgeSpec { edge: e_top, sense: Sense::Forward }]], Some(SurfaceCarrier { surface: s_top, sense: Sense::Forward }))?;
    let lateral = [
        CoedgeSpec { edge: e_bot, sense: Sense::Forward },
        CoedgeSpec { edge: e_seam, sense: Sense::Forward },
        CoedgeSpec { edge: e_top, sense: Sense::Reversed },
        CoedgeSpec { edge: e_seam, sense: Sense::Reversed },
    ];
    let f_lat = topo.make_face(&[&lateral], Some(SurfaceCarrier { surface: s_lat, sense: Sense::Forward }))?;
    let line2 = |origin: (f64, f64), dir: (f64, f64), range: Interval| PCurve::Line2(Line2 { origin: Uv::new(origin.0, origin.1), dir: Uv::new(dir.0, dir.1), range });
    let key = |edge, side| CoedgeKey { face: f_lat, edge, side };
    topo.set_pcurve(key(e_bot, Side::Left), Some(line2((0.0, 0.0), (1.0, 0.0), full)))?;
    topo.set_pcurve(key(e_seam, Side::Left), Some(line2((TAU, 0.0), (0.0, 1.0), Interval::new(0.0, h))))?;
    topo.set_pcurve(key(e_top, Side::Right), Some(line2((0.0, h), (1.0, 0.0), full)))?;
    topo.set_pcurve(key(e_seam, Side::Right), Some(line2((0.0, 0.0), (0.0, 1.0), Interval::new(0.0, h))))?;
    let uses = [f_bot, f_top, f_lat].map(|face| FaceUseSpec { face, side: Side::Left });
    let volume = topo.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &uses }])?;
    let body = topo.make_body(&[volume])?;
    Ok(CylinderIds { vertices: [vb, vt], edges: [e_bot, e_top, e_seam], faces: [f_bot, f_top, f_lat], volume, body, lateral_surface: s_lat })
}

/// Ids of the sphere fixture.
#[derive(Clone, Debug, PartialEq)]
pub struct SphereIds {
    /// `[south pole, north pole]`.
    pub vertices: [EntityId; 2],
    /// `[south pole edge, meridian seam, north pole edge]`; the pole edges are degenerate.
    pub edges: [EntityId; 3],
    /// The single face.
    pub face: EntityId,
    /// The volume.
    pub volume: EntityId,
    /// The body.
    pub body: EntityId,
    /// Handle of the sphere surface.
    pub surface: EntityId,
}

/// A solid sphere of radius `r` about the origin: 2 vertices (the poles), a meridian seam edge
/// used twice, and two degenerate closed pole edges (flag `DEGENERATE`, no curve). The face loop
/// is `[south pole, seam, north pole (reversed), seam (reversed)]`, i.e. the `(u, v)` rectangle.
pub fn sphere<E: TopologyEdit + ?Sized>(topo: &mut E, r: f64) -> Result<SphereIds, FixtureError> {
    let vs = topo.make_vertex(Point3::new(0.0, 0.0, -r))?;
    let vn = topo.make_vertex(Point3::new(0.0, 0.0, r))?;
    let frame = Frame::from_zx(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, -1.0, 0.0), Vector3::new(1.0, 0.0, 0.0))?;
    let meridian = add_curve(topo, CurveDef::Circle(Circle::new(frame, r)?))?;
    let seam = topo.make_edge(vs, vn, Some(CurveCarrier { curve: meridian, range: Interval::new(-FRAC_PI_2, FRAC_PI_2), sense: Sense::Forward }))?;
    let pole_s = topo.make_edge(vs, vs, None)?;
    let pole_n = topo.make_edge(vn, vn, None)?;
    topo.set_flags(pole_s, CellFlags::DEGENERATE)?;
    topo.set_flags(pole_n, CellFlags::DEGENERATE)?;
    let surface = add_surface(topo, SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), r)?))?;
    let lp = [
        CoedgeSpec { edge: pole_s, sense: Sense::Forward },
        CoedgeSpec { edge: seam, sense: Sense::Forward },
        CoedgeSpec { edge: pole_n, sense: Sense::Reversed },
        CoedgeSpec { edge: seam, sense: Sense::Reversed },
    ];
    let face = topo.make_face(&[&lp], Some(SurfaceCarrier { surface, sense: Sense::Forward }))?;
    let line2 = |origin: (f64, f64), dir: (f64, f64), range: Interval| PCurve::Line2(Line2 { origin: Uv::new(origin.0, origin.1), dir: Uv::new(dir.0, dir.1), range });
    let key = |edge, side| CoedgeKey { face, edge, side };
    topo.set_pcurve(key(pole_s, Side::Left), Some(line2((0.0, -FRAC_PI_2), (1.0, 0.0), Interval::new(0.0, TAU))))?;
    topo.set_pcurve(key(seam, Side::Left), Some(line2((TAU, 0.0), (0.0, 1.0), Interval::new(-FRAC_PI_2, FRAC_PI_2))))?;
    topo.set_pcurve(key(pole_n, Side::Right), Some(line2((0.0, FRAC_PI_2), (1.0, 0.0), Interval::new(0.0, TAU))))?;
    topo.set_pcurve(key(seam, Side::Right), Some(line2((0.0, 0.0), (0.0, 1.0), Interval::new(-FRAC_PI_2, FRAC_PI_2))))?;
    let uses = [FaceUseSpec { face, side: Side::Left }];
    let volume = topo.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &uses }])?;
    let body = topo.make_body(&[volume])?;
    Ok(SphereIds { vertices: [vs, vn], edges: [pole_s, seam, pole_n], face, volume, body, surface })
}

/// Ids of the baffle added by [`brick_with_baffle`].
#[derive(Clone, Debug, PartialEq)]
pub struct BaffleIds {
    /// The four corners.
    pub vertices: [EntityId; 4],
    /// The four edges.
    pub edges: [EntityId; 4],
    /// The baffle face (normal `+x`).
    pub face: EntityId,
}

/// A unit brick with a free-standing square sheet inside it at `x = 0.5`, registered as a
/// baffle (`Side::Both`) in an `Internal` shell (ring 1) of the brick's volume.
pub fn brick_with_baffle<E: TopologyEdit + ?Sized>(topo: &mut E) -> Result<(BrickIds, BaffleIds), FixtureError> {
    let min = Point3::new(0.0, 0.0, 0.0);
    let p = brick_parts(topo, min, [1.0, 1.0, 1.0])?;
    let pts = [(0.25, 0.25), (0.75, 0.25), (0.75, 0.75), (0.25, 0.75)];
    let vv: Vec<EntityId> = pts.iter().map(|&(y, z)| topo.make_vertex(Point3::new(0.5, y, z))).collect::<Result<_, _>>()?;
    let vs: [EntityId; 4] = arr(vv)?;
    let mut ev = Vec::with_capacity(4);
    for i in 0..4 {
        let (a, b) = (pts[i], pts[(i + 1) % 4]);
        ev.push(line_edge(topo, (vs[i], Point3::new(0.5, a.0, a.1)), (vs[(i + 1) % 4], Point3::new(0.5, b.0, b.1)))?);
    }
    let es: [EntityId; 4] = arr(ev)?;
    let l: Vec<CoedgeSpec> = es.iter().map(|&edge| CoedgeSpec { edge, sense: Sense::Forward }).collect();
    let surface = add_surface(topo, SurfaceDef::Plane(Plane { frame: Frame::from_z(Point3::new(0.5, 0.25, 0.25), Vector3::new(1.0, 0.0, 0.0))? }))?;
    let face = topo.make_face(&[&l], Some(SurfaceCarrier { surface, sense: Sense::Forward }))?;
    let outer: Vec<FaceUseSpec> = p.faces.iter().map(|&face| FaceUseSpec { face, side: Side::Left }).collect();
    let baffle = [FaceUseSpec { face, side: Side::Both }];
    let volume = topo.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &outer }, ShellSpec { kind: ShellKind::Internal, uses: &baffle }])?;
    let body = topo.make_body(&[volume])?;
    Ok((BrickIds { min, size: [1.0; 3], vertices: p.vertices, edges: p.edges, faces: p.faces, volume, body }, BaffleIds { vertices: vs, edges: es, face }))
}

/// Ids of the fin added by [`brick_with_fin`].
#[derive(Clone, Debug, PartialEq)]
pub struct FinIds {
    /// The two outer vertices `(-1,-1,0)` and `(-1,-1,1)`.
    pub vertices: [EntityId; 2],
    /// The three new edges (top, far, bottom).
    pub edges: [EntityId; 3],
    /// The fin face. It shares the brick's edge `edges[8]` (the z-edge at the origin corner), so
    /// that edge has three faces around it: `-x`, `-y` and the fin.
    pub face: EntityId,
}

/// A unit brick plus a free sheet attached along the brick's z-edge at the origin corner and
/// extending outward to `(-1, -1)`: a non-manifold edge with three faces around it.
pub fn brick_with_fin<E: TopologyEdit + ?Sized>(topo: &mut E) -> Result<(BrickIds, FinIds), FixtureError> {
    let b = brick(topo, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])?;
    let w0 = topo.make_vertex(Point3::new(-1.0, -1.0, 0.0))?;
    let w4 = topo.make_vertex(Point3::new(-1.0, -1.0, 1.0))?;
    let p = |x: f64, y: f64, z: f64| Point3::new(x, y, z);
    let top = line_edge(topo, (b.vertices[4], p(0.0, 0.0, 1.0)), (w4, p(-1.0, -1.0, 1.0)))?;
    let far = line_edge(topo, (w4, p(-1.0, -1.0, 1.0)), (w0, p(-1.0, -1.0, 0.0)))?;
    let bottom = line_edge(topo, (w0, p(-1.0, -1.0, 0.0)), (b.vertices[0], p(0.0, 0.0, 0.0)))?;
    let lp = [
        CoedgeSpec { edge: b.edges[8], sense: Sense::Forward },
        CoedgeSpec { edge: top, sense: Sense::Forward },
        CoedgeSpec { edge: far, sense: Sense::Forward },
        CoedgeSpec { edge: bottom, sense: Sense::Forward },
    ];
    let surface = add_surface(topo, SurfaceDef::Plane(Plane { frame: Frame::from_z(p(0.0, 0.0, 0.0), Vector3::new(1.0, -1.0, 0.0))? }))?;
    let face = topo.make_face(&[&lp], Some(SurfaceCarrier { surface, sense: Sense::Forward }))?;
    Ok((b, FinIds { vertices: [w0, w4], edges: [top, far, bottom], face }))
}
