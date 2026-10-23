//! Reference builders over any [`TopologyEdit`]: a brick and a cylinder. Used by tests here
//! and by downstream crates' tests. The caller opens the operation (`begin_op`) first.

use crate::data::*;
use crate::edit::*;
use crate::error::TopoError;
use crate::ids::*;
use core::f64::consts::TAU;
use sm_core::{Point3, Vector3};
use sm_geom_api::{Circle, CurveDef, Cylinder, Frame, GeomError, Interval, Line, Line2, PCurve, Plane, SimpleGeomStore, SurfaceDef};

/// Tolerance given to builder-made entities.
pub const BUILD_TOL: f64 = 1e-6;

/// Builder failure.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum BuildError {
    /// Topology edit failed.
    #[error(transparent)]
    Topo(#[from] TopoError),
    /// Geometry construction failed.
    #[error(transparent)]
    Geom(#[from] GeomError),
}

/// Ids of a brick. Vertex index is `x + 2y + 4z` (bits of the corner).
#[derive(Clone, Debug)]
pub struct BrickIds {
    /// The 8 corners.
    pub vertices: [VertexId; 8],
    /// The 12 edges: 4 along x, then 4 along y, then 4 along z (each from low to high end).
    pub edges: [EdgeId; 12],
    /// Faces in the order `-x, +x, -y, +y, -z, +z`.
    pub faces: [FaceId; 6],
    /// The volume.
    pub volume: VolumeId,
    /// The body (new unless one was passed in).
    pub body: BodyId,
}

const FACE_CYCLES: [[usize; 4]; 6] = [
    [0, 4, 6, 2], // -x : (0,0,0) (0,0,1) (0,1,1) (0,1,0)
    [1, 3, 7, 5], // +x : (1,0,0) (1,1,0) (1,1,1) (1,0,1)
    [0, 1, 5, 4], // -y
    [2, 6, 7, 3], // +y
    [0, 2, 3, 1], // -z
    [4, 5, 7, 6], // +z
];
const FACE_NORMALS: [[f64; 3]; 6] = [[-1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [0.0, 0.0, 1.0]];

/// Build an axis-aligned brick `[min, min + size]` with planar faces, line edges, one outer
/// shell (all uses `Forward`: outward normals). Creation order is canonical: vertices by
/// index, edges as documented on [`BrickIds::edges`], faces in the order of [`BrickIds::faces`].
pub fn make_brick<E: TopologyEdit>(ed: &mut E, geom: &mut SimpleGeomStore, min: Point3, size: Vector3, body: Option<BodyId>) -> Result<BrickIds, BuildError> {
    let corner = |i: usize| Point3::new(min.x + if i & 1 != 0 { size.x } else { 0.0 }, min.y + if i & 2 != 0 { size.y } else { 0.0 }, min.z + if i & 4 != 0 { size.z } else { 0.0 });
    let mut vs = Vec::with_capacity(8);
    for i in 0..8 {
        vs.push(ed.make_vertex(corner(i), BUILD_TOL)?);
    }
    let mut es: Vec<(usize, usize, EdgeId)> = Vec::with_capacity(12);
    for axis in 0..3 {
        let bit = 1usize << axis;
        for lo in (0..8).filter(|i| i & bit == 0) {
            let hi = lo | bit;
            let d = corner(hi) - corner(lo);
            let curve = geom.add_curve(CurveDef::Line(Line::new(corner(lo), d)?))?;
            let id = ed.make_edge(EdgeSpec { curve: Some(curve), range: Interval::new(0.0, d.norm()), start: vs[lo], end: vs[hi], tol: BUILD_TOL, degenerate: false })?;
            es.push((lo, hi, id));
        }
    }
    let mut faces = Vec::with_capacity(6);
    for (cycle, n) in FACE_CYCLES.iter().zip(FACE_NORMALS) {
        let p0 = corner(cycle[0]);
        let frame = Frame::from_zx(p0, Vector3::new(n[0], n[1], n[2]), corner(cycle[1]) - p0)?;
        let surface = geom.add_surface(SurfaceDef::Plane(Plane { frame }))?;
        let coedges = (0..4)
            .map(|i| {
                let (a, b) = (cycle[i], cycle[(i + 1) % 4]);
                let (_, _, id) = es.iter().find(|(lo, hi, _)| (*lo, *hi) == (a, b) || (*lo, *hi) == (b, a)).copied().ok_or(TopoError::Invalid("brick edge"))?;
                Ok(Coedge { edge: id, sense: if es.iter().any(|(lo, hi, _)| (*lo, *hi) == (a, b)) { Sense::Forward } else { Sense::Reversed }, pcurve: None })
            })
            .collect::<Result<Vec<_>, TopoError>>()?;
        faces.push(ed.make_face(FaceSpec { surface: Some(surface), surface_sense: Sense::Forward, tol: BUILD_TOL, loops: vec![LoopData { kind: LoopKind::Outer, coedges }], embedded_edges: vec![] })?);
    }
    let body = match body {
        Some(b) => b,
        None => ed.make_body()?,
    };
    let uses = faces.iter().map(|f| FaceUse { face: *f, sense: Sense::Forward }).collect();
    let volume = ed.make_volume(body, vec![ShellData { kind: ShellKind::Outer, uses }])?;
    let fail = || TopoError::Invalid("brick arity");
    Ok(BrickIds {
        vertices: vs.try_into().map_err(|_| fail())?,
        edges: es.iter().map(|e| e.2).collect::<Vec<_>>().try_into().map_err(|_| fail())?,
        faces: faces.try_into().map_err(|_| fail())?,
        volume,
        body,
    })
}

/// Ids of a cylinder.
#[derive(Clone, Debug)]
pub struct CylinderIds {
    /// `[bottom seam vertex, top seam vertex]`.
    pub vertices: [VertexId; 2],
    /// `[bottom circle, top circle, seam]`.
    pub edges: [EdgeId; 3],
    /// `[lateral, bottom cap, top cap]`.
    pub faces: [FaceId; 3],
    /// The volume.
    pub volume: VolumeId,
    /// The body.
    pub body: BodyId,
}

/// Build a solid cylinder of `radius` and `height` about the z axis from the origin: one seam
/// edge used twice (opposite senses, with pcurves at `u = 0` and `u = 2 pi`) by the lateral
/// face, two closed circular edges, two planar caps.
pub fn make_cylinder<E: TopologyEdit>(ed: &mut E, geom: &mut SimpleGeomStore, radius: f64, height: f64) -> Result<CylinderIds, BuildError> {
    let vb = ed.make_vertex(Point3::new(radius, 0.0, 0.0), BUILD_TOL)?;
    let vt = ed.make_vertex(Point3::new(radius, 0.0, height), BUILD_TOL)?;
    let top_frame = Frame { origin: Point3::new(0.0, 0.0, height), ..Frame::WORLD };
    let cb = geom.add_curve(CurveDef::Circle(Circle::new(Frame::WORLD, radius)?))?;
    let ct = geom.add_curve(CurveDef::Circle(Circle::new(top_frame, radius)?))?;
    let cs = geom.add_curve(CurveDef::Line(Line::new(Point3::new(radius, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0))?))?;
    let full = Interval::new(0.0, TAU);
    let eb = ed.make_edge(EdgeSpec { curve: Some(cb), range: full, start: vb, end: vb, tol: BUILD_TOL, degenerate: false })?;
    let et = ed.make_edge(EdgeSpec { curve: Some(ct), range: full, start: vt, end: vt, tol: BUILD_TOL, degenerate: false })?;
    let es = ed.make_edge(EdgeSpec { curve: Some(cs), range: Interval::new(0.0, height), start: vb, end: vt, tol: BUILD_TOL, degenerate: false })?;
    let line = |o: [f64; 2], d: [f64; 2]| Some(PCurve::Line(Line2 { origin: o, dir: d }));
    let lateral_surface = geom.add_surface(SurfaceDef::Cylinder(Cylinder::new(Frame::WORLD, radius)?))?;
    let lateral = ed.make_face(FaceSpec {
        surface: Some(lateral_surface),
        surface_sense: Sense::Forward,
        tol: BUILD_TOL,
        loops: vec![LoopData {
            kind: LoopKind::Outer,
            coedges: vec![
                Coedge { edge: eb, sense: Sense::Forward, pcurve: line([0.0, 0.0], [1.0, 0.0]) },
                Coedge { edge: es, sense: Sense::Forward, pcurve: line([TAU, 0.0], [0.0, 1.0]) },
                Coedge { edge: et, sense: Sense::Reversed, pcurve: line([0.0, height], [1.0, 0.0]) },
                Coedge { edge: es, sense: Sense::Reversed, pcurve: line([0.0, 0.0], [0.0, 1.0]) },
            ],
        }],
        embedded_edges: vec![],
    })?;
    let x = Vector3::new(1.0, 0.0, 0.0);
    let bottom_surface = geom.add_surface(SurfaceDef::Plane(Plane { frame: Frame::from_zx(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, -1.0), x)? }))?;
    let top_surface = geom.add_surface(SurfaceDef::Plane(Plane { frame: Frame::from_zx(Point3::new(0.0, 0.0, height), Vector3::new(0.0, 0.0, 1.0), x)? }))?;
    let cap = |surface, edge, sense| FaceSpec {
        surface: Some(surface),
        surface_sense: Sense::Forward,
        tol: BUILD_TOL,
        loops: vec![LoopData { kind: LoopKind::Outer, coedges: vec![Coedge { edge, sense, pcurve: None }] }],
        embedded_edges: vec![],
    };
    // Seen from below (normal -z) the circle's increasing-angle direction is clockwise: reversed.
    let bottom = ed.make_face(cap(bottom_surface, eb, Sense::Reversed))?;
    let top = ed.make_face(cap(top_surface, et, Sense::Forward))?;
    let body = ed.make_body()?;
    let uses = [lateral, bottom, top].iter().map(|f| FaceUse { face: *f, sense: Sense::Forward }).collect();
    let volume = ed.make_volume(body, vec![ShellData { kind: ShellKind::Outer, uses }])?;
    Ok(CylinderIds { vertices: [vb, vt], edges: [eb, et, es], faces: [lateral, bottom, top], volume, body })
}
