//! Reference fixtures built only through [`TopologyEdit`]: brick, touching bricks (and their
//! merge), cylinder with a periodic seam, brick with an internal baffle.
//!
//! They double as executable documentation of the orientation rules: every face normal points out
//! of its volume, so the volume sits on the face's `Left` side.

use crate::edit::TopologyEdit;
use crate::error::TopoError;
use crate::types::{CoedgeSpec, Coincidence, CurveCarrier, FaceUseSpec, Sense, Side, SurfaceCarrier};
use core::f64::consts::TAU;
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{Circle, Cylinder, Frame, GeomError, Interval, Line, MemoryStore, Plane};

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

/// Ids of a brick. Vertex index is `i + 2j + 4k` for the corner at `(i, j, k)`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BrickIds {
    /// Eight corners.
    pub vertices: [EntityId; 8],
    /// Edges: x-edges `j + 2k`, then y-edges `4 + i + 2k`, then z-edges `8 + i + 2j`.
    pub edges: [EntityId; 12],
    /// Faces in the order `-x, +x, -y, +y, -z, +z`.
    pub faces: [EntityId; 6],
    /// The volume.
    pub volume: EntityId,
    /// The body.
    pub body: EntityId,
}

const CORNER_CYCLES: [[usize; 4]; 6] = [[0, 4, 6, 2], [1, 3, 7, 5], [0, 1, 5, 4], [2, 6, 7, 3], [0, 2, 3, 1], [4, 5, 7, 6]];
const NORMALS: [[f64; 3]; 6] = [[-1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [0.0, 0.0, 1.0]];

/// Build an axis-aligned brick `[min, min + size]`.
pub fn brick<E: TopologyEdit + ?Sized>(topo: &mut E, store: &mut MemoryStore, min: Point3, size: [f64; 3]) -> Result<BrickIds, FixtureError> {
    let corner = |idx: usize| Point3::new(min.x + (idx & 1) as f64 * size[0], min.y + ((idx >> 1) & 1) as f64 * size[1], min.z + ((idx >> 2) & 1) as f64 * size[2]);
    let vs: Vec<EntityId> = (0..8).map(|idx| topo.make_vertex(corner(idx))).collect::<Result<_, _>>()?;
    let vertices: [EntityId; 8] = vs.try_into().map_err(|_| TopoError::IdExhausted)?;
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
        let curve = store.add_curve(Box::new(Line::new(corner(*a), Vector3::new(d[0], d[1], d[2]))?)).map_err(FixtureError::from)?;
        edge_vec.push(topo.make_edge(vertices[*a], vertices[*b], Some(CurveCarrier { curve, range: Interval::new(0.0, size[*axis]), sense: Sense::Forward }))?);
    }
    let edges: [EntityId; 12] = edge_vec.try_into().map_err(|_| TopoError::IdExhausted)?;
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
        let surface = store.add_surface(Box::new(Plane { frame })).map_err(FixtureError::from)?;
        face_vec.push(topo.make_face(&[&l], Some(SurfaceCarrier { surface, sense: Sense::Forward }))?);
    }
    let faces: [EntityId; 6] = face_vec.try_into().map_err(|_| TopoError::IdExhausted)?;
    let uses: Vec<FaceUseSpec> = faces.iter().map(|&face| FaceUseSpec { face, side: Side::Left }).collect();
    let volume = topo.make_volume(&[&uses])?;
    let body = topo.make_body(&[volume])?;
    Ok(BrickIds { vertices, edges, faces, volume, body })
}

/// Two unit bricks `[0,1]^3` and `[1,2] x [0,1]^2` that touch on the plane `x = 1` but share
/// nothing yet. Their coincident faces (A `+x`, B `-x`) are recorded `CoincidentUnimprinted`.
pub fn two_bricks<E: TopologyEdit + ?Sized>(topo: &mut E, store: &mut MemoryStore) -> Result<(BrickIds, BrickIds), FixtureError> {
    let a = brick(topo, store, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])?;
    let b = brick(topo, store, Point3::new(1.0, 0.0, 0.0), [1.0, 1.0, 1.0])?;
    topo.record_coincidence(a.faces[1], b.faces[0], Coincidence::CoincidentUnimprinted, None)?;
    Ok((a, b))
}

/// Merge the touching face of `b` (`-x`) into that of `a` (`+x`): vertices, then edges, then the
/// face. The pair goes `CoincidentUnimprinted -> ImprintedNotMerged -> Merged`.
pub fn merge_touching_bricks<E: TopologyEdit + ?Sized>(topo: &mut E, a: &BrickIds, b: &BrickIds) -> Result<(), FixtureError> {
    for j in 0..2 {
        for k in 0..2 {
            topo.merge_vertices(a.vertices[1 + 2 * j + 4 * k], b.vertices[2 * j + 4 * k])?;
        }
    }
    for k in 0..2 {
        topo.merge_edges(a.edges[5 + 2 * k], b.edges[4 + 2 * k], Sense::Forward)?;
    }
    for j in 0..2 {
        topo.merge_edges(a.edges[9 + 2 * j], b.edges[8 + 2 * j], Sense::Forward)?;
    }
    topo.record_coincidence(a.faces[1], b.faces[0], Coincidence::ImprintedNotMerged, None)?;
    topo.merge_faces(a.faces[1], b.faces[0], Sense::Reversed)?;
    Ok(())
}

/// Ids of the cylinder fixture.
#[derive(Clone, Debug, PartialEq, Eq)]
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
/// 3 edges (two closed circles and a seam used twice by the lateral face), 3 faces.
pub fn cylinder<E: TopologyEdit + ?Sized>(topo: &mut E, store: &mut MemoryStore, r: f64, h: f64) -> Result<CylinderIds, FixtureError> {
    let vb = topo.make_vertex(Point3::new(r, 0.0, 0.0))?;
    let vt = topo.make_vertex(Point3::new(r, 0.0, h))?;
    let x = Vector3::new(1.0, 0.0, 0.0);
    let z = Vector3::new(0.0, 0.0, 1.0);
    let bottom_frame = Frame::from_zx(Point3::new(0.0, 0.0, 0.0), z, x)?;
    let top_frame = Frame::from_zx(Point3::new(0.0, 0.0, h), z, x)?;
    let cb = store.add_curve(Box::new(Circle::new(bottom_frame, r)?)).map_err(FixtureError::from)?;
    let ct = store.add_curve(Box::new(Circle::new(top_frame, r)?)).map_err(FixtureError::from)?;
    let cs = store.add_curve(Box::new(Line::new(Point3::new(r, 0.0, 0.0), z)?)).map_err(FixtureError::from)?;
    let full = Interval::new(0.0, TAU);
    let e_bot = topo.make_edge(vb, vb, Some(CurveCarrier { curve: cb, range: full, sense: Sense::Forward }))?;
    let e_top = topo.make_edge(vt, vt, Some(CurveCarrier { curve: ct, range: full, sense: Sense::Forward }))?;
    let e_seam = topo.make_edge(vb, vt, Some(CurveCarrier { curve: cs, range: Interval::new(0.0, h), sense: Sense::Forward }))?;
    let s_bot = store.add_surface(Box::new(Plane { frame: Frame::from_zx(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, -1.0), x)? })).map_err(FixtureError::from)?;
    let s_top = store.add_surface(Box::new(Plane { frame: top_frame })).map_err(FixtureError::from)?;
    let s_lat = store.add_surface(Box::new(Cylinder::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), r)?)).map_err(FixtureError::from)?;
    let f_bot = topo.make_face(&[&[CoedgeSpec { edge: e_bot, sense: Sense::Reversed }]], Some(SurfaceCarrier { surface: s_bot, sense: Sense::Forward }))?;
    let f_top = topo.make_face(&[&[CoedgeSpec { edge: e_top, sense: Sense::Forward }]], Some(SurfaceCarrier { surface: s_top, sense: Sense::Forward }))?;
    let lateral = [
        CoedgeSpec { edge: e_bot, sense: Sense::Forward },
        CoedgeSpec { edge: e_seam, sense: Sense::Forward },
        CoedgeSpec { edge: e_top, sense: Sense::Reversed },
        CoedgeSpec { edge: e_seam, sense: Sense::Reversed },
    ];
    let f_lat = topo.make_face(&[&lateral], Some(SurfaceCarrier { surface: s_lat, sense: Sense::Forward }))?;
    let uses = [f_bot, f_top, f_lat].map(|face| FaceUseSpec { face, side: Side::Left });
    let volume = topo.make_volume(&[&uses])?;
    let body = topo.make_body(&[volume])?;
    Ok(CylinderIds { vertices: [vb, vt], edges: [e_bot, e_top, e_seam], faces: [f_bot, f_top, f_lat], volume, body, lateral_surface: s_lat })
}

/// Ids of the baffle added by [`brick_with_baffle`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BaffleIds {
    /// The four corners.
    pub vertices: [EntityId; 4],
    /// The four edges.
    pub edges: [EntityId; 4],
    /// The baffle face (normal `+x`).
    pub face: EntityId,
}

/// A unit brick with a free-standing square sheet inside it at `x = 0.5`, registered as a
/// baffle (`Side::Both`) of the brick's volume.
pub fn brick_with_baffle<E: TopologyEdit + ?Sized>(topo: &mut E, store: &mut MemoryStore) -> Result<(BrickIds, BaffleIds), FixtureError> {
    let b = brick(topo, store, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])?;
    let pts = [(0.25, 0.25), (0.75, 0.25), (0.75, 0.75), (0.25, 0.75)];
    let vv: Vec<EntityId> = pts.iter().map(|&(y, z)| topo.make_vertex(Point3::new(0.5, y, z))).collect::<Result<_, _>>()?;
    let vs: [EntityId; 4] = vv.try_into().map_err(|_| TopoError::IdExhausted)?;
    let mut ev = Vec::with_capacity(4);
    for i in 0..4 {
        let (p, q) = (pts[i], pts[(i + 1) % 4]);
        let dir = Vector3::new(0.0, q.0 - p.0, q.1 - p.1);
        let len = dir.norm();
        let curve = store.add_curve(Box::new(Line::new(Point3::new(0.5, p.0, p.1), dir)?)).map_err(FixtureError::from)?;
        ev.push(topo.make_edge(vs[i], vs[(i + 1) % 4], Some(CurveCarrier { curve, range: Interval::new(0.0, len), sense: Sense::Forward }))?);
    }
    let es: [EntityId; 4] = ev.try_into().map_err(|_| TopoError::IdExhausted)?;
    let l: Vec<CoedgeSpec> = es.iter().map(|&edge| CoedgeSpec { edge, sense: Sense::Forward }).collect();
    let surface = store.add_surface(Box::new(Plane { frame: Frame::from_z(Point3::new(0.5, 0.25, 0.25), Vector3::new(1.0, 0.0, 0.0))? })).map_err(FixtureError::from)?;
    let face = topo.make_face(&[&l], Some(SurfaceCarrier { surface, sense: Sense::Forward }))?;
    topo.add_face_use(b.volume, 0, FaceUseSpec { face, side: Side::Both })?;
    Ok((b, BaffleIds { vertices: vs, edges: es, face }))
}
