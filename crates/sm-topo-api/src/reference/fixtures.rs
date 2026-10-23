//! Canonical models built through the public edit API (so they also exercise it).
//!
//! Builders named `*_in_op` need an open op; the scenario builders (`two_bricks_*`,
//! `split_top_face`) manage their own ops with fixed `OpId`s so ids and hashes are reproducible.

use super::RefComplex;
use crate::edit::*;
use crate::error::TopoError;
use crate::ids::*;
use crate::model::*;
use crate::view::TopologyView;
use core::f64::consts::{FRAC_PI_2, TAU};
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::vecs::Frame;
use sm_geom_api::{AnalyticSurface, Circle, Cylinder, Line, ModelTolerance, PCurve, Plane, Sense, ShapeLibrary, Sphere};

/// New empty model with the default tolerance.
pub fn new_model() -> RefComplex {
    RefComplex::new(ModelTolerance::default(), ShapeLibrary::new())
}

/// Ids of a brick. Vertex index bits are `x | 2y | 4z` (bit set = max side). Faces are ordered
/// `-x, +x, -y, +y, -z, +z`.
#[derive(Clone, Debug)]
pub struct Brick {
    /// Body.
    pub body: BodyId,
    /// Volume.
    pub volume: VolumeId,
    /// Outer shell.
    pub shell: ShellRef,
    /// Eight vertices.
    pub vertices: [VertexId; 8],
    /// Twelve edges, in `edge_pairs` order.
    pub edges: [EdgeId; 12],
    /// Six faces.
    pub faces: [FaceId; 6],
}

fn edge_pairs() -> Vec<(usize, usize)> {
    let mut v = Vec::new();
    for a in 0..8usize {
        for bit in [1usize, 2, 4] {
            if a & bit == 0 {
                v.push((a, a | bit));
            }
        }
    }
    v
}

impl Brick {
    /// The edge between two vertex indices (order-insensitive).
    pub fn edge(&self, a: usize, b: usize) -> Option<EdgeId> {
        let (a, b) = (a.min(b), a.max(b));
        edge_pairs().iter().position(|&p| p == (a, b)).map(|i| self.edges[i])
    }
}

/// Create a straight edge between two existing vertices.
pub fn line_edge(c: &mut RefComplex, a: VertexId, b: VertexId) -> Result<EdgeId, TopoError> {
    let (pa, pb) = (c.vertex(a).ok_or(TopoError::NotFound(a.entity()))?.point, c.vertex(b).ok_or(TopoError::NotFound(b.entity()))?.point);
    let line = Line::through(pa, pb)?;
    let len = (pb - pa).norm();
    let curve = c.library_mut().add_curve(line)?;
    c.make_edge(EdgeSpec { curve, t0: 0.0, t1: len, start: a, end: b })
}

/// Create a planar face through `origin` with outward normal `n` and a single outer loop.
pub fn plane_face(c: &mut RefComplex, origin: Point3, n: Vector3, loop_edges: &[(EdgeId, Sense)]) -> Result<FaceId, TopoError> {
    let surface = c.library_mut().add_analytic_surface(AnalyticSurface::Plane(Plane::new(origin, n)?))?;
    let coedges = loop_edges.iter().map(|&(edge, sense)| CoedgeRec { kind: CoedgeKind::Edge { edge, sense }, pcurve: None }).collect();
    c.make_face(FaceSpec { surface, surface_sense: Sense::Forward, loops: vec![LoopRec { kind: LoopKind::Outer, coedges }] })
}

/// Build an axis-aligned brick (op must be open).
pub fn brick_in_op(c: &mut RefComplex, min: Point3, max: Point3) -> Result<Brick, TopoError> {
    let body = c.make_body()?;
    let volume = c.make_volume(body)?;
    let shell = c.make_shell(volume, ShellKind::Outer)?;
    let corner = |i: usize| Point3::new(if i & 1 == 0 { min.x } else { max.x }, if i & 2 == 0 { min.y } else { max.y }, if i & 4 == 0 { min.z } else { max.z });
    let mut vertices = Vec::new();
    for i in 0..8 {
        vertices.push(c.make_vertex(corner(i))?);
    }
    let pairs = edge_pairs();
    let mut edges = Vec::new();
    for &(a, b) in &pairs {
        edges.push(line_edge(c, vertices[a], vertices[b])?);
    }
    let mut faces = Vec::new();
    for (axis, side) in [(0usize, 0usize), (0, 1), (1, 0), (1, 1), (2, 0), (2, 1)] {
        let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
        let idx = |du: usize, dv: usize| (side << axis) | (du << u) | (dv << v);
        let mut ring = [idx(0, 0), idx(1, 0), idx(1, 1), idx(0, 1)];
        if side == 0 {
            ring.reverse();
        }
        let loop_edges: Vec<(EdgeId, Sense)> = (0..4)
            .filter_map(|k| {
                let (a, b) = (ring[k], ring[(k + 1) % 4]);
                let pos = pairs.iter().position(|&p| p == (a.min(b), a.max(b)))?;
                Some((edges[pos], if a < b { Sense::Forward } else { Sense::Reversed }))
            })
            .collect();
        let mut n = [0.0; 3];
        n[axis] = if side == 1 { 1.0 } else { -1.0 };
        let f = plane_face(c, corner(idx(0, 0)), Vector3::new(n[0], n[1], n[2]), &loop_edges)?;
        c.bind_face_use(FaceUse { face: f, side: FaceSide::Negative }, shell)?;
        faces.push(f);
    }
    let to_arr = |v: Vec<VertexId>| -> Result<[VertexId; 8], TopoError> { v.try_into().map_err(|_| TopoError::Unsupported("vertex count")) };
    let edges: [EdgeId; 12] = edges.try_into().map_err(|_| TopoError::Unsupported("edge count"))?;
    let faces: [FaceId; 6] = faces.try_into().map_err(|_| TopoError::Unsupported("face count"))?;
    Ok(Brick { body, volume, shell, vertices: to_arr(vertices)?, edges, faces })
}

/// A brick built in its own op.
pub fn brick(c: &mut RefComplex, op: OpId, min: Point3, max: Point3) -> Result<Brick, TopoError> {
    c.begin_op(op)?;
    let b = brick_in_op(c, min, max)?;
    c.end_op()?;
    Ok(b)
}

/// A cylinder with a periodic seam.
#[derive(Clone, Debug)]
pub struct CylinderIds {
    /// Body.
    pub body: BodyId,
    /// Volume.
    pub volume: VolumeId,
    /// `(bottom, top)` vertices (both on the seam).
    pub vertices: [VertexId; 2],
    /// Bottom circle, top circle, seam line.
    pub edges: [EdgeId; 3],
    /// Lateral, top cap, bottom cap.
    pub faces: [FaceId; 3],
}

/// Cylinder of radius `r` and height `h` about the z axis, seam at `(r, 0, z)` (op must be open).
/// The lateral face uses the seam edge twice with opposite senses.
pub fn cylinder_in_op(c: &mut RefComplex, r: f64, h: f64) -> Result<CylinderIds, TopoError> {
    let body = c.make_body()?;
    let volume = c.make_volume(body)?;
    let shell = c.make_shell(volume, ShellKind::Outer)?;
    let o = Point3::new(0.0, 0.0, 0.0);
    let top_o = Point3::new(0.0, 0.0, h);
    let up = Vector3::new(0.0, 0.0, 1.0);
    let v0 = c.make_vertex(Point3::new(r, 0.0, 0.0))?;
    let v1 = c.make_vertex(Point3::new(r, 0.0, h))?;
    let cb = c.library_mut().add_curve(Circle::new(Frame::from_z(o, up)?, r)?)?;
    let ct = c.library_mut().add_curve(Circle::new(Frame::from_z(top_o, up)?, r)?)?;
    let cs = c.library_mut().add_curve(Line::new(Point3::new(r, 0.0, 0.0), up)?)?;
    let e_b = c.make_edge(EdgeSpec { curve: cb, t0: 0.0, t1: TAU, start: v0, end: v0 })?;
    let e_t = c.make_edge(EdgeSpec { curve: ct, t0: 0.0, t1: TAU, start: v1, end: v1 })?;
    let e_s = c.make_edge(EdgeSpec { curve: cs, t0: 0.0, t1: h, start: v0, end: v1 })?;
    let lateral_surface = c.library_mut().add_analytic_surface(AnalyticSurface::Cylinder(Cylinder::new(Frame::from_z(o, up)?, r)?))?;
    let seg = |a: [f64; 2], b: [f64; 2]| Some(PCurve::Segment { start: a, end: b });
    let ed = |edge, sense, pcurve| CoedgeRec { kind: CoedgeKind::Edge { edge, sense }, pcurve };
    let lateral = c.make_face(FaceSpec {
        surface: lateral_surface,
        surface_sense: Sense::Forward,
        loops: vec![LoopRec {
            kind: LoopKind::Outer,
            coedges: vec![
                ed(e_b, Sense::Forward, seg([0.0, 0.0], [TAU, 0.0])),
                ed(e_s, Sense::Forward, seg([TAU, 0.0], [TAU, h])),
                ed(e_t, Sense::Reversed, seg([TAU, h], [0.0, h])),
                ed(e_s, Sense::Reversed, seg([0.0, h], [0.0, 0.0])),
            ],
        }],
    })?;
    let top = plane_face(c, top_o, up, &[(e_t, Sense::Forward)])?;
    let bottom = plane_face(c, o, -up, &[(e_b, Sense::Reversed)])?;
    for f in [lateral, top, bottom] {
        c.bind_face_use(FaceUse { face: f, side: FaceSide::Negative }, shell)?;
    }
    Ok(CylinderIds { body, volume, vertices: [v0, v1], edges: [e_b, e_t, e_s], faces: [lateral, top, bottom] })
}

/// A sphere: one face with a seam meridian and two pole corners.
#[derive(Clone, Debug)]
pub struct SphereIds {
    /// Volume.
    pub volume: VolumeId,
    /// South and north pole vertices.
    pub poles: [VertexId; 2],
    /// Seam meridian.
    pub seam: EdgeId,
    /// The single face.
    pub face: FaceId,
}

/// Sphere of radius `r` about the origin (op must be open). Poles appear as `Pole` coedges.
pub fn sphere_in_op(c: &mut RefComplex, r: f64) -> Result<SphereIds, TopoError> {
    let body = c.make_body()?;
    let volume = c.make_volume(body)?;
    let shell = c.make_shell(volume, ShellKind::Outer)?;
    let south = c.make_vertex(Point3::new(0.0, 0.0, -r))?;
    let north = c.make_vertex(Point3::new(0.0, 0.0, r))?;
    let o = Point3::new(0.0, 0.0, 0.0);
    let meridian = Circle::new(Frame::from_zx(o, Vector3::new(0.0, -1.0, 0.0), Vector3::new(1.0, 0.0, 0.0))?, r)?;
    let cm = c.library_mut().add_curve(meridian)?;
    let seam = c.make_edge(EdgeSpec { curve: cm, t0: -FRAC_PI_2, t1: FRAC_PI_2, start: south, end: north })?;
    let surf = c.library_mut().add_analytic_surface(AnalyticSurface::Sphere(Sphere::new(Frame::from_z(o, Vector3::new(0.0, 0.0, 1.0))?, r)?))?;
    let edge = |sense| CoedgeRec { kind: CoedgeKind::Edge { edge: seam, sense }, pcurve: None };
    let face = c.make_face(FaceSpec {
        surface: surf,
        surface_sense: Sense::Forward,
        loops: vec![LoopRec {
            kind: LoopKind::Outer,
            coedges: vec![CoedgeRec { kind: CoedgeKind::Pole { vertex: south }, pcurve: None }, edge(Sense::Forward), CoedgeRec { kind: CoedgeKind::Pole { vertex: north }, pcurve: None }, edge(Sense::Reversed)],
        }],
    })?;
    c.bind_face_use(FaceUse { face, side: FaceSide::Negative }, shell)?;
    Ok(SphereIds { volume, poles: [south, north], seam, face })
}

/// A block with a free-standing internal plate (baffle) and a free wire.
#[derive(Clone, Debug)]
pub struct BaffleBlock {
    /// The enclosing brick.
    pub brick: Brick,
    /// The internal shell holding the plate and the wire.
    pub internal: ShellRef,
    /// The plate face (both uses in the same volume).
    pub plate: FaceId,
    /// The plate's four edges.
    pub plate_edges: [EdgeId; 4],
    /// The free wire edge.
    pub wire: EdgeId,
}

/// Brick `[0,2] x [0,1] x [0,1]` with an internal plate at `x = 1` spanning `y, z in [0.25, 0.75]`
/// and a wire from `(0.25, 0.1, 0.1)` to `(0.25, 0.9, 0.1)` (op must be open).
pub fn baffle_block_in_op(c: &mut RefComplex) -> Result<BaffleBlock, TopoError> {
    let brick = brick_in_op(c, Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 1.0, 1.0))?;
    let internal = c.make_shell(brick.volume, ShellKind::Internal)?;
    let pts = [(0.25, 0.25), (0.75, 0.25), (0.75, 0.75), (0.25, 0.75)];
    let vs: Vec<VertexId> = pts.iter().map(|&(y, z)| c.make_vertex(Point3::new(1.0, y, z))).collect::<Result<_, _>>()?;
    let mut es = Vec::new();
    for k in 0..4 {
        es.push(line_edge(c, vs[k], vs[(k + 1) % 4])?);
    }
    let loop_edges: Vec<(EdgeId, Sense)> = es.iter().map(|&e| (e, Sense::Forward)).collect();
    let plate = plane_face(c, Point3::new(1.0, 0.25, 0.25), Vector3::new(1.0, 0.0, 0.0), &loop_edges)?;
    for side in FaceSide::BOTH {
        c.bind_face_use(FaceUse { face: plate, side }, internal)?;
    }
    let (w0, w1) = (c.make_vertex(Point3::new(0.25, 0.1, 0.1))?, c.make_vertex(Point3::new(0.25, 0.9, 0.1))?);
    let wire = line_edge(c, w0, w1)?;
    c.add_wire_edge(internal, wire)?;
    let plate_edges: [EdgeId; 4] = es.try_into().map_err(|_| TopoError::Unsupported("edge count"))?;
    Ok(BaffleBlock { brick, internal, plate, plate_edges, wire })
}

/// Two unit bricks `[0,1]^3` and `[1,2] x [0,1] x [0,1]` whose touching faces are identical, merged
/// into one shared face (ops 1 to 3).
#[derive(Clone, Debug)]
pub struct TwoBricks {
    /// Left brick.
    pub a: Brick,
    /// Right brick.
    pub b: Brick,
    /// The surviving shared face (A's `+x` face).
    pub shared: FaceId,
}

/// Build [`TwoBricks`] with a full vertex/edge/face merge.
pub fn two_bricks_merged(c: &mut RefComplex) -> Result<TwoBricks, TopoError> {
    let a = brick(c, OpId(1), Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0))?;
    let b = brick(c, OpId(2), Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 1.0, 1.0))?;
    c.begin_op(OpId(3))?;
    // A's +x face has vertices with x bit set (1,3,5,7); B's -x face has x bit clear (0,2,4,6).
    c.set_coincidence(a.faces[1], b.faces[0], Coincidence::CoincidentUnimprinted, Some(1.0))?;
    for k in 0..4 {
        let (ia, ib) = ([1, 3, 5, 7][k], [0, 2, 4, 6][k]);
        c.merge_vertices(a.vertices[ia], b.vertices[ib])?;
    }
    for (ea, eb) in [((1, 3), (0, 2)), ((5, 7), (4, 6)), ((1, 5), (0, 4)), ((3, 7), (2, 6))] {
        c.merge_edges(a.edge(ea.0, ea.1).ok_or(TopoError::Unsupported("edge"))?, b.edge(eb.0, eb.1).ok_or(TopoError::Unsupported("edge"))?)?;
    }
    c.merge_faces(a.faces[1], b.faces[0], Sense::Reversed)?;
    c.end_op()?;
    let shared = a.faces[1];
    Ok(TwoBricks { a, b, shared })
}

/// Index of the corner of `face`'s outer loop whose tail is `vertex`.
pub fn corner_index(view: &dyn TopologyView, face: FaceId, vertex: VertexId) -> Option<u32> {
    let f = view.face(face)?;
    f.loops.first()?.coedges.iter().position(|c| crate::view::coedge_ends(view, c).is_some_and(|e| e.0 == vertex)).map(|i| i as u32)
}

/// Result of [`two_bricks_imprint_merge`].
#[derive(Clone, Debug)]
pub struct ImprintedBricks {
    /// Brick A, `[0,1]^3`.
    pub a: Brick,
    /// Brick B, `[1,2] x [0,0.5] x [0,1]`: its `-x` face is half of A's `+x` face.
    pub b: Brick,
    /// A's `+x` face after the split: `(lower half y<0.5, upper half)`.
    pub a_halves: (FaceId, FaceId),
    /// The edge at `x = 1, y = 0.5` (three faces meet: non-manifold).
    pub cut_edge: EdgeId,
    /// Results of splitting A's bottom (`y` along `z = 0`) and top (`z = 1`) `+x` edges.
    pub splits: [EdgeSplit; 2],
}

/// Imprint and merge a half-size contact (ops 1 to 4); see [`two_bricks_imprint`] and [`merge_imprinted`].
pub fn two_bricks_imprint_merge(c: &mut RefComplex) -> Result<ImprintedBricks, TopoError> {
    let ib = two_bricks_imprint(c)?;
    merge_imprinted(c, &ib)?;
    Ok(ib)
}

/// Imprint only (ops 1 to 3): A's `+x` face is split along `y = 0.5`. Status of (A `+x`, B `-x`)
/// is `Overlapping`; the new lower half and B's `-x` face are `ImprintedNotMerged`.
pub fn two_bricks_imprint(c: &mut RefComplex) -> Result<ImprintedBricks, TopoError> {
    let a = brick(c, OpId(1), Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0))?;
    let b = brick(c, OpId(2), Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.5, 1.0))?;
    let miss = || TopoError::Unsupported("fixture edge lookup");

    c.begin_op(OpId(3))?;
    let (a_px, b_mx) = (a.faces[1], b.faces[0]);
    c.set_coincidence(a_px, b_mx, Coincidence::Overlapping, Some(0.5))?;
    let s_bottom = c.split_edge(a.edge(1, 3).ok_or_else(miss)?, SplitAt::NewVertex(0.5))?;
    let s_top = c.split_edge(a.edge(5, 7).ok_or_else(miss)?, SplitAt::NewVertex(0.5))?;
    let cut_edge = line_edge(c, s_bottom.vertex, s_top.vertex)?;
    let (i, j) = (corner_index(c, a_px, s_bottom.vertex).ok_or_else(miss)?, corner_index(c, a_px, s_top.vertex).ok_or_else(miss)?);
    let split = c.split_face(a_px, FaceCut { from: i.min(j), to: i.max(j), edge: cut_edge, inner_loops_to_first: vec![] })?;
    let first_has_v1 = corner_index(c, split.first, a.vertices[1]).is_some();
    let (lower, upper) = if first_has_v1 { (split.first, split.second) } else { (split.second, split.first) };
    c.set_coincidence(lower, b_mx, Coincidence::ImprintedNotMerged, Some(0.5))?;
    c.end_op()?;
    Ok(ImprintedBricks { a, b, a_halves: (lower, upper), cut_edge, splits: [s_bottom, s_top] })
}

/// Merge the imprinted contact (op 4): vertices, edges, then the face. Afterwards the lower half of
/// A's `+x` face is one face with two uses (A: `Aligned`, B: `Opposed`).
pub fn merge_imprinted(c: &mut RefComplex, ib: &ImprintedBricks) -> Result<(), TopoError> {
    let miss = || TopoError::Unsupported("fixture edge lookup");
    let (a, b) = (&ib.a, &ib.b);
    let [s_bottom, s_top] = ib.splits;
    c.begin_op(OpId(4))?;
    // A lower half corners (1, m0, m1, 5) <-> B -x face corners (0, 2, 6, 4)
    c.merge_vertices(a.vertices[1], b.vertices[0])?;
    c.merge_vertices(s_bottom.vertex, b.vertices[2])?;
    c.merge_vertices(s_top.vertex, b.vertices[6])?;
    c.merge_vertices(a.vertices[5], b.vertices[4])?;
    c.merge_edges(s_bottom.first, b.edge(0, 2).ok_or_else(miss)?)?;
    c.merge_edges(ib.cut_edge, b.edge(2, 6).ok_or_else(miss)?)?;
    c.merge_edges(s_top.first, b.edge(4, 6).ok_or_else(miss)?)?;
    c.merge_edges(a.edge(1, 5).ok_or_else(miss)?, b.edge(0, 4).ok_or_else(miss)?)?;
    c.merge_faces(ib.a_halves.0, b.faces[0], Sense::Reversed)?;
    c.end_op()?;
    Ok(())
}

/// Result of [`split_top_face`].
#[derive(Clone, Debug)]
pub struct SplitTop {
    /// The brick (its `+z` face is now retired).
    pub brick: Brick,
    /// The original `+z` face id.
    pub original: FaceId,
    /// The two pieces.
    pub pieces: FaceSplit,
    /// The two split edges' results (along x at y=0 and y=1).
    pub edge_splits: [EdgeSplit; 2],
    /// The new cutting edge.
    pub cut: EdgeId,
}

/// A unit brick whose `+z` face is cut in two along `x = 0.5` (op 2): the two `+z` edges along x,
/// `(4,5)` and `(6,7)`, are split at their midpoints and joined by a new edge.
pub fn split_top_face(c: &mut RefComplex) -> Result<SplitTop, TopoError> {
    let b = brick(c, OpId(1), Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0))?;
    let miss = || TopoError::Unsupported("fixture edge lookup");
    c.begin_op(OpId(2))?;
    let top = b.faces[5];
    let s1 = c.split_edge(b.edge(4, 5).ok_or_else(miss)?, SplitAt::NewVertex(0.5))?;
    let s2 = c.split_edge(b.edge(6, 7).ok_or_else(miss)?, SplitAt::NewVertex(0.5))?;
    let cut = line_edge(c, s1.vertex, s2.vertex)?;
    let (i, j) = (corner_index(c, top, s1.vertex).ok_or_else(miss)?, corner_index(c, top, s2.vertex).ok_or_else(miss)?);
    let pieces = c.split_face(top, FaceCut { from: i.min(j), to: i.max(j), edge: cut, inner_loops_to_first: vec![] })?;
    c.end_op()?;
    Ok(SplitTop { brick: b, original: top, pieces, edge_splits: [s1, s2], cut })
}

/// Convenience: entity ids of a list of faces.
pub fn entities(faces: &[FaceId]) -> Vec<EntityId> {
    faces.iter().map(|f| f.entity()).collect()
}
