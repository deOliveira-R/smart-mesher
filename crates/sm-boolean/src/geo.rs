//! Small geometry helpers shared by the kernel: point and vector arithmetic, sampling of edges
//! and faces through the carriers, face-oriented normals.

use sm_core::{Aabb, EntityId, Point3, Vector3};
use sm_geom_api::{ParametricCurve, ParametricSurface, Uv};
use sm_topo_api::{Carrier, CellFlags, CurveCarrier, Sense, SurfaceCarrier, TopologyView, TopologyViewExt};

pub(crate) fn vec3(p: Point3) -> Vector3 {
    Vector3::new(p.x, p.y, p.z)
}

pub(crate) fn pt(v: Vector3) -> Point3 {
    Point3::new(v.x, v.y, v.z)
}

pub(crate) fn unit(v: Vector3) -> Option<Vector3> {
    let n = v.norm();
    (n.is_finite() && n > 0.0).then(|| v * (1.0 / n))
}

pub(crate) fn dist(a: Point3, b: Point3) -> f64 {
    (a - b).norm()
}

pub(crate) fn lerp(a: Point3, b: Point3, t: f64) -> Point3 {
    a + (b - a) * t
}

/// The point of a vertex.
pub(crate) fn vertex_point<V: TopologyView + ?Sized>(view: &V, v: EntityId) -> Option<Point3> {
    match view.carrier(v) {
        Carrier::Point(p) => Some(*p),
        _ => None,
    }
}

/// The curve and carrier of an edge, if it has one.
pub(crate) fn edge_curve<'a, V: TopologyView + ?Sized>(view: &'a V, e: EntityId) -> Option<(&'a dyn ParametricCurve, CurveCarrier)> {
    match view.carrier(e) {
        Carrier::Curve(cc) => view.shapes().curve(cc.curve).map(|c| (c, *cc)),
        _ => None,
    }
}

/// The surface and carrier of a face, if it has one.
pub(crate) fn face_surface<'a, V: TopologyView + ?Sized>(view: &'a V, f: EntityId) -> Option<(&'a dyn ParametricSurface, SurfaceCarrier)> {
    match view.carrier(f) {
        Carrier::Surface(sc) => view.shapes().surface(sc.surface).map(|s| (s, *sc)),
        _ => None,
    }
}

/// Curve parameter at fraction `frac` of the edge (0 = start, 1 = end).
pub(crate) fn edge_param(cc: &CurveCarrier, frac: f64) -> f64 {
    match cc.sense {
        Sense::Forward => cc.range.lo + frac * (cc.range.hi - cc.range.lo),
        Sense::Reversed => cc.range.hi - frac * (cc.range.hi - cc.range.lo),
    }
}

/// The point at fraction `frac` of an edge, walking from its start vertex. Edges without a curve
/// are straight between their vertices.
pub(crate) fn edge_sample<V: TopologyView + ?Sized>(view: &V, e: EntityId, frac: f64) -> Option<Point3> {
    let (s, t) = view.edge_ends(e)?;
    if let Some((c, cc)) = edge_curve(view, e) {
        return Some(c.eval(edge_param(&cc, frac)));
    }
    let (ps, pt) = (vertex_point(view, s)?, vertex_point(view, t)?);
    Some(lerp(ps, pt, frac))
}

/// True for a pole edge.
pub(crate) fn is_degenerate<V: TopologyView + ?Sized>(view: &V, e: EntityId) -> bool {
    view.flags(e).contains(CellFlags::DEGENERATE)
}

/// Bounding box of a face from its vertices and nine samples per edge, grown by `pad`.
pub(crate) fn face_aabb<V: TopologyView + ?Sized>(view: &V, f: EntityId, pad: f64) -> Aabb {
    let mut b = Aabb::empty();
    for &e in view.face_edges(f) {
        if is_degenerate(view, e) {
            continue;
        }
        for k in 0..=8 {
            if let Some(p) = edge_sample(view, e, k as f64 / 8.0) {
                b.extend(p);
            }
        }
    }
    if !b.is_empty() {
        b.lo = Point3::new(b.lo.x - pad, b.lo.y - pad, b.lo.z - pad);
        b.hi = Point3::new(b.hi.x + pad, b.hi.y + pad, b.hi.z + pad);
    }
    b
}

pub(crate) fn aabb_overlap(a: &Aabb, b: &Aabb) -> bool {
    !(a.is_empty() || b.is_empty())
        && a.lo.x <= b.hi.x
        && b.lo.x <= a.hi.x
        && a.lo.y <= b.hi.y
        && b.lo.y <= a.hi.y
        && a.lo.z <= b.hi.z
        && b.lo.z <= a.hi.z
}

/// The face-oriented unit normal at the point of the face surface nearest to `p`, with that point.
pub(crate) fn face_normal_near<V: TopologyView + ?Sized>(view: &V, f: EntityId, p: Point3) -> Option<(Vector3, Point3, Uv)> {
    let (s, sc) = face_surface(view, f)?;
    let pr = s.project(p, None).ok()?;
    let n = s.normal(pr.uv).ok()?;
    let n = unit(n)? * sc.sense.sign();
    Some((n, pr.point, pr.uv))
}

/// Distance from `p` to the surface carrying `f` (None if the face has no surface or the
/// projection fails).
pub(crate) fn dist_to_face_surface<V: TopologyView + ?Sized>(view: &V, f: EntityId, p: Point3) -> Option<f64> {
    let (s, _) = face_surface(view, f)?;
    s.project(p, None).ok().map(|pr| pr.distance)
}

/// Sample points of a face boundary: every vertex and the interior samples of each edge.
pub(crate) fn face_boundary_samples<V: TopologyView + ?Sized>(view: &V, f: EntityId, per_edge: usize) -> Vec<Point3> {
    let mut out = Vec::new();
    for &e in view.face_edges(f) {
        if is_degenerate(view, e) {
            continue;
        }
        for k in 0..=per_edge {
            if let Some(p) = edge_sample(view, e, k as f64 / per_edge as f64) {
                out.push(p);
            }
        }
    }
    out
}
