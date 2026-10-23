//! Geometry rule `smartmesh.fan_vertex_mean.v1` for polygonal faces and polyhedral cells.
//!
//! Face: apex = vertex mean; area vector `A = 1/2 sum (p_i - apex) x (p_{i+1} - apex)` (equal to
//! `1/2 sum p_i x p_{i+1}` for a closed loop); centre = area-weighted centroid of the fan
//! triangles. Cell: the polyhedron bounded by the fan triangles of all faces. Its volume is
//! `V = 1/3 sum_f s_f A_f . (apex_f - o)` (`o` = first vertex of the first face; any `o` gives
//! the same value because the area vectors of a closed cell sum to zero), which is exactly the
//! sum of the side tetrahedra `(o, apex_f, p_i, p_{i+1})`; the centroid is their
//! volume-weighted centroid.
//!
//! Every function here works on `&dyn MeshRead`, reads only borrowed slices and performs **no
//! heap allocation**. Linear (degree 1) geometry only; high-order cells use their own quadrature.

use crate::index::{CellIx, FaceIx, MeshEntityKind, Side, VertexIx};
use crate::read::{Coords, GeometryCache, MeshRead};
use sm_core::{Point3, Vector3};

/// Id of the default geometry rule. Cached geometry is keyed by this id.
pub const FAN_VERTEX_MEAN_V1: &str = "smartmesh.fan_vertex_mean.v1";

/// Face geometry under the default rule.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FaceGeometry {
    /// Area vector along the loop's right-hand normal (left -> right).
    pub area_vector: Vector3,
    /// Fan-triangle centroid.
    pub centroid: Point3,
    /// Vertex-mean apex (persist for non-planar faces).
    pub apex: Point3,
}

#[inline]
fn pt(co: &Coords<'_>, v: VertexIx) -> Point3 {
    let i = v.get();
    Point3::new(co.x[i], co.y[i], co.z[i])
}

#[inline]
fn apex_of(co: &Coords<'_>, l: &[VertexIx]) -> Point3 {
    let (mut sx, mut sy, mut sz) = (0.0, 0.0, 0.0);
    for &v in l {
        let p = pt(co, v);
        sx += p.x;
        sy += p.y;
        sz += p.z;
    }
    let n = l.len().max(1) as f64;
    Point3::new(sx / n, sy / n, sz / n)
}

/// Geometry of face `f`: vertex-mean apex, area vector and fan centroid (three passes over the loop).
pub fn face_geometry(m: &dyn MeshRead, f: FaceIx) -> FaceGeometry {
    let co = m.coords();
    let l = m.face_vertices(f);
    let n = l.len();
    let apex = apex_of(&co, l);
    let mut area = Vector3::default();
    for i in 0..n {
        let (a, b) = (pt(&co, l[i]), pt(&co, l[(i + 1) % n]));
        area = area + (a - apex).cross(b - apex) * 0.5;
    }
    let len = area.norm();
    if len == 0.0 {
        return FaceGeometry {
            area_vector: area,
            centroid: apex,
            apex,
        };
    }
    let nhat = area * (1.0 / len);
    let (mut w, mut c) = (0.0, Vector3::default());
    for i in 0..n {
        let (a, b) = (pt(&co, l[i]), pt(&co, l[(i + 1) % n]));
        let wk = ((a - apex).cross(b - apex) * 0.5).dot(nhat);
        w += wk;
        c = c + Vector3::new(apex.x + a.x + b.x, apex.y + a.y + b.y, apex.z + a.z + b.z)
            * (wk / 3.0);
    }
    if w == 0.0 {
        return FaceGeometry {
            area_vector: area,
            centroid: apex,
            apex,
        };
    }
    FaceGeometry {
        area_vector: area,
        centroid: Point3::new(c.x / w, c.y / w, c.z / w),
        apex,
    }
}

/// Area vector of `f` (along the loop normal, left -> right).
pub fn face_area_vector(m: &dyn MeshRead, f: FaceIx) -> Vector3 {
    let co = m.coords();
    let l = m.face_vertices(f);
    let n = l.len();
    let apex = apex_of(&co, l);
    let mut area = Vector3::default();
    for i in 0..n {
        let (a, b) = (pt(&co, l[i]), pt(&co, l[(i + 1) % n]));
        area = area + (a - apex).cross(b - apex) * 0.5;
    }
    area
}

/// Centroid of face `f` under the default rule.
pub fn face_centroid(m: &dyn MeshRead, f: FaceIx) -> Point3 {
    face_geometry(m, f).centroid
}

/// Vertex-mean apex of face `f`.
pub fn face_apex(m: &dyn MeshRead, f: FaceIx) -> Point3 {
    apex_of(&m.coords(), m.face_vertices(f))
}

/// Cell volume and centroid under the default rule (the cell is the fan polyhedron of its faces).
pub fn cell_volume_centroid(m: &dyn MeshRead, c: CellIx) -> (f64, Point3) {
    let co = m.coords();
    let faces = m.cell_faces(c);
    let Some(first) = faces.first() else {
        return (0.0, Point3::default());
    };
    let o = m
        .face_vertices(first.face())
        .first()
        .map_or(Point3::default(), |&v| pt(&co, v));
    let (mut vol, mut moment) = (0.0, Vector3::default());
    for sf in faces {
        let l = m.face_vertices(sf.face());
        let n = l.len();
        let apex = apex_of(&co, l);
        for i in 0..n {
            let (mut a, mut b) = (pt(&co, l[i]), pt(&co, l[(i + 1) % n]));
            if sf.side() == Side::Right {
                core::mem::swap(&mut a, &mut b);
            }
            let v = (apex - o).dot((a - o).cross(b - o)) / 6.0;
            vol += v;
            moment = moment
                + Vector3::new(
                    o.x + apex.x + a.x + b.x,
                    o.y + apex.y + a.y + b.y,
                    o.z + apex.z + a.z + b.z,
                ) * (v / 4.0);
        }
    }
    if vol == 0.0 {
        return (0.0, o);
    }
    (
        vol,
        Point3::new(moment.x / vol, moment.y / vol, moment.z / vol),
    )
}

/// Cell volume under the default rule.
pub fn cell_volume(m: &dyn MeshRead, c: CellIx) -> f64 {
    cell_volume_centroid(m, c).0
}

/// Compute the geometry cache of a mesh under [`FAN_VERTEX_MEAN_V1`].
pub fn compute_geometry_cache(m: &dyn MeshRead) -> GeometryCache {
    let (nf, nc) = (m.count(MeshEntityKind::Face), m.count(MeshEntityKind::Cell));
    let mut g = GeometryCache {
        rule: FAN_VERTEX_MEAN_V1.to_string(),
        connectivity_hash: crate::validate::connectivity_hash(m),
        face_area: Vec::with_capacity(nf),
        face_centroid: Vec::with_capacity(nf),
        face_apex: Vec::with_capacity(nf),
        cell_volume: Vec::with_capacity(nc),
        cell_centroid: Vec::with_capacity(nc),
    };
    for f in 0..nf {
        let fg = face_geometry(m, FaceIx::new(f));
        g.face_area.push(fg.area_vector);
        g.face_centroid.push(fg.centroid);
        g.face_apex.push(fg.apex);
    }
    for c in 0..nc {
        let (v, p) = cell_volume_centroid(m, CellIx::new(c));
        g.cell_volume.push(v);
        g.cell_centroid.push(p);
    }
    g
}
