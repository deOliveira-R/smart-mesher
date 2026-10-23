//! Geometry rule `smartmesh.fan_vertex_mean.v1` for polygonal faces and polyhedral cells.
//!
//! Face: apex = vertex mean; area vector `A = 1/2 sum (p_i - apex) x (p_{i+1} - apex)` (equal to
//! `1/2 sum p_i x p_{i+1}` for a closed loop); centre = area-weighted centroid of the fan
//! triangles. Cell: the polyhedron whose boundary is the fan triangles of all faces; volume and
//! centroid come from the tetrahedra between those triangles and the cell vertex mean.
//! Linear (degree 1) geometry only; high-order cells use their own quadrature.

use crate::index::{CellFace, CellIx, FaceIx, Side};
use crate::read::MeshRead;
use sm_core::{Point3, Vector3};

/// Id of the default geometry rule. Cached geometry is keyed by this id.
pub const GEOMETRY_RULE_ID: &str = "smartmesh.fan_vertex_mean.v1";

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

fn mean<I: Iterator<Item = Point3>>(it: I) -> Point3 {
    let (mut s, mut n) = (Vector3::default(), 0.0);
    for p in it {
        s = s + Vector3::new(p.x, p.y, p.z);
        n += 1.0;
    }
    if n == 0.0 {
        return Point3::default();
    }
    Point3::new(s.x / n, s.y / n, s.z / n)
}

/// Geometry of face `f`.
pub fn face_geometry<M: MeshRead + ?Sized>(m: &M, f: FaceIx) -> FaceGeometry {
    let l = m.face_vertices(f);
    let pts: Vec<Point3> = l.iter().map(|v| m.point(v.get())).collect();
    face_geometry_of(&pts)
}

/// Geometry of a face given its loop points.
pub fn face_geometry_of(pts: &[Point3]) -> FaceGeometry {
    let apex = mean(pts.iter().copied());
    let n = pts.len();
    let mut area = Vector3::default();
    let tri: Vec<(Vector3, Point3)> = (0..n)
        .map(|i| {
            let (a, b) = (pts[i], pts[(i + 1) % n]);
            let v = (a - apex).cross(b - apex) * 0.5;
            (v, Point3::new((apex.x + a.x + b.x) / 3.0, (apex.y + a.y + b.y) / 3.0, (apex.z + a.z + b.z) / 3.0))
        })
        .collect();
    for (v, _) in &tri {
        area = area + *v;
    }
    let len = area.norm();
    if len == 0.0 {
        return FaceGeometry { area_vector: area, centroid: apex, apex };
    }
    let nhat = area * (1.0 / len);
    let (mut w, mut c) = (0.0, Vector3::default());
    for (v, p) in &tri {
        let wk = v.dot(nhat);
        w += wk;
        c = c + Vector3::new(p.x, p.y, p.z) * wk;
    }
    FaceGeometry { area_vector: area, centroid: Point3::new(c.x / w, c.y / w, c.z / w), apex }
}

/// Area vector of `f` (left -> right).
pub fn face_area_vector<M: MeshRead + ?Sized>(m: &M, f: FaceIx) -> Vector3 {
    face_geometry(m, f).area_vector
}

/// Cell volume and centroid under the default rule.
pub fn cell_volume_centroid<M: MeshRead + ?Sized>(m: &M, c: CellIx) -> (f64, Point3) {
    let o = mean(m.cell_vertices_sorted(c).iter().map(|v| m.point(v.get())));
    let (mut vol, mut moment) = (0.0, Vector3::default());
    for &CellFace { face, side } in m.cell_faces(c) {
        let l = m.face_vertices(face);
        let n = l.len();
        // Outward-ordered points.
        let pts: Vec<Point3> = (0..n)
            .map(|i| match side {
                Side::Left => m.point(l[i].get()),
                Side::Right => m.point(l[n - 1 - i].get()),
            })
            .collect();
        let apex = mean(pts.iter().copied());
        for i in 0..n {
            let (a, b) = (pts[i], pts[(i + 1) % n]);
            let v = (apex - o).dot((a - o).cross(b - o)) / 6.0;
            vol += v;
            let cx = (o.x + apex.x + a.x + b.x) / 4.0;
            let cy = (o.y + apex.y + a.y + b.y) / 4.0;
            let cz = (o.z + apex.z + a.z + b.z) / 4.0;
            moment = moment + Vector3::new(cx, cy, cz) * v;
        }
    }
    if vol == 0.0 {
        return (0.0, o);
    }
    (vol, Point3::new(moment.x / vol, moment.y / vol, moment.z / vol))
}

/// Cell volume under the default rule.
pub fn cell_volume<M: MeshRead + ?Sized>(m: &M, c: CellIx) -> f64 {
    cell_volume_centroid(m, c).0
}
