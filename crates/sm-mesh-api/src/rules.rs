//! Default polyhedral geometry rule `smartmesh.fan_vertex_mean.v1` (addendum §7.5).
//!
//! A face is a triangle fan about the vertex mean. Area vector `A = sum_i 1/2 (p_i - m) x
//! (p_{i+1} - m)`; the face centre is the area-weighted fan-triangle centroid. A cell volume is
//! the sum of signed fan tetrahedra against the cell vertex mean (equals OpenSn's side-tet sum).

use sm_core::{Point3, Vector3};

use crate::index::VertexIx;
use crate::query::CoordsRef;

pub use crate::schema::FAN_VERTEX_MEAN_V1;

/// Derived geometry of one face under the fan rule, in the stored loop orientation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FaceGeom {
    /// Area vector (normal times area), right-hand rule on the stored loop.
    pub area_vector: Vector3,
    /// Face centre.
    pub centroid: Point3,
    /// The fan apex (vertex mean).
    pub apex: Point3,
}

fn p(c: &CoordsRef<'_>, v: VertexIx) -> Point3 {
    Point3::new(c.x[v.index()], c.y[v.index()], c.z[v.index()])
}

/// Vertex mean of a loop (zero point for an empty loop).
pub fn vertex_mean(c: &CoordsRef<'_>, l: &[VertexIx]) -> Point3 {
    let n = l.len().max(1) as f64;
    let (mut x, mut y, mut z) = (0.0, 0.0, 0.0);
    for &v in l {
        let q = p(c, v);
        x += q.x;
        y += q.y;
        z += q.z;
    }
    Point3::new(x / n, y / n, z / n)
}

/// Evaluate the fan rule on a loop.
pub fn fan_face(c: &CoordsRef<'_>, l: &[VertexIx]) -> FaceGeom {
    let m = vertex_mean(c, l);
    let mut area = Vector3::default();
    let mut tris: Vec<(Vector3, Point3)> = Vec::with_capacity(l.len());
    for i in 0..l.len() {
        let (a, b) = (p(c, l[i]), p(c, l[(i + 1) % l.len()]));
        let ai = (a - m).cross(b - m) * 0.5;
        let ci = Point3::new((m.x + a.x + b.x) / 3.0, (m.y + a.y + b.y) / 3.0, (m.z + a.z + b.z) / 3.0);
        area = area + ai;
        tris.push((ai, ci));
    }
    let an = area.norm();
    let mut centroid = m;
    if an > 0.0 {
        let u = area * (1.0 / an);
        let (mut w, mut x, mut y, mut z) = (0.0, 0.0, 0.0, 0.0);
        for (ai, ci) in &tris {
            let wi = ai.dot(u);
            w += wi;
            x += wi * ci.x;
            y += wi * ci.y;
            z += wi * ci.z;
        }
        if w != 0.0 {
            centroid = Point3::new(x / w, y / w, z / w);
        }
    }
    FaceGeom { area_vector: area, centroid, apex: m }
}

/// Signed volume and centroid contribution of the fan tetrahedra of one face seen from a cell.
///
/// `outward` is a face loop; `o` is the cell vertex mean. Results are for the loop as given:
/// multiply by the cell's [`crate::index::Sign`] to get the outward-oriented contribution.
/// Returns `(volume, volume * centroid)` so contributions add up across faces.
pub fn fan_cell_contribution(c: &CoordsRef<'_>, outward: &[VertexIx], o: Point3) -> (f64, Vector3) {
    let m = vertex_mean(c, outward);
    let (mut vol, mut mom) = (0.0, Vector3::default());
    for i in 0..outward.len() {
        let (a, b) = (p(c, outward[i]), p(c, outward[(i + 1) % outward.len()]));
        let tri_area = (a - m).cross(b - m) * 0.5;
        let h = tri_area.dot(Point3::new((m.x + a.x + b.x) / 3.0, (m.y + a.y + b.y) / 3.0, (m.z + a.z + b.z) / 3.0) - o);
        let v = h / 3.0;
        let tc = Vector3::new(
            (o.x + m.x + a.x + b.x) / 4.0,
            (o.y + m.y + a.y + b.y) / 4.0,
            (o.z + m.z + a.z + b.z) / 4.0,
        );
        vol += v;
        mom = mom + tc * v;
    }
    (vol, mom)
}
