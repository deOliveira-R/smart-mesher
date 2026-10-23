//! Derived geometry under the default rule `smartmesh.fan_vertex_mean.v1`.
//!
//! Faces are triangle fans about their vertex mean; cells are sums of side tetrahedra from
//! the closure vertex mean. For planar faces and convex cells this equals the textbook
//! values; for non-planar faces it is the declared, reproducible definition.

use crate::point::{Orient, Point};
use crate::traits::{MeshGeometry, MeshTopology};
use sm_core::{Point3, Vector3};

fn mean(pts: &[Point3]) -> Point3 {
    let n = pts.len().max(1) as f64;
    let (x, y, z) = pts.iter().fold((0.0, 0.0, 0.0), |a, p| (a.0 + p.x, a.1 + p.y, a.2 + p.z));
    Point3::new(x / n, y / n, z / n)
}

fn lerp_sum(w: f64, p: Point3, acc: &mut [f64; 3]) {
    acc[0] += w * p.x;
    acc[1] += w * p.y;
    acc[2] += w * p.z;
}

/// Face and cell quantities derived from topology + coordinates.
pub trait MeshMetrics: MeshTopology + MeshGeometry {
    /// Vertex mean of a face (the fan apex).
    fn face_apex(&self, face: Point) -> Point3 {
        let pts: Vec<Point3> = self.cone(face).indices.iter().map(|&v| self.vertex_coord(v)).collect();
        mean(&pts)
    }

    /// Area vector `A = 1/2 sum p_i x p_{i+1}` of the stored loop: it points from `left`
    /// into `right` (outward for boundary faces).
    fn face_area_vector(&self, face: Point) -> Vector3 {
        let a = self.face_apex(face);
        let idx = self.cone(face).indices;
        let mut s = Vector3::new(0.0, 0.0, 0.0);
        for i in 0..idx.len() {
            let p = self.vertex_coord(idx[i]) - a;
            let q = self.vertex_coord(idx[(i + 1) % idx.len()]) - a;
            s = s + p.cross(q);
        }
        s * 0.5
    }

    /// Area-weighted fan centroid of a face (the apex if the area is zero).
    fn face_centroid(&self, face: Point) -> Point3 {
        let a = self.face_apex(face);
        let ahat = self.face_area_vector(face);
        let an = ahat.norm();
        if an == 0.0 {
            return a;
        }
        let ahat = ahat * (1.0 / an);
        let idx = self.cone(face).indices;
        let (mut acc, mut wsum) = ([0.0; 3], 0.0);
        for i in 0..idx.len() {
            let p = self.vertex_coord(idx[i]);
            let q = self.vertex_coord(idx[(i + 1) % idx.len()]);
            let w = 0.5 * (p - a).cross(q - a).dot(ahat);
            let c = Point3::new((a.x + p.x + q.x) / 3.0, (a.y + p.y + q.y) / 3.0, (a.z + p.z + q.z) / 3.0);
            lerp_sum(w, c, &mut acc);
            wsum += w;
        }
        Point3::new(acc[0] / wsum, acc[1] / wsum, acc[2] / wsum)
    }

    /// Signed volume and centroid of the side-tet decomposition of a cell.
    fn cell_volume_centroid(&self, cell: Point) -> (f64, Point3) {
        let vs: Vec<Point3> = self.closure_vertices(cell).iter().map(|&v| self.vertex_coord(v)).collect();
        let o = mean(&vs);
        let cone = self.cone(cell);
        let (mut vol, mut acc) = (0.0, [0.0; 3]);
        for (f, orient) in cone.iter() {
            let fa = self.face_apex(f);
            let mut idx: Vec<u32> = self.cone(f).indices.to_vec();
            if orient == Orient::Rev {
                idx.reverse();
            }
            for i in 0..idx.len() {
                let p = self.vertex_coord(idx[i]);
                let q = self.vertex_coord(idx[(i + 1) % idx.len()]);
                let v = (fa - o).dot((p - o).cross(q - o)) / 6.0;
                let c = Point3::new((o.x + fa.x + p.x + q.x) / 4.0, (o.y + fa.y + p.y + q.y) / 4.0, (o.z + fa.z + p.z + q.z) / 4.0);
                lerp_sum(v, c, &mut acc);
                vol += v;
            }
        }
        if vol == 0.0 { (0.0, o) } else { (vol, Point3::new(acc[0] / vol, acc[1] / vol, acc[2] / vol)) }
    }

    /// Signed cell volume (positive when the outward-loop convention holds).
    fn cell_volume(&self, cell: Point) -> f64 {
        self.cell_volume_centroid(cell).0
    }

    /// Cell centroid under the fan rule.
    fn cell_centroid(&self, cell: Point) -> Point3 {
        self.cell_volume_centroid(cell).1
    }
}

impl<T: MeshTopology + MeshGeometry + ?Sized> MeshMetrics for T {}
