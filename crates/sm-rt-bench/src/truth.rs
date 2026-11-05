//! Analytic truth of a case, independent of every tracker: nested solids (vertical cylinders,
//! boxes, spheres) where the innermost solid containing a point (highest `depth`) gives its region.
//! Everything outside all solids is the complement (`-1`).
//!
//! The scene is brute force (every query tests every solid after a box prefilter); it exists to
//! be obviously correct, not fast.

use serde::{Deserialize, Serialize};

/// Shape of one solid.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Shape {
    /// Cylinder parallel to z.
    Cyl {
        /// Axis x, y.
        c: [f64; 2],
        /// Radius.
        r: f64,
        /// Lower z.
        z0: f64,
        /// Upper z.
        z1: f64,
    },
    /// Sphere.
    Sphere {
        /// Centre.
        c: [f64; 3],
        /// Radius.
        r: f64,
    },
    /// Axis-aligned box.
    Box {
        /// Lower corner.
        lo: [f64; 3],
        /// Upper corner.
        hi: [f64; 3],
    },
}

/// One solid with its region and nesting depth.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Solid {
    /// The shape.
    pub shape: Shape,
    /// Region id of points whose innermost solid this is.
    pub region: i32,
    /// Nesting depth (inner solids have a larger depth than the solids around them).
    pub depth: u32,
}

impl Solid {
    /// Bounding box.
    pub fn aabb(&self) -> ([f64; 3], [f64; 3]) {
        match self.shape {
            Shape::Cyl { c, r, z0, z1 } => ([c[0] - r, c[1] - r, z0], [c[0] + r, c[1] + r, z1]),
            Shape::Sphere { c, r } => ([c[0] - r, c[1] - r, c[2] - r], [c[0] + r, c[1] + r, c[2] + r]),
            Shape::Box { lo, hi } => (lo, hi),
        }
    }

    /// True when `p` is inside or on the solid.
    pub fn contains(&self, p: [f64; 3]) -> bool {
        match self.shape {
            Shape::Cyl { c, r, z0, z1 } => {
                let (dx, dy) = (p[0] - c[0], p[1] - c[1]);
                dx * dx + dy * dy <= r * r && z0 <= p[2] && p[2] <= z1
            }
            Shape::Sphere { c, r } => {
                let q = [p[0] - c[0], p[1] - c[1], p[2] - c[2]];
                q[0] * q[0] + q[1] * q[1] + q[2] * q[2] <= r * r
            }
            Shape::Box { lo, hi } => (0..3).all(|a| lo[a] <= p[a] && p[a] <= hi[a]),
        }
    }

    /// Parameter interval `(t_in, t_out)` of the unit-speed ray inside the solid (`t` may be
    /// negative), or `None`.
    pub fn interval(&self, o: [f64; 3], d: [f64; 3]) -> Option<(f64, f64)> {
        match self.shape {
            Shape::Cyl { c, r, z0, z1 } => {
                let q = [o[0] - c[0], o[1] - c[1]];
                let a = d[0] * d[0] + d[1] * d[1];
                let (mut lo, mut hi) = if a < 1e-300 {
                    if q[0] * q[0] + q[1] * q[1] > r * r {
                        return None;
                    }
                    (f64::NEG_INFINITY, f64::INFINITY)
                } else {
                    // Perpendicular-offset form: keeps a grazing chord accurate far from the axis.
                    let tc = -(q[0] * d[0] + q[1] * d[1]) / a;
                    let m = [q[0] + d[0] * tc, q[1] + d[1] * tc];
                    let m2 = m[0] * m[0] + m[1] * m[1];
                    if m2 > r * r {
                        return None;
                    }
                    let half = ((r * r - m2) / a).sqrt();
                    (tc - half, tc + half)
                };
                slab(o[2], d[2], z0, z1, &mut lo, &mut hi)?;
                (lo < hi).then_some((lo, hi))
            }
            Shape::Sphere { c, r } => {
                let q = [o[0] - c[0], o[1] - c[1], o[2] - c[2]];
                let tc = -(q[0] * d[0] + q[1] * d[1] + q[2] * d[2]);
                let m = [q[0] + d[0] * tc, q[1] + d[1] * tc, q[2] + d[2] * tc];
                let m2 = m[0] * m[0] + m[1] * m[1] + m[2] * m[2];
                if m2 > r * r {
                    return None;
                }
                let half = (r * r - m2).sqrt();
                Some((tc - half, tc + half))
            }
            Shape::Box { lo: bl, hi: bh } => {
                let (mut lo, mut hi) = (f64::NEG_INFINITY, f64::INFINITY);
                for a in 0..3 {
                    slab(o[a], d[a], bl[a], bh[a], &mut lo, &mut hi)?;
                }
                (lo < hi).then_some((lo, hi))
            }
        }
    }
}

fn slab(o: f64, d: f64, l: f64, h: f64, lo: &mut f64, hi: &mut f64) -> Option<()> {
    if d == 0.0 {
        return (l <= o && o <= h).then_some(());
    }
    let (t0, t1) = ((l - o) / d, (h - o) / d);
    let (t0, t1) = if t0 < t1 { (t0, t1) } else { (t1, t0) };
    *lo = lo.max(t0);
    *hi = hi.min(t1);
    (*lo < *hi).then_some(())
}

/// A region sequence with chords: the result of W-track for one ray.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Track {
    /// Regions visited, in order (adjacent entries differ).
    pub regions: Vec<i32>,
    /// Chord length in each region.
    pub chords: Vec<f64>,
}

/// A set of nested solids.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Scene {
    /// The solids.
    pub solids: Vec<Solid>,
    boxes: Vec<([f64; 3], [f64; 3])>,
}

impl Scene {
    /// A scene over `solids`.
    pub fn new(solids: Vec<Solid>) -> Self {
        let boxes = solids.iter().map(Solid::aabb).collect();
        Self { solids, boxes }
    }

    fn may_hit(&self, i: usize, o: [f64; 3], d: [f64; 3]) -> bool {
        let (bl, bh) = self.boxes[i];
        let (mut lo, mut hi) = (f64::NEG_INFINITY, f64::INFINITY);
        (0..3).all(|a| slab(o[a], d[a], bl[a], bh[a], &mut lo, &mut hi).is_some())
    }

    /// Region of a point (`-1` outside everything).
    pub fn region_at(&self, p: [f64; 3]) -> i32 {
        self.solids
            .iter()
            .enumerate()
            .filter(|(i, s)| {
                let (bl, bh) = self.boxes[*i];
                (0..3).all(|a| bl[a] <= p[a] && p[a] <= bh[a]) && s.contains(p)
            })
            .max_by_key(|(i, s)| (s.depth, core::cmp::Reverse(*i)))
            .map_or(-1, |(_, s)| s.region)
    }

    /// Follow a ray from `o` (inside region `start`) until it first enters the complement or
    /// leaves every solid. Adjacent chords of equal region are merged.
    pub fn track(&self, o: [f64; 3], d: [f64; 3]) -> Track {
        let mut ts: Vec<f64> = vec![0.0];
        let mut hit = Vec::new();
        for (i, s) in self.solids.iter().enumerate() {
            if !self.may_hit(i, o, d) {
                continue;
            }
            if let Some((a, b)) = s.interval(o, d)
                && b > 0.0
            {
                hit.push((i, a, b));
                ts.push(a.max(0.0));
                ts.push(b);
            }
        }
        ts.sort_by(f64::total_cmp);
        ts.dedup();
        let mut tr = Track { regions: Vec::new(), chords: Vec::new() };
        for w in ts.windows(2) {
            let (a, b) = (w[0], w[1]);
            let mid = 0.5 * (a + b);
            let region = hit
                .iter()
                .filter(|(_, x, y)| *x < mid && mid < *y)
                .max_by_key(|(i, _, _)| (self.solids[*i].depth, core::cmp::Reverse(*i)))
                .map_or(-1, |(i, _, _)| self.solids[*i].region);
            if region == -1 {
                break;
            }
            if tr.regions.last() == Some(&region) {
                if let Some(l) = tr.chords.last_mut() {
                    *l += b - a;
                }
            } else {
                tr.regions.push(region);
                tr.chords.push(b - a);
            }
        }
        tr
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pin() -> Scene {
        Scene::new(vec![
            Solid { shape: Shape::Box { lo: [-0.63, -0.63, 0.0], hi: [0.63, 0.63, 100.0] }, region: 2, depth: 0 },
            Solid { shape: Shape::Cyl { c: [0.0, 0.0], r: 0.475, z0: 0.0, z1: 100.0 }, region: 1, depth: 1 },
            Solid { shape: Shape::Cyl { c: [0.0, 0.0], r: 0.4096, z0: 0.0, z1: 100.0 }, region: 0, depth: 2 },
        ])
    }

    #[test]
    fn chords_through_a_pin_are_closed_form() {
        let s = pin();
        let (rf, rc) = (0.4096f64, 0.475f64);
        // Ray along +x at offset b = 0.2 from the axis, starting in the moderator.
        let b = 0.2f64;
        let t = s.track([-0.6, b, 50.0], [1.0, 0.0, 0.0]);
        assert_eq!(t.regions, vec![2, 1, 0, 1, 2]);
        let cf = 2.0 * (rf * rf - b * b).sqrt();
        let cc = (rc * rc - b * b).sqrt() * 2.0 - cf;
        assert!((t.chords[2] - cf).abs() < 1e-14);
        assert!((t.chords[1] + t.chords[3] - cc).abs() < 1e-14);
        let total: f64 = t.chords.iter().sum();
        assert!((total - 1.23).abs() < 1e-14);
        assert_eq!(s.region_at([0.0, 0.0, 1.0]), 0);
        assert_eq!(s.region_at([0.44, 0.0, 1.0]), 1);
        assert_eq!(s.region_at([0.6, 0.0, 1.0]), 2);
        assert_eq!(s.region_at([0.7, 0.0, 1.0]), -1);
    }

    #[test]
    fn ray_ends_at_the_first_complement() {
        let s = pin();
        let t = s.track([0.0, 0.0, 50.0], [0.0, 0.0, 1.0]);
        assert_eq!(t.regions, vec![0]);
        assert!((t.chords[0] - 50.0).abs() < 1e-12);
    }
}
