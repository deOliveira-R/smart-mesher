//! Ruppert/Chew-style refinement: circumcenters of triangles that are bad in 3-D, inserted by
//! Bowyer-Watson with constrained edges as walls. No Steiner point is ever placed on a
//! constrained segment; a circumcenter that would encroach one or leave the domain is rejected
//! and its triangle reported as unfixable (Triangle's `-Y` behaviour).

use super::{Cdt, Geometry2, NONE, RefineOptions, RefineReport};
use sm_core::Point3;
use std::cmp::Reverse;
use std::collections::BinaryHeap;

/// `0.67 ~ 1.15 / sqrt(3)`: a triangle is too large when its circumradius exceeds
/// `0.67 * size_factor * h`.
pub(crate) const RADIUS_FACTOR: f64 = 0.67;

/// A rejected triangle is reported as unfixable only if its size excess is above this factor.
/// A triangle on a boundary segment of length `h` has a circumcenter inside the segment's
/// diametral circle whenever `R < h / sqrt(2) = 0.707 h`, so without a Steiner point on the
/// segment `R` in `(0.67 h, 0.707 h)` cannot be split. That window is tolerance, not failure.
pub(crate) const SIZE_SLACK: f64 = 1.06;

/// A triangle with circumradius below `MIN_RADIUS * h` is never split: its circumcenter would
/// create a feature far below the target size. Without segment splitting this is what stops the
/// cascade around an input corner that is too sharp for the angle bound (Ruppert's algorithm
/// terminates there only because it splits the segments).
pub(crate) const MIN_RADIUS: f64 = 0.02;

fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Shape of one triangle measured in 3-D (or in the size metric).
#[derive(Clone, Copy, Debug)]
pub(crate) struct Measure {
    /// Sine of the smallest angle (0 for a degenerate triangle).
    pub sin_min: f64,
    /// Circumradius in units of the target size.
    pub r_over_h: f64,
    /// The smallest angle, radians.
    pub min_angle: f64,
}

/// Measure the triangle `(a, b, c)` of surface points.
pub(crate) fn measure(g: &dyn Geometry2, a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> Measure {
    let centroid = Point3::new(
        (a[0] + b[0] + c[0]) / 3.0,
        (a[1] + b[1] + c[1]) / 3.0,
        (a[2] + b[2] + c[2]) / 3.0,
    );
    let (mut e0, mut e1, mut e2) = (sub3(c, b), sub3(a, c), sub3(b, a));
    let h = match g.metric_map(centroid) {
        Some(m) => {
            let ap = |e: [f64; 3]| {
                [
                    m[0][0] * e[0] + m[0][1] * e[1] + m[0][2] * e[2],
                    m[1][0] * e[0] + m[1][1] * e[1] + m[1][2] * e[2],
                    m[2][0] * e[0] + m[2][1] * e[1] + m[2][2] * e[2],
                ]
            };
            e0 = ap(e0);
            e1 = ap(e1);
            e2 = ap(e2);
            1.0
        }
        None => g.size(centroid),
    };
    let (l0, l1, l2) = (
        dot3(e0, e0).sqrt(),
        dot3(e1, e1).sqrt(),
        dot3(e2, e2).sqrt(),
    );
    let k2 = {
        let cr = cross3(e1, e2);
        0.5 * dot3(cr, cr).sqrt()
    };
    // Smallest angle is opposite the shortest edge.
    let (short, x, y) = if l0 <= l1 && l0 <= l2 {
        (l0, l1, l2)
    } else if l1 <= l2 {
        (l1, l0, l2)
    } else {
        (l2, l0, l1)
    };
    let _ = short;
    let denom = x * y;
    let sin_min = if denom > 0.0 { 2.0 * k2 / denom } else { 0.0 };
    let r = if k2 > 0.0 { l0 * l1 * l2 / (4.0 * k2) } else { f64::INFINITY };
    Measure {
        sin_min: sin_min.min(1.0),
        r_over_h: r / h,
        min_angle: if sin_min > 0.0 {
            // Angle opposite the shortest edge via atan2 at the vertex between the long edges.
            let cosv = ((x * x + y * y - short * short) / (2.0 * denom)).clamp(-1.0, 1.0);
            (2.0 * k2 / denom).atan2(cosv)
        } else {
            0.0
        },
    }
}

/// Circumcenter of the chart triangle in the metric `g = [g11, g12, g22]` (identity if `None`).
fn circumcenter(a: [f64; 2], b: [f64; 2], c: [f64; 2], g: Option<[f64; 3]>) -> Option<[f64; 2]> {
    // Work from the vertex opposite the longest edge for accuracy.
    let d = |p: [f64; 2], q: [f64; 2]| (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2);
    let (ab, bc, ca) = (d(a, b), d(b, c), d(c, a));
    let (o, p, q) = if ab >= bc && ab >= ca {
        (c, a, b)
    } else if bc >= ca {
        (a, b, c)
    } else {
        (b, c, a)
    };
    let (e1, e2) = ([p[0] - o[0], p[1] - o[1]], [q[0] - o[0], q[1] - o[1]]);
    let [g11, g12, g22] = g.unwrap_or([1.0, 0.0, 1.0]);
    let u1 = [g11 * e1[0] + g12 * e1[1], g12 * e1[0] + g22 * e1[1]];
    let u2 = [g11 * e2[0] + g12 * e2[1], g12 * e2[0] + g22 * e2[1]];
    let r1 = 0.5 * (e1[0] * u1[0] + e1[1] * u1[1]);
    let r2 = 0.5 * (e2[0] * u2[0] + e2[1] * u2[1]);
    let det = u1[0] * u2[1] - u1[1] * u2[0];
    if det == 0.0 || !det.is_finite() {
        return None;
    }
    let dx = (r1 * u2[1] - r2 * u1[1]) / det;
    let dy = (u1[0] * r2 - u2[0] * r1) / det;
    let pt = [o[0] + dx, o[1] + dy];
    (pt[0].is_finite() && pt[1].is_finite()).then_some(pt)
}

impl Cdt {
    /// Lift every point not lifted yet.
    pub(super) fn ensure_lift(&mut self, g: &dyn Geometry2) {
        for i in self.pts3.len()..self.pts.len() {
            let p = g.lift(self.pts[i]);
            self.pts3.push([p.x, p.y, p.z]);
        }
    }

    pub(super) fn measure_tri(&self, g: &dyn Geometry2, v: [u32; 3]) -> Measure {
        measure(
            g,
            self.pts3[v[0] as usize],
            self.pts3[v[1] as usize],
            self.pts3[v[2] as usize],
        )
    }

    /// Badness of a triangle: above 1 it violates the angle or the size criterion.
    fn badness(&self, g: &dyn Geometry2, v: [u32; 3], sin_limit: f64, size_limit: f64) -> f64 {
        let m = self.measure_tri(g, v);
        let ang = if m.sin_min > 0.0 { sin_limit / m.sin_min } else { 1e30 };
        let sz = m.r_over_h / size_limit;
        let b = ang.max(sz);
        if b.is_finite() { b } else { 1e30 }
    }

    /// The smallest 3-D angle of the triangulation in degrees and the triangle it occurs in
    /// (`None` for an empty mesh).
    pub fn min_angle(&mut self, g: &dyn Geometry2) -> Option<(f64, [u32; 3])> {
        self.ensure_lift(g);
        let mut best: Option<(f64, [u32; 3])> = None;
        for t in self.live().collect::<Vec<_>>() {
            let v = self.tri[t as usize];
            let a = self.measure_tri(g, v).min_angle.to_degrees();
            if best.is_none_or(|(b, _)| a < b) {
                best = Some((a, v));
            }
        }
        best
    }

    /// Refine until no triangle is bad in 3-D (minimum angle below `o.min_angle` degrees, or
    /// circumradius above `o.size_factor * h * 0.67`), or only unfixable ones are left.
    ///
    /// Worst triangles are served first (largest badness, then lowest creation id). A
    /// circumcenter that would encroach a constrained segment, fall on one, or leave the region
    /// is rejected: the triangle stays and is listed in [`RefineReport::unfixable`].
    pub fn refine(&mut self, g: &dyn Geometry2, o: &RefineOptions) -> RefineReport {
        self.ensure_lift(g);
        let sin_limit = o.min_angle.to_radians().sin();
        let size_limit = RADIUS_FACTOR * o.size_factor;
        let mut report = RefineReport::default();
        let mut heap: BinaryHeap<(u64, Reverse<u32>, u32)> = BinaryHeap::new();
        let live: Vec<u32> = self.live().collect();
        for t in live {
            let b = self.badness(g, self.tri[t as usize], sin_limit, size_limit);
            if b > 1.0 {
                heap.push((b.to_bits(), Reverse(self.tid[t as usize]), t));
            }
        }
        let mut rejected: Vec<(u32, u32)> = Vec::new();
        while let Some((_, Reverse(id), t)) = heap.pop() {
            if !self.alive(t) || self.tid[t as usize] != id {
                continue;
            }
            if self.pts.len() >= o.max_points {
                report.truncated = true;
                break;
            }
            let v = self.tri[t as usize];
            if self.measure_tri(g, v).r_over_h < MIN_RADIUS * o.size_factor {
                rejected.push((t, id));
                continue;
            }
            let (a, b, c) = (self.p(v[0]), self.p(v[1]), self.p(v[2]));
            let centroid = [(a[0] + b[0] + c[0]) / 3.0, (a[1] + b[1] + c[1]) / 3.0];
            let Some(p) = circumcenter(a, b, c, g.chart_metric(centroid)) else {
                rejected.push((t, id));
                continue;
            };
            if !self.try_insert(g, p, t) {
                rejected.push((t, id));
                continue;
            }
            report.inserted += 1;
            let newt = std::mem::take(&mut self.scratch.newt);
            for &s in &newt {
                let bd = self.badness(g, self.tri[s as usize], sin_limit, size_limit);
                if bd > 1.0 {
                    heap.push((bd.to_bits(), Reverse(self.tid[s as usize]), s));
                }
            }
            self.scratch.newt = newt;
        }
        for (t, id) in rejected {
            if self.alive(t) && self.tid[t as usize] == id {
                let v = self.tri[t as usize];
                let m = self.measure_tri(g, v);
                let angle_bad = m.sin_min < sin_limit;
                let size_bad = m.r_over_h > size_limit * SIZE_SLACK;
                if angle_bad || size_bad {
                    report.unfixable.push(v);
                }
            }
        }
        report
    }

    /// Insert chart point `p` as a new Steiner vertex, locating from triangle `from`. Returns
    /// false (and changes nothing) if it would encroach a constrained segment, lie on one, be
    /// a duplicate, or fall outside the region reachable without crossing a constrained edge.
    fn try_insert(&mut self, g: &dyn Geometry2, p: [f64; 2], from: u32) -> bool {
        let Some(t0) = self.locate(p, from, true) else {
            return false;
        };
        let tv = self.tri[t0 as usize];
        if tv.iter().any(|&w| self.p(w) == p) {
            return false;
        }
        self.build_cavity(p, t0, true);
        if self.check_cavity(p, true).is_err() {
            return false;
        }
        let v = self.pts.len() as u32;
        self.pts.push(p);
        let q = g.lift(p);
        self.pts3.push([q.x, q.y, q.z]);
        self.vtri.push(NONE);
        self.apply_cavity(v);
        true
    }
}
