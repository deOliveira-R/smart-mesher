//! Optimization: Jacobi Laplacian smoothing of the Steiner points in the chart, then flips to a
//! fixed point (exact in-circle where the chart is isometric, the 3-D opposite-angle sum
//! otherwise).

use super::refine::measure;
use super::{Cdt, Geometry2, NONE, OptimizeOptions};
use sm_core::Point3;
use sm_math::Sign;
use sm_math::predicates::{incircle, orient2d};

impl Cdt {
    /// Smooth (at most `o.sweeps` sweeps) and then flip to a fixed point.
    ///
    /// Only Steiner points move (input points, in particular boundary, embedded and pole-ring
    /// points, never do). A move is applied only if every incident triangle stays exactly
    /// counter-clockwise in the chart and the local 3-D minimum angle does not drop.
    pub fn optimize(&mut self, g: &dyn Geometry2, o: &OptimizeOptions) {
        self.ensure_lift(g);
        for _ in 0..o.sweeps {
            if !self.smooth_sweep(g) {
                break;
            }
        }
        self.flip_to_fixed_point(g);
    }

    /// The triangles around an interior vertex, or `None` for a boundary vertex.
    fn ring(&self, v: u32, out: &mut Vec<u32>) -> bool {
        out.clear();
        let start = self.vtri[v as usize];
        if start == NONE {
            return false;
        }
        let mut t = start;
        for _ in 0..=self.tri.len() {
            out.push(t);
            let k = self.index_of(t, v);
            let nx = self.nbr[t as usize][(k + 1) % 3];
            if nx == NONE {
                return false;
            }
            if nx == start {
                return true;
            }
            t = nx;
        }
        false
    }

    fn smooth_sweep(&mut self, g: &dyn Geometry2) -> bool {
        let mut ring = Vec::new();
        let mut targets: Vec<(u32, [f64; 2])> = Vec::new();
        for v in self.n_input as u32..self.pts.len() as u32 {
            if !self.ring(v, &mut ring) {
                continue;
            }
            let mut sum = [0.0; 2];
            for &t in &ring {
                let k = self.index_of(t, v);
                let w = self.p(self.tri[t as usize][(k + 1) % 3]);
                sum[0] += w[0];
                sum[1] += w[1];
            }
            let n = ring.len() as f64;
            targets.push((v, [sum[0] / n, sum[1] / n]));
        }
        let mut moved = false;
        for (v, target) in targets {
            if !self.ring(v, &mut ring) {
                continue;
            }
            let old = self.pts[v as usize];
            let old3 = self.pts3[v as usize];
            let q = g.lift(target);
            let new3 = [q.x, q.y, q.z];
            let mut before = f64::INFINITY;
            let mut after = f64::INFINITY;
            let mut ok = true;
            for &t in &ring {
                let tv = self.tri[t as usize];
                let pos = |w: u32, use_new: bool| -> [f64; 2] {
                    if w == v && use_new { target } else { self.pts[w as usize] }
                };
                let pos3 = |w: u32, use_new: bool| -> [f64; 3] {
                    if w == v && use_new { new3 } else { self.pts3[w as usize] }
                };
                if orient2d(pos(tv[0], true), pos(tv[1], true), pos(tv[2], true)) != Sign::Positive {
                    ok = false;
                    break;
                }
                // The smallest angle is at most 60 degrees, so its sine orders like the angle.
                before = before.min(
                    measure(g, pos3(tv[0], false), pos3(tv[1], false), pos3(tv[2], false)).sin_min,
                );
                after = after.min(
                    measure(g, pos3(tv[0], true), pos3(tv[1], true), pos3(tv[2], true)).sin_min,
                );
            }
            if ok && after >= before && target != old {
                self.pts[v as usize] = target;
                self.pts3[v as usize] = new3;
                moved = true;
            } else {
                self.pts[v as usize] = old;
                self.pts3[v as usize] = old3;
            }
        }
        moved
    }

    /// `(sin, cos)` of the angle at `apex` between the edges to `p` and `q`, scaled by the
    /// product of the edge lengths, in 3-D (or the metric): `(|e1 x e2|, e1 . e2)`.
    fn corner(&self, g: &dyn Geometry2, apex: u32, p: u32, q: u32) -> (f64, f64) {
        let (a, b, c) = (
            self.pts3[apex as usize],
            self.pts3[p as usize],
            self.pts3[q as usize],
        );
        let mut e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let mut e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        if let Some(m) = g.metric_map(Point3::new(a[0], a[1], a[2])) {
            let ap = |e: [f64; 3]| {
                [
                    m[0][0] * e[0] + m[0][1] * e[1] + m[0][2] * e[2],
                    m[1][0] * e[0] + m[1][1] * e[1] + m[1][2] * e[2],
                    m[2][0] * e[0] + m[2][1] * e[1] + m[2][2] * e[2],
                ]
            };
            e1 = ap(e1);
            e2 = ap(e2);
        }
        let cr = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        let dot = e1[0] * e2[0] + e1[1] * e2[1] + e1[2] * e2[2];
        ((cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt(), dot)
    }

    /// True if the angles opposite the edge `u-v` at `x` and `y` sum to more than pi (the
    /// 3-D Delaunay flip criterion). `sin(a + b) = (s1 c2 + c1 s2) / (L1 L2) < 0`, so no
    /// trigonometry is needed; the relative tolerance keeps ties from flipping back and forth.
    fn angle_sum_exceeds_pi(&self, g: &dyn Geometry2, x: u32, y: u32, u: u32, v: u32) -> bool {
        let (s1, c1) = self.corner(g, x, u, v);
        let (s2, c2) = self.corner(g, y, u, v);
        let scale = (s1.hypot(c1)) * (s2.hypot(c2));
        s1 * c2 + c1 * s2 < -1e-12 * scale
    }

    fn should_flip(&self, g: &dyn Geometry2, t: u32, i: usize) -> bool {
        let n = self.nbr[t as usize][i];
        if n == NONE || self.code[t as usize][i] != 0 {
            return false;
        }
        let tv = self.tri[t as usize];
        let x = tv[i];
        let (u, v) = (tv[(i + 1) % 3], tv[(i + 2) % 3]);
        let j = self.back_index(n, t);
        let y = self.tri[n as usize][j];
        let (o1, o2) = (
            orient2d(self.p(x), self.p(y), self.p(u)),
            orient2d(self.p(x), self.p(y), self.p(v)),
        );
        let convex = (o1 == Sign::Positive && o2 == Sign::Negative)
            || (o1 == Sign::Negative && o2 == Sign::Positive);
        if !convex {
            return false;
        }
        if g.is_isometric() {
            incircle(self.p(tv[0]), self.p(tv[1]), self.p(tv[2]), self.p(y)) == Sign::Positive
        } else {
            self.angle_sum_exceeds_pi(g, x, y, u, v)
        }
    }

    /// Flip with a worklist until no edge wants to flip. After a flip the four border edges of
    /// the quad are re-examined (their `(slot, index)` are known from the flip layout:
    /// `T1 = (a, b, d)` in slot `t` has `(b, d)` at 0 and `(a, b)` at 2; `T2 = (a, d, c)` in
    /// slot `n` has `(d, c)` at 0 and `(c, a)` at 1). Deterministic: the initial order is the
    /// slot order and the stack is LIFO.
    fn flip_to_fixed_point(&mut self, g: &dyn Geometry2) {
        let mut stack: Vec<(u32, u8)> = Vec::new();
        for t in (0..self.tri.len() as u32).rev() {
            if self.is_real(t) {
                for i in (0..3u8).rev() {
                    stack.push((t, i));
                }
            }
        }
        let mut budget = 64 * self.tri.len() + 1024;
        while let Some((t, i)) = stack.pop() {
            if budget == 0 {
                break;
            }
            if !self.is_real(t) || !self.should_flip(g, t, i as usize) {
                continue;
            }
            budget -= 1;
            let n = self.nbr[t as usize][i as usize];
            self.flip(t, i as usize);
            stack.extend([(t, 0), (t, 2), (n, 0), (n, 1)]);
        }
    }
}
