//! Bistellar flip planning: 2-3 face flips and n-to-m edge removal (Klincsek DP over the ring
//! polygon). Plans are computed without touching the triangulation; callers apply them with
//! [`crate::tds::Tds::replace`].
//!
//! Every new tet must have strictly positive *exact* orientation, so flips never create flat
//! tets. The 3-2 flip is edge removal with a ring of three.

use crate::delaunay::Dt;
use crate::geom::{mean_ratio, ori};
use crate::tds::GHOST;

/// A planned replacement of `old` tets by `new` tets.
pub(crate) struct Plan {
    pub old: Vec<u32>,
    pub new: Vec<[u32; 4]>,
    /// Smallest mean ratio among the new tets.
    pub min_q: f64,
}

/// Scratch buffers for edge removal planning.
#[derive(Default)]
pub(crate) struct FlipScratch {
    ring: Vec<u32>,
    poly: Vec<u32>,
    best: Vec<f64>,
    choice: Vec<u16>,
}

impl Dt {
    #[inline]
    fn q(&self, v: [u32; 4]) -> Option<f64> {
        let p = |i: u32| self.pts[i as usize];
        let (a, b, c, d) = (p(v[0]), p(v[1]), p(v[2]), p(v[3]));
        if ori(a, b, c, d) <= 0 {
            return None;
        }
        Some(mean_ratio(a, b, c, d))
    }

    /// Plan the removal of edge `(a,b)` (a tet `t0` containing it is given). `None` if the ring is
    /// open, touches a ghost, is longer than `max_ring` or no valid triangulation of the ring
    /// polygon exists.
    pub(crate) fn plan_edge_removal(&self, a: u32, b: u32, t0: u32, max_ring: usize, sc: &mut FlipScratch) -> Option<Plan> {
        if !self.tds.ring(a, b, t0, max_ring, &mut sc.ring, &mut sc.poly) {
            return None;
        }
        let k = sc.ring.len();
        if k < 3 || sc.ring.iter().any(|&t| self.tds.ghost_slot(t).is_some()) {
            return None;
        }
        // Orient the polygon so that [a,b,q_i,q_{i+1}] is positive.
        if self.sos().ori_s(a, b, sc.poly[0], sc.poly[1]) < 0 {
            sc.poly.reverse();
        }
        let qv = &sc.poly;
        let tri_q = |i: usize, j: usize, l: usize| -> f64 {
            let ta = self.q([a, qv[i], qv[j], qv[l]]);
            let tb = self.q([b, qv[i], qv[l], qv[j]]);
            match (ta, tb) {
                (Some(x), Some(y)) => x.min(y),
                _ => f64::NEG_INFINITY,
            }
        };
        let n = k;
        sc.best.clear();
        sc.best.resize(n * n, f64::NEG_INFINITY);
        sc.choice.clear();
        sc.choice.resize(n * n, 0);
        for i in 0..n - 1 {
            sc.best[i * n + i + 1] = f64::INFINITY;
        }
        for len in 2..n {
            for i in 0..n - len {
                let j = i + len;
                let mut bv = f64::NEG_INFINITY;
                let mut bm = 0u16;
                for m in i + 1..j {
                    let l = sc.best[i * n + m];
                    let r = sc.best[m * n + j];
                    if l == f64::NEG_INFINITY || r == f64::NEG_INFINITY {
                        continue;
                    }
                    let q = tri_q(i, m, j).min(l).min(r);
                    if q > bv {
                        bv = q;
                        bm = m as u16;
                    }
                }
                sc.best[i * n + j] = bv;
                sc.choice[i * n + j] = bm;
            }
        }
        let total = sc.best[n - 1];
        if total == f64::NEG_INFINITY {
            return None;
        }
        // Reconstruct.
        let mut new = Vec::with_capacity(2 * (n - 2));
        let mut stack = vec![(0usize, n - 1)];
        while let Some((i, j)) = stack.pop() {
            if j <= i + 1 {
                continue;
            }
            let m = sc.choice[i * n + j] as usize;
            new.push([a, qv[i], qv[m], qv[j]]);
            new.push([b, qv[i], qv[j], qv[m]]);
            stack.push((i, m));
            stack.push((m, j));
        }
        Some(Plan { old: sc.ring.clone(), new, min_q: total })
    }

    /// Plan the 2-3 flip of face `i` of tet `t`. `None` if either tet is a ghost, the face is on
    /// the boundary or the flip would create a non-positive tet.
    pub(crate) fn plan_flip23(&self, t: u32, i: usize) -> Option<Plan> {
        let n = self.tds.nbr[t as usize][i];
        if n == crate::tds::NONE || self.tds.ghost_slot(t).is_some() || self.tds.ghost_slot(n).is_some() {
            return None;
        }
        let s = self.tds.verts[t as usize][i];
        let [p, q, r] = self.tds.face(t, i);
        let j = self.tds.back_slot(n, t)?;
        let u = self.tds.verts[n as usize][j];
        debug_assert!(s != GHOST && u != GHOST);
        let cand = [[s, u, p, q], [s, u, q, r], [s, u, r, p]];
        let pp = |x: u32| self.pts[x as usize];
        let sg: Vec<i32> = cand.iter().map(|c| ori(pp(c[0]), pp(c[1]), pp(c[2]), pp(c[3]))).collect();
        let new: Vec<[u32; 4]> = if sg.iter().all(|&x| x > 0) {
            cand.to_vec()
        } else if sg.iter().all(|&x| x < 0) {
            cand.iter().map(|c| [c[0], c[1], c[3], c[2]]).collect()
        } else {
            return None;
        };
        let mut min_q = f64::INFINITY;
        for v in &new {
            min_q = min_q.min(self.q(*v)?);
        }
        Some(Plan { old: vec![t, n], new, min_q })
    }

    /// Smallest mean ratio of the given tets (live tet ids).
    pub(crate) fn min_quality(&self, tets: &[u32]) -> f64 {
        let mut m = f64::INFINITY;
        for &t in tets {
            let v = self.tds.verts[t as usize];
            let p = |i: usize| self.pts[v[i] as usize];
            m = m.min(mean_ratio(p(0), p(1), p(2), p(3)));
        }
        m
    }

    /// Apply a plan; returns the ids of the new tets.
    pub(crate) fn apply(&mut self, plan: &Plan) -> Result<Vec<u32>, crate::TetError> {
        let mut ids = Vec::new();
        self.tds.replace(&plan.old, &plan.new, &mut ids)?;
        Ok(ids)
    }
}

impl Dt {
    /// Remove tets of exactly zero volume (ghost tets are skipped) by edge-removal and 2-3 flips
    /// that create strictly positive tets only, falling back to a cavity refill or a Steiner
    /// point. Returns the number of flips. With `strict`, flats that cannot be removed are an
    /// error; otherwise they are left in place.
    pub(crate) fn remove_flats(&mut self, strict: bool, guard: &crate::guard::Guard<'_>) -> Result<usize, crate::TetError> {
        let mut sc = FlipScratch::default();
        let mut flips = 0;
        for _round in 0..64 {
            let mut flats: Vec<u32> = (0..self.tds.slots() as u32)
                .filter(|&t| {
                    self.tds.alive[t as usize] && self.tds.ghost_slot(t).is_none() && {
                        let v = self.tds.verts[t as usize];
                        let p = |i: usize| self.pts[v[i] as usize];
                        ori(p(0), p(1), p(2), p(3)) <= 0
                    }
                })
                .collect();
            if flats.is_empty() {
                return Ok(flips);
            }
            flats.sort_by_key(|&t| {
                let mut v = self.tds.verts[t as usize];
                v.sort_unstable();
                v
            });
            let mut progressed = false;
            for f in flats {
                if !self.tds.alive[f as usize] {
                    continue;
                }
                let v = self.tds.verts[f as usize];
                let p = |i: usize| self.pts[v[i] as usize];
                if ori(p(0), p(1), p(2), p(3)) > 0 {
                    continue;
                }
                let mut best: Option<Plan> = None;
                let consider = |plan: Plan, best: &mut Option<Plan>| {
                    if best.as_ref().is_none_or(|b| plan.min_q > b.min_q) {
                        *best = Some(plan);
                    }
                };
                for i in 0..4 {
                    for j in i + 1..4 {
                        if let Some(plan) = self.plan_edge_removal(v[i], v[j], f, 24, &mut sc) {
                            consider(plan, &mut best);
                        }
                    }
                }
                for k in 0..4 {
                    if let Some(plan) = self.plan_flip23(f, k) {
                        consider(plan, &mut best);
                    }
                }
                if let Some(plan) = best {
                    self.apply(&plan)?;
                    flips += 1;
                    progressed = true;
                } else if self.flat_by_cavity(f, guard)? {
                    flips += 1;
                    progressed = true;
                }
            }
            if !progressed {
                break;
            }
        }
        if strict {
            return Err(crate::TetError::Internal("could not remove zero-volume tets from the mesh".into()));
        }
        Ok(flips)
    }
}

impl Dt {
    /// Replace the flat tet `f` and a growing neighbourhood by an exactly valid tetrahedralization
    /// of the same region (backtracking gift wrapping).
    fn flat_by_cavity(&mut self, f: u32, guard: &crate::guard::Guard<'_>) -> Result<bool, crate::TetError> {
        use crate::tds::NONE;
        let mut r: Vec<u32> = vec![f];
        for _level in 0..4 {
            // Grow by one layer.
            let snapshot = r.clone();
            for &t in &snapshot {
                for k in 0..4 {
                    let n = self.tds.nbr[t as usize][k];
                    if n != NONE && !r.contains(&n) && self.tds.ghost_slot(n).is_none() {
                        r.push(n);
                    }
                }
            }
            if r.len() > 80 {
                break;
            }
            let mut front: Vec<[u32; 3]> = Vec::new();
            for &t in &r {
                for k in 0..4 {
                    let n = self.tds.nbr[t as usize][k];
                    if n == NONE || !r.contains(&n) {
                        front.push(self.tds.face(t, k));
                    }
                }
            }
            let mut bverts: Vec<u32> = front.iter().flatten().copied().collect();
            bverts.sort_unstable();
            bverts.dedup();
            if r.iter().any(|&t| self.tds.verts[t as usize].iter().any(|x| bverts.binary_search(x).is_err())) {
                continue;
            }
            if let Some((new, _)) = self.fill_with_steiner(&front, None, 20_000 + 400 * front.len(), guard) {
                let mut ids = Vec::new();
                self.tds.replace(&r, &new, &mut ids)?;
                return Ok(true);
            }
            // Not tetrahedralizable with its own vertices: add an interior Steiner point.
            if let Some(p) = self.kernel_point(&front, guard) {
                let id = self.push_point(p);
                let new: Vec<[u32; 4]> = front.iter().map(|f| [id, f[0], f[1], f[2]]).collect();
                let mut ids = Vec::new();
                self.tds.replace(&r, &new, &mut ids)?;
                return Ok(true);
            }
        }
        Ok(false)
    }
}

impl Dt {
    /// Locally-Delaunay flips on interior faces (constrained: boundary faces have no neighbour
    /// and never move). A face whose neighbour apex lies inside the circumsphere is flipped (2-3)
    /// when the union is convex, or via the 3-2 flip around one of its edges when that edge has
    /// exactly three tets. Returns the number of flips; bounded work, no guarantee of termination
    /// on degenerate input.
    pub(crate) fn delaunay_flips(&mut self) -> Result<usize, crate::TetError> {
        use crate::geom::in_sphere;
        use crate::tds::NONE;
        let mut sc = FlipScratch::default();
        let mut flips = 0usize;
        let budget = 20 * self.tds.len() + 1000;
        for _sweep in 0..12 {
            let mut changed = false;
            let n_slots = self.tds.slots() as u32;
            for t in 0..n_slots {
                if !self.tds.alive[t as usize] {
                    continue;
                }
                for i in 0..4 {
                    if !self.tds.alive[t as usize] {
                        break;
                    }
                    let n = self.tds.nbr[t as usize][i];
                    if n == NONE || n < t {
                        continue;
                    }
                    let v = self.tds.verts[t as usize];
                    let p = |k: usize| self.pts[v[k] as usize];
                    let Some(j) = self.tds.back_slot(n, t) else { continue };
                    let apex = self.pts[self.tds.verts[n as usize][j] as usize];
                    if !in_sphere(p(0), p(1), p(2), p(3), apex) {
                        continue;
                    }
                    if let Some(plan) = self.plan_flip23(t, i) {
                        self.apply(&plan)?;
                        flips += 1;
                        changed = true;
                    } else {
                        let f = self.tds.face(t, i);
                        for e in 0..3 {
                            let (a, b) = (f[e], f[(e + 1) % 3]);
                            if let Some(plan) = self.plan_edge_removal(a, b, t, 3, &mut sc)
                                && plan.old.len() == 3
                                && plan.old.contains(&n)
                            {
                                self.apply(&plan)?;
                                flips += 1;
                                changed = true;
                                break;
                            }
                        }
                    }
                    if flips > budget {
                        return Ok(flips);
                    }
                }
            }
            if !changed {
                break;
            }
        }
        Ok(flips)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::V3;

    /// Three tets around the axis a-b (a above, b below); the 3-2 flip must produce two tets.
    #[test]
    fn three_two_flip_orientation() {
        let pts: Vec<V3> = vec![
            [0.0, 0.0, 1.0],  // a 0
            [0.0, 0.0, -1.0], // b 1
            [1.0, 0.0, 0.0],  // 2
            [-0.5, 0.866, 0.0],
            [-0.5, -0.866, 0.0],
        ];
        let mut dt = Dt::new(pts);
        let mk = |x: u32, y: u32| {
            let mut v = [0, 1, x, y];
            if ori(dt_p(&dt_pts(), v[0]), dt_p(&dt_pts(), v[1]), dt_p(&dt_pts(), v[2]), dt_p(&dt_pts(), v[3])) < 0 {
                v.swap(2, 3);
            }
            v
        };
        fn dt_pts() -> Vec<V3> {
            vec![[0.0, 0.0, 1.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [-0.5, 0.866, 0.0], [-0.5, -0.866, 0.0]]
        }
        fn dt_p(p: &[V3], i: u32) -> V3 {
            p[i as usize]
        }
        let tets = [mk(2, 3), mk(3, 4), mk(4, 2)];
        // Close with ghost-like absent neighbours: use replace with open boundary (NONE neighbours).
        // replace() requires matching old boundary faces; with an empty old set every face must
        // pair, so add outer ghosts: build the full 5-point hull via the Delaunay builder instead.
        let _ = (&mut dt, tets);
        let dt = crate::delaunay::build(dt_pts(), None, None).unwrap();
        let mut sc = FlipScratch::default();
        let t0 = (0..dt.tds.slots() as u32).find(|&t| dt.tds.alive[t as usize] && dt.tds.has(t, 0) && dt.tds.has(t, 1) && dt.tds.ghost_slot(t).is_none());
        // The Delaunay of this bipyramid contains the axis edge only if the 3 equatorial tets exist.
        if let Some(t0) = t0 {
            let plan = dt.plan_edge_removal(0, 1, t0, 16, &mut sc).expect("3-2 flip valid");
            assert_eq!(plan.old.len(), 3);
            assert_eq!(plan.new.len(), 2);
            for v in &plan.new {
                let p = |i: usize| dt.pts[v[i] as usize];
                assert_eq!(ori(p(0), p(1), p(2), p(3)), 1);
            }
        }
    }
}
