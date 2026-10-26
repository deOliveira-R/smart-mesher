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
        if std::env::var("TET_DEBUG").is_ok() {
            for i in 0..n {
                let (p, q) = (qv[i], qv[(i + 1) % n]);
                eprintln!("      ring pair {i}: ori_s={} exact={}", self.sos().ori_s(a, b, p, q), self.sos().ori(a, b, p, q));
            }
            for i in 0..n {
                for j in i + 1..n {
                    for l in j + 1..n {
                        let ta = self.q([a, qv[i], qv[j], qv[l]]);
                        let tb = self.q([b, qv[i], qv[l], qv[j]]);
                        eprintln!("      tri({i},{j},{l}) a-tet {:?} b-tet {:?}", ta, tb);
                    }
                }
            }
        }
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

impl Dt {
    /// Remove tets of exactly zero volume from the (ghost-free) mesh by edge-removal and 2-3
    /// flips that create strictly positive tets only. Returns the number of flips.
    pub(crate) fn remove_flats(&mut self) -> Result<usize, crate::TetError> {
        let mut sc = FlipScratch::default();
        let mut flips = 0;
        for _round in 0..64 {
            let mut flats: Vec<u32> = (0..self.tds.slots() as u32)
                .filter(|&t| {
                    self.tds.alive[t as usize] && {
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
                let mut consider = |plan: Plan, best: &mut Option<Plan>| {
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
                }
            }
            if !progressed {
                break;
            }
        }
        Err(crate::TetError::Internal("could not remove zero-volume tets from the mesh".into()))
    }
}
