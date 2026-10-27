//! Mesh optimization: quality flips (maximize the minimum mean ratio) and constrained smoothing.
//!
//! Flips never remove a boundary face or edge (boundary faces have no neighbour and boundary
//! edges have open rings). Smoothing moves interior (Steiner) vertices only, by colour class:
//! vertices of one class share no tet, so their updates are independent and written back in
//! index order; the result is bit-identical for any thread count.

use crate::delaunay::Dt;
use crate::error::TetError;
use crate::flips::{FlipScratch, Plan};
use crate::geom::{V3, add, mean_ratio, ori, scale, sub};
use crate::guard::Guard;
use rayon::prelude::*;

impl Dt {
    /// One sweep of quality flips over tets with mean ratio below `threshold`. Returns the
    /// number of flips applied.
    pub(crate) fn flip_pass(&mut self, threshold: f64, sc: &mut FlipScratch) -> Result<usize, TetError> {
        let mut flips = 0;
        let n_slots = self.tds.slots() as u32;
        for t in 0..n_slots {
            if !self.tds.alive[t as usize] {
                continue;
            }
            let cur = self.min_quality(&[t]);
            if cur >= threshold {
                continue;
            }
            let v = self.tds.verts[t as usize];
            let mut best: Option<(f64, Plan)> = None;
            let consider = |plan: Plan, this: &Dt, best: &mut Option<(f64, Plan)>| {
                let old_q = this.min_quality(&plan.old);
                if plan.min_q > old_q + 1e-9 && best.as_ref().is_none_or(|b| plan.min_q > b.1.min_q) {
                    *best = Some((old_q, plan));
                }
            };
            for i in 0..4 {
                for j in i + 1..4 {
                    if let Some(plan) = self.plan_edge_removal(v[i], v[j], t, 12, sc) {
                        consider(plan, self, &mut best);
                    }
                }
                if let Some(plan) = self.plan_flip23(t, i) {
                    consider(plan, self, &mut best);
                }
            }
            if let Some((_, plan)) = best {
                self.apply(&plan)?;
                flips += 1;
            }
        }
        Ok(flips)
    }

    /// Repeated flip sweeps; returns the total number of flips.
    pub(crate) fn flip_optimize(&mut self, threshold: f64, max_sweeps: usize) -> Result<usize, TetError> {
        let mut sc = FlipScratch::default();
        let mut total = 0;
        for _ in 0..max_sweeps {
            let n = self.flip_pass(threshold, &mut sc)?;
            total += n;
            if n == 0 {
                break;
            }
        }
        Ok(total)
    }

    /// Laplacian smoothing of the vertices `>= first_movable` that are used by a tet, with a
    /// backtracking line search that only accepts moves that raise the local minimum mean ratio
    /// and keep every incident tet strictly positive. Stops early when a pass gains less than
    /// `1e-4` per movable vertex on average. Moves respect the S4 guard. Returns the number of passes performed.
    pub(crate) fn smooth(&mut self, first_movable: usize, max_passes: usize, guard: &Guard<'_>) -> usize {
        let np = self.pts.len();
        // Vertex -> tets (CSR). The topology does not change while smoothing.
        let mut count = vec![0u32; np + 1];
        let mut live: Vec<u32> = Vec::new();
        for t in 0..self.tds.slots() as u32 {
            if self.tds.alive[t as usize] {
                live.push(t);
                for &x in &self.tds.verts[t as usize] {
                    count[x as usize + 1] += 1;
                }
            }
        }
        for i in 0..np {
            count[i + 1] += count[i];
        }
        let mut fill = count.clone();
        let mut inc = vec![0u32; count[np] as usize];
        for &t in &live {
            for &x in &self.tds.verts[t as usize] {
                inc[fill[x as usize] as usize] = t;
                fill[x as usize] += 1;
            }
        }
        let movable: Vec<u32> = (first_movable..np).filter(|&v| count[v + 1] > count[v]).map(|v| v as u32).collect();
        if movable.is_empty() {
            return 0;
        }
        // Greedy colouring in index order over the vertex adjacency.
        let mut color = vec![u32::MAX; np];
        let mut classes: Vec<Vec<u32>> = Vec::new();
        let mut forbidden: Vec<u32> = Vec::new();
        for &v in &movable {
            forbidden.clear();
            for &t in &inc[count[v as usize] as usize..count[v as usize + 1] as usize] {
                for &x in &self.tds.verts[t as usize] {
                    if x != v && color[x as usize] != u32::MAX {
                        forbidden.push(color[x as usize]);
                    }
                }
            }
            forbidden.sort_unstable();
            forbidden.dedup();
            let mut c = 0;
            for &f in &forbidden {
                if f == c {
                    c += 1;
                } else if f > c {
                    break;
                }
            }
            color[v as usize] = c;
            if classes.len() <= c as usize {
                classes.resize(c as usize + 1, Vec::new());
            }
            classes[c as usize].push(v);
        }
        let mut passes = 0;
        for _ in 0..max_passes {
            let mut gain = 0.0;
            let mut moved_total = 0usize;
            for class in &classes {
                let updates: Vec<Option<(V3, f64)>> = {
                    let this = &*self;
                    class
                        .par_iter()
                        .map(|&v| this.smooth_vertex(v, guard, &inc[count[v as usize] as usize..count[v as usize + 1] as usize]))
                        .collect()
                };
                for (&v, u) in class.iter().zip(updates) {
                    if let Some((p, g)) = u {
                        self.pts[v as usize] = p;
                        gain += g;
                        moved_total += 1;
                    }
                }
            }
            passes += 1;
            if moved_total == 0 || gain < 1e-4 * movable.len() as f64 {
                break;
            }
        }
        passes
    }

    fn smooth_vertex(&self, v: u32, guard: &Guard<'_>, inc: &[u32]) -> Option<(V3, f64)> {
        let x = self.pts[v as usize];
        // Neighbour centroid (each neighbour counted once per incident tet; stable and cheap).
        let mut c = [0.0; 3];
        let mut n = 0.0;
        for &t in inc {
            for &y in &self.tds.verts[t as usize] {
                if y != v {
                    c = add(c, self.pts[y as usize]);
                    n += 1.0;
                }
            }
        }
        if n == 0.0 {
            return None;
        }
        let target = scale(c, 1.0 / n);
        let local_min = |pos: V3| -> Option<f64> {
            let mut m = f64::INFINITY;
            for &t in inc {
                let w = self.tds.verts[t as usize];
                let p = |i: usize| if w[i] == v { pos } else { self.pts[w[i] as usize] };
                if ori(p(0), p(1), p(2), p(3)) <= 0 {
                    return None;
                }
                m = m.min(mean_ratio(p(0), p(1), p(2), p(3)));
            }
            Some(m)
        };
        let base = local_min(x)?;
        // Ruling S4: a move may not bring the vertex closer (relative to the triangle size) to an
        // input triangle than the bound, or than it already is.
        let floor = guard.margin(x);
        let mut alpha = 1.0;
        for _ in 0..5 {
            let cand = add(x, scale(sub(target, x), alpha));
            if let Some(m) = local_min(cand)
                && m > base + 1e-12
                && guard.margin(cand) >= floor
            {
                return Some((cand, m - base));
            }
            alpha *= 0.5;
        }
        None
    }
}
