//! Delaunay refinement of the classified mesh with a frozen boundary.
//!
//! Bad tets (circumradius / shortest edge above the bound, or circumradius above 75% of the
//! local target size) are fixed by inserting their circumcenter through a *constrained cavity*:
//! the cavity grows only through interior faces (boundary faces have no neighbour), must be
//! star-shaped around the new point, and keeps the boundary untouched. A point that lies
//! outside, closer than 0.3 h to a vertex or a boundary triangle, or breaks star-shape is
//! retried on the segment from the tet centroid to the circumcenter (t = 1/2, 1/4) before the
//! tet is given up as unrefinable.

use crate::delaunay::Dt;
use crate::error::TetError;
use crate::geom::{V3, add, circumsphere, dist, dist2_point_tri, in_sphere, norm2, ori, sanitize, scale, sub};
use crate::tds::{FACE, NONE};
use crate::{SizeSpec, TetOptions};
use rayon::prelude::*;
use std::cmp::Reverse;
use std::collections::BinaryHeap;

/// Refinement state.
pub(crate) struct Refiner<'a, 'o> {
    pub dt: &'a mut Dt,
    /// Target size per point.
    pub h: Vec<f64>,
    opts: &'a TetOptions<'o>,
    pub n_steiner: usize,
    /// Repair flips applied.
    pub flips: usize,
    pub max_steiner: usize,
    pub n_real: usize,
    heap: BinaryHeap<(u64, Reverse<[u32; 4]>, u32)>,
    failed: Vec<[u32; 4]>,
    mark: Vec<u32>,
    epoch: u32,
    rng: u64,
    /// Failure reasons of insertion attempts (diagnostics).
    pub reasons: std::collections::BTreeMap<&'static str, usize>,
}

fn sorted4(mut v: [u32; 4]) -> [u32; 4] {
    v.sort_unstable();
    v
}

impl<'a, 'o> Refiner<'a, 'o> {
    pub fn new(dt: &'a mut Dt, h: Vec<f64>, opts: &'a TetOptions<'o>, max_steiner: usize, n_real: usize) -> Self {
        Self { dt, h, opts, n_steiner: 0, flips: 0, max_steiner, n_real, heap: BinaryHeap::new(), failed: Vec::new(), mark: Vec::new(), epoch: 0, rng: 0x5EED, reasons: Default::default() }
    }

    /// Target size at `p` as seen from the corners `v`.
    ///
    /// `hi = min_c (h_c + (g-1)|p-c|)` is the graded upper bound from the corner sizes; the user
    /// size (constant or field) can only lower it, and never below `lo = max_c (h_c - (g-1)|p-c|)`,
    /// so a fine user size next to a coarse frozen boundary grades smoothly instead of jumping.
    fn size_at(&self, p: V3, v: [u32; 4]) -> f64 {
        let g1 = self.opts.gradation - 1.0;
        let mut hi = f64::INFINITY;
        let mut lo = 0.0f64;
        for &c in &v {
            let hc = self.h[c as usize];
            if hc.is_finite() {
                let d = g1 * dist(p, self.dt.pts[c as usize]);
                hi = hi.min(hc + d);
                lo = lo.max(hc - d);
            }
        }
        let user = match self.opts.size {
            SizeSpec::FromBoundary => f64::INFINITY,
            SizeSpec::Constant(c) => c,
            SizeSpec::Field(f) => {
                let u = f(p);
                if u.is_finite() && u > 0.0 { u } else { f64::INFINITY }
            }
        };
        hi.min(user).max(lo.min(hi))
    }

    /// Badness of a tet: above 1 means it must be refined.
    fn badness(&self, t: u32) -> f64 {
        let v = self.dt.tds.verts[t as usize];
        let p = |i: usize| self.dt.pts[v[i] as usize];
        let (a, b, c, d) = (p(0), p(1), p(2), p(3));
        let mut l2min = f64::INFINITY;
        for (x, y) in [(a, b), (a, c), (a, d), (b, c), (b, d), (c, d)] {
            l2min = l2min.min(norm2(sub(x, y)));
        }
        let Some((_, r2)) = circumsphere(a, b, c, d) else { return 1e300 };
        let g = scale(add(add(a, b), add(c, d)), 0.25);
        // A tet cannot be smaller than the boundary triangles it carries: do not chase a size
        // target below what the frozen boundary allows.
        let mut rf = 0.0f64;
        for i in 0..4 {
            if self.dt.tds.nbr[t as usize][i] == NONE {
                let f = self.dt.tds.face(t, i);
                let (x, y, z) = (self.dt.pts[f[0] as usize], self.dt.pts[f[1] as usize], self.dt.pts[f[2] as usize]);
                let (la, lb, lc) = (dist(y, z), dist(x, z), dist(x, y));
                let area2 = norm2(crate::geom::cross(sub(y, x), sub(z, x))).sqrt();
                if area2 > 0.0 {
                    rf = rf.max(la * lb * lc / (2.0 * area2));
                }
            }
        }
        let hs = self.size_at(g, v);
        let re = (r2 / l2min).sqrt() / self.opts.radius_edge;
        let sz = r2.sqrt() / (0.75 * hs).max(1.2 * rf);
        re.max(sz)
    }

    fn push(&mut self, t: u32) {
        let v = self.dt.tds.verts[t as usize];
        let b = self.badness(t);
        if b > 1.0 {
            self.heap.push((b.min(1e300).to_bits(), Reverse(sorted4(v)), t));
        }
    }

    /// Run refinement to completion; returns the number of unrefinable tets.
    pub fn run(&mut self) -> Result<usize, TetError> {
        // Seed the queue: evaluate badness in parallel, push sequentially in slot order.
        let slots = self.dt.tds.slots();
        let bad: Vec<f64> = {
            let me = &*self;
            (0..slots as u32)
                .into_par_iter()
                .map(|t| if me.dt.tds.alive[t as usize] { me.badness(t) } else { 0.0 })
                .collect()
        };
        for (t, &b) in bad.iter().enumerate() {
            if b > 1.0 {
                let v = self.dt.tds.verts[t];
                self.heap.push((b.min(1e300).to_bits(), Reverse(sorted4(v)), t as u32));
            }
        }
        for round in 0..40 {
            self.drain()?;
            let any = self.repair_failed()?;
            if std::env::var("TET_DEBUG").is_ok() {
                eprintln!("  refine round {round}: repaired={any}, queue {}", self.heap.len());
            }
            if !any {
                break;
            }
        }
        self.drain()?;
        // Count failed tets that still exist.
        self.failed.sort_unstable();
        self.failed.dedup();
        let mut n = 0;
        for t in 0..self.dt.tds.slots() as u32 {
            if self.dt.tds.alive[t as usize] && self.failed.binary_search(&sorted4(self.dt.tds.verts[t as usize])).is_ok() {
                n += 1;
            }
        }
        Ok(n)
    }

    fn drain(&mut self) -> Result<(), TetError> {
        while let Some((_, Reverse(key), t)) = self.heap.pop() {
            if !self.dt.tds.alive[t as usize] || sorted4(self.dt.tds.verts[t as usize]) != key {
                continue;
            }
            if self.n_steiner >= self.max_steiner {
                return Err(TetError::SteinerBudget { limit: self.max_steiner });
            }
            if !self.refine_tet(t)? {
                if std::env::var("TET_DEBUG2").is_ok() {
                    let v = self.dt.tds.verts[t as usize];
                    let p = |i: usize| self.dt.pts[v[i] as usize];
                    let (a, b, c, d) = (p(0), p(1), p(2), p(3));
                    let g = scale(add(add(a, b), add(c, d)), 0.25);
                    let hs = self.size_at(g, v);
                    let r = circumsphere(a, b, c, d).map(|x| x.1.sqrt()).unwrap_or(f64::NAN);
                    let q = crate::quality::tet_quality([a, b, c, d]);
                    let nb = (0..4).filter(|&i| self.dt.tds.nbr[t as usize][i] == NONE).count();
                    eprintln!("  exact ori={} coords {:?} {:?} {:?} {:?}", ori(a, b, c, d), a, b, c, d);
                    eprintln!("  FAILED tet {:?} bfaces={nb} R={r:.4} h={hs:.4} R/h={:.3} re={:.3} mr={:.4} verts<n_real={}", v, r / hs, q.radius_edge, q.mean_ratio, v.iter().filter(|&&x| (x as usize) < self.n_real).count());
                }
                self.failed.push(key);
            }
        }
        Ok(())
    }

    /// Try to repair tets that insertion could not fix by a quality-improving flip around them.
    /// Returns `true` if any flip was applied (new work may have been queued).
    fn repair_failed(&mut self) -> Result<bool, TetError> {
        self.failed.sort_unstable();
        self.failed.dedup();
        let mut index: Vec<([u32; 4], u32)> = Vec::new();
        for t in 0..self.dt.tds.slots() as u32 {
            if self.dt.tds.alive[t as usize] {
                let k = sorted4(self.dt.tds.verts[t as usize]);
                if self.failed.binary_search(&k).is_ok() {
                    index.push((k, t));
                }
            }
        }
        let mut sc = crate::flips::FlipScratch::default();
        let mut any = false;
        for (key, t0) in index {
            if !self.dt.tds.alive[t0 as usize] || sorted4(self.dt.tds.verts[t0 as usize]) != key {
                continue;
            }
            let v = self.dt.tds.verts[t0 as usize];
            let cur = self.dt.min_quality(&[t0]);
            let mut best: Option<crate::flips::Plan> = None;
            for i in 0..4 {
                for j in i + 1..4 {
                    if let Some(plan) = self.dt.plan_edge_removal(v[i], v[j], t0, 10, &mut sc) {
                        let old_q = self.dt.min_quality(&plan.old);
                        if plan.min_q > old_q * 1.05 + 1e-12 && plan.min_q > cur && best.as_ref().is_none_or(|b| plan.min_q > b.min_q) {
                            best = Some(plan);
                        }
                    }
                }
                if let Some(plan) = self.dt.plan_flip23(t0, i) {
                    let old_q = self.dt.min_quality(&plan.old);
                    if plan.min_q > old_q * 1.05 + 1e-12 && plan.min_q > cur && best.as_ref().is_none_or(|b| plan.min_q > b.min_q) {
                        best = Some(plan);
                    }
                }
            }
            if let Some(plan) = best {
                let ids = self.dt.apply(&plan)?;
                for t in ids {
                    self.push(t);
                }
                self.flips += 1;
                any = true;
            }
        }
        Ok(any)
    }

    fn refine_tet(&mut self, t: u32) -> Result<bool, TetError> {
        let v = self.dt.tds.verts[t as usize];
        let p = |i: usize| self.dt.pts[v[i] as usize];
        let g = scale(add(add(p(0), p(1)), add(p(2), p(3))), 0.25);
        let mut cands: Vec<V3> = Vec::with_capacity(3);
        if let Some((c, _)) = circumsphere(p(0), p(1), p(2), p(3)) {
            cands.push(c);
            let d = sub(c, g);
            cands.push(add(g, scale(d, 0.5)));
            cands.push(add(g, scale(d, 0.25)));
        } else {
            cands.push(g);
        }
        for c in cands {
            if self.try_insert(c, t)? {
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn marked(&self, t: u32) -> bool {
        self.mark.get(t as usize).copied() == Some(self.epoch)
    }
    fn set_mark(&mut self, t: u32) {
        if self.mark.len() < self.dt.tds.slots() {
            self.mark.resize(self.dt.tds.slots() + 64, 0);
        }
        self.mark[t as usize] = self.epoch;
    }

    /// Locate the tet containing `p` (closed), starting from `start`; `None` if `p` is outside.
    fn locate(&mut self, p: V3, start: u32) -> Option<u32> {
        let mut t = start;
        for _ in 0..4000 {
            let v = self.dt.tds.verts[t as usize];
            self.rng = crate::delaunay::splitmix(self.rng);
            let i0 = (self.rng >> 33) as usize & 3;
            let mut moved = false;
            for k in 0..4 {
                let i = (i0 + k) & 3;
                let f = FACE[i];
                if ori(self.dt.pts[v[f[0]] as usize], self.dt.pts[v[f[1]] as usize], self.dt.pts[v[f[2]] as usize], p) > 0 {
                    let n = self.dt.tds.nbr[t as usize][i];
                    if n == NONE {
                        return None;
                    }
                    t = n;
                    moved = true;
                    break;
                }
            }
            if !moved {
                return Some(t);
            }
        }
        None
    }

    fn try_insert(&mut self, p: V3, start: u32) -> Result<bool, TetError> {
        let r = self.try_insert_inner(p, start)?;
        if let Err(why) = r {
            *self.reasons.entry(why).or_insert(0) += 1;
        }
        Ok(r.is_ok())
    }

    fn try_insert_inner(&mut self, p: V3, start: u32) -> Result<Result<(), &'static str>, TetError> {
        let Some(p) = sanitize(p) else { return Ok(Err("sanitize")) };
        let Some(t0) = self.locate(p, start) else { return Ok(Err("outside")) };
        let hp = self.size_at(p, self.dt.tds.verts[t0 as usize]);
        let min_d = 0.3 * hp;
        // p may not sit on the domain boundary.
        for i in 0..4 {
            if self.dt.tds.nbr[t0 as usize][i] == NONE {
                let f = self.dt.tds.face(t0, i);
                if ori(self.dt.pts[f[0] as usize], self.dt.pts[f[1] as usize], self.dt.pts[f[2] as usize], p) >= 0 {
                    return Ok(Err("on_boundary"));
                }
            }
        }
        // Cavity: conflicting tets reachable through interior faces, grown until star-shaped.
        self.epoch = self.epoch.wrapping_add(1);
        let mut cav: Vec<u32> = vec![t0];
        self.set_mark(t0);
        let mut head = 0;
        let mut guard = 0;
        let bfaces: Vec<(u32, usize)> = loop {
            while head < cav.len() {
                let t = cav[head];
                head += 1;
                for k in 0..4 {
                    let n = self.dt.tds.nbr[t as usize][k];
                    if n == NONE || self.marked(n) {
                        continue;
                    }
                    let v = self.dt.tds.verts[n as usize];
                    let q = |i: usize| self.dt.pts[v[i] as usize];
                    if in_sphere(q(0), q(1), q(2), q(3), p) {
                        self.set_mark(n);
                        cav.push(n);
                    }
                }
            }
            let mut bad_neighbor = None;
            let mut bf: Vec<(u32, usize)> = Vec::new();
            for &t in &cav {
                for k in 0..4 {
                    let n = self.dt.tds.nbr[t as usize][k];
                    if n != NONE && self.marked(n) {
                        continue;
                    }
                    let f = self.dt.tds.face(t, k);
                    let o = ori(self.dt.pts[f[0] as usize], self.dt.pts[f[1] as usize], self.dt.pts[f[2] as usize], p);
                    if o >= 0 {
                        if n == NONE {
                            return Ok(Err("star_none"));
                        }
                        bad_neighbor = Some(n);
                        break;
                    }
                    bf.push((t, k));
                }
                if bad_neighbor.is_some() {
                    break;
                }
            }
            match bad_neighbor {
                None => break bf,
                Some(n) => {
                    guard += 1;
                    if guard > 40 || cav.len() > 400 {
                        return Ok(Err("grow_limit"));
                    }
                    self.set_mark(n);
                    cav.push(n);
                }
            }
        };
        // Vertices of the cavity: none may be enclosed, none closer than min_d.
        let mut bverts: Vec<u32> = Vec::with_capacity(bfaces.len() * 3);
        for &(t, k) in &bfaces {
            bverts.extend(self.dt.tds.face(t, k));
        }
        bverts.sort_unstable();
        bverts.dedup();
        for &t in &cav {
            for &x in &self.dt.tds.verts[t as usize] {
                if bverts.binary_search(&x).is_err() {
                    return Ok(Err("enclosed"));
                }
            }
        }
        for &x in &bverts {
            if dist(p, self.dt.pts[x as usize]) < min_d {
                return Ok(Err("near_vertex"));
            }
        }
        // Distance to boundary triangles of the cavity.
        for &t in &cav {
            for k in 0..4 {
                if self.dt.tds.nbr[t as usize][k] == NONE {
                    let f = self.dt.tds.face(t, k);
                    if dist2_point_tri(p, self.dt.pts[f[0] as usize], self.dt.pts[f[1] as usize], self.dt.pts[f[2] as usize]) < min_d * min_d {
                        return Ok(Err("near_tri"));
                    }
                }
            }
        }
        let id = self.dt.push_point(p);
        self.h.push(hp);
        let new: Vec<[u32; 4]> = bfaces
            .iter()
            .map(|&(t, k)| {
                let f = self.dt.tds.face(t, k);
                [id, f[0], f[1], f[2]]
            })
            .collect();
        let mut ids = Vec::new();
        if let Err(e) = self.dt.tds.replace(&cav, &new, &mut ids) {
            self.dt.pts.pop();
            self.h.pop();
            return Err(e);
        }
        self.n_steiner += 1;
        for t in ids {
            self.push(t);
        }
        Ok(Ok(()))
    }
}
