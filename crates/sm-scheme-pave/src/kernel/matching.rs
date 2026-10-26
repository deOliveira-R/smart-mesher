//! Triangle-to-quad recombination: a perfect matching of the triangle dual graph, greedy by
//! quality first and Edmonds' blossom augmentation for the leftovers; when no perfect
//! matching exists the longest interior edge of each unmatched triangle is split (two more
//! triangles per split, the boundary is never touched) and the matching is retried.

use super::PaveError;
use super::cdt::NONE;
use super::front::Mesher;
use super::metric::Ord64;
use sm_math::{Sign, orient2d};
use std::collections::VecDeque;

/// Quality floors of the staged matching: pairs below the floor join only when nothing
/// better completes the matching.
const STAGES: [f64; 4] = [0.6, 0.35, 0.15, 1e-4];

#[derive(Clone, Copy, Debug)]
struct Cand {
    i: u32,
    j: u32,
    /// Slot of the shared edge in triangle `i` (its apex is the first quad vertex).
    slot: u8,
    min_sine: f64,
    weight: f64,
}

/// Edmonds' maximum cardinality matching (augmenting paths with blossom contraction),
/// working from an initial matching. Deterministic: roots and neighbours in index order.
struct Blossom<'a> {
    start: &'a [u32],
    adj: &'a [u32],
    mate: &'a mut Vec<u32>,
    p: Vec<u32>,
    base: Vec<u32>,
    used: Vec<bool>,
    blossom: Vec<bool>,
    mark: Vec<bool>,
    touched: Vec<u32>,
}

impl<'a> Blossom<'a> {
    fn new(start: &'a [u32], adj: &'a [u32], mate: &'a mut Vec<u32>) -> Self {
        let n = mate.len();
        Blossom { start, adj, mate, p: vec![NONE; n], base: (0..n as u32).collect(), used: vec![false; n], blossom: vec![false; n], mark: vec![false; n], touched: Vec::new() }
    }

    fn lca(&mut self, a: u32, b: u32) -> u32 {
        let mut seen: Vec<u32> = Vec::new();
        let mut a = self.base[a as usize];
        loop {
            self.mark[a as usize] = true;
            seen.push(a);
            let m = self.mate[a as usize];
            if m == NONE {
                break;
            }
            a = self.base[self.p[m as usize] as usize];
        }
        let mut b = self.base[b as usize];
        let res = loop {
            if self.mark[b as usize] {
                break b;
            }
            b = self.base[self.p[self.mate[b as usize] as usize] as usize];
        };
        for s in seen {
            self.mark[s as usize] = false;
        }
        res
    }

    fn mark_path(&mut self, mut v: u32, b: u32, mut children: u32, marked: &mut Vec<u32>) {
        while self.base[v as usize] != b {
            let (bv, bm) = (self.base[v as usize], self.base[self.mate[v as usize] as usize]);
            self.blossom[bv as usize] = true;
            self.blossom[bm as usize] = true;
            marked.push(bv);
            marked.push(bm);
            self.p[v as usize] = children;
            children = self.mate[v as usize];
            v = self.p[self.mate[v as usize] as usize];
        }
    }

    fn find_path(&mut self, root: u32) -> Option<u32> {
        for &t in &self.touched {
            self.used[t as usize] = false;
            self.p[t as usize] = NONE;
            self.base[t as usize] = t;
        }
        self.touched.clear();
        self.used[root as usize] = true;
        self.touched.push(root);
        let mut queue: VecDeque<u32> = VecDeque::new();
        queue.push_back(root);
        while let Some(v) = queue.pop_front() {
            let (s, e) = (self.start[v as usize] as usize, self.start[v as usize + 1] as usize);
            for k in s..e {
                let to = self.adj[k];
                if self.base[v as usize] == self.base[to as usize] || self.mate[v as usize] == to {
                    continue;
                }
                let mt = self.mate[to as usize];
                if to == root || (mt != NONE && self.p[mt as usize] != NONE) {
                    let cur = self.lca(v, to);
                    let mut marked: Vec<u32> = Vec::new();
                    self.mark_path(v, cur, to, &mut marked);
                    self.mark_path(to, cur, v, &mut marked);
                    let snapshot = self.touched.len();
                    for idx in 0..snapshot {
                        let i = self.touched[idx];
                        if self.blossom[self.base[i as usize] as usize] {
                            self.base[i as usize] = cur;
                            if !self.used[i as usize] {
                                self.used[i as usize] = true;
                                queue.push_back(i);
                            }
                        }
                    }
                    for m in marked {
                        self.blossom[m as usize] = false;
                    }
                } else if self.p[to as usize] == NONE {
                    self.p[to as usize] = v;
                    self.touched.push(to);
                    if mt == NONE {
                        return Some(to);
                    }
                    self.used[mt as usize] = true;
                    self.touched.push(mt);
                    queue.push_back(mt);
                }
            }
        }
        None
    }

    fn augment(&mut self, end: u32) {
        let mut v = end;
        while v != NONE {
            let pv = self.p[v as usize];
            let ppv = self.mate[pv as usize];
            self.mate[v as usize] = pv;
            self.mate[pv as usize] = v;
            v = ppv;
        }
    }

    fn run(&mut self) {
        for root in 0..self.mate.len() as u32 {
            if self.mate[root as usize] != NONE || self.start[root as usize] == self.start[root as usize + 1] {
                continue;
            }
            if let Some(end) = self.find_path(root) {
                self.augment(end);
            }
        }
    }
}

/// Match as many triangles as possible. Returns `mate` (triangle index to triangle index or
/// `NONE`).
fn match_triangles(n: usize, cands: &[Cand]) -> Vec<u32> {
    let mut order: Vec<usize> = (0..cands.len()).collect();
    order.sort_unstable_by(|a, b| Ord64(cands[*b].weight).cmp(&Ord64(cands[*a].weight)).then(cands[*a].i.cmp(&cands[*b].i)).then(cands[*a].j.cmp(&cands[*b].j)));
    let mut mate = vec![NONE; n];
    for &floor in &STAGES {
        for &c in &order {
            let c = cands[c];
            if c.min_sine >= floor && mate[c.i as usize] == NONE && mate[c.j as usize] == NONE {
                mate[c.i as usize] = c.j;
                mate[c.j as usize] = c.i;
            }
        }
        if mate.iter().all(|m| *m != NONE) {
            break;
        }
        // CSR of the pairs at or above the floor.
        let mut deg = vec![0u32; n + 1];
        for c in cands.iter().filter(|c| c.min_sine >= floor) {
            deg[c.i as usize + 1] += 1;
            deg[c.j as usize + 1] += 1;
        }
        for i in 0..n {
            deg[i + 1] += deg[i];
        }
        let mut fill = deg.clone();
        let mut adj = vec![0u32; deg[n] as usize];
        // Neighbours in ascending quality order: iterate candidates best first so that the
        // alternating search prefers good pairs.
        for &c in &order {
            let c = cands[c];
            if c.min_sine >= floor {
                adj[fill[c.i as usize] as usize] = c.j;
                fill[c.i as usize] += 1;
                adj[fill[c.j as usize] as usize] = c.i;
                fill[c.j as usize] += 1;
            }
        }
        Blossom::new(&deg, &adj, &mut mate).run();
        if mate.iter().all(|m| *m != NONE) {
            break;
        }
    }
    mate
}

/// Candidate pair of the triangles at positions `i < j` sharing the edge opposite vertex
/// `slot` of triangle `i`; `None` unless the union is a strictly convex quad.
fn candidate(m: &Mesher<'_>, t: u32, slot: usize, u: u32) -> Option<(f64, f64, [u32; 4])> {
    let tv = m.cdt.tv[t as usize];
    let (p, a, b) = (tv[slot], tv[(slot + 1) % 3], tv[(slot + 2) % 3]);
    let q = m.cdt.tv[u as usize].into_iter().find(|v| *v != a && *v != b)?;
    let pt = |v: u32| m.cdt.pts[v as usize];
    if orient2d(pt(p), pt(a), pt(q)) != Sign::Positive || orient2d(pt(q), pt(b), pt(p)) != Sign::Positive {
        return None;
    }
    let quad = [p, a, q, b];
    let bm = m.met[p as usize];
    let c: [[f64; 2]; 4] = core::array::from_fn(|k| bm.apply(pt(quad[k])));
    let mut min_sine = f64::INFINITY;
    let (mut lmin, mut lmax) = (f64::INFINITY, 0.0f64);
    for k in 0..4 {
        let (p0, p1, p2) = (c[(k + 3) % 4], c[k], c[(k + 1) % 4]);
        let e1 = [p1[0] - p0[0], p1[1] - p0[1]];
        let e2 = [p2[0] - p1[0], p2[1] - p1[1]];
        let (l1, l2) = (e1[0].hypot(e1[1]), e2[0].hypot(e2[1]));
        let s = (e1[0] * e2[1] - e1[1] * e2[0]) / (l1 * l2);
        min_sine = min_sine.min(s);
        lmin = lmin.min(l2);
        lmax = lmax.max(l2);
    }
    if !(min_sine.is_finite() && min_sine > 0.0 && lmax > 0.0) {
        return None;
    }
    Some((min_sine, min_sine * (lmin / lmax).sqrt(), quad))
}

/// Recombine the triangulation into quads. Returns the quads (counter-clockwise, vertex ids
/// of the mesher) and the number of edge-splitting rounds used.
pub(crate) fn recombine(m: &mut Mesher<'_>, max_rounds: u32) -> Result<(Vec<[u32; 4]>, u32), PaveError> {
    let mut rounds = 0u32;
    loop {
        let tris: Vec<u32> = m.cdt.triangles().collect();
        let n = tris.len();
        let mut idx = vec![NONE; m.cdt.tv.len()];
        for (i, t) in tris.iter().enumerate() {
            idx[*t as usize] = i as u32;
        }
        let mut cands: Vec<Cand> = Vec::with_capacity(3 * n / 2);
        for (i, &t) in tris.iter().enumerate() {
            for s in 0..3 {
                let u = m.cdt.tn[t as usize][s];
                if u == NONE || (idx[u as usize] as usize) < i {
                    continue;
                }
                if let Some((min_sine, weight, _)) = candidate(m, t, s, u) {
                    cands.push(Cand { i: i as u32, j: idx[u as usize], slot: s as u8, min_sine, weight });
                }
            }
        }
        let mate = match_triangles(n, &cands);
        let unmatched: Vec<u32> = (0..n as u32).filter(|i| mate[*i as usize] == NONE).collect();
        if unmatched.is_empty() {
            let mut quads = Vec::with_capacity(n / 2);
            for c in &cands {
                if mate[c.i as usize] == c.j {
                    if let Some((_, _, q)) = candidate(m, tris[c.i as usize], c.slot as usize, tris[c.j as usize]) {
                        quads.push(q);
                    }
                }
            }
            return Ok((quads, rounds));
        }
        if rounds >= max_rounds {
            return Err(PaveError::NoPerfectMatching { unmatched: unmatched.len() });
        }
        // Split the longest interior edge of every unmatched triangle.
        let mut edges: Vec<(u32, u32)> = Vec::new();
        for &i in &unmatched {
            let t = tris[i as usize];
            let v = m.cdt.tv[t as usize];
            let bm = m.met[v[0] as usize];
            let mut best: Option<(Ord64, u32, u32)> = None;
            for s in 0..3 {
                if m.cdt.tn[t as usize][s] == NONE || m.cdt.tc[t as usize][s] {
                    continue;
                }
                let (a, b) = (v[(s + 1) % 3], v[(s + 2) % 3]);
                let (pa, pb) = (m.cdt.pts[a as usize], m.cdt.pts[b as usize]);
                let l = Ord64(-bm.len([pb[0] - pa[0], pb[1] - pa[1]]));
                let key = (l, a.min(b), a.max(b));
                if best.is_none_or(|bst| key < bst) {
                    best = Some(key);
                }
            }
            if let Some((_, a, b)) = best {
                edges.push((a, b));
            }
        }
        edges.sort_unstable();
        edges.dedup();
        let mut split = 0usize;
        for (a, b) in edges {
            let Some((t, s)) = m.cdt.tri_with_edge(a, b) else { continue };
            if m.cdt.tn[t as usize][s] == NONE || m.cdt.tc[t as usize][s] {
                continue;
            }
            let (pa, pb) = (m.cdt.pts[a as usize], m.cdt.pts[b as usize]);
            let mid = [0.5 * (pa[0] + pb[0]), 0.5 * (pa[1] + pb[1])];
            if m.insert_free(mid, t)?.is_some() {
                split += 1;
            }
        }
        if split == 0 {
            return Err(PaveError::NoPerfectMatching { unmatched: unmatched.len() });
        }
        rounds += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn csr(n: usize, edges: &[(u32, u32)]) -> (Vec<u32>, Vec<u32>) {
        let mut deg = vec![0u32; n + 1];
        for (a, b) in edges {
            deg[*a as usize + 1] += 1;
            deg[*b as usize + 1] += 1;
        }
        for i in 0..n {
            deg[i + 1] += deg[i];
        }
        let mut fill = deg.clone();
        let mut adj = vec![0u32; deg[n] as usize];
        for (a, b) in edges {
            adj[fill[*a as usize] as usize] = *b;
            fill[*a as usize] += 1;
            adj[fill[*b as usize] as usize] = *a;
            fill[*b as usize] += 1;
        }
        (deg, adj)
    }

    #[test]
    fn blossom_finds_the_perfect_matching_of_an_odd_cycle_with_tails() {
        // Triangle 0-1-2 with pendant vertices 3 (on 0), 4 (on 1), and 5 (on 2): perfect
        // matching is 0-3, 1-4, 2-5, found only after contracting the odd cycle when started
        // from the bad matching 0-1.
        let edges = [(0, 1), (1, 2), (2, 0), (0, 3), (1, 4), (2, 5)];
        let (s, a) = csr(6, &edges);
        let mut mate = vec![NONE; 6];
        mate[0] = 1;
        mate[1] = 0;
        Blossom::new(&s, &a, &mut mate).run();
        assert!(mate.iter().all(|m| *m != NONE), "{mate:?}");
        for (i, m) in mate.iter().enumerate() {
            assert_eq!(mate[*m as usize] as usize, i);
        }
        // Petersen-like blossom case: a 5-cycle plus one pendant has no perfect matching.
        let edges = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 0), (0, 5)];
        let (s, a) = csr(6, &edges);
        let mut mate = vec![NONE; 6];
        Blossom::new(&s, &a, &mut mate).run();
        assert_eq!(mate.iter().filter(|m| **m == NONE).count(), 0);
    }

    #[test]
    fn blossom_reports_the_deficiency_of_a_star() {
        let edges = [(0, 1), (0, 2), (0, 3)];
        let (s, a) = csr(4, &edges);
        let mut mate = vec![NONE; 4];
        Blossom::new(&s, &a, &mut mate).run();
        assert_eq!(mate.iter().filter(|m| **m != NONE).count(), 2);
    }
}
