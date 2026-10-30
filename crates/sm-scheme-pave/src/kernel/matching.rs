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

/// Smallest pair quality (corner sine) the repair accepts.
const FLOOR_MIN: f64 = 1e-4;

/// The dual graph of the triangulation restricted to convex pairs. Every triangle has at
/// most three neighbours, so adjacency is a fixed array per triangle (`NONE` = no pair).
struct Graph {
    adj: Vec<[u32; 3]>,
    sine: Vec<[f64; 3]>,
    weight: Vec<[f64; 3]>,
}

/// Edmonds' maximum cardinality matching (augmenting paths with blossom contraction),
/// working from an initial matching. Deterministic: roots and neighbours in index order.
struct Blossom<'a> {
    g: &'a Graph,
    floor: f64,
    mate: &'a mut Vec<u32>,
    p: Vec<u32>,
    base: Vec<u32>,
    used: Vec<bool>,
    blossom: Vec<bool>,
    mark: Vec<bool>,
    touched: Vec<u32>,
}

impl<'a> Blossom<'a> {
    fn new(g: &'a Graph, floor: f64, mate: &'a mut Vec<u32>) -> Self {
        let n = mate.len();
        Blossom { g, floor, mate, p: vec![NONE; n], base: (0..n as u32).collect(), used: vec![false; n], blossom: vec![false; n], mark: vec![false; n], touched: Vec::new() }
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
            for k in 0..3 {
                let to = self.g.adj[v as usize][k];
                if to == NONE || self.g.sine[v as usize][k] < self.floor {
                    continue;
                }
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

    /// Augment from every exposed triangle with a neighbour.
    fn run(&mut self) {
        for root in 0..self.mate.len() as u32 {
            if self.mate[root as usize] != NONE || self.g.adj[root as usize].iter().all(|a| *a == NONE) {
                continue;
            }
            if let Some(end) = self.find_path(root) {
                self.augment(end);
            }
        }
    }
}

/// Greedy by quality, stage by stage, with blossom augmentation after each stage.
fn match_triangles(g: &Graph) -> Vec<u32> {
    let n = g.adj.len();
    let mut pairs: Vec<(Ord64, u32, u32, f64)> = Vec::with_capacity(3 * n / 2);
    for i in 0..n {
        for k in 0..3 {
            let j = g.adj[i][k];
            if j != NONE && (j as usize) > i {
                pairs.push((Ord64(g.weight[i][k]), i as u32, j, g.sine[i][k]));
            }
        }
    }
    pairs.sort_unstable_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
    let mut mate = vec![NONE; n];
    for &floor in &STAGES {
        for &(_, i, j, s) in &pairs {
            if s >= floor && mate[i as usize] == NONE && mate[j as usize] == NONE {
                mate[i as usize] = j;
                mate[j as usize] = i;
            }
        }
        if mate.iter().all(|m| *m != NONE) {
            break;
        }
        Blossom::new(g, floor, &mut mate).run();
        if mate.iter().all(|m| *m != NONE) {
            break;
        }
    }
    mate
}

/// Candidate pair of the triangles `t` and `u` sharing the edge opposite vertex `slot` of
/// `t`: `(min corner sine, weight, quad)`; `None` unless the union is a strictly convex quad.
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

/// The triangulation, its pair graph and a matching.
struct Analysis {
    tris: Vec<u32>,
    /// Triangle id to position in `tris`.
    idx: Vec<u32>,
    g: Graph,
    mate: Vec<u32>,
}

impl Analysis {
    fn unmatched(&self) -> Vec<u32> {
        (0..self.tris.len() as u32).filter(|i| self.mate[*i as usize] == NONE).collect()
    }

    /// Recompute the graph entries of the triangle at position `i` and of its neighbours.
    fn refresh(&mut self, m: &Mesher<'_>, t: u32) {
        let mut ids = vec![t];
        for s in 0..3 {
            let u = m.cdt.tn[t as usize][s];
            if u != NONE {
                ids.push(u);
            }
        }
        for t in ids {
            let i = self.idx[t as usize] as usize;
            for s in 0..3 {
                let u = m.cdt.tn[t as usize][s];
                let mut e = (NONE, 0.0, 0.0);
                if u != NONE {
                    // Canonical orientation: the lower id leads, so the pair is symmetric.
                    let c = if t < u {
                        candidate(m, t, s, u)
                    } else {
                        let su = (0..3).find(|k| m.cdt.tn[u as usize][*k] == t).unwrap_or(0);
                        candidate(m, u, su, t)
                    };
                    if let Some((sine, w, _)) = c {
                        e = (self.idx[u as usize], sine, w);
                    }
                }
                self.g.adj[i][s] = e.0;
                self.g.sine[i][s] = e.1;
                self.g.weight[i][s] = e.2;
            }
        }
    }
}

fn analyze(m: &Mesher<'_>) -> Analysis {
    let tris: Vec<u32> = m.cdt.triangles().collect();
    let n = tris.len();
    let mut idx = vec![NONE; m.cdt.tv.len()];
    for (i, t) in tris.iter().enumerate() {
        idx[*t as usize] = i as u32;
    }
    let g = Graph { adj: vec![[NONE; 3]; n], sine: vec![[0.0; 3]; n], weight: vec![[0.0; 3]; n] };
    let mut a = Analysis { tris, idx, g, mate: Vec::new() };
    for i in 0..n {
        let t = a.tris[i];
        a.refresh_one(m, t);
    }
    a.mate = match_triangles(&a.g);
    a
}

impl Analysis {
    /// Recompute only the entries of triangle `t` (used while building).
    fn refresh_one(&mut self, m: &Mesher<'_>, t: u32) {
        let i = self.idx[t as usize] as usize;
        for s in 0..3 {
            let u = m.cdt.tn[t as usize][s];
            let mut e = (NONE, 0.0, 0.0);
            if u != NONE {
                let c = if t < u {
                    candidate(m, t, s, u)
                } else {
                    let su = (0..3).find(|k| m.cdt.tn[u as usize][*k] == t).unwrap_or(0);
                    candidate(m, u, su, t)
                };
                if let Some((sine, w, _)) = c {
                    e = (self.idx[u as usize], sine, w);
                }
            }
            self.g.adj[i][s] = e.0;
            self.g.sine[i][s] = e.1;
            self.g.weight[i][s] = e.2;
        }
    }
}

/// Try to remove unmatched triangles by flipping edges near them (the triangulation stays
/// valid but need not stay Delaunay). A flip is kept only if re-augmenting lowers the number
/// of unmatched triangles; the work is bounded by an attempt budget.
fn flip_repair(m: &mut Mesher<'_>, a: &mut Analysis) {
    let mut budget = 64 + 24 * a.unmatched().len();
    for _pass in 0..6 {
        let before = a.unmatched().len();
        if before == 0 || budget == 0 {
            return;
        }
        let exposed: Vec<u32> = a.unmatched().into_iter().map(|i| a.tris[i as usize]).collect();
        for t0 in exposed {
            if a.mate[a.idx[t0 as usize] as usize] != NONE {
                continue;
            }
            // Edges of the exposed triangle, then of its neighbours.
            let mut edges: Vec<(u32, usize)> = (0..3).map(|s| (t0, s)).collect();
            for s in 0..3 {
                let n = m.cdt.tn[t0 as usize][s];
                if n == NONE {
                    continue;
                }
                for k in 0..3 {
                    if m.cdt.tn[n as usize][k] != t0 {
                        edges.push((n, k));
                    }
                }
            }
            for (t, s) in edges {
                if budget == 0 {
                    return;
                }
                let Some((u, undo)) = m.cdt.flip_if_convex(t, s) else { continue };
                m.work += a.tris.len() as u64;
                budget -= 1;
                let saved = a.mate.clone();
                let cur = saved.iter().filter(|x| **x == NONE).count();
                // The flipped triangles lose their partners; refresh the pair graph around.
                for x in [t, u] {
                    let (ix, mx) = (a.idx[x as usize] as usize, a.mate[a.idx[x as usize] as usize]);
                    if mx != NONE {
                        a.mate[mx as usize] = NONE;
                        a.mate[ix] = NONE;
                    }
                }
                a.refresh(m, t);
                a.refresh(m, u);
                Blossom::new(&a.g, FLOOR_MIN, &mut a.mate).run();
                let now = a.mate.iter().filter(|x| **x == NONE).count();
                if now < cur {
                    break;
                }
                m.cdt.undo_flip(undo);
                a.mate = saved;
                a.refresh(m, t);
                a.refresh(m, u);
            }
        }
        if a.unmatched().len() >= before {
            return;
        }
    }
}

/// Recombine the triangulation into quads. Returns the quads (counter-clockwise, vertex ids
/// of the mesher) and the number of edge-splitting rounds used.
pub(crate) fn recombine(m: &mut Mesher<'_>, max_rounds: u32) -> Result<(Vec<[u32; 4]>, u32), PaveError> {
    let mut rounds = 0u32;
    loop {
        let mut an = analyze(m);
        m.work += an.tris.len() as u64;
        if !an.unmatched().is_empty() {
            flip_repair(m, &mut an);
        }
        let unmatched = an.unmatched();
        if unmatched.is_empty() {
            let mut quads = Vec::with_capacity(an.tris.len() / 2);
            for i in 0..an.tris.len() {
                let j = an.mate[i] as usize;
                if j > i {
                    let (t, u) = (an.tris[i], an.tris[j]);
                    let s = (0..3).find(|k| m.cdt.tn[t as usize][*k] == u).unwrap_or(0);
                    let c = if t < u {
                        candidate(m, t, s, u)
                    } else {
                        let su = (0..3).find(|k| m.cdt.tn[u as usize][*k] == t).unwrap_or(0);
                        candidate(m, u, su, t)
                    };
                    match c {
                        Some((_, _, q)) => quads.push(q),
                        None => return Err(PaveError::Internal("matched pair is not convex")),
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
            let t = an.tris[i as usize];
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

    /// Graph with at most three neighbours per vertex (the dual graph's degree bound).
    fn csr(n: usize, edges: &[(u32, u32)]) -> Graph {
        let mut g = Graph { adj: vec![[NONE; 3]; n], sine: vec![[1.0; 3]; n], weight: vec![[1.0; 3]; n] };
        for (a, b) in edges {
            for (x, y) in [(*a, *b), (*b, *a)] {
                let slot = g.adj[x as usize].iter().position(|v| *v == NONE).expect("degree <= 3");
                g.adj[x as usize][slot] = y;
            }
        }
        g
    }

    #[test]
    fn blossom_finds_the_perfect_matching_of_an_odd_cycle_with_tails() {
        // Triangle 0-1-2 with pendant vertices 3 (on 0), 4 (on 1), and 5 (on 2): perfect
        // matching is 0-3, 1-4, 2-5, found only after contracting the odd cycle when started
        // from the bad matching 0-1.
        let edges = [(0, 1), (1, 2), (2, 0), (0, 3), (1, 4), (2, 5)];
        let g = csr(6, &edges);
        let mut mate = vec![NONE; 6];
        mate[0] = 1;
        mate[1] = 0;
        Blossom::new(&g, 0.0, &mut mate).run();
        assert!(mate.iter().all(|m| *m != NONE), "{mate:?}");
        for (i, m) in mate.iter().enumerate() {
            assert_eq!(mate[*m as usize] as usize, i);
        }
        // Petersen-like blossom case: a 5-cycle plus one pendant has no perfect matching.
        let edges = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 0), (0, 5)];
        let g = csr(6, &edges);
        let mut mate = vec![NONE; 6];
        Blossom::new(&g, 0.0, &mut mate).run();
        assert_eq!(mate.iter().filter(|m| **m == NONE).count(), 0);
    }

    /// Maximum matching size by exhaustive search.
    fn brute(n: usize, edges: &[(u32, u32)]) -> usize {
        fn go(i: usize, edges: &[(u32, u32)], used: &mut [bool], size: usize) -> usize {
            if i == edges.len() {
                return size;
            }
            let mut best = go(i + 1, edges, used, size);
            let (a, b) = (edges[i].0 as usize, edges[i].1 as usize);
            if !used[a] && !used[b] {
                used[a] = true;
                used[b] = true;
                best = best.max(go(i + 1, edges, used, size + 1));
                used[a] = false;
                used[b] = false;
            }
            best
        }
        go(0, edges, &mut vec![false; n], 0)
    }

    #[test]
    fn blossom_matches_brute_force_on_random_graphs_from_any_start() {
        let mut seed = 0x9e3779b97f4a7c15u64;
        let mut rnd = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };
        for case in 0..300 {
            let n = 4 + (rnd() % 9) as usize;
            let mut edges = Vec::new();
            for a in 0..n as u32 {
                for b in a + 1..n as u32 {
                    let deg = |v: u32| edges.iter().filter(|(x, y)| *x == v || *y == v).count();
                    if rnd() % 100 < 40 && deg(a) < 3 && deg(b) < 3 {
                        edges.push((a, b));
                    }
                }
            }
            let g = csr(n, &edges);
            // Start from a greedy (maximal, not maximum) matching.
            let mut mate = vec![NONE; n];
            for (x, y) in &edges {
                if mate[*x as usize] == NONE && mate[*y as usize] == NONE && rnd() % 2 == 0 {
                    mate[*x as usize] = *y;
                    mate[*y as usize] = *x;
                }
            }
            Blossom::new(&g, 0.0, &mut mate).run();
            let got = mate.iter().filter(|m| **m != NONE).count() / 2;
            for (i, m) in mate.iter().enumerate() {
                if *m != NONE {
                    assert_eq!(mate[*m as usize] as usize, i);
                    assert!(edges.contains(&(i.min(*m as usize) as u32, i.max(*m as usize) as u32)));
                }
            }
            assert_eq!(got, brute(n, &edges), "case {case}: {edges:?}");
        }
    }

    #[test]
    fn blossom_reports_the_deficiency_of_a_star() {
        let edges = [(0, 1), (0, 2), (0, 3)];
        let g = csr(4, &edges);
        let mut mate = vec![NONE; 4];
        Blossom::new(&g, 0.0, &mut mate).run();
        assert_eq!(mate.iter().filter(|m| **m != NONE).count(), 2);
    }
}
