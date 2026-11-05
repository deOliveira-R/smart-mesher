//! A static bounding-volume hierarchy over boxes, for candidate-pair search and indexed
//! classification.
//!
//! The build is deterministic: items are split at the median of their centroids along the
//! longest axis of the centroid box, ties broken by item index. Queries return item indices;
//! callers sort them when they need a fixed order.

use sm_core::Aabb;

const LEAF_SIZE: usize = 4;

/// Work counters of a query batch. Counts depend only on the inputs, never on threads or time.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CandidateStats {
    /// Leaf-level face-box tests (one per item box compared against a query).
    pub candidate_tests: u64,
    /// Node box visits (internal and leaf nodes; recorded only, not the bounded counter).
    pub node_visits: u64,
}

impl CandidateStats {
    pub(crate) fn add(&mut self, o: &CandidateStats) {
        self.candidate_tests += o.candidate_tests;
        self.node_visits += o.node_visits;
    }
}

#[derive(Clone, Copy, Debug)]
struct Node {
    lo: [f64; 3],
    hi: [f64; 3],
    /// Leaf: first item slot. Inner: index of the right child (left child is `self + 1`).
    a: u32,
    /// Leaf: item count (> 0). Inner: 0.
    n: u32,
}

/// Static BVH over a list of boxes (empty boxes are skipped).
#[derive(Clone, Debug, Default)]
pub(crate) struct Bvh {
    nodes: Vec<Node>,
    /// Item indices (into the build list), grouped by leaf.
    items: Vec<u32>,
}

fn arr(p: sm_core::Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}

impl Bvh {
    /// Build over `boxes`; empty boxes never match anything.
    pub(crate) fn build(boxes: &[Aabb]) -> Bvh {
        let mut items: Vec<u32> = (0..boxes.len() as u32).filter(|&i| !boxes[i as usize].is_empty()).collect();
        let mut bvh = Bvh { nodes: Vec::new(), items: Vec::new() };
        if items.is_empty() {
            return bvh;
        }
        bvh.nodes.reserve(2 * items.len() / LEAF_SIZE + 1);
        let n = items.len();
        bvh.rec(boxes, &mut items, 0, n);
        bvh.items = items;
        bvh
    }

    fn rec(&mut self, boxes: &[Aabb], items: &mut [u32], start: usize, end: usize) -> usize {
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        let (mut clo, mut chi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for &i in &items[start..end] {
            let b = &boxes[i as usize];
            let (bl, bh) = (arr(b.lo), arr(b.hi));
            for k in 0..3 {
                lo[k] = lo[k].min(bl[k]);
                hi[k] = hi[k].max(bh[k]);
                let c = 0.5 * (bl[k] + bh[k]);
                clo[k] = clo[k].min(c);
                chi[k] = chi[k].max(c);
            }
        }
        let me = self.nodes.len();
        self.nodes.push(Node { lo, hi, a: start as u32, n: (end - start) as u32 });
        if end - start <= LEAF_SIZE {
            return me;
        }
        let axis = (0..3).max_by(|&x, &y| (chi[x] - clo[x]).total_cmp(&(chi[y] - clo[y])).then(y.cmp(&x))).unwrap_or(0);
        let cen = |i: u32| -> f64 {
            let b = &boxes[i as usize];
            0.5 * (arr(b.lo)[axis] + arr(b.hi)[axis])
        };
        items[start..end].sort_by(|&p, &q| cen(p).total_cmp(&cen(q)).then(p.cmp(&q)));
        let mid = start + (end - start) / 2;
        self.rec(boxes, items, start, mid);
        let right = self.rec(boxes, items, mid, end);
        self.nodes[me].a = right as u32;
        self.nodes[me].n = 0;
        me
    }

    /// Push the items whose box passes `leaf`, descending only into nodes whose box passes `node`.
    fn visit(&self, boxes: &[Aabb], node: &dyn Fn(&[f64; 3], &[f64; 3]) -> bool, leaf: &dyn Fn(&Aabb) -> bool, out: &mut Vec<u32>, stats: &mut CandidateStats) {
        if self.nodes.is_empty() {
            return;
        }
        let mut stack: Vec<u32> = vec![0];
        while let Some(ni) = stack.pop() {
            let nd = &self.nodes[ni as usize];
            stats.node_visits += 1;
            if !node(&nd.lo, &nd.hi) {
                continue;
            }
            if nd.n > 0 {
                for &it in &self.items[nd.a as usize..(nd.a + nd.n) as usize] {
                    stats.candidate_tests += 1;
                    if leaf(&boxes[it as usize]) {
                        out.push(it);
                    }
                }
            } else {
                stack.push(nd.a);
                stack.push(ni + 1);
            }
        }
    }

    /// Items whose box overlaps `q` (closed boxes).
    pub(crate) fn overlapping(&self, boxes: &[Aabb], q: &Aabb, out: &mut Vec<u32>, stats: &mut CandidateStats) {
        let (ql, qh) = (arr(q.lo), arr(q.hi));
        self.visit(boxes, &|lo, hi| (0..3).all(|k| lo[k] <= qh[k] && ql[k] <= hi[k]), &|b| crate::geo::aabb_overlap(b, q), out, stats);
    }

    /// Items whose box meets the segment `origin + t dir`, `t` in `[0, reach]`.
    pub(crate) fn along_ray(&self, boxes: &[Aabb], origin: [f64; 3], dir: [f64; 3], reach: f64, out: &mut Vec<u32>, stats: &mut CandidateStats) {
        let hit = |lo: &[f64; 3], hi: &[f64; 3]| -> bool {
            let (mut t0, mut t1) = (0.0f64, reach);
            for k in 0..3 {
                if dir[k].abs() < 1e-300 {
                    if origin[k] < lo[k] || origin[k] > hi[k] {
                        return false;
                    }
                } else {
                    let (a, b) = ((lo[k] - origin[k]) / dir[k], (hi[k] - origin[k]) / dir[k]);
                    t0 = t0.max(a.min(b));
                    t1 = t1.min(a.max(b));
                    if t0 > t1 {
                        return false;
                    }
                }
            }
            true
        };
        self.visit(boxes, &hit, &|b| hit(&arr(b.lo), &arr(b.hi)), out, stats);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_core::Point3;

    fn lcg(s: &mut u64) -> f64 {
        *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (*s >> 11) as f64 / (1u64 << 53) as f64
    }

    fn boxes(n: usize, seed: u64) -> Vec<Aabb> {
        let mut s = seed;
        (0..n)
            .map(|_| {
                let (x, y, z) = (lcg(&mut s) * 100.0, lcg(&mut s) * 100.0, lcg(&mut s) * 100.0);
                let (a, b, c) = (lcg(&mut s) * 6.0, lcg(&mut s) * 6.0, lcg(&mut s) * 6.0);
                Aabb { lo: Point3::new(x, y, z), hi: Point3::new(x + a, y + b, z + c) }
            })
            .collect()
    }

    #[test]
    fn overlap_matches_brute_force() {
        let bx = boxes(500, 7);
        let t = Bvh::build(&bx);
        for q in &boxes(100, 9) {
            let (mut got, mut st) = (Vec::new(), CandidateStats::default());
            t.overlapping(&bx, q, &mut got, &mut st);
            got.sort();
            let want: Vec<u32> = (0..bx.len() as u32).filter(|&i| crate::geo::aabb_overlap(&bx[i as usize], q)).collect();
            assert_eq!(got, want);
        }
    }

    #[test]
    fn ray_reports_every_box_on_the_segment() {
        let bx = boxes(300, 3);
        let t = Bvh::build(&bx);
        let mut s = 11u64;
        for _ in 0..30 {
            let o = [lcg(&mut s) * 100.0, lcg(&mut s) * 100.0, lcg(&mut s) * 100.0];
            let d = [lcg(&mut s) - 0.5, lcg(&mut s) - 0.5, lcg(&mut s) - 0.5];
            let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            let d = [d[0] / n, d[1] / n, d[2] / n];
            let (mut got, mut st) = (Vec::new(), CandidateStats::default());
            t.along_ray(&bx, o, d, 150.0, &mut got, &mut st);
            for k in 0..=1500 {
                let t = 0.1 * k as f64;
                let p = Point3::new(o[0] + d[0] * t, o[1] + d[1] * t, o[2] + d[2] * t);
                for (i, b) in bx.iter().enumerate() {
                    if b.lo.x <= p.x && p.x <= b.hi.x && b.lo.y <= p.y && p.y <= b.hi.y && b.lo.z <= p.z && p.z <= b.hi.z {
                        assert!(got.contains(&(i as u32)));
                    }
                }
            }
        }
    }

    #[test]
    fn build_is_deterministic() {
        let bx = boxes(300, 5);
        let (a, b) = (Bvh::build(&bx), Bvh::build(&bx));
        assert_eq!(a.items, b.items);
    }
}
