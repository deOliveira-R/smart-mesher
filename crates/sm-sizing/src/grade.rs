//! Graded size from point samples: `h(x) = min(h_max, min_i (h_i + g |x - x_i|))`.
//!
//! This is the exact grade limit of a sample set (every pair of samples then satisfies
//! `h(p) <= h(q) + g |p - q|`, and so does every pair of points in space). It is evaluated by a
//! branch-and-bound over a kd-tree that stores, per node, the bounding box and the least sample
//! size below it, so a query is about `O(log n)` for a smooth field and does not depend on any
//! visiting order: the result is the exact minimum, bit-identical for any thread count.

use sm_core::Point3;
use sm_scheme_api::SizeField;

const NONE: u32 = u32::MAX;

struct Node {
    p: [f64; 3],
    h: f64,
    lo: [f64; 3],
    hi: [f64; 3],
    h_min: f64,
    left: u32,
    right: u32,
}

/// A graded size field over point samples `(x_i, h_i)`.
pub struct Graded {
    nodes: Vec<Node>,
    root: u32,
    grade: f64,
    h_max: f64,
}

fn coords(p: Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}

impl Graded {
    /// Build over `samples` (any order: they are sorted into the tree with the sample's value and
    /// coordinates as a total order, so the tree and every query are deterministic). `grade >= 0`.
    pub fn new(samples: &[(Point3, f64)], grade: f64, h_max: f64) -> Self {
        let mut items: Vec<([f64; 3], f64)> = samples.iter().map(|&(p, h)| (coords(p), h)).collect();
        let mut nodes = Vec::with_capacity(items.len());
        let root = build(&mut items, 0, &mut nodes);
        Self { nodes, root, grade, h_max }
    }

    /// Number of samples.
    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    /// True without samples (the field is `h_max` everywhere).
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    fn query(&self, node: u32, x: [f64; 3], best: &mut f64) {
        if node == NONE {
            return;
        }
        let n = &self.nodes[node as usize];
        // Lower bound of everything below: the least size there plus the grade times the
        // distance to the subtree's box.
        let d2 = n.box_d2(x);
        if n.h_min + self.grade * d2.sqrt() >= *best {
            return;
        }
        let d = ((n.p[0] - x[0]).powi(2) + (n.p[1] - x[1]).powi(2) + (n.p[2] - x[2]).powi(2)).sqrt();
        *best = best.min(n.h + self.grade * d);
        // Nearer child first for faster pruning (the minimum is order-independent).
        let (a, b) = (n.left, n.right);
        let da = self.node_d2(a, x);
        let db = self.node_d2(b, x);
        let (first, second) = if da <= db { (a, b) } else { (b, a) };
        self.query(first, x, best);
        self.query(second, x, best);
    }

    fn node_d2(&self, node: u32, x: [f64; 3]) -> f64 {
        if node == NONE { f64::INFINITY } else { self.nodes[node as usize].box_d2(x) }
    }
}

impl Node {
    /// Squared distance from `x` to the subtree's bounding box.
    fn box_d2(&self, x: [f64; 3]) -> f64 {
        (0..3)
            .map(|k| (self.lo[k] - x[k]).max(x[k] - self.hi[k]).max(0.0))
            .map(|d| d * d)
            .sum()
    }
}

fn build(items: &mut [([f64; 3], f64)], depth: usize, nodes: &mut Vec<Node>) -> u32 {
    if items.is_empty() {
        return NONE;
    }
    let axis = depth % 3;
    items.sort_by(|a, b| {
        a.0[axis]
            .total_cmp(&b.0[axis])
            .then(a.0[(axis + 1) % 3].total_cmp(&b.0[(axis + 1) % 3]))
            .then(a.0[(axis + 2) % 3].total_cmp(&b.0[(axis + 2) % 3]))
            .then(a.1.total_cmp(&b.1))
    });
    let mid = items.len() / 2;
    let (l, rest) = items.split_at_mut(mid);
    let (me, r) = rest.split_first_mut().expect("non-empty");
    let (p, h) = *me;
    let id = nodes.len() as u32;
    nodes.push(Node { p, h, lo: p, hi: p, h_min: h, left: NONE, right: NONE });
    let left = build(l, depth + 1, nodes);
    let right = build(r, depth + 1, nodes);
    let mut lo = p;
    let mut hi = p;
    let mut h_min = h;
    for c in [left, right] {
        if c != NONE {
            let cn = &nodes[c as usize];
            for k in 0..3 {
                lo[k] = lo[k].min(cn.lo[k]);
                hi[k] = hi[k].max(cn.hi[k]);
            }
            h_min = h_min.min(cn.h_min);
        }
    }
    let n = &mut nodes[id as usize];
    n.left = left;
    n.right = right;
    n.lo = lo;
    n.hi = hi;
    n.h_min = h_min;
    id
}

impl SizeField for Graded {
    fn size(&self, p: Point3) -> f64 {
        let mut best = self.h_max;
        self.query(self.root, coords(p), &mut best);
        best
    }
}
