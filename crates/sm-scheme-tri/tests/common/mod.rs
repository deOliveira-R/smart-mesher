//! Shared helpers for the integration tests.
#![allow(dead_code)]

use sm_core::Point3;
use sm_math::Sign;
use sm_math::predicates::{incircle, orient2d};
use sm_math::sum::NeumaierSum;
use sm_scheme_tri::cdt::{Cdt, Geometry2, Pslg, Segment, SegmentKind};
use sm_testkit::loops2d::Region2;
use std::collections::BTreeMap;

/// The plane z = 0, chart = identity.
pub struct Flat(pub f64);

impl Geometry2 for Flat {
    fn lift(&self, q: [f64; 2]) -> Point3 {
        Point3::new(q[0], q[1], 0.0)
    }
    fn size(&self, _p: Point3) -> f64 {
        self.0
    }
    fn is_isometric(&self) -> bool {
        true
    }
}

/// PSLG of a region: all loops as boundary segments.
pub fn pslg_of(r: &Region2) -> Pslg {
    let mut p = Pslg::default();
    for l in r.loops() {
        let base = p.points.len() as u32;
        let n = l.len() as u32;
        for q in l {
            p.points.push([q.0, q.1]);
        }
        for i in 0..n {
            p.segments.push(Segment {
                a: base + i,
                b: base + (i + 1) % n,
                kind: SegmentKind::Boundary,
            });
        }
    }
    p
}

/// xorshift64* for reproducible shuffles.
pub struct Rng(pub u64);

impl Rng {
    pub fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }
    pub fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
}

/// Shuffle the points of a PSLG (segments remapped).
pub fn shuffled(p: &Pslg, seed: u64) -> Pslg {
    let n = p.points.len();
    let mut perm: Vec<usize> = (0..n).collect();
    let mut rng = Rng(seed | 1);
    for i in (1..n).rev() {
        perm.swap(i, rng.below(i + 1));
    }
    // perm[new] = old
    let mut inv = vec![0u32; n];
    for (new, &old) in perm.iter().enumerate() {
        inv[old] = new as u32;
    }
    let points = perm.iter().map(|&o| p.points[o]).collect();
    let mut segments: Vec<Segment> = p
        .segments
        .iter()
        .map(|s| Segment { a: inv[s.a as usize], b: inv[s.b as usize], kind: s.kind })
        .collect();
    for i in (1..segments.len()).rev() {
        segments.swap(i, rng.below(i + 1));
    }
    Pslg { points, segments }
}

/// Triangles as sorted coordinate triples (bits), sorted: comparable across relabellings.
pub fn tri_set(c: &Cdt) -> Vec<[[u64; 2]; 3]> {
    let pts = c.points();
    let mut out: Vec<[[u64; 2]; 3]> = c
        .triangles()
        .iter()
        .map(|t| {
            let mut v = [0, 1, 2].map(|k| {
                let p = pts[t[k] as usize];
                [p[0].to_bits(), p[1].to_bits()]
            });
            v.sort();
            v
        })
        .collect();
    out.sort();
    out
}

/// Every triangle is exactly counter-clockwise.
pub fn all_positive(c: &Cdt) -> bool {
    let pts = c.points();
    c.triangles().iter().all(|t| {
        orient2d(pts[t[0] as usize], pts[t[1] as usize], pts[t[2] as usize]) == Sign::Positive
    })
}

/// Area (compensated) of the triangulation.
pub fn area(c: &Cdt) -> f64 {
    let pts = c.points();
    let mut s = NeumaierSum::new();
    for t in c.triangles() {
        let (a, b, d) = (pts[t[0] as usize], pts[t[1] as usize], pts[t[2] as usize]);
        s.add(0.5 * ((b[0] - a[0]) * (d[1] - a[1]) - (b[1] - a[1]) * (d[0] - a[0])));
    }
    s.value()
}

/// Count unconstrained interior edges whose opposite vertex is strictly inside the circumcircle.
pub fn delaunay_violations(c: &Cdt) -> usize {
    let pts = c.points();
    let tris = c.triangles();
    let nb = c.neighbors();
    let mut bad = 0;
    for (t, tv) in tris.iter().enumerate() {
        for i in 0..3 {
            let n = nb[t][i];
            if n == Cdt::NO_NEIGHBOR {
                continue;
            }
            let (a, b) = (tv[(i + 1) % 3], tv[(i + 2) % 3]);
            if c.is_constrained(a, b) {
                continue;
            }
            let nv = tris[n as usize];
            let opp = nv.iter().copied().find(|&x| x != a && x != b).unwrap();
            if incircle(
                pts[tv[0] as usize],
                pts[tv[1] as usize],
                pts[tv[2] as usize],
                pts[opp as usize],
            ) == Sign::Positive
            {
                bad += 1;
            }
        }
    }
    bad
}

/// Edges (as unordered point-id pairs) that belong to exactly one triangle.
pub fn boundary_edges(c: &Cdt) -> Vec<(u32, u32)> {
    let mut count: BTreeMap<(u32, u32), u32> = BTreeMap::new();
    for t in c.triangles() {
        for i in 0..3 {
            let (a, b) = (t[i], t[(i + 1) % 3]);
            *count.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    count.into_iter().filter(|(_, n)| *n == 1).map(|(e, _)| e).collect()
}
