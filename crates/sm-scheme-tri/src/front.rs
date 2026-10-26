//! Advancing-front triangulation in the chart (`triadvance`).
//!
//! The front is the set of directed boundary segments with the unmeshed region on their left
//! (embedded segments contribute both directions). The shortest front edge (3-D length, ties by
//! the pair of point ids) is closed with a triangle whose apex is either the *ideal point* (at
//! distance `h` from both ends, on the perpendicular bisector) or an existing node within
//! `1.5 h`, whichever gives the best 3-D minimum angle. Every candidate is checked with exact
//! predicates: the new triangle must be counter-clockwise, no front edge may cross its sides,
//! and no other node may lie in it. The result is a [`Cdt`] (so it can be smoothed and flipped
//! with [`Cdt::optimize`]), but it is **not** Delaunay: there is no angle guarantee, which is why
//! `triadvance` is the fallback of `trimesh`, not its replacement.
//!
//! Boundary segments must be oriented with the region on their left (outer loop
//! counter-clockwise, holes clockwise), which is what [`crate::chart::FaceChart`] produces.
//! Isolated points inside the region are inserted into the finished triangulation.

use crate::cdt::refine::measure;
use crate::cdt::{Cdt, Geometry2, OptimizeOptions, Pslg, SegmentKind};
use crate::error::TriError;
use sm_core::Point3;
use sm_math::Sign;
use sm_math::predicates::{in_exact_range, orient2d};
use std::collections::{BTreeMap, BTreeSet};

/// Options of [`advance`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AdvanceOptions {
    /// Scale of the target size (`1.0` = the geometry's size).
    pub size_factor: f64,
    /// Smooth and flip at the end ([`Cdt::optimize`]).
    pub optimize: bool,
    /// Budget on the number of points.
    pub max_points: usize,
    /// The iteration cap is `iteration_factor * (2 * boundary segments + ideal triangle count)`
    /// (default 20): an iteration closes one front edge, so it also caps the triangle count.
    pub iteration_factor: f64,
}

impl Default for AdvanceOptions {
    fn default() -> Self {
        Self {
            size_factor: 1.0,
            optimize: true,
            max_points: 2_000_000,
            iteration_factor: 20.0,
        }
    }
}

type Cell = (i64, i64);

/// Uniform grid of ids (`BTreeMap` cells, so iteration order is deterministic).
struct Grid {
    cell: f64,
    cells: BTreeMap<Cell, Vec<u32>>,
}

impl Grid {
    fn at(&self, p: [f64; 2]) -> Cell {
        ((p[0] / self.cell).floor() as i64, (p[1] / self.cell).floor() as i64)
    }

    fn range(&self, lo: [f64; 2], hi: [f64; 2]) -> impl Iterator<Item = Cell> {
        let (a, b) = (self.at(lo), self.at(hi));
        (a.0..=b.0).flat_map(move |i| (a.1..=b.1).map(move |j| (i, j)))
    }

    fn insert_point(&mut self, id: u32, p: [f64; 2]) {
        let c = self.at(p);
        self.cells.entry(c).or_default().push(id);
    }

    fn insert_box(&mut self, id: u32, lo: [f64; 2], hi: [f64; 2]) {
        for c in self.range(lo, hi).collect::<Vec<_>>() {
            self.cells.entry(c).or_default().push(id);
        }
    }

    fn remove_box(&mut self, id: u32, lo: [f64; 2], hi: [f64; 2]) {
        for c in self.range(lo, hi).collect::<Vec<_>>() {
            if let Some(v) = self.cells.get_mut(&c) {
                v.retain(|&x| x != id);
            }
        }
    }

    /// Ids in the cells overlapping the box, ascending and deduplicated.
    fn query(&self, lo: [f64; 2], hi: [f64; 2]) -> Vec<u32> {
        let mut out: Vec<u32> = Vec::new();
        for c in self.range(lo, hi) {
            if let Some(v) = self.cells.get(&c) {
                out.extend_from_slice(v);
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }
}

struct Front<'a> {
    g: &'a dyn Geometry2,
    pts: Vec<[f64; 2]>,
    p3: Vec<Point3>,
    nodes: Grid,
    /// Directed front edges `(a, b)` -> edge id.
    edges: BTreeMap<(u32, u32), u32>,
    /// Edge id -> `(a, b)` (removed edges are `None`).
    by_id: Vec<Option<(u32, u32)>>,
    edge_grid: Grid,
    /// Front edges ordered by (3-D length bits, a, b).
    queue: BTreeSet<(u64, u32, u32)>,
    tris: Vec<[u32; 3]>,
}

fn bbox(a: [f64; 2], b: [f64; 2]) -> ([f64; 2], [f64; 2]) {
    ([a[0].min(b[0]), a[1].min(b[1])], [a[0].max(b[0]), a[1].max(b[1])])
}

fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

fn dist3(a: Point3, b: Point3) -> f64 {
    (a - b).norm()
}

impl Front<'_> {
    fn o(&self, a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> Sign {
        orient2d(a, b, c)
    }

    /// Do the closed segments `p-q` and `r-s` conflict: cross, touch away from a shared end
    /// point, or overlap along a line? Points are `(id, coordinates)`; an id may be
    /// `u32::MAX` for a candidate that is not a node yet.
    fn segments_conflict(
        &self,
        p: (u32, [f64; 2]),
        q: (u32, [f64; 2]),
        r: (u32, [f64; 2]),
        s: (u32, [f64; 2]),
    ) -> bool {
        let same = |x: (u32, [f64; 2]), y: (u32, [f64; 2])| x.0 == y.0 && x.0 != u32::MAX;
        let on = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| -> bool {
            (c[0] >= a[0].min(b[0]) && c[0] <= a[0].max(b[0]))
                && (c[1] >= a[1].min(b[1]) && c[1] <= a[1].max(b[1]))
        };
        // The very same segment (the new side closes a front edge): no conflict.
        if (same(p, r) && same(q, s)) || (same(p, s) && same(q, r)) {
            return false;
        }
        // Segments sharing an end point: only a collinear overlap conflicts.
        for (x, ox, y, oy) in [(p, q, r, s), (p, q, s, r), (q, p, r, s), (q, p, s, r)] {
            if same(x, y) {
                let (da, db) = (sub(ox.1, x.1), sub(oy.1, x.1));
                return self.o(x.1, ox.1, oy.1) == Sign::Zero && da[0] * db[0] + da[1] * db[1] > 0.0;
            }
        }
        let (o1, o2) = (self.o(p.1, q.1, r.1), self.o(p.1, q.1, s.1));
        let (o3, o4) = (self.o(r.1, s.1, p.1), self.o(r.1, s.1, q.1));
        let nz = |o: Sign| o != Sign::Zero;
        if nz(o1) && nz(o2) && nz(o3) && nz(o4) && o1 != o2 && o3 != o4 {
            return true;
        }
        (o1 == Sign::Zero && on(p.1, q.1, r.1))
            || (o2 == Sign::Zero && on(p.1, q.1, s.1))
            || (o3 == Sign::Zero && on(r.1, s.1, p.1))
            || (o4 == Sign::Zero && on(r.1, s.1, q.1))
    }

    /// Would the triangle `(a, b, c)` (apex `c` at `cp`, `u32::MAX` for a new node) be a valid
    /// next triangle: counter-clockwise, its sides crossing no front edge, creating no front
    /// edge twice, and containing no other node?
    fn valid(&self, a: u32, b: u32, c: u32, cp: [f64; 2]) -> bool {
        let (pa, pb) = (self.pts[a as usize], self.pts[b as usize]);
        if self.o(pa, pb, cp) != Sign::Positive {
            return false;
        }
        if c != u32::MAX && (self.edges.contains_key(&(c, b)) || self.edges.contains_key(&(a, c))) {
            return false;
        }
        let lo = [pa[0].min(pb[0]).min(cp[0]), pa[1].min(pb[1]).min(cp[1])];
        let hi = [pa[0].max(pb[0]).max(cp[0]), pa[1].max(pb[1]).max(cp[1])];
        let (na, nb, nc) = ((a, pa), (b, pb), (c, cp));
        for e in self.edge_grid.query(lo, hi) {
            let Some((x, y)) = self.by_id[e as usize] else {
                continue;
            };
            let (nx, ny) = ((x, self.pts[x as usize]), (y, self.pts[y as usize]));
            if self.segments_conflict(na, nc, nx, ny) || self.segments_conflict(nb, nc, nx, ny) {
                return false;
            }
        }
        for n in self.nodes.query(lo, hi) {
            if n == a || n == b || n == c {
                continue;
            }
            let q = self.pts[n as usize];
            if self.o(pa, pb, q) != Sign::Negative
                && self.o(pb, cp, q) != Sign::Negative
                && self.o(cp, pa, q) != Sign::Negative
            {
                return false;
            }
        }
        true
    }

    fn add_edge(&mut self, a: u32, b: u32) {
        let id = self.by_id.len() as u32;
        self.by_id.push(Some((a, b)));
        self.edges.insert((a, b), id);
        let (lo, hi) = bbox(self.pts[a as usize], self.pts[b as usize]);
        self.edge_grid.insert_box(id, lo, hi);
        let len = dist3(self.p3[a as usize], self.p3[b as usize]);
        self.queue.insert((len.to_bits(), a, b));
    }

    fn remove_edge(&mut self, a: u32, b: u32) {
        if let Some(id) = self.edges.remove(&(a, b)) {
            self.by_id[id as usize] = None;
            let (lo, hi) = bbox(self.pts[a as usize], self.pts[b as usize]);
            self.edge_grid.remove_box(id, lo, hi);
            let len = dist3(self.p3[a as usize], self.p3[b as usize]);
            self.queue.remove(&(len.to_bits(), a, b));
        }
    }

    /// A new front edge `x -> y`; it cancels the opposite front edge if there is one.
    fn push_edge(&mut self, x: u32, y: u32) {
        if self.edges.contains_key(&(y, x)) {
            self.remove_edge(y, x);
        } else {
            self.add_edge(x, y);
        }
    }

    fn new_node(&mut self, q: [f64; 2]) -> u32 {
        let id = self.pts.len() as u32;
        self.pts.push(q);
        self.p3.push(self.g.lift(q));
        self.nodes.insert_point(id, q);
        id
    }

    /// Distance from `p` to the segment `u-v` (floating point; used for clearance only).
    fn dist_to_segment(p: [f64; 2], u: [f64; 2], v: [f64; 2]) -> f64 {
        let d = sub(v, u);
        let l2 = d[0] * d[0] + d[1] * d[1];
        let t = if l2 > 0.0 {
            (((p[0] - u[0]) * d[0] + (p[1] - u[1]) * d[1]) / l2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let q = [u[0] + t * d[0], u[1] + t * d[1]];
        ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2)).sqrt()
    }

    /// Quality key of the triangle `(a, b, c)`: bigger is better.
    fn quality(&self, a: u32, b: u32, c3: Point3) -> u64 {
        let m = measure(self.g, a3_arr(self.p3[a as usize]), a3_arr(self.p3[b as usize]), a3_arr(c3));
        m.sin_min.to_bits()
    }

    /// Best valid existing node among `candidates` for the front edge `(a, b)`.
    fn best_node(&self, a: u32, b: u32, candidates: &[u32]) -> Option<u32> {
        let mut best: Option<(u64, u32)> = None;
        for &n in candidates {
            if n == a || n == b {
                continue;
            }
            let cp = self.pts[n as usize];
            if !self.valid(a, b, n, cp) {
                continue;
            }
            let key = (self.quality(a, b, self.p3[n as usize]), u32::MAX - n);
            if best.is_none_or(|bst| key > bst) {
                best = Some(key);
            }
        }
        best.map(|(_, inv)| u32::MAX - inv)
    }

    /// Try to close front edge `(a, b)`. Returns false if no valid candidate exists.
    ///
    /// First the *ideal point* (isosceles apex at distance `h` from both ends) is tried, snapped
    /// to an existing node within `0.6 h` if one makes a valid triangle, and shrunk toward the
    /// edge if blocked; if that fails the best valid node within `1.5 h` closes the edge.
    fn close(&mut self, a: u32, b: u32, size_factor: f64) -> bool {
        let (pa, pb) = (self.pts[a as usize], self.pts[b as usize]);
        let (a3, b3) = (self.p3[a as usize], self.p3[b as usize]);
        let mid = [0.5 * (pa[0] + pb[0]), 0.5 * (pa[1] + pb[1])];
        let mid3 = self.g.lift(mid);
        let h = (self.g.size(mid3) * size_factor).max(f64::MIN_POSITIVE);
        let l3 = dist3(a3, b3);
        let e = sub(pb, pa);
        let lq = (e[0] * e[0] + e[1] * e[1]).sqrt();
        if !(crate::util::is_pos(lq) && crate::util::is_pos(l3)) {
            return false;
        }
        let nq = [-e[1] / lq, e[0] / lq];
        // Chart length per 3-D length across the edge, by a small probe.
        let eps = 0.25 * lq;
        let probe = self.g.lift([mid[0] + nq[0] * eps, mid[1] + nq[1] * eps]);
        let d3 = dist3(probe, mid3);
        if !crate::util::is_pos(d3) {
            return false;
        }
        let per3 = eps / d3;
        let height3 = (h * h - 0.25 * l3 * l3).max(0.0).sqrt().max(0.35 * l3);
        let mut choice: Option<(u32, [f64; 2])> = None;
        'ideal: for scale in [1.0, 0.8, 0.6] {
            let cp = [mid[0] + nq[0] * height3 * scale * per3, mid[1] + nq[1] * height3 * scale * per3];
            let snap = 0.6 * h * per3;
            let near = self.nodes.query([cp[0] - snap, cp[1] - snap], [cp[0] + snap, cp[1] + snap]);
            let near: Vec<u32> = near
                .into_iter()
                .filter(|&n| {
                    let d = sub(self.pts[n as usize], cp);
                    d[0] * d[0] + d[1] * d[1] <= snap * snap
                })
                .collect();
            if let Some(n) = self.best_node(a, b, &near) {
                choice = Some((n, self.pts[n as usize]));
                break 'ideal;
            }
            if !self.valid(a, b, u32::MAX, cp) {
                continue;
            }
            // Clearance from the front edges that do not touch this edge.
            let clear = 0.45 * h * per3;
            let mut ok = true;
            for eid in self.edge_grid.query([cp[0] - clear, cp[1] - clear], [cp[0] + clear, cp[1] + clear]) {
                if let Some((x, y)) = self.by_id[eid as usize]
                    && x != a
                    && x != b
                    && y != a
                    && y != b
                    && Self::dist_to_segment(cp, self.pts[x as usize], self.pts[y as usize]) < clear
                {
                    ok = false;
                    break;
                }
            }
            if ok && !near.iter().any(|&n| n != a && n != b) {
                choice = Some((u32::MAX, cp));
                break 'ideal;
            }
        }
        if choice.is_none() {
            // Closure: the best valid node within 1.5 h.
            let r = 1.5 * h * per3;
            let cand: Vec<u32> = self
                .nodes
                .query([mid[0] - r, mid[1] - r], [mid[0] + r, mid[1] + r])
                .into_iter()
                .filter(|&n| {
                    let d = sub(self.pts[n as usize], mid);
                    d[0] * d[0] + d[1] * d[1] <= r * r
                })
                .collect();
            if let Some(n) = self.best_node(a, b, &cand) {
                choice = Some((n, self.pts[n as usize]));
            }
        }
        let Some((id, cp)) = choice else {
            return false;
        };
        let c = if id == u32::MAX { self.new_node(cp) } else { id };
        self.remove_edge(a, b);
        self.tris.push([a, b, c]);
        self.push_edge(c, b);
        self.push_edge(a, c);
        true
    }
}

fn a3_arr(p: Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}

/// Triangulate the region bounded by `p` with an advancing front.
///
/// # Errors
/// [`TriError::Stuck`] when no front edge can be closed or the iteration cap is hit,
/// [`TriError::TooManyPoints`] over the budget, and the input errors of [`Cdt::new`]
/// ([`TriError::OutOfRange`], [`TriError::DuplicatePoint`], [`TriError::BadInput`]).
pub fn advance(p: &Pslg, g: &dyn Geometry2, o: &AdvanceOptions) -> Result<Cdt, TriError> {
    let n = p.points.len();
    for (i, q) in p.points.iter().enumerate() {
        if !(q[0].is_finite() && q[1].is_finite() && in_exact_range(q[0]) && in_exact_range(q[1])) {
            return Err(TriError::OutOfRange { point: i as u32 });
        }
    }
    let mut seen: BTreeMap<[u64; 2], u32> = BTreeMap::new();
    for (i, q) in p.points.iter().enumerate() {
        if let Some(&j) = seen.get(&[q[0].to_bits(), q[1].to_bits()]) {
            return Err(TriError::DuplicatePoint { a: j, b: i as u32 });
        }
        seen.insert([q[0].to_bits(), q[1].to_bits()], i as u32);
    }
    for (i, s) in p.segments.iter().enumerate() {
        if s.a as usize >= n || s.b as usize >= n || s.a == s.b {
            return Err(TriError::BadInput(format!("segment {i} is degenerate or out of range")));
        }
    }
    if p.segments.is_empty() {
        return Err(TriError::BadInput("the front needs boundary segments".into()));
    }
    // Typical chart edge length for the grid cell: the median segment length.
    let mut lens: Vec<f64> = p
        .segments
        .iter()
        .map(|s| {
            let (a, b) = (p.points[s.a as usize], p.points[s.b as usize]);
            ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
        })
        .collect();
    lens.sort_by(f64::total_cmp);
    let cell = lens[lens.len() / 2].max(f64::MIN_POSITIVE);
    let p3: Vec<Point3> = p.points.iter().map(|q| g.lift(*q)).collect();
    let mut f = Front {
        g,
        pts: p.points.clone(),
        p3,
        nodes: Grid { cell, cells: BTreeMap::new() },
        edges: BTreeMap::new(),
        by_id: Vec::new(),
        edge_grid: Grid { cell, cells: BTreeMap::new() },
        queue: BTreeSet::new(),
        tris: Vec::new(),
    };
    let mut on_segment = vec![false; n];
    for s in &p.segments {
        on_segment[s.a as usize] = true;
        on_segment[s.b as usize] = true;
    }
    for (i, q) in p.points.iter().enumerate() {
        if on_segment[i] {
            f.nodes.insert_point(i as u32, *q);
        }
    }
    for s in &p.segments {
        f.push_edge(s.a, s.b);
        if s.kind == SegmentKind::Embedded {
            f.push_edge(s.b, s.a);
        }
    }
    // Iteration cap: 20 * (2 * boundary segments + ideal triangle count).
    let ideal = {
        let area: f64 = {
            // Shoelace area of the boundary loops in the chart, scaled to 3-D by the typical
            // 3-D/chart length ratio of the segments.
            let mut s = 0.0;
            let mut l3 = 0.0;
            let mut lq = 0.0;
            for sg in p.segments.iter().filter(|s| s.kind == SegmentKind::Boundary) {
                let (a, b) = (p.points[sg.a as usize], p.points[sg.b as usize]);
                s += a[0] * b[1] - b[0] * a[1];
                l3 += dist3(f.p3[sg.a as usize], f.p3[sg.b as usize]);
                lq += ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt();
            }
            let ratio = if lq > 0.0 { l3 / lq } else { 1.0 };
            0.5 * s.abs() * ratio * ratio
        };
        let h = {
            let mid = f.pts[p.segments[0].a as usize];
            g.size(g.lift(mid)) * o.size_factor
        };
        area / (0.433_012_701_892_219_3 * h * h)
    };
    let cap = (o.iteration_factor * (2.0 * p.segments.len() as f64 + ideal)).ceil() as usize + 16;
    let mut iterations = 0usize;
    while !f.queue.is_empty() {
        iterations += 1;
        if iterations > cap {
            return Err(TriError::Stuck { front_edges: f.queue.len() });
        }
        if f.pts.len() > o.max_points {
            return Err(TriError::TooManyPoints { limit: o.max_points.min(u32::MAX as usize) as u32 });
        }
        // The shortest edge first; if it cannot be closed try the next ones.
        let candidates: Vec<(u32, u32)> = f.queue.iter().take(32).map(|&(_, a, b)| (a, b)).collect();
        let mut done = false;
        for (a, b) in candidates {
            if f.close(a, b, o.size_factor) {
                done = true;
                break;
            }
        }
        if !done {
            return Err(TriError::Stuck { front_edges: f.queue.len() });
        }
    }
    // Build the triangulation; the pslg points may have grown (ideal points).
    let mut full = p.clone();
    full.points = f.pts.clone();
    let mut cdt = Cdt::from_triangles(&full, &f.tris)?;
    cdt.set_input_count(n);
    for (i, on) in on_segment.iter().enumerate() {
        if !on {
            cdt.insert_input_point(i as u32)?;
        }
    }
    if o.optimize {
        cdt.optimize(g, &OptimizeOptions::default());
    }
    Ok(cdt)
}
