//! Ruling S4 for interior Steiner points that do not come from refinement (recovery and
//! smoothing): inside the solid they stay at least `0.3 h` from every input triangle, where `h`
//! is the mean edge length of that triangle.
//!
//! A uniform grid over the triangles (each listed in every cell its bounding box, grown by
//! `0.3 h`, touches) answers "how close is `p` to a triangle, relative to its size" in a few
//! distance tests. The inside test (side of the nearest triangle) is only used for points that are
//! closer than the bound.

use crate::geom::{V3, closest_point_tri, cross, dist, dist2_point_tri, dot, sub};

/// The S4 bound as a fraction of the local triangle size.
pub(crate) const S4: f64 = 0.3;

/// Spatial index of the input triangles.
pub(crate) struct Guard<'a> {
    pts: &'a [V3],
    tris: &'a [[u32; 3]],
    /// Mean edge length per triangle.
    h: Vec<f64>,
    /// Grow factor of the cell lists: a triangle is listed in every cell within `grow * h`.
    grow: f64,
    lo: V3,
    inv: V3,
    dim: [usize; 3],
    start: Vec<u32>,
    items: Vec<u32>,
}

impl<'a> Guard<'a> {
    /// Index `tris` over `pts`.
    pub fn new(pts: &'a [V3], tris: &'a [[u32; 3]]) -> Self {
        Self::with_grow(pts, tris, S4)
    }

    /// Index `tris` over `pts`, listing each triangle in the cells within `grow * h_T` of it
    /// (`grow >= S4`; the spacing meter uses a wider reach than the S4 guard).
    pub fn with_grow(pts: &'a [V3], tris: &'a [[u32; 3]], grow: f64) -> Self {
        let h: Vec<f64> = tris
            .iter()
            .map(|t| {
                let [a, b, c] = t.map(|i| pts[i as usize]);
                (dist(a, b) + dist(b, c) + dist(c, a)) / 3.0
            })
            .collect();
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for t in tris {
            for &i in t {
                for k in 0..3 {
                    lo[k] = lo[k].min(pts[i as usize][k]);
                    hi[k] = hi[k].max(pts[i as usize][k]);
                }
            }
        }
        if tris.is_empty() {
            lo = [0.0; 3];
            hi = [1.0; 3];
        }
        let pad = grow * h.iter().copied().fold(0.0, f64::max);
        for k in 0..3 {
            lo[k] -= pad;
            hi[k] += pad;
        }
        let mut hs = h.clone();
        hs.sort_unstable_by(f64::total_cmp);
        let med = hs.get(hs.len() / 2).copied().unwrap_or(1.0).max(f64::MIN_POSITIVE);
        let ext = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
        let span = ext[0].max(ext[1]).max(ext[2]).max(f64::MIN_POSITIVE);
        let cell = med.max(span / 48.0);
        let dim = ext.map(|e| ((e / cell).ceil() as usize).clamp(1, 48));
        let inv = [dim[0] as f64 / ext[0].max(f64::MIN_POSITIVE), dim[1] as f64 / ext[1].max(f64::MIN_POSITIVE), dim[2] as f64 / ext[2].max(f64::MIN_POSITIVE)];
        let mut g = Self { pts, tris, h, grow, lo, inv, dim, start: Vec::new(), items: Vec::new() };
        let ncell = dim[0] * dim[1] * dim[2];
        let mut count = vec![0u32; ncell + 1];
        for pass in 0..2 {
            let mut fill = count.clone();
            for (ti, t) in tris.iter().enumerate() {
                let (r0, r1) = g.range(ti, t);
                for z in r0[2]..=r1[2] {
                    for y in r0[1]..=r1[1] {
                        for x in r0[0]..=r1[0] {
                            let c = (z * dim[1] + y) * dim[0] + x;
                            if pass == 0 {
                                count[c + 1] += 1;
                            } else {
                                g.items[fill[c] as usize] = ti as u32;
                                fill[c] += 1;
                            }
                        }
                    }
                }
            }
            if pass == 0 {
                for i in 0..ncell {
                    count[i + 1] += count[i];
                }
                g.items = vec![0; count[ncell] as usize];
                g.start = count.clone();
            }
        }
        g
    }

    fn cell_of(&self, p: V3) -> [usize; 3] {
        let mut c = [0usize; 3];
        for k in 0..3 {
            let f = ((p[k] - self.lo[k]) * self.inv[k]).floor();
            c[k] = if f <= 0.0 { 0 } else { (f as usize).min(self.dim[k] - 1) };
        }
        c
    }

    fn range(&self, ti: usize, t: &[u32; 3]) -> ([usize; 3], [usize; 3]) {
        let grow = self.grow * self.h[ti];
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for &i in t {
            for k in 0..3 {
                lo[k] = lo[k].min(self.pts[i as usize][k] - grow);
                hi[k] = hi[k].max(self.pts[i as usize][k] + grow);
            }
        }
        (self.cell_of(lo), self.cell_of(hi))
    }

    /// The triangles listed in the cell of `p` as `(distance, index)`, nearest first (ties by
    /// index), at most `k`. Exact for every triangle within `grow * h_T` of `p`.
    pub fn near(&self, p: V3, k: usize) -> Vec<(f64, u32)> {
        if !self.in_grid(p) {
            return Vec::new();
        }
        let c = self.cell_of(p);
        let ci = (c[2] * self.dim[1] + c[1]) * self.dim[0] + c[0];
        let mut v: Vec<(f64, u32)> = self.items[self.start[ci] as usize..self.start[ci + 1] as usize]
            .iter()
            .map(|&ti| {
                let [a, b, d] = self.tris[ti as usize].map(|i| self.pts[i as usize]);
                (dist2_point_tri(p, a, b, d).sqrt(), ti)
            })
            .collect();
        v.sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
        v.truncate(k);
        v
    }

    /// The nearest triangle listed in the cell of `p`: `(distance, index)`; no allocation.
    pub fn nearest_listed(&self, p: V3) -> Option<(f64, u32)> {
        if !self.in_grid(p) {
            return None;
        }
        let c = self.cell_of(p);
        let ci = (c[2] * self.dim[1] + c[1]) * self.dim[0] + c[0];
        let mut best: Option<(f64, u32)> = None;
        for &ti in &self.items[self.start[ci] as usize..self.start[ci + 1] as usize] {
            let [a, b, d] = self.tris[ti as usize].map(|i| self.pts[i as usize]);
            let r = dist2_point_tri(p, a, b, d);
            if best.is_none_or(|x| r < x.0) {
                best = Some((r, ti));
            }
        }
        best.map(|(r, ti)| (r.sqrt(), ti))
    }

    /// Closest point on triangle `ti` to `p`.
    pub fn foot(&self, p: V3, ti: u32) -> V3 {
        let [a, b, d] = self.tris[ti as usize].map(|i| self.pts[i as usize]);
        closest_point_tri(p, a, b, d)
    }

    /// Mean edge length of triangle `ti`.
    pub fn tri_h(&self, ti: u32) -> f64 {
        self.h[ti as usize]
    }

    fn in_grid(&self, p: V3) -> bool {
        (0..3).all(|k| p[k] >= self.lo[k] && p[k] <= self.lo[k] + self.dim[k] as f64 / self.inv[k])
    }

    /// Smallest `dist(p, T) / h_T` over the triangles `T`, capped at [`S4`].
    pub fn margin(&self, p: V3) -> f64 {
        let inside_grid = (0..3).all(|k| p[k] >= self.lo[k] && p[k] <= self.lo[k] + self.dim[k] as f64 / self.inv[k]);
        if !inside_grid {
            return S4;
        }
        let c = self.cell_of(p);
        let ci = (c[2] * self.dim[1] + c[1]) * self.dim[0] + c[0];
        let mut m = S4;
        for &ti in &self.items[self.start[ci] as usize..self.start[ci + 1] as usize] {
            let t = self.tris[ti as usize];
            let [a, b, d] = t.map(|i| self.pts[i as usize]);
            let r = dist2_point_tri(p, a, b, d).sqrt() / self.h[ti as usize];
            m = m.min(r);
        }
        m
    }

    /// Is `p`, which is within the S4 distance of some triangle, on the solid side of its nearest
    /// input triangle (the triangles wind outward)? Local and O(cell), unlike a winding number;
    /// exact on the face interiors, a heuristic only in a thin band at edges and corners.
    fn inside_near(&self, p: V3) -> bool {
        let c = self.cell_of(p);
        let ci = (c[2] * self.dim[1] + c[1]) * self.dim[0] + c[0];
        let mut best: Option<(f64, u32)> = None;
        for &ti in &self.items[self.start[ci] as usize..self.start[ci + 1] as usize] {
            let [a, b, d] = self.tris[ti as usize].map(|i| self.pts[i as usize]);
            let r = dist2_point_tri(p, a, b, d);
            if best.is_none_or(|x| r < x.0) {
                best = Some((r, ti));
            }
        }
        let Some((_, ti)) = best else { return true };
        let [a, b, d] = self.tris[ti as usize].map(|i| self.pts[i as usize]);
        dot(sub(p, a), cross(sub(b, a), sub(d, a))) <= 0.0
    }

    /// The input triangle with the smallest `dist(p, T) / h_T` among those within the S4 bound
    /// of `p`: its vertices and `h_T`; `None` if `p` complies.
    pub fn nearest(&self, p: V3) -> Option<([V3; 3], f64)> {
        let inside_grid = (0..3).all(|k| p[k] >= self.lo[k] && p[k] <= self.lo[k] + self.dim[k] as f64 / self.inv[k]);
        if !inside_grid {
            return None;
        }
        let c = self.cell_of(p);
        let ci = (c[2] * self.dim[1] + c[1]) * self.dim[0] + c[0];
        let mut best: Option<(f64, u32)> = None;
        for &ti in &self.items[self.start[ci] as usize..self.start[ci + 1] as usize] {
            let [a, b, d] = self.tris[ti as usize].map(|i| self.pts[i as usize]);
            let r = dist2_point_tri(p, a, b, d).sqrt() / self.h[ti as usize];
            if r < S4 && best.is_none_or(|x| r < x.0) {
                best = Some((r, ti));
            }
        }
        let (_, ti) = best?;
        Some((self.tris[ti as usize].map(|i| self.pts[i as usize]), self.h[ti as usize]))
    }

    /// S4 score of a candidate interior point: [`S4`] if it complies (or lies outside the solid,
    /// where the point is dropped with the exterior), else its margin.
    pub fn score(&self, p: V3) -> f64 {
        let m = self.margin(p);
        if m >= S4 || !self.inside_near(p) { S4 } else { m }
    }

}
