//! Exact cavity tetrahedralization by backtracking gift wrapping.

use crate::guard::{Guard, S4};
use crate::delaunay::Dt;
use crate::geom::ori;
use crate::tds::{canon_tri, same_cycle};

impl Dt {
    /// Tetrahedralize the polyhedron bounded by `faces` (outward orientation) with exact
    /// closed-contact validity: depth-first gift wrapping with an empty-sphere preference and
    /// backtracking (complete for small cavities, bounded by a node budget). With
    /// `seg = Some((u,w))` no new tet may touch the open segment except by having `u-w` as an
    /// edge. `extra` are interior points (not on the front) the tets may use; a point that is
    /// not used must still lie outside every tet, which keeps it from being swallowed.
    pub(crate) fn fill(&self, faces: &[[u32; 3]], seg: Option<(u32, u32)>, extra: &[u32]) -> Option<Vec<[u32; 4]>> {
        self.fill_budgeted(faces, seg, extra, 2000 + 40 * faces.len())
    }

    /// [`Dt::fill`] with an explicit budget (number of candidate validity evaluations).
    pub(crate) fn fill_budgeted(&self, faces: &[[u32; 3]], seg: Option<(u32, u32)>, extra: &[u32], budget: usize) -> Option<Vec<[u32; 4]>> {
        let mut verts: Vec<u32> = faces.iter().flatten().copied().chain(extra.iter().copied()).collect();
        verts.sort_unstable();
        verts.dedup();
        let mut front: std::collections::VecDeque<[u32; 3]> = faces.iter().map(|&f| canon_tri(f)).collect();
        let mut out: Vec<[u32; 4]> = Vec::new();
        let mut budget = budget;
        if self.wrap_dfs(&mut front, &mut out, &verts, seg, &mut budget) { Some(out) } else { None }
    }

    fn wrap_dfs(
        &self,
        front: &mut std::collections::VecDeque<[u32; 3]>,
        out: &mut Vec<[u32; 4]>,
        verts: &[u32],
        seg: Option<(u32, u32)>,
        budget: &mut usize,
    ) -> bool {
        let Some(f) = front.pop_front() else { return true };
        if *budget == 0 {
            front.push_front(f);
            return false;
        }
        let s = self.sos();
        let [x, y, z] = f;
        let (px, py, pz) = (s.p(x), s.p(y), s.p(z));
        let mut cands: Vec<u32> = Vec::new();
        for &w in verts {
            if w == x || w == y || w == z || ori(px, py, pz, s.p(w)) >= 0 {
                continue;
            }
            *budget = budget.saturating_sub(1);
            if self.wrap_valid([w, x, y, z], front, verts, seg) {
                cands.push(w);
            }
        }
        // Order candidates by the empty-sphere preference (best first): a candidate whose
        // sphere contains another candidate loses to it.
        let mut ordered: Vec<u32> = Vec::with_capacity(cands.len());
        while !cands.is_empty() {
            let mut bi = 0;
            for i in 1..cands.len() {
                if s.in_sphere_s(cands[bi], x, y, z, cands[i]) {
                    bi = i;
                }
            }
            ordered.push(cands.remove(bi));
        }
        for w in ordered {
            let saved = front.clone();
            out.push([w, x, y, z]);
            for h in [[w, z, y], [w, x, z], [w, y, x]] {
                let hc = canon_tri(h);
                if let Some(pos) = front.iter().position(|g| *g == hc) {
                    front.remove(pos);
                } else {
                    front.push_back(canon_tri([h[0], h[2], h[1]]));
                }
            }
            if self.wrap_dfs(front, out, verts, seg, budget) {
                return true;
            }
            out.pop();
            *front = saved;
            if *budget == 0 {
                break;
            }
        }
        front.push_front(f);
        false
    }

    fn wrap_valid(&self, tet: [u32; 4], front: &std::collections::VecDeque<[u32; 3]>, verts: &[u32], seg: Option<(u32, u32)>) -> bool {
        let s = self.sos();
        let [w, x, y, z] = tet;
        let tp = [s.p(w), s.p(x), s.p(y), s.p(z)];
        // Other cavity vertices may not touch the closed tet.
        for &c in verts {
            if !tet.contains(&c) && crate::exact::pt_in_closed_tet(tp, s.p(c)) {
                return false;
            }
        }
        if let Some((u, v)) = seg
            && crate::exact::seg_touches_tet(tet, tp, u, v, s.p(u), s.p(v)) {
                return false;
            }
        // Outward faces of the new tet other than the wrapped one, with the opposite vertex.
        let outward = [([w, z, y], x), ([w, x, z], y), ([w, y, x], z)];
        let hbox = outward.map(|(h, _)| bbox(&[s.p(h[0]), s.p(h[1]), s.p(h[2])]));
        for g in front {
            let gbox = bbox(&[s.p(g[0]), s.p(g[1]), s.p(g[2])]);
            for (k, (h, opp)) in outward.iter().enumerate() {
                if (0..3).any(|a| gbox.1[a] < hbox[k].0[a] || hbox[k].1[a] < gbox.0[a]) {
                    continue; // disjoint boxes: no contact
                }
                if *g == canon_tri(*h) {
                    continue; // coincides with a front face: the tet closes it
                }
                // The reverse of a front face is a tet on its far side: invalid, unless the front
                // holds both sides of that face (a slit wall kept inside the cavity).
                if same_cycle(*g, [h[0], h[2], h[1]]) && !front.contains(&canon_tri(*h)) {
                    return false;
                }
                let gv = [(g[0], s.p(g[0])), (g[1], s.p(g[1])), (g[2], s.p(g[2]))];
                let hv = [(h[0], s.p(h[0])), (h[1], s.p(h[1])), (h[2], s.p(h[2]))];
                if crate::exact::tris_conflict(gv, hv, s.p(*opp)) {
                    return false;
                }
            }
        }
        true
    }
}

/// Axis-aligned bounding box `(min, max)` of points.
fn bbox(p: &[[f64; 3]]) -> ([f64; 3], [f64; 3]) {
    let mut lo = p[0];
    let mut hi = p[0];
    for q in p {
        for a in 0..3 {
            lo[a] = lo[a].min(q[a]);
            hi[a] = hi[a].max(q[a]);
        }
    }
    (lo, hi)
}

impl Dt {
    /// Fill a cavity with its own vertices; if that is impossible, retry with one interior
    /// Steiner point the tets may use (centroid, then points just inside the faces). Returns
    /// the tets and the number of Steiner points used (0 or 1); points not used are discarded.
    pub(crate) fn fill_with_steiner(&mut self, faces: &[[u32; 3]], seg: Option<(u32, u32)>, budget: usize, guard: &Guard<'_>) -> Option<(Vec<[u32; 4]>, usize)> {
        use crate::geom::{add, cross, dist, dot, sanitize, scale, sub};
        if let Some(t) = self.fill_budgeted(faces, seg, &[], budget) {
            return Some((t, 0));
        }
        let mut vs: Vec<u32> = faces.iter().flatten().copied().collect();
        vs.sort_unstable();
        vs.dedup();
        let mut c = [0.0; 3];
        for &v in &vs {
            c = add(c, self.pts[v as usize]);
        }
        let c = scale(c, 1.0 / vs.len() as f64);
        let mean_edge = {
            let mut m = 0.0;
            for f in faces {
                m += dist(self.pts[f[0] as usize], self.pts[f[1] as usize]);
            }
            m / faces.len() as f64
        };
        let mut cands: Vec<[f64; 3]> = vec![c];
        for (i, f) in faces.iter().take(8).enumerate() {
            let (a, b, d) = (self.pts[f[0] as usize], self.pts[f[1] as usize], self.pts[f[2] as usize]);
            let n = cross(sub(b, a), sub(d, a));
            let nl = dot(n, n).sqrt();
            if nl > 0.0 {
                let fc = scale(add(add(a, b), d), 1.0 / 3.0);
                // The right-hand normal of an outward face points out of the cavity.
                cands.push(sub(fc, scale(n, 0.3 * mean_edge / nl)));
                if i < 4 {
                    cands.push(sub(fc, scale(n, 0.6 * mean_edge / nl)));
                    cands.push(add(scale(c, 0.5), scale(fc, 0.5)));
                }
            }
        }
        // Ruling S4: candidates are tried best-score first (compliant ones, then by margin), so a
        // point too close to an input triangle inside the solid is used only if nothing better
        // fills the cavity; the order among equal scores is kept.
        let mut scored: Vec<(f64, usize, [f64; 3])> = cands.into_iter().enumerate().filter_map(|(i, p)| sanitize(p).map(|p| (guard.score(p), i, p))).collect();
        // Bucket by compliance (original order inside the compliant bucket: the centroid first),
        // then by margin.
        scored.sort_by(|a, b| (b.0 >= S4).cmp(&(a.0 >= S4)).then_with(|| if a.0 >= S4 { a.1.cmp(&b.1) } else { b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)) }));
        for (_, _, cand) in scored.into_iter().take(12) {
            let Some(p) = sanitize(cand) else { continue };
            let base = self.pts.len();
            let id = self.push_point(p);
            if let Some(t) = self.fill_budgeted(faces, seg, &[id], budget)
                && t.iter().any(|tet| tet.contains(&id))
            {
                return Some((t, 1));
            }
            self.truncate_points(base);
        }
        None
    }
}
