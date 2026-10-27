//! Exact cavity tetrahedralization by backtracking gift wrapping.

use crate::guard::{Guard, S4};
use crate::delaunay::Dt;
use crate::geom::ori;
use crate::tds::{canon_tri, same_cycle};

/// How many leading front faces the wrap looks at to pick the most constrained one.
const MRV_WIDTH: usize = 6;

/// Interior points the dead-face repair may add to one cavity.
const MAX_REPAIR: usize = 4;

impl Dt {
    /// Tetrahedralize the polyhedron bounded by `faces` (outward orientation) with exact
    /// closed-contact validity: depth-first gift wrapping with an empty-sphere preference and
    /// backtracking (complete for small cavities, bounded by a node budget). With
    /// `seg = Some((u,w))` no new tet may touch the open segment except by having `u-w` as an
    /// edge. `extra` are interior points (not on the front) the tets may use; a point that is
    /// not used must still lie outside every tet, which keeps it from being swallowed.
    ///
    /// `budget` bounds the number of candidate validity evaluations.
    pub(crate) fn fill_budgeted(&self, faces: &[[u32; 3]], seg: Option<(u32, u32)>, extra: &[u32], budget: usize) -> Option<Vec<[u32; 4]>> {
        self.fill_report(faces, seg, extra, budget).ok()
    }

    /// [`Dt::fill_budgeted`] that, on failure, says where the cavity is stuck (see [`FillFail`]).
    pub(crate) fn fill_report(&self, faces: &[[u32; 3]], seg: Option<(u32, u32)>, extra: &[u32], budget: usize) -> Result<Vec<[u32; 4]>, FillFail> {
        let mut verts: Vec<u32> = faces.iter().flatten().copied().chain(extra.iter().copied()).collect();
        verts.sort_unstable();
        verts.dedup();
        let mut front: std::collections::VecDeque<[u32; 3]> = faces.iter().map(|&f| canon_tri(f)).collect();
        let mut out: Vec<[u32; 4]> = Vec::new();
        let mut wrap = Wrap { verts: &verts, seg, budget, fail: FillFail::default() };
        if self.wrap_dfs(&mut front, &mut out, &mut wrap, 0) { Ok(out) } else { Err(wrap.fail) }
    }

    fn wrap_dfs(&self, front: &mut std::collections::VecDeque<[u32; 3]>, out: &mut Vec<[u32; 4]>, wrap: &mut Wrap<'_>, depth: usize) -> bool {
        if front.is_empty() {
            return true;
        }
        if wrap.budget == 0 {
            return false;
        }
        let (verts, seg) = (wrap.verts, wrap.seg);
        let s = self.sos();
        // Fail first: look at the first few front faces and wrap the one with the fewest valid
        // apexes; a face without any means this branch is dead.
        let mut pick: Option<(usize, Vec<u32>)> = None;
        let width = if depth == 0 { front.len() } else { front.len().min(MRV_WIDTH) };
        for idx in 0..width {
            let [x, y, z] = front[idx];
            let (px, py, pz) = (s.p(x), s.p(y), s.p(z));
            let mut cands: Vec<u32> = Vec::new();
            for &w in verts {
                if w == x || w == y || w == z || ori(px, py, pz, s.p(w)) >= 0 {
                    continue;
                }
                wrap.budget = wrap.budget.saturating_sub(1);
                match self.wrap_valid([w, x, y, z], front, verts, seg) {
                    Ok(()) => cands.push(w),
                    Err(Some(g)) if depth == 0 => {
                        if !wrap.fail.blockers.contains(&g) {
                            wrap.fail.blockers.push(g);
                        }
                    }
                    Err(_) => {}
                }
            }
            if cands.is_empty() {
                if wrap.fail.dead.is_none_or(|d| depth < d.0) {
                    wrap.fail.dead = Some((depth, front[idx]));
                }
                return false;
            }
            if pick.as_ref().is_none_or(|p| cands.len() < p.1.len()) {
                let one = cands.len() == 1;
                pick = Some((idx, cands));
                if one {
                    break;
                }
            }
        }
        let Some((idx, mut cands)) = pick else { return false };
        let Some(f) = front.remove(idx) else { return false };
        let [x, y, z] = f;
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
            if self.wrap_dfs(front, out, wrap, depth + 1) {
                return true;
            }
            out.pop();
            *front = saved;
            if wrap.budget == 0 {
                break;
            }
        }
        front.insert(idx, f);
        false
    }

    fn wrap_valid(&self, tet: [u32; 4], front: &std::collections::VecDeque<[u32; 3]>, verts: &[u32], seg: Option<(u32, u32)>) -> Result<(), Option<[u32; 3]>> {
        let s = self.sos();
        let [w, x, y, z] = tet;
        let tp = [s.p(w), s.p(x), s.p(y), s.p(z)];
        // Other cavity vertices may not touch the closed tet.
        for &c in verts {
            if !tet.contains(&c) && c < self.droppable_from && crate::exact::pt_in_closed_tet(tp, s.p(c)) {
                return Err(None);
            }
        }
        if let Some((u, v)) = seg
            && crate::exact::seg_touches_tet(tet, tp, u, v, s.p(u), s.p(v)) {
                return Err(None);
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
                    return Err(Some(*g));
                }
                let gv = [(g[0], s.p(g[0])), (g[1], s.p(g[1])), (g[2], s.p(g[2]))];
                let hv = [(h[0], s.p(h[0])), (h[1], s.p(h[1])), (h[2], s.p(h[2]))];
                if crate::exact::tris_conflict(gv, hv, s.p(*opp)) {
                    return Err(Some(*g));
                }
            }
        }
        Ok(())
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

/// State of one gift-wrapping search.
struct Wrap<'a> {
    verts: &'a [u32],
    seg: Option<(u32, u32)>,
    budget: usize,
    fail: FillFail,
}

/// Why a cavity fill failed.
#[derive(Default)]
pub(crate) struct FillFail {
    /// The shallowest front face found with no valid apex, as `(depth, face)`: where the cavity
    /// needs another vertex.
    pub dead: Option<(usize, [u32; 3])>,
    /// Front faces that made candidate tets over the first search level invalid by crossing or
    /// overlapping them: the cavity is too small on their other side.
    pub blockers: Vec<[u32; 3]>,
}

/// Steiner candidates supplied by the caller for a cavity around a segment: points in the wedge
/// on the solid side of the segment and on the exterior side (both best first).
#[derive(Default)]
pub(crate) struct Seeds {
    /// A point tried before all others: the centroid of a tiny surface component, the one point
    /// that sees a whole non-star-shaped pocket (Schönhardt-like prisms).
    pub pocket: Option<[f64; 3]>,
    pub inside: Vec<[f64; 3]>,
    pub outside: Vec<[f64; 3]>,
}

impl Dt {
    /// Fill a cavity with its own vertices; if that is impossible, retry with interior Steiner
    /// points the tets may use: the caller's seeds, the centroid and points just inside the faces
    /// (one or two at a time), then points added behind faces no vertex can close (up to
    /// `MAX_REPAIR`). Returns the tets and the number of Steiner points used; unused points are
    /// left out of the tets.
    pub(crate) fn fill_with_steiner(&mut self, faces: &[[u32; 3]], seg: Option<(u32, u32)>, inner: &[u32], seeds: &Seeds, budget: usize, guard: &Guard<'_>) -> Option<(Vec<[u32; 4]>, usize)> {
        use crate::geom::{add, cross, dist, dot, sanitize, scale, sub};
        if let Some(t) = self.fill_budgeted(faces, seg, inner, budget) {
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
        let cands: Vec<[f64; 3]> = scored.into_iter().filter_map(|(_, _, c)| sanitize(c)).take(12).collect();
        // Inside candidates: the caller's seeds and the cavity's own, best S4 score first (the
        // deepest point wins where no point complies, so one central point is preferred to
        // several shallow ones; the seeds come first among equals). Exterior seeds are free.
        let mut ins: Vec<(f64, usize, [f64; 3])> = seeds.inside.iter().copied().chain(cands.iter().copied()).filter_map(sanitize).enumerate().map(|(i, p)| (guard.score(p), i, p)).collect();
        ins.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
        let ins: Vec<[f64; 3]> = ins.into_iter().map(|x| x.2).collect();
        let outs: Vec<[f64; 3]> = seeds.outside.iter().copied().filter_map(sanitize).collect();
        let pts_base = self.pts.len();
        let mut singles: Vec<[f64; 3]> = seeds.pocket.into_iter().filter_map(sanitize).collect();
        singles.extend(ins.iter().take(8));
        singles.extend(outs.iter().take(6));
        for &p in singles.iter().take(14) {
            let base = self.pts.len();
            let id = self.push_point(p);
            if let Some(t) = self.fill_budgeted(faces, seg, &[inner, &[id]].concat(), budget)
                && t.iter().any(|tet| tet.contains(&id))
            {
                return Some((t, 1));
            }
            self.truncate_points(base);
        }
        if seg.is_none() {
            // Cavities of triangles and of zero-volume repairs: one point or none.
            self.truncate_points(pts_base);
            return None;
        }
        // A cavity with a slit (a constraint face kept inside it) has two sides that may each
        // need an interior point: one candidate on the solid side and one on the exterior side.
        let mut pairs: Vec<([f64; 3], [f64; 3])> = Vec::new();
        for &p in ins.iter().take(3) {
            for &q in outs.iter().take(3) {
                pairs.push((p, q));
            }
        }
        for (i, &p) in cands.iter().enumerate().take(4) {
            for &q in cands.iter().take(4).skip(i + 1) {
                pairs.push((p, q));
            }
        }
        for (p, q) in pairs.into_iter().take(6) {
            let base = self.pts.len();
            let a = self.push_point(p);
            let b = self.push_point(q);
            if let Some(t) = self.fill_budgeted(faces, seg, &[inner, &[a, b]].concat(), budget)
                && t.iter().any(|tet| tet.contains(&a))
                && t.iter().any(|tet| tet.contains(&b))
            {
                return Some((t, 2));
            }
            self.truncate_points(base);
        }
        // Dead-face repair: wherever the fill finds a face that no vertex can close, add a point
        // just behind that face (the side the cavity is on) and try again, a few times.
        let base = self.pts.len();
        let mut added: Vec<u32> = Vec::new();
        for _round in 0..MAX_REPAIR {
            let ex = [inner, &added[..]].concat();
            let dead = match self.fill_report(faces, seg, &ex, budget) {
                Ok(t) => {
                    let used = added.iter().filter(|a| t.iter().any(|tet| tet.contains(a))).count();
                    return Some((t, used));
                }
                Err(d) => d.dead,
            };
            let Some((_, f)) = dead else { break };
            let (a, b, d) = (self.pts[f[0] as usize], self.pts[f[1] as usize], self.pts[f[2] as usize]);
            let n = cross(sub(b, a), sub(d, a));
            let nl = dot(n, n).sqrt();
            if nl == 0.0 {
                break;
            }
            let h = (dist(a, b) + dist(b, d) + dist(d, a)) / 3.0;
            let fc = scale(add(add(a, b), d), 1.0 / 3.0);
            // Try the offsets deepest-first among the compliant ones; keep the one that gets the
            // furthest (success, or the dead face moves deeper).
            let mut offs: Vec<(f64, [f64; 3])> = Vec::new();
            let (e1, e2) = (sub(b, a), sub(d, a));
            for t in [0.5, 0.9] {
                for (i, j) in [(0.0, 0.0), (0.4, 0.0), (-0.4, 0.0), (0.0, 0.4), (0.0, -0.4), (0.3, 0.3), (-0.3, -0.3), (0.3, -0.3), (-0.3, 0.3)] {
                    let q = add(add(sub(fc, scale(n, t * h / nl)), scale(e1, i)), scale(e2, j));
                    if let Some(p) = sanitize(q) {
                        offs.push((guard.score(p), p));
                    }
                }
            }
            offs.sort_by_key(|x| std::cmp::Reverse(x.0 >= S4));
            let mut pick: Option<(usize, [f64; 3])> = None;
            for (_, p) in offs {
                let id = self.push_point(p);
                let ex2 = [&ex[..], &[id]].concat();
                match self.fill_report(faces, seg, &ex2, budget) {
                    Ok(t) => {
                        let mut all = added.clone();
                        all.push(id);
                        let used = all.iter().filter(|a| t.iter().any(|tet| tet.contains(a))).count();
                        return Some((t, used));
                    }
                    Err(FillFail { dead: Some((dd, _)), .. }) => {
                        if pick.is_none_or(|q| dd > q.0) {
                            pick = Some((dd, p));
                        }
                    }
                    Err(_) => {}
                }
                self.truncate_points(id as usize);
            }
            let Some((_, p)) = pick else { break };
            added.push(self.push_point(p));
        }
        self.truncate_points(base);
        None
    }
}
