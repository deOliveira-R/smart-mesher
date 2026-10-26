//! Exact cavity tetrahedralization by backtracking gift wrapping.

use crate::delaunay::Dt;
use crate::geom::ori;
use crate::tds::{FACE, canon_tri, same_cycle};

impl Dt {
    /// Tetrahedralize the polyhedron bounded by `faces` (outward orientation) with exact
    /// closed-contact validity: depth-first gift wrapping with an empty-sphere preference and
    /// backtracking (complete for small cavities, bounded by a node budget). With
    /// `seg = Some((u,w))` no new tet may touch the open segment except by having `u-w` as an
    /// edge.
    pub(crate) fn fill(&self, faces: &[[u32; 3]], seg: Option<(u32, u32)>) -> Option<Vec<[u32; 4]>> {
        let mut verts: Vec<u32> = faces.iter().flatten().copied().collect();
        verts.sort_unstable();
        verts.dedup();
        let mut front: std::collections::VecDeque<[u32; 3]> = faces.iter().map(|&f| canon_tri(f)).collect();
        let mut out: Vec<[u32; 4]> = Vec::new();
        let mut budget = 4000 + 200 * faces.len();
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
        *budget -= 1;
        let s = self.sos();
        let [x, y, z] = f;
        let (px, py, pz) = (s.p(x), s.p(y), s.p(z));
        let mut cands: Vec<u32> = Vec::new();
        for &w in verts {
            if w == x || w == y || w == z || ori(px, py, pz, s.p(w)) >= 0 {
                continue;
            }
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
        let dbg = std::env::var("TET_DEBUG").is_ok();
        for &c in verts {
            if !tet.contains(&c) && crate::exact::pt_in_closed_tet(tp, s.p(c)) {
                if dbg {
                    eprintln!("        reject {:?}: vertex {:?} in closed tet", tp, s.p(c));
                }
                return false;
            }
        }
        if let Some((u, v)) = seg {
            if crate::exact::seg_touches_tet(tet, tp, u, v, s.p(u), s.p(v)) {
                if dbg {
                    eprintln!("        reject {:?}: touches segment", tp);
                }
                return false;
            }
        }
        // Outward faces of the new tet other than the wrapped one, with the opposite vertex.
        let outward = [([w, z, y], x), ([w, x, z], y), ([w, y, x], z)];
        for g in front {
            for (h, opp) in &outward {
                if *g == canon_tri(*h) {
                    continue; // coincides with a front face: the tet closes it
                }
                if same_cycle(*g, [h[0], h[2], h[1]]) {
                    if dbg {
                        eprintln!("        reject: reverse front face");
                    }
                    return false;
                }
                let gv = [(g[0], s.p(g[0])), (g[1], s.p(g[1])), (g[2], s.p(g[2]))];
                let hv = [(h[0], s.p(h[0])), (h[1], s.p(h[1])), (h[2], s.p(h[2]))];
                if crate::exact::tris_conflict(gv, hv, s.p(*opp)) {
                    if dbg {
                        eprintln!("        reject {:?}: new face {:?} conflicts with front face {:?}", tp, hv.map(|x| x.1), gv.map(|x| x.1));
                    }
                    return false;
                }
            }
        }
        true
    }

}
