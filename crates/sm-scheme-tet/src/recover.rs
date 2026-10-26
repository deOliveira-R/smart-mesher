//! Boundary recovery without boundary Steiner points.
//!
//! Works on the Delaunay triangulation of the input points plus eight far auxiliary points
//! (see `DESIGN.md`). Constraint edges are recovered first, by flips that remove the edges
//! crossing the segment (n-to-m edge removal and 2-3 flips); then constraint triangles by
//! removing the tets they cross and re-tetrahedralizing the two cavities (gift wrapping with an
//! empty-sphere preference). A cavity that cannot be filled gets an interior Steiner point
//! (at most three tries) and is starred from it.
//!
//! All crossing decisions use symbolic perturbation (generic position); every tet created has
//! strictly positive exact orientation.

use crate::delaunay::Dt;
use crate::error::TetError;
use crate::flips::FlipScratch;
use crate::geom::{Pts, V3, add, dot, ori, pt, sanitize, scale, sub};
use crate::tds::{FACE, NONE, canon_tri, same_cycle, sorted3};

/// Constraint edges and triangles with their owners.
pub(crate) struct Constraints {
    pub edges: Vec<([u32; 2], u32)>,
    pub faces: Vec<([u32; 3], u32)>,
}

impl Constraints {
    pub fn new(tris: &[[u32; 3]]) -> Self {
        let mut edges: Vec<([u32; 2], u32)> = Vec::with_capacity(tris.len() * 3 / 2);
        let mut faces: Vec<([u32; 3], u32)> = Vec::with_capacity(tris.len());
        for (i, t) in tris.iter().enumerate() {
            faces.push((sorted3(t[0], t[1], t[2]), i as u32));
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                edges.push(([a.min(b), a.max(b)], i as u32));
            }
        }
        edges.sort_unstable();
        edges.dedup_by_key(|e| e.0);
        faces.sort_unstable();
        Self { edges, faces }
    }
    pub fn edge_owner(&self, a: u32, b: u32) -> Option<u32> {
        let k = [a.min(b), a.max(b)];
        self.edges.binary_search_by(|e| e.0.cmp(&k)).ok().map(|i| self.edges[i].1)
    }
    pub fn face_owner(&self, k: [u32; 3]) -> Option<u32> {
        self.faces.binary_search_by(|e| e.0.cmp(&k)).ok().map(|i| self.faces[i].1)
    }
}

/// Recovery driver.
pub(crate) struct Recover<'a> {
    pub dt: &'a mut Dt,
    pub con: &'a Constraints,
    pub flips: usize,
    pub max_steiner: usize,
    pub n_steiner: usize,
    sc: FlipScratch,
    buf: Vec<u32>,
    tabu: Vec<[u32; 2]>,
}

fn fail(tri: u32, edge: Option<[u32; 2]>, reason: impl Into<String>) -> TetError {
    TetError::RecoveryFailed { tri, edge, reason: reason.into() }
}

impl<'a> Recover<'a> {
    pub fn new(dt: &'a mut Dt, con: &'a Constraints, max_steiner: usize) -> Self {
        Self { dt, con, flips: 0, max_steiner, n_steiner: 0, sc: FlipScratch::default(), buf: Vec::new(), tabu: Vec::new() }
    }

    /// Recover every constraint edge and triangle.
    pub fn run(&mut self) -> Result<(), TetError> {
        for ei in 0..self.con.edges.len() {
            let (e, owner) = self.con.edges[ei];
            self.ensure_edge(e[0], e[1], owner)?;
        }
        for _round in 0..8 {
            let mut changed = false;
            for fi in 0..self.con.faces.len() {
                let (k, tri) = self.con.faces[fi];
                if !self.face_present(k) {
                    self.recover_face(tri, k)?;
                    changed = true;
                }
            }
            for ei in 0..self.con.edges.len() {
                let (e, owner) = self.con.edges[ei];
                if self.dt.tds.find_edge(e[0], e[1]).is_none() {
                    self.ensure_edge(e[0], e[1], owner)?;
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
        for &(k, tri) in &self.con.faces {
            if !self.face_present(k) {
                return Err(fail(tri, None, "triangle still missing after recovery rounds"));
            }
        }
        Ok(())
    }

    fn face_present(&mut self, k: [u32; 3]) -> bool {
        let mut buf = std::mem::take(&mut self.buf);
        self.dt.tds.star(k[0], &mut buf);
        let r = buf.iter().any(|&t| self.dt.tds.has(t, k[1]) && self.dt.tds.has(t, k[2]));
        self.buf = buf;
        r
    }

    // ---------------------------------------------------------------- edges

    fn seg_crosses_tet(&self, v: [u32; 4], u: u32, w: u32) -> bool {
        let s = self.dt.sos();
        let hu = v.iter().position(|&x| x == u);
        let hw = v.iter().position(|&x| x == w);
        let cone = |apex: usize, target: u32| -> bool {
            (0..4).filter(|&k| k != apex).all(|k| {
                let f = FACE[k];
                s.ori_s(v[f[0]], v[f[1]], v[f[2]], target) < 0
            })
        };
        match (hu, hw) {
            (Some(_), Some(_)) => false,
            (Some(a), None) => cone(a, w),
            (None, Some(b)) => cone(b, u),
            (None, None) => (0..4).any(|k| {
                let f = FACE[k];
                let (a, b, c) = (v[f[0]], v[f[1]], v[f[2]]);
                self.pierces(u, w, a, b, c)
            }),
        }
    }

    /// Does segment `(p,q)` cross triangle `(a,b,c)` (generic position)?
    fn pierces(&self, p: u32, q: u32, a: u32, b: u32, c: u32) -> bool {
        let s = self.dt.sos();
        if s.ori_s(a, b, c, p) == s.ori_s(a, b, c, q) {
            return false;
        }
        let o1 = s.ori_s(p, q, a, b);
        o1 == s.ori_s(p, q, b, c) && o1 == s.ori_s(p, q, c, a)
    }

    /// The tets crossed by segment `(u,w)` from `u` to `w`, and the crossed faces
    /// `(tet, slot)` (face of `tet` towards the next tet).
    fn corridor(&mut self, u: u32, w: u32) -> Option<(Vec<u32>, Vec<(u32, usize)>)> {
        let mut star = Vec::new();
        self.dt.tds.star(u, &mut star);
        let s = self.dt.sos();
        let mut t = NONE;
        for &c in &star {
            if self.dt.tds.ghost_slot(c).is_some() {
                continue;
            }
            let v = self.dt.tds.verts[c as usize];
            let su = v.iter().position(|&x| x == u)?;
            let inside = (0..4).filter(|&k| k != su).all(|k| {
                let f = FACE[k];
                s.ori_s(v[f[0]], v[f[1]], v[f[2]], w) < 0
            });
            if inside {
                t = c;
                break;
            }
        }
        if t == NONE {
            return None;
        }
        let mut tets = vec![t];
        let mut faces = Vec::new();
        let mut entry: Option<u32> = None; // tet we came from
        let limit = 4 * self.dt.tds.slots() + 16;
        for _ in 0..limit {
            let v = self.dt.tds.verts[t as usize];
            if v.contains(&w) {
                return Some((tets, faces));
            }
            let mut exit = None;
            for k in 0..4 {
                let n = self.dt.tds.nbr[t as usize][k];
                if n == NONE || Some(n) == entry {
                    continue;
                }
                let f = FACE[k];
                let (a, b, c) = (v[f[0]], v[f[1]], v[f[2]]);
                if a == u || b == u || c == u {
                    continue;
                }
                if self.pierces(u, w, a, b, c) {
                    exit = Some((k, n));
                    break;
                }
            }
            let (k, n) = exit?;
            faces.push((t, k));
            entry = Some(t);
            t = n;
            tets.push(t);
        }
        None
    }

    fn ensure_edge(&mut self, u: u32, w: u32, owner: u32) -> Result<(), TetError> {
        self.tabu.clear();
        for _iter in 0..600 {
            if self.dt.tds.find_edge(u, w).is_some() {
                return Ok(());
            }
            let Some((tets, faces)) = self.corridor(u, w) else {
                return Err(fail(owner, Some([u, w]), "could not trace the edge through the triangulation"));
            };
            for &(t, k) in &faces {
                let f = self.dt.tds.face(t, k);
                if let Some(other) = self.con.face_owner(sorted3(f[0], f[1], f[2])) {
                    return Err(TetError::SelfIntersection { tri_a: owner.min(other), tri_b: owner.max(other) });
                }
            }
            let before = tets.len();
            if std::env::var("TET_DEBUG").is_ok() {
                let pp = |i: u32| self.dt.pts[i as usize];
                eprintln!("ensure_edge {u}->{w} {:?}->{:?}: corridor {} tets", pp(u), pp(w), before);
                for &(t, k) in &faces {
                    let f = self.dt.tds.face(t, k);
                    eprintln!("   crossed face {:?} {:?} {:?}", pp(f[0]), pp(f[1]), pp(f[2]));
                }
            }
            let mut sorted_c = tets.clone();
            sorted_c.sort_unstable();
            let after_of = |this: &Self, old: &[u32], new: &[[u32; 4]]| -> usize {
                if new.iter().any(|v| v.contains(&u) && v.contains(&w)) {
                    return 0;
                }
                let removed = old.iter().filter(|t| sorted_c.binary_search(t).is_ok()).count();
                let added = new.iter().filter(|&&v| this.seg_crosses_tet(v, u, w)).count();
                before - removed + added
            };
            // Edge removals of the edges of crossed faces.
            let mut best: Option<(usize, usize, [u32; 2], crate::flips::Plan)> = None;
            let mut seen: Vec<[u32; 2]> = Vec::new();
            for &(t, k) in &faces {
                let f = self.dt.tds.face(t, k);
                for e in 0..3 {
                    let (x, y) = (f[e], f[(e + 1) % 3]);
                    let key = [x.min(y), x.max(y)];
                    if seen.contains(&key) || self.tabu.contains(&key) || self.con.edge_owner(x, y).is_some() {
                        continue;
                    }
                    seen.push(key);
                    if std::env::var("TET_DEBUG").is_ok() {
                        let mut r = Vec::new();
                        let mut p = Vec::new();
                        let ok = self.dt.tds.ring(x, y, t, 16, &mut r, &mut p);
                        eprintln!("   cand edge {:?}-{:?} ring ok={ok} k={} poly={:?}", self.dt.pts[x as usize], self.dt.pts[y as usize], r.len(), p.iter().map(|&i| if i == u32::MAX { [9.9; 3] } else { self.dt.pts[i as usize] }).collect::<Vec<_>>());
                    }
                    if let Some(plan) = self.dt.plan_edge_removal(x, y, t, 16, &mut self.sc) {
                        let after = after_of(self, &plan.old, &plan.new);
                        let cand = (after, plan.new.len(), key);
                        if best.as_ref().is_none_or(|b| cand < (b.0, b.1, b.2)) {
                            best = Some((cand.0, cand.1, cand.2, plan));
                        }
                    }
                }
            }
            if let Some((after, _, _, plan)) = best {
                if after < before {
                    self.dt.apply(&plan)?;
                    self.flips += 1;
                    continue;
                }
            }
            // 2-3 flips of crossed faces (reshape), best-first, never undone by edge removal.
            let mut best23: Option<(usize, [u32; 2], crate::flips::Plan)> = None;
            for &(t, k) in &faces {
                if let Some(plan) = self.dt.plan_flip23(t, k) {
                    let after = after_of(self, &plan.old, &plan.new);
                    let ne = [plan.new[0][0].min(plan.new[0][1]), plan.new[0][0].max(plan.new[0][1])];
                    if self.tabu.contains(&ne) {
                        continue;
                    }
                    if best23.as_ref().is_none_or(|b| (after, ne) < (b.0, b.1)) {
                        best23 = Some((after, ne, plan));
                    }
                }
            }
            if let Some((after, ne, plan)) = best23 {
                if after <= before {
                    self.dt.apply(&plan)?;
                    self.flips += 1;
                    self.tabu.push(ne);
                    continue;
                }
            }
            break;
        }
        self.edge_by_cavity(u, w, owner)
    }

    // ---------------------------------------------------------------- faces

    /// Do triangles `g` and `f` intersect improperly (generic position)?
    fn tri_pierce(&self, g: [u32; 3], f: [u32; 3]) -> bool {
        for e in 0..3 {
            let (p, q) = (g[e], g[(e + 1) % 3]);
            if !f.contains(&p) && !f.contains(&q) && self.pierces(p, q, f[0], f[1], f[2]) {
                return true;
            }
            let (p, q) = (f[e], f[(e + 1) % 3]);
            if !g.contains(&p) && !g.contains(&q) && self.pierces(p, q, g[0], g[1], g[2]) {
                return true;
            }
        }
        false
    }

    fn recover_face(&mut self, tri: u32, key: [u32; 3]) -> Result<(), TetError> {
        // The oriented triangle is not needed here, only its vertices.
        let [a, b, c] = key;
        for &(x, y) in &[(a, b), (b, c), (a, c)] {
            if self.dt.tds.find_edge(x, y).is_none() {
                self.ensure_edge(x, y, tri)?;
            }
        }
        if self.face_present(key) {
            return Ok(());
        }
        // Seed: the tet around edge (a,b) whose wedge contains the direction to c.
        let Some(t0) = self.dt.tds.find_edge(a, b) else { return Err(fail(tri, Some([a, b]), "edge vanished")) };
        let s = Pts(&self.dt.pts);
        let mut ring = Vec::new();
        let mut poly = Vec::new();
        if !self.dt.tds.ring(a, b, t0, 64, &mut ring, &mut poly) {
            return Err(fail(tri, Some([a, b]), "open ring around a constraint edge"));
        }
        let mut seed = NONE;
        for &t in &ring {
            let v = self.dt.tds.verts[t as usize];
            let mut o = v.iter().copied().filter(|&x| x != a && x != b);
            let (Some(mut x), Some(mut y)) = (o.next(), o.next()) else { continue };
            if s.ori_s(a, b, x, y) < 0 {
                std::mem::swap(&mut x, &mut y);
            }
            if s.ori_s(a, b, x, c) > 0 && s.ori_s(a, b, c, y) > 0 {
                seed = t;
                break;
            }
        }
        if seed == NONE {
            return Err(fail(tri, None, "no tet around the edge contains the triangle direction"));
        }
        // BFS over crossing tets.
        let mut rset: Vec<u32> = vec![seed];
        let mut head = 0;
        while head < rset.len() {
            let t = rset[head];
            head += 1;
            for k in 0..4 {
                let n = self.dt.tds.nbr[t as usize][k];
                if n == NONE || rset.contains(&n) {
                    continue;
                }
                let g = self.dt.tds.face(t, k);
                if g.iter().all(|x| key.contains(x)) {
                    continue;
                }
                if self.tri_pierce(g, key) {
                    if let Some(other) = self.con.face_owner(sorted3(g[0], g[1], g[2])) {
                        return Err(TetError::SelfIntersection { tri_a: tri.min(other), tri_b: tri.max(other) });
                    }
                    if self.dt.tds.ghost_slot(n).is_some() {
                        return Err(fail(tri, None, "triangle reaches the auxiliary hull"));
                    }
                    rset.push(n);
                }
            }
            if rset.len() > 4096 {
                return Err(fail(tri, None, "crossing region too large"));
            }
        }
        // Constraint edges that pierce the triangle, and internal constraint faces.
        for &t in &rset {
            let v = self.dt.tds.verts[t as usize];
            for i in 0..4 {
                for j in i + 1..4 {
                    let (p, q) = (v[i], v[j]);
                    if !key.contains(&p) && !key.contains(&q) {
                        if let Some(other) = self.con.edge_owner(p, q) {
                            if self.pierces(p, q, a, b, c) {
                                return Err(TetError::SelfIntersection { tri_a: tri.min(other), tri_b: tri.max(other) });
                            }
                        }
                    }
                }
            }
            for k in 0..4 {
                let n = self.dt.tds.nbr[t as usize][k];
                if n != NONE && rset.contains(&n) {
                    let g = self.dt.tds.face(t, k);
                    if let Some(other) = self.con.face_owner(sorted3(g[0], g[1], g[2])) {
                        return Err(fail(tri, None, format!("triangle {other} lies inside the cavity of this triangle")));
                    }
                }
            }
        }
        // Split the cavity boundary by the side of the triangle's plane.
        let mut top: Vec<[u32; 3]> = Vec::new();
        let mut bot: Vec<[u32; 3]> = Vec::new();
        let mut bverts: Vec<u32> = Vec::new();
        for &t in &rset {
            for k in 0..4 {
                let n = self.dt.tds.nbr[t as usize][k];
                if n != NONE && rset.contains(&n) {
                    continue;
                }
                let g = self.dt.tds.face(t, k);
                let mut up = false;
                let mut down = false;
                for &w in &g {
                    bverts.push(w);
                    if key.contains(&w) {
                        continue;
                    }
                    if s.ori_s(a, b, c, w) > 0 {
                        up = true;
                    } else {
                        down = true;
                    }
                }
                match (up, down) {
                    (true, false) => top.push(g),
                    (false, true) => bot.push(g),
                    (false, false) => return Err(fail(tri, None, "cavity boundary face made of triangle vertices only")),
                    (true, true) => return Err(fail(tri, None, "cavity boundary face straddles the triangle plane")),
                }
            }
        }
        bverts.sort_unstable();
        bverts.dedup();
        for &t in &rset {
            for &x in &self.dt.tds.verts[t as usize] {
                if bverts.binary_search(&x).is_err() {
                    return Err(fail(tri, None, "a vertex is enclosed by the cavity"));
                }
            }
        }
        // Caps: outward from the top polyhedron the triangle faces down (a,c,b).
        let mut new_tets: Vec<[u32; 4]> = Vec::new();
        for (side, faces) in [(true, &mut top), (false, &mut bot)] {
            let cap = if side { [a, c, b] } else { [a, b, c] };
            faces.insert(0, cap);
            match self.fill(faces, None) {
                Some(t) => new_tets.extend(t),
                None => {
                    let t = self.star_fill(tri, faces)?;
                    new_tets.extend(t);
                }
            }
        }
        let mut ids = Vec::new();
        self.dt.tds.replace(&rset, &new_tets, &mut ids).map_err(|e| fail(tri, None, format!("cavity commit failed: {e}")))?;
        self.flips += 1;
        Ok(())
    }

    // ---------------------------------------------------------------- cavity filling

    /// Tetrahedralize the polyhedron bounded by `faces` (outward orientation) with exact
    /// closed-contact validity: depth-first gift wrapping with an empty-sphere preference and
    /// backtracking (complete for small cavities, bounded by a node budget). With
    /// `seg = Some((u,w))` no new tet may touch the open segment except by having `u-w` as an
    /// edge.
    fn fill(&self, faces: &[[u32; 3]], seg: Option<(u32, u32)>) -> Option<Vec<[u32; 4]>> {
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
        let s = self.dt.sos();
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
        let s = self.dt.sos();
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

    /// Edge recovery by cavity re-tetrahedralization: remove every tet touching the open segment
    /// and refill with the segment as a mandatory edge.
    fn edge_by_cavity(&mut self, u: u32, w: u32, owner: u32) -> Result<(), TetError> {
        let (up, wp) = (self.dt.pts[u as usize], self.dt.pts[w as usize]);
        let touches = |this: &Self, t: u32| -> bool {
            let v = this.dt.tds.verts[t as usize];
            if v.contains(&crate::tds::GHOST) {
                return false;
            }
            let p = |i: usize| this.dt.pts[v[i] as usize];
            crate::exact::seg_touches_tet(v, [p(0), p(1), p(2), p(3)], u, w, up, wp)
        };
        let mut star = Vec::new();
        self.dt.tds.star(u, &mut star);
        let mut r: Vec<u32> = star.into_iter().filter(|&t| touches(self, t)).collect();
        if r.is_empty() {
            return Err(fail(owner, Some([u, w]), "no tet touches the edge"));
        }
        let mut tested: Vec<u32> = r.clone();
        let mut head = 0;
        while head < r.len() {
            let t = r[head];
            head += 1;
            for k in 0..4 {
                let n = self.dt.tds.nbr[t as usize][k];
                if n == NONE || tested.contains(&n) {
                    continue;
                }
                tested.push(n);
                if touches(self, n) {
                    r.push(n);
                }
            }
            if r.len() > 2048 {
                return Err(fail(owner, Some([u, w]), "edge cavity too large"));
            }
        }
        for level in 0..4 {
            if level > 0 {
                // Grow by one layer of neighbours across non-constraint faces.
                let snapshot = r.clone();
                for &t in &snapshot {
                    for k in 0..4 {
                        let n = self.dt.tds.nbr[t as usize][k];
                        if n == NONE || r.contains(&n) || self.dt.tds.ghost_slot(n).is_some() {
                            continue;
                        }
                        let g = self.dt.tds.face(t, k);
                        if self.con.face_owner(sorted3(g[0], g[1], g[2])).is_some() {
                            continue;
                        }
                        // Do not swallow a constraint face between two cavity tets.
                        let swallows = (0..4).any(|j| {
                            let m = self.dt.tds.nbr[n as usize][j];
                            m != NONE && m != t && r.contains(&m) && {
                                let h = self.dt.tds.face(n, j);
                                self.con.face_owner(sorted3(h[0], h[1], h[2])).is_some()
                            }
                        });
                        if swallows {
                            continue;
                        }
                        r.push(n);
                    }
                }
                if r.len() > 2048 {
                    break;
                }
            }
            let mut bfaces: Vec<[u32; 3]> = Vec::new();
            for &t in &r {
                for k in 0..4 {
                    let n = self.dt.tds.nbr[t as usize][k];
                    let g = self.dt.tds.face(t, k);
                    if n != NONE && r.contains(&n) {
                        if let Some(other) = self.con.face_owner(sorted3(g[0], g[1], g[2])) {
                            if std::env::var("TET_DEBUG").is_ok() {
                                eprintln!("  edge cavity {:?}->{:?}: internal constraint face {other}: {:?}", self.dt.pts[u as usize], self.dt.pts[w as usize], g.map(|i| self.dt.pts[i as usize]));
                            }
                            return Err(TetError::SelfIntersection { tri_a: owner.min(other), tri_b: owner.max(other) });
                        }
                        continue;
                    }
                    bfaces.push(g);
                }
            }
            let mut bverts: Vec<u32> = bfaces.iter().flatten().copied().collect();
            bverts.sort_unstable();
            bverts.dedup();
            let enclosed = r.iter().any(|&t| self.dt.tds.verts[t as usize].iter().any(|x| bverts.binary_search(x).is_err()));
            if enclosed {
                continue;
            }
            if let Some(new_tets) = self.fill(&bfaces, Some((u, w))) {
                let mut ids = Vec::new();
                self.dt.tds.replace(&r, &new_tets, &mut ids).map_err(|e| fail(owner, Some([u, w]), format!("edge cavity commit failed: {e}")))?;
                self.flips += 1;
                if self.dt.tds.find_edge(u, w).is_none() {
                    return Err(fail(owner, Some([u, w]), "edge missing after cavity refill"));
                }
                return Ok(());
            }
        }
        Err(fail(owner, Some([u, w]), "edge cavity cannot be tetrahedralized with the edge"))
    }

    /// Interior Steiner fallback: find a point that sees every face (strictly) and star it.
    fn star_fill(&mut self, tri: u32, faces: &[[u32; 3]]) -> Result<Vec<[u32; 4]>, TetError> {
        let pts: Vec<V3> = self.dt.pts.clone();
        let mut verts: Vec<u32> = faces.iter().flatten().copied().collect();
        verts.sort_unstable();
        verts.dedup();
        let cen = {
            let mut c = [0.0; 3];
            for &v in &verts {
                c = add(c, pts[v as usize]);
            }
            scale(c, 1.0 / verts.len() as f64)
        };
        let mut fc = [0.0; 3];
        for f in faces {
            fc = add(fc, scale(add(add(pts[f[0] as usize], pts[f[1] as usize]), pts[f[2] as usize]), 1.0 / 3.0));
        }
        let fcen = scale(fc, 1.0 / faces.len() as f64);
        let visible = |p: V3| faces.iter().all(|f| ori(pts[f[0] as usize], pts[f[1] as usize], pts[f[2] as usize], p) < 0);
        let mut cands: Vec<V3> = vec![cen, fcen];
        // Kernel search: alternating projections onto the inner half spaces.
        let mut p = cen;
        let h = {
            let mut m = 0.0;
            for f in faces {
                m += crate::geom::dist(pts[f[0] as usize], pts[f[1] as usize]);
            }
            m / faces.len() as f64
        };
        for _ in 0..60 {
            let mut moved = false;
            for f in faces {
                let (a, b, c) = (pts[f[0] as usize], pts[f[1] as usize], pts[f[2] as usize]);
                let n = crate::geom::cross(sub(b, a), sub(c, a));
                let nn = dot(n, n).sqrt();
                if nn == 0.0 {
                    continue;
                }
                let n = scale(n, 1.0 / nn);
                let d = dot(sub(p, a), n);
                // Inner side is negative; require at least 0.05 h inside.
                if d > -0.05 * h {
                    p = sub(p, scale(n, d + 0.1 * h));
                    moved = true;
                }
            }
            if !moved {
                break;
            }
        }
        cands.push(p);
        for c in cands.into_iter().take(3) {
            let Some(c) = sanitize(c) else { continue };
            if !visible(c) {
                continue;
            }
            if self.n_steiner + 1 > self.max_steiner {
                return Err(TetError::SteinerBudget { limit: self.max_steiner });
            }
            let id = self.dt.push_point(c);
            self.n_steiner += 1;
            return Ok(faces.iter().map(|f| [id, f[0], f[1], f[2]]).collect());
        }
        Err(fail(tri, None, "cavity cannot be tetrahedralized and has no interior kernel point (Steiner fallback failed)"))
    }
}

#[allow(dead_code)]
fn _unused(_: V3) {
    let _ = pt([0.0; 3]);
    let _ = Pts(&[]);
}
