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
use crate::tds::{FACE, NONE, sorted3};

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
    /// Points below this index are input points (the far points follow them).
    pub n_real: u32,
    /// Interior points suggested by failed recoveries (for the caller's restart).
    pub hints: Vec<V3>,
    /// Restart number (selects which hint candidates are produced).
    pub attempt: usize,
    /// Cheap mode (the first restarts): only quick repairs, failures become hints.
    cheap: bool,
    sc: FlipScratch,
    buf: Vec<u32>,
    tabu: Vec<[u32; 2]>,
}

/// Number of leading attempts that only try quick repairs (flips and small cavities).
const CHEAP_ATTEMPTS: usize = 2;

/// Tets crossed by a segment and the crossed faces as `(tet, slot)`.
type Corridor = (Vec<u32>, Vec<(u32, usize)>);

fn fail(tri: u32, edge: Option<[u32; 2]>, reason: impl Into<String>) -> TetError {
    TetError::RecoveryFailed { tri, edge, reason: reason.into() }
}

impl<'a> Recover<'a> {
    pub fn new(dt: &'a mut Dt, con: &'a Constraints, max_steiner: usize, n_real: u32, attempt: usize) -> Self {
        Self { dt, con, flips: 0, max_steiner, n_steiner: 0, n_real, hints: Vec::new(), attempt, cheap: attempt < CHEAP_ATTEMPTS, sc: FlipScratch::default(), buf: Vec::new(), tabu: Vec::new() }
    }

    /// Recover every constraint edge and triangle. A recovery failure does not stop the pass:
    /// every failing constraint contributes interior point hints ([`Recover::hints`]) and the
    /// first failure is returned at the end.
    pub fn run(&mut self) -> Result<(), TetError> {
        let mut first: Option<TetError> = None;
        let note = |first: &mut Option<TetError>, e: TetError| -> Result<(), TetError> {
            match e {
                e @ TetError::RecoveryFailed { .. } => {
                    first.get_or_insert(e);
                    Ok(())
                }
                other => Err(other),
            }
        };
        for ei in 0..self.con.edges.len() {
            let (e, owner) = self.con.edges[ei];
            if let Err(err) = self.ensure_edge(e[0], e[1], owner) {
                note(&mut first, err)?;
            }
        }
        if first.is_some() {
            return Err(first.unwrap_or_else(|| fail(0, None, "edge recovery failed")));
        }
        for _round in 0..8 {
            let mut changed = false;
            for fi in 0..self.con.faces.len() {
                let (k, tri) = self.con.faces[fi];
                if !self.face_present(k) {
                    if let Err(err) = self.recover_face(tri, k) {
                        note(&mut first, err)?;
                    }
                    changed = true;
                }
            }
            if first.is_some() {
                break;
            }
            for ei in 0..self.con.edges.len() {
                let (e, owner) = self.con.edges[ei];
                if self.dt.tds.find_edge(e[0], e[1]).is_none() {
                    if let Err(err) = self.ensure_edge(e[0], e[1], owner) {
                        note(&mut first, err)?;
                    }
                    changed = true;
                }
            }
            if first.is_some() || !changed {
                break;
            }
        }
        if let Some(e) = first {
            return Err(e);
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
    fn corridor(&mut self, u: u32, w: u32) -> Option<Corridor> {
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
            for (k, f) in FACE.iter().enumerate() {
                let n = self.dt.tds.nbr[t as usize][k];
                if n == NONE || Some(n) == entry {
                    continue;
                }
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
        match self.ensure_edge_once(u, w, owner) {
            Err(e @ TetError::RecoveryFailed { .. }) => {
                let hints = self.edge_hints(u, w);
                self.hints.extend(hints);
                Err(e)
            }
            other => other,
        }
    }

    /// Points worth adding when edge `(u,w)` cannot be recovered: the centroid of the input
    /// vertices around the tets that touch the segment, and a point between it and the segment
    /// midpoint (at most two). They may lie outside the solid: the triangulation outside the
    /// surface must be conforming too, and Steiner points that end up in the outside region are
    /// dropped with it.
    fn edge_hints(&mut self, u: u32, w: u32) -> Vec<V3> {
        let (up, wp) = (self.dt.pts[u as usize], self.dt.pts[w as usize]);
        let mut star = Vec::new();
        self.dt.tds.star(u, &mut star);
        let mut touching: Vec<u32> = Vec::new();
        let mut stack: Vec<u32> = star.into_iter().filter(|&t| self.touches(t, u, w, up, wp)).collect();
        while let Some(t) = stack.pop() {
            if touching.contains(&t) {
                continue;
            }
            touching.push(t);
            for k in 0..4 {
                let n = self.dt.tds.nbr[t as usize][k];
                if n != NONE && !touching.contains(&n) && self.touches(n, u, w, up, wp) {
                    stack.push(n);
                }
            }
            if touching.len() > 512 {
                break;
            }
        }
        let mut vs: Vec<u32> = touching.iter().flat_map(|&t| self.dt.tds.verts[t as usize]).filter(|&x| x < self.n_real).collect();
        vs.sort_unstable();
        vs.dedup();
        if vs.is_empty() {
            return Vec::new();
        }
        let mut c = [0.0; 3];
        for &v in &vs {
            c = add(c, self.dt.pts[v as usize]);
        }
        let c = scale(c, 1.0 / vs.len() as f64);
        let mid = scale(add(up, wp), 0.5);
        // Chambers next to the edge: on both sides of both incident surface triangles, at a
        // fraction of the triangle size (what the failing cavity usually needs is a point in the
        // pocket the diagonals enclose, inside or outside the solid).
        let mut chamber: Vec<V3> = Vec::new();
        let frac = [0.5, 0.35, 0.7][self.attempt % 3];
        for &(key, _) in self.con.faces.iter().filter(|(k, _)| k.contains(&u) && k.contains(&w)) {
            let [a, b, d] = key.map(|i| self.dt.pts[i as usize]);
            let n = crate::geom::cross(sub(b, a), sub(d, a));
            let nl = dot(n, n).sqrt();
            if nl == 0.0 {
                continue;
            }
            let h = (crate::geom::dist(a, b) + crate::geom::dist(b, d) + crate::geom::dist(d, a)) / 3.0;
            let fc = scale(add(add(a, b), d), 1.0 / 3.0);
            for sgn in [-1.0, 1.0] {
                chamber.push(add(fc, scale(n, sgn * frac * h / nl)));
            }
        }
        if self.attempt < 6 && !chamber.is_empty() {
            return chamber.into_iter().filter_map(sanitize).collect();
        }
        let mut cands: Vec<V3> = vec![c, add(scale(c, 0.5), scale(mid, 0.5))];
        for &v in &vs {
            if v != u && v != w {
                cands.push(add(mid, scale(sub(self.dt.pts[v as usize], mid), 0.4)));
            }
        }
        // Successive restarts use different candidates: the fixed ones first, then low-discrepancy
        // points spread over the neighbourhood (fresh ones for every restart).
        if self.attempt >= 6 {
            let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
            for &v in &vs {
                for a in 0..3 {
                    lo[a] = lo[a].min(self.dt.pts[v as usize][a]);
                    hi[a] = hi[a].max(self.dt.pts[v as usize][a]);
                }
            }
            let radical = |mut n: usize, b: usize| {
                let (mut f, mut r) = (1.0, 0.0);
                while n > 0 {
                    f /= b as f64;
                    r += f * (n % b) as f64;
                    n /= b;
                }
                r
            };
            let mut fresh = Vec::new();
            for j in 0..2 {
                let n = 2 * self.attempt + j + 1;
                fresh.push([
                    lo[0] + (hi[0] - lo[0]) * radical(n, 2),
                    lo[1] + (hi[1] - lo[1]) * radical(n, 3),
                    lo[2] + (hi[2] - lo[2]) * radical(n, 5),
                ]);
            }
            return fresh.into_iter().filter_map(sanitize).collect();
        }
        let skip = (2 * self.attempt) % cands.len();
        cands.rotate_left(skip);
        cands.into_iter().filter_map(sanitize).take(2).collect()
    }

    fn touches(&self, t: u32, u: u32, w: u32, up: V3, wp: V3) -> bool {
        let v = self.dt.tds.verts[t as usize];
        if v.contains(&crate::tds::GHOST) {
            return false;
        }
        let p = |i: usize| self.dt.pts[v[i] as usize];
        crate::exact::seg_touches_tet(v, [p(0), p(1), p(2), p(3)], u, w, up, wp)
    }

    fn ensure_edge_once(&mut self, u: u32, w: u32, owner: u32) -> Result<(), TetError> {
        self.tabu.clear();
        for _iter in 0..120 {
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
                    if let Some(plan) = self.dt.plan_edge_removal(x, y, t, 16, &mut self.sc) {
                        let after = after_of(self, &plan.old, &plan.new);
                        let cand = (after, plan.new.len(), key);
                        if best.as_ref().is_none_or(|b| cand < (b.0, b.1, b.2)) {
                            best = Some((cand.0, cand.1, cand.2, plan));
                        }
                    }
                }
            }
            if let Some((after, _, _, plan)) = best
                && after < before {
                    self.dt.apply(&plan)?;
                    self.flips += 1;
                    continue;
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
            if let Some((after, ne, plan)) = best23
                && after <= before {
                    self.dt.apply(&plan)?;
                    self.flips += 1;
                    self.tabu.push(ne);
                    continue;
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
                    if !key.contains(&p) && !key.contains(&q)
                        && let Some(other) = self.con.edge_owner(p, q)
                            && self.pierces(p, q, a, b, c) {
                                return Err(TetError::SelfIntersection { tri_a: tri.min(other), tri_b: tri.max(other) });
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
            match self.fill_steiner(faces, None) {
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
        let max_level = if self.cheap { 2 } else { 4 };
        for level in 0..max_level {
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
            if let Some(new_tets) = self.fill_steiner(&bfaces, Some((u, w))) {
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

    /// Fill a cavity (see [`Dt::fill_with_steiner`]); counts the Steiner point it adds.
    fn fill_steiner(&mut self, faces: &[[u32; 3]], seg: Option<(u32, u32)>) -> Option<Vec<[u32; 4]>> {
        if self.cheap {
            // First passes only look for the easy fills; failures are cheap and become hints.
            return self.dt.fill_budgeted(faces, seg, &[], 120 + 3 * faces.len());
        }
        if self.n_steiner + 1 > self.max_steiner {
            return self.dt.fill(faces, seg, &[]);
        }
        let (t, used) = self.dt.fill_with_steiner(faces, seg, 2000 + 40 * faces.len())?;
        self.n_steiner += used;
        Some(t)
    }

    /// Interior Steiner fallback: find a point that sees every face (strictly) and star it.
    fn star_fill(&mut self, tri: u32, faces: &[[u32; 3]]) -> Result<Vec<[u32; 4]>, TetError> {
        let Some(c) = self.dt.kernel_point(faces) else {
            // Suggest the centroid of the cavity (if inside the solid) to the caller's restart.
            let mut vs: Vec<u32> = faces.iter().flatten().copied().collect();
            vs.sort_unstable();
            vs.dedup();
            let mut c = [0.0; 3];
            for &v in &vs {
                c = add(c, self.dt.pts[v as usize]);
            }
            let c = scale(c, 1.0 / vs.len() as f64);
            let k = self.attempt % vs.len();
            let toward = self.dt.pts[vs[k] as usize];
            let blend = if self.attempt == 0 { c } else { add(scale(c, 0.6), scale(toward, 0.4)) };
            if let Some(c) = sanitize(blend) {
                self.hints.push(c);
            }
            return Err(fail(tri, None, "cavity cannot be tetrahedralized and has no interior kernel point (Steiner fallback failed)"));
        };
        if self.n_steiner + 1 > self.max_steiner {
            return Err(TetError::SteinerBudget { limit: self.max_steiner });
        }
        let id = self.dt.push_point(c);
        self.n_steiner += 1;
        Ok(faces.iter().map(|f| [id, f[0], f[1], f[2]]).collect())
    }
}

impl Dt {
    /// A point strictly on the inner side of every face of the polyhedron bounded by `faces`
    /// (outward orientation), found from the vertex centroid, the face centroid and an
    /// alternating-projection kernel search; `None` if the polyhedron is not star-shaped
    /// enough.
    pub(crate) fn kernel_point(&self, faces: &[[u32; 3]]) -> Option<V3> {
        let pts = &self.pts;
        let mut verts: Vec<u32> = faces.iter().flatten().copied().collect();
        verts.sort_unstable();
        verts.dedup();
        let mut cen = [0.0; 3];
        for &v in &verts {
            cen = add(cen, pts[v as usize]);
        }
        let cen = scale(cen, 1.0 / verts.len() as f64);
        let mut fc = [0.0; 3];
        for f in faces {
            fc = add(fc, scale(add(add(pts[f[0] as usize], pts[f[1] as usize]), pts[f[2] as usize]), 1.0 / 3.0));
        }
        let fcen = scale(fc, 1.0 / faces.len() as f64);
        let visible = |p: V3| faces.iter().all(|f| ori(pts[f[0] as usize], pts[f[1] as usize], pts[f[2] as usize], p) < 0);
        let h = {
            let mut m = 0.0;
            for f in faces {
                m += crate::geom::dist(pts[f[0] as usize], pts[f[1] as usize]);
            }
            m / faces.len() as f64
        };
        let mut p = cen;
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
                // The inner side is negative; keep at least 0.05 h inside.
                if d > -0.05 * h {
                    p = sub(p, scale(n, d + 0.1 * h));
                    moved = true;
                }
            }
            if !moved {
                break;
            }
        }
        [cen, fcen, p].into_iter().filter_map(sanitize).find(|&c| visible(c))
    }
}

#[allow(dead_code)]
fn _unused(_: V3) {
    let _ = pt([0.0; 3]);
    let _ = Pts(&[]);
}
