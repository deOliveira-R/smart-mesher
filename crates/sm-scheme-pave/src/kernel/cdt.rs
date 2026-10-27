//! Private constrained Delaunay triangulation: exact predicates (`sm-math`), SoA arrays, no
//! hash maps, fully deterministic. Duplicated in spirit with `sm-scheme-tri` (plan decision:
//! accepted for W3a, a shared L2 crate is a W3b candidate).
//!
//! Triangles are counter-clockwise. `tn[t][i]` is the neighbour across the edge *opposite*
//! vertex `i` (the edge `tv[t][i+1] -> tv[t][i+2]`); `tc[t][i]` flags that edge as constrained.

use core::ops::Range;
use sm_math::{Sign, incircle, orient2d};

/// "No triangle" / "no vertex".
pub(crate) const NONE: u32 = u32::MAX;

/// Failure of the triangulation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum CdtError {
    /// Two boundary points coincide.
    Duplicate,
    /// A constraint crosses another constraint or touches a foreign vertex.
    Crossing,
    /// A loop has the wrong winding (interior not on its left).
    Orientation(usize),
    /// An invariant broke (flip loop did not converge, ...).
    Internal(&'static str),
}

/// Result of a point location.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Loc {
    /// Strictly inside triangle `.0`.
    In(u32),
    /// On the edge opposite vertex `.1` of triangle `.0`.
    OnEdge(u32, u8),
    /// On an existing vertex.
    OnVertex(u32),
    /// Outside the triangulation (or behind a boundary edge).
    Outside,
}

/// Everything needed to undo one flip.
#[derive(Clone, Copy, Debug)]
pub(crate) struct FlipUndo {
    t: u32,
    u: u32,
    tv: [[u32; 3]; 2],
    tn: [[u32; 3]; 2],
    tc: [[bool; 3]; 2],
}

#[derive(Clone, Debug)]
pub(crate) struct Cdt {
    pub pts: Vec<[f64; 2]>,
    pub tv: Vec<[u32; 3]>,
    pub tn: Vec<[u32; 3]>,
    pub tc: Vec<[bool; 3]>,
    /// A live triangle incident to each vertex, or `NONE`.
    pub vt: Vec<u32>,
    free: Vec<u32>,
    live: usize,
}

#[inline]
fn pos(v: &[u32; 3], x: u32) -> usize {
    if v[0] == x {
        0
    } else if v[1] == x {
        1
    } else {
        2
    }
}

impl Cdt {
    /// Empty triangulation holding the points; no triangles yet.
    fn with_points(pts: Vec<[f64; 2]>) -> Self {
        let n = pts.len();
        Cdt { pts, tv: Vec::new(), tn: Vec::new(), tc: Vec::new(), vt: vec![NONE; n], free: Vec::new(), live: 0 }
    }

    pub fn is_live(&self, t: u32) -> bool {
        self.tv[t as usize][0] != NONE
    }

    /// Number of live triangles.
    #[cfg(test)]
    pub fn len(&self) -> usize {
        self.live
    }

    /// Live triangle ids, ascending.
    pub fn triangles(&self) -> impl Iterator<Item = u32> + '_ {
        (0..self.tv.len() as u32).filter(|t| self.is_live(*t))
    }

    fn alloc(&mut self, v: [u32; 3], n: [u32; 3], c: [bool; 3]) -> u32 {
        self.live += 1;
        if let Some(t) = self.free.pop() {
            self.tv[t as usize] = v;
            self.tn[t as usize] = n;
            self.tc[t as usize] = c;
            t
        } else {
            self.tv.push(v);
            self.tn.push(n);
            self.tc.push(c);
            (self.tv.len() - 1) as u32
        }
    }

    fn set_vt(&mut self, t: u32) {
        for v in self.tv[t as usize] {
            self.vt[v as usize] = t;
        }
    }

    /// Point to `new` from the triangle `n` across the edge `{x, y}`.
    fn relink(&mut self, n: u32, x: u32, y: u32, new: u32) {
        if n == NONE {
            return;
        }
        let v = self.tv[n as usize];
        for s in 0..3 {
            let (p, q) = (v[(s + 1) % 3], v[(s + 2) % 3]);
            if (p == x && q == y) || (p == y && q == x) {
                self.tn[n as usize][s] = new;
                return;
            }
        }
        debug_assert!(false, "relink: edge not found");
    }

    fn o2(&self, a: u32, b: u32, p: [f64; 2]) -> Sign {
        orient2d(self.pts[a as usize], self.pts[b as usize], p)
    }

    /// Mark edge `(t, i)` (and its mirror) as constrained.
    pub fn constrain(&mut self, t: u32, i: usize) {
        self.tc[t as usize][i] = true;
        let n = self.tn[t as usize][i];
        if n != NONE {
            let v = self.tv[t as usize];
            let (x, y) = (v[(i + 1) % 3], v[(i + 2) % 3]);
            let vn = self.tv[n as usize];
            for s in 0..3 {
                let (p, q) = (vn[(s + 1) % 3], vn[(s + 2) % 3]);
                if p == y && q == x {
                    self.tc[n as usize][s] = true;
                }
            }
        }
    }

    // ---------------------------------------------------------------- locate

    /// Locate `p` by a visibility walk from `hint` (any triangle id; a dead or `NONE` hint
    /// starts at the first live triangle). A walk blocked by a boundary edge answers
    /// `Outside`; callers that need certainty use [`Cdt::locate_scan`].
    pub fn locate(&self, p: [f64; 2], hint: u32) -> Loc {
        let mut t = if hint != NONE && (hint as usize) < self.tv.len() && self.is_live(hint) {
            hint
        } else {
            match self.triangles().next() {
                Some(t) => t,
                None => return Loc::Outside,
            }
        };
        let limit = 4 * self.tv.len() + 64;
        for step in 0..limit {
            let v = self.tv[t as usize];
            let mut blocked = false;
            let mut next = NONE;
            let mut zeros = [false; 3];
            for k in 0..3 {
                let i = (step + k) % 3;
                match self.o2(v[(i + 1) % 3], v[(i + 2) % 3], p) {
                    Sign::Negative => {
                        let n = self.tn[t as usize][i];
                        if n == NONE {
                            blocked = true;
                        } else if next == NONE {
                            next = n;
                        }
                    }
                    Sign::Zero => zeros[i] = true,
                    Sign::Positive => {}
                }
            }
            if next != NONE {
                t = next;
                continue;
            }
            if blocked {
                return Loc::Outside;
            }
            return match (zeros[0], zeros[1], zeros[2]) {
                (false, false, false) => Loc::In(t),
                (true, false, false) => Loc::OnEdge(t, 0),
                (false, true, false) => Loc::OnEdge(t, 1),
                (false, false, true) => Loc::OnEdge(t, 2),
                (false, true, true) => Loc::OnVertex(v[0]),
                (true, false, true) => Loc::OnVertex(v[1]),
                (true, true, false) => Loc::OnVertex(v[2]),
                (true, true, true) => Loc::Outside,
            };
        }
        self.locate_scan(p)
    }

    /// Exhaustive location (lowest triangle id wins on ties).
    pub fn locate_scan(&self, p: [f64; 2]) -> Loc {
        for t in self.triangles() {
            let v = self.tv[t as usize];
            let s: [Sign; 3] = core::array::from_fn(|i| self.o2(v[(i + 1) % 3], v[(i + 2) % 3], p));
            if s.contains(&Sign::Negative) {
                continue;
            }
            let z = s.iter().filter(|x| **x == Sign::Zero).count();
            return match z {
                0 => Loc::In(t),
                1 => Loc::OnEdge(t, s.iter().position(|x| *x == Sign::Zero).unwrap_or(0) as u8),
                2 => Loc::OnVertex(v[s.iter().position(|x| *x != Sign::Zero).unwrap_or(0)]),
                _ => Loc::Outside,
            };
        }
        Loc::Outside
    }

    // ---------------------------------------------------------------- edits

    /// Flip the edge opposite vertex `i` of `t`. Both triangles are reused: afterwards
    /// `t = (p, a, q)` and the returned `u = (p, q, b)` with `p` at index 0 of both.
    fn flip(&mut self, t: u32, i: usize) -> u32 {
        let tvv = self.tv[t as usize];
        let (p, a, b) = (tvv[i], tvv[(i + 1) % 3], tvv[(i + 2) % 3]);
        let u = self.tn[t as usize][i];
        let uv = self.tv[u as usize];
        let j = (0..3).find(|j| uv[(j + 1) % 3] == b && uv[(j + 2) % 3] == a).unwrap_or(0);
        let q = uv[j];
        let (n_bp, c_bp) = (self.tn[t as usize][(i + 1) % 3], self.tc[t as usize][(i + 1) % 3]);
        let (n_pa, c_pa) = (self.tn[t as usize][(i + 2) % 3], self.tc[t as usize][(i + 2) % 3]);
        let (n_aq, c_aq) = (self.tn[u as usize][(j + 1) % 3], self.tc[u as usize][(j + 1) % 3]);
        let (n_qb, c_qb) = (self.tn[u as usize][(j + 2) % 3], self.tc[u as usize][(j + 2) % 3]);
        self.tv[t as usize] = [p, a, q];
        self.tn[t as usize] = [n_aq, u, n_pa];
        self.tc[t as usize] = [c_aq, false, c_pa];
        self.tv[u as usize] = [p, q, b];
        self.tn[u as usize] = [n_qb, n_bp, t];
        self.tc[u as usize] = [c_qb, c_bp, false];
        self.relink(n_aq, a, q, t);
        self.relink(n_bp, b, p, u);
        self.set_vt(t);
        self.set_vt(u);
        u
    }

    /// Split triangle `t` with the new vertex `p` strictly inside; returns the three new
    /// triangles `(p, ., .)`, each with `p` at index 0 and the old edge at slot 0.
    fn split3(&mut self, t: u32, p: u32) -> [u32; 3] {
        let [a, b, c] = self.tv[t as usize];
        let [na, nb, nc] = self.tn[t as usize];
        let [ca, cb, cc] = self.tc[t as usize];
        let t1 = self.alloc([p, c, a], [nb, NONE, t], [cb, false, false]);
        let t2 = self.alloc([p, a, b], [nc, t1, NONE], [cc, false, false]);
        self.tv[t as usize] = [p, b, c];
        self.tn[t as usize] = [na, t1, t2];
        self.tc[t as usize] = [ca, false, false];
        self.tn[t1 as usize] = [nb, t2, t];
        self.tn[t2 as usize] = [nc, t, t1];
        self.relink(nb, c, a, t1);
        self.relink(nc, a, b, t2);
        self.set_vt(t);
        self.set_vt(t1);
        self.set_vt(t2);
        [t, t1, t2]
    }

    /// Split the (unconstrained, interior) edge opposite vertex `i` of `t` with the new vertex
    /// `p`; returns the four new triangles, each with `p` at index 2 or 1... (see below) and
    /// the edge opposite `p` at the slot returned by [`Cdt::slot_of`].
    fn split_edge(&mut self, t: u32, i: usize, p: u32) -> [u32; 4] {
        let tvv = self.tv[t as usize];
        let (c, a, b) = (tvv[i], tvv[(i + 1) % 3], tvv[(i + 2) % 3]);
        let u = self.tn[t as usize][i];
        let uv = self.tv[u as usize];
        let j = (0..3).find(|j| uv[(j + 1) % 3] == b && uv[(j + 2) % 3] == a).unwrap_or(0);
        let d = uv[j];
        let (n_bc, c_bc) = (self.tn[t as usize][(i + 1) % 3], self.tc[t as usize][(i + 1) % 3]);
        let (n_ca, c_ca) = (self.tn[t as usize][(i + 2) % 3], self.tc[t as usize][(i + 2) % 3]);
        let (n_ad, c_ad) = (self.tn[u as usize][(j + 1) % 3], self.tc[u as usize][(j + 1) % 3]);
        let (n_db, c_db) = (self.tn[u as usize][(j + 2) % 3], self.tc[u as usize][(j + 2) % 3]);
        // T_a = (c, a, p) reuses t; U_b = (d, b, p) reuses u.
        let t_b = self.alloc([c, p, b], [NONE, n_bc, NONE], [false, c_bc, false]);
        let u_a = self.alloc([d, p, a], [NONE, n_ad, NONE], [false, c_ad, false]);
        let (t_a, u_b) = (t, u);
        self.tv[t_a as usize] = [c, a, p];
        self.tn[t_a as usize] = [u_a, t_b, n_ca];
        self.tc[t_a as usize] = [false, false, c_ca];
        self.tv[u_b as usize] = [d, b, p];
        self.tn[u_b as usize] = [t_b, u_a, n_db];
        self.tc[u_b as usize] = [false, false, c_db];
        self.tn[t_b as usize] = [u_b, n_bc, t_a];
        self.tn[u_a as usize] = [t_a, n_ad, u_b];
        self.relink(n_bc, b, c, t_b);
        self.relink(n_ad, a, d, u_a);
        for x in [t_a, t_b, u_a, u_b] {
            self.set_vt(x);
        }
        [t_a, t_b, u_b, u_a]
    }

    /// Flip the edge opposite vertex `i` of `t` when it is interior, unconstrained and its
    /// quad strictly convex. Returns the other triangle `u` and the record that undoes the
    /// flip exactly (triangle ids included).
    pub fn flip_if_convex(&mut self, t: u32, i: usize) -> Option<(u32, FlipUndo)> {
        let n = self.tn[t as usize][i];
        if n == NONE || self.tc[t as usize][i] {
            return None;
        }
        let v = self.tv[t as usize];
        let (p, a, b) = (v[i], v[(i + 1) % 3], v[(i + 2) % 3]);
        let q = self.tv[n as usize].into_iter().find(|x| *x != a && *x != b)?;
        let ok = self.o2(p, a, self.pts[q as usize]) == Sign::Positive && self.o2(q, b, self.pts[p as usize]) == Sign::Positive;
        if !ok {
            return None;
        }
        let undo = FlipUndo {
            t,
            u: n,
            tv: [self.tv[t as usize], self.tv[n as usize]],
            tn: [self.tn[t as usize], self.tn[n as usize]],
            tc: [self.tc[t as usize], self.tc[n as usize]],
        };
        let u = self.flip(t, i);
        debug_assert_eq!(u, n);
        Some((u, undo))
    }

    /// Undo a flip made by [`Cdt::flip_if_convex`] (nothing else may have changed since).
    pub fn undo_flip(&mut self, f: FlipUndo) {
        for (k, x) in [f.t, f.u].into_iter().enumerate() {
            self.tv[x as usize] = f.tv[k];
            self.tn[x as usize] = f.tn[k];
            self.tc[x as usize] = f.tc[k];
        }
        for x in [f.t, f.u] {
            let v = self.tv[x as usize];
            for s in 0..3 {
                let n = self.tn[x as usize][s];
                if n != NONE && n != f.t && n != f.u {
                    self.relink(n, v[(s + 1) % 3], v[(s + 2) % 3], x);
                }
            }
            self.set_vt(x);
        }
    }

    /// Check the edge opposite vertex `i` of `t`; flip it if it is not locally Delaunay.
    /// Returns the two triangles (t, u) after the flip, `None` if nothing changed.
    fn legalize_edge(&mut self, t: u32, i: usize) -> Option<(u32, u32)> {
        let n = self.tn[t as usize][i];
        if n == NONE || self.tc[t as usize][i] {
            return None;
        }
        let v = self.tv[t as usize];
        let (a, b) = (v[(i + 1) % 3], v[(i + 2) % 3]);
        let nv = self.tv[n as usize];
        let q = (0..3).map(|j| nv[j]).find(|x| *x != a && *x != b)?;
        let inside = incircle(self.pts[v[0] as usize], self.pts[v[1] as usize], self.pts[v[2] as usize], self.pts[q as usize]);
        if inside != Sign::Positive {
            return None;
        }
        let u = self.flip(t, i);
        Some((t, u))
    }

    /// Delaunay-legalize after inserting a vertex: `stack` holds `(triangle, 0)` pairs whose
    /// slot-0 edge is opposite the new vertex (kept at index 0 by `flip`/`split3`).
    fn legalize_from(&mut self, stack: &mut Vec<u32>) {
        while let Some(t) = stack.pop() {
            if let Some((a, b)) = self.legalize_edge(t, 0) {
                stack.push(a);
                stack.push(b);
            }
        }
    }

    /// Insert point `p` at a located position. Fails on a vertex, a constrained edge or
    /// `Outside`. Returns the new vertex id.
    pub fn insert(&mut self, p: [f64; 2], loc: Loc) -> Result<u32, CdtError> {
        match loc {
            Loc::Outside | Loc::OnVertex(_) => Err(CdtError::Internal("insert at a vertex or outside")),
            Loc::In(t) => {
                let id = self.push_point(p);
                let tris = self.split3(t, id);
                let mut stack: Vec<u32> = tris.to_vec();
                self.legalize_from(&mut stack);
                Ok(id)
            }
            Loc::OnEdge(t, i) => {
                let i = i as usize;
                if self.tc[t as usize][i] || self.tn[t as usize][i] == NONE {
                    return Err(CdtError::Internal("insert on a constrained edge"));
                }
                let id = self.push_point(p);
                let tris = self.split_edge(t, i, id);
                // The edge opposite `p` in each: slot of the vertex `p`.
                let mut stack: Vec<u32> = Vec::new();
                for x in tris {
                    self.rotate_to_front(x, id);
                    stack.push(x);
                }
                self.legalize_from(&mut stack);
                Ok(id)
            }
        }
    }

    /// Rotate the vertex order of `t` (cyclically, keeping the orientation) so `v` is at
    /// index 0. Neighbour and constraint arrays rotate along.
    fn rotate_to_front(&mut self, t: u32, v: u32) {
        let k = pos(&self.tv[t as usize], v);
        if k == 0 {
            return;
        }
        let (tv, tn, tc) = (self.tv[t as usize], self.tn[t as usize], self.tc[t as usize]);
        self.tv[t as usize] = core::array::from_fn(|s| tv[(s + k) % 3]);
        self.tn[t as usize] = core::array::from_fn(|s| tn[(s + k) % 3]);
        self.tc[t as usize] = core::array::from_fn(|s| tc[(s + k) % 3]);
    }

    fn push_point(&mut self, p: [f64; 2]) -> u32 {
        self.pts.push(p);
        self.vt.push(NONE);
        (self.pts.len() - 1) as u32
    }

    // ---------------------------------------------------------------- queries

    /// A triangle containing the edge `{u, v}` and the slot of that edge in it.
    pub fn tri_with_edge(&self, u: u32, v: u32) -> Option<(u32, usize)> {
        let start = self.vt[u as usize];
        if start == NONE {
            return None;
        }
        let find = |t: u32| -> Option<(u32, usize)> {
            let tv = self.tv[t as usize];
            let k = pos(&tv, u);
            if tv[(k + 1) % 3] == v {
                Some((t, (k + 2) % 3))
            } else if tv[(k + 2) % 3] == v {
                Some((t, (k + 1) % 3))
            } else {
                None
            }
        };
        let mut t = start;
        for _ in 0..=self.tv.len() {
            if let Some(r) = find(t) {
                return Some(r);
            }
            let k = pos(&self.tv[t as usize], u);
            let n = self.tn[t as usize][(k + 1) % 3];
            if n == NONE {
                break;
            }
            t = n;
            if t == start {
                return None;
            }
        }
        // Hit the hull going CCW: go clockwise from the start.
        let mut t = start;
        for _ in 0..=self.tv.len() {
            let k = pos(&self.tv[t as usize], u);
            let n = self.tn[t as usize][(k + 2) % 3];
            if n == NONE {
                return None;
            }
            t = n;
            if let Some(r) = find(t) {
                return Some(r);
            }
        }
        None
    }

    /// The triangle that has the directed edge `a -> b` (the one on its left).
    pub fn left_of(&self, a: u32, b: u32) -> Option<u32> {
        let (t, s) = self.tri_with_edge(a, b)?;
        let tv = self.tv[t as usize];
        if tv[(s + 1) % 3] == a {
            Some(t)
        } else {
            let n = self.tn[t as usize][s];
            (n != NONE).then_some(n)
        }
    }

    // ---------------------------------------------------------------- build

    /// Constrained Delaunay triangulation of the polygon region bounded by `loops` (index
    /// ranges into `pts`; outer counter-clockwise, holes clockwise; the interior is on the
    /// left of every segment). Points keep their ids `0..pts.len()`.
    pub fn build(pts: &[[f64; 2]], loops: &[Range<usize>]) -> Result<Cdt, CdtError> {
        let nb = pts.len();
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for p in pts {
            for k in 0..2 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        let d = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(f64::MIN_POSITIVE);
        let (cx, cy) = (0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]));
        let s = 8.0 * d;
        let mut all = pts.to_vec();
        all.push([cx - 2.0 * s, cy - s]);
        all.push([cx + 2.0 * s, cy - s]);
        all.push([cx, cy + 2.0 * s]);
        let mut c = Cdt::with_points(all);
        let t0 = c.alloc([nb as u32, nb as u32 + 1, nb as u32 + 2], [NONE; 3], [false; 3]);
        c.set_vt(t0);
        let mut hint = t0;
        for (i, p) in pts.iter().enumerate() {
            let loc = c.locate(*p, hint);
            let loc = match loc {
                Loc::Outside => c.locate_scan(*p),
                l => l,
            };
            match loc {
                Loc::OnVertex(_) => return Err(CdtError::Duplicate),
                Loc::Outside => return Err(CdtError::Internal("point outside the super triangle")),
                l => {
                    let id = c.insert_with_id(i as u32, l)?;
                    hint = c.vt[id as usize];
                }
            }
        }
        for r in loops {
            let n = r.len();
            for k in 0..n {
                let a = (r.start + k) as u32;
                let b = (r.start + (k + 1) % n) as u32;
                c.insert_constraint(a, b)?;
            }
        }
        c.strip(nb, loops)?;
        c.restore_delaunay();
        Ok(c)
    }

    /// Insert the already stored point `id` at `loc` (used while building: boundary points
    /// keep their ids, the super vertices sit after them).
    fn insert_with_id(&mut self, id: u32, loc: Loc) -> Result<u32, CdtError> {
        match loc {
            Loc::In(t) => {
                let tris = self.split3(t, id);
                let mut stack: Vec<u32> = tris.to_vec();
                self.legalize_from(&mut stack);
                Ok(id)
            }
            Loc::OnEdge(t, i) => {
                let i = i as usize;
                if self.tn[t as usize][i] == NONE {
                    return Err(CdtError::Internal("point on the super triangle hull"));
                }
                let tris = self.split_edge(t, i, id);
                let mut stack: Vec<u32> = Vec::new();
                for x in tris {
                    self.rotate_to_front(x, id);
                    stack.push(x);
                }
                self.legalize_from(&mut stack);
                Ok(id)
            }
            _ => Err(CdtError::Internal("bad location")),
        }
    }

    /// Does the open segment `(x, y)` properly cross the open segment `(a, b)`? (Zero
    /// orientations count as "no".)
    fn crosses(&self, a: u32, b: u32, x: u32, y: u32) -> bool {
        let (pa, pb) = (self.pts[a as usize], self.pts[b as usize]);
        let (px, py) = (self.pts[x as usize], self.pts[y as usize]);
        let s1 = orient2d(pa, pb, px);
        let s2 = orient2d(pa, pb, py);
        let s3 = orient2d(px, py, pa);
        let s4 = orient2d(px, py, pb);
        s1 != Sign::Zero && s2 != Sign::Zero && s1 != s2 && s3 != Sign::Zero && s4 != Sign::Zero && s3 != s4
    }

    /// Make `a -> b` an edge of the triangulation and flag it constrained (Sloan's swap
    /// recovery).
    fn insert_constraint(&mut self, a: u32, b: u32) -> Result<(), CdtError> {
        if let Some((t, s)) = self.tri_with_edge(a, b) {
            self.constrain(t, s);
            return Ok(());
        }
        let (pa, pb) = (self.pts[a as usize], self.pts[b as usize]);
        // First crossed edge: the fan triangle of `a` whose wedge contains `b`.
        let mut crossed: Vec<(u32, u32)> = Vec::new();
        let start = self.vt[a as usize];
        if start == NONE {
            return Err(CdtError::Internal("vertex without triangle"));
        }
        // Collect the whole fan of `a` (both directions).
        let mut fan: Vec<u32> = vec![start];
        let mut t = start;
        for _ in 0..=self.tv.len() {
            let k = pos(&self.tv[t as usize], a);
            let n = self.tn[t as usize][(k + 1) % 3];
            if n == NONE || n == start {
                break;
            }
            fan.push(n);
            t = n;
        }
        let mut t = start;
        for _ in 0..=self.tv.len() {
            let k = pos(&self.tv[t as usize], a);
            let n = self.tn[t as usize][(k + 2) % 3];
            if n == NONE || n == start || fan.contains(&n) {
                break;
            }
            fan.push(n);
            t = n;
        }
        let mut first: Option<(u32, u32, u32)> = None; // (triangle, l, r)
        for &t in &fan {
            let tv = self.tv[t as usize];
            let k = pos(&tv, a);
            let (x, y) = (tv[(k + 1) % 3], tv[(k + 2) % 3]);
            let sx = orient2d(pa, self.pts[x as usize], pb);
            let sy = orient2d(pa, self.pts[y as usize], pb);
            if sx == Sign::Zero && self.between(a, b, x) || sy == Sign::Zero && self.between(a, b, y) {
                return Err(CdtError::Crossing);
            }
            // b strictly inside the wedge (x ccw of a, y further ccw): left of a->x, right of a->y.
            if sx == Sign::Positive && sy == Sign::Negative {
                first = Some((t, y, x)); // y is left of a->b? y is ccw of b => left; x is right.
            }
        }
        let (t0, mut l, mut r) = first.ok_or(CdtError::Crossing)?;
        crossed.push((l, r));
        let mut cur = t0;
        let mut cur_edge = (l, r);
        for _ in 0..=self.tv.len() {
            // The triangle across edge (l, r) away from `cur`.
            let (t1, s1) = self.tri_with_edge(cur_edge.0, cur_edge.1).ok_or(CdtError::Internal("edge lost"))?;
            let nxt = if t1 == cur { self.tn[t1 as usize][s1] } else { t1 };
            if nxt == NONE {
                return Err(CdtError::Crossing);
            }
            let tv = self.tv[nxt as usize];
            let w = tv.into_iter().find(|x| *x != l && *x != r).ok_or(CdtError::Internal("degenerate triangle"))?;
            if w == b {
                break;
            }
            match orient2d(pa, pb, self.pts[w as usize]) {
                Sign::Zero => return Err(CdtError::Crossing),
                Sign::Positive => {
                    l = w;
                }
                Sign::Negative => {
                    r = w;
                }
            }
            crossed.push((l, r));
            cur = nxt;
            cur_edge = (l, r);
        }
        // Flip the crossing edges away.
        let mut queue: std::collections::VecDeque<(u32, u32)> = crossed.into_iter().collect();
        let cap = 64 + 16 * queue.len() * queue.len();
        let mut iters = 0usize;
        while let Some((x, y)) = queue.pop_front() {
            iters += 1;
            if iters > cap {
                return Err(CdtError::Internal("edge recovery did not converge"));
            }
            let (t, s) = self.tri_with_edge(x, y).ok_or(CdtError::Internal("crossing edge vanished"))?;
            if self.tc[t as usize][s] {
                return Err(CdtError::Crossing);
            }
            let tv = self.tv[t as usize];
            let (p, e1, e2) = (tv[s], tv[(s + 1) % 3], tv[(s + 2) % 3]);
            let n = self.tn[t as usize][s];
            if n == NONE {
                return Err(CdtError::Crossing);
            }
            let q = self.tv[n as usize].into_iter().find(|v| *v != e1 && *v != e2).ok_or(CdtError::Internal("apex"))?;
            let convex = self.o2(p, e1, self.pts[q as usize]) == Sign::Positive
                && self.o2(q, e2, self.pts[p as usize]) == Sign::Positive;
            if !convex {
                queue.push_back((x, y));
                continue;
            }
            self.flip(t, s);
            if self.crosses(a, b, p, q) {
                queue.push_back((p, q));
            }
        }
        let (t, s) = self.tri_with_edge(a, b).ok_or(CdtError::Internal("constraint not recovered"))?;
        self.constrain(t, s);
        Ok(())
    }

    /// Is `x` strictly between `a` and `b`, given that the three are collinear?
    fn between(&self, a: u32, b: u32, x: u32) -> bool {
        let (pa, pb, px) = (self.pts[a as usize], self.pts[b as usize], self.pts[x as usize]);
        let d = [pb[0] - pa[0], pb[1] - pa[1]];
        let e = [px[0] - pa[0], px[1] - pa[1]];
        let dot = d[0] * e[0] + d[1] * e[1];
        dot > 0.0 && dot < d[0] * d[0] + d[1] * d[1]
    }

    /// Classify triangles by constraint-crossing parity from the super triangle, drop the
    /// outside ones and the super vertices, and check every loop has its interior on the left.
    fn strip(&mut self, nb: usize, loops: &[Range<usize>]) -> Result<(), CdtError> {
        let nt = self.tv.len();
        let mut label = vec![-1i8; nt];
        let mut queue: std::collections::VecDeque<u32> = std::collections::VecDeque::new();
        for t in self.triangles() {
            if self.tv[t as usize].iter().any(|v| *v as usize >= nb) {
                label[t as usize] = 0;
                queue.push_back(t);
            }
        }
        while let Some(t) = queue.pop_front() {
            for i in 0..3 {
                let n = self.tn[t as usize][i];
                if n == NONE {
                    continue;
                }
                let l = label[t as usize] ^ i8::from(self.tc[t as usize][i]);
                if label[n as usize] < 0 {
                    label[n as usize] = l;
                    queue.push_back(n);
                } else if label[n as usize] != l {
                    return Err(CdtError::Crossing);
                }
            }
        }
        for (ring, r) in loops.iter().enumerate() {
            let n = r.len();
            for k in 0..n {
                let (a, b) = ((r.start + k) as u32, (r.start + (k + 1) % n) as u32);
                let t = self.left_of(a, b).ok_or(CdtError::Internal("boundary edge lost"))?;
                if label[t as usize] != 1 {
                    return Err(CdtError::Orientation(ring));
                }
            }
        }
        // Kill outside triangles, compact.
        let mut map = vec![NONE; nt];
        let mut next = 0u32;
        for t in 0..nt as u32 {
            if self.is_live(t) && label[t as usize] == 1 {
                map[t as usize] = next;
                next += 1;
            }
        }
        let (mut tv, mut tn, mut tc) = (Vec::new(), Vec::new(), Vec::new());
        for t in 0..nt {
            if map[t] == NONE {
                continue;
            }
            tv.push(self.tv[t]);
            tn.push(self.tn[t].map(|n| if n == NONE { NONE } else { map[n as usize] }));
            tc.push(self.tc[t]);
        }
        self.tv = tv;
        self.tn = tn;
        self.tc = tc;
        self.free.clear();
        self.live = self.tv.len();
        self.pts.truncate(nb);
        self.vt = vec![NONE; nb];
        for t in 0..self.tv.len() as u32 {
            self.set_vt(t);
        }
        // Every edge without a neighbour must be a constraint (the region is bounded by loops).
        for t in 0..self.tv.len() {
            for i in 0..3 {
                if self.tn[t][i] == NONE && !self.tc[t][i] {
                    return Err(CdtError::Internal("unconstrained boundary edge"));
                }
            }
        }
        Ok(())
    }

    /// Lawson flips over every interior unconstrained edge until all are locally Delaunay.
    fn restore_delaunay(&mut self) {
        let mut stack: Vec<(u32, u8)> = Vec::new();
        for t in (0..self.tv.len() as u32).rev() {
            for i in (0..3u8).rev() {
                stack.push((t, i));
            }
        }
        while let Some((t, i)) = stack.pop() {
            if !self.is_live(t) {
                continue;
            }
            if let Some((a, b)) = self.legalize_edge(t, i as usize) {
                // Outer edges of the flipped quad: (a,0)=(a,q) (a,2)=(p,a) (b,0)=(q,b) (b,1)=(b,p).
                stack.push((a, 0));
                stack.push((a, 2));
                stack.push((b, 0));
                stack.push((b, 1));
            }
        }
    }

    // ---------------------------------------------------------------- geometry helpers

    /// Triangles whose circumcircle strictly contains `p`, reachable from the seed triangles
    /// across unconstrained edges (the Delaunay cavity), plus the seeds themselves.
    pub fn cavity(&self, p: [f64; 2], seeds: &[u32]) -> Vec<u32> {
        let mut out: Vec<u32> = seeds.to_vec();
        let mut k = 0;
        while k < out.len() {
            let t = out[k];
            k += 1;
            for i in 0..3 {
                let n = self.tn[t as usize][i];
                if n == NONE || self.tc[t as usize][i] || out.contains(&n) {
                    continue;
                }
                let v = self.tv[n as usize];
                if incircle(self.pts[v[0] as usize], self.pts[v[1] as usize], self.pts[v[2] as usize], p) == Sign::Positive {
                    out.push(n);
                }
            }
        }
        out
    }

    /// Seeds for [`Cdt::cavity`] from a location.
    pub fn seeds(&self, loc: Loc) -> Vec<u32> {
        match loc {
            Loc::In(t) => vec![t],
            Loc::OnEdge(t, i) => {
                let n = self.tn[t as usize][i as usize];
                if n == NONE { vec![t] } else { vec![t, n] }
            }
            _ => Vec::new(),
        }
    }

    /// Structural self-check (tests): adjacency symmetric, triangles strictly CCW, constraint
    /// flags mirrored, constrained or hull edges are the only unmatched ones, and every
    /// unconstrained interior edge is locally Delaunay.
    #[cfg(test)]
    pub fn validate(&self) -> Result<(), String> {
        for t in self.triangles() {
            let v = self.tv[t as usize];
            if self.o2(v[0], v[1], self.pts[v[2] as usize]) != Sign::Positive {
                return Err(format!("triangle {t} {v:?} not strictly ccw"));
            }
            for i in 0..3 {
                let n = self.tn[t as usize][i];
                if n == NONE {
                    continue;
                }
                let (x, y) = (v[(i + 1) % 3], v[(i + 2) % 3]);
                let nv = self.tv[n as usize];
                let j = (0..3).find(|j| nv[(j + 1) % 3] == y && nv[(j + 2) % 3] == x).ok_or(format!("tri {t} edge {i}: neighbour {n} does not share it"))?;
                if self.tn[n as usize][j] != t {
                    return Err(format!("asymmetric adjacency {t}/{n}"));
                }
                if self.tc[n as usize][j] != self.tc[t as usize][i] {
                    return Err(format!("constraint flag mismatch {t}/{n}"));
                }
                if !self.tc[t as usize][i] {
                    let q = nv[j];
                    if incircle(self.pts[v[0] as usize], self.pts[v[1] as usize], self.pts[v[2] as usize], self.pts[q as usize]) == Sign::Positive {
                        return Err(format!("edge {t}/{n} not Delaunay"));
                    }
                }
            }
        }
        Ok(())
    }
}

/// Signed area of a polygon (shoelace), positive counter-clockwise.
pub(crate) fn signed_area(p: &[[f64; 2]]) -> f64 {
    let mut s = 0.0;
    for i in 0..p.len() {
        let (a, b) = (p[i], p[(i + 1) % p.len()]);
        s += a[0] * b[1] - a[1] * b[0];
    }
    0.5 * s
}

#[cfg(test)]
#[allow(clippy::single_range_in_vec_init)]
mod tests {
    use super::*;

    fn square_loop(n: usize) -> Vec<[f64; 2]> {
        let mut v = Vec::new();
        for i in 0..n {
            v.push([i as f64 / n as f64, 0.0]);
        }
        for i in 0..n {
            v.push([1.0, i as f64 / n as f64]);
        }
        for i in 0..n {
            v.push([1.0 - i as f64 / n as f64, 1.0]);
        }
        for i in 0..n {
            v.push([0.0, 1.0 - i as f64 / n as f64]);
        }
        v
    }

    #[test]
    fn square_boundary_triangulates_with_collinear_nodes() {
        let p = square_loop(8);
        let c = Cdt::build(&p, &[0..p.len()]).expect("cdt");
        c.validate().expect("valid");
        // T = B - 2 for a simply connected polygon without interior points.
        assert_eq!(c.len(), p.len() - 2);
        let area: f64 = c.triangles().map(|t| {
            let v = c.tv[t as usize];
            signed_area(&[c.pts[v[0] as usize], c.pts[v[1] as usize], c.pts[v[2] as usize]])
        }).sum();
        assert!((area - 1.0).abs() < 1e-14);
    }

    #[test]
    fn l_shape_with_a_hole_classifies_by_parity() {
        let mut p: Vec<[f64; 2]> = vec![[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]];
        // clockwise hole
        p.extend([[1.0, 1.0], [1.0, 3.0], [3.0, 3.0], [3.0, 1.0]]);
        let c = Cdt::build(&p, &[0..4, 4..8]).expect("cdt");
        c.validate().expect("valid");
        let area: f64 = c.triangles().map(|t| {
            let v = c.tv[t as usize];
            signed_area(&[c.pts[v[0] as usize], c.pts[v[1] as usize], c.pts[v[2] as usize]])
        }).sum();
        assert!((area - 12.0).abs() < 1e-13, "{area}");
    }

    #[test]
    fn wrong_hole_winding_and_bowties_are_errors() {
        let mut p: Vec<[f64; 2]> = vec![[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]];
        p.extend([[1.0, 1.0], [3.0, 1.0], [3.0, 3.0], [1.0, 3.0]]); // ccw hole
        assert_eq!(Cdt::build(&p, &[0..4, 4..8]).err(), Some(CdtError::Orientation(1)));
        let bow = vec![[0.0, 0.0], [2.0, 2.0], [2.0, 0.0], [0.0, 2.0]];
        assert!(Cdt::build(&bow, &[0..4]).is_err());
    }

    #[test]
    fn interior_insertion_keeps_the_triangulation_delaunay() {
        let p = square_loop(4);
        let mut c = Cdt::build(&p, &[0..p.len()]).expect("cdt");
        let mut hint = NONE;
        for (k, q) in [[0.5, 0.5], [0.25, 0.25], [0.75, 0.3], [0.5, 0.75], [0.3, 0.6]].into_iter().enumerate() {
            let loc = c.locate(q, hint);
            let id = c.insert(q, loc).expect("insert");
            assert_eq!(id as usize, p.len() + k);
            hint = c.vt[id as usize];
            c.validate().expect("valid");
        }
        // 16 boundary + 5 interior: T = B + 2I - 2
        assert_eq!(c.len(), 16 + 10 - 2);
    }
}
