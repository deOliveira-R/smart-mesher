//! Construction: Hilbert-ordered Bowyer-Watson insertion with ghost triangles, plus the cavity
//! machinery that refinement reuses (with constrained edges as walls).

use super::{
    C_NONE, Cdt, DEAD, GHOST, NONE, Pslg, SegmentKind, TriError,
};
use sm_math::Sign;
use sm_math::predicates::{in_exact_range, incircle};
use std::collections::BTreeMap;

/// One edge of a cavity boundary: `a -> b` is counter-clockwise in the cavity triangle.
#[derive(Clone, Copy, Debug)]
pub(super) struct Bnd {
    pub a: u32,
    pub b: u32,
    pub out_t: u32,
    pub out_i: u8,
    pub code: u8,
}

/// Reusable buffers (no per-insertion allocation).
#[derive(Clone, Debug, Default)]
pub(super) struct Scratch {
    pub mark: Vec<u32>,
    pub epoch: u32,
    pub cav: Vec<u32>,
    pub stack: Vec<u32>,
    pub bnd: Vec<Bnd>,
    pub start_slot: Vec<u32>,
    pub end_slot: Vec<u32>,
    pub newt: Vec<u32>,
}

/// Why a cavity cannot be used for an insertion.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Reject {
    /// The point would encroach a constrained segment of the cavity boundary.
    Encroach,
    /// The cavity is not star-shaped around the point (the point is not visible from a
    /// boundary edge).
    NotStar,
}

/// Hilbert index of `(x, y)` on a `2^order` grid.
fn hilbert_index(mut x: u64, mut y: u64, order: u32) -> u128 {
    let n: u64 = 1 << order;
    let mut d: u128 = 0;
    let mut s: u64 = n >> 1;
    while s > 0 {
        let rx = u64::from(x & s != 0);
        let ry = u64::from(y & s != 0);
        d += u128::from(s) * u128::from(s) * u128::from((3 * rx) ^ ry);
        if ry == 0 {
            if rx == 1 {
                x = n - 1 - x;
                y = n - 1 - y;
            }
            std::mem::swap(&mut x, &mut y);
        }
        s >>= 1;
    }
    d
}

/// Point ids sorted along the Hilbert curve of the coordinates quantized on a fixed `2^-40` grid
/// of the (power-of-two enlarged) bounding box; ties by id. Depends only on the coordinates.
pub(super) fn hilbert_order(pts: &[[f64; 2]]) -> Vec<u32> {
    const BITS: u32 = 40;
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for p in pts {
        for k in 0..2 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let ext = (hi[0] - lo[0]).max(hi[1] - lo[1]);
    let scale = if ext > 0.0 && ext.is_finite() {
        crate::util::pow2_ceil(ext)
    } else {
        1.0
    };
    let grid = (1u64 << BITS) as f64;
    let max = (1u64 << BITS) - 1;
    let q = |v: f64, l: f64| -> u64 { (((v - l) / scale * grid).floor() as u64).min(max) };
    let mut keyed: Vec<(u128, u32)> = pts
        .iter()
        .enumerate()
        .map(|(i, p)| (hilbert_index(q(p[0], lo[0]), q(p[1], lo[1]), BITS), i as u32))
        .collect();
    keyed.sort_unstable();
    keyed.into_iter().map(|(_, i)| i).collect()
}

/// True if collinear `p` lies strictly between `a` and `b`.
pub(super) fn strictly_between(a: [f64; 2], b: [f64; 2], p: [f64; 2]) -> bool {
    let k = usize::from((a[0] - b[0]).abs() < (a[1] - b[1]).abs());
    (a[k] < p[k] && p[k] < b[k]) || (b[k] < p[k] && p[k] < a[k])
}

impl Cdt {
    pub(super) fn empty(pts: Vec<[f64; 2]>) -> Cdt {
        let n = pts.len();
        Cdt {
            n_input: n,
            pts,
            pts3: Vec::new(),
            tri: Vec::new(),
            nbr: Vec::new(),
            code: Vec::new(),
            tid: Vec::new(),
            vtri: vec![NONE; n],
            free: Vec::new(),
            next_tid: 0,
            hint: 0,
            ghosts: true,
            seg_of: BTreeMap::new(),
            scratch: Scratch::default(),
        }
    }

    /// Triangulate `input`: all points, then the segments, then the inside by parity.
    ///
    /// # Errors
    /// [`TriError::OutOfRange`] for a coordinate outside `[2^-100, 2^100]` (or non-finite),
    /// [`TriError::DuplicatePoint`], [`TriError::SelfIntersection`] for crossing segments,
    /// [`TriError::BadInput`] for malformed segments, [`TriError::Degenerate`] for fewer than
    /// three non-collinear points and [`TriError::Inconsistent`] for an open boundary chain.
    pub fn new(input: &Pslg) -> Result<Cdt, TriError> {
        let n = input.points.len();
        if n >= (u32::MAX / 2) as usize {
            return Err(TriError::BadInput("too many points".into()));
        }
        for (i, p) in input.points.iter().enumerate() {
            if !(p[0].is_finite() && p[1].is_finite() && in_exact_range(p[0]) && in_exact_range(p[1]))
            {
                return Err(TriError::OutOfRange { point: i as u32 });
            }
        }
        for (i, s) in input.segments.iter().enumerate() {
            if s.a as usize >= n || s.b as usize >= n || s.a == s.b {
                return Err(TriError::BadInput(format!(
                    "segment {i} ({}, {}) is degenerate or out of range",
                    s.a, s.b
                )));
            }
        }
        if n < 3 {
            return Err(TriError::Degenerate("fewer than three points".into()));
        }
        let order = hilbert_order(&input.points);
        let mut rank = vec![0u32; n];
        for (r, &v) in order.iter().enumerate() {
            rank[v as usize] = r as u32;
        }
        let mut c = Cdt::empty(input.points.clone());
        let (p0, p1) = (order[0], order[1]);
        if c.p(p0) == c.p(p1) {
            return Err(TriError::DuplicatePoint {
                a: p0.min(p1),
                b: p0.max(p1),
            });
        }
        let mut k = 2;
        while k < n && c.orient(p0, p1, c.p(order[k])) == Sign::Zero {
            k += 1;
        }
        if k == n {
            return Err(TriError::Degenerate(
                "all points are collinear".into(),
            ));
        }
        let pk = order[k];
        c.seed(p0, p1, pk);
        for (r, &v) in order.iter().enumerate() {
            if r < 2 || r == k {
                continue;
            }
            c.insert_site(v)?;
        }
        c.recover_all(input, &rank)?;
        c.flood_fill(input.segments.iter().any(|s| s.kind == SegmentKind::Boundary))?;
        Ok(c)
    }

    /// The first triangle and its three ghosts.
    fn seed(&mut self, a: u32, b: u32, c: u32) {
        let (a, b) = if self.orient(a, b, self.p(c)) == Sign::Positive {
            (a, b)
        } else {
            (b, a)
        };
        let t = self.alloc_tri([a, b, c], [NONE; 3], [C_NONE; 3]);
        // Ghost of the real edge x -> y is (y, x, GHOST); real edge i is opposite vertex i.
        let verts = [a, b, c];
        let mut g = [0u32; 3];
        for i in 0..3 {
            let (x, y) = (verts[(i + 1) % 3], verts[(i + 2) % 3]);
            g[i] = self.alloc_tri([y, x, GHOST], [NONE; 3], [C_NONE; 3]);
            self.nbr[g[i] as usize][2] = t;
            self.nbr[t as usize][i] = g[i];
        }
        // Ghost of edge i = (x -> y) = (v[i+1] -> v[i+2]): its edge opposite idx0 (=y) is (x, G),
        // shared with the ghost of the edge ending at x, which is edge i+2 = (v[i+2]... ) i.e. the
        // real edge (w -> x) with w = v[i+2]; opposite idx1 (=x) is (G, y), shared with the ghost
        // of the real edge (y -> z), z = v[i+1]... i.e. edge i+1.
        for i in 0..3 {
            self.nbr[g[i] as usize][0] = g[(i + 2) % 3];
            self.nbr[g[i] as usize][1] = g[(i + 1) % 3];
        }
        for v in [a, b, c] {
            self.vtri[v as usize] = t;
        }
        self.hint = t;
    }

    pub(super) fn alloc_tri(&mut self, v: [u32; 3], n: [u32; 3], c: [u8; 3]) -> u32 {
        let id = self.next_tid;
        self.next_tid += 1;
        if let Some(t) = self.free.pop() {
            let i = t as usize;
            self.tri[i] = v;
            self.nbr[i] = n;
            self.code[i] = c;
            self.tid[i] = id;
            t
        } else {
            self.tri.push(v);
            self.nbr.push(n);
            self.code.push(c);
            self.tid.push(id);
            self.scratch.mark.push(0);
            (self.tri.len() - 1) as u32
        }
    }

    pub(super) fn kill_tri(&mut self, t: u32) {
        let i = t as usize;
        self.tri[i] = [DEAD; 3];
        self.nbr[i] = [NONE; 3];
        self.free.push(t);
    }

    /// Does ghost `(x, y, GHOST)` "contain" `p` (strictly outside the hull edge, or inside it)?
    fn ghost_covers(&self, x: u32, y: u32, p: [f64; 2]) -> bool {
        match self.orient(x, y, p) {
            Sign::Positive => true,
            Sign::Zero => strictly_between(self.p(x), self.p(y), p),
            Sign::Negative => false,
        }
    }

    /// Is `p` strictly inside the (generalized) circumcircle of triangle `t`?
    pub(super) fn in_circle(&self, t: u32, p: [f64; 2]) -> bool {
        let v = self.tri[t as usize];
        if v[2] == GHOST {
            self.ghost_covers(v[0], v[1], p)
        } else {
            incircle(self.p(v[0]), self.p(v[1]), self.p(v[2]), p) == Sign::Positive
        }
    }

    /// A triangle containing `p` (closed), or a ghost whose hull edge `p` lies beyond. With
    /// `walls`, returns `None` when the straight walk would cross a constrained edge or leave
    /// the mesh.
    pub(super) fn locate(&self, p: [f64; 2], start: u32, walls: bool) -> Option<u32> {
        let mut t = start;
        let cap = 4 * self.tri.len() + 64;
        for step in 0..cap {
            if !self.alive(t) {
                break;
            }
            let v = self.tri[t as usize];
            if v[2] == GHOST {
                if self.ghost_covers(v[0], v[1], p) {
                    return Some(t);
                }
                t = self.nbr[t as usize][2];
                continue;
            }
            let mut moved = false;
            for kk in 0..3 {
                let k = (kk + step) % 3;
                let (a, b) = (v[(k + 1) % 3], v[(k + 2) % 3]);
                if self.orient(a, b, p) == Sign::Negative {
                    let n = self.nbr[t as usize][k];
                    if walls && (n == NONE || self.code[t as usize][k] != C_NONE) {
                        return None;
                    }
                    t = n;
                    moved = true;
                    break;
                }
            }
            if !moved {
                return Some(t);
            }
            if t == NONE {
                return None;
            }
        }
        // Fallback (cycling or a dead start): scan all triangles in slot order.
        self.locate_scan(p)
    }

    pub(super) fn locate_scan(&self, p: [f64; 2]) -> Option<u32> {
        for t in 0..self.tri.len() as u32 {
            if !self.alive(t) {
                continue;
            }
            let v = self.tri[t as usize];
            if v[2] == GHOST {
                if self.ghost_covers(v[0], v[1], p) {
                    return Some(t);
                }
            } else if (0..3).all(|k| {
                self.orient(v[(k + 1) % 3], v[(k + 2) % 3], p) != Sign::Negative
            }) {
                return Some(t);
            }
        }
        None
    }

    /// Grow the cavity of `p` from `t0`: triangles whose circumcircle contains `p`, connected
    /// through unconstrained edges (with `walls`, constrained edges and the region boundary stop
    /// the growth).
    pub(super) fn build_cavity(&mut self, p: [f64; 2], t0: u32, walls: bool) {
        self.scratch.epoch += 1;
        let epoch = self.scratch.epoch;
        let mut cav = std::mem::take(&mut self.scratch.cav);
        let mut stack = std::mem::take(&mut self.scratch.stack);
        let mut bnd = std::mem::take(&mut self.scratch.bnd);
        cav.clear();
        stack.clear();
        bnd.clear();
        self.scratch.mark[t0 as usize] = epoch;
        cav.push(t0);
        stack.push(t0);
        while let Some(t) = stack.pop() {
            for i in 0..3 {
                let n = self.nbr[t as usize][i];
                let c = self.code[t as usize][i];
                let wall = n == NONE || (walls && c != C_NONE);
                if !wall && self.scratch.mark[n as usize] == epoch {
                    continue;
                }
                if !wall && self.in_circle(n, p) {
                    self.scratch.mark[n as usize] = epoch;
                    cav.push(n);
                    stack.push(n);
                    continue;
                }
                let tv = self.tri[t as usize];
                let out_i = if n == NONE {
                    0
                } else {
                    let nn = self.nbr[n as usize];
                    if nn[0] == t {
                        0
                    } else if nn[1] == t {
                        1
                    } else {
                        2
                    }
                };
                bnd.push(Bnd {
                    a: tv[(i + 1) % 3],
                    b: tv[(i + 2) % 3],
                    out_t: n,
                    out_i,
                    code: c,
                });
            }
        }
        self.scratch.cav = cav;
        self.scratch.stack = stack;
        self.scratch.bnd = bnd;
    }

    /// Check the cavity in `scratch` for `p`: star-shaped (strictly positive new triangles)
    /// and, with `walls`, not encroaching any constrained boundary edge.
    pub(super) fn check_cavity(&self, p: [f64; 2], walls: bool) -> Result<(), Reject> {
        for b in &self.scratch.bnd {
            if b.a == GHOST || b.b == GHOST {
                continue;
            }
            if self.orient(b.a, b.b, p) != Sign::Positive {
                return Err(Reject::NotStar);
            }
            if walls && b.code != C_NONE {
                let (pa, pb) = (self.p(b.a), self.p(b.b));
                let (ax, ay, bx, by) = (pa[0] - p[0], pa[1] - p[1], pb[0] - p[0], pb[1] - p[1]);
                let dot = ax * bx + ay * by;
                // Points on the diametral circle (dot = 0 up to rounding) do not encroach.
                if dot < -1e-9 * ((ax * ax + ay * ay) * (bx * bx + by * by)).sqrt() {
                    return Err(Reject::Encroach);
                }
            }
        }
        Ok(())
    }

    #[inline]
    fn vi(&self, v: u32) -> usize {
        if v == GHOST { self.pts.len() } else { v as usize }
    }

    /// Replace the cavity in `scratch` by the fan around the (already stored) vertex `v`.
    /// Returns a new real triangle.
    pub(super) fn apply_cavity(&mut self, v: u32) -> u32 {
        let need = self.pts.len() + 1;
        if self.scratch.start_slot.len() < need {
            self.scratch.start_slot.resize(need, NONE);
            self.scratch.end_slot.resize(need, NONE);
        }
        let cav = std::mem::take(&mut self.scratch.cav);
        let bnd = std::mem::take(&mut self.scratch.bnd);
        let mut newt = std::mem::take(&mut self.scratch.newt);
        newt.clear();
        for &t in &cav {
            self.kill_tri(t);
        }
        let mut real = NONE;
        for b in &bnd {
            // (a, b, v): vertex a idx 0, b idx 1, v idx 2; edge opposite v is (a, b).
            let mut verts = [b.a, b.b, v];
            let mut nb = [NONE, NONE, b.out_t];
            let mut cd = [C_NONE, C_NONE, b.code];
            // Put GHOST last.
            if let Some(j) = verts.iter().position(|&x| x == GHOST) {
                let k = (j + 1) % 3;
                let rot = |a: [u32; 3]| [a[k % 3], a[(k + 1) % 3], a[(k + 2) % 3]];
                verts = rot(verts);
                nb = rot(nb);
                cd = [cd[k % 3], cd[(k + 1) % 3], cd[(k + 2) % 3]];
            }
            let s = self.alloc_tri(verts, nb, cd);
            newt.push(s);
            if b.out_t != NONE {
                self.nbr[b.out_t as usize][b.out_i as usize] = s;
            }
            let ia = self.vi(b.a);
            let ib = self.vi(b.b);
            self.scratch.start_slot[ia] = s;
            self.scratch.end_slot[ib] = s;
            if real == NONE && verts[2] != GHOST {
                real = s;
            }
            for x in verts {
                if x != GHOST {
                    self.vtri[x as usize] = s;
                }
            }
        }
        // Link the fan: the edge (b, v) of the triangle of boundary edge (a, b) is shared with
        // the triangle that starts at b; its edge (v, a) with the one that ends at a.
        for (k, b) in bnd.iter().enumerate() {
            let s = newt[k];
            let sv = self.tri[s as usize];
            let ia = sv.iter().position(|&x| x == b.a).unwrap_or(0);
            let ib = sv.iter().position(|&x| x == b.b).unwrap_or(0);
            let after = self.scratch.start_slot[self.vi(b.b)];
            let before = self.scratch.end_slot[self.vi(b.a)];
            self.nbr[s as usize][ia] = after;
            self.nbr[s as usize][ib] = before;
        }
        self.scratch.cav = cav;
        self.scratch.bnd = bnd;
        self.scratch.newt = newt;
        self.hint = real;
        real
    }

    /// Insert input/Steiner vertex `v` (already in `pts`) during construction.
    fn insert_site(&mut self, v: u32) -> Result<(), TriError> {
        let p = self.p(v);
        let Some(t0) = self.locate(p, self.hint, false) else {
            return Err(TriError::Degenerate("point location failed".into()));
        };
        let tv = self.tri[t0 as usize];
        if tv[2] != GHOST {
            for w in tv {
                if self.p(w) == p {
                    return Err(TriError::DuplicatePoint {
                        a: w.min(v),
                        b: w.max(v),
                    });
                }
            }
        }
        self.build_cavity(p, t0, false);
        if self.check_cavity(p, false).is_err() {
            return Err(TriError::Degenerate(
                "insertion produced a non-positive triangle".into(),
            ));
        }
        self.apply_cavity(v);
        Ok(())
    }

    /// Flip the edge opposite vertex `i` of `t` (a strictly convex quad). Returns the new
    /// diagonal's end points `(a, d)`.
    pub(super) fn flip(&mut self, t: u32, i: usize) -> (u32, u32) {
        let tv = self.tri[t as usize];
        let n = self.nbr[t as usize][i];
        let nv = self.tri[n as usize];
        let (a, b, c) = (tv[i], tv[(i + 1) % 3], tv[(i + 2) % 3]);
        let j = if self.nbr[n as usize][0] == t {
            0
        } else if self.nbr[n as usize][1] == t {
            1
        } else {
            2
        };
        let d = nv[j];
        debug_assert!(nv[(j + 1) % 3] == c && nv[(j + 2) % 3] == b);
        let ti = self.nbr[t as usize];
        let tc = self.code[t as usize];
        let ni = self.nbr[n as usize];
        let nc = self.code[n as usize];
        // Outer edges of the quad (see the derivation in the module docs of `flip`):
        let (n_ab, c_ab) = (ti[(i + 2) % 3], tc[(i + 2) % 3]); // t's edge opposite c: (a, b)
        let (n_ca, c_ca) = (ti[(i + 1) % 3], tc[(i + 1) % 3]); // t's edge opposite b: (c, a)
        let (n_bd, c_bd) = (ni[(j + 1) % 3], nc[(j + 1) % 3]); // n's edge opposite c: (b, d)
        let (n_dc, c_dc) = (ni[(j + 2) % 3], nc[(j + 2) % 3]); // n's edge opposite b: (d, c)
        // T1 = (a, b, d) in slot t; T2 = (a, d, c) in slot n.
        self.tri[t as usize] = [a, b, d];
        self.nbr[t as usize] = [n_bd, n, n_ab];
        self.code[t as usize] = [c_bd, C_NONE, c_ab];
        self.tri[n as usize] = [a, d, c];
        self.nbr[n as usize] = [n_dc, n_ca, t];
        self.code[n as usize] = [c_dc, c_ca, C_NONE];
        if n_ca != NONE {
            let k = self.back_index(n_ca, t);
            self.nbr[n_ca as usize][k] = n;
        }
        if n_bd != NONE {
            let k = self.back_index(n_bd, n);
            self.nbr[n_bd as usize][k] = t;
        }
        self.tid[t as usize] = self.next_tid;
        self.tid[n as usize] = self.next_tid + 1;
        self.next_tid += 2;
        self.vtri[a as usize] = t;
        self.vtri[b as usize] = t;
        self.vtri[d as usize] = t;
        self.vtri[c as usize] = n;
        (a, d)
    }

    /// Index in `t` of the neighbour pointer that equals `other`.
    pub(super) fn back_index(&self, t: u32, other: u32) -> usize {
        let nn = self.nbr[t as usize];
        if nn[0] == other {
            0
        } else if nn[1] == other {
            1
        } else {
            2
        }
    }
}
