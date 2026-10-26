//! Tetrahedral data structure: SoA vertices and neighbours with a ghost vertex.
//!
//! Every tet is `[v0,v1,v2,v3]` with positive orientation (see [`crate::geom`]). `nbr[t][i]` is
//! the tet across the face opposite `v_i`. The outward face opposite slot `i` is
//! `(v[FACE[i][0]], v[FACE[i][1]], v[FACE[i][2]])` (the opposite vertex lies on the negative
//! side of it). A *ghost* tet has [`GHOST`] as one vertex; its real face is a convex-hull face of
//! the point set. `NONE` neighbours occur only after outside tets are dropped.

use crate::error::TetError;

/// The ghost (point at infinity) vertex id.
pub const GHOST: u32 = u32::MAX;
/// "No tet".
pub const NONE: u32 = u32::MAX;
/// Outward face triples by opposite slot.
pub const FACE: [[usize; 3]; 4] = [[1, 2, 3], [0, 3, 2], [0, 1, 3], [0, 2, 1]];

#[inline]
pub(crate) fn sorted3(a: u32, b: u32, c: u32) -> [u32; 3] {
    let mut k = [a, b, c];
    if k[0] > k[1] {
        k.swap(0, 1);
    }
    if k[1] > k[2] {
        k.swap(1, 2);
    }
    if k[0] > k[1] {
        k.swap(0, 1);
    }
    k
}

/// True if `a` and `b` are the same cyclic triple.
#[inline]
pub(crate) fn same_cycle(a: [u32; 3], b: [u32; 3]) -> bool {
    a == b || a == [b[1], b[2], b[0]] || a == [b[2], b[0], b[1]]
}

/// Rotate a triple so the smallest entry is first (canonical form of an oriented triangle).
#[inline]
pub(crate) fn canon_tri(t: [u32; 3]) -> [u32; 3] {
    if t[0] <= t[1] && t[0] <= t[2] {
        t
    } else if t[1] <= t[2] {
        [t[1], t[2], t[0]]
    } else {
        [t[2], t[0], t[1]]
    }
}

#[derive(Clone, Copy)]
struct Ent {
    key: [u32; 3],
    tri: [u32; 3],
    /// Index into `new` (or the external tet for boundary entries).
    a: u32,
    slot: u8,
    ext_slot: u8,
    is_new: bool,
}

/// Tetrahedra with neighbours.
#[derive(Clone, Default)]
pub struct Tds {
    /// Vertices per tet slot.
    pub verts: Vec<[u32; 4]>,
    /// Neighbours per tet slot (`NONE` = boundary).
    pub nbr: Vec<[u32; 4]>,
    /// Slot liveness.
    pub alive: Vec<bool>,
    free: Vec<u32>,
    /// One incident tet per point (stale for points without tets).
    pub vtet: Vec<u32>,
    mark: Vec<u32>,
    epoch: u32,
    smark: Vec<u32>,
    sepoch: u32,
    n_alive: usize,
    ents: Vec<Ent>,
    stack: Vec<u32>,
}

impl Tds {
    /// Empty structure for `n_points` points.
    pub fn new(n_points: usize) -> Self {
        Self { vtet: vec![NONE; n_points], ..Self::default() }
    }

    /// Grow the per-point table.
    pub fn ensure_points(&mut self, n: usize) {
        if self.vtet.len() < n {
            self.vtet.resize(n, NONE);
        }
    }

    /// Number of live tets.
    pub fn len(&self) -> usize {
        self.n_alive
    }

    /// Number of tet slots (live or free).
    pub fn slots(&self) -> usize {
        self.verts.len()
    }

    /// Slot of the ghost vertex in `t`, if `t` is a ghost tet.
    #[inline]
    pub fn ghost_slot(&self, t: u32) -> Option<usize> {
        self.verts[t as usize].iter().position(|&v| v == GHOST)
    }

    /// Outward face opposite slot `i`.
    #[inline]
    pub fn face(&self, t: u32, i: usize) -> [u32; 3] {
        let v = &self.verts[t as usize];
        [v[FACE[i][0]], v[FACE[i][1]], v[FACE[i][2]]]
    }

    /// True if `t` has vertex `x`.
    #[inline]
    pub fn has(&self, t: u32, x: u32) -> bool {
        self.verts[t as usize].contains(&x)
    }

    /// Slot `j` of `n` with `nbr[n][j] == t`.
    #[inline]
    pub fn back_slot(&self, n: u32, t: u32) -> Option<usize> {
        self.nbr[n as usize].iter().position(|&x| x == t)
    }

    fn begin_mark(&mut self) {
        self.epoch = self.epoch.wrapping_add(1);
        if self.epoch == 0 {
            self.mark.iter_mut().for_each(|m| *m = 0);
            self.epoch = 1;
        }
    }

    fn alloc(&mut self) -> u32 {
        if let Some(t) = self.free.pop() {
            self.alive[t as usize] = true;
            self.n_alive += 1;
            return t;
        }
        self.verts.push([NONE; 4]);
        self.nbr.push([NONE; 4]);
        self.alive.push(true);
        self.mark.push(0);
        self.smark.push(0);
        self.n_alive += 1;
        (self.verts.len() - 1) as u32
    }

    /// Replace the tets `old` by `new` (positive tets), gluing faces. `old` must be live tets
    /// whose boundary faces are exactly covered, with opposite orientation, by faces of `new`;
    /// internal faces of `new` must pair up. On error nothing is modified. The ids of the new
    /// tets (in the order of `new`) are written to `ids`.
    pub fn replace(&mut self, old: &[u32], new: &[[u32; 4]], ids: &mut Vec<u32>) -> Result<(), TetError> {
        self.begin_mark();
        for &t in old {
            self.mark[t as usize] = self.epoch;
        }
        let mut ents = std::mem::take(&mut self.ents);
        ents.clear();
        for &t in old {
            for i in 0..4 {
                let n = self.nbr[t as usize][i];
                if n == NONE || self.mark[n as usize] != self.epoch {
                    let f = self.face(t, i);
                    let ext_slot = if n == NONE { 255 } else { self.back_slot(n, t).unwrap_or(255) as u8 };
                    ents.push(Ent { key: sorted3(f[0], f[1], f[2]), tri: f, a: n, slot: 0, ext_slot, is_new: false });
                }
            }
        }
        for (k, v) in new.iter().enumerate() {
            for i in 0..4 {
                let f = [v[FACE[i][0]], v[FACE[i][1]], v[FACE[i][2]]];
                ents.push(Ent { key: sorted3(f[0], f[1], f[2]), tri: f, a: k as u32, slot: i as u8, ext_slot: 0, is_new: true });
            }
        }
        ents.sort_unstable_by(|x, y| x.key.cmp(&y.key).then(x.is_new.cmp(&y.is_new)).then(x.a.cmp(&y.a)).then(x.slot.cmp(&y.slot)));
        // Validate groups: [bnd,new] or [new,new].
        let mut ok = true;
        let mut i = 0;
        while i < ents.len() {
            let mut j = i + 1;
            while j < ents.len() && ents[j].key == ents[i].key {
                j += 1;
            }
            let g = &ents[i..j];
            let good = match g.len() {
                2 => {
                    let rev = |t: [u32; 3]| [t[0], t[2], t[1]];
                    let (x, y) = (&g[0], &g[1]);
                    // bnd (is_new false) sorts first, so (bnd,new) or (new,new).
                    // Old boundary face and new face bound the same region: same orientation;
                    // two new tets across an internal face see it reversed.
                    y.is_new && if x.is_new { same_cycle(x.tri, rev(y.tri)) } else { same_cycle(x.tri, y.tri) }
                }
                _ => false,
            };
            if !good {
                ok = false;
                break;
            }
            i = j;
        }
        if !ok {
            self.ents = ents;
            return Err(TetError::Internal("replace: cavity faces do not match".into()));
        }
        // Apply.
        for &t in old {
            self.alive[t as usize] = false;
            self.n_alive -= 1;
            self.free.push(t);
        }
        ids.clear();
        for v in new {
            let id = self.alloc();
            self.verts[id as usize] = *v;
            self.nbr[id as usize] = [NONE; 4];
            ids.push(id);
            for &x in v {
                if x != GHOST {
                    self.vtet[x as usize] = id;
                }
            }
        }
        let mut i = 0;
        while i < ents.len() {
            let (x, y) = (ents[i], ents[i + 1]);
            if x.is_new {
                let (ta, tb) = (ids[x.a as usize], ids[y.a as usize]);
                self.nbr[ta as usize][x.slot as usize] = tb;
                self.nbr[tb as usize][y.slot as usize] = ta;
            } else {
                let tb = ids[y.a as usize];
                self.nbr[tb as usize][y.slot as usize] = x.a;
                if x.a != NONE && x.ext_slot != 255 {
                    self.nbr[x.a as usize][x.ext_slot as usize] = tb;
                }
            }
            i += 2;
        }
        self.ents = ents;
        Ok(())
    }

    /// Remove `old` tets without replacement: their neighbours outside the set get `NONE` there.
    pub fn remove(&mut self, old: &[u32]) {
        self.begin_mark();
        for &t in old {
            self.mark[t as usize] = self.epoch;
        }
        for &t in old {
            for i in 0..4 {
                let n = self.nbr[t as usize][i];
                if n != NONE && self.mark[n as usize] != self.epoch
                    && let Some(j) = self.back_slot(n, t) {
                        self.nbr[n as usize][j] = NONE;
                    }
            }
        }
        for &t in old {
            self.alive[t as usize] = false;
            self.n_alive -= 1;
            self.free.push(t);
        }
    }

    /// All live tets incident to `v`, in discovery order (depth first from `vtet[v]`).
    pub fn star(&mut self, v: u32, out: &mut Vec<u32>) {
        out.clear();
        let t0 = self.vtet[v as usize];
        if t0 == NONE || !self.alive[t0 as usize] || !self.has(t0, v) {
            return;
        }
        self.sepoch = self.sepoch.wrapping_add(1);
        if self.sepoch == 0 {
            self.smark.iter_mut().for_each(|m| *m = 0);
            self.sepoch = 1;
        }
        let mut stack = std::mem::take(&mut self.stack);
        stack.clear();
        stack.push(t0);
        self.smark[t0 as usize] = self.sepoch;
        while let Some(t) = stack.pop() {
            out.push(t);
            for i in 0..4 {
                if self.verts[t as usize][i] == v {
                    continue;
                }
                let n = self.nbr[t as usize][i];
                if n != NONE && self.smark[n as usize] != self.sepoch {
                    self.smark[n as usize] = self.sepoch;
                    stack.push(n);
                }
            }
        }
        self.stack = stack;
    }

    /// A tet containing both `a` and `b`, if the edge exists. Scans the star of `a` and stops at
    /// the first hit.
    pub fn find_edge(&mut self, a: u32, b: u32) -> Option<u32> {
        let t0 = self.vtet[a as usize];
        if t0 == NONE || !self.alive[t0 as usize] || !self.has(t0, a) {
            return None;
        }
        self.sepoch = self.sepoch.wrapping_add(1);
        if self.sepoch == 0 {
            self.smark.iter_mut().for_each(|m| *m = 0);
            self.sepoch = 1;
        }
        let mut stack = std::mem::take(&mut self.stack);
        stack.clear();
        stack.push(t0);
        self.smark[t0 as usize] = self.sepoch;
        let mut found = None;
        while let Some(t) = stack.pop() {
            if self.has(t, b) {
                found = Some(t);
                break;
            }
            for i in 0..4 {
                if self.verts[t as usize][i] == a {
                    continue;
                }
                let n = self.nbr[t as usize][i];
                if n != NONE && self.smark[n as usize] != self.sepoch {
                    self.smark[n as usize] = self.sepoch;
                    stack.push(n);
                }
            }
        }
        self.stack = stack;
        found
    }

    /// The tets around edge `(a,b)` in cyclic order, starting from `t0` (a tet containing the
    /// edge), together with the ring vertices: `poly[i]` is the apex shared by `ring[i-1]` and
    /// `ring[i]` (so `ring[i]` has edge vertices `a,b` and ring vertices `poly[i]`,
    /// `poly[i+1]`). Returns `None` if the ring is open (hits `NONE`) or longer than `max`.
    pub fn ring(&self, a: u32, b: u32, t0: u32, max: usize, ring: &mut Vec<u32>, poly: &mut Vec<u32>) -> bool {
        ring.clear();
        poly.clear();
        let mut t = t0;
        // Choose the exit slot: the first vertex that is neither a nor b.
        let v0 = self.verts[t as usize];
        let mut others = v0.iter().enumerate().filter(|&(_, &x)| x != a && x != b).map(|(i, _)| i);
        let (Some(s_enter), Some(s_exit)) = (others.next(), others.next()) else { return false };
        // We exit through the face opposite `s_exit`; the vertex at `s_enter` is shared.
        let mut shared = v0[s_enter];
        poly.push(v0[s_exit]);
        poly.push(shared);
        let mut exit_slot = s_exit;
        loop {
            ring.push(t);
            if ring.len() > max {
                return false;
            }
            let n = self.nbr[t as usize][exit_slot];
            if n == NONE {
                return false;
            }
            if n == t0 {
                break;
            }
            // In n, the entering face contains `shared`; exit opposite `shared`.
            let vn = self.verts[n as usize];
            let Some(ss) = vn.iter().position(|&x| x == shared) else { return false };
            // The other non-edge vertex of n becomes the next shared vertex.
            let Some(nxt) = vn.iter().position(|&x| x != a && x != b && x != shared) else { return false };
            poly.push(vn[nxt]);
            shared = vn[nxt];
            exit_slot = ss;
            t = n;
        }
        // The walk pushed the closing vertex twice (first and last).
        poly.pop();
        poly.len() == ring.len()
    }
}

#[cfg(test)]
impl Tds {
    /// Structural self-check (adjacency symmetry and face agreement); for tests and debug.
    pub fn check(&self) -> Result<(), String> {
        for t in 0..self.slots() as u32 {
            if !self.alive[t as usize] {
                continue;
            }
            for i in 0..4 {
                let n = self.nbr[t as usize][i];
                if n == NONE {
                    continue;
                }
                if !self.alive[n as usize] {
                    return Err(format!("tet {t} slot {i} points to dead tet {n}"));
                }
                let Some(j) = self.back_slot(n, t) else { return Err(format!("tet {t}->{n} not symmetric")) };
                let f = self.face(t, i);
                let g = self.face(n, j);
                if !same_cycle(f, [g[0], g[2], g[1]]) {
                    return Err(format!("faces of {t}:{i} {f:?} and {n}:{j} {g:?} not reversed"));
                }
            }
        }
        Ok(())
    }
}
