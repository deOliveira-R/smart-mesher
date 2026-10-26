//! Bowyer-Watson Delaunay tetrahedralization with a ghost vertex and symbolic perturbation.
//!
//! Insertion order is BRIO rounds (seeded from the point count only) with a Hilbert key inside
//! each round. All predicates are exact; ties (cospherical, coplanar) are broken by simulation of
//! simplicity with the point index as the perturbation id, so the triangulation of a point set is
//! unique and independent of the insertion order.

use crate::error::TetError;
use crate::geom::{Pts, V3};
use crate::tds::{FACE, GHOST, NONE, Tds};

/// Delaunay triangulation under construction.
pub(crate) struct Dt {
    pub pts: Vec<V3>,
    pub tds: Tds,
    pub last: u32,
    rng: u64,
    cmark: Vec<u32>,
    cepoch: u32,
    cav: Vec<u32>,
    newt: Vec<[u32; 4]>,
    ids: Vec<u32>,
    stack: Vec<u32>,
}

#[inline]
pub(crate) fn splitmix(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = x;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

impl Dt {
    pub fn new(pts: Vec<V3>) -> Self {
        let n = pts.len();
        Self {
            pts,
            tds: Tds::new(n),
            last: NONE,
            rng: splitmix(n as u64),
            cmark: Vec::new(),
            cepoch: 0,
            cav: Vec::new(),
            newt: Vec::new(),
            ids: Vec::new(),
            stack: Vec::new(),
        }
    }

    #[inline]
    pub fn sos(&self) -> Pts<'_> {
        Pts(&self.pts)
    }

    /// Append a point (it must still be inserted).
    pub fn push_point(&mut self, p: V3) -> u32 {
        self.pts.push(p);
        self.tds.ensure_points(self.pts.len());
        (self.pts.len() - 1) as u32
    }

    /// Build the first tet and its four ghosts from four points.
    pub fn init(&mut self, v: [u32; 4]) -> Result<(), TetError> {
        let s = self.sos();
        let mut v = v;
        if s.ori_s(v[0], v[1], v[2], v[3]) < 0 {
            v.swap(2, 3);
        }
        let mut new: Vec<[u32; 4]> = vec![v];
        for i in 0..4 {
            let f = [v[FACE[i][0]], v[FACE[i][1]], v[FACE[i][2]]];
            new.push([f[0], f[1], f[2], GHOST]);
        }
        let mut ids = Vec::new();
        self.tds.replace(&[], &new, &mut ids)?;
        self.last = ids[0];
        Ok(())
    }

    /// True if tet `t` is in conflict with point `p` (p inside its circumsphere, or beyond the
    /// hull face of a ghost tet).
    #[inline]
    fn conflict(&self, t: u32, p: u32) -> bool {
        let v = self.tds.verts[t as usize];
        let s = Pts(&self.pts);
        if let Some(g) = v.iter().position(|&x| x == GHOST) {
            let f = FACE[g];
            // Hull face outward from the interior = reverse of the ghost tet's outward face.
            let (x, y, z) = (v[f[0]], v[f[2]], v[f[1]]);
            s.ori_s(x, y, z, p) > 0
        } else {
            s.in_sphere_s(v[0], v[1], v[2], v[3], p)
        }
    }

    /// Find a tet in conflict with `p` by a stochastic visibility walk.
    fn locate(&mut self, p: u32) -> Result<u32, TetError> {
        let s = Pts(&self.pts);
        let mut t = self.last;
        if t == NONE || !self.tds.alive[t as usize] {
            t = (0..self.tds.slots() as u32).find(|&t| self.tds.alive[t as usize]).ok_or_else(|| TetError::Internal("empty triangulation".into()))?;
        }
        let limit = 50 + 4 * self.tds.slots();
        for _ in 0..limit {
            let v = self.tds.verts[t as usize];
            if let Some(g) = v.iter().position(|&x| x == GHOST) {
                if self.conflict(t, p) {
                    return Ok(t);
                }
                let n = self.tds.nbr[t as usize][g];
                if n == NONE {
                    break;
                }
                t = n;
                continue;
            }
            self.rng = splitmix(self.rng);
            let i0 = (self.rng >> 33) as usize & 3;
            let mut moved = false;
            for k in 0..4 {
                let i = (i0 + k) & 3;
                let f = FACE[i];
                if s.ori_s(v[f[0]], v[f[1]], v[f[2]], p) > 0 {
                    let n = self.tds.nbr[t as usize][i];
                    if n != NONE {
                        t = n;
                        moved = true;
                        break;
                    }
                }
            }
            if !moved {
                return Ok(t);
            }
        }
        // Fallback: linear scan.
        (0..self.tds.slots() as u32)
            .find(|&t| self.tds.alive[t as usize] && self.conflict(t, p))
            .ok_or_else(|| TetError::Internal("point location failed".into()))
    }

    fn mark_in(&mut self, t: u32) {
        if self.cmark.len() < self.tds.slots() {
            self.cmark.resize(self.tds.slots() + 64, 0);
        }
        self.cmark[t as usize] = 2 * self.cepoch + 1;
    }
    fn mark_out(&mut self, t: u32) {
        if self.cmark.len() < self.tds.slots() {
            self.cmark.resize(self.tds.slots() + 64, 0);
        }
        self.cmark[t as usize] = 2 * self.cepoch;
    }
    #[inline]
    fn marked(&self, t: u32) -> u32 {
        let m = self.cmark.get(t as usize).copied().unwrap_or(0);
        if m / 2 == self.cepoch { 1 + (m & 1) } else { 0 }
    }

    /// Insert point `p` (already in `pts`).
    pub fn insert(&mut self, p: u32) -> Result<(), TetError> {
        let t0 = self.locate(p)?;
        self.cepoch += 1;
        if self.cmark.len() < self.tds.slots() {
            self.cmark.resize(self.tds.slots() + 64, 0);
        }
        self.cav.clear();
        self.stack.clear();
        self.mark_in(t0);
        self.stack.push(t0);
        while let Some(t) = self.stack.pop() {
            self.cav.push(t);
            for i in 0..4 {
                let n = self.tds.nbr[t as usize][i];
                if n == NONE || self.marked(n) != 0 {
                    continue;
                }
                if self.conflict(n, p) {
                    self.mark_in(n);
                    self.stack.push(n);
                } else {
                    self.mark_out(n);
                }
            }
        }
        self.newt.clear();
        for &t in &self.cav {
            for i in 0..4 {
                let n = self.tds.nbr[t as usize][i];
                if n == NONE || self.marked(n) != 2 {
                    let f = self.tds.face(t, i);
                    self.newt.push([p, f[0], f[1], f[2]]);
                }
            }
        }
        let cav = std::mem::take(&mut self.cav);
        let newt = std::mem::take(&mut self.newt);
        let mut ids = std::mem::take(&mut self.ids);
        let r = self.tds.replace(&cav, &newt, &mut ids);
        if r.is_ok() {
            self.last = ids.iter().copied().find(|&t| self.tds.ghost_slot(t).is_none()).unwrap_or(ids[0]);
        }
        self.cav = cav;
        self.newt = newt;
        self.ids = ids;
        r
    }
}

/// 3D Hilbert key (Skilling's transposition) of quantized coordinates with `bits` bits each.
fn hilbert3(mut x: [u32; 3], bits: u32) -> u64 {
    let m = 1u32 << (bits - 1);
    let mut q = m;
    while q > 1 {
        let p = q - 1;
        for i in 0..3 {
            if x[i] & q != 0 {
                x[0] ^= p;
            } else {
                let t = (x[0] ^ x[i]) & p;
                x[0] ^= t;
                x[i] ^= t;
            }
        }
        q >>= 1;
    }
    for i in 1..3 {
        x[i] ^= x[i - 1];
    }
    let mut t = 0;
    let mut q = m;
    while q > 1 {
        if x[2] & q != 0 {
            t ^= q - 1;
        }
        q >>= 1;
    }
    for xi in &mut x {
        *xi ^= t;
    }
    let mut key = 0u64;
    for b in (0..bits).rev() {
        for xi in &x {
            key = (key << 1) | u64::from((xi >> b) & 1);
        }
    }
    key
}

/// Insertion order of `ids` (indices into `pts`): BRIO rounds, Hilbert order inside a round.
pub(crate) fn insertion_order(pts: &[V3], ids: &[u32]) -> Vec<u32> {
    let n = ids.len();
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for &i in ids {
        for k in 0..3 {
            lo[k] = lo[k].min(pts[i as usize][k]);
            hi[k] = hi[k].max(pts[i as usize][k]);
        }
    }
    let span = (0..3).map(|k| hi[k] - lo[k]).fold(0.0f64, f64::max);
    let bits = 20u32;
    let scale = if span > 0.0 { ((1u64 << bits) - 1) as f64 / span } else { 0.0 };
    let seed = splitmix(n as u64);
    let mut keyed: Vec<(u32, u64, u32)> = ids
        .iter()
        .map(|&i| {
            let p = pts[i as usize];
            let q = |k: usize| (((p[k] - lo[k]) * scale) as u64).min((1 << bits) - 1) as u32;
            let h = splitmix(seed ^ u64::from(i));
            // Rarer (larger trailing zero count) points go to earlier rounds.
            let level = h.trailing_zeros().min(20);
            (20 - level, hilbert3([q(0), q(1), q(2)], bits), i)
        })
        .collect();
    keyed.sort_unstable();
    keyed.into_iter().map(|k| k.2).collect()
}

/// Delaunay-triangulate all of `pts` (distinct, in exact range). Returns the real tets
/// (possibly including exactly flat ones) and the built structure.
pub(crate) fn build(pts: Vec<V3>, first: Option<[u32; 4]>, ids_override: Option<Vec<u32>>) -> Result<Dt, TetError> {
    let n = pts.len();
    let ids: Vec<u32> = ids_override.unwrap_or_else(|| (0..n as u32).collect());
    let order = insertion_order(&pts, &ids);
    let mut dt = Dt::new(pts);
    let (init, skip): ([u32; 4], Vec<u32>) = match first {
        Some(f) => (f, f.to_vec()),
        None => {
            let f = [order[0], order[1], order[2], order[3]];
            (f, f.to_vec())
        }
    };
    dt.init(init)?;
    for &p in &order {
        if skip.contains(&p) {
            continue;
        }
        dt.insert(p)?;
    }
    Ok(dt)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn incremental_consistency() {
        let mut r = 12345u64;
        let mut rnd = || {
            r = splitmix(r);
            (r >> 11) as f64 / (1u64 << 53) as f64
        };
        let pts: Vec<V3> = (0..60).map(|_| [rnd(), rnd(), rnd()]).collect();
        let order = insertion_order(&pts, &(0..60).collect::<Vec<_>>());
        let mut dt = Dt::new(pts);
        dt.init([order[0], order[1], order[2], order[3]]).unwrap();
        for &p in &order[4..] {
            dt.insert(p).unwrap();
            dt.tds.check().unwrap_or_else(|e| panic!("after inserting {p}: {e}"));
            for t in 0..dt.tds.slots() as u32 {
                if dt.tds.alive[t as usize] && dt.tds.ghost_slot(t).is_none() {
                    let v = dt.tds.verts[t as usize];
                    assert!(dt.sos().ori_s(v[0], v[1], v[2], v[3]) > 0, "inverted tet after {p}");
                }
            }
        }
    }
}

#[cfg(test)]
mod tests2 {
    use super::*;
    use crate::geom::vol6;

    #[test]
    fn volume_of_cube_cloud() {
        let mut r = 777u64;
        let mut rnd = || {
            r = splitmix(r);
            (r >> 11) as f64 / (1u64 << 53) as f64
        };
        for n in [10usize, 50, 200, 2000] {
            let mut pts: Vec<V3> = (0..n).map(|_| [rnd(), rnd(), rnd()]).collect();
            for i in 0..8 {
                pts.push([(i & 1) as f64, ((i >> 1) & 1) as f64, ((i >> 2) & 1) as f64]);
            }
            let dt = build(pts.clone(), None, None).unwrap();
            dt.tds.check().unwrap();
            let mut vol = 0.0;
            let mut nreal = 0;
            for t in 0..dt.tds.slots() as u32 {
                if dt.tds.alive[t as usize] && dt.tds.ghost_slot(t).is_none() {
                    let v = dt.tds.verts[t as usize];
                    vol += vol6(pts[v[0] as usize], pts[v[1] as usize], pts[v[2] as usize], pts[v[3] as usize]) / 6.0;
                    nreal += 1;
                }
            }
            eprintln!("n={n} real={nreal} vol={vol}");
        }
    }
}
