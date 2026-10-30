//! Frontal point placement. Rows of points are offset from the boundary by the local size,
//! along the inward normal of the front (a *square* is built on every front edge, so pairs
//! of triangles recombine into near-squares); collisions between fronts are resolved by the
//! Delaunay triangulation and a minimum-distance rule, not by seaming. A Delaunay refinement
//! pass fills whatever the rows leave.

use super::PaveError;
use super::cdt::{Cdt, Loc, NONE};
use super::metric::{Met, Ord64, SizeMap, Space};
use core::ops::Range;

/// Absolute bound on the vertices of one region; a request beyond it (for instance a size of
/// 1e-9) is a typed [`PaveError::TooLarge`] instead of a near-unbounded run.
pub(crate) const MAX_VERTICES: usize = 2_000_000;

/// Minimum distance (in sizes) between a frontal point and any existing vertex.
const ALPHA_ROW: f64 = 0.6;
/// Proposals closer than this fraction of the row distance count as duplicates.
const DUPLICATE: f64 = 0.4;
/// Same for refinement points.
const ALPHA_FILL: f64 = 0.5;
/// Triangles with circumradius above this many sizes are refined.
const FILL_RATIO: f64 = 0.85;

/// An ordered chain of vertices with the unmeshed region on its left.
#[derive(Clone, Debug)]
struct Chain {
    v: Vec<u32>,
    closed: bool,
}

enum Res {
    Accepted(u32),
    Crowd,
    Collide,
}

/// Knobs of the placement that the strategy ladder varies.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Tuning {
    /// Multiplier of the size field.
    pub hscale: f64,
    /// Multiplier of the row distance.
    pub row_factor: f64,
    /// Adjacent proposals dropped together where a row crowds.
    pub drop_run: usize,
}

/// The triangulation plus the per-vertex data the placement needs.
pub(crate) struct Mesher<'a> {
    pub cdt: Cdt,
    pub met: Vec<Met>,
    pub h: Vec<f64>,
    tag: Vec<u64>,
    /// Deterministic work counter (triangle visits of the recombination), for the ladder cut-off.
    pub work: u64,
    space: &'a Space<'a>,
    size_map: Option<SizeMap>,
    /// Soft vertex bound of this attempt (runaway guard, retried by the ladder).
    max_vertices: usize,
    hscale: f64,
    /// Multiplier of the row distance (strategy ladder).
    row_factor: f64,
    /// Adjacent proposals dropped together where a row crowds (strategy ladder).
    drop_run: usize,
}

fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

impl<'a> Mesher<'a> {
    pub fn new(space: &'a Space<'a>, cdt: Cdt, nb: usize, size_map: Option<SizeMap>, tuning: Tuning) -> Result<Self, PaveError> {
        let mut m = Mesher { work: 0, cdt, met: Vec::with_capacity(nb), h: Vec::with_capacity(nb), tag: Vec::with_capacity(nb), space, size_map, max_vertices: usize::MAX, hscale: tuning.hscale, row_factor: tuning.row_factor, drop_run: tuning.drop_run };
        for i in 0..nb {
            let w = m.cdt.pts[i];
            let (mt, h) = (m.space.met(w)?, m.h_at(w)?);
            m.met.push(mt);
            m.h.push(h);
            m.tag.push(0);
        }
        // How many vertices the wanted size asks for: the area of every boundary triangle
        // over the square of the size at its centroid (an absurd request is a typed error
        // before any point is placed).
        let tris: Vec<u32> = m.cdt.triangles().collect();
        let mut est = 1.0f64;
        for t in tris {
            let v = m.cdt.tv[t as usize];
            let p: [[f64; 2]; 3] = core::array::from_fn(|k| m.cdt.pts[v[k] as usize]);
            let area = 0.5 * ((p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0])).abs();
            let c = [(p[0][0] + p[1][0] + p[2][0]) / 3.0, (p[0][1] + p[1][1] + p[2][1]) / 3.0];
            let h = m.h_at(c)?;
            est += area / (h * h);
        }
        if est > MAX_VERTICES as f64 {
            return Err(PaveError::TooLarge { estimate: est.min(usize::MAX as f64) as usize, limit: MAX_VERTICES });
        }
        // Generous bound that only a runaway placement loop can reach.
        m.max_vertices = (nb + (40.0 * est) as usize + 10_000).min(MAX_VERTICES);
        Ok(m)
    }

    /// Tighten the soft vertex bound of this attempt to at most `n`.
    pub fn limit_vertices(&mut self, n: usize) {
        self.max_vertices = self.max_vertices.min(n.max(self.cdt.pts.len() + 1));
    }

    /// Fails when one more vertex would pass a bound: the absolute one is a typed error
    /// that ends the run, the soft one a retryable guard.
    fn check_budget(&self) -> Result<(), PaveError> {
        let n = self.cdt.pts.len();
        if n >= MAX_VERTICES {
            return Err(PaveError::TooLarge { estimate: n, limit: MAX_VERTICES });
        }
        if n >= self.max_vertices {
            return Err(PaveError::Internal("vertex budget exhausted"));
        }
        Ok(())
    }

    /// The wanted size at the frame point `w` before the strategy's `hscale`.
    pub fn wanted(&mut self, w: [f64; 2]) -> Result<f64, PaveError> {
        let k = core::mem::replace(&mut self.hscale, 1.0);
        let h = self.h_at(w);
        self.hscale = k;
        h
    }

    fn h_at(&mut self, w: [f64; 2]) -> Result<f64, PaveError> {
        // An explicit size is limited by the boundary-derived field, which carries the
        // gradation: the interior cannot be coarser than the boundary spacing allows. Finer
        // than the boundary is honoured as asked (no lower gradation; see DESIGN.md).
        let h = match (&self.space.size, &mut self.size_map) {
            (Some(f), Some(m)) => {
                let want = f(self.space.frame.to_uv(w));
                if !(want.is_finite() && want > 0.0) {
                    return Err(PaveError::BadMetric);
                }
                want.min(m.at(w))
            }
            (Some(f), None) => f(self.space.frame.to_uv(w)),
            (None, Some(m)) => m.at(w),
            (None, None) => 1.0,
        };
        let h = h * self.hscale;
        if h.is_finite() && h > 0.0 { Ok(h) } else { Err(PaveError::BadMetric) }
    }

    /// Insert the point `p` (already known to be inside) and record its data.
    fn insert_checked(&mut self, p: [f64; 2], loc: Loc, tag: u64) -> Result<u32, PaveError> {
        let met = self.space.met(p)?;
        let h = self.h_at(p)?;
        let id = self.cdt.insert(p, loc).map_err(|_| PaveError::Internal("point insertion failed"))?;
        debug_assert_eq!(id as usize, self.met.len());
        self.met.push(met);
        self.h.push(h);
        self.tag.push(tag);
        Ok(id)
    }

    /// Insert `p` wherever it lands (used by the edge splitting of the recombination).
    pub fn insert_free(&mut self, p: [f64; 2], hint: u32) -> Result<Option<u32>, PaveError> {
        self.check_budget()?;
        let loc = self.cdt.locate(p, hint);
        match loc {
            Loc::In(_) => {}
            Loc::OnEdge(t, i) if !self.cdt.tc[t as usize][i as usize] && self.cdt.tn[t as usize][i as usize] != NONE => {}
            _ => return Ok(None),
        }
        self.insert_checked(p, loc, u64::MAX).map(Some)
    }

    // ---------------------------------------------------------------- rows

    fn propose(&self, c: &Chain, i: usize) -> Option<[f64; 2]> {
        let n = c.v.len();
        let v = c.v[i] as usize;
        let prev = if i > 0 { Some(c.v[i - 1]) } else if c.closed { Some(c.v[n - 1]) } else { None };
        let next = if i + 1 < n { Some(c.v[i + 1]) } else if c.closed { Some(c.v[0]) } else { None };
        let (pv, b) = (self.cdt.pts[v], self.met[v]);
        let unit_normal = |from: [f64; 2], to: [f64; 2]| -> Option<[f64; 2]> {
            let d = b.apply(sub(to, from));
            let l = d[0].hypot(d[1]);
            if !(l.is_finite() && l > 1e-300) { None } else { Some([-d[1] / l, d[0] / l]) }
        };
        let n1 = prev.and_then(|p| unit_normal(self.cdt.pts[p as usize], pv));
        let n2 = next.and_then(|q| unit_normal(pv, self.cdt.pts[q as usize]));
        let h = self.h[v];
        let (dir, d) = match (n1, n2) {
            (Some(a), Some(c)) => {
                let s = [a[0] + c[0], a[1] + c[1]];
                let l = s[0].hypot(s[1]);
                if l < 1e-9 {
                    (a, h)
                } else {
                    let u = [s[0] / l, s[1] / l];
                    let cos_half = (u[0] * a[0] + u[1] * a[1]).clamp(0.5, 1.0);
                    (u, h / cos_half)
                }
            }
            (Some(a), None) | (None, Some(a)) => (a, h),
            (None, None) => return None,
        };
        let off = b.unapply([dir[0] * d * self.row_factor, dir[1] * d * self.row_factor]);
        Some([pv[0] + off[0], pv[1] + off[1]])
    }

    /// Try to insert a row point proposed from `src`; `tag` identifies the chain and pass.
    fn try_row(&mut self, p: [f64; 2], src: u32, tag: u64) -> Result<Res, PaveError> {
        let loc = self.cdt.locate(p, self.cdt.vt[src as usize]);
        match loc {
            Loc::Outside => return Ok(Res::Collide),
            Loc::OnVertex(v) => return Ok(if self.tag[v as usize] == tag { Res::Crowd } else { Res::Collide }),
            Loc::OnEdge(t, i) if self.cdt.tc[t as usize][i as usize] || self.cdt.tn[t as usize][i as usize] == NONE => return Ok(Res::Collide),
            _ => {}
        }
        let seeds = self.cdt.seeds(loc);
        let cav = self.cdt.cavity(p, &seeds);
        let (b, lim) = (self.met[src as usize], ALPHA_ROW * self.h[src as usize]);
        let (mut crowd, mut collide) = (false, false);
        for t in cav {
            for v in self.cdt.tv[t as usize] {
                if b.len(sub(p, self.cdt.pts[v as usize])) < lim {
                    if self.tag[v as usize] == tag { crowd = true } else { collide = true }
                }
            }
        }
        if collide {
            return Ok(Res::Collide);
        }
        if crowd {
            return Ok(Res::Crowd);
        }
        self.check_budget()?;
        Ok(Res::Accepted(self.insert_checked(p, loc, tag)?))
    }

    /// Indices of the proposals that survive the spacing filter along a chain. A closed
    /// chain keeps the parity of its length, so that the strip between two rows can be
    /// recombined on its own.
    fn thin(&self, c: &Chain, props: &[[f64; 2]]) -> Vec<usize> {
        let n = c.v.len();
        let dist = |i: usize, j: usize| self.met[c.v[i] as usize].len(sub(props[i], props[j]));
        let lim = |i: usize| ALPHA_ROW * self.h[c.v[i] as usize];
        let mut kept: Vec<usize> = Vec::with_capacity(n);
        let mut force = 0usize;
        for i in 0..n {
            if force > 0 {
                force -= 1;
                continue;
            }
            match kept.last().map(|&j| dist(i, j)) {
                None => kept.push(i),
                Some(d) if d >= lim(i) => kept.push(i),
                // Coincident proposals (the corners of a front): drop the duplicate only.
                Some(d) if d < DUPLICATE * lim(i) => {}
                // Genuine crowding: thin out in pairs of adjacent proposals, since a lone
                // dropped node leaves a wedge triangle that no convex pair can absorb.
                Some(_) => force = self.drop_run.max(1) - 1,
            }
        }
        if c.closed {
            while kept.len() > 1 && dist(kept[kept.len() - 1], kept[0]) < lim(kept[0]) {
                kept.pop();
            }
            if (n - kept.len()) % 2 == 1 && kept.len() > 3 {
                // Drop the kept proposal with the smallest neighbour gaps.
                let m = kept.len();
                let mut best: Option<(Ord64, usize)> = None;
                for k in 0..m {
                    let (a, b, c2) = (kept[(k + m - 1) % m], kept[k], kept[(k + 1) % m]);
                    let key = (Ord64(dist(a, b) + dist(b, c2)), k);
                    if best.is_none_or(|x| key < x) {
                        best = Some(key);
                    }
                }
                if let Some((_, k)) = best {
                    kept.remove(k);
                }
            }
        }
        kept
    }

    /// Row-by-row frontal insertion from the given loops.
    pub fn rows(&mut self, loops: &[Range<usize>]) -> Result<(), PaveError> {
        let mut chains: Vec<Chain> = loops.iter().map(|r| Chain { v: (r.start as u32..r.end as u32).collect(), closed: true }).collect();
        let mut pass = 0u64;
        while !chains.is_empty() {
            pass += 1;
            if pass > 1_000_000 {
                return Err(PaveError::Internal("too many rows"));
            }
            let before = self.cdt.pts.len();
            let mut next: Vec<Chain> = Vec::new();
            for (ci, c) in chains.iter().enumerate() {
                let tag = (pass << 32) | ci as u64;
                let n = c.v.len();
                let props: Vec<[f64; 2]> = (0..n).map(|i| self.propose(c, i).unwrap_or([f64::NAN; 2])).collect();
                let ok = |i: usize| props[i][0].is_finite() && props[i][1].is_finite();
                let kept = if props.iter().all(|p| p[0].is_finite() && p[1].is_finite()) { self.thin(c, &props) } else { (0..n).filter(|i| ok(*i)).collect() };
                let mut seq: Vec<Option<u32>> = Vec::with_capacity(kept.len());
                for &i in &kept {
                    match self.try_row(props[i], c.v[i], tag)? {
                        Res::Accepted(id) => seq.push(Some(id)),
                        Res::Crowd => {}
                        Res::Collide => seq.push(None),
                    }
                }
                // A thinned-out stretch of a closed chain stays connected; a skipped
                // (non-finite) proposal breaks an open one.
                split_pieces(seq, c.closed, &mut next);
            }
            if self.cdt.pts.len() == before {
                break;
            }
            chains = next;
        }
        Ok(())
    }

    // ---------------------------------------------------------------- fill

    /// Circumradius and circumcenter of triangle `t` in the metric frame of its first vertex.
    fn circum(&self, t: u32) -> Option<(f64, [f64; 2])> {
        let v = self.cdt.tv[t as usize];
        let b = self.met[v[0] as usize];
        let p: [[f64; 2]; 3] = core::array::from_fn(|k| self.cdt.pts[v[k] as usize]);
        let a = [0.0, 0.0];
        let bb = b.apply(sub(p[1], p[0]));
        let cc = b.apply(sub(p[2], p[0]));
        let d = 2.0 * (bb[0] * cc[1] - bb[1] * cc[0]);
        if d.abs() < 1e-300 {
            return None;
        }
        let (b2, c2) = (bb[0] * bb[0] + bb[1] * bb[1], cc[0] * cc[0] + cc[1] * cc[1]);
        let ux = (cc[1] * b2 - bb[1] * c2) / d;
        let uy = (bb[0] * c2 - cc[0] * b2) / d;
        let _ = a;
        let r = ux.hypot(uy);
        let off = b.unapply([ux, uy]);
        Some((r, [p[0][0] + off[0], p[0][1] + off[1]]))
    }

    /// Delaunay-refine until every triangle has circumradius at most `FILL_RATIO` sizes (or no
    /// admissible point is left).
    pub fn fill(&mut self) -> Result<(), PaveError> {
        for _round in 0..200 {
            let mut bad: Vec<(Ord64, u32, [u32; 3])> = Vec::new();
            for t in self.cdt.triangles() {
                let v = self.cdt.tv[t as usize];
                let h = (self.h[v[0] as usize] + self.h[v[1] as usize] + self.h[v[2] as usize]) / 3.0;
                if let Some((r, _)) = self.circum(t)
                    && r > FILL_RATIO * h
                {
                    bad.push((Ord64(-(r / h)), t, v));
                }
            }
            if bad.is_empty() {
                return Ok(());
            }
            bad.sort_unstable_by(|a, b| a.0.cmp(&b.0).then(a.1.cmp(&b.1)));
            let mut changed = false;
            for (_, t, snap) in bad {
                if !self.cdt.is_live(t) || self.cdt.tv[t as usize] != snap {
                    continue;
                }
                let v = snap;
                let h = (self.h[v[0] as usize] + self.h[v[1] as usize] + self.h[v[2] as usize]) / 3.0;
                let b = self.met[v[0] as usize];
                let centroid = {
                    let p: [[f64; 2]; 3] = core::array::from_fn(|k| self.cdt.pts[v[k] as usize]);
                    [(p[0][0] + p[1][0] + p[2][0]) / 3.0, (p[0][1] + p[1][1] + p[2][1]) / 3.0]
                };
                let mut cands: Vec<[f64; 2]> = Vec::with_capacity(2);
                if let Some((_, cc)) = self.circum(t)
                    && b.len(sub(cc, centroid)) <= 1.2 * h
                {
                    cands.push(cc);
                }
                cands.push(centroid);
                for p in cands {
                    let loc = self.cdt.locate(p, t);
                    match loc {
                        Loc::In(_) => {}
                        Loc::OnEdge(tt, i) if !self.cdt.tc[tt as usize][i as usize] && self.cdt.tn[tt as usize][i as usize] != NONE => {}
                        _ => continue,
                    }
                    let seeds = self.cdt.seeds(loc);
                    let cav = self.cdt.cavity(p, &seeds);
                    let lim = ALPHA_FILL * h;
                    let near = cav.iter().any(|ct| self.cdt.tv[*ct as usize].iter().any(|x| b.len(sub(p, self.cdt.pts[*x as usize])) < lim));
                    if near {
                        continue;
                    }
                    self.check_budget()?;
                    self.insert_checked(p, loc, u64::MAX)?;
                    changed = true;
                    break;
                }
            }
            if !changed {
                return Ok(());
            }
        }
        Ok(())
    }
}

/// Turn the accept/break sequence of one chain into the chains of the next row.
fn split_pieces(seq: Vec<Option<u32>>, closed: bool, out: &mut Vec<Chain>) {
    if seq.iter().all(|x| x.is_some()) {
        let v: Vec<u32> = seq.into_iter().flatten().collect();
        if closed && v.len() >= 3 {
            out.push(Chain { v, closed: true });
        } else if v.len() >= 2 {
            out.push(Chain { v, closed: false });
        }
        return;
    }
    let (first_some, last_some) = (seq.first().is_some_and(|x| x.is_some()), seq.last().is_some_and(|x| x.is_some()));
    let mut pieces: Vec<Vec<u32>> = vec![Vec::new()];
    for x in seq {
        match x {
            Some(id) => pieces.last_mut().map(|p| p.push(id)).unwrap_or(()),
            None => {
                if !pieces.last().is_some_and(|p| p.is_empty()) {
                    pieces.push(Vec::new());
                }
            }
        }
    }
    if pieces.last().is_some_and(|p| p.is_empty()) {
        pieces.pop();
    }
    if closed && first_some && last_some && pieces.len() > 1 {
        let mut last = pieces.pop().unwrap_or_default();
        last.extend(pieces.remove(0));
        pieces.insert(0, last);
    }
    for p in pieces {
        if p.len() >= 2 {
            out.push(Chain { v: p, closed: false });
        }
    }
}
