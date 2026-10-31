//! Ruling S4 for recovery points ([`Dt::relocate_for_s4`]): an interior point that boundary
//! recovery left closer than `0.3 h_T` to an input triangle `T` (a point that starred a side of a
//! suppressed boundary point, a kernel point of a thin cavity) is removed or moved deeper.
//!
//! This runs after classification, where every constraint face is a face of the mesh hull: a
//! cavity of interior tets never crosses one, and a refill that keeps the cavity's boundary
//! faces keeps every boundary face and edge. For a *target* (a nearby point at least
//! `0.3 h_T` from every input triangle) the cavity is grown from the point's star across every
//! interior face that does not see the target strictly, never enclosing a vertex other than an
//! interior point (those are dropped), and refilled by the star of the target when it sees every
//! face, or by exact gift wrapping with the target available as an apex; the refill with the
//! largest smallest dihedral angle over the targets wins. If no target works, the star is grown
//! by whole layers and refilled without a point or starred from its deepest kernel point (a
//! shallower one only if it raises the margin without lowering the worst mean ratio, revisited
//! by the next sweep). Sequential, in index order, every new tet validated exactly.

use crate::delaunay::Dt;
use crate::error::TetError;
use crate::geom::{V3, add, cross, dist, dot, mean_ratio, ori, sanitize, scale, sub};
use crate::guard::{Guard, S4};
use crate::quality::tet_quality;
use crate::spacing::{MIN_DIHEDRAL_DEG, Meter};
use crate::tds::NONE;
use std::collections::BTreeMap;

/// Fill work (see [`Dt::work`]) the calls of [`Dt::relocate_for_s4`] of one build may spend
/// together; the eleven hard relief cases of the tests use at most about 65 million units.
pub(crate) const RELOC_WORK: u64 = 80_000_000;
/// Sweeps of [`Dt::relocate_for_s4`] over the points.
const SWEEPS: usize = 3;
/// Targets tried per point.
const MAX_TARGETS: usize = 16;
/// Layers of tets around the star whose centroids are target candidates.
const TARGET_LAYERS: usize = 3;
/// Base budget (candidate tets) of a gift wrap towards a target.
const WRAP_BUDGET: usize = 10_000;
/// Gift wraps tried per point (targets nearest first).
const MAX_WRAPS: usize = 16;
/// Growth rounds of the fallback cavity around a point.
const GROW_ROUNDS: usize = 5;
/// Cavities stop growing beyond this many tets.
const MAX_TETS: usize = 400;
/// Cavities grown towards a target stop beyond this many tets (larger ones rarely refill).
const MAX_VISIBLE_TETS: usize = 100;

/// A refill: its smallest dihedral angle, the old tets (sorted), the new tets (`NEW` for the
/// target) and the target.
type Refill = (f64, Vec<u32>, Vec<[u32; 4]>, V3);

/// Mean ratio from which a kernel point counts as adequate (the spacing objective then decides).
const ADEQUATE_MEAN_RATIO: f64 = 0.1;

/// Is refill `a` (smallest dihedral, target) better than `b`? Objective (INT-7): among refills
/// whose smallest dihedral is above the floor, the target whose spacing `r` is nearest to the
/// goal wins; below the floor the larger smallest dihedral wins. Ties keep `b` (first found).
fn better(meter: &Meter<'_, '_>, a: (f64, V3), b: (f64, V3)) -> bool {
    let (a_ok, b_ok) = (a.0 >= MIN_DIHEDRAL_DEG, b.0 >= MIN_DIHEDRAL_DEG);
    match (a_ok, b_ok) {
        (true, true) => meter.deviation(a.1) < meter.deviation(b.1),
        (false, false) => a.0 > b.0,
        (x, _) => x,
    }
}

/// A fallback star: its margin, the cavity (sorted), its faces and the point.
type Fallback = (f64, Vec<u32>, Vec<[u32; 3]>, V3);

/// Placeholder id of the target in a [`Refill`].
const NEW: u32 = u32::MAX - 1;

/// Smallest dihedral angle (degrees) of `tets`, `NEW` standing for `t`.
fn min_dihedral(pts: &[V3], tets: &[[u32; 4]], t: V3) -> f64 {
    tets.iter()
        .map(|w| tet_quality(w.map(|i| if i == NEW { t } else { pts[i as usize] })).min_dihedral)
        .fold(f64::INFINITY, f64::min)
}

impl Dt {
    /// Remove or move the interior points (index `from..`) that violate ruling S4 (see the module
    /// docs), spending at most `budget` units of fill work (the rest is left in `budget`; a
    /// point is not started once it is used up). Returns the number of points removed or moved.
    pub(crate) fn relocate_for_s4(&mut self, from: usize, guard: &Guard<'_>, meter: &Meter<'_, '_>, budget: &mut u64) -> Result<usize, TetError> {
        let start = self.work.load(std::sync::atomic::Ordering::Relaxed);
        let r = self.relocate_within(from, guard, meter, start.saturating_add(*budget));
        *budget = budget.saturating_sub(self.work.load(std::sync::atomic::Ordering::Relaxed).saturating_sub(start));
        r
    }

    /// [`Dt::relocate_for_s4`] until the fill work counter reaches `limit`.
    fn relocate_within(&mut self, from: usize, guard: &Guard<'_>, meter: &Meter<'_, '_>, limit: u64) -> Result<usize, TetError> {
        let mut done = 0;
        let mut star = Vec::new();
        // Points that failed, with the vertices of their star then: retried only if it changed.
        let mut failed: BTreeMap<u32, Vec<[u32; 4]>> = BTreeMap::new();
        // Points added by a sweep (deeper, possibly still too shallow) are revisited by the next.
        for _sweep in 0..SWEEPS {
            let to = self.pts.len();
            let mut changed = false;
            for v in from..to {
                if guard.margin(self.pts[v]) >= S4 {
                    continue;
                }
                if self.work.load(std::sync::atomic::Ordering::Relaxed) >= limit {
                    return Ok(done);
                }
                self.tds.star(v as u32, &mut star);
                if star.is_empty() {
                    continue;
                }
                let mut sig: Vec<[u32; 4]> = star.iter().map(|&t| self.tds.verts[t as usize]).collect();
                sig.sort_unstable();
                if failed.get(&(v as u32)) == Some(&sig) {
                    continue;
                }
                if self.relocate_one(v as u32, &star, from as u32, guard, meter)? {
                    done += 1;
                    changed = true;
                } else {
                    failed.insert(v as u32, sig);
                }
            }
            if !changed {
                break;
            }
        }
        Ok(done)
    }

    /// Remove or move interior point `v` (star `star`); `true` if the mesh changed.
    fn relocate_one(&mut self, v: u32, star: &[u32], from: u32, guard: &Guard<'_>, meter: &Meter<'_, '_>) -> Result<bool, TetError> {
        let p = self.pts[v as usize];
        let targets = self.s4_targets(p, star, guard.nearest(p), guard, meter);
        if let Some(r) = self.best_refill(star, &targets, from, meter) {
            self.apply_refill(r)?;
            return Ok(true);
        }
        // Fallback: grow the star by whole layers; refill without a point, or star from the
        // deepest visible kernel candidate if it complies, else remember the deepest star that
        // raises the margin without lowering the cavity's worst mean ratio.
        let mut cav: Vec<u32> = star.to_vec();
        cav.sort_unstable();
        cav.dedup();
        let mut best: Option<Fallback> = None;
        for _round in 0..=GROW_ROUNDS {
            let (faces, outside) = self.cavity_faces(&cav);
            let Some(enclosed) = self.enclosed(&cav, &faces, from) else { break };
            let floor = enclosed.iter().map(|&x| guard.margin(self.pts[x as usize])).fold(S4, f64::min);
            let q_old = cav
                .iter()
                .map(|&t| {
                    let w = self.tds.verts[t as usize].map(|x| self.pts[x as usize]);
                    mean_ratio(w[0], w[1], w[2], w[3])
                })
                .fold(f64::INFINITY, f64::min);
            if let Some(tets) = self.fill_budgeted(&faces, None, &[], 2000 + 40 * faces.len()) {
                let mut ids = Vec::new();
                self.tds.replace(&cav, &tets, &mut ids)?;
                return Ok(true);
            }
            if let Some((m, q, x)) = self.deepest_star_point(&faces, guard, meter) {
                if m >= S4 {
                    self.star_from(&cav, &faces, x)?;
                    return Ok(true);
                }
                if m > floor && q >= q_old && best.as_ref().is_none_or(|b| m > b.0) {
                    best = Some((m, cav.clone(), faces.clone(), x));
                }
            }
            if cav.len() > MAX_TETS || outside.is_empty() {
                break;
            }
            cav.extend(outside);
            cav.sort_unstable();
            cav.dedup();
        }
        if let Some((_, cav, faces, x)) = best {
            self.star_from(&cav, &faces, x)?;
            return Ok(true);
        }
        Ok(false)
    }

    /// Boundary faces (outward) of the cavity `cav` (sorted) and the distinct tets beyond them.
    fn cavity_faces(&self, cav: &[u32]) -> (Vec<[u32; 3]>, Vec<u32>) {
        let mut faces: Vec<[u32; 3]> = Vec::new();
        let mut outside: Vec<u32> = Vec::new();
        for &t in cav {
            for k in 0..4 {
                let n = self.tds.nbr[t as usize][k];
                if n != NONE && cav.binary_search(&n).is_ok() {
                    continue;
                }
                faces.push(self.tds.face(t, k));
                if n != NONE {
                    outside.push(n);
                }
            }
        }
        outside.sort_unstable();
        outside.dedup();
        (faces, outside)
    }

    /// The vertices the cavity `cav` (boundary `faces`) encloses, if all are interior points
    /// (index `from..`) and no tet is a ghost.
    fn enclosed(&self, cav: &[u32], faces: &[[u32; 3]], from: u32) -> Option<Vec<u32>> {
        if cav.iter().any(|&t| self.tds.ghost_slot(t).is_some()) {
            return None;
        }
        let mut bverts: Vec<u32> = faces.iter().flatten().copied().collect();
        bverts.sort_unstable();
        bverts.dedup();
        let mut inner: Vec<u32> = cav.iter().flat_map(|&t| self.tds.verts[t as usize]).filter(|x| bverts.binary_search(x).is_err()).collect();
        inner.sort_unstable();
        inner.dedup();
        inner.iter().all(|&x| x >= from).then_some(inner)
    }

    /// Compliant targets near `p`: offsets along the inward normal of the input triangle
    /// `near` (with its `h`), and centroids of the tets within [`TARGET_LAYERS`] layers of
    /// `seed`; those with margin at least [`S4`], nearest to `p` first, at most
    /// [`MAX_TARGETS`].
    fn s4_targets(&self, p: V3, seed: &[u32], near: Option<([V3; 3], f64)>, guard: &Guard<'_>, meter: &Meter<'_, '_>) -> Vec<V3> {
        let mut seen: Vec<u32> = seed.to_vec();
        seen.sort_unstable();
        seen.dedup();
        let mut layer: Vec<u32> = seen.clone();
        for _ in 0..TARGET_LAYERS {
            let mut next: Vec<u32> = Vec::new();
            for &t in &layer {
                for k in 0..4 {
                    let n = self.tds.nbr[t as usize][k];
                    if n != NONE && seen.binary_search(&n).is_err() {
                        next.push(n);
                    }
                }
            }
            next.sort_unstable();
            next.dedup();
            seen.extend_from_slice(&next);
            seen.sort_unstable();
            layer = next;
        }
        seen.retain(|&t| self.tds.ghost_slot(t).is_none());
        // Offsets inward from the nearest input triangle and from the mesh boundary faces of the
        // seed. In a thin part an offset can leave the solid: such a target is never used as an
        // apex (no tet can reach it), but it still steers the growth of the cavity, whose refill
        // then needs no point.
        let mut offs: Vec<V3> = Vec::new();
        let push_offsets = |[a, b, c]: [V3; 3], foot: V3, h: f64, out: &mut Vec<V3>| {
            let n = cross(sub(b, a), sub(c, a));
            let nl = dot(n, n).sqrt();
            if nl > 0.0 && nl.is_finite() {
                let n = scale(n, 1.0 / nl);
                let foot = sub(foot, scale(n, dot(sub(foot, a), n)));
                for s in [0.35, 0.5, 0.75, 1.0] {
                    out.push(sub(foot, scale(n, s * h)));
                }
            }
        };
        if let Some((tri, h)) = near {
            push_offsets(tri, p, h, &mut offs);
        }
        for &t in seed {
            for k in 0..4 {
                if self.tds.nbr[t as usize][k] == NONE {
                    let [a, b, c] = self.tds.face(t, k).map(|i| self.pts[i as usize]);
                    let h = (dist(a, b) + dist(b, c) + dist(c, a)) / 3.0;
                    push_offsets([a, b, c], scale(add(add(a, b), c), 1.0 / 3.0), h, &mut offs);
                }
            }
        }
        let mut cands: Vec<V3> = offs;
        for &t in &seen {
            let w = self.tds.verts[t as usize].map(|x| self.pts[x as usize]);
            cands.push(scale(add(add(w[0], w[1]), add(w[2], w[3])), 0.25));
        }
        let mut ok: Vec<(f64, usize, V3)> = cands
            .into_iter()
            .enumerate()
            .filter_map(|(i, q)| sanitize(q).map(|q| (i, q)))
            .filter(|&(_, q)| guard.margin(q) >= S4)
            .map(|(i, q)| (dist(p, q), i, q))
            .collect();
        ok.sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
        // The nearest MAX_TARGETS stay candidates (they are the likeliest to be reachable); they
        // are tried in order of their spacing deviation (INT-7), nearest first on ties.
        let mut t: Vec<(f64, V3)> = ok.into_iter().take(MAX_TARGETS).map(|x| (meter.deviation(x.2), x.2)).collect();
        t.sort_by(|a, b| a.0.total_cmp(&b.0));
        t.into_iter().map(|x| x.1).collect()
    }

    /// The best refill of the cavities grown from `seed` towards each target (see the module
    /// docs): the one with the largest smallest dihedral angle (first on ties).
    fn best_refill(&mut self, seed: &[u32], targets: &[V3], from: u32, meter: &Meter<'_, '_>) -> Option<Refill> {
        let mut best: Option<Refill> = None;
        let mut grown: Vec<(Vec<u32>, Vec<[u32; 3]>, V3)> = Vec::new();
        // Stars of the targets that see every cavity face (cheap) first.
        for &t in targets {
            let Some((cav, faces)) = self.grow_visible(seed, t, from) else { continue };
            if faces.iter().all(|f| {
                let [a, b, c] = f.map(|i| self.pts[i as usize]);
                ori(a, b, c, t) < 0
            }) {
                let tets: Vec<[u32; 4]> = faces.iter().map(|f| [NEW, f[0], f[1], f[2]]).collect();
                let q = min_dihedral(&self.pts, &tets, t);
                if best.as_ref().is_none_or(|b| better(meter, (q, t), (b.0, b.3))) {
                    best = Some((q, cav, tets, t));
                }
            } else {
                grown.push((cav, faces, t));
            }
        }
        if best.is_some() {
            return best;
        }
        // Else gift wrapping with the target as an optional apex, smallest cavity first (stable:
        // nearest target first on ties).
        grown.sort_by_key(|g| g.0.len());
        for (cav, faces, t) in grown.into_iter().take(MAX_WRAPS) {
            let id = self.push_point(t);
            let fill = self.fill_budgeted(&faces, None, &[id], WRAP_BUDGET + 100 * faces.len());
            self.truncate_points(id as usize);
            let Some(mut tets) = fill else { continue };
            for w in &mut tets {
                for x in w.iter_mut() {
                    if *x == id {
                        *x = NEW;
                    }
                }
            }
            // The first refill found (targets nearest first): gift wrapping is the costly part.
            return Some((min_dihedral(&self.pts, &tets, t), cav, tets, t));
        }
        None
    }

    /// Commit a [`Refill`], adding its target if a new tet uses it.
    fn apply_refill(&mut self, r: Refill) -> Result<(), TetError> {
        let (_, cav, mut tets, t) = r;
        if tets.iter().any(|w| w.contains(&NEW)) {
            let id = self.push_point(t);
            for w in &mut tets {
                for x in w.iter_mut() {
                    if *x == NEW {
                        *x = id;
                    }
                }
            }
        }
        let mut ids = Vec::new();
        self.tds.replace(&cav, &tets, &mut ids)
    }

    /// Grow the cavity `seed` across every interior boundary face that does not see `t`
    /// strictly until all do (faces on the mesh boundary stay and are closed by other apexes);
    /// `None` if that encloses a vertex other than an interior point (index `from..`) or needs
    /// more than [`MAX_VISIBLE_TETS`] tets. Returns the cavity (sorted) and its boundary faces
    /// (outward).
    fn grow_visible(&self, seed: &[u32], t: V3, from: u32) -> Option<(Vec<u32>, Vec<[u32; 3]>)> {
        let mut cav: Vec<u32> = seed.to_vec();
        cav.sort_unstable();
        cav.dedup();
        loop {
            let mut faces: Vec<[u32; 3]> = Vec::new();
            let mut add: Vec<u32> = Vec::new();
            for &c in &cav {
                for k in 0..4 {
                    let n = self.tds.nbr[c as usize][k];
                    if n != NONE && cav.binary_search(&n).is_ok() {
                        continue;
                    }
                    let f = self.tds.face(c, k);
                    let [a, b, d] = f.map(|i| self.pts[i as usize]);
                    if n == NONE || ori(a, b, d, t) < 0 {
                        faces.push(f);
                    } else {
                        add.push(n);
                    }
                }
            }
            if add.is_empty() {
                self.enclosed(&cav, &faces, from)?;
                return Some((cav, faces));
            }
            if cav.len() + add.len() > MAX_VISIBLE_TETS {
                return None;
            }
            cav.extend(add);
            cav.sort_unstable();
            cav.dedup();
        }
    }

    /// The visible kernel candidate of the polyhedron `faces` with the largest S4 margin (capped
    /// at [`S4`]), ties broken by the worst mean ratio of its star: `(margin, quality, point)`.
    fn deepest_star_point(&self, faces: &[[u32; 3]], guard: &Guard<'_>, meter: &Meter<'_, '_>) -> Option<(f64, f64, V3)> {
        let mut best: Option<(f64, f64, V3)> = None;
        for q in self.kernel_candidates(faces) {
            let mut worst = f64::INFINITY;
            let mut ok = true;
            for f in faces {
                let [a, b, c] = f.map(|i| self.pts[i as usize]);
                if ori(a, b, c, q) >= 0 {
                    ok = false;
                    break;
                }
                worst = worst.min(mean_ratio(q, a, b, c));
            }
            if !ok {
                continue;
            }
            let m = guard.margin(q).min(S4);
            let wins = |b: (f64, f64, V3)| {
                if m != b.0 {
                    return m > b.0;
                }
                // Equal margin: among adequate shapes the spacing objective decides, else shape.
                if worst >= ADEQUATE_MEAN_RATIO && b.1 >= ADEQUATE_MEAN_RATIO {
                    meter.deviation(q) < meter.deviation(b.2)
                } else {
                    worst > b.1
                }
            };
            if best.is_none_or(wins) {
                best = Some((m, worst, q));
            }
        }
        best
    }

    /// Replace the tets `cav` by the star of a new point `p` over the cavity faces `faces`.
    fn star_from(&mut self, cav: &[u32], faces: &[[u32; 3]], p: V3) -> Result<(), TetError> {
        let id = self.push_point(p);
        let tets: Vec<[u32; 4]> = faces.iter().map(|f| [id, f[0], f[1], f[2]]).collect();
        let mut ids = Vec::new();
        self.tds.replace(cav, &tets, &mut ids)
    }
}
