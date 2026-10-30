//! Complete boundary recovery: temporary boundary Steiner points and their suppression.
//!
//! Hang Si's surface-preserving recovery (TetGen `-Y`; Si & Gärtner, "3D boundary recovery by
//! constrained Delaunay tetrahedralization", IJNME 2011; Si, "TetGen, a Delaunay-based quality
//! tetrahedral mesh generator", ACM TOMS 2015), on exact predicates. It runs when the
//! interior-point recovery of [`crate::recover`] is stuck, on the triangulation it left:
//!
//! 1. **Segments.** A missing input edge is recovered by flips; where they are stuck, a point is
//!    inserted *on* the edge where it leaves the first tet of its corridor (a 2-6 face split,
//!    else an n-2n edge split, else a 1-4 split inside that tet, each validated exactly), so the
//!    first piece is an edge at once and the rest is retried. Each split ends the corridor one
//!    face earlier, so an edge needs at most as many points as faces it crosses.
//! 2. **Facets.** A missing input triangle (a polygon now, with the points of its edges) is
//!    recovered by the cavity refill of [`crate::recover`] when its edges are whole; otherwise,
//!    or if that fails, every tet edge crossing it is removed by a flip that creates no new
//!    crossing, or else split where it crosses (each split removes one crossing edge and creates
//!    none), after which the triangle is tiled by mesh faces whose vertices all lie on it; the
//!    tiling is read off and checked. Nothing here gives up a recovered edge or tile.
//! 3. **Suppression.** Every boundary Steiner point is removed from the surface again: its fans
//!    in the surface are re-triangulated without it, which cuts its star into a solid side and an
//!    exterior side. Each side is refilled without a point, or starred from a point in its
//!    kernel next to the old position (one exists there: the old point sees every link face
//!    strictly, and the new fans are planar up to the rounding of boundary points not yet
//!    suppressed; every tet is validated exactly; a side whose kernel is hidden by a sliver
//!    lying on the surface absorbs the tets behind it). Solid-side points are interior Steiner
//!    points; exterior ones are dropped with the exterior. When all are gone the input
//!    triangles are faces again, bit-exactly.
//!
//! Nothing here depends on the order of a hash map; all loops run over sorted keys.

use crate::error::TetError;
use crate::geom::{V3, add, cross, dist, dot, mean_ratio, ori, sanitize, scale, sub};
use crate::recover::{Constraints, Recover};
use crate::tds::{GHOST, same_cycle, sorted3};
use std::collections::{BTreeMap, BTreeSet};

/// Splits per segment recovery.
const MAX_SPLITS: usize = 4096;
/// Flips and splits of crossing edges per input triangle.
const MAX_FACE_STEPS: usize = 8192;
/// Sweeps over the boundary Steiner points during suppression.
const MAX_SWEEPS: usize = 8;
/// Growth rounds of one side cavity of a suppressed point.
const SIDE_GROW: usize = 6;
/// Base budget (candidate tets) of a refill of a side without a point; a point star is the
/// cheap alternative, so the search is kept short.
const SIDE_FILL_BUDGET: usize = 200;
/// Side cavities refilled without a point have at most this many tets.
const MAX_FILL_TETS: usize = 64;
/// Side cavities stop growing beyond this many tets.
const MAX_SIDE_TETS: usize = 400;
/// The old tets (sorted) and the new tets of one side of a suppressed point.
type SideFill = (Vec<u32>, Vec<[u32; 4]>);
/// How one side cavity can be closed (see [`Recover::fill_side`]).
enum SideOutcome {
    /// Refilled without a point.
    Filled(Vec<[u32; 4]>),
    /// Starred from this point (not yet added).
    Star(V3),
    /// No candidate sees every face: the faces that hide the candidate with the fewest of them.
    Blocked(Vec<[u32; 3]>),
}
/// Placeholder vertex id for the point being inserted.
const NEW: u32 = u32::MAX - 1;

/// Where a temporary boundary Steiner point lies.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum On {
    /// On input edge `i` (index into the input [`Constraints::edges`]).
    Edge(u32),
    /// Inside input triangle `t`.
    Face(u32),
}

/// The surface during complete recovery.
#[derive(Default)]
pub(crate) struct Bnd {
    /// Tiles of the recovered input triangles: triangle (oriented like its input triangle) and
    /// the input triangle.
    pub subs: Vec<([u32; 3], u32)>,
    /// Points of each input edge, from the first to the second endpoint of its sorted key.
    chains: Vec<Vec<u32>>,
    /// Subsegment (sorted endpoints) and the input edge it lies on.
    pub segs: BTreeMap<[u32; 2], u32>,
    /// Temporary boundary Steiner points.
    pub on: BTreeMap<u32, On>,
    /// Steiner points inside each input triangle.
    face_pts: BTreeMap<u32, Vec<u32>>,
    /// Input triangles tiled so far.
    done: Vec<bool>,
    /// Tiles by sorted key: owner (input triangle) -> multiplicity.
    tile_faces: BTreeMap<[u32; 3], BTreeMap<u32, u32>>,
    /// Edges of the tiles: owner -> multiplicity.
    tile_edges: BTreeMap<[u32; 2], BTreeMap<u32, u32>>,
    /// Constraint-table entries inserted or removed: the work measure of the incremental
    /// constraint lookup (each tile, subsegment or split costs O(1) entries, never a rebuild).
    pub work: u64,
}

/// Add one `owner` to a multiset entry.
fn ms_add<K: Ord>(m: &mut BTreeMap<K, BTreeMap<u32, u32>>, k: K, owner: u32) {
    *m.entry(k).or_default().entry(owner).or_default() += 1;
}

/// Remove one `owner` from a multiset entry (empty entries are dropped).
fn ms_remove<K: Ord>(m: &mut BTreeMap<K, BTreeMap<u32, u32>>, k: K, owner: u32) {
    if let Some(o) = m.get_mut(&k) {
        if let Some(c) = o.get_mut(&owner) {
            *c -= 1;
            if *c == 0 {
                o.remove(&owner);
            }
        }
        if o.is_empty() {
            m.remove(&k);
        }
    }
}

impl Bnd {
    fn new(orig: &Constraints) -> Self {
        let chains = orig.edges.iter().map(|e| e.0.to_vec()).collect();
        let segs = orig.edges.iter().enumerate().map(|(i, e)| (e.0, i as u32)).collect();
        let work = orig.edges.len() as u64;
        Self { subs: Vec::new(), chains, segs, on: BTreeMap::new(), face_pts: BTreeMap::new(), done: vec![false; orig.tris.len()], tile_faces: BTreeMap::new(), tile_edges: BTreeMap::new(), work }
    }

    /// Owner of constraint edge `(a,b)`: the smallest of the owner of the input edge whose
    /// subsegment it is and the owners of the tiles that have it.
    pub(crate) fn edge_owner(&self, orig: &Constraints, a: u32, b: u32) -> Option<u32> {
        let k = key2(a, b);
        let seg = self.segs.get(&k).and_then(|&e| orig.edges.get(e as usize)).map(|e| e.1);
        let tile = self.tile_edges.get(&k).and_then(|o| o.keys().next().copied());
        match (seg, tile) {
            (Some(x), Some(y)) => Some(x.min(y)),
            (x, y) => x.or(y),
        }
    }

    /// Owner of tile `k` (sorted key), the smallest if several.
    pub(crate) fn face_owner(&self, k: [u32; 3]) -> Option<u32> {
        self.tile_faces.get(&k).and_then(|o| o.keys().next().copied())
    }

    /// Add tile `t` of input triangle `owner` to the surface and the lookup.
    fn add_tile(&mut self, t: [u32; 3], owner: u32) {
        self.index_tile(t, owner, true);
        self.subs.push((t, owner));
    }

    /// Insert (`add`) or remove the lookup entries of tile `t`.
    fn index_tile(&mut self, t: [u32; 3], owner: u32, add: bool) {
        let f = if add { ms_add::<[u32; 3]> } else { ms_remove::<[u32; 3]> };
        f(&mut self.tile_faces, sorted3(t[0], t[1], t[2]), owner);
        let e = if add { ms_add::<[u32; 2]> } else { ms_remove::<[u32; 2]> };
        for r in 0..3 {
            e(&mut self.tile_edges, key2(t[r], t[(r + 1) % 3]), owner);
        }
        self.work += 4;
    }

    /// Replace subsegment `old` by the subsegments `new` of input edge `e`.
    fn replace_seg(&mut self, old: &[[u32; 2]], new: &[[u32; 2]], e: u32) {
        for k in old {
            self.segs.remove(k);
        }
        for &k in new {
            self.segs.insert(k, e);
        }
        self.work += (old.len() + new.len()) as u64;
    }
}

#[inline]
fn key2(a: u32, b: u32) -> [u32; 2] {
    [a.min(b), a.max(b)]
}

fn fail(tri: u32, edge: Option<[u32; 2]>, reason: impl Into<String>) -> TetError {
    TetError::RecoveryFailed { tri, edge, reason: reason.into() }
}

/// Intersection of the line through `p, q` with the plane of `a, b, c` (floating point).
fn line_plane(p: V3, q: V3, a: V3, b: V3, c: V3) -> Option<V3> {
    let n = cross(sub(b, a), sub(c, a));
    let den = dot(n, sub(q, p));
    if den == 0.0 || !den.is_finite() {
        return None;
    }
    let t = dot(n, sub(a, p)) / den;
    sanitize(add(p, scale(sub(q, p), t)))
}

/// Point of segment `a-b` closest to the line through `p, q`, and its distance to that line.
fn edge_line_closest(a: V3, b: V3, p: V3, q: V3) -> Option<(V3, f64)> {
    let (d1, d2, r) = (sub(b, a), sub(q, p), sub(a, p));
    let (aa, bb, ab) = (dot(d1, d1), dot(d2, d2), dot(d1, d2));
    let (c1, c2) = (dot(d1, r), dot(d2, r));
    let den = aa * bb - ab * ab;
    if den.is_nan() || den <= 0.0 || bb == 0.0 {
        return None;
    }
    let s = ((ab * c2 - bb * c1) / den).clamp(0.0, 1.0);
    let x = add(a, scale(d1, s));
    let t = dot(sub(x, p), d2) / bb;
    let y = add(p, scale(d2, t));
    Some((sanitize(x)?, dist(x, y)))
}

/// Unit normal of triangle `t` (floating point).
fn unit_normal(pts: &[V3], t: [u32; 3]) -> Option<V3> {
    let [a, b, c] = t.map(|i| pts[i as usize]);
    let n = cross(sub(b, a), sub(c, a));
    let l = dot(n, n).sqrt();
    (l > 0.0 && l.is_finite()).then(|| scale(n, 1.0 / l))
}

/// Smallest angle (radians) of a 2-D triangle.
fn min_angle_2d(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    let ang = |p: [f64; 2], q: [f64; 2], r: [f64; 2]| {
        let (u, v) = ([q[0] - p[0], q[1] - p[1]], [r[0] - p[0], r[1] - p[1]]);
        let cr = u[0] * v[1] - u[1] * v[0];
        let dt = u[0] * v[0] + u[1] * v[1];
        cr.abs().atan2(dt)
    };
    ang(a, b, c).min(ang(b, c, a)).min(ang(c, a, b))
}

/// Triangulate the simple polygon `poly` (vertex ids, counter-clockwise seen from `normal`) by
/// ear clipping with exact 2-D orientations in the projection along the dominant axis of
/// `normal`; the ear with the largest smallest angle goes first (ties: lowest position).
/// Triangles keep the polygon's orientation.
fn triangulate_polygon(pts: &[V3], poly: &[u32], normal: V3) -> Option<Vec<[u32; 3]>> {
    let k = (0..3).max_by(|&i, &j| normal[i].abs().total_cmp(&normal[j].abs()).then(j.cmp(&i)))?;
    let sgn = if normal[k] > 0.0 { 1 } else { -1 };
    let (i1, i2) = ((k + 1) % 3, (k + 2) % 3);
    let q = |v: u32| [pts[v as usize][i1], pts[v as usize][i2]];
    let o2 = |a: u32, b: u32, c: u32| sgn * sm_math::orient2d(q(a), q(b), q(c)).as_i32();
    let mut ring: Vec<u32> = poly.to_vec();
    let mut out = Vec::with_capacity(poly.len().saturating_sub(2));
    while ring.len() > 3 {
        let m = ring.len();
        let mut best: Option<(f64, usize)> = None;
        for i in 0..m {
            let (a, b, c) = (ring[(i + m - 1) % m], ring[i], ring[(i + 1) % m]);
            if o2(a, b, c) <= 0 {
                continue;
            }
            let blocked = ring.iter().any(|&v| v != a && v != b && v != c && o2(a, b, v) >= 0 && o2(b, c, v) >= 0 && o2(c, a, v) >= 0);
            if blocked {
                continue;
            }
            let score = min_angle_2d(q(a), q(b), q(c));
            if best.is_none_or(|bb| score > bb.0) {
                best = Some((score, i));
            }
        }
        let (_, i) = best?;
        let m = ring.len();
        out.push([ring[(i + m - 1) % m], ring[i], ring[(i + 1) % m]]);
        ring.remove(i);
    }
    if ring.len() == 3 {
        if o2(ring[0], ring[1], ring[2]) <= 0 {
            return None;
        }
        out.push([ring[0], ring[1], ring[2]]);
    }
    Some(out)
}

/// Backtracking search for a tiling of a triangle by candidate faces (see `read_tiling`).
struct TileSearch<'a> {
    cand: &'a BTreeMap<(u32, u32), Vec<[u32; 3]>>,
    bnd: &'a BTreeSet<(u32, u32)>,
    vset: &'a [u32],
    covered: BTreeSet<(u32, u32)>,
    /// Directed edges that must be covered: the boundary pieces and the reverses of covered
    /// interior edges.
    need: BTreeSet<(u32, u32)>,
    tiles: Vec<[u32; 3]>,
    budget: usize,
}

impl TileSearch<'_> {
    fn dfs(&mut self) -> bool {
        let Some(&e) = self.need.iter().find(|e| !self.covered.contains(e)) else {
            return self.vset.iter().all(|v| self.tiles.iter().any(|t| t.contains(v)));
        };
        let Some(cands) = self.cand.get(&e) else { return false };
        for &tile in cands {
            if self.budget == 0 {
                return false;
            }
            self.budget -= 1;
            let des: [(u32, u32); 3] = [(tile[0], tile[1]), (tile[1], tile[2]), (tile[2], tile[0])];
            if des.iter().any(|de| self.covered.contains(de) || self.bnd.contains(&(de.1, de.0))) {
                continue;
            }
            let mut added: Vec<(u32, u32)> = Vec::new();
            for de in des {
                self.covered.insert(de);
                if !self.bnd.contains(&de) && self.need.insert((de.1, de.0)) {
                    added.push((de.1, de.0));
                }
            }
            self.tiles.push(tile);
            if self.dfs() {
                return true;
            }
            self.tiles.pop();
            for de in des {
                self.covered.remove(&de);
            }
            for a in added {
                self.need.remove(&a);
            }
        }
        false
    }
}

impl Recover<'_> {
    /// Recover every constraint, using temporary boundary Steiner points that are suppressed
    /// afterwards (see the module docs). Continues from the current triangulation.
    pub fn complete(&mut self) -> Result<(), TetError> {
        self.complete_mode = true;
        self.bnd = Bnd::new(self.orig);
        let r = self.complete_inner();
        let r = r.map_err(|e| self.map_err(e));
        self.complete_mode = false;
        for p in self.bnd.on.keys() {
            if let Some(x) = self.dt.pinned.get_mut(*p as usize) {
                *x = false;
            }
        }
        r
    }

    /// Replace Steiner vertices in the edge of an error by the input edge they lie on.
    fn map_err(&self, e: TetError) -> TetError {
        let n = self.n_real;
        let edge_in = |e: Option<[u32; 2]>| -> Option<[u32; 2]> {
            let e = e?;
            if e[0] < n && e[1] < n {
                return Some(e);
            }
            let i = *self.bnd.segs.get(&key2(e[0], e[1]))?;
            self.orig.edges.get(i as usize).map(|x| x.0)
        };
        match e {
            TetError::RecoveryFailed { tri, edge, reason } => TetError::RecoveryFailed { tri, edge: edge_in(edge), reason },
            other => other,
        }
    }

    fn complete_inner(&mut self) -> Result<(), TetError> {
        // Segments.
        for e in 0..self.orig.edges.len() {
            let ([a, b], owner) = self.orig.edges[e];
            if self.dt.tds.find_edge(a, b).is_none() {
                self.recover_seg(a, b, owner)?;
            }
        }
        // Facets.
        for tri in 0..self.orig.tris.len() as u32 {
            self.recover_facet(tri)?;
        }
        // Everything recovered must still be there.
        let segs: Vec<[u32; 2]> = self.bnd.segs.keys().copied().collect();
        for k in segs {
            if self.dt.tds.find_edge(k[0], k[1]).is_none() {
                let e = self.bnd.segs[&k];
                return Err(fail(self.orig.edges[e as usize].1, Some(k), "a recovered edge piece was lost"));
            }
        }
        let subs = self.bnd.subs.clone();
        for (t, tri) in subs {
            if !self.face_present(sorted3(t[0], t[1], t[2])) {
                return Err(fail(tri, None, "a recovered triangle piece was lost"));
            }
        }
        self.suppress_all()?;
        // Back to the input triangles: they must all be faces now.
        for &(k, tri) in &self.orig.faces {
            if !self.face_present(k) {
                return Err(fail(tri, None, "triangle missing after Steiner point suppression"));
            }
        }
        Ok(())
    }

    // ------------------------------------------------------------------------------- segments

    /// Recover constraint edge `(u0,w)`: flips, else split it where it leaves the first tet of
    /// its corridor and continue with the remaining piece.
    pub(crate) fn recover_seg(&mut self, u0: u32, w: u32, owner: u32) -> Result<(), TetError> {
        let (mut u, mut w) = (u0, w);
        for _ in 0..MAX_SPLITS {
            if self.dt.tds.find_edge(u, w).is_some() {
                return Ok(());
            }
            if self.edge_by_flips(u, w, owner)? {
                return Ok(());
            }
            // Split from `u`; if every point there is degenerate, from `w`.
            match self.split_seg_step(u, w, owner) {
                Ok(p) => u = p,
                Err(TetError::RecoveryFailed { .. }) => w = self.split_seg_step(w, u, owner)?,
                Err(e) => return Err(e),
            }
        }
        Err(fail(owner, Some([u0, w]), "segment recovery did not terminate"))
    }

    /// Insert one Steiner point on segment `(u,w)` such that `(u,p)` is an edge; returns `p`.
    fn split_seg_step(&mut self, u: u32, w: u32, owner: u32) -> Result<u32, TetError> {
        let Some((_, faces)) = self.corridor(u, w) else {
            return Err(fail(owner, Some([u, w]), "could not trace the segment through the triangulation"));
        };
        let Some(&(t0, k0)) = faces.first() else {
            return Err(fail(owner, Some([u, w]), "segment corridor without a crossed face"));
        };
        let f1 = self.dt.tds.face(t0, k0);
        let t1 = self.dt.tds.nbr[t0 as usize][k0];
        if let Some(other) = self.face_owner(sorted3(f1[0], f1[1], f1[2])) {
            if self.edge_hits_face(u, w, f1) {
                return Err(TetError::SelfIntersection { tri_a: owner.min(other), tri_b: owner.max(other) });
            }
            return Err(fail(owner, Some([u, w]), "segment runs into a boundary face"));
        }
        let (pu, pw) = (self.dt.pts[u as usize], self.dt.pts[w as usize]);
        let fp = f1.map(|i| self.dt.pts[i as usize]);
        let x = line_plane(pu, pw, fp[0], fp[1], fp[2]);
        // (A) on the crossed face.
        if let Some(x) = x
            && let Some(p) = self.split_simplex(&[t0, t1], &f1, x)?
        {
            return Ok(self.on_seg_split(u, w, p));
        }
        // (B) on an edge of the crossed face the segment passes (almost) through.
        let len = dist(pu, pw);
        let mut es: Vec<(f64, [u32; 2], V3)> = Vec::new();
        for e in 0..3 {
            let (a, b) = (f1[e], f1[(e + 1) % 3]);
            if self.edge_owner(a, b).is_some() {
                continue;
            }
            if let Some((p, d)) = edge_line_closest(self.dt.pts[a as usize], self.dt.pts[b as usize], pu, pw)
                && d <= 1e-8 * len
            {
                es.push((d, key2(a, b), p));
            }
        }
        es.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        for (_, e, p) in es {
            let mut ring = Vec::new();
            let mut poly = Vec::new();
            if self.dt.tds.ring(e[0], e[1], t0, 512, &mut ring, &mut poly)
                && let Some(q) = self.split_edge_near(&ring, e[0], e[1], p)?
            {
                return Ok(self.on_seg_split(u, w, q));
            }
        }
        // (C) on the segment before it reaches the crossed face, in whatever simplex around `u`
        // holds the point (a tet, or a face or edge the segment runs in exactly).
        if let Some(x) = x {
            for frac in [0.5, 0.25, 0.75, 0.1, 0.9] {
                if let Some(p) = sanitize(add(pu, scale(sub(x, pu), frac)))
                    && let Some(q) = self.insert_at_vertex_star(u, p)?
                {
                    return Ok(self.on_seg_split(u, w, q));
                }
            }
        }

        Err(fail(owner, Some([u, w]), "no valid Steiner point on the segment (it passes through a vertex)"))
    }

    /// Split edge `(x,y)` (its tets `ring`) at a point next to `px`, a point on it. Rounding puts
    /// `px` a little off the line; next to a ring tet that is almost flat this can invert a new
    /// tet, so a few other roundings of points within `1e-9 |xy|` of `px` are tried as well.
    fn split_edge_near(&mut self, ring: &[u32], x: u32, y: u32, px: V3) -> Result<Option<u32>, TetError> {
        let (a, b) = (self.dt.pts[x as usize], self.dt.pts[y as usize]);
        let ab = sub(b, a);
        let l2 = dot(ab, ab);
        if l2.is_nan() || l2 <= 0.0 {
            return Ok(None);
        }
        let t = (dot(sub(px, a), ab) / l2).clamp(0.0, 1.0);
        let mut cands: Vec<V3> = vec![px, add(a, scale(ab, t)), add(b, scale(sub(a, b), 1.0 - t))];
        for k in 1..=4 {
            for d in [1e-12, 1e-10, 1e-9] {
                for sgn in [1.0, -1.0] {
                    let tt = t + sgn * d * f64::from(k);
                    if tt > 0.0 && tt < 1.0 {
                        cands.push(add(a, scale(ab, tt)));
                    }
                }
            }
        }
        for c in cands {
            if let Some(q) = self.split_simplex(ring, &[x, y], c)? {
                return Ok(Some(q));
            }
        }
        Ok(None)
    }

    /// Flip away the flattest of the tets `ring` (the tets around an edge that cannot be split):
    /// an edge removal of one of its other edges or a 2-3 flip of one of its faces, keeping
    /// constraints and creating no shortcut; `true` if a flip was made.
    fn flip_slivers_around(&mut self, ring: &[u32]) -> Result<bool, TetError> {
        let mut order: Vec<(f64, u32)> = ring
            .iter()
            .filter(|&&t| self.dt.tds.alive[t as usize] && self.dt.tds.ghost_slot(t).is_none())
            .map(|&t| {
                let v = self.dt.tds.verts[t as usize].map(|x| self.dt.pts[x as usize]);
                (mean_ratio(v[0], v[1], v[2], v[3]), t)
            })
            .collect();
        order.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        let mut sc = crate::flips::FlipScratch::default();
        for &(_, t) in order.iter().take(2) {
            let v = self.dt.tds.verts[t as usize];
            for i in 0..4 {
                for j in i + 1..4 {
                    let (a, b) = (v[i], v[j]);
                    if self.edge_owner(a, b).is_some() {
                        continue;
                    }
                    if let Some(plan) = self.dt.plan_edge_removal(a, b, t, 16, &mut sc)
                        && !self.makes_shortcut(&plan.old, &plan.new)
                    {
                        self.dt.apply(&plan)?;
                        self.flips += 1;
                        return Ok(true);
                    }
                }
            }
            for k in 0..4 {
                let g = self.dt.tds.face(t, k);
                if self.face_owner(sorted3(g[0], g[1], g[2])).is_some() {
                    continue;
                }
                if let Some(plan) = self.dt.plan_flip23(t, k)
                    && !self.makes_shortcut(&plan.old, &plan.new)
                {
                    self.dt.apply(&plan)?;
                    self.flips += 1;
                    return Ok(true);
                }
            }
        }
        Ok(false)
    }

    /// Insert `p`, a point next to vertex `u`, into the simplex around `u` whose relative
    /// interior holds it (exact location: a tet, a face or an edge, never `u` itself).
    fn insert_at_vertex_star(&mut self, u: u32, p: V3) -> Result<Option<u32>, TetError> {
        let mut star = Vec::new();
        self.dt.tds.star(u, &mut star);
        for &t in &star {
            let v = self.dt.tds.verts[t as usize];
            if v.contains(&GHOST) {
                continue;
            }
            let c = v.map(|x| self.dt.pts[x as usize]);
            let o: Vec<i32> = (0..4).map(|k| {
                let f = crate::tds::FACE[k];
                ori(c[f[0]], c[f[1]], c[f[2]], p)
            }).collect();
            if o.iter().any(|&z| z > 0) {
                continue;
            }
            // Vertices of the simplex holding `p`: those opposite a face it is not on.
            let sigma: Vec<u32> = (0..4).filter(|&k| o[k] < 0).map(|k| v[k]).collect();
            return match sigma.len() {
                4 => self.split_simplex(&[t], &sigma, p),
                3 => {
                    let k = (0..4).find(|&k| o[k] == 0).unwrap_or(0);
                    let n = self.dt.tds.nbr[t as usize][k];
                    if n == crate::tds::NONE {
                        return Ok(None);
                    }
                    self.split_simplex(&[t, n], &sigma, p)
                }
                2 => {
                    let (mut ring, mut poly) = (Vec::new(), Vec::new());
                    if !self.dt.tds.ring(sigma[0], sigma[1], t, 512, &mut ring, &mut poly) {
                        return Ok(None);
                    }
                    self.split_simplex(&ring, &sigma, p)
                }
                _ => Ok(None),
            };
        }
        Ok(None)
    }

    /// Insert point `p` into the tets `old`, all containing the simplex `sigma`, by replacing in
    /// each of them every vertex of `sigma` by `p` in turn (1-4, 2-6 or n-2n split). `None` (and
    /// nothing changed) unless every new tet is strictly positive.
    fn split_simplex(&mut self, old: &[u32], sigma: &[u32], p: V3) -> Result<Option<u32>, TetError> {
        let Some(p) = sanitize(p) else { return Ok(None) };
        let mut news: Vec<[u32; 4]> = Vec::with_capacity(old.len() * sigma.len());
        for &t in old {
            let v = self.dt.tds.verts[t as usize];
            if v.contains(&GHOST) {
                return Ok(None);
            }
            for &s in sigma {
                let Some(i) = v.iter().position(|&x| x == s) else { return Ok(None) };
                let mut nv = v;
                nv[i] = NEW;
                let c = nv.map(|x| if x == NEW { p } else { self.dt.pts[x as usize] });
                if ori(c[0], c[1], c[2], c[3]) <= 0 {
                    return Ok(None);
                }
                news.push(nv);
            }
        }
        if self.n_steiner + 1 > self.max_steiner {
            return Err(TetError::SteinerBudget { limit: self.max_steiner });
        }
        let id = self.dt.push_point(p);
        self.n_steiner += 1;
        for nv in &mut news {
            for x in nv.iter_mut() {
                if *x == NEW {
                    *x = id;
                }
            }
        }
        let mut ids = Vec::new();
        self.dt.tds.replace(old, &news, &mut ids)?;
        self.flips += 1;
        Ok(Some(id))
    }

    /// Mark `p` as a boundary Steiner point.
    fn pin(&mut self, p: u32, on: On) {
        if self.dt.pinned.len() < self.dt.pts.len() {
            self.dt.pinned.resize(self.dt.pts.len(), false);
        }
        self.dt.pinned[p as usize] = true;
        self.bnd.on.insert(p, on);
        self.n_boundary_steiner += 1;
    }

    /// Record that subsegment `(u,w)` of an input edge was split at `p`.
    fn on_seg_split(&mut self, u: u32, w: u32, p: u32) -> u32 {
        if let Some(&e) = self.bnd.segs.get(&key2(u, w)) {
            self.bnd.replace_seg(&[key2(u, w)], &[key2(u, p), key2(p, w)], e);
            let chain = &mut self.bnd.chains[e as usize];
            if let Some(i) = chain.windows(2).position(|x| (x[0] == u && x[1] == w) || (x[0] == w && x[1] == u)) {
                chain.insert(i + 1, p);
            }
            self.pin(p, On::Edge(e));
        }
        p
    }

    // --------------------------------------------------------------------------------- facets

    /// The boundary of input triangle `tri` with the points of its edges, in its orientation.
    fn facet_polygon(&self, tri: u32) -> Vec<u32> {
        let t = self.orig.tris[tri as usize];
        let mut poly = Vec::new();
        for r in 0..3 {
            let (a, b) = (t[r], t[(r + 1) % 3]);
            let k = key2(a, b);
            let Ok(e) = self.orig.edges.binary_search_by(|x| x.0.cmp(&k)) else {
                poly.push(a);
                continue;
            };
            let chain = &self.bnd.chains[e];
            if chain.first() == Some(&a) {
                poly.extend_from_slice(&chain[..chain.len() - 1]);
            } else {
                poly.extend(chain.iter().rev().take(chain.len() - 1));
            }
        }
        poly
    }

    /// Recover input triangle `tri` as a set of tiles (see the module docs).
    fn recover_facet(&mut self, tri: u32) -> Result<(), TetError> {
        let t = self.orig.tris[tri as usize];
        let poly = self.facet_polygon(tri);
        let mut vs: Vec<u32> = poly.clone();
        vs.extend(self.bnd.face_pts.get(&tri).into_iter().flatten().copied());
        if let Some(tiles) = self.read_tiling(t, &poly, &vs) {
            self.commit_tiles(tri, tiles);
            return Ok(());
        }
        if poly.len() == 3 && vs.len() == 3 {
            match self.recover_face(tri, sorted3(t[0], t[1], t[2])) {
                Ok(()) => {
                    if let Some(tiles) = self.read_tiling(t, &poly, &vs) {
                        self.commit_tiles(tri, tiles);
                        return Ok(());
                    }
                }
                Err(TetError::RecoveryFailed { .. }) => {}
                Err(e) => return Err(e),
            }
        }
        // Remove the edges crossing the exact triangle; if a tiling still does not show up (the
        // points of its edges are rounded to both sides of its plane and an edge passes between
        // them and the exact boundary line), also those crossing an ear triangulation of the
        // polygon through the actual points.
        let tp = t.map(|i| self.dt.pts[i as usize]);
        self.clear_crossings(tri, &[tp], &mut vs)?;
        if let Some(tiles) = self.read_tiling(t, &poly, &vs) {
            self.commit_tiles(tri, tiles);
            return Ok(());
        }
        if poly.len() > 3
            && let Some(nrm) = unit_normal(&self.dt.pts, t)
            && let Some(ears) = triangulate_polygon(&self.dt.pts, &poly, nrm)
        {
            let refs: Vec<[V3; 3]> = ears.iter().map(|e| e.map(|i| self.dt.pts[i as usize])).collect();
            self.clear_crossings(tri, &refs, &mut vs)?;
        }
        let tiles = self.read_tiling(t, &poly, &vs).ok_or_else(|| fail(tri, None, "triangle is not tiled by mesh faces after removing its crossing edges"))?;
        self.commit_tiles(tri, tiles);
        Ok(())
    }

    /// Remove every tet edge that crosses one of the reference triangles `refs` of input
    /// triangle `tri` (no endpoint in `vs`): by an edge removal that creates no new crossing, by
    /// a split where it crosses (the point joins `vs`), or by removing an interior endpoint.
    fn clear_crossings(&mut self, tri: u32, refs: &[[V3; 3]], vs: &mut Vec<u32>) -> Result<(), TetError> {
        for _ in 0..MAX_FACE_STEPS {
            let Some((x, y, tet, r)) = self.crossing_edge(refs, vs) else { return Ok(()) };
            if let Some(other) = self.edge_owner(x, y) {
                return Err(TetError::SelfIntersection { tri_a: tri.min(other), tri_b: tri.max(other) });
            }
            if self.unflip_crossing(refs, vs, x, y, tet)? {
                continue;
            }
            let rp = refs[r];
            let Some(px) = line_plane(self.dt.pts[x as usize], self.dt.pts[y as usize], rp[0], rp[1], rp[2]) else {
                return Err(fail(tri, None, "an edge crossing the triangle is parallel to it"));
            };
            let mut ring = Vec::new();
            let mut rpoly = Vec::new();
            if !self.dt.tds.ring(x, y, tet, 512, &mut ring, &mut rpoly) {
                return Err(fail(tri, None, "open ring around an edge crossing the triangle"));
            }
            let split = self.split_edge_near(&ring, x, y, px)?;
            if split.is_none() {
                // No point of the edge is representable on the side of every ring tet it needs
                // (an almost flat ring tet): drop an endpoint that is an interior point instead.
                let mut gone = false;
                for z in [x, y] {
                    if z >= self.n_real && self.dt.droppable(z) && self.dt.remove_vertex(z)? {
                        gone = true;
                        break;
                    }
                }
                if gone || self.flip_slivers_around(&ring)? {
                    continue;
                }
            }
            let Some(p) = split else {
                return Err(fail(tri, None, "an edge crossing the triangle cannot be split on it"));
            };
            vs.push(p);
            self.bnd.face_pts.entry(tri).or_default().push(p);
            self.pin(p, On::Face(tri));
        }
        Err(fail(tri, None, "too many edges cross the triangle"))
    }

    fn commit_tiles(&mut self, tri: u32, tiles: Vec<[u32; 3]>) {
        for t in tiles {
            self.bnd.add_tile(t, tri);
        }
        self.bnd.done[tri as usize] = true;
    }

    /// Would replacing the tets `old` by `new` create a *shortcut*: an edge between two points of
    /// one split input edge that are not neighbours on it? Complete recovery never does: such an
    /// edge runs along the edge past a point rounded off it and leaves flat tets on the surface.
    pub(crate) fn makes_shortcut(&self, old: &[u32], new: &[[u32; 4]]) -> bool {
        if !self.complete_mode || self.bnd.on.is_empty() {
            return false;
        }
        let mut old_edges: BTreeSet<[u32; 2]> = BTreeSet::new();
        for &o in old {
            let v = self.dt.tds.verts[o as usize];
            for i in 0..4 {
                for j in i + 1..4 {
                    old_edges.insert(key2(v[i], v[j]));
                }
            }
        }
        new.iter().any(|v| (0..4).any(|i| (i + 1..4).any(|j| !old_edges.contains(&key2(v[i], v[j])) && self.is_shortcut(v[i], v[j]))))
    }

    /// Is `(x,y)` an edge between two points of one input edge's chain that are not neighbours
    /// on it?
    fn is_shortcut(&self, x: u32, y: u32) -> bool {
        if self.bnd.segs.contains_key(&key2(x, y)) {
            return false;
        }
        let chain_of = |v: u32| match self.bnd.on.get(&v) {
            Some(On::Edge(e)) => Some(*e),
            _ => None,
        };
        let on_chain = |v: u32, e: u32| chain_of(v) == Some(e) || self.orig.edges[e as usize].0.contains(&v);
        match (chain_of(x), chain_of(y)) {
            (Some(e), _) if on_chain(y, e) => true,
            (_, Some(e)) if on_chain(x, e) => true,
            (None, None) => self.orig.edges.binary_search_by(|z| z.0.cmp(&key2(x, y))).is_ok_and(|e| self.bnd.chains[e].len() > 2),
            _ => false,
        }
    }

    /// Remove the crossing edge `(x,y)` of triangle `t` by an edge removal whose new edges do not
    /// cross it; `true` if done.
    fn unflip_crossing(&mut self, refs: &[[V3; 3]], vs: &[u32], x: u32, y: u32, tet: u32) -> Result<bool, TetError> {
        let mut sc = crate::flips::FlipScratch::default();
        let Some(plan) = self.dt.plan_edge_removal(x, y, tet, 16, &mut sc) else { return Ok(false) };
        if self.makes_shortcut(&plan.old, &plan.new) {
            return Ok(false);
        }
        // Only the new edges (diagonals of the ring) matter; old ones are dealt with in turn.
        let mut old_edges: BTreeSet<[u32; 2]> = BTreeSet::new();
        for &o in &plan.old {
            let v = self.dt.tds.verts[o as usize];
            for i in 0..4 {
                for j in i + 1..4 {
                    old_edges.insert(key2(v[i], v[j]));
                }
            }
        }
        for v in &plan.new {
            for i in 0..4 {
                for j in i + 1..4 {
                    if !old_edges.contains(&key2(v[i], v[j])) && refs.iter().any(|&tp| self.crosses(tp, vs, v[i], v[j])) {
                        return Ok(false);
                    }
                }
            }
        }
        self.dt.apply(&plan)?;
        self.flips += 1;
        Ok(true)
    }

    /// Does the edge `(x,y)` (no endpoint in `vs`) cross the relative interior of triangle `tp`
    /// exactly?
    fn crosses(&self, tp: [V3; 3], vs: &[u32], x: u32, y: u32) -> bool {
        if x == GHOST || y == GHOST || vs.contains(&x) || vs.contains(&y) {
            return false;
        }
        let (px, py) = (self.dt.pts[x as usize], self.dt.pts[y as usize]);
        if ori(tp[0], tp[1], tp[2], px) * ori(tp[0], tp[1], tp[2], py) >= 0 {
            return false;
        }
        let o = [ori(px, py, tp[0], tp[1]), ori(px, py, tp[1], tp[2]), ori(px, py, tp[2], tp[0])];
        o.iter().all(|&z| z > 0) || o.iter().all(|&z| z < 0)
    }

    /// A tet edge `(x,y)` (neither endpoint on the triangle) that crosses the relative interior
    /// of triangle `t` exactly, with a tet containing it; `vs` are the vertices on the triangle.
    ///
    /// The tets whose interior meets the triangle are face-connected, and those without a vertex
    /// in `vs` have a crossing edge (their plane section reaches into the triangle, whose edge
    /// pieces are mesh edges). So if a crossing edge exists, one lies in a tet of the stars of
    /// `vs` or in a face neighbour of one.
    fn crossing_edge(&mut self, refs: &[[V3; 3]], vs: &[u32]) -> Option<(u32, u32, u32, usize)> {
        let mut tets: Vec<u32> = Vec::new();
        let mut star = Vec::new();
        for &v in vs {
            self.dt.tds.star(v, &mut star);
            tets.extend_from_slice(&star);
        }
        let n_star = tets.len();
        for i in 0..n_star {
            for k in 0..4 {
                let n = self.dt.tds.nbr[tets[i] as usize][k];
                if n != crate::tds::NONE {
                    tets.push(n);
                }
            }
        }
        let mut checked: BTreeSet<u32> = BTreeSet::new();
        for tet in tets {
            if !checked.insert(tet) {
                continue;
            }
            let v = self.dt.tds.verts[tet as usize];
            for i in 0..4 {
                for j in i + 1..4 {
                    if let Some(r) = refs.iter().position(|&tp| self.crosses(tp, vs, v[i], v[j])) {
                        return Some((v[i].min(v[j]), v[i].max(v[j]), tet, r));
                    }
                }
            }
        }
        None
    }

    /// The mesh faces with all vertices in `vs` (on input triangle `t`, whose boundary with the
    /// points of its edges is `poly`), oriented like `t`, if they tile it: every boundary piece
    /// once in the direction of `poly`, every other edge once in each direction, every vertex
    /// of `vs` used.
    fn read_tiling(&mut self, t: [u32; 3], poly: &[u32], vs: &[u32]) -> Option<Vec<[u32; 3]>> {
        let mut vset: Vec<u32> = vs.to_vec();
        vset.sort_unstable();
        let ns = unit_normal(&self.dt.pts, t)?;
        let mut keys: BTreeSet<[u32; 3]> = BTreeSet::new();
        let mut star = Vec::new();
        for &v in &vset {
            self.dt.tds.star(v, &mut star);
            for &tet in &star {
                for i in 0..4 {
                    let f = self.dt.tds.face(tet, i);
                    if f.iter().all(|x| vset.binary_search(x).is_ok()) {
                        keys.insert(sorted3(f[0], f[1], f[2]));
                    }
                }
            }
        }
        // Candidate tiles by directed edge, oriented like `t` by their normal. A needle face (a
        // sliver along an edge of `t`, whose points are rounded off the line) or one whose normal
        // is almost perpendicular to `t`'s may be used either way.
        let mut cand: BTreeMap<(u32, u32), Vec<[u32; 3]>> = BTreeMap::new();
        for k in keys {
            let n = cross(sub(self.dt.pts[k[1] as usize], self.dt.pts[k[0] as usize]), sub(self.dt.pts[k[2] as usize], self.dt.pts[k[0] as usize]));
            let d = dot(n, ns);
            let nl = dot(n, n).sqrt();
            let pk = k.map(|i| self.dt.pts[i as usize]);
            let lmax = dist(pk[0], pk[1]).max(dist(pk[1], pk[2])).max(dist(pk[2], pk[0]));
            // A needle (area tiny against its longest edge) has no reliable normal.
            let needle = nl <= 1e-6 * lmax * lmax;
            let mut both = Vec::new();
            if needle {
                both.push(k);
                both.push([k[0], k[2], k[1]]);
            } else if d > 1e-6 * nl {
                both.push(k);
            } else if d < -1e-6 * nl {
                both.push([k[0], k[2], k[1]]);
            } else {
                both.push(k);
                both.push([k[0], k[2], k[1]]);
            }
            for tile in both {
                for r in 0..3 {
                    cand.entry((tile[r], tile[(r + 1) % 3])).or_default().push(tile);
                }
            }
        }
        // Search a tiling from the boundary pieces: every directed edge used once, every interior
        // edge in both directions, no boundary piece backwards, every vertex of `vs` used. Tets
        // lying flat in the triangle offer two tilings of their outline; the search backtracks.
        let m = poly.len();
        let bnd: BTreeSet<(u32, u32)> = (0..m).map(|r| (poly[r], poly[(r + 1) % m])).collect();
        let mut st = TileSearch { cand: &cand, bnd: &bnd, vset: &vset, covered: BTreeSet::new(), need: bnd.clone(), tiles: Vec::new(), budget: 20_000 };
        if st.dfs() {
            return Some(st.tiles);
        }

        None
    }

    // ---------------------------------------------------------------------------- suppression

    fn suppress_all(&mut self) -> Result<(), TetError> {
        for _sweep in 0..MAX_SWEEPS {
            if self.bnd.on.is_empty() {
                return Ok(());
            }
            let pts: Vec<u32> = self.bnd.on.keys().rev().copied().collect();
            let mut progress = false;
            for p in pts {
                if self.suppress(p)? {
                    progress = true;
                }
            }
            if !progress {
                break;
            }
        }
        match self.bnd.on.keys().next() {
            None => Ok(()),
            Some(&p) => {
                let si = self.bnd.subs.iter().position(|s| s.0.contains(&p)).unwrap_or(0) as u32;
                Err(fail(si, None, "a temporary boundary Steiner point could not be moved off the surface"))
            }
        }
    }

    /// Remove boundary Steiner point `p` from the surface (see the module docs). `false` if it
    /// cannot be done now (nothing changed).
    fn suppress(&mut self, p: u32) -> Result<bool, TetError> {
        let Some(&on) = self.bnd.on.get(&p) else { return Ok(true) };
        // The fans of `p`, by input triangle: directed link edges x -> y of subfaces (p, x, y).
        let mut fans: BTreeMap<u32, Vec<(u32, u32)>> = BTreeMap::new();
        let mut fan_idx: Vec<usize> = Vec::new();
        for (i, s) in self.bnd.subs.iter().enumerate() {
            let t = s.0;
            if let Some(r) = t.iter().position(|&x| x == p) {
                fans.entry(s.1).or_default().push((t[(r + 1) % 3], t[(r + 2) % 3]));
                fan_idx.push(i);
            }
        }
        let want = match on {
            On::Face(_) => 1,
            On::Edge(_) => 2,
        };
        if fans.len() != want {
            return Err(fail(fan_idx.first().map_or(0, |&i| i as u32), None, "inconsistent fan around a boundary Steiner point"));
        }
        // Polygons without `p`, triangulated in their input triangle's plane.
        let mut new_subs: Vec<([u32; 3], u32)> = Vec::new();
        let mut ends: Vec<(u32, u32)> = Vec::new();
        for (&tri, edges) in &fans {
            let next: BTreeMap<u32, u32> = edges.iter().copied().collect();
            if next.len() != edges.len() {
                return Ok(false);
            }
            let heads: BTreeSet<u32> = edges.iter().map(|e| e.1).collect();
            let start = match on {
                On::Face(_) => edges.iter().map(|e| e.0).min(),
                On::Edge(_) => edges.iter().map(|e| e.0).find(|x| !heads.contains(x)),
            };
            let Some(start) = start else { return Ok(false) };
            let mut poly = vec![start];
            let mut cur = start;
            while let Some(&nx) = next.get(&cur) {
                if nx == start || poly.len() > edges.len() + 1 {
                    break;
                }
                poly.push(nx);
                cur = nx;
            }
            let closed = matches!(on, On::Face(_));
            if poly.len() != edges.len() + usize::from(!closed) {
                return Ok(false);
            }
            if !closed {
                ends.push((poly[0], poly[poly.len() - 1]));
            }
            let Some(nrm) = unit_normal(&self.dt.pts, self.orig.tris[tri as usize]) else { return Ok(false) };
            let Some(tris) = triangulate_polygon(&self.dt.pts, &poly, nrm) else { return Ok(false) };
            new_subs.extend(tris.into_iter().map(|t| (t, tri)));
        }
        if let [a, b] = ends[..]
            && (a.0 != b.1 || a.1 != b.0)
        {
            return Ok(false);
        }
        // Direction into the solid at `p`.
        let mut dsum = [0.0; 3];
        for &tri in fans.keys() {
            let Some(n) = unit_normal(&self.dt.pts, self.orig.tris[tri as usize]) else { return Ok(false) };
            dsum = sub(dsum, n);
        }
        let dl = dot(dsum, dsum).sqrt();
        if dl.is_nan() || dl <= 0.0 {
            return Ok(false);
        }
        let d_in = scale(dsum, 1.0 / dl);
        // Split the star by the fans into the solid and the exterior side.
        let mut star = Vec::new();
        self.dt.tds.star(p, &mut star);
        star.sort_unstable();
        let fan_keys: BTreeSet<[u32; 3]> = fan_idx.iter().map(|&i| {
            let t = self.bnd.subs[i].0;
            sorted3(t[0], t[1], t[2])
        }).collect();
        let mut seed_in = None;
        let mut seed_out = None;
        for &t in &star {
            for k in 0..4 {
                let g = self.dt.tds.face(t, k);
                if !g.contains(&p) || !fan_keys.contains(&sorted3(g[0], g[1], g[2])) {
                    continue;
                }
                let solid = fan_idx.iter().any(|&i| same_cycle(g, self.bnd.subs[i].0));
                if solid {
                    seed_in.get_or_insert(t);
                } else {
                    seed_out.get_or_insert(t);
                }
            }
        }

        let (Some(si), Some(so)) = (seed_in, seed_out) else { return Ok(false) };
        let flood = |this: &Self, seed: u32| -> Vec<u32> {
            let mut grp = vec![seed];
            let mut head = 0;
            while head < grp.len() {
                let t = grp[head];
                head += 1;
                for k in 0..4 {
                    if this.dt.tds.verts[t as usize][k] == p {
                        continue;
                    }
                    let g = this.dt.tds.face(t, k);
                    if fan_keys.contains(&sorted3(g[0], g[1], g[2])) {
                        continue;
                    }
                    let n = this.dt.tds.nbr[t as usize][k];
                    if n != crate::tds::NONE && star.binary_search(&n).is_ok() && !grp.contains(&n) {
                        grp.push(n);
                    }
                }
            }
            grp
        };
        let g_in = flood(self, si);
        let g_out = flood(self, so);
        if g_in.len() + g_out.len() != star.len() || g_in.iter().any(|t| g_out.contains(t)) {
            return Ok(false);
        }
        let polys_in: Vec<[u32; 3]> = new_subs.iter().map(|s| s.0).collect();
        let polys_out: Vec<[u32; 3]> = polys_in.iter().map(|t| [t[0], t[2], t[1]]).collect();
        // (A solid-side point pushed here stays unused if the exterior side fails; unused points
        // are dropped from the output.)
        let Some((old_in, mut new)) = self.side_fill(p, &g_in, &polys_in, &star, true, d_in)? else { return Ok(false) };
        let Some((old_out, new_out)) = self.side_fill(p, &g_out, &polys_out, &star, false, scale(d_in, -1.0))? else { return Ok(false) };
        if old_in.iter().any(|t| old_out.binary_search(t).is_ok()) {
            return Ok(false);
        }
        new.extend(new_out);
        let mut old = old_in;
        old.extend(old_out);
        old.sort_unstable();
        if !self.constraints_kept(&old, &new, p) {
            return Ok(false);
        }
        let mut ids = Vec::new();
        self.dt.tds.replace(&old, &new, &mut ids)?;
        self.flips += 1;
        // Update the surface.
        let mut keep: Vec<([u32; 3], u32)> = Vec::with_capacity(self.bnd.subs.len());
        let subs = std::mem::take(&mut self.bnd.subs);
        for (i, s) in subs.into_iter().enumerate() {
            if fan_idx.binary_search(&i).is_err() {
                keep.push(s);
            } else {
                self.bnd.index_tile(s.0, s.1, false);
            }
        }
        self.bnd.subs = keep;
        for (t, tri) in new_subs {
            self.bnd.add_tile(t, tri);
        }
        if let On::Edge(e) = on
            && let Some(&(a, b)) = ends.first()
        {
            self.bnd.replace_seg(&[key2(a, p), key2(b, p)], &[key2(a, b)], e);
        }
        self.bnd.on.remove(&p);
        if let Some(x) = self.dt.pinned.get_mut(p as usize) {
            *x = false;
        }
        Ok(true)
    }

    /// Refill one side of suppressed point `p`: the tets `group` of its star on that side,
    /// closed by the re-triangulated fans `polys` (outward for this side). When the side cannot
    /// be filled (a sliver of the star lying flat on the surface hides every point near `p`),
    /// the cavity absorbs the tets beyond the faces that block the points next to `p`, never
    /// across a constraint face and never enclosing a vertex that must stay; up to
    /// [`SIDE_GROW`] times. Returns the old tets (sorted) and the new ones. (Growing the solid
    /// side further for ruling S4 was tried and made later suppressions fail; points left too
    /// shallow are moved after classification instead, see [`crate::relocate`].)
    fn side_fill(&mut self, p: u32, group: &[u32], polys: &[[u32; 3]], star: &[u32], solid: bool, dir: V3) -> Result<Option<SideFill>, TetError> {
        let mut cav: Vec<u32> = group.to_vec();
        cav.sort_unstable();
        let pp = self.dt.pts[p as usize];
        for round in 0..=SIDE_GROW {
            let mut faces: Vec<[u32; 3]> = polys.to_vec();
            let mut outside: Vec<([u32; 3], u32)> = Vec::new();
            for &t in &cav {
                for k in 0..4 {
                    let g = self.dt.tds.face(t, k);
                    if g.contains(&p) {
                        continue;
                    }
                    let n = self.dt.tds.nbr[t as usize][k];
                    if n != crate::tds::NONE && cav.binary_search(&n).is_ok() {
                        continue;
                    }
                    faces.push(g);
                    outside.push((g, n));
                }
            }
            // Refills without a point are tried on the star itself only (a grown cavity is
            // starred: cheap, and it exists by construction of the growth).
            let try_fill = round == 0 && cav.len() <= MAX_FILL_TETS;
            let blockers = match self.fill_side(&faces, pp, dir, solid, try_fill) {
                SideOutcome::Filled(tets) => return Ok(Some((cav, tets))),
                SideOutcome::Star(q) => {
                    let tets = self.star_new(&faces, q)?;
                    return Ok(Some((cav, tets)));
                }
                SideOutcome::Blocked(b) => b,
            };
            if cav.len() > MAX_SIDE_TETS {
                break;
            }
            // Absorb the tets beyond the blocking faces (all boundary faces if none is known).
            let mut add: Vec<u32> = Vec::new();
            for &(g, n) in &outside {
                if !blockers.is_empty() && !blockers.contains(&g) {
                    continue;
                }
                if n == crate::tds::NONE || self.dt.tds.ghost_slot(n).is_some() || star.binary_search(&n).is_ok() || add.contains(&n) {
                    continue;
                }
                if self.face_owner(sorted3(g[0], g[1], g[2])).is_some() {
                    continue;
                }
                if self.absorb_ok(n, &cav, &add, p) {
                    add.push(n);
                }
            }
            if add.is_empty() {
                break;
            }
            cav.extend(add);
            cav.sort_unstable();
        }
        Ok(None)
    }

    /// Add point `q` and star the faces `faces` from it.
    fn star_new(&mut self, faces: &[[u32; 3]], q: V3) -> Result<Vec<[u32; 4]>, TetError> {
        if self.n_steiner + 1 > self.max_steiner {
            return Err(TetError::SteinerBudget { limit: self.max_steiner });
        }
        let id = self.dt.push_point(q);
        self.n_steiner += 1;
        Ok(faces.iter().map(|f| [id, f[0], f[1], f[2]]).collect())
    }

    /// May tet `n` join the side cavity `cav` (plus `add`)? Not if a face it shares with the
    /// cavity is a constraint face, or if it would enclose a vertex that cavity fills may not
    /// drop (every tet of that vertex in the cavity).
    fn absorb_ok(&mut self, n: u32, cav: &[u32], add: &[u32], p: u32) -> bool {
        let inside = |t: u32| t == n || cav.binary_search(&t).is_ok() || add.contains(&t);
        for k in 0..4 {
            let m = self.dt.tds.nbr[n as usize][k];
            if m != crate::tds::NONE && m != n && inside(m) {
                let h = self.dt.tds.face(n, k);
                if self.face_owner(sorted3(h[0], h[1], h[2])).is_some() {
                    return false;
                }
            }
        }
        let mut star = Vec::new();
        for v in self.dt.tds.verts[n as usize] {
            if v == p || v == GHOST || self.dt.droppable(v) {
                continue;
            }
            self.dt.tds.star(v, &mut star);
            if star.iter().all(|&t| inside(t)) {
                return false;
            }
        }
        true
    }

    /// True if every constraint edge and face of the tets `old` (other than those of `p`, which
    /// is removed) is an edge or face of the tets `new`.
    fn constraints_kept(&self, old: &[u32], new: &[[u32; 4]], p: u32) -> bool {
        let mut edges: BTreeSet<[u32; 2]> = BTreeSet::new();
        let mut faces: BTreeSet<[u32; 3]> = BTreeSet::new();
        for v in new {
            for i in 0..4 {
                for j in i + 1..4 {
                    edges.insert(key2(v[i], v[j]));
                }
                let f = crate::tds::FACE[i];
                faces.insert(sorted3(v[f[0]], v[f[1]], v[f[2]]));
            }
        }
        for &t in old {
            let v = self.dt.tds.verts[t as usize];
            for i in 0..4 {
                for j in i + 1..4 {
                    let (x, y) = (v[i], v[j]);
                    if x != p && y != p && self.edge_owner(x, y).is_some() && !edges.contains(&key2(x, y)) {
                        return false;
                    }
                }
                let g = self.dt.tds.face(t, i);
                let k = sorted3(g[0], g[1], g[2]);
                if !g.contains(&p) && self.face_owner(k).is_some() && !faces.contains(&k) {
                    return false;
                }
            }
        }
        true
    }

    /// Close the polyhedron bounded by `faces` (outward), one side of a suppressed point at
    /// `p`. The solid side is refilled without a point if possible; otherwise, of the candidate
    /// points (the polyhedron's kernel point, the deepest the kernel search finds, then `p`
    /// moved towards `dir` by 0.5 h down to 1e-12 h) that see every face, the best-shaped star
    /// wins. Points left too shallow are moved by [`crate::relocate`] after classification. On
    /// failure, the faces that hide the candidate with the fewest of them.
    fn fill_side(&self, faces: &[[u32; 3]], p: V3, dir: V3, solid: bool, try_fill: bool) -> SideOutcome {
        if solid
            && try_fill
            && let Some(t) = self.dt.fill_budgeted(faces, None, &[], SIDE_FILL_BUDGET + 10 * faces.len())
        {
            return SideOutcome::Filled(t);
        }
        let pts = &self.dt.pts;
        let mut h = 0.0;
        let mut cnt = 0.0;
        for f in faces {
            for &v in f {
                h += dist(p, pts[v as usize]);
                cnt += 1.0;
            }
        }
        let h = if cnt > 0.0 { h / cnt } else { 1.0 };
        // The kernel point (the deepest of the kernel search) and offsets. Review round 3 tried
        // ranking these by S4 margin before shape and a wider candidate set: the first changed
        // nothing on the hard relief cases after step 9 (`crate::relocate`) but cost time, the
        // second lost `relief n12 seed4 d0.97` (a deeper star blocked a later suppression).
        let mut cands: Vec<V3> = self.dt.kernel_point(faces, self.guard).into_iter().collect();
        for d in [0.5, 0.3, 0.2, 0.1, 0.05, 0.02, 1e-2, 3e-3, 1e-3, 1e-4, 1e-5, 1e-6, 1e-8, 1e-10, 1e-12] {
            if let Some(q) = sanitize(add(p, scale(dir, d * h))) {
                cands.push(q);
            }
        }
        let mut best: Option<(f64, V3)> = None;
        let mut blockers: Option<Vec<[u32; 3]>> = None;
        for q in cands {
            let mut worst = f64::INFINITY;
            let mut blk: Vec<[u32; 3]> = Vec::new();
            for f in faces {
                let [a, b, c] = f.map(|i| pts[i as usize]);
                if ori(a, b, c, q) >= 0 {
                    blk.push(*f);
                } else {
                    worst = worst.min(mean_ratio(q, a, b, c));
                }
            }
            if blk.is_empty() {
                if best.is_none_or(|b| worst > b.0) {
                    best = Some((worst, q));
                }
            } else if blockers.as_ref().is_none_or(|b| blk.len() < b.len()) {
                blockers = Some(blk);
            }
        }
        if let Some((_, q)) = best {
            return SideOutcome::Star(q);
        }
        if !solid
            && try_fill
            && let Some(t) = self.dt.fill_budgeted(faces, None, &[], SIDE_FILL_BUDGET + 10 * faces.len())
        {
            return SideOutcome::Filled(t);
        }
        SideOutcome::Blocked(blockers.unwrap_or_default())
    }
}

impl crate::delaunay::Dt {
    /// Remove the points with index in `from..to` whose star can be refilled without them
    /// (vertex removal by exact gift wrapping); points on the mesh boundary stay. Returns the
    /// number removed. Constraints are kept: every face and edge inside a star contains its
    /// centre, which is not a surface point.
    pub(crate) fn remove_points(&mut self, from: usize, to: usize) -> Result<usize, TetError> {
        let mut removed = 0;
        for v in from..to.min(self.pts.len()) {
            if self.remove_vertex(v as u32)? {
                removed += 1;
            }
        }
        Ok(removed)
    }

    /// Remove vertex `v` if its star is closed and can be refilled without it; `true` if done.
    pub(crate) fn remove_vertex(&mut self, v: u32) -> Result<bool, TetError> {
        let mut star = Vec::new();
        self.tds.star(v, &mut star);
        if star.is_empty() {
            return Ok(false);
        }
        let mut faces = Vec::with_capacity(star.len());
        for &t in &star {
            let Some(k) = self.tds.verts[t as usize].iter().position(|&x| x == v) else { return Ok(false) };
            if self.tds.ghost_slot(t).is_some() || (0..4).any(|j| j != k && self.tds.nbr[t as usize][j] == crate::tds::NONE) {
                return Ok(false);
            }
            faces.push(self.tds.face(t, k));
        }
        match self.fill_budgeted(&faces, None, &[], 2000 + 40 * faces.len()) {
            Some(new) => {
                let mut ids = Vec::new();
                self.tds.replace(&star, &new, &mut ids)?;
                Ok(true)
            }
            None => Ok(false),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{SurfaceInput, TetOptions};
    use sm_testkit::surf3;

    /// Mesh with complete recovery only (no interior-point recovery first) and check the
    /// boundary exactly, the orientation, the input points and the volume.
    fn complete_only(name: &str, inp: &SurfaceInput) {
        let m = crate::pipeline::tetrahedralize_with(inp, &TetOptions { optimize: 0, ..TetOptions::default() }, true).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_eq!(m.points[..inp.points.len()], inp.points[..], "{name}");
        let mut faces: BTreeMap<[u32; 3], usize> = BTreeMap::new();
        let mut vol = 0.0;
        for t in &m.tets {
            let q = t.map(|i| m.points[i as usize]);
            assert!(ori(q[0], q[1], q[2], q[3]) > 0, "{name}: tet {t:?}");
            vol += crate::geom::vol6(q[0], q[1], q[2], q[3]) / 6.0;
            for f in crate::tds::FACE {
                let g = crate::tds::canon_tri([t[f[0]], t[f[1]], t[f[2]]]);
                *faces.entry(g).or_default() += 1;
            }
        }
        let tris: BTreeSet<[u32; 3]> = inp.tris.iter().map(|&t| crate::tds::canon_tri(t)).collect();
        for t in &tris {
            assert_eq!(faces.get(t), Some(&1), "{name}: triangle {t:?}");
        }
        for f in faces.keys() {
            let rev = crate::tds::canon_tri([f[0], f[2], f[1]]);
            assert!(faces.contains_key(&rev) || tris.contains(f), "{name}: extra boundary face {f:?}");
        }
        let want = surf3::polyhedron_volume(&inp.points, &inp.tris);
        assert!((vol - want).abs() <= 1e-10 * want.abs().max(1.0), "{name}: volume {vol} vs {want}");
    }

    /// Complete recovery alone recovers every surf3 fixture (it is the fallback of the normal
    /// path, so it is exercised here on inputs where the normal path does not need it).
    #[test]
    fn complete_recovery_alone_meshes_the_fixtures() {
        let light = cfg!(debug_assertions);
        let mut cases: Vec<(String, surf3::SurfaceFixture)> = vec![
            ("schonhardt".into(), surf3::schonhardt()),
            ("cube(3)".into(), surf3::tri_cube(3)),
            ("cavity(2)".into(), surf3::cube_with_cavity(2)),
            ("two(2)".into(), surf3::two_cubes_shared(2)),
            ("ico(2)".into(), surf3::icosphere(1.0, 2)),
            ("torus(8,6)".into(), surf3::tri_torus(2.0, 0.7, 8, 6)),
            ("noisy(0)".into(), surf3::noisy_sphere(0)),
        ];
        if !light {
            for s in 1..6 {
                cases.push((format!("noisy({s})"), surf3::noisy_sphere(s)));
            }
            cases.push(("cube(6)".into(), surf3::tri_cube(6)));
            cases.push(("cavity(4)".into(), surf3::cube_with_cavity(4)));
            cases.push(("torus2(16,8)".into(), surf3::tri_torus(5.0, 0.5, 16, 8)));
            cases.push(("ico(3)".into(), surf3::icosphere(3.7, 3)));
        }
        for (name, f) in &cases {
            for v in 0..f.volumes.len() {
                complete_only(&format!("{name} v{v}"), &SurfaceInput { points: f.points.clone(), tris: f.oriented_tris(v) });
            }
        }
    }

    /// Height-field relief: the interior vertices of face `axis = 1` of `tri_cube(n)` pushed
    /// inwards by up to `depth` (SplitMix64 in point order); `two` also raises face `axis = 0`.
    pub(crate) fn relief(n: u32, seed: u64, depth: f64, axis: usize, two: bool) -> SurfaceInput {
        let f = surf3::tri_cube(n);
        let mut inp = SurfaceInput { points: f.points.clone(), tris: f.oriented_tris(0) };
        let mut rng = surf3::SplitMix64(seed);
        let (a1, a2) = ((axis + 1) % 3, (axis + 2) % 3);
        for p in &mut inp.points {
            let inner = p[a1] > 0.0 && p[a1] < 1.0 && p[a2] > 0.0 && p[a2] < 1.0;
            if inner && p[axis] == 1.0 {
                p[axis] = 1.0 - depth * rng.unit().abs();
            } else if two && inner && p[axis] == 0.0 {
                p[axis] = depth * rng.unit().abs();
            }
        }
        inp
    }

    /// Stacked twisted `k`-gon prisms: ring `i` at height `z_i` rotated by `rot_i` degrees.
    pub(crate) fn tower(k: u32, rings: &[(f64, f64)]) -> SurfaceInput {
        let mut points = Vec::new();
        for &(z, rot) in rings {
            for i in 0..k {
                let a = std::f64::consts::TAU * f64::from(i) / f64::from(k) + rot.to_radians();
                points.push([a.cos(), a.sin(), z]);
            }
        }
        let nr = rings.len() as u32;
        let top = k * (nr - 1);
        let mut tris = Vec::new();
        for i in 1..k - 1 {
            tris.push([0, i + 1, i]);
            tris.push([top, top + i, top + i + 1]);
        }
        for l in 0..nr - 1 {
            let (lo, hi) = (k * l, k * l + k);
            for i in 0..k {
                let j = (i + 1) % k;
                tris.push([lo + i, lo + j, hi + j]);
                tris.push([lo + i, hi + j, hi + i]);
            }
        }
        SurfaceInput { points, tris }
    }

    /// Hard surfaces for complete recovery alone (release only: about a hundred meshes).
    #[test]
    fn complete_recovery_alone_meshes_hard_surfaces() {
        if cfg!(debug_assertions) {
            return;
        }
        for n in [4u32, 6, 8] {
            for seed in 1..=4u64 {
                for depth in [0.6, 0.9] {
                    complete_only(&format!("relief n{n} s{seed} d{depth}"), &relief(n, seed, depth, 2, false));
                }
            }
        }
        for seed in [11u64, 12, 13] {
            complete_only(&format!("relief two s{seed}"), &relief(5, seed, 0.4, 2, true));
        }
        for seed in [21u64, 22] {
            complete_only(&format!("relief side s{seed}"), &relief(6, seed, 0.5, 0, false));
        }
        complete_only("hex tower", &tower(6, &[(0.0, 0.0), (0.3, 25.0), (0.6, 0.0), (0.9, 25.0)]));
        complete_only("quad tower", &tower(4, &[(0.0, 0.0), (0.5, 40.0), (1.0, 80.0)]));
        complete_only("tri tower", &tower(3, &[(0.0, 0.0), (0.4, 40.0), (0.8, 80.0), (1.2, 119.0)]));
        for tw in [25.0, 35.0, 45.0, 55.0] {
            complete_only(&format!("prism {tw}"), &tower(3, &[(0.0, 0.0), (1.0, tw)]));
        }
    }

    #[test]
    fn square_with_edge_points_triangulates_without_degenerate_ears() {
        // Unit square in z = 0 with a point on the bottom edge.
        let pts: Vec<V3> = vec![[0.0, 0.0, 0.0], [0.5, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]];
        let t = triangulate_polygon(&pts, &[0, 1, 2, 3, 4], [0.0, 0.0, 1.0]).unwrap();
        assert_eq!(t.len(), 3);
        for tri in &t {
            let [a, b, c] = tri.map(|i| pts[i as usize]);
            let n = cross(sub(b, a), sub(c, a));
            assert!(n[2] > 0.0, "{tri:?}");
        }
    }

    #[test]
    fn line_plane_and_closest_points() {
        let x = line_plane([0.0, 0.0, -1.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]).unwrap();
        assert_eq!(x, [0.0, 0.0, 0.0]);
        let (p, d) = edge_line_closest([-1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, -1.0, 1.0], [0.0, 1.0, 1.0]).unwrap();
        assert_eq!(p, [0.0, 0.0, 0.0]);
        assert_eq!(d, 1.0);
    }
}
