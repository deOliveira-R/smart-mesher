//! The compiled CSG leaf: cells as DNFs over classified surfaces, CSR adjacency, and the three
//! leaf-local questions (who contains this point, what is the next event, who is across).
//!
//! Cells must be disjoint (Q4): nothing here resolves a point by cell order. A point matched by
//! two cells is an overlap.

use super::cellindex::{CellIndex, DEFAULT_MIN_CELLS};
use super::kernel::{ClassifyError, Surf, V3};
use crate::vec3::{dot, madd, norm, sub};
use sm_core::{Aabb, EntityId, EntityKind, Point3};
use sm_geom_api::{GeomStore, ModelTolerance, Sign};
use sm_rt_api::{BuildIssue, ComplementPolicy, CsgLeafInput, Result, RtError, WorkCounters, hashed_direction};
use std::collections::BTreeMap;

/// Cap on reported issues per code.
const MAX_ISSUES: usize = 16;

/// Concentric radial structure for the fast path (nested spheres or coaxial cylinders).
pub(crate) struct Radial {
    sphere: bool,
    o: V3,
    a: V3,
    /// Distinct radii, ascending.
    radii: Vec<f64>,
    /// Candidate cells per shell `0..=radii.len()` (CSR).
    cand_off: Vec<u32>,
    cand: Vec<u32>,
}

/// Result of a point classification over cells.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Found {
    None,
    One(u32),
    Overlap,
}

/// What lies across a crossed surface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Next {
    Cell(u32),
    Exterior,
    Overlap,
}

/// A compiled CSG leaf.
pub(crate) struct Leaf {
    pub surfs: Vec<Surf>,
    pub surf_ids: Vec<EntityId>,
    term_off: Vec<u32>,
    hs_off: Vec<u32>,
    /// `surface << 1 | positive`.
    hs: Vec<u32>,
    cs_off: Vec<u32>,
    cs: Vec<u32>,
    sc_off: Vec<u32>,
    sc: Vec<u32>,
    pub cell_region: Vec<u32>,
    pub region_names: Vec<String>,
    pub volumes: Vec<Option<f64>>,
    radial: Option<Radial>,
    /// Raw cell boxes for the index (infinite sides allowed; a provably empty cell is unbounded here).
    cbox: Vec<[f64; 6]>,
    /// The kd tiling, built at `DEFAULT_MIN_CELLS` cells or more.
    pub(super) index: Option<CellIndex>,
    pub tol: f64,
    ang: f64,
    pub bounds: Aabb,
    pub complement: Option<String>,
}

fn issue(code: &'static str, msg: String) -> BuildIssue {
    BuildIssue::new(code, msg)
}

/// Axis-aligned box of a term when it is bounded by what this analysis understands.
fn term_box(surfs: &[Surf], hs: &[u32]) -> [f64; 6] {
    let mut b = [f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::INFINITY, f64::INFINITY];
    let mut cap = |lo: [f64; 3], hi: [f64; 3]| {
        for k in 0..3 {
            b[k] = b[k].max(lo[k]);
            b[3 + k] = b[3 + k].min(hi[k]);
        }
    };
    for &h in hs {
        let (s, pos) = ((h >> 1) as usize, h & 1 == 1);
        match &surfs[s] {
            Surf::Sphere { c, r } if !pos => cap([c[0] - r, c[1] - r, c[2] - r], [c[0] + r, c[1] + r, c[2] + r]),
            Surf::Torus { o, big, small, .. } if !pos => {
                let e = big + small;
                cap([o[0] - e, o[1] - e, o[2] - e], [o[0] + e, o[1] + e, o[2] + e]);
            }
            Surf::Cylinder { o, a, r } if !pos => {
                // Axis-aligned cylinder: bounded across the axis.
                for (k, ak) in a.iter().enumerate() {
                    if ak.abs() == 1.0 {
                        let mut lo = [f64::NEG_INFINITY; 3];
                        let mut hi = [f64::INFINITY; 3];
                        for j in (0..3).filter(|&j| j != k) {
                            lo[j] = o[j] - r;
                            hi[j] = o[j] + r;
                        }
                        cap(lo, hi);
                    }
                }
            }
            Surf::Plane { o, n } => {
                for k in 0..3 {
                    if n[k].abs() == 1.0 && n[(k + 1) % 3] == 0.0 && n[(k + 2) % 3] == 0.0 {
                        // sign * n.(p - o) >= 0
                        let side = if pos { n[k] } else { -n[k] };
                        let (mut lo, mut hi) = ([f64::NEG_INFINITY; 3], [f64::INFINITY; 3]);
                        if side > 0.0 {
                            lo[k] = o[k];
                        } else {
                            hi[k] = o[k];
                        }
                        cap(lo, hi);
                    }
                }
            }
            _ => {}
        }
    }
    b
}

fn box_finite(b: &[f64; 6]) -> bool {
    b.iter().all(|v| v.is_finite()) && (0..3).all(|k| b[k] <= b[3 + k])
}

/// Is the intersection of the term's half-spaces provably empty (sign conflicts)?
fn term_empty(surfs: &[Surf], hs: &[u32], tol: f64) -> bool {
    for (i, &h1) in hs.iter().enumerate() {
        let (s1, p1) = ((h1 >> 1) as usize, h1 & 1 == 1);
        for &h2 in &hs[i + 1..] {
            let (s2, p2) = ((h2 >> 1) as usize, h2 & 1 == 1);
            if s1 == s2 && p1 != p2 {
                return true;
            }
            match (&surfs[s1], &surfs[s2]) {
                (Surf::Plane { o: o1, n: n1 }, Surf::Plane { o: o2, n: n2 }) => {
                    let kappa = dot(*n1, *n2);
                    if (kappa.abs() - 1.0).abs() < 1e-12 {
                        let kappa = kappa.signum();
                        let (c1, c2) = (dot(*n1, *o1), dot(*n1, *o2));
                        // u = n1.p: s1 (u - c1) >= 0 and s2 kappa (u - c2) >= 0.
                        let (a, b) = (if p1 { 1.0 } else { -1.0 }, kappa * if p2 { 1.0 } else { -1.0 });
                        if (a > 0.0 && b < 0.0 && c1 > c2 + tol) || (a < 0.0 && b > 0.0 && c1 < c2 - tol) {
                            return true;
                        }
                    }
                }
                (Surf::Sphere { c: ca, r: ra }, Surf::Sphere { c: cb, r: rb }) => {
                    let dist = norm(sub(*ca, *cb));
                    if !p1 && !p2 && dist > ra + rb + tol {
                        return true;
                    }
                    if dist == 0.0 && ((!p1 && p2 && rb >= ra) || (p1 && !p2 && ra >= rb)) {
                        return true;
                    }
                }
                (Surf::Sphere { c, r }, Surf::Plane { o, n }) | (Surf::Plane { o, n }, Surf::Sphere { c, r }) => {
                    let (sp, pp) = if matches!(surfs[s1], Surf::Sphere { .. }) { (p1, p2) } else { (p2, p1) };
                    if !sp {
                        let side = if pp { 1.0 } else { -1.0 };
                        if side * dot(*n, sub(*c, *o)) + r < -tol {
                            return true;
                        }
                    }
                }
                _ => {}
            }
        }
    }
    false
}

impl Leaf {
    /// Compile `input`. Returns the leaf and the non-fatal issues; a fatal issue is
    /// `RtError::Build` with every issue found.
    pub(crate) fn compile(input: &CsgLeafInput<'_>, tol_m: &ModelTolerance) -> Result<(Self, Vec<BuildIssue>)> {
        let geom: &dyn GeomStore = input.geom;
        let mut fatal: Vec<BuildIssue> = Vec::new();
        let mut warn: Vec<BuildIssue> = Vec::new();
        let mut ids: BTreeMap<EntityId, u32> = BTreeMap::new();
        let mut surfs: Vec<Surf> = Vec::new();
        let mut surf_ids: Vec<EntityId> = Vec::new();
        let (mut term_off, mut hs_off, mut hs) = (vec![0u32], vec![0u32], Vec::<u32>::new());
        let mut bad: BTreeMap<EntityId, ()> = BTreeMap::new();
        for (ci, c) in input.cells.iter().enumerate() {
            for t in c.region.terms() {
                for h in &t.halfspaces {
                    let idx = match ids.get(&h.surface) {
                        Some(&i) => i,
                        None => match Surf::classify(geom, h.surface) {
                            Ok(s) => {
                                let i = surfs.len() as u32;
                                surfs.push(s);
                                surf_ids.push(h.surface);
                                ids.insert(h.surface, i);
                                i
                            }
                            Err(e) => {
                                if bad.insert(h.surface, ()).is_none() {
                                    let (code, msg) = match e {
                                        ClassifyError::Unknown => ("csg.unknown_surface", format!("cell {ci} names a surface the store does not hold as a CSG surface")),
                                        ClassifyError::Degenerate(m) => ("csg.degenerate_surface", format!("cell {ci}: {m}")),
                                    };
                                    fatal.push(issue(code, msg).with_entity(h.surface));
                                }
                                continue;
                            }
                        },
                    };
                    hs.push(idx << 1 | u32::from(h.sign == Sign::Positive));
                }
                hs_off.push(hs.len() as u32);
            }
            term_off.push((hs_off.len() - 1) as u32);
        }
        if !fatal.is_empty() {
            return Err(RtError::Build(fatal));
        }
        let n_cells = input.cells.len();
        let tol = tol_m.linear;

        // Regions by name, first appearance.
        let mut region_names: Vec<String> = Vec::new();
        let mut cell_region = Vec::with_capacity(n_cells);
        for c in input.cells {
            let r = region_names.iter().position(|n| *n == c.name).unwrap_or_else(|| {
                region_names.push(c.name.clone());
                region_names.len() - 1
            });
            cell_region.push(r as u32);
        }

        // Cell surface lists and the inverse CSR.
        let (mut cs_off, mut cs) = (vec![0u32], Vec::<u32>::new());
        let mut inv: Vec<Vec<u32>> = vec![Vec::new(); surfs.len()];
        for c in 0..n_cells {
            let (t0, t1) = (term_off[c] as usize, term_off[c + 1] as usize);
            let mut set: Vec<u32> = hs[hs_off[t0] as usize..hs_off[t1] as usize].iter().map(|h| h >> 1).collect();
            set.sort_unstable();
            set.dedup();
            for &s in &set {
                inv[s as usize].push(c as u32);
            }
            cs.extend_from_slice(&set);
            cs_off.push(cs.len() as u32);
        }
        let (mut sc_off, mut sc) = (vec![0u32], Vec::<u32>::new());
        for v in &inv {
            sc.extend_from_slice(v);
            sc_off.push(sc.len() as u32);
        }

        // Checks.
        let mut boxes: Vec<Option<[f64; 6]>> = Vec::with_capacity(n_cells);
        let mut cbox: Vec<[f64; 6]> = Vec::with_capacity(n_cells);
        let unbounded_box = [f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::INFINITY, f64::INFINITY];
        let mut n_empty = 0;
        for c in 0..n_cells {
            let (t0, t1) = (term_off[c] as usize, term_off[c + 1] as usize);
            let mut all_empty = true;
            let mut cb: Option<[f64; 6]> = Some([f64::INFINITY, f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY]);
            let mut ib = [f64::INFINITY, f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
            for t in t0..t1 {
                let th = &hs[hs_off[t] as usize..hs_off[t + 1] as usize];
                if term_empty(&surfs, th, tol) {
                    continue;
                }
                all_empty = false;
                let b = term_box(&surfs, th);
                for k in 0..3 {
                    ib[k] = ib[k].min(b[k]);
                    ib[3 + k] = ib[3 + k].max(b[3 + k]);
                }
                match (&mut cb, box_finite(&b)) {
                    (Some(acc), true) => {
                        for k in 0..3 {
                            acc[k] = acc[k].min(b[k]);
                            acc[3 + k] = acc[3 + k].max(b[3 + k]);
                        }
                    }
                    _ => cb = None,
                }
            }
            if all_empty {
                if n_empty < MAX_ISSUES {
                    warn.push(issue("csg.empty_cell", format!("cell {c} (`{}`) is provably empty", input.cells[c].name)));
                }
                n_empty += 1;
                cb = Some([f64::INFINITY, f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY]);
                // The index keeps a provably empty cell as a candidate everywhere, so it can never
                // disagree with the all-cells scan inside the on-face band.
                ib = unbounded_box;
            }
            boxes.push(cb);
            cbox.push(ib);
        }
        let mut n_unused = 0;
        for i in 0..geom.surface_count() {
            let Some(id) = EntityId::new(EntityKind::Surface, i as u64) else { break };
            if !ids.contains_key(&id) {
                if n_unused < MAX_ISSUES {
                    warn.push(issue("csg.unused_surface", format!("surface {i} of the store is named by no cell")).with_entity(id));
                }
                n_unused += 1;
            }
        }
        let unbounded = boxes.iter().filter(|b| b.is_none()).count();
        let complement = match input.complement {
            ComplementPolicy::Region(n) => Some(n.clone()),
            _ => None,
        };
        let big = 1e30;
        let bounds = match input.bounds {
            Some(b) => b,
            None if unbounded == 0 && n_cells > 0 => {
                let mut b = Aabb::empty();
                for cb in boxes.iter().flatten().filter(|b| b[0] <= b[3]) {
                    b = b.union(&Aabb { lo: Point3::new(cb[0], cb[1], cb[2]), hi: Point3::new(cb[3], cb[4], cb[5]) });
                }
                if b.lo.x > b.hi.x { Aabb { lo: Point3::new(-big, -big, -big), hi: Point3::new(big, big, big) } } else { b }
            }
            None => {
                let msg = format!("{unbounded} cell(s) are not provably bounded and the leaf has no bounds");
                if complement.is_some() {
                    return Err(RtError::Build(vec![issue("csg.unbounded", msg).suggest("give the leaf `bounds`; a complement region needs them")]));
                }
                warn.push(issue("csg.unbounded", msg).suggest("give the leaf `bounds`"));
                Aabb { lo: Point3::new(-big, -big, -big), hi: Point3::new(big, big, big) }
            }
        };
        if n_cells == 0 {
            return Err(RtError::Build(vec![issue("csg.empty_cell", "the leaf has no cells".into())]));
        }

        let mut leaf = Self {
            surfs,
            surf_ids,
            term_off,
            hs_off,
            hs,
            cs_off,
            cs,
            sc_off,
            sc,
            cell_region,
            region_names,
            volumes: input.cells.iter().map(|c| c.volume).collect(),
            radial: None,
            cbox,
            index: None,
            tol,
            ang: tol_m.angular,
            bounds,
            complement,
        };
        leaf.radial = leaf.detect_radial();
        leaf.set_index_threshold(DEFAULT_MIN_CELLS);
        Ok((leaf, warn))
    }

    /// Build the kd tiling when the leaf has at least `min_cells` cells, else drop it (the
    /// all-cells and all-surfaces paths, which are the oracle).
    pub(crate) fn set_index_threshold(&mut self, min_cells: usize) {
        self.index = None;
        if self.n_cells() >= min_cells {
            let b = &self.bounds;
            let bounds = [b.lo.x, b.lo.y, b.lo.z, b.hi.x, b.hi.y, b.hi.z];
            let cs = |c: usize| self.cell_surfs(c);
            self.index = Some(CellIndex::build(&self.cbox, bounds, self.tol, &cs));
        }
    }

    pub(crate) fn n_cells(&self) -> usize {
        self.cell_region.len()
    }
    pub(crate) fn n_surfs(&self) -> usize {
        self.surfs.len()
    }
    pub(crate) fn has_radial(&self) -> bool {
        self.radial.is_some()
    }
    /// Surfaces named by the cell, ascending.
    pub(crate) fn cell_surfs(&self, c: usize) -> &[u32] {
        &self.cs[self.cs_off[c] as usize..self.cs_off[c + 1] as usize]
    }
    /// Cells naming the surface, ascending.
    pub(crate) fn surf_cells(&self, s: usize) -> &[u32] {
        &self.sc[self.sc_off[s] as usize..self.sc_off[s + 1] as usize]
    }
    /// The terms of a cell as slices of encoded half-spaces.
    pub(crate) fn terms(&self, c: usize) -> impl Iterator<Item = &[u32]> {
        (self.term_off[c] as usize..self.term_off[c + 1] as usize).map(move |t| &self.hs[self.hs_off[t] as usize..self.hs_off[t + 1] as usize])
    }

    fn detect_radial(&self) -> Option<Radial> {
        let first = self.surfs.iter().position(|s| matches!(s, Surf::Sphere { .. } | Surf::Cylinder { .. }))?;
        let (sphere, o, a) = match &self.surfs[first] {
            Surf::Sphere { c, .. } => (true, *c, [0.0; 3]),
            Surf::Cylinder { o, a, .. } => (false, *o, *a),
            _ => return None,
        };
        let radius_of = |s: &Surf| -> Option<f64> {
            match s {
                Surf::Sphere { c, r } if sphere && *c == o => Some(*r),
                Surf::Cylinder { o: o2, a: a2, r } if !sphere && *o2 == o && *a2 == a => Some(*r),
                _ => None,
            }
        };
        let mut radii: Vec<f64> = self.surfs.iter().filter_map(radius_of).collect();
        if radii.len() < 2 {
            return None;
        }
        radii.sort_by(f64::total_cmp);
        radii.dedup();
        let m = radii.len();
        let (mut cand_off, mut cand) = (vec![0u32], Vec::<u32>::new());
        let rad_idx = |s: u32| radius_of(&self.surfs[s as usize]);
        for k in 0..=m {
            let lo_k = if k == 0 { 0.0 } else { radii[k - 1] };
            let hi_k = if k == m { f64::INFINITY } else { radii[k] };
            for c in 0..self.n_cells() {
                let covers = self.terms(c).any(|th| {
                    let (mut lower, mut upper) = (0.0f64, f64::INFINITY);
                    for &h in th {
                        if let Some(r) = rad_idx(h >> 1) {
                            if h & 1 == 1 {
                                lower = lower.max(r);
                            } else {
                                upper = upper.min(r);
                            }
                        }
                    }
                    lower <= lo_k && hi_k <= upper
                });
                if covers {
                    cand.push(c as u32);
                }
            }
            cand_off.push(cand.len() as u32);
        }
        Some(Radial { sphere, o, a, radii, cand_off, cand })
    }

    /// Side (+1 or -1) of surface `s` at `p`: by the sign of `sd` outside the on-face band,
    /// inside it by the direction `d` moves (`grad . d`), and a tangential `d` by the hashed
    /// direction of `p`.
    fn sense(&self, s: usize, p: V3, d: V3, cnt: &mut WorkCounters) -> i8 {
        cnt.surface_evals += 1;
        let v = self.surfs[s].sd(p);
        if v > self.tol {
            return 1;
        }
        if v < -self.tol {
            return -1;
        }
        let n = self.surfs[s].normal(p);
        let dn = dot(n, d);
        if dn > self.ang {
            return 1;
        }
        if dn < -self.ang {
            return -1;
        }
        let h = hashed_direction(Point3::new(p[0], p[1], p[2]));
        let hn = dot(n, [h.x, h.y, h.z]);
        if hn > 0.0 {
            1
        } else if hn < 0.0 {
            -1
        } else if v >= 0.0 {
            1
        } else {
            -1
        }
    }

    /// Does the cell contain `p`, with the side of one surface optionally forced?
    pub(crate) fn contains(&self, c: usize, p: V3, d: V3, force: Option<(u32, i8)>, cnt: &mut WorkCounters) -> bool {
        'terms: for th in self.terms(c) {
            for &h in th {
                let s = h >> 1;
                let sense = match force {
                    Some((fs, fsn)) if fs == s => fsn,
                    _ => self.sense(s as usize, p, d, cnt),
                };
                if (sense > 0) != (h & 1 == 1) {
                    continue 'terms;
                }
            }
            return true;
        }
        false
    }

    /// Classify `p` against every cell (the cold path): none, one, or an overlap.
    pub(crate) fn locate_all(&self, p: V3, d: V3, force: Option<(u32, i8)>, cnt: &mut WorkCounters) -> Found {
        let mut found = Found::None;
        for c in 0..self.n_cells() {
            cnt.locate_candidates += 1;
            if self.contains(c, p, d, force, cnt) {
                if found != Found::None {
                    return Found::Overlap;
                }
                found = Found::One(c as u32);
            }
        }
        found
    }

    /// Cold locate: the concentric fast path when it applies, else every cell.
    pub(crate) fn locate_cold(&self, p: V3, d: V3, cnt: &mut WorkCounters) -> Found {
        if let Some(r) = &self.radial {
            cnt.surface_evals += 1;
            let q = sub(p, r.o);
            let rho = if r.sphere { norm(q) } else { norm(madd(q, r.a, -dot(q, r.a))) };
            let k = r.radii.partition_point(|&x| x <= rho);
            let near_lo = k > 0 && rho - r.radii[k - 1] <= self.tol;
            let near_hi = k < r.radii.len() && r.radii[k] - rho <= self.tol;
            if !near_lo && !near_hi {
                let mut found = Found::None;
                for &c in &r.cand[r.cand_off[k] as usize..r.cand_off[k + 1] as usize] {
                    cnt.locate_candidates += 1;
                    if self.contains(c as usize, p, d, None, cnt) {
                        if found != Found::None {
                            return Found::Overlap;
                        }
                        found = Found::One(c);
                    }
                }
                return found;
            }
        }
        self.locate_scan(p, d, None, cnt)
    }

    /// Classify `p` against the cells of its tile when the index exists and holds `p`, else
    /// against every cell (counted in `slow_locates`). Same answer as [`Self::locate_all`].
    pub(crate) fn locate_scan(&self, p: V3, d: V3, force: Option<(u32, i8)>, cnt: &mut WorkCounters) -> Found {
        if let Some(cands) = self.index.as_ref().and_then(|ix| ix.cells_at(p)) {
            let mut found = Found::None;
            for &c in cands {
                cnt.locate_candidates += 1;
                if self.contains(c as usize, p, d, force, cnt) {
                    if found != Found::None {
                        return Found::Overlap;
                    }
                    found = Found::One(c);
                }
            }
            return found;
        }
        cnt.slow_locates += 1;
        self.locate_all(p, d, force, cnt)
    }

    /// The lowest surface within the on-face band of `p` among the cell's surfaces.
    pub(crate) fn on_face(&self, c: usize, p: V3) -> Option<u32> {
        self.cell_surfs(c).iter().copied().find(|&s| self.surfs[s as usize].sd(p).abs() <= self.tol)
    }

    /// The lowest surface of the leaf within the on-face band of `p` (for a point in the
    /// complement, which borders every surface).
    pub(crate) fn on_face_any(&self, p: V3) -> Option<u32> {
        (0..self.surfs.len() as u32).find(|&s| self.surfs[s as usize].sd(p).abs() <= self.tol)
    }

    /// Is `p` inside the leaf bounds by more than the on-face band?
    fn strictly_inside(&self, p: V3) -> bool {
        let (b, t) = (&self.bounds, self.tol);
        p[0] > b.lo.x + t && p[0] < b.hi.x - t && p[1] > b.lo.y + t && p[1] < b.hi.y - t && p[2] > b.lo.z + t && p[2] < b.hi.z - t
    }

    /// Across the surface `s` just crossed at `p` (moving toward side `sense`) out of `cur`
    /// (`None` for the complement): tests every cell of the `surface -> cells` CSR (no early
    /// exit, so two matches are an overlap); a crossing next to another surface, or
    /// strictly inside the bounds, falls back to the full-leaf scan (`slow_locates`).
    pub(crate) fn across(&self, cur: Option<u32>, s: u32, sense: i8, p: V3, d: V3, cnt: &mut WorkCounters) -> Next {
        let (mut n, mut one) = (0u32, 0u32);
        for &c in self.surf_cells(s as usize) {
            if Some(c) == cur {
                continue;
            }
            cnt.locate_candidates += 1;
            if self.contains(c as usize, p, d, Some((s, sense)), cnt) {
                n += 1;
                one = c;
            }
        }
        match n {
            0 => {
                // No neighbour matched. The full-leaf scan runs next to another surface of the
                // current cell, and anywhere strictly inside the leaf bounds (a coincident or
                // duplicate surface leaves the neighbour the CSR does not list); a crossing on
                // the bounds is a genuine exit.
                let edge = cur.is_some_and(|c| self.cell_surfs(c as usize).iter().any(|&o| o != s && self.surfs[o as usize].sd(p).abs() <= self.tol));
                if edge || self.strictly_inside(p) {
                    match self.locate_scan(p, d, Some((s, sense)), cnt) {
                        Found::One(c) if Some(c) != cur => Next::Cell(c),
                        Found::Overlap => Next::Overlap,
                        _ => Next::Exterior,
                    }
                } else {
                    Next::Exterior
                }
            }
            1 => Next::Cell(one),
            _ => Next::Overlap,
        }
    }
}
