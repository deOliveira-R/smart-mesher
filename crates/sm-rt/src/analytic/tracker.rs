//! [`AnalyticTracker`]: exact surface tracking in a CSG leaf.

use super::kernel::V3;
use super::leaf::{Found, Leaf, Next};
use crate::vec3::{dot, finite3, madd, scale, sub};
use rayon::prelude::*;
use sm_core::{Aabb, Point3, Vector3};
use sm_geom_api::Ray;
use sm_rt_api::{
    BoundaryDef, BoundarySelector, BoundarySpec, BuildIssue, BuildOptions, BuildReport, CacheKey, CellId, Closest, ContentHash, ContentHasher, CsgLeafInput, FaceAction,
    FaceId, LostCause, Located, PeriodicImage, RecoveryPolicy, RegionId, RegionInfo, RegionTable, ResponseKind, Result, RtError, Scratch, Step, StepEnd, SurfaceId, SurfaceInfo, SurfaceTable,
    TraceOptions, TrackBundle, TrackState, Tracker, TrackerCaps, TrackerInfo, check_direction, hashed_direction, lost,
};

/// Advance the position by `d` along the direction.
fn move_along(st: &mut TrackState, d: f64) {
    st.pos = Point3::new(st.pos.x + st.dir.x * d, st.pos.y + st.dir.y * d, st.pos.z + st.dir.z * d);
}

/// Builder version recorded in cache keys of analytic trackers.
pub const ANALYTIC_BUILDER_VERSION: u32 = 1;

enum Once {
    Done(Step),
    Fail(LostCause),
}

/// Exact tracker over one CSG leaf (`BuildInput::Hier` whose root is a single `Csg` leaf).
///
/// * Cells are disjoint (Q4). `locate` classifies a point by the cells' own surfaces; `step`
///   tests the cells across the crossed surface only. Two matches are `Lost(Overlap)`, never
///   resolved by order.
/// * `FaceId == SurfaceId`; a surface named by a `BoundarySpec` entity rule is a boundary where
///   `Reflect` and `Wrap` are applied in `step`; leaving the cells otherwise is a boundary with
///   the spec's default action (or, with `ComplementPolicy::Region`, an entry into the
///   complement cell inside the leaf bounds, whose six box faces follow the surfaces).
/// * No cache image: `export_cache` is `None`.
pub struct AnalyticTracker {
    leaf: Leaf,
    regions: RegionTable,
    surfaces: SurfaceTable,
    faces: Vec<FaceId>,
    /// Per surface (and six virtual box faces after them): the rule's action.
    actions: Vec<Option<FaceAction>>,
    default_action: FaceAction,
    complement: Option<RegionId>,
    info: TrackerInfo,
    recovery: RecoveryPolicy,
    block_size: usize,
    caps: TrackerCaps,
    ang: f64,
}

impl core::fmt::Debug for AnalyticTracker {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("AnalyticTracker").field("cells", &self.info.cells).field("surfaces", &self.leaf.n_surfs()).finish()
    }
}

fn a3(p: Point3) -> V3 {
    [p.x, p.y, p.z]
}
fn av(v: Vector3) -> V3 {
    [v.x, v.y, v.z]
}
fn pt(a: V3) -> Point3 {
    Point3::new(a[0], a[1], a[2])
}
fn vc(a: V3) -> Vector3 {
    Vector3::new(a[0], a[1], a[2])
}

/// Exit of the ray from the box (origin inside): `(t, face)` with faces `-x,+x,-y,+y,-z,+z`.
fn box_exit(o: V3, d: V3, b: &Aabb) -> (f64, usize) {
    let lo = [b.lo.x, b.lo.y, b.lo.z];
    let hi = [b.hi.x, b.hi.y, b.hi.z];
    let mut best = (f64::INFINITY, 0);
    for a in 0..3 {
        if d[a] > 0.0 {
            let t = (hi[a] - o[a]) / d[a];
            if t < best.0 {
                best = (t, 2 * a + 1);
            }
        } else if d[a] < 0.0 {
            let t = (lo[a] - o[a]) / d[a];
            if t < best.0 {
                best = (t, 2 * a);
            }
        }
    }
    (best.0.max(0.0), best.1)
}

impl AnalyticTracker {
    pub(crate) fn build(leaf_in: &CsgLeafInput<'_>, boundaries: &BoundarySpec, opts: &BuildOptions, key: CacheKey) -> Result<(Self, BuildReport)> {
        let (leaf, issues) = Leaf::compile(leaf_in, &opts.tol)?;
        let ns = leaf.n_surfs();
        let mut report = BuildReport::default();
        report.issues.extend(issues);

        // Boundary rules by surface handle (rules naming surfaces this leaf lacks do not apply).
        let local = |e: sm_core::EntityId| leaf.surf_ids.iter().position(|x| *x == e);
        let mut actions: Vec<Option<FaceAction>> = vec![None; ns + 6];
        for (sel, def) in &boundaries.rules {
            let e = match sel {
                BoundarySelector::Entity(e) => *e,
                _ => return Err(RtError::Unsupported("face-set boundary selectors on a CSG leaf")),
            };
            let Some(s) = local(e) else { continue };
            if let BoundaryDef::Respond(r) = def
                && !r.is_valid()
            {
                return Err(RtError::Build(vec![BuildIssue::new("boundary.bad_albedo", "albedo must be in [0, 1]").with_entity(e)]));
            }
            let mut missing = None;
            let act = def.resolve(|m| {
                let f = local(m).map(FaceId::new);
                if f.is_none() {
                    missing = Some(m);
                }
                f
            });
            let Some(act) = act else {
                let m = missing.map(|m| format!("{m}")).unwrap_or_default();
                return Err(RtError::Build(vec![BuildIssue::new("boundary.unknown_mate", format!("the mate surface {m} of a wrap rule is not a surface of the leaf")).with_entity(e)]));
            };
            actions[s] = if matches!(act, FaceAction::Transmit) { None } else { Some(act) };
        }
        let default_action = match boundaries.default {
            BoundaryDef::Wrap { .. } => return Err(RtError::Unsupported("a default Wrap boundary on a CSG leaf; name the surfaces")),
            BoundaryDef::Respond(r) if !r.is_valid() => return Err(RtError::Build(vec![BuildIssue::new("boundary.bad_albedo", "albedo must be in [0, 1]")])),
            BoundaryDef::Reflect => FaceAction::Reflect,
            BoundaryDef::Respond(r) => FaceAction::Respond(r),
            _ => FaceAction::Respond(ResponseKind::Vacuum),
        };

        // Tables.
        let mut regions: Vec<RegionInfo> = leaf.region_names.iter().map(RegionInfo::new).collect();
        let complement = leaf.complement.as_ref().map(|n| {
            regions.push(RegionInfo::new(n.clone()).as_implicit_complement());
            RegionId::new(regions.len() - 1)
        });
        let mut front = vec![RegionId::NONE; ns];
        let mut back = vec![RegionId::NONE; ns];
        for c in 0..leaf.n_cells() {
            let r = RegionId(leaf.cell_region[c]);
            for th in leaf.terms(c) {
                for &h in th {
                    let s = (h >> 1) as usize;
                    let slot = if h & 1 == 1 { &mut front[s] } else { &mut back[s] };
                    if slot.is_none() {
                        *slot = r;
                    }
                }
            }
        }
        let mut surfaces: Vec<SurfaceInfo> = (0..ns)
            .map(|s| {
                let mut si = SurfaceInfo::new(front[s], back[s]);
                si.entity = Some(leaf.surf_ids[s]);
                if let Some(a) = actions[s] {
                    si = si.with_boundary(a);
                }
                si
            })
            .collect();
        if let Some(c) = complement {
            for k in 0..6 {
                surfaces.push(SurfaceInfo::new(RegionId::NONE, c).with_boundary(default_action));
                actions[ns + k] = Some(default_action);
            }
        }

        // Structure hash by value: surface definitions, cell DNFs in order, names, volumes,
        // bounds, complement and the resolved boundary actions.
        let mut h = ContentHasher::new();
        h.write_u64(0xA7A1);
        h.write_u64(ns as u64);
        let mut words = Vec::new();
        for s in &leaf.surfs {
            words.clear();
            s.words(&mut words);
            h.write_u64(words.len() as u64);
            words.iter().for_each(|&w| h.write_u64(w));
        }
        h.write_u64(leaf.n_cells() as u64);
        for c in 0..leaf.n_cells() {
            h.write_u64(u64::from(leaf.cell_region[c]));
            h.write_u64(leaf.volumes[c].map_or(0, |_| 1));
            h.write_f64(leaf.volumes[c].unwrap_or(0.0));
            let nt = leaf.terms(c).count();
            h.write_u64(nt as u64);
            for th in leaf.terms(c) {
                h.write_u64(th.len() as u64);
                th.iter().for_each(|&w| h.write_u64(u64::from(w)));
            }
        }
        for n in &leaf.region_names {
            h.write_u64(n.len() as u64);
            h.write_bytes(n.as_bytes());
        }
        for b in [leaf.bounds.lo, leaf.bounds.hi] {
            h.write_f64(b.x);
            h.write_f64(b.y);
            h.write_f64(b.z);
        }
        h.write_u64(complement.map_or(0, |r| u64::from(r.0) + 1));
        default_action.hash_into(&mut h);
        for a in &actions[..ns] {
            match a {
                Some(a) => a.hash_into(&mut h),
                None => h.write_u64(99),
            }
        }
        h.write_f64(opts.tol.linear);
        let hash = h.finish();

        let mut info = TrackerInfo::new(leaf.bounds, leaf.n_cells(), hash);
        info.faces = ns;
        info.exact_faces = ns;
        info.tol = opts.tol;
        info.max_events = opts.max_events;
        info.key = Some(key);
        info.memory_bytes = ns * core::mem::size_of::<super::kernel::Surf>() + leaf.n_cells() * 64 + leaf.index.as_ref().map_or(0, super::cellindex::CellIndex::bytes);
        let wrap = actions.iter().flatten().any(|a| matches!(a, FaceAction::Wrap { .. }));
        let caps = if wrap { TrackerCaps::ANALYTIC.union(TrackerCaps::PERIODIC) } else { TrackerCaps::ANALYTIC };
        let t = Self {
            leaf,
            regions: RegionTable(regions),
            surfaces: SurfaceTable(surfaces),
            faces: (0..ns).map(FaceId::new).collect(),
            actions,
            default_action,
            complement,
            info,
            recovery: opts.recovery,
            block_size: opts.block_size.max(1) as usize,
            caps,
            ang: opts.tol.angular,
        };
        Ok((t, report))
    }

    fn n_surfs(&self) -> usize {
        self.leaf.n_surfs()
    }

    /// Number of cells.
    pub(crate) fn n_cells(&self) -> usize {
        self.leaf.n_cells()
    }

    /// Do cells lie on both sides of the surface (an interior face, not part of the outer
    /// boundary of the leaf)? The world uses it to tell a point on an outer face from one
    /// strictly inside the leaf.
    pub(crate) fn face_interior(&self, f: FaceId) -> bool {
        self.surfaces.0.get(f.get()).is_some_and(|s| !s.front.is_none() && !s.back.is_none())
    }

    /// Distance from the point to the surface of face `f` (a general quadric reports its absolute
    /// implicit value instead: callers use it against a tolerance band only).
    pub(crate) fn face_dist(&self, f: FaceId, p: V3) -> f64 {
        self.leaf.surfs.get(f.get()).map_or(f64::INFINITY, |s| s.closest(p).map_or_else(|| s.sd(p).abs(), |(d, _)| d))
    }

    /// First entry of the ray `o + t d` (`t >= 0`) into a cell of the leaf from outside it:
    /// `(t, surface)` of the earliest crossing of a surface that leads into a cell, ties by surface
    /// id. `None` if the ray never enters within `max_t`.
    pub(crate) fn enter_from_outside(&self, o: V3, d: V3, max_t: f64, sc: &mut Scratch) -> Option<(f64, u32)> {
        self.enter_impl(o, d, max_t, true, sc)
    }

    /// [`Self::enter_from_outside`] by the tile walk of the cell index (`indexed`, when the leaf
    /// has one) or by testing every surface (the oracle): the same answer. `(t, surface)`.
    pub fn entry_from_outside(&self, o: Point3, d: Vector3, max_t: f64, indexed: bool, scratch: &mut Scratch) -> Option<(f64, SurfaceId)> {
        self.enter_impl(a3(o), av(d), max_t, indexed, scratch).map(|(t, s)| (t, SurfaceId::new(s as usize)))
    }

    fn enter_impl(&self, o: V3, d: V3, max_t: f64, indexed: bool, sc: &mut Scratch) -> Option<(f64, u32)> {
        let mut best: Option<(f64, u32)> = None;
        let mut buf = [0.0f64; 4];
        let mut one = |s: u32, best: &mut Option<(f64, u32)>, sc: &mut Scratch| {
            let k = self.leaf.surfs[s as usize].hits(o, d, max_t, &mut buf);
            for &t in &buf[..k] {
                if best.is_some_and(|b| (t, s) >= (b.0, b.1)) {
                    continue;
                }
                let p = madd(o, d, t);
                let dn = dot(self.leaf.surfs[s as usize].normal(p), d);
                if dn == 0.0 {
                    continue;
                }
                let sense: i8 = if dn > 0.0 { 1 } else { -1 };
                if matches!(self.leaf.across(None, s, sense, p, d, &mut sc.counters), Next::Cell(_)) {
                    *best = Some((t, s));
                }
            }
        };
        match self.leaf.index.as_ref().filter(|_| indexed) {
            // A crossing that leads into a cell happens inside that cell's tile, whose surface
            // list holds the surface: walk the tiles front to back and stop once an event lies
            // strictly before the tile exit (every later tile starts at or after it).
            Some(ix) => ix.walk(o, d, max_t, |tile, t_exit| {
                for &s in ix.tile_surfs(tile) {
                    one(s, &mut best, sc);
                }
                best.is_some_and(|b| b.0 < t_exit)
            }),
            None => (0..self.n_surfs() as u32).for_each(|s| one(s, &mut best, sc)),
        }
        best
    }

    /// Rebuild the cell index for a threshold: built when the leaf has at least `min_cells`
    /// cells (default 64), dropped otherwise (`usize::MAX` gives the all-cells, all-surfaces
    /// paths, which are the oracle; `0` forces the index). Results do not depend on it.
    #[must_use]
    pub fn with_index_threshold(mut self, min_cells: usize) -> Self {
        let bytes = |t: &Self| t.leaf.index.as_ref().map_or(0, super::cellindex::CellIndex::bytes);
        let before = bytes(&self);
        self.leaf.set_index_threshold(min_cells);
        self.info.memory_bytes = self.info.memory_bytes - before + bytes(&self);
        self
    }

    /// Number of tiles of the cell index, `None` when the leaf has none.
    pub fn index_tiles(&self) -> Option<usize> {
        self.leaf.index.as_ref().map(super::cellindex::CellIndex::n_tiles)
    }

    /// Total cells listed over the tiles of the cell index (at most 16 times the cell count).
    pub fn index_listed_cells(&self) -> Option<usize> {
        self.leaf.index.as_ref().map(super::cellindex::CellIndex::listed_cells)
    }

    /// Hash of the cell index (node order and tile lists), `None` when the leaf has none.
    pub fn index_digest(&self) -> Option<ContentHash> {
        self.leaf.index.as_ref().map(super::cellindex::CellIndex::digest)
    }

    /// Locate `p` by the cell index (`indexed`) or by scanning every cell (the oracle), with the
    /// overlap flag: `(located, true)` when two cells claim the point (the cell is then none).
    ///
    /// # Errors
    /// `RtError::NonFinite` for a non-finite position or direction.
    pub fn locate_checked(&self, p: Point3, dir: Option<Vector3>, indexed: bool, scratch: &mut Scratch) -> Result<(Located, bool)> {
        let pa = a3(p);
        if !finite3(pa) {
            return Err(RtError::NonFinite("position"));
        }
        let d = match dir {
            Some(d) => av(d),
            None => {
                let h = hashed_direction(p);
                [h.x, h.y, h.z]
            }
        };
        if !finite3(d) {
            return Err(RtError::NonFinite("direction"));
        }
        if indexed {
            return Ok(self.locate_ov(pa, d, None, scratch));
        }
        let cnt = &mut scratch.counters;
        Ok(match self.leaf.locate_all(pa, d, None, cnt) {
            Found::One(c) => (Located::new(CellId(c), RegionId(self.leaf.cell_region[c as usize]), self.leaf.on_face(c as usize, pa).map(FaceId)), false),
            Found::Overlap => (Located::new(CellId::NONE, RegionId::NONE, None), true),
            Found::None => (
                match self.complement {
                    Some(r) if self.in_bounds(pa) => Located::new(CellId::COMPLEMENT, r, self.leaf.on_face_any(pa).map(FaceId)),
                    _ => Located::new(CellId::NONE, RegionId::NONE, None),
                },
                false,
            ),
        })
    }

    /// Kind of each surface (`plane`, `sphere`, `cylinder`, `cone`, `torus`, `quadric`), by
    /// surface id.
    pub fn surface_kinds(&self) -> Vec<&'static str> {
        self.leaf.surfs.iter().map(super::kernel::Surf::kind).collect()
    }

    /// Does `locate` use the concentric fast path (binary search on `r^2`-equivalent radii)?
    pub fn has_concentric_fast_path(&self) -> bool {
        self.leaf.has_radial()
    }

    /// `locate` without the concentric fast path (every cell is tested): the reference the fast
    /// path must agree with.
    pub fn locate_generic(&self, p: Point3, dir: Option<Vector3>, scratch: &mut Scratch) -> Result<Located> {
        let pa = a3(p);
        if !finite3(pa) {
            return Err(RtError::NonFinite("position"));
        }
        let d = match dir {
            Some(d) => av(d),
            None => {
                let h = hashed_direction(p);
                [h.x, h.y, h.z]
            }
        };
        let cnt = &mut scratch.counters;
        cnt.locates += 1;
        Ok(match self.leaf.locate_all(pa, d, None, cnt) {
            Found::One(c) => Located::new(CellId(c), RegionId(self.leaf.cell_region[c as usize]), self.leaf.on_face(c as usize, pa).map(FaceId)),
            Found::None => match self.complement {
                Some(r) if self.in_bounds(pa) => Located::new(CellId::COMPLEMENT, r, self.leaf.on_face_any(pa).map(FaceId)),
                _ => Located::new(CellId::NONE, RegionId::NONE, None),
            },
            Found::Overlap => Located::new(CellId::NONE, RegionId::NONE, None),
        })
    }

    fn region_of(&self, c: CellId) -> RegionId {
        if c == CellId::COMPLEMENT {
            self.complement.unwrap_or(RegionId::NONE)
        } else if c.is_none() || c.get() >= self.leaf.n_cells() {
            RegionId::NONE
        } else {
            RegionId(self.leaf.cell_region[c.get()])
        }
    }

    fn in_bounds(&self, p: V3) -> bool {
        let b = &self.leaf.bounds;
        let t = self.leaf.tol;
        p[0] >= b.lo.x - t && p[0] <= b.hi.x + t && p[1] >= b.lo.y - t && p[1] <= b.hi.y + t && p[2] >= b.lo.z - t && p[2] <= b.hi.z + t
    }

    /// Cell, region and the surface the point lies on (if any).
    fn locate_inner(&self, p: V3, d: V3, hint: Option<CellId>, sc: &mut Scratch) -> Located {
        self.locate_ov(p, d, hint, sc).0
    }

    /// `locate_inner` plus whether the point is matched by two cells (an overlap).
    pub(crate) fn locate_ov(&self, p: V3, d: V3, hint: Option<CellId>, sc: &mut Scratch) -> (Located, bool) {
        let cnt = &mut sc.counters;
        cnt.locates += 1;
        if let Some(h) = hint
            && !h.is_none()
            && h.get() < self.leaf.n_cells()
            && self.leaf.contains(h.get(), p, d, None, cnt)
            && self.leaf.on_face(h.get(), p).is_none()
        {
            return (Located::new(h, RegionId(self.leaf.cell_region[h.get()]), None), false);
        }
        match self.leaf.locate_cold(p, d, cnt) {
            Found::One(c) => {
                let on = self.leaf.on_face(c as usize, p).map(FaceId);
                (Located::new(CellId(c), RegionId(self.leaf.cell_region[c as usize]), on), false)
            }
            Found::None => (
                match self.complement {
                    Some(r) if self.in_bounds(p) => Located::new(CellId::COMPLEMENT, r, self.leaf.on_face_any(p).map(FaceId)),
                    _ => Located::new(CellId::NONE, RegionId::NONE, None),
                },
                false,
            ),
            Found::Overlap => (Located::new(CellId::NONE, RegionId::NONE, None), true),
        }
    }

    fn nearest(&self, p: V3) -> Option<SurfaceId> {
        let mut best: Option<(f64, usize)> = None;
        for (s, surf) in self.leaf.surfs.iter().enumerate() {
            let d = surf.sd(p).abs();
            if d.is_finite() && best.is_none_or(|b| d < b.0) {
                best = Some((d, s));
            }
        }
        best.map(|b| SurfaceId::new(b.1))
    }

    fn lost_step(&self, st: &TrackState, cause: LostCause, retries: u32) -> Step {
        let mut l = lost(st, cause).with_retries(retries);
        l.last_face = st.entered_via;
        let p = a3(st.pos);
        if finite3(p) {
            l.nearest_surface = self.nearest(p);
        }
        Step::new(0.0, st.cell, self.region_of(st.cell), StepEnd::Lost(l))
    }

    /// Append the roots of surface `s` in `(0, max_t]`, minus the duplicate of the crossing just made.
    fn push_hits(&self, s: u32, o: V3, d: V3, max_t: f64, entry: Option<u32>, sc: &mut Scratch) {
        let mut buf = [0.0f64; 4];
        sc.counters.surface_evals += 1;
        let k = self.leaf.surfs[s as usize].hits(o, d, max_t, &mut buf);
        for &t in &buf[..k] {
            if Some(s) == entry {
                // A duplicate of the crossing just made: same crossing direction, within the
                // on-face distance. A root crossing back (opposite direction) is a real
                // event, however close (a grazing chord).
                let surf = &self.leaf.surfs[s as usize];
                let (d0, dt) = (dot(surf.normal(o), d), dot(surf.normal(madd(o, d, t)), d));
                if (d0 > 0.0) == (dt > 0.0) && t <= self.leaf.tol / d0.abs().max(1e-3) {
                    continue;
                }
            }
            sc.hits.push((t, s));
        }
    }

    /// The first of the sorted `sc.hits` that is an event: a crossing out of the current cell
    /// into another cell (or, from the complement, into one), before the box exit.
    fn first_event(&self, cur_cell: Option<u32>, comp: bool, t_box: f64, o: V3, d: V3, sc: &mut Scratch) -> Option<(f64, u32, Next)> {
        for i in 0..sc.hits.len() {
            let (t, s) = sc.hits[i];
            if comp && t_box <= t {
                break;
            }
            let p = madd(o, d, t);
            let dn = dot(self.leaf.surfs[s as usize].normal(p), d);
            if dn == 0.0 {
                continue;
            }
            let sense: i8 = if dn > 0.0 { 1 } else { -1 };
            if let Some(c) = cur_cell
                && self.leaf.contains(c as usize, p, d, Some((s, sense)), &mut sc.counters)
            {
                continue;
            }
            let next = self.leaf.across(cur_cell, s, sense, p, d, &mut sc.counters);
            if comp && next == Next::Exterior {
                continue;
            }
            return Some((t, s, next));
        }
        None
    }

    fn step_once(&self, st: &mut TrackState, max_dist: f64, sc: &mut Scratch) -> Result<Once> {
        let (o, d) = (a3(st.pos), av(st.dir));
        let cur = st.cell;
        let comp = cur == CellId::COMPLEMENT;
        if cur.is_none() || (comp && self.complement.is_none()) {
            return Ok(Once::Fail(LostCause::NoCellContainsPoint));
        }
        if !comp && cur.get() >= self.leaf.n_cells() {
            return Err(RtError::UnknownId { kind: "cell", id: cur.0 });
        }
        let region = self.region_of(cur);
        let entry = st.entered_via.map(|f| f.0);
        let ns = self.n_surfs();
        let cur_cell = if comp { None } else { Some(cur.0) };

        let (t_box, k_box) = if comp { box_exit(o, d, &self.leaf.bounds) } else { (f64::INFINITY, 0) };
        let mut event: Option<(f64, u32, Next)> = None;
        match self.leaf.index.as_ref().filter(|_| comp) {
            // Complement with the cell index: the tiles along the ray front to back, each tested
            // against its own surfaces; the first valid event of a tile at or before its exit is
            // the first event of the ray (tiles are consecutive in `t`).
            Some(ix) => {
                let mut after = f64::NEG_INFINITY;
                ix.walk(o, d, t_box.min(max_dist), |tile, t_exit| {
                    sc.hits.clear();
                    for &s in ix.tile_surfs(tile) {
                        // Roots up to `max_dist` like the oracle path (a torus quartic is
                        // bracketed by its `t_max`), then cut to the tile.
                        self.push_hits(s, o, d, max_dist, entry, sc);
                    }
                    sc.hits.retain(|h| h.0 > after && h.0 <= t_exit);
                    sc.hits.sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
                    after = t_exit;
                    event = self.first_event(cur_cell, comp, t_box, o, d, sc);
                    event.is_some()
                });
            }
            None => {
                // Breakpoints: the first root of each relevant surface ahead of the position.
                sc.hits.clear();
                // The complement borders every cell: all surfaces are relevant.
                let (list, all): (&[u32], core::ops::Range<u32>) = if comp { (&[], 0..ns as u32) } else { (self.leaf.cell_surfs(cur.get()), 0..0) };
                for s in list.iter().copied().chain(all) {
                    self.push_hits(s, o, d, max_dist, entry, sc);
                }
                sc.hits.sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
                event = self.first_event(cur_cell, comp, t_box, o, d, sc);
            }
        }

        if comp && t_box <= max_dist && event.is_none_or(|e| t_box <= e.0) {
            // Leaving the leaf bounds through a virtual box face.
            let mut q = madd(o, d, t_box);
            let (axis, hi) = (k_box / 2, k_box % 2 == 1);
            q[axis] = if hi { [self.leaf.bounds.hi.x, self.leaf.bounds.hi.y, self.leaf.bounds.hi.z][axis] } else { [self.leaf.bounds.lo.x, self.leaf.bounds.lo.y, self.leaf.bounds.lo.z][axis] };
            let face = FaceId::new(ns + k_box);
            let action = self.default_action;
            st.pos = pt(q);
            st.entered_via = Some(face);
            if matches!(action, FaceAction::Reflect) {
                let mut nd = d;
                nd[axis] = -nd[axis];
                st.dir = vc(nd);
            }
            let end = StepEnd::Boundary { surface: SurfaceId(face.0), face, action };
            return Ok(Once::Done(Step::new(t_box, cur, region, end)));
        }
        let Some((t, s, next)) = event else {
            let dist = max_dist;
            if !dist.is_finite() {
                return Ok(Once::Fail(LostCause::NonFinite));
            }
            move_along(st, dist);
            if dist > 0.0 {
                st.entered_via = None;
            }
            return Ok(Once::Done(Step::new(dist, cur, region, StepEnd::Limit)));
        };
        if next == Next::Overlap {
            return Ok(Once::Fail(LostCause::Overlap));
        }
        let p = madd(o, d, t);
        let (face, surface) = (FaceId(s), SurfaceId(s));
        let rule = self.actions[s as usize];
        // Exterior of the cells: into the complement inside the bounds, else a boundary.
        let next = match (next, self.complement) {
            (Next::Exterior, Some(_)) if rule.is_none() && self.in_bounds(p) => Next::Cell(CellId::COMPLEMENT.0),
            (n, _) => n,
        };
        if rule.is_some() || next == Next::Exterior {
            let action = rule.unwrap_or(self.default_action);
            st.pos = pt(p);
            st.entered_via = Some(face);
            match action {
                FaceAction::Reflect => {
                    let n = self.leaf.surfs[s as usize].normal(p);
                    let nd = scale(n, 2.0 * dot(d, n));
                    st.dir = vc(sub(d, nd));
                }
                FaceAction::Wrap { transform, mate } => {
                    let (op, od, oe) = (st.pos, st.dir, st.entered_via);
                    let np = transform.apply_point(st.pos);
                    let nd = transform.apply_vector(st.dir);
                    // Neighbour search over the cells of the mate surface (no root re-locate).
                    let (nq, ndir) = (a3(np), av(nd));
                    let m = mate.get();
                    let dn = if m < ns { dot(self.leaf.surfs[m].normal(nq), ndir) } else { 0.0 };
                    let found = if m < ns { self.leaf.across(None, m as u32, if dn >= 0.0 { 1 } else { -1 }, nq, ndir, &mut sc.counters) } else { Next::Exterior };
                    let (cell, reg) = match found {
                        Next::Cell(c) => (CellId(c), RegionId(self.leaf.cell_region[c as usize])),
                        Next::Exterior if self.complement.is_some() && self.in_bounds(nq) => (CellId::COMPLEMENT, self.region_of(CellId::COMPLEMENT)),
                        Next::Exterior | Next::Overlap => {
                            st.pos = op;
                            st.dir = od;
                            st.entered_via = oe;
                            return Ok(Once::Fail(if found == Next::Overlap { LostCause::Overlap } else { LostCause::NoCellContainsPoint }));
                        }
                    };
                    st.pos = np;
                    st.dir = nd;
                    st.cell = cell;
                    st.region = reg;
                    st.entered_via = Some(mate);
                }
                _ => {}
            }
            let end = StepEnd::Boundary { surface, face, action };
            return Ok(Once::Done(Step::new(t, cur, region, end)));
        }
        let Next::Cell(nc) = next else { return Ok(Once::Fail(LostCause::NoCellContainsPoint)) };
        let to = CellId(nc);
        let to_region = self.region_of(to);
        st.pos = pt(p);
        st.cell = to;
        st.region = to_region;
        st.entered_via = Some(face);
        let end = if to_region == region { StepEnd::CellFace { face, to } } else { StepEnd::RegionBoundary { surface, face, to_region, to_cell: to } };
        Ok(Once::Done(Step::new(t, cur, region, end)))
    }
}

impl Tracker for AnalyticTracker {
    fn caps(&self) -> TrackerCaps {
        self.caps
    }
    fn info(&self) -> &TrackerInfo {
        &self.info
    }
    fn regions(&self) -> &RegionTable {
        &self.regions
    }
    fn surfaces(&self) -> &SurfaceTable {
        &self.surfaces
    }
    fn cell_region(&self, c: CellId) -> Option<RegionId> {
        if c == CellId::COMPLEMENT {
            return self.complement;
        }
        self.leaf.cell_region.get(c.get()).map(|&r| RegionId(r)).filter(|_| !c.is_none())
    }
    fn cell_volume(&self, c: CellId) -> Option<f64> {
        if c.is_none() {
            return None;
        }
        self.leaf.volumes.get(c.get()).copied().flatten()
    }
    fn export_cache(&self) -> Option<sm_rt_api::CacheImage> {
        None
    }
    fn surface_faces(&self, s: SurfaceId) -> &[FaceId] {
        self.faces.get(s.get()..s.get() + 1).unwrap_or(&[])
    }

    fn locate(&self, p: Point3, dir: Option<Vector3>, hint: Option<CellId>, scratch: &mut Scratch) -> Result<Located> {
        let pa = a3(p);
        if !finite3(pa) {
            return Err(RtError::NonFinite("position"));
        }
        let d = match dir {
            Some(d) => {
                let a = av(d);
                if !finite3(a) {
                    return Err(RtError::NonFinite("direction"));
                }
                a
            }
            None => {
                let h = hashed_direction(p);
                [h.x, h.y, h.z]
            }
        };
        Ok(self.locate_inner(pa, d, hint, scratch))
    }

    fn step(&self, st: &mut TrackState, max_dist: f64, scratch: &mut Scratch) -> Result<Step> {
        check_direction(st.dir, self.ang)?;
        if max_dist.is_nan() {
            return Err(RtError::NonFinite("max_dist"));
        }
        if max_dist < 0.0 {
            return Err(RtError::InvalidInput("max_dist must be >= 0"));
        }
        if !finite3(a3(st.pos)) {
            return Err(RtError::NonFinite("position"));
        }
        scratch.counters.steps += 1;
        let first = match self.step_once(st, max_dist, scratch)? {
            Once::Done(s) => return Ok(s),
            Once::Fail(c) => c,
        };
        // Overlaps are never retried by nudging.
        if first != LostCause::NoCellContainsPoint {
            return Ok(self.lost_step(st, first, 0));
        }
        let tries = self.recovery.tries();
        let (op, od) = (a3(st.pos), av(st.dir));
        let mut attempted = 0;
        for k in 0..tries {
            let nd = RecoveryPolicy::nudge_distance(&self.info.tol, k);
            if nd > max_dist {
                break;
            }
            attempted += 1;
            let q = madd(op, od, nd);
            let (loc, overlap) = self.locate_ov(q, od, None, scratch);
            if overlap {
                return Ok(self.lost_step(st, LostCause::Overlap, attempted));
            }
            if loc.cell.is_none() {
                continue;
            }
            let mut tmp = st.clone();
            tmp.pos = pt(q);
            tmp.cell = loc.cell;
            tmp.region = loc.region;
            tmp.entered_via = loc.on_face;
            match self.step_once(&mut tmp, max_dist - nd, scratch)? {
                Once::Done(mut s) => {
                    s.dist += nd;
                    *st = tmp;
                    scratch.counters.nudges += 1;
                    return Ok(s);
                }
                Once::Fail(LostCause::Overlap) => return Ok(self.lost_step(st, LostCause::Overlap, attempted)),
                Once::Fail(_) => {}
            }
        }
        let cause = if tries > 0 { LostCause::RecoveryFailed } else { first };
        Ok(self.lost_step(st, cause, attempted))
    }

    fn closest(&self, p: Point3, region: Option<RegionId>, _scratch: &mut Scratch) -> Result<Option<Closest>> {
        let pa = a3(p);
        if !finite3(pa) {
            return Err(RtError::NonFinite("position"));
        }
        if let Some(r) = region
            && r.get() >= self.regions.0.len()
        {
            return Err(RtError::UnknownId { kind: "region", id: r.0 });
        }
        let mut best: Option<Closest> = None;
        for (s, surf) in self.leaf.surfs.iter().enumerate() {
            if let Some(r) = region
                && !self.leaf.surf_cells(s).iter().any(|&c| self.leaf.cell_region[c as usize] == r.0)
            {
                continue;
            }
            let Some((dist, q)) = surf.closest(pa) else { return Err(RtError::Unsupported("closest on a general quadric")) };
            if best.is_none_or(|b| dist < b.dist) {
                best = Some(Closest::new(dist, SurfaceId::new(s), pt(q)));
            }
        }
        Ok(best)
    }

    fn face_normal(&self, f: FaceId, p: Point3) -> Result<Vector3> {
        let pa = a3(p);
        if !finite3(pa) {
            return Err(RtError::NonFinite("position"));
        }
        let ns = self.n_surfs();
        if f.get() < ns {
            let n = self.leaf.surfs[f.get()].normal(pa);
            if n == [0.0; 3] {
                return Err(RtError::InvalidInput("the surface is singular at this point"));
            }
            return Ok(vc(n));
        }
        if self.complement.is_some() && f.get() < ns + 6 {
            let k = f.get() - ns;
            let mut n = [0.0; 3];
            n[k / 2] = if k % 2 == 1 { 1.0 } else { -1.0 };
            return Ok(vc(n));
        }
        Err(RtError::UnknownId { kind: "face", id: f.0 })
    }

    fn periodic_image(&self, f: FaceId) -> Result<Option<PeriodicImage>> {
        if f.get() >= self.actions.len() {
            return Err(RtError::UnknownId { kind: "face", id: f.0 });
        }
        Ok(match self.actions[f.get()] {
            Some(FaceAction::Wrap { transform, mate }) if mate.get() < self.n_surfs() => {
                let to = self.leaf.surf_cells(mate.get()).first().map_or(CellId::NONE, |&c| CellId(c));
                Some(PeriodicImage::new(mate, to, transform))
            }
            _ => None,
        })
    }

    fn trace_bundle(&self, rays: &[Ray], opts: &TraceOptions) -> Result<TrackBundle> {
        // Fixed blocks, joined in index order: independent of the thread count.
        let parts: Vec<Result<TrackBundle>> = rays.par_chunks(self.block_size).map(|b| self.trace_bundle_block(b, opts, &mut Scratch::new())).collect();
        let mut all = TrackBundle::new();
        for p in parts {
            all.append(&p?)?;
        }
        Ok(all)
    }
}

/// The cache key of an analytic tracker (kind `Analytic`).
pub(crate) fn cache_key(leaf: &CsgLeafInput<'_>, spec_hash: ContentHash, boundaries: &BoundarySpec, opts: &BuildOptions) -> CacheKey {
    CacheKey::new(sm_rt_api::TrackerKind::Analytic, ANALYTIC_BUILDER_VERSION, leaf.content_hash(), spec_hash, boundaries.content_hash(), opts.content_hash())
}
