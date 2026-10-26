//! [`MeshTracker`]: the cell-walking tracker over the flat [`Core`].

use crate::core::{C_CONVEX, Core, NONE};
use crate::vec3::{add, dot, finite3, madd, norm, scale, sub};
use crate::walk::{ExitFail, GRAZE_COS, closest_on_triangle};
use rayon::prelude::*;
use sm_core::{Aabb, Point3, Vector3};
use sm_geom_api::Ray;
use sm_mesh_api::MeshRead;
use sm_rt_api::{
    BinMap, CacheImage, CellId, Closest, ContentHash, ContentHasher, FaceAction, FaceId, LostCause, LostInfo, Located, PeriodicImage, RecoveryPolicy, RegionId,
    RegionTable, Result, RtError, Scratch, Step, StepEnd, SurfaceId, SurfaceTable, TallySpec, TraceOptions, TrackBundle, TrackState, TrackerCaps, TrackerInfo,
    TrackerKind, hashed_direction,
};
use std::sync::atomic::{AtomicU64, Ordering};

/// Cell-walking tracker for polyhedral meshes and watertight facet soups (VOLUME_MESH +
/// EXACT_PLANAR). Built by [`crate::MeshTrackerBuilder`]; immutable and `Sync`.
#[derive(Debug)]
pub struct MeshTracker {
    pub(crate) core: Core,
    pub(crate) info: TrackerInfo,
    pub(crate) regions: RegionTable,
    pub(crate) surfaces: SurfaceTable,
    pub(crate) kind: TrackerKind,
    /// Region id of the implicit complement, when there is one.
    pub(crate) complement: Option<u32>,
    pub(crate) cell_sets: Vec<(String, Vec<u32>)>,
    pub(crate) block_size: usize,
    pub(crate) recovery: RecoveryPolicy,
    pub(crate) bounds: [f64; 6],
    nudges: AtomicU64,
}

/// The pieces of a tracker apart from its info and options.
pub(crate) struct Parts {
    pub core: Core,
    pub regions: RegionTable,
    pub surfaces: SurfaceTable,
    pub kind: TrackerKind,
    pub complement: Option<u32>,
    pub cell_sets: Vec<(String, Vec<u32>)>,
}

enum Once {
    Done(Step),
    Fail(LostCause),
}

enum Probe {
    In,
    Out { across: u32 },
    /// On (within tolerance of) faces; `rank` is 0 when the direction enters through all of them,
    /// 1 when it is tangent, 2 when it leaves.
    On { face: u32, rank: u8 },
}

impl MeshTracker {
    pub(crate) fn assemble(parts: Parts, mut info: TrackerInfo, block_size: usize, recovery: RecoveryPolicy) -> Self {
        let Parts { core, regions, surfaces, kind, complement, cell_sets } = parts;
        let mut b = [f64::INFINITY, f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
        for p in &core.pos {
            for k in 0..3 {
                b[k] = b[k].min(p[k]);
                b[3 + k] = b[3 + k].max(p[k]);
            }
        }
        info.bounds = Aabb { lo: Point3::new(b[0], b[1], b[2]), hi: Point3::new(b[3], b[4], b[5]) };
        Self { core, info, regions, surfaces, kind, complement, cell_sets, block_size, recovery, bounds: b, nudges: AtomicU64::new(0) }
    }

    /// Number of nudge recoveries performed by `step` since construction (a per-tracker
    /// diagnostic; it never influences a query result).
    pub fn nudge_count(&self) -> u64 {
        self.nudges.load(Ordering::Relaxed)
    }

    /// Which kind of input built this tracker.
    pub fn kind(&self) -> TrackerKind {
        self.kind
    }

    /// Structure hash over every array and table (thread-count independent).
    pub(crate) fn hash_structure(core: &Core, regions: &RegionTable, surfaces: &SurfaceTable, kind: TrackerKind, complement: Option<u32>, cell_sets: &[(String, Vec<u32>)]) -> ContentHash {
        let mut h = ContentHasher::new();
        h.write_u64(kind as u64);
        h.write_u64(core.pos.len() as u64);
        for p in &core.pos {
            for &c in p {
                h.write_f64(c);
            }
        }
        for col in [&core.fv_off, &core.fv, &core.f_left, &core.f_right, &core.f_surface, &core.f_action, &core.c_off, &core.c_face, &core.c_region] {
            h.write_u64(col.len() as u64);
            for &x in col.iter() {
                h.write_u64(u64::from(x));
            }
        }
        for pl in &core.f_plane {
            for &x in pl {
                h.write_f64(x);
            }
        }
        for a in &core.f_apex {
            for &x in a {
                h.write_f64(x);
            }
        }
        for &x in core.f_flags.iter().chain(&core.c_flags) {
            h.write_u64(u64::from(x));
        }
        for b in &core.c_bbox {
            for &x in b {
                h.write_f64(x);
            }
        }
        for &v in &core.c_volume {
            h.write_f64(v);
        }
        h.write_u64(core.actions.len() as u64);
        for a in &core.actions {
            a.hash_into(&mut h);
        }
        for bvh in [&core.face_bvh, &core.cell_bvh] {
            h.write_u64(bvh.bb.len() as u64);
            for b in &bvh.bb {
                for &x in b {
                    h.write_f64(x);
                }
            }
            for &x in bvh.link.iter().chain(&bvh.count).chain(&bvh.prims) {
                h.write_u64(u64::from(x));
            }
        }
        h.write_u64(regions.0.len() as u64);
        for r in &regions.0 {
            h.write_u64(r.entity.map_or(0, |e| e.to_raw()));
            h.write_u64(r.name.len() as u64);
            h.write_bytes(r.name.as_bytes());
            h.write_u64(u64::from(r.implicit_complement));
        }
        h.write_u64(surfaces.0.len() as u64);
        for s in &surfaces.0 {
            h.write_u64(s.entity.map_or(0, |e| e.to_raw()));
            h.write_u64(u64::from(s.front.0));
            h.write_u64(u64::from(s.back.0));
            match &s.boundary {
                Some(a) => {
                    h.write_u64(1);
                    a.hash_into(&mut h);
                }
                None => h.write_u64(0),
            }
        }
        h.write_u64(u64::from(complement.unwrap_or(NONE)));
        h.write_u64(cell_sets.len() as u64);
        for (n, cells) in cell_sets {
            h.write_u64(n.len() as u64);
            h.write_bytes(n.as_bytes());
            h.write_u64(cells.len() as u64);
            for &c in cells {
                h.write_u64(u64::from(c));
            }
        }
        h.finish()
    }

    #[inline]
    fn tol(&self) -> f64 {
        self.info.tol.linear
    }

    fn region_of(&self, c: u32) -> RegionId {
        if c == NONE {
            RegionId::NONE
        } else if c == CellId::COMPLEMENT.0 {
            self.complement.map_or(RegionId::NONE, RegionId)
        } else {
            RegionId(self.core.c_region[c as usize])
        }
    }

    fn in_bounds(&self, p: [f64; 3], pad: f64) -> bool {
        (0..3).all(|k| p[k] >= self.bounds[k] - pad && p[k] <= self.bounds[3 + k] + pad)
    }

    fn outside(&self, p: [f64; 3]) -> Located {
        match self.complement {
            Some(r) if self.in_bounds(p, self.tol()) => Located::new(CellId::COMPLEMENT, RegionId(r), None),
            _ => Located::new(CellId::NONE, RegionId::NONE, None),
        }
    }

    fn lost_info(&self, st: &TrackState, cause: LostCause, sc: &mut Scratch) -> LostInfo {
        let mut l = LostInfo::new(cause, st.pos, st.dir, st.cell);
        l.last_face = st.entered_via;
        let p = [st.pos.x, st.pos.y, st.pos.z];
        if finite3(p) {
            l.nearest_surface = self.closest_inner(p, None, sc).map(|c| c.surface);
        }
        if let Some(r) = self.regions.get(st.region) {
            l.classified_on = r.entity;
        }
        l
    }

    // ---- locate --------------------------------------------------------------------------

    fn convex_probe(&self, c: usize, p: [f64; 3], dir: &mut Option<[f64; 3]>) -> Probe {
        let tol = self.tol();
        let mut worst = (0.0, NONE);
        let mut on = NONE;
        let (mut into, mut leaving) = (true, false);
        for &f in self.core.cell_faces(c) {
            let pl = self.core.f_plane[f as usize];
            let s = self.core.sign(c, f as usize);
            let n = [pl[0], pl[1], pl[2]];
            let slack = s * (pl[3] - dot(n, p));
            if slack < -tol {
                if slack < worst.0 {
                    worst = (slack, f);
                }
            } else if slack <= tol {
                on = on.min(f);
                // Only a face the point is at or beyond constrains the direction: with positive
                // slack the particle is still inside and merely has a short chord to the face.
                if slack <= 0.0 {
                    let w = *dir.get_or_insert_with(|| {
                        let h = hashed_direction(Point3::new(p[0], p[1], p[2]));
                        [h.x, h.y, h.z]
                    });
                    let md = s * dot(n, w);
                    if md >= 0.0 {
                        into = false;
                    }
                    if md > GRAZE_COS {
                        leaving = true;
                    }
                }
            }
        }
        if worst.1 != NONE {
            let f = worst.1 as usize;
            let other = if self.core.f_left[f] as usize == c { self.core.f_right[f] } else { self.core.f_left[f] };
            return Probe::Out { across: other };
        }
        if on == NONE {
            Probe::In
        } else {
            Probe::On { face: on, rank: if into { 0 } else if leaving { 2 } else { 1 } }
        }
    }

    /// Inside test of a general cell by first-crossing orientation along `w` (majority of three
    /// directions when the first crossing is grazing).
    fn raycast_inside(&self, c: usize, p: [f64; 3], w0: [f64; 3]) -> bool {
        let one = |w: [f64; 3]| -> (bool, bool) {
            let mut best = (f64::INFINITY, false, false);
            for &f in self.core.cell_faces(c) {
                let s_pos = self.core.f_left[f as usize] as usize == c;
                let pl = self.core.f_plane[f as usize];
                let graze = dot([pl[0], pl[1], pl[2]], w).abs() < 1e-6;
                self.core.face_crossings(f as usize, p, w, |t, along| {
                    if t > 0.0 && t < best.0 {
                        best = (t, along == s_pos, graze);
                    }
                });
            }
            (best.1, best.2)
        };
        let (inside, graze) = one(w0);
        if !graze {
            return inside;
        }
        let mut votes = usize::from(inside);
        for k in 1..3 {
            let h = hashed_direction(Point3::new(p[k % 3], p[(k + 1) % 3], p[(k + 2) % 3]));
            if one([h.x, h.y, h.z]).0 {
                votes += 1;
            }
        }
        votes >= 2
    }

    fn generic_probe(&self, c: usize, p: [f64; 3], dir: &mut Option<[f64; 3]>) -> Probe {
        let tol = self.tol();
        let mut on = NONE;
        for &f in self.core.cell_faces(c) {
            if self.core.face_dist2(f as usize, p).0 <= tol * tol {
                on = on.min(f);
            }
        }
        let hashed = |p: [f64; 3]| {
            let h = hashed_direction(Point3::new(p[0], p[1], p[2]));
            [h.x, h.y, h.z]
        };
        if on == NONE {
            return if self.raycast_inside(c, p, hashed(p)) { Probe::In } else { Probe::Out { across: NONE } };
        }
        // Does the direction enter the cell? Probe just ahead (a chord that starts at the point and
        // is shorter than the tolerance) and 2 tol ahead (a point slightly outside that enters).
        let w = *dir.get_or_insert_with(|| hashed(p));
        let enters = [1e-3 * tol, 2.0 * tol].into_iter().any(|e| {
            let q = madd(p, w, e);
            self.raycast_inside(c, q, hashed(q))
        });
        Probe::On { face: on, rank: if enters { 0 } else { 2 } }
    }

    fn probe(&self, c: usize, p: [f64; 3], dir: &mut Option<[f64; 3]>) -> Probe {
        if self.core.c_flags[c] & C_CONVEX != 0 { self.convex_probe(c, p, dir) } else { self.generic_probe(c, p, dir) }
    }

    fn locate_mesh(&self, p: [f64; 3], dir: Option<[f64; 3]>, hint: Option<CellId>, sc: &mut Scratch) -> Located {
        let mut dir = dir;
        if let Some(h) = hint
            && h.get() < self.core.n_cells()
        {
            let mut cur = h.get();
            for _ in 0..8 {
                if self.core.c_flags[cur] & C_CONVEX == 0 {
                    break;
                }
                match self.convex_probe(cur, p, &mut dir) {
                    Probe::In => return Located::new(CellId::new(cur), RegionId(self.core.c_region[cur]), None),
                    Probe::Out { across } if across != NONE => cur = across as usize,
                    _ => break,
                }
            }
        }
        sc.hits.clear();
        {
            let (hits, ids) = (&mut sc.hits, &mut sc.ids);
            self.core.cell_bvh.visit_point(p, self.tol(), ids, |c| hits.push((0.0, c)));
        }
        sc.hits.sort_unstable_by_key(|h| h.1);
        // Cells the point is on the boundary of: the one the direction enters, else a tangent
        // one (lowest id each); if the direction only leaves, the point is outside.
        let mut best: [Option<(u32, u32)>; 3] = [None; 3];
        let mut lowest_face = NONE;
        for i in 0..sc.hits.len() {
            let c = sc.hits[i].1 as usize;
            match self.probe(c, p, &mut dir) {
                Probe::In => return Located::new(CellId::new(c), RegionId(self.core.c_region[c]), None),
                Probe::Out { .. } => {}
                Probe::On { face, rank } => {
                    lowest_face = lowest_face.min(face);
                    if best[rank as usize].is_none() {
                        best[rank as usize] = Some((c as u32, face));
                    }
                }
            }
        }
        match best[0].or(best[1]) {
            Some((c, f)) => Located::new(CellId(c), RegionId(self.core.c_region[c as usize]), Some(FaceId(f))),
            None => {
                let mut l = self.outside(p);
                l.on_face = (lowest_face != NONE).then_some(FaceId(lowest_face));
                l
            }
        }
    }

    /// Region cell on the `p` side of the first facet hit along `w`: `NONE` for the exterior.
    fn first_hit_cell(&self, p: [f64; 3], w0: [f64; 3], sc: &mut Scratch) -> u32 {
        let tol = self.tol();
        let cast = |w: [f64; 3], sc: &mut Scratch| -> (u32, bool) {
            match self.core.first_hit(p, w, tol, &mut sc.ids) {
                None => (NONE, false),
                Some((_, f, along)) => {
                    let fi = f as usize;
                    let pl = self.core.f_plane[fi];
                    let graze = dot([pl[0], pl[1], pl[2]], w).abs() < 1e-6;
                    (if along { self.core.f_left[fi] } else { self.core.f_right[fi] }, graze)
                }
            }
        };
        let (c0, graze) = cast(w0, sc);
        if !graze {
            return c0;
        }
        let mut votes = vec_votes(c0);
        for k in 1..3 {
            let h = hashed_direction(Point3::new(p[k % 3], p[(k + 1) % 3], p[(k + 2) % 3]));
            let (c, _) = cast([h.x, h.y, h.z], sc);
            votes.add(c);
        }
        votes.winner()
    }

    fn locate_facets(&self, p: [f64; 3], dir: Option<[f64; 3]>, sc: &mut Scratch) -> Located {
        let tol = self.tol();
        // Faces within tol of p.
        sc.hits.clear();
        {
            let (hits, ids) = (&mut sc.hits, &mut sc.ids);
            self.core.face_bvh.visit_point(p, tol, ids, |f| hits.push((0.0, f)));
        }
        let mut on = NONE;
        for i in 0..sc.hits.len() {
            let f = sc.hits[i].1;
            if self.core.face_dist2(f as usize, p).0 <= tol * tol {
                on = on.min(f);
            }
        }
        let hashed = |p: [f64; 3]| {
            let h = hashed_direction(Point3::new(p[0], p[1], p[2]));
            [h.x, h.y, h.z]
        };
        if on == NONE {
            let c = self.first_hit_cell(p, hashed(p), sc);
            return self.cell_located(c, p, None);
        }
        let w = dir.unwrap_or_else(|| hashed(p));
        // The region just ahead of the point; if that is outside, the region 2 tol ahead (see
        // `generic_probe`).
        let q1 = madd(p, w, 1e-3 * tol);
        let mut c = self.first_hit_cell(q1, hashed(q1), sc);
        if c == NONE {
            let q2 = madd(p, w, 2.0 * tol);
            c = self.first_hit_cell(q2, hashed(q2), sc);
        }
        self.cell_located(c, p, Some(FaceId(on)))
    }

    fn cell_located(&self, c: u32, p: [f64; 3], on: Option<FaceId>) -> Located {
        if c == NONE {
            let mut l = self.outside(p);
            l.on_face = on;
            l
        } else {
            Located::new(CellId(c), RegionId(self.core.c_region[c as usize]), on)
        }
    }

    pub(crate) fn locate_inner(&self, p: [f64; 3], dir: Option<[f64; 3]>, hint: Option<CellId>, sc: &mut Scratch) -> Located {
        if !self.in_bounds(p, self.tol()) {
            return Located::new(CellId::NONE, RegionId::NONE, None);
        }
        match self.kind {
            TrackerKind::Facets => self.locate_facets(p, dir, sc),
            _ => self.locate_mesh(p, dir, hint, sc),
        }
    }

    // ---- closest -------------------------------------------------------------------------

    fn closest_inner(&self, p: [f64; 3], region: Option<RegionId>, sc: &mut Scratch) -> Option<Closest> {
        let mut best: Option<(f64, u32, [f64; 3])> = None;
        let mut best2 = f64::INFINITY;
        self.core.face_bvh.visit_closest(p, best2, &mut sc.ids, |f| {
            let s = self.core.f_surface[f as usize];
            if s == NONE {
                return best2;
            }
            if let Some(r) = region {
                let info = &self.surfaces.0[s as usize];
                let comp = self.complement.is_some_and(|c| c == r.0);
                if !(info.front == r || info.back == r || (comp && info.back == RegionId::NONE)) {
                    return best2;
                }
            }
            let (d2, q) = self.core.face_dist2(f as usize, p);
            if best.is_none_or(|(bd, bf, _)| d2 < bd || (d2 == bd && f < bf)) {
                best = Some((d2, f, q));
                best2 = d2;
            }
            best2
        });
        best.map(|(d2, f, q)| Closest::new(d2.sqrt(), SurfaceId(self.core.f_surface[f as usize]), Point3::new(q[0], q[1], q[2])))
    }

    // ---- stepping ------------------------------------------------------------------------

    fn step_once(&self, st: &mut TrackState, max_dist: f64, sc: &mut Scratch) -> Result<Once> {
        let o = [st.pos.x, st.pos.y, st.pos.z];
        let d = [st.dir.x, st.dir.y, st.dir.z];
        if st.cell == CellId::COMPLEMENT {
            return Ok(self.complement_step(st, o, d, max_dist, sc));
        }
        if st.cell.is_none() {
            return Ok(Once::Fail(LostCause::NoCellContainsPoint));
        }
        let c = st.cell.get();
        if c >= self.core.n_cells() {
            return Err(RtError::UnknownId { kind: "cell", id: st.cell.0 });
        }
        let entry = st.entered_via.map_or(NONE, |f| f.0);
        let tol = self.tol();
        let ex = if self.core.c_flags[c] & C_CONVEX != 0 {
            self.core.exit_convex(c, o, d, entry, tol)
        } else {
            self.core.exit_generic(c, o, d, entry, tol, sc)
        };
        let ex = match ex {
            Ok(e) => e,
            Err(ExitFail::Outside) => return Ok(Once::Fail(LostCause::NoCellContainsPoint)),
            // `LostCause` has no sense-mismatch member in the frozen contract; both map here.
            Err(ExitFail::NoExit | ExitFail::Sense) => return Ok(Once::Fail(LostCause::NoExitFace)),
        };
        let region = RegionId(self.core.c_region[c]);
        if ex.t > max_dist {
            st.pos = vec_to_point(madd(o, d, max_dist));
            return Ok(Once::Done(Step::new(max_dist, st.cell, region, StepEnd::Limit)));
        }
        let f = ex.face as usize;
        let newpos = madd(o, d, ex.t);
        let other = if self.core.f_left[f] as usize == c { self.core.f_right[f] } else { self.core.f_left[f] };
        let face = FaceId(ex.face);
        let surface = SurfaceId(self.core.f_surface[f]);
        if other == NONE && self.core.f_action[f] == NONE && self.complement.is_some() {
            // Unclaimed exterior face: the particle leaves the body into the complement.
            let comp = self.complement.map_or(RegionId::NONE, RegionId);
            st.pos = vec_to_point(newpos);
            st.cell = CellId::COMPLEMENT;
            st.region = comp;
            st.entered_via = Some(face);
            let end = StepEnd::RegionBoundary { surface, face, to_region: comp, to_cell: CellId::COMPLEMENT };
            return Ok(Once::Done(Step::new(ex.t, CellId::new(c), region, end)));
        }
        if other == NONE {
            let ai = self.core.f_action[f];
            let action = if ai == NONE { FaceAction::Respond(sm_rt_api::ResponseKind::Vacuum) } else { self.core.actions[ai as usize] };
            st.pos = vec_to_point(newpos);
            st.entered_via = Some(face);
            match action {
                FaceAction::Reflect => {
                    let pl = self.core.f_plane[f];
                    let n = [pl[0], pl[1], pl[2]];
                    let nd = dot(d, n);
                    // Householder reflection of a unit vector is unit to rounding; no renormalisation,
                    // so axis-aligned directions reflect bit-exactly.
                    let r = sub(d, scale(n, 2.0 * nd));
                    st.dir = Vector3::new(r[0], r[1], r[2]);
                }
                FaceAction::Wrap { transform, mate } => {
                    st.pos = transform.apply_point(st.pos);
                    st.dir = transform.apply_vector(st.dir);
                    let m = mate.get();
                    if m >= self.core.n_faces() {
                        return Ok(Once::Fail(LostCause::InconsistentEntryFace));
                    }
                    let nc = self.core.f_left[m];
                    st.cell = CellId(nc);
                    st.region = RegionId(self.core.c_region[nc as usize]);
                    st.entered_via = Some(mate);
                }
                _ => {}
            }
            return Ok(Once::Done(Step::new(ex.t, CellId::new(c), region, StepEnd::Boundary { surface, face, action })));
        }
        let to_region = RegionId(self.core.c_region[other as usize]);
        st.pos = vec_to_point(newpos);
        st.cell = CellId(other);
        st.region = to_region;
        st.entered_via = Some(face);
        let end = if to_region == region { StepEnd::CellFace { face, to: CellId(other) } } else { StepEnd::RegionBoundary { surface, face, to_region, to_cell: CellId(other) } };
        Ok(Once::Done(Step::new(ex.t, CellId::new(c), region, end)))
    }

    fn complement_step(&self, st: &mut TrackState, o: [f64; 3], d: [f64; 3], max_dist: f64, sc: &mut Scratch) -> Once {
        let comp_region = self.complement.map_or(RegionId::NONE, RegionId);
        let limit = |st: &mut TrackState, t: f64| {
            st.pos = vec_to_point(madd(o, d, t));
            Once::Done(Step::new(t, CellId::COMPLEMENT, comp_region, StepEnd::Limit))
        };
        if let Some((t, f)) = self.core.enter_from_outside(o, d, self.tol(), &mut sc.ids) {
            let t = t.max(0.0);
            if t > max_dist {
                return limit(st, max_dist);
            }
            let fi = f as usize;
            let c = self.core.f_left[fi];
            st.pos = vec_to_point(madd(o, d, t));
            st.cell = CellId(c);
            st.region = RegionId(self.core.c_region[c as usize]);
            st.entered_via = Some(FaceId(f));
            let end = StepEnd::RegionBoundary { surface: SurfaceId(self.core.f_surface[fi]), face: FaceId(f), to_region: st.region, to_cell: st.cell };
            return Once::Done(Step::new(t, CellId::COMPLEMENT, comp_region, end));
        }
        let mut t_exit = f64::INFINITY;
        for k in 0..3 {
            if d[k] > 0.0 {
                t_exit = t_exit.min((self.bounds[3 + k] - o[k]) / d[k]);
            } else if d[k] < 0.0 {
                t_exit = t_exit.min((self.bounds[k] - o[k]) / d[k]);
            }
        }
        let t_exit = t_exit.max(0.0);
        if !t_exit.is_finite() || t_exit > max_dist {
            return limit(st, if max_dist.is_finite() { max_dist } else { 0.0 });
        }
        st.pos = vec_to_point(madd(o, d, t_exit));
        st.cell = CellId::NONE;
        st.region = RegionId::NONE;
        st.entered_via = None;
        Once::Done(Step::new(
            t_exit,
            CellId::COMPLEMENT,
            comp_region,
            StepEnd::RegionBoundary { surface: SurfaceId::NONE, face: FaceId::NONE, to_region: RegionId::NONE, to_cell: CellId::NONE },
        ))
    }
}

fn vec_to_point(a: [f64; 3]) -> Point3 {
    Point3::new(a[0], a[1], a[2])
}

/// Majority vote over up to three cell ids.
struct Votes {
    ids: [u32; 3],
    n: usize,
}

fn vec_votes(first: u32) -> Votes {
    Votes { ids: [first, 0, 0], n: 1 }
}

impl Votes {
    fn add(&mut self, c: u32) {
        if self.n < 3 {
            self.ids[self.n] = c;
            self.n += 1;
        }
    }
    fn winner(&self) -> u32 {
        for i in 0..self.n {
            if (0..self.n).filter(|&j| self.ids[j] == self.ids[i]).count() >= 2 {
                return self.ids[i];
            }
        }
        self.ids[0]
    }
}

impl sm_rt_api::Tracker for MeshTracker {
    fn caps(&self) -> TrackerCaps {
        TrackerCaps::VOLUME_MESH.union(TrackerCaps::EXACT_PLANAR)
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
            return self.complement.map(RegionId);
        }
        self.core.c_region.get(c.get()).map(|&r| RegionId(r))
    }
    fn cell_volume(&self, c: CellId) -> Option<f64> {
        self.core.c_volume.get(c.get()).copied()
    }
    fn mesh(&self) -> Option<&dyn MeshRead> {
        None
    }
    fn export_cache(&self) -> Option<CacheImage> {
        crate::image::export(self)
    }

    fn locate(&self, p: Point3, dir: Option<Vector3>, hint: Option<CellId>, scratch: &mut Scratch) -> Result<Located> {
        let pa = [p.x, p.y, p.z];
        if !finite3(pa) {
            return Err(RtError::NonFinite("position"));
        }
        let da = match dir {
            Some(d) => {
                let a = [d.x, d.y, d.z];
                if !finite3(a) {
                    return Err(RtError::NonFinite("direction"));
                }
                Some(a)
            }
            None => None,
        };
        Ok(self.locate_inner(pa, da, hint, scratch))
    }

    fn step(&self, st: &mut TrackState, max_dist: f64, scratch: &mut Scratch) -> Result<Step> {
        if max_dist.is_nan() || max_dist < 0.0 {
            return Err(RtError::InvalidInput("max_dist must be >= 0 and not NaN"));
        }
        if !finite3([st.pos.x, st.pos.y, st.pos.z]) || !finite3([st.dir.x, st.dir.y, st.dir.z]) {
            return Err(RtError::NonFinite("state"));
        }
        let region_of_state = |s: &Self, st: &TrackState| s.region_of(st.cell.0);
        let first = match self.step_once(st, max_dist, scratch)? {
            Once::Done(s) => return Ok(s),
            Once::Fail(cause) => cause,
        };
        // Recovery: attempt k moves from the original position by tol.linear * 2^k.
        let tries = self.recovery.tries();
        let origin = (st.pos, st.dir, st.cell, st.entered_via);
        for k in 0..tries {
            let nd = RecoveryPolicy::nudge_distance(&self.info.tol, k);
            if nd > max_dist {
                break;
            }
            let q = add([origin.0.x, origin.0.y, origin.0.z], scale([origin.1.x, origin.1.y, origin.1.z], nd));
            let loc = self.locate_inner(q, Some([origin.1.x, origin.1.y, origin.1.z]), Some(origin.2), scratch);
            if loc.cell.is_none() {
                continue;
            }
            let mut tmp = st.clone();
            tmp.pos = vec_to_point(q);
            tmp.cell = loc.cell;
            tmp.region = loc.region;
            tmp.entered_via = loc.on_face;
            if let Once::Done(mut s) = self.step_once(&mut tmp, max_dist - nd, scratch)? {
                s.dist += nd;
                *st = tmp;
                self.nudges.fetch_add(1, Ordering::Relaxed);
                return Ok(s);
            }
        }
        let cause = if tries > 0 { LostCause::RecoveryFailed } else { first };
        let info = self.lost_info(st, cause, scratch);
        let region = region_of_state(self, st);
        Ok(Step::new(0.0, st.cell, region, StepEnd::Lost(info)))
    }

    fn closest(&self, p: Point3, region: Option<RegionId>, scratch: &mut Scratch) -> Result<Option<Closest>> {
        let pa = [p.x, p.y, p.z];
        if !finite3(pa) {
            return Err(RtError::NonFinite("position"));
        }
        if let Some(r) = region
            && r.get() >= self.regions.0.len()
        {
            return Err(RtError::UnknownId { kind: "region", id: r.0 });
        }
        Ok(self.closest_inner(pa, region, scratch))
    }

    fn face_normal(&self, f: FaceId, p: Point3) -> Result<Vector3> {
        let pa = [p.x, p.y, p.z];
        if !finite3(pa) {
            return Err(RtError::NonFinite("position"));
        }
        let fi = f.get();
        if fi >= self.core.n_faces() {
            return Err(RtError::UnknownId { kind: "face", id: f.0 });
        }
        if self.core.f_flags[fi] & crate::core::F_PLANAR != 0 {
            let pl = self.core.f_plane[fi];
            return Ok(Vector3::new(pl[0], pl[1], pl[2]));
        }
        let mut best = (f64::INFINITY, [0.0, 0.0, 1.0]);
        for i in 0..self.core.n_tris(fi) {
            let (_, [a, b, c]) = self.core.tri(fi, i);
            let q = closest_on_triangle(pa, a, b, c);
            let dq = sub(pa, q);
            let d2 = dot(dq, dq);
            if d2 < best.0 {
                let n = crate::vec3::cross(sub(b, a), sub(c, a));
                let l = norm(n);
                if l > 0.0 {
                    best = (d2, scale(n, 1.0 / l));
                }
            }
        }
        Ok(Vector3::new(best.1[0], best.1[1], best.1[2]))
    }

    fn periodic_image(&self, f: FaceId) -> Result<Option<PeriodicImage>> {
        let fi = f.get();
        if fi >= self.core.n_faces() {
            return Err(RtError::UnknownId { kind: "face", id: f.0 });
        }
        let ai = self.core.f_action[fi];
        if ai == NONE {
            return Ok(None);
        }
        Ok(match self.core.actions[ai as usize] {
            FaceAction::Wrap { transform, mate } if mate.get() < self.core.n_faces() => {
                Some(PeriodicImage::new(mate, CellId(self.core.f_left[mate.get()]), transform))
            }
            _ => None,
        })
    }

    fn trace_bundle(&self, rays: &[Ray], opts: &TraceOptions) -> Result<TrackBundle> {
        let blocks: Vec<Result<TrackBundle>> = rays.par_chunks(self.block_size.max(1)).map_init(Scratch::new, |sc, chunk| self.trace_bundle_block(chunk, opts, sc)).collect();
        let mut out = TrackBundle::new();
        for b in blocks {
            out.append(&b?)?;
        }
        Ok(out)
    }

    fn bin_map(&self, spec: &TallySpec) -> Result<BinMap> {
        match spec {
            TallySpec::Cells => Ok(BinMap::identity(self.cell_count())),
            TallySpec::Regions => {
                let bins = (0..self.cell_count()).map(|c| self.cell_region(CellId::new(c)).map_or(BinMap::NO_BIN, |r| r.0)).collect();
                Ok(BinMap::new(self.regions.0.len(), bins))
            }
            TallySpec::MeshSets(names) => {
                let mut bins = vec![BinMap::NO_BIN; self.cell_count()];
                for (b, name) in names.iter().enumerate() {
                    let Some((_, cells)) = self.cell_sets.iter().find(|(n, _)| n == name) else {
                        return Err(RtError::InvalidInput("unknown cell set in TallySpec::MeshSets"));
                    };
                    for &c in cells {
                        if bins[c as usize] != BinMap::NO_BIN {
                            return Err(RtError::InvalidInput("cell sets of TallySpec::MeshSets overlap"));
                        }
                        bins[c as usize] = b as u32;
                    }
                }
                Ok(BinMap::new(names.len(), bins))
            }
            _ => Err(RtError::Unsupported("tally spec")),
        }
    }
}
