//! Reference implementor (feature `stubs`): a brute-force analytic world of spheres and boxes
//! in a box domain, optionally replicated as a rectangular lattice.
//!
//! It exists to pin down the *semantics* of [`Tracker`] (step events, tie rules, entry-face
//! exclusion, boundary actions, slot changes, determinism) with exact answers, not to be fast.
//! Every query tests every primitive of one element.
//!
//! # Layout
//! * Shapes are defined in an *element frame*. Without a lattice the element is the whole domain
//!   and the frame is the world frame. With a [`RectLattice`] the domain is the lattice box, the
//!   frame is centred on the element, and every shape must lie strictly inside the element
//!   (so lattice walls never cut a shape).
//! * Local cell `c < n` is shape `c` (the first shape containing a point wins, so list inner shapes
//!   first); local cell `n` is the background. The global cell of element `e` (x fastest) is
//!   `e * (n + 1) + c`. **Regions are prototypes**: region `c` is shared by all elements.
//! * Surface `i < n` is shape `i`'s surface (shared by all elements), surface `n + k` is domain
//!   face `k` (`-x,+x,-y,+y,-z,+z`). `FaceId` equals `SurfaceId`. Lattice walls are not model
//!   faces: crossing one is [`StepEnd::SlotChange`].
//! * Each domain face has its own [`FaceAction`]: `Respond(..)`, `Reflect` or `Wrap` (the mate
//!   must be a domain face).
//! * Hierarchy: with a lattice, `TrackState::levels` holds one [`Level`] (component 1 = the
//!   lattice, instance = element index, lattice = element indices).

use crate::boundary::{FaceAction, ResponseKind};
use crate::cache::{CacheImage, CacheKey, ContentHash, ContentHasher, SectionTag, TrackerKind};
use crate::error::{BuildIssue, LostCause, Result, RtError};
use crate::ids::{CellId, ComponentId, FaceId, InstanceId, RegionId, SurfaceId};
use crate::input::{BuildInput, BuildOptions, BuildReport, TrackerBuilder};
use crate::state::{
    Closest, LEVEL_INLINE, Level, Located, RegionInfo, RegionTable, Scratch, Step, StepEnd, SurfaceInfo, SurfaceTable,
    TrackState, TrackerCaps,
};
use crate::tally::TrackBundle;
use crate::tracker::{Tracker, TrackerInfo, check_direction, hashed_direction, lost, move_along};
use crate::world::{ComponentKind, LatticeKind, WorldSpec};
use sm_core::{Aabb, Point3, Transform, Vector3};
use sm_geom_api::{GeomError, ModelTolerance, Ray};
use smallvec::SmallVec;

/// An analytic primitive.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    /// Sphere.
    Sphere {
        /// Centre.
        center: Point3,
        /// Radius (positive).
        radius: f64,
    },
    /// Axis-aligned box.
    Box {
        /// Lower corner.
        lo: Point3,
        /// Upper corner.
        hi: Point3,
    },
}

fn comp(p: Point3, a: usize) -> f64 {
    [p.x, p.y, p.z][a]
}
fn compv(p: Vector3, a: usize) -> f64 {
    [p.x, p.y, p.z][a]
}
fn set_comp(p: &mut Point3, a: usize, v: f64) {
    match a {
        0 => p.x = v,
        1 => p.y = v,
        _ => p.z = v,
    }
}
fn shift(p: Point3, off: Vector3) -> Point3 {
    Point3::new(p.x - off.x, p.y - off.y, p.z - off.z)
}

impl Shape {
    /// Signed distance-like value: negative inside, positive outside (exact for spheres, the
    /// box version is the exact Euclidean distance outside and the nearest-face distance inside).
    pub fn signed_distance(&self, p: Point3) -> f64 {
        match *self {
            Shape::Sphere { center, radius } => (p - center).norm() - radius,
            Shape::Box { lo, hi } => {
                let mut out2 = 0.0;
                let mut inside = f64::INFINITY;
                for a in 0..3 {
                    let (l, h, x) = (comp(lo, a), comp(hi, a), comp(p, a));
                    let e = (l - x).max(x - h).max(0.0);
                    out2 += e * e;
                    inside = inside.min((x - l).min(h - x));
                }
                if out2 > 0.0 { out2.sqrt() } else { -inside }
            }
        }
    }

    /// Volume.
    pub fn volume(&self) -> f64 {
        match *self {
            Shape::Sphere { radius, .. } => 4.0 / 3.0 * core::f64::consts::PI * radius.powi(3),
            Shape::Box { lo, hi } => (hi.x - lo.x) * (hi.y - lo.y) * (hi.z - lo.z),
        }
    }

    /// Parameters `(t_in, t_out)` where the ray `o + t d` is inside, or `None` if it misses.
    pub fn interval(&self, o: Point3, d: Vector3) -> Option<(f64, f64)> {
        match *self {
            Shape::Sphere { center, radius } => {
                // Closest-approach form: the perpendicular offset m is formed directly, so a
                // grazing chord keeps full relative accuracy even when the origin is far away.
                let q = o - center;
                let tc = -q.dot(d);
                let m = q + d * tc;
                let m2 = m.dot(m);
                if m2 > radius * radius {
                    return None;
                }
                let mn = m2.sqrt();
                let half = ((radius - mn) * (radius + mn)).sqrt();
                Some((tc - half, tc + half))
            }
            Shape::Box { lo, hi } => {
                let (mut t0, mut t1) = (f64::NEG_INFINITY, f64::INFINITY);
                for a in 0..3 {
                    let (l, h, x, v) = (comp(lo, a), comp(hi, a), comp(o, a), compv(d, a));
                    if v == 0.0 {
                        if x < l || x > h {
                            return None;
                        }
                    } else {
                        let (a0, a1) = ((l - x) / v, (h - x) / v);
                        t0 = t0.max(a0.min(a1));
                        t1 = t1.min(a0.max(a1));
                    }
                }
                (t0 <= t1).then_some((t0, t1))
            }
        }
    }

    fn closest_point(&self, p: Point3) -> Point3 {
        match *self {
            Shape::Sphere { center, radius } => {
                let v = p - center;
                let n = v.norm();
                let u = if n > 0.0 { v * (1.0 / n) } else { Vector3::new(1.0, 0.0, 0.0) };
                center + u * radius
            }
            Shape::Box { lo, hi } => {
                let mut q = [comp(p, 0), comp(p, 1), comp(p, 2)];
                let inside = (0..3).all(|a| q[a] >= comp(lo, a) && q[a] <= comp(hi, a));
                if inside {
                    // Project onto the nearest face.
                    let (mut best, mut ba, mut bv) = (f64::INFINITY, 0, 0.0);
                    for (a, &qa) in q.iter().enumerate() {
                        for v in [comp(lo, a), comp(hi, a)] {
                            if (qa - v).abs() < best {
                                best = (qa - v).abs();
                                ba = a;
                                bv = v;
                            }
                        }
                    }
                    q[ba] = bv;
                } else {
                    for (a, qa) in q.iter_mut().enumerate() {
                        *qa = qa.clamp(comp(lo, a), comp(hi, a));
                    }
                }
                Point3::new(q[0], q[1], q[2])
            }
        }
    }

    /// Outward unit normal at (the point of the surface nearest to) `p`.
    fn outward_normal(&self, p: Point3) -> Result<Vector3> {
        match *self {
            Shape::Sphere { center, .. } => {
                let v = p - center;
                let n = v.norm();
                if n > 0.0 { Ok(v * (1.0 / n)) } else { Err(GeomError::Singular("sphere centre").into()) }
            }
            Shape::Box { lo, hi } => {
                let (mut best, mut out) = (f64::INFINITY, Vector3::new(1.0, 0.0, 0.0));
                for a in 0..3 {
                    for (v, s) in [(comp(lo, a), -1.0), (comp(hi, a), 1.0)] {
                        let dd = (comp(p, a) - v).abs();
                        if dd < best {
                            best = dd;
                            let mut n = [0.0; 3];
                            n[a] = s;
                            out = Vector3::new(n[0], n[1], n[2]);
                        }
                    }
                }
                Ok(out)
            }
        }
    }

    /// A point just outside the shape (used to find the region across its surface).
    fn outside_sample(&self) -> Point3 {
        match *self {
            Shape::Sphere { center, radius } => Point3::new(center.x + radius * (1.0 + 1e-6), center.y, center.z),
            Shape::Box { lo, hi } => Point3::new(hi.x + 1e-6 * (hi.x - lo.x), 0.5 * (lo.y + hi.y), 0.5 * (lo.z + hi.z)),
        }
    }

    fn hash_into(&self, h: &mut ContentHasher) {
        match *self {
            Shape::Sphere { center, radius } => {
                h.write_u64(1);
                for v in [center.x, center.y, center.z, radius] {
                    h.write_f64(v);
                }
            }
            Shape::Box { lo, hi } => {
                h.write_u64(2);
                for v in [lo.x, lo.y, lo.z, hi.x, hi.y, hi.z] {
                    h.write_f64(v);
                }
            }
        }
    }
}

/// A rectangular lattice of identical elements (the 2-level hierarchy of the stub).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RectLattice {
    /// Lower corner of element `(0, 0, 0)`.
    pub origin: Point3,
    /// Pitch along x, y, z.
    pub pitch: [f64; 3],
    /// Element counts.
    pub dims: [u32; 3],
}

impl RectLattice {
    fn bounds(&self) -> Aabb {
        let hi = |a: usize| comp(self.origin, a) + f64::from(self.dims[a]) * self.pitch[a];
        Aabb { lo: self.origin, hi: Point3::new(hi(0), hi(1), hi(2)) }
    }
}

/// Brute-force analytic tracker.
#[derive(Clone, Debug)]
pub struct AnalyticWorld {
    shapes: Vec<Shape>,
    lattice: Option<RectLattice>,
    /// Element box in the element frame.
    ebox: Aabb,
    domain: Aabb,
    actions: [FaceAction; 6],
    regions: RegionTable,
    surfaces: SurfaceTable,
    info: TrackerInfo,
    tol: ModelTolerance,
    block_size: usize,
}

fn box_exit(o: Point3, d: Vector3, b: &Aabb) -> (f64, usize) {
    let mut best = (f64::INFINITY, 0);
    for a in 0..3 {
        let v = compv(d, a);
        if v == 0.0 {
            continue;
        }
        let (t, k) = if v > 0.0 { ((comp(b.hi, a) - comp(o, a)) / v, 2 * a + 1) } else { ((comp(b.lo, a) - comp(o, a)) / v, 2 * a) };
        let t = t.max(0.0);
        if t < best.0 {
            best = (t, k);
        }
    }
    best
}

impl AnalyticWorld {
    /// A flat world: `shapes` (priority order, inner first) in `domain`, with one action per
    /// domain face (`-x,+x,-y,+y,-z,+z`).
    pub fn new(shapes: Vec<Shape>, domain: Aabb, actions: [FaceAction; 6]) -> Result<Self> {
        Self::build(shapes, None, domain, actions)
    }

    /// A flat world whose six faces all respond with `response`.
    pub fn simple(shapes: Vec<Shape>, domain: Aabb, response: ResponseKind) -> Result<Self> {
        Self::new(shapes, domain, [FaceAction::Respond(response); 6])
    }

    /// A 2-level world: `shapes` in the element frame, replicated over `lattice`; the domain is
    /// the lattice box.
    pub fn with_lattice(shapes: Vec<Shape>, lattice: RectLattice, actions: [FaceAction; 6]) -> Result<Self> {
        Self::build(shapes, Some(lattice), lattice.bounds(), actions)
    }

    /// Replace the model tolerance (`linear` is the on-face distance).
    pub fn with_tolerance(mut self, tol: ModelTolerance) -> Self {
        self.tol = tol;
        self.info.tol = tol;
        self
    }

    /// Rays per parallel block in `trace_bundle` (default 256).
    pub fn with_block_size(mut self, n: usize) -> Self {
        self.block_size = n.max(1);
        self
    }

    fn build(shapes: Vec<Shape>, lattice: Option<RectLattice>, domain: Aabb, actions: [FaceAction; 6]) -> Result<Self> {
        let n = shapes.len();
        let mut issues = Vec::new();
        if domain.is_empty() || (0..3).any(|a| comp(domain.hi, a) <= comp(domain.lo, a)) {
            issues.push(BuildIssue::new("analytic.domain", "domain box is empty"));
        }
        if let Some(l) = &lattice
            && (l.dims.contains(&0) || l.pitch.iter().any(|&p| !(p > 0.0 && p.is_finite())))
        {
            issues.push(BuildIssue::new("analytic.lattice", "lattice needs positive pitch and at least one element per axis"));
        }
        // Element box in the element frame.
        let ebox = match &lattice {
            Some(l) => Aabb {
                lo: Point3::new(-0.5 * l.pitch[0], -0.5 * l.pitch[1], -0.5 * l.pitch[2]),
                hi: Point3::new(0.5 * l.pitch[0], 0.5 * l.pitch[1], 0.5 * l.pitch[2]),
            },
            None => domain,
        };
        for (i, s) in shapes.iter().enumerate() {
            let ok = match *s {
                Shape::Sphere { center, radius } => radius > 0.0 && radius.is_finite() && center.x.is_finite() && center.y.is_finite() && center.z.is_finite(),
                Shape::Box { lo, hi } => (0..3).all(|a| comp(lo, a) < comp(hi, a)),
            };
            if !ok {
                issues.push(BuildIssue::new("analytic.shape", format!("shape {i} is degenerate or non-finite")));
                continue;
            }
            if lattice.is_some() {
                let inside = match *s {
                    Shape::Sphere { center, radius } => (0..3).all(|a| comp(center, a).abs() + radius < comp(ebox.hi, a)),
                    Shape::Box { lo, hi } => (0..3).all(|a| comp(lo, a) > comp(ebox.lo, a) && comp(hi, a) < comp(ebox.hi, a)),
                };
                if !inside {
                    issues.push(BuildIssue::new("analytic.shape_touches_wall", format!("shape {i} must lie strictly inside the lattice element")));
                }
            }
        }
        for (k, act) in actions.iter().enumerate() {
            match act {
                FaceAction::Transmit => issues.push(BuildIssue::new("analytic.action", format!("domain face {k} cannot transmit"))),
                FaceAction::Respond(r) if !r.is_valid() => issues.push(BuildIssue::new("analytic.action", format!("domain face {k}: albedo outside [0, 1]"))),
                FaceAction::Wrap { mate, .. } if !(n..n + 6).contains(&mate.get()) => {
                    issues.push(BuildIssue::new("analytic.action", format!("domain face {k}: wrap mate must be a domain face")));
                }
                _ => {}
            }
        }
        if !issues.is_empty() {
            return Err(RtError::Build(issues));
        }
        let mut regions: Vec<RegionInfo> = (0..n).map(|i| RegionInfo { entity: None, name: format!("shape{i}"), implicit_complement: false }).collect();
        regions.push(RegionInfo { entity: None, name: "background".into(), implicit_complement: true });
        let bg = RegionId::new(n);
        // Region across each shape surface: the first *other* shape containing a point just
        // outside it, else the background (exact for properly nested shapes).
        let mut surfaces: Vec<SurfaceInfo> = Vec::with_capacity(n + 6);
        for (i, s) in shapes.iter().enumerate() {
            let q = s.outside_sample();
            let across = shapes.iter().enumerate().position(|(j, o)| j != i && o.signed_distance(q) <= 0.0).map_or(bg, RegionId::new);
            surfaces.push(SurfaceInfo { entity: None, front: RegionId::new(i), back: across, boundary: None });
        }
        for act in actions {
            surfaces.push(SurfaceInfo { entity: None, front: bg, back: RegionId::NONE, boundary: Some(act) });
        }
        let diag = (domain.hi - domain.lo).norm();
        let tol = ModelTolerance::for_extent(diag);
        let n_elems = lattice.map_or(1, |l| l.dims.iter().map(|&d| d as usize).product());
        let mut w = Self {
            shapes,
            lattice,
            ebox,
            domain,
            actions,
            regions: RegionTable(regions),
            surfaces: SurfaceTable(surfaces),
            info: TrackerInfo::new(domain, n_elems * (n + 1), ContentHash(0)),
            tol,
            block_size: 256,
        };
        w.info.faces = n + 6;
        w.info.exact_faces = n + 6;
        w.info.prototypes = 1;
        w.info.copies = n_elems;
        w.info.tol = tol;
        w.info.memory_bytes = core::mem::size_of::<Self>() + w.shapes.len() * core::mem::size_of::<Shape>() + w.surfaces.0.len() * core::mem::size_of::<SurfaceInfo>();
        w.info.structure_hash = w.compute_hash();
        Ok(w)
    }

    fn compute_hash(&self) -> ContentHash {
        let mut h = ContentHasher::new();
        for s in &self.shapes {
            s.hash_into(&mut h);
        }
        match &self.lattice {
            Some(l) => {
                h.write_u64(1);
                for v in [l.origin.x, l.origin.y, l.origin.z] {
                    h.write_f64(v);
                }
                l.pitch.iter().for_each(|&v| h.write_f64(v));
                l.dims.iter().for_each(|&d| h.write_u64(u64::from(d)));
            }
            None => h.write_u64(0),
        }
        for p in [self.domain.lo, self.domain.hi] {
            for v in [p.x, p.y, p.z] {
                h.write_f64(v);
            }
        }
        for a in &self.actions {
            a.hash_into(&mut h);
        }
        h.finish()
    }

    fn n(&self) -> usize {
        self.shapes.len()
    }
    fn n_local(&self) -> usize {
        self.shapes.len() + 1
    }
    fn n_cells(&self) -> usize {
        self.info.cells
    }
    fn dom_surface(&self, k: usize) -> SurfaceId {
        SurfaceId::new(self.n() + k)
    }
    fn dims(&self) -> [usize; 3] {
        self.lattice.map_or([1, 1, 1], |l| [l.dims[0] as usize, l.dims[1] as usize, l.dims[2] as usize])
    }
    fn idx_of(&self, e: usize) -> [i32; 3] {
        let [nx, ny, _] = self.dims();
        [(e % nx) as i32, ((e / nx) % ny) as i32, (e / (nx * ny)) as i32]
    }
    fn lin_of(&self, idx: [i32; 3]) -> usize {
        let [nx, ny, _] = self.dims();
        idx[0] as usize + nx * (idx[1] as usize + ny * idx[2] as usize)
    }
    /// Element-frame origin in world coordinates.
    fn offset(&self, idx: [i32; 3]) -> Vector3 {
        match &self.lattice {
            Some(l) => Vector3::new(
                l.origin.x + (f64::from(idx[0]) + 0.5) * l.pitch[0],
                l.origin.y + (f64::from(idx[1]) + 0.5) * l.pitch[1],
                l.origin.z + (f64::from(idx[2]) + 0.5) * l.pitch[2],
            ),
            None => Vector3::new(0.0, 0.0, 0.0),
        }
    }
    /// World coordinate of the low (`hi = false`) or high wall of element `idx` along axis `a`.
    fn wall(&self, idx: [i32; 3], a: usize, hi: bool) -> f64 {
        match &self.lattice {
            Some(l) => comp(l.origin, a) + f64::from(idx[a] + i32::from(hi)) * l.pitch[a],
            None => comp(if hi { self.domain.hi } else { self.domain.lo }, a),
        }
    }
    fn levels_for(&self, e: usize, idx: [i32; 3]) -> SmallVec<[Level; LEVEL_INLINE]> {
        let mut v = SmallVec::new();
        if self.lattice.is_some() {
            v.push(Level { component: ComponentId(1), instance: InstanceId::new(e), lattice: idx });
        }
        v
    }

    /// First shape containing the element-frame point `p` (or the background).
    fn classify(&self, p: Point3) -> usize {
        self.shapes.iter().position(|s| s.signed_distance(p) <= 0.0).unwrap_or(self.n())
    }

    /// True if `p` is within the on-face tolerance of a lattice wall.
    fn near_wall(&self, p: Point3) -> bool {
        let Some(l) = &self.lattice else { return false };
        (0..3).any(|a| {
            let x = (comp(p, a) - comp(l.origin, a)) / l.pitch[a];
            (x - x.round()).abs() * l.pitch[a] <= self.tol.linear
        })
    }

    /// Element index of `p`; on a wall the direction decides (lowest index without one).
    fn elem_at(&self, p: Point3, dir: Option<Vector3>) -> Option<[i32; 3]> {
        if !self.domain.contains(p) {
            return None;
        }
        let Some(l) = &self.lattice else { return Some([0, 0, 0]) };
        let mut idx = [0i32; 3];
        for (a, slot) in idx.iter_mut().enumerate() {
            let x = (comp(p, a) - comp(l.origin, a)) / l.pitch[a];
            let r = x.round();
            let mut i = x.floor() as i32;
            if (x - r).abs() * l.pitch[a] <= self.tol.linear {
                i = match dir.map(|d| compv(d, a)) {
                    Some(v) if v > 0.0 => r as i32,
                    _ => r as i32 - 1,
                };
            }
            *slot = i.clamp(0, l.dims[a] as i32 - 1);
        }
        Some(idx)
    }

    /// Locate with the direction already resolved. Returns `(located, shape_face)`.
    fn locate_dir(&self, p: Point3, dir: Option<Vector3>) -> Option<Located> {
        let idx = self.elem_at(p, dir)?;
        let off = self.offset(idx);
        let pl = shift(p, off);
        let on_face = self.shapes.iter().position(|s| s.signed_distance(pl).abs() <= self.tol.linear).map(FaceId::new);
        let q = match (on_face, dir) {
            (Some(_), Some(d)) => Point3::new(pl.x + d.x * self.tol.linear * 10.0, pl.y + d.y * self.tol.linear * 10.0, pl.z + d.z * self.tol.linear * 10.0),
            _ => pl,
        };
        let c = self.classify(q);
        let e = self.lin_of(idx);
        Some(Located::new(CellId::new(e * self.n_local() + c), RegionId::new(c), on_face).with_levels(self.levels_for(e, idx)))
    }

    fn step_wall(&self, st: &mut TrackState, e: usize, t_wall: f64, k: usize, from: CellId, region: RegionId) -> Result<Step> {
        let idx = self.idx_of(e);
        let (a, hi) = (k / 2, k % 2 == 1);
        move_along(st, t_wall);
        set_comp(&mut st.pos, a, self.wall(idx, a, hi));
        st.entered_via = None;
        let neighbour = self.lattice.and_then(|l| {
            let ni = idx[a] + if hi { 1 } else { -1 };
            (0..l.dims[a] as i32).contains(&ni).then_some(ni)
        });
        if let Some(ni) = neighbour {
            let mut nidx = idx;
            nidx[a] = ni;
            let ne = self.lin_of(nidx);
            st.cell = CellId::new(ne * self.n_local() + self.n());
            st.region = RegionId::new(self.n());
            st.levels = self.levels_for(ne, nidx);
            return Ok(Step { dist: t_wall, cell: from, region, end: StepEnd::SlotChange { to_cell: st.cell } });
        }
        let action = self.actions[k];
        let surface = self.dom_surface(k);
        match action {
            FaceAction::Reflect => {
                let mut d = [st.dir.x, st.dir.y, st.dir.z];
                d[a] = -d[a];
                st.dir = Vector3::new(d[0], d[1], d[2]);
            }
            FaceAction::Wrap { transform, .. } => {
                let p = transform.apply_point(st.pos);
                let clamp = |lo: f64, hi: f64, v: f64| v.clamp(lo, hi);
                st.pos = Point3::new(
                    clamp(self.domain.lo.x, self.domain.hi.x, p.x),
                    clamp(self.domain.lo.y, self.domain.hi.y, p.y),
                    clamp(self.domain.lo.z, self.domain.hi.z, p.z),
                );
                let nd = transform.apply_vector(st.dir);
                st.dir = nd * (1.0 / nd.norm());
                let Some(loc) = self.locate_dir(st.pos, Some(st.dir)) else {
                    return Ok(Step { dist: t_wall, cell: from, region, end: StepEnd::Lost(lost(st, LostCause::NoCellContainsPoint)) });
                };
                st.cell = loc.cell;
                st.region = loc.region;
                st.levels = loc.levels;
            }
            FaceAction::Respond(_) | FaceAction::Transmit => {}
        }
        Ok(Step { dist: t_wall, cell: from, region, end: StepEnd::Boundary { surface, face: FaceId(surface.0), action } })
    }

    fn encode(&self) -> Vec<u8> {
        let mut o = Vec::new();
        let u = |o: &mut Vec<u8>, v: u64| o.extend_from_slice(&v.to_le_bytes());
        let f = |o: &mut Vec<u8>, v: f64| o.extend_from_slice(&v.to_le_bytes());
        u(&mut o, self.shapes.len() as u64);
        for s in &self.shapes {
            match *s {
                Shape::Sphere { center, radius } => {
                    u(&mut o, 1);
                    [center.x, center.y, center.z, radius, 0.0, 0.0].iter().for_each(|&v| f(&mut o, v));
                }
                Shape::Box { lo, hi } => {
                    u(&mut o, 2);
                    [lo.x, lo.y, lo.z, hi.x, hi.y, hi.z].iter().for_each(|&v| f(&mut o, v));
                }
            }
        }
        match &self.lattice {
            Some(l) => {
                u(&mut o, 1);
                [l.origin.x, l.origin.y, l.origin.z].iter().for_each(|&v| f(&mut o, v));
                l.pitch.iter().for_each(|&v| f(&mut o, v));
                l.dims.iter().for_each(|&d| u(&mut o, u64::from(d)));
            }
            None => u(&mut o, 0),
        }
        [self.domain.lo.x, self.domain.lo.y, self.domain.lo.z, self.domain.hi.x, self.domain.hi.y, self.domain.hi.z].iter().for_each(|&v| f(&mut o, v));
        for a in &self.actions {
            match *a {
                FaceAction::Transmit => u(&mut o, 0),
                FaceAction::Reflect => u(&mut o, 1),
                FaceAction::Wrap { transform, mate } => {
                    u(&mut o, 2);
                    transform.m.iter().flatten().for_each(|&v| f(&mut o, v));
                    [transform.t.x, transform.t.y, transform.t.z].iter().for_each(|&v| f(&mut o, v));
                    u(&mut o, u64::from(mate.0));
                }
                FaceAction::Respond(r) => {
                    u(&mut o, 3);
                    u(&mut o, u64::from(r.code()));
                    f(&mut o, if let ResponseKind::Albedo(a) = r { a } else { 0.0 });
                }
            }
        }
        o
    }

    fn decode(b: &[u8]) -> Result<Self> {
        struct Rd<'a>(&'a [u8]);
        impl Rd<'_> {
            fn u(&mut self) -> Result<u64> {
                let (h, t) = self.0.split_first_chunk::<8>().ok_or_else(|| RtError::CacheCorrupt("truncated stub section".into()))?;
                self.0 = t;
                Ok(u64::from_le_bytes(*h))
            }
            fn f(&mut self) -> Result<f64> {
                Ok(f64::from_bits(self.u()?))
            }
            fn pt(&mut self) -> Result<Point3> {
                Ok(Point3::new(self.f()?, self.f()?, self.f()?))
            }
        }
        let mut r = Rd(b);
        let n = r.u()? as usize;
        if n > b.len() {
            return Err(RtError::CacheCorrupt("shape count exceeds section".into()));
        }
        let mut shapes = Vec::with_capacity(n);
        for _ in 0..n {
            let tag = r.u()?;
            let v = [r.f()?, r.f()?, r.f()?, r.f()?, r.f()?, r.f()?];
            shapes.push(match tag {
                1 => Shape::Sphere { center: Point3::new(v[0], v[1], v[2]), radius: v[3] },
                2 => Shape::Box { lo: Point3::new(v[0], v[1], v[2]), hi: Point3::new(v[3], v[4], v[5]) },
                _ => return Err(RtError::CacheCorrupt("unknown shape tag".into())),
            });
        }
        let lattice = if r.u()? == 1 {
            let origin = r.pt()?;
            let pitch = [r.f()?, r.f()?, r.f()?];
            let dims = [r.u()? as u32, r.u()? as u32, r.u()? as u32];
            Some(RectLattice { origin, pitch, dims })
        } else {
            None
        };
        let domain = Aabb { lo: r.pt()?, hi: r.pt()? };
        let mut actions = [FaceAction::Transmit; 6];
        for a in &mut actions {
            *a = match r.u()? {
                0 => FaceAction::Transmit,
                1 => FaceAction::Reflect,
                2 => {
                    let mut m = [[0.0; 3]; 3];
                    for v in m.iter_mut().flatten() {
                        *v = r.f()?;
                    }
                    let t = Vector3::new(r.f()?, r.f()?, r.f()?);
                    let mut tr = Transform::IDENTITY;
                    tr.m = m;
                    tr.t = t;
                    FaceAction::Wrap { transform: tr, mate: FaceId(r.u()? as u32) }
                }
                3 => {
                    let code = r.u()?;
                    let alb = r.f()?;
                    FaceAction::Respond(match code {
                        1 => ResponseKind::Vacuum,
                        2 => ResponseKind::White,
                        3 => ResponseKind::Albedo(alb),
                        4 => ResponseKind::Reflective,
                        _ => return Err(RtError::CacheCorrupt("unknown response code".into())),
                    })
                }
                _ => return Err(RtError::CacheCorrupt("unknown action tag".into())),
            };
        }
        Self::build(shapes, lattice, domain, actions)
    }
}

impl Tracker for AnalyticWorld {
    fn caps(&self) -> TrackerCaps {
        let mut c = TrackerCaps::ANALYTIC.union(TrackerCaps::EXACT_PLANAR);
        if self.lattice.is_some() {
            c = c.union(TrackerCaps::HIERARCHY);
        }
        if self.actions.iter().any(|a| matches!(a, FaceAction::Wrap { .. })) {
            c = c.union(TrackerCaps::PERIODIC);
        }
        c
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
        (c.get() < self.n_cells()).then(|| RegionId::new(c.get() % self.n_local()))
    }
    /// Full primitive volume (nesting is not subtracted); `None` for the background.
    fn cell_volume(&self, c: CellId) -> Option<f64> {
        if c.get() >= self.n_cells() {
            return None;
        }
        self.shapes.get(c.get() % self.n_local()).map(Shape::volume)
    }

    fn locate(&self, p: Point3, dir: Option<Vector3>, _hint: Option<CellId>, _scratch: &mut Scratch) -> Result<Located> {
        if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
            return Err(RtError::NonFinite("position"));
        }
        let Some(idx0) = self.elem_at(p, None) else { return Ok(Located::new(CellId::NONE, RegionId::NONE, None)) };
        let pl0 = shift(p, self.offset(idx0));
        let on_shape = self.shapes.iter().any(|s| s.signed_distance(pl0).abs() <= self.tol.linear);
        // A point on any face with no direction gets the deterministic hashed one.
        let dir = match dir {
            None if on_shape || self.near_wall(p) => Some(hashed_direction(p)),
            d => d,
        };
        Ok(self.locate_dir(p, dir).unwrap_or_else(|| Located::new(CellId::NONE, RegionId::NONE, None)))
    }

    fn step(&self, st: &mut TrackState, max_dist: f64, scratch: &mut Scratch) -> Result<Step> {
        check_direction(st.dir, self.tol.angular)?;
        if max_dist.is_nan() {
            return Err(RtError::NonFinite("max_dist"));
        }
        let (from, region) = (st.cell, st.region);
        if from.is_none() || from.get() >= self.n_cells() {
            return Ok(Step { dist: 0.0, cell: from, region, end: StepEnd::Lost(lost(st, LostCause::NoCellContainsPoint)) });
        }
        let nl = self.n_local();
        let (e, c) = (from.get() / nl, from.get() % nl);
        let off = self.offset(self.idx_of(e));
        let (o, d) = (shift(st.pos, off), st.dir);
        let (t_wall, k_wall) = box_exit(o, d, &self.ebox);
        let skip = st.entered_via.map(|f| f.get());
        // Breakpoints (t, shape), only strictly ahead; the entry surface is ignored near t = 0.
        scratch.hits.clear();
        for (i, s) in self.shapes.iter().enumerate() {
            if let Some((a, b)) = s.interval(o, d) {
                for t in [a, b] {
                    let ignore = skip == Some(i) && t <= self.tol.linear;
                    if t > 0.0 && t < t_wall && !ignore {
                        scratch.hits.push((t, i as u32));
                    }
                }
            }
        }
        scratch.hits.sort_unstable_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
        // Walk intervals; the first one whose cell differs ends the step.
        let mut hit: Option<(f64, usize, usize)> = None;
        for w in 0..scratch.hits.len() {
            let (t, surf) = scratch.hits[w];
            let next = scratch.hits.get(w + 1).map_or(t_wall, |h| h.0);
            if next > t {
                let mid = 0.5 * (t + next);
                let q = Point3::new(o.x + d.x * mid, o.y + d.y * mid, o.z + d.z * mid);
                let cls = self.classify(q);
                if cls != c {
                    hit = Some((t, surf as usize, cls));
                    break;
                }
            }
        }
        match hit {
            Some((t, surf, cls)) if t <= max_dist => {
                move_along(st, t);
                st.cell = CellId::new(e * nl + cls);
                st.region = RegionId::new(cls);
                st.entered_via = Some(FaceId::new(surf));
                let end = StepEnd::RegionBoundary { surface: SurfaceId::new(surf), face: FaceId::new(surf), to_region: st.region, to_cell: st.cell };
                return Ok(Step { dist: t, cell: from, region, end });
            }
            None if t_wall <= max_dist => return self.step_wall(st, e, t_wall, k_wall, from, region),
            _ => {}
        }
        let dist = max_dist.min(t_wall);
        if !dist.is_finite() {
            return Ok(Step { dist: 0.0, cell: from, region, end: StepEnd::Lost(lost(st, LostCause::NonFinite)) });
        }
        move_along(st, dist);
        if dist > 0.0 {
            st.entered_via = None;
        }
        Ok(Step { dist, cell: from, region, end: StepEnd::Limit })
    }

    fn closest(&self, p: Point3, region: Option<RegionId>, _scratch: &mut Scratch) -> Result<Option<Closest>> {
        if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
            return Err(RtError::NonFinite("position"));
        }
        if let Some(r) = region
            && r.get() > self.n()
        {
            return Err(RtError::UnknownId { kind: "region", id: r.0 });
        }
        let Some(idx) = self.elem_at(p, None) else { return Ok(None) };
        let off = self.offset(idx);
        let pl = shift(p, off);
        let mut best: Option<Closest> = None;
        for (i, s) in self.shapes.iter().enumerate() {
            // The background (region n) is bounded by every shape.
            if region.is_some_and(|r| r.get() < self.n() && r.get() != i) {
                continue;
            }
            let dist = s.signed_distance(pl).abs();
            if best.is_none_or(|b| dist < b.dist) {
                let q = s.closest_point(pl);
                best = Some(Closest { dist, surface: SurfaceId::new(i), point: Point3::new(q.x + off.x, q.y + off.y, q.z + off.z) });
            }
        }
        Ok(best)
    }

    fn face_normal(&self, f: FaceId, p: Point3) -> Result<Vector3> {
        let n = self.n();
        if f.get() < n {
            let idx = self.elem_at(p, None).ok_or(RtError::NonFinite("point outside the domain"))?;
            return self.shapes[f.get()].outward_normal(shift(p, self.offset(idx)));
        }
        let k = f.get() - n;
        if k >= 6 {
            return Err(RtError::UnknownId { kind: "face", id: f.0 });
        }
        let mut v = [0.0; 3];
        v[k / 2] = if k.is_multiple_of(2) { -1.0 } else { 1.0 };
        Ok(Vector3::new(v[0], v[1], v[2]))
    }

    fn export_cache(&self) -> Option<CacheImage> {
        let key = self.info.key?;
        Some(CacheImage { key, sections: vec![(SectionTag::Private(0), self.encode())] })
    }

    fn trace_bundle(&self, rays: &[Ray], opts: &crate::tally::TraceOptions) -> Result<TrackBundle> {
        use rayon::prelude::*;
        // Fixed block partition independent of the thread count; blocks joined in index order.
        let parts: Vec<Result<TrackBundle>> = rays.par_chunks(self.block_size).map(|b| self.trace_bundle_block(b, opts, &mut Scratch::new())).collect();
        let mut all = TrackBundle::new();
        for p in parts {
            all.append(&p?);
        }
        Ok(all)
    }
}

/// Builds [`AnalyticWorld`]s from a [`WorldSpec`]. The leaf component 0 stands for the shape set
/// held by the builder; the world supplies the hierarchy: a lone `Leaf` (flat world in the
/// builder's domain) or a rectangular `Lattice` whose every slot is that leaf.
#[derive(Clone, Debug)]
pub struct StubBuilder {
    /// Shapes of the leaf (element frame).
    pub shapes: Vec<Shape>,
    /// Domain for a flat world (ignored with a lattice).
    pub domain: Aabb,
    /// Action per domain face.
    pub actions: [FaceAction; 6],
}

impl StubBuilder {
    fn parts(&self, w: &WorldSpec) -> Result<(Option<RectLattice>, Vec<BuildIssue>)> {
        let warnings = w.validate(1)?;
        let leaf = |c: ComponentId| matches!(w.components.get(c.get()), Some(ComponentKind::Leaf { input: 0 }));
        match &w.components[w.root.get()] {
            ComponentKind::Leaf { input: 0 } => Ok((None, warnings)),
            ComponentKind::Lattice(l) => {
                let LatticeKind::Rect { pitch } = l.kind else { return Err(RtError::Unsupported("hex lattice in the stub")) };
                if l.outer.is_some() || !l.fill.iter().copied().all(leaf) {
                    return Err(RtError::Unsupported("stub lattices need every slot to be leaf 0 and no outer fill"));
                }
                Ok((Some(RectLattice { origin: l.origin, pitch, dims: l.dims }), warnings))
            }
            _ => Err(RtError::Unsupported("stub worlds are a lone leaf or one rect lattice of leaf 0")),
        }
    }

    fn key(&self, input: &BuildInput<'_>, opts: &BuildOptions) -> Result<CacheKey> {
        let BuildInput::World(w) = input else { return Err(RtError::Unsupported("the stub builds from BuildInput::World only")) };
        let mut th = ContentHasher::new();
        self.shapes.iter().for_each(|s| s.hash_into(&mut th));
        let mut bh = ContentHasher::new();
        for p in [self.domain.lo, self.domain.hi] {
            [p.x, p.y, p.z].iter().for_each(|&v| bh.write_f64(v));
        }
        for a in &self.actions {
            a.hash_into(&mut bh);
        }
        Ok(CacheKey {
            kind: TrackerKind::Analytic,
            builder_version: self.builder_version(),
            topology_hash: th.finish(),
            hierarchy_hash: w.content_hash(),
            boundary_hash: bh.finish(),
            options_hash: opts.content_hash(),
        })
    }

    fn finish(&self, w: AnalyticWorld, key: CacheKey, opts: &BuildOptions) -> Box<dyn Tracker> {
        let mut w = w.with_tolerance(opts.tol).with_block_size(opts.block_size as usize);
        w.info.key = Some(key);
        w.info.max_events = opts.max_events;
        w.info.precision = crate::input::Precision::F64;
        Box::new(w)
    }
}

impl TrackerBuilder for StubBuilder {
    fn builder_version(&self) -> u32 {
        1
    }
    fn cache_key(&self, input: &BuildInput<'_>, opts: &BuildOptions) -> Result<CacheKey> {
        self.key(input, opts)
    }
    fn build(&self, input: &BuildInput<'_>, opts: &BuildOptions) -> Result<(Box<dyn Tracker>, BuildReport)> {
        let key = self.key(input, opts)?;
        let BuildInput::World(w) = input else { return Err(RtError::Unsupported("the stub builds from BuildInput::World only")) };
        let (lattice, warnings) = self.parts(w)?;
        let world = match lattice {
            Some(l) => AnalyticWorld::with_lattice(self.shapes.clone(), l, self.actions)?,
            None => AnalyticWorld::new(self.shapes.clone(), self.domain, self.actions)?,
        };
        let report = BuildReport { issues: warnings, ..BuildReport::default() };
        Ok((self.finish(world, key, opts), report))
    }
    fn load(&self, input: &BuildInput<'_>, opts: &BuildOptions, image: &CacheImage) -> Result<(Box<dyn Tracker>, BuildReport)> {
        let key = self.key(input, opts)?;
        image.check_key(&key)?;
        let data = image.section(SectionTag::Private(0)).ok_or_else(|| RtError::CacheCorrupt("missing stub section".into()))?;
        let world = AnalyticWorld::decode(data)?;
        let report = BuildReport { from_cache: true, ..BuildReport::default() };
        Ok((self.finish(world, key, opts), report))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boundary::wrap_transform;
    use crate::compat::{self, RayFire, RayOrientation};
    use crate::tally::{EndCode, Flow, Segment, TallySpec, TraceEnd, TraceOptions, TrackLengthTally};
    use crate::tracker::BoundaryAction;
    use crate::world::{InstancePolicy, Lattice};
    use sm_model_api::{Axis, Deck, DeckSide, Turns};

    fn p(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(x, y, z)
    }
    fn v(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3::new(x, y, z)
    }
    fn domain() -> Aabb {
        Aabb { lo: p(-5.0, -5.0, -5.0), hi: p(5.0, 5.0, 5.0) }
    }
    fn world(shapes: Vec<Shape>, r: ResponseKind) -> AnalyticWorld {
        AnalyticWorld::simple(shapes, domain(), r).unwrap()
    }
    fn vac() -> ResponseKind {
        ResponseKind::Vacuum
    }
    fn sphere(r: f64) -> Shape {
        Shape::Sphere { center: p(0.0, 0.0, 0.0), radius: r }
    }
    fn trace_all(w: &AnalyticWorld, o: Point3, d: Vector3) -> (Vec<Segment>, TraceEnd) {
        let mut sc = Scratch::new();
        let mut st = w.init_state(o, d, None, &mut sc).unwrap();
        let mut out = Vec::new();
        let end = w
            .trace(
                &mut st,
                &TraceOptions::default(),
                &mut |s: &Segment| {
                    out.push(*s);
                    Flow::Continue
                },
                &mut sc,
            )
            .unwrap();
        (out, end)
    }
    fn vacuum_end(e: &TraceEnd) -> bool {
        matches!(e, TraceEnd::Boundary { action: FaceAction::Respond(ResponseKind::Vacuum), .. })
    }

    #[test]
    fn sphere_chord_is_exact_for_many_impact_parameters() {
        let r = 2.0;
        let w = world(vec![sphere(r)], vac());
        for b in [0.0, 0.3, 1.0, 1.7, 1.999, 1.999999] {
            let (segs, end) = trace_all(&w, p(-5.0, b, 0.0), v(1.0, 0.0, 0.0));
            assert!(vacuum_end(&end));
            assert_eq!(segs.len(), 3, "b={b}");
            let want = 2.0 * ((r - b) * (r + b)).sqrt();
            let got = segs[1].length;
            assert!((got - want).abs() <= 1e-12 * want.max(1e-3), "b={b} got={got} want={want}");
            assert_eq!(segs[1].cell, CellId(0));
            let total: f64 = segs.iter().map(|s| s.length).sum();
            assert!((total - 10.0).abs() < 1e-12);
            assert!((segs[1].t0 - (5.0 - want / 2.0)).abs() < 1e-12);
        }
    }

    #[test]
    fn oblique_chord_through_sphere_and_miss() {
        let w = world(vec![sphere(2.0)], vac());
        let s3 = 3f64.sqrt();
        let (segs, _) = trace_all(&w, p(-3.0, -3.0, -3.0), v(1.0 / s3, 1.0 / s3, 1.0 / s3));
        assert!((segs.iter().find(|s| s.cell == CellId(0)).unwrap().length - 4.0).abs() < 1e-12);
        let (segs, _) = trace_all(&w, p(-5.0, 2.5, 0.0), v(1.0, 0.0, 0.0));
        assert_eq!(segs.len(), 1);
        assert!((segs[0].length - 10.0).abs() < 1e-12);
    }

    #[test]
    fn nested_shells_partition_the_outer_chord() {
        let (ri, ro) = (1.0, 2.0);
        let w = world(vec![sphere(ri), sphere(ro)], vac());
        let b = 0.6;
        let (segs, _) = trace_all(&w, p(-5.0, b, 0.0), v(1.0, 0.0, 0.0));
        let cells: Vec<u32> = segs.iter().map(|s| s.cell.0).collect();
        assert_eq!(cells, vec![2, 1, 0, 1, 2]);
        let inner = 2.0 * (ri * ri - b * b).sqrt();
        let outer = 2.0 * (ro * ro - b * b).sqrt();
        assert!((segs[2].length - inner).abs() < 1e-12);
        assert!((segs[1].length + segs[2].length + segs[3].length - outer).abs() < 1e-12);
        // Surface table of the nested shells: inner sphere's normal points into the outer shell.
        assert_eq!(w.surfaces().get(SurfaceId(0)).unwrap().back, RegionId(1));
        assert_eq!(w.surfaces().get(SurfaceId(1)).unwrap().back, RegionId(2));
        assert_eq!(w.surfaces().get(SurfaceId(0)).unwrap().front, RegionId(0));
    }

    #[test]
    fn box_chord_and_ray_starting_inside() {
        let w = world(vec![Shape::Box { lo: p(-1.0, -1.0, -1.0), hi: p(1.0, 1.0, 1.0) }], vac());
        let (segs, _) = trace_all(&w, p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0));
        assert_eq!(segs.len(), 2);
        assert!((segs[0].length - 1.0).abs() < 1e-14 && (segs[1].length - 4.0).abs() < 1e-14);
        assert_eq!(segs[0].face_in, None);
        assert_eq!(segs[1].face_in, Some(FaceId(0)));
    }

    #[test]
    fn fire_stops_at_region_boundary_and_peek_agrees() {
        let w = world(vec![sphere(2.0)], vac());
        let mut sc = Scratch::new();
        let mut st = w.init_state(p(-5.0, 0.0, 0.0), v(1.0, 0.0, 0.0), None, &mut sc).unwrap();
        // Peek does not move the state.
        let before = st.clone();
        let crate::state::Peek::Hit(h) = w.distance_to_boundary(&st, f64::INFINITY, &mut sc).unwrap() else { panic!() };
        assert_eq!(st, before);
        assert!((h.dist - 3.0).abs() < 1e-12 && h.to_region == RegionId(0) && h.action.is_none());
        let r = w.fire(&mut st, f64::INFINITY, 100, &mut sc).unwrap();
        assert!((r.dist - h.dist).abs() < 1e-15);
        let StepEnd::RegionBoundary { surface, to_region, .. } = r.end else { panic!("{:?}", r.end) };
        assert_eq!((surface, to_region), (h.surface, RegionId(0)));
        assert_eq!(w.surfaces().next_region(surface, RegionId(1)), Some(RegionId(0)));
        // Limit shorter than the boundary.
        let mut st = w.init_state(p(-5.0, 0.0, 0.0), v(1.0, 0.0, 0.0), None, &mut sc).unwrap();
        let r = w.fire(&mut st, 1.0, 100, &mut sc).unwrap();
        assert!(matches!(r.end, StepEnd::Limit) && (r.dist - 1.0).abs() < 1e-15);
        assert!(matches!(w.distance_to_boundary(&st, 0.5, &mut sc).unwrap(), crate::state::Peek::Limit));
        assert!(w.point_in_region(RegionId(0), p(0.5, 0.0, 0.0), None, &mut sc).unwrap());
        assert!(!w.point_in_region(RegionId(0), p(3.0, 0.0, 0.0), None, &mut sc).unwrap());
    }

    #[test]
    fn locate_on_surface_uses_direction_or_the_hashed_default() {
        let w = world(vec![sphere(2.0)], vac());
        let mut sc = Scratch::new();
        let on = p(-2.0, 0.0, 0.0);
        let inward = w.locate(on, Some(v(1.0, 0.0, 0.0)), None, &mut sc).unwrap();
        let outward = w.locate(on, Some(v(-1.0, 0.0, 0.0)), None, &mut sc).unwrap();
        assert_eq!((inward.cell, outward.cell), (CellId(0), CellId(1)));
        assert_eq!(inward.on_face, Some(FaceId(0)));
        assert_eq!(w.locate(p(9.0, 0.0, 0.0), None, None, &mut sc).unwrap().cell, CellId::NONE);
        // dir = None on a face: same answer as passing the hashed direction explicitly, every time.
        let none = w.locate(on, None, None, &mut sc).unwrap();
        let hashed = w.locate(on, Some(hashed_direction(on)), None, &mut sc).unwrap();
        assert_eq!(none, hashed);
        assert_eq!(none, w.locate(on, None, None, &mut sc).unwrap());
    }

    #[test]
    fn closest_to_sphere_and_normal() {
        let w = world(vec![sphere(2.0)], vac());
        let mut sc = Scratch::new();
        let c = w.closest(p(0.0, 0.0, 3.5), None, &mut sc).unwrap().unwrap();
        assert!((c.dist - 1.5).abs() < 1e-15 && (c.point.z - 2.0).abs() < 1e-15);
        let n = w.face_normal(FaceId(0), p(0.0, 0.0, 2.0)).unwrap();
        assert!((n - v(0.0, 0.0, 1.0)).norm() < 1e-15);
        assert!(matches!(w.face_normal(FaceId(99), p(0.0, 0.0, 0.0)), Err(RtError::UnknownId { .. })));
        assert!(matches!(w.closest(p(0.0, 0.0, 0.0), Some(RegionId(9)), &mut sc), Err(RtError::UnknownId { .. })));
    }

    #[test]
    fn reflective_response_conserves_path_length() {
        let w = world(vec![], ResponseKind::Reflective);
        let mut sc = Scratch::new();
        let mut st = w.init_state(p(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), None, &mut sc).unwrap();
        let mut travelled = 0.0;
        for _ in 0..5 {
            let r = w.fire(&mut st, f64::INFINITY, 10, &mut sc).unwrap();
            travelled += r.dist;
            let StepEnd::Boundary { face, action: FaceAction::Respond(k), .. } = r.end else { panic!() };
            assert_eq!(w.resolve_boundary(&mut st, face, k, [0.0, 0.0]).unwrap(), BoundaryAction::Continue);
        }
        assert!((travelled - 45.0).abs() < 1e-12, "{travelled}");
        assert_eq!(st.dir.x, -1.0);
    }

    #[test]
    fn white_albedo_and_vacuum_resolution_use_caller_uniforms() {
        let mut sc = Scratch::new();
        // White.
        let w = world(vec![], ResponseKind::White);
        let mut st = w.init_state(p(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), None, &mut sc).unwrap();
        let r = w.fire(&mut st, f64::INFINITY, 10, &mut sc).unwrap();
        let StepEnd::Boundary { face, action: FaceAction::Respond(k), .. } = r.end else { panic!() };
        w.resolve_boundary(&mut st, face, k, [0.37, 0.81]).unwrap();
        assert!((st.dir.norm() - 1.0).abs() < 1e-14 && st.dir.x < 0.0);
        assert!((st.dir.x * st.dir.x - (1.0 - 0.37)).abs() < 1e-12, "cos^2 = 1-u0");
        // Same uniforms, same direction: no hidden RNG.
        let mut st2 = w.init_state(p(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), None, &mut sc).unwrap();
        w.fire(&mut st2, f64::INFINITY, 10, &mut sc).unwrap();
        w.resolve_boundary(&mut st2, face, k, [0.37, 0.81]).unwrap();
        assert_eq!(st.dir, st2.dir);
        // Albedo 0.4: u0 = 0.3 survives (specular), u0 = 0.4 is absorbed.
        let w = world(vec![], ResponseKind::Albedo(0.4));
        let mut st = w.init_state(p(0.0, 0.0, 0.0), v(0.0, 1.0, 0.0), None, &mut sc).unwrap();
        let r = w.fire(&mut st, f64::INFINITY, 10, &mut sc).unwrap();
        let StepEnd::Boundary { face, action: FaceAction::Respond(k), .. } = r.end else { panic!() };
        assert_eq!(k, ResponseKind::Albedo(0.4));
        let mut survive = st.clone();
        assert_eq!(w.resolve_boundary(&mut survive, face, k, [0.3, 0.0]).unwrap(), BoundaryAction::Continue);
        assert_eq!(survive.dir.y, -1.0);
        let mut absorbed = st.clone();
        assert_eq!(w.resolve_boundary(&mut absorbed, face, k, [0.4, 0.0]).unwrap(), BoundaryAction::Terminate);
        assert_eq!(absorbed.dir, st.dir);
        assert!(matches!(w.resolve_boundary(&mut st, face, ResponseKind::Albedo(1.5), [0.0; 2]), Err(RtError::BadAlbedo { .. })));
        // Vacuum.
        let w = world(vec![], vac());
        let mut st = w.init_state(p(0.0, 0.0, 0.0), v(0.0, 1.0, 0.0), None, &mut sc).unwrap();
        let r = w.fire(&mut st, f64::INFINITY, 10, &mut sc).unwrap();
        let StepEnd::Boundary { face, action: FaceAction::Respond(k), .. } = r.end else { panic!() };
        assert_eq!(w.resolve_boundary(&mut st, face, k, [0.0; 2]).unwrap(), BoundaryAction::Terminate);
        // Invalid domain actions are build errors.
        assert!(AnalyticWorld::simple(vec![], domain(), ResponseKind::Albedo(2.0)).is_err());
        assert!(AnalyticWorld::new(vec![], domain(), [FaceAction::Transmit; 6]).is_err());
    }

    #[test]
    fn mirror_deck_is_applied_inside_step() {
        let w = AnalyticWorld::new(vec![], domain(), [FaceAction::Reflect; 6]).unwrap();
        let mut sc = Scratch::new();
        let mut st = w.init_state(p(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), None, &mut sc).unwrap();
        let s = w.step(&mut st, f64::INFINITY, &mut sc).unwrap();
        assert!((s.dist - 5.0).abs() < 1e-15);
        assert!(matches!(s.end, StepEnd::Boundary { action: FaceAction::Reflect, .. }));
        assert_eq!((st.pos.x, st.dir.x), (5.0, -1.0), "already reflected, still in the cell");
        assert!(st.cell == s.cell);
        // trace follows decks by default (unfolded path) and stops with `stop_at_deck`.
        let mut sc2 = Scratch::new();
        let mut st = w.init_state(p(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), None, &mut sc2).unwrap();
        let opts = TraceOptions { max_dist: 25.0, ..TraceOptions::default() };
        let mut n = 0;
        let end = w.trace(&mut st, &opts, &mut |_: &Segment| { n += 1; Flow::Continue }, &mut sc2).unwrap();
        // Chords 5, 10, 10 reach the limit exactly; the empty tail is not emitted.
        assert_eq!((n, end), (3, TraceEnd::Limit));
        let mut st = w.init_state(p(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), None, &mut sc2).unwrap();
        let opts = TraceOptions { stop_at_deck: true, ..TraceOptions::default() };
        let end = w.trace(&mut st, &opts, &mut |_: &Segment| Flow::Continue, &mut sc2).unwrap();
        assert!(matches!(end, TraceEnd::Boundary { action: FaceAction::Reflect, .. }));
    }

    /// Translation wrap on the x faces: leaving +x at x = 5 re-enters at x = -5 with the same
    /// direction; the y and z faces are vacuum.
    #[test]
    fn translation_wrap_moves_the_state_through_the_mate() {
        let n = 1; // one shape: it is surface 0, domain face k is surface 1 + k
        let deck = Deck::Translation { vector: v(10.0, 0.0, 0.0) };
        let a = wrap_transform(&deck, DeckSide::A).unwrap(); // -x face (A) -> +x face (B)
        let b = wrap_transform(&deck, DeckSide::B).unwrap(); // +x face (B) -> -x face (A)
        let mut acts = [FaceAction::Respond(vac()); 6];
        acts[0] = FaceAction::Wrap { transform: a, mate: FaceId::new(n + 1) };
        acts[1] = FaceAction::Wrap { transform: b, mate: FaceId::new(n) };
        let w = AnalyticWorld::new(vec![sphere(1.0)], domain(), acts).unwrap();
        assert!(w.caps().has(TrackerCaps::PERIODIC));
        let mut sc = Scratch::new();
        let mut st = w.init_state(p(2.0, 0.2, 0.0), v(1.0, 0.0, 0.0), None, &mut sc).unwrap();
        let s1 = w.fire(&mut st, f64::INFINITY, 50, &mut sc).unwrap();
        let StepEnd::Boundary { surface, action: FaceAction::Wrap { mate, .. }, .. } = s1.end else { panic!("{:?}", s1.end) };
        assert_eq!((surface, mate), (SurfaceId(2), FaceId(1)), "shape is surface 0, domain face k is surface 1 + k");
        assert!((s1.dist - 3.0).abs() < 1e-12);
        assert_eq!((st.pos.x, st.dir.x), (-5.0, 1.0));
        // Continue: sphere chord at y = 0.2 then the +x wall again, 5 + 5 further along... the
        // second leg crosses the sphere from -5 to 5.
        let (mut inside, mut total) = (0.0, 0.0);
        loop {
            let s = w.step(&mut st, f64::INFINITY, &mut sc).unwrap();
            total += s.dist;
            if s.cell == CellId(0) {
                inside += s.dist;
            }
            if matches!(s.end, StepEnd::Boundary { .. }) {
                break;
            }
        }
        assert!((total - 10.0).abs() < 1e-12);
        assert!((inside - 2.0 * (1.0f64 - 0.04).sqrt()).abs() < 1e-12);
    }

    /// Rotation by a quarter turn about z maps the y = 0 face (A) of a first-quadrant box onto the
    /// x = 0 face (B). Outward at A becomes inward at B (the deck sign rule), and `g^-1` returns.
    #[test]
    fn rotation_wrap_round_trips_with_g_and_g_inverse() {
        let deck = Deck::rotation(Axis { point: p(0.0, 0.0, 0.0), dir: v(0.0, 0.0, 1.0) }, Turns { num: 1, den: 4 });
        let (g, gi) = (wrap_transform(&deck, DeckSide::A).unwrap(), wrap_transform(&deck, DeckSide::B).unwrap());
        let dom = Aabb { lo: p(0.0, 0.0, -1.0), hi: p(4.0, 4.0, 1.0) };
        let mut acts = [FaceAction::Respond(vac()); 6];
        acts[2] = FaceAction::Wrap { transform: g, mate: FaceId::new(0) }; // -y face (k = 2) is A, mate -x face
        acts[0] = FaceAction::Wrap { transform: gi, mate: FaceId::new(2) }; // -x face (k = 0) is B
        let w = AnalyticWorld::new(vec![], dom, acts).unwrap();
        let mut sc = Scratch::new();
        // Leave A at (2, 0) moving -y: re-enter B at (0, 2) moving +x.
        let mut st = w.init_state(p(2.0, 1.0, 0.0), v(0.0, -1.0, 0.0), None, &mut sc).unwrap();
        let r = w.fire(&mut st, f64::INFINITY, 10, &mut sc).unwrap();
        assert!((r.dist - 1.0).abs() < 1e-15);
        assert!((st.pos - p(0.0, 2.0, 0.0)).norm() < 1e-12 && (st.dir - v(1.0, 0.0, 0.0)).norm() < 1e-12, "{:?} {:?}", st.pos, st.dir);
        let r = w.fire(&mut st, f64::INFINITY, 10, &mut sc).unwrap();
        assert!((r.dist - 4.0).abs() < 1e-12, "{}", r.dist);
        // The other way: leave B at (0, 3) moving -x: re-enter A at (3, 0) moving +y.
        let mut st = w.init_state(p(1.0, 3.0, 0.0), v(-1.0, 0.0, 0.0), None, &mut sc).unwrap();
        let r = w.fire(&mut st, f64::INFINITY, 10, &mut sc).unwrap();
        assert!((r.dist - 1.0).abs() < 1e-15);
        assert!((st.pos - p(3.0, 0.0, 0.0)).norm() < 1e-12 && (st.dir - v(0.0, 1.0, 0.0)).norm() < 1e-12, "{:?} {:?}", st.pos, st.dir);
        // g then g^-1 is the identity on a point and a direction.
        let q = p(2.0, 0.0, 0.5);
        assert!((gi.apply_point(g.apply_point(q)) - q).norm() < 1e-12);
    }

    fn lattice_world(actions: [FaceAction; 6]) -> AnalyticWorld {
        // 3 x 2 x 1 elements of pitch 4; each holds a sphere r = 1 at its centre and a small box.
        let shapes = vec![sphere(1.0), Shape::Box { lo: p(1.2, -0.3, -0.3), hi: p(1.8, 0.3, 0.3) }];
        AnalyticWorld::with_lattice(shapes, RectLattice { origin: p(0.0, 0.0, 0.0), pitch: [4.0, 4.0, 4.0], dims: [3, 2, 1] }, actions).unwrap()
    }

    #[test]
    fn lattice_slot_changes_and_exact_chords() {
        let w = lattice_world([FaceAction::Respond(vac()); 6]);
        assert!(w.caps().has(TrackerCaps::HIERARCHY));
        let info = w.info();
        assert_eq!((info.prototypes, info.copies, info.cells), (1, 6, 18));
        let mut sc = Scratch::new();
        // Along y = 2, z = 2 through the centres of elements 0, 1, 2 (x = 2, 6, 10).
        let mut st = w.init_state(p(0.0, 2.0, 2.0), v(1.0, 0.0, 0.0), None, &mut sc).unwrap();
        assert_eq!(st.levels.len(), 1);
        assert_eq!(st.levels[0].lattice, [0, 0, 0]);
        let mut slot_changes = Vec::new();
        let (mut sphere_len, mut total) = (0.0, 0.0);
        let mut cells = Vec::new();
        loop {
            let s = w.step(&mut st, f64::INFINITY, &mut sc).unwrap();
            total += s.dist;
            cells.push(s.cell.0);
            if s.cell.0 % 3 == 0 {
                sphere_len += s.dist;
            }
            match s.end {
                StepEnd::SlotChange { to_cell } => {
                    slot_changes.push((st.pos.x, to_cell.0, st.levels[0].lattice, st.levels[0].instance.0));
                    assert_eq!(to_cell, st.cell);
                    assert_eq!(s.region, RegionId(2), "walls are crossed in the background region");
                }
                StepEnd::Boundary { .. } => break,
                _ => {}
            }
        }
        assert!((total - 12.0).abs() < 1e-12);
        assert!((sphere_len - 6.0).abs() < 1e-12, "three diameters");
        assert_eq!(slot_changes, vec![(4.0, 5, [1, 0, 0], 1), (8.0, 8, [2, 0, 0], 2)]);
        // fire skips the slot changes: from the left wall it stops at the first sphere.
        let mut st = w.init_state(p(0.0, 2.0, 2.0), v(1.0, 0.0, 0.0), None, &mut sc).unwrap();
        let r = w.fire(&mut st, f64::INFINITY, 100, &mut sc).unwrap();
        assert!(matches!(r.end, StepEnd::RegionBoundary { .. }) && (r.dist - 1.0).abs() < 1e-12);
        // Locate in the third element carries its path; cell numbering is e * 3 + c.
        let loc = w.locate(p(10.0, 2.0, 2.0), None, None, &mut sc).unwrap();
        assert_eq!((loc.cell, loc.region), (CellId(2 * 3), RegionId(0)));
        assert_eq!(loc.levels[0].lattice, [2, 0, 0]);
        assert_eq!(w.cell_region(CellId(13)), Some(RegionId(1)));
        // A point on an interior wall resolves by direction, or by the hashed default.
        let a = w.locate(p(4.0, 2.0, 2.0), Some(v(1.0, 0.0, 0.0)), None, &mut sc).unwrap();
        let b = w.locate(p(4.0, 2.0, 2.0), Some(v(-1.0, 0.0, 0.0)), None, &mut sc).unwrap();
        assert_eq!((a.levels[0].lattice, b.levels[0].lattice), ([1, 0, 0], [0, 0, 0]));
        let n1 = w.locate(p(4.0, 2.0, 2.0), None, None, &mut sc).unwrap();
        assert_eq!(n1, w.locate(p(4.0, 2.0, 2.0), Some(hashed_direction(p(4.0, 2.0, 2.0))), None, &mut sc).unwrap());
    }

    #[test]
    fn lattice_diagonal_trace_sums_to_geometric_path() {
        let w = lattice_world([FaceAction::Respond(vac()); 6]);
        let s = 1.0 / 14f64.sqrt();
        let d = v(3.0 * s, 2.0 * s, 1.0 * s);
        let (segs, end) = trace_all(&w, p(0.1, 0.2, 0.3), d);
        assert!(vacuum_end(&end));
        let total: f64 = segs.iter().map(|x| x.length).sum();
        // Exit through the first wall reached: x = 12, y = 8, z = 4.
        let want = ((12.0 - 0.1) / d.x).min((8.0 - 0.2) / d.y).min((4.0 - 0.3) / d.z);
        assert!((total - want).abs() < 1e-11, "{total} {want}");
        // Segment t0s are continuous.
        for w2 in segs.windows(2) {
            assert!((w2[0].t0 + w2[0].length - w2[1].t0).abs() < 1e-12);
        }
    }

    #[test]
    fn lattice_wrap_enters_the_opposite_element() {
        let deck = Deck::Translation { vector: v(12.0, 0.0, 0.0) };
        let mut acts = [FaceAction::Respond(vac()); 6];
        acts[0] = FaceAction::Wrap { transform: wrap_transform(&deck, DeckSide::A).unwrap(), mate: FaceId::new(2 + 1) };
        acts[1] = FaceAction::Wrap { transform: wrap_transform(&deck, DeckSide::B).unwrap(), mate: FaceId::new(2) };
        let w = lattice_world(acts);
        let mut sc = Scratch::new();
        let mut st = w.init_state(p(0.5, 2.0, 2.0), v(-1.0, 0.0, 0.0), None, &mut sc).unwrap();
        let r = w.fire(&mut st, f64::INFINITY, 100, &mut sc).unwrap();
        assert!(matches!(r.end, StepEnd::Boundary { action: FaceAction::Wrap { .. }, .. }));
        assert_eq!(st.pos.x, 12.0);
        assert_eq!(st.levels[0].lattice, [2, 0, 0], "re-entered in the last column");
        assert_eq!(st.cell, CellId(2 * 3 + 2));
    }

    #[test]
    fn lattice_shapes_must_be_strictly_inside() {
        let big = vec![sphere(2.0)];
        let l = RectLattice { origin: p(0.0, 0.0, 0.0), pitch: [4.0; 3], dims: [2, 1, 1] };
        assert!(AnalyticWorld::with_lattice(big, l, [FaceAction::Respond(vac()); 6]).is_err());
    }

    #[test]
    fn bad_input_is_a_typed_error() {
        let w = world(vec![sphere(1.0)], vac());
        let mut sc = Scratch::new();
        assert!(matches!(w.init_state(p(0.0, 0.0, 0.0), v(0.0, 0.0, 0.0), None, &mut sc), Err(RtError::BadDirection { .. })));
        assert!(matches!(w.init_state(p(f64::NAN, 0.0, 0.0), v(1.0, 0.0, 0.0), None, &mut sc), Err(RtError::NonFinite(_))));
        assert!(AnalyticWorld::simple(vec![sphere(-1.0)], domain(), vac()).is_err());
        let mut st = w.init_state(p(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), None, &mut sc).unwrap();
        assert!(matches!(w.step(&mut st, f64::NAN, &mut sc), Err(RtError::NonFinite(_))));
        // Outside the model: a located state with NONE, and a step reports Lost as a value.
        let mut out = w.init_state(p(9.0, 0.0, 0.0), v(1.0, 0.0, 0.0), None, &mut sc).unwrap();
        assert!(out.cell.is_none());
        assert!(matches!(w.step(&mut out, 1.0, &mut sc).unwrap().end, StepEnd::Lost(_)));
    }

    fn fan(n: usize) -> Vec<Ray> {
        // Deterministic fan of rays across the domain, no RNG needed.
        (0..n)
            .map(|i| {
                let a = 0.1 + 2.9 * (i as f64) / (n as f64);
                Ray { origin: p(-4.9, -3.0 + 6.0 * (i as f64) / (n as f64), 0.2), dir: v(a.cos(), a.sin() * 0.3, 0.1) }
            })
            .collect()
    }

    #[test]
    fn bundle_hash_is_equal_for_one_and_four_threads_and_tally_sums_to_path() {
        let w = world(vec![sphere(1.0), sphere(2.0), Shape::Box { lo: p(2.5, -1.0, -1.0), hi: p(3.5, 1.0, 1.0) }], vac()).with_block_size(64);
        let rays = fan(400);
        let opts = TraceOptions::default();
        let serial = w.trace_bundle_block(&rays, &opts, &mut Scratch::new()).unwrap();
        let run = |threads| {
            let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
            pool.install(|| w.trace_bundle(&rays, &opts).unwrap())
        };
        let (one, four) = (run(1), run(4));
        assert_eq!(serial.content_hash(), one.content_hash());
        assert_eq!(one.content_hash(), four.content_hash());
        assert_eq!(serial, four);
        assert_eq!(serial.ray_count(), 400);
        assert!(serial.end.iter().all(|e| *e == EndCode::Vacuum));
        // Tally by cell: per-ray scored length equals the bundle's lengths.
        let mut sc = Scratch::new();
        let mut tally = TrackLengthTally::new(w.bin_map(&TallySpec::Cells).unwrap());
        for r in &rays {
            let n = r.dir.norm();
            let mut st = w.init_state(r.origin, r.dir * (1.0 / n), None, &mut sc).unwrap();
            w.trace(&mut st, &TraceOptions::default(), &mut tally, &mut sc).unwrap();
        }
        let sum: f64 = tally.acc.iter().sum();
        let bundle_sum: f64 = serial.length.iter().sum();
        assert!((sum - bundle_sum).abs() < 1e-9 * bundle_sum);
        assert_eq!(w.bin_map(&TallySpec::Regions).unwrap().n_bins, 4);
    }

    #[test]
    fn dagmc_compat_against_the_analytic_sphere() {
        let w = world(vec![sphere(2.0)], vac());
        let mut sc = Scratch::new();
        let sph = RegionId(0);
        // Exiting from the centre: surface 0 at distance 2.
        let RayFire::Hit(h) = compat::ray_fire(&w, sph, p(0.0, 0.0, 0.0), v(0.0, 1.0, 0.0), RayOrientation::Exiting, None, &mut sc).unwrap() else { panic!() };
        assert_eq!((h.surface, h.to_region), (SurfaceId(0), RegionId(1)));
        assert!((h.dist - 2.0).abs() < 1e-15);
        // Entering from outside.
        let RayFire::Hit(h) = compat::ray_fire(&w, sph, p(-5.0, 0.0, 0.0), v(1.0, 0.0, 0.0), RayOrientation::Entering, None, &mut sc).unwrap() else { panic!() };
        assert!((h.dist - 3.0).abs() < 1e-12 && h.to_region == sph);
        // An exiting ray from outside the volume misses; a limit shorter than the hit misses.
        assert_eq!(compat::ray_fire(&w, sph, p(-5.0, 0.0, 0.0), v(1.0, 0.0, 0.0), RayOrientation::Exiting, None, &mut sc).unwrap(), RayFire::Miss);
        assert_eq!(compat::ray_fire(&w, sph, p(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), RayOrientation::Exiting, Some(1.0), &mut sc).unwrap(), RayFire::Miss);
        // Any: from outside, the first sphere surface; the background ray out the domain wall is a miss
        // for the sphere volume.
        let RayFire::Hit(h) = compat::ray_fire(&w, sph, p(-5.0, 0.0, 0.0), v(1.0, 0.0, 0.0), RayOrientation::Any, None, &mut sc).unwrap() else { panic!() };
        assert!((h.dist - 3.0).abs() < 1e-12);
        assert_eq!(compat::ray_fire(&w, sph, p(-5.0, 4.0, 0.0), v(1.0, 0.0, 0.0), RayOrientation::Any, None, &mut sc).unwrap(), RayFire::Miss);
        // Exiting the background through the domain wall is a hit with no region beyond.
        let RayFire::Hit(h) = compat::ray_fire(&w, RegionId(1), p(-5.0, 4.0, 0.0), v(1.0, 0.0, 0.0), RayOrientation::Exiting, None, &mut sc).unwrap() else { panic!() };
        assert_eq!((h.surface, h.to_region), (SurfaceId(1 + 1), RegionId::NONE));
        // point_in_volume, next_vol, surface_sense, get_angle, closest_to_location.
        assert!(compat::point_in_volume(&w, sph, p(1.0, 0.0, 0.0), None, &mut sc).unwrap());
        assert!(!compat::point_in_volume(&w, sph, p(3.0, 0.0, 0.0), None, &mut sc).unwrap());
        assert_eq!(compat::next_vol(&w, SurfaceId(0), sph), Some(RegionId(1)));
        assert_eq!(compat::next_vol(&w, SurfaceId(0), RegionId(1)), Some(sph));
        assert_eq!((compat::surface_sense(&w, SurfaceId(0), sph), compat::surface_sense(&w, SurfaceId(0), RegionId(1))), (Some(1), Some(-1)));
        let n = compat::get_angle(&w, FaceId(0), p(0.0, -2.0, 0.0)).unwrap();
        assert!((n - v(0.0, -1.0, 0.0)).norm() < 1e-15, "outward from the volume, as sense +1 means");
        let c = compat::closest_to_location(&w, sph, p(0.0, 0.0, 0.5), &mut sc).unwrap().unwrap();
        assert!((c.dist - 1.5).abs() < 1e-15);
    }

    #[test]
    fn structure_hash_is_stable_and_sensitive() {
        let a = world(vec![sphere(1.0)], vac());
        let b = world(vec![sphere(1.0)], vac());
        let c = world(vec![sphere(1.0 + 1e-9)], vac());
        let d = world(vec![sphere(1.0)], ResponseKind::Reflective);
        assert_eq!(a.structure_hash(), b.structure_hash());
        assert_ne!(a.structure_hash(), c.structure_hash());
        assert_ne!(a.structure_hash(), d.structure_hash());
        assert_eq!(a.info().cells, 2);
        assert_eq!(a.bounds(), domain());
    }

    fn builder() -> StubBuilder {
        StubBuilder { shapes: vec![sphere(1.0), sphere(2.0)], domain: domain(), actions: [FaceAction::Respond(vac()); 6] }
    }
    fn flat_world_spec() -> WorldSpec {
        WorldSpec { components: vec![ComponentKind::Leaf { input: 0 }], root: ComponentId(0), policy: InstancePolicy::Hierarchical }
    }
    fn lattice_spec(dims: [u32; 3]) -> WorldSpec {
        let n = (dims[0] * dims[1] * dims[2]) as usize;
        WorldSpec {
            components: vec![
                ComponentKind::Leaf { input: 0 },
                ComponentKind::Lattice(Lattice { kind: LatticeKind::Rect { pitch: [6.0; 3] }, origin: p(0.0, 0.0, 0.0), dims, fill: vec![ComponentId(0); n], outer: None }),
            ],
            root: ComponentId(1),
            policy: InstancePolicy::Hierarchical,
        }
    }

    #[test]
    fn build_then_load_gives_the_same_structure_hash() {
        let opts = BuildOptions::default();
        for spec in [flat_world_spec(), lattice_spec([2, 2, 1])] {
            let input = BuildInput::World(&spec);
            let b = builder();
            let (built, report) = b.build(&input, &opts).unwrap();
            assert!(!report.from_cache && report.issues.is_empty());
            let image = built.export_cache().unwrap();
            // Through the byte format and back.
            let bytes = image.encode();
            let image = CacheImage::decode(&bytes).unwrap();
            let (loaded, report) = b.load(&input, &opts, &image).unwrap();
            assert!(report.from_cache);
            assert_eq!(built.structure_hash(), loaded.structure_hash());
            assert_eq!(built.info().key, loaded.info().key);
            assert_eq!(loaded.export_cache().unwrap().encode(), bytes);
            // And the two trackers answer alike.
            let rays: Vec<Ray> = (0..50)
                .map(|i| {
                    let a = 0.1 + 1.2 * f64::from(i) / 50.0;
                    Ray { origin: p(0.2, 0.2 + 0.01 * f64::from(i), 0.3), dir: v(a.cos(), a.sin() * 0.3, 0.1) }
                })
                .collect();
            let o = TraceOptions::default();
            assert_eq!(built.trace_bundle(&rays, &o).unwrap().content_hash(), loaded.trace_bundle(&rays, &o).unwrap().content_hash());
            // Building again is bit-identical.
            let (again, _) = b.build(&input, &opts).unwrap();
            assert_eq!(again.structure_hash(), built.structure_hash());
        }
    }

    #[test]
    fn a_stale_cache_names_what_changed() {
        let spec = flat_world_spec();
        let input = BuildInput::World(&spec);
        let opts = BuildOptions::default();
        let b = builder();
        let (built, _) = b.build(&input, &opts).unwrap();
        let image = built.export_cache().unwrap();
        // A boundary edit invalidates only the boundary component of the key.
        let mut b2 = builder();
        b2.actions[3] = FaceAction::Respond(ResponseKind::Reflective);
        assert!(matches!(b2.load(&input, &opts, &image), Err(RtError::CacheStale("boundary conditions changed"))));
        let mut b3 = builder();
        b3.shapes[0] = sphere(1.1);
        assert!(matches!(b3.load(&input, &opts, &image), Err(RtError::CacheStale("geometry changed"))));
        let mut o2 = opts;
        o2.block_size = 7;
        assert!(matches!(b.load(&input, &o2, &image), Err(RtError::CacheStale("options changed"))));
        let other = lattice_spec([1, 1, 1]);
        assert!(matches!(b.load(&BuildInput::World(&other), &opts, &image), Err(RtError::CacheStale("hierarchy changed"))));
        // Unsupported inputs are typed errors.
        let f = crate::FacetInput::default();
        let bs = crate::BoundarySpec::default();
        assert!(matches!(b.build(&BuildInput::Facets(&f, &bs), &opts), Err(RtError::Unsupported(_))));
    }
}
