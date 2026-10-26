//! A brute-force analytic world: the one stub implementor of [`Tracker`].
//!
//! Spheres and axis-aligned boxes nested in a world box. Later shapes have priority over earlier
//! ones where they overlap (the way nested CSG cells work). Every query tests every shape, so it
//! is O(shapes); the point is to pin the *semantics* (exact chords, side-consistent crossings,
//! in-structure deck handling, direction tie-breaks) that a real hierarchical implementor must
//! reproduce, not to be fast.
//!
//! Ids: world faces are `TopoFace` serials 1..=6 in the order -x,+x,-y,+y,-z,+z. Shape `i`
//! (0-based) is volume serial `i + 1`; its faces are serial `100 * (i + 1) + k` (a sphere has
//! only `k = 0`; a box has `k = 0..6` in the same order as the world faces).

use crate::boundary::{FaceAction, ResponseKind};
use crate::cache::RtCacheKey;
use crate::error::{LostReason, LostReport, Result, RtError};
use crate::tracker::{RtOptions, Tracker, TrackerInfo};
use crate::types::{BoundaryHit, DeckAction, Event, FlatIx, Location, Nearest, Scratch, Step, TrackState};
use crate::BoundarySpec;
use sm_core::{Aabb, EntityId, EntityKind, Point3, Vector3};
use sm_geom_api::{Crossing, Frame, ImplicitSurface, Interval, Quadric, Ray};
use sm_model_api::InstancePath;
use sm_topo_api::Region;

/// A primitive of the stub world.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum StubShape {
    /// Sphere.
    Sphere {
        /// Center.
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

/// Brute-force world, see the module docs.
#[derive(Debug)]
pub struct AnalyticWorld {
    shapes: Vec<StubShape>,
    world: Aabb,
    bounds: BoundarySpec,
    info: TrackerInfo,
    path: InstancePath,
}

fn face_id(serial: u64) -> EntityId {
    EntityId::new(EntityKind::TopoFace, serial).expect("stub serials are small and non-zero")
}

fn vol_id(serial: u64) -> EntityId {
    EntityId::new(EntityKind::TopoVolume, serial).expect("stub serials are small and non-zero")
}

fn pt(p: Point3, a: usize) -> f64 {
    [p.x, p.y, p.z][a]
}

fn vc(v: Vector3, a: usize) -> f64 {
    [v.x, v.y, v.z][a]
}

fn axis_normal(k: usize) -> Vector3 {
    let mut n = [0.0; 3];
    n[k / 2] = if k.is_multiple_of(2) { -1.0 } else { 1.0 };
    Vector3::new(n[0], n[1], n[2])
}

#[derive(Clone, Copy, Debug)]
struct Cand {
    t: f64,
    face: EntityId,
    /// `Some(k)` for a world face.
    world: Option<usize>,
    outward: Vector3,
}

/// Slab entry/exit of a box: `(t, face index, leaving)`.
fn slab(lo: Point3, hi: Point3, o: Point3, d: Vector3, tol: f64, limit: f64, out: &mut Vec<(f64, usize, bool)>) {
    let (mut tin, mut kin, mut tout, mut kout) = (f64::NEG_INFINITY, None, f64::INFINITY, None);
    for a in 0..3 {
        let (oa, da, la, ha) = (pt(o, a), vc(d, a), pt(lo, a), pt(hi, a));
        if da.abs() < 1e-300 {
            if oa < la - tol || oa > ha + tol {
                return;
            }
            continue;
        }
        let (t1, t2) = ((la - oa) / da, (ha - oa) / da);
        let (tn, fnn, tf, ff) = if da > 0.0 { (t1, 2 * a, t2, 2 * a + 1) } else { (t2, 2 * a + 1, t1, 2 * a) };
        if tn > tin {
            tin = tn;
            kin = Some(fnn);
        }
        if tf < tout {
            tout = tf;
            kout = Some(ff);
        }
    }
    if tin > tout + tol {
        return;
    }
    if let Some(k) = kin.filter(|_| tin >= -tol && tin <= limit) {
        out.push((tin.max(0.0), k, false));
    }
    if let Some(k) = kout.filter(|_| tout >= -tol && tout <= limit) {
        out.push((tout.max(0.0), k, true));
    }
}

impl AnalyticWorld {
    /// Build a world. `bounds` gives the action of each world face (unlisted faces use
    /// `bounds.exterior`, vacuum by default).
    pub fn new(world: Aabb, shapes: Vec<StubShape>, bounds: BoundarySpec, options: RtOptions) -> Self {
        let exact = 6 + shapes.iter().map(|s| if matches!(s, StubShape::Sphere { .. }) { 1 } else { 6 }).sum::<u32>();
        let info = TrackerInfo {
            bounds: world,
            copies: 1,
            prototypes: 1,
            exact_faces: exact,
            faceted_faces: Vec::new(),
            memory_bytes: (shapes.len() * core::mem::size_of::<StubShape>()) as u64,
            key: RtCacheKey::new(0, 0, &bounds, &options, 0),
            options,
        };
        Self { shapes, world, bounds, info, path: InstancePath::new() }
    }

    /// Face id of world face `k` (0..6).
    pub fn world_face(k: usize) -> EntityId {
        face_id(k as u64 + 1)
    }

    /// Face id `k` of shape `i`.
    pub fn shape_face(i: usize, k: usize) -> EntityId {
        face_id(100 * (i as u64 + 1) + k as u64)
    }

    /// Volume id of shape `i`.
    pub fn volume(i: usize) -> EntityId {
        vol_id(i as u64 + 1)
    }

    fn tol(&self) -> f64 {
        self.info.options.tol.linear
    }

    fn contains(&self, s: &StubShape, p: Point3, dir: Option<Vector3>) -> bool {
        let tol = self.tol();
        match *s {
            StubShape::Sphere { center, radius } => {
                let r = p - center;
                let d = r.norm() - radius;
                if d < -tol {
                    true
                } else if d > tol {
                    false
                } else {
                    dir.is_none_or(|v| v.dot(r) <= 0.0)
                }
            }
            StubShape::Box { lo, hi } => (0..3).all(|a| {
                let (g_lo, g_hi) = (pt(p, a) - pt(lo, a), pt(hi, a) - pt(p, a));
                if g_lo < -tol || g_hi < -tol {
                    return false;
                }
                let v = dir.map(|v| vc(v, a));
                !(g_lo <= tol && v.is_some_and(|v| v < 0.0)) && !(g_hi <= tol && v.is_some_and(|v| v > 0.0))
            }),
        }
    }

    fn region_at(&self, p: Point3, dir: Option<Vector3>) -> Region {
        (0..self.shapes.len()).rev().find(|i| self.contains(&self.shapes[*i], p, dir)).map_or(Region::Exterior, |i| Region::Volume(Self::volume(i)))
    }

    fn loc(&self, p: Point3, dir: Option<Vector3>) -> Location {
        Location { copy: FlatIx(0), region: self.region_at(p, dir) }
    }

    /// Nearest side-consistent crossing from `(p, d)` within `limit`, or `None`.
    fn nearest_hit(&self, p: Point3, d: Vector3, limit: f64, sc: &mut Scratch) -> Result<Option<Cand>> {
        let tol = self.tol();
        let ray = Ray { origin: p, dir: d };
        let mut best: Option<Cand> = None;
        let mut offer = |c: Cand| {
            let better = match best {
                None => true,
                Some(b) => c.t < b.t - tol || (c.t <= b.t + tol && c.face.to_raw() < b.face.to_raw()),
            };
            if better {
                best = Some(c);
            }
        };
        let mut slabs = Vec::with_capacity(2);
        for (i, s) in self.shapes.iter().enumerate() {
            let inside = self.contains(s, p, Some(d));
            match *s {
                StubShape::Sphere { center, radius } => {
                    sc.hits.clear();
                    Quadric::sphere(&Frame::at(center), radius).ray_hits(&ray, Interval::new(-tol, limit), &mut sc.hits)?;
                    for h in &sc.hits {
                        let leaving = match h.crossing {
                            Crossing::AlongNormal => true,
                            Crossing::AgainstNormal => false,
                            _ => continue,
                        };
                        if leaving == inside {
                            offer(Cand { t: h.t.max(0.0), face: Self::shape_face(i, 0), world: None, outward: h.normal });
                        }
                    }
                }
                StubShape::Box { lo, hi } => {
                    slabs.clear();
                    slab(lo, hi, p, d, tol, limit, &mut slabs);
                    for &(t, k, leaving) in &slabs {
                        if leaving == inside {
                            offer(Cand { t, face: Self::shape_face(i, k), world: None, outward: axis_normal(k) });
                        }
                    }
                }
            }
        }
        slabs.clear();
        slab(self.world.lo, self.world.hi, p, d, tol, limit, &mut slabs);
        for &(t, k, leaving) in &slabs {
            if leaving {
                offer(Cand { t, face: Self::world_face(k), world: Some(k), outward: axis_normal(k) });
            }
        }
        Ok(best)
    }

    /// March over hidden crossings (faces of a shape buried inside a higher-priority one) to the
    /// next visible one. Returns `(distance, candidate, location after)`.
    fn find(&self, pos: Point3, dir: Vector3, at: Location, limit: f64, sc: &mut Scratch) -> Result<Option<(f64, Cand, Location)>> {
        let (mut p, mut cur, mut acc) = (pos, at, 0.0);
        for _ in 0..4 * self.shapes.len() + 8 {
            let Some(c) = self.nearest_hit(p, dir, limit - acc, sc)? else { return Ok(None) };
            acc += c.t;
            p = p + dir * c.t;
            if c.world.is_some() {
                return Ok(Some((acc, c, cur)));
            }
            let after = self.loc(p, Some(dir));
            if after.region != cur.region {
                return Ok(Some((acc, c, after)));
            }
            cur = after;
        }
        Err(RtError::Lost(Box::new(LostReport::new(LostReason::NoExitFound, p, dir))))
    }

    fn lost(&self, reason: LostReason, pos: Point3, dir: Vector3) -> RtError {
        RtError::Lost(Box::new(LostReport::new(reason, pos, dir)))
    }
}

impl Tracker for AnalyticWorld {
    fn info(&self) -> &TrackerInfo {
        &self.info
    }

    fn locate(&self, p: Point3, dir: Option<Vector3>, _sc: &mut Scratch) -> Result<TrackState> {
        let d0 = dir.unwrap_or(Vector3::new(0.0, 0.0, 1.0));
        if ![p.x, p.y, p.z, d0.x, d0.y, d0.z].iter().all(|v| v.is_finite()) {
            return Err(self.lost(LostReason::NonFinite, p, d0));
        }
        let n = d0.norm();
        let unit = if n > 0.0 { Vector3::new(d0.x / n, d0.y / n, d0.z / n) } else { Vector3::new(0.0, 0.0, 1.0) };
        let tol = self.tol();
        if !(0..3).all(|a| pt(p, a) >= pt(self.world.lo, a) - tol && pt(p, a) <= pt(self.world.hi, a) + tol) {
            return Err(self.lost(LostReason::NoRegionAtPoint, p, unit));
        }
        Ok(TrackState::new(p, unit, self.loc(p, dir.map(|_| unit))))
    }

    fn distance_to_boundary(&self, st: &TrackState, limit: f64, sc: &mut Scratch) -> Result<Option<BoundaryHit>> {
        Ok(self.find(st.pos, st.dir, st.at, limit, sc)?.map(|(t, c, after)| BoundaryHit::new(t, c.face, c.outward, c.world.is_none().then_some(after))))
    }

    fn step(&self, st: &mut TrackState, limit: f64, sc: &mut Scratch) -> Result<Step> {
        let Some((t, c, after)) = self.find(st.pos, st.dir, st.at, limit, sc)? else {
            st.pos = st.pos + st.dir * limit;
            st.at = self.loc(st.pos, Some(st.dir));
            return Ok(Step { distance: limit, event: Event::LimitReached });
        };
        let from = st.at;
        st.pos = st.pos + st.dir * t;
        let Some(k) = c.world else {
            st.at = after;
            st.last = Some(crate::FaceSide { face: c.face, copy: FlatIx(0), positive: true });
            return Ok(Step { distance: t, event: Event::Crossing { face: c.face, from, to: after } });
        };
        let action = self.bounds.action_of(c.face).unwrap_or(self.bounds.exterior);
        let n = axis_normal(k);
        let event = match action {
            FaceAction::Reflect => {
                let f = 2.0 * st.dir.dot(n);
                st.dir = Vector3::new(st.dir.x - f * n.x, st.dir.y - f * n.y, st.dir.z - f * n.z);
                st.deck_events += 1;
                Event::Deck { face: c.face, action: DeckAction::Reflected }
            }
            FaceAction::Wrap { transform, mate } => {
                st.pos = transform.apply_point(st.pos);
                let v = transform.apply_vector(st.dir);
                let m = v.norm();
                st.dir = Vector3::new(v.x / m, v.y / m, v.z / m);
                st.at = self.loc(st.pos, Some(st.dir));
                st.deck_events += 1;
                Event::Deck { face: c.face, action: DeckAction::Wrapped { mate } }
            }
            FaceAction::Respond(kind) => Event::Response { face: c.face, kind, inward: Vector3::new(-n.x, -n.y, -n.z) },
            _ => Event::Response { face: c.face, kind: ResponseKind::Vacuum, inward: Vector3::new(-n.x, -n.y, -n.z) },
        };
        st.last = Some(crate::FaceSide { face: c.face, copy: FlatIx(0), positive: true });
        Ok(Step { distance: t, event })
    }

    fn nearest_surface(&self, st: &TrackState, _sc: &mut Scratch) -> Result<Nearest> {
        let p = st.pos;
        let mut best = Nearest::new(f64::INFINITY, Self::world_face(0));
        let mut offer = |d: f64, f: EntityId| {
            if d < best.distance || (d == best.distance && f.to_raw() < best.face.to_raw()) {
                best = Nearest::new(d, f);
            }
        };
        for k in 0..6usize {
            let a = k / 2;
            let g = if k.is_multiple_of(2) { pt(p, a) - pt(self.world.lo, a) } else { pt(self.world.hi, a) - pt(p, a) };
            offer(g.abs(), Self::world_face(k));
        }
        for (i, s) in self.shapes.iter().enumerate() {
            match *s {
                StubShape::Sphere { center, radius } => offer(((p - center).norm() - radius).abs(), Self::shape_face(i, 0)),
                StubShape::Box { lo, hi } => {
                    for k in 0..6usize {
                        let a = k / 2;
                        let g = if k.is_multiple_of(2) { pt(p, a) - pt(lo, a) } else { pt(hi, a) - pt(p, a) };
                        offer(g.abs(), Self::shape_face(i, k));
                    }
                }
            }
        }
        Ok(best)
    }

    fn normal_at(&self, face: EntityId, _copy: FlatIx, p: Point3) -> Result<Vector3> {
        let s = face.serial();
        if s <= 6 {
            return Ok(axis_normal(s as usize - 1));
        }
        let (i, k) = ((s / 100 - 1) as usize, (s % 100) as usize);
        match self.shapes.get(i) {
            Some(StubShape::Sphere { center, radius }) => {
                let r = p - *center;
                Ok(Vector3::new(r.x / radius, r.y / radius, r.z / radius))
            }
            Some(StubShape::Box { .. }) if k < 6 => Ok(axis_normal(k)),
            _ => Err(RtError::Input(format!("unknown face {face:?}"))),
        }
    }

    fn path_of(&self, copy: FlatIx) -> Option<&InstancePath> {
        (copy.0 == 0).then_some(&self.path)
    }
}
