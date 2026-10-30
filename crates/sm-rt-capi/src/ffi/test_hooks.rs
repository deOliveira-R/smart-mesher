//! Test-only exports (feature `stubs`): build trackers over the contract's stub `AnalyticWorld`
//! so the C tests have something to load without a mesh builder. Not part of `smrt.h`; the
//! declarations are in `tests/c/smrt_test.h`.

use super::{SmrtTracker, exclusive, guard, tracker_into_raw};
use crate::status::*;
use sm_core::{Aabb, Point3, Vector3};
use sm_rt_api::stub::{AnalyticWorld, RectLattice, Shape};
use sm_rt_api::{
    CellId, Closest, FaceAction, FaceId, Located, RegionId, RegionTable, ResponseKind, Result as RtResult, Scratch, Step,
    SurfaceTable, TrackState, Tracker, TrackerCaps, TrackerInfo,
};
use std::sync::Arc;

/// Half-width of the cubic test domain.
pub const DOMAIN_HALF: f64 = 10.0;

/// Kind: one sphere of radius `radius` at the origin, vacuum domain.
pub const STUB_SPHERE: i32 = 1;
/// Kind: five concentric spheres, radii `radius * k / 5` (k = 1..=5), vacuum domain.
pub const STUB_SHELLS: i32 = 2;
/// Kind: empty domain, mirror (deck) boundary on all six faces.
pub const STUB_MIRROR_BOX: i32 = 3;
/// Kind: empty domain, white response on all faces.
pub const STUB_WHITE_BOX: i32 = 4;
/// Kind: empty domain, reflective response on all faces.
pub const STUB_REFLECTIVE_BOX: i32 = 5;
/// Kind: empty domain, albedo 0.5 response on all faces.
pub const STUB_ALBEDO_BOX: i32 = 6;
/// Kind: 3x3x3 lattice with pitch 4 (domain `[0,12]^3`), one sphere of radius `radius` per element.
pub const STUB_LATTICE: i32 = 7;
/// Kind: the `STUB_SPHERE` world, but its region table has no implicit complement flag.
pub const STUB_NO_COMPLEMENT: i32 = 98;
/// Kind: the `STUB_SPHERE` world, but `step` panics.
pub const STUB_PANIC_ON_STEP: i32 = 99;

fn domain() -> Aabb {
    let h = DOMAIN_HALF;
    Aabb { lo: Point3::new(-h, -h, -h), hi: Point3::new(h, h, h) }
}

fn sphere(r: f64) -> Shape {
    Shape::Sphere { center: Point3::new(0.0, 0.0, 0.0), radius: r }
}

/// The stub world for a test kind; `None` for an unknown kind or an invalid world.
pub fn world(kind: i32, radius: f64) -> Option<AnalyticWorld> {
    let all = |a: FaceAction| [a; 6];
    let vac = ResponseKind::Vacuum;
    match kind {
        STUB_SPHERE | STUB_PANIC_ON_STEP | STUB_NO_COMPLEMENT => AnalyticWorld::simple(vec![sphere(radius)], domain(), vac).ok(),
        STUB_SHELLS => AnalyticWorld::simple((1..=5).map(|k| sphere(radius * f64::from(k) / 5.0)).collect(), domain(), vac).ok(),
        STUB_MIRROR_BOX => AnalyticWorld::new(vec![], domain(), all(FaceAction::Reflect)).ok(),
        STUB_WHITE_BOX => AnalyticWorld::simple(vec![], domain(), ResponseKind::White).ok(),
        STUB_REFLECTIVE_BOX => AnalyticWorld::simple(vec![], domain(), ResponseKind::Reflective).ok(),
        STUB_ALBEDO_BOX => AnalyticWorld::simple(vec![], domain(), ResponseKind::Albedo(0.5)).ok(),
        STUB_LATTICE => {
            let l = RectLattice::new(Point3::new(0.0, 0.0, 0.0), [4.0; 3], [3, 3, 3]);
            AnalyticWorld::with_lattice(vec![sphere(radius)], l, all(FaceAction::Respond(vac))).ok()
        }
        _ => None,
    }
}

/// A tracker whose `step` panics (everything else forwards to the wrapped world). Used to test
/// that no panic crosses the C boundary.
pub struct PanicOnStep(pub AnalyticWorld);

impl Tracker for PanicOnStep {
    fn caps(&self) -> TrackerCaps {
        self.0.caps()
    }
    fn info(&self) -> &TrackerInfo {
        self.0.info()
    }
    fn regions(&self) -> &RegionTable {
        self.0.regions()
    }
    fn surfaces(&self) -> &SurfaceTable {
        self.0.surfaces()
    }
    fn cell_region(&self, c: CellId) -> Option<RegionId> {
        self.0.cell_region(c)
    }
    fn cell_volume(&self, c: CellId) -> Option<f64> {
        self.0.cell_volume(c)
    }
    fn locate(&self, p: Point3, dir: Option<Vector3>, hint: Option<CellId>, scratch: &mut Scratch) -> RtResult<Located> {
        self.0.locate(p, dir, hint, scratch)
    }
    fn step(&self, _st: &mut TrackState, _max_dist: f64, _scratch: &mut Scratch) -> RtResult<Step> {
        panic!("deliberate panic in Tracker::step")
    }
    fn closest(&self, p: Point3, region: Option<RegionId>, scratch: &mut Scratch) -> RtResult<Option<Closest>> {
        self.0.closest(p, region, scratch)
    }
    fn face_normal(&self, f: FaceId, p: Point3) -> RtResult<Vector3> {
        self.0.face_normal(f, p)
    }
}

/// A tracker that forwards to a world but reports no implicit complement region.
pub struct NoComplement(AnalyticWorld, RegionTable);

impl NoComplement {
    fn new(w: AnalyticWorld) -> Self {
        let mut t = w.regions().clone();
        for r in &mut t.0 {
            r.implicit_complement = false;
        }
        Self(w, t)
    }
}

impl Tracker for NoComplement {
    fn caps(&self) -> TrackerCaps {
        self.0.caps()
    }
    fn info(&self) -> &TrackerInfo {
        self.0.info()
    }
    fn regions(&self) -> &RegionTable {
        &self.1
    }
    fn surfaces(&self) -> &SurfaceTable {
        self.0.surfaces()
    }
    fn cell_region(&self, c: CellId) -> Option<RegionId> {
        self.0.cell_region(c)
    }
    fn cell_volume(&self, c: CellId) -> Option<f64> {
        self.0.cell_volume(c)
    }
    fn locate(&self, p: Point3, dir: Option<Vector3>, hint: Option<CellId>, scratch: &mut Scratch) -> RtResult<Located> {
        self.0.locate(p, dir, hint, scratch)
    }
    fn step(&self, st: &mut TrackState, max_dist: f64, scratch: &mut Scratch) -> RtResult<Step> {
        self.0.step(st, max_dist, scratch)
    }
    fn closest(&self, p: Point3, region: Option<RegionId>, scratch: &mut Scratch) -> RtResult<Option<Closest>> {
        self.0.closest(p, region, scratch)
    }
    fn face_normal(&self, f: FaceId, p: Point3) -> RtResult<Vector3> {
        self.0.face_normal(f, p)
    }
}

/// Safe constructor used by Rust tests: the tracker handle for a kind.
pub fn tracker_for(kind: i32, radius: f64) -> Option<Arc<dyn Tracker>> {
    let w = world(kind, radius)?;
    Some(match kind {
        STUB_PANIC_ON_STEP => Arc::new(PanicOnStep(w)),
        STUB_NO_COMPLEMENT => Arc::new(NoComplement::new(w)),
        _ => Arc::new(w),
    })
}

/// Create a stub tracker handle for the C tests: `kind` is one of the `STUB_*` constants (values
/// in `smrt_test.h`), `radius` the sphere radius where the kind has one. The domain is
/// `[-10, 10]^3` (`[0, 12]^3` for the lattice). Free it with `smrt_tracker_free`. Returns `0`.
///
/// # Safety
/// `out` is null or valid for one write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_test_stub(kind: i32, radius: f64, out: *mut *mut SmrtTracker) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let out = unsafe { exclusive(out, "out")? };
        let t = tracker_for(kind, radius).ok_or_else(|| Fail::bad(format!("unknown stub kind {kind} or invalid radius {radius}")))?;
        *out = tracker_into_raw(t);
        Ok(SMRT_OK)
    })
}

