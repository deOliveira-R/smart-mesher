//! Local fixtures of the analytic tracker tests: a CSG leaf over a `MemoryStore`, with a
//! complement region inside the bounds, and chord measurements.
#![allow(dead_code)]

use sm_core::{Aabb, EntityId, Point3, Vector3};
use sm_geom_api::{Cone, CsgCell, Cylinder, Frame, GeomAppend, HalfSpace, MemoryStore, Plane, Quadric, Sign, Sphere, SurfaceDef, Torus};
pub use sm_geom_api::ImplicitDef;
use sm_rt::{AnalyticTracker, AnalyticTrackerBuilder};
use sm_rt_api::{
    BoundarySpec, BuildInput, BuildIssue, BuildOptions, CellId, ComplementPolicy, ComponentId, ComponentKind, CsgCellDef, CsgLeafInput, CsgRegion, Flow, HierInput, InstancePolicy, LeafInput, Result,
    Scratch, Segment, TraceEnd, TraceOptions, Tracker, WorldSpec,
};
pub use sm_testkit::surf3::SplitMix64;

pub fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}
pub fn v3(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}
pub fn unit(v: Vector3) -> Vector3 {
    v * (1.0 / v.norm())
}

/// FNV-1a of the case name: the seed of its generator.
pub fn seed_of(name: &str) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for b in name.bytes() {
        h = (h ^ u64::from(b)).wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}
pub fn rng(name: &str) -> SplitMix64 {
    SplitMix64(seed_of(name))
}

/// Number of lines for statistical tests: full in release, a tenth in debug builds.
pub fn n_lines() -> usize {
    if cfg!(debug_assertions) { 100_000 } else { 1_000_000 }
}

/// A uniformly distributed unit vector.
pub fn random_dir(r: &mut SplitMix64) -> Vector3 {
    let z = r.unit();
    let phi = std::f64::consts::PI * r.unit();
    let s = (1.0 - z * z).max(0.0).sqrt();
    v3(s * phi.cos(), s * phi.sin(), z)
}

/// A seeded frame at `origin` with a random orientation.
pub fn random_frame(r: &mut SplitMix64, origin: Point3) -> Frame {
    let z = random_dir(r);
    let hint = random_dir(r);
    Frame::from_zx(origin, z, hint).unwrap_or_else(|_| Frame::at(origin))
}

pub fn hs(s: EntityId, negative: bool) -> HalfSpace {
    HalfSpace { surface: s, sign: if negative { Sign::Negative } else { Sign::Positive } }
}

/// One intersection (term).
pub fn term(h: &[HalfSpace]) -> CsgCell {
    CsgCell::new(h.iter().copied())
}

/// A CSG leaf under construction.
pub struct Fx {
    pub store: MemoryStore,
    pub cells: Vec<CsgCellDef>,
    pub bounds: Option<Aabb>,
    pub complement: ComplementPolicy,
    pub boundaries: BoundarySpec,
}

impl Fx {
    /// Empty leaf with a complement region `"out"` inside `bounds`.
    pub fn new(half: f64) -> Self {
        Self {
            store: MemoryStore::new(),
            cells: Vec::new(),
            bounds: Some(Aabb { lo: p3(-half, -half, -half), hi: p3(half, half, half) }),
            complement: ComplementPolicy::Region("out".into()),
            boundaries: BoundarySpec::default(),
        }
    }
    pub fn add(&mut self, d: SurfaceDef) -> EntityId {
        self.store.add_surface(d).expect("valid surface")
    }
    pub fn store_add_raw(&mut self, d: SurfaceDef) -> Option<EntityId> {
        self.store.add_surface(d).ok()
    }
    pub fn plane(&mut self, o: Point3, n: Vector3) -> EntityId {
        self.add(SurfaceDef::Plane(Plane { frame: Frame::from_z(o, n).unwrap() }))
    }
    pub fn sphere(&mut self, c: Point3, r: f64) -> EntityId {
        self.add(SurfaceDef::Sphere(Sphere::new(Frame::at(c), r).unwrap()))
    }
    pub fn cylinder(&mut self, f: Frame, r: f64) -> EntityId {
        self.add(SurfaceDef::Cylinder(Cylinder::new(f, r).unwrap()))
    }
    pub fn cone(&mut self, apex_frame: Frame, half_angle: f64) -> EntityId {
        self.add(SurfaceDef::Cone(Cone::new(apex_frame, 0.0, half_angle).unwrap()))
    }
    pub fn torus(&mut self, f: Frame, major: f64, minor: f64) -> EntityId {
        self.add(SurfaceDef::Torus(Torus::new(f, major, minor).unwrap()))
    }
    pub fn quadric(&mut self, q: Quadric) -> EntityId {
        self.add(SurfaceDef::Implicit(ImplicitDef::Quadric(q)))
    }
    /// The six planes of the box `lo..hi` with outward normals, order `-x, +x, -y, +y, -z, +z`.
    pub fn box_planes(&mut self, lo: Point3, hi: Point3) -> [EntityId; 6] {
        [
            self.plane(lo, v3(-1.0, 0.0, 0.0)),
            self.plane(hi, v3(1.0, 0.0, 0.0)),
            self.plane(lo, v3(0.0, -1.0, 0.0)),
            self.plane(hi, v3(0.0, 1.0, 0.0)),
            self.plane(lo, v3(0.0, 0.0, -1.0)),
            self.plane(hi, v3(0.0, 0.0, 1.0)),
        ]
    }
    /// A cell that is a union of terms.
    pub fn cell(&mut self, name: &str, terms: Vec<CsgCell>) -> usize {
        self.cells.push(CsgCellDef::new(CsgRegion::new(terms), name));
        self.cells.len() - 1
    }
    /// A one-term cell.
    pub fn cell1(&mut self, name: &str, h: &[HalfSpace]) -> usize {
        self.cell(name, vec![term(h)])
    }

    /// Run `f` over the hierarchical input of this leaf.
    pub fn with_hier<R>(&self, f: impl FnOnce(&HierInput<'_>) -> R) -> R {
        let spec = WorldSpec::new(vec![ComponentKind::Leaf { input: 0 }], ComponentId(0), InstancePolicy::Auto);
        let leaf = CsgLeafInput::new(&self.store, &self.cells, self.bounds, &self.complement);
        let leaves = [LeafInput::Csg(leaf)];
        let h = HierInput::new(&spec, &leaves, &self.boundaries);
        f(&h)
    }
    pub fn build_with(&self, opts: &BuildOptions) -> Result<(AnalyticTracker, Vec<BuildIssue>)> {
        self.with_hier(|h| AnalyticTrackerBuilder.build_concrete(&BuildInput::Hier(*h), opts)).map(|(t, r)| (t, r.issues))
    }
    pub fn build(&self) -> AnalyticTracker {
        self.build_with(&BuildOptions::default()).expect("builds").0
    }
}

/// A traced line: `(cell, length)` per segment and how it ended.
pub struct Trace {
    pub segs: Vec<Segment>,
    pub end: TraceEnd,
}

pub fn trace(t: &AnalyticTracker, o: Point3, d: Vector3, max: f64, sc: &mut Scratch) -> Trace {
    let mut st = t.init_state(o, d, None, sc).expect("state");
    let mut segs = Vec::new();
    let end = t
        .trace(
            &mut st,
            &TraceOptions::default().with_max_dist(max),
            &mut |s: &Segment| {
                segs.push(*s);
                Flow::Continue
            },
            sc,
        )
        .expect("trace");
    Trace { segs, end }
}

impl Trace {
    /// Total length inside cells other than the complement.
    pub fn inside(&self) -> f64 {
        self.segs.iter().filter(|s| s.cell != CellId::COMPLEMENT).map(|s| s.length).sum()
    }
    /// Chords (consecutive segments of the same cell merged are not merged: one per segment) in
    /// cells other than the complement.
    pub fn chords(&self) -> Vec<f64> {
        self.segs.iter().filter(|s| s.cell != CellId::COMPLEMENT).map(|s| s.length).collect()
    }
    pub fn total(&self) -> f64 {
        self.segs.iter().map(|s| s.length).sum()
    }
}

/// A line through `centre` with direction `d` and perpendicular offset `b` along `e` (unit,
/// perpendicular to `d`), started `back` before the centre.
pub fn line(centre: Point3, d: Vector3, b: Vector3, back: f64) -> Point3 {
    centre + (b - d * back)
}

/// A unit vector perpendicular to `d`, from a random direction.
pub fn perp_to(r: &mut SplitMix64, d: Vector3) -> Vector3 {
    loop {
        let a = random_dir(r);
        let p = a - d * a.dot(d);
        if p.norm() > 1e-3 {
            return unit(p);
        }
    }
}
