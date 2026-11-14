//! [`WorldTracker`]: the hierarchical walker over prototype trackers (`BuildInput::Hier`).
//!
//! A world is a DAG of components (`WorldSpec`): leaves (a prototype [`MeshTracker`] built from a
//! mesh or a facet soup), unions of rigidly placed children and lattices (rectangular, and hexagonal in both orientations). Instances
//! share their prototype: regions, surfaces and faces are numbered per prototype
//! (prefix offsets over prototypes), cells per instance (prefix sums over instances, no table for
//! a lattice with a uniform fill). A particle's place is `TrackState::levels`: one entry per
//! union or lattice below the root.
//!
//! * A step runs the prototype's own `step` in the instance frame. Leaving the prototype through
//!   one of its outer faces is resolved by relocating the exit point from the root, with the
//!   direction deciding on shared walls: a neighbour instance (a `SlotChange` when the region is
//!   the same, a `RegionBoundary` otherwise) or the model boundary (the world's default
//!   boundary condition, `Reflect` or a `Respond` the caller resolves).
//! * Placements of a union must be disjoint. A point strictly inside two placements is
//!   [`LostCause::Overlap`](sm_rt_api::LostCause::Overlap) (never resolved by order); the step
//!   additionally tests, in unions whose placement boxes overlap, the chord it is about to
//!   take against each partner placement (see `probe_partners`).
//!
//! See `DESIGN.md`, "World".

use crate::bvh::{Box6, EMPTY_BOX, box_dist2, union};
use crate::builder::build_proto;
use crate::delta::flight::{box_exit, end_face};
use crate::hexlat::{self, Orient};
use crate::analytic::AnalyticTracker;
use crate::tracker::MeshTracker;
use crate::vec3::{dot, finite3, madd, norm, scale, sub};
use rayon::prelude::*;
use sm_core::{Aabb, Point3, Transform, Vector3};
use sm_geom_api::Ray;
use sm_mesh_api::MeshRead;
use sm_rt_api::{
    FlightSegment, HierarchyView, MajorantTracker, MaterialSetId, MaterialSets, NodeId, NodeInfo, NodeKind, SegmentQuery, SegmentScale, TrackMode, choose_mode,
    BinMap, BoundaryAction, BoundaryDef, BuildIssue, BuildOptions, BuildReport, CacheImage, CacheKey, CellId, Closest, ComponentId, ComponentKind, ContentHash, ContentHasher, FaceAction,
    FaceId, HierInput, InstanceId, LatticeKind, LeafInput, Level, LostCause, Located, PeriodicImage, RegionId, RegionTable, ResponseKind, Result, RtError, Scratch,
    Step, StepEnd, SurfaceId, SurfaceInfo, SurfaceTable, TallySpec, TraceOptions, TrackBundle, TrackState, Tracker, TrackerCaps, TrackerInfo, hashed_direction, lost,
    LEVEL_INLINE,
};
use smallvec::SmallVec;
use std::collections::BTreeSet;

type Levels = SmallVec<[Level; LEVEL_INLINE]>;

/// Largest global cell count (ids are `u32`; `NONE` and `COMPLEMENT` are reserved).
const MAX_CELLS: u64 = u32::MAX as u64 - 2;
const MAX_ISSUES: usize = 16;

// ---- affine maps -----------------------------------------------------------------------

/// `p -> m p + t` (rigid in practice: rotation, reflection and translation).
#[derive(Clone, Copy, Debug, PartialEq)]
struct Aff {
    m: [[f64; 3]; 3],
    t: [f64; 3],
}

impl Aff {
    const ID: Aff = Aff { m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: [0.0; 3] };

    fn from_transform(t: &Transform) -> Self {
        Self { m: t.m, t: [t.t.x, t.t.y, t.t.z] }
    }
    /// The map `p -> p - c`.
    fn shift_by(c: [f64; 3]) -> Self {
        Self { m: Self::ID.m, t: [-c[0], -c[1], -c[2]] }
    }
    #[inline]
    fn vec(&self, v: [f64; 3]) -> [f64; 3] {
        let m = &self.m;
        [m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2], m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2], m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]]
    }
    #[inline]
    fn pt(&self, p: [f64; 3]) -> [f64; 3] {
        let v = self.vec(p);
        [v[0] + self.t[0], v[1] + self.t[1], v[2] + self.t[2]]
    }
    /// Apply `self` first, then `next`.
    fn then(&self, next: &Aff) -> Aff {
        let a = &next.m;
        let b = &self.m;
        let mut m = [[0.0; 3]; 3];
        for (i, row) in m.iter_mut().enumerate() {
            for (j, e) in row.iter_mut().enumerate() {
                *e = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
            }
        }
        let t = next.pt(self.t);
        Aff { m, t }
    }
    /// `M^T v`: the vector map of the inverse of a rotation or reflection.
    #[inline]
    fn vec_back(&self, v: [f64; 3]) -> [f64; 3] {
        let m = &self.m;
        [m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2], m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2], m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2]]
    }
    /// Inverse image of a point (rigid maps only).
    fn pt_back(&self, q: [f64; 3]) -> [f64; 3] {
        self.vec_back(sub(q, self.t))
    }
}

fn is_rigid(t: &Transform) -> bool {
    let m = &t.m;
    let mut ok = m.iter().flatten().all(|v| v.is_finite());
    for i in 0..3 {
        for j in 0..3 {
            let d: f64 = (0..3).map(|k| m[k][i] * m[k][j]).sum();
            ok &= (d - if i == j { 1.0 } else { 0.0 }).abs() <= 1e-12;
        }
    }
    ok
}

fn box_of_points(pts: impl Iterator<Item = [f64; 3]>) -> Box6 {
    let mut b = EMPTY_BOX;
    for p in pts {
        b = union(&b, &[p[0], p[1], p[2], p[0], p[1], p[2]]);
    }
    b
}

fn corners(b: &Box6) -> [[f64; 3]; 8] {
    let mut c = [[0.0; 3]; 8];
    for (i, p) in c.iter_mut().enumerate() {
        *p = [b[3 * (i & 1)], b[1 + 3 * ((i >> 1) & 1)], b[2 + 3 * ((i >> 2) & 1)]];
    }
    c
}

fn box_empty(b: &Box6) -> bool {
    b[0] > b[3]
}

fn inside_box(b: &Box6, p: [f64; 3], pad: f64) -> bool {
    (0..3).all(|k| p[k] >= b[k] - pad && p[k] <= b[3 + k] + pad)
}

fn boxes_touch(a: &Box6, b: &Box6, pad: f64) -> bool {
    (0..3).all(|k| a[k] <= b[3 + k] + pad && b[k] <= a[3 + k] + pad)
}

/// Parameter interval of the line `o + t d` inside the box (padded), within `[0, t_max]`.
fn slab(o: [f64; 3], d: [f64; 3], b: &Box6, pad: f64, t_max: f64) -> Option<(f64, f64)> {
    let (mut lo, mut hi) = (0.0f64, t_max);
    for k in 0..3 {
        let (bl, bh) = (b[k] - pad, b[3 + k] + pad);
        if d[k] == 0.0 {
            if o[k] < bl || o[k] > bh {
                return None;
            }
        } else {
            let (mut a, mut c) = ((bl - o[k]) / d[k], (bh - o[k]) / d[k]);
            if a > c {
                core::mem::swap(&mut a, &mut c);
            }
            lo = lo.max(a);
            hi = hi.min(c);
            if lo > hi {
                return None;
            }
        }
    }
    Some((lo, hi))
}

// ---- compiled world --------------------------------------------------------------------

/// The tracker of one prototype: a mesh or facet leaf, or an analytic CSG leaf.
#[allow(clippy::large_enum_variant)] // boxing the mesh tracker adds a pointer chase to every step
pub(crate) enum Body {
    Mesh(MeshTracker),
    Csg(Box<AnalyticTracker>),
}

impl Body {
    fn mesh(&self) -> Option<&MeshTracker> {
        match self {
            Body::Mesh(m) => Some(m),
            Body::Csg(_) => None,
        }
    }
    fn dynt(&self) -> &dyn Tracker {
        match self {
            Body::Mesh(m) => m,
            Body::Csg(a) => &**a,
        }
    }
    fn regions(&self) -> &RegionTable {
        self.dynt().regions()
    }
    fn surfaces(&self) -> &SurfaceTable {
        self.dynt().surfaces()
    }
    fn info(&self) -> &TrackerInfo {
        self.dynt().info()
    }
    fn surface_faces(&self, s: SurfaceId) -> &[FaceId] {
        self.dynt().surface_faces(s)
    }
    fn surface_area(&self, s: SurfaceId) -> Result<Option<f64>> {
        self.dynt().surface_area(s)
    }
    fn n_cells(&self) -> usize {
        match self {
            Body::Mesh(m) => m.core.n_cells(),
            Body::Csg(a) => a.n_cells(),
        }
    }
    fn n_faces(&self) -> usize {
        match self {
            Body::Mesh(m) => m.core.n_faces(),
            Body::Csg(a) => a.info().faces,
        }
    }
    fn c_region(&self, local: usize) -> Option<u32> {
        match self {
            Body::Mesh(m) => m.core.c_region.get(local).copied(),
            Body::Csg(a) => a.cell_region(CellId::new(local)).map(|r| r.0),
        }
    }
    fn c_volume(&self, local: usize) -> Option<f64> {
        match self {
            Body::Mesh(m) => m.core.c_volume.get(local).copied(),
            Body::Csg(a) => a.cell_volume(CellId::new(local)),
        }
    }
    fn step(&self, st: &mut TrackState, max_dist: f64, sc: &mut Scratch) -> Result<Step> {
        match self {
            Body::Mesh(m) => m.step(st, max_dist, sc),
            Body::Csg(a) => a.step(st, max_dist, sc),
        }
    }
    /// Locate a leaf-frame point; the flag is set when two cells of the leaf hold it (an overlap).
    fn locate(&self, p: [f64; 3], dir: [f64; 3], hint: Option<CellId>, sc: &mut Scratch) -> (Located, bool) {
        match self {
            Body::Mesh(m) => (m.locate_inner(p, Some(dir), hint, sc), false),
            Body::Csg(a) => a.locate_ov(p, dir, hint, sc),
        }
    }
    /// Is a located point strictly inside a cell of the leaf (not on one of its outer faces)?
    fn strict(&self, loc: &Located) -> bool {
        if loc.cell.is_none() || loc.cell == CellId::COMPLEMENT {
            return false;
        }
        match self {
            Body::Mesh(m) => loc.on_face.is_none_or(|f| m.core.f_right[f.get()] != crate::core::NONE),
            Body::Csg(a) => loc.on_face.is_none_or(|f| a.face_interior(f)),
        }
    }
    /// Normal of a face at a leaf-frame point (a plane's normal for a mesh face).
    fn face_normal(&self, f: FaceId, p: [f64; 3]) -> Result<[f64; 3]> {
        match self {
            Body::Mesh(m) => m.face_normal(f, p3(p)).map(av3),
            Body::Csg(a) => a.face_normal(f, p3(p)).map(av3),
        }
    }
    fn closest_inner(&self, p: [f64; 3], region: Option<RegionId>, sc: &mut Scratch) -> Option<Closest> {
        match self {
            Body::Mesh(m) => m.closest_inner(p, region, sc),
            // A general quadric has no closest point: the leaf is skipped, as `closest` refuses it.
            Body::Csg(a) => a.closest(p3(p), region, sc).ok().flatten(),
        }
    }
    fn face_dist2(&self, f: usize, p: [f64; 3]) -> f64 {
        match self {
            Body::Mesh(m) => m.core.face_dist2(f, p).0,
            Body::Csg(a) => {
                let d = a.face_dist(FaceId::new(f), p);
                d * d
            }
        }
    }
    /// First entry of the ray into the leaf from outside, within `max_t`.
    fn enter_from_outside(&self, o: [f64; 3], d: [f64; 3], tol: f64, max_t: f64, sc: &mut Scratch) -> Option<f64> {
        match self {
            Body::Mesh(m) => m.core.enter_from_outside(o, d, tol, &mut sc.ids).map(|(t, _)| t),
            Body::Csg(a) => a.enter_from_outside(o, d, max_t, sc).map(|(t, _)| t),
        }
    }
    fn partition(&self) -> Option<&crate::delta::Partition> {
        self.mesh().and_then(MeshTracker::partition)
    }
}

struct Proto {
    tracker: Body,
    region_off: u32,
    surf_off: u32,
    face_off: u32,
}

struct Place {
    child: usize,
    fwd: Aff,
    inv: Aff,
    /// Box of the child in the union's frame.
    bbox: Box6,
}

/// Element layout of a lattice.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Shape {
    Rect,
    Hex(Orient),
}

struct Lat {
    shape: Shape,
    /// Rect: lower corner. Hex: centre of element `(0, 0, 0)`.
    origin: [f64; 3],
    /// Rect: pitch per axis. Hex: `[p, p, z_pitch]` (`z_pitch` is `0` for a single layer).
    pitch: [f64; 3],
    dims: [u32; 3],
    fill: Vec<u32>,
    /// All elements hold this component: cell ids are `element * cells(child)`, no table.
    uniform: Option<u32>,
    /// Prefix sum of cells over elements (empty when `uniform`).
    off: Vec<u64>,
}

impl Lat {
    fn center(&self, idx: [i32; 3]) -> [f64; 3] {
        match self.shape {
            Shape::Rect => [
                self.origin[0] + (f64::from(idx[0]) + 0.5) * self.pitch[0],
                self.origin[1] + (f64::from(idx[1]) + 0.5) * self.pitch[1],
                self.origin[2] + (f64::from(idx[2]) + 0.5) * self.pitch[2],
            ],
            Shape::Hex(o) => {
                let c = hexlat::center(o, self.pitch[0], idx[0], idx[1]);
                [self.origin[0] + c[0], self.origin[1] + c[1], self.origin[2] + f64::from(idx[2]) * self.pitch[2]]
            }
        }
    }
    fn lin(&self, idx: [i32; 3]) -> usize {
        idx[0] as usize + self.dims[0] as usize * (idx[1] as usize + self.dims[1] as usize * idx[2] as usize)
    }
    fn idx_of(&self, lin: usize) -> [i32; 3] {
        let (d0, d1) = (self.dims[0] as usize, self.dims[1] as usize);
        [(lin % d0) as i32, ((lin / d0) % d1) as i32, (lin / (d0 * d1)) as i32]
    }
    fn in_range(&self, idx: [i32; 3]) -> bool {
        (0..3).all(|a| idx[a] >= 0 && (idx[a] as u32) < self.dims[a])
    }
    fn n(&self) -> usize {
        self.fill.len()
    }
    /// Candidate indices along axis `a` for coordinate `p`: the containing slab, or both slabs
    /// of a wall within `tol`, the one the direction enters first. Axis z of a single-layer
    /// lattice is unconstrained (index 0). A hex z layer is centred on its index, so its walls
    /// are at half-integers (`shift` 0.5).
    fn axis_cands(&self, a: usize, p: f64, dir: f64, tol: f64) -> ([i32; 2], usize) {
        if a == 2 && self.dims[2] == 1 {
            return ([0, 0], 1);
        }
        let shift = if a == 2 && matches!(self.shape, Shape::Hex(_)) { 0.5 } else { 0.0 };
        let x = (p - self.origin[a]) / self.pitch[a] + shift;
        let r = x.round();
        if (x - r).abs() * self.pitch[a] <= tol {
            let (lo, hi) = (r as i32 - 1, r as i32);
            if dir > 0.0 { ([hi, lo], 2) } else { ([lo, hi], 2) }
        } else {
            ([x.floor() as i32, 0], 1)
        }
    }

    /// Element indices to try for the point `p` (lattice frame) moving along `dir`, in order:
    /// z layers outermost, then the planar candidates (see [`hexlat::candidates`] for the hex
    /// vertex-tie rule). Indices may be out of range.
    fn cands(&self, p: [f64; 3], dir: [f64; 3], tol: f64) -> SmallVec<[[i32; 3]; 8]> {
        let (cz, nz) = self.axis_cands(2, p[2], dir[2], tol);
        let mut out: SmallVec<[[i32; 3]; 8]> = SmallVec::new();
        match self.shape {
            Shape::Rect => {
                let (cx, nx) = self.axis_cands(0, p[0], dir[0], tol);
                let (cy, ny) = self.axis_cands(1, p[1], dir[1], tol);
                for &kz in &cz[..nz] {
                    for &ky in &cy[..ny] {
                        for &kx in &cx[..nx] {
                            out.push([kx, ky, kz]);
                        }
                    }
                }
            }
            Shape::Hex(o) => {
                let u = [p[0] - self.origin[0], p[1] - self.origin[1]];
                let xy = hexlat::candidates(o, self.pitch[0], u, [dir[0], dir[1]], tol);
                for &kz in &cz[..nz] {
                    for &(q, r) in &xy {
                        out.push([q, r, kz]);
                    }
                }
            }
        }
        out
    }
}

enum Kind {
    Leaf(usize),
    Union {
        places: Vec<Place>,
        /// Prefix sum of cells over placements.
        off: Vec<u64>,
        /// For each placement, the other placements whose boxes overlap its box (ascending).
        partners: Vec<Vec<u32>>,
    },
    Lattice(Lat),
}

struct Comp {
    kind: Kind,
    /// Bounding box in the component's own frame (empty box if it holds no cell).
    bounds: Box6,
    /// Cells of one instance.
    cells: u64,
    /// Leaf instances below one instance.
    inst: u64,
}

/// A union level of the current path whose placement has overlap partners.
struct Check {
    comp: usize,
    place: usize,
    /// Global-to-union-frame map.
    aff: Aff,
}

struct Frame {
    proto: usize,
    g2l: Aff,
    base: u64,
    checks: SmallVec<[Check; 2]>,
}

struct Hit {
    proto: usize,
    base: u64,
    local: Located,
    levels: Levels,
    /// Strictly inside a cell of the leaf body (not within tolerance of an outer face).
    strict: bool,
}

#[allow(clippy::large_enum_variant)] // boxing the hit would allocate on the hot path
enum Loc {
    Miss,
    Overlap,
    Hit(Hit),
}

/// Hierarchical tracker over prototype [`MeshTracker`]s. Built by
/// [`crate::MeshTrackerBuilder`] from [`sm_rt_api::BuildInput::Hier`]; immutable and `Sync`.
///
/// Semantics fixed by this implementation:
/// * A lattice element holds its fill in the element's frame (origin at the element centre,
///   `Lattice::element_center`). A fill must lie inside its element (checked at build,
///   `world.lattice_fit`); a gap inside an element is outside the model. Single-layer lattices
///   (`dims[2] == 1`) have no z walls.
/// * The leaf's own boundary conditions and complement policy do not apply in a world; the
///   world's `BoundarySpec::default` applies where a ray leaves the last instance. Per-surface
///   rules, periodic boundaries and lattice `outer` fills are
///   `RtError::Unsupported`. A CSG leaf is hosted as an [`AnalyticTracker`] prototype (no complement
///   region, vacuum boundaries; `partition.enabled` is refused for it).
/// * Region and surface tables hold each prototype once (regions are prototypes); a surface or
///   face id names the prototype's, `surface_area` is the area of one instance. `SurfaceInfo`
///   boundary actions are not recorded (they are a property of the world, not of a prototype).
/// * No cache image: `export_cache` is `None`.
pub struct WorldTracker {
    protos: Vec<Proto>,
    comps: Vec<Comp>,
    root: usize,
    regions: RegionTable,
    surfaces: SurfaceTable,
    sf_off: Vec<u32>,
    sf: Vec<FaceId>,
    exit_action: FaceAction,
    info: TrackerInfo,
    tol: f64,
    block_size: usize,
    any_partners: bool,
    overlap_vertex_tests: u64,
    /// The majorant node tables (`PartitionOptions::enabled`).
    hier: Option<WorldHier>,
}

impl core::fmt::Debug for WorldTracker {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("WorldTracker").field("prototypes", &self.protos.len()).field("components", &self.comps.len()).field("cells", &self.info.cells).finish()
    }
}

fn issue(code: &'static str, msg: String) -> RtError {
    RtError::Build(vec![BuildIssue::new(code, msg)])
}

fn p3(a: [f64; 3]) -> Point3 {
    Point3::new(a[0], a[1], a[2])
}
fn v3(a: [f64; 3]) -> Vector3 {
    Vector3::new(a[0], a[1], a[2])
}
fn a3(p: Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}
fn av3(v: Vector3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

impl WorldTracker {
    /// Build from a hierarchical input.
    pub(crate) fn build(h: &HierInput<'_>, opts: &BuildOptions, key: CacheKey) -> Result<(Self, BuildReport)> {
        let mut report = BuildReport::default();
        report.issues.extend(h.spec.validate(h.leaves.len())?);
        let exit_action = match h.boundaries.default {
            _ if !h.boundaries.rules.is_empty() => return Err(RtError::Unsupported("per-surface boundary rules on a hierarchical world; only the default applies")),
            BoundaryDef::Reflect => FaceAction::Reflect,
            BoundaryDef::Respond(r) => {
                if !r.is_valid() {
                    return Err(issue("boundary.bad_albedo", "albedo must be in [0, 1]".into()));
                }
                FaceAction::Respond(r)
            }
            BoundaryDef::Wrap { .. } => return Err(RtError::Unsupported("periodic boundary on a hierarchical world")),
            _ => FaceAction::Respond(ResponseKind::Vacuum),
        };
        if opts.partition.region_jump {
            return Err(RtError::Unsupported("region jump on a hierarchical world (flights use the prototypes' kd partitions; `fire` walks)"));
        }
        let spec = h.spec;
        let n = spec.components.len();
        let tol = opts.tol.linear;

        // Prototypes: one per distinct leaf input referenced, ascending input index.
        let mut used: BTreeSet<usize> = BTreeSet::new();
        for c in &spec.components {
            if let ComponentKind::Leaf { input } = c {
                used.insert(*input);
            }
        }
        let mut proto_of_input = vec![usize::MAX; h.leaves.len()];
        let mut protos: Vec<Proto> = Vec::new();
        let (mut ro, mut so, mut fo) = (0u64, 0u64, 0u64);
        for &inp in &used {
            let leaf: &LeafInput<'_> = &h.leaves[inp];
            let (tracker, rep) = build_proto(leaf, opts).map_err(|e| match e {
                RtError::Build(mut v) => {
                    for i in &mut v {
                        i.message = format!("leaf {inp}: {}", i.message);
                    }
                    RtError::Build(v)
                }
                other => other,
            })?;
            report.issues.extend(rep.issues);
            let to32 = |x: u64| u32::try_from(x).map_err(|_| RtError::Unsupported("more than u32::MAX regions, surfaces or faces over all prototypes"));
            let p = Proto { region_off: to32(ro)?, surf_off: to32(so)?, face_off: to32(fo)?, tracker };
            ro += p.tracker.regions().0.len() as u64;
            so += p.tracker.surfaces().0.len() as u64;
            fo += p.tracker.info().faces as u64;
            proto_of_input[inp] = protos.len();
            protos.push(p);
        }
        to_u32_check(ro, so, fo)?;

        // Tables.
        let mut regions = Vec::new();
        let mut surfaces = Vec::new();
        let mut sf: Vec<FaceId> = Vec::new();
        let mut sf_off: Vec<u32> = vec![0];
        for p in &protos {
            for r in &p.tracker.regions().0 {
                regions.push(r.clone());
            }
            for (si, s) in p.tracker.surfaces().0.iter().enumerate() {
                let sh = |r: RegionId| if r.is_none() { r } else { RegionId(r.0 + p.region_off) };
                let mut ns = SurfaceInfo::new(sh(s.front), sh(s.back));
                if let Some(e) = s.entity {
                    ns = ns.with_entity(e);
                }
                surfaces.push(ns);
                sf.extend(p.tracker.surface_faces(SurfaceId::new(si)).iter().map(|f| FaceId(f.0 + p.face_off)));
                sf_off.push(sf.len() as u32);
            }
        }

        // Components, children before parents.
        let kids = |c: usize| -> Vec<usize> {
            match &spec.components[c] {
                ComponentKind::Leaf { .. } => vec![],
                ComponentKind::Union { placements } => placements.iter().map(|p| p.child.get()).collect(),
                ComponentKind::Lattice(l) => l.fill.iter().map(|c| c.get()).collect(),
                _ => vec![],
            }
        };
        let order = post_order(n, spec.root.get(), &kids);
        let mut comps: Vec<Option<Comp>> = (0..n).map(|_| None).collect();
        for &c in &order {
            let comp = compile(spec.components.get(c), c, &comps, &protos, &proto_of_input, tol)?;
            comps[c] = Some(comp);
        }
        let root = spec.root.get();
        // Unreachable components compile to an inert leaf-less placeholder; keep indices dense.
        let comps: Vec<Comp> = comps
            .into_iter()
            .map(|c| c.unwrap_or(Comp { kind: Kind::Union { places: vec![], off: vec![], partners: vec![] }, bounds: EMPTY_BOX, cells: 0, inst: 0 }))
            .collect();
        if comps[root].cells == 0 {
            return Err(issue("world.empty", "the world has no cells".into()));
        }
        let any_partners = comps.iter().any(|c| matches!(&c.kind, Kind::Union { partners, .. } if partners.iter().any(|p| !p.is_empty())));

        // Info.
        let b = comps[root].bounds;
        let mut info = TrackerInfo::new(Aabb { lo: Point3::new(b[0], b[1], b[2]), hi: Point3::new(b[3], b[4], b[5]) }, comps[root].cells as usize, ContentHash(0));
        info.faces = protos.iter().map(|p| p.tracker.info().faces).sum();
        info.prototypes = protos.len();
        info.copies = comps[root].inst as usize;
        info.exact_faces = protos.iter().map(|p| p.tracker.info().exact_faces).sum();
        info.faceted_faces = protos.iter().map(|p| p.tracker.info().faceted_faces).sum();
        info.memory_bytes = protos.iter().map(|p| p.tracker.info().memory_bytes).sum::<usize>()
            + comps.iter().map(|c| core::mem::size_of::<Comp>() + comp_bytes(c)).sum::<usize>()
            + sf.len() * 4;
        info.tol = opts.tol;
        info.max_events = opts.max_events;
        info.key = Some(key);
        let mut t = Self {
            protos,
            comps,
            root,
            regions: RegionTable(regions),
            surfaces: SurfaceTable(surfaces),
            sf_off,
            sf,
            exit_action,
            info,
            tol,
            block_size: opts.block_size as usize,
            any_partners,
            overlap_vertex_tests: 0,
            hier: None,
        };
        t.info.structure_hash = t.hash_structure(spec.content_hash());
        t.narrow_phase()?;
        if opts.partition.enabled {
            let hier = build_hier(&t.protos, &t.comps, t.root)?;
            t.info.memory_bytes += hier.bytes();
            t.hier = Some(hier);
        }
        Ok((t, report))
    }

    fn hash_structure(&self, spec_hash: ContentHash) -> ContentHash {
        let mut h = ContentHasher::new();
        h.write_u64(3);
        h.write_hash(spec_hash);
        h.write_u64(self.protos.len() as u64);
        for p in &self.protos {
            h.write_hash(p.tracker.info().structure_hash);
            h.write_u64(u64::from(p.region_off));
            h.write_u64(u64::from(p.surf_off));
            h.write_u64(u64::from(p.face_off));
        }
        self.exit_action.hash_into(&mut h);
        h.write_u64(self.root as u64);
        h.write_u64(self.comps.len() as u64);
        for c in &self.comps {
            h.write_u64(c.cells);
            h.write_u64(c.inst);
            for &x in &c.bounds {
                h.write_f64(x);
            }
            match &c.kind {
                Kind::Leaf(p) => {
                    h.write_u64(1);
                    h.write_u64(*p as u64);
                }
                Kind::Union { places, off, partners } => {
                    h.write_u64(2);
                    h.write_u64(places.len() as u64);
                    for (i, p) in places.iter().enumerate() {
                        h.write_u64(p.child as u64);
                        for x in p.fwd.m.iter().flatten().chain(p.fwd.t.iter()) {
                            h.write_f64(*x);
                        }
                        h.write_u64(off[i]);
                        h.write_u64(partners[i].len() as u64);
                        partners[i].iter().for_each(|&q| h.write_u64(u64::from(q)));
                    }
                }
                Kind::Lattice(l) => {
                    h.write_u64(3);
                    l.origin.iter().chain(l.pitch.iter()).for_each(|&x| h.write_f64(x));
                    // Rect worlds keep their byte stream; a hex lattice adds its orientation.
                    if let Shape::Hex(o) = l.shape {
                        h.write_u64(if o == Orient::XRows { 2 } else { 3 });
                    }
                    l.dims.iter().for_each(|&d| h.write_u64(u64::from(d)));
                    h.write_u64(l.fill.len() as u64);
                    l.fill.iter().for_each(|&f| h.write_u64(u64::from(f)));
                    h.write_u64(l.uniform.map_or(u64::MAX, u64::from));
                    h.write_u64(l.off.len() as u64);
                    l.off.iter().for_each(|&o| h.write_u64(o));
                }
            }
        }
        h.finish()
    }

    /// Number of boundary-vertex location tests the build-time overlap check made (a counter, not
    /// a clock).
    pub fn overlap_vertex_tests(&self) -> u64 {
        self.overlap_vertex_tests
    }

    // ---- build-time overlap check ----------------------------------------------------------

    /// Vertices of the outer boundary of component `c`, in its own frame.
    fn boundary_vertices(&self, c: usize, out: &mut Vec<[f64; 3]>) {
        boundary_vertices_of(&|i| Some(&self.comps[i]), &self.protos, c, out);
    }

    /// Exact narrow phase over placement pairs whose boxes overlap: a boundary vertex of one
    /// strictly inside a cell of the other is `world.overlap`.
    fn narrow_phase(&mut self) -> Result<()> {
        let mut issues: Vec<BuildIssue> = Vec::new();
        let mut tests = 0u64;
        let mut sc = Scratch::new();
        for ci in 0..self.comps.len() {
            let Kind::Union { places, partners, .. } = &self.comps[ci].kind else { continue };
            for (i, plist) in partners.iter().enumerate() {
                for &j in plist {
                    let j = j as usize;
                    if j < i {
                        continue;
                    }
                    for (a, b) in [(i, j), (j, i)] {
                        let (pa, pb) = (&places[a], &places[b]);
                        let mut verts = Vec::new();
                        self.boundary_vertices(pa.child, &mut verts);
                        for v in verts {
                            let w = pa.fwd.pt(v);
                            if !inside_box(&pb.bbox, w, self.tol) {
                                continue;
                            }
                            tests += 1;
                            let q = pb.inv.pt(w);
                            let d = av3(hashed_direction(p3(w)));
                            if let Loc::Hit(h) = self.locate_rec(pb.child, q, pb.inv.vec(d), &mut sc, None, false)
                                && h.strict
                            {
                                if issues.len() < MAX_ISSUES {
                                    issues.push(
                                        BuildIssue::new(
                                            "world.overlap",
                                            format!("component {ci}: placements {a} (component {}) and {b} (component {}) overlap near ({:e}, {:e}, {:e}) in the frame of component {ci}", pa.child, pb.child, w[0], w[1], w[2]),
                                        )
                                        .at(p3(w))
                                        .suggest("make the placements disjoint"),
                                    );
                                }
                                break;
                            }
                        }
                    }
                }
            }
        }
        self.overlap_vertex_tests = tests;
        if issues.is_empty() { Ok(()) } else { Err(RtError::Build(issues)) }
    }

    // ---- paths and frames ---------------------------------------------------------------

    fn bad_levels() -> RtError {
        RtError::InvalidInput("TrackState::levels do not describe an instance of this world")
    }

    fn cell_off_in_lattice(&self, l: &Lat, lin: usize) -> u64 {
        match l.uniform {
            Some(c) => lin as u64 * self.comps[c as usize].cells,
            None => l.off[lin],
        }
    }

    /// Resolve a level path to its leaf prototype, the global-to-leaf map and the cell base.
    fn frame_of(&self, levels: &[Level]) -> Result<Frame> {
        let mut comp = self.root;
        let mut aff = Aff::ID;
        let mut base = 0u64;
        let mut checks: SmallVec<[Check; 2]> = SmallVec::new();
        for lvl in levels {
            if lvl.component.get() != comp {
                return Err(Self::bad_levels());
            }
            match &self.comps[comp].kind {
                Kind::Union { places, off, partners } => {
                    let i = lvl.instance.get();
                    let p = places.get(i).ok_or_else(Self::bad_levels)?;
                    if !partners[i].is_empty() {
                        checks.push(Check { comp, place: i, aff });
                    }
                    base += off[i];
                    aff = aff.then(&p.inv);
                    comp = p.child;
                }
                Kind::Lattice(l) => {
                    if !l.in_range(lvl.lattice) || l.lin(lvl.lattice) != lvl.instance.get() {
                        return Err(Self::bad_levels());
                    }
                    let lin = lvl.instance.get();
                    base += self.cell_off_in_lattice(l, lin);
                    aff = aff.then(&Aff::shift_by(l.center(lvl.lattice)));
                    comp = l.fill[lin] as usize;
                }
                Kind::Leaf(_) => return Err(Self::bad_levels()),
            }
        }
        match &self.comps[comp].kind {
            Kind::Leaf(p) => Ok(Frame { proto: *p, g2l: aff, base, checks }),
            _ => Err(Self::bad_levels()),
        }
    }

    /// Path, prototype and local cell of a global cell id.
    fn resolve_cell(&self, c: u64) -> Option<(Levels, usize, u32)> {
        if c >= self.comps[self.root].cells {
            return None;
        }
        let mut comp = self.root;
        let mut rem = c;
        let mut levels = Levels::new();
        loop {
            match &self.comps[comp].kind {
                Kind::Leaf(p) => return Some((levels, *p, u32::try_from(rem).ok()?)),
                Kind::Union { places, off, .. } => {
                    let i = off.partition_point(|&o| o <= rem).checked_sub(1)?;
                    rem -= off[i];
                    levels.push(Level::new(ComponentId::new(comp), InstanceId::new(i), [0; 3]));
                    comp = places[i].child;
                }
                Kind::Lattice(l) => {
                    let lin = match l.uniform {
                        Some(c) => {
                            let cells = self.comps[c as usize].cells;
                            let e = (rem / cells) as usize;
                            rem %= cells;
                            e
                        }
                        None => {
                            let e = l.off.partition_point(|&o| o <= rem).checked_sub(1)?;
                            rem -= l.off[e];
                            e
                        }
                    };
                    levels.push(Level::new(ComponentId::new(comp), InstanceId::new(lin), l.idx_of(lin)));
                    comp = l.fill[lin] as usize;
                }
            }
        }
    }

    fn proto_of_region(&self, r: RegionId) -> Option<usize> {
        let i = self.protos.partition_point(|p| p.region_off <= r.0).checked_sub(1)?;
        (r.get() < self.regions.0.len()).then_some(i)
    }
    fn proto_of_face(&self, f: u32) -> Option<usize> {
        let i = self.protos.partition_point(|p| p.face_off <= f).checked_sub(1)?;
        (((f - self.protos[i].face_off) as usize) < self.protos[i].tracker.info().faces).then_some(i)
    }
    fn proto_of_surface(&self, s: u32) -> Option<usize> {
        let i = self.protos.partition_point(|p| p.surf_off <= s).checked_sub(1)?;
        (((s - self.protos[i].surf_off) as usize) < self.protos[i].tracker.surfaces().0.len()).then_some(i)
    }

    // ---- locate -------------------------------------------------------------------------

    /// `excl`: the path of an instance to skip (`Some(rest)` while on that path; at the leaf with
    /// nothing left the instance is the excluded one). A particle that just left an instance
    /// through an outer face must not be given that instance back by rounding.
    fn locate_rec(&self, comp: usize, p: [f64; 3], dir: [f64; 3], sc: &mut Scratch, excl: Option<&[Level]>, carry: bool) -> Loc {
        let c = &self.comps[comp];
        if box_empty(&c.bounds) || !inside_box(&c.bounds, p, self.tol) {
            return Loc::Miss;
        }
        match &c.kind {
            Kind::Leaf(pi) => {
                if excl.is_some_and(|r| r.is_empty()) {
                    return Loc::Miss;
                }
                let tr = &self.protos[*pi].tracker;
                // With a partition the leaf is located from its kd candidate lists: the same
                // answer as the BVH path, without a BVH descent.
                let loc = match tr.mesh().and_then(|m| m.partition().map(|part| (m, part))) {
                    Some((m, part)) => m.locate_from_root(part, p, dir, sc),
                    None => {
                        let (loc, overlap) = tr.locate(p, dir, None, sc);
                        if overlap {
                            return Loc::Overlap;
                        }
                        loc
                    }
                };
                if loc.cell.is_none() || loc.cell == CellId::COMPLEMENT {
                    return Loc::Miss;
                }
                let strict = tr.strict(&loc);
                Loc::Hit(Hit { proto: *pi, base: 0, local: loc, levels: Levels::new(), strict })
            }
            Kind::Union { places, off, .. } => {
                let mut best: Option<(usize, Hit)> = None;
                for (i, pl) in places.iter().enumerate() {
                    if !inside_box(&pl.bbox, p, self.tol) {
                        continue;
                    }
                    let sub_excl = excl.and_then(|r| r.split_first()).filter(|(l, _)| l.component.get() == comp && l.instance.get() == i).map(|(_, rest)| rest);
                    match self.locate_rec(pl.child, pl.inv.pt(p), pl.inv.vec(dir), sc, sub_excl, carry) {
                        Loc::Miss => {}
                        Loc::Overlap => return Loc::Overlap,
                        Loc::Hit(h) => match &best {
                            None => best = Some((i, h)),
                            Some((_, b)) => {
                                if h.strict && b.strict {
                                    return Loc::Overlap;
                                }
                                if h.strict && !b.strict {
                                    best = Some((i, h));
                                }
                            }
                        },
                    }
                }
                match best {
                    None => Loc::Miss,
                    Some((i, mut h)) => {
                        h.base += off[i];
                        h.levels.insert(0, Level::new(ComponentId::new(comp), InstanceId::new(i), [0; 3]));
                        Loc::Hit(h)
                    }
                }
            }
            Kind::Lattice(l) => {
                // A hex point on a shared flat, with the direction along it: each leaf rules on
                // its own tangential tie and may take the point yet leave at once (two leaves
                // handing a ray back and forth with zero-length steps) or refuse it. The point is
                // still inside the footprint, so take the first candidate in order other than the
                // excluded one, ruled on with the direction nudged toward its centre (the state's
                // direction is not changed), provided the ray stays within `tol` of its hexagon
                // over a run of one edge. After a zero-length exit (`carry`) this goes first.
                // A hex stack and a ray parallel to the z-walls (`dz` too small to change layer
                // within the tolerance over any run in the lattice): a point on a hex flat of the
                // element the ray left is not handed to the neighbouring layer of the same column,
                // which holds it only within `tol` and ends the run at once (zero-length
                // hand-backs between the two layers).
                let parallel_z = matches!(l.shape, Shape::Hex(_)) && dir[2].abs() <= 1e-9;
                let excl_col = excl.and_then(|r| r.first()).filter(|lv| lv.component.get() == comp && parallel_z).map(|lv| lv.lattice);
                let same_column = |idx: [i32; 3]| {
                    let Shape::Hex(or) = l.shape else { return false };
                    // Only for a point on a hex flat of that column (a crossing of the pin
                    // surface at the wall belongs to the layer next door).
                    let q = sub(p, l.center(idx));
                    excl_col.is_some_and(|e| e[0] == idx[0] && e[1] == idx[1] && e[2] != idx[2]) && hexlat::sd(or, l.pitch[0], [q[0], q[1]]) >= -self.tol
                };
                let carry_pass = |sc: &mut Scratch| -> Option<Hit> {
                    let Shape::Hex(or) = l.shape else { return None };
                    for idx in l.cands(p, dir, self.tol) {
                        if !l.in_range(idx) || same_column(idx) {
                            continue;
                        }
                        let lin = l.lin(idx);
                        if excl.and_then(|r| r.split_first()).is_some_and(|(lv, _)| lv.component.get() == comp && lv.instance.get() == lin) {
                            continue;
                        }
                        let q = sub(p, l.center(idx));
                        let n = norm(q);
                        // For each flat within `tol` of the point, the distance outside it plus
                        // the sideways drift over a run of one edge (`p / sqrt 3`, the longest run
                        // along a flat) stays within `tol`, however grazing the departure. A ray
                        // leaving at a vertex is not carried into it.
                        let edge = l.pitch[0] / 3.0_f64.sqrt();
                        if n <= 0.0 || hexlat::run_leaves(or, l.pitch[0], [q[0], q[1]], [dir[0], dir[1]], edge, self.tol) {
                            continue;
                        }
                        if let Loc::Hit(mut h) = self.locate_rec(l.fill[lin] as usize, q, scale(q, -1.0 / n), sc, None, false) {
                            // The leaf must know it is on the flat the ray runs along, or it may
                            // end the run at once through that plane (rounding in the direction).
                            if let (Body::Csg(a), Kind::Leaf(_)) = (&self.protos[h.proto].tracker, &self.comps[l.fill[lin] as usize].kind)
                                && let Some(f) = a.on_face_along(h.local.cell, p3(q), v3(dir))
                            {
                                h.local.on_face = Some(f);
                            }
                            h.base += self.cell_off_in_lattice(l, lin);
                            h.levels.insert(0, Level::new(ComponentId::new(comp), InstanceId::new(lin), idx));
                            return Some(h);
                        }
                    }
                    None
                };
                if carry && excl.is_some_and(|r| r.first().is_some_and(|lv| lv.component.get() == comp))
                    && let Some(h) = carry_pass(sc)
                {
                    return Loc::Hit(h);
                }
                for idx in l.cands(p, dir, self.tol) {
                    if !l.in_range(idx) || same_column(idx) {
                        continue;
                    }
                    let lin = l.lin(idx);
                    let ce = l.center(idx);
                    let q = sub(p, ce);
                    let sub_excl = excl.and_then(|r| r.split_first()).filter(|(lv, _)| lv.component.get() == comp && lv.instance.get() == lin).map(|(_, rest)| rest);
                    // On a z-wall of a hex stack with `dz` ~ 0 the leaf's tie on its z plane is
                    // hashed, so each layer would take or refuse the point by its own hash. The
                    // point is in both layers: rule on it with the direction nudged into this
                    // layer (the state's direction is not changed), so the first candidate in
                    // order takes it and the run along the wall stays in one layer.
                    let mut ld = dir;
                    if parallel_z && l.pitch[2] > 0.0 && (q[2].abs() - 0.5 * l.pitch[2]).abs() <= self.tol {
                        ld[2] = -q[2].signum() * 1e-8;
                    }
                    match self.locate_rec(l.fill[lin] as usize, q, ld, sc, sub_excl, carry) {
                        Loc::Miss => {}
                        Loc::Overlap => return Loc::Overlap,
                        Loc::Hit(mut h) => {
                            h.base += self.cell_off_in_lattice(l, lin);
                            h.levels.insert(0, Level::new(ComponentId::new(comp), InstanceId::new(lin), idx));
                            return Loc::Hit(h);
                        }
                    }
                }
                match carry_pass(sc) {
                    Some(h) => Loc::Hit(h),
                    None => Loc::Miss,
                }
            }
        }
    }

    fn global_located(&self, h: Hit) -> Located {
        let p = &self.protos[h.proto];
        let cell = CellId::new((h.base + u64::from(h.local.cell.0)) as usize);
        let region = RegionId(p.region_off + h.local.region.0);
        let on_face = h.local.on_face.map(|f| FaceId(p.face_off + f.0));
        Located::new(cell, region, on_face).with_levels(h.levels)
    }

    // ---- stepping -----------------------------------------------------------------------

    /// Overlap test of the chord `[0, t_end]` of the ray (global `o`, `d`) in the current
    /// placement against its partners. Two checks: the midpoint of the chord strictly inside a
    /// partner (identical or nested placements), and for a leaf partner the first entry of the
    /// ray into it strictly inside the chord (placements that cross without a vertex inside the
    /// other).
    fn probe_partners(&self, chk: &Check, o: [f64; 3], d: [f64; 3], t_end: f64, sc: &mut Scratch) -> bool {
        let Kind::Union { places, partners, .. } = &self.comps[chk.comp].kind else { return false };
        let (ou, du) = (chk.aff.pt(o), chk.aff.vec(d));
        let mid = madd(ou, du, 0.5 * t_end);
        for &j in &partners[chk.place] {
            let pj = &places[j as usize];
            let (oj, dj) = (pj.inv.pt(ou), pj.inv.vec(du));
            // Midpoint of the chord strictly inside the partner.
            if inside_box(&pj.bbox, mid, self.tol)
                && let Loc::Hit(h) = self.locate_rec(pj.child, pj.inv.pt(mid), pj.inv.vec(hashed_dir_arr(mid)), sc, None, false)
                && h.strict
            {
                return true;
            }
            // Entry of the ray into the partner inside the chord.
            let entry = match &self.comps[pj.child].kind {
                Kind::Leaf(pi) => self.protos[*pi].tracker.enter_from_outside(oj, dj, self.tol, t_end, sc),
                _ => slab(oj, dj, &self.comps[pj.child].bounds, 0.0, t_end).map(|(a, _)| a),
            };
            if let Some(t) = entry
                && t > 2.0 * self.tol
                && t < t_end - 2.0 * self.tol
            {
                // A partner entered strictly inside the chord of the current cell. For a
                // non-leaf partner the box entry is only a candidate: confirm by locating just
                // inside the box.
                if matches!(&self.comps[pj.child].kind, Kind::Leaf(_)) {
                    return true;
                }
                let q = madd(ou, du, (t + 4.0 * self.tol).min(t_end));
                if let Loc::Hit(h) = self.locate_rec(pj.child, pj.inv.pt(q), pj.inv.vec(du), sc, None, false)
                    && h.strict
                {
                    return true;
                }
            }
        }
        false
    }

    fn lost_step(&self, st: &TrackState, cause: LostCause) -> Step {
        Step::new(0.0, st.cell, st.region, StepEnd::Lost(lost(st, cause)))
    }
}

/// Vertices of the outer boundary of component `c`, in its own frame (`comps` looks a compiled
/// component up, so the build can call it before the tracker exists).
fn boundary_vertices_of<'a>(comps: &dyn Fn(usize) -> Option<&'a Comp>, protos: &[Proto], c: usize, out: &mut Vec<[f64; 3]>) {
    let Some(comp) = comps(c) else { return };
match &comp.kind {
        // A CSG leaf has no vertices: overlaps with it are found by `probe_partners` and
        // `rt.check_overlaps`.
        Kind::Leaf(p) => {
            let Some(core) = protos[*p].tracker.mesh().map(|m| &m.core) else { return };
            let mut vs: BTreeSet<u32> = BTreeSet::new();
            for f in 0..core.n_faces() {
                if core.f_right[f] == crate::core::NONE {
                    vs.extend(core.face_verts(f).iter().copied());
                }
            }
            out.extend(vs.into_iter().map(|v| core.pos[v as usize]));
        }
        Kind::Union { places, .. } => {
            let mut tmp = Vec::new();
            for p in places {
                tmp.clear();
                boundary_vertices_of(comps, protos, p.child, &mut tmp);
                out.extend(tmp.iter().map(|&v| p.fwd.pt(v)));
            }
        }
        Kind::Lattice(l) => {
            let mut cache: Vec<(u32, Vec<[f64; 3]>)> = Vec::new();
            for e in 0..l.n() {
                let child = l.fill[e];
                let at = match cache.iter().position(|(c, _)| *c == child) {
                    Some(at) => at,
                    None => {
                        let mut v = Vec::new();
                        boundary_vertices_of(comps, protos, child as usize, &mut v);
                        cache.push((child, v));
                        cache.len() - 1
                    }
                };
                let ce = l.center(l.idx_of(e));
                out.extend(cache[at].1.iter().map(|v| [v[0] + ce[0], v[1] + ce[1], v[2] + ce[2]]));
            }
        }
    }
}

fn hashed_dir_arr(p: [f64; 3]) -> [f64; 3] {
    av3(hashed_direction(p3(p)))
}

fn to_u32_check(ro: u64, so: u64, fo: u64) -> Result<()> {
    if ro >= u64::from(u32::MAX) || so >= u64::from(u32::MAX) || fo >= u64::from(u32::MAX) {
        return Err(RtError::Unsupported("more than u32::MAX regions, surfaces or faces over all prototypes"));
    }
    Ok(())
}

fn comp_bytes(c: &Comp) -> usize {
    match &c.kind {
        Kind::Leaf(_) => 0,
        Kind::Union { places, off, partners } => places.len() * core::mem::size_of::<Place>() + off.len() * 8 + partners.iter().map(|p| p.len() * 4 + 24).sum::<usize>(),
        Kind::Lattice(l) => l.fill.len() * 4 + l.off.len() * 8,
    }
}

/// Children-first order of the components reachable from `root` (iterative; the graph is a DAG).
fn post_order(n: usize, root: usize, kids: &dyn Fn(usize) -> Vec<usize>) -> Vec<usize> {
    let mut state = vec![0u8; n];
    let mut out = Vec::new();
    let mut stack: Vec<(usize, Vec<usize>, usize)> = vec![(root, kids(root), 0)];
    state[root] = 1;
    while let Some(top) = stack.last_mut() {
        if let Some(&k) = top.1.get(top.2) {
            top.2 += 1;
            if state[k] == 0 {
                state[k] = 1;
                stack.push((k, kids(k), 0));
            }
        } else {
            out.push(top.0);
            state[top.0] = 2;
            stack.pop();
        }
    }
    out
}

fn compile(kind: Option<&ComponentKind>, ci: usize, done: &[Option<Comp>], protos: &[Proto], proto_of_input: &[usize], tol: f64) -> Result<Comp> {
    let child = |c: usize| done[c].as_ref().ok_or_else(|| issue("world.internal", format!("component {c} compiled out of order")));
    let cap = |x: u64| if x > MAX_CELLS { Err(RtError::Unsupported("more than u32::MAX - 2 cells over all instances")) } else { Ok(x) };
    match kind {
        Some(ComponentKind::Leaf { input }) => {
            let pi = proto_of_input[*input];
            let t = &protos[pi].tracker;
            let b = t.info().bounds;
            Ok(Comp { kind: Kind::Leaf(pi), bounds: [b.lo.x, b.lo.y, b.lo.z, b.hi.x, b.hi.y, b.hi.z], cells: t.n_cells() as u64, inst: 1 })
        }
        Some(ComponentKind::Union { placements }) => {
            let mut places = Vec::with_capacity(placements.len());
            let mut off = Vec::with_capacity(placements.len());
            let (mut cells, mut inst) = (0u64, 0u64);
            let mut bounds = EMPTY_BOX;
            for (i, p) in placements.iter().enumerate() {
                if !is_rigid(&p.transform) {
                    return Err(issue("world.nonrigid", format!("component {ci}: placement {i} is not a rigid transform (rotation or reflection plus translation)")));
                }
                let inv = p.transform.inverse().ok_or_else(|| issue("world.singular_transform", format!("component {ci}: singular placement {i}")))?;
                let ch = child(p.child.get())?;
                let fwd = Aff::from_transform(&p.transform);
                let bbox = if box_empty(&ch.bounds) { EMPTY_BOX } else { box_of_points(corners(&ch.bounds).into_iter().map(|c| fwd.pt(c))) };
                bounds = union(&bounds, &bbox);
                off.push(cells);
                cells = cap(cells + ch.cells)?;
                inst += ch.inst;
                places.push(Place { child: p.child.get(), fwd, inv: Aff::from_transform(&inv), bbox });
            }
            let n = places.len();
            let mut partners = vec![Vec::new(); n];
            // Sort-and-sweep over the boxes along x (ties by index).
            let mut idx: Vec<usize> = (0..n).filter(|&i| !box_empty(&places[i].bbox)).collect();
            idx.sort_by(|&a, &b| places[a].bbox[0].total_cmp(&places[b].bbox[0]).then(a.cmp(&b)));
            let mut active: Vec<usize> = Vec::new();
            for &i in &idx {
                active.retain(|&a| places[a].bbox[3] + tol >= places[i].bbox[0]);
                for &a in &active {
                    if boxes_touch(&places[a].bbox, &places[i].bbox, tol) {
                        partners[a].push(i as u32);
                        partners[i].push(a as u32);
                    }
                }
                active.push(i);
            }
            for p in &mut partners {
                p.sort_unstable();
            }
            Ok(Comp { kind: Kind::Union { places, off, partners }, bounds, cells, inst })
        }
        Some(ComponentKind::Lattice(l)) => {
            let (shape, pitch) = match l.kind {
                LatticeKind::Rect { pitch } => (Shape::Rect, pitch),
                LatticeKind::Hex { pitch, z_pitch } => (Shape::Hex(Orient::XRows), [pitch, pitch, if l.dims[2] > 1 { z_pitch } else { 0.0 }]),
                LatticeKind::HexYRows { pitch, z_pitch } => (Shape::Hex(Orient::YRows), [pitch, pitch, if l.dims[2] > 1 { z_pitch } else { 0.0 }]),
                _ => return Err(RtError::Unsupported("lattice kind")),
            };
            if l.outer.is_some() {
                return Err(RtError::Unsupported("lattice outer fill"));
            }
            let lat0 = Lat { shape, origin: a3(l.origin), pitch, dims: l.dims, fill: l.fill.iter().map(|c| c.0).collect(), uniform: None, off: Vec::new() };
            let uniform = lat0.fill.first().copied().filter(|f| lat0.fill.iter().all(|x| x == f));
            // Fit: each distinct fill lies inside its element (x, y; z with more than one layer).
            let mut distinct: Vec<u32> = lat0.fill.clone();
            distinct.sort_unstable();
            distinct.dedup();
            for &f in &distinct {
                let fc = child(f as usize)?;
                let b = fc.bounds;
                if box_empty(&b) {
                    continue;
                }
                let fit = |what: String| issue("world.lattice_fit", format!("component {ci}: fill component {f} extends beyond its lattice element {what}"));
                match shape {
                    Shape::Rect => {
                        let axes = if l.dims[2] > 1 { 3 } else { 2 };
                        for a in 0..axes {
                            let h = 0.5 * pitch[a];
                            if b[a] < -h - tol || b[3 + a] > h + tol {
                                return Err(fit(format!("along axis {a} ([{:e}, {:e}] vs half pitch {h:e})", b[a], b[3 + a])));
                            }
                        }
                    }
                    Shape::Hex(o) => hex_fit(o, pitch, l.dims[2] > 1, &b, fc, f as usize, protos, done, tol).map_err(fit)?,
                }
            }
            let mut lat = lat0;
            lat.uniform = uniform;
            let n = lat.n();
            let (mut cells, mut inst) = (0u64, 0u64);
            let mut bounds = EMPTY_BOX;
            if let Some(u) = uniform {
                let ch = child(u as usize)?;
                cells = cap(ch.cells.checked_mul(n as u64).ok_or(RtError::Unsupported("cell count overflow"))?)?;
                inst = ch.inst * n as u64;
                if !box_empty(&ch.bounds) {
                    let (c0, c1) = (lat.center(lat.idx_of(0)), lat.center(lat.idx_of(n - 1)));
                    bounds = [ch.bounds[0] + c0[0], ch.bounds[1] + c0[1], ch.bounds[2] + c0[2], ch.bounds[3] + c1[0], ch.bounds[4] + c1[1], ch.bounds[5] + c1[2]];
                }
            } else {
                for e in 0..n {
                    let ch = child(lat.fill[e] as usize)?;
                    lat.off.push(cells);
                    cells = cap(cells + ch.cells)?;
                    inst += ch.inst;
                    if !box_empty(&ch.bounds) {
                        let c = lat.center(lat.idx_of(e));
                        bounds = union(&bounds, &[ch.bounds[0] + c[0], ch.bounds[1] + c[1], ch.bounds[2] + c[2], ch.bounds[3] + c[0], ch.bounds[4] + c[1], ch.bounds[5] + c[2]]);
                    }
                }
            }
            Ok(Comp { kind: Kind::Lattice(lat), bounds, cells, inst })
        }
        _ => Err(issue("world.component", format!("component {ci} is missing or of an unsupported kind"))),
    }
}

/// The fit of a fill (bounds `b`, in the element frame) inside its hexagonal prism; `Err` names
/// the violation. Three tests of growing strength, all within `tol`: the box lies in the box
/// circumscribing the hexagon (and in half a z pitch for a stack); every boundary vertex of a
/// mesh fill lies in the hexagon; a leaf fill has no cell just outside the six flats or the six
/// corners (a probe, so a CSG fill that pokes out by less than the probe offset elsewhere is not
/// seen: documented in DESIGN.md).
#[allow(clippy::too_many_arguments)]
fn hex_fit(o: Orient, pitch: [f64; 3], stacked: bool, b: &Box6, fill: &Comp, f: usize, protos: &[Proto], done: &[Option<Comp>], tol: f64) -> core::result::Result<(), String> {
    let hb = hexlat::half_box(o, pitch[0]);
    for a in 0..2 {
        if b[a] < -hb[a] - tol || b[3 + a] > hb[a] + tol {
            return Err(format!("along axis {a} ([{:e}, {:e}] vs hexagon half extent {:e})", b[a], b[3 + a], hb[a]));
        }
    }
    if stacked {
        let h = 0.5 * pitch[2];
        if b[2] < -h - tol || b[5] > h + tol {
            return Err(format!("along axis 2 ([{:e}, {:e}] vs half z pitch {h:e})", b[2], b[5]));
        }
    }
    let mut vs = Vec::new();
    boundary_vertices_of(&|i| done[i].as_ref(), protos, f, &mut vs);
    if let Some(v) = vs.iter().find(|v| hexlat::sd(o, pitch[0], [v[0], v[1]]) > tol) {
        return Err(format!("(boundary vertex ({:e}, {:e}) is outside the hexagon of pitch {:e})", v[0], v[1], pitch[0]));
    }
    if let Kind::Leaf(pi) = &fill.kind {
        use core::f64::consts::{FRAC_PI_3, FRAC_PI_6};
        let z = 0.5 * (b[2] + b[5]);
        let (apothem, circum) = (0.5 * pitch[0], pitch[0] / 3.0_f64.sqrt());
        let out = 16.0 * tol;
        // Flat normals sit at the base angle plus k 60 degrees, corners half a step between.
        let base = if o == Orient::XRows { 0.0 } else { FRAC_PI_6 };
        let mut sc = Scratch::new();
        for k in 0..6 {
            for (rad, ang) in [(apothem + out, base + k as f64 * FRAC_PI_3), (circum + out, base + (k as f64 + 0.5) * FRAC_PI_3)] {
                let pt = [rad * ang.cos(), rad * ang.sin(), z];
                let (loc, overlap) = protos[*pi].tracker.locate(pt, hashed_dir_arr(pt), None, &mut sc);
                if overlap || (!loc.cell.is_none() && loc.cell != CellId::COMPLEMENT) {
                    return Err(format!("(the fill has a cell outside the hexagon of pitch {:e}, at ({:e}, {:e}))", pitch[0], pt[0], pt[1]));
                }
            }
        }
    }
    Ok(())
}

// ---- Tracker ---------------------------------------------------------------------------

impl Tracker for WorldTracker {
    fn caps(&self) -> TrackerCaps {
        let csg = self.protos.iter().any(|p| matches!(p.tracker, Body::Csg(_)));
        let mut c = TrackerCaps::HIERARCHY;
        if csg {
            c = c.union(TrackerCaps::ANALYTIC);
        } else if self.info.faceted_faces == 0 {
            c = c.union(TrackerCaps::EXACT_PLANAR);
        }
        if self.hier.is_some() { c.union(TrackerCaps::MAJORANT) } else { c }
    }
    fn majorant_view(&self) -> Option<&dyn MajorantTracker> {
        self.hier.as_ref().map(|_| self as &dyn MajorantTracker)
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
        let (_, p, local) = self.resolve_cell(u64::from(c.0))?;
        let pr = &self.protos[p];
        pr.tracker.c_region(local as usize).map(|r| RegionId(r + pr.region_off))
    }
    fn cell_volume(&self, c: CellId) -> Option<f64> {
        let (_, p, local) = self.resolve_cell(u64::from(c.0))?;
        self.protos[p].tracker.c_volume(local as usize)
    }
    fn mesh(&self) -> Option<&dyn MeshRead> {
        None
    }
    fn export_cache(&self) -> Option<CacheImage> {
        None
    }
    fn surface_area(&self, s: SurfaceId) -> Result<Option<f64>> {
        let p = self.proto_of_surface(s.0).ok_or(RtError::UnknownId { kind: "surface", id: s.0 })?;
        self.protos[p].tracker.surface_area(SurfaceId(s.0 - self.protos[p].surf_off))
    }
    fn surface_faces(&self, s: SurfaceId) -> &[FaceId] {
        match (self.sf_off.get(s.get()), self.sf_off.get(s.get() + 1)) {
            (Some(&a), Some(&b)) => &self.sf[a as usize..b as usize],
            _ => &[],
        }
    }

    fn locate(&self, p: Point3, dir: Option<Vector3>, hint: Option<CellId>, scratch: &mut Scratch) -> Result<Located> {
        let pa = a3(p);
        if !finite3(pa) {
            return Err(RtError::NonFinite("position"));
        }
        let d = match dir {
            Some(d) => {
                let a = av3(d);
                if !finite3(a) {
                    return Err(RtError::NonFinite("direction"));
                }
                a
            }
            None => hashed_dir_arr(pa),
        };
        // Fast path: a hint whose instance holds the point strictly inside its body. Skipped when
        // any union has overlap candidates (the full descent runs the overlap test).
        if let Some(h) = hint
            && !self.any_partners
            && let Some((levels, pi, local)) = self.resolve_cell(u64::from(h.0))
            && let Ok(fr) = self.frame_of(&levels)
        {
            let pr = &self.protos[pi];
            let (loc, overlap) = pr.tracker.locate(fr.g2l.pt(pa), fr.g2l.vec(d), Some(CellId(local)), scratch);
            if !overlap && pr.tracker.strict(&loc) {
                return Ok(self.global_located(Hit { proto: pi, base: fr.base, local: loc, levels, strict: true }));
            }
        }
        Ok(match self.locate_rec(self.root, pa, d, scratch, None, false) {
            Loc::Hit(h) => self.global_located(h),
            Loc::Miss | Loc::Overlap => Located::new(CellId::NONE, RegionId::NONE, None),
        })
    }

    fn step(&self, st: &mut TrackState, max_dist: f64, sc: &mut Scratch) -> Result<Step> {
        if max_dist.is_nan() || max_dist < 0.0 {
            return Err(RtError::InvalidInput("max_dist must be >= 0 and not NaN"));
        }
        let (o, d) = (a3(st.pos), av3(st.dir));
        if !finite3(o) || !finite3(d) {
            return Err(RtError::NonFinite("state"));
        }
        if st.cell.is_none() {
            return Ok(self.lost_step(st, LostCause::NoCellContainsPoint));
        }
        let fr = self.frame_of(&st.levels)?;
        let pr = &self.protos[fr.proto];
        let ncell = pr.tracker.n_cells() as u64;
        let c = u64::from(st.cell.0);
        if c < fr.base || c >= fr.base + ncell {
            return Err(RtError::UnknownId { kind: "cell", id: st.cell.0 });
        }
        let local_cell = (c - fr.base) as u32;
        let local_region = pr.tracker.c_region(local_cell as usize).ok_or(RtError::UnknownId { kind: "cell", id: st.cell.0 })?;
        let region_before = RegionId(local_region + pr.region_off);
        let mut ls = TrackState::new(p3(fr.g2l.pt(o)), v3(fr.g2l.vec(d)));
        ls.cell = CellId(local_cell);
        ls.region = RegionId(local_region);
        ls.entered_via = st.entered_via.and_then(|f| f.0.checked_sub(pr.face_off).filter(|&l| (l as usize) < pr.tracker.n_faces()).map(FaceId));
        let s = pr.tracker.step(&mut ls, max_dist, sc)?;
        let gface = |f: FaceId| FaceId(f.0 + pr.face_off);
        let gsurf = |x: SurfaceId| if x.is_none() { x } else { SurfaceId(x.0 + pr.surf_off) };
        let new_pos = madd(o, d, s.dist);
        if let StepEnd::Lost(mut l) = s.end {
            l.pos = st.pos;
            l.dir = st.dir;
            l.last_cell = st.cell;
            l.last_face = st.entered_via;
            l.nearest_surface = l.nearest_surface.map(gsurf);
            return Ok(Step::new(0.0, st.cell, region_before, StepEnd::Lost(l)));
        }
        for chk in &fr.checks {
            if self.probe_partners(chk, o, d, s.dist, sc) {
                return Ok(self.lost_step(st, LostCause::Overlap));
            }
        }
        let cell_after = CellId::new((fr.base + u64::from(ls.cell.0)) as usize);
        let region_after = RegionId(ls.region.0 + pr.region_off);
        let face_after = ls.entered_via.map(gface);
        let (cell_before, dist) = (st.cell, s.dist);
        let mk = |end: StepEnd| Step::new(dist, cell_before, region_before, end);
        let commit = |st: &mut TrackState| {
            st.pos = p3(new_pos);
            st.cell = cell_after;
            st.region = region_after;
            st.entered_via = face_after;
        };
        match s.end {
            StepEnd::Limit => {
                commit(st);
                Ok(mk(StepEnd::Limit))
            }
            StepEnd::CellFace { face, to } => {
                commit(st);
                Ok(mk(StepEnd::CellFace { face: gface(face), to: CellId::new((fr.base + u64::from(to.0)) as usize) }))
            }
            StepEnd::RegionBoundary { surface, face, to_region, to_cell } => {
                commit(st);
                Ok(mk(StepEnd::RegionBoundary {
                    surface: gsurf(surface),
                    face: gface(face),
                    to_region: RegionId(to_region.0 + pr.region_off),
                    to_cell: CellId::new((fr.base + u64::from(to_cell.0)) as usize),
                }))
            }
            StepEnd::Boundary { surface, face, .. } => {
                // The leaf's outer face: relocate the exit point from the root.
                match self.locate_rec(self.root, new_pos, d, sc, Some(&st.levels[..]), s.dist <= self.tol) {
                    Loc::Overlap => Ok(self.lost_step(st, LostCause::Overlap)),
                    Loc::Hit(h) => {
                        let to = self.global_located(h);
                        st.pos = p3(new_pos);
                        st.cell = to.cell;
                        st.region = to.region;
                        st.entered_via = to.on_face;
                        st.levels = to.levels;
                        let end = if to.region == region_before {
                            StepEnd::SlotChange { to_cell: to.cell }
                        } else {
                            StepEnd::RegionBoundary { surface: gsurf(surface), face: gface(face), to_region: to.region, to_cell: to.cell }
                        };
                        Ok(mk(end))
                    }
                    Loc::Miss => {
                        commit(st);
                        if matches!(self.exit_action, FaceAction::Reflect) {
                            let n = match &pr.tracker {
                                Body::Mesh(m) => {
                                    let pl = m.core.f_plane[face.get()];
                                    fr.g2l.vec_back([pl[0], pl[1], pl[2]])
                                }
                                Body::Csg(_) => fr.g2l.vec_back(pr.tracker.face_normal(face, a3(ls.pos))?),
                            };
                            let r = sub(d, scale(n, 2.0 * dot(d, n)));
                            st.dir = v3(r);
                        }
                        Ok(mk(StepEnd::Boundary { surface: gsurf(surface), face: gface(face), action: self.exit_action }))
                    }
                }
            }
            // `StepEnd::SlotChange` and `Lost` never come from a prototype.
            _ => Ok(self.lost_step(st, LostCause::NoExitFace)),
        }
    }

    fn closest(&self, p: Point3, region: Option<RegionId>, scratch: &mut Scratch) -> Result<Option<Closest>> {
        let pa = a3(p);
        if !finite3(pa) {
            return Err(RtError::NonFinite("position"));
        }
        let filter = match region {
            Some(r) => {
                let pi = self.proto_of_region(r).ok_or(RtError::UnknownId { kind: "region", id: r.0 })?;
                Some((pi, RegionId(r.0 - self.protos[pi].region_off)))
            }
            None => None,
        };
        let mut best: Option<(f64, SurfaceId, [f64; 3])> = None;
        self.closest_rec(self.root, pa, &Aff::ID, filter, &mut best, scratch);
        Ok(best.map(|(d, s, q)| Closest::new(d, s, p3(q))))
    }

    fn face_normal(&self, f: FaceId, p: Point3) -> Result<Vector3> {
        let pa = a3(p);
        if !finite3(pa) {
            return Err(RtError::NonFinite("position"));
        }
        let pi = self.proto_of_face(f.0).ok_or(RtError::UnknownId { kind: "face", id: f.0 })?;
        let local = FaceId(f.0 - self.protos[pi].face_off);
        let aff = self.find_instance_with_face(self.root, pa, &Aff::ID, pi, local).ok_or(RtError::InvalidInput("the point is not on an instance of the face"))?;
        let q = aff.pt(pa);
        let n = self.protos[pi].tracker.face_normal(local, q)?;
        Ok(v3(aff.vec_back(n)))
    }

    fn periodic_image(&self, _f: FaceId) -> Result<Option<PeriodicImage>> {
        Ok(None)
    }

    fn resolve_boundary(&self, st: &mut TrackState, face: FaceId, kind: ResponseKind, u: [f64; 2]) -> Result<sm_rt_api::BoundaryAction> {
        let unit = |x: f64| (0.0..=1.0).contains(&x);
        let normal = |this: &Self, st: &TrackState| -> Result<[f64; 3]> {
            let fr = this.frame_of(&st.levels)?;
            let pr = &this.protos[fr.proto];
            let local = face.0.checked_sub(pr.face_off).filter(|&l| (l as usize) < pr.tracker.n_faces()).ok_or(RtError::UnknownId { kind: "face", id: face.0 })?;
            let n = pr.tracker.face_normal(FaceId(local), fr.g2l.pt(a3(st.pos)))?;
            Ok(fr.g2l.vec_back(n))
        };
        let reflect = |this: &Self, st: &mut TrackState| -> Result<BoundaryAction> {
            let n = normal(this, st)?;
            let dv = av3(st.dir);
            let r = sub(dv, scale(n, 2.0 * dot(dv, n)));
            let l = norm(r);
            st.dir = v3(scale(r, 1.0 / l));
            st.entered_via = Some(face);
            Ok(BoundaryAction::Continue)
        };
        match kind {
            ResponseKind::Vacuum => Ok(BoundaryAction::Terminate),
            ResponseKind::Reflective => reflect(self, st),
            ResponseKind::Albedo(a) => {
                if !kind.is_valid() {
                    return Err(RtError::BadAlbedo { value: a });
                }
                if !unit(u[0]) {
                    return Err(RtError::InvalidInput("random number u[0] must be finite and in [0, 1]"));
                }
                if u[0] < a || a >= 1.0 { reflect(self, st) } else { Ok(BoundaryAction::Terminate) }
            }
            ResponseKind::White => {
                if !unit(u[0]) || !unit(u[1]) {
                    return Err(RtError::InvalidInput("random numbers u must be finite and in [0, 1]"));
                }
                let n = normal(self, st)?;
                let n_in = [-n[0], -n[1], -n[2]];
                let axis = if n_in[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
                let t1 = crate::vec3::cross(n_in, axis);
                let t1 = scale(t1, 1.0 / norm(t1));
                let t2 = crate::vec3::cross(n_in, t1);
                let (s, c) = (u[0].sqrt(), (1.0 - u[0]).sqrt());
                let phi = 2.0 * core::f64::consts::PI * u[1];
                let (sp, cp) = crate::vec3::sin_cos(phi);
                let dd = [
                    t1[0] * (s * cp) + t2[0] * (s * sp) + n_in[0] * c,
                    t1[1] * (s * cp) + t2[1] * (s * sp) + n_in[1] * c,
                    t1[2] * (s * cp) + t2[2] * (s * sp) + n_in[2] * c,
                ];
                st.dir = v3(scale(dd, 1.0 / norm(dd)));
                st.entered_via = Some(face);
                Ok(BoundaryAction::Continue)
            }
            _ => Err(RtError::Unsupported("response kind")),
        }
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
            _ => Err(RtError::Unsupported("tally spec on a hierarchical world")),
        }
    }
}

impl WorldTracker {
    fn closest_rec(&self, comp: usize, p: [f64; 3], g2l: &Aff, filter: Option<(usize, RegionId)>, best: &mut Option<(f64, SurfaceId, [f64; 3])>, sc: &mut Scratch) {
        let c = &self.comps[comp];
        if box_empty(&c.bounds) {
            return;
        }
        if let Some((bd, _, _)) = best
            && box_dist2(&c.bounds, p) >= *bd * *bd
        {
            return;
        }
        match &c.kind {
            Kind::Leaf(pi) => {
                let local_region = match filter {
                    Some((fp, r)) if fp == *pi => Some(r),
                    Some(_) => return,
                    None => None,
                };
                let pr = &self.protos[*pi];
                if let Some(cl) = pr.tracker.closest_inner(p, local_region, sc) {
                    let better = best.is_none_or(|(bd, _, _)| cl.dist < bd);
                    if better {
                        *best = Some((cl.dist, SurfaceId(cl.surface.0 + pr.surf_off), g2l.pt_back(a3(cl.point))));
                    }
                }
            }
            Kind::Union { places, .. } => {
                for pl in places {
                    self.closest_rec(pl.child, pl.inv.pt(p), &g2l.then(&pl.inv), filter, best, sc);
                }
            }
            Kind::Lattice(l) => {
                for e in 0..l.n() {
                    let ce = l.center(l.idx_of(e));
                    self.closest_rec(l.fill[e] as usize, sub(p, ce), &g2l.then(&Aff::shift_by(ce)), filter, best, sc);
                }
            }
        }
    }

    /// Global-to-leaf map of an instance of prototype `target` whose face `f` passes within a few
    /// tolerances of the global point `p`.
    fn find_instance_with_face(&self, comp: usize, p: [f64; 3], g2l: &Aff, target: usize, f: FaceId) -> Option<Aff> {
        let c = &self.comps[comp];
        let pad = 8.0 * self.tol;
        if box_empty(&c.bounds) {
            return None;
        }
        match &c.kind {
            Kind::Leaf(pi) => {
                if *pi != target || !inside_box(&c.bounds, p, pad) {
                    return None;
                }
                let d2 = self.protos[*pi].tracker.face_dist2(f.get(), p);
                (d2 <= pad * pad).then_some(*g2l)
            }
            Kind::Union { places, .. } => places.iter().filter(|pl| inside_box(&pl.bbox, p, pad)).find_map(|pl| self.find_instance_with_face(pl.child, pl.inv.pt(p), &g2l.then(&pl.inv), target, f)),
            Kind::Lattice(l) => {
                for idx in l.cands(p, [0.0; 3], pad) {
                    if !l.in_range(idx) {
                        continue;
                    }
                    let ce = l.center(idx);
                    if let Some(a) = self.find_instance_with_face(l.fill[l.lin(idx)] as usize, sub(p, ce), &g2l.then(&Aff::shift_by(ce)), target, f) {
                        return Some(a);
                    }
                }
                None
            }
        }
    }
}

// ---- majorant nodes and delta flights ---------------------------------------------------

const NO_NODE: u32 = u32::MAX;

/// A lattice or union component as a node of the prototype tree.
struct CompNode {
    parent: u32,
    bounds: Box6,
    set: u32,
    cells: u32,
}

/// The node tables of a world with `PartitionOptions::enabled`.
///
/// `NodeId`s index the prototype tree: first one node per reachable union or lattice
/// component (parents before children), then the partition nodes of each prototype leaf
/// (`proto_off`). Instances have no nodes: every element filled with one universe shares its
/// subtree, so the majorant slice does not grow with the element count.
pub(crate) struct WorldHier {
    comp_nodes: Vec<CompNode>,
    proto_off: Vec<u32>,
    proto_parent: Vec<u32>,
    /// Per prototype: its set ids to world set ids.
    set_map: Vec<Vec<u32>>,
    sets: MaterialSets,
    region_set: Vec<u32>,
    root_node: u32,
    n_nodes: u32,
}

impl WorldHier {
    fn bytes(&self) -> usize {
        self.comp_nodes.len() * core::mem::size_of::<CompNode>() + self.proto_off.len() * 8 + self.set_map.iter().map(|m| m.len() * 4).sum::<usize>() + self.region_set.len() * 4
    }
}

fn comp_kids(c: &Comp) -> Vec<usize> {
    let mut k: Vec<usize> = match &c.kind {
        Kind::Leaf(_) => Vec::new(),
        Kind::Union { places, .. } => places.iter().map(|p| p.child).collect(),
        Kind::Lattice(l) => l.fill.iter().map(|&f| f as usize).collect(),
    };
    k.sort_unstable();
    k.dedup();
    k
}

fn build_hier(protos: &[Proto], comps: &[Comp], root: usize) -> Result<WorldHier> {
    let n = comps.len();
    let order = post_order(n, root, &|c| comp_kids(&comps[c]));
    let mut comp_node = vec![NO_NODE; n];
    let mut ncomp = 0u32;
    for &c in order.iter().rev() {
        if !matches!(comps[c].kind, Kind::Leaf(_)) {
            comp_node[c] = ncomp;
            ncomp += 1;
        }
    }
    let no_part = || RtError::Unsupported("a prototype of this world has no partition");
    let mut proto_off = Vec::with_capacity(protos.len());
    let mut acc = u64::from(ncomp);
    for p in protos {
        proto_off.push(u32::try_from(acc).map_err(|_| RtError::Unsupported("more than u32::MAX majorant nodes"))?);
        acc += p.tracker.partition().ok_or_else(no_part)?.nodes.len() as u64;
    }
    let n_nodes = u32::try_from(acc).map_err(|_| RtError::Unsupported("more than u32::MAX majorant nodes"))?;
    // First parent in parents-first order.
    let mut comp_parent = vec![NO_NODE; n];
    let mut proto_parent = vec![NO_NODE; protos.len()];
    for &c in order.iter().rev() {
        let me = comp_node[c];
        if me == NO_NODE {
            continue;
        }
        for k in comp_kids(&comps[c]) {
            match comps[k].kind {
                Kind::Leaf(p) => {
                    if proto_parent[p] == NO_NODE {
                        proto_parent[p] = me;
                    }
                }
                _ => {
                    if comp_parent[k] == NO_NODE {
                        comp_parent[k] = me;
                    }
                }
            }
        }
    }
    // Sets: each prototype's sets in id order, then one singleton per region, then the
    // components (children first).
    let mut sets = MaterialSets::new();
    let mut set_map = Vec::with_capacity(protos.len());
    for p in protos {
        let part = p.tracker.partition().ok_or_else(no_part)?;
        let m: Vec<u32> = (0..part.sets.len()).map(|s| sets.intern(part.sets.get(MaterialSetId::new(s)).unwrap_or(&[]).iter().map(|r| RegionId(r.0 + p.region_off))).0).collect();
        set_map.push(m);
    }
    let nreg: usize = protos.iter().map(|p| p.tracker.regions().0.len()).sum();
    let region_set: Vec<u32> = (0..nreg).map(|r| sets.intern([RegionId::new(r)]).0).collect();
    let mut csets: Vec<Vec<u32>> = vec![Vec::new(); n];
    let mut comp_nodes: Vec<CompNode> = (0..ncomp).map(|_| CompNode { parent: NO_NODE, bounds: EMPTY_BOX, set: 0, cells: 0 }).collect();
    for &c in &order {
        match &comps[c].kind {
            Kind::Leaf(p) => {
                let part = protos[*p].tracker.partition().ok_or_else(no_part)?;
                csets[c] = part.sets.get(MaterialSetId(part.nodes[0].set)).unwrap_or(&[]).iter().map(|r| r.0 + protos[*p].region_off).collect();
            }
            _ => {
                let mut v: Vec<u32> = comp_kids(&comps[c]).into_iter().flat_map(|k| csets[k].clone()).collect();
                v.sort_unstable();
                v.dedup();
                let id = sets.intern(v.iter().map(|&r| RegionId(r))).0;
                let nd = &mut comp_nodes[comp_node[c] as usize];
                nd.parent = comp_parent[c];
                nd.bounds = comps[c].bounds;
                nd.set = id;
                nd.cells = u32::try_from(comps[c].cells).unwrap_or(u32::MAX);
                csets[c] = v;
            }
        }
    }
    let root_node = match comps[root].kind {
        Kind::Leaf(p) => proto_off[p],
        _ => comp_node[root],
    };
    Ok(WorldHier { comp_nodes, proto_off, proto_parent, set_map, sets, region_set, root_node, n_nodes })
}

impl HierarchyView for WorldTracker {
    fn node_count(&self) -> usize {
        self.hier.as_ref().map_or(0, |h| h.n_nodes as usize)
    }
    fn root(&self) -> NodeId {
        self.hier.as_ref().map_or(NodeId::NONE, |h| NodeId(h.root_node))
    }
    fn node(&self, n: NodeId) -> Option<NodeInfo> {
        let h = self.hier.as_ref()?;
        let i = n.0;
        let up = |p: u32| if p == NO_NODE { NodeId::NONE } else { NodeId(p) };
        if let Some(c) = h.comp_nodes.get(i as usize) {
            let b = if box_empty(&c.bounds) { [0.0; 6] } else { c.bounds };
            let bounds = Aabb { lo: Point3::new(b[0], b[1], b[2]), hi: Point3::new(b[3], b[4], b[5]) };
            // Unions and lattices both hold prototypes in their own frames, which do not tile
            // the component's box: `NodeKind::Lattice` is the kind whose children are not tiled.
            return Some(NodeInfo::new(n, up(c.parent), bounds, NodeKind::Lattice, MaterialSetId(c.set), c.cells));
        }
        let p = h.proto_off.partition_point(|&o| o <= i).checked_sub(1)?;
        let part = self.protos[p].tracker.partition()?;
        let local = i - h.proto_off[p];
        let info = HierarchyView::node(part, NodeId(local))?;
        let rec = part.nodes.get(local as usize)?;
        let parent = if rec.parent == NO_NODE { up(h.proto_parent[p]) } else { NodeId(rec.parent + h.proto_off[p]) };
        Some(NodeInfo::new(n, parent, info.bounds, info.kind, MaterialSetId(*h.set_map[p].get(info.set.get())?), info.cells))
    }
    fn node_of_cell(&self, c: CellId) -> Option<NodeId> {
        let h = self.hier.as_ref()?;
        let (_, p, local) = self.resolve_cell(u64::from(c.0))?;
        let part = self.protos[p].tracker.partition()?;
        let nn = part.cell_node.get(local as usize).copied().filter(|&x| x != crate::core::NONE)?;
        Some(NodeId(nn + h.proto_off[p]))
    }
    fn set_count(&self) -> usize {
        self.hier.as_ref().map_or(0, |h| h.sets.len())
    }
    fn set_regions(&self, s: MaterialSetId) -> Option<&[RegionId]> {
        self.hier.as_ref()?.sets.get(s)
    }
    fn region_set(&self, r: RegionId) -> Option<MaterialSetId> {
        self.hier.as_ref()?.region_set.get(r.get()).copied().map(MaterialSetId)
    }
}

/// Perturb the global point `g` by a few ulps (never along the ray: a ray nearly parallel to the
/// wall would not move the coordinate at all) until its image under `g2l` is on the entering side
/// of the wall `wall` of local axis `ax` (`hi`: the high wall), so the next descent from the rounded
/// global position picks the node behind the wall, never the node just left.
fn past_wall(g2l: &Aff, g: [f64; 3], ax: usize, wall: f64, hi: bool) -> [f64; 3] {
    let ok = |q: [f64; 3]| {
        let l = g2l.pt(q)[ax];
        if hi { l >= wall } else { l <= wall }
    };
    if ok(g) {
        return g;
    }
    // The global coordinate that moves local axis `ax` the most, bumped toward the wall's far side.
    let row = g2l.m[ax];
    let j = (0..3).max_by(|&a, &b| row[a].abs().total_cmp(&row[b].abs())).unwrap_or(0);
    let toward = if (row[j] > 0.0) == hi { 1.0 } else { -1.0 };
    let ulp = f64::EPSILON * (1.0 + g[0].abs().max(g[1].abs()).max(g[2].abs()));
    let mut q = g;
    for k in 1..=64 {
        q[j] = g[j] + toward * ulp * f64::from(k);
        if ok(q) {
            return q;
        }
    }
    q
}

impl WorldTracker {
    /// Hash of the prototypes' majorant partitions in prototype order, if the world has them.
    /// Independent of the thread count; `structure_hash` does not include it.
    pub fn partition_hash(&self) -> Option<ContentHash> {
        self.hier.as_ref()?;
        let mut h = ContentHasher::new();
        for p in &self.protos {
            h.write_hash(p.tracker.mesh()?.partition_hash()?);
        }
        Some(h.finish())
    }

    /// Prototype index and local node of a partition node id.
    fn proto_node(&self, node: NodeId) -> Option<(usize, u32)> {
        let h = self.hier.as_ref()?;
        if (node.0 as usize) < h.comp_nodes.len() || node.0 >= h.n_nodes {
            return None;
        }
        let p = h.proto_off.partition_point(|&o| o <= node.0).checked_sub(1)?;
        Some((p, node.0 - h.proto_off[p]))
    }

    fn through_lattice(&self, levels: &[Level]) -> bool {
        levels.iter().any(|l| matches!(self.comps[l.component.get()].kind, Kind::Lattice(_)))
    }
}

impl MajorantTracker for WorldTracker {
    fn hierarchy(&self) -> &dyn HierarchyView {
        self
    }

    fn advance_segment(&self, st: &mut TrackState, q: &SegmentQuery<'_>, sc: &mut Scratch) -> Result<FlightSegment> {
        let Some(h) = &self.hier else { return Err(RtError::Unsupported("tracker was built without a partition")) };
        let (o, d) = (a3(st.pos), av3(st.dir));
        if !finite3(o) || !finite3(d) {
            return Err(RtError::NonFinite("state"));
        }
        let (first, region, face_in) = (st.cell, st.region, st.entered_via);
        let set_of = |n: NodeId| self.node(n).map_or(MaterialSetId(0), |i| i.set);
        if first.is_none() || first == CellId::COMPLEMENT || first.get() >= self.info.cells {
            // Nothing located: a surface segment from `step`, which reports the loss.
            let node = NodeId(h.root_node);
            let sigma_m = q.input.majorant(node)?;
            let s = self.step(st, q.max_dist, sc)?;
            return Ok(FlightSegment::new(TrackMode::Surface, node, set_of(node), s.cell, s.region, s.dist, sigma_m).with_faces(face_in, end_face(&s.end)).with_resolved(true).with_end(s.end));
        }
        let fr = self.frame_of(&st.levels)?;
        let pr = &self.protos[fr.proto];
        let Some((mt, part)) = pr.tracker.mesh().and_then(|m| m.partition().map(|p| (m, p))) else { return Err(RtError::Unsupported("tracker was built without a partition")) };
        let c = u64::from(first.0);
        if c < fr.base || c >= fr.base + pr.tracker.n_cells() as u64 {
            return Err(RtError::UnknownId { kind: "cell", id: first.0 });
        }
        let (ol, dl) = (fr.g2l.pt(o), fr.g2l.vec(d));
        let mut visits = 0;
        let (_, lnode, node_box) = part.descend(0, part.root_box, ol, dl, &mut visits);
        sc.counters.node_visits += visits;
        let rec = &part.nodes[lnode as usize];
        let node_id = NodeId(h.proto_off[fr.proto] + lnode);
        // The one majorant entry this call reads, before the state moves.
        let sigma_m = q.input.majorant(node_id)?;
        let mode = choose_mode(q.policy, node_id, &q.input);
        let set = MaterialSetId(h.set_map[fr.proto][rec.set as usize]);
        let saved = st.clone();
        match mode {
            TrackMode::Surface if q.scale == SegmentScale::Cell => {
                let s = self.step(st, q.max_dist, sc)?;
                Ok(FlightSegment::new(mode, node_id, set, s.cell, s.region, s.dist, sigma_m).with_faces(face_in, end_face(&s.end)).with_resolved(true).with_end(s.end))
            }
            TrackMode::Surface => {
                let r = self.fire(st, q.max_dist, self.info.max_events, sc)?;
                let (mut end, mut total) = (r.end, r.dist);
                if let StepEnd::Lost(_) = end {
                    st.clone_from(&saved);
                    total = 0.0;
                    end = r.end;
                }
                Ok(FlightSegment::new(mode, node_id, set, first, region, total, sigma_m).with_faces(face_in, end_face(&end)).with_resolved(false).with_end(end))
            }
            TrackMode::Delta => {
                let (t_wall, ax, hi) = box_exit(ol, dl, &node_box);
                let want_wall = t_wall < q.max_dist;
                let limit = if want_wall { t_wall } else { q.max_dist };
                let mut total = 0.0;
                let mut end;
                if rec.boundary {
                    // An outer face of the prototype may be inside: walk with `step` (events,
                    // neighbour instances and the world boundary are its), up to the wall.
                    let start_levels = st.levels.clone();
                    let mut iters = 0u32;
                    loop {
                        let s = self.step(st, (limit - total).max(0.0), sc)?;
                        total += s.dist;
                        end = s.end;
                        let more = match end {
                            StepEnd::CellFace { .. } => true,
                            StepEnd::RegionBoundary { to_cell, .. } => !to_cell.is_none(),
                            _ => false,
                        };
                        // Another instance: the segment ends there (levels are constant over a segment).
                        if !more || st.levels != start_levels {
                            break;
                        }
                        iters += 1;
                        if iters > self.info.max_events {
                            end = StepEnd::Lost(lost(st, LostCause::MaxStepsExceeded));
                            break;
                        }
                    }
                } else {
                    // No outer face of the prototype inside the node: straight to the wall or the limit.
                    total = limit;
                    st.pos = p3(madd(o, d, limit));
                    end = StepEnd::Limit;
                }
                if let StepEnd::Lost(_) = end {
                    st.clone_from(&saved);
                    total = 0.0;
                } else if matches!(end, StepEnd::Limit) && (want_wall || !rec.boundary) {
                    let mut pl = fr.g2l.pt(a3(st.pos));
                    if want_wall {
                        let wall = if hi { node_box[3 + ax] } else { node_box[ax] };
                        pl[ax] = wall;
                        st.pos = p3(past_wall(&fr.g2l, fr.g2l.pt_back(pl), ax, wall, hi));
                    }
                    let loc = mt.locate_from_root(part, pl, dl, sc);
                    if loc.cell.is_none() || loc.cell == CellId::COMPLEMENT {
                        st.cell = CellId::NONE;
                        st.region = RegionId::NONE;
                        st.entered_via = None;
                        end = StepEnd::RegionBoundary { surface: SurfaceId::NONE, face: FaceId::NONE, to_region: RegionId::NONE, to_cell: CellId::NONE };
                    } else {
                        st.cell = CellId::new((fr.base + u64::from(loc.cell.0)) as usize);
                        st.region = RegionId(loc.region.0 + pr.region_off);
                        st.entered_via = loc.on_face.map(|f| FaceId(f.0 + pr.face_off));
                        end = if want_wall { StepEnd::SlotChange { to_cell: st.cell } } else { StepEnd::Limit };
                    }
                }
                Ok(FlightSegment::new(mode, node_id, set, first, region, total, sigma_m).with_faces(face_in, end_face(&end)).with_resolved(false).with_end(end))
            }
        }
    }

    fn locate_in_node(&self, node: NodeId, levels: &[Level], p: Point3, dir: Option<Vector3>, sc: &mut Scratch) -> Result<Located> {
        let Some((pi, local)) = self.proto_node(node) else {
            return Err(if self.hier.is_none() { RtError::Unsupported("tracker was built without a partition") } else { RtError::UnknownId { kind: "node", id: node.0 } });
        };
        let fr = self.frame_of(levels)?;
        if fr.proto != pi {
            return Err(RtError::InvalidInput("the levels do not lead to the prototype of the node"));
        }
        let pa = a3(p);
        if !finite3(pa) {
            return Err(RtError::NonFinite("position"));
        }
        let da = match dir {
            Some(v) => {
                let a = av3(v);
                if !finite3(a) {
                    return Err(RtError::NonFinite("direction"));
                }
                a
            }
            None => [0.0; 3],
        };
        let pr = &self.protos[pi];
        let (mt, part) = pr.tracker.mesh().and_then(|m| m.partition().map(|p| (m, p))).ok_or(RtError::Unsupported("tracker was built without a partition"))?;
        let rec = &part.nodes[local as usize];
        let (q, dl) = (fr.g2l.pt(pa), fr.g2l.vec(da));
        let tol = self.tol;
        let inside = (0..3).all(|k| q[k] >= rec.bounds[k] - tol && q[k] <= rec.bounds[3 + k] + tol);
        let (start, bx) = if inside { (rec.kd as usize, rec.bounds) } else { (0, part.root_box) };
        let mut visits = 0;
        let (leaf, _, _) = part.descend(start, bx, q, dl, &mut visits);
        sc.counters.node_visits += visits;
        // The owning element is resolved from the levels by index arithmetic.
        if self.through_lattice(levels) {
            sc.counters.lattice_index_locates += 1;
        }
        let lv: Levels = levels.iter().cloned().collect();
        let loc = mt.locate_leaf(part, leaf, q, dir.map(|_| dl), sc);
        if loc.cell.is_none() || loc.cell == CellId::COMPLEMENT {
            return Ok(Located::new(CellId::NONE, RegionId::NONE, None).with_levels(lv));
        }
        Ok(self.global_located(Hit { proto: pi, base: fr.base, local: loc, levels: lv, strict: true }))
    }

    fn safety_distance(&self, st: &TrackState, sc: &mut Scratch) -> Result<f64> {
        if st.cell.is_none() || st.cell == CellId::COMPLEMENT || st.cell.get() >= self.info.cells {
            return Ok(0.0);
        }
        let fr = self.frame_of(&st.levels)?;
        let pr = &self.protos[fr.proto];
        let c = u64::from(st.cell.0);
        if c < fr.base || c >= fr.base + pr.tracker.n_cells() as u64 {
            return Err(RtError::UnknownId { kind: "cell", id: st.cell.0 });
        }
        let Some(mt) = pr.tracker.mesh() else { return Err(RtError::Unsupported("safety distance of a CSG leaf")) };
        let mut ls = TrackState::new(p3(fr.g2l.pt(a3(st.pos))), v3(fr.g2l.vec(av3(st.dir))));
        ls.cell = CellId((c - fr.base) as u32);
        mt.safety_distance(&ls, sc)
    }
}
