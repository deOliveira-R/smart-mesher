//! The [`Tracker`] trait: the one query surface.
//!
//! Required methods are the primitives (`locate`, `step`, `closest`, tables, face geometry).
//! Everything a transport code calls (`fire`, `trace`, `trace_bundle`, `resolve_boundary`,
//! `bin_map`, `distance_to_boundary`) is a provided method built on `step`, so every implementor
//! gets identical event semantics and the conformance tests cover one code path.
//!
//! # Mapping from DAGMC
//! | DAGMC | here |
//! |---|---|
//! | `point_in_volume(vol, xyz, uvw, history)` | [`crate::compat::point_in_volume`] / [`Tracker::locate`] (direction-aware on a face) |
//! | `find_volume` | [`Tracker::locate`] |
//! | `ray_fire(vol, xyz, uvw, &surf, &dist, history, limit)` | [`crate::compat::ray_fire`] / [`Tracker::fire`] |
//! | `RayHistory` | `TrackState::entered_via` (exact adjacency makes a history unnecessary) |
//! | `next_vol(surf, vol)` | [`crate::compat::next_vol`] / [`SurfaceTable::next_region`] |
//! | `surface_sense` | [`crate::compat::surface_sense`] / [`SurfaceTable::sense`] |
//! | `closest_to_location` | [`Tracker::closest`] |
//! | `get_angle` / surface normal | [`crate::compat::get_angle`] / [`Tracker::face_normal`] |
//! | `test_volume_boundary` | `locate` with `dir` |
//! | implicit complement | `CellId::COMPLEMENT`, [`RegionTable::implicit_complement`] |
//!
//! # Hierarchy
//! A lattice or instance wall is not a model face. Crossing it ends a step with
//! [`StepEnd::SlotChange`] and updates `TrackState::levels`; `fire` skips these (and
//! `CellFace`) because the region does not change. `path_of` links an instance back to the
//! model's `InstancePath` for diagnostics.

use crate::boundary::ResponseKind;
use crate::cache::{CacheImage, CacheKey, ContentHash, ContentHasher};
use crate::error::{LostCause, LostInfo, Result, RtError};
use crate::ids::{CellId, FaceId, InstanceId, RegionId};
use crate::input::Precision;
use crate::state::{
    BoundaryHit, Closest, Located, Peek, RegionTable, Scratch, Step, StepEnd, SurfaceTable, TrackState, TrackerCaps,
};
use crate::tally::{BinMap, FireResult, Flow, Segment, SegmentSink, TallySpec, TraceEnd, TraceOptions, TrackBundle, advance};
use sm_core::{Aabb, Point3, Transform, Vector3};
use sm_geom_api::{ModelTolerance, Ray};
use sm_mesh_api::MeshRead;
use sm_model_api::InstancePath;

/// Validate a direction: finite, with `|norm - 1| <= tol`. Pass `info().tol.angular`.
pub fn check_direction(d: Vector3, tol: f64) -> Result<()> {
    let n = d.norm();
    if !n.is_finite() || (n - 1.0).abs() > tol {
        return Err(RtError::BadDirection { norm: n });
    }
    Ok(())
}

/// The deterministic default direction for a point on a face when the caller gave none
/// (`locate(p, None, ..)`): a unit vector that is a pure function of the bit pattern of `p`
/// (`-0.0` and NaN folded as in [`ContentHasher`]). No RNG, no time, no thread state: the same
/// point gives the same cell on every machine and run. This replaces DAGMC's random `uvw`.
/// Uniform on the sphere (Archimedes: uniform `z`, uniform azimuth).
pub fn hashed_direction(p: Point3) -> Vector3 {
    let mut h = ContentHasher::new();
    h.write_f64(p.x);
    h.write_f64(p.y);
    h.write_f64(p.z);
    let v = h.finish().0;
    let (hi, lo) = ((v >> 64) as u64, v as u64);
    let to_unit = |x: u64| (x >> 11) as f64 / (1u64 << 53) as f64;
    let z = 2.0 * to_unit(hi) - 1.0;
    let phi = 2.0 * core::f64::consts::PI * to_unit(lo);
    let s = ((1.0 - z) * (1.0 + z)).max(0.0).sqrt();
    Vector3::new(s * phi.cos(), s * phi.sin(), z)
}

/// Where a periodic face continues (a query-only view of a [`crate::FaceAction::Wrap`] face).
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct PeriodicImage {
    /// The matched face on the partner surface.
    pub face: FaceId,
    /// The cell behind it (inside the model).
    pub to_cell: CellId,
    /// Rigid map from this face's frame to the partner's (applied to position and direction).
    pub transform: Transform,
}

impl PeriodicImage {
    /// Construct (the type is `#[non_exhaustive]`).
    pub const fn new(face: FaceId, to_cell: CellId, transform: Transform) -> Self {
        Self { face, to_cell, transform }
    }
}

/// What `resolve_boundary` did.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum BoundaryAction {
    /// Particle leaves the problem (or was absorbed by an albedo roulette); state unchanged.
    Terminate,
    /// State was updated (reflected or re-emitted); keep tracking.
    Continue,
}

/// Build-time facts of a tracker: for provenance, cache checks and the benchmark claims
/// (memory per copy, exactness). Constructed by implementors with [`TrackerInfo::new`], then
/// the optional fields are set.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct TrackerInfo {
    /// Bounding box of the model.
    pub bounds: Aabb,
    /// Number of tracking cells (global, instance-numbered).
    pub cells: usize,
    /// Number of faces/facets.
    pub faces: usize,
    /// Distinct prototypes (shared acceleration structures). A flat tracker has 1.
    pub prototypes: usize,
    /// Total placed copies of prototypes. `copies / prototypes` is the sharing factor; with
    /// `memory_bytes` it makes "memory per copy" a measurable claim.
    pub copies: usize,
    /// Heap bytes held by the built structure (shared prototypes counted once).
    pub memory_bytes: usize,
    /// Faces tracked exactly (planar faces, analytic carriers).
    pub exact_faces: usize,
    /// Faces tracked through facets (NURBS or [`crate::FacetFallback::Always`]).
    pub faceted_faces: usize,
    /// Hash of the built structure (independent of thread count).
    pub structure_hash: ContentHash,
    /// Model tolerance the tracker was built with.
    pub tol: ModelTolerance,
    /// Traversal precision it was built with.
    pub precision: Precision,
    /// Event cap per `fire` / `trace` call (`BuildOptions::max_events`).
    pub max_events: u32,
    /// The cache key a rebuild of the same inputs has, when the builder recorded it.
    pub key: Option<CacheKey>,
}

impl TrackerInfo {
    /// Info with the identifying fields; counts default to a flat tracker (1 prototype, 1 copy).
    pub fn new(bounds: Aabb, cells: usize, structure_hash: ContentHash) -> Self {
        Self {
            bounds,
            cells,
            faces: 0,
            prototypes: 1,
            copies: 1,
            memory_bytes: 0,
            exact_faces: 0,
            faceted_faces: 0,
            structure_hash,
            tol: ModelTolerance::DEFAULT,
            precision: Precision::F64,
            max_events: 1_000_000,
            key: None,
        }
    }
}

/// The ray-tracing / particle-tracking query surface. Implementors are immutable after
/// construction and `Send + Sync`: any number of threads may query one tracker, each with its
/// own [`TrackState`] and [`Scratch`].
///
/// # Determinism contract
/// * Every query is a pure function of (tracker, arguments). No query reads time, thread id or
///   global state.
/// * Ties are broken by lowest id: when a ray exits through an edge or vertex shared by several
///   faces, the lowest `FaceId` among those with the minimal parameter wins; when a point lies on
///   several faces, the cell the direction points into wins, then the lowest `CellId`.
/// * With `dir = None` for a point on a face, the direction is [`hashed_direction`]`(p)`.
/// * Hits on exact (non-planar) faces are chosen by side consistency: a crossing is valid only
///   if its direction agrees with whether the particle is inside, so the `t ~ 0` self-hit is
///   rejected exactly, without an epsilon cutoff.
/// * Recovery (see [`crate::RecoveryPolicy`]) nudges by `tol.linear * 2^k` along the direction.
/// * `trace_bundle` returns rays in input order and its content hash is identical for any
///   thread count (fixed `block_size` partition, blocks joined in order).
/// * Building the same inputs gives the same [`Tracker::structure_hash`]; so does loading the
///   cache image of that build.
///
/// # Allocation
/// With a reused [`Scratch`] and hierarchy depth at most [`crate::LEVEL_INLINE`], `locate`,
/// `step`, `distance_to_boundary` and `closest` do not allocate after warm-up.
pub trait Tracker: Send + Sync {
    // ---- identity and tables -------------------------------------------------------------

    /// What this tracker supports.
    fn caps(&self) -> TrackerCaps;
    /// Build-time facts (bounds, counts, hash, tolerance).
    fn info(&self) -> &TrackerInfo;
    /// Regions.
    fn regions(&self) -> &RegionTable;
    /// Model surfaces.
    fn surfaces(&self) -> &SurfaceTable;
    /// Region of a cell.
    fn cell_region(&self, c: CellId) -> Option<RegionId>;
    /// Volume of a cell, when known (mesh-native: exact polyhedron volume from
    /// `sm_mesh_api::cell_volume`).
    fn cell_volume(&self, c: CellId) -> Option<f64>;
    /// The tracking mesh when there is one: the same object Sn/MOC and tallies index by `CellId`.
    fn mesh(&self) -> Option<&dyn MeshRead> {
        None
    }
    /// Model instance path of a hierarchy instance, for diagnostics (lost-particle and build
    /// reports link to model instances). `None` when the tracker is flat or has no model link.
    fn path_of(&self, _i: InstanceId) -> Option<&InstancePath> {
        None
    }
    /// Serialize the structure for the `.smrt` cache (`None` if not cacheable).
    fn export_cache(&self) -> Option<CacheImage> {
        None
    }

    // ---- primitives ----------------------------------------------------------------------

    /// Find the cell containing `p`. `dir` resolves a point on a face (the particle is in the
    /// cell it is moving into), as DAGMC's `point_in_volume(uvw)`; with `dir = None` on a face
    /// the direction is [`hashed_direction`]`(p)`. `hint` is a cell to start a visibility walk
    /// from (previous position of a particle); without it a BVH over cells is used. A point
    /// outside the model gives `CellId::NONE`, in the complement `CellId::COMPLEMENT`.
    fn locate(&self, p: Point3, dir: Option<Vector3>, hint: Option<CellId>, scratch: &mut Scratch) -> Result<Located>;

    /// Advance to the next cell face, slot change, region boundary, model boundary or the
    /// distance limit, whichever comes first, updating `st` (see [`StepEnd`] for the post-state
    /// of each case). `st.dir` must be unit. Deck boundaries are applied inside (see
    /// [`crate::boundary`]). Never allocates (see [`Scratch`]).
    fn step(&self, st: &mut TrackState, max_dist: f64, scratch: &mut Scratch) -> Result<Step>;

    /// Distance and location of the closest boundary of `region` (or of any region when `None`)
    /// from `p`. Used for safety distances and as the DAGMC `closest_to_location`.
    fn closest(&self, p: Point3, region: Option<RegionId>, scratch: &mut Scratch) -> Result<Option<Closest>>;

    /// Unit normal of a face at `p` (a point on the face), pointing out of its `front` region
    /// (outward at a boundary face). The point makes curved and analytic faces well defined.
    fn face_normal(&self, f: FaceId, p: Point3) -> Result<Vector3>;

    /// Where a `Wrap` face continues, as a query (no stepping).
    fn periodic_image(&self, _f: FaceId) -> Result<Option<PeriodicImage>> {
        Ok(None)
    }

    // ---- derived from info ---------------------------------------------------------------

    /// Bounding box of the model.
    fn bounds(&self) -> Aabb {
        self.info().bounds
    }
    /// Number of tracking cells (global, instance-numbered).
    fn cell_count(&self) -> usize {
        self.info().cells
    }
    /// Hash of the built structure.
    fn structure_hash(&self) -> ContentHash {
        self.info().structure_hash
    }

    // ---- provided ------------------------------------------------------------------------

    /// Locate a particle and build its state. Fails on a bad direction or a non-finite
    /// position; a point outside the model gives a state with `cell == CellId::NONE`.
    fn init_state(&self, pos: Point3, dir: Vector3, hint: Option<CellId>, scratch: &mut Scratch) -> Result<TrackState> {
        check_direction(dir, self.info().tol.angular)?;
        if !(pos.x.is_finite() && pos.y.is_finite() && pos.z.is_finite()) {
            return Err(RtError::NonFinite("position"));
        }
        let loc = self.locate(pos, Some(dir), hint, scratch)?;
        let mut st = TrackState::new(pos, dir);
        st.cell = loc.cell;
        st.region = loc.region;
        st.entered_via = loc.on_face;
        st.levels = loc.levels;
        Ok(st)
    }

    /// Peek: the next surface along the ray, without moving `st`. Equivalent to `fire` on a copy
    /// of the state (the copy is inline for depth up to [`crate::LEVEL_INLINE`], so no
    /// allocation). Deck boundaries on the way are *not* followed: a `Reflect` or `Wrap` surface
    /// is reported as the hit. Implementors may override with a cheaper query.
    fn distance_to_boundary(&self, st: &TrackState, limit: f64, scratch: &mut Scratch) -> Result<Peek> {
        let mut tmp = st.clone();
        let r = self.fire(&mut tmp, limit, self.info().max_events, scratch)?;
        Ok(match r.end {
            StepEnd::Limit => Peek::Limit,
            StepEnd::RegionBoundary { surface, face, to_region, .. } => {
                Peek::Hit(BoundaryHit::new(r.dist, surface, face, to_region, None))
            }
            StepEnd::Boundary { surface, face, action } => {
                Peek::Hit(BoundaryHit::new(r.dist, surface, face, RegionId::NONE, Some(action)))
            }
            StepEnd::Lost(l) => Peek::Lost(l),
            // `fire` never returns these; treat defensively as the limit.
            StepEnd::CellFace { .. } | StepEnd::SlotChange { .. } => Peek::Limit,
        })
    }

    /// DAGMC `ray_fire`: walk through cells of the current region until the region changes, a
    /// model boundary is reached, the limit is hit or the particle is lost. `end` is never
    /// `CellFace` or `SlotChange`: those are skipped (a table lookup each).
    fn fire(&self, st: &mut TrackState, dist_limit: f64, max_steps: u32, scratch: &mut Scratch) -> Result<FireResult> {
        let mut total = 0.0;
        for _ in 0..max_steps {
            let s = self.step(st, dist_limit - total, scratch)?;
            total += s.dist;
            if !matches!(s.end, StepEnd::CellFace { .. } | StepEnd::SlotChange { .. }) {
                return Ok(FireResult::new(total, s.end));
            }
        }
        Ok(FireResult::new(total, StepEnd::Lost(lost(st, LostCause::MaxStepsExceeded))))
    }

    /// Characteristics: emit one [`Segment`] per cell chord until the limit, a `Respond`
    /// boundary, loss or the sink stops. The same call serves a Monte Carlo track-length tally
    /// (sink scores `weight * length`, may `Stop` at a collision) and an MOC/Sn pass (sink
    /// records chords). Deck boundaries are followed unless `opts.stop_at_deck`. A zero-length
    /// `Limit` tail (the limit was reached exactly at an event) is not emitted as a segment.
    fn trace(&self, st: &mut TrackState, opts: &TraceOptions, sink: &mut dyn SegmentSink, scratch: &mut Scratch) -> Result<TraceEnd> {
        let mut t = 0.0;
        for _ in 0..opts.max_steps {
            let face_in = st.entered_via;
            let s = self.step(st, opts.max_dist - t, scratch)?;
            let face_out = match s.end {
                StepEnd::CellFace { face, .. } | StepEnd::RegionBoundary { face, .. } | StepEnd::Boundary { face, .. } => Some(face),
                _ => None,
            };
            if !matches!(s.end, StepEnd::Lost(_)) && !(s.dist == 0.0 && matches!(s.end, StepEnd::Limit)) {
                let seg = Segment::new(s.cell, s.region, t, s.dist).with_faces(face_in, face_out);
                if sink.segment(&seg) == Flow::Stop {
                    return Ok(TraceEnd::Stopped);
                }
            }
            t += s.dist;
            match s.end {
                StepEnd::Limit => return Ok(TraceEnd::Limit),
                StepEnd::CellFace { .. } | StepEnd::SlotChange { .. } => {}
                StepEnd::RegionBoundary { .. } if st.cell.is_none() => return Ok(TraceEnd::LeftModel),
                StepEnd::RegionBoundary { .. } => {}
                StepEnd::Boundary { action, .. } if action.applied_in_step() && !opts.stop_at_deck => {}
                StepEnd::Boundary { surface, face, action } => return Ok(TraceEnd::Boundary { surface, face, action }),
                StepEnd::Lost(l) => return Ok(TraceEnd::Lost(l)),
            }
        }
        Ok(TraceEnd::Lost(lost(st, LostCause::MaxStepsExceeded)))
    }

    /// Trace one block of rays into CSR storage. This is the unit of parallel work: an
    /// implementor's `trace_bundle` splits `rays` into fixed blocks of `BuildOptions::block_size`
    /// and calls this per block (one `Scratch` per worker).
    fn trace_bundle_block(&self, rays: &[Ray], opts: &TraceOptions, scratch: &mut Scratch) -> Result<TrackBundle> {
        let mut out = TrackBundle::new();
        let mut segs: Vec<Segment> = Vec::new();
        for r in rays {
            let d = r.dir;
            let n = d.norm();
            let unit = if n > 0.0 && n.is_finite() { d * (1.0 / n) } else { d };
            let mut st = self.init_state(r.origin, unit, None, scratch)?;
            segs.clear();
            let end = if st.cell.is_none() {
                TraceEnd::LeftModel
            } else {
                self.trace(
                    &mut st,
                    opts,
                    &mut |s: &Segment| {
                        segs.push(*s);
                        Flow::Continue
                    },
                    scratch,
                )?
            };
            out.push_ray(&segs, &end)?;
        }
        Ok(out)
    }

    /// Trace many rays into CSR storage (the MOC track database). The default is sequential;
    /// implementors parallelise over fixed blocks of `BuildOptions::block_size` rays, writing
    /// each block privately (`trace_bundle_block`) and concatenating blocks in index order
    /// (`TrackBundle::append`), so the result is independent of the thread count. Rays whose
    /// start is outside the model are recorded as `LeftModel` with no segments.
    fn trace_bundle(&self, rays: &[Ray], opts: &TraceOptions) -> Result<TrackBundle> {
        self.trace_bundle_block(rays, opts, &mut Scratch::new())
    }

    /// Apply a response boundary to a state sitting on a boundary face (after
    /// [`StepEnd::Boundary`] with a `Respond` action). `u` holds two uniforms in `[0,1]` from the
    /// caller's RNG; `White` reads both, `Albedo` reads `u[0]`, the others none.
    ///
    /// * `Vacuum`: terminate.
    /// * `Reflective`: exact specular reflection `d' = d - 2 (d.n) n` (exact for planar faces).
    /// * `White`: cosine-law direction about the inward normal.
    /// * `Albedo(a)`: if `u[0] < a` reflect specularly and continue, else terminate. `a` outside
    ///   `[0, 1]` is `RtError::BadAlbedo`.
    ///
    /// A used uniform that is NaN or outside `[0, 1]` is `RtError::InvalidInput` and the state
    /// is left unchanged.
    fn resolve_boundary(&self, st: &mut TrackState, face: FaceId, kind: ResponseKind, u: [f64; 2]) -> Result<BoundaryAction> {
        let unit = |x: f64| (0.0..=1.0).contains(&x);
        match kind {
            ResponseKind::Vacuum => Ok(BoundaryAction::Terminate),
            ResponseKind::Reflective => reflect_specular(self, st, face),
            ResponseKind::Albedo(a) => {
                if !kind.is_valid() {
                    return Err(RtError::BadAlbedo { value: a });
                }
                if !unit(u[0]) {
                    return Err(RtError::InvalidInput("random number u[0] must be finite and in [0, 1]"));
                }
                if u[0] < a { reflect_specular(self, st, face) } else { Ok(BoundaryAction::Terminate) }
            }
            ResponseKind::White => {
                if !unit(u[0]) || !unit(u[1]) {
                    return Err(RtError::InvalidInput("random numbers u must be finite and in [0, 1]"));
                }
                let n_in = -self.face_normal(face, st.pos)?;
                let axis = if n_in.x.abs() < 0.9 { Vector3::new(1.0, 0.0, 0.0) } else { Vector3::new(0.0, 1.0, 0.0) };
                let t1 = n_in.cross(axis);
                let t1 = t1 * (1.0 / t1.norm());
                let t2 = n_in.cross(t1);
                let (s, c) = (u[0].sqrt(), (1.0 - u[0]).sqrt());
                let phi = 2.0 * core::f64::consts::PI * u[1];
                let d = t1 * (s * phi.cos()) + t2 * (s * phi.sin()) + n_in * c;
                st.dir = d * (1.0 / d.norm());
                st.entered_via = Some(face);
                Ok(BoundaryAction::Continue)
            }
        }
    }

    /// Cell-to-bin map for tallies. `Cells` is the identity (the transport mesh *is* the tally
    /// mesh); `Regions` maps by region. Other specs are implementor-defined.
    fn bin_map(&self, spec: &TallySpec) -> Result<BinMap> {
        match spec {
            TallySpec::Cells => Ok(BinMap::identity(self.cell_count())),
            TallySpec::Regions => {
                let bins = (0..self.cell_count()).map(|c| self.cell_region(CellId::new(c)).map_or(BinMap::NO_BIN, |r| r.0)).collect();
                Ok(BinMap { n_bins: self.regions().0.len(), bin_of_cell: bins })
            }
            _ => Err(RtError::Unsupported("tally spec")),
        }
    }

    /// Whether `p` (moving along `dir`) is in `region`: DAGMC `point_in_volume`.
    fn point_in_region(&self, region: RegionId, p: Point3, dir: Option<Vector3>, scratch: &mut Scratch) -> Result<bool> {
        Ok(self.locate(p, dir, None, scratch)?.region == region)
    }
}

fn reflect_specular<T: Tracker + ?Sized>(t: &T, st: &mut TrackState, face: FaceId) -> Result<BoundaryAction> {
    let n = t.face_normal(face, st.pos)?;
    let d = st.dir - n * (2.0 * st.dir.dot(n));
    st.dir = d * (1.0 / d.norm());
    st.entered_via = Some(face);
    Ok(BoundaryAction::Continue)
}

/// Build a [`LostInfo`] from a state (helper for implementors).
pub fn lost(st: &TrackState, cause: LostCause) -> LostInfo {
    let mut l = LostInfo::new(cause, st.pos, st.dir, st.cell);
    l.last_face = st.entered_via;
    l
}

/// Advance a state to the end of a step of length `d` (helper for implementors).
pub fn move_along(st: &mut TrackState, d: f64) {
    advance(st, d);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hashed_direction_is_unit_deterministic_and_spread() {
        let p = Point3::new(1.0, -2.0, 0.5);
        let d = hashed_direction(p);
        assert!((d.norm() - 1.0).abs() < 1e-15);
        assert_eq!(d, hashed_direction(p));
        assert_eq!(hashed_direction(Point3::new(0.0, 1.0, 2.0)), hashed_direction(Point3::new(-0.0, 1.0, 2.0)));
        // Different points give different directions and cover both hemispheres.
        let mut up = 0;
        for i in 0..200 {
            let q = hashed_direction(Point3::new(f64::from(i), 0.5, -1.0));
            assert!((q.norm() - 1.0).abs() < 1e-15);
            if q.z > 0.0 {
                up += 1;
            }
        }
        assert!((60..140).contains(&up), "z sign balance {up}");
    }

    #[test]
    fn direction_check_uses_the_given_tolerance() {
        assert!(check_direction(Vector3::new(1.0, 0.0, 0.0), 1e-9).is_ok());
        assert!(check_direction(Vector3::new(1.0 + 1e-6, 0.0, 0.0), 1e-9).is_err());
        assert!(check_direction(Vector3::new(1.0 + 1e-6, 0.0, 0.0), 1e-3).is_ok());
        assert!(check_direction(Vector3::new(f64::NAN, 0.0, 0.0), 1.0).is_err());
    }
}
