//! The query surface: [`Tracker`], plus its options and info.
//!
//! # Mapping from DAGMC
//! | DAGMC `DagMC`                    | here                                              |
//! |----------------------------------|---------------------------------------------------|
//! | `find_volume` / `point_in_volume`| [`Tracker::locate`] (direction breaks ties on a surface) |
//! | `ray_fire`                       | [`Tracker::distance_to_boundary`] (peek) / [`Tracker::step`] (move + cross) |
//! | `next_vol(surf, vol)`            | inside `step`, read from face sides, no search    |
//! | `RayHistory`                     | [`TrackState`] (`last`, `levels`)                 |
//! | `closest_to_location`            | [`Tracker::nearest_surface`]                      |
//! | `get_angle`                      | [`Tracker::normal_at`]                            |
//! | `overlap_thickness`, `numerical_precision` | [`RtOptions::tol`] (from `ModelTolerance`, not hard-coded) |
//! | implicit complement              | `Region::Exterior` locations                      |
//!
//! # Contracts (binding on implementors)
//! - **No allocation after build** on `locate`, `step`, `distance_to_boundary`,
//!   `nearest_surface`, for states whose hierarchy depth is at most [`crate::LEVEL_INLINE`].
//!   Working memory is the caller's [`Scratch`].
//! - **Thread safety:** a `Tracker` is immutable after build; any number of threads may query it
//!   with their own `Scratch` and `TrackState`.
//! - **Determinism:** the result of a query depends only on its inputs and the tracker's
//!   content, never on thread count, call order or timing. Ties between faces at equal distance
//!   (within `tol.linear`) break by ascending face id, then copy index. `trace_many` output is
//!   bit-identical for any thread count.
//! - **No RNG.** White and albedo boundaries are reported ([`crate::Event::Response`]).
//! - **Lost particles are values** ([`crate::Event::Lost`]); no query panics on finite input.
//! - **Exactness:** analytic faces (planes, quadrics, tori, elliptic kinds, CSG half-spaces) are
//!   intersected through `ParametricSurface::ray_hits` / `ImplicitSurface::ray_hits`, with no
//!   faceting error. Only faces whose carrier has no implicit form (NURBS) use facets, and
//!   [`TrackerInfo::faceted_faces`] reports exactly which.

use crate::cache::RtCacheKey;
use crate::error::{LostReason, LostReport, Result, RtError};
use crate::types::{BoundaryHit, FlatIx, Nearest, Scratch, Segment, SegmentSink, Step, TraceBatch, TraceEnd, TrackState};
use sm_core::{Aabb, EntityId, Point3, Vector3};
use sm_geom_api::{ModelTolerance, Ray};
use sm_model_api::InstancePath;

/// What the tracker does when a step ends within tolerance of an edge or finds no consistent
/// neighbour.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum RecoveryPolicy {
    /// Report `Lost` immediately (use in tests and validation: a lost particle is a model bug).
    Fail,
    /// Retry up to `tries` times, moving the point along the direction by `tol.linear * 2^k`
    /// and re-locating with the direction hint. Deterministic (no random nudges).
    Nudge {
        /// Maximum retries.
        tries: u8,
    },
}

/// How facets are used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum FacetFallback {
    /// Facets only for carriers without an implicit form (NURBS). The default.
    NoImplicitOnly,
    /// Never use facets; such faces make the build fail with `RtError::Input`.
    Never,
    /// Facets for every face (DAGMC mode; a baseline for benchmarks and for pure-facet inputs).
    Always,
}

/// Build and run options. The content hash covers every field (see [`RtOptions::hash_words`]),
/// so a cache built with other options is never reused.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct RtOptions {
    /// Tolerance policy (take it from the topology view; `overlap_thickness` and
    /// `numerical_precision` of DAGMC both derive from it).
    pub tol: ModelTolerance,
    /// Recovery on inconsistent state.
    pub recovery: RecoveryPolicy,
    /// Facet use.
    pub facets: FacetFallback,
    /// Maximum events in one `trace`, a guard against reflection traps.
    pub max_events: u32,
    /// Chord deviation (relative to the face size) for the facet fallback. Ignored for analytic faces.
    pub facet_deflection: f64,
}

impl RtOptions {
    /// Defaults for a model with tolerance `tol`: nudge twice, facets for NURBS only,
    /// one million events, 1e-4 relative deflection.
    pub fn new(tol: ModelTolerance) -> Self {
        Self { tol, recovery: RecoveryPolicy::Nudge { tries: 2 }, facets: FacetFallback::NoImplicitOnly, max_events: 1_000_000, facet_deflection: 1e-4 }
    }

    /// The options as a word stream for the cache key (floats as raw bits; fields in
    /// declaration order).
    pub fn hash_words(&self) -> [u64; 8] {
        let (rec, tries) = match self.recovery {
            RecoveryPolicy::Fail => (0, 0),
            RecoveryPolicy::Nudge { tries } => (1, u64::from(tries)),
        };
        let fac = match self.facets {
            FacetFallback::NoImplicitOnly => 0,
            FacetFallback::Never => 1,
            FacetFallback::Always => 2,
        };
        [self.tol.linear.to_bits(), self.tol.angular.to_bits(), rec << 8 | tries, fac, u64::from(self.max_events), self.facet_deflection.to_bits(), self.tol.parametric.to_bits(), self.tol.linear_max.to_bits()]
    }
}

/// Facts about a built tracker, for reports and benchmarks.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct TrackerInfo {
    /// World-frame bounds of the model (all copies).
    pub bounds: Aabb,
    /// Number of volume copies (length of the flat table).
    pub copies: u32,
    /// Number of distinct prototype structures actually stored (copies share these).
    pub prototypes: u32,
    /// Faces intersected exactly.
    pub exact_faces: u32,
    /// Faces that fell back to facets, ascending (empty for a pure CSG or quadric model).
    pub faceted_faces: Vec<EntityId>,
    /// Heap bytes held by the structure (the memory-per-copy claim is `memory_bytes / copies`).
    pub memory_bytes: u64,
    /// The cache key of this structure.
    pub key: RtCacheKey,
    /// The options it was built with.
    pub options: RtOptions,
}

/// The tracking structure. Object-safe: transport codes hold `Box<dyn Tracker>`.
pub trait Tracker: Send + Sync + core::fmt::Debug {
    /// Facts about this structure.
    fn info(&self) -> &TrackerInfo;

    /// Find the location of `p`. `dir` resolves points on a surface: the particle is placed in
    /// the region it is *entering* (DAGMC's `uvw` argument). Outside every region the answer is
    /// the implicit complement if the model has one, otherwise `RtError::Lost(NoRegionAtPoint)`.
    /// Cost is a descent through the hierarchy (lattice index arithmetic, then the prototype's
    /// bounding hierarchy); after the first call stepping never re-searches.
    fn locate(&self, p: Point3, dir: Option<Vector3>, scratch: &mut Scratch) -> Result<TrackState>;

    /// Distance to the next face along the state's direction, without moving (`ray_fire`).
    /// `None` if nothing lies within `limit`. Hits closer than `tol.linear` that are consistent
    /// with the state's current side are real crossings; hits that are inconsistent (the
    /// self-hit at `t ~ 0`) are skipped exactly, with no distance heuristic.
    fn distance_to_boundary(&self, state: &TrackState, limit: f64, scratch: &mut Scratch) -> Result<Option<BoundaryHit>>;

    /// Move the particle to the next face (or `limit`) and apply what the face does: cross into
    /// the neighbour, descend or ascend the hierarchy, reflect or wrap at a deck face, report a
    /// response boundary. Updates `state` in place.
    fn step(&self, state: &mut TrackState, limit: f64, scratch: &mut Scratch) -> Result<Step>;

    /// Nearest face of the current region and the distance to it (`closest_to_location`), for
    /// next-event estimators and for `limit` selection in delta-tracking.
    fn nearest_surface(&self, state: &TrackState, scratch: &mut Scratch) -> Result<Nearest>;

    /// Unit normal of `face` at `p` in the world frame (`get_angle`), pointing along the face's
    /// own orientation.
    fn normal_at(&self, face: EntityId, copy: FlatIx, p: Point3) -> Result<Vector3>;

    /// Instance path naming `copy` (diagnostics only; may allocate lazily at build time, not per query).
    fn path_of(&self, copy: FlatIx) -> Option<&InstancePath>;

    /// Trace `ray` for at most `limit` of path length, sending every piece to `sink`. Reflect and
    /// wrap events continue. The default is built on `locate` + `step`.
    fn trace(&self, ray: &Ray, limit: f64, sink: &mut dyn SegmentSink, scratch: &mut Scratch) -> Result<TraceEnd> {
        let n = ray.dir.norm();
        if !(n.is_finite() && n > 0.0) || !(ray.origin.x.is_finite() && ray.origin.y.is_finite() && ray.origin.z.is_finite()) {
            return Ok(TraceEnd::Lost(Box::new(LostReport::new(LostReason::NonFinite, ray.origin, ray.dir))));
        }
        let dir = Vector3::new(ray.dir.x / n, ray.dir.y / n, ray.dir.z / n);
        let mut st = match self.locate(ray.origin, Some(dir), scratch) {
            Ok(s) => s,
            Err(RtError::Lost(r)) => return Ok(TraceEnd::Lost(r)),
            Err(e) => return Err(e),
        };
        let mut left = limit;
        for _ in 0..=self.info().options.max_events {
            let (start, d, at) = (st.pos, st.dir, st.at);
            let step = self.step(&mut st, left, scratch)?;
            sink.segment(&Segment { at, start, dir: d, length: step.distance });
            left -= step.distance;
            match step.event {
                crate::Event::Lost(r) => return Ok(TraceEnd::Lost(r)),
                crate::Event::LimitReached => return Ok(TraceEnd::LimitReached),
                crate::Event::Response { face, kind, .. } => return Ok(TraceEnd::Response { face, kind }),
                _ => {}
            }
            if left <= 0.0 {
                return Ok(TraceEnd::LimitReached);
            }
        }
        Ok(TraceEnd::Lost(Box::new(LostReport::new(LostReason::EventLimit, st.pos, st.dir))))
    }

    /// Trace many rays into a CSR [`TraceBatch`]. Contract: ray `i`'s result depends only on
    /// ray `i`, and the output is bit-identical for any thread count. The default is a
    /// sequential loop; the implementor overrides it with a fixed-chunk parallel version that
    /// concatenates chunk outputs in ray order.
    fn trace_many(&self, rays: &[Ray], limit: f64, out: &mut TraceBatch) -> Result<()> {
        let mut scratch = Scratch::new();
        out.offsets.clear();
        out.segments.clear();
        out.ends.clear();
        out.offsets.push(0);
        for r in rays {
            let end = self.trace(r, limit, &mut out.segments, &mut scratch)?;
            out.offsets.push(out.segments.len() as u32);
            out.ends.push(end);
        }
        Ok(())
    }
}
