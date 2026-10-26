//! The [`Tracker`] trait: the one query surface.
//!
//! Required methods are the primitives (`locate`, `step`, `closest`, tables, boundary
//! geometry). Everything a transport code calls (`fire`, `trace`, `trace_bundle`,
//! `resolve_boundary`, `bin_map`) is a provided method built on `step`, so every implementor
//! gets identical event semantics and the conformance tests cover one code path.
//!
//! # Mapping from DAGMC
//! | DAGMC | here |
//! |---|---|
//! | `point_in_volume(vol, xyz, uvw, history)` | [`Tracker::locate`] (direction-aware on a face) then compare region |
//! | `find_volume` | [`Tracker::locate`] |
//! | `ray_fire(vol, xyz, uvw, &surf, &dist, history, limit)` | [`Tracker::fire`] |
//! | `RayHistory` | `TrackState::entered_via` (exact adjacency makes a history unnecessary) |
//! | `next_vol(surf, vol)` | [`SurfaceTable::next_region`] |
//! | `closest_to_location` | [`Tracker::closest`] |
//! | `get_angle` / surface normal | [`Tracker::face_normal`] |
//! | `test_volume_boundary` | `locate` with `dir` |
//! | implicit complement | `CellId::COMPLEMENT`, [`RegionTable::implicit_complement`] |

use crate::cache::{CacheImage, ContentHash};
use crate::error::{LostCause, LostInfo, Result, RtError};
use crate::ids::{CellId, FaceId, RegionId};
use crate::state::{
    BoundaryKind, Closest, Located, RegionTable, Step, StepEnd, SurfaceTable, TrackState, TrackerCaps,
};
use crate::tally::{
    BinMap, FireResult, Flow, Segment, SegmentSink, TallySpec, TraceEnd, TraceOptions, TrackBundle, advance,
};
use sm_core::{Aabb, Point3, Transform, Vector3};
use sm_geom_api::Ray;
use sm_mesh_api::MeshRead;

/// Tolerance for "unit length" checks on directions.
pub const UNIT_TOL: f64 = 1e-9;

/// Validate a direction.
pub fn check_direction(d: Vector3) -> Result<()> {
    let n = d.norm();
    if !n.is_finite() || (n - 1.0).abs() > UNIT_TOL {
        return Err(RtError::BadDirection { norm: n });
    }
    Ok(())
}

/// Where a periodic face continues.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PeriodicImage {
    /// The matched face on the partner surface.
    pub face: FaceId,
    /// The cell behind it (inside the model).
    pub to_cell: CellId,
    /// Rigid map from this face's frame to the partner's (applied to position and direction).
    pub transform: Transform,
}

/// What `resolve_boundary` did.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum BoundaryAction {
    /// Particle leaves the problem; state unchanged.
    Terminate,
    /// State was updated (reflected, re-emitted, or moved to the periodic partner); keep tracking.
    Continue,
}

/// Build-time facts of a tracker, for provenance and cache checks.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackerInfo {
    /// Number of tracking cells.
    pub cells: usize,
    /// Number of faces/facets.
    pub faces: usize,
    /// Hash of the built structure (independent of thread count).
    pub structure_hash: ContentHash,
}

/// The ray-tracing / particle-tracking query surface. Implementors are immutable after
/// construction and `Send + Sync`: any number of threads may query one tracker, each with its
/// own [`TrackState`].
///
/// # Determinism contract
/// * Every query is a pure function of (tracker, arguments). No query reads time, thread id or
///   global state.
/// * Ties are broken by lowest id: when a ray exits through an edge or vertex shared by several
///   faces, the lowest `FaceId` among those with the minimal parameter wins; when a point lies on
///   several faces, the cell the direction points into wins, then the lowest `CellId`.
/// * `trace_bundle` returns rays in input order and its content hash is identical for any
///   thread count.
/// * Building the same inputs gives the same [`Tracker::structure_hash`].
pub trait Tracker: Send + Sync {
    // ---- identity and tables -------------------------------------------------------------

    /// What this tracker supports.
    fn caps(&self) -> TrackerCaps;
    /// Bounding box of the model.
    fn bounds(&self) -> Aabb;
    /// Regions.
    fn regions(&self) -> &RegionTable;
    /// Model surfaces.
    fn surfaces(&self) -> &SurfaceTable;
    /// Number of tracking cells (global, instance-numbered).
    fn cell_count(&self) -> usize;
    /// Region of a cell.
    fn cell_region(&self, c: CellId) -> Option<RegionId>;
    /// Volume of a cell, when known (mesh-native: exact polyhedron volume from
    /// `sm_mesh_api::cell_volume`).
    fn cell_volume(&self, c: CellId) -> Option<f64>;
    /// The tracking mesh when there is one: the same object Sn/MOC and tallies index by `CellId`.
    fn mesh(&self) -> Option<&dyn MeshRead> {
        None
    }
    /// Hash of the built structure.
    fn structure_hash(&self) -> ContentHash;
    /// Serialize the structure for the `.smsh` cache (`None` if not cacheable).
    fn export_cache(&self) -> Option<CacheImage> {
        None
    }

    // ---- primitives ----------------------------------------------------------------------

    /// Find the cell containing `p`. `dir` resolves a point on a face (the particle is in the
    /// cell it is moving into), as DAGMC's `point_in_volume(uvw)`. `hint` is a cell to start a
    /// visibility walk from (previous position of a particle); without it a BVH over cells is used.
    /// A point outside the model gives `CellId::NONE`, in the complement `CellId::COMPLEMENT`.
    fn locate(&self, p: Point3, dir: Option<Vector3>, hint: Option<CellId>) -> Result<Located>;

    /// Advance to the next cell face, region boundary, domain boundary or the distance limit,
    /// whichever comes first, updating `st` (see [`StepEnd`] for the post-state of each case).
    /// `st.dir` must be unit. Never allocates.
    fn step(&self, st: &mut TrackState, max_dist: f64) -> Result<Step>;

    /// Distance and location of the closest boundary of `region` (or of any region when `None`)
    /// from `p`. Used for safety distances and as the DAGMC `closest_to_location`.
    fn closest(&self, p: Point3, region: Option<RegionId>) -> Result<Option<Closest>>;

    /// Unit normal of a face pointing out of its `left` cell (outward at a boundary face).
    fn face_normal(&self, f: FaceId) -> Result<Vector3>;

    /// Periodic continuation of a boundary face on a `Periodic` surface.
    fn periodic_image(&self, _f: FaceId) -> Result<Option<PeriodicImage>> {
        Ok(None)
    }

    // ---- provided ------------------------------------------------------------------------

    /// Locate a particle and build its state. Fails on a bad direction or when the point is
    /// outside the model (`CellId::NONE`).
    fn init_state(&self, pos: Point3, dir: Vector3, hint: Option<CellId>) -> Result<TrackState> {
        check_direction(dir)?;
        if !(pos.x.is_finite() && pos.y.is_finite() && pos.z.is_finite()) {
            return Err(RtError::NonFinite("position"));
        }
        let loc = self.locate(pos, Some(dir), hint)?;
        let mut st = TrackState::new(pos, dir);
        st.cell = loc.cell;
        st.region = loc.region;
        st.entered_via = loc.on_face;
        Ok(st)
    }

    /// DAGMC `ray_fire`: walk through cells of the current region until the region changes, a
    /// domain boundary is reached, the limit is hit or the particle is lost. `end` is never
    /// `CellFace`. The interior cell faces are skipped but cost only a table lookup each.
    fn fire(&self, st: &mut TrackState, dist_limit: f64, max_steps: u32) -> Result<FireResult> {
        let mut total = 0.0;
        for _ in 0..max_steps {
            let s = self.step(st, dist_limit - total)?;
            total += s.dist;
            if !matches!(s.end, StepEnd::CellFace { .. }) {
                return Ok(FireResult { dist: total, end: s.end });
            }
        }
        Ok(FireResult { dist: total, end: StepEnd::Lost(lost(st, LostCause::MaxStepsExceeded)) })
    }

    /// Characteristics: emit one [`Segment`] per cell chord until the limit, a boundary, loss or
    /// the sink stops. The same call serves a Monte Carlo track-length tally (sink scores
    /// `weight * length`, may `Stop` at a collision) and an MOC/Sn pass (sink records chords).
    fn trace(&self, st: &mut TrackState, opts: &TraceOptions, sink: &mut dyn SegmentSink) -> Result<TraceEnd> {
        let mut t = 0.0;
        for _ in 0..opts.max_steps {
            let face_in = st.entered_via;
            let s = self.step(st, opts.max_dist - t)?;
            let face_out = match s.end {
                StepEnd::CellFace { face, .. } | StepEnd::RegionBoundary { face, .. } | StepEnd::DomainBoundary { face, .. } => Some(face),
                _ => None,
            };
            if !matches!(s.end, StepEnd::Lost(_)) {
                let seg = Segment { cell: s.cell, region: s.region, t0: t, length: s.dist, face_in, face_out };
                if sink.segment(&seg) == Flow::Stop {
                    return Ok(TraceEnd::Stopped);
                }
            }
            t += s.dist;
            match s.end {
                StepEnd::Limit => return Ok(TraceEnd::Limit),
                StepEnd::CellFace { .. } => {}
                StepEnd::RegionBoundary { .. } if st.cell.is_none() => return Ok(TraceEnd::LeftModel),
                StepEnd::RegionBoundary { .. } => {}
                StepEnd::DomainBoundary { surface, face, kind } => return Ok(TraceEnd::Domain { surface, face, kind }),
                StepEnd::Lost(l) => return Ok(TraceEnd::Lost(l)),
            }
        }
        Ok(TraceEnd::Lost(lost(st, LostCause::MaxStepsExceeded)))
    }

    /// Trace many rays into CSR storage (the MOC track database). The default is sequential;
    /// implementors parallelise over blocks of `BuildOptions::block_size` rays, writing each
    /// block privately and concatenating blocks in index order, so the result is independent of
    /// the thread count. Rays whose start is outside the model are recorded as `LeftModel`
    /// with no segments.
    fn trace_bundle(&self, rays: &[Ray], opts: &TraceOptions) -> Result<TrackBundle> {
        let mut out = TrackBundle::new();
        let mut segs: Vec<Segment> = Vec::new();
        for r in rays {
            let d = r.dir;
            let n = d.norm();
            let unit = if n > 0.0 { d * (1.0 / n) } else { d };
            let mut st = self.init_state(r.origin, unit, None)?;
            segs.clear();
            let end = if st.cell.is_none() {
                TraceEnd::LeftModel
            } else {
                self.trace(&mut st, opts, &mut |s: &Segment| {
                    segs.push(*s);
                    Flow::Continue
                })?
            };
            out.push_ray(&segs, &end);
        }
        Ok(out)
    }

    /// Apply a domain boundary condition to a state sitting on a boundary face (after
    /// [`StepEnd::DomainBoundary`]). `u` holds two uniforms in `[0,1)` from the caller's RNG;
    /// it is read only by `White`. Reflection is exact for planar faces:
    /// `d' = d - 2 (d.n) n`.
    fn resolve_boundary(&self, st: &mut TrackState, face: FaceId, kind: BoundaryKind, u: [f64; 2]) -> Result<BoundaryAction> {
        match kind {
            BoundaryKind::Vacuum => Ok(BoundaryAction::Terminate),
            BoundaryKind::Reflective => {
                let n = self.face_normal(face)?;
                let d = st.dir - n * (2.0 * st.dir.dot(n));
                st.dir = d * (1.0 / d.norm());
                st.entered_via = Some(face);
                Ok(BoundaryAction::Continue)
            }
            BoundaryKind::White => {
                let n_in = -self.face_normal(face)?;
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
            BoundaryKind::Periodic { .. } => {
                let img = self.periodic_image(face)?.ok_or(RtError::Unsupported("periodic boundary"))?;
                st.pos = img.transform.apply_point(st.pos);
                let d = img.transform.apply_vector(st.dir);
                st.dir = d * (1.0 / d.norm());
                st.cell = img.to_cell;
                st.region = self.cell_region(img.to_cell).unwrap_or(RegionId::NONE);
                st.entered_via = Some(img.face);
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
    fn point_in_region(&self, region: RegionId, p: Point3, dir: Option<Vector3>) -> Result<bool> {
        Ok(self.locate(p, dir, None)?.region == region)
    }

    /// Summary for provenance.
    fn info(&self) -> TrackerInfo {
        TrackerInfo { cells: self.cell_count(), faces: self.mesh().map_or(0, |m| m.face_count()), structure_hash: self.structure_hash() }
    }
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
