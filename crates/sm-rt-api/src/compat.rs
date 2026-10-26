//! DAGMC-shaped queries as thin functions over [`Tracker`], so a code written against DAGMC
//! (`ray_fire`, `point_in_volume`, `next_vol`, `surface_sense`, `get_angle`) ports without
//! restructuring. They add no state: each is a few lines over `locate`, `step` and the tables.
//!
//! Conventions (these differ from DAGMC only where DAGMC is random or ambiguous):
//! * a *volume* is a [`RegionId`], a *surface* is a [`SurfaceId`];
//! * `surface_sense` is `+1` when the surface normal points **out of** the volume (DAGMC
//!   `SENSE_FORWARD`), `-1` when it points in;
//! * `point_in_volume` with `dir = None` on a surface uses [`crate::hashed_direction`], not a
//!   random direction, so results are reproducible;
//! * a distance limit of `None` means unlimited (DAGMC uses `0`).

use crate::error::{LostInfo, Result};
use crate::ids::{FaceId, RegionId, SurfaceId};
use crate::state::{Closest, Scratch, StepEnd};
use crate::tracker::Tracker;
use sm_core::{Point3, Vector3};

/// Which crossings of the volume's surface `ray_fire` accepts (DAGMC `ray_orientation`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum RayOrientation {
    /// Only a crossing out of the volume (DAGMC default, orientation 1).
    #[default]
    Exiting,
    /// Only a crossing into the volume.
    Entering,
    /// Either.
    Any,
}

/// A surface hit found by [`ray_fire`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct FireHit {
    /// The surface hit (DAGMC `next_surf`).
    pub surface: SurfaceId,
    /// The face hit.
    pub face: FaceId,
    /// Distance from the origin (DAGMC `next_surf_dist`).
    pub dist: f64,
    /// Region on the other side; `RegionId::NONE` at a model boundary.
    pub to_region: RegionId,
}

impl FireHit {
    /// Construct (the type is `#[non_exhaustive]`).
    pub const fn new(surface: SurfaceId, face: FaceId, dist: f64, to_region: RegionId) -> Self {
        Self { surface, face, dist, to_region }
    }
}

/// Outcome of [`ray_fire`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum RayFire {
    /// A surface was hit within the limit.
    Hit(FireHit),
    /// No acceptable hit within the limit, the origin is outside the model, or the origin is not
    /// in the volume for an exiting ray.
    Miss,
    /// Tracking failed; a located value, never a panic.
    Lost(LostInfo),
}

/// DAGMC `ray_fire(volume, origin, dir, ...)`. `dir` must be unit.
///
/// * `Exiting`: the origin must be in `volume` (otherwise `Miss`); returns the first surface
///   the ray leaves it through.
/// * `Entering`: the first surface the ray enters `volume` through.
/// * `Any`: the first surface between `volume` and anything else.
///
/// An origin outside the model (`CellId::NONE`) is a `Miss` for every orientation, as in DAGMC
/// (no hit, no error); no recovery nudge is attempted. `Lost` is reserved for tracking failures
/// after a valid start.
///
/// Interior cell faces and lattice walls are skipped. A model boundary (`Boundary` event) is a
/// hit when the ray is in `volume` and the orientation is not `Entering`; deck boundaries are
/// reported, not followed, so the caller sees every surface.
pub fn ray_fire<T: Tracker + ?Sized>(
    t: &T,
    volume: RegionId,
    origin: Point3,
    dir: Vector3,
    orientation: RayOrientation,
    dist_limit: Option<f64>,
    scratch: &mut Scratch,
) -> Result<RayFire> {
    let mut st = t.init_state(origin, dir, None, scratch)?;
    if st.cell.is_none() {
        return Ok(RayFire::Miss);
    }
    let limit = dist_limit.unwrap_or(f64::INFINITY);
    let max = t.info().max_events;
    let mut total = 0.0;
    // One step budget in total (`max` events), spent directly on `step`: calling `fire` per
    // region change would allow up to `max^2` steps.
    for _ in 0..max {
        let from = st.region;
        let s = t.step(&mut st, limit - total, scratch)?;
        total += s.dist;
        match s.end {
            StepEnd::Limit => return Ok(RayFire::Miss),
            StepEnd::Lost(l) => return Ok(RayFire::Lost(l)),
            StepEnd::RegionBoundary { surface, face, to_region, .. } => {
                let accept = match orientation {
                    RayOrientation::Exiting => from == volume,
                    RayOrientation::Entering => to_region == volume,
                    RayOrientation::Any => from == volume || to_region == volume,
                };
                if accept {
                    return Ok(RayFire::Hit(FireHit::new(surface, face, total, to_region)));
                }
                if orientation == RayOrientation::Exiting {
                    return Ok(RayFire::Miss);
                }
            }
            StepEnd::Boundary { surface, face, .. } => {
                if from == volume && orientation != RayOrientation::Entering {
                    return Ok(RayFire::Hit(FireHit::new(surface, face, total, RegionId::NONE)));
                }
                return Ok(RayFire::Miss);
            }
            StepEnd::CellFace { .. } | StepEnd::SlotChange { .. } => {}
        }
    }
    Ok(RayFire::Lost(crate::tracker::lost(&st, crate::error::LostCause::MaxStepsExceeded)))
}

/// DAGMC `point_in_volume`: is `p` in `volume`? On a surface the direction `dir` decides (the
/// cell the particle moves into); `None` uses the deterministic hashed direction.
pub fn point_in_volume<T: Tracker + ?Sized>(t: &T, volume: RegionId, p: Point3, dir: Option<Vector3>, scratch: &mut Scratch) -> Result<bool> {
    t.point_in_region(volume, p, dir, scratch)
}

/// DAGMC `next_vol(surface, old_volume)`: the volume on the other side of `surface`.
pub fn next_vol<T: Tracker + ?Sized>(t: &T, surface: SurfaceId, from: RegionId) -> Option<RegionId> {
    t.surfaces().next_region(surface, from)
}

/// DAGMC `surface_sense`: `+1` if the surface normal points out of `volume`, `-1` if into it,
/// `None` if the volume does not touch the surface.
pub fn surface_sense<T: Tracker + ?Sized>(t: &T, surface: SurfaceId, volume: RegionId) -> Option<i32> {
    t.surfaces().sense(surface, volume)
}

/// DAGMC `get_angle`: the unit normal of `face` at `p`, pointing out of its `front` volume.
pub fn get_angle<T: Tracker + ?Sized>(t: &T, face: FaceId, p: Point3) -> Result<Vector3> {
    t.face_normal(face, p)
}

/// DAGMC `closest_to_location(volume, p)`.
pub fn closest_to_location<T: Tracker + ?Sized>(t: &T, volume: RegionId, p: Point3, scratch: &mut Scratch) -> Result<Option<Closest>> {
    t.closest(p, Some(volume), scratch)
}
