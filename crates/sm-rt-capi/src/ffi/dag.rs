//! DAGMC-shaped shims: the provided functions of `sm_rt_api::compat`, wrapped one to one.
//!
//! Volumes are regions (`SMRT_COMPLEMENT` = the implicit complement) and surfaces are model
//! surfaces. Differences from DAGMC: no history vector (`entered_via` replaces it), a null `dir`
//! uses the deterministic hashed direction instead of a random one, `dist_limit == 0` means
//! unlimited as in DAGMC, and a lost particle is `SMRT_ERR_LOST`.

use super::{SmrtCtx, SmrtTracker, exclusive, guard, read3, read3_opt, same_tracker, shared, write3};
use crate::convert::*;
use crate::status::*;
use crate::types::*;
use sm_rt_api::compat::{self, RayFire, RayOrientation};

/// DAGMC `ray_fire`. Fires from `origin[3]` along the unit `dir[3]` in `region` and reports the
/// first surface in `*out_surface` at distance `*out_dist`; `out_face` (may be null) gets the
/// face hit, which `smrt_dag_get_angle` takes. `orientation` is `SMRT_ORIENT_EXITING` (1),
/// `SMRT_ORIENT_ENTERING` (-1) or `SMRT_ORIENT_ANY` (0); `dist_limit` of `0` (or infinity) is
/// unlimited. Returns `0` for a hit, `1` for no hit (outputs `-1` and `0`), `SMRT_ERR_LOST` if
/// tracking failed. Uses `ctx` for scratch only; the context's particle state is untouched.
///
/// # Safety
/// `tracker` and `ctx` are null or live handles (the context made from this tracker, used by this
/// thread only); `origin`, `dir` are null or three readable doubles; `out_surface`, `out_dist`
/// are null or valid for a write, `out_face` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_dag_ray_fire(
    tracker: *const SmrtTracker,
    ctx: *mut SmrtCtx,
    region: i64,
    origin: *const f64,
    dir: *const f64,
    orientation: i32,
    dist_limit: f64,
    out_surface: *mut i64,
    out_dist: *mut f64,
    out_face: *mut i64,
) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (t, c, o, d, surf, dist) =
            unsafe { (shared(tracker, "tracker")?, exclusive(ctx, "ctx")?, read3(origin, "origin")?, read3(dir, "dir")?, exclusive(out_surface, "out_surface")?, exclusive(out_dist, "out_dist")?) };
        same_tracker(t, c)?;
        let orient = match orientation {
            SMRT_ORIENT_EXITING => RayOrientation::Exiting,
            SMRT_ORIENT_ENTERING => RayOrientation::Entering,
            SMRT_ORIENT_ANY => RayOrientation::Any,
            v => return Err(Fail::bad(format!("bad ray orientation {v}"))),
        };
        if dist_limit.is_nan() || dist_limit < 0.0 {
            return Err(Fail::bad("dist_limit must be >= 0 (0 = unlimited)"));
        }
        let limit = (dist_limit > 0.0 && dist_limit.is_finite()).then_some(dist_limit);
        let vol = region_in(region, t.complement)?;
        let r = compat::ray_fire(&*t.inner, vol, point(o), vector(d), orient, limit, &mut c.scratch)?;
        match r {
            RayFire::Hit(h) => {
                *surf = id_out(h.surface.0);
                *dist = h.dist;
                // SAFETY: a non-null `out_face` is valid for a write by the caller contract.
                if let Some(f) = unsafe { out_face.as_mut() } {
                    *f = id_out(h.face.0);
                }
                Ok(SMRT_OK)
            }
            RayFire::Miss => {
                *surf = SMRT_NONE;
                *dist = 0.0;
                // SAFETY: as above.
                if let Some(f) = unsafe { out_face.as_mut() } {
                    *f = SMRT_NONE;
                }
                Ok(SMRT_NO_HIT)
            }
            RayFire::Lost(l) => Err(Fail::Lost(format!("ray_fire lost the particle: {:?} at ({}, {}, {})", l.cause, l.pos.x, l.pos.y, l.pos.z))),
            _ => Err(Fail::Lost("ray_fire: unknown outcome".into())),
        }
    })
}

/// DAGMC `point_in_volume`: `*out_inside` is 1 if `p[3]` is in `region`, else 0. On a surface the
/// direction `dir_or_null` (three doubles) decides; null uses the hashed direction. Returns `0`.
///
/// # Safety
/// `tracker`, `ctx` as for `smrt_dag_ray_fire`; `p` three readable doubles; `dir_or_null` null or
/// three readable doubles; `out_inside` valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_dag_point_in_volume(
    tracker: *const SmrtTracker,
    ctx: *mut SmrtCtx,
    region: i64,
    p: *const f64,
    dir_or_null: *const f64,
    out_inside: *mut i32,
) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (t, c, p, d, out) = unsafe { (shared(tracker, "tracker")?, exclusive(ctx, "ctx")?, read3(p, "p")?, read3_opt(dir_or_null, "dir")?, exclusive(out_inside, "out_inside")?) };
        same_tracker(t, c)?;
        let vol = region_in(region, t.complement)?;
        let inside = compat::point_in_volume(&*t.inner, vol, point(p), d.map(vector), &mut c.scratch)?;
        *out = i32::from(inside);
        Ok(SMRT_OK)
    })
}

/// DAGMC `next_vol`: the region on the other side of `surface` when leaving `old_region`
/// (`SMRT_COMPLEMENT` for the implicit complement, `-1` outside the model). Returns `0`, or `1`
/// (output `-1`) when the region does not touch the surface.
///
/// # Safety
/// `tracker` is null or a live handle; `out_new_region` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_dag_next_vol(tracker: *const SmrtTracker, surface: i64, old_region: i64, out_new_region: *mut i64) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (t, out) = unsafe { (shared(tracker, "tracker")?, exclusive(out_new_region, "out_new_region")?) };
        let s = surface_in(surface)?;
        let from = region_in(old_region, t.complement)?;
        if t.inner.surfaces().get(s).is_none() {
            return Err(sm_rt_api::RtError::UnknownId { kind: "surface", id: s.0 }.into());
        }
        match compat::next_vol(&*t.inner, s, from) {
            Some(r) => {
                *out = region_out(r, t.complement);
                Ok(SMRT_OK)
            }
            None => {
                *out = SMRT_NONE;
                Ok(SMRT_NO_HIT)
            }
        }
    })
}

/// DAGMC `surface_sense`: `*out_sense` is `+1` if the surface normal points out of `region`
/// (DAGMC forward), `-1` if it points into it. Returns `0`, or `1` (output `0`) when the region
/// does not touch the surface.
///
/// # Safety
/// `tracker` is null or a live handle; `out_sense` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_dag_surface_sense(tracker: *const SmrtTracker, surface: i64, region: i64, out_sense: *mut i32) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (t, out) = unsafe { (shared(tracker, "tracker")?, exclusive(out_sense, "out_sense")?) };
        let s = surface_in(surface)?;
        let r = region_in(region, t.complement)?;
        if t.inner.surfaces().get(s).is_none() {
            return Err(sm_rt_api::RtError::UnknownId { kind: "surface", id: s.0 }.into());
        }
        match compat::surface_sense(&*t.inner, s, r) {
            Some(v) => {
                *out = v;
                Ok(SMRT_OK)
            }
            None => {
                *out = 0;
                Ok(SMRT_NO_HIT)
            }
        }
    })
}

/// DAGMC `get_angle`: the unit normal of face `face` at `p[3]`, pointing out of its front region.
/// `face` is the face id reported by `smrt_step`, `smrt_fire` or `smrt_dag_ray_fire` (`out_face`);
/// a tracker may number faces differently from surfaces (the stub does not). Returns `0`.
///
/// # Safety
/// `tracker` is null or a live handle; `p` three readable doubles; `out_normal` three writable
/// doubles.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_dag_get_angle(tracker: *const SmrtTracker, face: i64, p: *const f64, out_normal: *mut f64) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (t, p) = unsafe { (shared(tracker, "tracker")?, read3(p, "p")?) };
        if out_normal.is_null() {
            return Err(Fail::Null("out_normal"));
        }
        let n = compat::get_angle(&*t.inner, face_in(face)?, point(p))?;
        // SAFETY: `out_normal` is non-null and valid for three doubles by the caller contract.
        unsafe { write3(out_normal, arr(n), "out_normal")? };
        Ok(SMRT_OK)
    })
}

/// DAGMC `measure_volume`: the volume of `region`, the sum of its cell volumes in index order
/// (NaN, with status `0`, when a cell volume is unknown). Returns `0`.
///
/// # Safety
/// `tracker` is null or a live handle; `out_volume` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_dag_measure_volume(tracker: *const SmrtTracker, region: i64, out_volume: *mut f64) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (t, out) = unsafe { (shared(tracker, "tracker")?, exclusive(out_volume, "out_volume")?) };
        let r = region_in(region, t.complement)?;
        if t.inner.regions().get(r).is_none() {
            return Err(sm_rt_api::RtError::UnknownId { kind: "region", id: r.0 }.into());
        }
        *out = t.region_measures().get(r.get()).copied().unwrap_or(f64::NAN);
        Ok(SMRT_OK)
    })
}

/// DAGMC `measure_area`. The contract has no per-surface area table, so this returns
/// `SMRT_ERR_UNSUPPORTED` (`-5`) for every valid surface (see the api change request in
/// `W3_REPORT.md`). Unknown surfaces are `SMRT_ERR_UNKNOWN_ID`.
///
/// # Safety
/// `tracker` is null or a live handle; `out_area` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_dag_measure_area(tracker: *const SmrtTracker, surface: i64, out_area: *mut f64) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (t, out) = unsafe { (shared(tracker, "tracker")?, exclusive(out_area, "out_area")?) };
        let s = surface_in(surface)?;
        if t.inner.surfaces().get(s).is_none() {
            return Err(sm_rt_api::RtError::UnknownId { kind: "surface", id: s.0 }.into());
        }
        *out = f64::NAN;
        Err(sm_rt_api::RtError::Unsupported("surface areas are not in the tracker contract").into())
    })
}
