//! C ABI sketch for Fortran/C++ callers (feature `capi`). Opaque handles, flat `f64[3]`
//! arrays, integer status codes. The host (Rust) creates a world and hands the caller a
//! `SmRtWorld*`; there is no backend registry in the ABI yet (`sm_rt_world_open` is a TODO for
//! the real crate: it will take a `.smsh` path and a backend name).
//!
//! Status: `0` ok, `1` no hit / false, negative = [`crate::RtError::code`], `-100` null
//! pointer, `-101` panic caught. Handles are 32-bit ints, volumes and surfaces as in
//! [`crate::VolHandle`]. Every function is thread-safe on a shared world; a history belongs to
//! one particle.
//!
//! Fortran (iso_c_binding) shape of the main call:
//! ```fortran
//! integer(c_int) function sm_rt_ray_fire(w, vol, org, dir, hist, limit, orient, hit) bind(C)
//!   type(c_ptr), value :: w, hist
//!   integer(c_int32_t), value :: vol, orient
//!   real(c_double) :: org(3), dir(3)
//!   real(c_double), value :: limit          ! 0 = unlimited, like DAGMC
//!   type(sm_rt_hit) :: hit                  ! bind(C): surface, facet, dist
//! ```
#![allow(unsafe_code)]

use crate::error::RtError;
use crate::handle::{SurfHandle, VolHandle};
use crate::query::{FireOptions, RayHistory, RayOrientation};
use crate::world::World;
use sm_core::{Point3, Vector3};
use std::panic::{AssertUnwindSafe, catch_unwind};

/// Opaque world handle.
pub struct SmRtWorld(Box<dyn World>);
/// Opaque history handle.
pub struct SmRtHistory(RayHistory);

/// Result of a ray fire.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct SmRtHit {
    /// Surface handle.
    pub surface: u32,
    /// Facet index within the surface.
    pub facet: u32,
    /// Distance.
    pub dist: f64,
}

const NULL: i32 = -100;
const PANIC: i32 = -101;

/// Wrap a world for the C side. The caller owns the result and frees it with
/// [`sm_rt_world_free`].
pub fn world_into_raw(w: Box<dyn World>) -> *mut SmRtWorld {
    Box::into_raw(Box::new(SmRtWorld(w)))
}

fn guard(f: impl FnOnce() -> i32) -> i32 {
    catch_unwind(AssertUnwindSafe(f)).unwrap_or(PANIC)
}

fn pt(p: *const f64) -> Option<Point3> {
    // SAFETY: caller passes a valid pointer to 3 doubles (checked for null here).
    (!p.is_null()).then(|| unsafe { Point3::new(*p, *p.add(1), *p.add(2)) })
}

fn vec3(p: *const f64) -> Option<Vector3> {
    pt(p).map(|q| Vector3::new(q.x, q.y, q.z))
}

/// Free a world.
///
/// # Safety
/// `w` must come from [`world_into_raw`] and not be used afterwards (null is allowed).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sm_rt_world_free(w: *mut SmRtWorld) {
    if !w.is_null() {
        // SAFETY: per contract, `w` is a live Box from `world_into_raw`.
        drop(unsafe { Box::from_raw(w) });
    }
}

/// New empty history.
#[unsafe(no_mangle)]
pub extern "C" fn sm_rt_history_new() -> *mut SmRtHistory {
    Box::into_raw(Box::new(SmRtHistory(RayHistory::new())))
}

/// Free a history (null allowed).
///
/// # Safety
/// `h` must come from [`sm_rt_history_new`] and not be used afterwards.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sm_rt_history_free(h: *mut SmRtHistory) {
    if !h.is_null() {
        // SAFETY: per contract.
        drop(unsafe { Box::from_raw(h) });
    }
}

/// Clear a history. `mode`: 0 reset, 1 keep last intersection, 2 roll back last intersection.
///
/// # Safety
/// `h` must be a live history.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sm_rt_history_reset(h: *mut SmRtHistory, mode: i32) -> i32 {
    // SAFETY: per contract.
    let Some(h) = (unsafe { h.as_mut() }) else { return NULL };
    match mode {
        0 => h.0.reset(),
        1 => h.0.reset_to_last_intersection(),
        _ => h.0.rollback_last_intersection(),
    }
    0
}

/// DAGMC `ray_fire`. `orient`: 1 exiting, -1 entering, 0 any. `limit <= 0` is unlimited. `hist`
/// may be null. Returns 0 on hit (`out` filled), 1 on no hit.
///
/// # Safety
/// All pointers must be valid (`hist` may be null); `org` and `dir` point to 3 doubles.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sm_rt_ray_fire(
    w: *const SmRtWorld,
    vol: u32,
    org: *const f64,
    dir: *const f64,
    hist: *mut SmRtHistory,
    limit: f64,
    orient: i32,
    out: *mut SmRtHit,
) -> i32 {
    guard(|| {
        // SAFETY: per contract.
        let (Some(w), Some(o), Some(d), Some(out)) =
            (unsafe { w.as_ref() }, pt(org), vec3(dir), unsafe { out.as_mut() })
        else {
            return NULL;
        };
        // SAFETY: per contract; null means "no history".
        let h = unsafe { hist.as_mut() }.map(|h| &mut h.0);
        let opts = FireOptions {
            dist_limit: (limit > 0.0).then_some(limit),
            orientation: match orient {
                -1 => RayOrientation::Entering,
                0 => RayOrientation::Any,
                _ => RayOrientation::Exiting,
            },
        };
        match w.0.ray_fire(VolHandle(vol), o, d, opts, h) {
            Ok(Some(hit)) => {
                *out = SmRtHit { surface: hit.surface.0, facet: hit.facet.index, dist: hit.dist };
                0
            }
            Ok(None) => 1,
            Err(e) => e.code(),
        }
    })
}

/// DAGMC `point_in_volume`. `dir` may be null. Returns 0 outside, 1 inside, negative on error.
///
/// # Safety
/// Pointers must be valid; `dir` may be null; `p` and `dir` point to 3 doubles.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sm_rt_point_in_volume(w: *const SmRtWorld, vol: u32, p: *const f64, dir: *const f64) -> i32 {
    guard(|| {
        // SAFETY: per contract.
        let (Some(w), Some(p)) = (unsafe { w.as_ref() }, pt(p)) else { return NULL };
        match w.0.point_in_volume(VolHandle(vol), p, vec3(dir), None) {
            Ok(b) => i32::from(b),
            Err(e) => e.code(),
        }
    })
}

/// DAGMC `next_vol`; writes the volume on the other side of `surf` from `old`.
///
/// # Safety
/// `w` and `out` must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sm_rt_next_vol(w: *const SmRtWorld, surf: u32, old: u32, out: *mut u32) -> i32 {
    guard(|| {
        // SAFETY: per contract.
        let (Some(w), Some(out)) = (unsafe { w.as_ref() }, unsafe { out.as_mut() }) else { return NULL };
        match w.0.next_vol(SurfHandle(surf), VolHandle(old)) {
            Ok(v) => {
                *out = v.0;
                0
            }
            Err(e) => e.code(),
        }
    })
}

/// DAGMC `surface_sense`: writes `+1` or `-1`.
///
/// # Safety
/// `w` and `out` must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sm_rt_surface_sense(w: *const SmRtWorld, vol: u32, surf: u32, out: *mut i32) -> i32 {
    guard(|| {
        // SAFETY: per contract.
        let (Some(w), Some(out)) = (unsafe { w.as_ref() }, unsafe { out.as_mut() }) else { return NULL };
        match w.0.surface_sense(VolHandle(vol), SurfHandle(surf)) {
            Ok(s) => {
                *out = s.as_i32();
                0
            }
            Err(e) => e.code(),
        }
    })
}

/// DAGMC `get_angle`: writes the unit normal (3 doubles).
///
/// # Safety
/// Pointers must be valid; `hist` may be null; `p` and `out` point to 3 doubles.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sm_rt_get_angle(
    w: *const SmRtWorld,
    surf: u32,
    p: *const f64,
    hist: *const SmRtHistory,
    out: *mut f64,
) -> i32 {
    guard(|| {
        // SAFETY: per contract.
        let (Some(w), Some(p)) = (unsafe { w.as_ref() }, pt(p)) else { return NULL };
        if out.is_null() {
            return NULL;
        }
        // SAFETY: per contract.
        let h = unsafe { hist.as_ref() }.map(|h| &h.0);
        match w.0.get_angle(SurfHandle(surf), p, h) {
            Ok(n) => {
                // SAFETY: `out` is non-null and points to 3 doubles.
                unsafe {
                    *out = n.x;
                    *out.add(1) = n.y;
                    *out.add(2) = n.z;
                }
                0
            }
            Err(e) => e.code(),
        }
    })
}

/// DAGMC `measure_volume` (`what = 0`) or `measure_area` (`what = 1`).
///
/// # Safety
/// `w` and `out` must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sm_rt_measure(w: *const SmRtWorld, what: i32, entity: u32, out: *mut f64) -> i32 {
    guard(|| {
        // SAFETY: per contract.
        let (Some(w), Some(out)) = (unsafe { w.as_ref() }, unsafe { out.as_mut() }) else { return NULL };
        let r: Result<f64, RtError> =
            if what == 0 { w.0.measure_volume(VolHandle(entity)) } else { w.0.measure_area(SurfHandle(entity)) };
        match r {
            Ok(x) => {
                *out = x;
                0
            }
            Err(e) => e.code(),
        }
    })
}
