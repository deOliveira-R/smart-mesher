//! Tracker lifetime, introspection, `smrt_open`, version and the thread-local error message.

use super::{SmrtTracker, exclusive, guard, shared};
use crate::convert::region_in;
use crate::status::*;
use crate::types::*;
use std::ffi::{CStr, c_char};

/// Free a tracker handle. Contexts made from it keep the tracker alive until they are freed.
/// Returns `0`.
///
/// # Safety
/// `tracker` is null or a handle from this library (`smrt_open` or `tracker_into_raw`) not
/// freed before.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_tracker_free(tracker: *mut SmrtTracker) -> i32 {
    guard(|| {
        if tracker.is_null() {
            return Err(Fail::Null("tracker"));
        }
        // SAFETY: non-null, and by the caller contract created by `Box::into_raw` in
        // `tracker_into_raw` and freed exactly once.
        drop(unsafe { Box::from_raw(tracker) });
        Ok(SMRT_OK)
    })
}

/// Open a `.smrt` tracker image. Without the `sm-rt` cargo feature (the W3a state) no loader is
/// linked and this returns `SMRT_ERR_UNSUPPORTED` (`-5`) with an explanatory message; the
/// loader is wired when sm-rt is on main. On success `*out` is a tracker handle.
///
/// # Safety
/// `path` is null or a NUL-terminated string; `out` is null or valid for one write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_open(path: *const c_char, out: *mut *mut SmrtTracker) -> i32 {
    guard(|| {
        if path.is_null() {
            return Err(Fail::Null("path"));
        }
        // SAFETY: caller contract above.
        let out = unsafe { exclusive(out, "out")? };
        *out = std::ptr::null_mut();
        // SAFETY: `path` is non-null and NUL-terminated by the caller contract.
        let _path = unsafe { CStr::from_ptr(path) };
        let msg = if cfg!(feature = "sm-rt") {
            "smrt_open: the `sm-rt` feature is on but no loader is wired in this build"
        } else {
            "smrt_open: no tracker backend linked (build sm-rt-capi with the `sm-rt` feature)"
        };
        Err(Fail::Rt(sm_rt_api::RtError::Unsupported(msg)))
    })
}

/// Build-time facts of a tracker. Returns `0`.
///
/// # Safety
/// `tracker` is null or a live handle; `out` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_info(tracker: *const SmrtTracker, out: *mut SmrtInfo) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (t, out) = unsafe { (shared(tracker, "tracker")?, exclusive(out, "out")?) };
        let i = t.inner.info();
        let n = |v: usize| i64::try_from(v).unwrap_or(i64::MAX);
        let h = i.structure_hash.0;
        *out = SmrtInfo {
            cells: n(i.cells),
            faces: n(i.faces),
            regions: n(t.inner.regions().0.len()),
            surfaces: n(t.inner.surfaces().0.len()),
            prototypes: n(i.prototypes),
            copies: n(i.copies),
            memory_bytes: n(i.memory_bytes),
            exact_faces: n(i.exact_faces),
            faceted_faces: n(i.faceted_faces),
            caps: i64::from(t.inner.caps().0),
            complement_region: t.complement.map_or(SMRT_NONE, |r| i64::from(r.0)),
            max_events: i64::from(i.max_events),
            structure_hash_lo: h as u64,
            structure_hash_hi: (h >> 64) as u64,
            bounds_lo: [i.bounds.lo.x, i.bounds.lo.y, i.bounds.lo.z],
            bounds_hi: [i.bounds.hi.x, i.bounds.hi.y, i.bounds.hi.z],
            tol_linear: i.tol.linear,
            tol_angular: i.tol.angular,
        };
        Ok(SMRT_OK)
    })
}

/// Facts of one region: model entity serial, colour id (always `-1`), volume (cached, summed over
/// the region's cells in index order; NaN when unknown) and the complement flag. `region` may be
/// `-2` for the implicit complement. Returns `0`.
///
/// # Safety
/// `tracker` is null or a live handle; `out` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_region_info(tracker: *const SmrtTracker, region: i64, out: *mut SmrtRegionInfo) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (t, out) = unsafe { (shared(tracker, "tracker")?, exclusive(out, "out")?) };
        let r = region_in(region, t.complement)?;
        let info = t.inner.regions().get(r).ok_or(sm_rt_api::RtError::UnknownId { kind: "region", id: r.0 })?;
        *out = SmrtRegionInfo {
            original_id: info.entity.map_or(SMRT_NONE, |e| i64::try_from(e.serial()).unwrap_or(SMRT_NONE)),
            color_id: SMRT_NONE,
            measure: t.region_measures().get(r.get()).copied().unwrap_or(f64::NAN),
            implicit_complement: i64::from(info.implicit_complement),
        };
        Ok(SMRT_OK)
    })
}

/// ABI version as `major * 10000 + minor * 100 + patch`.
#[unsafe(no_mangle)]
pub extern "C" fn smrt_version() -> i32 {
    SMRT_ABI_VERSION
}

/// Copy this thread's last error message into `buf` (NUL-terminated, truncated to `cap` bytes)
/// and return the full message length in bytes without the NUL (`snprintf` semantics); `0` means
/// no message. `buf` may be null when `cap` is `0`, to ask for the length. The message belongs
/// to the calling thread: another thread's failures never change it, and successful calls do not
/// clear it.
///
/// # Safety
/// `buf` is null or valid for `cap` bytes of writes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_last_error_message(buf: *mut c_char, cap: i64) -> i32 {
    guard(|| {
        let cap = usize::try_from(cap).map_err(|_| Fail::bad("negative buffer capacity"))?;
        let len = if cap == 0 {
            last_error().len()
        } else {
            if buf.is_null() {
                return Err(Fail::Null("buf"));
            }
            // SAFETY: `buf` is non-null and valid for `cap` bytes by the caller contract; `u8`
            // and `c_char` have the same layout.
            let dst = unsafe { std::slice::from_raw_parts_mut(buf.cast::<u8>(), cap) };
            copy_last_error(dst)
        };
        Ok(i32::try_from(len).unwrap_or(i32::MAX))
    })
}
