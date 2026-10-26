//! The C ABI: handles, the panic/error guard and the pointer helpers. This module tree is the
//! only place in the crate with `unsafe` code; every block names the invariant it relies on.
//!
//! Entry-point rules (all `smrt_*` functions):
//! * the first thing is [`guard`]: no panic crosses the boundary, a panic becomes
//!   `SMRT_ERR_PANIC` and the message goes to this thread's slot;
//! * every pointer argument is null-checked before use (`SMRT_ERR_NULL`);
//! * a [`SmrtCtx`] is used by one thread at a time (documented, not locked); a [`SmrtTracker`]
//!   is shared read-only by any number of threads.

use crate::status::{Fail, SMRT_ERR_PANIC, panic_message, set_last_error};
use sm_rt_api::{CellId, RegionId, Scratch, Tracker, TrackState};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::{Arc, OnceLock};

mod bundle;
mod dag;
mod info;
mod track;

pub use bundle::*;
pub use dag::*;
pub use info::*;
pub use track::*;

#[cfg(feature = "stubs")]
pub mod test_hooks;

/// Opaque shared tracker handle (`SmrtTracker` in C): an `Arc<dyn Tracker>`, read-only.
pub struct SmrtTracker {
    pub(crate) inner: Arc<dyn Tracker>,
    pub(crate) complement: Option<RegionId>,
    measures: OnceLock<Vec<f64>>,
}

impl SmrtTracker {
    pub(crate) fn new(inner: Arc<dyn Tracker>) -> Self {
        let complement = inner.regions().implicit_complement();
        Self { inner, complement, measures: OnceLock::new() }
    }

    /// Volume per region: the sum of `cell_volume` over the region's cells in index order (a
    /// fixed order, so the sums are reproducible); NaN for a region with an unknown cell volume.
    /// Computed once, on first use.
    pub(crate) fn region_measures(&self) -> &[f64] {
        self.measures.get_or_init(|| {
            let t = &*self.inner;
            let mut m = vec![0.0; t.regions().0.len()];
            for c in 0..t.cell_count() {
                let id = CellId::new(c);
                let Some(r) = t.cell_region(id).filter(|r| r.get() < m.len()) else { continue };
                match t.cell_volume(id) {
                    Some(v) => m[r.get()] += v,
                    None => m[r.get()] = f64::NAN,
                }
            }
            m
        })
    }
}

/// Opaque per-thread context (`SmrtCtx` in C): the particle state and the query scratch.
pub struct SmrtCtx {
    pub(crate) tracker: Arc<dyn Tracker>,
    pub(crate) complement: Option<RegionId>,
    pub(crate) scratch: Scratch,
    pub(crate) st: TrackState,
}

/// Wrap a tracker in a C handle (for embedding Rust code and tests). The caller owns the
/// pointer and releases it with `smrt_tracker_free`. Never null.
///
/// ```
/// use std::sync::Arc;
/// use sm_core::{Aabb, Point3};
/// use sm_rt_api::stub::AnalyticWorld;
/// use sm_rt_api::{ResponseKind, Tracker};
///
/// let bounds = Aabb { lo: Point3::new(-1.0, -1.0, -1.0), hi: Point3::new(1.0, 1.0, 1.0) };
/// let world = AnalyticWorld::simple(vec![], bounds, ResponseKind::Vacuum).unwrap();
/// let t: Arc<dyn Tracker> = Arc::new(world);
/// let h = sm_rt_capi::tracker_into_raw(t);
/// assert!(!h.is_null());
/// // SAFETY: `h` came from `tracker_into_raw` and is freed exactly once.
/// assert_eq!(unsafe { sm_rt_capi::ffi::smrt_tracker_free(h) }, 0);
/// ```
pub fn tracker_into_raw(t: Arc<dyn Tracker>) -> *mut SmrtTracker {
    Box::into_raw(Box::new(SmrtTracker::new(t)))
}

/// Run an entry point body: panics become `SMRT_ERR_PANIC`, errors become their code, and in
/// both cases the message is stored for `smrt_last_error_message`. The success path touches no
/// thread-local state and does not allocate.
pub(crate) fn guard(f: impl FnOnce() -> Result<i32, Fail>) -> i32 {
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(Ok(code)) => code,
        Ok(Err(e)) => {
            set_last_error(&e.message());
            e.code()
        }
        Err(payload) => {
            set_last_error(&panic_message(&*payload));
            // Dropping a payload that itself panics would abort; that cannot be helped.
            drop(payload);
            SMRT_ERR_PANIC
        }
    }
}

/// Shared reference to a handle pointer.
///
/// # Safety
/// `p` is null or points to a live value of `T` that is not mutated for `'a`.
pub(crate) unsafe fn shared<'a, T>(p: *const T, name: &'static str) -> Result<&'a T, Fail> {
    // SAFETY: forwarded from this function's contract; `as_ref` handles null.
    unsafe { p.as_ref() }.ok_or(Fail::Null(name))
}

/// Exclusive reference to a handle or output pointer.
///
/// # Safety
/// `p` is null or points to a live value of `T` that nothing else accesses for `'a`.
pub(crate) unsafe fn exclusive<'a, T>(p: *mut T, name: &'static str) -> Result<&'a mut T, Fail> {
    // SAFETY: forwarded from this function's contract; `as_mut` handles null.
    unsafe { p.as_mut() }.ok_or(Fail::Null(name))
}

/// Read three doubles.
///
/// # Safety
/// `p` is null or points to at least three readable, aligned `f64`.
pub(crate) unsafe fn read3(p: *const f64, name: &'static str) -> Result<[f64; 3], Fail> {
    // SAFETY: caller guarantees three readable `f64` behind a non-null `p`.
    let s = unsafe { shared(p.cast::<[f64; 3]>(), name) }?;
    Ok(*s)
}

/// Read three doubles when the pointer is non-null.
///
/// # Safety
/// Same as [`read3`] when `p` is non-null.
pub(crate) unsafe fn read3_opt(p: *const f64, name: &'static str) -> Result<Option<[f64; 3]>, Fail> {
    if p.is_null() {
        return Ok(None);
    }
    // SAFETY: `p` is non-null here; the rest is the caller's contract.
    unsafe { read3(p, name) }.map(Some)
}

/// Write three doubles.
///
/// # Safety
/// `p` is null or points to at least three writable, aligned `f64`.
pub(crate) unsafe fn write3(p: *mut f64, v: [f64; 3], name: &'static str) -> Result<(), Fail> {
    // SAFETY: caller guarantees three writable `f64` behind a non-null `p`.
    let s = unsafe { exclusive(p.cast::<[f64; 3]>(), name) }?;
    *s = v;
    Ok(())
}

/// The `SmrtCtx` and `SmrtTracker` pointers of a shim must describe one tracker.
pub(crate) fn same_tracker(t: &SmrtTracker, c: &SmrtCtx) -> Result<(), Fail> {
    if std::ptr::eq(Arc::as_ptr(&t.inner).cast::<()>(), Arc::as_ptr(&c.tracker).cast::<()>()) {
        Ok(())
    } else {
        Err(Fail::bad("ctx was created from a different tracker"))
    }
}
