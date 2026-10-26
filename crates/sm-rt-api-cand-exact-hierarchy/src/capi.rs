//! C ABI sketch for Fortran/C++ transport codes (feature `capi`).
//!
//! Model: an opaque, immutable, shareable `SmrtTracker` and one opaque `SmrtCtx` per thread
//! (it owns the `Scratch` and the current `TrackState`, so the caller never sees them). All
//! functions return an [`SmrtStatus`]; none unwinds across the boundary for finite input, and a
//! lost particle is `SMRT_LOST`, not an abort. Strings and ids are plain `u64`/`f64`; arrays are
//! caller-owned pointers to 3 doubles.
//!
//! Equivalent C header (informative):
//! ```c
//! typedef struct SmrtTracker SmrtTracker;   typedef struct SmrtCtx SmrtCtx;
//! typedef struct { double distance; int32_t event; uint64_t face; uint64_t region; double inward[3]; } SmrtStep;
//! void smrt_tracker_free(SmrtTracker*);
//! SmrtCtx* smrt_ctx_new(const SmrtTracker*);   void smrt_ctx_free(SmrtCtx*);
//! int32_t smrt_locate(SmrtCtx*, const double pos[3], const double* dir /*nullable*/, uint64_t* region);
//! int32_t smrt_step(SmrtCtx*, double limit, SmrtStep* out);
//! int32_t smrt_state(const SmrtCtx*, double pos[3], double dir[3]);
//! ```
//! A Fortran binding uses `bind(C)` with the same names; ids are `integer(c_int64_t)`.

use crate::tracker::Tracker;
use crate::types::{Event, Scratch, TrackState};
use sm_core::{Point3, Vector3};
use sm_topo_api::Region;
use std::sync::Arc;

/// Return codes.
#[repr(i32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SmrtStatus {
    /// Success.
    Ok = 0,
    /// The particle is lost (details through a future `smrt_last_lost`).
    Lost = 1,
    /// Invalid input (non-finite numbers, no state yet).
    Invalid = 2,
    /// Unsupported by this implementor.
    Unsupported = 3,
    /// Geometry error.
    Geom = 4,
    /// Cache error.
    Cache = 5,
    /// A required pointer was null.
    NullArg = -1,
}

/// Event codes of [`SmrtStep::event`].
#[repr(i32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SmrtEvent {
    /// Crossed into another region.
    Crossing = 0,
    /// Slot or instance change.
    SlotChange = 1,
    /// Deck reflect or wrap applied.
    Deck = 2,
    /// Vacuum boundary: kill.
    Vacuum = 3,
    /// White boundary: caller re-emits.
    White = 4,
    /// Albedo boundary: caller reflects and scales.
    Albedo = 5,
    /// Consumer-owned reflection.
    Reflective = 6,
    /// Distance limit reached.
    Limit = 7,
    /// Lost.
    Lost = 8,
}

/// Opaque tracker handle.
pub struct SmrtTracker {
    inner: Arc<dyn Tracker>,
}

/// Opaque per-thread context.
pub struct SmrtCtx {
    tracker: Arc<dyn Tracker>,
    scratch: Scratch,
    state: Option<TrackState>,
}

/// Result of one step, `#[repr(C)]`.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SmrtStep {
    /// Distance travelled.
    pub distance: f64,
    /// An [`SmrtEvent`] code.
    pub event: i32,
    /// Raw id of the face involved, 0 if none.
    pub face: u64,
    /// Raw id of the volume entered (0 = implicit complement).
    pub region: u64,
    /// Inward normal for response events, else zeros.
    pub inward: [f64; 3],
}

/// Wrap a Rust tracker for C callers. The returned pointer is owned by the caller and released
/// with [`smrt_tracker_free`].
pub fn smrt_tracker_from(t: Arc<dyn Tracker>) -> *mut SmrtTracker {
    Box::into_raw(Box::new(SmrtTracker { inner: t }))
}

/// Release a tracker handle (contexts made from it keep their own reference). Null is a no-op.
///
/// # Safety
/// `t` must be null or a pointer from [`smrt_tracker_from`] that has not been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_tracker_free(t: *mut SmrtTracker) {
    if !t.is_null() {
        // SAFETY: by the contract above `t` came from `Box::into_raw` and is freed once.
        drop(unsafe { Box::from_raw(t) });
    }
}

/// Create a context for one thread. Returns null if `t` is null.
///
/// # Safety
/// `t` must be null or a live tracker handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_ctx_new(t: *const SmrtTracker) -> *mut SmrtCtx {
    // SAFETY: caller guarantees `t` is null or live.
    match unsafe { t.as_ref() } {
        Some(t) => Box::into_raw(Box::new(SmrtCtx { tracker: t.inner.clone(), scratch: Scratch::new(), state: None })),
        None => std::ptr::null_mut(),
    }
}

/// Release a context. Null is a no-op.
///
/// # Safety
/// `c` must be null or a pointer from [`smrt_ctx_new`] not yet freed, with no concurrent use.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_ctx_free(c: *mut SmrtCtx) {
    if !c.is_null() {
        // SAFETY: see the contract.
        drop(unsafe { Box::from_raw(c) });
    }
}

fn region_raw(r: Region) -> u64 {
    match r {
        Region::Volume(v) => v.to_raw(),
        _ => 0,
    }
}

fn status_of(e: &crate::RtError) -> SmrtStatus {
    match e {
        crate::RtError::Lost(_) => SmrtStatus::Lost,
        crate::RtError::Input(_) => SmrtStatus::Invalid,
        crate::RtError::Unsupported(_) => SmrtStatus::Unsupported,
        crate::RtError::Geom(_) => SmrtStatus::Geom,
        _ => SmrtStatus::Cache,
    }
}

/// Place the particle at `pos` (3 doubles) with optional direction `dir` (3 doubles, may be
/// null), and write the raw id of its volume to `region` (0 = implicit complement).
///
/// # Safety
/// `c` is a live context; `pos` points to 3 readable doubles; `dir` is null or points to 3
/// readable doubles; `region` is null or points to one writable `u64`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_locate(c: *mut SmrtCtx, pos: *const f64, dir: *const f64, region: *mut u64) -> i32 {
    // SAFETY: pointers are valid per the contract.
    let (Some(c), false) = (unsafe { c.as_mut() }, pos.is_null()) else { return SmrtStatus::NullArg as i32 };
    // SAFETY: `pos` has 3 doubles, `dir` is null or has 3.
    let (p, d) = unsafe { (std::slice::from_raw_parts(pos, 3), (!dir.is_null()).then(|| std::slice::from_raw_parts(dir, 3))) };
    let dv = d.map(|d| Vector3::new(d[0], d[1], d[2]));
    match c.tracker.locate(Point3::new(p[0], p[1], p[2]), dv, &mut c.scratch) {
        Ok(s) => {
            if !region.is_null() {
                // SAFETY: non-null and writable per the contract.
                unsafe { *region = region_raw(s.at.region) };
            }
            c.state = Some(s);
            SmrtStatus::Ok as i32
        }
        Err(e) => status_of(&e) as i32,
    }
}

/// Advance to the next face or `limit`, applying deck reflection/wrapping in place.
///
/// # Safety
/// `c` is a live context that has been located; `out` points to one writable [`SmrtStep`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_step(c: *mut SmrtCtx, limit: f64, out: *mut SmrtStep) -> i32 {
    // SAFETY: valid per the contract.
    let (Some(c), Some(out)) = (unsafe { c.as_mut() }, unsafe { out.as_mut() }) else { return SmrtStatus::NullArg as i32 };
    let Some(st) = c.state.as_mut() else { return SmrtStatus::Invalid as i32 };
    use crate::ResponseKind as R;
    match c.tracker.step(st, limit, &mut c.scratch) {
        Ok(step) => {
            let mut o = SmrtStep { distance: step.distance, event: SmrtEvent::Limit as i32, face: 0, region: region_raw(st.at.region), inward: [0.0; 3] };
            match step.event {
                Event::Crossing { face, .. } => (o.event, o.face) = (SmrtEvent::Crossing as i32, face.to_raw()),
                Event::SlotChange { .. } => o.event = SmrtEvent::SlotChange as i32,
                Event::Deck { face, .. } => (o.event, o.face) = (SmrtEvent::Deck as i32, face.to_raw()),
                Event::Response { face, kind, inward } => {
                    o.face = face.to_raw();
                    o.inward = [inward.x, inward.y, inward.z];
                    o.event = match kind {
                        R::Vacuum => SmrtEvent::Vacuum,
                        R::White => SmrtEvent::White,
                        R::Albedo(_) => SmrtEvent::Albedo,
                        _ => SmrtEvent::Reflective,
                    } as i32;
                }
                Event::Lost(_) => o.event = SmrtEvent::Lost as i32,
                _ => {}
            }
            *out = o;
            SmrtStatus::Ok as i32
        }
        Err(e) => status_of(&e) as i32,
    }
}

/// Read back the current position and direction (3 doubles each).
///
/// # Safety
/// `c` is a live context; `pos` and `dir` each point to 3 writable doubles.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_state(c: *const SmrtCtx, pos: *mut f64, dir: *mut f64) -> i32 {
    // SAFETY: valid per the contract.
    let Some(c) = (unsafe { c.as_ref() }) else { return SmrtStatus::NullArg as i32 };
    let Some(s) = c.state.as_ref() else { return SmrtStatus::Invalid as i32 };
    if pos.is_null() || dir.is_null() {
        return SmrtStatus::NullArg as i32;
    }
    // SAFETY: both point to 3 writable doubles.
    unsafe {
        std::slice::from_raw_parts_mut(pos, 3).copy_from_slice(&[s.pos.x, s.pos.y, s.pos.z]);
        std::slice::from_raw_parts_mut(dir, 3).copy_from_slice(&[s.dir.x, s.dir.y, s.dir.z]);
    }
    SmrtStatus::Ok as i32
}
