//! C ABI sketch for Fortran / C++ transport codes.
//!
//! Design rules: opaque handle, plain `repr(C)` structs of `f64`/`u32`/`i32` (no bool, no enums,
//! no pointers inside structs) so `iso_c_binding` can mirror them; every function returns an
//! `i32` status (0 = ok) and never unwinds; the caller owns its [`SmrtState`] (one per
//! particle, thread-private) while the tracker handle is shared and read-only, so threads call
//! it concurrently without locks. Ids are `u32` with `0xFFFFFFFF` for none.
//!
//! Fortran view (`bind(C)`):
//! ```text
//! type, bind(C) :: smrt_state
//!   real(c_double) :: pos(3), dir(3)
//!   integer(c_int32_t) :: cell, region, entered_via, n_levels   ! unsigned in C; compare to -1
//!   type(smrt_level) :: levels(4)
//! end type
//! function smrt_step(h, st, max_dist, out) bind(C, name="smrt_step")
//! ```
//! The header text is in [`C_HEADER`].

use crate::ids::{CellId, FaceId, RegionId, SurfaceId};
use crate::state::{BoundaryKind, Level, StepEnd, TrackState};
use crate::tracker::{BoundaryAction, Tracker};
use sm_core::{Point3, Vector3};
use std::ffi::{CStr, c_char};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::{Mutex, OnceLock};

/// Success.
pub const SMRT_OK: i32 = 0;
/// A pointer argument was null or an argument was out of range.
pub const SMRT_E_ARG: i32 = -1;
/// A typed error from the tracker; see `smrt_last_error_message`.
pub const SMRT_E_RT: i32 = -2;
/// A panic was caught at the boundary (a bug).
pub const SMRT_E_PANIC: i32 = -3;
/// No opener registered / unsupported.
pub const SMRT_E_UNSUPPORTED: i32 = -4;

/// `end_kind` values of [`SmrtStep`].
pub const SMRT_END_LIMIT: i32 = 0;
/// Crossed a face into a cell of the same region.
pub const SMRT_END_CELL_FACE: i32 = 1;
/// Crossed into another region.
pub const SMRT_END_REGION: i32 = 2;
/// Reached a domain boundary.
pub const SMRT_END_DOMAIN: i32 = 3;
/// Lost.
pub const SMRT_END_LOST: i32 = 4;

/// `bc_kind` values.
pub const SMRT_BC_VACUUM: i32 = 0;
/// Reflective.
pub const SMRT_BC_REFLECTIVE: i32 = 1;
/// White.
pub const SMRT_BC_WHITE: i32 = 2;
/// Periodic.
pub const SMRT_BC_PERIODIC: i32 = 3;

/// Opaque tracker handle. Created by an opener (`smrt_open`) or [`into_handle`]; freed by
/// `smrt_tracker_free`.
pub struct SmrtTracker(Box<dyn Tracker>);

/// One hierarchy level.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SmrtLevel {
    /// Component id.
    pub component: u32,
    /// Instance id.
    pub instance: u32,
    /// Lattice index.
    pub lattice: [i32; 3],
}

/// Particle geometry state (mirror of [`TrackState`]).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmrtState {
    /// Position.
    pub pos: [f64; 3],
    /// Unit direction.
    pub dir: [f64; 3],
    /// Cell id.
    pub cell: u32,
    /// Region id.
    pub region: u32,
    /// Entry face id (`u32::MAX` if none).
    pub entered_via: u32,
    /// Number of valid `levels`.
    pub n_levels: u32,
    /// Hierarchy path.
    pub levels: [SmrtLevel; crate::state::MAX_LEVELS],
}

/// One step result.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmrtStep {
    /// Distance travelled.
    pub dist: f64,
    /// Cell traversed.
    pub cell: u32,
    /// Region traversed.
    pub region: u32,
    /// One of the `SMRT_END_*` codes.
    pub end_kind: i32,
    /// Surface hit (`u32::MAX` if none).
    pub surface: u32,
    /// Face hit.
    pub face: u32,
    /// Region entered.
    pub to_region: u32,
    /// Cell entered.
    pub to_cell: u32,
    /// `SMRT_BC_*` for a domain end.
    pub bc_kind: i32,
    /// Periodic partner surface.
    pub partner: u32,
}

impl From<&SmrtState> for TrackState {
    fn from(s: &SmrtState) -> Self {
        let mut t = TrackState::new(Point3::new(s.pos[0], s.pos[1], s.pos[2]), Vector3::new(s.dir[0], s.dir[1], s.dir[2]));
        t.cell = CellId(s.cell);
        t.region = RegionId(s.region);
        t.entered_via = (s.entered_via != u32::MAX).then_some(FaceId(s.entered_via));
        let n = (s.n_levels as usize).min(crate::state::MAX_LEVELS);
        for l in &s.levels[..n] {
            t.levels.push(Level {
                component: crate::ids::ComponentId(l.component),
                instance: crate::ids::InstanceId(l.instance),
                lattice: l.lattice,
            });
        }
        t
    }
}

impl From<&TrackState> for SmrtState {
    fn from(t: &TrackState) -> Self {
        let mut s = SmrtState {
            pos: [t.pos.x, t.pos.y, t.pos.z],
            dir: [t.dir.x, t.dir.y, t.dir.z],
            cell: t.cell.0,
            region: t.region.0,
            entered_via: t.entered_via.map_or(u32::MAX, |f| f.0),
            n_levels: t.levels.len().min(crate::state::MAX_LEVELS) as u32,
            levels: [SmrtLevel::default(); crate::state::MAX_LEVELS],
        };
        for (d, l) in s.levels.iter_mut().zip(&t.levels) {
            *d = SmrtLevel { component: l.component.0, instance: l.instance.0, lattice: l.lattice };
        }
        s
    }
}

fn bc_code(k: BoundaryKind) -> (i32, u32) {
    match k {
        BoundaryKind::Vacuum => (SMRT_BC_VACUUM, u32::MAX),
        BoundaryKind::Reflective => (SMRT_BC_REFLECTIVE, u32::MAX),
        BoundaryKind::White => (SMRT_BC_WHITE, u32::MAX),
        BoundaryKind::Periodic { partner } => (SMRT_BC_PERIODIC, partner.0),
    }
}

fn bc_from(code: i32, partner: u32) -> Option<BoundaryKind> {
    Some(match code {
        SMRT_BC_VACUUM => BoundaryKind::Vacuum,
        SMRT_BC_REFLECTIVE => BoundaryKind::Reflective,
        SMRT_BC_WHITE => BoundaryKind::White,
        SMRT_BC_PERIODIC => BoundaryKind::Periodic { partner: SurfaceId(partner) },
        _ => return None,
    })
}

fn step_out(dist: f64, cell: CellId, region: RegionId, end: &StepEnd) -> SmrtStep {
    let mut o = SmrtStep {
        dist,
        cell: cell.0,
        region: region.0,
        surface: u32::MAX,
        face: u32::MAX,
        to_region: u32::MAX,
        to_cell: u32::MAX,
        partner: u32::MAX,
        ..SmrtStep::default()
    };
    match *end {
        StepEnd::Limit => o.end_kind = SMRT_END_LIMIT,
        StepEnd::CellFace { face, to } => {
            o.end_kind = SMRT_END_CELL_FACE;
            o.face = face.0;
            o.to_cell = to.0;
        }
        StepEnd::RegionBoundary { surface, face, to_region, to_cell } => {
            o.end_kind = SMRT_END_REGION;
            (o.surface, o.face, o.to_region, o.to_cell) = (surface.0, face.0, to_region.0, to_cell.0);
        }
        StepEnd::DomainBoundary { surface, face, kind } => {
            o.end_kind = SMRT_END_DOMAIN;
            (o.surface, o.face) = (surface.0, face.0);
            (o.bc_kind, o.partner) = bc_code(kind);
        }
        StepEnd::Lost(_) => o.end_kind = SMRT_END_LOST,
    }
    o
}

static LAST_ERROR: Mutex<String> = Mutex::new(String::new());

fn set_error(msg: String) {
    if let Ok(mut g) = LAST_ERROR.lock() {
        *g = msg;
    }
}

/// Run `f` with the handle, mapping errors and panics to status codes.
fn guarded(h: *const SmrtTracker, f: impl FnOnce(&dyn Tracker) -> Result<i32, crate::RtError>) -> i32 {
    if h.is_null() {
        return SMRT_E_ARG;
    }
    // SAFETY: the caller promises `h` came from `into_handle` and is not freed concurrently.
    let t: &dyn Tracker = unsafe { &*(*h).0 };
    match catch_unwind(AssertUnwindSafe(|| f(t))) {
        Ok(Ok(c)) => c,
        Ok(Err(e)) => {
            set_error(e.to_string());
            SMRT_E_RT
        }
        Err(_) => SMRT_E_PANIC,
    }
}

/// Wrap a tracker for C callers. Free with `smrt_tracker_free`.
pub fn into_handle(t: Box<dyn Tracker>) -> *mut SmrtTracker {
    Box::into_raw(Box::new(SmrtTracker(t)))
}

/// A function that opens a tracker from a `.smsh` path (registered by the real crate at load).
pub type Opener = fn(&str) -> Result<Box<dyn Tracker>, crate::RtError>;

static OPENER: OnceLock<Opener> = OnceLock::new();

/// Register the opener used by `smrt_open` (first registration wins).
pub fn register_opener(f: Opener) {
    let _ = OPENER.set(f);
}

/// Free a handle.
///
/// # Safety
/// `h` must be null or a pointer from [`into_handle`] / `smrt_open`, not used afterwards.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_tracker_free(h: *mut SmrtTracker) {
    if !h.is_null() {
        // SAFETY: per the contract above.
        drop(unsafe { Box::from_raw(h) });
    }
}

/// Open a tracker from a `.smsh` file (building or loading the cache as needed).
///
/// # Safety
/// `path` must be a valid NUL-terminated string and `out` a valid pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_open(path: *const c_char, out: *mut *mut SmrtTracker) -> i32 {
    if path.is_null() || out.is_null() {
        return SMRT_E_ARG;
    }
    let Some(open) = OPENER.get() else { return SMRT_E_UNSUPPORTED };
    // SAFETY: NUL-terminated by contract.
    let Ok(p) = unsafe { CStr::from_ptr(path) }.to_str() else { return SMRT_E_ARG };
    match catch_unwind(AssertUnwindSafe(|| open(p))) {
        Ok(Ok(t)) => {
            // SAFETY: `out` is valid by contract.
            unsafe { *out = into_handle(t) };
            SMRT_OK
        }
        Ok(Err(e)) => {
            set_error(e.to_string());
            SMRT_E_RT
        }
        Err(_) => SMRT_E_PANIC,
    }
}

/// Copy the last error message (NUL-terminated, truncated to `cap`) and return its full length.
///
/// # Safety
/// `buf` must be valid for `cap` bytes (or null with `cap == 0`).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_last_error_message(buf: *mut c_char, cap: usize) -> usize {
    let msg = LAST_ERROR.lock().map(|g| g.clone()).unwrap_or_default();
    if !buf.is_null() && cap > 0 {
        let n = msg.len().min(cap - 1);
        // SAFETY: `buf` valid for `cap` bytes; `n < cap`.
        unsafe {
            core::ptr::copy_nonoverlapping(msg.as_ptr().cast::<c_char>(), buf, n);
            *buf.add(n) = 0;
        }
    }
    msg.len()
}

/// Number of tracking cells.
///
/// # Safety
/// `h` must be a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_cell_count(h: *const SmrtTracker) -> u64 {
    let mut n = 0;
    guarded(h, |t| {
        n = t.cell_count() as u64;
        Ok(SMRT_OK)
    });
    n
}

/// Locate a particle and fill `out`. `hint` is a cell id or `u32::MAX`.
///
/// # Safety
/// `h` live; `pos`, `dir` point to 3 doubles; `out` valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_init_state(h: *const SmrtTracker, pos: *const f64, dir: *const f64, hint: u32, out: *mut SmrtState) -> i32 {
    if pos.is_null() || dir.is_null() || out.is_null() {
        return SMRT_E_ARG;
    }
    // SAFETY: 3 doubles each by contract.
    let (p, d) = unsafe { (core::slice::from_raw_parts(pos, 3), core::slice::from_raw_parts(dir, 3)) };
    guarded(h, |t| {
        let st = t.init_state(Point3::new(p[0], p[1], p[2]), Vector3::new(d[0], d[1], d[2]), (hint != u32::MAX).then_some(CellId(hint)))?;
        // SAFETY: `out` valid by contract.
        unsafe { *out = SmrtState::from(&st) };
        Ok(SMRT_OK)
    })
}

/// One step (see `Tracker::step`).
///
/// # Safety
/// `h` live; `st` and `out` valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_step(h: *const SmrtTracker, st: *mut SmrtState, max_dist: f64, out: *mut SmrtStep) -> i32 {
    if st.is_null() || out.is_null() {
        return SMRT_E_ARG;
    }
    guarded(h, |t| {
        // SAFETY: valid by contract.
        let mut ts = TrackState::from(unsafe { &*st });
        let s = t.step(&mut ts, max_dist)?;
        // SAFETY: valid by contract.
        unsafe {
            *st = SmrtState::from(&ts);
            *out = step_out(s.dist, s.cell, s.region, &s.end);
        }
        Ok(SMRT_OK)
    })
}

/// Fire to the next region boundary (DAGMC `ray_fire`).
///
/// # Safety
/// `h` live; `st` and `out` valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_fire(h: *const SmrtTracker, st: *mut SmrtState, dist_limit: f64, out: *mut SmrtStep) -> i32 {
    if st.is_null() || out.is_null() {
        return SMRT_E_ARG;
    }
    guarded(h, |t| {
        // SAFETY: valid by contract.
        let mut ts = TrackState::from(unsafe { &*st });
        let (c0, r0) = (ts.cell, ts.region);
        let r = t.fire(&mut ts, dist_limit, 1_000_000)?;
        // SAFETY: valid by contract.
        unsafe {
            *st = SmrtState::from(&ts);
            *out = step_out(r.dist, c0, r0, &r.end);
        }
        Ok(SMRT_OK)
    })
}

/// Apply a boundary condition. Returns 0 to continue tracking, 1 if the particle terminates, <0 on error.
///
/// # Safety
/// `h` live; `st` valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_resolve_boundary(h: *const SmrtTracker, st: *mut SmrtState, face: u32, bc_kind: i32, partner: u32, u0: f64, u1: f64) -> i32 {
    if st.is_null() {
        return SMRT_E_ARG;
    }
    let Some(kind) = bc_from(bc_kind, partner) else { return SMRT_E_ARG };
    guarded(h, |t| {
        // SAFETY: valid by contract.
        let mut ts = TrackState::from(unsafe { &*st });
        let a = t.resolve_boundary(&mut ts, FaceId(face), kind, [u0, u1])?;
        // SAFETY: valid by contract.
        unsafe { *st = SmrtState::from(&ts) };
        Ok(i32::from(a == BoundaryAction::Terminate))
    })
}

/// C header for the above, kept next to the code so they are reviewed together.
pub const C_HEADER: &str = r#"
#include <stdint.h>
#include <stddef.h>
typedef struct SmrtTracker SmrtTracker;               /* opaque */
typedef struct { uint32_t component, instance; int32_t lattice[3]; } SmrtLevel;
typedef struct { double pos[3], dir[3]; uint32_t cell, region, entered_via, n_levels; SmrtLevel levels[4]; } SmrtState;
typedef struct { double dist; uint32_t cell, region; int32_t end_kind; uint32_t surface, face, to_region, to_cell;
                 int32_t bc_kind; uint32_t partner; } SmrtStep;
int32_t smrt_open(const char* path, SmrtTracker** out);
void    smrt_tracker_free(SmrtTracker*);
uint64_t smrt_cell_count(const SmrtTracker*);
int32_t smrt_init_state(const SmrtTracker*, const double pos[3], const double dir[3], uint32_t hint, SmrtState* out);
int32_t smrt_step(const SmrtTracker*, SmrtState*, double max_dist, SmrtStep* out);
int32_t smrt_fire(const SmrtTracker*, SmrtState*, double dist_limit, SmrtStep* out);
int32_t smrt_resolve_boundary(const SmrtTracker*, SmrtState*, uint32_t face, int32_t bc_kind, uint32_t partner, double u0, double u1);
size_t  smrt_last_error_message(char* buf, size_t cap);
"#;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stub::{AnalyticWorld, Shape};
    use sm_core::Aabb;

    #[test]
    fn c_abi_round_trip_through_the_stub() {
        let dom = Aabb { lo: Point3::new(-5.0, -5.0, -5.0), hi: Point3::new(5.0, 5.0, 5.0) };
        let w = AnalyticWorld::new(vec![Shape::Sphere { center: Point3::new(0.0, 0.0, 0.0), radius: 2.0 }], dom, BoundaryKind::Reflective).unwrap();
        let h = into_handle(Box::new(w));
        let (pos, dir) = ([-5.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
        let mut st = SmrtState::default();
        let mut step = SmrtStep::default();
        // SAFETY: all pointers are to live locals; `h` is live until freed below.
        unsafe {
            assert_eq!(smrt_cell_count(h), 2);
            assert_eq!(smrt_init_state(h, pos.as_ptr(), dir.as_ptr(), u32::MAX, &mut st), SMRT_OK);
            assert_eq!(smrt_fire(h, &mut st, f64::INFINITY, &mut step), SMRT_OK);
            assert_eq!(step.end_kind, SMRT_END_REGION);
            assert!((step.dist - 3.0).abs() < 1e-12 && step.to_region == 0);
            assert_eq!(smrt_fire(h, &mut st, f64::INFINITY, &mut step), SMRT_OK);
            assert_eq!((step.end_kind, step.to_region), (SMRT_END_REGION, 1), "leaves the sphere");
            assert_eq!(smrt_fire(h, &mut st, f64::INFINITY, &mut step), SMRT_OK);
            assert_eq!(step.end_kind, SMRT_END_DOMAIN);
            assert_eq!(step.bc_kind, SMRT_BC_REFLECTIVE);
            assert_eq!(smrt_resolve_boundary(h, &mut st, step.face, step.bc_kind, step.partner, 0.0, 0.0), 0);
            assert_eq!(st.dir[0], -1.0);
            // Errors: bad direction gives SMRT_E_RT and a message; null handle gives SMRT_E_ARG.
            let bad = [0.0, 0.0, 0.0];
            assert_eq!(smrt_init_state(h, pos.as_ptr(), bad.as_ptr(), u32::MAX, &mut st), SMRT_E_RT);
            let mut buf = [0 as c_char; 64];
            assert!(smrt_last_error_message(buf.as_mut_ptr(), buf.len()) > 0);
            assert_eq!(smrt_step(core::ptr::null(), &mut st, 1.0, &mut step), SMRT_E_ARG);
            smrt_tracker_free(h);
        }
    }

    #[test]
    fn state_conversion_round_trips() {
        let mut t = TrackState::new(Point3::new(1.0, 2.0, 3.0), Vector3::new(0.0, 0.0, 1.0));
        t.cell = CellId(7);
        t.entered_via = Some(FaceId(9));
        t.levels.push(Level { component: crate::ids::ComponentId(1), instance: crate::ids::InstanceId(2), lattice: [3, -1, 0] });
        let back = TrackState::from(&SmrtState::from(&t));
        assert_eq!(back, TrackState { region: RegionId::NONE, ..t });
    }
}
