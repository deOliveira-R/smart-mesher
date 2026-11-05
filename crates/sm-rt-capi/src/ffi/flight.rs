//! Majorant partition queries (nodes, material sets) and callback-free flights.
//!
//! A flight is two-phase: the caller asks for the next segment, samples its own flight length
//! with its own cross sections, and either asks for the next segment or ends the flight at a
//! collision. The library never calls back and never keeps a caller pointer: the majorant and
//! block arrays are passed to every call (as `FlightInput` in Rust) and only read during it.

use super::{SmrtTracker, exclusive, guard, shared};
use crate::convert::*;
use crate::status::*;
use crate::types::*;
use sm_rt_api::{FlightCursor, FlightInput, HierarchyView, MajorantTracker, MaterialSetId, ModePolicy, NodeId, RegionId, RtError, Scratch, TrackState, Tracker};
use std::sync::Arc;

/// The majorant view of a tracker, or `Unsupported`.
fn view(t: &dyn Tracker) -> Result<&dyn MajorantTracker, Fail> {
    t.majorant_view().ok_or(Fail::Rt(RtError::Unsupported("tracker has no majorant view (TrackerCaps::MAJORANT)")))
}

fn node_in(v: i64, h: &dyn HierarchyView) -> Result<NodeId, Fail> {
    match u32::try_from(v) {
        Ok(n) if (n as usize) < h.node_count() => Ok(NodeId(n)),
        _ => Err(Fail::Rt(RtError::UnknownId { kind: "node", id: u32::try_from(v).unwrap_or(u32::MAX) })),
    }
}

fn set_in(v: i64, h: &dyn HierarchyView) -> Result<MaterialSetId, Fail> {
    match u32::try_from(v) {
        Ok(n) if (n as usize) < h.set_count() => Ok(MaterialSetId(n)),
        _ => Err(Fail::Rt(RtError::UnknownId { kind: "material set", id: u32::try_from(v).unwrap_or(u32::MAX) })),
    }
}

/// Number of nodes of the majorant partition: the length of every majorant array.
/// `SMRT_ERR_UNSUPPORTED` when the tracker has no partition. Returns `0`.
///
/// # Safety
/// `tracker` is null or a live handle; `out_n` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_n_nodes(tracker: *const SmrtTracker, out_n: *mut i64) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (t, out) = unsafe { (shared(tracker, "tracker")?, exclusive(out_n, "out_n")?) };
        *out = i64::try_from(view(&*t.inner)?.hierarchy().node_count()).unwrap_or(i64::MAX);
        Ok(SMRT_OK)
    })
}

/// Facts of node `node` (`SmrtNodeInfo`). Returns `0`; an unknown node is `SMRT_ERR_UNKNOWN_ID`.
///
/// # Safety
/// `tracker` is null or a live handle; `out` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_node_info(tracker: *const SmrtTracker, node: i64, out: *mut SmrtNodeInfo) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (t, out) = unsafe { (shared(tracker, "tracker")?, exclusive(out, "out")?) };
        let h = view(&*t.inner)?.hierarchy();
        let id = node_in(node, h)?;
        let n = h.node(id).ok_or(RtError::UnknownId { kind: "node", id: id.0 })?;
        *out = SmrtNodeInfo {
            parent: if n.parent.is_none() { SMRT_NONE } else { id_out(n.parent.0) },
            kind: node_kind_out(n.kind),
            set: id_out(n.set.0),
            cells: i64::from(n.cells),
            bounds_lo: parr(n.bounds.lo),
            bounds_hi: parr(n.bounds.hi),
        };
        Ok(SMRT_OK)
    })
}

/// The material set of `node` (the regions that occur inside it). Returns `0`.
///
/// # Safety
/// `tracker` is null or a live handle; `out_set` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_node_set(tracker: *const SmrtTracker, node: i64, out_set: *mut i64) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (t, out) = unsafe { (shared(tracker, "tracker")?, exclusive(out_set, "out_set")?) };
        let h = view(&*t.inner)?.hierarchy();
        let id = node_in(node, h)?;
        *out = id_out(h.node(id).ok_or(RtError::UnknownId { kind: "node", id: id.0 })?.set.0);
        Ok(SMRT_OK)
    })
}

/// Number of material sets (ids are dense `0..n`). Returns `0`.
///
/// # Safety
/// `tracker` is null or a live handle; `out_n` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_n_sets(tracker: *const SmrtTracker, out_n: *mut i64) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (t, out) = unsafe { (shared(tracker, "tracker")?, exclusive(out_n, "out_n")?) };
        *out = i64::try_from(view(&*t.inner)?.hierarchy().set_count()).unwrap_or(i64::MAX);
        Ok(SMRT_OK)
    })
}

/// The regions of material set `set`, ascending: up to `cap` ids are written to `out` and the
/// full count is stored in `*out_n` (so a first call with `cap = 0` sizes the buffer). Pair
/// `smrt_region_name` of each id with the caller's cross section to build a majorant. `out` may
/// be null when `cap` is `0`. Returns `0`.
///
/// # Safety
/// `tracker` is null or a live handle; `out` is null or valid for `cap` writes; `out_n` is null or
/// valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_set_regions(tracker: *const SmrtTracker, set: i64, out: *mut i64, cap: i64, out_n: *mut i64) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (t, out_n) = unsafe { (shared(tracker, "tracker")?, exclusive(out_n, "out_n")?) };
        let cap = usize::try_from(cap).map_err(|_| Fail::bad("negative capacity"))?;
        let h = view(&*t.inner)?.hierarchy();
        let id = set_in(set, h)?;
        let regs = h.set_regions(id).ok_or(RtError::UnknownId { kind: "material set", id: id.0 })?;
        if cap > 0 {
            if out.is_null() {
                return Err(Fail::Null("out"));
            }
            // SAFETY: `out` is non-null and valid for `cap` writes by the caller contract.
            let dst = unsafe { std::slice::from_raw_parts_mut(out, cap) };
            for (d, r) in dst.iter_mut().zip(regs) {
                *d = region_out(*r, t.complement);
            }
        }
        *out_n = i64::try_from(regs.len()).unwrap_or(i64::MAX);
        Ok(SMRT_OK)
    })
}

/// Read the caller's per-call arrays as a `FlightInput`: `n_nodes` must equal the node count and
/// `n_block <= n_nodes` (checked before any array is read), so a mismatch is `InvalidInput`
/// rather than an out-of-bounds read. Nothing is scanned; the one majorant a call needs is
/// checked by the library when it reads it.
///
/// # Safety
/// `majorants` is null or valid for `n_nodes` reads of `f64` and `block` is null or valid for
/// `n_block` reads of `u32`, both for the lifetime `'a` the caller chooses (one call).
unsafe fn flight_input<'a>(h: &dyn HierarchyView, majorants: *const f64, n_nodes: usize, block: *const u32, n_block: usize) -> Result<FlightInput<'a>, Fail> {
    if n_nodes != h.node_count() {
        return Err(Fail::Rt(RtError::InvalidInput("majorant slice length must equal the node count")));
    }
    if n_block > n_nodes {
        return Err(Fail::Rt(RtError::InvalidInput("block has more entries than the tracker has nodes")));
    }
    if majorants.is_null() {
        return Err(Fail::Null("majorants"));
    }
    // SAFETY: non-null and valid for `n_nodes` reads by the caller contract.
    let m = unsafe { std::slice::from_raw_parts(majorants, n_nodes) };
    let input = FlightInput::new(m);
    if n_block == 0 {
        return Ok(input);
    }
    if block.is_null() {
        return Err(Fail::Null("block"));
    }
    // SAFETY: non-null and valid for `n_block` reads of `u32` by the caller contract; `NodeId` is
    // `#[repr(transparent)]` over `u32`, so the slice has the same layout.
    let b = unsafe { std::slice::from_raw_parts(block.cast::<NodeId>(), n_block) };
    Ok(input.with_block(b))
}

/// One-time full check of the arrays a code will pass to every call (`FlightInput::validate`):
/// lengths, every majorant finite and `>= 0` (`SMRT_ERR_INVALID_MAJORANT`; the last-error message
/// names the node), `block` strictly ascending with every id below the node count. `O(n)`: run it
/// when the arrays are built, not per call. Returns `0`.
///
/// # Safety
/// `tracker` is null or a live handle; `majorants` is null or valid for `n_nodes` reads; `block`
/// is null or valid for `n_block` reads.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_flight_input_validate(tracker: *const SmrtTracker, majorants: *const f64, n_nodes: usize, block: *const u32, n_block: usize) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let t = unsafe { shared(tracker, "tracker")? };
        let v = view(&*t.inner)?;
        // SAFETY: caller contract above; the arrays are used only inside this call.
        let input = unsafe { flight_input(v.hierarchy(), majorants, n_nodes, block, n_block)? };
        input.validate(v.hierarchy())?;
        Ok(SMRT_OK)
    })
}

/// Opaque flight handle (`SmrtFlight` in C): the particle state, the scratch and the resumable
/// cursor. It keeps its tracker alive and holds no caller pointer between calls. One thread at
/// a time, like a `SmrtCtx`.
pub struct SmrtFlight {
    tracker: Arc<dyn Tracker>,
    complement: Option<RegionId>,
    scratch: Scratch,
    st: TrackState,
    cursor: FlightCursor,
}

/// Start a flight of the particle in `*state` (a located state, for example from
/// `smrt_get_state`; a copy is kept). `opts` may be null for the defaults (surface segments per
/// cell, unlimited, 1e6 segments). `SMRT_ERR_UNSUPPORTED` when the tracker has no majorant
/// partition. Free the flight with `smrt_flight_free`. Returns `0`; `*out` is null on failure.
///
/// # Safety
/// `tracker` is null or a live handle; `state` is null or valid for a read; `opts` is null or
/// valid for a read; `out` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_flight_new(tracker: *const SmrtTracker, state: *const SmrtState, opts: *const SmrtFlightOpts, out: *mut *mut SmrtFlight) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let out = unsafe { exclusive(out, "out")? };
        *out = std::ptr::null_mut();
        // SAFETY: caller contract above.
        let (t, s) = unsafe { (shared(tracker, "tracker")?, shared(state, "state")?) };
        // SAFETY: `opts` is null or valid for a read by the caller contract.
        let o = flight_opts_in(unsafe { opts.as_ref() })?;
        let st = state_in(&*t.inner, t.complement, s)?;
        let cursor = view(&*t.inner)?.begin(&st, &o)?;
        let f = SmrtFlight { tracker: Arc::clone(&t.inner), complement: t.complement, scratch: Scratch::with_capacity(64), st, cursor };
        *out = Box::into_raw(Box::new(f));
        Ok(SMRT_OK)
    })
}

/// Free a flight. Returns `0`.
///
/// # Safety
/// `flight` is null or a handle from `smrt_flight_new` not freed before, and no other thread
/// uses it.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_flight_free(flight: *mut SmrtFlight) -> i32 {
    guard(|| {
        if flight.is_null() {
            return Err(Fail::Null("flight"));
        }
        // SAFETY: non-null, created by `Box::into_raw` in `smrt_flight_new` and freed once.
        drop(unsafe { Box::from_raw(flight) });
        Ok(SMRT_OK)
    })
}

/// The next segment of the flight, with the caller's arrays for this call: `majorants` has one
/// entry per node (`n_nodes` must equal `smrt_n_nodes`; only the entry of the segment's leaf is
/// read, and it must be finite and `>= 0`), `block` lists the nodes forced to surface segments
/// (sorted, deduplicated, `n_block <= n_nodes`; may be null with `n_block = 0`). The arrays are
/// not retained, so they may change between calls. Returns `0` with `*out` filled and the state
/// at the segment's end, or `1` when the flight is over (see `smrt_flight_end`; `*out` is
/// untouched), or a negative status (the state is unchanged for `SMRT_ERR_INVALID_MAJORANT`,
/// `SMRT_ERR_NON_FINITE` and `SMRT_ERR_INVALID_INPUT`). Does not allocate after warm-up.
///
/// # Safety
/// `flight` is null or a live handle used by this thread only; `majorants` is null or valid for
/// `n_nodes` reads, `block` is null or valid for `n_block` reads; `out` is null or valid for a
/// write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_flight_next(flight: *mut SmrtFlight, majorants: *const f64, n_nodes: usize, block: *const u32, n_block: usize, out: *mut SmrtSegment) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (f, out) = unsafe { (exclusive(flight, "flight")?, exclusive(out, "out")?) };
        let SmrtFlight { tracker, complement, scratch, st, cursor } = f;
        let v = view(&**tracker)?;
        // SAFETY: caller contract above; the arrays are used only inside this call.
        let input = unsafe { flight_input(v.hierarchy(), majorants, n_nodes, block, n_block)? };
        match cursor.next_segment(v, input, st, scratch)? {
            Some(seg) => {
                segment_out(&seg, *complement, out);
                Ok(SMRT_OK)
            }
            None => Ok(SMRT_NO_HIT),
        }
    })
}

/// Locate the point at flight parameter `t` (between the last segment's `t0` and `t1`) from the
/// node that owns the majorant, to find the cell and region of a tentative collision. Does not
/// change the state or the flight. `SMRT_ERR_INVALID_INPUT` before the first segment or for `t`
/// outside the segment. Same array rules as `smrt_flight_next`. Returns `0`.
///
/// # Safety
/// As `smrt_flight_next`; `out` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_flight_probe(flight: *mut SmrtFlight, majorants: *const f64, n_nodes: usize, block: *const u32, n_block: usize, t: f64, out: *mut SmrtLocated) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (f, out) = unsafe { (exclusive(flight, "flight")?, exclusive(out, "out")?) };
        let SmrtFlight { tracker, complement, scratch, cursor, .. } = f;
        let v = view(&**tracker)?;
        // SAFETY: caller contract above; the arrays are used only inside this call.
        let input = unsafe { flight_input(v.hierarchy(), majorants, n_nodes, block, n_block)? };
        let loc = cursor.probe(v, input, scratch, t)?;
        located_out(&loc, *complement, out);
        Ok(SMRT_OK)
    })
}

/// End the flight with a real collision at flight parameter `t` inside the last segment: the
/// state is placed there (cell, region and levels located) and `smrt_flight_end` reports
/// `SMRT_FLIGHT_STOPPED`. Same array rules as `smrt_flight_next`. Returns `0`, with the location
/// in `*out`.
///
/// # Safety
/// As `smrt_flight_next`; `out` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_flight_stop_at(flight: *mut SmrtFlight, majorants: *const f64, n_nodes: usize, block: *const u32, n_block: usize, t: f64, out: *mut SmrtLocated) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (f, out) = unsafe { (exclusive(flight, "flight")?, exclusive(out, "out")?) };
        let SmrtFlight { tracker, complement, scratch, st, cursor } = f;
        let v = view(&**tracker)?;
        // SAFETY: caller contract above; the arrays are used only inside this call.
        let input = unsafe { flight_input(v.hierarchy(), majorants, n_nodes, block, n_block)? };
        let loc = cursor.stop_at(v, input, st, scratch, t)?;
        located_out(&loc, *complement, out);
        Ok(SMRT_OK)
    })
}

/// Check a collision's total cross section against the majorant cached at the last
/// `smrt_flight_next` (no array argument). Returns `0` when `sigma_t <= majorant * (1 + 1e-12)`,
/// `SMRT_ERR_MAJORANT` (`-12`) when it is larger (the message names the node, the point and the
/// cell; nothing advances and the state is unchanged), `SMRT_ERR_NON_FINITE` for a NaN, infinite
/// or negative value, `SMRT_ERR_INVALID_INPUT` before the first segment or after the flight ended.
///
/// # Safety
/// `flight` is null or a live handle used by this thread only.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_flight_check(flight: *const SmrtFlight, sigma_t: f64) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let f = unsafe { shared(flight, "flight")? };
        f.cursor.check_collision(sigma_t)?;
        Ok(SMRT_OK)
    })
}

/// Change the mode policy (`SMRT_POLICY_*`) from the next segment on (hybrid tracking). Returns `0`.
///
/// # Safety
/// `flight` is null or a live handle used by this thread only.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_flight_set_policy(flight: *mut SmrtFlight, policy: i64) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let f = unsafe { exclusive(flight, "flight")? };
        let p = match policy {
            SMRT_POLICY_SURFACE => ModePolicy::Surface,
            SMRT_POLICY_DELTA => ModePolicy::Delta,
            v => return Err(Fail::bad(format!("unknown flight policy {v}"))),
        };
        f.cursor.set_policy(p)?;
        Ok(SMRT_OK)
    })
}

/// The particle state of the flight (at the end of the last segment, or where the flight was
/// stopped). Returns `0`.
///
/// # Safety
/// `flight` is null or a live handle used by this thread only; `out` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_flight_get_state(flight: *const SmrtFlight, out: *mut SmrtState) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (f, out) = unsafe { (shared(flight, "flight")?, exclusive(out, "out")?) };
        state_out(&f.st, f.complement, out);
        Ok(SMRT_OK)
    })
}

/// Why the flight ended: `*out_kind` is `SMRT_FLIGHT_RUNNING` (0) while it goes on, else
/// `SMRT_FLIGHT_LIMIT`, `_LEFT_MODEL`, `_BOUNDARY`, `_LOST` or `_STOPPED`. `*out_event` (may be
/// null) holds the surface, face and action of a boundary or the cause of a loss, as
/// `smrt_step` spells them. `*out_dist` (may be null) is the flight parameter reached. Returns `0`.
///
/// # Safety
/// `flight` is null or a live handle; `out_kind` is null or valid for a write; `out_event` and
/// `out_dist` are null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_flight_end(flight: *const SmrtFlight, out_kind: *mut i32, out_event: *mut SmrtStep, out_dist: *mut f64) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (f, kind) = unsafe { (shared(flight, "flight")?, exclusive(out_kind, "out_kind")?) };
        let mut ev = SmrtStep::default();
        *kind = flight_end_out(f.cursor.end(), f.complement, &mut ev);
        // SAFETY: null or valid for a write by the caller contract.
        if let Some(o) = unsafe { out_event.as_mut() } {
            *o = ev;
        }
        // SAFETY: null or valid for a write by the caller contract.
        if let Some(d) = unsafe { out_dist.as_mut() } {
            *d = f.cursor.distance();
        }
        Ok(SMRT_OK)
    })
}
