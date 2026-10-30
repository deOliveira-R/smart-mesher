//! Tracking entry points: contexts, state, step / peek / fire, boundary response, locate, closest.

use super::{SmrtCtx, SmrtTracker, exclusive, guard, read3, read3_opt, shared, write3};
use crate::convert::*;
use crate::status::*;
use crate::types::*;
use sm_rt_api::{BoundaryAction, CellId, Level, RegionId, Scratch, StepEnd, TrackState, check_direction};
use sm_rt_api::{ComponentId, InstanceId};
use std::sync::Arc;

/// Create a context (particle state + scratch) for `tracker`. The context holds its own
/// reference to the tracker, so the two handles may be freed in any order.
///
/// Returns `0` and stores the new context in `*out` (free it with `smrt_ctx_free`).
///
/// # Safety
/// `tracker` is null or a live handle from this library; `out` is null or valid for one write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_ctx_new(tracker: *const SmrtTracker, out: *mut *mut SmrtCtx) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (t, out) = unsafe { (shared(tracker, "tracker")?, exclusive(out, "out")?) };
        let ctx = SmrtCtx {
            tracker: Arc::clone(&t.inner),
            complement: t.complement,
            scratch: Scratch::with_capacity(64),
            st: TrackState::new(sm_core::Point3::new(0.0, 0.0, 0.0), sm_core::Vector3::new(1.0, 0.0, 0.0)),
        };
        *out = Box::into_raw(Box::new(ctx));
        Ok(SMRT_OK)
    })
}

/// Free a context. Returns `0`.
///
/// # Safety
/// `ctx` is null or a handle from `smrt_ctx_new` not freed before, and no other thread uses it.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_ctx_free(ctx: *mut SmrtCtx) -> i32 {
    guard(|| {
        if ctx.is_null() {
            return Err(Fail::Null("ctx"));
        }
        // SAFETY: non-null, and by the caller contract created by `Box::into_raw` in
        // `smrt_ctx_new` and freed exactly once.
        drop(unsafe { Box::from_raw(ctx) });
        Ok(SMRT_OK)
    })
}

/// Locate a particle and make it the context's state.
///
/// `hint_cell` is a cell to start the walk from (`-1` for none). Returns `0` when the point is
/// inside the model, `1` when it is outside (the state is still set, with cell `-1`), or a
/// negative status (bad direction, non-finite position, unknown hint).
///
/// # Safety
/// `ctx` is null or a live context used by this thread only.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_init_state(ctx: *mut SmrtCtx, x: f64, y: f64, z: f64, u: f64, v: f64, w: f64, hint_cell: i64) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let c = unsafe { exclusive(ctx, "ctx")? };
        let hint = hint_in(hint_cell)?;
        let st = c.tracker.init_state(point([x, y, z]), vector([u, v, w]), hint, &mut c.scratch)?;
        c.st = st;
        Ok(if c.st.cell.is_none() { SMRT_NO_HIT } else { SMRT_OK })
    })
}

/// Copy the context's state into `*out`. Levels past eight are counted in `levels_total` and
/// not copied. Returns `0`.
///
/// # Safety
/// `ctx` is null or a live context used by this thread only; `out` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_get_state(ctx: *const SmrtCtx, out: *mut SmrtState) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (c, out) = unsafe { (shared(ctx, "ctx")?, exclusive(out, "out")?) };
        state_out(&c.st, c.complement, out);
        Ok(SMRT_OK)
    })
}

/// Replace the context's state with `*state` (for example to bank and resume a particle).
/// `levels_total` must equal `levels_len` and be at most 8 (a deeper path cannot be set from C:
/// `SMRT_ERR_BAD_ARG`); the direction must be unit within the model tolerance. Returns `0`.
///
/// # Safety
/// `ctx` is null or a live context used by this thread only; `state` is null or valid for a read.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_set_state(ctx: *mut SmrtCtx, state: *const SmrtState) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (c, s) = unsafe { (exclusive(ctx, "ctx")?, shared(state, "state")?) };
        let n = usize::try_from(s.levels_len).ok().filter(|&n| n <= SMRT_MAX_LEVELS && s.levels_total == s.levels_len);
        let Some(n) = n else {
            return Err(Fail::bad("levels_total must equal levels_len and be at most 8"));
        };
        check_direction(vector(s.dir), c.tracker.info().tol.angular)?;
        if !s.pos.iter().all(|v| v.is_finite()) {
            return Err(Fail::Rt(sm_rt_api::RtError::NonFinite("position")));
        }
        let mut st = TrackState::new(point(s.pos), vector(s.dir));
        st.cell = cell_in(s.cell)?;
        // NONE is "outside"; COMPLEMENT is valid only when the model has an implicit complement.
        let special = st.cell == CellId::NONE || (st.cell == CellId::COMPLEMENT && c.complement.is_some());
        if !special && st.cell.get() >= c.tracker.cell_count() {
            return Err(Fail::Rt(sm_rt_api::RtError::UnknownId { kind: "cell", id: st.cell.get() as u32 }));
        }
        st.region = region_in(s.region, c.complement)?;
        if st.region != RegionId::NONE && st.region.0 as usize >= c.tracker.regions().0.len() {
            return Err(Fail::Rt(sm_rt_api::RtError::UnknownId { kind: "region", id: st.region.0 }));
        }
        st.entered_via = if s.entered_via < 0 { None } else { Some(face_in(s.entered_via)?) };
        for l in &s.levels[..n] {
            let id = |v: i64| u32::try_from(v).map_err(|_| Fail::bad("negative level id"));
            st.levels.push(Level::new(ComponentId(id(l.component)?), InstanceId(id(l.instance)?), l.lattice));
        }
        c.st = st;
        Ok(SMRT_OK)
    })
}

/// Change the direction of the context's particle (after a scattering event). The new direction
/// must be unit within the model tolerance. The excluded entry face is kept. Returns `0`.
///
/// # Safety
/// `ctx` is null or a live context used by this thread only.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_set_direction(ctx: *mut SmrtCtx, u: f64, v: f64, w: f64) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let c = unsafe { exclusive(ctx, "ctx")? };
        let d = vector([u, v, w]);
        check_direction(d, c.tracker.info().tol.angular)?;
        c.st.dir = d;
        Ok(SMRT_OK)
    })
}

/// Advance the particle to its next event (a face, region or model boundary, or `max_dist`) and
/// describe it in `*out`. Deck boundaries (mirror, wrap) are applied inside the step; a response
/// boundary leaves the particle on the face for `smrt_resolve_boundary`. `max_dist` is `>= 0`
/// (use infinity for no limit). Returns `0` for an event and `1` when `max_dist` was reached
/// without one (`end == SMRT_END_LIMIT`, the state is advanced). A lost particle is status `0`
/// with `end == SMRT_END_LOST`. Does not allocate.
///
/// # Safety
/// `ctx` is null or a live context used by this thread only; `out` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_step(ctx: *mut SmrtCtx, max_dist: f64, out: *mut SmrtStep) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (c, out) = unsafe { (exclusive(ctx, "ctx")?, exclusive(out, "out")?) };
        let s = c.tracker.step(&mut c.st, max_dist, &mut c.scratch)?;
        step_out(&s, c.complement, out);
        Ok(if matches!(s.end, StepEnd::Limit) { SMRT_NO_HIT } else { SMRT_OK })
    })
}

/// The next surface along the particle's ray, without moving it (`distance_to_boundary`).
/// `*out` gets `dist`, `surface`, `face`, `end` (`SMRT_END_REGION_BOUNDARY` with `to_region`, or
/// `SMRT_END_BOUNDARY` with `action`; the boundary is reported, not followed). Returns `0` for a
/// hit (or a loss), `1` when nothing lies within `limit`.
///
/// # Safety
/// `ctx` is null or a live context used by this thread only; `out` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_peek(ctx: *mut SmrtCtx, limit: f64, out: *mut SmrtStep) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (c, out) = unsafe { (exclusive(ctx, "ctx")?, exclusive(out, "out")?) };
        let p = c.tracker.distance_to_boundary(&c.st, limit, &mut c.scratch)?;
        let found = peek_out(&p, c.st.cell, c.st.region, c.complement, out);
        Ok(if found { SMRT_OK } else { SMRT_NO_HIT })
    })
}

/// Walk through cells of the current region until the region changes, a model boundary is
/// reached, `dist_limit` is hit or the particle is lost (DAGMC `ray_fire`); interior faces and
/// lattice walls are skipped. `max_steps <= 0` means the tracker's event cap. In `*out`, `dist`
/// is the total distance, `cell` the starting cell and `region` the region fired through.
/// Returns `0` for an event, `1` for `SMRT_END_LIMIT`.
///
/// # Safety
/// `ctx` is null or a live context used by this thread only; `out` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_fire(ctx: *mut SmrtCtx, dist_limit: f64, max_steps: i64, out: *mut SmrtStep) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (c, out) = unsafe { (exclusive(ctx, "ctx")?, exclusive(out, "out")?) };
        let steps = if max_steps > 0 { u32::try_from(max_steps).unwrap_or(u32::MAX) } else { c.tracker.info().max_events };
        let (cell0, region0) = (c.st.cell, c.st.region);
        let r = c.tracker.fire(&mut c.st, dist_limit, steps, &mut c.scratch)?;
        fire_out(r.dist, &r.end, cell0, region0, c.complement, out);
        Ok(if matches!(r.end, StepEnd::Limit) { SMRT_NO_HIT } else { SMRT_OK })
    })
}

/// Apply a response boundary to a particle sitting on a boundary face, after a step ended with
/// `SMRT_END_BOUNDARY` and a response `action`. `kind` is `SMRT_RESP_*`; `albedo` is read only
/// for `SMRT_RESP_ALBEDO`; `u0`, `u1` are uniforms in `[0, 1]` from the caller's RNG (White
/// reads both, Albedo reads `u0`). Returns `0` if the particle continues (its new direction is
/// stored in `out_dir[3]` and in the state) and `1` if it is terminated.
///
/// # Safety
/// `ctx` is null or a live context used by this thread only; `out_dir` is null or valid for
/// three doubles.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_resolve_boundary(ctx: *mut SmrtCtx, face: i64, kind: i32, albedo: f64, u0: f64, u1: f64, out_dir: *mut f64) -> i32 {
    guard(|| {
        // SAFETY: caller contract above (checked for null before the call, written after).
        let c = unsafe { exclusive(ctx, "ctx")? };
        if out_dir.is_null() {
            return Err(Fail::Null("out_dir"));
        }
        let rk = response_in(kind, albedo)?;
        let act = c.tracker.resolve_boundary(&mut c.st, face_in(face)?, rk, [u0, u1])?;
        // SAFETY: `out_dir` is non-null and valid for three doubles by the caller contract.
        unsafe { write3(out_dir, arr(c.st.dir), "out_dir")? };
        Ok(if matches!(act, BoundaryAction::Terminate) { SMRT_NO_HIT } else { SMRT_OK })
    })
}

/// Find the cell containing `p[3]`; does not touch the context's state. `dir_or_null` (three
/// doubles) decides a point on a face (the cell it moves into); null uses the deterministic
/// hashed direction. `hint` is a start cell for the walk (`-1` none). Returns `0` when inside
/// the model, `1` when outside (`cell == -1`).
///
/// # Safety
/// `ctx` is null or a live context used by this thread only; `p` is null or three readable
/// doubles; `dir_or_null` is null or three readable doubles; `out` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_locate(ctx: *mut SmrtCtx, p: *const f64, dir_or_null: *const f64, hint: i64, out: *mut SmrtLocated) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (c, p, d, out) = unsafe { (exclusive(ctx, "ctx")?, read3(p, "p")?, read3_opt(dir_or_null, "dir")?, exclusive(out, "out")?) };
        let l = c.tracker.locate(point(p), d.map(vector), hint_in(hint)?, &mut c.scratch)?;
        located_out(&l, c.complement, out);
        Ok(if l.cell.is_none() { SMRT_NO_HIT } else { SMRT_OK })
    })
}

/// Distance and location of the closest boundary of `region` (`-1` = any region, `-2` the
/// implicit complement) from `p[3]` (DAGMC `closest_to_location`). Returns `0`, or `1` when the
/// tracker has no boundary to report.
///
/// # Safety
/// `ctx` is null or a live context used by this thread only; `p` is null or three readable
/// doubles; `out` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_closest(ctx: *mut SmrtCtx, p: *const f64, region: i64, out: *mut SmrtClosest) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (c, p, out) = unsafe { (exclusive(ctx, "ctx")?, read3(p, "p")?, exclusive(out, "out")?) };
        let r = match region {
            SMRT_NONE => None,
            v => Some(region_in(v, c.complement)?),
        };
        match c.tracker.closest(point(p), r, &mut c.scratch)? {
            Some(h) => {
                closest_out(&h, out);
                Ok(SMRT_OK)
            }
            None => Ok(SMRT_NO_HIT),
        }
    })
}
