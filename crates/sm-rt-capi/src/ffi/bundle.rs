//! `smrt_trace_bundle`: many rays into the CSR track database (layout in `crate::bundle`).

use super::{SmrtTracker, exclusive, guard, shared};
use crate::bundle::{encode_into, encoded_words};
use crate::status::*;
use crate::types::*;
use rayon::prelude::*;
use sm_core::{Point3, Vector3};
use sm_geom_api::Ray;
use sm_rt_api::{Scratch, TraceOptions, TrackBundle};

/// Read `n` values of a column.
///
/// # Safety
/// `p` is null or valid for `n` reads.
unsafe fn column<'a, T>(p: *const T, n: usize, name: &'static str) -> Result<&'a [T], Fail> {
    if p.is_null() {
        return Err(Fail::Null(name));
    }
    // SAFETY: non-null, and valid for `n` reads of `T` by the caller contract.
    Ok(unsafe { std::slice::from_raw_parts(p, n) })
}

fn options(o: Option<&SmrtTraceOpts>) -> Result<TraceOptions, Fail> {
    let mut t = TraceOptions::default();
    if let Some(o) = o {
        if o.max_dist.is_nan() {
            return Err(Fail::bad("max_dist is NaN"));
        }
        if o.max_dist > 0.0 && o.max_dist.is_finite() {
            t = t.with_max_dist(o.max_dist);
        }
        if o.max_steps > 0 {
            t = t.with_max_steps(u32::try_from(o.max_steps).unwrap_or(u32::MAX));
        }
        t = t.with_stop_at_deck(o.stop_at_deck != 0);
    }
    Ok(t)
}

/// Trace `n` rays (SoA arrays in `rays`) into a CSR track bundle, written to `out_buf` as
/// described in `src/bundle.rs` (a self-describing `uint64_t` word buffer). Rays are independent;
/// they are split into blocks of `block_size` rays (`<= 0`: the tracker's own partition), traced
/// in parallel with one scratch per block and joined in block order, so the bytes are identical
/// for any thread count and equal the Rust `Tracker::trace_bundle` result. `opts` may be null
/// (defaults). `*out_len` receives the words needed; if `cap` is smaller the call returns
/// `SMRT_ERR_BAD_ARG` (`-102`) and nothing is written (call again with a larger buffer, or ask for
/// the size with `out_buf == NULL`, `cap == 0`). Returns `0`.
///
/// # Safety
/// `tracker` is null or a live handle; `rays` is null or points to a `SmrtRays` whose arrays are
/// each valid for `n` reads (`start_cell` may be null); `opts` is null or valid for a read;
/// `out_buf` is null or valid for `cap` words of writes; `out_len` is null or valid for a write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn smrt_trace_bundle(
    tracker: *const SmrtTracker,
    n: i64,
    rays: *const SmrtRays,
    block_size: i64,
    opts: *const SmrtTraceOpts,
    out_buf: *mut u64,
    cap: u64,
    out_len: *mut u64,
) -> i32 {
    guard(|| {
        // SAFETY: caller contract above.
        let (t, r, out_len) = unsafe { (shared(tracker, "tracker")?, shared(rays, "rays")?, exclusive(out_len, "out_len")?) };
        // SAFETY: `opts` is null or valid for a read by the caller contract.
        let topts = options(unsafe { opts.as_ref() })?;
        let n = usize::try_from(n).map_err(|_| Fail::bad("negative ray count"))?;
        // SAFETY: each column is valid for `n` reads by the caller contract.
        let cols = unsafe { [column(r.x, n, "rays.x")?, column(r.y, n, "rays.y")?, column(r.z, n, "rays.z")?, column(r.u, n, "rays.u")?, column(r.v, n, "rays.v")?, column(r.w, n, "rays.w")?] };
        let list: Vec<Ray> =
            (0..n).map(|i| Ray { origin: Point3::new(cols[0][i], cols[1][i], cols[2][i]), dir: Vector3::new(cols[3][i], cols[4][i], cols[5][i]) }).collect();
        let tr = &*t.inner;
        let bundle = if block_size <= 0 {
            tr.trace_bundle(&list, &topts)?
        } else {
            let bs = usize::try_from(block_size).unwrap_or(usize::MAX).max(1);
            let parts: Vec<_> = list.par_chunks(bs).map(|c| tr.trace_bundle_block(c, &topts, &mut Scratch::new())).collect();
            let mut all = TrackBundle::new();
            for p in parts {
                all.append(&p?)?;
            }
            all
        };
        let need = encoded_words(&bundle);
        *out_len = need as u64;
        let room = usize::try_from(cap).unwrap_or(usize::MAX);
        if need == usize::MAX || room < need {
            return Err(Fail::bad(format!("output buffer too small: need {need} words, have {cap}")));
        }
        if out_buf.is_null() {
            return Err(Fail::Null("out_buf"));
        }
        // SAFETY: `out_buf` is non-null and valid for `cap >= need` words by the caller contract.
        let dst = unsafe { std::slice::from_raw_parts_mut(out_buf, need) };
        encode_into(&bundle, dst).ok_or_else(|| Fail::bad("encode failed"))?;
        Ok(SMRT_OK)
    })
}
