# sm-rt-capi: design

Layer L4, the one crate allowed `unsafe` (decision 12). It turns any `sm_rt_api::Tracker` into a C
library, `libsm_rt_capi` (`cdylib` + `staticlib`, plus the rlib for Rust embedding), with
`include/smrt.h`, `fortran/smrt.f90` and an optional C++ wrapper `include/smrt.hpp`.

## Positioning

Similar to DAGMC's C++ `DagMC` class (`ray_fire`, `point_in_volume`, `next_vol`, `surface_sense`,
`get_angle`) and to the handle-plus-status style of OpenMC's C API, this crate differs in three
ways. (1) It wraps a *trait*, not a geometry engine: the same ABI serves the W3a mesh walker, the
W3b analytic and hierarchy backends and the test stub, because it only sees `Tracker`. (2) Particle
state lives in a per-thread context that the library owns (`SmrtCtx`: `TrackState` + `Scratch`), so
the hot loop needs no `RayHistory` vector (`entered_via` replaces it), no allocation after the context
exists, and a Fortran or C++ code carries one pointer per thread. (3) Failure is a status code plus a
thread-local message, never a panic across the boundary and never a global lock. The result for a
transport code is a DAGMC-shaped port path (the `smrt_dag_*` shims) and a native path (`smrt_step`,
`smrt_fire`, `smrt_trace_bundle`) over one library that does not care which tracker is behind it.

## Layout of the crate

| Path | Content | `unsafe`? |
|---|---|---|
| `src/ffi/` | handles, `guard`, pointer helpers, every `extern "C"` entry point; `test_hooks.rs` (feature `stubs`) | yes, each block has a `// SAFETY:` comment, each `unsafe fn` a `# Safety` section |
| `src/status.rs` | status codes, `Fail`, thread-local last error | no |
| `src/convert.rs` | contract types to the C views and back (ids, state, steps, levels) | no |
| `src/types.rs` | `#[repr(C)]` structs and constants | no |
| `src/bundle.rs` | wire layout of a track bundle, encoder and decoder | no |

A test greps that no file under `src/` outside `src/ffi/` contains the keyword. The crate root is
`#![deny(unsafe_code)]` with a single `#[allow(unsafe_code)]` on `mod ffi`; the workspace
`unsafe_op_in_unsafe_fn = deny` stays on, so every dereference is an explicit block.

## Handles, ownership, threading

* `SmrtTracker`: `Arc<dyn Tracker>` plus the cached complement region and a lazily computed region
  measure table. A thin pointer (`Box`), because `Arc<dyn _>` itself is fat. Shared, read-only,
  `Send + Sync`: any number of threads use one handle.
* `SmrtCtx`: its own `Arc` clone of the tracker, a `Scratch` and the particle's `TrackState`. One
  thread at a time (documented, not locked). Tracker and context handles may be freed in any order.
* Rust constructors: `tracker_into_raw(Arc<dyn Tracker>) -> *mut SmrtTracker` (safe function).
  `smrt_open(path, &out)` takes a `.smrt` image path. The loader is not linked in W3a:
  without the `sm-rt` cargo feature (off) it returns `SMRT_ERR_UNSUPPORTED`. The feature exists so the
  orchestrator's wiring is a manifest change; see "smrt_open" in `W3_REPORT.md` for what else the
  loader needs.
* After a panic inside a context call the particle state may be inconsistent: re-initialise it.

## Ids

Ids are `int64_t`. `>= 0` is the dense id, `SMRT_NONE = -1` is "none / outside the model",
`SMRT_COMPLEMENT = -2` is the implicit complement (a region with the contract's `implicit_complement`
flag, or `CellId::COMPLEMENT`). **Outputs always spell the implicit-complement region
`SMRT_COMPLEMENT`; inputs accept it or the raw id** (`SmrtInfo.complement_region`). This gives DAGMC
code one name for the implicit complement volume on both sides of the API.

## Status codes

Every function returns `int32_t`.

| Code | Name | Meaning |
|---|---|---|
| 0 | `SMRT_OK` | done |
| 1 | `SMRT_NO_HIT` | not an error: `smrt_step`/`smrt_fire`/`smrt_peek` reached the limit without an event (`end == SMRT_END_LIMIT`); `smrt_init_state`/`smrt_locate` found the point outside the model; `smrt_closest` has no boundary; `smrt_resolve_boundary` terminated the particle; `smrt_dag_ray_fire` miss; `smrt_dag_next_vol`/`surface_sense` for a region that does not touch the surface |
| -1 .. -11 | `RtError::code()` | `Build`, `BadDirection`, `NonFinite`, `UnknownId`, `Unsupported`, `CacheStale`, `CacheCorrupt`, `Geom`, `BadAlbedo`, `BinCountMismatch`, `InvalidInput` (frozen by the contract) |
| -100 | `SMRT_ERR_NULL` | a required pointer was null |
| -101 | `SMRT_ERR_PANIC` | a panic was caught at the boundary (the process survives) |
| -102 | `SMRT_ERR_BAD_ARG` | bad enum, size, capacity, or an argument outside its domain |
| -103 | `SMRT_ERR_LOST` | a DAGMC shim lost the particle (elsewhere a lost particle is a value: `end == SMRT_END_LOST` with `lost_*` fields) |

`smrt_last_error_message(buf, cap)` copies the calling thread's last message (`snprintf` semantics,
returns the full length; `buf` may be null with `cap == 0`). Thread-local storage, no mutex. Only
failures write it; successful calls do not clear it.

## ABI table

All pointers are checked for null first. `ctx` functions take the context of exactly one thread.

| Function | Notes |
|---|---|
| `smrt_open(path, &tracker)` | `.smrt` loader; unsupported without a backend |
| `smrt_tracker_free(tracker)`, `smrt_ctx_new(tracker, &ctx)`, `smrt_ctx_free(ctx)` | lifetimes |
| `smrt_info(tracker, &SmrtInfo)` | cells, faces, regions, surfaces, prototypes, copies, memory_bytes, exact/faceted faces, caps, complement region, structure hash lo/hi, bounds, tolerances |
| `smrt_region_info(tracker, region, &SmrtRegionInfo)` | original id (model entity serial), colour id (always -1, the contract has none), measure (volume), complement flag |
| `smrt_init_state(ctx, x,y,z, u,v,w, hint_cell)` | locate and set the state; 1 if outside the model |
| `smrt_get_state(ctx, &SmrtState)`, `smrt_set_state(ctx, &SmrtState)`, `smrt_set_direction(ctx, u,v,w)` | bank, resume, scatter |
| `smrt_step(ctx, max_dist, &SmrtStep)` | one event; decks (mirror, wrap) applied inside; no allocation |
| `smrt_peek(ctx, limit, &SmrtStep)` | `distance_to_boundary`; state untouched |
| `smrt_fire(ctx, dist_limit, max_steps, &SmrtStep)` | skips cell faces and slot changes |
| `smrt_resolve_boundary(ctx, face, kind, albedo, u0, u1, out_dir[3])` | response boundaries; 0 continue, 1 terminate |
| `smrt_locate(ctx, p[3], dir_or_null, hint, &SmrtLocated)` | state untouched |
| `smrt_closest(ctx, p[3], region, &SmrtClosest)` | `closest_to_location` |
| `smrt_trace_bundle(tracker, n, &SmrtRays, block_size, opts, out_buf, cap, &out_len)` | see below |
| `smrt_dag_ray_fire`, `smrt_dag_point_in_volume`, `smrt_dag_next_vol`, `smrt_dag_surface_sense`, `smrt_dag_get_angle`, `smrt_dag_measure_volume`, `smrt_dag_measure_area` | DAGMC shims |
| `smrt_version()`, `smrt_last_error_message(buf, cap)` | |

Fortran spelling: `fortran/smrt.f90` mirrors `smrt.h` with four renames the language forces (Fortran is
case-insensitive and `end`/`kind` are poor identifiers): `SMRT_VERSION` is `SMRT_ABI_VERSION`
everywhere (C, Fortran and Rust), `SmrtStep%end` is `end_code`, the `kind` argument of
`smrt_resolve_boundary` is `resp_kind`, and the `uint64_t` hash fields of `SmrtInfo` are `int64` bit
patterns. `smrt_version()` is the one entry without `guard`: it returns a constant and cannot panic.

`SmrtState`, `SmrtStep`, `SmrtLocated`, `SmrtLevel[8]` mirror the contract. Hierarchy levels past
eight are counted in `levels_total` and not copied (`levels_len <= 8`): truncated, never an overrun.
`smrt_set_state` refuses `levels_total != levels_len` or more than eight levels (`-102`).
Every struct has only 8-byte members, so there is no hidden padding; a test compiles `static_assert`s
for every size and offset (generated from `size_of`/`offset_of`) against `smrt.h`.

### `smrt_step`, `smrt_fire` and events

`SmrtStep.end` is `SMRT_END_{LIMIT, CELL_FACE, SLOT_CHANGE, REGION_BOUNDARY, BOUNDARY, LOST}`.
`smrt_step` reports every cell face and slot change; `smrt_fire` skips both (they do not change the
region). For a fire, `cell` is the starting cell, `region` the region fired through and `dist` the
total distance. A boundary event carries `action`: `SMRT_ACT_VACUUM/WHITE/ALBEDO/REFLECTIVE` for
response boundaries (the particle sits on the face: call `smrt_resolve_boundary`; the albedo value is
in `SmrtStep.albedo`), `SMRT_ACT_MIRROR`/`SMRT_ACT_WRAP` for decks that `step` already applied.

### Bundles

`smrt_trace_bundle` splits the `n` rays (SoA arrays `x,y,z,u,v,w` and an optional `start_cell`, which
the contract's `trace_bundle` ignores) into blocks of `block_size` rays, traces the blocks in
parallel with rayon (one `Scratch` per block, `Tracker::trace_bundle_block`) and joins them in block
order, so the bytes are identical for any thread count and equal to the Rust `trace_bundle` result;
`block_size <= 0` calls the tracker's own `trace_bundle`. Output is a self-describing `uint64_t`
word buffer: header of 12 words (magic `SMRTBNDL`, version, `n_rays`, `n_segs`, six section offsets,
total words) then `offsets`, `cell`, `region` (u32 packed two per word), `length` (f64 bits), `end`,
`end_surface` (u32 packed). `*out_len` is the word count needed; a smaller `cap` returns `-102` and
writes nothing, and `out_buf == NULL` with `cap == 0` is the size query (it traces, so the caller who
sizes first pays twice). `crate::bundle::decode` reads it back into a `TrackBundle`.

## DAGMC shims and the differences from DAGMC

The shims call `sm_rt_api::compat` one to one. Documented differences:

1. **No `RayHistory`.** Exact adjacency and `TrackState::entered_via` exclude the entry face; nothing is
   passed or stored between `ray_fire` calls. `smrt_dag_ray_fire` uses the context for scratch only.
2. **Deterministic direction.** `smrt_dag_point_in_volume` with a null `dir` uses the contract's
   `hashed_direction(p)` (a pure function of the bits of `p`) instead of a random direction.
3. **`dist_limit == 0` means unlimited**, as in DAGMC; infinity too; negative or NaN is `-102`.
4. **Implicit complement** is `SMRT_COMPLEMENT` (see Ids), usable as `volume` and returned by `next_vol`.
5. **Surface sense** is `+1` when the surface normal points *out of* the volume, `-1` into it (the
   DAGMC forward/reverse convention); a volume that does not touch the surface gives status 1.
6. **`smrt_dag_get_angle` takes a face id**, not a surface id: the contract's `face_normal` is
   per face and has no surface-to-face map. `smrt_dag_ray_fire` returns the face hit in the optional
   `out_face`. (They coincide in the stub; in a mesh tracker they do not.)
7. **`smrt_dag_measure_volume`** sums `cell_volume` over the region's cells in index order (NaN if any
   is unknown). **`smrt_dag_measure_area` is unsupported** (`-5`): the contract has no per-surface area.
8. **Lost particles** are `SMRT_ERR_LOST` (`-103`) in the shims, a value (`SMRT_END_LOST`) elsewhere.
9. `ray_fire` from an origin outside the model, or from outside `volume` with `EXITING`, is a miss
   (status 1) and not an error, as in DAGMC.

## Robustness

* Every entry point runs inside `guard`: `catch_unwind(AssertUnwindSafe(..))`; a panic becomes `-101` and
  its payload text goes to the thread's message. Rust 2024 aborts on unwinding out of `extern "C"`;
  nothing can unwind out because the body is entirely inside the guard.
* The success path of `smrt_step`/`smrt_peek`/`smrt_fire` does no allocation (counting-allocator test,
  lattice world, hierarchy depth 1); errors allocate for the message.
* A `Tracker` that panics (test hook `STUB_PANIC_ON_STEP`) gives `-101`, the process survives and
  the message is that thread's own.
* Safe Rust cannot check that a C pointer is valid: each `# Safety` section states the contract
  (live handle from this library, no concurrent use of a context, buffers of the stated size).

## Benchmark protocol and recorded numbers

Harness: `bench/` (README there: formats, Docker commands, result layout). Record-only: nothing
gates and no speedup is claimed. Method: 3 warm-up and 7 measured repetitions of the whole ray set,
median with min and max, CPU model recorded. DAGMC side: conda-forge `dagmc` 3.2.4 (OBB tree,
`nodoubledown`) with `moab` 5.5.1 (the build that ships pymoab), `init_OBBTree`, then the standard
`ray_fire` + `next_vol` loop to leakage with the same rays; lost particles are counted as ray_fire
error, no hit inside a bounded volume, or `point_in_volume` disagreement on a 1-in-1000 audit.

Recorded on 2026-10-26, Intel Xeon @ 2.80 GHz, 4 vCPU (x86_64 VM, Docker Engine 29.9.2; the
arm64 build was not tried here), 200 000 rays per case. Files: `bench/results/C*-20261026T114120Z-24599.json`.
The VM is noisy (min/max spread of 10 to 30 %); read the medians as magnitudes.

| Case | DAGMC 1 thread, rays/s (median [min, max]) | DAGMC 4 threads | crossings | lost | `check_watertight` |
|---|---|---|---|---|---|
| C1 icosphere shell in a box | 211 556 [167 129, 222 256] | 603 113 [578 340, 771 742] | 245 988 | 0 | pass |
| C2 five nested icospheres | 128 681 [104 528, 147 056] | 365 589 [353 361, 451 582] | 301 831 | 0 | pass |
| C3 box with spherical void | 377 739 [328 094, 403 940] | 1 109 568 [826 676, 1 534 308] | 200 000 | 0 | pass |

DAGMC `build_s` (load + `init_OBBTree`): 0.10 to 0.14 s per case. An earlier run with a generator that
mislabelled 10 C2 starts (inside the analytic shell, outside the inscribed facets) counted those as lost;
the start-region rule in `bench/README.md` removed them. The sm-rt side is not in these files
(`sm-rt` is not on main; the script records `{"skipped": ...}` and says so).

C ABI over the analytic stub (`rt_capi_bench`, same rays, sphere geometry of C1/C2/C3 replaced by
exact spheres; **not a competitor number**), 1 thread:

| Case | direct `Tracker::fire` loop, rays/s | same loop through the C ABI, rays/s | `step` direct, ns | `step` via `smrt_step`, ns | overhead per step, ns (median [min, max]) |
|---|---|---|---|---|---|
| C1 | 5.94e6 | 4.82e6 | 131.4 | 142.9 | 9.6 [4.6, 31.6] |
| C2 | 3.63e6 | 2.75e6 | 166.4 | 183.1 | 16.7 [3.2, 29.1] |
| C3 | 5.81e6 | 3.25e6 | 185.2 | 222.2 | 35.6 [21.0, 44.5] |

Run-to-run spread on this VM is large: the same measurement in a previous run gave 30.9, 22.1 and 35.0 ns.
Read it as "10 to 40 ns per step".

Cross-checks: the stub (exact spheres) and DAGMC (faceted spheres) agree on the crossing count to
14 of 246 002 (C1) and 84 of 301 915 (C2) crossings, the facet error; C3 differs by design (the
stub treats the void as a region, DAGMC as the implicit complement). The per-step overhead is a few
tens of ns on a stub whose own step costs 130 to 200 ns; against a fast mesh walker it will be a
larger share, so the ABI is for the per-particle loop, with `smrt_trace_bundle` for bulk work.

`bench/gen_case.py` is deterministic (SHA-256 of all six output files in `bench/README.md` and
`bench/expected_sha256.txt`, asserted by `run_bench.sh --check-gen`).
