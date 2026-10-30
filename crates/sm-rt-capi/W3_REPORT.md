# W3_REPORT: sm-rt-capi

**Branch / SHA.** The session pinned the branch `claude/w3-sm-rt-capi-jc6kud`; the task text says
`claude/w3-sm-rt-capi`. Everything is pushed to the session branch. Merge by SHA (the last commit,
subject `w3 sm-rt-capi: done ...`).

**Status: done for the mandatory scope and for stretch S2; acceptance 8 (DAGMC numbers) was run on an
x86_64 machine, not arm64.** Stretch S1 (wire `smrt_open` to sm-rt), S3 (Embree), S4 (OpenMC), S5
(segment callbacks) were not attempted.

## Summary

* `libsm_rt_capi` (cdylib + staticlib + rlib): 26 exported `smrt_*` functions over `Arc<dyn Tracker>`,
  `include/smrt.h`, `fortran/smrt.f90` (checked with gfortran 13), `include/smrt.hpp` (S2, C++17 RAII).
* DAGMC shims, bundle codec, thread-local error message, panic containment, no-allocation hot path.
* Bench harness (`bench/`): deterministic case generator, DAGMC image, MOAB C++ converter, driver,
  `run_bench.sh`, and DAGMC numbers for C1-C3 measured and committed in `bench/results/`.
* Design, ABI table, status codes, DAGMC differences, protocol and numbers: `DESIGN.md`.

## Acceptance -> test

| # | Bullet | Test (all in `crates/sm-rt-capi/tests/`) |
|---|---|---|
| 1 | C program, chords b in {0,3,4.9}, nested shells, bit-exact mirror, White 1e5 | `c_abi::c_chords_lattice_mirror_and_white` (C source `tests/c/chords.c`, built with `cc`, linked to the cdylib); Rust twins `abi::sphere_chords_match_analytic_values`, `nested_shell_chords_sum_to_the_outer_chord`, `mirror_deck_reflects_bit_exactly`, `white_boundary_returns_unit_inward_vectors`, `reflective_albedo_and_vacuum_responses` |
| 2 | 2-level lattice chord sum + `SlotChange` reported by step, skipped by fire | `c_abi::c_chords_lattice_mirror_and_white` (lattice part), `abi::lattice_chord_sum_and_slot_change_skipping` |
| 3 | DAGMC shims on the stub sphere | `abi::dag_shims_on_the_sphere`, `dag_point_in_volume_agrees_with_geometry_for_1e5_points`, `surface_sense_is_consistent_with_next_vol`; C: `c_abi::c_dag_shims_status_and_panic_containment` |
| 4 | Null table, panic -> -101, per-thread message, `RtError` codes | `status::every_function_returns_null_status_for_a_null_pointer` (28 of the 29 functions, `smrt_version` has no pointers, x every required pointer), `a_panic_in_step_is_contained_and_reported_per_thread`, `every_rt_error_maps_to_its_distinct_negative_code`, `errors_reach_the_caller_with_messages`; C side in `dag.c` |
| 5 | `smrt.h` / `smrt.f90` vs exports and layouts; gfortran | `header::header_and_fortran_list_exactly_the_exported_symbols`, `struct_sizes_and_offsets_match_the_header` (generated `_Static_assert`s from `size_of`/`offset_of`, compiled by `cc`; header also compiled as C++), `fortran_module_compiles_and_runs_when_gfortran_exists` (compiles the module with `-std=f2008 -Wall`, runs `version.f90` and a chord program `chord.f90` that also checks struct sizes; prints `SKIPPED ...` when gfortran is absent). I installed gfortran in this VM, so the sub-test ran here. |
| 6 | Bundle bytes 1 vs N threads, equal to Rust `content_hash`, determinism harness | `bundle::bundle_bytes_equal_across_threads_blocks_and_the_rust_path` (1e5 rays; 1 vs 4 threads; block sizes 0/1/777/2^20; equals `TrackBundle` and its `content_hash`), `bundle_passes_the_testkit_determinism_check` (`sm_testkit::determinism::check`), `determinism::step_streams_are_bit_identical_across_runs_and_thread_counts`; unit tests of the codec in `src/bundle.rs`. All suites also pass with `RAYON_NUM_THREADS=1`. |
| 7 | No allocation in `smrt_step` after ctx creation; overhead recorded | `alloc::step_peek_fire_allocate_nothing_after_context_creation` (counting allocator; step, peek, fire, set_direction; lattice world). Overhead: below. |
| 8 | Bench harness | `run_bench.sh --check-gen` (passes, six SHA-256 in `bench/expected_sha256.txt` and `bench/README.md`); `docker build` of `dagmc/Dockerfile` succeeded on x86_64 (round 0 of the worker) and, after review round 0, on this macOS arm64 machine (linux-aarch64 image, no pymoab: the converter is now `dagmc/facets_to_h5m.cpp` on MOAB's C++ API); `dagmc_driver` ran C1, C2, C3 end to end on x86_64 (`bench/results/C{1,2,3}-20261026T114120Z-24599.json`) and C1 end to end on arm64 (`bench/results/C1-20261026T130138Z-18917.json`: watertight, 245 988 crossings, 0 lost, same crossing count as x86_64); the Rust stub side ran. |
| 9 | Lints, docs, unsafe confined | `cargo clippy -p sm-rt-capi --all-targets --all-features -- -D warnings` and without features: clean; `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-rt-capi --no-deps --all-features`: clean; `header::no_unsafe_outside_the_ffi_module` (keyword grep outside `src/ffi/`; inside, every block has a `SAFETY` comment and every `unsafe fn` a `# Safety` section) |
| S2 | C++ RAII wrapper | `c_abi::cpp_raii_wrapper` (`tests/c/raii.cpp`) |

## What changed in review round 0

* **Test harness is portable (macOS arm64 and Linux).** The cdylib name comes from
  `DLL_PREFIX`/`DLL_SUFFIX`, the C/C++ compiler is configured for the host triple (no forced Linux triple),
  rpath is the `-Wl,-rpath` form both linkers accept. `cargo test -p sm-rt-capi --all-features` passes on
  macOS arm64, including the C, C++ and gfortran tests.
* **Acceptance 8 on arm64.** Dockerfile without pymoab, C++ converter, C1 run on arm64 (see table row 8).
  Numbers from the arm64 Docker VM are not comparable with the x86_64 ones above (different CPU, emulated
  by a VM, `cpu_model` is "unknown"); no speedup over DAGMC is claimed.
* `run_bench.sh`: `--check-gen` falls back to `shasum -a 256 -c`; thread default falls back to
  `sysctl hw.ncpu` / `getconf` before 4.
* **Bulk bundles trace once.** New `smrt_trace_bundle_new` / `smrt_bundle_words` / `smrt_bundle_free`
  (opaque `SmrtBundle` handle; C header, Fortran interfaces, tests). `smrt_trace_bundle`'s size-query
  pattern still works but traces twice; its doc now says so. 29 functions are exported.
* `smrt_set_state` checks the cell id against `cell_count()` and returns `SMRT_ERR_UNKNOWN_ID` (-4).
* `smrt_version` now goes through `guard` (catch_unwind), so the rule holds at every entry point.
* Declined: strengthening the SAFETY-grep test window (the reviewer counted all 41 blocks by hand; the
  test stays as is). `start_cell` in `SmrtRays` is still accepted and ignored (the contract's
  `trace_bundle` has no hint); stays on the W3b list.

## What changed in review round 1

* **Fixed:** `smrt_set_state` rejected `SMRT_COMPLEMENT` as a cell (`CellId::COMPLEMENT` failed the
  `cell_count()` bound), breaking the `smrt_get_state` -> `smrt_set_state` round trip for trackers with an
  implicit complement. The bound check now exempts `CellId::COMPLEMENT` when the tracker has an implicit
  complement. Test added in `tests/abi.rs`.
* Report: Known gaps and Measured values updated (cross-platform tests, arm64 C1 file kept apart).
* Declined: stricter SAFETY test; it already checks every `unsafe {` block in `src/ffi` for a SAFETY comment
  within 4 lines, and the reviewer verified all blocks by script.

Definition of done run here: `cargo build --workspace`; `cargo test -p sm-rt-capi --all-features`;
`RAYON_NUM_THREADS=1 cargo test -p sm-rt-capi --all-features`; clippy; rustdoc: all pass. The
"cold replay from root" item does not apply: the crate registers no ops.

## Measured values

All numbers in this section are from the Intel Xeon @ 2.80 GHz, 4 vCPU, x86_64 machine, 2026-10-26. The one
arm64 file is separate: `bench/results/C1-20261026T130138Z-18917.json` (macOS arm64 Docker VM, linux-aarch64
image): DAGMC C1 median 139.6k rays/s (1 thread), 227.7k rays/s (4 threads), 245 988 crossings, 0 lost. Do not
compare it with the x86_64 numbers. The VM is noisy; spreads of 10 to 30 % between
min and max repetitions are normal here.

* **DAGMC (conda-forge dagmc 3.2.4 OBB tree + moab 5.5.1), 200 000 rays, 3 warm-up + 7 reps, median [min, max] rays/s:**
  C1 211 556 [167 129, 222 256] (1 thread), 603 113 [578 340, 771 742] (4 threads);
  C2 128 681 [104 528, 147 056], 365 589 [353 361, 451 582];
  C3 377 739 [328 094, 403 940], 1 109 568 [826 676, 1 534 308].
  Lost: 0, 0, 0 (counted as ray_fire error, no hit inside a bounded volume, or point_in_volume audit
  disagreement on 1-in-1000: 243, 328 and 200 audits).
  `check_watertight` passes for all three models. Build (`load_file` + `init_OBBTree`) 0.10-0.14 s.
  *A first run reported 10 lost rays on C2.* Diagnosis, with evidence: all ten failed at step 0 with
  `point_in_volume(start_volume, start) == 0`, and their `|p|` was 0.99923, 0.79979, ... just below the
  shell radii 1.0 and 0.8: inside the analytic ball the generator used for the start label, outside the
  inscribed icosphere. A generator bug, not a DAGMC loss. `gen_case.py` now rejects starts in the band
  `inscribed radius < |p| < radius` (rule in `bench/README.md`); C2 rays and the SHA-256 table changed
  (C1 too; C3 did not), and all numbers here are from the rerun.
* **C ABI over the analytic stub (1 thread):** per-step overhead of `smrt_step` over `Tracker::step`:
  C1 9.6 ns [4.6, 31.6], C2 16.7 ns [3.2, 29.1], C3 35.6 ns [21.0, 44.5] (stub step 131-185 ns); the previous
  run of the same code gave 30.9 / 22.1 / 35.0 ns, so read it as 10 to 40 ns on this noisy VM.
  Rays/s through the ABI vs the direct trait loop: C1 4.8e6 vs 5.9e6, C2 2.8e6 vs 3.6e6, C3 3.3e6 vs 5.8e6.
  This is an analytic brute-force stub: an overhead number, not a competitor number.
* DAGMC vs stub crossing counts: 245 988 vs 246 002 (C1), 301 831 vs 301 915 (C2): the difference is the
  facet approximation of the spheres.
* The sm-rt side of the comparison is absent from the result files (sm-rt is not on main; the files say
  `{"skipped": ...}`). No speedup is claimed anywhere.

## Deviations from the task text (with reasons)

1. **Context owns the state; `SmrtState` is read and written with `smrt_get_state` / `smrt_set_state`.** The
   frozen contract's rustdoc sketches `smrt_step(ctx, state, ...)`; the task text (ctx owns `TrackState`) wins.
   Added `smrt_get_state`, `smrt_set_state`, `smrt_set_direction` (a code banking particles or scattering needs them).
2. **Ids are `int64_t`** (`-1` none, `-2` `SMRT_COMPLEMENT`), not `uint32_t`, because the header may only use
   `int32_t/int64_t/uint64_t/double`. The implicit-complement *region* is always spelled `SMRT_COMPLEMENT` on output,
   and accepted as `-2` or by raw id on input (the stub's complement is a real region, not `CellId::COMPLEMENT`).
3. **`smrt_resolve_boundary(ctx, face, kind, albedo, u0, u1, out_dir)`** has an extra `albedo` argument: the
   task signature has no way to carry `ResponseKind::Albedo(a)`.
4. **`smrt_trace_bundle(tracker, n, rays, block_size, opts, out_buf, cap, out_len)`** has an extra `opts`
   (nullable `SmrtTraceOpts`: max distance, step cap, stop at deck). `rays` is a `SmrtRays` struct of seven
   SoA pointers. `start_cell` is accepted and ignored (the contract's `trace_bundle` has no hint). Output buffer
   layout is defined in `src/bundle.rs` / `DESIGN.md`; too small a buffer is `-102`, so the size query traces twice.
5. **`smrt_dag_ray_fire` has a trailing nullable `out_face`; `smrt_dag_get_angle` takes a face id**, not a surface
   id (see ACR 2). `smrt_dag_measure_area` returns `-5` (ACR 1). `smrt_dag_measure_volume` is the cached sum of
   `cell_volume` (NaN if any cell volume is unknown).
6. **Status 1 semantics** pinned in `DESIGN.md`; `SMRT_ERR_LOST = -103` added for the shims.
7. **`SmrtRegionInfo.color_id` is always `-1`**: the contract's `RegionInfo` has an entity and a name only.
   `original_id` is the model entity's serial.
8. **`smrt_open` always returns `-5` in W3a**, with or without the `sm-rt` feature (see Known gaps). The feature
   exists as an empty feature so the orchestrator's manifest line has a name to point at.
9. **Crate deps beyond the Deps list:** `sm-core` and `sm-geom-api` (contract crates; `Point3`, `Vector3`, `Ray`
   appear in `Tracker` signatures and are not re-exported by `sm-rt-api`), `rayon` (G2 allow-list; block-parallel
   bundles). Dev: `cc` (pre-approved), `sm-testkit` (determinism harness). No `libc`. `Cargo.lock` gained one
   package entry (this crate's dependency list); no other lock change.
10. **Docker.** The image uses the base image's native architecture and is built and run on both x86_64
    (worker) and linux-aarch64 (review round 0). The base image and the plan's `mamba install -c conda-forge
    dagmc moab` cannot be used as written: (a) the sandbox proxy blocks `prefix.dev`, which the miniforge
    channel alias resolves to, so the Dockerfile uses the channel as the URL `https://conda.anaconda.org/conda-forge`;
    (b) the `dagmc` default solve picks the double-down/Embree build, which is the S3 rival, not the reference, so
    the environment pins `dagmc=*=*nodoubledown*` (plain OBB-tree DAGMC) next to `moab` and `eigen` (needed by
    the MOAB headers). The first version pinned `python=3.12 moab=*=*py312*` for pymoab; that cannot be solved
    on linux-aarch64 (only moab 5.6.0 nompi builds, no pymoab), so pymoab is gone and `facets_to_h5m.cpp`
    writes the h5m through MOAB's C++ API (same surface/volume/`GEOM_SENSE_2` rules, same `check_watertight`
    report). The versions now are whatever conda-forge solves (aarch64: dagmc 3.2.4, moab 5.6.0).
    Behind a proxy `run_bench.sh` passes `HTTPS_PROXY` and the proxy CA as build inputs (`--network=host`).
11. Task text says `micromamba`; ruling R4 says `condaforge/miniforge3` + `mamba`; used the latter.

## Known gaps

* `smrt_open` cannot load anything. `TrackerBuilder::load(input, opts, image)` needs the original `BuildInput`
  and options, so a `.smrt` path alone cannot produce a tracker through the contract as frozen. sm-rt needs to
  export a loader that goes from a `CacheImage` (or a path) to `Box<dyn Tracker>` without the input (its task text
  promises "`load(image)` rebuilds the tracker without the mesh"). The wiring is then a manifest line plus a
  ~10-line function in `src/ffi/info.rs` behind `cfg(feature = "sm-rt")`; I did not write it against an API that
  does not exist yet.
* The sm-rt half of the benchmark (`trace_rays`) has never run: sm-rt is not on main. `run_bench.sh` calls
  `cargo run --release -p sm-rt --example trace_rays -- <facets.bin> <rays.bin> [--threads N]` when the file exists
  (CLI documented in `bench/README.md`); the sm-rt worker should confirm it matches.
* The `step` overhead numbers are noisy (shared VM); a quiet machine should re-run `run_bench.sh`.
* `smrt_set_state` cannot set a hierarchy deeper than 8 levels; `smrt_get_state` reports the true depth.
* After a caught panic the context's particle state is unspecified; re-initialise it.
* The Fortran module, header, and C/C++ tests build and pass with gfortran/cc on Linux x86_64 and macOS arm64
  (the tests find the cdylib via `DLL_PREFIX`/`DLL_SUFFIX` and use the host triple). Windows is untested.

## api_change_requests

None blocks the work; all are additive suggestions to the frozen `sm-rt-api` (`status` stays `done`).

1. **Surface area.** `fn surface_area(&self, s: SurfaceId) -> Result<Option<f64>> { Ok(None) }` as a provided method
   on `Tracker` (sum of facet/face areas per surface, in id order). Needed for DAGMC `measure_area`;
   `smrt_dag_measure_area` returns `-5` until it exists.
2. **Surface to face.** Either `fn surface_normal(&self, s: SurfaceId, p: Point3) -> Result<Vector3>` or
   `fn surface_faces(&self, s: SurfaceId) -> &[FaceId]`. DAGMC's `get_angle(surface, xyz)` takes a surface;
   the contract's `face_normal` takes a face, so the shim takes a face id (reported by `smrt_dag_ray_fire`'s `out_face`).
3. **Region attributes.** `RegionInfo` has no colour id or original (CAD) id besides `entity`; if a transport code
   needs them (DAGMC `id_by_index`, material groups), add `pub tags: Vec<(String, i64)>` or similar.
4. **Cache loader without the build input** (sm-rt side or contract): see Known gaps.

## Polish

* **Fixed:** `smrt_set_state` now bound-checks the region id against `regions().len()` (an id beyond the
  model returns `SMRT_ERR_UNKNOWN_ID`, -4, and leaves the state unchanged). Before, only the cell id was checked.
* **Tests:** `smrt_set_state(SMRT_COMPLEMENT)` (as cell and as region) on a tracker without an implicit
  complement returns -4 and keeps the state (`set_state_complement_without_a_complement_is_unknown_id`); an
  out-of-range region id is rejected in `locate_closest_state_and_normals`. The stub worlds all have a
  complement, so a test-hook kind `STUB_NO_COMPLEMENT` (98) wraps a world with the flag cleared.
* Declined: none.
