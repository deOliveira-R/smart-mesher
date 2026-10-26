# sm-rt-capi benchmark harness (record-only)

Compares the DAGMC reference tracker (Docker, conda-forge `dagmc` + `moab`) with smart-mesh's
tracker on the same triangle models and the same rays, and measures the overhead of the C ABI over
the contract's analytic stub. **Nothing here is a gate and no speedup is claimed**: the numbers are
measurements with the CPU model and date recorded next to them.

## Layout

| Path | What |
|---|---|
| `gen_case.py` | Writes `facets.bin`, `rays.bin`, `manifest.json` per case (stdlib only, deterministic). |
| `expected_sha256.txt` | SHA-256 of the default output (200 000 rays per case), asserted by `run_bench.sh --check-gen`. |
| `dagmc/Dockerfile` | Image with DAGMC 3.2.4 (OBB tree), MOAB (no pymoab), the conda C++ toolchain; builds the driver. |
| `dagmc/facets_to_h5m.cpp` | MOAB C++ API: facets to h5m (surfaces, volumes, `GEOM_SENSE_2`), then `check_watertight`. |
| `dagmc/dagmc_driver.cpp` | `init_OBBTree`, then `ray_fire` + `next_vol` to leakage; JSON result. |
| `dagmc/run_case.sh` | Container entry point. |
| `run_bench.sh` | Builds the image, runs the cases, merges all JSON into `results/`. |
| `../examples/rt_capi_bench.rs` | Same rays through the C ABI over the analytic stub (`--features stubs`). |
| `results/` | One JSON per case and run, named `<case>-<UTC>-<pid>.json`, never overwritten. |

## Cases

| Case | Model | Regions |
|---|---|---|
| C1 | icosphere shell (radius 1, subdivision 5, 20 480 triangles) in the box `[-2,2]^3` | 0 ball, 1 box minus ball |
| C2 | five nested icospheres (radii 0.2, 0.4, 0.6, 0.8, 1.0, subdivision 4) in the box | 0..4 shells, 5 box minus outer ball |
| C3 | box `[-2,2]^3` with a spherical void (radius 1, subdivision 5, normals into the void) | 0 solid; the void is the implicit complement |

Rays: splitmix64 seeded with the FNV-1a 64 hash of the case name. Origin uniform in the box (rejected
inside the void for C3), direction isotropic by rejection sampling in the unit ball and
normalisation. **Start-region rule:** the spheres are icospheres (inscribed polyhedra, vertices on
the true sphere), so a point with `inscribed radius < |p| < radius` is inside the analytic ball but
outside the faceted one; such starts are rejected (a band of about 1e-3 of the radius, computed
exactly from the triangle planes), so every start label is correct for both the faceted model and
the analytic ball. Without this rule 10 of 200 000 C2 starts were mislabelled and DAGMC reported them
as lost. Only IEEE-exact operations are used, so the bytes are identical on every platform.
Default 200 000 rays per case (`--rays N` to change; `--check-gen` only matches the default).

## File formats (little-endian)

**smfacets v1** (`facets.bin`)

| Bytes | Content |
|---|---|
| 8 | `u32 nv`, `u32 nt` |
| `nv * 24` | vertices, 3 x `f64` |
| `nt * 12` | triangles, 3 x `u32` vertex indices (right-hand rule gives the normal) |
| `nt * 8` | 2 x `i32` per triangle: **forward** and **reverse** region. Forward is the region the normal points *out of* (DAGMC forward volume, sense `+1`); `-1` is the implicit complement. |

**smrays v1** (`rays.bin`)

| Bytes | Content |
|---|---|
| 4 | `u32 n` |
| `n * 48` | per ray: origin (3 x `f64`) then unit direction (3 x `f64`) |
| `n * 4` | `i32` start region per ray (`-1` complement) |

`sm-rt/examples/trace_rays.rs` parses the same files. Its command line is
`trace_rays <facets.bin> <rays.bin> [--threads N]` and it prints one JSON object
`{build_s, rays, rays_per_s, crossings_per_s, lost, threads, cpu_model}`.

## Running

```sh
crates/sm-rt-capi/bench/run_bench.sh --check-gen          # data generation is reproducible
crates/sm-rt-capi/bench/run_bench.sh                       # everything below, all cases
crates/sm-rt-capi/bench/run_bench.sh --cases C1 --threads 4 --rays 50000
crates/sm-rt-capi/bench/run_bench.sh --skip-docker         # Rust side only
```

Docker commands the script runs, for reference:

```sh
docker build -t smrt-dagmc-bench crates/sm-rt-capi/bench/dagmc
docker run --rm -v "$PWD/crates/sm-rt-capi/bench/data:/data:ro" -v "$OUT:/out" smrt-dagmc-bench C1 1 4
```

`run_bench.sh` builds the image (behind an HTTPS proxy it passes `HTTPS_PROXY` and the proxy CA
bundle as build inputs), converts each case with `facets_to_h5m`, runs `dagmc_driver` with 1
thread and N threads, runs the C-ABI stub example, and runs `cargo run --release -p sm-rt --example
trace_rays` when `crates/sm-rt/examples/trace_rays.rs` exists (otherwise it says so and records
`{"skipped": ...}`). The image uses the base image's native architecture (x86_64 and aarch64 are both
supported by conda-forge).

## Method

* 3 warm-up and 7 measured repetitions of the whole ray set; median with min and max.
* A ray is followed to leakage (entering the implicit complement). `crossings` counts region
  crossings including the final one. Rays are independent, so the counts do not depend on threads.
* DAGMC loop: `ray_fire(volume, pos, dir, &surf, &dist, &history, 0, 1)`, `next_vol`, advance, keep
  the `RayHistory`. **Lost** particles are counted as: `ray_fire`/`next_vol` error; no surface hit
  from inside a bounded volume; or disagreement of `point_in_volume` on a 1-in-1000 audit (the start
  point must be in its start volume; a point `1e-6` past a crossing along the ray must be in the
  volume `next_vol` reported). The implicit complement is never audited: it has no bounding
  surface (no graveyard), so DAGMC's `point_in_volume` is not defined for it.
* `build_s`: DAGMC `load_file` + `init_OBBTree`; for the Rust side the tracker build.
* CPU model: `model name` from `/proc/cpuinfo` (or `lscpu`).

## Reading a result file

```jsonc
{ "case": "C1", "run_id": "...", "date_utc": "...",
  "dagmc":   { "t1": {"rays_per_s": {"median", "min", "max"}, "crossings_per_s": {...}, "lost": 0,
                      "lost_breakdown": {...}, "build_s": ..., "cpu_model": "..."}, "tN": {...} },
  "dagmc_build": { "watertight": true, "surfaces": 2, "volumes": 2, ... },
  "sm_rt":   { "t1": {...}, "tN": {...} }   // or {"skipped": "..."}
  "capi_stub": { "t1": {"rays_per_s_rust", "rays_per_s_capi", "step_ns_direct", "step_ns_capi",
                        "step_overhead_ns", ...} } }
```

Compare `dagmc.tN.rays_per_s.median` with `sm_rt.tN.rays_per_s` only when both ran on the same
machine in the same session (`cpu_model` and `date_utc` match). `capi_stub` is an analytic
sanity/overhead number (brute force over one or five spheres); it is not a competitor number.

## Recorded SHA-256 (default 200 000 rays)

```
6b2e378f02c9d2f112f5ca47cf66f7eb7a7f546cc34c5811b591d136d877d2c4  C1/facets.bin
42f6c62bae567e93724d60fddd5e682b8307a932cc33b0e8dd4c533a94c66047  C1/rays.bin
cd4171181096f065ef35c578020776cbc6fca4427d1e12339d29fc6b21db2b94  C2/facets.bin
a4bad00372dbb1d3200ed2c418cce0387579de684ad39c5772615493055adc54  C2/rays.bin
de41d3385e45b7545bb13c226e68f12de488e0467b80608139da64af65438ef1  C3/facets.bin
93c78ca90f72004e4943b2de7a62be7f9cf931fbbbfee0968af45ba1afc39209  C3/rays.bin
```
