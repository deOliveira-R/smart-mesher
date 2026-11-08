# sm-rt-bench design

The tracking benchmark of smart-mesh against DAGMC (MOAB, double-down/Embree) and XDG. RTBENCH-A
holds the generator, formats, agreement checker, fairness gates, result schema, report, self-test and
the Dockerfiles; RTBENCH-B (below, "sm-rt rows") adds the sm-rt adapters (facets, analytic through
RT-BRIDGE, hierarchy, delta and block flights) and the Docker competitor runs. Results of this
wave are macOS smoke rows; the published table waits for the Linux x86_64 machine (O11).

Acceptance is reproducibility and fairness, not winning. Nothing here gates on wall-clock time,
no speedup is claimed in code, docs or report text, and every number in a report is produced by
the harness.

## Fairness rules (each enforced by code or a test)

| Rule | Enforcement |
|---|---|
| (a) same bytes in: every tracker reads the same ray file and the same facet or tet file | the generator writes them once; every file has a SHA-256 in `smcase.json`; `smrt-bench gen --check` compares with `expected_sha256.txt` |
| (b) same session and machine: trackers interleaved ABAB per repetition (fixed index order, not alternated to ABBA: a known first-position bias, identical for every case, left as is); no comparison across machines or days | `runner::run_interleaved` (call order asserted from the adapters' own log); `report::render` refuses different `cpu_model`, OS/arch, `machine_id` or UTC day |
| (c) best effort for each side: hinted start for DAGMC and sm-rt alike; location rows separate | `Workload::Fire` (hinted, `prepare` untimed) and `Workload::FireLocate` are separate rows |
| (d) precision is a column | `Row::precision`; double-down is `mixed`, sm-rt is `f64` |
| (e) geometry fidelity is measured | `geom_error` per row: max and mean chord error and region-sequence failures per 1e5 tracks against `truth::Scene` |
| (f) memory and build time carry the same weight as rates | `structure_bytes`, `peak_rss_bytes`, `build_s`, `load_s`, `tree_s` columns |
| (g) a comparison is shown only if the agreement check passed | a tracker with `agreement.failed > 0` in a case renders `n/c (disagrees)` in every rate cell of that case; the counts stay visible |
| (h) cell walking alone is not claimed novel; rows list what they do not do | `report::NOT_MEASURED` is printed in every report |
| Q9: published tables come from one bare-metal Linux x86_64 machine | any other OS/arch is rendered under a `SMOKE RESULT` banner; x86_64 and aarch64 are never merged |

## Cases

`C1..C3` (L1 legacy) are regenerated natively and equal the W3a files byte for byte (the six
SHA-256 of `sm-rt-capi/bench/expected_sha256.txt` are asserted in `tests/cases.rs`). The other
cases:

| Case | Regions | Facet twin | Notes |
|---|---|---|---|
| `P1` | fuel 0, clad 1, water 2 | `sm_tess` of the `pin_assembly` base pin plus a hand-built planar moderator | r 0.4096 / 0.475, pitch 1.26, height 100; vacuum outside |
| `P2` | fuel 0, clad 1 | the flattened placements of `pin_assembly(17, ..)` | `9 n^2` = 2601 copies of one prototype (a 3x3 cluster of 17x17 lattices, no guide tubes, no plates, no moderator); triangles = copies x triangles of one pin, asserted |
| `H1` | inclusions 0..7, water 8 | cube, boxes, icospheres (hand-built) | 200 cm cube; 4 boxes (grid aligned, conforming in the tets) and 4 spheres (stair-stepped by tet centroid); Kuhn tets, `h1_cell` 4 cm (about 7.5e5 tets) |
| `F1` | shells 0..17 | nested full torus shells, R 600, minor radii 5..200 | synthetic; ports and the cut plane are not generated; at sag 0.05 it has about 1.8e6 triangles |

Start labels are exact for the facet twin and the analytic model: points inside a band around a
curved surface where the two can differ (the recorded `sag` for inscribed polygons, one cell
diagonal for the stair-stepped spheres of H1) are rejected, as in the W3a generator.

`Scale::{full, reduced, tiny}` keep the geometry families and shrink sizes for tests.

### Bytes and platforms

Rays, tets and the hand-built facets use only IEEE-exact operations. Facets from `sm_tess` use
`f32` positions and the platform's `sin`/`cos`, so `P1`, `P2`, `F1` facets, their rays (the
bounding box comes from them) and every `smcase.json` (it carries the platform tag) are recorded
in `expected_sha256.txt` **per `os-arch`** (`...@macos-aarch64`). The first run on the Linux x86_64
machine prints the missing lines with `smrt-bench gen --scale <s> --print`; until they are added
there `gen --check` reports them `unblessed` (exit code 2) and the test reports them without failing (`recorded_hashes_match...` still fails on any mismatch and on
any unblessed platform-independent file). Blessing Linux lines needs a run there and an orchestrator
ruling. This crate
does not claim identical tessellations across platforms.

## Formats

All little-endian. `smfacets v1` and `smrays v1` are those of `sm-rt-capi/bench/README.md`;
`smtets v1`: `u32 nn, u32 nt`, `nn` x 3 `f64`, `nt` x 4 `u32`, `nt` x `i32` region. `smcase.json`
(`smcase v1`) holds regions with the benchmark's own `sigma_t`/`sigma_a` (a transport workload, never an
input of sm-rt: ruling Q2), `sag`, the chord tolerance, bounding box, counts, `platform`, notes
and the SHA-256 of every other file. Readers check lengths and indices and return `BenchError`.

## Agreement checker

`compare::compare(judged, reference, diag, chord_tol)`: per track the region sequences must be
equal after dropping chords shorter than `5e-9 diag` (grazing: counted and listed, never hidden;
neighbours of equal region are merged), and chords must agree within `chord_tol =
max(1e-9 diag, 8 sag)`. The first 1000 disagreements go to `disagreements.jsonl` with the ray index
and both sequences (`create_new`).

## Result schema `smbench-result v1`

One file per case and tracker set, `results/<case>-<trackers>-<UTC>-<pid>.json`, opened with
`create_new`. Rows hold the columns of the draft (rates as median/min/max of the timed
repetitions, `segments_per_track`, `ns_per_segment`, build and memory columns, `lost`,
`geom_error`, `agreement`, `determinism`, counters). `result_hash` covers case, scale and the
non-timing fields, so two runs compare by one string. `bench-env.json` records the toolchain and
the machine id (a hash of CPU model, CPU count, OS and arch; no host names).

`--model PATH` (a local STEP file) imports through `sm-cadio`, tessellates with `sm_tess` and runs
on the facets. The model path and bytes are never recorded; results of a model run are refused
when the output directory is inside a git work tree (`result::inside_repo`; `.git` as a directory
or a file); `--dry-run` writes nothing at all. The crate `.gitignore` lists `*.step`, `*.stp`,
`*.h5m`, `models/` and `local/`.

## sm-rt rows (RTBENCH-B)

`smrt-bench run --trackers smrt` builds every row that the case allows (`crate::smrt`); a row that
cannot be built is listed under "Partial" in the report with the reason, never dropped silently.

| `mode` | Tracker | Geometry | Workloads |
|---|---|---|---|
| `facets` | `MeshTracker` over the facet twin | the triangles DAGMC and XDG hold | W-track, W-fire (hinted and located) |
| `analytic` | `AnalyticTracker` | exact spheres, cylinders and planes; P2 through `sm_rt::csg_world` (RT-BRIDGE), flattened | W-track, W-fire; W-flight in its own group |
| `hierarchy` | `WorldTracker` | P2: one pin prototype (facets) in the model's own lattice and cluster | W-track, W-fire; `pins_touched` |
| `walk`, `delta`, `block` | `MeshTracker` with a majorant partition | tet twin: H1's own mesh, else a stair-stepped Kuhn mesh of the scene | W-flight |

Rows of one tracker name (`smrt`) differ by `mode`; the report keys the fairness gates by
`(tracker, mode)`. Region ids are mapped back from the tracker's regions (model entity for facets
and meshes, region name for CSG cells), so every row speaks the manifest's ids.

* **W-track** ends at the first entry into the complement, as the truth scene does; sm-rt traces the
  whole line (the work is done, the timing includes it), DAGMC's loop stops at the complement, and the
  `segments` counter counts the segments up to that entry, so `segments_per_track` is comparable.
* **Flights** use `FlightCursor`: surface segments sample with the benchmark's own `sigma_t`, delta
  segments sample with the node majorant (the largest `sigma_t` of the regions of the node's set)
  and locate every tentative collision. `block` is the caller-driven mixed policy (ruling O3): delta
  everywhere except the leaves that hold more than one region, which are blocked to surface
  segments. The `walk` row steps cell by cell with `Tracker::step` (no partition). The delta modes
  leave the track-length slots of the tally at zero except in surface segments.
* **Counters** (work, never time) per row: `surface_segments`, `delta_segments`, `tentative`
  (as sm-rt counts them), `rejected` (virtual collisions, counted by the caller: sm-rt cannot know the
  verdict), `node_visits`, `face_tests`, `bvh_descents`, `lattice_index_locates`, and `pins_touched`
  (hierarchy W-track: the start element plus one per `SlotChange`, counted untimed in `prepare`).
  They join `result_hash` and the determinism hash only when non-zero, so rows of the stub keep
  their hashes.
* **P2 has no moderator** (pins stand in vacuum), so a ray of P2's own twin ends in its first pin
  and `pins_touched` is one per ray. The element counter is therefore checked on
  `smrt::hierarchy_moderated`: the same lattice and cluster filled with P1's moderated pin cell, against
  an independent grid walk (`tests/smrt.rs`, `dda_cells`): touched elements never exceed the cells the
  ray's projection crosses.
* **Analytic limits**: a cell is a solid minus the solids one level inside it, so a box inside a
  solid needs a union of half-spaces and is `Unavailable` (H1 has no analytic row); a cylinder inside a
  solid must span the parent's z range. The tracker builds no hierarchical CSG worlds
  (`RtError::Unsupported`), so the analytic P2 row is the flattened bridge world.
* **Zero-area triangles** (fixed in W3c): at scale `tiny` a ring point lay exactly on the diagonal of the
  square cell, so `twin::add_moderator` added a corner vertex on top of it and emitted zero-area
  triangles, which the tracker refuses (dropping them opened the shell, so the `facets` row of P1 was
  `partial`). The corner is now not added when it coincides with a ring point; the polyline is the same.
  Only `tiny/P1/facets.bin` and `tiny/P1/smcase.json` hashes changed (`expected_sha256.txt`); `reduced`
  and `full` bytes are identical, and a test asserts no zero-area triangle at either scale.

### Agreement and `geom_error`

An adapter on the exact geometry (`Adapter::exact`) is judged against the analytic truth. Adapters on a
facet or mesh twin are judged against the first of them (they hold the same triangles; their distance
from the truth is the `geom_error` column, not a disagreement). Flight tallies are judged against
the first adapter of the group by `runner::compare_tallies`: per-region collision counts, leaked and
absorbed histories, 4 sigma, with the variance estimated from the spread of the 256-history blocks
(identical blocks for every adapter and thread count).

**Twin bound** (`bound.rs`, acceptance 3 second half). A facet twin is inscribed: at a curved
surface the facet plane lies inside it by at most `sag`. A ray crossing at angle `theta` to the normal
reaches the facet plane at most `sag / cos(theta)` from the exact crossing, with
`cos(theta) = rho sqrt(1 - b^2 / r^2)` for a vertical cylinder (`rho` the horizontal speed, `b` the
horizontal offset from the axis) and `sqrt(1 - b^2 / r^2)` for a sphere. The bound of a ray is the
sum over all its crossings of curved surfaces plus `1e-9 diag`; flat faces are exact. A ray with
`cos(theta) < 0.05` at some crossing is grazing and is counted, not judged. `compare_bounded`
requires equal region sequences and chords within the bound; if that fails it may drop the chords
that could vanish in the twin (shorter than `2 bound`, then `4 bound`) with the tolerance widened by
`bound * (1 + 2 dropped)`. On P1 and P2 at scale `reduced` every judged ray passes.

### Flight unbiasedness (acceptance 6)

A pure absorber sphere (`Sigma` 0.3, `R` 5), starts uniform in the ball, isotropic. Expected
transmission per start: analytic row, `exp(-Sigma d)` with `d` the distance to the sphere; mesh rows
(stair-stepped sphere in a 16^3 Kuhn block), `exp(-Sigma L)` with `L` the chord the same mesh gives
that start. The variance is the exact binomial sum `sum p (1 - p)`; surface, delta and `block` are
within 4 sigma. The run in CI has 2e5 histories; the 1e6 run is `#[ignore]` (`--ignored --release`).

## Protocol for C++ drivers

Each driver prints one JSON `Row` per line on stdout (`#[serde(default)]`: omitted fields are
zero) and, once per run, a line `{"tracks": [{"r": [...], "c": [...]}, ...]}` with every ray's region
ids and chords in ray order, which the runner compares with the sm-rt facet row (agreement) and with
the analytic truth (`geom_error`). The container entry point is `run_case.sh <case> <threads>...`
with the case directory mounted read-only at `/data/<case>`; `SMRT_REPS` and `SMRT_WARMUP` come from
`run`, `SMRT_TRACKER`, `SMRT_MODE` and `SMRT_PRECISION` from the image. Rows carry rates (median,
min, max of the timed repetitions), `segments_per_track`, `ns_per_segment`, `build_s` split into
`load_s` and `tree_s`, `peak_rss_bytes`, `lost`, a determinism hash over the outputs in ray order
(`stable`: equal across the repetitions) and counters. To add a competitor: a Dockerfile under
`docker/<name>/`, a pinned entry in `docker/pins.env`, a driver under `drivers/<name>/`, an entry in
`docker::IMAGES`.

### W-fire and W-flight in the DAGMC drivers (W3c BENCH-WORKLOADS)

`dagmc_driver --workload track|fire|fire-locate|flight` (default `track`); `run_case.sh` loops over
`SMRT_WORKLOADS` (comma list) and the thread counts, and `run --workloads` (default all four)
sets it. After a row the driver prints, for the first thread count only, the dump of its workload:
`{"tracks": ...}`, `{"fires": [[dist, to_region], ...]}` (to-region `-1` complement, `-2` lost) or
`{"tally": [...], "tally_sq": [...], "blocks": B}`; the runner attaches a dump to the row printed
before it ([`docker::parse_driver_output`]).

* **Fire** is one `ray_fire` per ray from the region the ray file names (hinted, no ray history) up
  to the first region boundary. `fire-locate` first finds the start volume by a loop over the
  volumes with `point_in_volume`, then the implicit complement: DAGMC's own `find_volume` is not
  exported by libdagmc 3.2.4 (the declaration sits behind `MOAB_VERSION_*` macros no installed
  header defines), so the same linear loop is written in the driver. The fire rows are judged ray
  by ray against the sm-rt facets fire ([`compare::compare_fires`]: distances within `chord_tol`,
  regions equal, a ray with a distance below `5e-9 * diag` is counted as grazing, not failed).
* **Flight** reimplements the Rust surface history (`adapter.rs` `history`): the same SplitMix64
  stream per history (`for_index(seed, h)`, ported to C++, seeded from `--seed` and the history
  index only), `tau = -ln(1 - u)`, isotropic rejection direction, collision and track-length
  tallies, leaked and absorbed counts, history `h` starts from ray `h % rays` in the ray file's
  region, blocks of 256 histories reduced in block order (so the result is the same for any
  thread count). The port is not bitwise: `std::log` and `std::sqrt` may differ from Rust's libm,
  so a path can diverge after a rounding difference; the agreement is the statistical one,
  [`runner::compare_tallies_with`] against the sm-rt facets *surface* flight on the same triangles
  (4 sigma from the block spread; collision, track-length, leaked and absorbed slots, so
  `checked = 2 * regions + 2`). `sigma.txt` (`id sigma_t sigma_a`, shortest round-trip decimals)
  is written by the runner into the case directory; a region without a line is a lost history, as
  in the Rust adapter. The facets adapter's flight treats a step that ends in a complement cell as a
  leak (it used to error).
* Determinism of a competitor row stays observed: the driver hash is equal across repetitions
  (`stable`), the runner also marks a workload unstable when the hashes of its thread counts
  differ. Nothing asserts equality with sm-rt (`result_hash` of two runs is equal, so the
  competitor output is reproducible, but the hash of a row is not an sm-rt hash).
* `xdg` rows stay `partial` (no image, W4).

`run_competitor` never invents a number: no Docker, `UNSET` digests, a missing image, a failing
container or an unparsable line is `Partial` with the command and its last output line, and the
report lists it.

## Docker

`docker/{dagmc-moab,dagmc-dd,xdg}/Dockerfile`. `docker/pins.env` is the single source of every pin:
base images are `FROM image@sha256:${..._DIGEST}` (index digests of `condaforge/miniforge3:24.9.2-0`
and `ubuntu:24.04`, read with `docker buildx imagetools inspect`; an index digest covers arm64 and
amd64), and `DOUBLE_DOWN_COMMIT`, `DAGMC_VERSION`, `XDG_COMMIT` (`83efb306e152`, xdg-org/xdg
"Format a fatal_error message given a single integer (#264)", 2026-10-26) and `MOAB_VERSION` (`5.6.0`, the MOAB of the conda DAGMC images, so every
row is fed by one MOAB) are `ARG` defaults in the Dockerfiles that the lint requires to equal the
`pins.env` value (it fails on drift, on a `*_COMMIT` that is not 12 hex digits, on `:latest` and on a
digest-less `FROM`). `docker-check` passes each pin the Dockerfile declares as a build argument.

* **Platform.** Builds and runs name `--platform linux/<host arch>` and refuse an image of another
  architecture (a leftover amd64 image under qemu is not benchmarked). x86_64 and aarch64 results are
  never merged.
* **Conda locks.** `docker/locks/<image>.<platform>.txt` is the explicit list (`@EXPLICIT`, url#md5) of a
  built image. `smrt-bench docker-lock dagmc-moab` writes it from `/bench/conda-explicit.txt`;
  `docker-check` builds with the lock of exactly the build platform (`--build-arg LOCKFILE=`) and, when
  there is none, says `no lock for linux/<arch>: will solve and write one` and solves against
  conda-forge. A lock of the other architecture is never read. The dd image builds double-down and DAGMC
  from source only when the environment holds no conda double-down DAGMC, so a lock of the source-build
  path (conda parts only) is reused with the same source step.
* **apt.** Recorded, not frozen (ruling C13): the xdg image writes `dpkg-query -W` into
  `/bench/versions.txt`; its hash is in the identity. No snapshot mirror. XDG image and driver are W4.
* **Identity.** Each image records `/bench/{versions,conda-explicit,base-index-digest,source-commits}.txt`.
  `ImageIdentity` (image id, repo digest, base index digest, platform, source commits, conda lock hash,
  versions hash, MOAB version) is read after the build and stored per competitor tracker in the result's
  `images` map (`image_digest` keeps the first image id). It is not in `result_hash`, and files without
  `images` still load. `docker-check` prints it per image together with the driver's `--version` line.
* **VM block.** On macOS the container is in Docker Desktop's Linux VM. `bench-env.json` gets a
  `docker` block (`NCPU`, `MemTotal`, kernel, server version, OS, the container's own CPU line) and
  `report` prints it under the banner. `run` passes `--cpus` = the largest requested thread count and
  refuses more threads than the VM has CPUs (no clamp). Cores cannot be pinned there: every competitor
  row has the note `cores not pinned`, and the report lists it with the 8P+2E asymmetry of the M1 Max.
  The smoke set on the Mac is `--threads 1,4`.

`dagmc_driver --version` prints the backend (`moab-obb` or `double-down`) and the linked DAGMC version; the
images also check with `ldd` that `dagmc-moab` does not and `dagmc-dd` does link double-down.

The XDG API is readable (`include/xdg/xdg.h`: `XDG::create(MOAB, EMBREE)`, `ray_fire`, `find_volume`,
`walk_elements`; it reads the DAGMC style MOAB file), so the `xdg` rows can reuse `facets_to_h5m`; the
driver is W4 work. The pin is `83efb306e152`, the commit `docs/research/dagmc-internals.md` was read
at (user ruling 2026-11-08). W3c briefly pinned `fec80980ce29` because a clone of the personal fork
`pshriwise/xdg`, which stops at 2026-09-08, lacked `83efb306e152`; clone only `xdg-org/xdg`. When W4
writes the XDG driver, move the pin to the newest upstream commit that builds and runs.

Native linux/aarch64 on the macOS development machine (Docker Desktop, rebuilt 2026-11-08 from the
locks in `docker/locks/`; the lock rebuild has the same conda lock hash, versions hash and driver `--version`
as the floating solve):
`dagmc-moab` is conda-forge `dagmc 3.2.4 nodoubledown` with `moab 5.6.0`; `dagmc-dd` is built from source
(the conda-forge `*double_down*` variant does not resolve on arm64) and its driver links `libdd`; `xdg`
has no driver (`NoDriver`). `run --cases P1 --scale reduced --rays 20000` gave `dagmc-moab` and `dagmc-dd`
equal region sequences on all 20000 rays, and both agreed with the sm-rt facet row on all of them.

## Fairness of the published table (O11)

The published table waits for the bare-metal Linux x86_64 machine. Every other OS or architecture
renders under the `SMOKE RESULT` banner and `report` suffixes tracker names with `(smoke)`; no result
file or `REPORT.md` of this wave is committed (`results/` holds no files), and no text states a
ratio between trackers.

## Hashed output and the clock

`date_utc`, `git_sha`, the write counter and the process id appear only in `env`, in the file name and
in the file itself, never in `result_hash`, in `Output::content_hash` or in the determinism check
(`two_smrt_runs_give_one_result_hash_and_date_and_sequence_stay_out_of_it`).

## Known limitations

* `--model`: `sm_tess` cannot tessellate imported faces on periodic surfaces (cylinder, sphere,
  torus, cone) because `sm-cadio` leaves their pcurves unset; B-spline faces tessellate but are
  slow in a debug build. Real fusion models need that fixed upstream (or a pcurve pass). A `--model`
  run has no rows: no sm-rt row builds a general model here.
* The stub tracker only knows spheres and boxes; the sm-rt analytic row covers spheres and
  cylinders (see above).
* The `H1` mesh is a structured tet mesh because `sm-scheme-tet` has no conforming inclusion
  meshing in this wave. P2 has no tet twin (pins in vacuum do not fill their box), so it has no
  delta or block rows.
* The DAGMC driver implements W-track, W-fire (hinted and located) and W-flight; the XDG driver
  and the `smtets_to_moab` converter (W4) are not written, and the `xdg` image is reported
  `partial` (`NoDriver`) with the failing build command. The C++ flight is not bit-identical to
  the Rust one (libm), only statistically (4 sigma) in agreement.
* Competitor containers run in Docker Desktop's VM on a Mac: the cores are not pinned, and the 8P+2E
  asymmetry of the M1 Max is not visible. Only `--threads 1,4` is a smoke set there.
* apt packages of the xdg image are recorded, not frozen.
* Stretch B5 (`--external-h5m`), B6 (import of the smart-mesh DAGMC exporter's h5m), B7 (OpenMC CSG
  row) and B8 (through-the-C-ABI row) are not done.
* `smcase.json` embeds the platform tag, so its hash is platform-keyed even for C1-C3 and H1.
  Dropping the tag from the hashed manifest would move every blessed `smcase.json` hash; that
  needs an orchestrator ruling and a re-bless.
