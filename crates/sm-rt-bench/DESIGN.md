# sm-rt-bench design

The tracking benchmark of smart-mesh against DAGMC (MOAB, double-down/Embree) and XDG. This
crate holds the **stub-only half** (RTBENCH-A): generator, formats, agreement checker, fairness
gates, result schema, report, self-test and the Dockerfiles. Runs of sm-rt (analytic, hierarchy,
delta) and of the Docker competitors are RTBENCH-B.

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

## Protocol for C++ drivers (RTBENCH-B)

Each driver reads the case files and prints one JSON object per (workload, threads) in the row
format above, with a running checksum of its outputs and no I/O or allocation in the timed loop.
The Rust runner reads those lines into the same `Row` type, so one `report` renders everything.
To add a competitor: a Dockerfile under `docker/<name>/`, a pinned entry in `docker/pins.env`,
a driver under `drivers/<name>/`, an entry in `docker::IMAGES`.

## Docker

`docker/{dagmc-moab,dagmc-dd,xdg}/Dockerfile`. Base images are `FROM image@sha256:${..._DIGEST}`
with the digests in `docker/pins.env`, source commits pinned (double-down `437ad209eac5`, XDG
`83efb306e152`) and verified by `git rev-parse` in the build. The digests are **`UNSET`** in the
repository: no digest is invented here. `smrt-bench docker-check` lints the files (`:latest` and
digest-less `FROM` fail), refuses to build while a digest is `UNSET`, and otherwise builds each
image and records the failing command if Docker or a build is unavailable. The drivers the
Dockerfiles copy arrive with RTBENCH-B. None of the images has been built from this task.

## Known limitations

* `--model`: `sm_tess` cannot tessellate imported faces on periodic surfaces (cylinder, sphere,
  torus, cone) because `sm-cadio` leaves their pcurves unset; B-spline faces tessellate but are
  slow in a debug build. Real fusion models need that fixed upstream (or a pcurve pass).
* The stub tracker only knows spheres and boxes, so tracker rows exist for `C1`, `C2`, `H1`;
  the analytic cylinder and hierarchy rows need sm-rt (RTBENCH-B).
* The `H1` mesh is a structured tet mesh because `sm-scheme-tet` has no conforming inclusion
  meshing in this wave.
* W-flight runs in surface mode only here; delta and `block` modes need the delta tracker.

## Deferred to RTBENCH-B

* `REPORT_TEMPLATE.md` (the narrative skeleton around the tables `report::render` prints) is not in
  this crate yet; RTBENCH-B writes it together with the real competitor drivers.
* The dagmc-dd image checks the double-down commit only on the source-build branch; the conda-forge
  branch is identified by the package list in `conda-packages.txt`. The driver's `--version` is
  currently a macro set by the Dockerfile. RTBENCH-B makes the driver report the version of the
  library it actually links.
* `smcase.json` embeds the platform tag, so its hash is platform-keyed even for C1-C3 and H1.
  Dropping the tag from the hashed manifest would move every blessed `smcase.json` hash; that
  needs an orchestrator ruling and a re-bless.
