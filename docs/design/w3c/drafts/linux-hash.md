# W3c-LINUX-HASH: state hashes on Linux arm64 (and a path to x86_64)

> Planning draft (W3c), corrected by the planning critique. Changes against the original draft are marked **Critique delta**. `docs/design/w3c/plan.md` wins over this file.
>
> **Feeds plan tasks:** W3c-LINUX-HASH (S1, S2, S3 recipe and comparison run, orchestrator, read-only, sub-wave 1); sub-wave 0 (`--no-fail-fast`, `flat_golden` per the user's ruling); W3c-INT (S4 `platform_pins.rs` and `tools/platform/expected.txt`).

Original size: S. Original owners: orchestrator (`tools/platform/` scripts; CI), sm-cli (`crates/sm-cli/tests/platform_pins.rs`, test-only), sm-rt (separate micro-task only if Q3 is approved: `crates/sm-rt/tests/flat_golden.rs`). Depends: none for the comparison run (read-only against main d7ad022); Q9/O11 benchmark machine is independent; this task only de-risks it; sm-rt-bench pins.env digests are filled by the same recipe style but are not a dependency.

## Goal
Find out whether the hashed milestone scripts give the same state hashes on Linux as on macOS arm64, before the user's Linux x86_64 benchmark machine arrives. Leave behind (a) a repeatable container recipe and comparison script, (b) a committed baseline that CI enforces, and (c) a pre-decided path if hashes differ. Everything run on this Mac is labelled smoke. Nothing here changes pave/tri/sweep/model output.

## Owner
- Recipe and comparison script: orchestrator (`tools/platform/`, new files only). The task writes no crate source.
- Baseline pin test: `sm-cli`, test-only (`crates/sm-cli/tests/platform_pins.rs`). It follows the existing pattern of `SUBMAP_GRADED_*` in `milestone_w3b.rs`. **Critique delta:** written by W3c-INT (too many tasks add to sm-cli; integration owns the new sm-cli test files, as in W3b-INT).
- Anything under `sm-rt` is a separate micro-task (see Finding F6 and Q3). **Critique delta:** the `flat_golden` edit is an orchestrator item in sub-wave 0, applied once the user rules on plan C11.

## Findings

**F1. The design already says hashes are cross-platform by quantization, and the evidence supports it.**
- `docs/ARCHITECTURE.md:52-53`: platforms cannot be bit-identical (libm and FMA differ), so the CI state hash covers topology plus quantized coordinates and "must match on every platform".
- `crates/sm-layer-api/src/mesh.rs:80-84` and `:225-234`: coordinates go through `Digest::f64_quantized(x, quanta.length)`, parameters through `quanta.param`, and real tag values through a 40-bit mantissa (`scale_free_code`).
- `crates/sm-geom-api/src/tol.rs:48,113-115`: with the default tolerance, the length quantum is 1e-6 * 2^-10 (about 1e-9) and the parameter quantum is 1e-9 * 2^-10 (about 1e-12).
- `topology_hash` is an exact integer hash (`sm-mesh-api/src/validate.rs:1360`).
- So a libm difference of 1 ulp (about 1e-16 relative) changes a hash only in two ways. One is a quantization-boundary flip, about 1e-7 per perturbed coordinate. The other is a discrete decision (rounding, sort, threshold) taken at a near-tie.
- Real tags: the grep found no op in the meshing path that writes real-valued tags (outside `sm-mesh-api`, `sm-meshio` and `sm-layer-api`). If one did, a flip would be about 1e-4 per value; the run in A2 confirms the milestone scripts do not reach this.

**F2. CI is already a Linux x86_64 run, and it already has partial evidence.**
- `.github/workflows/ci.yml` runs `tools/ci.sh` on `ubuntu-latest`. That is the same OS and arch as the planned benchmark machine, but on a different CPU and with glibc 2.39.
- In the latest gate run (`gh run 42412194333`, main d7ad022), `tests/milestone.rs`, `milestone_w3a`, `milestone_w3b` (including `milestone_w3b ... ok`) and `submap_graded_input_is_pinned ... ok` all passed.
- `submap_graded_input_is_pinned` compares three literal hashes blessed on macOS (`milestone_w3b.rs:603-605`: topology `c99bcaa8d93e9a5e`, geometry `43ecc0a4b59b3715`, state `7b71fd6d...0b1f`). So at least one real pave/sweep/submap path already matches macOS arm64 against Linux x86_64.
- The milestone tests compare front ends on one machine. They do not pin `h_model`, `h_auto`, `h_mesh` or the W2 and W3a hashes as literals. The cross-platform claim for those is therefore unverified, and this task closes that gap.

**F3. The CI gate has been red since 2026-11-05, and that hides everything after `sm-rt` in the workspace test order.**
- `cargo test --workspace` in `tools/ci.sh` stops at the first failing test binary. The failure is `crates/sm-rt/tests/flat_golden.rs:55`: `hex_block6: (structure hash, bundle hash) moved`.
- The structure hash is equal on Linux (left `2089110304...0514`). Only the bundle hash differs (Linux `44718444...`, macOS-recorded `272840827...`).
- Crates that sort after `sm-rt` are not exercised on Linux since then: `sm-rt-api`, `sm-rt-bench`, `sm-rt-capi`, all `sm-scheme-*`, `sm-server`, `sm-sizing`, `sm-smooth`, `sm-tess`, `sm-testkit`, `sm-topo`, `sm-view`.
- The CI python job also fails (`test_errors.py ... DID NOT RAISE InvalidParamsError`). That is unrelated to libm.
- **Critique delta:** the critique confirms GitHub CI is red on main (the last 4 runs, including 42412194333): the gate at `flat_golden.rs:55` and the python job at `test_errors.py::test_python_side_validation_raises_before_any_call[<lambda>5] DID NOT RAISE InvalidParamsError`. The python failure gets its own sm-py micro task in sub-wave 0 (root cause first; a likely cause is a new op keyword after the `ops.pyi` regeneration).

**F4. Cause of the `flat_golden` mismatch: the test's ray generator, not the tracker.**
- `crates/sm-rt/tests/common/mod.rs:53-57` (`random_frame`) builds directions with `phi.cos()` and `phi.sin()`. The recorded constants (`flat_golden.rs:49-56`) were therefore produced with Apple libm.
- The structure hash (the tracker build) matches, and the tracker library on the flat path has no transcendental call. The only libm uses in `sm-rt/src` are `vec3.rs:46` (`sin_cos`, used by `world.rs:1303` and `analytic/check.rs:127`), `analytic/kernel.rs:74` (cone), `hypot` in the torus/cone kernels, and `cbrt` in `check.rs:103`.
- This is a strong hypothesis rather than a proof. The container run (A4) settles it, because the tracker's bundle hash must then match once the rays are fixed.
- The bundle hash is a raw-bit hash and is not quantized. It is therefore platform-keyed by nature, like the facets in `sm-rt-bench`.
- **Critique delta:** F4's "structure hash is equal on Linux" counts as settled only when the worker pastes both `(structure hash, bundle hash)` tuples, macOS and Linux (from CI logs or the container run), into `docs/design/w3c/platform-hashes.md`.

**F5. Where the platform libm sits in the hashed paths** (counts from a grep of `.sin( .cos( .sin_cos( .tan( .atan2( .acos( .asin( .exp( .ln( .powf( .cbrt( .hypot(` and friends, `crates/*/src`, including in-file test modules).
- No portable math library is used anywhere: no `libm`, `libm-sys` or `statrs` dependency, and no hand-written `sincos`. Rust's `f64` methods call the platform libm (Apple libm on macOS, glibc on Linux gnu).
- The only mitigation in the tree is `crates/sm-rt/src/vec3.rs:42-47`. It splits `sin` and `cos` with `black_box` so release builds do not fuse them into `sincos`. That makes results profile-independent, not platform-independent.
- Pure IEEE operations (identical everywhere): `+ - * /`, `sqrt`, `abs`, `floor`, `round`, `min`, `max`, `powi` (compiler-builtins repeated multiplication), `to_degrees`, `to_radians` (multiplications).
- `mul_add` (4 uses, e.g. `sm-sizing/src/iia.rs:139`) is correctly rounded everywhere (hardware FMA on arm64, software `fma` on baseline x86_64).
- Rust never contracts `a*b+c` into an FMA, and no `target-cpu` flag is set (`.cargo`, `Cargo.toml` and CI have none), so FMA contraction cannot differ between builds.

Libm-dependent code on hashed paths:

| Crate | Calls (file:line) | Reached by milestone scripts? |
|---|---|---|
| sm-scheme-pave | `kernel/mod.rs:321-323` (`ln`/`exp` geometric-mean size scale); `kernel/corner.rs:218-221`, `strip.rs` (`atan2` turn angles, sorted with `total_cmp` and index tie-break); about 29 sites in total | Yes (every pave face). Inputs are axis-aligned, so `atan2` hits its exact special cases. `ln`/`exp` feed a continuous scale; the effect on the mesh is through rounding and thresholds. |
| sm-scheme-curve | `spacing.rs` (`exp`, `ln`, `exp_m1`, `ln_1p`) for geometric and bias spacing | Only with non-uniform spacing. The milestone scripts use uniform sizes. |
| sm-sizing | `auto.rs:113` (`2f64.powf((factor-1)*0.5)`), `field.rs:99` (`powf`) | `mesh.size` with a field or `auto`-sizing only. `factor == 1` gives `powf(.., 0.0) == 1` exactly. |
| sm-scheme-sweep | `place.rs` (`powf`), `emit.rs` (`sin_cos` bend) | Only with non-default placement or bend. |
| sm-scheme-tri | `cdt/refine.rs` (`atan2`, `sin` of min angle), `scheme.rs` | Only for trimesh or tridelaunay faces. |
| sm-scheme-tet | `steiner.rs`, `recover.rs`, `geom.rs`, `quality.rs` (`atan2`, `sin_cos`, `powf`) | In the W3a tet path (planar bricks). |
| sm-geom, sm-geom-api | `nurbs/exact.rs` (arc to NURBS `sin_cos`), `intersect/*.rs`, `analytic.rs`, `elliptic.rs` (`sin_cos`, `atan2`, `hypot`, `acos`) | Only for curved geometry. The W2, W3a and W3b scripts are all planar (the W3a STEP has 23 `PLANE`s and nothing else). |
| sm-tess | `sample.rs` (`acos`, `cos`, `sin` for chord and angle sampling), `f32` positions | Only for curved faces and edges. |
| sm-model | `prim2.rs`, `queries.rs` (`atan2`, `sin_cos` for cylinder, cone, torus, sphere) | Not for bricks. |
| sm-lang | `eval.rs` (`powf`, `atan2` in scripts) | Only if a script uses them. |
| sm-quality | `solid.rs`, `faces.rs`, `fv.rs`, `planar.rs` (`powf`, `atan2`, `asin`, `acos`) | Reported by `mesh.info`/`quality`, not hashed. Confirm in A2. |
| sm-rt | see F4 | Raw-bit hashes only (bundle, tally). Never quantized. |

- Quantization turns most of this into a non-issue. The two real risks are (a) discrete decisions on curved geometry (tessellation counts from `acos`, tie-breaks) and (b) raw-bit hashes outside the state hash.
- `sm-rt-bench`: `DESIGN.md:47-56` and `src/cases.rs:567-580` already key facets from `sm_tess`, their rays, and every `smcase.json` per `@os-arch`, and state that tessellations are not claimed identical across platforms. `DESIGN.md:227-229`: the platform tag inside `smcase.json` makes even the C1-C3 and H1 `smcase.json` hashes platform-keyed.
- LLVM constant-folds `sin(const)` and similar with the compile host's libm. That matters only for cross-compiled builds; the container builds natively, so it is moot here.

**F6. Likely outcome (stated with evidence).**
1. **Milestone and W2/W3a/W3b state hashes will match on Linux arm64.** The scripts are planar, the libm uses they reach are exact-special-case or continuous-scale uses, hashing is quantized, and Linux x86_64 CI already reproduces `SUBMAP_GRADED`, which includes pave, sweep and submap.
2. **Raw-bit hashes will differ.** These are the `sm-rt` bundle and tally hashes in `flat_golden` (confirmed on x86_64 CI), the `sm-rt-bench` facets and `smcase.json` (already keyed per platform), and anything that hashes exact bits (`geometry_hash`, `.smsh` bytes).
3. **Curved-geometry quantized hashes are the unknown.** They are the likeliest place for a state hash to diverge: tessellation through `sm_tess` with `acos` count decisions, NURBS arcs, and the `f32` facets. No script in the milestones exercises them, and the container run does not either. See Q2 and A6. **Critique delta:** the W3c round-pin milestone (`tools/w3c_pin.smesh`, W3c-INT) is the first hashed curved script; INT adds it to the comparison.
4. **x86_64 may differ again, and arm64 Linux does not prove it.**
   - glibc on x86_64 selects `sin`/`cos`/`exp`/`log`/`pow`/`atan` variants at run time through ifunc (FMA or non-FMA). aarch64 glibc compiles the same C with contraction.
   - The GitHub runner (FMA) is a fair proxy for the user's machine. A `--platform linux/amd64` container on this Mac runs under Rosetta or QEMU; whether it reports FMA to glibc is uncertain, so treat it as a variant-sensitivity preview, not as the benchmark.
   - Different glibc versions (bookworm 2.36 versus CI's 2.39) are a second axis, which is why the recipe pins digests.

## Scope

**S1. Container recipe** (`tools/platform/Dockerfile`, `tools/platform/run_container.sh`)
- Base image `rust:1.99-bookworm@sha256:<digest>`. `rust-toolchain.toml` pins channel 1.99, so the image must carry the same minor. The digest is read on this Mac with `docker buildx imagetools inspect` and written into `tools/platform/pins.env` as `RUST_DIGEST=`. It is never invented; the lint of `sm-rt-bench` (`lint_dockerfile`, `docker.rs:26-36`) is the model.
- Second image `ubuntu:24.04@sha256:fe81560990fae86dc4743202b2cb4d3df98d0bcf949173cecef15ce1127ef852` (already in `crates/sm-rt-bench/docker/pins.env`) with rustup 1.99. It matches the CI glibc 2.39.
- Needed in the image: `build-essential`, `python3-venv` (for the Python row), `curl`, `git`.
- Run with `--platform linux/arm64` (native on the M1 Max, no emulation). Optional `--platform linux/amd64` as the labelled preview of F6.4.
- The repo is mounted read-only at `/src`. Named volumes hold `CARGO_HOME` and `CARGO_TARGET_DIR=/cache/target`. A host `target/` is never mounted (it would clobber the macOS artifacts). `cargo build --locked`.
- Output goes to a bind-mounted `/out`.

**S2. Comparison script** (`tools/platform/hashes.sh`, `tools/platform/compare.py`)
- `hashes.sh <out-file>` runs on either host. It builds `smesh` (debug, as CI; release optional) and prints sorted `script|key|value` lines with the same pinning the tests use (`--author it --fixed-time 2026-10-24T00:00:00Z`).
  - W2: `crates/sm-cli/tests/data/milestone.smesh` through `smesh run --memory --print-hash` and its CLI project run. Record the per-node hashes from `smesh log --json`, the `smesh hash --json` per-layer hashes (`model`, `intent`, `meshes`, `fem`), and the `session.hash` printed lines.
  - W2 fork: `fork.smesh` with `-p`, then `smesh hash --at main|alt|coarse --json` (the commands at `milestone.rs:698-720`).
  - W3a: emit the scripts with `MILESTONE_W3A_EMIT=<dir> cargo test -p sm-cli --test milestone_w3a emit_conformance_scripts`. Then run `w3a_hybrid.smesh` and `w3a_tet.smesh` the same way, including `--at tet`.
  - W3b: `tools/w3b_milestone.smesh` in a scratch cwd. Record `h_model`, `h_auto`, `h_mesh`, the `SUBMAP_GRADED` topology/geometry/state hashes, and `sha256` of the exported `.step` (sanity only).
  - Then run `cargo test -p sm-cli --test milestone_w3b -- --nocapture` and capture its rows, including M6 (delta versus surface z; the report says it is deterministic only per libm, so only pass/fail is compared, not the value).
- `compare.py a.txt b.txt` prints the first differing key per script and exits 1. It also prints which layer differed (from the layer hashes) and the first differing node in the timeline. That is the bisect tool for Outcome B.
- The container also runs `cargo test -p sm-rt --test flat_golden` and `smrt-bench gen --scale tiny --check` with `--print`. These give the Linux aarch64 `@linux-aarch64` lines for the `sm-rt-bench` `expected_sha256.txt` format and the `flat_golden` values (F4).

**S3. Where the result is committed**
- `tools/platform/expected.txt`: one line per `script|key` with the shared hash (identity hashes, expected equal on all platforms), plus an optional `@os-arch` overlay line only where a divergence is accepted. It uses the same `@os-arch` suffix convention as `crates/sm-rt-bench/expected_sha256.txt`. **Critique delta:** committed at INT, subject to plan C12.
- `docs/design/w3c/platform-hashes.md` (an orchestrator note) with the date, image digests, host/guest platform tag, and the comparison table. Labelled smoke.
- `sm-rt-bench`: the `@linux-aarch64` lines are printed with `smrt-bench gen --scale <s> --print` and only recorded in the note. They are not blessed in `expected_sha256.txt` (blessing needs the orchestrator ruling already required in `DESIGN.md:51-53`, and published numbers wait for the x86_64 machine, O11).

**S4. Pin test** (`crates/sm-cli/tests/platform_pins.rs`)
- Reads `tools/platform/expected.txt` and runs the same scripts in process. This makes the already-existing Linux x86_64 CI the continuous cross-platform guard (F2).
- The check is guarded on `expected.txt` lines only. It does not add literals to `milestone_w3b.rs`, so no existing test is edited.
- **Critique delta:** S4 moves to W3c-INT.

## Contract changes / ACRs
None. No `crates/*-api`, `sm-testkit` or `sm-protocol` change.

## Acceptance
1. `tools/platform/run_container.sh --platform linux/arm64` builds with every `FROM` pinned by `@sha256:` (a lint test or script check refuses `:latest` and unpinned `FROM`), and prints `platform linux-aarch64 glibc <version>` and the image digest into its output. Digests are read, never fabricated.
2. `hashes.sh` run on macOS and in the container gives, per script, the same set of keys (a missing key on either side is a failure of the script, not a silent skip). Covered scripts: W2 `milestone.smesh` (printed hash, per-layer hashes, all log node hashes), W2 `fork.smesh` (`main`, `alt`, `coarse` and after), W3a hybrid (including `tet`) and tet, W3b `h_model`, `h_auto`, `h_mesh`, plus the `SUBMAP_GRADED` triple.
3. `compare.py` on two runs of the same host (same platform, run twice, and `RAYON_NUM_THREADS=1` against default) reports zero differences (control: the script itself is deterministic). With a hand-edited key the script exits 1 and names the key, the script and the first differing timeline node (negative control).
4. `cargo test -p sm-rt --test flat_golden` in the container returns the Linux aarch64 `(structure hash, bundle hash)` for all four cases. The structure hashes equal the macOS constants (`flat_golden.rs:49-56`), which confirms F4 (only the ray generator is libm-dependent). If a structure hash differs, that is a finding: the tracker build itself depends on the platform. **Critique delta:** both tuples (macOS and Linux) are pasted into `platform-hashes.md` before plan C11 counts as settled.
5. Result table in `docs/design/w3c/platform-hashes.md`: for each identity hash, `equal` or `differs` with the first differing node and layer, and for each raw-bit hash the Linux value next to the macOS value. Expected-equal rows that differ are flagged for Q2.
6. A curved-geometry probe (not part of the milestone, recorded as information only): a script with a cylinder, a sphere and a torus (`model.create_*`), meshed with a default size and an `auto`/`tetmesh` volume, plus `model.export` and a re-import of a curved STEP fixture. The state hash, the quantized `geometry_hash` and the `sm_tess` facet counts are compared between macOS and Linux. The result decides whether Outcome B below needs the portable-libm discussion for the curved paths. It changes no hash because it is a new script.
7. If all milestone hashes are equal: `expected.txt` is written from them and `platform_pins.rs` passes on macOS and in the container. `RAYON_NUM_THREADS=1` against default gives the same lines (determinism). Evidence that the x86_64 CI run also passes is the first green gate after the CI fix in Q4. **Critique delta:** at INT.
8. Existing goldens: with the new files only, `cargo test -p sm-cli`, the W2/W3a/W3b milestones and every existing hash are unchanged (no source edit in any crate).

## Gates / dependencies
- Docker 29.9 on this Mac (present), network for the image pull and crates.
- The `flat_golden` question (Q3) and the CI `--no-fail-fast` change (Q4) are orchestrator decisions. They do not block the comparison, but until Q3 is settled the gate stays red and `platform_pins.rs` cannot be seen passing on x86_64 CI after `sm-rt`.
  - **Critique delta:** `--no-fail-fast` is an orchestrator action in sub-wave 0. The `flat_golden` change needs an explicit user ruling (plan C11), because the test comment says a moved value is never re-blessed; it is applied in sub-wave 0 once ruled.
- Definition of done for the new test follows `CLAUDE.md`: `cargo test -p sm-cli`, clippy `-D warnings`, determinism (two runs and 1 against N threads), rustdoc on public items.

## Decision path if hashes differ (pre-decided so the run does not stall)
1. **Localize.** `compare.py` gives script, layer, first differing op, and the first differing coordinate (exact bits dump). Classify it:
   - (a) a quantization-boundary flip: one coordinate differs by 1 quantum. Probability about 1e-7 per perturbed coordinate, so one flip points at a libm-fed value, not at a bug.
   - (b) a discrete decision on a near-tie: a count, sort or threshold depends on a libm result. Replace the comparison with an exact or tolerance-based one (a code fix owned by that crate, with a test that checks the tie case on analytic values).
   - (c) a raw-bit hash that was never meant to be portable: record per `@os-arch` (existing convention).
2. **Ruling proposed (Q2):** keep the Architecture rule. State (identity) hashes must be equal on every platform. Raw-bit hashes are per-platform baselines.
   - For a quantized identity hash that diverges in a real case, fix the cause locally first (b).
   - Only if a libm-dependent value cannot be made decision-safe, adopt a portable math layer on hashed non-hot paths: a small `sm-core::math` shim over the pure-Rust `libm` crate (deterministic given IEEE `+ - * /` and a correctly rounded `fma`), used by `sm-geom`, `sm-model`, `sm-tess`, the schemes and `sm-sizing`. `sm-rt` hot loops stay on platform libm, because their hashes are raw-bit and per-platform anyway.
3. **Cost of the portable layer, if chosen:** results differ from Apple libm and from glibc in last-bit cases. That moves quantized hashes only when a value sits within 1e-16 of a boundary (not expected for the planar milestones), but it moves `sm_tess` `f32` facets and therefore every `@macos-aarch64` line in `sm-rt-bench`. Those would be re-blessed once in a dedicated, reported change. `libm` is also slower than glibc by roughly 1.5-3x for `sin`/`cos`/`pow` (typical, not measured here), so measure `sm-tess` and pave before adopting.
4. The alternative, per-platform baselines for state hashes, makes every state-hash literal an `os-arch` table and breaks the "same script, same hash" promise that the replay and fork features rely on across machines (see the strict replay check in `sm-engine`). It is therefore the fallback, not the default.

## Hash impact
None. The task adds scripts, a note, an `expected.txt` and a test file. No pave, tri, sweep, model, mesher or tess output changes, and no existing hash moves (W2 milestone/fork, W3a A/B, W3b `h_model`, `h_auto`, `h_mesh`, `SUBMAP_GRADED` all stay).
- The only hash-moving outcome is the conditional portable-libm decision above. It needs a separate ruling and a separate task, and it is called out here so it is never taken silently.
- Q3 (`flat_golden`) changes no production output. It changes a test constant keyed by platform.

## Size
S (one afternoon of container work, the scripts, one test file, one note). It becomes M only if Q2's portable-libm branch is taken.

## Risks
- Container build time: a cold `cargo build` in a fresh arm64 volume is long (CI's gate takes about 20-30 minutes including tests). Named volumes make reruns fast; the first run should not be timed as a result.
- Docker Desktop on Apple Silicon is a VM with its own kernel and a fixed glibc from the image. It tests the libm of the image, not of the host Linux kernel (irrelevant for libm, but the CPU feature flags glibc sees come from the VM).
- `rust:1.99-*` tag availability and the rustup component set must be confirmed when the digest is read. `rust-toolchain.toml` can otherwise trigger a toolchain download inside the container and break pinning (use `RUSTUP_TOOLCHAIN` from the image).
- F4 is a hypothesis until A4. A structure hash that differs on Linux would mean a worse problem than test-side rays.
- Curved geometry (A6) is the weak spot: the evidence for equal hashes is strong for planar scripts only. A `Differs` result there is an expected finding, not a failure of this task.
- The python row needs `maturin` over the network; skipped with a printed reason as in `milestone_w3b.rs`.
- The red CI gate (F3) means the "x86_64 CI passed" argument covers only crates before `sm-rt`. This is stated in the note so nobody over-reads it.

## Questions
- Q1 Should the milestone identity hashes become literal pins that the existing Linux x86_64 CI enforces every push (via `tools/platform/expected.txt` and `crates/sm-cli/tests/platform_pins.rs`)? (rec: Yes. CI already runs ubuntu-latest x86_64, so the pins give a permanent macOS-vs-Linux guard for free. The cost is that any intentional hash move needs a re-bless of one text file, the same as SUBMAP_GRADED today. Keep them in the data file, not as literals in milestone_w3b.rs, so the existing tests stay untouched.) → plan C12 (first part).
- Q2 Ruling if a quantized identity hash differs between platforms on a real case: local decision-safe fix first, then a portable libm layer only if needed, with raw-bit hashes staying per-platform? (rec: Adopt that order; do not adopt the portable layer pre-emptively.) → plan C12 (second part).
- Q3 May we (a) key the four `flat_golden` bundle-hash constants per os-arch or (b) replace the generator with a libm-free one and re-bless once? (rec: (a).) **Critique delta:** this changes a test whose comment says "a moved value is a stop, never a re-bless"; per-os-arch constants need an explicit user ruling → plan C11. Applied by the orchestrator in sub-wave 0 once ruled.
- Q4 Should the orchestrator fix `flat_golden` first and switch `tools/ci.sh` to `cargo test --no-fail-fast`? **Critique delta:** orchestrator action in sub-wave 0, not a user question. Dropped from the user list.
