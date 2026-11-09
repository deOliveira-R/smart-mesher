# W3c wave report

Date: 2026-11-08. Pull requests: sub-wave 0 PR #1 (ed068df), sub-waves 1+2 PR #2 (d2e7731), XDG pin PR #3 (2a4d162), integration (W3c-INT, this report). Plan: `plan.md`; rulings C1 to C15: `rulings.md` (they override the plan and the drafts). Standing principle: Cubit is a bar to surpass, not a spec to match.

## What W3c delivered

- **Sub-wave 0 (PR #1, ed068df).** CI repair: `--no-fail-fast` in `tools/ci.sh`, per-platform `flat_golden` bundle constants (C11), the sm-py `test_errors.py` fix, venv cache. Export ACRs (note severity, `split_warped_faces` option, C10). `roxmltree` for sm-csgio. Start of the Linux hash work.
- **Sub-waves 1+2 (PR #2, d2e7731).**
  - Mesher: MESHER-REPLAN (plan-time blame, tet fallback with `mesh.auto_tet`, replan budget), CYL-SCHEMES and CYL-PIN (round pin in a brick: through pin all hex, enclosed pin all tet, C1).
  - Export: EXPORT-A/B/C (options, warnings on CLI stderr, MCP text and Python `ExportWarning`, C10; `split_warped_faces`).
  - CSG: CSG-CORE and CSG-OPENMC (`sm-csgio`: OpenMC `geometry.xml` and `materials.xml` into an analytic tracker, `smcsg` CLI, overlap and gap checks, OpenMC itself as the oracle, C6 to C9). MCNP is not a target.
  - Benchmark: BENCH-PIN (XDG pin, Docker identity), BENCH-WORKLOADS (W-fire and W-flight in the DAGMC drivers).
  - Cleanup: CLEAN-RT (ABI 10200, driver guard, acc04), CLEAN-SERVER (503 refusal thread, `OPENING` test), CLEAN-CADIO (DESIGN wording for `attributes: false`), LINUX-HASH (arm64 container recipe and comparison scripts).
- **XDG pin (PR #3, 2a4d162).** `sm-rt-bench` pinned at `83efb306e152` again (C15); BENCH-PIN had moved it to a commit from a lagging personal fork.
- **W3c-INT (this report).** Combine of eight tasks, then platform pins and docs. Details below.

## W3c-INT: what each task did

| Task | Commit | Result |
|---|---|---|
| PIN | ef419f2 | New `tools/w3c_pin.smesh`, `w3c_pin_enclosed.smesh`, `w3c_pin_leftout.smesh`, `w3c_warped.smesh`, `tools/w3c_driver.py`, `crates/sm-cli/tests/milestone_w3c_pin.rs`. Round-pin milestone (rows P1 to P11, below). No existing hash or golden moved. |
| FIX-PY | 68f3005 | Dropped the two stale `xfail` marks and `NO_SERVER_A`; new `test_gmsh_binary_matches_cli` (Python and CLI gmsh files, ASCII and binary, byte-identical). `crates/sm-py/ci/py.sh` passes; wheel state hash prefix 1fc1366d (W2 milestone). |
| FIX-MESHER | 8a7f8d3 | `plan_blame.rs` asserts the `ReplanLimit` Display text (`gave up after 1 attempts`) only; `plan_with_blame` reuses the kept trial error (one planner pass less per kept rejection). Hash census before and after byte-identical. |
| FIX-SERVER | 829d4c4 | auto_select: through-pin plan rows, `mesh.auto_tet` warning text, `rejections == 0`, three-way hash (in-process, CLI, MCP) for the through pin, the lattice, the enclosed pin and B5; transport stall bound of 1 s with 7 silent connections; rendezvous with a timeout instead of a barrier; Drop guards for scratch dirs. |
| FIX-CSGIO | 9bccad8 | 1-thread and 4-thread findings compared; 1e-11 relative tolerance, n = 1 and 2, `translated` fixture; oracle test with a row-swapped lattice and a transposed rotation (each must disagree with OpenMC); cli precedence; `world.rs` dead filter to `debug_assert!`; DESIGN.md step 5 (two leaves); `gen.py` hardened against leaks (not run). |
| FIX-RT | c2c4754 | acc04 clears the counters before `next_segment` and asserts `bvh_descents == 0`; DESIGN.md says `smrt_version` goes through `guard`. |
| FIX-BENCH | bc1dadc | `Competitor::Rows` carries the sha256 image id the container ran by, and `file.images` describes that image. |
| FIX-CLI-TMP | da17ad0 | sm-cli test helpers use `CARGO_TARGET_TMPDIR` (the unfixed code added 16 dirs to `$TMPDIR` per run, the fixed code adds none). |
| PLATFORM | 3dce752 | Hash table macOS arm64, Linux arm64 on glibc 2.36 and 2.39; `tools/platform/expected.txt` (62 lines, 7 scripts) and `crates/sm-cli/tests/platform_pins.rs` (C12); LINUX-HASH follow-ups. Table in `platform-hashes.md`. |
| DOCS | this commit | This report, W3b report numbering and CI section, PARITY rows, backlog dispositions, B6 and `plan.md` wording. |

Combine step 1 merged the eight worker commits with `--no-ff` merges onto 2a4d162 (8f62f64) with no conflicts. Its test run was cut short and gave no result (D16).

## Milestone results

Round-pin milestone `crates/sm-cli/tests/milestone_w3c_pin.rs` `milestone_w3c_pin` (new files only; the W3b milestone is untouched). Default threads and `RAYON_NUM_THREADS=1`.

| Row | Result | Evidence |
|---|---|---|
| P1 through pin and 3x3 lattice | pass | `model.subtract` without `keep`, imprint, merge; `model.copy` gives 18 volumes; Hex 432 (as CYL-PIN); model and mesh volume 14.2884 to 1e-9 |
| P2 explicit assignments, replay | pass | `mesh.auto` records 36 explicit assignments; replay and replay with the strategy renamed `no_such_strategy` reach the same hash, so no strategy runs |
| P3 validation | pass | `mesh.validate` has 0 violations |
| P4 `.smrt` regions | pass | regions `["fuel","moderator"]` through `sm_rt::read_smrt`; no warning on CLI, MCP, in-process or Python |
| P5 volume left out of the blocks | pass | `smrt-region-names` warning ("36 of 48 cells are in no block") on CLI stderr, MCP text, in-process result and Python `ExportWarning`; stdout unchanged; `.smrt` bytes equal across front ends |
| P6 OpenFOAM `split_warped_faces` | pass | planar pin: flag on and off give the same case; warped fixture: the flag changes the case and gives `foam.warped_faces_split`; CLI, in-process and Python digests equal; gmsh refuses the flag with `bad-request` |
| P7 record and replay | pass | `smesh record --forks refs --verify`; MCP `record_script` equals the CLI record |
| P8 enclosed pin (H = 0.8, C1) | pass | Tet 733, 0 violations, 2 `mesh.auto_tet` warnings (one per volume) |
| P9, P10 front ends and threads | pass | CLI, CLI project, MCP, in-process and Python (wheel) give equal hashes and file digests; two runs and 1 thread equal |
| P11 new hashes pinned as literals | pass | pin `h_model` f238f034..., `h_auto` 1f4b0912..., `h_mesh` 6953b909...; enclosed `h_model` 8cdcb65b..., `h_auto` 5dd202e5..., `h_mesh` 0804bbbf...; left-out `h_mesh` 87055d62...; warped `h_mesh` 45de87c0... (macOS arm64) |

Existing milestone hashes (C3): none moved. Evidence: the FIX-MESHER census (`collect.py`, 97 lines, byte-identical before and after) for W2 milestone and fork `1fc1366d...`, W3b `h_model` 66e4ac03..., `h_auto` fca39b79..., `h_mesh` 6f6c19f0...; the py.sh wheel check prints 1fc1366df0c3; `platform_pins` on three platforms equals `expected.txt`, which includes w3a A/B and `SUBMAP_GRADED`. Nothing was re-blessed.

Goldens: only the sm-mcp `session_export` entry and the two sm-protocol insertions (sub-wave 0) changed. The INT tasks moved none.

## Deviations

Numbered in one scheme. D1 to D5 come from the sub-wave review (backlog "Deviations to record"), D6 onwards from INT.

1. **D1: CYL-PIN acceptance 7.** `mesh.info` shows 0 rejections, because the replan runs in `mesh.auto`'s scratch plan. The `mesh.auto_tet` warnings are the evidence, and the tests pin `rejections == 0`.
2. **D2: B5 (plate with a hole)** is a hex sweep under `mesh.auto` (154 hexes), not tets. MESHER-REPLAN rewrote the b5 row; CYL-PIN owns it.
3. **D3: the census does not show B6.** `plate` is its only error-to-success change.
4. **D4: temp-dir leak.** CLEAN-RT moved the capi tests to `CARGO_TARGET_TMPDIR`; the root cause of the old empty `smrt-capi-*` dirs is unknown. INT found a second leak (sm-cli, `smesh-cli-*` and `smesh-full-*`, 16 dirs per run) and fixed it. The sm-csgio leak could not be reproduced. Other crates (sm-engine, sm-rt, sm-mcp, sm-server, sm-cadio, sm-rt-bench) still use `temp_dir()` in places (backlog).
5. **D5: BENCH-WORKLOADS acceptance 6** uses the C1 tiny ball (same optical radius 1.5).
6. **D6: B6 outcome (C2).** Auto next to a hard tet neighbour gives a conformal tet fill with a wedge layer at the shared face (`{"Tet": 160, "Wedge": 32}`), accepted as the `default` outcome. The W3b plan B6 wording is amended.
7. **D7: `session.export` in the script language** has no result and no options, so `w3c_pin.smesh` writes `pin.smrt` and a default OpenFOAM case; the warning and flag checks run through CLI export, MCP, in-process export and Python.
8. **D8: enclosed pin** is a single cell (733 tets) in its own script file, not a row inside `w3c_pin.smesh`.
9. **D9: P5 uses the left-out fixture** (one volume in no block, "cells are in no block"), not a model with no blocks at all. It runs the same `smrt-region-names` path; a no-blocks check is a backlog item.
10. **D10: P1 volume check** compares the total volume, which is the same for any conformal fill. A per-block fuel volume against the polygonal-cylinder area is a backlog item.
11. **D11: pinned literals were measured on macOS arm64.** Linux arm64 on two glibc versions gives the same values (platform table). Linux x86_64 also gives them: `platform_pins.rs` passed on the GitHub gate (runs 42544784727 and 42553841653).
12. **D12: the Python pin run** does not replay the pin as typed ops (W3b did for its milestone); it runs the script and the typed exports.
13. **D13: `tools/w3c_driver.py`** has not been run through mypy or ruff.
14. **D14: FIX-BENCH report claim.** The worker said `sha256:` assertions were added to two bench tests; the diff has none. `run_competitor_with` already returns Partial when the id has no `sha256:` prefix, so there is no correctness gap.
15. **D15: `BadOpName`** is used for a test-only rendezvous timeout in `crates/sm-server/src/standard.rs` (no better variant).
16. **D16: Combine step 1 test run.** It returned no result. The final combine covered it: on ae88c68 local `tools/ci.sh` rc=0 with no FAILED lines; `py.sh` rc=0, 59 passed, wheel check printing 1fc1366df0c3 under numpy 2.5.4 and 1.26.4; C3 census of release builds of 2a4d162 and ae88c68 byte-identical (16 rows, default threads and 1 thread); `w3c_pin.smesh` equal over 4 runs.

## Census

- Sub-wave census (MESHER-REPLAN, CYL-SCHEMES, CYL-PIN): the recorded changes are error to success. `plate` is the only error-to-success case in the census (D3); B5 moved to a hex sweep (D2); the round pin through case and the enclosed case are new.
- INT census (FIX-MESHER, `collect.py` before and after the `known` change): 97 sorted lines, byte-identical, for W2 milestone, W2 fork and W3b milestone.
- No existing milestone hash moved in INT (C3).

## GitHub CI status

Red on `main` since the W3b-1 merge (2026-11-05) for `flat_golden` (bundle hash on Linux) and sm-py `test_errors.py [<lambda>5]`. Both causes were fixed in sub-wave 0 (items 0.2, 0.3). The W3c-0 PR history, in order:
- 42444227750 on 90c503d: red on sm-scheme-tet `int7.rs:269` `restart_hint_index_work_is_linear_in_candidates`. `cos(TAU/3)` differs by one ulp between macOS and glibc, so the k=3 tower did not restart on Linux. Fixed in sub-wave 0 by e6c1527 (int7 tower ring coordinates pinned to f64 bit patterns).
- 42448206605 on e6c1527: green, and it printed the x86_64 `flat_golden` bundle rows.
- 9ac71ca added those x86_64 rows for the other three cases.
- 42453750390 on 9ac71ca: red on `milestone_w3b` row 8, because `w3b_python.sh` could not find `target/tmp/w3b-venv/bin/activate` (a partial venv restored by the CI cache).
- 83aea58 rebuilds a partial venv in `tools/w3b_python.sh`.
- 42457132759 on 83aea58: green.

GitHub CI has been green since the W3c-0 merge:
- `main`: 42463845479 (W3c-0 merge), 42482808601 (sub-waves 1+2 merge, PR #2), 42483031650 (backlog), 42496428949 (PR #3 merge).
- PR runs: 42457132759, 42475545325, 42489103474.

From W3c on a wave merges only with GitHub CI green as well as the local run (process note in `rulings.md`).

GitHub CI result of the final INT PR (PR #4, run 42544784727 on bb964d7): green. `gate` pass (1h34m), `python` pass (2m43s), `web` pass (12s). Second run 42553841653 on ae88c68: all three pass. `platform_pins` (`identity_hashes_match_expected_on_every_platform`) passed on Linux x86_64 in both `ci.sh` passes, so the identity hashes of `expected.txt` hold on x86_64 glibc as well (C12).

## Platform hash results

Details in `platform-hashes.md` ("W3c-INT rerun"). All identity hashes are equal across macOS arm64, Linux arm64 on glibc 2.36 (`rust:1.99-bookworm`) and Linux arm64 on glibc 2.39 (`ubuntu:24.04`), both with rustc 1.99.1.

| Script | Keys | macOS vs each Linux |
|---|---|---|
| w2_milestone | 21 | equal |
| w2_fork | 25 | equal |
| w3a_hybrid | 63 | equal |
| w3a_tet | 49 | equal |
| w3b_milestone | 56 | equal |
| w3b_submap_graded | 25 | equal |
| w3c_pin | 49 | equal (first hashed curved mesh: `h_mesh` 6953b909..., 432 cells) |
| curved_probe | 19 | equal (not pinned) |

C12: the identity hashes are pinned in `tools/platform/expected.txt` (62 lines, 7 scripts; per-node rows, floats and curved_probe are not pinned) and enforced by `platform_pins.rs`, which passes on macOS and in both containers at default threads and 1 thread. Nothing is pinned per platform. `flat_golden` on Linux arm64 gives the same four (structure, bundle) tuples on both images. Linux x86_64 (glibc, which picks libm variants at run time) also matches: `platform_pins.rs` passed in both `ci.sh` passes of GitHub runs 42544784727 and 42553841653.

## Benchmark smoke

On this Mac (Apple M1 Max, Docker Desktop 29.9.2, linux/arm64 VM, 10 CPUs, 15.6 GiB), under the `SMOKE RESULT` banner (ruling Q9); nothing under `results/` is committed; no ratios.

- `smrt-bench docker-check`: rc = 2, from xdg only (NoDriver: the XDG driver is W4; `pins.env` carries `XDG_COMMIT=83efb306e152`). Both DAGMC images built:
  - `smrt-bench-dagmc-moab` sha256:567fa5e9d40b..., MOAB 5.6.0, conda-forge dagmc 3.2.4.
  - `smrt-bench-dagmc-dd` sha256:58cb810de430..., double-down 437ad209eac5, DAGMC v3.2.4 from source.
- `run --cases P1 --scale reduced --trackers smrt,dagmc-moab,dagmc-dd --threads 1,4 --workloads track,fire,fire-locate,flight`: rc = 0; 36 rows, 0 disagreements; determinism "stable" on all rows; result_hash a6df0bf3e2e9....
- Correctness lines from the report: 10000 of 10000 fire and fire-locate rays agree with the sm-rt facets fire for both DAGMC backends; flight tallies 8 slots at 4 sigma, 0 failed; region sequences of moab and dd equal on all rays; track rows 0 failures.
- Noted: Docker rays are not pinned to cores; the dagmc-dd w-fire rows show a wide min-max spread on the Mac.

## W4 deferrals

- H1 analytic case and the P2 tet twin in the benchmark.
- `--model` for imported periodic faces (sm-cadio leaves pcurves unset).
- XDG tet walk and the XDG driver, at the newest upstream commit that builds (C15).
- O-grid all-hex pins (C1) and hex-tet transition cells.
- `hex_only` and `tet` strategies (C5; W3c-STRATEGIES was dropped).
- `smesh run` text mode printing per-op `delta.diagnostics`.
- CSG B-rep import (C8); hex lattices, periodic boundaries and DAGMC universes in the OpenMC importer stay refused.
- Backlog items with triggers: `docs/design/w3/backlog.md`.
