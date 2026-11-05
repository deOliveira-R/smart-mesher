# sm-py W3b report (W3b-PY)

## What landed

* `core.rs`: `Host` (`Mutex<Server>`; `memory`, `memory_with`, `open`, `call`, `stage`, `mesh_ids`,
  `mesh_tables`), `HostError` (`NoMesh`, `AmbiguousMesh`, `UnknownMesh`, `Poisoned`),
  `MeshTables`/`tables_of` (the 22 always-present `.smsh` arrays).
* `errors.rs`: code to class name. `py.rs`: the `smartmesh._native` module under one
  `#[allow(unsafe_code)]`.
* Python package `smartmesh`: `Session` (apply, query, cmd, run_script, batch, timeline, export,
  stage, import_step, meshes, mesh), typed results, `EntityId`, `Mesh`, `smsh` reader, error
  classes, generated `ops.pyi`, `tools/gen_stubs.py`, `bench/tables.py`, `ci/py.sh`.

## Acceptance (draft acceptance 1-11 with the Q7 deltas)

| # | Result | Evidence |
|---|---|---|
| 1 | pass | `tests/test_slice.py`: cmd per line, typed ops, run_script, `smesh run --print-hash`, MCP `session_hash` all `1fc1366df0c3...`; undo/redo/checkout/branch restore hashes |
| 2 | pass | `tests/test_mesh.py::test_ten_cube_tables_are_analytic` (1331/3300/1000, `(1000, 8)`, bbox `[-5, 5]^3` exactly, 6 faces per cell, E = 3630, Euler 1, `left < right`) |
| 3 | pass | `test_session_tables_equal_the_smsh_file_and_are_read_only_snapshots` |
| 4 | pass | `test_polyhedra_volumes_by_divergence` (tet 2-cube to poly; total 8 within 1e-12 relative) |
| 5 | pass | `tests/test_errors.py`; panic hook `translate(_native._panic)` |
| 6 | pass | `tests/test_ids_typing.py` (round trip, drift, mypy strict + rejection of misspellings) |
| 7 | pass | `tests/test_determinism.py` and `tests/host.rs::determinism_*` |
| 8 | pass | `tests/test_step.py` (volumes 1200, 200, 600; inline, staged and `{sha256}` hashes equal) |
| 9 | pass | `tests/host.rs`: `Host::call` equals `InProcess` for every `METHODS` entry; tables equal the mesh and `.smsh`; unsafe token scan |
| 10 | pass | `ci/py.sh` wheel check (numpy 2.x on 3.14, numpy 1.26.4 on 3.12) |
| 11 | recorded | `DESIGN.md` last section |

## Notes for the orchestrator

* `ops.pyi` is a golden (generated from the catalog). W3b-INT regenerates it with
  `python crates/sm-py/tools/gen_stubs.py` after the merge, like the MCP and CLI goldens.
* `Cargo.lock` changed only in the `sm-py` entry (new dependencies).
* The library crate is named `_native` (it must match `module-name = "smartmesh._native"`).
* The command language here is `model.create_brick(x: 10.0, ...)`, not the Cubit-style
  `brick 10 10 10` the draft quoted; the slice is the one in `sm-cli/tests/data/milestone.smesh`.
* Per-platform wheels are a CI matrix, not built locally.
