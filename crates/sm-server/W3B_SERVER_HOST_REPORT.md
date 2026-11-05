# W3b-SERVER-HOST report

## What landed

- **Viewer transport.** `serve_http_ws_listener` / `serve_http_ws` (feature `ws`): one port serves the
  `sm_view::assets()` files on plain `GET`/`HEAD` (`/` is `index.html`, 404 and 405 are typed) and WebSocket
  clients. Any number of connections run on their own threads but only move text; the calling thread owns the
  endpoint and handles one request at a time, so the session keeps its single writer and all clients share one
  state. No server push (O16): clients poll `session.hash`. `smesh serve --ws ADDR` uses it (sm-cli feature `ws`,
  on by default; without it the old usage error stays).
- **`script.record`.** `Server` dispatches to `sm_lang::record_script` (and `verify_recording` for `verify`),
  `smesh record [--forks head|refs] [--check none|tips|every] [--verify] [--raw] [-o FILE]`, MCP tool
  `record_script`.
- **Plugin registration.** `sm_view::ViewPlugin` in `each_plugin` and in the builder chain; `sm-view` path
  dependency. `view.scene`, `view.mesh_skin`, `view.mesh_scalar` are in the `standard()` catalog.
- **AUTO acceptance B** in `tests/auto_select.rs`, run through `standard()` with the real schemes.
  `standard_schemes()` keeps its strict lists (plan: runs without `mesh.auto` keep their hashes);
  `DefaultStrategy` is already in every `MesherConfig`'s strategy registry, so there was nothing to register.

## AUTO acceptance B, outcome per item

| id | outcome | evidence |
|---|---|---|
| B1 brick | pass | `b1_...`: after `mesh.auto` + `mesh.run`, topology and geometry hash equal the explicit-map script; `mesh.plan` before `mesh.auto` shows `auto` map 100; no `mesh.auto_tet`. |
| B2 L-slab | pass | `b2_...`: sweep + one submap source face, 24 hex, V = 3 within 1e-12, validate valid. Negative control: without `mesh.auto` the strict lists fail with "8 faces; map needs exactly 6". |
| B3 prism | pass (first half) | `b3_...`: sweep + pave source, 6 hex, V = 0.5. The second half (`surface:["map","submap","trimesh"]` gives wedges) is not expressible: `standard()` registers only `DefaultStrategy::default()` and `mesh.auto` has no list parameters (the plan replaced the draft's `mesh.auto surface:[...]`). Covered in sm-mesher's stub tests (`with_lists`). |
| B4 tetrahedron | pass | `b4_...`: tetmesh, four tridelaunay faces (imposed), V = 1/6 within 1e-12, `mesh.auto_tet` quotes the map and sweep reasons and the fix. The draft's "hex-only policy fails with `NoApplicableScheme`" half needs list control (see B3). |
| B5 plate with hole | pinned: auto fails | `b5_...`: `mesh.auto` reports `no applicable scheme for Surface(Face#10)` (map and submap: inner loops; pave, trimesh, triadvance, tridelaunay: rejected by the volume's imposition); state unchanged. The explicit `mesh.scheme tetmesh` workaround meshes it: V within 2 percent of 16 - pi, valid, rejections <= 2. |
| B6 hard tet neighbour | fail, pinned | `b6_...`: `mesh.auto` fails on the shared face (imposed quad by the right volume's map, tri by the left tetmesh); the replan loop does not blame the auto volume. State unchanged. With the right volume explicit tetmesh it meshes, V = 16 (two 2-cubes), valid. See needs_ruling. |
| B7 census | pass, report below | `census_of_the_fixture_models_and_the_repository_scripts`: 18 models, 17 volumes, 7 changed. Every volume with an intent scheme is unchanged. |
| B8 determinism | pass | `b8_...`: hashes equal for two runs and 1 thread vs default (brick, L, prism, tetra); L-slab hash equal in process, `smesh run` and MCP `run_script`. |
| B9 tie | pass | `b9_...`: trimesh vs tridelaunay faces under a tet parent give equal topology and geometry hashes. |

### Census (changed cases)

Before = `mesh.run` with the recorded intent and the strict registry lists; after = the `default` strategy's
assignments as hard schemes. All changed rows are error to success, plus one error to a different error.

| model | volume | before | after |
|---|---|---|---|
| L-slab | TopoVolume#3 | error (8 faces; map needs 6) | sweep |
| prism | TopoVolume#3 | error (5 faces) | sweep |
| tetra | TopoVolume#3 | error (4 faces) | tetmesh |
| plate | TopoVolume#3 | error (7 faces) | error (no applicable scheme for the hole face) |
| w3a/head_b.smesh | TopoVolume#7 | error (5 faces) | sweep |
| w3a/head_b.smesh | TopoVolume#8 | error (8 faces) | sweep |
| w3a/head_b.smesh | TopoVolume#9 | error (7 faces) | tetmesh |

The three `head_b` rows are volumes the script has not yet given a scheme at the point where it stops when run
alone (it continues `head_a`); in the full W3a milestone they get explicit schemes and `mesh.auto` is not
called, so no existing script, hash or golden moves: `mesh.auto` is opt-in and `standard_schemes()` is
unchanged. Many repository scripts stop early when run in a bare session (inputs, includes), so the corpus is
the models they reach, 17 volumes in 18 models.

## Goldens

- **MCP tool list (not edited, W3b-INT blesses it):** adding the `record_script` fixed tool changes
  `crates/sm-mcp/tests/mcp.rs`: `golden_session_transcript` (the `tools/list` output),
  `help_lists_and_describes` and `tools_cover_catalog_batch_and_fixed` (tool counts), and
  `tests/standard.rs::golden_standard_tool_list` (`golden/tools.standard.txt`: the `record_script` tool and the
  `view.*` queries). Additive only. These four are the only failures in `cargo test --workspace`.
- sm-server has no op-count snapshot: nothing to update there. `w3_wiring::script_record_is_typed_unsupported`
  became `script_record_on_an_empty_timeline_records_nothing`.
- No state hash, layer hash, export golden or determinism golden moved.
