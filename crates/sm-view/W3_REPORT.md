# sm-view: W3b-VIEW report

Branch `w3b-1/view-0`. Design: `DESIGN.md`. Spec: `docs/design/w3b/plan.md` (W3b-VIEW) and `drafts/view.md`, with the plan's ruling deltas (O16 polling, label to palette color, WebGL2).

## Acceptance

| id | result | evidence |
|---|---|---|
| 1 box skin | pass | `tests/view.rs::acc1_box_skin`: 80 triangles, 40 distinct facets, every `tri_owner` nonzero, area 40 within 1e-12 |
| 2 clip | pass | `acc2_clip`: 8 clipped / 8 kept, 48 triangles, area 24, 8 cap triangles at x = 2 all showing kept cells; plane removing everything gives an empty skin; plane through centroids keeps them |
| 3 two volumes | pass | `acc3_two_volumes`: 80 + 16 triangles with the cap double sided (owner `TopoFace`), `volumes=[one]` gives 48 with 8 cap triangles owned by the volume, cap area 4 |
| 4 polyhedral | pass | `acc4_polyhedral_skin` on `dual_patch`: every skin edge used twice with opposite directions, area equals the fixture area and the sum of boundary facet areas within 1e-12, 14 centroid vertices appended after the node vertices |
| 5 surface mesh | pass | `acc5_surface_mesh`: 3x3 quads give 18 triangles, area 9, 24 outline edges; clip removes polygons by centroid |
| 6 scalar | pass | `acc6_scalar`: values equal `sm_quality::compute` bitwise after the f32 cast, `worst[0]` touches the moved node, `bad_cells` equals the independent count, min/max/mean recomputed, box is all 1.0; `acc6b` covers the typed unsupported metric |
| 7 typed errors | pass | `acc7_typed_errors`: unknown metric lists names, `TooLarge{tris:80,max:10}` with the hint, `AmbiguousMesh`, `NoMesh`, unknown mesh, bad clip |
| 8 determinism, property | pass | `acc8_determinism_bytes` (`sm_testkit::determinism::check`, 1 vs 4 threads, scene/skin/clipped skin/scalars and the polyhedral skin as JSON strings including base64); `clip_partitions_cells_and_skin_faces` with `config(48)` |
| 9 node unit tests | pass | `npm test`: 26 tests (base64 round trip against node's encoder and the Rust fixture, truncated buffer throws `CodecError`, fit-to-bbox corners inside the frustum for four boxes and four aspects, CPU pick from +x/+y/+z (and the negatives) returns `TopoFace#2/4/6`, raster agrees with the pick) |
| 10 assets, dist hash | pass | `tests/assets.rs`: `assets()` lists index.html, viewer.js, viewer.js.map with mime types and bytes equal to `web/dist`; `dist.sha256` matches; the esbuild rebuild test reproduces dist byte for byte (skips with a message if node or node_modules is absent) |
| 11 docs, lints | pass | `DESIGN.md`, this report; rustdoc `-D warnings`; clippy `--workspace --all-targets -D warnings`; `tsc --noEmit --strict` clean |
| browser e2e | pass (ran) | `SM_VIEWER_E2E=1 npm run e2e` on macOS Chrome with SwiftShader flags: ID buffer differs from `raster.ts` in 0.14 to 0.17% of pixels (limit 0.5%), click returns the predicted owner, clip slider gives 48 triangles, findings row centers the camera exactly. It skips with a reason when no browser is found, so CI without Chromium is unaffected |

## Deviations and decisions

- `tri_owner` indexes a per-reply `owners` table instead of carrying a raw id (an `EntityId` does not fit a `u32` and the client needs the kind). Recorded in `DESIGN.md`.
- `Skin.positions` in the library form is f64; the reply casts to f32 once. This lets the area tests hold 1e-12.
- Replies gain `schema_version`, `mesh`, `kept_cells`, `node_vertices`, `undefined_cells`, `worst_end`, and `view.scene` gains `volumes[]` with colors (the client colors skin and model by volume).
- `view.scene` mesh `name` is the id text (meshes have no names).
- The clip slider has 4 positions in `fixture://` mode (the golden fixture holds the replies for those) and 32 live.
- The `hexbox` fixture is bundled into viewer.js so `?fixture=hexbox` works offline. The model tessellation and the validate/quality findings in it are hand-built in `tests/fixtures.rs` (no sm-tess / sm-diag dependency, decision 12).
- `Cargo.lock` gained the sm-view dependency rows; it is left out of the commit (root file, other W3b-1 workers touch it too) and the orchestrator regenerates it at merge.

## Backlog

- Live smoke against `smesh serve --ws` (W3b-INT) and HTTP serving of `assets()` (SERVER-HOST); the default `ws://<page host>` assumes the server accepts the upgrade on the page's origin.
- `state.changed` push (O16 later wave), binary frames, LOD.
- Model faces are tinted by the first colored volume only; per-face-volume colors need an adjacency query.
- The 1e5-cell base64 measurement is an `#[ignore]` test (27 s in debug); numbers are in `DESIGN.md`.
