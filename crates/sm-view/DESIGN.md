# sm-view: design

## Positioning

**Similar to** the ParaView / Cubit graphics windows (skin of a volume mesh, clip plane, per-cell scalar coloring, picking), **differs in** keeping the renderer thin and the meaning on the server: the server answers three read-only queries with renderer-neutral buffers, every triangle carries the dense id of the cell it shows and the topo entity that owns it, and the quality overlay is `sm-quality` output, never a client recomputation; findings from `mesh.validate` / `mesh.quality` are drawn where they are (location markers, owner highlight) with the suggested op text shown but never run; the clip is cell-based and exact (it shows real cells, as a solver sees them) and every reply is byte-identical for 1 or N threads, **giving** a human (browser) and an agent (MCP) one session to look at, with time travel through `at`.

Layers: `sm-view` (L3) depends on contract crates only (`sm-core`, `sm-op-api`, `sm-layer-api`, `sm-mesh-api`, `sm-topo-api`, `sm-model-api`, `sm-quality`). It does not depend on `sm-tess`, `sm-diag`, `sm-server` or `sm-engine`: the client calls `view.tessellate`, `mesh.validate`, `mesh.quality`, `timeline.*`, `session.hash` and `script.run` itself, so the server never joins them. Serving the assets over HTTP is SERVER-HOST.

## Queries (plugin id `view`, all `TypedQuery`, none mutates state)

All take the mesh by id; omitted, it is the only mesh of the state. No mesh gives `NoMesh`, several give `AmbiguousMesh` (the ids are listed), an unknown id gives `UnknownEntity`. `Request::Query.at` selects the revision, so scrubbing the timeline needs no new protocol.

### `view.scene {}`

`{schema_version, bbox, volumes[], bodies[], meshes[]}`, sorted by id. `bbox` is the union of the model vertices (`Carrier::Point`) and every mesh's nodes (zeros when empty; curved edges are not sampled). `meshes[i]` has `id, name, dim, cells, facets, nodes, bbox, digest, derived_from`. `name` is the id text (meshes have no names yet). `digest` is hex `SHA-256("smartmesh.view.mesh.v1" domain, mesh id raw, connectivity_hash, coordinate_hash)` (exact coordinate bits), so it changes whenever the buffers could.

**Colors.** Model colors are labels without RGB. The display color is the fixed rule `PALETTE[label % 12]` (`palette_rgb`, the 12 entries are in `queries.rs` and mirrored in `web/src/batch.ts`). A volume's color is its live model color (`live_volume_color`); a body takes the color of its first colored volume and the color's name. Uncolored: `null`. The same rule colors the viewer's swatches.

### `view.mesh_skin {mesh?, volumes?, clip?, outline=true, max_tris=2_000_000}`

Reply fields (every buffer is padded RFC 4648 base64 of little-endian values):

| field | bytes | meaning |
|---|---|---|
| `positions` | `vertex_count * 12` | `f32 x, y, z` per vertex |
| `indices` | `tri_count * 12` | `u32` ×3 per triangle |
| `tri_cell` | `tri_count * 4` | `u32` dense index of the visible cell |
| `tri_owner` | `tri_count * 4` | `u32`: 0 none, else 1 + index into `owners` |
| `outline_indices` | `2 * 4 * segments` | `u32` pairs, a line list, sorted and unique; empty without `outline` |
| `owners` | JSON | topo entity ids (`TopoFace#2`), ascending, deduplicated |

plus `tri_count, vertex_count, node_vertices, bbox, clipped_cells, kept_cells, mesh`. Byte offsets inside a buffer are `i * stride`; there is no header and no alignment padding. `positions` are the f64 positions cast to `f32` once (the library form `skin()` keeps f64).

**Deviation from the draft: `tri_owner`.** A topo `EntityId` is 8 bits of kind and 56 bits of serial and does not fit a `u32`, and the client needs the kind to say `TopoFace#2` versus `TopoVolume#2`. So `tri_owner` indexes a per-reply table (`owners`); 0 still means "none".

**Kept cells.** A cell is *eligible* when it is active (`cell_active`) and, if `volumes` is given, its classification entity is one of them. It is *removed* when `clip` is given and the centroid (`cell_volume_centroid`, the mesh geometry rule) has `dot(c - origin, normal) > 0`; a centroid exactly on the plane is kept. `clipped_cells` counts eligible cells that were removed, `kept_cells` the rest. There is no geometric cutting: the cut surface consists of real cell facets, so it matches what a solver sees, and it is exact and deterministic. A plane that removes everything gives an empty skin (`bbox` zeros), not an error. A zero or non-finite normal is `BadClip`.

**Skin facets** (dimension-3 meshes, facets in face-index order):

1. exactly one kept side: the facet is drawn from the kept cell's side (outward for that cell: the loop as stored for the left cell, reversed for the right cell);
2. both sides kept with different classification entities (an interface between volumes): drawn twice, left side then right side (double sided, each with its own cell);
3. otherwise nothing.

Surface-only meshes (`dim() == 2`) draw every kept polygon (`cell_corners` cycle, as stored). Clip and `volumes` act on the polygon's centroid and classification as for solids.

**Owner rule.** `tri_owner` names the facet's classification entity when it is a model face (dimension 2) and the facet is a boundary facet or an interface drawn from both sides; otherwise (a facet exposed by the clip or by the volume filter) it names the visible cell's classification entity, i.e. the volume. So with both volumes shown the cap between them reports its model face; with `volumes=[one]` it is a skin face of that volume.

**Triangulation and vertex order.** Triangles are ordered by facet (or cell) index, then side, then fan order. Per facet of n vertices: n = 3 one triangle; n = 4 two, `(0,1,2)` and `(0,2,3)`; n > 4 (polyhedral faces, polygons) a fan from the centroid (vertex mean, the `fan_vertex_mean.v1` apex the mesh geometry rule uses, so area agrees with `face_area_vector`) appended as an extra vertex. Vertex numbering is deterministic: first all mesh nodes in first-use order (scan of the triangles' facets in order), then the centroid vertices in facet order, so every index `>= node_vertices` is a centroid. No hash iteration anywhere. Non-convex dual faces stay watertight because the fan is closed around the shared boundary loop.

**Limits.** The triangle count is computed before any buffer is allocated. Above `max_tris` the call fails with `TooLarge { tris, max, hint }` ("clip, pass `volumes`, or raise max_tris"). Nothing is ever decimated silently.

### `view.mesh_scalar {mesh?, metric, bad_below?, bad_above?, worst=20}`

`metric` is any `sm_quality::Metric` wire name (`scaled_jacobian`, `edge_ratio`, ...). `values` is `compute(mesh, metric, default options)` cast to `f32` once, one per dense cell (NaN where the metric is undefined for the cell's shape or the cell is inactive). `min`, `max`, `mean` are over the finite values (sequential sum in cell order), `null` when there are none; `undefined_cells` counts the NaNs; `bad_cells` counts finite values below `bad_below` or above `bad_above`. `worst` lists up to 20 cells with value and centroid, worst first, ties by cell index; `worst_end` says which end: high when only `bad_above` is given, low when `bad_below` is given, else the sense of the metric's `Profile::Generic` rule (lower is worse when it has none). An unknown name is `UnknownMetric` listing the valid names; a metric no cell shape of the mesh supports is `Unsupported` (typed, names the metric). The client colors a triangle by `values[tri_cell[t]]`.

## Cache keys

The client refetches a skin or scalar buffer only when the key changes: `scene` is keyed by the state hash (`session.hash`), the skin by `(mesh digest, request parameters)`, the scalar by `(digest, metric, bad_below)`, validate and quality by the digest. A time-travel read with `at` uses the same keys, so revisiting a revision is free.

## The web client (`web/`)

TypeScript, raw WebGL2, no runtime dependency (dev dependencies only: typescript, esbuild, playwright-core, @types/node, pinned by `package-lock.json`). The bundle `dist/` (index.html, viewer.js, viewer.js.map) is committed and `assets()` embeds it with `include_bytes!`, so `cargo build` never needs node; `web/dist.sha256` (sha256sum format) is checked by a Rust test, and a second test rebuilds with esbuild into `web/.distcheck` and compares byte for byte when `node` and `web/node_modules` exist (it prints a skip otherwise).

Modules (`web/src/`): `codec.ts` (base64 to typed arrays, length and range checks that throw `CodecError`), `camera.ts` (z-up orbit camera, fit-to-bbox, rays, canonical views), `pick.ts` (CPU ray/triangle pick), `raster.ts` (CPU ID rasterizer, the reference for the browser ID pass), `batch.ts` (buffer expansion and the color rules), `rpc.ts` (JSON-RPC client, `WsTransport`, `FixtureTransport`), `gl.ts` (flat-shaded triangles, ID pass, lines, point sprites), `app.ts` (state and panels), `main.ts` (transport from the URL).

What it shows (read-only): the model (`view.tessellate`: faces with a hatch for `boundary_only`, edges as lines), the mesh skin with outline toggle, per-volume visibility and a clip plane (axis and slider; sends `clip`), a metric selector (`view.mesh_scalar`) with `bad_below` in red, the findings of `mesh.validate` and `mesh.quality` (each with a location marker and an owner highlight; clicking flies the camera to the location and shows the suggested op text with a copy button, never run), the timeline (`timeline.log` and branch tips from `timeline.status`; selecting a node re-requests everything with `at=<node>`, HEAD untouched), a header with state hash and mesh digest, picking through the ID buffer, and a one-line console that sends `script.run` (the only call that can change state, and only on Enter).

**Live updates (O16).** There is no `state.changed` push: the client polls `session.hash` every 2 s and refreshes when the HEAD hash changes (while a historic revision is shown it only refreshes the timeline).

**Transports.** `ws://host:port` (`?ws=`, default `ws://<page host>`): text JSON-RPC 2.0, one socket, replies matched by id; `fixture://name` (`?fixture=hexbox`): replays recorded replies through the same decode path. The `hexbox` fixture is bundled into viewer.js, so `index.html?fixture=hexbox` runs offline. Fixtures are written by the Rust test `fixtures` from real replies (the model tessellation and the validate/quality findings are hand-built in that test, since sm-view does not depend on sm-tess or sm-diag); `SM_BLESS=1 cargo test -p sm-view --test fixtures` rewrites them. A fixture entry is `{match, reply}` per query name or method; the first entry whose `match` is a subset of the request wins (`@at` matches the revision).

**WebGL2 versus WebGPU (Q8).** The buffer contract is renderer neutral (flat arrays, ids, no GL types), so a WebGPU backend can be added later without touching Rust. WebGL2 is used now because it runs everywhere including headless Chromium on SwiftShader, which headless WebGPU does not reliably do. Binary WebSocket frames are deferred; base64 costs a fixed 4/3.

**Base64 overhead (recorded, never gated; `cargo test -p sm-view -- --ignored --nocapture base64`).** A 46×46×47 hex block (99 452 cells): full skin 25 760 triangles, 12 882 vertices, raw 875 864 B, base64 1 167 824 B (ratio 1.3333), whole reply 1 168 165 B; clipped at x = 23: 17 204 triangles, 8 604 vertices, raw 584 960 B, base64 779 952 B, reply 780 297 B. Padding adds at most 2 characters per buffer.

## Testing

Rust (`cargo test -p sm-view`): library and query tests on testkit meshes (acceptance 1-7), byte determinism for 1 and N threads across `view.scene`, clipped and unclipped skin and scalars, a `config(n)` property test for random clip planes on the box (kept plus clipped cells is 16, every skin triangle's facet has exactly one kept side), the fixtures golden, assets and the dist hash. Node (`npm test`, `tsc` then `node --test`): codec against the Rust-written fixture, camera, CPU pick and raster, rpc and fixtures, batch and colors. Browser e2e (`SM_VIEWER_E2E=1 npm run e2e`): headless Chromium on SwiftShader compares the ID buffer with `raster.ts` (limit 0.5% of pixels), clicks, the clip slider and the findings row; skips with a printed reason when `playwright-core` or a Chromium binary is absent. A live smoke against `smesh serve --ws` belongs to integration.

## Not done

Binary frames and diffs, LOD beyond `max_tris`, WebGPU, track overlays, picking-based editing, Tauri, authentication.
