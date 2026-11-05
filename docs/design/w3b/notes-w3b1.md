# W3b-1 notes (2026-11-05)

All 19 W3b-1 tasks passed their Opus review and merged with no conflicts (combine 0241002). The final review's A/B left every W2/W3a state hash and exported file identical, at default thread count and with one thread.

At integration the orchestrator regenerated two goldens. Both changes only add entries for new ops.
- `crates/sm-mcp/tests/golden/tools.standard.txt`: 10 tools.
- `crates/sm-py/python/smartmesh/ops.pyi`: new ops, plus the `mesh.smooth` `set` argument.

## Decisions taken as the tasks implemented them
These read the plan or a draft in a way the plan left open. They stand unless a later ruling changes them.
- **PRIM:** the `model.move_to` target is `{x?, y?, z?}` (an omitted axis is unchanged). The planned `[Option<f64>; 3]` cannot be recorded, because op records refuse null.
- **PRIM:** acceptance 5's Euler wording is corrected. A z-cut hollow sphere part has V − E + F = 3 (annular cap). Only the hollow octant (2) and the whole hollow sphere (4) match the plan text.
- **SCRIPT:** the Raw fallback is tested with a non-canonical id spelling. A dead literal id cannot replay through Raw, because Raw goes through the same schema-guided conversion.
- **EXPORT:** `PolyPolicy::Decompose` splits a polyhedron only when every neighbour across its polygonal faces is decomposed. sm-meshio also depends on sm-topo-api.
- **SURFPLAN:** Q13's guard also refuses a face next to a face of an already-meshed volume. Face runs check the cell budget on the first plan only.
- **VIEW:** `tri_owner` indexes a per-reply owners table, because an `EntityId` does not fit a u32.
- **PY:** the native module is `smartmesh._native` (lib name `_native`).
- **QUALITY:** the two-boundary-face transport flag is a separate diagnostic, not a sixth Transport metric. As a sixth metric it moved an sm-diag test. Equiangle, quad radius ratio and MaxStretch definitions are documented in the crate.
- **PAVE:** valence cleanup is opt-in (`cleanup`). The corner rule runs only after the ladder fails, so meshes the ladder already finds stay unchanged.
- **POLY:** an agglomeration that makes an invalid cell now returns an error instead of Ok. The test hook `with_unchecked_merges` moves behind a `test-hooks` feature (backlog).
- **TET:** no small input reaches the hole stage, so the n=64 cliff regression stays release-only, with a debug restart-free guard.
- **SOS:** the debug grid test checks every 16th ray; release checks every ray. Its counting allocator uses `unsafe` in a test, which the CLAUDE.md wording does not cover (backlog).
- **RTBENCH-A:**
  - P2 is a 3×3 cluster of 17×17 lattices of one pin prototype.
  - H1 uses a structured Kuhn tet mesh with stair-stepped sphere inclusions.
  - F1 is nested torus shells with no ports and no analytic truth.
  - Facet hashes are blessed per OS and architecture, with macOS arm64 only so far.
  - Docker digests are UNSET until the benchmark machine fills them.
  - `--model` cannot tessellate imported periodic faces yet, because sm-cadio leaves pcurves unset.

## W3b-INT hand-offs
- `tools/conformance/requirements.txt`: add `h5py==3.16.0` and enable the `gmsh==4.15.2` pin.
- `tools/conformance.sh`: add `--only dagmc|gmsh`, and the docker build/run of `tools/conformance/dagmc` for Tier B (exit 3 when the image is missing, unless `SM_CONFORMANCE_ALLOW_SKIP=1`).
- EXPORT-WIRE: `DagmcOptions` / `GmshOptions` are `#[non_exhaustive]`, deny unknown fields and have builders. Reports carry `LossReport` with `h5m.*` / `msh.*` codes.
- Run the sm-view web Node job on Node 20 once; it has not run there yet.

## Backlog
- **sm-rt** `tests/flat_golden.rs` `hex_block6`: the bundle hash differs between debug and release, while the structure hash matches. This was already true at 2c80270, so the trace path depends on the build profile. Find the profile-dependent float path (RT-DELTA-WORLD or orchestrator).
- **sm-poly:** move `with_unchecked_merges` behind a `test-hooks` feature.
- **sm-server:** the scoped `OPENING` thread-local binds stores only while plugins install on the thread that calls `open`. Assert this in a test.
- **sm-cadio:** record the dev-only dependency cycle on sm-server in DESIGN.md.
- **sm-rt-bench:** keep `date_utc` and the sequence counter out of the hashed outputs.
- **sm-rt:** the mesh-leaf part of `LeafInput::content_hash` duplicates `mesh_topology_hash` in the builder; share one implementation.
- **sm-boolean:** `merge.rs:390` has its own O(F²) face-box scan. The copy-on-write cost at 10⁵ faces is still open.
- **sm-mesher:**
  - Volume `mesh.size` / `mesh.intervals` do not reach a face-only `mesh.run` (Cubit parity).
  - A re-plan after a failed face run is not budget-checked.
- **CLAUDE.md:** test-only `unsafe` (counting allocators in sm-math and sm-rt tests) is not covered by the current wording. It needs a one-line rule from the user, or the allocators move to a test-support crate.
- **Process:** some workers' commits carry a `Claude Sonnet 5.5` co-author line (the model that actually ran) instead of the requested Opus line. Keep it: it is accurate.
