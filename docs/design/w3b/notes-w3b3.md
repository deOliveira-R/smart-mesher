# W3b-3 notes (2026-11-06)

Merged as 40c36c0: RT-DELTA-WORLD, RT-BRIDGE, SERVER-HOST, RTBENCH-B and CAPI2, plus the WebSocket Origin fix acac66b.
- CI passed.
- A/B against 4286563: every state hash and exported file is identical.
- The combine step regenerated the MCP tool list and the session goldens. These changes only add entries: `record_script`, `rt.check_overlaps`, `rt.partition`, three `view.*` queries, and tool counts 19 → 20.

## Fixed before merge
- **`smesh serve --ws` security:** the WebSocket upgrade now accepts a browser `Origin` only when it matches the served address; any other origin gets 403. Live connections are capped at 64, and the 65th gets 503. The reviewer probed for bypasses (case, ports, `null`, IPv6, userinfo, rebinding) and found none.
- **Census test:** it no longer leaves `out.smsh`/`out.vtu` in `crates/sm-server`.
- **Debug/release hash difference (sm-rt `flat_golden`):** in release builds the compiler merged `sin`/`cos` into one `sincos` call, which rounds differently. That is removed, and the hashes now match across build profiles.

## Orchestrator rulings
- **R4.** SERVER-HOST's sm-cli/sm-mcp edits (`smesh record`, `serve --ws` with a default `ws` feature, about 130 + 28 lines) are accepted, as EXPORT-WIRE's were.
- **R5.** CAPI2, like POLY2, is an orchestrator micro task with no plan block.
- **R6.** The combine step's additive MCP golden regeneration is covered by O13. W3b-INT regenerates only what new INT work adds.
- **R7.** RT-DELTA-WORLD's chord-sum tolerance `1e-12·max(1, 0.05/min|d_k|)` on grazing lattice lines follows the same argument as R1. It substitutes a lattice-vs-single-mesh comparison for the mirror-lateral slab, because a world has one boundary rule.
- **R8.** AUTO acceptance:
  - **B6** (auto next to a hard tet neighbour) fails and stays pinned. W3c micro task in sm-mesher's `DefaultStrategy`: respect triangle faces imposed by a neighbour, and blame the auto volume so the tet fallback is reached.
  - **B5** (plate with a hole) is a known limit; its outcome is pinned.
  - **B3 (second half) and B4 (hex-only):** covered only by sm-mesher stub tests, because `standard()` registers only `default`.
  - **Census:** 7 changed cases, all error → success except one error → a different error. No golden moved. Accepted as satisfying Q5 / O10.

## Open for the benchmark (with O11, the Linux machine)
- The DAGMC drivers implement W-track only. W-fire and W-flight, the XDG driver and `smtets_to_moab` are not written.
- The analytic row has no H1 case, and P2 has no tet twin.
- The P1 `facets` row at `tiny` scale is partial (zero-area triangles in the hand-built moderator).
- Linux `smcase.json` hash lines are unblessed until the machine exists.

## W3b-INT hand-offs (in addition to notes-w3b1.md and notes-w3b2.md)
- Repeat SERVER-HOST acceptance 2 (`script.record`) on `tools/w3b_milestone.smesh`.
- sm-cli `serve` usage text: mention the Origin rule and the connection cap.

## Backlog
- **RT-DELTA-WORLD `tests/delta.rs` acc04:** the `bvh_descents == 0` bracket now covers only `probe`. Bracket `next_segment` too.
- **RT-BRIDGE:**
  - The 2D lattice z pitch is a 1e9 sentinel.
  - Analytic void/complement stepping and cold locate are O(surfaces)/O(cells); a cell-box BVH is deferred.
- **Accept loop:** the 503 refusal runs on the accept thread (up to ~1.2 s stall per excess connection).
- **`SMRT_NODE_UNKNOWN = 5`:** added without an ABI version bump (additive).
