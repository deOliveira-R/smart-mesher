# W3b-2 notes (2026-11-05)

Eight tasks merged as b67f107: RT-ANALYTIC, AUTO, EXPORT-WIRE, DIAG, CAD-ATTR, FOAM-WARP, SUBMAP and POLY2.
- CI passed.
- A/B against 77c47bb: every state hash and exported file is identical.
- The combine step regenerated the MCP tool list. The diff only adds `mesh__auto`, `mesh__auto_strategies` and `mesh__unflag_cells`.
- RT-DELTA left one acceptance item failing after three review rounds. It is being finished under rulings R1 and R2 (`rulings.md`), and RT-CAPI runs after it.

## Behaviour changes to note
- **AUTO:** every `mesh.run` and `mesh.plan` now allows `16 + 2·n_targets` re-plans instead of 16. Only runs that used to fail with ReplanLimit can change; the A/B showed no hash movement.
- **SUBMAP:** output on periodic faces (pcurve blend) and on graded neighbours (cut fractions) changes, with no `mesh.run` IMPL_VERSION or mesher service-version bump. Twelve existing submap fixtures kept identical hashes.
- **FOAM-WARP:** opt-in (`FoamOptions::split_warped_faces`, default false), and not reachable through `session.export` yet.
- **DIAG:** flag then unflag restores content and sets but not the state hash (ruling R3).

## Ownership deviations accepted
- **EXPORT-WIRE:** about 30 lines in sm-cli (three formats, extension inference, `--option` parsing) instead of one-line edits.
- **New dev-dependency cycles:** sm-server → sm-cli/sm-mcp, sm-diag → sm-smooth. They cost build time only.
- **POLY2:** an orchestrator micro task with no plan block.

## W3b-INT hand-offs (adds to `notes-w3b1.md`)
- `tools/conformance.sh --only dagmc|gmsh` is still not applied.
- Decide whether to bump the `mesh.run` / mesher version for the SUBMAP output change, or pin a graded-input fixture hash.
- Wire FOAM-WARP's `split_warped_faces` through `session.export` (protocol option).
- Regenerate the MCP/CLI goldens and the Python stub.
- Add a W3b note to `crates/sm-diag/W3_REPORT.md`; the test comment in `tests/suggest_pair.rs` points to it.

## Backlog
- **CAD-ATTR:** an explicit `attributes: false` serializes into the op record. The node hash then differs from an omitted field, while the state hash is the same. Normalize `Some(false)` to `None`, or fix the DESIGN.md wording.
