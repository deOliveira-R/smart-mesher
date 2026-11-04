# W3b rulings (user, 2026-11-04)

These override the W3b drafts and the planning critique. Where a ruling leaves a gap, the plan lists it as open; it does not fill it.

## Ray tracing
- **Q1. Contract base.** fd4a235 (segment-iterator-first, branch `w3b-panel/rt-1`). Grafts: opaque resumable cursor; C ABI from 30e2742; deduplicated material sets; `block(node)`, face_in/out; safety distance plus REGION_JUMP. Dropped by Q2: the cross-section table bound to structure_hash, `from_flat`, and ed4ea61's surface-mode σ_t(E) sampling. Losers archived as tags `archive/w3b/rt-api/hierarchy-first` (ed4ea61) and `archive/w3b/rt-api/transport-first` (30e2742).
- **Q2. Cross sections never enter smart-mesh or sm-rt.**
  - smart-mesh exports a file: geometry, connectivity and the material name of each region.
  - The transport code opens the file through sm-rt (a library it links, via the C ABI) and pairs the material names with its own cross sections.
  - For delta tracking, sm-rt reports which materials each spatial region (lattice cell, tree node) contains. The caller computes the majorant per region and passes one number per region. sm-rt holds no σ_t(E).
  - sm-rt reports a collision whose caller-supplied σ_t exceeds the region majorant as an error.
  - Re-check that `BuildOptions` stays `Copy`.
- **Q3. Hex lattices: stretch.** Pointy-top only. Flat-top through a 30° rotated placement; no HexY kind. ANALYTIC acceptance 9 moves to stretch, and the HexY ACR is dropped. Meshing hex assemblies (sweep from a meshed face) is the existing sweep scheme, not part of this item.
- **Q4. CSG cells must be disjoint.** First-wins order is removed, and so is the stub's Union priority rule. The stub parity tests that rely on it change, and any golden change needs an explicit bless ruling.
  - Overlap cannot be proven absent for general CSG. A Check-stage op samples points and rays to detect overlaps, as OpenMC's geometry debug mode does.
  - The MCNP and OpenMC importers reject overlapping decks.
- **Q9. Benchmark.** The published table comes from one bare-metal Linux x86_64 machine, arranged by the user; a macOS run is a smoke result only.
  - The repo carries a synthetic fusion-like model.
  - A `--model <path>` flag takes a local STEP file, which is never committed. The user has several STEP files and can generate test geometry through a FreeCAD MCP.

## Meshing
- **Q5. Automatic meshing is pluggable.** `mesh.auto {strategy}` with registered strategies. The `default` strategy:
  - surfaces: map, submap, pave, trimesh;
  - volumes: map, sweep, tetmesh, with a `mesh.auto_tet` warning on the tet fallback;
  - triangle sources: pave.

  **Determinism constraint:** a strategy only chooses schemes, and its choices are written into history as explicit scheme assignments. Replay never re-runs a strategy, so improving or replacing a strategy never moves an old state hash. Explicit-scheme runs stay bit-identical, and every case that changes is reported.
- **Q13.** Surface-only `mesh.run` on a face that shares an edge with a face meshed in another entry is refused, with an error naming the fix.
- **Q14.**
  - Ratify the provisional transport quality thresholds.
  - `mesh.smooth {set}` reuses `flag_cells` set names.
  - The tet size floor stays a reported preference.
  - STEP names become groups first; colors come only from solid-attached colors.
  - The overlay MeshRead stays deferred.

## Exports and model
- **Q6. DAGMC h5m.**
  - Download a small MOAB-written two-volume h5m with graveyard from DAGMC or OpenMC test data (licence-checked) and commit it with its h5dump.
  - Materials are named `mat:<set name>` and `mat:Graveyard`; the graveyard is opt-in.
  - msh4 is ASCII by default, binary as an option.
  - Polyhedra to msh4 is an error by default.
- **Q10. STEP export is a Model-stage op** (`model.export`, using the existing `sm_cadio::write_step`). It is not part of the mesh export wiring. Mesh export wiring (DAGMC h5m, Gmsh msh4) is one Export-stage unit (protocol, server, cli, lang). These are two tasks.
- **Q11.**
  - New `model.create_sphere_part`; `create_sphere` is unchanged.
  - Copies carry no names, colors, groups or boundary kinds (`model.copy_attributes` comes later).
  - Prism and pyramid radius is the circumradius, with vertex 0 on +x.
  - `move_to` anchors on the selection centroid.
- **Q12.** Script record:
  - `Check::Tips` by default;
  - `Forks::Head` on a non-main head leaves a documented extra `main`;
  - no messages or authors in comments.

## Tooling
- **Q7. Python bindings.** Package `smartmesh`, Python ≥ 3.10 abi3, not on PyPI, no zero-copy views. CLAUDE.md now allows PyO3 macro-generated `unsafe` in `crates/sm-py` (ddc2122).
- **Q8. Web viewer.** WebGL2 with a renderer-neutral buffer contract. Client in `crates/sm-view/web/`, read-only plus script console. The Node CI job is non-blocking.

## Contract-batch proposals (approved by the user 2026-11-04)
From the post-W3a contract batch review. P1 still flips only after a milestone A/B with unchanged hashes.
- **P1.** Firmness cascade:
  - resolve at command time in `mesh.size` / `mesh.intervals`, so stored intent is order-free;
  - flip `mesh.run` to Firmness only after a milestone A/B with unchanged hashes;
  - any hash change needs a ruling.
- **P2.** Re-bless the `mesh.quality` output schema golden once, to publish `scheme_notes` / `n_scheme_notes`.
- **P3.** Move scheme notes from `Plan` to an unhashed `MeshEntry::notes`. Until then, `mesh.to_poly` clears `plan.notes`.
- **P4.** sm-op-api ACR `ApplyCtx::retire_range` for bulk delete, with a test that the hash equals per-id retire.
- **P5.** sm-protocol ACR: `session.export` gets `split_small_determinant` (default true) and empty/wedge patch naming.

## Plan open items (ruled by the user 2026-11-04, accepting the recommendations)
- **O1.** The transport code opens a self-contained `.smrt` file written by `session.export` with `ExportFormat::Smrt`.
- **O2.** One majorant per region, passed by the caller on each tracking call. There is no energy-band index in the contract.
- **O3.** The caller chooses between surface and delta tracking per region (`set_policy`, `block`).
- **O5.** `model.export` is a read-only query. The file write happens outside history, like `session.export`.
- **O6.** `rt.check_overlaps` is a read-only query, seed 0 by default.
- **O7.** MCNP/OpenMC importers (which reject overlapping decks) go to W3c.
- **O12.** Ask the user for FreeCAD-generated shapes with known volumes when the CSG bridge starts.
- **O16.** The viewer polls `session.hash`; server push waits for a later wave.
- **O17.** Overlaps between bodies are found at track time and by `rt.check_overlaps`, not proved absent at build.
- **P1-P5.** Approved.
