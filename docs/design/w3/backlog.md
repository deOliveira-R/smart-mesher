# W3 backlog (collected at the end of W2)

Sources: W2 integration (decision 10), the post-integration contract batch, and the W2 polish wave (decision 11). Each item names its owner. "Contract" means a frozen crate, so the orchestrator schedules it.

## Contract changes
- **CR-2 completion** (`sm-model-api`): `validate` still uses `FaceMoments::plane`. Switch it to `TopologyView::face_plane`, then drop sm-model's `Tolerance::with_linear(1e-6)` mirror-deck workaround.
- **Plan once** (`sm-scheme-api`): `Driver::mesh_from_plan`, plus `DriverOptions::max_cells` or a cell estimate on `Plan`. Today `mesh.run` plans twice and keeps its own budget.
- **Planner firmness cascade** (`sm-scheme-api`): today an edge request always beats a face or volume request. Cubit's rule is that firmness decides first, then the latest command wins (decision 9).
- **Scheme-derived counts** (`sm-scheme-api`): `IntervalSink::goal` is averaged with the edge's sizing goal and the Default counts. So bias/stretch schemes that derive n from sizes cannot use it: n comes out as 2 instead of 6. They need a preference that replaces the sizing goal without becoming a soft count.
- **Bulk mesh commit** (`sm-layer-api`, `sm-mesh-api`): bulk adopt/retire and `MeshBuild::add_template_cells`. A 10⁶-cell `mesh.run`+`mesh.delete` takes 10.6 s, against 4.7 s for the driver alone.
- **`ModelLayer` from `(TopoHost, Model)`** (`sm-layer-api`): lets sm-fem test against `sm_testkit::topo::mirror_chamber()` instead of an in-test op.
- **Overlay `MeshRead` for export** (`sm-mesh-api`): sm-fem's `clone_host` is O(mesh) per export and has not been measured at 10⁶ cells.
- **Element type in `.smsh`** (`sm-meshio` format): it is checked at export but not written. Fold this into the polyhedral-format work.

## Features
- **`mesh.plan` / explain query:** report which scheme ran or will run on each entity (`mesh.info` has counts only).
- **`mesh.run` on faces alone** (Cubit `mesh surface`). Needed for QUAD4 and the P0 surface-only map.
- **Tri/tet meshing,** for the P0 TET4 and QUAD4 element types.
- **Script record:** write a session's op history out as a `.smsh` script (PARITY A, P1).
- **`smesh serve --ws`:** forward to sm-server's `ws` feature.
- **Sequential meshing against meshed neighbours:** replaces the `mesh.run` neighbour guard (decision 9).

## Small owner items
- sm-server `write_atomic` uses a fixed `.part` temp name. Use a unique one (pid + counter) so concurrent exports are safe.
- sm-mcp: an engine-backed variant of `respelled_ids_feed_back_as_arguments` (needs a real-kernel dev-dependency).
- sm-mesh `contract_alloc`: the warm-up before the measured pass weakens the zero-allocation contract. Drop it (the per-thread counter alone fixed the flake), or assert that the cold pass allocates nothing too.
- A `cargo fmt` pass over sm-mcp and sm-scheme-api (pre-existing drift; CI does not check fmt).

## Validation, quality diagnostics, exporter conformance (user direction, 2026-10-25)
Principle: our own checks decide whether a mesh is valid and how good it is. External tools (OpenFOAM `checkMesh`, Fluent readers, ParaView/meshio) only judge an exporter. If our validator passes and `checkMesh` fails, the bug is in the OpenFOAM exporter. Exporters own each target's quirks (e.g. OpenFOAM's 7-point `wedge` vs 6-point `prism`, Fluent c0/c1 orientation). Internal cells stay exact. Exports are never tuned to one tool.

- **Validity exists; expose it.** `sm_mesh_api::validate` + `validate_geometry` already cover checkMesh's topology part and more (about 35 stable codes: closedness, incidence, orientation, positive volume, normal direction, planarity, classification). Add a `mesh.validate` op (CLI + MCP) that returns the violations with code, entity id, classification (which surface/volume), and location. Every mesh-producing op must pass it in tests, so "our mesh is invalid" is always our bug.
- **Finite-volume face metrics** (`sm-quality`): non-orthogonality, FV skewness, face warp/flatness, face concavity, neighbour volume ratio, cell determinant, interpolation weight. Write each formula in `DESIGN.md` next to the checkMesh / Fluent definition, so our numbers can be cross-checked against `checkMesh` output, not just its pass/fail.
- **Quality diagnostics op** (`mesh.quality`, CLI + MCP): summary, histogram, worst-N cells with location, aggregation per geometry entity (through classification: "bad cells cluster on surface 12 near curve 7"), threshold profiles (generic, openfoam, fluent, transport), and suggested actions (intervals on curve X, scheme change, smoothing). It can store the offending cells as a group, so follow-up ops (smooth, refine) can target them.
- **Exporter conformance harness** (CI, Docker OpenFOAM v2506): for each fixture, our validator passes → export → `checkMesh -allGeometry -allTopology` (polyMesh) / `fluent3DMeshToFoam` + `checkMesh` (Fluent) / meshio round-trip (VTU, CGNS). Any external failure is an exporter bug. Also compare max non-orthogonality and skewness against our metrics within tolerance. References: `polyDualMesh` (tet → poly), Gmsh (tet quality).

## Exporter format rulings (user, 2026-10-25)
- **OpenFOAM polyMesh export: ASCII only.** No OpenFOAM binary writer; users who want binary run `foamFormatConvert` (OpenFOAM's own tool).
- **Fluent `.msh` export: ASCII first** (spec: `docs/research/fluent-msh.md`). Binary waits until the open binary questions (§14 items 5–8) are settled against Fluent itself.
- **Fluent gap:** the user will try to get Fluent (trial or student licence). Checklist once available: (1) load `docs/research/fluent-msh/*.msh` and the exporter's fixtures, confirm no orientation or volume complaints; (2) save a Fluent-written ASCII and binary `.msh` of a small mixed mesh as golden files (settles binary newline, int width, 39 domain-id, solid zone type); (3) check that a polyhedral (type 7) mesh and a non-conformal interface load.
