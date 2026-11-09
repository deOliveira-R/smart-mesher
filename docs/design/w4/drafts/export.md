# W4 export area: exporters, export options, sizing functions, GPU, CLI diagnostics, test hygiene

> Planning draft (W4), read-only against `main` at `d2f4803`. Feeds the W4 plan tasks EXPORT-*, SIZE-*, GPU-DECIDE, CLI-DIAG and HYGIENE-*. The standing principle applies: Cubit is a bar to surpass. MCNP is not a target. OpenMC is the CSG format and the oracle.
>
> **Critique (Opus) of this draft:** every file:line claim was re-checked at `d2f4803`. Corrections are marked **Critique delta:** inline. The main changes: two backlog items cited as open are already done, scheme-derived counts need no ACR, relative intervals and curvature curve meshing already exist in part, the DAGMC model path does not make the benchmark geometry-identical (it already is), CGNS must be one zone, a Fluent exporter (user ruling, full spec) was missing, the curvature field needs geometry at build time, three of seven questions were not user decisions and a fourth is partly answered by the user's moose env.

## Findings (what is true today, with file:line)

### Exporters
- **PLAN.md lists five "remaining exporters"** (Gmsh, Abaqus, Nastran, CGNS, DAGMC) at `docs/PLAN.md:75`. Two of the five already exist.
  - **Gmsh MSH 4.1 exists**, ASCII and binary (`crates/sm-meshio/src/gmsh/mod.rs:1-12`).
    - Polyhedra follow `PolyPolicy` `Error | Decompose | Drop` (`gmsh/mod.rs:28-42`), because msh4 cannot carry them. **Critique delta:** line range was 31-45; the enum is at 32-42.
    - A cell set that covers part of a `TopoVolume` splits that volume into several Gmsh entities, with loss code `msh.entity_split` (`gmsh/mod.rs:4-8`).
    - High-order cells are written as their linear corner types, with loss item `msh.high_order_dropped` (`gmsh/write.rs:252,293-335`). **Critique delta:** they are degraded, not dropped (`write.rs:335`: "high-order cells are written as their linear corner types").
  - **DAGMC `.h5m` exists** (`crates/sm-meshio/src/dagmc/mod.rs:1-12`).
    - Triangles, surfaces with forward and backward volumes, `mat:<set>` groups and an opt-in graveyard, written by a private pure-Rust HDF5 subset (`crates/sm-meshio/src/h5/mod.rs:1-13`, `crates/sm-meshio/H5M_FORMAT.md`). **Critique delta:** `dagmc/write.rs:245-250` is `export_h5m`; the HDF5 subset is `h5/mod.rs`.
    - A volume that does not close is `DagmcError::OpenSurface` (`dagmc/mod.rs:11`).
    - `model_from_mesh` needs a dimension-3 mesh and at least one cell (`dagmc/model.rs:173-179`).
    - **Gap:** `model_from_facets` (`dagmc/model.rs:326`) is called only by `crates/sm-meshio/tests/dagmc.rs`. `session.export` has no path to it (`crates/sm-server/src/mesh_export.rs:117-124` calls `export_h5m`, which uses `model_from_mesh`). A model with no volume mesh cannot be exported as h5m.
    - The benchmark does not use our writer. It builds DAGMC inputs with MOAB's C++ `crates/sm-rt-bench/drivers/dagmc/facets_to_h5m.cpp`.
    - **Critique delta:** the benchmark already feeds both trackers from one tessellation. `facets_of_complex` (`crates/sm-rt-bench/src/twin.rs:59-71`) tessellates the B-rep with `sm_tess`, the result is `facets.bin`, MOAB turns it into the h5m (`facets_to_h5m.cpp:1-11`) and sm-rt builds `BuildInput::Facets` from the same file (`crates/sm-rt-bench/src/smrt.rs:101`). There is no production adapter from `sm_tess::Tessellation` to `sm_rt_api::FacetInput`; the only one is the bench's `twin.rs`. `sm_tess` stores `f32` positions and uses platform `sin`/`cos`, so facet bytes are blessed per `os-arch` (`twin.rs:4-5`).
  - **Exodus II exists** (`crates/sm-exodus`, `options.rs:1-60`). It has NFACED polyhedra marked "experimental: unverified against SEACAS" (`options.rs:36-38`) and no netCDF-4/HDF5 (`docs/PARITY.md:101`). **Critique delta:** PARITY line is 101, not 100. The structural half of NFACED checking already exists: `tools/conformance/check_exodus.py:54-110` rebuilds NFACED cells from `facconn`/`fbconn` and checks side sets. Also, `sm-protocol/src/request.rs:18` documents Exodus as "NetCDF-4", which is wrong (CDF-1/2/5); a doc-only fix in a frozen crate.
  - **OpenFOAM, VTU and `.smsh` exist.** `.smrt` is a ray-tracing cache, not an interchange format.
- **Abaqus, Nastran and CGNS do not exist.** `sm-meshio` has no module for them (`crates/sm-meshio/src/lib.rs:15-33`) and `docs/PARITY.md:105-106` marks them open. Nothing in `docs/research` specifies Abaqus or Nastran. CGNS is covered by research (`docs/research/mesh-ir-synthesis.md:12,38-40`), which names it "good enough to adopt as the neutral primary interchange".
  - **Critique delta:** the only Abaqus mention in research (`docs/research/delta-tracking.md:126`) is the MCNP6 unstructured-mesh workflow (Abaqus-built UM meshes). MCNP is not a target, so that is not a reader we serve.
- **Critique delta (missing): Fluent `.msh` export is a standing user ruling with a full spec.** `docs/design/w3/backlog.md:37-40` (exporter format rulings, user, 2026-10-25): "Fluent `.msh` export: ASCII first (spec: `docs/research/fluent-msh.md`)". The spec covers polyhedra (cell type 7, `fluent-msh.md:240`) and was checked against OpenFOAM v2506's `fluent3DMeshToFoam` + `checkMesh`, including a 1060-polyhedron dual (`fluent-msh.md:9-17`). The conformance harness already names that pipeline (`backlog.md:35`). `PARITY.md:106` lists Fluent as P2. No Fluent module exists in `sm-meshio`.
- **A private HDF5 writer already exists** (`crates/sm-meshio/src/h5/`). The `.h5m` writer uses it. CGNS/HDF5 could reuse and extend it. **Critique delta:** the subset is superblock v2, object header v2, compact links sorted by name, no creation order, contiguous datasets, no chunking or variable-length data (`h5/mod.rs:1-13`). Whether the CGNS library accepts groups without link-creation-order tracking is unverified (see Risks).
- **The export option plumbing is in place.**
  - `session.export` carries `split_small_determinant`, `split_warped_faces`, `empty_patches` and `wedge_patches` for OpenFOAM, plus a per-format `options` object (`crates/sm-protocol/src/request.rs:214-245`). Notes carry a severity, and the CLI, MCP and Python surface warnings (`docs/PARITY.md:102-103`; W3c EXPORT-A/B/C).
  - The server validates per-format option keys through `options_map` (`crates/sm-server/src/mesh_export.rs:39-94`). New formats are added there.
  - **Critique delta:** only Gmsh, DAGMC and `.smrt` are dispatched by `mesh_export` (`mesh_export.rs:23-25`). Every other format goes through `sm_fem::export` (`host.rs:248-253`), and `host.rs:202-205` rejects any `options` for those formats. So `.smsh`, VTU, Exodus and OpenFOAM accept no option object today.
  - The W3c draft `docs/design/w3c/drafts/export-opts.md` is therefore **done**. Only the backlog row "Export options: by block/sideset/nodeset, 2D/3D, renumbering, everything/overwrite" (`docs/PARITY.md:107`) is open.
  - `ExodusOptions` has block selection (`BlockSelection::Labelled | Names | None`, `crates/sm-exodus/src/options.rs:5-16`). Other formats have no selection. **Critique delta:** this is library-only. `sm_fem::export` always uses `ExodusOptions::default()` (`crates/sm-fem/src/export.rs:249`), so no user surface reaches it.
- **Backlog items that touch export.**
  - `smtets_to_moab`, trigger: the XDG tet walk (`docs/design/w3/backlog.md:107`).
  - ~~`write_atomic` uses a fixed `.part` temp name (`backlog.md:24`).~~ **Critique delta:** done. `host.rs:489-494` names the temp file with pid and a counter. Mark the backlog row done.
  - Overlay `MeshRead` for export: sm-fem's `clone_host` is O(mesh) per export and unmeasured at 10^6 cells (`backlog.md:12`; the copy is `crates/sm-fem/src/lower.rs:128`).
  - ~~Element type is checked at export but not written to `.smsh` (`backlog.md:13`).~~ **Critique delta:** done. `.smsh` 1.1 writes `element_types` (`crates/sm-meshio/SMSH_FORMAT.md:71,163`) and `sm-fem/src/export.rs:238-240` fills it. Mark the backlog row done.
  - A DAGMC reference volume sits in two groups (`backlog.md:92`, a note for the export docs only).
  - `RegionInfo.tags` is not in cache images (`backlog.md:98`). The trigger is the first producer, for example DAGMC property import.

### Sizing
- `crates/sm-sizing` has four parts.
  - Exact integer-program interval matching (`iia.rs`, 2352 lines): hard and soft counts, equal-edge classes, lexicographic objective, double-double tie checks (`iia.rs:1-40,1728-1801`).
  - The corner picker (`picker.rs`).
  - `size_field`, with three named functions: `linear`, `geometric` and `bias` (`field.rs:1-29`). `Constant` is served by `sm_scheme_api::sizing::field_for`.
  - `auto_size`: `clamp(0.01 d 2^((f-1)/2), l_min/4, d/2)` (`auto.rs:1-12,117-118`).
- **There is no curvature or proximity size field.**
  - `size_field` knows no `curvature`, `skeleton` or `field` name. An unknown name returns `SchemeError::BadParams` (`field.rs:38`). **Critique delta:** line 38, not 35.
  - `SizingSpec::Function` documents "curvature, linear, bias, field, ..." (`crates/sm-scheme-api/src/intent.rs:380-399`), but the doc is aspirational.
  - `auto_size` depends on the bounding box and the shortest edge. It has no local feature size.
  - sm-tess has `chord_tol` and `angle_tol` for faceting, not for meshing (`crates/sm-tess/src/lib.rs:9-12`).
  - **Critique delta:** curvature-driven *curve* meshing already exists. `sm-scheme-curve` registers `equal`, `bias`, `dualbias`, `stretch` and `curvature` (`crates/sm-scheme-curve/src/lib.rs:1,26-33`; density rule in its `DESIGN.md:28`), and the distribution registers them (`crates/sm-server/src/standard.rs:25`). `docs/PARITY.md:59` still shows them unchecked; that row is stale.
  - **Critique delta:** the size-field builder gets no geometry. `SizeFieldFn` is `Fn(&SizingSpec) -> ...` (`crates/sm-mesher/src/config.rs:15-16`), injected as `sm_sizing::size_field` (`standard.rs:184`). A curvature field cannot be built through this seam as it stands.
- **The size field is isotropic in use.**
  - The `SizeField` trait has `size(p)` and a default isotropic `metric(p)` (`sm-scheme-api/src/sizing.rs:14-24`).
  - It is consumed by the tri chart (`sm-scheme-tri/src/chart.rs:212,656`), the tri front (`front.rs:294,470`), the tet scheme (`sm-scheme-tet/src/scheme.rs:234-249`), edge interval goals (`sm-scheme-api/src/geom.rs:68-77`, adaptive Gauss to 1e-10, up to 2^14 panels) and the driver (`sm-mesher/src/mesh_ops.rs:262-271`). **Critique delta:** added the tet, front and edge-goal consumers; the edge goal integrates `1/h` along every edge, so `size(p)` must be cheap and smooth enough for that quadrature to converge.
  - A size spec is a model-wide default (`Defaults.sizing`, `intent.rs:432-441`). There is no per-entity-set field and no field-from-file. **Critique delta:** removed "plus per-target sizing in the plan (`plan.rs:110`)". `Plan.sizing` is the plan's copy of the intent's default sizing, so `generate` can rebuild the field (`plan.rs:108-110`); it is not per-target.
- **Interval-matching gaps** listed in `docs/PARITY.md:56,59`: relative intervals, periodic intervals, vertex sizing, pinpoint/copy curve schemes, "mesh preview". **Critique delta:** bias, dualbias, stretch and curvature are no longer gaps (see above).
  - **Critique delta:** relative intervals already have a contract and a solver. `MeshIntent.intervals` is documented as "match intervals (`sum_equal`), relative intervals and sums over a chain, as rows of the IIA system" (`intent.rs:485-489`), as `IntervalConstraint::Linear` rows (`sm-scheme-api/src/problem.rs:100-119`), and `iia.rs` solves them. The only producer is a test (`sm-mesher/src/intent_layer.rs:159`). The gap is a user-facing op, not solver rows.
  - **Critique delta:** the scheme-derived-counts contract is already done. `IntervalSink::prefer` and `Origin::Preference` landed in 3a7f17e (`problem.rs:35-43,1040-1058`), and the reference solver honours them: soft beats preference beats default goals (`solve.rs:226-229,250-256`). Two things are left, neither frozen: (a) the production `IiaSolver` ignores the origin and treats a preference as a plain default goal (`iia.rs:1758-1761`); (b) the curve schemes still emit derived counts as `out.fix(e, n, Firmness::Soft)` (`sm-scheme-curve/src/schemes.rs:173-175`), not `prefer`. The backlog line `backlog.md:9` is stale as a contract item.
  - A failed re-plan after a face run is not budget-checked (`backlog.md:94`).

### GPU
- Nothing exists. `docs/ARCHITECTURE.md:9,21` plans `wgpu` compute "later" for smoothing, quality and big queries.
- The CPU code is already rayon-parallel and deterministic (`sm-smooth/src/smooth.rs:150,162,272,317`; `sm-quality/src/check.rs:23,41`).
- Determinism is binding (`CLAUDE.md`: bit-identical for 1 thread and N threads). A GPU path has to meet that on three drivers (Metal, Vulkan, DX12).

### CLI per-op diagnostics
- `smesh run` text mode calls `render::script` (`crates/sm-cli/src/render.rs:42-64`). It prints `r.output` to stdout and only the script-level `r.diagnostics` as one compact JSON line on stderr (`render.rs:53-62`).
- Per-op warnings are not script diagnostics. They live in each executed command's event, as `Event::Op { delta: Some(delta) }` (`crates/sm-lang/src/interp.rs:931`; `interp.rs:919` is the plan-only path with `delta: None`).
  - The server serialises them into `ScriptResult.events` (`crates/sm-server/src/server.rs:117-134`, shape in `event_json`, `server.rs:305-318`; field `events: Vec<Value>`, `sm-protocol/src/result.rs:250`).
  - `render::script` never reads `events`. So the `mesh.auto_tet` warning (`sm-mesher/src/auto.rs:597`, one per volume) is invisible in text mode of `smesh run`.
- `render::apply` already prints `delta.diagnostics` as `severity: code: message` on stderr (`render.rs:77-87`) for single ops. The format exists. It is not applied to scripts.

### Test hygiene backlog
These are the items in the "From W3c-INT review" list (`docs/design/w3/backlog.md:145-154`) and the carried list (`backlog.md:129-141`). None is blocking.
- **`milestone_w3c_pin`** (`crates/sm-cli/tests/milestone_w3c_pin.rs`).
  - P5/P6/P9 skip with a println when maturin is missing (`milestone_w3c_pin.rs:484`). They should fail under `CI=true`.
  - Add a per-block fuel volume check against the polygonal-cylinder area times H.
  - Add a no-blocks fixture.
  - Replace `position(..).unwrap_or(0)` with `enumerate` in the `ENCLOSED_HASHES` lookup.
- **`tools/w3c_driver.py`:** run mypy and ruff.
- **sm-py `crates/sm-py/ci/py.sh`:** pass `--basetemp`.
- **sm-cli:** `target/tmp` per-pid dirs need a Drop guard (the helpers already use `CARGO_TARGET_TMPDIR`, `backlog.md:126`). Other crates (sm-engine, sm-rt, sm-mcp, sm-server, sm-cadio, sm-rt-bench) use `temp_dir()` and should move to `CARGO_TARGET_TMPDIR` when next touched.
- **sm-csgio `crates/sm-csgio/tests/fixtures/openmc/gen.py`:** `openmc.reset_auto_ids()` per model, and exit 143 on SIGTERM.
- **sm-server:** assert distinct `copy_of` values in the through-pin plan rows.
- **sm-rt-bench:** check the `sha256:` image id before the platform, assert `image_id` in `absorber_unbiased`, and give an exact lost count in `flight_step_into_an_unmapped_region_is_lost_not_a_leak`.
- **`tools/platform/collect.py:58`:** `error.new` goes through `last_line(r)`. Record the macOS `rustc -V`. Run `cargo fmt` on `platform_pins.rs`.
- **sm-mesher `auto.rs`:** comment that the `known` shortcut assumes `plan_with` is pure.
- **Other:** `RegistryError::BadOpName` is misused for a test timeout in `crates/sm-server/src/standard.rs:282-284`. The `sm-rt-capi` DESIGN.md:111 line is 118 columns wide.
- **Ownership problem.** These files sit in many crates and in `tools/`. Under CLAUDE.md a worker owns one directory. So the hygiene work needs per-crate micro-tasks or one orchestrator commit. It cannot be one worker. **Critique delta:** three items the draft assigned to the orchestrator are in crates: `platform_pins.rs` is `crates/sm-cli/tests/` (sm-cli), the `BadOpName` use is `crates/sm-server/src/standard.rs` (sm-server; a new generic variant would be an `sm-op-api` ACR, `crates/sm-op-api/src/error.rs:111`), and `DESIGN.md:111` is `crates/sm-rt-capi/` (sm-rt-capi).

## Design

### D1. Exporters: what to build, and why this set
The goal is the user's transport code and ray tracing, so the question for each format is "who reads it, and does it carry polyhedra?"

| Format | Reader | Polyhedra | Verdict |
|---|---|---|---|
| `.smsh` | the user's transport code (if it reads our native format) | yes | done |
| Gmsh msh4 | Gmsh, meshio, FEM codes | no (`PolyPolicy`) | done, keep as is |
| DAGMC h5m | OpenMC, DAGMC, XDG | n/a (facets) | done; add the model-only path (D2) |
| Exodus II | MOOSE, Griffin, libMesh (OpenMC unstructured mesh) | NFACED, unverified | done; verify NFACED (see below) |
| CGNS/HDF5 | ParaView, CFD, SU2 (PETSc rejects NGON) | NGON_n + NFACE_n | **build** |
| Fluent `.msh` (ASCII) | Fluent, OpenFOAM `fluent3DMeshToFoam`, many CFD readers | cell type 7 | **build** (user ruling, spec exists) |
| Abaqus `.inp` | FE codes (structural/thermal) | no | **build, small, later** |
| Nastran bulk data | FE codes | no | on request (W5+) |

**Critique delta:** added `.smsh` (the most direct route to the user's own code, already polyhedral) and Fluent. Removed "Martz-style UM meshes" as an Abaqus reader: that is MCNP6 UM, and MCNP is not a target.

- **CGNS (HDF5, `NGON_n`/`NFACE_n`, `ElementStartOffset` CSR).**
  - It is the one new format that carries polyhedra and has a formal spec and a checker (`cgnscheck`). The research synthesis already picks CGNS as the neutral interchange (`mesh-ir-synthesis.md:38-40`).
  - It reuses the in-tree HDF5 writer, so no new dependency. It is a deterministic pure function of the mesh, like the other writers.
  - **Critique delta:** one unstructured `Zone_t` for the whole mesh, not one zone per cell set. Separate zones have separate node numbering, so shared interface nodes would be duplicated and need `GridConnectivity_t` to stitch, which most readers handle badly. Blocks become `Elements_t` sections (one per block and standard type, or one `NGON_n`/`NFACE_n` pair) and `Family_t` nodes. Face sets become `BC_t` with `GridLocation FaceCenter` and a `PointList` of face element numbers. Cell sets that are not blocks become `ZoneSubRegion_t`.
  - The oracle is `cgnscheck`, the CGNS library (`cgnslist` or pyCGNS) and ParaView's CGNS reader. `h5py` checks structure only. This is the existing rule: external tools judge exporters, our validator judges meshes (`backlog.md:30-35`). **Critique delta:** dropped meshio as an oracle for polyhedra; as far as known its CGNS reader does not handle NGON/NFACE (confirm in the spike). None of these tools is installed: the openmc env has `h5dump` and `mbconvert`, no `cgnscheck`; meshio lives in the conformance venv (`tools/conformance.sh:12-13`). Adding conda-forge `cgns` to the conformance env is an orchestrator `tools/` step.
  - Parallel-HDF5 `cgp_*` and time series are out of scope.
  - **Better than Cubit:** ours writes NGON/NFACE from the same exact internal cells as OpenFOAM, with signed face references checked for closure. **Critique delta:** removed "Cubit's CGNS export is unstructured-only and drops general polyhedra". Nothing in the repo or research sources it.
- **Fluent `.msh` (ASCII).** **Critique delta (new):** The user ruled "ASCII first" (`backlog.md:38`) and the spec is written and checked against OpenFOAM (`docs/research/fluent-msh.md`). It carries polyhedra (cell type 7) with face-based connectivity that maps 1:1 to our OpenFOAM face list (right-hand normal toward c0, `fluent-msh.md:20,365`). The oracle exists today: `fluent3DMeshToFoam` + `checkMesh` in the OpenFOAM docker the conformance harness already runs. It is cheaper to verify than CGNS and reaches more CFD tools. The spec's open items (§14, binary layout and Fluent's own reader) stay open; ASCII only.
- **Abaqus `.inp`.**
  - It is a small ASCII format: `*NODE`, `*ELEMENT, TYPE=C3D4/C3D8/C3D6/...`, `*ELSET`, `*NSET`, `*SURFACE`.
  - It is the interchange for the structural/thermal side of nuclear work. **Critique delta:** removed "and for UM-style delta-tracking meshes" (MCNP6 UM).
  - The exporter owns Abaqus quirks. Examples: C3D6 wedge node order, and the face numbering of element-based `*SURFACE` records. The face numbering is the part that needs a reference to check against. **Critique delta:** a 5-node pyramid element name in Abaqus/Standard is unverified; confirm it from the Abaqus element library before promising pyramids, else pyramids follow the polyhedron policy.
  - Polyhedra are an error (`PolyPolicy`-style: `error | decompose | drop`, shared with Gmsh). Ours adds a loss report for everything dropped. **Critique delta:** node order and face numbering come from Abaqus' documented element tables, not "Cubit's baseline".
  - Abaqus gets the shared set selection (D3) from the first version.
  - The oracle is a round-trip through meshio (reads `.inp`) and node-order checks against Abaqus' documented element tables. We have no Abaqus licence, so this is stated as a limit.
- **Nastran.** It is the same shape as Abaqus (small fixed/free-field ASCII, no polyhedra) and has the least user demand. Defer until asked. When built it reuses the Abaqus set-selection code and the shared loss report.
- **Not building:** Patran, I-DEAS UNV, LS-DYNA, Sierra, VRML. `PARITY.md:106` keeps them P2. This draft does not schedule them.
- **Exodus NFACED.** It is "unverified against SEACAS" (`options.rs:36-38`). Verifying it is cheaper than a new format. See EXPORT-EXO-VERIFY. **Critique delta:** removed "the only route to MOOSE/Griffin for polyhedral transport meshes". Griffin's polyhedral support is an open item (`docs/research/mesh-ir-critic.md:17`), and the Exodus library cannot resolve side sets on NFACED (`mesh-ir-synthesis.md:12`). The user's moose env has libMesh 1.9.0-pre with `C0Polyhedron` and `meshtool-opt` (`/Users/rodrigo/opt/miniforge/envs/moose/libmesh/`), which is a real local reader to test against. Its installed headers do not mention NFACED, so the test may show that libMesh cannot read it; that is a useful result either way.

### D2. DAGMC: the model-only export path
Today the h5m comes from a volume mesh's skin and interfaces (`model_from_mesh`). DAGMC does not need a volume mesh, only closed triangulated surfaces.
- **Design.** A query-time path "model facets to h5m": `sm-tess` tessellates the merged B-rep, a new adapter turns the `Tessellation` into `FacetInput` (face sides give forward and backward volumes, as `twin.rs:59-71` does for the bench), and the existing `model_from_facets` (`dagmc/model.rs:326`) builds the model. Group names come from the model's blocks (`mat:<name>`).
- **Critique delta (why):** the draft said this makes the `.smrt` versus DAGMC/XDG comparison geometry-identical. It already is: the bench builds both from one `facets.bin` (Findings). Session `.smrt` export is a mesh tracker (`BuildInput::Mesh`, `mesh_export.rs:170`), not a facet one, so "the same command also writes the `.smrt` from the same facets" is not true either. The real value is user-facing: an OpenMC user gets a DAGMC h5m from STEP without Cubit or MOAB, in one deterministic command. Cubit is the standard DAGMC authoring tool today, so this is a direct surpass item.
- **Why better than DAGMC's own workflow:** DAGMC users run Cubit or CAD-to-DAG to make an h5m. Ours is one command from the merged B-rep, every volume is checked closed (`OpenSurface`), and the bytes are identical across thread counts. **Critique delta:** bytes are per platform, not universal: sm-tess uses `f32` positions and platform `sin`/`cos` (`twin.rs:4-5`), so hashes are blessed per `os-arch` like the bench twins.
- **Second use:** the bench can later swap MOAB's `facets_to_h5m.cpp` for our writer only after a structural diff shows the two files equal on the same facets. Until then MOAB stays the reference path.
- **`smtets_to_moab`** (tet inputs for the XDG tet walk, `backlog.md:107`) is a separate task from the facet path. **Critique delta:** the XDG tet walk is itself a W4 deferral (`docs/design/w3c/w3c-report.md:132`). If the rt/bench area schedules it in W4, this trigger fires in W4. Coordinate with that area instead of parking it in W5+.
- **Region granularity.** The `.smrt` has one region per block. A DAGMC property import (`RegionInfo.tags`, `backlog.md:98`) is a producer that would trigger a cache-image version bump. Do not start it here.

### D3. Export options: a selection model shared by all formats
- **Today:** only Exodus can select blocks, and only through the library (`BlockSelection`; `sm-fem/src/export.rs:249` uses the default). The server validates per-format option objects for Gmsh and DAGMC only (`mesh_export.rs:39-94`); every other format rejects `options` (`host.rs:202-205`).
- **Design:** one `select` option object, accepted by every exporter that writes sets: `blocks`, `sidesets`, `nodesets` (names or `all`), `dimension` (`auto | 2 | 3`), `overwrite`.
  - It is validated once, in sm-server, for both dispatch paths (`mesh_export` and `sm_fem::export`).
  - **Critique delta:** apply it as a writer-level filter, not an overlay `MeshRead` view. Writers already choose which sets to write (Exodus `BlockSelection::Names` does exactly this). A filter keeps the selected cells in their original order and compacts nodes to those the selected cells use, in original order. That is O(selected), needs no frozen-crate change, and does not wait on the overlay contract. The overlay view (`backlog.md:12`) stays a separate performance item for `clone_host` (`sm-fem/src/lower.rs:128`).
  - **Critique delta:** `renumber` is dropped from the first cut (it was question 4). No solver in the user's stack has asked for it; RCM can be added later as an opt-in key. This is a planning call, not a user decision.
- **Better than Cubit:** Cubit's export flags are per format and positional. Ours is one named object, listed in the op catalog and checked against the exporter's loss report. A set that is requested but empty is a warning, not silence.
- **Hash impact:** none for existing exports (absent `select` reproduces current bytes).

### D4. Sizing: curvature, proximity and skeleton as one field family
The design target is a size field that never fails silently and is cheap to evaluate.
- **Curvature.**
  - On a curve or surface, the chord of length `h` on a circle of radius `R` has sagitta `s = R - sqrt(R^2 - h^2/4)`. Solving `s = t` gives `h(p) = min(h_max, 2 sqrt(t (2R - t)))`, which is about `sqrt(8 R t)` for `t << R`. **Critique delta:** use the exact form, as sm-tess does (`lib.rs:10-12`, `2 acos(1 - chord_tol / r)`); then the acceptance can be exact.
  - `R(p)` is the local radius from the exact curvature of the B-rep. **Critique delta (verified):** `CurveJet` carries `d1`, `d2` (`sm-geom-api/src/curve.rs:8-14`) and `SurfaceJet` carries `du, dv, duu, duv, dvv` (`surface.rs:9-21`), enough for curve curvature and surface principal curvatures. No geom-api change.
  - **Critique delta (evaluation):** `SizeField::size(p)` has no entity context (`sizing.rs:14-16`), and `SizeFieldFn` gets no geometry (`config.rs:15-16`). Design: (1) change `SizeFieldFn` to also take `&dyn TopologyView` and the region's targets (sm-mesher type, plus one line in sm-server `standard.rs:184`; neither is frozen); (2) at build time, sample every edge and face of the region with the exact jets at a fixed parameter grid and store `(point, h)` samples in an immutable kd-tree; (3) apply the grade limit on those samples; (4) `size(p)` is the graded value from the nearest samples, clamped to `h_max` far from any surface. Volume-interior points (the tet scheme queries them) get the graded value, which is what tet quality needs. This keeps the trait a pure function of the point (`sizing.rs:12-13`) and keeps `size(p)` O(log n).
  - The angle form `h <= R * angle` is a second cap, applied as a `min`. **Critique delta:** this answers the draft's question 6; sm-tess already takes both tolerances and uses the tighter one (`lib.rs:10-12`).
  - **Better than Cubit:** Cubit's curvature sizing works on the facetted approximation. Ours samples the exact analytic curvature and is checkable against a closed form.
- **Proximity (thin-gap) sizing.** `h <= gap / n`, from the medial-axis distance. A later step.
- **Skeleton sizing.** It targets thin features using a medial-axis-like skeleton. It is the most expensive item and the one with weak evidence of user need. Defer.
- **Grading.** One `grade` limit `g` applied as `h'(p) = min_q (h(q) + g |p - q|)` over the sample set. It keeps neighbouring sizes smooth and is the standard remedy for abrupt size jumps (Alauzet, "Size gradation control of anisotropic meshes", 2010). **Critique delta:** the all-pairs form is O(n^2) in samples. Use a Dijkstra-style relaxation over the kd-tree neighbour graph with ties broken by sample index, so the result is deterministic and the cost is O(n log n). Note that `geom::edge_size_goal` integrates `1/h` to 1e-10 by panel doubling (`geom.rs:68-77`); a field with kinks will run to its 2^14-panel cap, so measure the goal cost on a curved model.
- **`SizingSpec::Function` names added:** `curvature`, `grade` (wrapper), later `proximity`. The enum already carries free-form `name` and `params`, so `sm-scheme-api` does not change. Only `sm-sizing::size_field`, the `SizeFieldFn` type and its one injection site change.
- **Vertex sizing as a field.** **Critique delta (new):** Cubit's vertex sizing is a point source with a size that grades away. That is exactly a `point` size function (`points: [[x,y,z], h]`, wrapped by `grade`). Done that way, it needs no planner or intent change; the edge goals already come from the field (`geom.rs:68-77`).
- **Interval matching.** The integer program is the reason Cubit's interval counts are consistent. Keep it. **Critique delta:** the remaining work is smaller than drafted:
  1. **Relative intervals and sums.** The row kind and the solver exist (`IntervalConstraint::Linear`, `problem.rs:100-119`; `MeshIntent.intervals`, `intent.rs:485-489`). Add a user op in `sm-mesher/src/intent_ops.rs` (for example `mesh.interval_relation {edges_a, edges_b, ratio}`) that pushes the row.
  2. **Vertex sizing:** as a size function (above), not solver rows.
  3. **Periodic intervals.** **Critique delta:** this is not "needed for `--model` periodic faces". That W4 deferral is sm-cadio leaving pcurves unset on imported periodic faces (`w3c-report.md:131`), a CAD-import issue. Periodic intervals (a count in the periodic direction of a seamless surface) wait until such surfaces mesh at all.
  4. **Scheme-derived counts.** No ACR. Make `IiaSolver` honour `Origin::Preference` with the reference solver's precedence (soft, then preference, then default; `solve.rs:250-256`), then switch the curve schemes from `fix(.., Soft)` to `prefer` (`schemes.rs:173-175`). Close `backlog.md:9`.
  - The deviation objective and tie-break stay lexicographic. No producer calls `prefer` today, so the solver change moves no hash. The curve-scheme switch can move counts where a user soft count shares a class with a derived count; the stop rule applies.

### D5. GPU smoothing and quality: decide now, do not build now
- **Decision: do not build in W4.** Record it as W5+ with a trigger.
- **Reasons.**
  1. The binding rule is bit-identical output for any thread count. **Critique delta (stronger reason):** WGSL has no portable `f64`; Metal has no double precision at all. So a `wgpu` smoother on the user's Mac would run in `f32` and cannot give the CPU's bits. A GPU smoother would need its own `gpu` hash domain and its own acceptance (quality equal within a tolerance). That breaks the replay story (Q5, deterministic replay) unless the GPU path is opt-in and its result is recorded, not re-computed, in the timeline.
  2. **Critique delta:** no measurement shows smoothing or quality as a bottleneck. The measured 10^6-cell costs are bulk commit and delete (10.6 s versus 4.7 s, `backlog.md:11`; `mesh.delete` 4.5 s, `backlog.md:97`). Smoothing and quality at 10^6 cells have not been timed. The draft's "not in smoothing or quality" claimed more than the data shows.
  3. `wgpu` adds a large dependency and three driver matrices to CI. The platform hash work (W3c) already cost a wave.
- **Trigger to revisit:** a measured smoothing or quality pass above a threshold the user names (for example, more than 60 s at 10^7 cells on CPU), together with a rule for GPU determinism approved by the user.
- **Cheap prerequisite now (optional):** a benchmark of `sm-smooth` and `sm-quality` at 10^6 cells, so the trigger has a baseline. **Critique delta:** replaced "kernels are SoA (`kernel.rs:143`)"; that line is `use rayon::prelude::*;`, not evidence of layout.

### D6. CLI per-op diagnostics
- `render::script` walks `r.events`; for each `op` or `batch` event with a non-null `delta.diagnostics` it prints `severity: code: message` to stderr, using the same formatter as `render::apply` (`render.rs:77-87`, extracted to one function).
- Output order is script order. stdout is unchanged. Exit code is unchanged (a warning does not fail the run).
- `--json` stays the raw result. The script-level JSON `diagnostics:` line stays for parse and lowering diagnostics.
- **Better than Cubit:** Cubit prints warnings into a journal-like log interleaved with output. Ours separates them to stderr in a stable `severity: code: message` form that a CI log can grep.
- Backlog trigger was "first user report of a silent tet fallback" (`backlog.md:142`). The item is also a recorded W4 deferral (`w3c-report.md:135`) and in the "small, take with W4" list, so it is scheduled.

### D7. Test hygiene
One batch, split by owner. No new design. Each item is a test or tooling edit with an exact assertion. Ownership follows CLAUDE.md: items in a crate go to a worker that owns that crate, and `tools/`, CI and docs items go in one orchestrator commit.

## Tasks

Sizes: S under half a day, M about a day, L several days. All tasks follow CLAUDE.md (determinism test, rustdoc, clippy `-D warnings`). **Hash rule:** a moved existing hash stops the task and goes to the user. No worker blesses.

### EXPORT-SELECT (M)
- **Owner:** `crates/sm-meshio` (filter in the Gmsh, DAGMC and new writers), `crates/sm-exodus` (none: `BlockSelection` exists), `crates/sm-fem` (pass the selection into `export`/`ExportRequest`), `crates/sm-server` (validation for both dispatch paths, `host.rs:202-205`, `mesh_export.rs`). **Critique delta:** `options` is already a free-form object on `Request::Export` (`request.rs:231-234`), so no protocol field ACR. Its doc string (`request.rs:231-232`) lists per-format options and needs an orchestrator doc edit. Three crates means either one worker with explicit multi-crate ownership (as W3c-EXPORT-A did for sm-fem + sm-server) or a split.
- **Scope:** the `select` object of D3 for Exodus, Gmsh, DAGMC, VTU and the new exporters. OpenFOAM keeps its patches and zones. No `renumber`.
- **Acceptance:**
  - Export of two blocks out of three writes exactly the cells of those two blocks (count equal to the sum of block sizes), and exactly the nodes those cells use.
  - **Critique delta:** replaced "deleting an unselected block does not change the selected bytes" (not judgeable: a mesh delete can change ids). New: the selected export is byte-identical to the export of a test mesh built with only those cells and nodes, in the same relative order.
  - A sideset named but empty gives a `Warning` note with code `export.empty_selection`.
  - An unknown set name is `bad-request` that lists the available names.
  - Absent `select`: every existing export golden is unchanged, and `each_format_is_byte_identical_in_process_cli_and_mcp` passes unchanged.
  - Filtering allocates O(selected) beyond the lowered mesh (counting allocator, test code only).
- **Hash impact:** none (defaults reproduce current bytes).
- **Depends:** none. **Critique delta:** no longer depends on the overlay `MeshRead` contract.

### EXPORT-CGNS (L)
- **Owner:** `crates/sm-meshio/src/cgns/` (new module), `crates/sm-meshio/src/h5/` extensions and `CGNS_FORMAT.md`. Wiring in `crates/sm-server` (`mesh_export::handles`) and `crates/sm-cli` (`--format`, extension inference at `sm-cli/src/lib.rs:619-626`) is a second small step. The `ExportFormat` enum (`sm-protocol/src/request.rs:13-29`) is an orchestrator ACR. **Critique delta:** added the CLI wiring site.
- **Scope:** CGNS 4.x HDF5 file: `CGNSLibraryVersion`, `Base`, **one** unstructured `Zone`, `Elements_t` sections per block and standard type (TETRA_4, HEXA_8, PENTA_6, PYRA_5) and `NGON_n`/`NFACE_n` with `ElementStartOffset`, `BC_t` from face sets with `GridLocation FaceCenter` and `PointList`, `Family_t` per block, `ZoneSubRegion_t` for other cell sets. Extends the private `h5` writer with what CGNS needs (string attributes `label`/`name`/`type`/`flags`, the root ` hdf5version` and ` format` datasets).
- **Acceptance:**
  - `h5py` reads the file and the node tree matches the format spec (names, types, dimensions).
  - `cgnscheck` passes on a hex brick, a tet mesh, a mixed mesh and a polyhedral dual (gate `SM_CONFORMANCE=1`, after the orchestrator adds conda-forge `cgns` to the conformance env).
  - The CGNS library (`cgnslist` or pyCGNS) and ParaView's reader read it. Cell counts equal ours, and the cell volumes recomputed from the read file match ours to 1e-12 relative on all four fixtures. **Critique delta:** replaced meshio, which is not a known NGON/NFACE reader.
  - Signed `NFACE_n` references: the sum of signed face-area vectors of every cell is zero to 1e-12 relative (closed cell), the analytic check for orientation.
  - Output bytes are identical for 1 and N threads and across two runs.
  - A polyhedron of 5 or more faces round-trips through our own parser with equal face lists.
- **Hash impact:** new format, new hash domain. No existing hash moves.
- **Depends:** none. Prefer after EXPORT-SELECT so families and BCs follow the shared selection. **Critique delta:** first step is a spike: write a two-cell file with the extended `h5` writer and check that the CGNS library opens it (link creation order, see Risks). If it does not, the `h5` writer needs creation-order tracking, which grows the task.

### EXPORT-FLUENT (M) — **Critique delta: new task**
- **Owner:** `crates/sm-meshio/src/fluent/` and `FLUENT_FORMAT.md` (from `docs/research/fluent-msh.md`). Wiring as for CGNS; `ExportFormat` ACR.
- **Scope:** ASCII `.msh`: header, nodes, faces with c0/c1 (right-hand normal toward c0), cells with types (tet, hex, wedge, pyramid, polyhedron 7), zones from blocks and face sets. Never write the 58/59 sections (`fluent-msh.md:573`). No binary.
- **Acceptance:**
  - `fluent3DMeshToFoam` + `checkMesh -allGeometry -allTopology` in the existing OpenFOAM docker pass on a hex brick, a mixed mesh and a polyhedral dual (gate `SM_CONFORMANCE=1`).
  - The OpenFOAM mesh read back has our cell count and our cell volumes to 1e-12 relative.
  - A file with every face reversed is caught by the same check (negative test, as in `fluent-msh.md:15`).
  - Bytes identical for 1 and N threads and across two runs.
- **Hash impact:** new domain only.
- **Depends:** none. The Fluent licence check (`backlog.md:40`) stays open and is stated as a limit.

### EXPORT-ABAQUS (M)
- **Owner:** `crates/sm-meshio/src/abaqus/` and `ABAQUS_FORMAT.md`. Wiring in sm-server and sm-cli. `ExportFormat` ACR.
- **Scope:** `.inp` writer: `*HEADING`, `*NODE`, `*ELEMENT` for C3D4/C3D8/C3D6 (and the pyramid type only if confirmed; C3D10/C3D20 only when high-order cells exist), `*ELSET` per block, `*NSET`, element-based `*SURFACE` per sideset. Polyhedra follow `error | decompose | drop` with loss codes. Reals shortest round-trip.
- **Acceptance:**
  - Per-element-type node order table in the format doc, with one analytic fixture per type: the signed volume computed from the written connectivity is positive and equals the analytic volume (unit cube 1, unit wedge 0.5, unit tet 1/6, unit pyramid 1/3 if supported) to 1e-14.
  - Surface face numbering check: for each element type, the face named by `S1..S6` has outward normal equal to the expected axis in the unit fixtures.
  - meshio reads the file and equal cell counts per type result.
  - Decomposed polyhedra conserve volume to 1e-12 relative.
  - Determinism and loss-report tests like the Gmsh ones.
- **Hash impact:** new domain only.
- **Depends:** EXPORT-SELECT (soft). Share `PolyPolicy` with Gmsh. **Critique delta:** moving `PolyPolicy` to a common module must keep `sm_meshio::gmsh::PolyPolicy` as a re-export; `sm-server/src/mesh_export.rs` imports that path.

### EXPORT-DAGMC-MODEL (M/L)
- **Owner:** `crates/sm-meshio/src/dagmc` (only if `model_from_facets` needs changes) and `crates/sm-server` (the tessellation-to-`FacetInput` adapter and the request path; sm-server already depends on sm-tess, `Cargo.toml:31`). Which request carries it (a `source: "model"` choice on `session.export`, or a new query) is a protocol decision and an orchestrator ACR. **Critique delta:** the adapter is new production code (the only one today is the bench's `twin.rs`), so M/L, not M.
- **Scope:** export an h5m straight from B-rep facets via `sm_tess::tessellate` and `model_from_facets`, with names from the model's blocks (`mat:<name>`), a `chord_tol` option, the graveyard option and the same loss report.
- **Acceptance:**
  - h5py/MOAB read it, and the DAGMC conformance row (`SM_CONFORMANCE=1 tools/conformance.sh --only dagmc`) passes, including `check_watertight`.
  - **Critique delta (new, strongest):** on the bench's pin twin facets, our h5m and MOAB's `facets_to_h5m` output agree structurally (same triangle, surface and volume counts, same sense pairs, same group membership) under an h5py comparison.
  - Closed-surface check: every volume's signed triangle volume (computed in f64 from the stored f32 coordinates) equals the same sum over the input facets to 1e-12 relative. For an analytic cylinder fixture, the faceted volume lies below the analytic one by at most the side area times `chord_tol`.
  - **Critique delta:** replaced "10^4 rays give the same region sequences in the `.smrt` and DAGMC". The session `.smrt` is a mesh tracker, and rays that graze edges can legitimately differ. If a ray check is kept, it reuses the bench's agreement criterion with its bound (`sm-rt-bench/src/bound.rs:113`) and runs under `SM_CONFORMANCE=1` (DAGMC is docker-only).
  - Output bytes identical across runs and thread counts on one platform; hashes blessed per `os-arch`.
- **Hash impact:** new path. Existing h5m bytes from `model_from_mesh` must not change (`tests/dagmc.rs` goldens).
- **Depends:** none.

### EXPORT-EXO-VERIFY (S)
- **Owner:** `crates/sm-exodus` (tests and the `polyhedra` doc only, unless a bug is found). Any conformance script change is `tools/` (orchestrator).
- **Scope:** **Critique delta:** step 1 is concrete: run libMesh's `meshtool-opt` from the user's moose env on the NFACED fixture. If it reads the file as `C0Polyhedron` cells, that is the oracle; if it refuses, record that libMesh (hence MOOSE/Griffin through Exodus) cannot read our polyhedra. Step 2: SEACAS `exodiff`/`epu` if the user has them. The structural check exists (`check_exodus.py`).
- **Acceptance:** a polyhedral dual read by libMesh has equal face-per-cell counts and cell volumes to 1e-12 relative. If no real reader accepts NFACED, the "experimental" mark stays and the report says which reader refused and with what error.
- **Hash impact:** none.
- **Depends:** none.

### SIZE-CURV (M/L)
- **Owner:** `crates/sm-sizing` (`field.rs`, new `curvature.rs`, `grade.rs`), plus `crates/sm-mesher` (`SizeFieldFn` takes the topology view and targets, `config.rs:15-16`, `mesh_ops.rs:262-271`) and one line in `crates/sm-server/src/standard.rs:184`. **Critique delta:** the draft said sm-sizing only; the builder has no geometry today. No frozen crate changes.
- **Scope:** `curvature`, `point` and `grade` size functions (D4).
- **Acceptance:**
  - Circle of radius R with tolerance t: `size` equals `2 sqrt(t (2R - t))` to 1e-12 relative at 100 sample points of the circle (with grading off), and the meshed curve's maximum sagitta, measured on the exact circle, is at most `t (n_goal / n)^2` where `n` is the solved count. **Critique delta:** exact formula, and the bound accounts for the solver rounding the goal.
  - Cylinder and sphere: on sample points of the surface, the size equals the closed form from the smaller principal radius.
  - A plane gives `h_max` everywhere.
  - `grade g`: for every pair of samples, `h(p) <= h(q) + g |p - q|` (exact check over all pairs on a small fixture).
  - `point` source with grade: `size(x) = min(h_max, h0 + g |x - x0|)` exactly on a box with no curvature.
  - Field values are bit-identical for 1 and N threads and across two runs. Property test with `sm_testkit::strategies::config(n)`.
  - An invalid parameter (`tol <= 0`, `grade < 0`) is `SchemeError::BadParams` naming the key.
  - Edge-goal cost on a curved model is recorded (panel count per edge), so a kinked field that hits the 2^14 cap is visible.
- **Hash impact:** new names only. With no `curvature` spec, existing hashes are unchanged (explicit-scheme bit-identity, Q5). The `SizeFieldFn` signature change must move no hash.
- **Depends:** none.

### SIZE-INTERVALS (M) — **Critique delta: re-scoped, was L**
- **Owner:** `crates/sm-sizing/src/iia.rs` (honour `Origin::Preference`), `crates/sm-scheme-curve` (`fix(.., Soft)` to `prefer`), `crates/sm-mesher/src/intent_ops.rs` (relative-interval op). No ACR. Three crates: split into three S tasks or one worker with explicit ownership.
- **Scope:** relative intervals as a user op over existing `Linear` rows; preference handling in `IiaSolver`; curve schemes using `prefer`. Vertex sizing moves to SIZE-CURV (`point`). Periodic intervals are not scheduled.
- **Acceptance:**
  - Relative intervals with ratio 2: the counts satisfy `n_a = 2 n_b` exactly, and the op is recorded and replayed with the same hash.
  - `IiaSolver` and `ReferenceIntervalSolver` give equal counts on a property-test family that mixes soft, preference and default goals (`sm_testkit::strategies::config(n)`).
  - A `bias` curve with a derived count of 6 next to a model default size that alone would give 2 yields 6; a user soft count on the same class still wins.
  - Existing property tests (`deviation_key`, `compare_deviation_sums`) pass unchanged; shuffled constraint lists give identical counts.
- **Hash impact:** the solver change moves none (no producer of `prefer` today). The curve-scheme switch may move counts where a user soft count shares a class with a derived count: run the hash census; any move stops the task.
- **Depends:** none.

### GPU-DECIDE (S, orchestrator)
- **Owner:** orchestrator (docs only).
- **Scope:** record decision D5 in the W4 plan and in `docs/PARITY.md`. Move the row to W5+ with the trigger.
- **Acceptance:** one paragraph, with the trigger, the determinism rule and the `f64` limit of WGSL/Metal.
- **Hash impact:** none.
- **Depends:** the user's answer to question 4 below.

### CLI-DIAG (S)
- **Owner:** `crates/sm-cli` (`render.rs`). MCP and Python already return events and need no change.
- **Scope:** D6.
- **Acceptance:**
  - A script that meshes an enclosed pin prints one `warning: mesh.auto_tet: ...` line per tet-fallback volume on stderr, and stdout equals the previous stdout byte for byte.
  - A clean script prints no extra lines.
  - Batch events with deltas print too, in script order.
  - `--json` output is unchanged byte for byte.
  - Exit code is 0 on warnings, non-zero only on error as today.
  - The text-mode lines equal the `delta.diagnostics` entries of the JSON result (cross-check test).
- **Hash impact:** none (render only).
- **Depends:** none.

### HYGIENE-* (S each, one task per owning crate)
- **HYGIENE-CLI** (`crates/sm-cli`): Drop guard for per-pid temp dirs; the `milestone_w3c_pin.rs` items (fail instead of skip under `CI=true`, per-block fuel volume against the polygonal-cylinder area times H, a no-blocks fixture, `enumerate` lookup); `cargo fmt` of `tests/platform_pins.rs`. Acceptance: the pin test fails with `CI=true` and no maturin; the fuel volume equals `(n/2) r^2 sin(2 pi / n) H` to 1e-9 relative; `target/tmp` holds no leftover dirs after a run. **Critique delta:** `platform_pins.rs` moved here from HYGIENE-TOOLS; dropped "move to `CARGO_TARGET_TMPDIR`" (done, `backlog.md:126`).
- **HYGIENE-SERVER** (`crates/sm-server`): distinct `copy_of` values in the through-pin plan rows; move `temp_dir()` uses to `CARGO_TARGET_TMPDIR`; the `BadOpName` test timeout (`standard.rs:282-284`) gets a test-local error or a comment, since a new `RegistryError` variant is an `sm-op-api` ACR. Acceptance: the assertion fails if two rows share a `copy_of`.
- **HYGIENE-RTBENCH** (`crates/sm-rt-bench`): `sha256:` image id check before the platform check, `image_id` asserted in `absorber_unbiased`, exact lost count in `flight_step_into_an_unmapped_region_is_lost_not_a_leak`. Acceptance: the lost count equals the number of rays that meet the unmapped sphere, derived analytically from the fixture.
- **HYGIENE-CSGIO** (`crates/sm-csgio`): `gen.py` `openmc.reset_auto_ids()`, exit 143 on SIGTERM. Acceptance: a subset run regenerates `geometry.xml` byte for byte equal to the full run.
- **HYGIENE-MESHER** (`crates/sm-mesher`): `known`/`plan_with` purity note, and an optional call-count test.
- **HYGIENE-PY** (`crates/sm-py`): `ci/py.sh --basetemp`.
- **HYGIENE-RTCAPI** (`crates/sm-rt-capi`): wrap `DESIGN.md:111` to the doc width. **Critique delta:** moved out of HYGIENE-TOOLS; it is in a crate.
- **HYGIENE-TOOLS** (orchestrator commit): `tools/platform/collect.py:58`, macOS `rustc -V`, mypy/ruff for `tools/w3c_driver.py`; mark `backlog.md:9,13,24` done and `PARITY.md:59` partly done (doc fixes from this critique); fix the Exodus "NetCDF-4" doc in `sm-protocol/src/request.rs:18`.
- **Hash impact for all:** none. A test-only change that moves a hash is a bug in the change.
- **Depends:** none. Trigger "next edit of that file" means these can ride along with the tasks that edit the same files (for example HYGIENE-CLI with CLI-DIAG).

### Task dependency summary
- Independent: CLI-DIAG, EXPORT-EXO-VERIFY, EXPORT-DAGMC-MODEL, EXPORT-FLUENT, SIZE-CURV, SIZE-INTERVALS, HYGIENE-*, GPU-DECIDE.
- EXPORT-SELECT before EXPORT-CGNS and EXPORT-ABAQUS (soft).
- **Critique delta:** no task waits on an `sm-scheme-api` ACR any more.
- Orchestrator ACRs: `ExportFormat` additions (CGNS, Fluent, Abaqus) and the DAGMC model-source request. Doc-only: `request.rs:18` and `request.rs:231-232`.

## Priority (for the user's goals: polyhedral meshes for the transport code, and ray tracing beating DAGMC/XDG)

**Now (first W4 sub-wave):**
- EXPORT-DAGMC-MODEL. **Critique delta:** it is not an input the benchmark lacks; it is the user-facing "STEP to DAGMC without Cubit" path for OpenMC users, which is a direct surpass item.
- EXPORT-EXO-VERIFY. It decides, at small cost, whether Exodus is a real polyhedral route to MOOSE/libMesh at all.
- CLI-DIAG. It is small, scheduled as a W4 deferral, and removes a silent tet fallback.
- SIZE-INTERVALS. **Critique delta:** moved up; it is now small and closes a stale contract item.
- GPU-DECIDE. It costs one paragraph and prevents drift.
- HYGIENE-*. They are cheap and ride along with the tasks that touch the same files.

**Later W4 sub-wave:**
- EXPORT-SELECT, then EXPORT-FLUENT and EXPORT-CGNS. **Critique delta:** Fluent before CGNS: it has a user ruling, a checked spec and an oracle that already runs; CGNS needs a reader installed and an HDF5 spike. Both matter once polyhedral output (tet dual, W4's polyhedron scheme) exists in volume.
- SIZE-CURV. It makes unstructured meshes of curved models fit a tolerance. Schedule it with the polyhedral meshing task (the tet dual inherits the tet sizes).
- EXPORT-ABAQUS. Useful, not on the critical path of either goal.

**W5+ or on request:**
- Nastran and the P2 formats (Patran, UNV, LS-DYNA, Sierra, VRML). No reader in the user's stack needs them.
- Skeleton and proximity sizing. Medial-axis cost, weak evidence of need.
- Periodic intervals.
- GPU smoothing and quality (D5).
- Per-volume `.smrt` region tags and DAGMC property import, tied to the first request.
- **Critique delta:** `smtets_to_moab` is not here: it follows the XDG tet walk, which is a W4 deferral owned by the rt/bench area.

## Risks
- **Abaqus conformance without Abaqus.** We have no Abaqus licence. Element face numbering and node order are checked against the documented tables and meshio only. A wrong `*SURFACE` face number would go unseen. Mitigation: analytic outward-normal checks per type; state the limit in the format doc. Same stance as the Fluent gap (`backlog.md:40`).
- **CGNS scope creep.** CGNS is large. Restrict to the unstructured subset above. PETSc rejects NGON/NFACE (`mesh-ir-synthesis.md:14`), so "reads in every tool" is not a goal. **Critique delta:** the PETSc row is line 14, not 12.
- **Hand-written HDF5.** The h5 writer is a private subset: compact links sorted by name and no creation-order tracking (`h5/mod.rs:4-7`). **Critique delta:** CGNS node order (sections, zones) is semantically meaningful and the CGNS library may rely on link creation order. If it does, the writer needs creation-order tracking and indices. Mitigation: the two-cell spike first, with the CGNS library as the gate, not h5py.
- **Selection semantics.** Node compaction in original order must be stated in the format docs, or users will compare ids across exports and be surprised.
- **Curvature fields and the all-hex schemes.** A curvature-driven interval count on a curve may disagree with sweep source/target matching. The integer program resolves it, but it can move counts of models that already carry sizing. The new field only applies when requested, so default hashes do not move. A model that now uses it reports changed counts.
- **Edge-goal cost.** A kinked graded field slows the adaptive edge quadrature to its cap (`geom.rs:68-77`). Measure on a curved model before relying on it.
- **Facet bytes are per platform.** DAGMC model export inherits sm-tess `f32` and platform `sin`/`cos`; its goldens go into the per-`os-arch` lists, not the universal ones.
- **GPU decision may be revisited.** If a 10^7-cell model arrives, the CPU path may not be fast enough. The trigger in D5 handles that.
- **Hygiene sprawl.** The items cross many crates and `tools/`. Without one owner per item, a worker will touch a crate it does not own. Hence the per-crate split.
- ~~**Overlay view performance.**~~ **Critique delta:** no longer a risk of this area; selection does not use the view.
- ~~**Renumbering changes bytes.**~~ **Critique delta:** dropped with `renumber`.

## Questions for the user (real decisions only)

1. **What file does your transport code read for polyhedral meshes: `.smsh`, Exodus, CGNS, Fluent `.msh`, OpenFOAM, or something else?**
   - This decides the export order. If it reads `.smsh` or OpenFOAM, the new exporters are interchange only and can slip a sub-wave with no loss.
   - Recommendation otherwise: Fluent `.msh` (your ASCII-first ruling, spec and oracle ready), then CGNS, then Abaqus; no Nastran until a reader needs it.
   - **Critique delta:** reframed around the reader; added Fluent, which the draft left out despite the 2026-10-25 ruling.
2. **Should DAGMC h5m be exportable straight from the B-rep, with no volume mesh (EXPORT-DAGMC-MODEL)?**
   - Recommendation: yes. It gives OpenMC users STEP to DAGMC without Cubit or MOAB, one command, every volume checked closed. **Critique delta:** the benchmark comparison is already geometry-identical, so that is not the reason.
   - Request shape: a `source: "model"` choice on `session.export` (recommended, one verb for all exports) or a separate query.
3. **Do you have SEACAS (`exodiff`, `epu`) anywhere, or a Griffin build?** **Critique delta:** narrowed. Your moose env already has libMesh 1.9.0-pre with `meshtool-opt`, which EXPORT-EXO-VERIFY uses first.
4. **GPU smoothing and quality: confirm "not in W4", with the trigger in D5 (a measured pass above a threshold you name, plus an approved GPU determinism rule)?**
   - Recommendation: yes. The bit-identical rule cannot hold on Metal (no `f64`), and smoothing is not yet measured as a bottleneck.

**Critique delta:** removed three questions that are not user decisions. Renumbering (old 4) is dropped from the first cut by planning. Sagitta versus angle (old 6) is answered in D4: sagitta primary, angle as a cap, as sm-tess already does. Hygiene ownership (old 7) is fixed by CLAUDE.md.
