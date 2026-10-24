# W2 plan: first vertical slice

Base: `origin/main` at `30f5509` (W1 merged plus the W1 ACR batch). Every signature quoted below was checked against that tree.
An experiment in a scratch worktree (a throwaway probe crate, nothing committed) confirmed:
- `dyn TopologyEdit` is object safe.
- `TopoStore` and `CellComplex` are both `Clone + Send + Sync + 'static`.
- `sm_mesh::Mesh` and `ReferenceMesh` are `Clone + MeshRead`, and both implement `MeshEdit`.
- `dyn Plugin` is object safe.
- `sm_scheme_api::SchemeRegistry` is `Any + Send + Sync`, so it can be a `Services` entry.

Milestone (PLAN.md W2): `brick → size → mesh (map) → block → export .smsh/.vtu`. It must run from the CLI, from MCP and from a timeline fork, and all three must give matching state hashes.

---

## 1. Gap analysis

### 1.1 What main has today

| Piece | Where | State |
|---|---|---|
| Op/timeline contract, `State`, `LayerType`, `Registry`, `Session` | `sm-op-api` | frozen, complete |
| Project on disk, `Workspace` facade (`apply_json`, `query`, `fork(name, at)`, `checkout`, `undo/redo`, `status`, `state_hash`, `services`, `clock`, `author`) | `sm-engine` | done |
| Command language, `Host` trait (`apply`, `query`, `timeline`, `timeline_head`, `relations`, `load`), `EngineHost`, `PlanHost`, `QueryRelations("topo.related")` | `sm-lang` | done; **no Host over `Session`/`Workspace`** |
| Topology + geometry store (`TopologyEdit`, append-only `GeomAppend` via `shapes_mut`) | `sm-topo` (`TopoStore`), reference `CellComplex` | done; brick/cylinder/sphere exist only as **fixtures** (`sm_topo_api::fixtures::{brick, cylinder, sphere}`, `brick` takes a min corner, not a centre) |
| Analytic + NURBS geometry, `CurveDef::transformed` / `SurfaceDef::transformed` | `sm-geom-api`, `sm-geom` | done (so transforms need no new contract) |
| Model semantics: colors, groups, boundaries/decks, symmetry, instancing, decomposition | `sm_model_api::Model` (a **concrete** struct of tables, not a trait) | done, already production quality |
| Meshing driver: plan → solve → schedule → generate → commit (`Driver::mesh(input, intent, exec, builder, alloc)`), `ReferenceIntervalSolver`, `ReferenceCornerPicker`, `WholeFaces`/`DecompositionFaces`, `Sequential`/`ScopedThreads` | `sm-scheme-api` | done, and **the driver is production code** (pure, executor injected) |
| Stub schemes (`equal`, `map`, `submap`, `sweep`, …), `brick_setup`, `row_setup`, `registry()` | `sm-scheme-api/tests/stub_schemes/mod.rs` | **reachable only from that crate's integration tests** |
| Mesh store and builder (`MeshBuilder`, `Mesh: MeshRead + MeshEdit`), `Catalog`, `generate::hex_block` | `sm-mesh` | done |
| `.smsh` writer (`SmshWriter::new(&dyn MeshRead).provenance(..).deck(DeckRecord)`), `write_vtu(&dyn MeshRead, ..)` | `sm-meshio` | done; **no lowering from the model to mesh sets** (meshio request 4) |
| Quality metrics | `sm-quality` | done |

### 1.2 What is missing for the milestone

1. **No production `LayerType` and no production `Plugin` exist.** `grep "LayerType for|Plugin for"` finds only test code (sm-engine, sm-lang, sm-testkit). `sm_op_api::layer::names` reserves `model`, `partition`, `intent` and `meshes`, but nothing implements them. So today nobody stores topology or meshes in `State`, and nobody registers `model.create_brick`, `mesh.size`, `mesh.run`, `fem.block_add` or `topo.related`.
2. **No real schemes.** Missing: curve schemes (equal, bias, dualbias, stretch, curvature), surface map and volume map. The `mapvol` stub is plan-level only; its `generate` returns `Unsupported`.
3. **No production interval matching, corner picker, size fields or auto size.** Only the greedy reference solver and the reference picker exist.
4. **No FE layer**, i.e. no blocks with element types. There is also no lowering from blocks to cell sets, from colored faces to side sets, or from decks to named face sets plus `DeckRecord`.
5. **No front ends.** Missing: a Host over the session, a protocol, a server, MCP and the `smesh` binary. The `session.*` script namespace (export, save, hash) does not exist.
6. **No assembly** of a standard distribution: plugins, plus a `SchemeRegistry` with fallback and auto lists, injected as a service.

### 1.3 Who registers what (decision)

| Op / query | Registered by | Layer it writes | Reads |
|---|---|---|---|
| `model.create_brick` (+ cylinder, sphere, …), `model.move/rotate/scale/reflect`, `model.delete`, queries `topo.related`, `model.list`, `model.measure` | **sm-model** (`ModelPlugin`) | `model` | |
| `mesh.size`, `mesh.intervals`, `mesh.scheme`, `mesh.run`, `mesh.delete`, query `mesh.info` | **sm-mesher** (`MeshPlugin`) | `intent` (own type, private to sm-mesher), `meshes` | `model`, service `MesherConfig` |
| `fem.block_add`, `fem.block_remove`, `fem.block_element_type`, `fem.sideset_add` | **sm-fem** (`FemPlugin`) | `model` (blocks are volume **colors**, sidesets are face colors; this follows ir.md §5, where coloring includes blocks), `fem` (element type per block color) | `meshes` (export only) |
| `view.tessellate` (query) | **sm-tess** (`TessPlugin`) | | `model` |
| `export` | **not an op.** Export writes a file and changes no state (ARCHITECTURE §7 lists `export` next to `apply_op`). It is the protocol method `session.export` and the script command `session.export(...)`. The implementation is `sm_fem::export(&State, &ExportRequest, &mut dyn Write)`. | none | `model`, `meshes`, `fem` |
| `model` and `meshes` layer registration | **`sm_layer_api::CoreLayers`** (new contract crate, ACR-3). One plugin registers both shared layers, so a layer is never registered twice (`DuplicateLayer`) and every crate can build test states without its siblings. | | |

### 1.4 Who runs the vertex → curve → surface → volume hierarchy and writes the mesh

**The `mesh.run` op in sm-mesher**, using the contract's `Driver`. sm-mesher never names a scheme crate (concepts.yaml: `scheme-* → mesh-driver`). The body of `mesh.run`:

```
// ApplyCtx borrows are exclusive: clone the read layers first (Arc clones, cheap), then take layer_mut.
cfg    = ctx.service::<MesherConfig>()?                 // SchemeRegistry + solver + picker + size-field factory
model  = ctx.layer::<ModelLayer>()?.clone()             // Arc<dyn TopoHost> + Arc<Model>
intent_layer = ctx.layer::<IntentLayer>()?.clone()
alloc0 = ctx.layer::<MeshLayer>()?.alloc().clone()
input = MeshInput::new(model.topo(), model.model(), &WholeFaces, &targets)   // W2: whole volumes only
drv   = Driver { registry: &cfg.registry, solver: &*cfg.solver, picker: &*cfg.picker,
                 size: field.as_deref(), options: DriverOptions::new() }
alloc = alloc0.clone()                                  // work on a copy; commit only on Ok
out   = drv.mesh(&input, &intent_layer.intent_for(&targets), &RayonExec, MeshBuilder::new(), &mut alloc)?
ctx.adopt_as("mesh", &[mesh_id]); ctx.adopt_range(..) for MeshVertex, MeshFace, MeshCell, MeshSet, MeshEdge
                                                        // same order `finish` draws them (Mesh first)
ctx.layer_mut::<MeshLayer>()?.insert(mesh_id, MeshEntry::new(Box::new(out.finished.mesh), targets, Some(out.plan), model.hash_quanta()), alloc)?
```

- Classification comes from the driver's `commit` through `WholeFaces::classify`. Nodes go on vertices, edges, faces and volumes; boundary faces go on `TopoFace`; cells go on `TopoVolume`.
- The interval count for each edge comes from the plan (`GenCtx::intervals`).
- Atomicity: the driver builds a fresh builder and `finish` leaves the allocator unchanged on error, so a failed `mesh.run` adopts nothing and the state stays byte-identical.

### 1.5 How the mesh layer is stored in `State`

`meshes` = `sm_layer_api::MeshLayer` (ACR-3):
- Fields: `BTreeMap<EntityId(Mesh), MeshEntry>` plus the shared `SerialAlloc`.
- `MeshEntry { mesh: Arc<dyn MeshHost>, selection: Vec<EntityId>, plan: Option<Arc<Plan>>, quanta: HashQuanta, stale: bool }`.
- Copy-on-write: cloning the layer copies only `Arc`s.
- Digest: per entry, `id`, `selection`, `stale`, `topology_hash(mesh)` (an exact integer hash), then every point coordinate through `Digest::f64_quantized(x, quanta.length)` and every `Param` value through `Digest::f64_quantized(v, quanta.param)`. Quanta follow W0c Q2: `linear·2⁻¹⁰` and `parametric·2⁻¹⁰` of the model tolerance at insert time. This is platform-independent in identity mode and bit-exact in cache-key mode.
- The `plan` is kept for provenance but **not hashed**. It holds computed floats (lengths, angles), and it is a deterministic function of hashed content anyway.
- Snapshots: no snapshot hook in W2 (replay). That is a W3 item.
- `on_retired`: marks the entries whose selection names a retired entity as `stale`. It does not delete them, because their `Mesh*` ids would stay live in the entity table. `mesh.delete` is the op that retires mesh ids.

`model` = `sm_layer_api::ModelLayer`:
- Fields: `Arc<dyn TopoHost>` (`TopoStore` in production, `CellComplex` in tests), `Arc<Model>`, and `next_op: u64`.
- The topology `OpId` for `begin_op` comes from `next_op`. It never comes from node depth or from time.
- Digest: the `TopologyView::canonical_visit` stream, with `HashWord::Int → Digest::u64` and `HashWord::Float{raw,q} → Digest::f64_with(raw, q)`, then `Model::canonical_visit(tol, ..)` with the same mapping, then `next_op`.
- Copy-on-write clones the whole `TopoStore` on each model op. That is fine for W2 and is a known risk for W3 (see §5).

`intent` = `sm_mesher::IntentLayer { intent: MeshIntent }`, which `DEPENDS_ON [model]`.
- Digest: `d.value(&intent)`, i.e. canonical JSON. Its floats are user input and therefore bit-identical everywhere.
- `on_retired` drops the requests on retired entities.

`fem` = `sm_fem::FemLayer { element_types: BTreeMap<EntityId(Color), ElementType> }`, which `DEPENDS_ON [model]`.

Not needed in W2: the `partition` layer. A brick is `PieceRef::Whole`, resolved by `WholeFaces`.

### 1.6 Reference implementations: production crates or not

- **`sm_model_api::Model`: use as is.** It is a concrete, persisted table store with lineage re-keying. W2 needs no `sm-model-semantics` crate.
- **`Driver`: use as is.** sm-mesher only adds a rayon `Executor`, the service lookup and the layers.
- **`ReferenceIntervalSolver` / `ReferenceCornerPicker`: defaults and oracles.** sm-sizing provides the production versions (optimal integer assignment, Cubit-style picking), injected through `MesherConfig`. Until sm-sizing merges, the references run.
- **`CellComplex`, `ReferenceBuilder`: oracles only.** Production uses `TopoStore` and `sm_mesh::MeshBuilder`.

---

## 2. Task list (12 parallel tasks, 1 micro task, plus the integration phase)

Layers used here, counted from the bottom (a crate may depend only on lower layers): C = contracts; L0 sm-math; L1 sm-geom, sm-topo, sm-mesh; L2 sm-model, sm-tess, sm-sizing, sm-meshio; L3 scheme crates, sm-mesher, sm-quality, sm-fem; L4 sm-engine, sm-lang; L5 sm-server; L6 sm-mcp; L7 sm-cli.

Rules common to every task:
- Follow CLAUDE.md: ownership of one crate directory only, the determinism rules, no `unwrap` in library code, `thiserror` errors, rustdoc on every public item, clippy `-D warnings`.
- **Keep the skeleton entry points** (ACR-5) with exactly the given signatures.
- Every op uses `#[schemars(extend("x-entity-kind" = [...]))]` on its id fields and declares `OUTPUTS` (ACR-2).
- The determinism test means:
  - `sm_testkit::determinism::check` comparing state, layer and mesh hashes over two runs, `RAYON_NUM_THREADS=1` against the default, and concurrent runs;
  - plus a cold replay from the root, which must give no `Diverged`. Crates below L4 use `sm_op_api::Session::state_at_from_root`; L5 and above may use `sm_engine::Workspace::memory()`.

### T1. `sm-model` (L2, new): primitives, transforms and topology queries
- **Deps:** sm-core, sm-math, sm-geom-api, sm-geom, sm-topo-api, sm-topo, sm-model-api, sm-op-api, sm-layer-api. Dev: sm-testkit (cold replay through `sm_op_api::{Engine, Session}`, which is a contract and lower; no dev-dependency on sm-engine, which is L4).
- **Scope:**
  - `ModelPlugin`, which registers ops and queries only. It does **not** register `ModelLayer`; `CoreLayers` does that.
  - Ops:
    - `model.create_brick {x, y?, z?, center?: [f64;3]}`, centred like Cubit; `y` and `z` default to `x`. Outputs `body`(1), `volume`(1), `faces`(6), `edges`(12), `vertices`(8).
    - `model.create_cylinder {height, radius}` and `model.create_sphere {radius}`, ported from the `sm_topo_api::fixtures` constructions.
    - Stretch goals: `model.create_frustum`, `model.create_torus`, `model.create_prism`, `model.create_pyramid`.
    - `model.move {entities, delta}`, `model.rotate {entities, axis, angle_deg, origin?}`, `model.scale {entities, factor, origin?}`, `model.reflect {entities, normal, origin?}`. These use `set_vertex_point` + `replace_curve/replace_surface` with `CurveDef/SurfaceDef::transformed`, plus `reverse_face` for improper transforms, then `Model::apply_transform` to keep decks consistent.
    - `model.delete {bodies}`, via `kill_cascade`; it retires ids.
  - Queries:
    - `topo.related {kind, within} -> [EntityId]`, exactly the shape sm-lang's `QueryRelations` calls: either hierarchy direction, sorted, de-duplicated.
    - `model.list {kind}`.
    - `model.measure {entity}`: length, area, volume and bounding box through `face_moments`.
  - All topology edits go through `sm_layer_api::topo_txn` / `commit_topology` (ACR-3).
- **Acceptance:**
  - `create_brick(10)`: V/E/F = 8/12/6, vertices exactly at ±5 (bit-exact), volume 1000 from moments to 1e-12, `validate` clean, Euler = 2.
  - The brick's canonical hash equals the hash of the `CellComplex` fixture `brick(min = (-5,-5,-5), size 10)`. This is a differential test between the production and reference hosts.
  - `Delta.created` lists exactly the 28 topology ids in ascending order, and `outputs` lists exactly the declared roles.
  - Cylinder volume πr²h and sphere volume 4/3·πr³ match within the quadrature tolerance.
  - Move by (1, 2, 3): every vertex is translated bit-exactly for representable values.
  - Rotate by 90° about z: vertex set within 1e-12, outward normals kept.
  - Reflect: volume stays positive, every face normal points outward, `validate` clean, and a mirror deck declared beforehand is still consistent (`sm_model_api::validate`).
  - `model.delete`: ids are retired, a later `topo.related` does not list them, and the `MeshLayer` entries on the deleted volume become `stale`.
  - `topo.related` matches brute force on every `sm_testkit::topo::all_topo_fixtures()` case (volume→faces, face→volumes, body→volumes, edge→faces, vertex→edges).
  - A transform of a meshed volume is refused with `OpError::Invalid` ("mesh.delete first"). It is checked through `ctx.layer::<MeshLayer>()`, and a missing layer is tolerated.
- **Must NOT:** register layers; write the `meshes` or `intent` layers; implement booleans, webcut or imprint (W3); use node depth or time as the `OpId`.

### T2. `sm-tess` (L2, new): B-rep tessellation for display
- **Deps:** sm-core, sm-math, sm-geom-api, sm-geom, sm-topo-api, sm-op-api, sm-layer-api, rayon.
- **Scope:**
  - `tessellate(view: &dyn TopologyView, faces: &[EntityId], opts: &TessOptions) -> Result<Tessellation>`: f32 positions, u32 indices, per-face index ranges, and per-edge polylines shared by adjacent faces. Controlled by chord, angle and max-edge tolerances.
  - `TessPlugin` registers the query `view.tessellate {entities, chord_tol?, angle_tol?}`, which returns base64 buffers and ranges.
- **Acceptance:**
  - Brick: 12 triangles, total area exactly 600.
  - Cylinder lateral area 2πrh within the chord-error bound. The bound should be derived and asserted, e.g. area error ≤ 2πh·(r - r·cos(θ/2)) for the chosen θ.
  - Sphere area within the bound.
  - Watertight: every edge's vertices are shared bit-exactly by both adjacent faces.
  - Each triangle normal agrees in sign with the face normal.
  - 1 thread and N threads give byte-identical buffers.
- **Must NOT:** mutate state or register ops; produce mesh-layer entities. This task is not on the milestone path and can be dropped if the wave is late.

### T3. `sm-sizing` (L2, new): interval matching, corner picking, size fields
- **Deps:** sm-core, sm-math, sm-geom-api, sm-topo-api, sm-scheme-api. Dev: sm-testkit, sm-scheme-api[stubs].
- **Scope:**
  - `IiaSolver: IntervalSolver`. It is optimal on the integer program over `IntervalConstraint::{Fixed, Goal, Range, Linear, EvenSum}`. The objective is lexicographic: first max relative deviation `|n−g|/g`, then the sum of relative deviations, then the lexicographically smallest count vector in ascending edge order (a deterministic tie-break).
    - It can decompose into independent components (the union-find classes) and solve them in parallel with a fixed merge order.
    - Beyond a documented size it falls back to `ReferenceIntervalSolver`. If the fallback still finds no feasible solution, it returns `Infeasible` with the involved constraints and entities.
  - `AnglePicker: CornerPicker`, Cubit-style: End ≤ 135°, Side < 225°, Corner < 315°, Reversal above that. It changes the fewest types needed to meet the rule sums for `Map` (4 Ends) and `Submap` (Σ = 4); hard `CornerIntent`s win.
  - Size fields: `Constant`, `Linear {p0, h0, p1, h1}` and `Bias`/`Geometric` along an axis, through `pub fn size_field(spec: &SizingSpec) -> Result<Option<Box<dyn SizeField>>, SchemeError>`. It handles `SizingSpec::Function{name,..}` and falls back to `sm_scheme_api::sizing::field_for` for `Constant`.
  - `pub fn auto_size(view: &dyn TopologyView, targets: &[EntityId], factor: u32) -> Result<f64, SizingError>`: a deterministic, documented formula, monotone in `factor` (1..=10, Cubit range).
- **Acceptance:**
  - A 4-sided face with goals (3, 5.2, 3.4, 5) gives the opposite-equal solution (3|3, 5|5).
  - A brick with size 1 and edge 10 gives every count = 10.
  - `row_setup(3)` with mixed hard counts gives the unique feasible answer.
  - Conflicting hard counts give `Infeasible` naming both rows.
  - Even-sum rows with odd goals give the minimal parity repair.
  - **Optimality against brute force:** a seeded proptest with ≤ 6 variables, counts ≤ 8 and random rows; the objective must equal exhaustive enumeration.
  - Agreement with `ReferenceIntervalSolver` on every structured row in the scheme-api stub scenarios.
  - Picker on a rectangle with a mid-edge vertex gives types `End×4, Side`. An L-shaped face under `Submap` gives the correct sum with one Corner. An infeasible request gives an error naming the request.
  - Size-field integral `h = 0.01 + x` on a unit edge gives `ln 101` (reusing `geom::edge_size_goal`).
  - `auto_size` is monotone and gives an exact value on a unit cube for the documented factor.
  - Determinism: the same answer for 1 and N threads and for permuted component order.
- **Must NOT:** register ops or layers; depend on scheme crates or on sm-mesher; use floating-point LP results without exact integer verification of feasibility.

### T4. `sm-scheme-curve` (L3, new): equal, bias, dualbias, stretch, curvature
- **Deps:** sm-core, sm-math, sm-geom-api, sm-topo-api, sm-scheme-api. Dev: sm-testkit, sm-scheme-api[stubs], sm-mesh.
- **Scope:**
  - Five `Scheme`s, `dim = Curve`, built with `GenCtx::curve_patch(edge, param_fractions)`.
  - Arc-length spacing is converted to parameter fractions inside the scheme (adaptive Gauss-Legendre plus a monotone inverse), because `curve_patch` takes **parameter** fractions.
  - Params are typed serde structs inside `SchemeChoice::params`:
    - bias: `{ratio}` or `{first_size}` or `{first_size, last_size}`, with `start?: TopoVertex`;
    - dualbias: `{ratio}` or `{first_size}`;
    - stretch: `{first_size, last_size?, max_ratio?}`;
    - curvature: `{factor}`.
  - `pub fn register(reg: &mut SchemeRegistry) -> sm_scheme_api::Result<()>`.
- **Acceptance (analytic):**
  - equal: on a line, nodes at `i·L/n` within 1 ulp·L; on a circular arc, equal arc lengths within 1e-12·L; on a NURBS curve, equal arc lengths within 1e-9 relative.
  - bias ratio r: consecutive segment ratios = r within 1e-12 and total length exact. `first_size` mode recovers r by Newton (closure checked). `start` on the far vertex reverses the sequence.
  - dualbias: segment lengths are mirror-symmetric within 1e-12.
  - stretch: the first segment equals `first_size` within 1e-12 and the ratio never exceeds `max_ratio`.
  - curvature: on a line and on a circle it reduces to equal; on an ellipse the segments are shorter where curvature is high, and monotone.
  - An end-to-end brick through `Driver::mesh` with the stub map gives classified nodes on the edges.
  - Invalid params give `NotApplicable` with a reason, never a panic.
- **Must NOT:** compute node keys or positions itself (`curve_patch` owns that); register auto lists (the composer does); depend on other W2 crates.

### T5. `sm-scheme-map` (L3, new): surface map
- **Deps:** sm-core, sm-math, sm-geom-api, sm-topo-api, sm-scheme-api. Dev: sm-testkit, sm-scheme-api[stubs], sm-mesh, sm-quality.
- **Scope:**
  - `Scheme` "map", `dim = Surface`, caps `QUAD | STRUCTURED | BLOCK_STRUCTURED`.
  - `requirements`: `vertex_rule(f, VertexRule::Map)`.
  - `interval_constraints`: sides 0=2 and 1=3 from `FaceLayout` (several coedges per logical side are summed).
  - `generate`: transfinite interpolation (Coons) in the face's parameter space (uv from coedge pcurves or `snap_to_face`), evaluated on the surface, emitting `Patch::grid` (`NodeGrid` ni×nj) and quads oriented along the face normal.
  - Planar faces without pcurves fall back to 3-D TFI plus projection.
  - `pub fn register(reg: &mut SchemeRegistry) -> Result<()>`.
- **Acceptance:**
  - A planar 10×6 rectangle with counts (5, 3) gives 15 quads, interior nodes on the exact TFI lattice within 1e-12, and area exactly 60 (`sm_quality`).
  - A trapezoid gives the TFI closed form.
  - A quarter-cylinder 4-sided face: every node lies on the cylinder within 1e-12 and the grid matches the uv lattice.
  - A face with a mid-edge vertex (5 vertices, Side corner) meshes as 4 logical sides.
  - A face with ≠ 4 sides, or with inner loops, gives `NotApplicable`.
  - `NodeGrid::line_from` walks match the element walk (`Patch::strip_from`).
  - The surface-only commit of all six brick faces (dimension-2 builder) gives V − E + F = 2.
- **Must NOT:** implement submap (W3); depend on sm-scheme-curve. Use the stub `equal` for curves in tests.

### T6. `sm-scheme-mapvol` (L3, new): volume map
- **Deps:** sm-core, sm-math, sm-geom-api, sm-topo-api, sm-scheme-api. Dev: sm-testkit, sm-scheme-api[stubs], sm-mesh, sm-quality.
- **Scope:**
  - `Scheme` "map", `dim = Volume`, caps `HEX | STRUCTURED | BLOCK_STRUCTURED`.
  - `requirements`: the piece has exactly 6 faces; impose `require_caps(face, QUAD | STRUCTURED, Hard)` on each.
  - `interval_constraints`: the three edge-direction classes are equal across the volume (derived from the face grids' logical sides).
  - `generate`: reads the six face grids through `GenCtx::grid`, orients them into one (i, j, k) frame, then emits interior nodes by 3-D transfinite interpolation (faces, edges and corners) and hexes with template corners.
  - `pub fn register(reg: &mut SchemeRegistry) -> Result<()>`.
- **Acceptance:**
  - A 10×10×10 brick, size 1, gives 1000 hexes, 1331 nodes, every node within 1e-12 of the integer lattice, total volume 1000 within 1e-9, `validate` clean, min scaled Jacobian = 1 within 1e-12.
  - A 2×3×4 brick with hard counts gives 24 hexes.
  - A parallelepiped built in the test with `make_*` gives TFI that is exact for affine maps (1e-12).
  - Any face rotation or orientation of the six grids produces the same mesh. Test it by building the brick with permuted face orders.
  - A non-six-faced piece gives `NotApplicable`.
  - Byte-identical output under `Sequential` and `ScopedThreads(4)`.
- **Must NOT:** claim faces (map does not write face meshes); implement sweep or submap; depend on sm-scheme-map. Test with the stub surface map from ACR-1.

### T7. `sm-mesher` (L3, new): the intent and meshes plugin, the mesh-driver node
- **Deps:** sm-core, sm-topo-api, sm-model-api, sm-mesh-api, sm-mesh, sm-scheme-api, sm-op-api, sm-layer-api, rayon, serde, schemars. Dev: sm-testkit, sm-scheme-api[stubs], sm-quality (replay through `sm_op_api::Session`).
- **Scope:**
  - `IntentLayer` (`intent`, depends on `model`).
  - `MeshPlugin` registers `IntentLayer`, the ops and the queries. It does not register `MeshLayer`; `CoreLayers` does.
  - `MesherConfig` service, a skeleton type: `new(registry)` defaults to the reference solver and picker, with `with_solver`, `with_picker`, `with_size_fields(Arc<dyn Fn(&SizingSpec) -> Result<Option<Box<dyn SizeField>>, SchemeError> + Send + Sync>)` and `with_auto_size(Arc<dyn Fn(&dyn TopologyView, &[EntityId], u32) -> Result<f64, String> + Send + Sync>)`.
  - `RayonExec: Executor`.
  - Ops:
    - `mesh.size {targets, size?, auto?, firmness?}`. `auto` is resolved to a size at apply time through the hook and recorded as a size.
    - `mesh.intervals {targets, count, firmness?}`.
    - `mesh.scheme {targets, scheme, params?, fallbacks?}`.
    - `mesh.run {targets}` (§1.4); output `mesh`(1). Refused on an already meshed volume ("mesh.delete first").
    - `mesh.delete {meshes? | targets?}` retires every `Mesh*` id of the mesh.
  - Query `mesh.info {mesh}`: counts by shape, `topology_hash`, quantized geometry hash, total volume, `stale`.
- **Acceptance:**
  - With the stub registry (ACR-1): `mesh.scheme(targets, scheme: "sweep", params: {source, target})`. The stub volume scheme is named `"sweep"` and takes source/target face params, exactly as in `sm-scheme-api/tests/brick.rs`; there is no alias mechanism. A brick of 10 with size 1 gives 1000 cells.
  - `State::count_of(MeshCell) == 1000` and `count_of(MeshVertex) == 1331`.
  - `Delta.created` equals the adopted ranges, and `outputs.mesh` = `[Mesh#1]`.
  - A failing scheme triggers re-plan and fallback, recorded in `MeshEntry.plan.attempts`.
  - Failure is atomic: a hard infeasible intent leaves the state hash and entity table unchanged.
  - `mesh.delete` brings the live entity counts back.
  - Intent digest: the same requests in a different op order give the same state hash. Repeated `mesh.size` replaces the old value; it does not accumulate.
  - Undo plus redo and cold replay give no `Diverged`.
  - `RAYON_NUM_THREADS=1` and the default give the same state hash for a 20³ brick.
  - Benchmark (criterion): 100³ brick plan + generate + commit; record the number in DESIGN.md.
- **Must NOT:** depend on scheme crates or on sm-sizing (both are injected); write `model`; register the `meshes` layer; mint ids outside `SerialAlloc` + `adopt_range`.

### T8. `sm-fem` (L3, new): blocks, sidesets, deck lowering, export
- **Deps:** sm-core, sm-topo-api, sm-model-api, sm-mesh-api, sm-meshio, sm-op-api, sm-layer-api, serde, schemars. Dev: sm-testkit, sm-scheme-api[stubs], sm-mesh.
- **Scope:**
  - `FemLayer`.
  - `FemPlugin` ops:
    - `fem.block_add {block: u32, volumes?, faces?, name?}`. It creates or extends the `Color` with `Domain::Volume` and label `block` through `model.add_color` / `color_volumes` inside `sm_layer_api::model_txn`, so it never opens a topology op. The Color id is adopted through `model_txn` roles, and the op outputs `block`. Moving a volume out of another block emits a warning diagnostic. Every fem op that edits `Model` uses `model_txn`.
    - `fem.block_remove`.
    - `fem.block_element_type {block, element_type}`, with `HEX8 | TET4 | WEDGE6 | PYRAMID5 | QUAD4 | TRI3`. Higher-order types give `Unsupported` until degree elevation exists.
    - `fem.sideset_add {sideset, faces, wrt?}`, using `Domain::Face` colors.
  - `pub fn lower(model: &ModelLayer, mesh: &MeshEntry, fem: &FemLayer, alloc: &SerialAlloc) -> Result<Lowered, FemError>`. It clones the mesh (`MeshHost::clone_host`) and adds sets through `MeshEdit::add_set`/`set_label`, minting set ids from a **clone** of the layer allocator:
    - block colors → `SetMembers::Cells` named by color, labelled by color label;
    - face colors → `SetMembers::Sides` wrt volume;
    - each deck boundary → face sets `deck_<boundary>_a`/`_b`, plus `DeckRecord::from_deck(..)` (meshio request 4 and decision 35 "Deck → named face-set lowering").
    - It checks that the element type matches the cell shapes.
  - `pub fn export(state: &State, req: &ExportRequest, w: &mut dyn Write) -> Result<ExportReport, FemError>`, with `ExportRequest {format: Smsh | Vtu, mesh: Option<EntityId>, provenance: Option<String>}`. It looks up the layers by type and writes through `SmshWriter`/`write_vtu`. It is a pure function of `(state, req)`.
- **Acceptance:**
  - Stub-meshed brick (`CellComplex` + driver) plus block 1 gives a `.smsh` with cell set `block_1` of label 1 holding 1000 cells; the round trip through `SmshFile::decode` matches.
  - The VTU has a `set:block_1` array that is all 1.
  - Two volumes in two blocks give disjoint sets that cover all cells.
  - A TET4 block on hexes gives an error.
  - Sideset on the +x face gives 100 sides, all on `Left` of their boundary face.
  - `sm_testkit::topo::mirror_chamber()` meshed gives a deck face set whose faces all lie on the mirror plane (1e-12), and `DeckRecord` names it.
  - Export is byte-identical over repeated runs and thread counts.
  - Export of a state with no `fem` layer works and writes no sets.
- **Must NOT:** mutate the `meshes` layer (lowering is export-time only); add an EntityKind; define a protocol; write files itself (it gets a `Write`).

### T9. `sm-lang` W2 delta (L4, existing crate)
- **Scope:**
  - `session.*` namespace mirroring `timeline.*`:
    - `session.export(path: STRING, format?: "smsh"|"vtu", mesh?: id, provenance?: bool)` and `session.save()` are statements;
    - `session.hash()` → string and `session.status()` → object are expressions;
    - they dispatch to `Host::session(&SessionCommand)`, whose stub ACR-5 adds pre-wave;
    - `session-in-batch` error; `plan-session` warning under `PlanHost`.
  - `x-entity-kind` checking at the argument span (W1 SHOULD 14) and kind-aware completion.
  - Static role checks from `OpInfo.outputs`: `q.voluem` gets did-you-mean in batch and plan mode; a bare `b.role` is **allowed in a batch when the declared cardinality is `One`** (relaxing an error into a meaning, so it is compatible).
  - Delete `schema::is_batch_ref` in favour of `sm_op_api::batch::is_reference`, and `ID_PATTERN` in favour of `sm_core::ENTITY_ID_PATTERN`.
  - Complete `timeline.*` / `session.*` names.
- **Acceptance:**
  - Golden diagnostics for every new error.
  - `session.*` events reach a recording test host with exact `SessionCommand` values.
  - A demo op with `x-entity-kind = ["TopoVolume"]` given a face set gives a `wrong-kind` error at the argument span.
  - A batch with bare `b.volume` for a `One` role equals the top-level form (same `core.batch` canonical bytes and same state hash).
  - All 136 existing tests still pass; LANGUAGE.md §4.6/§4.7 updated.
- **Must NOT:** do file I/O; implement any Host; change existing meanings (only error → meaning relaxations).

### T10. `sm-server` (L5, new): session host, protocol endpoint, transports, standard distribution
- **Deps:** sm-core, sm-op-api, sm-engine, sm-lang, sm-protocol, sm-layer-api, sm-fem, sm-model, sm-mesher, sm-sizing, sm-scheme-curve, sm-scheme-map, sm-scheme-mapvol, sm-tess, sm-topo (the `TopoStore` factory for `CoreLayers`), sm-scheme-api (`SchemeRegistry`), serde_json. Optional feature `ws` (tungstenite).
- **Scope:**
  - `SessionHost<'w>: sm_lang::Host` over `&mut sm_engine::Workspace` (W1 lang request 3, decision 35):
    - `apply` = `Workspace::apply_json(rec.name, rec.params, version)`;
    - `timeline(Branch{name, at})` → `Workspace::fork(name, at)`. This is verified at `sm-engine/src/workspace.rs:957`: it creates the ref **and** checks it out (HEAD attached, state moved), which is what LANGUAGE.md §4.6 requires, so no separate checkout. `Checkout(rev)` → `checkout`, `Undo`/`Redo`;
    - **Path rule for `session.export` / `Export.path`:** a relative path resolves against the project root when a project is attached (`-p dir`), otherwise against the server process's working directory. An absolute path is used as is. T12 documents the same rule;
    - `timeline_head` from `status()` (branch, 64-hex node);
    - `relations` = `QueryRelations::new("topo.related")`;
    - `load` from the request's include map, then the project directory;
    - `session(..)` → export through `sm_fem::export`, save, hash.
  - `Server: sm_protocol::Endpoint`, a single-writer dispatcher for every `Request` variant (ACR-4).
  - Transports: in-process, stdio JSON-lines, and `ws` behind the feature.
  - `pub fn standard() -> WorkspaceBuilder` with:
    - `CoreLayers::new(|| Box::new(TopoStore::new()))`, `ModelPlugin`, `MeshPlugin`, `FemPlugin`, `TessPlugin`;
    - services `Services::new().with_versioned("mesher", 1, MesherConfig::new(registry).with_solver(Box::new(IiaSolver::default())).with_picker(Box::new(AnglePicker::default())).with_size_fields(..).with_auto_size(..))`;
    - a registry built from `sm_scheme_curve::register`, `sm_scheme_map::register` and `sm_scheme_mapvol::register`, then `set_auto(Curve, ["equal"])`, `set_auto(Surface, ["map"])`, `set_auto(Volume, ["map"])`.
- **Acceptance (demo plugins as in `sm-engine/tests/common` and `sm-lang/tests/common`, plus `CoreLayers` and the stub schemes):**
  - `script.run` gives the same state hash and node sequence as the equivalent `apply` calls.
  - Every timeline request maps correctly: fork at an earlier node, then checkout back, gives a bit-identical state hash.
  - `session.hash` at a rev equals `Workspace::state_at(rev).hash()`.
  - Errors carry the contract `code()` strings.
  - A stdio transport round trip runs over an in-memory pipe.
  - `standard()` builds and its catalog lists the skeleton ops.
  - Determinism: the same request log gives the same responses with a pinned `Clock` and author.
- **Must NOT:** implement ops or layers; parse the command language itself (sm-lang does); hold more than one `Workspace` writer per project; put business logic in transports.

### T11. `sm-mcp` (L6, new): MCP server over the protocol
- **Deps:** sm-protocol, sm-op-api (for `OpInfo`), serde_json. **Not** sm-server: it codes against `dyn Endpoint`. Dev: a fake endpoint in its tests.
- **Scope:**
  - Stdio MCP (JSON-RPC 2.0, newline-delimited; protocol `2025-06-18`): `initialize`, `tools/list`, `tools/call`, `ping`; notifications are ignored.
  - Tools:
    - one per op and per query from `Request::Catalog` (`tool_name()`, `inputSchema` = params schema with `x-entity-kind`/`x-positional` kept, description from the schema plus `outputs`);
    - `core__batch` with `Registry::batch_schema()`;
    - fixed tools `run_script`, `session_hash`, `session_export`, `session_status`, `timeline_branch`, `timeline_checkout`, `timeline_undo`, `timeline_redo`, `timeline_log`, `help`. These have no `__`, so they cannot collide with op tools, which always contain `__`.
  - Results: `structuredContent` = the protocol result, `isError` for op failures, with ids respelled through `sm_lang::respell_json` in the text content only. The dependency on sm-lang is optional and can be dropped if that function is not wanted.
  - `pub fn run_stdio(endpoint: &mut dyn Endpoint, input: impl BufRead, output: impl Write) -> io::Result<()>`.
- **Acceptance:**
  - Scripted session transcripts (golden JSON) against the fake endpoint.
  - `tools/list` is sorted and stable, all names ≤ 64 bytes, every `inputSchema` has `type: object`.
  - Unknown tool or method gives the right JSON-RPC error.
  - Large results stream as one line.
  - A determinism golden.
- **Must NOT:** depend on sm-server or sm-engine; keep state beyond the endpoint; invent tools that are not ops, queries or protocol methods.

### T12. `sm-cli` (L7, new): the `smesh` binary
- **Deps:** sm-protocol, sm-server, sm-mcp, clap. Dev: assert on the binary via `env!("CARGO_BIN_EXE_smesh")`.
- **Scope:**
  - `smesh new <dir>`
  - `smesh run <script> [-p dir | --memory] [--author A] [--fixed-time T] [--print-hash]`
  - `smesh apply <op> <json>` and `smesh query <name> <json>`
  - `smesh log`, `smesh status`, `smesh hash [--at rev]`
  - `smesh checkout <rev>`, `smesh fork <name> [--at hash]`, `smesh undo`, `smesh redo`
  - `smesh export <file> [--mesh id] [--format]`
  - `smesh serve [--stdio | --ws addr]`
  - `smesh mcp [-p dir | --memory]`, which calls `sm_mcp::run_stdio` over an in-process `Server` built from `sm_server::standard()`
  - `smesh help [op]`
  - `--json` on every command. Exit codes: 0 ok, 1 op or script error, 2 usage error.
- **Acceptance:**
  - Each command against an in-process server with demo plugins.
  - Exit codes; `--json` schemas; `--fixed-time` makes node hashes reproducible.
  - It also writes `tests/milestone.rs` (§4) **marked `#[ignore = "W2 integration"]`**; the integration phase un-ignores it.
- **Must NOT:** talk to `Workspace` except through `sm-server`; contain meshing or modeling logic.

### T13 (micro). `sm-topo` maintenance (L1, existing crate)
- **Scope:** fix decision 36. `tests/ref_reference.rs::building_many_bricks_is_near_linear` should time through `TopoStore` directly (no `Dual` twin, paranoid off) and keep a 200-brick `Dual` variant for correctness.
- **Acceptance:** the test passes in release under its 2 s budget on the M1; total `cargo test -p sm-topo --release` time is reported.
- **Must NOT:** change store behaviour or weaken the differential coverage.

### Integration phase (after the merges): integration agent
- Owns `crates/sm-cli/tests/milestone*.rs` plus fix-up micro tasks routed back to owners.
- Runs §4. A failure becomes a fix task for the owning crate, never a cross-crate patch.

### Parallelism and blocking

| Task | Codes against | Blocks on another W2 crate? |
|---|---|---|
| T1 sm-model | sm-layer-api, sm-topo | no |
| T2 sm-tess | sm-layer-api, sm-topo-api fixtures | no |
| T3 sm-sizing | sm-scheme-api, stubs | no |
| T4–T6 schemes | sm-scheme-api, stubs (`equal`, `map`) for missing siblings | no |
| T7 sm-mesher | sm-layer-api, stubs, `CellComplex` fixtures | no (solver and sizing are injected) |
| T8 sm-fem | sm-layer-api, stubs + driver to make real classified meshes | no |
| T9 sm-lang | own crate | no |
| T10 sm-server | sm-protocol, `Host::session` stub, demo plugins; skeleton entry points for `standard()` | compiles day one; `standard()` behaviour is only meaningful after the merges (tested in the integration phase) |
| T11 sm-mcp | sm-protocol, fake endpoint | no |
| T12 sm-cli | sm-protocol + sm-server's public API (`Server::new(WorkspaceBuilder)`, `standard()`) | **soft**: needs T10's two constructors, so they are fixed in the skeleton (ACR-5); milestone test runs post-merge |

---

## 3. Contract changes before workers start (orchestrator, stage 0)

All are additive. ACR-3 and ACR-4 are new crates.

### ACR-1. `sm-scheme-api`: link the stub schemes (biggest enabler of parallel work)
- **What:**
  - Move `tests/stub_schemes/mod.rs` to `src/stubs.rs`, behind feature `stubs = []`, as `pub mod stubs` (`#[doc(hidden)]`; `expect`s allowed in test-only code).
  - Exported: `Equal`, `Map`, `Submap`, `Pave`, `Tri`, `Sweep`, `MSweep`, `MapVol`, `TetVol`, `PolyVol`, `AlwaysFail`, `BadReq`, `Noop`, `Lift`, `Boom`, `pub fn registry() -> SchemeRegistry`, `pub fn brick_setup(size: [f64; 3]) -> (CellComplex, BrickIds, Model)`, `pub fn row_setup(n: usize) -> (CellComplex, Vec<BrickIds>, Model)`.
  - The crate's own tests use `sm_scheme_api::stubs` through a dev-dependency on itself with the feature, or `required-features`.
- **Why:** without it, sm-mesher, sm-fem, sm-sizing, the three scheme crates and sm-server cannot run `Driver::mesh` end to end in isolation. The stubs are integration-test-only today.
- **Verified:** the stub module imports only `serde`, `sm-core`, `sm-mesh-api`, `sm-model-api`, `sm-topo-api` (including `sm_topo_api::fixtures`) and `sm-scheme-api` itself, all of which are already dependencies. It uses no sibling `tests/` helpers, so the move is mechanical.
- **Alternative:** `sm_testkit::schemes` would add an `sm-scheme-api` dependency to testkit. The feature keeps the stubs next to the trait.

### ACR-2. `sm-op-api`: output roles, positional marker, `x-entity-kind` (decision 33 design)
```rust
// registry.rs (additive)
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")] #[non_exhaustive]
pub enum Cardinality { One, Optional, Many }

#[derive(Clone, Copy, Debug, serde::Serialize)] #[non_exhaustive]
pub struct OutputRole { pub name: &'static str, pub kinds: &'static [EntityKind], pub cardinality: Cardinality, pub doc: &'static str }
impl OutputRole { pub const fn new(name: &'static str, kinds: &'static [EntityKind], cardinality: Cardinality, doc: &'static str) -> Self }

pub const X_ENTITY_KIND: &str = "x-entity-kind";   // property (or array) schema: ["TopoVolume", ...], canonical EntityKind names; on an array it applies to items
pub const X_POSITIONAL: &str = "x-positional";

// op.rs: TypedOp gains provided consts
const OUTPUTS: &'static [OutputRole] = &[];
const POSITIONAL: Option<&'static str> = None;
// OpType (object safe) gains provided methods
fn outputs(&self) -> &'static [OutputRole] { &[] }
fn positional(&self) -> Option<&'static str> { None }
// OpInfo (#[non_exhaustive]) gains
pub outputs: Vec<OutputRole>, pub positional: Option<String>,
```
- **Registry:** when `POSITIONAL = Some(p)`, it injects `"x-positional": true` into `properties[p]` of the published schema. Schemas are not hashed, so no format changes.
- **Engine:** when an op declares a non-empty `OUTPUTS`, an `alloc_as`/`adopt_as` role that is not declared, or a cardinality or kind violation, is `OpError::Invalid` (checked after the body). Undeclared ops are unchanged.
- **Why now:** W2 writes about 20 ops. Retrofitting the declarations later touches every op again. sm-lang (static role checks), MCP (tool descriptions) and Python stubs all read them. Not strictly needed for the milestone, but cheap only now.

### ACR-3. New contract crate `sm-layer-api` (C): the shared `model` and `meshes` layers
- **Deps:** sm-core, sm-geom-api, sm-topo-api, sm-model-api, sm-mesh-api, sm-op-api, sm-scheme-api (for `Plan`). It depends on no implementation crate: hosts are trait objects.
```rust
pub trait TopoHost: TopologyEdit + Send + Sync + 'static {
    fn clone_host(&self) -> Box<dyn TopoHost>;
    fn as_view(&self) -> &dyn TopologyView;
}
impl<T: TopologyEdit + Clone + Send + Sync + 'static> TopoHost for T { .. }

pub trait MeshHost: MeshEdit + Send + Sync + 'static {
    fn clone_host(&self) -> Box<dyn MeshHost>;
    fn as_read(&self) -> &dyn MeshRead;
}
impl<T: MeshEdit + Clone + Send + Sync + 'static> MeshHost for T { .. }

#[derive(Clone)]
pub struct ModelLayer { /* topo: Arc<dyn TopoHost>, model: Arc<Model>, next_op: u64 */ }
impl ModelLayer {
    pub fn new(topo: Box<dyn TopoHost>) -> Self;
    pub fn topo(&self) -> &dyn TopologyView;
    pub fn model(&self) -> &Model;
    pub fn tolerance(&self) -> ModelTolerance;
    pub fn hash_quanta(&self) -> HashQuanta;           // linear·2^-10, parametric·2^-10
}
impl Default for ModelLayer   // CellComplex host (reference); distributions pass a factory
impl LayerType for ModelLayer // NAME = names::MODEL; DEPENDS_ON = []; digest per §1.5; is_empty = no live cells && model == Model::new() && next_op == 0

/// A topology transaction: begin_op(next_op) .. f .. end_op; on Err abort_op. Model ids minted by
/// `f` (colors, groups, boundaries) are detected by diffing `Model::ids()`.
pub struct TopoTxn { pub summary: OpSummary, pub model_created: Vec<EntityId> }
pub fn topo_txn<R>(ctx: &mut ApplyCtx<'_>, f: impl FnOnce(&mut dyn TopoHost, &mut Model) -> Result<R, OpError>)
    -> Result<(R, TopoTxn), OpError>;
/// Adopt every id of the transaction in ascending (kind, serial) order, assigning roles where listed;
/// retire `summary.retired`; apply lineage to the model (`Model::apply_lineage`); record successors
/// from Split/Merged lineage. Must be called exactly once per successful `topo_txn`.
pub fn commit_topology(ctx: &mut ApplyCtx<'_>, txn: &TopoTxn, roles: &[(&str, &[EntityId])]) -> Result<(), OpError>;

/// Model-only transaction (fem.block_add, fem.sideset_add, coloring, groups): no begin_op/end_op,
/// `next_op` unchanged. Runs `f` on the model with read access to the topology, then adopts every
/// model-minted id (diff of `Model::ids()`) in ascending order with the given roles. Atomic: on
/// Err nothing is written or adopted.
pub fn model_txn<R>(
    ctx: &mut ApplyCtx<'_>,
    f: impl FnOnce(&mut Model, &dyn TopologyView) -> Result<R, OpError>,
    roles: impl FnOnce(&R, &[EntityId]) -> Vec<(&'static str, Vec<EntityId>)>,
) -> Result<R, OpError>;

// Mutable access inside ModelLayer (written down so it is not guessed): `Arc::make_mut` needs
// `T: Clone` and is unavailable for `dyn TopoHost`, so `topo_mut()` does `Arc::get_mut` when the
// Arc is unique, else replaces it with `Arc::from(self.topo.clone_host())` and returns that;
// `model_mut()` is `Arc::make_mut` (Model: Clone). Both are crate-private helpers used by
// topo_txn / model_txn; ops never hold `&mut dyn TopoHost` outside a transaction.

#[derive(Clone)] #[non_exhaustive]
pub struct MeshEntry { pub mesh: Arc<dyn MeshHost>, pub selection: Vec<EntityId>, pub plan: Option<Arc<Plan>>, pub quanta: HashQuanta, pub stale: bool }
impl MeshEntry { pub fn new(mesh: Box<dyn MeshHost>, selection: Vec<EntityId>, plan: Option<Plan>, quanta: HashQuanta) -> Self }

#[derive(Clone, Default)]
pub struct MeshLayer { /* BTreeMap<EntityId, MeshEntry>, SerialAlloc */ }
impl MeshLayer {
    pub fn meshes(&self) -> impl Iterator<Item = (EntityId, &MeshEntry)>;
    pub fn get(&self, mesh: EntityId) -> Option<&MeshEntry>;
    pub fn mesh_of(&self, entity: EntityId) -> Option<EntityId>;   // mesh whose selection contains a volume/face
    pub fn alloc(&self) -> &SerialAlloc;
    pub fn insert(&mut self, id: EntityId, e: MeshEntry, alloc_after: SerialAlloc) -> Result<(), LayerError>; // id must be the entry mesh's id
    pub fn remove(&mut self, mesh: EntityId) -> Option<MeshEntry>;
}
impl LayerType for MeshLayer  // NAME = names::MESHES; DEPENDS_ON = [names::MODEL]; digest per §1.5; on_retired marks stale

pub struct CoreLayers { /* topo factory */ }
impl CoreLayers { pub fn new(topo: impl Fn() -> Box<dyn TopoHost> + Send + Sync + 'static) -> Self }
impl Plugin for CoreLayers    // registers ModelLayer (via register_layer_with) and MeshLayer
```
- **Why a contract:** the mesher, fem, tess, model and server crates all read or write these two layers. If one implementation crate owned them, all the others would wait for its merge.
- **Why trait objects:** an API crate must not depend on an implementation crate (decision 17). The probe confirmed object safety and `Clone`.
- **Orchestrator test before freezing:**
  - Build a brick on a `CellComplex` through `topo_txn`/`commit_topology` in an `Engine`, and check `Delta.created` and that the state hash equals the hash of the same ops on `TopoStore`. This test lives in sm-layer-api's tests with a sm-topo dev-dependency.
  - Insert a `ReferenceMesh` from `hex_block` and check that the quantized digest is stable under a 1e-15 perturbation and that the exact (cache-key) digest changes.

### ACR-4. New crate `sm-protocol` (C, protocol v0, marked unstable until W3)
- **Deps:** sm-core, sm-op-api, serde, serde_json, schemars.
```rust
#[derive(Serialize, Deserialize, JsonSchema)]
#[serde(tag = "method", content = "params")] #[non_exhaustive]
pub enum Request {
    #[serde(rename = "catalog")]            Catalog,
    #[serde(rename = "apply")]              Apply { op: String, params: Value, #[serde(default)] version: Option<u32> },
    #[serde(rename = "query")]              Query { name: String, #[serde(default)] input: Value, #[serde(default)] at: Option<Rev> },
    #[serde(rename = "script.run")]         ScriptRun { text: String, #[serde(default)] path: Option<String>, #[serde(default)] includes: BTreeMap<String, String> },
    #[serde(rename = "timeline.status")]    Status,
    #[serde(rename = "timeline.log")]       Log { #[serde(default)] from: Option<Hash256>, limit: usize },
    #[serde(rename = "timeline.checkout")]  Checkout { rev: Rev },
    #[serde(rename = "timeline.branch")]    Branch { name: String, #[serde(default)] at: Option<Hash256> },
    #[serde(rename = "timeline.undo")]      Undo,
    #[serde(rename = "timeline.redo")]      Redo,
    #[serde(rename = "session.hash")]       Hash { #[serde(default)] at: Option<Rev> },
    #[serde(rename = "session.export")]     Export { path: Option<String>, #[serde(default)] format: Option<ExportFormat>, #[serde(default)] mesh: Option<EntityId>, #[serde(default)] provenance: bool, #[serde(default)] inline: bool },
    #[serde(rename = "session.save")]       Save,
    #[serde(rename = "session.open")]       Open { path: String, #[serde(default)] create: bool },
}
pub enum ExportFormat { Smsh, Vtu }
pub struct Envelope { pub jsonrpc: String, pub id: Value, #[serde(flatten)] pub request: Request }
pub struct RpcError { pub code: i64, pub kind: String /* contract code() */, pub message: String, #[serde(default)] pub data: Value }
pub struct ApplyResult  { pub node: Hash256, pub state: Hash256, pub delta: Delta }
pub struct HashResult   { pub state: Hash256, pub layers: BTreeMap<String, Hash256> }
pub struct ExportResult { pub path: Option<String>, pub format: ExportFormat, pub bytes: u64, pub sha256: Hash256, #[serde(default)] pub base64: Option<String> }
pub struct ScriptResult { pub state: Hash256, pub output: String, pub events: Vec<Value>, pub diagnostics: Value }
pub struct StatusResult { /* mirror of sm_engine::Status */ }
pub struct CatalogResult { pub protocol: u32, pub ops: Vec<OpInfo>, pub queries: Vec<QueryInfo>, pub batch_schema: Value }
pub trait Endpoint { fn call(&mut self, req: Request) -> Result<Value, RpcError>; }
pub const PROTOCOL_VERSION: u32 = 0;
```
- **Why:** concepts.yaml edges `mcp → server`, `cli → server` and `py → server` all go through "protocol". With the types fixed, sm-mcp and sm-cli are parallel with sm-server.

### ACR-5. Pre-wave skeletons and stubs (not frozen contracts, but the orchestrator writes them so nothing blocks)
- **Skeleton crates**, each with a `Cargo.toml` listing the allowed dependencies from §2: `sm-model`, `sm-tess`, `sm-sizing`, `sm-scheme-curve`, `sm-scheme-map`, `sm-scheme-mapvol`, `sm-mesher`, `sm-fem`, `sm-server`, `sm-mcp`, `sm-cli`. The root uses the `crates/*` glob, so workers never touch the root `Cargo.toml`. The entry points below stay fixed, and workers fill in the bodies:
  - `sm_model::ModelPlugin`, `sm_tess::TessPlugin`, `sm_mesher::MeshPlugin`, `sm_fem::FemPlugin`: unit structs implementing `Plugin` that register nothing yet.
  - `sm_scheme_curve::register`, `sm_scheme_map::register`, `sm_scheme_mapvol::register`: `fn(&mut SchemeRegistry) -> sm_scheme_api::Result<()>`.
  - `sm_sizing::{IiaSolver, AnglePicker}` (`Default`), `sm_sizing::size_field`, `sm_sizing::auto_size` (signatures in T3).
  - `sm_mesher::MesherConfig` (constructor and builders in T7).
  - `sm_fem::{export, ExportRequest, ExportReport, FemError}`. The skeleton `export` already writes a mesh without sets (identity lowering), so sm-server can test export.
  - `sm_server::{Server::new(WorkspaceBuilder) -> Result<Server>, Server: Endpoint, standard() -> WorkspaceBuilder}`.
  - `sm_mcp::run_stdio`.
- **`sm-lang` stub**, an additive change to an implementation crate:
  - the defaulted `Host::session(&mut self, cmd: &SessionCommand) -> Result<Json, HostError>` (default `session-unsupported`);
  - `#[non_exhaustive] pub enum SessionCommand { Export { path: String, format: Option<String>, mesh: Option<EntityId>, provenance: bool }, Save, Hash, Status }`.
  - T9 does the parsing; T10 implements the trait method from day one.

### Stage-0b notes (binding for stage-1 workers)
These came out of the stage-0b reviews and are now on main.

- **Ids in output roles.**
  - `ApplyCtx::record_role(role, ids)` reports surviving ids. Examples: moved volumes from `model.move`, and imprint survivors.
  - Do not retire an id after recording it in a role.
  - Ids are sorted within each call. Across calls they come out in call order. `topo_txn`/`commit_topology` roles are different: they are merged ascending across the whole transaction.
- **Mesh lifecycle.**
  - Insert meshes only through `sm_layer_api::insert_mesh`. Its minted-id set must equal the mesh's ids exactly, so any `MeshEdit` builder must assign ids only in `finish`.
  - Delete meshes only through `retire_mesh`. Calling `ctx.retire` on a `Mesh` id leaks its children.
  - `mesh.delete` over several meshes loops `retire_mesh` inside one op. Any error fails the whole op.
- **Model records.** `model_txn` retires the record ids that `f` removed (colors, groups, boundaries). Ops never retire them by hand.
- **Export with `mesh = None` (T8 and T10).**
  - If the state has exactly one mesh, export it.
  - If it has none, return a typed `no-mesh` error.
  - If it has several, return a typed `ambiguous-mesh` error.
  - The `Request::Export` doc on main already states this rule. T8 implements it, and T10 maps the errors.
- **T10.**
  - `standard()` must not discard scheme `register` errors. A fallible `try_standard() -> Result<WorkspaceBuilder, _>` plus a panicking `standard()` is fine.
  - Set the auto scheme lists strictly.
  - Add a project-backed constructor, such as `Server::open(builder, dir)` or `Server::from_workspace(Workspace)`, for `smesh -p dir`.
  - `Server::new` stays in-memory.
- **T11.** A malformed JSON line gets a -32700 reply with `id: null` (the skeleton skips it silently).
- **T2.** The query is `view.tessellate`; fix the skeleton's rustdoc.

**Orchestrator docs (no worker):** amend ARCHITECTURE §4 (`.cache/HEAD`, `.gitignore`, `project.toml` keys; decision 35); add the `sm-layer-api` and `sm-protocol` nodes to `docs/graph/concepts.yaml`.

**Explicitly not needed:**
- No new `EntityKind`: blocks are `Color`s keyed by label.
- No geometry transform ACR: `CurveDef/SurfaceDef::transformed` exists.
- No `sm-mesh-api` change: `MeshEdit::add_set`/`set_label` cover lowering.
- No `sm-engine` change: `apply_json`, `fork`, `clock`, `author` and `services` cover the host.

---

## 4. Integration milestone: executable test plan

**Home:** `crates/sm-cli/tests/milestone.rs`. It is the only crate where `env!("CARGO_BIN_EXE_smesh")` exists, and `smesh mcp` exposes MCP from the same binary, so all three paths are tested against one artifact. Data files go in `crates/sm-cli/tests/data/`. Written by T12 as `#[ignore]` and turned on by the integration agent.

### 4.1 Script `tests/data/milestone.smesh` (real sm-lang syntax: named args, string enums, `let`, `.role`, `[0]`)
```
// brick -> size -> map -> block -> export
let b = model.create_brick(x: 10.0, y: 10.0, z: 10.0)
assert count(b.volume) == 1
mesh.size(targets: b.volume, size: 1.0)
mesh.scheme(targets: b.volume, scheme: "map")
let m = mesh.run(targets: b.volume)
let info = mesh.info(mesh: m.mesh[0])
assert info.cells == 1000
assert info.vertices == 1331
fem.block_add(block: 1, volumes: b.volume)
session.export(path: "out.smsh")
session.export(path: "out.vtu")
print session.hash()
```

### 4.2 Fork script `tests/data/fork.smesh`
```
let b = model.create_brick(10.0)                 // single required param: positional
let h0 = timeline.head().node
mesh.size(targets: b.volume, size: 1.0)
mesh.scheme(targets: b.volume, scheme: "map")
mesh.run(targets: b.volume)
fem.block_add(block: 1, volumes: b.volume)
let main_hash = session.hash()
timeline.branch("alt", at: h0)                  // fork at the brick, re-apply the same ops
mesh.size(targets: b.volume, size: 1.0)
mesh.scheme(targets: b.volume, scheme: "map")
mesh.run(targets: b.volume)
fem.block_add(block: 1, volumes: b.volume)
assert session.hash() == main_hash
timeline.branch("coarse", at: h0)
mesh.size(targets: b.volume, size: 2.0)
mesh.scheme(targets: b.volume, scheme: "map")
let c = mesh.run(targets: b.volume)
assert mesh.info(mesh: c.mesh[0]).cells == 125
assert session.hash() != main_hash
timeline.checkout("main")
assert session.hash() == main_hash
```

### 4.3 CLI commands (spawned from the test, `T` = temp dir)
Export paths in the script are relative. They resolve against the project root under `-p` and against the process's working directory under `--memory` (rule in T10). So each `--memory` run is spawned with its own `current_dir` (`$T/m1`, `$T/m2`, `$T/m3`), and runs never overwrite each other's `out.smsh`.
```
smesh new $T/cli
smesh run tests/data/milestone.smesh -p $T/cli --author it --fixed-time 2026-10-24T00:00:00Z --json   # -> H_cli, node list N_cli; writes $T/cli/out.{smsh,vtu}
smesh hash -p $T/cli                       # reopen + strict replay; must print H_cli
smesh export $T/cli/re.smsh -p $T/cli      # export after cold replay
(cd $T/m1) RAYON_NUM_THREADS=1 smesh run <abs>/milestone.smesh --memory --print-hash --author it --fixed-time 2026-10-24T00:00:00Z  # H_1t
(cd $T/m2) smesh run <abs>/milestone.smesh --memory --print-hash --author it --fixed-time 2026-10-24T00:00:00Z                      # H_mem
(cd $T/m3) smesh run <abs>/milestone.smesh --memory --print-hash                                                                    # real clock: H equal, provenance differs
smesh run tests/data/fork.smesh -p $T/fork --json                                # asserts inside the script
smesh log -p $T/fork --json                # branches main, alt, coarse present
smesh checkout alt -p $T/fork && smesh hash -p $T/fork   # == H_cli
```

### 4.4 MCP calls (spawn `smesh mcp --memory --author it --fixed-time 2026-10-24T00:00:00Z`, newline-delimited JSON-RPC)
```
{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"it","version":"0"}}}
{"jsonrpc":"2.0","method":"notifications/initialized"}
{"jsonrpc":"2.0","id":2,"method":"tools/list"}
{"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"model__create_brick","arguments":{"x":10,"y":10,"z":10}}}
   -> structuredContent.delta.outputs.volume == ["TopoVolume#1"]
{"jsonrpc":"2.0","id":4,"method":"tools/call","params":{"name":"mesh__size","arguments":{"targets":["TopoVolume#1"],"size":1.0}}}
{"jsonrpc":"2.0","id":5,"method":"tools/call","params":{"name":"mesh__scheme","arguments":{"targets":["TopoVolume#1"],"scheme":"map"}}}
{"jsonrpc":"2.0","id":6,"method":"tools/call","params":{"name":"mesh__run","arguments":{"targets":["TopoVolume#1"]}}}
{"jsonrpc":"2.0","id":7,"method":"tools/call","params":{"name":"fem__block_add","arguments":{"block":1,"volumes":["TopoVolume#1"]}}}
{"jsonrpc":"2.0","id":8,"method":"tools/call","params":{"name":"session_export","arguments":{"path":"<T>/mcp.smsh"}}}
{"jsonrpc":"2.0","id":9,"method":"tools/call","params":{"name":"session_export","arguments":{"path":"<T>/mcp.vtu"}}}
{"jsonrpc":"2.0","id":10,"method":"tools/call","params":{"name":"session_hash","arguments":{}}}          # -> H_mcp
{"jsonrpc":"2.0","id":11,"method":"tools/call","params":{"name":"run_script","arguments":{"text":"<fork.smesh contents>"}}}   # in a second, fresh process: asserts pass
```
- `x: 10` (an integer here) against `10.0` (a float in the script) is deliberate. `apply_json` re-encodes through the registry, so the node bodies must be identical.

### 4.5 In-process path (same test file)
`sm_server::standard().memory()` with `apply_json` calls in milestone order gives `H_api`. It is also replayed through `Workspace::state_at` from the root (no caches).

### 4.6 Which hashes must match

| # | Assertion |
|---|---|
| 1 | `H_cli == H_mem == H_1t == H_mcp == H_api` (`State::hash()`, quantized), and the same equality per layer (`State::layer_hashes()`: `model`, `intent`, `meshes`, `fem`). |
| 2 | Fork: `hash(alt head) == hash(main head)`, `hash(coarse) != hash(main)`, and checkout back to main is bit-identical (asserted in the script and again from Rust). |
| 3 | Cold replay: reopening the project (`smesh hash -p`) reproduces `H_cli` under the strict `Diverged` policy. |
| 4 | Node hashes: with `--author` and `--fixed-time` pinned, the CLI and MCP node lists are equal. Unpinned (real clock), node hashes differ but every `Check.state` is equal. Node hashes include `Meta`; state hashes do not. |
| 5 | Exports: `out.vtu` bytes are equal across CLI, MCP and the re-export after replay. `out.smsh` bytes are equal with pinned meta. Without pinning they differ only in the provenance field: compare the decoded arrays and the header `HashInfo`. |
| 6 | Content: 1331 nodes and 1000 hexes, every node within 1e-12 of the integer lattice `{-5..5}³`, total volume 1000 (1e-9), `validate` clean, min scaled Jacobian 1 (`sm_quality`). The `.smsh` decodes with cell set `block_1`, label 1, 1000 cells. The VTU parses with `tests/vtu_check.py`-style stdlib decoding, and with VTK when `SM_PYTHON` has it. |
| 7 | Determinism: `RAYON_NUM_THREADS=1` and the default give the same state hash and identical export bytes (row 1 includes `H_1t`). |
| 8 | Reference vs production host: the script state hash with `CoreLayers::new(CellComplex)` equals the one with `TopoStore`. This is the canonical-stream equivalence the W1 differential tests guarantee. |

---

## 5. Risks and order

### Order (stages)
1. **Stage 0, orchestrator, gates everything.**
   - ACR-1 (stubs), ACR-3 (`sm-layer-api` + its two self-tests), ACR-2 (`OpInfo` roles), ACR-4 (`sm-protocol`), ACR-5 (skeletons + `Host::session` stub), docs.
   - Gate: the workspace builds, `cargo test -p sm-layer-api -p sm-scheme-api --features stubs` passes, and clippy is clean.
2. **Stage 1, at most 8 parallel workers**, queued by critical path:
   - first batch: T7 sm-mesher, T6 mapvol, T5 map, T1 sm-model, T8 sm-fem, T10 sm-server, T4 curve, T3 sm-sizing;
   - second batch: T9 sm-lang, T11 sm-mcp, T12 sm-cli, T2 sm-tess, T13 sm-topo.
3. **Merge order** (only to keep merges trivial, since the crates are disjoint):
   - sm-lang (the `Host::session` body) before sm-server;
   - scheme crates, sizing and mesher before sm-server's `standard()` test;
   - sm-cli last.
4. **Stage 2, integration agent:** turn on `milestone.rs`; failures go back as fix tasks to the owners; then the completeness critic against PARITY P0 rows (A: script, headless; B: brick; C: interval count, size, auto size, equal, map, map; D: blocks HEX8; E: `.smsh`, VTU; F: timeline fork, MCP, deterministic replay).

### Critical path
Stage 0 → {T5, T6, T7, T1, T8, T10} → T12 milestone. T3, T4, T9, T11, T2 and T13 have slack. Curve "equal" is on the path but small (T4 must deliver `equal` first).

### Risks
1. **`sm-layer-api` design error found mid-wave.** Its consumers are T1, T2, T7, T8 and T10, so a fix is costly. Mitigation: the orchestrator writes the two self-tests above, has one Opus reviewer write `mesh.run` and `create_brick` sketches against it before freezing, and keeps `MeshEntry` `#[non_exhaustive]`.
2. **Copy-on-write cost.** Every model op clones the whole `TopoStore` (`clone_host`) when the state is shared with the session caches. Trivial for W2, O(model) per op later. Track it as a W3 item (persistent rows or `Arc`-chunked arenas, plus the layer snapshot hook).
3. **Driver commit throughput at 10⁶ cells** is unmeasured: commit uses incremental `add_vertex`/`add_cell`. Mitigation: the T7 benchmark; if slow, an ACR for a bulk commit path using `MeshBuild::add_template_cells` (accepted in W1 decision 11).
4. **IIA optimality versus run time.** Mitigation: the documented size cap with fallback to the reference solver. Brute-force proptests guard correctness, and the milestone does not depend on optimality.
5. **Protocol churn.** v0 is labelled unstable; MCP and CLI read `PROTOCOL_VERSION`.
6. **`x-entity-kind` plus `OUTPUTS` adoption is uneven** across about 20 ops. Mitigation: a registry-level test in sm-server's `standard()` that every op declares `OUTPUTS` and every `EntityId` property carries `x-entity-kind`.
7. **Blocks as colors** inherit the exact-partition rule, so a volume can belong to one block only. Cubit allows overlaps with a warning. This is a decision for the orchestrator to ratify. The alternative, `Group`-backed blocks, can be added later as a new `fem` op without breaking stored projects.
8. **Stale meshes after topology changes.** W2 refuses transforms on meshed volumes and marks the entries of deleted volumes `stale`. Cubit's "move mesh with geometry" is W3.
9. **Build contention.** 12 worktrees on 10 cores; use the shared `sccache`, and queue by critical path, not by size.
10. **The MCP spec version moves.** Pin `2025-06-18` and negotiate down to `2024-11-05` if the client asks.
