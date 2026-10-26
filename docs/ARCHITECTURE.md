# smart-mesh — Architecture (draft v0)

Status: **proposal, awaiting review**. Nothing here is built yet.

## 1. Goals

1. Feature parity with Coreform Cubit (tracked in [`PARITY.md`](PARITY.md)), then surpass it.
2. Own B-rep geometry kernel. OpenCASCADE/ACIS papers and theory are reference material, not dependencies.
3. High performance: native code, SIMD, multithreading, and GPU compute where it pays off.
4. **Timeline:** every change is a recorded operation. You can step back to any point and fork from it.
5. **Git-native state:** a project is a directory of plain text files that diff and merge sensibly.
6. **Client/server split:** the backend runs locally or remotely; the frontend connects to either; headless mode is first-class.
7. **Agent-native:** an MCP server exposes the same operations a human uses, so Claude can drive it.
8. Modern mesh output with first-class **polyhedral** cells. The first consumer is our neutron transport code.

## 2. Language and stack

| Layer | Choice | Why |
|---|---|---|
| Kernel, meshers, I/O, server | **Rust** (cargo workspace) | Native speed, SIMD (`std::simd`/`wide`), easy data parallelism (`rayon`), memory safety for a codebase written by many agents, and compile-time enforcement of crate interfaces, which is what makes parallel development safe. Compiles to WASM. |
| GPU compute (later) | `wgpu` compute shaders | Portable (Metal/Vulkan/DX12). For smoothing, quality evaluation, big queries. |
| Frontend | **TypeScript + WebGPU** (three.js/WebGPU renderer or raw) | Runs in a browser against a local or remote backend; can be wrapped in Tauri for a desktop app later. Gets render-ready binary buffers from the backend, so it never holds the full volume mesh. |
| Scripting | **Python** via PyO3 (`smartmesh` module) | Parametric studies, notebooks. Same op API. |
| Agent interface | **MCP server** (Rust, stdio + HTTP) | Thin adapter over the backend protocol. |

Why not C++: we would gain little speed over Rust and lose safety and the compiler-checked crate boundaries. cadcore shows a pure-Rust B-rep is practical.

## 3. The central idea: everything is an Op

```
  Command language ─┐
  Python API ───────┤
  MCP tools ────────┼──►  Op (serializable data)  ──►  Engine.apply(state, op) ──► new state + Delta
  GUI actions ──────┘                                           │
                                                                 ▼
                                                        Timeline (op DAG)
```

- An **Op** is a plain, serializable, versioned value, e.g. `CreateBrick{ x, y, z }`, `Webcut{ targets, tool }`, `SetScheme{ entity, scheme }`, `Mesh{ entities }`, `AddToBlock{...}`.
- **Every mutation goes through an Op.** No front end mutates state directly. (Cubit has the same principle: its Python API is read-only and changes go through `cmd`.)
- `apply` is a **pure, deterministic** function of (prior state, op). That gives us:
  - **History:** the state at any timeline node is a replay from the root, sped up by cached snapshots.
  - **Forking:** a new op whose parent is an older node makes a branch.
  - **Git:** commit the op log, not the derived geometry/mesh.
  - **Remote:** the client sends ops; the server sends deltas.
  - **Testing:** golden tests are op scripts plus expected hashes.

### Determinism rules (binding on all crates)
- No iteration over `HashMap`/`HashSet` where order affects output (use `IndexMap`/`BTreeMap` or sort).
- Parallel reductions must be order-independent or done in a fixed order.
- RNG is always explicitly seeded from the op.
- Floating-point results must be bit-identical across thread counts on the same platform.
- Different platforms can't be bit-identical (libm and FMA differ). The CI state hash therefore covers topology plus **quantized** coordinates, and that hash must match on every platform.

### Persistent naming: two features, scoped separately
- **(a) Checkout / fork (P0–P1).** Replay a prefix of the timeline exactly, then append new ops. Nothing upstream changes, so deterministic IDs are enough and the naming problem never comes up. This is what the timeline needs.
- **(b) Edit an old op and regenerate everything downstream (P2).** This is the topological naming problem from parametric CAD. It needs lineage plus **selectors**, geometric queries such as "the face of body 3 with normal +z" that survive upstream edits.

`sm-naming` now provides: stable IDs assigned deterministically at creation, short integer display IDs (as in Cubit), and **lineage recording** ("surface 12 split into 12.a and 12.b by op #47"). Selectors come later and build on the recorded lineage. Agents must not build (b) ahead of time.

## 4. Project on disk (git-friendly)

```
myproject.smesh/
  project.toml            # format version, name, [units], [tolerances], [metadata]; no timestamps
  .gitignore              # "/.cache/" (written at create)
  timeline/
    ops/<hash>.toml       # one op per file: {parent = "<hash>", op = {...}, author, time}
    refs/heads/main       # text file containing a node hash (like git refs)
    refs/heads/try-pave
  inputs/                 # imported CAD (STEP etc.), LFS if large
  exports/                # optional pinned outputs
  .cache/                 # per checkout, gitignored, never authoritative:
    HEAD                  #   current ref or node (per checkout, like git's HEAD; W0a decision 1)
    reflog.jsonl          #   undo history
    redo.json             #   redo stack
    snapshots/            #   state snapshots (accelerators; replay is the source of truth)
    lock                  #   advisory write lock
```

See `crates/sm-engine/DESIGN.md` for the exact `project.toml` keys and file formats.

- Ops are content-addressed and point to their parent, so appending never causes merge conflicts. Concurrent branches simply coexist.
- Git history (what was committed) and the in-app timeline (every action, including experiments) are separate on purpose. The in-app timeline is saved in the project and committed along with it.
- Derived data is rebuilt by replay. Expensive results can be pinned in `.cache` and shared through a content-addressed store keyed by op hash.

## 5. Crate map and interfaces

Each crate is a unit of parallel work with a written interface contract (`docs/interfaces/<crate>.md`) and its own test suite.

```
L0  sm-math        vectors, transforms, tolerances, robust exact predicates (orient2d/3d, incircle/insphere), intervals
L1  sm-geom        curves & surfaces: analytic (line, circle, ellipse, plane, cylinder, cone, sphere, torus) + NURBS;
                   eval/derivatives, closest point, curve/surface intersections, parametric utilities
L1  sm-topo        cellular B-rep: Body ▸ Volume ▸ Shell ▸ FaceUse ▸ Face ▸ Loop ▸ Coedge ▸ Edge ▸ Vertex
                   **non-manifold** (volumes share faces after merge, as in Cubit), arena + typed stable IDs, lineage.
                   FaceUse (radial-edge style) records how each volume sees a shared face (sense ±), the same way
                   Coedge records how a face sees an edge. Sidesets "wrt volume", sweep source/target and shell
                   closure depend on it.
L2  sm-naming      persistent IDs, lineage tracking, selectors
L2  sm-model       geometry ops: primitives, extrude/revolve/sweep/loft, transforms,
                   virtual topology (composite/partition/collapse), healing/defeaturing
L2  sm-boolean     booleans (unite/subtract/intersect), imprint, merge, webcut family (one fragment kernel)
L2  sm-tess        tessellation of B-rep for display & facet-based geometry
L2  sm-cadio       STEP AP203/214/242 read+write (content-addressed import), IGES read, STL/OBJ read+write
L1  sm-mesh        mesh store: nodes, mixed elements (bar/tri/quad/tet/hex/wedge/pyramid/**polyhedron**,
                   high-order), owner association (every node/element owned by a topo entity), adjacency
L2  sm-sizing      intervals, interval matching (integer program), size fields, bias, sizing functions
L2  sm-quality     quality metrics (Verdict-equivalent set + finite-volume face metrics), validity classification
L3  sm-scheme-*    one crate per scheme family, all implementing the same traits (below)
L3  sm-diag        mesh.validate / mesh.quality / mesh.flag_cells: located, geometry-linked, actionable diagnostics
L3  sm-poly        tet → polyhedral conversion (median dual, planar faces, features), agglomeration
L3  sm-rt          ray tracing / particle tracking (implements sm-rt-api: cell walking, BVH locate, .smrt cache)
L3  sm-smooth      Laplacian, centroid area pull, equipotential, Winslow, condition-number/optimization, untangle
L3  sm-modify      refine, coarsen, pillow, collapse, remesh, align, mesh boolean/cleanup
L2  sm-fem         blocks, sidesets, nodesets, materials, element type assignment, BC sets
L2  sm-meshio      native format (below), VTU/VTKHDF, OpenFOAM polyMesh (ASCII), Gmsh msh4, Abaqus, Nastran, CGNS,
                   DAGMC h5m, Fluent (spec: docs/research/fluent-msh.md)
L2  sm-exodus      Exodus II writer (own netCDF classic / 64-bit-offset encoder), NFACED/NSIDED polyhedra
L4  sm-engine      Op registry, apply(), state, Delta, undo/redo, timeline DAG, snapshots, replay
L4  sm-lang        our command language → Ops (parser, completion, help generated from Op schemas)
L4  sm-rt-capi     C ABI (+ DAGMC-shaped shims) over sm-rt-api trackers for Fortran/C++ transport codes
L5  sm-server      session host; protocol over WebSocket (JSON-RPC control + binary frames for buffers)
L5  sm-mcp         MCP tools → protocol
L5  sm-py          PyO3 bindings
L5  sm-cli         `smesh` binary: headless run of scripts, server start, export
    frontend/      TS + WebGPU client
```

A crate may depend only on crates in lower layers.

### Contracts are code, not prose
Written interface docs alone won't keep 20 agents in sync. The contracts are **small frozen `*-api` crates that only the orchestrator edits**:

- `sm-geom-api`: `ParametricCurve`, `ParametricSurface` traits, param ranges, eval/derivative/projection signatures
- `sm-topo-api`: typed IDs, read-only topology queries, `FaceUse`/`Coedge` sense
- `sm-mesh-api`: mesh data types, element types, ownership association
- `sm-scheme-api`: `CurveScheme`/`SurfaceScheme`/`VolumeScheme`/`MeshTransform`, inputs, `SizeField`, `IntervalConstraint`, errors
- `sm-op-api`: `Op` trait, `Delta`, op metadata/schema

Plus **`sm-testkit`**: synthetic fixtures (unit square, L-shape, annulus, plate with holes, analytic cylinder/sphere/torus, discretized boundary loops, cube/cylinder volumes with source/target) and shared property checks (positive Jacobians, conformity, watertightness, Euler characteristic). Every implementation crate compiles and tests against `*-api` + `sm-testkit` from day one, before the real `sm-geom`/`sm-topo` exist. An agent that needs an interface change asks the orchestrator. It does not edit the API crate itself.

### The key mesher interfaces (sketch)

```rust
/// What every surface scheme receives. It never touches the B-rep store directly.
pub struct SurfaceMeshInput<'a> {
    pub surface: &'a dyn ParametricSurface,      // eval(u,v), derivatives, inverse projection
    pub boundary: &'a [BoundaryLoop],            // ordered, already-discretized loops (nodes + params)
    pub hard_points: &'a [ParamPoint],
    pub size: &'a dyn SizeField,
    pub params: &'a SchemeParams,                // scheme-specific options (serde)
}
pub trait SurfaceScheme: Send + Sync {
    fn name(&self) -> &'static str;
    fn applicability(&self, input: &SurfaceMeshInput) -> Applicability; // for auto scheme selection
    fn interval_constraints(&self, input: &SurfaceMeshInput) -> Vec<IntervalConstraint>; // for matching
    fn mesh(&self, input: &SurfaceMeshInput) -> Result<SurfaceMeshPatch, MeshError>;
}
// CurveScheme and VolumeScheme follow the same pattern. Conversion schemes (THex, QTri, HTet)
// implement `MeshTransform`. The engine runs the hierarchy: vertices → curves → surfaces → volumes.
```

Because schemes see only these plain inputs, they can be built and tested without the kernel, using synthetic inputs (a unit square, an annulus, an analytic cylinder). That is what makes 20+ scheme crates parallelizable.

## 6. Native mesh format (`.smsh`, proposal)

Requirements from the transport code: polyhedral cells, regions/materials, boundary labels, fast reads of large meshes, easy readers in C++/Python/Fortran, and room for partitions and high-order geometry.

- **Container:** one file with a JSON header (schema, array table) followed by 64-byte-aligned little-endian arrays. It can be memory-mapped and read with zero copies, and a reader is about 100 lines in any language (similar idea to safetensors). The schema does not depend on the container, so an HDF5 mapping can be added if HDF5 tooling turns out to matter more.
- **Topology:** CSR-style arrays. Standard cells have `cell_type` + `connectivity` + `offsets`. Polyhedra are defined by faces: a `faces` CSR plus a `cell_faces` CSR with orientation signs, the same model as CGNS NFACE_n and OpenFOAM.
- **Labels:** cell regions (blocks/materials), face sets (sidesets), node sets, all with names and integer IDs.
- **Optional:** owner geometry IDs for each entity (for adaptivity and re-meshing later), high-order nodes, partition maps, provenance (project timeline hash that produced it).
- Exporters to Exodus II, VTKHDF/VTU (ParaView, which supports polyhedra), Gmsh, and DAGMC keep other tools in reach.

## 7. Protocol (backend ⇄ frontend/MCP/Python)

- Transport: WebSocket (remote/local) or in-process (Python, tests).
- Control: JSON-RPC: `apply_op`, `query`, `timeline.checkout/branch/list`, `export`, `subscribe`.
- Data: binary frames with render buffers (surface tessellations, mesh skins, highlight sets) with LOD, sent as diffs.
- Op schemas are generated from Rust types (`schemars`), so MCP tool definitions, command-language help, Python stubs and frontend types all come from one source.

## 8. Development model

- **Phase 0, serial (orchestrator):** workspace skeleton; `sm-math`; all `*-api` crates (topology data model with FaceUse, mesh data model, Op/engine contract, scheme traits); `sm-testkit`; CI with determinism checks.
- **Phase 1, parallel:** each crate gets an agent working against frozen interfaces. Interface changes go through the orchestrator only.
- **Verification:** each scheme and op has synthetic and analytic tests, quality thresholds, property tests (watertightness, Euler characteristic, positive Jacobians), and golden op-script hashes. Install Gmsh as a reference mesher for comparison benchmarks.

First vertical slice (end of Phase 0):
`brick 10 10 10` → `size 1` → `mesh` (mapped hex) → `block 1 add volume 1` → `export out.smsh` + `out.vtu`,
running from the CLI, from MCP, and replayed from the timeline with a fork.

## 9. Open questions

1. Native format: should `.smsh` be co-designed with the transport code's mesh reader in the other repo? That code's needs should decide the format, including own zero-copy container vs HDF5. Which repo, and who owns the reader?
2. Units/tolerances: should the project have one global modeling tolerance (ACIS style) or tolerant topology per entity (Parasolid style)? Proposal: global default, per-entity overrides on imported geometry.
3. License for the project.
4. Command language style: verb-first (`mesh volume 1`) or something closer to a small expression language (`mesh(vol[1])`, pipelines, variables)?
