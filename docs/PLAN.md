# smart-mesh — Execution plan (dynamic multi-agent workflow)

How we build `ARCHITECTURE.md` to `PARITY.md` with orchestrated agents.

## Roles

| Role | Model | Does |
|---|---|---|
| Orchestrator | Opus (main session) | Owns the `*-api` crates, workspace `Cargo.toml`, CI, and merges. Writes each wave's workflow script, reads the results, and decides the next wave. |
| Workers | Sonnet | Implement one crate or one scheme against the frozen APIs, each in its own git worktree and branch. |
| Advisors / judges / reviewers | Fable | Design panels, adversarial code review, numerical-correctness review, and API-gap critique. |

## Rules that make parallelism safe

1. **Disjoint ownership.** Each task owns exactly one directory (`crates/<name>/`). Phase 0 pre-creates every crate skeleton and adds it to the workspace, so workers never touch `Cargo.toml` at the root, the `*-api` crates, or another crate.
2. **Frozen contracts.** `*-api` crates change only through the orchestrator. A worker that hits a gap returns an `api_change_request` in its structured output instead of working around it. Requests are batched between waves, and an API bump re-runs the affected crates' tests.
3. **Stubs before dependencies.** Workers compile against `*-api` traits + `sm-testkit` fixtures, not against sibling implementations. For example, a paving agent tests on a synthetic annulus long before `sm-geom` exists.
4. **Worktree isolation.** Each worker runs in its own worktree on branch `wave<N>/<crate>`, which also keeps cargo target-dir locks from colliding. Merges are trivial because the directories don't overlap.
5. **Definition of done** (checked by script and reviewer, not taken from the worker's word): `cargo build`, `cargo test -p <crate>`, `cargo clippy -D warnings`, a determinism test (same output with 1 thread and N threads, and across repeated runs), property tests from `sm-testkit`, and docs on every public item.
6. **Determinism and no-HashMap-order rules** from `ARCHITECTURE.md §3` are part of every worker prompt.

## Per-task harness (each item in a wave)

```
implement (Sonnet, worktree): commits on branch wave<N>/<crate>, returns {branch, sha, status}
  status ∈ {done, blocked_on_api}. blocked_on_api exits the pipeline at once with its api_change_request
  → gate (low effort): build/test/clippy/determinism   ──fail──┐
  → adversarial review (Fable, 2 lenses:               │
      correctness/numerics + contract/determinism)     │
  → fix loop (Sonnet), at most 3 rounds  ◄──────────────┘
  → result: {branch, status, test summary, api_change_requests, known_gaps}
```
Run as `pipeline(tasks, implement, gate, review, fix)` with no barrier: a fast crate finishes while slow ones are still running.

## Per-wave harness

```
wave workflow:  pipeline(per-task harness)  →  orchestrator merges green branches
                → integration agent: workspace build + cross-crate tests + milestone scenario
                → completeness critic (Fable): "which PARITY items in this wave's scope are missing/untested?"
orchestrator (between waves): resolve API change requests, bump APIs, update PARITY.md checkboxes, plan the next wave
```

The orchestrator reviews every wave before starting the next, so plans change based on what each wave actually produced.

## Waves

### W0: Contracts (orchestrator-heavy, gate for everything)
0. Initial commit: docs, `CLAUDE.md` (binding rules, injected into every agent automatically), workspace skeleton.
1. **Design panels, one per API**, run concurrently. Each panel has 3 Fable designers with different approaches, and each designer delivers **compiling Rust**: the api crate plus one stub implementor.
   - `sm-geom-api` + `sm-topo-api` together, because they decide how topology references geometry (face → surface handle), the tolerance model, cellular non-manifold topology, FaceUse/Coedge, and IDs + lineage. **`sm-naming`'s P0 scope (stable IDs + lineage) folds into `sm-topo`**; selectors stay a W5 crate.
   - `sm-mesh-api`: mixed + polyhedral + high-order elements, ownership, CSR storage.
   - `sm-scheme-api`: built from `research/cubit-schemes.md` (vertex types, interval constraints, imposed child schemes, fallback chains, atomic multi-volume transforms).
   - `sm-op-api`: Op, Delta, timeline node, schema generation.
   Judges score each design by writing stubs that express 3–4 PARITY items against it (does it compile, and is it natural?). The orchestrator synthesizes from the winner and grafts in the best ideas from the other designs.
2. **Orchestrator writes** `sm-math` core types, the final `*-api` crates, CI script, and skeletons for every crate in the map.
3. **`sm-testkit` with real fixtures** (agents): analytic `ParametricSurface`/`ParametricCurve` impls (plane, cylinder, cone, sphere, torus), discretized boundary loops, small hex/tet/wedge/pyramid/polyhedral meshes, synthetic B-rep volumes (cube, cylinder, merged two-block), and property checkers. W1/W2 depend on these.
4. **API gap critique, loop until dry** (parallel Fable reviewers, one per PARITY section): write stub code expressing every PARITY item. Any item that can't be expressed, or that needs a breaking change, is a gap and gets fixed in the API. Repeat until 2 rounds find no new gaps.

### W1: Foundations (about 10 parallel tasks)
`sm-math` exact predicates · `sm-geom` analytic curves/surfaces · `sm-geom` NURBS · `sm-topo` arena + Euler ops + stable IDs/lineage · `sm-mesh` store + adjacency · `sm-engine` (op registry, timeline DAG, replay, snapshots, project on-disk format) · `sm-meshio` (.smsh + VTU) · `sm-lang` parser → Ops · `sm-quality` metrics · `sm-testkit` fixture expansion.

### W2: First vertical slice (about 10 tasks)
`sm-model` primitives + transforms · `sm-tess` · curve schemes (equal, bias, dualbias, stretch, curvature) · surface map · volume map · `sm-sizing` (intervals, IIA-style interval matching, size fields, corner picking/vertex types) · `sm-fem` · `sm-server` + protocol · `sm-mcp` · `sm-cli`.
**Milestone:** `brick → size → mesh (map) → block → export .smsh/.vtu`, run from the CLI, from MCP, and from a timeline fork, with matching state hashes.

### W3: Core parity (about 14 tasks)
Booleans · imprint/merge (shared faces) · webcut family · STEP read/write · submap · pave · tri (Delaunay + advancing front) · sweep (1-to-1, rotational) · tetmesh (Delaunay + boundary recovery + optimization) · smoothing suite · Exodus II writer (pure Rust netCDF) · PyO3 bindings · frontend viewer (TS + WebGPU) · auto scheme selection.
**Milestone:** multi-volume merged CAD part → conformal hex/tet mesh → sidesets → Exodus + .smsh, verified in ParaView. Compared against Gmsh on quality metrics.

### W4: Full parity (about 16 tasks)
Virtual topology (composite/partition/collapse) · healing/defeaturing · multisweep (many-to-one/many) · polyhedral meshing (tet dual) + polyhedron scheme · THex/QTri/HTet/TQuad · copy mesh · boundary layers · refinement/coarsening/pillow · high-order elements · remaining exporters (Gmsh, Abaqus, Nastran, CGNS, DAGMC) · IGES/STL import · mesh-based geometry · MCNP CSG import · sizing functions (curvature, linear, skeleton) · GPU smoothing/quality.

### W5+: Beyond Cubit
Edit-old-op-and-regenerate (selectors) · Sculpt-like overlay hex · adaptive remeshing from solution fields (transport-code coupling) · collaborative multi-user sessions.

## Running it

- Each wave is one `Workflow` run (W0 is two: the design panel, then the gap-critique loop). This machine has 10 cores, so at most **8 agents run at once**; larger waves queue automatically. Eight parallel `cargo` builds in separate worktrees will saturate the CPU, so use `sccache` as a shared compiler cache. If that's still too slow, move the backend build to a remote box (the architecture supports this anyway).
- Prerequisites: `rustup update` (installed Rust is 1.73 from 2023; we need ≥1.85 for edition 2024), `cargo install sccache cargo-nextest`, `brew install gmsh` (reference mesher for comparisons).
- Between waves the orchestrator merges, commits (`wave<N>: ...`), updates `PARITY.md`, and reports a short status to the user.
- Workflow runs can be resumed if interrupted (`resumeFromRunId`).
