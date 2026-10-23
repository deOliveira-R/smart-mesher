# sm-testkit: final design (frozen test-support contract)

Test support for every implementation crate; add it as a `[dev-dependencies]` entry. Orchestrator-owned: implementation crates do not edit it (request changes through `api_change_requests`). Depends on `sm-core`, `sm-geom-api`, `sm-topo-api`, `sm-mesh-api`, `sm-model-api`, `thiserror` and `proptest` (`default-features = false, features = ["std"]`, to keep compile time low). It has no `rayon`, no `serde` and no `sm-op-api`.

Principle: **reuse the contracts, add oracles.** Fixtures are the contract crates' own types (`Plane`, `Cylinder`, `CellComplex`, `ReferenceBuilder`, `Model`); this crate adds closed-form expectations and checkers, and wraps the contracts' own validators instead of re-implementing them. Where an independent implementation is cheap (orientation rule, cell closure, watertightness, Euler counts) it is written separately on purpose, so a bug in a contract validator does not hide itself.

| Module | Content |
|---|---|
| `geom` | `SurfaceExpect` (implemented for `Plane`, `Cylinder`, `Cone`, `Sphere`, `Torus`): exact normal, `K`, `H`, area of a parameter rectangle, closest point, sample parameters. `CurveExpect` (`Line`, `Circle`, `Ellipse`): tangent, curvature, arc length, closest point. `curvatures_from_jet`, `ellipse_perimeter` (AGM, machine precision), `surface_fixtures()` and `curve_fixtures()` (world and tilted frames). |
| `loops2d` | `Region2` (outer loop CCW, holes CW, exact area, expected submap `VertexType`s): square, rectangle, rectangle with two holes, annulus, L-shape, circle, five-sided submap region. `vertex_types`, `interior_angles`, turning weights. |
| `meshes` | `MeshFixture` (mesh, exact cell volumes). Single hex/tet/wedge/pyramid, truncated octahedron, hexagonal prism, cube with a split face, 2x2x2 hex block, hex+wedge+pyramid+tet transition, 7-cell honeycomb dual patch. `build_mesh` (points + `CellSpec`s, classified like a CAD solid) and `convex_loops`. |
| `topo` | `TopoFixture` (counts, Euler, volumes, area): cube, brick, cylinder, sphere, merged bricks, webcut cube. `ModelFixture`: `pin_lattice_stub` (3x3 lattice instance), `mirror_chamber` (webcut + mirror cut + deck face). `finish_boundaries`, `model_errors`. |
| `check` | `mesh_valid`, `orientation_rule`, `closed_cells`, `positive_volumes`, `watertight`, `classification_complete`, `check_mesh_euler`, `conformity` (+ `Carriers`, `box_carriers`); `topo_valid`, `check_topo_euler`, `manifold_edges`, `model_valid`; `mesh_hash`, `topo_hash`. All return `Result<(), CheckError>` (check name plus a list of problems). |
| `broken` | `Tampered` (editable copy of a `ReferenceMesh`; swap left/right, reverse a loop, move a vertex, drop a cell face, open the boundary, declassify) and broken topology/model fixtures (fin, baffle, two disjoint bricks, moved vertex, bad mirror deck). |
| `determinism` | `check(runs, threads, f)`, `par_map`, `Fnv`, `hash_f64_bits`. |
| `strategies` | `brick_spec`, `hex_block_spec`, `translation`, `rotation`, `reflection`, `proper_transform`, `any_transform`. |

## Conventions

- **Curvature sign.** `H` is positive when the surface bends away from its normal `du x dv` (a sphere with outward normal has `H = 1/r`). Normals follow the `sm-geom-api` convention (outward for sphere, torus, cone; radial for cylinder).
- **Discretized curves.** A circle or annulus with `n` points has the exact area of its *n-gon* (`ngon_area`), not `pi r^2`. Vertex types of such loops are not meaningful and `outer_types` is `None`.
- **Vertex types** (Cubit nomenclature): interior angle about 90, 180, 270, 360 degrees = `End`, `Side`, `Corner`, `Reversal`. Right-angled outer loops have turning sum `+4` (weights `+1, 0, -1, -2`), hole loops `-4`.
- **Mesh classification.** `build_mesh` puts cells on `TopoVolume#1`, groups boundary faces by plane into `TopoFace#k`, vertices on one group on that face, on two groups on a `TopoEdge`, on three or more on their own `TopoVertex`, interior vertices on the volume. Ids come from a fresh `SerialAlloc` per fixture, so fixtures are reproducible bit for bit.
- **Hash rule.** `topo_hash` is `TopologyView::canonical_hash` (quantized, cross-platform). `mesh_hash` mixes `connectivity_hash`, `topology_hash` and `geometry_hash`.

## Determinism harness

`check(runs, threads, f)` with `f: Fn(usize) -> T` (the argument is the number of workers `f` may use): baseline `f(1)`; then `runs` times `f(1)` and `f(threads)`; then `threads` concurrent `f(1)` calls on std threads (catches hidden global state). A difference returns `DeterminismError::Mismatch` naming the run; a panic in a concurrent call returns `Panicked`. `par_map(threads, n, f)` runs `f(0..n)` on contiguous chunks and returns results in index order, so a reduction over its result is independent of scheduling. Crates that use rayon call `check` with a closure that builds a pool of `threads` workers (the equivalent of `RAYON_NUM_THREADS=1` against the default).

## Negative tests

`MeshBuild::finish` normalizes orientation and rejects bad incidence, so a finished mesh cannot be invalid. `Tampered` copies the columns into public owned vectors and implements `MeshRead` over them; every checker has a test that fails on a tampered or broken input, and the same checker passes on the untouched copy (`tests/negative.rs`). Topology negatives use `brick_with_fin`, `brick_with_baffle` (valid but not manifold), `two_disjoint_bricks` (Euler 4), `brick_with_moved_vertex` (carriers contradict the points) and `model_with_bad_mirror`.

## Tests (`cargo test -p sm-testkit`, about 1 s)

`tests/geom.rs` (normals, curvature, area against quadrature, closest points against `project`, lengths, ellipse perimeter), `tests/loops2d.rs`, `tests/meshes.rs` (volumes, shapes, counts, every mesh checker on every fixture), `tests/topo.rs` (counts, Euler, volumes and areas, flatten counts, deck face, symmetry detection), `tests/negative.rs`, `tests/determinism.rs` (harness self-test and the fixture pipeline), `tests/strategies.rs` (proptest, 32 cases per property).

## Limits and deviations

- No ellipse closest point (a quartic root; `Ellipse::project` is `Unsupported` in the contract) and no closed-form partial ellipse arc length; only the full period has one.
- The sphere's `face_moments`/`volume_moments` are errors in the reference complex (faces touch poles), so `sphere` carries closed-form expectations that the reference topology cannot confirm; the test asserts the error instead.
- `manifold_edges` fails on the merged-bricks complex (three faces meet on the junction edges) by design: it checks a manifold *surface* complex, not a valid non-manifold B-rep.
- `Tampered` leaves the reverse classification index stale (as a corrupted file would).
- Fixtures that the task lists but the contracts cannot yet express: none missing. The mirror-cut chamber's deck face is the cap of `Model::apply_cut`; the 3x3 pin lattice is a stub (rectangular pins, no cylinders).
- The task asked for "dual patch" and "prism split polyhedron": the dual patch is seven welded hexagonal prisms; "prism split polyhedron" is the cube with a split face plus a hexagonal prism.
