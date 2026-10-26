# sm-smooth: design

## Positioning

Similar to Mesquite (Brewer, Freitag, Knupp: Jacobi / Gauss-Seidel sweeps of node-local
minimisation of target-matrix quality) and to Cubit's `smooth` family (Laplacian, centroid-area
pull, Winslow, condition number, with surface and curve constraint handling), differs in four
ways, giving a smoother that is safe to run on any classified mesh and replayable.

1. **Constraints come from the B-rep classification and the exact analytic carriers**
   (`ParametricCurve::project`, `ParametricSurface::project`, `classify_point_on_face`), not from
   facets or a stored normal field. A node on a model edge slides on that edge, a node on a face
   slides on that face inside its trim, a vertex node never moves.
2. **"Never inverts" is a contract on every method** (the guard), not an option, and it covers
   the cells of unselected neighbours too, so the mesh stays conformal.
3. **Decks and interfaces are respected**: mirror-deck nodes slide in the mirror plane (the
   plane is the face carrier), translation / rotation pairs and explicit interface faces are
   pinned.
4. **Deterministic**: colouring plus ordered writes make every schedule bit-identical for 1 or N
   threads; the op is on the replayable, hashable timeline with a preview query and located
   diagnostics.

Disadvantage today: Mesquite has analytic gradients and Hessians (Feasible Newton) and handles
more element types, so our finite-difference steepest descent is slower per node and may stall
in flat valleys. Advantage: exact geometry projection and a deterministic, replayable result.

## Public surface

`smooth(&SmoothInput, &SmoothRequest) -> Result<SmoothOutcome, SmoothError>` is pure (nothing is
written). `apply_outcome(&mut dyn MeshEdit, &SmoothOutcome)` runs `set_point` for each moved
vertex, then one `set_classes` with the refreshed `Param`s. `SmoothPlugin` registers the op
`mesh.smooth` (NO_OUTPUTS, writes only the `meshes` layer through `MeshLayer::replace_mesh`) and
the query `mesh.smooth_preview` (same arguments, returns the `SmoothReport`).

Fixtures (all from `sm_testkit::lattice`, names as in the task): `hex`, `tet_kuhn`, `quad`,
`tube_quad`, `perturb`, `LatticeFixture::perturbed`, plus `sm_testkit::topo::mirror_chamber`,
`sm_testkit::determinism::check_f64_bits`, `Session::state_at_from_root`.

## Pipeline

1. **Checks.** `high_order_cell` or `face_curve` anywhere, extra coordinate nodes, or inactive
   (refined) cells give `Unsupported`. Request ranges give `BadParams`.
2. **Template cells.** CSR `cell -> corners` (`cell_corners`) and `vertex -> cells`. A cell
   without a corner list (a polyhedron) freezes all its vertices, reason `polyhedral`.
3. **In-entity adjacency** (the Laplacian stencil), from the face loops (`columns()`; a dim-2
   mesh has two-vertex facets). Each loop edge `(a, b)` of a mesh face classified on entity `X`
   is a candidate neighbour pair, accepted for node `v` (classified on `E`) when

   | class of `v` | accepted when |
   |---|---|
   | `TopoVolume` | always (every edge neighbour) |
   | `TopoFace` `F` | `X == F` and `class(w)` in the closure of `F` |
   | `TopoEdge` `E` | `X == E` or `X` is above `E` (`coboundary`), and `class(w)` in the closure of `E` |

   The closure is computed through `topo.boundary`. Neighbours are sorted and distinct, so the
   mean is over distinct nodes. A dim-2 tube seam works because the seam facets are classified
   on the lateral face, which is above the seam edge, and their end nodes are on the seam edge.
4. **Selection** (`NodeSelect`).
   - `All`: volume-classified nodes; with `fix_boundary: false` also edge and face nodes. In a
     dim-2 mesh the `TopoFace` nodes are the interior nodes and are always selected
     (deviation: the task text says "volume-classified", which would leave a surface mesh with
     nothing to move; `fix_boundary` then governs the `TopoEdge` nodes only).
   - `Entities`: a listed `TopoVolume` selects its nodes. A listed `TopoFace` or `TopoEdge` selects
     its own nodes explicitly (any `fix_boundary`). With `fix_boundary: false`, the face and edge
     nodes in the closure of the listed volumes also move when every **meshed** volume above them
     (via `coboundary`) is listed. The closure test counts only entities that carry mesh nodes
     (`classified_entities`), so a face shared with an unmeshed neighbour moves with its volume,
     and a lone meshed face (`lattice::quad` sits on one face of a brick) does not pin its edges
     because of the unmeshed side faces. In a dim-2 mesh a listed `TopoFace` plays the volume's
     role for its edges.
   - `Vertices`: explicit; the constraint table and pins still apply.
   An unknown or node-less listed entity gives `UnknownEntity`; a selection with nothing free
   gives `NothingToSmooth`.
5. **Pins**, in this order, counted in `report.pinned` by reason for selected nodes:
   `vertex` (class `TopoVertex`), `unprojectable` (class `Partition*`, or no resolvable
   curve / parametric surface), `polyhedral`, `deck` (faces of a translation or rotation
   deck pair and their boundary edges), `interface` (explicit `Interface` boundary faces and
   edges), `unsupported_shape` (Smart / Optimize when an incident cell has no `Shape` /
   `Condition`, for example a polygon), `winslow_unstructured`, `isolated`. Mirror decks are
   not pinned: the face carrier is the plane. `report.pinned_entities` lists the faces
   (`deck`, `interface`) or classifying entities behind the pins (at most 20 each).

## Constraint table

| Class | Motion |
|---|---|
| `TopoVertex` | fixed |
| `TopoEdge` | candidate projected with `curve.project(p, Some(t))`; `t` wrapped by the curve period into the `CurveCarrier.range`; rejected if outside |
| `TopoFace` | `surface.project(p, Some(uv))`, then `classify_point_on_face`; rejected if `Out` |
| `TopoVolume` | free in R^3 |
| `Partition*`, composite / implicit-only carrier | pinned, `unprojectable` |

The stored `Param` seeds the hint. The hint is then per-node state, updated when a move is
accepted, so it never depends on scheduling. `NaN` means "no hint" (global projection).

## Update rules

- **Laplacian**: `x <- x + relax (mean(neighbours) - x)`, then project.
- **SmartLaplacian**: the same; accepted only if the minimum incident `Shape` does not decrease.
- **CentroidArea**: `x <- sum A_c centroid_c / sum A_c` with `A_c = Metric::Volume` (area in
  2-D, never negative) and the vertex mean as centroid. Face nodes of a 3-D mesh use the
  in-entity mesh faces (`A` = half the Newell norm); edge nodes use the in-entity edges
  (length-weighted midpoints). Then `relax`, project.
- **Winslow** (2-D, all-quad, a node with exactly four incident quads whose wedges close):
  neighbours `E, N, W, S, NE, NW, SW, SE` are read in the orientation of the cells.
  `x_xi = (E-W)/2`, `x_eta = (N-S)/2`, `alpha = |x_eta|^2`, `gamma = |x_xi|^2`,
  `beta = x_xi . x_eta`,
  `x <- [alpha (E+W) + gamma (N+S) - beta/2 (NE - NW - SE + SW)] / (2 (alpha + gamma))`;
  then `relax`, project. The update is invariant under a 90 degree relabelling and affine
  invariant. Every other node is pinned `winslow_unstructured`.
- **Optimize**: `F_v = sum_{c in v} beta_c^p`, `beta = 1/Shape` (`MeanRatio`) or `Condition`
  (`Condition`), evaluated with `sm_quality::eval` on the template corner lists (one code path
  for hex, tet, quad, tri, wedge, pyramid). `F_v` is `+inf` when a cell is degenerate or its
  metric undefined. Gradient: central finite difference with `delta = 1e-6 l_v`, `l_v` the mean
  in-entity edge length at the start. Direction: `-g` projected on the tangent space (R^3; the
  plane orthogonal to the unit surface normal; the unit curve tangent), normalised. Armijo
  backtracking, `c1 = 1e-4`, initial step `0.25 l_v`, halving, at most 11 trial steps
  (`0.25 l_v 2^-k`, `k = 0..=10`). Each trial is projected, must pass the guard, and must satisfy
  `F(trial) <= F(x) - c1 alpha |g_t|`. Nothing acceptable: the node stays. The node is also left
  alone when `|g_t| l_v <= 1e-12 F`.

## Guard (always on)

For a candidate position `x'` of node `v`, for **every** incident template cell `c` (cells of
unselected volumes included):

```
SJ'_c >= min(SJ_c, guard_floor)   and   (Vol_c > 0  =>  Vol'_c > 0)
```

with `SJ = Metric::ScaledJacobian`, `Vol = Metric::Volume`, `' ` meaning "with `v` at `x'`".
A non-finite `SJ'` fails. A cell that is already inverted may not get worse. In 2-D every
`eval` gets `Context.normal` = the carrier normal (surface normal at the nearest point to the
cell's vertex mean, times the carrier sense; `CarrierNormals`, a private `FaceNormals`), so a
flipped quad reads inverted (`Context::default()` would use the cell's own Newell normal and
never see a flip). The same `CarrierNormals` is passed as `SurfaceNormals::Field` to
`compute` / `validity` for the report.

## Schedules

- **Colored**: greedy first-fit colouring of the movable nodes in ascending index; two nodes
  conflict when they share a template cell. Colours run in order; inside a colour every node is
  computed in parallel from the current positions into a slot per node and the results are
  written in ascending index order. Same-colour nodes share no cell, so a node never reads
  another node of its colour and the global `sum beta^p` is non-increasing each sweep.
- **Jacobi**: all candidates from the old positions (each passes the per-node guard against
  the old positions), applied together; then a repair loop: every cell touched by a moved node is
  checked against the pre-sweep state with the guard predicate, and all moved nodes of every
  violating cell are reverted (at most 20 passes, then the whole sweep is reverted). Reverts are
  counted in `rejected_guard`. (Deviation: the per-node guard in the compute phase is extra, it
  only reduces the work of the repair loop.)
- **Stopping**: the sweep limit, or `max_v |dx_v| / l_v < tolerance`, or a sweep that moved
  nothing (a fixed point). `max` is order independent. `converged` is true in the last two
  cases.

Counters (`accepted`, `rejected_guard`, `rejected_geometry`) are node-update counts summed over
sweeps; `accepted` counts updates that changed the position.

## Op and diagnostics

`mesh.smooth { mesh, entities?, method, iterations?, tolerance?, schedule?, fix_boundary?,
guard_floor? }`, `deny_unknown_fields`, `x-entity-kind` on every id. `method` is internally
tagged (`{"kind":"laplacian","relax":1.0}`, `{"kind":"optimize","objective":"mean_ratio","p":4}`).
The body validates and smooths on the shared mesh first (no copy), then clones the host,
applies the outcome and calls `MeshLayer::replace_mesh` (allocator unchanged, no id minted),
so every failure leaves the state byte-identical. Error mapping: `UnknownEntity` ->
`OpError::UnknownEntity`, `Unsupported` -> `OpError::Unsupported`, the rest -> `Invalid`.

Diagnostics: info `smooth.report` (the report as JSON); warnings `smooth.inverted_input`
(mesh plus up to 10 inverted cell ids; hint that untangling is deferred), `smooth.no_improvement`
(minimum `Shape` not improved; suggests intervals, scheme, `fix_boundary` or `optimize`),
`smooth.not_converged`, `smooth.pinned_deck` (names the faces), `smooth.pinned_interface`,
`smooth.pinned_polyhedral`.

## Measured values

Acceptance thresholds were not relaxed. Measured values (release build, this VM, 4 cores) are
in `W3_REPORT.md`; the ones that fixed a free choice of the task text are:

- Acceptance 4: with `lattice::perturb` the per-axis displacement is `amp * U[-1, 1)`, so at
  `0.45 h` most seeds invert quads (only seed 4 of seeds 0..29 stays valid). The test scans
  seeds 0.. and takes the first valid one (seed 4, initial min `ScaledJacobian` 0.0608).
- Acceptance 4 (L shape): a 8x8-cell L region with the reflex corner moved by (-0.5, -0.5), interior
  jitter 0.3 h, seed 19; one unguarded Jacobi Laplacian sweep inverts a cell. Boundary nodes are
  fixed on a `TopoVertex`.
- Acceptance 7: Colored Gauss-Seidel on 10^3 converges like `cos^2(pi/10)^k`; after 100 sweeps
  from a 0.2 h volume jitter the deviation is 1.8e-6 (above 1e-6), from a 0.1 h jitter it is
  below 1e-6. The test uses 0.1 h.

## Performance (criterion, record-only, 4 cores)

`cargo bench -p sm-smooth --bench smooth`; each case runs 1 and 10 sweeps, so one sweep costs
`(t10 - t1) / 9` and the fixed cost (preparation, the two quality passes, cache build) is
`t1 - (t10 - t1) / 9`. 64^3 hex lattice (274 625 nodes, 250 047 movable), volume nodes
jittered by 0.2 h, `fix_boundary: true`:

| case | t1 | t10 | per sweep | fixed | node updates / s |
|---|---|---|---|---|---|
| Laplacian, Colored | 0.542 s | 2.324 s | 0.198 s | 0.34 s | 1.26 M |
| Laplacian, Jacobi | 0.593 s | 2.775 s | 0.242 s | 0.35 s | 1.03 M |
| Optimize p=4, Colored, 16^3 (3 375 movable) | 43.8 ms | 506.9 ms | 51.5 ms | below noise | 0.066 M |

Before the per-cell metric cache (the guard re-evaluated the old cells of every candidate) the
same cases measured 0.672 s / 3.735 s (0.340 s per sweep, 0.74 M updates/s, Colored),
0.679 s / 4.095 s (0.380 s per sweep, 0.66 M/s, Jacobi) and 45.1 ms / 497.9 ms (50.3 ms per
sweep, Optimize). The cache halves the metric evaluations of Laplacian-type sweeps and does not
help `Optimize`, which is dominated by the 6 gradient and up to 11 line-search objective
evaluations per node.

The target of 5 M node-updates/s for Laplacian on 4 threads is **not met** (1.26 M/s). The
guard dominates: each candidate evaluates `ScaledJacobian` and `Volume` on the 8 hexes of the
node through `sm_quality::eval` (about 150-200 ns per call, including its power-of-two
rescaling). Options for W3b: a fused SJ+volume kernel for the guard, a skip of the volume test
when every corner Jacobian is positive by a margin, and a cheaper smooth-region fast path. The
whole-mesh clone of `replace_mesh` and the `set_classes` rebuild are not in these numbers (the
bench calls the pure library function).

## Deviations from Mesquite and Cubit

- Steepest descent with finite-difference gradients instead of feasible Newton; one objective
  family (`sum beta^p`), no target matrices, no TMP metrics, no untangling.
- Cubit's `smooth` has no inversion guard (it can tangle a mesh); here the guard cannot be
  disabled. Cubit pairs deck nodes; here they are pinned (tied smoothing is W4).
- Constraints are exact projections; Mesquite moves boundary nodes on facets and Cubit on
  its geometry engine's closest-point query. Equivalent for analytic carriers, exact here.
