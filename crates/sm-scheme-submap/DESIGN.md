# sm-scheme-submap

The surface `submap` scheme: a single-loop face with more than four logical sides (L, U,
staircase, ...) is meshed as a union of mapped rectangular blocks, all-quad, with valence-4
interior nodes.

## Positioning

> Conceptually **Cubit's `submap`** (and Gmsh's transfinite surface on a decomposed face): the
> corner types must sum to 4, opposite directions carry equal interval sums, and the face is
> meshed as mapped rectangular sub-blocks. It differs from `sm-scheme-map` in the vertex rule
> (`VertexRule::Submap`, corner sum 4 instead of four Ends) and in the interval rows (one row
> per *direction*, `sum(+i) = sum(-i)` and `sum(+j) = sum(-j)`, instead of per opposite side).
> It differs from the `Submap` in `sm-scheme-api::stubs` in that the stub only works for four
> sides and has no geometry: here the lattice is validated exactly, decomposed and meshed.
> The result is a conforming, all-quad, valence-4-interior mesh whose layout is a pure
> function of the solved plan; node keys are canonical, so output is bit-identical for 1 and
> N threads.

## Phases

| Phase | What it does |
|---|---|
| `caps` | `QUAD \| BLOCK_STRUCTURED` (no `STRUCTURED`: a volume `map` refuses it; a sweep reads it through `Patch::strip_from`). `Patch::grid` is `None`. |
| `applicability` | `no` for a non-surface target, params, more than one loop, embedded edges or vertices, fewer than 4 corners, any `Reversal` (measured or requested). Otherwise corner types as the picker will see them (firmest request wins, else `VertexType::nearest`); sum of `submap_weight()` = 4 gives `yes(80)`, anything else `yes(20, "corner types need picking")`. Auto selection is W3b; the auto list is unchanged. |
| `requirements` | `vertex_rule(face, VertexRule::Submap)` |
| `interval_constraints` | `sum_equal(edges of +i sides, edges of -i sides)` and the same for `j`, from `LogicalSide::dir` (never recomputed). An empty row after cancellation is skipped. No layout or fewer than 4 sides is `SchemeError::Input`. |
| `generate` | below |

The two rows give **closure** of the lattice polygon and nothing more: a spiral or a deep U
satisfies both and still overlaps itself. `generate` catches that exactly (below) and reports a
`Constraint` failure that the driver turns into its fallback chain.

## `lattice` (pure, no topology types)

`lattice_polygon(&[(SideDir, u32)])` accumulates the interval sums along the directions from
`(0, 0)` and checks, in integers: at least four sides, no zero side, closure, **positive signed
area** (face on the left), **simplicity**. Sides are axis-parallel, so a closed side is its own
bounding box and "two sides meet" is "two boxes overlap": every non-adjacent pair is tested,
neighbours must not fold back. `pick_area()` returns `(A, B, I)` with `B = sum(lens)` and
`I = A - B/2 + 1`; `inside(a, b)` is an exact ray cast. Every message starts with `lattice:`.

Internally a CSR index (`PointRows`) numbers every point of the closed polygon in row-major
`(b, a)` order from the per-row cell spans; it replaces the `BTreeMap<(i64,i64), _>` the task
sketches (same determinism, no per-point allocation, `O(log runs)` lookup), see W3_REPORT.md.

## `blocks`

`decompose(&LatticePolygon) -> Vec<Block>`: from every reflex vertex the incident side that runs
along `j` extends into the interior to the first boundary contact. The cuts are vertical, so
they are parallel and never cross, and every piece is a rectangle. It is computed as the
vertical slab decomposition (strips between consecutive vertex abscissae, pieces between the
horizontal sides that span the strip) with horizontally adjacent equal pieces merged: two pieces
with equal `j`-interval are never separated by a cut, and two with different intervals always
are (the reflex vertex at the step ends a cut that covers the whole shared segment), so the
merge reproduces the cut pieces. The tests compare `cuts(decompose(p))` with the literal ray
extension on 200 seeded staircases and on plus, H and T shapes. Blocks are sorted by
`(j0, i0)`; `cuts(&blocks)` lists the shared segments `(x, j0, j1)`.

## Generation

1. `side_nodes(f, k)` of every side gives the boundary nodes; side `k` starts at
   `verts[k]` and node `m` sits at `verts[k] + m * step(dir)`. The lattice polygon is built from
   `(dir, nodes - 1)`; a lattice failure maps to `Constraint` (message from `SubmapError`,
   the first node of every side attached through `GenerateFailure::with_nodes`).
2. **Interior numbering:** the interior points (not boundary nodes) are ranked in row-major
   `(b, a)` order; the key is `NodeKey::new(f, rank)`. Boundary nodes are `PatchBuilder::refer`.
3. **Cut nodes:** a cut `P -> Q` with `m` interior points gets `P + (Q - P) s/(m+1)`,
   `s = 1..m`, then `geom::snap_to_face` (so a cut is straight in 3-D, an exact lattice line on
   a planar face).
4. **Block interiors:** Coons (TFI) on the four lattice lines of the block with 3-D chord-length
   fractions averaged over opposite lines (`u = (ub[i] + ut[i]) / 2`, `v = (vl[j] + vr[j]) / 2`,
   as in `sm-scheme-map`), then `snap_to_face`. The kernel is a private copy (`tfi.rs`, 3-D
   only); sharing it through a math crate is out of scope.
5. One `Quad` per lattice cell, row-major, `[(a,b), (a+1,b), (a+1,b+1), (a,b+1)]`; the loop has
   the face on its left, so quads follow the face normal.
6. **Orientation check:** each quad's diagonal cross product must agree with
   `geom::face_normal` at its centroid; a fold is `FailureKind::Quality` with the four nodes
   attached.

Failure mapping: lattice closure/positivity/simplicity -> `Constraint`; hard `Reversal` in the
layout -> `NotApplicable`; `snap_to_face` or `face_normal` failure -> `Geometry`; missing node
or layout, or an inconsistent count (Pick vs the index vs the quads) -> `Internal`. No
`unwrap` on input-reachable paths.

## Numeric invariants (all tested)

- `A` quads, `I + B` nodes, `V - E + F = 1`; the area equals the lattice-cell union exactly on a
  planar face (`1e-12`).
- Uniform boundaries give unit-lattice quads with scaled Jacobian `1` (to `1e-12`); the
  rectangle's node set is bit-exact.
- Blocks are disjoint, total area `A`; a `k`-step staircase has `k + 1` blocks.
- Interior ranks are `0..I` in row-major order; no thread-dependent assignment anywhere.

Measured: the hard-count L of acceptance 2 (counts `5,3,3,2,2,5`, strongly non-uniform
boundary spacing) has minimum scaled Jacobian **0.9806**; the test pins `0.9 x` that value
(0.8825, above the 0.5 floor). Graded cut-line spacing (stretch) is therefore not needed for
the shipped cases and is not implemented.

That holds only for the shipped cases. Strongly graded hard counts on the L (counts in
side order) still give valid meshes with exact area but low quality, measured in review:

| L counts | min scaled Jacobian |
|---|---|
| (10, 1, 9, 9, 1, 10) | 0.2195 |
| (7, 6, 1, 1, 6, 7) | 0.324 |
| (3, 1, 2, 5, 1, 6) | 0.614 |

These are the measured motivation for the stretch item (graded cut-line spacing, W3b).

## Cost and parallelism

`generate` is `O(rows * sides + points)`: one pass for the row index, one for boundary nodes,
one `snap` per interior node (`project` on the carrier), one pass for quads. Buffers are flat
`Vec`s sized once. Parallelism comes from the driver (jobs of a wave); inside one face the
work is sequential, so results do not depend on the thread count.

## Deferred to W3b

- Parameter-space blending through pcurves (`GenCtx::loop_uvs`), periodic and seam faces.
- `Reversal` (slit) corners.
- Inner loops (holes).
- Auto selection by score (`yes(80)` / `yes(20)` are provisional, decision 12 and ruling S1).
