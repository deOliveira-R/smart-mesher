# sm-scheme-map

The surface `map` scheme: a face with four logical sides is meshed as one structured block of
quadrilaterals by transfinite (Coons) interpolation in the face's parameter space.

## Positioning

> Like **Cubit's `map` scheme** (and Gmsh's `TransfiniteSurface` + `Recombine`), it meshes a face
> with four logical sides as an `ni x nj` block, opposite sides carrying equal interval sums,
> corners typed End/Side by the driver's vertex rule. It differs in three ways:
> - **(A) The blend runs in the surface's `(u, v)` space, using the coedge pcurves.** Boundary
>   nodes get parameters from their own coedge's pcurve, so a periodic seam (`u = 2 pi` on one
>   coedge, `u = 0` on the other) is unambiguous, and every interior node is an *evaluation of
>   the surface*, not a projection of a 3-D blend. (A 3-D blend plus projection is also kept, as
>   the fallback for faces without pcurves.)
> - **(B) Pure and parallel-safe.** `generate` is a pure function of the plan; interior nodes
>   have canonical keys `(face, (j-1)(ni-2)+(i-1))`, so 1 thread and N threads are bit-identical.
> - **(C) Declarative.** It states `VertexRule::Map` and two interval rows; the corner picker and
>   the interval solver (not the scheme) decide which corners are Sides and the counts.
>
> This gives exact lattices on planar and analytic faces (nodes on the surface to rounding, grid
> equal to the `(u, v)` lattice for uniform boundaries), well-defined meshes across seams, and a
> `NodeGrid` that a volume `map`/`sweep` can walk without knowing this scheme's numbering.

## Phases

| Phase | What it does |
|---|---|
| `caps` | `QUAD \| STRUCTURED \| BLOCK_STRUCTURED` |
| `applicability` | `NotApplicable` for: non-surface targets, non-empty params, anything but exactly one boundary loop (inner loops), embedded edges/vertices (map cannot conform), fewer than 4 real corners. Score 100 for four End corners (rest Side), 50 when only End/Side corner types occur but the picker must change some, 10 when re-entrant (Corner/Reversal) corners would be forced to Side/End. |
| `requirements` | `vertex_rule(face, VertexRule::Map)` |
| `interval_constraints` | logical sides 0 = 2 and 1 = 3 as sums over the coedges of each side (several coedges per side add up). A seam used on both opposite sides cancels and is skipped (an empty row is invalid). A layout with a side count other than four is reported as a `NotApplicable` generation failure, which the driver blames and re-plans. |
| `generate` | below |

## Generation

1. Four logical sides come from `GenCtx::side_nodes`; the boundary lines are bottom = side 0,
   right = side 1, top = side 2 reversed, left = side 3 reversed. Grid frame, quad order
   `[(i,j), (i+1,j), (i+1,j+1), (i,j+1)]` and interior key numbering are exactly the contract
   convention of `NodeGrid` (row `j = 0` along side 0, column `ni-1` along side 1). The loop has
   the face on its left, so quads follow the face normal.
2. **Blending weights** are normalized *3-D chord-length fractions* of the boundary nodes,
   averaged over opposite lines (`u = (ub[i] + ut[i]) / 2`, `v = (vl[j] + vr[j]) / 2`, see
   `tfi`). Uniform straight sides give the bilinear map of the corners (trapezoid closed form,
   exact lattice on a rectangle); graded sides give an interior that follows the grading. The
   weights are taken from 3-D distances (the physical node distribution) even when the blend
   itself runs in `(u, v)`, so the result does not depend on how the surface is parametrized.
3. **Coordinates of the blend** (the frame):
   - *parameter space* when the face has a surface carrier and every coedge of the four sides has
     a pcurve: node `(u, v)` from the coedge pcurve at the node's curve parameter (vertex nodes at
     the carrier range ends, honouring edge sense and coedge side); the sides must agree at
     shared corners (1e-9 relative) and close the loop, else the face falls back to 3-D.
     Interior parameters are Coons-blended, points come from `ParametricSurface::eval_many`, and
     each interior node carries `Param::Surface([u, v])`.
   - *3-D + projection* (planar faces without pcurves, e.g. the brick fixtures; any face whose
     pcurves are missing or inconsistent): Coons in 3-D, each interior node projected onto the
     carrier (`ParametricSurface::project`, with the previous node's `(u, v)` as hint).
   - *3-D, no projection* for a face with no surface carrier (`Param::None`).
4. **Orientation check** (`FailureKind::Quality`): every quad's diagonal cross product must have
   a positive dot product with the Newell normal of the boundary outline (in `(u, v, 0)` for the
   parameter-space path). A fold-over (non-mappable boundary) is a fallback-eligible failure,
   not a malformed mesh.
5. The patch holds the boundary by reference (`PatchBuilder::refer`), the interior as owned
   nodes, `Patch::grid`, and one `Quad` per cell.

## Layout

- `tfi.rs` is a dimension-generic kernel (`chord_fractions`, `Weights`, `coons`) with no
  dependency on the scheme crate, so it is unit-tested alone (bilinear closed form, graded
  columns, shape errors).
- `lib.rs` is the scheme: phases, the pcurve extraction (`side_uvs`), the orientation check.

## Cost and parallelism

`generate` is O(ni * nj): one pass over the interior for the blend, one `eval_many` (or
projection) call, one pass for elements. All buffers are flat `Vec`s sized once (`[f64; D]`
rows, `NodeKey`s, `u32` indices); there is no per-element allocation. Parallelism comes from
the driver (jobs of one wave run concurrently); inside one face the work is sequential, which
keeps the projection hint chain and the results independent of thread count. No bench is
included: T5 names no hot path, and a 100 x 100 face is ~10^4 evaluations.

## Known limits

- Faces with a collapsed side (a pole: sphere, cone apex) have fewer than four real corners and
  are `NotApplicable`; they need a polar scheme.
- No parameters (no bias/hints); grading comes from the curve schemes through the boundary.
- The blend is transfinite, not elliptic: strongly concave or highly twisted boundaries can fold
  (detected, reported as a quality failure so the driver falls back to another scheme).
