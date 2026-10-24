# sm-math design

## Positioning

`sm-math` is similar to Shewchuk's `predicates.c` and the `robust` crate: the same
filtered-then-exact `orient2d` / `orient3d` / `incircle` / `insphere`, with the same sign
conventions and the same static error bounds. It differs in three ways. First, the exact stage is
a general expansion engine (`exact::Expansion`, with `add`, `mul`, `scale`, `compress`) instead of
hand-unrolled per-predicate code, which is what allows the second difference: a simulation of
simplicity module (`sos`) that turns each predicate into one that never returns zero, with a real,
documented perturbation, so Delaunay code needs no special cases for grids and other degenerate
inputs. Third, it bundles the numeric utilities every mesher crate needs next to the predicates:
interval arithmetic that needs no rounding-mode control, bracketed root finders, small dense linear
algebra (3x3 / 4x4 solves, Jacobi eigen and SVD) and compensated sums with fixed-order parallel
reductions. This gives one dependency, one set of platform-independent guarantees (plain IEEE-754
`+ - * /` in round-to-nearest, no FMA, no `unsafe`, no global state) and bit-identical output for any
thread count.

## Layout

| module | contents |
|---|---|
| `sign` | `Sign` (`Negative`/`Zero`/`Positive`) with `Neg` and `Mul` |
| `exact` | `two_sum`, `two_diff`, `fast_two_sum`, `two_product` (Veltkamp/Dekker split), `Expansion`, `det_exact` |
| `predicates` | `orient2d`, `orient3d`, `incircle`, `insphere` (+ `*_det` returning the sign-exact determinant, + `try_*` range-checked) |
| `sos` | `orient2d_sos`, `orient3d_sos`, `incircle_sos`, `insphere_sos` over `Tagged<P>` points |
| `interval` | `Interval` and its arithmetic |
| `roots` | `brent`, `newton_bisect` |
| `linalg` | `Mat3`/`Mat4`, `solve3`, `solve4`, `inverse3`, `det3`, `det4`, `sym3_eigen`, `svd3` |
| `sum` | `KahanSum`, `NeumaierSum`, `par_sum*`, `par_reduce` |

`Point3`, `Vector3` and `Transform` are `sm-core`'s and are re-exported; nothing is redefined.
2D predicates take `[f64; 2]` (there is no `Point2` in `sm-core`); 3D predicates take `Point3`.

## Predicates

Each predicate is the determinant of the matrix with rows `[coords.., lift, 1]` in argument order,
(lift = squared norm for the circle/sphere tests). Conventions match `predicates.c`:
`orient2d > 0` for counter-clockwise; `orient3d > 0` when `d` is below the counter-clockwise plane
`abc`; `incircle > 0` when `d` is inside; `insphere > 0` when `e` is inside given
`orient3d(a,b,c,d) > 0`.

Stages:

1. **Filter (Shewchuk's stage A).** Evaluate the determinant in `f64` together with the "permanent"
   (the same expression with absolute values). If `|det| > errbound * permanent`, with the
   standard bounds `(3+16e)e`, `(7+56e)e`, `(10+96e)e`, `(16+224e)e` and `e = 2^-53`, the sign is
   proved. The shortcut tests of `orient2d` (opposite-sign products) are kept.
2. **Exact.** Otherwise the *same polynomial* is evaluated with `Expansion` arithmetic on exact
   coordinate differences (`two_diff`). Expansions are non-overlapping sequences of doubles; add,
   scale and multiply are exact barring overflow/underflow, so the sign of the top component is the
   true sign. No intermediate adaptive stages (Shewchuk's B/C/D): see "Known gaps" in the report.
   Measured cost (release, x86-64): 4 ns / 11 ns / 9 ns / 36 ns per filtered
   `orient2d` / `orient3d` / `incircle` / `insphere`; exactly degenerate inputs 0.2 / 0.4 / 0.8 / 3 us.

Exactness range: every coordinate is `0` or has magnitude in `[2^-100, 2^100]` (see
`predicates` docs). That keeps the degree-5 insphere products clear of overflow and underflow. The
`try_*` variants check it and return `PredicateError::OutOfRange`.

Platform independence: only `+ - *` and comparisons in binary64 round-to-nearest-even. No `mul_add`
(Dekker splitting instead, because FMA availability differs between x86-64 and aarch64 and a fused
operation inside a transformation changes its error term), no extended precision, no
`-ffast-math`-style reassociation (Rust forbids it), no `unsafe`.

## Simulation of simplicity

Edelsbrunner & Mücke (1990) for all four predicates, uniformly. Each non-constant matrix entry
`(row r, column c)` is perturbed by its own infinitesimal `eps^(2^g)`, where `g` is the rank of
`(id_r, c)` in lexicographic order and `id_r` is the caller's unique, stable point id. Because the
determinant is multilinear in the entries, the perturbed determinant is a sum over *partial
permutation matrices* `S` (cells with distinct rows and columns); the coefficient of `S` is the
signed minor of the unperturbed matrix with `S`'s rows/columns removed, and monomials are ordered
by the binary number `sum 2^g`. The sign is that of the lowest monomial with a nonzero
coefficient; all minors are evaluated exactly with `Expansion`. A maximal `S` has coefficient
`+-1`, so the procedure always terminates with a nonzero sign for distinct ids.

The lifted coordinate of `incircle`/`insphere` is perturbed by its own infinitesimal, ordered after
the spatial ones, i.e. an infinitesimal weight. The x/y/z perturbations are shared between
`orient*` and `incircle`/`insphere`, so the predicates are mutually consistent: the induced
triangulation is a regular triangulation of the perturbed points (Delaunay whenever the input is
in general position). Tests build the SoS-Delaunay triangulation of grids in 2D and 3D by brute
force and verify it tiles the hull exactly.

Fast path: one plain exact predicate call; the tie-break only runs on exactly zero. Cost on the
tie-break path (~15 us for `insphere`) is dominated by exact minors.

## Intervals

Round-to-nearest plus the exact error term of the operation (from `two_sum`/`two_product`) says on
which side of the computed value the true result lies, so only that bound moves one ulp outward:
identical to directed rounding for `+ - *` (and so `[2,2]+[3,3] = [5,5]`). Division, `sqrt` and
results outside `[2^-900, 2^900]` are widened one ulp on both sides (documented conservative
widening). No transcendentals (their `libm` implementations are platform-dependent and not
correctly rounded). `0 * inf = 0`; `lo<=hi`, never NaN; division by an interval containing zero is
`ENTIRE` (`checked_div` returns an error).

## Roots

`brent`: classical zeroin (IQI / secant / bisection) with a hard iteration cap, a bracket check by
sign (no `f(a)*f(b)` overflow), non-finite detection and a tolerance
`x_tol + 2*EPSILON*|x|`. `newton_bisect`: rtsafe-style, falls back to bisection when Newton leaves
the bracket, converges too slowly, or the derivative is zero/non-finite.

## Linear algebra

Partial-pivot elimination with a relative singularity test (`16*EPSILON*max|a_ij|`), so uniformly
scaled matrices behave identically. `sym3_eigen`: cyclic Jacobi, fixed pair order, ascending
eigenvalues, sign-canonical eigenvectors (largest component positive). `svd3`: one-sided Jacobi
(Hestenes), never forms `A^T A`, completes `U` for rank-deficient input.

## Summation

`NeumaierSum` (and `KahanSum`). `par_*` helpers fix the reduction tree by chunk length (a parameter
or `DEFAULT_CHUNK = 4096`, never the thread count): chunk partials are computed independently,
collected in chunk order, merged sequentially. Output is bit-identical for 1 and N threads and
depends only on the chunk length.

## Tests

- Exact `i128` oracle independent of the crate's expansion code (lattice coordinates scaled by
  `2^k`): exhaustive grid triples/quadruples, cocircular/cospherical lattice sets, one-unit nudges,
  Shewchuk's near-degenerate `orient2d` grid, full-mantissa `orient2d`.
- Algebraic tests of the expansion engine (ring identities hold only if every op is exact) and
  error-free transformations against `i128`.
- Proptest: permutation parity of all four predicates, exact transforms (scaling, translation,
  proper rotation, mirror flips sign), repeated points.
- SoS: independent symbolic polynomial oracle, antisymmetry, never zero, equality with the exact
  sign when nonzero, and grid Delaunay tiling in 2D and 3D.
- Intervals: enclosure of exact results (via expansions), tightness for exact ops.
- Linalg/roots against analytic results (rotated diagonals, cuboid inertia tensor, rank-deficient
  SVD, SVD vs Gram-matrix eigenvalues, `x^2-2`, `cos x = x`, odd and double roots).
- Determinism: workload hashed for 1 vs N rayon threads, repeated, and concurrent callers
  (`sm_testkit::determinism::check`).
