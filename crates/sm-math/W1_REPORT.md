# W1 report: sm-math

## Summary

Implemented `crates/sm-math` per the task:

- `predicates`: `orient2d`, `orient3d`, `incircle`, `insphere` with Shewchuk's stage-A filter, stage B and an
  exact expansion fallback; `*_det` variants; `try_*` range-checked variants.
- `exact`: `two_sum`/`two_diff`/`fast_two_sum`/`two_product` (Dekker, no FMA), public `Expansion`
  arithmetic, `det_exact`.
- `sos`: `orient2d_sos`, `orient3d_sos`, `incircle_sos`, `insphere_sos` (Edelsbrunner-Mücke, uniform
  across all four, point ids supplied by caller).
- `interval`: `Interval` with exact-error-term directed bounds for `+ - *`, one-ulp widening for
  `/` and `sqrt`.
- `roots`: `brent`, `newton_bisect` (default tolerance is bracket-relative with an absolute floor, so
  roots at exactly 0 converge).
- `linalg`: `solve3`/`solve4`/`inverse3`/`det3`/`det4`, `sym3_eigen` (Jacobi), `svd3` (one-sided Jacobi).
- `sum`: `KahanSum`, `NeumaierSum`, deterministic `par_sum*`/`par_reduce` helpers.
- `DESIGN.md` (positioning + design), `benches/kernels.rs` (criterion).

`sm-core`'s `Point3`, `Vector3`, `Transform` are used and re-exported; nothing duplicated.

## Test summary

All pass in debug, with `RAYON_NUM_THREADS=1`, and clippy/doc are clean:

```
cargo build
cargo test -p sm-math                          # 58 tests across 9 integration files
RAYON_NUM_THREADS=1 cargo test -p sm-math
cargo clippy -p sm-math --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc -p sm-math --no-deps
```

- `tests/predicates.rs`: exact `i128` oracle (independent of the expansion code). All ordered grid
  triples (6x6), quadruples (3x3x3 orient3d, 5x5 incircle), 200k random insphere 5-tuples from a
  3x3x3 grid, all 12-point cocircular and 30-point cospherical lattice sets, 60k one-unit nudges of
  collinear/coplanar/cocircular/cospherical lattice data scaled by `2^k`, Shewchuk's
  `(0.5 + i*2^-53, 0.5 + j*2^-53)` figure (naive evaluation provably wrong somewhere in it, every
  cell correct here), full-mantissa `orient2d`.
- `tests/exact.rs`, `tests/invariants.rs`: error-free transformations vs `i128`; ring identities
  for expansions; permutation parity of all four predicates; exact transforms (scale, rotation,
  translation, mirror) using `sm_core::Transform`.
- `tests/sos.rs`: oracle = independent sparse-polynomial expansion of the perturbed determinant
  using global ids; agreement for all four predicates on degenerate grids with shuffled ids;
  antisymmetry; duplicate-id error; brute-force SoS-Delaunay of 2D grids (5 shapes) and 3D grids (3
  shapes) must tile the hull exactly (areas/volumes sum, interior edges/faces matched).
- `tests/interval.rs`, `tests/roots.rs`, `tests/linalg.rs`, `tests/sum.rs`: see DESIGN.md.
- `tests/determinism.rs`: whole workload hashed via `sm_testkit::determinism::check` (2 runs, 4
  threads, concurrent callers) and compared for pools of 1, 4 and 7 rayon threads.

Benchmarks (release, this container; `cargo bench -p sm-math --bench kernels`): filtered
`orient2d` 4 ns, `orient3d` 11 ns, `incircle` 9 ns, `insphere` 36 ns; exactly degenerate
`orient2d` 0.2 us, `orient3d` 0.4 us, `incircle` 0.8 us, `insphere` 3.3 us, `insphere_sos` 15 us;
1e6-element sum: naive 1.3 ms, Neumaier 2.7 ms, 4-core `par_sum` 0.9 ms.

## Deviations from the contracts' intent

- **Predicates are A + B + full exact, not Shewchuk's four-stage adaptive scheme.** Stage B is
  implemented (fix0); stages C/D (tail corrections) are not. Correctness is
  identical; near-degenerate-but-nonzero inputs that stage B would resolve cheaply instead pay the
  full exact evaluation (0.2-3 us). Allocation: the filter path is allocation-free. The exact
  fallback of `orient2d` fits the 24-component inline `SmallVec` of `Expansion`, but every other
  exact fallback (`orient3d`, `incircle`, `insphere`) spills to the heap (intermediate products have
  tens to hundreds of components), and so does the SoS tie-break (one flat `Vec<Expansion>` per call). Fine for
  the rare exact path, not for tight per-element loops that hit exact degeneracy constantly.
- **Documented input range** for exactness: coordinates `0` or `2^-100 <= |x| <= 2^100`.
- **SoS lifted coordinate** is perturbed independently (infinitesimal weights), not along the
  paraboloid as in the original Edelsbrunner-Mücke incircle derivation. Consequence: a regular
  triangulation of the perturbed points; still consistent with `orient*_sos` (verified by the
  tiling tests). Ids must be consistent across all calls within one dimension.
- **Intervals** have no transcendental functions (platform `libm` is not correctly rounded).
- 2D predicates take `[f64; 2]`: `sm-core` has no `Point2`.

## Known gaps

- No Shewchuk stages C/D (above); the stage-B speedup has not been benchmarked.
- Plain (non-`try_`) predicates return a wrong nonzero sign for subnormal coordinates (documented).
- `Interval` lacks `exp`/`ln`/trig; `powi` is repeated multiplication (loose for wide intervals).
- `svd3` / `sym3_eigen` are 3x3 only; no 2x2/4x4 SVD, no general `N` eigen.
- Exactness is not guaranteed outside the stated coordinate range (no error is raised by the plain
  functions; use `try_*`).
- Parallel sums are deterministic per chunk length; not chunk-length independent (an
  order-independent exact accumulator was out of scope).
- No benchmark comparison against `predicates.c` / `robust` (neither is a workspace dependency).

## Workspace side effects

- `Cargo.lock` changed (new deps: rayon and its crossbeam deps, criterion and its deps; smallvec bumped
  1.16.3 -> 1.16.4; proptest was already present). Per decisions.md the orchestrator regenerates it at merge.
- Branch: the task text names `claude/w1-sm-math`; the session was assigned
  `claude/w1-sm-math-qk2k73`, which is where the work is pushed.

## api_change_requests

None blocking. Optional, non-frozen additions to consider for `sm-core` (additive):

1. crate: `sm-core`
   what: a `Point2` / `Vector2` type (or documented decision that 2D code uses `[f64; 2]`).
   why: 2D predicates and loop code currently take `[f64; 2]`.
   proposed signature: `pub struct Point2 { pub x: f64, pub y: f64 }` with `From<[f64; 2]>`.
2. crate: `sm-core`
   what: `Mat3` alias (`[[f64; 3]; 3]`) matching `Transform::m`.
   why: `sm-math::linalg::Mat3` is defined there; other crates would need to depend on `sm-math` to
   name it. Not required.
   proposed signature: `pub type Mat3 = [[f64; 3]; 3];`

## fix0 (review findings)

Must fix:
- `sym3_eigen` and `svd3` now scale the input by an exact power of two (max |a_ij| into [1, 2)),
  run the Jacobi method, and scale eigenvalues / singular values back. The squared convergence
  quantities can no longer overflow or underflow. Tests at 1e+-150, 1e+-300 (and 1e160, 1e-170).

Should fix, done:
1. `Interval::around` with NaN and `hull_of(inf, inf)` / `(-inf, -inf)` now return `ENTIRE`.
2. `Interval::mid` is clamped into `[lo, hi]` (fixes `[5e-324, 5e-324]`).
3. `solve3/solve4/inverse3` equilibrate rows by powers of two (exact) and use a fixed tolerance
   `16 * EPSILON` on the equilibrated matrix, so `diag(1e20, 1, 1)` solves. `Singular.pivot` is
   reported in equilibrated units (documented). `inverse3` factors once (new private `Lu`).
4. `NeumaierSum::add` propagates non-finite terms like plain addition (`[1, inf]` gives `inf`).
5. `brent` uses `0.5*c - 0.5*b` for `xm` and a saturated span for `d`/`e`; the full-range
   bracket `[-1e308, 1.7e308]` works. The 200-iteration default is kept; the docs now say that
   higher-multiplicity roots (`x^3` needs about 150) may need a larger `max_iter`.
6. New tests: extreme-scale eigen/SVD, row-scaled solve, Interval invariants for non-finite
   constructor args, subnormal `mid`, Neumaier infinity, Brent full-range bracket.
7. Shewchuk stage B implemented for all four predicates: the exact determinant of the rounded
   differences (expansions of plain doubles, no tails), accepted when the B bound clears or when
   every difference was exact (all tails zero). Only otherwise does the full exact path run.
   Stages C/D (tail corrections, bound C) are still not implemented. New randomized differential
   test (20k near-degenerate cases per predicate vs a `det_exact` oracle). Speedups were not
   re-benchmarked in this pass.
8. All four predicates take the sign from the top expansion component (`signed_estimate`); the
   estimate only supplies the magnitude.
9. `det_exact` documents its square-matrix precondition with a `# Panics` section (signature
   unchanged, as it is public API).
10. SoS minors no longer allocate a `Vec<Vec<Expansion>>` per minor: determinants are expanded
    over index arrays (`det_minor`). Expansion temporaries may still spill to the heap.

Skipped: none. Not done beyond the above: stages C/D, and enlarging the inline expansion capacity.

## fix1 (review round 2)

1. MUST: `svd3` returned `NoConvergence` for identical-row (rank-1) input at ~28% of random
   inputs. Cause: exactly parallel columns (including the rounding-noise column) can never meet
   `|gamma| <= EPSILON*sqrt(alpha*beta)`. Fix: before the orthogonality test, a column with squared
   norm `<= EPSILON^2 * ||A||_F^2` is zeroed (the standard one-sided Jacobi guard; `U` is then
   completed as for any rank-deficient input). Tests in `tests/fix1.rs`: the reproducer at scales
   1e-300..1e254, plus a 30k identical-row fuzz asserting no errors, reconstruction and
   orthogonality.
2. `newton_bisect`: initial midpoint, `dx_old`, `dx` and the bracket-width test now use
   `0.5*a + 0.5*b` and the saturated `span`; wide-bracket tests added.
3. SoS: sorted matching order is cached per rank permutation (150 `OnceLock` slots), and the
   per-row `Vec`s are replaced by one flat `Vec<Expansion>` per call. Not re-benchmarked.
4. SoS fall-through now debug-asserts and returns the new `SosError::Unresolved` instead of a
   mislabelled `DuplicateId`.
5. Stale docs fixed (this report, `DESIGN.md`, `predicates.rs` module doc): A + B + full exact.
6. Subnormal wrong-sign example added to the `predicates` module docs with guidance to use
   `try_*`; behaviour unchanged.
7. `svd3` doc claim weakened to absolute `O(EPSILON*||A||)` accuracy; no hypot-style accumulation.
8. `DESIGN.md` notes the `Sign`/`Interval` name collision with `sm-geom-api`.

Skipped: SHOULD FIX item 4 (Shewchuk stages C/D and keeping exact fallbacks off the heap) is a
performance-only gap with no correctness impact; not attempted here.

## polish0 (review round 3)

1. BLOCKING: `svd3` still returned `NoConvergence` for ~3 in 1M finite inputs (generic full-rank
   and identical-column). Cause: one pair settled at `|gamma|/sqrt(alpha*beta)` of 1.0-1.3
   `EPSILON`, just above the skip threshold and inside the rounding error of the 3-term dot
   product; the resulting rotation had `c == 1`, `|t| ~ 1e-16`, only flipped the sign of `gamma`,
   and kept `rotated` true to the 60-sweep cap. Fix (class, not instance): the skip tolerance is now
   `4*EPSILON` (relative), and a rotation that cannot change either column (`c == 1` and
   `|t| * (sqrt(alpha/beta) + sqrt(beta/alpha)) <= EPSILON`; the column-ratio factor matters because the
   smaller column moves by `t` times the larger one) counts as not performed. Tests in
   `tests/polish0.rs`: the three reviewer matrices, a 50k default fuzz (full-rank, rank-1, rank-2,
   identical columns, identical rows; scales 2^[-400,400]) and a 1M `#[ignore]` fuzz
   (`cargo test --release -p sm-math --test polish0 -- --ignored`), all asserting no errors,
   reconstruction and orthogonality.
2. `svd3`: the flush ran before the orthogonality test and zeroed genuine small values of
   already-orthogonal pairs (`diag(1,1,1e-17)` gave `s3 = 0`). Order is now: orthogonal-enough
   -> keep both and continue; otherwise flush only if the pair needs a rotation and the smaller
   column has squared norm `<= EPSILON^2 ||A||_F^2 / 64`. Tests: `diag(1,1,1e-17)`,
   `diag(1,1e-9,1e-18)` return the diagonal to relative 1e-15 with finite `condition()`;
   `[[1,0,.5],[0,1,.5],[0,0,1e-16]]` has `s3 > 0`. The identical-row fuzz is rerun.
3. `tests/predicates.rs::range_edge_scaled_lattices`: collinear, coplanar, cocircular and
   cospherical lattice data at `2^k`, `k` in {-100..-96, 92..96}, with one-unit nudges, through
   `try_*` against the `i128` oracle (plus a just-outside rejection test). These nudge by one
   lattice unit (`2^k`), not by ulps, because the `i128` oracle cannot hold degree-5 terms at ulp
   resolution. Polish1 adds `range_edge_ulp_nudged_lattices`: +-1/+-2 ulp nudges (via `to_bits`)
   of one coordinate of degenerate data at the same `k`, checked against the same data through
   `try_*` at `k = 0` (power-of-two scaling is exact, so the sign must match). Coordinates that
   leave `[2^-100, 2^100]` are skipped (`try_*` rejects them). svd3 rustdoc `# Errors` now matches
   the code (`|t|*(sqrt(a/b)+sqrt(b/a))` no-op test, `tiny = EPSILON² ||A||_F² / 64`).
4. `sym3_eigen`: a rotation with `c == 1 && |t| <= EPSILON` only zeroes `a_pq` (no vector update);
   `a_pq` negligible against both diagonals is dropped; a stall guard stops sweeping when the
   off-diagonal norm stops decreasing at the rounding floor. Fuzzed like `svd3` (50k default, 1M ignored,
   including near-repeated spectra).
5. Performance gap (documentation only): Shewchuk stages C/D are still missing, and the exact
   fallbacks (all but `orient2d`) plus the SoS tie-break allocate on the heap. See `DESIGN.md`
   and the TODO below.

TODO: benchmark the predicates against Shewchuk's `predicates.c` on near-degenerate workloads
(nearly-cocircular/coplanar point sets, not only exactly degenerate ones) to decide whether stages
C/D and an allocation-free exact path are worth implementing.

Skipped: none.
