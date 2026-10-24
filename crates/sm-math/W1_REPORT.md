# W1 report: sm-math

## Summary

Implemented `crates/sm-math` per the task:

- `predicates`: `orient2d`, `orient3d`, `incircle`, `insphere` with Shewchuk's stage-A filter and an
  exact expansion fallback; `*_det` variants; `try_*` range-checked variants.
- `exact`: `two_sum`/`two_diff`/`fast_two_sum`/`two_product` (Dekker, no FMA), public `Expansion`
  arithmetic, `det_exact`.
- `sos`: `orient2d_sos`, `orient3d_sos`, `incircle_sos`, `insphere_sos` (Edelsbrunner-Mücke, uniform
  across all four, point ids supplied by caller).
- `interval`: `Interval` with exact-error-term directed bounds for `+ - *`, one-ulp widening for
  `/` and `sqrt`.
- `roots`: `brent`, `newton_bisect`.
- `linalg`: `solve3`/`solve4`/`inverse3`/`det3`/`det4`, `sym3_eigen` (Jacobi), `svd3` (one-sided Jacobi).
- `sum`: `KahanSum`, `NeumaierSum`, deterministic `par_sum*`/`par_reduce` helpers.
- `DESIGN.md` (positioning + design), `benches/kernels.rs` (criterion).

`sm-core`'s `Point3`, `Vector3`, `Transform` are used and re-exported; nothing duplicated.

## Test summary

All pass in debug, with `RAYON_NUM_THREADS=1`, and clippy/doc are clean:

```
cargo build
cargo test -p sm-math                          # 57 tests across 9 integration files
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

- **Exact stage is filter + full exact, not Shewchuk's four-stage adaptive scheme.** Stages B-D
  (progressively tighter bounds reusing partial results) are not implemented. Correctness is
  identical; near-degenerate-but-nonzero inputs that stage B would resolve cheaply instead pay the
  full exact evaluation (0.2-3 us). Expansions live in `SmallVec` (24 inline components), so the
  exact path allocates only for the largest `insphere` cases.
- **Documented input range** for exactness: coordinates `0` or `2^-100 <= |x| <= 2^100`.
- **SoS lifted coordinate** is perturbed independently (infinitesimal weights), not along the
  paraboloid as in the original Edelsbrunner-Mücke incircle derivation. Consequence: a regular
  triangulation of the perturbed points; still consistent with `orient*_sos` (verified by the
  tiling tests). Ids must be consistent across all calls within one dimension.
- **Intervals** have no transcendental functions (platform `libm` is not correctly rounded).
- 2D predicates take `[f64; 2]`: `sm-core` has no `Point2`.

## Known gaps

- No Shewchuk stages B-D (above).
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
