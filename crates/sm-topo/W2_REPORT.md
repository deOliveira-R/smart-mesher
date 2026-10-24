# sm-topo W2 report (T13)

## Summary
Decision 36 / T13. The timing test lives in `tests/perf.rs` (the plan's `ref_reference.rs` path does not exist). It already times `TopoStore` directly (no `Dual`/`Twin`, paranoid off), so no change to it was needed. Added the missing correctness variant. No store behaviour or existing test/hash changed.

## Acceptance -> test
| Bullet | Test |
|---|---|
| Timing test passes in release under 2 s | `perf::building_many_bricks_is_near_linear` (10 000 bricks, 0.33 s release in this VM; debug build uses 1 000 and skips the assert) |
| 200-brick differential variant for correctness | `perf::building_200_bricks_matches_the_reference` (reference `CellComplex` vs paranoid `TopoStore` via `Twin`, full state and `validate_par` compared) |

## Timing
`cargo test -p sm-topo --release --all-features`: ~16 s wall on a warm build (incremental compile included; this VM, not the M1). The perf test itself is 0.33 s. The 2 s budget was not measured on the M1.

## Deviations / gaps / api_change_requests
None. Note: the plan names `tests/ref_reference.rs`; the test is actually in `tests/perf.rs`.
