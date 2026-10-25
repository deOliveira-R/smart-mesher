# sm-mesh W2 report

## Polish wave

| # | item | status |
|---|------|--------|
| 1 | `contract_alloc` flake (1 stray allocation under load) | fixed (`geometry_and_walking_over_dyn_mesh_read_allocate_nothing`) |
| 2 | same flake, 2 allocations at line 72 | fixed (same test; duplicate of 1) |
| 3 | same flake, failed 1 of 3 isolated reruns | fixed (same test; duplicate of 1) |

Root cause: the counting allocator used one process-global `AtomicUsize`, so allocations made by
any other thread (the libtest harness main thread, result channel, output capture) landed inside the
measured region. Fix: the counter is now a `const`-initialised, destructor-free `thread_local`
`Cell`, so only the measuring thread's allocations count, and the workload runs once as a warm-up
before the measured pass. The self-check that a real allocation is seen is kept. Verified 60/60
passes with 4 concurrent loops (previously ~4 of 15 failed on the base commit).

Not done (needs orchestrator): none.
Changed goldens/hashes: none (test-only change).

Gate: `tools/ci.sh` exit 0; `RAYON_NUM_THREADS=1 cargo test -p sm-mesh --all-features` exit 0;
`cargo clippy -p sm-mesh --all-targets --all-features -- -D warnings` exit 0.
