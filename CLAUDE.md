# smart-mesh: rules for every agent

Read `docs/ARCHITECTURE.md` (design), `docs/PLAN.md` (how work is split), `docs/PARITY.md` (scope).

## Ownership
- You own only the directory your task names (usually `crates/<name>/`). Do not edit other crates, the root `Cargo.toml`, CI, or docs outside your crate.
- `crates/*-api` and `crates/sm-testkit` are frozen contracts that only the orchestrator edits. If you need a change, stop and return `status: blocked_on_api` with an `api_change_request` (what, why, proposed signature). Do not work around a contract with duplicated types or `unsafe` casts.
- Commit your work and report the commit SHA; the SHA, not the branch name, is what the orchestrator merges.
- Work only inside your own worktree. Never run git commands in, or cd into, the main checkout (the repo root outside `.claude/worktrees/`).
- In a worktree, never check out an existing branch name (it may be checked out in another worktree). Use `git checkout --detach <sha>`, commit, then `git branch <new-unique-name>` at your commit. Run git commands one at a time in simple form.

## Determinism (binding)
- Never let `HashMap`/`HashSet` iteration order affect output. Use `IndexMap`, `BTreeMap`, or sort first.
- Parallel reductions must be order-independent or done in a fixed order. Output must be bit-identical for 1 thread and N threads.
- Any randomness uses an RNG explicitly seeded from inputs. No time-based or thread-based seeds.
- Every crate has a determinism test: run twice, and with `RAYON_NUM_THREADS=1` vs default, then compare hashes.

## Definition of done
`cargo build`, `cargo test -p <crate>`, `cargo clippy -p <crate> -- -D warnings`, determinism test passing, property tests from `sm-testkit` where they apply, and rustdoc on every public item. Numerical code gets tests against analytic results, not only "doesn't crash".

## Style
- Rust edition 2024. No `unwrap()` in library code paths that inputs can reach; return typed errors (`thiserror`).
- Performance matters: avoid per-element heap allocation in hot loops, prefer SoA/CSR layouts, and use `rayon` for data parallelism.
- Keep comments short and useful.
