# W4-INT benchmark smoke (plan.md section 5 item 4)

> **SMOKE RESULT.** Produced on macOS arm64 (Apple M1 Max, 10 logical CPUs, 16 GiB Docker VM), under a heavily
> loaded host, not on bare-metal Linux x86_64 with the performance governor (ruling Q9). It shows that the
> harness runs end to end and that every tracker agrees with the sm-rt facet twin. It is not a published table.
> **No ratio between trackers is stated or implied anywhere in this file.**
>
> **x86_64 table: pending the user's run.** The user has not yet run the three sessions of
> `docs/design/w4/bench-x86.md` (script `tools/bench/x86_64.sh`, ruling D6). Nothing here stands in for them, and
> arm64 and x86_64 results are never merged. The pre-registered verdict (`crates/sm-rt-bench/VERDICT.md`) is computed
> from the three x86 sessions only.

## What was run

- Code: `714e2af` (code-identical to `main` `b51806f`; `git diff b51806f 714e2af` is `docs/design/w4/rulings.md` only).
  `cargo build --release -p sm-rt-bench`.
- Command (UTC 2026-11-16 04:34, output dir outside the repository):

  ```sh
  target/release/smrt-bench run --cases P1,H1,P2 --scale reduced --trackers smrt,dagmc-moab,dagmc-dd,xdg \
      --threads 1,4 --out <scratchpad>/smoke-results
  ```

  Defaults: 7 repetitions, 3 warmups, 10000 rays, workloads track, fire, fire-locate, flight, walk (walk only where a
  tet mesh exists and only on trackers that have it). `report` was run on the same directory (`w4int-bench-smoke-report.md` in the session scratchpad); the tables below are generated from the result JSON
  files with the same rows, regrouped (facet/analytic/hierarchy and competitor rows; rows on tet meshes; polyhedral rows). **Nothing under `crates/sm-rt-bench/results/` is committed** (it holds no files).
- Result files: `P1-...-0.json` (36 rows), `H1-...-1.json` (46 rows), `P2-...-2.json` (42 rows); result hashes
  `823011d7c45d`, `831abe336938`, `46742fa28561` (not blessed values, only recorded).
- Host load: load average 114 / 39.8 / 18.9 (1, 5, 15 min) at the start of the run (four agents were building;
  the run itself finished in about 30 s). Timings below carry that noise. The dagmc-dd rows in particular have min/max
  spreads of a factor of 2 to 10 inside one row. Do not read an ordering from them.
- Docker Desktop VM: 10 CPUs, 15.6 GiB, kernel 7.0.14-linuxkit, server 29.9.3. Cores cannot be pinned there; every competitor
  row carries `cores not pinned`, and the M1 Max has 8P+2E cores.

## Image identities (recorded in the result files' `images` map)

| image | id the rows ran on | platform | base index digest | sources | MOAB | conda lock sha256 | versions.txt sha256 |
|---|---|---|---|---|---|---|---|
| dagmc-moab | `sha256:cb2f193217504d3a2089c17f6b2f934c6b18ff125fb49ab3a61eab4d1516eae0` | linux/arm64 | `sha256:937dc1e8ab9f...f3fad` | conda-forge only (dagmc 3.2.4 nodoubledown) | 5.6.0 | `d0e68e3f0c8f2d8b595551c7091db7561a920dbe4bca711d8cb55e81916bd5b7` | `1a588c45d801f23b820e98efdc0f29b1f2e8dd8e12fa13c0dda9ccfec5c01351` |
| dagmc-dd | `sha256:11f9571c052cb4da508f5a1362e0947cb4092c7bf378c6169dbe43b99f3fdfbd` | linux/arm64 | `sha256:937dc1e8ab9f...f3fad` | double-down `437ad209eac5869e40c73a1c5960c821f4eabb13`; DAGMC v3.2.4 (source build) | 5.6.0 | `d86f8f82e4ef87a7012f5b2f3e673627525951a5e0b1909df8859be2ffd61b49` | `38cd8aec004c7138065840ad980cb4fefc8966fc20161c578dc5da7794222e68` |
| xdg | none (no image) | linux/arm64 | UBUNTU `fe81560990fa...ef852` | pin `a5d67e635b96` | 5.6.0 | apt, recorded not frozen | n/a |

Drivers (`docker-check`): `dagmc_driver backend=moab-obb dagmc=DagMC version 3.2` and
`dagmc_driver backend=double-down dagmc=DagMC version 3.2`. Full base index digest:
`sha256:937dc1e8ab9ffc5f388f0d2a1ce5d24ba5b15850771571d7442fac1af53f3fad`. Locks: `docker/locks/dagmc-moab.linux-aarch64.txt`,
`docker/locks/dagmc-dd.linux-aarch64.txt`.

Note on ids. After the smoke run I ran `smrt-bench docker-check` (to record the xdg failure). The two shared tags `smrt-bench-dagmc-moab` and `smrt-bench-dagmc-dd` now resolve to other ids than during the run
(`0e639a4317226805...` and `c71415aee38ad32f...`, both with creation date 2026-11-09); `docker-check` rebuilds with `docker build`, so it probably re-tagged them from the layer cache, but I did not
verify that nor rule out another session re-tagging them. The conda lock hashes, versions hashes and driver `--version` lines that `docker-check` printed for them equal those stored in the result files. The rows
above ran on the ids in the table, which are the ones stored in the result files. Docker images were not pruned.

## xdg: Partial (ruling OR-d)

`xdg` is x86-only. On this arm64 host `docker-check` reports `smrt-bench-xdg: Failed(...)` (the `docker build` of
`docker/xdg/Dockerfile` at pin `a5d67e635b96` exits with code 2 in the XDG `clone, cmake, build` step, consistent with the documented
`<immintrin.h>` include in `include/xdg/vec3da.h` (not confirmed from a compiler line); the `Failed` message of this run carries only the exit code, not the last two compiler lines that `crates/sm-rt-bench/DESIGN.md` says it carries: a deviation of the harness from its
documentation, not investigated here, and I did not reproduce the compiler error by hand). `run --trackers ...,xdg` therefore records, per case, `partial: xdg: docker image inspect
smrt-bench-xdg failed: the image is not built`, exit code 2, and writes no xdg row. XDG rows (fire, tet walk) exist only on the x86_64 table.
`ImageStatus` for xdg is therefore not recorded in the `images` map here.

## Agreement and fairness checks (not timings)

- Every competitor row: `dagmc-moab` and `dagmc-dd` fire and fire-locate agree with the sm-rt facets fire on 10000 of
  10000 rays in all three cases (0 grazing, 0 failed). Flight tallies (8 slots P1, 12 H1, 6 P2) agree at 4 sigma with 0 failed.
  Region sequences of `dagmc-moab` and `dagmc-dd` are equal on 10000 of 10000 rays in every case.
- Over all 124 rows: lost 0, agreement failures 0. The exit code was not captured; per `cli.rs` a `Partial` competitor sets it to 2, so expect 2 here.
- sm-rt, H1 `jump` vs cell `walk` W-fire: max delta d 3.411e-12 cm over 10000 rays (run tolerance 3.464e-7; face tests per
  fire 59.8 for the cell walk against 10.7 for the region jump, 10000 jumps taken).
- Tet-mesh rows present: H1 (its own tets): `block`, `delta`, `walk` (fire, fire-locate, flight and the `w-walk` row), `jump` and the polyhedral
  rows; P1 (the stair-stepped Kuhn twin of the scene): `block`, `delta` and `walk` flight rows only. The conforming P1 tet twin is not
  produced (`crates/sm-rt-bench/DESIGN.md`, "Conforming P1 tet twin: not produced").
- Rows not produced, with the harness's own reasons: H1 `analytic` (a box inside a solid needs a union of half-spaces in
  sm-rt); P2 flight rows on tets (the pins do not fill their box, so P2 has no tet twin).
- The `report` text also prints "Verdict" sections with per-cell signs (they include an `H1/w-walk/walk` cell, `incomplete` for lack of an xdg baseline row at 1 thread). They are
  harness checks under the SMOKE banner (`report` marks them "not a published verdict") and are deliberately not copied here.

## Tables

Timings are rays/s as the harness reports them (median of 7 repetitions, with min and max). `tracker` `smrt` rows are
sm-rt at `714e2af`. Competitor rows ran in the Docker VM; the others on the host.
Under the load given above the second significant digit means nothing.

### P1

36 rows, result_hash `823011d7c45d326b`, scale reduced.

| tracker | mode | workload | thr | rays/s median [min, max] | lost | agreement failed/checked |
|---|---|---|---|---|---|---|
| dagmc-dd | embree | w-fire | 1 | 9.465e+05 [5.785e+05, 2.11e+06] | 0 | 0/10000 |
| dagmc-dd | embree | w-fire | 4 | 1.379e+06 [7.01e+05, 1.555e+06] | 0 | 0/10000 |
| dagmc-dd | embree | w-fire-locate | 1 | 7.36e+05 [5.915e+05, 1.388e+06] | 0 | 0/10000 |
| dagmc-dd | embree | w-fire-locate | 4 | 7.002e+05 [5.328e+05, 1.124e+06] | 0 | 0/10000 |
| dagmc-dd | embree | w-flight | 1 | 8.179e+05 [2.824e+05, 8.392e+05] | 0 | 0/8 |
| dagmc-dd | embree | w-flight | 4 | 2.726e+06 [2.671e+06, 3.011e+06] | 0 | 0/8 |
| dagmc-dd | embree | w-track | 1 | 8.6e+05 [2.188e+05, 9.437e+05] | 0 | 0/10000 |
| dagmc-dd | embree | w-track | 4 | 2.906e+06 [1.718e+05, 3.643e+06] | 0 | 0/10000 |
| dagmc-moab | obb | w-fire | 1 | 2.94e+05 [2.89e+05, 2.972e+05] | 0 | 0/10000 |
| dagmc-moab | obb | w-fire | 4 | 1.138e+06 [1.119e+06, 1.149e+06] | 0 | 0/10000 |
| dagmc-moab | obb | w-fire-locate | 1 | 1.316e+05 [1.304e+05, 1.339e+05] | 0 | 0/10000 |
| dagmc-moab | obb | w-fire-locate | 4 | 4.92e+05 [4.449e+05, 4.969e+05] | 0 | 0/10000 |
| dagmc-moab | obb | w-flight | 1 | 9.779e+04 [9.678e+04, 9.849e+04] | 0 | 0/8 |
| dagmc-moab | obb | w-flight | 4 | 3.668e+05 [3.644e+05, 3.745e+05] | 0 | 0/8 |
| dagmc-moab | obb | w-track | 1 | 1.121e+05 [1.108e+05, 1.145e+05] | 0 | 0/10000 |
| dagmc-moab | obb | w-track | 4 | 4.277e+05 [3.374e+05, 4.493e+05] | 0 | 0/10000 |
| smrt | analytic | w-fire | 1 | 4.778e+06 [4.744e+06, 4.842e+06] | 0 | 0/0 |
| smrt | analytic | w-fire | 4 | 4.814e+06 [4.728e+06, 4.984e+06] | 0 | 0/0 |
| smrt | analytic | w-fire-locate | 1 | 3.477e+06 [3.456e+06, 3.509e+06] | 0 | 0/0 |
| smrt | analytic | w-fire-locate | 4 | 3.487e+06 [3.471e+06, 3.492e+06] | 0 | 0/0 |
| smrt | analytic | w-flight | 1 | 1.64e+06 [1.63e+06, 1.654e+06] | 0 | 0/0 |
| smrt | analytic | w-flight | 4 | 5.723e+06 [5.173e+06, 6.11e+06] | 0 | 0/0 |
| smrt | analytic | w-track | 1 | 1.468e+06 [1.444e+06, 1.475e+06] | 0 | 0/10000 |
| smrt | analytic | w-track | 4 | 4.397e+06 [4.258e+06, 4.561e+06] | 0 | 0/10000 |
| smrt | facets | w-fire | 1 | 1.294e+06 [1.284e+06, 1.298e+06] | 0 | 0/0 |
| smrt | facets | w-fire | 4 | 1.307e+06 [1.293e+06, 1.314e+06] | 0 | 0/0 |
| smrt | facets | w-fire-locate | 1 | 5.51e+05 [5.362e+05, 5.527e+05] | 0 | 0/0 |
| smrt | facets | w-fire-locate | 4 | 5.507e+05 [5.195e+05, 5.515e+05] | 0 | 0/0 |
| smrt | facets | w-track | 1 | 2.703e+05 [2.663e+05, 2.744e+05] | 0 | 0/10000 |
| smrt | facets | w-track | 4 | 9.736e+05 [9.049e+05, 9.998e+05] | 0 | 0/10000 |

P1 rows on a tet mesh (`block`, `delta`, `walk`, `jump`; stair-stepped tets, not the facet twin the competitors hold, so they render apart):

| tracker | mode | workload | thr | rays/s median [min, max] | lost | agreement failed/checked |
|---|---|---|---|---|---|---|
| smrt | block | w-flight | 1 | 1.381e+06 [1.376e+06, 1.385e+06] | 0 | 0/4 |
| smrt | block | w-flight | 4 | 4.693e+06 [4.553e+06, 4.903e+06] | 0 | 0/4 |
| smrt | delta | w-flight | 1 | 9.19e+05 [9.113e+05, 9.22e+05] | 0 | 0/4 |
| smrt | delta | w-flight | 4 | 3.27e+06 [3.19e+06, 3.351e+06] | 0 | 0/4 |
| smrt | walk | w-flight | 1 | 2.708e+06 [2.685e+06, 2.74e+06] | 0 | 0/0 |
| smrt | walk | w-flight | 4 | 9.308e+06 [8.847e+06, 9.744e+06] | 0 | 0/0 |

### H1

46 rows, result_hash `831abe3369386cdc`, scale reduced.

| tracker | mode | workload | thr | rays/s median [min, max] | lost | agreement failed/checked |
|---|---|---|---|---|---|---|
| dagmc-dd | embree | w-fire | 1 | 3.962e+06 [3.72e+06, 4.197e+06] | 0 | 0/10000 |
| dagmc-dd | embree | w-fire | 4 | 9.19e+06 [8.595e+06, 1.237e+07] | 0 | 0/10000 |
| dagmc-dd | embree | w-fire-locate | 1 | 1.372e+06 [1.329e+06, 1.448e+06] | 0 | 0/10000 |
| dagmc-dd | embree | w-fire-locate | 4 | 4.332e+06 [4.281e+06, 4.516e+06] | 0 | 0/10000 |
| dagmc-dd | embree | w-flight | 1 | 1.21e+06 [1.183e+06, 1.273e+06] | 0 | 0/12 |
| dagmc-dd | embree | w-flight | 4 | 4.174e+06 [3.976e+06, 4.442e+06] | 0 | 0/12 |
| dagmc-dd | embree | w-track | 1 | 3.041e+06 [2.93e+06, 3.059e+06] | 0 | 0/10000 |
| dagmc-dd | embree | w-track | 4 | 8.17e+06 [7.922e+06, 1.107e+07] | 0 | 0/10000 |
| dagmc-moab | obb | w-fire | 1 | 7.477e+05 [7.181e+05, 7.504e+05] | 0 | 0/10000 |
| dagmc-moab | obb | w-fire | 4 | 2.649e+06 [2.516e+06, 2.902e+06] | 0 | 0/10000 |
| dagmc-moab | obb | w-fire-locate | 1 | 3.108e+05 [3.074e+05, 3.149e+05] | 0 | 0/10000 |
| dagmc-moab | obb | w-fire-locate | 4 | 1.18e+06 [1.151e+06, 1.193e+06] | 0 | 0/10000 |
| dagmc-moab | obb | w-flight | 1 | 2.277e+05 [2.237e+05, 2.286e+05] | 0 | 0/12 |
| dagmc-moab | obb | w-flight | 4 | 8.628e+05 [8.486e+05, 8.737e+05] | 0 | 0/12 |
| dagmc-moab | obb | w-track | 1 | 6.341e+05 [6.196e+05, 6.419e+05] | 0 | 0/10000 |
| dagmc-moab | obb | w-track | 4 | 2.263e+06 [2.07e+06, 2.363e+06] | 0 | 0/10000 |
| smrt | facets | w-fire | 1 | 8.545e+05 [8.495e+05, 8.831e+05] | 0 | 0/0 |
| smrt | facets | w-fire | 4 | 8.445e+05 [7.891e+05, 8.577e+05] | 0 | 0/0 |
| smrt | facets | w-fire-locate | 1 | 3.439e+05 [3.347e+05, 3.473e+05] | 0 | 0/0 |
| smrt | facets | w-fire-locate | 4 | 3.472e+05 [3.403e+05, 3.499e+05] | 0 | 0/0 |
| smrt | facets | w-track | 1 | 2.596e+05 [2.576e+05, 2.676e+05] | 0 | 0/10000 |
| smrt | facets | w-track | 4 | 9.805e+05 [7.787e+05, 9.852e+05] | 0 | 0/10000 |

H1 rows on a tet mesh (`block`, `delta`, `walk`, `jump`; stair-stepped tets, not the facet twin the competitors hold, so they render apart):

| tracker | mode | workload | thr | rays/s median [min, max] | lost | agreement failed/checked |
|---|---|---|---|---|---|---|
| smrt | block | w-flight | 1 | 2.383e+05 [2.316e+05, 2.454e+05] | 0 | 0/7 |
| smrt | block | w-flight | 4 | 9.317e+05 [8.932e+05, 9.501e+05] | 0 | 0/7 |
| smrt | delta | w-flight | 1 | 2.366e+05 [2.268e+05, 2.407e+05] | 0 | 0/7 |
| smrt | delta | w-flight | 4 | 9.481e+05 [9.229e+05, 9.623e+05] | 0 | 0/7 |
| smrt | jump | w-fire | 1 | 9.562e+05 [8.666e+05, 9.975e+05] | 0 | 0/10000 |
| smrt | jump | w-fire | 4 | 9.516e+05 [9.282e+05, 9.919e+05] | 0 | 0/10000 |
| smrt | jump | w-fire-locate | 1 | 6.653e+05 [6.475e+05, 7.202e+05] | 0 | 0/10000 |
| smrt | jump | w-fire-locate | 4 | 6.881e+05 [6.51e+05, 6.938e+05] | 0 | 0/10000 |
| smrt | walk | w-fire | 1 | 1.042e+06 [1.004e+06, 1.068e+06] | 0 | 0/10000 |
| smrt | walk | w-fire | 4 | 1.031e+06 [9.877e+05, 1.089e+06] | 0 | 0/10000 |
| smrt | walk | w-fire-locate | 1 | 7.49e+05 [7.303e+05, 7.559e+05] | 0 | 0/10000 |
| smrt | walk | w-fire-locate | 4 | 7.45e+05 [7.256e+05, 7.541e+05] | 0 | 0/10000 |
| smrt | walk | w-flight | 1 | 1.205e+06 [1.183e+06, 1.295e+06] | 0 | 0/0 |
| smrt | walk | w-flight | 4 | 4.404e+06 [4.099e+06, 4.494e+06] | 0 | 0/0 |
| smrt | walk | w-walk | 1 | 6.165e+05 [5.98e+05, 6.332e+05] | 0 | 0/10000 |
| smrt | walk | w-walk | 4 | 1.958e+06 [1.525e+06, 1.974e+06] | 0 | 0/10000 |

H1 polyhedral rows (separate table, ruling D6: different mesh, same geometry; never read by the verdict):

| tracker | mode | workload | thr | rays/s median [min, max] | lost | agreement failed/checked |
|---|---|---|---|---|---|---|
| smrt | poly-ref | w-flight | 1 | 1.219e+06 [1.072e+06, 1.275e+06] | 0 | 0/0 |
| smrt | poly-ref | w-flight | 4 | 4.393e+06 [4.274e+06, 4.488e+06] | 0 | 0/0 |
| smrt | poly-ref | w-track | 1 | 6.081e+05 [5.718e+05, 6.249e+05] | 0 | 0/10000 |
| smrt | poly-ref | w-track | 4 | 1.929e+06 [1.886e+06, 1.954e+06] | 0 | 0/10000 |
| smrt | poly-walk | w-flight | 1 | 4.784e+04 [4.672e+04, 4.819e+04] | 0 | 0/7 |
| smrt | poly-walk | w-flight | 4 | 1.826e+05 [1.814e+05, 1.874e+05] | 0 | 0/7 |
| smrt | poly-walk | w-track | 1 | 3.627e+04 [3.599e+04, 3.642e+04] | 0 | 0/10000 |
| smrt | poly-walk | w-track | 4 | 1.406e+05 [1.399e+05, 1.416e+05] | 0 | 0/10000 |

### P2

42 rows, result_hash `46742fa28561dc7c`, scale reduced.

| tracker | mode | workload | thr | rays/s median [min, max] | lost | agreement failed/checked |
|---|---|---|---|---|---|---|
| dagmc-dd | embree | w-fire | 1 | 2.497e+06 [2.329e+06, 2.606e+06] | 0 | 0/10000 |
| dagmc-dd | embree | w-fire | 4 | 6.929e+06 [6.598e+06, 7.155e+06] | 0 | 0/10000 |
| dagmc-dd | embree | w-fire-locate | 1 | 1.328e+06 [1.319e+06, 1.342e+06] | 0 | 0/10000 |
| dagmc-dd | embree | w-fire-locate | 4 | 4.176e+06 [4.057e+06, 4.348e+06] | 0 | 0/10000 |
| dagmc-dd | embree | w-flight | 1 | 1.007e+06 [9.335e+05, 1.019e+06] | 0 | 0/6 |
| dagmc-dd | embree | w-flight | 4 | 3.402e+06 [3.327e+06, 3.422e+06] | 0 | 0/6 |
| dagmc-dd | embree | w-track | 1 | 1.024e+06 [9.957e+05, 1.093e+06] | 0 | 0/10000 |
| dagmc-dd | embree | w-track | 4 | 3.303e+06 [3.196e+06, 3.968e+06] | 0 | 0/10000 |
| dagmc-moab | obb | w-fire | 1 | 1.39e+05 [1.357e+05, 1.421e+05] | 0 | 0/10000 |
| dagmc-moab | obb | w-fire | 4 | 5.568e+05 [5.538e+05, 5.644e+05] | 0 | 0/10000 |
| dagmc-moab | obb | w-fire-locate | 1 | 6.474e+04 [5.153e+04, 6.562e+04] | 0 | 0/10000 |
| dagmc-moab | obb | w-fire-locate | 4 | 1.292e+05 [8.803e+04, 1.686e+05] | 0 | 0/10000 |
| dagmc-moab | obb | w-flight | 1 | 4.5e+04 [4.351e+04, 5.155e+04] | 0 | 0/6 |
| dagmc-moab | obb | w-flight | 4 | 2.025e+05 [1.829e+05, 2.092e+05] | 0 | 0/6 |
| dagmc-moab | obb | w-track | 1 | 5.684e+04 [5.46e+04, 5.767e+04] | 0 | 0/10000 |
| dagmc-moab | obb | w-track | 4 | 2.217e+05 [2.06e+05, 2.247e+05] | 0 | 0/10000 |
| smrt | analytic | w-fire | 1 | 4.838e+06 [4.773e+06, 5.06e+06] | 0 | 0/0 |
| smrt | analytic | w-fire | 4 | 4.897e+06 [4.887e+06, 4.915e+06] | 0 | 0/0 |
| smrt | analytic | w-fire-locate | 1 | 2.419e+06 [2.399e+06, 2.513e+06] | 0 | 0/0 |
| smrt | analytic | w-fire-locate | 4 | 2.426e+06 [2.486e+05, 2.431e+06] | 0 | 0/0 |
| smrt | analytic | w-flight | 1 | 1.355e+06 [9.945e+05, 1.388e+06] | 0 | 0/0 |
| smrt | analytic | w-flight | 4 | 3.982e+06 [3.82e+06, 4.6e+06] | 0 | 0/0 |
| smrt | analytic | w-track | 1 | 1.246e+06 [1.24e+06, 1.271e+06] | 0 | 0/10000 |
| smrt | analytic | w-track | 4 | 4.036e+06 [3.974e+06, 4.139e+06] | 0 | 0/10000 |
| smrt | analytic-hierarchy | w-fire | 1 | 3.731e+06 [3.654e+06, 3.897e+06] | 0 | 0/0 |
| smrt | analytic-hierarchy | w-fire | 4 | 3.736e+06 [4.019e+05, 3.748e+06] | 0 | 0/0 |
| smrt | analytic-hierarchy | w-fire-locate | 1 | 1.913e+06 [1.885e+06, 1.998e+06] | 0 | 0/0 |
| smrt | analytic-hierarchy | w-fire-locate | 4 | 1.919e+06 [1.639e+06, 1.925e+06] | 0 | 0/0 |
| smrt | analytic-hierarchy | w-track | 1 | 1.042e+06 [1.032e+06, 1.074e+06] | 0 | 0/10000 |
| smrt | analytic-hierarchy | w-track | 4 | 3.437e+06 [1.926e+06, 3.528e+06] | 0 | 0/10000 |
| smrt | facets | w-fire | 1 | 5.58e+05 [5.46e+05, 5.678e+05] | 0 | 0/0 |
| smrt | facets | w-fire | 4 | 5.733e+05 [1.89e+05, 5.801e+05] | 0 | 0/0 |
| smrt | facets | w-fire-locate | 1 | 2.557e+05 [2.518e+05, 2.581e+05] | 0 | 0/0 |
| smrt | facets | w-fire-locate | 4 | 1.954e+05 [1.554e+05, 2.576e+05] | 0 | 0/0 |
| smrt | facets | w-track | 1 | 5.775e+04 [5.653e+04, 5.88e+04] | 0 | 0/10000 |
| smrt | facets | w-track | 4 | 2.259e+05 [2.073e+05, 2.282e+05] | 0 | 0/10000 |
| smrt | hierarchy | w-fire | 1 | 1.294e+06 [1.25e+06, 1.308e+06] | 0 | 0/0 |
| smrt | hierarchy | w-fire | 4 | 1.296e+06 [1.291e+06, 1.297e+06] | 0 | 0/0 |
| smrt | hierarchy | w-fire-locate | 1 | 5.095e+05 [4.965e+05, 5.228e+05] | 0 | 0/0 |
| smrt | hierarchy | w-fire-locate | 4 | 4.948e+05 [2.555e+05, 5.218e+05] | 0 | 0/0 |
| smrt | hierarchy | w-track | 1 | 3.444e+05 [3.289e+05, 3.52e+05] | 0 | 0/10000 |
| smrt | hierarchy | w-track | 4 | 1.237e+06 [5.549e+05, 1.247e+06] | 0 | 0/10000 |


## sm-rt rays/s before and after W4-RT-TIDY (P2 and H1; W4-RT-TIDY never measured it)

Question: did W4-RT-TIDY (graze/hop changes in `crates/sm-rt/src/{graze,tracker,world}.rs` and `delta/{flight,jump}.rs`,
merged in `cb6d6a4`) slow sm-rt down? Method:

- **Before** = `33efd2c` (first parent of the RT-TIDY merge `cb6d6a4`; it is `Merge W4-BOOL-TIDY`, the W4c combine
  state just before RT-TIDY). **After** = `main` `b51806f` (the binary was built at `714e2af`, which differs from `b51806f` in
  `docs/design/w4/rulings.md` only).
- `git diff --stat 33efd2c b51806f -- crates/sm-rt crates/sm-rt-bench crates/sm-rt-api` touches only `crates/sm-rt`
  (8 files); `sm-rt-bench`, the case generator and the harness are byte-identical between the two, so the difference is sm-rt alone.
- Release build of `smrt-bench` at each commit (one target dir, binaries copied aside), `run --cases P2,H1 --scale reduced
  --trackers smrt --threads 1,4` (7 repetitions, 3 warmups, 10000 rays, default workloads), **3 runs each, interleaved**
  (base, main, base, main, base, main) so slow drift of the host load hits both equally.
- Each cell below is the median over the 3 runs of the per-run median; in brackets the min and max of the 3 run medians,
  and in parentheses the run-to-run spread (max - min) / median.
- Host load average (1, 5, 15 min), before and after each run (`uptime`): base1 27.96 29.67 19.38 -> 23.52 28.56 19.22;
  main1 -> 19.50 27.43 18.98; base2 -> 16.12 26.29 18.72; main2 -> 13.64 25.25 18.48; base3 -> 11.35 23.97 18.18;
  main3 -> 12.77 23.62 18.16. That is 10 logical CPUs at a load of 11 to 28, i.e. oversubscribed throughout (other agents were building).
  Four-thread rows suffer most; one-thread rows are the cleaner reading.
- The two binaries are different files (`cmp`: differ at byte 217; sha1 `9a6dde9addc7...` for base, `222447bb3428...` for main), so the null result below is a comparison of two
  builds and not of one binary with itself.
- Same machine, same toolchain (rustc 1.99.1), same data (`reduced`).

**Result.** Over the 56 cells (P2 and H1, every sm-rt mode and workload, 1 and 4 threads) the median change main vs base lies
between -2.3% and +2.9%, with signs scattered both ways, and the per-cell run-to-run spreads are 0.0% to 17.5% (the 17.5% cell is the
base `P2 analytic w-flight 4` row with one slow run; all others at or below 8.4%). So **no slowdown or speed-up of W4-RT-TIDY is
resolvable at this load**. The rows on the path the tidy touched (hops: `P2 hierarchy`, `P2 analytic-hierarchy`,
H1 `delta`/`block` flight and `jump` fire) read between -2.3% and +2.6%, inside the spread of the same cells. For a
claim tighter than a few percent the run needs the quiet x86 machine of ruling D6 (or this Mac idle).

| case | mode | workload | thr | base 33efd2c: median of 3 [min, max] | main b51806f: median of 3 [min, max] | main vs base |
|---|---|---|---|---|---|---|
| H1 | block | w-flight | 1 | 2.486e+05 [2.461e+05, 2.509e+05] (1.9%) | 2.511e+05 [2.5e+05, 2.513e+05] (0.5%) | +1.0% |
| H1 | block | w-flight | 4 | 9.633e+05 [9.437e+05, 9.674e+05] (2.5%) | 9.414e+05 [9.41e+05, 9.681e+05] (2.9%) | -2.3% |
| H1 | delta | w-flight | 1 | 2.481e+05 [2.459e+05, 2.512e+05] (2.1%) | 2.51e+05 [2.487e+05, 2.517e+05] (1.2%) | +1.2% |
| H1 | delta | w-flight | 4 | 9.598e+05 [9.453e+05, 9.643e+05] (2.0%) | 9.526e+05 [9.41e+05, 9.684e+05] (2.9%) | -0.8% |
| H1 | facets | w-fire | 1 | 8.752e+05 [8.692e+05, 8.799e+05] (1.2%) | 8.628e+05 [8.537e+05, 8.675e+05] (1.6%) | -1.4% |
| H1 | facets | w-fire | 4 | 8.787e+05 [8.73e+05, 8.815e+05] (1.0%) | 8.673e+05 [8.587e+05, 8.75e+05] (1.9%) | -1.3% |
| H1 | facets | w-fire-locate | 1 | 3.543e+05 [3.531e+05, 3.557e+05] (0.7%) | 3.537e+05 [3.536e+05, 3.55e+05] (0.4%) | -0.2% |
| H1 | facets | w-fire-locate | 4 | 3.533e+05 [3.519e+05, 3.56e+05] (1.2%) | 3.567e+05 [3.547e+05, 3.575e+05] (0.8%) | +1.0% |
| H1 | facets | w-track | 1 | 2.655e+05 [2.632e+05, 2.681e+05] (1.8%) | 2.675e+05 [2.635e+05, 2.682e+05] (1.8%) | +0.7% |
| H1 | facets | w-track | 4 | 9.93e+05 [9.841e+05, 9.995e+05] (1.5%) | 9.994e+05 [9.968e+05, 1.006e+06] (0.9%) | +0.6% |
| H1 | jump | w-fire | 1 | 9.792e+05 [9.729e+05, 9.812e+05] (0.8%) | 9.758e+05 [9.476e+05, 9.806e+05] (3.4%) | -0.3% |
| H1 | jump | w-fire | 4 | 9.696e+05 [9.683e+05, 9.914e+05] (2.4%) | 9.873e+05 [9.824e+05, 9.91e+05] (0.9%) | +1.8% |
| H1 | jump | w-fire-locate | 1 | 6.909e+05 [6.675e+05, 7.117e+05] (6.4%) | 7.079e+05 [7.079e+05, 7.082e+05] (0.0%) | +2.5% |
| H1 | jump | w-fire-locate | 4 | 6.978e+05 [6.949e+05, 7.057e+05] (1.5%) | 7.02e+05 [6.978e+05, 7.085e+05] (1.5%) | +0.6% |
| H1 | poly-ref | w-flight | 1 | 1.238e+06 [1.232e+06, 1.239e+06] (0.6%) | 1.244e+06 [1.235e+06, 1.246e+06] (0.9%) | +0.5% |
| H1 | poly-ref | w-flight | 4 | 4.367e+06 [4.363e+06, 4.5e+06] (3.1%) | 4.431e+06 [4.387e+06, 4.465e+06] (1.8%) | +1.5% |
| H1 | poly-ref | w-track | 1 | 6.194e+05 [6.187e+05, 6.259e+05] (1.2%) | 6.317e+05 [6.252e+05, 6.329e+05] (1.2%) | +2.0% |
| H1 | poly-ref | w-track | 4 | 1.954e+06 [1.941e+06, 1.981e+06] (2.0%) | 1.956e+06 [1.843e+06, 1.991e+06] (7.6%) | +0.1% |
| H1 | poly-walk | w-flight | 1 | 4.836e+04 [4.8e+04, 4.853e+04] (1.1%) | 4.839e+04 [4.791e+04, 4.848e+04] (1.2%) | +0.1% |
| H1 | poly-walk | w-flight | 4 | 1.858e+05 [1.837e+05, 1.858e+05] (1.1%) | 1.857e+05 [1.839e+05, 1.861e+05] (1.2%) | -0.1% |
| H1 | poly-walk | w-track | 1 | 3.709e+04 [3.704e+04, 3.732e+04] (0.8%) | 3.723e+04 [3.7e+04, 3.738e+04] (1.0%) | +0.4% |
| H1 | poly-walk | w-track | 4 | 1.434e+05 [1.433e+05, 1.438e+05] (0.3%) | 1.43e+05 [1.39e+05, 1.439e+05] (3.4%) | -0.3% |
| H1 | walk | w-fire | 1 | 1.058e+06 [1.056e+06, 1.061e+06] (0.5%) | 1.051e+06 [1.024e+06, 1.059e+06] (3.4%) | -0.7% |
| H1 | walk | w-fire | 4 | 1.057e+06 [1.057e+06, 1.067e+06] (1.0%) | 1.065e+06 [1.062e+06, 1.07e+06] (0.7%) | +0.8% |
| H1 | walk | w-fire-locate | 1 | 7.585e+05 [7.561e+05, 7.744e+05] (2.4%) | 7.804e+05 [7.77e+05, 7.831e+05] (0.8%) | +2.9% |
| H1 | walk | w-fire-locate | 4 | 7.726e+05 [7.722e+05, 7.733e+05] (0.1%) | 7.739e+05 [7.736e+05, 7.816e+05] (1.0%) | +0.2% |
| H1 | walk | w-flight | 1 | 1.253e+06 [1.253e+06, 1.256e+06] (0.2%) | 1.262e+06 [1.248e+06, 1.264e+06] (1.3%) | +0.7% |
| H1 | walk | w-flight | 4 | 4.509e+06 [4.502e+06, 4.542e+06] (0.9%) | 4.472e+06 [4.394e+06, 4.534e+06] (3.1%) | -0.8% |
| H1 | walk | w-walk | 1 | 6.256e+05 [6.152e+05, 6.275e+05] (2.0%) | 6.373e+05 [6.357e+05, 6.423e+05] (1.0%) | +1.9% |
| H1 | walk | w-walk | 4 | 1.959e+06 [1.952e+06, 1.992e+06] (2.0%) | 1.995e+06 [1.959e+06, 2.022e+06] (3.1%) | +1.8% |
| P2 | analytic | w-fire | 1 | 4.903e+06 [4.876e+06, 4.909e+06] (0.7%) | 4.858e+06 [4.816e+06, 4.903e+06] (1.8%) | -0.9% |
| P2 | analytic | w-fire | 4 | 4.877e+06 [4.851e+06, 4.878e+06] (0.5%) | 4.889e+06 [4.874e+06, 4.926e+06] (1.0%) | +0.2% |
| P2 | analytic | w-fire-locate | 1 | 2.424e+06 [2.421e+06, 2.429e+06] (0.3%) | 2.415e+06 [2.412e+06, 2.419e+06] (0.3%) | -0.4% |
| P2 | analytic | w-fire-locate | 4 | 2.42e+06 [2.417e+06, 2.426e+06] (0.3%) | 2.417e+06 [2.401e+06, 2.418e+06] (0.7%) | -0.1% |
| P2 | analytic | w-flight | 1 | 1.382e+06 [1.37e+06, 1.384e+06] (1.0%) | 1.384e+06 [1.378e+06, 1.409e+06] (2.2%) | +0.1% |
| P2 | analytic | w-flight | 4 | 5.101e+06 [4.209e+06, 5.103e+06] (17.5%) | 4.993e+06 [4.912e+06, 4.998e+06] (1.7%) | -2.1% |
| P2 | analytic | w-track | 1 | 1.253e+06 [1.246e+06, 1.261e+06] (1.2%) | 1.252e+06 [1.249e+06, 1.254e+06] (0.3%) | -0.1% |
| P2 | analytic | w-track | 4 | 3.975e+06 [3.677e+06, 4.011e+06] (8.4%) | 4.01e+06 [3.957e+06, 4.031e+06] (1.9%) | +0.9% |
| P2 | analytic-hierarchy | w-fire | 1 | 3.681e+06 [3.679e+06, 3.686e+06] (0.2%) | 3.725e+06 [3.593e+06, 3.729e+06] (3.6%) | +1.2% |
| P2 | analytic-hierarchy | w-fire | 4 | 3.675e+06 [3.65e+06, 3.68e+06] (0.8%) | 3.74e+06 [3.723e+06, 3.752e+06] (0.8%) | +1.8% |
| P2 | analytic-hierarchy | w-fire-locate | 1 | 1.908e+06 [1.907e+06, 1.908e+06] (0.0%) | 1.924e+06 [1.916e+06, 1.928e+06] (0.6%) | +0.8% |
| P2 | analytic-hierarchy | w-fire-locate | 4 | 1.906e+06 [1.903e+06, 1.908e+06] (0.3%) | 1.923e+06 [1.922e+06, 1.923e+06] (0.0%) | +0.9% |
| P2 | analytic-hierarchy | w-track | 1 | 1.039e+06 [1.036e+06, 1.05e+06] (1.4%) | 1.051e+06 [1.05e+06, 1.052e+06] (0.2%) | +1.1% |
| P2 | analytic-hierarchy | w-track | 4 | 3.366e+06 [3.314e+06, 3.425e+06] (3.3%) | 3.454e+06 [3.397e+06, 3.499e+06] (3.0%) | +2.6% |
| P2 | facets | w-fire | 1 | 5.799e+05 [5.732e+05, 5.809e+05] (1.3%) | 5.75e+05 [5.61e+05, 5.777e+05] (2.9%) | -0.9% |
| P2 | facets | w-fire | 4 | 5.709e+05 [5.62e+05, 5.735e+05] (2.0%) | 5.774e+05 [5.737e+05, 5.816e+05] (1.4%) | +1.1% |
| P2 | facets | w-fire-locate | 1 | 2.592e+05 [2.587e+05, 2.594e+05] (0.3%) | 2.599e+05 [2.584e+05, 2.604e+05] (0.8%) | +0.2% |
| P2 | facets | w-fire-locate | 4 | 2.603e+05 [2.53e+05, 2.604e+05] (2.8%) | 2.592e+05 [2.587e+05, 2.6e+05] (0.5%) | -0.4% |
| P2 | facets | w-track | 1 | 5.879e+04 [5.868e+04, 5.916e+04] (0.8%) | 5.907e+04 [5.857e+04, 5.918e+04] (1.0%) | +0.5% |
| P2 | facets | w-track | 4 | 2.262e+05 [2.175e+05, 2.273e+05] (4.3%) | 2.247e+05 [2.201e+05, 2.272e+05] (3.2%) | -0.6% |
| P2 | hierarchy | w-fire | 1 | 1.305e+06 [1.304e+06, 1.305e+06] (0.1%) | 1.292e+06 [1.255e+06, 1.292e+06] (2.9%) | -1.0% |
| P2 | hierarchy | w-fire | 4 | 1.304e+06 [1.301e+06, 1.306e+06] (0.4%) | 1.296e+06 [1.292e+06, 1.299e+06] (0.5%) | -0.6% |
| P2 | hierarchy | w-fire-locate | 1 | 5.209e+05 [5.199e+05, 5.21e+05] (0.2%) | 5.22e+05 [5.213e+05, 5.225e+05] (0.2%) | +0.2% |
| P2 | hierarchy | w-fire-locate | 4 | 5.208e+05 [5.115e+05, 5.213e+05] (1.9%) | 5.223e+05 [5.219e+05, 5.227e+05] (0.2%) | +0.3% |
| P2 | hierarchy | w-track | 1 | 3.483e+05 [3.474e+05, 3.497e+05] (0.7%) | 3.497e+05 [3.482e+05, 3.501e+05] (0.6%) | +0.4% |
| P2 | hierarchy | w-track | 4 | 1.249e+06 [1.219e+06, 1.254e+06] (2.8%) | 1.255e+06 [1.228e+06, 1.264e+06] (2.9%) | +0.5% |

## Notes for the W4-INT report and later tasks

- The smoke covers every row type of plan section 5 item 4: P1 and H1 and P2 at `reduced`; trackers `smrt`, `dagmc-moab`,
  `dagmc-dd`, `xdg` (Partial, OR-d); `walk` (H1 fire/flight/`w-walk`, P1 flight) and `jump` (H1); the polyhedral rows (`poly-ref`, `poly-walk`, H1 only, the only case
  with its own tet mesh to dualize, labelled "different mesh, same geometry"); P2 `analytic-hierarchy` and `hierarchy`.
- `xdg` image build log was not captured (the harness builds with `docker build -q`). If the INT report wants the compiler
  lines, run `docker build --platform linux/arm64 -f crates/sm-rt-bench/docker/xdg/Dockerfile ...` by hand; the failure is the one already documented in
  `crates/sm-rt-bench/DESIGN.md` ("XDG pin") and was not re-diagnosed here.
- No change was made to `crates/sm-rt-bench` or anywhere else outside `docs/design/w4/int/`.
- Scratch outputs (result JSON, raw logs) stay outside the repository: the smoke results directory, `w4int-bench-smoke-run.log`,
  `w4int-bench-docker-check.log`, `w4int-bench-smoke-report.md` and `w4int-bench-ab-table.md` in the session scratchpad.
  `smrt-bench` removed its own `$TMPDIR/smrt-bench-case-*` directories.
