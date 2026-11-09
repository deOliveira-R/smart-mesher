# W4 draft: rt (ray tracing that beats DAGMC and XDG)

Standing principle (user): Cubit is a bar to surpass, not to match. MCNP is not a target. OpenMC is the CSG format and the oracle. Base: `d2f4803`. Upstream XDG checked on 2026-11-08 with `git ls-remote` and a clone of xdg-org/xdg only. Paths below that start with `xdg/` are in that clone at `a5d67e6`.

**Critique delta:** the critic re-checked every file:line below at `d2f4803`, and re-cloned xdg-org/xdg on 2026-11-08 (`git ls-remote` HEAD = `a5d67e635b96`). Corrections are marked inline.

## Findings (what is true today, with file:line)

### XDG driver and the pin
1. **No XDG driver exists.** `docker/xdg/Dockerfile` copies `drivers/xdg/` (`crates/sm-rt-bench/docker/xdg/Dockerfile:30`) and compiles `xdg_driver.cpp` and `smtets_to_moab.cpp` (`:37-41`). `drivers/` holds only `dagmc/` (`dagmc_driver.cpp`, `facets_to_h5m.cpp`, `run_case.sh`). `docker.rs:267-271` reports the xdg image as `NoDriver`. The W3c smoke confirmed it: `docker-check` rc = 2, from xdg only (`w3c-report.md:121`). `sm-rt-bench/DESIGN.md:213` says "`xdg` rows stay `partial` (no image, W4)".
2. **The pin is 5 commits behind upstream HEAD.** `pins.env:19` has `XDG_COMMIT=83efb306e152` (the pins.env comment says 2026-10-26; the git date is 2026-10-25). Upstream HEAD is `a5d67e6` (2026-11-06). The 5 commits are `5bee55b`, `e5d3662`, `56438be`, `12f56f1`, `a5d67e6`: `.github/workflows/ci.yml`, plus `num_volume_element_faces` (`include/xdg/mesh_manager_interface.h` +4, `src/mesh_manager_interface.cpp` +28) and two tests. C15 asks for "the newest upstream commit that builds and runs" (`rulings.md:23`).
   **Critique delta:** the new functions are face *counts* (`4 * num_elem` for tets, `6 *` for hexes), not face accessors. No driver call needs them, so they are no reason to order the tasks. `a5d67e6` differs from `12f56f1` only in `ci.yml`, so "HEAD builds" and "`12f56f1` builds" are the same question.
   **Critique delta:** the literal `83efb306e152` appears in more places than the draft listed: `docker.rs:686` (equality test), `docker.rs:704`, `:706`, `:707` (the drift test uses it as the `replace` target, so after a bump that test stops testing anything and fails), `sm-rt-bench/DESIGN.md:224`, `:259`, `:261`, `docs/research/dagmc-internals.md:16` and `w3c-report.md:121` (a report, left as history). All but the report must follow a bump.
3. **What XDG offers to a driver** (a5d67e6):
   - `XDG::ray_fire(volume, ...)` (`xdg/include/xdg/xdg.h:71`), `point_in_volume` (`:66`), `find_volume` (`:30`), `find_element` (`:33`, `:35`), `segments` (`:43`, `:52`).
   - Element walking: `MeshManager::walk_elements` (`src/mesh_manager_interface.cpp:144-180`) loops `next_element` (`:194`), which runs a Plücker test on each face of the element (`:217`, `:232`, `:243`) and breaks ties with `tie_tol = TINY_BIT * max(1, |min_dist|)` (`:289`, `TINY_BIT = 1e-10`, `include/xdg/constants.h:30`). A step that finds no finite exit only warns and breaks (`:159-162`). Adjacency is MOAB's element adjacency (`src/moab/mesh_manager.cpp:521-535`), so a walk crosses volume boundaries without stopping.
   - `XDG::segments(start, end)` over the whole model (`src/xdg.cpp:145-199`): `find_element(r)` is a BVH point location (`:162`). Outside the mesh it fires a ray against the implicit complement from `r + u * TINY_BIT` (`:168`), moves `hit + TINY_BIT` (`:173`), re-enters through `get_boundary_face_element` (`:181`), then calls `walk_elements` (`:189`). The per-volume overload (`:203-228`) starts from `find_element(volume_tree, start)` (`:210`), or after a `ray_fire` at `hit + TINY_BIT` (`:218`).
   - `find_volume` is a linear loop over volumes with one `point_in_volume` each (`src/xdg.cpp:115-130`).
   - Upstream ships `tools/ray_benchmark.cpp` and `tools/walk_elements.cpp`. Our driver must not use them for timing (they generate their own rays), but they show the intended calling pattern.
   **Critique delta:** the draft put the `TINY_BIT` nudges at `xdg.cpp:181`; they are at `:168`, `:173` and `:218`. `:181` is the boundary-face adjacency lookup. `find_volume`'s line numbers are in the source itself, not only in the research notes.
4. **MOAB input shape.** XDG's MOAB manager reads entity sets tagged `GEOM_DIMENSION` (volume 3, surface 2), `CATEGORY`, `GLOBAL_ID` and a `GEOM_SENSE_2` surface-to-volume sense tag (`src/moab/mesh_manager.cpp:131-162`, volumes and surfaces collected at `:67-102`).
   **Critique delta:** the draft said "a bare tet soup will not load". That is wrong. When the file has no volume sets, `init` builds one volume from all volume elements, skins it into one boundary surface and adds the surface to the volume (`src/moab/mesh_manager.cpp:104-124`). A bare tet file loads as one volume. What is lost is the regions: per-region `ray_fire(volume, ...)`, `find_volume` and the complement path then do not match our regions. The L2 walk does not need volumes (it walks MOAB adjacency and returns element ids, which the driver maps to regions from `tets.bin`). Per-region sets matter only if XDG `fire` rows run on tet input. This makes the converter smaller (see W4-RT-TETS).
5. **`smtets v1` exists, its consumers do not.** The format is in `formats.rs:187-222` (`Tets`), and `cases.rs:136` writes `tets.bin` for `H1`. The H1 twin is `kuhn_tets` (`cases.rs:422`): a structured Kuhn subdivision with stair-stepped spheres (`sm-rt-bench/DESIGN.md:37`). P2 has no tet twin (`w3c-report.md` W4 deferrals).
   **Critique delta:** `kuhn_tets` is at `:422`, not `:421`. The `walk`, `delta` and `block` rows are **W-flight only** (`sm-rt-bench/DESIGN.md:99`). There is no sm-rt W-track over tets today, so the L2 comparison below needs a new `walk` W-track row as well as the XDG one. P2 has no tet twin because "pins in vacuum do not fill their box" (`DESIGN.md:293-295`): there is nothing between the pins to mesh. Also deferred and not in the draft: "H1 analytic case" (`w3c-report.md:130`). H1 has no analytic row because a box inside a solid needs a union of half-spaces (`DESIGN.md:125-126`).

### sm-rt performance backlog
6. **Cold locate is a scan of every cell.** `locate_all` (`crates/sm-rt/src/analytic/leaf.rs:471-483`) tests all cells. `locate_cold` (`:486-510`) uses a concentric fast path (binary search over radii) when it applies, else `locate_all`. Measured: 1.2e7 points/s with the concentric path, 2.9e6 without, on a 12-ring pin cell (`sm-rt/DESIGN.md:322-326`). A model without concentric structure pays O(cells) per cold locate. DESIGN.md lists "a cell-box BVH for the cold CSG locate" as not done (`DESIGN.md:573`).
7. **Complement stepping is O(surfaces).** In `step_once` a ray in the complement takes all surfaces as relevant (`analytic/tracker.rs:374`, the `0..ns` range), roots them all (`:376-392`) and sorts the hits (`:393`). A ray inside a cell tests only that cell's surfaces (`cell_surfs`). The crossing lookup `across` uses the `surface -> cells` CSR (`leaf.rs:535`) but falls back to `locate_all` next to another surface or strictly inside the bounds (`:549-557`), which is O(cells) and counted as `slow_locates` (12 per 1e6 segments on the pin cell, `DESIGN.md:324`).
   **Critique delta:** there is a fourth O(surfaces) scan: `on_face_any` (`leaf.rs:517-519`), called for every complement locate (`tracker.rs:278`, `:325`). The hit sort is at `:393`, not `:397`.
   **Critique delta (the trigger already fires):** the backlog trigger (line 105) is "a benchmark where analytic stepping or cold locate dominates". The bench already has one: the P2 `analytic` row is the bridge world **flattened into one leaf** (`sm-rt-bench/src/smrt.rs:224-234`, `sm-rt/src/bridge.rs:23-25`, `:505-512`). At full scale that is 2601 pins x 2 cells in one leaf, no concentric structure, and pins in vacuum, so most of every track is complement stepping over about 5e3 surfaces. `sm-rt/DESIGN.md:533-534` warns about exactly this ("stepping through it scans all surfaces ... use it for small worlds"). No new OpenMC model is needed to show the cost.
8. **2D lattice z sentinel.** `PITCH_2D = 1e9` (`crates/sm-rt/src/bridge.rs:121`) is used when a model lattice has no z pitch and `n[2] == 1` (`:303`). `sm-rt` computes element centres as `origin + (idx + 0.5) * pitch` (`world.rs:194-199`). Nothing is known to fail with it.
   **Critique delta:** the draft said the frozen API forces this (`sm-rt-api/src/stub.rs:430`). It does not. `stub.rs:430` validates the reference **stub's** own `RectLattice`, not the `WorldSpec` the bridge builds. The world validator `Lattice::shape_issues` requires a positive z pitch only when `dims[2] > 1` (`sm-rt-api/src/world.rs:122-124`), and the type's doc says "z pitch ignored for 2D lattices" (`:39`). In sm-rt, `axis_cands` treats z of a single-layer lattice as unconstrained (`world.rs:212-214`) and the fit check skips z (`:999`). The only things that need `pz > 0` are sm-rt's own check (`bridge.rs:304`) and `Lat::center` (`world.rs:198`). So the fix is sm-rt's, not an API change. One wrinkle for the orchestrator: the API helpers `Lattice::element_center` and `locate_rect` (`sm-rt-api/src/world.rs:74-80`, `:99-107`) use `pitch[2]` unconditionally, against their own doc. With `pz = 0`, `locate_rect` divides by zero and returns `None`. Nothing in the repo calls `locate_rect`. The bridge's flatten calls `element_transform` (`bridge.rs:533`), which is fine with `pz = 0` and `origin.z = 0`. A doc or helper fix is orchestrator work but does not block sm-rt. Also latent: the API accepts a NaN or infinite z pitch for a single layer, and sm-rt's `Lat::center` then yields NaN.

### Benchmark machine and fairness already built
9. **Rules exist and are enforced.** `DESIGN.md:14-25` fairness table (a)-(h) and Q9: same bytes in, ABAB interleave, hinted start for both sides with separate locate rows, precision column, `geom_error`, memory and build columns, a comparison shown only if the agreement check passed, `NOT_MEASURED` footer (`report.rs:18`), and any OS or arch other than Linux x86_64 rendered under `SMOKE RESULT` (`report.rs:74-76`). `report::render` refuses mixed `cpu_model`, OS/arch, `machine_id` or UTC day (rule b, `report.rs:70`).
   **Critique delta:** `EnvRecord::smoke` is only `os != "linux" || arch != "x86_64"` (`result.rs:160-161`). A GitHub x86_64 runner VM is therefore *not* smoke today. The bare-metal check in W4-RT-X86 must go into this decision, not only into the script. `governor` is already recorded (`result.rs:154`).
10. **We have no x86_64 benchmark machine.** The Mac is an M1 Max, and Docker Desktop runs a linux/arm64 VM (`w3c-report.md:119`). Docker images name `--platform linux/<host arch>` and refuse another arch (`DESIGN.md:230`), so there is no emulated shortcut, and an emulated run would be meaningless anyway. Embree and double-down use different SIMD paths on arm64 than on AVX2 x86_64, so arm64 numbers cannot stand in for x86_64. GitHub's x86_64 runners are shared VMs: right for correctness (they already run `platform_pins`, `w3c-report.md:116`), wrong for timing.
11. **Linux hash lines are open.** Linux `smcase.json` and facet hashes are unblessed. `gen --check` reports them `unblessed` (`DESIGN.md` "Bytes and platforms"; backlog line 109). The first Linux run prints them with `gen --print`.
12. **Result-writer hygiene item.** The backlog asks that `date_utc` and the sequence counter stay outside `result_hash` the next time the writer is edited (backlog line 111).
   **Critique delta:** this is already true and tested. `sm-rt-bench/DESIGN.md:281-283` states it, and `tests/smrt.rs:465` (`two_smrt_runs_give_one_result_hash_and_date_and_sequence_stay_out_of_it`) enforces it. The XDG driver only has to keep that test green. The "if found inside, needs a ruling" contingency is dropped from W4-RT-XDG-DRIVER.
13. **Critique delta (new finding): sm-rt already has the region jump, the bench does not use it.** `MeshTracker` with `partition.enabled` and `region_jump` has a per-region boundary BVH `fire` (`sm-rt/DESIGN.md:331-336`, `:385-392`), tested against the walk to ruling R1 (`tests/delta_jump.rs`; `hex_block(40)`: 163 face tests per fire by the walk, 11 by the jump, `DESIGN.md:404-405`). `sm-rt-bench` never sets it (no `region_jump` in `crates/sm-rt-bench/src`). Claim 9, the one known loss, can therefore be measured now on H1's tets with a bench-only change.
14. **Critique delta (new finding): polyhedral cells are the user's production path, and the bench has no polyhedral row.** The transport code consumes polyhedral meshes (`docs/ARCHITECTURE.md:14`, `:112`, sm-poly median dual). `MeshTracker` holds polyhedral cells natively (`sm-rt/DESIGN.md:9-15`, convex and non-convex paths at `:51-57`). XDG supports tets and hexes only (`src/moab/mesh_manager.cpp:77-81` refuses mixed, and `next_element` handles 3- and 4-vertex faces, `mesh_manager_interface.cpp:213-264`).

## Design

### The claim and how it can be shown
"Beats DAGMC/XDG" means something different per workload. The harness already separates them. We keep them separate and never average.

| Layer | Question | sm-rt row | Competitor row | Why fair |
|---|---|---|---|---|
| L1 same triangles | Is our facet tracker as fast as double-down on identical triangles? | `facets` | `dagmc-moab`, `dagmc-dd`, `xdg` (ray_fire) | same `facets.bin`, same rays, agreement gate |
| L2 same tets | Is our cell walk faster than XDG `segments` on identical tets? | `walk` (new W-track) | `xdg` (segments) | same `tets.bin`; compare per-element segment lists |
| L2b same domain, polyhedra | What does the user's own mesh type cost? | `walk` on the median dual of the L2 tets | `xdg` on the L2 tets | same geometry and regions; different cells, labelled. XDG cannot read polyhedra |
| L3 best of each | What does a user get? | `analytic`, `hierarchy`, `delta`, `jump` | best DAGMC/XDG row | different geometry fidelity, shown as `geom_error` and `structure_bytes` |
| L4 flights | Time per history in a transport loop | W-flight | DAGMC W-flight (done), XDG flight | same SplitMix64 stream, 4-sigma tally agreement |

**Critique delta:** L2's sm-rt row is new (the `walk` row is W-flight only today, finding 5). L2b is added (finding 14). A dual cell replaces several tets, so a polyhedral walk takes fewer steps per chord. The user ships exactly this, and no competitor can run it. It is labelled as a different mesh and never presented as a same-input comparison. `jump` is added to L3 (finding 13).

What we expect to be able to win, from the claims ledger (`docs/research/tracking-landscape.md:37-51`), and what is a known loss:
- **Wins we can defend:** exact analytic faces (no faceting error, one cylinder instead of thousands of triangles); instancing (P2: 2601 copies of one prototype versus DAGMC flattening; the `structure_bytes` column); hinted location and no root restart; bit-identical output for 1 and N threads (ledger claim 8 says DAGMC uses `rand()`; the bench measures only our side, so the report states ours and does not grade theirs).
- **Known loss, to be published as a loss:** claim 9. In a large homogeneous region DAGMC does one O(log T) ray fire, while a cell walk pays per cell.
  **Critique delta:** the region jump already exists in sm-rt (finding 13), so "until it runs in the bench" is a small bench task, not a future feature. It moves into the first sub-wave (W4-RT-JUMP-ROW). The walk-only loss row stays in the table beside the jump row, so the loss stays visible.
- **XDG is not DAGMC.** It is DAGMC's lineage and walks tets and hexes already (`dagmc-internals.md` XDG section), so "cell walking is novel" is not a claim.
  **Critique delta:** the draft's L2 argument ("exact adjacency and no per-step nudge versus per-step Plücker tests with `TINY_BIT` offsets") overstates the difference. XDG also walks exact MOAB adjacency. Its per-step cost is up to 4 Plücker tests with a `TINY_BIT` tie tolerance and no nudge (`mesh_manager_interface.cpp:217-289`). The nudges are only at complement re-entry (`xdg.cpp:168`, `:173`, `:218`). L2 is a plain speed and robustness measurement with no structural claim. The structural claims are L2b (polyhedra), L3 (analytic, instancing) and the jump.

### Pre-registered success criteria (so we cannot tune the verdict afterwards)
A row is a win only if all of these hold, on the x86_64 machine, in each of three independent sessions:
1. The agreement check passed for both trackers (fairness rule g); otherwise the cell shows `n/c`.
2. The median rate is higher by more than the larger min-max spread of the two rows, at 1 thread and at the machine's full physical-core count.
3. `structure_bytes`, `build_s` and `peak_rss_bytes` are reported next to it. No win is claimed where a tracker is more than 2x worse on memory or build time unless that is stated beside the number.

Statement of the result: "sm-rt beats double-down on W-fire in k of n cases and loses in the rest", with the cases named. No single headline number.

**Critique delta:** rule b refuses mixed UTC days in `report::render` (`report.rs:70`, `check_same_machine`). The three-session criterion therefore cannot be a relaxation of `render`. It must be a separate aggregator over three single-session reports that only compares their verdict signs, never their numbers. W4-RT-PUBLISH says so now.

### Workloads and scales
Existing cases stay (P1 pin, P2 17x17 clusters, H1 inclusions, F1 torus shells). For credibility we add scale, not new geometry families:
- **Full-scale tiers** for all cases: P2 at 1e4+ pins (a core-sized lattice, where flattening hurts DAGMC memory), F1 at about 1e6+ triangles (already 1.8e6 at sag 0.05), and H1 at 1e7 tets (XDG's element BVH build and RSS; 7.5e5 today).
  **Critique delta:** today `Scale::full()` is P2 `lattice_n = 17` (2601 pins, about 4e6 triangles) and 200 000 rays (`cases.rs:96-99`). 1e4+ pins and 1e6 rays are a new tier (`xl`), not edits of `full`, so the blessed `full` hashes do not move. H1 at 1e7 Kuhn tets needs `n = 119` cells per side (6 n^3). At sm-rt's recorded 142 bytes per face (`sm-rt/DESIGN.md:46`, `:564`; measured on hex and facet meshes, so treat it as an estimate for tets) and about 2e7 faces, that is roughly 3 GB for sm-rt alone. It fits the benchmark machine, not CI. The tier runs only there.
- **A conforming tet twin of P1 from our own mesher**, not a Kuhn stair-step. The Kuhn mesh is cheap and exact for the walk, but its stair-stepped surfaces make segments differ from the analytic ones, so it measures walking only. A conforming mesh tests volume and boundary handling together. The H1 Kuhn case stays as the structured baseline.
  **Critique delta:** P2 is dropped from the conforming twin. P2 has nothing between its pins (finding 5), so a P2 tet twin would either add a moderator region (a new geometry, which this section says we avoid) or be disjoint pin islands joined through the complement (that measures XDG's `ray_fire` re-entry, not walking). P1 has a moderator, and the W3c milestone already meshes an enclosed pin all-tet (`w3c/plan.md:160`). `sm-scheme-tet` keeps boundaries frozen (`sm-scheme-tet/DESIGN.md:13-20`), so a multi-region conforming mesh is the normal `mesh.run` path. The dependency question stays (finding below, R8).
- **One local real model** via the existing `--model` path (the model path is never recorded, and results are refused inside a git work tree, `DESIGN.md:84-85`). It goes in a separate, labelled table, because nobody else can reproduce it.
  **Critique delta:** "a `--model` run has no rows: no sm-rt row builds a general model here" (`DESIGN.md:288-291`), and imported periodic faces do not tessellate (sm-cadio leaves pcurves unset, a W4 deferral). The local-model table is therefore blocked on that upstream fix and is not a W4-rt deliverable. Keep Q3 and move it after the first table.
- Thread counts 1, 4, and all physical cores (SMT off or cores pinned with `taskset`). 7 reps and 3 warm-ups as today (`cli.rs:211`).
- **Cold versus warm:** a cold-cache first rep recorded as its own column for the build-and-first-ray story.
  **Critique delta:** today the warm-up reps are timed but not reported (`runner.rs:45-52`). "Cold" needs a definition that can be judged: the first rep of a fresh process after the build, with page cache not dropped (dropping it needs root and would differ between the Docker and native sides). Otherwise it is noise and should be dropped.

### XDG driver (design)
`xdg_driver.cpp` follows the DAGMC driver's protocol exactly (JSON `Row` lines, dump lines, `--workload`, `--version`; `dagmc_driver.cpp:4-22`, `:315-346`), so `runner` and `compare` change nothing:
- `fire` = `ray_fire(volume, start, dir, INFTY, EXITING)` from the file's region (hinted). `fire-locate` = `find_volume` (a linear loop over volumes, `xdg.cpp:115-130`). This is the same split as the DAGMC driver.
- `track` = `XDG::segments` over the whole tet model for tet cases, and a surface loop of `ray_fire` for facet cases (so XDG appears in L1 and L2, not only L2).
- `flight` = a port of the same SplitMix64 flight as `dagmc_driver.cpp` (already ported once). Element or volume lookups use XDG's own calls.
- **Critique delta:** the precision column is `mixed`, the same as double-down. Embree traverses in single precision, and XDG carries `INFTYF` for its float ray path (`include/xdg/constants.h:20-22`). The acceptance names the value instead of "read from the build". Embree's version and the apt packages go to `versions.txt` (already the recipe; apt versions are recorded, not frozen, C13).
- Threads via OpenMP or `std::thread`, with a ray-ordered determinism hash identical in form to the DAGMC driver's.
- The Docker build takes the pin from `pins.env` only. The pin is bumped to the newest upstream commit that builds and runs (`a5d67e6` today; re-check with `git ls-remote https://github.com/xdg-org/xdg` at task time). If HEAD does not build, take the nearest older commit that does, and record why.

### Cell index for analytic leaves (design)
The draft proposed one flat SoA BVH over cell bounding boxes, used for `locate_cold`, complement stepping and the `across` fallback, with results equal to `locate_all` and built only above a cell-count threshold.

**Critique delta (better design):** use a **disjoint kd tiling over cell boxes**, the same structure `MeshTracker` already uses for its majorant partition (`sm-rt/DESIGN.md:338-341`: "a kd tree over the cell boxes padded by 4 `tol`; leaves tile it; a cell is listed in every leaf it may meet"), not an overlapping BVH. Reasons:
- **Complement stepping gets early exit.** With a BVH, a ray query returns every cell box the ray crosses along its whole length, and then they are all rooted and sorted, as today but fewer. With a tiling, the ray walks the leaves front to back. In each leaf it roots only the candidates' surfaces, clipped to the leaf interval, and it stops at the first event. The event is the same: the first root that enters a cell lies inside that cell's box, so inside a leaf that lists the cell. Ties sort the same way, by `(t, surface id)`.
- **Point locate** is a descent to one leaf plus a scan of its candidates (all of them, so an overlap is still reported). `on_face_any` (finding 7) uses the same leaf's surfaces.
- **One structure, a fourth use:** the same tiling is the majorant partition that delta tracking needs on CSG leaves. `AnalyticTrackerBuilder` refuses a partition today (`sm-rt/DESIGN.md:428`). Backlog line 45 asks for "a disjoint partition traversed by DDA ... a BVH has overlapping boxes, so it serves point location but not majorant segments". A BVH would have to be built a second time later.

The rest of the draft's design stays: built only above a cell-count threshold (an internal constant with a crate-private override for tests, so no `BuildOptions` change in the frozen API). Cells without a provable box go to an always-test list. Results are equal to the scan everywhere. The structure is derived and not hashed. The draft's comparison with OpenMC's per-universe cell scan stays.

**Critique delta (higher-value fix for the flagship case):** for P2 the right answer is not a faster flat leaf but no flattening. A CSG prototype hosted by `WorldTracker` under `Lattice`/`Union` gives an O(1) lattice index plus a 2-3-cell leaf, exact faces and instancing in one row: the strongest row conceivable against DAGMC on a lattice. It is listed as deferred (`sm-rt/DESIGN.md:528-530`, `:571`: "CSG leaves under `Union`/`Lattice` (needs `WorldTracker` to host a non-mesh leaf)"). OpenMC's own models are built this way (universes), so flat leaves stay large only for flat imports. That is the cell index's job. Both are proposed. See W4-RT-CSG-WORLD.

### Linux x86_64 benchmark machine (design)
- **Bare metal, not a VM:** a noisy neighbour or a hypervisor steal-time spike would break rule b. Record `lscpu`, kernel, governor, SMT state and the isolated core list in `bench-env.json` (extend the existing machine record, `result.rs:381`).
- **One script:** `tools/bench/x86_64.sh` (orchestrator-owned) checks `uname -m`, sets the performance governor, builds the three images, runs `docker-check`, runs `gen --check` (the unblessed lines are then printed for blessing, finding 11), runs the full `run` per case in one session, then runs `report`. Everything lands in a dated results directory outside git.
- **Same-session rule** stays: all trackers of a case run within one script run, ABAB interleaved.
- **Cross-session reproducibility:** run the script on three different days. A win in the report needs the same sign in all three (criteria above; a separate aggregator, not `render`).
- **CPU flags:** our build and the competitors' builds are what each project ships by default for x86_64 (Embree dispatches at run time). A second sm-rt row built with `-C target-cpu=native` is published and labelled. The headline uses the default build.
- **Critique delta:** on a Linux host Docker runs natively, so the competitor rows and the native sm-rt rows both run on the host kernel. Each side is pinned to the same core set (`--cpuset-cpus` for containers, `taskset` for native). Without that, risk 7 recurs.
- arm64 results (this Mac, CI arm runners) stay a separate `SMOKE RESULT` table. They are never merged with x86_64 (existing rule).

## Tasks

The owner crate is `crates/sm-rt-bench` unless stated. API-owning crates (`sm-rt-api`, `sm-testkit`) are the orchestrator's.

### W4-RT-XDG-PIN. S
**Critique delta:** folded into W4-RT-XDG-DRIVER as its first step. The new upstream commits add nothing the driver needs (finding 2). The bump is a one-line edit plus the build, and the driver worker is the one who finds out whether HEAD builds. A separate task only adds a merge. The content stays, as the driver's step 0:
- **Scope:** re-read `git ls-remote` of xdg-org/xdg (never a fork), build candidate commits in the existing recipe, pin the newest one that builds and runs the driver. Record the date and the commit subject in the pins.env comment (same wording as C15's).
- **Acceptance:** the Dockerfile `ARG` equals `pins.env` (the existing lint passes). **Critique delta:** the draft wanted the test at `docker.rs:686` to read the pin from `pins.env`, but that makes it compare pins.env with itself and removes the guard. Instead, one `const XDG_PIN` in `docker.rs` is used by the equality test (`:686`) and the drift test (`:704-707`). A bump then edits `pins.env`, the Dockerfile `ARG` and that const, and the lint and test catch any pair that disagrees. `sm-rt-bench/DESIGN.md:224`, `:259`, `:261` updated. `xdg_driver --version` prints the pin (already asserted in the Dockerfile, `:41`). `docs/research/dagmc-internals.md:16` and the rulings are the orchestrator's.

### W4-RT-XDG-DRIVER. L
- **Owner:** `drivers/xdg/{xdg_driver.cpp,run_case.sh}`, `docker/pins.env`, `docker/xdg/Dockerfile`, `src/docker.rs`, `src/cli.rs`, `src/runner.rs`, `DESIGN.md`.
- **Scope:** step 0 (the pin, above), then the driver design above: fire, fire-locate, track (facet and tet), flight; the same row and dump protocol as the DAGMC driver; remove the `NoDriver` status once the driver exists; a determinism hash per row. **Critique delta:** the `date_utc`/counter item is already done (finding 12). Keep `tests/smrt.rs:465` green, nothing more.
- **Acceptance (exact checks):**
  1. 10000 of 10000 fire and fire-locate rays agree with the sm-rt facets fire within `chord_tol` (`compare_fires`), regions equal, grazing rays counted (as the DAGMC rows today, `w3c-report.md:124-125`).
  2. Flight tallies agree with the sm-rt surface flight at 4 sigma, `checked = 2 * regions + 2`, 0 failed.
  3. On C1: XDG fire distances equal the analytic chord within the facet `sag` bound (`bound.rs`). **Critique delta:** check at task start that C1 has a `truth::Scene` (C1..C3 go through `build_legacy`, `cases.rs:256`, `:264`). If it has none, use P1, which has one (`cases.rs:349`).
  4. A row whose agreement fails renders `n/c (disagrees)`. A missing image or a crash is `Partial` with the command (never an invented number).
  5. The driver's output is identical for 1 and N threads (ray-ordered hash), and `result_hash` of two runs is equal.
  6. `docker-check` reports xdg `Ready` for the host arch.
  7. **Critique delta:** the `precision` column of every xdg row is `mixed`.
- **Gates:** `cargo test -p sm-rt-bench` (including the Docker lint tests, which skip without Docker as today), and the smoke run `run --cases P1 --scale reduced --trackers smrt,dagmc-dd,xdg`.
- **Hash impact:** none. The result schema fields are unchanged.
- **Depends on:** nothing.

### W4-RT-TETS. M
- **Owner:** `drivers/xdg/smtets_to_moab.cpp`, `src/cases.rs`, `src/twin.rs`, `src/smrt.rs` (new `walk` W-track row), `expected_sha256.txt` (new lines only).
- **Scope:** (a) `smtets_to_moab`: read `smtets v1` and write an `.h5m`. **Critique delta:** a bare tet file already loads as one volume (finding 4), and the walk needs nothing more. The first version writes the tets in file order plus a `REGION` integer tag per tet, and the driver maps XDG element ids back through it. Per-region volume sets, interface surfaces and sense tags are a second step, needed only if XDG `fire` rows run on tet input. Do the second step only if L3 wants XDG-on-tets fire rows. (b) A `walk` workload in the XDG driver (`segments`) and the matching new sm-rt `walk` W-track row, comparing per-element segment lists. (c) A conforming P1 tet twin (P2 dropped, see Workloads). Whether `sm-rt-bench` may depend on `sm-mesher` is the orchestrator's ruling. If not, generate the tets with the `smesh` CLI into the case directory and pin their SHA-256 per os-arch.
- **Acceptance:**
  1. Round trip: every tet of `tets.bin` appears once in the `.h5m`. **Critique delta:** XDG's element id for tet `i` is checked to be a fixed function of `i` (MOAB ids follow creation order). `nt` and `nn` are equal, and the volumes sum to the input volume to 1e-12 relative.
  2. XDG loads the file with no warning, and `num_volume_elements` equals `nt`. (The draft's "interface triangles in exactly one surface set" check moves to the optional second step.)
  3. Walk agreement on H1: for 1e5 rays, the sequence of tets (after dropping chords shorter than `5e-9 diag`) is equal between sm-rt `walk` and XDG, and chord lengths agree within `1e-9 diag`. For each ray the chord sum equals the analytic cube chord to `1e-9 diag`. **Critique delta:** H1 rays start inside the cube, so XDG never takes its `TINY_BIT` re-entry path (`xdg.cpp:168-173`). Rays that do (conforming P1 has a vacuum outside) are compared with a tolerance of `2 * TINY_BIT` per re-entry, counted and listed.
  4. The new `tets.bin` bytes are hash-pinned per os-arch, the same way as the facet files.
- **Gates:** as above. `gen --check` passes with the new lines.
- **Hash impact:** new lines in `expected_sha256.txt` only. The existing C1..C3 legacy files and the six W3a hashes stay byte equal (asserted in `tests/cases.rs`).
- **Depends on:** W4-RT-XDG-DRIVER for the driver file. **Critique delta:** in practice one worker should own both, because they share `xdg_driver.cpp` and `run_case.sh`, not only the script. Alternatively, run TETS after DRIVER merges. Sizing goes from M to S-M with the smaller converter.

### W4-RT-JUMP-ROW. S (new)
**Critique delta:** added (finding 13).
- **Owner:** `crates/sm-rt-bench/src/smrt.rs`, `DESIGN.md`.
- **Scope:** a `jump` mode: `MeshTracker` on H1's tets with `partition.enabled` and `region_jump`, W-fire hinted and located, beside XDG and DAGMC fire rows on H1's facets. The walk-only fire row stays as the published claim-9 loss row.
- **Acceptance:** fire results agree with the `walk` row by ruling R1 (`sm-rt/DESIGN.md:394-404`) on all H1 rays. Counters `face_tests` per fire are reported for both. Determinism (1 vs N threads, two runs) is the same as the other rows.
- **Caveat:** H1's facets are icospheres and its tets are stair-stepped (`DESIGN.md:37`), so a jump-on-tets row and a DAGMC-on-facets row disagree by geometry. That is an L3 comparison with `geom_error`, not a same-input win. The same-input comparison for the jump is XDG `ray_fire` on the *tet* model's region boundaries. That needs the per-region volume sets of the TETS converter's second step, and it is the reason to do that step.
- **Hash impact:** none on existing rows (counters join hashes only when non-zero, `DESIGN.md:117-119`).
- **Depends on:** nothing. Rust-only, so it runs in parallel with the C++ tasks.

### W4-RT-POLY-ROW. M (new, for the user's goal)
**Critique delta:** added (finding 14). Proposed, pending the dependency ruling.
- **Owner:** `crates/sm-rt-bench/src/{cases.rs,smrt.rs}`, `DESIGN.md`.
- **Scope:** the median dual of H1's tets (`sm-poly`, `mesh.to_poly`) as a polyhedral twin. Run `walk` W-track and W-flight on it beside the tet `walk` and XDG `segments` rows on the same domain (L2b).
- **Acceptance:** per-ray region sequences equal those of the tet walk after merging equal-region neighbours (dual cells straddle no region boundary: they are keyed `(vertex, region)`, `sm-poly/DESIGN.md:16`; sm-poly takes regions from cell classification, so the bench must hand it H1's per-tet regions as volumes). Chord sums equal the cube chord to `1e-9 diag`. Flight tallies agree with the tet `walk` at 4 sigma. The rows are labelled "different mesh, same geometry".
- **Hash impact:** new case files only.
- **Depends on:** the orchestrator allowing `sm-rt-bench -> sm-poly` (R8), or an offline dual written with `smesh`.

### W4-RT-X86. M
- **Owner:** `tools/bench/x86_64.sh` and `docs/design/w4/bench-x86.md` (orchestrator, outside the crate), and `crates/sm-rt-bench/src/result.rs` for the machine record.
- **Scope:** the script and recipe of the Linux x86_64 section. Extend `bench-env.json` with `lscpu`, governor, SMT, isolated cores and kernel. A self-check refuses a published table if the host is not Linux x86_64, not bare metal (DMI/`systemd-detect-virt` is `none`), or the governor is not `performance`.
- **Acceptance:** on the arm64 Mac the script runs in dry mode and exits with "not x86_64". On the machine, the output contains the `gen --print` lines for blessing. **Critique delta:** `EnvRecord::smoke` (`result.rs:160-161`) gains the virt and governor inputs, so a GitHub x86_64 VM renders `SMOKE RESULT` (today it would not). A unit test of the banner decision covers fake environments: Linux x86_64 bare metal with performance governor (no banner), Linux x86_64 VM (banner), governor `powersave` (banner), macOS arm64 (banner). Note that the banner text in `report.rs:75` already says "not on bare-metal Linux x86_64", so the code now matches its text. Containers and native runs use the same `--cpuset-cpus`/`taskset` core list, recorded in `bench-env.json`.
- **Hash impact:** none. Linux `smcase.json` and facet hashes get blessed after an orchestrator ruling (existing rule).
- **Depends on:** the machine (Q1) for the real run. The script and the banner change can be written and tested first.

### W4-RT-PUBLISH. M
- **Owner:** `src/report.rs`, and `docs/design/w4/bench-report.md` (orchestrator).
- **Scope:** the pre-registered criteria as code: the report's verdict column is computed from the rows by the rules above (agreement, spread, memory and build qualifiers). The loss rows (claim 9) are shown, and the full-scale tiers. **Critique delta:** the three-session check is a separate `report --sessions A,B,C` aggregator. Each session is first rendered by the unchanged `render` (rule b intact), then only the per-row verdict signs are compared. Numbers from different days never appear in one table. The local-model table is removed from scope (blocked, see Workloads).
- **Acceptance:** a golden-file test of the verdict logic on fabricated rows covering a win, a spread overlap (no claim), a disagreement (`n/c`), a memory exception (qualifier text), mixed-day input to `render` (refused), and three sessions with one sign flip (no claim). No wall-clock gating in code (existing principle).
- **Hash impact:** none.
- **Depends on:** W4-RT-XDG-DRIVER and W4-RT-TETS for real rows. **Critique delta:** the verdict code and its golden test need only fabricated rows, so they can be written in the first sub-wave. Only the real table waits for the machine.

### W4-RT-CELLBVH. L (renamed W4-RT-CELLINDEX)
- **Owner:** `crates/sm-rt/src/analytic/` (`leaf.rs`, `tracker.rs`, a new `cellindex.rs`), `crates/sm-rt/DESIGN.md`.
- **Scope:** **Critique delta:** the kd tiling of the design (not a BVH), built when `n_cells` is above a threshold. Users: `locate_cold`, complement stepping (front-to-back leaves, early exit), the `across` fallback and `on_face_any`. The trigger benchmark is the existing P2 `analytic` row (finding 7). The first step records its `surface_evals`, `locate_candidates` and `slow_locates` per segment at `reduced` and `full`, with no new OpenMC model needed. An `sm-csgio` OpenMC import with 1e3+ cells is an optional second model for acceptance 1.
- **Acceptance:**
  1. For 1e6 random points (and 1e5 points placed within `tol` of surfaces) on three models (P2 flattened, the 12-ring pin cell with the threshold forced to 0, and one random-cell model), `locate_cold` with the index equals `locate_all`: same cell, same `Overlap` outcomes.
  2. A property test using `sm_testkit::strategies::config(n)` for random cell sets.
  3. 1000 complement rays: the event sequence (surface, distance bit-exact, `Next`) equals the all-surfaces implementation, which stays as a crate-private oracle path.
  4. Counters: `surface_evals` per complement step on P2 full drops from `n_surfaces` to at most the candidate surfaces of the leaves visited. The assertion is on the counter ratio, not on time.
  5. Determinism: node order bit-identical in two runs and for `RAYON_NUM_THREADS=1` versus default.
  6. The 12-ring pin cell counters (`DESIGN.md:324-325`) do not change (the index is not built below the threshold).
- **Gates:** `cargo test -p sm-rt`, clippy, rustdoc on new public items, the determinism test.
- **Hash impact:** none intended. The structure hash is by value of surfaces and cells (`DESIGN.md:318-320`) and the index is derived, not hashed. The flat-world golden hashes (`tests/flat_golden.rs`) and the W3b `h_*` hashes must stay identical. Any move stops the task (C3).
- **Depends on:** nothing. Rust-only, independent of the XDG tasks.

### W4-RT-CSG-WORLD. L (new)
**Critique delta:** added (design section).
- **Owner:** `crates/sm-rt/src/{world.rs,bridge.rs,analytic/}`, `crates/sm-rt/DESIGN.md`; `crates/sm-rt-bench/src/smrt.rs` for the row.
- **Scope:** `WorldTracker` hosts a CSG (analytic) prototype leaf under `Lattice` and `Union`. `CsgWorld` builds it without flattening when every leaf is CSG. P2 gets an `analytic-hierarchy` row. Flattening stays as the fallback and as the oracle.
- **Acceptance:** for every P2 ray, the event sequence (region, surface, distance within the world tolerance) equals the flattened analytic row. The existing test that pins CSG under `Union`/`Lattice` as `Unsupported` (`DESIGN.md:529`) is replaced by this equality. The OpenMC-oracle tests in `sm-csgio` (`tests/openmc_cross.rs`) pass on the hierarchical build. `pins_touched` matches the facet `hierarchy` row. `structure_bytes` of P2 is independent of the number of copies, up to the lattice fill array.
- **Hash impact:** new rows only. The flattened world and its hashes are unchanged. If the world structure hash of a CSG world is pinned anywhere, it is checked before and after (C3).
- **Depends on:** nothing. Can share a worker with W4-RT-CELLINDEX (same crate), done one after the other.

### W4-RT-LATZ. S (sm-rt, not an API change)
- **Critique delta:** the owner is `crates/sm-rt/src/{bridge.rs,world.rs}` only. No `blocked_on_api` (finding 8: the API already ignores the z pitch of a single-layer lattice).
- **Scope:** the bridge passes `pitch[2] = 0` for a single-layer lattice with no z pitch, with `origin.z = 0`. `Lat::center` and the fit check use `origin.z` for `dims[2] == 1` and never read `pitch[2]`, which also removes the NaN case for an infinite or NaN z pitch the API accepts. Report to the orchestrator (no blocking) that `Lattice::element_center`/`locate_rect` (`sm-rt-api/src/world.rs:74-80`, `:99-107`) do not honour the "z ignored" doc.
- **Acceptance:** a 2D lattice model gives the same cell and region sequence as today on the OpenMC-oracle tests (`sm-csgio`). A single-layer lattice with `pitch[2]` of 0, NaN or infinity builds and tracks identically. No `1e9` literal remains in `bridge.rs`.
- **Hash impact:** the lattice component hash writes the pitch (`world.rs:525`). To keep hashes, write `0` for `pitch[2]` when `dims[2] == 1`. That still moves the hash of any world built with today's `1e9`. **Critique delta:** the draft said this "touches the flat world golden hashes". Flat worlds have no lattice, and `tools/platform/expected.txt` pins no sm-rt structure hash. Grep at task time for any pinned hash of a bridged 2D-lattice world. If one exists, stop and ask (C3).
- **Depends on:** nothing. Cheap. Do it in the same worker as W4-RT-CSG-WORLD, which touches the same lattice code.

## Priority (for the user's goals: polyhedral meshes for the transport code, and ray tracing that beats DAGMC/XDG)

- **Now (first W4 sub-wave):**
  - C++ and Docker lane: W4-RT-XDG-DRIVER (with the pin), then W4-RT-TETS (same worker).
  - Rust lane in parallel: W4-RT-JUMP-ROW (S), the W4-RT-X86 script and banner change, and the W4-RT-PUBLISH verdict code on fabricated rows.

  Reason: the beat-DAGMC claim cannot be made or refuted until XDG rows exist and an x86_64 run is possible.
  **Critique delta:** the jump row moves up because the one known loss can be measured with code that exists. The PUBLISH verdict code moves up because it needs no machine. Pre-registering in code before any real table exists is the point of "pre-registered".
- **Next (second sub-wave, Rust):** W4-RT-CSG-WORLD plus W4-RT-LATZ (one worker), W4-RT-CELLINDEX, and W4-RT-POLY-ROW once its dependency is ruled.
  **Critique delta:** the draft gated the cell index on "the table shows a loss". The trigger already fires on our own P2 analytic row (finding 7), and the measurement is a first step of the task, so no x86_64 table is needed to justify it. CSG-WORLD is the bigger win for P2 and for OpenMC-style models. POLY-ROW is the row closest to what the user ships.
- **After the first real x86_64 table:** the real PUBLISH table, the three sessions, and the `xl` tier.
- **W5+ / on trigger:** OpenMC end-to-end (see Q4) and the local real model (blocked on sm-cadio pcurves).
- **Not in W4:** hex lattices (needs an explicit request, backlog line 112), GPU, XDG's GPRT backend (its docs say volumetric tracking is not implemented there, per the research notes), libMesh backend.

## Risks

1. **XDG may not build or may change API again.** Mitigation: pin by commit and record the build recipe. A failure is a `Partial` result with the failing command, never a made-up number. XDG HEAD moved 5 commits in 2 weeks.
2. **We lose on some rows.** The pre-registered criteria make losses visible. Claim 9 is expected to lose on the walk row. The jump row is the answer, and it is shown beside the loss, not instead of it. A positive headline is not guaranteed, and the report must say so.
3. **Unfair-comparison accusations are easy to attract:** different geometry fidelity (analytic versus facets), mixed precision (double-down, XDG), our own ray sampling. Mitigation: layers L1-L4 and the `precision` and `geom_error` columns already in the schema. L1 uses identical triangles. L2b is labelled "different mesh".
4. **The tet twin choice biases XDG.** The Kuhn mesh is structured and benign for BVH point location. Mitigation: the conforming P1 twin from our mesher, and the report says which mesh each row used.
5. **No x86_64 machine means nothing publishable.** The whole "now" group still produces correct arm64 smoke results and correctness agreement. Only the table waits.
6. **Noise on shared hardware** invalidates timing. Mitigation: bare metal, pinned cores, performance governor, three sessions.
7. **Docker runs are not pinned to cores** (noted in the W3c smoke: wide min-max on `dagmc-dd` w-fire rows, `w3c-report.md:126`). On bare metal, containers get `--cpuset-cpus` and native runs `taskset`, with the same core list. Part of W4-RT-X86.
8. **The conforming tet twin and the polyhedral twin need `sm-mesher`/`sm-poly` as dependencies of `sm-rt-bench`** (today it depends on neither, `Cargo.toml:18-27`). This is an ownership ruling for the orchestrator, not a user question.
9. **Unverified:** whether Ubuntu 24.04 apt ships Embree 3 or 4, and which one XDG supports at HEAD. The build will tell (recorded in `versions.txt`).
10. **Critique delta (new):** the 1e7-tet tier is about 3 GB for sm-rt alone (Workloads). It is benchmark-machine only, and the machine needs at least 64 GB for both trackers and their builds.

## Questions for the user

1. **Which x86_64 machine, and who arranges it?** Q9 says "a bare-metal Linux x86_64 machine arranged by the user", but none exists yet. Options: a box of yours; hourly rented bare metal (a cloud "metal" instance or a dedicated server) for a day per session; a VM (not acceptable for timing). *Recommendation: rented bare metal with at least 16 physical cores, SMT off and 64 GB RAM, one day per session, three sessions. I deliver a one-command script (W4-RT-X86). Nothing publishable before this.*
2. **Critique delta: no longer a question.** The pre-registered win criteria (agreement gate, spread test at 1 thread and all physical cores, memory and build qualifiers, three sessions, losses published) follow directly from the fairness rules the user already ruled on (Q9, rule g). They are adopted as the default and listed in the plan. The user can still veto them.
3. **May one local real model (your STEP files) appear in a separate, labelled "not reproducible" table?** It would never be committed. *Recommendation: yes, but after the first table. It is blocked today on sm-cadio pcurves for periodic faces (`sm-rt-bench/DESIGN.md:287-291`).*
4. **Is an end-to-end OpenMC-versus-OpenMC-with-sm-rt run wanted?** It needs an sm-rt plugin inside OpenMC (a large job and a fork to maintain), so it is a W5-size task. The W-flight mini transport already compares time per history. *Recommendation: not in W4. W-flight (all three trackers) is the W4 transport result, and the OpenMC plugin is decided after the table.*
5. **Critique delta: no longer a question.** Headline on default builds for everyone, with a labelled second sm-rt row at `target-cpu=native`. This follows from "same bytes in, best effort for each side". Adopted as the default.
6. **Critique delta (new, a real decision): should the polyhedral row (L2b) be part of the "beats XDG" story?** It is the user's production path and XDG cannot run it, but it is not a same-input comparison. *Recommendation: yes, as a separately labelled table, "what the transport code's mesh costs", never mixed into the k-of-n verdict.*
