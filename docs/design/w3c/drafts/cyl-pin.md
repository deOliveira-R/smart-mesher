# CYL-PIN: round pin cell with mesh.auto "default"

> Planning draft (W3c), corrected by the planning critique. Changes against the original draft are marked **Critique delta**. `docs/design/w3c/plan.md` wins over this file.
>
> **Feeds plan tasks:** W3c-CYL-SCHEMES (Scope 1 and 3: sweep F1, tet F4, census) and W3c-CYL-PIN (Scope 4, acceptance 2-8: pins, lattice, B5 tests in `sm-server/tests/auto_select.rs`). Scope 2 (the plan-time replan, F3) and acceptance 6 (B6) move to W3c-MESHER-REPLAN (`drafts/auto-b6.md`). The pin milestone script and test move to W3c-INT.

Original size: M. Original owners: sm-scheme-sweep (fix 1), sm-mesher (fix 2, tests), sm-scheme-tet (fix 3, enclosed pin only), sm-server (tests: auto_select B5/B6), sm-cli (new round-pin milestone, optional). Depends: none of W3b.

**Critique delta (owners).** CYL-SCHEMES owns `crates/sm-scheme-sweep/` and `crates/sm-scheme-tet/` (explicit two-crate ownership, one worker). CYL-PIN owns only `crates/sm-server/tests/auto_select.rs` rows b5 and the new pin/lattice rows (test file only). sm-mesher belongs to MESHER-REPLAN. sm-cli (milestone) belongs to INT.

## Goal
`mesh.auto { strategy: "default" }` plus `mesh.run` meshes the natural round pin cell (cylinder in a 1.26 x 1.26 x 1.0 brick) and the 3x3 lattice of it. Through pin (H=1.0) is all-hex. Enclosed pin (H=0.8) is all-tet. B5 (plate with a hole) passes with no extra work.

## Owner
Three small crates, one fix each (details under Scope). Tests live in sm-mesher, sm-scheme-sweep, sm-scheme-tet and sm-server. No frozen-contract change, so no ACR.

**Critique delta.** See the owners note above: the sm-mesher fix is MESHER-REPLAN's.

## Findings (root causes, confirmed by experiment)
I rebuilt a throwaway copy of the repo in the scratchpad (not in the worktree) and applied three patches. Both pin scripts, B5, B6 and both lattices then ran. Nothing in the repo was edited.

### F1. sm-scheme-sweep accepts a face pair that is not a prism (cause of the H=1.0 error)
- Place: `crates/sm-scheme-sweep/src/discover.rs`, `discover_columns`, the final loop (about line 312). It only checks `coedge_count(l) >= 4` for each linking face.
- `scheme.rs::find_columns` (about line 70) tries pairs lowest id first and takes the first one that passes. `analyse` uses it for the applicability score (60).
- Through-pin moderator: the first "valid" pair is two opposite side faces (x-, x+). The annulus faces (top and bottom, with the hole) and the cylinder face then become "linking" faces.
- `require_caps(linking, QUAD | BLOCK_STRUCTURED, Hard)` is imposed on them. This is the `Caps(1026)` in the error. map and submap refuse a face with an inner loop. pave, trimesh, triadvance and tridelaunay are rejected by the imposition. Result: `no applicable scheme for Surface(Face#13)`.
- The correct pair (top annulus to bottom annulus) is valid. I checked it with explicit `source`/`target`: the moderator sweeps fine. Source gets `pave`, target gets `copy`, the cylinder face gets `map`.
- Pair choice is not a W3b mesher-policy problem. It is a missing strip check.

**Critique delta (hash risk).** Because `find_columns` takes the first passing pair (lowest id first), the new strip check can change which pair wins on an input that succeeds today. The W3b milestone has 18 `sweep` volumes whose cause was "not investigated" (W3b report advisory); that is exactly the population at risk for `h_mesh`. The census (acceptance 9) is therefore a blocking first step, run before and after the fix.

### F2. Same missing check causes the H=0.8 "inconsistent orientation" error, in two stages
- The enclosed moderator has 9 faces: 6 outer and 3 cavity faces (two disks and the cylinder face).
- The pair (cavity disk 1, cavity disk 2) passes: each disk has one vertex, and the only linking edge is the seam. The 6 outer faces and the cylinder face become "linking" faces.
- The disks have 1 coedge, so the `< 4 coedges` check should reject them. It does not, because the check only looks at faces other than source and target. With this pair, the outer faces (4 coedges each) and the cylinder face (4 coedges: seam twice) all pass.
- So `sweep` scores 60 for the moderator. It generates a bogus mesh that shares its source with the cylinder volume's sweep. `MeshBuilder` then reports `InconsistentOrientation(FaceIx(0))`.
- The report called it "internal-looking" for this reason: nothing refuses the bogus sweep up front.
- With F1's fix the error becomes a clean `no applicable scheme for Surface(Face#9)`. Face 9 is the shared cylinder face. The cylinder volume imposes quads (sweep) and the moderator imposes triangles (tetmesh).

### F3. Plan-time failures are never re-planned (B6 root cause; also needed for the enclosed pin)
- Places: `crates/sm-mesher/src/auto.rs` `scratch_plan` (about line 189) and `crates/sm-mesher/src/mesh_ops.rs` (about line 337).
- Both call `driver.plan(..)?` once, then `mesh_from_plan`.
- The re-plan loop (`sm-scheme-api` `driver.rs` `mesh_loop`) only runs for failures after a successful first plan. A `NoApplicableScheme` found while planning (blame = `imposed_by.first()`, see `error.rs:350`) returns immediately and is never blamed or retried.
- After F1, face 9 fails exactly this way. Blame would go to the cylinder volume's sweep, which would then fall back.
- Same cause as B6 (auto next to a hard tet neighbour).

**Critique delta (blame order, missed by this draft).** `blameable_imposers` sorts by target (`planner.rs:1550-1573`), so `blame()` (`error.rs:350`) takes the lowest target id. Face 9 is imposed by the cylinder's `sweep` (QUAD) and the moderator's `tetmesh` (TRI). If the moderator sorts first, its `tetmesh` is rejected; the moderator then has no scheme left (sweep is NotApplicable after F1 and map refuses it), and the run fails. The experiment passed, which may be an artefact of the creation order used. The fix (owned by MESHER-REPLAN): after a failure on the same face, try the remaining entries of `NoApplicableScheme.imposed_by` (a public field) in order instead of stopping at `.first()`. The pin acceptance runs brick-first and cylinder-first creation.

**Critique delta (where the replan goes).** Only `scratch_plan` (`auto.rs`). `mesh.auto` stores explicit assignments (Q5), so the quad/tri conflict on face 9 is already resolved by `mesh.run` time; unassigned volumes reach only `[map]` (`standard_schemes`, `standard.rs:37-48`), and blame skips hard assignments. `mesh_ops.rs` is not edited. If the enclosed pin does not pass without a `mesh_ops.rs` edit, the worker stops and reports.

### F4. sm-scheme-tet refuses a cylinder (enclosed pin only)
- Place: `crates/sm-scheme-tet/src/scheme.rs:174`: `if u.uses.len() < 4 { "a closed shell needs at least four faces" }`.
- A cylinder has 3 faces. The enclosed fuel cylinder cannot fall back to tets, so the quad/tri conflict on face 9 has no resolution.
- Lowering the limit to 3 works in the experiment. A tet mesh of the moderator alone (with its cavity) is already valid today, so multi-shell volumes are not a problem.

### Not causes
- pave, tri and map already handle holes and periodic faces. Face 9 meshed by `map` as a periodic face works.
- The `keep: true` note from the orchestrator is correct. The report's repro recipe should say "subtract without keep, then imprint and merge".

## Scope
1. **sm-scheme-sweep (F1).** After the existing coedge check in `discover_columns`, reject a pair unless every linking face has exactly one loop and at least 2 column coedges (a seam counts twice).
   - Add the reasons `linking face {l} has inner loops (not a strip)` and `linking face {l} has {n} column coedges`.
   - Do not require every other edge to lie on the source or target. My first, stricter version broke 3 existing tests in `tests/bricks.rs`: multi-step columns and edges at mid-height from neighbours.
   - Update the DESIGN.md pipeline item 1.
   - **Critique delta:** report error-text changes too: an explicit `sweep` on an invalid pair now fails at plan time, not at generation.
2. **sm-mesher (F3).** Add a helper `plan_replanning(driver, input, intent) -> Result<Plan>`:
   - loop: `plan_with(rej)`;
   - on `Err(e)` with `e.blame() = Some((t, s))`, `s != copy`, not already rejected and the budget not exhausted, insert `Attempt::new(t, s, Phase::Resolve, FailureKind::NotApplicable, e.to_string())` and retry;
   - otherwise return `Err(e)`.
   - `Attempt::new` exists, so no ACR is needed.
   - Use the helper in `scratch_plan` and in `mesh.run`. The first plan is unchanged when it succeeds, so successful runs are bit-identical.
   - Pass the rejections on: `mesh_from_plan` already reads `plan.rejections`.
   - **Critique delta:** moved to MESHER-REPLAN (`drafts/auto-b6.md` Scope "B6", with its shared budget and the `imposed_by` iteration). `scratch_plan` only; not `mesh.run`.
3. **sm-scheme-tet (F4).** Replace `< 4` by a check that the shell is closed (`shell_closed` already exists). A minimum of 3 faces is enough for the cylinder.
   - Run the tet spike below before committing to it.
   - **Critique delta:** the shell rule (any closed shell of 3+ faces vs periodic shells only) is decided by the tet spike (acceptance 10), not by the user.
4. **sm-server tests.**
   - `b5_plate_with_a_hole_pinned_outcome` in `crates/sm-server/tests/auto_select.rs` becomes a pass test.
   - `b6_auto_next_to_a_hard_tet_neighbour_pinned_outcome` also changes (see B6 below). **Critique delta:** b6 belongs to MESHER-REPLAN.

## Contract changes / ACRs
None.
- `Attempt::new` and `Rejections::insert` are public.
- sm-scheme-api is untouched. A `Driver::plan_replanning` there would be tidier. Offer it as an optional additive ACR, but it is not needed.

## Acceptance (numbered, each a test)
1. **Sweep strip check (sweep unit tests).**
   - Brick with a through cylinder hole (1.26 x 1.26 x 1.0, r = 0.4): the pair (x-, x+) is rejected with "inner loops". The auto pair is (top annulus, bottom annulus). Exactly 1 valid pair.
   - Enclosed-cavity volume: `sweep` is NotApplicable.
   - Every existing sweep test and golden is unchanged.
2. **Through pin, all-hex.**
   - Script from the task with H=1.0 and the unchanged `mesh.auto` / `mesh.run`.
   - Observed: 2 volumes, 220 hexes, `validate` empty, total volume 1.5876 (brick volume, to 1e-12). Interface faces conformal (the shared cylinder face is quads on both sides).
   - Plan: the cylinder is a `sweep` of a `pave` disk with a `copy` target. The moderator is a `sweep` of a `pave` annulus with a `copy` target. Faces 1-4 and the cylinder face are `map`.
   - Volume check: the exact moderator volume is 1.5876 - (n/2) r^2 sin(2 pi / n) with n from the mesh (here n = 12, giving 1.1076). The test also checks it is within 2.2 percent of 1.5876 - pi r^2 = 1.0850 for that size.
   - **Critique delta:** cell counts here and in 3-5 (220, 2542, 432, 6647, 154) were observed on the throwaway patch. They are pinned as observed values, re-observed on the final merged code (CYL-SCHEMES + MESHER-REPLAN), not copied from this draft.
3. **Enclosed pin, all-tet.**
   - H=0.8 (needs F2 and F3 and F4): 2542 tets, `validate` empty, total volume 1.5876 to 1e-12, conformal.
   - `mesh.auto_tet` warning for both volumes, quoting why sweep/map were passed over.
   - What is reachable: all-tet only. Hex fuel with a tet moderator needs pyramid or transition cells, which are deferred. All-hex needs an O-grid style webcut decomposition (model work), which is not in this task. Record both as W4 items.
   - **Critique delta:** run with brick-first and with cylinder-first creation order (blame order depends on entity ids); both must pass with the same cell kinds. Must pass with no edit to `mesh_ops.rs`.
4. **3x3 lattice of round pins** (`model.copy` as in the milestone, size 0.3).
   - Through: 18 volumes, 432 hexes, valid, volume 9 x 1.5876 = 14.2884 (observed 14.288400000000074).
   - Enclosed: 18 volumes, 6647 tets, valid.
5. **B5 moves from limit to pass.**
   - 4x4x1 minus an r=1 cylinder, size 0.5, `mesh.auto`: 154 hexes, valid, volume 13.0 (= 16 - 3, polygon of 12 sides; within 1.1 percent of 16 - pi).
   - Replace `b5_plate_with_a_hole_pinned_outcome`, and keep the tetmesh workaround as a second assertion.
6. **B6** (folded in, same function).
   - Observed with F3 only: the right volume meshes. 160 tets plus 32 wedges, valid, conformal. The right volume sweeps with wedges, not tetmesh as the old acceptance text said.
   - Update `b6_...`: no error, validate empty, volume 16 to 1e-9, scheme outcome pinned as observed.
   - Decision: see Question 2.
   - **Critique delta:** moved to MESHER-REPLAN (`drafts/auto-b6.md` acceptance 6). The two prototypes found the same mesh `{Tet:160, Wedge:32}` independently.
7. **Replan count.** Assert `plan.rejections` is non-empty for the enclosed pin and B6 (the rejection of the sweep is recorded). Note: `mesh.info.rejections` printed 0 in my runs, so decide which field the test reads.
   - **Critique delta (decided):** the test reads `MeshEntry::plan.rejections` through `mesh.info`. If `mesh.info` does not expose the plan's rejections, this item is dropped (no new query field is added for it); the worker says which in the commit.
8. **Determinism.** For acceptance 2, 3, 4 and 5: two runs and `RAYON_NUM_THREADS=1` vs default, with identical state hashes. CLI, MCP and in-process agree.
9. **Census.** Full `cargo test` of sm-server, sm-cli (milestone, milestone_w3a, p0_rows, `milestone_w3b`), sm-mesher, sm-scheme-*: no golden or hash moves except the `b5` and `b6` pinned tests. Any other change stops the task.
   - **Critique delta:** this is the first step of CYL-SCHEMES and a blocking gate. Record `milestone`, `milestone_w3a`, `milestone_w3b` (`h_model`, `h_auto`, `h_mesh`, `SUBMAP_GRADED`) and `p0_rows` on `d7ad022` before the fix and again after it. A moved hash stops the task and goes to the user, never a re-bless (Q5 requires explicit-scheme bit-identity).
10. **Tet spike (F4).** A lone cylinder with `tetmesh`: valid, volume equals the polygon volume exactly. Also a lone moderator with a cavity, plus a hard-tet fuel next to a tet moderator.
11. Clippy `-D warnings`, rustdoc, DESIGN.md sections in sweep, mesher and tet.

## Gates / dependencies
- None on other W3c tasks. Run before the DAGMC smoke benchmark, which would otherwise have no round pins.
  - **Critique delta:** dropped. sm-rt-bench does not use the mesher (no dependency on sm-mesher, sm-server or sm-scheme-*; its P1 and H1 twins are hand-built in `twin::` and `cases.rs`). CYL-PIN (tests) depends on MESHER-REPLAN and CYL-SCHEMES.
- Test status of my experiment: `sm-scheme-sweep` and `sm-mesher` (including all their test binaries) pass with the final patches. `sm-scheme-tet` passed in the first run; the strip check was loosened afterwards and I did not re-run it separately. I started `sm-server`, `sm-scheme-api` and `sm-cli` (milestone, milestone_w3a, p0_rows) in the throwaway copy, but the debug-profile `milestone_w3a` run had not finished when I stopped. The census in acceptance 9 is therefore still to do. Do it first.

## Hash impact
- No existing state hash should move, by construction:
  - F1 only changes inputs where a linking face has an inner loop or no column coedge. Today those always fail (map refuses inner loops and the imposition blocks the rest) or produce an invalid mesh.
  - F3 only runs when the first plan failed.
  - F4 only affects shells with 3 faces, which were refused.
- Exception to flag: a successful input could change only if the auto pair choice used to land on a face pair that is now rejected AND the resulting mesh still worked. I found no such case in the sweep and mesher test suites. The census (acceptance 9) must confirm for sm-server and sm-cli.
  - **Critique delta:** "by construction" is not established for F1 (first passing pair wins; see F1 delta). The census is blocking, and a moved hash is a stop.
- Plans are not hashed (`MeshEntry::plan`), so the extra recorded rejections do not matter.
- `mesh.auto` stores explicit assignments, so replay hashes are unaffected.

## Size
M. About 20 lines of code in sweep, about 25 in mesher, 1 line in tet, plus tests and DESIGN.md text. Most of the effort is the census and the new tests.

**Critique delta:** CYL-SCHEMES is S (sweep + tet + census); CYL-PIN is S (tests).

## Risks
- F4 (tet on 3 faces) is verified only through the whole pin and lattice runs. A lone cylinder tet mesh needs the spike (acceptance 10).
- `mesh.run` also gets the plan-time replan. It is behaviour-neutral for working inputs, but it turns some errors into successes. Make sure no test pins one of those errors (the census will show). **Critique delta:** void; `mesh.run` gets no replan.
- The mixed outcome in B6 (wedges next to tets) is valid but may surprise. If hex-only or tet-only is preferred, the policy must say so.
- The enclosed pin ends all-tet, which is poor for a reactor use case. All-hex needs model decomposition and belongs in W4.
- Throwaway patch is not the final code. Names and error texts above are suggestions.
- **Critique delta (new):** blame order depends on entity ids (F3 delta); covered by the two creation orders in acceptance 3.

## Milestone question (recommendation)
Keep `tools/w3b_milestone.smesh` and its hashes as is. Add a second script `tools/w3c_pin.smesh` and a test `crates/sm-cli/tests/milestone_w3c_pin.rs` for the round through-pin lattice (18 volumes, 432 hexes, 14.2884). Use it for the DAGMC smoke benchmark. Do not switch the existing milestone to round pins, because that would move every W3b hash and the C/Python/viewer rows.

**Critique delta:** the script and test are written by W3c-INT (workers do not edit `tools/`, and sm-cli test files are integration's). "Use it for the DAGMC smoke benchmark" is dropped (the benchmark does not use the mesher).

## Doc fix
Correct the W3b report repro recipe: use `model.subtract` without `keep`, then imprint and merge. With `keep: true` the original target stays and the volumes overlap.

**Critique delta:** belongs to W3c-CLEAN-DOCS (orchestrator).

## Questions
- q1 Should the enclosed pin (H=0.8) accept an all-tet result for now, with the all-hex O-grid decomposition deferred to W4? (rec: Yes. Hex fuel with a tet moderator needs transition cells (deferred) and hex-only needs webcut decomposition. All-tet is reachable now with a one-line tet change. The through pin, which is the usual reactor case, is all-hex.) → plan C1.
- q2 Fold B6 (auto next to a hard tet neighbour) into this task, since the same plan-time replan fixes it and its pinned test must change anyway? (rec: Yes. Same function, tiny extra work. Pin the observed outcome (tets plus wedges, valid, conformal) and note that the old text expecting tetmesh is outdated.) **Critique delta:** answered by the plan (B6 goes to MESHER-REPLAN); the outcome question is plan C2.
- q3 Should the round-pin lattice be a new milestone script and test, leaving the existing W3b milestone and its hashes untouched? (rec: Yes.) → plan C4.
- q4 Should the tet scheme be allowed to mesh any closed shell of 3 or more faces, or only periodic (seam) shells such as a cylinder? **Critique delta:** dropped from the user list; decided by the tet spike (acceptance 10).
