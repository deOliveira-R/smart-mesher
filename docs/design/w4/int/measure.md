# W4-INT MEASURE table and dual eligibility rates (plan.md section 5 item 5)

Inputs for three orchestrator decisions: VORO-CLIP / CONFORM (ruling D1, trigger-based), `CONVEX_CHECK_MAX`, and
the circumcentric default (ruling D10). This file states each trigger, says whether the data meets it and recommends.
**It decides nothing.**

## What was measured, and how

- Code: `714e2af` for the original rows; the refcorpus rows were run at `02bbd50` (no difference under `crates/` or `tools/` from `714e2af`; `main` `b51806f` the same for code). Release `smesh` (`crates/sm-cli`), run per mesh as a script; the
  only new files are in `docs/design/w4/int/`. No crate was edited.
- Per mesh, three calls on the tet source or the hex mesh: `mesh.poly_info` on the source (carries `dual_eligibility`
  for an all-tet mesh: exact `insphere` non-Delaunay count and exact `orient3d` outside-circumcentre count, W4-POLY-MEASURE),
  `mesh.to_poly {keep_source: true}` (barycentric, the default) then `mesh.poly_info` on the dual (convexity in the `sm-rt` sense,
  fast-path share, over-64-face share, convex-over-64, hub-split share, faces per cell), and `mesh.to_poly {dual: "circumcentric"}`
  (the strict gate; what it answers is recorded in the last section).
- Definitions (`crates/sm-poly/W4_POLY_MEASURE.md`): **convex** = all faces planar and no vertex outside a face plane (the `sm-rt` test
  without the face cap); **fast path** = convex and at most 64 faces (`CONVEX_CHECK_MAX`, `sm_quality::CONVEX_CHECK_MAX`);
  **over 64** = more than 64 faces; **convex over 64** = convex but over the cap (what raising `CONVEX_CHECK_MAX` would recover).
- Meshes. Census meshes: `polydual` (conformance fixture script, brick 10x10x6, size 1.0), the W3a part tet branch whose dual is
  `w3a_poly` (path B, size 1.0, script emitted by `milestone_w3a::emit_conformance_scripts`), `w3c_pin_enclosed` (733 tets). Milestone-like
  meshes: the OpenMC pin deck family meshed all-tet by `mesh.size` + `mesh.auto {strategy: "tet"}` (a generated 1x1 and 2x2 pin lattice, radii
  0.3, 0.4, 0.5, pitch 1.26, h = 1.0 with a void gap, the deck of `crates/sm-csgbrep/tests/common/decks.rs::pin_lattice`, imported
  with `smesh import-openmc`; sizes 0.15 and 0.1) and the all-hex pin and lattice meshes `w3c_pin`, `w3b_milestone`, `w4_pin_ogrid`,
  `w4_pin_enclosed_hex`, `w3c_warped`.
- Numbers for `polydual`, `w3a_poly` and `w3c_pin_enclosed` reproduce the a1 table of `crates/sm-poly/W4_POLY_MEASURE.md` (measured at `1a76bc2`) to every
  printed digit, so the barycentric dual and the eligibility counts did not move between `1a76bc2` and `714e2af` (consistent with C3).
- **Not measured, and why.** (a) `tools/w4_poly.smesh` and its milestone test do not exist yet (they are W4-INT item 1, another worker). The nearest
  real thing is the committed OpenMC fixture `crates/sm-csgio/tests/fixtures/openmc/pin` (the item-1 deck), meshed all-tet with `mesh.size 0.04` and no
  interval pins (601850 tets, about 20 s in release; the row "csgio fixture pin size 0.04"); the item-1 test will pin intervals on the circles, so its mesh differs and its numbers
  will differ somewhat. The generated wide-gap pin decks are smaller stand-ins. (b) The three W4-REFCORPUS OpenMC decks `openmc-pin`, `openmc-pin-layered` and `openmc-mini-3x3` do not give a row: tried at one setting only, `intervals 16` on every circle with size 0.3, the all-tet strategy refused them at the surface quality gate ("minimum angle 5.030 deg is below the gate 12.000 deg"; for the layered pin "advancing front stuck", 0.174 deg), because their gap and ring widths are 0.008 to 0.02 cm and a conforming mesh needs far finer sizes (about 1e6 tets, as for the P1 twin in `crates/sm-rt-bench/DESIGN.md`); finer sizes were not tried, and `openmc-assembly-17x17` was not tried at all. The clean STEP entries of W4-REFCORPUS (`step-clean-pin`, `-cladded-pin`, `-assembly-can`, `-grid-spacer`, the ones `refcorpus.md` lists as feeding POLY-MEASURE) all mesh today in seconds and are in the table (sizes in the row names); none was refused. The `step-dirty-*` entries and the `vtb-*`, `beavrs-*` models were not tried. (c) Mixed hex+tet meshes
  (`mesh.to_poly` on mixed input, W4-POLY-MIXED) are not in the table: it covers all-tet sources and all-hex meshes. (d) `mesh.to_poly` refuses an all-hex mesh
  ("needs at least one tetrahedron"), so hex rows have no dual and show the hex mesh itself.
- Thread independence: the three pin-deck rows (source and barycentric counts and the circumcentric refusal text, 9 lines) and the refcorpus `step-clean-pin` size 0.1 run (full JSON output, excluding timing) were re-run with `RAYON_NUM_THREADS=1` and are identical to the default run.

## The MEASURE table

Eligibility columns are on the all-tet source mesh; the dual columns are on the barycentric dual (the op default,
hub-split planar mode, `all_polyhedra`).

| mesh | source cells | tets: non-Delaunay faces / interior | tets: outside circumcentres / tets | dual cells (barycentric) | convex | fast path | over 64 faces | convex over 64 | hub-split share | faces/cell mean, max |
|---|---|---|---|---|---|---|---|---|---|---|
| polydual (10x10x6 brick, size 1.0) | 4292 | 442 / 7990 = 0.055 | 3 / 4292 = 0.0007 | 1051 | 5 (0.005) | 5 (0.005) | 196 (0.186) | 0 | 0.955 | 47.2, 108 |
| w3a_tet part (3 volumes, size 1.0) | 16338 | 1504 / 30788 = 0.049 | 51 / 16338 = 0.0031 | 3982 | 16 (0.004) | 16 (0.004) | 741 (0.186) | 0 | 0.953 | 47.4, 120 |
| w3c_pin_enclosed (733 tets) | 733 | 102 / 1342 = 0.076 | 59 / 733 = 0.0805 | 301 | 30 (0.100) | 30 (0.100) | 20 (0.066) | 0 | 0.943 | 36.9, 92 |
| w3c_pin | 648 | n/a (not all-tet) | n/a (not all-tet) | n/a (to_poly refuses all-hex) | 648 (1.000) [hex mesh itself] | 648 (1.000) | 0 | 0 | - | 6.0, 6 |
| w3c_warped | 125 | n/a (not all-tet) | n/a (not all-tet) | n/a (to_poly refuses all-hex) | 0 (0.000) [hex mesh itself] | 0 (0.000) | 0 | 0 | - | 6.0, 6 |
| w3b_milestone | 675 | n/a (not all-tet) | n/a (not all-tet) | n/a (to_poly refuses all-hex) | 675 (1.000) [hex mesh itself] | 675 (1.000) | 0 | 0 | - | 6.0, 6 |
| w4_pin_ogrid | 2160 | n/a (not all-tet) | n/a (not all-tet) | n/a (to_poly refuses all-hex) | 2160 (1.000) [hex mesh itself] | 2160 (1.000) | 0 | 0 | - | 6.0, 6 |
| w4_pin_enclosed_hex | 400 | n/a (not all-tet) | n/a (not all-tet) | n/a (to_poly refuses all-hex) | 400 (1.000) [hex mesh itself] | 400 (1.000) | 0 | 0 | - | 6.0, 6 |
| deck_pin1 size 0.15 | 5828 | 581 / 11136 = 0.052 | 69 / 5828 = 0.0118 | 1845 | 12 (0.007) | 12 (0.007) | 115 (0.062) | 0 | 0.960 | 42.4, 111 |
| deck_pin2 size 0.15 | 23492 | 2244 / 45534 = 0.049 | 282 / 23492 = 0.0120 | 7415 | 46 (0.006) | 46 (0.006) | 486 (0.066) | 0 | 0.958 | 42.6, 120 |
| deck_pin1 size 0.1 | 16213 | 1596 / 31252 = 0.051 | 402 / 16213 = 0.0248 | 4470 | 27 (0.006) | 27 (0.006) | 503 (0.113) | 0 | 0.959 | 45.8, 105 |
| csgio fixture pin size 0.04 | 601850 | 53245 / 1182629 = 0.045 | 12098 / 601850 = 0.0201 | 141769 | 491 (0.003) | 491 (0.003) | 26193 (0.185) | 0 | 0.960 | 51.0, 132 |
| refcorpus step-clean-pin size 0.2 | 7893 | 712 / 14646 = 0.049 | 10 / 7893 = 0.0013 | 1962 | 9 (0.005) | 9 (0.005) | 345 (0.176) | 0 | 0.958 | 49.4, 114 |
| refcorpus step-clean-pin size 0.1 | 44573 | 3729 / 84560 = 0.044 | 25 / 44573 = 0.0006 | 9931 | 34 (0.003) | 34 (0.003) | 2327 (0.234) | 0 | 0.956 | 52.8, 120 |
| refcorpus step-clean-cladded-pin size 0.1 | 64272 | 5714 / 115849 = 0.049 | 809 / 64272 = 0.0126 | 17368 | 61 (0.004) | 61 (0.004) | 2143 (0.123) | 0 | 0.962 | 48.3, 123 |
| refcorpus step-clean-assembly-can size 0.5 | 60147 | 4736 / 99883 = 0.047 | 927 / 60147 = 0.0154 | 20664 | 237 (0.011) | 237 (0.011) | 15 (0.001) | 0 | 0.965 | 38.5, 71 |
| refcorpus step-clean-grid-spacer size 0.3 | 51945 | 4760 / 97385 = 0.049 | 23 / 51945 = 0.0004 | 12123 | 57 (0.005) | 57 (0.005) | 2435 (0.201) | 0 | 0.956 | 49.4, 117 |
| refcorpus step-clean-grid-spacer size 0.1 | 714237 | 61320 / 1369612 = 0.045 | 200 / 714237 = 0.0003 | 150455 | 571 (0.004) | 571 (0.004) | 39793 (0.264) | 0 | 0.956 | 53.1, 141 |

Reading the columns: "non-Delaunay faces / interior" counts interior faces whose opposite vertex lies strictly inside the neighbour's
circumsphere (exact `insphere`; a lower bound, see `dual_eligibility`); "outside circumcentres / tets" counts tets whose
circumcentre is outside their region (exact `orient3d`). `w3c_warped` is a 5x5x5 warped hex mesh: its faces are non-planar
(max non-planarity 8.5e-4), so none of its cells passes the planar `sm-rt` convexity rule even though it is all hex. The
all-hex rows are what the user's transport code and `sm-rt` get from `hex_only`: 100% convex and fast-path (6 faces each), no hub triangles.

## What the strict circumcentric gate answers on the same meshes

`mesh.to_poly {dual: "circumcentric"}` (opt-in, strict, W4-POLY-VORO) refuses every real all-tet mesh above (13 of 13) and changes nothing
(the state hash is unchanged by a refusal; the message is the `NotOrthogonalDual` report):

| source mesh | refusal |
|---|---|
| polydual | 442 non-Delaunay interior faces, 3 tets with an outside circumcentre |
| w3a_tet part | 1504 non-Delaunay interior faces, 51 tets with an outside circumcentre |
| w3c_pin_enclosed | 102 non-Delaunay interior faces, 59 tets with an outside circumcentre |
| pin deck 1x1, size 0.15 | 581 non-Delaunay interior faces, 69 tets with an outside circumcentre |
| pin deck 2x2, size 0.15 | 2244 non-Delaunay interior faces, 282 tets with an outside circumcentre |
| pin deck 1x1, size 0.1 | 1596 non-Delaunay interior faces, 402 tets with an outside circumcentre |
| csgio `pin` fixture, size 0.04 (601850 tets) | 53245 non-Delaunay interior faces, 12098 tets with an outside circumcentre |
| refcorpus step-clean-pin, size 0.2 | 712 non-Delaunay interior faces, 10 tets with an outside circumcentre |
| refcorpus step-clean-pin, size 0.1 | 3729 non-Delaunay interior faces, 25 tets with an outside circumcentre |
| refcorpus step-clean-cladded-pin, size 0.1 | 5714 non-Delaunay interior faces, 809 tets with an outside circumcentre |
| refcorpus step-clean-assembly-can, size 0.5 | 4736 non-Delaunay interior faces, 927 tets with an outside circumcentre |
| refcorpus step-clean-grid-spacer, size 0.3 | 4760 non-Delaunay interior faces, 23 tets with an outside circumcentre |
| refcorpus step-clean-grid-spacer, size 0.1 (714237 tets) | 61320 non-Delaunay interior faces, 200 tets with an outside circumcentre |

The gate passes only on the analytic fixtures of W4-POLY-VORO (Kuhn block and BCC patch; `crates/sm-poly/tests/voro*.rs`), whose eligibility
counts are 0/0 by construction. On every mesh the tet mesher produced, both counts are non-zero, so the eligibility report says
**barycentric** for all of them. Prediction, not a measurement of that mesh: the W4-INT item 1 milestone test (pin deck, intervals on the circles) will most likely
also get a refusal and should assert `dual: "barycentric"` from the report; the committed `pin` fixture at size 0.04, the closest measured mesh, is refused.

## Decisions, triggers and data

**1. VORO-CLIP and POLY-CONFORM (ruling D1: trigger-based; plan section 8).**

- Trigger text (plan section 8 and `drafts/poly.md`): "refusals dominated by outside circumcentres -> VORO-CLIP; by non-Delaunay interior
  faces -> CONFORM (scope approved under ruling D10)". VORO-CLIP: "most real meshes fail the VORO gate on outside circumcentres".
  CONFORM: "non-Delaunay interior faces (from smoothing and recovery) dominate the refusals".
- Data. 13 of 13 real tet meshes are refused (all of them, strict gate; 7 census/pin-deck meshes and 6 refcorpus STEP meshes). Every refusal has both causes at once. Counts: non-Delaunay
  faces vs outside-circumcentre tets are 442 vs 3, 1504 vs 51, 581 vs 69, 2244 vs 282, 1596 vs 402, 53245 vs 12098 (`pin` fixture), 102 vs 59 (enclosed pin), and on the refcorpus STEP meshes 712 vs 10 and 3729 vs 25 (pin), 5714 vs 809 (cladded pin), 4736 vs 927 (assembly can), 4760 vs 23 and 61320 vs 200 (grid spacer). The
  non-Delaunay share is 4.4% to 7.6% of the interior faces on every mesh (stable with size and model; 4.4% to 4.9% on the six refcorpus meshes). The outside-circumcentre share is 0.03% to 8.1% of the
  tets (1.5% on the assembly can, 1.3% on the cladded pin, 0.03% to 0.04% on the grid spacer, 0.06% to 0.13% on the clean pin) and grows with the boundary-to-volume ratio: 1.2% at size 0.15, 2.5% at 0.1 and 2.0% at 0.04 on the pin family, 8.1% on the 733-tet enclosed pin.
- Does the data meet the triggers? By count, **non-Delaunay faces dominate on every mesh** (1.7x on the enclosed pin, 4.4x to 306x on the
  others (4.4x on the `pin` fixture, 5.1x on the assembly can, 7.1x on the cladded pin, 149x on the clean pin at 0.1, 306x on the grid spacer at 0.1), with the caveat that a face and a tet are different units), so the **CONFORM trigger is met**. The **VORO-CLIP trigger as worded
  ("most meshes fail on outside circumcentres") is met only in the weak sense** that every mesh also has outside circumcentres; those would still refuse
  the gate after a Delaunay restoration unless the boundary is protected. The numbers do not show outside circumcentres dominating on any mesh.
- Recommendation (not a decision): treat the evidence as CONFORM first. Its draft scope is a boundary-protected (Gabriel) Delaunay option plus a final
  Delaunay restoration after smoothing, which addresses both counts at the source; re-run this table on its output before spending on VORO-CLIP
  (L/XL). If the orchestrator holds that orthogonality is a goal and not a requirement (ruling D1), the cheaper reading is to defer both to W5 with
  these numbers as the baseline, because the barycentric dual already produces valid, volume-conserving, `sm-rt`-correct meshes. Ruling D10's
  "optional Delaunay output in `sm-scheme-tet` if MEASURE needs it" is the same item.

**2. `CONVEX_CHECK_MAX` (plan section 8: "MEASURE shows many convex cells over 64 faces").**

- Data: **convex over 64 faces = 0 on every dual** (13 of 13), including the finer meshes (503 cells over the cap at size 0.1 and 26193 at size 0.04 on the pin deck, 2327 on the clean STEP pin, 39793 on the grid spacer at 0.1, all non-convex).
  Cells over 64 faces are 0.07% (assembly can) and 6.2% to 26.4% of the barycentric duals and every one of them is non-convex (hub-split cells; the hub-split share is 94% to 96.5% on every dual); the all-hex meshes have 6 faces per cell.
- Trigger not met. Recommendation: do not raise it; a larger cap recovers no cell. (A circumcentric/Voronoi cell has about 15.5 faces on average, far below the cap,
  so the cap would matter for neither dual.)

**3. Circumcentric default (ruling D10: "circumcentric dual opt-in until W4-INT decides").**

- Data: the strict gate passes 0 of 13 real tet meshes (above); the barycentric dual's fast-path share (convex and at most 64 faces) is 0.3% to 10% of cells
  (polydual 0.5%, w3a 0.4%, enclosed pin 10.0%, pin decks 0.6% to 0.7%, `pin` fixture 0.35%, refcorpus STEP meshes 0.3% to 1.1%); the dual is valid and conserves volume on all of them.
- Recommendation: **keep circumcentric opt-in** (and keep barycentric the default). Making circumcentric the default would turn `mesh.to_poly` into a refusal on every
  mesh the tet mesher makes today, and would move the barycentric hashes pinned by C3 (the `w3a_poly`, `polydual` fixtures and the sm-poly counts). Revisit after CONFORM
  (item 1) shows a mesh family that passes the gate; the decision rule can be "default circumcentric only when the eligibility report says the mesh qualifies, else barycentric"
  (what plan item 1 already does for the milestone), which needs a new op argument value, not a hash move.
- For the transport and ray-tracing goals today, hex meshes (`hex_only`, `ogrid`) are the all-convex fast-path route (100% of cells), the barycentric tet dual is the valid but
  mostly non-convex route (cells go through the exact fan walk), and the circumcentric dual is available only on analytic lattices.

**4. Not covered by this data.** `drafts/poly.md` §W4-POLY-SHAPES (trigger: "VORO shows hex-lattice outputs matter") was not evaluated: it needs a VORO result on a
mesh the gate accepts, and no real mesh is accepted. Mixed hex+tet meshes through `mesh.to_poly` (W4-POLY-MIXED) are not in this table.

## Reproduction

`smesh` release at `714e2af` (refcorpus rows: `02bbd50`, same code); per mesh a script of the form `<model + mesh.run> ; let ti = mesh.poly_info(mesh: m.mesh[0]) ; let pb = mesh.to_poly(mesh: m.mesh[0], keep_source: true) ;
let bi = mesh.poly_info(mesh: pb.mesh[0])`, and a second run with `mesh.to_poly(..., dual: "circumcentric")`. Scripts: `tools/*.smesh` as named above, the polydual script of
`tools/conformance/fixtures/polydual.smesh` (without its fem lines), `w3a_tet.smesh` from `MILESTONE_W3A_EMIT=<dir> cargo test --release -p sm-cli --test milestone_w3a emit_conformance_scripts`, and the pin deck
by `smesh -p proj import-openmc geometry.xml --materials materials.xml` followed by
`mesh.size(targets: all volumes, size: 0.15) ; mesh.auto(strategy: "tet") ; let m = mesh.run(targets: all volumes)`.
Counts are sums over fixed chunks, so they are the same for one thread and many.

Refcorpus rows: the STEP files are the W4-REFCORPUS clean entries built in `~/.cache/smart-mesh/refcorpus/step-clean-*` (staged with `smesh -p proj stage file.step`; sha256 pin `67a67d90...`, cladded-pin `658ed812...`, assembly-can `f4f751dc...`, grid-spacer `463940f2...`). Script per row: `let r = model.import_step(source: {sha256: "<sha>"}, name: "<n>")` ; `mesh.size(targets: all volumes, size: <s>)` ; `mesh.auto(strategy: "tet")` ; `let m = mesh.run(targets: all volumes)` ; then the same `mesh.poly_info`, barycentric `mesh.to_poly {keep_source: true}`, `mesh.poly_info` and circumcentric `mesh.to_poly` calls as above. Wall time (release, this Mac): 0.4 s (pin 0.2) to 37 s (grid spacer 0.1, 714237 tets); all others under 5 s. The sizes are not tuned to a quality target: 0.1 for the pin, cladded pin and spacer, 0.5 for the can, plus a coarser pin (0.2) and spacer (0.3) to show the shares do not depend on size.
