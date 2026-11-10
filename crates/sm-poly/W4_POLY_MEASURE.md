# W4-POLY-MEASURE: measurements and pins

Base `1a76bc2`. Numbers are measured, not asserted (the table test is `#[ignore]`d:
`cargo test -p sm-poly --release --test measure -- --ignored --nocapture`; the three scripted rows come from
`mesh.poly_info` on the CLI, scripts are the committed fixtures with `keep_source: true`).

## Table

Poly meshes: the library rows (tet_block, warped) use `PolyOptions::new()` (`Planar`, no agglomeration, without `with_all_polyhedra(true)`); the CLI rows use the `mesh.to_poly` op, which sets `with_all_polyhedra(true)`, so hub-split share and faces/cell differ slightly between the two groups. Eligibility rates are on the source tet mesh.
"convex" = all faces planar and no vertex outside a face plane (the `sm-rt` test without the face cap);
"fast path" = convex and at most 64 faces; "over 64" = more than 64 faces; "convex over 64" = convex but over the cap
(what raising `CONVEX_CHECK_MAX` would recover).

| model | poly cells | convex | fast path | over 64 faces | convex over 64 | hub-split share | faces/cell (mean, max) | non-Delaunay faces / interior faces | outside circumcentres / tets |
|---|---|---|---|---|---|---|---|---|---|
| `tet_block` 4^3 (Kuhn) | 125 | 0.264 | 0.264 | 0 | 0 | 0.598 | 18.5 | 0 | 0 |
| `tet_block_warped` 40^3, amp 0.15 | 68921 | 0.0001 | 0.0001 | 0 | 0 | 0.889 | 51.2 | 0.248 | 0 |
| `polydual` (brick 10x10x6, size 1.0) | 1051 | 5 / 1051 = 0.005 | 0.005 | 196 / 1051 = 0.186 | 0 | 0.955 | 47.2, 108 | 442 / 7990 = 0.055 | 3 / 4292 = 0.0007 |
| `w3a_poly` (path B, size 1.0) | 3982 | 16 / 3982 = 0.004 | 0.004 | 741 / 3982 = 0.186 | 0 | 0.953 | 47.4, 120 | 1504 / 30788 = 0.049 | 51 / 16338 = 0.003 |
| W3c enclosed pin (733 tets) | 301 | 30 / 301 = 0.100 | 0.100 | 20 / 301 = 0.066 | 0 | 0.943 | 36.9, 92 | 102 / 1342 = 0.076 | 59 / 733 = 0.080 |

W4-REFCORPUS models: the corpus has not landed at `1a76bc2` (`tools/refcorpus/` does not exist), so there are no rows for it.
Rerun the table test after it lands; the `row` helper takes any `MeshRead`.

## Reading

- Kill criterion (draft): "most cells of real duals are already fast-path eligible" is false. 0.4 to 10 percent of the cells of the real
  duals are eligible, so VORO is not dropped.
- `CONVEX_CHECK_MAX` lever: not worth raising. In every row the number of convex cells over the 64-face cap is 0. All cells over the cap
  (up to 19 percent of the cells) are non-convex, so a larger cap recovers nothing.
- Hub splitting is the cause: 94 to 96 percent of the interior faces of the real duals belong to a multi-face neighbour pair.
- Trigger for VORO-CLIP / POLY-CONFORM (ruling D1/D10): on the real tet meshes 5 to 8 percent of the interior faces are non-locally-Delaunay
  and 0.07 to 8 percent of the tets have an outside circumcentre. Non-Delaunay faces dominate on `polydual` and `w3a_poly`
  (442 faces against 3 tets; 1504 against 51), while on the enclosed pin both are material (102 faces, 59 tets = 8 percent).
  The strict gate would refuse all four real meshes. The orchestrator decides.
- The perturbed block is a random-perturbation stress case, not a mesher output; its 25 percent non-Delaunay share is from the perturbation.

## Pins on poly output (starting list of the draft, checked)

Tests and goldens that a change of poly geometry or topology moves (none of them is touched by this task):
- `crates/sm-poly/tests/acceptance.rs`: `block333` counts (375 faces at `:93`, 464 points, 279 edge faces, 64 cells, 96 boundary faces; `Planar` by formula at `:150`), `acc7_determinism` (topology and geometry hashes of `tet_block`, `tet_block_two_volumes`, `tet_block_warped`, `tet_lshape`, Keep and Planar; census rows `poly|acc7.*`), `acc1_single_tet` (18 faces).
- `crates/sm-poly/tests/{agglomerate,robust,shapes,starness,props,ops,resolvers,perf}.rs`: counts and invariants of the same fixtures; `ops.rs` `acc8_poly_info_equals_poly_stats` compares the query with the library (both gained the same fields).
- `crates/sm-cli/tests/milestone_w3a.rs` (tet to poly fork and replay, INT-1 `cells = pairs + wedges - split_cells` from `mesh.poly_info`, row 12 `poly_info` vs `poly_stats`), the `w3a_poly` and `w3a_tet` layer hashes in `tools/platform/expected.txt`.
- Conformance fixtures `tools/conformance/fixtures/polydual.smesh` (sha256 in the census) and `w3a_poly` (via `crates/sm-cli/tests/w3a_support`), with the OpenFOAM `checkMesh` concave-cell warnings (1046 and 3966, `docs/design/w3/w3a-report.md:42`).
- `crates/sm-server/tests/{export_wire,auto_select}.rs` (poly export through the wire).
- Not a pin on the mesh but on the `mesh.poly_info` JSON: it grows by the fields above; no test lists its key set exactly (`milestone_w3a` row 11 reads `wedges` and `split_cells` by key).

## Notes

- `mesh.quality` is unchanged: the convexity rule is not a `Metric` and is in no profile `rules`. No `mesh.quality` schema golden was touched.
- Draft acceptance (a) states the dent value as `d / V^(1/3)`. The metric as defined (a vertex distance over the face planes) gives
  `2 d / sqrt(1 + 4 d^2) / V^(1/3)` for the dent over a whole unit face; the test pins that value to 1e-12 and the derivation is in the test.
