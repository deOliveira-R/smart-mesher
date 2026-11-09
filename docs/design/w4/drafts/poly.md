# POLY: polyhedral meshing in W4 (tet dual, Voronoi dual, agglomeration, quality, exporters)

> Planning draft (W4), area "poly", corrected by the Opus critique. Changes against the original draft are marked **Critique delta**. Base: `main` at `d2f4803`. Model: `docs/design/w3c/drafts/*.md`. Every claim about current code has a file:line. Claims I could not check are marked "unverified".
>
> **Feeds plan tasks:** W4-POLY-MEASURE, W4-POLY-VORO, W4-POLY-VOLDIAG (first sub-wave); W4-POLY-AGG (+ ACR-POLY-LEVELMAP), W4-POLY-SHAPES, W4-POLY-SCHEME (later sub-wave); W4-POLY-VORO-CLIP / POLY-CONFORM (trigger: MEASURE numbers); FLUENT/CGNS (trigger: licence); W5+ items. ACR-POLY2 (optional, sub-wave 0).
>
> **Critique delta (task list).** ACR-POLY1 is removed: `MeshEntry.derived_from` already exists (`crates/sm-layer-api/src/mesh.rs:37`, default `None` at `:58`), and `mesh.to_poly` sets it (`crates/sm-poly/src/ops.rs:100`, commit 5532769). W4-POLY-SMSH is reduced to a note, because `.smsh` 1.1 already has element types and a geometry cache (see Findings). SCHEME and VORO-CLIP are new.

## Goal

The user wants two things: polyhedral meshes for their own neutron-transport code, and ray tracing (`sm-rt`) that beats DAGMC/XDG. Today `mesh.to_poly` gives a valid, conservative, model-classified barycentric (median) dual of a tet mesh. The open question for W4 is whether that dual is the right polyhedral mesh. It is not the best one, and the reason is convexity, together with the face count that hub splitting creates.

## Findings (what is true today)

**What exists**
- `sm-poly` builds one cell per (vertex, region) and one face per primal edge. Boundary fans come from the model's `TopoFace`s. Classification is carried, the conversion is multi-material aware, faces are planar and volume is conserved to rounding (`crates/sm-poly/DESIGN.md:3-26`). There is one op, `mesh.to_poly {mesh, keep_source?}` (`crates/sm-poly/src/ops.rs:33-42`), and one query, `mesh.poly_info` (`PolyStats`, `crates/sm-poly/src/stats.rs:28-60`).
- Dual points are tet centroids, boundary-triangle centroids and edge midpoints (`DESIGN.md:35-43`). An interior edge face through `k` tet centroids is generally non-planar. The op always uses `FaceMode::Planar` (`src/options.rs:7-22`), which replaces such a face by hub triangles about the edge midpoint (`DESIGN.md:102-122`). On the 3x3x3 block this gives 375 faces in `Keep` and 627 in `Planar`: 90 faces are split into 342 hub triangles (`crates/sm-poly/W3_REPORT.md:60`), about 3.4 triangles per split face (`W3_REPORT.md:140`). Two cells can then share several faces (`docs/design/w3/plan.md:970`).
- A cell at a reflex edge that fails the fan rule is split into wedges of tet corners (`DESIGN.md:179-233`). The source must be all-tet: any other cell gives `PolyError::NotTet` (`src/prep.rs:237-238`, `src/error.rs:14`). Hybrid input is a listed known gap (`DESIGN.md:300-307`).
- Agglomeration exists only as a library option (`src/options.rs:81-101`, called at `src/dual.rs:105-106`). It is off by default and not an op argument. Its default concavity limit is 0.1, but unmerged `Planar` cells already have concavity 0.14 (median) and 0.235 (max), so by default it merges nothing. At limit 0.75 it makes 20 merges on `tet_block([4,4,4])` (`DESIGN.md:271-287`).
- `all_polyhedra` is set by `mesh.to_poly`. It subdivides one face of every dual cell that came out as a tet, hex, wedge or pyramid, and the same face in the neighbour, only so that a `POLYHEDRON` block accepts the cell (`src/options.rs:38-44`, `DESIGN.md:146-154`).
- Quality: `sm-quality` has finite-volume metrics (`crates/sm-quality/src/fv.rs:119-149`) and polyhedron metrics (`src/polyhedron.rs:1-5`). The Transport profile grades face flatness, face concavity, starness, edge ratio and volume ratio (`src/profile.rs:263-269`). **There is no cell-convexity metric in sm-quality.** The only cell concavity measure is the agglomeration guard in sm-poly (`crates/sm-poly/src/options.rs:77`, computed in `src/agglom.rs`).
- Exporters:
  - `.smsh` and VTU carry `POLYHEDRON` (`crates/sm-meshio/SMSH_FORMAT.md:175-188`, `PARITY.md:116`).
  - Exodus writes NFACED (`PARITY.md:101`).
  - OpenFOAM polyMesh has `split_warped_faces` (`PARITY.md:102`).
  - Gmsh msh4 cannot carry polyhedra, so a `PolyPolicy` either errors, splits into tets or drops them (`crates/sm-meshio/src/gmsh/mod.rs:28-41`, `docs/design/w3/gmsh-polyhedra.md`).
  - The DAGMC h5m writer fans polygons from a centroid (`crates/sm-meshio/src/dagmc/mod.rs:5`), so it is a facet export.
- **No Fluent or CGNS writer exists.** In `crates/*/src`, "fluent" appears only in quality-profile and CLI/diag enums (`crates/sm-cli/src/args.rs:34-35`, `crates/sm-quality/src/profile.rs:66-67`). PARITY lists both as P2 and open (`PARITY.md:106`). The Fluent research is in `docs/research/fluent-msh.md`, and the golden files wait for a licence (`docs/design/w3/backlog.md:40`).
  **Critique delta:** the original cited `PARITY.md:111` (that line is the git-native format) and `profile.rs:70` (that line is `Transport`).
- **`.smsh` already writes the element type and can carry derived geometry.**
  - `element_types` is "always written (possibly `[]`) in 1.1" (`SMSH_FORMAT.md:71`, §2.9 at `:163-183`).
  - An optional geometry cache holds `gc_face_area`, `gc_face_centroid`, `gc_face_apex`, `gc_cell_volume` and `gc_cell_centroid`, with the rule and the hashes it was computed for (`SMSH_FORMAT.md:259`). It is backed by `sm_mesh_api::GeometryCache`, `compute_geometry_cache` (`crates/sm-mesh-api/src/geometry.rs:225`) and `MeshBuild::set_geometry_cache` (`src/build.rs:522`).
  - Cell tags and level maps are serialized too (`SMSH_FORMAT.md:260-262`).
  - Unverified: whether `mesh.to_poly` or the export path fills the geometry cache today. A grep was not possible in this session.

  **Critique delta:** the original said the element type is not written and that there is no derived geometry. Both claims are wrong at d2f4803. The backlog line it relied on (`backlog.md:13`) is stale.
- The C ABI covers tracking only (`smrt_*` entries in `crates/sm-rt-capi/src`, from `smrt_ctx_new` to `smrt_test_stub`). It has no C topology reader. `crates/sm-meshio/readers/` holds one reference reader, `smsh.py`.

**The convexity finding (central)**
- `sm-rt` takes the Cyrus-Beck walk only for a cell flagged `C_CONVEX`. That needs every face planar, every cell vertex on the inner side of every face plane within `planarity_tol * diam`, and at most `CONVEX_CHECK_MAX = 64` faces (`crates/sm-rt/src/core.rs:21-28`, check at `core.rs:198-210`; `crates/sm-rt/DESIGN.md:51-56`). Every other cell goes through the exact `orient3d` fan walk (`sm-rt/DESIGN.md:57-66`).
- Convexity also gates more than the walk. The safety distance is "exact ... for a convex cell, 0 otherwise" (`sm-rt/DESIGN.md:381`, `crates/sm-rt-api/src/segments.rs:584`). The locate visibility walk uses convex neighbours (`sm-rt/DESIGN.md:101`), and so does majorant-node preassignment (`sm-rt/DESIGN.md:345`).
- Planar-mode dual cells are not convex: hub triangles bulge, giving concavity of 0.14 median and 0.235 max (above). This is not an accident. Wherever a non-planar polygon is cut into planar pieces, the shared surface has a fold, and every non-flat fold is reflex on one of its two sides.
- OpenFOAM `checkMesh` agrees. INT-9 recorded "concave cells" as a warning on the duals: 1046 on `polydual` and 3966 on `w3a_poly` (`docs/design/w3/w3a-report.md:42`). `w3a_poly` has about 4000 cells (4023 on path A, `w3a-report.md:45`). The W3a ruling called these warnings "intrinsic to the exact median dual" and asked to "revisit default agglomeration" (`docs/design/w3/plan.md:1314`).
- Face count. On a Delaunay-like tet mesh an interior vertex has about 15.5 edges on average (Poisson-Delaunay), and a ring has about 5 tets. Almost every ring of tet centroids is non-planar, so a `Planar` interior cell has roughly 15 × 3.4 ≈ 50 to 80 faces. Unverified estimate: many interior cells exceed the 64-face cap regardless of shape. MEASURE measures it. A Voronoi cell averages 15.5 faces (Meijering 1953), well under the cap.
- **Not verified:** the share of `C_CONVEX` cells on a real dual. `TrackerInfo` has `cells` (`crates/sm-rt-api/src/tracker.rs:107-108`) but no convex count. `C_CONVEX`/`c_flags` are `pub(crate)` in sm-rt (`core.rs:21`).
  **Critique delta:** the original ACR asked to add `cells`, which already exists. Only `convex_cells` is missing.

**The "analytic faces" claim**
- `backlog.md:50` says the sm-rt advantage rests on "exact analytic faces on polyhedral cells". There are two separate mechanisms behind this:
  1. Analytic surfaces come from the B-rep-to-CSG bridge and the analytic leaf, which handle planes, spheres, cylinders, tori and quadrics (`sm-rt/DESIGN.md:508-535`, `253-269`; `FacetFallback`, `crates/sm-rt-api/src/input.rs:68-79`).
  2. Polyhedral mesh cells have planar faces only. Non-planar mesh faces are tracked through fan triangles and counted in `TrackerInfo::faceted_faces` (`sm-rt/DESIGN.md:36-41`). `f_surface` marks the model surface of an interface or boundary face (`core.rs:45`), not a curved carrier.

  So on polyhedral cells "exact" means exact planes of a faceted boundary. The benchmark wording should separate "analytic surfaces (CSG/B-rep bridge)" from "exact planar polyhedral cells".
  **Critique delta:** the original said no path carries a cylinder. For mesh faces that is true, but the bridge does carry cylinders, and the corrected wording must not under-claim it.
- Dual boundary points lie on the faceted surface, so curved bodies deviate by the chord error (`DESIGN.md:300-307`). W3c recorded that the fuel volume is compared only in total (`docs/design/w3c/w3c-report.md:68`), and the per-block check is a backlog item (`backlog.md:131`, `:154`).

**Hash exposure**
- Poly output is pinned by:
  - sm-poly count tests (375 asserted at `crates/sm-poly/tests/acceptance.rs:93`; `Planar` asserted by formula at `:150`, not as 627) and `acc7_determinism` (`acceptance.rs:412`);
  - `crates/sm-cli/tests/milestone_w3a.rs` (tet → poly fork and replay, `:263-280`);
  - the conformance fixtures `polydual` and `w3a_poly` (`w3a-report.md:11`; `tools/conformance/fixtures/polydual.smesh`);
  - callers of `mesh.to_poly` in sm-exodus, sm-mcp, sm-py, sm-server and sm-mesher tests.

  `tools/platform/expected.txt` has **no** poly entry (grep empty).
  **Critique delta:** the original said "possibly" in expected.txt (it is not), and it cited a pending `derived_from` ACR with a `DIGEST_VERSION` bump. That ACR already landed (see the task-list note), so W4 has no pending digest move from it. The original cited `backlog.md:70,128` for it; the contract note is `backlog.md:61`.
- Rule from W3c: if an existing hash moves, the task stops and the decision goes to the user. Every geometry-changing task below is therefore opt-in, and the default produces today's bytes. Adding an optional `dual` field to `ToPoly` changes no op hash when it is omitted, which is the same pattern as `keep_source`'s `skip_serializing_if` at `ops.rs:39-41`. The MCP tool golden lists names only (`crates/sm-mcp/tests/golden/tools.standard.txt:23`). Unverified: whether `session.out.jsonl` embeds the `mesh.to_poly` schema. VORO checks this first, and if it does, it is a golden re-bless, not a state hash.

**Cubit comparison**
- Our research records only that Cubit's `polyhedron` volume scheme "auto-sets surfaces; assembly-wide setup required" (`docs/research/cubit-schemes.md:17`). PARITY lists `polyhedron` as open in both the surface (P1, `PARITY.md:66`) and the volume (P2, `PARITY.md:73`) scheme lists. I claim nothing more about Cubit. For quality the honest comparators are OpenFOAM `polyDualMesh` and the Fluent and STAR-CCM+ conversions (`sm-poly/DESIGN.md:5-8`).

## Design

### What a transport code needs (stated as requirements, to be confirmed by the user)
1. **Conformal, classified cells**: one material per cell, interface faces shared. Already met.
2. **Planar faces**: a face has one normal, so a ray or an upwind decision is one plane test. Met (hub triangles), at the cost of several faces per neighbour pair.
3. **Convex cells**: needed for Cyrus-Beck, the exact safety distance and the locate walk, and for sweeps (a well-defined upwind/downwind split per face). Not met.
4. **Positive sub-geometry about the cell and face centres** (fan side tetrahedra): needed by polyhedral FE and mean-value bases. Met (`validate_geometry`; wedge repair, `DESIGN.md:179-198`).
5. **Orthogonality** (face normal along the line between the cell generators): needed for two-point-flux FV diffusion and coarse-mesh acceleration. Irrelevant for pure ray tracing.
6. **Preserved material volumes**: neutronics compares reaction rates per material. Not met for curved regions (chord error).
7. **Cell count**: dual cells come one per vertex, about one fifth to one sixth of the tet count (Peric 2004, ERCOFTAC Bulletin 62). That is a cost reduction, not a guarantee of equal accuracy.
8. **One face per neighbour pair**: simplifies the transport code's connectivity and sweep graph. Not met in `Planar` mode (`plan.md:970`).
   **Critique delta:** added. The Voronoi dual meets this one too.

Requirements 3, 5, 6 and 8 depend on the transport code's discretization; see Question 1.

### Convexity theory that drives the choice
A simple convex polyhedral cell complex in 3D (four cells at every interior vertex) is a power diagram, that is, a weighted Voronoi diagram (Aurenhammer, *Discrete Comput. Geom.* 2, 1987; also the survey in *ACM Computing Surveys* 1991).

**Critique delta (theory).** The original said vertex motion "cannot work in general (overdetermined)". That is wrong. The point positions that make every edge face of a tet dual planar are exactly the orthogonal duals: for each tet, the point equidistant, in the power distance, from its four vertices. Unweighted, that point is the circumcentre. With one weight per vertex it is the weighted circumcentre, and an affine image of such a dual is also planar. So planarizing by moving points is possible, and the circumcentre is that move. What cannot be done is to planarize while keeping the points near the centroids on a non-Delaunay mesh. Weights give one extra degree of freedom per vertex. They can recentre an orthogonal dual of a non-Delaunay but regular mesh (Mullen, Memari, de Goes, Desbrun, "HOT: Hodge-optimized triangulations", SIGGRAPH 2011). That is a W5+ extension (POLY-POWER below), not a W4 task.

**Recommended construction: circumcentric (Voronoi) dual as an opt-in dual-point rule next to the barycentric one.**
- Replace the tet centroid by the tet circumcentre and the boundary-triangle centroid by the triangle's circumcentre (in its plane). Keep the edge midpoint.
- Every point of an edge face is then equidistant from the edge's two endpoints, so each edge face lies exactly in the bisector plane, Delaunay or not. No hub triangles are needed, and each neighbour pair shares one face.
- On a Delaunay mesh whose circumcentres lie inside their regions, the cell is the Voronoi cell clipped by nothing: convex and exactly orthogonal.
- The topology machinery stays: ring walk, slots, boundary fans, classification and planarity decision (`prep.rs`, `polydata.rs`). Only the point positions and a validity gate change.

Details that matter:
- **Degenerate lattices (welding).** On the Kuhn lattice all 6 tets of a cube share one circumcentre, the cube centre. Face-diagonal and body-diagonal edge faces then collapse, and the dual is a pure hex lattice. Merge such points and drop zero-area faces without a tolerance. Tets go in one group when the exact `insphere` (`crates/sm-math/src/predicates.rs:469`) returns zero for the neighbour's opposite vertex. Boundary triangles need the same treatment: on the Kuhn block a boundary triangle's circumcentre is the midpoint of its hypotenuse (the face diagonal), so it coincides with that edge's midpoint and must weld too.
  **Critique delta:** boundary welding added.
- **Non-Delaunay input and the gate.** The tet mesher is Delaunay with exact predicates (`crates/sm-scheme-tet/src/delaunay.rs:1-6`), but boundary recovery and smoothing break the property. Two failure modes exist:
  1. an interior face that is not locally Delaunay (exact `insphere` across the face);
  2. a tet whose circumcentre lies outside its region (exact `orient3d` against the region-boundary facets of the ring; in practice a boundary tet that is obtuse at the boundary).

  Either can make a ring self-overlap or a cell leave the domain.

  **Critique delta (fallback is not feasible).** The original proposed a non-strict mode that "reports and lets the existing wedge repair run". The wedge repair rebuilds a cell from **barycentric** corners: kites `(m_vw, fc, c_t, fc')` through tet centroids in the planes `λ_v = λ_w` (`DESIGN.md:196-201`). A repaired cell would then share faces through centroids with neighbours whose faces run through circumcentres, which is not conformal. It also breaks the original's own rule "never mix rules inside one ring". The circumcentric analogue of a corner is Voronoi ∩ tet, the bisector clip of each tet. That is a different construction (VORO-CLIP below). VORO in W4 is therefore **strict only**: if any failure of either kind exists, the op refuses with a typed `NotOrthogonalDual { non_delaunay_faces, outside_circumcentres, first_at }` and the state is unchanged. The counts are also available as a dry-run report (a `mesh.poly_info`-style query on the tet mesh, from MEASURE), so the user sees beforehand whether a mesh qualifies.
- **Boundary.** Circumcentres of boundary tets often lie outside the domain. The full answer is the restricted Voronoi diagram (Voronoi ∩ region), computed by clipping each tet against the bisector planes of nearby generators, with robust predicates. References: Yan, Wang, Lévy, Liu, SGP 2009 (surface case); Lévy, "Robustness and efficiency of geometric programs: the Predicate Construction Kit", *CAD* 72, 2016. The other answer is to remove the violation at the source with a boundary-protected (Gabriel) conforming Delaunay option in `sm-scheme-tet` (Cheng, Dey, Shewchuk, *Delaunay Mesh Generation*, 2012). Which one is worth building depends on MEASURE's violation rates. Both are trigger tasks (VORO-CLIP, CONFORM).
- **Features.** Keep the model-driven feature rule (`DESIGN.md:10-13`). The circumcentric rule changes positions only.
- **Wedge repair.** It is not used in circumcentric mode. A Voronoi cell restricted to a locally convex region is convex, so the star-about-centroid failures the repair exists for (`DESIGN.md:181-189`) occur only at reflex region edges. In strict mode such a cell fails `validate_geometry` and is refused with `UnrepairedCell` (the existing typed error), counted in the report.
  **Critique delta:** stated explicitly; the original kept `concave.rs` in the circumcentric path.

**Why this is better than Cubit's and polyDualMesh-style duals:**
- it is planar and convex by construction, not planar by cutting and then concave;
- it has one face per neighbour pair;
- it is exactly orthogonal on Delaunay input;
- features come from the model, and multi-material interfaces are handled;
- validity is decided exactly before anything is built.

I cannot say what Cubit's `polyhedron` scheme does. The claim is against the barycentric duals of OpenFOAM, Fluent and STAR-CCM+.

### Alternatives weighed
- **Keep the barycentric dual and repair convexity** (merging kites back, concavity-tuned agglomeration). Agglomeration cannot remove concavity it starts with (`DESIGN.md:283-287`). Keep the barycentric dual as the default because it is valid on any tet mesh, but stop investing in making it convex.
- **Restricted Voronoi of the mesh vertices (VORO-CLIP).** It is valid on any tet mesh, keeps the exact faceted volume, has planar faces, and its cells are convex wherever the region is locally convex. It is the general form of VORO. The cost is constructed intersection points and their robust predicates (PCK-style). Size L to XL. Trigger: MEASURE shows that real meshes mostly fail the strict gate at the boundary.
  **Critique delta:** new; the original had only the free-generator RVD.
- **Restricted Voronoi of free generators with CVT** (Du, Faber, Gunzburger, *SIAM Review* 41, 1999; Liu et al., *ACM TOG* 28, 2009). This would be a true polyhedron scheme independent of the tet mesher. A newer competitor that needs no clipping is VoroCrust (Abdelkader et al., *ACM TOG* 39, 2020). **W5+**.
- **Direct tet agglomeration** (merge tets into convex polyhedra). Planarity comes for free, but cell shapes follow the tet mesh and there is no orthogonality. This is the right tool for coarsening (AGG), not for the first construction.
- **Hex-dominant / hybrid dual.** Needs a transition rule at hex/tet interfaces. W5+ (`DESIGN.md:302`). The W3c round pin comes out all-hex (through pin) or all-tet (enclosed pin) (`docs/design/w3c/drafts/cyl-pin.md:12`), so the all-tet case is the one that matters now.
  **Critique delta:** the original cited `PARITY.md:62` for this; that line is the P0 map scheme.
- **Power (weighted) dual** (HOT, above). It extends VORO to regular non-Delaunay meshes. W5+.

### Quality metrics for polyhedra (what to add)
- `cell_convexity`: the largest outside distance of a cell vertex over the cell's face planes, relative to `V^(1/3)`. It comes with a boolean `fast_path_eligible`, decided by the exact `sm-rt` rule: all faces planar under `planarity_tol`, every vertex within `planarity_tol * diam` of the box diagonal (`core.rs:198-208`), and at most 64 faces. Keep it out of the profile `rules` like `TRANSPORT_EXTRA` (`profile.rs:271-282`), so existing reports and goldens do not move.
- `mesh.poly_info` additions: convex cell count, cells over 64 faces, hub-split share, largest face count, cell pairs that share more than one face.
- A tet-mesh **dual-eligibility report**: interior faces that are not locally Delaunay, and tets whose circumcentre lies outside their region. These two numbers decide between VORO, VORO-CLIP and CONFORM.
  **Critique delta:** added.
- Per-region volume against the model volume (VOLDIAG).

### Block shapes
A `POLYHEDRON` block could accept cells whose face lists form a tet, hex, wedge or pyramid and export them by face lists. Then `all_polyhedra` could be off.

**Critique delta (value and owners).** On barycentric duals of real meshes almost no cell has a fast shape, so the gain shows up mainly on Kuhn circumcentric lattices and agglomerated cubes. The shape check sits in three places:
- `crates/sm-fem/src/lower.rs:199-205`;
- the `.smsh` writer and reader (`crates/sm-meshio/src/smsh/writer.rs:434`, `reader.rs:609`);
- the format rule "every member's `cell_shape` equals `shape`" (`SMSH_FORMAT.md:179-180`).

Relaxing it is a spec change that 1.1 readers would reject, unless the writer encodes such cells with `cell_shape` 4 (polyhedron). The task is M, owns sm-fem and sm-meshio, and moves to the later sub-wave.

### `.smsh` for the transport code
**Critique delta (replaces the original section).** Format 1.1 already holds arbitrary polyhedra, the element type and an optional derived-geometry cache (see Findings), so no format bump is needed. Two things remain:
1. check whether the export path fills `gc_*` for poly meshes (unverified), and fill it if it does not;
2. store the circumcentric generator point as a cell tag (`tag.<i>` f64 can hold three columns or three tags).

Both belong inside VORO and Question 5, not in a separate task.

## Tasks

Common rules (CLAUDE.md applies): determinism test, `sm_testkit::strategies::config(n)`, rustdoc, no wall-clock gates, no `unwrap` on input paths. Hash rule: a moved existing hash goes to the user.

### ACR-POLY2 (orchestrator, sub-wave 0, optional)
- Additive `TrackerInfo::convex_cells: usize` in `sm-rt-api`. `TrackerInfo` is `#[non_exhaustive]` and built with `TrackerInfo::new` (`tracker.rs:99-104`); sm-rt fills it. It goes to `rt.info` only.
- The C struct `SmrtInfo` (`crates/sm-rt-capi/src/types.rs:205-207`, `include/smrt.h:173`) stays unchanged. Adding the count there is an ABI decision, outside this ACR.
- Hash impact: none on mesh, state or `.smrt` bytes; the `rt.info` output grows.
- Without the ACR, MEASURE uses the public `safety_distance` observable instead (below).

**Critique delta:** `cells` removed (it exists), C ABI exposure stated, and ACR-POLY1 dropped (already landed).

### W4-POLY-MEASURE (S/M)
- **Owner:** `crates/sm-quality/` (additive metric) and `crates/sm-poly/` (stats, dual-eligibility report, tests). It reads `sm-rt` only through its public API, as a dev-dependency of sm-poly tests.
- **Scope:**
  1. `cell_convexity` and `fast_path_eligible` as above.
  2. Additive `PolyStats` fields: convex cells, cells over 64 faces, hub-split share, max faces per cell, multi-face neighbour pairs.
  3. The dual-eligibility counts on a tet mesh: non-locally-Delaunay interior faces (exact `insphere`) and outside-region circumcentres (exact `orient3d`), as a library function plus a field of a query.
  4. A measurement table (in the report, not asserted) on `tet_block([4,4,4])`, `tet_block([40,40,40])` perturbed, `polydual`, `w3a_poly` and the W3c enclosed-pin tet mesh. Columns: convex share, over-64 share, the `sm-rt` fast-path share, and the two eligibility rates on the source tet meshes.
  5. A list of every test and golden that pins poly output (starting list under Findings).

  **Critique delta:** the original scope (5) is dropped as done. `UnrepairedCell` maps to `OpError::Failed` (`ops.rs` `refuse`), the post-merge `InvalidAgglomeration` check exists (`dual.rs:105-115`), and `robust.rs` asserts the region (commit 32a7659). Scope 3 is new.
- **Acceptance:**
  - (a) Analytic cells:
    - a unit cube is convex and eligible;
    - an L-shaped prism is not, with excess equal to the hand value;
    - a cube with one face split into two coplanar triangles is convex;
    - a cube with an inward pyramid dent of depth `d = 0.02` on one face has `cell_convexity = d / V^(1/3)` to 1e-12, with `V = 1 - d/3`;
    - a convex prism over a 63-gon (65 faces) is convex with `fast_path_eligible = false`.
  - (b) Agreement with sm-rt, cell by cell, on the fixtures: for a state at each cell's volume centroid, `safety_distance > 0` iff `fast_path_eligible`. This uses the contract observable `segments.rs:584` and `sm-rt/DESIGN.md:381`, because `c_flags` is `pub(crate)`. If ACR-POLY2 lands, `TrackerInfo::convex_cells` equals the metric count too.
  - (c) Eligibility: on `tet_block` (Kuhn), non-Delaunay faces = 0 and outside circumcentres = 0. The Kuhn circumcentres lie on cube body diagonals, and on-boundary counts as inside. A hand-built 2-tet configuration with one flipped face gives exactly 1 non-Delaunay face. One obtuse boundary tet gives exactly 1 outside circumcentre.
  - (d) The column hash is the same for `RAYON_NUM_THREADS=1` and the default.

  **Critique delta:** (b) was not reachable through the public API as written. The dent value is now stated, and (c) is new.
- **Gates:** `cargo build`, `cargo test -p sm-poly -p sm-quality`, clippy `-D warnings`, rustdoc, determinism.
- **Hash impact:** none on mesh or state hashes. `mesh.poly_info` and `mesh.quality` JSON gain fields, and the metric stays out of the profile `rules`. Unverified: whether a `mesh.quality` output-schema golden exists that lists metrics (the W3b-0 re-bless note is `backlog.md:69`). The task checks; if one exists, re-blessing it is a user-visible golden change and is reported.
- **Kill criterion (new):** if the measurements show that most cells of real duals are already fast-path eligible, VORO drops to a later sub-wave. I expect the opposite.
- **Depends:** none.

### W4-POLY-VORO (L)
- **Owner:** `crates/sm-poly/` (new `circum.rs`; changes in `prep.rs`, `polydata.rs`, `options.rs`, `ops.rs`). It may add `sm-scheme-tet` as a **dev-dependency** for the Delaunay test inputs. Unverified whether its Delaunay of a point set is public API; if it is not, the test builds Delaunay inputs from a BCC lattice (below).
- **Scope:**
  - `PolyOptions::dual_points: DualPoints { Barycentric (default), Circumcentric }`.
  - Op argument `dual: "barycentric" | "circumcentric"` on `mesh.to_poly`, omitted from the canonical params when it is the default.
  - Circumcentres for tets and boundary triangles, midpoints for edges.
  - Exact-`insphere` welding of cospherical tet groups (and of boundary circumcentres with edge midpoints), and dropping of zero-area faces.
  - The strict gate with the typed error `NotOrthogonalDual` (counts plus first location).
  - `DualReport` gains `welded_points`, `zero_faces`, `non_delaunay_faces` and `outside_circumcentres` (the last two are 0 on success).
  - The generator point stored as a cell tag (like `poly.wedges`, `DESIGN.md:220`).
  - `all_polyhedra` behaviour unchanged.

  **Critique delta:** there is no non-strict mode and no wedge repair in this mode (see Design). The tag line was `:226`.
- **Acceptance (analytic or exact):**
  1. Kuhn block `tet_block([a,b,c])`. The circumcentric dual equals the cube lattice:
     - `(a+1)(b+1)(c+1)` cells;
     - interior cells have 6 faces and volume exactly 1;
     - face, edge and corner cells have volumes 1/2, 1/4 and 1/8;
     - total volume `abc`;
     - no hub triangle;
     - `welded_points` and `zero_faces` equal the hand counts.

     The cell count before `all_polyhedra` subdivision is checked through `DualReport`. With `all_polyhedra` (the op default) every hex cell gets one diagonal split, so the face count is the hex-lattice count plus one per cell. This number is pinned.
     **Critique delta:** the original ignored that `all_polyhedra` subdivides every hex cell.
  2. BCC lattice (body-centred cubic points, cube edge `h`): its Delaunay tetrahedralization is the disphenoid honeycomb, unique and non-degenerate. The interior circumcentric cells are truncated octahedra: exactly 14 faces (6 squares, 8 regular hexagons) and volume `h^3 / 2`, to 1e-12 relative. Every face is the perpendicular bisector of its two generators, and every interior cell passes the `sm-rt` convex rule. Built in the test as an interior-only patch, with boundary cells excluded from the shape checks but included in the volume sum.
     **Critique delta:** this replaces "Delaunay of a seeded point set". A perturbed Kuhn lattice is not Delaunay in general, so it would not have tested the claimed property.
  3. Delaunay of seeded random points (if the `sm-scheme-tet` dev-dependency works): every cell passes `validate_geometry`; every face is planar to 1e-12 relative; for each face vertex, `|p - g1| = |p - g2|` to 1e-12 relative; the volume sum equals the tet volume to 1e-12. Inputs come from `sm_testkit::strategies::config(n)`.
  4. `max_nonplanarity` is 0 within rounding, with no hub triangles in either face mode.
  5. Non-Delaunay input (a Kuhn block with one hand-flipped face, or a perturbed block checked exactly): `strict` refuses with `NotOrthogonalDual { non_delaunay_faces = hand count }`, and the state hash is unchanged.
  6. Multi-volume Kuhn block and L-shape: interface faces are shared, and classification is the same as for the barycentric dual (existing `acc4_*`/`acc6_*` tests run in both modes where the input qualifies).
  7. With `dual` absent the op reproduces today's counts and state hash exactly: a test pins the hash before the change. Determinism: 1 thread vs N threads.
  8. `sm-rt` on the circumcentric dual of the BCC patch: every interior cell takes the fast path (`safety_distance > 0` at the centroid). The total track length per region along a fixed seeded ray set equals that of the source tet mesh to 1e-9 relative (same faceted region boundary).
     **Critique delta:** the original asked for "chords equal the tet-mesh chords", which is undefined because the cells differ. It also used a perturbed Kuhn block that would be refused by the strict gate.
- **Gates:** as MEASURE, plus `cargo test -p sm-poly --all-features`, and the existing `acc*` tests unchanged.
- **Hash impact:** none for existing hashes (opt-in, default omitted from the params). New circumcentric hashes are new pins (macOS arm64, Linux in `tools/platform`), added by INT. A possible MCP schema golden re-bless (Findings).
- **Depends:** MEASURE (metric and eligibility function).
- **Open risk:** on real CAD tet meshes the strict gate may refuse most inputs, at the boundary. MEASURE's eligibility rates show this before VORO is merged. VORO-CLIP or CONFORM is then the follow-up, and that choice goes to the orchestrator with the numbers.

### W4-POLY-VORO-CLIP (L/XL), trigger-based
- **Owner:** `crates/sm-poly/`.
- **Scope:** restricted Voronoi of the mesh vertices, clipped to each region. Each tet is clipped by the bisector planes of its candidate generators, which are the Delaunay neighbours of its vertices. Exact combinatorics come from PCK-style predicates on constructed points.
- **Trigger:** MEASURE shows that most real meshes fail the VORO gate on outside circumcentres.
- **Acceptance sketch:** it equals VORO wherever VORO succeeds (bitwise on the BCC and Kuhn fixtures); on any `w3a_tet` mesh, every cell passes `validate_geometry`, faces are planar, and the volume equals the faceted volume to 1e-12.

### POLY-CONFORM (cross-area: `crates/sm-scheme-tet/`), trigger-based
- **Scope:** an option for a boundary-protected (Gabriel) Delaunay mesh, and a final Delaunay restoration after smoothing, so VORO's gate passes at the source.
- **Trigger:** MEASURE shows that non-Delaunay interior faces (from smoothing and recovery) dominate the refusals.

**Critique delta:** moved from W5+ to trigger-based in W4, because VORO's usefulness on real meshes depends on either this task or VORO-CLIP.

### W4-POLY-VOLDIAG (S)
- **Owner:** `crates/sm-diag/` (query). The CAD volume comes from `TopologyView::volume_moments` (`crates/sm-topo-api/src/view.rs:115`, `geometry.rs:428`).
- **Scope:** per region (volume and block), the mesh volume against the model volume, as a diagnostic (`mesh.volume_check`, or a field of `mesh.quality`). It works for tet, hex and poly meshes, and quantifies the chord error. It closes the W3c D10 backlog item (`backlog.md:131`, `:154`) on the diagnostic side.
- **Acceptance:**
  - A cylinder whose circle carries `n` equal intervals has mesh volume `0.5 n r^2 sin(2 pi / n) H` to 1e-12 relative, and the deficit against `pi r^2 H` equals the analytic difference.
  - A brick has deficit 0 to 1e-12.
  - The tet mesh and its dual (both modes) report the same volumes to 1e-12.
- **Hash impact:** none (query).
- **Depends:** none. The fix itself (volume-matched tessellation) belongs to `sm-tess`; see Question 2.

### W4-POLY-AGG (M) + ACR-POLY-LEVELMAP (orchestrator), later W4 sub-wave
- **Owner:** `crates/sm-poly/` (new op `mesh.agglomerate`); convexity from MEASURE.
- **Scope:**
  - A reachable agglomeration op with a target cell count or a size field (`sm-sizing`).
  - Merge gate: the exact `sm-rt` convex rule on the merged cell (no separate concavity number). Never across regions; interface faces kept.
  - Output: a new mesh entry plus a fine→coarse **`LevelMap`** with an optional face map (`crates/sm-mesh-api/src/hierarchy.rs:1-30`). It was written for CMFD (coarse cells, surface currents through the face map), is validated against both meshes' connectivity hashes, and is serialized by `.smsh` (`SMSH_FORMAT.md:262`).
  - It replaces the concavity-0.1 guard that merges nothing today.

  **Critique delta:** the original used a cell tag for the map; the existing `LevelMap` is the right carrier. `MeshLayer` has no field for level maps (none found in `crates/sm-layer-api/src/mesh.rs`), so storing one in session state needs **ACR-POLY-LEVELMAP** in the frozen `sm-layer-api`: a level-map table keyed `(fine, coarse)` that is hashed. Until that lands, AGG can emit the map only through export, which is weaker.
- **Acceptance:**
  - The circumcentric Kuhn lattice `4x4x4` (125 cells) merged by 2x2x2 blocks gives the hand count of coarse cells. The 8 interior coarse cubes have volume 8 and are convex, and the `LevelMap` is exact, including its face map.
  - The cell count drops by exactly `merges`.
  - The volume sum, the interface faces and `validate_geometry` are preserved.
  - Nothing merges across volumes (existing test kept).
  - Determinism.
- **Hash impact:** new op, new hashes. The ACR moves the mesh-layer digest **only if** an empty level-map table enters the digest. It must be designed so that an empty table hashes as before, and a test pins that.
- **Depends:** MEASURE, VORO, ACR-POLY-LEVELMAP, and Question 4.

### W4-POLY-SHAPES (M), later sub-wave
- **Owner:** `crates/sm-fem/` (`lower.rs:199-205`), `crates/sm-meshio/` (`.smsh` writer/reader rule and spec, VTU), `crates/sm-exodus/` (NFACED), `crates/sm-poly/` (option).
- **Scope:** a `POLYHEDRON` block accepts any 3D cell and every exporter writes such cells by face list. The op gains `keep_fast_shapes: bool` (default false, so today's output is unchanged).
- **Acceptance:**
  - With `keep_fast_shapes: true` on a Kuhn circumcentric dual (pure hexes), there are `(a+1)(b+1)(c+1)` cells and exactly the hex-lattice face count.
  - Round trip through `.smsh` and VTU preserves cell volumes to 1e-12.
  - OpenFOAM `checkMesh` (conformance harness) reports `Mesh OK.` with matching cell and face counts.
  - A 1.1 reader still reads files written with the default options.
  - The default op output is identical to today.

  **Critique delta:** the original asked for "checkMesh preserves volumes to 1e-12". checkMesh prints a few digits only, so it judges OK and counts, and our own round trip judges volumes.
- **Hash impact:** none by default. If the `.smsh` rule must change, that is a format decision for the user.
- **Depends:** VORO.

### W4-POLY-SCHEME (S/M), later sub-wave
- **Owner:** `crates/sm-mesher/`, with sm-poly through its public API.
- **Scope:** `PLAN.md:75` names "polyhedral meshing (tet dual) + polyhedron scheme". Today the user needs two ops and gets two mesh entries (`DESIGN.md:248-251`, the `ambiguous-mesh` consequence). The scheme `mesh.scheme(scheme: "polyhedron", dual?)` makes `mesh.run` produce tets and dualize them in one step, with `keep_source: false`. That closes the volume `polyhedron` row of PARITY (`PARITY.md:73`) by our own definition.
- **Acceptance:** for a fixed script, `mesh.run` with `polyhedron` gives the same mesh hash as `tetmesh` followed by `mesh.to_poly(keep_source: false)`; one mesh entry; `session.export` without a mesh name works.
- **Hash impact:** new scheme, new hashes only.
- **Depends:** none; uses VORO's `dual` once it lands.

**Critique delta:** new; the original did not address the PLAN item.

### W4-POLY-FLUENT (M) and CGNS NGON/NFACE (M), trigger-based
- **Owner:** `crates/sm-meshio/`.
- **Scope:** a Fluent `.msh` ASCII writer with polyhedral cells (`docs/research/fluent-msh.md`; ASCII first per `backlog.md:39`), and CGNS NGON_n/NFACE_n.
- **Acceptance:** Fluent files load in `fluent3DMeshToFoam` plus `checkMesh` (conformance harness, `backlog.md:35`), and in Fluent itself once a licence exists (checklist `backlog.md:40`). CGNS files read back through `meshio` or the CGNS library.
- **Hash impact:** none (new exporters).
- **Depends:** a Fluent licence for "done". Neither format is on the user's stated path (transport code and ray tracing).

### W5+ items (not scheduled)
- **POLY-RVD (L):** restricted Voronoi of free generators with CVT, as a polyhedron scheme independent of the tet mesh (new `sm-scheme-voro`); compare against VoroCrust.
- **POLY-POWER (M):** weighted (power) dual with HOT weight optimization for non-Delaunay regular meshes.
- **POLY-MIXED (L):** partial dualization of hybrid meshes (needs conformize, `DESIGN.md:302`).
- **POLY-BULK (S, ACR):** bulk builder commit (`backlog.md:10`), because the serial commit dominates large duals (`DESIGN.md:169-177`). Trigger: a 10^6-cell poly requirement.
- Sweep-order or cycle information per direction for Sn. Trigger: the transport code asks for it.

## Priority (for the user's goals)

- **First sub-wave:**
  - MEASURE: cheap, and its numbers decide the rest.
  - VORO: the main design bet; strict, opt-in, no hash risk.
  - VOLDIAG: neutronics correctness, tiny.
  - ACR-POLY2 is optional and can go in sub-wave 0.
- **Later sub-wave:**
  - AGG plus the LevelMap ACR: high value if the transport code uses CMFD-style acceleration (Question 4).
  - SCHEME: PLAN parity, small.
  - SHAPES: only after VORO shows hex-lattice outputs matter.
- **Trigger-based:** VORO-CLIP or CONFORM (MEASURE's eligibility rates), FLUENT and CGNS (licence), POLY-BULK.
- **W5+:** RVD, POWER, MIXED.

For ray tracing the biggest lever is the fast-path share, and in `Planar` mode both concavity and the 64-face cap limit it. So MEASURE comes first, then VORO.

One cross-area lever: if MEASURE shows many cells that are convex but have more than 64 faces, raising `CONVEX_CHECK_MAX` (`core.rs:28`) is a one-line sm-rt change with a benchmark. It belongs to the sm-rt area, not here.

For the transport code the biggest unknown is its discretization (Question 1).

**Critique delta:** SHAPES moved down, SCHEME added, and the cap note added.

## Risks

1. **The Delaunay assumption, at the boundary.** VORO is strict, so on real CAD meshes it may refuse most inputs. MEASURE quantifies this before merge, and VORO-CLIP or CONFORM is the planned answer.
2. **Exact construction.** Welding and gates use exact predicates on input points only, which is cheap. VORO-CLIP needs predicates on constructed points, which is the hard part and is sized as L/XL.
3. **Slivers.** Circumcentric duals of slivers have tiny or huge faces and are often non-Delaunay. The tet scheme's quality floor (`PARITY.md:70`) does not remove all of them. Report a face-area ratio.
   **Critique delta:** the original cited `PARITY.md:68` (the P0 map scheme).
4. **Hash exposure.** Omitting the default new op argument is the safeguard, and it is tested. The list of pins is in Findings and is completed by MEASURE.
5. **Curved boundaries.** Faceting loses material volume (D10). The fix is in `sm-tess` (Question 2).
6. **Fluent unknowns.** Without a licence the writer cannot be certified (`backlog.md:40`).
7. **Benchmark wording.** "Exact analytic faces on polyhedral cells" (`backlog.md:50`) merges two mechanisms (the CSG bridge and planar mesh cells). Split the claim before numbers are published.
8. **Tolerance sharing.** The metric lives in sm-quality and the flag in sm-rt, and neither crate may depend on the other. A cross-check test (MEASURE (b)) guards the shared rule.
9. **LevelMap ACR digest.** A new hashed table in `MeshLayer` must leave existing digests unchanged when it is empty, or it becomes a user decision.

## Questions for the user

1. **What discretization does the transport code use?** The options are FV diffusion, discontinuous FE on polyhedra, characteristics or Monte Carlo with mesh tallies. The answer decides whether convexity, orthogonality and one face per neighbour pair are hard requirements. Recommendation: target planar faces and convex cells, which is what ray tracing wants and what most polyhedral schemes assume, and keep star-shapedness as the fallback the validator already enforces.
2. **Curved material boundaries: faceted with matched volume, or faceted and inscribed?** Recommendation: keep planar facets, add a volume-matched tessellation option in `sm-tess`, and report the deficit with VOLDIAG. Do not project dual points onto the CAD surface, because that breaks planarity. Tracking with exact surfaces already exists through the CSG bridge for models that need it.
3. **May the circumcentric dual become the default of `mesh.to_poly` later?** That would move every existing poly hash. Recommendation: opt-in for all of W4, decided at W4-INT from the MEASURE and VORO numbers. This can wait until INT.
4. **Does the transport code use coarse-mesh acceleration (CMFD or similar) that needs a fine→coarse cell and face map?** If yes, AGG emits `LevelMap`s and needs ACR-POLY-LEVELMAP so that maps live in session state. Recommendation: yes, if CMFD is planned.
   **Critique delta:** the original said "almost free"; it needs a frozen-crate ACR.
5. **Will the transport code read `.smsh` directly** (`ARCHITECTURE.md:196`, open question 1), and should exports always fill the existing geometry cache (`gc_*`, `SMSH_FORMAT.md:259`) for poly meshes? Recommendation: yes for poly meshes, with no format bump.
   **Critique delta:** reworded, because format 1.1 already has the section.
6. **Is a Fluent licence (trial or student) coming?** This is information, not a design decision. Recommendation: defer FLUENT until it exists; OpenFOAM `checkMesh` already judges the poly exports.
7. **May `sm-scheme-tet` gain a "Delaunay output" option** (boundary protection and a final Delaunay restoration, CONFORM) if MEASURE shows its meshes mostly fail the VORO gate? This is a cross-area scope decision.
   **Critique delta:** new.
