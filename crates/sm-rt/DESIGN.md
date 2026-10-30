# sm-rt: design

## Positioning

`sm-rt` is similar to DAGMC with MOAB and double-down (a ray/`point_in_volume`/`closest_to_location`
service over a model) and to the cell-to-cell walkers used in unstructured-mesh Monte Carlo
(Plücker-style exact crossing tests inside each cell), and it implements the frozen `sm-rt-api`
contract (`Tracker`, `TrackerBuilder`, `.smrt` cache). It differs in three ways. First, the
polyhedral transport mesh *is* the tracking structure: a face belongs to exactly two cells by
construction, so crossing a face is a table lookup and a chord is exact on the planar fan
geometry; a BVH exists only for locating a point without a hint, for the complement outside the
mesh, and for facet worlds. Second, there is no repair step and no random history: watertightness
is a located, grouped diagnostic (`check_facets`) and a point on a face is resolved by the
direction, then by the contract's hashed direction, so every answer is a pure function of the
inputs. Third, a facet shell set is converted to the *same* arrays (one polyhedral cell per
region), so the mesh walker, the cache and the determinism rules are one code path. The result is
a tracker whose lost-particle audit, thread-count independence and cache round trip are tested
bit for bit; no speed claim against DAGMC is made.

## Data layout (`core.rs`)

One flat structure serves both inputs; nothing is allocated per element in a query.

| column | meaning |
|---|---|
| `pos` | vertex coordinates |
| `fv_off`, `fv` | CSR face loops (mesh face loops, or the triangle corners of a facet) |
| `f_left`, `f_right` | cells of a face (`NONE` = outside the model); the loop normal points left to right |
| `f_plane` | unit normal and offset `n.x = d`; `f_apex` the vertex mean; `f_flags` planar bit |
| `f_surface`, `f_action` | model surface of a region interface / boundary face; index into `actions` |
| `c_off`, `c_face` | CSR faces of a cell (the cell's sign for a face is `+` when it is the left cell) |
| `c_region`, `c_flags`, `c_bbox`, `c_volume` | region, convex/BVH flags, box, volume |
| `face_bvh` | BVH over *surface* faces (interfaces and boundary): complement, facet cells, `closest` |
| `cell_bvh` | BVH over cell boxes (`locate` without a hint) |

Face geometry is the `FAN_VERTEX_MEAN_V1` fan about the vertex-mean apex (3-vertex faces are one
triangle), so chords agree with `sm_mesh_api::cell_volume`; the mesh path takes `cell_volume`
from `sm_mesh_api` itself. Faces that are not planar within `BuildOptions::planarity_tol`
(relative to their diameter) are *not* an error: they are tracked exactly through their fan
triangles, listed as non-fatal `mesh.nonplanar_face` issues in the `BuildReport` and counted in
`TrackerInfo::faceted_faces` (0 for a planar mesh; all faces for a facet input). The BVH is a
deterministic median split on the longest axis of the box centroids, ties by id, leaves sorted,
so it is a pure function of the boxes (build is rayon-parallel only in the per-face/per-cell
derivation, collected in index order).

Memory is 142 bytes per face for a hex mesh and 154 for a facet shell (`TrackerInfo::memory_bytes`
counts lengths, not capacities, so a loaded cache reports the same number).

## Cell walking (`walk.rs`)

* **Convex cells** (all faces planar, every cell vertex on the inner side of every face plane
  within `planarity_tol`, at most 64 faces): Cyrus-Beck. The exit is the minimum parameter over the
  outward-moving planes; the entry face is excluded by identity only when the ray grazes it
  (`|n.d| <= 1e-9`); a face the ray runs along while on it is never an exit; the lowest `FaceId`
  among exits tied within `64 eps (1 + t + |o|)` wins (edge and vertex exits). A position more
  than `tol` outside any face plane is `Outside` (a wrong cell is detected, not walked).
* **Everything else** (non-convex, non-planar faces, facet cells): the line `(o, o + d)` crosses a
  fan triangle iff the three edge functions `orient3d(o, o+d, a, b)` have equal sign. Each edge's
  endpoints are ordered by vertex index (apex edges by `n_vertices + face`), so neighbouring
  triangles of a cell evaluate each shared edge identically, and an exact zero is resolved by
  `sm_math::sos::orient3d_sos` with ids = vertex indices (line points get the two largest ids). An
  exactly degenerate edge therefore allocates inside `sm-math`; generic positions never do. This is a
  known gap that cannot be fixed locally (`orient3d_sos` is owned by `sm-math`; a fix needs an
  allocation-free SoS entry point there). The
  parameter `t` comes from the face plane (the triangle plane for a non-planar face). The exit is
  the first outward crossing at `t >= -tol`; entering crossings at `t <= 4 tol` are the entry
  itself and ignored; a first crossing that is entering beyond that is retried against exiting
  crossings within `4 tol`, else the step reports `Lost(NoExitFace)` (the frozen `LostCause` has no
  sense-mismatch member). Cells with more than 24 faces whose faces are all surface faces (facet
  shells) walk through `face_bvh` with ordered ray traversal and the same exact test at the leaves.
* **Events.** `CellFace` (same region), `RegionBoundary` (other region), `Boundary` (model
  boundary; the contract has one boundary event). `Reflect` (mirror deck) is applied inside `step`
  as `d - 2 (d.n) n` with no renormalisation, which is bit-exact for axis-aligned faces; `Wrap`
  applies the deck transform to position and direction and continues in the mate's cell with
  `entered_via = mate`. `Respond` leaves the state on the face for `resolve_boundary`; the tracker
  draws no random number.
* **Recovery.** `RecoveryPolicy::Nudge{tries}`: attempt `k` re-locates `tol * 2^k` ahead of the
  original position with the direction and the failed cell as hint, retries the step and adds the
  distance to the step; each success increments `MeshTracker::nudge_count()` (the frozen
  `LostInfo` has no `retries` field). `Fail` returns the located `Lost` immediately. A lost step
  leaves the state unchanged and carries cause, position, direction, last cell, last face, the
  nearest surface and the region's entity.

## Tolerance policy

Three constants interact; all are multiples of `BuildOptions::tol.linear`:

| constant | value | where | meaning |
|---|---|---|---|
| on-face band | `tol` | `locate`, `exit_convex` | a point within `tol` of a face plane is on it; beyond `tol` outside a convex cell it is `Outside` |
| locate probe | `1e-3 tol` and `2 tol` | `generic_probe`, `locate_facets` | the direction "enters" a non-convex cell / facet region if the point just ahead or 2 tol ahead is inside |
| entry window | `4 tol` | `decide_exit` | entering crossings this close to the start are the entry, not a sense error |

So a start up to `4 tol` outside a non-convex cell is accepted and the chord begins at the given
position (it may be up to `4 tol` early); convex cells refuse a start more than `tol` outside and
the nudge re-locates. A point on a boundary face whose direction leaves the model is
`CellId::NONE` with `on_face` set, so `init_state` on an outward-facing hull point is `LeftModel`.

## Locate

`locate(p, dir, hint, scratch)`: points outside the (tolerance-padded) box are `NONE`. Mesh: the
hint cell and up to 8 convex neighbours by the most violated plane (visibility walk), then cell-BVH
candidates in ascending cell id. A strictly inside cell wins; otherwise among the cells the point is
on the boundary of, the lowest id whose direction enters (rank 0), then a tangent one (rank 1); if
the direction only leaves, the point is outside and `on_face` is the lowest face. With `dir = None`
the contract's `hashed_direction(p)` is used (lazily, only on a face). Non-convex cells: first
crossing orientation along the hashed direction (majority of three directions when the first
crossing is within `1e-6` of grazing). Facets: a face within `tol` makes it an on-face query,
otherwise the first facet hit along the hashed direction names the region (front if the line runs
along the normal, back otherwise), with the same majority rule.

`closest(p, region)` is exact to the fan triangles: branch-and-bound over `face_bvh`, ties by face id.

## Complement

`CellId::COMPLEMENT` is the space inside the box of the vertices that no cell covers. It exists
for facet input (region `complement`) and for `ComplementPolicy::Region(name)`. A point there
locates to `COMPLEMENT`; `step` there finds the first entering exterior-facing boundary face through
`face_bvh` and ends with `RegionBoundary{to_cell}`; if there is none the ray leaves the box and the
step ends with `RegionBoundary{surface: NONE, face: NONE, to_region: NONE, to_cell: NONE}` after the
distance to the box (the state then has `cell == NONE`, which `trace` reports as `LeftModel`).
A one-cell (exterior) face that no `BoundarySpec` rule claims, when a complement exists, is crossed
into it: `step` ends with `RegionBoundary{to_region: complement, to_cell: COMPLEMENT}` (no
`Boundary` event; the face has no action in the table). Disjoint bodies and a non-convex hull are
therefore connected through the complement. Explicit rules (Reflect, Wrap, Respond) still claim
their faces and give `Boundary`. The complement ends at the box of the vertices. Ruling (DAGMC graveyard convention): when a
complement exists, `BoundarySpec.default` applies at the complement's outer box exit, not on the
unclaimed faces that lead into it. A vacuum default (or none) keeps the exit
`RegionBoundary{.. to_cell: NONE}` (`LeftModel`); any other default (`Reflect`, `Respond` with
white, albedo or reflective) ends the step with `Boundary{surface: NONE, face, action}` in the
complement, where `face` is a virtual id `n_faces + k` for the box side `k` (-x, +x, -y, +y, -z, +z;
`face_normal` returns its outward axis normal, so `resolve_boundary` works). `Reflect` is applied
inside `step` (exact axis flip). A `Wrap` default cannot apply at a box and falls back to vacuum.
The action is the `exit_action` of the core, stored in the `Boundaries` section (builder version
2). The build emits a non-fatal `boundary.default_at_complement_exit` warning in `BuildReport`
whenever a non-vacuum default would have applied to faces that now lead into the complement
(`boundary.default_wrap_at_complement_exit` for the `Wrap` fallback). With `ComplementPolicy::Outside` (no complement) the unclaimed faces keep `spec.default`
and the outside of a mesh is `NONE`.

## Facet input

`build` runs `FacetInput::validate` and `check_facets`, which groups the located findings of
`FacetInput::check_watertight` (one issue per bad edge) into connected components per volume and
kind: one missing triangle is one `facet.open_edge` whose `location` is the mean of the hole's
vertices (on the missing triangle) and `gap` the longest open edge. The other codes are
`facet.nonmanifold_edge`, `facet.inconsistent_orientation` and `deck.mismatch` (a periodic rule whose
mate face is not in the input). `facet.coincident` flags facets of different shells whose snapped corners (grid of 1e-9 of the
model diagonal) coincide: touching volumes that were not imprinted and merged, as DAGMC requires;
the build refuses them. Should such facets reach `locate` anyway, tied first hits (within the ray
tie width) resolve to the volume side, and distinct volumes among the ties go to the 3-direction
majority vote. Nothing is ever repaired; a non-watertight input is
`RtError::Build`. Each region becomes one cell; a triangle with an exterior front is flipped so its
left cell is a volume; one `SurfaceInfo` per `(B-rep face, front, back)`.

## Boundaries and decks

`BoundarySpec` rules are matched by `BoundarySelector::Entity` (mesh faces classified on the model
face; facet faces carrying the entity) or `FaceSet` (mesh only); the first rule wins, then the
default. A `Wrap` rule names a model mate face; the tracker needs a mesh-face mate, so each boundary
face is matched to the face of the mate entity under the transform within `tol.linear` (sorted
search, lowest id on ties); an unmatched face is a `deck.mismatch` build issue. Non-matching
periodic faces are W3b, and so `TrackerCaps::PERIODIC` is left unset although matched `Wrap`s are
applied in `step` (the task fixes the caps to `VOLUME_MESH | EXACT_PLANAR`; the bit can be turned on
once non-matching faces are supported).

## Cache (`image.rs`)

`export_cache` writes the contract sections: `FacePlanes`, `CellInfo` (cell CSR, regions, flags,
boxes, volumes), `Bvh` (surface faces), `CellBvh`, `Tables` (regions, surfaces, mesh cell sets),
`Boundaries`; and `Private(0..3)` (header, vertices, face connectivity, apexes). `load` copies into
`Vec`s, validates every length and index (a hostile image is `CacheCorrupt`, never a panic) and
recomputes the structure hash, which must equal the stored one. The `.smrt` file is the
`CacheImage::encode` bytes verbatim (`write_smrt`, `read_smrt`). The key is the contract's
`CacheKey`: topology = `sm_mesh_api::topology_hash` + `geometry_hash` + region rule + complement
(mesh) or `FacetInput::content_hash`; boundary = `BoundarySpec::content_hash`; options =
`BuildOptions::content_hash`; builder version 1. The mesh's own hashes define identity, so a key is
exactly as canonical as the mesh; the cell index is part of it.

## Plugin

`RtPlugin` (id `rt`) registers the queries `rt.info`, `rt.check` and `rt.locate` over a `Mesh`
entity: the tracker is built from the `meshes` layer entry with the model's boundary tables
(`BoundarySpec::add_model_face`; response records default to vacuum, overridable per record name)
and cached in the session's derived cache keyed by the exact content of `model` and `meshes`.
`rt.fire` is not provided.

## Determinism

Queries are pure functions of the tracker and arguments; ties go to the lowest id; no clock, thread
id or RNG is read. `trace_bundle` is overridden (rayon over fixed `block_size` blocks, blocks
appended in order) and is bit-identical to the default sequential walk (tested). The only mutable
state is `nudge_count()`, which no query reads. Tests: build, cache bytes and bundle hash for
`RAYON_NUM_THREADS=1` against N threads and concurrent runs (`sm_testkit::determinism::check`).

## Throughput (record only)

Release build, `cargo run --release -p sm-rt --example bench_rt 100`, CPU:
`Intel(R) Xeon(R) Processor @ 2.10GHz` (4 vCPU). No gate; no comparison with DAGMC is claimed.

| quantity | 1 thread | 4 threads |
|---|---|---|
| step rate, hex_block(100) (1e6 cells), hex-cell crossings/s | 1.8e6 | 8.0e6 |
| step rate, hex_block(10) (cache resident), crossings/s | 5.4e6 | 1.4e7 |
| `locate` points/s, hex_block(100), no hint | 3.4e5 | 1.4e6 |
| `locate` points/s, hex_block(10), no hint | 2.2e6 | 7.9e6 |
| `trace_bundle` rays/s, icosphere (level 4, 5120 facets) | 2.5e5 | 1.0e6 |
| build hex_block(100), seconds (excludes the 14 s reference-mesh construction) | 4.7 | 2.9 |
| bytes per face, hex_block(100) / icosphere(4) | 142 / 154 | same |

The large-mesh step rate is bound by cache misses (the cells of a ray are not neighbours in
memory); a cell-local copy of the planes made the cache-resident case 25 % faster but cost 45 %
more memory and did not change the large case, so it was dropped.

## Deferred (W3b) and not done

ANALYTIC and HIERARCHY backends, `Brep` and `World` inputs (`Unsupported`), non-matching periodic
faces, `.smsh` chunk embedding, mmap load, SIMD/packet traversal, `Precision::F32Traversal`,
`rt.fire`, exact `closest` for curved faces, overlap detection in `check`.
