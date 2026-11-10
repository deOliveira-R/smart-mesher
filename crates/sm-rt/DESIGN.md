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
  `sm_math::sos::orient3d_sos_noalloc` with ids = vertex indices (line points get the two largest
  ids), the allocation-free entry with the same results as the allocating one, so a degenerate edge
  allocates no more than a generic one (`tests/alloc.rs`, Kuhn-lattice rays). The
  parameter `t` comes from the face plane (the triangle plane for a non-planar face). The exit is
  the first outward crossing at `t >= -tol`; entering crossings at `t <= 4 tol` are the entry
  itself and ignored; a first crossing that is entering beyond that is retried against exiting
  crossings within `4 tol`, else the step reports `Lost(SenseMismatch)`; `LostInfo::retries` counts
  the nudges a failed recovery tried (`RecoveryFailed`). Cells with more than 24 faces whose faces are all surface faces (facet
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
`Boundaries`; and `Private(0..3)` (header, vertices, face connectivity, apexes); `Private(4)`
(region tags) only when some region carries tags, so tag-free images keep their bytes. `load` copies into
`Vec`s, validates every length and index (a hostile image is `CacheCorrupt`, never a panic) and
recomputes the structure hash, which must equal the stored one. The `.smrt` file is the
`CacheImage::encode` bytes verbatim (`write_smrt`, `read_smrt`). The key is the contract's
`CacheKey`: topology = `sm_mesh_api::topology_hash` + `geometry_hash` + region rule + complement
(mesh) or `FacetInput::content_hash`; boundary = `BoundarySpec::content_hash`; options =
`BuildOptions::content_hash`; builder version 1. The mesh's own hashes define identity, so a key is
exactly as canonical as the mesh; the cell index is part of it.

`TrackerBuilder::load_image` (and the free `load_image`) opens an image with no input: the reader
path of the handover file (`session.export` format `smrt`, read by the transport code through the C
ABI). Such a tracker has no mesh (`Tracker::mesh()` is `None`); a build from an `Arc` mesh
(`MeshTrackerBuilder::build_arc`) keeps the handle, so `mesh()` is `Some` for it. Region tags
(`RegionInfo::tags`) are attached by `BuildExtras` (by region name), enter the structure hash and the
cache key, and survive the image. `surface_area` (sum of fan-triangle areas, ascending face order)
and `surface_faces` (CSR, ascending) are derived at assembly and never serialized.

**Region rules (W4-0.3).** `RegionRule::ByTaggedCellSets(Vec<RegionSet>)` builds exactly as
`ByCellSets` (same checks: set exists, is a cell set, no overlap, full cover), but region `i` is
named `RegionSet::name` (the set name is only the lookup key) and carries `RegionSet::tags` in
order, through `RegionInfo::with_tag`. The rule has its own cache-key tag (3). `ByCellSets` is the
same code with `name == set` and no tags, so its trackers, images and hashes are unchanged (pinned
in `tests/w4_0_3.rs`). `TrackerInfo::convex_cells` counts the cells flagged `C_CONVEX` (the convex
fast path) for mesh trackers; analytic and world trackers report 0. `rt.info` prints it.

*Finding on `docs/design/w3/backlog.md` ("`RegionInfo.tags` not in cache images and hashed without
a presence marker").* Stale: images carry tags in an optional section written only when some region
has tags (`image.rs`), and `hash_structure` writes a tag count only for regions that have tags, so
the count is the presence marker and tag-free trackers keep their hashes. No change was needed.

## World (`world.rs`)

`BuildInput::Hier` builds a `WorldTracker` (`MeshTrackerBuilder::build`/`build_world`):
a DAG of components over prototype trackers, one per distinct leaf input: a `MeshTracker` (mesh or
facets) or, since W4-RT-WORLD, an `AnalyticTracker` (a `Csg` leaf; `Body` in `world.rs`, an enum, not
a trait object, so the step stays a direct call). A leaf is built as a prototype with vacuum
boundaries and no complement: its own boundary conditions and complement policy do not apply in a
world.

**CSG prototypes (W4-RT-WORLD).** A CSG leaf under `Union`/`Lattice` is tracked in exact geometry
inside its element and the world resolves its outer faces as for a mesh leaf (relocate from the root;
a neighbour is `SlotChange`/`RegionBoundary`, no neighbour is the world boundary). Differences from a
mesh prototype, all deliberate:
* *Outer face.* A point on a surface that has cells on only one side (`SurfaceInfo` front or back is
  `NONE`) is on an outer face (`Body::strict` is false); a surface with cells on both sides is
  interior. Mesh leaves use `f_right == NONE`.
* *Reflection at the world boundary* uses the surface normal at the exit point (`face_normal`), the
  mesh leaf its face plane.
* *Overlap.* A CSG leaf has no vertex list, so the build-time narrow phase (boundary vertices of one
  placement inside another) skips it; overlaps between placements are found by `probe_partners` at
  step time (chord midpoint inside a partner; first entry into a CSG partner, from
  `AnalyticTracker::enter_from_outside`, strictly inside the chord) and by `rt.check_overlaps`. Two
  cells of one leaf holding a point is `Loc::Overlap`.
* *No complement region, no partition.* The world is the boundary of the cells; a bridged world with
  a complement region is refused by `CsgWorld::world_tracker` (use `analytic_tracker`);
  `partition.enabled` with a CSG leaf is `Unsupported("majorant partition of a CSG leaf")`.
  `safety_distance` of a CSG leaf is `Unsupported`. Hex lattices stay refused (W4-RT-HEX).
* *Caps:* `HIERARCHY | ANALYTIC` (no `EXACT_PLANAR`, which names planar facets).
* *Dispatch:* a world whose root is one bare CSG leaf is still the `AnalyticTracker` (same cache key);
  any other world with a CSG leaf is a `WorldTracker` (kind `World`).
The structure hash of a mesh-only world is unchanged (it hashes each prototype's own hash).
The 2D-lattice z rule is unchanged (ruling OR-a): the element centre z is `origin.z + 0.5 * pitch[2]`.

**Numbering.** Regions, surfaces and faces are numbered per prototype (prefix offsets over
prototypes in ascending leaf-input order): regions are prototypes, so every instance of a leaf
shares its regions, and `surface_area` is the area of one instance. Cells are numbered per instance
by prefix sums in ascending instance order (union placements in order; lattice elements x fastest):
a lattice with one fill component computes `element * cells(fill)` with no table, otherwise one
`u64` prefix per element. The reverse map (`cell_region`, `cell_volume`, the locate hint) is a
binary search per level. The total must stay below `u32::MAX - 1`.

**Levels.** `TrackState::levels` has one entry per union or lattice below the root: a union entry is
`(component, placement index, [0; 3])`, a lattice entry `(component, linear element index, element
index)`. The leaf is the child of the last entry (a bare leaf root has no levels). A step rebuilds
the instance frame from the levels (composed global-to-leaf map, no cache, no allocation up to
`LEVEL_INLINE`). Placements must be rigid (`R^T R = I` within 1e-12, mirrors allowed), else
`world.nonrigid`; lattices are rectangular with `outer: None` (hex lattices, outer fills, per-surface
boundary rules and periodic world boundaries are `Unsupported`; hex is stretch S1, not done).
`InstancePolicy` is ignored (the contract says it never changes results).

**Lattices.** An element's fill lives in the element frame (origin at the element centre, as
`Lattice::element_center`); the fill must fit its element (`world.lattice_fit`, x and y, and z with
more than one layer), so a particle leaving a fill through its outer face has left the element. A
single-layer lattice has no z walls. The element index is O(1) index arithmetic (floor division;
on a wall within `tol` both neighbours are candidates and the leaf's own direction-aware locate
decides, preferring the slab the direction enters). The walls come from the prototype's outer faces,
which coincide with the element walls: there is no separate Amanatides-Woo march, because a fill
that fits and a particle that leaves it need only the neighbour lookup; edge and corner exits fall
out of the candidate product (diagonal neighbour first when the direction enters it).

**Step.** The prototype's own `step` runs in the instance frame (distances are preserved by the
rigid maps; the global position is advanced as `o + d t`, never mapped back). A prototype face
or region event maps its ids to global ones. Leaving through an outer face relocates the exit
point from the root excluding the instance just left (rounding must not hand it back): a neighbour
instance gives `SlotChange` when the region is the same and `RegionBoundary` (surface and face of the
face just left) otherwise; no neighbour means the model boundary, where the world's default
`BoundarySpec` applies (`Reflect` is applied in `step`, a `Respond` is left on the face and resolved
by `resolve_boundary`, which the world overrides to take the normal from the instance frame).
`face_normal(face, p)` locates an instance of the face near `p` and rotates the prototype normal
(an extra search; `resolve_boundary` uses the state's levels instead). `closest` is a branch-and-bound
over components (a lattice scans its elements, O(elements)). Recovery nudges happen inside the
prototype.

**Overlap (Q4).** Union placements must be disjoint. Build: sort-and-sweep over the placed boxes
(tolerance padded); for each overlapping pair, every boundary vertex (vertex of a face with one
cell) of each is located in the other, and one strictly inside a cell beyond `tol` is
`world.overlap` naming both placements and the point (frame of the union). AABB overlap alone is
not an error: a pin placed in the hole of a moderator leaf builds, its chords exact. The narrow phase
counts its work (`overlap_vertex_tests`) and misses overlaps where no vertex of either lies inside
the other (crossed bars), and identical placements (all vertices on faces). Track time: `locate`
of a point strictly inside two placements gives no cell (the `NoCellContainsPoint` path); a step in a
placement that has box-overlap partners tests the chord it is about to take against each partner,
first by the chord midpoint strictly inside the partner, then (leaf partners) by the first entry of
the ray into the partner strictly inside the chord (`enter_from_outside`; a non-leaf partner uses
its box and confirms by locating just inside). A hit is `Lost(Overlap)`, state unchanged and never
nudged. Cost: two partner queries per step in a placement with partners, none otherwise. The
sampling check `rt.check_overlaps` (and the CSG leaf) is RT-ANALYTIC/RT-BRIDGE.

**Cache.** A world has no cache image (`export_cache` is `None`; `load` is `Unsupported`); its
cache key is `World`, topology `HierInput::content_hash`, hierarchy `WorldSpec::content_hash`.

## Analytic leaf (`analytic/`)

`AnalyticTrackerBuilder` builds an `AnalyticTracker` from a `BuildInput::Hier` whose root is one
`Csg` leaf (`MeshTrackerBuilder::build` dispatches a bare CSG leaf to it). Tracking is surface
tracking in exact geometry: no mesh, no facets. **Scope:** a single CSG leaf with its own boundaries.
`Union` and `Lattice` worlds of CSG leaves are the `WorldTracker`'s (W4-RT-WORLD, see "World"); this
builder refuses them with `Unsupported`.

**Cells are disjoint (Q4).** There is no cell order and no first-wins rule (OpenMC and MCNP resolve
overlaps by order; this tracker reports them). `locate` evaluates each cell by its own half-spaces;
a point matched by two cells is `NONE` from `locate` (the same path as no cell) and
`Lost(Overlap)` inside `step`, never retried by nudging. `step` tests only the cells of the
`surface -> cells` CSR across the crossed surface, evaluates **all** of them (no early exit), and
reports two matches as `Lost(Overlap)` with the state unchanged. Completeness is the sampled
`check_overlaps(&HierInput, n_points, n_rays, seed)` (stratified points and points along random
chords, seeded by `seed` and the leaf hash; same findings for any thread count; `csg.overlap`
and `csg.gap` with the sample point). The session op `rt.check_overlaps` belongs to RT-BRIDGE.

**Surface kinds** (`kernel.rs`), classified once from the `SurfaceDef` (not from coefficients):

| kind | `sd` (sign of the half-space function) | ray roots | conditioning |
|---|---|---|---|
| plane | `n . (p - o)` | one | exact |
| sphere | `\|p - c\| - r` | closest approach `tc -+ sqrt((r - m)(r + m))` | perpendicular offset `m` formed directly |
| cylinder (any axis) | `\|perp(p - o)\| - r` | same with the perpendicular parts | same |
| cone (double cone, apex handled) | `rho cos a - \|z\| sin a` | quadratic after shifting to the closest approach to the apex | shifted origin; double roots give no event |
| torus (ring) | `hypot(hypot(x, y) - R, z) - r` | quartic about the closest approach to the centre, fixed-size solver, two Newton steps on compensated (Graillat) Horner residuals | chord of a near-tangent line has relative error `~ eps / sqrt(r^2 - b^2)` |
| general quadric (also the elliptic kinds) | `f / \|grad f\|` | ray polynomial evaluated in double-double | limited by the input coefficients |

`SurfaceDef::Cone` is one nappe parametrically, but its implicit form (the one half-spaces use) is
the double cone; both nappes are surfaces here, and a cell takes one nappe by a plane. Tangent
(double) roots produce **no** event: a zero-length chord changes no cell. No kernel allocates.

**Tolerance and ties.** A point with `|sd| <= tol.linear` is on that surface; its side is the sign
of `grad . dir` (moving into), a tangential direction uses `hashed_direction(p)`, `dir = None`
uses `hashed_direction(p)` for every on-face decision. An edge point uses the rule per surface. The band is the contract `tol.linear` (1e-6 by default), wider than the draft's 1e-12: a point that close to a surface is on it for every decision, which keeps `locate` and `step` consistent. When no neighbour cell across a crossed surface matches, `step` runs the full-leaf scan (counted in `slow_locates`) next to another surface or strictly inside the bounds, so coincident or duplicate surfaces cannot send a particle to vacuum mid-geometry. A wrap lands by the cell search over the mate surface's cells, and an overlap there or in nudge recovery is `Lost(Overlap)`, never retried.
`Located::on_face` is the lowest on-face surface of the matched cell (of the whole leaf for the
complement). After a hit the crossed surface's side is forced to the side the ray moves into and
the other surfaces within tolerance use the direction rule; if the current cell still contains
the point (a union cell crossing an interior surface) there is no event. A root of the entry
surface that repeats the crossing just made (same direction, within `tol / |d.n|`) is skipped; a
root crossing back is an event however close (a grazing chord).

**Concentric fast path.** Nested spheres or coaxial cylinders (a pin cell, TRISO layers) locate by
a binary search over the distinct radii plus the other half-spaces of the one or two candidate
cells per shell; a point within `tol` of a radius takes the generic path. `surface_evals` per
locate on a 12-ring pin cell is 3. The generic path (`locate_generic`) evaluates every cell.

**Counters** (`Scratch::counters`): `surface_evals`, `locate_candidates`, `slow_locates` (the
cold full-leaf scan, also used for an edge crossing next to another surface), `nudges`.

**Boundaries.** `FaceId == SurfaceId` (local ids by first appearance in the cell DNFs). A surface
named by a `BoundarySpec` entity rule is a boundary: `Reflect` and `Wrap` are applied in `step`
(bit-exact for planes), `Respond` is reported and not applied. Leaving the cells otherwise is a
boundary with the spec's default (`Reflect` or `Respond`); a default `Wrap` and face-set selectors
are `Unsupported`. With `ComplementPolicy::Region` the complement cell fills the leaf bounds and
its six box faces follow the surfaces. The cap `PERIODIC` is set when a rule wraps.

**Build checks** (`BuildReport::issues`; located by cell or surface): `csg.unbounded` (a cell is
not provably bounded and the leaf has no bounds; fatal with a complement region), `csg.degenerate_surface`
(fatal), `csg.empty_cell` (a cell whose terms are provably empty by sign conflicts, parallel planes
or sphere and plane tests), `csg.unused_surface` (a store surface no cell names), `csg.unknown_surface`
(fatal), `boundary.unknown_mate` (fatal). The "provably bounded" test understands spheres, tori,
axis-aligned cylinders and axis-aligned planes; anything else needs the leaf `bounds`.

**Structure hash** is by value (surface definitions, cell DNFs in input order, names, volumes,
bounds, complement, resolved boundary actions), so permuting surface handles does not move it.
No cache image.

**Record-only numbers** (acceptance 16; release, 1 thread, Apple M1 Max, no gate, harness
included; `cargo test --release -p sm-rt --test analytic_bench -- --ignored --nocapture`):
12-ring pin cell in a box, 3.8e6 segments/s, 23 surface evals per segment, `slow_locates` 12 per 1e6
segments (edge crossings); `locate` 1.2e7 points/s with the concentric fast path and 2.9e6 without;
torus 1.3e5 lines/s.

**Cell index (`analytic/cellindex.rs`, W4-RT-CELLINDEX).** A disjoint kd tiling over the cell boxes,
built when the leaf has at least 64 cells (`AnalyticTracker::with_index_threshold` overrides it:
`0` forces it, `usize::MAX` removes it and gives the all-cells, all-surfaces paths, which stay as
the oracle). The boxes are the `term_box` necessary conditions (infinite sides kept; a provably
empty or non-boxable cell is a candidate everywhere) padded by `4 tol`, so a point inside the
on-face band of a surface still reaches every cell that could claim it. Nodes split at the median
of the box centres on the axis with the smallest larger child (ties: total, then axis), tiles
hold at most 8 cells or stop at depth 40, and a tile lists its cells and their surfaces
ascending. The tree is a pure function of the boxes (sequential build): `index_digest()` is
bit-identical across runs and thread counts. It is derived data: not hashed, not in
`memory_bytes`, no cache image.
Users: `locate_cold` (after the concentric fast path), the `across` full-leaf fallback
(`locate_scan`, same answer as `locate_all`), and the complement step, which walks the tiles
along the ray front to back, tests only their surfaces and takes the first valid event at or
before the tile exit (`sc.counters.surface_evals` counts per tile). A point outside the root box
falls back to the full scan. Not indexed: `on_face_any` (a complement point reports the lowest
surface of the whole leaf within the band, including the plane of a distant cell), `enter_from_outside`
and the `Wrap` neighbour search (CSR). Known limit: two surfaces with identical geometry that
only one cell names can pick a different surface id for the same event than the oracle (the
distance and the next cell are equal);. P2 (flattened,
3 x 3 x 9 pins, 324 surfaces): 22 surface evaluations per complement step against 326.

**Differences from OpenMC and MCNP.** Disjoint cells instead of first-wins order; no per-surface
sense array (senses are evaluated from `sd`); exact `closest` for plane, sphere, cylinder, cone and
torus (`Unsupported` for a general quadric). No speedup over either is claimed.
## Majorant partition and delta flights (`delta/`, W3b-RT-DELTA)

Switched on by `BuildOptions::partition.enabled` (default off: nothing below exists and the
structure hash, image bytes and every W3a output are unchanged). Mesh input only; a facet input is
`Unsupported`. `region_jump` needs `enabled`. The tracker then has `caps |= MAJORANT`
(`| REGION_JUMP` with the jump index) and `majorant_view()`.

**Partition.** A kd tree over the cell boxes padded by 4 `tol`; leaves tile it. A cell is listed in
every leaf it may meet (its box padded by 2 `tol` meets the leaf, and for convex cells no face
plane has all eight corners beyond it by more than 2 `tol`), so the listing is a superset of what
the cell BVH would return. The split rule never sees a cross section: split while the candidates
hold more than one region and more than `leaf_cells` cells (depth at most `max_depth`); 8 equally
spaced planes per axis, cost = sum over the children of volume fraction x (candidates + region
spread), ties to the lowest axis then position, a plane that separates nothing stops the split. A
node whose box lies inside one convex cell with a 2 `tol` margin is *preassigned* (one candidate,
locating inside it tests nothing). The node that stops splitting keeps a **location-only
subtree** (kd nodes without a `NodeId`) that splits by cell count alone, so locating in a
one-region leaf with thousands of cells still tests few cells; majorant nodes and their material
sets are untouched by it. `NodeId`s are the nodes in preorder; the set of a leaf is the regions of
its candidates (plus the complement region unless preassigned), of an internal node the union of its
children; sets are `MaterialSets` (ids by first appearance in node order, then one singleton per
region). Build recurses with `rayon::join` and collects in order, so the bytes do not depend on
the thread count (`partition_hash`).

*Size.* The split rule is the contract's and has no node budget, and a plane that cuts cells puts
them in both children, so the tree can be much larger than the cell count when no plane lines up with
the cells. Recorded (`tests/delta.rs`, `checker_block(12)`: 1728 cells, 4 regions by `(i + j + k) mod 4`,
candidate planes at ninths of the box): `leaf_cells = 8` gives 7877 nodes (1.9 MB tracker memory),
`leaf_cells = 1` gives 993771 nodes (99 MB), because a region can only be isolated by splitting down
to the `16 tol` width limit or `max_depth`. A cap would change the split rule (a node budget cannot be
applied inside the parallel build without changing which subtrees stop, and an error after the fact
would turn a legal build into a failure), so none is applied here: `max_depth` (default 24, at most
48) and `leaf_cells` are the controls, and `rt.partition` reports `nodes`, `leaves`, `depth` and `memory_bytes` before a code commits to a partition.

**Segments.** `advance_segment` descends the kd tree from the root with the point and the
direction (a point on a split goes to the child the ray enters; O(depth), counted in
`node_visits`), reads exactly one majorant (`FlightInput::majorant`, before the state moves) and
picks the mode with `choose_mode`. *Surface*: one `step` (`SegmentScale::Cell`; a segment is the
`trace` segment bit for bit) or one `fire` (`Region`, which jumps when it can). *Delta*: the
segment ends at the wall of the node box (`SlotChange` into the same or the next cell), at the
limit, or at a model event. A node none of whose candidate cells has a face with no cell behind it
(`boundary` flag) goes straight to the wall with no cell walking; a node that has one walks its
cells with `step` up to the wall, so deck events, `Respond` faces and the complement are exactly
`step`'s. At the end of a delta segment the cell is located from the new node's candidate list; the
coordinate that reached the wall is snapped to it, so the next descent enters the next node.
Complement and unlocated states are surface segments (the mode is reported as `Surface`).
`locate_in_node` descends from the node (from the kd root if the point is outside the node, which a
region-run segment may be) and runs the same candidate loop as the BVH path of `locate` (cells
whose box padded by `tol` contains the point, ascending ids, the same probes and tie ranks), so
the answer equals `locate(p, dir, None)` with no BVH descent (`bvh_descents == 0`).
`safety_distance` is the exact distance to the nearest face plane of a convex cell, 0 otherwise;
the cursor caches it. The contract gives `locate_in_node` no previous cell, so the "previous cell
if the cached safety is positive" shortcut is left to the caller (the safety test shows it).

**Region jump.** Per region a BVH over the faces that separate it from another region or from
nothing. `fire` from a convex or general cell of region `R` finds the first crossing that leaves `R`
(ties within the ray tie width go to the lowest face id), puts the state on that face in the cell on
`R`'s side and lets one `step` apply the event, so the end kind, surface, `to_region`, `to_cell` and
the deck action are `step`'s; the distance is the walk's up to the tolerance of ruling R1 (below).
Anything unusual (entering
crossing before the exit, no exit, a lost step) falls back to the walk. A `Limit` ends with one
locate.

*Tolerance (ruling R1).* A flat `1e-12 (1 + d)` between jump and walk is ill-posed for grazing exits:
the walk's error grows like `1 / |d.n|` with the exit face normal `n`. A jump must (1) return the same
end kind, surface, region and `to_region` as the walk; (2) match the exact exit distance, computed from
the exit face, within `1e-12 (1 + d)`; (3) agree with the walk within
`1e-12 (1 + d) / max(|d.n|, 1e-6)`. On an exact edge or vertex tie the face and `to_cell` may differ
(the exact predicates decide which face of the edge the jump leaves through); kind, surface,
`to_region` and distance still agree, and the jump's face must contain the exit point (closed
polygon) or share a vertex with the walk's face. `tests/delta_jump.rs` checks all of it: on rays
through cell centres along axes and diagonals, on a non-convex region, and on a complement region
(`ComplementPolicy::Region` with a reflecting default, rays ending on a virtual box side `n_faces + k`
and rays starting in the complement are counted and compared against the exact box-plane distance).
`step` and `trace` never jump. Recorded (`examples/bench_delta.rs`, counters only): `hex_block(40)`,
5000 fires, 163 face tests (27 steps) per fire by the walk, 11 by the jump.

**Cache.** Sections `Private(5)` (options, root box), `Private(6)` (kd nodes), `Private(7)`
(candidates, node sets and cells, sets). (`Private(4)` is the region-tag table, so the plan's
`4..8` became `5..8`.) Derived data (node boxes, parents, boundary flags, cell-to-node, the jump
index) is rebuilt on load; the decoder checks every count against the bytes left and the tree
structure (children after parents, each node referenced once), and `Partition::from_raw` checks the
meaning that does not need the mesh geometry: a leaf node's set contains the region of every
candidate cell of every kd leaf below it, a node that is itself a kd leaf has exactly its candidate
count of cells (one when preassigned), a node below a leaf node counts at least its distinct cells, and
a parent's set covers each child's; a mismatch is `CacheCorrupt`. It does not re-derive that a
candidate list is geometrically complete (a cell dropped from a leaf it meets is not detected), so a
hostile image can make `locate` answer wrongly but never read out of range. `structure_hash` does not include
the partition (the options are in the cache key, so another option is `CacheStale`); the partition
has its own `partition_hash`. A world's prototypes carry the partition when
`PartitionOptions::enabled` (see "Flights across worlds"); a world has no cache image.

## Flights across worlds (`world.rs`, W3b-RT-DELTA-WORLD)

With `PartitionOptions::enabled` every prototype leaf of a `BuildInput::Hier` gets its kd partition
(`leaf_cells` and `max_depth` apply to each) and the `WorldTracker` has the majorant view
(`caps |= MAJORANT`). Without it nothing changes: no node tables, the same `locate` path, the same
hashes. `region_jump` on a world is `Unsupported` (`fire` walks), as is a facet leaf (no partition) and
a CSG leaf (`AnalyticTrackerBuilder` refuses a partition): an option is honoured or refused, never
dropped.

**Nodes are the prototype tree.** `NodeId`s are first one node per reachable union or lattice
component (reverse post-order, so a parent precedes its children), then the partition nodes of each
prototype in leaf-input order (`proto_off`). Instances have no nodes: every element filled with one
universe shares its prototype's nodes, so `node_count` is independent of the element count (a 3x1 and a
30x3 lattice of one pin give the same slice length; tested) and per-instance majorants need distinct
prototypes, as distinct materials do (regions are prototypes). The parent of a prototype's root node
is the first component node that holds it in that order. A component node reports `NodeKind::Lattice`
for unions and lattices alike: the kind whose children are prototypes in their own frames, which
`validate_partition` does not tile against the node's box. Material sets are interned world-wide
(prototype sets mapped by the region offset, one singleton per region, then the component sets as the
union of their children's), so a node's set covers every region below it and `validate_partition`
passes. Node boxes are in the node's own frame (prototype frame for partition nodes).

**Segments.** `advance_segment` resolves the instance from `TrackState::levels` (index arithmetic,
the composed global-to-leaf map), descends the prototype's kd tree from the root with the local point
and direction, and reads the one majorant of that partition node: lattice and union nodes are internal
and never read. A *surface* segment is one `step` (or one `fire`); a *delta* segment is the chord in
the prototype node's box, as for a mesh tracker: straight to the wall when no outer face of the
prototype is among the node's candidate cells, otherwise walked with the world's `step` up to the
wall, so neighbour instances and the world boundary are `step`'s events. The segment ends there with
the `step` event, or at the wall as `SlotChange` into the same or the next cell of the *same*
instance (`levels` do not change inside a segment; a `step` that crosses into another instance ends
the segment). A lattice wall is therefore crossed inside the segment stream with no per-element
state: the next call starts in the neighbour element and finds its node by the kd descent of the
shared prototype. Rounding: the wall coordinate is snapped in the local frame and the global
position is mapped back; if rounding leaves its local image on the wrong side of the wall, the
global coordinate that moves that local axis the most is bumped by a few ulps (never along the ray,
which would not move the coordinate for a ray nearly parallel to the wall) until the image is on
the entering side, so the next descent never returns the node just left (`past_wall`). A global
position carries an absolute error of about an ulp, so the exit of a line that grazes a wall is off
by that over `|d.n|` (the argument of ruling R1): segment sums equal the chord to 1e-12 for ordinary
lines and to that scaled by `0.05 / min|d_k|` for grazing ones (`tests/delta_world.rs`).

**Locating.** `locate_in_node` takes the prototype node and the segment's `levels`: the element is
resolved from the levels by index arithmetic (counted in `lattice_index_locates`, one per probe, equal
to `tentative` in a delta flight through a lattice), the point is mapped into the prototype frame,
the kd descent starts at the node, and the leaf's candidate list is searched with the same probes
as `locate` (no BVH). With a partition the world's own `locate` and step relocation also search
prototype leaves through the kd candidate lists instead of the leaf BVH, which gives the same cells
(a partition does not change `trace_bundle`; tested), so `bvh_descents == 0` in flights, including
across element walls. `safety_distance` maps the state into the prototype frame and asks the
prototype.

`WorkCounters::bvh_descents` counts the BVH point-location descents of the mesh tracker (it was
never incremented before, so `bvh_descents == 0` asserted nothing); a flight start with no hint is
located through the BVH and is counted, the flight is not.

**Recorded** (`tests/delta_world.rs`, debug, counters only): 600 histories on a 17x17 lattice of
two-region pins (pure absorber, 0.8 / 0.2 per cm): surface 3370 segments; delta 2797 segments and 580
tentative collisions, `bvh_descents == 0` in both; segment sums equal the box chord to about 2e-13 on
a two-level lattice and a rotated placement. A world has one default boundary rule, so the slab of the
single-mesh tests (mirror sides, vacuum ends) is not available: the isotropic-source test compares
the lattice with the same geometry as one mesh, and the analytic `E3` leakage stays on the single mesh
(`delta_stats.rs`).

## Plugin

`RtPlugin` (id `rt`) registers the queries `rt.info`, `rt.check`, `rt.locate` and `rt.partition` over a `Mesh`
entity: the tracker is built from the `meshes` layer entry with the model's boundary tables
(`BoundarySpec::add_model_face`; response records default to vacuum, overridable per record name)
and cached in the session's derived cache keyed by the exact content of `model` and `meshes`.
`rt.partition {mesh, leaf_cells?, max_depth?}` (defaults 8 and 24; `leaf_cells >= 1`, `max_depth <= 48`)
builds the mesh tracker with the majorant partition and reports its structure only: `nodes`,
`leaves`, `depth`, `preassigned_leaves`, `sets`, `cells_per_leaf {min, max, mean}`, `memory_bytes`,
`structure_hash` (the tracker's) and `partition_hash` (arrays, sets and options). Omitted parameters
equal their defaults in the cache key. `rt.fire` is not provided.

`rt.check_overlaps {bodies?, samples?, rays?, seed?: u64 = 0}` (read-only query, ruling O6) runs the
sampled overlap and gap check over the *model* seen as CSG cells (next section), not over a mesh. Defaults
are 10000 points and 500 chords; the cache key is the canonical input with every default filled in, so a call
without `seed` is a call with `seed: 0` (same cache entry, same bytes), and the output echoes the effective
seed. Findings carry the sample point and the cells (volume id, color name, placed copy). `ok` means no
overlap (gaps are reported, they do not fail the check; pins in vacuum are all gap). It runs on the
flattened world when that has at most `MAX_FLAT_CELLS` cells (so two placed copies that overlap are found)
and per prototype leaf otherwise (`scope: "prototypes"`).

## Model to CSG bridge (`bridge.rs`, W3b-RT-BRIDGE)

`csg_world(model, topo, opts) -> CsgWorld` turns a model into CSG leaves and a `WorldSpec`.
* A volume is one cell named by its color (id when uncolored); its region is the intersection of one
  half-space per face use: the face's surface on the side the volume lies. `Side::Left` means the face
  normal points out of the volume, the carrier sense says whether the face normal is the surface normal,
  and `Sign::Negative` is `f <= 0` (every kind's `f` grows outward), so the volume is on the negative side
  iff (Left, Forward) or (Right, Reversed). A `Carrier::Csg` volume is used as given.
* Planes, spheres, cylinders, tori, implicit quadrics and elliptic cylinders are taken as they are. A cone
  face brings the plane through its apex (the kernel's cone is a double cone; the face is one nappe).
* A prototype is a leaf; clusters and instances are `Union`s; a Cartesian lattice is a rectangular
  `Lattice` (slot (0,0,0) at the lattice origin, so the world origin is `-pitch/2`; a 2D lattice gets
  z pitch 1e9, the model has none). Volumes in no prototype are cells of a root leaf.
* **Unsupported**, typed with the entity: NURBS faces (names the face), a volume outside a cone nappe,
  elliptic cones, baffles and faces without a surface, a volume that uses one surface on both sides,
  hex lattices, recolored or clipped slots/members/instances, path overrides. A model with nothing to track
  is `Invalid`.
* **Assumption, not checked here:** a volume equals the intersection of its face half-spaces and its faces
  cover their surfaces (convex pieces, annuli of coaxial quadrics). Anything else shows as overlaps or gaps in
  `rt.check_overlaps`.
* **Hosting:** `CsgWorld::world_tracker` builds the hierarchy as it is (one analytic prototype per leaf,
  a lattice index, no flattening; P2 `structure_bytes` grows only by the 4-byte fill entry per element).
  The analytic tracker takes one bare CSG leaf; `CsgWorld::flatten` expands the
  world into one leaf, one cell per placed volume copy, surfaces moved by `SurfaceDef::transformed` (the
  returned `Sense` flips the half-space under improper placements), identical surfaces shared;
  `CsgWorld::analytic_tracker` does that when needed and is the oracle of the hierarchical build
  (`tests/csg_world.rs`: cell, region and chord equal on seeded rays). Tracking results do not depend
  on flattening.
  The complement region option makes the space between cells a region; stepping through it scans all
  surfaces (no cell-box BVH yet), so use it for small worlds or short paths.

## Determinism

Queries are pure functions of the tracker and arguments; ties go to the lowest id; no clock, thread
id or RNG is read. `trace_bundle` is overridden (rayon over fixed `block_size` blocks, blocks
appended in order) and is bit-identical to the default sequential walk (tested). The only mutable
state is `nudge_count()`, which no query reads.
*Profile independence.* Release builds fuse `x.sin()` and `x.cos()` of one argument into a single
`sincos` call that can differ by an ulp from the two libm calls a debug build makes, so a ray
generator written `[s * phi.cos(), s * phi.sin(), z]` gave other directions, hence another bundle hash,
in release (the `hex_block6` golden in `tests/flat_golden.rs` differed between profiles for this
reason; no hash moved, the tests' generators were wrong). `vec3::sin_cos` keeps the calls separate
(`black_box` on one argument) in the library (the white-boundary direction, the overlap sampler); the
test helpers do the same. Tests: build, cache bytes and bundle hash for
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

hex lattices (W4-RT-HEX),
`closest` lower bound for general quadrics,
`Brep` and `World` inputs (`Unsupported`), region jump and facet or CSG leaves in flown worlds, a world cache image,
non-matching periodic faces, `.smsh` chunk embedding, mmap load, SIMD/packet traversal, `Precision::F32Traversal`,
`rt.fire`, exact `closest` for curved faces, overlap detection in `check` (the world has its own).
