# sm-topo-api: cellular non-manifold topology (candidate: radial-edge)

Style: Weiler radial-edge / partial-entity structure. Every level has an explicit *use*; one global
modeling tolerance; exact predicates for sign decisions (`sm-geom-api`).
Tests: `cargo test -p sm-topo-api` (19 tests). Includes the determinism test
(`determinism_same_build_same_ids_and_hash`). The crate has no rayon, so thread count cannot affect
output; the test also rebuilds on a second thread and compares ids, ring order and state hash.

## Model

```
Body > Volume > Shell > FaceUse (x2 per Face, mates) > Face > Loop > Coedge > Edge > Vertex
                          |                                                  ^
                          +--- EdgeUse (coedge x side, or wire) in the radial ring of an edge
                          +--- VertexUse (corner / wire end / lone) listed per vertex
```

- **Face** is geometric (surface handle, `surface_sense`, loops) and has exactly two **face uses**,
  indexed by `FaceSide::{Negative, Positive}`. The normal points away from `Negative` and into
  `Positive` (the IR rule "normal goes from left to right"). Each use is owned by a shell of one
  volume, or by `Exterior`. That is "each face with two sides, side -> volume or exterior".
- **FaceUse sense** (`UseSense::{Aligned, Opposed}`): whether the face normal points out of (Aligned)
  or into (Opposed) the owning volume. After a merge the two volumes sharing a face see it with
  opposite senses. It is stored redundantly with the side so consumers need not know the side
  convention; `validate` checks they agree.
- **Loop / Coedge** are stored once per face in `Negative`-side order. `LoopUse` and `EdgeUse` for the
  `Positive` side are derived (reversed order and direction), so the two sides can never disagree.
  `CoedgeKind::{Edge{edge, sense}, Pole{vertex}}`: pole steps on a sphere or cone apex are degenerate
  coedges, not edges (they do not count in Euler formulas and have no ring). `pcurve` carries the
  parameter-space curve, which is what disambiguates seams.
- **Seam** = one edge used twice by the same face with opposite senses and pcurves on `u = lo` and
  `u = hi`. Its ring has four uses and validates like any other.
- **Radial ring** (`radial_ring(edge)`): all edge uses around an edge, counter-clockwise about the edge
  direction. The two uses of one coedge are adjacent (zero-thickness face); between uses of different
  faces lies one wedge owned by one volume or the exterior. `validate` checks that both neighbours of
  every wedge name the same owner. This is what makes T-junctions of three or more faces (imprint and
  merge) first-class. The ring is canonical: it starts at the smallest `(face, loop, index)` coedge,
  angles use the face's inward direction `n x t`, ties break by coedge ref.
- **Shell** kinds: `Outer`, `Void` (cavity), `Internal` (free-standing sheets/baffles, wires, lone
  vertices inside the volume). A baffle is a face whose two uses are in the same volume
  (`FaceClass::Baffle`); a wire edge is `EdgeUse::Wire` in its volume. Bodies also hold sheets
  (both uses exterior), free wires and points, which covers non-manifold lower-dimensional parts.
- **Imprint/merge status** per coincident face pair: `Coincidence::{Merged, ImprintedNotMerged,
  CoincidentUnimprinted, Overlapping}` in `CoincidenceRecord`. `merge_faces` records `Merged`
  itself; the others are set by the imprint/detection ops via `set_coincidence`. Detected states are
  therefore explicit data, never an accident of geometry.
- **Ids.** Five kinds are stable `EntityId`s (`BodyId, VolumeId, FaceId, EdgeId, VertexId`), dense
  serials in creation order, never reused. Retired entities keep their record with
  `EntityState::{Split, Merged, Deleted}(op)`. Uses and sub-structures are *structural handles*
  `(owner id, index/side)`, so `sm-core::EntityKind` is unchanged. Stable ids come from calling the
  edit API in a canonical, data-defined order (documented contract on `TopologyEdit`); the state hash
  detects violations in tests.
- **Lineage** (`LineageLog`): `Created`, `Split{parent, children}`, `Merged{survivor, absorbed, gap}`,
  `Deleted`, each with `(op, seq)`. Queries: `successors`, `current_of` (what an old id has become),
  `ancestors_of`, `history_of`. No selectors (P2).
- **Shape handles.** `EdgeRec.curve` and `FaceRec.surface` are `EntityId`s resolved by
  `TopologyView::shapes() -> &dyn ShapeStore`. An edge is a parameter interval `[t0, t1]` on a shared
  curve; a closed edge has `start == end` and `t1 - t0 == period`.
- **Virtual topology overlay** (`overlay.rs`): `VirtualOverlay` trait + `SimpleOverlay`. Composite
  (adjacent hosts shown as one) and partition (one host as pieces). Virtual entities live in a
  reserved serial range (`VIRTUAL_SERIAL_BIT`), the overlay takes only `&dyn TopologyView`, so host
  entities and the host state hash cannot change (tested). `visible_faces/edges` give the combined view.
- **Read / edit split.** `TopologyView` (object-safe, `Send + Sync`, borrowed slices on hot paths) is
  all that meshers, coloring, boundary kinds and tracking see. `TopologyEdit: TopologyView` adds
  Euler-style operators: `make_*`, `bind/release_face_use`, `split_edge` (MEV), `split_face` (MEF),
  `merge_vertices/edges/faces`, `kill_*`, `set_coincidence`, bracketed by `begin_op/end_op`.
  Booleans, imprint, merge, webcut, extrude are compositions of these (lives in `sm-model`).

## Tolerance decision: one global tolerance, no per-entity tolerance

`ModelTolerance` (from `sm-geom-api`) is part of the complex and its hash. Identity decisions
(vertices coincide, edge ends match a curve, edges merge) compare against `resolution`.

Why not Parasolid-style tolerant topology:
1. Coincidence under per-entity tolerances is not transitive (A~B, B~C, A!~C), so merge results depend
   on traversal order. That breaks the determinism rule; a global tolerance gives equivalence classes.
2. Per-entity floats leak into every predicate and every hash (and into git diffs of saved state).
3. Persistent naming and lineage want merge to be a pure function of the model.

Consequence we accept: a merge only succeeds when the geometries already agree within `resolution`.
There is no op-scoped "merge tolerance" argument. A user who wants Cubit's loose merge tolerance has two
options: set a larger project resolution at import, or run a *snap* step first (`sm-heal`/`sm-model`
moves the dropped side's geometry onto the survivor), after which the topology merge is exact. The
`gap` actually discarded is recorded in `LineageEvent::Merged` as the audit trail. Sloppy imported
CAD is healed or rejected at import, not carried as tolerant topology.

## Requirement coverage

| Requirement | Where |
|---|---|
| Body > Volume > Shell > FaceUse > Face > Loop > Coedge > Edge > Vertex | `model.rs`, `ids.rs` |
| Two sides per face, side -> volume or exterior | `FaceRec.uses`, `UseOwner`, `Side`, `TopologyView::side_of/face_sides` |
| FaceUse sense after merge, coedge sense | `UseSense`, `FaceSide::sense`, `CoedgeKind::Edge.sense`; test `two_bricks_share_one_face_with_opposite_use_senses` |
| Internal faces, baffles, wires | `ShellKind::Internal`, `FaceClass::Baffle`, `EdgeUse::Wire`; test `baffle_and_wire_inside_a_volume` |
| Imprint/merge status per coincident pair | `Coincidence`, `CoincidenceRecord`, `set_coincidence`; test `imprint_then_merge_progression_and_non_manifold_edge` |
| Stable EntityIds, deterministic | `ids.rs`, dense serials; `state_hash`; determinism test |
| Lineage (split, merged, deleted) | `lineage.rs`; tests `face_split_records_lineage...`, `lineage_queries_on_a_handmade_log` |
| Read-only query trait separate from edit trait | `view.rs` / `edit.rs` |
| Euler-style operators | `TopologyEdit` |
| Topology -> geometry references | `EdgeRec.curve`, `FaceRec.surface`, `ShapeStore` |
| Tolerance model decided and justified | section above |
| Virtual topology overlay hook | `overlay.rs`; test `overlay_composites_without_touching_the_host` |
| Reference implementors | `reference::RefComplex` + `reference::fixtures` |
| Brick | `brick_counts_sides_and_rings` (V8 E12 F6, Euler 2, every ring has 4 uses) |
| Two bricks imprinted + merged, two sides, opposite senses | `two_bricks_merged`, `two_bricks_imprint_merge` (half-size contact, 3-face ring at the cut edge) |
| Face split with lineage | `split_top_face` |
| Cylinder with periodic seam (and sphere with poles) | `cylinder_in_op`, `sphere_in_op` |
| Baffle face inside a volume | `baffle_block_in_op` |
| Same build, same ids/hash | `determinism_same_build_same_ids_and_hash` |
| Validator independent of implementation | `validate()` checks derived indices against primary records; `validator_detects_corrupt_rings_and_wedges` |

## Tradeoffs

- **Radial-edge is the heaviest of the usual choices.** Explicit uses and radial rings cost memory and
  an `end_op` rebuild, but give O(1) answers to "which volume is across this face", "what surrounds this
  edge" and "what are the wedges". Those drive sweep source/target, sideset-with-respect-to-volume,
  mesh-walking tracking, and imprint/merge correctness. A half-edge layout answers the face and loop
  questions as fast but needs bolt-ons for 3+ faces on an edge.
- **Structural handles instead of allocated ids for uses.** Zero allocator state, impossible to
  dangle, but coedge indices shift when a loop's edge is split (documented; handles are valid between
  ops that do not edit that loop).
- **Coedges are stored once per face**, with the second side derived. Fewer invariants to break, but
  per-side data (Cubit's vertex type per surface, firmness) is keyed by `(face, vertex)` or `CoedgeRef`
  by the scheme layer, not stored here.
- **The reference store rebuilds all derived indices in `end_op`.** Obviously correct and independent
  of edit order; O(model) per op. A production store updates incrementally under the same observable
  contract (the validator and `state_hash` are the oracle).
- **Ring ordering is first-order** (tangent plane at the edge midpoint). Faces tangent along an edge
  (cusps) tie-break by id; curvature-aware ordering is future work.
- **Polyline pcurves are dropped on edge split**; segments are subdivided.
- **No op-scoped tolerance** (see above): simpler and deterministic, but loose merges need a snap step.

## Open issues

- `EntityKind` has no variants for shell, loop, coedge or uses, so they cannot appear in lineage or
  coloring. If persistent naming (P2) needs them, `sm-core` needs new kinds.
- Inner-loop assignment in `split_face` is caller-provided; `split_face` rejects cuts that touch an
  inner loop. Webcut across holes needs a richer op.
- Partition overlays record identity only; geometry and boundary of virtual pieces is the job of
  `sm-virtual-topo`.
- Body connectivity is maintained by the ops, not recomputed; a body split by a boolean needs
  `make_body` plus a volume move primitive (not in the API yet).
- Persistent on-disk form is not defined here (the types derive serde behind the feature); canonical
  file order follows addendum §7.2 (kind, id).
