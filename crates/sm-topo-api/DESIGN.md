# sm-topo-api: design (candidate "cellcomplex")

The cellular non-manifold complex as **cells per dimension plus one oriented incidence relation**, with geometry attached as carriers through a GMI-like store, and everything else (loops, coedges, shells, face uses, face sides) *derived*, not stored.

## The one idea

A complex is a set of cells of rank 0..3 (+ a grouping rank for bodies) and **one** incidence relation, stored as CSR boundary rows. Every entry carries the same tag, `Side { Left, Right, Both }`:

| Relation (entry in the parent's row) | `Left` | `Right` | `Both` |
|---|---|---|---|
| edge -> vertex | edge starts here | edge ends here | closed edge: one vertex at both ends (single entry) |
| face -> edge (coedge, in a cyclic loop) | loop runs along the edge, face on its left (`Forward`) | against it (`Reversed`) | embedded edge only |
| volume -> face (face use) | volume on the face's left, normal points **out** of the volume (`Forward`) | normal points **in** (`Reversed`) | **baffle**: volume on both sides |

The face normal points from its left side to its right side (docs/design/ir.md §6). `Exterior` is the absence of a volume on a side, so "each face has two sides, each side -> volume or exterior" is a derived query, `face_sides(face) -> {left: Region, right: Region}`, `Region = Volume(id) | Exterior`.

Because the tag is the same everywhere, one mechanism covers the cases that radial-edge B-reps special-case: coedge direction, FaceUse sense after merge, periodic seams (an edge listed twice, `Left` and `Right`, in one loop), closed edges, baffles.

## Stored vs derived

Stored (`CellComplex`, `complex.rs`): per rank a table indexed by serial (`alive`, `born: OpId`, `carrier`, `flags`, boundary row); sparse tolerance overrides (`BTreeMap`); face-pair coincidence records; an append-only lineage log; the current op.

Derived (lazily built into a `OnceLock`, dropped by every edit): the coboundary rows (transpose, ascending by parent id) and the live-cell lists. Views computed on demand from rows (`derived.rs`, trait `TopologyViewExt`, blanket-implemented for every `TopologyView` including `dyn`):

| View | Computed from |
|---|---|
| `Loop` / `Coedge` | a ring of a face row; one entry |
| `Shell` / `FaceUse` | a ring of a volume row; one entry (a `Both` entry yields two uses with opposite senses) |
| `FaceSides`, `is_baffle`, `is_boundary_face`, `is_interface_face`, `is_free_face` | the volume entries of a face's coboundary |
| radial star of an edge (`radial_uses`), `seam_edges`, `wire_edges`, `closure`, `descendants`, `lineage_of` | coboundary / rows / lineage |

Why not stored entities: `sm-core::EntityKind` (frozen) has no Shell/FaceUse/Loop/Coedge kinds, so they could not carry stable ids anyway. Cells that need ids (vertex, edge, face, volume, body) have them.

### Rows are ordered, canonically
- Face row = coedges grouped by `ring` (loop index; 0 is the outer loop by builder convention), each loop **cyclic in traversal order** and rotated to start at its smallest `(edge id, side)` (addendum §7.2). Order is meaningful: at a vertex with more than two coedges of one face (pinches, seams) loops are not recoverable from bare incidence. Embedded members use `ring = u16::MAX`.
- Volume and body rows are sets, sorted by `(ring, id, side)`. `ring` is the shell index; shell membership is the builder's statement, the API does not infer inside/outside.
- Edge row is `[start(Left), end(Right)]` or `[v(Both)]`.

Storage is a paged CSR (`(start, len)` spans into one flat vector): an edit appends the new row and orphans the old; `compact` rewrites in slot order. Rows of a cell never live in per-cell `Vec`s, so there is no per-element heap allocation in queries; `boundary`/`coboundary` return slices.

## Traits

`TopologyView` (read-only, object-safe, `Send + Sync`): `model_tolerance`, `dim_of`, `is_alive`, `cells(dim)`, `boundary`, `coboundary`, `carrier`, `flags`, `tolerance_override`/`tolerance`, `coincidences`, `lineage`, `born`, `revision`, and a default `canonical_hash()` (FNV-1a over everything in canonical order; floats by bit pattern).

`TopologyEdit: TopologyView` (object-safe): `begin_op`; construction `make_vertex/edge/face/volume/body`, `add_face_use`, `remove_face_use`, `embed`, `unembed`; Euler-style refinement `split_edge`, `split_face` (by chord); `merge_vertices/edges/faces`; `reverse_face`, `kill`, `set_carrier`, `set_tolerance`, `set_flags`, `record_coincidence`, `set_model_tolerance`. Imprint = `split_edge` + `split_face` + `record_coincidence`; merge = bottom-up `merge_*`; booleans/webcut in `sm-model` compose these with geometry computed elsewhere.

Contracts: edits are **atomic** (validated before mutation; tested by hash equality after a failed edit); new ids are the next unused serial of their kind, in the order documented per method; serials are never reused (dead cells stay in the table with `alive = false`).

## Requirement map

| Requirement | Where |
|---|---|
| Body > Volume > Shell > FaceUse > Face > Loop > Coedge > Edge > Vertex | Body/Volume/Face/Edge/Vertex stored ranks; Shell/FaceUse/Loop/Coedge derived views (justified above) |
| Each face two sides -> volume or exterior | `face_sides`, `Region`; validator rejects two cells on one side |
| FaceUse sense after merge, Coedge sense | `Side::sense()`, `FaceUse.sense`, `Coedge.sense`; `merge_faces` orients the shared face from the lower to the higher volume id (§6) |
| Internal faces/baffles, wires | `Side::Both` volume->face; `embed` for wire edges/points (ring `u16::MAX`); free faces/wires grouped by bodies |
| Imprint/merge status per coincident face pair | `FacePair { a, b, status: Merged / ImprintedNotMerged / CoincidentUnimprinted / Overlapping, overlap_area }`, sorted table; `merge_faces` writes `Merged`; statuses of dead faces are dropped, `Merged` history is kept |
| Stable ids deterministic | per-kind serial counters, documented creation order, `OpId` supplied by the caller |
| Lineage | `LineageRecord { op, kind: Split/Merged/Deleted/Modified, from, to }`, append-only; `born(id)` for creation; `lineage_of`, `descendants` |
| Read-only query trait separate from edit trait | `TopologyView` / `TopologyEdit` |
| Topology -> geometry | `Carrier { Point, Curve{curve, range, sense}, Surface{surface, sense}, None }` with `EntityId` handles resolved by `sm_geom_api::GeometryStore`; `validate_geometry` checks them |
| Virtual topology overlay | `TopologyOverlay: TopologyView` + `Overlay` (composite faces): virtual cells of kind `PartitionCell`, hidden host cells, patched rows; host untouched |
| Reference implementors + tests | `CellComplex`, `fixtures` (brick, touching bricks + merge, cylinder with seam, baffle), `tests/reference.rs` |

## Tolerance: DECISION

**Global modelling tolerance plus sparse per-entity overrides** (`Tolerance` in `sm-geom-api`, overrides in a `BTreeMap<EntityId, f64>`; `tolerance(id)` falls back to the global linear value).

- Native construction is exact to the global tolerance, so the dense case costs one `f64` for the whole model and every comparison has one answer.
- Imported STEP/IGES geometry carries per-entity tolerances; those become overrides, and only on the entities that need them. This is the Parasolid-style escape hatch without Parasolid's cost (every query consulting per-entity tolerances).
- Merge semantics are well defined: vertices merge when `|p - q| <= tol(a) + tol(b)`; the survivor takes `max(tol)`; `validate_geometry` checks curve ends against vertices and edge samples against faces within the summed tolerances.
- Not chosen: pure global (cannot represent imported sloppy models, forces blanket healing before use), pure per-entity (every cell stores and propagates a tolerance; tolerance growth during booleans is a known source of non-robustness).

## Persistent naming scope

Exactly P0-P1 (docs/ARCHITECTURE.md §3): stable ids and lineage, no selectors. Lineage lets a later `sm-naming` map an old id to its live descendants (`descendants`).

## Overlay (virtual topology)

`Overlay::composite_faces` creates a `PartitionCell` face standing for several host faces. Interior edges (shared by two members) and absorbed vertices are hidden; boundary edges, the parent volumes and bodies get patched rows pointing to the virtual face. The overlay is a `TopologyView`, so a mesher meshes the composite as one face; `resolve(host_id)` and `virtual_members(virtual_id)` map between layers; `clear()` restores the host. Unsupported (reported, not wrong): pinched boundaries, non-manifold interiors, members on different sides of a volume. Partition (split) cells and edge composites follow the same pattern and are not implemented here.

## Tradeoffs and open issues

- **Edit/query interleaving is O(N) per rebuild.** Every edit drops the derived index; the next query rebuilds the coboundary. Fine for op-at-a-time use, bad for algorithms that alternate thousands of edits and queries. A production store should maintain the coboundary incrementally; the trait surface does not change.
- **Loops are ordered rows, not objects.** Cheap and canonical, but loop-level edits (e.g. moving a coedge between loops) are row rewrites. `split_face` supports single-loop faces only (holes and embedded members are `Unsupported`).
- **Shells are rings the builder assigns.** The API does not decide which shell is outer or whether a volume's shell closes; `validate` checks closure (`boundary(boundary) = 0`) but not inside/outside classification. That needs geometry (a ray cast through `ParametricSurface::ray_hits`).
- **Radial order is not stored.** `radial_uses(edge)` is in face-id order; angular order around a non-manifold edge is a geometry query.
- **`merge_faces` checks edge *sets*, not loop correspondence under `relative`.** A wrong `relative` succeeds but yields a shell that `validate` flags (net edge use != 0). Known hole.
- **Chord vertex ambiguity.** `split_face` takes the first occurrence of a vertex on the loop; a pinched loop with a vertex visited twice needs the caller to pass a chord that is unambiguous.
- **`PartitionCell` is shared by overlay cells of any rank**, so rank cannot be read from the id of a virtual cell; hence `TopologyView::dim_of`.
- **Hash is exact-bit.** It is bit-identical for the same platform; the cross-platform CI hash needs coordinates quantized first (ARCHITECTURE §3), a policy for the orchestrator.
- **CSG volume carriers** are not modelled (`Carrier::None` for volumes); the CSG front end maps volumes to half-space expressions in the model layer.
- Determinism test: `building_twice_gives_identical_ids_and_hashes` (twice, plus 4 concurrent threads). The crate has no rayon, so `RAYON_NUM_THREADS=1` vs default is trivially the same; the test suite passes under both.
- `sm-testkit` does not exist yet, so the "property tests from sm-testkit" clause is vacuous; `random_edge_splits_keep_the_complex_valid` is a self-contained randomized invariant test (Euler characteristic, `validate` after every step).
