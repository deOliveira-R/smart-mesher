# sm-topo-api: design (candidate "occt-like")

Cellular non-manifold complex in the OCCT/Parasolid tradition: **shared entity handles + orientation + location + explicit use records + per-entity tolerance.** Depends on `sm-core` and `sm-geom-api` only.

## Hierarchy

```
Body ─ Volume ─ Shell ─ FaceUse ─▶ Face ─ Loop ─ Coedge ─▶ Edge ─▶ Vertex
          │                          │                          ╰ curve: CurveId
          ╰ Wire (embedded)          ╰ surface: SurfaceId, surface_sense
Body ─ free Wire        Instance ─▶ (Body | Volume) + Sense + Location (+ parent Instance)
```

- **Entities with `EntityId`:** Vertex, Edge, Face, Volume, Body, Instance (typed newtypes `VertexId` .. `InstanceId`). They are **shared**: an edge is one record used by every adjacent face.
- **Use records** (no ids; addressed relative to their owner): `Coedge` (an edge used by a loop/wire, with `Sense` and an optional pcurve), `FaceUse` (a face used by a shell, with `Sense`), `LoopData`, `ShellData`, `WireData`. `CoedgeRef { owner, ring, index }` and `FaceUseRef { volume, shell, index }` address them. This justifies the "equivalent" in the brief: sm-core has no `EntityKind` for Loop/Coedge/Shell/FaceUse, and the sm-core contract is frozen, so those are dependent records of their owner. Schemes key per-use attributes (Cubit vertex types per surface, sweep source/target) on `CoedgeRef` or `(FaceId, VertexId)`, never on the shared vertex.
- **Orientation (binding, documented in `data.rs`):** face normal = surface normal x `surface_sense`; outer loops are counter-clockwise about the normal, inner loops clockwise; `FaceUse::Forward` = the normal points **out of** the volume; `Coedge::Forward` = start to end.

## Two sides, FaceUse sense, exterior

A face has up to two uses; **sides are derived, not stored**: `face_sides(f) -> { front, back }` where `front` is where the normal points. `Forward` use fills `back`, `Reversed` fills `front`; an unfilled side is `Side::Exterior` (the implicit complement, as in DAGMC's implicit complement and ir.md §5.2).

| State | Uses | `face_role` |
|---|---|---|
| model boundary | one use | `Boundary` |
| merged interface | two uses, different volumes, **opposite** senses | `Interface` |
| baffle / internal face | two uses, same volume, opposite senses | `Baffle` |
| free sheet | none | `Free` |
| illegal | >2 uses or equal senses | `Invalid` (flagged by `validate`) |

Two merged bricks: the survivor face has A `Forward` and B `Reversed`; the test asserts this. A baffle sits in the volume's outer shell as two uses and its edges cancel in the shell balance.

## Imprint / merge status

`CoincidenceRecord { a, b, status, overlap_area }` per face pair (`a < b`), `ImprintStatus` = `Merged | ImprintedNotMerged | CoincidentUnimprinted | Overlapping` (ir.md §3). `merge_faces` sets `Merged` itself; the other states are recorded by the imprint algorithm via `set_coincidence`. For a merged pair `a` is the survivor and `b` the retired id (history, resolvable through lineage). So "touching but not shared" is a queryable state.

## Query vs edit

- `TopologyView` (read-only, object-safe, `Send + Sync`): record access, `edge_coedges`, `vertex_edges`, `face_uses` (required, served from reverse indices), plus derived `face_sides`, `face_role`, `edge_faces`, `volume_faces`, `face_edges`, `coedge`, `coedge_vertices`, `coincidence`, `instance_location`, `lineage`, `virtual_overlay`. Meshers, semantics and tracking depend only on this.
- `TopologyEdit: TopologyView` (object-safe): `begin_op(OpId)`/`end_op`, `make_vertex/edge/face/body/volume/instance`, `add_wire`, `add_baffle`, and the Euler-style pairs `split_edge`/`merge_edges`, `split_face`/`merge_faces`, `merge_vertices`, plus `set_coincidence`, `raise_tolerance`, `record_generated`, `delete_*`. Every edit is atomic (an `Err` leaves the complex bit-identical; tested via the content hash) and attributed to the caller-supplied `OpId`, so replaying the timeline reproduces ids and lineage. Booleans, imprint and heal are algorithms *built on* this trait (`sm-topo`); they are not in the contract.

## Ids and determinism

`IdAllocator`: a never-reused counter per `EntityKind`; serial = creation order = arena index in the reference `Complex`. Operations creating several entities of one kind do so in a documented canonical order (brick: vertices by corner index, edges by axis then low corner, faces `-x +x -y +y -z +z`; split results `[first, second]`; merge survivor = lowest id). `topology_hash(&dyn TopologyView)` hashes structure, senses, ids, lineage and **quantized** coordinates (quantum `10 * linear_min`) so it matches across platforms. Tests: same build twice and from four threads give the same ids and hash; a failed edit leaves the hash unchanged.

## Lineage (P0-P1 only)

`LineageLog` of `LineageRecord { op, kind }`, `kind` = `Created | Generated | Split | Merged | Modified | Deleted`. Splits retire the parent and mint fresh child ids; merges keep the lowest id. `successors(id)` follows split/merge chains to live entities (empty if deleted); `parents(id)` goes one step back. No selectors (ARCHITECTURE §3).

## Tolerance model: per-entity tolerant topology (decision)

Each vertex, edge and face carries its own `tol` (OCCT). `TolerancePolicy` (from sm-geom-api) bounds them: floor `linear_min` (small tolerances are raised to it), ceiling `linear_max` (growth beyond it is an error, not silently accepted).

Why not one global tolerance: imported STEP/IGES/SAT is exactly the tolerant kind (gaps of 1e-6 to 1e-4 next to exact 1e-9 features); one global value is either too small to sew the sloppy parts or large enough to collapse the fine ones, and neutronics models mix mm-scale pins with m-scale vessels. Merge needs a pairwise test anyway (`dist <= tol_a + tol_b`, `coincident`).

Rules that keep it safe and deterministic: tolerances only **grow** (`raise_tolerance`), only inside an op, always logged as `Modified`; merging grows the survivor to `max(own, dist + other)` (covers the other's ball); invariants checked by `validate`/`check_edge_geometry` (edge end within `edge.tol + vertex.tol` of its vertex). The known costs: non-transitive coincidence (A~B, B~C, not A~C) needs the merge order to be canonical (lowest-id survivor, ids ascending), and tolerance creep needs the ceiling plus healing. Cost accepted over the global model's all-or-nothing behavior.

## Instancing and locations

`Location(Transform)` (compose, inverse, `flips_orientation`) and `ShapeRef { id, sense, location }` are the topology-level instancing mechanism: an `Instance` places a shared Body/Volume definition, optionally inside a parent `Instance` (lattice cell); definitions are never copied. A mirror (`det < 0`) reverses effective orientation (`Location::orient`), which is how an unfolded reflected copy flips loops and `FaceUse` senses (ir.md §5.1 constraint 1) without touching the prototype. Lattice *indexing* semantics (hex/cartesian grids) belong to the instancing layer; this crate supplies the nesting and placement it needs.

## Virtual topology overlay

`VirtualTopology` (object-safe) holds `VirtualPiece { id: PartitionCell, kind, hosts, defined_by }` with kinds `CompositeFace | CompositeEdge | PartitionFace | PartitionVolume | Collapsed`. Hosts are referenced by id and are never modified. `TopologyView::virtual_overlay()` is the hook (default `None`); `WithOverlay { host, overlay }` is a wrapper that delegates every query to the host and only adds the overlay, so the host hash is provably unchanged (tested). `OverlayTable` is the reference implementor (deterministic ids, one composite per face).

## Geometry references

Faces/edges hold `SurfaceId`/`CurveId` into a `GeomStore` (many-to-one sharing). Coedges optionally hold a `PCurve` (same parameter as the edge). The topology never evaluates geometry except in the optional `check_edge_geometry`.

## Reference implementors and builders

`Complex` implements `TopologyView + TopologyEdit` (arenas with tombstones; reverse indices rebuilt after structural edits, O(N) per edit, deliberately simple). `builders::{make_brick, make_cylinder}` build over any `TopologyEdit`. Tests (`tests/complex.rs`): brick; two bricks merged (opposite senses, 3-coedge non-manifold edges); face split with lineage (neighbour face's loop gains the split-edge halves, shell stays closed); cylinder with periodic seam (seam edge used twice, pcurves at `0` and `2 pi` agree with the edge curves); baffle; determinism (ids, hash, threads); tolerance rules; atomic failures; instancing with a mirror; overlay; object safety.

## Tradeoffs and limits

- Records are returned as `&Data` structs (AoS) rather than an SoA/CSR view: CAD-scale topology (1e3-1e5 entities) is not the hot loop; meshers copy what they need into CSR.
- No half-edge/radial-order structure: around a non-manifold edge, `edge_coedges` gives an unordered (id-ordered) set; the radial order needs geometry and is a derived query for a later crate.
- `split_face` handles a single-loop face with no embedded edges; general cases (holes, embedded curves) compose from multiple calls or need an extension.
- Coincidence records involving retired faces are kept as history.
- `merge_edges` drops the absorbed coedges' pcurves (parameterizations may differ); recomputing them is the geometric algorithm's job.
- Lineage `Split` retires the parent (fresh child ids) rather than letting one child inherit the id: simpler and stable, at the cost of one more id per split.

## Open issues for the orchestrator

1. `EntityKind` has no Loop/Coedge/Shell/FaceUse; handled as owner-relative addresses. If persistent per-coedge ids are wanted for P2 naming, sm-core needs new kinds (api change request, not needed for P0-P1).
2. `sm-core::Transform` has no compose/inverse; implemented in `Location` here.
3. Cargo.lock changes (new crates) are not committed on this branch.
