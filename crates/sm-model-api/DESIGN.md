# sm-model-api: final design (frozen contract)

Model-semantics and decomposition layers of `docs/design/ir.md` §5, as **attribute tables over
topology**. Base: the `tables` candidate, with grafts from `grouptheory`/`selections` and the
must-fix list of `docs/design/w0/semantics-recommendation.md`, modified by the W0c decisions in
`docs/design/w0/decisions.md`. The crate depends on `sm-core`, `sm-topo-api` (its
`TopologyView` read trait, `Lineage`, `CutOutcome`, `CapFace`, `FaceMoments`, `Moments`,
`coincident_faces`) and `sm-geom-api` (feature chain only); it never mutates topology.

## 1. The model in one paragraph

A `Model` is a set of private tables keyed by `EntityId`: colors (with numeric labels), volume and
face colors, groups, boundary records, a face-keyed role table, one optional `Symmetry`, an
append-only `Provenance` log, instancing tables and the decomposition tree. Topology is only read.
Topology *changes* (cuts) come back as data and are applied by `Model::apply_cut` /
`Model::apply_lineage`, which re-key every table through the lineage transactionally.

| Module | Content |
|---|---|
| `color` | `ColorRow{id,label,name,domain}`, `Domain`, `GroupRow`; lookups by label and name; recursive group expansion |
| `boundary` | `Deck{Mirror,Translation,Rotation}`, `InterfaceIntent`, `BoundaryKind`, `DeckSide`, `FaceRole`, `FaceKindView`, `DeckLookup` |
| `symmetry` | `Symmetry`, `Provenance`, `CutSpec`/`plan_cut`/`apply_cut`, `point_group`, `declare_symmetry` |
| `detect` | `detect_symmetry`, `verify_transform`, `Candidate`, `Verification{Unverified,Verified,Rejected{stage,..}}` |
| `unfold` | `unfold`, `UnfoldedModel`, `CopyRef{base,elem}`, `Glue` (a view; mints nothing) |
| `instancing` | `Prototype`, `Lattice`, `InstanceRow`, `ColorMap`, `PathOverride`, `flatten`, `FlatModel` |
| `decomposition` | `PieceRef{Whole,Cell}`, `PieceFace{Whole,Part,Cut}`, `split`/`composite`/`unsplit` |
| `validate` | `validate`, `validate_unfolded`, `Report`, `IssueCode` |
| `xf` | `Tolerance`, `Plane`, `Axis`, `Turns`, `transform_moments`, `transform_face_moments`, internal cache and bins |

## 2. Decisions that shape the API

1. **Deck kinds are per face.** Membership, side and mate live in one table keyed by face
   (`FaceBoundary{boundary, role}`), so the two directions cannot disagree. A reflective unit cell
   with mirrors at `x = 0` and `x = 1` validates. Only *declaring* a symmetry needs a finite point
   group: `declare_symmetry` closes the point part (BFS, deduplicated through a quantized index
   with neighbour-cell probing, `GROUP_TOL = 1e-9`) and returns `GroupNotFinite` for parallel
   mirrors (they compose to a translation; declare a translation with a window instead).
2. **Parameters are the source of truth.** `Deck` stores plane / vector / axis + `Turns{num,den}`
   (exact rational angle, reduced, order = `den`); `Deck::transform()` derives the matrix.
   Degenerate decks cannot enter a model (`Deck::check` in `add_boundary`), so `transform()` is
   infallible by contract.
3. **Sign rule `n_B = -g(n_A)`** (`IssueCode::DeckOrientation`): a translation pair whose volumes
   would end up on the same side is rejected even when the geometry coincides. For a mirror
   `g(n) = -n`, so the face is its own mate with the same outward normal.
4. **Geometry checks use integrated moments** (`FaceMoments`: area, first, second, vector
   area): independent of seam position, sensitive to shape (second moment), exact under rigid
   transforms (`transform_face_moments`). A planar face is `nᵀSn = d²A` (`FaceMoments::plane`).
   All comparisons scale with the model's length scale (cube root of total volume).
5. **Cuts are two-phase.** `plan_cut(&CutSpec) -> CutPlan` (pure: half-spaces whose normals point
   into the kept region, plus the deck role of each cap); the kernel executes it and returns a
   `CutOutcome{lineage, caps}`; `apply_cut` verifies caps lie on planned planes (by moments), that
   kept volumes lie in the half-spaces, re-keys, tags caps (mirror self-paired, slab/wedge A/B
   paired by moments under the deck transform), applies `cap_color`, reuses an equal existing
   generator, records `Provenance::Cut`. No second code path for pre-split geometry: the kernel
   returns an empty lineage and only caps.
6. **Lineage chains are resolved transitively** (`webcut` records `Split`, `kill_cascade` records
   `Deleted` on the discarded child; the color must land on the live child). When a deck mate
   splits, children are re-paired by matching face moments under the deck transform; the pairing
   stays reciprocal by construction.
7. **Unfold is a view.** `CopyRef{base, elem}` names a copy; element order is translation shifts
   (first generator slowest) times the point group. Glue rule `A@e ~ B@(e∘g⁻¹)`, mirror
   `a@e ~ a@(e∘g)`; faces whose partner is outside the window stay `exposed`. `kind_of` turns a
   glued deck face into `Interior`; `sense` is `Reversed` exactly for improper copies (the loop of
   each image face is reversed so the stored normal follows the right-hand rule again, then
   left/right is re-derived from volume ids; sense is never mixed with that swap);
   `origin_of` follows the recorded cut lineage to the pre-cut entity; `refold` returns the exact
   `Symmetry`. The view owns its data (serde).
8. **O(1) deck lookup for tracking.** `Model::deck_lookup(&t)` builds a dense table once;
   `hit_by_ordinal(i)` is O(1) (`i` is the index in `cells(Dim::Face)`), `hit(face)` adds one
   binary search. `DeckHit{mate, to_mate, improper}`; B faces carry the inverse transform.
9. **Interfaces are two M:N face sets** with an optional transform. Facing = summed vector areas
   opposed (after the transform); every face needs a counterpart (a recorded
   `coincident_faces` pair, or overlapping bounds); a face recorded `Merged` with a face of the
   interface, or an interface declared on a shared face, is rejected. Coincidence is read from
   topology, never inferred from moments. Interior faces between different volume colors derive
   `Interface(Conformal)` in `face_kind`; the validator only requires an explicit kind on boundary
   faces. An uncolored boundary face is a **Warning**; `Report::promote` turns it into an error at
   export time.
10. **Detection** (`detect_symmetry`): per-color invariants (stage 0, necessary), then volumes
    (stage 1) and faces with color, boundary record and side volumes (stage 2) through binned
    matching, O(n log n). Directions come from the coordinate axes, principal axes, planar-face
    normals, face-centroid offsets, same-color volume offsets and bisectors, and cross products of
    axes with those, so degenerate inertia (hex, C4) is covered. Only the maximal cyclic order per
    axis is reported, plus a greedy minimal generator set and the group order; `complete` is false
    when an eigenspace was degenerate and no in-plane source existed. Candidates that pass stage 0
    but fail later are reported with `Rejected{stage, residual, reason}`;
    `Model::declare_detected` refuses unverified ones unless forced. Translation symmetry is
    declared, not detected.
11. **Instancing** shares `Placement{transform, orientation_reversed}` with unfold. Copies are
    named by `InstancePath` (`Instance` then `Slot{lattice, ix}` steps); no ids are minted per
    copy, so ids are never reused and removing a slot shifts nobody. Slots order by `(k, j, i)`;
    `FlatModel.paths` are strictly increasing, so `find` is a binary search over a placement arena
    (`entries` hold `u32` indices, not 240 B clones). Recolor order: prototype, inner slot, outer
    slot, instance, then `PathOverride`s (deeper prefixes first, so outer wins); a per-entity key
    beats a per-color key inside one `ColorMap`; domains are checked on insertion and by the
    validator. Hex `rings = 5` is 61 slots.
12. **Decomposition** is an implicit binary cut tree per volume: an undecomposed volume is
    `PieceRef::Whole(volume)` (virtual, nothing minted). `split` mints two `PartitionCell` ids,
    keeps the parent inactive, `unsplit` restores the parent with its original id; ids are never
    reused. Pieces have no color field (read through the volume). `PieceFace::Part{face,piece}` is
    the mesh classifier's key for the part of a face that straddles a cut; cut faces are local
    `CutId` handles (this layer mints no `PartitionFace` ids, which belong to the topology
    overlay). `composite` refuses mixed colors.

## 3. Topology queries used (all from the `sm-topo-api` read trait)

`cells`, `volume_faces`, `face_edges`, `face_ordinal`, `face_moments`, `volume_moments`,
`face_sides` / `face_class` (derived), `coincident_faces`, `boundary` (bodies), `dim_of`,
`is_alive`, `tolerance`. Nothing else; no mutation.

## 4. Determinism and ids

`BTreeMap`/`BTreeSet`/sorted `Vec` everywhere; outputs in ascending id or documented canonical
order; no randomness, no threads. Ids minted here (`Color`, `Group`, `Boundary`, `Prototype`,
`Lattice`, `Instance`, `PartitionCell`) come from one per-kind allocator starting at serial 1 and
never reused. `tests/determinism.rs` runs the whole pipeline twice and on four `std::thread`s and
compares hashes (API crates do not use rayon).

## 5. Deviations from the recommendation / decisions (honest list)

- Topology parameter is `&dyn TopologyView` rather than `&impl TopologyView` (the trait is
  object-safe; `TopologyViewExt` works on `dyn`).
- `apply_cut` returns the deck `EntityId` of the caps instead of `()`.
- `plan_cut` is fallible (`Result`): a wedge must span less than half a turn and have a start
  point off the axis. `CutPlan` has an extra `roles` field.
- `CutId(u32)` is kept as the local handle for virtual cut faces (no `PartitionFace` minted here).
  A cut face of a parent piece that straddles a further split is inherited by both children as
  `PieceFace::CutPart{cut, side, piece}` (a distinct key per child, like `Part` for model faces);
  `Model::split_sides` classifies the parent's cut faces explicitly (`SplitSides`), and
  `split` assumes every parent cut face straddles.
- `composite(pieces, t)` takes the topology so the faces of `Whole` members are captured when the
  composite is formed; `pieces_of_face` then finds composites of whole volumes too, and
  `pieces_of_face_in(f, t)` adds the `Whole` pieces of volumes that are not decomposed.
- `apply_lineage` also refuses a change to any model face listed by a piece, not only a change
  of the decomposed volume id (`DecompositionActive`).
- Coincidence of interface faces is read from the topology record only (binding decision 5).
  The one exception is an interface with an explicit `transform`: the two face sets are not
  colocated, so no record can exist, and the transformed bounding **spheres** (from the second
  moments, r = 1.01 sqrt(3 tr(S_c)/A)) must overlap. This is the only place geometry stands in
  for a record.
- `apply_cut` runs the point-group closure on the new generator set (`GroupNotFinite`), checks
  the shape of a hand-built `CutPlan`, and checks the kept side by the volume centroid **and**
  every face-outline corner. Volumes with few or no corners (smooth closed surfaces) are only
  checked by their centroid.
- `Deck::check` requires a rotation angle to be a reduced proper fraction `0 < num < den` with
  `den <= MAX_GROUP_ORDER`; the validator and `plan_cut` use widened arithmetic.
- `unfold` bounds the element count (`MAX_GROUP_ORDER`) with checked arithmetic before it
  allocates. Lattice layouts are bounded to `MAX_LAYOUT_SLOTS` and `flatten` to
  `MAX_FLAT_ENTRIES`.
- With `serde`, `Model` deserializes through `Model::check_integrity` (row kinds and keys, id
  allocator ahead of every minted id, acyclic lattices and cut trees, bounded layouts).
- `FlatModel::fingerprint` hashes raw `f64` bits and is **single-platform**;
  `fingerprint_quantized(linear)` applies the Q2 step `linear * 2^-10` and is the cross-platform
  hash.
- `DetectionResult::complete` is false when an eigenspace is degenerate and no geometric
  direction source (face normal, face or volume offset; cross products do not count) lies in
  it. It is a heuristic, not a proof of completeness.
- A mirror cut that deletes one face of an existing translation or rotation pair fails with a
  `CutInvalid` that names the lost partner; unassign the pair first.
- Extra issue codes: `RotationOrder` (a deck rotation must be a primitive `1/n`), `GeometryUnavailable`
  (warning: moments unsupported for a face, e.g. at a pole).
- `detect_symmetry` and `verify_transform` propagate `Topology(Unsupported)` when a face has no
  moments (pole faces, faces without a surface carrier): detection hard-fails there instead of
  degrading to `complete: false`; only the validator degrades (`GeometryUnavailable`).
- `color_by_label` / `color_by_name` are linear scans over the color table (not indexed).
- Two parallel mirrors cannot be declared as one symmetry (`GroupNotFinite`); the cell stays valid.
- `Cargo.lock` is not touched (decisions.md Q3); the orchestrator regenerates it at merge.
- `serde_json` is a **dev-dependency** only (round-trip tests); the shipped dependency set is
  `sm-core`, `sm-geom-api`, `sm-topo-api`, `serde` (feature), `smallvec`, `thiserror`. `indexmap`
  is not needed.
- `sm-testkit` does not exist yet, so its property tests are not used.
- Moments of faces touching a pole (sphere, cone apex) are `Unsupported` in the reference
  topology, so deck/interface geometry checks on such faces report `GeometryUnavailable` instead
  of a verdict.

## 6. Open points

- Mixed-material cells stay out (ir.md §7): volume coloring is an exact partition.
- `EntityKind::PartitionFace` for cut faces would let `CutId` become a persistent id once the
  decomposition layer and the overlay agree on one allocator.
- Hex lattice symmetry (D6 of an assembly) is expressible with decks over a prototype but is not
  detected on lattices.
- Slab/wedge cuts need a 1:1 pairing of cap faces; an M:N cap pairing needs a kernel that imprints
  the two caps identically.
