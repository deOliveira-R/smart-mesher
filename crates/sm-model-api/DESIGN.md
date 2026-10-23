# sm-model-api: design (candidate "selections")

Model-semantics and decomposition layers of ir.md §5. API crate with a reference implementation, tested against a mock `TopologyView`. Deps: `sm-core`, `thiserror`, `smallvec`; optional `serde` (feature). Feature `mock` exposes `mock::TableMock` for other crates' contract tests.

## Core idea

Everything the layer stores is a **typed selection with a lifetime and invariants** (STK parts/selectors, SGC active selections).

| Thing | Selection of | Invariant |
|---|---|---|
| Volume color (abstract material id) | `Selection<Volume>` | exclusive (a volume has at most one); must cover all domain volumes (validator) |
| Boundary color | `Selection<Face>` | exclusive per face |
| Group (block, assembly) | volumes, faces, or other groups | none (labels may overlap); nesting acyclic by construction |
| Boundary kind (`KindSet`) | `Selection<Face>` + `BoundaryKind` | exclusive; must cover every boundary face of the domain |
| Deck pairing | face pairs `(a, b)` + `Generator` | `b = g(a)`, `n_B = -g_lin(n_A)`; mirror self-paired |
| Piece (`PartitionCell`) | `Part`s: volume restricted by a cut path | leaves tile each domain volume exactly; holds no colors |
| Prototype | `Selection<Volume>` | volumes exist |

`Selection<K>` (`K` = `Volume | Face`): sorted, deduplicated `Vec<EntityId>`; construction rejects wrong-kind ids; algebra `union / intersection / difference / complement` as sorted merges. `Lifetime::GeometryBound` (topology ids, source of truth) vs `Lifetime::MeshBound{mesh}` (mesh ids obtained by `lower()` through classification; derived, cannot be stored as coloring, cannot be mixed with geometry-bound ones).

`Selector<K>` is the unevaluated expression (`All`, `InDomain`, `Ids`, `Named`, `Color`, `Kind(filter)`, `Unassigned`, `DomainBoundary`, `Piece`, `Not/And/Or/Diff`, `FacesOf`, `VolumesOf`), resolved against model + topology. Ops can store selectors and re-resolve after topology changes. Colors are labels only; no physics anywhere.

## Boundary-ness is relative to the domain

The **domain** is a volume selection (the fundamental domain). A face is a *boundary face of the domain* iff exactly one side is in the domain. This makes cut-and-keep-chamber a view change that needs no geometry kernel:

1. `cut_keep_chamber(CutSpec)`: chamber half-spaces from the generator (mirror: one plane; translation: slab; rotation order n >= 3: wedge of two planes, optional start direction).
2. Domain := volumes whose barycentre is inside every half-space.
3. Domain-boundary faces lying on a cut plane (centroid distance and normal alignment within tolerance) become the deck set. Mirror: `(f, f)`. Translation/rotation: A on plane 0, B on plane 1, matched by `B = g(A)` (centroid and area); unmatched faces fail with `PairingFailed`.
4. The generator is recorded in the symmetry; a detected-but-uncut generator gets its pairing filled in.
On any error the domain is restored. **The physical split of a straddling volume is a topology-layer operation done beforehand**; the model layer contributes plane, keep predicate and automatic tagging. A plane that cuts through an unsplit volume finds no cut face and fails with `NoCutFaces` (tested). Volumes outside the domain are not deleted; undoing the cut is `set_domain(None)`.

## Boundary kinds

`BoundaryKind::{Deck{pairing, side}, Response{tag}, Interface(InterfaceIntent)}`. `InterfaceIntent{between: Option<(color, color)>, conformity: Conformal | NonConformal{partner}}`. Response tags are opaque strings. The validator requires a kind on every domain-boundary face, rejects deck kinds on interior faces, and checks `between` against the colors on the two sides.

## Symmetry

- `Generator::{Mirror{plane}, Translation{origin, vector}, Rotation{origin, axis, order}}`; `transform()` gives the affine map (mirror det = -1).
- **Declared**: `declare_symmetry` (imported pre-cut geometry: tag faces with `declare_deck`, then declare), or by `cut_keep_chamber`.
- **Detected**: `detect_symmetry` returns `DetectionResult{barycentre, per-color invariants (measure, barycentre, second moment), candidates}`. Every symmetry of a colored domain fixes its barycentre G. Mirror candidates: perpendicular bisectors of same-color volume pairs plus coordinate planes through G. Rotation candidates: axes through G along coordinate axes and same-color barycentre differences; largest verified order up to `max_rotation_order`. Candidates are deduplicated by quantized canonical key and sorted. Each is verified (`Verification::{Unverified, Verified{max_residual}, Rejected{stage, residual}}`) in three stages: per-color barycentre and second-moment invariance; every volume has a same-color image with equal measure and `M S M^T` moments; every face has an image with equal area, aligned normal, same side colors, boundary color and boundary kind. `declare_detected` refuses non-verified candidates unless forced. The search is heuristic (can miss symmetries); verification is sound up to tolerance. Translations are not detected (a finite model has none).
- **Unfold** (`Model::unfold`): derived full model. Group elements by BFS closure of mirrors/rotations under right-multiplication (dedup by quantized matrix, cap `max_elements`), times translation powers `0..repeat` (a translation without `repeat` is `UnboundedGenerator`). Copy 0 keeps the original ids; copies get ids `kind#(id_base + n)` in canonical order. Deck faces coincident across copies are glued: for pair `(a, b)`, `b@c` is dropped and `a@c'` (`G_c' = G_c * S`) gains the volume behind `b` as `right`; mirror faces keep the lower copy. Improper copies set `loop_reversed`, with left/right kept so normals still point out of `left`. Also set when the stored normal pointed into the domain (domain-relative swap). Reversibility: `UnfoldedModel{source_of, fundamental_domain, refold, generators}`; the unfold is a pure function of `{domain, generators}`.

## Instancing (shares the group-action machinery)

`action::Placement{transform, orientation_reversed, path}` is what both unfold elements and lattice flatten produce. `Instancing` holds `Prototype` (volume selection), `Lattice` (cartesian `i,j,k` or hex axial `q,r,k`, pointy or flat, nestable, sparse: explicit `None` is a void, unlisted slots use `default_fill`), `Instance` (top-level placement). A `Fill` is `{target, overrides: ColorMap, local transform}`. Flatten is depth-first in lexicographic slot order; derived ids `Instance#(id_base + n)` in emission order; path = `Instance` step then `Slot{lattice, slot}` steps; cycles are `Cycle` errors. Override rule: **nearest the leaf is applied first**, then each enclosing level; collapsed into one map per leaf. Hex pointy basis `a1 = (p, 0)`, `a2 = (p/2, p sqrt3 / 2)`; flat basis `a1 = (p sqrt3/2, p/2)`, `a2 = (0, p)`.

## Decomposition

`Decomposition` = pieces over `Part{volume, path of (cutter, side)}` plus recorded `Cutter{surface, neg_only, pos_only}` (cuts are recorded, not applied; a cutter lists faces lying wholly on one side, all other faces straddle and belong to both sides, e.g. sweep source/target). Ops: `trivial` (one per volume), `split_piece` / `unsplit` (reverse of the last split), `composite` (mappable chunk over several volumes), `set_hint` (sweep source/target). Queries: `pieces_of_volume`, `volumes_of_piece`, `faces_of_piece` (model faces plus virtual `PieceFace::Cut{..other piece}`), `pieces_of_face`, `check_tiling`. Pieces store no colors: `Model::piece_colors` derives them from volumes, and `coloring_fingerprint()` (FNV over canonical colors) is unchanged by every decomposition op (tested); the validator warns when the fingerprint recorded at decomposition time is stale.

## Validator

`Model::validate(topo, tol) -> Report{issues sorted by (code, entities)}`: unknown entities; volume coloring partition (uncolored, multi-colored, empty); uncolored boundary faces (warning); boundary kind on every domain-boundary face; deck on interior; interface vs colors; deck pairs (mirror: centroid on plane, normal parallel; translation/rotation: `g(centroid_A) = centroid_B`, equal area, `n_B = -g_lin(n_A)`; same adjacent color); symmetry bookkeeping; instancing (missing volumes, unknown override colors, cycles); decomposition tiling and staleness.

## TopologyView (input to sm-topo-api)

```
fn volumes(&self) -> Vec<EntityId>;                                   // ascending
fn faces(&self) -> Vec<EntityId>;                                     // ascending
fn volume_faces(&self, volume: EntityId) -> Vec<EntityId>;            // ascending
fn face_sides(&self, face: EntityId) -> Option<FaceSides>;            // {left, right}; right None = boundary
fn volume_mass_properties(&self, volume: EntityId) -> Option<MassProps>; // measure, barycentre, central second moment
fn face_properties(&self, face: EntityId) -> Option<FaceProps>;       // centroid, unit normal (left to right), area
```
No edge, vertex, curve or surface queries are used; no mutation. All lists ascending so determinism does not depend on the implementor.

## Tradeoffs

- **Domain as a view, not a topology edit.** Cheap, reversible, kernel-free; costs: cut-away volumes still exist, the precondition "volumes are pre-split at the plane" is on the topology layer, and straddlers are detected only indirectly (no cut face found).
- **Centroid-based keep predicate and face matching** are exact for pre-split convex-ish pieces but heuristic in general (tolerances `Tolerance{length, angle, relative}`). Face geometry is summarized by centroid/normal/area, so non-planar or partially overlapping deck faces are only checked at that resolution.
- **Verification is O(V^2 + F^2)** linear scans (fine for model-level entity counts, not for million-face models; a spatial hash would replace it).
- **Explicit pair lists** for deck faces (not recomputed geometrically) make unfold and validation simple and deterministic; the validator guards them.
- **Rotation order 2 cannot cut a wedge** (half-space with two faces on one plane); it can be declared and unfolded.
- **Unfold does not merge** volumes that become adjacent across glued faces (e.g. two halves of a bisected volume stay two volumes with equal color); the full-model topology merge is a topology-layer pass.
- **Mixed generator groups**: translation powers are applied after the point operation (`P T^k`), valid when the point group preserves the translation direction; screw/glide groups are not modelled.
- **Quantized keys (1e-7)** dedupe group elements; elements within 1e-7 of a rounding boundary could split.
- Cargo.lock gains `thiserror` and `smallvec` entries (committed).

## Determinism

No `HashMap`/`HashSet`; `BTreeMap`/`BTreeSet`/sorted `Vec` everywhere; no randomness; no parallel sections (so thread count cannot affect output). `tests::determinism_two_runs_hash_equal` hashes (quantized) unfold, validation, detection and nested flatten across two full runs. `sm-testkit` does not exist yet in this tree, so there are no testkit property tests; rayon is not an allowed dependency of API crates.

## Open issues

- `action::{compose, inverse, rotation, reflection, translation}` are free functions here because `sm-core::Transform` has none; candidates to upstream.
- Mixed-material cells (volume fractions) remain open per ir.md §7; colors here are an exact partition.
- Persistent naming (P0-P1): ids are stable and deterministic; no selectors or lineage are stored here beyond `Selector` expressions, which are query-time only. Lineage for edits that split or merge entities (re-pointing selections) is not implemented; the topology layer's lineage record would drive it.
- Edges and vertices are not colored; vertex types and corner picking belong to the intent layer.
- Coordinates of derived entities in the unfolded model (transformed geometry) are the topology layer's job; this layer returns placements and id maps only.
