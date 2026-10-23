# sm-model-api: design (candidate: grouptheory)

Model-semantics and decomposition layers of `docs/design/ir.md` §5. Style: **group theory first (orbifold view)**.

## 1. The model in one paragraph

A model is a **fundamental piece** `F` (volumes, faces, colors), a **finite group action** on it, and **boundary identifications** (deck pairs). Symmetry (mirror / translation / rotation deck groups) and instancing (prototypes, cartesian/hex lattices, nesting) are the same thing: a `GroupAction` yielding `Placement { label, xf, improper }`. **Coloring is a function on the quotient** (`volume -> color` on `F`) pulled back along the covering projection to every copy. **Unfold and flatten are one functor**: `piece x placements -> cover`. The only place equivariance is intentionally broken is the per-placement `Recolor` of instancing, and it composes functorially (inner first, outer last). Colors are labels only; consumers resolve them to physics.

```
Coloring (on F) ──π*──▶ colors of every cover/flat entity
Symmetry  = (generators, domain: Full|Reduced, origin)   ──unfold──▶ Cover  ──refold──▶ Symmetry
Instancing= (prototypes, instances, lattices)            ──flatten─▶ Flattened ──apply_group──▶ Flattened
Decomposition = pieces over volumes (no color stored, undo stack)
```

## 2. Key types

| Module | Types |
|---|---|
| `group` | `Placement<L>`, `GroupAction` (`DeckGroup` and `Lattice` and `Cover` implement it), `DeckOp {Mirror, Rotation, Translation}`, `DeckGenerator`, `DeckGroup`, `DeckElement`, `ElemIx` |
| `xf` | `compose`, `inverse`, `approx_eq` (sm-core `Transform` is minimal and exactly-comparable), `Plane`, `Axis`, `Turns` (rational angles), `Tolerance`, Jacobi `eigen_symmetric` |
| `coloring` | `Coloring` (volume colors, face/boundary colors, nested named `Group`s), `ColorRole {Volume, Boundary}` |
| `boundary` | `BoundaryKind {Deck{boundary, side}, Response, Interface(BetweenColors / NonConformal)}`, `DeckBoundary`, `DeckSides`, `BoundaryTable` |
| `symmetry` | `Symmetry`, `FundamentalDomain {Full\|Reduced, chamber}`, `CutSpec`, `CutPlan`, `CutReport`, `Cover`, `CoverEntity`, `Glue`, `deck_partner` |
| `detect` | `detect`, `Candidate`, `Verification {Unchecked, Rejected(reason), Verified{..}}`, `DetectionReport` |
| `instancing` | `Prototype`, `Instance`, `Lattice` (`Cartesian`, `Hex` axial), `Placed`, `Recolor`, `Instancing::flatten`, `InstancePath`, `Flattened` |
| `decomposition` | `Decomposition`, `Piece`, `PieceFace {Model, Virtual}`, `PieceShape {Free, Mappable, Sweep}`, undo stack |
| `topo` | `TopologyView` (incidence), `GeometryView` (measures), `Moments`, `FaceSides` |
| `validate` | `validate`, `Report`, `IssueCode` |
| `model` | `Model` aggregate (coloring + boundaries + symmetry + instancing), `apply_cut`, `declare_deck`, `adopt_detected` |

## 3. How each requirement is met

1. **Coloring.** Volume-role colors partition volumes exactly (one color per volume by construction, validator checks totality); boundary-role colors label faces. `Group`s (block/assembly/set) nest acyclically and expand transitively. Nothing physical is stored.
2. **Boundary kind.** Every face has `Deck`, `Response` or `Interface`. A `DeckBoundary` references a generator and carries `SelfPaired(faces)` (mirror: plane fixed pointwise) or `Paired{a,b}` with `B = g(A)` (translation, rotation). Interface is `BetweenColors` (shared interior face) or `NonConformal{partner, transform}`. **Sign convention** (validated): `g` maps the outward normal of an `A` face to the *negative* of its `B` partner's outward normal; for a mirror, `g(n) = -n` on the plane.
3. **Symmetry.**
   - *Declared*: `Model::declare_deck` tags existing faces (imported pre-cut geometry; `faces_on_plane` finds the faces).
   - *Cut and keep chamber*: **cutting is a topology operation; this layer only reads topology**, so it is a two-phase protocol. `plan_cut(CutSpec)` returns the half-spaces to keep, the new generator and which plane plays which deck role (`SelfPaired`/`A`/`B`). The topology engine executes it and returns a `CutReport` (volume/face lineage, removed entities, cap faces with their plane). `Model::apply_cut` inherits colors/kinds/group membership along the lineage, retires dead entities, tags caps as deck boundaries automatically (optionally with a cap color), and records the symmetry as `Reduced`. Mirror, `1/n` wedge (`n>=3`) and periodic slab are provided; repeated cuts reuse equal generators and extend the chamber (Z2 x Z2 etc.).
   - *Detected*: `detect` (colored invariants; see below).
   - *Unfold*: `Symmetry::unfold -> Cover`. A `CoverEntity` is `(base, ElemIx)`; no topology is copied. Colors are pulled back (`Cover::volume_color`). `Cover::reverses_orientation` says which copies are improper, so the materializer reverses face loops and re-applies the left/right rule (a mirror reverses the loop; `fwd/rev` roles map unchanged). `Cover::glue(face, e)` answers where a deck face is identified: `b = g(a)` gives `A@e ~ B@(e∘g⁻¹)` and `B@e ~ A@(e∘g)`; a mirror pairs `a@e` with `a@(e∘g)`. Faces whose partner copy is outside the window stay `Open`. `Cover::refold` returns the exact `Symmetry`. `Cover::moments` lets tests prove the cover equals the original model.
   - *Translations*: a translation generates Z, an infinite group, so the generator carries a **window** `copies >= 2`; the unfold is `T^0..T^(copies-1)` and its end faces stay open. Group = `(translation window) x (point group)`, with `T^k ∘ p` (assumes translations are normalized by the point group; true for mirrors containing the translation direction and rotations about it). Closure is capped (`MAX_GROUP_ORDER = 4096`) and returns `GroupTooLarge`, never hangs.
4. **Instancing.** `Prototype` = set of volumes; `Instance` = `Placed{fill, xf (may be improper), recolor}`; `Lattice` = cartesian `Z^3` window or hex axial `(q,r,k)` with `rings`; sparse slot overrides over a homogeneous default (`Fill::Empty` holes), `outer` background, unlimited nesting (cycle-checked). `Lattice` implements `GroupAction` (pure translations). `flatten` returns leaves with `InstancePath`, transform, `improper` flag and the recolor chain. **Canonical order**: depth first, slots by `(k, row, col)` ascending (cartesian `k=z,row=y,col=x`; hex `k`, `row=r`, `col=q`); independent of slot insertion order. **Recolor semantics**: effective color = `σ_outer ∘ … ∘ σ_inner (base)`; within one level a per-volume override beats a color substitution. `Flattened::apply_group` composes a flat lattice with a deck group (symmetric reduced lattice). `fingerprint` is a stable content hash.
5. **Decomposition.** `Decomposition::identity` (one piece per volume), `split` (single volume, across a registered `VirtualCut`, sweep source/target roles may reference the cut), `composite` (merges pieces; internal cuts vanish; **refuses to mix colors**), `undo`/`reset` (snapshot stack). Pieces are `EntityKind::PartitionCell`. Pieces store no color: `piece_volume_color` derives it from the parent volume(s), so coloring cannot change. Queries: `pieces_of_volume`, `volumes_of_piece`, `pieces_of_face`, `faces_of_piece` (with `whole` flag), `cuts_of_piece`, `neighbors`, `sweep_pieces`. The geometry of cuts belongs to the virtual-topology layer, which tells this layer which model faces each side touches.
6. **Validator.** Coloring totality/role/stale entities/group cycles; every boundary face has a kind; deck on interior faces; mirror faces on the plane; paired faces match centroid+area and the normal sign rule (bijection, or aggregate area/centroid with a warning when counts differ); generator without faces; interface sanity; instancing references/cycles/prototype volumes/recolor targets; decomposition coverage (volumes and faces), composite colors, whole faces shared, cut sharing, sweep roles, measure sums.

### Detection
Needs coloring first (a symmetry maps each color to itself). Per-color moments and the global barycentre `C`; candidate directions from principal axes of the global inertia, volume-barycentre offsets from `C`, and same-color barycentre differences (capped); candidate mirrors (normal = direction) and rotations (orders `2..=12`) all through `C`. **Screening** keeps candidates preserving every color's moments. **Verification** maps every volume (moments) and face (centroid, area, normal, face color, boundary-kind class) onto a distinct image; the result is `Verified{counts}` or `Rejected(reason)` (candidates that pass screening but fail verification are reported, not hidden). Output order is canonical. Translations are not detected (a finite model has none); declare periodicity with a slab cut. The reference verification is O(V² + F²).

## 4. Topology queries needed (input to sm-topo-api)

`TopologyView` (incidence):
- `volumes() -> Vec<EntityId>` sorted
- `faces() -> Vec<EntityId>` sorted
- `face_sides(face) -> Option<FaceSides{fwd, rev}>`: normal points `fwd -> rev`; boundary face has `rev = None`, outward normal. (This is the FaceUse/sense information.)
- provided: `volume_faces(volume)`, `boundary_faces()` (an index-backed override is welcome)

`GeometryView: TopologyView` (shape-layer measures, kept separate on purpose):
- `face_centroid(face) -> Option<Point3>`
- `face_area(face) -> Option<f64>`
- `face_normal(face) -> Option<Vector3>` (outward on boundary faces; at the centroid for curved faces)
- `volume_moments(volume) -> Option<Moments{measure, centroid, second}>` (`second = ∫ x xᵀ dV` about the origin; barycentre and inertia follow)

Nothing else: no vertices, edges, loops or curve queries. Cut execution is a topology *operation*, not a query; its result comes back as a `CutReport`.

## 5. Tradeoffs

- **Cover is a view, not a materialized model.** Cheap, exactly reversible, and colors need no copying; materializing (`bake`) is the topology engine's job from the `Cover` plan.
- **Deck sets, not per-face partner pointers.** Faces on both sides may subdivide differently; the partner is found geometrically (`deck_partner`) or checked in aggregate. Cost: one lookup per face when a partner is needed.
- **Translations are windowed.** Honest about Z being infinite; open ends are explicit (`Glue::Open`).
- **Detection is exact but quadratic** in the reference verifier, and its candidate generator is bounded (not exhaustive for highly degenerate inertia with no colored offsets); the API does not change when a spatial index replaces the scan.
- **No rayon.** The allowed dependency set for API crates excludes it, so the 1-thread vs N-thread test is vacuous by construction. The determinism test runs the pipeline twice and across spawned threads and compares hashes (fixed-key hasher). Everything is `BTreeMap`/sorted `Vec`; no RNG.
- **Mock topology is public** (`mock`) so downstream crates can write the same tests until sm-topo exists; it is not a kernel.
- `Recolor` per level is cloned into an `Arc` once per visited container, not per leaf.

## 6. Open points

- `EntityKind::Instance` is shared by prototypes, instances and lattices; a registry lookup disambiguates. Separate kinds would be cleaner (orchestrator decision).
- `Cargo.lock` gains `smallvec`, `thiserror` (merge step updates it).
- Interface records at the mesh layer (`EntityKind::Interface`) are lowered from `BoundaryKind::Interface` via classification; not designed here.
- Mixed-material cells (volume fractions) remain open as in ir.md §7; coloring is an exact partition.
- Hex lattice rotational/mirror symmetry of the lattice itself (e.g. D6 of an assembly) is expressible with `DeckGroup` + `apply_group`; automatic detection on lattices is not implemented.
