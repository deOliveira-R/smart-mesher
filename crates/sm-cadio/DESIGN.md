# sm-cadio: STEP import (AP203 / AP214 / AP242 B-rep), content-addressed

**Layer L2.** Depends on `sm-core`, `sm-geom-api`, `sm-topo-api`, `sm-op-api`, `sm-layer-api`
(contracts) and, for tests only, `sm-topo`, `sm-testkit`. No OCCT, no C, no other W3a crate.

## Positioning

Similar to OpenCASCADE's `STEPControl_Reader` (and the FreeCAD and Gmsh importers built on it):
an AP203/214/242 B-rep reader that keeps analytic surfaces and NURBS with their trimming loops.
It differs in being pure Rust with a hand-written Part 21 parser (byte offsets in errors, a depth
cap, no EXPRESS code generation), in reading straight into our non-manifold cell complex with
deterministic ids and a fixed creation order, in naming the file by its content hash so the
import is a replayable, forkable timeline op, and in judging validity with our own validators
(`sm_topo_api::validate` / `validate_geometry`) with located, classified diagnostics. That gives
the neutronics code exact planes, quadrics and tori in the tracking structure instead of the
facets a DAGMC-style flow starts from.

## Layout

| Module | Content |
|---|---|
| `part21` | tokenizer and parser: `parse(&[u8]) -> StepFile { header, entities: BTreeMap<u64, Entity> }` |
| `decode` (`read_step`) | `StepFile` to `BrepModel`: units, geometry, topology, edge ranges, weld, closure check. Pure, no topology store |
| `gcodec` | curves and surfaces from STEP entities, scaled into the target unit |
| `edge` (`edge_range`) | parameter range and sense of an edge on its carrier curve |
| `assembly` | `MAPPED_ITEM` / `ITEM_DEFINED_TRANSFORMATION` placements flattened into one solid per instance |
| `write` (`write_step`, `export_step`) | the STEP writer (AP214 / AP242), not an op |
| `weld` | `weld_vertices` (sorted grid) and duplicate-edge merge |
| `realize` | `realize<E: TopologyEdit + ?Sized>(&BrepModel, &mut E, &RealizeOptions)`, the pole rule, the orientation self-check |
| `op` | `model.import_step` (`ImportStep`) |
| `query` / `inspect` | `cad.step_info` |
| `inputs` | `InputStore`, `MemoryInputStore`, `DirInputStore`, `StepInputs`, `stage_file`, `sha256_hex`: the only file I/O |
| `source` | the `{sha256}` / `{inline}` parameter, `MAX_INLINE_BYTES = 8 MiB` |

`CadioPlugin` (`id() == "cadio"`) registers the op and the query and no layers; `CoreLayers`
registers `model` and `meshes`. `sm-model` does not register `model.import_step`.

## Content-addressed import

`apply` is a pure function of `(params, state, services)`, so the op never takes a path.

- `source: {"inline": "<base64>"}`: at most 8 MiB decoded. The bytes live in the op record, so
  replay, fork and git are self-contained. The encoded length is checked before decoding.
- `source: {"sha256": "<64 lowercase hex>"}`: the bytes come from the injected
  `StepInputs(Arc<dyn InputStore>)` service. They are re-hashed on read. A missing blob is
  `Invalid("step.input_missing: input blob <sha> not found; stage it first")`, a mismatch is
  `Invalid("step.input_mismatch: ...")`; both are deterministic.
- The inline and `sha256` forms of one file give the same state hash (the layer digests do not
  see the op parameters). The reader is versioned through `IMPL_VERSION`; the service is not.
- Front ends stage a path into `<project>/inputs/<sha256>.stp` (`DirInputStore::put`,
  `stage_file`) and record `{sha256}`. Providing the service in `sm-server` is integration item D6.

## Part 21 parser

Hand-written, byte oriented, no allocation per token beyond the `Param` tree. Lists are built on
a shared scratch stack and copied with exact capacity. Handles `/* */` comments anywhere between
tokens, multi-line records, forward references, complex entities (including OCC's spaced
`#426 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(...) );`), typed parameters
(`LENGTH_MEASURE(25.4)`), `''`, `\X2\..\X0\`, `\X4\..\X0\`, `\X\HH`, `\\` (and `\S\`, `\P?\`
raw), non-UTF-8 strings as ISO 8859-1, reals such as `1.`, `-.5`, `1.E-3` parsed with
`str::parse::<f64>` (correctly rounded), enumerations, `$`, `*`, binary literals.
Limits: list nesting 256 (`TooDeep`, no stack overflow on 100 000 parentheses), input 2 GiB
(`TooLarge`), one `DATA` section. Every syntax error carries `line:col` (1-based, bytes).

## Supported entities

| Group | Entities |
|---|---|
| Solids | `MANIFOLD_SOLID_BREP`, `BREP_WITH_VOIDS` (voids as `ORIENTED_CLOSED_SHELL` or `CLOSED_SHELL`), simple or complex spelling; `FACETED_BREP` (`FACE` / `POLY_LOOP`, planes and straight edges are synthesized) |
| Sheets (`sheets: true`) | `SHELL_BASED_SURFACE_MODEL` over `OPEN_SHELL` / `CLOSED_SHELL`: one body of free faces, no volume, listed after the solids |
| Topology | `CLOSED_SHELL`, `ADVANCED_FACE`, `FACE_SURFACE`, `FACE_BOUND`, `FACE_OUTER_BOUND`, `EDGE_LOOP`, `VERTEX_LOOP` (a sphere pole, stretch S1), `ORIENTED_EDGE`, `EDGE_CURVE`, `VERTEX_POINT` |
| Curves | `LINE` (the `VECTOR` magnitude is ignored: our line is unit speed), `CIRCLE`, `ELLIPSE`, `B_SPLINE_CURVE_WITH_KNOTS`, rational complex form (`RATIONAL_B_SPLINE_CURVE`); `SURFACE_CURVE`, `SEAM_CURVE`, `INTERSECTION_CURVE` resolve to `curve_3d` and their pcurves are ignored; `TRIMMED_CURVE` is its basis curve (an edge's range comes from its vertices) |
| Surfaces | `PLANE`, `CYLINDRICAL_SURFACE`, `CONICAL_SURFACE` (semi-angle converted from the context's plane-angle unit, must be in (0, pi/2)), `SPHERICAL_SURFACE`, `TOROIDAL_SURFACE` (major > minor), `B_SPLINE_SURFACE_WITH_KNOTS` and the rational complex form (multiplicity + distinct knot lists are expanded; the control net is `u`-major, `iu * nv + iv`) |
| Units | `SI_UNIT` with a prefix, `CONVERSION_BASED_UNIT` (inch and others through `LENGTH_MEASURE_WITH_UNIT`), `UNCERTAINTY_MEASURE_WITH_UNIT`; units are per shape-representation context, so each solid has its own scale |
| Skipped, counted | sheets when `sheets` is off, colors, styles, layers, tessellation, PMI |
| `Unsupported{id, entity}` | everything else geometric (`OFFSET_SURFACE`, `SURFACE_OF_REVOLUTION`, `DEGENERATE_TOROIDAL_SURFACE`, `COMPOSITE_CURVE`, `TESSELLATED_SOLID`, ...) |

## Reading rules

- Every list of the `BrepModel` is in ascending STEP id order. Solids are self-contained: the
  reader never merges cells across solids (that is `model.merge`'s job).
- `ADVANCED_FACE.same_sense` is the surface carrier sense; `ORIENTED_EDGE.orientation` XOR
  `FACE_BOUND.orientation` is the coedge sense (and `.F.` reverses the order of the loop);
  `EDGE_CURVE.same_sense` is the curve carrier sense. A reversed `ORIENTED_CLOSED_SHELL` flips
  its faces. The outer bound is `FACE_OUTER_BOUND` (else the first).
- `edge_range`: parameters from `ParametricCurve::project`. A closed circle/ellipse edge gets
  `[t0, t0 + 2 pi]` (a vertex at angle `-2e-16`, as OCC writes it, is snapped to `t0 = 0`). A
  closed B-spline edge needs its vertex at the curve start, else `Unsupported`. A vertex farther
  than `max(weld_tol, 10 x file uncertainty)` from its curve is `GeometryMismatch`; a smaller gap
  above `weld_tol` is warning `step.edge_gap`. A non-periodic edge whose `same_sense` contradicts
  its vertices takes the sense from the geometry (warning `step.edge_sense`).
- Weld: vertices within `weld_tol` merge (lowest STEP id wins) through a sorted grid in a
  `BTreeMap`, O(n log n), never in hash order. Duplicate `EDGE_CURVE`s with the same welded ends
  and five sampled curve points within tolerance merge, recording the relative direction.
- Shell closure: every edge must cancel (forward uses minus reverse uses) in every shell, else
  `OpenShell { solid, issues }` names the open edges (mixed-orientation shells are reported the
  same way, never repaired).
- Faceted solids: vertices are the welded points; one straight edge per unordered vertex pair
  (id of the `POLY_LOOP` that first used it, so these lists are not ascending by STEP id); each
  face gets a plane through its outer loop (Newell normal). Inward loops are repaired by the
  orientation check like any inverted solid.

## Assemblies (stretch S3)

A file with `MAPPED_ITEM` or `NEXT_ASSEMBLY_USAGE_OCCURRENCE` is flattened by default
(`ReadOptions::flatten_assemblies`; the op always flattens). A *link* places a child
representation in a parent one: either a `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION`
(simple or complex) whose `ITEM_DEFINED_TRANSFORMATION(item1, item2)` maps the child's placement
`item1` onto the parent's `item2` (`T = F2 o F1^-1`, `rep_1` the child), or a `MAPPED_ITEM` with
its `REPRESENTATION_MAP`. A solid has one instance per path from its representation up to a root
representation; each instance is a copy of the solid with the composed transform applied through
`CurveDef::transformed` / `SurfaceDef::transformed` (so a solid placed twice becomes two
`Solid`s with `instance` 0 and 1). Both frames are read in the target unit, so a translation
written in a parent's unit is converted. Identity placements are snapped to exact identity and
leave the geometry untouched (the milestone part reads bit for bit as without flattening).
Mirrored placements, other transformation operators and unplaceable `MAPPED_ITEM`s keep the
solids in local coordinates and raise `step.assembly_transforms_ignored`; flattening raises the
info `step.assembly_flattened`.

## The writer (stretch S2)

`write_step(view, bodies, &WriteOptions, &mut dyn Write) -> WriteReport` and
`export_step(state, ...)` (a session state's `model` layer) write one `MANIFOLD_SOLID_BREP` /
`BREP_WITH_VOIDS` per volume into one `ADVANCED_BREP_SHAPE_REPRESENTATION`. Entity numbers are
assigned in a fixed order (units and product structure, then per solid vertices, edges, faces,
shells and the solid, each ascending by model id); geometry entities are shared inside a solid
and never across solids. Reals are the shortest text that parses back to the same bits, always
with a `.`. Edges carry no range (STEP has none); the writer checks that the stored range ends
at the vertices. A face used on its right side, and every face of a void shell, is written with
reversed loops and `same_sense`; a void shell is an `ORIENTED_CLOSED_SHELL(.F.)` over faces whose
normals point out of the cavity, exactly the encoding the reader flips back. Pole edges are
dropped (the seam pair closes the loop). Losses (`WriteReport::losses`): a face shared by two
volumes is written once per volume, baffles, internal shells, embedded wires, coincidence
records. Carriers: line, circle, ellipse, NURBS (rational complex form) and plane, cylinder,
cone, sphere, torus, NURBS; anything else is `Unsupported` naming the cell. The reader keeps
unit and orthogonal axes and directions bit for bit (no renormalization), so
`write(read(write(X))) == write(X)` byte for byte.

## Realization

Creation order is fixed: vertices (solids in order, ascending id), edges, faces (pole edges are
created with their face), then volumes and bodies, one body per solid. Each distinct STEP
curve/surface gets one handle, added when its first user is created. Faces use `Side::Left`;
void shells are `ShellKind::Void`. `realize` runs inside an open operation on any
`TopologyEdit` host; the op wraps it in one `topo_txn` plus `commit_topology`, so a failure
leaves no state behind.

### The pole rule

A loop where consecutive coedges are the same edge traversed oppositely (a seam pair) and the
vertex between them lies on a `Domain2.singularities` point of the face's surface (sphere pole,
cone apex) gets a `DEGENERATE` closed edge there, with a `PCurve::Line2` pole pcurve
`origin = (u_in, v_pole)`, `dir = (1, 0)`, `range = [0, 2 pi]`. The sweep is
`-arrival * sigma * 2 pi`, where `arrival = sign(v_pole - v)` is the direction the previous edge
approaches the pole and `sigma = +-1` the carrier sense: the sign that gives positive face area.
A positive sweep is a `Forward` coedge (`Left` key), a negative one a `Reversed` coedge (`Right`
key, `origin.u = u_in - 2 pi`). `u_in` is the `u` the face sampler will have reached at that
point: `realize` walks the loop in the store's canonical order with the sampler's nearest-copy
unwrapping, starting from the sampler's first Gauss node, so the branch of `u` always agrees.
Pcurves are set **only** for pole coedges; geometry queries unwrap the other seams by
projection. A `VERTEX_LOOP` on a sphere pole (OCC's sphere) is expanded to the seam pair first:
the antipodal vertex and a meridian circle at `u = 0` are synthesized after the file's cells
and carry the id of the `VERTEX_LOOP`.

### Orientation self-check

Each shell is realized first into a scratch `CellComplex` (one volume per shell) and its
`volume_moments` read. An outer shell with negative volume, or a void with positive volume,
has its loops and carrier senses flipped for the real run, and warning `step.inverted_solid`
names the body. `RealizeOptions::fix_orientation = false` skips the scratch run.

## Diagnostics

Located messages through `ctx.warn` / `ctx.info`; codes are stable.

| Code | Severity | Meaning |
|---|---|---|
| `step.units` | info (warning if undeclared) | detected length and angle unit, uncertainty, conversion applied |
| `step.inverted_solid` | warning | a shell had inward normals and was flipped (body id attached) |
| `step.welded` | info | counts of welded vertices and merged duplicate edges |
| `step.sheets_skipped` | info | sheet entities present and not imported (`sheets` is off) |
| `step.unsupported_entity` | warning | solid-like entities that are not imported (`FACETED_BREP`, ...) |
| `step.validate` | warning | one per `Severity::Warning` validator issue, with the located cell id |
| `step.edge_gap` | warning | a vertex off its edge curve by more than `weld_tol` |
| `step.edge_sense` | warning | `EDGE_CURVE.same_sense` repaired from the geometry |
| `step.assembly_transforms_ignored` | warning | the file has `MAPPED_ITEM` / `NEXT_ASSEMBLY_USAGE_OCCURRENCE`; solids are in local coordinates |
| `step.attributes` | info | counts of colors, styles, layers, tessellation, PMI not imported |

A `Severity::Error` issue found by `check: true` fails the op with `OpError::Invalid`
("step.validate: ..." listing the first 20). Errors map to `OpError::Invalid("step.<code>:
...")`: `step.syntax`, `step.truncated`, `step.too_deep`, `step.too_large`,
`step.missing_entity`, `step.wrong_type`, `step.bad_param`, `step.cycle`, `step.unsupported`,
`step.units`, `step.geometry_mismatch`, `step.open_shell`, `step.topology`, `step.inline`,
plus `step.input_missing`, `step.input_mismatch`, `step.bad_sha256`, `step.no_solids`.
A file with nothing to import is `Invalid("step.no_solids: ...")`.

## Determinism

Nothing iterates a hash container into output. Per-solid decoding and per-curve/surface
decoding run under rayon and are collected by index, the first error by index wins. The
`BrepModel` and the state hash are bit-identical for two runs and for `RAYON_NUM_THREADS=1`
against the default (`tests/determinism.rs`).

## `IMPL_VERSION` policy

`ImportStep::IMPL_VERSION` (recorded in each node as `op:model.import_step`) is bumped whenever
the cells created for the same bytes and params change: a different creation order, a changed
pole or orientation rule, a changed weld or tolerance default, new geometry decoding. Replay of
an older node then reports `Upgraded` instead of a bare `Diverged`. The payload `VERSION` is
bumped only when the parameters change shape. The `StepInputs` service is unversioned: it is a
pure lookup by content hash and cannot change what a hash means.

## Performance (release, record-only)

`cargo bench -p sm-cadio --bench perf` on the build machine (one run, criterion means):

| What | Target | Measured |
|---|---|---|
| parse a generated 55.3 MB file (8347 bricks) | <= 2 s, >= 40 MB/s | 1.19 s, 46.6 MB/s (44.4 MiB/s), one thread |
| import 1000 disjoint bricks, `read_step` only | | 174 ms |
| import 1000 disjoint bricks through the op (`TopoStore`, `check: true`) | <= 3 s | 501 ms |
| same, `check: false` | | 487 ms |
| `weld_vertices`, 1e5 points | <= 1 s | 180 ms |

The import numbers depend on the deferred bulk commit (`topo_txn` clones the host per
transaction); they are recorded, not gated.

## Not done / limits

- Sheets are free faces: no volume, no shell closure check, and the writer does not write
  them (reported as a loss). `TESSELLATED_SOLID` is reported as `step.unsupported_entity`.
- `PCURVE` / `DEFINITIONAL_REPRESENTATION` import, surfaces of revolution / extrusion / offset,
  periodic B-splines, IGES, STL/OBJ, colors, names, PMI, `.stpz`, Part 21 edition 3.
- Reader-created faces carry no pcurves except at poles. A closed, non-periodic NURBS face
  (the OCC loft) therefore cannot be integrated by `volume_moments` / `face_moments`: the
  sampler cannot tell `u = 0` from `u = 1` on the seam (api change request in `W3_REPORT.md`).
- A void shell that is wrongly oriented by the writer is repaired per shell, but a shell that
  mixes orientations is reported as `OpenShell`, never repaired.

See `W3_REPORT.md` for the measured values, deviations and gaps.
