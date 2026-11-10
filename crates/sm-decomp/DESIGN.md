# sm-decomp

Cut planner for round cavities (W4 task HEX-DECOMP; `docs/design/w4/drafts/hexpin.md` D2). It is a pure planner: given a volume that no hex scheme accepts, it returns an ordered `CutStep` list, each step holding the exact arguments of an existing op (`model.webcut_cylinder`, `model.webcut_plane`), with a cylinder plus a plane at every cavity end face. It is a library with no plugin and no registered ops, and the O-grid scheme does not consume it. The query `mesh.decompose_plan` (W4-HEX-QUERY, `sm-mesher`) and the `hex_only` error text are its consumers.

## API

- `plan_cuts(&dyn TopologyView, volume) -> Result<Vec<CutStep>, DecompError>`: reads the volume, then calls the core.
- `volume_view` / `VolumeView` / `FaceView` / `FaceKind`: the plain-data view (faces with surface class, vertex positions, area). `plan_cuts_view(&VolumeView, tol)` is the pure core; tests and non-topology callers use it.
- `CutStep { step, cut: CutKind, targets: Vec<Target> }`; `Target` is `Id {id}` or `Step {step, role}`; `Role` is `positive` or `negative`, the `WEBCUT_OUTPUTS` roles of `sm-boolean`. All types serialize (`serde`) to the op argument names: `{"step":1,"op":"model.webcut_plane","plane":{"origin":..,"normal":..},"targets":[{"step":0,"role":"positive"}]}`.
- `render_script(steps, root, prefix)`: the same plan as command-language lines (`let cut0 = model.webcut_cylinder(...)`, later steps use `cut0.positive`).

## What it plans

Axis-aligned version: all cavity walls share one axis direction `d` (sign fixed: first nonzero component positive). A cavity wall is a cylinder face whose outward volume normal points to the axis; it must be a whole cylinder (area check), so slots are refused.

1. Group the walls into cavities (same centre and radius). Disjoint footprints only: overlapping or nested cavities (a stepped bore) are refused.
2. Every enclosed cavity (closed at both ends inside the host along `d`) gets one `webcut_cylinder` of its radius about its axis. A blind cavity (open at one end) gets its cylinder after its end plane, on the solid piece beyond the cavity (`positive` of the plane when the cavity is open at the low end, `negative` when open at the high end). The kernel refuses a cylinder cut that does not separate a volume ("odd number of loops"), so the cylinder cannot go on the host; and the plane alone leaves a solid slab whose imprinted face has a hole, which does not sweep (both found by the real-model test). The target is the volume, then the `positive` output (outside) of the previous step.
3. A `webcut_plane` normal to `d` at every distinct axial coordinate where a cavity wall face starts or ends strictly inside the host, ascending, each cutting the `positive` output of the step before. This covers the cavity end faces (the draft's critique delta: a cylinder alone leaves columns of different length) and the splits that an imprint with stacked pellets leaves in the wall.
4. A cavity that already runs through the host with a one-piece wall needs nothing, so a volume that sweeps as is returns an empty list. A volume with no cavity wall returns an empty list as well.

Refused with a reason (`DecompError::NoDecomposition`, never a guess): different cavity axes, overlapping/nested cavities, a wall that is not a whole cylinder, a face that is not a plane or cylinder, a plane neither parallel nor normal to the axis, a cylinder not parallel to the axis, a plane inside the host that closes no cavity.

## Determinism

Walls are sorted by (centre, radius, extent) before grouping and end coordinates are sorted and deduplicated within the tolerance, so the plan does not depend on face order (tested by rotating and reversing the face list). No hash maps, no parallelism, no randomness. Coordinates are copied from the model (`-0.0` is normalised to `0.0`). Plan text and JSON are byte-stable.

## Spike record (re-run at `1a76bc2`)

Enclosed pin (brick 1.26 x 1.26 x 1.0, fuel and cavity r 0.4, h 0.8):
- cylinder cut alone: 4 volumes; `mesh.auto default` then falls back to tets (692 tets, 3 `mesh.validate` violations). The draft's "fails outright" does not reproduce at this commit.
- cylinder plus planes at z = -0.4 and 0.4: 6 volumes, `{Hex: 80}`, 0 violations, mesh volume 1.5876000000000015, moderator volume 1.1854761403405059.

## Not done

True off-axis (non-coordinate) bores are supported only when every cavity shares one direction; cross holes, stepped bores and slots are refused. The planner does not call sweep, so "already sweeps" means "needs no cut of the kinds above".
