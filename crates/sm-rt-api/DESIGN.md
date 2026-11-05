# sm-rt-api: design notes

This file records where the contract departs from the obvious reading of the upstream contracts
and of the DAGMC/OpenMC shapes it mirrors, so the `sm-rt` and `sm-rt-capi` workers do not
rediscover the reasons. The crate-level rustdoc in `src/lib.rs` is the normative description of
the query surface; this file explains the choices.

## Deviations and design-arounds

* **`ResponseMap` (`BTreeMap<EntityId, ResponseKind>`).** `sm_model_api`'s
  `BoundaryKind::Response` is only a tag: the model says "this boundary responds" and leaves
  the meaning (vacuum, reflective, white, albedo) to the consumer. The deck therefore supplies
  the meaning per boundary entity. A `BTreeMap` keeps iteration, and hence the boundary hash,
  deterministic.
* **`ModelTolerance` comes from `sm_geom_api`.** The tracker defines no tolerance constants of
  its own. `linear` is the on-face distance and the nudge unit, `angular` bounds the
  unit-direction check. Using the model's type keeps one tolerance policy across topology,
  geometry and tracking, and puts it into the options hash.
* **`WorldSpec::from_model` is not added.** Deriving a `WorldSpec` from an
  `sm_model_api` model needs the model's instance tree and the inputs the builder was given;
  that mapping belongs in `sm-rt` (the implementor), which also owns the leaf-to-input
  numbering. The contract only fixes the target shape (`WorldSpec`, `ComponentKind`,
  `Lattice`) and `Tracker::path_of` for the way back to `InstancePath`.
* **`distance_to_boundary` returns `Result<Peek>`.** A peek can fail on a bad state
  (non-unit direction, non-finite position), which is an input error, while a tracking failure
  is a value (`Peek::Lost`). Both channels exist so a C wrapper maps errors to a status code
  and losses to a located record.
* **`bounds`, `cell_count` and `structure_hash` are default methods derived from `info()`.**
  `TrackerInfo` already carries them, so a second required accessor would be a place for the
  two to disagree. Implementors may override for speed, never for a different value.
* **`face_normal(FaceId, Point3)`.** The point makes curved and analytic faces well defined
  and costs planar faces nothing. `compat::get_angle` is a thin wrapper.
* **`resolve_boundary` takes a `ResponseKind`.** The step reports `FaceAction::Respond(kind)`;
  the caller passes that kind back with two uniforms from its own RNG. The tracker never owns
  random state (determinism rule: no hidden seeds), and the albedo is validated at the
  point of use (`RtError::BadAlbedo`).
* **No `LostInfo.retries` yet.** Recovery (`RecoveryPolicy::Nudge`) is specified as a fixed
  rule (attempt `k` moves `tol.linear * 2^k` from the position where the loss was detected,
  see `RecoveryPolicy::nudge_distance`), but the number of attempts used is not yet recorded
  in `LostInfo`. `LostInfo` is `#[non_exhaustive]`, so the field can be added later without
  breaking implementors.
* **No per-region `measure_area` / `measure_volume`.** Only `Tracker::cell_volume` exists
  (exact, `None` when not known). Region measures are sums over cells in the caller
  (`bin_map(TallySpec::Regions)`), which keeps the reduction order explicit and deterministic;
  surface areas need a per-face area table that no consumer has asked for yet.
* **No `serde` feature.** The only persisted form is the `.smrt` cache (`CacheImage`, a
  fixed binary layout with its own version and per-section hashes). A serde derive on the
  public structs would create a second, unversioned wire format that is not covered by the
  cache key. Add it behind a feature if a consumer needs it, as an additive change.

## Growth rules

* Growable structs and enums are `#[non_exhaustive]`: `TraceOptions`, `Segment`, `Step`,
  `FireResult`, `BoundaryHit`, `Closest`, `RegionInfo`, `SurfaceInfo`, `EndCode`, and the
  option/enum types that already were. Downstream crates construct them through `new` /
  `Default` / `with_*` builders; matching on the enums needs a wildcard arm.
* `EndCode` is `#[repr(u8)]` with frozen discriminants (`EndCode::from_u8` decodes them; `Other = 9` is any end without its own code).
  `RtError::code()` values are frozen and a new variant gets the next free negative code
  (`BinCountMismatch` is `-10`, `InvalidInput` `-11`).
* `SectionTag::Private(u16)` maps to wire codes `0x1000..=0x10FFF`, so the code cannot
  overflow; other codes above the public range are rejected as corrupt.
* The crate is `#![forbid(unsafe_code)]`. The test allocator in `tests/alloc.rs` is the only
  unsafe code, with `// SAFETY:` comments.

## Stub conventions worth knowing

* The stub's "background" is a real cell per element, numbered after the shape cells. Its region
  carries `implicit_complement`, but `locate` never returns `CellId::COMPLEMENT`; that sentinel
  is for trackers whose complement has no cells.
* Stub shapes must be pairwise nested or disjoint (partial overlap is an `RtError::Build` error with issue `analytic.overlap`, like `csg.overlap`/`world.overlap`); a point belongs to the innermost shape containing it, and order carries no priority. `AnalyticWorld::cell_volume` is exact unless a shape leaves the element box.
* `ray_fire` with an origin outside the model is `RayFire::Miss` (DAGMC returns no hit);
  `Lost` is only for failures after a valid start.

## W3b extension: segment iterator, majorants, CSG leaf (rewritten at W3b-0)

`segments.rs`: `FlightCursor` (opaque, resumable, no borrow; `Flight` is a thin wrapper), `FlightInput { majorants, block }` passed on every call (one majorant per prototype-tree `NodeId`; the tracker holds no cross section and no copy), `check_collision` against the cached node majorant (`RtError::MajorantViolated`, code -12; `InvalidMajorant` shares -11), `MajorantTracker`/`HierarchyView` (disjoint partition, deduplicated `MaterialSets`), `WorkCounters` in `Scratch`. `ModePolicy` is `Surface | Delta`; hybrid is caller-driven (`set_policy`, `block`). The energy-band axis, `MajorantTable` and `FlightStats` are gone. `PartitionOptions` is hashed into `BuildOptions` only when non-default; `BuildOptions` stays `Copy`. `csg.rs`: `CsgRegion`, `CsgCellDef`, `CsgLeafInput` (cells must be disjoint; overlap is `csg.overlap`/`world.overlap` at build, `LostCause::Overlap` at track time). `BuildInput::Hier(HierInput)` with `LeafInput::{Mesh, Facets, Csg}`; `TrackerBuilder::load_image`; `Tracker::{surface_area, surface_faces}`; `RegionInfo::tags`; `LostCause::{SenseMismatch, Overlap}`, `LostInfo::retries`. Tests: `tests/segments.rs`, `tests/freeze.rs`.
