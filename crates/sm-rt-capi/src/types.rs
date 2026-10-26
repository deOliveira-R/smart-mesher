//! `#[repr(C)]` types of the C ABI and their constants. These mirror `include/smrt.h`; a test
//! compares sizes and offsets with the header compiled by `cc`.
//!
//! Ids cross the ABI as `int64_t`: a non-negative value is the dense id, `-1` ([`SMRT_NONE`]) is
//! "none / outside the model" and `-2` ([`SMRT_COMPLEMENT`]) is the implicit complement
//! (a region with the contract's `implicit_complement` flag, or `CellId::COMPLEMENT`). Outputs
//! always spell the implicit-complement region `SMRT_COMPLEMENT`; inputs accept both that and the
//! region's raw id. All structs have only 8-byte members, so there is no hidden padding.

/// ABI version `major * 10000 + minor * 100 + patch`, returned by `smrt_version`.
pub const SMRT_ABI_VERSION: i32 = 10000;
/// Id sentinel: none / outside the model.
pub const SMRT_NONE: i64 = -1;
/// Id sentinel: the implicit complement (region or cell).
pub const SMRT_COMPLEMENT: i64 = -2;
/// Hierarchy levels carried in the C view of a state.
pub const SMRT_MAX_LEVELS: usize = 8;

/// `SmrtStep::end`: the distance limit was reached inside the cell.
pub const SMRT_END_LIMIT: i32 = 0;
/// `SmrtStep::end`: crossed an interior face into a cell of the same region.
pub const SMRT_END_CELL_FACE: i32 = 1;
/// `SmrtStep::end`: crossed a lattice or instance wall.
pub const SMRT_END_SLOT_CHANGE: i32 = 2;
/// `SmrtStep::end`: crossed into another region (a DAGMC surface hit).
pub const SMRT_END_REGION_BOUNDARY: i32 = 3;
/// `SmrtStep::end`: reached a model boundary (see `action`).
pub const SMRT_END_BOUNDARY: i32 = 4;
/// `SmrtStep::end`: the particle is lost (see `lost_*`).
pub const SMRT_END_LOST: i32 = 5;

/// `SmrtStep::action`: no boundary action (not a boundary event).
pub const SMRT_ACT_NONE: i32 = 0;
/// `action`: vacuum response (caller resolves; same value as `SMRT_RESP_VACUUM`).
pub const SMRT_ACT_VACUUM: i32 = 1;
/// `action`: white response.
pub const SMRT_ACT_WHITE: i32 = 2;
/// `action`: albedo response (`SmrtStep::albedo` holds the coefficient).
pub const SMRT_ACT_ALBEDO: i32 = 3;
/// `action`: reflective response.
pub const SMRT_ACT_REFLECTIVE: i32 = 4;
/// `action`: mirror deck, already applied inside the step.
pub const SMRT_ACT_MIRROR: i32 = 10;
/// `action`: translation/rotation wrap deck, already applied inside the step.
pub const SMRT_ACT_WRAP: i32 = 11;

/// `smrt_resolve_boundary` response kinds (the contract's `ResponseKind::code()`).
pub const SMRT_RESP_VACUUM: i32 = 1;
/// White (cosine-law re-emission).
pub const SMRT_RESP_WHITE: i32 = 2;
/// Albedo (needs the `albedo` argument).
pub const SMRT_RESP_ALBEDO: i32 = 3;
/// Specular reflection.
pub const SMRT_RESP_REFLECTIVE: i32 = 4;

/// DAGMC ray orientation: exiting the volume.
pub const SMRT_ORIENT_EXITING: i32 = 1;
/// DAGMC ray orientation: entering the volume.
pub const SMRT_ORIENT_ENTERING: i32 = -1;
/// DAGMC ray orientation: either.
pub const SMRT_ORIENT_ANY: i32 = 0;

/// `SmrtStep::lost_cause`: not lost.
pub const SMRT_LOST_NONE: i32 = 0;

/// One hierarchy level (mirrors the contract's `Level`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SmrtLevel {
    /// Component id.
    pub component: i64,
    /// Placed instance id.
    pub instance: i64,
    /// Lattice index at this level.
    pub lattice: [i32; 3],
    /// Padding (zero).
    pub pad: i32,
}

/// A particle state (mirrors `TrackState`). `levels_total` is the true depth; `levels` holds the
/// first `min(levels_total, 8)` entries, so deeper hierarchies are truncated in the C view.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmrtState {
    /// Position.
    pub pos: [f64; 3],
    /// Unit direction.
    pub dir: [f64; 3],
    /// Tracking cell (`-1` outside the model, `-2` complement cell).
    pub cell: i64,
    /// Region (`-1` none, `-2` implicit complement).
    pub region: i64,
    /// Face crossed to get here (`-1` none).
    pub entered_via: i64,
    /// True hierarchy depth.
    pub levels_total: i32,
    /// Entries valid in `levels`.
    pub levels_len: i32,
    /// Hierarchy path (first 8 levels).
    pub levels: [SmrtLevel; SMRT_MAX_LEVELS],
}

/// One step, peek or fire result.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmrtStep {
    /// Distance travelled (or distance to the hit for a peek).
    pub dist: f64,
    /// Cell traversed.
    pub cell: i64,
    /// Its region.
    pub region: i64,
    /// `SMRT_END_*`.
    pub end: i32,
    /// `SMRT_ACT_*` for boundary events and boundary peeks, else 0.
    pub action: i32,
    /// Face crossed or hit (`-1` none).
    pub face: i64,
    /// Model surface crossed or hit (`-1` none).
    pub surface: i64,
    /// Region entered (`-1` outside the model, `-2` implicit complement).
    pub to_region: i64,
    /// Cell entered (`-1` outside the model).
    pub to_cell: i64,
    /// Albedo coefficient when `action == SMRT_ACT_ALBEDO`.
    pub albedo: f64,
    /// `SMRT_END_LOST` only: 1 no cell, 2 no exit face, 3 inconsistent entry face, 4 non-finite,
    /// 5 step cap, 6 recovery failed (0 otherwise).
    pub lost_cause: i32,
    /// Padding (zero).
    pub pad: i32,
    /// `SMRT_END_LOST` only: last valid cell.
    pub lost_cell: i64,
    /// `SMRT_END_LOST` only: position.
    pub lost_pos: [f64; 3],
    /// `SMRT_END_LOST` only: direction.
    pub lost_dir: [f64; 3],
}

/// Result of `smrt_locate`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmrtLocated {
    /// Cell (`-1` outside, `-2` complement cell).
    pub cell: i64,
    /// Region (`-1` none, `-2` implicit complement).
    pub region: i64,
    /// Face the point lies on, if any (`-1` none).
    pub on_face: i64,
    /// True hierarchy depth.
    pub levels_total: i32,
    /// Entries valid in `levels`.
    pub levels_len: i32,
    /// Hierarchy path (first 8 levels).
    pub levels: [SmrtLevel; SMRT_MAX_LEVELS],
}

/// Result of `smrt_closest`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmrtClosest {
    /// Distance to the closest boundary.
    pub dist: f64,
    /// Surface that owns it.
    pub surface: i64,
    /// Closest point.
    pub point: [f64; 3],
}

/// Build-time facts (`smrt_info`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmrtInfo {
    /// Tracking cells.
    pub cells: i64,
    /// Faces / facets.
    pub faces: i64,
    /// Regions.
    pub regions: i64,
    /// Model surfaces.
    pub surfaces: i64,
    /// Distinct prototypes.
    pub prototypes: i64,
    /// Placed copies.
    pub copies: i64,
    /// Heap bytes of the built structure.
    pub memory_bytes: i64,
    /// Faces tracked exactly.
    pub exact_faces: i64,
    /// Faces tracked through facets.
    pub faceted_faces: i64,
    /// `TrackerCaps` bit set.
    pub caps: i64,
    /// Raw id of the implicit-complement region (`-1` if none); outputs spell it `-2`.
    pub complement_region: i64,
    /// Event cap per fire / trace.
    pub max_events: i64,
    /// Low 64 bits of the structure hash.
    pub structure_hash_lo: u64,
    /// High 64 bits of the structure hash.
    pub structure_hash_hi: u64,
    /// Bounding box minimum.
    pub bounds_lo: [f64; 3],
    /// Bounding box maximum.
    pub bounds_hi: [f64; 3],
    /// Model linear tolerance.
    pub tol_linear: f64,
    /// Model angular tolerance.
    pub tol_angular: f64,
}

/// Per-region facts (`smrt_region_info`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmrtRegionInfo {
    /// Serial of the model entity the region comes from (`-1` if it has none).
    pub original_id: i64,
    /// Colour id; reserved, always `-1` (the contract carries none).
    pub color_id: i64,
    /// Volume: the sum of `cell_volume` over the region's cells in index order; NaN if any cell
    /// volume is unknown.
    pub measure: f64,
    /// 1 for the implicit complement.
    pub implicit_complement: i64,
}

/// Options for `smrt_trace_bundle` (null pointer = defaults).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SmrtTraceOpts {
    /// Maximum distance per ray (`<= 0` or infinite = unlimited).
    pub max_dist: f64,
    /// Step cap per ray (`<= 0` = the contract default).
    pub max_steps: i64,
    /// Non-zero: end the trace at deck boundaries.
    pub stop_at_deck: i64,
}

/// Ray bundle in SoA form (`smrt_trace_bundle`). Every array has `n` entries.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct SmrtRays {
    /// Origin x.
    pub x: *const f64,
    /// Origin y.
    pub y: *const f64,
    /// Origin z.
    pub z: *const f64,
    /// Direction x (unit).
    pub u: *const f64,
    /// Direction y.
    pub v: *const f64,
    /// Direction z.
    pub w: *const f64,
    /// Start cell hint per ray; may be null. Ignored by the contract's `trace_bundle` (each ray
    /// is located from scratch); kept so callers can pass their SoA unchanged.
    pub start_cell: *const i64,
}
