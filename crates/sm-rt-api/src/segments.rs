//! The segment iterator: one flight type, two modes (surface and delta tracking).
//!
//! # Thesis
//! A flight is a sequence of *segments* `{t0, t1, node, cell, sigma_M}`. In **surface** mode a
//! segment is one cell (or one region run) and `sigma_M` is that region's total cross section
//! (its singleton material set), so a transport code never rejects anything. In **delta**
//! mode a segment is the chord through one leaf of a *disjoint* majorant partition, with the
//! leaf's local majorant. The transport code samples inside the segment and asks the flight to
//! locate the tentative collision ([`Flight::probe`]) starting from the node that owns the
//! majorant, never from the root. Switching mode between segments is unbiased, so a hybrid
//! policy is a per-segment choice ([`ModePolicy::Auto`]).
//!
//! Everything else hangs off the segment: [`MajorantTable`] (keyed by material set and energy
//! band), [`FlightTally`] (track-length scores on surface segments, collision scores on delta
//! segments) and [`FlightStats`] (work counters for benchmarks; no wall clock).
//!
//! # Driving a flight (Monte Carlo)
//! ```text
//! let mut fl = Flight::new(&tracker, &mut st, &mut scratch, &table, opts)?;
//! 'flight: while let Some(seg) = fl.next_segment()? {
//!     let mut t = seg.t0;
//!     loop {
//!         t += -ln(xi()) / seg.sigma_m;                 // sigma_m = 0: never collides
//!         if !(t < seg.t1) { break; }                   // flew the whole segment
//!         let loc = fl.probe(t)?;                       // hinted, no root restart
//!         let sigma_t = xs(loc.region, energy);
//!         table.check(seg.set, band, sigma_t)?;         // violation is an error
//!         if xi() * seg.sigma_m < sigma_t {             // real (always true in surface mode)
//!             fl.stop_at(t)?;                           // state now at the collision
//!             break 'flight;                            // ... sample reaction, new direction
//!         }                                             // else virtual: keep sampling in seg
//!     }
//! }
//! let end = fl.end();    // Boundary / Limit / LeftModel / Stopped (collision) / Lost
//! ```
//! In surface mode the same loop works unchanged: `sigma_M == sigma_t`, so every tentative
//! collision is real. A surface-only code can skip the probe, because `seg.cell` is the cell
//! (`seg.resolved`). Sn/MOC and mesh tallies use
//! [`SegmentScale::Cell`] with [`ModePolicy::Surface`] and score every segment.
//!
//! # Contract for implementors ([`MajorantTracker`])
//! * Leaves of [`HierarchyView`] **tile** the model disjointly ([`validate_partition`] checks
//!   it). A BVH has overlapping boxes and is not a partition; it may serve `locate`, not this.
//! * `advance_segment` moves the state to the end of the segment, applying events exactly as
//!   `Tracker::step` does (lattice walls are `StepEnd::SlotChange`, reflect/wrap are applied,
//!   `Respond` boundaries are not). Segments of one flight are contiguous and their lengths
//!   sum to the chord, whatever the mode.
//! * Mode choice is a pure function of `(node, band, policy)` ([`choose_mode`]); results are
//!   bit-identical for any thread count.
//! * `locate_in_node` starts from the node: O(1) lattice index, ring search, walk inside a
//!   meshed leaf. It must not restart from the root.

use crate::error::{BuildIssue, LostCause, Result, RtError};
use crate::ids::{CellId, FaceId, MaterialSetId, NodeId, RegionId};
use crate::state::{Located, Scratch, StepEnd, TrackState};
use crate::tally::{BinMap, TraceEnd};
use crate::tracker::{Tracker, lost};
use sm_core::{Aabb, Point3, Vector3};

// ---------------------------------------------------------------------------------------------
// Hierarchy
// ---------------------------------------------------------------------------------------------

/// What a node of the majorant partition is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum NodeKind {
    /// An internal node: a lattice, a universe, a kd/octree split. Never yields segments.
    Internal,
    /// A leaf holding analytic primitives.
    Analytic,
    /// A leaf holding mesh cells.
    Mesh,
    /// A leaf of one region (no inner structure; one cell).
    Homogeneous,
}

/// One node of the disjoint partition.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct NodeInfo {
    /// The node.
    pub id: NodeId,
    /// Its parent (`NodeId::NONE` for the root).
    pub parent: NodeId,
    /// World-frame bounds (exact for lattice slots; conservative boxes are allowed for
    /// leaves, but leaves must still be disjoint as *regions of space*).
    pub bounds: Aabb,
    /// Kind of node.
    pub kind: NodeKind,
    /// The deduplicated set of regions occurring inside.
    pub set: MaterialSetId,
    /// Number of tracking cells inside (a delta segment over one cell gains nothing).
    pub cells: u32,
}

impl NodeInfo {
    /// Construct (the type is `#[non_exhaustive]`).
    pub const fn new(id: NodeId, parent: NodeId, bounds: Aabb, kind: NodeKind, set: MaterialSetId, cells: u32) -> Self {
        Self { id, parent, bounds, kind, set, cells }
    }
    /// True for nodes that yield segments.
    pub fn is_leaf(&self) -> bool {
        !matches!(self.kind, NodeKind::Internal)
    }
}

/// Read-only view of the disjoint majorant partition and the material sets.
pub trait HierarchyView: Send + Sync {
    /// Number of nodes (ids are dense `0..node_count`).
    fn node_count(&self) -> usize;
    /// The root node.
    fn root(&self) -> NodeId;
    /// A node, or `None` for an unknown id.
    fn node(&self, n: NodeId) -> Option<NodeInfo>;
    /// The leaf owning a cell.
    fn node_of_cell(&self, c: CellId) -> Option<NodeId>;
    /// Number of material sets (ids are dense `0..set_count`).
    fn set_count(&self) -> usize;
    /// Regions of a set, ascending and deduplicated.
    fn set_regions(&self, s: MaterialSetId) -> Option<&[RegionId]>;
    /// The singleton set `{r}`: every region has one.
    fn region_set(&self, r: RegionId) -> Option<MaterialSetId>;
}

/// Check that the leaves tile the root: pairwise interior-disjoint boxes whose volumes sum to
/// the root's volume (relative `1e-9`). `O(L^2)`; a build-time or test check.
pub fn validate_partition(h: &dyn HierarchyView) -> Result<()> {
    let vol = |b: &Aabb| (b.hi.x - b.lo.x).max(0.0) * (b.hi.y - b.lo.y).max(0.0) * (b.hi.z - b.lo.z).max(0.0);
    let overlap = |a: &Aabb, b: &Aabb| {
        let d = |l0: f64, h0: f64, l1: f64, h1: f64| (h0.min(h1) - l0.max(l1)).max(0.0);
        d(a.lo.x, a.hi.x, b.lo.x, b.hi.x) * d(a.lo.y, a.hi.y, b.lo.y, b.hi.y) * d(a.lo.z, a.hi.z, b.lo.z, b.hi.z)
    };
    let mut issues = Vec::new();
    let Some(root) = h.node(h.root()) else {
        return Err(RtError::Build(vec![BuildIssue::new("hierarchy.root", "root node is missing")]));
    };
    let leaves: Vec<NodeInfo> = (0..h.node_count()).filter_map(|i| h.node(NodeId::new(i))).filter(NodeInfo::is_leaf).collect();
    let total: f64 = leaves.iter().map(|l| vol(&l.bounds)).sum();
    if (total - vol(&root.bounds)).abs() > 1e-9 * vol(&root.bounds).max(f64::MIN_POSITIVE) {
        issues.push(BuildIssue::new("hierarchy.cover", format!("leaf volumes sum to {total}, root volume is {}", vol(&root.bounds))));
    }
    for (i, a) in leaves.iter().enumerate() {
        for b in &leaves[i + 1..] {
            let o = overlap(&a.bounds, &b.bounds);
            if o > 1e-12 * vol(&a.bounds).max(vol(&b.bounds)) {
                issues.push(BuildIssue::new("hierarchy.overlap", format!("leaves {} and {} overlap (volume {o})", a.id.0, b.id.0)));
            }
        }
    }
    if issues.is_empty() { Ok(()) } else { Err(RtError::Build(issues)) }
}

// ---------------------------------------------------------------------------------------------
// Energy bands and the majorant table
// ---------------------------------------------------------------------------------------------

/// Partition of the energy axis into bands; the table's second key.
#[derive(Clone, Debug, PartialEq)]
pub struct EnergyBands {
    edges: Vec<f64>,
}

impl EnergyBands {
    /// One band for all energies (multigroup codes use their group index as the band instead).
    pub fn single() -> Self {
        Self { edges: Vec::new() }
    }
    /// `edges.len() + 1` bands: band `k` holds `edges[k-1] <= e < edges[k]`. Edges must be
    /// finite and strictly increasing.
    pub fn new(edges: Vec<f64>) -> Result<Self> {
        if !edges.iter().all(|e| e.is_finite()) || edges.windows(2).any(|w| w[0] >= w[1]) {
            return Err(RtError::InvalidInput("energy band edges must be finite and strictly increasing"));
        }
        Ok(Self { edges })
    }
    /// Number of bands.
    pub fn n_bands(&self) -> usize {
        self.edges.len() + 1
    }
    /// Band of an energy.
    pub fn band_of(&self, e: f64) -> Result<u32> {
        if e.is_nan() {
            return Err(RtError::NonFinite("energy"));
        }
        Ok(self.edges.partition_point(|&x| x <= e) as u32)
    }
}

/// One table entry.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct MajorantEntry {
    /// `sigma_M`: an upper bound of the total cross section of every region in the set.
    pub majorant: f64,
    /// Volume-weighted mean total cross section over the set (the build-time cost model:
    /// efficiency is `mean_total / majorant`, the fraction of real collisions in delta mode).
    pub mean_total: f64,
}

impl MajorantEntry {
    /// Construct (the type is `#[non_exhaustive]`).
    pub const fn new(majorant: f64, mean_total: f64) -> Self {
        Self { majorant, mean_total }
    }
}

/// Majorants keyed by `(material set, energy band)`, built by the caller from its cross
/// sections over the tracker's sets.
#[derive(Clone, Debug, PartialEq)]
pub struct MajorantTable {
    bands: EnergyBands,
    n_sets: usize,
    entries: Vec<MajorantEntry>,
}

/// Relative slack for rounding when comparing a cross section to its majorant.
const MAJORANT_SLACK: f64 = 1e-12;

impl MajorantTable {
    /// Build by calling `f(set, regions, band)` for every set (singletons included) and band, in
    /// ascending set then band order. Entries must be finite, `majorant >= 0` and
    /// `0 <= mean_total <= majorant`.
    pub fn from_fn(
        h: &dyn HierarchyView,
        bands: EnergyBands,
        mut f: impl FnMut(MaterialSetId, &[RegionId], u32) -> MajorantEntry,
    ) -> Result<Self> {
        let (n_sets, nb) = (h.set_count(), bands.n_bands());
        let mut entries = Vec::with_capacity(n_sets * nb);
        for s in 0..n_sets {
            let id = MaterialSetId::new(s);
            let regions = h.set_regions(id).ok_or(RtError::UnknownId { kind: "material set", id: id.0 })?;
            for b in 0..nb {
                let e = f(id, regions, b as u32);
                let ok = e.majorant.is_finite()
                    && e.majorant >= 0.0
                    && e.mean_total.is_finite()
                    && e.mean_total >= 0.0
                    && e.mean_total <= e.majorant * (1.0 + MAJORANT_SLACK);
                if !ok {
                    return Err(RtError::InvalidInput("majorant entries need 0 <= mean_total <= majorant, both finite"));
                }
                entries.push(e);
            }
        }
        Ok(Self { bands, n_sets, entries })
    }
    /// The energy bands.
    pub fn bands(&self) -> &EnergyBands {
        &self.bands
    }
    /// Number of bands.
    pub fn n_bands(&self) -> usize {
        self.bands.n_bands()
    }
    /// Number of material sets.
    pub fn n_sets(&self) -> usize {
        self.n_sets
    }
    /// The entry for `(set, band)`.
    pub fn entry(&self, set: MaterialSetId, band: u32) -> Option<MajorantEntry> {
        let (s, b) = (set.get(), band as usize);
        (s < self.n_sets && b < self.n_bands()).then(|| self.entries[s * self.n_bands() + b])
    }
    /// `sigma_M` for `(set, band)`.
    pub fn majorant(&self, set: MaterialSetId, band: u32) -> Option<f64> {
        self.entry(set, band).map(|e| e.majorant)
    }
    /// Real-collision fraction `mean_total / majorant` (1 for a void set).
    pub fn efficiency(&self, set: MaterialSetId, band: u32) -> Option<f64> {
        self.entry(set, band).map(|e| if e.majorant > 0.0 { e.mean_total / e.majorant } else { 1.0 })
    }
    /// Error with [`RtError::MajorantViolated`] if `sigma_t` exceeds the majorant (beyond
    /// `1e-12` relative rounding). Never clamps.
    pub fn check(&self, set: MaterialSetId, band: u32, sigma_t: f64) -> Result<()> {
        let e = self.entry(set, band).ok_or(RtError::UnknownId { kind: "material set", id: set.0 })?;
        if !sigma_t.is_finite() || sigma_t < 0.0 {
            return Err(RtError::NonFinite("sigma_t"));
        }
        if sigma_t > e.majorant * (1.0 + MAJORANT_SLACK) {
            return Err(RtError::MajorantViolated { set: set.0, band, sigma_t, majorant: e.majorant });
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------------------------
// Segments
// ---------------------------------------------------------------------------------------------

/// How a segment was produced.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackMode {
    /// One cell (or region run); `sigma_M` is the region's own total, nothing is rejected.
    Surface,
    /// The chord through one leaf with the leaf's majorant.
    Delta,
}

/// Per-segment mode choice.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum ModePolicy {
    /// Always surface segments.
    Surface,
    /// Always delta segments (leaves of one cell still yield one cell-wide segment).
    Delta,
    /// Delta in a leaf with more than one cell whose real-collision fraction
    /// ([`MajorantTable::efficiency`]) is at least `min_efficiency` (Serpent's rejection test,
    /// `0.9`), else surface. Pure in `(node, band)`.
    Auto {
        /// Threshold in `(0, 1]`.
        min_efficiency: f64,
    },
}

/// Granularity of surface segments.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum SegmentScale {
    /// One segment per cell: Sn/MOC characteristics and mesh tallies.
    #[default]
    Cell,
    /// One segment per maximal run inside one region (no per-cell structure; may use the
    /// exact per-region boundary structure when [`crate::TrackerCaps::REGION_JUMP`] is set).
    Region,
}

/// Shared mode rule for implementors: what `policy` means for a leaf.
pub fn choose_mode(policy: ModePolicy, table: &MajorantTable, node_set: MaterialSetId, band: u32, cells_in_node: usize) -> TrackMode {
    match policy {
        ModePolicy::Surface => TrackMode::Surface,
        ModePolicy::Delta => TrackMode::Delta,
        ModePolicy::Auto { min_efficiency } => {
            if cells_in_node > 1 && table.efficiency(node_set, band).is_some_and(|e| e >= min_efficiency) {
                TrackMode::Delta
            } else {
                TrackMode::Surface
            }
        }
    }
}

/// One segment of a flight.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct FlightSegment {
    /// How it was produced.
    pub mode: TrackMode,
    /// Flight parameter at the start (distance from the flight's origin).
    pub t0: f64,
    /// Flight parameter at the end.
    pub t1: f64,
    /// The leaf node.
    pub node: NodeId,
    /// The material set whose majorant is `sigma_m` (a region singleton in surface mode).
    pub set: MaterialSetId,
    /// The cell at `t0`.
    pub cell: CellId,
    /// The region at `t0`.
    pub region: RegionId,
    /// True if the whole segment lies in `cell` (always true for surface segments of
    /// [`SegmentScale::Cell`]); false for delta and region-run segments.
    pub resolved: bool,
    /// `sigma_M` for the flight's band.
    pub sigma_m: f64,
    /// Face crossed to enter the segment, if any.
    pub face_in: Option<FaceId>,
    /// Face crossed to leave it, if any.
    pub face_out: Option<FaceId>,
    /// The event that ended the segment (as `Tracker::step` reports it).
    pub end: StepEnd,
}

impl FlightSegment {
    /// What an implementor returns from `advance_segment`: relative to the state's position
    /// (`t0 = 0`, `t1 = length`); [`Flight`] shifts it to flight coordinates.
    pub fn new(mode: TrackMode, node: NodeId, set: MaterialSetId, cell: CellId, region: RegionId, length: f64, sigma_m: f64) -> Self {
        Self {
            mode,
            t0: 0.0,
            t1: length,
            node,
            set,
            cell,
            region,
            resolved: mode == TrackMode::Surface,
            sigma_m,
            face_in: None,
            face_out: None,
            end: StepEnd::Limit,
        }
    }
    /// Set the entry and exit faces.
    pub fn with_faces(mut self, face_in: Option<FaceId>, face_out: Option<FaceId>) -> Self {
        self.face_in = face_in;
        self.face_out = face_out;
        self
    }
    /// Set the ending event.
    pub fn with_end(mut self, end: StepEnd) -> Self {
        self.end = end;
        self
    }
    /// Set whether the segment lies in a single cell.
    pub fn with_resolved(mut self, resolved: bool) -> Self {
        self.resolved = resolved;
        self
    }
    /// `t1 - t0`.
    pub fn length(&self) -> f64 {
        self.t1 - self.t0
    }
}

/// What a flight asks of a tracker for the next segment.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub struct SegmentQuery<'a> {
    /// Mode policy.
    pub policy: ModePolicy,
    /// Surface-segment granularity.
    pub scale: SegmentScale,
    /// Energy band into `table`.
    pub band: u32,
    /// The majorant table.
    pub table: &'a MajorantTable,
    /// Remaining distance (the segment ends in `StepEnd::Limit` if it gets there).
    pub max_dist: f64,
}

impl<'a> SegmentQuery<'a> {
    /// Construct (the type is `#[non_exhaustive]`).
    pub fn new(policy: ModePolicy, scale: SegmentScale, band: u32, table: &'a MajorantTable, max_dist: f64) -> Self {
        Self { policy, scale, band, table, max_dist }
    }
}

/// The segment-producing capability of a tracker (`Tracker::majorant_view`).
pub trait MajorantTracker: Tracker {
    /// The disjoint partition and the material sets.
    fn hierarchy(&self) -> &dyn HierarchyView;

    /// Move `st` to the end of the next segment and describe it. Obeys the module contract:
    /// events as `Tracker::step`, contiguous, lengths sum to the chord, no allocation after
    /// warm-up, mode via [`choose_mode`]. On `StepEnd::Lost` the state is unchanged.
    fn advance_segment(&self, st: &mut TrackState, q: &SegmentQuery<'_>, scratch: &mut Scratch) -> Result<FlightSegment>;

    /// Locate `p` starting from `node` (never from the root). `dir` breaks ties on faces.
    fn locate_in_node(&self, node: NodeId, p: Point3, dir: Option<Vector3>, scratch: &mut Scratch) -> Result<Located>;

    /// A lower bound of the distance from the state to any cell face (an exact one for convex
    /// cells): a sampled flight shorter than this needs no intersection test. `0.0` is always
    /// valid and is the default.
    fn safety_distance(&self, _st: &TrackState, _scratch: &mut Scratch) -> Result<f64> {
        Ok(0.0)
    }
}

// ---------------------------------------------------------------------------------------------
// The flight
// ---------------------------------------------------------------------------------------------

/// Options of a flight.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct FlightOptions {
    /// Stop after this flight parameter (`TraceEnd::Limit`).
    pub max_dist: f64,
    /// Segment budget (`TraceEnd::Lost(MaxStepsExceeded)` beyond it).
    pub max_steps: u32,
    /// Return every `Respond`/`Reflect`/`Wrap` boundary instead of continuing through applied ones.
    pub stop_at_deck: bool,
    /// Mode policy (changeable between segments with [`Flight::set_policy`]).
    pub policy: ModePolicy,
    /// Granularity of surface segments.
    pub scale: SegmentScale,
    /// Energy band into the table.
    pub band: u32,
}

impl Default for FlightOptions {
    fn default() -> Self {
        Self { max_dist: f64::INFINITY, max_steps: 1_000_000, stop_at_deck: false, policy: ModePolicy::Surface, scale: SegmentScale::Cell, band: 0 }
    }
}

impl FlightOptions {
    /// Set the distance limit.
    pub fn with_max_dist(mut self, d: f64) -> Self {
        self.max_dist = d;
        self
    }
    /// Set the segment budget.
    pub fn with_max_steps(mut self, n: u32) -> Self {
        self.max_steps = n;
        self
    }
    /// Set `stop_at_deck`.
    pub fn with_stop_at_deck(mut self, stop: bool) -> Self {
        self.stop_at_deck = stop;
        self
    }
    /// Set the mode policy.
    pub fn with_policy(mut self, p: ModePolicy) -> Self {
        self.policy = p;
        self
    }
    /// Set the surface-segment granularity.
    pub fn with_scale(mut self, s: SegmentScale) -> Self {
        self.scale = s;
        self
    }
    /// Set the energy band.
    pub fn with_band(mut self, b: u32) -> Self {
        self.band = b;
        self
    }
}

/// Work counters of a flight (deterministic; benchmark with these, not clocks).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct FlightStats {
    /// Surface segments produced.
    pub surface_segments: u64,
    /// Delta segments produced.
    pub delta_segments: u64,
    /// Hinted point locations ([`Flight::probe`], [`Flight::stop_at`]).
    pub probes: u64,
}

impl FlightStats {
    /// Add another flight's counters.
    pub fn merge(&mut self, o: &Self) {
        self.surface_segments += o.surface_segments;
        self.delta_segments += o.delta_segments;
        self.probes += o.probes;
    }
}

/// A flight: the segment iterator over one particle track.
///
/// It is a lending iterator ([`Flight::next_segment`]) because it mutates the borrowed
/// state. After a segment is returned the state sits at the segment's end; [`Flight::probe`]
/// and [`Flight::stop_at`] address points inside the *last* segment.
pub struct Flight<'a> {
    view: &'a dyn MajorantTracker,
    st: &'a mut TrackState,
    scratch: &'a mut Scratch,
    table: &'a MajorantTable,
    opts: FlightOptions,
    t: f64,
    steps: u32,
    pre: TrackState,
    last: Option<FlightSegment>,
    end: Option<TraceEnd>,
    stats: FlightStats,
}

impl<'a> Flight<'a> {
    /// Start a flight from a located state. Errors with `Unsupported` if the tracker has no
    /// majorant view, `InvalidInput` for a bad band, limit or policy.
    pub fn new(tracker: &'a dyn Tracker, st: &'a mut TrackState, scratch: &'a mut Scratch, table: &'a MajorantTable, opts: FlightOptions) -> Result<Self> {
        let view = tracker.majorant_view().ok_or(RtError::Unsupported("tracker has no majorant view (TrackerCaps::MAJORANT)"))?;
        if opts.band as usize >= table.n_bands() {
            return Err(RtError::InvalidInput("band is outside the majorant table"));
        }
        if opts.max_dist.is_nan() || opts.max_dist < 0.0 {
            return Err(RtError::InvalidInput("max_dist must be >= 0"));
        }
        check_policy(opts.policy)?;
        let end = st.cell.is_none().then_some(TraceEnd::LeftModel);
        let pre = st.clone();
        Ok(Self { view, st, scratch, table, opts, t: 0.0, steps: 0, pre, last: None, end, stats: FlightStats::default() })
    }

    /// Change the mode policy; applies from the next segment (switching is unbiased).
    pub fn set_policy(&mut self, p: ModePolicy) -> Result<()> {
        check_policy(p)?;
        self.opts.policy = p;
        Ok(())
    }

    /// The next segment, or `None` once the flight has ended ([`Flight::end`]). The state is
    /// advanced to the segment's end.
    pub fn next_segment(&mut self) -> Result<Option<FlightSegment>> {
        if self.end.is_some() {
            return Ok(None);
        }
        if self.steps >= self.opts.max_steps {
            self.end = Some(TraceEnd::Lost(lost(self.st, LostCause::MaxStepsExceeded)));
            return Ok(None);
        }
        self.steps += 1;
        self.pre.clone_from(self.st);
        let q = SegmentQuery::new(self.opts.policy, self.opts.scale, self.opts.band, self.table, self.opts.max_dist - self.t);
        let mut seg = self.view.advance_segment(self.st, &q, self.scratch)?;
        if !(seg.length().is_finite() && seg.length() >= 0.0) {
            return Err(RtError::NonFinite("segment length"));
        }
        seg.t0 += self.t;
        seg.t1 += self.t;
        if let StepEnd::Lost(l) = seg.end {
            self.end = Some(TraceEnd::Lost(l));
            return Ok(None);
        }
        self.t = seg.t1;
        match seg.end {
            StepEnd::Limit => self.end = Some(TraceEnd::Limit),
            StepEnd::RegionBoundary { .. } if self.st.cell.is_none() => self.end = Some(TraceEnd::LeftModel),
            StepEnd::Boundary { action, .. } if action.applied_in_step() && !self.opts.stop_at_deck => {}
            StepEnd::Boundary { surface, face, action } => self.end = Some(TraceEnd::Boundary { surface, face, action }),
            _ => {}
        }
        if seg.length() == 0.0 && matches!(seg.end, StepEnd::Limit) {
            return Ok(None);
        }
        match seg.mode {
            TrackMode::Surface => self.stats.surface_segments += 1,
            TrackMode::Delta => self.stats.delta_segments += 1,
        }
        self.last = Some(seg);
        Ok(Some(seg))
    }

    fn point_at(&self, seg: &FlightSegment, t: f64) -> Result<Point3> {
        if !(t.is_finite() && t >= seg.t0 && t <= seg.t1) {
            return Err(RtError::InvalidInput("t is outside the current segment"));
        }
        Ok(self.pre.pos + self.pre.dir * (t - seg.t0))
    }

    /// Locate the point at flight parameter `t` inside the last segment, from the node that
    /// owns the majorant. Does not change the state or the flight.
    pub fn probe(&mut self, t: f64) -> Result<Located> {
        let seg = self.last.ok_or(RtError::InvalidInput("no current segment"))?;
        let p = self.point_at(&seg, t)?;
        self.stats.probes += 1;
        if seg.resolved {
            let face = if t == seg.t0 { self.pre.entered_via } else { None };
            return Ok(Located::new(seg.cell, seg.region, face).with_levels(self.pre.levels.clone()));
        }
        self.view.locate_in_node(seg.node, p, Some(self.pre.dir), self.scratch)
    }

    /// End the flight with a collision at `t` inside the last segment: the state is placed
    /// there (cell, region, levels located from the node) and `end()` is `Stopped`.
    pub fn stop_at(&mut self, t: f64) -> Result<Located> {
        let loc = self.probe(t)?;
        let seg = self.last.ok_or(RtError::InvalidInput("no current segment"))?;
        self.st.pos = self.point_at(&seg, t)?;
        self.st.dir = self.pre.dir;
        self.st.cell = loc.cell;
        self.st.region = loc.region;
        self.st.entered_via = loc.on_face;
        self.st.levels = loc.levels.clone();
        self.t = t;
        self.last = None;
        self.end = Some(TraceEnd::Stopped);
        Ok(loc)
    }

    /// Lower bound of the distance to any face from the current state.
    pub fn safety_distance(&mut self) -> Result<f64> {
        self.view.safety_distance(self.st, self.scratch)
    }

    /// Flight parameter reached so far.
    pub fn distance(&self) -> f64 {
        self.t
    }

    /// Why the flight ended, once it has.
    pub fn end(&self) -> Option<&TraceEnd> {
        self.end.as_ref()
    }

    /// Work counters so far.
    pub fn stats(&self) -> &FlightStats {
        &self.stats
    }
}

fn check_policy(p: ModePolicy) -> Result<()> {
    if let ModePolicy::Auto { min_efficiency } = p
        && !(min_efficiency > 0.0 && min_efficiency <= 1.0)
    {
        return Err(RtError::InvalidInput("min_efficiency must be in (0, 1]"));
    }
    Ok(())
}

// ---------------------------------------------------------------------------------------------
// Tallies
// ---------------------------------------------------------------------------------------------

/// Flux tally for hybrid tracking: a track-length estimator on resolved surface segments plus
/// a collision estimator for collisions found in delta segments. Reduce one per worker with
/// [`FlightTally::merge`] in a fixed order.
///
/// Mixing is unbiased only if each particle flight scores each region of space by exactly one
/// estimator: score track length on surface segments, and collisions (`w / sigma_t`) on real
/// collisions inside delta segments. [`FlightTally::score_track`] refuses a segment that
/// cannot be binned per cell.
#[derive(Clone, Debug, PartialEq)]
pub struct FlightTally {
    bins: BinMap,
    /// Track-length scores per bin.
    pub track: Vec<f64>,
    /// Collision-estimator scores per bin.
    pub collision: Vec<f64>,
}

impl FlightTally {
    /// Zeroed tally over `bins`.
    pub fn new(bins: BinMap) -> Self {
        let n = bins.n_bins;
        Self { bins, track: vec![0.0; n], collision: vec![0.0; n] }
    }
    /// Score `weight * length` of a resolved surface segment. Errors with `Unsupported` for a
    /// delta or unresolved segment.
    pub fn score_track(&mut self, seg: &FlightSegment, weight: f64) -> Result<()> {
        if seg.mode != TrackMode::Surface || !seg.resolved {
            return Err(RtError::Unsupported("track-length scoring needs a resolved surface segment; score collisions instead"));
        }
        if let Some(b) = self.bins.bin(seg.cell) {
            self.track[b] += weight * seg.length();
        }
        Ok(())
    }
    /// Score a collision in `cell`: `weight / sigma_t`. A zero `sigma_t` scores nothing.
    pub fn score_collision(&mut self, cell: CellId, weight: f64, sigma_t: f64) -> Result<()> {
        if !(sigma_t.is_finite() && sigma_t >= 0.0) {
            return Err(RtError::NonFinite("sigma_t"));
        }
        if sigma_t > 0.0
            && let Some(b) = self.bins.bin(cell)
        {
            self.collision[b] += weight / sigma_t;
        }
        Ok(())
    }
    /// Flux estimate in a bin: both estimators summed.
    pub fn flux(&self, bin: usize) -> Option<f64> {
        Some(self.track.get(bin)? + self.collision.get(bin)?)
    }
    /// Add another tally (same bin count).
    pub fn merge(&mut self, o: &Self) -> Result<()> {
        if self.track.len() != o.track.len() {
            return Err(RtError::BinCountMismatch { left: self.track.len(), right: o.track.len() });
        }
        for (a, b) in self.track.iter_mut().zip(&o.track) {
            *a += *b;
        }
        for (a, b) in self.collision.iter_mut().zip(&o.collision) {
            *a += *b;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bands_partition_the_axis() {
        let b = EnergyBands::new(vec![1.0, 10.0]).unwrap();
        assert_eq!(b.n_bands(), 3);
        assert_eq!([0.5, 1.0, 9.9, 10.0, 1e9].map(|e| b.band_of(e).unwrap()), [0, 1, 1, 2, 2]);
        assert!(b.band_of(f64::NAN).is_err());
        assert!(EnergyBands::new(vec![2.0, 1.0]).is_err());
        assert_eq!(EnergyBands::single().band_of(5.0).unwrap(), 0);
    }

    #[test]
    fn violation_code_is_frozen() {
        let e = RtError::MajorantViolated { set: 0, band: 0, sigma_t: 2.0, majorant: 1.0 };
        assert_eq!(e.code(), -12);
        assert!(e.to_string().contains("majorant violated"));
    }

    #[test]
    fn tally_scores_and_merges() {
        let mut a = FlightTally::new(BinMap::identity(2));
        let seg = FlightSegment::new(TrackMode::Surface, NodeId(0), MaterialSetId(0), CellId(1), RegionId(0), 2.0, 1.0);
        a.score_track(&seg, 0.5).unwrap();
        a.score_collision(CellId(0), 1.0, 4.0).unwrap();
        let mut b = FlightTally::new(BinMap::identity(2));
        b.merge(&a).unwrap();
        assert_eq!((b.flux(0), b.flux(1)), (Some(0.25), Some(1.0)));
        let delta = FlightSegment::new(TrackMode::Delta, NodeId(0), MaterialSetId(0), CellId(0), RegionId(0), 1.0, 1.0);
        assert!(a.score_track(&delta, 1.0).is_err());
        assert!(matches!(a.merge(&FlightTally::new(BinMap::identity(3))), Err(RtError::BinCountMismatch { .. })));
    }
}
