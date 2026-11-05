//! The segment iterator: one flight type, two modes (surface and delta tracking).
//!
//! # Thesis
//! A flight is a sequence of *segments* `{t0, t1, node, cell, sigma_M}`. In **surface** mode a
//! segment is one cell (or one region run); the node majorant is a bound only and the caller
//! samples with its own `sigma_t(region)`, so nothing is rejected. In **delta** mode a segment
//! is the chord through one leaf of a *disjoint* majorant partition, with the leaf's majorant.
//! The transport code samples inside the segment and asks the flight to locate the tentative
//! collision ([`FlightCursor::probe`]) starting from the node that owns the majorant, never from
//! the root. Switching mode between segments is unbiased, so a hybrid is a per-segment choice
//! ([`FlightCursor::set_policy`] and [`FlightInput::with_block`]).
//!
//! # Cross sections never enter this crate
//! The tracker reports which regions each node contains ([`HierarchyView::set_regions`]; region
//! names are the material names). The caller pairs them with its own cross sections, computes
//! one majorant per node, and passes the slice on **every** call inside [`FlightInput`]. The
//! tracker holds no `sigma_t` and keeps no copy of the slice; to change energy the caller
//! passes another slice. A real collision whose `sigma_t` exceeds the node majorant is an error
//! ([`FlightCursor::check_collision`], [`RtError::MajorantViolated`]); it is never clamped.
//!
//! **`NodeId`s index the prototype tree, not placed instances.** A node is a partition node of a
//! prototype leaf, a lattice component or a universe/placement component of the `WorldSpec`.
//! Every instance of one prototype (every element filled with the same universe, every copy of
//! a placed leaf) shares its `NodeId`s; the instance is resolved through `TrackState::levels` and
//! index arithmetic. The slice length is therefore independent of the instance count. A
//! per-instance majorant needs distinct prototypes, which is also what distinct materials
//! need, because regions are prototypes.
//!
//! # Per-call validation is O(1)
//! Each call checks the slice lengths (`majorants.len() == node_count`,
//! `block.len() <= node_count`) and the one majorant entry it reads (finite, `>= 0`); it never
//! scans the slices. [`FlightInput::validate`] checks every entry and that `block` is sorted
//! and deduplicated; callers run it once when they build the slices.
//!
//! # Driving a flight (Monte Carlo)
//! ```text
//! let input = FlightInput::new(&majorants).with_block(&blocked);   // validated once
//! let mut cur = tracker.majorant_view().unwrap().begin(&st, &opts)?;
//! 'flight: while let Some(seg) = cur.next_segment(view, input, &mut st, &mut scratch)? {
//!     let mut t = seg.t0;
//!     loop {
//!         t += -ln(xi()) / seg.sigma_m;                 // sigma_m = 0: never collides
//!         if !(t < seg.t1) { break; }                   // flew the whole segment
//!         let loc = cur.probe(view, input, &mut scratch, t)?;    // hinted, no root restart
//!         let sigma_t = xs(loc.region, energy);         // the caller's own cross section
//!         cur.check_collision(sigma_t)?;                // sigma_t > majorant is an error
//!         if xi() * seg.sigma_m < sigma_t {             // real (always true if sigma_m == sigma_t)
//!             cur.stop_at(view, input, &mut st, &mut scratch, t)?;   // state at the collision
//!             break 'flight;                            // ... sample reaction, new direction
//!         }                                             // else virtual: keep sampling in seg
//!     }
//! }
//! let end = cur.end();   // Boundary / Limit / LeftModel / Stopped (collision) / Lost
//! ```
//! A surface-only code can skip the probe, because `seg.cell` is the cell (`seg.resolved`).
//! Sn/MOC and mesh tallies use [`SegmentScale::Cell`] with [`ModePolicy::Surface`] and score
//! every segment. [`Flight`] is a thin borrowing wrapper over the cursor for Rust callers that
//! keep one input for the whole flight.
//!
//! # Contract for implementors ([`MajorantTracker`])
//! * Leaves of [`HierarchyView`] **tile** the model disjointly ([`validate_partition`] checks
//!   it, including that a node's material set covers every region below it). A BVH has
//!   overlapping boxes and is not a partition; it may serve `locate`, not this.
//! * Material sets are deduplicated and stored in CSR; ids are assigned in ascending order of
//!   first appearance ([`MaterialSets`] does exactly this).
//! * `advance_segment` moves the state to the end of the segment, applying events exactly as
//!   `Tracker::step` does (lattice walls are `StepEnd::SlotChange`, reflect/wrap are applied,
//!   `Respond` boundaries are not). Segments of one flight are contiguous and their lengths
//!   sum to the chord, whatever the mode. A delta segment ends at the wall of its node; a wall
//!   that is not a model face is reported as `StepEnd::SlotChange` with the cell entered
//!   (possibly the same cell), and `levels` change only at real lattice or placement walls. It
//!   reads exactly one majorant entry
//!   ([`FlightInput::majorant`], which validates it) and reports it as `sigma_m`.
//! * Mode choice is a pure function of `(policy, node, block)` ([`choose_mode`]); results are
//!   bit-identical for any thread count.
//! * `locate_in_node` starts from the node: O(1) lattice index, ring search, walk inside a
//!   meshed leaf. It must not restart from the root.

use crate::error::{BuildIssue, LostCause, Result, RtError};
use crate::ids::{CellId, FaceId, MaterialSetId, NodeId, RegionId};
use crate::state::{Level, Located, Scratch, StepEnd, TrackState};
use crate::tally::{BinMap, TraceEnd};
use crate::tracker::{Tracker, lost};
use sm_core::{Aabb, Point3, Vector3};
use std::collections::BTreeMap;

// ---------------------------------------------------------------------------------------------
// Work counters
// ---------------------------------------------------------------------------------------------

/// Deterministic work counters (benchmark with these, not clocks). They live in
/// [`Scratch::counters`]; the flight cursor increments the segment-level ones and an implementor
/// the rest. Nothing reads them back into a result.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct WorkCounters {
    /// `Tracker::step` calls.
    pub steps: u64,
    /// Point locations.
    pub locates: u64,
    /// Segments produced.
    pub segments: u64,
    /// Surface-mode segments.
    pub surface_segments: u64,
    /// Delta-mode segments.
    pub delta_segments: u64,
    /// Partition nodes visited.
    pub node_visits: u64,
    /// Face (plane) intersection tests.
    pub face_tests: u64,
    /// Analytic surface evaluations.
    pub surface_evals: u64,
    /// Candidate cells tested by locations.
    pub locate_candidates: u64,
    /// BVH descents.
    pub bvh_descents: u64,
    /// Lattice locations answered by O(1) index arithmetic.
    pub lattice_index_locates: u64,
    /// Locations that fell back to a slow scan.
    pub slow_locates: u64,
    /// Recovery nudges.
    pub nudges: u64,
    /// Tentative collisions located (delta mode).
    pub tentative: u64,
    /// Tentative collisions the caller rejected (virtual collisions); incremented by the caller.
    pub rejected: u64,
    /// Safety-distance queries answered from the cursor's cache.
    pub safety_skips: u64,
    /// Region jumps taken by `fire`.
    pub jumps: u64,
}

impl WorkCounters {
    /// Add another set of counters (reduce workers in a fixed order).
    pub fn add(&mut self, o: &Self) {
        self.steps += o.steps;
        self.locates += o.locates;
        self.segments += o.segments;
        self.surface_segments += o.surface_segments;
        self.delta_segments += o.delta_segments;
        self.node_visits += o.node_visits;
        self.face_tests += o.face_tests;
        self.surface_evals += o.surface_evals;
        self.locate_candidates += o.locate_candidates;
        self.bvh_descents += o.bvh_descents;
        self.lattice_index_locates += o.lattice_index_locates;
        self.slow_locates += o.slow_locates;
        self.nudges += o.nudges;
        self.tentative += o.tentative;
        self.rejected += o.rejected;
        self.safety_skips += o.safety_skips;
        self.jumps += o.jumps;
    }
}

// ---------------------------------------------------------------------------------------------
// Hierarchy
// ---------------------------------------------------------------------------------------------

/// What a node of the majorant partition is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum NodeKind {
    /// An internal node: a universe or a kd/octree split. Never yields segments. Its children
    /// tile it in its own frame.
    Internal,
    /// A lattice: an internal node whose children are prototypes in element frames, repeated by
    /// index arithmetic (instances are not nodes). Never yields segments; its children are not
    /// expected to tile its bounds.
    Lattice,
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
    /// Frame-local bounds (exact for lattice slots; conservative boxes are allowed for leaves,
    /// but leaves must still be disjoint as *regions of space*).
    pub bounds: Aabb,
    /// Kind of node.
    pub kind: NodeKind,
    /// The deduplicated set of regions occurring inside (covers every region below it).
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
        !matches!(self.kind, NodeKind::Internal | NodeKind::Lattice)
    }
}

/// Read-only view of the disjoint majorant partition and the material sets.
pub trait HierarchyView: Send + Sync {
    /// Number of nodes (ids are dense `0..node_count`). This is the length of every majorant
    /// slice a caller passes.
    fn node_count(&self) -> usize;
    /// The root node.
    fn root(&self) -> NodeId;
    /// A node, or `None` for an unknown id.
    fn node(&self, n: NodeId) -> Option<NodeInfo>;
    /// A leaf that meets the cell (the lowest id when several do: a cell of a gridded prototype
    /// can span several leaves, so this is not unique ownership).
    fn node_of_cell(&self, c: CellId) -> Option<NodeId>;
    /// Number of material sets (ids are dense `0..set_count`).
    fn set_count(&self) -> usize;
    /// Regions of a set, ascending and deduplicated. Region names
    /// (`RegionInfo::name`) are the material names the caller pairs with its cross sections.
    fn set_regions(&self, s: MaterialSetId) -> Option<&[RegionId]>;
    /// The singleton set `{r}`: every region has one.
    fn region_set(&self, r: RegionId) -> Option<MaterialSetId>;
}

/// Deduplicated material sets in CSR form. Ids are dense and assigned in ascending order of
/// first appearance; lookup uses a `BTreeMap`, so nothing depends on hash order. Implementors
/// of [`HierarchyView`] use it to hold their sets.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MaterialSets {
    offsets: Vec<u32>,
    regions: Vec<RegionId>,
    index: BTreeMap<Vec<u32>, u32>,
}

impl Default for MaterialSets {
    fn default() -> Self {
        Self { offsets: vec![0], regions: Vec::new(), index: BTreeMap::new() }
    }
}

impl MaterialSets {
    /// An empty collection.
    pub fn new() -> Self {
        Self::default()
    }
    /// Add the set of `regions` (sorted and deduplicated here) and return its id; an equal set
    /// that is already present returns the existing id.
    pub fn intern(&mut self, regions: impl IntoIterator<Item = RegionId>) -> MaterialSetId {
        let mut key: Vec<u32> = regions.into_iter().map(|r| r.0).collect();
        key.sort_unstable();
        key.dedup();
        if let Some(&id) = self.index.get(&key) {
            return MaterialSetId(id);
        }
        let id = self.len() as u32;
        self.regions.extend(key.iter().map(|&r| RegionId(r)));
        self.offsets.push(self.regions.len() as u32);
        self.index.insert(key, id);
        MaterialSetId(id)
    }
    /// Number of sets.
    pub fn len(&self) -> usize {
        self.offsets.len() - 1
    }
    /// True if there are no sets.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
    /// Regions of a set, ascending.
    pub fn get(&self, s: MaterialSetId) -> Option<&[RegionId]> {
        let i = s.get();
        (i < self.len()).then(|| &self.regions[self.offsets[i] as usize..self.offsets[i + 1] as usize])
    }
}

/// Check the partition. For every internal node that is not a lattice, its children are pairwise
/// interior-disjoint boxes whose volumes sum to the node's volume (relative `1e-9`), in the
/// node's frame (a lattice's children are prototypes in element frames and are not tiled
/// against it). Every node's material set contains the regions of each of its children (a
/// node's set covers every region below it). `O(L^2)`; a build-time or test check.
pub fn validate_partition(h: &dyn HierarchyView) -> Result<()> {
    let vol = |b: &Aabb| (b.hi.x - b.lo.x).max(0.0) * (b.hi.y - b.lo.y).max(0.0) * (b.hi.z - b.lo.z).max(0.0);
    let overlap = |a: &Aabb, b: &Aabb| {
        let d = |l0: f64, h0: f64, l1: f64, h1: f64| (h0.min(h1) - l0.max(l1)).max(0.0);
        d(a.lo.x, a.hi.x, b.lo.x, b.hi.x) * d(a.lo.y, a.hi.y, b.lo.y, b.hi.y) * d(a.lo.z, a.hi.z, b.lo.z, b.hi.z)
    };
    let mut issues = Vec::new();
    if h.node(h.root()).is_none() {
        return Err(RtError::Build(vec![BuildIssue::new("hierarchy.root", "root node is missing")]));
    }
    let nodes: Vec<NodeInfo> = (0..h.node_count()).filter_map(|i| h.node(NodeId::new(i))).collect();
    let mut children: BTreeMap<NodeId, Vec<&NodeInfo>> = BTreeMap::new();
    for n in &nodes {
        if !n.parent.is_none() {
            children.entry(n.parent).or_default().push(n);
        }
    }
    for (parent, kids) in &children {
        let Some(p) = nodes.iter().find(|n| n.id == *parent) else {
            issues.push(BuildIssue::new("hierarchy.parent", format!("node {} names a missing parent {}", kids[0].id.0, parent.0)));
            continue;
        };
        if p.kind != NodeKind::Lattice {
            let total: f64 = kids.iter().map(|k| vol(&k.bounds)).sum();
            if (total - vol(&p.bounds)).abs() > 1e-9 * vol(&p.bounds).max(f64::MIN_POSITIVE) {
                issues.push(BuildIssue::new(
                    "hierarchy.cover",
                    format!("children of node {} have volumes summing to {total}, its volume is {}", p.id.0, vol(&p.bounds)),
                ));
            }
            for (i, a) in kids.iter().enumerate() {
                for b in &kids[i + 1..] {
                    let o = overlap(&a.bounds, &b.bounds);
                    if o > 1e-12 * vol(&a.bounds).max(vol(&b.bounds)) {
                        issues.push(BuildIssue::new("hierarchy.overlap", format!("nodes {} and {} overlap (volume {o})", a.id.0, b.id.0)));
                    }
                }
            }
        }
        let Some(parent_set) = h.set_regions(p.set) else {
            issues.push(BuildIssue::new("hierarchy.set", format!("node {} names an unknown material set", p.id.0)));
            continue;
        };
        for k in kids {
            let Some(child_set) = h.set_regions(k.set) else {
                issues.push(BuildIssue::new("hierarchy.set", format!("node {} names an unknown material set", k.id.0)));
                continue;
            };
            if let Some(r) = child_set.iter().find(|r| parent_set.binary_search(r).is_err()) {
                issues.push(BuildIssue::new(
                    "hierarchy.containment",
                    format!("node {} holds region {} that its parent {} does not list", k.id.0, r.0, p.id.0),
                ));
            }
        }
    }
    if issues.is_empty() { Ok(()) } else { Err(RtError::Build(issues)) }
}

// ---------------------------------------------------------------------------------------------
// Per-call input
// ---------------------------------------------------------------------------------------------

/// The caller's per-call data: one majorant per node and the nodes where delta tracking is
/// blocked. Passed to every `next_segment`, `probe` and `stop_at`; the cursor never holds the
/// slices, so they may differ between calls (each segment uses the slices of the call that
/// produced it).
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub struct FlightInput<'a> {
    majorants: &'a [f64],
    block: &'a [NodeId],
}

impl<'a> FlightInput<'a> {
    /// A majorant slice indexed by `NodeId` (length `HierarchyView::node_count()`). Leaves use
    /// their own entry; internal nodes are not read.
    pub const fn new(majorants: &'a [f64]) -> Self {
        Self { majorants, block: &[] }
    }
    /// Force surface segments in `block` (blockdt). The caller keeps it sorted and
    /// deduplicated: membership is a binary search, `O(log n_block)` per node entered. An
    /// unsorted `block` is not detected per call and gives deterministic but unspecified
    /// membership, never a panic; [`Self::validate`] detects it.
    pub const fn with_block(mut self, block: &'a [NodeId]) -> Self {
        self.block = block;
        self
    }
    /// The majorant slice.
    pub const fn majorants(&self) -> &'a [f64] {
        self.majorants
    }
    /// The blocked nodes.
    pub const fn block(&self) -> &'a [NodeId] {
        self.block
    }
    /// True if `node` is in `block`.
    pub fn is_blocked(&self, node: NodeId) -> bool {
        self.block.binary_search(&node).is_ok()
    }
    /// The slice-length checks of every call: `InvalidInput` unless
    /// `majorants.len() == node_count` and `block.len() <= node_count`. O(1).
    pub fn check_lengths(&self, node_count: usize) -> Result<()> {
        if self.majorants.len() != node_count {
            return Err(RtError::InvalidInput("majorant slice length must equal the node count"));
        }
        if self.block.len() > node_count {
            return Err(RtError::InvalidInput("block has more entries than the tracker has nodes"));
        }
        Ok(())
    }
    /// The majorant of `node`, checked: finite and `>= 0`, else
    /// [`RtError::InvalidMajorant`] naming the node. An id beyond the slice is `InvalidInput`.
    /// This is the one entry a call reads.
    pub fn majorant(&self, node: NodeId) -> Result<f64> {
        let v = *self.majorants.get(node.get()).ok_or(RtError::InvalidInput("node is outside the majorant slice"))?;
        if !(v.is_finite() && v >= 0.0) {
            return Err(RtError::InvalidMajorant { node, value: v });
        }
        Ok(v)
    }
    /// One-time full check: lengths, every majorant entry (finite, `>= 0`), and that `block` is
    /// strictly ascending (sorted, deduplicated) with every id below the node count. `O(n)`;
    /// run it when the slices are built, not per call.
    pub fn validate(&self, view: &dyn HierarchyView) -> Result<()> {
        let n = view.node_count();
        self.check_lengths(n)?;
        for i in 0..n {
            self.majorant(NodeId::new(i))?;
        }
        if self.block.windows(2).any(|w| w[0] >= w[1]) {
            return Err(RtError::InvalidInput("block must be sorted and deduplicated"));
        }
        if self.block.last().is_some_and(|b| b.get() >= n) {
            return Err(RtError::InvalidInput("block names a node outside the tracker"));
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
    /// One cell (or region run); the caller samples with its own `sigma_t(region)`.
    Surface,
    /// The chord through one leaf with the leaf's majorant.
    Delta,
}

/// Per-segment mode choice. Hybrid tracking is caller-driven: change the policy between
/// segments with [`FlightCursor::set_policy`] or block nodes with [`FlightInput::with_block`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ModePolicy {
    /// Always surface segments.
    Surface,
    /// Delta segments, except in blocked nodes (leaves of one cell still yield one cell-wide
    /// segment).
    Delta,
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

/// Shared mode rule for implementors: surface under [`ModePolicy::Surface`] and in blocked
/// nodes, delta otherwise. A pure function of its arguments.
pub fn choose_mode(policy: ModePolicy, node: NodeId, input: &FlightInput<'_>) -> TrackMode {
    match policy {
        ModePolicy::Surface => TrackMode::Surface,
        ModePolicy::Delta if input.is_blocked(node) => TrackMode::Surface,
        ModePolicy::Delta => TrackMode::Delta,
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
    /// The node's material set (the regions that may occur in the segment).
    pub set: MaterialSetId,
    /// The cell at `t0`.
    pub cell: CellId,
    /// The region at `t0`.
    pub region: RegionId,
    /// True if the whole segment lies in `cell` (always true for surface segments of
    /// [`SegmentScale::Cell`]); false for delta and region-run segments.
    pub resolved: bool,
    /// The node's majorant, `majorants[node]` of the call. In a surface segment it is a bound
    /// only; the caller samples with its own `sigma_t(region)`.
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
    /// (`t0 = 0`, `t1 = length`); the cursor shifts it to flight coordinates.
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
    /// The caller's majorants and blocked nodes for this call.
    pub input: FlightInput<'a>,
    /// Remaining distance (the segment ends in `StepEnd::Limit` if it gets there).
    pub max_dist: f64,
}

impl<'a> SegmentQuery<'a> {
    /// Construct (the type is `#[non_exhaustive]`).
    pub fn new(policy: ModePolicy, scale: SegmentScale, input: FlightInput<'a>, max_dist: f64) -> Self {
        Self { policy, scale, input, max_dist }
    }
}

/// The segment-producing capability of a tracker (`Tracker::majorant_view`).
pub trait MajorantTracker: Tracker {
    /// The disjoint partition and the material sets.
    fn hierarchy(&self) -> &dyn HierarchyView;

    /// Move `st` to the end of the next segment and describe it. Obeys the module contract:
    /// events as `Tracker::step`, contiguous, lengths sum to the chord, no allocation after
    /// warm-up, mode via [`choose_mode`], one validated majorant read via
    /// [`FlightInput::majorant`] (before the state moves, so a bad entry leaves `st`
    /// unchanged). On `StepEnd::Lost` the state is unchanged.
    fn advance_segment(&self, st: &mut TrackState, q: &SegmentQuery<'_>, scratch: &mut Scratch) -> Result<FlightSegment>;

    /// Locate `p` starting from `node` (never from the root). `levels` is the instance path of
    /// the segment (`TrackState::levels` at its start; constant over a segment), which resolves
    /// the prototype node to one instance. `dir` breaks ties on faces.
    fn locate_in_node(&self, node: NodeId, levels: &[Level], p: Point3, dir: Option<Vector3>, scratch: &mut Scratch) -> Result<Located>;

    /// A lower bound of the distance from the state to any cell face (an exact one for convex
    /// cells): a sampled flight shorter than this needs no intersection test. `0.0` is always
    /// valid and is the default.
    fn safety_distance(&self, _st: &TrackState, _scratch: &mut Scratch) -> Result<f64> {
        Ok(0.0)
    }

    /// Start a flight from a located state: an opaque, resumable [`FlightCursor`] that borrows
    /// nothing. `InvalidInput` for a bad limit.
    fn begin(&self, st: &TrackState, o: &FlightOptions) -> Result<FlightCursor> {
        FlightCursor::new(st, o)
    }
}

// ---------------------------------------------------------------------------------------------
// The flight
// ---------------------------------------------------------------------------------------------

/// Options of a flight. No energy band: the caller changes energy by passing another majorant
/// slice.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct FlightOptions {
    /// Stop after this flight parameter (`TraceEnd::Limit`).
    pub max_dist: f64,
    /// Segment budget (`TraceEnd::Lost(MaxStepsExceeded)` beyond it).
    pub max_steps: u32,
    /// Return every `Respond`/`Reflect`/`Wrap` boundary instead of continuing through applied ones.
    pub stop_at_deck: bool,
    /// Mode policy (changeable between segments with [`FlightCursor::set_policy`]).
    pub policy: ModePolicy,
    /// Granularity of surface segments.
    pub scale: SegmentScale,
}

impl Default for FlightOptions {
    fn default() -> Self {
        Self { max_dist: f64::INFINITY, max_steps: 1_000_000, stop_at_deck: false, policy: ModePolicy::Surface, scale: SegmentScale::Cell }
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
}

/// Relative slack for rounding when comparing a cross section to its majorant.
const MAJORANT_SLACK: f64 = 1e-12;

/// A cached lower bound of the distance to any face, valid in `cell` around `pos`.
#[derive(Clone, Copy, Debug)]
struct SafetyCache {
    cell: CellId,
    pos: Point3,
    dist: f64,
}

/// What `check_collision` needs about the current segment.
#[derive(Clone, Copy, Debug)]
struct CurrentSegment {
    node: NodeId,
    majorant: f64,
}

/// A resumable flight: the segment iterator over one particle track.
///
/// It owns no borrow and no heap at hierarchy depth up to `LEVEL_INLINE`: the tracker, the
/// state, the scratch and the per-call [`FlightInput`] are passed to every call, so a code can
/// keep a cursor in its particle bank across calls and threads. After `next_segment` returns
/// a segment the state sits at the segment's end; [`FlightCursor::probe`],
/// [`FlightCursor::stop_at`] and [`FlightCursor::check_collision`] address the *last* segment.
/// The safety-distance cache lives here, not in `TrackState` (whose equality is unchanged).
#[derive(Clone, Debug)]
pub struct FlightCursor {
    opts: FlightOptions,
    t: f64,
    steps: u32,
    pre: TrackState,
    last: Option<FlightSegment>,
    end: Option<TraceEnd>,
    cur: Option<CurrentSegment>,
    at: (Point3, CellId),
    safety: Option<SafetyCache>,
}

fn check_opts(o: &FlightOptions) -> Result<()> {
    if o.max_dist.is_nan() || o.max_dist < 0.0 {
        return Err(RtError::InvalidInput("max_dist must be >= 0"));
    }
    Ok(())
}

impl FlightCursor {
    /// Start a flight from a located state (use [`MajorantTracker::begin`]). `InvalidInput`
    /// for a bad limit.
    pub fn new(st: &TrackState, o: &FlightOptions) -> Result<Self> {
        check_opts(o)?;
        let end = st.cell.is_none().then_some(TraceEnd::LeftModel);
        Ok(Self { opts: *o, t: 0.0, steps: 0, pre: st.clone(), last: None, end, cur: None, at: (st.pos, st.cell), safety: None })
    }

    /// Change the mode policy; applies from the next segment (switching is unbiased).
    pub fn set_policy(&mut self, p: ModePolicy) -> Result<()> {
        self.opts.policy = p;
        Ok(())
    }

    /// The next segment, or `None` once the flight has ended ([`FlightCursor::end`]). The state
    /// is advanced to the segment's end. `InvalidInput` for slices of the wrong length;
    /// `InvalidMajorant` for a bad entry at the segment's node (the state is then unchanged).
    pub fn next_segment(
        &mut self,
        t: &dyn MajorantTracker,
        input: FlightInput<'_>,
        st: &mut TrackState,
        s: &mut Scratch,
    ) -> Result<Option<FlightSegment>> {
        if self.end.is_some() {
            self.cur = None;
            return Ok(None);
        }
        input.check_lengths(t.hierarchy().node_count())?;
        if self.steps >= self.opts.max_steps {
            self.end = Some(TraceEnd::Lost(lost(st, LostCause::MaxStepsExceeded)));
            self.cur = None;
            return Ok(None);
        }
        self.pre.clone_from(st);
        let q = SegmentQuery::new(self.opts.policy, self.opts.scale, input, self.opts.max_dist - self.t);
        let mut seg = t.advance_segment(st, &q, s)?;
        self.steps += 1;
        if !(seg.length().is_finite() && seg.length() >= 0.0) {
            return Err(RtError::NonFinite("segment length"));
        }
        seg.t0 += self.t;
        seg.t1 += self.t;
        s.counters.segments += 1;
        if let StepEnd::Lost(l) = seg.end {
            self.end = Some(TraceEnd::Lost(l));
            self.cur = None;
            return Ok(None);
        }
        self.t = seg.t1;
        match seg.end {
            StepEnd::Limit => self.end = Some(TraceEnd::Limit),
            StepEnd::RegionBoundary { .. } if st.cell.is_none() => self.end = Some(TraceEnd::LeftModel),
            StepEnd::Boundary { action, .. } if action.applied_in_step() && !self.opts.stop_at_deck => {
                self.safety = None;
            }
            StepEnd::Boundary { surface, face, action } => {
                self.safety = None;
                self.end = Some(TraceEnd::Boundary { surface, face, action });
            }
            _ => {}
        }
        if seg.length() == 0.0 && matches!(seg.end, StepEnd::Limit) {
            self.cur = None;
            return Ok(None);
        }
        match seg.mode {
            TrackMode::Surface => s.counters.surface_segments += 1,
            TrackMode::Delta => s.counters.delta_segments += 1,
        }
        self.cur = Some(CurrentSegment { node: seg.node, majorant: seg.sigma_m });
        self.at = (self.pre.pos, seg.cell);
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
    /// owns the majorant. Does not change the state or the flight; remembers the point for
    /// [`FlightCursor::check_collision`].
    pub fn probe(&mut self, tr: &dyn MajorantTracker, input: FlightInput<'_>, s: &mut Scratch, t: f64) -> Result<Located> {
        input.check_lengths(tr.hierarchy().node_count())?;
        let seg = self.last.ok_or(RtError::InvalidInput("no current segment"))?;
        let p = self.point_at(&seg, t)?;
        s.counters.locates += 1;
        if seg.mode == TrackMode::Delta {
            s.counters.tentative += 1;
        }
        let loc = if seg.resolved {
            let face = if t == seg.t0 { self.pre.entered_via } else { None };
            Located::new(seg.cell, seg.region, face).with_levels(self.pre.levels.clone())
        } else {
            tr.locate_in_node(seg.node, &self.pre.levels, p, Some(self.pre.dir), s)?
        };
        self.at = (p, loc.cell);
        Ok(loc)
    }

    /// End the flight with a collision at `t` inside the last segment: the state is placed
    /// there (cell, region, levels located from the node) and `end()` is `Stopped`.
    pub fn stop_at(
        &mut self,
        tr: &dyn MajorantTracker,
        input: FlightInput<'_>,
        st: &mut TrackState,
        s: &mut Scratch,
        t: f64,
    ) -> Result<Located> {
        let loc = self.probe(tr, input, s, t)?;
        let seg = self.last.ok_or(RtError::InvalidInput("no current segment"))?;
        st.pos = self.point_at(&seg, t)?;
        st.dir = self.pre.dir;
        st.cell = loc.cell;
        st.region = loc.region;
        st.entered_via = loc.on_face;
        st.levels = loc.levels.clone();
        self.t = t;
        self.last = None;
        self.cur = None;
        self.safety = None;
        self.end = Some(TraceEnd::Stopped);
        Ok(loc)
    }

    /// Check a collision's total cross section against the majorant of the current segment's
    /// node, cached by the last `next_segment`. Takes no slice. `Err(MajorantViolated)`
    /// (code -12) when `sigma_t > majorant * (1 + 1e-12)`; never clamped; nothing advances.
    /// `NonFinite` for a NaN, infinite or negative `sigma_t`; `InvalidInput` before the first
    /// segment or after the flight has ended (`next_segment` returned `None`, or `stop_at`).
    pub fn check_collision(&self, sigma_t: f64) -> Result<()> {
        let cur = self.cur.ok_or(RtError::InvalidInput("no current segment to check a collision against"))?;
        if !(sigma_t.is_finite() && sigma_t >= 0.0) {
            return Err(RtError::NonFinite("sigma_t"));
        }
        if sigma_t > cur.majorant * (1.0 + MAJORANT_SLACK) {
            return Err(RtError::MajorantViolated { node: cur.node, sigma_t, majorant: cur.majorant, pos: self.at.0, cell: self.at.1 });
        }
        Ok(())
    }

    /// Lower bound of the distance to any face from the state. A bound computed earlier in the
    /// same cell is reused, reduced by the distance moved since (counted in
    /// `Scratch::counters.safety_skips`); the result is always a valid lower bound.
    pub fn safety_distance(&mut self, tr: &dyn MajorantTracker, st: &TrackState, s: &mut Scratch) -> Result<f64> {
        if let Some(c) = self.safety
            && c.cell == st.cell
        {
            let d = c.dist - (st.pos - c.pos).norm();
            if d > 0.0 {
                s.counters.safety_skips += 1;
                return Ok(d);
            }
        }
        let d = tr.safety_distance(st, s)?;
        self.safety = Some(SafetyCache { cell: st.cell, pos: st.pos, dist: d });
        Ok(d)
    }

    /// Flight parameter reached so far.
    pub fn distance(&self) -> f64 {
        self.t
    }

    /// Why the flight ended, once it has.
    pub fn end(&self) -> Option<&TraceEnd> {
        self.end.as_ref()
    }
}

/// A borrowing wrapper over [`FlightCursor`] for Rust callers that keep one tracker, state,
/// scratch and input for the whole flight. Codes that bank particles use the cursor directly.
pub struct Flight<'a> {
    view: &'a dyn MajorantTracker,
    st: &'a mut TrackState,
    scratch: &'a mut Scratch,
    input: FlightInput<'a>,
    cur: FlightCursor,
}

impl<'a> Flight<'a> {
    /// Start a flight from a located state. Errors with `Unsupported` if the tracker has no
    /// majorant view, `InvalidInput` for a bad limit or slice length.
    pub fn new(tracker: &'a dyn Tracker, st: &'a mut TrackState, scratch: &'a mut Scratch, input: FlightInput<'a>, opts: FlightOptions) -> Result<Self> {
        let view = tracker.majorant_view().ok_or(RtError::Unsupported("tracker has no majorant view (TrackerCaps::MAJORANT)"))?;
        input.check_lengths(view.hierarchy().node_count())?;
        let cur = view.begin(st, &opts)?;
        Ok(Self { view, st, scratch, input, cur })
    }
    /// Use other slices from the next call on.
    pub fn set_input(&mut self, input: FlightInput<'a>) -> Result<()> {
        input.check_lengths(self.view.hierarchy().node_count())?;
        self.input = input;
        Ok(())
    }
    /// Change the mode policy; applies from the next segment.
    pub fn set_policy(&mut self, p: ModePolicy) -> Result<()> {
        self.cur.set_policy(p)
    }
    /// See [`FlightCursor::next_segment`].
    pub fn next_segment(&mut self) -> Result<Option<FlightSegment>> {
        self.cur.next_segment(self.view, self.input, self.st, self.scratch)
    }
    /// See [`FlightCursor::probe`].
    pub fn probe(&mut self, t: f64) -> Result<Located> {
        self.cur.probe(self.view, self.input, self.scratch, t)
    }
    /// See [`FlightCursor::stop_at`].
    pub fn stop_at(&mut self, t: f64) -> Result<Located> {
        self.cur.stop_at(self.view, self.input, self.st, self.scratch, t)
    }
    /// See [`FlightCursor::check_collision`].
    pub fn check_collision(&self, sigma_t: f64) -> Result<()> {
        self.cur.check_collision(sigma_t)
    }
    /// See [`FlightCursor::safety_distance`].
    pub fn safety_distance(&mut self) -> Result<f64> {
        self.cur.safety_distance(self.view, self.st, self.scratch)
    }
    /// Flight parameter reached so far.
    pub fn distance(&self) -> f64 {
        self.cur.distance()
    }
    /// Why the flight ended, once it has.
    pub fn end(&self) -> Option<&TraceEnd> {
        self.cur.end()
    }
    /// The underlying cursor.
    pub fn cursor(&self) -> &FlightCursor {
        &self.cur
    }
}

// ---------------------------------------------------------------------------------------------
// Tallies
// ---------------------------------------------------------------------------------------------

/// Flux tally for hybrid tracking: a track-length estimator on resolved surface segments plus
/// a collision estimator for collisions found in delta segments. Reduce one per worker with
/// [`FlightTally::merge`] in a fixed order. It holds no cross section: `score_collision` takes
/// the caller's `sigma_t`.
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
    fn violation_code_is_frozen() {
        let e = RtError::MajorantViolated { node: NodeId(2), sigma_t: 2.0, majorant: 1.0, pos: Point3::new(0.0, 0.0, 0.0), cell: CellId(1) };
        assert_eq!(e.code(), -12);
        assert!(e.to_string().contains("majorant violated"));
    }

    #[test]
    fn material_sets_dedup_in_first_appearance_order() {
        let mut m = MaterialSets::new();
        let r = |v: &[u32]| v.iter().map(|&x| RegionId(x)).collect::<Vec<_>>();
        let a = m.intern(r(&[2, 0]));
        let b = m.intern(r(&[1]));
        let c = m.intern(r(&[0, 2, 2]));
        assert_eq!((a.0, b.0, c.0), (0, 1, 0));
        assert_eq!(m.len(), 2);
        assert_eq!(m.get(a), Some(&r(&[0, 2])[..]));
        assert!(m.get(MaterialSetId(2)).is_none());
    }

    #[test]
    fn flight_input_checks_are_constant_time_and_named() {
        let maj = [1.0, f64::NAN, -1.0, 2.0];
        let inp = FlightInput::new(&maj);
        assert!(inp.check_lengths(4).is_ok());
        assert!(matches!(inp.check_lengths(5), Err(RtError::InvalidInput(_))));
        let blk = [NodeId(0), NodeId(1), NodeId(2), NodeId(3), NodeId(4)];
        assert!(matches!(FlightInput::new(&maj).with_block(&blk).check_lengths(4), Err(RtError::InvalidInput(_))));
        assert_eq!(inp.majorant(NodeId(0)).unwrap(), 1.0);
        for bad in [1, 2] {
            assert!(matches!(inp.majorant(NodeId(bad)), Err(RtError::InvalidMajorant { node, .. }) if node.0 == bad));
        }
        assert!(matches!(inp.majorant(NodeId(9)), Err(RtError::InvalidInput(_))));
        let sorted = [NodeId(1), NodeId(3)];
        assert!(FlightInput::new(&maj).with_block(&sorted).is_blocked(NodeId(3)));
        // An unsorted block never panics; membership is unspecified.
        let unsorted = [NodeId(3), NodeId(1)];
        let _ = FlightInput::new(&maj).with_block(&unsorted).is_blocked(NodeId(1));
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

    #[test]
    fn work_counters_add_every_field() {
        let mut a = WorkCounters { steps: 1, jumps: 2, ..WorkCounters::default() };
        let b = WorkCounters { steps: 10, jumps: 20, rejected: 3, ..WorkCounters::default() };
        a.add(&b);
        assert_eq!((a.steps, a.jumps, a.rejected), (11, 22, 3));
    }
}
