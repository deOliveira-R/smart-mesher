//! Segment-wise delta tracking over a disjoint partition (additive extension for W3b).
//!
//! # Who owns what (the transport-integration rule)
//! * **The tracker owns geometry**: a [`MajorantStructure`], a disjoint partition of the model
//!   into [`NodeInfo`] leaves (lattice elements, instances, meshed kd/octree leaves, analytic
//!   primitives), each with bounds and a deduplicated [`MaterialSetId`] (the regions inside).
//!   It is built once, is read-only, and is part of the cache image.
//! * **The caller owns physics**: a [`MajorantTable`] of `(set, energy band) -> (sigma_max,
//!   sigma_mean)`, built from the caller's cross sections (unionized grids, URR maxima, TMS
//!   temperature bounds, density multipliers). The tracker never sees a nuclide or a material.
//!   The table is immutable and shared by all threads; rebuild it when cross sections change
//!   (it is keyed by `structure_hash`, so a stale table is a typed error, not wrong physics).
//! * **The caller owns randomness and the estimator**: the tracker takes a uniform number per
//!   flight (or a [`CollisionOracle`]) and never draws one itself.
//!
//! # One iterator, two modes
//! [`MajorantTracker::majorant_segments`] yields [`MajorantSegment`]s that partition the ray.
//! In [`TrackMode::Surface`] a segment is one cell and `sigma_m` is that cell's own total, so
//! nothing is rejected (`exact`). In [`TrackMode::Delta`] a segment is one node chord and
//! `sigma_m` is the node majorant. A set with a single region is also `exact` (a homogeneous
//! node is a region jump: one segment, no rejection). Switching mode between segments is
//! unbiased because every segment is an independent exponential race.
//!
//! # Estimators (what to score where)
//! Surface segments carry exact chords: use the **track-length** estimator on them (the optional
//! `track_sink` of [`MajorantTracker::next_tentative`] receives exactly those, truncated at the
//! tentative point). Delta segments carry no per-cell chords: use the **collision** estimator
//! at accepted collisions, and optionally at tentative ones through
//! [`CollisionOracle::tentative`] (Serpent's CFE scores a fraction of virtual collisions).
//! Mixing both in one tally is sound only when each particle scores each path piece with one
//! estimator; the mode of the piece is in the segment and in the [`TentativeHit`].
//!
//! # Threading
//! Everything on the tracker and the table is `&self`. Per thread, keep one [`FlightScratch`]
//! (it embeds [`Scratch`] and a stack, and carries deterministic work counters). Counters merge
//! with [`FlightCounters::merge`] in a fixed order; results never depend on thread count.
//!
//! # Draft C surface (two-phase, no callbacks, for OpenMC/Serpent-style loops)
//! ```text
//! typedef struct SmrtMajorant SmrtMajorant;   /* opaque, shared read-only */
//! int smrt_majorant_new(const SmrtTracker *, uint32_t n_bands, const double *edges /*n_bands+1*/,
//!                       const double *sigma_max /*n_sets*n_bands, set-major*/,
//!                       const double *sigma_mean, SmrtMajorant **out);
//! uint32_t smrt_n_sets(const SmrtTracker *);  int smrt_set_regions(const SmrtTracker *, uint32_t set,
//!                       uint32_t *out, uint32_t cap);   /* so the caller can compute its maxima */
//! void smrt_majorant_free(SmrtMajorant *);
//! typedef struct { double dist, x, y, z, sigma_m; uint32_t node, set, cell, region; int32_t mode, exact; } SmrtTentative;
//! /* xi in (0, 1]. 0: tentative collision (fill *out), 1: ended (`end` code as smrt_step), <0: error.
//!    The state is at the tentative point, located starting from the owning node. */
//! int smrt_next_tentative(SmrtCtx *, SmrtState *, const SmrtMajorant *, uint32_t band, double xi,
//!                         double max_dist, SmrtTentative *out, int32_t *end);
//! /* Caller computes sigma_t at the point, draws its own acceptance number, continues the flight
//!    with another smrt_next_tentative, or scores the real collision. */
//! ```

use crate::cache::{ContentHash, ContentHasher};
use crate::error::{Result, RtError};
use crate::ids::{CellId, FaceId, MaterialSetId, NodeId, RegionId};
use crate::state::{Located, Scratch, TrackState};
use crate::tally::{Flow, Segment, SegmentSink, TraceEnd, TraceOptions};
use crate::tracker::{Tracker, move_along};
use sm_core::{Aabb, Point3};

/// Index of an energy band (`0..EnergyBands::len()`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[repr(transparent)]
pub struct BandId(pub u16);

/// Energy band edges chosen by the caller (any unit; only ordering matters).
#[derive(Clone, Debug, PartialEq)]
pub struct EnergyBands {
    edges: Vec<f64>,
}

impl EnergyBands {
    /// Strictly increasing, finite, positive edges; at least two (one band).
    pub fn new(edges: Vec<f64>) -> Result<Self> {
        if edges.len() < 2 || edges.len() > usize::from(u16::MAX) {
            return Err(RtError::InvalidInput("energy bands need 2..=65535 edges"));
        }
        if !edges.iter().all(|e| e.is_finite() && *e > 0.0) || edges.windows(2).any(|w| w[1] <= w[0]) {
            return Err(RtError::InvalidInput("energy edges must be positive, finite and strictly increasing"));
        }
        Ok(Self { edges })
    }
    /// One band covering everything (grey or multigroup-collapsed use).
    pub fn single() -> Self {
        Self { edges: vec![f64::MIN_POSITIVE, f64::MAX] }
    }
    /// Number of bands.
    pub fn len(&self) -> usize {
        self.edges.len() - 1
    }
    /// Never empty (at least one band).
    pub fn is_empty(&self) -> bool {
        false
    }
    /// The edges.
    pub fn edges(&self) -> &[f64] {
        &self.edges
    }
    /// Band containing `e` (`[lo, hi)`, the top edge belongs to the last band); `None` outside.
    pub fn band_of(&self, e: f64) -> Option<BandId> {
        let (lo, hi) = (self.edges[0], self.edges[self.edges.len() - 1]);
        if !(e >= lo && e <= hi) {
            return None;
        }
        let i = self.edges.partition_point(|&x| x <= e).saturating_sub(1).min(self.len() - 1);
        Some(BandId(i as u16))
    }
}

/// What a partition node is (for diagnostics and the build-time cost model).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum NodeKind {
    /// The model root.
    Root,
    /// A lattice element.
    Lattice,
    /// An instance of a prototype.
    Instance,
    /// A kd/octree leaf over meshed regions.
    Meshed,
    /// An analytic primitive's region.
    Analytic,
}

/// One node of the disjoint partition. Build with [`NodeInfo::new`].
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct NodeInfo {
    /// Index; parents come first.
    pub id: NodeId,
    /// Parent, `None` for the root.
    pub parent: Option<NodeId>,
    /// World-frame bounds (for a transformed instance, the bounds of its image).
    pub bounds: Aabb,
    /// Regions inside the node.
    pub set: MaterialSetId,
    /// Kind.
    pub kind: NodeKind,
    /// Tracking cells inside (the cost of a surface-mode walk through the node).
    pub cells: u32,
    /// True if this node is a leaf of the partition: segments are emitted per leaf.
    pub leaf: bool,
    /// Build-time cost-model verdict: delta mode may pay off here (false for single-cell
    /// nodes, or nodes whose hinted locate costs more than the walk). Depends only on
    /// geometry, so it is deterministic. [`HybridPolicy`] combines it with the run-time test.
    pub delta_ok: bool,
}

impl NodeInfo {
    /// A leaf node with `delta_ok = cells > 1`.
    pub fn new(id: NodeId, parent: Option<NodeId>, bounds: Aabb, set: MaterialSetId, kind: NodeKind, cells: u32) -> Self {
        Self { id, parent, bounds, set, kind, cells, leaf: true, delta_ok: cells > 1 }
    }
}

/// The tracker-owned partition: nodes, region sets and the region to leaf-set map.
#[derive(Clone, Debug)]
pub struct MajorantStructure {
    nodes: Vec<NodeInfo>,
    set_offsets: Vec<u32>,
    set_regions: Vec<RegionId>,
    region_set: Vec<MaterialSetId>,
    hash: ContentHash,
}

impl MajorantStructure {
    /// Validate and freeze. `set_offsets` is CSR into `set_regions` (`n_sets + 1` entries);
    /// `region_set[r]` is the single-region set of region `r` (used for surface-mode segments,
    /// where the majorant is that cell's own total). Checks parent order, id ranges, and that
    /// each `region_set` entry is a set of exactly that region.
    pub fn new(nodes: Vec<NodeInfo>, set_offsets: Vec<u32>, set_regions: Vec<RegionId>, region_set: Vec<MaterialSetId>) -> Result<Self> {
        let bad = |m: &'static str| Err(RtError::InvalidInput(m));
        if set_offsets.first() != Some(&0) || set_offsets.windows(2).any(|w| w[1] < w[0]) || set_offsets.last().map(|&x| x as usize) != Some(set_regions.len()) {
            return bad("majorant set offsets are not a valid CSR");
        }
        let n_sets = set_offsets.len() - 1;
        for (i, n) in nodes.iter().enumerate() {
            if n.id.get() != i || n.parent.is_some_and(|p| p.get() >= i) || n.set.get() >= n_sets || n.bounds.is_empty() {
                return bad("majorant node ids, parents, sets and bounds must be consistent");
            }
        }
        if nodes.is_empty() || !nodes.iter().any(|n| n.leaf) {
            return bad("majorant structure needs a leaf node");
        }
        for (r, s) in region_set.iter().enumerate() {
            let ok = s.get() < n_sets && {
                let (a, b) = (set_offsets[s.get()] as usize, set_offsets[s.get() + 1] as usize);
                b == a + 1 && set_regions[a].get() == r
            };
            if !ok {
                return bad("region_set must map each region to its own single-region set");
            }
        }
        let mut h = ContentHasher::new();
        h.write_u64(nodes.len() as u64);
        for n in &nodes {
            h.write_u64(u64::from(n.id.0));
            h.write_u64(u64::from(n.parent.map_or(u32::MAX, |p| p.0)));
            for v in [n.bounds.lo.x, n.bounds.lo.y, n.bounds.lo.z, n.bounds.hi.x, n.bounds.hi.y, n.bounds.hi.z] {
                h.write_f64(v);
            }
            h.write_u64(u64::from(n.set.0) | u64::from(n.leaf) << 32 | u64::from(n.delta_ok) << 33);
            h.write_u64(u64::from(n.cells));
        }
        for &o in &set_offsets {
            h.write_u64(u64::from(o));
        }
        for r in &set_regions {
            h.write_u64(u64::from(r.0));
        }
        for s in &region_set {
            h.write_u64(u64::from(s.0));
        }
        Ok(Self { nodes, set_offsets, set_regions, region_set, hash: h.finish() })
    }
    /// All nodes, parents first.
    pub fn nodes(&self) -> &[NodeInfo] {
        &self.nodes
    }
    /// One node.
    pub fn node(&self, id: NodeId) -> Option<&NodeInfo> {
        self.nodes.get(id.get())
    }
    /// Number of material sets.
    pub fn n_sets(&self) -> usize {
        self.set_offsets.len() - 1
    }
    /// Regions of a set, ascending.
    pub fn regions_of(&self, set: MaterialSetId) -> &[RegionId] {
        match self.set_offsets.get(set.get()..=set.get() + 1) {
            Some(w) => &self.set_regions[w[0] as usize..w[1] as usize],
            None => &[],
        }
    }
    /// The single-region set of `region` (`None` for an unknown region).
    pub fn leaf_set(&self, region: RegionId) -> Option<MaterialSetId> {
        self.region_set.get(region.get()).copied()
    }
    /// Content hash (a [`MajorantTable`] is bound to it).
    pub fn content_hash(&self) -> ContentHash {
        self.hash
    }
}

/// Majorant and mean total cross section of one (set, band), caller-supplied.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct MajorantEntry {
    /// Upper bound of sigma_t over the set's regions and the band (inverse length).
    pub sigma_max: f64,
    /// Volume-weighted mean sigma_t; only feeds the rejection-probability test.
    pub sigma_mean: f64,
}

impl MajorantEntry {
    /// `0 <= sigma_mean <= sigma_max < inf`.
    pub fn new(sigma_max: f64, sigma_mean: f64) -> Result<Self> {
        if !(sigma_max.is_finite() && sigma_mean.is_finite() && sigma_mean >= 0.0 && sigma_mean <= sigma_max) {
            return Err(RtError::InvalidInput("majorant entry needs 0 <= sigma_mean <= sigma_max < inf"));
        }
        Ok(Self { sigma_max, sigma_mean })
    }
    /// Probability that a tentative collision is rejected: `1 - mean / max` (0 for vacuum).
    pub fn rejection_probability(&self) -> f64 {
        if self.sigma_max > 0.0 { 1.0 - self.sigma_mean / self.sigma_max } else { 0.0 }
    }
}

/// Caller-owned majorants, set-major `n_sets x n_bands`, bound to one structure.
#[derive(Clone, Debug)]
pub struct MajorantTable {
    bands: EnergyBands,
    n_sets: usize,
    entries: Vec<MajorantEntry>,
    structure: ContentHash,
    hash: ContentHash,
}

impl MajorantTable {
    /// Build from flat set-major arrays (the C ABI shape).
    pub fn from_flat(s: &MajorantStructure, bands: EnergyBands, sigma_max: &[f64], sigma_mean: &[f64]) -> Result<Self> {
        let n = s.n_sets() * bands.len();
        if sigma_max.len() != n || sigma_mean.len() != n {
            return Err(RtError::InvalidInput("majorant arrays must have n_sets * n_bands entries"));
        }
        let entries = sigma_max.iter().zip(sigma_mean).map(|(&a, &b)| MajorantEntry::new(a, b)).collect::<Result<Vec<_>>>()?;
        let mut h = ContentHasher::new();
        h.write_hash(s.content_hash());
        for e in bands.edges() {
            h.write_f64(*e);
        }
        for e in &entries {
            h.write_f64(e.sigma_max);
            h.write_f64(e.sigma_mean);
        }
        Ok(Self { bands, n_sets: s.n_sets(), entries, structure: s.content_hash(), hash: h.finish() })
    }
    /// Build by asking `f(regions_of_set, band)` for every (set, band), in set-major order.
    pub fn from_fn(s: &MajorantStructure, bands: EnergyBands, mut f: impl FnMut(&[RegionId], BandId) -> Result<MajorantEntry>) -> Result<Self> {
        let (mut mx, mut mn) = (Vec::new(), Vec::new());
        for set in 0..s.n_sets() {
            for b in 0..bands.len() {
                let e = f(s.regions_of(MaterialSetId::new(set)), BandId(b as u16))?;
                mx.push(e.sigma_max);
                mn.push(e.sigma_mean);
            }
        }
        Self::from_flat(s, bands, &mx, &mn)
    }
    /// The bands.
    pub fn bands(&self) -> &EnergyBands {
        &self.bands
    }
    /// Entry of a (set, band); `None` when out of range.
    pub fn entry(&self, set: MaterialSetId, band: BandId) -> Option<MajorantEntry> {
        (set.get() < self.n_sets && usize::from(band.0) < self.bands.len()).then(|| self.entries[set.get() * self.bands.len() + usize::from(band.0)])
    }
    /// Err unless the table was built for `s`.
    pub fn check_for(&self, s: &MajorantStructure) -> Result<()> {
        if self.structure == s.content_hash() { Ok(()) } else { Err(RtError::CacheStale("majorant table was built for a different structure")) }
    }
    /// Content hash (structure, edges, entries): the key for reproducibility records.
    pub fn content_hash(&self) -> ContentHash {
        self.hash
    }
    /// Verify a caller cross section against the majorant. `sigma_t > sigma_max` (beyond a
    /// `1e-12` relative slack for rounding) is [`RtError::MajorantViolated`]; for a `tight`
    /// (single-region) set `sigma_t` must also not be below it by more than the slack.
    pub fn check_sigma(&self, set: MaterialSetId, band: BandId, sigma_t: f64, tight: bool) -> Result<()> {
        let e = self.entry(set, band).ok_or(RtError::UnknownId { kind: "material set", id: set.0 })?;
        let slack = 1e-12 * e.sigma_max.max(f64::MIN_POSITIVE);
        let bad = !sigma_t.is_finite() || sigma_t < 0.0 || sigma_t > e.sigma_max + slack || (tight && sigma_t < e.sigma_max - slack);
        if bad { Err(RtError::MajorantViolated { set: set.0, band: band.0, sigma_t, sigma_max: e.sigma_max }) } else { Ok(()) }
    }
}

/// How a node is tracked.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackMode {
    /// One segment per cell, `sigma_m` = the cell's own total: exact, nothing rejected.
    Surface,
    /// One segment per node chord with the node majorant: rejection sampling.
    Delta,
}

/// Per-(node, band) mode choice; depends only on (node, entry), never on the particle history.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct HybridPolicy {
    /// Serpent's `dt`: delta tracking iff the rejection probability is `<=` this (0.9 default;
    /// 0 = never, 1 = always).
    pub dt_threshold: f64,
    /// Override both tests (not `blocked`): `Some(Surface)` for MOC/Sn or mesh tallies,
    /// `Some(Delta)` for tests.
    pub force: Option<TrackMode>,
    /// Nodes where delta tracking is forbidden (Serpent's `blockdt`), kept sorted.
    pub blocked: Vec<NodeId>,
}

impl Default for HybridPolicy {
    fn default() -> Self {
        Self { dt_threshold: 0.9, force: None, blocked: Vec::new() }
    }
}

impl HybridPolicy {
    /// Always surface tracking (MOC, Sn, mesh tallies).
    pub fn surface_only() -> Self {
        Self { force: Some(TrackMode::Surface), ..Self::default() }
    }
    /// Delta wherever allowed (tests).
    pub fn delta_only() -> Self {
        Self { force: Some(TrackMode::Delta), ..Self::default() }
    }
    /// Set the Serpent `dt` rejection threshold (`0` never, `1` always delta).
    pub fn with_threshold(mut self, dt: f64) -> Self {
        self.dt_threshold = dt;
        self
    }
    /// Forbid delta tracking in `node`.
    pub fn block(mut self, node: NodeId) -> Self {
        if let Err(i) = self.blocked.binary_search(&node) {
            self.blocked.insert(i, node);
        }
        self
    }
    /// The mode for `node` given the majorant entry of its set and band.
    pub fn mode_for(&self, node: &NodeInfo, e: MajorantEntry) -> TrackMode {
        if self.blocked.binary_search(&node.id).is_ok() {
            return TrackMode::Surface;
        }
        match self.force {
            Some(m) => m,
            None if node.delta_ok && e.rejection_probability() <= self.dt_threshold => TrackMode::Delta,
            None => TrackMode::Surface,
        }
    }
}

/// One piece of the ray. Segments are contiguous: `t0` of the next is `t0 + length` of this.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct MajorantSegment {
    /// Leaf node of the partition.
    pub node: NodeId,
    /// Mode this piece was tracked in.
    pub mode: TrackMode,
    /// The cell (surface mode); `CellId::NONE` in delta mode, where the cell is unknown until a
    /// tentative collision is located.
    pub cell: CellId,
    /// The region (surface mode); `RegionId::NONE` in delta mode.
    pub region: RegionId,
    /// Material set whose majorant applies (node set in delta mode, the cell's leaf set in
    /// surface mode).
    pub set: MaterialSetId,
    /// Ray parameter at entry, from the start of the trace.
    pub t0: f64,
    /// Length of the piece.
    pub length: f64,
    /// Majorant over the piece (inverse length).
    pub sigma_m: f64,
    /// True when `sigma_m` is the exact total (surface mode, or a single-region set): no
    /// rejection can occur.
    pub exact: bool,
    /// Face entered through (surface mode).
    pub face_in: Option<FaceId>,
    /// Face left through (surface mode).
    pub face_out: Option<FaceId>,
}

impl MajorantSegment {
    /// A segment without faces; set them directly.
    #[allow(clippy::too_many_arguments)]
    pub fn new(node: NodeId, mode: TrackMode, cell: CellId, region: RegionId, set: MaterialSetId, t0: f64, length: f64, sigma_m: f64, exact: bool) -> Self {
        Self { node, mode, cell, region, set, t0, length, sigma_m, exact, face_in: None, face_out: None }
    }
}

/// Receives majorant segments in ray order. Return [`Flow::Stop`] to end the trace with the
/// state at the end of that segment.
pub trait MajorantSink {
    /// Called once per segment.
    fn segment(&mut self, seg: &MajorantSegment) -> Flow;
}

impl<F: FnMut(&MajorantSegment) -> Flow> MajorantSink for F {
    fn segment(&mut self, seg: &MajorantSegment) -> Flow {
        self(seg)
    }
}

/// Deterministic work counters (the only performance numbers allowed in gates).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FlightCounters {
    /// Surface-mode segments emitted.
    pub surface_segments: u64,
    /// Delta-mode segments emitted.
    pub delta_segments: u64,
    /// Tentative collisions produced.
    pub tentative: u64,
    /// Tentative collisions rejected.
    pub rejected: u64,
    /// Hinted point locations (one per delta tentative collision).
    pub locates: u64,
    /// Nodes visited by the partition traversal.
    pub node_visits: u64,
}

impl FlightCounters {
    /// Add `o` into `self` (merge per-thread counters in thread-index order).
    pub fn merge(&mut self, o: &Self) {
        self.surface_segments += o.surface_segments;
        self.delta_segments += o.delta_segments;
        self.tentative += o.tentative;
        self.rejected += o.rejected;
        self.locates += o.locates;
        self.node_visits += o.node_visits;
    }
}

/// Per-thread working memory for flights: `base` serves every `Tracker` query; `stack` is the
/// partition traversal stack. Same no-allocation-after-warm-up contract as [`Scratch`].
#[derive(Debug, Default)]
pub struct FlightScratch {
    /// Scratch for the underlying tracker queries.
    pub base: Scratch,
    /// Traversal stack (node ids).
    pub stack: Vec<u32>,
    /// Work counters.
    pub counters: FlightCounters,
}

impl FlightScratch {
    /// Empty.
    pub fn new() -> Self {
        Self::default()
    }
}

/// A tentative collision: the particle state has been moved to `pos` and located.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct TentativeHit {
    /// Distance flown by this call.
    pub dist: f64,
    /// Position.
    pub pos: Point3,
    /// Leaf node that owned the majorant.
    pub node: NodeId,
    /// Cell found at `pos` (located from the node, not from the root).
    pub cell: CellId,
    /// Its region: look up sigma_t with this.
    pub region: RegionId,
    /// Set whose majorant was used.
    pub set: MaterialSetId,
    /// The majorant used.
    pub sigma_m: f64,
    /// True when no rejection can occur.
    pub exact: bool,
    /// Mode of the piece the collision fell in (selects the estimator).
    pub mode: TrackMode,
}

/// Result of [`MajorantTracker::next_tentative`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Flight {
    /// Tentative collision (state at the point).
    Tentative(TentativeHit),
    /// The flight ended first: limit, boundary needing `resolve_boundary`, leakage or loss.
    /// `dist` is the distance flown; the state is at the end point.
    End {
        /// Why.
        end: TraceEnd,
        /// Distance flown by this call.
        dist: f64,
    },
}

/// Result of [`MajorantTracker::fly`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FlightOutcome {
    /// An accepted (real) collision.
    Collision {
        /// The collision (`dist` is the total from the start of `fly`).
        hit: TentativeHit,
        /// Tentative collisions rejected before it.
        rejected: u32,
    },
    /// The flight ended without a real collision.
    End {
        /// Why.
        end: TraceEnd,
        /// Total distance flown.
        dist: f64,
        /// Tentative collisions rejected on the way.
        rejected: u32,
    },
}

/// The caller's side of a flight: random numbers and cross sections.
pub trait CollisionOracle {
    /// Next uniform number in `[0, 1)`; the tracker draws `1 - u` for distances and `u` for
    /// acceptance, one per call, in a fixed order.
    fn uniform(&mut self) -> f64;
    /// Total cross section at the tentative point (region, position, the caller's energy).
    fn sigma_t(&mut self, at: &TentativeHit) -> f64;
    /// Called for every tentative collision after the decision (virtual-collision scoring).
    fn tentative(&mut self, _at: &TentativeHit, _sigma_t: f64, _accepted: bool) {}
}

/// Everything a flight needs besides the particle: band, table, policy and trace limits.
/// Cheap to copy; build one per (particle energy band, table) and reuse it.
#[derive(Clone, Copy, Debug)]
pub struct FlightConfig<'a> {
    /// Energy band of the particle (`EnergyBands::band_of`); re-derive after an energy change.
    pub band: BandId,
    /// The caller's majorants.
    pub table: &'a MajorantTable,
    /// Mode choice.
    pub policy: &'a HybridPolicy,
    /// Distance and step limits.
    pub opts: TraceOptions,
}

impl<'a> FlightConfig<'a> {
    /// Default limits (no distance limit).
    pub fn new(band: BandId, table: &'a MajorantTable, policy: &'a HybridPolicy) -> Self {
        Self { band, table, policy, opts: TraceOptions::default() }
    }
    /// With other limits.
    pub fn with_opts(mut self, opts: TraceOptions) -> Self {
        self.opts = opts;
        self
    }
}

/// A tracker that also supports segment-wise delta tracking. Required methods give the
/// partition walk and hinted location; flights are provided on top.
pub trait MajorantTracker: Tracker {
    /// The disjoint partition (built with the tracker, cached with it).
    fn majorant_structure(&self) -> &MajorantStructure;

    /// The segment iterator. Walks the ray from `st` and emits [`MajorantSegment`]s that
    /// partition it, honouring decks as [`Tracker::trace`] does, until the limit, a `Respond`
    /// boundary, loss, or the sink stops. Per leaf, `policy.mode_for` decides the mode.
    /// After `Stopped` the state is at the end of the last segment; in a surface segment its
    /// cell is the post-step cell, in a delta segment its cell is stale until
    /// [`Self::locate_in_node`] is called. Mode changes between segments are unbiased. Never
    /// allocates after warm-up.
    fn majorant_segments(
        &self,
        st: &mut TrackState,
        cfg: &FlightConfig<'_>,
        sink: &mut dyn MajorantSink,
        fs: &mut FlightScratch,
    ) -> Result<TraceEnd>;

    /// Hinted point location: start at `node` (O(1) lattice index, radial binary search, a
    /// walk or small BVH inside a meshed leaf) and never at the root. `dir` resolves a point on
    /// a face as in [`Tracker::locate`]. `p` outside the node's bounds is an error.
    fn locate_in_node(&self, node: NodeId, p: Point3, dir: Option<sm_core::Vector3>, fs: &mut FlightScratch) -> Result<Located>;

    /// Fly until the first tentative collision. `xi` is in `(0, 1]` and the optical depth is
    /// `-ln xi`, consumed across segments with their own `sigma_m` (exact for varying
    /// majorants). `track_sink`, if given, receives the surface-mode segments flown, the last
    /// one truncated at the collision, for the track-length estimator. On `Tentative` the state
    /// is moved to the point and located from the owning node (surface mode: from the cell).
    fn next_tentative(
        &self,
        st: &mut TrackState,
        cfg: &FlightConfig<'_>,
        xi: f64,
        mut track_sink: Option<&mut dyn SegmentSink>,
        fs: &mut FlightScratch,
    ) -> Result<Flight> {
        if !(xi > 0.0 && xi <= 1.0) {
            return Err(RtError::InvalidInput("xi must be in (0, 1]"));
        }
        cfg.table.check_for(self.majorant_structure())?;
        let mut tau = -xi.ln();
        let mut hit: Option<(MajorantSegment, f64)> = None;
        let mut flown = 0.0;
        let end = self.majorant_segments(
            st,
            cfg,
            &mut |s: &MajorantSegment| {
                let od = s.sigma_m * s.length;
                let found = s.sigma_m > 0.0 && tau <= od;
                let here = if found { tau / s.sigma_m } else { s.length };
                if s.mode == TrackMode::Surface
                    && let Some(sink) = track_sink.as_deref_mut()
                {
                    let seg = Segment::new(s.cell, s.region, s.t0, here).with_faces(s.face_in, if found { None } else { s.face_out });
                    if sink.segment(&seg) == Flow::Stop && !found {
                        // The track sink may end the flight (reported as `Stopped`).
                        flown = s.t0 + here;
                        return Flow::Stop;
                    }
                }
                if found {
                    hit = Some((*s, here));
                    flown = s.t0 + here;
                    return Flow::Stop;
                }
                tau -= od;
                flown = s.t0 + s.length;
                Flow::Continue
            },
            fs,
        )?;
        let Some((seg, here)) = hit else {
            return Ok(Flight::End { end, dist: flown });
        };
        // Back up from the end of the segment to the collision point (same direction).
        move_along(st, -(seg.length - here));
        let loc = match seg.mode {
            TrackMode::Delta => {
                fs.counters.locates += 1;
                self.locate_in_node(seg.node, st.pos, Some(st.dir), fs)?
            }
            TrackMode::Surface => self.locate(st.pos, Some(st.dir), Some(seg.cell), &mut fs.base)?,
        };
        st.cell = loc.cell;
        st.region = loc.region;
        st.entered_via = loc.on_face;
        st.levels = loc.levels;
        fs.counters.tentative += 1;
        Ok(Flight::Tentative(TentativeHit {
            dist: seg.t0 + here,
            pos: st.pos,
            node: seg.node,
            cell: st.cell,
            region: st.region,
            set: seg.set,
            sigma_m: seg.sigma_m,
            exact: seg.exact,
            mode: seg.mode,
        }))
    }

    /// Complete flight to the first real collision or the end of the flight: repeats
    /// [`Self::next_tentative`], asks the oracle for sigma_t, verifies it against the table
    /// (a violation is an error, never clamped), accepts with probability `sigma_t / sigma_m`
    /// (always, without drawing, when `exact`). A rejected collision continues from the
    /// tentative point with a fresh optical depth.
    fn fly(
        &self,
        st: &mut TrackState,
        cfg: &FlightConfig<'_>,
        oracle: &mut dyn CollisionOracle,
        mut track_sink: Option<&mut dyn SegmentSink>,
        fs: &mut FlightScratch,
    ) -> Result<FlightOutcome> {
        let (mut total, mut rejected) = (0.0, 0u32);
        let mut c = *cfg;
        for _ in 0..cfg.opts.max_steps {
            let xi = 1.0 - oracle.uniform();
            match self.next_tentative(st, &c, xi, track_sink.as_mut().map(|s| &mut **s as &mut dyn SegmentSink), fs)? {
                Flight::End { end, dist } => return Ok(FlightOutcome::End { end, dist: total + dist, rejected }),
                Flight::Tentative(mut hit) => {
                    total += hit.dist;
                    hit.dist = total;
                    let sigma_t = oracle.sigma_t(&hit);
                    cfg.table.check_sigma(hit.set, cfg.band, sigma_t, hit.exact)?;
                    let accepted = hit.exact || oracle.uniform() * hit.sigma_m < sigma_t;
                    oracle.tentative(&hit, sigma_t, accepted);
                    if accepted {
                        return Ok(FlightOutcome::Collision { hit, rejected });
                    }
                    rejected += 1;
                    fs.counters.rejected += 1;
                    c.opts.max_dist = cfg.opts.max_dist - total;
                }
            }
        }
        Err(RtError::InvalidInput("flight exceeded max_steps tentative collisions"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bx() -> Aabb {
        Aabb { lo: Point3::new(0.0, 0.0, 0.0), hi: Point3::new(1.0, 1.0, 1.0) }
    }
    fn structure() -> MajorantStructure {
        let node = NodeInfo::new(NodeId(0), None, bx(), MaterialSetId(0), NodeKind::Root, 2);
        MajorantStructure::new(
            vec![node],
            vec![0, 2, 3, 4],
            vec![RegionId(0), RegionId(1), RegionId(0), RegionId(1)],
            vec![MaterialSetId(1), MaterialSetId(2)],
        )
        .unwrap()
    }

    #[test]
    fn band_lookup_and_validation() {
        let b = EnergyBands::new(vec![1.0, 2.0, 4.0]).unwrap();
        assert_eq!((b.band_of(1.0), b.band_of(1.99), b.band_of(2.0), b.band_of(4.0)), (Some(BandId(0)), Some(BandId(0)), Some(BandId(1)), Some(BandId(1))));
        assert_eq!((b.band_of(0.5), b.band_of(4.1), b.band_of(f64::NAN)), (None, None, None));
        assert!(EnergyBands::new(vec![1.0]).is_err() && EnergyBands::new(vec![2.0, 1.0]).is_err() && EnergyBands::new(vec![0.0, 1.0]).is_err());
        assert_eq!(EnergyBands::single().band_of(1e-5), Some(BandId(0)));
    }

    #[test]
    fn structure_rejects_inconsistent_input() {
        let node = NodeInfo::new(NodeId(0), None, bx(), MaterialSetId(0), NodeKind::Root, 2);
        // region_set pointing at a multi-region set.
        let r = MajorantStructure::new(vec![node.clone()], vec![0, 2, 3, 4], vec![RegionId(0), RegionId(1), RegionId(0), RegionId(1)], vec![MaterialSetId(0), MaterialSetId(2)]);
        assert!(r.is_err());
        let mut bad = node;
        bad.parent = Some(NodeId(0));
        assert!(MajorantStructure::new(vec![bad], vec![0, 1], vec![RegionId(0)], vec![MaterialSetId(0)]).is_err());
    }

    #[test]
    fn table_is_bound_to_its_structure_and_checks_sigma() {
        let s = structure();
        let t = MajorantTable::from_fn(&s, EnergyBands::single(), |regs, _| {
            let m = regs.iter().map(|r| [1.0, 3.0][r.get()]).fold(0.0, f64::max);
            MajorantEntry::new(m, 0.5 * m)
        })
        .unwrap();
        t.check_for(&s).unwrap();
        assert_eq!(t.entry(MaterialSetId(0), BandId(0)).unwrap().sigma_max, 3.0);
        assert!(t.check_sigma(MaterialSetId(0), BandId(0), 3.0, false).is_ok());
        assert!(matches!(t.check_sigma(MaterialSetId(0), BandId(0), 3.1, false), Err(RtError::MajorantViolated { .. })));
        assert!(t.check_sigma(MaterialSetId(1), BandId(0), 0.5, true).is_err(), "tight set must match");
        assert!(MajorantTable::from_flat(&s, EnergyBands::single(), &[1.0], &[1.0]).is_err());
        assert!(MajorantEntry::new(1.0, 2.0).is_err());
    }

    #[test]
    fn policy_follows_serpent_threshold_and_blocking() {
        let s = structure();
        let n = &s.nodes()[0];
        let lo = MajorantEntry::new(2.0, 1.0).unwrap(); // p_rej 0.5
        let hi = MajorantEntry::new(10.0, 0.5).unwrap(); // p_rej 0.95
        let p = HybridPolicy::default();
        assert_eq!((p.mode_for(n, lo), p.mode_for(n, hi)), (TrackMode::Delta, TrackMode::Surface));
        assert_eq!(p.clone().with_threshold(1.0).mode_for(n, hi), TrackMode::Delta);
        assert_eq!(HybridPolicy::delta_only().block(NodeId(0)).mode_for(n, lo), TrackMode::Surface);
        assert_eq!(HybridPolicy::surface_only().mode_for(n, lo), TrackMode::Surface);
    }
}
