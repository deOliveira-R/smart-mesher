//! The majorant hierarchy: the one structure behind surface tracking, delta tracking and
//! hybrid tracking (additive extension for W3b).
//!
//! # Thesis
//! A **disjoint partition of space** (lattice and universe nodes, a kd split over meshed
//! regions) is the centre. Every node has bounds, a deduplicated [`MaterialSetId`] (the
//! materials of everything below it) and, through a [`MajorantTable`], a majorant per
//! [`BandId`]. Tracking is a *view* over it:
//!
//! * **Delta view**: a ray marches the partition by DDA and yields [`MajorantSegment`]s
//!   `{t0, t1, node, set}` that tile the ray. The caller samples with `Σ_M(set, band)` and rejects
//!   tentative collisions after a **hinted** [`MajorantHierarchy::locate_in`] that starts at the
//!   node that owns the majorant, never at the root.
//! * **Surface view**: the same march, with the policy refining every node to its cells, yields
//!   one segment per cell chord with [`SegmentMode::Surface`] and the cell's own singleton set.
//!   Nothing is rejected; the caller uses the exact `Σ_t`. This is today's `Tracker::trace`
//!   expressed as a segment stream.
//! * **Hybrid view**: [`RefinePolicy::PerNode`] reads the build-time cost-model decision of
//!   [`MajorantTable`] per `(node, band)`; [`RefinePolicy::Threshold`] is Serpent's `dt` rule on
//!   the node's bound. Switching mode inside one flight is unbiased because both views sample the
//!   same `Σ_t`: tracking is memoryless, so each segment may use either.
//!
//! The overlapping BVH of the tracker stays the index for unhinted point location. It cannot
//! produce majorant segments, since segments must partition the ray.
//!
//! # Contract summary (binding on implementors)
//! 1. **Tiling.** The children of a node tile its bounds: pairwise disjoint interiors whose union
//!    is the parent's extent. Gaps are filled by an explicit filler child. [`NodeTable::validate`]
//!    checks this for `Extent::Box` nodes.
//! 2. **Set containment.** A node's set contains the material of every cell below it, so
//!    `Σ_M(node.set, band) >= Σ_t(material, E)` for all `E` in the band. The table builds this
//!    with `max`/`min`, which are order-independent, so the table is bit-identical for any thread
//!    count. A violation is [`RtError::MajorantViolation`], never clamped.
//! 3. **Segments tile the ray.** For one march, `next` yields segments with `t0` of the first
//!    equal to the start (0), `t1` of each equal to the `t0` of the next *bitwise*, and the last
//!    `t1` equal to the end distance (`opts.max_dist` for a `Limit` end, the exit distance for a
//!    boundary end). No segment has negative length. Zero-length segments are not emitted.
//! 4. **Surface segments are exact.** A `Surface` segment lies in one cell, carries that cell and
//!    its region, and its set is the singleton set of the cell's material. A `Delta` segment has
//!    `cell == CellId::NONE`.
//! 5. **Hinted location.** `locate_in(node, p, dir)` answers the cell of `p` among the cells
//!    below `node`, in O(1) for lattices and O(log) for kd and radial structures plus a local
//!    walk or small BVH in a meshed leaf. A point outside the node's extent gives
//!    `CellId::NONE`; it never falls back to the root. A point within `tol.linear` of the extent
//!    counts as inside, and on a wall shared with a sibling the node's own side wins over the
//!    direction, so a collision sampled at the very end of a segment still locates in it.
//! 6. **Determinism.** Same inputs, same segments, same bits; no dependence on threads, time or
//!    allocation order. Ties on a wall go to the lowest axis; all walls reached at the same `t`
//!    are crossed together.
//! 7. **Estimators.** `Surface` segments support the track-length estimator; `Delta` segments
//!    support the collision estimator only ([`MajorantSegment::estimator`]). A tally that mixes
//!    both scores each segment by its own estimator; the mixed result is unbiased because each
//!    segment's score is unbiased for its own interval.
//!
//! # Driving it (Monte Carlo)
//! ```text
//! let h = tracker.majorant_hierarchy(&materials, &HierarchyOptions::default())?;
//! let table = MajorantTable::build(&*h, &bands, &xs_bound, &CostModel::default())?;   // once
//! let band = bands.band_of(energy)?;
//! let opts = MarchOptions::new(max_dist, band).with_refine(RefinePolicy::PerNode);
//! match sample_flight(&*h, &state, &opts, &table, &sigma_t_at_energy, &mut rng, &mut score,
//!                     &mut stats, &mut scratch)? {
//!     FlightEnd::Collision { state, dist } => { /* real collision; `state` is located */ }
//!     FlightEnd::Ended(MarchEnd::Boundary { .. }) => { let st = h.finish(..)?; /* resolve BC */ }
//!     FlightEnd::Ended(_) => { /* limit or lost */ }
//! }
//! ```
//! MOC and Sn keep `Tracker::trace_bundle` (cell chords, no Monte Carlo). A mesh tally that needs
//! per-cell track lengths sets `RefinePolicy::Cells`, which is exactly that cell walk.

use crate::boundary::FaceAction;
use crate::cache::{ContentHash, ContentHasher};
use crate::error::{BuildIssue, LostInfo, Result, RtError};
use crate::ids::{BandId, CellId, FaceId, MaterialSetId, NodeId, RegionId, SurfaceId};
use crate::state::{LEVEL_INLINE, Located, Scratch, TrackState};
use crate::tally::Flow;
use sm_core::{Aabb, Point3, Vector3};
use smallvec::SmallVec;
use std::collections::BTreeMap;

fn at(p: Point3, a: usize) -> f64 {
    [p.x, p.y, p.z][a]
}

// ---------------------------------------------------------------------------------------------
// Energy bands, materials, material sets
// ---------------------------------------------------------------------------------------------

/// Energy bands: `n` bands from `n + 1` ascending edges in eV. A majorant is per band.
#[derive(Clone, Debug, PartialEq)]
pub struct EnergyBands {
    edges: Vec<f64>,
}

impl EnergyBands {
    /// Bands from ascending edges (eV). Needs at least two edges, the first `>= 0`, all
    /// strictly ascending, only the last may be `+inf`.
    pub fn new(edges: Vec<f64>) -> Result<Self> {
        if edges.len() < 2 {
            return Err(RtError::InvalidInput("energy bands need at least two edges"));
        }
        let ok = edges[0] >= 0.0
            && edges[0].is_finite()
            && edges.windows(2).all(|w| w[0] < w[1])
            && edges[..edges.len() - 1].iter().all(|e| e.is_finite())
            && !edges[edges.len() - 1].is_nan();
        if !ok {
            return Err(RtError::InvalidInput("energy band edges must be ascending, non-negative, finite except the last"));
        }
        Ok(Self { edges })
    }

    /// One band over `[0, inf)`: multigroup codes with their own group index, MOC, tests.
    pub fn single() -> Self {
        Self { edges: vec![0.0, f64::INFINITY] }
    }

    /// Number of bands.
    pub fn len(&self) -> usize {
        self.edges.len() - 1
    }

    /// Always false (at least one band).
    pub fn is_empty(&self) -> bool {
        false
    }

    /// The edges.
    pub fn edges(&self) -> &[f64] {
        &self.edges
    }

    /// Band of energy `e`: lower edge inclusive, upper exclusive, the last band includes its
    /// upper edge. `None` outside the range or for NaN.
    pub fn band_of(&self, e: f64) -> Option<BandId> {
        let (lo, hi) = (self.edges[0], self.edges[self.edges.len() - 1]);
        if e.is_nan() || e < lo || e > hi {
            return None;
        }
        let k = self.edges.partition_point(|&x| x <= e);
        Some(BandId::new(k.saturating_sub(1).min(self.len() - 1)))
    }

    /// `(lo, hi)` of band `b`.
    pub fn bounds(&self, b: BandId) -> Option<(f64, f64)> {
        (b.get() < self.len()).then(|| (self.edges[b.get()], self.edges[b.get() + 1]))
    }

    fn hash_into(&self, h: &mut ContentHasher) {
        h.write_u64(self.edges.len() as u64);
        self.edges.iter().for_each(|&e| h.write_f64(e));
    }
}

/// Region to material key, supplied by the caller (the tracker knows regions, the transport code
/// knows materials). Equal keys share majorants and deduplicate in [`MaterialSets`].
#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub struct MaterialMap {
    by_region: Vec<u32>,
}

impl MaterialMap {
    /// Material key per region, indexed by `RegionId`.
    pub fn new(by_region: Vec<u32>) -> Self {
        Self { by_region }
    }
    /// Region `i` is material `i`.
    pub fn identity(n_regions: usize) -> Self {
        Self { by_region: (0..n_regions as u32).collect() }
    }
    /// Number of regions covered.
    pub fn len(&self) -> usize {
        self.by_region.len()
    }
    /// True if no region is covered.
    pub fn is_empty(&self) -> bool {
        self.by_region.is_empty()
    }
    /// Material key of `r` (`None` for `NONE` or out of range).
    pub fn material_of(&self, r: RegionId) -> Option<u32> {
        self.by_region.get(r.get()).copied().filter(|_| !r.is_none())
    }
    /// Content hash of the map (part of the hierarchy's cache identity).
    pub fn content_hash(&self) -> ContentHash {
        let mut h = ContentHasher::new();
        self.by_region.iter().for_each(|&m| h.write_u64(u64::from(m)));
        h.finish()
    }
}

/// Deduplicated sets of material keys, CSR layout. Members are sorted and unique, so equal sets
/// intern to one id.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct MaterialSets {
    offsets: Vec<u32>,
    members: Vec<u32>,
    index: BTreeMap<Vec<u32>, MaterialSetId>,
}

impl MaterialSets {
    /// Empty collection.
    pub fn new() -> Self {
        Self::default()
    }

    /// The id of the set with these members, adding it if new. Order and repeats are ignored.
    pub fn intern(&mut self, members: &[u32]) -> MaterialSetId {
        let mut m = members.to_vec();
        m.sort_unstable();
        m.dedup();
        if let Some(&id) = self.index.get(&m) {
            return id;
        }
        if self.offsets.is_empty() {
            self.offsets.push(0);
        }
        let id = MaterialSetId::new(self.offsets.len() - 1);
        self.members.extend_from_slice(&m);
        self.offsets.push(self.members.len() as u32);
        self.index.insert(m, id);
        id
    }

    /// Existing id of the set with these members.
    pub fn find(&self, members: &[u32]) -> Option<MaterialSetId> {
        let mut m = members.to_vec();
        m.sort_unstable();
        m.dedup();
        self.index.get(&m).copied()
    }

    /// Number of sets.
    pub fn len(&self) -> usize {
        self.offsets.len().saturating_sub(1)
    }

    /// True if there are no sets.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Sorted members of set `s`.
    pub fn get(&self, s: MaterialSetId) -> Option<&[u32]> {
        (s.get() < self.len()).then(|| &self.members[self.offsets[s.get()] as usize..self.offsets[s.get() + 1] as usize])
    }

    /// True if set `s` contains `material`.
    pub fn contains(&self, s: MaterialSetId, material: u32) -> bool {
        self.get(s).is_some_and(|m| m.binary_search(&material).is_ok())
    }

    /// Content hash.
    pub fn content_hash(&self) -> ContentHash {
        let mut h = ContentHasher::new();
        h.write_u64(self.len() as u64);
        for s in 0..self.len() {
            let m = self.get(MaterialSetId::new(s)).unwrap_or(&[]);
            h.write_u64(m.len() as u64);
            m.iter().for_each(|&x| h.write_u64(u64::from(x)));
        }
        h.finish()
    }
}

// ---------------------------------------------------------------------------------------------
// Nodes
// ---------------------------------------------------------------------------------------------

/// How a node's extent relates to its `bounds`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Extent {
    /// The node *is* its axis-aligned box. Children tile it; DDA applies.
    Box,
    /// The node is an exact region (a meshed region, a rotated instance) and `bounds` is only a
    /// bound. It is a leaf for majorant purposes; leaving it is an exact boundary query by the
    /// implementor (the one-jump "region jump"), and siblings are disjoint as true shapes.
    Exact,
}

/// What a node is and how its children are addressed.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum NodeKind {
    /// No children. Refining a leaf means walking its cells with the tracker.
    Leaf,
    /// Regular rectangular grid; children are contiguous from `first_child`, x fastest. The
    /// element of a point is arithmetic (O(1)) and DDA steps one element at a time.
    Lattice {
        /// Lower corner of element `(0, 0, 0)`.
        origin: Point3,
        /// Pitch per axis.
        pitch: [f64; 3],
        /// Element counts per axis.
        dims: [u32; 3],
    },
    /// kd split: two children, low then high side of `plane` on `axis` (0, 1, 2).
    Split {
        /// Split axis.
        axis: u8,
        /// Split coordinate.
        plane: f64,
    },
    /// Generic box tiling with `n_children` children (universe of placed components plus a
    /// filler); traversal tests the child boxes.
    Partition,
}

/// One node of the hierarchy. Build with [`NodeInfo::new`] and the `with_*` setters.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct NodeInfo {
    /// Kind and child addressing.
    pub kind: NodeKind,
    /// Bounds (the extent when `extent == Box`, a bound otherwise).
    pub bounds: Aabb,
    /// Box or exact extent.
    pub extent: Extent,
    /// Parent (`NodeId::NONE` for the root).
    pub parent: NodeId,
    /// First child (`NONE` if none); children are `first_child .. first_child + n_children`.
    pub first_child: NodeId,
    /// Number of children.
    pub n_children: u32,
    /// Materials of everything below this node.
    pub set: MaterialSetId,
    /// Cells below this node (the cost model's `n^(1/3)` crossings per chord).
    pub n_cells: u32,
    /// Exact volume of the extent when known.
    pub volume: Option<f64>,
}

impl NodeInfo {
    /// A box leaf with `n_cells` cells and material set `set`; no parent yet.
    pub fn new(bounds: Aabb, set: MaterialSetId, n_cells: u32) -> Self {
        Self {
            kind: NodeKind::Leaf,
            bounds,
            extent: Extent::Box,
            parent: NodeId::NONE,
            first_child: NodeId::NONE,
            n_children: 0,
            set,
            n_cells,
            volume: None,
        }
    }
    /// Set the kind.
    pub fn with_kind(mut self, kind: NodeKind) -> Self {
        self.kind = kind;
        self
    }
    /// Set the extent.
    pub fn with_extent(mut self, extent: Extent) -> Self {
        self.extent = extent;
        self
    }
    /// Set the parent.
    pub fn with_parent(mut self, parent: NodeId) -> Self {
        self.parent = parent;
        self
    }
    /// Set the children range.
    pub fn with_children(mut self, first: NodeId, n: u32) -> Self {
        self.first_child = first;
        self.n_children = n;
        self
    }
    /// Set the exact volume.
    pub fn with_volume(mut self, v: f64) -> Self {
        self.volume = Some(v);
        self
    }
    /// Volume of the extent: the exact one, else the bounds' box (a bound for `Exact`).
    pub fn volume_estimate(&self) -> f64 {
        self.volume.unwrap_or_else(|| {
            let d = self.bounds.hi - self.bounds.lo;
            (d.x * d.y * d.z).max(0.0)
        })
    }
}

/// All nodes; node 0 is the root.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct NodeTable(pub Vec<NodeInfo>);

impl NodeTable {
    /// Node `n`.
    pub fn get(&self, n: NodeId) -> Option<&NodeInfo> {
        self.0.get(n.get())
    }
    /// Number of nodes.
    pub fn len(&self) -> usize {
        self.0.len()
    }
    /// True if there are no nodes.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
    /// Children of `n` in order.
    pub fn children(&self, n: NodeId) -> impl Iterator<Item = NodeId> + '_ {
        let (first, k) =
            self.get(n).map_or((0, 0), |i| (i.first_child.get(), if i.first_child.is_none() { 0 } else { i.n_children as usize }));
        (first..first + k).map(NodeId::new)
    }

    /// Check structure: ids in range, parent links, set containment of children in parents, and,
    /// for `Extent::Box` parents that are not lattices, that children lie inside the parent,
    /// have pairwise disjoint interiors and tile its volume (relative `1e-9`). Returns located
    /// issues, empty when valid. O(sum of k^2) over non-lattice nodes.
    pub fn validate(&self, sets: &MaterialSets) -> Vec<BuildIssue> {
        let mut out = Vec::new();
        let vol = |b: &Aabb| {
            let d = b.hi - b.lo;
            d.x * d.y * d.z
        };
        for (i, n) in self.0.iter().enumerate() {
            let id = NodeId::new(i);
            if n.set.get() >= sets.len() {
                out.push(BuildIssue::new("majorant.set_range", format!("node {i} has an unknown material set")));
            }
            let kids: Vec<NodeId> = self.children(id).collect();
            if !n.first_child.is_none() && kids.iter().any(|k| k.get() >= self.len()) {
                out.push(BuildIssue::new("majorant.child_range", format!("node {i} has children past the table")));
                continue;
            }
            for &k in &kids {
                let c = &self.0[k.get()];
                if c.parent != id {
                    out.push(BuildIssue::new("majorant.parent_link", format!("node {} does not name node {i} as its parent", k.get())));
                }
                let (pm, cm) = (sets.get(n.set), sets.get(c.set));
                if let (Some(pm), Some(cm)) = (pm, cm)
                    && !cm.iter().all(|m| pm.binary_search(m).is_ok())
                {
                    out.push(BuildIssue::new(
                        "majorant.set_containment",
                        format!("node {} has materials missing from its parent {i}", k.get()),
                    ));
                }
            }
            let tiled = n.extent == Extent::Box && !kids.is_empty() && !matches!(n.kind, NodeKind::Lattice { .. });
            if tiled {
                let tol = 1e-9 * vol(&n.bounds).abs().max(f64::MIN_POSITIVE);
                let mut sum = 0.0;
                for (a, &ka) in kids.iter().enumerate() {
                    let ca = &self.0[ka.get()];
                    let inside = (0..3).all(|ax| {
                        let (l, h, pl, ph) = (at(ca.bounds.lo, ax), at(ca.bounds.hi, ax), at(n.bounds.lo, ax), at(n.bounds.hi, ax));
                        l >= pl - 1e-12 * pl.abs().max(1.0) && h <= ph + 1e-12 * ph.abs().max(1.0)
                    });
                    if !inside {
                        out.push(BuildIssue::new("majorant.child_outside", format!("node {} lies outside parent {i}", ka.get())));
                    }
                    if ca.extent == Extent::Box {
                        sum += vol(&ca.bounds);
                    }
                    for &kb in &kids[a + 1..] {
                        let cb = &self.0[kb.get()];
                        let overlap: f64 = (0..3)
                            .map(|ax| {
                                (at(ca.bounds.hi, ax).min(at(cb.bounds.hi, ax)) - at(ca.bounds.lo, ax).max(at(cb.bounds.lo, ax))).max(0.0)
                            })
                            .product();
                        if overlap > tol {
                            out.push(BuildIssue::new(
                                "majorant.siblings_overlap",
                                format!("nodes {} and {} overlap (volume {overlap:e})", ka.get(), kb.get()),
                            ));
                        }
                    }
                }
                if kids.iter().all(|k| self.0[k.get()].extent == Extent::Box) && (sum - vol(&n.bounds)).abs() > tol {
                    out.push(BuildIssue::new("majorant.not_a_tiling", format!("children of node {i} cover {sum} of {}", vol(&n.bounds))));
                }
            }
        }
        out
    }
}

// ---------------------------------------------------------------------------------------------
// Majorant table and the hybrid cost model
// ---------------------------------------------------------------------------------------------

/// Bounds of a total cross section over one (material set, band): `min <= Σ_t(E) <= max`.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Majorant {
    /// Lower bound (the minorant; decomposition tracking and the rejection bound use it).
    pub min: f64,
    /// Upper bound (the majorant `Σ_M`).
    pub max: f64,
}

impl Majorant {
    /// Upper bound on the probability that a tentative collision is rejected, `1 - min/max`
    /// (0 when `max == 0`).
    pub fn rejection_bound(&self) -> f64 {
        if self.max > 0.0 { 1.0 - self.min / self.max } else { 0.0 }
    }
}

/// The caller's cross sections: bounds per material and band. The caller owns the hard cases
/// (unresolved-resonance tables at their maximum, temperature and density ranges, S(alpha,beta)):
/// the returned `max` must bound every realization the transport code can sample in the band.
pub trait XsBound: Sync {
    /// `[min, max]` of `Σ_t` of `material` over `band` (finite, `0 <= min <= max`).
    fn sigma_t(&self, material: u32, band: BandId) -> Majorant;
}

impl<F: Fn(u32, BandId) -> Majorant + Sync> XsBound for F {
    fn sigma_t(&self, material: u32, band: BandId) -> Majorant {
        self(material, band)
    }
}

/// Per-(node, band) tracking mode decided at build time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum NodeMode {
    /// Stop at this node and use delta tracking with its majorant.
    Delta,
    /// Descend: the children (finally the cells) are tracked by surface.
    Surface,
}

/// Inputs of the build-time decision, in relative cost units (a `locate_in` costs
/// `locate_cost`, one cell crossing `cell_cost`). Both modes depend only on (node, band).
///
/// For a node with mean chord `l = volume^(1/3)` and `n` cells:
/// `cost_delta = (1 + Σ_M l) * locate_cost` (one segment plus the expected tentative
/// collisions, each a hinted location) and `cost_surface = max(1, n^(1/3)) * cell_cost` (cells
/// crossed per chord). A node is [`NodeMode::Delta`] iff `rejection_bound <= max_rejection` and
/// `cost_delta <= cost_surface`. The rejection test is Serpent's `dt` rule applied to the
/// bound; the cost test is what OpenMC's root-restarting prototype lacks.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct CostModel {
    /// Highest rejection probability at which delta tracking is used (Serpent `dt`, default 0.9).
    pub max_rejection: f64,
    /// Relative cost of one hinted location.
    pub locate_cost: f64,
    /// Relative cost of one cell crossing.
    pub cell_cost: f64,
}

impl Default for CostModel {
    fn default() -> Self {
        Self { max_rejection: 0.9, locate_cost: 1.0, cell_cost: 1.0 }
    }
}

impl CostModel {
    /// A model (the type is `#[non_exhaustive]`).
    pub fn new(max_rejection: f64, locate_cost: f64, cell_cost: f64) -> Self {
        Self { max_rejection, locate_cost, cell_cost }
    }

    /// The decision for a node with these bounds on its cross section.
    pub fn decide(&self, node: &NodeInfo, m: Majorant) -> NodeMode {
        let l = node.volume_estimate().cbrt();
        let cost_delta = (1.0 + m.max * l) * self.locate_cost;
        let cost_surface = f64::from(node.n_cells).cbrt().max(1.0) * self.cell_cost;
        if m.rejection_bound() <= self.max_rejection && cost_delta <= cost_surface { NodeMode::Delta } else { NodeMode::Surface }
    }
}

/// Majorants keyed by (material set, band), plus the build-time mode per (node, band). Plain
/// data: share it read-only between threads.
#[derive(Clone, Debug, PartialEq)]
pub struct MajorantTable {
    bands: EnergyBands,
    n_sets: usize,
    n_nodes: usize,
    entries: Vec<Majorant>,
    modes: Vec<NodeMode>,
    hash: ContentHash,
}

impl MajorantTable {
    /// Build over `h`: each set's entry is the `min`/`max` over its member materials, so a node
    /// bounds everything below it by construction. Deterministic (order-independent reductions).
    /// Errors with `InvalidInput` if `xs` returns a non-finite, negative or inverted range.
    pub fn build(h: &dyn MajorantHierarchy, bands: &EnergyBands, xs: &dyn XsBound, cost: &CostModel) -> Result<Self> {
        let (sets, nodes) = (h.material_sets(), h.nodes());
        let nb = bands.len();
        let mut entries = Vec::with_capacity(sets.len() * nb);
        for s in 0..sets.len() {
            let members = sets.get(MaterialSetId::new(s)).unwrap_or(&[]);
            for b in 0..nb {
                let mut m = Majorant { min: f64::INFINITY, max: 0.0 };
                for &mat in members {
                    let r = xs.sigma_t(mat, BandId::new(b));
                    if !(r.min.is_finite() && r.max.is_finite() && r.min >= 0.0 && r.min <= r.max) {
                        return Err(RtError::InvalidInput("cross-section bounds must be finite with 0 <= min <= max"));
                    }
                    m.min = m.min.min(r.min);
                    m.max = m.max.max(r.max);
                }
                if members.is_empty() {
                    m.min = 0.0;
                }
                entries.push(m);
            }
        }
        let mut modes = Vec::with_capacity(nodes.len() * nb);
        for n in &nodes.0 {
            for b in 0..nb {
                let m =
                    entries.get(n.set.get() * nb + b).copied().ok_or(RtError::InvalidInput("node refers to an unknown material set"))?;
                modes.push(cost.decide(n, m));
            }
        }
        let mut t = Self { bands: bands.clone(), n_sets: sets.len(), n_nodes: nodes.len(), entries, modes, hash: ContentHash(0) };
        let mut hh = ContentHasher::new();
        let sh = h.structure_hash().0;
        hh.write_u64((sh >> 64) as u64);
        hh.write_u64(sh as u64);
        bands.hash_into(&mut hh);
        t.entries.iter().for_each(|m| {
            hh.write_f64(m.min);
            hh.write_f64(m.max);
        });
        t.modes.iter().for_each(|&m| hh.write_u64(u64::from(m == NodeMode::Delta)));
        t.hash = hh.finish();
        Ok(t)
    }

    /// The bands.
    pub fn bands(&self) -> &EnergyBands {
        &self.bands
    }
    /// Number of material sets.
    pub fn n_sets(&self) -> usize {
        self.n_sets
    }
    /// Bounds of `set` in `band`.
    pub fn get(&self, set: MaterialSetId, band: BandId) -> Option<Majorant> {
        (set.get() < self.n_sets && band.get() < self.bands.len()).then(|| self.entries[set.get() * self.bands.len() + band.get()])
    }
    /// Build-time mode of `node` in `band`.
    pub fn mode(&self, node: NodeId, band: BandId) -> Option<NodeMode> {
        (node.get() < self.n_nodes && band.get() < self.bands.len()).then(|| self.modes[node.get() * self.bands.len() + band.get()])
    }
    /// `Ok` if `sigma` is within the majorant of (`set`, `band`), else
    /// [`RtError::MajorantViolation`]. Call it at every tentative collision: a violation is a
    /// bug in the caller's bounds, and clamping would hide a bias.
    pub fn check(&self, set: MaterialSetId, band: BandId, sigma: f64) -> Result<()> {
        let m = self.get(set, band).ok_or(RtError::UnknownId { kind: "material set", id: set.0 })?;
        if sigma <= m.max { Ok(()) } else { Err(RtError::MajorantViolation { set: set.0, band: band.0, sigma, majorant: m.max }) }
    }
    /// Content hash: hierarchy structure, bands, entries, modes. Rebuilt on demand, never cached
    /// in the `.smrt` sidecar (it is cheap and depends on the caller's cross sections).
    pub fn content_hash(&self) -> ContentHash {
        self.hash
    }
}

// ---------------------------------------------------------------------------------------------
// Segments and the march
// ---------------------------------------------------------------------------------------------

/// How the caller treats a segment.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum SegmentMode {
    /// One cell, exact `Σ_t`: sample with it, nothing is rejected, track-length scoring is valid.
    Surface,
    /// Many cells below one node: sample with `Σ_M`, locate at each tentative collision with
    /// `locate_in(node, ..)`, accept with `Σ_t/Σ_M`, collision-estimator scoring.
    Delta,
}

/// Which tally estimator a segment supports.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Estimator {
    /// Score `weight * length * (per-length response)`.
    TrackLength,
    /// Score at collisions (real and, with weights, virtual).
    Collision,
}

/// One piece of a ray: `[t0, t1]` in one node. Build with [`MajorantSegment::delta`] or [`MajorantSegment::surface`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct MajorantSegment {
    /// Node that owns the majorant (the hint for `locate_in`). For a surface segment, the leaf
    /// containing the cell.
    pub node: NodeId,
    /// Material set (`Σ_M` for `Delta`; the cell's singleton set for `Surface`).
    pub set: MaterialSetId,
    /// Ray distance at entry (measured from the march start).
    pub t0: f64,
    /// Ray distance at exit.
    pub t1: f64,
    /// Mode.
    pub mode: SegmentMode,
    /// The cell (`Surface` only; `CellId::NONE` for `Delta`).
    pub cell: CellId,
    /// The cell's region (`Surface` only; `RegionId::NONE` for `Delta`).
    pub region: RegionId,
}

impl MajorantSegment {
    /// A delta segment in `node`.
    pub fn delta(node: NodeId, set: MaterialSetId, t0: f64, t1: f64) -> Self {
        Self { node, set, t0, t1, mode: SegmentMode::Delta, cell: CellId::NONE, region: RegionId::NONE }
    }
    /// A surface segment in `cell` of `region`, inside leaf `node`.
    pub fn surface(node: NodeId, set: MaterialSetId, t0: f64, t1: f64, cell: CellId, region: RegionId) -> Self {
        Self { node, set, t0, t1, mode: SegmentMode::Surface, cell, region }
    }
    /// Chord length `t1 - t0`.
    pub fn length(&self) -> f64 {
        self.t1 - self.t0
    }
    /// The estimator this segment supports.
    pub fn estimator(&self) -> Estimator {
        match self.mode {
            SegmentMode::Surface => Estimator::TrackLength,
            SegmentMode::Delta => Estimator::Collision,
        }
    }
}

/// Which nodes the march stops at. All variants depend only on (node, set, band), never on the
/// particle, so the choice is reproducible and cacheable.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum RefinePolicy {
    /// Stop at the coarsest node: the leaf of the top-level DDA. Pure delta tracking.
    Coarsest,
    /// Refine to cells: every segment is a cell chord. Pure surface tracking (MOC/Sn, mesh tallies).
    Cells,
    /// Stop at the first node on the path with `rejection_bound <= threshold` (Serpent `dt`).
    Threshold(f64),
    /// Stop at the first node whose [`MajorantTable::mode`] is `Delta` (build-time cost model).
    PerNode,
}

impl RefinePolicy {
    /// True if the march should emit a delta segment for `node` rather than descend. A leaf
    /// that does not stop is refined into cell chords. Implementors call this at every node.
    pub fn stop_at(&self, table: &MajorantTable, node: NodeId, set: MaterialSetId, band: BandId) -> bool {
        match *self {
            Self::Coarsest => true,
            Self::Cells => false,
            Self::Threshold(t) => table.get(set, band).is_some_and(|m| m.rejection_bound() <= t),
            Self::PerNode => table.mode(node, band) == Some(NodeMode::Delta),
        }
    }
}

/// Limits and policy for one march.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct MarchOptions {
    /// Stop after this ray distance (`+inf` allowed).
    pub max_dist: f64,
    /// Band of the particle's energy. A march uses one band; a collision starts a new one.
    pub band: BandId,
    /// Where to stop descending.
    pub refine: RefinePolicy,
    /// Safety cap on emitted segments (`MarchEnd::MaxSegments` beyond it).
    pub max_segments: u32,
}

impl MarchOptions {
    /// `Coarsest` policy, cap one million segments.
    pub fn new(max_dist: f64, band: BandId) -> Self {
        Self { max_dist, band, refine: RefinePolicy::Coarsest, max_segments: 1_000_000 }
    }
    /// Set the policy.
    pub fn with_refine(mut self, r: RefinePolicy) -> Self {
        self.refine = r;
        self
    }
    /// Set the segment cap.
    pub fn with_max_segments(mut self, n: u32) -> Self {
        self.max_segments = n;
        self
    }
}

/// Why a march ended.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum MarchEnd {
    /// `max_dist` reached.
    Limit,
    /// Reached a model boundary. Call [`MajorantHierarchy::finish`]: for `Reflect` and `Wrap` it
    /// returns the state with the action applied (as `Tracker::step` does), for `Respond` the
    /// state on the face for `Tracker::resolve_boundary`.
    Boundary {
        /// Model surface.
        surface: SurfaceId,
        /// Face.
        face: FaceId,
        /// What the surface does.
        action: FaceAction,
    },
    /// The start point is in no cell.
    Lost(LostInfo),
    /// `max_segments` emitted.
    MaxSegments,
}

/// One DDA level in a cursor. `idx`/`t_next`/`step` are the DDA of that node (unused entries zero).
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct CursorLevel {
    /// Node being traversed.
    pub node: NodeId,
    /// Element index inside it (lattice) or child index in `idx[0]` (split, partition).
    pub idx: [i32; 3],
    /// Ray distance of the next wall crossing per axis (`+inf` if parallel).
    pub t_next: [f64; 3],
    /// `+1` or `-1` per axis (0 if parallel).
    pub step: [i32; 3],
}

impl CursorLevel {
    /// A level for `node` with zeroed DDA state.
    pub fn new(node: NodeId) -> Self {
        Self { node, idx: [0; 3], t_next: [f64::INFINITY; 3], step: [0; 3] }
    }
}

/// Resumable march state, caller-owned (one per particle). Fields are implementor-owned; callers
/// read `origin`, `dir`, `t`, [`Self::end`] and [`Self::segments`] only. Inline up to
/// [`LEVEL_INLINE`] DDA levels, so no heap allocation for hierarchies that deep or shallower.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct MarchCursor {
    /// Ray origin (global frame).
    pub origin: Point3,
    /// Unit direction.
    pub dir: Vector3,
    /// Ray distance reached so far (the `t1` of the last segment).
    pub t: f64,
    /// Options of this march.
    pub opts: MarchOptions,
    /// DDA stack, root first.
    pub levels: SmallVec<[CursorLevel; LEVEL_INLINE]>,
    /// Leaf of the last emitted segment (the hint for `finish`).
    pub node: NodeId,
    /// Set when the march is over.
    pub end: Option<MarchEnd>,
    /// Segments emitted.
    pub emitted: u32,
    /// Implementor scratch (for example the cell walk position inside a leaf).
    pub aux: [f64; 4],
}

impl MarchCursor {
    /// A cursor at the start of the ray `origin + t * dir`.
    pub fn new(origin: Point3, dir: Vector3, opts: MarchOptions) -> Self {
        Self { origin, dir, t: 0.0, opts, levels: SmallVec::new(), node: NodeId::NONE, end: None, emitted: 0, aux: [0.0; 4] }
    }
    /// Why the march ended, once `next` has returned `None`.
    pub fn end(&self) -> Option<&MarchEnd> {
        self.end.as_ref()
    }
    /// Segments emitted so far.
    pub fn segments(&self) -> u32 {
        self.emitted
    }
    /// Point at ray distance `t`.
    pub fn point_at(&self, t: f64) -> Point3 {
        self.origin + self.dir * t
    }
}

/// Options for [`crate::Tracker::majorant_hierarchy`]. Start from `default()`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct HierarchyOptions {
    /// Split a meshed region into kd leaves until a leaf has at most this many cells.
    pub max_leaf_cells: u32,
    /// Depth cap of the kd split.
    pub max_depth: u32,
}

impl Default for HierarchyOptions {
    fn default() -> Self {
        Self { max_leaf_cells: 64, max_depth: 24 }
    }
}

/// The disjoint majorant hierarchy of a tracker. Built once (`Tracker::majorant_hierarchy`),
/// shared read-only, `Send + Sync`. Its methods are pure: all mutable state is in the
/// caller's [`MarchCursor`] and [`Scratch`]. See the module docs for the binding contract.
pub trait MajorantHierarchy: Send + Sync {
    /// Nodes; node 0 is the root.
    fn nodes(&self) -> &NodeTable;
    /// Deduplicated material sets referenced by the nodes.
    fn material_sets(&self) -> &MaterialSets;
    /// Hash of structure, node sets and the material map; the tracker's `structure_hash`
    /// combined with the [`MaterialMap`]'s.
    fn structure_hash(&self) -> ContentHash;
    /// The leaf that owns `cell` (the hint to resume at after a real collision in a known cell).
    fn node_of_cell(&self, cell: CellId) -> Option<NodeId>;
    /// Singleton set of `material` (what `Surface` segments carry).
    fn singleton(&self, material: u32) -> Option<MaterialSetId> {
        self.material_sets().find(&[material])
    }

    /// Hinted point location: the cell of `p` among the cells below `node`, with the hierarchy
    /// path in `levels`. A point beyond the extent (by more than `tol.linear`) gives
    /// `CellId::NONE`, never a fallback to the root; on a wall shared with a sibling the node's
    /// own side wins. `dir` decides a point on a face, as in `Tracker::locate`.
    fn locate_in(&self, node: NodeId, p: Point3, dir: Option<Vector3>, scratch: &mut Scratch) -> Result<Located>;

    /// Exact lower bound of the distance from `p` to the boundary of `node`'s extent: the
    /// distance to the nearest box face for `Extent::Box`, `0` for `Extent::Exact` unless the
    /// implementor knows better. A sampled flight shorter than this needs no boundary test
    /// (the safety distance of Geant4 and DAGMC's signed-distance prototype, exact).
    fn safety(&self, node: NodeId, p: Point3) -> f64 {
        let Some(n) = self.nodes().get(node) else {
            return 0.0;
        };
        if n.extent != Extent::Box || !n.bounds.contains(p) {
            return 0.0;
        }
        (0..3).map(|a| (at(p, a) - at(n.bounds.lo, a)).min(at(n.bounds.hi, a) - at(p, a))).fold(f64::INFINITY, f64::min)
    }

    /// Start a march from `st` (position and direction; its cell is only a hint). If the start
    /// is in no cell the cursor is returned already ended with [`MarchEnd::Lost`].
    fn begin(&self, st: &TrackState, opts: &MarchOptions, table: &MajorantTable, scratch: &mut Scratch) -> Result<MarchCursor>;

    /// The next segment, or `None` when the march ended (see [`MarchCursor::end`]). Segments
    /// tile the ray (contract item 3). The same `table`, bands and options as `begin`.
    fn next(&self, cur: &mut MarchCursor, table: &MajorantTable, scratch: &mut Scratch) -> Result<Option<MajorantSegment>>;

    /// A located [`TrackState`] at the cursor's current point, found from the node of the last
    /// segment. After a `MarchEnd::Boundary` it is the state on the face with `Reflect` and
    /// `Wrap` already applied. Use it after the march ends; after a collision use
    /// [`Self::locate_on_segment`].
    fn finish(&self, cur: &MarchCursor, scratch: &mut Scratch) -> Result<TrackState>;

    /// Hinted location of the point `s` into `seg` (`0 <= s <= length`): the state a collision
    /// there continues from. Starts at `seg.node`, never at the root.
    fn locate_on_segment(&self, cur: &MarchCursor, seg: &MajorantSegment, s: f64, scratch: &mut Scratch) -> Result<TrackState> {
        if !(s >= 0.0 && s <= seg.length() * (1.0 + 1e-12)) {
            return Err(RtError::InvalidInput("collision distance outside the segment"));
        }
        let p = cur.point_at(seg.t0 + s);
        let loc = self.locate_in(seg.node, p, Some(cur.dir), scratch)?;
        let mut st = TrackState::new(p, cur.dir);
        st.cell = loc.cell;
        st.region = loc.region;
        st.entered_via = None;
        st.levels = loc.levels;
        Ok(st)
    }

    /// Run a march to its end, calling `sink` per segment (`Flow::Stop` ends it early with
    /// `Ok(None)`). Returns the end reason.
    fn march(
        &self,
        st: &TrackState,
        opts: &MarchOptions,
        table: &MajorantTable,
        sink: &mut dyn FnMut(&MajorantSegment) -> Flow,
        scratch: &mut Scratch,
    ) -> Result<Option<MarchEnd>> {
        let mut cur = self.begin(st, opts, table, scratch)?;
        while let Some(seg) = self.next(&mut cur, table, scratch)? {
            if sink(&seg) == Flow::Stop {
                return Ok(None);
            }
        }
        Ok(cur.end)
    }
}

// ---------------------------------------------------------------------------------------------
// Reference driver
// ---------------------------------------------------------------------------------------------

/// Work counters of flights (no wall clock): the quantities the benchmark compares.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FlightStats {
    /// Segments consumed.
    pub segments: u64,
    /// Tentative collisions (delta segments only).
    pub tentative: u64,
    /// Tentative collisions rejected (virtual).
    pub rejected: u64,
    /// Hinted locations performed.
    pub locates: u64,
}

/// What happened along a flight, for scoring.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum FlightEvent<'a> {
    /// The particle crossed `length` of `seg` without a real collision: score a track length
    /// for `Surface` segments (a collision-estimator tally ignores it).
    Traverse {
        /// The segment.
        seg: &'a MajorantSegment,
        /// Distance travelled in it (the segment length, or less for the collision segment).
        length: f64,
    },
    /// A tentative collision in a delta segment: score a collision-estimator contribution
    /// (real if `accepted`, else virtual).
    Tentative {
        /// Cell found by the hinted location.
        cell: CellId,
        /// Its region.
        region: RegionId,
        /// `Σ_t` at the point.
        sigma_t: f64,
        /// `Σ_M` of the node.
        majorant: f64,
        /// Whether it was a real collision.
        accepted: bool,
    },
}

/// End of a flight.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum FlightEnd {
    /// A real collision at ray distance `dist`; `state` is located (cell, levels).
    Collision {
        /// State at the collision.
        state: TrackState,
        /// Ray distance from the start.
        dist: f64,
    },
    /// No collision: the march ended (limit, boundary, lost). For a boundary call
    /// `MajorantHierarchy::finish` with a cursor-free restart, or use `begin`/`next` directly.
    Ended(MarchEnd),
}

/// Reference flight: sample to the next real collision over a march, in either mode per
/// segment. This is the normative statement of how segments are consumed; production codes may
/// inline it. `sigma_t` is the exact total cross section of a region at the particle's energy
/// (the caller maps region to material); `rng` returns uniforms in `[0, 1)`.
///
/// * `Surface` segment: flight `-ln(1-ξ)/Σ_t`; shorter than the segment is a real collision.
/// * `Delta` segment: repeat `-ln(1-ξ)/Σ_M`; past the segment end, move on; else locate with
///   the hint `seg.node`, check `Σ_t <= Σ_M` ([`MajorantTable::check`]) and accept with
///   probability `Σ_t/Σ_M`.
#[allow(clippy::too_many_arguments)]
pub fn sample_flight(
    h: &dyn MajorantHierarchy,
    st: &TrackState,
    opts: &MarchOptions,
    table: &MajorantTable,
    sigma_t: &dyn Fn(RegionId) -> f64,
    rng: &mut dyn FnMut() -> f64,
    on_event: &mut dyn FnMut(&FlightEvent<'_>),
    stats: &mut FlightStats,
    scratch: &mut Scratch,
) -> Result<FlightEnd> {
    let mut cur = h.begin(st, opts, table, scratch)?;
    while let Some(seg) = h.next(&mut cur, table, scratch)? {
        stats.segments += 1;
        let len = seg.length();
        match seg.mode {
            SegmentMode::Surface => {
                let sig = sigma_t(seg.region);
                let s = if sig > 0.0 { -(1.0 - rng()).ln() / sig } else { f64::INFINITY };
                if s < len {
                    on_event(&FlightEvent::Traverse { seg: &seg, length: s });
                    stats.locates += 1;
                    let state = h.locate_on_segment(&cur, &seg, s, scratch)?;
                    return Ok(FlightEnd::Collision { state, dist: seg.t0 + s });
                }
                on_event(&FlightEvent::Traverse { seg: &seg, length: len });
            }
            SegmentMode::Delta => {
                let maj = table.get(seg.set, opts.band).ok_or(RtError::UnknownId { kind: "material set", id: seg.set.0 })?.max;
                let mut pos = 0.0;
                loop {
                    pos += if maj > 0.0 { -(1.0 - rng()).ln() / maj } else { f64::INFINITY };
                    if pos >= len {
                        on_event(&FlightEvent::Traverse { seg: &seg, length: len });
                        break;
                    }
                    stats.tentative += 1;
                    stats.locates += 1;
                    let state = h.locate_on_segment(&cur, &seg, pos, scratch)?;
                    let sig = sigma_t(state.region);
                    table.check(seg.set, opts.band, sig)?;
                    let accepted = rng() * maj < sig;
                    on_event(&FlightEvent::Tentative { cell: state.cell, region: state.region, sigma_t: sig, majorant: maj, accepted });
                    if accepted {
                        return Ok(FlightEnd::Collision { state, dist: seg.t0 + pos });
                    }
                    stats.rejected += 1;
                }
            }
        }
    }
    Ok(FlightEnd::Ended(cur.end.unwrap_or(MarchEnd::Limit)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bands_edges_and_lookup() {
        let b = EnergyBands::new(vec![0.0, 1.0, 10.0, f64::INFINITY]).unwrap();
        assert_eq!(b.len(), 3);
        assert_eq!(b.band_of(0.0), Some(BandId(0)));
        assert_eq!(b.band_of(1.0), Some(BandId(1)));
        assert_eq!(b.band_of(9.999), Some(BandId(1)));
        assert_eq!(b.band_of(10.0), Some(BandId(2)));
        assert_eq!(b.band_of(f64::INFINITY), Some(BandId(2)));
        assert_eq!(b.band_of(-1.0), None);
        assert_eq!(b.band_of(f64::NAN), None);
        assert!(EnergyBands::new(vec![1.0]).is_err() && EnergyBands::new(vec![0.0, 0.0]).is_err());
        assert!(EnergyBands::new(vec![0.0, f64::INFINITY, f64::INFINITY]).is_err());
        assert_eq!(EnergyBands::single().band_of(5.0), Some(BandId(0)));
    }

    #[test]
    fn material_sets_dedup_and_contain() {
        let mut s = MaterialSets::new();
        let a = s.intern(&[3, 1, 3]);
        let b = s.intern(&[1, 3]);
        let c = s.intern(&[2]);
        assert_eq!((a, b, c), (MaterialSetId(0), MaterialSetId(0), MaterialSetId(1)));
        assert_eq!(s.get(a), Some(&[1u32, 3][..]));
        assert!(s.contains(a, 3) && !s.contains(a, 2));
        assert_eq!(s.find(&[2]), Some(c));
        assert_eq!(s.find(&[9]), None);
    }

    #[test]
    fn majorant_rejection_bound() {
        assert_eq!(Majorant { min: 1.0, max: 4.0 }.rejection_bound(), 0.75);
        assert_eq!(Majorant::default().rejection_bound(), 0.0);
    }

    #[test]
    fn cost_model_prefers_delta_for_thin_many_cell_nodes_and_surface_for_thick() {
        let b = Aabb { lo: Point3::new(0.0, 0.0, 0.0), hi: Point3::new(1.0, 1.0, 1.0) };
        let n = NodeInfo::new(b, MaterialSetId(0), 1000);
        let c = CostModel::default();
        // Optically thin: Σ_M l = 0.5, cost 1.5 against 10 cell crossings.
        assert_eq!(c.decide(&n, Majorant { min: 0.4, max: 0.5 }), NodeMode::Delta);
        // Optically thick: cost 101 against 10.
        assert_eq!(c.decide(&n, Majorant { min: 90.0, max: 100.0 }), NodeMode::Surface);
        // Thin but a strong absorber inside: rejection bound 0.99 > 0.9.
        assert_eq!(c.decide(&n, Majorant { min: 0.005, max: 0.5 }), NodeMode::Surface);
    }
}
