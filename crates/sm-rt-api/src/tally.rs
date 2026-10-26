//! Segments, sinks, bundles and tally binning: the part shared by Monte Carlo and Sn/MOC.
//!
//! A *segment* is one cell's chord along a ray. A Monte Carlo track-length tally, an MOC
//! characteristic (optical-depth integral per cell and a track weight), and an Sn
//! cell-ordering/tracing pass all consume the same segment stream from the same structure.

use crate::cache::ContentHasher;
use crate::error::{LostInfo, RtError};
use crate::ids::{CellId, FaceId, RegionId, SurfaceId};
use crate::boundary::{FaceAction, ResponseKind};
use crate::state::TrackState;

/// One cell's chord along a ray.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Segment {
    /// The cell.
    pub cell: CellId,
    /// Its region.
    pub region: RegionId,
    /// Ray parameter at entry, measured from the ray start (distance, rays are unit speed).
    pub t0: f64,
    /// Chord length in the cell.
    pub length: f64,
    /// Face the ray entered the cell through (`None` for the first segment).
    pub face_in: Option<FaceId>,
    /// Face the ray leaves through (`None` if the ray ended inside the cell or crossed a
    /// lattice/instance wall that is not a model face).
    pub face_out: Option<FaceId>,
}

/// What a sink wants next.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Flow {
    /// Keep going.
    Continue,
    /// End the trace here (e.g. a Monte Carlo collision happened in this segment).
    Stop,
}

/// Receives segments in ray order.
pub trait SegmentSink {
    /// Called once per segment, in order.
    fn segment(&mut self, seg: &Segment) -> Flow;
}

impl<F: FnMut(&Segment) -> Flow> SegmentSink for F {
    fn segment(&mut self, seg: &Segment) -> Flow {
        self(seg)
    }
}

/// Limits for a trace.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TraceOptions {
    /// Maximum total distance (`f64::INFINITY` for none).
    pub max_dist: f64,
    /// Step cap against infinite walk loops; exceeding it is `LostCause::MaxStepsExceeded`.
    pub max_steps: u32,
    /// End the trace at deck boundaries too (`Reflect`, `Wrap`). By default a trace continues
    /// through them, since `step` already applied them; the segments then follow the unfolded path
    /// (MOC with reflective or periodic boundaries). `Respond` boundaries always end the trace.
    pub stop_at_deck: bool,
}

impl Default for TraceOptions {
    fn default() -> Self {
        Self { max_dist: f64::INFINITY, max_steps: 1_000_000, stop_at_deck: false }
    }
}

/// Why a trace ended.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum TraceEnd {
    /// `max_dist` reached.
    Limit,
    /// The sink asked to stop.
    Stopped,
    /// Left the model (into outside space) through a non-boundary surface.
    LeftModel,
    /// Reached a model boundary. A `Respond` action is unresolved (state on the face; the caller
    /// calls `resolve_boundary`); `Reflect` / `Wrap` appear only with `stop_at_deck` and were
    /// already applied.
    Boundary {
        /// The surface.
        surface: SurfaceId,
        /// The face.
        face: FaceId,
        /// Its action.
        action: FaceAction,
    },
    /// Lost.
    Lost(LostInfo),
}

/// Result of [`crate::Tracker::fire`] (DAGMC `ray_fire`): distance to the next *region* change.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FireResult {
    /// Distance travelled.
    pub dist: f64,
    /// How it ended (`Limit`, `RegionBoundary`, `Boundary` or `Lost`; never `CellFace` or
    /// `SlotChange`).
    pub end: crate::state::StepEnd,
}

/// How a bundle ray ended (flat form of [`TraceEnd`] for SoA storage).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum EndCode {
    /// Distance limit.
    Limit = 0,
    /// Sink stopped (never produced by bundles).
    Stopped = 1,
    /// Left the model.
    LeftModel = 2,
    /// Vacuum boundary.
    Vacuum = 3,
    /// Reflective boundary (a mirror deck or a reflective response).
    Reflective = 4,
    /// White boundary.
    White = 5,
    /// Periodic boundary (a wrap deck).
    Periodic = 6,
    /// Lost.
    Lost = 7,
    /// Albedo boundary (the coefficient is in the surface table).
    Albedo = 8,
}

impl From<&TraceEnd> for EndCode {
    fn from(e: &TraceEnd) -> Self {
        match e {
            TraceEnd::Limit => Self::Limit,
            TraceEnd::Stopped => Self::Stopped,
            TraceEnd::LeftModel => Self::LeftModel,
            TraceEnd::Lost(_) => Self::Lost,
            TraceEnd::Boundary { action, .. } => match action {
                FaceAction::Respond(ResponseKind::Vacuum) => Self::Vacuum,
                FaceAction::Respond(ResponseKind::Reflective) | FaceAction::Reflect => Self::Reflective,
                FaceAction::Respond(ResponseKind::White) => Self::White,
                FaceAction::Respond(ResponseKind::Albedo(_)) => Self::Albedo,
                FaceAction::Wrap { .. } => Self::Periodic,
                _ => Self::Limit,
            },
        }
    }
}

/// Segments of many rays in CSR layout: the MOC / Sn track database. Ray `i`'s segments are
/// `offsets[i]..offsets[i+1]`. Rays appear in input order regardless of thread count.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TrackBundle {
    /// `n_rays + 1` offsets into the segment columns.
    pub offsets: Vec<u32>,
    /// Cell per segment.
    pub cell: Vec<u32>,
    /// Region per segment.
    pub region: Vec<u32>,
    /// Chord length per segment.
    pub length: Vec<f64>,
    /// End code per ray.
    pub end: Vec<EndCode>,
    /// Surface of the boundary hit per ray (`SurfaceId::NONE` if none).
    pub end_surface: Vec<u32>,
}

impl TrackBundle {
    /// Number of rays.
    pub fn ray_count(&self) -> usize {
        self.end.len()
    }
    /// Segment index range of ray `i`.
    pub fn ray_range(&self, i: usize) -> core::ops::Range<usize> {
        self.offsets[i] as usize..self.offsets[i + 1] as usize
    }
    /// Start a bundle.
    pub fn new() -> Self {
        Self { offsets: vec![0], ..Self::default() }
    }
    /// Append one finished ray.
    pub fn push_ray(&mut self, segs: &[Segment], end: &TraceEnd) {
        for s in segs {
            self.cell.push(s.cell.0);
            self.region.push(s.region.0);
            self.length.push(s.length);
        }
        self.offsets.push(self.cell.len() as u32);
        self.end.push(end.into());
        self.end_surface.push(match end {
            TraceEnd::Boundary { surface, .. } => surface.0,
            _ => SurfaceId::NONE.0,
        });
    }
    /// Append another bundle's rays after this one's (used to join per-block results in block order).
    pub fn append(&mut self, o: &Self) {
        let base = self.cell.len() as u32;
        self.offsets.extend(o.offsets.iter().skip(1).map(|&x| x + base));
        self.cell.extend_from_slice(&o.cell);
        self.region.extend_from_slice(&o.region);
        self.length.extend_from_slice(&o.length);
        self.end.extend_from_slice(&o.end);
        self.end_surface.extend_from_slice(&o.end_surface);
    }
    /// Stable hash of the bundle (lengths by bit pattern): equal for 1 and N threads.
    pub fn content_hash(&self) -> crate::cache::ContentHash {
        let mut h = ContentHasher::new();
        h.write_u64(self.ray_count() as u64);
        for &o in &self.offsets {
            h.write_u64(u64::from(o));
        }
        for (&c, &r) in self.cell.iter().zip(&self.region) {
            h.write_u64(u64::from(c));
            h.write_u64(u64::from(r));
        }
        for &l in &self.length {
            h.write_f64(l);
        }
        for (&e, &s) in self.end.iter().zip(&self.end_surface) {
            h.write_u64(e as u64);
            h.write_u64(u64::from(s));
        }
        h.finish()
    }
}

/// What to bin tallies by.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum TallySpec {
    /// One bin per tracking cell (the Sn / MOC flux mesh and the Monte Carlo mesh tally on the
    /// transport mesh itself).
    Cells,
    /// One bin per region.
    Regions,
    /// One bin per named mesh set (cell sets / blocks), in the given order.
    MeshSets(Vec<String>),
}

/// Cell to bin map. A cell may be in no bin (`NO_BIN`).
#[derive(Clone, Debug, PartialEq)]
pub struct BinMap {
    /// Number of bins.
    pub n_bins: usize,
    /// Bin per cell.
    pub bin_of_cell: Vec<u32>,
}

impl BinMap {
    /// Cell not in any bin.
    pub const NO_BIN: u32 = u32::MAX;
    /// Identity map over `n` cells.
    pub fn identity(n: usize) -> Self {
        Self { n_bins: n, bin_of_cell: (0..n as u32).collect() }
    }
    /// Bin of a cell, or `None`.
    pub fn bin(&self, c: CellId) -> Option<usize> {
        match self.bin_of_cell.get(c.get()) {
            Some(&b) if b != Self::NO_BIN => Some(b as usize),
            _ => None,
        }
    }
}

/// Track-length tally: sum of `weight * length` per bin. One per worker; merge in a fixed order.
///
/// Determinism: the partition of rays over workers must depend only on the input (e.g.
/// contiguous blocks of fixed size), never on timing, and workers' tallies must be merged in
/// worker-index order with [`TrackLengthTally::merge`]. Then results are bit-identical for any
/// thread count that uses the same block partition; for different partitions use a block size
/// independent of the thread count (the contract's recommended default).
#[derive(Clone, Debug, PartialEq)]
pub struct TrackLengthTally {
    bins: BinMap,
    /// Accumulated score per bin.
    pub acc: Vec<f64>,
    /// Weight applied to the segments that follow.
    pub weight: f64,
}

impl TrackLengthTally {
    /// A zeroed tally.
    pub fn new(bins: BinMap) -> Self {
        let n = bins.n_bins;
        Self { bins, acc: vec![0.0; n], weight: 1.0 }
    }
    /// Add `other` into `self`, bin by bin in index order.
    pub fn merge(&mut self, other: &Self) -> Result<(), RtError> {
        if self.acc.len() != other.acc.len() {
            return Err(RtError::Unsupported("tally bin counts differ"));
        }
        for (a, b) in self.acc.iter_mut().zip(&other.acc) {
            *a += *b;
        }
        Ok(())
    }
}

impl SegmentSink for TrackLengthTally {
    fn segment(&mut self, seg: &Segment) -> Flow {
        if let Some(b) = self.bins.bin(seg.cell) {
            self.acc[b] += self.weight * seg.length;
        }
        Flow::Continue
    }
}

/// A tiny helper: advance `st` by `d` along its direction (used by default trait methods).
pub(crate) fn advance(st: &mut TrackState, d: f64) {
    st.pos = st.pos + st.dir * d;
}
