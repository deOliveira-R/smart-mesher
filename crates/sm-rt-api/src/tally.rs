//! Segments, sinks, bundles and tally binning: the part shared by Monte Carlo and Sn/MOC.
//!
//! A *segment* is one cell's chord along a ray. A Monte Carlo track-length tally, an MOC
//! characteristic (optical-depth integral per cell and a track weight), and an Sn
//! cell-ordering/tracing pass all consume the same segment stream from the same structure.

use crate::cache::ContentHasher;
use crate::error::{BuildIssue, LostInfo, RtError};
use crate::ids::{CellId, FaceId, RegionId, SurfaceId};
use crate::boundary::{FaceAction, ResponseKind};
use crate::state::TrackState;

/// One cell's chord along a ray. Build with [`Segment::new`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct Segment {
    /// The cell.
    pub cell: CellId,
    /// Its region.
    pub region: RegionId,
    /// Ray parameter at entry, measured from the ray start (distance, rays are unit speed).
    pub t0: f64,
    /// Chord length in the cell.
    pub length: f64,
    /// Face the ray entered the cell through. `None` for the first segment and for the first
    /// segment after a `SlotChange` (a lattice or instance wall is not a model face).
    pub face_in: Option<FaceId>,
    /// Face the ray leaves through (`None` if the ray ended inside the cell or crossed a
    /// lattice/instance wall that is not a model face).
    pub face_out: Option<FaceId>,
}

impl Segment {
    /// A segment with no entry or exit face; set them with [`Self::with_faces`].
    pub fn new(cell: CellId, region: RegionId, t0: f64, length: f64) -> Self {
        Self { cell, region, t0, length, face_in: None, face_out: None }
    }
    /// The same with entry and exit faces.
    pub fn with_faces(mut self, face_in: Option<FaceId>, face_out: Option<FaceId>) -> Self {
        self.face_in = face_in;
        self.face_out = face_out;
        self
    }
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

/// Limits for a trace. Start from [`TraceOptions::default`] and use the `with_*` builders.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
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

impl TraceOptions {
    /// Set the maximum total distance.
    pub fn with_max_dist(mut self, d: f64) -> Self {
        self.max_dist = d;
        self
    }
    /// Set the step cap.
    pub fn with_max_steps(mut self, n: u32) -> Self {
        self.max_steps = n;
        self
    }
    /// Set whether the trace ends at deck boundaries.
    pub fn with_stop_at_deck(mut self, stop: bool) -> Self {
        self.stop_at_deck = stop;
        self
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
#[non_exhaustive]
pub struct FireResult {
    /// Distance travelled.
    pub dist: f64,
    /// How it ended (`Limit`, `RegionBoundary`, `Boundary` or `Lost`; never `CellFace` or
    /// `SlotChange`).
    pub end: crate::state::StepEnd,
}

impl FireResult {
    /// A result of travelling `dist` and ending with `end`.
    pub fn new(dist: f64, end: crate::state::StepEnd) -> Self {
        Self { dist, end }
    }
}

/// How a bundle ray ended (flat form of [`TraceEnd`] for SoA storage).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
#[non_exhaustive]
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
    /// Any other end (a face action with no dedicated code, e.g. `Transmit`, or one added later).
    Other = 9,
}

impl EndCode {
    /// Decode a stored discriminant (`None` for an unknown code).
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Limit,
            1 => Self::Stopped,
            2 => Self::LeftModel,
            3 => Self::Vacuum,
            4 => Self::Reflective,
            5 => Self::White,
            6 => Self::Periodic,
            7 => Self::Lost,
            8 => Self::Albedo,
            9 => Self::Other,
            _ => return None,
        })
    }
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
                _ => Self::Other,
            },
        }
    }
}

impl Default for TrackBundle {
    /// Same as [`TrackBundle::new`]: an empty bundle with `offsets == [0]`.
    fn default() -> Self {
        Self::new()
    }
}

/// Segments of many rays in CSR layout: the MOC / Sn track database. Ray `i`'s segments are
/// `offsets[i]..offsets[i+1]`. Rays appear in input order regardless of thread count.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
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
    /// Segment index range of ray `i`; `None` if `i` is out of range or the offsets are
    /// malformed (non-monotone or past the columns).
    pub fn ray_range(&self, i: usize) -> Option<core::ops::Range<usize>> {
        let a = *self.offsets.get(i)? as usize;
        let b = *self.offsets.get(i.checked_add(1)?)? as usize;
        (a <= b && b <= self.cell.len()).then_some(a..b)
    }
    /// Start a bundle.
    pub fn new() -> Self {
        Self { offsets: vec![0], cell: Vec::new(), region: Vec::new(), length: Vec::new(), end: Vec::new(), end_surface: Vec::new() }
    }
    /// Append one finished ray. Offsets are `u32`: returns `RtError::Unsupported` (and leaves
    /// the bundle unchanged) if the segment count would pass `u32::MAX`.
    pub fn push_ray(&mut self, segs: &[Segment], end: &TraceEnd) -> Result<(), RtError> {
        let total = self.cell.len().checked_add(segs.len()).and_then(|t| u32::try_from(t).ok());
        let Some(total) = total else {
            return Err(RtError::Unsupported("track bundle exceeds u32::MAX segments"));
        };
        for s in segs {
            self.cell.push(s.cell.0);
            self.region.push(s.region.0);
            self.length.push(s.length);
        }
        self.offsets.push(total);
        self.end.push(end.into());
        self.end_surface.push(match end {
            TraceEnd::Boundary { surface, .. } => surface.0,
            _ => SurfaceId::NONE.0,
        });
        Ok(())
    }
    /// Append another bundle's rays after this one's (used to join per-block results in block
    /// order). `RtError::Unsupported` (bundle unchanged) if the joined segment count would pass
    /// `u32::MAX`.
    pub fn append(&mut self, o: &Self) -> Result<(), RtError> {
        let too_big = || RtError::Unsupported("track bundle exceeds u32::MAX segments");
        let base = u32::try_from(self.cell.len()).map_err(|_| too_big())?;
        let added = u32::try_from(o.cell.len()).map_err(|_| too_big())?;
        base.checked_add(added).ok_or_else(too_big)?;
        self.offsets.extend(o.offsets.iter().skip(1).map(|&x| x.saturating_add(base)));
        self.cell.extend_from_slice(&o.cell);
        self.region.extend_from_slice(&o.region);
        self.length.extend_from_slice(&o.length);
        self.end.extend_from_slice(&o.end);
        self.end_surface.extend_from_slice(&o.end_surface);
        Ok(())
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
#[non_exhaustive]
pub struct BinMap {
    /// Number of bins.
    pub n_bins: usize,
    /// Bin per cell.
    pub bin_of_cell: Vec<u32>,
}

impl BinMap {
    /// Cell not in any bin.
    pub const NO_BIN: u32 = u32::MAX;
    /// A map; run [`Self::validate`] (or use [`TrackLengthTally::try_new`]) before tallying.
    pub fn new(n_bins: usize, bin_of_cell: Vec<u32>) -> Self {
        Self { n_bins, bin_of_cell }
    }
    /// Check that every assigned bin is below `n_bins`; `RtError::Build` (`tally.bin_range`)
    /// otherwise.
    pub fn validate(&self) -> Result<(), RtError> {
        let issues: Vec<BuildIssue> = self
            .bin_of_cell
            .iter()
            .enumerate()
            .filter(|&(_, &b)| b != Self::NO_BIN && b as usize >= self.n_bins)
            .take(8)
            .map(|(c, &b)| BuildIssue::new("tally.bin_range", format!("cell {c} maps to bin {b}, but there are {} bins", self.n_bins)))
            .collect();
        if issues.is_empty() { Ok(()) } else { Err(RtError::Build(issues)) }
    }
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
    /// A zeroed tally. Does not validate `bins`: a segment whose bin is out of range is
    /// skipped. Use [`Self::try_new`] to reject such a map up front.
    pub fn new(bins: BinMap) -> Self {
        let n = bins.n_bins;
        Self { bins, acc: vec![0.0; n], weight: 1.0 }
    }
    /// A zeroed tally from a validated map ([`BinMap::validate`]).
    pub fn try_new(bins: BinMap) -> Result<Self, RtError> {
        bins.validate()?;
        Ok(Self::new(bins))
    }
    /// Add `other` into `self`, bin by bin in index order.
    pub fn merge(&mut self, other: &Self) -> Result<(), RtError> {
        if self.acc.len() != other.acc.len() {
            return Err(RtError::BinCountMismatch { left: self.acc.len(), right: other.acc.len() });
        }
        for (a, b) in self.acc.iter_mut().zip(&other.acc) {
            *a += *b;
        }
        Ok(())
    }
}

impl SegmentSink for TrackLengthTally {
    fn segment(&mut self, seg: &Segment) -> Flow {
        if let Some(slot) = self.bins.bin(seg.cell).and_then(|b| self.acc.get_mut(b)) {
            *slot += self.weight * seg.length;
        }
        Flow::Continue
    }
}

/// A tiny helper: advance `st` by `d` along its direction (used by default trait methods).
pub(crate) fn advance(st: &mut TrackState, d: f64) {
    st.pos = st.pos + st.dir * d;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ray_range_is_checked() {
        let mut b = TrackBundle::new();
        let s = Segment::new(CellId(0), RegionId(0), 0.0, 1.5);
        b.push_ray(&[s, s], &TraceEnd::Limit).unwrap();
        b.push_ray(&[], &TraceEnd::LeftModel).unwrap();
        assert_eq!(b.ray_range(0), Some(0..2));
        assert_eq!(b.ray_range(1), Some(2..2));
        assert_eq!(b.ray_range(2), None);
        assert_eq!(b.ray_range(usize::MAX), None);
        let bad = TrackBundle { offsets: vec![0, 9], ..TrackBundle::default() };
        assert_eq!(bad.ray_range(0), None, "offset past the columns");
        let rev = TrackBundle { offsets: vec![3, 1], cell: vec![0; 4], ..TrackBundle::default() };
        assert_eq!(rev.ray_range(0), None, "non-monotone offsets");
    }

    #[test]
    fn merge_rejects_mismatched_bins_with_its_own_error() {
        let mut a = TrackLengthTally::new(BinMap::identity(2));
        let b = TrackLengthTally::new(BinMap::identity(3));
        assert!(matches!(a.merge(&b), Err(RtError::BinCountMismatch { left: 2, right: 3 })));
        let mut c = TrackLengthTally::new(BinMap::identity(2));
        c.acc = vec![1.0, 2.0];
        a.merge(&c).unwrap();
        a.merge(&c).unwrap();
        assert_eq!(a.acc, vec![2.0, 4.0]);
    }

    #[test]
    fn end_codes_are_frozen_and_decode() {
        for (c, v) in [
            (EndCode::Limit, 0u8),
            (EndCode::Stopped, 1),
            (EndCode::LeftModel, 2),
            (EndCode::Vacuum, 3),
            (EndCode::Reflective, 4),
            (EndCode::White, 5),
            (EndCode::Periodic, 6),
            (EndCode::Lost, 7),
            (EndCode::Albedo, 8),
            (EndCode::Other, 9),
        ] {
            assert_eq!(c as u8, v);
            assert_eq!(EndCode::from_u8(v), Some(c));
        }
        assert_eq!(EndCode::from_u8(10), None);
    }

    #[test]
    fn builders_set_fields() {
        let o = TraceOptions::default().with_max_dist(2.0).with_max_steps(7).with_stop_at_deck(true);
        assert_eq!((o.max_dist, o.max_steps, o.stop_at_deck), (2.0, 7, true));
        let s = Segment::new(CellId(1), RegionId(2), 0.5, 1.0).with_faces(Some(FaceId(3)), None);
        assert_eq!((s.face_in, s.face_out), (Some(FaceId(3)), None));
    }

    #[test]
    fn mismatched_bin_map_is_skipped_or_rejected_never_a_panic() {
        let bad = BinMap::new(1, vec![5]);
        assert!(matches!(bad.validate(), Err(RtError::Build(_))));
        assert!(TrackLengthTally::try_new(bad.clone()).is_err());
        let mut t = TrackLengthTally::new(bad);
        let seg = Segment::new(CellId(0), RegionId(0), 0.0, 1.0);
        assert_eq!(t.segment(&seg), Flow::Continue);
        assert_eq!(t.acc, vec![0.0]);
        assert!(BinMap::new(2, vec![0, BinMap::NO_BIN, 1]).validate().is_ok());
    }

    #[test]
    fn unlisted_boundary_actions_map_to_other() {
        let e = TraceEnd::Boundary { surface: SurfaceId(0), face: FaceId(0), action: FaceAction::Transmit };
        assert_eq!(EndCode::from(&e), EndCode::Other);
        assert_eq!(EndCode::from_u8(9), Some(EndCode::Other));
        assert_eq!(EndCode::from_u8(10), None);
    }
}
