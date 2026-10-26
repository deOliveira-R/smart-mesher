//! Value types of the query surface: locations, particle state, events, segments, scratch.
//!
//! Nothing here allocates per query once a [`Scratch`] and a [`TrackState`] exist.

use crate::boundary::ResponseKind;
use crate::error::LostReport;
use smallvec::SmallVec;
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::RayHit;
use sm_topo_api::Region;

/// Index of one *copy* of a volume in the flattened hierarchy (the same table
/// `sm_model_api::flatten` enumerates, in the same order). A pin repeated 50k times has 50k
/// `FlatIx` values but one prototype structure; the hot path never builds an `InstancePath`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FlatIx(pub u32);

/// Where a particle is: a copy of a region. `Region::Exterior` is the implicit complement.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Location {
    /// The copy (prototype placement) the particle is in.
    pub copy: FlatIx,
    /// The volume of the prototype, or the implicit complement.
    pub region: Region,
}

/// A face and the side of it a particle sits on. Replaces DAGMC's `RayHistory` facet list: one
/// small value is enough because exact surfaces need no "already hit" facet memory; consistency
/// of the crossing direction with `positive` rejects the self-hit root at `t ~ 0`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaceSide {
    /// The face (`EntityKind::TopoFace`).
    pub face: EntityId,
    /// Copy whose placement frame the face was crossed in.
    pub copy: FlatIx,
    /// True if the particle is on the side the face normal points to.
    pub positive: bool,
}

/// One level of the current hierarchy path: a container (lattice or cluster, by dense index) and
/// the slot or member the particle is in. Together the levels let a lattice step to the
/// neighbouring slot with index arithmetic (no search).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Level {
    /// Dense index of the lattice or cluster in the tracker's own tables.
    pub container: u32,
    /// Dense slot or member index inside it.
    pub slot: u32,
}

/// Inline capacity of the level stack (pin, assembly, core, plus headroom). Deeper hierarchies
/// still work and then allocate once.
pub const LEVEL_INLINE: usize = 8;

/// The complete, self-contained state of one particle for geometry purposes. Cheap to copy
/// (`Clone`), no references into the tracker, so histories can be banked and resumed.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct TrackState {
    /// World-frame position.
    pub pos: Point3,
    /// World-frame unit direction.
    pub dir: Vector3,
    /// Current location.
    pub at: Location,
    /// Hierarchy path from the root, outermost first.
    pub levels: SmallVec<[Level; LEVEL_INLINE]>,
    /// Last crossed face, if the particle sits on a surface.
    pub last: Option<FaceSide>,
    /// Number of reflect/wrap events so far (a trap guard, also a tally of deck use).
    pub deck_events: u32,
}

impl TrackState {
    /// A state at `pos` going `dir` at `at`, with an empty path. Implementors fill `levels`.
    pub fn new(pos: Point3, dir: Vector3, at: Location) -> Self {
        Self { pos, dir, at, levels: SmallVec::new(), last: None, deck_events: 0 }
    }
}

/// How a deck face acted on the particle (already applied to the state).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum DeckAction {
    /// Mirror: specular reflection about the (possibly curved) face.
    Reflected,
    /// Wrap: position and direction mapped by the deck transform to the mate face.
    Wrapped {
        /// The face the particle re-enters on.
        mate: EntityId,
    },
}

/// What ended a step.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum Event {
    /// Crossed an interior face; the state now sits in `to`.
    Crossing {
        /// The face.
        face: EntityId,
        /// Location before.
        from: Location,
        /// Location after (read from face sides or lattice arithmetic, not by search).
        to: Location,
    },
    /// Entered or left a lattice slot or instance without crossing a model face (the slot wall).
    SlotChange {
        /// Location before.
        from: Location,
        /// Location after.
        to: Location,
    },
    /// A deck face acted (reflect or wrap). The state was updated in place and tracking can
    /// continue; the particle is on `face` (reflect) or on the mate (wrap).
    Deck {
        /// The face that was hit.
        face: EntityId,
        /// What was done.
        action: DeckAction,
    },
    /// A response boundary the consumer owns (vacuum, white, albedo). The state sits on the face
    /// and is not modified: the caller kills, re-emits or reflects the particle, using its own
    /// RNG (the tracker never draws random numbers).
    Response {
        /// The face.
        face: EntityId,
        /// What the boundary asks for.
        kind: ResponseKind,
        /// Unit normal pointing into the model (for sampling a diffuse direction).
        inward: Vector3,
    },
    /// `limit` was reached before any surface.
    LimitReached,
    /// The particle is lost and recovery failed. Same value as `RtError::Lost`, delivered as an
    /// event so batch tracing continues with the other particles.
    Lost(Box<LostReport>),
}

impl Event {
    /// True if tracing the same particle further makes sense without the caller acting.
    pub fn continues(&self) -> bool {
        matches!(self, Event::Crossing { .. } | Event::SlotChange { .. } | Event::Deck { .. })
    }
}

/// One step: the distance moved and the event at its end.
#[derive(Clone, Debug, PartialEq)]
pub struct Step {
    /// Distance travelled in the world frame (exact chord to the surface, `>= 0`).
    pub distance: f64,
    /// What happened at the end.
    pub event: Event,
}

/// Result of the non-mutating query [`crate::Tracker::distance_to_boundary`] (`ray_fire`).
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct BoundaryHit {
    /// Distance to the next face.
    pub distance: f64,
    /// The face.
    pub face: EntityId,
    /// Unit normal at the hit in the world frame, pointing out of the current region.
    pub outward: Vector3,
    /// Location after crossing (`None` for deck and response faces).
    pub next: Option<Location>,
}

impl BoundaryHit {
    /// Construct (the type is `#[non_exhaustive]`).
    pub const fn new(distance: f64, face: EntityId, outward: Vector3, next: Option<Location>) -> Self {
        Self { distance, face, outward, next }
    }
}

/// Result of [`crate::Tracker::nearest_surface`] (`closest_to_location`).
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct Nearest {
    /// Distance to the nearest face of the current region (an upper bound if the
    /// implementor uses bounding boxes for facet faces; exact for analytic faces).
    pub distance: f64,
    /// The face.
    pub face: EntityId,
}

impl Nearest {
    /// Construct (the type is `#[non_exhaustive]`).
    pub const fn new(distance: f64, face: EntityId) -> Self {
        Self { distance, face }
    }
}

/// A piece of a trace: the particle was in `at` for `length` starting at `start`. This is the
/// unit a flux or reaction-rate estimator (track-length tally, MOC segment) consumes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Segment {
    /// Where the piece is.
    pub at: Location,
    /// World-frame start point.
    pub start: Point3,
    /// World-frame unit direction on this piece (changes after reflections).
    pub dir: Vector3,
    /// Chord length.
    pub length: f64,
}

/// Receiver of segments in path order. Implemented by tally code; `&mut dyn` keeps `Tracker`
/// object-safe.
pub trait SegmentSink {
    /// Called once per piece, in order.
    fn segment(&mut self, s: &Segment);
}

impl SegmentSink for Vec<Segment> {
    fn segment(&mut self, s: &Segment) {
        self.push(*s);
    }
}

/// How a whole trace ended.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum TraceEnd {
    /// Total path length `limit` was used up.
    LimitReached,
    /// A response boundary stopped it (vacuum leakage, white, albedo).
    Response {
        /// The face.
        face: EntityId,
        /// The requested behaviour.
        kind: ResponseKind,
    },
    /// Lost; the report says why.
    Lost(Box<LostReport>),
}

/// Reusable per-thread working memory. Create one per worker, pass it to every call; the
/// tracker's hot path allocates nothing once the buffers have grown.
#[derive(Debug, Default)]
pub struct Scratch {
    /// Surface hit buffer handed to `ParametricSurface::ray_hits`.
    pub hits: Vec<RayHit>,
    /// Implementor's candidate list (face or leaf indices), cleared before use.
    pub candidates: Vec<u32>,
}

impl Scratch {
    /// Empty scratch.
    pub fn new() -> Self {
        Self::default()
    }
}

/// Output of [`crate::Tracker::trace_many`]: CSR layout, ray `i` owns
/// `segments[offsets[i]..offsets[i + 1]]`. Identical for any thread count.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TraceBatch {
    /// `rays + 1` offsets into `segments`.
    pub offsets: Vec<u32>,
    /// All segments, ray after ray.
    pub segments: Vec<Segment>,
    /// How each ray ended.
    pub ends: Vec<TraceEnd>,
}

impl TraceBatch {
    /// Segments of ray `i`.
    pub fn of(&self, i: usize) -> &[Segment] {
        &self.segments[self.offsets[i] as usize..self.offsets[i + 1] as usize]
    }
}
