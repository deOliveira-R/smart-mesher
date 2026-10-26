//! Query argument and result types.

use crate::boundary::Boundary;
use crate::handle::{FacetId, SurfHandle, VolHandle};
use sm_core::{Point3, Vector3};

/// Which crossings `ray_fire` accepts (DAGMC `ray_orientation`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum RayOrientation {
    /// Only hits where the ray leaves the volume (`dir . n * sense > 0`). DAGMC `1`, the default
    /// for particle tracking.
    #[default]
    Exiting,
    /// Only hits where the ray enters the volume. DAGMC `-1`.
    Entering,
    /// Any non-tangent crossing. DAGMC `0`.
    Any,
}

/// Options of [`crate::World::ray_fire`].
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct FireOptions {
    /// Ignore hits farther than this. `None` is unlimited (DAGMC `0`).
    pub dist_limit: Option<f64>,
    /// Accepted crossing orientation.
    pub orientation: RayOrientation,
}

/// The facets a ray must not hit again (DAGMC `RayHistory`). Reusable: clearing keeps capacity,
/// so the hot loop allocates nothing after warm-up.
///
/// Protocol: pass the same history to consecutive `ray_fire` calls while a particle stays on or
/// near a surface. `ray_fire` appends the hit facet. Call [`RayHistory::reset`] when the
/// particle scatters away from the surface, [`RayHistory::reset_to_last_intersection`] when it
/// stays on the last hit surface (e.g. after reflection), and
/// [`RayHistory::rollback_last_intersection`] to undo the last crossing (DAGMC `rollback_last_intersection`).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RayHistory {
    facets: Vec<FacetId>,
}

impl RayHistory {
    /// Empty history.
    pub const fn new() -> Self {
        Self { facets: Vec::new() }
    }
    /// Forget everything.
    pub fn reset(&mut self) {
        self.facets.clear();
    }
    /// Keep only the most recent facet.
    pub fn reset_to_last_intersection(&mut self) {
        if let Some(&last) = self.facets.last() {
            self.facets.clear();
            self.facets.push(last);
        }
    }
    /// Drop the most recent facet.
    pub fn rollback_last_intersection(&mut self) {
        self.facets.pop();
    }
    /// Record a hit (backends call this from `ray_fire`).
    pub fn add(&mut self, f: FacetId) {
        self.facets.push(f);
    }
    /// True if `f` is excluded.
    pub fn contains(&self, f: FacetId) -> bool {
        self.facets.contains(&f)
    }
    /// Most recent facet.
    pub fn last(&self) -> Option<FacetId> {
        self.facets.last().copied()
    }
    /// Number of facets.
    pub fn len(&self) -> usize {
        self.facets.len()
    }
    /// True if empty.
    pub fn is_empty(&self) -> bool {
        self.facets.is_empty()
    }
    /// The recorded facets, oldest first.
    pub fn facets(&self) -> &[FacetId] {
        &self.facets
    }
}

/// A ray/surface hit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    /// The surface hit (DAGMC `next_surf`).
    pub surface: SurfHandle,
    /// The facet hit (also appended to the history).
    pub facet: FacetId,
    /// Distance along the unit direction (DAGMC `next_surf_dist`).
    pub dist: f64,
}

/// Result of [`crate::World::closest_to_location`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Closest {
    /// Distance to the nearest point of the volume's boundary.
    pub dist: f64,
    /// The surface holding it.
    pub surface: SurfHandle,
}

/// One piece of a traced ray inside one volume.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Segment {
    /// The volume.
    pub volume: VolHandle,
    /// Distance from the trace start where the segment begins.
    pub t0: f64,
    /// Distance from the trace start where it ends.
    pub t1: f64,
    /// The surface that ends the segment; `None` if the trace ended inside the volume.
    pub exit: Option<SurfHandle>,
}

impl Segment {
    /// `t1 - t0`.
    pub fn length(&self) -> f64 {
        self.t1 - self.t0
    }
}

/// Options of [`crate::World::trace`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TraceOptions {
    /// Stop after this distance (`f64::INFINITY` for none).
    pub max_dist: f64,
    /// Fail with `TooManyCrossings` after this many crossings (guards against a stuck ray).
    pub max_crossings: u32,
}

impl Default for TraceOptions {
    fn default() -> Self {
        Self { max_dist: f64::INFINITY, max_crossings: 1_000_000 }
    }
}

/// Why a trace stopped.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum TraceEnd {
    /// Reached `max_dist` inside a volume.
    Distance,
    /// Reached a non-interior boundary (vacuum, reflecting, periodic ...). The caller applies it
    /// with [`crate::World::apply_boundary`].
    Boundary {
        /// The boundary surface.
        surface: SurfHandle,
        /// Its condition.
        kind: Boundary,
        /// Distance from the trace start.
        t: f64,
    },
    /// Left through the unbounded implicit complement: the ray never hits anything again.
    Escaped,
    /// A bounded volume had no exit along the ray: a gap in the geometry (a lost particle).
    Lost {
        /// The volume the ray was in.
        volume: VolHandle,
        /// Distance from the trace start where it was last seen.
        t: f64,
    },
    /// The segment sink asked to stop.
    Stopped,
}

/// Summary of a finished trace.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TraceOutcome {
    /// Why it stopped.
    pub end: TraceEnd,
    /// Total distance traced.
    pub dist: f64,
    /// The volume the ray is in at the end.
    pub volume: VolHandle,
    /// Number of surface crossings.
    pub crossings: u32,
}

/// Convenience: a particle position and direction pair for batch calls.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayIn {
    /// Volume the ray starts in.
    pub volume: VolHandle,
    /// Start.
    pub origin: Point3,
    /// Unit direction.
    pub dir: Vector3,
}
