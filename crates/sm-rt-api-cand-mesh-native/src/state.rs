//! Particle position state, step results, and the region/surface tables.

use crate::error::LostInfo;
use crate::ids::{CellId, ComponentId, FaceId, InstanceId, RegionId, SurfaceId};
use sm_core::{EntityId, Point3, Vector3};
use smallvec::SmallVec;

/// Deepest supported hierarchy (world > lattice > assembly > pin ...). OpenMC allows more;
/// four covers core / assembly / pin / sub-pin and keeps `TrackState` small and `Copy`-cheap.
pub const MAX_LEVELS: usize = 4;

/// Boundary condition kind of a domain surface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum BoundaryKind {
    /// Particle leaves the problem.
    Vacuum,
    /// Specular reflection about the face plane (exact for planar faces).
    Reflective,
    /// Re-emitted with a cosine-law direction; needs two uniforms from the caller's RNG.
    White,
    /// Re-enters through the matched partner surface (rigid transform in the tracker).
    Periodic {
        /// The partner surface.
        partner: SurfaceId,
    },
}

/// A particle's place in the hierarchy: one entry per level below the root.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Level {
    /// Component the particle is in at this level.
    pub component: ComponentId,
    /// Placed instance at this level.
    pub instance: InstanceId,
    /// Lattice index at this level, when the component is a lattice.
    pub lattice: [i32; 3],
}

/// Everything a tracker needs to continue a particle, owned by the caller (one per particle /
/// per thread). Replaces DAGMC's `RayHistory`: `entered_via` excludes the face just crossed,
/// so a ray never re-hits its entry face, and no history buffer is needed because adjacency is
/// exact.
#[derive(Clone, Debug, PartialEq)]
pub struct TrackState {
    /// Position (global frame).
    pub pos: Point3,
    /// Unit direction (global frame).
    pub dir: Vector3,
    /// Tracking cell (`CellId::NONE` outside the model, `CellId::COMPLEMENT` in the complement).
    pub cell: CellId,
    /// Region of `cell`.
    pub region: RegionId,
    /// Face crossed to get here, if any.
    pub entered_via: Option<FaceId>,
    /// Hierarchy path (empty for a flat world).
    pub levels: SmallVec<[Level; MAX_LEVELS]>,
}

impl TrackState {
    /// An unlocated state (cell/region are `NONE`); use `Tracker::init_state` to locate it.
    pub fn new(pos: Point3, dir: Vector3) -> Self {
        Self { pos, dir, cell: CellId::NONE, region: RegionId::NONE, entered_via: None, levels: SmallVec::new() }
    }
}

/// What ended a step.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum StepEnd {
    /// The distance limit was reached inside the cell. State is advanced.
    Limit,
    /// Crossed an interior face into a cell of the *same* region. State is in `to`.
    CellFace {
        /// The face crossed.
        face: FaceId,
        /// The cell entered.
        to: CellId,
    },
    /// Crossed into a different region (a DAGMC surface hit). State is in `to_cell`.
    RegionBoundary {
        /// The model surface crossed.
        surface: SurfaceId,
        /// The face crossed.
        face: FaceId,
        /// Region entered.
        to_region: RegionId,
        /// Cell entered (`CellId::COMPLEMENT` or `NONE` possible).
        to_cell: CellId,
    },
    /// Reached the domain boundary. State stays in the last cell, on the face; the caller
    /// applies `Tracker::resolve_boundary`.
    DomainBoundary {
        /// The model surface.
        surface: SurfaceId,
        /// The face.
        face: FaceId,
        /// Its boundary condition.
        kind: BoundaryKind,
    },
    /// The particle is lost; the state is unchanged.
    Lost(LostInfo),
}

/// One step: from the state's position to the next event, inside a single cell.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Step {
    /// Distance travelled (the chord of the cell along the ray).
    pub dist: f64,
    /// The cell traversed.
    pub cell: CellId,
    /// Its region.
    pub region: RegionId,
    /// What ended the step.
    pub end: StepEnd,
}

/// Result of locating a point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Located {
    /// The cell (`NONE` outside the model, `COMPLEMENT` in the implicit complement).
    pub cell: CellId,
    /// Its region.
    pub region: RegionId,
    /// Set when the point is on a face within tolerance (the direction then decided the cell).
    pub on_face: Option<FaceId>,
}

/// Closest-point answer (DAGMC `closest_to_location`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Closest {
    /// Distance to the closest boundary of the region.
    pub dist: f64,
    /// The surface that owns it.
    pub surface: SurfaceId,
    /// The closest point.
    pub point: Point3,
}

/// One material region (a volume).
#[derive(Clone, Debug, PartialEq)]
pub struct RegionInfo {
    /// The B-rep volume (or `MeshBlock`) it comes from.
    pub entity: Option<EntityId>,
    /// Name / material tag (DAGMC `mat:fuel`).
    pub name: String,
    /// True for the implicit complement.
    pub implicit_complement: bool,
}

/// One model surface.
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceInfo {
    /// The B-rep face or boundary set.
    pub entity: Option<EntityId>,
    /// Regions on the front (normal side) and back; `NONE` means outside. This is DAGMC `next_vol`.
    pub front: RegionId,
    /// Back region.
    pub back: RegionId,
    /// Boundary condition when the surface is a domain boundary.
    pub boundary: Option<BoundaryKind>,
}

/// Region table, dense by `RegionId`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RegionTable(pub Vec<RegionInfo>);

impl RegionTable {
    /// Info for a region.
    pub fn get(&self, r: RegionId) -> Option<&RegionInfo> {
        self.0.get(r.get())
    }
    /// The implicit complement region, if any.
    pub fn implicit_complement(&self) -> Option<RegionId> {
        self.0.iter().position(|r| r.implicit_complement).map(RegionId::new)
    }
}

/// Surface table, dense by `SurfaceId`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SurfaceTable(pub Vec<SurfaceInfo>);

impl SurfaceTable {
    /// Info for a surface.
    pub fn get(&self, s: SurfaceId) -> Option<&SurfaceInfo> {
        self.0.get(s.get())
    }
    /// DAGMC `next_vol`: the region reached by leaving `from` through `s`.
    pub fn next_region(&self, s: SurfaceId, from: RegionId) -> Option<RegionId> {
        let i = self.get(s)?;
        if i.front == from { Some(i.back) } else if i.back == from { Some(i.front) } else { None }
    }
}

/// What a tracker can do, as a bit set. Callers check this instead of probing for errors.
/// New capabilities are new bits (additive).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct TrackerCaps(pub u32);

impl TrackerCaps {
    /// Has a polyhedral volume mesh: cell-to-cell walking, `mesh()` is `Some`, tallies bin by mesh cell.
    pub const VOLUME_MESH: Self = Self(1);
    /// Cell chords are exact (planar faces, exact adjacency); absent for a faceted-surface
    /// tracker that approximates curved surfaces by triangles.
    pub const EXACT_PLANAR: Self = Self(2);
    /// Curved-surface hits are refined against the analytic B-rep surface.
    pub const ANALYTIC_REFINE: Self = Self(4);
    /// Supports lattices / instancing without flattening.
    pub const HIERARCHY: Self = Self(8);
    /// Supports periodic boundaries (`Tracker::periodic_image`).
    pub const PERIODIC: Self = Self(16);
    /// Exact analytic primitives (stub / CSG-like).
    pub const ANALYTIC: Self = Self(32);
    /// Union of two sets.
    pub const fn union(self, o: Self) -> Self {
        Self(self.0 | o.0)
    }
    /// True if every bit of `o` is set.
    pub const fn has(self, o: Self) -> bool {
        self.0 & o.0 == o.0
    }
}
