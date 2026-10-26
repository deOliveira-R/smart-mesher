//! Particle position state, step results, caller-owned scratch, and the region/surface tables.

use crate::boundary::FaceAction;
use crate::error::LostInfo;
use crate::ids::{CellId, ComponentId, FaceId, InstanceId, RegionId, SurfaceId};
use sm_core::{EntityId, Point3, Vector3};
use smallvec::SmallVec;

/// Hierarchy levels stored inline in a [`TrackState`]. Deeper paths spill to the heap (still
/// deterministic); there is no hard depth cap. Eight covers core / assembly / pin / sub-pin with
/// room to spare. `WorldSpec::validate` only warns above it.
pub const LEVEL_INLINE: usize = 8;

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
/// per thread). Replaces DAGMC's `RayHistory`: `entered_via` excludes the face just crossed, so
/// a ray never re-hits its entry face, and no history buffer is needed because adjacency is
/// exact.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
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
    /// Hierarchy path (empty for a flat world). Inline up to [`LEVEL_INLINE`] levels.
    pub levels: SmallVec<[Level; LEVEL_INLINE]>,
}

impl TrackState {
    /// An unlocated state (cell/region are `NONE`); use `Tracker::init_state` to locate it.
    pub fn new(pos: Point3, dir: Vector3) -> Self {
        Self { pos, dir, cell: CellId::NONE, region: RegionId::NONE, entered_via: None, levels: SmallVec::new() }
    }
}

/// Caller-owned working memory for queries, one per thread, reused across calls.
///
/// # No-allocation contract
/// After a warm-up (the first few queries may grow the buffers), `locate`, `step`,
/// `distance_to_boundary`, `closest` and the provided methods built on them perform **no heap
/// allocation** while the hierarchy depth is at most [`LEVEL_INLINE`]. Implementors put every
/// temporary in a `Scratch` field, never in a local `Vec`. Contents are meaningless between
/// calls; a `Scratch` carries no state a result depends on, so sharing one between unrelated
/// queries of the same thread is safe and results are bit-identical with a fresh one.
#[derive(Clone, Debug, Default)]
#[non_exhaustive]
pub struct Scratch {
    /// General `(parameter, id)` buffer (candidate hits along a ray).
    pub hits: Vec<(f64, u32)>,
    /// General id buffer (candidate faces, BVH stack).
    pub ids: Vec<u32>,
    /// General real buffer.
    pub work: Vec<f64>,
}

impl Scratch {
    /// Empty scratch.
    pub fn new() -> Self {
        Self::default()
    }
    /// Scratch with capacity for `n` entries in each buffer.
    pub fn with_capacity(n: usize) -> Self {
        Self { hits: Vec::with_capacity(n), ids: Vec::with_capacity(n), work: Vec::with_capacity(n) }
    }
    /// Empty every buffer, keeping capacity.
    pub fn clear(&mut self) {
        self.hits.clear();
        self.ids.clear();
        self.work.clear();
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
    /// Crossed a lattice or instance wall that is not a model face: the cell changes, the
    /// region need not. State is in `to_cell`, `levels` updated.
    SlotChange {
        /// The cell entered.
        to_cell: CellId,
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
    /// Reached a boundary surface of the model. The single boundary event; read `action`:
    ///
    /// * [`FaceAction::Reflect`] and [`FaceAction::Wrap`] were **applied inside `step`**: the
    ///   state is already reflected, or moved through the mate face (position, direction, cell,
    ///   levels updated).
    /// * [`FaceAction::Respond`] was **not** applied: the state sits on the face in the last cell
    ///   and the caller calls `Tracker::resolve_boundary`.
    ///
    /// `Transmit` never appears here. See [`crate::boundary`].
    Boundary {
        /// The model surface.
        surface: SurfaceId,
        /// The face.
        face: FaceId,
        /// What the surface does.
        action: FaceAction,
    },
    /// The particle is lost; the state is unchanged.
    Lost(LostInfo),
}

/// One step: from the state's position to the next event, inside a single cell.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
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

impl Step {
    /// A step of length `dist` through `cell` of `region`, ended by `end`.
    pub fn new(dist: f64, cell: CellId, region: RegionId, end: StepEnd) -> Self {
        Self { dist, cell, region, end }
    }
}

/// Result of locating a point.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct Located {
    /// The cell (`NONE` outside the model, `COMPLEMENT` in the implicit complement).
    pub cell: CellId,
    /// Its region.
    pub region: RegionId,
    /// Set when the point is on a face within tolerance (the direction then decided the cell).
    pub on_face: Option<FaceId>,
    /// Hierarchy path of the cell (empty for a flat tracker); `init_state` copies it into the
    /// state.
    pub levels: SmallVec<[Level; LEVEL_INLINE]>,
}

impl Located {
    /// A flat result (no hierarchy path).
    pub fn new(cell: CellId, region: RegionId, on_face: Option<FaceId>) -> Self {
        Self { cell, region, on_face, levels: SmallVec::new() }
    }
    /// The same with a hierarchy path.
    pub fn with_levels(mut self, levels: SmallVec<[Level; LEVEL_INLINE]>) -> Self {
        self.levels = levels;
        self
    }
}

/// Closest-point answer (DAGMC `closest_to_location`).
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct Closest {
    /// Distance to the closest boundary of the region.
    pub dist: f64,
    /// The surface that owns it.
    pub surface: SurfaceId,
    /// The closest point.
    pub point: Point3,
}

impl Closest {
    /// A closest-point answer.
    pub fn new(dist: f64, surface: SurfaceId, point: Point3) -> Self {
        Self { dist, surface, point }
    }
}

/// The next surface a ray reaches, without moving (see `Tracker::distance_to_boundary`).
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct BoundaryHit {
    /// Distance along the ray.
    pub dist: f64,
    /// The model surface.
    pub surface: SurfaceId,
    /// The face.
    pub face: FaceId,
    /// Region entered across a region boundary; `RegionId::NONE` at a model boundary.
    pub to_region: RegionId,
    /// What the surface does when it is a model boundary (`None` for a region interface).
    pub action: Option<FaceAction>,
}

impl BoundaryHit {
    /// A hit at `dist` on `surface`/`face`, entering `to_region`, with the boundary `action` if
    /// the surface is a model boundary.
    pub fn new(dist: f64, surface: SurfaceId, face: FaceId, to_region: RegionId, action: Option<FaceAction>) -> Self {
        Self { dist, surface, face, to_region, action }
    }
}

/// Result of a peek along a ray.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum Peek {
    /// No surface within the limit.
    Limit,
    /// The next surface.
    Hit(BoundaryHit),
    /// Tracking failed (a value, not an error).
    Lost(LostInfo),
}

/// One material region (a volume).
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct RegionInfo {
    /// The B-rep volume (or `MeshBlock`) it comes from.
    pub entity: Option<EntityId>,
    /// Name / material tag (DAGMC `mat:fuel`).
    pub name: String,
    /// True for the implicit complement.
    pub implicit_complement: bool,
}

impl RegionInfo {
    /// A named ordinary region (not the implicit complement), with no model entity.
    pub fn new(name: impl Into<String>) -> Self {
        Self { entity: None, name: name.into(), implicit_complement: false }
    }
    /// Link the region to its model entity.
    pub fn with_entity(mut self, e: EntityId) -> Self {
        self.entity = Some(e);
        self
    }
    /// Mark the region as the implicit complement.
    pub fn as_implicit_complement(mut self) -> Self {
        self.implicit_complement = true;
        self
    }
}

/// One model surface.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct SurfaceInfo {
    /// The B-rep face or boundary set.
    pub entity: Option<EntityId>,
    /// Region the surface normal points **out of** (the DAGMC "forward" volume, sense `+1`;
    /// `sm_topo_api::FaceSides::left`). `NONE` means outside the model.
    pub front: RegionId,
    /// Region the normal points **into** (`FaceSides::right`; sense `-1`).
    pub back: RegionId,
    /// What the surface does when it is a model boundary (`None` for a region interface).
    pub boundary: Option<FaceAction>,
}

impl SurfaceInfo {
    /// A surface with `front` (normal points out of) and `back` (points into) regions, no
    /// entity and no boundary action.
    pub fn new(front: RegionId, back: RegionId) -> Self {
        Self { entity: None, front, back, boundary: None }
    }
    /// Link the surface to its model entity.
    pub fn with_entity(mut self, e: EntityId) -> Self {
        self.entity = Some(e);
        self
    }
    /// Make the surface a model boundary with `action`.
    pub fn with_boundary(mut self, action: FaceAction) -> Self {
        self.boundary = Some(action);
        self
    }
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
    /// DAGMC `surface_sense`: `+1` if the surface normal points out of `region` (`region` is
    /// `front`), `-1` if it points into it (`back`), `None` if the region does not touch it.
    pub fn sense(&self, s: SurfaceId, region: RegionId) -> Option<i32> {
        let i = self.get(s)?;
        if i.front == region { Some(1) } else if i.back == region { Some(-1) } else { None }
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
    /// Curved-surface hits are refined against the analytic B-rep surface. A refined hit is
    /// re-validated against the planar cell: if it falls outside, the planar hit is used and
    /// counted in diagnostics.
    pub const ANALYTIC_REFINE: Self = Self(4);
    /// Supports lattices / instancing without flattening.
    pub const HIERARCHY: Self = Self(8);
    /// Applies `FaceAction::Wrap` (translation / rotation decks) inside `step`.
    pub const PERIODIC: Self = Self(16);
    /// Exact analytic primitives: regions bounded by `sm_geom_api` `ray_hits`, not only mesh cells.
    pub const ANALYTIC: Self = Self(32);
    /// Traversal in `f32` boxes with `f64` confirmation (`Precision::F32Traversal`).
    pub const F32_TRAVERSAL: Self = Self(64);
    /// Union of two sets.
    pub const fn union(self, o: Self) -> Self {
        Self(self.0 | o.0)
    }
    /// True if every bit of `o` is set.
    pub const fn has(self, o: Self) -> bool {
        self.0 & o.0 == o.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deep_level_stack_spills_and_stays_equal() {
        let mut st = TrackState::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        for i in 0..LEVEL_INLINE {
            st.levels.push(Level { component: ComponentId(i as u32), instance: InstanceId(0), lattice: [i as i32, 0, 0] });
        }
        assert!(!st.levels.spilled());
        for i in LEVEL_INLINE..12 {
            st.levels.push(Level { component: ComponentId(i as u32), instance: InstanceId(1), lattice: [0, i as i32, 0] });
        }
        assert!(st.levels.spilled() && st.levels.len() == 12);
        let c = st.clone();
        assert_eq!(c, st);
        assert_eq!(c.levels[11].lattice, [0, 11, 0]);
    }

    #[test]
    fn surface_table_sense_and_next_region() {
        let t = SurfaceTable(vec![SurfaceInfo::new(RegionId(0), RegionId(1))]);
        let s = SurfaceId(0);
        assert_eq!((t.sense(s, RegionId(0)), t.sense(s, RegionId(1)), t.sense(s, RegionId(2))), (Some(1), Some(-1), None));
        assert_eq!(
            (t.next_region(s, RegionId(0)), t.next_region(s, RegionId(1)), t.next_region(s, RegionId(2))),
            (Some(RegionId(1)), Some(RegionId(0)), None)
        );
    }

    #[test]
    fn caps_bits_are_distinct() {
        let all = [
            TrackerCaps::VOLUME_MESH,
            TrackerCaps::EXACT_PLANAR,
            TrackerCaps::ANALYTIC_REFINE,
            TrackerCaps::HIERARCHY,
            TrackerCaps::PERIODIC,
            TrackerCaps::ANALYTIC,
            TrackerCaps::F32_TRAVERSAL,
        ];
        let u = all.iter().fold(TrackerCaps::default(), |a, &b| a.union(b));
        assert_eq!(u.0.count_ones() as usize, all.len());
        assert!(u.has(TrackerCaps::HIERARCHY) && !TrackerCaps::default().has(TrackerCaps::HIERARCHY));
    }
}
