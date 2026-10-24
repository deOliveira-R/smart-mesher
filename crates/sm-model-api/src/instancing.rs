//! Instancing: prototypes, instances, lattices (cartesian + hex, sparse, nestable), clusters
//! (fills at explicit transforms, nestable: TRISO in a pebble, pebbles in a core), recoloring and
//! a deterministic flatten whose copies are named by instance paths (no ids are minted for
//! copies, so ids are never reused).
//!
//! Instancing is a group action on a fundamental piece, like symmetry: [`Placement`] and
//! [`unfold`](mod@crate::unfold) share the same copy description.
//!
//! **Recolor order** (inner first, outer last): prototype color, then the inner slot or cluster
//! member, outer slot(s) / member(s), the instance, and finally [`PathOverride`]s (deeper prefixes first). Within one
//! [`ColorMap`] a per-volume entry beats a per-color entry. A volume color may only map to a
//! volume color, and a face color to a face color (checked on insertion and by the validator).

use crate::error::{ModelError, Result, expect_kind};
use crate::model::Model;
use sm_core::{EntityId, EntityKind, Transform, Vector3};
use smallvec::SmallVec;
use std::collections::{BTreeMap, BTreeSet};

/// A rigid placement of a copy and whether it reverses orientation (improper).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Placement {
    /// Transform from the prototype/fundamental frame to the model frame.
    pub transform: Transform,
    /// True if `transform` is improper: consumers reverse face loops of the copy.
    pub orientation_reversed: bool,
}

impl Placement {
    /// The identity placement.
    pub fn identity() -> Self {
        Self { transform: Transform::IDENTITY, orientation_reversed: false }
    }

    /// Placement for a transform.
    pub fn of(transform: Transform) -> Self {
        Self { transform, orientation_reversed: transform.is_improper() }
    }
}

/// Index of a lattice slot. Cartesian: `(i, j, k)` from 0. Hex: axial `(q, r)` in `i, j`, layer `k`.
/// Ordered by `(k, j, i)`: the canonical slot order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct SlotIx {
    /// First index (cartesian x; hex q).
    pub i: i32,
    /// Second index (cartesian y; hex r).
    pub j: i32,
    /// Layer.
    pub k: i32,
}

impl SlotIx {
    /// Construct a slot index.
    pub const fn new(i: i32, j: i32, k: i32) -> Self {
        Self { i, j, k }
    }
}

impl Ord for SlotIx {
    fn cmp(&self, o: &Self) -> core::cmp::Ordering {
        (self.k, self.j, self.i).cmp(&(o.k, o.j, o.i))
    }
}
impl PartialOrd for SlotIx {
    fn partial_cmp(&self, o: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(o))
    }
}

/// One step of an instance path.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub enum PathStep {
    /// A top-level `Instance`.
    Instance(EntityId),
    /// A slot of a lattice.
    Slot {
        /// The lattice.
        lattice: EntityId,
        /// The slot.
        ix: SlotIx,
    },
    /// A member of a [`Cluster`].
    Member {
        /// The cluster.
        cluster: EntityId,
        /// Index into [`Cluster::members`].
        ix: u32,
    },
}

/// Stable name of a flattened copy: no minted ids. Paths compare in emission order.
pub type InstancePath = SmallVec<[PathStep; 4]>;

/// Hex row orientation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub enum HexOrient {
    /// Rows along x: `(x, y) = pitch * (q + r/2, sqrt(3)/2 r)`.
    XRows,
    /// Rows along y: `(x, y) = pitch * (sqrt(3)/2 q, r + q/2)`.
    YRows,
}

/// Slot geometry of a lattice. Slot `(0, 0, 0)` is at the lattice origin.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub enum Layout {
    /// `n[0] x n[1] x n[2]` slots, indices from 0.
    Cartesian {
        /// Slot counts per axis (at least 1).
        n: [u32; 3],
        /// Center spacing per axis.
        pitch: Vector3,
    },
    /// Hex (axial `(q, r)`) with `rings` rings (1 = a single slot, 5 = 61 slots) and `nz` layers.
    Hex {
        /// Number of rings, at least 1: `max(|q|, |r|, |q+r|) < rings` are in range.
        rings: u32,
        /// Center-to-center distance in the plane.
        pitch: f64,
        /// Row orientation.
        orient: HexOrient,
        /// Number of axial layers, at least 1.
        nz: u32,
        /// Axial spacing.
        dz: f64,
    },
}

/// Largest slot count a stored [`Layout`] may have.
pub const MAX_LAYOUT_SLOTS: usize = 1 << 22;

impl Layout {
    /// All in-range slots in canonical order `(k, j, i)`. Empty for a layout over
    /// [`MAX_LAYOUT_SLOTS`] (such a layout cannot be stored in a model).
    pub fn indices(&self) -> Vec<SlotIx> {
        if self.slot_count_checked().is_none() {
            return Vec::new();
        }
        let mut v = Vec::new();
        match *self {
            Layout::Cartesian { n, .. } => {
                for k in 0..n[2] as i32 {
                    for j in 0..n[1] as i32 {
                        for i in 0..n[0] as i32 {
                            v.push(SlotIx::new(i, j, k));
                        }
                    }
                }
            }
            Layout::Hex { rings, nz, .. } => {
                let m = rings as i32 - 1;
                for k in 0..nz as i32 {
                    for j in -m..=m {
                        for i in -m..=m {
                            if (i + j).abs() <= m {
                                v.push(SlotIx::new(i, j, k));
                            }
                        }
                    }
                }
            }
        }
        v
    }

    /// Number of in-range slots (saturating at `usize::MAX`).
    pub fn slot_count(&self) -> usize {
        self.slot_count_u128().try_into().unwrap_or(usize::MAX)
    }

    fn slot_count_u128(&self) -> u128 {
        match *self {
            Layout::Cartesian { n, .. } => n.iter().map(|x| u128::from(*x)).product(),
            Layout::Hex { rings, nz, .. } => {
                let r = u128::from(rings);
                (3 * r * r.saturating_sub(1) + 1) * u128::from(nz)
            }
        }
    }

    /// The slot count if it is within [`MAX_LAYOUT_SLOTS`].
    fn slot_count_checked(&self) -> Option<usize> {
        usize::try_from(self.slot_count_u128()).ok().filter(|n| *n <= MAX_LAYOUT_SLOTS)
    }

    /// True if `ix` is inside the layout.
    pub fn contains(&self, ix: SlotIx) -> bool {
        match *self {
            Layout::Cartesian { n, .. } => [ix.i, ix.j, ix.k].iter().zip(n).all(|(a, b)| *a >= 0 && (*a as u32) < b),
            Layout::Hex { rings, nz, .. } => {
                let m = rings as i32 - 1;
                ix.i.abs() <= m && ix.j.abs() <= m && (ix.i + ix.j).abs() <= m && ix.k >= 0 && (ix.k as u32) < nz
            }
        }
    }

    /// Slot center relative to the lattice origin.
    pub fn offset(&self, ix: SlotIx) -> Vector3 {
        match *self {
            Layout::Cartesian { pitch, .. } => Vector3::new(f64::from(ix.i) * pitch.x, f64::from(ix.j) * pitch.y, f64::from(ix.k) * pitch.z),
            Layout::Hex { pitch, orient, dz, .. } => {
                let (q, r) = (f64::from(ix.i), f64::from(ix.j));
                let h = 3f64.sqrt() / 2.0;
                let (x, y) = match orient {
                    HexOrient::XRows => (q + r / 2.0, h * r),
                    HexOrient::YRows => (h * q, r + q / 2.0),
                };
                Vector3::new(pitch * x, pitch * y, f64::from(ix.k) * dz)
            }
        }
    }

    pub(crate) fn check(&self) -> Result<()> {
        let ok = match *self {
            Layout::Cartesian { n, pitch } => n.iter().all(|d| *d >= 1) && [pitch.x, pitch.y, pitch.z].iter().all(|p| p.is_finite()),
            Layout::Hex { rings, nz, pitch, dz, .. } => rings >= 1 && nz >= 1 && pitch.is_finite() && dz.is_finite(),
        };
        if !ok {
            return Err(ModelError::Invalid("lattice layout needs at least one slot per axis and finite pitches".into()));
        }
        if self.slot_count_checked().is_none() {
            return Err(ModelError::Invalid(format!("lattice layout has more than {MAX_LAYOUT_SLOTS} slots")));
        }
        Ok(())
    }
}

/// What fills an instance, a lattice slot or a cluster member.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub enum Fill {
    /// A [`Prototype`].
    Prototype(EntityId),
    /// A nested [`Lattice`].
    Lattice(EntityId),
    /// Nothing (a hole).
    Void,
    /// A nested [`Cluster`] (irregular placements).
    Cluster(EntityId),
}

/// A recoloring, typed by domain: `volume` maps volume colors (or single `TopoVolume`s) to volume
/// colors; `face` maps face colors (or single `TopoFace`s) to face colors. A per-entity key beats
/// a per-color key.
///
/// `#[non_exhaustive]` (W0g r3): build it with [`ColorMap::new`] (or `Default`) plus
/// [`ColorMap::with_volume`] / [`ColorMap::with_face`]. A per-domain field added later is
/// `#[serde(default, skip_serializing_if = "BTreeMap::is_empty")]` and enters `canonical_visit`
/// only when non-empty, so stored models and their hashes are unchanged.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct ColorMap {
    /// Volume recoloring: key `TopoVolume` or volume `Color`; value volume `Color`.
    pub volume: BTreeMap<EntityId, EntityId>,
    /// Face recoloring: key `TopoFace` or face `Color`; value face `Color`.
    pub face: BTreeMap<EntityId, EntityId>,
}

impl ColorMap {
    /// The empty recoloring.
    pub fn new() -> Self {
        Self::default()
    }

    /// With the volume recoloring `volume`.
    #[must_use]
    pub fn with_volume(mut self, volume: BTreeMap<EntityId, EntityId>) -> Self {
        self.volume = volume;
        self
    }

    /// With the face recoloring `face`.
    #[must_use]
    pub fn with_face(mut self, face: BTreeMap<EntityId, EntityId>) -> Self {
        self.face = face;
        self
    }

    /// True if the map recolors nothing.
    pub fn is_empty(&self) -> bool {
        self.volume.is_empty() && self.face.is_empty()
    }

    /// Apply to a volume `v` currently colored `c`.
    pub fn apply_volume(&self, v: EntityId, c: EntityId) -> EntityId {
        self.volume.get(&v).or_else(|| self.volume.get(&c)).copied().unwrap_or(c)
    }

    /// Apply to a face `f` currently colored `c`.
    pub fn apply_face(&self, f: EntityId, c: EntityId) -> EntityId {
        self.face.get(&f).or_else(|| self.face.get(&c)).copied().unwrap_or(c)
    }
}

/// A slot of a lattice: what fills it and how it is recolored.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct Slot {
    /// The fill.
    pub fill: Fill,
    /// Recoloring of everything below this slot.
    pub recolor: ColorMap,
    /// A live `TopoVolume` whose region bounds this slot's fill (the copies are truncated to
    /// it, like an MCNP/OpenMC container cell). It lies in the **lattice frame** (the frame the
    /// slot offsets are measured in), so [`flatten`] places it with the lattice placement.
    /// `None`: the fill is not clipped at this level.
    #[cfg_attr(feature = "serde", serde(default))]
    pub clip: Option<EntityId>,
}

impl Slot {
    /// A slot filled with `fill`, no recoloring, not clipped.
    pub fn new(fill: Fill) -> Self {
        Self { fill, recolor: ColorMap::default(), clip: None }
    }
    /// Set the recoloring.
    pub fn with_recolor(mut self, recolor: ColorMap) -> Self {
        self.recolor = recolor;
        self
    }
    /// Clip the fill to a volume in the lattice frame.
    pub fn with_clip(mut self, clip: EntityId) -> Self {
        self.clip = Some(clip);
        self
    }
}

/// A lattice: a layout plus a **sparse** slot table. In-range slots without an entry take `outer`.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct Lattice {
    /// Slot geometry.
    pub layout: Layout,
    /// Placement of the lattice origin in its parent frame.
    pub origin: Transform,
    /// Fill of slots without an entry.
    pub outer: Fill,
    /// Explicit slots.
    #[cfg_attr(feature = "serde", serde(with = "crate::serde_pairs"))]
    #[cfg_attr(feature = "schemars", schemars(with = "Vec<(SlotIx, Slot)>"))]
    pub slots: BTreeMap<SlotIx, Slot>,
}

impl Lattice {
    /// A lattice with a layout, origin placement, outer fill and explicit slots.
    pub fn new(layout: Layout, origin: Transform, outer: Fill, slots: BTreeMap<SlotIx, Slot>) -> Self {
        Self { layout, origin, outer, slots }
    }
}

/// One member of a [`Cluster`]: a fill at an explicit placement in the cluster frame.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct ClusterMember {
    /// The fill.
    pub fill: Fill,
    /// Placement of the fill in the cluster frame (may be improper).
    pub at: Transform,
    /// Recoloring of everything below this member (applied like a slot's).
    pub recolor: ColorMap,
    /// A live `TopoVolume` bounding this member's fill, in the **cluster frame** (it does not
    /// move with `at`), so [`flatten`] places it with the cluster placement. `None`: not clipped.
    #[cfg_attr(feature = "serde", serde(default))]
    pub clip: Option<EntityId>,
}

impl ClusterMember {
    /// A member `fill` placed at `at`, no recoloring, not clipped.
    pub fn new(fill: Fill, at: Transform) -> Self {
        Self { fill, at, recolor: ColorMap::default(), clip: None }
    }
    /// Set the recoloring.
    pub fn with_recolor(mut self, recolor: ColorMap) -> Self {
        self.recolor = recolor;
        self
    }
    /// Clip the fill to a volume in the cluster frame.
    pub fn with_clip(mut self, clip: EntityId) -> Self {
        self.clip = Some(clip);
        self
    }
}

/// Irregular placements nested inside a fill (OpenMC/Serpent universes, the CSG front end):
/// TRISO particles in a pebble, pebbles in a core, a rotated lattice slot (a one-member
/// cluster). Members flatten in index order and their paths use [`PathStep::Member`].
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct Cluster {
    /// The members, in flatten order.
    pub members: Vec<ClusterMember>,
}

impl Cluster {
    /// A cluster of `members`, in flatten order.
    pub fn new(members: Vec<ClusterMember>) -> Self {
        Self { members }
    }
}

/// A reusable definition: volumes of the topology (stored once, placed many times).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct Prototype {
    /// The volumes.
    pub volumes: BTreeSet<EntityId>,
}

impl Prototype {
    /// A prototype of `volumes`.
    pub fn new(volumes: BTreeSet<EntityId>) -> Self {
        Self { volumes }
    }
}

/// A placement of a [`Fill`] at the top level.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct InstanceRow {
    /// What is placed.
    pub of: Fill,
    /// Placement (may be improper: a mirrored instance).
    pub at: Transform,
    /// Recoloring of everything below.
    pub recolor: ColorMap,
    /// A live `TopoVolume` whose region bounds the fill (an MCNP/OpenMC container cell, or a
    /// lattice truncated by a core barrel). It lies in the model frame (the instance's parent
    /// frame; it does not move with `at`). `None`: not clipped.
    #[cfg_attr(feature = "serde", serde(default))]
    pub clip: Option<EntityId>,
}

impl InstanceRow {
    /// An instance of `fill` placed at `at`, no recoloring, not clipped.
    pub fn new(fill: Fill, at: Transform) -> Self {
        Self { of: fill, at, recolor: ColorMap::default(), clip: None }
    }
    /// Set the recoloring.
    pub fn with_recolor(mut self, recolor: ColorMap) -> Self {
        self.recolor = recolor;
        self
    }
    /// Clip the fill to a volume in the model frame.
    pub fn with_clip(mut self, clip: EntityId) -> Self {
        self.clip = Some(clip);
        self
    }
}

/// A recoloring of every copy whose path starts with `prefix`. Reaches inner slots from an outer
/// level (for example absorbers only in assembly #3).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct PathOverride {
    /// The path prefix.
    pub prefix: InstancePath,
    /// The recoloring.
    pub recolor: ColorMap,
}

impl PathOverride {
    /// Recolor every copy under `prefix`.
    pub fn new(prefix: InstancePath, recolor: ColorMap) -> Self {
        Self { prefix, recolor }
    }
}

/// The instancing tables.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct Instancing {
    /// Prototypes by `Prototype` id.
    pub prototypes: BTreeMap<EntityId, Prototype>,
    /// Lattices by `Lattice` id.
    pub lattices: BTreeMap<EntityId, Lattice>,
    /// Instances by `Instance` id (all are top-level roots).
    pub instances: BTreeMap<EntityId, InstanceRow>,
    /// Path overrides, in declaration order.
    pub overrides: Vec<PathOverride>,
    /// Clusters by `Cluster` id (absent in older data).
    #[cfg_attr(feature = "serde", serde(default))]
    pub clusters: BTreeMap<EntityId, Cluster>,
}

const MAX_DEPTH: usize = 64;

/// Largest number of volume copies [`flatten`] will produce.
pub const MAX_FLAT_ENTRIES: usize = 1 << 24;

impl Instancing {
    /// Empty tables.
    pub fn new() -> Self {
        Self::default()
    }

    /// The nested fills of a lattice or cluster (`None` for other ids or unknown rows).
    fn children_of(&self, id: EntityId) -> Option<Vec<Fill>> {
        if let Some(lat) = self.lattices.get(&id) {
            return Some(lat.slots.values().map(|s| s.fill).chain([lat.outer]).collect());
        }
        self.clusters.get(&id).map(|c| c.members.iter().map(|m| m.fill).collect())
    }

    /// A lattice or cluster that is (transitively) its own descendant, if any.
    pub fn find_cycle(&self) -> Option<EntityId> {
        fn visit(ins: &Instancing, l: EntityId, stack: &mut Vec<EntityId>, done: &mut BTreeSet<EntityId>) -> Option<EntityId> {
            if stack.contains(&l) {
                return Some(l);
            }
            if done.contains(&l) {
                return None;
            }
            let fills = ins.children_of(l)?;
            stack.push(l);
            for f in fills {
                if let Fill::Lattice(c) | Fill::Cluster(c) = f
                    && let Some(x) = visit(ins, c, stack, done)
                {
                    return Some(x);
                }
            }
            stack.pop();
            done.insert(l);
            None
        }
        let mut done = BTreeSet::new();
        self.lattices.keys().chain(self.clusters.keys()).find_map(|l| visit(self, *l, &mut Vec::new(), &mut done))
    }
}

impl Model {
    pub(crate) fn check_fill(&self, f: Fill) -> Result<()> {
        match f {
            Fill::Void => Ok(()),
            Fill::Prototype(p) => {
                expect_kind(p, EntityKind::Prototype)?;
                self.instancing.prototypes.contains_key(&p).then_some(()).ok_or(ModelError::Unknown(p))
            }
            Fill::Lattice(l) => {
                expect_kind(l, EntityKind::Lattice)?;
                self.instancing.lattices.contains_key(&l).then_some(()).ok_or(ModelError::Unknown(l))
            }
            Fill::Cluster(c) => {
                expect_kind(c, EntityKind::Cluster)?;
                self.instancing.clusters.contains_key(&c).then_some(()).ok_or(ModelError::Unknown(c))
            }
        }
    }

    pub(crate) fn check_cluster(&self, c: &Cluster) -> Result<()> {
        if c.members.len() > MAX_FLAT_ENTRIES {
            return Err(ModelError::Invalid(format!("a cluster has at most {MAX_FLAT_ENTRIES} members")));
        }
        for m in &c.members {
            self.check_fill(m.fill)?;
            self.check_color_map(&m.recolor)?;
            Self::check_clip(m.clip)?;
            if !(m.at.m.iter().flatten().all(|x| x.is_finite()) && [m.at.t.x, m.at.t.y, m.at.t.z].iter().all(|x| x.is_finite())) {
                return Err(ModelError::Invalid("cluster member placement must be finite".into()));
            }
        }
        Ok(())
    }

    /// Add a cluster. Member fills must exist, recolor maps are domain-checked, clips must
    /// name `TopoVolume`s and placements be finite. Acyclic by construction (fills reference
    /// existing ids).
    pub fn add_cluster(&mut self, c: Cluster) -> Result<EntityId> {
        self.check_cluster(&c)?;
        let id = self.ids.alloc(EntityKind::Cluster)?;
        self.instancing.clusters.insert(id, c);
        Ok(id)
    }

    /// Replace the members of cluster `id` (same checks as [`Self::add_cluster`]). Rejects a
    /// fill that would make the cluster contain itself ([`ModelError::Cycle`]).
    pub fn set_cluster(&mut self, id: EntityId, c: Cluster) -> Result<()> {
        expect_kind(id, EntityKind::Cluster)?;
        if !self.instancing.clusters.contains_key(&id) {
            return Err(ModelError::Unknown(id));
        }
        self.check_cluster(&c)?;
        let old = self.instancing.clusters.insert(id, c);
        if self.instancing.find_cycle().is_some() {
            if let Some(o) = old {
                self.instancing.clusters.insert(id, o);
            }
            return Err(ModelError::Cycle(id));
        }
        Ok(())
    }

    /// A clip must name a `TopoVolume` (liveness is [`crate::validate()`]'s).
    pub(crate) fn check_clip(clip: Option<EntityId>) -> Result<()> {
        clip.map_or(Ok(()), |c| expect_kind(c, EntityKind::TopoVolume))
    }

    /// Check key kinds, value domains and that colors exist.
    pub(crate) fn check_color_map(&self, m: &ColorMap) -> Result<()> {
        let check = |map: &BTreeMap<EntityId, EntityId>, ent: EntityKind, dom: crate::color::Domain| -> Result<()> {
            for (k, v) in map {
                match k.kind() {
                    x if x == ent => {}
                    EntityKind::Color => match self.colors.get(k) {
                        None => return Err(ModelError::Unknown(*k)),
                        Some(c) if c.domain != dom => return Err(ModelError::ColorDomain(*k)),
                        Some(_) => {}
                    },
                    _ => return Err(ModelError::WrongKind { id: *k, expected: ent, found: k.kind() }),
                }
                match self.colors.get(v) {
                    None => return Err(ModelError::Unknown(*v)),
                    Some(c) if c.domain != dom => return Err(ModelError::ColorDomain(*v)),
                    Some(_) => {}
                }
            }
            Ok(())
        };
        check(&m.volume, EntityKind::TopoVolume, crate::color::Domain::Volume)?;
        check(&m.face, EntityKind::TopoFace, crate::color::Domain::Face)
    }

    /// The instancing tables (read only).
    pub fn instancing(&self) -> &Instancing {
        &self.instancing
    }

    /// Add a prototype made of `vols`.
    pub fn add_prototype(&mut self, vols: &[EntityId]) -> Result<EntityId> {
        if vols.is_empty() {
            return Err(ModelError::Invalid("a prototype needs at least one volume".into()));
        }
        for v in vols {
            expect_kind(*v, EntityKind::TopoVolume)?;
        }
        let id = self.ids.alloc(EntityKind::Prototype)?;
        self.instancing.prototypes.insert(id, Prototype { volumes: vols.iter().copied().collect() });
        Ok(id)
    }

    /// Add a lattice. Fills must exist; slots must be inside the layout; recolor maps are
    /// domain-checked. The result is acyclic by construction (fills reference existing ids).
    pub fn add_lattice(&mut self, l: Lattice) -> Result<EntityId> {
        l.layout.check()?;
        self.check_fill(l.outer)?;
        for (ix, s) in &l.slots {
            if !l.layout.contains(*ix) {
                return Err(ModelError::Invalid(format!("slot {ix:?} is outside the lattice layout")));
            }
            self.check_fill(s.fill)?;
            self.check_color_map(&s.recolor)?;
            Self::check_clip(s.clip)?;
        }
        let id = self.ids.alloc(EntityKind::Lattice)?;
        self.instancing.lattices.insert(id, l);
        Ok(id)
    }

    /// Set (replace) one slot of a lattice. Rejects a fill that would make the lattice contain
    /// itself.
    pub fn set_lattice_slot(&mut self, lattice: EntityId, ix: SlotIx, slot: Slot) -> Result<()> {
        let lat = self.instancing.lattices.get(&lattice).ok_or(ModelError::Unknown(lattice))?;
        if !lat.layout.contains(ix) {
            return Err(ModelError::Invalid(format!("slot {ix:?} is outside the lattice layout")));
        }
        self.check_fill(slot.fill)?;
        self.check_color_map(&slot.recolor)?;
        Self::check_clip(slot.clip)?;
        let old = self.instancing.lattices.get_mut(&lattice).and_then(|l| l.slots.insert(ix, slot));
        if self.instancing.find_cycle().is_some() {
            if let Some(l) = self.instancing.lattices.get_mut(&lattice) {
                match old {
                    Some(o) => l.slots.insert(ix, o),
                    None => l.slots.remove(&ix),
                };
            }
            return Err(ModelError::Cycle(lattice));
        }
        Ok(())
    }

    /// Add a top-level instance.
    pub fn add_instance(&mut self, i: InstanceRow) -> Result<EntityId> {
        self.check_fill(i.of)?;
        self.check_color_map(&i.recolor)?;
        Self::check_clip(i.clip)?;
        let id = self.ids.alloc(EntityKind::Instance)?;
        self.instancing.instances.insert(id, i);
        Ok(id)
    }

    /// Add a recoloring for every copy under `o.prefix`.
    pub fn add_path_override(&mut self, o: PathOverride) -> Result<()> {
        self.check_color_map(&o.recolor)?;
        if o.prefix.is_empty() {
            return Err(ModelError::Invalid("an override prefix needs at least one step".into()));
        }
        self.instancing.overrides.push(o);
        Ok(())
    }

    /// The recolor maps that apply along `path`, outermost first (instance, then slots).
    fn levels_of(&self, path: &[PathStep]) -> Result<Vec<&ColorMap>> {
        let mut out = Vec::with_capacity(path.len());
        for s in path {
            match s {
                PathStep::Instance(i) => out.push(&self.instancing.instances.get(i).ok_or(ModelError::Unknown(*i))?.recolor),
                PathStep::Slot { lattice, ix } => {
                    let l = self.instancing.lattices.get(lattice).ok_or(ModelError::Unknown(*lattice))?;
                    if let Some(s) = l.slots.get(ix) {
                        out.push(&s.recolor);
                    }
                }
                PathStep::Member { cluster, ix } => {
                    let c = self.instancing.clusters.get(cluster).ok_or(ModelError::Unknown(*cluster))?;
                    let m = c.members.get(*ix as usize).ok_or_else(|| ModelError::Invalid(format!("cluster {cluster} has no member {ix}")))?;
                    out.push(&m.recolor);
                }
            }
        }
        Ok(out)
    }

    /// Overrides whose prefix starts `path`, ordered deepest prefix first (so outer ones win).
    fn matching_overrides(&self, path: &[PathStep]) -> Vec<&ColorMap> {
        let mut v: Vec<&PathOverride> = self.instancing.overrides.iter().filter(|o| path.starts_with(&o.prefix)).collect();
        v.sort_by_key(|o| core::cmp::Reverse(o.prefix.len()));
        v.into_iter().map(|o| &o.recolor).collect()
    }

    /// The color of face `f` in the copy named `path`: its face color after the recolor chain
    /// (inner first, outer last, path overrides at the end). `None` if the face is uncolored.
    pub fn resolve_face_color(&self, path: &[PathStep], f: EntityId) -> Result<Option<EntityId>> {
        let Some(mut c) = self.face_color.get(&f).copied() else { return Ok(None) };
        for m in self.levels_of(path)?.into_iter().rev() {
            c = m.apply_face(f, c);
        }
        for m in self.matching_overrides(path) {
            c = m.apply_face(f, c);
        }
        Ok(Some(c))
    }
}

/// One flattened volume of one copy.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct FlatEntry {
    /// Index into [`FlatModel::paths`].
    pub path: u32,
    /// Index into [`FlatModel::placements`] (shared by all entries of a path).
    pub placement: u32,
    /// The prototype volume (stored once).
    pub volume: EntityId,
    /// Resolved volume color.
    pub color: EntityId,
    /// Index into [`FlatModel::clips`] of the innermost clip bounding this copy; follow
    /// [`FlatClip::outer`] for the enclosing ones (the copy is bounded by all of them). `None`:
    /// not clipped.
    #[cfg_attr(feature = "serde", serde(default))]
    pub clip: Option<u32>,
}

impl FlatEntry {
    /// An entry of path `path` (index), placement index, prototype volume, color and innermost clip.
    pub fn new(path: u32, placement: u32, volume: EntityId, color: EntityId, clip: Option<u32>) -> Self {
        Self { path, placement, volume, color, clip }
    }
}

/// One clip occurrence of a flattened model: an [`InstanceRow::clip`] or [`Slot::clip`] volume
/// at the placement it has in the model frame.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct FlatClip {
    /// The clip volume (stored once, like prototype volumes).
    pub volume: EntityId,
    /// Where the clip volume sits in the model frame (identity for an instance clip, the
    /// lattice placement for a slot clip).
    pub placement: Placement,
    /// The enclosing clip (an outer level's), if any: an index into [`FlatModel::clips`].
    pub outer: Option<u32>,
}

impl FlatClip {
    /// A clip occurrence.
    pub fn new(volume: EntityId, placement: Placement, outer: Option<u32>) -> Self {
        Self { volume, placement, outer }
    }
}

/// Result of [`flatten`]. Entries are in canonical order: instance id, slot `(k, j, i)` or
/// cluster member index (nested), volume id. Paths are strictly increasing, so lookups by path are binary searches.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct FlatModel {
    /// One placement per path.
    pub placements: Vec<Placement>,
    /// One path per placed prototype copy, ascending.
    pub paths: Vec<InstancePath>,
    /// The volumes, ascending by `(path, volume)`.
    pub entries: Vec<FlatEntry>,
    /// Clip occurrences, in walk order (an outer clip before the clips nested in it).
    #[cfg_attr(feature = "serde", serde(default))]
    pub clips: Vec<FlatClip>,
}

impl FlatModel {
    /// A flattened model from its columns.
    pub fn new(placements: Vec<Placement>, paths: Vec<InstancePath>, entries: Vec<FlatEntry>, clips: Vec<FlatClip>) -> Self {
        Self { placements, paths, entries, clips }
    }

    /// The entries of the copy named `p`.
    pub fn find(&self, p: &[PathStep]) -> Option<std::ops::Range<usize>> {
        let ix = self.paths.binary_search_by(|q| q.as_slice().cmp(p)).ok()? as u32;
        let lo = self.entries.partition_point(|e| e.path < ix);
        let hi = self.entries.partition_point(|e| e.path <= ix);
        Some(lo..hi)
    }

    /// The placement of an entry.
    pub fn placement_of(&self, e: &FlatEntry) -> &Placement {
        &self.placements[e.placement as usize]
    }

    /// FNV-1a fingerprint of the whole result (paths, placements, volumes, colors) over the raw
    /// `f64` bits of the placements. Bit-exact on **one platform only**; use
    /// [`FlatModel::fingerprint_quantized`] for a cross-platform hash.
    pub fn fingerprint(&self) -> u64 {
        self.fingerprint_with(|v| v.to_bits())
    }

    /// Cross-platform fingerprint: placement numbers are quantized with the step
    /// `linear * 2^-10` (the Q2 rule, `linear` = the model's linear tolerance) before hashing.
    pub fn fingerprint_quantized(&self, linear: f64) -> u64 {
        let q = linear.abs().max(1e-300) * (1.0 / 1024.0);
        self.fingerprint_with(|v| (v / q).round() as i64 as u64)
    }

    fn fingerprint_with(&self, num: impl Fn(f64) -> u64) -> u64 {
        let mut h = 0xcbf2_9ce4_8422_2325u64;
        let mut eat = |x: u64| {
            for b in x.to_le_bytes() {
                h ^= u64::from(b);
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        };
        for p in &self.paths {
            eat(p.len() as u64);
            for s in p {
                match s {
                    PathStep::Instance(i) => {
                        eat(1);
                        eat(i.to_raw());
                    }
                    PathStep::Slot { lattice, ix } => {
                        eat(2);
                        eat(lattice.to_raw());
                        eat(ix.i as u32 as u64);
                        eat(ix.j as u32 as u64);
                        eat(ix.k as u32 as u64);
                    }
                    PathStep::Member { cluster, ix } => {
                        eat(3);
                        eat(cluster.to_raw());
                        eat(u64::from(*ix));
                    }
                }
            }
        }
        for pl in &self.placements {
            pl.transform.m.iter().flatten().chain([&pl.transform.t.x, &pl.transform.t.y, &pl.transform.t.z]).for_each(|v| eat(num(*v)));
            eat(u64::from(pl.orientation_reversed));
        }
        for e in &self.entries {
            eat(u64::from(e.path));
            eat(u64::from(e.placement));
            eat(e.volume.to_raw());
            eat(e.color.to_raw());
        }
        // Clips are hashed only when present, so unclipped models keep their fingerprints.
        if !self.clips.is_empty() {
            let opt = |o: Option<u32>| o.map_or(0, |x| u64::from(x) + 1);
            for e in &self.entries {
                eat(opt(e.clip));
            }
            for c in &self.clips {
                eat(c.volume.to_raw());
                c.placement.transform.m.iter().flatten().chain([&c.placement.transform.t.x, &c.placement.transform.t.y, &c.placement.transform.t.z]).for_each(|v| eat(num(*v)));
                eat(u64::from(c.placement.orientation_reversed));
                eat(opt(c.outer));
            }
        }
        h
    }
}

struct Walk<'a> {
    m: &'a Model,
    out: FlatModel,
    /// Slots and members visited (bounded by [`MAX_FLAT_ENTRIES`], so void-only nesting cannot
    /// run away either).
    visits: usize,
}

impl<'a> Walk<'a> {
    fn push_clip(&mut self, volume: EntityId, at: Transform, outer: Option<u32>) -> Option<u32> {
        let ix = self.out.clips.len() as u32;
        self.out.clips.push(FlatClip { volume, placement: Placement::of(at), outer });
        Some(ix)
    }

    fn visit(&mut self) -> Result<()> {
        self.visits += 1;
        if self.visits > MAX_FLAT_ENTRIES {
            return Err(ModelError::Invalid(format!("flattening would visit more than {MAX_FLAT_ENTRIES} slots and members")));
        }
        Ok(())
    }

    fn go(&mut self, fill: Fill, at: Transform, path: &mut InstancePath, levels: &mut Vec<&'a ColorMap>, clip: Option<u32>, depth: usize) -> Result<()> {
        if depth > MAX_DEPTH {
            return Err(ModelError::Invalid("instancing nesting too deep".into()));
        }
        match fill {
            Fill::Void => {}
            Fill::Prototype(p) => {
                let proto = self.m.instancing.prototypes.get(&p).ok_or(ModelError::Unknown(p))?;
                if self.out.entries.len().saturating_add(proto.volumes.len()) > MAX_FLAT_ENTRIES {
                    return Err(ModelError::Invalid(format!("flattening would exceed {MAX_FLAT_ENTRIES} entries")));
                }
                let pix = self.out.paths.len() as u32;
                let plix = self.out.placements.len() as u32;
                let overrides = self.m.matching_overrides(path);
                for v in &proto.volumes {
                    let mut c = self.m.volume_color.get(v).copied().ok_or_else(|| ModelError::Invalid(format!("volume {v} has no color")))?;
                    for lv in levels.iter().rev() {
                        c = lv.apply_volume(*v, c);
                    }
                    for o in &overrides {
                        c = o.apply_volume(*v, c);
                    }
                    self.out.entries.push(FlatEntry { path: pix, placement: plix, volume: *v, color: c, clip });
                }
                self.out.paths.push(path.clone());
                self.out.placements.push(Placement::of(at));
            }
            Fill::Lattice(l) => {
                let lat = self.m.instancing.lattices.get(&l).ok_or(ModelError::Unknown(l))?;
                let base = at.compose(&lat.origin);
                for ix in lat.layout.indices() {
                    let slot = lat.slots.get(&ix);
                    let f = slot.map_or(lat.outer, |s| s.fill);
                    if f == Fill::Void {
                        continue;
                    }
                    self.visit()?;
                    let local = Transform::translation(lat.layout.offset(ix));
                    path.push(PathStep::Slot { lattice: l, ix });
                    let pushed = slot.is_some_and(|s| !s.recolor.is_empty());
                    if let Some(s) = slot.filter(|_| pushed) {
                        levels.push(&s.recolor);
                    }
                    let inner = match slot.and_then(|s| s.clip) {
                        Some(cv) => self.push_clip(cv, base, clip),
                        None => clip,
                    };
                    self.go(f, base.compose(&local), path, levels, inner, depth + 1)?;
                    if pushed {
                        levels.pop();
                    }
                    path.pop();
                }
            }
            Fill::Cluster(c) => {
                let cl = self.m.instancing.clusters.get(&c).ok_or(ModelError::Unknown(c))?;
                for (ix, mem) in cl.members.iter().enumerate() {
                    if mem.fill == Fill::Void {
                        continue;
                    }
                    self.visit()?;
                    path.push(PathStep::Member { cluster: c, ix: ix as u32 });
                    let pushed = !mem.recolor.is_empty();
                    if pushed {
                        levels.push(&mem.recolor);
                    }
                    let inner = match mem.clip {
                        Some(cv) => self.push_clip(cv, at, clip),
                        None => clip,
                    };
                    self.go(mem.fill, at.compose(&mem.at), path, levels, inner, depth + 1)?;
                    if pushed {
                        levels.pop();
                    }
                    path.pop();
                }
            }
        }
        Ok(())
    }
}

/// Flatten every instance into placed copies with resolved colors, in canonical order. Copies are
/// named by [`InstancePath`]; no ids are minted, so nothing is ever reused. Each entry carries
/// its innermost clip ([`FlatEntry::clip`], chained through [`FlatClip::outer`]). Fails on
/// reference cycles, unknown ids or uncolored prototype volumes.
pub fn flatten(m: &Model) -> Result<FlatModel> {
    if let Some(l) = m.instancing.find_cycle() {
        return Err(ModelError::Cycle(l));
    }
    let mut w = Walk { m, out: FlatModel::default(), visits: 0 };
    for (id, inst) in &m.instancing.instances {
        let mut path: InstancePath = SmallVec::from_slice(&[PathStep::Instance(*id)]);
        let mut levels: Vec<&ColorMap> = vec![&inst.recolor];
        let clip = inst.clip.and_then(|cv| w.push_clip(cv, Transform::IDENTITY, None));
        w.go(inst.of, inst.at, &mut path, &mut levels, clip, 0)?;
    }
    Ok(w.out)
}
