//! Instancing: prototypes, instances, lattices (cartesian + hex, sparse, nestable), recoloring and
//! a deterministic flatten whose copies are named by instance paths (no ids are minted for
//! copies, so ids are never reused).
//!
//! Instancing is a group action on a fundamental piece, like symmetry: [`Placement`] and
//! [`unfold`](mod@crate::unfold) share the same copy description.
//!
//! **Recolor order** (inner first, outer last): prototype color, then the inner slot, outer
//! slot(s), the instance, and finally [`PathOverride`]s (deeper prefixes first). Within one
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
}

/// Stable name of a flattened copy: no minted ids. Paths compare in emission order.
pub type InstancePath = SmallVec<[PathStep; 4]>;

/// Hex row orientation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum HexOrient {
    /// Rows along x: `(x, y) = pitch * (q + r/2, sqrt(3)/2 r)`.
    XRows,
    /// Rows along y: `(x, y) = pitch * (sqrt(3)/2 q, r + q/2)`.
    YRows,
}

/// Slot geometry of a lattice. Slot `(0, 0, 0)` is at the lattice origin.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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

/// What fills an instance or a lattice slot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Fill {
    /// A [`Prototype`].
    Prototype(EntityId),
    /// A nested [`Lattice`].
    Lattice(EntityId),
    /// Nothing (a hole).
    Void,
}

/// A recoloring, typed by domain: `volume` maps volume colors (or single `TopoVolume`s) to volume
/// colors; `face` maps face colors (or single `TopoFace`s) to face colors. A per-entity key beats
/// a per-color key.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ColorMap {
    /// Volume recoloring: key `TopoVolume` or volume `Color`; value volume `Color`.
    pub volume: BTreeMap<EntityId, EntityId>,
    /// Face recoloring: key `TopoFace` or face `Color`; value face `Color`.
    pub face: BTreeMap<EntityId, EntityId>,
}

impl ColorMap {
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
pub struct Slot {
    /// The fill.
    pub fill: Fill,
    /// Recoloring of everything below this slot.
    pub recolor: ColorMap,
}

/// A lattice: a layout plus a **sparse** slot table. In-range slots without an entry take `outer`.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Lattice {
    /// Slot geometry.
    pub layout: Layout,
    /// Placement of the lattice origin in its parent frame.
    pub origin: Transform,
    /// Fill of slots without an entry.
    pub outer: Fill,
    /// Explicit slots.
    #[cfg_attr(feature = "serde", serde(with = "crate::serde_pairs"))]
    pub slots: BTreeMap<SlotIx, Slot>,
}

/// A reusable definition: volumes of the topology (stored once, placed many times).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Prototype {
    /// The volumes.
    pub volumes: BTreeSet<EntityId>,
}

/// A placement of a [`Fill`] at the top level.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct InstanceRow {
    /// What is placed.
    pub of: Fill,
    /// Placement (may be improper: a mirrored instance).
    pub at: Transform,
    /// Recoloring of everything below.
    pub recolor: ColorMap,
}

/// A recoloring of every copy whose path starts with `prefix`. Reaches inner slots from an outer
/// level (for example absorbers only in assembly #3).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PathOverride {
    /// The path prefix.
    pub prefix: InstancePath,
    /// The recoloring.
    pub recolor: ColorMap,
}

/// The instancing tables.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Instancing {
    /// Prototypes by `Prototype` id.
    pub prototypes: BTreeMap<EntityId, Prototype>,
    /// Lattices by `Lattice` id.
    pub lattices: BTreeMap<EntityId, Lattice>,
    /// Instances by `Instance` id (all are top-level roots).
    pub instances: BTreeMap<EntityId, InstanceRow>,
    /// Path overrides, in declaration order.
    pub overrides: Vec<PathOverride>,
}

const MAX_DEPTH: usize = 64;

/// Largest number of volume copies [`flatten`] will produce.
pub const MAX_FLAT_ENTRIES: usize = 1 << 24;

impl Instancing {
    /// A lattice that is (transitively) its own descendant, if any.
    pub fn find_cycle(&self) -> Option<EntityId> {
        fn visit(ins: &Instancing, l: EntityId, stack: &mut Vec<EntityId>, done: &mut BTreeSet<EntityId>) -> Option<EntityId> {
            if stack.contains(&l) {
                return Some(l);
            }
            if done.contains(&l) {
                return None;
            }
            let lat = ins.lattices.get(&l)?;
            stack.push(l);
            for f in lat.slots.values().map(|s| s.fill).chain([lat.outer]) {
                if let Fill::Lattice(c) = f
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
        self.lattices.keys().find_map(|l| visit(self, *l, &mut Vec::new(), &mut done))
    }
}

impl Model {
    fn check_fill(&self, f: Fill) -> Result<()> {
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
        }
    }

    /// Check key kinds, value domains and that colors exist.
    fn check_color_map(&self, m: &ColorMap) -> Result<()> {
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
pub struct FlatEntry {
    /// Index into [`FlatModel::paths`].
    pub path: u32,
    /// Index into [`FlatModel::placements`] (shared by all entries of a path).
    pub placement: u32,
    /// The prototype volume (stored once).
    pub volume: EntityId,
    /// Resolved volume color.
    pub color: EntityId,
}

/// Result of [`flatten`]. Entries are in canonical order: instance id, slot `(k, j, i)` (nested),
/// volume id. Paths are strictly increasing, so lookups by path are binary searches.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FlatModel {
    /// One placement per path.
    pub placements: Vec<Placement>,
    /// One path per placed prototype copy, ascending.
    pub paths: Vec<InstancePath>,
    /// The volumes, ascending by `(path, volume)`.
    pub entries: Vec<FlatEntry>,
}

impl FlatModel {
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
        h
    }
}

struct Walk<'a> {
    m: &'a Model,
    out: FlatModel,
}

impl<'a> Walk<'a> {
    fn go(&mut self, fill: Fill, at: Transform, path: &mut InstancePath, levels: &mut Vec<&'a ColorMap>, depth: usize) -> Result<()> {
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
                    self.out.entries.push(FlatEntry { path: pix, placement: plix, volume: *v, color: c });
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
                    let local = Transform::translation(lat.layout.offset(ix));
                    path.push(PathStep::Slot { lattice: l, ix });
                    let pushed = slot.is_some_and(|s| !s.recolor.is_empty());
                    if let Some(s) = slot.filter(|_| pushed) {
                        levels.push(&s.recolor);
                    }
                    self.go(f, base.compose(&local), path, levels, depth + 1)?;
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
/// named by [`InstancePath`]; no ids are minted, so nothing is ever reused. Fails on reference
/// cycles, unknown ids or uncolored prototype volumes.
pub fn flatten(m: &Model) -> Result<FlatModel> {
    if let Some(l) = m.instancing.find_cycle() {
        return Err(ModelError::Cycle(l));
    }
    let mut w = Walk { m, out: FlatModel::default() };
    for (id, inst) in &m.instancing.instances {
        let mut path: InstancePath = SmallVec::from_slice(&[PathStep::Instance(*id)]);
        let mut levels: Vec<&ColorMap> = vec![&inst.recolor];
        w.go(inst.of, inst.at, &mut path, &mut levels, 0)?;
    }
    Ok(w.out)
}
