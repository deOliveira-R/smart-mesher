//! Instancing tables: prototypes, instances, lattices (cartesian + hex, nestable), per-instance
//! color overrides, sparse slots, and a deterministic flatten with instance paths.
//!
//! Instancing is a group action on a fundamental piece, like symmetry: [`Model::flatten`]
//! produces [`Placed`] copies exactly as [`crate::symmetry`]'s unfold does, so downstream
//! consumers (classification, tracking, export) handle one copy-list shape.
//!
//! All three tables are keyed by `EntityKind::Instance` ids. Everything iterates in table or
//! `Vec` order, so flatten output is deterministic by construction.

use crate::action::{PathStep, Placed, SlotIndex, Path, translation};
use crate::error::{ModelError, Result};
use crate::model::Model;
use sm_core::{EntityId, EntityKind, Transform, Vector3};
use std::collections::BTreeMap;
use std::ops::Range;

/// A per-instance color change. Overrides of outer levels apply after (and so win over) inner ones.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Override {
    /// Every volume with color `from` gets color `to`.
    ReplaceColor {
        /// Color to replace.
        from: EntityId,
        /// Replacement.
        to: EntityId,
    },
    /// One prototype volume gets color `to`.
    SetVolume {
        /// The `TopoVolume` to recolor.
        volume: EntityId,
        /// New color.
        to: EntityId,
    },
    /// Everything below gets color `to` (homogenization of a whole instance/slot).
    SetAll {
        /// New color.
        to: EntityId,
    },
}

/// What fills an instance or a lattice slot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Fill {
    /// A [`Prototype`].
    Prototype(EntityId),
    /// A nested [`Lattice`].
    Lattice(EntityId),
    /// No geometry: a homogenized region of one volume color.
    Homogeneous(EntityId),
}

/// Member of a prototype.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Member {
    /// A volume of the topology (stored once, placed many times).
    Volume(EntityId),
    /// A nested instance.
    Instance(EntityId),
}

/// A reusable piece: volumes of the topology plus nested instances, in its own frame.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Prototype {
    /// Unique name.
    pub name: String,
    /// Members in emit order.
    pub members: Vec<Member>,
}

/// A placement of a [`Fill`].
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct InstanceRow {
    /// What is placed.
    pub fill: Fill,
    /// Placement (may be improper: a mirrored instance).
    pub transform: Transform,
    /// Color overrides, applied in order.
    pub overrides: Vec<Override>,
}

/// Hex row orientation (which direction neighbouring slots of a row run).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum HexOrientation {
    /// Rows along x: `(x, y) = pitch * (q + r/2, sqrt(3)/2 * r)`.
    XRows,
    /// Rows along y: `(x, y) = pitch * (sqrt(3)/2 * q, r + q/2)`.
    YRows,
}

/// Slot geometry of a lattice. Slot `[0,0,0]` is centred at `origin`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Layout {
    /// Cartesian `dims[0] x dims[1] x dims[2]` slots, indices `[i, j, k]` from 0.
    Cartesian {
        /// Slot center spacing per axis.
        pitch: [f64; 3],
        /// Slot counts per axis (at least 1).
        dims: [u32; 3],
    },
    /// Hex (axial `[q, r, k]`) with `rings` rings (1 = a single slot), `layers` axial layers.
    Hex {
        /// Center-to-center distance in the plane.
        pitch: f64,
        /// Axial spacing between layers.
        axial_pitch: f64,
        /// Number of rings, at least 1; `max(|q|,|r|,|q+r|) < rings` are in range.
        rings: u32,
        /// Number of axial layers, at least 1; `k` in `0..layers`.
        layers: u32,
        /// Row orientation.
        orientation: HexOrientation,
    },
}

impl Layout {
    /// All in-range slot indices, ascending (the canonical slot order).
    pub fn indices(&self) -> Vec<SlotIndex> {
        let mut v = Vec::new();
        match *self {
            Layout::Cartesian { dims, .. } => {
                for i in 0..dims[0] as i32 {
                    for j in 0..dims[1] as i32 {
                        for k in 0..dims[2] as i32 {
                            v.push([i, j, k]);
                        }
                    }
                }
            }
            Layout::Hex { rings, layers, .. } => {
                let n = rings as i32 - 1;
                for q in -n..=n {
                    for r in -n..=n {
                        if (q + r).abs() <= n {
                            for k in 0..layers as i32 {
                                v.push([q, r, k]);
                            }
                        }
                    }
                }
            }
        }
        v
    }

    /// True if `idx` is inside the layout.
    pub fn contains(&self, idx: SlotIndex) -> bool {
        match *self {
            Layout::Cartesian { dims, .. } => (0..3).all(|a| idx[a] >= 0 && (idx[a] as u32) < dims[a]),
            Layout::Hex { rings, layers, .. } => {
                let n = rings as i32 - 1;
                idx[0].abs() <= n && idx[1].abs() <= n && (idx[0] + idx[1]).abs() <= n && idx[2] >= 0 && (idx[2] as u32) < layers
            }
        }
    }

    /// Slot center offset from the lattice origin.
    pub fn offset(&self, idx: SlotIndex) -> Vector3 {
        match *self {
            Layout::Cartesian { pitch, .. } => Vector3::new(f64::from(idx[0]) * pitch[0], f64::from(idx[1]) * pitch[1], f64::from(idx[2]) * pitch[2]),
            Layout::Hex { pitch, axial_pitch, orientation, .. } => {
                let (q, r) = (f64::from(idx[0]), f64::from(idx[1]));
                let h = 3f64.sqrt() / 2.0;
                let (x, y) = match orientation {
                    HexOrientation::XRows => (q + r / 2.0, h * r),
                    HexOrientation::YRows => (h * q, r + q / 2.0),
                };
                Vector3::new(pitch * x, pitch * y, f64::from(idx[2]) * axial_pitch)
            }
        }
    }

    /// The placement list of the layout in canonical slot order: `(index, translation)`.
    pub fn placements(&self) -> Vec<(SlotIndex, Transform)> {
        self.indices().into_iter().map(|i| (i, translation(self.offset(i)))).collect()
    }

    fn check(&self) -> Result<()> {
        let ok = match *self {
            Layout::Cartesian { dims, .. } => dims.iter().all(|d| *d >= 1),
            Layout::Hex { rings, layers, .. } => rings >= 1 && layers >= 1,
        };
        if ok { Ok(()) } else { Err(ModelError::Invalid("lattice layout needs at least one slot per axis".into())) }
    }
}

/// An explicitly filled slot.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Slot {
    /// What fills the slot.
    pub fill: Fill,
    /// Extra placement within the slot (e.g. a rotated pin), after the slot translation.
    pub transform: Option<Transform>,
    /// Per-slot color overrides.
    pub overrides: Vec<Override>,
}

/// A lattice: a layout plus a **sparse** slot table. In-range slots without an entry take `outer`
/// (or stay empty if `outer` is `None`). The index map is separate from the fill records,
/// so many slots can share one fill.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Lattice {
    /// Unique name.
    pub name: String,
    /// Slot geometry.
    pub layout: Layout,
    /// Slot index -> fill.
    pub slots: BTreeMap<SlotIndex, Slot>,
    /// Fill for in-range slots with no entry.
    pub outer: Option<Fill>,
}

/// The instancing tables (all keyed by `EntityKind::Instance` ids, one id space).
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Instancing {
    /// Prototype table.
    pub prototypes: BTreeMap<EntityId, Prototype>,
    /// Instance table.
    pub instances: BTreeMap<EntityId, InstanceRow>,
    /// Lattice table.
    pub lattices: BTreeMap<EntityId, Lattice>,
    /// Top-level instances that make up the assembled model, in order.
    pub roots: Vec<EntityId>,
}

/// What a flattened entry is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum FlatContent {
    /// A placed prototype volume with its resolved color (after overrides).
    Volume {
        /// The topology volume (stored once).
        volume: EntityId,
        /// Resolved color; `None` if the volume is uncolored.
        color: Option<EntityId>,
    },
    /// A homogenized region.
    Homogeneous {
        /// Resolved color.
        color: EntityId,
    },
}

/// One flattened copy.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FlatEntry {
    /// Instance path from the root.
    pub placed: Placed,
    /// Content.
    pub content: FlatContent,
}

/// Result of [`Model::flatten`].
#[derive(Clone, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FlatModel {
    /// Entries in canonical (depth-first, table) order.
    pub entries: Vec<FlatEntry>,
    /// Entry-index ranges per distinct instance path (contiguous runs), in order.
    pub ranges: Vec<(Path, Range<usize>)>,
}

impl FlatModel {
    /// FNV-1a fingerprint of the whole result (paths, transforms, content), for determinism tests
    /// and cache keys. Bit-exact on one platform.
    pub fn fingerprint(&self) -> u64 {
        let mut h = 0xcbf2_9ce4_8422_2325u64;
        let mut eat = |x: u64| {
            for b in x.to_le_bytes() {
                h ^= u64::from(b);
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        };
        for e in &self.entries {
            for s in &e.placed.path {
                match s {
                    PathStep::Element(n) => eat(1 << 32 | u64::from(*n)),
                    PathStep::Instance(i) => {
                        eat(2);
                        eat(i.to_raw());
                    }
                    PathStep::Slot { lattice, index } => {
                        eat(3);
                        eat(lattice.to_raw());
                        index.iter().for_each(|c| eat(*c as u32 as u64));
                    }
                }
            }
            e.placed.transform.m.iter().flatten().chain([&e.placed.transform.t.x, &e.placed.transform.t.y, &e.placed.transform.t.z]).for_each(|v| eat(v.to_bits()));
            match e.content {
                FlatContent::Volume { volume, color } => {
                    eat(volume.to_raw());
                    eat(color.map_or(0, EntityId::to_raw));
                }
                FlatContent::Homogeneous { color } => eat(color.to_raw() ^ 0xff),
            }
        }
        h
    }
}

const MAX_DEPTH: usize = 64;

impl Model {
    fn alloc_instance(&mut self) -> Result<EntityId> {
        self.ids.alloc(EntityKind::Instance)
    }

    /// Add a prototype.
    pub fn add_prototype(&mut self, name: &str, members: &[Member]) -> Result<EntityId> {
        if self.instancing.prototypes.values().any(|p| p.name == name) {
            return Err(ModelError::Invalid(format!("duplicate prototype name {name:?}")));
        }
        let id = self.alloc_instance()?;
        self.instancing.prototypes.insert(id, Prototype { name: name.to_owned(), members: members.to_vec() });
        Ok(id)
    }

    /// Add an instance (not yet a root).
    pub fn add_instance(&mut self, fill: Fill, transform: Transform, overrides: Vec<Override>) -> Result<EntityId> {
        let id = self.alloc_instance()?;
        self.instancing.instances.insert(id, InstanceRow { fill, transform, overrides });
        Ok(id)
    }

    /// Add a lattice.
    pub fn add_lattice(&mut self, name: &str, layout: Layout, slots: BTreeMap<SlotIndex, Slot>, outer: Option<Fill>) -> Result<EntityId> {
        layout.check()?;
        if self.instancing.lattices.values().any(|l| l.name == name) {
            return Err(ModelError::Invalid(format!("duplicate lattice name {name:?}")));
        }
        if let Some(bad) = slots.keys().find(|i| !layout.contains(**i)) {
            return Err(ModelError::Invalid(format!("slot {bad:?} is outside the lattice layout")));
        }
        let id = self.alloc_instance()?;
        self.instancing.lattices.insert(id, Lattice { name: name.to_owned(), layout, slots, outer });
        Ok(id)
    }

    /// Mark an instance as top-level.
    pub fn add_root(&mut self, instance: EntityId) -> Result<()> {
        if !self.instancing.instances.contains_key(&instance) {
            return Err(ModelError::Unknown(instance));
        }
        self.instancing.roots.push(instance);
        Ok(())
    }

    /// Flatten all roots into placed copies with resolved colors. Deterministic: depth-first in
    /// table order, slots ascending. Fails on reference cycles or unknown ids.
    pub fn flatten(&self) -> Result<FlatModel> {
        let mut out = Vec::new();
        for r in &self.instancing.roots {
            let inst = self.instancing.instances.get(r).ok_or(ModelError::Unknown(*r))?;
            let placed = Placed::root().child(PathStep::Instance(*r), &inst.transform);
            let mut ov: Vec<&[Override]> = vec![&inst.overrides];
            let mut stack = Vec::new();
            self.flatten_fill(&inst.fill, &placed, &mut ov, &mut stack, &mut out)?;
        }
        let mut ranges: Vec<(Path, Range<usize>)> = Vec::new();
        for (i, e) in out.iter().enumerate() {
            match ranges.last_mut() {
                Some((p, r)) if *p == e.placed.path => r.end = i + 1,
                _ => ranges.push((e.placed.path.clone(), i..i + 1)),
            }
        }
        Ok(FlatModel { entries: out, ranges })
    }

    fn resolve_color(&self, base: Option<EntityId>, volume: Option<EntityId>, ov: &[&[Override]]) -> Option<EntityId> {
        let mut c = base;
        // Innermost level first; outer levels applied later and so win.
        for level in ov.iter().rev() {
            for o in *level {
                match *o {
                    Override::ReplaceColor { from, to } if c == Some(from) => c = Some(to),
                    Override::SetVolume { volume: v, to } if volume == Some(v) => c = Some(to),
                    Override::SetAll { to } => c = Some(to),
                    _ => {}
                }
            }
        }
        c
    }

    fn flatten_fill<'a>(&'a self, fill: &Fill, at: &Placed, ov: &mut Vec<&'a [Override]>, stack: &mut Vec<EntityId>, out: &mut Vec<FlatEntry>) -> Result<()> {
        if stack.len() > MAX_DEPTH {
            return Err(ModelError::Invalid("instancing nesting too deep".into()));
        }
        match *fill {
            Fill::Homogeneous(color) => {
                let c = self.resolve_color(Some(color), None, ov).unwrap_or(color);
                out.push(FlatEntry { placed: at.clone(), content: FlatContent::Homogeneous { color: c } });
            }
            Fill::Prototype(p) => {
                if stack.contains(&p) {
                    return Err(ModelError::Cycle(p));
                }
                let proto = self.instancing.prototypes.get(&p).ok_or(ModelError::Unknown(p))?;
                stack.push(p);
                for m in &proto.members {
                    match *m {
                        Member::Volume(v) => {
                            let color = self.resolve_color(self.volume_color.get(&v).copied(), Some(v), ov);
                            out.push(FlatEntry { placed: at.clone(), content: FlatContent::Volume { volume: v, color } });
                        }
                        Member::Instance(i) => {
                            let inst = self.instancing.instances.get(&i).ok_or(ModelError::Unknown(i))?;
                            let child = at.child(PathStep::Instance(i), &inst.transform);
                            ov.push(&inst.overrides);
                            self.flatten_fill(&inst.fill, &child, ov, stack, out)?;
                            ov.pop();
                        }
                    }
                }
                stack.pop();
            }
            Fill::Lattice(l) => {
                if stack.contains(&l) {
                    return Err(ModelError::Cycle(l));
                }
                let lat = self.instancing.lattices.get(&l).ok_or(ModelError::Unknown(l))?;
                stack.push(l);
                for (idx, tr) in lat.layout.placements() {
                    let (fill, extra, slot_ov): (Fill, Option<Transform>, &'a [Override]) = match lat.slots.get(&idx) {
                        Some(s) => (s.fill, s.transform, &s.overrides),
                        None => match lat.outer {
                            Some(f) => (f, None, &[]),
                            None => continue,
                        },
                    };
                    let local = extra.map_or(tr, |e| crate::action::compose(&tr, &e));
                    let child = at.child(PathStep::Slot { lattice: l, index: idx }, &local);
                    ov.push(slot_ov);
                    self.flatten_fill(&fill, &child, ov, stack, out)?;
                    ov.pop();
                }
                stack.pop();
            }
        }
        Ok(())
    }
}
