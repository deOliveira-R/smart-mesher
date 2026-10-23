//! Instancing: prototypes, instances and nestable lattices (ir.md §5, addendum §7).
//!
//! These are group actions on a base piece, so they produce the same [`Placement`]s as
//! symmetry unfold. Flattening is a deterministic depth-first walk in lexicographic slot
//! order. Override rule: **the override nearest the leaf is applied first**, then each
//! enclosing level's override is applied to the result.

use crate::action::{compose, translation, PathStep, Placement, Slot};
use crate::error::ModelError;
use crate::selection::{Selection, Volume};
use sm_core::{EntityId, EntityKind, Transform, Vector3};
use std::collections::{BTreeMap, BTreeSet};

/// A recoloring `from -> to` applied to a prototype's colors (volume or boundary colors).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ColorMap(BTreeMap<EntityId, EntityId>);

impl ColorMap {
    /// The empty map.
    pub fn new() -> Self {
        Self::default()
    }
    /// Add `from -> to`; both must be `Color` ids.
    pub fn insert(&mut self, from: EntityId, to: EntityId) -> Result<(), ModelError> {
        for id in [from, to] {
            if id.kind() != EntityKind::Color {
                return Err(ModelError::KindMismatch { expected: EntityKind::Color, found: id });
            }
        }
        self.0.insert(from, to);
        Ok(())
    }
    /// Image of `c` (itself if not overridden).
    pub fn get(&self, c: EntityId) -> EntityId {
        self.0.get(&c).copied().unwrap_or(c)
    }
    /// True if there are no overrides.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
    /// Iterate `(from, to)` ascending by `from`.
    pub fn iter(&self) -> impl Iterator<Item = (EntityId, EntityId)> + '_ {
        self.0.iter().map(|(a, b)| (*a, *b))
    }
}

/// Reusable geometry: a set of volumes (and, implicitly, their faces) that instances copy.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Prototype {
    /// Id (`Instance` kind).
    pub id: EntityId,
    /// Unique name.
    pub name: String,
    /// The volumes that make up the prototype.
    pub volumes: Selection<Volume>,
}

/// What a slot or instance places.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Target {
    /// A prototype.
    Prototype(EntityId),
    /// A nested lattice.
    Lattice(EntityId),
}

/// A placed thing with its per-placement overrides and local offset.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Fill {
    /// What is placed.
    pub target: Target,
    /// Color overrides for this placement.
    pub overrides: ColorMap,
    /// Extra transform inside the slot (applied before the slot offset).
    pub local: Transform,
}

impl Fill {
    /// Place `target` unmodified.
    pub fn new(target: Target) -> Self {
        Self { target, overrides: ColorMap::new(), local: Transform::IDENTITY }
    }
    /// Builder: add an override.
    pub fn with_override(mut self, from: EntityId, to: EntityId) -> Result<Self, ModelError> {
        self.overrides.insert(from, to)?;
        Ok(self)
    }
}

/// Orientation of a hexagonal lattice.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum HexOrientation {
    /// Axial basis `a1 = (p, 0)`, `a2 = (p/2, p sqrt(3)/2)`: rows run along x.
    Pointy,
    /// Axial basis `a1 = (p sqrt(3)/2, p/2)`, `a2 = (0, p)`: columns run along y.
    Flat,
}

/// Lattice shape. Slots are `(a, b, c)`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum LatticeGeometry {
    /// `nx * ny * nz` slots `(i, j, k)`, slot origin at `(i px, j py, k pz)`.
    Cartesian {
        /// Slots along x.
        nx: u32,
        /// Slots along y.
        ny: u32,
        /// Slots along z.
        nz: u32,
        /// Pitch per axis.
        pitch: Vector3,
    },
    /// Hexagonal ring lattice. Slots `(q, r, k)` with `|q|, |r|, |q + r| < rings`, `0 <= k < layers`.
    /// `rings = 1` is a single central slot, `rings = 2` is seven slots.
    Hex {
        /// Number of rings including the centre.
        rings: u32,
        /// Number of axial layers.
        layers: u32,
        /// Centre-to-centre pitch in the plane.
        pitch: f64,
        /// Layer pitch along z.
        dz: f64,
        /// In-plane orientation.
        orientation: HexOrientation,
    },
}

impl LatticeGeometry {
    /// Is `s` a slot of this lattice?
    pub fn contains(&self, s: Slot) -> bool {
        match *self {
            LatticeGeometry::Cartesian { nx, ny, nz, .. } => {
                (0..nx as i64).contains(&i64::from(s.a)) && (0..ny as i64).contains(&i64::from(s.b)) && (0..nz as i64).contains(&i64::from(s.c))
            }
            LatticeGeometry::Hex { rings, layers, .. } => {
                let n = i64::from(rings);
                let (q, r) = (i64::from(s.a), i64::from(s.b));
                q.abs() < n && r.abs() < n && (q + r).abs() < n && (0..i64::from(layers)).contains(&i64::from(s.c))
            }
        }
    }

    /// All slots, lexicographic `(a, b, c)` order.
    pub fn all_slots(&self) -> Vec<Slot> {
        let mut v = Vec::new();
        match *self {
            LatticeGeometry::Cartesian { nx, ny, nz, .. } => {
                for a in 0..nx as i32 {
                    for b in 0..ny as i32 {
                        for c in 0..nz as i32 {
                            v.push(Slot::new(a, b, c));
                        }
                    }
                }
            }
            LatticeGeometry::Hex { rings, layers, .. } => {
                let n = rings as i32;
                for q in -(n - 1)..n {
                    for r in -(n - 1)..n {
                        if (q + r).abs() < n {
                            for c in 0..layers as i32 {
                                v.push(Slot::new(q, r, c));
                            }
                        }
                    }
                }
            }
        }
        v
    }

    /// Offset of the slot's origin from the lattice origin.
    pub fn slot_offset(&self, s: Slot) -> Vector3 {
        match *self {
            LatticeGeometry::Cartesian { pitch, .. } => {
                Vector3::new(f64::from(s.a) * pitch.x, f64::from(s.b) * pitch.y, f64::from(s.c) * pitch.z)
            }
            LatticeGeometry::Hex { pitch, dz, orientation, .. } => {
                let (q, r) = (f64::from(s.a), f64::from(s.b));
                let h = 3f64.sqrt() / 2.0;
                let (x, y) = match orientation {
                    HexOrientation::Pointy => (pitch * (q + 0.5 * r), pitch * h * r),
                    HexOrientation::Flat => (pitch * h * q, pitch * (r + 0.5 * q)),
                };
                Vector3::new(x, y, f64::from(s.c) * dz)
            }
        }
    }
}

/// A lattice: geometry plus sparse slot contents.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Lattice {
    /// Id (`Instance` kind).
    pub id: EntityId,
    /// Unique name.
    pub name: String,
    /// Shape.
    pub geometry: LatticeGeometry,
    /// Placement of the lattice origin in its parent frame.
    pub transform: Transform,
    /// Explicit slot contents; `None` is an explicit void.
    pub slots: BTreeMap<Slot, Option<Fill>>,
    /// Content of slots not listed in `slots`. `None`: unlisted slots are empty (sparse).
    pub default_fill: Option<Fill>,
}

impl Lattice {
    /// Effective content of a slot.
    pub fn fill_at(&self, s: Slot) -> Option<&Fill> {
        match self.slots.get(&s) {
            Some(explicit) => explicit.as_ref(),
            None => self.default_fill.as_ref(),
        }
    }
}

/// A top-level placement of a prototype or lattice.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Instance {
    /// Id (`Instance` kind).
    pub id: EntityId,
    /// Unique name.
    pub name: String,
    /// What is placed and how it is recolored.
    pub fill: Fill,
    /// Placement in model space.
    pub transform: Transform,
}

/// One leaf of the flattened instance tree.
#[derive(Clone, Debug, PartialEq)]
pub struct FlatInstance {
    /// Derived id (`Instance` kind), assigned in flatten order.
    pub id: EntityId,
    /// The prototype being copied.
    pub prototype: EntityId,
    /// Composed placement; its path lists every instance and slot on the way down.
    pub placement: Placement,
    /// Fully composed color overrides (identity entries removed).
    pub overrides: ColorMap,
}

impl FlatInstance {
    /// Color of a prototype color `c` in this copy.
    pub fn effective_color(&self, c: EntityId) -> EntityId {
        self.overrides.get(c)
    }
}

/// All instancing records of a model.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Instancing {
    prototypes: BTreeMap<EntityId, Prototype>,
    lattices: BTreeMap<EntityId, Lattice>,
    instances: BTreeMap<EntityId, Instance>,
    names: BTreeSet<String>,
    next: u64,
}

impl Instancing {
    /// An empty store.
    pub fn new() -> Self {
        Self::default()
    }

    fn mint(&mut self, name: &str) -> Result<EntityId, ModelError> {
        if !self.names.insert(name.to_owned()) {
            return Err(ModelError::DuplicateName(name.to_owned()));
        }
        self.next += 1;
        EntityId::new(EntityKind::Instance, self.next).ok_or(ModelError::SerialOverflow(EntityKind::Instance))
    }

    /// Add a prototype.
    pub fn add_prototype(&mut self, name: &str, volumes: Selection<Volume>) -> Result<EntityId, ModelError> {
        let id = self.mint(name)?;
        self.prototypes.insert(id, Prototype { id, name: name.to_owned(), volumes });
        Ok(id)
    }

    /// Add an empty lattice.
    pub fn add_lattice(&mut self, name: &str, geometry: LatticeGeometry, transform: Transform) -> Result<EntityId, ModelError> {
        let id = self.mint(name)?;
        self.lattices.insert(id, Lattice { id, name: name.to_owned(), geometry, transform, slots: BTreeMap::new(), default_fill: None });
        Ok(id)
    }

    fn check_target(&self, t: Target) -> Result<(), ModelError> {
        let ok = match t {
            Target::Prototype(p) => self.prototypes.contains_key(&p),
            Target::Lattice(l) => self.lattices.contains_key(&l),
        };
        if ok {
            Ok(())
        } else {
            Err(ModelError::UnknownEntity(match t {
                Target::Prototype(i) | Target::Lattice(i) => i,
            }))
        }
    }

    /// Fill (or void, with `None`) one slot.
    pub fn set_slot(&mut self, lattice: EntityId, slot: Slot, fill: Option<Fill>) -> Result<(), ModelError> {
        if let Some(f) = &fill {
            self.check_target(f.target)?;
        }
        let l = self.lattices.get_mut(&lattice).ok_or(ModelError::UnknownEntity(lattice))?;
        if !l.geometry.contains(slot) {
            return Err(ModelError::Invalid(format!("slot {slot:?} is outside lattice {lattice}")));
        }
        l.slots.insert(slot, fill);
        Ok(())
    }

    /// Set the content of every slot not listed explicitly.
    pub fn set_default_fill(&mut self, lattice: EntityId, fill: Option<Fill>) -> Result<(), ModelError> {
        if let Some(f) = &fill {
            self.check_target(f.target)?;
        }
        let l = self.lattices.get_mut(&lattice).ok_or(ModelError::UnknownEntity(lattice))?;
        l.default_fill = fill;
        Ok(())
    }

    /// Add a top-level instance.
    pub fn add_instance(&mut self, name: &str, fill: Fill, transform: Transform) -> Result<EntityId, ModelError> {
        self.check_target(fill.target)?;
        let id = self.mint(name)?;
        self.instances.insert(id, Instance { id, name: name.to_owned(), fill, transform });
        Ok(id)
    }

    /// Prototypes, ascending by id.
    pub fn prototypes(&self) -> impl Iterator<Item = &Prototype> {
        self.prototypes.values()
    }
    /// Lattices, ascending by id.
    pub fn lattices(&self) -> impl Iterator<Item = &Lattice> {
        self.lattices.values()
    }
    /// Instances, ascending by id.
    pub fn instances(&self) -> impl Iterator<Item = &Instance> {
        self.instances.values()
    }
    /// Lookup a prototype.
    pub fn prototype(&self, id: EntityId) -> Option<&Prototype> {
        self.prototypes.get(&id)
    }
    /// Lookup a lattice.
    pub fn lattice(&self, id: EntityId) -> Option<&Lattice> {
        self.lattices.get(&id)
    }

    /// Flatten every top-level instance, ascending by instance id. Derived ids are
    /// `Instance#(id_base + n)` with `n` the emission index.
    pub fn flatten_all(&self, id_base: u64) -> Result<Vec<FlatInstance>, ModelError> {
        let mut out = Vec::new();
        for inst in self.instances.values() {
            self.flatten_into(inst, id_base, &mut out)?;
        }
        Ok(out)
    }

    /// Flatten one top-level instance.
    pub fn flatten(&self, instance: EntityId, id_base: u64) -> Result<Vec<FlatInstance>, ModelError> {
        let inst = self.instances.get(&instance).ok_or(ModelError::UnknownEntity(instance))?;
        let mut out = Vec::new();
        self.flatten_into(inst, id_base, &mut out)?;
        Ok(out)
    }

    fn flatten_into(&self, inst: &Instance, id_base: u64, out: &mut Vec<FlatInstance>) -> Result<(), ModelError> {
        let base = Placement::root().then(&inst.transform, PathStep::Instance(inst.id));
        let mut chain: Vec<&ColorMap> = Vec::new();
        let mut stack: Vec<EntityId> = Vec::new();
        self.walk(&inst.fill, &base, &mut chain, &mut stack, id_base, out)
    }

    fn walk<'a>(
        &'a self,
        fill: &'a Fill,
        base: &Placement,
        chain: &mut Vec<&'a ColorMap>,
        stack: &mut Vec<EntityId>,
        id_base: u64,
        out: &mut Vec<FlatInstance>,
    ) -> Result<(), ModelError> {
        let transform = compose(&base.transform, &fill.local);
        let here = Placement { orientation_reversed: transform.is_improper(), transform, path: base.path.clone() };
        chain.push(&fill.overrides);
        match fill.target {
            Target::Prototype(p) => {
                let id = EntityId::new(EntityKind::Instance, id_base + out.len() as u64)
                    .ok_or(ModelError::SerialOverflow(EntityKind::Instance))?;
                out.push(FlatInstance { id, prototype: p, placement: here, overrides: collapse(chain) });
            }
            Target::Lattice(lid) => {
                if stack.contains(&lid) {
                    return Err(ModelError::Cycle(lid));
                }
                let lat = self.lattices.get(&lid).ok_or(ModelError::UnknownEntity(lid))?;
                stack.push(lid);
                // Union of explicit slots and (if any) the default over all slots, in slot order.
                let slots: Vec<Slot> = if lat.default_fill.is_some() {
                    lat.geometry.all_slots()
                } else {
                    lat.slots.keys().copied().collect()
                };
                for s in slots {
                    let Some(child) = lat.fill_at(s) else { continue };
                    let local = compose(&lat.transform, &translation(lat.geometry.slot_offset(s)));
                    let child_base = here.then(&local, PathStep::Slot { lattice: lid, slot: s });
                    self.walk(child, &child_base, chain, stack, id_base, out)?;
                }
                stack.pop();
            }
        }
        chain.pop();
        Ok(())
    }
}

/// Collapse an outermost-first chain of override maps into one map (innermost applied first).
fn collapse(chain: &[&ColorMap]) -> ColorMap {
    let keys: BTreeSet<EntityId> = chain.iter().flat_map(|m| m.0.keys().copied()).collect();
    let mut out = ColorMap::new();
    for k in keys {
        let v = chain.iter().rev().fold(k, |c, m| m.get(c));
        if v != k {
            out.0.insert(k, v);
        }
    }
    out
}
