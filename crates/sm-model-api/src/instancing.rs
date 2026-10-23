//! Instancing: prototypes, instances and lattices, as group actions on a fundamental piece.
//!
//! A [`Prototype`] is the fundamental piece (a set of volumes of the base model). An [`Instance`] is
//! one placement. A [`Lattice`] is a finite window of a translation action (cartesian `Z^3` or hex
//! axial coordinates `(q, r, k)`), nestable: a slot may hold another lattice. This is the same
//! machinery as symmetry ([`crate::group`]): `Lattice` implements [`GroupAction`], and a symmetric
//! (reduced) model composes with a lattice through [`Flattened::apply_group`].
//!
//! [`Instancing::flatten`] is the functor from the instance tree to a flat list of
//! `(path, prototype, transform, recolor chain)` in a documented canonical order.
//!
//! **Canonical order.** Slots are visited by `(k, row, col)` ascending (cartesian: `k=z`, `row=y`,
//! `col=x`; hex: `k` axial layer, `row=r`, `col=q`), depth first, so output never depends on
//! insertion order of the (ordered) slot map.
//!
//! **Recoloring (the one place equivariance breaks).** Each placement may carry a [`Recolor`]. The
//! effective color of a prototype volume inside an instance path is
//! `σ_outer ∘ … ∘ σ_inner (base color)`: inner recolors apply first, outer last. Within one
//! [`Recolor`], a per-volume override wins over the color substitution.

use crate::error::{ModelError, Result};
use crate::group::{DeckElement, GroupAction, Placement};
use crate::ids::IdAllocator;
use crate::xf::{compose, translation};
use sm_core::{EntityId, EntityKind, Point3, Transform, Vector3};
use smallvec::SmallVec;
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

/// A recoloring applied at one level of an instance path.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Recolor {
    /// Prototype volume -> color (absolute override).
    pub by_volume: BTreeMap<EntityId, EntityId>,
    /// Color -> color substitution.
    pub by_color: BTreeMap<EntityId, EntityId>,
}

impl Recolor {
    /// True if it changes nothing.
    pub fn is_identity(&self) -> bool {
        self.by_volume.is_empty() && self.by_color.is_empty()
    }
    /// Apply to a volume currently colored `color`.
    pub fn apply(&self, volume: EntityId, color: EntityId) -> EntityId {
        self.by_volume.get(&volume).or_else(|| self.by_color.get(&color)).copied().unwrap_or(color)
    }
}

/// The fundamental piece: a set of volumes of the base model.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Prototype {
    /// `EntityKind::Instance` id.
    pub id: EntityId,
    /// Display name.
    pub name: String,
    /// Volumes making up the piece.
    pub volumes: BTreeSet<EntityId>,
}

/// What a placement or slot holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Fill {
    /// Nothing (sparse slot).
    Empty,
    /// A prototype, instance or lattice (all `EntityKind::Instance` ids).
    Ref(EntityId),
}

/// A placed fill: content, local transform and recolor.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Placed {
    /// Content.
    pub fill: Fill,
    /// Local transform (applied before the slot/instance position). May be improper.
    pub xf: Transform,
    /// Recolor applied to everything inside.
    pub recolor: Recolor,
}

impl Placed {
    /// Identity placement of `fill`.
    pub fn of(fill: Fill) -> Self {
        Self { fill, xf: Transform::IDENTITY, recolor: Recolor::default() }
    }
}

/// A single placed instance.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Instance {
    /// `EntityKind::Instance` id.
    pub id: EntityId,
    /// Display name.
    pub name: String,
    /// Placement.
    pub placed: Placed,
}

/// Hex lattice orientation (direction of the first axial basis vector `a1`; `a2` is `a1` rotated by 60 degrees).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum HexOrientation {
    /// `a1` along +x (rows along x).
    A1AlongX,
    /// `a1` at 30 degrees (`a2` along +y).
    A1At30,
}

/// Lattice extent and spacing.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum LatticeGeometry {
    /// `dims[0] x dims[1] x dims[2]` slots, `col in 0..nx`, `row in 0..ny`, `k in 0..nz`; slot
    /// `(0,0,0)` is centred at `origin`.
    Cartesian {
        /// Slots per axis.
        dims: [u32; 3],
        /// Pitch per axis.
        pitch: [f64; 3],
        /// Centre of slot (0,0,0).
        origin: Point3,
    },
    /// Hexagonal array of `rings` rings (1 = a single slot), `layers` axial layers. Slot `(q, r)`
    /// satisfies `|q|,|r|,|q+r| < rings`; centre is `origin + q a1 + r a2 + k layer_pitch ẑ`.
    Hex {
        /// Number of rings (>= 1).
        rings: u32,
        /// Axial layers (>= 1).
        layers: u32,
        /// Centre-to-centre pitch in the plane.
        pitch: f64,
        /// Axial layer pitch.
        layer_pitch: f64,
        /// In-plane orientation.
        orientation: HexOrientation,
        /// Centre of slot `(0,0,0)`.
        origin: Point3,
    },
}

/// Canonical slot address. Ordering is `(k, row, col)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SlotIndex {
    /// Axial layer (`z` index).
    pub k: i32,
    /// Row (`y` index; hex `r`).
    pub row: i32,
    /// Column (`x` index; hex `q`).
    pub col: i32,
}

impl SlotIndex {
    /// Construct from `(col, row, k)` = `(x or q, y or r, z)`.
    pub const fn new(col: i32, row: i32, k: i32) -> Self {
        Self { k, row, col }
    }
}

/// A lattice: geometry, a default fill for homogeneous slots, and sparse per-slot overrides.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Lattice {
    /// `EntityKind::Instance` id.
    pub id: EntityId,
    /// Display name.
    pub name: String,
    /// Extent and spacing.
    pub geometry: LatticeGeometry,
    /// Fill of every slot not in `slots`.
    pub default: Placed,
    /// Sparse slot overrides (including `Fill::Empty` holes).
    pub slots: BTreeMap<SlotIndex, Placed>,
    /// Fill outside the array (background); not part of flatten, for exporters.
    pub outer: Fill,
}

impl Lattice {
    /// Slot addresses in canonical order.
    pub fn slot_indices(&self) -> Vec<SlotIndex> {
        let mut out = Vec::new();
        match self.geometry {
            LatticeGeometry::Cartesian { dims, .. } => {
                for k in 0..dims[2] as i32 {
                    for row in 0..dims[1] as i32 {
                        for col in 0..dims[0] as i32 {
                            out.push(SlotIndex { k, row, col });
                        }
                    }
                }
            }
            LatticeGeometry::Hex { rings, layers, .. } => {
                let n = rings as i32 - 1;
                for k in 0..layers as i32 {
                    for row in -n..=n {
                        for col in -n..=n {
                            if (col + row).abs() <= n {
                                out.push(SlotIndex { k, row, col });
                            }
                        }
                    }
                }
            }
        }
        out
    }

    /// True if `s` lies inside the array.
    pub fn contains(&self, s: SlotIndex) -> bool {
        match self.geometry {
            LatticeGeometry::Cartesian { dims, .. } => {
                (0..dims[0] as i32).contains(&s.col) && (0..dims[1] as i32).contains(&s.row) && (0..dims[2] as i32).contains(&s.k)
            }
            LatticeGeometry::Hex { rings, layers, .. } => {
                let n = rings as i32 - 1;
                s.col.abs() <= n && s.row.abs() <= n && (s.col + s.row).abs() <= n && (0..layers as i32).contains(&s.k)
            }
        }
    }

    /// Centre of a slot.
    pub fn slot_center(&self, s: SlotIndex) -> Vector3 {
        match self.geometry {
            LatticeGeometry::Cartesian { pitch, origin, .. } => Vector3::new(
                origin.x + f64::from(s.col) * pitch[0],
                origin.y + f64::from(s.row) * pitch[1],
                origin.z + f64::from(s.k) * pitch[2],
            ),
            LatticeGeometry::Hex { pitch, layer_pitch, orientation, origin, .. } => {
                let th0 = match orientation {
                    HexOrientation::A1AlongX => 0.0,
                    HexOrientation::A1At30 => std::f64::consts::FRAC_PI_6,
                };
                let th1 = th0 + std::f64::consts::FRAC_PI_3;
                let (q, r) = (f64::from(s.col), f64::from(s.row));
                Vector3::new(
                    origin.x + pitch * (q * th0.cos() + r * th1.cos()),
                    origin.y + pitch * (q * th0.sin() + r * th1.sin()),
                    origin.z + f64::from(s.k) * layer_pitch,
                )
            }
        }
    }

    /// What a slot holds (override or default).
    pub fn placed_at(&self, s: SlotIndex) -> &Placed {
        self.slots.get(&s).unwrap_or(&self.default)
    }

    fn check_geometry(&self) -> Result<()> {
        match self.geometry {
            LatticeGeometry::Cartesian { dims, pitch, .. } => {
                if dims.contains(&0) || pitch.iter().any(|p| !(p.is_finite() && *p > 0.0)) {
                    return Err(ModelError::InvalidLattice("cartesian dims and pitch must be positive".into()));
                }
            }
            LatticeGeometry::Hex { rings, layers, pitch, layer_pitch, .. } => {
                if rings == 0 || layers == 0 || !(pitch.is_finite() && pitch > 0.0) || !layer_pitch.is_finite() || (layers > 1 && layer_pitch <= 0.0) {
                    return Err(ModelError::InvalidLattice("hex rings/layers/pitch invalid".into()));
                }
            }
        }
        Ok(())
    }
}

/// The lattice as a pure translation action: one placement per slot (including empty ones), in
/// canonical order. Slot-local transforms and fills are applied by [`Instancing::flatten`].
impl GroupAction for Lattice {
    type Label = SlotIndex;
    fn placements(&self) -> Result<Vec<Placement<SlotIndex>>> {
        self.check_geometry()?;
        Ok(self.slot_indices().into_iter().map(|s| Placement::new(s, translation(self.slot_center(s)))).collect())
    }
}

/// One step of an instance path.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PathStep {
    /// The instance or lattice entered.
    pub container: EntityId,
    /// Slot within a lattice (`None` for a plain instance).
    pub slot: Option<SlotIndex>,
}

/// Path from the flatten root to a leaf; unique per leaf and stable across runs.
#[derive(Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct InstancePath(pub SmallVec<[PathStep; 4]>);

/// One leaf of a flattened instance tree.
#[derive(Clone, Debug)]
pub struct FlatItem {
    /// Path from the root.
    pub path: InstancePath,
    /// The prototype (or, when flattening was depth-limited, the unexpanded container).
    pub target: EntityId,
    /// Prototype coordinates -> root coordinates.
    pub xf: Transform,
    /// True if `xf` reverses orientation (flip loops / left-right).
    pub improper: bool,
    /// Recolors from outermost to innermost.
    pub recolors: SmallVec<[Arc<Recolor>; 2]>,
    /// Deck-group element, when produced by [`Flattened::apply_group`].
    pub element: Option<DeckElement>,
}

impl FlatItem {
    /// Effective color of prototype volume `volume` in this instance: base color pulled back, then
    /// recolored innermost-first.
    pub fn volume_color(&self, coloring: &crate::coloring::Coloring, volume: EntityId) -> Option<EntityId> {
        let mut c = coloring.volume_color(volume)?;
        for r in self.recolors.iter().rev() {
            c = r.apply(volume, c);
        }
        Some(c)
    }
}

/// Flatten options.
#[derive(Clone, Copy, Debug, Default)]
pub struct FlattenOptions {
    /// Maximum container nesting to expand (`None` = fully). At the limit the container itself is
    /// emitted as the item `target`.
    pub depth: Option<u32>,
}

/// Result of flattening: leaves in canonical order.
#[derive(Clone, Debug, Default)]
pub struct Flattened {
    /// Leaves.
    pub items: Vec<FlatItem>,
}

impl Flattened {
    /// Compose with a group action (e.g. unfold a symmetric reduced lattice): for every placement
    /// `p` (outer loop) and every item (inner loop), emit the item transformed by `p`.
    pub fn apply_group<A: GroupAction<Label = DeckElement>>(&self, action: &A) -> Result<Flattened> {
        let ps = action.placements()?;
        let mut items = Vec::with_capacity(ps.len() * self.items.len());
        for p in &ps {
            for it in &self.items {
                let xf = compose(&p.xf, &it.xf);
                items.push(FlatItem { xf, improper: xf.is_improper(), element: Some(p.label.clone()), ..it.clone() });
            }
        }
        Ok(Flattened { items })
    }

    /// 64-bit FNV-1a fingerprint of the full content (paths, targets, transforms bitwise, recolors).
    /// Stable across runs and thread counts; a cache key helper and a determinism probe.
    pub fn fingerprint(&self) -> u64 {
        let mut h = Fnv::new();
        for it in &self.items {
            for s in &it.path.0 {
                h.u64(s.container.to_raw());
                match s.slot {
                    Some(k) => {
                        h.u64(1);
                        h.u64(k.k as u64);
                        h.u64(k.row as u64);
                        h.u64(k.col as u64);
                    }
                    None => h.u64(0),
                }
            }
            h.u64(it.target.to_raw());
            for row in &it.xf.m {
                for v in row {
                    h.u64(v.to_bits());
                }
            }
            h.u64(it.xf.t.x.to_bits());
            h.u64(it.xf.t.y.to_bits());
            h.u64(it.xf.t.z.to_bits());
            for r in &it.recolors {
                for (a, b) in r.by_volume.iter().chain(r.by_color.iter()) {
                    h.u64(a.to_raw());
                    h.u64(b.to_raw());
                }
            }
            if let Some(e) = &it.element {
                h.u64(u64::from(e.point));
                for s in &e.shift {
                    h.u64(u64::from(*s));
                }
            }
        }
        h.0
    }
}

struct Fnv(u64);
impl Fnv {
    fn new() -> Self {
        Fnv(0xcbf29ce484222325)
    }
    fn u64(&mut self, v: u64) {
        for b in v.to_le_bytes() {
            self.0 ^= u64::from(b);
            self.0 = self.0.wrapping_mul(0x100000001b3);
        }
    }
}

/// Registry of prototypes, instances and lattices.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Instancing {
    /// Prototypes by id.
    pub prototypes: BTreeMap<EntityId, Prototype>,
    /// Instances by id.
    pub instances: BTreeMap<EntityId, Instance>,
    /// Lattices by id.
    pub lattices: BTreeMap<EntityId, Lattice>,
}

impl Instancing {
    /// An empty registry.
    pub fn new() -> Self {
        Self::default()
    }

    /// Define a prototype.
    pub fn add_prototype(&mut self, ids: &mut dyn IdAllocator, name: &str, volumes: impl IntoIterator<Item = EntityId>) -> Result<EntityId> {
        let volumes: BTreeSet<EntityId> = volumes.into_iter().collect();
        if let Some(&bad) = volumes.iter().find(|v| v.kind() != EntityKind::TopoVolume) {
            return Err(ModelError::WrongKind { id: bad, found: bad.kind(), expected: "TopoVolume" });
        }
        let id = ids.alloc(EntityKind::Instance);
        self.prototypes.insert(id, Prototype { id, name: name.to_string(), volumes });
        Ok(id)
    }

    /// Define an instance.
    pub fn add_instance(&mut self, ids: &mut dyn IdAllocator, name: &str, placed: Placed) -> Result<EntityId> {
        self.check_fill(placed.fill)?;
        let id = ids.alloc(EntityKind::Instance);
        self.instances.insert(id, Instance { id, name: name.to_string(), placed });
        Ok(id)
    }

    /// Define a lattice with a default fill.
    pub fn add_lattice(&mut self, ids: &mut dyn IdAllocator, name: &str, geometry: LatticeGeometry, default: Placed) -> Result<EntityId> {
        self.check_fill(default.fill)?;
        let id = ids.alloc(EntityKind::Instance);
        let lat = Lattice { id, name: name.to_string(), geometry, default, slots: BTreeMap::new(), outer: Fill::Empty };
        lat.check_geometry()?;
        self.lattices.insert(id, lat);
        Ok(id)
    }

    /// Override one slot (sparse). The slot must lie inside the array.
    pub fn set_slot(&mut self, lattice: EntityId, slot: SlotIndex, placed: Placed) -> Result<()> {
        self.check_fill(placed.fill)?;
        let lat = self.lattices.get_mut(&lattice).ok_or(ModelError::UnknownEntity(lattice))?;
        if !lat.contains(slot) {
            return Err(ModelError::InvalidLattice(format!("slot {slot:?} outside lattice {lattice}")));
        }
        lat.slots.insert(slot, placed);
        Ok(())
    }

    fn check_fill(&self, f: Fill) -> Result<()> {
        match f {
            Fill::Empty => Ok(()),
            Fill::Ref(id) if self.prototypes.contains_key(&id) || self.instances.contains_key(&id) || self.lattices.contains_key(&id) => Ok(()),
            Fill::Ref(id) => Err(ModelError::UnknownEntity(id)),
        }
    }

    /// Check that all references resolve and the container graph is acyclic (cheap: does not
    /// enumerate slots).
    pub fn check_references(&self) -> Result<()> {
        let refs = |id: EntityId| -> Vec<EntityId> {
            let mut out = Vec::new();
            let mut push = |f: Fill| {
                if let Fill::Ref(r) = f {
                    out.push(r);
                }
            };
            if let Some(i) = self.instances.get(&id) {
                push(i.placed.fill);
            }
            if let Some(l) = self.lattices.get(&id) {
                push(l.default.fill);
                push(l.outer);
                for p in l.slots.values() {
                    push(p.fill);
                }
            }
            out
        };
        let mut state: BTreeMap<EntityId, bool> = BTreeMap::new(); // false = in progress, true = done
        let roots: Vec<EntityId> = self.instances.keys().chain(self.lattices.keys()).copied().collect();
        for root in roots {
            let mut stack: Vec<(EntityId, Vec<EntityId>)> = vec![(root, refs(root))];
            if state.get(&root) == Some(&true) {
                continue;
            }
            state.insert(root, false);
            while let Some((node, pending)) = stack.last_mut() {
                match pending.pop() {
                    Some(c) => {
                        if !(self.prototypes.contains_key(&c) || self.instances.contains_key(&c) || self.lattices.contains_key(&c)) {
                            return Err(ModelError::UnknownEntity(c));
                        }
                        match state.get(&c) {
                            Some(false) => return Err(ModelError::Cycle(c)),
                            Some(true) => {}
                            None => {
                                if self.prototypes.contains_key(&c) {
                                    state.insert(c, true);
                                } else {
                                    state.insert(c, false);
                                    stack.push((c, refs(c)));
                                }
                            }
                        }
                    }
                    None => {
                        state.insert(*node, true);
                        stack.pop();
                    }
                }
            }
        }
        Ok(())
    }

    /// Flatten the tree under `root` (a prototype, instance or lattice id) into leaves, in canonical
    /// order. Errors on dangling references and cycles.
    pub fn flatten(&self, root: EntityId, opts: FlattenOptions) -> Result<Flattened> {
        let mut out = Flattened::default();
        let mut stack = Vec::new();
        let mut chain: Vec<Arc<Recolor>> = Vec::new();
        let mut path = InstancePath::default();
        self.visit(root, Transform::IDENTITY, &mut chain, &mut path, 0, opts, &mut stack, &mut out)?;
        Ok(out)
    }

    #[allow(clippy::too_many_arguments)]
    fn visit(
        &self,
        id: EntityId,
        xf: Transform,
        chain: &mut Vec<Arc<Recolor>>,
        path: &mut InstancePath,
        level: u32,
        opts: FlattenOptions,
        stack: &mut Vec<EntityId>,
        out: &mut Flattened,
    ) -> Result<()> {
        let emit = |out: &mut Flattened, chain: &Vec<Arc<Recolor>>, path: &InstancePath| {
            out.items.push(FlatItem {
                path: path.clone(),
                target: id,
                xf,
                improper: xf.is_improper(),
                recolors: chain.iter().cloned().collect(),
                element: None,
            });
        };
        if self.prototypes.contains_key(&id) {
            emit(out, chain, path);
            return Ok(());
        }
        let is_container = self.instances.contains_key(&id) || self.lattices.contains_key(&id);
        if !is_container {
            return Err(ModelError::UnknownEntity(id));
        }
        if stack.contains(&id) {
            return Err(ModelError::Cycle(id));
        }
        if opts.depth.is_some_and(|d| level >= d) {
            emit(out, chain, path);
            return Ok(());
        }
        stack.push(id);
        if let Some(inst) = self.instances.get(&id) {
            self.descend(&inst.placed, Arc::new(inst.placed.recolor.clone()), PathStep { container: id, slot: None }, xf, chain, path, level, opts, stack, out)?;
        } else if let Some(lat) = self.lattices.get(&id) {
            lat.check_geometry()?;
            let def_arc = Arc::new(lat.default.recolor.clone());
            let slot_arcs: BTreeMap<SlotIndex, Arc<Recolor>> = lat.slots.iter().map(|(&s, p)| (s, Arc::new(p.recolor.clone()))).collect();
            for s in lat.slot_indices() {
                let placed = lat.placed_at(s);
                if placed.fill == Fill::Empty {
                    continue;
                }
                let arc = slot_arcs.get(&s).cloned().unwrap_or_else(|| def_arc.clone());
                let at = compose(&xf, &translation(lat.slot_center(s)));
                self.descend(placed, arc, PathStep { container: id, slot: Some(s) }, at, chain, path, level, opts, stack, out)?;
            }
        }
        stack.pop();
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn descend(
        &self,
        placed: &Placed,
        recolor: Arc<Recolor>,
        step: PathStep,
        base: Transform,
        chain: &mut Vec<Arc<Recolor>>,
        path: &mut InstancePath,
        level: u32,
        opts: FlattenOptions,
        stack: &mut Vec<EntityId>,
        out: &mut Flattened,
    ) -> Result<()> {
        let Fill::Ref(child) = placed.fill else { return Ok(()) };
        let pushed = !recolor.is_identity();
        if pushed {
            chain.push(recolor);
        }
        path.0.push(step);
        let r = self.visit(child, compose(&base, &placed.xf), chain, path, level + 1, opts, stack, out);
        path.0.pop();
        if pushed {
            chain.pop();
        }
        r
    }
}
