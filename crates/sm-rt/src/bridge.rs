//! Model to CSG world bridge: [`csg_world`] turns the volumes, prototypes, lattices and clusters
//! of a model into CSG leaves and a [`WorldSpec`], and [`CsgWorld`] hosts the result.
//!
//! # Mapping
//! * A **volume** becomes one cell named by its color (or its id when uncolored). Its region is the
//!   intersection of one half-space per face use: the face's surface, on the side the volume lies
//!   (`Side::Left` means the face normal points out of the volume; the carrier `Sense` says whether
//!   the face normal is the surface normal). A volume whose faces use one surface on both sides, a
//!   baffle, or a face without a surface is not such an intersection and is refused. A volume
//!   with a `Carrier::Csg` is taken as given.
//! * Supported surfaces are planes, spheres, cylinders, tori, implicit quadrics and one nappe of a
//!   cone (the analytic kernel's cone is a double cone, so the nappe's apex plane is added as a
//!   half-space; a volume *outside* a cone is refused). NURBS faces are [`BridgeError::Unsupported`]
//!   and name the face.
//! * A **prototype** becomes a leaf, a **cluster** or the top-level instances a `Union`, a
//!   Cartesian **lattice** a rectangular `Lattice` (hex layouts, recolors and clips are refused).
//!   Volumes that belong to no prototype are cells of a root leaf.
//! * The faces of a volume are assumed to cover their whole surface and the volume is assumed to
//!   equal the intersection of its face half-spaces (true for convex pieces and for annuli of
//!   coaxial quadrics). Other volumes are not detected here; the sampled check
//!   (`rt.check_overlaps`) finds the overlaps and gaps they cause.
//!
//! [`CsgWorld::world_tracker`] hosts the world as it is: the world walker tiles and places analytic
//! prototype leaves, so a lattice of 2601 pins costs one pin and an index. The analytic tracker
//! hosts one CSG leaf, so [`CsgWorld::flatten`] expands the world into one leaf (surfaces moved with
//! `SurfaceDef::transformed`); [`CsgWorld::analytic_tracker`] does that and is the oracle of the
//! hierarchical build. Flattening never changes tracking results, only the number of cells.

use crate::analytic::{AnalyticTracker, AnalyticTrackerBuilder, check_located};
use crate::builder::MeshTrackerBuilder;
use crate::world::WorldTracker;
use sm_core::{EntityId, Transform};
use sm_geom_api::{CsgCell, Frame, GeomAppend, GeomStore, HalfSpace, MemoryStore, Plane, Sense, Sign, SurfaceDef};
use sm_model_api::{Fill, Layout, Model};
use sm_rt_api::{
    BoundarySpec, BuildInput, BuildIssue, BuildOptions, BuildReport, ComplementPolicy, ComponentId, ComponentKind, CsgCellDef, CsgLeafInput, CsgRegion, HierInput, InstancePolicy, Lattice, LatticeKind,
    LeafInput, Placement, RtError, WorldSpec,
};
use sm_topo_api::{Carrier, Dim, Side, TopologyView};
use std::collections::{BTreeMap, BTreeSet};

/// Why a model cannot be bridged.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum BridgeError {
    /// Something the analytic CSG tracker cannot represent, with the entity concerned.
    #[error("unsupported{}: {what}", .entity.map(|e| format!(" ({e})")).unwrap_or_default())]
    Unsupported {
        /// The face, volume, lattice or instance at fault.
        entity: Option<EntityId>,
        /// What it is and why it is refused.
        what: String,
    },
    /// The model or the request is inconsistent.
    #[error("{0}")]
    Invalid(String),
}

impl From<BridgeError> for RtError {
    fn from(e: BridgeError) -> Self {
        match &e {
            BridgeError::Unsupported { entity, what } => {
                let mut i = BuildIssue::new("csg.unsupported", what.clone());
                if let Some(en) = entity {
                    i = i.with_entity(*en);
                }
                RtError::Build(vec![i])
            }
            BridgeError::Invalid(m) => RtError::Build(vec![BuildIssue::new("csg.invalid", m.clone())]),
        }
    }
}

fn unsupported(entity: impl Into<Option<EntityId>>, what: impl Into<String>) -> BridgeError {
    BridgeError::Unsupported { entity: entity.into(), what: what.into() }
}

fn invalid(m: impl Into<String>) -> BridgeError {
    BridgeError::Invalid(m.into())
}

/// What [`csg_world`] reads besides the model.
#[derive(Clone, Debug, Default)]
pub struct CsgWorldOptions {
    /// Bodies or volumes to include (default: everything). Applies to prototype volumes and to
    /// volumes outside any prototype alike.
    pub bodies: Option<Vec<EntityId>>,
    /// Name of the region that fills the space inside the leaf bounds but outside every cell
    /// (default: leaving the cells leaves the model). Needs bounded cells.
    pub complement: Option<String>,
}

/// Where a cell of a [`CsgWorld`] leaf comes from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CellInfo {
    /// The model volume.
    pub volume: EntityId,
    /// The cell (region) name: the volume's color name.
    pub name: String,
    /// Which placed copy (in depth-first placement order) in a flattened leaf; 0 in a prototype leaf.
    pub copy: u32,
}

struct LeafCells {
    cells: Vec<CsgCellDef>,
    info: Vec<CellInfo>,
}

/// A model expressed as CSG leaves plus the world graph that places them. Owns its geometry.
pub struct CsgWorld {
    store: MemoryStore,
    spec: WorldSpec,
    leaves: Vec<LeafCells>,
    complement: ComplementPolicy,
}

impl core::fmt::Debug for CsgWorld {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("CsgWorld").field("leaves", &self.leaves.len()).field("components", &self.spec.components.len()).finish()
    }
}

/// z pitch of a 2D lattice: the model gives none, and the world spec needs a positive one that
/// holds the whole element.
const PITCH_2D: f64 = 1e9;

/// Default cap on the cells of a flattened world.
pub const MAX_FLAT_CELLS: usize = 1 << 21;

struct Builder<'a> {
    model: &'a Model,
    topo: &'a dyn TopologyView,
    selected: Option<BTreeSet<EntityId>>,
    store: MemoryStore,
    by_def: BTreeMap<String, EntityId>,
    comps: Vec<ComponentKind>,
    leaves: Vec<LeafCells>,
    protos: BTreeMap<EntityId, Option<ComponentId>>,
    lattices: BTreeMap<EntityId, Option<ComponentId>>,
    clusters: BTreeMap<EntityId, Option<ComponentId>>,
    void: Option<ComponentId>,
    depth: usize,
}

fn add_def(store: &mut MemoryStore, by_def: &mut BTreeMap<String, EntityId>, def: SurfaceDef) -> Result<EntityId, BridgeError> {
    let key = format!("{def:?}");
    if let Some(id) = by_def.get(&key) {
        return Ok(*id);
    }
    let id = store.add_surface(def).map_err(|e| invalid(format!("surface: {e}")))?;
    by_def.insert(key, id);
    Ok(id)
}

fn opposite(s: Sign) -> Sign {
    match s {
        Sign::Negative => Sign::Positive,
        Sign::Positive => Sign::Negative,
    }
}

impl<'a> Builder<'a> {
    fn push(&mut self, k: ComponentKind) -> ComponentId {
        self.comps.push(k);
        ComponentId((self.comps.len() - 1) as u32)
    }

    fn wanted(&self, v: EntityId) -> bool {
        self.selected.as_ref().is_none_or(|s| s.contains(&v))
    }

    fn volume_name(&self, v: EntityId) -> String {
        self.model.volume_color(v).and_then(|c| self.model.color(c)).map_or_else(|| v.to_string(), |c| c.name.clone())
    }

    /// The half-spaces that say "on `sign` side of `surf`", `entity` being the cell blamed on refusal.
    fn halfspaces(&mut self, surf: EntityId, sign: Sign, entity: EntityId) -> Result<Vec<HalfSpace>, BridgeError> {
        let def = self.topo.shapes().surface_def(surf).ok_or_else(|| invalid(format!("{entity}: surface {surf} is not in the geometry store")))?.clone();
        match &def {
            SurfaceDef::Nurbs(_) => return Err(unsupported(entity, format!("{entity} lies on NURBS surface {surf}; only planes, spheres, cylinders, cones, tori and quadrics are tracked analytically"))),
            SurfaceDef::EllipticCone(_) => return Err(unsupported(entity, format!("{entity} lies on an elliptic cone ({surf}); its nappe cannot be selected"))),
            SurfaceDef::Cone(_) if sign == Sign::Positive => {
                return Err(unsupported(entity, format!("{entity} lies outside the cone {surf}; the region is not an intersection of half-spaces")));
            }
            _ => {}
        }
        let id = add_def(&mut self.store, &mut self.by_def, def.clone())?;
        let mut out = vec![HalfSpace { surface: id, sign }];
        if let SurfaceDef::Cone(c) = &def {
            // The kernel's cone is a double cone: keep the nappe of the surface.
            let frame = Frame::from_z(c.apex(), c.frame.z).map_err(|e| invalid(format!("{entity}: cone apex: {e}")))?;
            let plane = add_def(&mut self.store, &mut self.by_def, SurfaceDef::Plane(Plane { frame }))?;
            out.push(HalfSpace { surface: plane, sign: Sign::Positive });
        }
        Ok(out)
    }

    fn cell_of(&mut self, v: EntityId) -> Result<(CsgCellDef, CellInfo), BridgeError> {
        let name = self.volume_name(v);
        let mut hs: Vec<HalfSpace> = Vec::new();
        let push = |hs: &mut Vec<HalfSpace>, h: HalfSpace| -> Result<(), BridgeError> {
            if hs.contains(&h) {
                return Ok(());
            }
            if hs.iter().any(|o| o.surface == h.surface) {
                return Err(unsupported(v, format!("{v} uses one surface on both sides, so it is not an intersection of half-spaces")));
            }
            hs.push(h);
            Ok(())
        };
        if let Carrier::Csg(cell) = self.topo.carrier(v) {
            for h in cell.halfspaces.clone() {
                for x in self.halfspaces(h.surface, h.sign, v)? {
                    push(&mut hs, x)?;
                }
            }
        } else {
            let faces: Vec<_> = self.topo.boundary(v).iter().filter(|i| self.topo.dim_of(i.cell) == Some(Dim::Face)).copied().collect();
            if faces.is_empty() {
                return Err(invalid(format!("{v} has no faces")));
            }
            for inc in faces {
                let face = inc.cell;
                let sc = match self.topo.carrier(face) {
                    Carrier::Surface(sc) => *sc,
                    Carrier::None => return Err(unsupported(face, format!("{face} of {v} has no surface"))),
                    _ => return Err(unsupported(face, format!("{face} of {v} is a virtual or composite face"))),
                };
                // Outward normal of the volume is the surface normal iff (Left, Forward) or (Right, Reversed).
                let outward_is_surface_normal = match (inc.side, sc.sense) {
                    (Side::Both, _) => return Err(unsupported(face, format!("{face} of {v} is a baffle (the volume lies on both sides)"))),
                    (Side::Left, sm_topo_api::Sense::Forward) | (Side::Right, sm_topo_api::Sense::Reversed) => true,
                    _ => false,
                };
                let sign = if outward_is_surface_normal { Sign::Negative } else { Sign::Positive };
                for x in self.halfspaces(sc.surface, sign, face)? {
                    push(&mut hs, x)?;
                }
            }
        }
        let region = CsgRegion::single(CsgCell::new(hs));
        Ok((CsgCellDef::new(region, name.clone()), CellInfo { volume: v, name, copy: 0 }))
    }

    fn leaf(&mut self, vols: impl IntoIterator<Item = EntityId>) -> Result<Option<ComponentId>, BridgeError> {
        let (mut cells, mut info) = (Vec::new(), Vec::new());
        for v in vols {
            let (c, i) = self.cell_of(v)?;
            cells.push(c);
            info.push(i);
        }
        if cells.is_empty() {
            return Ok(None);
        }
        self.leaves.push(LeafCells { cells, info });
        Ok(Some(self.push(ComponentKind::Leaf { input: self.leaves.len() - 1 })))
    }

    fn proto(&mut self, pid: EntityId) -> Result<Option<ComponentId>, BridgeError> {
        if let Some(c) = self.protos.get(&pid) {
            return Ok(*c);
        }
        let p = self.model.instancing().prototypes.get(&pid).ok_or_else(|| invalid(format!("unknown prototype {pid}")))?;
        let vols: Vec<EntityId> = p.volumes.iter().copied().filter(|v| self.wanted(*v)).collect();
        if let Some(bad) = vols.iter().find(|v| !self.topo.is_alive(**v)) {
            return Err(invalid(format!("prototype {pid} names {bad}, which is not a live volume")));
        }
        let c = self.leaf(vols)?;
        self.protos.insert(pid, c);
        Ok(c)
    }

    fn void(&mut self) -> ComponentId {
        if let Some(v) = self.void {
            return v;
        }
        let v = self.push(ComponentKind::Union { placements: Vec::new() });
        self.void = Some(v);
        v
    }

    fn fill(&mut self, f: Fill, owner: EntityId) -> Result<Option<ComponentId>, BridgeError> {
        self.depth += 1;
        if self.depth > 64 {
            return Err(invalid("instancing nesting too deep"));
        }
        let r = match f {
            Fill::Void => None,
            Fill::Prototype(p) => self.proto(p)?,
            Fill::Lattice(l) => self.lattice(l)?,
            Fill::Cluster(c) => self.cluster(c)?,
            _ => return Err(unsupported(owner, "a fill kind this bridge does not know")),
        };
        self.depth -= 1;
        Ok(r)
    }

    fn lattice(&mut self, lid: EntityId) -> Result<Option<ComponentId>, BridgeError> {
        if let Some(c) = self.lattices.get(&lid) {
            return Ok(*c);
        }
        let model = self.model;
        let lat = model.instancing().lattices.get(&lid).ok_or_else(|| invalid(format!("unknown lattice {lid}")))?;
        let Layout::Cartesian { n, pitch } = lat.layout else {
            return Err(unsupported(lid, format!("{lid} is a hex lattice; only Cartesian layouts are bridged")));
        };
        let pz = if pitch.z > 0.0 { pitch.z } else if n[2] == 1 { PITCH_2D } else { 0.0 };
        if !(pitch.x > 0.0 && pitch.y > 0.0 && pz > 0.0 && pitch.x.is_finite() && pitch.y.is_finite()) {
            return Err(unsupported(lid, format!("{lid} has a non-positive pitch {:?}; the world spec needs positive pitches", [pitch.x, pitch.y, pitch.z])));
        }
        let mut fills = Vec::new();
        for ix in lat.layout.indices() {
            let slot = lat.slots.get(&ix);
            if let Some(s) = slot
                && (!s.recolor.is_empty() || s.clip.is_some())
            {
                return Err(unsupported(lid, format!("{lid} slot ({}, {}, {}) is recolored or clipped", ix.i, ix.j, ix.k)));
            }
            let f = slot.map_or(lat.outer, |s| s.fill);
            let c = self.fill(f, lid)?;
            fills.push(match c {
                Some(c) => c,
                None => self.void(),
            });
        }
        // Slot (0, 0, 0) sits at the lattice origin; world lattices centre elements at `origin + (i + 1/2) pitch`.
        let origin = sm_core::Point3::new(-0.5 * pitch.x, -0.5 * pitch.y, -0.5 * pz);
        let comp = self.push(ComponentKind::Lattice(Lattice::new(LatticeKind::Rect { pitch: [pitch.x, pitch.y, pz] }, origin, n, fills, None)));
        let comp = if lat.origin == Transform::IDENTITY { comp } else { self.push(ComponentKind::Union { placements: vec![Placement::new(comp, lat.origin)] }) };
        self.lattices.insert(lid, Some(comp));
        Ok(Some(comp))
    }

    fn cluster(&mut self, cid: EntityId) -> Result<Option<ComponentId>, BridgeError> {
        if let Some(c) = self.clusters.get(&cid) {
            return Ok(*c);
        }
        let model = self.model;
        let cl = model.instancing().clusters.get(&cid).ok_or_else(|| invalid(format!("unknown cluster {cid}")))?;
        let mut placements = Vec::new();
        for (k, m) in cl.members.iter().enumerate() {
            if !m.recolor.is_empty() || m.clip.is_some() {
                return Err(unsupported(cid, format!("{cid} member {k} is recolored or clipped")));
            }
            if let Some(c) = self.fill(m.fill, cid)? {
                placements.push(Placement::new(c, m.at));
            }
        }
        let c = (!placements.is_empty()).then(|| self.push(ComponentKind::Union { placements }));
        self.clusters.insert(cid, c);
        Ok(c)
    }
}

/// Express the model as CSG leaves and a world graph. See the module docs for the mapping.
///
/// # Errors
/// [`BridgeError::Unsupported`] naming the face, volume, lattice or instance that cannot be
/// represented; [`BridgeError::Invalid`] for an inconsistent model or selection.
pub fn csg_world(model: &Model, topo: &dyn TopologyView, opts: &CsgWorldOptions) -> Result<CsgWorld, BridgeError> {
    let selected = match &opts.bodies {
        None => None,
        Some(list) => {
            if list.is_empty() {
                return Err(invalid("bodies is empty; omit it to include everything"));
            }
            let mut set = BTreeSet::new();
            for &id in list {
                match topo.dim_of(id) {
                    Some(Dim::Body) if topo.is_alive(id) => set.extend(topo.boundary(id).iter().map(|e| e.cell).filter(|c| topo.dim_of(*c) == Some(Dim::Volume))),
                    Some(Dim::Volume) if topo.is_alive(id) => {
                        set.insert(id);
                    }
                    _ => return Err(invalid(format!("{id} is not a live body or volume"))),
                }
            }
            Some(set)
        }
    };
    let inst = model.instancing();
    if let Some(l) = inst.find_cycle() {
        return Err(invalid(format!("instancing cycle through {l}")));
    }
    let mut b = Builder {
        model,
        topo,
        selected,
        store: MemoryStore::new(),
        by_def: BTreeMap::new(),
        comps: Vec::new(),
        leaves: Vec::new(),
        protos: BTreeMap::new(),
        lattices: BTreeMap::new(),
        clusters: BTreeMap::new(),
        void: None,
        depth: 0,
    };
    if !inst.overrides.is_empty() {
        let who = inst.overrides.iter().find_map(|o| match o.prefix.first() {
            Some(sm_model_api::PathStep::Instance(i)) => Some(*i),
            _ => None,
        });
        return Err(unsupported(who, "path overrides (recoloring by instance path) are not bridged"));
    }
    let in_proto: BTreeSet<EntityId> = inst.prototypes.values().flat_map(|p| p.volumes.iter().copied()).collect();
    let root_vols: Vec<EntityId> = topo.cells(Dim::Volume).iter().copied().filter(|v| topo.is_alive(*v) && !in_proto.contains(v) && b.wanted(*v)).collect();
    let root_leaf = b.leaf(root_vols)?;
    let mut placements = Vec::new();
    for (id, row) in &inst.instances {
        if !row.recolor.is_empty() || row.clip.is_some() {
            return Err(unsupported(*id, format!("{id} is recolored or clipped")));
        }
        if let Some(c) = b.fill(row.of, *id)? {
            placements.push(Placement::new(c, row.at));
        }
    }
    let root = match (root_leaf, placements.is_empty()) {
        (None, true) => return Err(invalid("the model has nothing to track (no volumes selected)")),
        (Some(l), true) => l,
        (l, false) => {
            let mut all: Vec<Placement> = l.into_iter().map(|l| Placement::new(l, Transform::IDENTITY)).collect();
            all.extend(placements);
            b.push(ComponentKind::Union { placements: all })
        }
    };
    let spec = WorldSpec::new(b.comps, root, InstancePolicy::Auto);
    spec.validate(b.leaves.len()).map_err(|e| invalid(e.to_string()))?;
    let complement = opts.complement.clone().map_or(ComplementPolicy::Outside, ComplementPolicy::Region);
    Ok(CsgWorld { store: b.store, spec, leaves: b.leaves, complement })
}

struct Flat {
    store: MemoryStore,
    by_def: BTreeMap<String, EntityId>,
    cells: Vec<CsgCellDef>,
    info: Vec<CellInfo>,
    copies: u32,
}

impl CsgWorld {
    /// The world graph (leaf `i` of the spec is [`Self::cells`]`(i)`).
    pub fn spec(&self) -> &WorldSpec {
        &self.spec
    }

    /// Number of CSG leaves (one per used prototype, plus the root leaf).
    pub fn leaf_count(&self) -> usize {
        self.leaves.len()
    }

    /// The cells of leaf `i`.
    pub fn cells(&self, leaf: usize) -> &[CsgCellDef] {
        self.leaves.get(leaf).map_or(&[], |l| &l.cells)
    }

    /// Where each cell of leaf `i` comes from (parallel to [`Self::cells`]).
    pub fn cell_info(&self, leaf: usize) -> &[CellInfo] {
        self.leaves.get(leaf).map_or(&[], |l| &l.info)
    }

    /// The geometry store the leaves' half-spaces name.
    pub fn geom(&self) -> &dyn GeomStore {
        &self.store
    }

    /// True when the world is one bare leaf (the analytic tracker builds from it as is).
    pub fn is_single_leaf(&self) -> bool {
        matches!(self.spec.components.get(self.spec.root.get()), Some(ComponentKind::Leaf { .. }))
    }

    /// The leaf inputs, indexed by `ComponentKind::Leaf { input }`.
    pub fn leaf_inputs(&self) -> Vec<LeafInput<'_>> {
        self.leaves.iter().map(|l| LeafInput::Csg(CsgLeafInput::new(&self.store, &l.cells, None, &self.complement))).collect()
    }

    /// Run `f` over the hierarchical input of this world.
    pub fn with_hier<R>(&self, boundaries: &BoundarySpec, f: impl FnOnce(&HierInput<'_>) -> R) -> R {
        let leaves = self.leaf_inputs();
        let h = HierInput::new(&self.spec, &leaves, boundaries);
        f(&h)
    }

    /// Cells of the flattened world (each placed copy counted), saturating.
    pub fn flat_cell_count(&self) -> u64 {
        fn count(w: &CsgWorld, c: usize, memo: &mut [Option<u64>]) -> u64 {
            if let Some(n) = memo[c] {
                return n;
            }
            let n = match &w.spec.components[c] {
                ComponentKind::Leaf { input } => w.leaves[*input].cells.len() as u64,
                ComponentKind::Union { placements } => placements.iter().fold(0u64, |a, p| a.saturating_add(count(w, p.child.get(), memo))),
                ComponentKind::Lattice(l) => l.fill.iter().fold(0u64, |a, f| a.saturating_add(count(w, f.get(), memo))),
                _ => 0,
            };
            memo[c] = Some(n);
            n
        }
        let mut memo = vec![None; self.spec.components.len()];
        count(self, self.spec.root.get(), &mut memo)
    }

    /// Expand the world into one bare leaf with one cell per placed volume copy: every surface
    /// is moved to its placed position. Identical surfaces are shared. Region names are kept, so
    /// regions are the same as in the world. Cell order is depth-first placement order.
    ///
    /// # Errors
    /// [`BridgeError::Invalid`] when the result would exceed `max_cells` or a surface cannot be
    /// moved (a non-uniformly scaled torus).
    pub fn flatten(&self, max_cells: usize) -> Result<CsgWorld, BridgeError> {
        let n = self.flat_cell_count();
        if n > max_cells as u64 {
            return Err(invalid(format!("the flattened world would have {n} cells, over the limit {max_cells}")));
        }
        let mut out = Flat { store: MemoryStore::new(), by_def: BTreeMap::new(), cells: Vec::new(), info: Vec::new(), copies: 0 };
        self.walk(self.spec.root.get(), &Transform::IDENTITY, &mut out)?;
        let spec = WorldSpec::new(vec![ComponentKind::Leaf { input: 0 }], ComponentId(0), InstancePolicy::Flatten);
        Ok(CsgWorld { store: out.store, spec, leaves: vec![LeafCells { cells: out.cells, info: out.info }], complement: self.complement.clone() })
    }

    fn walk(&self, c: usize, t: &Transform, out: &mut Flat) -> Result<(), BridgeError> {
        match &self.spec.components[c] {
            ComponentKind::Leaf { input } => self.place_leaf(*input, t, out),
            ComponentKind::Union { placements } => {
                for p in placements {
                    self.walk(p.child.get(), &t.compose(&p.transform), out)?;
                }
                Ok(())
            }
            ComponentKind::Lattice(l) => {
                let [nx, ny, nz] = l.dims;
                let mut k = 0usize;
                for iz in 0..nz as i32 {
                    for iy in 0..ny as i32 {
                        for ix in 0..nx as i32 {
                            let child = l.fill[k];
                            k += 1;
                            self.walk(child.get(), &t.compose(&l.element_transform([ix, iy, iz])), out)?;
                        }
                    }
                }
                Ok(())
            }
            _ => Err(invalid("a component kind this bridge does not know")),
        }
    }

    fn place_leaf(&self, input: usize, t: &Transform, out: &mut Flat) -> Result<(), BridgeError> {
        let lf = &self.leaves[input];
        let copy = out.copies;
        out.copies += 1;
        let mut remap: BTreeMap<EntityId, (EntityId, bool)> = BTreeMap::new();
        for (cell, info) in lf.cells.iter().zip(&lf.info) {
            let mut terms = Vec::new();
            for term in cell.region.terms() {
                let mut hs = Vec::with_capacity(term.halfspaces.len());
                for h in &term.halfspaces {
                    let (id, flip) = match remap.get(&h.surface) {
                        Some(x) => *x,
                        None => {
                            let def = self.store.surface_def(h.surface).ok_or_else(|| invalid("a half-space names a surface that is not in the store"))?;
                            let (moved, sense) = def.transformed(t).map_err(|e| invalid(format!("{}: cannot place the surface: {e}", info.volume)))?;
                            let id = add_def(&mut out.store, &mut out.by_def, moved)?;
                            let x = (id, sense == Sense::Reversed);
                            remap.insert(h.surface, x);
                            x
                        }
                    };
                    hs.push(HalfSpace { surface: id, sign: if flip { opposite(h.sign) } else { h.sign } });
                }
                terms.push(CsgCell::new(hs));
            }
            let mut d = CsgCellDef::new(CsgRegion::new(terms), cell.name.clone());
            if let Some(v) = cell.volume {
                d = d.with_volume(v * t.det().abs());
            }
            out.cells.push(d);
            out.info.push(CellInfo { copy, ..info.clone() });
        }
        Ok(())
    }

    /// Build the analytic tracker of this world, flattening it first unless it is one bare leaf.
    ///
    /// # Errors
    /// The flatten errors, and whatever the analytic builder reports (`csg.unbounded`, ...).
    pub fn analytic_tracker(&self, boundaries: &BoundarySpec, opts: &BuildOptions, max_cells: usize) -> Result<(AnalyticTracker, BuildReport), RtError> {
        let flat;
        let w = if self.is_single_leaf() {
            self
        } else {
            flat = self.flatten(max_cells)?;
            &flat
        };
        w.with_hier(boundaries, |h| AnalyticTrackerBuilder.build_concrete(&BuildInput::Hier(*h), opts))
    }
}

impl CsgWorld {
    /// Build the hierarchical tracker of this world: the lattices and unions are walked as they
    /// are and each prototype leaf is one analytic tracker, so nothing is flattened (the cost
    /// does not grow with the number of copies). Tracking results equal those of
    /// [`Self::analytic_tracker`] up to the tolerance; that one stays as the oracle.
    ///
    /// What lies outside the cells is the model boundary (a world has no complement region), so
    /// the world must have been bridged without [`CsgWorldOptions::complement`].
    ///
    /// # Errors
    /// `RtError::Unsupported` for a complement region or a hex lattice; the world builder's errors
    /// otherwise (`world.lattice_fit` when a prototype does not fit its lattice element, ...).
    pub fn world_tracker(&self, boundaries: &BoundarySpec, opts: &BuildOptions) -> Result<(WorldTracker, BuildReport), RtError> {
        if matches!(self.complement, ComplementPolicy::Region(_)) {
            return Err(RtError::Unsupported("a hierarchical CSG world has no complement region; bridge without `complement` or use `analytic_tracker`"));
        }
        self.with_hier(boundaries, |h| MeshTrackerBuilder.build_world(&BuildInput::Hier(*h), opts))
    }
}

/// Which leaves a sampled check ran over.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum CheckScope {
    /// The whole world expanded to one leaf: overlaps between placed copies are found.
    Flattened,
    /// Each prototype leaf alone (the flattened world was over the cell limit).
    Prototypes,
}

/// One finding of [`check_world_overlaps`].
#[derive(Clone, Debug, PartialEq)]
pub struct WorldFinding {
    /// The finding (`csg.overlap`, `csg.gap`, `csg.unbounded`, ...), located.
    pub issue: BuildIssue,
    /// The cells it names (two for an overlap), as volumes and names.
    pub cells: Vec<CellInfo>,
}

/// The sampled overlap and gap check over a bridged world ([`crate::check_overlaps`] per leaf).
/// A world that places or tiles leaves is flattened first when it has at most `max_cells`
/// cells, so overlaps between copies are found; otherwise the prototype leaves are checked alone
/// (reported in the scope).
///
/// # Errors
/// [`BridgeError::Invalid`] when flattening fails for another reason than the cell limit.
pub fn check_world_overlaps(world: &CsgWorld, n_points: usize, n_rays: usize, seed: u64, max_cells: usize) -> Result<(CheckScope, Vec<WorldFinding>), BridgeError> {
    let flat;
    let (w, scope) = if world.is_single_leaf() || world.flat_cell_count() > max_cells as u64 {
        (world, if world.is_single_leaf() { CheckScope::Flattened } else { CheckScope::Prototypes })
    } else {
        flat = world.flatten(max_cells)?;
        (&flat, CheckScope::Flattened)
    };
    let bc = BoundarySpec::default();
    let found = w.with_hier(&bc, |h| check_located(h, n_points, n_rays, seed));
    let out = found
        .into_iter()
        .map(|f| {
            let info = w.cell_info(f.leaf);
            WorldFinding { cells: f.cells.iter().filter_map(|c| info.get(*c as usize).cloned()).collect(), issue: f.issue }
        })
        .collect();
    Ok((scope, out))
}
