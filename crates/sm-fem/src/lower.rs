//! Lowering of blocks, sidesets and decks to mesh sets; see [`lower`](crate::lower).

use std::collections::BTreeMap;
use std::sync::Arc;

use sm_core::{EntityId, EntityKind};
use sm_layer_api::{MeshEntry, MeshHost, ModelLayer};
use sm_mesh_api::{CellIx, FaceIx, MeshRead, SerialAlloc, SetMembers, Side, SignedFace};
use sm_meshio::DeckRecord;
use sm_model_api::{BoundaryKind, DeckSide, Domain, FaceRole};
use sm_topo_api::TopologyView;

use crate::element::ElementType;
use crate::error::FemError;
use crate::layer::FemLayer;
use crate::region::{
    PlanItem, REGION_SET_PREFIX, RegionError, RegionSplit, RegionSummary, block_name_ok,
    plan_regions,
};

/// Where a lowered set came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum SetOrigin {
    /// A volume color (block).
    Block,
    /// A face color (sideset).
    Sideset,
    /// One side of a deck boundary.
    DeckFace,
    /// A region of the export split (`region/<name>`, see [`crate::region`]).
    Region,
}

/// A summary of one set added by [`lower`].
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct SetSummary {
    /// Set name.
    pub name: String,
    /// What produced it.
    pub origin: SetOrigin,
    /// Integer label (the color label), if any.
    pub label: Option<i64>,
    /// Number of members (sides for a sideset).
    pub len: usize,
}

/// A mesh with the model's blocks, sidesets and decks lowered into sets.
#[derive(Clone)]
#[non_exhaustive]
pub struct Lowered {
    /// The mesh to write. This is the entry's own mesh (shared, not copied) when nothing was
    /// added, otherwise a copy with the new sets.
    pub mesh: Arc<dyn MeshHost>,
    /// Deck metadata for `.smsh`, ascending by boundary id.
    pub decks: Vec<DeckRecord>,
    /// The sets added, in the order they were added.
    pub sets: Vec<SetSummary>,
    /// Declared element type per emitted block set (by set name); blocks with none are absent.
    pub block_types: BTreeMap<String, ElementType>,
    /// The regions of a split lowering ([`lower_split`]), in name order; empty otherwise. Each
    /// has a set in `sets` with origin [`SetOrigin::Region`].
    pub regions: Vec<RegionSummary>,
}

impl Lowered {
    /// The identity lowering: the entry's mesh as it is.
    pub fn identity(mesh: &MeshEntry) -> Self {
        Self {
            mesh: Arc::clone(&mesh.mesh),
            decks: Vec::new(),
            sets: Vec::new(),
            block_types: BTreeMap::new(),
            regions: Vec::new(),
        }
    }
}

struct Pending {
    name: String,
    members: SetMembers,
    label: Option<i64>,
    origin: SetOrigin,
    ty: Option<ElementType>,
}

/// Lower the blocks, sidesets and decks of `model` onto a copy of `mesh`.
///
/// The lowering is export-time only. It clones the mesh, adds sets to the clone with ids minted
/// from a clone of the layer allocator, and leaves the `meshes` layer untouched, so the same
/// `(state, mesh)` always lowers to the same sets with the same ids.
///
/// | Model data | Mesh result |
/// |---|---|
/// | volume color (block) `c` | `SetMembers::Cells` named like `c`, label `c.label`: the cells classified on the volumes colored `c` |
/// | face color (sideset) `c` | `SetMembers::Sides` named like `c`, label `c.label`: for each colored `TopoFace`, the sides of the mesh faces classified on it |
/// | deck boundary `b` | `SetMembers::Faces` `deck_<b>_a` (side A, or the self-paired mirror faces) and `deck_<b>_b` (side B), plus a [`DeckRecord`] naming them |
///
/// Blocks, sidesets and decks are all lowered from the **model** tables, whether or not the state
/// has a `fem` layer: the layer only adds declared element types and `wrt` preferences. So a
/// state without it still exports its volume colors, face colors and deck boundaries as sets,
/// and exports no sets only when the model holds none.
///
/// Colors and deck faces are found through the mesh classification, including overlay pieces: a
/// color on a hidden host face or volume reaches the mesh faces and cells classified on the live
/// `PartitionFace`/`PartitionCell` pieces that cover it.
///
/// A sideset face takes the side of the volume it was added `wrt`; without one, a boundary face
/// gives its only (`Left`) side and an interior face gives both sides. Empty sets are not
/// emitted. Colors are lowered in ascending id order, so output order never depends on
/// insertion order.
///
/// Set ids are minted from a clone of `alloc` (pass `MeshLayer::alloc()`), so nothing is
/// allocated in the state.
///
/// # Errors
/// - [`FemError::ElementMismatch`] when a block's declared element type does not match the shape
///   of its cells (only declared types are checked);
/// - [`FemError::Unsupported`] when a sideset `wrt` volume is gone and covered by no live cell;
/// - [`FemError::Mesh`] when the mesh already has a set of the same name as a color or deck;
/// - [`FemError::Meshio`] for a deck the file format cannot describe.
pub fn lower(
    model: &ModelLayer,
    mesh: &MeshEntry,
    fem: &FemLayer,
    alloc: &SerialAlloc,
) -> Result<Lowered, FemError> {
    let m = mesh.mesh();
    let mut pending: Vec<Pending> = Vec::new();
    blocks(model, m, fem, &mut pending)?;
    sidesets(model, m, fem, &mut pending)?;
    let decks = deck_sets(model, m, &mut pending);
    assemble(mesh, alloc, pending, decks, Vec::new())
}

/// Errors of [`lower_split`]: the lowering's own, or the region split's.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum SplitError {
    /// [`lower`] failed.
    #[error(transparent)]
    Lower(#[from] FemError),
    /// The split cannot be made.
    #[error(transparent)]
    Region(#[from] RegionError),
}

/// [`lower`], then one cell set `region/<name>` per region of `split` (see [`crate::region`]).
///
/// The block, sideset and deck sets are exactly those of [`lower`]; the region sets follow them,
/// ordered by name. An empty `split` is [`lower`] itself. Every cell must be in a block, so each
/// region has a block to be named after: cells of a volume without a tag in a requested part
/// simply have no such part in the name.
///
/// # Errors
/// [`SplitError::Region`] with [`RegionError::NeedBlocks`] when the blocks do not cover the mesh,
/// [`RegionError::NameReserved`] for a block name with `@`, `/`, `[`, `]` or `,`,
/// [`RegionError::Ambiguous`], [`RegionError::Instances`] (the model has instances); otherwise the errors of [`lower`].
pub fn lower_split(
    model: &ModelLayer,
    mesh: &MeshEntry,
    fem: &FemLayer,
    alloc: &SerialAlloc,
    split: &RegionSplit,
) -> Result<Lowered, SplitError> {
    if split.is_empty() {
        return Ok(lower(model, mesh, fem, alloc)?);
    }
    if !model.model().instancing().instances.is_empty() {
        return Err(RegionError::Instances.into());
    }
    let m = mesh.mesh();
    let mut pending: Vec<Pending> = Vec::new();
    blocks(model, m, fem, &mut pending)?;
    sidesets(model, m, fem, &mut pending)?;
    let decks = deck_sets(model, m, &mut pending);

    let block_sets: Vec<&Pending> = pending
        .iter()
        .filter(|p| p.origin == SetOrigin::Block)
        .collect();
    if block_sets.is_empty() {
        return Err(
            RegionError::NeedBlocks("the model has no blocks (fem.block_add)".into()).into(),
        );
    }
    if let Some(p) = block_sets.iter().find(|p| !block_name_ok(&p.name)) {
        return Err(RegionError::NameReserved(p.name.clone()).into());
    }
    // Each cell has one volume with one color, so a cell is in at most one block set: the
    // blocks cover the mesh exactly when their sizes sum to the cell count.
    let covered: usize = block_sets.iter().map(|p| p.members.len()).sum();
    let total = m.cell_count();
    if covered != total {
        let why = format!(
            "{} of {total} cells are in no block",
            total - covered.min(total)
        );
        return Err(RegionError::NeedBlocks(why).into());
    }
    let (md, topo) = (model.model(), model.topo());
    // Tag of a live volume or overlay piece: its own, else that of a stored host volume whose
    // cover contains it (the lowest host id), as `live_volume_color` does for colors.
    let mut inherited: BTreeMap<EntityId, &sm_model_api::RegionTag> = BTreeMap::new();
    for (k, t) in md.region_tags() {
        if !topo.is_alive(*k) {
            for x in topo.live_cover(*k) {
                inherited.entry(x).or_insert(t);
            }
        }
    }
    let tag_of = |ent: EntityId| md.region_tag(ent).or_else(|| inherited.get(&ent).copied());
    let mut groups: Vec<(&str, Option<&sm_model_api::RegionTag>, Vec<CellIx>)> = Vec::new();
    for p in &block_sets {
        let SetMembers::Cells(cells) = &p.members else {
            continue;
        };
        let mut by_ent: BTreeMap<EntityId, Vec<CellIx>> = BTreeMap::new();
        for &c in cells {
            by_ent.entry(m.cell_class(c).entity).or_default().push(c);
        }
        for (ent, cs) in by_ent {
            groups.push((p.name.as_str(), tag_of(ent), cs));
        }
    }
    let plan = plan_regions(
        split,
        md.lattice_frames(),
        groups.iter().map(|(b, t, c)| PlanItem {
            block: b,
            tag: *t,
            cells: c,
        }),
    )?;
    let mut summaries = Vec::with_capacity(plan.regions.len());
    for r in plan.regions {
        let name = r.name.format();
        let set = format!("{REGION_SET_PREFIX}{name}");
        let len = r.cells.len();
        pending.push(Pending {
            name: set.clone(),
            members: SetMembers::Cells(r.cells),
            label: None,
            origin: SetOrigin::Region,
            ty: None,
        });
        summaries.push(RegionSummary {
            name,
            set,
            tags: r.tags,
            len,
        });
    }
    Ok(assemble(mesh, alloc, pending, decks, summaries)?)
}

fn assemble(
    mesh: &MeshEntry,
    alloc: &SerialAlloc,
    pending: Vec<Pending>,
    decks: Vec<DeckPlan>,
    regions: Vec<RegionSummary>,
) -> Result<Lowered, FemError> {
    if pending.is_empty() {
        return Ok(Lowered::identity(mesh));
    }
    let mut host = (*mesh.mesh).clone_host();
    let mut alloc = alloc.clone();
    let mut sets = Vec::with_capacity(pending.len());
    let mut block_types = BTreeMap::new();
    for p in pending {
        if let Some(ty) = p.ty {
            block_types.insert(p.name.clone(), ty);
        }
        let len = p.members.len();
        host.add_set(&p.name, p.members, &mut alloc)?;
        if let Some(l) = p.label {
            host.set_label(&p.name, Some(l))?;
        }
        sets.push(SetSummary {
            name: p.name,
            origin: p.origin,
            label: p.label,
            len,
        });
    }
    let mut records = Vec::with_capacity(decks.len());
    for d in decks {
        records.push(DeckRecord::from_deck(d.name, &d.deck, d.a, d.b)?);
    }
    Ok(Lowered {
        mesh: Arc::from(host),
        decks: records,
        sets,
        block_types,
        regions,
    })
}

fn blocks(
    model: &ModelLayer,
    m: &dyn MeshRead,
    fem: &FemLayer,
    out: &mut Vec<Pending>,
) -> Result<(), FemError> {
    let (md, topo) = (model.model(), model.topo());
    let mut buckets: BTreeMap<EntityId, Vec<CellIx>> = BTreeMap::new();
    for &ent in m.classified_entities() {
        if !matches!(
            ent.kind(),
            EntityKind::TopoVolume | EntityKind::PartitionCell
        ) {
            continue;
        }
        let Some(set) = m.classified_on(ent) else {
            continue;
        };
        if set.cells.is_empty() {
            continue;
        }
        if let Some(c) = md.live_volume_color(ent, topo) {
            buckets
                .entry(c)
                .or_default()
                .extend(set.cells.iter().copied());
        }
    }
    for (color, mut cells) in buckets {
        let Some(row) = md.color(color).filter(|r| r.domain == Domain::Volume) else {
            continue;
        };
        cells.sort_unstable();
        cells.dedup();
        let declared = fem.element_type(color);
        if let Some(ty) = declared {
            if !ty.is_linear() {
                return Err(FemError::Unsupported(format!("element type {ty}")));
            }
            let want = ty.shape();
            let bad = |c: &CellIx| m.cell_shape(*c) != want;
            if let Some(i) = cells.iter().position(bad) {
                return Err(FemError::ElementMismatch {
                    block: row.name.clone(),
                    declared: ty,
                    cell: cells[i].0,
                    found: m.cell_shape(cells[i]),
                });
            }
        }
        out.push(Pending {
            name: row.name.clone(),
            members: SetMembers::Cells(cells),
            label: Some(i64::from(row.label)),
            origin: SetOrigin::Block,
            ty: declared,
        });
    }
    Ok(())
}

fn sidesets(
    model: &ModelLayer,
    m: &dyn MeshRead,
    fem: &FemLayer,
    out: &mut Vec<Pending>,
) -> Result<(), FemError> {
    let (md, topo) = (model.model(), model.topo());
    // Live piece -> hidden host face carrying memberships (lowest host wins, as in
    // `live_face_color`).
    let mut host_of: BTreeMap<EntityId, EntityId> = BTreeMap::new();
    // Face -> the sideset colors `fem.sideset_add` put it in (the model keeps only one color).
    let mut extra: BTreeMap<EntityId, Vec<EntityId>> = BTreeMap::new();
    let mut last = None;
    for (face, color, _) in fem.memberships() {
        extra.entry(face).or_default().push(color);
        if last.replace(face) != Some(face) && !topo.is_alive(face) {
            for x in topo.live_cover(face) {
                host_of.entry(x).or_insert(face);
            }
        }
    }
    let mut by_color: BTreeMap<EntityId, Vec<EntityId>> = BTreeMap::new();
    for &ent in m.classified_entities() {
        if !matches!(ent.kind(), EntityKind::TopoFace | EntityKind::PartitionFace) {
            continue;
        }
        let mut colors: Vec<EntityId> = md.live_face_color(ent, topo).into_iter().collect();
        for src in [Some(ent), host_of.get(&ent).copied()]
            .into_iter()
            .flatten()
        {
            if let Some(cs) = extra.get(&src) {
                colors.extend_from_slice(cs);
            }
        }
        colors.sort_unstable();
        colors.dedup();
        for c in colors {
            by_color.entry(c).or_default().push(ent);
        }
    }
    for (color, ents) in by_color {
        let Some(row) = md.color(color).filter(|r| r.domain == Domain::Face) else {
            continue;
        };
        let mut sides: Vec<SignedFace> = Vec::new();
        for ent in ents {
            let Some(set) = m.classified_on(ent) else {
                continue;
            };
            let mut prefs = fem.side_wrt(ent, color);
            if prefs.is_empty()
                && let Some(h) = host_of.get(&ent)
            {
                prefs = fem.side_wrt(*h, color);
            }
            let wrt = wrt_cells(topo, ent, prefs)?;
            for &f in &set.faces {
                push_sides(m, f, wrt.as_deref(), &mut sides);
            }
        }
        if sides.is_empty() {
            continue;
        }
        sides.sort_unstable_by_key(|s| s.to_raw());
        sides.dedup();
        out.push(Pending {
            name: row.name.clone(),
            members: SetMembers::Sides(sides),
            label: Some(i64::from(row.label)),
            origin: SetOrigin::Sideset,
            ty: None,
        });
    }
    Ok(())
}

/// The model entities a cell may be classified on to count as one of the `wrt` volumes: the
/// volume itself, or the live pieces covering it when it is a hidden overlay host. Sorted;
/// `None` when no preference was given.
fn wrt_cells(
    topo: &dyn TopologyView,
    face: EntityId,
    wrt: &[EntityId],
) -> Result<Option<Vec<EntityId>>, FemError> {
    if wrt.is_empty() {
        return Ok(None);
    }
    let mut ents = Vec::with_capacity(wrt.len());
    for &v in wrt {
        if topo.is_alive(v) {
            ents.push(v);
            continue;
        }
        let cover = topo.live_cover(v);
        if cover.is_empty() {
            return Err(FemError::Unsupported(format!(
                "sideset face {face} is taken wrt {v}, which has no live cells"
            )));
        }
        ents.extend(cover);
    }
    ents.sort_unstable();
    ents.dedup();
    Ok(Some(ents))
}

fn push_sides(m: &dyn MeshRead, f: FaceIx, wrt: Option<&[EntityId]>, out: &mut Vec<SignedFace>) {
    let (left, right) = (m.face_left(f), m.face_right(f));
    match wrt {
        Some(v) => {
            let is_wrt = |c: CellIx| v.binary_search(&m.cell_class(c).entity).is_ok();
            if is_wrt(left) {
                out.push(SignedFace::new(f, Side::Left));
            }
            if right.is_some_and(is_wrt) {
                out.push(SignedFace::new(f, Side::Right));
            }
        }
        None => {
            out.push(SignedFace::new(f, Side::Left));
            if right.is_some() {
                out.push(SignedFace::new(f, Side::Right));
            }
        }
    }
}

struct DeckPlan {
    name: String,
    deck: sm_model_api::Deck,
    a: Option<String>,
    b: Option<String>,
}

fn deck_sets(model: &ModelLayer, m: &dyn MeshRead, out: &mut Vec<Pending>) -> Vec<DeckPlan> {
    let (md, topo) = (model.model(), model.topo());
    let mut plans = Vec::new();
    for row in md.boundaries() {
        let BoundaryKind::Deck(deck) = row.kind else {
            continue;
        };
        let (mut a, mut b): (Vec<FaceIx>, Vec<FaceIx>) = (Vec::new(), Vec::new());
        for (face, fb) in md.boundary_faces(row.id) {
            let FaceRole::Deck { side, .. } = fb.role else {
                continue;
            };
            // A hidden host face reaches the mesh through the pieces that cover it.
            let own = m.classified_on(face);
            let cover = if own.is_none() && !topo.is_alive(face) {
                topo.live_cover(face)
            } else {
                Vec::new()
            };
            let target = match side {
                DeckSide::A | DeckSide::SelfPaired => &mut a,
                DeckSide::B => &mut b,
            };
            for set in own
                .into_iter()
                .chain(cover.iter().filter_map(|x| m.classified_on(*x)))
            {
                target.extend(set.faces.iter().copied());
            }
        }
        if a.is_empty() && b.is_empty() {
            continue;
        }
        let mut add = |suffix: &str, mut faces: Vec<FaceIx>| -> Option<String> {
            if faces.is_empty() {
                return None;
            }
            faces.sort_unstable();
            faces.dedup();
            let name = format!("deck_{}_{suffix}", row.name);
            out.push(Pending {
                name: name.clone(),
                members: SetMembers::Faces(faces),
                label: None,
                origin: SetOrigin::DeckFace,
                ty: None,
            });
            Some(name)
        };
        let (a, b) = (add("a", a), add("b", b));
        plans.push(DeckPlan {
            name: row.name.clone(),
            deck,
            a,
            b,
        });
    }
    plans
}
