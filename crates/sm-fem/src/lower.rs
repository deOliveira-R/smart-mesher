//! Lowering of blocks, sidesets and decks to mesh sets; see [`lower`](crate::lower).

use std::collections::BTreeMap;
use std::sync::Arc;

use rayon::prelude::*;
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{MeshEntry, MeshHost, ModelLayer};
use sm_mesh_api::{CellIx, FaceIx, MeshRead, SerialAlloc, SetMembers, Side, SignedFace};
use sm_meshio::DeckRecord;
use sm_model_api::{BoundaryKind, DeckSide, Domain, FaceRole};

use crate::error::FemError;
use crate::layer::FemLayer;

/// Below this many cells the shape check runs sequentially.
const PAR_MIN: usize = 1 << 14;

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
}

impl Lowered {
    /// The identity lowering: the entry's mesh as it is.
    pub fn identity(mesh: &MeshEntry) -> Self {
        Self {
            mesh: Arc::clone(&mesh.mesh),
            decks: Vec::new(),
            sets: Vec::new(),
        }
    }
}

struct Pending {
    name: String,
    members: SetMembers,
    label: Option<i64>,
    origin: SetOrigin,
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
    sidesets(model, m, fem, &mut pending);
    let decks = deck_sets(model, m, &mut pending);

    if pending.is_empty() {
        return Ok(Lowered::identity(mesh));
    }
    let mut host = (*mesh.mesh).clone_host();
    let mut alloc = alloc.clone();
    let mut sets = Vec::with_capacity(pending.len());
    for p in pending {
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
        if let Some(ty) = fem.element_type(color) {
            if !ty.is_linear() {
                return Err(FemError::Unsupported(format!("element type {ty}")));
            }
            let want = ty.shape();
            let bad = |c: &CellIx| m.cell_shape(*c) != want;
            let at = if cells.len() >= PAR_MIN {
                cells.par_iter().position_first(bad)
            } else {
                cells.iter().position(bad)
            };
            if let Some(i) = at {
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
        });
    }
    Ok(())
}

fn sidesets(model: &ModelLayer, m: &dyn MeshRead, fem: &FemLayer, out: &mut Vec<Pending>) {
    let md = model.model();
    let mut by_color: BTreeMap<EntityId, Vec<EntityId>> = BTreeMap::new();
    for (face, color) in md.face_coloring() {
        by_color.entry(color).or_default().push(face);
    }
    for (color, faces) in by_color {
        let Some(row) = md.color(color).filter(|r| r.domain == Domain::Face) else {
            continue;
        };
        let mut sides: Vec<SignedFace> = Vec::new();
        for face in faces {
            let Some(set) = m.classified_on(face) else {
                continue;
            };
            let wrt = fem.side_wrt(face);
            for &f in &set.faces {
                push_sides(m, f, wrt, &mut sides);
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
        });
    }
}

fn push_sides(m: &dyn MeshRead, f: FaceIx, wrt: Option<EntityId>, out: &mut Vec<SignedFace>) {
    let (left, right) = (m.face_left(f), m.face_right(f));
    match wrt {
        Some(v) => {
            if m.cell_class(left).entity == v {
                out.push(SignedFace::new(f, Side::Left));
            }
            if right.is_some_and(|r| m.cell_class(r).entity == v) {
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
    let md = model.model();
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
            let Some(set) = m.classified_on(face) else {
                continue;
            };
            match side {
                DeckSide::A | DeckSide::SelfPaired => a.extend(set.faces.iter().copied()),
                DeckSide::B => b.extend(set.faces.iter().copied()),
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
