//! Export selection: the `select` object shared by every set-writing exporter.
//!
//! [`apply`] cuts a mesh down to the cells of the selected blocks and keeps the sidesets and
//! nodesets that were asked for. The result is an ordinary mesh that any writer takes, so the
//! selection never touches a writer.
//!
//! * The kept cells stay in their original order and the kept vertices are those the kept cells
//!   use, in their original order. Faces are the faces of the kept cells; a face whose other cell
//!   was dropped becomes a boundary face.
//! * Every other set (a "companion": a region set, a deck face set, any set that is not a block,
//!   sideset or nodeset of the selection) is restricted to the kept entities, and dropped when
//!   nothing is left, so a selected subset exports only the regions it contains and the names do
//!   not change.
//! * A sideset or nodeset that was named but ends up empty is reported as
//!   [`EMPTY_SELECTION`]; an unknown name is [`SelectError::UnknownSet`], which lists the names
//!   there are.
//! * The work and the allocation are proportional to what is kept (sorted lists and binary
//!   search, no table sized to the source mesh).
//!
//! Ids are minted afresh (a new mesh id and dense ids), so a selected mesh is a new mesh: level
//! maps of the source do not apply to it.

use std::collections::BTreeSet;

use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh::{Mesh, MeshBuilder};
use sm_mesh_api::{
    CellIx, Classification, FaceIx, FaceKind, MeshBuild, MeshDim, MeshEntityKind, MeshEntityRef,
    MeshError, MeshRead, MeshSet, SerialAlloc, SetMembers, Side, SignedFace, TagValues, VertexIx,
};

use crate::report::{LossItem, LossReport, LossSeverity};

/// Loss code of a named sideset or nodeset (or block) that holds nothing after the selection.
pub const EMPTY_SELECTION: &str = "export.empty_selection";
/// Loss code for edge tags that a selection cannot carry.
pub const EDGE_TAGS_DROPPED: &str = "export.select_edge_tags_dropped";

/// Which sets of one role to keep.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub enum Pick {
    /// Every set of the role.
    #[default]
    All,
    /// Exactly these sets (an empty list keeps none).
    Names(Vec<String>),
}

/// The spatial dimension an export is asked to have.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Dimension {
    /// The dimension of the mesh.
    #[default]
    Auto,
    /// A dimension-2 (surface) mesh; refused for any other.
    Two,
    /// A dimension-3 (volume) mesh; refused for any other.
    Three,
}

/// What to export: the `select` object.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Select {
    /// Blocks (cell sets) whose cells are written.
    pub blocks: Pick,
    /// Sidesets (face and side sets) written.
    pub sidesets: Pick,
    /// Nodesets (vertex and point sets) written.
    pub nodesets: Pick,
    /// The dimension the mesh must have.
    pub dimension: Dimension,
}

impl Select {
    /// True when nothing is selected away: the default selection, which leaves the mesh as it is.
    pub fn is_all(&self) -> bool {
        *self == Self::default()
    }
}

/// Which sets are not blocks, sidesets or nodesets of the selection.
///
/// By default a set's role follows its members: cell sets are blocks, face and side sets are
/// sidesets, vertex and point sets are nodesets. Names listed as companions are restricted to
/// the kept entities instead (and dropped when empty), whatever their members.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Roles {
    companions: BTreeSet<String>,
}

impl Roles {
    /// Roles by members only.
    pub fn by_kind() -> Self {
        Self::default()
    }

    /// Treat the set `name` as a companion.
    #[must_use]
    pub fn companion(mut self, name: impl Into<String>) -> Self {
        self.companions.insert(name.into());
        self
    }
}

/// Why a selection failed.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum SelectError {
    /// A named set is not in the mesh (with that role).
    #[error("unknown {role} `{name}` (available: {})", .available.join(", "))]
    UnknownSet {
        /// `block`, `sideset` or `nodeset`.
        role: &'static str,
        /// The name asked for.
        name: String,
        /// The names there are, in mesh order.
        available: Vec<String>,
    },
    /// The mesh has another dimension than asked.
    #[error("`dimension` {asked} does not match the mesh (dimension {mesh})")]
    Dimension {
        /// The dimension asked for.
        asked: u8,
        /// The dimension of the mesh.
        mesh: u8,
    },
    /// The selection holds no cell.
    #[error("the selection holds no cell")]
    Empty,
    /// The mesh has data a selection cannot carry.
    #[error("select does not support {0}")]
    Unsupported(&'static str),
    /// The selected mesh could not be built (for example a cut that no model face explains).
    #[error("the selected mesh is not valid: {0}")]
    Build(#[from] MeshError),
}

/// A selected mesh with what the selection could not do.
#[derive(Debug)]
pub struct Selected {
    /// The selected mesh.
    pub mesh: Mesh,
    /// Named sets that came out empty (`export.empty_selection`) and other losses.
    pub report: LossReport,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Role {
    Block,
    Sideset,
    Nodeset,
    Companion,
}

impl Role {
    const fn name(self) -> &'static str {
        match self {
            Self::Block => "block",
            Self::Sideset => "sideset",
            Self::Nodeset => "nodeset",
            Self::Companion => "set",
        }
    }
}

fn role_of(s: &MeshSet, roles: &Roles) -> Role {
    if roles.companions.contains(&s.name) {
        return Role::Companion;
    }
    match s.members {
        SetMembers::Cells(_) => Role::Block,
        SetMembers::Faces(_) | SetMembers::Sides(_) | SetMembers::ShellSides(_) => Role::Sideset,
        SetMembers::Vertices(_) | SetMembers::Points(_) => Role::Nodeset,
        _ => Role::Companion,
    }
}

/// Position of `x` in the ascending list `list`.
fn rank(list: &[u32], x: u32) -> Option<u32> {
    list.binary_search(&x).ok().map(|i| i as u32)
}

/// Position of an entity that must be in `list` (the lists are built from the same cells).
fn must_rank(list: &[u32], x: u32) -> Result<u32, MeshError> {
    rank(list, x).ok_or_else(|| MeshError::State("selection lists disagree".into()))
}

/// Ascending union of already sorted-or-not lists, without duplicates.
fn sorted_unique(mut v: Vec<u32>) -> Vec<u32> {
    v.sort_unstable();
    v.dedup();
    v
}

/// The members of one set restricted to the kept entities, remapped to the new indices.
struct Keep<'a> {
    src: &'a dyn MeshRead,
    cells: &'a [u32],
    faces: &'a [u32],
    verts: &'a [u32],
}

impl Keep<'_> {
    fn restrict(&self, m: &SetMembers) -> SetMembers {
        match m {
            SetMembers::Cells(v) => SetMembers::Cells(
                v.iter().filter_map(|c| rank(self.cells, c.0)).map(CellIx).collect(),
            ),
            SetMembers::Faces(v) => SetMembers::Faces(
                v.iter().filter_map(|f| rank(self.faces, f.0)).map(FaceIx).collect(),
            ),
            SetMembers::Vertices(v) => SetMembers::Vertices(
                v.iter().filter_map(|x| rank(self.verts, x.0)).map(VertexIx).collect(),
            ),
            SetMembers::Sides(v) => SetMembers::Sides(
                v.iter()
                    .filter_map(|sf| {
                        let f = sf.face();
                        let cell = match sf.side() {
                            Side::Left => Some(self.src.face_left(f)),
                            Side::Right => self.src.face_right(f),
                        }?;
                        rank(self.cells, cell.0)?;
                        Some(SignedFace::new(FaceIx(rank(self.faces, f.0)?), sf.side()))
                    })
                    .collect(),
            ),
            SetMembers::ShellSides(v) => SetMembers::ShellSides(
                v.iter().filter_map(|w| Some((rank(self.cells, w >> 1)? << 1) | (w & 1))).collect(),
            ),
            SetMembers::Points(v) => SetMembers::Points(
                v.iter()
                    .filter(|p| p.0 < self.src.vertex_count() as u32)
                    .filter_map(|p| rank(self.verts, p.0))
                    .map(sm_mesh_api::PointIx)
                    .collect(),
            ),
            // Other member kinds do not exist today; an unknown one keeps nothing.
            _ => SetMembers::Cells(Vec::new()),
        }
    }
}

fn names_of(src: &dyn MeshRead, roles: &Roles, role: Role) -> Vec<String> {
    src.sets().iter().filter(|s| role_of(s, roles) == role).map(|s| s.name.clone()).collect()
}

/// Check that every name of `pick` is a set of `role`.
fn check_names(src: &dyn MeshRead, roles: &Roles, role: Role, pick: &Pick) -> Result<(), SelectError> {
    let Pick::Names(names) = pick else { return Ok(()) };
    for n in names {
        let found = src.sets().iter().any(|s| &s.name == n && role_of(s, roles) == role);
        if !found {
            return Err(SelectError::UnknownSet {
                role: role.name(),
                name: n.clone(),
                available: names_of(src, roles, role),
            });
        }
    }
    Ok(())
}

/// Validate the names and the dimension of `sel` against `src` without building anything.
///
/// # Errors
/// [`SelectError::UnknownSet`] and [`SelectError::Dimension`].
pub fn check(src: &dyn MeshRead, sel: &Select, roles: &Roles) -> Result<(), SelectError> {
    let dim = src.dim().get();
    match sel.dimension {
        Dimension::Two if dim != 2 => return Err(SelectError::Dimension { asked: 2, mesh: dim }),
        Dimension::Three if dim != 3 => return Err(SelectError::Dimension { asked: 3, mesh: dim }),
        _ => {}
    }
    check_names(src, roles, Role::Block, &sel.blocks)?;
    check_names(src, roles, Role::Sideset, &sel.sidesets)?;
    check_names(src, roles, Role::Nodeset, &sel.nodesets)
}

fn wanted(pick: &Pick, name: &str) -> bool {
    match pick {
        Pick::All => true,
        Pick::Names(v) => v.iter().any(|n| n == name),
    }
}

/// Cut `src` down to `sel`. See the module documentation.
///
/// # Errors
/// [`SelectError`]: unknown names, a dimension mismatch, no cell left, high-order data, or a cut
/// that the classification rules refuse.
pub fn apply(src: &dyn MeshRead, sel: &Select, roles: &Roles) -> Result<Selected, SelectError> {
    check(src, sel, roles)?;
    let mut report = LossReport::new();

    // Kept cells, ascending.
    let cells: Vec<u32> = match &sel.blocks {
        Pick::All => (0..src.cell_count() as u32).collect(),
        Pick::Names(names) => {
            let mut all = Vec::new();
            for s in src.sets() {
                if let SetMembers::Cells(v) = &s.members
                    && role_of(s, roles) == Role::Block
                    && names.contains(&s.name)
                {
                    all.extend(v.iter().map(|c| c.0));
                }
            }
            sorted_unique(all)
        }
    };
    if cells.is_empty() {
        return Err(SelectError::Empty);
    }
    // Kept faces and vertices, ascending.
    let mut face_list = Vec::new();
    for &c in &cells {
        if src.high_order_cell(CellIx(c)).is_some() {
            return Err(SelectError::Unsupported("high-order cells"));
        }
        face_list.extend(src.cell_faces(CellIx(c)).iter().map(|sf| sf.face().0));
    }
    let faces = sorted_unique(face_list);
    let mut vert_list = Vec::new();
    for &f in &faces {
        if src.face_curve(FaceIx(f)).is_some() {
            return Err(SelectError::Unsupported("curved faces"));
        }
        vert_list.extend(src.face_vertices(FaceIx(f)).iter().map(|v| v.0));
    }
    let verts = sorted_unique(vert_list);
    let nv = |v: VertexIx| must_rank(&verts, v.0).map(VertexIx);

    let mut b = MeshBuilder::with_dim(src.dim());
    for &v in &verts {
        let p: Point3 = src.point(v as usize);
        let nvx = b.add_vertex(p)?;
        b.set_class(MeshEntityRef::Vertex(nvx), src.vertex_class(VertexIx(v)))?;
    }

    // A face that was interior and loses a cell becomes a boundary face, which the
    // classification rules want on a model face: one synthetic face stands in for the cut.
    let top = faces
        .iter()
        .map(|&f| src.face_class(FaceIx(f)).entity)
        .filter(|e| e.kind() == EntityKind::TopoFace)
        .map(EntityId::serial)
        .max()
        .unwrap_or(0);
    let synthetic = EntityId::new(EntityKind::TopoFace, top + 1).map(Classification::on);
    let kept_cell = |c: CellIx| rank(&cells, c.0).is_some();
    for &f in &faces {
        let fi = FaceIx(f);
        let lp = src
            .face_vertices(fi)
            .iter()
            .map(|&v| nv(v))
            .collect::<Result<Vec<_>, _>>()?;
        let nf = b.add_face(&lp)?;
        let was_two = src.face_right(fi).is_some();
        let n_kept = usize::from(kept_cell(src.face_left(fi)))
            + usize::from(src.face_right(fi).is_some_and(kept_cell));
        let class = src.face_class(fi);
        let class = if was_two && n_kept == 1 && class.dim() != Some(2) {
            synthetic.ok_or(MeshError::SerialOverflow)?
        } else {
            class
        };
        b.set_class(MeshEntityRef::Face(nf), class)?;
        if n_kept == 1 && !was_two && src.face_kind(fi) != FaceKind::Boundary {
            b.set_face_kind(nf, src.face_kind(fi))?;
        }
    }
    for &c in &cells {
        let ci = CellIx(c);
        let sfs = src
            .cell_faces(ci)
            .iter()
            .map(|sf| must_rank(&faces, sf.face().0).map(|f| SignedFace::new(FaceIx(f), sf.side())))
            .collect::<Result<Vec<_>, _>>()?;
        let corners = src
            .cell_corners(ci)
            .map(|cs| cs.iter().map(|&v| nv(v)).collect::<Result<Vec<_>, _>>())
            .transpose()?;
        let nc = b.add_cell(&sfs, corners.as_deref())?;
        b.set_class(MeshEntityRef::Cell(nc), src.cell_class(ci))?;
        if let Some(p) = src.cell_parent(ci)
            && let Some(np) = rank(&cells, p.0)
        {
            b.set_cell_parent(nc, CellIx(np))?;
        }
        if !src.cell_active(ci) {
            b.set_cell_active(nc, false)?;
        }
    }
    if let Some(t) = src.edges() {
        let mut seen = BTreeSet::new();
        for &f in &faces {
            let lp = src.face_vertices(FaceIx(f));
            for i in 0..lp.len() {
                let (x, y) = (lp[i], lp[(i + 1) % lp.len()]);
                let key = (x.min(y), x.max(y));
                if seen.insert(key)
                    && let Some(e) = t.find(key.0, key.1)
                {
                    b.set_class(MeshEntityRef::Edge(nv(key.0)?, nv(key.1)?), t.class[e.get()])?;
                }
            }
        }
    }

    // Sets.
    let keep = Keep { src, cells: &cells, faces: &faces, verts: &verts };
    for s in src.sets() {
        let role = role_of(s, roles);
        let pick = match role {
            Role::Block => &sel.blocks,
            Role::Sideset => &sel.sidesets,
            Role::Nodeset => &sel.nodesets,
            Role::Companion => &Pick::All,
        };
        if !wanted(pick, &s.name) {
            continue;
        }
        let members = keep.restrict(&s.members);
        let named = matches!(pick, Pick::Names(_)) && role != Role::Companion;
        if members.is_empty() {
            if named {
                report.push(LossItem::new(
                    EMPTY_SELECTION,
                    LossSeverity::Dropped,
                    1,
                    format!("{} `{}` is empty after the selection", role.name(), s.name),
                ));
            }
            continue;
        }
        b.add_set(&s.name, members)?;
        if let Some(l) = s.label {
            b.set_label(&s.name, Some(l))?;
        }
    }

    // Tags.
    for t in src.tags() {
        let list: &[u32] = match t.target {
            MeshEntityKind::Cell => &cells,
            MeshEntityKind::Face => &faces,
            MeshEntityKind::Vertex => &verts,
            _ => {
                report.push(LossItem::new(
                    EDGE_TAGS_DROPPED,
                    LossSeverity::Dropped,
                    1,
                    format!("edge tag `{}`", t.name),
                ));
                continue;
            }
        };
        let values = match &t.values {
            TagValues::I64(v) => TagValues::I64(list.iter().map(|&i| v[i as usize]).collect()),
            TagValues::F64(v) => TagValues::F64(list.iter().map(|&i| v[i as usize]).collect()),
            _ => return Err(SelectError::Unsupported("this kind of tag values")),
        };
        b.add_tag(&t.name, t.target, values)?;
    }

    let mut alloc = SerialAlloc::new();
    let mesh = b.finish(&mut alloc)?.mesh;
    debug_assert!(mesh.dim() == src.dim() || src.dim() == MeshDim::Three);
    Ok(Selected { mesh, report })
}
