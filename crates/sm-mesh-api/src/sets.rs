//! Named sets and typed tag columns.

use crate::index::{CellIx, FaceIx, MeshEntityKind, PointIx, SignedFace, VertexIx};
use sm_core::EntityId;

/// Members of a set, ascending and unique.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub enum SetMembers {
    /// A vertex set.
    Vertices(Vec<VertexIx>),
    /// A face set (e.g. a boundary patch).
    Faces(Vec<FaceIx>),
    /// A cell set (e.g. a block).
    Cells(Vec<CellIx>),
    /// A side set (W0f): cell sides of faces. The side selects the face's `face_left` or
    /// `face_right` cell (a `Right` entry needs a two-cell face); both sides of a face are two
    /// entries. Ascending by raw word (`face << 1 | side`) and unique. `kind()` is `Face`.
    Sides(Vec<SignedFace>),
    /// A shell side set (W0f gaps 2; the Exodus shell sideset, PARITY D "sheet"): packed
    /// `cell << 1 | side` words of a **dimension-2** mesh, side 0 = top (the side the cell normal
    /// points to: right-hand rule on its corner cycle, which follows its classified face's
    /// normal), 1 = bottom. Ascending and unique. `kind()` is `Cell`.
    ShellSides(Vec<u32>),
    /// A point set (W0f gaps 2): indices into the `coords()` space (vertices, then high-order
    /// extra nodes), ascending and unique, range-checked against the point count. `kind()` is
    /// `Vertex` (a node set that may hold high-order nodes).
    Points(Vec<PointIx>),
}

impl SetMembers {
    /// Entity kind of the members.
    pub const fn kind(&self) -> MeshEntityKind {
        match self {
            SetMembers::Vertices(_) => MeshEntityKind::Vertex,
            SetMembers::Faces(_) => MeshEntityKind::Face,
            SetMembers::Cells(_) => MeshEntityKind::Cell,
            SetMembers::Sides(_) => MeshEntityKind::Face,
            SetMembers::ShellSides(_) => MeshEntityKind::Cell,
            SetMembers::Points(_) => MeshEntityKind::Vertex,
        }
    }
    /// Stable variant code (hashed: `Faces` and `Sides` share a kind).
    pub const fn code(&self) -> u8 {
        match self {
            SetMembers::Vertices(_) => 0,
            SetMembers::Faces(_) => 1,
            SetMembers::Cells(_) => 2,
            SetMembers::Sides(_) => 3,
            SetMembers::ShellSides(_) => 4,
            SetMembers::Points(_) => 5,
        }
    }
    /// Number of members (sides for a side set).
    pub fn len(&self) -> usize {
        match self {
            SetMembers::Vertices(v) => v.len(),
            SetMembers::Faces(v) => v.len(),
            SetMembers::Cells(v) => v.len(),
            SetMembers::Sides(v) => v.len(),
            SetMembers::ShellSides(v) => v.len(),
            SetMembers::Points(v) => v.len(),
        }
    }
    /// True if the set has no members.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
    /// Raw member words, ascending: indices, or packed `face << 1 | side` (side set) and
    /// `cell << 1 | side` (shell side set) words.
    pub fn raw(&self) -> Vec<u32> {
        match self {
            SetMembers::Vertices(v) => v.iter().map(|x| x.0).collect(),
            SetMembers::Faces(v) => v.iter().map(|x| x.0).collect(),
            SetMembers::Cells(v) => v.iter().map(|x| x.0).collect(),
            SetMembers::Sides(v) => v.iter().map(|x| x.to_raw()).collect(),
            SetMembers::ShellSides(v) => v.clone(),
            SetMembers::Points(v) => v.iter().map(|x| x.0).collect(),
        }
    }
    /// The largest entity index referenced (the face of a side, the cell of a shell side, the
    /// point of a point set), for range checks. A point set is checked against the point count,
    /// not the vertex count.
    pub fn max_index(&self) -> Option<u32> {
        match self {
            SetMembers::Sides(v) => v.last().map(|x| x.face().0),
            SetMembers::ShellSides(v) => v.last().map(|x| x >> 1),
            _ => self.raw().last().copied(),
        }
    }
    /// True if entity `ix` (of [`kind`](Self::kind)) is a member; for a (shell) side set, if
    /// either side of face (cell) `ix` is; for a point set, if point `ix` is (`O(log n)`).
    pub fn contains(&self, ix: u32) -> bool {
        match self {
            SetMembers::Vertices(v) => v.binary_search(&VertexIx(ix)).is_ok(),
            SetMembers::Faces(v) => v.binary_search(&FaceIx(ix)).is_ok(),
            SetMembers::Cells(v) => v.binary_search(&CellIx(ix)).is_ok(),
            SetMembers::Sides(v) => {
                let lo = ix.checked_mul(2);
                lo.is_some_and(|lo| v.binary_search_by_key(&lo, |x| x.to_raw()).is_ok() || v.binary_search_by_key(&(lo | 1), |x| x.to_raw()).is_ok())
            }
            SetMembers::ShellSides(v) => ix.checked_mul(2).is_some_and(|lo| v.binary_search(&lo).is_ok() || v.binary_search(&(lo | 1)).is_ok()),
            SetMembers::Points(v) => v.binary_search(&PointIx(ix)).is_ok(),
        }
    }
}

/// A named set of mesh entities. Sets are annotations: they never replace classification.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct MeshSet {
    /// Stable id (`EntityKind::MeshSet`, minted by the mesh layer).
    pub id: EntityId,
    /// Unique name within the mesh.
    pub name: String,
    /// Optional integer label for formats that number sets (`.smsh`, Exodus block / side-set
    /// ids). Unique per members kind when present.
    #[cfg_attr(feature = "serde", serde(default))]
    pub label: Option<i64>,
    /// Members.
    pub members: SetMembers,
}

impl MeshSet {
    /// A set without a label.
    pub fn new(id: EntityId, name: impl Into<String>, members: SetMembers) -> Self {
        Self { id, name: name.into(), label: None, members }
    }
    /// Set the integer label.
    pub fn with_label(mut self, label: i64) -> Self {
        self.label = Some(label);
        self
    }
}

/// Values of a tag column: one per entity, in dense index order.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub enum TagValues {
    /// Integer values (material ids, block ids, flags).
    I64(Vec<i64>),
    /// Real values (hashed by bit pattern).
    F64(Vec<f64>),
}

impl TagValues {
    /// Number of values.
    pub fn len(&self) -> usize {
        match self {
            TagValues::I64(v) => v.len(),
            TagValues::F64(v) => v.len(),
        }
    }
    /// True if there are no values.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
    /// Integer values, if this is an integer column.
    pub fn as_i64(&self) -> Option<&[i64]> {
        match self {
            TagValues::I64(v) => Some(v),
            TagValues::F64(_) => None,
        }
    }
    /// Real values, if this is a real column.
    pub fn as_f64(&self) -> Option<&[f64]> {
        match self {
            TagValues::F64(v) => Some(v),
            TagValues::I64(_) => None,
        }
    }
}

/// A named typed column over all entities of one kind (dense; one value per entity). Columns are
/// keyed by `(target, name)`.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct TagColumn {
    /// Name, unique per target kind.
    pub name: String,
    /// Which entities the column is over.
    pub target: MeshEntityKind,
    /// One value per entity of `target`, in dense index order.
    pub values: TagValues,
}

impl TagColumn {
    /// A column over the entities of `target`.
    pub fn new(name: impl Into<String>, target: MeshEntityKind, values: TagValues) -> Self {
        Self { name: name.into(), target, values }
    }
}
