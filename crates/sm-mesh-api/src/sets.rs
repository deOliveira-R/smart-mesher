//! Named sets and typed tag columns.

use crate::index::{CellIx, FaceIx, MeshEntityKind, VertexIx};
use sm_core::EntityId;

/// Members of a set, ascending and unique.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SetMembers {
    /// A vertex set.
    Vertices(Vec<VertexIx>),
    /// A face set (e.g. a boundary patch).
    Faces(Vec<FaceIx>),
    /// A cell set (e.g. a block).
    Cells(Vec<CellIx>),
}

impl SetMembers {
    /// Entity kind of the members.
    pub const fn kind(&self) -> MeshEntityKind {
        match self {
            SetMembers::Vertices(_) => MeshEntityKind::Vertex,
            SetMembers::Faces(_) => MeshEntityKind::Face,
            SetMembers::Cells(_) => MeshEntityKind::Cell,
        }
    }
    /// Number of members.
    pub fn len(&self) -> usize {
        match self {
            SetMembers::Vertices(v) => v.len(),
            SetMembers::Faces(v) => v.len(),
            SetMembers::Cells(v) => v.len(),
        }
    }
    /// True if the set has no members.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
    /// Raw member indices, ascending.
    pub fn raw(&self) -> Vec<u32> {
        match self {
            SetMembers::Vertices(v) => v.iter().map(|x| x.0).collect(),
            SetMembers::Faces(v) => v.iter().map(|x| x.0).collect(),
            SetMembers::Cells(v) => v.iter().map(|x| x.0).collect(),
        }
    }
    /// True if `ix` is a member (`O(log n)`).
    pub fn contains(&self, ix: u32) -> bool {
        match self {
            SetMembers::Vertices(v) => v.binary_search(&VertexIx(ix)).is_ok(),
            SetMembers::Faces(v) => v.binary_search(&FaceIx(ix)).is_ok(),
            SetMembers::Cells(v) => v.binary_search(&CellIx(ix)).is_ok(),
        }
    }
}

/// A named set of mesh entities. Sets are annotations: they never replace classification.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MeshSet {
    /// Stable id (`EntityKind::Group`).
    pub id: EntityId,
    /// Unique name within the mesh.
    pub name: String,
    /// Members.
    pub members: SetMembers,
}

/// Values of a tag column: one per entity, in dense index order.
#[derive(Clone, Debug, PartialEq)]
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
pub struct TagColumn {
    /// Name, unique per target kind.
    pub name: String,
    /// Which entities the column is over.
    pub target: MeshEntityKind,
    /// One value per entity of `target`, in dense index order.
    pub values: TagValues,
}
