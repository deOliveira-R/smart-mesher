//! Named sets and integer tag columns.

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

/// A named integer column over all entities of one kind (dense; one value per entity).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TagColumn {
    /// Unique name within the mesh.
    pub name: String,
    /// Which entities the column is over.
    pub target: MeshEntityKind,
    /// One value per entity of `target`, in dense index order.
    pub values: Vec<i64>,
}
