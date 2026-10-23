//! Dense indices and the oriented cell->face reference.
//!
//! Stable identity is an [`sm_core::EntityId`]; the dense indices here are derived columns.
//! In a finished mesh, dense index order equals ascending stable-id order, so "lower cell id"
//! and "lower cell index" are the same thing.

macro_rules! dense_index {
    ($(#[$m:meta])* $name:ident) => {
        $(#[$m])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
        pub struct $name(pub u32);
        impl $name {
            /// Sentinel meaning "none" in raw `u32` columns.
            pub const NONE: u32 = u32::MAX;
            /// Build from a `usize` (debug-asserts that it fits below the sentinel).
            pub const fn new(i: usize) -> Self {
                debug_assert!(i < u32::MAX as usize);
                Self(i as u32)
            }
            /// The index as `usize`, for slicing.
            pub const fn get(self) -> usize {
                self.0 as usize
            }
        }
    };
}

dense_index!(
    /// Dense index of a mesh vertex (also the index of its point in the coordinate arrays).
    VertexIx
);
dense_index!(
    /// Dense index of a mesh face.
    FaceIx
);
dense_index!(
    /// Dense index of a mesh cell.
    CellIx
);
dense_index!(
    /// Dense index of a mesh edge in an [`crate::EdgeTable`].
    EdgeIx
);

/// Which side of a face a cell is on, given the face's orientation rule.
///
/// The face loop's right-hand normal points from `Left` to `Right`. A `Left` cell sees the
/// normal as *outward*; a `Right` cell sees it as *inward*. This is the CGNS NFACE sign.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Side {
    /// The cell sees the normal as outward (lower cell id of the pair; the only cell of a boundary face).
    Left,
    /// The cell sees the normal as inward.
    Right,
}

impl Side {
    /// +1.0 for `Left` (outward), -1.0 for `Right` (inward).
    pub const fn sign(self) -> f64 {
        match self {
            Side::Left => 1.0,
            Side::Right => -1.0,
        }
    }
    /// The other side.
    pub const fn flipped(self) -> Self {
        match self {
            Side::Left => Side::Right,
            Side::Right => Side::Left,
        }
    }
}

/// One entry of the oriented cell->face table.
///
/// In a finished mesh `side` obeys the declared orientation rule. In a builder it is relative
/// to the face's *current* loop and is normalised by `finish`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct CellFace {
    /// The face.
    pub face: FaceIx,
    /// Which side of the face this cell is on.
    pub side: Side,
}

/// Kind of mesh entity addressed by classification, sets and tags.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum MeshEntityKind {
    /// Vertex.
    Vertex,
    /// Edge (optional, derived).
    Edge,
    /// Face.
    Face,
    /// Cell.
    Cell,
}

/// A reference to one mesh entity of any kind.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum MeshEntity {
    /// A vertex.
    Vertex(VertexIx),
    /// An edge.
    Edge(EdgeIx),
    /// A face.
    Face(FaceIx),
    /// A cell.
    Cell(CellIx),
}

impl MeshEntity {
    /// The kind of this entity.
    pub const fn kind(self) -> MeshEntityKind {
        match self {
            MeshEntity::Vertex(_) => MeshEntityKind::Vertex,
            MeshEntity::Edge(_) => MeshEntityKind::Edge,
            MeshEntity::Face(_) => MeshEntityKind::Face,
            MeshEntity::Cell(_) => MeshEntityKind::Cell,
        }
    }
}
