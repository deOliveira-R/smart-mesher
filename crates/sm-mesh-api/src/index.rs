//! Dense indices, the packed signed cell->face reference, and entity kinds.
//!
//! Stable identity is an [`sm_core::EntityId`]; the dense indices here are derived columns.
//! In a finished mesh, dense index order equals ascending stable-id order, so "lower cell id"
//! and "lower cell index" are the same thing.

/// Sentinel stored in raw `u32` columns (`face_right`, parents) meaning "no cell".
pub const NO_CELL: u32 = u32::MAX;

/// Largest number of faces a mesh may hold (the sign of a [`SignedFace`] takes one bit).
pub const MAX_FACES: usize = 1 << 31;

macro_rules! dense_index {
    ($(#[$m:meta])* $name:ident) => {
        $(#[$m])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
        #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
        #[repr(transparent)]
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
    /// Dense index of a mesh vertex.
    VertexIx
);
dense_index!(
    /// Dense index of a mesh edge in an [`crate::EdgeTable`].
    EdgeIx
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
    /// Index into the coordinate arrays: `[0, n_vertices)` are the vertices (same number as
    /// their [`VertexIx`]), the rest are extra high-order nodes.
    PointIx
);

impl From<VertexIx> for PointIx {
    fn from(v: VertexIx) -> Self {
        PointIx(v.0)
    }
}

/// Which side of a face a cell is on, given the face's orientation rule.
///
/// The face loop's right-hand normal points from `Left` to `Right`. A `Left` cell sees the
/// normal as *outward*; a `Right` cell sees it as *inward*. This is the CGNS NFACE sign.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Side {
    /// The cell sees the normal as outward (lower cell id of the pair; the only cell of a one-cell face).
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

/// One entry of the oriented cell->face table: a face index with the cell's side packed into
/// bit 0 (`0` = [`Side::Left`], outward; `1` = [`Side::Right`], inward). At most
/// [`MAX_FACES`] faces can be addressed.
///
/// In a finished mesh the side obeys the declared orientation rule. In a builder it is relative
/// to the face's *current* loop and is normalised by `finish`.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct SignedFace(u32);

impl SignedFace {
    /// Pack a face and side. `f` must be below [`MAX_FACES`] (debug-asserted; builders reject larger meshes).
    pub const fn new(f: FaceIx, s: Side) -> Self {
        debug_assert!((f.0 as usize) < MAX_FACES);
        Self((f.0 << 1) | (matches!(s, Side::Right) as u32))
    }
    /// The face.
    pub const fn face(self) -> FaceIx {
        FaceIx(self.0 >> 1)
    }
    /// The side of the face this cell is on.
    pub const fn side(self) -> Side {
        if self.0 & 1 == 0 {
            Side::Left
        } else {
            Side::Right
        }
    }
    /// +1.0 (outward / CGNS NFACE `+`) or -1.0 (inward).
    pub const fn sign(self) -> f64 {
        self.side().sign()
    }
    /// The raw packed word (`face << 1 | side`), the on-disk form.
    pub const fn to_raw(self) -> u32 {
        self.0
    }
    /// Decode a raw packed word.
    pub const fn from_raw(raw: u32) -> Self {
        Self(raw)
    }
}

impl core::fmt::Debug for SignedFace {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "{}F{}",
            if self.side() == Side::Left { '+' } else { '-' },
            self.face().0
        )
    }
}

/// Kind of mesh entity addressed by classification, sets and tags.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum MeshEntityKind {
    /// Vertex.
    Vertex,
    /// Edge (optional table).
    Edge,
    /// Face.
    Face,
    /// Cell.
    Cell,
}

/// What a face is in the global mesh. `Interior` faces have two cells; every other kind has one
/// cell (the left cell, with the normal outward) and no right cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum FaceKind {
    /// Two cells; `face_right` is present.
    #[default]
    Interior,
    /// Model boundary: one cell, classified on a `TopoFace`.
    Boundary,
    /// One side of a non-conformal interface (hanging nodes, sliding meshes). One cell; the
    /// interface records that pair it with the other side live in the `interfaces` domain.
    NonConformal,
    /// Cut between partitions. One cell; reserved, partition-local views are not [`crate::MeshRead`].
    PartitionCut,
}

/// A reference to one mesh entity by dense index.
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
    /// Build from a kind and a raw dense index.
    pub const fn from_parts(k: MeshEntityKind, ix: u32) -> Self {
        match k {
            MeshEntityKind::Vertex => MeshEntity::Vertex(VertexIx(ix)),
            MeshEntityKind::Edge => MeshEntity::Edge(EdgeIx(ix)),
            MeshEntityKind::Face => MeshEntity::Face(FaceIx(ix)),
            MeshEntityKind::Cell => MeshEntity::Cell(CellIx(ix)),
        }
    }
}

/// An entity reference for builders, where edges do not have indices yet: an edge is named by
/// its two end vertices (order irrelevant).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum MeshEntityRef {
    /// A vertex.
    Vertex(VertexIx),
    /// The edge between two vertices.
    Edge(VertexIx, VertexIx),
    /// A (provisional) face.
    Face(FaceIx),
    /// A cell.
    Cell(CellIx),
}
