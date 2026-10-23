//! Dense indices, dimensions and signed face references.
//!
//! Stable identity is an [`sm_core::EntityId`]; the dense `u32` indices here are positions in
//! the tables and are derived (canonical order = ascending id).

use serde::{Deserialize, Serialize};

/// Entity dimension of a mesh table.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Dim {
    /// Vertices / nodes (dimension 0).
    Vertex = 0,
    /// Edges (dimension 1).
    Edge = 1,
    /// Faces (dimension 2).
    Face = 2,
    /// Cells (dimension 3).
    Cell = 3,
}

impl Dim {
    /// All dimensions in ascending order.
    pub const ALL: [Dim; 4] = [Dim::Vertex, Dim::Edge, Dim::Face, Dim::Cell];
}

macro_rules! ix_type {
    ($(#[$m:meta])* $name:ident) => {
        $(#[$m])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize)]
        pub struct $name(pub u32);
        impl $name {
            /// Position as `usize`.
            pub const fn index(self) -> usize {
                self.0 as usize
            }
        }
        impl From<u32> for $name {
            fn from(v: u32) -> Self {
                Self(v)
            }
        }
    };
}

ix_type!(
    /// Dense index into the vertex table.
    VertexIx
);
ix_type!(
    /// Dense index into the edge table.
    EdgeIx
);
ix_type!(
    /// Dense index into the face table.
    FaceIx
);
ix_type!(
    /// Dense index into the cell table.
    CellIx
);

/// Which side of a face a cell is on.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Sign {
    /// The cell is the face's `left` cell: the stored normal is outward (CGNS NFACE `+`).
    Plus,
    /// The cell is the face's `right` cell: the stored normal is inward (CGNS NFACE `-`).
    Minus,
}

impl Sign {
    /// `+1.0` or `-1.0`.
    pub const fn as_f64(self) -> f64 {
        match self {
            Sign::Plus => 1.0,
            Sign::Minus => -1.0,
        }
    }
    /// The opposite sign.
    pub const fn flip(self) -> Self {
        match self {
            Sign::Plus => Sign::Minus,
            Sign::Minus => Sign::Plus,
        }
    }
}

/// A face reference with sign, packed in one `u32` (bit 0 = sign, upper 31 bits = face index).
/// Limits a mesh to 2^31 faces.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct SignedFace(u32);

impl SignedFace {
    /// Pack a face and sign. The builder rejects meshes with 2^31 or more faces.
    pub const fn new(face: FaceIx, sign: Sign) -> Self {
        Self((face.0 << 1) | matches!(sign, Sign::Minus) as u32)
    }
    /// The face.
    pub const fn face(self) -> FaceIx {
        FaceIx(self.0 >> 1)
    }
    /// The sign of this cell relative to the face normal.
    pub const fn sign(self) -> Sign {
        if self.0 & 1 == 0 { Sign::Plus } else { Sign::Minus }
    }
}
