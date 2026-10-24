//! Finite-element types a block can declare.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_mesh_api::CellShape;

/// An element type of a block (Exodus/Cubit names).
///
/// Only the linear types are supported in W2: [`ElementType::is_linear`] is false for the
/// others, and `fem.block_element_type` refuses them until degree elevation exists.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize, JsonSchema,
)]
#[non_exhaustive]
pub enum ElementType {
    /// 8-node hexahedron.
    #[serde(rename = "HEX8")]
    Hex8,
    /// 4-node tetrahedron.
    #[serde(rename = "TET4")]
    Tet4,
    /// 6-node wedge (triangular prism).
    #[serde(rename = "WEDGE6")]
    Wedge6,
    /// 5-node pyramid.
    #[serde(rename = "PYRAMID5")]
    Pyramid5,
    /// 4-node quadrilateral.
    #[serde(rename = "QUAD4")]
    Quad4,
    /// 3-node triangle.
    #[serde(rename = "TRI3")]
    Tri3,
    /// 20-node serendipity hexahedron (not supported yet).
    #[serde(rename = "HEX20")]
    Hex20,
    /// 27-node triquadratic hexahedron (not supported yet).
    #[serde(rename = "HEX27")]
    Hex27,
    /// 10-node tetrahedron (not supported yet).
    #[serde(rename = "TET10")]
    Tet10,
    /// 15-node wedge (not supported yet).
    #[serde(rename = "WEDGE15")]
    Wedge15,
    /// 13-node pyramid (not supported yet).
    #[serde(rename = "PYRAMID13")]
    Pyramid13,
    /// 8-node quadrilateral (not supported yet).
    #[serde(rename = "QUAD8")]
    Quad8,
    /// 6-node triangle (not supported yet).
    #[serde(rename = "TRI6")]
    Tri6,
}

impl ElementType {
    /// The canonical name (`"HEX8"`).
    pub const fn name(self) -> &'static str {
        match self {
            Self::Hex8 => "HEX8",
            Self::Tet4 => "TET4",
            Self::Wedge6 => "WEDGE6",
            Self::Pyramid5 => "PYRAMID5",
            Self::Quad4 => "QUAD4",
            Self::Tri3 => "TRI3",
            Self::Hex20 => "HEX20",
            Self::Hex27 => "HEX27",
            Self::Tet10 => "TET10",
            Self::Wedge15 => "WEDGE15",
            Self::Pyramid13 => "PYRAMID13",
            Self::Quad8 => "QUAD8",
            Self::Tri6 => "TRI6",
        }
    }

    /// True for the supported (linear) types.
    pub const fn is_linear(self) -> bool {
        matches!(
            self,
            Self::Hex8 | Self::Tet4 | Self::Wedge6 | Self::Pyramid5 | Self::Quad4 | Self::Tri3
        )
    }

    /// The mesh cell shape this type describes (higher-order types share their linear shape).
    pub const fn shape(self) -> CellShape {
        match self {
            Self::Hex8 | Self::Hex20 | Self::Hex27 => CellShape::Hex,
            Self::Tet4 | Self::Tet10 => CellShape::Tet,
            Self::Wedge6 | Self::Wedge15 => CellShape::Wedge,
            Self::Pyramid5 | Self::Pyramid13 => CellShape::Pyramid,
            Self::Quad4 | Self::Quad8 => CellShape::Quad,
            Self::Tri3 | Self::Tri6 => CellShape::Tri,
        }
    }

    /// Stable code for hashing; never reuse a value.
    pub(crate) const fn code(self) -> u64 {
        match self {
            Self::Hex8 => 1,
            Self::Tet4 => 2,
            Self::Wedge6 => 3,
            Self::Pyramid5 => 4,
            Self::Quad4 => 5,
            Self::Tri3 => 6,
            Self::Hex20 => 7,
            Self::Hex27 => 8,
            Self::Tet10 => 9,
            Self::Wedge15 => 10,
            Self::Pyramid13 => 11,
            Self::Quad8 => 12,
            Self::Tri6 => 13,
        }
    }
}

impl std::fmt::Display for ElementType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}
