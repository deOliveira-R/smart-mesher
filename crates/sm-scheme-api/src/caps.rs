//! Capabilities: what a scheme produces. Impositions are expressed over capabilities, so a
//! new structured scheme satisfies "quad and structured" without editing the crates that
//! impose it.

use core::ops::{BitAnd, BitOr, BitOrAssign};
use serde::{Deserialize, Serialize};

/// A set of capability bits. Combine with `|`; query with [`Caps::contains`] and
/// [`Caps::intersects`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Caps(pub u32);

impl Caps {
    /// No capability.
    pub const NONE: Caps = Caps(0);
    /// Produces triangles on a surface.
    pub const TRI: Caps = Caps(1);
    /// Produces quadrilaterals on a surface.
    pub const QUAD: Caps = Caps(1 << 1);
    /// Produces a logically structured (i, j) grid ([`crate::NodeGrid`]).
    pub const STRUCTURED: Caps = Caps(1 << 2);
    /// Produces tetrahedra.
    pub const TET: Caps = Caps(1 << 3);
    /// Produces hexahedra.
    pub const HEX: Caps = Caps(1 << 4);
    /// Produces wedges (prisms).
    pub const WEDGE: Caps = Caps(1 << 5);
    /// Produces pyramids.
    pub const PYRAMID: Caps = Caps(1 << 6);
    /// Produces general polygons or polyhedra.
    pub const POLY: Caps = Caps(1 << 7);
    /// Can grow a boundary layer from a face.
    pub const BOUNDARY_LAYER: Caps = Caps(1 << 8);
    /// Writes the meshes of its lower-dimensional children itself (claims).
    pub const MESHES_CHILDREN: Caps = Caps(1 << 9);
    /// The scheme produces whatever its copy source produces. Resolved through the copy
    /// class root by the driver; only meaningful for the reserved `copy` scheme.
    pub const INHERIT: Caps = Caps(1 << 31);

    /// True if every bit of `other` is set.
    pub const fn contains(self, other: Caps) -> bool {
        self.0 & other.0 == other.0
    }

    /// True if any bit of `other` is set.
    pub const fn intersects(self, other: Caps) -> bool {
        self.0 & other.0 != 0
    }

    /// The bits of `self` not in `other`.
    pub const fn without(self, other: Caps) -> Caps {
        Caps(self.0 & !other.0)
    }
}

impl BitOr for Caps {
    type Output = Caps;
    fn bitor(self, rhs: Caps) -> Caps {
        Caps(self.0 | rhs.0)
    }
}

impl BitOrAssign for Caps {
    fn bitor_assign(&mut self, rhs: Caps) {
        self.0 |= rhs.0;
    }
}

impl BitAnd for Caps {
    type Output = Caps;
    fn bitand(self, rhs: Caps) -> Caps {
        Caps(self.0 & rhs.0)
    }
}
