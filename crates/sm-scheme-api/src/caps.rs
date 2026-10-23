//! Capabilities: what a scheme's output *is*, so requirements can name properties instead of
//! scheme names.
//!
//! A sweep needs its linking faces to be structured quads; a mapped face, a submapped face and a
//! copy of either all qualify. Naming the property (`Caps::QUAD | Caps::STRUCTURED`) instead of
//! the list `{map, submap}` lets a deck copy satisfy a sweep and lets new schemes plug in without
//! editing every scheme that imposes requirements.

use core::ops::{BitOr, BitOrAssign};
use serde::{Deserialize, Serialize};

/// A set of capability bits.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct Caps(pub u32);

impl Caps {
    /// No capability.
    pub const NONE: Caps = Caps(0);
    /// Produces triangles (surface) or tri-faced cells.
    pub const TRI: Caps = Caps(1 << 0);
    /// Produces quadrilaterals.
    pub const QUAD: Caps = Caps(1 << 1);
    /// Exposes a logical `(i, j)` grid of its surface patch ([`crate::Grid`]); mapped faces.
    pub const STRUCTURED: Caps = Caps(1 << 2);
    /// Produces tetrahedra.
    pub const TET: Caps = Caps(1 << 3);
    /// Produces hexahedra.
    pub const HEX: Caps = Caps(1 << 4);
    /// Produces wedges.
    pub const WEDGE: Caps = Caps(1 << 5);
    /// Produces pyramids.
    pub const PYRAMID: Caps = Caps(1 << 6);
    /// Produces general polyhedra.
    pub const POLY: Caps = Caps(1 << 7);
    /// Inserts boundary layers (forbidden on deck faces, ir.md §5.1).
    pub const BOUNDARY_LAYER: Caps = Caps(1 << 8);
    /// Meshes some of its own lower-dimensional children (`Plan::claims`), e.g. a sweep target.
    pub const MESHES_CHILDREN: Caps = Caps(1 << 9);
    /// The output has the capabilities of a source entity's mesh (copy schemes). The driver
    /// resolves it against the source's scheme before checking requirements.
    pub const INHERIT: Caps = Caps(1 << 10);

    /// True if every bit of `other` is set here.
    pub const fn contains(self, other: Caps) -> bool {
        self.0 & other.0 == other.0
    }

    /// True if any bit of `other` is set here.
    pub const fn intersects(self, other: Caps) -> bool {
        self.0 & other.0 != 0
    }

    /// True if no bit is set.
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// Bits set here and not in `other`.
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

/// Role of a face in a sweep (Cubit source / target / linking).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Role {
    /// Meshed first, by any surface scheme the requirements allow.
    Source,
    /// Meshed by the volume scheme itself (claimed), never by the surface pass.
    Target,
    /// Joins source to target; must be structured so layers can be read off it.
    Linking,
}
