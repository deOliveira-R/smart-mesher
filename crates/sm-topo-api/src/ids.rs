//! Identifiers and structural handles.
//!
//! Two tiers, by design:
//!
//! 1. **Entities** carry stable, never-reused [`EntityId`]s (`BodyId`, `VolumeId`, `FaceId`,
//!    `EdgeId`, `VertexId`). Only these appear in lineage, coloring, boundary kinds and on disk.
//! 2. **Uses and sub-structures** (shells, loops, coedges, face uses, edge uses, vertex uses) are
//!    addressed *structurally*: `(owner id, small index/side)`. They need no allocator, can never
//!    dangle independently of their owner, and are as stable as the owner is.

use core::fmt;
use sm_core::{EntityId, EntityKind};

/// Serial bit marking entities of a virtual overlay (never allocated by a host complex).
pub const VIRTUAL_SERIAL_BIT: u64 = 1 << 55;

macro_rules! typed_id {
    ($(#[$m:meta])* $name:ident, $kind:ident) => {
        $(#[$m])*
        #[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(EntityId);

        impl $name {
            /// Build from a serial. `None` if the serial does not fit in 56 bits.
            pub const fn new(serial: u64) -> Option<Self> {
                match EntityId::new(EntityKind::$kind, serial) {
                    Some(id) => Some(Self(id)),
                    None => None,
                }
            }
            /// Narrow an [`EntityId`]; `None` if it has a different kind.
            pub fn from_entity(id: EntityId) -> Option<Self> {
                (id.kind() == EntityKind::$kind).then_some(Self(id))
            }
            /// The underlying entity id.
            pub const fn entity(self) -> EntityId {
                self.0
            }
            /// The serial number (dense, in creation order for host entities).
            pub const fn serial(self) -> u64 {
                self.0.serial()
            }
            /// True for ids minted by a virtual overlay.
            pub const fn is_virtual(self) -> bool {
                self.0.serial() & VIRTUAL_SERIAL_BIT != 0
            }
        }
        impl fmt::Debug for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "{:?}", self.0)
            }
        }
        impl From<$name> for EntityId {
            fn from(v: $name) -> EntityId {
                v.0
            }
        }
    };
}

typed_id!(
    /// A body: a connected collection of volumes, sheets and wires.
    BodyId, TopoBody
);
typed_id!(
    /// A volume: a region of space bounded by shells.
    VolumeId, TopoVolume
);
typed_id!(
    /// A face: a bounded piece of surface with two sides.
    FaceId, TopoFace
);
typed_id!(
    /// An edge: a bounded piece of curve shared by any number of faces.
    EdgeId, TopoEdge
);
typed_id!(
    /// A vertex: a point.
    VertexId, TopoVertex
);

/// Lifecycle operation number (monotonically increasing per model). Orders lineage.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OpId(pub u64);

/// Whether an entity is live. Retired entities keep their id and record (ids are never reused),
/// so lineage can always be resolved.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum EntityState {
    /// Live.
    Active,
    /// Replaced by children (`Split`) in the given op.
    Split(OpId),
    /// Absorbed into a survivor (`Merged`) in the given op.
    Merged(OpId),
    /// Removed in the given op.
    Deleted(OpId),
}

impl EntityState {
    /// True if live.
    pub const fn is_active(self) -> bool {
        matches!(self, Self::Active)
    }
}

/// Which side of a face. The face normal is `surface normal * surface_sense`.
/// `Negative` is the side the normal points *away from* (the normal leaves a volume on this side);
/// `Positive` is the side the normal points *into*. Mirrors the IR rule "normal points from left to right".
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum FaceSide {
    /// Behind the normal (`left`).
    Negative,
    /// In front of the normal (`right`).
    Positive,
}

impl FaceSide {
    /// Both sides, `Negative` first.
    pub const BOTH: [FaceSide; 2] = [FaceSide::Negative, FaceSide::Positive];
    /// Array index (0 or 1).
    pub const fn index(self) -> usize {
        match self {
            Self::Negative => 0,
            Self::Positive => 1,
        }
    }
    /// The other side.
    pub const fn opposite(self) -> Self {
        match self {
            Self::Negative => Self::Positive,
            Self::Positive => Self::Negative,
        }
    }
    /// How a volume on this side sees the face normal.
    pub const fn sense(self) -> UseSense {
        match self {
            Self::Negative => UseSense::Aligned,
            Self::Positive => UseSense::Opposed,
        }
    }
}

/// FaceUse sense: how the volume owning a use sees the face normal.
/// `Aligned`: the normal points out of the volume. `Opposed`: it points into the volume.
/// After a merge, the two volumes sharing a face see it with opposite senses.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum UseSense {
    /// Face normal points out of the owning volume.
    Aligned,
    /// Face normal points into the owning volume.
    Opposed,
}

/// What lies on one side of a face.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Side {
    /// A volume.
    Volume(VolumeId),
    /// Nothing is modeled there (the exterior / implicit complement).
    Exterior,
}

impl Side {
    /// The volume, if any.
    pub const fn volume(self) -> Option<VolumeId> {
        match self {
            Self::Volume(v) => Some(v),
            Self::Exterior => None,
        }
    }
}

/// A shell of a volume: `(volume, index into the volume's shells)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ShellRef {
    /// Owning volume.
    pub volume: VolumeId,
    /// Index in `VolumeRec::shells`.
    pub index: u16,
}

/// A face loop: `(face, index into the face's loops)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct LoopRef {
    /// Owning face.
    pub face: FaceId,
    /// Index in `FaceRec::loops`.
    pub index: u16,
}

/// A coedge: the use of an edge by a face loop, in the loop's stored order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CoedgeRef {
    /// Owning face.
    pub face: FaceId,
    /// Loop index.
    pub loop_index: u16,
    /// Position in the loop.
    pub index: u32,
}

impl CoedgeRef {
    /// The loop this coedge belongs to.
    pub const fn loop_ref(self) -> LoopRef {
        LoopRef { face: self.face, index: self.loop_index }
    }
}

/// One side of a face as seen by one volume (Weiler's face use).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FaceUse {
    /// The face.
    pub face: FaceId,
    /// Which side.
    pub side: FaceSide,
}

impl FaceUse {
    /// The mate (same face, other side).
    pub const fn mate(self) -> Self {
        Self { face: self.face, side: self.side.opposite() }
    }
}

/// A loop as seen from one side of its face (Weiler's loop use). The `Positive` side traverses
/// the stored loop in reverse order and direction.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct LoopUse {
    /// The face use.
    pub face_use: FaceUse,
    /// Loop index within the face.
    pub loop_index: u16,
}

/// An edge as seen by a face use, or as a wire inside a volume (Weiler's edge use).
/// Edge uses form the *radial ring* around an edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum EdgeUse {
    /// The coedge of a face, seen from one side of that face.
    Coedge {
        /// The coedge.
        coedge: CoedgeRef,
        /// The side from which the coedge is seen.
        side: FaceSide,
    },
    /// A wire edge lying inside a volume (or free in the exterior when `volume` is `None`).
    Wire {
        /// The edge.
        edge: EdgeId,
        /// Containing volume.
        volume: Option<VolumeId>,
    },
}

/// Where a vertex is used (Weiler's vertex use).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum VertexSite {
    /// Tail corner of a coedge as traversed from `side` (also pole corners).
    Corner {
        /// The coedge.
        coedge: CoedgeRef,
        /// The traversal side.
        side: FaceSide,
    },
    /// End of a wire edge.
    WireEnd {
        /// The wire edge.
        edge: EdgeId,
        /// Which end.
        at_end: bool,
    },
    /// An isolated vertex in a volume.
    Lone {
        /// The volume (`None` = free in the exterior).
        volume: Option<VolumeId>,
    },
}

/// A vertex use: a vertex and where it occurs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct VertexUse {
    /// The vertex.
    pub vertex: VertexId,
    /// The occurrence.
    pub site: VertexSite,
}
