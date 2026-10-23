//! Typed entity handles, operation ids, orientation sense and the deterministic id allocator.

use sm_core::{EntityId, EntityKind};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

macro_rules! topo_id {
    ($(#[$m:meta])* $name:ident, $kind:expr) => {
        $(#[$m])*
        #[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
        #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
        pub struct $name(pub(crate) EntityId);
        impl $name {
            /// Wrap an id; `None` if its kind is wrong.
            pub fn new(id: EntityId) -> Option<Self> {
                (id.kind() == $kind).then_some(Self(id))
            }
            /// The underlying entity id.
            pub const fn id(self) -> EntityId {
                self.0
            }
            /// The 56-bit serial (also the arena index in the reference complex).
            pub const fn serial(self) -> u64 {
                self.0.serial()
            }
        }
        impl core::fmt::Debug for $name {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                core::fmt::Debug::fmt(&self.0, f)
            }
        }
        impl core::fmt::Display for $name {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                core::fmt::Display::fmt(&self.0, f)
            }
        }
        impl From<$name> for EntityId {
            fn from(v: $name) -> EntityId {
                v.0
            }
        }
    };
}

topo_id!(
    /// Handle of a vertex (`EntityKind::TopoVertex`).
    VertexId,
    EntityKind::TopoVertex
);
topo_id!(
    /// Handle of an edge (`EntityKind::TopoEdge`).
    EdgeId,
    EntityKind::TopoEdge
);
topo_id!(
    /// Handle of a face (`EntityKind::TopoFace`).
    FaceId,
    EntityKind::TopoFace
);
topo_id!(
    /// Handle of a volume (`EntityKind::TopoVolume`).
    VolumeId,
    EntityKind::TopoVolume
);
topo_id!(
    /// Handle of a body (`EntityKind::TopoBody`).
    BodyId,
    EntityKind::TopoBody
);
topo_id!(
    /// Handle of an instance record (`EntityKind::Instance`).
    InstanceId,
    EntityKind::Instance
);

/// Identifier of the operation (timeline node) that performed an edit. Supplied by the
/// caller (the timeline), never invented by the topology, so replay reproduces lineage.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct OpId(pub u64);

/// Relative orientation of a use with respect to its definition (OCCT `TopAbs_Orientation`
/// restricted to the two cases a cell complex needs).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Sense {
    /// Same direction as the definition.
    Forward,
    /// Opposite direction.
    Reversed,
}

impl Sense {
    /// The opposite sense.
    pub const fn flip(self) -> Self {
        match self {
            Self::Forward => Self::Reversed,
            Self::Reversed => Self::Forward,
        }
    }
    /// Composition: `Forward` is the identity, two reversals cancel.
    pub const fn compose(self, other: Self) -> Self {
        match (self, other) {
            (Self::Forward, s) | (s, Self::Forward) => s,
            _ => Self::Forward,
        }
    }
    /// `+1` for `Forward`, `-1` for `Reversed`.
    pub const fn sign(self) -> i32 {
        match self {
            Self::Forward => 1,
            Self::Reversed => -1,
        }
    }
}

/// Deterministic id allocator: one never-reused counter per [`EntityKind`].
///
/// Ids come from creation order. An operation that creates several entities of one kind at
/// once must create them in a canonical order (documented per edit method), so replaying
/// the same op sequence reproduces the same serials.
#[derive(Clone, Debug)]
pub struct IdAllocator {
    next: [u64; 256],
}

impl Default for IdAllocator {
    fn default() -> Self {
        Self { next: [0; 256] }
    }
}

impl IdAllocator {
    /// A fresh allocator (all counters at 0).
    pub fn new() -> Self {
        Self::default()
    }

    /// Allocate the next id of `kind`; `None` if the 56-bit serial space is exhausted.
    pub fn alloc(&mut self, kind: EntityKind) -> Option<EntityId> {
        let slot = &mut self.next[kind as usize];
        let id = EntityId::new(kind, *slot)?;
        *slot += 1;
        Some(id)
    }

    /// How many ids of `kind` have been issued.
    pub fn issued(&self, kind: EntityKind) -> u64 {
        self.next[kind as usize]
    }
}
