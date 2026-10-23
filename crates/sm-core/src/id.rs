//! Stable entity ids (docs/design/ir.md §6).
//!
//! An [`EntityId`] is a `u64`: the top 8 bits are the [`EntityKind`], the low 56 bits a serial.
//! Serials are never reused. They are assigned per operation by a deterministic canonical
//! order, so replaying the timeline reproduces the same ids.

use core::fmt;

/// What kind of entity an id refers to. Stored in the top 8 bits of an [`EntityId`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
#[non_exhaustive]
pub enum EntityKind {
    // Model layer: topology
    /// Topological vertex.
    TopoVertex = 1,
    /// Topological edge (curve).
    TopoEdge = 2,
    /// Topological face (surface).
    TopoFace = 3,
    /// Topological volume (region of the cell complex).
    TopoVolume = 4,
    /// Body: a connected collection of volumes, faces, edges.
    TopoBody = 5,
    // Model layer: shape
    /// Curve geometry.
    Curve = 16,
    /// Surface geometry.
    Surface = 17,
    // Model layer: semantics
    /// Coloring: an abstract material/color.
    Color = 32,
    /// Named group (block, assembly, set).
    Group = 33,
    /// Deck transformation or boundary-kind record.
    Boundary = 34,
    /// Prototype / instance / lattice.
    Instance = 35,
    // Partition layer
    /// Partition cell (virtual decomposition piece).
    PartitionCell = 48,
    // Mesh layer
    /// A mesh (one level of a hierarchy).
    Mesh = 64,
    /// Mesh vertex (node).
    MeshVertex = 65,
    /// Mesh edge.
    MeshEdge = 66,
    /// Mesh face.
    MeshFace = 67,
    /// Mesh cell.
    MeshCell = 68,
    /// Non-conformal interface record.
    Interface = 69,
}

impl EntityKind {
    /// Decode a kind byte. Returns `None` for unknown values.
    pub const fn from_u8(v: u8) -> Option<Self> {
        use EntityKind::*;
        Some(match v {
            1 => TopoVertex,
            2 => TopoEdge,
            3 => TopoFace,
            4 => TopoVolume,
            5 => TopoBody,
            16 => Curve,
            17 => Surface,
            32 => Color,
            33 => Group,
            34 => Boundary,
            35 => Instance,
            48 => PartitionCell,
            64 => Mesh,
            65 => MeshVertex,
            66 => MeshEdge,
            67 => MeshFace,
            68 => MeshCell,
            69 => Interface,
            _ => return None,
        })
    }
}

/// A stable, never-reused entity id: 8-bit [`EntityKind`] + 56-bit serial.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct EntityId(u64);

impl EntityId {
    /// Largest serial that fits in 56 bits.
    pub const MAX_SERIAL: u64 = (1 << 56) - 1;

    /// Build an id. Returns `None` if `serial` exceeds [`Self::MAX_SERIAL`].
    pub const fn new(kind: EntityKind, serial: u64) -> Option<Self> {
        if serial > Self::MAX_SERIAL {
            return None;
        }
        Some(Self(((kind as u64) << 56) | serial))
    }

    /// The entity kind.
    pub const fn kind(self) -> EntityKind {
        match EntityKind::from_u8((self.0 >> 56) as u8) {
            Some(k) => k,
            None => panic!("EntityId with invalid kind byte"),
        }
    }

    /// The 56-bit serial.
    pub const fn serial(self) -> u64 {
        self.0 & Self::MAX_SERIAL
    }

    /// Raw `u64` (on-disk form).
    pub const fn to_raw(self) -> u64 {
        self.0
    }

    /// Decode a raw `u64`. Returns `None` if the kind byte is unknown.
    pub const fn from_raw(raw: u64) -> Option<Self> {
        match EntityKind::from_u8((raw >> 56) as u8) {
            Some(_) => Some(Self(raw)),
            None => None,
        }
    }
}

impl fmt::Debug for EntityId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}#{}", self.kind(), self.serial())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let id = EntityId::new(EntityKind::MeshCell, 42).unwrap();
        assert_eq!(id.kind(), EntityKind::MeshCell);
        assert_eq!(id.serial(), 42);
        assert_eq!(EntityId::from_raw(id.to_raw()), Some(id));
        assert!(EntityId::new(EntityKind::MeshCell, EntityId::MAX_SERIAL + 1).is_none());
        assert!(EntityId::from_raw(0).is_none());
    }

    #[test]
    fn ordering_groups_by_kind_then_serial() {
        let a = EntityId::new(EntityKind::TopoVertex, 999).unwrap();
        let b = EntityId::new(EntityKind::TopoEdge, 0).unwrap();
        assert!(a < b);
    }
}
