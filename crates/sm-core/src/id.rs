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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
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
    /// Placement of a prototype (an instance).
    Instance = 35,
    /// A reusable definition placed by instances (e.g. a pin, an assembly).
    Prototype = 36,
    /// A lattice of slots holding instances (cartesian or hex).
    Lattice = 37,
    /// A symmetry / deck-group record (generators + fundamental domain).
    Symmetry = 38,
    /// A cluster: fills placed at explicit transforms inside a fill (irregular nesting, e.g. TRISO
    /// particles in a pebble).
    Cluster = 39,
    // Decomposition layer (virtual, reversible pieces; one kind per rank)
    /// Decomposition piece of rank 3 (a virtual volume piece).
    PartitionCell = 48,
    /// Decomposition face of rank 2 (a virtual cut or composite face).
    PartitionFace = 49,
    /// Decomposition edge of rank 1.
    PartitionEdge = 50,
    /// Decomposition vertex of rank 0.
    PartitionVertex = 51,
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
    /// Named mesh set (node/side/element set), minted by the mesh layer.
    MeshSet = 70,
    /// Mesh instance (placement of a prototype mesh or an unfolded image), minted by the mesh layer.
    MeshInstance = 71,
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
            36 => Prototype,
            37 => Lattice,
            38 => Symmetry,
            39 => Cluster,
            48 => PartitionCell,
            49 => PartitionFace,
            50 => PartitionEdge,
            51 => PartitionVertex,
            64 => Mesh,
            65 => MeshVertex,
            66 => MeshEdge,
            67 => MeshFace,
            68 => MeshCell,
            69 => Interface,
            70 => MeshSet,
            71 => MeshInstance,
            _ => return None,
        })
    }

    /// Every kind, in ascending byte order.
    pub const ALL: &'static [EntityKind] = {
        use EntityKind::*;
        &[
            TopoVertex, TopoEdge, TopoFace, TopoVolume, TopoBody, Curve, Surface, Color, Group, Boundary, Instance, Prototype,
            Lattice, Symmetry, Cluster, PartitionCell, PartitionFace, PartitionEdge, PartitionVertex, Mesh, MeshVertex, MeshEdge, MeshFace,
            MeshCell, Interface, MeshSet, MeshInstance,
        ]
    };

    /// Stable text name (the on-disk spelling used by `Kind#serial`; frozen).
    pub const fn name(self) -> &'static str {
        use EntityKind::*;
        match self {
            TopoVertex => "TopoVertex",
            TopoEdge => "TopoEdge",
            TopoFace => "TopoFace",
            TopoVolume => "TopoVolume",
            TopoBody => "TopoBody",
            Curve => "Curve",
            Surface => "Surface",
            Color => "Color",
            Group => "Group",
            Boundary => "Boundary",
            Instance => "Instance",
            Prototype => "Prototype",
            Lattice => "Lattice",
            Symmetry => "Symmetry",
            Cluster => "Cluster",
            PartitionCell => "PartitionCell",
            PartitionFace => "PartitionFace",
            PartitionEdge => "PartitionEdge",
            PartitionVertex => "PartitionVertex",
            Mesh => "Mesh",
            MeshVertex => "MeshVertex",
            MeshEdge => "MeshEdge",
            MeshFace => "MeshFace",
            MeshCell => "MeshCell",
            Interface => "Interface",
            MeshSet => "MeshSet",
            MeshInstance => "MeshInstance",
        }
    }
}

impl fmt::Display for EntityKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

impl core::str::FromStr for EntityKind {
    type Err = ParseIdError;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        EntityKind::ALL.iter().copied().find(|k| k.name() == s).ok_or(ParseIdError)
    }
}

/// JSON-schema pattern of the text form of an [`EntityId`] (`Kind#serial`). Syntax only: the
/// kind name and serial range are checked by parsing.
pub const ENTITY_ID_PATTERN: &str = "^[A-Za-z]+#[0-9]+$";

/// A stable, never-reused entity id: 8-bit [`EntityKind`] + 56-bit serial. Never zero (kind
/// byte 0 is invalid), so `Option<EntityId>` is 8 bytes.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct EntityId(core::num::NonZeroU64);

impl EntityId {
    /// Largest serial that fits in 56 bits.
    pub const MAX_SERIAL: u64 = (1 << 56) - 1;

    /// Build an id. Returns `None` if `serial` exceeds [`Self::MAX_SERIAL`].
    pub const fn new(kind: EntityKind, serial: u64) -> Option<Self> {
        if serial > Self::MAX_SERIAL {
            return None;
        }
        match core::num::NonZeroU64::new(((kind as u64) << 56) | serial) {
            Some(n) => Some(Self(n)),
            None => None,
        }
    }

    /// The entity kind.
    pub const fn kind(self) -> EntityKind {
        match EntityKind::from_u8((self.0.get() >> 56) as u8) {
            Some(k) => k,
            None => panic!("EntityId with invalid kind byte"),
        }
    }

    /// The 56-bit serial.
    pub const fn serial(self) -> u64 {
        self.0.get() & Self::MAX_SERIAL
    }

    /// Raw `u64` (on-disk form).
    pub const fn to_raw(self) -> u64 {
        self.0.get()
    }

    /// Decode a raw `u64`. Returns `None` if the kind byte is unknown.
    pub const fn from_raw(raw: u64) -> Option<Self> {
        match (EntityKind::from_u8((raw >> 56) as u8), core::num::NonZeroU64::new(raw)) {
            (Some(_), Some(n)) => Some(Self(n)),
            _ => None,
        }
    }
}

impl fmt::Debug for EntityId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}#{}", self.kind().name(), self.serial())
    }
}

impl fmt::Display for EntityId {
    /// Text form `Kind#serial`, e.g. `MeshCell#42`. Used by text codecs (TOML/JSON).
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}#{}", self.kind().name(), self.serial())
    }
}

impl core::str::FromStr for EntityId {
    type Err = ParseIdError;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let (k, n) = s.split_once('#').ok_or(ParseIdError)?;
        let serial: u64 = n.parse().map_err(|_| ParseIdError)?;
        let kind: EntityKind = k.parse()?;
        EntityId::new(kind, serial).ok_or(ParseIdError)
    }
}

/// Error parsing an [`EntityId`] from its `Kind#serial` text form.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ParseIdError;

impl fmt::Display for ParseIdError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("invalid entity id (expected `Kind#serial`)")
    }
}

impl std::error::Error for ParseIdError {}

/// Serializes as the text form `Kind#serial` (human-readable, git-diffable). Binary backends store [`EntityId::to_raw`].
#[cfg(feature = "serde")]
impl serde::Serialize for EntityId {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.collect_str(self)
    }
}

#[cfg(feature = "serde")]
impl<'de> serde::Deserialize<'de> for EntityId {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let s = <std::borrow::Cow<'de, str>>::deserialize(d)?;
        s.parse().map_err(serde::de::Error::custom)
    }
}

#[cfg(feature = "schemars")]
impl schemars::JsonSchema for EntityId {
    fn schema_name() -> std::borrow::Cow<'static, str> {
        "EntityId".into()
    }
    fn json_schema(_: &mut schemars::SchemaGenerator) -> schemars::Schema {
        schemars::json_schema!({ "type": "string", "pattern": ENTITY_ID_PATTERN, "description": "Stable entity id `Kind#serial`" })
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
    fn nonzero_layout_and_raw_bits() {
        assert_eq!(core::mem::size_of::<Option<EntityId>>(), 8);
        assert!(EntityId::from_raw(0).is_none());
        for &k in EntityKind::ALL {
            for serial in [0, 1, 42, EntityId::MAX_SERIAL] {
                let id = EntityId::new(k, serial).unwrap();
                assert_eq!(id.to_raw(), ((k as u64) << 56) | serial);
                assert_eq!(id.kind(), k);
                assert_eq!(id.serial(), serial);
                assert_eq!(EntityId::from_raw(id.to_raw()), Some(id));
                let text = format!("{}#{serial}", k.name());
                assert_eq!(id.to_string(), text);
                assert_eq!(text.parse::<EntityId>(), Ok(id));
            }
        }
    }

    #[test]
    fn text_roundtrip() {
        let id = EntityId::new(EntityKind::TopoFace, 7).unwrap();
        assert_eq!(id.to_string(), "TopoFace#7");
        assert_eq!("TopoFace#7".parse::<EntityId>(), Ok(id));
        assert!("Nope#1".parse::<EntityId>().is_err());
    }

    #[test]
    fn kind_names_roundtrip() {
        let mut prev = 0u8;
        for &k in EntityKind::ALL {
            assert_eq!(k.name(), format!("{k:?}"));
            assert_eq!(k.name().parse::<EntityKind>(), Ok(k));
            assert_eq!(EntityKind::from_u8(k as u8), Some(k));
            assert!(k as u8 > prev);
            prev = k as u8;
        }
        let n = (0u8..=255).filter_map(EntityKind::from_u8).count();
        assert_eq!(n, EntityKind::ALL.len());
        assert_eq!(EntityKind::MeshSet as u8, 70);
        assert_eq!(EntityKind::MeshInstance as u8, 71);
    }

    #[test]
    fn ordering_groups_by_kind_then_serial() {
        let a = EntityId::new(EntityKind::TopoVertex, 999).unwrap();
        let b = EntityId::new(EntityKind::TopoEdge, 0).unwrap();
        assert!(a < b);
    }
}
