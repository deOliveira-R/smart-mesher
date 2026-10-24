//! The entity-kind alias table: typed-id spellings and plural keywords (frozen by a test).

use sm_core::EntityKind;

/// One row: the kind, its preferred singular spelling, extra accepted singular aliases and its plural keyword.
#[derive(Clone, Copy, Debug)]
pub struct KindRow {
    /// The kind.
    pub kind: EntityKind,
    /// Preferred singular spelling in `Kind#serial` (the canonical `sm-core` name, or a short alias).
    pub singular: &'static str,
    /// Plural keyword for `all <plural>` and `<plural> in ...`.
    pub plural: &'static str,
}

/// Every kind in `EntityKind::ALL` order. The canonical `sm-core` name is always accepted besides `singular`.
pub const KIND_TABLE: &[KindRow] = &[
    KindRow { kind: EntityKind::TopoVertex, singular: "Vertex", plural: "vertices" },
    KindRow { kind: EntityKind::TopoEdge, singular: "Edge", plural: "edges" },
    KindRow { kind: EntityKind::TopoFace, singular: "Face", plural: "faces" },
    KindRow { kind: EntityKind::TopoVolume, singular: "Volume", plural: "volumes" },
    KindRow { kind: EntityKind::TopoBody, singular: "Body", plural: "bodies" },
    KindRow { kind: EntityKind::Curve, singular: "Curve", plural: "curves" },
    KindRow { kind: EntityKind::Surface, singular: "Surface", plural: "surfaces" },
    KindRow { kind: EntityKind::Color, singular: "Color", plural: "colors" },
    KindRow { kind: EntityKind::Group, singular: "Group", plural: "groups" },
    KindRow { kind: EntityKind::Boundary, singular: "Boundary", plural: "boundaries" },
    KindRow { kind: EntityKind::Instance, singular: "Instance", plural: "instances" },
    KindRow { kind: EntityKind::Prototype, singular: "Prototype", plural: "prototypes" },
    KindRow { kind: EntityKind::Lattice, singular: "Lattice", plural: "lattices" },
    KindRow { kind: EntityKind::Symmetry, singular: "Symmetry", plural: "symmetries" },
    KindRow { kind: EntityKind::Cluster, singular: "Cluster", plural: "clusters" },
    KindRow { kind: EntityKind::PartitionCell, singular: "PartitionCell", plural: "partition_cells" },
    KindRow { kind: EntityKind::PartitionFace, singular: "PartitionFace", plural: "partition_faces" },
    KindRow { kind: EntityKind::PartitionEdge, singular: "PartitionEdge", plural: "partition_edges" },
    KindRow { kind: EntityKind::PartitionVertex, singular: "PartitionVertex", plural: "partition_vertices" },
    KindRow { kind: EntityKind::Mesh, singular: "Mesh", plural: "meshes" },
    KindRow { kind: EntityKind::MeshVertex, singular: "Node", plural: "nodes" },
    KindRow { kind: EntityKind::MeshEdge, singular: "MeshEdge", plural: "mesh_edges" },
    KindRow { kind: EntityKind::MeshFace, singular: "MeshFace", plural: "mesh_faces" },
    KindRow { kind: EntityKind::MeshCell, singular: "Element", plural: "elements" },
    KindRow { kind: EntityKind::Interface, singular: "Interface", plural: "interfaces" },
    KindRow { kind: EntityKind::MeshSet, singular: "MeshSet", plural: "mesh_sets" },
    KindRow { kind: EntityKind::MeshInstance, singular: "MeshInstance", plural: "mesh_instances" },
];

/// Resolve a singular spelling: the short alias or the canonical `sm-core` name.
pub fn kind_from_singular(name: &str) -> Option<EntityKind> {
    KIND_TABLE
        .iter()
        .find(|r| r.singular == name || r.kind.name() == name)
        .map(|r| r.kind)
}

/// Resolve a plural keyword.
pub fn kind_from_plural(name: &str) -> Option<EntityKind> {
    KIND_TABLE.iter().find(|r| r.plural == name).map(|r| r.kind)
}

/// The preferred singular spelling of a kind (falls back to the canonical name for kinds added later).
pub fn singular_of(kind: EntityKind) -> &'static str {
    KIND_TABLE.iter().find(|r| r.kind == kind).map_or(kind.name(), |r| r.singular)
}

/// The plural keyword of a kind, if the kind is in the table.
pub fn plural_of(kind: EntityKind) -> Option<&'static str> {
    KIND_TABLE.iter().find(|r| r.kind == kind).map(|r| r.plural)
}

/// Every accepted singular spelling (aliases and canonical names), for suggestions and completion.
pub fn singular_spellings() -> Vec<&'static str> {
    let mut v: Vec<&'static str> = KIND_TABLE.iter().flat_map(|r| [r.singular, r.kind.name()]).collect();
    v.sort_unstable();
    v.dedup();
    v
}

/// Every plural keyword.
pub fn plural_spellings() -> Vec<&'static str> {
    KIND_TABLE.iter().map(|r| r.plural).collect()
}

/// Parse the text form of an entity id, accepting the aliases (`Volume#3`) as well as canonical names.
pub fn parse_entity_text(s: &str) -> Result<sm_core::EntityId, String> {
    let (k, n) = s.split_once('#').ok_or_else(|| format!("`{s}` is not an entity id (expected `Kind#serial`)"))?;
    let kind = kind_from_singular(k).ok_or_else(|| format!("unknown entity kind `{k}`"))?;
    let serial: u64 = n.parse().map_err(|_| format!("`{n}` is not a valid serial number"))?;
    sm_core::EntityId::new(kind, serial).ok_or_else(|| format!("serial {serial} is too large"))
}
