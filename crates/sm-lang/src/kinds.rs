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
    /// Further accepted singular spellings in scripts (Cubit vocabulary: `Surface` is a face, `Curve` an edge).
    pub aliases: &'static [&'static str],
    /// Further accepted plural keywords.
    pub plural_aliases: &'static [&'static str],
}

/// Every kind in `EntityKind::ALL` order. The canonical `sm-core` name is always accepted besides `singular`.
pub const KIND_TABLE: &[KindRow] = &[
    KindRow { kind: EntityKind::TopoVertex, singular: "Vertex", plural: "vertices", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::TopoEdge, singular: "Edge", plural: "edges", aliases: &["Curve"], plural_aliases: &["curves"] },
    KindRow { kind: EntityKind::TopoFace, singular: "Face", plural: "faces", aliases: &["Surface"], plural_aliases: &["surfaces"] },
    KindRow { kind: EntityKind::TopoVolume, singular: "Volume", plural: "volumes", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::TopoBody, singular: "Body", plural: "bodies", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::Curve, singular: "GeomCurve", plural: "geom_curves", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::Surface, singular: "GeomSurface", plural: "geom_surfaces", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::Color, singular: "Color", plural: "colors", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::Group, singular: "Group", plural: "groups", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::Boundary, singular: "Boundary", plural: "boundaries", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::Instance, singular: "Instance", plural: "instances", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::Prototype, singular: "Prototype", plural: "prototypes", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::Lattice, singular: "Lattice", plural: "lattices", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::Symmetry, singular: "Symmetry", plural: "symmetries", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::Cluster, singular: "Cluster", plural: "clusters", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::PartitionCell, singular: "PartitionCell", plural: "partition_cells", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::PartitionFace, singular: "PartitionFace", plural: "partition_faces", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::PartitionEdge, singular: "PartitionEdge", plural: "partition_edges", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::PartitionVertex, singular: "PartitionVertex", plural: "partition_vertices", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::Mesh, singular: "Mesh", plural: "meshes", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::MeshVertex, singular: "Node", plural: "nodes", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::MeshEdge, singular: "MeshEdge", plural: "mesh_edges", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::MeshFace, singular: "MeshFace", plural: "mesh_faces", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::MeshCell, singular: "Element", plural: "elements", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::Interface, singular: "Interface", plural: "interfaces", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::MeshSet, singular: "MeshSet", plural: "mesh_sets", aliases: &[], plural_aliases: &[] },
    KindRow { kind: EntityKind::MeshInstance, singular: "MeshInstance", plural: "mesh_instances", aliases: &[], plural_aliases: &[] },
];

/// Resolve a singular spelling as written in a script: the preferred spelling, an alias, or the canonical `sm-core` name.
///
/// Aliases win over canonical names: `Surface` and `Curve` are the Cubit words for faces and edges, so they never
/// reach the geometry carriers (`GeomSurface`, `GeomCurve`).
pub fn kind_from_singular(name: &str) -> Option<EntityKind> {
    KIND_TABLE
        .iter()
        .find(|r| r.singular == name || r.aliases.contains(&name))
        .or_else(|| KIND_TABLE.iter().find(|r| r.kind.name() == name && !is_claimed_alias(name)))
        .map(|r| r.kind)
}

fn is_claimed_alias(name: &str) -> bool {
    KIND_TABLE.iter().any(|r| r.singular == name || r.aliases.contains(&name))
}

/// Resolve a plural keyword (the preferred one or an alias).
pub fn kind_from_plural(name: &str) -> Option<EntityKind> {
    KIND_TABLE.iter().find(|r| r.plural == name || r.plural_aliases.contains(&name)).map(|r| r.kind)
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
    let mut v: Vec<&'static str> = KIND_TABLE.iter().flat_map(|r| [r.singular, r.kind.name()].into_iter().chain(r.aliases.iter().copied())).collect();
    v.sort_unstable();
    v.dedup();
    v
}

/// Every plural keyword.
pub fn plural_spellings() -> Vec<&'static str> {
    KIND_TABLE.iter().flat_map(|r| std::iter::once(r.plural).chain(r.plural_aliases.iter().copied())).collect()
}

/// Parse the text form of an entity id held in a string (a JSON value, `ents("...")`).
///
/// A string is wire form: the canonical `sm-core` parse is tried first, so `"Surface#7"` is the geometry carrier
/// exactly as an op or query reports it. `sm-core`'s `FromStr` is itself lenient and accepts the script aliases
/// (`"Face#7"`, `"Volume#3"`), so readers of wire text accept alias spellings; output stays canonical. The fallback
/// only matters for spellings that only [`kind_from_singular`] knows.
/// In *source* (`Surface#7` without quotes) `Surface` is a face; see [`kind_from_singular`].
pub fn parse_entity_text(s: &str) -> Result<sm_core::EntityId, String> {
    if let Ok(id) = s.parse::<sm_core::EntityId>() {
        return Ok(id);
    }
    let (k, n) = s.split_once('#').ok_or_else(|| format!("`{s}` is not an entity id (expected `Kind#serial`)"))?;
    let kind = kind_from_singular(k).ok_or_else(|| format!("unknown entity kind `{k}`"))?;
    let serial: u64 = n.parse().map_err(|_| format!("`{n}` is not a valid serial number"))?;
    sm_core::EntityId::new(kind, serial).ok_or_else(|| format!("serial {serial} is too large"))
}

/// Why a user-written string is not an entity id.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum UserIdError {
    /// `Surface#n` or `Curve#n`: in source these are Cubit's face and edge, but `sm-core` prints the geometry carriers
    /// with the same names, so a string with this prefix could mean either. The language refuses to guess.
    AmbiguousKind(String),
    /// Not an id at all (message as [`parse_entity_text`]).
    Invalid(String),
}

impl UserIdError {
    /// The message.
    pub fn message(&self) -> &str {
        match self {
            Self::AmbiguousKind(m) | Self::Invalid(m) => m,
        }
    }

    /// The help line of an ambiguous kind.
    pub const AMBIGUOUS_HELP: &'static str = "write `Face#n` (or `Edge#n`) for the topology entity, or `GeomSurface#n` (or `GeomCurve#n`) for the geometry carrier";
}

/// Parse an id from a string a *user wrote* (a string literal, `ents("...")`, a string argument). Like
/// [`parse_entity_text`], except that the prefixes `Surface#` and `Curve#` are an error: `sm-core` prints the geometry
/// carriers that way while bare source `Surface#5` is a face, so the same text would silently mean two entities.
/// Ids inside query results are wire form and go through [`parse_entity_text`] instead.
pub fn parse_user_entity_text(s: &str) -> Result<sm_core::EntityId, UserIdError> {
    if let Some((k, n)) = s.split_once('#')
        && (k == "Surface" || k == "Curve")
        && !n.is_empty()
        && n.bytes().all(|b| b.is_ascii_digit())
    {
        let (face, carrier) = if k == "Surface" { ("Face", "GeomSurface") } else { ("Edge", "GeomCurve") };
        return Err(UserIdError::AmbiguousKind(format!(
            "`{s}` is ambiguous: `{k}` names the {face} in scripts but the geometry carrier {carrier} in canonical text"
        )));
    }
    parse_entity_text(s).map_err(UserIdError::Invalid)
}
