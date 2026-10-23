//! The unified point space: a [`Point`] is `(stratum, dense index)`.
//!
//! Following DMPlex, every mesh entity (vertex, edge, face, cell) is a point of one
//! space stratified by dimension. Points order by `(stratum, index)`, which is the
//! canonical order (vertices < edges < faces < cells).

use sm_core::EntityKind;

/// Dimension stratum of a [`Point`] in the Hasse diagram.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum Stratum {
    /// Dimension 0.
    Vertex = 0,
    /// Dimension 1 (derived from face loops).
    Edge = 1,
    /// Dimension 2: a polygon with an ordered vertex loop.
    Face = 2,
    /// Dimension 3: a polyhedron (possibly with a standard shape tag).
    Cell = 3,
}

impl Stratum {
    /// All strata in canonical order.
    pub const ALL: [Stratum; 4] = [Stratum::Vertex, Stratum::Edge, Stratum::Face, Stratum::Cell];

    /// Topological dimension of the stratum.
    pub const fn dim(self) -> u8 {
        self as u8
    }

    /// The [`EntityKind`] used for stable ids of entities in this stratum.
    pub const fn entity_kind(self) -> EntityKind {
        match self {
            Stratum::Vertex => EntityKind::MeshVertex,
            Stratum::Edge => EntityKind::MeshEdge,
            Stratum::Face => EntityKind::MeshFace,
            Stratum::Cell => EntityKind::MeshCell,
        }
    }

    /// Inverse of [`Stratum::entity_kind`].
    pub const fn from_entity_kind(k: EntityKind) -> Option<Self> {
        match k {
            EntityKind::MeshVertex => Some(Stratum::Vertex),
            EntityKind::MeshEdge => Some(Stratum::Edge),
            EntityKind::MeshFace => Some(Stratum::Face),
            EntityKind::MeshCell => Some(Stratum::Cell),
            _ => None,
        }
    }
}

/// A mesh entity: `(stratum, dense index within the stratum)`.
///
/// The dense index is a derived column; the stable identity is the `EntityId` returned by
/// `MeshTopology::entity_id`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Point {
    /// Stratum (dimension) of the point.
    pub stratum: Stratum,
    /// Dense index within the stratum.
    pub index: u32,
}

impl Point {
    /// Construct a point.
    pub const fn new(stratum: Stratum, index: u32) -> Self {
        Self { stratum, index }
    }
    /// A vertex point.
    pub const fn vertex(index: u32) -> Self {
        Self::new(Stratum::Vertex, index)
    }
    /// An edge point.
    pub const fn edge(index: u32) -> Self {
        Self::new(Stratum::Edge, index)
    }
    /// A face point.
    pub const fn face(index: u32) -> Self {
        Self::new(Stratum::Face, index)
    }
    /// A cell point.
    pub const fn cell(index: u32) -> Self {
        Self::new(Stratum::Cell, index)
    }
    /// Iterate the points `0..n` of a stratum in index order.
    pub fn range(stratum: Stratum, n: u32) -> impl Iterator<Item = Point> + Clone {
        (0..n).map(move |i| Point::new(stratum, i))
    }
}

/// Sentinel for "no entity" in raw `u32` index columns.
pub const NONE: u32 = u32::MAX;

/// Orientation of one incidence (cone/support entry).
///
/// * cell -> face: `Fwd` when the face's normal points out of the cell (the cell is the
///   face's `left`), `Rev` when it points in (the cell is the `right`). This is the CGNS
///   NFACE sign.
/// * face -> edge: `Fwd` when the edge's stored direction (low -> high vertex index)
///   agrees with the face loop direction.
/// * all other incidences are `Fwd`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(i8)]
pub enum Orient {
    /// Same sense (+1).
    Fwd = 1,
    /// Opposite sense (-1).
    Rev = -1,
}

impl Orient {
    /// `+1.0` or `-1.0`.
    pub const fn sign(self) -> f64 {
        match self {
            Orient::Fwd => 1.0,
            Orient::Rev => -1.0,
        }
    }
    /// The opposite orientation.
    pub const fn flip(self) -> Self {
        match self {
            Orient::Fwd => Orient::Rev,
            Orient::Rev => Orient::Fwd,
        }
    }
}
