//! Value types of the cell complex: dimensions, the single `Side` tag, incidences, carriers,
//! coincidence status and lineage records.

use sm_core::{EntityId, EntityKind, Point3};
use sm_geom_api::Interval;
use smallvec::SmallVec;

/// Cell dimension. `Body` is a grouping rank above `Volume`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Dim {
    /// 0-cells.
    Vertex = 0,
    /// 1-cells.
    Edge = 1,
    /// 2-cells.
    Face = 2,
    /// 3-cells (regions of space).
    Volume = 3,
    /// Grouping of cells of any dimension (connected component, imported solid).
    Body = 4,
}

impl Dim {
    /// All ranks, ascending.
    pub const ALL: [Dim; 5] = [Dim::Vertex, Dim::Edge, Dim::Face, Dim::Volume, Dim::Body];

    /// Dense index `0..5`.
    pub const fn index(self) -> usize {
        self as usize
    }

    /// The `EntityKind` of cells of this dimension.
    pub const fn kind(self) -> EntityKind {
        match self {
            Dim::Vertex => EntityKind::TopoVertex,
            Dim::Edge => EntityKind::TopoEdge,
            Dim::Face => EntityKind::TopoFace,
            Dim::Volume => EntityKind::TopoVolume,
            Dim::Body => EntityKind::TopoBody,
        }
    }

    /// Dimension for a topology `EntityKind`; `None` for every other kind.
    pub const fn of_kind(k: EntityKind) -> Option<Dim> {
        match k {
            EntityKind::TopoVertex => Some(Dim::Vertex),
            EntityKind::TopoEdge => Some(Dim::Edge),
            EntityKind::TopoFace => Some(Dim::Face),
            EntityKind::TopoVolume => Some(Dim::Volume),
            EntityKind::TopoBody => Some(Dim::Body),
            _ => None,
        }
    }
}

/// The one side tag used by every incidence of the complex.
///
/// Meaning per relation (entry in the *parent's* boundary row):
/// - edge row -> vertex: `Left` = the edge starts here, `Right` = the edge ends here,
///   `Both` = closed edge whose start and end are this vertex (single entry).
/// - face row -> edge (a coedge): `Left` = the loop traverses the edge along its direction, so the
///   face lies on the left of the edge (coedge sense `Forward`); `Right` = against it
///   (`Reversed`). A seam edge appears twice, once each. `Both` is used only for embedded edges.
/// - volume row -> face (a face use): `Left` = the volume is on the left of the face, i.e. the
///   face normal points out of the volume (`Forward`); `Right` = the volume is on the right,
///   the normal points into it (`Reversed`); `Both` = baffle: the volume is on both sides.
///
/// The face normal points from its left side to its right side (docs/design/ir.md §6).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Side {
    /// Left.
    Left = 0,
    /// Right.
    Right = 1,
    /// Both sides at once.
    Both = 2,
}

impl Side {
    /// Swap left and right (orientation reversal); `Both` is unchanged.
    pub const fn flip(self) -> Side {
        match self {
            Side::Left => Side::Right,
            Side::Right => Side::Left,
            Side::Both => Side::Both,
        }
    }

    /// The oriented-use sense: `Left` and `Both` are `Forward`, `Right` is `Reversed`.
    pub const fn sense(self) -> Sense {
        match self {
            Side::Right => Sense::Reversed,
            _ => Sense::Forward,
        }
    }
}

/// Relative orientation of a use or a carrier.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Sense {
    /// Same orientation.
    Forward,
    /// Opposite orientation.
    Reversed,
}

impl Sense {
    /// The opposite sense.
    pub const fn flip(self) -> Sense {
        match self {
            Sense::Forward => Sense::Reversed,
            Sense::Reversed => Sense::Forward,
        }
    }
    /// The `Left`/`Right` side tag for this sense.
    pub const fn side(self) -> Side {
        match self {
            Sense::Forward => Side::Left,
            Sense::Reversed => Side::Right,
        }
    }
}

/// What lies on one side of a face.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Region {
    /// A volume of the complex.
    Volume(EntityId),
    /// The unbounded complement: no volume of the complex is on this side (void/outside).
    Exterior,
}

/// One oriented incidence: the same record is used in a parent's boundary row (`cell` = the
/// lower-dimensional cell) and in a child's coboundary row (`cell` = the parent).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Incidence {
    /// The other end of the incidence.
    pub cell: EntityId,
    /// Side tag, see [`Side`].
    pub side: Side,
    /// Loop index within a face row, shell index within a volume row (0 = outer), or
    /// [`Incidence::EMBEDDED`] for members embedded in the interior (wires, points).
    pub ring: u16,
}

impl Incidence {
    /// The `ring` value of embedded members.
    pub const EMBEDDED: u16 = u16::MAX;

    /// Construct.
    pub const fn new(cell: EntityId, side: Side, ring: u16) -> Self {
        Self { cell, side, ring }
    }
}

/// Edge-to-curve link: which curve, which parameter range, and whether the edge runs against it.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CurveCarrier {
    /// Handle of the curve (`EntityKind::Curve`) in the geometry store.
    pub curve: EntityId,
    /// Curve parameter range covered by the edge (`lo < hi`; may span a seam of a periodic curve).
    pub range: Interval,
    /// `Forward`: the edge starts at `range.lo`; `Reversed`: it starts at `range.hi`.
    pub sense: Sense,
}

/// Face-to-surface link.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SurfaceCarrier {
    /// Handle of the surface (`EntityKind::Surface`) in the geometry store.
    pub surface: EntityId,
    /// `Forward`: the face normal equals the surface normal `du x dv`; `Reversed`: opposite.
    pub sense: Sense,
}

/// The geometry attached to a cell (GMI-like carrier). Resolved through a
/// `sm_geom_api::GeometryStore`; topology never owns shape.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Carrier {
    /// No geometry (volumes, bodies, topology-only edges/faces such as degenerate edges).
    None,
    /// A vertex position.
    Point(Point3),
    /// An edge's curve.
    Curve(CurveCarrier),
    /// A face's surface.
    Surface(SurfaceCarrier),
}

/// Per-cell flag bits.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CellFlags(pub u8);

impl CellFlags {
    /// No flags.
    pub const NONE: CellFlags = CellFlags(0);
    /// A collapsed edge (zero length), e.g. at a sphere pole or cone apex. It has a vertex row
    /// but no curve.
    pub const DEGENERATE: CellFlags = CellFlags(1);
    /// True if all bits of `other` are set.
    pub const fn contains(self, other: CellFlags) -> bool {
        self.0 & other.0 == other.0
    }
    /// Bitwise union.
    pub const fn union(self, other: CellFlags) -> CellFlags {
        CellFlags(self.0 | other.0)
    }
}

/// Identifier of the engine operation that caused a change. Supplied by the caller (the op
/// engine's op number), so lineage is replay-stable.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OpId(pub u64);

/// Imprint/merge state of a coincident face pair (docs/design/ir.md §3).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Coincidence {
    /// One shared face with two sides. The record names the survivor and the absorbed (dead) face.
    Merged,
    /// Boundaries match exactly (imprinted) but there are still two faces.
    ImprintedNotMerged,
    /// Geometrically coincident but not yet imprinted: boundaries differ.
    CoincidentUnimprinted,
    /// Partial overlap: some regions coincide, others do not.
    Overlapping,
}

/// A recorded relation between two faces. `a < b` by id.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FacePair {
    /// Lower id.
    pub a: EntityId,
    /// Higher id.
    pub b: EntityId,
    /// Status.
    pub status: Coincidence,
    /// Overlap area when known (for `Overlapping`).
    pub overlap_area: Option<f64>,
}

/// What an operation did to an entity.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum LineageKind {
    /// `from = [x]` was replaced by `to = [y, z]` (parent dead, children alive).
    Split,
    /// `from = [survivor, absorbed]`, `to = [survivor]`; the absorbed cell is dead.
    Merged,
    /// `from = [x]` was deleted; `to` is empty.
    Deleted,
    /// `from = [x]`, `to = [x]`: identity kept, orientation or carrier changed.
    Modified,
}

/// One append-only lineage record ("entity X split into Y, Z by op N").
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LineageRecord {
    /// The operation.
    pub op: OpId,
    /// What happened.
    pub kind: LineageKind,
    /// Source entities.
    pub from: SmallVec<[EntityId; 2]>,
    /// Resulting entities.
    pub to: SmallVec<[EntityId; 2]>,
}

/// Input to face construction: one coedge of a loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CoedgeSpec {
    /// The edge.
    pub edge: EntityId,
    /// `Forward` if the loop traverses the edge along its direction.
    pub sense: Sense,
}

/// Input to volume construction: one face use.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaceUseSpec {
    /// The face.
    pub face: EntityId,
    /// Which side of the face the volume occupies.
    pub side: Side,
}
