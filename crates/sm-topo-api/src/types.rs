//! Value types of the cell complex: dimensions, the single `Side` tag, incidences, carriers,
//! coincidence status, lineage and coedge keys.

use sm_core::{EntityId, EntityKind, Point3};
use sm_geom_api::{CsgCell, Interval};
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

    /// The `EntityKind` of real (non-virtual) cells of this dimension.
    pub const fn kind(self) -> EntityKind {
        match self {
            Dim::Vertex => EntityKind::TopoVertex,
            Dim::Edge => EntityKind::TopoEdge,
            Dim::Face => EntityKind::TopoFace,
            Dim::Volume => EntityKind::TopoVolume,
            Dim::Body => EntityKind::TopoBody,
        }
    }

    /// The `EntityKind` of virtual (decomposition overlay) cells of this dimension, if any.
    pub const fn virtual_kind(self) -> Option<EntityKind> {
        match self {
            Dim::Vertex => Some(EntityKind::PartitionVertex),
            Dim::Edge => Some(EntityKind::PartitionEdge),
            Dim::Face => Some(EntityKind::PartitionFace),
            Dim::Volume => Some(EntityKind::PartitionCell),
            Dim::Body => None,
        }
    }

    /// Dimension for a real topology `EntityKind`; `None` for every other kind (including the
    /// virtual kinds, see [`Dim::of_kind`]).
    pub const fn of_real_kind(k: EntityKind) -> Option<Dim> {
        match k {
            EntityKind::TopoVertex => Some(Dim::Vertex),
            EntityKind::TopoEdge => Some(Dim::Edge),
            EntityKind::TopoFace => Some(Dim::Face),
            EntityKind::TopoVolume => Some(Dim::Volume),
            EntityKind::TopoBody => Some(Dim::Body),
            _ => None,
        }
    }

    /// Dimension for any cell kind: the real kinds and the per-rank virtual kinds
    /// (`PartitionVertex/Edge/Face/Cell` = 0/1/2/3-cells of the decomposition overlay).
    pub const fn of_kind(k: EntityKind) -> Option<Dim> {
        match k {
            EntityKind::PartitionVertex => Some(Dim::Vertex),
            EntityKind::PartitionEdge => Some(Dim::Edge),
            EntityKind::PartitionFace => Some(Dim::Face),
            EntityKind::PartitionCell => Some(Dim::Volume),
            other => Self::of_real_kind(other),
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
/// The face normal points from its left side to its right side (docs/design/ir.md §6): the
/// loops of a face run counter-clockwise seen from its **right** side.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
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

impl From<sm_geom_api::Sense> for Sense {
    fn from(s: sm_geom_api::Sense) -> Self {
        match s {
            sm_geom_api::Sense::Forward => Sense::Forward,
            sm_geom_api::Sense::Reversed => Sense::Reversed,
        }
    }
}

impl From<Sense> for sm_geom_api::Sense {
    fn from(s: Sense) -> Self {
        match s {
            Sense::Forward => sm_geom_api::Sense::Forward,
            Sense::Reversed => sm_geom_api::Sense::Reversed,
        }
    }
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
    /// `+1.0` for `Forward`, `-1.0` for `Reversed`.
    pub const fn sign(self) -> f64 {
        match self {
            Sense::Forward => 1.0,
            Sense::Reversed => -1.0,
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

/// Stable name of one use of an edge by a face: the face, the edge and the side the loop
/// traverses it on (`Left` = along the edge, `Right` = against it). An edge appears at most
/// once per side in a face's loops, so a periodic seam is `{f, e, Left}` and `{f, e, Right}`.
/// The key does not shift when other coedges of the loop change; when the edge itself is split,
/// the lineage maps `edge -> [e1, e2]` and callers re-key through the successors.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct CoedgeKey {
    /// The face.
    pub face: EntityId,
    /// The edge.
    pub edge: EntityId,
    /// `Left` or `Right`.
    pub side: Side,
}

/// Stable name of a corner (vertex use) of a face loop: the coedge that arrives at it. Unlike a
/// `(face, vertex)` pair it stays unique where a seam vertex appears twice in one loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct CornerKey {
    /// The face.
    pub face: EntityId,
    /// The coedge arriving at the corner.
    pub incoming: CoedgeKey,
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

/// The geometry attached to a cell (GMI-like carrier), resolved through
/// [`crate::TopologyView::shapes`]; topology never owns shape beyond the store.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum Carrier {
    /// No geometry (bodies, topology-only cells, degenerate edges).
    None,
    /// A vertex position.
    Point(Point3),
    /// An edge's curve.
    Curve(CurveCarrier),
    /// A face's surface.
    Surface(SurfaceCarrier),
    /// A volume described as a CSG cell (intersection of half-spaces over stored surfaces).
    Csg(CsgCell),
    /// A virtual face made of host faces (decomposition overlay): the member host faces.
    Composite(SmallVec<[EntityId; 4]>),
}

/// Per-cell flag bits.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CellFlags(pub u8);

impl CellFlags {
    /// No flags.
    pub const NONE: CellFlags = CellFlags(0);
    /// A collapsed edge (zero length) at a sphere pole or cone apex. It has a vertex row but no
    /// curve; its coedges are [`CoedgeKind::Pole`], excluded from Euler counts, shell closure
    /// and radial rings.
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

/// Whether a coedge runs along a real edge or is a degenerate pole coedge.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CoedgeKind {
    /// A coedge of a real (curve-carrying) edge.
    Edge,
    /// A degenerate coedge at a sphere pole / cone apex (zero-length edge).
    Pole,
}

/// Identifier of the engine operation that caused a change. Supplied by the caller (the op
/// engine's op number), so lineage is replay-stable.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OpId(pub u64);

/// Lifecycle state of a cell id. Retired ids always resolve: they keep the operation that
/// retired them, so old references can be followed through the lineage log.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum EntityState {
    /// Alive.
    Live,
    /// Replaced by children in the given operation.
    Split(OpId),
    /// Absorbed into a survivor in the given operation.
    Merged(OpId),
    /// Deleted in the given operation.
    Deleted(OpId),
}

/// What kind of shell a volume ring is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ShellKind {
    /// The closed outer boundary (ring 0). Never holds baffles or wires.
    Outer,
    /// A closed inner boundary (cavity); its faces still have the volume on their left.
    Void,
    /// Baffles and wires inside the volume (`Side::Both` uses, embedded sheets).
    Internal,
}

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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FacePair {
    /// Lower id.
    pub a: EntityId,
    /// Higher id.
    pub b: EntityId,
    /// Status.
    pub status: Coincidence,
    /// The operation that last changed the status.
    pub op: OpId,
}

impl FacePair {
    /// The other member of the pair, if `f` is one of them.
    pub fn other(&self, f: EntityId) -> Option<EntityId> {
        if f == self.a {
            Some(self.b)
        } else if f == self.b {
            Some(self.a)
        } else {
            None
        }
    }
}

/// What a [`LineageKind::Modified`] record changed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum ModKind {
    /// A face's orientation was reversed (loops, sides, carrier sense).
    Reversed,
    /// A carrier was replaced.
    Carrier,
    /// A vertex moved.
    Point,
    /// A curve definition was replaced.
    Curve,
    /// A surface definition was replaced.
    Surface,
    /// A tolerance grew.
    Tolerance,
    /// A coincidence status changed.
    Coincidence,
    /// An inner loop was added or removed.
    InnerLoop,
}

/// What an operation did to an entity. Open for extension (a `Copied` kind may follow; today
/// op-level successors carry copies as `A -> [A, A']`): downstream matches need a wildcard arm.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum LineageKind {
    /// The entity was created by a composite operation that declared it (`record_created`).
    /// Every cell also has [`crate::TopologyView::born`].
    Created,
    /// The entity was replaced by `children` (it is retired, the children are live).
    Split {
        /// The replacements, in the documented creation order.
        children: SmallVec<[EntityId; 2]>,
    },
    /// The entity was absorbed into the survivor `into`; `gap` is the geometric gap that the
    /// merge discarded (an audit trail), in model length units.
    Merged {
        /// The survivor (always the lower id).
        into: EntityId,
        /// The measured gap.
        gap: f64,
    },
    /// The entity was deleted.
    Deleted,
    /// The entity kept its id but changed.
    Modified(ModKind),
    /// The entity (a face) was generated on a tool surface; "the new cut face" is found by this.
    Generated {
        /// The tool surface handle.
        from_surface: EntityId,
    },
}

/// Alias used by the decision log: `LineageEvent::Merged { gap }`.
pub type LineageEvent = LineageKind;

/// One append-only lineage record: "`entity` underwent `kind` in operation `op`".
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LineageRecord {
    /// The operation.
    pub op: OpId,
    /// The subject entity.
    pub entity: EntityId,
    /// What happened to it.
    pub kind: LineageKind,
}

/// Succession of one entity: `old` became `new` (empty `new` = removed). The shape the model
/// layer consumes.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Lineage {
    /// The retired (or kept) entity.
    pub old: EntityId,
    /// Its successors, in creation order.
    pub new: SmallVec<[EntityId; 2]>,
}

/// A face created by a cut on a tool surface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CapFace {
    /// The cap face.
    pub face: EntityId,
    /// The tool surface it was generated from (from `record_generated`).
    pub from_surface: EntityId,
}

/// The result of a topology cut operation: what each retired entity became, and which live
/// faces are caps (generated on a tool surface) in that operation.
#[derive(Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CutOutcome {
    /// Successions, in log order.
    pub lineage: Vec<Lineage>,
    /// Cap faces, in log order.
    pub caps: Vec<CapFace>,
}

/// Input to face construction: one coedge of a loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CoedgeSpec {
    /// The edge.
    pub edge: EntityId,
    /// `Forward` if the loop traverses the edge along its direction.
    pub sense: Sense,
}

/// Input to volume construction: one face use.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FaceUseSpec {
    /// The face.
    pub face: EntityId,
    /// Which side of the face the volume occupies.
    pub side: Side,
}

/// Input to volume construction: one shell.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ShellSpec<'a> {
    /// What the shell is.
    pub kind: ShellKind,
    /// Its face uses.
    pub uses: &'a [FaceUseSpec],
}

/// How a face is cut by [`crate::TopologyEdit::split_face`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum FaceCut<'a> {
    /// An existing open edge joining two distinct vertices of the face's outer loop.
    /// Children: `[first, second]`, `first` being the loop segment from the earlier loop
    /// position of the two chord ends.
    Chord {
        /// The chord edge.
        edge: EntityId,
    },
    /// A closed loop of edges. Its coedge senses give the traversal of the child that bounds it.
    /// The chain must run in the face's own loop direction (counter-clockwise seen from the face's
    /// right side, like `make_face` loops); the orientation is not checked here. A clockwise
    /// chain gives a disk child oriented against its parent and a mis-directed hole loop, which
    /// a negative `face_moments` area exposes.
    /// If the loop has no vertex on the face, it lies in the interior: children are
    /// `[face with the loop as a new inner loop, disk bounded by the loop]` (this is how a
    /// smaller face is imprinted onto a larger one). If it passes through a vertex that appears
    /// twice in the outer loop (a periodic seam vertex) it splits the face into an annulus pair:
    /// children are `[segment from the earlier occurrence + the loop, the rest + reversed loop]`.
    ClosedLoop {
        /// The loop, in traversal order.
        edges: &'a [CoedgeSpec],
    },
    /// An open chain of edges (a polyline imprint, a curved cut made of several edges). The
    /// chain runs head to tail; its start (tail of the first coedge) and end (head of the last)
    /// are two distinct vertices of the face's outer loop (after
    /// [`crate::TopologyEdit::bridge_loops`] every loop of a holed face is one outer loop), and
    /// its interior vertices must not lie on any loop of the face (that they lie inside the face
    /// geometrically is not checked). A one-edge chain is a [`FaceCut::Chord`]. Children:
    /// `[first, second]`, `first` being the loop segment from the earlier loop position of the
    /// two chain ends, as for `Chord`.
    Chain {
        /// The chain, in traversal order.
        edges: &'a [CoedgeSpec],
    },
}

/// Which child gets each existing inner loop when a face is split.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum InnerLoopAssign<'a> {
    /// The face has no inner loops (error otherwise).
    None,
    /// `(old ring, child index 0 or 1)` for every inner loop.
    Explicit(&'a [(u16, u8)]),
    /// Decide by a point-in-face test on the loop's first vertex (needs face geometry).
    ByPointTest,
}

/// Which child gets each non-outer shell when a volume is split.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ShellAssign<'a> {
    /// The volume has only its outer shell (error otherwise).
    None,
    /// `(old ring, child index 0 or 1)` for every shell other than ring 0.
    Explicit(&'a [(u16, u8)]),
}

/// Summary of a committed operation.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OpSummary {
    /// The operation.
    pub op: OpId,
    /// Cells created, ascending by id.
    pub created: Vec<EntityId>,
    /// Cells retired (split, merged or deleted), ascending by id.
    pub retired: Vec<EntityId>,
    /// Index of the first lineage record of the operation.
    pub lineage_start: usize,
    /// Number of lineage records of the operation.
    pub lineage_len: usize,
}

/// The id given to a typed wrapper has the wrong entity kind.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
#[error("{0:?} has the wrong kind for this typed id")]
pub struct WrongKind(pub EntityId);

macro_rules! typed_id {
    ($(#[$m:meta])* $name:ident, $kind:ident) => {
        $(#[$m])*
        #[repr(transparent)]
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
        #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
        #[cfg_attr(feature = "serde", serde(transparent))]
        pub struct $name(EntityId);

        impl $name {
            /// Wrap `id` if it has the right kind.
            pub fn new(id: EntityId) -> Option<Self> {
                (id.kind() == EntityKind::$kind).then_some(Self(id))
            }
            /// The raw id.
            pub const fn get(self) -> EntityId {
                self.0
            }
            /// Wrap an id the caller has already checked (or took from a rank-typed list).
            #[allow(dead_code)]
            pub(crate) const fn trusted(id: EntityId) -> Self {
                Self(id)
            }
        }

        impl From<$name> for EntityId {
            fn from(v: $name) -> EntityId {
                v.0
            }
        }

        impl TryFrom<EntityId> for $name {
            type Error = WrongKind;
            fn try_from(id: EntityId) -> Result<Self, WrongKind> {
                Self::new(id).ok_or(WrongKind(id))
            }
        }
    };
}

typed_id!(
    /// A vertex id (`EntityKind::TopoVertex`), zero-cost over [`EntityId`].
    VertexId,
    TopoVertex
);
typed_id!(
    /// An edge id (`EntityKind::TopoEdge`), zero-cost over [`EntityId`].
    EdgeId,
    TopoEdge
);
typed_id!(
    /// A face id (`EntityKind::TopoFace`), zero-cost over [`EntityId`].
    FaceId,
    TopoFace
);
typed_id!(
    /// A volume id (`EntityKind::TopoVolume`), zero-cost over [`EntityId`].
    VolumeId,
    TopoVolume
);
typed_id!(
    /// A body id (`EntityKind::TopoBody`), zero-cost over [`EntityId`].
    BodyId,
    TopoBody
);
