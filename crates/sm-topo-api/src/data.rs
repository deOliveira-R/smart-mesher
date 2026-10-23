//! Entity records. These are plain data returned by the read-only [`crate::TopologyView`].
//!
//! # Conventions (binding)
//! - A **face normal** is the surface normal (`S_u × S_v`) times `surface_sense`.
//! - An **outer loop** runs counter-clockwise seen from the tip of the face normal (right-hand
//!   rule); **inner loops** run clockwise.
//! - A [`Coedge`] with `Forward` sense traverses its edge from `start` to `end`.
//! - A [`FaceUse`] with `Forward` sense means the face normal points **out of** the volume
//!   (the volume lies behind the face); `Reversed` means the normal points **into** the volume.
//! - The side the normal points to is the face's **front**; the other is its **back**.

use crate::ids::{BodyId, EdgeId, FaceId, Sense, VertexId, VolumeId};
use smallvec::SmallVec;
use sm_core::Point3;
use sm_geom_api::{CurveId, Interval, PCurve, SurfaceId};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A vertex: a point with its own tolerance ball.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct VertexData {
    /// Id.
    pub id: VertexId,
    /// Position.
    pub point: Point3,
    /// Tolerance (>= policy minimum): radius of the ball in which this vertex is "the same place".
    pub tol: f64,
}

/// An edge: a bounded piece of a 3D curve between two vertices (shared by every face using it).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct EdgeData {
    /// Id.
    pub id: EdgeId,
    /// 3D curve; `None` for degenerate edges (collapsed at a pole) or not-yet-geometric edges.
    pub curve: Option<CurveId>,
    /// Parameter range on the curve, traversed from `start` to `end`.
    pub range: Interval,
    /// Start vertex (`C(range.lo)`).
    pub start: VertexId,
    /// End vertex (`C(range.hi)`); equal to `start` for closed edges.
    pub end: VertexId,
    /// Tolerance: max distance between the curve and its vertices/pcurves.
    pub tol: f64,
    /// True if the edge is collapsed to a point (sphere/cone pole).
    pub degenerate: bool,
}

/// One use of an edge by a loop or wire (a *coedge*). The same edge appears once per adjacent
/// face; a seam edge appears twice in the same loop with opposite senses.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Coedge {
    /// The shared edge.
    pub edge: EdgeId,
    /// Traversal direction relative to the edge.
    pub sense: Sense,
    /// The edge in the owning face's `(u, v)` space (same parameter as the 3D curve).
    /// Required for periodic seams on exact surfaces; `None` otherwise.
    pub pcurve: Option<PCurve>,
}

/// Role of a loop on a face.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum LoopKind {
    /// Outer boundary (counter-clockwise about the normal). A face has exactly one, first.
    Outer,
    /// Hole boundary (clockwise about the normal).
    Inner,
}

/// A closed ring of coedges bounding a face.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct LoopData {
    /// Outer or inner.
    pub kind: LoopKind,
    /// Coedges in traversal order; coedge `i` ends where `i + 1` starts (cyclically).
    pub coedges: Vec<Coedge>,
}

/// A face: a region of a surface bounded by loops. Faces are shared between the (up to two)
/// volumes on either side; sides are expressed through [`FaceUse`]s, not stored here.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct FaceData {
    /// Id.
    pub id: FaceId,
    /// Supporting surface; `None` for not-yet-geometric faces.
    pub surface: Option<SurfaceId>,
    /// Whether the face normal equals (`Forward`) or opposes (`Reversed`) the surface normal.
    pub surface_sense: Sense,
    /// Tolerance: max distance between the surface and the face's edges/vertices.
    pub tol: f64,
    /// Loops; the first is the outer loop.
    pub loops: Vec<LoopData>,
    /// Edges lying inside the face but not bounding it (imprinted curves, respect constraints).
    pub embedded_edges: Vec<EdgeId>,
}

/// One use of a face by a volume's shell (an explicit use record).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct FaceUse {
    /// The shared face.
    pub face: FaceId,
    /// `Forward`: the face normal points out of the volume. `Reversed`: into it.
    pub sense: Sense,
}

/// Role of a shell of a volume.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum ShellKind {
    /// Outer boundary of the volume (the first shell). Also holds baffles.
    Outer,
    /// Boundary of a cavity inside the volume.
    Inner,
}

/// A closed collection of face uses bounding (part of) a volume.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ShellData {
    /// Outer or inner.
    pub kind: ShellKind,
    /// Face uses in deterministic (insertion) order.
    pub uses: Vec<FaceUse>,
}

/// A chain of coedges not bounding any face (a curve embedded in a volume or free in a body).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct WireData {
    /// Coedges in order (consecutive coedges share a vertex; need not be closed).
    pub coedges: Vec<Coedge>,
}

/// A volume: a region of space bounded by shells. Volumes are what get colored.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct VolumeData {
    /// Id.
    pub id: VolumeId,
    /// Owning body.
    pub body: BodyId,
    /// Shells; the first is the outer shell.
    pub shells: Vec<ShellData>,
    /// Wires embedded in the volume interior (line sources, respected curves).
    pub wires: Vec<WireData>,
}

/// A body: a connected set of volumes (plus free wires) forming one CAD solid or assembly part.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct BodyData {
    /// Id.
    pub id: BodyId,
    /// Volumes in creation order.
    pub volumes: Vec<VolumeId>,
    /// Wires not belonging to any volume.
    pub free_wires: Vec<WireData>,
}

/// What a face side faces.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Side {
    /// A volume of the complex.
    Volume(VolumeId),
    /// The implicit exterior (complement of all volumes).
    Exterior,
}

/// Both sides of a face. `front` is where the face normal points.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct FaceSides {
    /// Side the normal points into.
    pub front: Side,
    /// Side behind the normal.
    pub back: Side,
}

/// Role of a face in the complex, derived from its sides.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum FaceRole {
    /// No volume uses it (free sheet).
    Free,
    /// One side is a volume, the other the exterior.
    Boundary,
    /// Two different volumes meet at the face.
    Interface,
    /// The same volume is on both sides (internal face / baffle).
    Baffle,
    /// Both sides are exterior but the face has uses, or two uses of equal sense: inconsistent.
    Invalid,
}

/// Address of a coedge owner.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum CoedgeOwner {
    /// A loop of a face.
    Face(FaceId),
    /// A wire of a volume.
    Volume(VolumeId),
    /// A free wire of a body.
    Body(BodyId),
}

/// Stable address of a coedge: `(owner, ring index, position)`. Schemes key per-use attributes
/// (vertex types, intervals) on this instead of on the shared edge/vertex.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CoedgeRef {
    /// Owning face, volume or body.
    pub owner: CoedgeOwner,
    /// Loop index (face) or wire index (volume/body).
    pub ring: u32,
    /// Position within the ring.
    pub index: u32,
}

/// Address of a face use: `(volume, shell index, position)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FaceUseRef {
    /// Volume whose shell holds the use.
    pub volume: VolumeId,
    /// Shell index in the volume.
    pub shell: u32,
    /// Position in the shell.
    pub index: u32,
}

/// Imprint/merge status of a pair of coincident (or overlapping) faces.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum ImprintStatus {
    /// One shared face with two sides. The pair record keeps `(survivor, absorbed)`.
    Merged,
    /// Faces coincide exactly (imprinted) but are still two entities.
    ImprintedNotMerged,
    /// Faces lie on each other but their boundaries are not imprinted onto each other.
    CoincidentUnimprinted,
    /// Faces overlap only partly; `overlap_area` says by how much.
    Overlapping,
}

/// Status record for a pair of faces, normalized so `a < b` (by id).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CoincidenceRecord {
    /// Lower-id face (the survivor when merged).
    pub a: FaceId,
    /// Higher-id face (absorbed when merged).
    pub b: FaceId,
    /// Current status.
    pub status: ImprintStatus,
    /// Area of the common region when known (always for `Overlapping`).
    pub overlap_area: Option<f64>,
}

/// Small id list used by lineage records.
pub type IdList = SmallVec<[sm_core::EntityId; 4]>;
