//! Record types of the radial-edge cell complex.
//!
//! ```text
//! Body > Volume > Shell > FaceUse -(mate)- FaceUse
//!                            \             /
//!                             Face (surface, sense) > Loop > Coedge > Edge (curve) > Vertex (point)
//! ```
//!
//! A *face* is geometric: one surface patch bounded by loops. It has exactly two *face uses*
//! (one per [`FaceSide`]); each use is owned by a shell of one volume, or by the exterior.
//! Merged interface faces therefore exist once, with two uses belonging to two different volumes.
//! A *coedge* is the loop's own record of an edge. Seen from a face use it yields an
//! [`EdgeUse`]; all edge uses around an edge form its radial ring.

use crate::ids::*;
use sm_core::{EntityId, Point3};
use sm_geom_api::{PCurve, Sense};

/// A vertex.
#[derive(Clone, Debug, PartialEq)]
pub struct VertexRec {
    /// Position.
    pub point: Point3,
    /// Lifecycle.
    pub state: EntityState,
}

/// An edge: curve segment `[t0, t1]` of a shared curve, running from `start` to `end` in
/// increasing parameter. A closed edge (full circle) has `start == end` and `t1 - t0 == period`.
#[derive(Clone, Debug, PartialEq)]
pub struct EdgeRec {
    /// Curve handle (`EntityKind::Curve`), resolved by a `ShapeStore`.
    pub curve: EntityId,
    /// Start parameter.
    pub t0: f64,
    /// End parameter.
    pub t1: f64,
    /// Vertex at `t0`.
    pub start: VertexId,
    /// Vertex at `t1`.
    pub end: VertexId,
    /// Lifecycle.
    pub state: EntityState,
}

/// Outer boundary or hole.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LoopKind {
    /// Outer boundary: the face lies to the left of travel w.r.t. the face normal.
    Outer,
    /// Hole boundary.
    Inner,
}

/// What a coedge refers to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CoedgeKind {
    /// A real edge, traversed `Forward` (start to end) or `Reversed`.
    Edge {
        /// The edge.
        edge: EdgeId,
        /// Traversal sense relative to the edge.
        sense: Sense,
    },
    /// Degenerate step at a surface pole: the parameter-space boundary collapses to `vertex`.
    /// It is not an edge (no curve, no radial ring) and does not count in Euler formulas.
    Pole {
        /// The pole vertex.
        vertex: VertexId,
    },
}

/// One step of a face loop.
#[derive(Clone, Debug, PartialEq)]
pub struct CoedgeRec {
    /// Edge or pole.
    pub kind: CoedgeKind,
    /// Optional parameter-space curve on the face's surface (required for seams to be unambiguous).
    pub pcurve: Option<PCurve>,
}

/// A closed chain of coedges. Stored in the order seen from the `Negative` side of the face.
#[derive(Clone, Debug, PartialEq)]
pub struct LoopRec {
    /// Outer or inner.
    pub kind: LoopKind,
    /// Coedges in traversal order.
    pub coedges: Vec<CoedgeRec>,
}

/// Owner of a face use.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum UseOwner {
    /// A shell of some volume.
    Shell(ShellRef),
    /// Nothing is modeled on this side.
    Exterior,
}

/// A face use record.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaceUseRec {
    /// How the owning volume sees the face normal. Always equals `side.sense()`; stored so
    /// consumers need not know the side convention. `validate` checks it.
    pub sense: UseSense,
    /// Owner.
    pub owner: UseOwner,
}

/// A face.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceRec {
    /// Surface handle (`EntityKind::Surface`).
    pub surface: EntityId,
    /// Face normal = surface normal if `Forward`, negated if `Reversed`.
    pub surface_sense: Sense,
    /// Loops; at most one `Outer`, listed first. Empty only for closed surfaces without boundary.
    pub loops: Vec<LoopRec>,
    /// The two face uses, indexed by [`FaceSide::index`].
    pub uses: [FaceUseRec; 2],
    /// Lifecycle.
    pub state: EntityState,
}

/// Role of a shell within its volume.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ShellKind {
    /// Outer boundary of the volume.
    Outer,
    /// Boundary of a cavity (void) inside the volume.
    Void,
    /// Free-standing internal sheets, wires and points inside the volume (baffles, internal curves).
    Internal,
}

/// A shell: a connected set of face uses (plus free wires and points for `Internal`).
#[derive(Clone, Debug, PartialEq)]
pub struct ShellRec {
    /// Role.
    pub kind: ShellKind,
    /// Face uses, in insertion order.
    pub face_uses: Vec<FaceUse>,
    /// Wire edges inside the volume.
    pub wires: Vec<EdgeId>,
    /// Isolated vertices inside the volume.
    pub points: Vec<VertexId>,
}

/// A volume.
#[derive(Clone, Debug, PartialEq)]
pub struct VolumeRec {
    /// Owning body.
    pub body: BodyId,
    /// Shells; at most one `Outer`.
    pub shells: Vec<ShellRec>,
    /// Lifecycle.
    pub state: EntityState,
}

/// A body: volumes plus the lower-dimensional parts that belong to no volume.
#[derive(Clone, Debug, PartialEq)]
pub struct BodyRec {
    /// Volumes, ascending.
    pub volumes: Vec<VolumeId>,
    /// Sheet faces (both uses exterior).
    pub sheets: Vec<FaceId>,
    /// Free wire edges (not inside any volume).
    pub wires: Vec<EdgeId>,
    /// Free vertices.
    pub points: Vec<VertexId>,
    /// Lifecycle.
    pub state: EntityState,
}

/// Imprint/merge status of a pair of geometrically related faces (docs/design/ir.md §3).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Coincidence {
    /// Became one shared face (two uses). The record keeps both original ids.
    Merged,
    /// Boundaries match exactly (after imprint) but they are still two faces.
    ImprintedNotMerged,
    /// Geometrically coincident, but their boundaries do not match yet (needs imprint).
    CoincidentUnimprinted,
    /// Partially overlapping (needs imprint to split into coincident and non-coincident parts).
    Overlapping,
}

/// Status record for one face pair. For unmerged statuses `a < b`. For `Merged`, `a` is the
/// survivor and `b` the absorbed (retired) face, in either id order.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoincidenceRecord {
    /// First face (the survivor when `Merged`).
    pub a: FaceId,
    /// Second face (the absorbed face when `Merged`).
    pub b: FaceId,
    /// Status.
    pub status: Coincidence,
    /// Overlap area when known.
    pub overlap_area: Option<f64>,
    /// Op that set the status.
    pub op: OpId,
}

/// Role of a face in the complex.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaceClass {
    /// One side a volume, the other the exterior.
    Boundary(VolumeId),
    /// Two different volumes meet here (merged interface).
    Interface(VolumeId, VolumeId),
    /// Both sides belong to the same volume (baffle / internal sheet).
    Baffle(VolumeId),
    /// Both sides exterior (sheet body).
    Sheet,
}

/// Input for creating an edge.
#[derive(Clone, Debug, PartialEq)]
pub struct EdgeSpec {
    /// Curve handle.
    pub curve: EntityId,
    /// Start parameter.
    pub t0: f64,
    /// End parameter.
    pub t1: f64,
    /// Start vertex.
    pub start: VertexId,
    /// End vertex.
    pub end: VertexId,
}

/// Input for creating a face (both uses start out exterior).
#[derive(Clone, Debug, PartialEq)]
pub struct FaceSpec {
    /// Surface handle.
    pub surface: EntityId,
    /// Normal sense.
    pub surface_sense: Sense,
    /// Loops, outer first.
    pub loops: Vec<LoopRec>,
}

impl CoedgeKind {
    /// The edge, for real edges.
    pub const fn edge(self) -> Option<EdgeId> {
        match self {
            Self::Edge { edge, .. } => Some(edge),
            Self::Pole { .. } => None,
        }
    }
}

/// Deterministic 64-bit FNV-1a hasher used for state hashes (no randomness, platform independent).
#[derive(Clone, Copy, Debug)]
pub struct StableHasher(u64);

impl Default for StableHasher {
    fn default() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
}

impl StableHasher {
    /// Feed bytes.
    pub fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 ^= u64::from(b);
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    /// Feed a `u64`.
    pub fn write_u64(&mut self, v: u64) {
        self.write(&v.to_le_bytes());
    }
    /// Feed a float quantized to a grid of size `quantum` (cross-platform stable, see ARCHITECTURE §3).
    pub fn write_quantized(&mut self, v: f64, quantum: f64) {
        let q = (v / quantum).round();
        // Saturating cast: NaN maps to 0, infinities to the extremes; both are deterministic.
        self.write(&(q as i64).to_le_bytes());
    }
    /// Final hash.
    pub fn finish(&self) -> u64 {
        self.0
    }
}
