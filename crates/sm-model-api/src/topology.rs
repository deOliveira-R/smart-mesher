//! The narrow, read-only topology interface this layer needs.
//!
//! This trait is the **input to the `sm-topo-api` design**: it lists exactly the topology
//! queries the model-semantics and decomposition layers use, and nothing else. All five
//! queries are plain reads; no mutation is ever requested through it. Topology
//! *mutations* (cutting, merging) come back to this crate as data (see
//! [`crate::symmetry::CutOutcome`]), never as calls.
//!
//! Contract for all implementations: ids are `EntityKind::TopoVolume` / `TopoFace`; lists are
//! returned in ascending id order; unknown ids give empty results (never panic).

use sm_core::{EntityId, Point3, Vector3};
use smallvec::SmallVec;

/// How a volume uses a face (the `FaceUse` sense of docs/ARCHITECTURE.md).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Sense {
    /// The face normal (right-hand rule on its loop) points **out of** the volume.
    Forward,
    /// The face normal points **into** the volume.
    Reverse,
}

impl Sense {
    /// The opposite sense.
    pub const fn flipped(self) -> Self {
        match self {
            Sense::Forward => Sense::Reverse,
            Sense::Reverse => Sense::Forward,
        }
    }

    /// The sense seen after applying a transform; improper transforms reverse orientation.
    pub const fn under(self, improper: bool) -> Self {
        if improper { self.flipped() } else { self }
    }
}

/// One use of a face by a volume.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FaceSide {
    /// The `TopoVolume` using the face.
    pub volume: EntityId,
    /// Orientation of that use.
    pub sense: Sense,
}

/// Exact zeroth/first/second volume moments of a region, in model coordinates.
///
/// From these the layer derives barycentres and inertia tensors for symmetry detection.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Moments {
    /// Volume, `integral 1 dV`.
    pub volume: f64,
    /// `integral x dV`.
    pub first: Vector3,
    /// `integral x x^T dV` (row-major, symmetric).
    pub second: [[f64; 3]; 3],
}

/// Read-only topology queries used by the model layers. Implemented by `sm-topo`
/// (and by [`crate::mock::MockTopology`] in tests).
pub trait TopologyView {
    /// All `TopoVolume` ids, ascending.
    fn volumes(&self) -> Vec<EntityId>;

    /// All `TopoFace` ids, ascending.
    fn faces(&self) -> Vec<EntityId>;

    /// The uses of `face` by volumes, ascending by volume id. Length 1 means a model
    /// boundary face; length 2 an interior (merged) face. Unknown face: empty.
    fn face_sides(&self, face: EntityId) -> SmallVec<[FaceSide; 2]>;

    /// Corner points of the face's outer loop in loop order (vertex positions only; curved
    /// edges are represented by their end vertices). Used for geometric pairing and plane
    /// checks, so it must be deterministic. Unknown face: empty.
    fn face_outline(&self, face: EntityId) -> Vec<Point3>;

    /// Exact volume moments of `volume`. `None` if unknown or not available.
    fn volume_moments(&self, volume: EntityId) -> Option<Moments>;
}

/// Geometric tolerances used by comparisons in this crate.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Tolerance {
    /// Absolute distance tolerance in model units.
    pub linear: f64,
    /// Absolute tolerance on dimensionless quantities (matrix entries, direction cosines).
    pub angular: f64,
}

impl Default for Tolerance {
    fn default() -> Self {
        Self { linear: 1e-8, angular: 1e-9 }
    }
}
