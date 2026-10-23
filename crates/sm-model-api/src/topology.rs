//! The narrow topology interface this layer needs.
//!
//! [`TopologyView`] lists *exactly* the queries the model-semantics and decomposition layers
//! make. It is an input to the `sm-topo-api` design: anything not listed here is not used.
//! The model never mutates topology. Cutting a model to a chamber is a *view* change (the
//! domain selection in [`crate::Model`]); the physical split of a volume is a topology-layer
//! operation done beforehand.
//!
//! Every list returned is in ascending [`EntityId`] order, whatever the implementor stores.
//! The model relies on it for determinism.

use sm_core::{EntityId, Point3, Vector3};

/// The two volumes a face separates.
///
/// Orientation rule (ir.md §6): the normal points from `left` to `right`; a boundary face has
/// `right == None` and the normal points outward.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaceSides {
    /// Volume behind the normal.
    pub left: Option<EntityId>,
    /// Volume in front of the normal.
    pub right: Option<EntityId>,
}

/// Mass properties of a volume (uniform density; "mass" is geometric).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProps {
    /// Measure (volume).
    pub volume: f64,
    /// Barycentre.
    pub centroid: Point3,
    /// Central second moment tensor `S = integral (x - c)(x - c)^T dV` (symmetric). The inertia
    /// tensor is `tr(S) I - S`. It transforms as `M S M^T` under a linear map `M`.
    pub second_moment: [[f64; 3]; 3],
}

/// Geometric summary of a face.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FaceProps {
    /// Area centroid, projected onto the face if it is curved.
    pub centroid: Point3,
    /// Unit normal at `centroid`, oriented left to right.
    pub normal: Vector3,
    /// Area.
    pub area: f64,
}

/// Read-only topology queries used by the model-semantics and decomposition layers.
pub trait TopologyView {
    /// All volume ids, ascending.
    fn volumes(&self) -> Vec<EntityId>;
    /// All face ids, ascending.
    fn faces(&self) -> Vec<EntityId>;
    /// Faces bounding `volume`, ascending. Empty if the volume is unknown.
    fn volume_faces(&self, volume: EntityId) -> Vec<EntityId>;
    /// The two sides of `face`, or `None` if the face is unknown.
    fn face_sides(&self, face: EntityId) -> Option<FaceSides>;
    /// Mass properties of `volume`, or `None` if unknown.
    fn volume_mass_properties(&self, volume: EntityId) -> Option<MassProps>;
    /// Geometry of `face`, or `None` if unknown.
    fn face_properties(&self, face: EntityId) -> Option<FaceProps>;
}

/// Exact signatures of the [`TopologyView`] queries, as a list for the `sm-topo-api` design.
pub const REQUIRED_QUERIES: [&str; 6] = [
    "fn volumes(&self) -> Vec<EntityId>",
    "fn faces(&self) -> Vec<EntityId>",
    "fn volume_faces(&self, volume: EntityId) -> Vec<EntityId>",
    "fn face_sides(&self, face: EntityId) -> Option<FaceSides>",
    "fn volume_mass_properties(&self, volume: EntityId) -> Option<MassProps>",
    "fn face_properties(&self, face: EntityId) -> Option<FaceProps>",
];
