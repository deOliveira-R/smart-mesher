//! Build inputs and the builder trait.
//!
//! Three sources, one structure:
//! 1. **B-rep** (preferred): `TopologyView` + optional `Instancing`. Faces keep their exact
//!    carrier; volumes keep their shells; face sides give neighbours; `Carrier::Csg` volumes
//!    need no trimming at all.
//! 2. **Facets**: a [`FacetSoup`] (for example converted from `sm-tess` output, whose `f32`
//!    positions are widened by the converter that lives in the implementor). Each face brings its
//!    sides, so the "next volume after a crossing" is still read, not searched. Used for
//!    imported STL/h5m and for the DAGMC-mode baseline.
//! 3. **Volume mesh**: any `MeshRead`; cells are the regions and `face_left`/`face_right` give
//!    mesh walking. Boundary faces carry classification, hence boundary actions.

use crate::boundary::BoundarySpec;
use crate::cache::{RtCacheBlob, RtCacheKey};
use crate::error::Result;
use crate::tracker::{RtOptions, Tracker};
use sm_core::EntityId;
use sm_mesh_api::{InstancedView, MeshRead};
use sm_model_api::Instancing;
use sm_topo_api::{FaceSides, TopologyView};
use std::ops::Range;

/// One face of a facet soup: a triangle range plus the volumes on its two sides.
#[derive(Clone, Debug, PartialEq)]
pub struct FacetFace {
    /// The face id (reported in events).
    pub face: EntityId,
    /// Triangles of this face: `indices[3 * r.start .. 3 * r.end]`.
    pub triangles: Range<u32>,
    /// Region the triangle normals point away from / into.
    pub sides: FaceSides,
}

/// Indexed triangle soup with per-face ranges. **Watertight by construction is required**: faces
/// that meet along an edge must share vertex indices there (as `sm-tess` guarantees). The
/// implementor verifies this and reports open or non-manifold edges in [`BuildReport`]; it never
/// runs a make-watertight repair (DAGMC's `make_watertight` mutates the mesh; we reject and
/// report with face ids instead).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FacetSoup {
    /// `x, y, z` per vertex, double precision.
    pub positions: Vec<f64>,
    /// Three vertex indices per triangle, counter-clockwise about the face normal.
    pub indices: Vec<u32>,
    /// Faces, ascending by id, with disjoint consecutive triangle ranges.
    pub faces: Vec<FacetFace>,
}

/// The geometry source of a build.
#[non_exhaustive]
pub enum Geometry<'a> {
    /// Exact B-rep, optionally with a lattice/instance hierarchy over its volumes. Prototypes
    /// are the `Instancing` prototypes; one structure is built per prototype and shared.
    Brep {
        /// The topology and its shape store.
        topo: &'a dyn TopologyView,
        /// Hierarchy; `None` = the volumes of `topo` are the whole model.
        instancing: Option<&'a Instancing>,
        /// Hash of the hierarchy (`Model::canonical_visit` words), for the cache key.
        hierarchy_hash: u64,
    },
    /// Facets with explicit sides.
    Facets {
        /// The soup.
        soup: &'a FacetSoup,
        /// Hash of the soup, for the cache key.
        soup_hash: u64,
    },
    /// A volume mesh with optional placements of prototype meshes.
    Mesh {
        /// The mesh.
        mesh: &'a dyn MeshRead,
        /// Instanced view over catalog meshes, if any.
        instances: Option<&'a InstancedView>,
        /// Mesh connectivity hash, for the cache key.
        mesh_hash: u64,
    },
}

/// Everything a build needs.
#[non_exhaustive]
pub struct BuildInput<'a> {
    /// Geometry.
    pub geometry: Geometry<'a>,
    /// Boundary actions.
    pub boundaries: &'a BoundarySpec,
    /// Options.
    pub options: RtOptions,
}

impl<'a> BuildInput<'a> {
    /// Assemble an input.
    pub fn new(geometry: Geometry<'a>, boundaries: &'a BoundarySpec, options: RtOptions) -> Self {
        Self { geometry, boundaries, options }
    }

    /// The cache key this input maps to for `builder_version`.
    pub fn cache_key(&self, builder_version: u32) -> RtCacheKey {
        let (t, h) = match &self.geometry {
            Geometry::Brep { topo, hierarchy_hash, .. } => (topo.canonical_hash(), *hierarchy_hash),
            Geometry::Facets { soup_hash, .. } => (*soup_hash, 0),
            Geometry::Mesh { mesh_hash, .. } => (*mesh_hash, 0),
        };
        RtCacheKey::new(t, h, self.boundaries, &self.options, builder_version)
    }
}

/// Kinds of build finding. Stable codes for MCP/CLI diagnostics (located and linked to geometry).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[non_exhaustive]
pub enum BuildIssueCode {
    /// Facet edge used by one triangle only (a gap): the DAGMC lost-particle source.
    OpenEdge,
    /// Facet edge used by more than two triangles.
    NonManifoldEdge,
    /// A face's sides disagree with the volumes' shells (orientation or missing neighbour).
    SideMismatch,
    /// Two volumes overlap (a sampled point is inside both); `face` names a face in the overlap.
    Overlap,
    /// A face fell back to facets (information).
    FacetedFace,
    /// A deck mate does not map onto its partner within tolerance.
    DeckMismatch,
}

/// One finding with its geometric link.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct BuildIssue {
    /// Code.
    pub code: BuildIssueCode,
    /// Face the finding is on, if any.
    pub face: Option<EntityId>,
    /// Volume the finding is in, if any.
    pub volume: Option<EntityId>,
    /// Where, in the world frame.
    pub at: Option<sm_core::Point3>,
    /// Human text, with a suggested fix.
    pub message: String,
}

/// What a build learned. Errors that make tracking unsound (`OpenEdge`, `SideMismatch`) fail
/// the build under `RecoveryPolicy::Fail` and are returned here otherwise.
#[derive(Clone, Debug, Default, PartialEq)]
#[non_exhaustive]
pub struct BuildReport {
    /// Findings, sorted by `(code, face, volume)`.
    pub issues: Vec<BuildIssue>,
}

/// Builds trackers. Implemented by `sm-rt`; the contract tests run against any implementor.
pub trait TrackerBuilder: Send + Sync {
    /// Build the structure. Deterministic: same input, same structure bytes, for any thread
    /// count (parallel construction merges by fixed order).
    fn build(&self, input: &BuildInput<'_>) -> Result<(Box<dyn Tracker>, BuildReport)>;

    /// Version stamped into cache keys.
    fn builder_version(&self) -> u32;

    /// Serialize a tracker this builder produced.
    fn export(&self, tracker: &dyn Tracker) -> Result<RtCacheBlob>;

    /// Rebuild a tracker from a blob; `RtError::Cache` if the blob's key differs from
    /// `input.cache_key(self.builder_version())`, which the caller answers with [`Self::build`].
    fn restore(&self, input: &BuildInput<'_>, blob: &RtCacheBlob) -> Result<Box<dyn Tracker>>;
}
