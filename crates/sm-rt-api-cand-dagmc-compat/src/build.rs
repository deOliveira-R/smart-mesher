//! Build inputs, parameters and the [`Backend`] trait.
//!
//! Three ways in, all lowering to the same [`crate::World`]:
//! * **B-rep** ([`BrepInput`]): faces with the volume on each side (`sm_topo_api::Region`), each
//!   face either exact (a geometry-store surface) or faceted (from `sm-tess`). Sealed by
//!   construction: no `make_watertight` pass is needed, because facets of adjacent faces share
//!   the tessellated edge vertices.
//! * **Facets only** (also [`BrepInput`] with `SurfaceGeometry::Facets`): the DAGMC-style path.
//! * **Volume mesh** ([`MeshInput`]): boundary and inter-volume faces of an `sm-mesh` mesh,
//!   grouped by classification.
//! * **Analytic** ([`crate::AnalyticSpec`]): spheres and boxes, for tests and simple decks.

use crate::analytic::AnalyticSpec;
use crate::boundary::Boundary;
use crate::cache::{CacheBlob, ContentHash};
use crate::error::Result;
use crate::world::World;
use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_geom_api::GeomStore;
use sm_mesh_api::MeshRead;
use sm_topo_api::Region;

/// Arithmetic used inside a backend's hit test.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Precision {
    /// Everything in `f64` (default; the only one with a robustness guarantee).
    #[default]
    F64,
    /// `f32` traversal with `f64` confirmation of every hit (the double-down trick: fast
    /// bounding-volume tests, exact hit distance).
    F32Traversal,
}

/// Tunables of a build. None of them may change results across thread counts.
#[derive(Clone, Debug, PartialEq)]
pub struct BuildParams {
    /// Hit-test arithmetic.
    pub precision: Precision,
    /// Distance under which a point counts as on a surface (relative to the model size).
    pub surface_eps: f64,
    /// Fail the build if open edges are found (faceted input).
    pub require_sealed: bool,
    /// Largest gap between facet edges still treated as sealed, in model units.
    pub seal_tol: f64,
    /// Facets per leaf of the acceleration structure.
    pub leaf_size: u32,
    /// Build threads (`None` = all). Never affects the result.
    pub threads: Option<u32>,
}

impl Default for BuildParams {
    fn default() -> Self {
        Self {
            precision: Precision::F64,
            surface_eps: 1e-9,
            require_sealed: true,
            seal_tol: 1e-9,
            leaf_size: 4,
            threads: None,
        }
    }
}

/// An edge shared by fewer or more than two facets of one volume (a leak path).
#[derive(Clone, Debug, PartialEq)]
pub struct UnsealedEdge {
    /// Surfaces of the two facets (or the one facet) at the edge, by source entity.
    pub surfaces: Vec<EntityId>,
    /// Edge midpoint.
    pub at: Point3,
    /// Gap width (`0.0` for a missing neighbour).
    pub gap: f64,
}

/// What a build found. Actionable: every problem is located and linked to source entities, so
/// the validation tools can show it (DAGMC users only get a lost-particle count at run time).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BuildReport {
    /// Open edges (empty for sealed input).
    pub unsealed_edges: Vec<UnsealedEdge>,
    /// Degenerate facets dropped, by source surface.
    pub degenerate_facets: Vec<(EntityId, u32)>,
    /// Volumes whose signed facet volume disagrees with the sense table (inside-out volumes).
    pub inverted_volumes: Vec<EntityId>,
    /// True if the world came from the cache instead of a build.
    pub from_cache: bool,
}

/// A volume of the input.
#[derive(Clone, Debug, PartialEq)]
pub struct VolumeSpec {
    /// Source volume.
    pub id: EntityId,
    /// Material or group tag (`0` = none).
    pub tag: u32,
}

/// Triangles of one surface (from `sm-tess`), indices local to `xyz`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FacetSoup {
    /// Vertex positions.
    pub xyz: Vec<[f64; 3]>,
    /// Triangles, counter-clockwise seen from the forward side (the normal points out of the
    /// forward volume).
    pub tris: Vec<[u32; 3]>,
}

/// Geometry of one surface.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum SurfaceGeometry {
    /// Triangle facets. Hits are exact on the facets; the facets approximate the surface.
    Facets(FacetSoup),
    /// The exact carrier surface in the geometry store of [`BrepInput::geom`], clipped to the
    /// face by its facets (used for exact analytic ray hits with facets as the trim).
    Exact {
        /// Store handle (`EntityKind::Surface`).
        carrier: EntityId,
        /// Trim facets.
        trim: FacetSoup,
    },
}

/// One surface (face) of the input.
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceSpec {
    /// Source face.
    pub id: EntityId,
    /// Geometry.
    pub geometry: SurfaceGeometry,
    /// Region the normal points out of (`sm_topo_api::Side::Left`).
    pub forward: Region,
    /// Region the normal points into (`Side::Right`).
    pub reverse: Region,
    /// Boundary condition (lowered from the deck by the caller).
    pub boundary: Boundary,
}

/// B-rep input (see module docs).
pub struct BrepInput<'a> {
    /// Volumes, any order; handles are assigned ascending by `id`.
    pub volumes: Vec<VolumeSpec>,
    /// Surfaces, any order; handles are assigned ascending by `id`.
    pub surfaces: Vec<SurfaceSpec>,
    /// Geometry store for `SurfaceGeometry::Exact`.
    pub geom: Option<&'a dyn GeomStore>,
}

/// Volume-mesh input.
pub struct MeshInput<'a> {
    /// The mesh. Volumes are the `Volume` entities cells are classified on; surfaces are the
    /// faces between cells of different volumes or on the domain boundary, grouped by their
    /// classification entity. Polygon faces are split into triangles by a fixed fan from the
    /// lowest vertex, so shared faces triangulate identically.
    pub mesh: &'a dyn MeshRead,
    /// Boundary condition per classification entity (default `Interior`/`Vacuum` for
    /// interior/domain-boundary faces).
    pub boundaries: Vec<(EntityId, Boundary)>,
}

/// One placed copy of a prototype.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InstanceSpec {
    /// Index into [`InstancedInput::prototypes`].
    pub prototype: u32,
    /// Placement (may be improper: mirrored copy).
    pub transform: Transform,
    /// Source instance entity, if any.
    pub id: Option<EntityId>,
}

/// A regular lattice of one prototype (OpenMC-style rectangular lattice, without per-copy
/// storage).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LatticeSpec {
    /// Index into [`InstancedInput::prototypes`].
    pub prototype: u32,
    /// Lower corner of copy `(0,0,0)`.
    pub origin: Point3,
    /// Pitch along x, y, z.
    pub pitch: Vector3,
    /// Copies along x, y, z.
    pub dims: [u32; 3],
}

/// Hierarchy: prototypes placed by instances and lattices. Instances share one acceleration
/// structure per prototype (memory scales with prototypes, not copies); a ray is transformed
/// into each instance's frame. Volume handles are per (instance, prototype volume), allocated
/// in instance order then prototype order.
pub struct InstancedInput<'a> {
    /// Prototype geometries.
    pub prototypes: Vec<BuildInput<'a>>,
    /// Individual placements.
    pub instances: Vec<InstanceSpec>,
    /// Lattices.
    pub lattices: Vec<LatticeSpec>,
}

/// Everything a backend can be built from.
#[non_exhaustive]
pub enum BuildInput<'a> {
    /// B-rep or facets.
    Brep(BrepInput<'a>),
    /// Volume mesh.
    Mesh(MeshInput<'a>),
    /// Analytic solids.
    Analytic(AnalyticSpec),
    /// Instances and lattices of other inputs.
    Instanced(InstancedInput<'a>),
}

impl BuildInput<'_> {
    /// Hash of the input content (geometry, senses, boundaries), independent of ordering and
    /// threads. The cache key is `(input_hash, params_hash, backend name)`.
    pub fn content_hash(&self) -> ContentHash {
        crate::cache::hash_input(self)
    }
}

/// A tracking backend: builds a [`World`] and restores it from the cache.
pub trait Backend: Send + Sync {
    /// Short name (also stored in the cache header).
    fn name(&self) -> &'static str;
    /// Build from scratch.
    fn build(&self, input: &BuildInput<'_>, params: &BuildParams) -> Result<(Box<dyn World>, BuildReport)>;
    /// Restore from a cache blob. The caller has already checked the key with
    /// [`crate::CacheHeader::matches`]; the backend verifies its own section checksums.
    fn restore(&self, input: &BuildInput<'_>, blob: &CacheBlob) -> Result<Box<dyn World>>;
}
