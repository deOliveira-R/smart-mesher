//! Build inputs and options: a volume mesh, a facet soup, a B-rep with facets, or a hierarchical
//! world.
//!
//! The preferred input is the volume mesh ([`MeshInput`]): it is the structure the transport
//! solvers already use, so tracking, flux mesh and tallies share cell ids. Facets are for
//! surface-only geometry (no volume mesh yet, or imported DAGMC `.h5m`/STL); a B-rep adds the
//! analytic surfaces so curved hits can be refined to the true surface.
//!
//! Tolerances come from the model ([`sm_geom_api::ModelTolerance`], carried in
//! [`BuildOptions::tol`]), never from constants in the tracker.

use crate::boundary::BoundarySpec;
use crate::cache::{CacheImage, CacheKey, ContentHash, ContentHasher};
use crate::error::{BuildIssue, Result, RtError};
use crate::tracker::Tracker;
use sm_core::{EntityId, Point3};
use sm_geom_api::{GeomStore, ModelTolerance};
use sm_mesh_api::MeshRead;
use sm_topo_api::{Region, TopologyView, TopologyViewExt};
use std::collections::BTreeMap;

/// Floating-point mode of the traversal acceleration structure.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum Precision {
    /// Everything in `f64`.
    #[default]
    F64,
    /// `f32` bounding boxes (expanded outward by the `f32` rounding error) for the traversal,
    /// `f64` for every intersection and confirmation, as in double-down's mixed-precision BVH.
    /// Results must be bit-identical to [`Precision::F64`]; only speed differs. Requires
    /// [`crate::TrackerCaps::F32_TRAVERSAL`]; otherwise the build treats it as `F64`.
    F32Traversal,
}

/// What a tracker does when a particle is lost (no cell contains it, no exit face).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum RecoveryPolicy {
    /// Report `StepEnd::Lost` immediately. Used by validation runs.
    Fail,
    /// Retry up to `tries` times: attempt `k = 0..tries` moves the position **from where the
    /// loss was detected** (not cumulatively) along the direction by `tol.linear * 2^k`
    /// ([`RecoveryPolicy::nudge_distance`]), re-locates with that direction, and continues on
    /// success. Deterministic: the nudge depends only on the state and the attempt number.
    Nudge {
        /// Maximum number of nudges.
        tries: u8,
    },
}

impl RecoveryPolicy {
    /// Number of nudge attempts (`0` for `Fail`).
    pub fn tries(self) -> u32 {
        match self {
            Self::Fail => 0,
            Self::Nudge { tries } => u32::from(tries),
        }
    }
    /// Distance of nudge attempt `k`: `tol.linear * 2^k`, measured from the position where the
    /// loss was detected. The one rule every implementor uses.
    pub fn nudge_distance(tol: &ModelTolerance, k: u32) -> f64 {
        tol.linear * 2f64.powi(k.min(255) as i32)
    }
}

/// How curved faces without an exact analytic carrier are treated.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum FacetFallback {
    /// Use facets only for faces with no implicit/analytic form (NURBS); report them in
    /// [`crate::TrackerInfo::faceted_faces`].
    #[default]
    NoImplicitOnly,
    /// Never facet: a face without an exact form is a build issue.
    Never,
    /// Facet every face (the DAGMC behaviour; for comparisons).
    Always,
}

/// Tunables of a build. Defaults are part of the options hash.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct BuildOptions {
    /// Maximum primitives per BVH leaf.
    pub bvh_leaf_size: u32,
    /// Model tolerance. `tol.linear` is the on-face distance, the nudge unit of
    /// [`RecoveryPolicy::Nudge`] and the coincidence distance; `tol.angular` bounds the
    /// unit-length check of directions.
    pub tol: ModelTolerance,
    /// Planarity tolerance for accepting a mesh face as exactly planar (relative to its diameter).
    /// Faces beyond it are a build issue (`mesh.nonplanar_face`), not silently split.
    pub planarity_tol: f64,
    /// Rays per work block in batch APIs. Independent of the thread count by contract, so the
    /// reduction order (and hence the floating-point result) never depends on it.
    pub block_size: u32,
    /// Lost-particle handling.
    pub recovery: RecoveryPolicy,
    /// Traversal precision.
    pub precision: Precision,
    /// Facet policy.
    pub facets: FacetFallback,
    /// Event cap per `fire` / `trace` call (a loop guard); exceeding it is
    /// `LostCause::MaxStepsExceeded`.
    pub max_events: u32,
}

impl Default for BuildOptions {
    fn default() -> Self {
        Self {
            bvh_leaf_size: 4,
            tol: ModelTolerance::DEFAULT,
            planarity_tol: 1e-9,
            block_size: 256,
            recovery: RecoveryPolicy::Nudge { tries: 3 },
            precision: Precision::F64,
            facets: FacetFallback::NoImplicitOnly,
            max_events: 1_000_000,
        }
    }
}

impl BuildOptions {
    /// Hash for the cache key. Covers every field.
    pub fn content_hash(&self) -> ContentHash {
        let mut h = ContentHasher::new();
        h.write_u64(u64::from(self.bvh_leaf_size));
        for v in [self.tol.linear, self.tol.angular, self.tol.parametric, self.tol.linear_max, self.planarity_tol] {
            h.write_f64(v);
        }
        h.write_u64(u64::from(self.block_size));
        match self.recovery {
            RecoveryPolicy::Fail => h.write_u64(0),
            RecoveryPolicy::Nudge { tries } => {
                h.write_u64(1);
                h.write_u64(u64::from(tries));
            }
        }
        h.write_u64(match self.precision {
            Precision::F64 => 0,
            Precision::F32Traversal => 1,
        });
        h.write_u64(match self.facets {
            FacetFallback::NoImplicitOnly => 0,
            FacetFallback::Never => 1,
            FacetFallback::Always => 2,
        });
        h.write_u64(u64::from(self.max_events));
        h.finish()
    }
}

/// How mesh cells map to regions.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum RegionRule {
    /// Region = the `TopoVolume` the cell is classified on (`MeshRead::cell_class`). Cells
    /// classified on a partition cell use the partition's parent volume.
    ByVolumeClassification,
    /// Region = the named cell set (block); every cell must be in exactly one.
    ByCellSets(Vec<String>),
}

/// What lies outside the cells (DAGMC implicit complement).
#[derive(Clone, Debug, PartialEq, Default)]
#[non_exhaustive]
pub enum ComplementPolicy {
    /// Leaving the mesh leaves the model.
    #[default]
    Outside,
    /// Space between the mesh boundary and the domain box is a named region (e.g. "void").
    Region(String),
}

/// A polyhedral volume mesh to track in.
#[derive(Clone, Copy)]
#[non_exhaustive]
pub struct MeshInput<'a> {
    /// The mesh. `sm_mesh_api::validate` must pass; the builder re-checks only what tracking needs.
    pub mesh: &'a dyn MeshRead,
    /// Cell to region.
    pub regions: &'a RegionRule,
    /// Boundary conditions.
    pub boundaries: &'a BoundarySpec,
    /// Outside policy.
    pub complement: &'a ComplementPolicy,
}

impl<'a> MeshInput<'a> {
    /// Construct (the type is `#[non_exhaustive]`).
    pub fn new(mesh: &'a dyn MeshRead, regions: &'a RegionRule, boundaries: &'a BoundarySpec, complement: &'a ComplementPolicy) -> Self {
        Self { mesh, regions, boundaries, complement }
    }
}

/// Triangles with the regions on both sides. Positions are `f64` (unlike the render
/// `Tessellation`) because tracking precision is a correctness issue, not a display one.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FacetInput {
    /// Vertex positions. Triangles share vertices by index; shared curves must share vertices.
    pub vertices: Vec<[f64; 3]>,
    /// Triangles, counter-clockwise seen from the `back` side: the normal (right-hand rule)
    /// points out of `front` into `back`.
    pub triangles: Vec<[u32; 3]>,
    /// Region the normal points out of (`sm_topo_api::FaceSides::left`; DAGMC forward volume).
    pub front: Vec<Region>,
    /// Region the normal points into (`FaceSides::right`).
    pub back: Vec<Region>,
    /// Source B-rep face of each triangle (becomes the model surface).
    pub face: Vec<EntityId>,
}

/// A leak in a facet boundary: where two surfaces fail to meet. The structured form of the
/// `facet.open_edge` issue, for DAGMC-style `make_watertight` style reports.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct UnsealedEdge {
    /// The volume whose boundary is open.
    pub volume: EntityId,
    /// The B-rep face(s) owning the triangle that uses the edge.
    pub surfaces: Vec<EntityId>,
    /// Midpoint of the open edge.
    pub at: Point3,
    /// Length of the open edge: an upper bound of the opening's extent.
    pub gap: f64,
}

/// Per undirected edge of one volume's boundary: uses low to high, uses high to low, owner face.
#[derive(Clone, Copy)]
struct EdgeUse {
    fwd: u32,
    bwd: u32,
    face: Option<EntityId>,
}

type EdgeUses = BTreeMap<(u32, u32), EdgeUse>;

impl FacetInput {
    /// Check lengths, index ranges and finiteness.
    pub fn validate(&self) -> Result<()> {
        let n = self.triangles.len();
        let mut issues = Vec::new();
        if self.front.len() != n || self.back.len() != n || self.face.len() != n {
            issues.push(BuildIssue::new("facet.column_length", "front/back/face columns must have one entry per triangle"));
        }
        for (i, v) in self.vertices.iter().enumerate() {
            if !v.iter().all(|c| c.is_finite()) {
                issues.push(BuildIssue::new("facet.non_finite_vertex", format!("vertex {i} is not finite")));
            }
        }
        for (i, t) in self.triangles.iter().enumerate() {
            if t.iter().any(|&v| v as usize >= self.vertices.len()) {
                issues.push(BuildIssue::new("facet.index_range", format!("triangle {i} indexes a missing vertex")));
            } else if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
                issues.push(BuildIssue::new("facet.degenerate", format!("triangle {i} repeats a vertex")).at(self.point(t[0])));
            }
        }
        if issues.is_empty() { Ok(()) } else { Err(RtError::Build(issues)) }
    }

    fn point(&self, v: u32) -> Point3 {
        let p = self.vertices[v as usize];
        Point3::new(p[0], p[1], p[2])
    }

    /// Merge vertices with bit-identical coordinates (deterministic: the lowest index wins) and
    /// remap triangles. Returns the number of vertices removed. This is *exact* welding only;
    /// near-misses are a producer bug to fix at the source (shared B-rep edges tessellate to one
    /// polyline), not something tracking papers over with a tolerance.
    ///
    /// Fails with `RtError::Build` (`facet.index_range`) before changing anything if a triangle
    /// indexes a missing vertex.
    pub fn weld_exact(&mut self) -> Result<usize> {
        let nv = self.vertices.len();
        let issues: Vec<BuildIssue> = self
            .triangles
            .iter()
            .enumerate()
            .filter(|(_, t)| t.iter().any(|&v| v as usize >= nv))
            .take(8)
            .map(|(i, _)| BuildIssue::new("facet.index_range", format!("triangle {i} indexes a missing vertex")))
            .collect();
        if !issues.is_empty() {
            return Err(RtError::Build(issues));
        }
        let mut seen: BTreeMap<[u64; 3], u32> = BTreeMap::new();
        let mut map = Vec::with_capacity(self.vertices.len());
        let mut kept = Vec::new();
        for v in &self.vertices {
            let key = [norm_bits(v[0]), norm_bits(v[1]), norm_bits(v[2])];
            let id = *seen.entry(key).or_insert_with(|| {
                kept.push(*v);
                (kept.len() - 1) as u32
            });
            map.push(id);
        }
        let removed = self.vertices.len() - kept.len();
        self.vertices = kept;
        for t in &mut self.triangles {
            for v in t.iter_mut() {
                *v = map[*v as usize];
            }
        }
        Ok(removed)
    }

    /// Content hash of the facets (cache key `topology_hash` for a facet input).
    pub fn content_hash(&self) -> ContentHash {
        let mut h = ContentHasher::new();
        h.write_u64(self.vertices.len() as u64);
        for v in &self.vertices {
            for &c in v {
                h.write_f64(c);
            }
        }
        h.write_u64(self.triangles.len() as u64);
        for t in &self.triangles {
            for &i in t {
                h.write_u64(u64::from(i));
            }
        }
        let reg = |h: &mut ContentHasher, r: &Region| match r {
            Region::Volume(v) => h.write_u64(v.to_raw()),
            Region::Exterior => h.write_u64(0),
        };
        for r in self.front.iter().chain(&self.back) {
            reg(&mut h, r);
        }
        for f in &self.face {
            h.write_u64(f.to_raw());
        }
        h.finish()
    }

    /// Watertightness check per volume: the triangles bounding each volume, oriented outward
    /// from it, must form a closed, consistently oriented 2-manifold (every edge used once in
    /// each direction). This is the check DAGMC runs as a separate `make_watertight` repair;
    /// here it is a located build diagnostic and there is no repair step. Open edges carry the
    /// edge length as `gap`. Output order is deterministic (volume id, then edge vertex pair).
    pub fn check_watertight(&self) -> Vec<BuildIssue> {
        self.scan_edges().into_iter().map(|(i, _)| i).collect()
    }

    /// The open edges of [`Self::check_watertight`] in structured form.
    pub fn unsealed_edges(&self) -> Vec<UnsealedEdge> {
        self.scan_edges().into_iter().filter_map(|(_, u)| u).collect()
    }

    fn scan_edges(&self) -> Vec<(BuildIssue, Option<UnsealedEdge>)> {
        let mut per_region: BTreeMap<EntityId, EdgeUses> = BTreeMap::new();
        for (i, t) in self.triangles.iter().enumerate() {
            let (Some(f), Some(b)) = (self.front.get(i), self.back.get(i)) else { continue };
            if t.iter().any(|&v| v as usize >= self.vertices.len()) {
                continue;
            }
            for (reg, flip) in [(f, false), (b, true)] {
                let Region::Volume(vol) = reg else { continue };
                let mut tri = *t;
                if flip {
                    tri.swap(1, 2);
                }
                let edges = per_region.entry(*vol).or_default();
                for k in 0..3 {
                    let (a, c) = (tri[k], tri[(k + 1) % 3]);
                    let e = edges.entry((a.min(c), a.max(c))).or_insert(EdgeUse { fwd: 0, bwd: 0, face: self.face.get(i).copied() });
                    if a < c { e.fwd += 1 } else { e.bwd += 1 }
                }
            }
        }
        let mut out = Vec::new();
        for (vol, edges) in per_region {
            for ((a, b), e) in edges {
                if (e.fwd, e.bwd) == (1, 1) {
                    continue;
                }
                let (pa, pb) = (self.point(a), self.point(b));
                let mid = Point3::new((pa.x + pb.x) / 2.0, (pa.y + pb.y) / 2.0, (pa.z + pb.z) / 2.0);
                let len = (pb - pa).norm();
                let mut unsealed = None;
                let (code, msg, fix) = if e.fwd + e.bwd == 1 {
                    unsealed = Some(UnsealedEdge { volume: vol, surfaces: e.face.into_iter().collect(), at: mid, gap: len });
                    ("facet.open_edge", "edge belongs to one triangle only (gap in the boundary)", "imprint and merge the touching volumes, or re-tessellate so shared curves use one polyline")
                } else if e.fwd + e.bwd > 2 {
                    ("facet.nonmanifold_edge", "edge shared by more than two triangles of this volume", "check for a baffle or duplicate face")
                } else {
                    ("facet.inconsistent_orientation", "two triangles traverse the edge in the same direction", "flip the face sense in the B-rep or the facet winding")
                };
                let mut issue = BuildIssue::new(code, msg).with_entity(vol).at(mid).suggest(fix);
                if unsealed.is_some() {
                    issue = issue.with_gap(len);
                }
                out.push((issue, unsealed));
            }
        }
        out
    }
}

fn norm_bits(x: f64) -> u64 {
    if x == 0.0 { 0 } else { x.to_bits() }
}

/// How hits on curved B-rep faces are treated.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum SurfaceRefine {
    /// Use the facets as the geometry.
    #[default]
    None,
    /// After a facet hit, re-intersect the analytic surface (`GeomStore` `ray_hits`) near the
    /// hit and report that point and normal. Removes faceting error from chords. The refined hit
    /// is re-validated against the planar cell; if it falls outside, the planar hit is used.
    Analytic,
}

/// A B-rep model plus its facets.
#[derive(Clone, Copy)]
#[non_exhaustive]
pub struct BrepInput<'a> {
    /// Topology.
    pub topo: &'a dyn TopologyView,
    /// Carrier geometry.
    pub geom: &'a dyn GeomStore,
    /// Facets of the faces of `volumes` (from sm-tess, converted to `f64`).
    pub facets: &'a FacetInput,
    /// Curved-hit treatment.
    pub refine: SurfaceRefine,
    /// Boundary conditions.
    pub boundaries: &'a BoundarySpec,
}

impl<'a> BrepInput<'a> {
    /// Construct (the type is `#[non_exhaustive]`).
    pub fn new(topo: &'a dyn TopologyView, geom: &'a dyn GeomStore, facets: &'a FacetInput, refine: SurfaceRefine, boundaries: &'a BoundarySpec) -> Self {
        Self { topo, geom, facets, refine, boundaries }
    }
}

/// Build the `front`/`back` region of a face from the topology: `front = left`, `back = right`
/// (the normal points out of left into right, `docs/design/ir.md` section 6). `None` for a dead or unknown face.
pub fn facet_sides_from_topo(view: &dyn TopologyView, face: EntityId) -> Option<(Region, Region)> {
    view.face_sides(face).map(|s| (s.left, s.right))
}

/// Any buildable input.
#[derive(Clone, Copy)]
#[non_exhaustive]
pub enum BuildInput<'a> {
    /// A volume mesh (mesh-native tracker).
    Mesh(MeshInput<'a>),
    /// Triangles only.
    Facets(&'a FacetInput, &'a BoundarySpec),
    /// B-rep with facets.
    Brep(BrepInput<'a>),
    /// A hierarchical world over previously built trackers.
    World(&'a crate::world::WorldSpec),
}

/// What a build found besides the tracker. Nothing here is fatal (fatal problems are
/// `Err(RtError::Build)`); a clean build has everything empty.
#[derive(Clone, Debug, Default, PartialEq)]
#[non_exhaustive]
pub struct BuildReport {
    /// Warnings and located findings (for example a hierarchy deeper than `LEVEL_INLINE`).
    pub issues: Vec<BuildIssue>,
    /// Leaks found in facet input (never repaired).
    pub unsealed_edges: Vec<UnsealedEdge>,
    /// Indices of facets dropped as degenerate.
    pub degenerate_facets: Vec<u32>,
    /// True when the tracker was restored from a [`CacheImage`].
    pub from_cache: bool,
}

/// Builds trackers from inputs. Implemented once by the real crate (`sm-rt`).
pub trait TrackerBuilder: Send + Sync {
    /// Algorithm version for cache keys.
    fn builder_version(&self) -> u32;
    /// The cache key a build of `input` with `opts` would have.
    fn cache_key(&self, input: &BuildInput<'_>, opts: &BuildOptions) -> Result<CacheKey>;
    /// Build from scratch. Deterministic: same inputs and options give a bit-identical tracker
    /// (same `structure_hash`) for any thread count.
    fn build(&self, input: &BuildInput<'_>, opts: &BuildOptions) -> Result<(Box<dyn Tracker>, BuildReport)>;
    /// Rebuild from a cache image; `Err(CacheStale)` if the key differs. **Contract:** the
    /// result has the same `structure_hash` as `build` of the same inputs, and
    /// `report.from_cache` is true.
    fn load(&self, input: &BuildInput<'_>, opts: &BuildOptions, image: &CacheImage) -> Result<(Box<dyn Tracker>, BuildReport)>;
}

#[cfg(test)]
mod tests {
    #[test]
    fn nudge_rule_is_pinned_and_not_cumulative() {
        let tol = ModelTolerance::DEFAULT;
        let seq: Vec<f64> = (0..5).map(|k| RecoveryPolicy::nudge_distance(&tol, k)).collect();
        let l = tol.linear;
        assert_eq!(seq, vec![l, 2.0 * l, 4.0 * l, 8.0 * l, 16.0 * l]);
        assert_eq!(RecoveryPolicy::Fail.tries(), 0);
        assert_eq!(RecoveryPolicy::Nudge { tries: 3 }.tries(), 3);
        assert!(RecoveryPolicy::nudge_distance(&tol, u32::MAX).is_finite());
    }

    use super::*;

    fn vol(n: u64) -> EntityId {
        EntityId::new(sm_core::EntityKind::TopoVolume, n).unwrap()
    }
    fn face(n: u64) -> EntityId {
        EntityId::new(sm_core::EntityKind::TopoFace, n).unwrap()
    }

    /// Unit tetrahedron, outward normals, front = volume, back = exterior.
    fn tet() -> FacetInput {
        FacetInput {
            vertices: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            triangles: vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
            front: vec![Region::Volume(vol(1)); 4],
            back: vec![Region::Exterior; 4],
            face: (1..=4).map(face).collect(),
        }
    }

    #[test]
    fn closed_tet_is_watertight() {
        let f = tet();
        f.validate().unwrap();
        assert!(f.check_watertight().is_empty() && f.unsealed_edges().is_empty());
        // Convention pin: the right-hand-rule normal of each triangle points out of `front`
        // (the volume) toward `back`, i.e. away from the tet's centroid.
        let c = [0.25, 0.25, 0.25];
        for t in &f.triangles {
            let (a, b, d) = (f.vertices[t[0] as usize], f.vertices[t[1] as usize], f.vertices[t[2] as usize]);
            let (u, w) = ([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [d[0] - a[0], d[1] - a[1], d[2] - a[2]]);
            let n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]];
            let out = n[0] * (a[0] - c[0]) + n[1] * (a[1] - c[1]) + n[2] * (a[2] - c[2]);
            assert!(out > 0.0, "normal must point from front (volume) to back");
        }
    }

    #[test]
    fn missing_triangle_is_located_with_gap_and_surfaces() {
        let mut f = tet();
        f.triangles.pop();
        f.front.pop();
        f.back.pop();
        f.face.pop();
        let issues = f.check_watertight();
        assert_eq!(issues.len(), 3);
        assert!(issues.iter().all(|i| i.code == "facet.open_edge" && i.entity == Some(vol(1)) && i.location.is_some() && i.gap.is_some_and(|g| g > 0.0)));
        // The three open edges are the edges of the missing face (1,2,3): each of length sqrt(2).
        let gaps: Vec<f64> = f.unsealed_edges().iter().map(|u| u.gap).collect();
        assert_eq!(gaps.len(), 3);
        assert!(gaps.iter().all(|g| (g - 2f64.sqrt()).abs() < 1e-15));
        assert!(f.unsealed_edges().iter().all(|u| u.volume == vol(1) && u.surfaces.len() == 1));
    }

    #[test]
    fn flipped_triangle_is_inconsistent() {
        let mut f = tet();
        f.triangles[3] = [1, 3, 2];
        assert!(f.check_watertight().iter().any(|i| i.code == "facet.inconsistent_orientation"));
    }

    #[test]
    fn exact_weld_merges_duplicates_deterministically() {
        let mut f = tet();
        f.vertices.push([0.0, 0.0, -0.0]);
        f.triangles[0][0] = 4;
        assert_eq!(f.weld_exact().unwrap(), 1);
        assert_eq!(f.triangles[0][0], 0);
        assert!(f.check_watertight().is_empty());
    }

    #[test]
    fn weld_rejects_out_of_range_indices_without_changing_anything() {
        let mut f = FacetInput { vertices: vec![[0.0; 3]], triangles: vec![[0, 1, 2]], ..FacetInput::default() };
        let before = f.clone();
        assert!(matches!(f.weld_exact(), Err(RtError::Build(_))));
        assert_eq!(f, before);
    }

    #[test]
    fn validate_rejects_bad_input() {
        let mut f = tet();
        f.triangles[0] = [0, 0, 1];
        assert!(f.validate().is_err());
    }

    #[test]
    fn hashes_cover_every_field() {
        let a = tet();
        let mut b = tet();
        assert_eq!(a.content_hash(), b.content_hash());
        b.vertices[1][0] = 1.0 + 1e-12;
        assert_ne!(a.content_hash(), b.content_hash());
        let o = BuildOptions::default();
        let mut variants = vec![o.content_hash()];
        let mut p = o;
        p.bvh_leaf_size += 1;
        variants.push(p.content_hash());
        let mut p = o;
        p.tol = ModelTolerance::for_extent(100.0);
        variants.push(p.content_hash());
        let mut p = o;
        p.planarity_tol *= 2.0;
        variants.push(p.content_hash());
        let mut p = o;
        p.block_size += 1;
        variants.push(p.content_hash());
        let mut p = o;
        p.recovery = RecoveryPolicy::Fail;
        variants.push(p.content_hash());
        let mut p = o;
        p.recovery = RecoveryPolicy::Nudge { tries: 9 };
        variants.push(p.content_hash());
        let mut p = o;
        p.precision = Precision::F32Traversal;
        variants.push(p.content_hash());
        let mut p = o;
        p.facets = FacetFallback::Never;
        variants.push(p.content_hash());
        let mut p = o;
        p.max_events += 1;
        variants.push(p.content_hash());
        let n = variants.len();
        variants.sort();
        variants.dedup();
        assert_eq!(variants.len(), n, "every option changes the hash");
    }
}
