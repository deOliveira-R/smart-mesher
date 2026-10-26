//! Build inputs: a volume mesh, a facet soup, or a B-rep with facets.
//!
//! The preferred input is the volume mesh ([`MeshInput`]): it is the structure the transport
//! solvers already use, so tracking, flux mesh and tallies share cell ids. Facets are for
//! surface-only geometry (no volume mesh yet, or imported DAGMC `.h5m`/STL); a B-rep adds the
//! analytic surfaces so curved hits can be refined to the true surface.

use crate::error::{BuildIssue, Result, RtError};
use crate::state::BoundaryKind;
use crate::tracker::Tracker;
use sm_core::{EntityId, Point3};
use sm_geom_api::GeomStore;
use sm_mesh_api::MeshRead;
use sm_topo_api::{Region, TopologyView, TopologyViewExt};
use std::collections::BTreeMap;

/// Tunables of a build. Defaults are part of the options hash.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BuildOptions {
    /// Maximum primitives per BVH leaf.
    pub bvh_leaf_size: u32,
    /// Relative tolerance for treating a point as on a face (times the cell diameter).
    pub on_face_tol: f64,
    /// Planarity tolerance for accepting a mesh face as exactly planar (relative to its diameter).
    /// Faces beyond it are a build issue (`mesh.nonplanar_face`), not silently split.
    pub planarity_tol: f64,
    /// Rays per work block in batch APIs. Independent of the thread count by contract, so the
    /// reduction order (and hence the floating-point result) never depends on it.
    pub block_size: u32,
    /// Try nudge-and-relocate before reporting a particle lost.
    pub recover_lost: bool,
}

impl Default for BuildOptions {
    fn default() -> Self {
        Self { bvh_leaf_size: 4, on_face_tol: 1e-12, planarity_tol: 1e-9, block_size: 256, recover_lost: true }
    }
}

impl BuildOptions {
    /// Hash for the cache key.
    pub fn content_hash(&self) -> crate::cache::ContentHash {
        let mut h = crate::cache::ContentHasher::new();
        h.write_u64(u64::from(self.bvh_leaf_size));
        h.write_f64(self.on_face_tol);
        h.write_f64(self.planarity_tol);
        h.write_u64(u64::from(self.block_size));
        h.write_u64(u64::from(self.recover_lost));
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

/// Selects model surfaces for a boundary condition.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum BoundarySelector {
    /// A B-rep face (the mesh boundary faces classified on it).
    Entity(EntityId),
    /// A named face set of the mesh (a sideset).
    FaceSet(String),
}

/// Boundary conditions from the deck.
#[derive(Clone, Debug, PartialEq)]
pub struct BoundarySpec {
    /// Applied to boundary faces no selector claims.
    pub default: BoundaryKind,
    /// Explicit conditions; the first matching selector wins.
    pub rules: Vec<(BoundarySelector, BoundaryKind)>,
}

impl Default for BoundarySpec {
    fn default() -> Self {
        Self { default: BoundaryKind::Vacuum, rules: Vec::new() }
    }
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

/// Triangles with the regions on both sides. Positions are `f64` (unlike the render
/// `Tessellation`) because tracking precision is a correctness issue, not a display one.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FacetInput {
    /// Vertex positions. Triangles share vertices by index; shared curves must share vertices.
    pub vertices: Vec<[f64; 3]>,
    /// Triangles (counter-clockwise seen from the front).
    pub triangles: Vec<[u32; 3]>,
    /// Region on the front (normal side) of each triangle.
    pub front: Vec<Region>,
    /// Region on the back of each triangle.
    pub back: Vec<Region>,
    /// Source B-rep face of each triangle (becomes the model surface).
    pub face: Vec<EntityId>,
}

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
    pub fn weld_exact(&mut self) -> usize {
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
        removed
    }

    /// Watertightness check per volume: the triangles bounding each volume, oriented outward
    /// from it, must form a closed, consistently oriented 2-manifold (every edge used once in
    /// each direction). This is the check DAGMC runs as a separate `make_watertight` repair;
    /// here it is a located build diagnostic and there is no repair step.
    pub fn check_watertight(&self) -> Vec<BuildIssue> {
        let mut per_region: BTreeMap<EntityId, EdgeUses> = BTreeMap::new();
        for (i, t) in self.triangles.iter().enumerate() {
            let (Some(f), Some(b)) = (self.front.get(i), self.back.get(i)) else { continue };
            for (reg, flip) in [(f, false), (b, true)] {
                let Region::Volume(vol) = reg else { continue };
                let mut tri = *t;
                if flip {
                    tri.swap(1, 2);
                }
                let edges = per_region.entry(*vol).or_default();
                for k in 0..3 {
                    let (a, c) = (tri[k], tri[(k + 1) % 3]);
                    let e = edges.entry((a.min(c), a.max(c))).or_default();
                    if a < c { e.0 += 1 } else { e.1 += 1 }
                }
            }
        }
        let mut issues = Vec::new();
        for (vol, edges) in per_region {
            for ((a, b), (fwd, bwd)) in edges {
                if (fwd, bwd) == (1, 1) {
                    continue;
                }
                let (pa, pb) = (self.point(a), self.point(b));
                let mid = Point3::new((pa.x + pb.x) / 2.0, (pa.y + pb.y) / 2.0, (pa.z + pb.z) / 2.0);
                let (code, msg, fix) = if fwd + bwd == 1 {
                    ("facet.open_edge", "edge belongs to one triangle only (gap in the boundary)", "imprint and merge the touching volumes, or re-tessellate so shared curves use one polyline")
                } else if fwd + bwd > 2 {
                    ("facet.nonmanifold_edge", "edge shared by more than two triangles of this volume", "check for a baffle or duplicate face")
                } else {
                    ("facet.inconsistent_orientation", "two triangles traverse the edge in the same direction", "flip the face sense in the B-rep or the facet winding")
                };
                issues.push(BuildIssue::new(code, msg).with_entity(vol).at(mid).suggest(fix));
            }
        }
        issues
    }
}

/// Per undirected edge: (uses low to high, uses high to low).
type EdgeUses = BTreeMap<(u32, u32), (u32, u32)>;

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
    /// hit and report that point and normal. Removes faceting error from chords.
    Analytic,
}

/// A B-rep model plus its facets.
#[derive(Clone, Copy)]
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

/// Build the `front`/`back` region of a face from the topology (the normal points from left to
/// right, `docs/design/ir.md` section 6). `None` for a dead or unknown face.
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

/// Builds trackers from inputs. Implemented once by the real crate (`sm-rt`).
pub trait TrackerBuilder: Send + Sync {
    /// Algorithm version for cache keys.
    fn builder_version(&self) -> u32;
    /// The cache key a build of `input` with `opts` would have.
    fn cache_key(&self, input: &BuildInput<'_>, opts: &BuildOptions) -> Result<crate::cache::CacheKey>;
    /// Build from scratch. Deterministic: same inputs and options give a bit-identical tracker
    /// (same `structure_hash`) for any thread count.
    fn build(&self, input: &BuildInput<'_>, opts: &BuildOptions) -> Result<Box<dyn Tracker>>;
    /// Rebuild from a cache image; `Err(CacheStale)` if the key differs. Must equal `build`.
    fn load(&self, input: &BuildInput<'_>, opts: &BuildOptions, image: &crate::cache::CacheImage) -> Result<Box<dyn Tracker>>;
}

#[cfg(test)]
mod tests {
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
        assert!(f.check_watertight().is_empty());
    }

    #[test]
    fn missing_triangle_is_located() {
        let mut f = tet();
        f.triangles.pop();
        f.front.pop();
        f.back.pop();
        f.face.pop();
        let issues = f.check_watertight();
        assert_eq!(issues.len(), 3);
        assert!(issues.iter().all(|i| i.code == "facet.open_edge" && i.entity == Some(vol(1)) && i.location.is_some()));
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
        assert_eq!(f.weld_exact(), 1);
        assert_eq!(f.triangles[0][0], 0);
        assert!(f.check_watertight().is_empty());
    }

    #[test]
    fn validate_rejects_bad_input() {
        let mut f = tet();
        f.triangles[0] = [0, 0, 1];
        assert!(f.validate().is_err());
    }
}
