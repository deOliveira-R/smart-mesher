//! The sm-rt rows: adapters that build the benchmark's cases into sm-rt trackers.
//!
//! | Row mode | Tracker | Geometry |
//! |---|---|---|
//! | `facets` | `MeshTracker` over the facet twin | triangles at the case's `sag` (what DAGMC and XDG hold) |
//! | `analytic` | `AnalyticTracker` | exact planes, spheres and cylinders; P2 through `sm_rt::csg_world` (RT-BRIDGE) |
//! | `hierarchy` | `WorldTracker` | P2: one pin prototype (facets) in the model's lattice and cluster |
//! | `surface`, `delta`, `block` | `MeshTracker` with a majorant partition | a tet mesh (H1's own, or a stair-stepped Kuhn twin of the scene) |
//!
//! Region ids: tracker regions are mapped back to the benchmark's ids (`-1` is the complement), by
//! model entity for facets and meshes, by name for CSG cells.

use crate::adapter::{Adapter, FlightMode, TrackerAdapter};
use crate::cases::{CaseData, PIN_H, PITCH, R_CLAD, R_FUEL, Scale};
use crate::error::{BenchError, Result};
use crate::formats::{Facets, RegionSpec, Tets};
use crate::truth::{Scene, Shape, Solid};
use sm_core::{Aabb, EntityId, EntityKind, Point3, Vector3};
use sm_geom_api::{CsgCell, Cylinder, Frame, GeomAppend, HalfSpace, MemoryStore, Plane, Sign, Sphere, SurfaceDef};
use sm_mesh_api::{CellShape, SerialAlloc};
use sm_rt::{AnalyticTrackerBuilder, MeshTrackerBuilder};
use sm_rt_api::{
    BoundarySpec, BuildInput, BuildOptions, ComplementPolicy, ComponentId, ComponentKind, CsgCellDef, CsgLeafInput, CsgRegion, FacetInput, HierInput, InstancePolicy, LeafInput, MeshInput,
    PartitionOptions, RegionRule, RegionTable, Tracker, WorldSpec,
};
use sm_testkit::meshes::{CellSpec, build_mesh_volumes};
use sm_topo_api::Region;

fn bad<E: core::fmt::Display>(e: E) -> BenchError {
    BenchError::Tracker(e.to_string())
}

fn volume_id(region: i32) -> Result<EntityId> {
    EntityId::new(EntityKind::TopoVolume, u64::try_from(region + 1).map_err(|_| BenchError::Geometry("negative region id".into()))?).ok_or_else(|| BenchError::Geometry("region id out of range".into()))
}

fn region_of(r: i32) -> Result<Region> {
    if r < 0 { Ok(Region::Exterior) } else { Ok(Region::Volume(volume_id(r)?)) }
}

/// Region table to benchmark ids by model entity (`TopoVolume` serial minus one); the implicit
/// complement is `-1`.
fn map_by_entity(regions: &RegionTable) -> Vec<i32> {
    regions.0.iter().map(|r| if r.implicit_complement { -1 } else { r.entity.map_or(-1, |e| e.serial() as i32 - 1) }).collect()
}

/// Region table to benchmark ids by region name.
fn map_by_name(regions: &RegionTable, specs: &[RegionSpec]) -> Vec<i32> {
    regions.0.iter().map(|r| if r.implicit_complement { -1 } else { specs.iter().find(|s| s.name == r.name).map_or(-1, |s| s.id) }).collect()
}

/// True when triangle `t` of `f` has (numerically) no area: two corners coincide. The coarse
/// hand-built moderator strip of the smallest scale has a few such triangles.
fn zero_area(f: &Facets, t: [u32; 3]) -> bool {
    let p = |i: u32| f.verts[i as usize];
    let (a, b, c) = (p(t[0]), p(t[1]), p(t[2]));
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    n[0] * n[0] + n[1] * n[1] + n[2] * n[2] == 0.0
}

/// The facet twin as a tracker input: one B-rep face per distinct `(front, back)` pair. Triangles
/// with zero area (two coincident corners) are dropped: they cover no surface and the tracker
/// refuses them; the case files themselves are not touched.
///
/// # Errors
/// [`BenchError::Geometry`] for a region id that is not representable.
pub fn facet_input(f: &Facets) -> Result<FacetInput> {
    let mut out = FacetInput::default();
    out.vertices = f.verts.clone();
    out.triangles = f.tris.clone();
    let mut faces: Vec<[i32; 2]> = Vec::new();
    out.triangles.clear();
    for (s, &t) in f.sides.iter().zip(&f.tris) {
        if zero_area(f, t) {
            continue;
        }
        out.triangles.push(t);
        out.front.push(region_of(s[0])?);
        out.back.push(region_of(s[1])?);
        let k = match faces.iter().position(|x| x == s) {
            Some(k) => k,
            None => {
                faces.push(*s);
                faces.len() - 1
            }
        };
        out.face.push(EntityId::new(EntityKind::TopoFace, k as u64 + 1).ok_or_else(|| BenchError::Geometry("face id out of range".into()))?);
    }
    Ok(out)
}

/// Cell-walking tracker over the facet twin (`facets` rows).
///
/// # Errors
/// The tracker's build error (open shells, bad orientation, ...).
pub fn facets_adapter(data: &CaseData) -> Result<TrackerAdapter> {
    let fi = facet_input(&data.facets)?;
    let spec = BoundarySpec::default();
    let (t, _) = MeshTrackerBuilder.build_concrete(&BuildInput::Facets(&fi, &spec), &BuildOptions::default()).map_err(bad)?;
    let map = map_by_entity(t.regions());
    Ok(TrackerAdapter::new("smrt", Box::new(t)).with_mode("facets").with_region_map(map).on_twin())
}

// ---- analytic -------------------------------------------------------------------------------

fn pt(a: [f64; 3]) -> Point3 {
    Point3::new(a[0], a[1], a[2])
}

struct CsgBuild {
    store: MemoryStore,
    planes: Vec<([u64; 3], [u64; 3], EntityId)>,
}

impl CsgBuild {
    fn plane(&mut self, o: [f64; 3], n: [f64; 3]) -> Result<EntityId> {
        let key = (o.map(f64::to_bits), n.map(f64::to_bits));
        if let Some(p) = self.planes.iter().find(|p| (p.0, p.1) == key) {
            return Ok(p.2);
        }
        let f = Frame::from_z(pt(o), Vector3::new(n[0], n[1], n[2])).map_err(bad)?;
        let id = self.store.add_surface(SurfaceDef::Plane(Plane { frame: f })).map_err(bad)?;
        self.planes.push((key.0, key.1, id));
        Ok(id)
    }
}

/// Surfaces of one solid and its inside as one intersection.
struct SolidCsg {
    inside: Vec<HalfSpace>,
    /// Half-spaces that exclude the solid from a parent (a single one: curved solids only).
    outside: Option<HalfSpace>,
    z: Option<(f64, f64)>,
}

fn hs(surface: EntityId, negative: bool) -> HalfSpace {
    HalfSpace { surface, sign: if negative { Sign::Negative } else { Sign::Positive } }
}

fn solid_csg(b: &mut CsgBuild, s: &Solid) -> Result<SolidCsg> {
    match s.shape {
        Shape::Sphere { c, r } => {
            let id = b.store.add_surface(SurfaceDef::Sphere(Sphere::new(Frame::at(pt(c)), r).map_err(bad)?)).map_err(bad)?;
            Ok(SolidCsg { inside: vec![hs(id, true)], outside: Some(hs(id, false)), z: None })
        }
        Shape::Cyl { c, r, z0, z1 } => {
            let f = Frame::from_z(Point3::new(c[0], c[1], 0.0), Vector3::new(0.0, 0.0, 1.0)).map_err(bad)?;
            let id = b.store.add_surface(SurfaceDef::Cylinder(Cylinder::new(f, r).map_err(bad)?)).map_err(bad)?;
            let lo = b.plane([0.0, 0.0, z0], [0.0, 0.0, -1.0])?;
            let hi = b.plane([0.0, 0.0, z1], [0.0, 0.0, 1.0])?;
            Ok(SolidCsg { inside: vec![hs(id, true), hs(lo, true), hs(hi, true)], outside: Some(hs(id, false)), z: Some((z0, z1)) })
        }
        Shape::Box { lo, hi } => {
            let mut inside = Vec::new();
            for a in 0..3 {
                let mut n = [0.0; 3];
                n[a] = -1.0;
                inside.push(hs(b.plane(lo, n)?, true));
                n[a] = 1.0;
                inside.push(hs(b.plane(hi, n)?, true));
            }
            Ok(SolidCsg { inside, outside: None, z: Some((lo[2], hi[2])) })
        }
    }
}

fn reference_point(s: &Solid) -> [f64; 3] {
    match s.shape {
        Shape::Sphere { c, .. } => c,
        Shape::Cyl { c, z0, z1, .. } => [c[0], c[1], 0.5 * (z0 + z1)],
        Shape::Box { lo, hi } => [0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (lo[2] + hi[2])],
    }
}

/// The exact analytic tracker of a scene of nested boxes, spheres and vertical cylinders: each
/// solid is a cell, minus the solids one level inside it. Children must be spheres or cylinders
/// spanning the parent's z range (their complement is then one half-space).
///
/// # Errors
/// [`BenchError::Unavailable`] for a child the half-space algebra cannot exclude (a box), or
/// the tracker's build error.
pub fn analytic_from_scene(scene: &Scene, regions: &[RegionSpec]) -> Result<TrackerAdapter> {
    let mut b = CsgBuild { store: MemoryStore::new(), planes: Vec::new() };
    let csg: Vec<SolidCsg> = scene.solids.iter().map(|s| solid_csg(&mut b, s)).collect::<Result<_>>()?;
    let mut cells: Vec<CsgCellDef> = Vec::new();
    for (i, s) in scene.solids.iter().enumerate() {
        let mut hsp = csg[i].inside.clone();
        for (j, c) in scene.solids.iter().enumerate() {
            if c.depth != s.depth + 1 || !s.contains(reference_point(c)) {
                continue;
            }
            let Some(out) = csg[j].outside else {
                return Err(BenchError::Unavailable("a box inside a solid needs a union of half-spaces; the analytic row covers spheres and cylinders inside solids".into()));
            };
            if let (Shape::Cyl { .. }, Some(pz), Some(cz)) = (c.shape, csg[i].z, csg[j].z)
                && pz != cz
            {
                return Err(BenchError::Unavailable("a cylinder inside a solid must span the parent's z range".into()));
            }
            hsp.push(out);
        }
        let name = regions.iter().find(|r| r.id == s.region).map(|r| r.name.clone()).ok_or_else(|| BenchError::Geometry(format!("solid region {} has no region spec", s.region)))?;
        cells.push(CsgCellDef::new(CsgRegion::new(vec![CsgCell::new(hsp)]), name));
    }
    let boundaries = BoundarySpec::default();
    let complement = ComplementPolicy::Outside;
    let t = {
        let spec = WorldSpec::new(vec![ComponentKind::Leaf { input: 0 }], ComponentId(0), InstancePolicy::Auto);
        let leaf = CsgLeafInput::new(&b.store, &cells, None::<Aabb>, &complement);
        let leaves = [LeafInput::Csg(leaf)];
        let h = HierInput::new(&spec, &leaves, &boundaries);
        AnalyticTrackerBuilder.build_concrete(&BuildInput::Hier(h), &BuildOptions::default()).map_err(bad)?.0
    };
    let map = map_by_name(t.regions(), regions);
    Ok(TrackerAdapter::new("smrt", Box::new(t)).with_mode("analytic").with_region_map(map))
}

fn p2_fixture(scale: &Scale) -> Result<sm_testkit::topo::ModelFixture> {
    sm_testkit::topo::pin_assembly(scale.lattice_n, PITCH, R_FUEL, R_CLAD, PIN_H).map_err(|e| BenchError::Geometry(e.to_string()))
}

/// P2 through RT-BRIDGE: the model is bridged to a CSG world (`sm_rt::csg_world`), flattened and
/// built as the analytic tracker. Region names are the model's color names (`fuel`, `clad`).
///
/// # Errors
/// The bridge's or the builder's error.
pub fn analytic_p2_bridge(scale: &Scale, regions: &[RegionSpec]) -> Result<TrackerAdapter> {
    let fix = p2_fixture(scale)?;
    let w = sm_rt::csg_world(&fix.model, &fix.complex, &sm_rt::CsgWorldOptions::default()).map_err(bad)?;
    let (t, _) = w.analytic_tracker(&BoundarySpec::default(), &BuildOptions::default(), sm_rt::MAX_FLAT_CELLS).map_err(bad)?;
    let map = map_by_name(t.regions(), regions);
    Ok(TrackerAdapter::new("smrt", Box::new(t)).with_mode("analytic").with_region_map(map))
}

/// The deck of case A17 as the analytic tracker, with the deck's own boundaries (reflective sides).
///
/// # Errors
/// [`BenchError::Unavailable`] when the corpus is not built, or the import's or builder's error.
pub fn analytic_deck(regions: &[RegionSpec]) -> Result<TrackerAdapter> {
    let deck = crate::cases::a17_deck()?;
    let (t, _) = deck.tracker(&BuildOptions::default()).map_err(bad)?;
    let map = map_by_name(t.regions(), regions);
    Ok(TrackerAdapter::new("smrt", Box::new(t)).with_mode("analytic").with_region_map(map))
}

/// The analytic row of a case: P2 through the bridge, everything else from its scene.
///
/// # Errors
/// [`BenchError::Unavailable`] when the case has no analytic scene or one the half-space algebra
/// cannot express.
pub fn analytic_adapter(data: &CaseData, scale: &Scale) -> Result<TrackerAdapter> {
    if data.case == "A17" {
        return analytic_deck(&data.manifest.regions);
    }
    if data.case == "P2" {
        return analytic_p2_bridge(scale, &data.manifest.regions);
    }
    let scene = data.scene.as_ref().ok_or_else(|| BenchError::Unavailable("the case has no analytic scene".into()))?;
    analytic_from_scene(scene, &data.manifest.regions)
}

// ---- hierarchy (P2) -------------------------------------------------------------------------

/// P2 as a hierarchical world: one pin prototype (the facet twin of the base pin, fuel 0, clad 1,
/// cladding against the complement) placed by the model's own lattice and cluster
/// (`sm_rt::csg_world(..).spec()`), built by `MeshTrackerBuilder::build_world`.
///
/// # Errors
/// The bridge's or the world builder's error.
pub fn hierarchy_p2(scale: &Scale, regions: &[RegionSpec]) -> Result<TrackerAdapter> {
    let fix = p2_fixture(scale)?;
    let tw = crate::twin::pin_twin(&fix, scale.sag_pin, R_CLAD, PIN_H)?;
    let mut base = tw.facets;
    for s in &mut base.sides {
        if *s == [1, 2] {
            *s = [1, -1];
        }
    }
    let w = sm_rt::csg_world(&fix.model, &fix.complex, &sm_rt::CsgWorldOptions::default()).map_err(bad)?;
    if w.leaf_count() != 1 {
        return Err(BenchError::Unavailable("P2 hierarchy expects one pin prototype".into()));
    }
    let fi = facet_input(&base)?;
    let leaves = [LeafInput::Facets(&fi)];
    let boundaries = BoundarySpec::default();
    let h = HierInput::new(w.spec(), &leaves, &boundaries);
    let (t, _) = MeshTrackerBuilder.build_world(&BuildInput::Hier(h), &BuildOptions::default()).map_err(bad)?;
    let table = t.regions();
    let map = if table.0.iter().any(|r| r.entity.is_some()) { map_by_entity(table) } else { map_by_name(table, regions) };
    Ok(TrackerAdapter::new("smrt", Box::new(t)).with_mode("hierarchy").with_region_map(map).counting_touched().on_twin())
}

/// P2 as a hierarchical analytic world (W4-RT-WORLD): the model's lattice and cluster hosting
/// one exact CSG pin prototype, built by `CsgWorld::world_tracker` with no flattening. It is the
/// `analytic-hierarchy` row; the flattened `analytic` row is its oracle.
///
/// # Errors
/// The bridge's or the world builder's error.
pub fn analytic_hierarchy_p2(scale: &Scale, regions: &[RegionSpec]) -> Result<TrackerAdapter> {
    let fix = p2_fixture(scale)?;
    let w = sm_rt::csg_world(&fix.model, &fix.complex, &sm_rt::CsgWorldOptions::default()).map_err(bad)?;
    let (t, _) = w.world_tracker(&BoundarySpec::default(), &BuildOptions::default()).map_err(bad)?;
    let map = map_by_name(t.regions(), regions);
    Ok(TrackerAdapter::new("smrt", Box::new(t)).with_mode("analytic-hierarchy").with_region_map(map).counting_touched())
}

/// The model's lattice and cluster filled with the moderated pin cell of P1 (fuel, clad, water in
/// a square cell of one pitch), so that rays cross lattice elements. P2's own twin has no
/// moderator (pins stand in vacuum), so a ray there never leaves its first pin; this world is
/// what the elements-touched counter is checked on.
///
/// # Errors
/// The twin's, the bridge's or the world builder's error.
pub fn hierarchy_moderated(scale: &Scale) -> Result<TrackerAdapter> {
    let fix1 = sm_testkit::topo::pin_assembly(1, PITCH, R_FUEL, R_CLAD, PIN_H).map_err(|e| BenchError::Geometry(e.to_string()))?;
    let mut tw = crate::twin::pin_twin(&fix1, scale.sag_pin, R_CLAD, PIN_H)?;
    crate::twin::add_moderator(&mut tw, 0.5 * PITCH, PIN_H, 2)?;
    let fix = p2_fixture(scale)?;
    let w = sm_rt::csg_world(&fix.model, &fix.complex, &sm_rt::CsgWorldOptions::default()).map_err(bad)?;
    let fi = facet_input(&tw.facets)?;
    let leaves = [LeafInput::Facets(&fi)];
    let boundaries = BoundarySpec::default();
    let h = HierInput::new(w.spec(), &leaves, &boundaries);
    let (t, _) = MeshTrackerBuilder.build_world(&BuildInput::Hier(h), &BuildOptions::default()).map_err(bad)?;
    let map = map_by_entity(t.regions());
    Ok(TrackerAdapter::new("smrt", Box::new(t)).with_mode("hierarchy-moderated").with_region_map(map).counting_touched().on_twin())
}

// ---- tet meshes and flights -----------------------------------------------------------------

/// A Kuhn tet mesh of the box `lo..hi` with `n` cells per axis, six tets per cell; a tet's region
/// is `region_of(centroid)` (stair-stepped across curved boundaries).
///
/// # Errors
/// [`BenchError::Geometry`] when the mesh would not fit in `u32` indices.
pub fn kuhn_box(lo: [f64; 3], hi: [f64; 3], n: [usize; 3], region_of: &dyn Fn([f64; 3]) -> i32) -> Result<Tets> {
    let np = [n[0] + 1, n[1] + 1, n[2] + 1];
    let h: [f64; 3] = core::array::from_fn(|a| (hi[a] - lo[a]) / n[a] as f64);
    if np[0].saturating_mul(np[1]).saturating_mul(np[2]) > u32::MAX as usize || n.contains(&0) {
        return Err(BenchError::Geometry("tet mesh size".into()));
    }
    let node = |i: usize, j: usize, k: usize| (i + np[0] * (j + np[1] * k)) as u32;
    let mut t = Tets::default();
    for k in 0..np[2] {
        for j in 0..np[1] {
            for i in 0..np[0] {
                t.nodes.push([lo[0] + h[0] * i as f64, lo[1] + h[1] * j as f64, lo[2] + h[2] * k as f64]);
            }
        }
    }
    const PERMS: [[usize; 3]; 6] = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]];
    for k in 0..n[2] {
        for j in 0..n[1] {
            for i in 0..n[0] {
                for (pi, perm) in PERMS.iter().enumerate() {
                    let mut c = [i, j, k];
                    let mut ids = [node(c[0], c[1], c[2]); 4];
                    for (s, &ax) in perm.iter().enumerate() {
                        c[ax] += 1;
                        ids[s + 1] = node(c[0], c[1], c[2]);
                    }
                    if pi == 1 || pi == 2 || pi == 5 {
                        ids.swap(2, 3);
                    }
                    let cen: [f64; 3] = core::array::from_fn(|a| ids.iter().map(|&v| t.nodes[v as usize][a]).sum::<f64>() / 4.0);
                    t.tets.push(ids);
                    t.region.push(region_of(cen));
                }
            }
        }
    }
    Ok(t)
}

/// The tet twin of a case for the delta and block rows: H1's own mesh, else a stair-stepped
/// Kuhn mesh of the scene's bounding box with about `cells` cells along the longest axis
/// (regions by the analytic scene at each tet centroid). `None` when the case has neither.
///
/// # Errors
/// [`BenchError::Geometry`] when the scene leaves part of its box in the complement.
pub fn tet_twin(data: &CaseData, cells: usize) -> Result<Option<Tets>> {
    if let Some(t) = &data.tets {
        return Ok(Some(t.clone()));
    }
    let Some(scene) = &data.scene else { return Ok(None) };
    let (lo, hi) = (data.manifest.bbox_lo, data.manifest.bbox_hi);
    let ext: [f64; 3] = core::array::from_fn(|a| hi[a] - lo[a]);
    let hmax = ext.iter().copied().fold(0.0, f64::max) / cells.max(1) as f64;
    let n: [usize; 3] = core::array::from_fn(|a| ((ext[a] / hmax).round() as usize).max(1));
    let bad_cell = std::cell::Cell::new(false);
    let t = kuhn_box(lo, hi, n, &|p| {
        let r = scene.region_at(p);
        if r < 0 {
            bad_cell.set(true);
        }
        r
    })?;
    if bad_cell.get() {
        return Err(BenchError::Geometry(format!("{}: the scene does not fill its bounding box, so it has no tet twin", data.case)));
    }
    Ok(Some(t))
}

/// A tet mesh as a partitioned cell-walking tracker flying in `mode`. Regions are the benchmark
/// ids (`TopoVolume` serial = id + 1).
///
/// # Errors
/// The mesh builder's or the tracker's error.
pub fn flight_adapter(tets: &Tets, mode: FlightMode, leaf_cells: u32) -> Result<TrackerAdapter> {
    mesh_adapter(&tet_mesh(tets)?, mode, leaf_cells, None)
}

/// A mesh (tets or polyhedra) as a cell-walking tracker flying in `mode`, regions by the volume
/// classification. `label` replaces the default `mode` column (`walk`, `delta`, `block`).
///
/// # Errors
/// The tracker's error.
pub fn mesh_adapter(mesh: &sm_mesh_api::reference::ReferenceMesh, mode: FlightMode, leaf_cells: u32, label: Option<&str>) -> Result<TrackerAdapter> {
    let rule = RegionRule::ByVolumeClassification;
    let (bnd, comp) = (BoundarySpec::default(), ComplementPolicy::Outside);
    let mut part = PartitionOptions::default();
    part.enabled = mode != FlightMode::Surface;
    part.leaf_cells = leaf_cells;
    let opts = BuildOptions::default().with_partition(part);
    let input = BuildInput::Mesh(MeshInput::new(mesh, &rule, &bnd, &comp));
    let (t, _) = MeshTrackerBuilder.build_concrete(&input, &opts).map_err(bad)?;
    let map = map_by_entity(t.regions());
    let name = label.map_or_else(
        || match mode {
            FlightMode::Surface => "walk".to_string(),
            m => m.name().to_string(),
        },
        str::to_string,
    );
    Ok(TrackerAdapter::new("smrt", Box::new(t)).with_flight_mode(mode).with_mode(name).with_region_map(map).on_twin())
}

/// Mints a feature curve per pair of model faces. The mesh's faces are the region interfaces (no
/// model topology exists behind the benchmark's tets), so two interfaces meeting along a mesh edge
/// are one curve by identity; only the identity of the edge matters to the dual's geometry.
/// See above.
#[doc(hidden)]
pub struct PairEdges;

impl sm_poly::EdgeResolver for PairEdges {
    fn resolve(&self, a: EntityId, b: EntityId, _ends: [sm_mesh_api::Classification; 2]) -> Option<EntityId> {
        // 20 bits per serial; a larger serial has no unambiguous pairing, so no identity.
        const LIM: u64 = 1 << 20;
        let (x, y) = (a.serial(), b.serial());
        if x >= LIM || y >= LIM {
            return None;
        }
        EntityId::new(EntityKind::TopoEdge, (x << 20) + y)
    }
}

/// The median dual of a tet mesh (`sm-poly`), the polyhedral twin of the L2b rows: one cell per
/// (vertex, region), regions kept. Dual cells straddle no region boundary, so the region interfaces
/// are the tets' own triangles and the geometry is the same; only the cells differ.
///
/// # Errors
/// [`BenchError::Geometry`] when the tets cannot be dualized (not conforming, bad classification).
pub fn poly_twin(tets: &Tets) -> Result<(sm_mesh_api::reference::ReferenceMesh, sm_poly::DualReport)> {
    let src = tet_mesh(tets)?;
    let mut alloc = SerialAlloc::new();
    let (f, rep) = sm_poly::dualize(&src, &sm_poly::PolyOptions::new(), &PairEdges, sm_mesh_api::reference::ReferenceBuilder::new(), &mut alloc).map_err(|e| BenchError::Geometry(format!("dual of the tets: {e}")))?;
    Ok((f.mesh, rep))
}

/// The `poly-walk` row: cell walking over the polyhedral dual of the tets. The mode starts with
/// `poly`, so the report keeps it out of the verdict ("different mesh, same geometry", ruling D6).
///
/// # Errors
/// The dual's or the tracker's error.
pub fn poly_adapter(tets: &Tets) -> Result<TrackerAdapter> {
    let (mesh, _) = poly_twin(tets)?;
    mesh_adapter(&mesh, FlightMode::Surface, 8, Some("poly-walk"))
}

/// The `jump` row (W4-RT-JUMP-ROW): `MeshTracker` over the tets with a majorant partition and
/// `region_jump`, so `fire` leaves a region through its boundary faces instead of walking cell by
/// cell. It records the tracker's work counters of W-fire runs (`face_tests`, `node_visits`, `jumps`).
///
/// # Errors
/// The mesh builder's or the tracker's error.
pub fn jump_adapter(tets: &Tets, leaf_cells: u32) -> Result<TrackerAdapter> {
    let mesh = tet_mesh(tets)?;
    let rule = RegionRule::ByVolumeClassification;
    let (bnd, comp) = (BoundarySpec::default(), ComplementPolicy::Outside);
    let mut part = PartitionOptions::default();
    part.enabled = true;
    part.region_jump = true;
    part.leaf_cells = leaf_cells;
    let opts = BuildOptions::default().with_partition(part);
    let input = BuildInput::Mesh(MeshInput::new(&mesh, &rule, &bnd, &comp));
    let (t, _) = MeshTrackerBuilder.build_concrete(&input, &opts).map_err(bad)?;
    let map = map_by_entity(t.regions());
    Ok(TrackerAdapter::new("smrt", Box::new(t)).with_mode("jump").with_region_map(map).on_twin().counting_fire_work())
}

/// The reference of the `jump` row: the same tets walked cell by cell (no partition), W-fire work
/// counters recorded. Its `mode` is `walk`.
///
/// # Errors
/// The mesh builder's or the tracker's error.
pub fn walk_fire_adapter(tets: &Tets) -> Result<TrackerAdapter> {
    Ok(flight_adapter(tets, FlightMode::Surface, 8)?.counting_fire_work())
}

/// The W-walk row of a tet mesh: the cell-walking tracker of the `walk` flight row, reporting the
/// tet index of every cell crossed. The cell-to-tet map is built by locating each tet's centroid,
/// so it does not assume the tracker keeps the input order; it must be a bijection, so the tracker
/// must have exactly one cell per tet.
///
/// # Errors
/// The mesh builder's or the tracker's error, or [`BenchError::Tracker`] when the cell count is not
/// the tet count or a centroid is not located in a cell of its own.
pub fn walk_adapter(tets: &Tets) -> Result<TrackerAdapter> {
    let a = flight_adapter(tets, FlightMode::Surface, 8)?;
    let n = tets.tets.len();
    // One cell per tet: a tracker with more cells would leave unmapped cells at -1, which `walk`
    // rejects only when a ray meets one; refuse it here with the numbers.
    if a.tracker().cell_count() != n {
        return Err(BenchError::Tracker(format!("the tracker has {} cells for {n} tets (W-walk needs one cell per tet)", a.tracker().cell_count())));
    }
    let mut map = vec![-1i32; n];
    let mut scratch = sm_rt_api::Scratch::new();
    for (i, t) in tets.tets.iter().enumerate() {
        let c: [f64; 3] = core::array::from_fn(|k| t.iter().map(|&v| tets.nodes[v as usize][k]).sum::<f64>() / 4.0);
        let loc = a.tracker().locate(pt(c), None, None, &mut scratch).map_err(bad)?;
        let cell = loc.cell;
        if cell.is_none() || cell.get() >= map.len() || map[cell.get()] >= 0 {
            return Err(BenchError::Tracker(format!("tet {i}: its centroid is not located in a cell of its own (cell {})", cell.0)));
        }
        map[cell.get()] = i as i32;
    }
    Ok(a.with_cell_map(map).with_mode("walk"))
}

/// The tets as a reference mesh whose volume classes are the regions.
///
/// # Errors
/// [`BenchError::Geometry`] when the mesh builder refuses the tets.
pub fn tet_mesh(tets: &Tets) -> Result<sm_mesh_api::reference::ReferenceMesh> {
    let cells: Vec<(CellSpec, u64)> = tets
        .tets
        .iter()
        .zip(&tets.region)
        .map(|(t, &r)| Ok((CellSpec::Template(CellShape::Tet, t.iter().map(|&v| v as usize).collect()), volume_id(r)?.serial())))
        .collect::<Result<_>>()?;
    build_mesh_volumes(&tets.nodes, &cells, &mut SerialAlloc::new()).map_err(|e| BenchError::Geometry(e.to_string()))
}

/// The sm-rt adapters of one case, grouped by what their results may be compared with.
pub struct SmrtRows {
    /// W-track and W-fire rows: facets, analytic and (P2) hierarchy.
    pub geometric: Vec<Box<dyn Adapter>>,
    /// W-flight on the exact geometry (the analytic row alone).
    pub exact_flight: Vec<Box<dyn Adapter>>,
    /// W-flight on the tet twin: walk (surface), delta and block, whose tallies are compared.
    pub mesh_flight: Vec<Box<dyn Adapter>>,
    /// W-fire on the tet twin: the cell walk (reference) and the `jump` row (cases with a tet mesh).
    pub jump_fire: Vec<Box<dyn Adapter>>,
    /// W-track and W-flight on the tet walk (reference) and on its polyhedral dual (`poly-walk`).
    pub poly: Vec<Box<dyn Adapter>>,
    /// Why a row was not built.
    pub notes: Vec<String>,
}

/// Every sm-rt row of a case that can be built, with the reasons for the rows that cannot.
/// `flight_cells` is the tet twin's resolution along the longest axis for cases without a tet mesh.
pub fn smrt_rows(data: &CaseData, scale: &Scale, flight_cells: usize) -> SmrtRows {
    let mut rows = SmrtRows { geometric: Vec::new(), exact_flight: Vec::new(), mesh_flight: Vec::new(), jump_fire: Vec::new(), poly: Vec::new(), notes: Vec::new() };
    let case = data.case.clone();
    let mut notes: Vec<String> = Vec::new();
    let mut note = |what: &str, e: BenchError| notes.push(format!("{case}: smrt {what} not run: {e}"));
    // A deck case (A17) has no facet twin: nothing to build, nothing partial.
    if !data.facets.tris.is_empty() {
        match facets_adapter(data) {
            Ok(a) => rows.geometric.push(Box::new(a)),
            Err(e) => note("facets", e),
        }
    }
    match analytic_adapter(data, scale) {
        Ok(a) => rows.geometric.push(Box::new(a)),
        Err(e) => note("analytic", e),
    }
    if let Ok(a) = analytic_adapter(data, scale) {
        rows.exact_flight.push(Box::new(a));
    }
    if data.case == "P2" {
        match hierarchy_p2(scale, &data.manifest.regions) {
            Ok(a) => rows.geometric.push(Box::new(a)),
            Err(e) => note("hierarchy", e),
        }
        match analytic_hierarchy_p2(scale, &data.manifest.regions) {
            Ok(a) => rows.geometric.push(Box::new(a)),
            Err(e) => note("analytic-hierarchy", e),
        }
    }
    if let Some(t) = &data.tets {
        match (walk_fire_adapter(t), jump_adapter(t, 8)) {
            (Ok(w), Ok(j)) => {
                rows.jump_fire.push(Box::new(w));
                rows.jump_fire.push(Box::new(j));
            }
            (Err(e), _) | (_, Err(e)) => note("jump", e),
        }
    }
    if let Some(t) = &data.tets {
        // The reference has its own mode (`poly-ref`): `walk` is the mesh_flight row's, and a second
        // `walk` row on the same cell would be dropped silently by the verdict. `poly-*` modes are
        // outside the verdict, so the reference is too.
        let reference = tet_mesh(t).and_then(|m| mesh_adapter(&m, FlightMode::Surface, 8, Some("poly-ref")));
        match (reference, poly_adapter(t)) {
            (Ok(w), Ok(p)) => {
                rows.poly.push(Box::new(w));
                rows.poly.push(Box::new(p));
            }
            (Err(e), _) | (_, Err(e)) => note("poly-walk", e),
        }
    }
    match tet_twin(data, flight_cells) {
        Ok(Some(t)) => {
            for mode in [FlightMode::Surface, FlightMode::Delta, FlightMode::Block] {
                match flight_adapter(&t, mode, 8) {
                    Ok(a) => rows.mesh_flight.push(Box::new(a)),
                    Err(e) => note(mode.name(), e),
                }
            }
        }
        Ok(None) => notes.push(format!("{case}: smrt flight rows on a tet twin not run: the case has no tet twin")),
        Err(e) => note("flight rows", e),
    }
    rows.notes = notes;
    rows
}

/// Largest `|d_a - d_b|` of the W-fire distances of two adapters over the rays neither loses, and
/// the number of rays compared (used to record how close the jump is to the walk, R1 level).
///
/// # Errors
/// The adapters' errors, or [`BenchError::Check`] when the fire outputs differ in length.
pub fn max_fire_delta(data: &CaseData, a: &mut dyn Adapter, b: &mut dyn Adapter) -> Result<(f64, usize)> {
    use crate::adapter::{WorkInput, Workload};
    let w = WorkInput { rays: &data.rays, regions: &data.manifest.regions, threads: 1, histories: 0, seed: 1 };
    let run = |x: &mut dyn Adapter| -> Result<Vec<(f64, i32)>> {
        x.prepare(Workload::Fire, &w)?;
        Ok(x.run(Workload::Fire, &w)?.fires)
    };
    let (fa, fb) = (run(a)?, run(b)?);
    if fa.len() != fb.len() {
        return Err(BenchError::Check(format!("fire outputs have {} and {} rays", fa.len(), fb.len())));
    }
    let both: Vec<f64> = fa.iter().zip(&fb).filter(|(x, y)| x.1 != -2 && y.1 != -2).map(|(x, y)| (x.0 - y.0).abs()).collect();
    Ok((both.iter().copied().fold(0.0, f64::max), both.len()))
}

/// The rows' tracker handle, for tests that need the concrete `Tracker`.
pub fn tracker_of(a: &TrackerAdapter) -> &dyn Tracker {
    a.tracker()
}
