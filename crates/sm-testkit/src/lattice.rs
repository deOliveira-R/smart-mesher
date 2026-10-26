//! Classified lattice meshes on a real topology: structured hex and Kuhn-tet boxes, a planar quad
//! sheet and a cylindrical quad tube, each with the `CellComplex` it is classified on, a model,
//! the unperturbed node positions, and a deterministic constrained jitter ([`perturb`]).
//!
//! Smoothers and quality tools need a mesh whose boundary nodes know their model entity and
//! parameter, with the exact lattice to compare against. Every node is classified like
//! `sm_mesh_api::reference::hex_block` (corners on `TopoVertex`, edge nodes on `TopoEdge` with
//! `Param::Curve`, face nodes on `TopoFace` with `Param::Surface`, interior nodes on the
//! `TopoVolume`); the parameters are the projections onto the carriers in the topology, so the
//! mesh passes `sm_mesh_api::validate` and `check::conformity` against them.

use crate::meshes::{box_entity, build_box_lattice};
use crate::surf3::{SplitMix64, mix64};
use crate::topo::{BuildError, finish_boundaries};
use core::f64::consts::TAU;
use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::{
    CellShape, Classification, FaceIx, MeshBuild, MeshEdit, MeshEntity, MeshEntityKind, MeshEntityRef, MeshError, MeshRead, Param, SerialAlloc,
    VertexIx,
};
use sm_model_api::{Domain, Model};
use sm_topo_api::fixtures::{self, FixtureError, with_op};
use sm_topo_api::{CellComplex, Carrier, OpId, TopologyView};

/// A lattice fixture could not be built.
#[derive(Debug, thiserror::Error)]
pub enum LatticeError {
    /// A topology or model fixture failed.
    #[error(transparent)]
    Build(#[from] BuildError),
    /// A topology fixture failed.
    #[error(transparent)]
    Fixture(#[from] FixtureError),
    /// The mesh builder rejected the input.
    #[error(transparent)]
    Mesh(#[from] MeshError),
    /// A model operation failed.
    #[error(transparent)]
    Model(#[from] sm_model_api::ModelError),
    /// A degenerate size or count.
    #[error("invalid lattice: {0}")]
    Invalid(String),
    /// A carrier is missing or cannot project.
    #[error("geometry: {0}")]
    Geometry(String),
}

/// Result alias of this module.
pub type Result<T> = core::result::Result<T, LatticeError>;

/// A classified mesh with its model and exact node positions.
#[derive(Clone, Debug)]
pub struct LatticeFixture {
    /// The topology the mesh is classified on.
    pub topo: CellComplex,
    /// The model tables (a volume color and the outer boundary).
    pub model: Model,
    /// The mesh.
    pub mesh: ReferenceMesh,
    /// Unperturbed position of every vertex, in vertex order.
    pub exact: Vec<[f64; 3]>,
    /// Node spacing: the smallest cell size over the axes.
    pub h: f64,
}

impl LatticeFixture {
    /// A copy whose nodes are jittered by [`perturb`] with amplitude `amplitude_over_h * self.h`.
    pub fn perturbed(&self, seed: u64, amplitude_over_h: f64) -> Result<Self> {
        let mut out = self.clone();
        perturb(&mut out.mesh, &out.topo, seed, amplitude_over_h * self.h)?;
        Ok(out)
    }
}

fn check_size(n: &[usize], size: &[f64]) -> Result<()> {
    if n.contains(&0) || size.iter().any(|s| !(s.is_finite() && *s > 0.0)) {
        return Err(LatticeError::Invalid(format!("counts {n:?} must be positive and sizes {size:?} positive and finite")));
    }
    Ok(())
}

/// The classification parameter of the point `p` on `entity`, from its carrier in `topo`
/// (`Param::None` for vertices, volumes and entities without a carrier).
fn param_on(topo: &dyn TopologyView, entity: EntityId, p: Point3) -> Param {
    match (entity.kind(), topo.carrier(entity)) {
        (EntityKind::TopoEdge, Carrier::Curve(cc)) => topo
            .shapes()
            .curve(cc.curve)
            .and_then(|c| c.project(p, None).ok())
            .map_or(Param::None, |r| Param::Curve(r.t)),
        (EntityKind::TopoFace, Carrier::Surface(sc)) => topo
            .shapes()
            .surface(sc.surface)
            .and_then(|s| s.project(p, None).ok())
            .map_or(Param::None, |r| Param::Surface([r.uv.u, r.uv.v])),
        _ => Param::None,
    }
}

fn box_model(t: &CellComplex) -> Result<Model> {
    let mut m = Model::new();
    let vol = t.cells(sm_topo_api::types::Dim::Volume).first().copied();
    if let Some(v) = vol {
        let c = m.add_color(1, "mat", Domain::Volume)?;
        m.color_volumes(c, &[v])?;
    }
    finish_boundaries(&mut m, t, 7).map_err(BuildError::from)?;
    Ok(m)
}

fn exact_of(mesh: &ReferenceMesh) -> Vec<[f64; 3]> {
    (0..mesh.vertex_count())
        .map(|i| {
            let p = mesh.point(i);
            [p.x, p.y, p.z]
        })
        .collect()
}

fn box_lattice(n: [usize; 3], size: [f64; 3], kuhn: bool) -> Result<LatticeFixture> {
    check_size(&n, &size)?;
    let mut topo = CellComplex::new();
    with_op(&mut topo, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), size))?;
    let model = box_model(&topo)?;
    let pt = |i: usize, a: usize| size[a] * i as f64 / n[a] as f64;
    let point = |i: usize, j: usize, k: usize| Point3::new(pt(i, 0), pt(j, 1), pt(k, 2));
    let mesh = build_box_lattice(n.map(|x| x as u32), kuhn, &point, &|e, p| param_on(&topo, e, p), &mut SerialAlloc::new())?;
    let h = (0..3).map(|a| size[a] / n[a] as f64).fold(f64::INFINITY, f64::min);
    Ok(LatticeFixture { exact: exact_of(&mesh), topo, model, mesh, h })
}

/// A structured `n0 x n1 x n2` hexahedral lattice on a `CellComplex` brick `[0, size]`:
/// `(n0+1)(n1+1)(n2+1)` nodes at `size_a * i / n_a`, total volume `size0 size1 size2`, one
/// volume, classified as described in the module docs. `h` is `min(size_a / n_a)`.
pub fn hex(n: [usize; 3], size: [f64; 3]) -> Result<LatticeFixture> {
    box_lattice(n, size, false)
}

/// The same lattice with every hexahedron split into 6 Kuhn tetrahedra
/// ([`crate::meshes::tet_block`]): `6 n0 n1 n2` positive tets, volume `size0 size1 size2`.
pub fn tet_kuhn(n: [usize; 3], size: [f64; 3]) -> Result<LatticeFixture> {
    box_lattice(n, size, true)
}

/// A planar `n0 x n1` quadrilateral sheet `[0, size0] x [0, size1]` at `z = 0` on the `+z` face of
/// a brick `[0,size0] x [0,size1] x [-1,0]` (4 `TopoEdge`, 4 `TopoVertex`), a dimension-2 mesh
/// with `(n0+1)(n1+1)` nodes, counter-clockwise quads (normal `+z`), area `size0 size1`. Only the
/// `+z` face is meshed. `h` is `min(size_a / n_a)`.
pub fn quad(n: [usize; 2], size: [f64; 2]) -> Result<LatticeFixture> {
    check_size(&n, &size)?;
    let mut topo = CellComplex::new();
    with_op(&mut topo, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, -1.0), [size[0], size[1], 1.0]))?;
    let model = box_model(&topo)?;
    let n3 = [n[0] as u32, n[1] as u32, 1];
    let ent = |pos: [Option<usize>; 2]| box_entity(n3, [pos[0], pos[1], Some(1)]);
    let m0 = n[0] + 1;
    let mut b = ReferenceBuilder::surface();
    for j in 0..=n[1] {
        for i in 0..=n[0] {
            let p = Point3::new(size[0] * i as f64 / n[0] as f64, size[1] * j as f64 / n[1] as f64, 0.0);
            let v = b.add_vertex(p)?;
            let e = ent([Some(i), Some(j)])?;
            b.set_class(MeshEntityRef::Vertex(v), Classification { entity: e, param: param_on(&topo, e, p) })?;
        }
    }
    let face = ent([None, None])?;
    for j in 0..n[1] {
        for i in 0..n[0] {
            let at = |di: usize, dj: usize| VertexIx::new((j + dj) * m0 + i + di);
            let added = b.add_cell_from_vertices(CellShape::Quad, &[at(0, 0), at(1, 0), at(1, 1), at(0, 1)])?;
            b.set_class(MeshEntityRef::Cell(added.cell), Classification::on(face))?;
        }
    }
    for f in 0..b.face_count() {
        let f = FaceIx::new(f);
        let class = if b.face_degree(f) == 1 {
            let l = b.face_loop(f);
            let (p, q) = ((l[0].get() % m0, l[0].get() / m0), (l[1].get() % m0, l[1].get() / m0));
            ent(if p.1 == q.1 { [None, Some(p.1)] } else { [Some(p.0), None] })?
        } else {
            face
        };
        b.set_class(MeshEntityRef::Face(f), Classification::on(class))?;
    }
    let mesh = b.finish(&mut SerialAlloc::new())?.mesh;
    let h = (size[0] / n[0] as f64).min(size[1] / n[1] as f64);
    Ok(LatticeFixture { exact: exact_of(&mesh), topo, model, mesh, h })
}

/// A dimension-2 quad mesh of the lateral face of `sm_topo_api::fixtures::cylinder(r, height)`,
/// closed across the seam: `n_theta` nodes per ring (angle `2 pi i / n_theta`, `n_theta >= 3`) and
/// `n_z + 1` rings. Ring nodes lie on the two circle edges, seam nodes on the seam edge, the two
/// seam ends on the two `TopoVertex`, everything else on the lateral `TopoFace`. Quads are
/// counter-clockwise seen from outside, area `2 pi r height` in the limit. `h` is
/// `min(2 pi r / n_theta, height / n_z)`.
pub fn tube_quad(n_theta: usize, n_z: usize, r: f64, height: f64) -> Result<LatticeFixture> {
    check_size(&[n_theta.saturating_sub(2), n_z], &[r, height])?;
    let mut topo = CellComplex::new();
    let ids = with_op(&mut topo, OpId(1), |t| fixtures::cylinder(t, r, height))?;
    let model = box_model(&topo)?;
    let (e_bot, e_top, e_seam) = (ids.edges[0], ids.edges[1], ids.edges[2]);
    let lateral = ids.faces[2];
    let entity = |it: usize, iz: usize| -> EntityId {
        match (iz, it) {
            (0, 0) => ids.vertices[0],
            (0, _) => e_bot,
            (z, 0) if z == n_z => ids.vertices[1],
            (z, _) if z == n_z => e_top,
            (_, 0) => e_seam,
            _ => lateral,
        }
    };
    let mut b = ReferenceBuilder::surface();
    for iz in 0..=n_z {
        for it in 0..n_theta {
            let a = TAU * it as f64 / n_theta as f64;
            let p = Point3::new(r * a.cos(), r * a.sin(), height * iz as f64 / n_z as f64);
            let v = b.add_vertex(p)?;
            let e = entity(it, iz);
            b.set_class(MeshEntityRef::Vertex(v), Classification { entity: e, param: param_on(&topo, e, p) })?;
        }
    }
    for iz in 0..n_z {
        for it in 0..n_theta {
            let at = |dt: usize, dz: usize| VertexIx::new((iz + dz) * n_theta + (it + dt) % n_theta);
            let added = b.add_cell_from_vertices(CellShape::Quad, &[at(0, 0), at(1, 0), at(1, 1), at(0, 1)])?;
            b.set_class(MeshEntityRef::Cell(added.cell), Classification::on(lateral))?;
        }
    }
    for f in 0..b.face_count() {
        let f = FaceIx::new(f);
        let class = if b.face_degree(f) == 1 {
            // A boundary facet runs along the bottom or the top ring.
            if b.face_loop(f)[0].get() < n_theta { e_bot } else { e_top }
        } else {
            lateral
        };
        b.set_class(MeshEntityRef::Face(f), Classification::on(class))?;
    }
    let mesh = b.finish(&mut SerialAlloc::new())?.mesh;
    let h = (TAU * r / n_theta as f64).min(height / n_z as f64);
    Ok(LatticeFixture { exact: exact_of(&mesh), topo, model, mesh, h })
}

/// Jitter the nodes of `mesh` deterministically and keep each on its classified entity.
///
/// Node `i` is displaced by `amp * (u0, u1, u2)` with `u` in `[-1, 1)` drawn from a SplitMix64
/// stream seeded by `mix64(seed) ^ mix64(i)` (no shared RNG state, no thread dependence), then
/// moved onto its entity: corner vertices stay fixed, an edge node is projected onto the curve, a
/// face node onto the surface (the classification parameter is updated), a volume node moves
/// freely. `amp` is an absolute length; use `amplitude_over_h * fixture.h` or
/// [`LatticeFixture::perturbed`]. Entities that are not `Topo*` kinds are left alone.
pub fn perturb(mesh: &mut ReferenceMesh, topo: &dyn TopologyView, seed: u64, amp: f64) -> Result<()> {
    for i in 0..mesh.vertex_count() {
        let c = mesh.classification(MeshEntityKind::Vertex, i as u32);
        if !matches!(c.entity.kind(), EntityKind::TopoEdge | EntityKind::TopoFace | EntityKind::TopoVolume) {
            continue;
        }
        let p = mesh.point(i);
        let mut rng = SplitMix64(mix64(seed) ^ mix64(i as u64));
        let q = Point3::new(p.x + amp * rng.unit(), p.y + amp * rng.unit(), p.z + amp * rng.unit());
        let missing = || LatticeError::Geometry(format!("no carrier for {:?}", c.entity));
        let (moved, param) = match (c.entity.kind(), topo.carrier(c.entity)) {
            (EntityKind::TopoEdge, Carrier::Curve(cc)) => {
                let curve = topo.shapes().curve(cc.curve).ok_or_else(missing)?;
                let r = curve.project(q, None).map_err(|e| LatticeError::Geometry(e.to_string()))?;
                (r.point, Param::Curve(r.t))
            }
            (EntityKind::TopoFace, Carrier::Surface(sc)) => {
                let surf = topo.shapes().surface(sc.surface).ok_or_else(missing)?;
                let r = surf.project(q, None).map_err(|e| LatticeError::Geometry(e.to_string()))?;
                (r.point, Param::Surface([r.uv.u, r.uv.v]))
            }
            (EntityKind::TopoVolume, _) => (q, Param::None),
            _ => return Err(missing()),
        };
        let v = VertexIx::new(i);
        mesh.set_point(v.into(), moved)?;
        mesh.set_class(MeshEntity::Vertex(v), Classification { entity: c.entity, param })?;
    }
    Ok(())
}
