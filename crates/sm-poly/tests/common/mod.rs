//! Shared helpers: resolvers, hashing, invariants.
#![allow(dead_code)]

use sm_core::{EntityId, EntityKind};
use sm_mesh::MeshBuilder;
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::{
    CellIx, Classification, FaceIx, MeshEntityKind, MeshRead, SerialAlloc, cell_volume,
    face_area_vector,
};
use sm_mesh_api::CellShape;
use sm_testkit::meshes::CellSpec;
use sm_poly::{DualReport, EdgeResolver, PolyError, PolyOptions, dualize};

/// Mints a `TopoEdge` id from the face pair (for fixtures whose edges are single intervals).
pub struct PairResolver;

impl EdgeResolver for PairResolver {
    fn resolve(&self, a: EntityId, b: EntityId, _ends: [Classification; 2]) -> Option<EntityId> {
        EntityId::new(EntityKind::TopoEdge, a.serial() * 1000 + b.serial())
    }
}

pub fn dual_ref(
    src: &dyn MeshRead,
    opts: &PolyOptions,
    r: &dyn EdgeResolver,
) -> Result<(ReferenceMesh, DualReport), PolyError> {
    let mut alloc = SerialAlloc::new();
    let (f, rep) = dualize(src, opts, r, ReferenceBuilder::new(), &mut alloc)?;
    Ok((f.mesh, rep))
}

pub fn dual_prod(
    src: &dyn MeshRead,
    opts: &PolyOptions,
    r: &dyn EdgeResolver,
) -> Result<(sm_mesh::Mesh, DualReport), PolyError> {
    let mut alloc = SerialAlloc::new();
    let (f, rep) = dualize(src, opts, r, MeshBuilder::new(), &mut alloc)?;
    Ok((f.mesh, rep))
}

pub fn volumes(m: &dyn MeshRead) -> Vec<f64> {
    (0..m.count(MeshEntityKind::Cell)).map(|c| cell_volume(m, CellIx::new(c))).collect()
}

pub fn boundary_area_on(m: &dyn MeshRead, model: EntityId) -> f64 {
    m.classified_on(model)
        .map(|s| s.faces.iter().filter(|f| m.is_boundary_face(**f)).map(|f| face_area_vector(m, *f).norm()).sum())
        .unwrap_or(0.0)
}

pub fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-300)
}

pub fn id(kind: EntityKind, serial: u64) -> EntityId {
    EntityId::new(kind, serial).expect("id")
}

pub fn n_faces(m: &dyn MeshRead) -> usize {
    m.count(MeshEntityKind::Face)
}

pub fn face_sizes(m: &dyn MeshRead) -> Vec<usize> {
    (0..n_faces(m)).map(|f| m.face_vertices(FaceIx::new(f)).len()).collect()
}

/// Resolver that must never be asked.
pub struct NeverResolver;

impl EdgeResolver for NeverResolver {
    fn resolve(&self, _a: EntityId, _b: EntityId, _ends: [Classification; 2]) -> Option<EntityId> {
        None
    }
}

/// A copy of a lattice mesh with a fully classified edge table: edges along a model edge are
/// classified on it (found through the topology), the others derive.
pub fn with_edge_table(mesh: &ReferenceMesh, topo: &dyn sm_topo_api::TopologyView) -> ReferenceMesh {
    use sm_mesh_api::{MeshEdit, VertexIx};
    use sm_topo_api::TopologyViewExt;
    let on_curve = |c: Classification, curve: EntityId| match c.entity.kind() {
        EntityKind::TopoEdge => c.entity == curve,
        EntityKind::TopoVertex => topo.edge_ends(curve).is_some_and(|(p, q)| p == c.entity || q == c.entity),
        _ => false,
    };
    let mut explicit: Vec<(VertexIx, VertexIx, Classification)> = Vec::new();
    for f in 0..mesh.face_count() {
        let l = mesh.face_vertices(FaceIx::new(f));
        for i in 0..l.len() {
            let (x, y) = (l[i], l[(i + 1) % l.len()]);
            let (cx, cy) = (mesh.vertex_class(x), mesh.vertex_class(y));
            let curve = [cx, cy].iter().find(|c| c.entity.kind() == EntityKind::TopoEdge).map(|c| c.entity);
            if let Some(curve) = curve
                && on_curve(cx, curve)
                && on_curve(cy, curve)
            {
                explicit.push((x.min(y), x.max(y), Classification::on(curve)));
            }
        }
    }
    explicit.sort_by_key(|e| (e.0, e.1));
    explicit.dedup_by_key(|e| (e.0, e.1));
    let mut out = mesh.clone();
    out.ensure_edges(&mut SerialAlloc::new(), &explicit).expect("edge table");
    out
}

pub fn orient(pts: &[[f64; 3]], v: [usize; 4]) -> CellSpec {
    let (a, b, c, d) = (pts[v[0]], pts[v[1]], pts[v[2]], pts[v[3]]);
    let det = (b[0] - a[0]) * ((c[1] - a[1]) * (d[2] - a[2]) - (c[2] - a[2]) * (d[1] - a[1]))
        - (b[1] - a[1]) * ((c[0] - a[0]) * (d[2] - a[2]) - (c[2] - a[2]) * (d[0] - a[0]))
        + (b[2] - a[2]) * ((c[0] - a[0]) * (d[1] - a[1]) - (c[1] - a[1]) * (d[0] - a[0]));
    assert!(det != 0.0);
    let o = if det > 0.0 { vec![v[0], v[1], v[2], v[3]] } else { vec![v[0], v[2], v[1], v[3]] };
    CellSpec::Template(CellShape::Tet, o)
}

/// The BCC point block of `n^3` unit cubes: corners then centres. Tets are the disphenoid
/// honeycomb `{c1, c2, p, q}` (adjacent centres, an edge of their shared square), and two tets
/// per boundary pyramid. Returns the points, the tets and the number of corner points.
pub fn bcc_patch(n: usize) -> (Vec<[f64; 3]>, Vec<CellSpec>, usize) {
    let m = n + 1;
    let corner = |i: usize, j: usize, k: usize| (k * m + j) * m + i;
    let centre = |i: usize, j: usize, k: usize| m * m * m + (k * n + j) * n + i;
    let mut pts = Vec::new();
    for k in 0..m {
        for j in 0..m {
            for i in 0..m {
                pts.push([i as f64, j as f64, k as f64]);
            }
        }
    }
    for k in 0..n {
        for j in 0..n {
            for i in 0..n {
                pts.push([i as f64 + 0.5, j as f64 + 0.5, k as f64 + 0.5]);
            }
        }
    }
    let mut cells = Vec::new();
    for k in 0..n {
        for j in 0..n {
            for i in 0..n {
                let c = [i, j, k];
                for d in 0..3 {
                    for s in 0..2 {
                        let (u, v) = ((d + 1) % 3, (d + 2) % 3);
                        let mut nb = [i as isize, j as isize, k as isize];
                        nb[d] += if s == 1 { 1 } else { -1 };
                        let exists = nb.iter().all(|&x| (0..n as isize).contains(&x));
                        if s == 0 && exists {
                            continue; // the neighbour's s = 1 pass builds this square
                        }
                        let cyc: Vec<usize> = [(0, 0), (1, 0), (1, 1), (0, 1)]
                            .iter()
                            .map(|&(a, b)| {
                                let mut q = c;
                                q[d] += s;
                                q[u] += a;
                                q[v] += b;
                                corner(q[0], q[1], q[2])
                            })
                            .collect();
                        let c1 = centre(i, j, k);
                        if exists {
                            let c2 = centre(nb[0] as usize, nb[1] as usize, nb[2] as usize);
                            for e in 0..4 {
                                cells.push(orient(&pts, [c1, c2, cyc[e], cyc[(e + 1) % 4]]));
                            }
                        } else {
                            // boundary pyramid: two tets split along a diagonal (cospherical base)
                            cells.push(orient(&pts, [c1, cyc[0], cyc[1], cyc[2]]));
                            cells.push(orient(&pts, [c1, cyc[0], cyc[2], cyc[3]]));
                        }
                    }
                }
            }
        }
    }
    (pts, cells, m * m * m)
}

