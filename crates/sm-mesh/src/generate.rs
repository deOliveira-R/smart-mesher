//! Generators for large structured test and benchmark meshes.
//!
//! [`hex_block`] builds the same mesh as `sm_mesh_api::reference::hex_block` (a CAD-box
//! classification: 8 `TopoVertex`, 12 `TopoEdge`, 6 `TopoFace`, one `TopoVolume`) through any
//! [`MeshBuild`]; [`hex_block_bulk`] builds it with the parallel bulk path of [`MeshBuilder`].

use crate::{Mesh, MeshBuilder};
use rayon::prelude::*;
use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh_api::{
    CellShape, Classification, FaceIx, Finished, MeshBuild, MeshEntityRef, MeshError, Result,
    SerialAlloc, Side, VertexIx,
};

fn model(kind: EntityKind, serial: usize) -> Option<Classification> {
    EntityId::new(kind, serial as u64).map(Classification::on)
}

fn need(c: Option<Classification>) -> Result<Classification> {
    c.ok_or(MeshError::SerialOverflow)
}

/// Class of a lattice entity: per axis either a lattice index (`Some`) or free (`None`). Axes on
/// the box surface are "fixed"; the rest slide along the entity.
fn class_of(n: [usize; 3], pos: [Option<usize>; 3]) -> Option<Classification> {
    let ext = |a: usize, q: usize| q == 0 || q == n[a];
    let side = |a: usize, q: usize| usize::from(q == n[a]);
    let fixed: Vec<usize> = (0..3).filter(|&a| pos[a].is_some_and(|q| ext(a, q))).collect();
    let sliding: Vec<usize> = (0..3).filter(|a| !fixed.contains(a)).collect();
    let sd = |a: usize| side(a, pos[a].unwrap_or(0));
    match fixed.len() {
        3 => model(EntityKind::TopoVertex, 1 + sd(0) + 2 * sd(1) + 4 * sd(2)),
        2 => model(EntityKind::TopoEdge, 1 + sliding[0] * 4 + sd(fixed[0]) + 2 * sd(fixed[1])),
        1 => model(EntityKind::TopoFace, 1 + fixed[0] * 2 + sd(fixed[0])),
        _ => model(EntityKind::TopoVolume, 1),
    }
}

/// Template-order vertices of cell `(i, j, k)` of an `m`-point lattice.
fn cell_verts(m: [usize; 3], i: usize, j: usize, k: usize) -> [VertexIx; 8] {
    let ix = |i: usize, j: usize, k: usize| VertexIx::new((k * m[1] + j) * m[0] + i);
    [
        ix(i, j, k),
        ix(i + 1, j, k),
        ix(i + 1, j + 1, k),
        ix(i, j + 1, k),
        ix(i, j, k + 1),
        ix(i + 1, j, k + 1),
        ix(i + 1, j + 1, k + 1),
        ix(i, j + 1, k + 1),
    ]
}

/// Which model face (axis, side) each template face of cell `(i, j, k)` lies on, if any.
/// Template face order: -z, +z, -y, +x, +y, -x.
fn faces_on(n: [usize; 3], i: usize, j: usize, k: usize) -> [(usize, usize, bool); 6] {
    [
        (2, 0, k == 0),
        (2, 1, k + 1 == n[2]),
        (1, 0, j == 0),
        (0, 1, i + 1 == n[0]),
        (1, 1, j + 1 == n[1]),
        (0, 0, i == 0),
    ]
}

fn check(n: [u32; 3]) -> Result<[usize; 3]> {
    if n.contains(&0) {
        return Err(MeshError::State("hex_block needs at least one cell per axis".into()));
    }
    Ok([n[0] as usize, n[1] as usize, n[2] as usize])
}

/// An `n[0] x n[1] x n[2]` block of hexahedra with cell sizes `h`, from the origin, classified
/// like a CAD box (identical to `sm_mesh_api::reference::hex_block`, and with `edges` to
/// `hex_block_with_edges`: a fully classified edge table), built cell by cell through `b`.
pub fn hex_block<B: MeshBuild>(
    mut b: B,
    n: [u32; 3],
    h: [f64; 3],
    edges: bool,
    alloc: &mut SerialAlloc,
) -> Result<Finished<B::Output>> {
    let n = check(n)?;
    let m = [n[0] + 1, n[1] + 1, n[2] + 1];
    for k in 0..m[2] {
        for j in 0..m[1] {
            for i in 0..m[0] {
                let v = b.add_vertex(Point3::new(i as f64 * h[0], j as f64 * h[1], k as f64 * h[2]))?;
                b.set_class(MeshEntityRef::Vertex(v), need(class_of(n, [Some(i), Some(j), Some(k)]))?)?;
            }
        }
    }
    let vol = need(model(EntityKind::TopoVolume, 1))?;
    for k in 0..n[2] {
        for j in 0..n[1] {
            for i in 0..n[0] {
                let added = b.add_cell_from_vertices(CellShape::Hex, &cell_verts(m, i, j, k))?;
                b.set_class(MeshEntityRef::Cell(added.cell), vol)?;
                for (sf, (axis, s, on)) in added.faces.iter().zip(faces_on(n, i, j, k)) {
                    if on {
                        let c = need(model(EntityKind::TopoFace, 1 + axis * 2 + s))?;
                        b.set_class(MeshEntityRef::Face(sf.face()), c)?;
                    }
                }
            }
        }
    }
    if edges {
        let ix = |i: usize, j: usize, k: usize| VertexIx::new((k * m[1] + j) * m[0] + i);
        let ext = |a: usize, q: usize| q == 0 || q == n[a];
        for k in 0..m[2] {
            for j in 0..m[1] {
                for i in 0..m[0] {
                    for a in 0..3 {
                        let p = [i, j, k];
                        if p[a] + 1 >= m[a] {
                            continue;
                        }
                        let mut q = p;
                        q[a] += 1;
                        if (0..3).filter(|&x| x != a && ext(x, p[x])).count() == 0 {
                            continue;
                        }
                        let mut pos = [Some(p[0]), Some(p[1]), Some(p[2])];
                        pos[a] = None;
                        b.set_class(
                            MeshEntityRef::Edge(ix(p[0], p[1], p[2]), ix(q[0], q[1], q[2])),
                            need(class_of(n, pos))?,
                        )?;
                    }
                }
            }
        }
    }
    b.finish(alloc)
}

/// The same mesh as [`hex_block`] (without edges), built with the bulk path: vertices, cells,
/// face table and classification are all computed in parallel.
pub fn hex_block_bulk(n: [u32; 3], h: [f64; 3], alloc: &mut SerialAlloc) -> Result<Finished<Mesh>> {
    let n = check(n)?;
    let m = [n[0] + 1, n[1] + 1, n[2] + 1];
    let mut b = MeshBuilder::new();
    b.add_vertices_with(m[0] * m[1] * m[2], |v| {
        let (i, j, k) = (v % m[0], (v / m[0]) % m[1], v / (m[0] * m[1]));
        Point3::new(i as f64 * h[0], j as f64 * h[1], k as f64 * h[2])
    })?;
    let nc = n[0] * n[1] * n[2];
    let verts: Vec<VertexIx> = (0..nc)
        .into_par_iter()
        .flat_map_iter(|c| cell_verts(m, c % n[0], (c / n[0]) % n[1], c / (n[0] * n[1])))
        .collect();
    b.add_uniform_cells(CellShape::Hex, &verts)?;
    b.classify_vertices_with(|v| class_of(n, [Some(v % m[0]), Some((v / m[0]) % m[1]), Some(v / (m[0] * m[1]))]))?;
    let vol = model(EntityKind::TopoVolume, 1);
    b.classify_cells_with(|_| vol)?;
    let fc: Vec<Option<Classification>> = (0..b.face_count())
        .into_par_iter()
        .map(|f| {
            let fi = FaceIx::new(f);
            if b.face_degree(fi) != 1 {
                return None;
            }
            let c = b.face_cell(fi, Side::Left).or_else(|| b.face_cell(fi, Side::Right))?.get();
            let slot = b.cell_faces(sm_mesh_api::CellIx::new(c)).iter().position(|sf| sf.face() == fi)?;
            let (axis, s, on) = faces_on(n, c % n[0], (c / n[0]) % n[1], c / (n[0] * n[1]))[slot];
            if on { model(EntityKind::TopoFace, 1 + axis * 2 + s) } else { None }
        })
        .collect();
    b.classify_faces_with(|f| fc[f])?;
    b.finish(alloc)
}
