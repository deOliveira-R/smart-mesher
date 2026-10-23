//! Canned reference meshes used as fixtures and by documentation examples.

use super::{ReferenceBuilder, ReferenceMesh};
use crate::build::{MeshBuild, SerialAlloc};
use crate::classify::Classification;
use crate::error::{MeshError, Result};
use crate::index::{MeshEntityRef, VertexIx};
use crate::shape::CellShape;
use sm_core::{EntityId, EntityKind, Point3};

fn model(kind: EntityKind, serial: u64) -> Result<Classification> {
    Ok(Classification::on(
        EntityId::new(kind, serial).ok_or(MeshError::SerialOverflow)?,
    ))
}

/// An `n[0] x n[1] x n[2]` block of hexahedra with cell sizes `h`, from the origin, classified
/// like a CAD box: 8 `TopoVertex`, 12 `TopoEdge`, 6 `TopoFace` (`-x,+x,-y,+y,-z,+z` are
/// `TopoFace#1..6`) and one `TopoVolume#1`. No edge table.
///
/// Fails with [`MeshError::State`] if any `n[i]` is zero.
pub fn hex_block(n: [u32; 3], h: [f64; 3], alloc: &mut SerialAlloc) -> Result<ReferenceMesh> {
    build_block(n, h, alloc, false)
}

/// Like [`hex_block`] but with a fully classified edge table: edges along model edges lie on
/// `TopoEdge`, edges inside a model face on that `TopoFace`, interior edges on the volume.
pub fn hex_block_with_edges(
    n: [u32; 3],
    h: [f64; 3],
    alloc: &mut SerialAlloc,
) -> Result<ReferenceMesh> {
    build_block(n, h, alloc, true)
}

fn build_block(
    n: [u32; 3],
    h: [f64; 3],
    alloc: &mut SerialAlloc,
    edges: bool,
) -> Result<ReferenceMesh> {
    if n.contains(&0) {
        return Err(MeshError::State(
            "hex_block needs at least one cell per axis".into(),
        ));
    }
    let mut b = ReferenceBuilder::new();
    let m = [n[0] as usize + 1, n[1] as usize + 1, n[2] as usize + 1];
    let ix = |i: usize, j: usize, k: usize| VertexIx::new((k * m[1] + j) * m[0] + i);
    let ext = |a: usize, q: usize| q == 0 || q == n[a] as usize;
    let side = |a: usize, q: usize| usize::from(q == n[a] as usize);
    // Class of a lattice entity: per axis either a lattice index (Some) or free (None).
    // Axes on the box surface are "fixed"; the rest slide along the entity.
    let class_of = |pos: [Option<usize>; 3]| -> Result<Classification> {
        let fixed: Vec<usize> = (0..3)
            .filter(|&a| pos[a].is_some_and(|q| ext(a, q)))
            .collect();
        let sliding: Vec<usize> = (0..3).filter(|a| !fixed.contains(a)).collect();
        let sd = |a: usize| side(a, pos[a].unwrap_or(0));
        match fixed.len() {
            3 => model(
                EntityKind::TopoVertex,
                1 + (sd(0) + 2 * sd(1) + 4 * sd(2)) as u64,
            ),
            2 => model(
                EntityKind::TopoEdge,
                1 + (sliding[0] * 4 + sd(fixed[0]) + 2 * sd(fixed[1])) as u64,
            ),
            1 => model(
                EntityKind::TopoFace,
                1 + (fixed[0] * 2 + sd(fixed[0])) as u64,
            ),
            _ => model(EntityKind::TopoVolume, 1),
        }
    };
    for k in 0..m[2] {
        for j in 0..m[1] {
            for i in 0..m[0] {
                let v = b.add_vertex(Point3::new(
                    i as f64 * h[0],
                    j as f64 * h[1],
                    k as f64 * h[2],
                ))?;
                b.set_class(
                    MeshEntityRef::Vertex(v),
                    class_of([Some(i), Some(j), Some(k)])?,
                )?;
            }
        }
    }
    for k in 0..n[2] as usize {
        for j in 0..n[1] as usize {
            for i in 0..n[0] as usize {
                let vs = [
                    ix(i, j, k),
                    ix(i + 1, j, k),
                    ix(i + 1, j + 1, k),
                    ix(i, j + 1, k),
                    ix(i, j, k + 1),
                    ix(i + 1, j, k + 1),
                    ix(i + 1, j + 1, k + 1),
                    ix(i, j + 1, k + 1),
                ];
                let added = b.add_cell_from_vertices(CellShape::Hex, &vs)?;
                b.set_class(
                    MeshEntityRef::Cell(added.cell),
                    model(EntityKind::TopoVolume, 1)?,
                )?;
                // Template face order: -z, +z, -y, +x, +y, -x.
                let faces_on = [
                    (2usize, 0usize, k == 0),
                    (2, 1, k + 1 == n[2] as usize),
                    (1, 0, j == 0),
                    (0, 1, i + 1 == n[0] as usize),
                    (1, 1, j + 1 == n[1] as usize),
                    (0, 0, i == 0),
                ];
                for (sf, (axis, s, on)) in added.faces.iter().zip(faces_on) {
                    if on {
                        b.set_class(
                            MeshEntityRef::Face(sf.face()),
                            model(EntityKind::TopoFace, 1 + (axis * 2 + s) as u64)?,
                        )?;
                    }
                }
            }
        }
    }
    if edges {
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
                        let nb = (0..3).filter(|&x| x != a && ext(x, p[x])).count();
                        if nb == 0 {
                            continue;
                        }
                        let mut pos = [Some(p[0]), Some(p[1]), Some(p[2])];
                        pos[a] = None;
                        b.set_class(
                            MeshEntityRef::Edge(ix(p[0], p[1], p[2]), ix(q[0], q[1], q[2])),
                            class_of(pos)?,
                        )?;
                    }
                }
            }
        }
    }
    Ok(b.finish(alloc)?.mesh)
}
