//! Shared fixtures for the contract tests.
#![allow(dead_code, missing_docs)]

use sm_core::{EntityId, EntityKind, Point3, Transform, Vector3};
use sm_mesh_api::reference::{Mesh, RefBuilder};
use sm_mesh_api::{CellShape, MeshBuilder, VertexIx};

pub fn id(kind: EntityKind, serial: u64) -> EntityId {
    EntityId::new(kind, serial).expect("valid serial")
}

pub fn mesh_id(serial: u64) -> EntityId {
    id(EntityKind::Mesh, serial)
}

/// Builder holding an `nx x ny x nz` hex block with spacing `h`, vertex index `i + (nx+1)(j + (ny+1)k)`.
pub fn block_builder(serial: u64, n: [usize; 3], h: [f64; 3]) -> RefBuilder {
    let mut b = Mesh::builder(mesh_id(serial)).expect("mesh id");
    for k in 0..=n[2] {
        for j in 0..=n[1] {
            for i in 0..=n[0] {
                b.add_vertex(Point3::new(i as f64 * h[0], j as f64 * h[1], k as f64 * h[2]));
            }
        }
    }
    let v = |i: usize, j: usize, k: usize| VertexIx((i + (n[0] + 1) * (j + (n[1] + 1) * k)) as u32);
    for k in 0..n[2] {
        for j in 0..n[1] {
            for i in 0..n[0] {
                let c = [
                    v(i, j, k),
                    v(i + 1, j, k),
                    v(i + 1, j + 1, k),
                    v(i, j + 1, k),
                    v(i, j, k + 1),
                    v(i + 1, j, k + 1),
                    v(i + 1, j + 1, k + 1),
                    v(i, j + 1, k + 1),
                ];
                b.add_cell(CellShape::Hex, &c).expect("hex");
            }
        }
    }
    b
}

pub fn block(serial: u64, n: [usize; 3], h: [f64; 3]) -> Mesh {
    block_builder(serial, n, h).finish().expect("finish")
}

/// Mirror in the plane `x = 0`.
pub fn mirror_x() -> Transform {
    Transform { m: [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: Vector3::new(0.0, 0.0, 0.0) }
}
