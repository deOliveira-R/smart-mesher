//! Structured hex block generator (useful for tests and as a worked example of the builder).

use super::{Mesh, MeshBuilder};
use crate::error::MeshError;
use crate::shape::CellShape;
use crate::traits::{CellSpec, MeshEdit};
use sm_core::{EntityId, Point3};
use smallvec::SmallVec;

/// Corner offsets `(di, dj, dk)` of the six outward-wound hex faces
/// (x-, x+, y-, y+, z-, z+).
const FACES: [[[u32; 3]; 4]; 6] = [
    [[0, 0, 0], [0, 0, 1], [0, 1, 1], [0, 1, 0]],
    [[1, 0, 0], [1, 1, 0], [1, 1, 1], [1, 0, 1]],
    [[0, 0, 0], [1, 0, 0], [1, 0, 1], [0, 0, 1]],
    [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]],
    [[0, 0, 0], [0, 1, 0], [1, 1, 0], [1, 0, 0]],
    [[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]],
];

/// Builder holding an `nx*ny*nz` hex block of overall `size` at `origin`, cells ordered with
/// `i` fastest. Not yet finished, so callers can add classification and labels.
pub fn hex_block_builder(
    mesh: EntityId,
    serial_base: u64,
    n: [u32; 3],
    origin: [f64; 3],
    size: [f64; 3],
) -> Result<MeshBuilder, MeshError> {
    let mut b = MeshBuilder::new(mesh, serial_base);
    for k in 0..=n[2] {
        for j in 0..=n[1] {
            for i in 0..=n[0] {
                let f = |a: u32, o: f64, s: f64, m: u32| o + s * f64::from(a) / f64::from(m);
                b.add_vertex(Point3::new(f(i, origin[0], size[0], n[0]), f(j, origin[1], size[1], n[1]), f(k, origin[2], size[2], n[2])), None);
            }
        }
    }
    let vid = |i: u32, j: u32, k: u32| crate::point::Point::vertex(i + (n[0] + 1) * (j + (n[1] + 1) * k));
    for k in 0..n[2] {
        for j in 0..n[1] {
            for i in 0..n[0] {
                let faces = FACES
                    .iter()
                    .map(|f| f.iter().map(|d| vid(i + d[0], j + d[1], k + d[2])).collect())
                    .collect::<SmallVec<_>>();
                b.add_cell(&CellSpec { shape: CellShape::Hex, faces, class: None })?;
            }
        }
    }
    Ok(b)
}

/// Finished `nx*ny*nz` hex block (see [`hex_block_builder`]).
pub fn hex_block(mesh: EntityId, serial_base: u64, n: [u32; 3], origin: [f64; 3], size: [f64; 3]) -> Result<Mesh, MeshError> {
    hex_block_builder(mesh, serial_base, n, origin, size)?.finish()
}
