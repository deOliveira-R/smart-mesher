//! Degenerate ray families for exactness (simulation of simplicity) tests.

use sm_core::{Point3, Vector3};

/// The 7 directions along which the Kuhn tetrahedral lattice has edges: three axes, three face
/// diagonals and the main diagonal, all with non-negative integer components.
pub const KUHN_DIRECTIONS: [[i32; 3]; 7] = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1], [1, 1, 1]];

/// Rays that hit every vertex of the integer lattice `[0, n]^3` exactly and run along its Kuhn
/// edges (the lattice of `meshes::tet_block(n, [1.0; 3])`).
///
/// For each direction `d` in [`KUHN_DIRECTIONS`] (outer loop) and each lattice vertex `v`
/// (`k`, then `j`, then `i` fastest), the ray starts at `v - d` and has the unnormalised integer
/// direction `d`, so `origin + d == v` exactly in floating point. Each ray therefore passes
/// through a vertex, contains the lattice edge `v -> v + d` where that edge exists (the axis,
/// face-diagonal and main-diagonal "fan" edges around the main diagonal), and lies in lattice
/// faces. Every query is degenerate for a tetrahedral lattice, which is what a symbolic
/// perturbation has to resolve. `n` is raised to at least 1; the result has `7 (n + 1)^3`
/// entries.
pub fn degenerate_grid(n: u32) -> Vec<(Point3, Vector3)> {
    let n = n.max(1) as i32;
    let mut out = Vec::with_capacity(KUHN_DIRECTIONS.len() * ((n + 1) as usize).pow(3));
    for d in KUHN_DIRECTIONS {
        for k in 0..=n {
            for j in 0..=n {
                for i in 0..=n {
                    let o = [i - d[0], j - d[1], k - d[2]].map(f64::from);
                    out.push((Point3::new(o[0], o[1], o[2]), Vector3::new(f64::from(d[0]), f64::from(d[1]), f64::from(d[2]))));
                }
            }
        }
    }
    out
}
