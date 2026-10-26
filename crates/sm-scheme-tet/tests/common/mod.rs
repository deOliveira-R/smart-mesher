//! Shared helpers for the acceptance tests.
#![allow(dead_code)]

use sm_scheme_tet::{SurfaceInput, TetMesh, ori};
use sm_testkit::surf3::SurfaceFixture;
use std::collections::BTreeMap;

pub fn input_of(f: &SurfaceFixture, v: usize) -> SurfaceInput {
    SurfaceInput { points: f.points.clone(), tris: f.oriented_tris(v) }
}

/// Signed volume of a tet (positive = template orientation).
pub fn tet_vol(m: &TetMesh, t: [u32; 4]) -> f64 {
    let q = t.map(|i| m.points[i as usize]);
    let d = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let (u, v, w) = (d(q[1], q[0]), d(q[2], q[0]), d(q[3], q[0]));
    (u[0] * (v[1] * w[2] - v[2] * w[1]) - u[1] * (v[0] * w[2] - v[2] * w[0]) + u[2] * (v[0] * w[1] - v[1] * w[0])) / 6.0
}

pub fn total_volume(m: &TetMesh) -> f64 {
    sm_math::sum::sum_neumaier(&m.tets.iter().map(|&t| tet_vol(m, t)).collect::<Vec<_>>())
}

/// Every tet strictly positive under exact arithmetic.
pub fn all_exact_positive(m: &TetMesh) -> bool {
    m.tets.iter().all(|t| {
        let q = t.map(|i| m.points[i as usize]);
        ori(q[0], q[1], q[2], q[3]) > 0
    })
}

/// Check that each input triangle is a face of exactly one tet, with the tet on the inner side.
pub fn check_boundary(m: &TetMesh, tris: &[[u32; 3]]) -> Result<(), String> {
    // Map canonical oriented tet faces.
    let canon = |t: [u32; 3]| {
        if t[0] <= t[1] && t[0] <= t[2] { t } else if t[1] <= t[2] { [t[1], t[2], t[0]] } else { [t[2], t[0], t[1]] }
    };
    let mut faces: BTreeMap<[u32; 3], usize> = BTreeMap::new();
    for t in &m.tets {
        for f in [[t[1], t[2], t[3]], [t[0], t[3], t[2]], [t[0], t[1], t[3]], [t[0], t[2], t[1]]] {
            *faces.entry(canon(f)).or_default() += 1;
        }
    }
    for (i, t) in tris.iter().enumerate() {
        match faces.get(&canon(*t)) {
            Some(1) => {}
            other => return Err(format!("tri {i} {t:?}: outward face count {other:?}")),
        }
    }
    // Every outward face whose reverse is absent must be an input triangle.
    let tri_set: std::collections::BTreeSet<[u32; 3]> = tris.iter().map(|t| canon(*t)).collect();
    for f in faces.keys() {
        let rev = canon([f[0], f[2], f[1]]);
        if !faces.contains_key(&rev) && !tri_set.contains(f) {
            return Err(format!("boundary face {f:?} is not an input triangle"));
        }
    }
    Ok(())
}
