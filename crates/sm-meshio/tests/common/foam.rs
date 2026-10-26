//! Helpers for the OpenFOAM tests: meshes with sets, and a geometry oracle written
//! independently of the crate's checker (Newell area vectors and the divergence theorem about the origin).
#![allow(dead_code)]

use sm_meshio::foam::PolyMesh;
use sm_mesh_api::reference::{ReferenceMesh, hex_block};
use sm_mesh_api::*;

pub const SIDES: [&str; 6] = ["xmin", "xmax", "ymin", "ymax", "zmin", "zmax"];

/// The face indices of `m` that are boundary faces lying on `axis = value`.
pub fn faces_on(m: &dyn MeshRead, axis: usize, value: f64) -> Vec<FaceIx> {
    let co = m.coords();
    let c = [co.x, co.y, co.z][axis];
    (0..m.face_count())
        .map(FaceIx::new)
        .filter(|f| m.face_kind(*f) == FaceKind::Boundary && m.face_vertices(*f).iter().all(|v| (c[v.get()] - value).abs() < 1e-12))
        .collect()
}

/// A 2x2x2 unit block (h = 1/2) with face sets xmin xmax ymin ymax zmin zmax (set ids ascending in
/// that order, 4 faces each) and cell sets b1 (cells 0..4) and b2 (cells 4..8).
pub fn sets_block() -> ReferenceMesh {
    let mut alloc = SerialAlloc::new();
    let mut m = hex_block([2, 2, 2], [0.5; 3], &mut alloc).unwrap();
    for (i, name) in SIDES.iter().enumerate() {
        let f = faces_on(&m, i / 2, (i % 2) as f64);
        assert_eq!(f.len(), 4);
        MeshEdit::add_set(&mut m, name, SetMembers::Faces(f), &mut alloc).unwrap();
    }
    MeshEdit::add_set(&mut m, "b1", SetMembers::Cells((0..4).map(CellIx::new).collect()), &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, "b2", SetMembers::Cells((4..8).map(CellIx::new).collect()), &mut alloc).unwrap();
    m
}

type V3 = [f64; 3];

fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// Newell area vector of face `i` of the polyMesh (half the sum of `p_i x p_{i+1}`).
pub fn area_vector(pm: &PolyMesh, i: usize) -> V3 {
    let f = pm.face(i);
    let mut s = [0.0; 3];
    for k in 0..f.len() {
        let (a, b) = (pm.points[f[k] as usize], pm.points[f[(k + 1) % f.len()] as usize]);
        let c = cross(a, b);
        for d in 0..3 {
            s[d] += c[d] / 2.0;
        }
    }
    s
}

/// Per cell: `(volume, |sum of outward Sf|, sum |Sf|)` from the foam arrays only. The volume is
/// `1/3 sum Sf . p0` (exact for planar faces); a cell sees `+Sf` as owner and `-Sf` as neighbour.
pub fn cell_stats(pm: &PolyMesh) -> Vec<(f64, f64, f64)> {
    let nc = pm.n_cells as usize;
    let mut vol = vec![0.0; nc];
    let mut sum = vec![[0.0; 3]; nc];
    let mut abs = vec![0.0; nc];
    for i in 0..pm.n_faces() {
        let sf = area_vector(pm, i);
        let p0 = pm.points[pm.face(i)[0] as usize];
        let len = (sf[0] * sf[0] + sf[1] * sf[1] + sf[2] * sf[2]).sqrt();
        let dotp = sf[0] * p0[0] + sf[1] * p0[1] + sf[2] * p0[2];
        let o = pm.owner[i] as usize;
        vol[o] += dotp / 3.0;
        abs[o] += len;
        for d in 0..3 {
            sum[o][d] += sf[d];
        }
        if i < pm.n_internal() {
            let n = pm.neighbour[i] as usize;
            vol[n] -= dotp / 3.0;
            abs[n] += len;
            for d in 0..3 {
                sum[n][d] -= sf[d];
            }
        }
    }
    (0..nc).map(|c| (vol[c], (sum[c][0].powi(2) + sum[c][1].powi(2) + sum[c][2].powi(2)).sqrt(), abs[c])).collect()
}
