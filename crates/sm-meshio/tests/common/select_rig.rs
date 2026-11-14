//! A row of unit hexes in `x`, one volume per entry, with block, sideset and nodeset sets.
#![allow(dead_code)]

use sm_mesh_api::reference::ReferenceMesh;
use sm_mesh_api::{
    CellIx, CellShape, FaceIx, MeshRead, SerialAlloc, SetMembers, Side, SignedFace, VertexIx,
};
use sm_testkit::meshes::{CellSpec, MeshFixture, SetSpec, build_mesh_volumes, with_sets};

/// `vols.len()` unit hexes along x; cell `i` is on `TopoVolume#vols[i]`. Points are numbered
/// `(k * 2 + j) * (n + 1) + i`.
pub fn row_points_cells(vols: &[u64]) -> (Vec<[f64; 3]>, Vec<(CellSpec, u64)>) {
    let n = vols.len();
    let id = |i: usize, j: usize, k: usize| (k * 2 + j) * (n + 1) + i;
    let mut pts = vec![[0.0; 3]; (n + 1) * 4];
    for k in 0..2 {
        for j in 0..2 {
            for i in 0..=n {
                pts[id(i, j, k)] = [i as f64, j as f64, k as f64];
            }
        }
    }
    let cells = (0..n)
        .map(|i| {
            let v = vec![
                id(i, 0, 0),
                id(i + 1, 0, 0),
                id(i + 1, 1, 0),
                id(i, 1, 0),
                id(i, 0, 1),
                id(i + 1, 0, 1),
                id(i + 1, 1, 1),
                id(i, 1, 1),
            ];
            (CellSpec::Template(CellShape::Hex, v), vols[i])
        })
        .collect();
    (pts, cells)
}

/// The boundary sidesets of a row: `xmin`, `xmax`, `ymin`, `ymax`, `zmin`, `zmax` (faces whose
/// loop lies in the plane; one `Left` side each), only the planes that have faces.
pub fn plane_sidesets(m: &dyn MeshRead) -> Vec<SetSpec> {
    let n = m.vertex_count() / 4;
    let planes: [(&str, usize, f64); 6] = [
        ("xmin", 0, 0.0),
        ("xmax", 0, (n - 1) as f64),
        ("ymin", 1, 0.0),
        ("ymax", 1, 1.0),
        ("zmin", 2, 0.0),
        ("zmax", 2, 1.0),
    ];
    planes
        .iter()
        .filter_map(|&(name, axis, v)| {
            let sides: Vec<SignedFace> = (0..m.face_count())
                .filter(|&f| m.face_right(FaceIx::new(f)).is_none())
                .filter(|&f| {
                    m.face_vertices(FaceIx::new(f)).iter().all(|p| {
                        let q = m.point(p.get());
                        [q.x, q.y, q.z][axis] == v
                    })
                })
                .map(|f| SignedFace::new(FaceIx::new(f), Side::Left))
                .collect();
            (!sides.is_empty()).then(|| SetSpec::new(name, SetMembers::Sides(sides)))
        })
        .collect()
}

/// The vertices with `x == 0` as nodeset `nx0`.
pub fn nodeset_x0(m: &dyn MeshRead) -> SetSpec {
    let v = (0..m.vertex_count())
        .filter(|&v| m.point(v).x == 0.0)
        .map(VertexIx::new)
        .collect();
    SetSpec::new("nx0", SetMembers::Vertices(v))
}

/// Cell sets `names[k]` for the cells with volume `k + 1`.
pub fn block_sets(vols: &[u64], names: &[&str]) -> Vec<SetSpec> {
    names
        .iter()
        .enumerate()
        .map(|(k, name)| {
            let want = (k + 1) as u64;
            let c: Vec<CellIx> = vols
                .iter()
                .enumerate()
                .filter(|(_, v)| **v == want)
                .map(|(i, _)| CellIx::new(i))
                .collect();
            SetSpec::new(name, SetMembers::Cells(c))
        })
        .collect()
}

/// A fixture wrapper for [`with_sets`].
pub fn fixture(mesh: ReferenceMesh, n: usize) -> MeshFixture {
    MeshFixture {
        name: "row",
        mesh,
        cell_volumes: vec![1.0; n],
        volume: n as f64,
        area: 0.0,
        boundary_faces: 0,
    }
}

/// A row with the volumes `vols` and the sets: blocks `names`, the plane sidesets (without
/// those in `skip`) and `nx0`.
pub fn row_without(vols: &[u64], names: &[&str], skip: &[&str]) -> ReferenceMesh {
    let (pts, cells) = row_points_cells(vols);
    let mesh = build_mesh_volumes(&pts, &cells, &mut SerialAlloc::new()).expect("mesh");
    let mut specs = block_sets(vols, names);
    specs.extend(
        plane_sidesets(&mesh)
            .into_iter()
            .filter(|s| !skip.contains(&s.name.as_str())),
    );
    specs.push(nodeset_x0(&mesh));
    with_sets(fixture(mesh, vols.len()), &specs)
        .expect("sets")
        .mesh
}

/// [`row_without`] with every sideset.
pub fn row(vols: &[u64], names: &[&str]) -> ReferenceMesh {
    row_without(vols, names, &[])
}
