//! The polyhedra policy shared by the exporters whose format has no general polyhedron.

use serde::{Deserialize, Serialize};

/// What to do with polyhedral cells in a format that cannot carry them (Gmsh msh4, Abaqus).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[non_exhaustive]
pub enum PolyPolicy {
    /// Fail with the writer's polyhedra error.
    #[default]
    Error,
    /// Split each polyhedron into tetrahedra around a cell centroid and face centroids (new
    /// nodes after the original ones). Allowed only when every neighbour across a polygonal
    /// face is also decomposed, else the writer's non-conforming-decomposition error.
    Decompose,
    /// Omit polyhedra and report `<format>.poly_dropped`.
    Drop,
}

/// Why a polyhedron could not be fanned into tetrahedra.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum FanError {
    /// The cell indexes a vertex that does not exist.
    MissingVertex { cell: usize },
    /// A face has fewer than 3 vertices or a missing vertex.
    BadPolygon { face: u32 },
    /// The tet ids ran past `u32::MAX` (the writer's id space).
    TooManyIds,
    /// A tetrahedron of the fan has a volume <= 0 (a non-star-shaped cell or a flat face).
    Inverted { cell: usize, face: u32 },
}

fn centroid(ids: &[u32], pts: &[[f64; 3]]) -> [f64; 3] {
    let mut c = [0.0; 3];
    for &i in ids {
        for (k, ck) in c.iter_mut().enumerate() {
            *ck += pts[i as usize][k] / ids.len() as f64;
        }
    }
    c
}

/// Six times the volume of the tetrahedron `t` (positive for the node order the writers emit).
fn det6(t: [u32; 4], pts: &[[f64; 3]]) -> f64 {
    let p = |i: u32| pts[i as usize];
    let (a, b, c, d) = (p(t[0]), p(t[1]), p(t[2]), p(t[3]));
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let w = [d[0] - a[0], d[1] - a[1], d[2] - a[2]];
    u[0] * (v[1] * w[2] - v[2] * w[1]) - u[1] * (v[0] * w[2] - v[2] * w[0]) + u[2] * (v[0] * w[1] - v[1] * w[0])
}

/// Fan polyhedron `cell` into tetrahedra around its vertex-average centroid (a new node pushed
/// onto `pts`) and, for faces with 4 or more vertices, a face-centroid node (shared through
/// `face_centre`). Triangles give one tet `[l0, l2, l1, centre]`, polygons one tet
/// `[a, face_centre, b, centre]` per edge, where `loop` is the face loop oriented outward.
/// `node` is called after every node pushed and `tet(face, nodes)` for every tet, in a fixed
/// order. A tet with det <= 0 is refused (the writers would hand an inverted element on).
pub(crate) fn fan_cell(
    mesh: &dyn sm_mesh_api::MeshRead,
    cell: usize,
    nv: usize,
    pts: &mut Vec<[f64; 3]>,
    face_centre: &mut std::collections::BTreeMap<u32, u32>,
    mut node: impl FnMut(),
    mut tet: impl FnMut(u32, [u32; 4]) -> Result<(), FanError>,
) -> Result<(), FanError> {
    use sm_mesh_api::{CellIx, Side};
    let ci = CellIx::new(cell);
    let mut vs: Vec<u32> = mesh.cell_faces(ci).iter().flat_map(|sf| mesh.face_vertices(sf.face()).iter().map(|v| v.0)).collect();
    vs.sort_unstable();
    vs.dedup();
    if vs.iter().any(|&v| v as usize >= nv) {
        return Err(FanError::MissingVertex { cell });
    }
    let cc = pts.len() as u32;
    let c = centroid(&vs, pts);
    pts.push(c);
    node();
    let mut emit = |f: u32, t: [u32; 4], pts: &[[f64; 3]]| -> Result<(), FanError> {
        // NaN fails the comparison too.
        if det6(t, pts) <= 0.0 || det6(t, pts).is_nan() {
            return Err(FanError::Inverted { cell, face: f });
        }
        tet(f, t)
    };
    for sf in mesh.cell_faces(ci) {
        let f = sf.face();
        let mut loop_: Vec<u32> = mesh.face_vertices(f).iter().map(|v| v.0).collect();
        if loop_.len() < 3 || loop_.iter().any(|&v| v as usize >= nv) {
            return Err(FanError::BadPolygon { face: f.0 });
        }
        if sf.side() == Side::Right {
            loop_.reverse();
        }
        let n = loop_.len();
        if n == 3 {
            emit(f.0, [loop_[0], loop_[2], loop_[1], cc], pts)?;
            continue;
        }
        let fc = match face_centre.get(&f.0) {
            Some(&id) => id,
            None => {
                let id = pts.len() as u32;
                let p = centroid(&loop_, pts);
                pts.push(p);
                node();
                face_centre.insert(f.0, id);
                id
            }
        };
        for k in 0..n {
            emit(f.0, [loop_[k], fc, loop_[(k + 1) % n], cc], pts)?;
        }
    }
    Ok(())
}
