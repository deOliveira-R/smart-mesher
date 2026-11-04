//! INT-8 and 2D cases: with `empty` or `wedge` patches checkMesh evaluates the cell determinant
//! in the resolved directions only (`polyMesh::calcDirections` -> `geometricD`), so a one-layer
//! hex slab or an axisymmetric wedge mesh is not split.

mod common;

use sm_mesh_api::reference::{ReferenceMesh, hex_block};
use sm_mesh_api::*;
use sm_meshio::foam::*;
use sm_testkit::meshes::{CellSpec, build_mesh};

/// Boundary faces of `m` whose centroid satisfies `pick(centroid)`.
fn boundary_faces(m: &dyn MeshRead, pick: impl Fn([f64; 3]) -> bool) -> Vec<FaceIx> {
    let co = m.coords();
    (0..m.face_count())
        .map(FaceIx::new)
        .filter(|f| {
            if m.face_kind(*f) != FaceKind::Boundary {
                return false;
            }
            let vs = m.face_vertices(*f);
            let n = vs.len() as f64;
            let c = [
                vs.iter().map(|v| co.x[v.get()]).sum::<f64>() / n,
                vs.iter().map(|v| co.y[v.get()]).sum::<f64>() / n,
                vs.iter().map(|v| co.z[v.get()]).sum::<f64>() / n,
            ];
            pick(c)
        })
        .collect()
}

/// An `n x n x 1` slab with face sets `front` (z = 0) and `back` (z = 1).
fn slab(n: u32) -> ReferenceMesh {
    let mut alloc = SerialAlloc::new();
    let mut m = hex_block([n, n, 1], [1.0; 3], &mut alloc).unwrap();
    let front = boundary_faces(&m, |c| c[2] < 0.5);
    let back = boundary_faces(&m, |c| c[2] > 0.5);
    assert_eq!((front.len(), back.len()), ((n * n) as usize, (n * n) as usize));
    MeshEdit::add_set(&mut m, "front", SetMembers::Faces(front), &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, "back", SetMembers::Faces(back), &mut alloc).unwrap();
    m
}

fn slab_opts(front: FoamPatchType, back: FoamPatchType) -> FoamOptions {
    FoamOptions::new().patches(PatchSource::Explicit(vec![PatchSpec::new("front").of_type(front), PatchSpec::new("back").of_type(back)]))
}

const THETA: f64 = 2.5 * std::f64::consts::PI / 180.0;

/// Axisymmetric wedge mesh about the x axis: `nx` cells along x, a prism row on the axis
/// (r in 0..0.5) and a hex row (r in 0.5..1), wedge planes at z = +-r sin(2.5 deg). Face sets
/// `front` (z > 0) and `back` (z < 0).
fn wedge_mesh(nx: usize) -> ReferenceMesh {
    let (s, c) = THETA.sin_cos();
    let mut p: Vec<[f64; 3]> = (0..=nx).map(|i| [i as f64, 0.0, 0.0]).collect();
    for r in [0.5, 1.0] {
        for sign in [1.0, -1.0] {
            for i in 0..=nx {
                p.push([i as f64, r * c, sign * r * s]);
            }
        }
    }
    let n1 = nx + 1;
    let axis = |i: usize| i;
    let front = |j: usize, i: usize| n1 + j * 2 * n1 + i;
    let back = |j: usize, i: usize| n1 + j * 2 * n1 + n1 + i;
    let mut cells = Vec::new();
    for i in 0..nx {
        cells.push(CellSpec::Template(CellShape::Wedge, vec![axis(i), back(0, i), front(0, i), axis(i + 1), back(0, i + 1), front(0, i + 1)]));
    }
    for i in 0..nx {
        cells.push(CellSpec::Template(
            CellShape::Hex,
            vec![back(0, i), back(1, i), front(1, i), front(0, i), back(0, i + 1), back(1, i + 1), front(1, i + 1), front(0, i + 1)],
        ));
    }
    let mut alloc = SerialAlloc::new();
    let mut m = build_mesh(&p, &cells, &mut alloc).unwrap();
    let f = boundary_faces(&m, |c| c[2] > 1e-9);
    let b = boundary_faces(&m, |c| c[2] < -1e-9);
    assert_eq!((f.len(), b.len()), (2 * nx, 2 * nx));
    MeshEdit::add_set(&mut m, "front", SetMembers::Faces(f), &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, "back", SetMembers::Faces(b), &mut alloc).unwrap();
    m
}

fn assert_unsplit(m: &ReferenceMesh, o: &FoamOptions, n_cells: u32) -> PolyMesh {
    let b = build_polymesh(m, o).unwrap();
    assert_eq!(b.mesh.n_cells, n_cells);
    assert!(b.mesh.cell_map.is_empty());
    assert!(b.mesh.files().unwrap().iter().all(|f| f.name != CELL_MAP_FILE));
    assert!(b.report.loss.get("foam.small_det_split").is_none());
    assert!(cell_determinants(&b.mesh).iter().all(|d| *d >= SMALL_DETERMINANT));
    b.mesh
}

#[test]
fn directions_follow_empty_and_wedge_patches() {
    let m = slab(2);
    let plain = build_polymesh(&m, &FoamOptions::new().split_small_determinant(false)).unwrap().mesh;
    assert_eq!(geometric_directions(&plain), [1, 1, 1]);
    let both = build_polymesh(&m, &slab_opts(FoamPatchType::Empty, FoamPatchType::Empty)).unwrap().mesh;
    assert_eq!(geometric_directions(&both), [1, 1, -1]);
    let w = wedge_mesh(2);
    let pm = build_polymesh(&w, &slab_opts(FoamPatchType::Wedge, FoamPatchType::Wedge)).unwrap().mesh;
    assert_eq!(geometric_directions(&pm), [1, 1, -1]);
}

#[test]
fn a_one_layer_slab_with_empty_patches_is_not_split() {
    // (A 1x1 slab is a lone cell: no internal face, determinant 0 in any dimension, flagged.)
    for n in [2u32, 4] {
        let m = slab(n);
        let o = slab_opts(FoamPatchType::Empty, FoamPatchType::Empty);
        let pm = assert_unsplit(&m, &o, n * n);
        assert_eq!(pm.patches.iter().filter(|p| p.patch_type == FoamPatchType::Empty).count(), 2);
        // The same mesh without empty patches is the INT-8 case: every cell is split.
        let plain = build_polymesh(&m, &FoamOptions::new()).unwrap().mesh;
        assert!(plain.n_cells > n * n, "n={n}");
    }
    // Hand values: interior 4 faces -> xx = yy = 2, zz = 1 -> 4/8; corner cell 2 faces -> 1/8.
    let pm = assert_unsplit(&slab(3), &slab_opts(FoamPatchType::Empty, FoamPatchType::Empty), 9);
    let d = cell_determinants(&pm);
    let (lo, hi) = d.iter().fold((f64::MAX, 0.0f64), |(l, h), x| (l.min(*x), h.max(*x)));
    assert!((lo - 0.125).abs() < 1e-14 && (hi - 0.5).abs() < 1e-14, "{lo} {hi}");
    // One empty patch and one plain patch still knock out z (the empty patch alone decides).
    assert_eq!(geometric_directions(&pm), [1, 1, -1]);
}

#[test]
fn only_the_empty_patch_set_decides_the_directions() {
    // Front empty, back a plain patch: the normals still say z.
    let m = slab(2);
    let pm = assert_unsplit(&m, &slab_opts(FoamPatchType::Empty, FoamPatchType::Patch), 4);
    assert_eq!(geometric_directions(&pm), [1, 1, -1]);
    // The split can be turned off outright for any mesh.
    let off = build_polymesh(&m, &FoamOptions::new().split_small_determinant(false)).unwrap();
    assert_eq!(off.mesh.n_cells, 4);
}

#[test]
fn a_one_layer_wedge_case_is_not_split() {
    // (nx = 1 leaves each cell one internal face: rank 1 even in 2D, flagged like checkMesh.)
    for nx in [2usize, 3] {
        let m = wedge_mesh(nx);
        let o = slab_opts(FoamPatchType::Wedge, FoamPatchType::Wedge);
        let pm = assert_unsplit(&m, &o, 2 * nx as u32);
        assert_eq!(pm.patches.iter().filter(|p| p.patch_type == FoamPatchType::Wedge).count(), 2);
        // As plain patches the same mesh is split.
        let plain = build_polymesh(&m, &FoamOptions::new()).unwrap().mesh;
        assert!(plain.n_cells > 2 * nx as u32, "nx={nx}");
    }
}

#[test]
fn the_boundary_file_round_trips_empty_and_wedge() {
    for t in [FoamPatchType::Empty, FoamPatchType::Wedge] {
        let b = build_polymesh(&slab(2), &slab_opts(t, t)).unwrap();
        let files = b.mesh.files().unwrap();
        let text = String::from_utf8(files.iter().find(|f| f.name == "boundary").unwrap().bytes.clone()).unwrap();
        assert!(text.contains(&format!("type            {};", t.as_str())), "{text}");
        let parsed = parse_polymesh(&files).unwrap();
        assert_eq!(parsed, b.mesh);
        assert_eq!(FoamPatchType::from_word(t.as_str()), Some(t));
    }
}

#[test]
fn the_2d_export_is_deterministic_over_threads() {
    let hash = |threads: usize| {
        rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(|| {
            let mut h = 0xcbf2_9ce4_8422_2325u64;
            for (m, t) in [(slab(4), FoamPatchType::Empty), (wedge_mesh(3), FoamPatchType::Wedge)] {
                for f in build_polymesh(&m, &slab_opts(t, t)).unwrap().mesh.files().unwrap() {
                    for x in f.name.as_bytes().iter().chain(&f.bytes) {
                        h = (h ^ u64::from(*x)).wrapping_mul(0x0100_0000_01b3);
                    }
                }
            }
            h
        })
    };
    let base = hash(1);
    assert_eq!(base, hash(1));
    assert_eq!(base, hash(4));
    assert_eq!(base, hash(0));
}

/// Writes the 2D fixtures to `$SM_FOAM2D_OUT/{slab,wedge}/constant/polyMesh` for checkMesh runs.
#[test]
fn emit_2d_cases_for_checkmesh() {
    let Ok(out) = std::env::var("SM_FOAM2D_OUT") else { return };
    let out = std::path::PathBuf::from(out);
    let r = write_polymesh(&slab(4), &slab_opts(FoamPatchType::Empty, FoamPatchType::Empty), &out.join("slab/constant/polyMesh")).unwrap();
    assert_eq!(r.n_cells, 16);
    let r = write_polymesh(&wedge_mesh(4), &slab_opts(FoamPatchType::Wedge, FoamPatchType::Wedge), &out.join("wedge/constant/polyMesh")).unwrap();
    assert_eq!(r.n_cells, 8);
}
