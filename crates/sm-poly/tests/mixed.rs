//! W4-POLY-MIXED: `dualize` on mixed meshes (tets plus wedges, pyramids, hexes).
//! Tests have no RNG; nothing here writes to the temp dir.
#![allow(missing_docs)]

mod common;

use common::*;
use sm_core::EntityKind;
use sm_mesh_api::reference::ReferenceMesh;
use sm_mesh_api::{
    CellIx, CellShape, FaceIx, GeomTol, MeshEntityKind, MeshRead, SerialAlloc, geometry_hash, topology_hash, validate,
    validate_geometry,
};
use sm_poly::{DualPoints, DualReport, FaceMode, PolyError, PolyOptions};
use sm_testkit::meshes::{self, CellSpec, build_mesh_volumes};

type P = [f64; 3];

fn det(p: &[P; 4]) -> f64 {
    let s = |a: P, b: P| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let (u, v, w) = (s(p[1], p[0]), s(p[2], p[0]), s(p[3], p[0]));
    u[0] * (v[1] * w[2] - v[2] * w[1]) - u[1] * (v[0] * w[2] - v[2] * w[0]) + u[2] * (v[0] * w[1] - v[1] * w[0])
}

/// A positively oriented tet cell spec.
fn tet(pts: &[P], mut ix: [usize; 4]) -> CellSpec {
    if det(&ix.map(|i| pts[i])) < 0.0 {
        ix.swap(2, 3);
    }
    CellSpec::Template(CellShape::Tet, ix.to_vec())
}

/// An `n x n` square footprint of unit squares, each split along its diagonal into two
/// triangles, over two layers: wedges for `z in [0, 1]` (volume 1) and the same prisms cut
/// into three tets each, by the global vertex order, for `z in [1, 2]` (volume 2).
fn wedge_tet_stack(n: usize) -> (ReferenceMesh, f64) {
    let w = n + 1;
    let layer = w * w;
    let mut pts: Vec<P> = Vec::new();
    for k in 0..3 {
        for j in 0..w {
            for i in 0..w {
                pts.push([i as f64, j as f64, k as f64]);
            }
        }
    }
    let id = |i: usize, j: usize, k: usize| k * layer + j * w + i;
    let mut cells: Vec<(CellSpec, u64)> = Vec::new();
    let mut tris: Vec<[usize; 3]> = Vec::new();
    for j in 0..n {
        for i in 0..n {
            tris.push([id(i, j, 0), id(i + 1, j, 0), id(i + 1, j + 1, 0)]);
            tris.push([id(i, j, 0), id(i + 1, j + 1, 0), id(i, j + 1, 0)]);
        }
    }
    for t in &tris {
        let up = |k: usize| t.map(|v| v + k * layer);
        let (b, m) = (up(0), up(1));
        cells.push((CellSpec::Template(CellShape::Wedge, vec![b[0], b[1], b[2], m[0], m[1], m[2]]), 1));
    }
    for t in &tris {
        let (b, q) = (t.map(|v| v + layer), t.map(|v| v + 2 * layer));
        let mut o = [0usize, 1, 2];
        o.sort_by_key(|&k| b[k]);
        let (a, bb, c) = (o[0], o[1], o[2]);
        for ix in [[b[a], b[bb], b[c], q[c]], [b[a], b[bb], q[bb], q[c]], [b[a], q[a], q[bb], q[c]]] {
            cells.push((tet(&pts, ix), 2));
        }
    }
    let mesh = build_mesh_volumes(&pts, &cells, &mut SerialAlloc::new()).expect("stack");
    (mesh, (n * n * 2) as f64)
}

/// A hex (volume 1) under a pyramid and a tet (volume 2): the hex|pyramid face is a model face,
/// the pyramid|tet face lies inside volume 2.
fn hex_pyramid_tet() -> ReferenceMesh {
    let mut p: Vec<P> = vec![
        [0., 0., 0.],
        [1., 0., 0.],
        [1., 1., 0.],
        [0., 1., 0.],
        [0., 0., 1.],
        [1., 0., 1.],
        [1., 1., 1.],
        [0., 1., 1.],
    ];
    p.push([0.5, 0.5, 2.0]); // 8
    p.push([0.5, -1.0, 1.5]); // 9
    let cells = vec![
        (CellSpec::Template(CellShape::Hex, (0..8).collect()), 1),
        (CellSpec::Template(CellShape::Pyramid, vec![4, 5, 6, 7, 8]), 2),
        (tet(&p, [4, 5, 8, 9]), 2),
    ];
    build_mesh_volumes(&p, &cells, &mut SerialAlloc::new()).expect("hex pyramid tet")
}

fn run(m: &ReferenceMesh, opts: &PolyOptions) -> (ReferenceMesh, DualReport) {
    dual_ref(m, opts, &PairResolver).unwrap_or_else(|e| panic!("{e}"))
}

fn total_volume(m: &dyn MeshRead) -> f64 {
    volumes(m).iter().sum()
}

/// Interface faces are shared: no boundary face of the output lies inside the source's interior,
/// so the output's boundary area equals the source's.
fn boundary_area(m: &dyn MeshRead) -> f64 {
    (0..m.count(MeshEntityKind::Face))
        .filter(|&f| m.is_boundary_face(FaceIx::new(f)))
        .map(|f| sm_mesh_api::face_area_vector(m, FaceIx::new(f)).norm())
        .sum()
}

fn check_valid(m: &ReferenceMesh) {
    check_valid_with(m, &GeomTol::default());
}

fn check_valid_with(m: &ReferenceMesh, tol: &GeomTol) {
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert!(validate_geometry(m, tol).is_empty(), "{:?}", validate_geometry(m, tol));
}

#[test]
fn acc1_wedge_tet_stack_dualizes_and_conserves_volume() {
    for mode in [FaceMode::Planar, FaceMode::Keep] {
        let (src, vol) = wedge_tet_stack(3);
        let opts = PolyOptions::new().with_faces(mode);
        let (out, rep) = run(&src, &opts);
        // `Keep` emits one polygon per dual edge, which need not be planar
        let tol = if mode == FaceMode::Keep { GeomTol::new().with_planarity(f64::INFINITY) } else { GeomTol::default() };
        check_valid_with(&out, &tol);
        assert!(rel(total_volume(&out), vol) < 1e-12, "{mode:?} {} vs {vol}", total_volume(&out));
        assert!(rel(boundary_area(&out), boundary_area(&src)) < 1e-12);
        // the 18 wedges are kept (as polyhedra: their transition triangle is cut into kites)
        assert_eq!(rep.kept_cells, 18);
        // tet region: one dual cell per vertex of the tet layer (the 4x4 vertices of z = 1 and 2,
        // all in volume 2); the wedge volume has its 18 cells
        let tet_vertices = 2 * 16;
        assert_eq!(rep.cells, tet_vertices + 18, "{mode:?}");
        // the two volumes keep their volume (9 + ... = 9 each)
        let v: Vec<f64> = volumes(&out);
        let (mut v1, mut v2) = (0.0, 0.0);
        for (c, x) in v.iter().enumerate() {
            let cls = out.cell_class(CellIx::new(c)).entity;
            if cls.serial() == 1 { v1 += x } else { v2 += x }
        }
        assert!(rel(v1, 9.0) < 1e-12 && rel(v2, 9.0) < 1e-12, "{v1} {v2}");
    }
}

#[test]
fn acc1_edge_table_of_a_mixed_result_classifies_every_edge() {
    let (src, vol) = wedge_tet_stack(2);
    let (out, _) = run(&src, &PolyOptions::new().with_edges(Some(true)));
    assert!(out.edges().is_some());
    check_valid(&out);
    assert!(rel(total_volume(&out), vol) < 1e-12);
}

#[test]
fn acc1_transition_triangles_are_shared_and_cut_into_three() {
    let (src, _) = wedge_tet_stack(2);
    let (out, _) = run(&src, &PolyOptions::new());
    // 8 transition triangles at z = 1, each cut into 3 kites shared by a dual cell and a wedge
    let shared: Vec<usize> = (0..out.count(MeshEntityKind::Face))
        .filter(|&f| {
            let fx = FaceIx::new(f);
            out.face_right(fx).is_some()
                && out.cell_class(out.face_left(fx)).entity != out.cell_class(out.face_right(fx).unwrap()).entity
        })
        .collect();
    assert_eq!(shared.len(), 8 * 3);
    for &f in &shared {
        assert_eq!(out.face_vertices(FaceIx::new(f)).len(), 4);
    }
}

#[test]
fn acc1_hex_pyramid_tet_with_a_volume_interior_transition() {
    let src = hex_pyramid_tet();
    let want = 1.0 + 1.0 / 3.0 + det(&[[0.5, 0.5, 2.0], [0., 0., 1.], [1., 0., 1.], [0.5, -1.0, 1.5]]).abs() / 6.0;
    let (out, rep) = run(&src, &PolyOptions::new());
    check_valid(&out);
    assert!(rel(total_volume(&out), want) < 1e-12, "{} {want}", total_volume(&out));
    assert_eq!(rep.kept_cells, 2);
    assert_eq!(rep.cells, 4 + 2);
    assert!(rel(boundary_area(&out), boundary_area(&src)) < 1e-12);
    // the hex keeps its volume exactly
    let hex = (0..out.count(MeshEntityKind::Cell)).find(|&c| out.cell_class(CellIx::new(c)).entity.serial() == 1).unwrap();
    assert!(rel(volumes(&out)[hex], 1.0) < 1e-14);
}

#[test]
fn acc1_mixed_transition_fixture_all_in_one_volume() {
    let f = meshes::mixed_transition().unwrap();
    let (out, rep) = run(&f.mesh, &PolyOptions::new());
    check_valid(&out);
    assert!(rel(total_volume(&out), f.volume) < 1e-12);
    assert!(rel(boundary_area(&out), f.area) < 1e-12);
    assert_eq!(rep.kept_cells, 3);
    // non-tet cells keep their exact volumes (hex, wedge, pyramid sit after the 4 dual cells)
    let v = volumes(&out);
    for (c, want) in [(4, 1.0), (5, 0.5), (6, 1.0 / 3.0)] {
        assert!(rel(v[c], want) < 1e-12, "cell {c}: {} vs {want}", v[c]);
    }
}

#[test]
fn acc1_untouched_non_tet_cells_keep_their_shape() {
    // a wedge layer with no tets next to some wedges: those stay primitive wedges
    let (src, _) = wedge_tet_stack(1);
    let (out, _) = run(&src, &PolyOptions::new());
    check_valid(&out);
    let shapes: Vec<CellShape> = (0..out.count(MeshEntityKind::Cell)).map(|c| out.cell_shape(CellIx::new(c))).collect();
    assert!(shapes.iter().all(|s| *s == CellShape::Polyhedron || *s == CellShape::Wedge || *s == CellShape::Hex));
}

#[test]
fn acc2_all_tet_output_is_unchanged() {
    // Hashes taken at the commit before W4-POLY-MIXED (212689e).
    let cases: [(&str, ReferenceMesh, u64, u64); 3] = [
        ("tet_block", meshes::tet_block([4, 3, 3], [1.0, 0.7, 1.3]).unwrap().mesh, PIN_BLOCK.0, PIN_BLOCK.1),
        ("two_volumes", meshes::tet_block_two_volumes([4, 2, 2], [1.0; 3], 2).unwrap().mesh, PIN_TWO.0, PIN_TWO.1),
        ("lshape", meshes::tet_lshape(2, 1.0).unwrap().mesh, PIN_L.0, PIN_L.1),
    ];
    for (name, m, topo, geo) in cases {
        let opts = PolyOptions::new().with_edges(Some(true));
        let (out, _) = dual_ref(&m, &opts, &sm_poly::VertexClassResolver).unwrap();
        assert_eq!((topology_hash(&out), geometry_hash(&out)), (topo, geo), "{name}");
    }
}

const PIN_BLOCK: (u64, u64) = (16537787867412839815, 12505874610224242311);
const PIN_TWO: (u64, u64) = (16931969246938507279, 14214863444417510811);
const PIN_L: (u64, u64) = (16712207976776051621, 8165287658278682185);

#[test]
fn acc3_mixed_dual_is_deterministic_across_threads() {
    let (src, _) = wedge_tet_stack(3);
    let hashes = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| {
            let (m, _) = dual_prod(&src, &PolyOptions::new(), &PairResolver).unwrap();
            (topology_hash(&m), geometry_hash(&m))
        })
    };
    let h = sm_testkit::determinism::check(2, 4, hashes).expect("deterministic");
    let (m, _) = dual_prod(&src, &PolyOptions::new(), &PairResolver).unwrap();
    assert_eq!(h, (topology_hash(&m), geometry_hash(&m)));
}

#[test]
fn acc3_production_mesh_builder_agrees_with_the_reference_builder() {
    let (src, vol) = wedge_tet_stack(2);
    let (m, _) = dual_prod(&src, &PolyOptions::new().with_all_polyhedra(true), &PairResolver).unwrap();
    assert!(validate(&m).is_empty());
    assert!(rel(total_volume(&m), vol) < 1e-12);
}

#[test]
fn unsupported_options_are_refused_typed_and_leave_alloc_alone() {
    let (src, _) = wedge_tet_stack(1);
    let mut alloc = SerialAlloc::new();
    let before: Vec<_> = alloc.state().collect();
    let r = sm_poly::dualize(
        &src,
        &PolyOptions::new().with_dual(DualPoints::Circumcentric),
        &PairResolver,
        sm_mesh_api::reference::ReferenceBuilder::new(),
        &mut alloc,
    );
    assert!(matches!(r, Err(PolyError::Mixed(_))));
    let r = sm_poly::dualize(
        &src,
        &PolyOptions::new().with_agglomerate(Some(sm_poly::Agglomerate::new(0.5))),
        &PairResolver,
        sm_mesh_api::reference::ReferenceBuilder::new(),
        &mut alloc,
    );
    assert!(matches!(r, Err(PolyError::Mixed(_))));
    assert_eq!(alloc.state().collect::<Vec<_>>(), before);
    // an all-hex mesh is still `NotTet`
    let hex = meshes::single_hex().unwrap();
    assert!(matches!(dual_ref(&hex.mesh, &PolyOptions::new(), &PairResolver), Err(PolyError::NotTet { .. })));
    let _ = EntityKind::Mesh;
}

#[test]
fn acc4_smsh_export_of_a_mixed_result_fills_the_geometry_cache() {
    use sm_meshio::{SmshFile, SmshWriter};
    let (src, _) = wedge_tet_stack(2);
    let (m, _) = dual_prod(&src, &PolyOptions::new(), &PairResolver).unwrap();
    let bytes = SmshWriter::new(&m).to_bytes().expect("smsh");
    let file = SmshFile::parse(&bytes).unwrap();
    assert!(file.header().geometry_cache.is_some(), "a result with polyhedra carries gc_*");
    let d = file.decode().unwrap();
    let g = d.geometry_cache.as_ref().expect("cache decoded");
    let nc = m.count(MeshEntityKind::Cell);
    assert_eq!(g.cell_volume.len(), nc);
    for c in 0..nc {
        let (v, p) = sm_mesh_api::cell_volume_centroid(&m, CellIx::new(c));
        assert!(rel(g.cell_volume[c], v) <= 1e-12, "cell {c}");
        assert!((g.cell_centroid[c].x - p.x).abs() <= 1e-12 * (1.0 + p.x.abs()), "cell {c}");
    }
    assert_eq!(bytes, SmshWriter::new(&m).to_bytes().unwrap(), "the file is deterministic");
}
