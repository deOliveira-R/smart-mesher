//! INT-8: OpenFOAM export splits cells whose checkMesh determinant is below 0.001.

mod common;

use common::foam::{cell_stats, sets_block};
use sm_mesh_api::reference::{ReferenceMesh, hex_block};
use sm_mesh_api::*;
use sm_meshio::foam::*;
use sm_testkit::determinism;
use sm_testkit::meshes::{CellSpec, build_mesh, single_hex, single_tet, tet_block};

fn split_off() -> FoamOptions {
    FoamOptions::new().split_small_determinant(false)
}

/// Two tets glued on the face (0 1 2); each has one internal face and three boundary faces.
fn two_tets() -> ReferenceMesh {
    let p = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.], [0.3, 0.3, -1.0]];
    let cells = [CellSpec::Template(CellShape::Tet, vec![0, 1, 2, 3]), CellSpec::Template(CellShape::Tet, vec![0, 2, 1, 4])];
    let mut alloc = SerialAlloc::new();
    let mut m = build_mesh(&p, &cells, &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, "up", SetMembers::Cells(vec![CellIx::new(0)]), &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, "down", SetMembers::Cells(vec![CellIx::new(1)]), &mut alloc).unwrap();
    m
}

#[test]
fn determinant_matches_a_hand_computed_case() {
    // Cell 0 has four internal faces with area vectors (1/2) * (1,0,0), (0,1,0), (0,0,1), (1,1,1).
    // Magnitudes 1/2 (x3) and sqrt(3)/2: avg = (3 + sqrt 3) / 8. Tensor sum Sf Sf^T = (I + J)/4,
    // eigenvalues 1, 1/4, 1/4 -> det 1/16; OpenFOAM: |det| / avg^6 / 8.
    // Cells 1..4 have one internal face: rank 1, det 0.
    let points = vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
    let faces: [[u32; 3]; 4] = [[0, 2, 3], [0, 3, 1], [0, 1, 2], [1, 2, 3]];
    let mut pm = PolyMesh { points, n_cells: 5, ..Default::default() };
    pm.face_offsets = vec![0, 3, 6, 9, 12];
    pm.face_points = faces.concat();
    pm.owner = vec![0; 4];
    pm.neighbour = vec![1, 2, 3, 4];
    let d = cell_determinants(&pm);
    let avg: f64 = (3.0 + 3f64.sqrt()) / 8.0;
    let want = (1.0 / 16.0) / avg.powi(6) / 8.0;
    assert!((d[0] - want).abs() < 1e-12 * want, "{} vs {want}", d[0]);
    assert!(d[1..].iter().all(|x| *x < 1e-12));
    // Three orthogonal equal faces: identity tensor, det 1, / 8.
    pm.face_offsets = vec![0, 3, 6, 9];
    pm.face_points = faces[..3].concat();
    pm.owner = vec![0; 3];
    pm.neighbour = vec![1, 2, 3];
    pm.n_cells = 4;
    assert!((cell_determinants(&pm)[0] - 0.125).abs() < 1e-14);
    // An interior cube cell (six faces) is 1; a cell with no internal face is 0.
    let h = hex_block([3, 3, 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let hp = build_polymesh(&h, &split_off()).unwrap().mesh;
    let d = cell_determinants(&hp);
    assert!((d.iter().cloned().fold(f64::MAX, f64::min) - 0.125).abs() < 1e-14);
    assert!((d.iter().cloned().fold(0.0, f64::max) - 1.0).abs() < 1e-14);
    let lone = build_polymesh(&single_hex().unwrap().mesh, &split_off()).unwrap().mesh;
    assert_eq!(cell_determinants(&lone), [0.0]);
}

#[test]
fn a_tet_with_three_boundary_faces_is_split_and_every_exported_cell_passes() {
    let m = two_tets();
    let plain = build_polymesh(&m, &split_off()).unwrap();
    assert!(cell_determinants(&plain.mesh).iter().all(|d| *d < SMALL_DETERMINANT));
    let b = build_polymesh(&m, &FoamOptions::default()).unwrap();
    let pm = &b.mesh;
    assert_eq!(pm.n_cells, 8);
    assert_eq!(pm.points.len(), plain.mesh.points.len() + 2);
    assert_eq!(pm.n_internal(), 1 + 2 * 6);
    assert_eq!(pm.n_faces(), 6 + 1 + 12);
    for (c, d) in cell_determinants(pm).iter().enumerate() {
        assert!(*d >= SMALL_DETERMINANT, "exported cell {c}: determinant {d}");
    }
    assert!(check_polymesh(pm, 1e-9).is_empty(), "{:?}", check_polymesh(pm, 1e-9));
    // Geometry: closed cells, positive volumes, volumes add up to the parents'.
    let before = cell_stats(&plain.mesh);
    let after = cell_stats(pm);
    for (c, &(vol, closure, area)) in after.iter().enumerate() {
        assert!(vol > 0.0 && closure <= 1e-12 * area, "cell {c}: {vol} {closure}");
    }
    for parent in 0..2 {
        let sum: f64 = (0..8usize).filter(|&c| pm.cell_map[c] == parent).map(|c| after[c].0).sum();
        assert!((sum - before[parent as usize].0).abs() < 1e-14);
    }
    // Cell map, zones, patches.
    assert_eq!(pm.cell_map, [0, 0, 0, 0, 1, 1, 1, 1]);
    let zones: Vec<(&str, &[u32])> = pm.cell_zones.iter().map(|z| (z.name.as_str(), z.labels.as_slice())).collect();
    assert_eq!(zones, [("up", &[0, 1, 2, 3][..]), ("down", &[4, 5, 6, 7][..])]);
    assert_eq!(pm.patches.len(), 1);
    assert_eq!((pm.patches[0].start_face, pm.patches[0].n_faces), (13, 6));
    // Each boundary face keeps its points and now belongs to a sub-cell of its parent.
    for i in 0..6 {
        assert_eq!(pm.face(13 + i), plain.mesh.face(1 + i));
        assert_eq!(pm.cell_map[pm.owner[13 + i] as usize], plain.mesh.owner[1 + i]);
    }
    // Report.
    let item = b.report.loss.get("foam.small_det_split").unwrap();
    assert_eq!(item.count, 2);
    assert!(item.detail.contains("8 sub-cells") && item.detail.contains("smeshCellMap"));
    assert!(b.report.loss.get("foam.small_det_remaining").is_none());
    assert_eq!((b.report.n_cells, b.report.n_faces, b.report.n_internal_faces, b.report.n_points), (8, 19, 13, 7));
    assert_eq!(b.report.shape_counts.tetrahedra, 8);
    assert_eq!(b.report.patches[0].start_face, 13);
    assert_eq!(b.face_of_foam.len(), 19);
    assert_eq!(b.point_of_foam.len(), 7);
    assert_eq!(b.point_of_foam[5], u32::MAX);
}

#[test]
fn hex_meshes_without_small_determinants_export_unchanged() {
    for m in [hex_block([2, 2, 2], [0.5; 3], &mut SerialAlloc::new()).unwrap(), hex_block([3, 3, 3], [1.0; 3], &mut SerialAlloc::new()).unwrap(), sets_block()] {
        let a = build_polymesh(&m, &FoamOptions::default()).unwrap();
        let b = build_polymesh(&m, &split_off()).unwrap();
        assert_eq!(a.mesh.files().unwrap(), b.mesh.files().unwrap());
        assert_eq!(a.report, b.report);
        assert!(a.mesh.cell_map.is_empty());
        assert!(a.mesh.files().unwrap().iter().all(|f| f.name != CELL_MAP_FILE));
    }
    // A lone cell has no internal face (determinant 0): split into face pyramids, which pass.
    for (f, n) in [(single_hex().unwrap(), 6), (single_tet().unwrap(), 4)] {
        let b = build_polymesh(&f.mesh, &FoamOptions::default()).unwrap();
        assert_eq!(b.mesh.n_cells, n);
        assert!(cell_determinants(&b.mesh).iter().all(|d| *d >= SMALL_DETERMINANT));
        assert!(b.report.loss.get("foam.small_det_remaining").is_none());
    }
}

#[test]
fn a_hex_with_one_internal_face_is_split_into_face_pyramids() {
    let m = hex_block([2, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let b = build_polymesh(&m, &FoamOptions::default()).unwrap();
    let pm = &b.mesh;
    assert_eq!(pm.n_cells, 12);
    assert_eq!(pm.cell_map, [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1]);
    assert_eq!(b.report.shape_counts.pyramids, 12);
    assert_eq!(b.report.shape_counts.hexahedra, 0);
    assert!(check_polymesh(pm, 1e-9).is_empty());
    let vol: f64 = cell_stats(pm).iter().map(|c| c.0).sum();
    assert!((vol - 2.0).abs() < 1e-14);
    // Pyramids over a cube's faces from its centre are all alike; they pass.
    assert!(cell_determinants(pm).iter().all(|d| *d >= SMALL_DETERMINANT));
}

#[test]
fn cell_map_round_trips_through_files_and_directory() {
    let b = build_polymesh(&two_tets(), &FoamOptions::default()).unwrap();
    let files = b.mesh.files().unwrap();
    assert_eq!(files.last().unwrap().name, CELL_MAP_FILE);
    assert_eq!(parse_polymesh(&files).unwrap(), b.mesh);
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("foam_split_dir");
    let _ = std::fs::remove_dir_all(&dir);
    b.mesh.write_dir(&dir).unwrap();
    assert!(dir.join(CELL_MAP_FILE).is_file());
    assert_eq!(read_polymesh_dir(&dir).unwrap(), b.mesh);
    // A mesh without a split removes the stale map.
    build_polymesh(&sets_block(), &FoamOptions::default()).unwrap().mesh.write_dir(&dir).unwrap();
    assert!(!dir.join(CELL_MAP_FILE).exists());
    // A map of the wrong length is rejected.
    let mut bad = files.clone();
    let last = bad.len() - 1;
    bad[last].bytes = String::from_utf8(bad[last].bytes.clone()).unwrap().replacen("\n8\n(", "\n7\n(", 1).replacen("1\n)", ")", 1).into_bytes();
    assert!(parse_polymesh(&bad).is_err());
}

#[test]
fn tet_blocks_leave_no_small_determinant_and_export_deterministically() {
    for n in [[1, 1, 1], [2, 2, 2]] {
        let f = tet_block(n, [1.0; 3]).unwrap();
        let b = build_polymesh(&f.mesh, &FoamOptions::default()).unwrap();
        let before = build_polymesh(&f.mesh, &split_off()).unwrap();
        assert!(cell_determinants(&before.mesh).iter().any(|d| *d < SMALL_DETERMINANT), "{n:?}: the fixture exercises the split");
        assert!(check_polymesh(&b.mesh, 1e-9).is_empty());
        let vol: f64 = cell_stats(&b.mesh).iter().map(|c| c.0).sum();
        assert!((vol - f.volume).abs() < 1e-12 * f.volume, "{n:?}");
        // Cells that were not split keep their identity in the map.
        assert_eq!(b.mesh.cell_map.len(), b.mesh.n_cells as usize);
        assert!(b.mesh.cell_map.windows(2).all(|w| w[0] <= w[1]));
        let left = b.report.loss.get("foam.small_det_remaining").map_or(0, |i| i.count);
        assert_eq!(left, cell_determinants(&b.mesh).iter().filter(|d| **d < SMALL_DETERMINANT).count() as u64);
    }
    let f = tet_block([2, 2, 2], [1.0; 3]).unwrap();
    determinism::check(1, 4, |t| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(t).build().unwrap();
        pool.install(|| {
            let b = build_polymesh(&f.mesh, &FoamOptions::default()).unwrap();
            (b.mesh.files().unwrap(), b.face_of_foam, b.point_of_foam, b.report)
        })
    })
    .unwrap();
}

#[test]
fn classification_patches_split_too() {
    // Guard: the split never reads classification, so a classification-patch export splits too.
    let m = two_tets();
    let b = build_polymesh(&m, &FoamOptions::new().patches(PatchSource::Classification)).unwrap();
    assert_eq!(b.mesh.n_cells, 8);
    assert!(b.mesh.patches.iter().map(|p| p.n_faces).sum::<u32>() == 6);
}

#[test]
fn a_split_never_leaves_a_cell_worse_than_its_parent() {
    // Two flat slivers glued on (0 1 2); a split is applied only when it improves the worst cell.
    let p = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0.3, 0.3, 1e-4], [0.3, 0.3, -1e-4]];
    let cells = [CellSpec::Template(CellShape::Tet, vec![0, 1, 2, 3]), CellSpec::Template(CellShape::Tet, vec![0, 2, 1, 4])];
    let m = build_mesh(&p, &cells, &mut SerialAlloc::new()).unwrap();
    let plain = build_polymesh(&m, &split_off()).unwrap();
    assert!(cell_determinants(&plain.mesh).iter().all(|d| *d < SMALL_DETERMINANT));
    let b = build_polymesh(&m, &FoamOptions::default()).unwrap();
    // No exported cell is worse than its parent, whether or not it was split.
    let before = cell_determinants(&plain.mesh);
    for (c, d) in cell_determinants(&b.mesh).iter().enumerate() {
        let parent = if b.mesh.cell_map.is_empty() { c } else { b.mesh.cell_map[c] as usize };
        assert!(*d > before[parent], "cell {c}: {d} vs parent {}", before[parent]);
    }
}
