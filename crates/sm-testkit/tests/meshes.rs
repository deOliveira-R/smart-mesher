//! Mesh fixtures: volumes, counts, shapes, validity.

use sm_mesh_api::{CellIx, CellShape, MeshRead, cell_volume};
use sm_testkit::check::*;
use sm_testkit::meshes::*;

#[test]
fn every_fixture_has_the_documented_volumes_and_is_valid() {
    for f in all_mesh_fixtures().unwrap() {
        let m = &f.mesh;
        assert_eq!(m.cell_count(), f.cell_volumes.len(), "{}", f.name);
        let mut total = 0.0;
        for (i, v) in f.cell_volumes.iter().enumerate() {
            let got = cell_volume(m, CellIx::new(i));
            assert!((got - v).abs() < 1e-12 * v.max(1.0), "{} cell {i}: {got} vs {v}", f.name);
            total += got;
        }
        assert!((total - f.volume).abs() < 1e-12 * f.volume, "{}", f.name);
        mesh_valid(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
        orientation_rule(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
        closed_cells(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
        positive_volumes(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
        watertight(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
        classification_complete(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
        check_mesh_euler(m, 1).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
        let nb = (0..m.face_count()).filter(|&f| m.face_right(sm_mesh_api::FaceIx::new(f)).is_none()).count();
        assert_eq!(nb, f.boundary_faces, "{}", f.name);
        // The boundary surface has the documented exact area.
        let area: f64 = (0..m.face_count())
            .map(sm_mesh_api::FaceIx::new)
            .filter(|f| m.face_right(*f).is_none())
            .map(|f| sm_mesh_api::face_area_vector(m, f).norm())
            .sum();
        assert!((area - f.area).abs() < 1e-12 * f.area, "{}: area {area} vs {}", f.name, f.area);
    }
}

#[test]
fn single_cells_have_their_shapes_and_counts() {
    for (f, shape, nv, nf, vol) in [
        (single_hex().unwrap(), CellShape::Hex, 8, 6, 1.0),
        (single_tet().unwrap(), CellShape::Tet, 4, 4, 1.0 / 6.0),
        (single_wedge().unwrap(), CellShape::Wedge, 6, 5, 0.5),
        (single_pyramid().unwrap(), CellShape::Pyramid, 5, 5, 1.0 / 3.0),
    ] {
        assert_eq!(f.mesh.cell_shape(CellIx::new(0)), shape, "{}", f.name);
        assert_eq!((f.mesh.vertex_count(), f.mesh.face_count(), f.mesh.cell_count()), (nv, nf, 1), "{}", f.name);
        assert_eq!(f.volume, vol);
    }
}

#[test]
fn polyhedral_cells() {
    let to = truncated_octahedron().unwrap();
    let m = &to.mesh;
    assert_eq!(m.cell_shape(CellIx::new(0)), CellShape::Polyhedron);
    assert_eq!((m.vertex_count(), m.face_count()), (24, 14));
    // 6 squares and 8 hexagons; Euler V - E + F = 2 gives E = 36.
    let sizes: Vec<usize> = (0..14).map(|f| m.face_vertices(sm_mesh_api::FaceIx::new(f)).len()).collect();
    assert_eq!(sizes.iter().filter(|&&s| s == 4).count(), 6);
    assert_eq!(sizes.iter().filter(|&&s| s == 6).count(), 8);
    assert_eq!(mesh_euler(m), 1);
    let hp = hexagonal_prism(2.0, 3.0).unwrap();
    assert_eq!(hp.mesh.cell_shape(CellIx::new(0)), CellShape::Polyhedron);
    assert_eq!(hp.mesh.face_count(), 8);
    assert!((hp.volume - 1.5 * 3.0_f64.sqrt() * 4.0 * 3.0).abs() < 1e-12);
    let sc = split_face_cube().unwrap();
    assert_eq!(sc.mesh.cell_shape(CellIx::new(0)), CellShape::Polyhedron);
    assert_eq!(sc.mesh.face_count(), 7);
}

#[test]
fn hex_block_and_transition_counts() {
    let hb = hex_block_2x2x2().unwrap();
    assert_eq!((hb.mesh.vertex_count(), hb.mesh.face_count(), hb.mesh.cell_count()), (27, 36, 8));
    let t = mixed_transition().unwrap();
    let shapes: Vec<CellShape> = (0..4).map(|c| t.mesh.cell_shape(CellIx::new(c))).collect();
    assert_eq!(shapes, [CellShape::Hex, CellShape::Wedge, CellShape::Pyramid, CellShape::Tet]);
    // Three interior (shared) faces: hex|wedge, hex|pyramid, pyramid|tet.
    let interior = t.mesh.face_count() - t.boundary_faces;
    assert_eq!(interior, 3);
    assert!((t.volume - (1.0 + 0.5 + 1.0 / 3.0 + 1.25 / 6.0)).abs() < 1e-14);
}

#[test]
fn dual_patch_shares_hexagon_faces() {
    let d = dual_patch().unwrap();
    assert_eq!(d.mesh.cell_count(), 7);
    // 7 hexagons: 24 distinct points per level; 6 shared side quads between centre and ring plus 6
    // between ring neighbours = 12 interior faces.
    assert_eq!(d.mesh.vertex_count(), 48);
    let interior = d.mesh.face_count() - d.boundary_faces;
    assert_eq!(interior, 12);
    let hex_area = 1.5 * 3.0_f64.sqrt();
    assert!((d.volume - 7.0 * hex_area).abs() < 1e-12);
}

#[test]
fn fixtures_are_reproducible() {
    for (a, b) in all_mesh_fixtures().unwrap().into_iter().zip(all_mesh_fixtures().unwrap()) {
        assert_eq!(mesh_hash(&a.mesh), mesh_hash(&b.mesh), "{}", a.name);
    }
}
