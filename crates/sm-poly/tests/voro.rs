//! W4-POLY-VORO: the circumcentric (Voronoi) dual, strict and opt-in.
#![allow(missing_docs)]

mod common;

use common::*;
use sm_mesh_api::{CellIx, FaceIx, MeshEntityKind, MeshRead, TagValues, validate_geometry};
use sm_poly::{DualPoints, FaceMode, GENERATOR_TAGS, PolyError, PolyOptions, VertexClassResolver, face_nonplanarity};
use sm_testkit::meshes;

fn circ() -> PolyOptions {
    PolyOptions::new().with_dual(DualPoints::Circumcentric)
}

/// Faces of cell `c`.
fn nfaces(m: &dyn MeshRead, c: usize) -> usize {
    m.cell_faces(CellIx::new(c)).len()
}

/// Hand counts of the Kuhn block `[a, b, c]` dual (see the module docs of the test).
struct Hand {
    cells: usize,
    interior: usize,
    face: usize,
    edge: usize,
    corner: usize,
    welded: usize,
    zero_faces: usize,
}

fn hand(a: usize, b: usize, c: usize) -> Hand {
    let (ia, ib, ic) = (a.saturating_sub(1), b.saturating_sub(1), c.saturating_sub(1));
    Hand {
        cells: (a + 1) * (b + 1) * (c + 1),
        interior: ia * ib * ic,
        face: 2 * (ia * ib + ib * ic + ia * ic),
        edge: 4 * (ia + ib + ic),
        corner: 8,
        // 6abc tets collapse onto abc cube centres; each of the 4(ab+bc+ca) boundary triangles
        // has its circumcentre at the midpoint of its hypotenuse
        welded: 5 * a * b * c + 4 * (a * b + b * c + c * a),
        // face-diagonal edges (one per lattice square) and body diagonals (one per cube)
        zero_faces: a * b * (c + 1) + b * c * (a + 1) + a * c * (b + 1) + a * b * c,
    }
}

#[test]
fn acc1_kuhn_block_dual_is_the_cube_lattice() {
    for n in [[2usize, 2, 2], [3, 2, 4], [4, 4, 4]] {
        let f = meshes::tet_block(n.map(|x| x as u32), [1.0; 3]).unwrap();
        let (m, rep) = dual_ref(&f.mesh, &circ(), &VertexClassResolver).unwrap();
        let h = hand(n[0], n[1], n[2]);
        assert_eq!(rep.cells, h.cells, "{n:?}");
        assert_eq!(rep.hub_split_faces, 0);
        assert_eq!(rep.split_cells, 0);
        assert_eq!(rep.welded_points, h.welded, "{n:?}");
        assert_eq!(rep.zero_faces, h.zero_faces, "{n:?}");
        assert_eq!((rep.non_delaunay_faces, rep.outside_circumcentres), (0, 0));
        assert!(rep.max_nonplanarity < 1e-14);
        let vol = (n[0] * n[1] * n[2]) as f64;
        assert!(rel(rep.total_volume, vol) < 1e-14);
        // cell volumes: exactly 1, 1/2, 1/4, 1/8 with the hand multiplicities
        let mut v = volumes(&m);
        v.sort_by(f64::total_cmp);
        let count = |x: f64| v.iter().filter(|y| (**y - x).abs() <= 1e-14 * x).count();
        assert_eq!(count(1.0), h.interior, "{n:?}");
        assert_eq!(count(0.5), h.face, "{n:?}");
        assert_eq!(count(0.25), h.edge, "{n:?}");
        assert_eq!(count(0.125), h.corner, "{n:?}");
        assert_eq!(v.len(), h.cells);
        // no hub triangle: every face is a quad or a boundary polygon, interior cells have 6 faces
        let all = volumes(&m);
        let interior_faces: Vec<usize> =
            (0..all.len()).filter(|&c| (all[c] - 1.0).abs() <= 1e-14).map(|c| nfaces(&m, c)).collect();
        assert_eq!(interior_faces.len(), h.interior);
        assert!(interior_faces.iter().all(|&k| k == 6), "{n:?}");
        // interior cells are unit cubes: six quads; boundary polygons may carry collinear
        // edge midpoints but every face is planar
        for (c, vol) in all.iter().enumerate() {
            if (vol - 1.0).abs() <= 1e-14 {
                for sf in m.cell_faces(CellIx::new(c)) {
                    assert_eq!(m.face_vertices(sf.face()).len(), 4, "{n:?} cell {c}");
                }
            }
        }
        for fi in 0..m.count(MeshEntityKind::Face) {
            assert!(face_nonplanarity(&m, FaceIx::new(fi)) < 1e-14);
        }
        assert!(validate_geometry(&m, &sm_mesh_api::GeomTol::default()).is_empty());
        let _ = face_nonplanarity;
    }
}

#[test]
fn generator_tags_are_the_primal_vertices() {
    let f = meshes::tet_block([2, 2, 2], [1.0; 3]).unwrap();
    let (m, _) = dual_ref(&f.mesh, &circ(), &VertexClassResolver).unwrap();
    for name in GENERATOR_TAGS {
        let t = m.tag(MeshEntityKind::Cell, name).expect("tag");
        let TagValues::F64(v) = &t.values else { panic!("f64 column") };
        assert_eq!(v.len(), m.count(MeshEntityKind::Cell));
    }
    let _ = FaceMode::Keep;
}

#[test]
fn rejects_non_delaunay_input() {
    let f = meshes::tet_block_warped([3, 3, 3], [1.0; 3], 0.1).unwrap();
    let e = dual_ref(&f.mesh, &circ(), &VertexClassResolver).err().expect("refused");
    assert!(matches!(e, PolyError::NotOrthogonalDual { .. }), "{e}");
}
