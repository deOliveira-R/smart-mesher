//! W4-POLY-VORO: the circumcentric (Voronoi) dual, strict and opt-in.
#![allow(missing_docs)]

mod common;

use common::*;
use sm_mesh_api::{CellIx, CellShape, FaceIx, MeshEntityKind, MeshRead, SerialAlloc, TagValues, geometry_hash, topology_hash, validate_geometry};
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
    let f = meshes::tet_block([3, 2, 2], [1.0; 3]).unwrap();
    let (m, _) = dual_ref(&f.mesh, &circ(), &VertexClassResolver).unwrap();
    let nc = m.count(MeshEntityKind::Cell);
    for name in GENERATOR_TAGS {
        let t = m.tag(MeshEntityKind::Cell, name).expect("tag");
        let TagValues::F64(v) = &t.values else { panic!("f64 column") };
        assert_eq!(v.len(), nc);
    }
    // one generator per source vertex (a single volume), bit-identical to its coordinates
    let col = |k: usize| match &m.tag(MeshEntityKind::Cell, GENERATOR_TAGS[k]).unwrap().values {
        TagValues::F64(v) => v.clone(),
        _ => unreachable!(),
    };
    let (gx, gy, gz) = (col(0), col(1), col(2));
    let mut got: Vec<[u64; 3]> = (0..nc).map(|c| [gx[c].to_bits(), gy[c].to_bits(), gz[c].to_bits()]).collect();
    let co = f.mesh.coords();
    let mut want: Vec<[u64; 3]> = (0..co.x.len()).map(|v| [co.x[v].to_bits(), co.y[v].to_bits(), co.z[v].to_bits()]).collect();
    got.sort_unstable();
    want.sort_unstable();
    assert_eq!(got, want);
    // an interior cell is a unit cube centred on its generator
    for c in 0..nc {
        let (v, p) = sm_mesh_api::cell_volume_centroid(&m, CellIx::new(c));
        if (v - 1.0).abs() < 1e-12 {
            assert!((p.x - gx[c]).abs() < 1e-12 && (p.y - gy[c]).abs() < 1e-12 && (p.z - gz[c]).abs() < 1e-12);
        }
    }
    let _ = FaceMode::Keep;
}

#[test]
fn rejects_non_delaunay_input() {
    let f = meshes::tet_block_warped([3, 3, 3], [1.0; 3], 0.1).unwrap();
    let e = dual_ref(&f.mesh, &circ(), &VertexClassResolver).expect_err("refused");
    assert!(matches!(e, PolyError::NotOrthogonalDual { .. }), "{e}");
}

// ---- refusals, interfaces, determinism ----------------------------------------------------

use sm_mesh_api::reference::ReferenceMesh;
use sm_poly::{ToPoly, dualize};
use sm_testkit::meshes::CellSpec;

struct Pair;
impl sm_poly::EdgeResolver for Pair {
    fn resolve(&self, a: sm_core::EntityId, b: sm_core::EntityId, _e: [sm_mesh_api::Classification; 2]) -> Option<sm_core::EntityId> {
        sm_core::EntityId::new(sm_core::EntityKind::TopoEdge, a.serial() * 1000 + b.serial())
    }
}

fn tets(pts: &[[f64; 3]], cells: &[[usize; 4]]) -> ReferenceMesh {
    let c: Vec<CellSpec> = cells.iter().map(|t| CellSpec::Template(CellShape::Tet, t.to_vec())).collect();
    meshes::build_mesh(pts, &c, &mut SerialAlloc::new()).unwrap()
}

#[test]
fn acc5_hand_built_refusals_name_the_exact_counts_and_leave_state_alone() {
    // two tets on the face (0,1,2) with the far vertex of one strictly inside the other's sphere
    let pts = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0.2, 0.2, -1.0], [0.2, 0.2, 0.1]];
    let m = tets(&pts, &[[0, 1, 2, 3], [0, 2, 1, 4]]);
    let mut alloc = SerialAlloc::new();
    let before: Vec<_> = alloc.state().collect();
    let e = dualize(&m, &circ(), &Pair, sm_mesh::MeshBuilder::new(), &mut alloc).expect_err("refused");
    match &e {
        PolyError::NotOrthogonalDual { non_delaunay_faces: 1, first_at, .. } => {
            // the offending face is (0, 1, 2): its centroid
            assert!((first_at[0] - 1.0 / 3.0).abs() < 1e-12 && (first_at[1] - 1.0 / 3.0).abs() < 1e-12 && first_at[2] == 0.0, "{first_at:?}");
        }
        other => panic!("{other}"),
    }
    assert_eq!(before, alloc.state().collect::<Vec<_>>(), "alloc unchanged on refusal");
    // the same mesh in the barycentric dual is fine (strict gate is opt-in)
    // one obtuse boundary tet: exactly one outside circumcentre
    let one = tets(&[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0.2, 0.2, 0.1]], &[[0, 1, 2, 3]]);
    let e = dual_ref(&one, &circ(), &Pair).expect_err("refused");
    assert!(matches!(e, PolyError::NotOrthogonalDual { non_delaunay_faces: 0, outside_circumcentres: 1, .. }), "{e}");
    assert!(dual_ref(&one, &PolyOptions::new(), &Pair).is_ok());
    // a flat tet has no circumcentre
    let flat = tets(&[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.]], &[[0, 1, 2, 3]]);
    let e = dual_ref(&flat, &circ(), &Pair).expect_err("refused");
    assert!(matches!(e, PolyError::DegenerateTet { .. } | PolyError::NotOrthogonalDual { .. }), "{e}");
}

/// Faces whose two cells are in different regions.
fn interface_faces(m: &dyn MeshRead) -> usize {
    (0..m.count(MeshEntityKind::Face))
        .filter(|&f| {
            let f = FaceIx::new(f);
            m.face_right(f).is_some_and(|r| m.classification(MeshEntityKind::Cell, r.get() as u32) != m.classification(MeshEntityKind::Cell, m.face_left(f).get() as u32))
        })
        .count()
}

#[test]
fn acc6_multi_volume_interfaces_are_shared_like_the_barycentric_dual() {
    let f = meshes::tet_block_two_volumes([4, 2, 2], [1.0; 3], 2).unwrap();
    let (c, rep) = dual_ref(&f.mesh, &circ(), &VertexClassResolver).unwrap();
    let (b, _) = dual_ref(&f.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap();
    assert_eq!(interface_faces(&c), interface_faces(&b));
    assert_eq!(interface_faces(&c), 3 * 3, "one shared polygon per lattice vertex of the interface plane");
    assert!(rel(rep.total_volume, 16.0) < 1e-14);
    // per-region volumes: 8 + 8
    let mut by_region = std::collections::BTreeMap::new();
    for (c_, v) in volumes(&c).iter().enumerate() {
        *by_region.entry(c.classification(MeshEntityKind::Cell, c_ as u32).entity).or_insert(0.0) += v;
    }
    assert_eq!(by_region.len(), 2);
    assert!(by_region.values().all(|v| rel(*v, 8.0) < 1e-14), "{by_region:?}");
    assert!(validate_geometry(&c, &sm_mesh_api::GeomTol::default()).is_empty());
}

#[test]
fn acc6_lshape_passes_the_gate_and_is_refused_at_the_reflex_edge() {
    // The L-shape is a Kuhn lattice (Delaunay, circumcentres inside), but the Voronoi cells of the
    // vertices on its reflex edge are not star-shaped about their centroid. Circumcentric mode
    // has no wedge repair, so these are refused with the existing typed error (draft: Design,
    // "Wedge repair"); the barycentric dual of the same mesh is built.
    let l = meshes::tet_lshape(2, 1.0).unwrap();
    let e = dual_ref(&l.mesh, &circ(), &VertexClassResolver).expect_err("refused");
    match e {
        PolyError::UnrepairedCell { count, at, .. } => {
            assert_eq!(count, 3);
            // the failing cells sit on the reflex edge x = y = 2 (the first, near its end)
            assert!((at[0] - at[1]).abs() < 1e-12, "{at:?}");
        }
        other => panic!("{other}"),
    }
    assert!(dual_ref(&l.mesh, &PolyOptions::new(), &VertexClassResolver).is_ok());
}

#[test]
fn acc7_circumcentric_dual_is_bit_identical_across_thread_counts() {
    let f = meshes::tet_block([4, 3, 3], [1.0, 0.7, 1.3]).unwrap();
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| {
            let (m, rep) = dual_prod(&f.mesh, &circ().with_edges(Some(true)), &VertexClassResolver).unwrap();
            (topology_hash(&m), geometry_hash(&m), rep.welded_points, rep.zero_faces)
        })
    };
    let h = sm_testkit::determinism::check(2, 4, run).unwrap();
    let (m, rep) = dual_prod(&f.mesh, &circ().with_edges(Some(true)), &VertexClassResolver).unwrap();
    assert_eq!(h, (topology_hash(&m), geometry_hash(&m), rep.welded_points, rep.zero_faces));
}

#[test]
fn acc7_default_params_do_not_mention_dual() {
    let id = sm_core::EntityId::new(sm_core::EntityKind::Mesh, 1).unwrap();
    let default: ToPoly = serde_json::from_value(serde_json::json!({"mesh": id.to_string()})).unwrap();
    let v = serde_json::to_value(&default).unwrap();
    assert!(v.get("dual").is_none(), "{v}");
    assert_eq!(v, serde_json::json!({"mesh": id.to_string()}));
    let c: ToPoly = serde_json::from_value(serde_json::json!({"mesh": id.to_string(), "dual": "circumcentric"})).unwrap();
    assert_eq!(c.dual, DualPoints::Circumcentric);
    assert_eq!(serde_json::to_value(&c).unwrap()["dual"], "circumcentric");
    assert!(serde_json::from_value::<ToPoly>(serde_json::json!({"mesh": id.to_string(), "dual": "bogus"})).is_err());
    // the library options likewise
    let o = serde_json::to_value(PolyOptions::new()).unwrap();
    assert!(o.get("dual").is_none(), "{o}");
}

#[test]
fn acc1_all_polyhedra_face_count_and_acc4_keep_mode_is_planar_without_hubs() {
    let f = meshes::tet_block([3, 3, 3], [1.0; 3]).unwrap();
    // acc 4: Keep mode, no hub triangle, planar to rounding
    let (k, rep) = dual_ref(&f.mesh, &circ().with_faces(FaceMode::Keep), &VertexClassResolver).unwrap();
    assert_eq!((rep.hub_split_faces, rep.hub_triangles, rep.split_fans), (0, 0, 0));
    assert!(rep.max_nonplanarity < 1e-14, "{}", rep.max_nonplanarity);
    // Keep and Planar agree on a flat-faced lattice
    let (p, _) = dual_ref(&f.mesh, &circ(), &VertexClassResolver).unwrap();
    assert_eq!(k.count(MeshEntityKind::Face), p.count(MeshEntityKind::Face));
    // acc 1, last sentence: the hex-lattice face count (144 interior + 96 boundary polygons)
    // before `all_polyhedra`; the op default adds one subdivision face per cell
    assert_eq!(p.count(MeshEntityKind::Face), 144 + 96);
    let (q, rep) = dual_ref(&f.mesh, &circ().with_all_polyhedra(true), &VertexClassResolver).unwrap();
    assert_eq!(rep.cells, 64);
    // 240 faces plus one subdivision face per cell
    assert_eq!(q.count(MeshEntityKind::Face), 256);
    assert!((0..64).all(|c| q.cell_shape(CellIx::new(c)) == CellShape::Polyhedron));
}
