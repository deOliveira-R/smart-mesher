//! Acceptance bullets 1-6, 7 (differential, determinism) and 10 (errors) of W3-POLY.
#![allow(missing_docs)]

mod common;

use common::*;
use sm_core::{EntityId, EntityKind};
use sm_mesh_api::{
    CellIx, CellShape, FaceIx, FaceKind, GeomTol, HashQuanta, MeshDim, MeshEdit, MeshEntityKind, MeshRead, SerialAlloc,
    ViolationCode, geometry_hash, geometry_hash_quantized, topology_hash, validate, validate_geometry,
};
use sm_poly::{
    EdgeResolver, FaceMode, PolyError, PolyOptions, TopologyResolver, VertexClassResolver, dualize, face_nonplanarity,
};
use sm_testkit::{lattice, meshes};

const TOL: f64 = 1e-12;

fn keep() -> PolyOptions {
    PolyOptions::new().with_faces(FaceMode::Keep)
}

fn no_planarity() -> GeomTol {
    GeomTol::new().with_planarity(f64::INFINITY)
}

fn vclass_dims(m: &dyn MeshRead) -> [usize; 4] {
    let mut c = [0usize; 4];
    for v in 0..m.vertex_count() {
        c[m.vertex_class(sm_mesh_api::VertexIx::new(v)).dim().unwrap() as usize] += 1;
    }
    c
}

// ---- 1 ------------------------------------------------------------------------------------

#[test]
fn acc1_single_tet() {
    let f = meshes::single_tet().unwrap();
    let (m, rep) = dual_ref(&f.mesh, &PolyOptions::new(), &PairResolver).unwrap();
    assert_eq!(rep.cells, 4);
    assert_eq!(m.cell_count(), 4);
    for v in volumes(&m) {
        assert!(rel(v, (1.0 / 6.0) / 4.0) < 1e-14, "volume {v}");
    }
    assert_eq!(rep.edge_faces, 6);
    assert_eq!(rep.boundary_faces, 12);
    assert_eq!(m.face_count(), 18);
    let boundary = (0..m.face_count()).filter(|&f| m.is_boundary_face(FaceIx::new(f))).count();
    assert_eq!(boundary, 12);
    assert!(rep.max_nonplanarity < 1e-13, "planarity {}", rep.max_nonplanarity);
    for f in 0..m.face_count() {
        assert!(face_nonplanarity(&m, FaceIx::new(f)) < 1e-13);
    }
    assert!(validate(&m).is_empty(), "{:?}", validate(&m));
    assert!(validate_geometry(&m, &GeomTol::default()).is_empty());
    // the 4 cells are hexahedra by topology
    assert!((0..4).all(|c| m.cell_shape(CellIx::new(c)) == CellShape::Hex));
}

// ---- 2 ------------------------------------------------------------------------------------

/// Independent recomputation of the planarity metric through the contract's face geometry.
fn recomputed_max_nonplanarity(m: &dyn MeshRead) -> f64 {
    let co = m.coords();
    (0..m.face_count())
        .map(|f| {
            let fi = FaceIx::new(f);
            let l = m.face_vertices(fi);
            if l.len() <= 3 {
                return 0.0;
            }
            let g = sm_mesh_api::face_geometry(m, fi);
            let len = g.area_vector.norm();
            if len == 0.0 {
                return 0.0;
            }
            let nhat = g.area_vector * (1.0 / len);
            let dev = l.iter().map(|v| (co.get(v.get()) - g.apex).dot(nhat).abs()).fold(0.0, f64::max);
            dev / len.sqrt()
        })
        .fold(0.0, f64::max)
}

#[test]
fn acc2_block333_keep_counts_and_volumes() {
    let f = meshes::tet_block([3, 3, 3], [1.0; 3]).unwrap();
    let (m, rep) = dual_ref(&f.mesh, &keep(), &VertexClassResolver).unwrap();
    assert_eq!(rep.cells, 64);
    assert_eq!(m.cell_count(), 64);
    assert_eq!(rep.edge_faces, 279);
    assert_eq!(rep.boundary_faces, 96);
    assert_eq!(m.face_count(), 375);
    assert_eq!(rep.points, 464);
    assert_eq!(m.vertex_count(), 464);
    // 162 TopoVolume + 234 TopoFace + 60 TopoEdge + 8 TopoVertex
    assert_eq!(vclass_dims(&m), [8, 60, 234, 162]);
    let vols = volumes(&m);
    assert!(rel(vols.iter().sum::<f64>(), 27.0) < TOL);
    assert!(rel(rep.total_volume, 27.0) < TOL);
    assert_eq!(vols.iter().filter(|v| rel(**v, 1.0) < TOL).count(), 8);
    assert_eq!(vols.iter().filter(|v| rel(**v, 0.5) < TOL).count(), 24);
    // boundary area 54, 9 on each of the six model faces
    let area: f64 = (0..m.face_count())
        .filter(|&f| m.is_boundary_face(FaceIx::new(f)))
        .map(|f| sm_mesh_api::face_area_vector(&m, FaceIx::new(f)).norm())
        .sum();
    assert!(rel(area, 54.0) < TOL);
    for k in 1..=6u64 {
        let a = boundary_area_on(&m, id(EntityKind::TopoFace, k));
        assert!(rel(a, 9.0) < TOL, "face {k}: {a}");
    }
    assert!(validate(&m).is_empty(), "{:?}", validate(&m));
    assert!(validate_geometry(&m, &no_planarity()).is_empty());
    let r = recomputed_max_nonplanarity(&m);
    assert!(r > 1e-3, "Keep must have non-planar faces, got {r}");
    assert!(rel(rep.max_nonplanarity, r) < 1e-9, "{} vs {r}", rep.max_nonplanarity);
}

// ---- 3 ------------------------------------------------------------------------------------

#[test]
fn acc3_block333_planar_matches_keep() {
    let f = meshes::tet_block([3, 3, 3], [1.0; 3]).unwrap();
    let (k, _) = dual_ref(&f.mesh, &keep(), &VertexClassResolver).unwrap();
    let (m, rep) = dual_ref(&f.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap();
    assert!(validate(&m).is_empty());
    assert!(validate_geometry(&m, &GeomTol::default()).is_empty());
    assert!(rel(volumes(&m).iter().sum::<f64>(), 27.0) < TOL);
    assert_eq!(volumes(&m).iter().filter(|v| rel(**v, 1.0) < TOL).count(), 8);
    assert_eq!(rep.cells, 64);
    assert_eq!(rep.boundary_faces, 96);
    assert!(m.face_count() >= 375);
    // Independent derivation from the Keep mesh: its non-planar edge faces and their hub triangles.
    let tol = GeomTol::default().planarity;
    let (mut split, mut tris) = (0usize, 0usize);
    for f in 0..k.face_count() {
        let fi = FaceIx::new(f);
        if k.is_boundary_face(fi) || face_nonplanarity(&k, fi) <= tol {
            continue;
        }
        let l = k.face_vertices(fi);
        let open = l.iter().any(|v| k.vertex_class(*v).dim().is_some_and(|d| d < 3));
        split += 1;
        tris += if open { l.len() - 2 } else { l.len() };
    }
    assert!(split > 0);
    assert_eq!(rep.hub_split_faces, split);
    assert_eq!(rep.hub_triangles, tris);
    assert_eq!(m.face_count(), 375 - rep.hub_split_faces + rep.hub_triangles);
    // every face is a triangle or planar
    assert!(rep.max_nonplanarity <= GeomTol::default().planarity);
}

// ---- 4 ------------------------------------------------------------------------------------

#[test]
fn acc4_two_volumes() {
    let f = meshes::tet_block_two_volumes([4, 2, 2], [1.0; 3], 2).unwrap();
    let (m, rep) = dual_ref(&f.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap();
    assert_eq!(rep.cells, 54);
    assert_eq!(m.cell_count(), 54);
    assert!(validate(&m).is_empty(), "{:?}", validate(&m));
    assert!(validate_geometry(&m, &GeomTol::default()).is_empty());
    // volume 8 per TopoVolume
    let regions: Vec<EntityId> = (1..=2).map(|k| id(EntityKind::TopoVolume, k)).collect();
    for r in &regions {
        let set = m.classified_on(*r).unwrap();
        let v: f64 = set.cells.iter().map(|c| sm_mesh_api::cell_volume(&m, *c)).sum();
        assert!(rel(v, 8.0) < TOL, "{r:?}: {v}");
        assert_eq!(set.cells.len(), 27);
    }
    // exactly 9 faces between different volumes, all classified on the interface TopoFace
    let src_iface = (0..f.mesh.face_count())
        .map(FaceIx::new)
        .find(|&fi| f.mesh.face_kind(fi) == FaceKind::Interior && {
            let (l, r) = f.mesh.face_cells(fi);
            f.mesh.cell_class(l).entity != f.mesh.cell_class(r.unwrap()).entity
        })
        .map(|fi| f.mesh.face_class(fi).entity)
        .unwrap();
    let mut n = 0;
    for fi in (0..m.face_count()).map(FaceIx::new) {
        let (l, r) = m.face_cells(fi);
        let Some(r) = r else { continue };
        let (cl, cr) = (m.cell_class(l).entity, m.cell_class(r).entity);
        if cl != cr {
            n += 1;
            assert_eq!(m.face_class(fi).entity, src_iface);
        } else {
            assert_eq!(m.face_class(fi).entity, cl, "same-volume interior faces stay on the volume");
        }
    }
    assert_eq!(n, 9);
}

fn interface_area(m: &dyn MeshRead, face: EntityId) -> f64 {
    m.classified_on(face)
        .map(|s| s.faces.iter().map(|f| sm_mesh_api::face_area_vector(m, *f).norm()).sum())
        .unwrap_or(0.0)
}

#[test]
fn acc4_interface_area_and_no_boundary_flag_on_interface() {
    let f = meshes::tet_block_two_volumes([4, 2, 2], [1.0; 3], 2).unwrap();
    let (m, _) = dual_ref(&f.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap();
    let iface = (0..m.face_count())
        .map(FaceIx::new)
        .find(|&fi| {
            let (l, r) = m.face_cells(fi);
            r.is_some_and(|r| m.cell_class(l).entity != m.cell_class(r).entity)
        })
        .map(|fi| m.face_class(fi).entity)
        .unwrap();
    // the interface plane is 2 x 2
    assert!(rel(interface_area(&m, iface), 4.0) < TOL, "{}", interface_area(&m, iface));
    assert!(m.classified_on(iface).unwrap().faces.iter().all(|f| !m.is_boundary_face(*f)));
}

// ---- 5 ------------------------------------------------------------------------------------

#[test]
fn acc5_warped_and_lshape() {
    let w = meshes::tet_block_warped([3, 3, 3], [1.0; 3], 0.1).unwrap();
    let l = meshes::tet_lshape(2, 1.0).unwrap();
    for (name, f) in [("warped", &w), ("lshape", &l)] {
        let (m, rep) = dual_ref(&f.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap();
        assert!(rel(volumes(&m).iter().sum::<f64>(), f.volume) < TOL, "{name} planar volume");
        assert!(rel(rep.total_volume, f.volume) < TOL);
        assert!(validate(&m).is_empty(), "{name}: {:?}", validate(&m));
        let g = validate_geometry(&m, &GeomTol::default());
        assert!(g.is_empty(), "{name} Planar: {:?}", &g[..g.len().min(5)]);
        let (k, krep) = dual_ref(&f.mesh, &keep(), &VertexClassResolver).unwrap();
        assert!(rel(krep.total_volume, f.volume) < TOL, "{name} keep volume");
        assert!(validate(&k).is_empty());
        let g = validate_geometry(&k, &no_planarity());
        assert!(g.is_empty(), "{name} Keep: {:?}", &g[..g.len().min(5)]);
        assert!(!g.iter().any(|v| v.code == ViolationCode::GeomSideTet));
    }
}

// ---- 6 ------------------------------------------------------------------------------------

fn area_by_class(m: &dyn MeshRead) -> std::collections::BTreeMap<EntityId, f64> {
    let mut out = std::collections::BTreeMap::new();
    for f in (0..m.face_count()).map(FaceIx::new).filter(|&f| m.is_boundary_face(f)) {
        *out.entry(m.face_class(f).entity).or_insert(0.0) += sm_mesh_api::face_area_vector(m, f).norm();
    }
    out
}

#[test]
fn acc6_face_areas_are_recovered() {
    let fixtures = [
        meshes::tet_block([3, 3, 3], [1.0; 3]).unwrap(),
        meshes::tet_block([2, 3, 4], [0.5, 1.0, 2.0]).unwrap(),
        meshes::tet_block_warped([3, 3, 3], [1.0; 3], 0.1).unwrap(),
        meshes::tet_lshape(2, 1.0).unwrap(),
        meshes::tet_block_two_volumes([4, 2, 2], [1.0; 3], 2).unwrap(),
    ];
    for f in &fixtures {
        let want = area_by_class(&f.mesh);
        let (m, _) = dual_ref(&f.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap();
        let got = area_by_class(&m);
        assert_eq!(want.keys().collect::<Vec<_>>(), got.keys().collect::<Vec<_>>(), "{}", f.name);
        for (k, a) in &want {
            assert!(rel(got[k], *a) < TOL, "{} {k:?}: {} vs {a}", f.name, got[k]);
        }
        // the classified_on index agrees
        for (k, a) in &want {
            assert!(rel(boundary_area_on(&m, *k), *a) < TOL);
        }
    }
}

#[test]
fn acc6_edge_table_is_classified() {
    // Counts and classes on the box: 12 curves x 6 dual segments.
    let f = meshes::tet_block([3, 3, 3], [1.0; 3]).unwrap();
    for mode in [FaceMode::Keep, FaceMode::Planar] {
        let (m, _) = dual_ref(&f.mesh, &PolyOptions::new().with_faces(mode).with_edges(Some(true)), &VertexClassResolver).unwrap();
        let t = m.edges().expect("edge table");
        assert!(validate(&m).is_empty(), "{mode:?}: {:?}", validate(&m));
        let mut on_curve = 0;
        for (i, [a, b]) in t.vertices.iter().enumerate() {
            let (ca, cb) = (m.vertex_class(*a), m.vertex_class(*b));
            let (da, db) = (ca.dim().unwrap(), cb.dim().unwrap());
            if da <= 1 && db <= 1 {
                on_curve += 1;
                let e = t.class[i];
                assert_eq!(e.dim(), Some(1));
                let want = if da == 1 { ca.entity } else { cb.entity };
                assert_eq!(e.entity, want);
                if da == 1 && db == 1 {
                    assert_eq!(ca.entity, cb.entity);
                }
            }
        }
        assert_eq!(on_curve, 72, "{mode:?}");
        // edge ids exist for all edges; every class is a model entity
        assert!(t.class.iter().all(|c| c.dim().is_some()));
    }
    // L-shape (reflex edge) with the vertex-class resolver
    let l = meshes::tet_lshape(2, 1.0).unwrap();
    let (m, _) = dual_ref(&l.mesh, &PolyOptions::new().with_edges(Some(true)), &VertexClassResolver).unwrap();
    assert!(m.edges().is_some());
    assert!(validate(&m).is_empty(), "{:?}", validate(&m));
    // the reflex curve: its dual segments are on one TopoEdge that has vertices on it
    let t = m.edges().unwrap();
    for (i, [a, b]) in t.vertices.iter().enumerate() {
        let (ca, cb) = (m.vertex_class(*a), m.vertex_class(*b));
        if ca.dim().unwrap() <= 1 && cb.dim().unwrap() <= 1 {
            assert_eq!(t.class[i].dim(), Some(1));
        }
    }
}

#[test]
fn acc6_source_edge_table_is_used_first_and_edges_follow_the_source() {
    let lf = lattice::tet_kuhn([3, 3, 3], [3.0; 3]).unwrap();
    let src = with_edge_table(&lf.mesh, &lf.topo);
    assert!(src.edges().is_some());
    // The table answers every feature question, so a resolver that refuses everything is fine,
    // and the output carries an edge table by default.
    let (m, _) = dual_ref(&src, &PolyOptions::new(), &NeverResolver).unwrap();
    assert!(m.edges().is_some(), "the source has an edge table, so the dual has one");
    assert!(validate(&m).is_empty(), "{:?}", validate(&m));
    // without the source table the same resolver fails, and the default is no table
    assert!(dual_ref(&lf.mesh, &PolyOptions::new(), &NeverResolver).is_err());
    let (m, _) = dual_ref(&lf.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap();
    assert!(m.edges().is_none());
    // explicit settings override the source
    let (m, _) = dual_ref(&src, &PolyOptions::new().with_edges(Some(false)), &NeverResolver).unwrap();
    assert!(m.edges().is_none());
}

#[test]
fn acc6_topology_resolver_on_lattice_fixture() {
    // `lattice::tet_kuhn` carries the CellComplex the mesh is classified on.
    let lf = lattice::tet_kuhn([3, 3, 3], [3.0, 3.0, 3.0]).unwrap();
    let r = TopologyResolver(&lf.topo);
    let (m, rep) = dual_ref(&lf.mesh, &PolyOptions::new().with_edges(Some(true)), &r).unwrap();
    assert_eq!(rep.cells, 64);
    assert!(validate(&m).is_empty());
    // the minimal fixture (2 intervals per curve) works with both resolvers and gives one mesh
    let lf = lattice::tet_kuhn([2, 2, 2], [2.0, 2.0, 2.0]).unwrap();
    let (a, _) = dual_ref(&lf.mesh, &PolyOptions::new(), &TopologyResolver(&lf.topo)).unwrap();
    let (b, _) = dual_ref(&lf.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap();
    assert_eq!(topology_hash(&a), topology_hash(&b));
}

#[test]
fn acc6_single_interval_curves_need_topology() {
    // n = [1,1,1] cannot be built by the fixture; use a block whose curves have one interval in
    // one direction via PairResolver-less VertexClassResolver: fails with AmbiguousEdge.
    let f = meshes::single_tet().unwrap();
    let e = dual_ref(&f.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap_err();
    assert!(matches!(e, PolyError::AmbiguousEdge { .. }), "{e:?}");
}

// ---- 7 ------------------------------------------------------------------------------------

#[test]
fn acc7_builders_agree() {
    let fixtures = [
        meshes::single_tet().unwrap(),
        meshes::tet_block([3, 3, 3], [1.0; 3]).unwrap(),
        meshes::tet_block_two_volumes([4, 2, 2], [1.0; 3], 2).unwrap(),
        meshes::tet_lshape(2, 1.0).unwrap(),
    ];
    let q = HashQuanta::new(1e-9, 1e-9);
    for f in &fixtures {
        for mode in [FaceMode::Planar, FaceMode::Keep] {
            for edges in [Some(false), Some(true)] {
                if f.name == "single_tet" && edges == Some(true) {
                    // fine: PairResolver is used for single_tet only
                }
                let opts = PolyOptions::new().with_faces(mode).with_edges(edges);
                let r: &dyn EdgeResolver = if f.name == "single_tet" { &PairResolver } else { &VertexClassResolver };
                let (a, ra) = dual_ref(&f.mesh, &opts, r).unwrap();
                let (b, rb) = dual_prod(&f.mesh, &opts, r).unwrap();
                assert_eq!(topology_hash(&a), topology_hash(&b), "{} {mode:?} {edges:?}", f.name);
                assert_eq!(geometry_hash_quantized(&a, &q), geometry_hash_quantized(&b, &q), "{}", f.name);
                assert_eq!(ra.cells, rb.cells);
                assert_eq!(ra.points, rb.points);
            }
        }
    }
}

fn pool_hash(src: &dyn MeshRead, threads: usize, mode: FaceMode, r: &dyn EdgeResolver) -> (u64, u64) {
    let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
    pool.install(|| {
        let (m, _) = dual_prod(src, &PolyOptions::new().with_faces(mode).with_edges(Some(true)), r).unwrap();
        (topology_hash(&m), geometry_hash(&m))
    })
}

#[test]
fn acc7_determinism() {
    let fixtures = [
        meshes::tet_block([4, 3, 3], [1.0, 0.7, 1.3]).unwrap(),
        meshes::tet_block_two_volumes([4, 2, 2], [1.0; 3], 2).unwrap(),
        meshes::tet_block_warped([3, 3, 3], [1.0; 3], 0.1).unwrap(),
        meshes::tet_lshape(2, 1.0).unwrap(),
    ];
    for f in &fixtures {
        for mode in [FaceMode::Planar, FaceMode::Keep] {
            let h = sm_testkit::determinism::check(2, 4, |threads| pool_hash(&f.mesh, threads, mode, &VertexClassResolver))
                .unwrap_or_else(|e| panic!("{} {mode:?}: {e}", f.name));
            // and the default global pool agrees with the 1-thread baseline
            let (m, _) = dual_prod(&f.mesh, &PolyOptions::new().with_faces(mode).with_edges(Some(true)), &VertexClassResolver).unwrap();
            assert_eq!(h, (topology_hash(&m), geometry_hash(&m)));
        }
    }
}

// ---- 10 -----------------------------------------------------------------------------------

#[test]
fn acc10_errors_are_typed() {
    let hex = meshes::single_hex().unwrap();
    assert!(matches!(dual_ref(&hex.mesh, &PolyOptions::new(), &PairResolver), Err(PolyError::NotTet { .. })));
    let wedge = meshes::single_wedge().unwrap();
    assert!(matches!(dual_ref(&wedge.mesh, &PolyOptions::new(), &PairResolver), Err(PolyError::NotTet { .. })));
    // a mesh that is already polyhedral
    let f = meshes::tet_block([3, 3, 3], [1.0; 3]).unwrap();
    let (poly, _) = dual_ref(&f.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap();
    assert!(matches!(dual_ref(&poly, &PolyOptions::new(), &VertexClassResolver), Err(PolyError::NotTet { .. })));
    // dimension 2
    let q = lattice::quad([2, 2], [1.0, 1.0]).unwrap();
    assert_eq!(q.mesh.dim(), MeshDim::Two);
    assert!(matches!(dual_ref(&q.mesh, &PolyOptions::new(), &PairResolver), Err(PolyError::Dim2)));
    // a non-conformal face (a boundary face of a fixture, re-marked through the edit API)
    let mut nc = meshes::tet_block([2, 2, 2], [1.0; 3]).unwrap().mesh;
    let bf = (0..nc.face_count()).map(FaceIx::new).find(|f| nc.is_boundary_face(*f)).unwrap();
    MeshEdit::set_face_kind(&mut nc, bf, FaceKind::NonConformal).unwrap();
    assert!(matches!(
        dual_ref(&nc, &PolyOptions::new(), &VertexClassResolver),
        Err(PolyError::UnsupportedFaceKind { face }) if face == bf
    ));
}

#[test]
fn alloc_is_unchanged_on_error() {
    let hex = meshes::single_hex().unwrap();
    let mut alloc = SerialAlloc::new();
    let before: Vec<_> = alloc.state().collect();
    let r = dualize(&hex.mesh, &PolyOptions::new(), &PairResolver, sm_mesh::MeshBuilder::new(), &mut alloc);
    assert!(r.is_err());
    assert_eq!(before, alloc.state().collect::<Vec<_>>());
    // an AmbiguousEdge failure comes late in the pipeline and must not touch the allocator either
    let t = meshes::single_tet().unwrap();
    let r = dualize(&t.mesh, &PolyOptions::new(), &VertexClassResolver, sm_mesh::MeshBuilder::new(), &mut alloc);
    assert!(matches!(r, Err(PolyError::AmbiguousEdge { .. })));
    assert_eq!(before, alloc.state().collect::<Vec<_>>());
    let _ = MeshEntityKind::Cell;
}
