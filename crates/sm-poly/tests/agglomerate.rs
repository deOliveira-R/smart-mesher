//! Stretch S2: agglomeration.
#![allow(missing_docs)]

mod common;

use common::*;
use sm_core::EntityKind;
use sm_mesh_api::{FaceIx, GeomTol, MeshRead, geometry_hash, topology_hash, validate, validate_geometry};
use sm_poly::{Agglomerate, FaceMode, PolyOptions, VertexClassResolver, face_nonplanarity};
use sm_testkit::meshes;

/// Planar-mode dual cells already have concavity up to 0.235 on the Kuhn block (median 0.14,
/// hub triangles bulge), so the default limit 0.1 merges nothing there; 1.0 admits the 20 merges
/// of the corner cells (measured: nothing merges up to 0.5, 20 merges from 0.75 on).
fn opts(ratio: f64) -> PolyOptions {
    PolyOptions::new().with_agglomerate(Some(Agglomerate::new(ratio).with_max_concavity(1.0)))
}

fn volume_faces(m: &dyn MeshRead) -> usize {
    (0..m.face_count())
        .map(FaceIx::new)
        .filter(|&f| m.face_right(f).is_some_and(|r| m.cell_class(m.face_left(f)).entity != m.cell_class(r).entity))
        .count()
}

#[test]
fn s2_block444_merges_small_cells() {
    let f = meshes::tet_block([4, 4, 4], [1.0; 3]).unwrap();
    let (base, brep) = dual_ref(&f.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap();
    assert_eq!(brep.merges, 0);
    let (m, rep) = dual_ref(&f.mesh, &opts(0.75), &VertexClassResolver).unwrap();
    assert!(rep.merges > 0, "ratio 0.75 must merge the corner and edge cells");
    assert_eq!(m.cell_count(), base.cell_count() - rep.merges);
    assert_eq!(rep.cells, m.cell_count());
    assert!(rel(volumes(&m).iter().sum::<f64>(), 64.0) < 1e-12);
    assert!(validate(&m).is_empty(), "{:?}", validate(&m));
    let g = validate_geometry(&m, &GeomTol::default());
    assert!(g.is_empty(), "{:?}", &g[..g.len().min(3)]);
    // merging removes faces only, so every face is planar and the faces are a subset
    assert!(m.face_count() < base.face_count());
    for fi in 0..m.face_count() {
        assert!(face_nonplanarity(&m, FaceIx::new(fi)) <= GeomTol::default().planarity);
    }
    // merging never creates a smaller cell
    assert!(rep.min_cell_volume >= brep.min_cell_volume * (1.0 - 1e-12));
    // boundary tiling is untouched
    let area = |m: &dyn MeshRead| (0..m.face_count()).map(FaceIx::new).filter(|f| m.is_boundary_face(*f)).count();
    assert_eq!(area(&m), area(&base));
}

#[test]
fn s2_default_concavity_guard_rejects_the_hub_bulges() {
    // Measured: the unmerged Planar cells have concavity 0.14 (median) to 0.235 (max), above the
    // default 0.1, so no pair passes; the output is the unagglomerated dual.
    let f = meshes::tet_block([4, 4, 4], [1.0; 3]).unwrap();
    let (base, _) = dual_prod(&f.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap();
    let o = PolyOptions::new().with_agglomerate(Some(Agglomerate::new(0.75)));
    let (m, rep) = dual_prod(&f.mesh, &o, &VertexClassResolver).unwrap();
    assert_eq!(rep.merges, 0);
    assert_eq!(topology_hash(&m), topology_hash(&base));
    // a looser guard merges, a tighter one cannot merge more
    let loose = PolyOptions::new().with_agglomerate(Some(Agglomerate::new(0.75).with_max_concavity(1.0)));
    let (_, r1) = dual_prod(&f.mesh, &loose, &VertexClassResolver).unwrap();
    assert_eq!(r1.merges, 20);
}

#[test]
fn s2_rounds_zero_is_the_identity() {
    let f = meshes::tet_block([4, 4, 4], [1.0; 3]).unwrap();
    let (base, _) = dual_prod(&f.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap();
    let o = PolyOptions::new().with_agglomerate(Some(Agglomerate::new(0.75).with_max_concavity(1.0).with_max_rounds(0)));
    let (m, rep) = dual_prod(&f.mesh, &o, &VertexClassResolver).unwrap();
    assert_eq!(rep.merges, 0);
    assert_eq!((topology_hash(&m), geometry_hash(&m)), (topology_hash(&base), geometry_hash(&base)));
    // ratio 0 merges nothing either
    let (m, rep) = dual_prod(&f.mesh, &opts(0.0), &VertexClassResolver).unwrap();
    assert_eq!(rep.merges, 0);
    assert_eq!(topology_hash(&m), topology_hash(&base));
}

#[test]
fn s2_never_merges_across_volumes_and_keeps_the_interface() {
    let f = meshes::tet_block_two_volumes([4, 2, 2], [1.0; 3], 2).unwrap();
    let (base, _) = dual_ref(&f.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap();
    for ratio in [0.5, 0.75, 1.2] {
        let (m, rep) = dual_ref(&f.mesh, &opts(ratio), &VertexClassResolver).unwrap();
        assert!(validate(&m).is_empty(), "{ratio}: {:?}", validate(&m));
        assert!(validate_geometry(&m, &GeomTol::default()).is_empty(), "{ratio}");
        assert_eq!(volume_faces(&m), 9, "{ratio}: interface faces survive");
        assert_eq!(m.cell_count(), base.cell_count() - rep.merges);
        for k in 1..=2 {
            let r = id(EntityKind::TopoVolume, k);
            let v: f64 = m.classified_on(r).unwrap().cells.iter().map(|c| sm_mesh_api::cell_volume(&m, *c)).sum();
            assert!(rel(v, 8.0) < 1e-12, "{ratio}: volume {k}: {v}");
        }
    }
}

#[test]
fn s2_is_deterministic_and_builders_agree() {
    let f = meshes::tet_block([4, 4, 4], [1.0, 0.8, 1.2]).unwrap();
    let o = opts(0.75).with_edges(Some(true));
    let h = sm_testkit::determinism::check(2, 4, |threads| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| {
            let (m, _) = dual_prod(&f.mesh, &o, &VertexClassResolver).unwrap();
            (topology_hash(&m), geometry_hash(&m))
        })
    })
    .unwrap();
    let (a, _) = dual_ref(&f.mesh, &o, &VertexClassResolver).unwrap();
    assert_eq!(h.0, topology_hash(&a));
    assert!(validate(&a).is_empty(), "{:?}", validate(&a));
}

#[test]
fn s2_keep_mode_and_warped_inputs_stay_valid() {
    let w = meshes::tet_block_warped([4, 4, 4], [1.0; 3], 0.1).unwrap();
    for mode in [FaceMode::Planar, FaceMode::Keep] {
        let (m, rep) = dual_ref(&w.mesh, &opts(0.75).with_faces(mode), &VertexClassResolver).unwrap();
        assert!(validate(&m).is_empty(), "{mode:?}: {:?}", validate(&m));
        assert!(rel(rep.total_volume, w.volume) < 1e-12);
        if mode == FaceMode::Planar {
            let g = validate_geometry(&m, &GeomTol::default());
            assert!(g.is_empty(), "{:?}", &g[..g.len().min(3)]);
        }
    }
}
