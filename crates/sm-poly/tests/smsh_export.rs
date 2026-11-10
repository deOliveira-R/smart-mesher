//! Ruling D2: a polyhedral `.smsh` export fills the geometry cache (`gc_*`), and a non-polyhedral
//! export does not change.
#![allow(missing_docs)]

mod common;

use common::*;
use sm_mesh_api::{CellIx, MeshEntityKind, MeshRead, cell_volume_centroid, compute_geometry_cache};
use sm_meshio::{SmshFile, SmshWriter};
use sm_poly::{DualPoints, PolyOptions, VertexClassResolver};
use sm_testkit::meshes;

fn bytes(m: &dyn MeshRead) -> Vec<u8> {
    SmshWriter::new(m).to_bytes().expect("smsh")
}

#[test]
fn polyhedral_export_carries_the_geometry_cache() {
    let src = meshes::tet_block([3, 3, 3], [1.0; 3]).unwrap();
    for dual in [DualPoints::Barycentric, DualPoints::Circumcentric] {
        let (poly, _) = dual_prod(&src.mesh, &PolyOptions::new().with_all_polyhedra(true).with_dual(dual), &VertexClassResolver).unwrap();
        assert!(poly.geometry_cache().is_none(), "the builder fills nothing: the writer does");
        let b = bytes(&poly);
        let file = SmshFile::parse(&b).unwrap();
        assert!(file.header().geometry_cache.is_some(), "{dual:?}");
        let d = file.decode().unwrap();
        let g = d.geometry_cache.as_ref().expect("cache decoded");
        let nc = poly.count(MeshEntityKind::Cell);
        assert_eq!(g.cell_volume.len(), nc);
        for c in 0..nc {
            let (v, p) = cell_volume_centroid(&poly, CellIx::new(c));
            assert!(rel(g.cell_volume[c], v) <= 1e-12, "{dual:?} cell {c}");
            for (a, b) in [(g.cell_centroid[c].x, p.x), (g.cell_centroid[c].y, p.y), (g.cell_centroid[c].z, p.z)] {
                assert!((a - b).abs() <= 1e-12 * (1.0 + b.abs()), "{dual:?} cell {c} centroid");
            }
        }
        // identical to the library cache, and the file is deterministic
        let lib = compute_geometry_cache(&poly);
        assert_eq!(g.cell_volume, lib.cell_volume);
        assert_eq!(b, bytes(&poly));
    }
}

#[test]
fn non_polyhedral_export_is_unchanged() {
    // a tet mesh and a hex mesh written without a cache stay without one
    let tet = meshes::tet_block([2, 2, 2], [1.0; 3]).unwrap();
    assert!(SmshFile::parse(&bytes(&tet.mesh)).unwrap().header().geometry_cache.is_none());
    let hex = sm_mesh_api::reference::hex_block([2, 2, 2], [1.0; 3], &mut sm_mesh_api::SerialAlloc::new()).unwrap();
    assert!(SmshFile::parse(&bytes(&hex)).unwrap().header().geometry_cache.is_none());
}
