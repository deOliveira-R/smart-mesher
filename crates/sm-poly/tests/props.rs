//! Acceptance bullet 9: properties over random tet blocks.
#![allow(missing_docs)]

mod common;

use common::*;
use proptest::prelude::*;
use sm_mesh_api::{GeomTol, MeshRead, validate, validate_geometry};
use sm_poly::{PolyOptions, VertexClassResolver};
use sm_testkit::meshes;

proptest! {
    #![proptest_config(sm_testkit::strategies::config(40))]

    #[test]
    fn prop_block_dual_is_valid_and_conserves_volume(
        n in prop::array::uniform3(2u32..=5),
        h in prop::array::uniform3(0.1f64..3.0),
    ) {
        let f = meshes::tet_block(n, h).unwrap();
        let (m, rep) = dual_prod(&f.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap();
        let verts = ((n[0] + 1) * (n[1] + 1) * (n[2] + 1)) as usize;
        prop_assert_eq!(m.cell_count(), verts);
        prop_assert_eq!(rep.cells, verts);
        let want = n[0] as f64 * n[1] as f64 * n[2] as f64 * h[0] * h[1] * h[2];
        let got: f64 = volumes(&m).iter().sum();
        prop_assert!(rel(got, want) < 1e-12, "{} vs {}", got, want);
        prop_assert!(validate(&m).is_empty(), "{:?}", validate(&m));
        let g = validate_geometry(&m, &GeomTol::default());
        prop_assert!(g.is_empty(), "{:?}", &g[..g.len().min(3)]);
    }
}
