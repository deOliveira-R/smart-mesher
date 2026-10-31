//! A polyhedral mesh keeps every cell `Polyhedron`, and the repair of non-star-shaped cells
//! converges (regressions from the W3 integration milestone).
#![allow(missing_docs)]

mod common;

use common::*;
use sm_mesh_api::reference::ReferenceMesh;
use sm_mesh_api::{CellIx, CellShape, GeomTol, MeshRead, SerialAlloc, validate, validate_geometry};
use sm_poly::{FaceMode, PolyOptions, TopologyResolver};
use sm_testkit::meshes::{self, CellSpec};
use sm_testkit::lattice;

/// One tet split at its centroid: four tets around a vertex of degree 4, whose dual cell is a
/// tetrahedron.
fn tet_with_centre() -> ReferenceMesh {
    let pts = [[0., 0., 0.], [3., 0., 0.], [0., 3., 0.], [0., 0., 3.], [0.75, 0.75, 0.75]];
    let cells: Vec<CellSpec> = [[0, 1, 2, 4], [0, 3, 1, 4], [0, 2, 3, 4], [1, 3, 2, 4]]
        .iter()
        .map(|t| CellSpec::Template(CellShape::Tet, t.to_vec()))
        .collect();
    meshes::build_mesh(&pts, &cells, &mut SerialAlloc::new()).unwrap()
}

#[test]
fn tetrahedral_dual_cells_stay_polyhedra() {
    let src = tet_with_centre();
    let vol: f64 = (0..src.cell_count())
        .map(|c| sm_mesh_api::cell_volume(&src, CellIx::new(c)))
        .sum();
    for mode in [FaceMode::Planar, FaceMode::Keep] {
        let (m, rep) =
            dual_prod(&src, &PolyOptions::new().with_faces(mode).with_all_polyhedra(true), &PairResolver).unwrap();
        assert_eq!(m.cell_count(), 5);
        // the interior vertex has four neighbours: its dual is a tetrahedron before the repair
        for c in 0..m.cell_count() {
            let s = m.cell_shape(CellIx::new(c));
            assert!(
                !matches!(
                    s,
                    CellShape::Tet | CellShape::Hex | CellShape::Wedge | CellShape::Pyramid
                ),
                "{mode:?}: cell {c} is a {s:?}"
            );
        }
        assert!(validate(&m).is_empty(), "{:?}", validate(&m));
        let g = validate_geometry(&m, &GeomTol::new().with_planarity(f64::INFINITY));
        assert!(g.is_empty(), "{mode:?}: {:?}", &g[..g.len().min(3)]);
        assert!(rel(rep.total_volume, vol) < 1e-12, "{mode:?}");
    }
}

#[test]
fn repair_cascade_converges_on_perturbed_blocks() {
    // Refining a split cell bends the faces of its neighbours, so a failure can cascade along a
    // model edge. With an 8-round cap these two jittered blocks (valid sources) left 1 and 6 bad
    // cells; the cascade needs about 10 rounds.
    for (n, size, seed) in [([10, 10, 6], [10.0, 10.0, 6.0], 7u64), ([12, 12, 8], [12.0, 12.0, 8.0], 0)] {
        let lf = lattice::tet_kuhn(n, size).unwrap().perturbed(seed, 0.31).unwrap();
        assert!(validate_geometry(&lf.mesh, &GeomTol::new()).is_empty(), "{n:?}: the source must be valid");
        let r = TopologyResolver(&lf.topo);
        let (m, _) = dual_prod(&lf.mesh, &PolyOptions::new().with_all_polyhedra(true), &r).unwrap();
        assert!(validate(&m).is_empty(), "{n:?}: {:?}", validate(&m));
        let g = validate_geometry(&m, &GeomTol::new());
        assert!(g.is_empty(), "{n:?}: {} violations, e.g. {:?}", g.len(), &g[..g.len().min(2)]);
        for c in 0..m.cell_count() {
            let s = m.cell_shape(CellIx::new(c));
            assert!(s == CellShape::Polyhedron, "{n:?}: cell {c} is a {s:?}");
        }
    }
}

