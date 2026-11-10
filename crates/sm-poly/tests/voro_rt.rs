//! W4-POLY-VORO acceptance 8: `sm-rt` on the circumcentric dual. Every cell of the Kuhn dual is a
//! box and every interior BCC cell a truncated octahedron; both take the fast path
//! (`safety_distance > 0` at the centroid), and the track length of seeded rays equals that of the
//! source tet mesh (the same faceted boundary).
#![allow(missing_docs)]

mod common;

use common::*;
use sm_core::{Point3, Vector3};
use sm_geom_api::Ray;
use sm_mesh_api::{CellIx, MeshEntityKind, MeshRead, SerialAlloc, cell_volume_centroid};
use sm_poly::{DualPoints, PolyOptions, VertexClassResolver};
use sm_quality::{RT_PLANARITY_TOL, cell_convexity};
use sm_rt::MeshTrackerBuilder;
use sm_rt_api::{BoundarySpec, BuildInput, BuildOptions, ComplementPolicy, MeshInput, PartitionOptions, RegionRule, Scratch, TraceOptions, Tracker};
use sm_testkit::meshes;

fn tracker(mesh: &dyn MeshRead) -> sm_rt::MeshTracker {
    let mut p = PartitionOptions::default();
    p.enabled = true;
    let opts = BuildOptions::default().with_partition(p);
    let (bnd, comp) = (BoundarySpec::default(), ComplementPolicy::Outside);
    let input = BuildInput::Mesh(MeshInput::new(mesh, &RegionRule::ByVolumeClassification, &bnd, &comp));
    MeshTrackerBuilder.build_concrete(&input, &opts).expect("build").0
}

/// Total chord per seeded ray through `[0, side]^3` is the same on both trackers.
fn same_chords(src: &dyn MeshRead, dual: &sm_rt::MeshTracker, side: f64) {
    // seeded rays through the box; total chord per ray equals the tet mesh's
    let mut s = 0x2545_F491_4F6C_DD1Du64;
    let mut u = move || {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        (s >> 11) as f64 / (1u64 << 53) as f64
    };
    let rays: Vec<Ray> = (0..500)
        .map(|_| {
            let d = Vector3::new(u() - 0.5, u() - 0.5, u() - 0.5);
            Ray { origin: Point3::new(side * u(), side * u(), side * u()), dir: d }
        })
        .collect();
    let to = TraceOptions::default();
    let a = tracker(src).trace_bundle(&rays, &to).unwrap();
    let b = dual.trace_bundle(&rays, &to).unwrap();
    for i in 0..rays.len() {
        let total = |tb: &sm_rt_api::TrackBundle| tb.ray_range(i).map(|r| tb.length[r].iter().sum::<f64>()).unwrap_or(0.0);
        let (la, lb) = (total(&a), total(&b));
        assert!(la > 0.0 && rel(lb, la) < 1e-9, "ray {i}: {la} vs {lb}");
    }
}

#[test]
fn circumcentric_cells_take_the_fast_path_and_conserve_track_length() {
    let src = meshes::tet_block([3, 3, 3], [1.0; 3]).unwrap();
    let opts = PolyOptions::new().with_dual(DualPoints::Circumcentric);
    let (dual, _) = dual_ref(&src.mesh, &opts, &VertexClassResolver).unwrap();
    let t = tracker(&dual);
    let view = t.majorant_view().expect("partition");
    let mut sc = Scratch::new();
    let nc = dual.count(MeshEntityKind::Cell);
    for c in 0..nc {
        assert!(cell_convexity(&dual, CellIx::new(c), RT_PLANARITY_TOL).eligible, "cell {c}");
        let (_, g) = cell_volume_centroid(&dual, CellIx::new(c));
        let st = t.init_state(g, Vector3::new(1.0, 0.0, 0.0), None, &mut sc).unwrap();
        assert_eq!(st.cell.get(), c);
        assert!(view.safety_distance(&st, &mut sc).unwrap() > 0.0, "cell {c}: not on the fast path");
    }
    assert_eq!(t.info().convex_cells, nc);
    same_chords(&src.mesh, &t, 3.0);
}

/// The BCC patch is not a substitute for Kuhn: its ungated dual has boundary faces folded about the
/// box plane (the boundary-pyramid circumcentres sit h/4 outside), with zero area vector, and
/// `sm-rt` refuses to build on it. The interior truncated octahedra are checked in `voro_patch`.
#[test]
fn bcc_ungated_dual_is_refused_by_sm_rt_for_degenerate_boundary_faces() {
    let (pts, cells, _) = bcc_patch(4);
    let src = meshes::build_mesh(&pts, &cells, &mut SerialAlloc::new()).unwrap();
    let opts = PolyOptions::new().with_dual(DualPoints::Circumcentric);
    let (dual, _) = sm_poly::with_ungated_circumcentric(|| dual_ref(&src, &opts, &PairResolver)).unwrap();
    let mut p = PartitionOptions::default();
    p.enabled = true;
    let (bnd, comp) = (BoundarySpec::default(), ComplementPolicy::Outside);
    let input = BuildInput::Mesh(MeshInput::new(&dual, &RegionRule::ByVolumeClassification, &bnd, &comp));
    let err = MeshTrackerBuilder.build_concrete(&input, &BuildOptions::default().with_partition(p)).expect_err("refused");
    assert!(format!("{err:?}").contains("mesh.degenerate_face"), "{err:?}");
}
