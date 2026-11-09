//! W4-POLY-MEASURE (b), (c), (d): agreement of the convexity rule with `sm-rt`, dual eligibility
//! of tet meshes, determinism; plus the measurement table (ignored, run with `--ignored`).
#![allow(missing_docs)]

mod common;

use common::*;
use sm_core::{Point3, Vector3};
use sm_mesh_api::reference::{ReferenceMesh, hex_block};
use sm_mesh_api::{CellIx, CellShape, MeshEntityKind, MeshRead, SerialAlloc, cell_volume_centroid};
use sm_poly::{DualEligibility, FaceMode, PolyOptions, VertexClassResolver, dual_eligibility, poly_stats};
use sm_quality::{RT_PLANARITY_TOL, cell_convexity, convexity_report};
use sm_rt::MeshTrackerBuilder;
use sm_rt_api::{BoundarySpec, BuildInput, BuildOptions, ComplementPolicy, MeshInput, PartitionOptions, RegionRule, Scratch, Tracker};
use sm_testkit::determinism::check;
use sm_testkit::meshes::{self, CellSpec};

fn tracker(mesh: &dyn MeshRead) -> sm_rt::MeshTracker {
    let mut p = PartitionOptions::default();
    p.enabled = true;
    let opts = BuildOptions::default().with_partition(p);
    let (bnd, comp) = (BoundarySpec::default(), ComplementPolicy::Outside);
    let input = BuildInput::Mesh(MeshInput::new(mesh, &RegionRule::ByVolumeClassification, &bnd, &comp));
    MeshTrackerBuilder.build_concrete(&input, &opts).expect("build").0
}

/// (b): for a state at each cell's volume centroid, `safety_distance > 0` iff the cell is fast-path
/// eligible, and `TrackerInfo::convex_cells` equals the metric count. Returns `(cells, eligible)`.
fn agree_with_rt(name: &str, mesh: &dyn MeshRead) -> (usize, usize) {
    let t = tracker(mesh);
    let view = t.majorant_view().expect("partition");
    let mut sc = Scratch::new();
    let nc = mesh.count(MeshEntityKind::Cell);
    let mut eligible = 0;
    let mut convex_only = 0;
    for c in 0..nc {
        let x = cell_convexity(mesh, CellIx::new(c), RT_PLANARITY_TOL);
        // The centroid may lie outside a concave cell: fall back to the vertex mean of its corners.
        let (_, g) = cell_volume_centroid(mesh, CellIx::new(c));
        let mut st = t.init_state(g, Vector3::new(1.0, 0.0, 0.0), None, &mut sc).unwrap();
        if st.cell.get() != c {
            let co = mesh.coords();
            let mut vs: Vec<usize> = mesh.cell_faces(CellIx::new(c)).iter().flat_map(|sf| mesh.face_vertices(sf.face()).iter().map(|v| v.get())).collect();
            vs.sort_unstable();
            vs.dedup();
            let k = 1.0 / vs.len() as f64;
            let m = Point3::new(
                vs.iter().map(|&v| co.x[v]).sum::<f64>() * k,
                vs.iter().map(|&v| co.y[v]).sum::<f64>() * k,
                vs.iter().map(|&v| co.z[v]).sum::<f64>() * k,
            );
            st = t.init_state(m, Vector3::new(1.0, 0.0, 0.0), None, &mut sc).unwrap();
        }
        assert_eq!(st.cell.get(), c, "{name}: cell {c}: no interior point found");
        let s = view.safety_distance(&st, &mut sc).unwrap();
        assert_eq!(s > 0.0, x.eligible, "{name}: cell {c}: safety {s} vs eligible {} ({x:?})", x.eligible);
        eligible += usize::from(x.eligible);
        convex_only += usize::from(x.convex);
    }
    let r = convexity_report(mesh, RT_PLANARITY_TOL);
    assert_eq!((r.eligible, r.convex, r.cells), (eligible, convex_only, nc));
    assert_eq!(t.info().cells, nc);
    assert_eq!(t.info().convex_cells, r.eligible, "{name}: rt convex_cells vs metric");
    (nc, eligible)
}

#[test]
fn convexity_rule_agrees_with_sm_rt_cell_by_cell() {
    let mut total = (0, 0);
    let mut add = |name: &str, m: &dyn MeshRead| {
        let (n, e) = agree_with_rt(name, m);
        total = (total.0 + n, total.1 + e);
    };
    // all convex: a hex block
    let hb = hex_block([3, 3, 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    add("hex_block", &hb);
    let tb = meshes::tet_block([3, 3, 3], [1.0; 3]).unwrap();
    add("tet_block", &tb.mesh);
    for (name, src) in [
        ("tet_block", meshes::tet_block([3, 3, 3], [1.0; 3]).unwrap()),
        ("tet_block_warped", meshes::tet_block_warped([3, 3, 3], [1.0; 3], 0.1).unwrap()),
        ("two_volumes", meshes::tet_block_two_volumes([4, 2, 2], [1.0; 3], 2).unwrap()),
        ("lshape", meshes::tet_lshape(2, 1.0).unwrap()),
    ] {
        for mode in [FaceMode::Planar, FaceMode::Keep] {
            let (d, _) = dual_ref(&src.mesh, &PolyOptions::new().with_faces(mode), &VertexClassResolver).unwrap_or_else(|e| panic!("{name} {mode:?}: {e}"));
            add(&format!("{name}/{mode:?}"), &d);
        }
    }
    assert!(total.1 > 0 && total.1 < total.0, "both outcomes occur: {total:?}");
}

fn two_tets(d1: [f64; 3], d2: [f64; 3]) -> ReferenceMesh {
    two_tets_on([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]], d1, d2)
}

fn two_tets_on(base: [[f64; 3]; 3], d1: [f64; 3], d2: [f64; 3]) -> ReferenceMesh {
    let pts = [base[0], base[1], base[2], d1, d2];
    let cells = [CellSpec::Template(CellShape::Tet, vec![0, 1, 2, 3]), CellSpec::Template(CellShape::Tet, vec![0, 2, 1, 4])];
    meshes::build_mesh(&pts, &cells, &mut SerialAlloc::new()).unwrap()
}

#[test]
fn kuhn_blocks_are_fully_eligible() {
    for n in [[2, 2, 2], [3, 4, 2]] {
        let f = meshes::tet_block(n, [1.0, 0.5, 2.0]).unwrap();
        let e = dual_eligibility(&f.mesh).unwrap();
        assert_eq!((e.non_delaunay_faces, e.outside_circumcentres), (0, 0), "{n:?}: {e:?}");
        assert_eq!(e.tets, 6 * n.iter().product::<u32>() as usize);
        assert!(e.interior_faces > 0);
    }
    // two volumes: the interface faces take the region path
    let f = meshes::tet_block_two_volumes([4, 2, 2], [1.0; 3], 2).unwrap();
    let e = dual_eligibility(&f.mesh).unwrap();
    assert_eq!((e.non_delaunay_faces, e.outside_circumcentres), (0, 0), "{e:?}");
}

#[test]
fn one_flipped_face_is_exactly_one_non_delaunay_face() {
    // Face (0,1,2) in z = 0; its circumsphere with d1 = (.2,.2,-1) has centre (.5,.5,-.34) and
    // R^2 = .6156. d2 = (.2,.2,.1) is inside (.18 + .44^2 = .3736), d2' = (.2,.2,1) is outside.
    let bad = dual_eligibility(&two_tets([0.2, 0.2, -1.0], [0.2, 0.2, 0.1])).unwrap();
    assert_eq!((bad.tets, bad.interior_faces, bad.non_delaunay_faces), (2, 1, 1));
    let good = dual_eligibility(&two_tets([0.2, 0.2, -1.0], [0.2, 0.2, 1.0])).unwrap();
    assert_eq!((good.interior_faces, good.non_delaunay_faces), (1, 0));
    // a cospherical pair is accepted: six unit-cube corners lie on one sphere
    let cosph = two_tets_on([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]], [1., 1., 1.], [0., 0., 0.]);
    assert_eq!(dual_eligibility(&cosph).unwrap().non_delaunay_faces, 0);
}

#[test]
fn one_obtuse_boundary_tet_is_exactly_one_outside_circumcentre() {
    let one = |d: [f64; 3]| {
        let pts = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], d];
        meshes::build_mesh(&pts, &[CellSpec::Template(CellShape::Tet, vec![0, 1, 2, 3])], &mut SerialAlloc::new()).unwrap()
    };
    // flat tet (apex .1 above the middle of the base): the circumcentre is at z = -1.55
    let e = dual_eligibility(&one([0.2, 0.2, 0.1])).unwrap();
    assert_eq!((e.tets, e.outside_circumcentres), (1, 1));
    // the regular tetrahedron (centre at the origin) has its circumcentre inside
    let reg = [[1., 1., 1.], [1., -1., -1.], [-1., 1., -1.], [-1., -1., 1.]];
    let m = meshes::build_mesh(&reg, &[CellSpec::Template(CellShape::Tet, vec![0, 1, 2, 3])], &mut SerialAlloc::new()).unwrap();
    assert_eq!(dual_eligibility(&m).unwrap().outside_circumcentres, 0);
    // a circumcentre exactly on a boundary face counts as inside: all four corners are at
    // distance 5 from the origin, which lies in the base face z = 0
    let on = [[5., 0., 0.], [-5., 0., 0.], [3., 4., 0.], [0., 0., 5.]];
    let m = meshes::build_mesh(&on, &[CellSpec::Template(CellShape::Tet, vec![0, 1, 2, 3])], &mut SerialAlloc::new()).unwrap();
    assert_eq!(dual_eligibility(&m).unwrap().outside_circumcentres, 0);
}

#[test]
fn not_a_tet_mesh_has_no_eligibility() {
    let hb = hex_block([2, 2, 2], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    assert!(dual_eligibility(&hb).is_none());
    assert!(poly_stats(&hb).dual_eligibility.is_none());
}

#[test]
fn poly_stats_carry_the_new_fields() {
    let f = meshes::tet_block([3, 3, 3], [1.0; 3]).unwrap();
    let (planar, _) = dual_ref(&f.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap();
    let (keep, _) = dual_ref(&f.mesh, &PolyOptions::new().with_faces(FaceMode::Keep), &VertexClassResolver).unwrap();
    let (sp, sk) = (poly_stats(&planar), poly_stats(&keep));
    // Keep: one face per edge, nothing split, no multi-face pair; Planar splits 90 faces into hubs.
    assert_eq!((sk.multi_face_pairs, sk.hub_split_share), (0, 0.0));
    assert!(sp.multi_face_pairs > 0 && sp.hub_split_share > 0.0 && sp.hub_split_share < 1.0);
    assert_eq!(sp.fast_path_cells, sp.convex_cells - sp.convex_over_cap_cells);
    assert!(sp.convex_cells < sp.cells, "hub triangles make cells non-convex");
    assert!(sp.max_cell_convexity > 0.0);
    // the source is all-tet and (Kuhn) fully eligible
    let e: DualEligibility = poly_stats(&f.mesh).dual_eligibility.unwrap();
    assert_eq!((e.tets, e.non_delaunay_faces, e.outside_circumcentres), (162, 0, 0));
    assert!(serde_json::to_string(&sp).unwrap().contains("convex_cells"));
}

/// (d): the hash of every measure is the same for 1 and N threads and for repeated runs.
#[test]
fn measures_are_bit_identical_across_thread_counts() {
    let src = meshes::tet_block_warped([12, 12, 12], [1.0; 3], 0.15).unwrap();
    let (dual, _) = dual_ref(&src.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap();
    assert!(dual.cell_count() > 2048 && src.mesh.cell_count() > 2048, "more than one chunk");
    let fp = |t: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(t).build().unwrap();
        pool.install(|| {
            let h = sm_testkit::determinism::hash_f64_bits(&sm_quality::convexity_column(&dual, RT_PLANARITY_TOL));
            (h, serde_json::to_string(&poly_stats(&dual)).unwrap(), serde_json::to_string(&poly_stats(&src.mesh)).unwrap())
        })
    };
    check(2, 4, fp).unwrap();
    // the ambient pool (RAYON_NUM_THREADS) equals an explicit single thread
    let ambient = (
        sm_testkit::determinism::hash_f64_bits(&sm_quality::convexity_column(&dual, RT_PLANARITY_TOL)),
        serde_json::to_string(&poly_stats(&dual)).unwrap(),
        serde_json::to_string(&poly_stats(&src.mesh)).unwrap(),
    );
    assert_eq!(ambient, fp(1));
    // the warped block exercises both counts
    let e = dual_eligibility(&src.mesh).unwrap();
    assert!(e.interior_faces > 0);
}

/// The measurement table of the report: `cargo test -p sm-poly --release --test measure -- --ignored --nocapture`.
#[test]
#[ignore = "measurement table for the report; prints, asserts nothing"]
fn measurement_table() {
    let row = |name: &str, src: &dyn MeshRead| {
        let (dual, _) = dual_ref(src, &PolyOptions::new(), &VertexClassResolver).unwrap();
        let s = poly_stats(&dual);
        let e = dual_eligibility(src).unwrap();
        let n = s.cells as f64;
        println!(
            "| {name} | {} | {:.4} | {:.4} | {:.4} | {:.4} | {:.4} | {:.4} | {:.4} | {:.4} |",
            s.cells,
            s.convex_cells as f64 / n,
            s.over_cap_cells as f64 / n,
            s.convex_over_cap_cells as f64 / n,
            s.fast_path_cells as f64 / n,
            s.hub_split_share,
            s.faces_per_cell.mean,
            e.non_delaunay_share,
            e.outside_share
        );
    };
    row("tet_block 4^3", &meshes::tet_block([4, 4, 4], [1.0; 3]).unwrap().mesh);
    row("tet_block 40^3 perturbed 0.15", &meshes::tet_block_warped([40, 40, 40], [1.0; 3], 0.15).unwrap().mesh);
}
