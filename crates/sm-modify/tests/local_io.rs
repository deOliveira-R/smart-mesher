//! MOD-REFINE-L acceptance 4 and 5: the `sm-rt` walker crosses the refined (polyhedral) mesh with
//! the chord sums of the box, and the OpenFOAM polyMesh and Exodus exports accept it.

use sm_core::{Point3, Vector3};
use sm_geom_api::Ray;
use sm_exodus::{ExodusOptions, exodus_bytes, read_exodus};
use sm_mesh::MeshBuilder;
use sm_mesh_api::reference::hex_block_with_edges;
use sm_mesh_api::{CellShape, MeshRead, SerialAlloc};
use sm_meshio::foam::{FoamOptions, build_polymesh, check_polymesh};
use sm_modify::{LocalOptions, refine_local};
use sm_rt::MeshTrackerBuilder;
use sm_rt_api::{BoundarySpec, BuildInput, BuildOptions, ComplementPolicy, MeshInput, RegionRule, TraceOptions, Tracker};

fn refined(n: u32, sel: &[usize], levels: u32) -> sm_mesh::Mesh {
    let src = hex_block_with_edges([n, n, n], [1.0; 3], &mut SerialAlloc::new()).expect("block");
    let mut mask = vec![false; (n * n * n) as usize];
    for &c in sel {
        mask[c] = true;
    }
    refine_local(&src, None, &LocalOptions { selected: &mask, levels, balance: true, project: false }, MeshBuilder::new(), &mut SerialAlloc::new()).expect("refine").finished.mesh
}

fn tracker(m: &dyn MeshRead) -> sm_rt::MeshTracker {
    let (bnd, comp) = (BoundarySpec::default(), ComplementPolicy::Outside);
    let input = BuildInput::Mesh(MeshInput::new(m, &RegionRule::ByVolumeClassification, &bnd, &comp));
    MeshTrackerBuilder.build_concrete(&input, &BuildOptions::default()).expect("build").0
}

/// Distance from `o` along unit `d` to the exit of `[0, side]^3` (slab method; `o` inside).
fn box_exit(o: [f64; 3], d: [f64; 3], side: f64) -> f64 {
    (0..3)
        .filter(|&i| d[i] != 0.0)
        .map(|i| if d[i] > 0.0 { (side - o[i]) / d[i] } else { -o[i] / d[i] })
        .fold(f64::INFINITY, f64::min)
}

#[test]
fn the_walker_crosses_the_refined_mesh_with_the_box_chords() {
    // 3x3x3 with the centre refined twice (neighbours raised by the balance): faces, edges, hanging nodes.
    let m = refined(3, &[13], 2);
    assert!(m.cell_count() > 64 + 6 * 8);
    let polys = (0..m.cell_count()).filter(|&c| m.cell_shape(sm_mesh_api::CellIx::new(c)) == CellShape::Polyhedron).count();
    assert!(polys > 0);
    let t = tracker(&m);
    let mut s = 0x2545_F491_4F6C_DD1Du64;
    let mut u = move || {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        (s >> 11) as f64 / (1u64 << 53) as f64
    };
    let mut rays = Vec::new();
    let mut want = Vec::new();
    while rays.len() < 10_000 {
        let d = Vector3::new(u() - 0.5, u() - 0.5, u() - 0.5);
        let n = d.norm();
        if n < 1e-3 {
            continue;
        }
        let dir = [d.x / n, d.y / n, d.z / n];
        let o = [3.0 * u(), 3.0 * u(), 3.0 * u()];
        want.push(box_exit(o, dir, 3.0));
        rays.push(Ray { origin: Point3::new(o[0], o[1], o[2]), dir: Vector3::new(dir[0], dir[1], dir[2]) });
    }
    let b = t.trace_bundle(&rays, &TraceOptions::default()).expect("trace");
    let mut worst = 0.0f64;
    for (i, w) in want.iter().enumerate() {
        let total: f64 = b.ray_range(i).map(|r| b.length[r].iter().sum::<f64>()).unwrap_or(0.0);
        worst = worst.max((total - w).abs());
    }
    assert!(worst < 1e-12, "worst chord-sum error {worst:e}");
}

#[test]
fn openfoam_and_exodus_take_the_polyhedral_result() {
    let m = refined(3, &[13], 1);
    let b = build_polymesh(&m, &FoamOptions::new().split_small_determinant(false)).expect("polyMesh");
    assert_eq!(b.mesh.n_cells as usize, m.cell_count());
    let issues = check_polymesh(&b.mesh, 1e-9);
    assert!(issues.is_empty(), "{issues:?}");
    let s = b.report.shape_counts;
    assert_eq!((s.hexahedra, s.polyhedra), (16, 18), "8 children + 8 corners; 6 face + 12 edge neighbours");
    let bytes = exodus_bytes(&m, &ExodusOptions::default()).expect("exodus");
    let back = read_exodus(&bytes).expect("read back");
    assert_eq!(back.num_elem, m.cell_count());
    assert_eq!(back.num_nodes, m.vertex_count());
    let kinds: Vec<(&str, usize)> = back.blocks.iter().map(|b| (b.elem_type.as_str(), b.num_elem)).collect();
    assert!(kinds.contains(&("NFACED", 18)) && kinds.contains(&("HEX8", 16)), "{kinds:?}");
}
