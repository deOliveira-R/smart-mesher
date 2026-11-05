//! W3b-SMOOTH: `NodeSelect::Set` and the `set` argument of `mesh.smooth`.

mod common;

use common::*;
use sm_mesh_api::{CellIx, MeshEdit, MeshRead, SerialAlloc, SetMembers};
use sm_quality::{Metric, Options, compute};
use sm_smooth::{Method, NodeSelect, Objective, Schedule, SmoothError};
use sm_testkit::lattice::{self, LatticeFixture};

/// A perturbed 6^3 hex block with a cell set `hot` (the cells with all lattice indices < 3)
/// and an empty vertex set `verts`.
fn fixture() -> (LatticeFixture, Vec<CellIx>) {
    let mut f = lattice::hex([6, 6, 6], [1.0; 3]).expect("hex").perturbed(5, 0.25).expect("perturb");
    // Lattice cells are numbered x-fastest.
    let cells: Vec<CellIx> = (0..216).filter(|c| c % 6 < 3 && (c / 6) % 6 < 3 && c / 36 < 3).map(CellIx::new).collect();
    let mut alloc = SerialAlloc::new();
    f.mesh.add_set("hot", SetMembers::Cells(cells.clone()), &mut alloc).expect("add set");
    f.mesh.add_set("verts", SetMembers::Vertices(vec![]), &mut alloc).expect("add vertex set");
    (f, cells)
}

fn set_req(method: Method, name: &str, fix_boundary: bool) -> sm_smooth::SmoothRequest {
    let mut r = req(method, Schedule::Colored, 20, fix_boundary);
    r.nodes = NodeSelect::Set(name.into());
    r
}

fn bits(p: &[[f64; 3]]) -> Vec<[u64; 3]> {
    p.iter().map(|q| q.map(f64::to_bits)).collect()
}

fn corners_of(m: &dyn MeshRead, cells: &[CellIx]) -> Vec<bool> {
    let mut v = vec![false; m.vertex_count()];
    for c in cells {
        for x in m.cell_corners(*c).expect("corners") {
            v[x.get()] = true;
        }
    }
    v
}

#[test]
fn nodes_outside_the_set_are_bit_identical_and_inside_improves() {
    let (f, cells) = fixture();
    let in_set = corners_of(&f.mesh, &cells);
    let start = pts_of(&f.mesh);
    for method in [
        Method::Laplacian { relax: 1.0 },
        Method::SmartLaplacian { relax: 1.0 },
        Method::Optimize { objective: Objective::MeanRatio, p: 4.0 },
    ] {
        for fix_boundary in [true, false] {
            let o = run(&f, &set_req(method, "hot", fix_boundary));
            let (b, a) = (bits(&start), bits(&o.points));
            let mut moved = 0;
            for i in 0..b.len() {
                if !in_set[i] {
                    assert_eq!(b[i], a[i], "node {i} outside the set moved ({method:?})");
                } else {
                    moved += usize::from(b[i] != a[i]);
                }
            }
            assert!(moved > 0, "{method:?}: the set's nodes moved");
            let after = applied(&f, &o);
            let opts = Options::default();
            let q0 = compute(&f.mesh, Metric::Shape, &opts).expect("q0");
            let q1 = compute(&after, Metric::Shape, &opts).expect("q1");
            let min = |q: &[f64]| cells.iter().map(|c| q[c.get()]).fold(f64::INFINITY, f64::min);
            assert!(min(&q1) >= min(&q0) - 1e-12, "{method:?}: min Shape {} -> {}", min(&q0), min(&q1));
        }
    }
}

#[test]
fn unknown_set_lists_the_cell_set_names() {
    let (f, _) = fixture();
    for name in ["nope", "verts"] {
        let e = sm_smooth::smooth(&input(&f), &set_req(Method::default(), name, true)).expect_err("unknown set");
        match &e {
            SmoothError::UnknownSet { name: n, available } => {
                assert_eq!(n, name);
                assert_eq!(available, &["hot".to_owned()], "only cell sets are listed");
            }
            other => panic!("{other:?}"),
        }
        assert!(e.to_string().contains("hot"), "{e}");
    }
}

#[test]
fn set_selection_is_deterministic_across_threads() {
    let (f, _) = fixture();
    let r = set_req(Method::SmartLaplacian { relax: 1.0 }, "hot", false);
    let pool = |t: usize| rayon::ThreadPoolBuilder::new().num_threads(t).build().expect("pool").install(|| bits(&run(&f, &r).points));
    let (a, b, c) = (pool(1), pool(4), bits(&run(&f, &r).points));
    assert_eq!(a, b);
    assert_eq!(a, c);
    assert_eq!(a, pool(1));
}

mod op_level {
    use super::*;
    use serde_json::json;

    fn jittered() -> (sm_op_api::Session, sm_core::EntityId, Vec<[f64; 3]>) {
        let (mut s, mesh) = meshed_brick([4.0; 3], 1.0);
        apply(&mut s, &JitterMesh { mesh, seed: 8, amp: 0.2, volume_only: true }).expect("jitter");
        let start = mesh_points(&s, mesh);
        (s, mesh, start)
    }

    #[test]
    fn smooth_set_targets_the_flag_cells_set() {
        let (mut s, mesh, start) = jittered();
        let (_, d) = run_json(
            &mut s,
            "mesh.flag_cells",
            json!({"name": "bad", "metrics": ["scaled_jacobian"], "thresholds": {"scaled_jacobian": {"warn": 0.9, "fail": 0.95}}}),
        )
        .expect("flag");
        assert!(!d.outputs["set"].is_empty(), "a set was stored");
        let in_set: Vec<bool> = {
            let layer = s.state().layer::<sm_layer_api::MeshLayer>().expect("layer");
            let m = layer.get(mesh).expect("mesh").mesh();
            let set = m.set_by_name("bad").expect("set");
            let SetMembers::Cells(c) = &set.members else { panic!("cell set") };
            assert!(c.len() < m.cell_count(), "partial set");
            corners_of(m, c)
        };
        run_json(&mut s, "mesh.smooth", json!({"mesh": mesh.to_string(), "set": "bad", "method": {"kind": "laplacian"}, "iterations": 5}))
            .expect("smooth set");
        let end = mesh_points(&s, mesh);
        let (b, a) = (bits(&start), bits(&end));
        let mut moved = 0;
        for i in 0..b.len() {
            if in_set[i] {
                moved += usize::from(b[i] != a[i]);
            } else {
                assert_eq!(b[i], a[i], "node {i} outside the set moved");
            }
        }
        assert!(moved > 0);
    }

    #[test]
    fn unknown_set_and_conflicts_are_typed_errors() {
        let (mut s, mesh, _) = jittered();
        let h0 = s.state().hash();
        let err = run_json(&mut s, "mesh.smooth", json!({"mesh": mesh.to_string(), "set": "nope", "method": {"kind": "laplacian"}}))
            .expect_err("unknown set")
            .to_string();
        assert!(err.contains("nope") && err.contains("mesh sets"), "{err}");
        assert_eq!(s.state().hash(), h0);
        let both = json!({"mesh": mesh.to_string(), "set": "x", "entities": ["TopoVolume#1"], "method": {"kind": "laplacian"}});
        assert!(run_json(&mut s, "mesh.smooth", both.clone()).is_err());
        assert!(query(&s, "mesh.smooth_preview", both).is_err());
        assert_eq!(s.state().hash(), h0);
    }
}
