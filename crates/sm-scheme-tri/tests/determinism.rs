//! Acceptance 9: bit-identical results across runs, executors and thread counts.

mod common;

use common::*;
use sm_mesh_api::{MeshRead, geometry_hash, topology_hash};
use sm_scheme_api::stubs::{brick_setup, registry as stub_registry};
use sm_scheme_api::{
    Driver, Executor, Firm, IntervalRequest, MeshInput, MeshIntent, ReferenceCornerPicker,
    ReferenceIntervalSolver, ScopedThreads, Sequential, SchemeChoice, Firmness, WholeFaces,
};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::SerialAlloc;
use sm_scheme_tri::cdt::{Cdt, OptimizeOptions, RefineOptions};
use sm_testkit::determinism::{check, hash_f64_bits};
use sm_testkit::loops2d;
use sm_testkit::topo::planar_face;
use sm_topo_api::fixtures::{cylinder, sphere, with_op};
use sm_topo_api::{CellComplex, OpId};

fn kernel_hash(r: &loops2d::Region2, h: f64) -> u64 {
    let mut p = pslg_of(r);
    // Refine at the boundary spacing `h` (the Flat geometry's size).
    let mut c = Cdt::new(&p).expect("cdt");
    let g = Flat(h);
    c.refine(&g, &RefineOptions::default());
    c.optimize(&g, &OptimizeOptions::default());
    p.points.clear();
    let coords: Vec<f64> = c.points().iter().flat_map(|q| [q[0], q[1]]).collect();
    let tris: Vec<f64> = c.triangles().iter().flatten().map(|&i| f64::from(i)).collect();
    hash_f64_bits(&coords) ^ hash_f64_bits(&tris).rotate_left(17)
}

#[test]
fn acc9_pure_kernel_is_deterministic() {
    for (r, h) in [
        (loops2d::square(), 0.25),
        (loops2d::annulus(1.0, 3.0, 48), 0.4),
        (loops2d::l_shape(), 0.25),
        (loops2d::rectangle_with_holes(), 0.6),
    ] {
        let hash = check(2, 4, |_threads| kernel_hash(&r, h)).unwrap_or_else(|e| panic!("{}: {e}", r.name));
        assert_ne!(hash, 0);
    }
}

fn driver_hashes(threads: usize) -> Vec<(u64, u64, u64)> {
    let exec: Box<dyn Executor> = if threads == 1 {
        Box::new(Sequential)
    } else {
        Box::new(ScopedThreads(threads))
    };
    let mut out = Vec::new();
    // Brick surface, cylinder lateral face, sphere.
    {
        let (topo, ids, model) = brick_setup([1.0, 1.5, 2.0]);
        let mut reg = stub_registry();
        sm_scheme_tri::register(&mut reg).unwrap();
        let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
        let driver = Driver::new(&reg, &solver, &picker);
        let sel = ids.faces;
        let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
        let mut intent = MeshIntent::default();
        for (i, e) in ids.edges.iter().enumerate() {
            intent.set_intervals(*e, Firm::hard(IntervalRequest::Count([6u32, 9, 12][i / 4])));
        }
        for f in ids.faces {
            intent.set_scheme(f, SchemeChoice::new("trimesh", Firmness::Hard));
        }
        let o = driver
            .mesh(&input, &intent, exec.as_ref(), ReferenceBuilder::surface(), &mut SerialAlloc::new())
            .unwrap();
        out.push(hashes(&o.finished.mesh));
    }
    {
        let mut c = CellComplex::new();
        let ids = with_op(&mut c, OpId(1), |t| cylinder(t, 1.0, 2.0)).unwrap();
        let counts = vec![(ids.edges[0], 12u32), (ids.edges[1], 12), (ids.edges[2], 6)];
        let o = mesh_faces(&c, &[ids.faces[2]], &counts, "trimesh", exec.as_ref()).unwrap();
        out.push(hashes(&o.finished.mesh));
    }
    {
        let mut c = CellComplex::new();
        let ids = with_op(&mut c, OpId(1), |t| sphere(t, 1.0)).unwrap();
        let o = mesh_faces(&c, &[ids.face], &[(ids.edges[1], 16)], "trimesh", exec.as_ref()).unwrap();
        out.push(hashes(&o.finished.mesh));
    }
    {
        let fx = planar_face(&loops2d::rectangle_with_holes(), 0.0).unwrap();
        let counts = counts_for(&fx, 0.5);
        let o = mesh_faces(&fx.complex, &[fx.face], &counts, "tridelaunay", exec.as_ref()).unwrap();
        out.push(hashes(&o.finished.mesh));
    }
    out
}

fn hashes(m: &sm_mesh_api::reference::ReferenceMesh) -> (u64, u64, u64) {
    let coords: Vec<f64> = (0..m.vertex_count())
        .flat_map(|v| {
            let p = m.point(v);
            [p.x, p.y, p.z]
        })
        .collect();
    (topology_hash(m), geometry_hash(m), hash_f64_bits(&coords))
}

#[test]
fn acc9_driver_sequential_vs_scoped_threads_is_bit_identical() {
    let base = check(2, 4, driver_hashes).expect("deterministic");
    assert_eq!(base.len(), 4);
    // And the executor flavours agree with each other explicitly.
    assert_eq!(driver_hashes(1), driver_hashes(4));
}

#[test]
fn acc9_no_hash_containers_in_the_sources() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    fn walk(d: &std::path::Path, bad: &mut Vec<String>) {
        for e in std::fs::read_dir(d).unwrap() {
            let p = e.unwrap().path();
            if p.is_dir() {
                walk(&p, bad);
            } else if p.extension().is_some_and(|x| x == "rs") {
                let s = std::fs::read_to_string(&p).unwrap();
                if s.contains("HashMap") || s.contains("HashSet") {
                    bad.push(p.display().to_string());
                }
            }
        }
    }
    let mut bad = Vec::new();
    walk(&dir, &mut bad);
    assert!(bad.is_empty(), "HashMap/HashSet in {bad:?}");
}
