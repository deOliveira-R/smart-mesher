//! Acceptance 5: nodes stay on their geometry, and the trim-reject path.

#![allow(clippy::needless_range_loop)]

mod common;

use common::*;
use sm_core::{EntityKind, Point3};
use sm_mesh_api::MeshEdit;
use sm_mesh_api::{Classification, MeshEntityKind, MeshRead, Param, VertexIx};
use sm_smooth::{Method, NodeSelect, Objective, Schedule, SmoothRequest, smooth};
use sm_testkit::lattice;
use sm_topo_api::{Carrier, TopologyView};

fn methods() -> Vec<(&'static str, Method)> {
    vec![
        ("laplacian", Method::Laplacian { relax: 1.0 }),
        ("smart", Method::SmartLaplacian { relax: 1.0 }),
        ("centroid", Method::CentroidArea { relax: 1.0 }),
        ("optimize", Method::Optimize { objective: Objective::MeanRatio, p: 4.0 }),
        ("condition", Method::Optimize { objective: Objective::Condition, p: 4.0 }),
        ("winslow", Method::Winslow { relax: 1.0 }),
    ]
}

fn hyp(p: &[f64; 3]) -> f64 {
    p[0].hypot(p[1])
}

/// Check every node of the applied mesh against its geometry; returns the worst violation.
fn check_on_geometry(f: &lattice::LatticeFixture, mesh: &dyn MeshRead, label: &str) {
    let pts = pts_of(mesh);
    let mut worst = 0.0f64;
    for (i, p) in pts.iter().enumerate() {
        let c: Classification = mesh.classification(MeshEntityKind::Vertex, i as u32);
        match c.entity.kind() {
            EntityKind::TopoVertex => {
                for k in 0..3 {
                    assert_eq!(p[k].to_bits(), f.exact[i][k].to_bits(), "{label}: vertex node {i}");
                }
            }
            EntityKind::TopoFace => {
                worst = worst.max((hyp(p) - 1.0).abs());
                assert!((hyp(p) - 1.0).abs() <= 1e-12, "{label}: face node {i} r = {}", hyp(p));
                let Carrier::Surface(sc) = f.topo.carrier(c.entity) else { panic!("surface carrier") };
                let s = f.topo.shapes().surface(sc.surface).expect("surface");
                let Param::Surface([u, v]) = c.param else { panic!("{label}: face node {i} lost its Param: {:?}", c.param) };
                let q = s.eval(sm_geom_api::Uv::new(u, v));
                let d = ((q.x - p[0]).powi(2) + (q.y - p[1]).powi(2) + (q.z - p[2]).powi(2)).sqrt();
                assert!(d <= 1e-12, "{label}: face node {i}: eval(uv) differs by {d:e}");
            }
            EntityKind::TopoEdge => {
                let Carrier::Curve(cc) = f.topo.carrier(c.entity) else { panic!("curve carrier") };
                let curve = f.topo.shapes().curve(cc.curve).expect("curve");
                let Param::Curve(t) = c.param else { panic!("{label}: edge node {i} lost its Param") };
                assert!(t >= cc.range.lo && t <= cc.range.hi, "{label}: edge node {i}: t = {t} outside {:?}", cc.range);
                let q = curve.eval(t);
                let d = ((q.x - p[0]).powi(2) + (q.y - p[1]).powi(2) + (q.z - p[2]).powi(2)).sqrt();
                assert!(d <= 1e-12, "{label}: edge node {i}: eval(t) differs by {d:e}");
                // circle edges: r = 1 and z in {0, 2}; seam: on the line (1, 0, z)
                let on_circle = (hyp(p) - 1.0).abs() <= 1e-12 && (p[2].abs() <= 1e-12 || (p[2] - 2.0).abs() <= 1e-12);
                let on_seam = (p[0] - 1.0).abs() <= 1e-12 && p[1].abs() <= 1e-12;
                assert!(on_circle || on_seam, "{label}: edge node {i} at {p:?} is on neither a circle nor the seam");
            }
            k => panic!("unexpected class {k:?}"),
        }
    }
    let _ = worst;
}

#[test]
fn a5_nodes_stay_on_the_cylinder_for_every_method() {
    let f = lattice::tube_quad(12, 6, 1.0, 2.0).expect("tube");
    let p = f.perturbed(9, 0.2).expect("perturb");
    for (name, m) in methods() {
        for sched in [Schedule::Colored, Schedule::Jacobi] {
            let o = run(&p, &req(m, sched, 60, false));
            let mesh = applied(&p, &o);
            check_on_geometry(&p, &mesh, &format!("{name} {sched:?}"));
            assert!(o.report.accepted > 0, "{name}: nothing moved");
            assert!(o.report.movable > 0);
            // edge and face nodes were refreshed
            assert!(!o.classes.is_empty());
            eprintln!("a5 {name} {sched:?}: sweeps {} accepted {} rej_guard {} rej_geom {}", o.report.iterations_run, o.report.accepted, o.report.rejected_guard, o.report.rejected_geometry);
        }
    }
}

#[test]
fn a5_laplacian_and_centroid_restore_equal_spacing() {
    let f = lattice::tube_quad(12, 6, 1.0, 2.0).expect("tube");
    let p = f.perturbed(9, 0.2).expect("perturb");
    for (name, m) in [("laplacian", Method::Laplacian { relax: 1.0 }), ("centroid", Method::CentroidArea { relax: 1.0 })] {
        let mut r = req(m, Schedule::Colored, 400, false);
        r.tolerance = 0.0;
        let o = run(&p, &r);
        // angular and axial spacing
        let (mut dth, mut dz) = (0.0f64, 0.0f64);
        for (i, q) in o.points.iter().enumerate() {
            let e = f.exact[i];
            let mut d = q[1].atan2(q[0]) - e[1].atan2(e[0]);
            d = (d + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
            dth = dth.max(d.abs());
            dz = dz.max((q[2] - e[2]).abs());
        }
        eprintln!("a5 {name}: max angular error {dth:e}, axial error {dz:e}, dev {:e}", max_dev(&o.points, &f.exact));
        assert!(dth <= 1e-6 && dz <= 1e-6, "{name}: {dth:e} {dz:e}");
    }
}

#[test]
fn a5_a_projection_outside_the_trimmed_face_is_rejected() {
    // 4x4 sheet on [0,1]^2. The four edge neighbours of the centre node are moved (test-side)
    // to (10, 10): the Laplacian target projects onto the plane at (10, 10), outside the trim.
    let f = lattice::quad([4, 4], [1.0; 2]).expect("quad");
    let mut mesh = f.mesh.clone();
    let centre = VertexIx::new(2 * 5 + 2);
    for (di, dj) in [(1i32, 0i32), (-1, 0), (0, 1), (0, -1)] {
        let v = VertexIx::new(((2 + dj) * 5 + 2 + di) as usize);
        mesh.set_point(v.into(), Point3::new(10.0, 10.0, 0.0)).expect("set");
    }
    let input = sm_smooth::SmoothInput { mesh: &mesh, topo: &f.topo, model: &f.model };
    let r = SmoothRequest {
        nodes: NodeSelect::Vertices(vec![centre]),
        method: Method::Laplacian { relax: 1.0 },
        iterations: 1,
        tolerance: 0.0,
        schedule: Schedule::Colored,
        ..SmoothRequest::default()
    };
    let before = mesh.point(centre.get());
    let o = smooth(&input, &r).expect("smooth");
    assert_eq!(o.report.rejected_geometry, 1);
    assert_eq!(o.report.accepted, 0);
    for (k, b) in [before.x, before.y, before.z].iter().enumerate() {
        assert_eq!(o.points[centre.get()][k].to_bits(), b.to_bits(), "the node did not move");
    }
    assert!(o.classes.is_empty());
}
