//! Acceptance 4: the guard makes "never inverts" a contract on every method.

mod common;

use common::*;
use sm_core::{EntityId, Point3};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::{CellShape, Classification, FaceIx, MeshBuild, MeshEdit, MeshEntityKind, MeshEntityRef, MeshRead, PointIx, SerialAlloc, VertexIx};
use sm_quality::{Metric, Options, SurfaceNormals, compute, summarize, validity};
use sm_smooth::{Method, NodeSelect, Objective, Schedule, SmoothInput, SmoothRequest, smooth};
use sm_testkit::lattice;
use sm_testkit::surf3::{SplitMix64, mix64};
use sm_topo_api::TopologyView;

fn opts() -> Options<'static> {
    Options::default().with_normals(SurfaceNormals::Uniform([0.0, 0.0, 1.0]))
}

fn min_sj(m: &dyn MeshRead) -> f64 {
    summarize(&compute(m, Metric::ScaledJacobian, &opts()).expect("sj")).min
}

fn methods() -> Vec<(String, Method)> {
    let mut v = Vec::new();
    for relax in [1.0, 1.5, 1.9] {
        v.push((format!("laplacian r{relax}"), Method::Laplacian { relax }));
        v.push((format!("smart r{relax}"), Method::SmartLaplacian { relax }));
        v.push((format!("centroid r{relax}"), Method::CentroidArea { relax }));
    }
    for p in [2.0, 4.0] {
        v.push((format!("optimize p{p}"), Method::Optimize { objective: Objective::MeanRatio, p }));
    }
    v
}

/// First seed (deterministic scan) whose 0.45h jitter keeps the 12x12 lattice valid.
fn valid_seed(f: &lattice::LatticeFixture) -> u64 {
    (0..200u64)
        .find(|s| {
            let p = f.perturbed(*s, 0.45).expect("perturb");
            validity(&p.mesh, &opts()).expect("validity").all_valid()
        })
        .expect("some seed keeps the lattice valid")
}

#[test]
fn a4_every_method_keeps_a_jittered_quad_lattice_valid() {
    let f = lattice::quad([12, 12], [1.0; 2]).expect("lattice");
    let seed = valid_seed(&f);
    let p = f.perturbed(seed, 0.45).expect("perturb");
    assert!(validity(&p.mesh, &opts()).expect("validity").all_valid(), "precondition");
    let sj0 = min_sj(&p.mesh);
    let mut runs = 0;
    for (name, m) in methods() {
        for sched in [Schedule::Colored, Schedule::Jacobi] {
            for iters in 1..=20 {
                let mut r = req(m, sched, iters, false);
                r.tolerance = 0.0;
                let o = run(&p, &r);
                let mesh = applied(&p, &o);
                let v = validity(&mesh, &opts()).expect("validity");
                assert!(v.all_valid(), "{name} {sched:?} {iters}: {} inverted {} degenerate", v.n_inverted, v.n_degenerate);
                let sj = min_sj(&mesh);
                assert!(sj >= sj0.min(1e-3), "{name} {sched:?} {iters}: min SJ {sj} < min({sj0}, 1e-3)");
                runs += 1;
            }
        }
    }
    eprintln!("a4: seed {seed}, {runs} runs valid; initial min SJ {sj0}");
}

/// An L-shaped quad region `[0,n]^2 minus [k,n]^2` (cells of side 1): boundary nodes are
/// classified on a `TopoVertex` (fixed), interior nodes on the planar face. `corner` moves the
/// reflex corner node.
fn l_mesh(f: &lattice::LatticeFixture, n: usize, k: usize, corner: (f64, f64)) -> ReferenceMesh {
    let face = f.mesh.classification(MeshEntityKind::Cell, 0).entity;
    // An edge of the face and one of its end vertices.
    let edge: EntityId = f.topo.boundary(face).iter().map(|i| i.cell).find(|c| c.kind() == sm_core::EntityKind::TopoEdge).expect("edge");
    let vert: EntityId = f.topo.boundary(edge).first().expect("vertex").cell;
    let cell_in = |i: usize, j: usize| i < n && j < n && (i < k || j < k);
    let mut b = ReferenceBuilder::surface();
    let mut ix = vec![usize::MAX; (n + 1) * (n + 1)];
    for j in 0..=n {
        for i in 0..=n {
            let around: Vec<bool> = [i.wrapping_sub(1), i]
                .iter()
                .flat_map(|&a| [j.wrapping_sub(1), j].into_iter().map(move |c| (a, c)))
                .map(|(a, c)| cell_in(a, c))
                .collect();
            let cnt = around.iter().filter(|x| **x).count();
            if cnt == 0 {
                continue;
            }
            let mut p = Point3::new(i as f64, j as f64, 0.0);
            if (i, j) == (k, k) {
                p = Point3::new(p.x + corner.0, p.y + corner.1, 0.0);
            }
            let v = b.add_vertex(p).expect("vertex");
            ix[j * (n + 1) + i] = v.get();
            let e = if cnt < 4 { vert } else { face };
            b.set_class(MeshEntityRef::Vertex(v), Classification::on(e)).expect("class");
        }
    }
    for j in 0..n {
        for i in 0..n {
            if !cell_in(i, j) {
                continue;
            }
            let at = |di: usize, dj: usize| VertexIx::new(ix[(j + dj) * (n + 1) + i + di]);
            let c = b.add_cell_from_vertices(CellShape::Quad, &[at(0, 0), at(1, 0), at(1, 1), at(0, 1)]).expect("cell");
            b.set_class(MeshEntityRef::Cell(c.cell), Classification::on(face)).expect("class");
        }
    }
    for fi in 0..b.face_count() {
        let class = if b.face_degree(FaceIx::new(fi)) == 1 { edge } else { face };
        b.set_class(MeshEntityRef::Face(FaceIx::new(fi)), Classification::on(class)).expect("class");
    }
    b.finish(&mut SerialAlloc::new()).expect("finish").mesh
}

/// A test-local unguarded Jacobi Laplacian over the interior (face-classified) nodes.
fn unguarded(m: &ReferenceMesh, sweeps: usize, relax: f64) -> Vec<[f64; 3]> {
    let mut pos = pts_of(m);
    let n = pos.len();
    let mut nb: Vec<Vec<usize>> = vec![Vec::new(); n];
    for fi in 0..m.face_count() {
        let l = m.face_vertices(FaceIx::new(fi));
        let (a, c) = (l[0].get(), l[1].get());
        nb[a].push(c);
        nb[c].push(a);
    }
    for v in &mut nb {
        v.sort_unstable();
        v.dedup();
    }
    for _ in 0..sweeps {
        let old = pos.clone();
        for v in 0..n {
            if m.vertex_class(VertexIx::new(v)).entity.kind() != sm_core::EntityKind::TopoFace {
                continue;
            }
            let k = nb[v].len() as f64;
            let mut s = [0.0; 3];
            for &w in &nb[v] {
                for c in 0..3 {
                    s[c] += old[w][c];
                }
            }
            for c in 0..3 {
                pos[v][c] = old[v][c] + relax * (s[c] / k - old[v][c]);
            }
        }
    }
    pos
}

/// The L-shaped case found by scan: reflex corner pulled by (-0.5, -0.5), interior jitter 0.3
/// with seed 19. One unguarded Jacobi Laplacian sweep inverts a cell.
fn l_case() -> (lattice::LatticeFixture, ReferenceMesh) {
    let f = lattice::quad([8, 8], [8.0, 8.0]).expect("lattice"); // the trimmed face must contain the L
    let mut m = l_mesh(&f, 8, 4, (-0.5, -0.5));
    for v in 0..m.vertex_count() {
        if m.vertex_class(VertexIx::new(v)).entity.kind() != sm_core::EntityKind::TopoFace {
            continue;
        }
        let mut h = SplitMix64(mix64(19) ^ mix64(v as u64));
        let p = m.point(v);
        m.set_point(PointIx::new(v), Point3::new(p.x + 0.3 * h.unit(), p.y + 0.3 * h.unit(), 0.0)).expect("set");
    }
    (f, m)
}

#[test]
fn a4_the_guard_is_exercised_on_an_l_shaped_region() {
    let (f, m) = l_case();
    assert!(validity(&m, &opts()).expect("v").all_valid(), "precondition: the L mesh starts valid");
    // A test-local unguarded Jacobi Laplacian sweep inverts a cell ...
    let mut bad = m.clone();
    set_points(&mut bad, &unguarded(&m, 1, 1.0));
    let n = validity(&bad, &opts()).expect("v").n_inverted;
    assert!(n >= 1, "the unguarded sweep must invert a cell");
    // ... ours never does, for every method and schedule.
    let sj0 = min_sj(&m);
    let input = SmoothInput { mesh: &m, topo: &f.topo, model: &f.model };
    let mut refused = 0;
    for (name, method) in methods() {
        for sched in [Schedule::Colored, Schedule::Jacobi] {
            for iters in [1, 2, 5, 20] {
                let r = SmoothRequest { nodes: NodeSelect::All, method, iterations: iters, tolerance: 0.0, schedule: sched, ..SmoothRequest::default() };
                let o = smooth(&input, &r).expect("smooth");
                let mut mm = m.clone();
                set_points(&mut mm, &o.points);
                assert!(validity(&mm, &opts()).expect("v").all_valid(), "{name} {sched:?} {iters}");
                assert!(min_sj(&mm) >= sj0.min(1e-3), "{name} {sched:?} {iters}");
                refused += o.report.rejected_guard;
            }
        }
    }
    assert!(refused > 0, "the guard refused at least one move");
    eprintln!("a4 L: unguarded sweep inverts {n} cell(s); the guard refused {refused} moves over the matrix");
}

#[test]
fn a4_an_inverted_input_never_gets_worse() {
    let f = lattice::quad([12, 12], [1.0; 2]).expect("lattice");
    let p = (0..200u64)
        .map(|s| f.perturbed(s, 0.45).expect("perturb"))
        .find(|p| validity(&p.mesh, &opts()).expect("v").n_inverted == 1)
        .expect("a seed with exactly one inverted cell");
    let sj0 = min_sj(&p.mesh);
    for (name, m) in methods() {
        for sched in [Schedule::Colored, Schedule::Jacobi] {
            let mut r = req(m, sched, 20, false);
            r.tolerance = 0.0;
            let o = run(&p, &r);
            assert_eq!(o.report.before.n_inverted, 1);
            assert!(o.report.after.n_inverted <= 1, "{name} {sched:?}");
            let mesh = applied(&p, &o);
            assert!(min_sj(&mesh) >= sj0 - 1e-15, "{name} {sched:?}: {} < {sj0}", min_sj(&mesh));
            assert_eq!(validity(&mesh, &opts()).expect("v").n_inverted, o.report.after.n_inverted);
            assert_eq!(o.report.inverted_before.len(), 1);
        }
    }
}
