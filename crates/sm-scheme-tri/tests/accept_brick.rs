//! Acceptance 8: `Driver` integration with the stub registry plus this crate's `register`.

mod common;

use common::*;
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{FaceIx, FaceKind, MeshRead, SerialAlloc, validate};
use sm_scheme_api::stubs::{brick_setup, registry as stub_registry};
use sm_scheme_api::{
    Driver, Firm, Firmness, IntervalRequest, MeshInput, MeshIntent, ReferenceCornerPicker,
    ReferenceIntervalSolver, SchemeChoice, Sequential, WholeFaces,
};

/// The stubs plus this crate's schemes: no name collides (this crate never registers `tri`).
fn registry_with_stubs() -> sm_scheme_api::SchemeRegistry {
    let mut r = stub_registry();
    sm_scheme_tri::register(&mut r).expect("no collision with the stubs");
    r
}

#[test]
fn acc8_brick_surface_with_trimesh_is_closed_valid_and_exact() {
    let (topo, ids, model) = brick_setup([1.0, 1.5, 2.0]);
    let reg = registry_with_stubs();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = ids.faces;
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    // Edges 0-3 run along x (length 1), 4-7 along y (1.5), 8-11 along z (2): spacing 1/6.
    for (i, e) in ids.edges.iter().enumerate() {
        let c = [6u32, 9, 12][i / 4];
        intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(c)));
    }
    for f in ids.faces {
        intent.set_scheme(f, SchemeChoice::new("trimesh", Firmness::Hard));
    }
    let run = |exec: &dyn sm_scheme_api::Executor| {
        driver
            .mesh(&input, &intent, exec, ReferenceBuilder::surface(), &mut SerialAlloc::new())
            .expect("surface mesh")
    };
    let out = run(&Sequential);
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(euler(m), 2);
    // Closed: every facet has two cells; shared edges exist once (distinct vertex pairs).
    assert!((0..m.face_count()).all(|f| m.face_kind(FaceIx::new(f)) == FaceKind::Interior));
    let used = edge_use(m);
    assert_eq!(used.len(), m.face_count(), "shared edges are not duplicated");
    assert!(used.values().all(|&n| n == 2));
    // Total area 2 (1.5 * 2 + 1 * 2 + 1 * 1.5) = 13, exact to 1e-12.
    assert!((mesh_area(m) - 13.0).abs() <= 1e-12 * 13.0, "{}", mesh_area(m));
    // Every triangle follows its face's outward normal.
    for c in 0..m.cell_count() {
        let ci = sm_mesh_api::CellIx::new(c);
        let cyc = m.cell_corners(ci).unwrap();
        let p: Vec<_> = cyc.iter().map(|v| m.point(v.get())).collect();
        let n = (p[1] - p[0]).cross(p[2] - p[0]);
        let want = sm_scheme_api::geom::face_normal(&topo, m.cell_class(ci).entity, p[0]).unwrap();
        assert!(n.dot(want) > 0.0, "cell {c}");
    }
    // The same mesh with four threads.
    let again = run(&sm_scheme_api::ScopedThreads(4));
    assert_eq!(sm_mesh_api::topology_hash(&again.finished.mesh), sm_mesh_api::topology_hash(m));
    assert_eq!(sm_mesh_api::geometry_hash(&again.finished.mesh), sm_mesh_api::geometry_hash(m));
}

#[test]
fn acc8_planar_face_with_two_embedded_edges_conforms() {
    let mut fx = sm_testkit::topo::planar_face(&sm_testkit::loops2d::square(), 0.0).unwrap();
    let (wires, _) = embed_wires(
        &mut fx,
        0.0,
        &[([0.2, 0.3], [0.8, 0.3]), ([0.3, 0.6], [0.7, 0.9])],
        &[],
    );
    let mut counts = counts_for(&fx, 0.1);
    counts.push((wires[0], 6));
    counts.push((wires[1], 4));
    let out = mesh_faces(&fx.complex, &[fx.face], &counts, "tridelaunay", &Sequential).expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty());
    use sm_topo_api::TopologyViewExt;
    let used = edge_use(m);
    for (e, n) in [(wires[0], 6u32), (wires[1], 4)] {
        let (a, b) = fx.complex.edge_ends(e).unwrap();
        let mut keys = vec![sm_scheme_api::NodeKey::vertex(a)];
        keys.extend((1..n).map(|i| sm_scheme_api::NodeKey::new(e, i)));
        keys.push(sm_scheme_api::NodeKey::vertex(b));
        for w in keys.windows(2) {
            let (x, y) = (out.vertex_of(w[0]).unwrap().0 as usize, out.vertex_of(w[1]).unwrap().0 as usize);
            assert!(used.contains_key(&(x.min(y), x.max(y))), "edge {e} segment {w:?}");
        }
    }
    assert!((mesh_area(m) - 1.0).abs() <= 1e-12);
}

#[test]
fn registration_never_registers_tri_chains_or_auto_lists() {
    use sm_scheme_api::{SchemeDim, SchemeRegistry};
    let mut r = SchemeRegistry::new();
    sm_scheme_tri::register(&mut r).unwrap();
    let names: Vec<&str> = r.of_dim(SchemeDim::Surface).map(|s| s.name()).collect();
    assert_eq!(names, ["triadvance", "tridelaunay", "trimesh"]);
    assert!(r.auto(SchemeDim::Surface).is_empty());
    assert!(r.fallbacks(SchemeDim::Surface, "trimesh").is_empty());
    assert!(r.get(SchemeDim::Surface, "tri").is_none());
    assert!(sm_scheme_tri::register(&mut r).is_err(), "double registration is refused");
}
