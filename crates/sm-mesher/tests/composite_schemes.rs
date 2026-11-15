//! W4-VTOPO-MESH: every surface scheme meshes a planar composite face exactly like its host
//! face, and a curved non-periodic composite is projected onto its surface.

use std::sync::Arc;

use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{CurveDef, Interval, Line, NurbsSurface, SurfaceDef};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::{MeshRead, SerialAlloc};
use sm_scheme_api::stubs::{Equal, brick_setup};
use sm_scheme_api::{
    Driver, Firm, Firmness, IntervalRequest, MeshInput, MeshIntent, ReferenceCornerPicker,
    ReferenceIntervalSolver, SchemeChoice, SchemeRegistry, Sequential, WholeFaces,
};
use sm_topo_api::fixtures::{BrickIds, with_op};
use sm_topo_api::{
    CellComplex, CoedgeSpec, CurveCarrier, FaceCut, InnerLoopAssign, OpId, Overlay, Sense,
    SurfaceCarrier, TopologyEdit, TopologyView,
};

const SCHEMES: [&str; 6] = ["pave", "map", "submap", "trimesh", "tridelaunay", "triadvance"];

fn registry() -> SchemeRegistry {
    let mut reg = SchemeRegistry::new();
    reg.register(Box::new(Equal)).expect("equal");
    sm_scheme_pave::register(&mut reg).expect("pave");
    sm_scheme_map::register(&mut reg).expect("map");
    sm_scheme_submap::register(&mut reg).expect("submap");
    sm_scheme_tri::register(&mut reg).expect("tri");
    reg
}

/// Mesh face `f` of `topo` with `scheme` forced on it.
fn mesh_face(
    topo: &dyn TopologyView,
    f: EntityId,
    scheme: &str,
    mut intent: MeshIntent,
) -> ReferenceMesh {
    intent.set_scheme(
        f,
        SchemeChoice::with_params(scheme, serde_json::json!({}), Firmness::Hard),
    );
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let model = sm_model_api::Model::new();
    let sel = [f];
    let input = MeshInput::new(topo, &model, &WholeFaces, &sel);
    driver
        .mesh(&input, &intent, &Sequential, ReferenceBuilder::surface(), &mut SerialAlloc::new())
        .unwrap_or_else(|e| panic!("{scheme}: {e}"))
        .finished
        .mesh
}

fn counts(edges: &[EntityId], n: u32) -> MeshIntent {
    let mut intent = MeshIntent::default();
    for e in edges {
        intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(n)));
    }
    intent
}

/// Node coordinates sorted lexicographically (ids differ between host and composite).
fn sorted_points(m: &dyn MeshRead) -> Vec<Point3> {
    let q = |v: f64| (v * 1e7).round() as i64;
    let mut p: Vec<Point3> = (0..m.vertex_count()).map(|v| m.point(v)).collect();
    p.sort_by_key(|p| (q(p.x), q(p.y), q(p.z)));
    p
}

fn min_scaled_jacobian(m: &dyn MeshRead) -> f64 {
    sm_quality::report(m, &[sm_quality::Metric::ScaledJacobian], &sm_quality::Options::default())
        .expect("quality")
        .remove(0)
        .1
        .min
}

/// The top face of a unit brick, split by its diagonal and composited back.
fn composite_top() -> (Overlay, BrickIds, EntityId) {
    let (mut t, ids, _) = brick_setup([1.0; 3]);
    let halves = with_op(&mut t, OpId(2), |t| {
        let line = Line::new(Point3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 1.0, 0.0)).unwrap();
        let curve = t.shapes_mut().unwrap().add_curve(CurveDef::Line(line)).unwrap();
        let chord = t.make_edge(
            ids.vertices[4],
            ids.vertices[7],
            Some(CurveCarrier { curve, range: Interval::new(0.0, 2f64.sqrt()), sense: Sense::Forward }),
        )?;
        Ok(t.split_face(ids.faces[5], FaceCut::Chord { edge: chord }, InnerLoopAssign::None)?)
    })
    .unwrap();
    let mut ov = Overlay::new(Arc::new(t));
    let vf = ov.composite_faces(&halves).unwrap();
    (ov, ids, vf)
}

#[test]
fn every_scheme_meshes_a_planar_composite_like_its_host_face() {
    let (host, ids, _) = brick_setup([1.0; 3]);
    let (ov, ids2, vf) = composite_top();
    let top_edges: Vec<EntityId> = host.face_edges(ids.faces[5]).to_vec();
    assert_eq!(top_edges.len(), 4);
    for name in SCHEMES {
        let want = mesh_face(&host, ids.faces[5], name, counts(&top_edges, 4));
        let got = mesh_face(&ov, vf, name, counts(&top_edges, 4));
        assert_eq!(got.cell_count(), want.cell_count(), "{name}: cells");
        assert_eq!(got.vertex_count(), want.vertex_count(), "{name}: nodes");
        let (a, b) = (sorted_points(&got), sorted_points(&want));
        for (p, q) in a.iter().zip(&b) {
            assert!((*p - *q).norm() < 1e-9, "{name}: {p:?} vs {q:?}");
            assert!((p.z - 1.0).abs() < 1e-12, "{name}: node off the plane {p:?}");
        }
        let sj = min_scaled_jacobian(&got);
        eprintln!(
            "composite-vs-host {name}: cells {} nodes {} min scaled Jacobian {sj:.4} (host {:.4})",
            got.cell_count(),
            got.vertex_count(),
            min_scaled_jacobian(&want)
        );
        assert!(sj > 0.0, "{name}: inverted cell");
        let _ = &ids2;
    }
}

/// Bilinear patch through `P(u, v) = (2u, v, 0.3 (u (1 - v) + v (1 - u)))`, `u, v` in `[0, 1]`
/// (curved, non-periodic; its iso-lines are straight), cut by the iso-line `u = 1/2` into two
/// quads on one surface handle, as a composite.
fn curved_composite() -> (Overlay, EntityId, Vec<EntityId>, SurfaceDef) {
    let z = |u: f64, v: f64| 0.3 * (u * (1.0 - v) + v * (1.0 - u));
    let pt = |u: f64, v: f64| Point3::new(2.0 * u, v, z(u, v));
    let ctrl = vec![pt(0.0, 0.0), pt(0.0, 1.0), pt(1.0, 0.0), pt(1.0, 1.0)];
    let def = SurfaceDef::Nurbs(
        NurbsSurface::new(1, 1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0], 2, 2, ctrl, None).unwrap(),
    );
    let mut t = CellComplex::new();
    let mut edges = Vec::new();
    let faces = with_op(&mut t, OpId(1), |t| {
        let uv = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.0, 1.0), (0.5, 1.0), (0.0, 1.0)];
        let v: Vec<EntityId> = uv.iter().map(|q| t.make_vertex(pt(q.0, q.1)).unwrap()).collect();
        let edge = |t: &mut CellComplex, a: usize, b: usize| {
            let (pa, pb) = (pt(uv[a].0, uv[a].1), pt(uv[b].0, uv[b].1));
            let curve = t
                .shapes_mut()
                .unwrap()
                .add_curve(CurveDef::Line(Line::new(pa, pb - pa).unwrap()))
                .unwrap();
            t.make_edge(
                v[a],
                v[b],
                Some(CurveCarrier { curve, range: Interval::new(0.0, 1.0), sense: Sense::Forward }),
            )
        };
        let e = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 0), (1, 4)]
            .map(|(a, b)| edge(t, a, b).unwrap());
        edges.extend_from_slice(&e[..6]);
        let surface = t.shapes_mut().unwrap().add_surface(def.clone()).unwrap();
        let sc = Some(SurfaceCarrier { surface, sense: Sense::Forward });
        let c = |i: usize, fwd: bool| CoedgeSpec {
            edge: e[i],
            sense: if fwd { Sense::Forward } else { Sense::Reversed },
        };
        let a = t.make_face(&[&[c(0, true), c(6, true), c(4, true), c(5, true)]], sc)?;
        let b = t.make_face(&[&[c(1, true), c(2, true), c(3, true), c(6, false)]], sc)?;
        Ok([a, b])
    })
    .unwrap();
    let mut ov = Overlay::new(Arc::new(t));
    let vf = ov.composite_faces(&faces).unwrap();
    (ov, vf, edges, def)
}

#[test]
fn a_curved_non_periodic_composite_projects_onto_its_surface() {
    let (ov, vf, edges, def) = curved_composite();
    assert!(ov.composite_surface(vf).is_some());
    let surf = def.as_surface().expect("parametric");
    for name in ["pave", "trimesh", "tridelaunay", "triadvance"] {
        let m = mesh_face(&ov, vf, name, counts(&edges, 4));
        let mut interior = 0;
        for v in 0..m.vertex_count() {
            let p = m.point(v);
            let d = surf.project(p, None).expect("project").distance;
            assert!(d < 1e-9, "{name}: node {p:?} is {d:e} off the surface");
            let on_edge = p.y.abs() < 1e-12 || (p.y - 1.0).abs() < 1e-12 || p.x.abs() < 1e-12 || (p.x - 2.0).abs() < 1e-12;
            if !on_edge {
                interior += 1;
            }
        }
        assert!(interior > 0, "{name}: no interior node");
        assert!(m.cell_count() > 0);
        eprintln!(
            "curved composite {name}: cells {} nodes {} interior {interior} min scaled Jacobian {:.4}",
            m.cell_count(),
            m.vertex_count(),
            min_scaled_jacobian(&m)
        );
    }
}
