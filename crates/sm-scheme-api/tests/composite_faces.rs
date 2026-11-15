//! ACR-V2a: composite faces through `GenCtx::loop_uvs` / `geom::face_normal`, and the planner
//! mapping `PartitionFace` intent to `Target::Surface`.

use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{MeshRead, SerialAlloc};
use sm_geom_api::{Cylinder, CurveDef, Frame, Interval, Line, SurfaceDef};
use sm_scheme_api::stubs::{brick_setup, registry};
use sm_scheme_api::{
    BoundaryLoop, Driver, Firm, Firmness, GenCtx, IntervalRequest, MeshInput, MeshIntent,
    PatchSet, ReferenceCornerPicker, Sequential, ReferenceIntervalSolver, SchemeChoice, SchemeError, Target,
    WholeFaces, geom,
};
use sm_topo_api::fixtures::{BrickIds, with_op};
use sm_topo_api::{
    CellComplex, Carrier, CoedgeSpec, CurveCarrier, FaceCut, InnerLoopAssign, OpId, Overlay,
    Sense, SurfaceCarrier, TopologyEdit, TopologyView,
};
use std::sync::Arc;

fn diagonal() -> CurveDef {
    CurveDef::Line(Line::new(Point3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 1.0, 0.0)).unwrap())
}

fn all_counts(ids: &BrickIds, n: u32) -> MeshIntent {
    let mut intent = MeshIntent::default();
    for e in ids.edges {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Count(n)));
    }
    intent
}

/// Loop uvs of face `f` of `topo` (a surface-only selection of `[f]`) and its normal at `p`.
fn uvs_and_normal(
    topo: &dyn TopologyView,
    ids: &BrickIds,
    f: EntityId,
    p: Point3,
) -> (Vec<[f64; 2]>, Vector3) {
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let model = sm_model_api::Model::new();
    let sel = [f];
    let input = MeshInput::new(topo, &model, &WholeFaces, &sel);
    let out = driver
        .mesh(
            &input,
            &all_counts(ids, 3),
            &Sequential,
            ReferenceBuilder::surface(),
            &mut SerialAlloc::new(),
        )
        .expect("surface mesh");
    // One face's mesh: 3 x 3 quads.
    assert_eq!(out.finished.mesh.cell_count(), 9);
    let cx = GenCtx::new(&input, &out.plan, &out.patches, None);
    let loops = cx.loops(f).expect("loops");
    assert_eq!(loops.len(), 1);
    let uvs = cx.loop_uvs(f, &loops[0]).expect("loop uvs");
    (uvs, geom::face_normal(topo, f, p).expect("normal"))
}

#[test]
fn planar_composite_meshes_like_the_host_face() {
    // Reference: the unsplit top face of a unit brick.
    let (host, ids, _) = brick_setup([1.0; 3]);
    let top = ids.faces[5];
    let p = Point3::new(0.3, 0.6, 1.0);
    let (want_uv, want_n) = uvs_and_normal(&host, &ids, top, p);
    assert_eq!(want_uv.len(), 12);

    // The same brick with its top face split in two, then composited back.
    let (mut t, ids2, _) = brick_setup([1.0; 3]);
    let halves = with_op(&mut t, OpId(2), |t| {
        let curve = t.shapes_mut().unwrap().add_curve(diagonal()).unwrap();
        let chord = t.make_edge(
            ids2.vertices[4],
            ids2.vertices[7],
            Some(CurveCarrier {
                curve,
                range: Interval::new(0.0, 2f64.sqrt()),
                sense: Sense::Forward,
            }),
        )?;
        Ok(t.split_face(ids2.faces[5], FaceCut::Chord { edge: chord }, InnerLoopAssign::None)?)
    })
    .unwrap();
    let mut ov = Overlay::new(Arc::new(t));
    let vf = ov.composite_faces(&halves).unwrap();
    assert!(matches!(ov.carrier(vf), Carrier::Composite(_)));
    let (got_uv, got_n) = uvs_and_normal(&ov, &ids2, vf, p);

    assert_eq!(got_uv.len(), want_uv.len());
    for (g, w) in got_uv.iter().zip(&want_uv) {
        assert!(
            (g[0] - w[0]).abs() < 1e-12 && (g[1] - w[1]).abs() < 1e-12,
            "{g:?} vs {w:?}"
        );
    }
    assert!((got_n - want_n).norm() < 1e-12, "{got_n:?} vs {want_n:?}");
    assert!((got_n - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-12);
}

/// Two quads sharing an edge, both on a cylinder surface (topology only), as a composite.
fn cylinder_composite() -> (Overlay, EntityId) {
    let mut t = CellComplex::new();
    let faces = with_op(&mut t, OpId(1), |t| {
        let p = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0), (0.0, 1.0)];
        let v: Vec<EntityId> = p
            .iter()
            .map(|q| t.make_vertex(Point3::new(q.0, q.1, 0.0)).unwrap())
            .collect();
        let edge = |t: &mut CellComplex, a: usize, b: usize| {
            let (pa, pb) = (Point3::new(p[a].0, p[a].1, 0.0), Point3::new(p[b].0, p[b].1, 0.0));
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
        let cyl = SurfaceDef::Cylinder(Cylinder::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0).unwrap());
        let surface = t.shapes_mut().unwrap().add_surface(cyl).unwrap();
        let sc = Some(SurfaceCarrier { surface, sense: Sense::Forward });
        let f = |i: usize, fwd: bool| CoedgeSpec {
            edge: e[i],
            sense: if fwd { Sense::Forward } else { Sense::Reversed },
        };
        let a = t.make_face(&[&[f(0, true), f(6, true), f(4, true), f(5, true)]], sc)?;
        let b = t.make_face(&[&[f(1, true), f(2, true), f(3, true), f(6, false)]], sc)?;
        Ok([a, b])
    })
    .unwrap();
    let mut ov = Overlay::new(Arc::new(t));
    let vf = ov.composite_faces(&faces).unwrap();
    (ov, vf)
}

#[test]
fn periodic_composite_is_refused_not_guessed() {
    let (ov, vf) = cylinder_composite();
    assert_eq!(ov.composite_surface(vf), None);
    // face_normal: typed refusal, not the Newell fallback.
    match geom::face_normal(&ov, vf, Point3::new(1.0, 0.0, 0.5)) {
        Err(SchemeError::Input(m)) => assert!(m.contains("V2b"), "{m}"),
        other => panic!("expected a typed refusal, got {other:?}"),
    }
    // loop_uvs: None (no uv at all), through a GenCtx over the overlay.
    let (host, ids, model) = brick_setup([1.0; 3]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.faces[5]];
    let input = MeshInput::new(&host, &model, &WholeFaces, &sel);
    let plan = driver.plan(&input, &all_counts(&ids, 2)).unwrap();
    let ov_input = MeshInput::new(&ov, &model, &WholeFaces, &sel);
    let patches = PatchSet::default();
    let cx = GenCtx::new(&ov_input, &plan, &patches, None);
    let lp = BoundaryLoop { ring: 0, nodes: Vec::new(), uses: Vec::new() };
    assert_eq!(cx.loop_uvs(vf, &lp), None);
}

/// Plan a surface-only selection of the bottom and top faces of a brick, each a composite of
/// two host halves (`PartitionFace` ids; the bottom composite has the lower id), tied by an
/// explicit `copy` intent on the bottom composite. A face with the copy intent is never the
/// class root; without the intent the lowest target is. That is only seen if the planner treats
/// the intent on a composite as one on a `Surface` target.
fn copy_class_root(with_intent: bool) -> (Target, EntityId, EntityId) {
    let (mut t, ids, model) = brick_setup([1.0; 3]);
    let split = |t: &mut CellComplex, op: u64, f: usize, a: usize, b: usize, z: f64| {
        with_op(t, OpId(op), |t| {
            let line = Line::new(Point3::new(0.0, 0.0, z), Vector3::new(1.0, 1.0, 0.0)).unwrap();
            let curve = t.shapes_mut().unwrap().add_curve(CurveDef::Line(line)).unwrap();
            let chord = t.make_edge(
                ids.vertices[a],
                ids.vertices[b],
                Some(CurveCarrier {
                    curve,
                    range: Interval::new(0.0, 2f64.sqrt()),
                    sense: Sense::Forward,
                }),
            )?;
            Ok(t.split_face(ids.faces[f], FaceCut::Chord { edge: chord }, InnerLoopAssign::None)?)
        })
        .unwrap()
    };
    let top = split(&mut t, 2, 5, 4, 7, 1.0);
    let bottom = split(&mut t, 3, 4, 0, 3, 0.0);
    let mut ov = Overlay::new(Arc::new(t));
    let vb = ov.composite_faces(&bottom).unwrap();
    let vt = ov.composite_faces(&top).unwrap();
    assert!(vb < vt);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [vb, vt];
    let input = MeshInput::new(&ov, &model, &WholeFaces, &sel);
    let mut intent = all_counts(&ids, 2);
    if with_intent {
        intent.set_scheme(
            vb,
            SchemeChoice::with_params(
                "copy",
                serde_json::json!({
                    "source": vt,
                    "transform": Transform::translation(Vector3::new(0.0, 0.0, -1.0)),
                }),
                Firmness::Hard,
            ),
        );
    } else {
        intent.set_scheme(
            vt,
            SchemeChoice::with_params(
                "copy",
                serde_json::json!({
                    "source": vb,
                    "transform": Transform::translation(Vector3::new(0.0, 0.0, 1.0)),
                }),
                Firmness::Hard,
            ),
        );
    }
    let plan = driver.plan(&input, &intent).expect("plan");
    let (class, _) = plan.copy_of(Target::Surface(vb)).or_else(|| plan.copy_of(Target::Surface(vt))).expect("class");
    (class.root, vb, vt)
}

#[test]
fn copy_intent_on_a_composite_face_keeps_it_out_of_the_root() {
    // Control: the copy intent is on the higher id, so the lower one stays the root.
    let (root, vb, _) = copy_class_root(false);
    assert_eq!(root, Target::Surface(vb));
    // The intent on the lower id moves the root to the other composite.
    let (root, _, vt) = copy_class_root(true);
    assert_eq!(root, Target::Surface(vt));
}
