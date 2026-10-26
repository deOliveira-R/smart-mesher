//! Applicability of `pave` (plan W3-PAVE item 9): the reasons it refuses a face, and the one
//! score it gives a face it accepts.

mod common;

use common::*;
use sm_core::{EntityId, Point3};
use sm_geom_api::{CurveDef, Interval, Line};
use sm_scheme_api::geom;
use sm_scheme_api::stubs::brick_setup;
use sm_scheme_api::{
    Applicability, Caps, MeshInput, MeshIntent, Params, PlanCtx, SchemeDim, Target, WholeFaces,
};
use sm_topo_api::fixtures::with_op;
use sm_topo_api::{
    CellComplex, CoedgeKey, CoedgeSpec, CurveCarrier, OpId, Sense, TopologyEdit,
};

fn applicability(topo: &CellComplex, face: EntityId, params: Params) -> Applicability {
    let reg = registry();
    let model = sm_model_api::Model::new();
    let sel = [face];
    let input = MeshInput::new(topo, &model, &WholeFaces, &sel);
    let intent = MeshIntent::default();
    let cx = PlanCtx::new(&input, &intent);
    reg.get(SchemeDim::Surface, "pave").expect("pave").applicability(&cx, Target::Surface(face), &params)
}

fn refusal(a: Applicability) -> String {
    match a {
        Applicability::NotApplicable { reason, .. } => reason,
        other => panic!("expected NotApplicable, got {other:?}"),
    }
}

#[test]
fn a_plain_face_scores_forty() {
    let (topo, ids, _) = brick_setup([1.0, 1.0, 1.0]);
    match applicability(&topo, ids.faces[4], Params::default()) {
        Applicability::Applicable { score, reason, .. } => assert_eq!((score, reason.as_str()), (40, "unstructured quad")),
        other => panic!("{other:?}"),
    }
    // The known parameters are accepted.
    let p = Params(serde_json::json!({"gradation": 0.5, "smooth_iters": 5}));
    assert!(matches!(applicability(&topo, ids.faces[4], p), Applicability::Applicable { .. }));
}

#[test]
fn unknown_and_out_of_range_parameters_are_refused_with_a_reason() {
    let (topo, ids, _) = brick_setup([1.0, 1.0, 1.0]);
    let r = refusal(applicability(&topo, ids.faces[4], Params(serde_json::json!({"bogus": 1}))));
    assert!(r.contains("bogus") || r.contains("unknown field"), "{r}");
    let r = refusal(applicability(&topo, ids.faces[4], Params(serde_json::json!({"gradation": -1.0}))));
    assert!(r.contains("gradation"), "{r}");
    let r = refusal(applicability(&topo, ids.faces[4], Params(serde_json::json!({"smooth_iters": 1_000_000}))));
    assert!(r.contains("smooth_iters"), "{r}");
}

#[test]
fn a_face_with_an_embedded_edge_is_refused() {
    let (mut topo, ids, _) = brick_setup([1.0, 1.0, 1.0]);
    let face = ids.faces[4];
    let lv: Vec<EntityId> = sm_topo_api::TopologyViewExt::loop_vertices(&topo, face, 0).collect();
    let (a, c) = (lv[0], lv[2]);
    let (pa, pc) = (geom::vertex_point(&topo, a).expect("a"), geom::vertex_point(&topo, c).expect("c"));
    with_op(&mut topo, OpId(2), |t| {
        let line = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(pa, pc - pa)?))?;
        let e = t.make_edge(a, c, Some(CurveCarrier { curve: line, range: Interval::new(0.0, (pc - pa).norm()), sense: Sense::Forward }))?;
        t.embed(face, e)?;
        Ok(())
    })
    .expect("embed");
    let r = refusal(applicability(&topo, face, Params::default()));
    assert!(r.contains("embedded"), "{r}");
}

#[test]
fn a_sphere_face_with_poles_is_refused() {
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), |t| sm_topo_api::fixtures::sphere(t, 1.0)).expect("sphere");
    let r = refusal(applicability(&c, ids.face, Params::default()));
    assert!(r.contains("pole") || r.contains("degenerate"), "{r}");
}

#[test]
fn a_face_without_a_surface_carrier_is_refused() {
    let mut c = CellComplex::new();
    let face = with_op(&mut c, OpId(1), |t| {
        let pts = [Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)];
        let vs: Vec<EntityId> = pts.iter().map(|p| t.make_vertex(*p)).collect::<Result<_, _>>()?;
        let mut loop_: Vec<CoedgeSpec> = Vec::new();
        for i in 0..3 {
            let (p, q) = (pts[i], pts[(i + 1) % 3]);
            let line = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(p, (q - p) * (1.0 / (q - p).norm()))?))?;
            let e = t.make_edge(vs[i], vs[(i + 1) % 3], Some(CurveCarrier { curve: line, range: Interval::new(0.0, (q - p).norm()), sense: Sense::Forward }))?;
            loop_.push(CoedgeSpec { edge: e, sense: Sense::Forward });
        }
        Ok(t.make_face(&[&loop_], None)?)
    })
    .expect("face");
    let r = refusal(applicability(&c, face, Params::default()));
    assert!(r.contains("surface carrier"), "{r}");
}

#[test]
fn a_periodic_face_whose_seam_cannot_be_unrolled_is_refused() {
    let (mut c, ids, _) = cylinder(1.0, 1.0);
    let lateral = ids.faces[2];
    // With its pcurves the lateral face is fine ...
    assert!(matches!(applicability(&c, lateral, Params::default()), Applicability::Applicable { .. }));
    // ... without them `loop_uvs` could not tell the seam branches apart.
    let keys: Vec<CoedgeKey> = geom::face_loops(&c, lateral).into_iter().flatten().collect();
    with_op(&mut c, OpId(2), |t| {
        for k in &keys {
            t.set_pcurve(*k, None)?;
        }
        Ok(())
    })
    .expect("drop pcurves");
    let r = refusal(applicability(&c, lateral, Params::default()));
    assert!(r.contains("periodic") && r.contains("pcurve"), "{r}");
    // The same planar cap stays applicable: planar surfaces project.
    assert!(matches!(applicability(&c, ids.faces[0], Params::default()), Applicability::Applicable { .. }));
}

#[test]
fn caps_are_quad_only_and_groups_are_not_supported() {
    let reg = registry();
    let s = reg.get(SchemeDim::Surface, "pave").expect("pave");
    let caps = s.caps(&Params::default());
    assert_eq!(caps, Caps::QUAD);
    assert!(!caps.contains(Caps::STRUCTURED) && !caps.contains(Caps::BLOCK_STRUCTURED));
    assert!(!s.supports_groups());
}
