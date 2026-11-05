//! W3b-SURFPLAN: surface-only `mesh.run` (faces) and the `mesh.plan` explain query.

mod common;

use common::*;
use proptest::prelude::*;
use serde_json::{Value, json};
use sm_core::EntityKind;
use sm_layer_api::MeshLayer;
use sm_mesh_api::{CellIx, CellShape, MeshDim, MeshRead, cell_volume, validate};
use sm_op_api::{DerivedCache, QuietControl, Session};
use sm_scheme_api::stubs;

fn err_text<T>(r: Result<T, sm_op_api::TimelineError>) -> String {
    match r {
        Ok(_) => panic!("expected an error"),
        Err(e) => format!("{e:?} {e}"),
    }
}

/// A session over a brick `size` with size 1 requested on its volume.
fn brick_s(size: [f64; 3]) -> Session {
    let mut s = session();
    run(&mut s, &brick_op(size)).expect("brick");
    run_json(&mut s, "mesh.size", json!({"targets": FACES, "size": 1.0})).expect("size");
    s
}

/// A face run sees only requests on faces and edges (see `mesh.volume_request_ignored`).
const FACES: [&str; 6] = [
    "TopoFace#1",
    "TopoFace#2",
    "TopoFace#3",
    "TopoFace#4",
    "TopoFace#5",
    "TopoFace#6",
];

fn run_faces(s: &mut Session, faces: &[&str]) -> Result<Value, sm_op_api::TimelineError> {
    let (_, d) = run_json(s, "mesh.run", json!({ "targets": faces }))?;
    Ok(json!(d.outputs["mesh"][0].to_string()))
}

fn query(s: &Session, name: &str, params: Value) -> Result<Value, sm_op_api::RegistryError> {
    let q = QuietControl::new();
    s.engine().run_query(s.state(), name, &params, &q.control())
}

fn plan(s: &Session, params: Value) -> Value {
    query(s, "mesh.plan", params).expect("mesh.plan")
}

fn first_mesh(s: &Session) -> (sm_core::EntityId, sm_layer_api::MeshEntry) {
    let layer = s.state().layer::<MeshLayer>().expect("layer");
    let (id, e) = layer.meshes().next().expect("a mesh");
    (id, e.clone())
}

fn area(m: &dyn MeshRead) -> f64 {
    (0..m.cell_count())
        .map(|c| cell_volume(m, CellIx::new(c)))
        .sum()
}

/// Everything a refused op must leave alone.
fn fingerprint(s: &Session) -> String {
    let st = s.state();
    let kinds = [
        EntityKind::Mesh,
        EntityKind::MeshVertex,
        EntityKind::MeshEdge,
        EntityKind::MeshFace,
        EntityKind::MeshCell,
    ];
    let next: Vec<_> = kinds.iter().map(|k| st.peek_next(*k)).collect();
    format!(
        "{:?} {} {next:?} {:?}",
        st.hash(),
        st.entity_count(),
        st.layer_hashes()
    )
}

// ------------------------------------------------------------------ acceptance 1 to 3

#[test]
fn one_face_of_a_brick_is_a_quad_surface() {
    let mut s = brick_s([2.0, 3.0, 4.0]);
    let before = s.state().count_of(EntityKind::MeshCell);
    run_faces(&mut s, &["TopoFace#5"]).expect("run");
    let (mid, e) = first_mesh(&s);
    let m = e.mesh();
    assert_eq!(m.dim(), MeshDim::Two);
    assert_eq!(e.selection, vec![face(5)]);
    assert_eq!((m.cell_count(), m.vertex_count()), (6, 12));
    assert!((area(m) - 6.0).abs() < 1e-12);
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    let boundary = (0..m.face_count())
        .filter(|&f| m.face_kind(sm_mesh_api::FaceIx::new(f)) == sm_mesh_api::FaceKind::Boundary)
        .count();
    assert_eq!(boundary, 10, "boundary facets 2 * (2 + 3)");
    assert!((0..6).all(|c| m.cell_shape(CellIx::new(c)) == CellShape::Quad));
    assert_eq!(m.classified_on(face(5)).map(|c| c.cells.len()), Some(6));
    let st = s.state();
    assert_eq!(st.count_of(EntityKind::Mesh), 1);
    assert_eq!(st.count_of(EntityKind::MeshCell), before + 6);
    let info = query(&s, "mesh.info", json!({"mesh": mid.to_string()})).expect("info");
    assert_eq!(info["cells"], 6);
    assert_eq!(info["cell_shapes"], json!({"Quad": 6}));
    assert!((info["volume"].as_f64().expect("f64") - 6.0).abs() < 1e-12);
    assert_eq!(info["selection"], json!(["TopoFace#5"]));
}

#[test]
fn all_six_faces_make_a_closed_surface() {
    let mut s = brick_s([2.0, 3.0, 4.0]);
    run_faces(
        &mut s,
        &[
            "TopoFace#1",
            "TopoFace#2",
            "TopoFace#3",
            "TopoFace#4",
            "TopoFace#5",
            "TopoFace#6",
        ],
    )
    .expect("run");
    let (_, e) = first_mesh(&s);
    let m = e.mesh();
    assert_eq!(m.cell_count(), 2 * (6 + 8 + 12));
    assert!((area(m) - 52.0).abs() < 1e-12);
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(
        m.vertex_count() as i64 - m.face_count() as i64 + m.cell_count() as i64,
        2
    );
    // Closed: every facet has two cells, and the normals agree across shared edges (validate
    // reports an inconsistent orientation).
    assert!(
        (0..m.face_count())
            .all(|f| m.face_kind(sm_mesh_api::FaceIx::new(f)) == sm_mesh_api::FaceKind::Interior)
    );
}

#[test]
fn a_tri_scheme_meshes_a_face_with_triangles() {
    let mut s = brick_s([2.0, 3.0, 4.0]);
    run_json(
        &mut s,
        "mesh.scheme",
        json!({"targets": ["TopoFace#5"], "scheme": "tri"}),
    )
    .expect("scheme");
    let (_, d) = run_json(&mut s, "mesh.run", json!({"targets": ["TopoFace#5"]})).expect("run");
    let (mid, e) = first_mesh(&s);
    assert_eq!(d.outputs["mesh"], vec![mid]);
    let m = e.mesh();
    assert!(m.cell_count() > 0);
    assert!((0..m.cell_count()).all(|c| m.cell_shape(CellIx::new(c)) == CellShape::Tri));
    assert!((area(m) - 6.0).abs() < 1e-12);
    let info = query(&s, "mesh.info", json!({"mesh": mid.to_string()})).expect("info");
    assert_eq!(
        info["cell_shapes"],
        json!({"Tri": m.cell_count()}),
        "{info}"
    );
}

// ------------------------------------------------------------------ acceptance 4: guards

#[test]
fn guards_refuse_atomically_with_the_documented_text() {
    // Mixed targets.
    let mut s = brick_s([2.0, 3.0, 4.0]);
    let fp = fingerprint(&s);
    let e = err_text(run_json(
        &mut s,
        "mesh.run",
        json!({"targets": ["TopoVolume#1", "TopoFace#5"]}),
    ));
    assert!(
        e.contains("mesh.run takes volumes or faces, not both; run it twice"),
        "{e}"
    );
    assert_eq!(fingerprint(&s), fp);

    // A face meshed twice.
    run_faces(&mut s, &["TopoFace#5"]).expect("run");
    let fp = fingerprint(&s);
    let e = err_text(run_faces(&mut s, &["TopoFace#5"]));
    assert!(
        e.contains("TopoFace#5 is already meshed (Mesh#1); mesh.delete first"),
        "{e}"
    );
    assert_eq!(fingerprint(&s), fp);

    // A volume with a meshed face.
    let e = err_text(run_json(
        &mut s,
        "mesh.run",
        json!({"targets": ["TopoVolume#1"]}),
    ));
    assert!(
        e.contains("TopoFace#5 is in Mesh#1; mesh.delete first"),
        "{e}"
    );
    assert_eq!(fingerprint(&s), fp);

    // An edge neighbour meshed in another entry (Q13 text, verbatim).
    let e = err_text(run_faces(&mut s, &["TopoFace#1"]));
    assert!(
        e.contains("mesh them together in one mesh.run, or mesh.delete Mesh#1 first"),
        "{e}"
    );
    assert!(e.contains("TopoFace#1 shares TopoEdge#"), "{e}");
    assert_eq!(fingerprint(&s), fp);
    // Meshed together they are fine (a neighbour in the same run is no conflict).
    let mut s2 = brick_s([2.0, 3.0, 4.0]);
    run_faces(&mut s2, &["TopoFace#1", "TopoFace#5"]).expect("together");
    assert_eq!(first_mesh(&s2).1.selection, vec![face(1), face(5)]);

    // A face of a meshed volume.
    let mut s3 = brick_s([2.0, 3.0, 4.0]);
    run_json(&mut s3, "mesh.run", json!({"targets": ["TopoVolume#1"]})).expect("volume");
    let fp = fingerprint(&s3);
    let e = err_text(run_faces(&mut s3, &["TopoFace#5"]));
    assert!(
        e.contains("TopoFace#5 bounds TopoVolume#1, already meshed (Mesh#1)"),
        "{e}"
    );
    assert_eq!(fingerprint(&s3), fp);
}

#[test]
fn a_volume_run_is_unchanged_by_the_face_guards() {
    let mut s = brick_session([4.0; 3], 1.0);
    mesh_run(&mut s).expect("run");
    assert_eq!(s.state().count_of(EntityKind::MeshCell), 64);
}

// ------------------------------------------------------------------ acceptance 5

#[test]
fn delete_stale_and_hard_intervals_work_on_face_meshes() {
    let mut s = brick_s([2.0, 3.0, 4.0]);
    let base = s.state().entity_count();
    run_faces(&mut s, &["TopoFace#5"]).expect("run");
    assert!(s.state().entity_count() > base);
    run_json(&mut s, "mesh.delete", json!({"targets": ["TopoFace#5"]})).expect("delete");
    assert_eq!(s.state().entity_count(), base);
    assert_eq!(s.state().count_of(EntityKind::Mesh), 0);

    // A hard count on an edge of the face, and the opposite edge follows (map rows).
    let rep = plan(&s, json!({"targets": ["TopoFace#5"]}));
    let e0 = rep["intervals"][0]["edge"]
        .as_str()
        .expect("edge")
        .to_owned();
    run_json(
        &mut s,
        "mesh.intervals",
        json!({"targets": [e0], "count": 5}),
    )
    .expect("hard");
    run_faces(&mut s, &["TopoFace#5"]).expect("run");
    let (mid, e) = first_mesh(&s);
    let ran = plan(&s, json!({"mesh": mid.to_string()}));
    let rows = ran["intervals"].as_array().expect("rows");
    let counts: Vec<u64> = rows
        .iter()
        .map(|r| r["count"].as_u64().expect("n"))
        .collect();
    assert_eq!(counts.iter().filter(|&&c| c == 5).count(), 2, "{counts:?}");
    assert_eq!(
        rows.iter().find(|r| r["edge"] == json!(e0)).expect("e0")["count"],
        5
    );
    // Opposite edges carry equal counts, so the cells are the product of one count per pair.
    let mut sorted = counts.clone();
    sorted.sort_unstable();
    let cells = e.mesh().classified_on(face(5)).map(|c| c.cells.len());
    assert_eq!(cells, Some((sorted[0] * sorted[2]) as usize));

    // Deleting the volume marks the face mesh stale.
    run(&mut s, &DeleteVolume { volume: vol(1) }).expect("delete volume");
    let ran = plan(&s, json!({"mesh": mid.to_string()}));
    assert_eq!(ran["stale"], true);
}

// ------------------------------------------------------------------ acceptance 6, 7

#[test]
fn mesh_plan_ran_matches_will_run_and_explains_each_entity() {
    let mut s = brick_s([2.0, 3.0, 4.0]);
    let before = fingerprint(&s);
    let will = plan(&s, json!({"targets": ["TopoFace#5"]}));
    assert_eq!(fingerprint(&s), before, "a query changes nothing");
    assert_eq!(will["status"], "will_run");
    assert_eq!(will["dim"], 2);
    assert_eq!(will["blockers"], json!([]));
    assert_eq!(will["estimated_cells"], 6);
    assert_eq!(will["selection"], json!(["TopoFace#5"]));
    let face_row = will["entities"]
        .as_array()
        .expect("rows")
        .iter()
        .find(|r| r["target"] == "TopoFace#5")
        .expect("face row");
    assert_eq!(face_row["elements"], "QUAD4");
    assert_eq!(face_row["cells"], Value::Null, "{face_row}");

    run_faces(&mut s, &["TopoFace#5"]).expect("run");
    let (mid, _) = first_mesh(&s);
    let ran = plan(&s, json!({"mesh": mid.to_string()}));
    assert_eq!(ran["status"], "ran");
    assert_eq!(ran["plan_hash"], will["plan_hash"], "plan and replay agree");
    assert_eq!(ran["mesh"], json!(mid.to_string()));
    assert_eq!(ran["stale"], false);
    assert_eq!(ran["dim"], 2);
    assert_eq!(ran["estimated_cells"], Value::Null);
    let rows = ran["entities"].as_array().expect("rows");
    let f = rows
        .iter()
        .find(|r| r["target"] == "TopoFace#5")
        .expect("face");
    assert_eq!(f["scheme"], "map");
    assert!(f["how"] == "auto" || f["how"].as_str().is_some_and(|h| h.starts_with("intent#")));
    assert_eq!(f["elements"], json!({"Quad": 6}));
    assert_eq!(f["cells"], 6);
    assert_eq!(f["kind"], "surface");
    let edges: Vec<&Value> = rows.iter().filter(|r| r["kind"] == "curve").collect();
    assert_eq!(edges.len(), 4);
    assert!(edges.iter().all(|r| r["scheme"] == "equal"));
    // Ascending: curves before surfaces.
    assert!(rows.windows(2).all(|w| {
        let k = |r: &Value| match r["kind"].as_str().expect("kind") {
            "curve" => 0,
            "surface" => 1,
            _ => 2,
        };
        k(&w[0]) <= k(&w[1])
    }));
    let iv = ran["intervals"].as_array().expect("intervals");
    let mut counts: Vec<u64> = iv.iter().map(|r| r["count"].as_u64().expect("n")).collect();
    counts.sort_unstable();
    assert_eq!(counts, [2, 2, 3, 3]);
    assert!(iv.iter().all(|r| r["why"] == "sizing_goal"), "{iv:?}");
    assert!(iv.iter().all(|r| r["goal"].as_f64().is_some()));
    // `attempts: false` leaves out the alternatives.
    let lean = plan(&s, json!({"mesh": mid.to_string(), "attempts": false}));
    assert!(
        lean["entities"]
            .as_array()
            .expect("rows")
            .iter()
            .all(|r| r["alternatives"] == json!([]))
    );
}

#[test]
fn mesh_plan_reports_claimed_copy_and_failed_alternatives() {
    let mut reg = stubs::registry();
    reg.register(Box::new(stubs::AlwaysFail))
        .expect("always_fail");
    let mut s = Session::new(engine_with(reg, DerivedCache::default()));
    run(&mut s, &brick_op([2.0; 3])).expect("brick");
    run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoVolume#1"], "size": 1.0}),
    )
    .expect("size");
    run_json(
        &mut s,
        "mesh.scheme",
        json!({"targets": ["TopoVolume#1"], "scheme": "sweep",
               "params": {"source": ["TopoFace#5"], "target": ["TopoFace#6"]}}),
    )
    .expect("scheme");
    let will = plan(&s, json!({"targets": ["TopoVolume#1"]}));
    assert_eq!(will["status"], "will_run");
    assert_eq!(will["dim"], 3);
    let rows = will["entities"].as_array().expect("rows");
    let vol_row = rows
        .iter()
        .find(|r| r["target"] == "TopoVolume#1")
        .expect("volume");
    assert_eq!(vol_row["how"], "intent#0");
    assert_eq!(vol_row["scheme"], "sweep");
    assert!(
        rows.iter()
            .any(|r| !r["claimed_by"].is_null() || !r["copy_of"].is_null()),
        "the sweep target is a copy or claimed: {rows:?}"
    );
    // Run it: always_fail wins the surface auto score, fails, and the chain recovers.
    mesh_run(&mut s).expect("recovers");
    let (mid, _) = first_mesh(&s);
    let ran = plan(&s, json!({"mesh": mid.to_string()}));
    let alts: Vec<&Value> = ran["entities"]
        .as_array()
        .expect("rows")
        .iter()
        .flat_map(|r| r["alternatives"].as_array().expect("alts").iter())
        .collect();
    assert!(
        alts.iter()
            .any(|a| a["scheme"] == "always_fail" && a["phase"] == "Generate"),
        "{alts:?}"
    );
    let stored = first_mesh(&s).1.plan.expect("plan");
    assert_eq!(stored.rejections.len(), 1);
}

// ------------------------------------------------------------------ acceptance 8

#[test]
fn an_infeasible_intent_is_a_status_not_an_error() {
    let mut s = brick_session([10.0; 3], 1.0);
    run_json(
        &mut s,
        "mesh.intervals",
        json!({"targets": ["TopoEdge#1"], "count": 3}),
    )
    .expect("3");
    run_json(
        &mut s,
        "mesh.intervals",
        json!({"targets": ["TopoEdge#3"], "count": 5}),
    )
    .expect("5");
    let before = fingerprint(&s);
    let r = plan(&s, json!({"targets": ["TopoVolume#1"]}));
    assert_eq!(r["status"], "infeasible");
    let f = &r["failure"];
    assert!(
        f["message"]
            .as_str()
            .expect("msg")
            .to_lowercase()
            .contains("infeasible"),
        "{f}"
    );
    assert!(
        f["entities"].as_array().is_some_and(|e| !e.is_empty()),
        "{f}"
    );
    assert!(
        f["constraints"].as_array().is_some_and(|e| !e.is_empty()),
        "{f}"
    );
    assert_eq!(r["entities"], json!([]));
    assert_eq!(r["plan_hash"], Value::Null);
    assert_eq!(fingerprint(&s), before);
}

// ------------------------------------------------------------------ acceptance 9

/// `map` plus a preference of 4 intervals on every edge of the face: a second source on each
/// edge, next to the intent's.
struct MapPrefer;

impl sm_scheme_api::Scheme for MapPrefer {
    fn name(&self) -> &str {
        "mapprefer"
    }
    fn dim(&self) -> sm_scheme_api::SchemeDim {
        sm_scheme_api::SchemeDim::Surface
    }
    fn caps(&self, p: &sm_scheme_api::Params) -> sm_scheme_api::Caps {
        stubs::Map.caps(p)
    }
    fn applicability(
        &self,
        cx: &sm_scheme_api::PlanCtx<'_>,
        t: sm_scheme_api::Target,
        p: &sm_scheme_api::Params,
    ) -> sm_scheme_api::Applicability {
        stubs::Map.applicability(cx, t, p)
    }
    fn requirements(
        &self,
        cx: &sm_scheme_api::PlanCtx<'_>,
        t: sm_scheme_api::Target,
        p: &sm_scheme_api::Params,
        out: &mut sm_scheme_api::Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        stubs::Map.requirements(cx, t, p, out)
    }
    fn interval_constraints(
        &self,
        cx: &sm_scheme_api::IntervalCtx<'_>,
        t: sm_scheme_api::Target,
        p: &sm_scheme_api::Params,
        out: &mut sm_scheme_api::IntervalSink<'_>,
    ) -> sm_scheme_api::Result<()> {
        stubs::Map.interval_constraints(cx, t, p, out)?;
        if let sm_scheme_api::Target::Surface(f) = t
            && let Some(l) = cx.layout(f)
        {
            for side in &l.sides {
                for c in &side.coedges {
                    out.prefer(c.edge, 4.0);
                }
            }
        }
        Ok(())
    }
    fn generate(
        &self,
        cx: &sm_scheme_api::GenCtx<'_>,
        t: sm_scheme_api::Target,
        p: &sm_scheme_api::Params,
    ) -> Result<sm_scheme_api::Patch, sm_scheme_api::GenerateFailure> {
        stubs::Map.generate(cx, t, p)
    }
}

#[test]
fn two_sources_on_an_edge_are_both_listed_and_the_winner_is_marked() {
    let mut reg = stubs::registry();
    reg.register(Box::new(MapPrefer)).expect("mapprefer");
    let mut s = Session::new(engine_with(reg, DerivedCache::default()));
    run(&mut s, &brick_op([2.0, 3.0, 4.0])).expect("brick");
    run_json(&mut s, "mesh.size", json!({"targets": FACES, "size": 1.0})).expect("size");
    run_json(
        &mut s,
        "mesh.scheme",
        json!({"targets": ["TopoFace#5"], "scheme": "mapprefer"}),
    )
    .expect("scheme");
    let rep = plan(&s, json!({"targets": ["TopoFace#5"]}));
    let e0 = rep["intervals"][0]["edge"]
        .as_str()
        .expect("edge")
        .to_owned();
    run_json(
        &mut s,
        "mesh.intervals",
        json!({"targets": [e0], "count": 3}),
    )
    .expect("hard");
    let rep = plan(&s, json!({"targets": ["TopoFace#5"]}));
    let row = rep["intervals"]
        .as_array()
        .expect("rows")
        .iter()
        .find(|r| r["edge"] == json!(e0))
        .expect("row");
    assert_eq!(row["count"], 3);
    assert_eq!(row["why"], "hard_intent");
    let reqs = row["requests"].as_array().expect("requests");
    let hard = reqs.iter().find(|r| r["kind"] == "fixed").expect("fixed");
    assert_eq!(
        (hard["winner"].as_bool(), hard["agrees"].as_bool()),
        (Some(true), Some(true))
    );
    let pref = reqs
        .iter()
        .find(|r| {
            r["kind"] == "goal"
                && r["origin"]
                    .as_str()
                    .is_some_and(|o| o.starts_with("preference"))
        })
        .expect("the scheme's preference is listed");
    assert_eq!(
        (pref["winner"].as_bool(), pref["agrees"].as_bool()),
        (Some(false), Some(false))
    );
    assert_eq!(pref["value"], 4.0);

    // An edge nobody fixed: the preference decides (it replaces the sizing goal), named.
    let other = rep["intervals"]
        .as_array()
        .expect("rows")
        .iter()
        .find(|r| r["why"] == "scheme")
        .expect("a scheme-decided edge");
    assert_eq!(other["scheme"], "mapprefer");
    assert_eq!(other["target"], "TopoFace#5");
}

#[test]
fn a_count_solved_from_a_row_is_derived_and_lists_the_losing_goal() {
    let mut s = brick_s([2.0, 3.0, 4.0]);
    let rep = plan(&s, json!({"targets": ["TopoFace#5"]}));
    let e0 = rep["intervals"][0]["edge"]
        .as_str()
        .expect("edge")
        .to_owned();
    run_json(
        &mut s,
        "mesh.intervals",
        json!({"targets": [e0], "count": 7}),
    )
    .expect("hard");
    let rep = plan(&s, json!({"targets": ["TopoFace#5"]}));
    let rows = rep["intervals"].as_array().expect("rows");
    let derived: Vec<&Value> = rows.iter().filter(|r| r["why"] == "derived").collect();
    assert_eq!(
        derived.len(),
        1,
        "the opposite edge follows the row: {rows:?}"
    );
    let d = derived[0];
    assert_eq!(d["count"], 7);
    assert!(d["constraints"].as_array().is_some_and(|c| !c.is_empty()));
    let reqs = d["requests"].as_array().expect("reqs");
    assert!(
        reqs.iter()
            .any(|r| r["kind"] == "goal" && r["agrees"] == false && r["winner"] == false)
    );
    assert!(
        reqs.iter()
            .any(|r| r["kind"] == "linear" && r["winner"] == true)
    );
}

// ------------------------------------------------------------------ acceptance 10

#[test]
fn blockers_carry_the_texts_mesh_run_fails_with() {
    let mut s = brick_s([2.0, 3.0, 4.0]);
    run_faces(&mut s, &["TopoFace#5"]).expect("run");
    for target in ["TopoFace#5", "TopoFace#1", "TopoVolume#1"] {
        let r = plan(&s, json!({"targets": [target]}));
        assert_eq!(r["status"], "will_run", "{r}");
        let blockers = r["blockers"].as_array().expect("blockers");
        assert!(!blockers.is_empty(), "{target}");
        let e = err_text(run_json(&mut s, "mesh.run", json!({"targets": [target]})));
        assert!(
            e.contains(blockers[0].as_str().expect("text")),
            "{target}: {e} vs {blockers:?}"
        );
    }
    // Argument errors are errors.
    assert!(query(&s, "mesh.plan", json!({})).is_err());
    assert!(
        query(
            &s,
            "mesh.plan",
            json!({"mesh": "Mesh#1", "targets": ["TopoFace#5"]})
        )
        .is_err()
    );
    assert!(
        query(
            &s,
            "mesh.plan",
            json!({"targets": ["TopoFace#5", "TopoVolume#1"]})
        )
        .is_err()
    );
    assert!(query(&s, "mesh.plan", json!({"targets": ["TopoFace#99"]})).is_err());
    assert!(query(&s, "mesh.plan", json!({"mesh": "Mesh#9"})).is_err());
}

#[test]
fn a_mesh_without_a_plan_is_invalid() {
    use sm_layer_api::{MeshEntry, ModelLayer, insert_mesh};
    #[derive(Debug, serde::Serialize, serde::Deserialize, schemars::JsonSchema)]
    #[serde(deny_unknown_fields)]
    struct Imported {}
    impl sm_op_api::TypedOp for Imported {
        const NAME: &'static str = "test.imported";
        fn apply(&self, ctx: &mut sm_op_api::ApplyCtx<'_>) -> Result<(), sm_op_api::OpError> {
            let alloc0 = ctx.layer::<MeshLayer>()?.alloc().clone();
            let quanta = ctx.layer::<ModelLayer>()?.hash_quanta();
            let mut alloc = alloc0.clone();
            let fin = sm_mesh::generate::hex_block_bulk([1; 3], [1.0; 3], &mut alloc)
                .map_err(|e| sm_op_api::OpError::Invalid(e.to_string()))?;
            let entry = MeshEntry::new(Box::new(fin.mesh), vec![vol(1)], None, quanta);
            insert_mesh(ctx, entry, &alloc0, alloc, "mesh")
        }
    }
    let mut reg = sm_op_api::Registry::new();
    reg.install(&sm_layer_api::CoreLayers::new(cell_complex))
        .expect("core");
    reg.install(&sm_mesher::MeshPlugin).expect("mesher");
    reg.register::<CreateBrick>().expect("brick");
    reg.register::<Imported>().expect("imported");
    let eng = sm_op_api::Engine::new(
        std::sync::Arc::new(reg),
        sm_op_api::Services::new().with_versioned(
            "mesher",
            1,
            sm_mesher::MesherConfig::new(stubs::registry()),
        ),
        std::sync::Arc::new(DerivedCache::default()),
    )
    .expect("engine");
    let mut s = Session::new(eng);
    run(&mut s, &brick_op([1.0; 3])).expect("brick");
    run(&mut s, &Imported {}).expect("imported");
    let e = query(&s, "mesh.plan", json!({"mesh": "Mesh#1"})).expect_err("no plan");
    assert!(e.to_string().contains("mesh has no plan"), "{e}");
}

// ------------------------------------------------------------------ acceptance 11

fn flow(threads: usize) -> (sm_op_api::Hash256, sm_op_api::Hash256, String) {
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(threads)
        .build()
        .expect("pool");
    pool.install(|| {
        let mut s = brick_s([2.0, 3.0, 4.0]);
        let will = plan(&s, json!({"targets": ["TopoFace#5"]}));
        run_faces(&mut s, &["TopoFace#5"]).expect("run");
        run_json(
            &mut s,
            "mesh.scheme",
            json!({"targets": ["TopoFace#1"], "scheme": "tri"}),
        )
        .expect("scheme");
        // A second, disjoint-edge-free check: face 1 touches face 5, so mesh it after delete.
        run_json(&mut s, "mesh.delete", json!({"targets": ["TopoFace#5"]})).expect("delete");
        run_faces(&mut s, &["TopoFace#1"]).expect("tri face");
        run_faces(&mut s, &["TopoFace#5"]).expect_err("shares an edge with face 1");
        let exact = sm_op_api::Layer::exact_hash(s.state().layer::<MeshLayer>().expect("layer"));
        let ran = plan(&s, json!({"mesh": "Mesh#2"}));
        (
            s.state().hash(),
            exact,
            format!(
                "{}|{}",
                serde_json::to_string(&will).expect("json"),
                serde_json::to_string(&ran).expect("json")
            ),
        )
    })
}

#[test]
fn surface_runs_and_reports_are_bit_identical_across_threads() {
    sm_testkit::determinism::check(2, 4, flow).expect("deterministic");
}

#[test]
fn cold_replay_of_a_face_script_reproduces_the_state_hash() {
    let eng = engine_with(stubs::registry(), DerivedCache::disabled());
    let mut s = Session::new(eng);
    run(&mut s, &brick_op([2.0, 3.0, 4.0])).expect("brick");
    run_json(&mut s, "mesh.size", json!({"targets": FACES, "size": 1.0})).expect("size");
    run_json(
        &mut s,
        "mesh.scheme",
        json!({"targets": ["TopoFace#5"], "scheme": "tri"}),
    )
    .expect("scheme");
    let (h, _) = run_json(&mut s, "mesh.run", json!({"targets": ["TopoFace#5"]})).expect("run");
    let want = s.state().hash();
    let q = QuietControl::new();
    assert_eq!(
        s.state_at_from_root(h, &q.control())
            .expect("replay")
            .hash(),
        want
    );
}

// ------------------------------------------------------------------ acceptance 12

#[test]
fn schemas_list_faces() {
    let eng = engine();
    let reg = eng.registry();
    for name in ["mesh.run", "mesh.delete"] {
        let schema = serde_json::to_value(reg.schema(name).expect("schema")).expect("json");
        let kinds = &schema["properties"]["targets"]["x-entity-kind"];
        assert!(
            kinds
                .as_array()
                .expect("kinds")
                .contains(&json!("TopoFace")),
            "{name}: {kinds}"
        );
        assert!(
            kinds
                .as_array()
                .expect("kinds")
                .contains(&json!("TopoVolume")),
            "{name}: {kinds}"
        );
    }
    assert!(reg.query_names().contains(&"mesh.plan"));
    let cat = reg.query_catalog();
    let plan_q = cat
        .into_iter()
        .map(|c| c.expect("entry"))
        .find(|c| c.name == "mesh.plan")
        .expect("mesh.plan in the catalog");
    let schema = serde_json::to_value(plan_q.schema).expect("json");
    let kinds = &schema["properties"]["targets"]["x-entity-kind"];
    assert!(
        kinds
            .as_array()
            .expect("kinds")
            .contains(&json!("TopoFace")),
        "{kinds}"
    );
    assert!(schema["properties"]["mesh"].get("x-entity-kind").is_some());
}

proptest! {
    #![proptest_config(sm_testkit::strategies::config(24))]

    #[test]
    fn a_random_face_meshes_to_its_area(
        a in 1u32..=6, b in 1u32..=6, c in 1u32..=6, f in 1u64..=6,
    ) {
        let size = [f64::from(a), f64::from(b), f64::from(c)];
        let mut s = brick_s(size);
        let fid = format!("TopoFace#{f}");
        run_faces(&mut s, &[fid.as_str()]).expect("run");
        let (mid, e) = first_mesh(&s);
        let axis = ((f - 1) / 2) as usize;
        let dims: Vec<u32> = (0..3).filter(|&i| i != axis).map(|i| [a, b, c][i]).collect();
        let m = e.mesh();
        prop_assert_eq!(m.cell_count() as u32, dims[0] * dims[1]);
        prop_assert!((area(m) - f64::from(dims[0] * dims[1])).abs() < 1e-9);
        prop_assert!(validate(m).is_empty());
        let r = plan(&s, json!({"mesh": mid.to_string()}));
        let mut counts: Vec<u64> = r["intervals"].as_array().expect("rows").iter()
            .map(|x| x["count"].as_u64().expect("n")).collect();
        counts.sort_unstable();
        let mut want = vec![u64::from(dims[0]), u64::from(dims[0]), u64::from(dims[1]), u64::from(dims[1])];
        want.sort_unstable();
        prop_assert_eq!(counts, want);
    }
}

#[test]
fn a_size_on_the_volume_does_not_reach_a_face_run_and_says_so() {
    let mut s = session();
    run(&mut s, &brick_op([2.0, 3.0, 4.0])).expect("brick");
    run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoVolume#1"], "size": 1.0}),
    )
    .expect("size");
    let (_, d) = run_json(&mut s, "mesh.run", json!({"targets": ["TopoFace#5"]})).expect("run");
    assert_eq!(first_mesh(&s).1.mesh().cell_count(), 1);
    assert!(
        d.diagnostics
            .iter()
            .any(|x| x.code == "mesh.volume_request_ignored"),
        "{:?}",
        d.diagnostics
    );
    // mesh.plan shows it: every edge is the built-in default.
    let r = plan(&s, json!({"mesh": "Mesh#1"}));
    assert!(
        r["intervals"]
            .as_array()
            .expect("rows")
            .iter()
            .all(|x| x["why"] == "default")
    );
}
