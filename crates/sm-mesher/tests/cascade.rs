//! Command-time resolution of the firmness cascade (ruling P1): with
//! `RequestCascade::Firmness` the stored intent alone decides the plan, whatever the history.
#![allow(missing_docs)]

mod common;
use common::*;
use serde_json::json;
use sm_core::EntityId;
use sm_layer_api::ModelLayer;
use sm_mesher::{IntentLayer, MesherConfig};
use sm_op_api::{DerivedCache, Session};
use sm_scheme_api::{Firm, IntervalRequest, RequestCascade, stubs};

fn session_with(cascade: RequestCascade) -> Session {
    let cfg = MesherConfig::new(stubs::registry()).with_cascade(cascade);
    let mut s = Session::new(engine_cfg(cfg, DerivedCache::default()));
    run(&mut s, &brick_op([4.0; 3])).expect("brick");
    s
}

fn first_edge(s: &Session) -> EntityId {
    let m = s.state().layer::<ModelLayer>().expect("model");
    m.topo().face_edges(face(1))[0]
}

fn request(s: &Session, id: EntityId) -> Option<Firm<IntervalRequest>> {
    s.state()
        .layer::<IntentLayer>()
        .and_then(|l| l.intent().entities.get(&id).and_then(|e| e.intervals))
}

fn intervals(s: &mut Session, t: EntityId, count: u32, firmness: &str) {
    run_json(
        s,
        "mesh.intervals",
        json!({"targets": [t.to_string()], "count": count, "firmness": firmness}),
    )
    .expect("intervals");
}

#[test]
fn a_later_equally_firm_face_request_is_written_on_the_edge_it_overrides() {
    let mut s = session_with(RequestCascade::Firmness);
    let e = first_edge(&s);
    intervals(&mut s, e, 3, "soft");
    intervals(&mut s, face(1), 5, "soft");
    assert_eq!(request(&s, e), Some(Firm::soft(IntervalRequest::Count(5))));
    // The other order: the edge is the latest command and keeps its own count.
    let mut s = session_with(RequestCascade::Firmness);
    intervals(&mut s, face(1), 5, "soft");
    intervals(&mut s, e, 3, "soft");
    assert_eq!(request(&s, e), Some(Firm::soft(IntervalRequest::Count(3))));
    assert_eq!(
        request(&s, face(1)),
        Some(Firm::soft(IntervalRequest::Count(5)))
    );
}

#[test]
fn a_firmer_request_is_never_overridden_and_nothing_is_written_for_it() {
    let mut s = session_with(RequestCascade::Firmness);
    let e = first_edge(&s);
    intervals(&mut s, e, 3, "hard");
    intervals(&mut s, face(1), 5, "soft");
    assert_eq!(request(&s, e), Some(Firm::hard(IntervalRequest::Count(3))));
    // A weaker edge request later does not touch a firmer face request either.
    let mut s = session_with(RequestCascade::Firmness);
    intervals(&mut s, face(1), 5, "hard");
    intervals(&mut s, e, 3, "soft");
    assert_eq!(
        request(&s, face(1)),
        Some(Firm::hard(IntervalRequest::Count(5)))
    );
}

#[test]
fn a_volume_request_overrides_an_equally_firm_face_on_its_edges() {
    let mut s = session_with(RequestCascade::Firmness);
    intervals(&mut s, face(1), 5, "soft");
    intervals(&mut s, vol(1), 7, "soft");
    let e = first_edge(&s);
    assert_eq!(request(&s, e), Some(Firm::soft(IntervalRequest::Count(7))));
}

#[test]
fn the_level_cascade_stores_requests_as_given() {
    let mut s = session_with(RequestCascade::Level);
    let e = first_edge(&s);
    intervals(&mut s, e, 3, "soft");
    intervals(&mut s, face(1), 5, "soft");
    assert_eq!(request(&s, e), Some(Firm::soft(IntervalRequest::Count(3))));
    assert_eq!(
        request(&s, face(1)),
        Some(Firm::soft(IntervalRequest::Count(5)))
    );
}

#[test]
fn the_same_request_on_neighbouring_volumes_writes_no_edge_rows() {
    let cfg = MesherConfig::new(stubs::registry()).with_cascade(RequestCascade::Firmness);
    let mut s = Session::new(engine_cfg(cfg, DerivedCache::default()));
    run(&mut s, &brick_op([1.0, 2.0, 3.0])).expect("brick");
    let (_, d) = run(&mut s, &Webcut { frac: 0.5 }).expect("cut");
    let vols = d.outputs.get("volumes").cloned().expect("volumes");
    assert_eq!(vols.len(), 2);
    for v in vols {
        run_json(
            &mut s,
            "mesh.size",
            json!({"targets": [v.to_string()], "size": 1.0}),
        )
        .expect("size");
    }
    let l = s.state().layer::<IntentLayer>().expect("intent");
    assert_eq!(l.intent().entities.len(), 2, "{:?}", l.intent().entities.keys());
}
