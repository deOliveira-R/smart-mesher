//! W3b-SWEEP: pair prefilter before `discover_columns` (identical results, fewer walks) and the
//! reason text.

mod common;

use common::{plan_only, registry, registry_with_sources};
use sm_core::EntityId;
use sm_model_api::Model;
use sm_scheme_api::{Firm, Firmness, IntervalRequest, MeshIntent, SchemeChoice, Target};
use sm_scheme_sweep::{counters, discover_columns};
use sm_topo_api::TopologyView;

fn faces_edges(t: &dyn TopologyView, f: EntityId) -> Vec<EntityId> {
    t.face_edges(f).to_vec()
}

fn faces_of(t: &dyn TopologyView, v: EntityId) -> Vec<EntityId> {
    t.volume_faces(v).to_vec()
}

/// Brute force: lowest-id valid pair by calling `discover_columns` on every pair, and the
/// number of valid pairs.
fn brute(t: &dyn TopologyView, v: EntityId) -> (Option<(EntityId, EntityId)>, usize) {
    let faces = faces_of(t, v);
    let one = [(v, faces.clone())];
    let (mut first, mut k) = (None, 0);
    for (i, &s) in faces.iter().enumerate() {
        for &g in &faces[i + 1..] {
            if discover_columns(t, &one, s, g).is_ok() {
                k += 1;
                first.get_or_insert((s, g));
            }
        }
    }
    (first, k)
}

#[test]
fn prefilter_cuts_discover_calls_on_a_20_face_block() {
    let fx = sm_testkit::topo::regular_prism(18, 1.0, 2.0).expect("prism");
    let topo = &fx.topo.complex;
    let n_faces = faces_of(topo, fx.volume).len();
    assert_eq!(n_faces, 20);
    let pairs = (n_faces * (n_faces - 1) / 2) as u64;

    counters::reset_discover_calls();
    let (first, _k) = brute(topo, fx.volume);
    assert_eq!(counters::discover_calls(), pairs);
    assert!(first.is_some());

    let mut intent = MeshIntent::default();
    intent.set_scheme(fx.volume, SchemeChoice::new("sweep", Firmness::Hard));
    intent.set_scheme(fx.source, SchemeChoice::new("onepolygon", Firmness::Hard));
    for e in faces_edges(topo, fx.source) {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Count(1)));
    }
    for e in &fx.columns {
        intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(2)));
    }
    counters::reset_discover_calls();
    let plan = plan_only(
        &registry_with_sources(),
        topo,
        &Model::new(),
        &[fx.volume],
        &intent,
    )
    .expect("plan");
    let calls = counters::discover_calls();
    assert!(
        calls > 0 && calls < pairs,
        "{calls} discover calls for {pairs} pairs"
    );
    let row = plan.target(Target::Volume(fx.volume)).expect("row");
    assert_eq!(row.scheme.as_str(), "sweep");
}

#[test]
fn reason_names_source_target_and_valid_pair_count() {
    let (topo, ids, model) = sm_scheme_api::stubs::brick_setup([1.0, 1.0, 1.0]);
    let (first, k) = brute(&topo, ids.volume);
    let (s, g) = first.expect("pair");
    assert_eq!(k, 3, "a box has three axis pairs");
    let plan = plan_only(
        &registry(),
        &topo,
        &model,
        &[ids.volume],
        &MeshIntent::default(),
    )
    .expect("plan");
    let row = plan.target(Target::Volume(ids.volume)).expect("row");
    assert_eq!(row.scheme.as_str(), "sweep");
    assert_eq!(
        row.reason,
        format!("1-to-1 sweepable from {s} to {g} (3 valid pairs)")
    );
}
