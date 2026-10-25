//! Regression: the standard distribution is buildable over the reference `CellComplex` core, and
//! `standard_with` pins author and clock (integration milestone row 8 and the sm-cli seam).
use serde_json::json;
use sm_server::{ApplyOptions, FixedClock, Workspace, reference_standard, standard, standard_with};

const T: &str = "2026-10-24T00:00:00Z";

fn run(mut ws: Workspace) -> (String, Vec<(String, String)>) {
    let o = ApplyOptions::default();
    let b = ws.apply_json("model.create_brick", json!({"x":4,"y":4,"z":4}), &o).expect("brick");
    let vol = b.delta.outputs["volume"][0].to_string();
    for (op, p) in [
        ("mesh.size", json!({"targets":[vol],"size":1.0})),
        ("mesh.scheme", json!({"targets":[vol],"scheme":"map"})),
        ("mesh.run", json!({"targets":[vol]})),
    ] {
        ws.apply_json(op, p, &o).expect("op");
    }
    let layers = ws.state().layer_hashes().into_iter().map(|(k, v)| (k, v.to_string())).collect();
    (ws.state_hash().to_string(), layers)
}

#[test]
fn reference_core_matches_topostore_hashes() {
    let pin = |b: sm_engine::WorkspaceBuilder| b.author("t").clock(FixedClock(T.into())).memory().expect("ws");
    let a = run(pin(standard()));
    let b = run(pin(reference_standard()));
    assert_eq!(a.0, b.0, "state hash");
    assert_eq!(a.1, b.1, "layer hashes");
}

#[test]
fn standard_with_pins_author_and_clock() {
    let brick = |b: sm_engine::WorkspaceBuilder| {
        let mut ws = b.memory().expect("ws");
        ws.apply_json("model.create_brick", json!({"x":1,"y":1,"z":1}), &ApplyOptions::default()).expect("brick").node
    };
    let pinned = brick(standard_with(Some("t".into()), Some(T.into())));
    assert_eq!(pinned, brick(standard_with(Some("t".into()), Some(T.into()))), "reproducible");
    assert_eq!(pinned, brick(standard().author("t").clock(FixedClock(T.into()))), "same as by hand");
}
