//! Curvature sizing on faces bounded only by constant-u curves (the two halves of a plane-split
//! sphere): both halves must get the chord-height refinement, so their meshed volumes agree.

use std::f64::consts::PI;

use serde_json::{Value, json};
use sm_protocol::{Endpoint, Request};
use sm_server::{Server, standard_with};

fn run(cut: &str) -> Value {
    let mut s = Server::new(standard_with(Some("tester".into()), Some("2026-11-15T00:00:00Z".into()))).expect("server");
    let text = format!(
        "let s = model.create_sphere(radius: 1.0)\n\
         let c = model.webcut_plane(targets: s.body, plane: {cut})\n\
         mesh.default_size(function: \"curvature\", params: {{tol: 0.01, h_max: 0.5}})\n\
         mesh.auto(strategy: \"tet\")\n\
         let m = mesh.run(targets: all volumes)\n"
    );
    s.call(Request::ScriptRun { text, path: None, includes: Default::default() }).expect("script");
    s.call(Request::Query { name: "mesh.info".into(), input: json!({"mesh": "Mesh#1"}), at: None }).expect("info")
}

/// Hemispheres cut by a plane through the centre, whatever the normal: equal inscribed-polyhedron volumes.
#[test]
fn both_hemispheres_are_refined_alike() {
    for normal in ["[0.0, 1.0, 0.0]", "[0.0, 0.0, 1.0]"] {
        let info = run(&format!("{{origin: [0.0, 0.0, 0.0], normal: {normal}}}"));
        let mut v: Vec<f64> = info["volume_by_owner"].as_object().expect("owners").values().map(|x| x.as_f64().expect("f64")).collect();
        v.sort_by(f64::total_cmp);
        assert_eq!(v.len(), 2, "{info}");
        let half = 2.0 * PI / 3.0;
        // Inscribed mesh: below the exact half by a few percent at tol 0.01 (chord height).
        for x in &v {
            assert!(*x < half && *x > 0.96 * half, "{normal}: {x} vs {half}: {info}");
        }
        assert!((v[1] - v[0]) / v[1] < 1e-3, "{normal}: halves differ: {v:?}");
    }
}
