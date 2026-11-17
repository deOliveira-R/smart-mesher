//! W4-VTOPO-MESH end to end: a brick whose top is split by a webcut and re-joined by
//! `model.composite_faces` meshes exactly like the plain brick, on any rayon pool size.

use serde_json::{Value, json};
use sm_layer_api::ModelLayer;
use sm_protocol::{Endpoint, Request};
use sm_server::{Server, standard_with};
use sm_topo_api::{Dim, TopologyViewExt};

fn script(s: &mut Server, text: &str) -> Value {
    s.call(Request::ScriptRun { text: text.into(), path: None, includes: Default::default() })
        .expect("script")
}

/// Ids of the faces whose outline lies in the plane z = 0.5.
fn top_faces(s: &Server) -> Vec<String> {
    let st = s.workspace().state();
    let t = st.layer::<ModelLayer>().expect("model layer").topo();
    t.cells(Dim::Face)
        .iter()
        .filter(|&&f| {
            let pts = t.face_outline(f);
            !pts.is_empty() && pts.iter().all(|p| (p.z - 0.5).abs() < 1e-9)
        })
        .map(|f| format!("{f}"))
        .collect()
}

/// Mesh the split brick on a pool of `threads`; returns (mesh.info, state hash, top face count).
fn run(composite: bool, threads: usize) -> (Value, String, usize) {
    let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool");
    pool.install(|| {
        let mut s =
            Server::new(standard_with(Some("t".into()), Some("2026-11-15T00:00:00Z".into())))
                .expect("server");
        script(
            &mut s,
            "let b = model.create_brick(x: 1.0, y: 1.0, z: 1.0)\n\
             let c = model.webcut_plane(targets: b.body, plane: {origin: [0.1, 0.0, 0.0], normal: [1.0, 0.0, 0.0]})\n",
        );
        script(&mut s, "model.unite(bodies: all bodies, regularize: false)\n");
        let tops = top_faces(&s);
        if composite {
            assert_eq!(tops.len(), 2, "two top halves before compositing: {tops:?}");
            let list = tops.iter().map(|f| format!("\"{f}\"")).collect::<Vec<_>>().join(", ");
            script(&mut s, &format!("model.composite_faces(faces: [{list}])\n"));
        }
        script(
            &mut s,
            "mesh.default_size(size: 0.25)\nmesh.auto()\nlet m = mesh.run(targets: all volumes)\n",
        );
        let info = s
            .call(Request::Query {
                name: "mesh.info".into(),
                input: json!({"mesh": "Mesh#1"}),
                at: None,
            })
            .expect("mesh.info");
        (info, format!("{:?}", s.workspace().state().hash()), tops.len())
    })
}

#[test]
fn composite_top_meshes_like_plain_brick_and_is_thread_independent() {
    let (plain, _, n_plain) = run(false, 1);
    let (comp1, h1, n_comp) = run(true, 1);
    let (comp4, h4, _) = run(true, 4);
    assert_eq!((n_plain, n_comp), (2, 2));
    // 4 x 4 x 4 hexes, 5^3 vertices (hand count: size 0.25 on a unit cube).
    for run in [&plain, &comp1] {
        assert_eq!(run["cells"].as_u64(), Some(64), "{run}");
        assert_eq!(run["vertices"].as_u64(), Some(125), "{run}");
    }
    for k in ["cells", "vertices", "faces", "cell_shapes", "volume"] {
        assert_eq!(comp1[k], plain[k], "composite top changes {k}");
    }
    assert_eq!(comp1["volume"].as_f64(), Some(1.0));
    assert_eq!(comp1, comp4);
    assert_eq!(h1, h4, "state hash differs between 1 and 4 threads");
}
