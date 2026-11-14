//! W4-BOOL-HEAL-A T0: `model.regularize`, the exact join of co-surface faces and degree-2 edges.

mod common;

use std::f64::consts::PI;

use common::rig::*;
use common::*;
use sm_boolean::ops::{AxisPlane, PlaneAxis, PlaneSpec, Regularize, Unite, WebcutPlane};
use sm_core::EntityId;
use sm_layer_api::{MeshLayer, ModelLayer};
use sm_protocol::{Endpoint, Request};
use sm_server::{Server, standard_with};
use sm_topo_api::{Dim, TopologyViewExt};

const R: f64 = 0.75;
const H: f64 = 2.0;

/// A cylinder cut through its axis and fused back without regularizing.
fn halves(r: &mut Rig) -> EntityId {
    let c = r.cylinder(H, R, [0.0, 0.0, 0.0]);
    let plane = PlaneSpec::Axis(AxisPlane {
        axis: PlaneAxis::X,
        offset: 0.0,
    });
    let d = r
        .run(&WebcutPlane {
            targets: vec![c.body],
            plane,
            keep: None,
            merge: None,
            imprint: None,
        })
        .unwrap();
    let bodies: Vec<EntityId> = d.outputs["positive"]
        .iter()
        .chain(&d.outputs["negative"])
        .copied()
        .collect();
    assert_eq!(bodies.len(), 2);
    let u = r
        .run(&Unite {
            bodies,
            keep: None,
            regularize: Some(false),
            tol: None,
        })
        .unwrap();
    u.outputs["volumes"][0]
}

#[test]
fn regularize_restores_the_three_faces_of_a_cylinder() {
    let mut reference = Rig::cc();
    reference.cylinder(H, R, [0.0, 0.0, 0.0]);
    let want = counts(reference.topo());
    assert_eq!(want[2], 3);

    let mut r = Rig::cc();
    let v = halves(&mut r);
    let before = counts(r.topo());
    assert!(before[2] > 3, "the webcut halves are split: {before:?}");
    let d = r.run(&Regularize { volumes: vec![v] }).unwrap();
    let t = r.topo();
    assert_valid(t);
    assert_eq!(counts(t), want, "counts of create_cylinder");
    assert!(!d.outputs["faces"].is_empty() && !d.outputs["edges"].is_empty());
    let vol = volume_of(t, v);
    assert!(
        (vol - PI * R * R * H).abs() <= 1e-12 * PI * R * R * H,
        "volume {vol}"
    );
    // The lateral face keeps a seam (one edge used twice), so the pcurve synthesis still works.
    let lateral: Vec<EntityId> = t
        .cells(Dim::Face)
        .iter()
        .copied()
        .filter(|&f| !t.seam_edges(f).is_empty())
        .collect();
    assert_eq!(lateral.len(), 1, "one lateral face with a seam");
    let area = t.face_moments(lateral[0]).unwrap().area;
    assert!((area - 2.0 * PI * R * H).abs() <= 1e-12 * area);
}

#[test]
fn regularize_is_a_no_op_on_a_regular_body_and_deterministic() {
    let mut r = Rig::cc();
    let c = r.brick([1.0, 2.0, 3.0], [0.5, 1.0, 1.5]);
    // The hash stream minus the revision word (index 4, the op counter): nothing to join, nothing changes.
    let words = |t: &dyn sm_topo_api::TopologyView| {
        let mut w = sm_topo_api::canonical_words(t);
        w.remove(4);
        w
    };
    let w0 = words(r.topo());
    let (_, rep) = r
        .lib(|h| sm_boolean::regularize_volumes(h, &[c.volume]))
        .unwrap();
    assert!(rep.joined_faces.is_empty() && rep.joined_edges.is_empty() && rep.refused.is_empty());
    let d = r
        .run(&Regularize {
            volumes: vec![c.volume],
        })
        .unwrap();
    assert!(
        d.outputs.get("refused").is_none_or(|v| v.is_empty()),
        "nothing refused: {:?}",
        d.outputs
    );
    assert_eq!(words(r.topo()), w0);
    // Twice from scratch gives the same state hash.
    let (mut a, mut b) = (Rig::cc(), Rig::cc());
    for rig in [&mut a, &mut b] {
        let v = halves(rig);
        rig.run(&Regularize { volumes: vec![v] }).unwrap();
    }
    assert_eq!(a.hash(), b.hash());
}

#[test]
fn regularize_refuses_a_bad_argument_without_changing_the_state() {
    let mut r = Rig::cc();
    let c = r.brick([1.0; 3], [0.5; 3]);
    let h = r.hash();
    assert!(r.run(&Regularize { volumes: vec![] }).is_err());
    assert!(
        r.run(&Regularize {
            volumes: vec![c.body]
        })
        .is_err()
    );
    assert_eq!(r.hash(), h);
}

#[test]
fn regularized_cylinder_meshes_with_mesh_auto() {
    const SCRIPT: &str = r#"
let c = model.create_cylinder(height: 2.0, radius: 0.75)
let w = model.webcut_plane(targets: c.body, plane: {origin: [0.0, 0.0, 0.0], normal: [1.0, 0.0, 0.0]})
let u = model.unite(bodies: w.positive | w.negative, regularize: false)
model.regularize(volumes: u.volumes)
mesh.size(targets: all volumes, size: 0.4)
mesh.auto(strategy: "default")
let m = mesh.run(targets: all volumes)
"#;
    let mut s = Server::new(standard_with(
        Some("tester".into()),
        Some("2026-10-24T00:00:00Z".into()),
    ))
    .expect("server");
    s.call(Request::ScriptRun {
        text: SCRIPT.into(),
        path: None,
        includes: Default::default(),
    })
    .unwrap_or_else(|e| panic!("script: {e} {}", e.data));
    let st = s.workspace().state();
    let model = st.layer::<ModelLayer>().expect("model");
    assert_eq!(model.topo().cells(Dim::Face).len(), 3);
    let mesh = st.layer::<MeshLayer>().expect("mesh");
    let v = model.topo().cells(Dim::Volume)[0];
    assert!(mesh.mesh_of(v).is_some(), "the volume has a mesh");
}

/// Cut every body in `bodies` at `axis = 0`, returning all the pieces.
fn cut_all(r: &mut Rig, bodies: Vec<EntityId>, axis: PlaneAxis) -> Vec<EntityId> {
    let plane = PlaneSpec::Axis(AxisPlane { axis, offset: 0.0 });
    let d = r
        .run(&WebcutPlane {
            targets: bodies,
            plane,
            keep: None,
            merge: None,
            imprint: None,
        })
        .unwrap();
    d.outputs["positive"]
        .iter()
        .chain(&d.outputs["negative"])
        .copied()
        .collect()
}

/// Several joins in one pass chain (keep of one pair is the drop of the next); the roles must
/// list only the survivors.
#[test]
fn regularize_survives_chained_joins_of_quartered_and_octant_bodies() {
    let mut reference = Rig::cc();
    reference.cylinder(H, R, [0.0, 0.0, 0.0]);
    let want = counts(reference.topo());
    assert_eq!(want, [2, 3, 3, 1, 1]);

    let mut r = Rig::cc();
    let c = r.cylinder(H, R, [0.0, 0.0, 0.0]);
    let q = cut_all(&mut r, vec![c.body], PlaneAxis::X);
    let q = cut_all(&mut r, q, PlaneAxis::Y);
    assert_eq!(q.len(), 4);
    let u = r
        .run(&Unite {
            bodies: q,
            keep: None,
            regularize: Some(false),
            tol: None,
        })
        .unwrap();
    let v = u.outputs["volumes"][0];
    r.run(&Regularize { volumes: vec![v] }).unwrap();
    let t = r.topo();
    assert_valid(t);
    assert_eq!((counts(t)[0], counts(t)[1], counts(t)[2]), (2, 3, 3));
    assert_eq!(
        counts(t),
        want,
        "quartered cylinder returns to create_cylinder counts"
    );
    let vol = volume_of(t, v);
    assert!(
        (vol - PI * R * R * H).abs() <= 1e-12 * PI * R * R * H,
        "volume {vol}"
    );

    let mut r = Rig::cc();
    let b = r.brick([1.0, 1.0, 1.0], [0.0, 0.0, 0.0]);
    let o = cut_all(&mut r, vec![b.body], PlaneAxis::X);
    let o = cut_all(&mut r, o, PlaneAxis::Y);
    let o = cut_all(&mut r, o, PlaneAxis::Z);
    assert_eq!(o.len(), 8);
    let u = r
        .run(&Unite {
            bodies: o,
            keep: None,
            regularize: Some(false),
            tol: None,
        })
        .unwrap();
    let v = u.outputs["volumes"][0];
    r.run(&Regularize { volumes: vec![v] }).unwrap();
    let t = r.topo();
    assert_valid(t);
    assert_eq!(
        counts(t),
        [8, 12, 6, 1, 1],
        "octant brick regularizes to the brick's V, E, F"
    );
    let vol = volume_of(t, v);
    assert!((vol - 1.0).abs() <= 1e-12, "volume {vol}");
}
