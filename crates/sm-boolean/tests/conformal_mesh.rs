//! Acceptance 12: a webcut brick meshed through the driver is conformal on the cap.

mod common;

use common::rig::*;
use sm_boolean::ops::{AxisPlane, PlaneAxis, PlaneSpec, WebcutPlane};
use sm_mesh_api::{MeshEntityKind, MeshRead, cell_volume, validate};
use sm_topo_api::TopologyViewExt;

fn cut_x2(merge: Option<bool>) -> (Rig, sm_op_api::Delta) {
    let mut r = Rig::cc();
    let b = r.brick([4.0, 2.0, 2.0], [2.0, 1.0, 1.0]);
    let d = r.run(&WebcutPlane { targets: vec![b.body], plane: PlaneSpec::Axis(AxisPlane { axis: PlaneAxis::X, offset: 2.0 }), keep: None, merge, imprint: None }).unwrap();
    (r, d)
}

#[test]
fn merged_cap_gives_a_conformal_mesh() {
    let (r, d) = cut_x2(None);
    let vols = d.outputs["volumes"].clone();
    let out = mesh_volumes(&r, &vols, 1.0);
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 16);
    assert_eq!(m.count(MeshEntityKind::Vertex), 45);
    let total: f64 = (0..16).map(|c| cell_volume(m, sm_mesh_api::CellIx(c))).sum();
    assert!((total - 16.0).abs() < 1e-12, "volume {total}");
    let cap = d.outputs["cut_faces"][0];
    let set = m.classified_on(cap).expect("cap classified");
    assert_eq!(set.faces.len(), 4);
    // Every cap quad has a cell on each side.
    for &q in &set.faces {
        assert!(m.face_cells(q).1.is_some(), "cap quad {q:?} has one side only");
    }
    let _ = r.topo().edge_ends(r.topo().cells(sm_topo_api::Dim::Edge)[0]);
}

#[test]
fn unmerged_cap_doubles_the_interface_nodes() {
    let (r, d) = cut_x2(Some(false));
    let vols = d.outputs["volumes"].clone();
    let out = mesh_volumes(&r, &vols, 1.0);
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 16);
    assert_eq!(m.count(MeshEntityKind::Vertex), 54);
}
