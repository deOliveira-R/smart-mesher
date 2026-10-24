//! Acceptance tests of T7: the intent and mesh ops end to end with the stub schemes.

mod common;

use common::*;
use serde_json::json;
use sm_core::EntityKind;
use sm_layer_api::{MeshLayer, ModelLayer};
use sm_mesh_api::MeshRead;
use sm_op_api::{DerivedCache, LayerType, QuietControl};
use sm_scheme_api::stubs;

#[test]
fn stub_sweep_brick_10_size_1_gives_1000_cells_and_1331_vertices() {
    let mut s = brick_session([10.0; 3], 1.0);
    let before = s.state().entity_count();
    let (_, d) = mesh_run(&mut s).expect("mesh.run");
    let st = s.state();
    assert_eq!(st.count_of(EntityKind::MeshCell), 1000);
    assert_eq!(st.count_of(EntityKind::MeshVertex), 1331);
    assert_eq!(st.count_of(EntityKind::Mesh), 1);
    assert_eq!(d.outputs["mesh"], vec![id(EntityKind::Mesh, 1)]);
    assert_eq!(d.touched_layers, vec!["meshes".to_owned()]);
    // Delta.created is exactly what the mesh holds, ascending by (kind, serial), Mesh first.
    let layer = st.layer::<MeshLayer>().expect("layer");
    let (mid, e) = layer.meshes().next().expect("one mesh");
    let m = e.mesh();
    assert_eq!(d.created.first(), Some(&mid));
    assert!(d.created.windows(2).all(|w| w[0] < w[1]));
    assert_eq!(st.entity_count() - before, d.created.len());
    let of = |k| d.created.iter().filter(|i| i.kind() == k).count();
    assert_eq!(of(EntityKind::MeshCell), m.cell_count());
    assert_eq!(of(EntityKind::MeshFace), m.face_count());
    assert_eq!(of(EntityKind::MeshVertex), m.vertex_count());
    assert_eq!(e.selection, vec![vol(1)]);
    assert!(!e.stale);
    assert!(e.plan.is_some());
}
