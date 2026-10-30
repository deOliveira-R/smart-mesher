//! INT-4: `mesh.run` refuses a volume that already has more than one mesh entry.

mod common;

use common::*;
use serde_json::json;
use sm_core::EntityKind;
use sm_layer_api::{CoreLayers, MeshEntry, MeshLayer, ModelLayer, insert_mesh};
use sm_op_api::{ApplyCtx, DerivedCache, Engine, OpError, Registry, Services, Session, TypedOp};
use sm_scheme_api::stubs;

/// Adds a second mesh entry on volume 1 through the layer API, the way `mesh.to_poly` does.
#[derive(Debug, serde::Serialize, serde::Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
struct AddSecondMesh {}

impl TypedOp for AddSecondMesh {
    const NAME: &'static str = "test.add_second_mesh";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let alloc0 = ctx.layer::<MeshLayer>()?.alloc().clone();
        let quanta = ctx.layer::<ModelLayer>()?.hash_quanta();
        let mut alloc = alloc0.clone();
        let fin = sm_mesh::generate::hex_block_bulk([1; 3], [1.0; 3], &mut alloc)
            .map_err(|e| OpError::Invalid(e.to_string()))?;
        let entry = MeshEntry::new(Box::new(fin.mesh), vec![vol(1)], None, quanta);
        insert_mesh(ctx, entry, &alloc0, alloc, "mesh")
    }
}

fn session_with_second_mesh_op() -> Session {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::new(cell_complex)).expect("core");
    reg.install(&sm_mesher::MeshPlugin).expect("mesher");
    reg.register::<CreateBrick>().expect("brick");
    reg.register::<AddSecondMesh>().expect("second");
    let cfg = sm_mesher::MesherConfig::new(stubs::registry());
    Session::new(
        Engine::new(
            std::sync::Arc::new(reg),
            Services::new().with_versioned("mesher", 1, cfg),
            std::sync::Arc::new(DerivedCache::default()),
        )
        .expect("engine"),
    )
}

#[test]
fn mesh_run_refuses_a_volume_with_two_mesh_entries_and_changes_nothing() {
    let mut s = session_with_second_mesh_op();
    run(&mut s, &brick_op([2.0; 3])).expect("brick");
    run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoVolume#1"], "size": 1.0}),
    )
    .expect("size");
    mesh_run(&mut s).expect("first mesh");
    run(&mut s, &AddSecondMesh {}).expect("second entry");
    assert_eq!(s.state().count_of(EntityKind::Mesh), 2);
    let before = s.state().entity_count();
    let msg = match mesh_run(&mut s) {
        Ok(_) => panic!("expected an error"),
        Err(e) => format!("{e:?} {e}"),
    };
    assert!(msg.contains("TopoVolume#1"), "{msg}");
    assert!(msg.contains("Mesh#1") && msg.contains("Mesh#2"), "{msg}");
    assert!(msg.contains("mesh.delete"), "{msg}");
    assert_eq!(s.state().entity_count(), before, "state unchanged");
    assert_eq!(s.state().count_of(EntityKind::Mesh), 2);
    assert_eq!(s.state().layer::<MeshLayer>().expect("layer").len(), 2);
}
