//! `export_step` reports colors and group names (not written to STEP) as located losses, and the
//! report is the only thing that changes: the bytes equal the plain topology write.

mod common;

use std::sync::Arc;

use common::*;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_cadio::{ImportStep, MemoryInputStore, Source, StepInputs, WriteOptions, export_step, write_step};
use sm_core::EntityId;
use sm_layer_api::{LayerResultExt, model_txn};
use sm_model_api::Domain;
use sm_op_api::{ApplyCtx, DerivedCache, Engine, OpError, Services, Session, TypedOp};
use sm_topo_api::Dim;

/// Test op: a volume color, a face color and a group holding `body`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct Decorate {
    body: EntityId,
    volume: EntityId,
    face: EntityId,
}

impl TypedOp for Decorate {
    const NAME: &'static str = "test.decorate";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        model_txn(
            ctx,
            |m, _topo| {
                let cv = m.add_color(1, "steel", Domain::Volume).or_op()?;
                let cf = m.add_color(2, "inlet", Domain::Face).or_op()?;
                m.color_volumes(cv, &[self.volume]).or_op()?;
                m.color_faces(cf, &[self.face]).or_op()?;
                let g = m.add_group("fuel", &[self.body]).or_op()?;
                Ok(vec![cv, cf, g])
            },
            |ids, _new| vec![("rows", ids.clone())],
        )
        .map(|_| ())
    }
}

fn session_with_two_bricks() -> Session {
    let store = Arc::new(MemoryInputStore::new());
    let mut reg = registry(cell_complex);
    reg.register::<Decorate>().expect("register");
    let services = Services::new().with(StepInputs(store));
    let mut s = Session::new(Engine::new(Arc::new(reg), services, Arc::new(DerivedCache::default())).expect("engine"));
    let op = ImportStep { source: Source::inline(&data("two_bricks.stp")), name: None, to_unit: None, assume_unit: None, weld_tol: None, solids: None, sheets: None, check: None };
    run(&mut s, &op).expect("import");
    s
}

#[test]
fn colors_and_groups_are_reported_as_located_losses() {
    let mut s = session_with_two_bricks();
    let o = WriteOptions::default();
    let (b0, b1, v0, f0, plain) = {
        let topo = model(&s).topo();
        let bodies = topo.cells(Dim::Body).to_vec();
        assert!(bodies.len() >= 2);
        let vol_of = |b: EntityId| topo.boundary(b).iter().map(|e| e.cell).find(|c| topo.dim_of(*c) == Some(Dim::Volume)).expect("volume");
        let v0 = vol_of(bodies[0]);
        let f0 = topo.boundary(v0)[0].cell;
        let mut plain = Vec::new();
        write_step(topo, None, &o, &mut plain).expect("plain");
        (bodies[0], bodies[1], v0, f0, plain)
    };
    run(&mut s, &Decorate { body: b0, volume: v0, face: f0 }).expect("decorate");

    // Everything exported: the volume color, the face color and the group member are located.
    let mut bytes = Vec::new();
    let rep = export_step(s.state(), None, &o, &mut bytes).expect("export");
    assert_eq!(bytes, plain, "the report changes, never the bytes");
    let at = |needle: &str, cell: EntityId| rep.losses.iter().any(|l| l.cell == Some(cell) && l.what.contains(needle) && l.what.contains("not written"));
    assert!(at("steel", v0), "{:?}", rep.losses);
    assert!(at("inlet", f0), "{:?}", rep.losses);
    assert!(at("fuel", b0), "{:?}", rep.losses);
    // Deterministic: a second export reports the same.
    let again = export_step(s.state(), None, &o, &mut Vec::new()).expect("again");
    assert_eq!(rep.losses, again.losses);

    // Only the other body exported: nothing colored or grouped is written, nothing is reported.
    let rep1 = export_step(s.state(), Some(&[b1]), &o, &mut Vec::new()).expect("subset");
    assert!(!rep1.losses.iter().any(|l| l.what.contains("steel") || l.what.contains("inlet") || l.what.contains("fuel")), "{:?}", rep1.losses);
}
