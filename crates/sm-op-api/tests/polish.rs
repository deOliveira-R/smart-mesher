//! Stage-0b polish: registration validation, `record_role`, owned catalog types.

use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_op_api::*;

const OK_OUT: &[OutputRole] = &[
    OutputRole::new("vols", &[EntityKind::TopoVolume], Cardinality::Many, "volumes"),
    OutputRole::new("_x1", &[], Cardinality::Optional, "anything"),
];
const BAD_NAME: &[OutputRole] = &[OutputRole::new("Vols", &[], Cardinality::Many, "")];
const DUP: &[OutputRole] = &[
    OutputRole::new("a", &[], Cardinality::Many, ""),
    OutputRole::new("a", &[], Cardinality::Many, ""),
];

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Make {
    n: u32,
}
impl TypedOp for Make {
    const NAME: &'static str = "t.make";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        for _ in 0..self.n {
            ctx.alloc(EntityKind::TopoVolume)?;
        }
        Ok(())
    }
}

/// Reports `ids` under role `vols`; optionally retires `retire` first or allocates a new one.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Report {
    ids: Vec<EntityId>,
    #[serde(default)]
    retire: Option<EntityId>,
    #[serde(default)]
    alloc_first: bool,
}
impl TypedOp for Report {
    const NAME: &'static str = "t.report";
    const OUTPUTS: &'static [OutputRole] = OK_OUT;
    const POSITIONAL: Option<&'static str> = Some("ids");
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if let Some(r) = self.retire {
            ctx.retire(r)?;
        }
        if self.alloc_first {
            ctx.alloc_as("vols", EntityKind::TopoVolume)?;
        }
        ctx.record_role("vols", &self.ids)
    }
}

macro_rules! bad_op {
    ($ty:ident, $name:literal, $out:expr, $pos:expr) => {
        #[derive(Debug, Serialize, Deserialize, JsonSchema)]
        struct $ty {
            a: u32,
        }
        impl TypedOp for $ty {
            const NAME: &'static str = $name;
            const OUTPUTS: &'static [OutputRole] = $out;
            const POSITIONAL: Option<&'static str> = $pos;
            fn apply(&self, _: &mut ApplyCtx<'_>) -> Result<(), OpError> {
                Ok(())
            }
        }
    };
}
bad_op!(BadPos, "t.badpos", &[], Some("nope"));
bad_op!(BadRole, "t.badrole", BAD_NAME, None);
bad_op!(DupRole, "t.duprole", DUP, None);

fn vol(n: u64) -> EntityId {
    EntityId::new(EntityKind::TopoVolume, n).unwrap()
}

#[test]
fn registration_rejects_bad_positional_and_roles() {
    let mut r = Registry::new();
    assert!(matches!(
        r.register::<BadPos>().err().unwrap(),
        RegistryError::Schema { .. }
    ));
    assert!(matches!(
        r.register::<BadRole>().err().unwrap(),
        RegistryError::BadOutputRole { .. }
    ));
    assert!(matches!(
        r.register::<DupRole>().err().unwrap(),
        RegistryError::DuplicateOutputRole { .. }
    ));
    assert_eq!(r.op_names(), ["core.batch"], "rejected ops are not registered");
    r.register::<Report>().unwrap();
    assert!(r.catalog().iter().all(Result::is_ok));
    assert!(valid_role_name("_a9") && !valid_role_name("9a") && !valid_role_name(""));
}

fn engine() -> Engine {
    let mut r = Registry::new();
    r.register::<Make>().unwrap();
    r.register::<Report>().unwrap();
    Engine::new(Arc::new(r), Services::new(), Arc::new(DerivedCache::default())).unwrap()
}

fn report(ids: Vec<EntityId>, retire: Option<EntityId>, alloc_first: bool) -> Report {
    Report {
        ids,
        retire,
        alloc_first,
    }
}

#[test]
fn record_role_reports_live_ids_sorted_without_adopting() {
    let e = engine();
    let q = QuietControl::new();
    let mut st = e.registry().new_state();
    e.apply(&mut st, &Make { n: 3 }, &q.control()).unwrap();
    let d = e
        .apply(&mut st, &report(vec![vol(3), vol(1), vol(3)], None, false), &q.control())
        .unwrap();
    assert_eq!(d.outputs["vols"], vec![vol(1), vol(3)]);
    assert!(d.created.is_empty(), "record_role does not create");
    // An id allocated earlier in the same op is accepted.
    let d = e
        .apply(&mut st, &report(vec![vol(4)], None, true), &q.control())
        .unwrap();
    assert_eq!(d.outputs["vols"], vec![vol(4)]);
}

#[test]
fn record_role_rejects_retired_unknown_and_wrong_kind() {
    let e = engine();
    let q = QuietControl::new();
    let mut st = e.registry().new_state();
    e.apply(&mut st, &Make { n: 2 }, &q.control()).unwrap();
    let h = st.hash();
    let face = EntityId::new(EntityKind::TopoFace, 1).unwrap();
    for op in [
        report(vec![vol(9)], None, false),
        report(vec![vol(1)], Some(vol(1)), false),
        report(vec![face], None, false),
    ] {
        let r = e.apply(&mut st, &op, &q.control());
        assert!(
            matches!(r, Err(ApplyError::Op { source: OpError::Invalid(_), .. })),
            "{op:?}: {r:?}"
        );
    }
    assert_eq!(st.hash(), h);
}

#[test]
fn owned_catalog_round_trips_with_tool_name() {
    let e = engine();
    let reg = e.registry();
    for i in reg.catalog().into_iter().map(Result::unwrap) {
        let v = serde_json::to_value(&i).unwrap();
        assert_eq!(v["tool_name"], i.name.replace('.', "__"));
        let o: OpInfoOwned = serde_json::from_value(v.clone()).unwrap();
        assert_eq!(o, OpInfoOwned::from(&i));
        assert_eq!(serde_json::to_value(&o).unwrap(), v, "same JSON shape");
    }
    let rep = reg
        .catalog()
        .into_iter()
        .map(Result::unwrap)
        .find(|i| i.name == "t.report")
        .unwrap();
    let o = OpInfoOwned::from(&rep);
    assert_eq!(o.outputs[0].name, "vols");
    assert_eq!(o.positional.as_deref(), Some("ids"));
}
