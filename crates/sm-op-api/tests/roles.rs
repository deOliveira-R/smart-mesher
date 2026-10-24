//! ACR-2: output roles, `x-positional`, `x-entity-kind`.

use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_op_api::*;

const OUT: &[OutputRole] = &[
    OutputRole::new("vol", &[EntityKind::TopoVolume], Cardinality::One, "the volume"),
    OutputRole::new("faces", &[EntityKind::TopoFace], Cardinality::Many, "its faces"),
    OutputRole::new("tag", &[], Cardinality::Optional, "optional anything"),
];

/// `mode`: 0 ok, 1 undeclared role, 2 wrong kind, 3 missing `vol`, 4 two `vol`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Mk {
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoFace"]))]
    target: Option<EntityId>,
    #[schemars(extend("x-entity-kind" = ["TopoFace"]))]
    many: Vec<EntityId>,
    mode: u32,
}

fn body(mode: u32, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
    match mode {
        3 => {}
        4 => {
            ctx.alloc_as("vol", EntityKind::TopoVolume)?;
            ctx.alloc_as("vol", EntityKind::TopoVolume)?;
        }
        m => {
            let k = if m == 2 {
                EntityKind::TopoEdge
            } else {
                EntityKind::TopoVolume
            };
            ctx.alloc_as("vol", k)?;
            ctx.alloc_as("faces", EntityKind::TopoFace)?;
            ctx.alloc_as("faces", EntityKind::TopoFace)?;
            if m == 1 {
                ctx.alloc_as("extra", EntityKind::TopoFace)?;
            }
        }
    }
    Ok(())
}

impl TypedOp for Mk {
    const NAME: &'static str = "t.mk";
    const OUTPUTS: &'static [OutputRole] = OUT;
    const POSITIONAL: Option<&'static str> = Some("target");
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        body(self.mode, ctx)
    }
}

/// Same body, nothing declared.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Free {
    mode: u32,
}

impl TypedOp for Free {
    const NAME: &'static str = "t.free";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        body(self.mode, ctx)
    }
}

fn engine() -> Engine {
    let mut r = Registry::new();
    r.register::<Mk>().unwrap();
    r.register::<Free>().unwrap();
    Engine::new(
        Arc::new(r),
        Services::new(),
        Arc::new(DerivedCache::default()),
    )
    .unwrap()
}

fn mk(mode: u32) -> Mk {
    Mk {
        target: None,
        many: vec![],
        mode,
    }
}

#[test]
fn declared_roles_pass_and_violations_are_invalid_without_state_change() {
    let e = engine();
    let q = QuietControl::new();
    let mut st = e.registry().new_state();
    let d = e.apply(&mut st, &mk(0), &q.control()).unwrap();
    assert_eq!(d.outputs["faces"].len(), 2);
    let h = st.hash();
    for mode in 1..=4 {
        let err = e.apply(&mut st, &mk(mode), &q.control()).unwrap_err();
        assert!(
            matches!(
                &err,
                ApplyError::Op {
                    source: OpError::Invalid(_),
                    ..
                }
            ),
            "mode {mode}: {err:?}"
        );
        assert_eq!(st.hash(), h, "state unchanged after a failed op");
    }
}

#[test]
fn undeclared_ops_are_unchecked_and_hash_identically() {
    let e = engine();
    let q = QuietControl::new();
    let (mut a, mut b) = (e.registry().new_state(), e.registry().new_state());
    e.apply(&mut a, &mk(0), &q.control()).unwrap();
    e.apply(&mut b, &Free { mode: 0 }, &q.control()).unwrap();
    assert_eq!(a.hash(), b.hash(), "declaring OUTPUTS does not change hashes");
    // Violations are accepted when nothing is declared.
    e.apply(&mut b, &Free { mode: 1 }, &q.control()).unwrap();
    e.apply(&mut b, &Free { mode: 3 }, &q.control()).unwrap();
}

#[test]
fn nested_batch_members_are_checked() {
    let e = engine();
    let q = QuietControl::new();
    let mut st = e.registry().new_state();
    let rec = |m| OpRecord::of(&mk(m)).unwrap();
    let ok = Batch {
        ops: vec![rec(0), rec(0)],
    };
    e.apply(&mut st, &ok, &q.control()).unwrap();
    let h = st.hash();
    let bad = Batch {
        ops: vec![rec(0), rec(2)],
    };
    assert!(e.apply(&mut st, &bad, &q.control()).is_err());
    assert_eq!(st.hash(), h);
}

#[test]
fn info_exposes_roles_and_positional_marker() {
    let e = engine();
    let cat = e.registry().catalog();
    let find = |n: &str| {
        cat.iter()
            .map(|i| i.as_ref().unwrap())
            .find(|i| i.name == n)
            .unwrap()
    };
    let info = find("t.mk");
    assert_eq!(info.outputs.len(), 3);
    assert_eq!(info.positional.as_deref(), Some("target"));
    let sv = info.schema.clone().to_value();
    let props = &sv["properties"];
    assert_eq!(props["target"][X_POSITIONAL], true);
    assert!(props["many"].get(X_POSITIONAL).is_none());
    let free = find("t.free");
    assert!(free.outputs.is_empty() && free.positional.is_none());
    serde_json::to_value(info).unwrap();
}

#[test]
fn x_entity_kind_attribute_reaches_the_schema_and_the_batch_schema() {
    let e = engine();
    let s = e.registry().schema("t.mk").unwrap().to_value();
    let kinds = serde_json::json!(["TopoVolume", "TopoFace"]);
    assert_eq!(s["properties"]["target"][X_ENTITY_KIND], kinds);
    assert_eq!(
        s["properties"]["many"][X_ENTITY_KIND],
        serde_json::json!(["TopoFace"])
    );
    // Names are canonical `EntityKind` names.
    for n in kinds.as_array().unwrap() {
        serde_json::from_value::<EntityKind>(n.clone()).unwrap();
    }
    // Annotations survive the batch rewrite of inline ids to the shared def.
    let b = e.registry().batch_schema().unwrap().to_value();
    let t = &b["$defs"]["t.mk.params"]["properties"]["target"];
    assert_eq!(t[X_ENTITY_KIND], kinds, "{t}");
    assert_eq!(t[X_POSITIONAL], true, "{t}");
}
