//! W0f gaps4: `[*]` batch splices, the typed batch schema, `Batch::concat`, versioned services.

use std::collections::BTreeMap;
use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::json;
use sm_core::{EntityId, EntityKind};
use sm_op_api::*;

#[derive(Clone, Default)]
struct Pieces {
    /// piece -> value; plus the order in which "fuse" saw piece lists.
    vals: BTreeMap<EntityId, i64>,
    fused: Vec<Vec<EntityId>>,
}

impl LayerType for Pieces {
    const NAME: &'static str = "pieces";
    fn digest(&self, d: &mut Digest) {
        for (id, v) in &self.vals {
            d.id(*id).i64(*v);
        }
        for f in &self.fused {
            d.u64(f.len() as u64);
            for id in f {
                d.id(*id);
            }
        }
    }
    fn is_empty(&self) -> bool {
        self.vals.is_empty() && self.fused.is_empty()
    }
    fn on_retired(&mut self, ids: &[EntityId]) {
        for id in ids {
            self.vals.remove(id);
        }
    }
}

/// Webcut-like: allocates `n` pieces under role `piece`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Cut {
    n: u32,
}

impl TypedOp for Cut {
    const NAME: &'static str = "t.cut";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        for i in 0..self.n {
            let id = ctx.alloc_as("piece", EntityKind::TopoVolume)?;
            ctx.layer_mut::<Pieces>()?.vals.insert(id, i64::from(i));
        }
        Ok(())
    }
}

/// An op over a list of pieces.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Fuse {
    pieces: Vec<EntityId>,
    #[serde(default)]
    label: Option<String>,
}

impl TypedOp for Fuse {
    const NAME: &'static str = "t.fuse";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        for p in &self.pieces {
            ctx.resolve(*p)?;
        }
        let scale = ctx.service::<Scale>().map_or(1, |s| s.0);
        let l = ctx.layer_mut::<Pieces>()?;
        for p in &self.pieces {
            *l.vals.get_mut(p).ok_or(OpError::UnknownEntity(*p))? *= scale;
        }
        l.fused.push(self.pieces.clone());
        Ok(())
    }
}

struct Scale(i64);

fn registry() -> Registry {
    let mut r = Registry::new();
    r.register_layer::<Pieces>().unwrap();
    r.register::<Cut>().unwrap();
    r.register::<Fuse>().unwrap();
    r
}

fn session_with(services: Services) -> Session {
    Session::new(
        Engine::new(
            Arc::new(registry()),
            services,
            Arc::new(DerivedCache::default()),
        )
        .unwrap(),
    )
}

fn rec(name: &str, p: serde_json::Value) -> OpRecord {
    OpRecord::new(name, 1, p).unwrap()
}

fn apply(s: &mut Session, op: &dyn Op) -> Result<(Hash256, Delta), TimelineError> {
    let c = QuietControl::new();
    s.apply(op, Meta::default(), &c.control())
}

#[test]
fn star_reference_splices_every_piece_and_round_trips() {
    let mut s = session_with(Services::new());
    let b = Batch {
        ops: vec![
            rec("t.cut", json!({"n": 4})),
            rec("t.cut", json!({"n": 0})),
            rec(
                "t.fuse",
                json!({"pieces": ["$0.piece[*]", "$1.piece[*]"], "label": "$$0.piece[*]"}),
            ),
        ],
    };
    let (h, d) = apply(&mut s, &b).unwrap();
    let pieces = d.outputs["0.piece"].clone();
    assert_eq!(pieces.len(), 4);
    assert!(!d.outputs.contains_key("1.piece"), "empty role");
    let fused = &s.state().layer::<Pieces>().unwrap().fused;
    assert_eq!(fused, &vec![pieces.clone()], "allocation order, empty role splices nothing");

    // The stored node keeps the reference strings, not the spliced ids.
    let stored = &s.timeline().get(h).unwrap().body.op;
    let ops = stored.params["ops"].as_array().unwrap();
    assert_eq!(
        ops[2]["params"],
        json!({"pieces": ["$0.piece[*]", "$1.piece[*]"], "label": "$$0.piece[*]"})
    );

    // A client re-sending the stored record gets the same node; replay gives the same hash.
    let again: Batch = from_value(&stored.params).unwrap();
    let mut s2 = session_with(Services::new());
    let (h2, _) = apply(&mut s2, &again).unwrap();
    assert_eq!(h, h2);
    let c = QuietControl::new();
    assert_eq!(
        s.state_at_from_root(h, &c.control()).unwrap().hash(),
        s.state().hash()
    );
    let files = s.timeline().to_files().unwrap();
    let t = Timeline::from_files(&files).unwrap();
    let reopened = Session::open(
        Engine::new(
            Arc::new(registry()),
            Services::new(),
            Arc::new(DerivedCache::default()),
        )
        .unwrap(),
        t,
        Head::Ref("main".into()),
        &c.control(),
    )
    .unwrap();
    assert_eq!(reopened.state().hash(), s.state().hash());
}

#[test]
fn star_reference_outside_an_array_is_invalid_and_bare_alias_is_first() {
    let mut s = session_with(Services::new());
    let bad = Batch {
        ops: vec![
            rec("t.cut", json!({"n": 2})),
            rec("t.fuse", json!({"pieces": [], "label": "$0.piece[*]"})),
        ],
    };
    assert!(apply(&mut s, &bad).is_err());
    let ok = Batch {
        ops: vec![
            rec("t.cut", json!({"n": 2})),
            rec("t.fuse", json!({"pieces": ["$0.piece"]})),
        ],
    };
    let (_, d) = apply(&mut s, &ok).unwrap();
    assert_eq!(
        s.state().layer::<Pieces>().unwrap().fused,
        vec![vec![d.outputs["0.piece"][0]]]
    );
}

#[test]
fn concat_matches_sequential_application() {
    let member = OpRecord::of(&Batch {
        ops: vec![
            rec("t.cut", json!({"n": 3})),
            rec("t.fuse", json!({"pieces": ["$0.piece[*]"], "label": "$$x"})),
            rec("t.fuse", json!({"pieces": ["$0.piece[2]", "$0.piece"]})),
        ],
    })
    .unwrap();
    let rs = vec![
        rec("t.cut", json!({"n": 2})),
        member.clone(),
        rec("t.fuse", json!({"pieces": ["TopoVolume#1"], "label": "$lit"})),
        member,
    ];
    let mut seq = session_with(Services::new());
    for r in &rs {
        let op = seq.engine().registry().decode(r).unwrap();
        apply(&mut seq, op.as_ref()).unwrap();
    }
    let flat = Batch::concat(&rs).unwrap();
    assert_eq!(flat.ops.len(), 1 + 3 + 1 + 3);
    assert_eq!(flat.ops[2].params["label"], json!("$$x"));
    assert_eq!(flat.ops[4].params["label"], json!("$$lit"));
    assert_eq!(flat.ops[6].params["pieces"], json!(["$5.piece[*]"]));
    let mut one = session_with(Services::new());
    apply(&mut one, &flat).unwrap();
    assert_eq!(one.state().hash(), seq.state().hash());
    let fused = &one.state().layer::<Pieces>().unwrap().fused;
    assert_eq!(fused.len(), 5);
}

#[test]
fn batch_schema_types_every_op_and_accepts_references() {
    let r = registry();
    let s = r.batch_schema().unwrap();
    assert_eq!(r.schema("core.batch").unwrap(), s);
    let cat = r.catalog();
    let b = cat
        .iter()
        .map(|x| x.as_ref().unwrap())
        .find(|i| i.name == "core.batch")
        .unwrap();
    assert_eq!(b.schema, s);
    let v = s.as_value();
    let variants = v["properties"]["ops"]["items"]["oneOf"].as_array().unwrap();
    let names: Vec<&str> = variants
        .iter()
        .map(|x| x["properties"]["name"]["const"].as_str().unwrap())
        .collect();
    assert_eq!(names, vec!["t.cut", "t.fuse"], "core.batch itself excluded");
    assert_eq!(
        variants[1]["properties"]["params"]["$ref"],
        json!("#/$defs/t.fuse.params")
    );
    let defs = v["$defs"].as_object().unwrap();
    assert!(defs.contains_key("t.cut.params") && defs.contains_key("t.fuse.params"));
    let any = defs["BatchEntityId"]["anyOf"].as_array().unwrap();
    let pats: Vec<&str> = any.iter().map(|x| x["pattern"].as_str().unwrap()).collect();
    assert_eq!(
        pats,
        vec![
            "^[A-Za-z]+#[0-9]+$",
            batch::REF_PATTERN,
            batch::ESCAPED_LITERAL_PATTERN
        ]
    );
    // No raw EntityId pattern or `#/$defs/EntityId` reference survives in the op params.
    let text = serde_json::to_string(&defs["t.fuse.params"]).unwrap();
    assert!(text.contains("#/$defs/BatchEntityId"), "{text}");
    assert!(!text.contains("#/$defs/EntityId"), "{text}");
    assert!(!text.contains("[A-Za-z]"), "{text}");
    // The per-op schema is unchanged.
    assert!(
        serde_json::to_string(&r.schema("t.fuse").unwrap())
            .unwrap()
            .contains("EntityId")
    );
}

#[test]
fn versioned_service_change_reports_upgraded() {
    let svc = |v: u32, k: i64| Services::new().with_versioned("scale", v, Scale(k));
    let mut s = session_with(svc(1, 2));
    let (_, d) = apply(&mut s, &Cut { n: 2 }).unwrap();
    let (h, _) = apply(
        &mut s,
        &Fuse {
            pieces: d.outputs["piece"].clone(),
            label: None,
        },
    )
    .unwrap();
    assert_eq!(
        s.timeline().get(h).unwrap().check.versions.get("service:scale"),
        Some(&1)
    );
    assert_eq!(s.engine().services().versions().collect::<Vec<_>>(), vec![("scale", 1)]);
    let t = s.timeline().clone();
    let c = QuietControl::new();
    let open = |services: Services| {
        Session::open(
            Engine::new(Arc::new(registry()), services, Arc::new(DerivedCache::default()))
                .unwrap(),
            t.clone(),
            Head::Ref("main".into()),
            &c.control(),
        )
    };
    match open(svc(2, 3)).err().unwrap() {
        TimelineError::Upgraded { changed, .. } => {
            assert_eq!(changed, vec!["service:scale".to_owned()])
        }
        e => panic!("{e:?}"),
    }
    // Same version, different behaviour: plain divergence.
    assert!(matches!(
        open(svc(1, 3)).err().unwrap(),
        TimelineError::Diverged { .. }
    ));
    // Unversioned service (old timelines): the key exists on one side only, so Diverged.
    assert!(matches!(
        open(Services::new().with(Scale(3))).err().unwrap(),
        TimelineError::Diverged { .. }
    ));
    assert!(open(svc(1, 2)).is_ok());
}
