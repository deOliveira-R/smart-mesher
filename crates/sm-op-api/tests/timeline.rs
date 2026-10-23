//! Behavioural tests of the contract using a small two-concept sample world.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_op_api::*;
use std::sync::{Arc, Mutex};

// ---------- sample "model" concept ----------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Shape {
    name: String,
    dims: Vec<f64>,
    parent: Option<Eid>,
}

type Model = MapLayer<Shape>;

/// Create an axis-aligned box: one body and its volume.
#[derive(Serialize, Deserialize, JsonSchema)]
struct CreateBox {
    /// Name of the body.
    name: String,
    /// Extent along x.
    x: f64,
    /// Extent along y.
    y: f64,
    /// Extent along z.
    z: f64,
}

impl Op for CreateBox {
    const KIND: &'static str = "model.create_box";
    const VERSION: u32 = 1;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if self.x <= 0.0 || self.y <= 0.0 || self.z <= 0.0 {
            return Err(OpError::invalid("box extents must be positive"));
        }
        let body = ctx.alloc(EntityKind::TopoBody)?;
        let vol = ctx.alloc(EntityKind::TopoVolume)?;
        let patch = MapPatch::new()
            .set(
                body,
                Shape {
                    name: self.name.clone(),
                    dims: vec![self.x, self.y, self.z],
                    parent: None,
                },
            )
            .set(
                vol,
                Shape {
                    name: format!("{}.vol", self.name),
                    dims: vec![],
                    parent: Some(body.into()),
                },
            );
        ctx.patch::<Model>(LayerId::MODEL, &patch)
    }
}

/// Rename an existing entity.
#[derive(Serialize, Deserialize, JsonSchema)]
struct Rename {
    /// Entity to rename (an id returned by an earlier op).
    target: Eid,
    /// New name.
    name: String,
}

impl Op for Rename {
    const KIND: &'static str = "model.rename";
    const VERSION: u32 = 2;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let mut s = ctx
            .layer::<Model>(LayerId::MODEL)?
            .get(self.target)
            .cloned()
            .ok_or(OpError::UnknownEntity(self.target))?;
        s.name.clone_from(&self.name);
        ctx.patch::<Model>(LayerId::MODEL, &MapPatch::new().set(self.target, s))
    }
}

/// Writes, allocates, then fails: must leave no trace.
#[derive(Serialize, Deserialize, JsonSchema)]
struct FailAfterWrite {
    /// Entity removed before failing.
    target: Eid,
    /// Panic instead of returning an error.
    panic: bool,
}

impl Op for FailAfterWrite {
    const KIND: &'static str = "test.fail_after_write";
    const VERSION: u32 = 1;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        ctx.patch::<Model>(LayerId::MODEL, &MapPatch::new().remove(self.target))?;
        ctx.alloc(EntityKind::TopoFace)?;
        if self.panic {
            panic!("boom");
        }
        Err(OpError::invalid("deliberate failure"))
    }
}

struct ModelConcept;

impl Concept for ModelConcept {
    fn name(&self) -> &'static str {
        "model"
    }
    fn register(&self, r: &mut Registry) -> Result<(), RegistryError> {
        r.register_layer::<Model>(LayerId::MODEL)?;
        r.register_op::<CreateBox>()?;
        r.register_op::<Rename>()?;
        r.register_op::<FailAfterWrite>()?;
        // v1 of rename called the field `title`.
        r.register_migration(Rename::KIND, 1, |mut v| {
            if let Value::Table(t) = &mut v
                && let Some(x) = t.remove("title")
            {
                t.insert("name".into(), x);
            }
            Ok(v)
        })
    }
}

// ---------- sample "mesh" concept (separate layers, same api) ----------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct PartRec {
    of: Eid,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct MeshRec {
    of: Eid,
    cells: u32,
}

/// Split an entity into virtual partition cells.
#[derive(Serialize, Deserialize, JsonSchema)]
struct Partition {
    /// Entity to partition.
    target: Eid,
    /// Number of pieces.
    parts: u32,
}

impl Op for Partition {
    const KIND: &'static str = "partition.split";
    const VERSION: u32 = 1;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if !ctx.layer::<Model>(LayerId::MODEL)?.contains(self.target) {
            return Err(OpError::UnknownEntity(self.target));
        }
        let mut patch = MapPatch::new();
        for _ in 0..self.parts {
            patch = patch.set(
                ctx.alloc(EntityKind::PartitionCell)?,
                PartRec { of: self.target },
            );
        }
        ctx.patch::<MapLayer<PartRec>>(LayerId::PARTITION, &patch)
    }
}

/// Generate a mesh of an entity, in blocks, with progress reports.
#[derive(Serialize, Deserialize, JsonSchema)]
struct Mesh {
    /// Entity to mesh.
    target: Eid,
    /// Work blocks (progress granularity).
    blocks: u32,
}

impl Op for Mesh {
    const KIND: &'static str = "mesh.generate";
    const VERSION: u32 = 1;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if !ctx.layer::<Model>(LayerId::MODEL)?.contains(self.target) {
            return Err(OpError::UnknownEntity(self.target));
        }
        let id = ctx.alloc(EntityKind::Mesh)?;
        let mut cells = 0;
        for b in 0..self.blocks {
            ctx.checkpoint(f64::from(b) / f64::from(self.blocks), "meshing")?;
            cells += 8;
        }
        ctx.checkpoint(1.0, "done")?;
        ctx.patch::<MapLayer<MeshRec>>(
            LayerId::MESHES,
            &MapPatch::new().set(
                id,
                MeshRec {
                    of: self.target,
                    cells,
                },
            ),
        )
    }
}

struct MeshConcept;

impl Concept for MeshConcept {
    fn name(&self) -> &'static str {
        "mesh"
    }
    fn register(&self, r: &mut Registry) -> Result<(), RegistryError> {
        r.register_layer::<MapLayer<PartRec>>(LayerId::PARTITION)?;
        r.register_layer::<MapLayer<MeshRec>>(LayerId::MESHES)?;
        r.register_op::<Partition>()?;
        r.register_op::<Mesh>()
    }
}

// ---------- helpers ----------

fn registry() -> Arc<Registry> {
    let mut r = Registry::new();
    r.install(&ModelConcept).unwrap();
    r.install(&MeshConcept).unwrap();
    Arc::new(r)
}

fn meta(n: u32) -> NodeMeta {
    NodeMeta {
        author: "test".into(),
        time: format!("2026-01-01T00:00:{n:02}Z"),
        message: None,
    }
}

fn cbox(name: &str) -> CreateBox {
    CreateBox {
        name: name.into(),
        x: 1.0,
        y: 2.0,
        z: 0.5,
    }
}

fn eid(kind: EntityKind, serial: u64) -> Eid {
    EntityId::new(kind, serial).unwrap().into()
}

/// body A, body B, rename A, partition A, mesh A
fn build(ws: &mut Workspace<MemStore>) -> Vec<ApplyReport> {
    let mut reports = Vec::new();
    reports.push(ws.apply(&cbox("A"), meta(0)).unwrap());
    reports.push(ws.apply(&cbox("B"), meta(1)).unwrap());
    let a = reports[0].created[0].into();
    reports.push(
        ws.apply(
            &Rename {
                target: a,
                name: "A2".into(),
            },
            meta(2),
        )
        .unwrap(),
    );
    reports.push(
        ws.apply(
            &Partition {
                target: a,
                parts: 3,
            },
            meta(3),
        )
        .unwrap(),
    );
    reports.push(
        ws.apply(
            &Mesh {
                target: a,
                blocks: 4,
            },
            meta(4),
        )
        .unwrap(),
    );
    reports
}

// ---------- tests ----------

#[test]
fn replay_determinism() {
    let mut ws = Workspace::open(registry(), MemStore::new()).unwrap();
    build(&mut ws);
    let live = ws.state_hash();
    let a = ws.replay_from_root().unwrap().hash();
    let b = ws.replay_from_root().unwrap().hash();
    assert_eq!(a, b);
    assert_eq!(a, live);
    // A second independent workspace running the same ops reaches the same node and state.
    let mut ws2 = Workspace::open(registry(), MemStore::new()).unwrap();
    build(&mut ws2);
    assert_eq!(ws2.at(), ws.at());
    assert_eq!(ws2.state_hash(), live);
    assert_eq!(ws2.store().files(), ws.store().files());
}

#[test]
fn ids_are_assigned_deterministically_and_referenced_by_later_ops() {
    let mut ws = Workspace::open(registry(), MemStore::new()).unwrap();
    let r = build(&mut ws);
    assert_eq!(
        r[0].created,
        vec![
            eid(EntityKind::TopoBody, 1).0,
            eid(EntityKind::TopoVolume, 1).0
        ]
    );
    assert_eq!(
        r[1].created,
        vec![
            eid(EntityKind::TopoBody, 2).0,
            eid(EntityKind::TopoVolume, 2).0
        ]
    );
    let model = ws.state().layer::<Model>(LayerId::MODEL).unwrap();
    assert_eq!(model.get(r[0].created[0]).unwrap().name, "A2"); // Rename found the earlier id
    assert_eq!(r[3].created.len(), 3);
    assert_eq!(ws.state().ids().last(EntityKind::PartitionCell), 3);
    // Referencing a missing id fails cleanly.
    let before = ws.state_hash();
    let bad = ws.apply(
        &Rename {
            target: eid(EntityKind::TopoBody, 99),
            name: "x".into(),
        },
        meta(9),
    );
    assert!(matches!(
        bad,
        Err(TimelineError::Apply(ApplyError::Op(
            OpError::UnknownEntity(_)
        )))
    ));
    assert_eq!(ws.state_hash(), before);
}

#[test]
fn failing_op_changes_nothing() {
    let mut ws = Workspace::open(registry(), MemStore::new()).unwrap();
    let r = build(&mut ws);
    let (hash, at, files) = (ws.state_hash(), ws.at(), ws.store().files().clone());
    let target = r[1].created[0].into();
    let err = ws
        .apply(
            &FailAfterWrite {
                target,
                panic: false,
            },
            meta(10),
        )
        .unwrap_err();
    assert!(matches!(
        err,
        TimelineError::Apply(ApplyError::Op(OpError::Invalid(_)))
    ));
    assert_eq!((ws.state_hash(), ws.at()), (hash, at));
    assert_eq!(ws.store().files(), &files);
    assert_eq!(ws.state().ids().last(EntityKind::TopoFace), 0);
    // Invalid params also fail before touching anything.
    let err = ws
        .apply(
            &CreateBox {
                name: "bad".into(),
                x: -1.0,
                y: 1.0,
                z: 1.0,
            },
            meta(11),
        )
        .unwrap_err();
    assert!(matches!(err, TimelineError::Apply(ApplyError::Op(_))));
    assert_eq!(ws.state_hash(), hash);
}

#[test]
fn panicking_op_is_rolled_back_too() {
    let mut ws = Workspace::open(registry(), MemStore::new()).unwrap();
    let r = build(&mut ws);
    let hash = ws.state_hash();
    let target = r[1].created[0].into();
    let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _ = ws.apply(
            &FailAfterWrite {
                target,
                panic: true,
            },
            meta(10),
        );
    }));
    assert!(res.is_err());
    assert_eq!(ws.state_hash(), hash);
}

#[test]
fn fork_from_old_node_and_move_between_branches_by_deltas() {
    let mut ws = Workspace::open(registry(), MemStore::new()).unwrap();
    let r = build(&mut ws);
    ws.create_branch("main").unwrap_err(); // already attached to unborn main? exists after apply
    let tip_main = ws.at().unwrap();
    let hash_main = ws.state_hash();
    let hash_after_b = r[1].state_hash;

    // Fork from the node after "B".
    ws.checkout(&Target::Node(r[1].node)).unwrap();
    assert_eq!(ws.state_hash(), hash_after_b);
    assert!(matches!(ws.head(), Head::Detached(_)));
    let fork = ws.apply(&cbox("C"), meta(20)).unwrap();
    ws.create_branch("try-c").unwrap();
    // Same deterministic ids as the original line would have handed out next.
    assert_eq!(fork.created[0], eid(EntityKind::TopoBody, 3).0);
    let hash_fork = ws.state_hash();
    assert_ne!(hash_fork, hash_main);

    // Hop between the two tips: only deltas, and states match what we saw.
    ws.checkout(&Target::Branch("main".into())).unwrap();
    assert_eq!((ws.at(), ws.state_hash()), (Some(tip_main), hash_main));
    ws.checkout(&Target::Branch("try-c".into())).unwrap();
    assert_eq!((ws.at(), ws.state_hash()), (Some(fork.node), hash_fork));
    // And they equal a from-scratch op replay.
    assert_eq!(ws.replay_from_root().unwrap().hash(), hash_fork);
    ws.checkout(&Target::Node(r[0].node)).unwrap();
    assert_eq!(ws.state_hash(), r[0].state_hash);
    assert_eq!(ws.store().branches().unwrap().len(), 2);
}

#[test]
fn reopen_from_files_uses_deltas_and_survives_missing_or_corrupt_deltas() {
    let mut ws = Workspace::open(registry(), MemStore::new()).unwrap();
    build(&mut ws);
    let (hash, at) = (ws.state_hash(), ws.at());
    let files = ws.store().files().clone();

    let ws2 = Workspace::open(registry(), MemStore::from_files(files.clone())).unwrap();
    assert_eq!((ws2.state_hash(), ws2.at()), (hash, at));

    // Drop all deltas: ops are the source of truth and rebuild them.
    let stripped = files
        .iter()
        .filter(|(k, _)| !k.starts_with("timeline/deltas/"))
        .map(|(k, v)| (k.clone(), v.clone()));
    let ws3 = Workspace::open(registry(), MemStore::from_files(stripped.collect())).unwrap();
    assert_eq!(ws3.state_hash(), hash);
    assert_eq!(
        ws3.store()
            .files()
            .keys()
            .filter(|k| k.starts_with("timeline/deltas/"))
            .count(),
        5
    );

    // A delta that lies (wrong content, valid syntax) is caught by the hash check.
    let mut tampered = files.clone();
    let key = tampered
        .keys()
        .find(|k| k.starts_with("timeline/deltas/"))
        .unwrap()
        .clone();
    let text = tampered[&key].replace("A", "Z");
    tampered.insert(key, text);
    let ws4 = Workspace::open(registry(), MemStore::from_files(tampered)).unwrap();
    assert_eq!(ws4.state_hash(), hash);
}

#[test]
fn toml_roundtrip_is_canonical_and_hash_stable() {
    let node = Node {
        parent: None,
        op: OpRecord::new(&cbox("quote\"d \\ name\n")).unwrap(),
        meta: NodeMeta {
            author: "me".into(),
            time: "t".into(),
            message: Some("hi".into()),
        },
    };
    let text = node.to_toml().unwrap();
    let back = Node::from_toml(&text).unwrap();
    assert_eq!(back, node);
    assert_eq!(back.to_toml().unwrap(), text);
    assert_eq!(back.hash().unwrap(), node.hash().unwrap());

    // Golden: the canonical text and its hash are part of the on-disk contract.
    let g = Node {
        parent: None,
        op: OpRecord::new(&cbox("A")).unwrap(),
        meta: NodeMeta {
            author: "test".into(),
            time: "2026-01-01T00:00:00Z".into(),
            message: None,
        },
    };
    let expected = "format = 1\n\n[meta]\nauthor = \"test\"\ntime = \"2026-01-01T00:00:00Z\"\n\n[op]\nkind = \"model.create_box\"\nversion = 1\n\n[op.params]\nname = \"A\"\nx = 1.0\ny = 2.0\nz = 0.5\n";
    assert_eq!(g.to_toml().unwrap(), expected);
    assert_eq!(g.hash().unwrap().to_hex(), GOLDEN_HASH);

    // Every stored node file round-trips and matches its file name.
    let mut ws = Workspace::open(registry(), MemStore::new()).unwrap();
    build(&mut ws);
    for (path, text) in ws.store().files() {
        if let Some(name) = path.strip_prefix("timeline/ops/") {
            let n = Node::from_toml(text).unwrap();
            assert_eq!(n.to_toml().unwrap(), *text);
            assert_eq!(format!("{}.toml", n.hash().unwrap()), name);
        }
    }
    assert!(ws.store().files().contains_key("timeline/refs/heads/main"));
    assert_eq!(ws.store().files()["timeline/HEAD"], "ref: heads/main\n");
}

const GOLDEN_HASH: &str = "13a8e5b6b3b602c5c666e3853afec50f18a07f2818055bdcd41b92f21a51525d";

#[test]
fn schema_generation_for_ops() {
    let r = registry();
    let kinds: Vec<_> = r.op_kinds().collect();
    assert_eq!(
        kinds,
        [
            "mesh.generate",
            "model.create_box",
            "model.rename",
            "partition.split",
            "test.fail_after_write"
        ]
    );
    let schema = r.op_schema("model.create_box").unwrap().unwrap();
    let props = schema.get("properties").unwrap();
    for f in ["name", "x", "y", "z"] {
        assert!(props.get(f).is_some(), "missing property {f}");
    }
    assert_eq!(
        props.get("x").unwrap().get("description").unwrap().as_str(),
        Some("Extent along x.")
    );
    assert_eq!(
        schema.get("description").unwrap().as_str(),
        Some("Create an axis-aligned box: one body and its volume.")
    );
    let rename = r.op_schema("model.rename").unwrap().unwrap();
    assert!(
        rename.to_json().contains("\"integer\""),
        "Eid is an integer in the schema"
    );
    // Deterministic text, usable as MCP tool input schema.
    let a = r.catalog().unwrap().to_json();
    assert_eq!(a, registry().catalog().unwrap().to_json());
    assert!(a.contains("\"version\":2"));
}

#[test]
fn op_version_migration() {
    let mut ws = Workspace::open(registry(), MemStore::new()).unwrap();
    let r = ws.apply(&cbox("A"), meta(0)).unwrap();
    let id = r.created[0];
    // A v1 record from an old project file used `title`.
    let old = OpRecord {
        kind: "model.rename".into(),
        version: 1,
        params: from_value(&to_value(&serde_json_like(id, "Old")).unwrap()).unwrap(),
    };
    ws.apply_record(old, meta(1), &ApplyOptions::default())
        .unwrap();
    assert_eq!(
        ws.state()
            .layer::<Model>(LayerId::MODEL)
            .unwrap()
            .get(id)
            .unwrap()
            .name,
        "Old"
    );
    // Newer than supported is refused.
    let future = OpRecord {
        kind: "model.rename".into(),
        version: 3,
        params: Value::Table(Table::new()),
    };
    let err = ws
        .apply_record(future, meta(2), &ApplyOptions::default())
        .unwrap_err();
    assert!(matches!(
        err,
        TimelineError::Apply(ApplyError::UnsupportedVersion { .. })
    ));
    let unknown = OpRecord {
        kind: "nope".into(),
        version: 1,
        params: Value::Table(Table::new()),
    };
    assert!(matches!(
        ws.apply_record(unknown, meta(3), &ApplyOptions::default()),
        Err(TimelineError::Apply(ApplyError::UnknownOp(_)))
    ));
}

#[derive(Serialize)]
struct RenameV1 {
    target: Eid,
    title: String,
}

fn serde_json_like(id: EntityId, title: &str) -> RenameV1 {
    RenameV1 {
        target: id.into(),
        title: title.into(),
    }
}

#[derive(Default)]
struct Recorder(Mutex<Vec<(f64, String)>>);

impl ProgressSink for Recorder {
    fn report(&self, fraction: f64, message: &str) {
        self.0.lock().unwrap().push((fraction, message.to_owned()));
    }
}

#[test]
fn progress_and_cancellation() {
    let mut ws = Workspace::open(registry(), MemStore::new()).unwrap();
    let r = ws.apply(&cbox("A"), meta(0)).unwrap();
    let target: Eid = r.created[0].into();

    let rec = Recorder::default();
    let opts = ApplyOptions {
        progress: Some(&rec),
        cancel: CancelToken::new(),
    };
    ws.apply_record(
        OpRecord::new(&Mesh { target, blocks: 4 }).unwrap(),
        meta(1),
        &opts,
    )
    .unwrap();
    let seen = rec.0.lock().unwrap().clone();
    assert_eq!(seen.len(), 5);
    assert!(seen.windows(2).all(|w| w[0].0 <= w[1].0));
    assert_eq!(seen.last().unwrap().0, 1.0);

    // Cancel from inside the progress callback after the second report.
    struct CancelAt(CancelToken, Mutex<u32>);
    impl ProgressSink for CancelAt {
        fn report(&self, _: f64, _: &str) {
            let mut n = self.1.lock().unwrap();
            *n += 1;
            if *n == 2 {
                self.0.cancel();
            }
        }
    }
    let token = CancelToken::new();
    let sink = CancelAt(token.clone(), Mutex::new(0));
    let opts = ApplyOptions {
        progress: Some(&sink),
        cancel: token,
    };
    let (hash, at) = (ws.state_hash(), ws.at());
    let err = ws
        .apply_record(
            OpRecord::new(&Mesh { target, blocks: 10 }).unwrap(),
            meta(2),
            &opts,
        )
        .unwrap_err();
    assert!(matches!(
        err,
        TimelineError::Apply(ApplyError::Op(OpError::Cancelled))
    ));
    assert_eq!((ws.state_hash(), ws.at()), (hash, at));
    assert_eq!(ws.state().ids().last(EntityKind::Mesh), 1); // the cancelled op's id was released
}

#[test]
fn derived_cache_is_keyed_by_content_hash() {
    let mut ws = Workspace::open(registry(), MemStore::new()).unwrap();
    let r = ws.apply(&cbox("A"), meta(0)).unwrap();
    let key = |ws: &Workspace<MemStore>| ws.state().derived_key(&[LayerId::MODEL]);
    let volume = |ws: &Workspace<MemStore>| {
        ws.state().derived().get_or_compute("volume", key(ws), || {
            let m = ws.state().layer::<Model>(LayerId::MODEL).unwrap();
            m.iter()
                .filter(|(_, s)| !s.dims.is_empty())
                .map(|(_, s)| s.dims.iter().product::<f64>())
                .sum::<f64>()
        })
    };
    assert_eq!(*volume(&ws), 1.0);
    assert_eq!(*volume(&ws), 1.0);
    assert_eq!(ws.state().derived().stats(), (1, 1));
    // An op on an unrelated layer keeps the entry valid.
    ws.apply(
        &Partition {
            target: r.created[0].into(),
            parts: 2,
        },
        meta(1),
    )
    .unwrap();
    assert_eq!(*volume(&ws), 1.0);
    assert_eq!(ws.state().derived().stats(), (2, 1));
    // A model change produces a new key.
    ws.apply(&cbox("B"), meta(2)).unwrap();
    assert_eq!(*volume(&ws), 2.0);
    assert_eq!(ws.state().derived().stats(), (2, 2));
    // Derived data is not part of the state hash.
    let h = ws.state_hash();
    ws.state()
        .derived()
        .get_or_compute("other", key(&ws), || 5u32);
    assert_eq!(ws.state_hash(), h);
}

#[test]
fn concurrent_nothing_depends_on_registration_order() {
    let mut r1 = Registry::new();
    r1.install(&ModelConcept).unwrap();
    r1.install(&MeshConcept).unwrap();
    let mut r2 = Registry::new();
    r2.install(&MeshConcept).unwrap();
    r2.install(&ModelConcept).unwrap();
    assert_eq!(r1.new_state().hash(), r2.new_state().hash());
    assert_eq!(r1.fingerprint(), r2.fingerprint());
    assert!(
        r1.install(&ModelConcept).is_err(),
        "duplicate registration is rejected"
    );
}
