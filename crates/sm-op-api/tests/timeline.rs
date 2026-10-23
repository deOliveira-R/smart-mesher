//! Behavioural tests of the reference engine on the sample stack.

use std::sync::atomic::{AtomicU64, Ordering};

use sm_op_api::sample::{IntentOp, MeshOp, ModelOp, PartitionOp, SampleStack};
use sm_op_api::timeline::{HEAD_PATH, NodeBody, node_path, ref_path};
use sm_op_api::{
    CancelToken, Control, Hash256, Id, Meta, OpError, OpOf, Progress, Session, SessionConfig,
    SessionError, StackOp, Timeline, op_schema,
};

type Op = OpOf<SampleStack>;

fn meta(msg: &str) -> Meta {
    Meta {
        author: "test".into(),
        message: msg.into(),
        time: 0,
    }
}

fn brick(x: f64) -> Op {
    StackOp::Model(ModelOp::CreateBrick {
        origin: [x, 0.0, 0.0],
        size: [2.0, 2.0, 2.0],
    })
}

fn session(every: u64) -> Session<SampleStack> {
    Session::new(SessionConfig {
        snapshot_every: every,
    })
}

/// A script touching all four layers; returns node hashes.
fn run_script(s: &mut Session<SampleStack>) -> Vec<Hash256> {
    let mut nodes = Vec::new();
    let mut apply = |s: &mut Session<SampleStack>, op: Op| {
        let a = s.apply(op, meta("step")).unwrap();
        nodes.push(a.node);
        a
    };
    let b1 = apply(s, brick(0.0));
    let (body1, vol1) = (b1.delta.created[0], b1.delta.created[1]);
    let b2 = apply(s, brick(5.0));
    let vol2 = b2.delta.created[1];
    apply(
        s,
        StackOp::Model(ModelOp::Translate {
            targets: vec![body1],
            by: [0.0, 1.0, 0.0],
        }),
    );
    let sp = apply(
        s,
        StackOp::Partition(PartitionOp::Split {
            volume: vol1,
            parts: 2,
        }),
    );
    let cell = sp.delta.created[0];
    apply(
        s,
        StackOp::Intent(IntentOp::SetSize {
            targets: vec![vol1, cell],
            size: 0.5,
        }),
    );
    apply(
        s,
        StackOp::Meshes(MeshOp::Generate {
            volumes: vec![vol1, vol2],
        }),
    );
    nodes
}

#[test]
fn replay_determinism() {
    let mut a = session(3);
    let mut b = session(3);
    let na = run_script(&mut a);
    let nb = run_script(&mut b);
    assert_eq!(na, nb, "node hashes are a pure function of the script");
    assert_eq!(
        a.state().state_hash().unwrap(),
        b.state().state_hash().unwrap()
    );
    let head = *na.last().unwrap();
    let replayed = a.state_at_from_root(&head).unwrap();
    assert_eq!(
        replayed.state_hash().unwrap(),
        a.state().state_hash().unwrap()
    );
    // The mesh really was produced: 1x... brick 2^3 at h=0.5 is 64 cells, the second at h=1.0 is 8.
    let cells: Vec<u64> = a
        .state()
        .meshes()
        .meshes
        .values()
        .map(|m| m.cells)
        .collect();
    assert_eq!(cells, vec![64, 8]);
}

#[test]
fn snapshots_agree_with_replay_from_root() {
    let mut s = session(2);
    let nodes = run_script(&mut s);
    assert!(s.snapshot_count() >= 2);
    for n in &nodes {
        let fast = s.state_at(n).unwrap().state_hash().unwrap();
        let slow = s.state_at_from_root(n).unwrap().state_hash().unwrap();
        assert_eq!(fast, slow);
    }
}

#[test]
fn fork_from_old_node_reassigns_same_ids() {
    let mut s = session(0);
    let nodes = run_script(&mut s);
    let main_tip = *nodes.last().unwrap();
    let main_hash = s.state().state_hash().unwrap();

    // Fork after the first brick: the next brick must get the same ids main gave its second one.
    s.checkout(&nodes[0].to_hex()).unwrap();
    s.branch("alt").unwrap();
    let alt = s.apply(brick(5.0), meta("step")).unwrap();
    assert_eq!(
        alt.node, nodes[1],
        "same parent + same op + same meta is the same node"
    );

    let other = s.apply(brick(9.0), meta("alt 2")).unwrap();
    assert_ne!(other.node, nodes[2]);
    assert_eq!(s.timeline().branch("alt"), Some(other.node));
    assert_eq!(
        s.timeline().branch("main"),
        Some(main_tip),
        "main did not move"
    );

    // Back on main, state is intact.
    s.checkout("main").unwrap();
    assert_eq!(s.state().state_hash().unwrap(), main_hash);
    assert_eq!(s.timeline().children(&nodes[1]).len(), 2);
}

#[test]
fn op_references_ids_created_earlier() {
    let mut s = session(0);
    let a = s.apply(brick(0.0), meta("")).unwrap();
    let ids: Vec<String> = a.delta.created.iter().map(ToString::to_string).collect();
    assert_eq!(ids, ["TopoBody#1", "TopoVolume#1"]);
    // A later op names the id by its stable string form.
    let body: Id = "TopoBody#1".parse().unwrap();
    s.apply(
        StackOp::Model(ModelOp::Delete {
            targets: vec![body],
        }),
        meta(""),
    )
    .unwrap();
    assert!(s.state().model().bodies.is_empty());
    // Ids are never reused.
    let c = s.apply(brick(0.0), meta("")).unwrap();
    assert_eq!(c.delta.created[0].to_string(), "TopoBody#2");
}

#[test]
fn failing_op_changes_nothing() {
    let mut s = session(0);
    let a = s.apply(brick(0.0), meta("")).unwrap();
    let body1 = a.delta.created[0];
    let before = s.state().state_hash().unwrap();
    let nodes_before = s.timeline().len();
    let ghost: Id = "TopoBody#77".parse().unwrap();
    // The first target would be translated before the second fails.
    let err = s
        .apply(
            StackOp::Model(ModelOp::Translate {
                targets: vec![body1, ghost],
                by: [1.0; 3],
            }),
            meta(""),
        )
        .unwrap_err();
    assert_eq!(err, SessionError::Op(OpError::UnknownEntity(ghost)));
    assert_eq!(s.state().state_hash().unwrap(), before);
    assert_eq!(s.timeline().len(), nodes_before);
    assert_eq!(s.state().model().bodies[&body1].origin, [0.0; 3]);
    // Ids allocated inside a failing op are not consumed.
    let err = s
        .apply(
            StackOp::Partition(PartitionOp::Split {
                volume: a.delta.created[1],
                parts: 1,
            }),
            meta(""),
        )
        .unwrap_err();
    assert!(matches!(err, SessionError::Op(OpError::Invalid(_))));
    // Wrong layer order (no volume) is a typed error too.
    let err = s
        .apply(
            StackOp::Meshes(MeshOp::Generate {
                volumes: vec![ghost],
            }),
            meta(""),
        )
        .unwrap_err();
    assert!(matches!(err, SessionError::Op(OpError::UnknownEntity(_))));
    assert_eq!(s.state().state_hash().unwrap(), before);
}

struct CancelAfter {
    token: CancelToken,
    reports: AtomicU64,
    limit: u64,
}

impl Control for CancelAfter {
    fn report(&self, _: &Progress<'_>) {
        if self.reports.fetch_add(1, Ordering::Relaxed) + 1 >= self.limit {
            self.token.cancel();
        }
    }
    fn is_cancelled(&self) -> bool {
        self.token.is_cancelled()
    }
}

#[test]
fn cancellation_leaves_everything_untouched() {
    let mut s = session(0);
    let v1 = s.apply(brick(0.0), meta("")).unwrap().delta.created[1];
    let v2 = s.apply(brick(5.0), meta("")).unwrap().delta.created[1];
    let before = s.state().state_hash().unwrap();
    let n = s.timeline().len();

    let ctl = CancelAfter {
        token: CancelToken::new(),
        reports: AtomicU64::new(0),
        limit: 2,
    };
    let err = s
        .apply_with(
            StackOp::Meshes(MeshOp::Generate {
                volumes: vec![v1, v2],
            }),
            meta(""),
            &ctl,
        )
        .unwrap_err();
    assert_eq!(err, SessionError::Op(OpError::Cancelled));
    assert_eq!(
        ctl.reports.load(Ordering::Relaxed),
        2,
        "stopped at the second checkpoint"
    );
    assert_eq!(s.state().state_hash().unwrap(), before);
    assert_eq!(s.timeline().len(), n);
    assert!(s.state().meshes().meshes.is_empty());

    // Same op, not cancelled, succeeds and gets Mesh#1 (the cancelled attempt consumed nothing).
    let ok = s
        .apply(
            StackOp::Meshes(MeshOp::Generate {
                volumes: vec![v1, v2],
            }),
            meta(""),
        )
        .unwrap();
    assert_eq!(ok.delta.created[0].to_string(), "Mesh#1");
}

fn encode(b: &NodeBody<Op>) -> Result<String, String> {
    toml::to_string(b).map_err(|e| e.to_string())
}
fn decode(t: &str) -> Result<NodeBody<Op>, String> {
    toml::from_str(t).map_err(|e| e.to_string())
}

#[test]
fn toml_roundtrip_keeps_hashes_stable() {
    let mut s = session(0);
    let nodes = run_script(&mut s);
    let files = s.timeline().to_files(encode).unwrap();

    // One file per node, plus main and HEAD.
    assert_eq!(files.len(), nodes.len() + 2);
    let head_text = &files.iter().find(|(p, _)| p == HEAD_PATH).unwrap().1;
    assert_eq!(head_text, "ref: refs/heads/main\n");
    let node_text = &files
        .iter()
        .find(|(p, _)| *p == node_path(&nodes[3]))
        .unwrap()
        .1;
    println!("{node_text}");
    assert!(node_text.contains("layer = \"partition\""));
    assert!(node_text.contains(&format!("parent = \"{}\"", nodes[2])));

    let loaded: Timeline<Op> = Timeline::from_files(files.clone(), decode).unwrap();
    assert_eq!(loaded.len(), nodes.len());
    for n in &nodes {
        assert_eq!(loaded.node(n).unwrap(), s.timeline().node(n).unwrap());
    }
    assert_eq!(loaded.branch("main"), nodes.last().copied());
    // Re-serializing the loaded timeline is byte-identical.
    assert_eq!(loaded.to_files(encode).unwrap(), files);

    // A loaded timeline replays to the same state.
    let s2 = Session::<SampleStack>::from_timeline(loaded, SessionConfig::default()).unwrap();
    assert_eq!(
        s2.state().state_hash().unwrap(),
        s.state().state_hash().unwrap()
    );
}

#[test]
fn hash_ignores_toml_formatting_and_key_order() {
    let mut s = session(0);
    let n = s.apply(brick(1.5), meta("hello")).unwrap().node;
    let text = encode(s.timeline().node(&n).unwrap().body()).unwrap();
    // Reorder the lines inside the file and add comments: the parsed body is the same.
    let mut lines: Vec<&str> = text.lines().collect();
    lines.reverse();
    let scrambled = format!("# comment\n{}\n", lines.join("\n"));
    if let Ok(body) = decode(&scrambled) {
        assert_eq!(sm_op_api::Node::new(body).unwrap().hash(), n);
    }
    let spaced = text.replace(" = ", "   =   ");
    assert_eq!(
        sm_op_api::Node::new(decode(&spaced).unwrap())
            .unwrap()
            .hash(),
        n
    );
}

#[test]
fn corrupted_node_file_is_detected() {
    let mut s = session(0);
    s.apply(brick(1.0), meta("a")).unwrap();
    let mut files = s.timeline().to_files(encode).unwrap();
    for (p, t) in &mut files {
        if p.starts_with("timeline/ops/") {
            *t = t.replace("1.0", "1.25");
        }
    }
    assert!(Timeline::<Op>::from_files(files, decode).is_err());
}

#[test]
fn node_files_on_disk_layout() {
    let mut s = session(0);
    let nodes = run_script(&mut s);
    s.checkout(&nodes[1].to_hex()).unwrap();
    s.branch("try-pave").unwrap();
    s.apply(brick(7.0), meta("exp")).unwrap();

    let root = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("timeline-layout");
    let _ = std::fs::remove_dir_all(&root);
    for (p, t) in s.timeline().to_files(encode).unwrap() {
        let path = root.join(p);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, t).unwrap();
    }
    assert!(root.join(node_path(&nodes[0])).is_file());
    assert!(root.join(ref_path("try-pave")).is_file());
    assert_eq!(
        std::fs::read_to_string(root.join(HEAD_PATH)).unwrap(),
        "ref: refs/heads/try-pave\n"
    );

    let mut files = Vec::new();
    let mut stack = vec![root.join("timeline")];
    while let Some(d) = stack.pop() {
        for e in std::fs::read_dir(d).unwrap() {
            let p = e.unwrap().path();
            if p.is_dir() {
                stack.push(p);
            } else {
                let rel = p
                    .strip_prefix(&root)
                    .unwrap()
                    .to_string_lossy()
                    .replace('\\', "/");
                files.push((rel, std::fs::read_to_string(&p).unwrap()));
            }
        }
    }
    let loaded = Timeline::<Op>::from_files(files, decode).unwrap();
    assert_eq!(loaded.len(), s.timeline().len());
    assert_eq!(loaded.branch("try-pave"), s.timeline().branch("try-pave"));
    assert_eq!(loaded.branch("main"), nodes.last().copied());
}

#[test]
fn golden_node_hash() {
    // Pins the canonical encoding: if this changes, every stored timeline changes identity.
    let body = NodeBody {
        format: 1,
        parent: None,
        op_version: 1,
        meta: Meta {
            author: "golden".into(),
            message: String::new(),
            time: 7,
        },
        op: brick(0.0),
    };
    let h = sm_op_api::Node::new(body).unwrap().hash();
    assert_eq!(h.to_hex(), GOLDEN);
}

const GOLDEN: &str = "e2fa3bfe9f34e5453495fbe680e0a2618c2ad1dfac7cf9572975946ca7c1a8d2";

#[test]
fn schema_of_sample_op() {
    let schema = serde_json::to_value(op_schema::<SampleStack>()).unwrap();
    let text = schema.to_string();
    for name in [
        "CreateBrick",
        "Translate",
        "Split",
        "SetSize",
        "Generate",
        "EntityId",
    ] {
        assert!(text.contains(name), "schema lacks {name}");
    }
    assert!(text.contains("\"layer\"") && text.contains("\"args\""));
    let one = serde_json::to_value(schemars::schema_for!(ModelOp)).unwrap();
    assert!(one.to_string().contains("\"kind\""));
}

#[test]
fn derived_cache_is_keyed_by_content_and_outside_the_hash() {
    let mut s = session(0);
    let v = s.apply(brick(0.0), meta("")).unwrap().delta.created[1];
    s.apply(
        StackOp::Meshes(MeshOp::Generate { volumes: vec![v] }),
        meta(""),
    )
    .unwrap();
    let calls = AtomicU64::new(0);
    let total = |st: &sm_op_api::State<SampleStack>| {
        st.derived_or_compute("cell-total", &["meshes"], |st| {
            calls.fetch_add(1, Ordering::Relaxed);
            Ok::<u64, OpError>(st.meshes().meshes.values().map(|m| m.cells).sum())
        })
    };
    let h0 = s.state().state_hash().unwrap();
    assert_eq!(*total(s.state()).unwrap(), 8);
    assert_eq!(*total(s.state()).unwrap(), 8);
    assert_eq!(calls.load(Ordering::Relaxed), 1);
    assert_eq!(
        s.state().state_hash().unwrap(),
        h0,
        "cache is outside the state hash"
    );

    // An unrelated layer change keeps the entry valid; a meshes change invalidates it.
    s.apply(
        StackOp::Model(ModelOp::Translate {
            targets: vec!["TopoBody#1".parse().unwrap()],
            by: [1.0; 3],
        }),
        meta(""),
    )
    .unwrap();
    total(s.state()).unwrap();
    assert_eq!(calls.load(Ordering::Relaxed), 1);
    s.apply(
        StackOp::Meshes(MeshOp::Generate { volumes: vec![v] }),
        meta(""),
    )
    .unwrap();
    assert_eq!(*total(s.state()).unwrap(), 16);
    assert_eq!(calls.load(Ordering::Relaxed), 2);
    // Checking out an older node shares the same cache.
    let old = s
        .timeline()
        .path_to(&s.timeline().resolve("HEAD").unwrap())
        .unwrap()[1];
    s.checkout(&old.to_hex()).unwrap();
    assert_eq!(s.state().derived().len(), 2);
}
