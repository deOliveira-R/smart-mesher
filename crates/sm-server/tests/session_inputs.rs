//! Per-session input stores (W3b-SERVER0): sessions stage and replay their own bytes whatever
//! thread builds, opens or drives them.
mod common;

use common::TempDir;
use serde_json::{Value, json};
use sm_op_api::Hash256;
use sm_protocol::{Endpoint, Request};
use sm_server::{Server, standard_with};

const TIME: &str = "2026-10-24T00:00:00Z";

fn builder() -> sm_engine::WorkspaceBuilder {
    standard_with(Some("tester".into()), Some(TIME.into()))
}

/// Two distinct but equally valid STEP byte strings (the second has a trailing newline).
fn step(variant: usize) -> Vec<u8> {
    let mut v = sm_testkit::w3a::STEP.to_vec();
    if variant % 2 == 1 {
        v.push(b'\n');
    }
    v
}

fn import_req(sha: &str) -> Request {
    Request::Apply { op: "model.import_step".into(), params: json!({"source": {"sha256": sha}}), version: None }
}

fn import_staged(s: &mut Server, bytes: &[u8]) -> Hash256 {
    let r = s.stage_input(bytes).expect("stage");
    s.call(import_req(&r.sha256)).unwrap_or_else(|e| panic!("import failed: {e}"));
    s.workspace().state_hash()
}

/// The state hash of a serial, fresh in-memory run importing `bytes`.
fn serial_hash(bytes: &[u8]) -> Hash256 {
    let mut s = Server::new(builder()).expect("server");
    import_staged(&mut s, bytes)
}

#[test]
fn two_servers_on_one_thread_import_their_own_bytes() {
    let (a_bytes, b_bytes) = (step(0), step(1));
    // Both builders exist before either server is made.
    let (ba, bb) = (builder(), builder());
    let mut a = Server::new(ba).expect("a");
    let mut b = Server::new(bb).expect("b");
    let ha = import_staged(&mut a, &a_bytes);
    let hb = import_staged(&mut b, &b_bytes);
    assert_eq!(ha, serial_hash(&a_bytes));
    assert_eq!(hb, serial_hash(&b_bytes));
    // (Equal bytes-independent geometry gives equal state hashes; isolation is checked below.)
    let _ = (ha, hb);
    // A store never serves another session's blob.
    let mut a2 = Server::new(builder()).expect("a2");
    a2.stage_input(&a_bytes).expect("stage");
    assert!(a2.call(import_req(&Hash256::of(&b_bytes).to_hex())).is_err());
}

#[test]
fn eight_threads_open_and_replay_their_own_projects() {
    let dir = TempDir::new("srv0threads");
    let root = dir.path().to_owned();
    let handles: Vec<_> = (0..8usize)
        .map(|i| {
            let root = root.clone();
            std::thread::spawn(move || {
                let bytes = step(i);
                let proj = root.join(format!("p{i}"));
                // An unrelated builder alive on this thread must not matter.
                let _noise = builder();
                let mut s = Server::create(builder(), &proj, "p").expect("create");
                let h = import_staged(&mut s, &bytes);
                s.call(Request::Save).expect("save");
                drop(s);
                let re = Server::open(builder(), &proj).expect("replay in own project");
                assert_eq!(re.workspace().state_hash(), h);
                (bytes, h)
            })
        })
        .collect();
    for h in handles {
        let (bytes, hash) = h.join().expect("thread");
        assert_eq!(hash, serial_hash(&bytes), "equals the serial run");
    }
}

#[test]
fn staged_on_one_thread_applied_on_another() {
    let dir = TempDir::new("srv0xthread");
    let proj = dir.path().join("proj");
    let bytes = step(0);
    // Built and staged on thread A.
    let (mut s, sha) = std::thread::scope(|sc| {
        sc.spawn(|| {
            let s = Server::create(builder(), &proj, "p").expect("create");
            let r = s.stage_input(&bytes).expect("stage");
            (s, r.sha256)
        })
        .join()
        .expect("A")
    });
    // Applied on thread B.
    let h = std::thread::scope(|sc| {
        sc.spawn(|| {
            let v: Value = s.call(import_req(&sha)).unwrap_or_else(|e| panic!("import failed: {e}"));
            assert!(v.is_object());
            s.workspace().state_hash()
        })
        .join()
        .expect("B")
    });
    assert_eq!(h, serial_hash(&bytes));
}
