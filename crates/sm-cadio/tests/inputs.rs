//! The content-addressed input stores (the only file I/O of the crate).

use std::sync::Arc;

use sm_cadio::{DirInputStore, InputStore, MemoryInputStore, StepInputs, sha256_hex, stage_file};

fn tmp(name: &str) -> std::path::PathBuf {
    let d = std::env::temp_dir().join(format!("sm-cadio-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    d
}

#[test]
fn sha256_matches_the_known_digests() {
    assert_eq!(sha256_hex(b""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    assert_eq!(sha256_hex(b"abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    assert_eq!(sha256_hex(sm_testkit::w3a::STEP), sm_testkit::step::W3A_PART.sha256);
}

#[test]
fn memory_store_round_trips_and_deduplicates() {
    let s = MemoryInputStore::new();
    let a = s.put(b"hello");
    assert_eq!(a.bytes, 5);
    assert_eq!(a.sha256, sha256_hex(b"hello"));
    assert_eq!(s.put(b"hello"), a);
    assert_eq!(&*s.get(&a.sha256).expect("stored"), b"hello");
    assert!(s.get(&"0".repeat(64)).is_none());
}

#[test]
fn dir_store_writes_sha256_dot_stp_and_reads_it_back() {
    let dir = tmp("dir");
    let store = DirInputStore::new(&dir);
    assert!(store.get(&"a".repeat(64)).is_none(), "a missing directory is just a miss");
    let r = store.put(sm_testkit::w3a::STEP).expect("put");
    assert_eq!(r.sha256, sm_testkit::step::W3A_PART.sha256);
    assert!(dir.join(format!("{}.stp", r.sha256)).is_file());
    assert_eq!(&*store.get(&r.sha256).expect("get"), sm_testkit::w3a::STEP);
    // Idempotent, and no temporary files are left behind.
    assert_eq!(store.put(sm_testkit::w3a::STEP).expect("put again"), r);
    let names: Vec<String> = std::fs::read_dir(&dir).expect("ls").map(|e| e.expect("entry").file_name().to_string_lossy().into_owned()).collect();
    assert_eq!(names, [format!("{}.stp", r.sha256)]);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn dir_store_refuses_names_that_are_not_hashes() {
    let dir = tmp("traversal");
    std::fs::create_dir_all(&dir).expect("mkdir");
    std::fs::write(dir.join("secret.stp"), b"x").expect("write");
    let store = DirInputStore::new(&dir);
    for bad in ["secret", "../secret", "secret.stp", "", &"A".repeat(64), &"g".repeat(64), &format!("../{}", "a".repeat(61))] {
        assert!(store.get(bad).is_none(), "{bad:?}");
    }
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn stage_file_stages_a_path_and_returns_the_reference_to_record() {
    let dir = tmp("stage");
    let file = tmp("stage-src");
    std::fs::create_dir_all(&file).expect("mkdir");
    let src = file.join("part.step");
    std::fs::write(&src, sm_testkit::step::CYLINDER_R2_H5.bytes).expect("write");
    let store = DirInputStore::new(dir.join("inputs"));
    let r = stage_file(&store, &src).expect("stage");
    assert_eq!(r.sha256, sm_testkit::step::CYLINDER_R2_H5.sha256);
    assert_eq!(r.bytes as usize, sm_testkit::step::CYLINDER_R2_H5.bytes.len());
    assert!(stage_file(&store, &file.join("missing.step")).is_err());
    let svc = StepInputs(Arc::new(store));
    assert!(svc.0.get(&r.sha256).is_some());
    let _ = std::fs::remove_dir_all(&dir);
    let _ = std::fs::remove_dir_all(&file);
}
