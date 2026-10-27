//! Content-addressed input blobs: the only file I/O of the crate.
//!
//! The import op never takes a path. A STEP file is named by its sha256 and its bytes come from
//! the injected [`StepInputs`] service (or are inlined in the op). [`MemoryInputStore`] serves
//! tests and `--memory` projects, [`DirInputStore`] a project's `inputs/` directory
//! (`<sha256>.stp`).

use std::collections::BTreeMap;
use std::io;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Lowercase hex sha256 of `bytes`.
pub fn sha256_hex(bytes: &[u8]) -> String {
    let d = Sha256::digest(bytes);
    let mut s = String::with_capacity(64);
    for b in d {
        s.push(char::from_digit(u32::from(b >> 4), 16).unwrap_or('0'));
        s.push(char::from_digit(u32::from(b & 15), 16).unwrap_or('0'));
    }
    s
}

/// Whether `s` is 64 lowercase hex digits.
pub(crate) fn is_sha256_hex(s: &str) -> bool {
    s.len() == 64 && s.bytes().all(|c| c.is_ascii_digit() || (b'a'..=b'f').contains(&c))
}

/// A staged input: its hash (what ops record) and its size.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct InputRef {
    /// Lowercase hex sha256 of the bytes.
    pub sha256: String,
    /// Size in bytes.
    pub bytes: u64,
}

/// Read access to staged inputs by content hash.
pub trait InputStore: Send + Sync {
    /// The bytes whose sha256 is `sha256` (64 lowercase hex digits), if staged. The caller
    /// re-hashes them, so a store need not verify.
    fn get(&self, sha256: &str) -> Option<Arc<[u8]>>;
}

/// The injected service the import op reads `{sha256}` sources through.
///
/// Unversioned: it is a pure content-addressed lookup, so replay depends only on the bytes.
#[derive(Clone)]
pub struct StepInputs(pub Arc<dyn InputStore>);

impl std::fmt::Debug for StepInputs {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("StepInputs(..)")
    }
}

/// An in-memory store.
#[derive(Debug, Default)]
pub struct MemoryInputStore {
    blobs: Mutex<BTreeMap<String, Arc<[u8]>>>,
}

impl MemoryInputStore {
    /// An empty store.
    pub fn new() -> Self {
        Self::default()
    }

    /// Store `bytes` and return their reference.
    pub fn put(&self, bytes: &[u8]) -> InputRef {
        let sha = sha256_hex(bytes);
        let mut g = self.blobs.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
        g.entry(sha.clone()).or_insert_with(|| Arc::from(bytes));
        InputRef { sha256: sha, bytes: bytes.len() as u64 }
    }

    /// Insert `bytes` under an arbitrary key without checking the hash (tests of the integrity
    /// check).
    pub fn put_unchecked(&self, key: &str, bytes: &[u8]) {
        let mut g = self.blobs.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
        g.insert(key.to_owned(), Arc::from(bytes));
    }
}

impl InputStore for MemoryInputStore {
    fn get(&self, sha256: &str) -> Option<Arc<[u8]>> {
        self.blobs.lock().unwrap_or_else(std::sync::PoisonError::into_inner).get(sha256).cloned()
    }
}

/// A directory store: blob `<sha256>` lives in `<root>/<sha256>.stp`.
#[derive(Clone, Debug)]
pub struct DirInputStore {
    root: PathBuf,
}

impl DirInputStore {
    /// A store rooted at `root` (created on the first `put`).
    pub fn new(root: impl Into<PathBuf>) -> Self {
        Self { root: root.into() }
    }

    /// The root directory.
    pub fn root(&self) -> &Path {
        &self.root
    }

    fn path_of(&self, sha256: &str) -> PathBuf {
        self.root.join(format!("{sha256}.stp"))
    }

    /// Store `bytes` (written to a temporary file, then renamed, so a reader never sees a
    /// partial blob) and return their reference.
    pub fn put(&self, bytes: &[u8]) -> io::Result<InputRef> {
        let sha = sha256_hex(bytes);
        std::fs::create_dir_all(&self.root)?;
        let path = self.path_of(&sha);
        if !path.exists() {
            let tmp = self.root.join(format!("{sha}.tmp"));
            std::fs::write(&tmp, bytes)?;
            std::fs::rename(&tmp, &path)?;
        }
        Ok(InputRef { sha256: sha, bytes: bytes.len() as u64 })
    }
}

impl InputStore for DirInputStore {
    fn get(&self, sha256: &str) -> Option<Arc<[u8]>> {
        // Only a well-formed hash can name a file: no path separators or `..` get through.
        if !is_sha256_hex(sha256) {
            return None;
        }
        std::fs::read(self.path_of(sha256)).ok().map(Arc::from)
    }
}

/// Read `path` and stage its bytes in `store`. Front ends call this and record the returned
/// `{sha256}` in the op instead of the path.
pub fn stage_file(store: &DirInputStore, path: &Path) -> io::Result<InputRef> {
    let bytes = std::fs::read(path)?;
    store.put(&bytes)
}
