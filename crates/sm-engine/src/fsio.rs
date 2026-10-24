//! Crash-safe file primitives: atomic replace, durable directory entries, the project write lock
//! and a fault injector for crash tests.

use std::fs::{self, File, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};

use crate::error::{EngineError, Result};

/// How hard writes try to survive power loss.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Durability {
    /// `fsync` every file before its rename and the directory after it (default). A node file
    /// is durable before any ref points at it, so a power cut never leaves a dangling ref.
    #[default]
    Full,
    /// Atomic rename without `fsync`. A process crash is still safe; a power cut can leave a
    /// ref whose node never reached the disk. For tests, benchmarks and throw-away projects.
    Fast,
}

/// Crash simulation for tests: counts the instrumented points of every atomic write and delete
/// and fails at the chosen one, leaving on disk exactly what a killed process would leave
/// (a half-written temp file, a complete temp file, or a renamed file without the directory
/// sync). Install with [`WorkspaceBuilder::fault`](crate::WorkspaceBuilder::fault).
///
/// Each atomic write has three points (`torn temp`, `complete temp`, `renamed`); each delete has
/// one. Run once with [`FaultInjector::counting`] to learn how many points a scenario has, then
/// once per point with [`FaultInjector::fail_at`]. Writes run sequentially while an injector is
/// installed, so the point numbering is deterministic.
#[derive(Debug)]
pub struct FaultInjector {
    fail_at: usize,
    seen: AtomicUsize,
}

impl FaultInjector {
    /// Never fails; only counts points.
    pub fn counting() -> std::sync::Arc<Self> {
        Self::fail_at(usize::MAX)
    }

    /// Fails when the `n`-th point (0-based) is reached.
    pub fn fail_at(n: usize) -> std::sync::Arc<Self> {
        std::sync::Arc::new(Self {
            fail_at: n,
            seen: AtomicUsize::new(0),
        })
    }

    /// Points passed so far (including the failing one).
    pub fn points(&self) -> usize {
        self.seen.load(Ordering::SeqCst)
    }

    /// True once the injected failure happened.
    pub fn tripped(&self) -> bool {
        self.points() > self.fail_at
    }

    pub(crate) fn hit(&self, what: &str) -> io::Result<()> {
        let i = self.seen.fetch_add(1, Ordering::SeqCst);
        if i == self.fail_at {
            return Err(io::Error::other(format!(
                "injected crash at point {i} ({what})"
            )));
        }
        Ok(())
    }
}

pub(crate) fn hit(fault: Option<&FaultInjector>, what: &str) -> io::Result<()> {
    fault.map_or(Ok(()), |f| f.hit(what))
}

static TMP_SEQ: AtomicU64 = AtomicU64::new(0);

/// Suffix of temp files. A ref name may not end in `.lock`, so a temp file can never be
/// mistaken for a ref, and node files are matched by `<hash>.toml`.
pub(crate) const TMP_SUFFIX: &str = ".lock";

fn tmp_path(path: &Path) -> PathBuf {
    let name = path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    let n = TMP_SEQ.fetch_add(1, Ordering::Relaxed);
    path.with_file_name(format!("{name}.{}-{n}{TMP_SUFFIX}", std::process::id()))
}

fn sync_dir(dir: &Path) {
    // Directory sync is best-effort: unsupported on some platforms.
    if let Ok(d) = File::open(dir) {
        let _ = d.sync_all();
    }
}

/// Write `data` to `path` so that readers (and a crash at any moment) see either the old file or
/// the complete new one: temp file in the same directory, optional `fsync`, rename, directory sync.
pub(crate) fn atomic_write(
    path: &Path,
    data: &[u8],
    dur: Durability,
    fault: Option<&FaultInjector>,
) -> io::Result<()> {
    let parent = path
        .parent()
        .ok_or_else(|| io::Error::other("path has no parent directory"))?;
    fs::create_dir_all(parent)?;
    let tmp = tmp_path(path);
    let mut f = File::create(&tmp)?;
    let (a, b) = data.split_at(data.len() / 2);
    f.write_all(a)?;
    hit(fault, "torn temp")?;
    f.write_all(b)?;
    if dur == Durability::Full {
        f.sync_all()?;
    }
    drop(f);
    hit(fault, "complete temp")?;
    if let Err(e) = fs::rename(&tmp, path) {
        let _ = fs::remove_file(&tmp);
        return Err(e);
    }
    hit(fault, "renamed")?;
    if dur == Durability::Full {
        sync_dir(parent);
    }
    Ok(())
}

/// Remove a file; a missing file is fine.
pub(crate) fn remove_file(path: &Path, fault: Option<&FaultInjector>) -> io::Result<()> {
    hit(fault, "delete")?;
    match fs::remove_file(path) {
        Err(e) if e.kind() != io::ErrorKind::NotFound => Err(e),
        _ => Ok(()),
    }
}

/// Remove empty directories from `dir` upwards, stopping at (and keeping) `stop`.
pub(crate) fn prune_empty_dirs(mut dir: &Path, stop: &Path) {
    while dir != stop && dir.starts_with(stop) {
        if fs::remove_dir(dir).is_err() {
            break;
        }
        match dir.parent() {
            Some(p) => dir = p,
            None => break,
        }
    }
}

/// Delete stray temp files left by a killed writer under `dir` (recursively). Safe because the
/// caller holds the project lock.
pub(crate) fn sweep_tmp(dir: &Path) -> io::Result<usize> {
    let mut n = 0;
    let rd = match fs::read_dir(dir) {
        Ok(rd) => rd,
        Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(0),
        Err(e) => return Err(e),
    };
    for entry in rd {
        let entry = entry?;
        let p = entry.path();
        if entry.file_type()?.is_dir() {
            n += sweep_tmp(&p)?;
        } else if p.to_string_lossy().ends_with(TMP_SUFFIX) && fs::remove_file(&p).is_ok() {
            n += 1;
        }
    }
    Ok(n)
}

const LOCK_ATTEMPTS: u32 = 25;
const LOCK_RETRY: std::time::Duration = std::time::Duration::from_millis(4);

/// Exclusive advisory lock on `<cache>/lock`, released on drop.
#[derive(Debug)]
pub(crate) struct ProjectLock {
    _file: Option<File>,
}

impl ProjectLock {
    pub(crate) fn acquire(root: &Path, cache: &Path) -> Result<Self> {
        fs::create_dir_all(cache).map_err(|e| EngineError::io(cache, e))?;
        let path = cache.join("lock");
        let f = OpenOptions::new()
            .create(true)
            .truncate(false)
            .write(true)
            .open(&path)
            .map_err(|e| EngineError::io(&path, e))?;
        // A lock that was just released can still look held for a moment (a child process
        // forked by another thread holds a copy of the descriptor until it execs), so wait
        // briefly before reporting contention.
        for attempt in 0..LOCK_ATTEMPTS {
            match f.try_lock() {
                Ok(()) => return Ok(Self { _file: Some(f) }),
                Err(fs::TryLockError::WouldBlock) => {
                    if attempt + 1 < LOCK_ATTEMPTS {
                        std::thread::sleep(LOCK_RETRY);
                    }
                }
                // File systems without locking: proceed unlocked rather than refuse to open.
                Err(fs::TryLockError::Error(e)) if e.kind() == io::ErrorKind::Unsupported => {
                    return Ok(Self { _file: None });
                }
                Err(fs::TryLockError::Error(e)) => return Err(EngineError::io(&path, e)),
            }
        }
        Err(EngineError::Locked(root.to_owned()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dir(tag: &str) -> PathBuf {
        let p = std::env::temp_dir().join(format!("sm-engine-unit-{tag}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&p);
        fs::create_dir_all(&p).unwrap();
        p
    }

    fn files(d: &Path) -> Vec<String> {
        let mut v: Vec<String> = fs::read_dir(d)
            .unwrap()
            .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
            .collect();
        v.sort();
        v
    }

    #[test]
    fn atomic_write_replaces_and_leaves_no_temp_file() {
        let d = dir("replace");
        let f = d.join("a.txt");
        atomic_write(&f, b"one", Durability::Full, None).unwrap();
        atomic_write(&f, b"two two", Durability::Fast, None).unwrap();
        assert_eq!(fs::read(&f).unwrap(), b"two two");
        assert_eq!(files(&d), ["a.txt"]);
        atomic_write(&d.join("sub/dir/b.txt"), b"", Durability::Fast, None).unwrap();
        assert!(d.join("sub/dir/b.txt").is_file());
        fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn a_crash_at_every_point_leaves_old_or_new_content_never_a_mixture() {
        for point in 0..3 {
            let d = dir(&format!("crash{point}"));
            let f = d.join("a.txt");
            atomic_write(&f, b"old old old", Durability::Fast, None).unwrap();
            let inj = FaultInjector::fail_at(point);
            let r = atomic_write(&f, b"brand new content", Durability::Fast, Some(&inj));
            assert!(r.is_err() && inj.tripped());
            let now = fs::read(&f).unwrap();
            let expect: &[u8] = if point == 2 {
                b"brand new content"
            } else {
                b"old old old"
            };
            assert_eq!(now, expect, "point {point}");
            // The dead writer's temp file is recognizable and swept.
            assert_eq!(sweep_tmp(&d).unwrap(), usize::from(point < 2));
            assert_eq!(files(&d), ["a.txt"]);
            fs::remove_dir_all(d).unwrap();
        }
    }

    #[test]
    fn counting_injector_never_fails() {
        let d = dir("count");
        let inj = FaultInjector::counting();
        atomic_write(&d.join("x"), b"x", Durability::Fast, Some(&inj)).unwrap();
        remove_file(&d.join("x"), Some(&inj)).unwrap();
        remove_file(&d.join("x"), Some(&inj)).unwrap(); // missing is fine
        assert_eq!(inj.points(), 3 + 1 + 1);
        assert!(!inj.tripped());
        fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn lock_is_exclusive_and_released_on_drop() {
        let d = dir("lock");
        let cache = d.join(".cache");
        let a = ProjectLock::acquire(&d, &cache).unwrap();
        assert!(matches!(
            ProjectLock::acquire(&d, &cache),
            Err(EngineError::Locked(_))
        ));
        drop(a);
        ProjectLock::acquire(&d, &cache).unwrap();
        fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn prune_empty_dirs_stops_at_the_boundary() {
        let d = dir("prune");
        fs::create_dir_all(d.join("heads/a/b")).unwrap();
        prune_empty_dirs(&d.join("heads/a/b"), &d.join("heads"));
        assert!(d.join("heads").is_dir() && !d.join("heads/a").exists());
        fs::remove_dir_all(d).unwrap();
    }
}
