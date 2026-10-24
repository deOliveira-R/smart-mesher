//! Project directory layout and whole-timeline save/load.
//!
//! ```text
//! <root>/project.toml
//! <root>/.gitignore                 (ignores .cache/)
//! <root>/timeline/ops/<hash>.toml   content-addressed nodes (immutable)
//! <root>/timeline/refs/heads/<name> one file per branch, "<hash>\n"
//! <root>/.cache/                    HEAD, reflog.jsonl, redo.json, snapshots/, lock (gitignored)
//! ```
//!
//! The text of every node and ref comes from `Timeline::node_file` / `ref_file`, so what is
//! written here is exactly what `Timeline::to_files` produces.

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use rayon::prelude::*;
use sm_op_api::{Hash256, Timeline, valid_ref_name};

use crate::error::{EngineError, Result};
use crate::fsio::{self, Durability, FaultInjector, TMP_SUFFIX};

/// Below this many files, writes and reads stay on the calling thread.
const PAR_MIN: usize = 64;

/// Names of OS metadata files that may appear inside the timeline and are not ours.
const JUNK: &[&str] = &[".DS_Store", "Thumbs.db", "desktop.ini"];

/// Paths of a project directory.
#[derive(Clone, Debug)]
pub struct Layout {
    root: PathBuf,
}

impl Layout {
    /// Layout rooted at `root`.
    pub fn new(root: impl Into<PathBuf>) -> Self {
        Self { root: root.into() }
    }
    /// The project directory.
    pub fn root(&self) -> &Path {
        &self.root
    }
    /// `project.toml`.
    pub fn project_toml(&self) -> PathBuf {
        self.root.join("project.toml")
    }
    /// `.gitignore` at the project root.
    pub fn gitignore(&self) -> PathBuf {
        self.root.join(".gitignore")
    }
    /// `timeline/`.
    pub fn timeline_dir(&self) -> PathBuf {
        self.root.join("timeline")
    }
    /// `.cache/` (derived, gitignored, per checkout).
    pub fn cache_dir(&self) -> PathBuf {
        self.root.join(".cache")
    }
    /// Path of a timeline file given its timeline-relative name (`ops/<hash>.toml`,
    /// `refs/heads/<name>`), as produced by `Timeline::node_file` / `ref_file`.
    pub fn timeline_file(&self, rel: &str) -> PathBuf {
        let mut p = self.timeline_dir();
        p.extend(rel.split('/'));
        p
    }
    pub(crate) fn heads_dir(&self) -> PathBuf {
        self.timeline_dir().join("refs").join("heads")
    }
    pub(crate) fn ops_dir(&self) -> PathBuf {
        self.timeline_dir().join("ops")
    }
}

/// What [`load_timeline`] skipped or noticed.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct LoadReport {
    /// Files that were ignored (stray temp files, OS junk, unrecognized names), as paths
    /// relative to `timeline/`.
    pub ignored: Vec<String>,
    /// Node files read.
    pub nodes: usize,
    /// Ref files read.
    pub refs: usize,
}

fn walk(dir: &Path, rel: &str, out: &mut Vec<(String, PathBuf)>) -> io::Result<()> {
    let rd = match fs::read_dir(dir) {
        Ok(rd) => rd,
        Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(()),
        Err(e) => return Err(e),
    };
    let mut entries = rd.collect::<io::Result<Vec<_>>>()?;
    entries.sort_by_key(|e| e.file_name());
    for e in entries {
        let name = e.file_name().to_string_lossy().into_owned();
        let rel = if rel.is_empty() {
            name
        } else {
            format!("{rel}/{name}")
        };
        if e.file_type()?.is_dir() {
            walk(&e.path(), &rel, out)?;
        } else {
            out.push((rel, e.path()));
        }
    }
    Ok(())
}

/// Read `timeline/` into the path-to-text map `Timeline::from_files` expects. Files are read in
/// parallel; the map is ordered, so the result does not depend on scheduling.
pub fn read_timeline_files(layout: &Layout) -> Result<(BTreeMap<String, String>, LoadReport)> {
    let mut report = LoadReport::default();
    let mut wanted: Vec<(String, PathBuf)> = Vec::new();
    for (sub, prefix) in [
        (layout.ops_dir(), "ops"),
        (layout.heads_dir(), "refs/heads"),
    ] {
        let mut found = Vec::new();
        walk(&sub, prefix, &mut found).map_err(|e| EngineError::io(&sub, e))?;
        for (rel, path) in found {
            let leaf = rel.rsplit('/').next().unwrap_or(&rel);
            let keep = if prefix == "ops" {
                rel.strip_prefix("ops/")
                    .and_then(|f| f.strip_suffix(".toml"))
                    .is_some_and(|h| h.parse::<Hash256>().is_ok())
            } else {
                !leaf.ends_with(TMP_SUFFIX)
                    && !JUNK.contains(&leaf)
                    && rel.strip_prefix("refs/heads/").is_some_and(valid_ref_name)
            };
            if keep {
                wanted.push((rel, path));
            } else {
                report.ignored.push(rel);
            }
        }
    }
    let read = |(rel, path): &(String, PathBuf)| -> Result<(String, String)> {
        let text = fs::read_to_string(path).map_err(|e| EngineError::io(path, e))?;
        Ok((rel.clone(), text))
    };
    let results: Vec<Result<(String, String)>> = if wanted.len() >= PAR_MIN {
        wanted.par_iter().map(read).collect()
    } else {
        wanted.iter().map(read).collect()
    };
    let mut files = BTreeMap::new();
    for r in results {
        let (rel, text) = r?; // first error in path order, whatever the scheduling
        if rel.starts_with("ops/") {
            report.nodes += 1;
        } else {
            report.refs += 1;
        }
        files.insert(rel, text);
    }
    Ok((files, report))
}

/// Load the timeline of a project directory, verifying every node's hash against its file name.
pub fn load_timeline(layout: &Layout) -> Result<(Timeline, LoadReport)> {
    let (files, report) = read_timeline_files(layout)?;
    Ok((Timeline::from_files(&files)?, report))
}

/// Knobs of the write paths.
#[derive(Clone, Copy, Default)]
pub(crate) struct WriteCtx<'a> {
    pub(crate) durability: Durability,
    pub(crate) fault: Option<&'a FaultInjector>,
}

impl WriteCtx<'_> {
    pub(crate) fn write(&self, path: &Path, data: &[u8]) -> Result<()> {
        fsio::atomic_write(path, data, self.durability, self.fault)
            .map_err(|e| EngineError::io(path, e))
    }
    pub(crate) fn remove(&self, path: &Path) -> Result<()> {
        fsio::remove_file(path, self.fault).map_err(|e| EngineError::io(path, e))
    }
}

/// Write the node files `hashes` (always replacing an existing file). Parallel when there are
/// many and no fault injector is installed; the first failing hash in input order is reported.
pub(crate) fn write_nodes(
    layout: &Layout,
    tl: &Timeline,
    hashes: &[Hash256],
    ctx: WriteCtx<'_>,
    mut done: impl FnMut(Hash256),
) -> Result<()> {
    let one = |h: &Hash256| -> Result<()> {
        let (rel, text) = tl.node_file(*h)?;
        ctx.write(&layout.timeline_file(&rel), text.as_bytes())
    };
    if hashes.len() >= PAR_MIN && ctx.fault.is_none() {
        let results: Vec<Result<()>> = hashes.par_iter().map(one).collect();
        let mut first_err = None;
        for (h, r) in hashes.iter().zip(results) {
            match r {
                Ok(()) => done(*h),
                Err(e) => {
                    first_err.get_or_insert(e);
                }
            }
        }
        first_err.map_or(Ok(()), Err)
    } else {
        for h in hashes {
            one(h)?;
            done(*h);
        }
        Ok(())
    }
}

fn conflicts(a: &str, b: &str) -> bool {
    let (a, b) = (a.to_ascii_lowercase(), b.to_ascii_lowercase());
    a == b || a.starts_with(&format!("{b}/")) || b.starts_with(&format!("{a}/"))
}

/// Bring the ref files in line with `now`, given what is `on_disk`: refs that clash by
/// file-versus-directory or by case with a ref being written are deleted first, then new and
/// moved refs are written, then `mid` runs (HEAD is written there, so it never names a branch
/// that is already gone), then the remaining deleted refs are removed. A crash between steps
/// leaves refs pointing at existing nodes (the node files were written before this).
pub(crate) fn sync_refs(
    layout: &Layout,
    tl: &Timeline,
    on_disk: &mut BTreeMap<String, Hash256>,
    ctx: WriteCtx<'_>,
    mid: &mut dyn FnMut() -> Result<()>,
) -> Result<()> {
    let now = tl.refs();
    let changed: Vec<&String> = now
        .iter()
        .filter(|(n, h)| on_disk.get(*n) != Some(h))
        .map(|(n, _)| n)
        .collect();
    let gone: Vec<String> = on_disk
        .keys()
        .filter(|n| !now.contains_key(*n))
        .cloned()
        .collect();
    let remove = |name: &str, on_disk: &mut BTreeMap<String, Hash256>| -> Result<()> {
        let path = layout.timeline_file(&format!("refs/heads/{name}"));
        ctx.remove(&path)?;
        if let Some(p) = path.parent() {
            fsio::prune_empty_dirs(p, &layout.heads_dir());
        }
        on_disk.remove(name);
        Ok(())
    };
    let (early, late): (Vec<String>, Vec<String>) = gone
        .into_iter()
        .partition(|g| changed.iter().any(|c| conflicts(g, c)));
    for g in &early {
        remove(g, on_disk)?;
    }
    for name in changed {
        if let Some((rel, text)) = tl.ref_file(name) {
            ctx.write(&layout.timeline_file(&rel), text.as_bytes())?;
            on_disk.insert(name.clone(), now[name]);
        }
    }
    mid()?;
    for g in &late {
        remove(g, on_disk)?;
    }
    Ok(())
}

/// Save a whole timeline into a project directory: every node file that does not exist yet (and
/// every one in `rewrite`), then the refs.
pub fn save_timeline(
    layout: &Layout,
    tl: &Timeline,
    rewrite: &[Hash256],
    durability: Durability,
) -> Result<()> {
    let ctx = WriteCtx {
        durability,
        fault: None,
    };
    let rewrite: BTreeSet<Hash256> = rewrite.iter().copied().collect();
    let mut todo: Vec<Hash256> = tl
        .nodes()
        .map(|(h, _)| h)
        .filter(|h| rewrite.contains(h) || !layout.timeline_file(&format!("ops/{h}.toml")).exists())
        .collect();
    todo.sort();
    write_nodes(layout, tl, &todo, ctx, |_| {})?;
    let mut disk = read_refs_on_disk(layout)?;
    sync_refs(layout, tl, &mut disk, ctx, &mut || Ok(()))
}

/// The refs currently stored under `timeline/refs/heads`.
pub(crate) fn read_refs_on_disk(layout: &Layout) -> Result<BTreeMap<String, Hash256>> {
    let mut found = Vec::new();
    walk(&layout.heads_dir(), "", &mut found)
        .map_err(|e| EngineError::io(layout.heads_dir(), e))?;
    let mut out = BTreeMap::new();
    for (rel, path) in found {
        if rel.ends_with(TMP_SUFFIX)
            || JUNK.contains(&rel.rsplit('/').next().unwrap_or(""))
            || !valid_ref_name(&rel)
        {
            continue;
        }
        let text = fs::read_to_string(&path).map_err(|e| EngineError::io(&path, e))?;
        if let Ok(h) = text.trim().parse::<Hash256>() {
            out.insert(rel, h);
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conflict_rule_covers_case_and_file_directory_clashes() {
        assert!(conflicts("a", "a/b"));
        assert!(conflicts("a/b", "a"));
        assert!(conflicts("Feat", "feat"));
        assert!(conflicts("Feat/x", "feat"));
        assert!(!conflicts("a", "ab"));
        assert!(!conflicts("a/b", "a/c"));
    }

    #[test]
    fn layout_maps_timeline_relative_names() {
        let l = Layout::new("/p");
        assert_eq!(
            l.timeline_file("refs/heads/feat/x"),
            PathBuf::from("/p/timeline/refs/heads/feat/x")
        );
        assert_eq!(l.cache_dir(), PathBuf::from("/p/.cache"));
    }
}
