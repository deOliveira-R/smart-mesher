//! On-disk timeline layout (docs/ARCHITECTURE.md §4):
//!
//! ```text
//! timeline/
//!   ops/<hash>.toml        one node per file; file name = SHA-256 of the file bytes
//!   refs/heads/<name>      text file containing a node hash
//!   HEAD                   "ref: refs/heads/<name>", a node hash, or empty (detached at root)
//! ```

use std::fs;
use std::path::Path;

use crate::error::TimelineError;
use crate::hash::Hash256;
use crate::timeline::{Head, Node, Timeline, valid_ref_name};

/// Write all nodes, refs and HEAD under `dir` (created if needed). Existing node files are
/// left alone (they are immutable by construction); refs and HEAD are rewritten.
pub fn save(timeline: &Timeline, dir: &Path) -> Result<(), TimelineError> {
    let ops = dir.join("ops");
    fs::create_dir_all(&ops)?;
    for (h, node) in timeline.nodes() {
        let p = ops.join(format!("{h}.toml"));
        if !p.exists() {
            fs::write(p, node.to_toml()?)?;
        }
    }
    let heads = dir.join("refs").join("heads");
    fs::create_dir_all(&heads)?;
    for (name, h) in timeline.refs() {
        let p = heads.join(name);
        if let Some(parent) = p.parent() {
            fs::create_dir_all(parent)?;
        }
        fs::write(p, format!("{h}\n"))?;
    }
    let head = match timeline.head() {
        Head::Ref(n) => format!("ref: refs/heads/{n}\n"),
        Head::Detached(Some(h)) => format!("{h}\n"),
        Head::Detached(None) => String::new(),
    };
    fs::write(dir.join("HEAD"), head)?;
    Ok(())
}

fn collect_refs(base: &Path, rel: &str, out: &mut Vec<(String, Hash256)>) -> Result<(), TimelineError> {
    let mut entries: Vec<_> = fs::read_dir(base)?.collect::<Result<_, _>>()?;
    entries.sort_by_key(|e| e.file_name());
    for e in entries {
        let name = e.file_name().to_string_lossy().into_owned();
        let full = if rel.is_empty() { name.clone() } else { format!("{rel}/{name}") };
        if e.file_type()?.is_dir() {
            collect_refs(&e.path(), &full, out)?;
        } else {
            let text = fs::read_to_string(e.path())?;
            let h = text.trim().parse().map_err(|_| TimelineError::Format(format!("bad ref file {full}")))?;
            out.push((full, h));
        }
    }
    Ok(())
}

/// Load a timeline, verifying every node file against its hash name.
pub fn load(dir: &Path) -> Result<Timeline, TimelineError> {
    let mut pending: Vec<(Hash256, Node)> = Vec::new();
    let mut files: Vec<_> = fs::read_dir(dir.join("ops"))?.collect::<Result<_, _>>()?;
    files.sort_by_key(|e| e.file_name());
    for e in files {
        let fname = e.file_name().to_string_lossy().into_owned();
        let Some(stem) = fname.strip_suffix(".toml") else { continue };
        let expected: Hash256 = stem.parse().map_err(|_| TimelineError::Format(format!("bad node file name {fname}")))?;
        let text = fs::read_to_string(e.path())?;
        let actual = Hash256::of(text.as_bytes());
        if actual != expected {
            return Err(TimelineError::HashMismatch { expected, actual });
        }
        pending.push((expected, Node::from_toml(&text)?));
    }
    let mut tl = Timeline::new();
    // Insert parents before children; a pass with no progress means a dangling parent.
    while !pending.is_empty() {
        let before = pending.len();
        let mut rest = Vec::new();
        for (h, n) in pending {
            match n.parent {
                Some(p) if tl.get(p).is_none() => rest.push((h, n)),
                _ => {
                    tl.insert(n)?;
                }
            }
        }
        if rest.len() == before {
            let missing = rest.first().and_then(|(_, n)| n.parent).unwrap_or_default();
            return Err(TimelineError::UnknownParent(missing));
        }
        pending = rest;
    }
    let heads = dir.join("refs").join("heads");
    if heads.exists() {
        let mut refs = Vec::new();
        collect_refs(&heads, "", &mut refs)?;
        for (name, h) in refs {
            if !valid_ref_name(&name) {
                return Err(TimelineError::BadRefName(name));
            }
            tl.set_ref(&name, h)?;
        }
    }
    let head_text = fs::read_to_string(dir.join("HEAD")).unwrap_or_default();
    let head_text = head_text.trim();
    let head = if let Some(r) = head_text.strip_prefix("ref: refs/heads/") {
        Head::Ref(r.to_owned())
    } else if head_text.is_empty() {
        Head::Detached(None)
    } else {
        Head::Detached(Some(head_text.parse().map_err(|_| TimelineError::Format("bad HEAD".into()))?))
    };
    tl.set_head(head)?;
    Ok(tl)
}
