//! The gitignored `.cache/` directory: per-checkout state that is not part of the project.
//!
//! Everything here is derived or personal (HEAD, reflog, redo stack, state snapshots). A missing
//! or damaged cache never prevents opening a project: HEAD falls back to
//! `Timeline::default_head`, the reflog and redo stack start empty, and a snapshot that fails
//! its checks is ignored.

use std::fs::{self, OpenOptions};
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::PathBuf;

use sm_op_api::{Hash256, Head, ReflogEntry, Timeline};
use sm_op_api::{KindSnapshot, StateSnapshot};

use crate::error::{EngineError, Result};
use crate::fs::{Layout, WriteCtx};

impl Layout {
    fn head_file(&self) -> PathBuf {
        self.cache_dir().join("HEAD")
    }
    fn reflog_file(&self) -> PathBuf {
        self.cache_dir().join("reflog.jsonl")
    }
    fn redo_file(&self) -> PathBuf {
        self.cache_dir().join("redo.json")
    }
    pub(crate) fn snapshot_dir(&self) -> PathBuf {
        self.cache_dir().join("snapshots")
    }
    fn snapshot_file(&self, node: Hash256) -> PathBuf {
        self.snapshot_dir().join(format!("{node}.snap"))
    }
}

/// HEAD as stored in `.cache/HEAD` (one line of JSON: `{"ref":"main"}`, `{"node":"<hash>"}` or
/// `"empty"`), or `None` if missing or unreadable.
pub(crate) fn read_head(layout: &Layout) -> Option<Head> {
    let text = fs::read_to_string(layout.head_file()).ok()?;
    serde_json::from_str(text.trim()).ok()
}

pub(crate) fn write_head(layout: &Layout, head: &Head, ctx: WriteCtx<'_>) -> Result<()> {
    let mut s = serde_json::to_string(head).map_err(|e| EngineError::Format(e.to_string()))?;
    s.push('\n');
    ctx.write(&layout.head_file(), s.as_bytes())
}

/// The newest `limit` reflog entries. A torn last line or any unparsable line ends the read;
/// the second value says whether the file should be rewritten (torn, or longer than `2 * limit`).
pub(crate) fn read_reflog(layout: &Layout, limit: usize) -> (Vec<ReflogEntry>, bool) {
    let Ok(text) = fs::read_to_string(layout.reflog_file()) else {
        return (Vec::new(), false);
    };
    let mut entries = Vec::new();
    let mut clean = text.is_empty() || text.ends_with('\n');
    for line in text.lines() {
        match serde_json::from_str::<ReflogEntry>(line) {
            Ok(e) => entries.push(e),
            Err(_) => clean = false, // skip the bad line, keep the newer entries
        }
    }
    let too_long = entries.len() > limit.saturating_mul(2);
    if entries.len() > limit {
        entries.drain(..entries.len() - limit);
    }
    (entries, !clean || too_long)
}

/// Replace the reflog file with exactly `entries`.
pub(crate) fn rewrite_reflog(
    layout: &Layout,
    entries: &[ReflogEntry],
    ctx: WriteCtx<'_>,
) -> Result<()> {
    let mut buf = String::new();
    for e in entries {
        buf.push_str(&serde_json::to_string(e).map_err(|e| EngineError::Format(e.to_string()))?);
        buf.push('\n');
    }
    ctx.write(&layout.reflog_file(), buf.as_bytes())
}

/// Append entries as whole lines (repairing a torn last line first by starting a new one).
pub(crate) fn append_reflog(layout: &Layout, entries: &[ReflogEntry]) -> Result<()> {
    if entries.is_empty() {
        return Ok(());
    }
    let path = layout.reflog_file();
    fs::create_dir_all(layout.cache_dir()).map_err(|e| EngineError::io(layout.cache_dir(), e))?;
    let mut f = OpenOptions::new()
        .create(true)
        .read(true)
        .append(true)
        .open(&path)
        .map_err(|e| EngineError::io(&path, e))?;
    let io = |e| EngineError::io(&path, e);
    let len = f.metadata().map_err(io)?.len();
    let mut buf = String::new();
    if len > 0 {
        let mut last = [0u8; 1];
        f.seek(SeekFrom::End(-1)).map_err(io)?;
        f.read_exact(&mut last).map_err(io)?;
        if last[0] != b'\n' {
            buf.push('\n');
        }
    }
    for e in entries {
        buf.push_str(&serde_json::to_string(e).map_err(|e| EngineError::Format(e.to_string()))?);
        buf.push('\n');
    }
    f.write_all(buf.as_bytes()).map_err(io)
}

/// The persisted redo stack (bottom first), empty if missing or unreadable.
pub(crate) fn read_redo(layout: &Layout) -> Vec<Hash256> {
    fs::read_to_string(layout.redo_file())
        .ok()
        .and_then(|t| serde_json::from_str(&t).ok())
        .unwrap_or_default()
}

pub(crate) fn write_redo(layout: &Layout, redo: &[Hash256], ctx: WriteCtx<'_>) -> Result<()> {
    if redo.is_empty() {
        return ctx.remove(&layout.redo_file());
    }
    let s = serde_json::to_string(redo).map_err(|e| EngineError::Format(e.to_string()))?;
    ctx.write(&layout.redo_file(), s.as_bytes())
}

// ------------------------------------------------------------------ snapshots

const MAGIC: &[u8; 8] = b"SMSNAP01";

/// A decoded snapshot file.
pub(crate) struct SnapshotFile {
    pub(crate) node: Hash256,
    pub(crate) state: Hash256,
    pub(crate) snap: StateSnapshot,
}

fn put_u32(buf: &mut Vec<u8>, v: usize) -> Result<()> {
    let v = u32::try_from(v).map_err(|_| EngineError::Format("snapshot field too large".into()))?;
    buf.extend_from_slice(&v.to_le_bytes());
    Ok(())
}

/// Binary container: magic, node hash, state hash, kinds (JSON), layers (name + bytes), then a
/// SHA-256 of everything before it.
pub(crate) fn encode_snapshot(
    node: Hash256,
    state: Hash256,
    snap: &StateSnapshot,
) -> Result<Vec<u8>> {
    let kinds = serde_json::to_vec(&snap.kinds).map_err(|e| EngineError::Format(e.to_string()))?;
    let mut buf =
        Vec::with_capacity(128 + kinds.len() + snap.layers.values().map(Vec::len).sum::<usize>());
    buf.extend_from_slice(MAGIC);
    buf.extend_from_slice(&node.0);
    buf.extend_from_slice(&state.0);
    put_u32(&mut buf, kinds.len())?;
    buf.extend_from_slice(&kinds);
    put_u32(&mut buf, snap.layers.len())?;
    for (name, bytes) in &snap.layers {
        put_u32(&mut buf, name.len())?;
        buf.extend_from_slice(name.as_bytes());
        buf.extend_from_slice(&(bytes.len() as u64).to_le_bytes());
        buf.extend_from_slice(bytes);
    }
    let sum = Hash256::of(&buf);
    buf.extend_from_slice(&sum.0);
    Ok(buf)
}

struct Reader<'a> {
    b: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8]> {
        let end = self.pos.checked_add(n).filter(|e| *e <= self.b.len());
        let end = end.ok_or_else(|| EngineError::Format("snapshot truncated".into()))?;
        let s = &self.b[self.pos..end];
        self.pos = end;
        Ok(s)
    }
    fn array<const N: usize>(&mut self) -> Result<[u8; N]> {
        let mut a = [0u8; N];
        a.copy_from_slice(self.take(N)?);
        Ok(a)
    }
    fn u32(&mut self) -> Result<usize> {
        Ok(u32::from_le_bytes(self.array()?) as usize)
    }
    fn u64(&mut self) -> Result<usize> {
        usize::try_from(u64::from_le_bytes(self.array()?))
            .map_err(|_| EngineError::Format("snapshot field too large".into()))
    }
}

pub(crate) fn decode_snapshot(bytes: &[u8]) -> Result<SnapshotFile> {
    let body_len = bytes
        .len()
        .checked_sub(32)
        .ok_or_else(|| EngineError::Format("snapshot truncated".into()))?;
    let (body, sum) = bytes.split_at(body_len);
    if Hash256::of(body).0 != sum {
        return Err(EngineError::Format("snapshot checksum mismatch".into()));
    }
    let mut r = Reader { b: body, pos: 0 };
    if r.take(8)? != MAGIC {
        return Err(EngineError::Format("not a snapshot file".into()));
    }
    let node = Hash256(r.array()?);
    let state = Hash256(r.array()?);
    let n = r.u32()?;
    let kinds: Vec<KindSnapshot> = serde_json::from_slice(r.take(n)?)
        .map_err(|e| EngineError::Format(format!("snapshot kinds: {e}")))?;
    let count = r.u32()?;
    let mut layers = std::collections::BTreeMap::new();
    for _ in 0..count {
        let n = r.u32()?;
        let name = std::str::from_utf8(r.take(n)?)
            .map_err(|_| EngineError::Format("snapshot layer name".into()))?
            .to_owned();
        let len = r.u64()?;
        layers.insert(name, r.take(len)?.to_vec());
    }
    if r.pos != body.len() {
        return Err(EngineError::Format("snapshot has trailing bytes".into()));
    }
    Ok(SnapshotFile {
        node,
        state,
        snap: StateSnapshot { kinds, layers },
    })
}

pub(crate) fn write_snapshot(
    layout: &Layout,
    node: Hash256,
    state: Hash256,
    snap: &StateSnapshot,
    ctx: WriteCtx<'_>,
) -> Result<()> {
    let bytes = encode_snapshot(node, state, snap)?;
    ctx.write(&layout.snapshot_file(node), &bytes)
}

/// Read the snapshot stored for `node`, checking that it names that node.
pub(crate) fn read_snapshot(layout: &Layout, node: Hash256) -> Result<Option<SnapshotFile>> {
    let path = layout.snapshot_file(node);
    let bytes = match fs::read(&path) {
        Ok(b) => b,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(EngineError::io(&path, e)),
    };
    let f = decode_snapshot(&bytes)?;
    if f.node != node {
        return Err(EngineError::Format("snapshot names another node".into()));
    }
    Ok(Some(f))
}

/// Nodes that have a snapshot file.
pub(crate) fn snapshot_nodes(layout: &Layout) -> Vec<Hash256> {
    let Ok(rd) = fs::read_dir(layout.snapshot_dir()) else {
        return Vec::new();
    };
    let mut v: Vec<Hash256> = rd
        .filter_map(|e| e.ok())
        .filter_map(|e| {
            let name = e.file_name().to_string_lossy().into_owned();
            name.strip_suffix(".snap")?.parse().ok()
        })
        .collect();
    v.sort();
    v
}

/// Keep the `keep` deepest snapshots (ties broken by hash) and delete the rest, including
/// snapshots of nodes that are not in the timeline. `protect` (the snapshot just written) is
/// always kept and counts toward `keep`.
pub(crate) fn prune_snapshots(
    layout: &Layout,
    tl: &Timeline,
    keep: usize,
    protect: Option<Hash256>,
    ctx: WriteCtx<'_>,
) -> Result<()> {
    let mut known = Vec::new();
    for h in snapshot_nodes(layout) {
        match tl.depth(h) {
            Some(d) => known.push((std::cmp::Reverse(d), h)),
            None => ctx.remove(&layout.snapshot_file(h))?,
        }
    }
    known.sort();
    if let Some(p) = protect {
        known.retain(|(_, h)| *h != p);
    }
    let keep = keep.saturating_sub(usize::from(protect.is_some()));
    for (_, h) in known.into_iter().skip(keep) {
        ctx.remove(&layout.snapshot_file(h))?;
    }
    Ok(())
}

/// Delete one snapshot (a stale or damaged one).
pub(crate) fn remove_snapshot(layout: &Layout, node: Hash256, ctx: WriteCtx<'_>) -> Result<()> {
    ctx.remove(&layout.snapshot_file(node))
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_core::EntityKind;

    fn sample() -> (Hash256, Hash256, StateSnapshot) {
        let mut layers = std::collections::BTreeMap::new();
        layers.insert("counter".to_owned(), vec![1, 2, 3, 4, 5]);
        layers.insert("model".to_owned(), Vec::new());
        let snap = StateSnapshot {
            kinds: vec![KindSnapshot {
                kind: EntityKind::TopoBody,
                next: 7,
                retired: vec![2, 5],
            }],
            layers,
        };
        (Hash256::of(b"node"), Hash256::of(b"state"), snap)
    }

    #[test]
    fn snapshot_round_trips() {
        let (n, s, snap) = sample();
        let bytes = encode_snapshot(n, s, &snap).unwrap();
        let f = decode_snapshot(&bytes).unwrap();
        assert_eq!((f.node, f.state), (n, s));
        assert_eq!(f.snap, snap);
    }

    #[test]
    fn damaged_snapshots_are_errors_never_panics() {
        let (n, s, snap) = sample();
        let bytes = encode_snapshot(n, s, &snap).unwrap();
        for len in 0..bytes.len() {
            assert!(
                decode_snapshot(&bytes[..len]).is_err(),
                "truncated to {len}"
            );
        }
        for i in 0..bytes.len() {
            let mut b = bytes.clone();
            b[i] ^= 0x40;
            assert!(decode_snapshot(&b).is_err(), "bit flip at {i}");
        }
        // A forged length with a valid checksum must not read out of bounds either.
        let mut forged = bytes[..bytes.len() - 32].to_vec();
        forged[8 + 64] = 0xff;
        forged[8 + 65] = 0xff;
        forged[8 + 66] = 0xff;
        forged[8 + 67] = 0x7f;
        let sum = Hash256::of(&forged);
        forged.extend_from_slice(&sum.0);
        assert!(decode_snapshot(&forged).is_err());
    }
}
