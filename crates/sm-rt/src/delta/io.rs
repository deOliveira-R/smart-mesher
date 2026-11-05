//! The partition as `.smrt` sections `Private(5)`, `Private(6)` and `Private(7)`. (`Private(4)` is
//! the region-tag table.) A tracker without a partition writes none of them, so its image bytes
//! are unchanged. The decoder never panics: every count is checked against the bytes left, and
//! `Partition::from_raw` validates the structure.

use super::kd::Raw;
use super::{KdNode, Params, Partition};
use crate::core::Core;
use sm_rt_api::{CacheImage, MaterialSetId, Result, RtError, SectionTag};

const TAG_META: u16 = 5;
const TAG_KD: u16 = 6;
const TAG_LISTS: u16 = 7;
const VERSION: u32 = 1;

fn bad(msg: &str) -> RtError {
    RtError::CacheCorrupt(msg.to_owned())
}

#[derive(Default)]
struct W(Vec<u8>);

impl W {
    fn u32(&mut self, v: u32) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn u64(&mut self, v: u64) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn f64(&mut self, v: f64) {
        self.0.extend_from_slice(&v.to_bits().to_le_bytes());
    }
    fn vec_u32(&mut self, v: &[u32]) {
        self.u64(v.len() as u64);
        for &x in v {
            self.u32(x);
        }
    }
}

/// The sections of a partition.
pub(crate) fn write_sections(p: &Partition) -> Vec<(SectionTag, Vec<u8>)> {
    let mut meta = W::default();
    meta.u32(VERSION);
    meta.u32(p.params.leaf_cells);
    meta.u32(p.params.max_depth);
    meta.u32(u32::from(p.params.region_jump));
    for x in p.root_box {
        meta.f64(x);
    }
    let mut kd = W::default();
    kd.u64(p.kd.len() as u64);
    for n in &p.kd {
        kd.f64(n.split);
        kd.u32(u32::from(n.axis));
        kd.u32(u32::from(n.flags));
        kd.u32(n.node);
        kd.u32(n.a);
        kd.u32(n.b);
    }
    let mut lists = W::default();
    lists.vec_u32(&p.cands);
    let node_set: Vec<u32> = p.nodes.iter().map(|n| n.set).collect();
    let node_cells: Vec<u32> = p.nodes.iter().map(|n| n.cells).collect();
    lists.vec_u32(&node_set);
    lists.vec_u32(&node_cells);
    lists.u64(p.sets.len() as u64);
    for s in 0..p.sets.len() {
        let regions: Vec<u32> = p.sets.get(MaterialSetId::new(s)).unwrap_or(&[]).iter().map(|r| r.0).collect();
        lists.vec_u32(&regions);
    }
    vec![(SectionTag::Private(TAG_META), meta.0), (SectionTag::Private(TAG_KD), kd.0), (SectionTag::Private(TAG_LISTS), lists.0)]
}

struct R<'a> {
    b: &'a [u8],
    at: usize,
}

impl<'a> R<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8]> {
        let end = self.at.checked_add(n).filter(|&e| e <= self.b.len()).ok_or_else(|| bad("truncated partition section"))?;
        let s = &self.b[self.at..end];
        self.at = end;
        Ok(s)
    }
    fn u32(&mut self) -> Result<u32> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().map_err(|_| bad("u32"))?))
    }
    fn u64(&mut self) -> Result<u64> {
        Ok(u64::from_le_bytes(self.take(8)?.try_into().map_err(|_| bad("u64"))?))
    }
    fn f64(&mut self) -> Result<f64> {
        Ok(f64::from_bits(self.u64()?))
    }
    /// A count whose `elem`-byte entries must fit in the bytes left.
    fn len(&mut self, elem: usize) -> Result<usize> {
        let n = usize::try_from(self.u64()?).map_err(|_| bad("count"))?;
        if n.checked_mul(elem).is_none_or(|b| b > self.b.len() - self.at) {
            return Err(bad("count exceeds the section"));
        }
        Ok(n)
    }
    fn vec_u32(&mut self) -> Result<Vec<u32>> {
        let n = self.len(4)?;
        (0..n).map(|_| self.u32()).collect()
    }
    fn done(&self) -> Result<()> {
        if self.at == self.b.len() { Ok(()) } else { Err(bad("trailing bytes in a partition section")) }
    }
}

/// Decode the partition of an image, if it has one.
pub(crate) fn read_sections(img: &CacheImage, core: &Core, nreg: usize) -> Result<Option<Partition>> {
    let Some(mb) = img.section(SectionTag::Private(TAG_META)) else { return Ok(None) };
    let kb = img.section(SectionTag::Private(TAG_KD)).ok_or_else(|| bad("missing kd section"))?;
    let lb = img.section(SectionTag::Private(TAG_LISTS)).ok_or_else(|| bad("missing candidate section"))?;
    let mut m = R { b: mb, at: 0 };
    if m.u32()? != VERSION {
        return Err(RtError::CacheStale("partition version"));
    }
    let params = Params { leaf_cells: m.u32()?, max_depth: m.u32()?, region_jump: m.u32()? != 0 };
    let mut root_box = [0.0; 6];
    for x in &mut root_box {
        *x = m.f64()?;
    }
    m.done()?;
    let mut k = R { b: kb, at: 0 };
    let nk = k.len(28)?;
    let mut kd = Vec::with_capacity(nk);
    for _ in 0..nk {
        let split = k.f64()?;
        let axis = u8::try_from(k.u32()?).map_err(|_| bad("kd axis"))?;
        let flags = u8::try_from(k.u32()?).map_err(|_| bad("kd flags"))?;
        kd.push(KdNode { split, axis, flags, node: k.u32()?, a: k.u32()?, b: k.u32()? });
    }
    k.done()?;
    let mut l = R { b: lb, at: 0 };
    let cands = l.vec_u32()?;
    let node_set = l.vec_u32()?;
    let node_cells = l.vec_u32()?;
    let ns = l.len(8)?;
    let mut set_lists = Vec::with_capacity(ns);
    for _ in 0..ns {
        set_lists.push(l.vec_u32()?);
    }
    l.done()?;
    Partition::from_raw(Raw { params, root_box, kd, cands, node_set, node_cells, set_lists }, core, nreg).map(Some)
}
