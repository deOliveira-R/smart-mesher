//! `.smrt` cache image: serialization of a [`MeshTracker`] into the contract's sections, the
//! strict decoder (never panics on hostile bytes), and the `.smrt` file helpers.

use crate::bvh::Bvh;
use crate::core::{Core, F_PLANAR, NONE};
use crate::tracker::{MeshTracker, Parts};
use sm_core::{EntityId, Transform, Vector3};
use sm_geom_api::ModelTolerance;
use sm_rt_api::{
    CacheImage, CacheKey, ContentHash, FaceAction, FaceId, Precision, RecoveryPolicy, RegionId, RegionInfo, RegionTable, ResponseKind, Result, RtError, SectionTag, SurfaceInfo,
    SurfaceTable, TrackerInfo, TrackerKind,
};
use std::io::Write;

const META_VERSION: u32 = 1;
const TAG_META: u16 = 0;
const TAG_POS: u16 = 1;
const TAG_FACES: u16 = 2;
const TAG_APEX: u16 = 3;

fn bad(msg: &str) -> RtError {
    RtError::CacheCorrupt(msg.to_owned())
}

// ---- writing ------------------------------------------------------------------------------

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
    fn bytes(&mut self, b: &[u8]) {
        self.u64(b.len() as u64);
        self.0.extend_from_slice(b);
    }
    fn vec_u32(&mut self, v: &[u32]) {
        self.u64(v.len() as u64);
        for &x in v {
            self.u32(x);
        }
    }
    fn vec_f64(&mut self, v: impl Iterator<Item = f64>, n: usize) {
        self.u64(n as u64);
        for x in v {
            self.f64(x);
        }
    }
}

fn write_bvh(w: &mut W, b: &Bvh) {
    w.u64(b.bb.len() as u64);
    for bb in &b.bb {
        for &x in bb {
            w.f64(x);
        }
    }
    w.vec_u32(&b.link);
    w.vec_u32(&b.count);
    w.vec_u32(&b.prims);
}

fn write_action(w: &mut W, a: &FaceAction) {
    match a {
        FaceAction::Reflect => w.u32(1),
        FaceAction::Wrap { transform, mate } => {
            w.u32(2);
            for row in &transform.m {
                for &v in row {
                    w.f64(v);
                }
            }
            w.f64(transform.t.x);
            w.f64(transform.t.y);
            w.f64(transform.t.z);
            w.u32(mate.0);
        }
        FaceAction::Respond(r) => {
            w.u32(3);
            w.u32(r.code());
            w.f64(if let ResponseKind::Albedo(a) = r { *a } else { 0.0 });
        }
        _ => w.u32(0),
    }
}

/// Export a tracker as a cache image.
pub(crate) fn export(t: &MeshTracker) -> Option<CacheImage> {
    let key = t.info.key?;
    let c = &t.core;
    let mut meta = W::default();
    meta.u32(META_VERSION);
    meta.u32(t.kind as u32);
    meta.u64(c.pos.len() as u64);
    meta.u64(c.n_faces() as u64);
    meta.u64(c.n_cells() as u64);
    let tol = t.info.tol;
    for v in [tol.linear, tol.angular, tol.parametric, tol.linear_max] {
        meta.f64(v);
    }
    meta.u32(t.info.max_events);
    meta.u64(t.block_size as u64);
    match t.recovery {
        RecoveryPolicy::Nudge { tries } => {
            meta.u32(1);
            meta.u32(u32::from(tries));
        }
        _ => {
            meta.u32(0);
            meta.u32(0);
        }
    }
    meta.u32(t.complement.unwrap_or(NONE));
    meta.0.extend_from_slice(&t.info.structure_hash.0.to_le_bytes());

    let mut pos = W::default();
    pos.vec_f64(c.pos.iter().flat_map(|p| *p), c.pos.len() * 3);

    let mut faces = W::default();
    faces.vec_u32(&c.fv_off);
    faces.vec_u32(&c.fv);
    faces.vec_u32(&c.f_left);
    faces.vec_u32(&c.f_right);
    faces.vec_u32(&c.f_surface);
    faces.vec_u32(&c.f_action);
    faces.bytes(&c.f_flags);

    let mut planes = W::default();
    planes.vec_f64(c.f_plane.iter().flat_map(|p| *p), c.f_plane.len() * 4);

    let mut apex = W::default();
    apex.vec_f64(c.f_apex.iter().flat_map(|p| *p), c.f_apex.len() * 3);

    let mut cells = W::default();
    cells.vec_u32(&c.c_off);
    cells.vec_u32(&c.c_face);
    cells.vec_u32(&c.c_region);
    cells.bytes(&c.c_flags);
    cells.vec_f64(c.c_bbox.iter().flat_map(|b| *b), c.c_bbox.len() * 6);
    cells.vec_f64(c.c_volume.iter().copied(), c.c_volume.len());

    let mut tables = W::default();
    tables.u64(t.regions.0.len() as u64);
    for r in &t.regions.0 {
        tables.u64(r.entity.map_or(0, |e| e.to_raw()));
        tables.bytes(r.name.as_bytes());
        tables.u32(u32::from(r.implicit_complement));
    }
    tables.u64(t.surfaces.0.len() as u64);
    for s in &t.surfaces.0 {
        tables.u64(s.entity.map_or(0, |e| e.to_raw()));
        tables.u32(s.front.0);
        tables.u32(s.back.0);
        match &s.boundary {
            Some(a) => {
                tables.u32(1);
                write_action(&mut tables, a);
            }
            None => tables.u32(0),
        }
    }
    tables.u64(t.cell_sets.len() as u64);
    for (n, cs) in &t.cell_sets {
        tables.bytes(n.as_bytes());
        tables.vec_u32(cs);
    }

    let mut bnd = W::default();
    bnd.u64(c.actions.len() as u64);
    for a in &c.actions {
        write_action(&mut bnd, a);
    }

    let mut fb = W::default();
    write_bvh(&mut fb, &c.face_bvh);
    let mut cb = W::default();
    write_bvh(&mut cb, &c.cell_bvh);

    Some(CacheImage {
        key,
        sections: vec![
            (SectionTag::FacePlanes, planes.0),
            (SectionTag::CellInfo, cells.0),
            (SectionTag::Bvh, fb.0),
            (SectionTag::CellBvh, cb.0),
            (SectionTag::Tables, tables.0),
            (SectionTag::Boundaries, bnd.0),
            (SectionTag::Private(TAG_META), meta.0),
            (SectionTag::Private(TAG_POS), pos.0),
            (SectionTag::Private(TAG_FACES), faces.0),
            (SectionTag::Private(TAG_APEX), apex.0),
        ],
    })
}

// ---- reading ------------------------------------------------------------------------------

struct R<'a> {
    b: &'a [u8],
    at: usize,
}

impl<'a> R<'a> {
    fn new(b: &'a [u8]) -> Self {
        Self { b, at: 0 }
    }
    fn take(&mut self, n: usize) -> Result<&'a [u8]> {
        let end = self.at.checked_add(n).filter(|&e| e <= self.b.len()).ok_or_else(|| bad("section truncated"))?;
        let s = &self.b[self.at..end];
        self.at = end;
        Ok(s)
    }
    fn u32(&mut self) -> Result<u32> {
        let s = self.take(4)?;
        Ok(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
    }
    fn u64(&mut self) -> Result<u64> {
        let s = self.take(8)?;
        Ok(u64::from_le_bytes([s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7]]))
    }
    fn f64(&mut self) -> Result<f64> {
        Ok(f64::from_bits(self.u64()?))
    }
    fn len(&mut self, elem: usize) -> Result<usize> {
        let n = usize::try_from(self.u64()?).map_err(|_| bad("length overflow"))?;
        if n.checked_mul(elem).is_none_or(|b| b > self.b.len() - self.at) {
            return Err(bad("length exceeds section"));
        }
        Ok(n)
    }
    fn bytes(&mut self) -> Result<Vec<u8>> {
        let n = self.len(1)?;
        Ok(self.take(n)?.to_vec())
    }
    fn vec_u32(&mut self) -> Result<Vec<u32>> {
        let n = self.len(4)?;
        let s = self.take(n * 4)?;
        Ok(s.as_chunks::<4>().0.iter().map(|c| u32::from_le_bytes(*c)).collect())
    }
    fn vec_f64(&mut self) -> Result<Vec<f64>> {
        let n = self.len(8)?;
        let s = self.take(n * 8)?;
        Ok(s.as_chunks::<8>().0.iter().map(|c| f64::from_bits(u64::from_le_bytes(*c))).collect())
    }
    fn done(&self) -> Result<()> {
        if self.at == self.b.len() { Ok(()) } else { Err(bad("trailing bytes in section")) }
    }
}

fn chunk3(v: Vec<f64>) -> Result<Vec<[f64; 3]>> {
    if !v.len().is_multiple_of(3) {
        return Err(bad("column length"));
    }
    Ok(v.as_chunks::<3>().0.to_vec())
}

fn read_bvh(b: &[u8]) -> Result<Bvh> {
    let mut r = R::new(b);
    let n = r.len(48)?;
    let mut bb = Vec::with_capacity(n);
    for _ in 0..n {
        let mut x = [0.0; 6];
        for v in &mut x {
            *v = r.f64()?;
        }
        bb.push(x);
    }
    let bvh = Bvh { bb, link: r.vec_u32()?, count: r.vec_u32()?, prims: r.vec_u32()? };
    r.done()?;
    Ok(bvh)
}

fn read_action(r: &mut R<'_>) -> Result<FaceAction> {
    Ok(match r.u32()? {
        0 => FaceAction::Transmit,
        1 => FaceAction::Reflect,
        2 => {
            let mut m = [[0.0; 3]; 3];
            for row in &mut m {
                for v in row.iter_mut() {
                    *v = r.f64()?;
                }
            }
            let t = Vector3::new(r.f64()?, r.f64()?, r.f64()?);
            let mate = FaceId(r.u32()?);
            FaceAction::Wrap { transform: Transform { m, t }, mate }
        }
        3 => {
            let code = r.u32()?;
            let a = r.f64()?;
            FaceAction::Respond(match code {
                1 => ResponseKind::Vacuum,
                2 => ResponseKind::White,
                3 => ResponseKind::Albedo(a),
                4 => ResponseKind::Reflective,
                _ => return Err(bad("unknown response kind")),
            })
        }
        _ => return Err(bad("unknown face action")),
    })
}

fn entity_opt(raw: u64) -> Result<Option<EntityId>> {
    if raw == 0 { Ok(None) } else { EntityId::from_raw(raw).map(Some).ok_or_else(|| bad("bad entity id")) }
}

fn section(img: &CacheImage, tag: SectionTag) -> Result<&[u8]> {
    img.section(tag).ok_or_else(|| bad("missing section"))
}

/// Decode and fully validate an image into a tracker (no mesh needed).
pub(crate) fn import(img: &CacheImage) -> Result<MeshTracker> {
    let mut m = R::new(section(img, SectionTag::Private(TAG_META))?);
    if m.u32()? != META_VERSION {
        return Err(RtError::CacheStale("meta version"));
    }
    let kind = TrackerKind::from_u32(m.u32()?).filter(|k| matches!(k, TrackerKind::MeshWalk | TrackerKind::Facets)).ok_or_else(|| bad("tracker kind"))?;
    if kind != img.key.kind {
        return Err(bad("tracker kind differs from the key"));
    }
    let (nv, nf, nc) = (m.u64()? as usize, m.u64()? as usize, m.u64()? as usize);
    let tol = ModelTolerance::new(m.f64()?, m.f64()?, m.f64()?, m.f64()?);
    let max_events = m.u32()?;
    let block_size = m.u64()? as usize;
    let recovery = match (m.u32()?, m.u32()?) {
        (0, _) => RecoveryPolicy::Fail,
        (1, t) => RecoveryPolicy::Nudge { tries: u8::try_from(t).map_err(|_| bad("recovery tries"))? },
        _ => return Err(bad("recovery policy")),
    };
    let complement = match m.u32()? {
        NONE => None,
        r => Some(r),
    };
    let hash = ContentHash(u128::from_le_bytes(m.take(16)?.try_into().map_err(|_| bad("hash"))?));
    m.done()?;

    let mut core = Core::default();
    let mut r = R::new(section(img, SectionTag::Private(TAG_POS))?);
    core.pos = chunk3(r.vec_f64()?)?;
    r.done()?;
    let mut r = R::new(section(img, SectionTag::Private(TAG_FACES))?);
    core.fv_off = r.vec_u32()?;
    core.fv = r.vec_u32()?;
    core.f_left = r.vec_u32()?;
    core.f_right = r.vec_u32()?;
    core.f_surface = r.vec_u32()?;
    core.f_action = r.vec_u32()?;
    core.f_flags = r.bytes()?;
    r.done()?;
    let mut r = R::new(section(img, SectionTag::FacePlanes)?);
    let pl = r.vec_f64()?;
    r.done()?;
    if !pl.len().is_multiple_of(4) {
        return Err(bad("plane column length"));
    }
    core.f_plane = pl.as_chunks::<4>().0.to_vec();
    let mut r = R::new(section(img, SectionTag::Private(TAG_APEX))?);
    core.f_apex = chunk3(r.vec_f64()?)?;
    r.done()?;
    let mut r = R::new(section(img, SectionTag::CellInfo)?);
    core.c_off = r.vec_u32()?;
    core.c_face = r.vec_u32()?;
    core.c_region = r.vec_u32()?;
    core.c_flags = r.bytes()?;
    let bb = r.vec_f64()?;
    if !bb.len().is_multiple_of(6) {
        return Err(bad("box column length"));
    }
    core.c_bbox = bb.as_chunks::<6>().0.to_vec();
    core.c_volume = r.vec_f64()?;
    r.done()?;
    let mut r = R::new(section(img, SectionTag::Boundaries)?);
    let na = r.len(4)?;
    for _ in 0..na {
        core.actions.push(read_action(&mut r)?);
    }
    r.done()?;
    core.face_bvh = read_bvh(section(img, SectionTag::Bvh)?)?;
    core.cell_bvh = read_bvh(section(img, SectionTag::CellBvh)?)?;

    let mut r = R::new(section(img, SectionTag::Tables)?);
    let nr = r.len(8)?;
    let mut regions = Vec::with_capacity(nr);
    for _ in 0..nr {
        let ent = entity_opt(r.u64()?)?;
        let name = String::from_utf8(r.bytes()?).map_err(|_| bad("region name"))?;
        let comp = r.u32()? != 0;
        let mut info = RegionInfo::new(name);
        if let Some(e) = ent {
            info = info.with_entity(e);
        }
        if comp {
            info = info.as_implicit_complement();
        }
        regions.push(info);
    }
    let ns = r.len(8)?;
    let mut surfaces = Vec::with_capacity(ns);
    for _ in 0..ns {
        let ent = entity_opt(r.u64()?)?;
        let mut s = SurfaceInfo::new(RegionId(r.u32()?), RegionId(r.u32()?));
        if let Some(e) = ent {
            s = s.with_entity(e);
        }
        if r.u32()? == 1 {
            s = s.with_boundary(read_action(&mut r)?);
        }
        surfaces.push(s);
    }
    let ncs = r.len(8)?;
    let mut cell_sets = Vec::with_capacity(ncs);
    for _ in 0..ncs {
        let name = String::from_utf8(r.bytes()?).map_err(|_| bad("set name"))?;
        cell_sets.push((name, r.vec_u32()?));
    }
    r.done()?;

    validate_core(&core, nv, nf, nc, regions.len(), surfaces.len(), &cell_sets)?;
    let regions = RegionTable(regions);
    let surfaces = SurfaceTable(surfaces);
    let structure = MeshTracker::hash_structure(&core, &regions, &surfaces, kind, complement, &cell_sets);
    if structure != hash {
        return Err(bad("structure hash mismatch"));
    }
    let info = make_info(&core, &regions, &surfaces, kind, tol, max_events, hash, Some(img.key));
    Ok(MeshTracker::assemble(Parts { core, regions, surfaces, kind, complement, cell_sets }, info, block_size, recovery))
}

fn validate_core(c: &Core, nv: usize, nf: usize, nc: usize, nreg: usize, nsurf: usize, sets: &[(String, Vec<u32>)]) -> Result<()> {
    let ok = c.pos.len() == nv
        && c.fv_off.len() == nf + 1
        && c.f_left.len() == nf
        && c.f_right.len() == nf
        && c.f_surface.len() == nf
        && c.f_action.len() == nf
        && c.f_flags.len() == nf
        && c.f_plane.len() == nf
        && c.f_apex.len() == nf
        && c.c_off.len() == nc + 1
        && c.c_region.len() == nc
        && c.c_flags.len() == nc
        && c.c_bbox.len() == nc
        && c.c_volume.len() == nc;
    if !ok {
        return Err(bad("column length mismatch"));
    }
    let mono = |o: &[u32], end: usize| o.first() == Some(&0) && o.windows(2).all(|w| w[0] <= w[1]) && o.last().is_some_and(|&l| l as usize == end);
    if !mono(&c.fv_off, c.fv.len()) || !mono(&c.c_off, c.c_face.len()) {
        return Err(bad("offset columns"));
    }
    if c.fv_off.windows(2).any(|w| w[1] - w[0] < 3) {
        return Err(bad("face loop shorter than 3"));
    }
    if c.fv.iter().any(|&v| v as usize >= nv) {
        return Err(bad("face vertex out of range"));
    }
    if c.f_left.iter().any(|&x| x as usize >= nc) || c.f_right.iter().any(|&x| x != NONE && x as usize >= nc) {
        return Err(bad("face cell out of range"));
    }
    if c.c_face.iter().any(|&f| f as usize >= nf) {
        return Err(bad("cell face out of range"));
    }
    for ci in 0..nc {
        if c.cell_faces(ci).iter().any(|&f| c.f_left[f as usize] as usize != ci && c.f_right[f as usize] as usize != ci) {
            return Err(bad("cell lists a face that does not list it"));
        }
    }
    if c.c_region.iter().any(|&r| r as usize >= nreg) || c.f_surface.iter().any(|&s| s != NONE && s as usize >= nsurf) || c.f_action.iter().any(|&a| a != NONE && a as usize >= c.actions.len()) {
        return Err(bad("table index out of range"));
    }
    if c.actions.iter().any(|a| matches!(a, FaceAction::Wrap { mate, .. } if mate.get() >= nf)) {
        return Err(bad("wrap mate out of range"));
    }
    if c.f_flags.iter().any(|&f| f & !F_PLANAR != 0) || c.c_flags.iter().any(|&f| f > 3) {
        return Err(bad("flag bits"));
    }
    if !c.face_bvh.validate(nf) || !c.cell_bvh.validate(nc) {
        return Err(bad("bvh structure"));
    }
    if sets.iter().any(|(_, cs)| cs.iter().any(|&x| x as usize >= nc)) {
        return Err(bad("cell set member out of range"));
    }
    Ok(())
}

/// The `TrackerInfo` of a structure (bounds are filled by `MeshTracker::assemble`).
#[allow(clippy::too_many_arguments)]
pub(crate) fn make_info(core: &Core, regions: &RegionTable, surfaces: &SurfaceTable, kind: TrackerKind, tol: ModelTolerance, max_events: u32, hash: ContentHash, key: Option<CacheKey>) -> TrackerInfo {
    let mut info = TrackerInfo::new(sm_core::Aabb::empty(), core.n_cells(), hash);
    let nf = core.n_faces();
    let nonplanar = core.f_flags.iter().filter(|&&f| f & F_PLANAR == 0).count();
    info.faces = nf;
    let table_bytes: usize = regions.0.iter().map(|r| 24 + r.name.len()).sum::<usize>() + surfaces.0.len() * core::mem::size_of::<SurfaceInfo>();
    info.memory_bytes = core.memory_bytes() + table_bytes;
    if kind == TrackerKind::Facets {
        info.faceted_faces = nf;
        info.exact_faces = 0;
    } else {
        info.faceted_faces = nonplanar;
        info.exact_faces = nf - nonplanar;
    }
    info.tol = tol;
    info.precision = Precision::F64;
    info.max_events = max_events;
    info.key = key;
    info
}

/// Write a cache image as a `.smrt` stream: the [`CacheImage::encode`] bytes verbatim.
pub fn write_smrt<W: Write>(mut w: W, image: &CacheImage) -> std::io::Result<()> {
    w.write_all(&image.encode())
}

/// Write a cache image to a `.smrt` file.
pub fn write_smrt_file(path: impl AsRef<std::path::Path>, image: &CacheImage) -> std::io::Result<()> {
    std::fs::write(path, image.encode())
}

/// Parse `.smrt` bytes (magic, version, alignment and every section hash are verified).
pub fn read_smrt(bytes: &[u8]) -> Result<CacheImage> {
    CacheImage::decode(bytes)
}
