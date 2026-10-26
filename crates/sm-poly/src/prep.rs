//! Analysis of the source tet mesh: regions, (edge, region) slots, rings, split edges, feature
//! classes and boundary fans. Everything here is a pure function of the source; per-slot and
//! per-cell work runs on rayon into position-ordered chunks, so results do not depend on the
//! thread count.

use crate::error::PolyError;
use crate::geom::{self, V3};
use crate::options::FaceMode;
use crate::resolver::EdgeResolver;
use rayon::prelude::*;
use sm_core::EntityId;
use sm_mesh_api::{
    CellIx, CellShape, FaceIx, FaceKind, GeomTol, MeshDim, MeshEntityKind, MeshError, MeshRead, Side,
    VertexIx,
};
use smallvec::SmallVec;

/// Relative coplanarity tolerance of the split-edge test (fixed, not an option: exact boundary
/// tiling is kept).
pub(crate) const COPLANAR_REL: f64 = 1e-9;
/// Slots per parallel chunk.
const SLOT_CHUNK: usize = 1024;
/// Cells per parallel chunk.
const CELL_CHUNK: usize = 1024;
/// Marks "none" in `u32` columns.
pub(crate) const NONE: u32 = u32::MAX;

const PAIRS: [(usize, usize); 6] = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)];

/// Facts about one (edge, region) slot.
#[derive(Clone, Copy, Debug)]
pub(crate) struct SlotMeta {
    /// The ring is an open path (the edge lies on the region boundary).
    pub open: bool,
    /// Entering / exiting region-boundary face of an open path (`NONE` for a ring).
    pub f_in: u32,
    pub f_out: u32,
    /// The edge face is non-planar and replaced by hub triangles (`Planar` mode).
    pub hub: bool,
    /// Open path whose two boundary triangles are separated by a feature (split edge).
    pub split: bool,
    /// Model entity the edge midpoint lies on (open paths only).
    pub mid: Option<EntityId>,
}

/// One boundary polygon of a cell.
#[derive(Clone, Copy, Debug)]
pub(crate) struct FanMeta {
    /// Cell index.
    pub cell: u32,
    /// Number of points.
    pub len: u32,
    /// `TopoFace` the polygon lies on.
    pub cls: EntityId,
    /// The polygon is shared with the cell of the neighbouring region.
    pub iface: bool,
}

/// The analysed source.
pub(crate) struct Prep<'a> {
    pub src: &'a dyn MeshRead,
    pub nt: usize,
    pub nf: usize,
    pub nv: usize,
    pub ne: usize,
    pub tv: Vec<[u32; 4]>,
    pub treg: Vec<u32>,
    pub regions: Vec<EntityId>,
    pub tcent: Vec<V3>,
    pub ekeys: Vec<u64>,
    pub e_slot_off: Vec<u32>,
    pub s_edge: Vec<u32>,
    pub s_reg: Vec<u32>,
    pub s_off: Vec<u32>,
    pub ring: Vec<u32>,
    pub meta: Vec<SlotMeta>,
    /// Class of each edge midpoint (edges that are open in some region).
    pub mid_cls: Vec<Option<EntityId>>,
    /// Dual cells as `region << 32 | vertex`, ascending.
    pub ckeys: Vec<u64>,
    pub s_ca: Vec<u32>,
    pub s_cb: Vec<u32>,
    pub fan_metas: Vec<FanMeta>,
    pub fan_pts: Vec<u32>,
    pub split_fans: usize,
}

impl Prep<'_> {
    /// Virtual point ids: tets, boundary-triangle centroids, edge midpoints, primal vertices,
    /// edge hubs.
    pub fn p_tri(&self, f: u32) -> u32 {
        self.nt as u32 + f
    }
    pub fn p_mid(&self, e: u32) -> u32 {
        (self.nt + self.nf) as u32 + e
    }
    pub fn p_vert(&self, v: u32) -> u32 {
        (self.nt + self.nf + self.ne) as u32 + v
    }
    pub fn p_hub(&self, e: u32) -> u32 {
        (self.nt + self.nf + self.ne + self.nv) as u32 + e
    }
    pub fn n_virtual(&self) -> usize {
        self.nt + self.nf + 2 * self.ne + self.nv
    }

    /// Coordinates of a virtual point.
    pub fn coord(&self, vid: u32) -> V3 {
        coord_of(self, vid as usize)
    }

    /// The vertices of edge `e`.
    pub fn edge_ends(&self, e: u32) -> (u32, u32) {
        let k = self.ekeys[e as usize];
        ((k >> 32) as u32, k as u32)
    }

    /// Slots of edge `e`.
    pub fn slots_of_edge(&self, e: usize) -> std::ops::Range<usize> {
        self.e_slot_off[e] as usize..self.e_slot_off[e + 1] as usize
    }

    /// Slot of the (edge, region) pair, if any.
    fn slot_for(&self, a: u32, b: u32, region: u32) -> Option<usize> {
        let key = pair_key(a, b);
        let e = self.ekeys.binary_search(&key).ok()?;
        self.slots_of_edge(e).find(|&s| self.s_reg[s] == region)
    }
}

pub(crate) fn pair_key(a: u32, b: u32) -> u64 {
    let (lo, hi) = if a < b { (a, b) } else { (b, a) };
    (u64::from(lo) << 32) | u64::from(hi)
}

fn coord_of(p: &Prep<'_>, i: usize) -> V3 {
    let co = p.src.coords();
    let pt = |v: usize| [co.x[v], co.y[v], co.z[v]];
    if i < p.nt {
        return p.tcent[i];
    }
    let i = i - p.nt;
    if i < p.nf {
        let l = p.src.face_vertices(FaceIx::new(i));
        return geom::mean(&[pt(l[0].get()), pt(l[1].get()), pt(l[2].get())]);
    }
    let i = i - p.nf;
    if i < p.ne {
        let (a, b) = p.edge_ends(i as u32);
        let (pa, pb) = (pt(a as usize), pt(b as usize));
        return [(pa[0] + pb[0]) * 0.5, (pa[1] + pb[1]) * 0.5, (pa[2] + pb[2]) * 0.5];
    }
    let i = i - p.ne;
    if i < p.nv {
        return pt(i);
    }
    let i = i - p.nv;
    coord_of(p, p.nt + p.nf + i)
}

fn too_large() -> PolyError {
    PolyError::Mesh(MeshError::TooLarge)
}

/// Which face of tet `t` carries edge `a -> b` in its outward loop (`exit`) and which carries
/// `b -> a` (`entry`). A rotation about `a -> b` (right-handed) leaves a tet through `exit`.
fn exit_entry(src: &dyn MeshRead, t: u32, a: u32, b: u32) -> Option<(sm_mesh_api::SignedFace, sm_mesh_api::SignedFace)> {
    let mut exit = None;
    let mut entry = None;
    for &sf in src.cell_faces(CellIx(t)) {
        let l = src.face_vertices(sf.face());
        let n = l.len();
        let mut dir = 0i8;
        for i in 0..n {
            let (x, y) = (l[i].0, l[(i + 1) % n].0);
            if x == a && y == b {
                dir = 1;
                break;
            }
            if x == b && y == a {
                dir = -1;
                break;
            }
        }
        if dir == 0 {
            continue;
        }
        if sf.side() == Side::Right {
            dir = -dir;
        }
        if dir == 1 {
            exit = Some(sf);
        } else {
            entry = Some(sf);
        }
    }
    Some((exit?, entry?))
}

struct SlotChunk {
    ring: Vec<u32>,
    meta: Vec<SlotMeta>,
}

struct FanChunk {
    metas: Vec<FanMeta>,
    pts: Vec<u32>,
    split_fans: usize,
}

impl<'a> Prep<'a> {
    /// Validate the source and run every analysis stage.
    pub fn build(
        src: &'a dyn MeshRead,
        mode: FaceMode,
        resolver: &dyn EdgeResolver,
    ) -> Result<Self, PolyError> {
        if src.dim() == MeshDim::Two {
            return Err(PolyError::Dim2);
        }
        let nt = src.count(MeshEntityKind::Cell);
        let nf = src.count(MeshEntityKind::Face);
        let nv = src.count(MeshEntityKind::Vertex);
        if nt == 0 {
            return Err(PolyError::Mesh(MeshError::State("the mesh has no cells".into())));
        }
        if let Some(c) = (0..nt).find(|&c| src.cell_shape(CellIx::new(c)) != CellShape::Tet) {
            return Err(PolyError::NotTet { cell: CellIx::new(c) });
        }
        if let Some(f) = (0..nf).find(|&f| {
            !matches!(src.face_kind(FaceIx::new(f)), FaceKind::Interior | FaceKind::Boundary)
        }) {
            return Err(PolyError::UnsupportedFaceKind { face: FaceIx::new(f) });
        }
        if 6 * nt >= NONE as usize || nt + nf + 2 * (6 * nt) + nv >= NONE as usize {
            return Err(too_large());
        }

        // Tets and regions.
        let tv: Vec<[u32; 4]> = (0..nt)
            .map(|c| {
                src.cell_corners(CellIx::new(c)).and_then(|k| {
                    (k.len() == 4).then(|| [k[0].0, k[1].0, k[2].0, k[3].0])
                })
            })
            .collect::<Option<_>>()
            .ok_or(PolyError::NotTet { cell: CellIx::new(0) })?;
        let cls: Vec<EntityId> = (0..nt).map(|c| src.cell_class(CellIx::new(c)).entity).collect();
        let mut regions = cls.clone();
        regions.sort_unstable();
        regions.dedup();
        let treg: Vec<u32> = cls
            .iter()
            .map(|e| regions.binary_search(e).map_or(0, |r| r as u32))
            .collect();
        let co = src.coords();
        let tcent: Vec<V3> = tv
            .par_iter()
            .map(|t| {
                let p = |i: usize| [co.x[t[i] as usize], co.y[t[i] as usize], co.z[t[i] as usize]];
                geom::mean(&[p(0), p(1), p(2), p(3)])
            })
            .collect();

        // (edge, region) slots.
        let mut raw: Vec<(u64, u32, u32)> = (0..6 * nt)
            .into_par_iter()
            .map(|i| {
                let (t, k) = (i / 6, i % 6);
                let (x, y) = PAIRS[k];
                (pair_key(tv[t][x], tv[t][y]), treg[t], t as u32)
            })
            .collect();
        raw.par_sort_unstable();
        let mut ekeys: Vec<u64> = Vec::new();
        let mut e_slot_off: Vec<u32> = Vec::new();
        let (mut s_edge, mut s_reg, mut s_off) = (Vec::new(), Vec::new(), Vec::new());
        let mut s_tets: Vec<u32> = Vec::with_capacity(raw.len());
        for (i, &(key, r, t)) in raw.iter().enumerate() {
            let new_edge = i == 0 || key != raw[i - 1].0;
            if new_edge {
                e_slot_off.push(s_edge.len() as u32);
                ekeys.push(key);
            }
            if new_edge || r != raw[i - 1].1 {
                s_edge.push((ekeys.len() - 1) as u32);
                s_reg.push(r);
                s_off.push(i as u32);
            }
            s_tets.push(t);
        }
        e_slot_off.push(s_edge.len() as u32);
        s_off.push(raw.len() as u32);
        drop(raw);
        let ne = ekeys.len();
        let ns = s_edge.len();

        let mut prep = Prep {
            src,
            nt,
            nf,
            nv,
            ne,
            tv,
            treg,
            regions,
            tcent,
            ekeys,
            e_slot_off,
            s_edge,
            s_reg,
            s_off,
            ring: Vec::new(),
            meta: Vec::new(),
            mid_cls: vec![None; ne],
            ckeys: Vec::new(),
            s_ca: Vec::new(),
            s_cb: Vec::new(),
            fan_metas: Vec::new(),
            fan_pts: Vec::new(),
            split_fans: 0,
        };

        // Rings, split edges, feature classes, hub decisions.
        let chunks: Vec<Result<SlotChunk, PolyError>> = (0..ns.div_ceil(SLOT_CHUNK))
            .into_par_iter()
            .map(|c| prep.slot_chunk(c * SLOT_CHUNK, ((c + 1) * SLOT_CHUNK).min(ns), &s_tets, mode, resolver))
            .collect();
        let mut ring = Vec::with_capacity(s_tets.len());
        let mut meta = Vec::with_capacity(ns);
        for ch in chunks {
            let ch = ch?;
            ring.extend_from_slice(&ch.ring);
            meta.extend_from_slice(&ch.meta);
        }
        prep.ring = ring;
        prep.meta = meta;
        for s in 0..ns {
            if let (true, Some(c)) = (prep.meta[s].open, prep.meta[s].mid) {
                let e = prep.s_edge[s] as usize;
                match prep.mid_cls[e] {
                    None => prep.mid_cls[e] = Some(c),
                    Some(p) if p == c => {}
                    Some(p) => return Err(PolyError::AmbiguousEdge { a: p.min(c), b: p.max(c) }),
                }
            }
        }

        prep.build_cells();
        prep.build_fans(mode)?;
        Ok(prep)
    }

    fn slot_chunk(
        &self,
        s0: usize,
        s1: usize,
        s_tets: &[u32],
        mode: FaceMode,
        resolver: &dyn EdgeResolver,
    ) -> Result<SlotChunk, PolyError> {
        let mut out = SlotChunk { ring: Vec::with_capacity((self.s_off[s1] - self.s_off[s0]) as usize), meta: Vec::with_capacity(s1 - s0) };
        for s in s0..s1 {
            let tets = &s_tets[self.s_off[s] as usize..self.s_off[s + 1] as usize];
            let (a, b) = self.edge_ends(self.s_edge[s]);
            let region = self.s_reg[s];
            let bad = || PolyError::OpenRing { edge: [VertexIx(a), VertexIx(b)] };
            let same_region = |sf: sm_mesh_api::SignedFace| -> Option<u32> {
                self.src
                    .across(sf)
                    .map(|n| n.0)
                    .filter(|&n| self.treg[n as usize] == region)
            };
            // Start: the tet entered through a region-boundary face, else the lowest tet.
            let mut start = tets[0];
            let mut open = false;
            for &t in tets {
                let (_, en) = exit_entry(self.src, t, a, b).ok_or_else(bad)?;
                if same_region(en).is_none() {
                    if open {
                        return Err(bad());
                    }
                    open = true;
                    start = t;
                }
            }
            let ring_start = out.ring.len();
            let mut t = start;
            let mut f_out = NONE;
            loop {
                out.ring.push(t);
                let (ex, _) = exit_entry(self.src, t, a, b).ok_or_else(bad)?;
                match same_region(ex) {
                    Some(n) => {
                        if !open && n == start {
                            break;
                        }
                        if out.ring.len() - ring_start >= tets.len() {
                            return Err(bad());
                        }
                        t = n;
                    }
                    None => {
                        if !open {
                            return Err(bad());
                        }
                        f_out = ex.face().0;
                        break;
                    }
                }
            }
            if out.ring.len() - ring_start != tets.len() {
                return Err(bad());
            }
            let mut m = SlotMeta { open, f_in: NONE, f_out, hub: false, split: false, mid: None };
            #[cfg(debug_assertions)]
            self.assert_ring_orientation(&out.ring[ring_start..], s, a, b, open, f_out, start);
            if open {
                let (_, en) = exit_entry(self.src, start, a, b).ok_or_else(bad)?;
                m.f_in = en.face().0;
                self.classify_open(&mut m, a, b, resolver)?;
            }
            if mode == FaceMode::Planar {
                let ring = &out.ring[ring_start..];
                let k = ring.len();
                if k + if open { 3 } else { 0 } > 3 {
                    let mut pts: SmallVec<[V3; 24]> = SmallVec::new();
                    if open {
                        pts.push(self.coord(self.p_tri(m.f_in)));
                    }
                    pts.extend(ring.iter().map(|&t| self.tcent[t as usize]));
                    if open {
                        pts.push(self.coord(self.p_tri(m.f_out)));
                        pts.push(self.coord(self.p_mid(self.s_edge[s])));
                    }
                    m.hub = geom::nonplanarity(&pts) > GeomTol::default().planarity;
                }
            }
            out.meta.push(m);
        }
        Ok(out)
    }

    /// Debug check of the topologically derived orientation: the centroid polygon of the slot
    /// winds counterclockwise about `a -> b`, so its Newell normal has a non-negative component
    /// along `b - a` (inverted or flat input can only make it smaller).
    #[cfg(debug_assertions)]
    #[allow(clippy::too_many_arguments)]
    fn assert_ring_orientation(&self, ring: &[u32], s: usize, a: u32, b: u32, open: bool, f_out: u32, start: u32) {
        let mut pts: SmallVec<[V3; 24]> = SmallVec::new();
        if open {
            let (_, en) = exit_entry(self.src, start, a, b).expect("entry face");
            pts.push(self.coord(self.p_tri(en.face().0)));
        }
        pts.extend(ring.iter().map(|&t| self.tcent[t as usize]));
        if open {
            pts.push(self.coord(self.p_tri(f_out)));
            pts.push(self.coord(self.p_mid(self.s_edge[s])));
        }
        let mut n = [0.0; 3];
        for i in 0..pts.len() {
            let c = geom::cross(pts[i], pts[(i + 1) % pts.len()]);
            n = [n[0] + c[0], n[1] + c[1], n[2] + c[2]];
        }
        let (pa, pb) = (self.coord(self.p_vert(a)), self.coord(self.p_vert(b)));
        let along = geom::dot(n, geom::sub(pb, pa));
        let scale = geom::norm(n) * geom::norm(geom::sub(pb, pa));
        debug_assert!(along >= -1e-9 * scale, "ring of edge ({a}, {b}) winds clockwise about a -> b");
    }

    /// Split flag and midpoint class of an open path.
    fn classify_open(
        &self,
        m: &mut SlotMeta,
        a: u32,
        b: u32,
        resolver: &dyn EdgeResolver,
    ) -> Result<(), PolyError> {
        let src = self.src;
        let (fi, fo) = (FaceIx(m.f_in), FaceIx(m.f_out));
        let (ci, co_) = (src.face_class(fi), src.face_class(fo));
        for (f, c) in [(fi, ci), (fo, co_)] {
            if c.dim() != Some(2) {
                return Err(PolyError::BadClassification { face: f });
            }
        }
        let table = src.edges().and_then(|t| {
            t.find(VertexIx(a), VertexIx(b)).map(|e| t.class[e.get()])
        });
        let feature = table.filter(|c| c.dim() == Some(1)).map(|c| c.entity);
        let mid = if let Some(e) = feature {
            e
        } else if ci.entity == co_.entity {
            ci.entity
        } else {
            let (lo, hi) = (ci.entity.min(co_.entity), ci.entity.max(co_.entity));
            let ends = [src.vertex_class(VertexIx(a)), src.vertex_class(VertexIx(b))];
            resolver
                .resolve(lo, hi, ends)
                .ok_or(PolyError::AmbiguousEdge { a: lo, b: hi })?
        };
        m.mid = Some(mid);
        m.split = if ci.entity != co_.entity || feature.is_some() {
            true
        } else {
            let (lo, hi) = if fi.0 < fo.0 { (fi, fo) } else { (fo, fi) };
            !self.triangles_coplanar(lo, hi, a, b)
        };
        Ok(())
    }

    /// The two triangles that share edge `a-b` lie in one plane (within [`COPLANAR_REL`]).
    fn triangles_coplanar(&self, f1: FaceIx, f2: FaceIx, a: u32, b: u32) -> bool {
        let co = self.src.coords();
        let pt = |v: u32| [co.x[v as usize], co.y[v as usize], co.z[v as usize]];
        let other = |f: FaceIx| -> Option<u32> {
            self.src.face_vertices(f).iter().map(|v| v.0).find(|&v| v != a && v != b)
        };
        let (Some(c), Some(d)) = (other(f1), other(f2)) else {
            return false;
        };
        let (lo, hi) = if a < b { (a, b) } else { (b, a) };
        geom::coplanar(pt(lo), pt(hi), pt(c), pt(d), COPLANAR_REL)
    }

    /// The dual cells and, per slot, the cells of its two end vertices.
    fn build_cells(&mut self) {
        let mut keys: Vec<u64> = (0..4 * self.nt)
            .into_par_iter()
            .map(|i| (u64::from(self.treg[i / 4]) << 32) | u64::from(self.tv[i / 4][i % 4]))
            .collect();
        keys.par_sort_unstable();
        keys.dedup();
        self.ckeys = keys;
        let cell = |r: u32, v: u32| -> u32 {
            self.ckeys
                .binary_search(&((u64::from(r) << 32) | u64::from(v)))
                .map_or(NONE, |c| c as u32)
        };
        let ends: Vec<(u32, u32)> = (0..self.s_edge.len())
            .into_par_iter()
            .map(|s| {
                let (a, b) = self.edge_ends(self.s_edge[s]);
                (cell(self.s_reg[s], a), cell(self.s_reg[s], b))
            })
            .collect();
        self.s_ca = ends.iter().map(|e| e.0).collect();
        self.s_cb = ends.iter().map(|e| e.1).collect();
    }

    /// Region-boundary triangle entries `(face, region)`, ascending by face.
    fn boundary_entries(&self) -> Vec<(u32, u32)> {
        let src = self.src;
        let mut out = Vec::new();
        for f in 0..self.nf {
            let fi = FaceIx::new(f);
            let rl = self.treg[src.face_left(fi).get()];
            match src.face_right(fi) {
                None => out.push((f as u32, rl)),
                Some(r) => {
                    let rr = self.treg[r.get()];
                    if rl != rr {
                        out.push((f as u32, rl.min(rr)));
                        out.push((f as u32, rl.max(rr)));
                    }
                }
            }
        }
        out
    }

    /// Boundary fans of every cell that has region-boundary triangles.
    fn build_fans(&mut self, mode: FaceMode) -> Result<(), PolyError> {
        let entries = self.boundary_entries();
        // (cell, entry) incidences, sorted.
        let mut inc: Vec<u64> = Vec::with_capacity(entries.len() * 3);
        for (ei, &(f, r)) in entries.iter().enumerate() {
            for v in self.src.face_vertices(FaceIx(f)) {
                let c = self
                    .ckeys
                    .binary_search(&((u64::from(r) << 32) | u64::from(v.0)))
                    .map_err(|_| PolyError::UnsupportedFeature { vertex: *v })?;
                inc.push(((c as u64) << 32) | ei as u64);
            }
        }
        inc.par_sort_unstable();
        // Group by cell.
        let mut groups: Vec<(u32, usize, usize)> = Vec::new();
        let mut i = 0;
        while i < inc.len() {
            let c = (inc[i] >> 32) as u32;
            let mut j = i;
            while j < inc.len() && (inc[j] >> 32) as u32 == c {
                j += 1;
            }
            groups.push((c, i, j));
            i = j;
        }
        let chunks: Vec<Result<FanChunk, PolyError>> = (0..groups.len().div_ceil(CELL_CHUNK))
            .into_par_iter()
            .map(|c| {
                let g = &groups[c * CELL_CHUNK..((c + 1) * CELL_CHUNK).min(groups.len())];
                self.fan_chunk(g, &inc, &entries, mode)
            })
            .collect();
        for ch in chunks {
            let ch = ch?;
            self.fan_metas.extend_from_slice(&ch.metas);
            self.fan_pts.extend_from_slice(&ch.pts);
            self.split_fans += ch.split_fans;
        }
        Ok(())
    }

    fn fan_chunk(
        &self,
        groups: &[(u32, usize, usize)],
        inc: &[u64],
        entries: &[(u32, u32)],
        mode: FaceMode,
    ) -> Result<FanChunk, PolyError> {
        let mut out = FanChunk { metas: Vec::new(), pts: Vec::new(), split_fans: 0 };
        for &(cell, i0, i1) in groups {
            let key = self.ckeys[cell as usize];
            let (region, v) = ((key >> 32) as u32, key as u32);
            // Outward triangle loops rotated to start at v: (face, p, q).
            let mut tris: SmallVec<[(u32, u32, u32); 16]> = SmallVec::new();
            for &x in &inc[i0..i1] {
                let (f, r) = entries[(x & 0xffff_ffff) as usize];
                debug_assert_eq!(r, region);
                let fi = FaceIx(f);
                let l = self.src.face_vertices(fi);
                let tet_is_left = self.treg[self.src.face_left(fi).get()] == region;
                let o: [u32; 3] = if tet_is_left { [l[0].0, l[1].0, l[2].0] } else { [l[2].0, l[1].0, l[0].0] };
                let k = o.iter().position(|&x| x == v).unwrap_or(0);
                tris.push((f, o[(k + 1) % 3], o[(k + 2) % 3]));
            }
            self.fans_of_cell(cell, v, region, &tris, mode, &mut out)?;
        }
        Ok(out)
    }

    fn fans_of_cell(
        &self,
        cell: u32,
        v: u32,
        region: u32,
        tris: &[(u32, u32, u32)],
        mode: FaceMode,
        out: &mut FanChunk,
    ) -> Result<(), PolyError> {
        let n = tris.len();
        let bad = || PolyError::OpenRing { edge: [VertexIx(v), VertexIx(v)] };
        // Slot of edge (v, w) in this region, for p and q of each triangle.
        let mut sp: SmallVec<[usize; 16]> = SmallVec::new();
        let mut sq: SmallVec<[usize; 16]> = SmallVec::new();
        for &(_, p, q) in tris {
            let a = self.slot_for(v, p, region).ok_or_else(bad)?;
            let b = self.slot_for(v, q, region).ok_or_else(bad)?;
            if !self.meta[a].open || !self.meta[b].open {
                return Err(bad());
            }
            sp.push(a);
            sq.push(b);
        }
        let split_p = |k: usize| self.meta[sp[k]].split;
        let split_q = |k: usize| self.meta[sq[k]].split;
        let nsplit = (0..n).filter(|&k| split_p(k)).count();
        if nsplit == 1 {
            return Err(PolyError::UnsupportedFeature { vertex: VertexIx(v) });
        }
        // next[k]: the triangle that continues the fan counterclockwise (loop v, q_k, r).
        let mut next: SmallVec<[usize; 16]> = SmallVec::new();
        for k in 0..n {
            if split_q(k) {
                next.push(usize::MAX);
            } else {
                let j = (0..n).find(|&j| j != k && tris[j].1 == tris[k].2).ok_or_else(bad)?;
                next.push(j);
            }
        }
        let mut visited: SmallVec<[bool; 16]> = SmallVec::from_elem(false, n);
        let mut chains: SmallVec<[SmallVec<[usize; 8]>; 4]> = SmallVec::new();
        for k in 0..n {
            if !split_p(k) {
                continue;
            }
            let mut chain: SmallVec<[usize; 8]> = SmallVec::new();
            let mut cur = k;
            loop {
                if visited[cur] {
                    return Err(bad());
                }
                visited[cur] = true;
                chain.push(cur);
                if next[cur] == usize::MAX {
                    break;
                }
                cur = next[cur];
            }
            chains.push(chain);
        }
        let n_open = chains.len();
        for k in 0..n {
            if visited[k] {
                continue;
            }
            let mut chain: SmallVec<[usize; 8]> = SmallVec::new();
            let mut cur = k;
            loop {
                if visited[cur] {
                    if cur != k {
                        return Err(bad());
                    }
                    break;
                }
                visited[cur] = true;
                chain.push(cur);
                if next[cur] == usize::MAX {
                    return Err(bad());
                }
                cur = next[cur];
            }
            chains.push(chain);
        }
        for (ci, chain) in chains.iter().enumerate() {
            let is_open = ci < n_open;
            let mut pts: SmallVec<[u32; 32]> = SmallVec::new();
            if is_open {
                pts.push(self.p_vert(v));
            }
            for (pos, &k) in chain.iter().enumerate() {
                if pos == 0 {
                    pts.push(self.p_mid(self.s_edge[sp[k]]));
                }
                pts.push(self.p_tri(tris[k].0));
                if is_open || pos + 1 < chain.len() {
                    pts.push(self.p_mid(self.s_edge[sq[k]]));
                }
            }
            let f0 = FaceIx(tris[chain[0]].0);
            let cls = self.src.face_class(f0).entity;
            let iface = self.src.face_kind(f0) == FaceKind::Interior;
            let mut kites = false;
            if mode == FaceMode::Planar && pts.len() > 3 {
                let coords: SmallVec<[V3; 32]> = canonical_cycle(&pts).iter().map(|&p| self.coord(p)).collect();
                kites = geom::nonplanarity(&coords) > GeomTol::default().planarity;
            }
            if kites {
                out.split_fans += 1;
                for &k in chain {
                    let kite = [
                        self.p_vert(v),
                        self.p_mid(self.s_edge[sp[k]]),
                        self.p_tri(tris[k].0),
                        self.p_mid(self.s_edge[sq[k]]),
                    ];
                    out.metas.push(FanMeta { cell, len: 4, cls, iface });
                    out.pts.extend_from_slice(&kite);
                }
            } else {
                out.metas.push(FanMeta { cell, len: pts.len() as u32, cls, iface });
                out.pts.extend_from_slice(&pts);
            }
        }
        Ok(())
    }
}

/// The cycle rotated to start at its smallest point id and run towards the smaller neighbour,
/// so the two cells of an interface polygon (which see it in opposite orders) evaluate
/// identical arithmetic.
fn canonical_cycle(pts: &[u32]) -> SmallVec<[u32; 32]> {
    let n = pts.len();
    let k = (0..n).min_by_key(|&i| pts[i]).unwrap_or(0);
    let fwd = pts[(k + 1) % n] <= pts[(k + n - 1) % n];
    (0..n)
        .map(|i| if fwd { pts[(k + i) % n] } else { pts[(k + n - i) % n] })
        .collect()
}
