//! Bulk insertion of template cells: the face table is built by a parallel sort of face records
//! instead of one hash probe per face.
//!
//! The result is exactly what adding the same cells one by one with
//! `add_cell_from_vertices` would give (same provisional face numbering, same cell face lists,
//! same errors for the lowest offending cell), so `finish` and every later call behave alike.

use crate::builder::{FLAG_TEMPLATE, MeshBuilder};
use crate::class::ClassCol;
use crate::util::{CHUNK, ErrSink, chunk_bounds, nil, prefix, range, split_by_offsets};
use rayon::prelude::*;
use sm_core::Point3;
use sm_mesh_api::{
    CellIx, CellShape, Classification, FaceIx, LoopRelation, MAX_FACES, MeshError, Result,
    SignedFace, VertexIx, loop_relation,
};
use std::ops::Range;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering::Relaxed};

/// One face-record: the sorted vertex set of local face `r - base(c)` of cell `c`.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Rec {
    key: [u32; 4],
    r: u32,
    c: u32,
}

/// Error candidate found while resolving a run of equal face keys.
#[derive(Clone, Copy)]
struct Bad {
    /// Record of the offending cell (the lowest wins).
    r: u32,
    /// Head record of the face it clashed with (face id is resolved on the error path).
    head: u32,
    non_manifold: bool,
}

/// Inputs shared by the parallel stages.
struct Input<'a> {
    shapes: &'a [CellShape],
    offsets: &'a [u64],
    verts: &'a [VertexIx],
    rbase: &'a [u32],
}

impl Input<'_> {
    /// Outward loop of the local face behind record `rec`.
    fn loop_of(&self, rec: &Rec) -> ([VertexIx; 4], usize) {
        let c = rec.c as usize;
        let l = (rec.r - self.rbase[c]) as usize;
        let lf = self.shapes[c].local_faces()[l];
        let vs = &self.verts[range(self.offsets, c)];
        let mut out = [VertexIx(0); 4];
        for (j, &i) in lf.iter().enumerate() {
            out[j] = vs[i as usize];
        }
        (out, lf.len())
    }
}

/// Resolve one run (records with the same vertex set, ascending `r`) into face heads and sides.
/// `hd[k]` receives `head_record << 1 | side` (side 1 = right of the head's loop).
fn resolve_run(inp: &Input<'_>, run: &[Rec], hd: &mut [u32], bad: &mut Option<Bad>) {
    let mut faces: smallvec::SmallVec<[(usize, u8); 4]> = smallvec::SmallVec::new();
    let mut loops: smallvec::SmallVec<[([VertexIx; 4], usize); 4]> = smallvec::SmallVec::new();
    for (k, rec) in run.iter().enumerate() {
        let (lp, n) = inp.loop_of(rec);
        let found = faces.iter().position(|&(h, _)| {
            let (hl, hn) = loops[h];
            loop_relation(&lp[..n], &hl[..hn]).is_some()
        });
        match found {
            None => {
                faces.push((k, 1));
                loops.push((lp, n));
                hd[k] = rec.r << 1;
            }
            Some(fi) => {
                let (h, mask) = faces[fi];
                let (hl, hn) = loops[h];
                // Same cyclic order as the head's loop: this cell sees it outward (left side).
                let left = loop_relation(&lp[..n], &hl[..hn]) == Some(LoopRelation::Same);
                let bit = if left { 1 } else { 2 };
                if mask == 3 || mask & bit != 0 {
                    if bad.is_none_or(|b| rec.r < b.r) {
                        *bad = Some(Bad { r: rec.r, head: run[h].r, non_manifold: mask == 3 });
                    }
                    hd[k] = rec.r << 1;
                } else {
                    faces[fi].1 |= bit;
                    hd[k] = (run[h].r << 1) | u32::from(!left);
                }
            }
        }
    }
}

impl MeshBuilder {
    /// Append `n` vertices at once; `f(i)` gives the point of the `i`-th new vertex. Computed in
    /// parallel; returns the range of new vertex indices.
    pub fn add_vertices_with(&mut self, n: usize, f: impl Fn(usize) -> Point3 + Sync) -> Result<Range<usize>> {
        let start = self.x.len();
        if start + n + self.extra.len() >= u32::MAX as usize - 1 {
            return Err(MeshError::TooLarge);
        }
        let pts: Vec<Point3> = (0..n).into_par_iter().map(|i| f(i)).collect();
        self.x.extend(pts.iter().map(|p| p.x));
        self.y.extend(pts.iter().map(|p| p.y));
        self.z.extend(pts.iter().map(|p| p.z));
        self.vclass.ent.resize(start + n, nil());
        self.vcell.resize(start + n, CellIx::NONE);
        Ok(start..start + n)
    }

    /// Add `n = verts.len() / shape.vertex_count()` cells of one template `shape`, `verts` holding
    /// each cell's vertices in template order. See [`MeshBuilder::add_cells_bulk`].
    pub fn add_uniform_cells(&mut self, shape: CellShape, verts: &[VertexIx]) -> Result<Range<usize>> {
        let k = shape.vertex_count().ok_or_else(|| {
            MeshError::State(format!("{shape:?} is not a template shape: bulk insertion needs a fixed vertex count"))
        })?;
        if verts.len() % k != 0 {
            return Err(MeshError::State(format!("{shape:?} needs {k} vertices per cell, got {}", verts.len())));
        }
        let n = verts.len() / k;
        let shapes = vec![shape; n];
        let offsets: Vec<u64> = (0..=n as u64).map(|i| i * k as u64).collect();
        self.add_cells_bulk(&shapes, &offsets, verts)
    }

    /// Add many template cells at once and build their face table in parallel.
    ///
    /// Cell `c` has shape `shapes[c]` and vertices `verts[offsets[c]..offsets[c + 1]]` in the
    /// template order of its shape. The builder must hold no faces or cells yet (vertices may
    /// exist). The outcome equals calling [`sm_mesh_api::MeshBuild::add_cell_from_vertices`] for
    /// every cell in order: same provisional face numbering (order of first appearance), same
    /// cell face lists, same error for the lowest offending cell. On `Err` the builder is
    /// unchanged. Returns the range of new cell indices.
    pub fn add_cells_bulk(
        &mut self,
        shapes: &[CellShape],
        offsets: &[u64],
        verts: &[VertexIx],
    ) -> Result<Range<usize>> {
        let n = shapes.len();
        if self.nfaces() != 0 || self.ncells() != 0 {
            return Err(MeshError::State("bulk insertion needs a builder without faces or cells".into()));
        }
        if offsets.len() != n + 1
            || offsets[0] != 0
            || offsets[n] != verts.len() as u64
            || offsets.windows(2).any(|w| w[0] > w[1])
        {
            return Err(MeshError::State("offsets must be a CSR array over `verts`".into()));
        }
        if n >= u32::MAX as usize - 1 {
            return Err(MeshError::TooLarge);
        }
        let dim = self.dim;
        let nv = self.x.len();

        // Validate cells (the lowest bad cell is reported, as the incremental path would).
        let errs = ErrSink::default();
        (0..n).into_par_iter().for_each(|c| {
            let vs = &verts[range(offsets, c)];
            let shape = shapes[c];
            if shape.vertex_count() != Some(vs.len()) {
                errs.push(c, MeshError::State(format!(
                    "{shape:?} needs {:?} vertices, got {}",
                    shape.vertex_count(),
                    vs.len()
                )));
            } else if shape.dim() != dim {
                errs.push(c, MeshError::State(format!("{shape:?} cells do not belong in a {dim:?} mesh")));
            } else if let Some(v) = vs.iter().find(|v| v.get() >= nv) {
                errs.push(c, MeshError::UnknownVertex(*v));
            } else if (0..vs.len()).any(|i| (i + 1..vs.len()).any(|j| vs[i] == vs[j])) {
                errs.push(c, MeshError::InvalidLoop("needs at least 3 distinct vertices".into()));
            }
        });
        if let Some(e) = errs.take_first() {
            return Err(e);
        }

        // Record bases: cell c owns records rbase[c]..rbase[c + 1], one per local face.
        let counts: Vec<u32> = shapes.par_iter().map(|s| s.local_faces().len() as u32).collect();
        let total: u64 = counts.iter().map(|&k| u64::from(k)).sum();
        if total >= 1 << 31 {
            return Err(MeshError::TooLarge);
        }
        let rbase: Vec<u32> = prefix(&counts).into_iter().map(|x| x as u32).collect();
        let inp = Input { shapes, offsets, verts, rbase: &rbase };

        // Sorted face records.
        let rb = rbase.as_slice();
        let mut recs: Vec<Rec> = (0..n)
            .into_par_iter()
            .flat_map_iter(move |c| {
                let vs = &verts[range(offsets, c)];
                shapes[c].local_faces().iter().enumerate().map(move |(l, lf)| {
                    let mut key = [u32::MAX; 4];
                    for (j, &i) in lf.iter().enumerate() {
                        key[j] = vs[i as usize].0;
                    }
                    key[..lf.len()].sort_unstable();
                    Rec { key, r: rb[c] + l as u32, c: c as u32 }
                })
            })
            .collect();
        recs.par_sort_unstable();
        let nr = recs.len();

        // Chunk the sorted records at run boundaries and resolve every run.
        let mut bounds = chunk_bounds(nr, CHUNK * 8);
        for b in bounds.iter_mut().skip(1) {
            while *b < nr && recs[*b].key == recs[*b - 1].key {
                *b += 1;
            }
        }
        bounds.dedup();
        let mut hd = vec![0u32; nr];
        let mut hd_chunks: Vec<&mut [u32]> = Vec::with_capacity(bounds.len());
        {
            let mut rest = hd.as_mut_slice();
            for w in bounds.windows(2) {
                let (a, b) = rest.split_at_mut(w[1] - w[0]);
                hd_chunks.push(a);
                rest = b;
            }
        }
        let bads: Vec<Option<Bad>> = hd_chunks
            .into_par_iter()
            .zip(bounds.par_windows(2))
            .map(|(hd, w)| {
                let chunk = &recs[w[0]..w[1]];
                let mut bad = None;
                let mut s = 0;
                while s < chunk.len() {
                    let mut e = s + 1;
                    while e < chunk.len() && chunk[e].key == chunk[s].key {
                        e += 1;
                    }
                    if e - s == 1 {
                        hd[s] = chunk[s].r << 1;
                    } else {
                        resolve_run(&inp, &chunk[s..e], &mut hd[s..e], &mut bad);
                    }
                    s = e;
                }
                bad
            })
            .collect();
        let is_head = |i: usize| hd[i] >> 1 == recs[i].r;

        // Face heads in record order: a bitmap whose set bits are the head records.
        let words: Vec<AtomicU64> = (0..nr / 64 + 1).map(|_| AtomicU64::new(0)).collect();
        (0..nr).into_par_iter().filter(|&i| is_head(i)).for_each(|i| {
            let r = recs[i].r as usize;
            words[r >> 6].fetch_or(1 << (r & 63), Relaxed);
        });
        let words: Vec<u64> = words.into_iter().map(AtomicU64::into_inner).collect();
        let mut before = Vec::with_capacity(words.len());
        let mut acc = 0u32;
        for w in &words {
            before.push(acc);
            acc += w.count_ones();
        }
        let nfaces = acc as usize;
        let rank = |r: u32| -> u32 {
            let (w, b) = ((r >> 6) as usize, r & 63);
            before[w] + (words[w] & ((1u64 << b) - 1)).count_ones()
        };

        if let Some(bad) = bads.into_iter().flatten().min_by_key(|b| b.r) {
            let f = FaceIx(rank(bad.head));
            return Err(if bad.non_manifold {
                MeshError::NonManifoldFace(f)
            } else {
                MeshError::InconsistentOrientation(f)
            });
        }
        if nfaces > MAX_FACES {
            return Err(MeshError::TooLarge);
        }

        // Cell face lists (record index == slot in the cell CSR).
        let cell_raw: Vec<AtomicU32> = (0..nr).map(|_| AtomicU32::new(0)).collect();
        (0..nr).into_par_iter().for_each(|i| {
            let sf = SignedFace::new(
                FaceIx(rank(hd[i] >> 1)),
                if hd[i] & 1 == 1 { sm_mesh_api::Side::Right } else { sm_mesh_api::Side::Left },
            );
            cell_raw[recs[i].r as usize].store(sf.to_raw(), Relaxed);
        });
        let cell_sf: Vec<SignedFace> = cell_raw.par_iter().map(|a| SignedFace::from_raw(a.load(Relaxed))).collect();

        // Faces in provisional order: head records ascending.
        let mut head_r: Vec<u32> = Vec::with_capacity(nfaces);
        for (w, &word) in words.iter().enumerate() {
            let mut bits = word;
            while bits != 0 {
                head_r.push((w as u32) << 6 | bits.trailing_zeros());
                bits &= bits - 1;
            }
        }
        let cell_of = |r: u32| rbase.partition_point(|&b| b <= r) - 1;
        let lens: Vec<u32> = head_r
            .par_iter()
            .map(|&r| {
                let c = cell_of(r);
                shapes[c].local_faces()[(r - rbase[c]) as usize].len() as u32
            })
            .collect();
        let face_off = prefix(&lens);
        let mut face_verts = vec![VertexIx(0); face_off[nfaces] as usize];
        {
            let fb = chunk_bounds(nfaces, CHUNK);
            split_by_offsets(&mut face_verts, &face_off, &fb)
                .into_par_iter()
                .zip(fb.par_windows(2))
                .for_each(|(s, w)| {
                    let base = face_off[w[0]] as usize;
                    for j in w[0]..w[1] {
                        let r = head_r[j];
                        let c = cell_of(r);
                        let lf = shapes[c].local_faces()[(r - rbase[c]) as usize];
                        let vs = &verts[range(offsets, c)];
                        let at = face_off[j] as usize - base;
                        for (q, &i) in lf.iter().enumerate() {
                            s[at + q] = vs[i as usize];
                        }
                    }
                });
        }
        let mut face_cells = vec![[CellIx::NONE; 2]; nfaces];
        let mut face_sides = vec![1u8; nfaces];
        for (j, &r) in head_r.iter().enumerate() {
            face_cells[j][0] = cell_of(r) as u32;
        }
        for (i, rec) in recs.iter().enumerate() {
            if !is_head(i) {
                let j = rank(hd[i] >> 1) as usize;
                // The second record sits on the head's left or right (builder sides are relative
                // to the head's loop; `finish` normalises them).
                let side = (hd[i] & 1) as usize;
                face_cells[j][side] = rec.c;
                face_sides[j] |= if side == 0 { 1 } else { 2 };
            }
        }
        // First cell using each vertex.
        let first: Vec<AtomicU32> = (0..nv).map(|_| AtomicU32::new(CellIx::NONE)).collect();
        (0..n).into_par_iter().for_each(|c| {
            for v in &verts[range(offsets, c)] {
                first[v.get()].fetch_min(c as u32, Relaxed);
            }
        });

        // Commit.
        self.vcell = first.into_iter().map(AtomicU32::into_inner).collect();
        self.face_off = face_off;
        self.face_verts = face_verts;
        self.face_sides = face_sides;
        self.face_cells = face_cells;
        self.fclass = ClassCol { ent: vec![nil(); nfaces], ..ClassCol::default() };
        self.cell_off = rbase.iter().map(|&x| u64::from(x)).collect();
        self.cell_sf = cell_sf;
        self.corner_off = offsets.to_vec();
        self.corner_v = verts.to_vec();
        self.cell_flag = vec![FLAG_TEMPLATE; n];
        self.cclass = ClassCol { ent: vec![nil(); n], ..ClassCol::default() };
        self.parent = vec![CellIx::NONE; n];
        self.active = vec![true; n];
        self.reset_face_index();
        Ok(0..n)
    }

    /// Classify vertices in parallel: `f(v)` gives the class of vertex `v`, or `None` to leave it
    /// unset. Every class is checked like [`sm_mesh_api::MeshBuild::set_class`]; the lowest
    /// failing vertex is reported and nothing is changed on `Err`.
    pub fn classify_vertices_with(&mut self, f: impl Fn(usize) -> Option<Classification> + Sync) -> Result<()> {
        let n = self.x.len();
        let col = classify_col(n, &f, |_| Ok(()), &self.vclass)?;
        self.vclass = col;
        Ok(())
    }

    /// Classify cells in parallel (see [`MeshBuilder::classify_vertices_with`]); cells must lie on
    /// entities of the mesh dimension.
    pub fn classify_cells_with(&mut self, f: impl Fn(usize) -> Option<Classification> + Sync) -> Result<()> {
        let dim = self.dim;
        let check = move |c: &Classification| {
            if dim.accepts_cell(c.entity.kind()) {
                Ok(())
            } else {
                Err(MeshError::BadClassification(format!(
                    "cells of a {dim:?} mesh cannot be classified on {:?}",
                    c.entity.kind()
                )))
            }
        };
        let col = classify_col(self.ncells(), &f, check, &self.cclass)?;
        self.cclass = col;
        Ok(())
    }

    /// Classify (provisional) faces in parallel (see [`MeshBuilder::classify_vertices_with`]).
    pub fn classify_faces_with(&mut self, f: impl Fn(usize) -> Option<Classification> + Sync) -> Result<()> {
        let col = classify_col(self.nfaces(), &f, |_| Ok(()), &self.fclass)?;
        self.fclass = col;
        Ok(())
    }
}

/// New class column from `f`, keeping `old` where `f` gives `None`.
fn classify_col(
    n: usize,
    f: &(impl Fn(usize) -> Option<Classification> + Sync),
    extra: impl Fn(&Classification) -> Result<()> + Sync,
    old: &ClassCol,
) -> Result<ClassCol> {
    let errs = ErrSink::default();
    let new: Vec<Option<Classification>> = (0..n)
        .into_par_iter()
        .map(|i| {
            let c = f(i)?;
            match crate::builder::check_class(&c).and_then(|()| extra(&c)) {
                Ok(()) => Some(c),
                Err(e) => {
                    errs.push(i, e);
                    None
                }
            }
        })
        .collect();
    if let Some(e) = errs.take_first() {
        return Err(e);
    }
    let mut col = old.clone();
    for (i, c) in new.into_iter().enumerate() {
        if let Some(c) = c {
            col.set(i, c);
        }
    }
    Ok(col)
}
