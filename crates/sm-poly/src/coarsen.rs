//! Coarse meshes from a partition of the fine cells (op `mesh.agglomerate`).
//!
//! [`build_coarse`] turns "fine cell -> group" into a coarse mesh and the face map of a
//! `LevelMap`. A coarse face is a maximal set of edge-adjacent, coplanar fine faces with the same
//! group pair and class whose boundary is one simple cycle (else the fine faces stay separate
//! coarse faces). A vertex is dropped when it is collinear in every coarse face loop that holds
//! it and lies on no model vertex, so a merged face has no hanging vertices. Everything is serial
//! and ordered by fine index, so the output is deterministic.
//!
//! [`group_blocks`], [`group_keys`] and [`greedy`] produce the partitions; [`union_convex`] is
//! the merge gate (the `sm-rt` convex rule, same arithmetic as `sm_quality::cell_convexity`).

use crate::error::PolyError;
use crate::geom::{self, V3};
use sm_core::EntityId;
use sm_mesh_api::{
    CellIx, Classification, FaceIx, FaceKind, Finished, MeshBuild, MeshEntityKind, MeshEntityRef, MeshRead, NO_CELL, SerialAlloc, Side, SignedFace,
    VertexIx, cell_volume, cell_volume_centroid, face_area_vector,
};
use smallvec::SmallVec;
use std::collections::BTreeMap;

/// Relative planarity / convexity tolerance of the `sm-rt` rule.
const TOL: f64 = sm_quality::RT_PLANARITY_TOL;

fn pt(src: &dyn MeshRead, v: VertexIx) -> V3 {
    let p = src.point(v.get());
    [p.x, p.y, p.z]
}

fn bbox_diam(pts: &[V3]) -> f64 {
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for p in pts {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    geom::norm(geom::sub(hi, lo))
}

/// Fan-vertex-mean plane of a loop: (unit normal, apex, ok).
fn plane(l: &[V3]) -> (V3, V3, bool) {
    let apex = geom::mean(l);
    let mut a = [0.0; 3];
    for i in 0..l.len() {
        let c = geom::cross(geom::sub(l[i], apex), geom::sub(l[(i + 1) % l.len()], apex));
        a = [a[0] + 0.5 * c[0], a[1] + 0.5 * c[1], a[2] + 0.5 * c[2]];
    }
    let len = geom::norm(a);
    if len > 0.0 && len.is_finite() { ([a[0] / len, a[1] / len, a[2] / len], apex, true) } else { ([0.0; 3], apex, false) }
}

/// The `sm-rt` convex rule on the union of `cells`: every outer face planar, every vertex of the
/// union inside every outer face plane within `TOL * diam`. `inside` says whether a cell belongs
/// to the union (faces towards it are internal).
pub(crate) fn union_convex(src: &dyn MeshRead, cells: &[CellIx], inside: &dyn Fn(CellIx) -> bool) -> bool {
    let mut pts: SmallVec<[V3; 96]> = SmallVec::new();
    let mut spans: SmallVec<[(usize, usize, f64); 16]> = SmallVec::new();
    for &c in cells {
        for sf in src.cell_faces(c) {
            if src.across(*sf).is_some_and(&inside) {
                continue;
            }
            let l = src.face_vertices(sf.face());
            let s = pts.len();
            pts.extend(l.iter().map(|&v| pt(src, v)));
            spans.push((s, l.len(), if sf.side() == Side::Left { 1.0 } else { -1.0 }));
        }
    }
    if spans.is_empty() || !pts.iter().all(|q| q.iter().all(|x| x.is_finite())) {
        return false;
    }
    let diam = bbox_diam(&pts);
    let mut planes: SmallVec<[(V3, f64, f64); 16]> = SmallVec::new();
    for &(s, n, sign) in &spans {
        let l = &pts[s..s + n];
        let (nrm, apex, ok) = plane(l);
        if !ok {
            return false;
        }
        let d = geom::dot(nrm, apex);
        let dev = l.iter().map(|&p| (geom::dot(nrm, p) - d).abs()).fold(0.0, f64::max);
        if !(n == 3 || dev <= TOL * bbox_diam(l)) {
            return false;
        }
        planes.push((nrm, d, sign));
    }
    let excess = planes.iter().flat_map(|&(n, d, s)| pts.iter().map(move |&p| s * (geom::dot(n, p) - d))).fold(0.0f64, f64::max);
    excess <= TOL * diam
}

/// Dense group numbers (by first fine cell) from arbitrary labels.
fn densify(labels: &[u64]) -> Vec<u32> {
    let mut map: BTreeMap<u64, u32> = BTreeMap::new();
    let mut out = Vec::with_capacity(labels.len());
    // First-appearance order, not label order, so group numbers follow the fine cell order.
    let mut next = 0u32;
    for &l in labels {
        let g = *map.entry(l).or_insert_with(|| {
            next += 1;
            next - 1
        });
        out.push(g);
    }
    out
}

/// Split every label class into its face-connected components (ids by first fine cell).
pub(crate) fn components(src: &dyn MeshRead, label: &[u64]) -> Vec<u32> {
    let nc = src.count(MeshEntityKind::Cell);
    let mut comp = vec![u64::MAX; nc];
    let mut next = 0u64;
    let mut stack: Vec<usize> = Vec::new();
    for s in 0..nc {
        if comp[s] != u64::MAX {
            continue;
        }
        comp[s] = next;
        stack.push(s);
        while let Some(c) = stack.pop() {
            for sf in src.cell_faces(CellIx::new(c)) {
                if let Some(o) = src.across(*sf) {
                    let o = o.get();
                    if comp[o] == u64::MAX && label[o] == label[s] {
                        comp[o] = next;
                        stack.push(o);
                    }
                }
            }
        }
        next += 1;
    }
    densify(&comp)
}

/// Group members by group id.
pub(crate) fn members(group_of: &[u32], n: usize) -> Vec<Vec<CellIx>> {
    let mut m = vec![Vec::new(); n];
    for (c, &g) in group_of.iter().enumerate() {
        m[g as usize].push(CellIx::new(c));
    }
    m
}

/// Blocks: label = (region, axis-aligned bin of the centroid), then connected components.
pub(crate) fn group_blocks(src: &dyn MeshRead, region: &[u32], size: [f64; 3], origin: [f64; 3]) -> Vec<u32> {
    let nc = src.count(MeshEntityKind::Cell);
    let mut ids: BTreeMap<(u32, [i64; 3]), u64> = BTreeMap::new();
    let mut label = Vec::with_capacity(nc);
    for c in 0..nc {
        let g = cell_volume_centroid(src, CellIx::new(c)).1;
        let bin = [
            ((g.x - origin[0]) / size[0]).floor() as i64,
            ((g.y - origin[1]) / size[1]).floor() as i64,
            ((g.z - origin[2]) / size[2]).floor() as i64,
        ];
        let n = ids.len() as u64;
        label.push(*ids.entry((region[c], bin)).or_insert(n));
    }
    components(src, &label)
}

/// Greedy pairwise agglomeration. Cells merge only within one `key` class; the total stops at
/// `global_target`, and a class never drops below `key_min[key]`. Each round visits groups by
/// ascending `(volume, id)` and merges each with the neighbour sharing the most face area whose
/// union passes [`union_convex`] (ties: lowest id); a group merges at most once per round.
pub(crate) fn greedy(src: &dyn MeshRead, key: &[u32], key_min: &[usize], global_target: usize) -> Vec<u32> {
    let nc = src.count(MeshEntityKind::Cell);
    let mut group_of: Vec<u32> = (0..nc as u32).collect();
    let mut mem: Vec<Vec<CellIx>> = (0..nc).map(|c| vec![CellIx::new(c)]).collect();
    let mut key_count = vec![0usize; key_min.len()];
    for &k in key {
        key_count[k as usize] += 1;
    }
    let mut remaining = nc;
    let vol: Vec<f64> = (0..nc).map(|c| cell_volume(src, CellIx::new(c))).collect();
    let mut gvol = vol;
    loop {
        if remaining <= global_target {
            break;
        }
        let mut order: Vec<usize> = (0..nc).filter(|&g| !mem[g].is_empty()).collect();
        order.sort_by(|&a, &b| gvol[a].total_cmp(&gvol[b]).then(a.cmp(&b)));
        let mut taken = vec![false; nc];
        let mut merges: Vec<(usize, usize)> = Vec::new();
        for &g in &order {
            if remaining <= global_target {
                break;
            }
            let k = key[mem[g][0].get()] as usize;
            if taken[g] || key_count[k] <= key_min[k] {
                continue;
            }
            let mut nb: SmallVec<[(usize, f64); 16]> = SmallVec::new();
            for &c in &mem[g] {
                for sf in src.cell_faces(c) {
                    let Some(o) = src.across(*sf) else { continue };
                    let og = group_of[o.get()] as usize;
                    if og == g || taken[og] || key[o.get()] as usize != k {
                        continue;
                    }
                    let a = face_area_vector(src, sf.face()).norm();
                    match nb.iter_mut().find(|e| e.0 == og) {
                        Some(e) => e.1 += a,
                        None => nb.push((og, a)),
                    }
                }
            }
            nb.sort_by(|x, y| y.1.total_cmp(&x.1).then(x.0.cmp(&y.0)));
            for &(og, _) in &nb {
                let mut union: Vec<CellIx> = mem[g].iter().chain(mem[og].iter()).copied().collect();
                union.sort_unstable();
                let (gg, oo) = (g as u32, og as u32);
                let inside = |c: CellIx| {
                    let x = group_of[c.get()];
                    x == gg || x == oo
                };
                if union_convex(src, &union, &inside) {
                    taken[g] = true;
                    taken[og] = true;
                    merges.push((g.min(og), g.max(og)));
                    key_count[k] -= 1;
                    remaining -= 1;
                    break;
                }
            }
        }
        if merges.is_empty() {
            break;
        }
        for (s, t) in merges {
            let moved = std::mem::take(&mut mem[t]);
            for c in &moved {
                group_of[c.get()] = s as u32;
            }
            mem[s].extend(moved);
            mem[s].sort_unstable();
            gvol[s] += gvol[t];
        }
    }
    let lab: Vec<u64> = group_of.iter().map(|&g| u64::from(g)).collect();
    densify(&lab)
}

/// Dissolve every group of more than one cell that fails the convex rule into single cells.
/// Returns the new numbering and the number of dissolved groups.
pub(crate) fn dissolve_nonconvex(src: &dyn MeshRead, group_of: &[u32], n: usize) -> (Vec<u32>, usize) {
    let mem = members(group_of, n);
    let mut lab: Vec<u64> = group_of.iter().map(|&g| u64::from(g)).collect();
    let mut dissolved = 0;
    for (g, m) in mem.iter().enumerate() {
        if m.len() > 1 {
            let inside = |c: CellIx| group_of[c.get()] == g as u32;
            if !union_convex(src, m, &inside) {
                dissolved += 1;
                for c in m {
                    lab[c.get()] = (n + c.get()) as u64;
                }
            }
        }
    }
    (densify(&lab), dissolved)
}

/// One coarse face under construction.
struct Comp {
    /// Fine faces it carries.
    faces: Vec<usize>,
    lo: u32,
    hi: u32,
    cls: Classification,
    /// Loop over fine vertex numbers, outward from the group `lo`.
    lp: Vec<u32>,
}

fn find(p: &mut [usize], mut x: usize) -> usize {
    while p[x] != x {
        p[x] = p[p[x]];
        x = p[x];
    }
    x
}

/// The single boundary cycle of a set of consistently oriented loops (shared edges cancel), or
/// `None` if the boundary is not one simple cycle.
fn merged_loop(loops: &[&[u32]]) -> Option<Vec<u32>> {
    let mut count: BTreeMap<(u32, u32), i32> = BTreeMap::new();
    for l in loops {
        for i in 0..l.len() {
            let (a, b) = (l[i], l[(i + 1) % l.len()]);
            if a == b {
                return None;
            }
            if let Some(c) = count.get_mut(&(b, a)) {
                *c -= 1;
            } else {
                *count.entry((a, b)).or_insert(0) += 1;
            }
        }
    }
    let mut next: BTreeMap<u32, u32> = BTreeMap::new();
    for (&(a, b), &c) in &count {
        if c == 0 {
            continue;
        }
        if c != 1 || next.insert(a, b).is_some() {
            return None;
        }
    }
    let (&start, _) = next.iter().next()?;
    let mut out = vec![start];
    let mut at = *next.get(&start)?;
    while at != start {
        out.push(at);
        at = *next.get(&at)?;
        if out.len() > next.len() {
            return None;
        }
    }
    (out.len() == next.len() && out.len() >= 3).then_some(out)
}

/// Build the coarse mesh of the partition `group_of` (dense, `0..n_groups`). Returns the finished
/// mesh and the face map (fine face -> final coarse face, [`NO_CELL`] for faces inside a group).
/// With `merge_classes` faces of different model entities between the same two groups may merge
/// (the merged face takes the class with the most area), which is what a CMFD pin cell wants.
pub(crate) fn build_coarse<B: MeshBuild>(
    src: &dyn MeshRead,
    group_of: &[u32],
    n_groups: usize,
    merge_classes: bool,
    mut b: B,
    alloc: &mut SerialAlloc,
) -> Result<(Finished<B::Output>, Vec<u32>), PolyError> {
    let bad = |m: String| PolyError::Coarsen(m);
    let nf = src.count(MeshEntityKind::Face);
    // External fine faces, oriented outward from the lower group.
    struct Ext {
        f: usize,
        lo: u32,
        hi: u32,
        cls: Classification,
        lp: Vec<u32>,
    }
    let mut ext: Vec<Ext> = Vec::new();
    for f in 0..nf {
        let fi = FaceIx::new(f);
        if !matches!(src.face_kind(fi), FaceKind::Interior | FaceKind::Boundary) {
            return Err(PolyError::UnsupportedFaceKind { face: fi });
        }
        let gl = group_of[src.face_left(fi).get()];
        let gr = src.face_right(fi).map(|r| group_of[r.get()]);
        let (lo, hi, flip) = match gr {
            Some(gr) if gr == gl => continue,
            Some(gr) if gr < gl => (gr, gl, true),
            Some(gr) => (gl, gr, false),
            None => (gl, u32::MAX, false),
        };
        let mut lp: Vec<u32> = src.face_vertices(fi).iter().map(|v| v.0).collect();
        if flip {
            lp.reverse();
        }
        ext.push(Ext { f, lo, hi, cls: src.face_class(fi), lp });
    }
    // Union edge-adjacent coplanar faces of the same pair and class.
    let key_of = |e: &Ext| if merge_classes { (e.lo, e.hi, None, [0; 4]) } else { (e.lo, e.hi, Some(e.cls.entity), e.cls.param_bits()) };
    let planes: Vec<(V3, V3, f64)> = ext
        .iter()
        .map(|e| {
            let l: SmallVec<[V3; 16]> = e.lp.iter().map(|&v| pt(src, VertexIx(v))).collect();
            let (n, a, _) = plane(&l);
            (n, a, bbox_diam(&l))
        })
        .collect();
    let mut edge_faces: BTreeMap<(u32, u32), SmallVec<[usize; 2]>> = BTreeMap::new();
    for (i, e) in ext.iter().enumerate() {
        for k in 0..e.lp.len() {
            let (a, c) = (e.lp[k], e.lp[(k + 1) % e.lp.len()]);
            edge_faces.entry((a.min(c), a.max(c))).or_default().push(i);
        }
    }
    let mut parent: Vec<usize> = (0..ext.len()).collect();
    for fs in edge_faces.values() {
        for x in 0..fs.len() {
            for y in x + 1..fs.len() {
                let (i, j) = (fs[x], fs[y]);
                if key_of(&ext[i]) != key_of(&ext[j]) {
                    continue;
                }
                let (ni, ai, di) = planes[i];
                let (nj, _, dj) = planes[j];
                let tol = TOL * di.max(dj);
                let co = geom::dot(ni, nj) > 1.0 - 1e-9
                    && ext[j].lp.iter().all(|&v| geom::dot(ni, geom::sub(pt(src, VertexIx(v)), ai)).abs() <= tol);
                if co {
                    let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                    parent[ri.max(rj)] = ri.min(rj);
                }
            }
        }
    }
    let mut by_root: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for i in 0..ext.len() {
        let r = find(&mut parent, i);
        by_root.entry(r).or_default().push(i);
    }
    let nv = src.count(MeshEntityKind::Vertex);
    let collinear = |a: u32, v: u32, c: u32| {
        let (pa, pv, pc) = (pt(src, VertexIx(a)), pt(src, VertexIx(v)), pt(src, VertexIx(c)));
        let (x, y) = (geom::sub(pa, pv), geom::sub(pc, pv));
        let (nx, ny) = (geom::norm(x), geom::norm(y));
        geom::norm(geom::cross(x, y)) <= 1e-12 * nx * ny && geom::dot(x, y) < 0.0
    };
    // A merged face must stay star-shaped about its vertex mean once the collinear vertices are
    // dropped (the fan rule of the geometry validator); a merge that is not is undone, which can
    // change the dropped set, so repeat until stable. `forced` only grows.
    let mut forced: std::collections::BTreeSet<usize> = std::collections::BTreeSet::new();
    let (comps, needed, used) = loop {
        let mut comps: Vec<(usize, Comp)> = Vec::new();
        for (&root, idx) in &by_root {
            if idx.len() > 1 && !forced.contains(&root) {
                let loops: Vec<&[u32]> = idx.iter().map(|&i| ext[i].lp.as_slice()).collect();
                let merged = merged_loop(&loops).filter(|m| {
                    // The merged polygon must carry exactly the area of its parts (no fold).
                    let l: SmallVec<[V3; 32]> = m.iter().map(|&v| pt(src, VertexIx(v))).collect();
                    let (n, _, _) = plane(&l);
                    let sum: f64 = idx.iter().map(|&i| face_area_vector(src, FaceIx::new(ext[i].f)).norm()).sum();
                    (fan_area(&l, n) - sum).abs() <= 1e-9 * sum
                });
                if let Some(lp) = merged {
                    // Class of the merged face: the one carrying most area (ties: lowest entity).
                    let mut share: BTreeMap<(EntityId, [u64; 4]), (f64, Classification)> = BTreeMap::new();
                    for &i in idx {
                        let a = face_area_vector(src, FaceIx::new(ext[i].f)).norm();
                        share.entry((ext[i].cls.entity, ext[i].cls.param_bits())).or_insert((0.0, ext[i].cls)).0 += a;
                    }
                    let mut best: Option<(f64, Classification)> = None;
                    for (_, (a, c)) in share {
                        if best.is_none_or(|(ba, _)| a > ba) {
                            best = Some((a, c));
                        }
                    }
                    let e = &ext[idx[0]];
                    let cls = best.map_or(e.cls, |b| b.1);
                    comps.push((root, Comp { faces: idx.iter().map(|&i| ext[i].f).collect(), lo: e.lo, hi: e.hi, cls, lp }));
                    continue;
                }
            }
            for &i in idx {
                let e = &ext[i];
                comps.push((root, Comp { faces: vec![e.f], lo: e.lo, hi: e.hi, cls: e.cls, lp: e.lp.clone() }));
            }
        }
        comps.sort_by_key(|(_, c)| c.faces[0]);
        // Which vertices stay: corners, model vertices, and any vertex that is not collinear in some face.
        let mut needed = vec![false; nv];
        let mut used = vec![false; nv];
        for (_, c) in &comps {
            let n = c.lp.len();
            for i in 0..n {
                let v = c.lp[i];
                used[v as usize] = true;
                if !collinear(c.lp[(i + n - 1) % n], v, c.lp[(i + 1) % n]) {
                    needed[v as usize] = true;
                }
            }
        }
        for v in 0..nv {
            if used[v] && src.vertex_class(VertexIx::new(v)).dim() == Some(0) {
                needed[v] = true;
            }
        }
        let reduce = |c: &Comp, needed: &[bool]| -> Vec<u32> { c.lp.iter().copied().filter(|&v| needed[v as usize]).collect() };
        for (_, c) in &comps {
            if reduce(c, &needed).len() < 3 {
                for &v in &c.lp {
                    needed[v as usize] = true;
                }
            }
        }
        let mut undone = false;
        for (root, c) in &comps {
            let red = reduce(c, &needed);
            let l: SmallVec<[V3; 32]> = red.iter().map(|&v| pt(src, VertexIx(v))).collect();
            if !fan_ok(&l) {
                if c.faces.len() > 1 {
                    undone |= forced.insert(*root);
                } else {
                    // An unmerged face keeps every vertex it had.
                    for &v in &c.lp {
                        needed[v as usize] = true;
                    }
                }
            }
        }
        if !undone {
            break (comps.into_iter().map(|(_, c)| c).collect::<Vec<Comp>>(), needed, used);
        }
    };
    let reduce = |c: &Comp, needed: &[bool]| -> Vec<u32> { c.lp.iter().copied().filter(|&v| needed[v as usize]).collect() };
    let mut vnew = vec![u32::MAX; nv];
    let mut n_pts = 0u32;
    for v in 0..nv {
        if used[v] && needed[v] {
            vnew[v] = n_pts;
            n_pts += 1;
        }
    }
    b.reserve(n_pts as usize, n_groups);
    for v in 0..nv {
        if vnew[v] != u32::MAX {
            let p = src.point(v);
            let h = b.add_vertex(p).map_err(|e| bad(e.to_string()))?;
            b.set_class(MeshEntityRef::Vertex(h), src.vertex_class(VertexIx::new(v))).map_err(|e| bad(e.to_string()))?;
        }
    }
    // Faces, with edge classes when the source has an edge table.
    let table = src.edges();
    let mut edge_cls: BTreeMap<(u32, u32), Classification> = BTreeMap::new();
    let mut handles: Vec<FaceIx> = Vec::with_capacity(comps.len());
    let mut cell_faces: Vec<Vec<SignedFace>> = vec![Vec::new(); n_groups];
    for c in &comps {
        let red = reduce(c, &needed);
        let lp: Vec<VertexIx> = red.iter().map(|&v| VertexIx(vnew[v as usize])).collect();
        let h = b.add_face(&lp).map_err(|e| bad(e.to_string()))?;
        b.set_class(MeshEntityRef::Face(h), c.cls).map_err(|e| bad(e.to_string()))?;
        handles.push(h);
        cell_faces[c.lo as usize].push(SignedFace::new(h, Side::Left));
        if c.hi != u32::MAX {
            cell_faces[c.hi as usize].push(SignedFace::new(h, Side::Right));
        }
        if let Some(t) = table {
            // Class of a coarse segment: that of the first fine edge it spans.
            let n = c.lp.len();
            let kept: Vec<usize> = (0..n).filter(|&i| needed[c.lp[i] as usize]).collect();
            for (k, &i) in kept.iter().enumerate() {
                let j = kept[(k + 1) % kept.len()];
                let (a, e2) = (c.lp[i], c.lp[(i + 1) % n]);
                if let Some(ei) = t.find(VertexIx(a), VertexIx(e2)) {
                    let (x, y) = (vnew[c.lp[i] as usize], vnew[c.lp[j] as usize]);
                    edge_cls.entry((x.min(y), x.max(y))).or_insert(t.class[ei.get()]);
                }
            }
        }
    }
    // Cells: class of the volume with the largest share of the group.
    let mut share: Vec<BTreeMap<EntityId, f64>> = vec![BTreeMap::new(); n_groups];
    for c in 0..group_of.len() {
        let ci = CellIx::new(c);
        *share[group_of[c] as usize].entry(src.cell_class(ci).entity).or_insert(0.0) += cell_volume(src, ci);
    }
    for (g, sfs) in cell_faces.iter().enumerate() {
        let cell = b.add_cell(sfs, None).map_err(|e| bad(format!("coarse cell {g}: {e}")))?;
        let mut best: Option<(EntityId, f64)> = None;
        for (&e, &v) in &share[g] {
            if best.is_none_or(|(_, bv)| v > bv) {
                best = Some((e, v));
            }
        }
        let (entity, _) = best.ok_or_else(|| bad(format!("coarse cell {g} has no fine cell")))?;
        b.set_class(MeshEntityRef::Cell(cell), Classification::on(entity)).map_err(|e| bad(e.to_string()))?;
    }
    for (&(x, y), &c) in &edge_cls {
        b.set_class(MeshEntityRef::Edge(VertexIx(x), VertexIx(y)), c).map_err(|e| bad(e.to_string()))?;
    }
    let fin = b.finish(alloc).map_err(|e| bad(e.to_string()))?;
    let mut face_map = vec![NO_CELL; nf];
    for (c, h) in comps.iter().zip(&handles) {
        let fin_ix = fin.face_remap[h.get()].0;
        for &f in &c.faces {
            face_map[f] = fin_ix;
        }
    }
    Ok((fin, face_map))
}

/// Every fan triangle `(p_i, p_{i+1}, apex)` of the loop has the orientation of the loop normal
/// (the loop is star-shaped about its vertex mean), as `validate_geometry` requires.
fn fan_ok(l: &[V3]) -> bool {
    if l.len() < 3 {
        return false;
    }
    let (n, apex, ok) = plane(l);
    ok && (0..l.len()).all(|i| geom::dot(geom::cross(geom::sub(l[i], apex), geom::sub(l[(i + 1) % l.len()], apex)), n) > 0.0)
}

/// Area of a planar loop along the unit normal `n`.
fn fan_area(l: &[V3], n: V3) -> f64 {
    let apex = geom::mean(l);
    let mut a = [0.0; 3];
    for i in 0..l.len() {
        let c = geom::cross(geom::sub(l[i], apex), geom::sub(l[(i + 1) % l.len()], apex));
        a = [a[0] + 0.5 * c[0], a[1] + 0.5 * c[1], a[2] + 0.5 * c[2]];
    }
    geom::dot(a, n)
}
