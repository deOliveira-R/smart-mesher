//! Greedy agglomeration of small cells on the [`PolyData`] (stretch S2).
//!
//! Serial and deterministic: candidates are visited by ascending `(volume, index)`, neighbours by
//! descending shared area (ties lowest index), and each cell merges at most once per round. A
//! merge removes the shared faces and keeps the other faces as they are, so planarity, interface
//! faces and classes are untouched.

use crate::geom::{self, V3};
use crate::options::Agglomerate;
use crate::polydata::PolyData;
use crate::prep::NONE;
use sm_mesh_api::{Side, SignedFace};
use smallvec::SmallVec;

/// Merge cells; returns the number of merges.
pub(crate) fn agglomerate(d: &mut PolyData, a: &Agglomerate) -> usize {
    let mut total = 0;
    for _ in 0..a.max_rounds {
        let merged = round(d, a);
        if merged == 0 {
            break;
        }
        total += merged;
    }
    total
}

fn pt(d: &PolyData, p: u32) -> V3 {
    d.pts[p as usize]
}

/// Vertex mean of a face loop.
pub(crate) fn apex(d: &PolyData, l: &[u32]) -> V3 {
    let ps: SmallVec<[V3; 16]> = l.iter().map(|&p| pt(d, p)).collect();
    geom::mean(&ps)
}

/// Area vector (along the loop normal) of a face under the fan-vertex-mean rule.
fn area_vector(d: &PolyData, l: &[u32]) -> V3 {
    let ap = apex(d, l);
    let mut a = [0.0; 3];
    for i in 0..l.len() {
        let c = geom::cross(geom::sub(pt(d, l[i]), ap), geom::sub(pt(d, l[(i + 1) % l.len()]), ap));
        a = [a[0] + c[0] * 0.5, a[1] + c[1] * 0.5, a[2] + c[2] * 0.5];
    }
    a
}

/// Volume and centroid of the cell bounded by `faces` (the fan polyhedron of the geometry rule).
pub(crate) fn vol_centroid(d: &PolyData, faces: &[SignedFace]) -> (f64, V3) {
    let Some(first) = faces.first() else {
        return (0.0, [0.0; 3]);
    };
    let o = pt(d, d.face_loop(first.face().get())[0]);
    let (mut vol, mut mom) = (0.0, [0.0; 3]);
    for sf in faces {
        let l = d.face_loop(sf.face().get());
        let n = l.len();
        let ap = apex(d, l);
        for i in 0..n {
            let (mut a, mut b) = (pt(d, l[i]), pt(d, l[(i + 1) % n]));
            if sf.side() == Side::Right {
                std::mem::swap(&mut a, &mut b);
            }
            let v = geom::dot(geom::sub(ap, o), geom::cross(geom::sub(a, o), geom::sub(b, o))) / 6.0;
            vol += v;
            for k in 0..3 {
                mom[k] += (o[k] + ap[k] + a[k] + b[k]) * (v / 4.0);
            }
        }
    }
    if vol == 0.0 {
        return (0.0, o);
    }
    (vol, [mom[0] / vol, mom[1] / vol, mom[2] / vol])
}

/// The other cell of face `f` seen from `c`.
fn other(d: &PolyData, f: usize, c: u32) -> u32 {
    let [l, r] = d.f_cells[f];
    if l == c { r } else { l }
}

fn round(d: &mut PolyData, a: &Agglomerate) -> usize {
    let nc = d.n_cells();
    let vols: Vec<f64> = (0..nc)
        .map(|c| if d.c_alive[c] { vol_centroid(d, d.cell_faces(c)).0 } else { f64::NAN })
        .collect();
    let mut sorted: Vec<f64> = vols.iter().copied().filter(|v| !v.is_nan()).collect();
    if sorted.is_empty() {
        return 0;
    }
    sorted.sort_by(f64::total_cmp);
    let n = sorted.len();
    let median = if n % 2 == 1 { sorted[n / 2] } else { 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]) };
    let thr = a.min_volume_ratio * median;
    let mut cands: Vec<usize> = (0..nc).filter(|&c| d.c_alive[c] && vols[c] < thr).collect();
    cands.sort_by(|&x, &y| vols[x].total_cmp(&vols[y]).then(x.cmp(&y)));

    let mut taken = vec![false; nc];
    let mut merges: Vec<(usize, usize)> = Vec::new();
    for &c in &cands {
        if taken[c] {
            continue;
        }
        // Neighbours of the same class and their shared area.
        let mut nb: SmallVec<[(u32, f64); 16]> = SmallVec::new();
        for sf in d.cell_faces(c) {
            let f = sf.face().get();
            let o = other(d, f, c as u32);
            if o == NONE || taken[o as usize] || !d.c_alive[o as usize] || d.c_region[o as usize] != d.c_region[c] {
                continue;
            }
            let area = geom::norm(area_vector(d, d.face_loop(f)));
            match nb.iter_mut().find(|e| e.0 == o) {
                Some(e) => e.1 += area,
                None => nb.push((o, area)),
            }
        }
        nb.sort_by(|x, y| y.1.total_cmp(&x.1).then(x.0.cmp(&y.0)));
        for &(o, _) in &nb {
            if merge_ok(d, c, o as usize, a.max_concavity) {
                taken[c] = true;
                taken[o as usize] = true;
                merges.push((c.min(o as usize), c.max(o as usize)));
                break;
            }
        }
    }
    if merges.is_empty() {
        return 0;
    }
    apply(d, &merges);
    merges.len()
}

/// Faces of `a` and `b` without the faces they share, and the shared faces.
fn split_faces(d: &PolyData, a: usize, b: usize) -> (SmallVec<[SignedFace; 32]>, SmallVec<[usize; 8]>) {
    let mut merged = SmallVec::new();
    let mut shared = SmallVec::new();
    for sf in d.cell_faces(a) {
        let f = sf.face().get();
        if other(d, f, a as u32) == b as u32 {
            shared.push(f);
        } else {
            merged.push(*sf);
        }
    }
    for sf in d.cell_faces(b) {
        if !shared.contains(&sf.face().get()) {
            merged.push(*sf);
        }
    }
    (merged, shared)
}

/// The shared faces form one disc patch (their boundary is one simple cycle) and the merged cell
/// is valid and near-convex.
fn merge_ok(d: &PolyData, a: usize, b: usize, max_concavity: f64) -> bool {
    let (faces, shared) = split_faces(d, a, b);
    if shared.is_empty() || !one_boundary_cycle(d, &shared) {
        return false;
    }
    let (vol, g) = vol_centroid(d, &faces);
    if vol <= 0.0 {
        return false;
    }
    let mut verts: SmallVec<[u32; 64]> = faces.iter().flat_map(|sf| d.face_loop(sf.face().get()).iter().copied()).collect();
    verts.sort_unstable();
    verts.dedup();
    let scale = vol.cbrt();
    for sf in &faces {
        let l = d.face_loop(sf.face().get());
        let ap = apex(d, l);
        let av = area_vector(d, l);
        let len = geom::norm(av);
        if len == 0.0 {
            return false;
        }
        let nrm = [av[0] / len * sf.side().sign(), av[1] / len * sf.side().sign(), av[2] / len * sf.side().sign()];
        let worst = verts.iter().map(|&v| geom::dot(geom::sub(pt(d, v), ap), nrm)).fold(f64::NEG_INFINITY, f64::max);
        if worst / scale > max_concavity {
            return false;
        }
        // side tetrahedra of the validity rule
        let n = l.len();
        for i in 0..n {
            let (mut p, mut q) = (pt(d, l[i]), pt(d, l[(i + 1) % n]));
            if sf.side() == Side::Right {
                std::mem::swap(&mut p, &mut q);
            }
            if geom::dot(geom::sub(ap, g), geom::cross(geom::sub(p, g), geom::sub(q, g))) <= 0.0 {
                return false;
            }
        }
    }
    true
}

/// True if the boundary edges of the face set (edges used by exactly one of its faces) form one
/// simple closed cycle.
fn one_boundary_cycle(d: &PolyData, faces: &[usize]) -> bool {
    let mut edges: SmallVec<[(u32, u32); 32]> = SmallVec::new();
    for &f in faces {
        let l = d.face_loop(f);
        for i in 0..l.len() {
            let (x, y) = (l[i], l[(i + 1) % l.len()]);
            edges.push((x.min(y), x.max(y)));
        }
    }
    edges.sort_unstable();
    let mut boundary: SmallVec<[(u32, u32); 32]> = SmallVec::new();
    let mut i = 0;
    while i < edges.len() {
        let mut j = i;
        while j < edges.len() && edges[j] == edges[i] {
            j += 1;
        }
        if j - i == 1 {
            boundary.push(edges[i]);
        } else if j - i != 2 {
            return false;
        }
        i = j;
    }
    if boundary.is_empty() {
        return false;
    }
    // every boundary vertex has degree 2, and walking from one edge visits them all
    let mut deg: SmallVec<[(u32, u32); 32]> = SmallVec::new();
    for &(x, y) in &boundary {
        for v in [x, y] {
            match deg.iter_mut().find(|e| e.0 == v) {
                Some(e) => e.1 += 1,
                None => deg.push((v, 1)),
            }
        }
    }
    if deg.iter().any(|e| e.1 != 2) {
        return false;
    }
    let mut visited = 1usize;
    let mut used: SmallVec<[bool; 32]> = SmallVec::from_elem(false, boundary.len());
    used[0] = true;
    let mut at = boundary[0].1;
    let start = boundary[0].0;
    while at != start {
        let Some(k) = (0..boundary.len()).find(|&k| !used[k] && (boundary[k].0 == at || boundary[k].1 == at)) else {
            return false;
        };
        used[k] = true;
        visited += 1;
        at = if boundary[k].0 == at { boundary[k].1 } else { boundary[k].0 };
    }
    visited == boundary.len()
}

/// Apply the merges `(survivor, absorbed)` (disjoint pairs) and rebuild the cell table.
fn apply(d: &mut PolyData, merges: &[(usize, usize)]) {
    let nc = d.n_cells();
    let mut new_list: Vec<Option<Vec<SignedFace>>> = vec![None; nc];
    for &(s, t) in merges {
        let (faces, shared) = split_faces(d, s, t);
        for f in shared {
            d.f_alive[f] = false;
        }
        for sf in d.cell_faces(t).to_vec() {
            let f = sf.face().get();
            if d.f_alive[f] {
                for c in &mut d.f_cells[f] {
                    if *c == t as u32 {
                        *c = s as u32;
                    }
                }
            }
        }
        new_list[s] = Some(faces.to_vec());
        d.c_alive[t] = false;
    }
    let (mut off, mut cf) = (vec![0u32], Vec::new());
    for (c, list) in new_list.iter().enumerate() {
        match list {
            Some(l) => cf.extend_from_slice(l),
            None if d.c_alive[c] => cf.extend_from_slice(d.cell_faces(c)),
            None => {}
        }
        off.push(cf.len() as u32);
    }
    d.c_off = off;
    d.c_faces = cf;
}
