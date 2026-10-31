//! checkMesh's cell determinant and the export-time split of cells that fail it (ruling INT-8).
//!
//! The internal mesh stays exact. A cell with few internal faces (a tet with three boundary
//! faces) has rank-deficient internal face normals, so OpenFOAM's `checkMesh` reports
//! `Cells with small determinant`. The exporter maps such cells to a form checkMesh accepts: it
//! splits each into one sub-cell per face (a face pyramid around the cell's vertex centroid; for
//! a tet, the centroid split into 4 tets). Sub-cells keep the parent's zones and boundary
//! patches, and `smeshCellMap` records which internal cell each exported cell came from.

use super::{FoamFile, PolyFaceZone, PolyMesh, PolyPatch, PolyZone};
use rayon::prelude::*;

const NONE: u32 = u32::MAX;

/// checkMesh's threshold: cells with a determinant below this are split at export.
pub const SMALL_DETERMINANT: f64 = 1e-3;

/// File name (inside `constant/polyMesh`) of the exported-cell -> internal-cell map. Written
/// only when at least one cell was split. Format: an OpenFOAM `labelList` (`class labelList`,
/// `object smeshCellMap`): `N`, `(`, one internal cell index per line (line `i` belongs to
/// exported cell `i`), `)`. [`parse_polymesh`](super::parse_polymesh) restores it into
/// [`PolyMesh::cell_map`].
pub const CELL_MAP_FILE: &str = "smeshCellMap";

/// Newell area vector of a face loop (half the sum of `p_i x p_{i+1}`).
pub(crate) fn area_vector(points: &[[f64; 3]], loop_: &[u32]) -> [f64; 3] {
    let mut s = [0.0; 3];
    for k in 0..loop_.len() {
        let (a, b) = (points[loop_[k] as usize], points[loop_[(k + 1) % loop_.len()] as usize]);
        s[0] += (a[1] * b[2] - a[2] * b[1]) / 2.0;
        s[1] += (a[2] * b[0] - a[0] * b[2]) / 2.0;
        s[2] += (a[0] * b[1] - a[1] * b[0]) / 2.0;
    }
    s
}

/// OpenFOAM `primitiveMeshTools::cellDeterminant` of every cell. With `Sf` the face area
/// vectors of the cell's internal faces and `avg` their mean magnitude, the area tensor is
/// `sum (Sf/avg)(Sf/avg)^T` and the result is `|det| / 8` (an interior cube cell is 1, a cube
/// corner cell with 3 internal faces is 1/8). A cell with no internal face, or `avg` below
/// 1e-150, is 0: checkMesh flags it.
pub fn cell_determinants(pm: &PolyMesh) -> Vec<f64> {
    let nc = pm.n_cells as usize;
    let ni = pm.n_internal();
    let sf: Vec<[f64; 3]> = (0..ni).into_par_iter().map(|i| area_vector(&pm.points, pm.face(i))).collect();
    // Serial in face order: the sums are bit-identical for any thread count.
    let mut a = vec![[0.0f64; 6]; nc];
    let mut mag = vec![0.0f64; nc];
    let mut cnt = vec![0u32; nc];
    for (i, n) in sf.iter().enumerate() {
        let t = [n[0] * n[0], n[0] * n[1], n[0] * n[2], n[1] * n[1], n[1] * n[2], n[2] * n[2]];
        let m = (t[0] + t[3] + t[5]).sqrt();
        for c in [pm.owner[i] as usize, pm.neighbour[i] as usize] {
            for k in 0..6 {
                a[c][k] += t[k];
            }
            mag[c] += m;
            cnt[c] += 1;
        }
    }
    (0..nc)
        .map(|c| {
            let avg = if cnt[c] > 0 { mag[c] / f64::from(cnt[c]) } else { 0.0 };
            if cnt[c] == 0 || avg < 1e-150 {
                return 0.0;
            }
            let k = 1.0 / (avg * avg);
            let [xx, xy, xz, yy, yz, zz] = a[c].map(|v| v * k);
            let det = xx * (yy * zz - yz * yz) - xy * (xy * zz - yz * xz) + xz * (xy * yz - yy * xz);
            det.abs() / 8.0
        })
        .collect()
}

/// Sum of the signed volumes of the fan tets (apex, m, a, b) over an outward base loop, `m` the
/// loop's vertex average: positive when `apex` lies inside the base's half-space.
fn pyramid_volume(points: &[[f64; 3]], loop_: &[u32], apex: [f64; 3]) -> f64 {
    let n = loop_.len();
    let mut m = [0.0; 3];
    for &v in loop_ {
        for (d, x) in m.iter_mut().enumerate() {
            *x += points[v as usize][d] / n as f64;
        }
    }
    let sub = |p: [f64; 3], q: [f64; 3]| [p[0] - q[0], p[1] - q[1], p[2] - q[2]];
    let mut vol = 0.0;
    for e in 0..n {
        let (a, b) = (sub(points[loop_[e] as usize], m), sub(points[loop_[(e + 1) % n] as usize], m));
        let c = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
        let h = sub(m, apex);
        vol += (c[0] * h[0] + c[1] * h[1] + c[2] * h[2]) / 6.0;
    }
    vol
}

/// What [`split_small_determinant`] did.
#[derive(Debug, Default)]
pub(crate) struct SplitInfo {
    /// Internal cells that were split (ascending).
    pub split_cells: Vec<u32>,
    /// Sub-cells made.
    pub sub_cells: u64,
    /// Sub-cells by shape: tet, pyramid, other.
    pub sub_shapes: [u64; 3],
    /// Cells below the threshold that could not be split (open or non-manifold cell, or a
    /// face pyramid around the centroid with non-positive volume).
    pub unsplit: u64,
    /// Exported cells still below the threshold after the split.
    pub remaining: u64,
    /// Foam face before the split -> foam face after it.
    pub new_of_old_face: Vec<u32>,
    /// Number of faces added (all internal).
    pub added_faces: usize,
}

struct Ent {
    own: u32,
    nei: u32,
    old: u32,
    tri: [u32; 3],
}

/// Split every cell whose [`cell_determinants`] entry is below `threshold` into one sub-cell
/// per face around the cell's vertex centroid. Returns `None` when no cell qualifies; the mesh
/// is then untouched, which is also the case when `split_cells` is empty (nothing splittable).
///
/// Parent `p` becomes consecutive cells in parent order, so the owner < neighbour rule and the
/// upper-triangular face order survive. New faces are the triangles (centroid, edge) between
/// two sub-cells; new points (centroids) are appended in parent order.
pub(crate) fn split_small_determinant(pm: &mut PolyMesh, threshold: f64) -> Option<SplitInfo> {
    let nc = pm.n_cells as usize;
    let (nf, ni) = (pm.n_faces(), pm.n_internal());
    let det = cell_determinants(pm);
    let mut cand = vec![NONE; nc];
    let mut n_cand = 0usize;
    for c in 0..nc {
        if det[c] < threshold {
            cand[c] = n_cand as u32;
            n_cand += 1;
        }
    }
    if n_cand == 0 {
        return None;
    }
    // Faces of each candidate in ascending foam face order: (face, cell is owner).
    let mut cfaces: Vec<Vec<(u32, bool)>> = vec![Vec::new(); n_cand];
    for f in 0..nf {
        let o = pm.owner[f] as usize;
        if cand[o] != NONE {
            cfaces[cand[o] as usize].push((f as u32, true));
        }
        if f < ni {
            let n = pm.neighbour[f] as usize;
            if cand[n] != NONE {
                cfaces[cand[n] as usize].push((f as u32, false));
            }
        }
    }
    let outward = |f: u32, own: bool| -> Vec<u32> {
        let l = pm.face(f as usize);
        if own { l.to_vec() } else { l.iter().rev().copied().collect() }
    };
    // A cell is splittable when every directed edge of its outward loops is matched by the
    // reverse edge in exactly one other face. across[k][i][e] = local face across edge e of face i.
    let mut splittable = vec![false; n_cand];
    let mut across: Vec<Vec<Vec<u32>>> = vec![Vec::new(); n_cand];
    let mut centre = vec![[0.0f64; 3]; n_cand];
    for (k, fs) in cfaces.iter().enumerate() {
        if fs.len() < 4 {
            continue;
        }
        let loops: Vec<Vec<u32>> = fs.iter().map(|&(f, o)| outward(f, o)).collect();
        let mut edges: Vec<(u32, u32, u32)> = Vec::new();
        for (i, l) in loops.iter().enumerate() {
            for e in 0..l.len() {
                edges.push((l[e], l[(e + 1) % l.len()], i as u32));
            }
        }
        edges.sort_unstable();
        if !edges.windows(2).all(|w| (w[0].0, w[0].1) != (w[1].0, w[1].1)) {
            continue;
        }
        let mut ac: Vec<Vec<u32>> = Vec::with_capacity(loops.len());
        let mut ok = true;
        'faces: for (i, l) in loops.iter().enumerate() {
            let mut row = Vec::with_capacity(l.len());
            for e in 0..l.len() {
                let (a, b) = (l[e], l[(e + 1) % l.len()]);
                match edges.binary_search_by(|x| (x.0, x.1).cmp(&(b, a))) {
                    Ok(p) if edges[p].2 as usize != i => row.push(edges[p].2),
                    _ => {
                        ok = false;
                        break 'faces;
                    }
                }
            }
            ac.push(row);
        }
        if !ok {
            continue;
        }
        let mut vs: Vec<u32> = loops.iter().flatten().copied().collect();
        vs.sort_unstable();
        vs.dedup();
        let mut sum = [0.0; 3];
        for &v in &vs {
            for (d, x) in sum.iter_mut().enumerate() {
                *x += pm.points[v as usize][d];
            }
        }
        let n = vs.len() as f64;
        let cen = [sum[0] / n, sum[1] / n, sum[2] / n];
        if loops.iter().all(|l| pyramid_volume(&pm.points, l, cen) > 0.0) {
            splittable[k] = true;
            across[k] = ac;
            centre[k] = cen;
        }
    }
    let mut info = SplitInfo { unsplit: splittable.iter().filter(|s| !**s).count() as u64, ..Default::default() };
    if !splittable.iter().any(|s| *s) {
        return Some(info);
    }
    // New cell numbering.
    let is_split = |c: usize| cand[c] != NONE && splittable[cand[c] as usize];
    let mut first = vec![0u32; nc + 1];
    for c in 0..nc {
        first[c + 1] = first[c] + if is_split(c) { cfaces[cand[c] as usize].len() as u32 } else { 1 };
    }
    let new_nc = first[nc] as usize;
    let mut sub_own = vec![NONE; nf];
    let mut sub_nei = vec![NONE; ni];
    let mut cell_map = vec![0u32; new_nc];
    let mut new_points: Vec<[f64; 3]> = Vec::new();
    let base_np = pm.points.len() as u32;
    let mut centroid_of = vec![NONE; n_cand];
    for c in 0..nc {
        for s in first[c]..first[c + 1] {
            cell_map[s as usize] = c as u32;
        }
        if !is_split(c) {
            continue;
        }
        let k = cand[c] as usize;
        let fs = &cfaces[k];
        centroid_of[k] = base_np + new_points.len() as u32;
        new_points.push(centre[k]);
        for (i, &(f, own)) in fs.iter().enumerate() {
            let sc = first[c] + i as u32;
            if own {
                sub_own[f as usize] = sc;
            } else {
                sub_nei[f as usize] = sc;
            }
            info.sub_shapes[match pm.face(f as usize).len() {
                3 => 0,
                4 => 1,
                _ => 2,
            }] += 1;
        }
        info.split_cells.push(c as u32);
        info.sub_cells += fs.len() as u64;
    }
    let own_of = |f: usize| if sub_own[f] != NONE { sub_own[f] } else { first[pm.owner[f] as usize] };
    // Internal faces: old ones remapped, plus the triangles between sub-cells of one parent.
    let mut ents: Vec<Ent> = Vec::with_capacity(ni + 6 * info.split_cells.len());
    for f in 0..ni {
        let nei = if sub_nei[f] != NONE { sub_nei[f] } else { first[pm.neighbour[f] as usize] };
        ents.push(Ent { own: own_of(f), nei, old: f as u32, tri: [0; 3] });
    }
    for &c in &info.split_cells {
        let k = cand[c as usize] as usize;
        let cpt = centroid_of[k];
        for (i, &(f, own)) in cfaces[k].iter().enumerate() {
            let l = outward(f, own);
            for e in 0..l.len() {
                let j = across[k][i][e] as usize;
                if j < i {
                    continue;
                }
                // The lower sub-cell i owns it. Its outward base loop runs a -> b, so the pyramid's
                // side face, outward from i, is (b, a, apex).
                let (a, b) = (l[e], l[(e + 1) % l.len()]);
                ents.push(Ent { own: first[c as usize] + i as u32, nei: first[c as usize] + j as u32, old: NONE, tri: [b, a, cpt] });
            }
        }
    }
    ents.sort_unstable_by_key(|e| (e.own, e.nei, e.old, e.tri));
    info.added_faces = ents.len() - ni;
    let mut new_of_old = vec![NONE; nf];
    let mut face_offsets: Vec<u64> = Vec::with_capacity(nf + info.added_faces + 1);
    face_offsets.push(0);
    let mut face_points: Vec<u32> = Vec::with_capacity(pm.face_points.len() + 3 * info.added_faces);
    let mut owner: Vec<u32> = Vec::with_capacity(nf + info.added_faces);
    let mut neighbour: Vec<u32> = Vec::with_capacity(ents.len());
    for (i, e) in ents.iter().enumerate() {
        if e.old == NONE {
            face_points.extend_from_slice(&e.tri);
        } else {
            face_points.extend_from_slice(pm.face(e.old as usize));
            new_of_old[e.old as usize] = i as u32;
        }
        face_offsets.push(face_points.len() as u64);
        owner.push(e.own);
        neighbour.push(e.nei);
    }
    for (f, slot) in new_of_old.iter_mut().enumerate().skip(ni) {
        *slot = owner.len() as u32;
        face_points.extend_from_slice(pm.face(f));
        face_offsets.push(face_points.len() as u64);
        owner.push(own_of(f));
    }
    let added = info.added_faces as u32;
    let patches: Vec<PolyPatch> = pm.patches.iter().map(|p| PolyPatch { start_face: p.start_face + added, ..p.clone() }).collect();
    let cell_zones: Vec<PolyZone> = pm
        .cell_zones
        .iter()
        .map(|z| PolyZone { name: z.name.clone(), labels: z.labels.iter().flat_map(|&c| first[c as usize]..first[c as usize + 1]).collect() })
        .collect();
    let face_zones: Vec<PolyFaceZone> = pm
        .face_zones
        .iter()
        .map(|z| {
            let mut pairs: Vec<(u32, bool)> = z.labels.iter().zip(&z.flips).map(|(&l, &fl)| (new_of_old[l as usize], fl)).collect();
            pairs.sort_unstable();
            PolyFaceZone { name: z.name.clone(), labels: pairs.iter().map(|p| p.0).collect(), flips: pairs.iter().map(|p| p.1).collect() }
        })
        .collect();
    pm.points.extend_from_slice(&new_points);
    pm.face_offsets = face_offsets;
    pm.face_points = face_points;
    pm.owner = owner;
    pm.neighbour = neighbour;
    pm.n_cells = new_nc as u32;
    pm.patches = patches;
    pm.cell_zones = cell_zones;
    pm.face_zones = face_zones;
    pm.cell_map = cell_map;
    info.remaining = cell_determinants(pm).iter().filter(|d| **d < threshold).count() as u64;
    info.new_of_old_face = new_of_old;
    Some(info)
}

/// The `smeshCellMap` file of `map` (exported cell -> internal cell).
pub(crate) fn cell_map_file(map: &[u32]) -> FoamFile {
    let mut s = String::from("FoamFile\n{\n    version     2.0;\n    format      ascii;\n    arch        \"LSB;label=32;scalar=64\";\n    class       labelList;\n");
    s.push_str(&format!("    note        \"exported cell -> internal cell\";\n    location    \"constant/polyMesh\";\n    object      {CELL_MAP_FILE};\n}}\n\n"));
    s.push_str(&format!("{}\n(\n", map.len()));
    for m in map {
        s.push_str(&m.to_string());
        s.push('\n');
    }
    s.push_str(")\n");
    FoamFile { name: CELL_MAP_FILE.to_string(), bytes: s.into_bytes() }
}
