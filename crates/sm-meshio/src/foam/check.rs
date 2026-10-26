//! checkMesh's topology and orientation checks, in process, over a [`PolyMesh`].

// Index-heavy numeric loops over three components and parallel arrays read best as ranges.
#![allow(clippy::needless_range_loop)]

use super::PolyMesh;
use rayon::prelude::*;

/// One finding of [`check_polymesh`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FoamIssue {
    /// Stable code, `foam.*`.
    pub code: &'static str,
    /// Face involved (foam numbering), if any.
    pub face: Option<u32>,
    /// Cell involved, if any.
    pub cell: Option<u32>,
    /// Human-readable detail.
    pub detail: String,
}

fn issue(code: &'static str, face: Option<u32>, cell: Option<u32>, detail: String) -> FoamIssue {
    FoamIssue { code, face, cell, detail }
}

type V3 = [f64; 3];

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}

/// Area vector (pointing along the loop's right-hand normal) and centroid of a face, with
/// OpenFOAM's estimate: triangles about the vertex mean weighted by their projected area.
fn face_geometry(pts: &[V3], f: &[u32]) -> (V3, V3) {
    let p = |i: usize| pts[f[i] as usize];
    if f.len() == 3 {
        let n = cross(sub(p(1), p(0)), sub(p(2), p(0)));
        let c = [(p(0)[0] + p(1)[0] + p(2)[0]) / 3.0, (p(0)[1] + p(1)[1] + p(2)[1]) / 3.0, (p(0)[2] + p(1)[2] + p(2)[2]) / 3.0];
        return ([n[0] / 2.0, n[1] / 2.0, n[2] / 2.0], c);
    }
    let k = f.len() as f64;
    let mut est = [0.0; 3];
    for i in 0..f.len() {
        for d in 0..3 {
            est[d] += p(i)[d] / k;
        }
    }
    let (mut sum_n, mut sum_a, mut sum_ac) = ([0.0; 3], 0.0, [0.0; 3]);
    for i in 0..f.len() {
        let (a, b) = (p(i), p((i + 1) % f.len()));
        let n = cross(sub(b, a), sub(est, a));
        let c = [(a[0] + b[0] + est[0]) / 3.0, (a[1] + b[1] + est[1]) / 3.0, (a[2] + b[2] + est[2]) / 3.0];
        for d in 0..3 {
            sum_n[d] += n[d];
        }
        let len = norm(n);
        // Weight by the area, signed against the running normal so that folded fans cancel.
        let w = len * if dot(n, sum_n) < 0.0 { -1.0 } else { 1.0 };
        sum_a += w;
        for d in 0..3 {
            sum_ac[d] += w * c[d];
        }
    }
    let area = [sum_n[0] / 2.0, sum_n[1] / 2.0, sum_n[2] / 2.0];
    let c = if sum_a.abs() > 1e-300 { [sum_ac[0] / sum_a, sum_ac[1] / sum_a, sum_ac[2] / sum_a] } else { est };
    (area, c)
}

/// Check the topology and orientation of a polyMesh, like the topology part of `checkMesh`.
/// `tol` is the relative tolerance of the closedness test (`1e-9` is a good value).
///
/// Codes: `foam.index_range` (indices or array sizes inconsistent; geometry checks are then
/// skipped), `foam.owner_not_below_neighbour`, `foam.not_upper_triangular` (internal faces not
/// ordered by owner then neighbour), `foam.upper_triangular_tie` (equal `(owner, neighbour)` of
/// two faces: a warning, checkMesh may need strictly increasing neighbours), `foam.patch_range`
/// (patches do not tile `n_internal..n_faces`), `foam.point_unused`, `foam.cell_too_few_faces`
/// (fewer than 4), `foam.cell_unclosed` (`|sum Sf| > tol * sum |Sf|`),
/// `foam.cell_nonpositive_volume`, `foam.face_orientation` (`Sf . (fc - cc) <= 0` for the owner).
///
/// This does not decide mesh validity: that stays `sm_mesh_api::validate`. Never panics on an
/// inconsistent mesh.
pub fn check_polymesh(m: &PolyMesh, tol: f64) -> Vec<FoamIssue> {
    let mut out = Vec::new();
    let (nf, ni, np, nc) = (m.owner.len(), m.neighbour.len(), m.points.len(), m.n_cells as usize);

    // ---- structure: if this fails nothing below can index safely -------------------------------
    let mut structural = Vec::new();
    if m.face_offsets.len() != nf + 1 || m.face_offsets.first() != Some(&0) || m.face_offsets.last().copied() != Some(m.face_points.len() as u64) || m.face_offsets.windows(2).any(|w| w[0] > w[1]) {
        structural.push(issue("foam.index_range", None, None, "face offsets are not a valid CSR over face_points".into()));
    }
    if ni > nf {
        structural.push(issue("foam.index_range", None, None, format!("{ni} neighbours for {nf} faces")));
    }
    if let Some(i) = m.face_points.iter().position(|v| *v as usize >= np) {
        structural.push(issue("foam.index_range", None, None, format!("face point entry {i} = {} is past the {np} points", m.face_points[i])));
    }
    if let Some(f) = m.owner.iter().position(|c| *c as usize >= nc) {
        structural.push(issue("foam.index_range", Some(f as u32), Some(m.owner[f]), format!("owner {} is past the {nc} cells", m.owner[f])));
    }
    if let Some(f) = m.neighbour.iter().position(|c| *c as usize >= nc) {
        structural.push(issue("foam.index_range", Some(f as u32), Some(m.neighbour[f]), format!("neighbour {} is past the {nc} cells", m.neighbour[f])));
    }
    if structural.is_empty() && (0..nf).any(|f| m.face_offsets[f + 1] - m.face_offsets[f] < 3) {
        structural.push(issue("foam.index_range", None, None, "a face has fewer than 3 points".into()));
    }
    if !structural.is_empty() {
        return structural;
    }

    // ---- ordering ------------------------------------------------------------------------------
    for i in 0..ni {
        if m.owner[i] >= m.neighbour[i] {
            out.push(issue("foam.owner_not_below_neighbour", Some(i as u32), Some(m.owner[i]), format!("owner {} >= neighbour {}", m.owner[i], m.neighbour[i])));
        }
    }
    for i in 1..ni {
        let (a, b) = ((m.owner[i - 1], m.neighbour[i - 1]), (m.owner[i], m.neighbour[i]));
        if a > b {
            out.push(issue("foam.not_upper_triangular", Some(i as u32), Some(m.owner[i]), format!("face {i} ({}, {}) comes after ({}, {})", b.0, b.1, a.0, a.1)));
        } else if a == b {
            out.push(issue("foam.upper_triangular_tie", Some(i as u32), Some(m.owner[i]), format!("faces {} and {i} both join cells {} and {}", i - 1, b.0, b.1)));
        }
    }
    let mut next = ni as u64;
    for p in &m.patches {
        if u64::from(p.start_face) != next || u64::from(p.start_face) + u64::from(p.n_faces) > nf as u64 {
            out.push(issue("foam.patch_range", None, None, format!("patch '{}' [{}, +{}) does not continue at face {next} (n_faces {nf})", p.name, p.start_face, p.n_faces)));
        }
        next = u64::from(p.start_face) + u64::from(p.n_faces);
    }
    if next != nf as u64 {
        out.push(issue("foam.patch_range", None, None, format!("patches end at face {next}, the mesh has {nf} faces")));
    }
    let mut used = vec![false; np];
    for v in &m.face_points {
        used[*v as usize] = true;
    }
    for (i, u) in used.iter().enumerate() {
        if !u {
            out.push(issue("foam.point_unused", None, None, format!("point {i} belongs to no face")));
        }
    }

    // ---- geometry ------------------------------------------------------------------------------
    let geo: Vec<(V3, V3)> = (0..nf).into_par_iter().map(|f| face_geometry(&m.points, m.face(f))).collect();
    let mut nfaces = vec![0u32; nc];
    let mut sum_sf = vec![[0.0f64; 3]; nc];
    let mut sum_abs = vec![0.0f64; nc];
    let mut est = vec![[0.0f64; 3]; nc];
    // Fixed face order: deterministic.
    for f in 0..nf {
        let (sf, fc) = geo[f];
        let len = norm(sf);
        let o = m.owner[f] as usize;
        nfaces[o] += 1;
        for d in 0..3 {
            sum_sf[o][d] += sf[d];
            est[o][d] += fc[d];
        }
        sum_abs[o] += len;
        if f < ni {
            let n = m.neighbour[f] as usize;
            nfaces[n] += 1;
            for d in 0..3 {
                sum_sf[n][d] -= sf[d];
                est[n][d] += fc[d];
            }
            sum_abs[n] += len;
        }
    }
    for c in 0..nc {
        if nfaces[c] > 0 {
            for d in 0..3 {
                est[c][d] /= f64::from(nfaces[c]);
            }
        }
    }
    // Pyramid decomposition about the face-centre mean: volume and centroid.
    let mut vol = vec![0.0f64; nc];
    let mut cen = vec![[0.0f64; 3]; nc];
    for f in 0..nf {
        let (sf, fc) = geo[f];
        let mut add = |c: usize, sign: f64| {
            let pv = sign * dot(sf, sub(fc, est[c])) / 3.0;
            vol[c] += pv;
            for d in 0..3 {
                cen[c][d] += pv * (0.75 * fc[d] + 0.25 * est[c][d]);
            }
        };
        add(m.owner[f] as usize, 1.0);
        if f < ni {
            add(m.neighbour[f] as usize, -1.0);
        }
    }
    for c in 0..nc {
        if nfaces[c] < 4 {
            out.push(issue("foam.cell_too_few_faces", None, Some(c as u32), format!("cell {c} has {} faces", nfaces[c])));
        }
        if norm(sum_sf[c]) > tol * sum_abs[c] {
            out.push(issue("foam.cell_unclosed", None, Some(c as u32), format!("|sum Sf| = {:e} against sum |Sf| = {:e}", norm(sum_sf[c]), sum_abs[c])));
        }
        if vol[c] <= 0.0 || vol[c].is_nan() {
            out.push(issue("foam.cell_nonpositive_volume", None, Some(c as u32), format!("volume {:e}", vol[c])));
        } else {
            for d in 0..3 {
                cen[c][d] /= vol[c];
            }
        }
    }
    for f in 0..nf {
        let c = m.owner[f] as usize;
        if vol[c] > 0.0 && dot(geo[f].0, sub(geo[f].1, cen[c])) <= 0.0 {
            out.push(issue("foam.face_orientation", Some(f as u32), Some(c as u32), format!("face {f} does not point out of its owner {c}")));
        }
    }
    out
}
