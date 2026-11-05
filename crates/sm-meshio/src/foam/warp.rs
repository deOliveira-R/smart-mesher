//! Export-time split of warped faces into planar pieces (W3b FOAM-WARP, triage GO 4(b)).
//!
//! The internal mesh stays exact. A polygonal face whose vertices do not lie in one plane is
//! replaced by the triangles (edge, apex) about the face centre `fc` that OpenFOAM itself uses
//! (area-weighted centroid of the triangles about the vertex mean). Each triangle is planar by
//! construction. The pieces' area vectors sum to the face's `Sf`, and the apex lies on the
//! plane `Sf . (x - fc) = 0`, so the volume `1/3 sum Sf . (fc - est)` of every adjacent cell is
//! conserved to rounding. Both cells see the same pieces, so the cells stay closed.

// Parallel arrays indexed by face read best as a range.
#![allow(clippy::needless_range_loop)]

use super::check::{concave_cell_count, face_geometry};
use super::{PolyFaceZone, PolyMesh, PolyPatch};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// Default [`warp_tolerance`](super::FoamOptions::warp_tolerance): relative planarity deviation
/// above which a face is split.
pub const WARP_TOLERANCE: f64 = 1e-9;

/// What the warped-face split did, recorded before and after (INT-9 counts).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct WarpReport {
    /// Faces that were split.
    pub faces_split: u64,
    /// Planar pieces written in their place.
    pub pieces: u64,
    /// Face-centre points added.
    pub points_added: u64,
    /// Cells with at least one split face.
    pub cells_affected: u64,
    /// Cells [`concave_cell_count`](super::concave_cell_count) flags before the split.
    pub concave_before: u64,
    /// The same count after the split.
    pub concave_after: u64,
}

/// Planarity deviation of face `face`: the largest distance of a vertex from the plane through
/// the face centre with the face's area-vector normal, divided by `sqrt(|Sf|)`. Zero for a
/// triangle, a planar face, or a face of zero area.
pub fn face_warpage(pm: &PolyMesh, face: usize) -> f64 {
    warpage(&pm.points, pm.face(face))
}

fn warpage(points: &[[f64; 3]], f: &[u32]) -> f64 {
    if f.len() <= 3 {
        return 0.0;
    }
    let (sf, fc) = face_geometry(points, f);
    let len = (sf[0] * sf[0] + sf[1] * sf[1] + sf[2] * sf[2]).sqrt();
    if len <= 0.0 {
        return 0.0;
    }
    let dev = f
        .iter()
        .map(|&v| {
            let p = points[v as usize];
            ((sf[0] * (p[0] - fc[0]) + sf[1] * (p[1] - fc[1]) + sf[2] * (p[2] - fc[2])) / len).abs()
        })
        .fold(0.0, f64::max);
    dev / len.sqrt()
}

/// Result of [`split_warped_faces`] with the maps the builder needs.
pub(crate) struct WarpInfo {
    pub report: WarpReport,
    /// Cells (ascending) with a split face.
    pub cells: Vec<u32>,
    /// Old foam face -> its first piece (length `n_faces + 1`, the last entry the new count).
    pub first_of_old: Vec<u32>,
    /// Internal faces added.
    pub added_internal: usize,
}

/// Replace every face with [`face_warpage`] above `tol` by planar triangles about its centre.
/// Returns `None` (mesh untouched) when no face qualifies. Faces keep their order and pieces are
/// consecutive, so the owner / neighbour order and the patch ranges survive.
pub(crate) fn split_warped_faces(pm: &mut PolyMesh, tol: f64) -> Option<WarpInfo> {
    let (nf, ni) = (pm.n_faces(), pm.n_internal());
    let apex: Vec<Option<[f64; 3]>> = (0..nf)
        .into_par_iter()
        .map(|f| {
            let l = pm.face(f);
            (l.len() > 3 && warpage(&pm.points, l) > tol).then(|| face_geometry(&pm.points, l).1)
        })
        .collect();
    if apex.iter().all(Option::is_none) {
        return None;
    }
    let concave_before = concave_cell_count(pm);
    let base_np = pm.points.len();
    let mut first = Vec::with_capacity(nf + 1);
    let mut points = pm.points.clone();
    let mut offsets: Vec<u64> = Vec::with_capacity(nf + 1);
    offsets.push(0);
    let mut fp: Vec<u32> = Vec::with_capacity(pm.face_points.len());
    let (mut owner, mut neighbour) = (Vec::with_capacity(nf), Vec::with_capacity(ni));
    let mut cells = BTreeSet::new();
    let (mut faces_split, mut pieces, mut added_internal) = (0u64, 0u64, 0usize);
    for f in 0..nf {
        first.push(owner.len() as u32);
        let l = pm.face(f);
        match apex[f] {
            None => {
                fp.extend_from_slice(l);
                offsets.push(fp.len() as u64);
                owner.push(pm.owner[f]);
                if f < ni {
                    neighbour.push(pm.neighbour[f]);
                }
            }
            Some(c) => {
                let ap = points.len() as u32;
                points.push(c);
                faces_split += 1;
                pieces += l.len() as u64;
                cells.insert(pm.owner[f]);
                if f < ni {
                    cells.insert(pm.neighbour[f]);
                    added_internal += l.len() - 1;
                }
                for e in 0..l.len() {
                    fp.extend_from_slice(&[l[e], l[(e + 1) % l.len()], ap]);
                    offsets.push(fp.len() as u64);
                    owner.push(pm.owner[f]);
                    if f < ni {
                        neighbour.push(pm.neighbour[f]);
                    }
                }
            }
        }
    }
    first.push(owner.len() as u32);
    let patches: Vec<PolyPatch> = pm
        .patches
        .iter()
        .map(|p| {
            let (a, b) = (p.start_face as usize, (p.start_face + p.n_faces) as usize);
            PolyPatch { start_face: first[a], n_faces: first[b] - first[a], ..p.clone() }
        })
        .collect();
    let face_zones: Vec<PolyFaceZone> = pm
        .face_zones
        .iter()
        .map(|z| {
            let mut labels = Vec::new();
            let mut flips = Vec::new();
            for (&l, &fl) in z.labels.iter().zip(&z.flips) {
                for n in first[l as usize]..first[l as usize + 1] {
                    labels.push(n);
                    flips.push(fl);
                }
            }
            PolyFaceZone { name: z.name.clone(), labels, flips }
        })
        .collect();
    let points_added = (points.len() - base_np) as u64;
    pm.points = points;
    pm.face_offsets = offsets;
    pm.face_points = fp;
    pm.owner = owner;
    pm.neighbour = neighbour;
    pm.patches = patches;
    pm.face_zones = face_zones;
    let report = WarpReport { faces_split, pieces, points_added, cells_affected: cells.len() as u64, concave_before, concave_after: concave_cell_count(pm) };
    Some(WarpInfo { report, cells: cells.into_iter().collect(), first_of_old: first, added_internal })
}
