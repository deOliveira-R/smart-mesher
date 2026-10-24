//! Face-loop based kernels shared by the template solids and general polyhedra: fan volume,
//! surface area, dihedral angles, star-shapedness.
//!
//! A polyhedron is given as points plus outward-oriented loops (right-hand rule, normal out).

use crate::vec::{TINY, V, add, cross, dot, norm, scale, sub, unit};
use smallvec::SmallVec;

/// Local vertex index of a face loop.
pub(crate) trait Ix: Copy {
    fn ix(self) -> usize;
}
impl Ix for u8 {
    #[inline]
    fn ix(self) -> usize {
        self as usize
    }
}
impl Ix for u32 {
    #[inline]
    fn ix(self) -> usize {
        self as usize
    }
}

fn mean<I: Ix>(p: &[V], l: &[I]) -> V {
    let s = l.iter().fold([0.0; 3], |s, &i| add(s, p[i.ix()]));
    scale(s, 1.0 / l.len() as f64)
}

/// Area vector and apex of a loop under `smartmesh.fan_vertex_mean.v1`.
pub(crate) fn face_area_vector<I: Ix>(p: &[V], l: &[I]) -> (V, V) {
    let apex = mean(p, l);
    let n = l.len();
    let mut a = [0.0; 3];
    for i in 0..n {
        let (u, v) = (sub(p[l[i].ix()], apex), sub(p[l[(i + 1) % n].ix()], apex));
        a = add(a, scale(cross(u, v), 0.5));
    }
    (a, apex)
}

/// Polyhedron volume under the fan rule (`1/3 sum A_f . (apex_f - o)`, `o` = first vertex).
pub(crate) fn fan_volume<I: Ix>(p: &[V], faces: &[&[I]]) -> f64 {
    let Some(first) = faces.first() else { return 0.0 };
    let o = p[first[0].ix()];
    let mut v = 0.0;
    for l in faces {
        let (a, apex) = face_area_vector(p, l);
        v += dot(a, sub(apex, o));
    }
    v / 3.0
}

/// Total surface area (sum of fan-rule face areas).
pub(crate) fn surface_area<I: Ix>(p: &[V], faces: &[&[I]]) -> f64 {
    faces.iter().map(|l| norm(face_area_vector(p, l).0)).sum()
}

/// Wadell sphericity `pi^(1/3) (6 V)^(2/3) / S`; 0 for a non-positive volume.
pub(crate) fn sphericity<I: Ix>(p: &[V], faces: &[&[I]]) -> f64 {
    let (v, s) = (fan_volume(p, faces), surface_area(p, faces));
    if v <= TINY || s <= TINY {
        return 0.0;
    }
    core::f64::consts::PI.cbrt() * (6.0 * v).powf(2.0 / 3.0) / s
}

/// Smallest `cos` of the angle between an outward face normal and the direction from the vertex
/// mean to the face mean. Positive iff every face is "visible" from the vertex mean.
pub(crate) fn starness<I: Ix>(p: &[V], faces: &[&[I]]) -> f64 {
    let all = p.iter().fold([0.0; 3], |s, &q| add(s, q));
    let c = scale(all, 1.0 / p.len().max(1) as f64);
    let mut m = f64::INFINITY;
    for l in faces {
        let (a, apex) = face_area_vector(p, l);
        let d = sub(apex, c);
        let den = norm(a) * norm(d);
        m = m.min(if den <= TINY { 0.0 } else { dot(a, d) / den });
    }
    m
}

/// `(min, max)` interior dihedral angle in degrees over the edges that two faces share, or
/// `None` if no edge has a twin face.
///
/// Edges next to a face with a zero area vector (collapsed face) have no defined dihedral angle
/// and are skipped. If edges exist but none is valid (a flat hex, a collapsed tet), the cell has
/// no meaningful angle: the result is `(0, 180)`, so it shows up as the worst cell for both
/// `MinAngle` (0) and `MaxAngle` (180) instead of looking perfectly flat.
pub(crate) fn dihedral_extrema<I: Ix>(p: &[V], faces: &[&[I]]) -> Option<(f64, f64)> {
    let normals: SmallVec<[V; 16]> = faces.iter().map(|l| unit(face_area_vector(p, l).0)).collect();
    // Directed edges (from, to, face), sorted: twin lookup is a binary search, not a face scan.
    let mut directed: SmallVec<[(u32, u32, u32); 64]> = SmallVec::new();
    for (f, lf) in faces.iter().enumerate() {
        let n = lf.len();
        for i in 0..n {
            directed.push((lf[i].ix() as u32, lf[(i + 1) % n].ix() as u32, f as u32));
        }
    }
    directed.sort_unstable();
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let mut degenerate_edges = false;
    for (f, lf) in faces.iter().enumerate() {
        let n = lf.len();
        for i in 0..n {
            let (a, b) = (lf[i].ix(), lf[(i + 1) % n].ix());
            if a >= b {
                continue;
            }
            // The twin face traverses b -> a (smallest face index first, via the sorted edge map).
            let k = directed.partition_point(|&(x, y, _)| (x, y) < (b as u32, a as u32));
            let Some(&(x, y, g)) = directed.get(k) else { continue };
            if (x, y) != (b as u32, a as u32) {
                continue;
            }
            let g = g as usize;
            if normals[f] == [0.0; 3] || normals[g] == [0.0; 3] {
                degenerate_edges = true;
                continue;
            }
            let e = unit(sub(p[b], p[a]));
            let phi = dot(cross(normals[f], normals[g]), e).atan2(dot(normals[f], normals[g]));
            let interior = 180.0 - phi.to_degrees();
            lo = lo.min(interior);
            hi = hi.max(interior);
        }
    }
    if lo <= hi {
        Some((lo, hi))
    } else {
        degenerate_edges.then_some((0.0, 180.0))
    }
}
