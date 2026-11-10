//! Dual eligibility of a tet mesh (can a circumcentric dual be built?) and neighbour-pair
//! statistics of a polyhedral mesh.
//!
//! A circumcentric (Voronoi) dual of a tet mesh is orthogonal and convex only if the mesh is
//! locally Delaunay and every circumcentre lies inside its region. [`dual_eligibility`] counts the
//! violations of both, with exact predicates and without constructing a circumcentre.

use rayon::prelude::*;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_math::exact::Expansion;
use sm_math::{Point3, Sign, predicates};
use sm_mesh_api::{CellIx, CellShape, FaceIx, MeshDim, MeshEntityKind, MeshRead, VertexIx};

use crate::stats::CHUNK;

/// Counts that decide whether a circumcentric dual of a tet mesh is valid, see
/// [`dual_eligibility`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[non_exhaustive]
pub struct DualEligibility {
    /// Tetrahedra.
    pub tets: usize,
    /// Interior faces (two tets).
    pub interior_faces: usize,
    /// Interior faces that are not locally Delaunay: the far vertex of one tet is strictly inside
    /// the circumsphere of the other (exact `insphere`).
    pub non_delaunay_faces: usize,
    /// Tets whose circumcentre lies strictly outside their region: across a boundary face or a
    /// face towards another region (exact; on the face counts as inside).
    pub outside_circumcentres: usize,
    /// `non_delaunay_faces / interior_faces` (0 without interior faces).
    pub non_delaunay_share: f64,
    /// `outside_circumcentres / tets` (0 without tets).
    pub outside_share: f64,
}

pub(crate) type P = [f64; 3];

fn pt(m: &dyn MeshRead, v: VertexIx) -> P {
    let co = m.coords();
    [co.x[v.get()], co.y[v.get()], co.z[v.get()]]
}

fn p3(p: P) -> Point3 {
    Point3::new(p[0], p[1], p[2])
}

/// Exact sign of `(circumcentre(a, b, c, d) - a) . n`, times the sign of `(d - a) . n`, where
/// `n = (b - a) x (c - a)`: positive when the circumcentre of `abcd` is on the same side of the
/// plane `abc` as `d`, negative when on the other side, zero on the plane.
///
/// With `B = b - a`, `C = c - a`, `D = d - a`, the circumcentre offset is
/// `(|B|^2 C x D + |C|^2 D x B + |D|^2 B x C) / (2 B.(C x D))`, so the sign is that of
/// `N = |B|^2 (C x D).n + |C|^2 (D x B).n + |D|^2 |n|^2` (`B.(C x D) = D.n`).
pub(crate) fn circumcentre_side(a: P, b: P, c: P, d: P) -> Sign {
    // Floating-point filter: evaluate `N` in `f64` together with the same expression over absolute
    // values (a permanent, so no cancellation). `N` has depth six in the rounded differences, so
    // its rounding error is below about 30 ulp of that permanent; the bound below is three times
    // larger. A clear sign is returned; anything near zero, or out of the safe magnitude range,
    // goes to the exact path.
    let df = |p: P| [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let (bb, cc, dd) = (df(b), df(c), df(d));
    let cr = |u: [f64; 3], v: [f64; 3]| [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    let cra = |u: [f64; 3], v: [f64; 3]| [(u[1] * v[2]).abs() + (u[2] * v[1]).abs(), (u[2] * v[0]).abs() + (u[0] * v[2]).abs(), (u[0] * v[1]).abs() + (u[1] * v[0]).abs()];
    let dt = |u: [f64; 3], v: [f64; 3]| u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    let (n, na) = (cr(bb, cc), cra(bb, cc));
    let (cd, cda) = (cr(cc, dd), cra(cc, dd));
    let (db, dba) = (cr(dd, bb), cra(dd, bb));
    let (b2, c2, d2) = (dt(bb, bb), dt(cc, cc), dt(dd, dd));
    let nn = dt(n, n);
    let val = b2 * dt(cd, n) + c2 * dt(db, n) + d2 * nn;
    let mag = b2 * dt(cda, na) + c2 * dt(dba, na) + d2 * nn;
    if mag.is_finite() && (1e-100..1e100).contains(&mag) && val.abs() > 1e-14 * mag {
        return if val > 0.0 { Sign::Positive } else { Sign::Negative };
    }
    circumcentre_side_exact(a, b, c, d)
}

/// The expansion-arithmetic evaluation of [`circumcentre_side`] (always exact).
pub(crate) fn circumcentre_side_exact(a: P, b: P, c: P, d: P) -> Sign {
    type E = Expansion;
    let diff = |p: P| -> [E; 3] { [E::from_diff(p[0], a[0]), E::from_diff(p[1], a[1]), E::from_diff(p[2], a[2])] };
    let cross = |u: &[E; 3], v: &[E; 3]| -> [E; 3] {
        [u[1].mul(&v[2]).sub(&u[2].mul(&v[1])), u[2].mul(&v[0]).sub(&u[0].mul(&v[2])), u[0].mul(&v[1]).sub(&u[1].mul(&v[0]))]
    };
    let dot = |u: &[E; 3], v: &[E; 3]| -> E { u[0].mul(&v[0]).add(&u[1].mul(&v[1])).add(&u[2].mul(&v[2])) };
    let (bb, cc, dd) = (diff(b), diff(c), diff(d));
    let n = cross(&bb, &cc);
    let t1 = dot(&bb, &bb).mul(&dot(&cross(&cc, &dd), &n));
    let t2 = dot(&cc, &cc).mul(&dot(&cross(&dd, &bb), &n));
    let t3 = dot(&dd, &dd).mul(&dot(&n, &n));
    t1.add(&t2).add(&t3).sign()
}

/// The four corners of tet `c`, if it is one.
fn tet_corners(m: &dyn MeshRead, c: CellIx) -> Option<[VertexIx; 4]> {
    if m.cell_shape(c) != CellShape::Tet {
        return None;
    }
    let k = m.cell_corners(c)?;
    (k.len() == 4).then(|| [k[0], k[1], k[2], k[3]])
}

/// Face `f` is interior and not locally Delaunay: the far vertex of one tet is strictly inside the
/// circumsphere of the other (exact `insphere`; cospherical is accepted).
fn face_non_delaunay(m: &dyn MeshRead, f: FaceIx) -> Option<bool> {
    let r = m.face_right(f)?;
    let l = m.face_left(f);
    let (Some(tl), Some(tr)) = (tet_corners(m, l), tet_corners(m, r)) else { return Some(false) };
    let fv = m.face_vertices(f);
    let far = |t: &[VertexIx; 4]| t.iter().copied().find(|v| !fv.contains(v));
    let (Some(dl), Some(dr)) = (far(&tl), far(&tr)) else { return Some(false) };
    let (a, b, c) = (p3(pt(m, fv[0])), p3(pt(m, fv[1])), p3(pt(m, fv[2])));
    let o = predicates::orient3d(a, b, c, p3(pt(m, dl)));
    let s = predicates::insphere(a, b, c, p3(pt(m, dl)), p3(pt(m, dr)));
    // `insphere` is positive inside for a positively oriented `abcd`.
    Some(match (o, s) {
        (Sign::Zero, _) | (_, Sign::Zero) => false,
        (o, s) => o == s,
    })
}

/// The circumcentre of tet `c` lies strictly outside one of its faces that is a model boundary or
/// separates two regions (exact; on the face counts as inside).
fn tet_outside(m: &dyn MeshRead, c: CellIx) -> bool {
    let Some(t) = tet_corners(m, c) else { return false };
    let region = m.classification(MeshEntityKind::Cell, c.get() as u32);
    for sf in m.cell_faces(c) {
        let walled = match m.across(*sf) {
            None => true,
            Some(o) => m.classification(MeshEntityKind::Cell, o.get() as u32) != region,
        };
        if !walled {
            continue;
        }
        let fv = m.face_vertices(sf.face());
        let Some(d) = t.iter().copied().find(|v| !fv.contains(v)) else { continue };
        if fv.len() == 3 && circumcentre_side(pt(m, fv[0]), pt(m, fv[1]), pt(m, fv[2]), pt(m, d)) == Sign::Negative {
            return true;
        }
    }
    false
}

/// Counts for one chunk of faces (interior faces, non-Delaunay faces) and of cells (outside).
fn count_chunk(m: &dyn MeshRead, k: usize, nf: usize, nc: usize) -> (usize, usize, usize) {
    let (mut interior, mut non_del, mut outside) = (0, 0, 0);
    for f in k * CHUNK..((k + 1) * CHUNK).min(nf) {
        if let Some(bad) = face_non_delaunay(m, FaceIx::new(f)) {
            interior += 1;
            non_del += usize::from(bad);
        }
    }
    // Cells of this chunk index range (same chunking over cells).
    for c in k * CHUNK..((k + 1) * CHUNK).min(nc) {
        outside += usize::from(tet_outside(m, CellIx::new(c)));
    }
    (interior, non_del, outside)
}

/// Where the first violation is: the centroid of the lowest non-Delaunay face, else the vertex
/// mean of the lowest tet with an outside circumcentre. Serial; for the refusal message only.
pub(crate) fn first_violation(m: &dyn MeshRead) -> Option<[f64; 3]> {
    let mean = |vs: &[VertexIx]| {
        let n = vs.len() as f64;
        let mut s = [0.0; 3];
        for v in vs {
            let p = pt(m, *v);
            s = [s[0] + p[0], s[1] + p[1], s[2] + p[2]];
        }
        [s[0] / n, s[1] / n, s[2] / n]
    };
    let (nc, nf) = (m.count(MeshEntityKind::Cell), m.count(MeshEntityKind::Face));
    if let Some(f) = (0..nf).find(|&f| face_non_delaunay(m, FaceIx::new(f)) == Some(true)) {
        return Some(mean(m.face_vertices(FaceIx::new(f))));
    }
    let c = (0..nc).find(|&c| tet_outside(m, CellIx::new(c)))?;
    tet_corners(m, CellIx::new(c)).map(|t| mean(&t))
}

/// Dual-eligibility counts of an all-tet 3-D mesh; `None` for any other mesh.
///
/// * Non-Delaunay faces: for each interior face `abc` between tets `abcd1` and `abcd2`, the
///   exact `insphere(a, b, c, d1, d2)` (cospherical is accepted).
/// * Outside circumcentres: for each tet, the exact side of the circumcentre against each of its
///   faces that lies on the model boundary or towards another region (different cell
///   classification). Only the tet's own faces are tested: a circumcentre that leaves through a
///   facet of a different tet is not seen, so the count is a lower bound, not exact.
///
/// Counts are sums over fixed chunks, so the result does not depend on the thread count.
pub fn dual_eligibility(m: &dyn MeshRead) -> Option<DualEligibility> {
    if m.dim() != MeshDim::Three {
        return None;
    }
    let (nc, nf) = (m.count(MeshEntityKind::Cell), m.count(MeshEntityKind::Face));
    if nc == 0 || (0..nc).any(|c| m.cell_shape(CellIx::new(c)) != CellShape::Tet) {
        return None;
    }
    let parts: Vec<(usize, usize, usize)> =
        (0..nf.max(nc).div_ceil(CHUNK)).into_par_iter().map(|k| count_chunk(m, k, nf, nc)).collect();
    let (interior, non_del, outside) = parts.into_iter().fold((0, 0, 0), |a, b| (a.0 + b.0, a.1 + b.1, a.2 + b.2));
    Some(DualEligibility {
        tets: nc,
        interior_faces: interior,
        non_delaunay_faces: non_del,
        outside_circumcentres: outside,
        non_delaunay_share: if interior > 0 { non_del as f64 / interior as f64 } else { 0.0 },
        outside_share: outside as f64 / nc as f64,
    })
}

/// `(pairs of cells that share more than one interior face, interior faces in such pairs,
/// interior faces)`.
pub(crate) fn multi_face_pairs(m: &dyn MeshRead) -> (usize, usize, usize) {
    let nf = m.count(MeshEntityKind::Face);
    let mut pairs: Vec<(u32, u32)> = (0..nf)
        .filter_map(|f| {
            let f = FaceIx::new(f);
            let r = m.face_right(f)?;
            let l = m.face_left(f);
            Some((l.min(r).get() as u32, l.max(r).get() as u32))
        })
        .collect();
    let interior = pairs.len();
    pairs.sort_unstable();
    let (mut multi, mut in_multi, mut i) = (0, 0, 0);
    while i < pairs.len() {
        let mut j = i;
        while j < pairs.len() && pairs[j] == pairs[i] {
            j += 1;
        }
        if j - i > 1 {
            multi += 1;
            in_multi += j - i;
        }
        i = j;
    }
    (multi, in_multi, interior)
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use sm_testkit::strategies::config;

    fn sub(a: P, b: P) -> P {
        [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
    }
    fn cross(a: P, b: P) -> P {
        [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
    }
    fn dot(a: P, b: P) -> f64 {
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    }

    fn pt() -> impl Strategy<Value = P> {
        [-4.0f64..4.0, -4.0..4.0, -4.0..4.0]
    }

    proptest! {
        #![proptest_config(config(512))]
        // The filtered side equals the exact one, also on lattice points where zeros are common.
        #[test]
        fn filter_agrees_with_exact_on_lattice_and_reals(
            a in [-3i32..3, -3..3, -3..3], b in [-3i32..3, -3..3, -3..3],
            c in [-3i32..3, -3..3, -3..3], d in [-3i32..3, -3..3, -3..3],
            r in [-1.0f64..1.0, -1.0..1.0, -1.0..1.0], s in 0u8..2,
        ) {
            let f = |p: [i32; 3]| p.map(f64::from);
            let (a, b, c, d) = (f(a), f(b), f(c), f(d));
            prop_assert_eq!(circumcentre_side(a, b, c, d), circumcentre_side_exact(a, b, c, d));
            // a non-lattice perturbation of one point (`s` selects scaled-by-1e-7 or full)
            let k = if s == 0 { 1e-7 } else { 1.0 };
            let d2 = [d[0] + k * r[0], d[1] + k * r[1], d[2] + k * r[2]];
            prop_assert_eq!(circumcentre_side(a, b, c, d2), circumcentre_side_exact(a, b, c, d2));
        }
    }

    proptest! {
        #![proptest_config(config(256))]
        // The exact side agrees with a floating-point circumcentre whenever that is clear-cut.
        #[test]
        fn circumcentre_side_matches_floating_point(a in pt(), b in pt(), c in pt(), d in pt()) {
            let (bb, cc, dd) = (sub(b, a), sub(c, a), sub(d, a));
            let det = dot(bb, cross(cc, dd));
            prop_assume!(det.abs() > 1e-2);
            let (cd, db, bc) = (cross(cc, dd), cross(dd, bb), cross(bb, cc));
            let k = 1.0 / (2.0 * det);
            let o = [0, 1, 2].map(|i| (dot(bb, bb) * cd[i] + dot(cc, cc) * db[i] + dot(dd, dd) * bc[i]) * k);
            let nn = dot(bc, bc).sqrt();
            let (so, sd) = (dot(o, bc) / nn, dot(dd, bc) / nn);
            prop_assume!(so.abs() > 1e-6);
            let same = (so > 0.0) == (sd > 0.0);
            let got = circumcentre_side(a, b, c, d);
            prop_assert_eq!(got == Sign::Positive, same);
            prop_assert_eq!(got == Sign::Negative, !same);
        }
    }
}
