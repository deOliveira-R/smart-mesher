//! Circumcentric dual points: circumcentres of tets and boundary triangles, and the exact
//! welding of points that coincide.
//!
//! Welding is decided by exact predicates, never by a tolerance. Two tets of one region that
//! share a face are one group when the far vertex of one is exactly on the circumsphere of the
//! other (`insphere = 0`, as on a Kuhn lattice, where the six tets of a cube share the cube
//! centre). A boundary triangle's circumcentre is welded with the midpoint of an edge when the
//! triangle has an exact right angle opposite that edge, and with an adjacent tet when the tet's
//! circumcentre lies exactly in the triangle's plane. A group is represented by one anchor point
//! (an edge midpoint over a triangle over a tet, then the lowest id), whose coordinates every
//! member uses, so welded points are bit-identical.

use crate::error::PolyError;
use crate::geom::{self, V3};
use crate::measure::circumcentre_side;
use crate::prep::pair_key;
use rayon::prelude::*;
use sm_math::exact::Expansion;
use sm_math::{Point3, Sign, predicates};
use sm_mesh_api::{CellIx, FaceIx, MeshEntityKind, MeshRead};

fn pt(m: &dyn MeshRead, v: u32) -> V3 {
    let co = m.coords();
    [co.x[v as usize], co.y[v as usize], co.z[v as usize]]
}

fn p3(p: V3) -> Point3 {
    Point3::new(p[0], p[1], p[2])
}

/// Circumcentre of tet `abcd`: `a + (|B|^2 C x D + |C|^2 D x B + |D|^2 B x C) / (2 B.(C x D))`
/// with `B = b - a` and so on. The caller guarantees a non-flat tet.
pub(crate) fn tet_circumcentre(a: V3, b: V3, c: V3, d: V3) -> V3 {
    let (bb, cc, dd) = (geom::sub(b, a), geom::sub(c, a), geom::sub(d, a));
    let (cd, db, bc) = (geom::cross(cc, dd), geom::cross(dd, bb), geom::cross(bb, cc));
    let k = 1.0 / (2.0 * geom::dot(bb, cd));
    let (b2, c2, d2) = (geom::dot(bb, bb), geom::dot(cc, cc), geom::dot(dd, dd));
    [0, 1, 2].map(|i| a[i] + (b2 * cd[i] + c2 * db[i] + d2 * bc[i]) * k)
}

/// Circumcentre of triangle `abc` (in its plane): `a + ((|B|^2 C - |C|^2 B) x n) / (2 |n|^2)`,
/// `n = B x C`. A degenerate triangle gives non-finite coordinates.
pub(crate) fn tri_circumcentre(a: V3, b: V3, c: V3) -> V3 {
    let (bb, cc) = (geom::sub(b, a), geom::sub(c, a));
    let n = geom::cross(bb, cc);
    let (b2, c2) = (geom::dot(bb, bb), geom::dot(cc, cc));
    let u = [0, 1, 2].map(|i| b2 * cc[i] - c2 * bb[i]);
    let w = geom::cross(u, n);
    let k = 1.0 / (2.0 * geom::dot(n, n));
    [0, 1, 2].map(|i| a[i] + w[i] * k)
}

/// The angle at `x` of triangle `xyz` is exactly a right angle.
fn right_angle_at(x: V3, y: V3, z: V3) -> bool {
    type E = Expansion;
    let d = |i: usize| E::from_diff(y[i], x[i]).mul(&E::from_diff(z[i], x[i]));
    d(0).add(&d(1)).add(&d(2)).sign() == Sign::Zero
}

/// Circumcentres and the weld table of a source mesh.
pub(crate) struct CircumPoints {
    /// Circumcentre of every tet (its own, before welding).
    pub tcirc: Vec<V3>,
    /// Anchor of every id below `nt + nf + ne` (tets, then boundary triangles, then edge mids).
    pub anchor: Vec<u32>,
}

struct Uf(Vec<u32>);

impl Uf {
    fn find(&mut self, mut x: u32) -> u32 {
        while self.0[x as usize] != x {
            let p = self.0[self.0[x as usize] as usize];
            self.0[x as usize] = p;
            x = p;
        }
        x
    }
    fn union(&mut self, a: u32, b: u32) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.0[ra.max(rb) as usize] = ra.min(rb);
        }
    }
}

/// Compute circumcentres and welds. `tv` are the tet corners, `treg` their regions, `ekeys` the
/// sorted edge keys ([`pair_key`]). Fails on a flat tet (the first by index).
pub(crate) fn build(src: &dyn MeshRead, tv: &[[u32; 4]], treg: &[u32], ekeys: &[u64]) -> Result<CircumPoints, PolyError> {
    let (nt, nf, ne) = (tv.len(), src.count(MeshEntityKind::Face), ekeys.len());
    let tc: Vec<Option<V3>> = tv
        .par_iter()
        .map(|t| {
            let p = t.map(|v| pt(src, v));
            (predicates::orient3d(p3(p[0]), p3(p[1]), p3(p[2]), p3(p[3])) != Sign::Zero)
                .then(|| tet_circumcentre(p[0], p[1], p[2], p[3]))
        })
        .collect();
    if let Some(c) = tc.iter().position(Option::is_none) {
        return Err(PolyError::DegenerateTet { cell: CellIx::new(c) });
    }
    let tcirc: Vec<V3> = tc.into_iter().flatten().collect();
    let p_tri = |f: usize| (nt + f) as u32;
    let p_mid = |e: usize| (nt + nf + e) as u32;
    // Welds found per face, in parallel; applied in face order.
    let found: Vec<Vec<(u32, u32)>> = (0..nf)
        .into_par_iter()
        .map(|f| {
            let fi = FaceIx::new(f);
            let l = src.face_left(fi).get();
            let r = src.face_right(fi).map(|r| r.get());
            let fv = src.face_vertices(fi);
            let mut out = Vec::new();
            if fv.len() != 3 {
                return out;
            }
            let far = |t: usize| tv[t].iter().copied().find(|v| !fv.iter().any(|w| w.0 == *v));
            let tri = [pt(src, fv[0].0), pt(src, fv[1].0), pt(src, fv[2].0)];
            if let Some(r) = r
                && treg[l] == treg[r]
                && let (Some(dl), Some(dr)) = (far(l), far(r))
                && predicates::insphere(p3(tri[0]), p3(tri[1]), p3(tri[2]), p3(pt(src, dl)), p3(pt(src, dr))) == Sign::Zero
            {
                out.push((l as u32, r as u32));
            }
            if r.is_none_or(|r| treg[l] != treg[r]) {
                for k in 0..3 {
                    let (y, z) = (fv[(k + 1) % 3].0, fv[(k + 2) % 3].0);
                    if right_angle_at(tri[k], tri[(k + 1) % 3], tri[(k + 2) % 3])
                        && let Ok(e) = ekeys.binary_search(&pair_key(y, z))
                    {
                        out.push((p_tri(f), p_mid(e)));
                    }
                }
                for t in std::iter::once(l).chain(r) {
                    if let Some(d) = far(t)
                        && circumcentre_side(tri[0], tri[1], tri[2], pt(src, d)) == Sign::Zero
                    {
                        out.push((p_tri(f), t as u32));
                    }
                }
            }
            out
        })
        .collect();
    let n = nt + nf + ne;
    let mut uf = Uf((0..n as u32).collect());
    for (a, b) in found.into_iter().flatten() {
        uf.union(a, b);
    }
    // Anchor of a group: the best of (edge mid, triangle, tet), then the lowest id.
    let rank = |id: u32| -> (u8, u32) {
        let i = id as usize;
        (if i < nt { 2 } else if i < nt + nf { 1 } else { 0 }, id)
    };
    let mut best = vec![u32::MAX; n];
    for id in 0..n as u32 {
        let r = uf.find(id) as usize;
        if best[r] == u32::MAX || rank(id) < rank(best[r]) {
            best[r] = id;
        }
    }
    let anchor: Vec<u32> = (0..n as u32).map(|id| best[uf.find(id) as usize]).collect();
    Ok(CircumPoints { tcirc, anchor })
}
