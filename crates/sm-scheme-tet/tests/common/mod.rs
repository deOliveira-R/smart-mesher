//! Shared helpers for the acceptance tests.
#![allow(dead_code)]

use sm_scheme_tet::{SurfaceInput, TetMesh, ori};
use sm_testkit::surf3::SurfaceFixture;
use std::collections::BTreeMap;

mod r;
#[allow(unused_imports)]
pub use r::*;

pub fn input_of(f: &SurfaceFixture, v: usize) -> SurfaceInput {
    SurfaceInput { points: f.points.clone(), tris: f.oriented_tris(v) }
}

/// Signed volume of a tet (positive = template orientation).
pub fn tet_vol(m: &TetMesh, t: [u32; 4]) -> f64 {
    let q = t.map(|i| m.points[i as usize]);
    let d = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let (u, v, w) = (d(q[1], q[0]), d(q[2], q[0]), d(q[3], q[0]));
    (u[0] * (v[1] * w[2] - v[2] * w[1]) - u[1] * (v[0] * w[2] - v[2] * w[0]) + u[2] * (v[0] * w[1] - v[1] * w[0])) / 6.0
}

pub fn total_volume(m: &TetMesh) -> f64 {
    sm_math::sum::sum_neumaier(&m.tets.iter().map(|&t| tet_vol(m, t)).collect::<Vec<_>>())
}

/// Every tet strictly positive under exact arithmetic.
pub fn all_exact_positive(m: &TetMesh) -> bool {
    m.tets.iter().all(|t| {
        let q = t.map(|i| m.points[i as usize]);
        ori(q[0], q[1], q[2], q[3]) > 0
    })
}

/// Check that each input triangle is a face of exactly one tet, with the tet on the inner side.
pub fn check_boundary(m: &TetMesh, tris: &[[u32; 3]]) -> Result<(), String> {
    // Map canonical oriented tet faces.
    let canon = |t: [u32; 3]| {
        if t[0] <= t[1] && t[0] <= t[2] { t } else if t[1] <= t[2] { [t[1], t[2], t[0]] } else { [t[2], t[0], t[1]] }
    };
    let mut faces: BTreeMap<[u32; 3], usize> = BTreeMap::new();
    for t in &m.tets {
        for f in [[t[1], t[2], t[3]], [t[0], t[3], t[2]], [t[0], t[1], t[3]], [t[0], t[2], t[1]]] {
            *faces.entry(canon(f)).or_default() += 1;
        }
    }
    for (i, t) in tris.iter().enumerate() {
        match faces.get(&canon(*t)) {
            Some(1) => {}
            other => return Err(format!("tri {i} {t:?}: outward face count {other:?}")),
        }
    }
    // Every outward face whose reverse is absent must be an input triangle.
    let tri_set: std::collections::BTreeSet<[u32; 3]> = tris.iter().map(|t| canon(*t)).collect();
    for f in faces.keys() {
        let rev = canon([f[0], f[2], f[1]]);
        if !faces.contains_key(&rev) && !tri_set.contains(f) {
            return Err(format!("boundary face {f:?} is not an input triangle"));
        }
    }
    Ok(())
}

fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Distance from `p` to triangle `a,b,c` (Ericson).
pub fn point_tri_dist(p: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    let (ab, ac, ap) = (sub3(b, a), sub3(c, a), sub3(p, a));
    let (d1, d2) = (dot3(ab, ap), dot3(ac, ap));
    let at = |q: [f64; 3]| dot3(sub3(p, q), sub3(p, q)).sqrt();
    let mix = |q: [f64; 3], u: [f64; 3], s: f64| at([q[0] + u[0] * s, q[1] + u[1] * s, q[2] + u[2] * s]);
    if d1 <= 0.0 && d2 <= 0.0 {
        return at(a);
    }
    let bp = sub3(p, b);
    let (d3, d4) = (dot3(ab, bp), dot3(ac, bp));
    if d3 >= 0.0 && d4 <= d3 {
        return at(b);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return mix(a, ab, d1 / (d1 - d3));
    }
    let cp = sub3(p, c);
    let (d5, d6) = (dot3(ab, cp), dot3(ac, cp));
    if d6 >= 0.0 && d5 <= d6 {
        return at(c);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return mix(a, ac, d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return mix(b, sub3(c, b), w);
    }
    let den = 1.0 / (va + vb + vc);
    let (v, w) = (vb * den, vc * den);
    at([a[0] + ab[0] * v + ac[0] * w, a[1] + ab[1] * v + ac[1] * w, a[2] + ab[2] * v + ac[2] * w])
}

/// Smallest `dist(p, T) / h_T` over the Steiner points `p` of `m` and the input triangles `T`
/// (`h_T` = mean edge length of `T`); `f64::INFINITY` without Steiner points.
pub fn min_s4_margin(m: &TetMesh, inp: &SurfaceInput) -> f64 {
    let mut worst = f64::INFINITY;
    for p in &m.points[m.n_input..] {
        for t in &inp.tris {
            let [a, b, c] = t.map(|i| inp.points[i as usize]);
            let e = |x: [f64; 3], y: [f64; 3]| dot3(sub3(x, y), sub3(x, y)).sqrt();
            let h = (e(a, b) + e(b, c) + e(c, a)) / 3.0;
            worst = worst.min(point_tri_dist(*p, a, b, c) / h);
        }
    }
    worst
}

/// The sharpest solid-side angle (degrees) between two input triangles sharing an edge: an
/// upper bound on the smallest dihedral angle of any mesh of the surface (the tets around that
/// edge split the wedge).
pub fn min_solid_wedge_deg(inp: &SurfaceInput) -> f64 {
    let p = |i: u32| inp.points[i as usize];
    let unit = |a: [f64; 3]| {
        let l = dot3(a, a).sqrt();
        [a[0] / l, a[1] / l, a[2] / l]
    };
    let cross = |a: [f64; 3], b: [f64; 3]| [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
    // Directed edge -> third vertex of the triangle that has it.
    let mut third: BTreeMap<(u32, u32), u32> = BTreeMap::new();
    for t in &inp.tris {
        for r in 0..3 {
            third.insert((t[r], t[(r + 1) % 3]), t[(r + 2) % 3]);
        }
    }
    let mut worst = 180.0f64;
    for (&(a, b), &c) in &third {
        let Some(&d) = third.get(&(b, a)) else { continue };
        let e = unit(sub3(p(b), p(a)));
        let perp = |x: [f64; 3]| {
            let v = sub3(x, p(a));
            let s = dot3(v, e);
            unit([v[0] - s * e[0], v[1] - s * e[1], v[2] - s * e[2]])
        };
        let (u1, u2) = (perp(p(c)), perp(p(d)));
        let phi = dot3(u1, u2).clamp(-1.0, 1.0).acos().to_degrees();
        // Outward normal of (a, b, c): the solid lies on its negative side.
        let n1 = cross(sub3(p(b), p(a)), sub3(p(c), p(a)));
        let ang = if dot3(u2, n1) < 0.0 { phi } else { 360.0 - phi };
        worst = worst.min(ang);
    }
    worst
}

/// Smallest dihedral angle (degrees) over the tets with a Steiner vertex and over those with
/// input vertices only.
pub fn min_dihedral_split(m: &TetMesh) -> (f64, f64) {
    let (mut with, mut without) = (180.0f64, 180.0f64);
    for t in &m.tets {
        let q = sm_scheme_tet::tet_quality(t.map(|i| m.points[i as usize])).min_dihedral;
        if t.iter().any(|&i| i as usize >= m.n_input) {
            with = with.min(q);
        } else {
            without = without.min(q);
        }
    }
    (with, without)
}

/// `tri_cube(n)` whose top face carries a `k x k` hard relief patch in its corner: the interior
/// vertices of the patch pushed down by up to `depth * k / n` (SplitMix64 in point order), the
/// relief cases of `a15` scaled into a finer cube.
pub fn patch_cube(n: u32, k: u32, seed: u64, depth: f64) -> SurfaceInput {
    let f = sm_testkit::surf3::tri_cube(n);
    let mut inp = input_of(&f, 0);
    let mut rng = sm_testkit::surf3::SplitMix64(seed);
    let lim = f64::from(k) / f64::from(n);
    for p in &mut inp.points {
        if p[2] == 1.0 && p[0] > 0.0 && p[0] < lim && p[1] > 0.0 && p[1] < lim {
            p[2] = 1.0 - depth * lim * rng.unit().abs();
        }
    }
    inp
}
