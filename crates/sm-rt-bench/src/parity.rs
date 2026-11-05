//! Point-in-solid by ray parity on a triangle soup (brute force; for model dry runs and tests).

use crate::formats::Facets;

/// Number of crossings (mod 2) of a fixed skew ray from `p` with the **hull** of the facets (the
/// triangles that border the complement, region `-1`): `1` inside the model, `0` outside. Interior
/// interfaces between regions are ignored.
pub fn point_in_facets(f: &Facets, p: [f64; 3]) -> u8 {
    let n = 14.0f64.sqrt();
    let d = [1.0 / n, 2.0 / n, 3.0 / n];
    let mut hits = 0u32;
    for i in 0..f.tris.len() {
        if f.sides[i][0] != -1 && f.sides[i][1] != -1 {
            continue;
        }
        let c = f.corners(i);
        let e1 = [c[1][0] - c[0][0], c[1][1] - c[0][1], c[1][2] - c[0][2]];
        let e2 = [c[2][0] - c[0][0], c[2][1] - c[0][1], c[2][2] - c[0][2]];
        let h = [d[1] * e2[2] - d[2] * e2[1], d[2] * e2[0] - d[0] * e2[2], d[0] * e2[1] - d[1] * e2[0]];
        let a = e1[0] * h[0] + e1[1] * h[1] + e1[2] * h[2];
        if a.abs() < 1e-14 {
            continue;
        }
        let s = [p[0] - c[0][0], p[1] - c[0][1], p[2] - c[0][2]];
        let u = (s[0] * h[0] + s[1] * h[1] + s[2] * h[2]) / a;
        if !(0.0..=1.0).contains(&u) {
            continue;
        }
        let q = [s[1] * e1[2] - s[2] * e1[1], s[2] * e1[0] - s[0] * e1[2], s[0] * e1[1] - s[1] * e1[0]];
        let v = (d[0] * q[0] + d[1] * q[1] + d[2] * q[2]) / a;
        if v < 0.0 || u + v > 1.0 {
            continue;
        }
        let t = (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2]) / a;
        if t > 0.0 {
            hits += 1;
        }
    }
    (hits % 2) as u8
}
