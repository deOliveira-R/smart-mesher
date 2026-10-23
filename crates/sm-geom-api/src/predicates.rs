//! Exact orientation predicates (reference implementation).
//!
//! A floating-point filter decides easy cases; near-degenerate inputs fall back to exact
//! expansion arithmetic (error-free `two_sum` / `two_prod`), so the returned [`Sign`] is always
//! the true sign of the determinant of the input doubles. `sm-math` may supply faster versions
//! behind the same signatures; topology and tolerance decisions only ever consume the sign.

use sm_core::Point3;

/// Sign of an exactly evaluated quantity.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sign {
    /// Strictly negative.
    Negative,
    /// Exactly zero.
    Zero,
    /// Strictly positive.
    Positive,
}

const EPS: f64 = 1.110_223_024_625_156_5e-16; // 2^-53

fn two_sum(a: f64, b: f64) -> (f64, f64) {
    let s = a + b;
    let bb = s - a;
    (s, (a - (s - bb)) + (b - bb))
}

fn two_prod(a: f64, b: f64) -> (f64, f64) {
    let p = a * b;
    (p, a.mul_add(b, -p))
}

/// Add a double to a non-overlapping expansion (Shewchuk's grow-expansion).
fn grow(e: &[f64], b: f64) -> Vec<f64> {
    let mut q = b;
    let mut h = Vec::with_capacity(e.len() + 1);
    for &x in e {
        let (s, err) = two_sum(q, x);
        q = s;
        if err != 0.0 {
            h.push(err);
        }
    }
    h.push(q);
    h
}

fn add(e: &[f64], f: &[f64]) -> Vec<f64> {
    f.iter().fold(e.to_vec(), |acc, &x| grow(&acc, x))
}

fn scale(e: &[f64], b: f64) -> Vec<f64> {
    let mut acc = Vec::new();
    for &c in e {
        let (p, err) = two_prod(c, b);
        acc = grow(&acc, err);
        acc = grow(&acc, p);
    }
    acc
}

fn neg(e: &[f64]) -> Vec<f64> {
    e.iter().map(|x| -x).collect()
}

fn prod2(a: f64, b: f64) -> Vec<f64> {
    let (p, e) = two_prod(a, b);
    vec![e, p]
}

fn sign_of(e: &[f64]) -> Sign {
    match e.iter().rev().find(|&&x| x != 0.0) {
        None => Sign::Zero,
        Some(&x) if x > 0.0 => Sign::Positive,
        Some(_) => Sign::Negative,
    }
}

fn float_sign(x: f64) -> Sign {
    if x > 0.0 {
        Sign::Positive
    } else if x < 0.0 {
        Sign::Negative
    } else {
        Sign::Zero
    }
}

/// Orientation of `c` relative to the directed line `a -> b` in the plane (x, y): positive if
/// `a, b, c` are counter-clockwise, negative if clockwise, zero if collinear. Exact.
pub fn orient2d(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> Sign {
    let l = (a[0] - c[0]) * (b[1] - c[1]);
    let r = (a[1] - c[1]) * (b[0] - c[0]);
    let det = l - r;
    let bound = (3.0 + 16.0 * EPS) * EPS * (l.abs() + r.abs());
    if det > bound || -det > bound {
        return float_sign(det);
    }
    orient2d_exact(a, b, c)
}

/// Exact path of [`orient2d`]: det3 `[[ax ay 1] [bx by 1] [cx cy 1]]` as exact products.
pub(crate) fn orient2d_exact(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> Sign {
    let mut e: Vec<f64> = Vec::new();
    for (x, y, s) in [(a[0], b[1], 1.0), (a[0], c[1], -1.0), (a[1], b[0], -1.0), (a[1], c[0], 1.0), (b[0], c[1], 1.0), (b[1], c[0], -1.0)] {
        let p = prod2(x, y);
        e = add(&e, &if s > 0.0 { p } else { neg(&p) });
    }
    sign_of(&e)
}

fn triple(p: [f64; 3], q: [f64; 3], r: [f64; 3]) -> Vec<f64> {
    let mut e: Vec<f64> = Vec::new();
    for (i, j, k, s) in [(0, 1, 2, 1.0), (0, 2, 1, -1.0), (1, 0, 2, -1.0), (1, 2, 0, 1.0), (2, 0, 1, 1.0), (2, 1, 0, -1.0)] {
        let t = scale(&prod2(q[j], r[k]), p[i]);
        e = add(&e, &if s > 0.0 { t } else { neg(&t) });
    }
    e
}

/// Orientation of `d` relative to the plane through `a, b, c`: positive if `d` lies below the
/// plane when `a, b, c` appear counter-clockwise from above (Shewchuk's convention), i.e.
/// positive for `d` on the side opposite to the right-hand normal `(b-a) x (c-a)`. Exact.
pub fn orient3d(a: Point3, b: Point3, c: Point3, d: Point3) -> Sign {
    let (adx, ady, adz) = (a.x - d.x, a.y - d.y, a.z - d.z);
    let (bdx, bdy, bdz) = (b.x - d.x, b.y - d.y, b.z - d.z);
    let (cdx, cdy, cdz) = (c.x - d.x, c.y - d.y, c.z - d.z);
    let t1 = adx * (bdy * cdz - bdz * cdy);
    let t2 = ady * (bdz * cdx - bdx * cdz);
    let t3 = adz * (bdx * cdy - bdy * cdx);
    let det = t1 + t2 + t3;
    let perm = adx.abs() * ((bdy * cdz).abs() + (bdz * cdy).abs()) + ady.abs() * ((bdz * cdx).abs() + (bdx * cdz).abs()) + adz.abs() * ((bdx * cdy).abs() + (bdy * cdx).abs());
    let bound = (7.0 + 56.0 * EPS) * EPS * perm;
    if det > bound || -det > bound {
        return float_sign(det);
    }
    orient3d_exact(a, b, c, d)
}

/// Exact path of [`orient3d`]: det4 of rows `(x y z 1)`, expanded along the column of ones.
pub(crate) fn orient3d_exact(a: Point3, b: Point3, c: Point3, d: Point3) -> Sign {
    let p = [[a.x, a.y, a.z], [b.x, b.y, b.z], [c.x, c.y, c.z], [d.x, d.y, d.z]];
    let mut e: Vec<f64> = Vec::new();
    for (skip, s) in [(0usize, -1.0), (1, 1.0), (2, -1.0), (3, 1.0)] {
        let rows: Vec<[f64; 3]> = (0..4).filter(|&i| i != skip).map(|i| p[i]).collect();
        let t = triple(rows[0], rows[1], rows[2]);
        e = add(&e, &if s > 0.0 { t } else { neg(&t) });
    }
    sign_of(&e)
}
