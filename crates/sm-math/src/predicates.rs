//! Exact geometric predicates: [`orient2d`], [`orient3d`], [`incircle`], [`insphere`].
//!
//! Each predicate first evaluates the determinant in plain `f64` together with a rigorous
//! forward error bound (Shewchuk's "stage A" filter). When the bound proves the sign, that is
//! the answer. Otherwise stage B re-evaluates the determinant of the rounded differences exactly
//! (accepted when its bound clears or every difference was exact). Only if that is inconclusive
//! is the full polynomial re-evaluated exactly with expansion arithmetic ([`crate::exact`]),
//! which always yields the true sign, zero included. So: A + B + full exact; Shewchuk's stages
//! C/D are not implemented (performance only, never correctness).
//!
//! # Conventions (identical to Shewchuk's `predicates.c`)
//! - [`orient2d`] is positive when `a, b, c` are in counter-clockwise order.
//! - [`orient3d`] is positive when `d` lies below the plane through `a, b, c`, where `a, b, c`
//!   appear counter-clockwise seen from above (i.e. `d` is on the side opposite the normal
//!   `(b - a) x (c - a)`).
//! - [`incircle`] is positive when `d` lies inside the circle through the counter-clockwise
//!   triangle `a, b, c`.
//! - [`insphere`] is positive when `e` lies inside the sphere through `a, b, c, d`, provided
//!   `orient3d(a, b, c, d) > 0`; the sign is reversed otherwise.
//!
//! Equivalently, each is the sign of the determinant of the matrix with rows
//! `[coords..., lift, 1]` (lift = squared norm for the circle/sphere tests) in argument order.
//!
//! # Input range
//! Results are exact when every coordinate is finite and is either `0` or has magnitude in
//! `[2^-100, 2^100]`. That keeps every intermediate product (degree up to 5) free of overflow
//! and underflow. Outside that range the functions may return a wrong sign; the `try_*`
//! variants check the range and return [`PredicateError::OutOfRange`]. Coordinates in
//! practical models (metres, millimetres, normalized units) are far inside it.
//!
//! The plain predicates do not check the range and can silently return a wrong nonzero sign
//! for subnormal inputs. For example `incircle([1.0, 1.75], [1.5, 0.75], [1.0, -0.25],
//! [-0.75, -5e-324])` yields `Negative`, but the points are cocircular at `y = 0` and the true
//! answer is `Positive`. Use the `try_*` variants whenever inputs may be subnormal or huge.
//!
//! # Platform independence
//! No FMA, no extended precision, no rounding-mode changes: only IEEE-754 binary64 `+ - *` in
//! round-to-nearest-even. The filter constants are the standard ones and the exact stage is
//! integer-like, so every platform returns identical signs.

use crate::Sign;
use crate::exact::{Expansion, two_diff};
use sm_core::Point3;

const EPS: f64 = f64::EPSILON * 0.5;
const CCW_ERR_A: f64 = (3.0 + 16.0 * EPS) * EPS;
const O3D_ERR_A: f64 = (7.0 + 56.0 * EPS) * EPS;
const ICC_ERR_A: f64 = (10.0 + 96.0 * EPS) * EPS;
const ISP_ERR_A: f64 = (16.0 + 224.0 * EPS) * EPS;

// Stage-B bounds (Shewchuk): error of the exact determinant of the rounded differences.
const CCW_ERR_B: f64 = (2.0 + 12.0 * EPS) * EPS;
const O3D_ERR_B: f64 = (3.0 + 28.0 * EPS) * EPS;
const ICC_ERR_B: f64 = (4.0 + 48.0 * EPS) * EPS;
const ISP_ERR_B: f64 = (5.0 + 72.0 * EPS) * EPS;

const RANGE_MAX: f64 = f64::from_bits((1023 + 100) << 52);
const RANGE_MIN: f64 = f64::from_bits((1023 - 100) << 52);

/// Error returned by the checked (`try_*`) predicate variants.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum PredicateError {
    /// A coordinate is NaN, infinite, or outside the exactness range (see the module docs).
    #[error("coordinate is non-finite or outside [2^-100, 2^100] in magnitude")]
    OutOfRange,
}

/// True if `x` is within the range for which the predicates are guaranteed exact: zero, or
/// finite with magnitude in `[2^-100, 2^100]`.
#[inline]
pub fn in_exact_range(x: f64) -> bool {
    x == 0.0 || (x.abs() >= RANGE_MIN && x.abs() <= RANGE_MAX)
}

fn check(xs: &[f64]) -> Result<(), PredicateError> {
    if xs.iter().all(|&x| in_exact_range(x)) {
        Ok(())
    } else {
        Err(PredicateError::OutOfRange)
    }
}

#[inline]
fn d(a: f64, b: f64) -> Expansion {
    Expansion::from_diff(a, b)
}

/// Rounded difference `fl(a - b)` and whether it was exact (zero tail).
#[inline]
fn rd(a: f64, b: f64) -> (f64, bool) {
    let (x, y) = two_diff(a, b);
    (x, y == 0.0)
}

/// Float value of an exact expansion whose sign is taken from the top component (as in
/// Shewchuk's `predicates.c`), so correctness does not depend on how the components sum in
/// floating point. The magnitude is the estimate.
fn signed_estimate(e: &Expansion) -> f64 {
    match e.components().last() {
        None => 0.0,
        Some(&top) => {
            let est = e.estimate();
            if est.is_sign_negative() == top.is_sign_negative() && est != 0.0 {
                est
            } else {
                top
            }
        }
    }
}

/// Stage B: accept the sign of the exact determinant `eb` of the *rounded* differences when it
/// clears `bound * permanent`, or when every difference was exact (then `eb` is the answer).
#[inline]
fn stage_b(eb: &Expansion, all_exact: bool, bound: f64, permanent: f64) -> Option<f64> {
    let est = eb.estimate();
    if all_exact || est.abs() >= bound * permanent {
        Some(signed_estimate(eb))
    } else {
        None
    }
}

// ---------------------------------------------------------------------------------------------
// orient2d

/// Determinant `(a-c) x (b-c)` with the exact sign; the magnitude is approximate.
///
/// Positive iff `a, b, c` are counter-clockwise. See the module docs for the input range.
#[inline]
pub fn orient2d_det(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    let left = (a[0] - c[0]) * (b[1] - c[1]);
    let right = (a[1] - c[1]) * (b[0] - c[0]);
    let det = left - right;
    let sum = if left > 0.0 {
        if right <= 0.0 {
            return det;
        }
        left + right
    } else if left < 0.0 {
        if right >= 0.0 {
            return det;
        }
        -left - right
    } else {
        return det;
    };
    if det.abs() >= CCW_ERR_A * sum {
        return det;
    }
    orient2d_adapt(a, b, c, sum)
}

fn orient2d_core(acx: &Expansion, acy: &Expansion, bcx: &Expansion, bcy: &Expansion) -> Expansion {
    acx.mul(bcy).sub(&acy.mul(bcx))
}

#[inline(never)]
fn orient2d_adapt(a: [f64; 2], b: [f64; 2], c: [f64; 2], permanent: f64) -> f64 {
    let (acx, e1) = rd(a[0], c[0]);
    let (acy, e2) = rd(a[1], c[1]);
    let (bcx, e3) = rd(b[0], c[0]);
    let (bcy, e4) = rd(b[1], c[1]);
    let f = Expansion::from_f64;
    let eb = orient2d_core(&f(acx), &f(acy), &f(bcx), &f(bcy));
    if let Some(v) = stage_b(&eb, e1 && e2 && e3 && e4, CCW_ERR_B, permanent) {
        return v;
    }
    let (acx, acy, bcx, bcy) = (d(a[0], c[0]), d(a[1], c[1]), d(b[0], c[0]), d(b[1], c[1]));
    signed_estimate(&orient2d_core(&acx, &acy, &bcx, &bcy))
}

/// Exact orientation of the triangle `a, b, c`: [`Sign::Positive`] if counter-clockwise,
/// [`Sign::Negative`] if clockwise, [`Sign::Zero`] if collinear.
#[inline]
pub fn orient2d(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> Sign {
    Sign::of(orient2d_det(a, b, c))
}

/// [`orient2d`] with an input range check.
pub fn try_orient2d(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> Result<Sign, PredicateError> {
    check(&[a[0], a[1], b[0], b[1], c[0], c[1]])?;
    Ok(orient2d(a, b, c))
}

// ---------------------------------------------------------------------------------------------
// orient3d

/// Determinant of `[a-d; b-d; c-d]` with the exact sign; the magnitude is approximate.
#[inline]
pub fn orient3d_det(a: Point3, b: Point3, c: Point3, d: Point3) -> f64 {
    let (adx, bdx, cdx) = (a.x - d.x, b.x - d.x, c.x - d.x);
    let (ady, bdy, cdy) = (a.y - d.y, b.y - d.y, c.y - d.y);
    let (adz, bdz, cdz) = (a.z - d.z, b.z - d.z, c.z - d.z);
    let bdxcdy = bdx * cdy;
    let cdxbdy = cdx * bdy;
    let cdxady = cdx * ady;
    let adxcdy = adx * cdy;
    let adxbdy = adx * bdy;
    let bdxady = bdx * ady;
    let det = adz * (bdxcdy - cdxbdy) + bdz * (cdxady - adxcdy) + cdz * (adxbdy - bdxady);
    let permanent = (bdxcdy.abs() + cdxbdy.abs()) * adz.abs()
        + (cdxady.abs() + adxcdy.abs()) * bdz.abs()
        + (adxbdy.abs() + bdxady.abs()) * cdz.abs();
    if det.abs() > O3D_ERR_A * permanent {
        return det;
    }
    orient3d_adapt(a, b, c, d, permanent)
}

#[inline(never)]
fn orient3d_adapt(a: Point3, b: Point3, c: Point3, p: Point3, permanent: f64) -> f64 {
    let (adx, e1) = rd(a.x, p.x);
    let (ady, e2) = rd(a.y, p.y);
    let (adz, e3) = rd(a.z, p.z);
    let (bdx, e4) = rd(b.x, p.x);
    let (bdy, e5) = rd(b.y, p.y);
    let (bdz, e6) = rd(b.z, p.z);
    let (cdx, e7) = rd(c.x, p.x);
    let (cdy, e8) = rd(c.y, p.y);
    let (cdz, e9) = rd(c.z, p.z);
    let f = Expansion::from_f64;
    let eb = orient3d_core(
        [&f(adx), &f(ady), &f(adz)],
        [&f(bdx), &f(bdy), &f(bdz)],
        [&f(cdx), &f(cdy), &f(cdz)],
    );
    let all = e1 && e2 && e3 && e4 && e5 && e6 && e7 && e8 && e9;
    if let Some(v) = stage_b(&eb, all, O3D_ERR_B, permanent) {
        return v;
    }
    let (adx, ady, adz) = (d(a.x, p.x), d(a.y, p.y), d(a.z, p.z));
    let (bdx, bdy, bdz) = (d(b.x, p.x), d(b.y, p.y), d(b.z, p.z));
    let (cdx, cdy, cdz) = (d(c.x, p.x), d(c.y, p.y), d(c.z, p.z));
    signed_estimate(&orient3d_core(
        [&adx, &ady, &adz],
        [&bdx, &bdy, &bdz],
        [&cdx, &cdy, &cdz],
    ))
}

fn orient3d_core(a: [&Expansion; 3], b: [&Expansion; 3], c: [&Expansion; 3]) -> Expansion {
    let ([adx, ady, adz], [bdx, bdy, bdz], [cdx, cdy, cdz]) = (a, b, c);
    let t1 = adz.mul(&bdx.mul(cdy).sub(&cdx.mul(bdy)));
    let t2 = bdz.mul(&cdx.mul(ady).sub(&adx.mul(cdy)));
    let t3 = cdz.mul(&adx.mul(bdy).sub(&bdx.mul(ady)));
    t1.add(&t2).add(&t3)
}

/// Exact side of plane `a, b, c` on which `d` lies; see the module docs for the sign convention.
#[inline]
pub fn orient3d(a: Point3, b: Point3, c: Point3, d: Point3) -> Sign {
    Sign::of(orient3d_det(a, b, c, d))
}

/// [`orient3d`] with an input range check.
pub fn try_orient3d(a: Point3, b: Point3, c: Point3, d: Point3) -> Result<Sign, PredicateError> {
    check(&[a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z, d.x, d.y, d.z])?;
    Ok(orient3d(a, b, c, d))
}

// ---------------------------------------------------------------------------------------------
// incircle

/// Lifted determinant for the in-circle test, with the exact sign; magnitude approximate.
#[inline]
pub fn incircle_det(a: [f64; 2], b: [f64; 2], c: [f64; 2], p: [f64; 2]) -> f64 {
    let (adx, ady) = (a[0] - p[0], a[1] - p[1]);
    let (bdx, bdy) = (b[0] - p[0], b[1] - p[1]);
    let (cdx, cdy) = (c[0] - p[0], c[1] - p[1]);
    let bdxcdy = bdx * cdy;
    let cdxbdy = cdx * bdy;
    let alift = adx * adx + ady * ady;
    let cdxady = cdx * ady;
    let adxcdy = adx * cdy;
    let blift = bdx * bdx + bdy * bdy;
    let adxbdy = adx * bdy;
    let bdxady = bdx * ady;
    let clift = cdx * cdx + cdy * cdy;
    let det = alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);
    let permanent = (bdxcdy.abs() + cdxbdy.abs()) * alift
        + (cdxady.abs() + adxcdy.abs()) * blift
        + (adxbdy.abs() + bdxady.abs()) * clift;
    if det.abs() > ICC_ERR_A * permanent {
        return det;
    }
    incircle_adapt(a, b, c, p, permanent)
}

#[inline(never)]
fn incircle_adapt(a: [f64; 2], b: [f64; 2], c: [f64; 2], p: [f64; 2], permanent: f64) -> f64 {
    let (adx, e1) = rd(a[0], p[0]);
    let (ady, e2) = rd(a[1], p[1]);
    let (bdx, e3) = rd(b[0], p[0]);
    let (bdy, e4) = rd(b[1], p[1]);
    let (cdx, e5) = rd(c[0], p[0]);
    let (cdy, e6) = rd(c[1], p[1]);
    let f = Expansion::from_f64;
    let eb = incircle_core(&f(adx), &f(ady), &f(bdx), &f(bdy), &f(cdx), &f(cdy));
    if let Some(v) = stage_b(&eb, e1 && e2 && e3 && e4 && e5 && e6, ICC_ERR_B, permanent) {
        return v;
    }
    let (adx, ady) = (d(a[0], p[0]), d(a[1], p[1]));
    let (bdx, bdy) = (d(b[0], p[0]), d(b[1], p[1]));
    let (cdx, cdy) = (d(c[0], p[0]), d(c[1], p[1]));
    signed_estimate(&incircle_core(&adx, &ady, &bdx, &bdy, &cdx, &cdy))
}

fn incircle_core(
    adx: &Expansion,
    ady: &Expansion,
    bdx: &Expansion,
    bdy: &Expansion,
    cdx: &Expansion,
    cdy: &Expansion,
) -> Expansion {
    let alift = adx.square().add(&ady.square());
    let blift = bdx.square().add(&bdy.square());
    let clift = cdx.square().add(&cdy.square());
    let t1 = alift.mul(&bdx.mul(cdy).sub(&cdx.mul(bdy)));
    let t2 = blift.mul(&cdx.mul(ady).sub(&adx.mul(cdy)));
    let t3 = clift.mul(&adx.mul(bdy).sub(&bdx.mul(ady)));
    t1.add(&t2).add(&t3)
}

/// Exact in-circle test: [`Sign::Positive`] if `d` is strictly inside the circle through the
/// counter-clockwise triangle `a, b, c`, [`Sign::Negative`] outside, [`Sign::Zero`] on it.
#[inline]
pub fn incircle(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> Sign {
    Sign::of(incircle_det(a, b, c, d))
}

/// [`incircle`] with an input range check.
pub fn try_incircle(
    a: [f64; 2],
    b: [f64; 2],
    c: [f64; 2],
    d: [f64; 2],
) -> Result<Sign, PredicateError> {
    check(&[a[0], a[1], b[0], b[1], c[0], c[1], d[0], d[1]])?;
    Ok(incircle(a, b, c, d))
}

// ---------------------------------------------------------------------------------------------
// insphere

/// Lifted determinant for the in-sphere test, with the exact sign; magnitude approximate.
#[inline]
pub fn insphere_det(a: Point3, b: Point3, c: Point3, dd: Point3, e: Point3) -> f64 {
    let (aex, aey, aez) = (a.x - e.x, a.y - e.y, a.z - e.z);
    let (bex, bey, bez) = (b.x - e.x, b.y - e.y, b.z - e.z);
    let (cex, cey, cez) = (c.x - e.x, c.y - e.y, c.z - e.z);
    let (dex, dey, dez) = (dd.x - e.x, dd.y - e.y, dd.z - e.z);

    let aexbey = aex * bey;
    let bexaey = bex * aey;
    let ab = aexbey - bexaey;
    let bexcey = bex * cey;
    let cexbey = cex * bey;
    let bc = bexcey - cexbey;
    let cexdey = cex * dey;
    let dexcey = dex * cey;
    let cd = cexdey - dexcey;
    let dexaey = dex * aey;
    let aexdey = aex * dey;
    let da = dexaey - aexdey;
    let aexcey = aex * cey;
    let cexaey = cex * aey;
    let ac = aexcey - cexaey;
    let bexdey = bex * dey;
    let dexbey = dex * bey;
    let bd = bexdey - dexbey;

    let abc = aez * bc - bez * ac + cez * ab;
    let bcd = bez * cd - cez * bd + dez * bc;
    let cda = cez * da + dez * ac + aez * cd;
    let dab = dez * ab + aez * bd + bez * da;

    let alift = aex * aex + aey * aey + aez * aez;
    let blift = bex * bex + bey * bey + bez * bez;
    let clift = cex * cex + cey * cey + cez * cez;
    let dlift = dex * dex + dey * dey + dez * dez;

    let det = (dlift * abc - clift * dab) + (blift * cda - alift * bcd);

    let (aez_p, bez_p, cez_p, dez_p) = (aez.abs(), bez.abs(), cez.abs(), dez.abs());
    let permanent = ((cexdey.abs() + dexcey.abs()) * bez_p
        + (dexbey.abs() + bexdey.abs()) * cez_p
        + (bexcey.abs() + cexbey.abs()) * dez_p)
        * alift
        + ((dexaey.abs() + aexdey.abs()) * cez_p
            + (aexcey.abs() + cexaey.abs()) * dez_p
            + (cexdey.abs() + dexcey.abs()) * aez_p)
            * blift
        + ((aexbey.abs() + bexaey.abs()) * dez_p
            + (bexdey.abs() + dexbey.abs()) * aez_p
            + (dexaey.abs() + aexdey.abs()) * bez_p)
            * clift
        + ((bexcey.abs() + cexbey.abs()) * aez_p
            + (cexaey.abs() + aexcey.abs()) * bez_p
            + (aexbey.abs() + bexaey.abs()) * cez_p)
            * dlift;
    if det.abs() > ISP_ERR_A * permanent {
        return det;
    }
    insphere_adapt(a, b, c, dd, e, permanent)
}

#[inline(never)]
fn insphere_adapt(a: Point3, b: Point3, c: Point3, dd: Point3, e: Point3, permanent: f64) -> f64 {
    let pts = [a, b, c, dd];
    let mut r = [0.0f64; 12];
    let mut all = true;
    for (i, q) in pts.iter().enumerate() {
        for (k, (u, v)) in [(q.x, e.x), (q.y, e.y), (q.z, e.z)].into_iter().enumerate() {
            let (x, ex) = rd(u, v);
            r[3 * i + k] = x;
            all &= ex;
        }
    }
    let f = Expansion::from_f64;
    let fe: [Expansion; 12] = core::array::from_fn(|i| f(r[i]));
    let eb = insphere_core(&fe);
    if let Some(v) = stage_b(&eb, all, ISP_ERR_B, permanent) {
        return v;
    }
    let mut full: [Expansion; 12] = core::array::from_fn(|_| Expansion::zero());
    for (i, q) in pts.iter().enumerate() {
        full[3 * i] = d(q.x, e.x);
        full[3 * i + 1] = d(q.y, e.y);
        full[3 * i + 2] = d(q.z, e.z);
    }
    signed_estimate(&insphere_core(&full))
}

/// Exact insphere determinant from the 12 differences `[a-e, b-e, c-e, d-e]` (x, y, z each).
fn insphere_core(v: &[Expansion; 12]) -> Expansion {
    let (aex, aey, aez) = (&v[0], &v[1], &v[2]);
    let (bex, bey, bez) = (&v[3], &v[4], &v[5]);
    let (cex, cey, cez) = (&v[6], &v[7], &v[8]);
    let (dex, dey, dez) = (&v[9], &v[10], &v[11]);
    let ab = aex.mul(bey).sub(&bex.mul(aey));
    let bc = bex.mul(cey).sub(&cex.mul(bey));
    let cd = cex.mul(dey).sub(&dex.mul(cey));
    let da = dex.mul(aey).sub(&aex.mul(dey));
    let ac = aex.mul(cey).sub(&cex.mul(aey));
    let bd = bex.mul(dey).sub(&dex.mul(bey));
    let abc = aez.mul(&bc).sub(&bez.mul(&ac)).add(&cez.mul(&ab));
    let bcd = bez.mul(&cd).sub(&cez.mul(&bd)).add(&dez.mul(&bc));
    let cda = cez.mul(&da).add(&dez.mul(&ac)).add(&aez.mul(&cd));
    let dab = dez.mul(&ab).add(&aez.mul(&bd)).add(&bez.mul(&da));
    let lift =
        |x: &Expansion, y: &Expansion, z: &Expansion| x.square().add(&y.square()).add(&z.square());
    let (alift, blift, clift, dlift) = (
        lift(aex, aey, aez),
        lift(bex, bey, bez),
        lift(cex, cey, cez),
        lift(dex, dey, dez),
    );
    let left = dlift.mul(&abc).sub(&clift.mul(&dab));
    let right = blift.mul(&cda).sub(&alift.mul(&bcd));
    left.add(&right)
}

/// Exact in-sphere test: [`Sign::Positive`] if `e` is strictly inside the sphere through
/// `a, b, c, d` (given `orient3d(a, b, c, d) > 0`), [`Sign::Negative`] outside, [`Sign::Zero`]
/// on the sphere. The sign flips if the tetrahedron `a, b, c, d` is negatively oriented.
#[inline]
pub fn insphere(a: Point3, b: Point3, c: Point3, d: Point3, e: Point3) -> Sign {
    Sign::of(insphere_det(a, b, c, d, e))
}

/// [`insphere`] with an input range check.
pub fn try_insphere(
    a: Point3,
    b: Point3,
    c: Point3,
    d: Point3,
    e: Point3,
) -> Result<Sign, PredicateError> {
    check(&[
        a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z, d.x, d.y, d.z, e.x, e.y, e.z,
    ])?;
    Ok(insphere(a, b, c, d, e))
}
