//! Least-squares affine fits: the layer placement map and the sweep-target copy transform.

use crate::error::SweepError;
use sm_core::{Point3, Transform, Vector3};
use sm_math::linalg::{Mat3, sym3_eigen};

/// Eigenvalues of the moment matrix below this fraction of the largest are treated as null.
const RANK_TOL: f64 = 1e-10;

fn sub(a: Point3, b: Point3) -> Vector3 {
    a - b
}

fn centroid(p: &[Point3]) -> Point3 {
    let mut s = Vector3::default();
    for q in p {
        s = s + Vector3::new(q.x, q.y, q.z);
    }
    let k = 1.0 / p.len() as f64;
    Point3::new(s.x * k, s.y * k, s.z * k)
}

/// Least-squares affine map `x -> M x + t` taking `src[i]` to `dst[i]`.
///
/// Centred normal equations `M = C Bᵀ (B Bᵀ)⁺` with `B = [b_i - b̄]`, `C = [c_i - c̄]` and
/// `t = c̄ - M b̄`; sums run in input order, so a caller that sorts its points by `NodeKey` gets
/// a reproducible result. The pseudo-inverse comes from the symmetric eigen decomposition of
/// `B Bᵀ`, dropping eigenvalues below `1e-10` of the largest. A planar point set therefore has
/// a null normal direction `n`; the map is completed there with the identity (`M n = n`), so it
/// is the exact in-plane map and stays invertible with `det > 0` for orientation-preserving
/// data.
///
/// # Errors
/// [`SweepError::TooFewPoints`] for fewer than 4 points, [`SweepError::LengthMismatch`],
/// [`SweepError::NonFinite`], and [`SweepError::Degenerate`] when the source points are
/// collinear or coincident (moment-matrix rank below 2).
pub fn fit_affine(src: &[Point3], dst: &[Point3]) -> Result<Transform, SweepError> {
    if src.len() != dst.len() {
        return Err(SweepError::LengthMismatch {
            src: src.len(),
            dst: dst.len(),
        });
    }
    if src.len() < 4 {
        return Err(SweepError::TooFewPoints {
            have: src.len(),
            need: 4,
        });
    }
    fit_affine_unchecked(src, dst)
}

/// [`fit_affine`] without the point-count floor (rank must still be at least 2).
pub(crate) fn fit_affine_unchecked(
    src: &[Point3],
    dst: &[Point3],
) -> Result<Transform, SweepError> {
    let finite = |p: &Point3| p.x.is_finite() && p.y.is_finite() && p.z.is_finite();
    if !src.iter().all(finite) || !dst.iter().all(finite) {
        return Err(SweepError::NonFinite);
    }
    let (bb, cb) = (centroid(src), centroid(dst));
    let mut bmat: Mat3 = [[0.0; 3]; 3];
    let mut cmat: Mat3 = [[0.0; 3]; 3];
    for (b, c) in src.iter().zip(dst) {
        let (u, v) = (sub(*b, bb), sub(*c, cb));
        let (u, v) = ([u.x, u.y, u.z], [v.x, v.y, v.z]);
        for i in 0..3 {
            for j in 0..3 {
                bmat[i][j] += u[i] * u[j];
                cmat[i][j] += v[i] * u[j];
            }
        }
    }
    let eig = sym3_eigen(&bmat)?;
    let top = eig.values[2];
    if top.is_nan() || top <= 0.0 {
        return Err(SweepError::Degenerate { rank: 0 });
    }
    let kept = eig.values.iter().filter(|v| **v >= RANK_TOL * top).count();
    if kept < 2 {
        return Err(SweepError::Degenerate { rank: kept });
    }
    // pinv = sum over kept 1/lambda v v^T; completion = sum over dropped v v^T.
    let mut pinv: Mat3 = [[0.0; 3]; 3];
    let mut complete: Mat3 = [[0.0; 3]; 3];
    for k in 0..3 {
        let v = eig.vector(k);
        let v = [v.x, v.y, v.z];
        let keep = eig.values[k] >= RANK_TOL * top;
        let w = if keep { 1.0 / eig.values[k] } else { 0.0 };
        for i in 0..3 {
            for j in 0..3 {
                if keep {
                    pinv[i][j] += w * v[i] * v[j];
                } else {
                    complete[i][j] += v[i] * v[j];
                }
            }
        }
    }
    let mut m: Mat3 = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] = (0..3).map(|k| cmat[i][k] * pinv[k][j]).sum::<f64>() + complete[i][j];
        }
    }
    let lin = Transform {
        m,
        t: Vector3::default(),
    };
    let mb = lin.apply_vector(Vector3::new(bb.x, bb.y, bb.z));
    Ok(Transform {
        m,
        t: Vector3::new(cb.x - mb.x, cb.y - mb.y, cb.z - mb.z),
    })
}

fn unit(v: Vector3) -> Option<Vector3> {
    let l = v.norm();
    (l.is_finite() && l > 0.0).then(|| v * (1.0 / l))
}

/// Maximum residual `|T(src_i) - dst_i|` of `t` over the pairs.
pub(crate) fn max_residual(t: &Transform, src: &[Point3], dst: &[Point3]) -> f64 {
    src.iter()
        .zip(dst)
        .map(|(s, d)| (t.apply_point(*s) - *d).norm())
        .fold(0.0, f64::max)
}

/// The affine map that carries the source face's vertices onto the target face's, or `None`
/// if the correspondence is not affine within `tol`.
///
/// `src[i]` corresponds to `dst[i]`; `n_src` and `n_dst` are the **outward** normals of the
/// swept solid on the source and the target face. A pure translation (all `dst - src` equal
/// within `tol`) is returned as an exact [`Transform::translation`] of their mean. Otherwise
/// both vertex sets are coplanar, so one virtual pair `c_s + l n_s -> c_t - l n_t` is added
/// (`c` the centroids, `l` the mean vertex distance from `c_s`); the fit is then full rank, and
/// it encodes that the map takes the outward source normal to the inward target normal (a
/// rotation by `phi` about `z` comes out as `Rz(phi)`). The map is accepted only if the largest
/// vertex residual is at most `tol` and its determinant is positive.
pub fn fit_target_transform(
    src: &[Point3],
    dst: &[Point3],
    n_src: Vector3,
    n_dst: Vector3,
    tol: f64,
) -> Option<Transform> {
    if src.is_empty() || src.len() != dst.len() || !(tol >= 0.0 && tol.is_finite()) {
        return None;
    }
    let n = src.len() as f64;
    // All differences bit-identical: use the first one, so a pure translation is exact.
    let d0 = dst[0] - src[0];
    let d = if src.iter().zip(dst).all(|(s, t)| *t - *s == d0) {
        d0
    } else {
        let mut d = Vector3::default();
        for (s, t) in src.iter().zip(dst) {
            d = d + (*t - *s);
        }
        d * (1.0 / n)
    };
    let translation = Transform::translation(d);
    if max_residual(&translation, src, dst) <= tol {
        return Some(translation);
    }
    let (cs, ct) = (centroid(src), centroid(dst));
    let l = src.iter().map(|p| (*p - cs).norm()).sum::<f64>() / n;
    let (ns, nd) = (unit(n_src)?, unit(n_dst)?);
    if l.is_nan() || l <= 0.0 {
        return None;
    }
    let mut a: Vec<Point3> = src.to_vec();
    let mut b: Vec<Point3> = dst.to_vec();
    a.push(cs + ns * l);
    b.push(ct + (-nd) * l);
    let t = fit_affine_unchecked(&a, &b).ok()?;
    (t.det() > 0.0 && max_residual(&t, src, dst) <= tol).then_some(t)
}

/// Best-effort transform for `target_mode: "copy"` when the strict fit fails: the same
/// augmented least-squares map without the acceptance tests, else the translation of the
/// centroids. Returns the transform and its largest vertex residual.
pub(crate) fn best_effort_transform(
    src: &[Point3],
    dst: &[Point3],
    n_src: Vector3,
    n_dst: Vector3,
) -> Option<(Transform, f64)> {
    if src.is_empty() || src.len() != dst.len() {
        return None;
    }
    let (cs, ct) = (centroid(src), centroid(dst));
    let n = src.len() as f64;
    let l = src.iter().map(|p| (*p - cs).norm()).sum::<f64>() / n;
    let fitted = (|| {
        let (ns, nd) = (unit(n_src)?, unit(n_dst)?);
        if l.is_nan() || l <= 0.0 {
            return None;
        }
        let mut a: Vec<Point3> = src.to_vec();
        let mut b: Vec<Point3> = dst.to_vec();
        a.push(cs + ns * l);
        b.push(ct + (-nd) * l);
        fit_affine_unchecked(&a, &b).ok().filter(|t| t.det() > 0.0)
    })();
    let t = fitted.unwrap_or_else(|| Transform::translation(ct - cs));
    let r = max_residual(&t, src, dst);
    Some((t, r))
}
