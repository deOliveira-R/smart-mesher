//! Small dense linear algebra for geometry kernels and quality metrics.
//!
//! Matrices are row-major `[[f64; N]; N]` ([`Mat3`], [`Mat4`]), the same layout as
//! [`sm_core::Transform::m`]. Vectors are [`sm_core::Vector3`] or `[f64; N]`.
//!
//! - [`solve3`], [`solve4`], [`inverse3`], [`det3`], [`det4`]: Gaussian elimination with
//!   partial pivoting and a relative singularity test.
//! - [`sym3_eigen`]: symmetric 3x3 eigen decomposition by cyclic Jacobi. Unlike the closed-form
//!   (Cardano) solution it stays accurate for repeated and nearly repeated eigenvalues, which
//!   is exactly what moment and quality computations on regular shapes produce.
//! - [`svd3`]: singular value decomposition by one-sided Jacobi (Hestenes), which keeps small
//!   singular values accurate and never forms `AᵀA`.
//!
//! All routines use a fixed sweep and pair order, so results are deterministic.

use sm_core::Vector3;

/// Row-major 3x3 matrix: `m[row][col]`.
pub type Mat3 = [[f64; 3]; 3];
/// Row-major 4x4 matrix: `m[row][col]`.
pub type Mat4 = [[f64; 4]; 4];

/// The 3x3 identity.
pub const IDENTITY3: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// Failure of a dense linear-algebra routine.
#[derive(Clone, Copy, Debug, PartialEq, thiserror::Error)]
pub enum LinAlgError {
    /// The matrix is singular to working precision: elimination found a pivot no larger than
    /// `16 * EPSILON * max|a_ij|`.
    #[error("matrix is singular to working precision (pivot {pivot} at step {step})")]
    Singular {
        /// Elimination step at which it failed.
        step: usize,
        /// The offending pivot magnitude.
        pivot: f64,
    },
    /// An entry was NaN or infinite.
    #[error("matrix contains a non-finite entry")]
    NonFinite,
    /// An iterative method failed to converge (not expected for finite input).
    #[error("iteration did not converge")]
    NoConvergence,
}

/// Matrix product `a * b`.
pub fn mat3_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut c = [[0.0; 3]; 3];
    for (i, row) in c.iter_mut().enumerate() {
        for (j, e) in row.iter_mut().enumerate() {
            *e = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    c
}

/// Transpose.
pub fn mat3_transpose(a: &Mat3) -> Mat3 {
    [
        [a[0][0], a[1][0], a[2][0]],
        [a[0][1], a[1][1], a[2][1]],
        [a[0][2], a[1][2], a[2][2]],
    ]
}

/// Matrix-vector product `a * v`.
pub fn mat3_mul_vec(a: &Mat3, v: Vector3) -> Vector3 {
    Vector3::new(
        a[0][0] * v.x + a[0][1] * v.y + a[0][2] * v.z,
        a[1][0] * v.x + a[1][1] * v.y + a[1][2] * v.z,
        a[2][0] * v.x + a[2][1] * v.y + a[2][2] * v.z,
    )
}

/// Column `k` of `a` as a vector.
pub fn mat3_column(a: &Mat3, k: usize) -> Vector3 {
    Vector3::new(a[0][k], a[1][k], a[2][k])
}

/// Frobenius norm.
pub fn mat3_frobenius(a: &Mat3) -> f64 {
    a.iter().flatten().map(|x| x * x).sum::<f64>().sqrt()
}

/// Determinant of a 3x3 matrix (plain floating point, not exact).
pub fn det3(m: &Mat3) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

/// Determinant of a 4x4 matrix by cofactor expansion on 2x2 minors (not exact).
pub fn det4(m: &Mat4) -> f64 {
    let s0 = m[0][0] * m[1][1] - m[1][0] * m[0][1];
    let s1 = m[0][0] * m[1][2] - m[1][0] * m[0][2];
    let s2 = m[0][0] * m[1][3] - m[1][0] * m[0][3];
    let s3 = m[0][1] * m[1][2] - m[1][1] * m[0][2];
    let s4 = m[0][1] * m[1][3] - m[1][1] * m[0][3];
    let s5 = m[0][2] * m[1][3] - m[1][2] * m[0][3];
    let c5 = m[2][2] * m[3][3] - m[3][2] * m[2][3];
    let c4 = m[2][1] * m[3][3] - m[3][1] * m[2][3];
    let c3 = m[2][1] * m[3][2] - m[3][1] * m[2][2];
    let c2 = m[2][0] * m[3][3] - m[3][0] * m[2][3];
    let c1 = m[2][0] * m[3][2] - m[3][0] * m[2][2];
    let c0 = m[2][0] * m[3][1] - m[3][0] * m[2][1];
    s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0
}

/// Solve `A x = b` by Gaussian elimination with partial pivoting.
fn solve_n<const N: usize>(a: &[[f64; N]; N], b: &[f64; N]) -> Result<[f64; N], LinAlgError> {
    let mut m = *a;
    let mut x = *b;
    let mut scale = 0.0f64;
    for v in m.iter().flatten().chain(x.iter()) {
        if !v.is_finite() {
            return Err(LinAlgError::NonFinite);
        }
    }
    for v in m.iter().flatten() {
        scale = scale.max(v.abs());
    }
    let tol = 16.0 * f64::EPSILON * scale;
    for k in 0..N {
        let mut piv = k;
        for i in k + 1..N {
            if m[i][k].abs() > m[piv][k].abs() {
                piv = i;
            }
        }
        let pivot = m[piv][k].abs();
        if pivot <= tol || scale == 0.0 {
            return Err(LinAlgError::Singular { step: k, pivot });
        }
        m.swap(k, piv);
        x.swap(k, piv);
        let pivot_row = m[k];
        for i in k + 1..N {
            let f = m[i][k] / pivot_row[k];
            if f != 0.0 {
                for (e, p) in m[i][k..].iter_mut().zip(&pivot_row[k..]) {
                    *e -= f * p;
                }
                x[i] -= f * x[k];
            }
        }
    }
    for k in (0..N).rev() {
        let mut s = x[k];
        for j in k + 1..N {
            s -= m[k][j] * x[j];
        }
        x[k] = s / m[k][k];
    }
    Ok(x)
}

/// Solve the 3x3 system `a x = b`.
///
/// # Errors
/// [`LinAlgError::Singular`] if a pivot is below `16 * EPSILON * max|a_ij|`;
/// [`LinAlgError::NonFinite`] for NaN/inf input.
pub fn solve3(a: &Mat3, b: Vector3) -> Result<Vector3, LinAlgError> {
    let x = solve_n(a, &[b.x, b.y, b.z])?;
    Ok(Vector3::new(x[0], x[1], x[2]))
}

/// Solve the 4x4 system `a x = b`. Errors as for [`solve3`].
pub fn solve4(a: &Mat4, b: [f64; 4]) -> Result<[f64; 4], LinAlgError> {
    solve_n(a, &b)
}

/// Inverse of a 3x3 matrix. Errors as for [`solve3`].
pub fn inverse3(a: &Mat3) -> Result<Mat3, LinAlgError> {
    let mut inv = [[0.0; 3]; 3];
    for k in 0..3 {
        let mut e = [0.0; 3];
        e[k] = 1.0;
        let col = solve_n(a, &e)?;
        for (row, v) in inv.iter_mut().zip(col) {
            row[k] = v;
        }
    }
    Ok(inv)
}

/// Eigen decomposition `A = V diag(values) Vᵀ` of a symmetric 3x3 matrix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SymEigen3 {
    /// Eigenvalues in ascending order.
    pub values: [f64; 3],
    /// Orthogonal matrix whose **columns** are the unit eigenvectors, in the same order as
    /// `values`. Each column is sign-normalised so its largest-magnitude component is positive
    /// (lowest index wins ties), making the output canonical.
    pub vectors: Mat3,
}

impl SymEigen3 {
    /// Unit eigenvector `k` (column `k` of [`SymEigen3::vectors`]).
    pub fn vector(&self, k: usize) -> Vector3 {
        mat3_column(&self.vectors, k)
    }
}

fn canonical_sign(c: [f64; 3]) -> f64 {
    let mut big = 0;
    for i in 1..3 {
        if c[i].abs() > c[big].abs() {
            big = i;
        }
    }
    if c[big] < 0.0 { -1.0 } else { 1.0 }
}

/// Symmetric 3x3 eigen decomposition by cyclic Jacobi rotations.
///
/// Only the average `(a_ij + a_ji) / 2` is used, so slightly asymmetric input is symmetrised.
/// Eigenvalues are ascending; eigenvectors are orthonormal and sign-canonical (see
/// [`SymEigen3`]). Absolute eigenvalue error is `O(EPSILON * ‖A‖)`.
///
/// # Errors
/// [`LinAlgError::NonFinite`] for NaN/inf input.
pub fn sym3_eigen(a: &Mat3) -> Result<SymEigen3, LinAlgError> {
    if a.iter().flatten().any(|v| !v.is_finite()) {
        return Err(LinAlgError::NonFinite);
    }
    let mut m = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] = 0.5 * (a[i][j] + a[j][i]);
        }
    }
    let mut v = IDENTITY3;
    let fro2: f64 = m.iter().flatten().map(|x| x * x).sum();
    for _ in 0..64 {
        let off2 = 2.0 * (m[0][1] * m[0][1] + m[0][2] * m[0][2] + m[1][2] * m[1][2]);
        if off2 <= 1e-34 * fro2 || off2 == 0.0 {
            break;
        }
        for (p, q) in [(0usize, 1usize), (0, 2), (1, 2)] {
            let apq = m[p][q];
            if apq == 0.0 {
                continue;
            }
            let theta = (m[q][q] - m[p][p]) / (2.0 * apq);
            let t = if theta.abs() > 1e150 {
                0.5 / theta
            } else {
                theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt())
            };
            // `signum` of +0.0 is 1.0, which is what we want when theta == 0.
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            let (app, aqq) = (m[p][p], m[q][q]);
            m[p][p] = app - t * apq;
            m[q][q] = aqq + t * apq;
            m[p][q] = 0.0;
            m[q][p] = 0.0;
            let r = 3 - p - q;
            let (arp, arq) = (m[r][p], m[r][q]);
            m[r][p] = c * arp - s * arq;
            m[p][r] = m[r][p];
            m[r][q] = s * arp + c * arq;
            m[q][r] = m[r][q];
            for row in v.iter_mut() {
                let (vp, vq) = (row[p], row[q]);
                row[p] = c * vp - s * vq;
                row[q] = s * vp + c * vq;
            }
        }
    }
    let mut idx = [0usize, 1, 2];
    let vals = [m[0][0], m[1][1], m[2][2]];
    idx.sort_by(|&i, &j| vals[i].total_cmp(&vals[j]));
    let mut out = SymEigen3 {
        values: [vals[idx[0]], vals[idx[1]], vals[idx[2]]],
        vectors: [[0.0; 3]; 3],
    };
    for (k, &src) in idx.iter().enumerate() {
        let col = [v[0][src], v[1][src], v[2][src]];
        let sg = canonical_sign(col);
        for (row, c) in out.vectors.iter_mut().zip(col) {
            row[k] = sg * c;
        }
    }
    Ok(out)
}

/// Singular value decomposition `A = U diag(s) Vᵀ` of a 3x3 matrix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Svd3 {
    /// Orthogonal matrix; columns are the left singular vectors.
    pub u: Mat3,
    /// Singular values, descending, all `>= 0`.
    pub s: [f64; 3],
    /// Orthogonal matrix; columns are the right singular vectors.
    pub v: Mat3,
}

impl Svd3 {
    /// `s_max / s_min`, or `+inf` for a singular matrix.
    pub fn condition(&self) -> f64 {
        if self.s[2] == 0.0 {
            f64::INFINITY
        } else {
            self.s[0] / self.s[2]
        }
    }

    /// Rebuild `U diag(s) Vᵀ`.
    pub fn reconstruct(&self) -> Mat3 {
        let mut us = self.u;
        for row in us.iter_mut() {
            for (k, e) in row.iter_mut().enumerate() {
                *e *= self.s[k];
            }
        }
        mat3_mul(&us, &mat3_transpose(&self.v))
    }
}

fn cross(a: Vector3, b: Vector3) -> Vector3 {
    a.cross(b)
}

fn unit(v: Vector3) -> Vector3 {
    let n = v.norm();
    Vector3::new(v.x / n, v.y / n, v.z / n)
}

/// SVD of a 3x3 matrix by one-sided Jacobi.
///
/// For a rank-deficient matrix the left singular vectors belonging to zero singular values are
/// completed to an orthonormal basis, so `U` is always orthogonal. `U` and `V` may have
/// determinant `-1` (they are orthogonal, not necessarily rotations).
///
/// # Errors
/// [`LinAlgError::NonFinite`] for NaN/inf input, [`LinAlgError::NoConvergence`] if 60 sweeps
/// do not suffice (not expected).
pub fn svd3(a: &Mat3) -> Result<Svd3, LinAlgError> {
    if a.iter().flatten().any(|v| !v.is_finite()) {
        return Err(LinAlgError::NonFinite);
    }
    // Columns of `w` start as the columns of A and are rotated until mutually orthogonal.
    let mut w = *a;
    let mut v = IDENTITY3;
    let mut converged = false;
    for _ in 0..60 {
        let mut rotated = false;
        for (p, q) in [(0usize, 1usize), (0, 2), (1, 2)] {
            let (mut alpha, mut beta, mut gamma) = (0.0, 0.0, 0.0);
            for row in &w {
                alpha += row[p] * row[p];
                beta += row[q] * row[q];
                gamma += row[p] * row[q];
            }
            if gamma == 0.0 || gamma.abs() <= f64::EPSILON * (alpha * beta).sqrt() {
                continue;
            }
            rotated = true;
            let zeta = (beta - alpha) / (2.0 * gamma);
            let t = if zeta.abs() > 1e150 {
                0.5 / zeta
            } else {
                zeta.signum() / (zeta.abs() + (1.0 + zeta * zeta).sqrt())
            };
            let c = 1.0 / (1.0 + t * t).sqrt();
            let s = c * t;
            for m in [&mut w, &mut v] {
                for row in m.iter_mut() {
                    let (xp, xq) = (row[p], row[q]);
                    row[p] = c * xp - s * xq;
                    row[q] = s * xp + c * xq;
                }
            }
        }
        if !rotated {
            converged = true;
            break;
        }
    }
    if !converged {
        return Err(LinAlgError::NoConvergence);
    }
    let sig: [f64; 3] = core::array::from_fn(|k| {
        (w[0][k] * w[0][k] + w[1][k] * w[1][k] + w[2][k] * w[2][k]).sqrt()
    });
    let mut idx = [0usize, 1, 2];
    idx.sort_by(|&i, &j| sig[j].total_cmp(&sig[i]));
    let smax = sig[idx[0]];
    let live = |k: usize| sig[idx[k]] > smax * f64::MIN_POSITIVE.sqrt() && sig[idx[k]] > 0.0;
    let mut cols: [Option<Vector3>; 3] = [None; 3];
    for (k, col) in cols.iter_mut().enumerate() {
        if live(k) {
            let s = sig[idx[k]];
            *col = Some(Vector3::new(
                w[0][idx[k]] / s,
                w[1][idx[k]] / s,
                w[2][idx[k]] / s,
            ));
        }
    }
    // Complete missing left vectors (only trailing ones can be missing).
    let rank = cols.iter().take_while(|c| c.is_some()).count();
    match rank {
        0 => {
            cols = [
                Some(Vector3::new(1.0, 0.0, 0.0)),
                Some(Vector3::new(0.0, 1.0, 0.0)),
                Some(Vector3::new(0.0, 0.0, 1.0)),
            ]
        }
        1 => {
            let u1 = cols[0].unwrap_or_default();
            let axes = [
                Vector3::new(1.0, 0.0, 0.0),
                Vector3::new(0.0, 1.0, 0.0),
                Vector3::new(0.0, 0.0, 1.0),
            ];
            let mut best = 0;
            for i in 1..3 {
                if [u1.x, u1.y, u1.z][i].abs() < [u1.x, u1.y, u1.z][best].abs() {
                    best = i;
                }
            }
            let u2 = unit(cross(u1, axes[best]));
            cols[1] = Some(u2);
            cols[2] = Some(unit(cross(u1, u2)));
        }
        2 => {
            let (u1, u2) = (cols[0].unwrap_or_default(), cols[1].unwrap_or_default());
            cols[2] = Some(unit(cross(u1, u2)));
        }
        _ => {}
    }
    let mut out = Svd3 {
        u: [[0.0; 3]; 3],
        s: [sig[idx[0]], sig[idx[1]], sig[idx[2]]],
        v: [[0.0; 3]; 3],
    };
    for k in 0..3 {
        let c = cols[k].unwrap_or_default();
        let (cx, cy, cz) = (c.x, c.y, c.z);
        out.u[0][k] = cx;
        out.u[1][k] = cy;
        out.u[2][k] = cz;
        for (out_row, v_row) in out.v.iter_mut().zip(&v) {
            out_row[k] = v_row[idx[k]];
        }
    }
    Ok(out)
}
