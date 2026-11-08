//! `translation` and `rotation` of a filled cell.
//!
//! OpenMC maps a point `r` of the cell's frame to the filling universe's frame as
//! `u = M (r - t)`: the translation `t` is subtracted first, then the rotation matrix `M` is
//! applied. The IR wants the inverse (universe point to cell frame), `r = M^T u + t`.
//! `M` comes from three angles `(phi, theta, psi)` in degrees, or from nine row-major entries.

use super::{Ctx, invalid, numbers, unsupported};
use crate::ImportError;
use sm_core::{Transform, Vector3};

/// OpenMC's matrix for the angles (degrees): the product `Rz(-psi) Ry(-theta) Rx(-phi)`.
fn from_angles(a: [f64; 3]) -> [[f64; 3]; 3] {
    let (p, t, s) = (-a[0].to_radians(), -a[1].to_radians(), -a[2].to_radians());
    let (sp, cp, st, ct, ss, cs) = (p.sin(), p.cos(), t.sin(), t.cos(), s.sin(), s.cos());
    [
        [ct * cs, -cp * ss + sp * st * cs, sp * ss + cp * st * cs],
        [ct * ss, cp * cs + sp * st * ss, -sp * cs + cp * st * ss],
        [-st, sp * ct, cp * ct],
    ]
}

/// The IR transform of a fill with the given `translation` and `rotation` field texts.
pub(super) fn fill_transform(cx: &Ctx, translation: Option<&str>, rotation: Option<&str>) -> Result<Transform, ImportError> {
    let t = match translation {
        None => [0.0; 3],
        Some(s) => {
            let v = numbers(cx, "translation", s)?;
            <[f64; 3]>::try_from(v).map_err(|v| invalid(cx, format!("translation needs 3 values, found {}", v.len())))?
        }
    };
    let m = match rotation {
        None => [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        Some(s) => {
            let v = numbers(cx, "rotation", s)?;
            match v.len() {
                3 => from_angles([v[0], v[1], v[2]]),
                9 => {
                    let m = [[v[0], v[1], v[2]], [v[3], v[4], v[5]], [v[6], v[7], v[8]]];
                    check_rotation(cx, &m)?;
                    m
                }
                n => return Err(invalid(cx, format!("rotation needs 3 angles or 9 matrix entries, found {n}"))),
            }
        }
    };
    // r = M^T u + t
    let mt = [[m[0][0], m[1][0], m[2][0]], [m[0][1], m[1][1], m[2][1]], [m[0][2], m[1][2], m[2][2]]];
    Ok(Transform { m: mt, t: Vector3::new(t[0], t[1], t[2]) })
}

/// OpenMC does not check its nine entries; a matrix that is not a proper rotation would shear or
/// mirror space, which the IR (rigid placements) does not model.
fn check_rotation(cx: &Ctx, m: &[[f64; 3]; 3]) -> Result<(), ImportError> {
    for i in 0..3 {
        for j in 0..3 {
            let d: f64 = (0..3).map(|k| m[i][k] * m[j][k]).sum();
            if (d - f64::from(u8::from(i == j))).abs() > 1e-9 {
                return Err(unsupported(cx, "rotation matrix is not orthonormal".into()));
            }
        }
    }
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if det < 0.0 {
        return Err(unsupported(cx, "rotation matrix is a reflection (determinant -1)".into()));
    }
    Ok(())
}
