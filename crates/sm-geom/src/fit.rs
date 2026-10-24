//! Interpolating NURBS through points (Piegl and Tiller A9.1): chord-length parameters, knot
//! averaging and a banded solve. Used to turn marched intersection polylines into curves.

use sm_core::Point3;
use sm_geom_api::{GeomError, NurbsCurve};

/// Solve the banded system with half-bandwidth `bw` (`a[i][j - i + bw]`) by Gaussian
/// elimination without pivoting (the collocation matrix of a B-spline basis is totally
/// positive, so this is stable) for three right-hand sides.
fn banded_solve(a: &mut [[f64; 7]], bw: usize, rhs: &mut [[f64; 3]]) -> Result<(), GeomError> {
    let n = a.len();
    for i in 0..n {
        let piv = a[i][bw];
        if piv.abs() < 1e-300 {
            return Err(GeomError::Singular("interpolation matrix"));
        }
        for r in i + 1..(i + bw + 1).min(n) {
            let f = a[r][bw + i - r] / piv;
            if f == 0.0 {
                continue;
            }
            for c in i..(i + bw + 1).min(n) {
                let (src, dst) = (c + bw - i, c + bw - r);
                let v = a[i][src];
                a[r][dst] -= f * v;
            }
            for k in 0..3 {
                let v = rhs[i][k];
                rhs[r][k] -= f * v;
            }
        }
    }
    for i in (0..n).rev() {
        for k in 0..3 {
            let mut s = rhs[i][k];
            for c in i + 1..(i + bw + 1).min(n) {
                s -= a[i][c + bw - i] * rhs[c][k];
            }
            rhs[i][k] = s / a[i][bw];
        }
    }
    Ok(())
}

/// Interpolate `points` (at least two, no repeated consecutive points) by a clamped NURBS of
/// degree `min(degree, n - 1)` (polynomial), parametrized by chord length over `[0, 1]`. The
/// curve passes through every point; between points it is the C2 cubic (degree 3) interpolant.
/// A closed loop (first point equal to last) is a clamped closed curve, C0 at the seam.
pub fn interpolate_points(points: &[Point3], degree: usize) -> Result<NurbsCurve, GeomError> {
    let n = points.len();
    if n < 2 {
        return Err(GeomError::Invalid("interpolation needs at least two points"));
    }
    let p = degree.clamp(1, 3).min(n - 1);
    let mut u = vec![0.0; n];
    for i in 1..n {
        let d = (points[i] - points[i - 1]).norm();
        if d <= 0.0 {
            return Err(GeomError::Invalid("consecutive interpolation points coincide"));
        }
        u[i] = u[i - 1] + d;
    }
    let total = u[n - 1];
    u.iter_mut().for_each(|x| *x /= total);
    u[n - 1] = 1.0;
    // Averaged knots.
    let mut knots = vec![0.0; p + 1];
    for j in 1..n - p {
        knots.push(u[j..j + p].iter().sum::<f64>() / p as f64);
    }
    knots.extend(vec![1.0; p + 1]);
    // Collocation matrix in band form.
    let bw = p;
    let mut a = vec![[0.0f64; 7]; n];
    for (i, row) in a.iter_mut().enumerate() {
        let t = u[i];
        let span = {
            // Span with knots[span] <= t < knots[span + 1], last span for t = 1.
            let mut s = p;
            while s + 1 < n && knots[s + 1] <= t {
                s += 1;
            }
            s
        };
        let basis = {
            let mut nvals = vec![0.0; p + 1];
            let (mut left, mut right) = (vec![0.0; p + 1], vec![0.0; p + 1]);
            nvals[0] = 1.0;
            for j in 1..=p {
                left[j] = t - knots[span + 1 - j];
                right[j] = knots[span + j] - t;
                let mut saved = 0.0;
                for r in 0..j {
                    let temp = nvals[r] / (right[r + 1] + left[j - r]);
                    nvals[r] = saved + right[r + 1] * temp;
                    saved = left[j - r] * temp;
                }
                nvals[j] = saved;
            }
            nvals
        };
        for (j, b) in basis.iter().enumerate() {
            let col = span - p + j;
            row[col + bw - i] = *b;
        }
    }
    let mut rhs: Vec<[f64; 3]> = points.iter().map(|q| [q.x, q.y, q.z]).collect();
    banded_solve(&mut a, bw, &mut rhs)?;
    let control = rhs.into_iter().map(|c| Point3::new(c[0], c[1], c[2])).collect();
    NurbsCurve::new(p, knots, control, None)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_geom_api::ParametricCurve;

    #[test]
    fn interpolates_circle_samples_and_stays_close_between() {
        let n = 80;
        let pts: Vec<Point3> = (0..=n).map(|i| {
            let a = core::f64::consts::TAU * i as f64 / n as f64;
            Point3::new(2.0 * a.cos(), 2.0 * a.sin(), 0.3 * a)
        }).collect();
        let c = interpolate_points(&pts, 3).unwrap();
        // Interpolation at the data parameters: chord-length parameters.
        let mut u = vec![0.0];
        for i in 1..pts.len() {
            u.push(u[i - 1] + (pts[i] - pts[i - 1]).norm());
        }
        let tot = *u.last().unwrap();
        for (i, q) in pts.iter().enumerate() {
            assert!((c.eval(u[i] / tot) - *q).norm() < 1e-10, "point {i}");
        }
        // Between samples the helix is reproduced to O(h^4).
        for i in 0..n {
            let a = core::f64::consts::TAU * (i as f64 + 0.5) / n as f64;
            let want = Point3::new(2.0 * a.cos(), 2.0 * a.sin(), 0.3 * a);
            let pr = c.project(want, None).unwrap();
            assert!(pr.distance < 5e-6, "mid {i}: {}", pr.distance);
        }
        assert!(interpolate_points(&pts[..1], 3).is_err());
        assert!(interpolate_points(&[pts[0], pts[0]], 3).is_err());
        // Two points: a straight segment.
        let l = interpolate_points(&pts[..2], 3).unwrap();
        assert_eq!(l.degree, 1);
    }
}
