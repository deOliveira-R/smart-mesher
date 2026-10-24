//! Kernels for 2-D cells: triangles, quadrilaterals and general polygons.
//!
//! The corner `k` of a polygon `p[0..n]` (counter-clockwise about the reference normal `n`) has
//! edge vectors `a = p[k+1] - p[k]`, `b = p[k-1] - p[k]` and Jacobian `J_k = (a x b) . n`.

use crate::metric::{Context, Metric, relative_size_squared};
use crate::vec::{DBL_MAX, TINY, V, add, cap, cross, dot, norm, norm2, scale, sub, unit};
use sm_mesh_api::CellShape;

const SQRT3: f64 = 1.732_050_807_568_877_2;

/// Twice the area vector, taken relative to `p[0]` for accuracy far from the origin.
fn newell(p: &[V]) -> V {
    let mut s = [0.0; 3];
    for k in 1..p.len() - 1 {
        s = add(s, cross(sub(p[k], p[0]), sub(p[k + 1], p[0])));
    }
    s
}

#[inline]
fn corner(p: &[V], k: usize, nh: V) -> (V, V, f64) {
    let n = p.len();
    let a = sub(p[(k + 1) % n], p[k]);
    let b = sub(p[(k + n - 1) % n], p[k]);
    (a, b, dot(cross(a, b), nh))
}

fn edge_len(p: &[V], k: usize) -> f64 {
    norm(sub(p[(k + 1) % p.len()], p[k]))
}

/// Signed area (positive when counter-clockwise about `nh`).
pub(crate) fn area(p: &[V], nh: V) -> f64 {
    0.5 * dot(newell(p), nh)
}

fn min_max_edges(p: &[V]) -> (f64, f64) {
    let (mut lo, mut hi) = (f64::INFINITY, 0.0f64);
    for k in 0..p.len() {
        let l = edge_len(p, k);
        lo = lo.min(l);
        hi = hi.max(l);
    }
    (lo, hi)
}

/// Interior angle at corner `k` in degrees, in `[0, 360)` (reflex corners exceed 180).
fn angle(p: &[V], k: usize, nh: V) -> f64 {
    let (a, b, _) = corner(p, k, nh);
    let t = dot(cross(a, b), nh).atan2(dot(a, b)).to_degrees();
    if t < 0.0 { t + 360.0 } else { t }
}

/// Bilinear Jacobian determinant at `(xi, eta)` of a quad.
fn quad_detj(p: &[V], xi: f64, eta: f64, nh: V) -> f64 {
    let xx = |c: [f64; 4]| (0..4).fold([0.0; 3], |s, i| add(s, scale(p[i], c[i])));
    let x_xi = xx([(-1.0 + eta) / 4.0, (1.0 - eta) / 4.0, (1.0 + eta) / 4.0, (-1.0 - eta) / 4.0]);
    let x_eta = xx([(-1.0 + xi) / 4.0, (-1.0 - xi) / 4.0, (1.0 + xi) / 4.0, (1.0 - xi) / 4.0]);
    dot(cross(x_xi, x_eta), nh)
}

/// Tri/quad/polygon dispatcher. `p` has already been validated by [`crate::eval`].
pub(crate) fn eval(m: Metric, shape: CellShape, p: &[V], ctx: &Context) -> Option<f64> {
    let n = p.len();
    let nh = match ctx.normal {
        Some(v) => unit(v),
        None => unit(newell(p)),
    };
    let a_signed = area(p, nh);
    let corners = |f: &mut dyn FnMut(V, V, f64)| {
        for k in 0..n {
            let (a, b, j) = corner(p, k, nh);
            f(a, b, j);
        }
    };
    let is_tri = shape == CellShape::Tri;

    // Scaled Jacobian: min over corners of J / (|a||b|).
    let scaled_jac = || {
        let mut sj = f64::INFINITY;
        corners(&mut |a, b, j| {
            let d = norm(a) * norm(b);
            sj = sj.min(if d <= TINY { 0.0 } else { j / d });
        });
        sj
    };
    let shear = || scaled_jac().max(0.0);
    // Quad/tri per-corner Frobenius aspect (a.a + b.b) / (2 J), square weight.
    let af_quad = |mean: bool| {
        let (mut mx, mut sum) = (0.0f64, 0.0);
        corners(&mut |a, b, j| {
            let v = if j <= TINY { DBL_MAX } else { (norm2(a) + norm2(b)) / (2.0 * j) };
            mx = mx.max(v);
            sum += v;
        });
        if mean { cap(sum / n as f64) } else { mx }
    };
    let sum_l2 = || (0..n).map(|k| norm2(sub(p[(k + 1) % n], p[k]))).sum::<f64>();
    let perimeter = || (0..n).map(|k| edge_len(p, k)).sum::<f64>();
    let shape_value = || -> f64 {
        if is_tri {
            if a_signed <= TINY { 0.0 } else { 4.0 * SQRT3 * a_signed / sum_l2() }
        } else {
            let mut s = f64::INFINITY;
            corners(&mut |a, b, j| {
                let d = norm2(a) + norm2(b);
                s = s.min(if j <= TINY || d <= TINY { 0.0 } else { 2.0 * j / d });
            });
            s
        }
    };
    // Quad principal axes and cross derivative.
    let (x1, x2, x12) = if n >= 4 {
        (
            add(sub(p[1], p[0]), sub(p[2], p[3])),
            add(sub(p[2], p[1]), sub(p[3], p[0])),
            add(sub(p[0], p[1]), sub(p[2], p[3])),
        )
    } else {
        ([0.0; 3], [0.0; 3], [0.0; 3])
    };
    let rel = || relative_size_squared(a_signed, ctx.ref_size);

    let v = match m {
        Metric::Volume => a_signed,
        Metric::Jacobian => {
            let mut j = f64::INFINITY;
            corners(&mut |_, _, c| j = j.min(c));
            j
        }
        // Triangles are normalised by 2/sqrt(3) so the equilateral triangle scores 1.
        Metric::ScaledJacobian => if is_tri { scaled_jac() * 2.0 / SQRT3 } else { scaled_jac() },
        Metric::AspectRatio => {
            let (_, lmax) = min_max_edges(p);
            if a_signed <= TINY {
                DBL_MAX
            } else {
                let c = 4.0 * (core::f64::consts::PI / n as f64).tan();
                cap(lmax * perimeter() / (c * a_signed))
            }
        }
        Metric::AspectFrobenius | Metric::Condition if is_tri => {
            if a_signed <= TINY { DBL_MAX } else { cap(sum_l2() / (4.0 * SQRT3 * a_signed)) }
        }
        Metric::Condition => af_quad(false),
        Metric::MaxAspectFrobenius => af_quad(false),
        Metric::MeanAspectFrobenius => af_quad(true),
        Metric::RadiusRatio => {
            // R / (2 r) = abc (a + b + c) / (16 A^2)
            if a_signed <= TINY {
                DBL_MAX
            } else {
                let l: [f64; 3] = [edge_len(p, 0), edge_len(p, 1), edge_len(p, 2)];
                cap(l[0] * l[1] * l[2] * (l[0] + l[1] + l[2]) / (16.0 * a_signed * a_signed))
            }
        }
        Metric::EdgeRatio => {
            let (lo, hi) = min_max_edges(p);
            if lo <= TINY { DBL_MAX } else { cap(hi / lo) }
        }
        Metric::MaxEdgeRatio => {
            let (l1, l2) = (norm(x1), norm(x2));
            if l1 <= TINY || l2 <= TINY { DBL_MAX } else { cap((l1 / l2).max(l2 / l1)) }
        }
        Metric::Skew => {
            let (l1, l2) = (norm(x1), norm(x2));
            if l1 <= TINY || l2 <= TINY { DBL_MAX } else { (dot(x1, x2) / (l1 * l2)).abs() }
        }
        Metric::Taper => {
            let d = norm(x1).min(norm(x2));
            if d <= TINY { DBL_MAX } else { cap(norm(x12) / d) }
        }
        Metric::Warpage => {
            let mut nn = [[0.0; 3]; 4];
            for (k, slot) in nn.iter_mut().enumerate() {
                let (a, b, _) = corner(p, k, nh);
                *slot = unit(cross(a, b));
            }
            dot(nn[0], nn[2]).powi(3).min(dot(nn[1], nn[3]).powi(3))
        }
        Metric::Stretch => {
            let (lo, _) = min_max_edges(p);
            let dmax = norm(sub(p[2], p[0])).max(norm(sub(p[3], p[1])));
            if dmax <= TINY { DBL_MAX } else { core::f64::consts::SQRT_2 * lo / dmax }
        }
        Metric::Shear => shear(),
        Metric::Oddy => {
            let mut worst = 0.0f64;
            corners(&mut |a, b, _| {
                let (g11, g22, g12) = (norm2(a), norm2(b), dot(a, b));
                let g = g11 * g22 - g12 * g12;
                let o = if g <= TINY { DBL_MAX } else { ((g11 - g22).powi(2) + 4.0 * g12 * g12) / (2.0 * g) };
                worst = worst.max(o);
            });
            cap(worst)
        }
        Metric::Distortion => {
            if is_tri {
                if a_signed > TINY {
                    1.0
                } else if a_signed < -TINY {
                    -1.0
                } else {
                    0.0
                }
            } else {
                let g = 1.0 / 3.0f64.sqrt();
                let mut det = [0.0; 4];
                for (i, d) in det.iter_mut().enumerate() {
                    let (xi, eta) = (if i & 1 == 0 { -g } else { g }, if i & 2 == 0 { -g } else { g });
                    *d = quad_detj(p, xi, eta, nh);
                }
                let mean = det.iter().sum::<f64>() / 4.0;
                let min = det.iter().copied().fold(f64::INFINITY, f64::min);
                if mean.abs() <= TINY { 0.0 } else { min / mean.abs() }
            }
        }
        Metric::Shape => shape_value(),
        Metric::RelativeSizeSquared => rel(),
        Metric::ShapeAndSize => rel() * shape_value(),
        Metric::ShearAndSize => rel() * shear(),
        Metric::MinAngle => (0..n).map(|k| angle(p, k, nh)).fold(f64::INFINITY, f64::min),
        Metric::MaxAngle => (0..n).map(|k| angle(p, k, nh)).fold(0.0, f64::max),
        _ => return None,
    };
    Some(v)
}
