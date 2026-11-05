//! W3b metrics: equiangle skew, tet mean ratio / inradius / squish index, quad radius ratio, hex
//! nodal Jacobian ratio and wedge max stretch. Pure kernels on already-scaled corner lists.

use crate::metric::{Context, Metric};
use crate::vec::{DBL_MAX, TINY, V, add, cap, cross, det3, dot, norm, norm2, scale, sub, unit};
use crate::{planar, solid};
use sm_mesh_api::CellShape;

const HEX_CORNERS: [[usize; 4]; 8] =
    [[0, 1, 3, 4], [1, 2, 0, 5], [2, 3, 1, 6], [3, 0, 2, 7], [4, 7, 5, 0], [5, 4, 6, 1], [6, 5, 7, 2], [7, 6, 4, 3]];

/// True if this module evaluates `m` on `shape`.
pub(crate) fn owns(m: Metric, shape: CellShape) -> bool {
    match m {
        Metric::EquiangleSkew
        | Metric::MeanRatio
        | Metric::Inradius
        | Metric::SquishIndex
        | Metric::NodalJacobianRatio
        | Metric::MaxStretch => true,
        Metric::RadiusRatio => shape == CellShape::Quad,
        _ => false,
    }
}

/// Reference interior (dihedral) angle in degrees for `shape`.
fn ideal_angle(shape: CellShape) -> f64 {
    match shape {
        CellShape::Tri => 60.0,
        CellShape::Tet => (1.0f64 / 3.0).acos().to_degrees(),
        _ => 90.0,
    }
}

/// `max((a_max - a_e) / (180 - a_e), (a_e - a_min) / a_e)`.
pub(crate) fn equiangle(min: f64, max: f64, ideal: f64) -> f64 {
    ((max - ideal) / (180.0 - ideal)).max((ideal - min) / ideal)
}

pub(crate) fn eval(m: Metric, shape: CellShape, p: &[V], ctx: &Context) -> Option<f64> {
    Some(match m {
        Metric::EquiangleSkew => {
            let (lo, hi) = if matches!(shape, CellShape::Tri | CellShape::Quad) {
                (planar::eval(Metric::MinAngle, shape, p, ctx)?, planar::eval(Metric::MaxAngle, shape, p, ctx)?)
            } else {
                (solid::eval(Metric::MinAngle, shape, p, ctx)?, solid::eval(Metric::MaxAngle, shape, p, ctx)?)
            };
            equiangle(lo, hi, ideal_angle(shape))
        }
        Metric::RadiusRatio => quad_radius_ratio(p, ctx),
        Metric::MeanRatio => tet_mean_ratio(p),
        Metric::Inradius => tet_inradius(p),
        Metric::SquishIndex => tet_squish(p),
        Metric::NodalJacobianRatio => hex_nodal_ratio(p),
        Metric::MaxStretch => wedge_max_stretch(p),
        _ => return None,
    })
}

/// Six times the signed volume and the total face area of a tetrahedron.
fn tet_det_and_area(p: &[V]) -> (f64, f64) {
    let (a, b, c) = (sub(p[1], p[0]), sub(p[2], p[0]), sub(p[3], p[0]));
    let det = det3(a, b, c);
    let half = |u: V, v: V| 0.5 * norm(cross(u, v));
    let s = half(a, b) + half(a, c) + half(b, c) + half(sub(p[2], p[1]), sub(p[3], p[1]));
    (det, s)
}

/// `12 (3V)^(2/3) / sum l^2` (1 for the regular tetrahedron), 0 if `V <= 0`.
fn tet_mean_ratio(p: &[V]) -> f64 {
    let (det, _) = tet_det_and_area(p);
    if det <= TINY {
        return 0.0;
    }
    let mut s = 0.0;
    for i in 0..4 {
        for j in (i + 1)..4 {
            s += norm2(sub(p[i], p[j]));
        }
    }
    12.0 * (det / 2.0).powf(2.0 / 3.0) / s
}

/// Inscribed-sphere radius `3V / sum A` (negative for an inverted tetrahedron).
fn tet_inradius(p: &[V]) -> f64 {
    let (det, s) = tet_det_and_area(p);
    if s <= TINY { 0.0 } else { det / (2.0 * s) }
}

/// `max_f (1 - A_f . d_f / (|A_f||d_f|))`, `d_f` from the vertex mean to the face mean and `A_f`
/// the outward area vector; 1 for a degenerate face.
fn tet_squish(p: &[V]) -> f64 {
    let c = scale(add(add(p[0], p[1]), add(p[2], p[3])), 0.25);
    let mut worst = 0.0f64;
    for f in CellShape::Tet.local_faces() {
        let (a, b, d) = (p[f[0] as usize], p[f[1] as usize], p[f[2] as usize]);
        let av = scale(cross(sub(b, a), sub(d, a)), 0.5);
        let dv = sub(scale(add(add(a, b), d), 1.0 / 3.0), c);
        let den = norm(av) * norm(dv);
        worst = worst.max(if den <= TINY { 1.0 } else { 1.0 - dot(av, dv) / den });
    }
    worst
}

/// `(d_max / 2) / (2A / P) / sqrt2`: half the longer diagonal over the inradius `2A/P`, scaled so
/// the square scores 1; `DBL_MAX` for a non-positive area.
fn quad_radius_ratio(p: &[V], ctx: &Context) -> f64 {
    let nh = match ctx.normal {
        Some(v) => unit(v),
        None => unit(cross(sub(p[2], p[0]), sub(p[3], p[1]))),
    };
    let a = planar::area(p, nh);
    let per: f64 = (0..4).map(|k| norm(sub(p[(k + 1) % 4], p[k]))).sum();
    if a <= TINY || per <= TINY {
        return DBL_MAX;
    }
    let dmax = norm(sub(p[2], p[0])).max(norm(sub(p[3], p[1])));
    cap(0.5 * dmax / (2.0 * a / per) / core::f64::consts::SQRT_2)
}

/// Smallest over largest corner Jacobian determinant (signed; 0 if the largest is not positive).
fn hex_nodal_ratio(p: &[V]) -> f64 {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for c in &HEX_CORNERS {
        let o = p[c[0]];
        let d = det3(sub(p[c[1]], o), sub(p[c[2]], o), sub(p[c[3]], o));
        lo = lo.min(d);
        hi = hi.max(d);
    }
    if hi <= TINY { 0.0 } else { lo / hi }
}

/// Worst quad face of `d_max / (sqrt2 l_min)` (inverse quad stretch; 1 for the regular wedge).
fn wedge_max_stretch(p: &[V]) -> f64 {
    let mut worst = 0.0f64;
    for f in CellShape::Wedge.local_faces().iter().filter(|f| f.len() == 4) {
        let q = |i: usize| p[f[i] as usize];
        let lmin = (0..4).map(|i| norm(sub(q((i + 1) % 4), q(i)))).fold(f64::INFINITY, f64::min);
        let dmax = norm(sub(q(2), q(0))).max(norm(sub(q(3), q(1))));
        let v = if lmin <= TINY { DBL_MAX } else { cap(dmax / (core::f64::consts::SQRT_2 * lmin)) };
        worst = worst.max(v);
    }
    worst
}
