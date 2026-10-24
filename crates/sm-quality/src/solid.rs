//! Kernels for the template solids: tetrahedron, hexahedron, wedge and pyramid.
//!
//! Corner `[n, a, b, c]` of a template has edge vectors `p[a]-p[n]`, `p[b]-p[n]`, `p[c]-p[n]`
//! ordered so that the determinant is positive for a right-handed element. The weighted
//! (Knupp) matrix at a corner is `T = A W^-1`, where the columns of `W` are the corner edge
//! vectors of the ideal element, so `T` is orthogonal (up to scale) exactly for the ideal shape.

use crate::faces;
use crate::metric::{Context, Metric, relative_size_squared};
use crate::vec::{DBL_MAX, TINY, V, add, cap, cross, det3, dot, norm, norm2, scale, sub, unit};
use sm_mesh_api::CellShape;

const HEX_CORNERS: &[[u8; 4]] = &[
    [0, 1, 3, 4],
    [1, 2, 0, 5],
    [2, 3, 1, 6],
    [3, 0, 2, 7],
    [4, 7, 5, 0],
    [5, 4, 6, 1],
    [6, 5, 7, 2],
    [7, 6, 4, 3],
];
const WEDGE_CORNERS: &[[u8; 4]] =
    &[[0, 1, 2, 3], [1, 2, 0, 4], [2, 0, 1, 5], [3, 5, 4, 0], [4, 3, 5, 1], [5, 4, 3, 2]];
/// Base corners only: the apex has no canonical corner frame.
const PYRAMID_CORNERS: &[[u8; 4]] = &[[0, 1, 3, 4], [1, 2, 0, 4], [2, 3, 1, 4], [3, 0, 2, 4]];
const TET_CORNERS: &[[u8; 4]] = &[[0, 1, 2, 3]];

const I3: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
const S3: f64 = 0.577_350_269_189_625_8; // 1/sqrt(3)
const S6: f64 = 0.408_248_290_463_863; // 1/sqrt(6)
/// `W^-1` of the regular tetrahedron `W = [(1,0,0), (1/2,sqrt3/2,0), (1/2,sqrt3/6,sqrt6/3)]`.
const TET_WI: [[f64; 3]; 3] = [[1.0, -S3, -S6], [0.0, 2.0 * S3, -S6], [0.0, 0.0, 3.0 * S6]];
/// `W^-1` of the equilateral-base wedge with height equal to the edge length.
const WEDGE_WI: [[f64; 3]; 3] = [[1.0, -S3, 0.0], [0.0, 2.0 * S3, 0.0], [0.0, 0.0, 1.0]];
/// `W^-1` of the square pyramid with all eight edges equal.
const PYR_WI: [[f64; 3]; 3] =
    [[1.0, 0.0, -core::f64::consts::FRAC_1_SQRT_2], [0.0, 1.0, -core::f64::consts::FRAC_1_SQRT_2], [0.0, 0.0, core::f64::consts::SQRT_2]];

/// Tet-style Frobenius aspect of the corner tetrahedra of the ideal wedge (unit edges). Verdict
/// divides by the rounded literal 1.16477; the exact value makes the ideal wedge score exactly 1.
fn wedge_af_ideal() -> f64 {
    4.0 / (3.0 * 1.5f64.cbrt())
}

struct Tpl {
    corners: &'static [[u8; 4]],
    wi: [[f64; 3]; 3],
    /// Factor that makes the scaled Jacobian of the ideal element 1.
    sj_scale: f64,
}

fn tpl(shape: CellShape) -> Tpl {
    match shape {
        CellShape::Hex => Tpl { corners: HEX_CORNERS, wi: I3, sj_scale: 1.0 },
        CellShape::Wedge => Tpl { corners: WEDGE_CORNERS, wi: WEDGE_WI, sj_scale: 2.0 * S3 },
        CellShape::Pyramid => Tpl { corners: PYRAMID_CORNERS, wi: PYR_WI, sj_scale: core::f64::consts::SQRT_2 },
        _ => Tpl { corners: TET_CORNERS, wi: TET_WI, sj_scale: core::f64::consts::SQRT_2 },
    }
}

/// Edge-vector frame of a corner.
#[inline]
fn frame(p: &[V], c: &[u8; 4]) -> [V; 3] {
    let o = p[c[0] as usize];
    [sub(p[c[1] as usize], o), sub(p[c[2] as usize], o), sub(p[c[3] as usize], o)]
}

/// `T = A W^-1` as three column vectors.
#[inline]
fn weighted(f: &[V; 3], wi: &[[f64; 3]; 3]) -> [V; 3] {
    let col = |j: usize| add(add(scale(f[0], wi[0][j]), scale(f[1], wi[1][j])), scale(f[2], wi[2][j]));
    [col(0), col(1), col(2)]
}

/// Quantities of the weighted matrix at one corner.
struct Corner {
    det_t: f64,
    frob2: f64,
    inv_frob2: f64,
}

fn corner_data(f: &[V; 3], wi: &[[f64; 3]; 3]) -> Corner {
    let t = weighted(f, wi);
    let det_t = det3(t[0], t[1], t[2]);
    let frob2 = norm2(t[0]) + norm2(t[1]) + norm2(t[2]);
    let adj2 = norm2(cross(t[1], t[2])) + norm2(cross(t[2], t[0])) + norm2(cross(t[0], t[1]));
    Corner {
        det_t,
        frob2,
        inv_frob2: if det_t.abs() <= TINY { f64::INFINITY } else { adj2 / (det_t * det_t) },
    }
}

impl Corner {
    fn condition(&self) -> f64 {
        if self.det_t <= TINY { DBL_MAX } else { cap((self.frob2 * self.inv_frob2).sqrt() / 3.0) }
    }
    fn shape(&self) -> f64 {
        if self.det_t <= TINY { 0.0 } else { 3.0 * self.det_t.powf(2.0 / 3.0) / self.frob2 }
    }
    fn aspect_frobenius(&self) -> f64 {
        if self.det_t <= TINY { DBL_MAX } else { cap(self.frob2 / (3.0 * self.det_t.powf(2.0 / 3.0))) }
    }
}

/// Principal axes of a hexahedron and their mixed second derivatives.
struct HexAxes {
    x: [V; 3],
    x12: V,
    x13: V,
    x23: V,
}

fn hex_axes(p: &[V]) -> HexAxes {
    let s = |idx: [(usize, f64); 8]| idx.iter().fold([0.0; 3], |s, &(i, w)| add(s, scale(p[i], w)));
    HexAxes {
        x: [
            s([(1, 1.0), (0, -1.0), (2, 1.0), (3, -1.0), (5, 1.0), (4, -1.0), (6, 1.0), (7, -1.0)]),
            s([(3, 1.0), (0, -1.0), (2, 1.0), (1, -1.0), (7, 1.0), (4, -1.0), (6, 1.0), (5, -1.0)]),
            s([(4, 1.0), (0, -1.0), (5, 1.0), (1, -1.0), (6, 1.0), (2, -1.0), (7, 1.0), (3, -1.0)]),
        ],
        x12: s([(0, 1.0), (1, -1.0), (2, 1.0), (3, -1.0), (4, 1.0), (5, -1.0), (6, 1.0), (7, -1.0)]),
        x13: s([(1, 1.0), (0, -1.0), (5, -1.0), (4, 1.0), (2, 1.0), (3, -1.0), (6, -1.0), (7, 1.0)]),
        x23: s([(3, 1.0), (0, -1.0), (7, -1.0), (4, 1.0), (2, 1.0), (1, -1.0), (6, -1.0), (5, 1.0)]),
    }
}

const SX: [f64; 8] = [-1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0];
const SY: [f64; 8] = [-1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0];
const SZ: [f64; 8] = [-1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0];

/// Columns `dx/dxi, dx/deta, dx/dzeta` of the trilinear map and the shape-function derivatives.
fn hex_jac(p: &[V], xi: f64, eta: f64, zeta: f64) -> ([V; 3], [[f64; 3]; 8]) {
    let (mut gx, mut gy, mut gz) = ([0.0; 3], [0.0; 3], [0.0; 3]);
    let mut dn = [[0.0; 3]; 8];
    for i in 0..8 {
        let (fx, fy, fz) = (1.0 + SX[i] * xi, 1.0 + SY[i] * eta, 1.0 + SZ[i] * zeta);
        dn[i] = [SX[i] * fy * fz / 8.0, SY[i] * fx * fz / 8.0, SZ[i] * fx * fy / 8.0];
        gx = add(gx, scale(p[i], dn[i][0]));
        gy = add(gy, scale(p[i], dn[i][1]));
        gz = add(gz, scale(p[i], dn[i][2]));
    }
    ([gx, gy, gz], dn)
}

fn hex_detj(p: &[V], xi: f64, eta: f64, zeta: f64) -> f64 {
    let ([gx, gy, gz], _) = hex_jac(p, xi, eta, zeta);
    det3(gx, gy, gz)
}

/// Tet/linear-element distortion: the sign of the (constant) Jacobian. Verdict returns a
/// constant 1; the sign is kept so inverted tets are visible (documented deviation).
fn tet_distortion(p: &[V]) -> f64 {
    let d = det3(sub(p[1], p[0]), sub(p[2], p[0]), sub(p[3], p[0]));
    if d > TINY {
        1.0
    } else if d < -TINY {
        -1.0
    } else {
        0.0
    }
}

/// Pronto characteristic length `sqrt(0.5 V^2 / sum_i |dV/dx_i|^2)` (`sqrt(3)/3` on the unit cube).
/// `V` and the nodal volume gradients are integrated exactly with the 2x2x2 Gauss rule.
fn hex_dimension(p: &[V]) -> f64 {
    let g = S3;
    let (mut vol, mut grad) = (0.0, [[0.0; 3]; 8]);
    for k in 0..8 {
        let s = |bit: usize| if k & bit == 0 { -g } else { g };
        let ([gx, gy, gz], dn) = hex_jac(p, s(1), s(2), s(4));
        vol += det3(gx, gy, gz);
        let (cx, cy, cz) = (cross(gy, gz), cross(gz, gx), cross(gx, gy));
        for i in 0..8 {
            grad[i] = add(grad[i], add(add(scale(cx, dn[i][0]), scale(cy, dn[i][1])), scale(cz, dn[i][2])));
        }
    }
    let sum: f64 = grad.iter().map(|q| norm2(*q)).sum();
    if sum <= TINY { 0.0 } else { (0.5 * vol * vol / sum).sqrt() }
}

/// Minimum Jacobian over the Gauss points *and* the nodes, over the mean Gauss Jacobian.
fn hex_distortion(p: &[V]) -> f64 {
    let g = S3;
    let (mut sum, mut min) = (0.0, f64::INFINITY);
    for k in 0..8 {
        let s = |bit: usize, v: f64| if k & bit == 0 { -v } else { v };
        let d = hex_detj(p, s(1, g), s(2, g), s(4, g));
        sum += d;
        min = min.min(d).min(hex_detj(p, s(1, 1.0), s(2, 1.0), s(4, 1.0)));
    }
    let mean = sum / 8.0;
    if mean.abs() <= TINY { 0.0 } else { min / mean.abs() }
}

/// Wedge distortion: minimum corner determinant over twice the volume (1 for an affine wedge).
fn wedge_distortion(p: &[V], vol: f64) -> f64 {
    let mut j = f64::INFINITY;
    for c in WEDGE_CORNERS {
        let f = frame(p, c);
        j = j.min(det3(f[0], f[1], f[2]));
    }
    if vol.abs() <= TINY { 0.0 } else { j / (2.0 * vol.abs()) }
}

/// Tet-style Frobenius aspect of the six corner tetrahedra of a wedge, over the value of the
/// ideal wedge, floored at 1. Returns `(max, mean)`.
fn wedge_aspect_frobenius(p: &[V]) -> (f64, f64) {
    let (mut mx, mut sum) = (0.0f64, 0.0);
    let ideal = wedge_af_ideal();
    for c in WEDGE_CORNERS {
        let a = corner_data(&frame(p, c), &TET_WI).aspect_frobenius();
        let v = if a >= DBL_MAX { DBL_MAX } else { a / ideal };
        mx = mx.max(v);
        sum += v;
    }
    (mx.max(1.0), (sum / 6.0).clamp(1.0, DBL_MAX))
}

/// Verdict wedge shape: minimum over corners of `3 (det 2/sqrt3)^(2/3) / sum |edge|^2`, capped at 1.
fn wedge_shape(p: &[V]) -> f64 {
    let mut s = 1.0f64;
    for c in WEDGE_CORNERS {
        let f = frame(p, c);
        let det = det3(f[0], f[1], f[2]);
        if det <= TINY {
            return 0.0;
        }
        s = s.min(3.0 * (det * 2.0 * S3).powf(2.0 / 3.0) / (norm2(f[0]) + norm2(f[1]) + norm2(f[2])));
    }
    s
}

/// Verdict pyramid shape: base quad shape x cosine of the apex offset angle x height/length balance.
fn pyramid_shape(p: &[V]) -> f64 {
    let n = unit(cross(sub(p[2], p[0]), sub(p[3], p[1])));
    let c = scale(add(add(p[0], p[1]), add(p[2], p[3])), 0.25);
    let up = sub(p[4], c);
    let (d, len) = (dot(up, n), norm(up));
    let (_, lmax) = edge_extrema(CellShape::Pyramid, p);
    let l = lmax * core::f64::consts::FRAC_1_SQRT_2;
    if d <= TINY || len <= TINY || l <= TINY {
        return 0.0;
    }
    let cos = d / len;
    if cos <= 0.0 {
        return 0.0;
    }
    let ctx = Context { normal: Some(n), ref_size: f64::NAN };
    let base = crate::planar::eval(Metric::Shape, CellShape::Quad, &p[..4], &ctx).unwrap_or(0.0);
    base * cos * (d / l).min(l / d)
}

/// Pyramid scaled Jacobian before flooring and folding: minimum tet scaled Jacobian over the
/// four sub-tetrahedra times `sqrt2`. Negative for an inverted pyramid.
pub(crate) fn pyramid_signed_sj(p: &[V]) -> f64 {
    const SUBS: [[usize; 4]; 4] = [[0, 1, 2, 4], [0, 2, 3, 4], [0, 1, 3, 4], [1, 2, 3, 4]];
    let mut m = f64::INFINITY;
    for s in SUBS {
        let q = [p[s[0]], p[s[1]], p[s[2]], p[s[3]]];
        m = m.min(tet_eval(Metric::ScaledJacobian, &q).unwrap_or(0.0));
    }
    m * core::f64::consts::SQRT_2
}

/// Pyramid scaled Jacobian as Verdict reports it: floored at 0 and folded into `[0, 1]`.
fn pyramid_scaled_jacobian(p: &[V]) -> f64 {
    let x = pyramid_signed_sj(p).max(0.0);
    if x > 1.0 { 2.0 - x } else { x }
}

/// Mean corner determinant of a hexahedron (Verdict's size measure for relative-size metrics).
fn hex_mean_corner_det(p: &[V]) -> f64 {
    let s: f64 = HEX_CORNERS
        .iter()
        .map(|c| {
            let f = frame(p, c);
            det3(f[0], f[1], f[2])
        })
        .sum();
    s / 8.0
}

fn edge_extrema(shape: CellShape, p: &[V]) -> (f64, f64) {
    let (mut lo, mut hi) = (f64::INFINITY, 0.0f64);
    for [a, b] in shape.local_edges() {
        let l = norm(sub(p[*a as usize], p[*b as usize]));
        lo = lo.min(l);
        hi = hi.max(l);
    }
    (lo, hi)
}

/// Tetrahedron-only quantities.
fn tet_eval(m: Metric, p: &[V]) -> Option<f64> {
    let (a, b, c) = (sub(p[1], p[0]), sub(p[2], p[0]), sub(p[3], p[0]));
    let det = det3(a, b, c);
    let l = |i: usize, j: usize| norm(sub(p[i], p[j]));
    let (l01, l02, l03, l12, l13, l23) = (l(0, 1), l(0, 2), l(0, 3), l(1, 2), l(1, 3), l(2, 3));
    // Opposite-face areas of vertices 3, 2, 0, 1.
    let area = |u: V, v: V| 0.5 * norm(cross(u, v));
    let faces_a = [
        area(a, b),
        area(a, c),
        area(sub(p[2], p[1]), sub(p[3], p[1])),
        area(b, c),
    ];
    let sum_a: f64 = faces_a.iter().sum();
    let v = match m {
        Metric::Jacobian => det,
        Metric::ScaledJacobian => {
            let prod = (l01 * l02 * l03).max(l01 * l12 * l13).max(l02 * l12 * l23).max(l03 * l13 * l23);
            if prod <= TINY { 0.0 } else { core::f64::consts::SQRT_2 * det / prod }
        }
        Metric::AspectRatio => {
            if det <= TINY {
                DBL_MAX
            } else {
                let lmax = l01.max(l02).max(l03).max(l12).max(l13).max(l23);
                cap(lmax * sum_a / (6.0 * 6.0f64.sqrt() * (det / 6.0)))
            }
        }
        Metric::AspectGamma => {
            if det <= TINY {
                DBL_MAX
            } else {
                let s = l01 * l01 + l02 * l02 + l03 * l03 + l12 * l12 + l13 * l13 + l23 * l23;
                cap((s / 6.0).powf(1.5) * core::f64::consts::SQRT_2 / (12.0 * det / 6.0))
            }
        }
        Metric::RadiusRatio => {
            if det <= TINY {
                DBL_MAX
            } else {
                let num = add(add(scale(cross(b, c), norm2(a)), scale(cross(c, a), norm2(b))), scale(cross(a, b), norm2(c)));
                let circ = norm(num) / (2.0 * det);
                let inr = det / (2.0 * sum_a);
                cap(circ / (3.0 * inr))
            }
        }
        Metric::CollapseRatio => {
            // Opposite-face edge sets for vertices 0..3.
            let opp = [l12.max(l13).max(l23), l02.max(l03).max(l23), l01.max(l03).max(l13), l01.max(l02).max(l12)];
            // faces_a is ordered (012, 013, 123, 023); the face opposite vertex i:
            let a_opp = [faces_a[2], faces_a[3], faces_a[1], faces_a[0]];
            let mut r = f64::INFINITY;
            for i in 0..4 {
                let h = if a_opp[i] <= TINY { 0.0 } else { det / (2.0 * a_opp[i]) };
                r = r.min(if opp[i] <= TINY { 0.0 } else { h / opp[i] });
            }
            r
        }
        _ => return None,
    };
    Some(v)
}

/// Solid dispatcher. `p` has already been validated by [`crate::eval`].
pub(crate) fn eval(m: Metric, shape: CellShape, p: &[V], ctx: &Context) -> Option<f64> {
    let t = tpl(shape);
    let is_tet = shape == CellShape::Tet;
    let faces = shape.local_faces();
    let vol_fn = || faces::fan_volume(p, faces);
    let frames = |f: &mut dyn FnMut(&Corner)| {
        for c in t.corners {
            f(&corner_data(&frame(p, c), &t.wi));
        }
    };
    let n_corners = t.corners.len() as f64;
    let is_hex = shape == CellShape::Hex;
    // Verdict sizes a hex by its mean corner determinant, not by its volume.
    let rel = || relative_size_squared(if is_hex { hex_mean_corner_det(p) } else { vol_fn() }, ctx.ref_size);
    let shape_value = || match shape {
        CellShape::Wedge => wedge_shape(p),
        CellShape::Pyramid => pyramid_shape(p),
        _ => {
            let mut s = f64::INFINITY;
            frames(&mut |c| s = s.min(c.shape()));
            s
        }
    };
    // Cheap raw-frame passes (no weighted matrix) for the Jacobian-type metrics.
    let raw = |f: &mut dyn FnMut(f64, f64)| {
        for c in t.corners {
            let fr = frame(p, c);
            f(det3(fr[0], fr[1], fr[2]), norm(fr[0]) * norm(fr[1]) * norm(fr[2]));
        }
    };
    let corner_sj = || {
        let mut sj = f64::INFINITY;
        raw(&mut |det, prod| sj = sj.min(if prod <= TINY { 0.0 } else { det / prod }));
        sj
    };
    let hex = (shape == CellShape::Hex).then(|| hex_axes(p));

    let v = match m {
        Metric::Volume => vol_fn(),
        Metric::Jacobian | Metric::ScaledJacobian | Metric::AspectRatio | Metric::AspectGamma | Metric::RadiusRatio | Metric::CollapseRatio
            if is_tet =>
        {
            return tet_eval(m, p);
        }
        Metric::RelativeSizeSquared => rel(),
        Metric::Jacobian => {
            let mut j = f64::INFINITY;
            raw(&mut |det, _| j = j.min(det));
            if let Some(h) = &hex {
                j = j.min(det3(h.x[0], h.x[1], h.x[2]) / 64.0);
            }
            j
        }
        Metric::ScaledJacobian if shape == CellShape::Pyramid => pyramid_scaled_jacobian(p),
        Metric::ScaledJacobian => {
            let mut sj = corner_sj();
            if let Some(h) = &hex {
                let d = norm(h.x[0]) * norm(h.x[1]) * norm(h.x[2]);
                sj = sj.min(if d <= TINY { 0.0 } else { det3(h.x[0], h.x[1], h.x[2]) / d });
            }
            sj * t.sj_scale
        }
        // Hexes: Verdict's `condition_comp / 3` per corner, so max AF == Condition.
        Metric::AspectFrobenius | Metric::MaxAspectFrobenius | Metric::Condition if is_hex => {
            let mut mx = 0.0f64;
            frames(&mut |c| mx = mx.max(c.condition()));
            mx
        }
        Metric::MeanAspectFrobenius if is_hex => {
            let mut s = 0.0;
            frames(&mut |c| s += c.condition());
            cap(s / n_corners)
        }
        // Wedges: tet-style AF of the six corner tetrahedra; Condition == MaxAspectFrobenius.
        Metric::AspectFrobenius | Metric::MaxAspectFrobenius | Metric::Condition if shape == CellShape::Wedge => {
            wedge_aspect_frobenius(p).0
        }
        Metric::MeanAspectFrobenius if shape == CellShape::Wedge => wedge_aspect_frobenius(p).1,
        Metric::AspectFrobenius | Metric::MaxAspectFrobenius => {
            let mut mx = 0.0f64;
            frames(&mut |c| mx = mx.max(c.aspect_frobenius()));
            mx
        }
        Metric::MeanAspectFrobenius => {
            let mut s = 0.0;
            frames(&mut |c| s += c.aspect_frobenius());
            cap(s / n_corners)
        }
        Metric::Condition => {
            let mut mx = 0.0f64;
            frames(&mut |c| mx = mx.max(c.condition()));
            mx
        }
        Metric::Shape => shape_value(),
        Metric::ShapeAndSize => rel() * shape_value(),
        Metric::Shear | Metric::ShearAndSize => {
            let s = corner_sj().max(0.0);
            if m == Metric::Shear { s } else { rel() * s }
        }
        Metric::Distortion => match shape {
            CellShape::Hex => hex_distortion(p),
            CellShape::Wedge => wedge_distortion(p, vol_fn()),
            _ => tet_distortion(p),
        },
        Metric::EdgeRatio => {
            let (lo, hi) = edge_extrema(shape, p);
            if lo <= TINY { DBL_MAX } else { cap(hi / lo) }
        }
        Metric::MinAngle | Metric::MaxAngle => {
            let (lo, hi) = faces::dihedral_extrema(p, faces)?;
            if m == Metric::MinAngle { lo } else { hi }
        }
        Metric::Sphericity => faces::sphericity(p, faces),
        Metric::Starness => faces::starness(p, faces),
        Metric::Dimension => hex_dimension(p),
        Metric::MaxEdgeRatio | Metric::Skew | Metric::Taper => {
            let h = hex.as_ref()?;
            let l = [norm(h.x[0]), norm(h.x[1]), norm(h.x[2])];
            if l.iter().any(|&v| v <= TINY) {
                return Some(DBL_MAX);
            }
            match m {
                Metric::MaxEdgeRatio => {
                    let mut r = 0.0f64;
                    for i in 0..3 {
                        for j in 0..3 {
                            r = r.max(l[i] / l[j]);
                        }
                    }
                    cap(r)
                }
                Metric::Skew => {
                    let c = |i: usize, j: usize| (dot(h.x[i], h.x[j]) / (l[i] * l[j])).abs();
                    c(0, 1).max(c(0, 2)).max(c(1, 2))
                }
                _ => cap(
                    (norm(h.x12) / l[0].min(l[1]))
                        .max(norm(h.x13) / l[0].min(l[2]))
                        .max(norm(h.x23) / l[1].min(l[2])),
                ),
            }
        }
        Metric::Stretch => {
            let (lo, _) = edge_extrema(shape, p);
            let dmax = [(0, 6), (1, 7), (2, 4), (3, 5)].iter().map(|&(a, b)| norm(sub(p[a], p[b]))).fold(0.0, f64::max);
            if dmax <= TINY { DBL_MAX } else { 3.0f64.sqrt() * lo / dmax }
        }
        Metric::Diagonal => {
            let d: [f64; 4] = [(0, 6), (1, 7), (2, 4), (3, 5)].map(|(a, b)| norm(sub(p[a], p[b])));
            let (lo, hi) = d.iter().fold((f64::INFINITY, 0.0f64), |(l, h), &x| (l.min(x), h.max(x)));
            if hi <= TINY { 0.0 } else { lo / hi }
        }
        Metric::Oddy => {
            // Centre sample first (principal axes), then the 8 corners; DBL_MAX if any det <= TINY.
            let oddy = |f: [V; 3]| {
                let g = |i: usize, j: usize| dot(f[i], f[j]);
                let fro2: f64 = (0..3).map(|i| (0..3).map(|j| g(i, j) * g(i, j)).sum::<f64>()).sum();
                let tr = g(0, 0) + g(1, 1) + g(2, 2);
                let det = det3(f[0], f[1], f[2]);
                if det <= TINY { DBL_MAX } else { (fro2 - tr * tr / 3.0) / det.powf(4.0 / 3.0) }
            };
            let mut worst = hex.as_ref().map_or(0.0, |h| oddy(h.x));
            for cn in t.corners {
                worst = worst.max(oddy(frame(p, cn)));
            }
            cap(worst)
        }
        _ => return None,
    };
    Some(v)
}
