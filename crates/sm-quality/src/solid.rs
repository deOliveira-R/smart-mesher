//! Kernels for the template solids: tetrahedron, hexahedron, wedge and pyramid.
//!
//! Corner `[n, a, b, c]` of a template has edge vectors `p[a]-p[n]`, `p[b]-p[n]`, `p[c]-p[n]`
//! ordered so that the determinant is positive for a right-handed element. The weighted
//! (Knupp) matrix at a corner is `T = A W^-1`, where the columns of `W` are the corner edge
//! vectors of the ideal element, so `T` is orthogonal (up to scale) exactly for the ideal shape.

use crate::faces;
use crate::metric::{Context, Metric, relative_size_squared};
use crate::vec::{DBL_MAX, TINY, V, add, cap, cross, det3, dot, norm, norm2, scale, sub};
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

fn hex_detj(p: &[V], xi: f64, eta: f64, zeta: f64) -> f64 {
    let (mut gx, mut gy, mut gz) = ([0.0; 3], [0.0; 3], [0.0; 3]);
    for i in 0..8 {
        let (fx, fy, fz) = (1.0 + SX[i] * xi, 1.0 + SY[i] * eta, 1.0 + SZ[i] * zeta);
        gx = add(gx, scale(p[i], SX[i] * fy * fz / 8.0));
        gy = add(gy, scale(p[i], SY[i] * fx * fz / 8.0));
        gz = add(gz, scale(p[i], SZ[i] * fx * fy / 8.0));
    }
    det3(gx, gy, gz)
}

fn wedge_detj(p: &[V], xi: f64, eta: f64, zeta: f64) -> f64 {
    let l = [1.0 - xi - eta, xi, eta];
    let (dxi, deta) = ([-1.0, 1.0, 0.0], [-1.0, 0.0, 1.0]);
    let (mut gx, mut gy, mut gz) = ([0.0; 3], [0.0; 3], [0.0; 3]);
    for (i, q) in p.iter().enumerate().take(6) {
        let (zf, dz) = if i < 3 { ((1.0 - zeta) / 2.0, -0.5) } else { ((1.0 + zeta) / 2.0, 0.5) };
        gx = add(gx, scale(*q, dxi[i % 3] * zf));
        gy = add(gy, scale(*q, deta[i % 3] * zf));
        gz = add(gz, scale(*q, l[i % 3] * dz));
    }
    det3(gx, gy, gz)
}

fn distortion(shape: CellShape, p: &[V]) -> f64 {
    let g = S3;
    let mut det: [f64; 8] = [0.0; 8];
    let n = match shape {
        CellShape::Hex => {
            for (i, d) in det.iter_mut().enumerate() {
                let s = |bit: usize| if i & bit == 0 { -g } else { g };
                *d = hex_detj(p, s(1), s(2), s(4));
            }
            8
        }
        CellShape::Wedge => {
            let tri = [(1.0 / 6.0, 1.0 / 6.0), (2.0 / 3.0, 1.0 / 6.0), (1.0 / 6.0, 2.0 / 3.0)];
            for (i, d) in det.iter_mut().take(6).enumerate() {
                let (xi, eta) = tri[i % 3];
                *d = wedge_detj(p, xi, eta, if i < 3 { -g } else { g });
            }
            6
        }
        _ => {
            // Constant Jacobian: +1 for a positive element, -1 for an inverted one.
            let d = det3(sub(p[1], p[0]), sub(p[2], p[0]), sub(p[3], p[0]));
            return if d > TINY {
                1.0
            } else if d < -TINY {
                -1.0
            } else {
                0.0
            };
        }
    };
    let mean = det[..n].iter().sum::<f64>() / n as f64;
    let min = det[..n].iter().copied().fold(f64::INFINITY, f64::min);
    if mean.abs() <= TINY { 0.0 } else { min / mean.abs() }
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
    let rel = || relative_size_squared(vol_fn(), ctx.ref_size);
    let shape_value = || {
        let mut s = f64::INFINITY;
        frames(&mut |c| s = s.min(c.shape()));
        s
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
        Metric::ScaledJacobian => {
            let mut sj = corner_sj();
            if let Some(h) = &hex {
                let d = norm(h.x[0]) * norm(h.x[1]) * norm(h.x[2]);
                sj = sj.min(if d <= TINY { 0.0 } else { det3(h.x[0], h.x[1], h.x[2]) / d });
            }
            sj * t.sj_scale
        }
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
        Metric::Distortion => distortion(shape, p),
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
        Metric::Dimension => vol_fn().max(0.0).cbrt(),
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
            let mut worst = 0.0f64;
            for cn in t.corners {
                let f = frame(p, cn);
                let g = |i: usize, j: usize| dot(f[i], f[j]);
                let fro2: f64 = (0..3).map(|i| (0..3).map(|j| g(i, j) * g(i, j)).sum::<f64>()).sum();
                let tr = g(0, 0) + g(1, 1) + g(2, 2);
                let det = det3(f[0], f[1], f[2]);
                let o = if det.abs() <= TINY { DBL_MAX } else { (fro2 - tr * tr / 3.0) / det.abs().powf(4.0 / 3.0) };
                worst = worst.max(o);
            }
            cap(worst)
        }
        _ => return None,
    };
    Some(v)
}
