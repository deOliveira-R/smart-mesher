//! Interior layer placement: an affine map fitted on the boundary columns plus a Shepard
//! interpolation of the boundary residual.

use crate::error::SweepError;
use crate::fit::fit_affine;
use crate::params::Placement;
use rayon::prelude::*;
use sm_core::{Point3, Transform, Vector3};
use sm_scheme_api::NodeKey;

/// Interior nodes (or layers) at which the placement switches to rayon. Output does not depend
/// on it.
pub const PAR_MIN: usize = 1024;

/// Relative tolerance of the translation fast path.
const TRANSLATION_TOL: f64 = 1e-12;

/// The boundary nodes of the source and their columns.
#[derive(Clone, Copy, Debug)]
pub struct Boundary<'a> {
    /// Canonical keys, one per boundary node (any order; unique).
    pub keys: &'a [NodeKey],
    /// Source positions `b_j`, same order as `keys`.
    pub src: &'a [Point3],
    /// Column positions, row-major `[j * (layers + 1) + l]` for `l = 0..=layers`
    /// (`l = 0` is the source node, `l = layers` the target node).
    pub cols: &'a [Point3],
    /// Number of layers `L` (cells per column).
    pub layers: usize,
}

/// The interior source nodes to place.
#[derive(Clone, Copy, Debug)]
pub struct Interior<'a> {
    /// Canonical keys, one per node.
    pub keys: &'a [NodeKey],
    /// Source positions, same order as `keys`.
    pub pos: &'a [Point3],
}

/// Placement settings.
#[derive(Clone, Copy, Debug)]
pub struct PlaceOptions {
    /// Placement mode.
    pub placement: Placement,
    /// Shepard exponent `p` (positive).
    pub shepard_power: f64,
    /// Nearest boundary nodes per interior node (`0`: all).
    pub support: usize,
}

impl Default for PlaceOptions {
    fn default() -> Self {
        Self {
            placement: Placement::Auto,
            shepard_power: 3.0,
            support: 16,
        }
    }
}

/// How a layer was placed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LayerMode {
    /// All columns displace by the same vector: `x + d`.
    Translation,
    /// Fitted affine map plus Shepard residual.
    Affine,
    /// Shepard interpolation of the column displacements.
    Interpolate,
}

/// Result of [`place_layers`].
#[derive(Clone, Debug, PartialEq)]
pub struct Placed {
    /// Interior node positions for layers `1..layers`, row-major `[i * (layers - 1) + (l - 1)]`
    /// over the interior nodes in input order. Empty when `layers == 1`.
    pub positions: Vec<Point3>,
    /// The mode used for each layer `1..layers`.
    pub modes: Vec<LayerMode>,
}

enum Fit {
    Translation(Vector3),
    Affine(Transform, Vec<Vector3>),
    Interp(Vec<Vector3>),
}

/// Sparse Shepard weights, `k` neighbours per interior node (indices into the key-sorted
/// boundary).
struct Weights {
    k: usize,
    idx: Vec<u32>,
    w: Vec<f64>,
}

/// Uniform grid over the sorted boundary points for k-nearest-neighbour queries.
struct Grid {
    lo: [f64; 3],
    h: f64,
    dims: [usize; 3],
    start: Vec<u32>,
    items: Vec<u32>,
}

impl Grid {
    fn new(pts: &[Point3]) -> Self {
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for p in pts {
            for (a, v) in [p.x, p.y, p.z].into_iter().enumerate() {
                lo[a] = lo[a].min(v);
                hi[a] = hi[a].max(v);
            }
        }
        let ext = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
        let big = ext.iter().cloned().fold(0.0, f64::max);
        let live: Vec<f64> = ext.iter().cloned().filter(|e| *e > 1e-9 * big).collect();
        let target_cells = (pts.len() as f64 / 2.0).max(1.0);
        let h = if live.is_empty() {
            1.0
        } else {
            let vol: f64 = live.iter().product();
            (vol / target_cells)
                .powf(1.0 / live.len() as f64)
                .max(1e-300)
        };
        let mut dims = [1usize; 3];
        for a in 0..3 {
            if ext[a] > 1e-9 * big {
                dims[a] = ((ext[a] / h).floor() as usize + 1).min(1 << 12);
            }
        }
        let mut g = Grid {
            lo,
            h,
            dims,
            start: vec![0; dims[0] * dims[1] * dims[2] + 1],
            items: vec![0; pts.len()],
        };
        let cell: Vec<usize> = pts.iter().map(|p| g.cell_of(*p)).collect();
        for c in &cell {
            g.start[*c + 1] += 1;
        }
        for i in 0..g.start.len() - 1 {
            g.start[i + 1] += g.start[i];
        }
        let mut fill = g.start.clone();
        for (i, c) in cell.iter().enumerate() {
            g.items[fill[*c] as usize] = i as u32;
            fill[*c] += 1;
        }
        g
    }

    fn coords(&self, p: Point3) -> [usize; 3] {
        let mut c = [0usize; 3];
        for (a, v) in [p.x, p.y, p.z].into_iter().enumerate() {
            let q = ((v - self.lo[a]) / self.h).floor();
            c[a] = if q.is_finite() && q > 0.0 {
                (q as usize).min(self.dims[a] - 1)
            } else {
                0
            };
        }
        c
    }

    fn cell_of(&self, p: Point3) -> usize {
        let c = self.coords(p);
        (c[2] * self.dims[1] + c[1]) * self.dims[0] + c[0]
    }

    /// The `k` nearest points to `x` as `(distance², index)` ascending (ties by index).
    fn knn(&self, pts: &[Point3], x: Point3, k: usize, best: &mut Vec<(f64, u32)>) {
        best.clear();
        let c = self.coords(x);
        let maxdim = self.dims.iter().cloned().max().unwrap_or(1);
        for r in 0..=maxdim {
            let ri = r as isize;
            let rng = |a: usize| {
                let lo = (c[a] as isize - ri).max(0);
                let hi = (c[a] as isize + ri).min(self.dims[a] as isize - 1);
                lo..=hi
            };
            for z in rng(2) {
                for y in rng(1) {
                    for xx in rng(0) {
                        let d = (xx - c[0] as isize)
                            .abs()
                            .max((y - c[1] as isize).abs())
                            .max((z - c[2] as isize).abs());
                        if d != ri {
                            continue;
                        }
                        let ci =
                            (z as usize * self.dims[1] + y as usize) * self.dims[0] + xx as usize;
                        for &it in &self.items[self.start[ci] as usize..self.start[ci + 1] as usize]
                        {
                            let q = pts[it as usize];
                            let d2 =
                                (q.x - x.x).powi(2) + (q.y - x.y).powi(2) + (q.z - x.z).powi(2);
                            let cand = (d2, it);
                            if best.len() < k || cand < best[best.len() - 1] {
                                let at = best.partition_point(|b| *b < cand);
                                best.insert(at, cand);
                                best.truncate(k);
                            }
                        }
                    }
                }
            }
            if best.len() == k {
                let reach = r as f64 * self.h;
                if best[k - 1].0 <= reach * reach {
                    break;
                }
            }
        }
    }
}

fn weights(bsrc: &[Point3], interior: &[Point3], support: usize, power: f64) -> Weights {
    let nb = bsrc.len();
    let k = if support == 0 { nb } else { support.min(nb) };
    let grid = (k < nb).then(|| Grid::new(bsrc));
    let mut idx = vec![0u32; interior.len() * k];
    let mut w = vec![0.0f64; interior.len() * k];
    let one = |x: Point3, ids: &mut [u32], ws: &mut [f64], best: &mut Vec<(f64, u32)>| {
        match &grid {
            Some(g) => g.knn(bsrc, x, k, best),
            None => {
                best.clear();
                best.extend(bsrc.iter().enumerate().map(|(j, q)| {
                    let d2 = (q.x - x.x).powi(2) + (q.y - x.y).powi(2) + (q.z - x.z).powi(2);
                    (d2, j as u32)
                }));
                best.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
            }
        }
        let coincident = best.first().is_some_and(|b| b.0 == 0.0);
        let mut sum = 0.0;
        for (s, (d2, j)) in best.iter().enumerate() {
            ids[s] = *j;
            ws[s] = if coincident {
                f64::from(u8::from(s == 0))
            } else {
                d2.powf(-0.5 * power)
            };
            sum += ws[s];
        }
        for v in ws.iter_mut() {
            *v /= sum;
        }
    };
    if k == 0 {
        return Weights { k, idx, w };
    }
    if interior.len() >= PAR_MIN {
        idx.par_chunks_mut(k)
            .zip(w.par_chunks_mut(k))
            .zip(interior.par_iter())
            .for_each_init(Vec::new, |best, ((ids, ws), x)| one(*x, ids, ws, best));
    } else {
        let mut best = Vec::new();
        for ((ids, ws), x) in idx.chunks_mut(k).zip(w.chunks_mut(k)).zip(interior) {
            one(*x, ids, ws, &mut best);
        }
    }
    Weights { k, idx, w }
}

fn scale_of(src: &[Point3]) -> f64 {
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for p in src {
        for (a, v) in [p.x, p.y, p.z].into_iter().enumerate() {
            lo[a] = lo[a].min(v);
            hi[a] = hi[a].max(v);
        }
    }
    let d = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt();
    if d > 0.0 { d } else { 1.0 }
}

/// Place the interior nodes of layers `1..layers` from the boundary columns.
///
/// One layer `l` at a time: let `c_j` be the layer-`l` position of boundary node `j` (sorted by
/// `NodeKey`; ties in distance are also broken by key, so the result does not depend on the
/// input order).
/// 1. If every `c_j - b_j` agrees within `1e-12` of the source bounding-box diagonal the layer
///    is a translation: `x + d` with `d` the mean (bit-exact lattices stay exact).
/// 2. Otherwise [`crate::fit_affine`] gives `M`, `t`; the residuals `r_j = c_j - M b_j - t` are
///    interpolated with Shepard weights `w_j ~ d_j^-p` over the `support` nearest boundary
///    nodes: `x -> M x + t + sum_j w_j(x) r_j`. Weights are computed once per node and reused
///    for every layer. A boundary node of the mesh is never moved: the columns are inputs.
/// 3. [`Placement::Interpolate`] skips the affine part: `x + sum_j w_j (c_j - b_j)`.
///    [`Placement::Auto`] uses the affine part when the fit succeeds and falls back to
///    interpolation when it does not (too few or degenerate boundary points).
///
/// # Errors
/// [`SweepError::Input`] for inconsistent lengths, duplicate keys or a non-positive exponent;
/// the fit error for [`Placement::Affine`] when the fit is impossible.
pub fn place_layers(
    b: &Boundary<'_>,
    interior: &Interior<'_>,
    opts: &PlaceOptions,
) -> Result<Placed, SweepError> {
    let bad = |m: &str| SweepError::Input(m.to_string());
    let (nb, layers) = (b.keys.len(), b.layers);
    if layers == 0 {
        return Err(bad("layers must be at least 1"));
    }
    if b.src.len() != nb || b.cols.len() != nb * (layers + 1) {
        return Err(bad("boundary arrays have inconsistent lengths"));
    }
    if interior.keys.len() != interior.pos.len() {
        return Err(bad("interior arrays have inconsistent lengths"));
    }
    if !(opts.shepard_power.is_finite() && opts.shepard_power > 0.0) {
        return Err(bad("shepard_power must be positive"));
    }
    let ni = interior.pos.len();
    if layers == 1 || ni == 0 {
        return Ok(Placed {
            positions: Vec::new(),
            modes: vec![LayerMode::Translation; layers - 1],
        });
    }
    if nb == 0 {
        return Err(bad("no boundary nodes"));
    }
    let mut order: Vec<usize> = (0..nb).collect();
    order.sort_by_key(|j| b.keys[*j]);
    if order.windows(2).any(|w| b.keys[w[0]] == b.keys[w[1]]) {
        return Err(bad("duplicate boundary keys"));
    }
    let bsrc: Vec<Point3> = order.iter().map(|j| b.src[*j]).collect();
    let stride = layers + 1;
    let scale = scale_of(&bsrc);
    let layer_fit = |l: usize| -> Result<Fit, SweepError> {
        let c: Vec<Point3> = order.iter().map(|j| b.cols[*j * stride + l]).collect();
        let mut mean = Vector3::default();
        for (s, p) in bsrc.iter().zip(&c) {
            mean = mean + (*p - *s);
        }
        let mut mean = mean * (1.0 / nb as f64);
        // Identical displacements (the common lattice case) are used as they are, so no
        // rounding enters through the sum.
        let d0 = c[0] - bsrc[0];
        if bsrc.iter().zip(&c).all(|(s, p)| *p - *s == d0) {
            mean = d0;
        }
        let tol = TRANSLATION_TOL * scale;
        if bsrc
            .iter()
            .zip(&c)
            .all(|(s, p)| (*p - *s - mean).norm() <= tol)
        {
            return Ok(Fit::Translation(mean));
        }
        let affine = match opts.placement {
            Placement::Interpolate => None,
            Placement::Affine => Some(fit_affine(&bsrc, &c)?),
            Placement::Auto => fit_affine(&bsrc, &c).ok(),
        };
        Ok(match affine {
            Some(t) => {
                let res = bsrc
                    .iter()
                    .zip(&c)
                    .map(|(s, p)| *p - t.apply_point(*s))
                    .collect();
                Fit::Affine(t, res)
            }
            None => Fit::Interp(bsrc.iter().zip(&c).map(|(s, p)| *p - *s).collect()),
        })
    };
    let fits: Vec<Fit> = if layers > PAR_MIN {
        (1..layers)
            .into_par_iter()
            .map(layer_fit)
            .collect::<Result<_, _>>()?
    } else {
        (1..layers).map(layer_fit).collect::<Result<_, _>>()?
    };
    let need_w = fits.iter().any(|f| !matches!(f, Fit::Translation(_)));
    let w = if need_w {
        weights(&bsrc, interior.pos, opts.support, opts.shepard_power)
    } else {
        Weights {
            k: 0,
            idx: Vec::new(),
            w: Vec::new(),
        }
    };
    let nl = layers - 1;
    let mut positions = vec![Point3::default(); ni * nl];
    let fill = |i: usize, out: &mut [Point3]| {
        let x = interior.pos[i];
        let (ids, ws) = (&w.idx[i * w.k..(i + 1) * w.k], &w.w[i * w.k..(i + 1) * w.k]);
        for (slot, fit) in out.iter_mut().zip(&fits) {
            let shep = |res: &[Vector3]| {
                let mut s = Vector3::default();
                for (j, wj) in ids.iter().zip(ws) {
                    s = s + res[*j as usize] * *wj;
                }
                s
            };
            *slot = match fit {
                Fit::Translation(d) => x + *d,
                Fit::Affine(t, res) => t.apply_point(x) + shep(res),
                Fit::Interp(res) => x + shep(res),
            };
        }
    };
    if ni >= PAR_MIN {
        positions
            .par_chunks_mut(nl)
            .enumerate()
            .for_each(|(i, out)| fill(i, out));
    } else {
        for (i, out) in positions.chunks_mut(nl).enumerate() {
            fill(i, out);
        }
    }
    let modes = fits
        .iter()
        .map(|f| match f {
            Fit::Translation(_) => LayerMode::Translation,
            Fit::Affine(..) => LayerMode::Affine,
            Fit::Interp(_) => LayerMode::Interpolate,
        })
        .collect();
    Ok(Placed { positions, modes })
}
