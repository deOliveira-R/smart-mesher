//! The kernels: `ogrid_disk`, `ogrid_annulus_quad`, `ring` and `polar_disk`. Pure functions on
//! counts and reals; no topology types. All output is deterministic (fixed loop order, no hashing).

use crate::mesh::{OgridError, PlanarMesh, Result, check_common, grading, polar_pt};
use std::f64::consts::{FRAC_PI_4, PI};

fn lerp(a: [f64; 2], b: [f64; 2], t: f64) -> [f64; 2] {
    [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]
}

/// Fill the four collar blocks (sides `e = 0..4`, `m[e]` intervals, `k` layers) between an inner
/// and an outer node loop that already exist. Sides run counter-clockwise; the outer loop is to
/// the right of travel. Layer `l` of side `e` at `i` lies at `inner + t_l (outer - inner)`.
fn fill_collar(
    nodes: &mut Vec<[f64; 2]>,
    quads: &mut Vec<[u32; 4]>,
    m: [usize; 4],
    k: usize,
    t: &[f64],
    inner: &dyn Fn(usize, usize) -> u32,
    outer: &dyn Fn(usize, usize) -> u32,
) {
    let place = |nodes: &[[f64; 2]], e: usize, i: usize, l: usize| {
        lerp(nodes[inner(e, i) as usize], nodes[outer(e, i) as usize], t[l])
    };
    // Radial lines (shared by the two sides meeting at corner e), then side interiors.
    let mut radial: Vec<Vec<u32>> = Vec::with_capacity(4);
    for e in 0..4 {
        let mut line = vec![0u32; k + 1];
        for l in 1..k {
            line[l] = nodes.len() as u32;
            let p = place(nodes, e, 0, l);
            nodes.push(p);
        }
        radial.push(line);
    }
    let mut base = [0u32; 4];
    for e in 0..4 {
        base[e] = nodes.len() as u32;
        for i in 1..m[e] {
            for l in 1..k {
                let p = place(nodes, e, i, l);
                nodes.push(p);
            }
        }
    }
    let id = |e: usize, i: usize, l: usize| -> u32 {
        if l == 0 {
            inner(e, i)
        } else if l == k {
            outer(e, i)
        } else if i == 0 {
            radial[e][l]
        } else if i == m[e] {
            radial[(e + 1) % 4][l]
        } else {
            base[e] + ((i - 1) * (k - 1) + (l - 1)) as u32
        }
    };
    for e in 0..4 {
        for i in 0..m[e] {
            for l in 0..k {
                quads.push([id(e, i, l), id(e, i, l + 1), id(e, i + 1, l + 1), id(e, i + 1, l)]);
            }
        }
    }
}

/// Jacobi Laplacian smoothing of the nodes not flagged fixed; `iters` sweeps, fixed order.
fn smooth(mesh: &mut PlanarMesh, fixed: &[bool], iters: usize) {
    let n = mesh.nodes.len();
    let mut nbr: Vec<Vec<u32>> = vec![Vec::new(); n];
    for (a, b) in mesh.quads.iter().flat_map(|q| (0..4).map(move |i| (q[i], q[(i + 1) % 4]))) {
        nbr[a as usize].push(b);
        nbr[b as usize].push(a);
    }
    for l in &mut nbr {
        l.sort_unstable();
        l.dedup();
    }
    for _ in 0..iters {
        let old = mesh.nodes.clone();
        for v in 0..n {
            if fixed[v] || nbr[v].is_empty() {
                continue;
            }
            let (mut sx, mut sy) = (0.0, 0.0);
            for &w in &nbr[v] {
                sx += old[w as usize][0];
                sy += old[w as usize][1];
            }
            let c = nbr[v].len() as f64;
            mesh.nodes[v] = [sx / c, sy / c];
        }
    }
}

/// Parameters of [`ogrid_disk`].
#[derive(Clone, Debug, PartialEq)]
pub struct DiskParams {
    /// Circle nodes; must equal `2 (a + b)`.
    pub n: usize,
    /// Central block intervals along the first and third sides (`a >= 1`).
    pub a: usize,
    /// Central block intervals along the second and fourth sides (`b >= 1`).
    pub b: usize,
    /// Radial layers of every collar block (`k >= 1`).
    pub k: usize,
    /// Outermost over innermost collar layer thickness (1 = uniform).
    pub ratio: f64,
    /// Angle in degrees of the first corner and of circle node 0 (node `j` is at `phase + 360 j / n`).
    /// 45 with `a == b` puts the corners on the diagonals.
    pub phase_deg: f64,
    /// Circle radius.
    pub radius: f64,
    /// Central block corner radius over the circle radius, in (0, 1).
    pub core: f64,
    /// Laplacian smoothing sweeps over the free nodes. Smoothing also relaxes the grading, so use 0
    /// with `ratio != 1`.
    pub smooth_iters: usize,
}

impl DiskParams {
    /// Defaults: unit radius, uniform layers, 45 degrees, `core` 0.5, no smoothing.
    pub fn new(n: usize, a: usize, b: usize, k: usize) -> Self {
        Self { n, a, b, k, ratio: 1.0, phase_deg: 45.0, radius: 1.0, core: 0.5, smooth_iters: 0 }
    }

    /// The balanced split of `n` (even, >= 4): `a = b` when `n % 4 == 0`, else `b = a + 1`.
    pub fn balanced(n: usize, k: usize) -> Self {
        let h = n / 2;
        Self::new(n, h / 2, h - h / 2, k)
    }
}

/// Five-block O-grid of a disk: a central `a x b` block plus four collar blocks of `k` layers.
/// Quad count `a b + k n`; four valence-3 interior nodes (the central block corners). Node order:
/// circle nodes `0..n`, then the central block (`(a+1)(b+1)` nodes, node `(i, j)` at `j (a+1) + i`),
/// then collar radial lines and side interiors.
pub fn ogrid_disk(p: &DiskParams) -> Result<PlanarMesh> {
    let (n, a, b, k) = (p.n, p.a, p.b, p.k);
    check_common(k, p.ratio, p.radius)?;
    if a == 0 || b == 0 || n != 2 * (a + b) {
        return Err(OgridError::Counts(format!("need a, b >= 1 and n = 2(a+b), got n={n} a={a} b={b}")));
    }
    if !(p.core.is_finite() && p.core > 0.0 && p.core < 1.0) || !p.phase_deg.is_finite() {
        return Err(OgridError::Param("core must be in (0,1) and phase finite".into()));
    }
    let phi0 = p.phase_deg.to_radians();
    let dphi = 2.0 * PI / n as f64;
    let mut nodes: Vec<[f64; 2]> = (0..n).map(|j| polar_pt(p.radius, phi0 + dphi * j as f64)).collect();
    // Circle corners at indices 0, a, a+b, 2a+b; central corners at `core` times them.
    let off = [0, a, a + b, 2 * a + b];
    let q: Vec<[f64; 2]> = off.iter().map(|&o| [nodes[o][0] * p.core, nodes[o][1] * p.core]).collect();
    let c0 = nodes.len() as u32;
    let cid = |i: usize, j: usize| c0 + (j * (a + 1) + i) as u32;
    for j in 0..=b {
        let (l, r) = (lerp(q[0], q[3], j as f64 / b as f64), lerp(q[1], q[2], j as f64 / b as f64));
        for i in 0..=a {
            nodes.push(lerp(l, r, i as f64 / a as f64));
        }
    }
    let mut quads = Vec::new();
    for j in 0..b {
        for i in 0..a {
            quads.push([cid(i, j), cid(i + 1, j), cid(i + 1, j + 1), cid(i, j + 1)]);
        }
    }
    let m = [a, b, a, b];
    let inner = |e: usize, i: usize| match e {
        0 => cid(i, 0),
        1 => cid(a, i),
        2 => cid(a - i, b),
        _ => cid(0, b - i),
    };
    let outer = |e: usize, i: usize| ((off[e] + i) % n) as u32;
    let t = grading(k, p.ratio);
    fill_collar(&mut nodes, &mut quads, m, k, &t, &inner, &outer);
    let mut mesh = PlanarMesh { nodes, quads, polys: Vec::new() };
    if p.smooth_iters > 0 {
        let mut fixed = vec![false; mesh.nodes.len()];
        fixed[..n].iter_mut().for_each(|f| *f = true);
        smooth(&mut mesh, &fixed, p.smooth_iters);
    }
    Ok(mesh)
}

/// Parameters of [`ogrid_annulus_quad`].
#[derive(Clone, Debug, PartialEq)]
pub struct AnnulusQuadParams {
    /// Intervals of the four square sides, counter-clockwise from the top side; the circle has
    /// `n = m[0] + m[1] + m[2] + m[3]` nodes.
    pub m: [usize; 4],
    /// Radial layers (`k >= 1`).
    pub k: usize,
    /// Outermost (square side) over innermost (circle) layer thickness.
    pub ratio: f64,
    /// Angle in degrees of circle node 0, the first block corner. The square corner of side 0's
    /// start is at 45 degrees, so 45 puts the radial lines on the diagonals.
    pub phase_deg: f64,
    /// Circle radius.
    pub radius: f64,
    /// Square side length; must exceed `2 radius`.
    pub side: f64,
}

/// An annulus between a circle and a concentric axis-aligned square: four trapezoid blocks,
/// radial lines from the square corners. Quad count `k n`. Node order: circle `0..n`, the square
/// loop (`n` nodes, counter-clockwise from the corner at 45 degrees), then the interior layers.
///
/// # Errors
/// Circle nodes are equally spaced in angle but square nodes are equally spaced along each
/// side, so unequal `m` entries (or a circle close to the square) can twist the radial lines.
/// After construction every quad corner is checked; [`OgridError::Inverted`] is returned when any
/// scaled Jacobian is `<= 0` or any node is non-finite. Balanced `m` (all equal) with
/// `phase_deg` = 45 (mod 360) and the circle strictly inside the square never fails; other phases
/// twist the radial lines and may return `Inverted`.
pub fn ogrid_annulus_quad(p: &AnnulusQuadParams) -> Result<PlanarMesh> {
    check_common(p.k, p.ratio, p.radius)?;
    let n: usize = p.m.iter().sum();
    if p.m.contains(&0) {
        return Err(OgridError::Counts("every square side needs >= 1 interval".into()));
    }
    if !(p.side.is_finite() && p.side > 2.0 * p.radius) || !p.phase_deg.is_finite() {
        return Err(OgridError::Param("side must exceed the circle diameter and phase must be finite".into()));
    }
    let dphi = 2.0 * PI / n as f64;
    let phi0 = p.phase_deg.to_radians();
    let mut nodes: Vec<[f64; 2]> = (0..n).map(|j| polar_pt(p.radius, phi0 + dphi * j as f64)).collect();
    let h = p.side / 2.0;
    let corner = |e: usize| polar_pt(h * std::f64::consts::SQRT_2, FRAC_PI_4 + 0.5 * PI * e as f64);
    let mut off = [0usize; 4];
    for e in 1..4 {
        off[e] = off[e - 1] + p.m[e - 1];
    }
    // The square loop: node (e, i) at `off[e] + i`.
    for e in 0..4 {
        for i in 0..p.m[e] {
            nodes.push(lerp(corner(e), corner((e + 1) % 4), i as f64 / p.m[e] as f64));
        }
    }
    let inner = |e: usize, i: usize| ((off[e] + i) % n) as u32;
    let outer = |e: usize, i: usize| (n + (off[e] + i) % n) as u32;
    let t = grading(p.k, p.ratio);
    let mut quads = Vec::new();
    fill_collar(&mut nodes, &mut quads, p.m, p.k, &t, &inner, &outer);
    let mesh = PlanarMesh { nodes, quads, polys: Vec::new() };
    // Unequal side counts twist the radial lines (circle nodes are equally spaced in angle,
    // square nodes along each side); never return a tangled mesh.
    let sj = mesh.min_scaled_jacobian();
    if !mesh.nodes.iter().all(|q| q[0].is_finite() && q[1].is_finite()) || sj.is_nan() || sj <= 0.0 {
        return Err(OgridError::Inverted(format!("m={:?}, min scaled Jacobian {sj}", p.m)));
    }
    Ok(mesh)
}

/// Parameters of [`ring`].
#[derive(Clone, Debug, PartialEq)]
pub struct RingParams {
    /// Nodes around each circle (`n >= 3`).
    pub n: usize,
    /// Radial layers (`k >= 1`).
    pub k: usize,
    /// Inner radius.
    pub r_in: f64,
    /// Outer radius (> `r_in`).
    pub r_out: f64,
    /// Outermost over innermost layer thickness.
    pub ratio: f64,
    /// Angle in degrees of node 0 of every circle.
    pub phase_deg: f64,
}

/// A structured `n x k` ring between two concentric circles. Quad count `k n`; node `(l, j)` is
/// `l n + j` (layer `l = 0` is the inner circle). Euler characteristic 0.
pub fn ring(p: &RingParams) -> Result<PlanarMesh> {
    check_common(p.k, p.ratio, p.r_in)?;
    if p.n < 3 || !(p.r_out.is_finite() && p.r_out > p.r_in) || !p.phase_deg.is_finite() {
        return Err(OgridError::Counts("ring needs n >= 3, r_out > r_in and a finite phase".into()));
    }
    let t = grading(p.k, p.ratio);
    let (phi0, dphi) = (p.phase_deg.to_radians(), 2.0 * PI / p.n as f64);
    let mut nodes = Vec::with_capacity((p.k + 1) * p.n);
    for l in 0..=p.k {
        let r = p.r_in + t[l] * (p.r_out - p.r_in);
        nodes.extend((0..p.n).map(|j| polar_pt(r, phi0 + dphi * j as f64)));
    }
    let id = |l: usize, j: usize| (l * p.n + j % p.n) as u32;
    let mut quads = Vec::with_capacity(p.k * p.n);
    for l in 0..p.k {
        for j in 0..p.n {
            quads.push([id(l, j), id(l + 1, j), id(l + 1, j + 1), id(l, j + 1)]);
        }
    }
    Ok(PlanarMesh { nodes, quads, polys: Vec::new() })
}

/// Parameters of [`polar_disk`].
#[derive(Clone, Debug, PartialEq)]
pub struct PolarParams {
    /// Nodes per ring and sides of the central polygon (`n >= 3`).
    pub n: usize,
    /// Rings of quads around the polygon (`k >= 1`).
    pub k: usize,
    /// Outermost over innermost ring thickness.
    pub ratio: f64,
    /// Angle in degrees of node 0 of every ring.
    pub phase_deg: f64,
    /// Circle radius.
    pub radius: f64,
    /// Radius of the central polygon over the circle radius; `None` is `1 / (k + 1)`, which makes
    /// every ring as thick as the polygon radius when `ratio` is 1.
    pub core: Option<f64>,
}

impl PolarParams {
    /// Defaults: unit radius, uniform rings, phase 0, default core.
    pub fn new(n: usize, k: usize) -> Self {
        Self { n, k, ratio: 1.0, phase_deg: 0.0, radius: 1.0, core: None }
    }
}

/// Polar disk: one central n-gon and `k` rings of `n` quads (`k n` quads). Node `(l, j)` is
/// `l n + j`, ring `l = k` is the circle (nodes `k n ..`), so the circle nodes are the LAST `n`.
pub fn polar_disk(p: &PolarParams) -> Result<PlanarMesh> {
    check_common(p.k, p.ratio, p.radius)?;
    if p.n < 3 || !p.phase_deg.is_finite() {
        return Err(OgridError::Counts("polar needs n >= 3 and a finite phase".into()));
    }
    let core = p.core.unwrap_or(1.0 / (p.k as f64 + 1.0));
    if !(core.is_finite() && core > 0.0 && core < 1.0) {
        return Err(OgridError::Param("core must be in (0,1)".into()));
    }
    let t = grading(p.k, p.ratio);
    let (phi0, dphi) = (p.phase_deg.to_radians(), 2.0 * PI / p.n as f64);
    let mut nodes = Vec::with_capacity((p.k + 1) * p.n);
    for l in 0..=p.k {
        let r = p.radius * (core + (1.0 - core) * t[l]);
        nodes.extend((0..p.n).map(|j| polar_pt(r, phi0 + dphi * j as f64)));
    }
    let id = |l: usize, j: usize| (l * p.n + j % p.n) as u32;
    let mut quads = Vec::with_capacity(p.k * p.n);
    for l in 0..p.k {
        for j in 0..p.n {
            quads.push([id(l, j), id(l + 1, j), id(l + 1, j + 1), id(l, j + 1)]);
        }
    }
    let poly: Vec<u32> = (0..p.n).map(|j| id(0, j)).collect();
    Ok(PlanarMesh { nodes, quads, polys: vec![poly] })
}
