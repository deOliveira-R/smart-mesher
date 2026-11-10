//! Planar mesh output of the kernels, and the metrics the acceptance tests and the adapter use.

use std::f64::consts::PI;

/// A planar mesh in the plane z = 0, centred on the origin. Quads and polygons are counter-clockwise.
#[derive(Clone, Debug, PartialEq)]
pub struct PlanarMesh {
    /// Node coordinates; each kernel documents its node order (circle node `j` is at angle `phase + 2 pi j / n`).
    pub nodes: Vec<[f64; 2]>,
    /// Quads, counter-clockwise.
    pub quads: Vec<[u32; 4]>,
    /// General polygons (the `polar` centre n-gon), counter-clockwise. Empty for the other kernels.
    pub polys: Vec<Vec<u32>>,
}

/// Why a kernel refused its parameters.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum OgridError {
    /// A count is too small or inconsistent (the message says which).
    #[error("invalid counts: {0}")]
    Counts(String),
    /// A real parameter is not finite or not in range.
    #[error("invalid parameter: {0}")]
    Param(String),
    /// The construction produced a tangled mesh: a quad corner has scaled Jacobian <= 0 or a node
    /// is not finite (the message names the worst value).
    #[error("inverted or non-finite mesh: {0}")]
    Inverted(String),
}

/// Result alias of the kernels.
pub type Result<T> = std::result::Result<T, OgridError>;

fn area_of(nodes: &[[f64; 2]], ids: &[u32]) -> f64 {
    let mut s = 0.0;
    for i in 0..ids.len() {
        let p = nodes[ids[i] as usize];
        let q = nodes[ids[(i + 1) % ids.len()] as usize];
        s += p[0] * q[1] - q[0] * p[1];
    }
    0.5 * s
}

impl PlanarMesh {
    /// Total area (sum of signed cell areas in a fixed order).
    pub fn area(&self) -> f64 {
        let q: f64 = self.quads.iter().map(|c| area_of(&self.nodes, c)).sum();
        let p: f64 = self.polys.iter().map(|c| area_of(&self.nodes, c)).sum();
        q + p
    }

    /// Number of cells (quads plus polygons).
    pub fn cell_count(&self) -> usize {
        self.quads.len() + self.polys.len()
    }

    fn cell_edges(&self) -> Vec<(u32, u32)> {
        let mut e = Vec::new();
        let loops = self.quads.iter().map(|q| q.as_slice()).chain(self.polys.iter().map(|p| p.as_slice()));
        for c in loops {
            for i in 0..c.len() {
                let (a, b) = (c[i], c[(i + 1) % c.len()]);
                e.push((a.min(b), a.max(b)));
            }
        }
        e.sort_unstable();
        e
    }

    /// Euler characteristic V - E + F (V counts the nodes used by a cell).
    pub fn euler(&self) -> i64 {
        let mut used = vec![false; self.nodes.len()];
        for &v in self.quads.iter().flatten().chain(self.polys.iter().flatten()) {
            used[v as usize] = true;
        }
        let v = used.iter().filter(|u| **u).count() as i64;
        let mut e = self.cell_edges();
        e.dedup();
        v - e.len() as i64 + self.cell_count() as i64
    }

    /// Per node: the number of incident edges, and whether the node is on the mesh boundary
    /// (touches an edge that has exactly one cell).
    pub fn valence_and_boundary(&self) -> (Vec<u32>, Vec<bool>) {
        let all = self.cell_edges();
        let mut val = vec![0u32; self.nodes.len()];
        let mut bnd = vec![false; self.nodes.len()];
        let mut i = 0;
        while i < all.len() {
            let mut j = i;
            while j < all.len() && all[j] == all[i] {
                j += 1;
            }
            let (a, b) = all[i];
            val[a as usize] += 1;
            val[b as usize] += 1;
            if j - i == 1 {
                bnd[a as usize] = true;
                bnd[b as usize] = true;
            }
            i = j;
        }
        (val, bnd)
    }

    /// Scaled Jacobian at corner `c` of quad `q`: the sine of the interior angle there (cross of
    /// the two unit edges), negative for an inverted corner.
    pub fn corner_sj(&self, q: &[u32; 4], c: usize) -> f64 {
        let p = self.nodes[q[c] as usize];
        let a = self.nodes[q[(c + 1) % 4] as usize];
        let b = self.nodes[q[(c + 3) % 4] as usize];
        let (u, v) = ([a[0] - p[0], a[1] - p[1]], [b[0] - p[0], b[1] - p[1]]);
        let (lu, lv) = (u[0].hypot(u[1]), v[0].hypot(v[1]));
        if lu == 0.0 || lv == 0.0 {
            return 0.0;
        }
        (u[0] * v[1] - u[1] * v[0]) / (lu * lv)
    }

    /// Minimum scaled Jacobian over all quad corners (1 = square, <= 0 = degenerate or inverted).
    pub fn min_scaled_jacobian(&self) -> f64 {
        self.quads.iter().flat_map(|q| (0..4).map(move |c| self.corner_sj(q, c))).fold(f64::INFINITY, f64::min)
    }

    /// Minimum scaled Jacobian over the quad corners located at node `node`.
    pub fn min_sj_at_node(&self, node: u32) -> f64 {
        let mut m = f64::INFINITY;
        for q in &self.quads {
            for c in 0..4 {
                if q[c] == node {
                    m = m.min(self.corner_sj(q, c));
                }
            }
        }
        m
    }

    /// Minimum signed quad area (negative means an inverted quad).
    pub fn min_quad_area(&self) -> f64 {
        self.quads.iter().map(|q| area_of(&self.nodes, q)).fold(f64::INFINITY, f64::min)
    }

    /// FNV-1a hash over the bit patterns of the nodes and the cell lists (for determinism tests).
    pub fn hash(&self) -> u64 {
        let mut h = 0xcbf2_9ce4_8422_2325u64;
        let mut eat = |x: u64| {
            for b in x.to_le_bytes() {
                h ^= u64::from(b);
                h = h.wrapping_mul(0x0100_0000_01b3);
            }
        };
        eat(self.nodes.len() as u64);
        for p in &self.nodes {
            eat(p[0].to_bits());
            eat(p[1].to_bits());
        }
        for q in &self.quads {
            q.iter().for_each(|&v| eat(u64::from(v)));
        }
        for p in &self.polys {
            eat(p.len() as u64);
            p.iter().for_each(|&v| eat(u64::from(v)));
        }
        h
    }
}

/// Area of the regular n-gon inscribed in a circle of radius `r`.
pub fn inscribed_area(n: usize, r: f64) -> f64 {
    0.5 * n as f64 * r * r * (2.0 * PI / n as f64).sin()
}

/// Layer positions `t_0 = 0 ..= t_k = 1` with geometric thickness ratio `ratio` (outermost layer
/// thickness over innermost; 1 is uniform).
pub(crate) fn grading(k: usize, ratio: f64) -> Vec<f64> {
    let w: Vec<f64> = (0..k).map(|l| if k == 1 { 1.0 } else { ratio.powf(l as f64 / (k - 1) as f64) }).collect();
    let total: f64 = w.iter().sum();
    let mut t = Vec::with_capacity(k + 1);
    let mut acc = 0.0;
    t.push(0.0);
    for x in &w[..k - 1] {
        acc += x;
        t.push(acc / total);
    }
    t.push(1.0);
    t
}

/// Accepted layer ratio range: `1 / RATIO_MAX ..= RATIO_MAX`.
pub const RATIO_MAX: f64 = 1e6;
/// Accepted `|phase|` in degrees.
pub const PHASE_MAX_DEG: f64 = 3600.0;

pub(crate) fn check_common(k: usize, ratio: f64, r: f64, phase_deg: f64) -> Result<()> {
    if k == 0 {
        return Err(OgridError::Counts("k (layers) must be >= 1".into()));
    }
    if !(ratio.is_finite() && (1.0 / RATIO_MAX..=RATIO_MAX).contains(&ratio)) {
        return Err(OgridError::Param(format!("ratio {ratio} must be in [{:e}, {:e}]", 1.0 / RATIO_MAX, RATIO_MAX)));
    }
    if !(phase_deg.is_finite() && phase_deg.abs() <= PHASE_MAX_DEG) {
        return Err(OgridError::Param(format!(
            "phase {phase_deg} must be finite with |phase| <= {PHASE_MAX_DEG} degrees"
        )));
    }
    if !(r.is_finite() && r > 0.0) {
        return Err(OgridError::Param(format!("radius {r} must be finite and > 0")));
    }
    Ok(())
}

pub(crate) fn polar_pt(r: f64, ang: f64) -> [f64; 2] {
    [r * ang.cos(), r * ang.sin()]
}

/// Final guard of every kernel: finite nodes and every scaled Jacobian > 0, else `Inverted`.
pub(crate) fn guard(mesh: PlanarMesh, what: &str) -> Result<PlanarMesh> {
    let sj = mesh.min_scaled_jacobian();
    if !mesh.nodes.iter().all(|q| q[0].is_finite() && q[1].is_finite()) || sj.is_nan() || sj <= 0.0 {
        return Err(OgridError::Inverted(format!("{what}, min scaled Jacobian {sj}")));
    }
    Ok(mesh)
}
