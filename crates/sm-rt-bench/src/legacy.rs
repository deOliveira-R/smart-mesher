//! The W3a legacy cases C1, C2, C3 (`sm-rt-capi/bench/gen_case.py`), regenerated natively.
//!
//! Only IEEE-exact operations are used, so the bytes are identical on every platform and equal
//! the files of the W3a harness (`expected_sha256.txt` there is the cross-check, asserted in the
//! tests of this crate when the W3a data is reachable by path).

use crate::error::{BenchError, Result};
use crate::formats::{Facets, Rays};
use crate::rng::SplitMix64;

/// Box half width of the legacy cases.
pub const HALF: f64 = 2.0;

fn unit3(p: [f64; 3]) -> [f64; 3] {
    let n = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
    [p[0] / n, p[1] / n, p[2] / n]
}

/// Icosphere with outward normals: `(vertices, triangles)`.
pub fn icosphere(radius: f64, subdivisions: u32) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
    let t = (1.0 + 5.0f64.sqrt()) / 2.0;
    let raw = [
        [-1.0, t, 0.0],
        [1.0, t, 0.0],
        [-1.0, -t, 0.0],
        [1.0, -t, 0.0],
        [0.0, -1.0, t],
        [0.0, 1.0, t],
        [0.0, -1.0, -t],
        [0.0, 1.0, -t],
        [t, 0.0, -1.0],
        [t, 0.0, 1.0],
        [-t, 0.0, -1.0],
        [-t, 0.0, 1.0],
    ];
    let mut verts: Vec<[f64; 3]> = raw.iter().map(|p| unit3(*p)).collect();
    let mut tris: Vec<[u32; 3]> = vec![
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];
    for _ in 0..subdivisions {
        let mut cache: std::collections::BTreeMap<(u32, u32), u32> = std::collections::BTreeMap::new();
        let mut mid = |verts: &mut Vec<[f64; 3]>, a: u32, b: u32| -> u32 {
            let key = if a < b { (a, b) } else { (b, a) };
            *cache.entry(key).or_insert_with(|| {
                let (pa, pb) = (verts[a as usize], verts[b as usize]);
                let m = [(pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0, (pa[2] + pb[2]) / 2.0];
                verts.push(unit3(m));
                (verts.len() - 1) as u32
            })
        };
        let mut new = Vec::with_capacity(tris.len() * 4);
        for [a, b, c] in tris {
            let ab = mid(&mut verts, a, b);
            let bc = mid(&mut verts, b, c);
            let ca = mid(&mut verts, c, a);
            new.extend_from_slice(&[[a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]]);
        }
        tris = new;
    }
    let verts = verts.iter().map(|v| [radius * v[0], radius * v[1], radius * v[2]]).collect();
    (verts, tris)
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(u: [f64; 3], v: [f64; 3]) -> [f64; 3] {
    [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
}

/// Smallest distance from the origin to a triangle plane.
pub fn inscribed_radius(verts: &[[f64; 3]], tris: &[[u32; 3]]) -> f64 {
    let mut best = f64::INFINITY;
    for t in tris {
        let (pa, pb, pc) = (verts[t[0] as usize], verts[t[1] as usize], verts[t[2] as usize]);
        let n = cross(sub(pb, pa), sub(pc, pa));
        let norm = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        best = best.min((n[0] * pa[0] + n[1] * pa[1] + n[2] * pa[2]).abs() / norm);
    }
    best
}

/// Axis-aligned cube `[-h, h]^3` (12 triangles, outward normals) centred at `c`.
pub fn cube(c: [f64; 3], h: f64) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
    boxed([c[0] - h, c[1] - h, c[2] - h], [c[0] + h, c[1] + h, c[2] + h])
}

/// Axis-aligned box `[lo, hi]` (12 triangles, outward normals).
pub fn boxed(lo: [f64; 3], hi: [f64; 3]) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
    let v = vec![
        [lo[0], lo[1], lo[2]],
        [hi[0], lo[1], lo[2]],
        [hi[0], hi[1], lo[2]],
        [lo[0], hi[1], lo[2]],
        [lo[0], lo[1], hi[2]],
        [hi[0], lo[1], hi[2]],
        [hi[0], hi[1], hi[2]],
        [lo[0], hi[1], hi[2]],
    ];
    let t = vec![
        [0, 3, 2],
        [0, 2, 1],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [2, 3, 7],
        [2, 7, 6],
        [1, 2, 6],
        [1, 6, 5],
        [3, 0, 4],
        [3, 4, 7],
    ];
    (v, t)
}

/// A legacy case.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Legacy {
    /// Icosphere shell (r 1, subdivision 5) in the box.
    C1,
    /// Five nested icospheres in the box.
    C2,
    /// Box with a spherical void.
    C3,
}

impl Legacy {
    /// All three.
    pub const ALL: [Legacy; 3] = [Legacy::C1, Legacy::C2, Legacy::C3];

    /// The case name used for seeding and file names.
    pub fn name(self) -> &'static str {
        match self {
            Legacy::C1 => "C1",
            Legacy::C2 => "C2",
            Legacy::C3 => "C3",
        }
    }

    /// Parse a case name.
    pub fn parse(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|c| c.name() == s)
    }
}

/// The facet soup, the sphere radii and the inscribed radii of a legacy case.
pub struct LegacyGeom {
    /// The facets.
    pub facets: Facets,
    /// True sphere radii, innermost first.
    pub radii: Vec<f64>,
    /// Inscribed radii (same order).
    pub inradii: Vec<f64>,
}

/// Build the facets of `case`.
pub fn build(case: Legacy) -> Result<LegacyGeom> {
    let mut f = Facets::default();
    let (radii, inradii);
    match case {
        Legacy::C1 => {
            let (v, t) = icosphere(1.0, 5);
            inradii = vec![inscribed_radius(&v, &t)];
            f.add(&v, &t, 0, 1, false)?;
            let (v, t) = cube([0.0; 3], HALF);
            f.add(&v, &t, 1, -1, false)?;
            radii = vec![1.0];
        }
        Legacy::C2 => {
            radii = vec![0.2, 0.4, 0.6, 0.8, 1.0];
            let mut inr = Vec::new();
            for (k, r) in radii.iter().enumerate() {
                let (v, t) = icosphere(*r, 4);
                f.add(&v, &t, k as i32, k as i32 + 1, false)?;
                inr.push(inscribed_radius(&v, &t));
            }
            inradii = inr;
            let (v, t) = cube([0.0; 3], HALF);
            f.add(&v, &t, radii.len() as i32, -1, false)?;
        }
        Legacy::C3 => {
            let (v, t) = cube([0.0; 3], HALF);
            f.add(&v, &t, 0, -1, false)?;
            let (v, t) = icosphere(1.0, 5);
            f.add(&v, &t, 0, -1, true)?;
            inradii = vec![inscribed_radius(&v, &t)];
            radii = vec![1.0];
        }
    }
    Ok(LegacyGeom { facets: f, radii, inradii })
}

/// Start region of a point, or `None` when it is not a valid start (inside the sphere band of
/// the faceted model, or inside the C3 void).
pub fn start_region(case: Legacy, g: &LegacyGeom, p: [f64; 3]) -> Option<i32> {
    let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
    if g.inradii.iter().zip(&g.radii).any(|(rin, rad)| *rin < r && r < *rad) {
        return None;
    }
    if case == Legacy::C3 {
        return if r >= g.radii[0] { Some(0) } else { None };
    }
    Some(g.radii.iter().filter(|q| r >= **q).count() as i32)
}

/// `n` rays of a legacy case (same stream as `gen_case.py`).
pub fn rays(case: Legacy, g: &LegacyGeom, n: usize) -> Result<Rays> {
    if n > u32::MAX as usize {
        return Err(BenchError::Usage("too many rays".into()));
    }
    let mut rng = SplitMix64::from_name(case.name());
    let mut out = Rays::default();
    while out.len() < n {
        let p = [(2.0 * rng.unit() - 1.0) * HALF, (2.0 * rng.unit() - 1.0) * HALF, (2.0 * rng.unit() - 1.0) * HALF];
        let d = rng.direction();
        if let Some(reg) = start_region(case, g, p) {
            out.push(p, d, reg);
        }
    }
    Ok(out)
}
