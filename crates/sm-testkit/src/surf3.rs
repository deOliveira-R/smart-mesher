//! Closed triangulated surfaces as pure data, with exact reference values.
//!
//! Boundary-recovery and tetrahedralization tests need a watertight triangle surface with a
//! known enclosed volume but should not depend on a surface mesher. A [`SurfaceFixture`] is a
//! point list, a triangle list and one [`VolumeSurface`] per enclosed volume.
//!
//! **Orientation.** `tris` is stored in the natural orientation of each solid piece (outward from
//! the solid it bounds). A [`VolumeSurface`] picks triangles with a flip flag: `flip == true`
//! means the stored winding is opposite to the volume's outward direction. A shared interface
//! triangle is therefore listed once and flipped by the volume on the other side, and a cavity
//! surface (stored outward from the cavity's cube) is flipped by the surrounding volume.
//!
//! **Enclosed volume.** `enclosed_volume` is the volume of the *polyhedron* by the divergence
//! theorem, `1/6 sum p0 . (p1 x p2)` over the oriented triangles about a bounding-box centre,
//! with a compensated sum. For flat-faced fixtures it is exact up to rounding; for curved
//! fixtures it is less than the smooth limit (documented per constructor).

use std::collections::BTreeMap;

/// A closed surface soup with its enclosed volumes.
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceFixture {
    /// Fixture name.
    pub name: &'static str,
    /// Points.
    pub points: Vec<[f64; 3]>,
    /// Triangles (point indices), each in its solid's outward orientation (see the module docs).
    pub tris: Vec<[u32; 3]>,
    /// The enclosed volumes. Empty for the open negative case.
    pub volumes: Vec<VolumeSurface>,
}

/// The boundary of one enclosed volume.
#[derive(Clone, Debug, PartialEq)]
pub struct VolumeSurface {
    /// Name of the volume.
    pub name: String,
    /// Boundary triangles as `(index into SurfaceFixture::tris, flip)`; `flip` means the stored
    /// winding points into this volume.
    pub tris: Vec<(u32, bool)>,
    /// Volume of the polyhedron bounded by the oriented triangles (divergence theorem).
    pub enclosed_volume: f64,
    /// Number of closed shells (2 for a volume with one cavity).
    pub n_shells: usize,
}

impl SurfaceFixture {
    /// The triangles of volume `v`, wound outward from that volume. Empty if `v` is out of range.
    pub fn oriented_tris(&self, v: usize) -> Vec<[u32; 3]> {
        let Some(vol) = self.volumes.get(v) else { return Vec::new() };
        vol.tris
            .iter()
            .filter_map(|&(t, flip)| self.tris.get(t as usize).map(|&[a, b, c]| if flip { [a, c, b] } else { [a, b, c] }))
            .collect()
    }
}

/// SplitMix64: a tiny generator seeded from its argument.
#[derive(Clone, Debug)]
pub struct SplitMix64(pub u64);

impl SplitMix64 {
    /// The next 64 random bits.
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        mix64(self.0)
    }

    /// Uniform in `[-1, 1)` (53 random bits).
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 52) as f64 - 1.0
    }
}

/// The SplitMix64 output function: a bijective 64-bit mixer, handy as a stateless hash.
pub fn mix64(x: u64) -> u64 {
    let mut z = x;
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

/// Neumaier compensated sum.
fn kahan(xs: impl Iterator<Item = f64>) -> f64 {
    let (mut s, mut c) = (0.0f64, 0.0f64);
    for x in xs {
        let t = s + x;
        c += if s.abs() >= x.abs() { (s - t) + x } else { (x - t) + s };
        s = t;
    }
    s + c
}

/// Volume of the polyhedron bounded by `tris` (outward oriented), by the divergence theorem about
/// the bounding-box centre. `NaN` if a triangle names a missing point.
pub fn polyhedron_volume(points: &[[f64; 3]], tris: &[[u32; 3]]) -> f64 {
    if points.is_empty() {
        return 0.0;
    }
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in points {
        for a in 0..3 {
            lo[a] = lo[a].min(p[a]);
            hi[a] = hi[a].max(p[a]);
        }
    }
    let c = [0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (lo[2] + hi[2])];
    let mut terms = Vec::with_capacity(tris.len());
    for t in tris {
        let (Some(a), Some(b), Some(d)) = (points.get(t[0] as usize), points.get(t[1] as usize), points.get(t[2] as usize)) else {
            return f64::NAN;
        };
        let u = [a[0] - c[0], a[1] - c[1], a[2] - c[2]];
        let v = [b[0] - c[0], b[1] - c[1], b[2] - c[2]];
        let w = [d[0] - c[0], d[1] - c[1], d[2] - c[2]];
        terms.push((u[0] * (v[1] * w[2] - v[2] * w[1]) - u[1] * (v[0] * w[2] - v[2] * w[0]) + u[2] * (v[0] * w[1] - v[1] * w[0])) / 6.0);
    }
    kahan(terms.into_iter())
}

/// Volume of a ball of radius `r`: the smooth limit of [`icosphere`] (`4/3 pi r^3`).
pub fn ball_volume(r: f64) -> f64 {
    4.0 / 3.0 * core::f64::consts::PI * r * r * r
}

/// Volume of a torus: the smooth limit of [`tri_torus`] (`2 pi^2 R r^2`).
pub fn torus_volume(big_r: f64, small_r: f64) -> f64 {
    2.0 * core::f64::consts::PI * core::f64::consts::PI * big_r * small_r * small_r
}

/// `(name, oriented triangle list, shell count)` of a volume under construction.
type VolumeSpec = (String, Vec<(u32, bool)>, usize);

fn assemble(name: &'static str, points: Vec<[f64; 3]>, tris: Vec<[u32; 3]>, volumes: Vec<VolumeSpec>) -> SurfaceFixture {
    let mut fx = SurfaceFixture { name, points, tris, volumes: Vec::new() };
    fx.volumes = volumes.into_iter().map(|(name, tris, n_shells)| VolumeSurface { name, tris, enclosed_volume: 0.0, n_shells }).collect();
    for v in 0..fx.volumes.len() {
        fx.volumes[v].enclosed_volume = polyhedron_volume(&fx.points, &fx.oriented_tris(v));
    }
    fx
}

/// Points deduplicated by exact coordinates.
#[derive(Default)]
struct Soup {
    points: Vec<[f64; 3]>,
    index: BTreeMap<[u64; 3], u32>,
}

impl Soup {
    fn add(&mut self, p: [f64; 3]) -> u32 {
        let key = p.map(|x| (x + 0.0).to_bits());
        *self.index.entry(key).or_insert_with(|| {
            self.points.push(p);
            (self.points.len() - 1) as u32
        })
    }
}

/// The six faces (`-x,+x,-y,+y,-z,+z`) of the box `[min, min + size]`, each an `n x n` grid with
/// alternating diagonals, outward wound.
fn box_faces(soup: &mut Soup, n: u32, min: [f64; 3], size: [f64; 3]) -> [Vec<[u32; 3]>; 6] {
    let n = n.max(1) as usize;
    let coord = |l: [usize; 3]| [0, 1, 2].map(|a| min[a] + size[a] * l[a] as f64 / n as f64);
    let mut out: [Vec<[u32; 3]>; 6] = Default::default();
    for ax in 0..3 {
        let (u, v) = ((ax + 1) % 3, (ax + 2) % 3);
        for s in 0..2 {
            let tris = &mut out[2 * ax + s];
            for b in 0..n {
                for a in 0..n {
                    let mut q = [0u32; 4];
                    for (slot, (du, dv)) in [(0, 0), (1, 0), (1, 1), (0, 1)].into_iter().enumerate() {
                        let mut l = [0usize; 3];
                        l[ax] = s * n;
                        l[u] = a + du;
                        l[v] = b + dv;
                        q[slot] = soup.add(coord(l));
                    }
                    let pair = if (a + b) % 2 == 0 { [[q[0], q[1], q[2]], [q[0], q[2], q[3]]] } else { [[q[0], q[1], q[3]], [q[1], q[2], q[3]]] };
                    for t in pair {
                        tris.push(if s == 1 { t } else { [t[0], t[2], t[1]] });
                    }
                }
            }
        }
    }
    out
}

/// The unit cube `[0,1]^3` with an `n x n` grid per face split by alternating diagonals (`n` is
/// raised to at least 1): `6 n^2 + 2` points, `12 n^2` outward-wound triangles, one volume of
/// volume 1 and one shell. The 6 faces are consecutive blocks of `2 n^2` triangles in the order
/// `-x,+x,-y,+y,-z,+z`.
pub fn tri_cube(n: u32) -> SurfaceFixture {
    let mut soup = Soup::default();
    let faces = box_faces(&mut soup, n, [0.0; 3], [1.0; 3]);
    let tris: Vec<[u32; 3]> = faces.into_iter().flatten().collect();
    let all = (0..tris.len() as u32).map(|t| (t, false)).collect();
    assemble("tri_cube", soup.points, tris, vec![("cube".into(), all, 1)])
}

/// Same as [`tri_cube`] with the first triangle removed: the surface is open (one boundary
/// loop of 3 edges), so `volumes` is empty. The error-path input for "surface not closed".
pub fn open_cube_missing_tri(n: u32) -> SurfaceFixture {
    let mut fx = tri_cube(n);
    fx.name = "open_cube_missing_tri";
    fx.tris.remove(0);
    fx.volumes.clear();
    fx
}

/// A cube `[-1,1]^3` (side 2) with a cubic cavity `[-1/2,1/2]^3` (side 1), `n x n` per face on
/// both: one volume of volume `8 - 1 = 7` with two shells. The outer triangles come first
/// (`12 n^2`), then the cavity's (`12 n^2`), stored outward from the cavity's cube so the volume
/// lists them with `flip = true` (they then point into the cavity, away from the solid).
pub fn cube_with_cavity(n: u32) -> SurfaceFixture {
    let mut soup = Soup::default();
    let outer: Vec<[u32; 3]> = box_faces(&mut soup, n, [-1.0; 3], [2.0; 3]).into_iter().flatten().collect();
    let inner: Vec<[u32; 3]> = box_faces(&mut soup, n, [-0.5; 3], [1.0; 3]).into_iter().flatten().collect();
    let no = outer.len() as u32;
    let tris: Vec<[u32; 3]> = outer.into_iter().chain(inner).collect();
    let list = (0..tris.len() as u32).map(|t| (t, t >= no)).collect();
    assemble("cube_with_cavity", soup.points, tris, vec![("solid".into(), list, 2)])
}

/// Two unit cubes `[0,1]^3` and `[1,2] x [0,1]^2`, sharing one identical `n x n` interface
/// triangulation (same point ids and the same `2 n^2` triangle ids). Cube A's triangles come
/// first (`12 n^2`), then cube B's without its `-x` face (`10 n^2`). `volumes[0]` is A with all
/// its triangles; `volumes[1]` is B with its own triangles plus A's `+x` face triangles flipped.
/// Each volume is 1, the union 2.
pub fn two_cubes_shared(n: u32) -> SurfaceFixture {
    let mut soup = Soup::default();
    let a = box_faces(&mut soup, n, [0.0; 3], [1.0; 3]);
    let b = box_faces(&mut soup, n, [1.0, 0.0, 0.0], [1.0; 3]);
    let per_face = a[0].len() as u32;
    let mut tris: Vec<[u32; 3]> = a.into_iter().flatten().collect();
    let na = tris.len() as u32;
    tris.extend(b.into_iter().skip(1).flatten());
    let vol_a: Vec<(u32, bool)> = (0..na).map(|t| (t, false)).collect();
    // A's +x face is the second block of `per_face` triangles.
    let mut vol_b: Vec<(u32, bool)> = (per_face..2 * per_face).map(|t| (t, true)).collect();
    vol_b.extend((na..tris.len() as u32).map(|t| (t, false)));
    assemble("two_cubes_shared", soup.points, tris, vec![("a".into(), vol_a, 1), ("b".into(), vol_b, 1)])
}

/// Midpoint-subdivided icosahedron projected to radius `r`, `level` times: `10 * 4^level + 2`
/// points and `20 * 4^level` outward-wound triangles (level 3: 642 / 1280; level 4: 2562 /
/// 5120). One volume.
///
/// `enclosed_volume` is the volume of the polyhedron (divergence theorem), which is strictly
/// below the smooth limit `4/3 pi r^3` ([`ball_volume`]); the ratio is above 0.95 from level 3.
pub fn icosphere(r: f64, level: u32) -> SurfaceFixture {
    let (points, tris) = icosphere_data(r, level);
    let all = (0..tris.len() as u32).map(|t| (t, false)).collect();
    assemble("icosphere", points, tris, vec![("ball".into(), all, 1)])
}

fn icosphere_data(r: f64, level: u32) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
    let t = (1.0 + 5f64.sqrt()) / 2.0;
    let raw: [[f64; 3]; 12] = [
        [-1., t, 0.],
        [1., t, 0.],
        [-1., -t, 0.],
        [1., -t, 0.],
        [0., -1., t],
        [0., 1., t],
        [0., -1., -t],
        [0., 1., -t],
        [t, 0., -1.],
        [t, 0., 1.],
        [-t, 0., -1.],
        [-t, 0., 1.],
    ];
    let proj = |p: [f64; 3]| {
        let l = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
        [p[0] / l * r, p[1] / l * r, p[2] / l * r]
    };
    let mut points: Vec<[f64; 3]> = raw.iter().map(|p| proj(*p)).collect();
    let base: [[u32; 3]; 20] = [
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
    // Make every base face outward (the centroid and the normal agree).
    let mut tris: Vec<[u32; 3]> = base
        .iter()
        .map(|&[a, b, c]| {
            let (pa, pb, pc) = (points[a as usize], points[b as usize], points[c as usize]);
            let n = cross(sub(pb, pa), sub(pc, pa));
            let cen = [pa[0] + pb[0] + pc[0], pa[1] + pb[1] + pc[1], pa[2] + pb[2] + pc[2]];
            if dot(n, cen) >= 0.0 { [a, b, c] } else { [a, c, b] }
        })
        .collect();
    for _ in 0..level {
        let mut mids: BTreeMap<(u32, u32), u32> = BTreeMap::new();
        let mut next = Vec::with_capacity(tris.len() * 4);
        for &[a, b, c] in &tris {
            let mut mid = |p: u32, q: u32| -> u32 {
                *mids.entry((p.min(q), p.max(q))).or_insert_with(|| {
                    let (u, v) = (points[p as usize], points[q as usize]);
                    points.push(proj([u[0] + v[0], u[1] + v[1], u[2] + v[2]]));
                    (points.len() - 1) as u32
                })
            };
            let (ab, bc, ca) = (mid(a, b), mid(b, c), mid(c, a));
            next.extend([[a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]]);
        }
        tris = next;
    }
    (points, tris)
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// A torus about the `z` axis with major radius `big_r`, minor radius `small_r`, `nu` points around
/// the axis and `nv` around the tube (`nu, nv >= 3`, clamped): `nu nv` points, `2 nu nv`
/// outward-wound triangles, genus 1 (V - E + F = 0), one volume. `enclosed_volume` is the
/// polyhedron volume, below the smooth limit `2 pi^2 R r^2` ([`torus_volume`]); for
/// `(2, 0.7, 48, 24)` it is within 3% of it.
pub fn tri_torus(big_r: f64, small_r: f64, nu: u32, nv: u32) -> SurfaceFixture {
    let (nu, nv) = (nu.max(3), nv.max(3));
    let mut points = Vec::with_capacity((nu * nv) as usize);
    for i in 0..nu {
        let u = core::f64::consts::TAU * f64::from(i) / f64::from(nu);
        for j in 0..nv {
            let v = core::f64::consts::TAU * f64::from(j) / f64::from(nv);
            let rho = big_r + small_r * v.cos();
            points.push([rho * u.cos(), rho * u.sin(), small_r * v.sin()]);
        }
    }
    let id = |i: u32, j: u32| (i % nu) * nv + (j % nv);
    let mut tris = Vec::with_capacity((2 * nu * nv) as usize);
    for i in 0..nu {
        for j in 0..nv {
            let (a, b, c, d) = (id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1));
            tris.push([a, b, c]);
            tris.push([a, c, d]);
        }
    }
    if polyhedron_volume(&points, &tris) < 0.0 {
        for t in &mut tris {
            *t = [t[0], t[2], t[1]];
        }
    }
    let all = (0..tris.len() as u32).map(|t| (t, false)).collect();
    assemble("tri_torus", points, tris, vec![("torus".into(), all, 1)])
}

/// The Schonhardt polyhedron: a triangular prism whose top triangle (radius 1, height 1) is
/// twisted by 30 degrees against the bottom, with each side quadrilateral split along its reflex
/// diagonal. 6 points (bottom 0-2, top 3-5), 8 outward-wound triangles (bottom, top, then 2 per
/// side), non-convex, and it has no tetrahedralization without an interior Steiner point.
pub fn schonhardt() -> SurfaceFixture {
    let twist = core::f64::consts::PI / 6.0;
    let mut points = Vec::with_capacity(6);
    for (z, rot) in [(0.0, 0.0), (1.0, twist)] {
        for i in 0..3 {
            let a = core::f64::consts::TAU * f64::from(i) / 3.0 + rot;
            points.push([a.cos(), a.sin(), z]);
        }
    }
    let mut tris: Vec<[u32; 3]> = vec![[0, 2, 1], [3, 4, 5]];
    for i in 0..3u32 {
        let j = (i + 1) % 3;
        tris.push([i, j, 3 + j]);
        tris.push([i, 3 + j, 3 + i]);
    }
    let all = (0..tris.len() as u32).map(|t| (t, false)).collect();
    assemble("schonhardt", points, tris, vec![("prism".into(), all, 1)])
}

/// [`icosphere`] of radius 1 and level 3 (642 points, 1280 triangles) with every point scaled
/// radially by `1 + 0.1 u`, `u` in `[-1, 1)` drawn in point order from [`SplitMix64`] seeded with
/// `seed`. Radial scaling keeps the surface star-shaped about the origin, so it stays a simple
/// closed surface, but it is far from Delaunay and stresses boundary recovery.
pub fn noisy_sphere(seed: u64) -> SurfaceFixture {
    let (mut points, tris) = icosphere_data(1.0, 3);
    let mut rng = SplitMix64(seed);
    for p in &mut points {
        let s = 1.0 + 0.1 * rng.unit();
        *p = [p[0] * s, p[1] * s, p[2] * s];
    }
    let all = (0..tris.len() as u32).map(|t| (t, false)).collect();
    assemble("noisy_sphere", points, tris, vec![("ball".into(), all, 1)])
}

/// Every fixture with its documented default arguments.
pub fn all_surface_fixtures() -> Vec<SurfaceFixture> {
    vec![
        tri_cube(4),
        icosphere(1.0, 3),
        tri_torus(2.0, 0.7, 48, 24),
        cube_with_cavity(3),
        two_cubes_shared(3),
        schonhardt(),
        noisy_sphere(1),
    ]
}
