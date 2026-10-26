//! Shared test helpers: seeded IUR lines, independent convex-polytope clipping, tracker setup.
#![allow(dead_code)]

use rayon::prelude::*;
use sm_core::{Point3, Vector3};
use sm_mesh_api::{CellIx, FaceIx, MeshRead, Side};
use sm_core::{EntityId, EntityKind};
use sm_rt::{MeshTracker, MeshTrackerBuilder};
use sm_rt_api::{FacetInput, RtError};
use sm_testkit::surf3::{SurfaceFixture, icosphere};
use sm_topo_api::Region;
use sm_rt_api::{
    BoundarySpec, BuildInput, BuildOptions, ComplementPolicy, FaceAction, RecoveryPolicy, RegionRule, Scratch, StepEnd, Tracker, TrackerBuilder,
};
pub use sm_testkit::surf3::SplitMix64;

/// Number of lines for the statistical acceptance tests: the full count in release builds,
/// a tenth in debug builds (documented in W3_REPORT.md).
pub fn n_lines() -> usize {
    if cfg!(debug_assertions) { 100_000 } else { 1_000_000 }
}

/// FNV-1a of the case name: the seed of its generator.
pub fn seed_of(name: &str) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for b in name.bytes() {
        h = (h ^ u64::from(b)).wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

pub fn rng(name: &str) -> SplitMix64 {
    SplitMix64(seed_of(name))
}

pub fn p3(a: [f64; 3]) -> Point3 {
    Point3::new(a[0], a[1], a[2])
}
pub fn v3(a: [f64; 3]) -> Vector3 {
    Vector3::new(a[0], a[1], a[2])
}
pub fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// Uniform direction on the sphere and the two orthonormal vectors perpendicular to it.
pub fn random_frame(r: &mut SplitMix64) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let z = r.unit();
    let phi = std::f64::consts::PI * r.unit();
    let s = (1.0 - z * z).max(0.0).sqrt();
    let d = [s * phi.cos(), s * phi.sin(), z];
    let a = if d[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let e1 = cross(d, a);
    let l = dot(e1, e1).sqrt();
    let e1 = [e1[0] / l, e1[1] / l, e1[2] / l];
    let e2 = cross(d, e1);
    (d, e1, e2)
}

/// An isotropic-uniform-random line: direction uniform, point uniform on the disk of radius
/// `rho` about `center` normal to the direction. Returns `(point, dir)`.
pub fn iur_line(r: &mut SplitMix64, center: [f64; 3], rho: f64) -> ([f64; 3], [f64; 3]) {
    let (d, e1, e2) = random_frame(r);
    let rad = rho * ((r.unit() + 1.0) * 0.5).sqrt();
    let ang = std::f64::consts::PI * r.unit();
    let (c, s) = (rad * ang.cos(), rad * ang.sin());
    ([center[0] + c * e1[0] + s * e2[0], center[1] + c * e1[1] + s * e2[1], center[2] + c * e1[2] + s * e2[2]], d)
}

/// Convex polytope as outward planes `n . x <= d`, built independently of the tracker.
#[derive(Clone, Debug)]
pub struct Convex {
    pub planes: Vec<([f64; 3], f64)>,
}

impl Convex {
    /// Parameter interval of the line `o + t d` inside the polytope.
    pub fn clip(&self, o: [f64; 3], d: [f64; 3]) -> Option<(f64, f64)> {
        let (mut lo, mut hi) = (f64::NEG_INFINITY, f64::INFINITY);
        for (n, c) in &self.planes {
            let nd = dot(*n, d);
            let slack = c - dot(*n, o);
            if nd == 0.0 {
                if slack < 0.0 {
                    return None;
                }
            } else if nd > 0.0 {
                hi = hi.min(slack / nd);
            } else {
                lo = lo.max(slack / nd);
            }
            if lo > hi {
                return None;
            }
        }
        Some((lo, hi))
    }

    pub fn boxed(lo: [f64; 3], hi: [f64; 3]) -> Self {
        let mut planes = Vec::new();
        for k in 0..3 {
            let mut n = [0.0; 3];
            n[k] = 1.0;
            planes.push((n, hi[k]));
            n[k] = -1.0;
            planes.push((n, -lo[k]));
        }
        Self { planes }
    }
}

/// The convex polytope of mesh cell `c` from its faces (Newell normals, outward by the sign).
pub fn cell_polytope(m: &dyn MeshRead, c: usize) -> Convex {
    let co = m.coords();
    let mut planes = Vec::new();
    for sf in m.cell_faces(CellIx::new(c)) {
        let l = m.face_vertices(sf.face());
        let pts: Vec<[f64; 3]> = l.iter().map(|v| [co.x[v.get()], co.y[v.get()], co.z[v.get()]]).collect();
        let mut n = [0.0; 3];
        for i in 0..pts.len() {
            let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
            n[0] += (a[1] - b[1]) * (a[2] + b[2]);
            n[1] += (a[2] - b[2]) * (a[0] + b[0]);
            n[2] += (a[0] - b[0]) * (a[1] + b[1]);
        }
        let len = dot(n, n).sqrt();
        let s = if sf.side() == Side::Left { 1.0 } else { -1.0 };
        let n = [s * n[0] / len, s * n[1] / len, s * n[2] / len];
        let mean = pts.iter().fold([0.0; 3], |a, p| [a[0] + p[0], a[1] + p[1], a[2] + p[2]]);
        let mean = [mean[0] / pts.len() as f64, mean[1] / pts.len() as f64, mean[2] / pts.len() as f64];
        planes.push((n, dot(n, mean)));
    }
    Convex { planes }
}

pub fn opts_fail() -> BuildOptions {
    let mut o = BuildOptions::default();
    o.recovery = RecoveryPolicy::Fail;
    o
}

pub fn build_mesh_tracker(mesh: &dyn MeshRead, bnd: &BoundarySpec, opts: &BuildOptions) -> MeshTracker {
    let regions = RegionRule::ByVolumeClassification;
    let complement = ComplementPolicy::Outside;
    let input = BuildInput::Mesh(sm_rt_api::MeshInput::new(mesh, &regions, bnd, &complement));
    MeshTrackerBuilder.build_concrete(&input, opts).expect("build").0
}

pub fn vacuum() -> BoundarySpec {
    BoundarySpec::default()
}

/// Trace from `p0` (a point on or inside the model) along `dir` until a boundary, the end or a
/// loss; returns the chord per cell and how it ended.
pub struct Traced {
    pub chords: Vec<(u32, f64)>,
    pub lost: bool,
    pub left: bool,
}

pub fn trace_to_end(t: &dyn Tracker, p0: [f64; 3], dir: [f64; 3], sc: &mut Scratch) -> Traced {
    let mut out = Traced { chords: Vec::new(), lost: false, left: false };
    let Ok(mut st) = t.init_state(p3(p0), v3(dir), None, sc) else {
        out.lost = true;
        return out;
    };
    if st.cell.is_none() {
        out.left = true;
        return out;
    }
    for _ in 0..1_000_000 {
        let Ok(s) = t.step(&mut st, f64::INFINITY, sc) else {
            out.lost = true;
            return out;
        };
        out.chords.push((s.cell.0, s.dist));
        match s.end {
            StepEnd::Lost(_) => {
                out.lost = true;
                return out;
            }
            StepEnd::Boundary { action: FaceAction::Reflect | FaceAction::Wrap { .. }, .. } => {}
            StepEnd::Boundary { .. } => return out,
            StepEnd::RegionBoundary { .. } if st.cell.is_none() => {
                out.left = true;
                return out;
            }
            _ => {}
        }
    }
    out.lost = true;
    out
}

/// Per-cell chord statistics over lines, accumulated in fixed chunks and combined in order.
pub struct CellStats {
    pub sum: Vec<f64>,
    pub sumsq: Vec<f64>,
    /// Lines with a non-empty chord, and the sum / sum of squares of their total chord.
    pub hits: usize,
    pub line_sum: f64,
    pub line_sumsq: f64,
    pub n: usize,
    pub lost: usize,
    pub max_err: f64,
}

pub fn run_iur(
    t: &(dyn Tracker + Sync),
    name: &str,
    n: usize,
    center: [f64; 3],
    rho: f64,
    // analytic runs of a line: list of (t_in, t_out) maximal intervals inside the model
    runs: &(dyn Fn([f64; 3], [f64; 3]) -> Vec<(f64, f64)> + Sync),
) -> CellStats {
    let nc = t.cell_count();
    // Lines are generated in index order from one generator per chunk (seeded from the case
    // name and the chunk index), so the result does not depend on the thread count.
    let chunk = 10_000;
    let nchunks = n.div_ceil(chunk);
    let parts: Vec<CellStats> = (0..nchunks)
        .into_par_iter()
        .map(|ci| {
            let mut r = SplitMix64(seed_of(name) ^ sm_testkit::surf3::mix64(ci as u64 + 1));
            let mut sc = Scratch::new();
            let mut s = CellStats { sum: vec![0.0; nc], sumsq: vec![0.0; nc], hits: 0, line_sum: 0.0, line_sumsq: 0.0, n: 0, lost: 0, max_err: 0.0 };
            let mut line_cell = vec![0.0; nc];
            for _ in 0..chunk.min(n - ci * chunk) {
                let (o, d) = iur_line(&mut r, center, rho);
                s.n += 1;
                let mut touched: Vec<usize> = Vec::new();
                let mut line_total = 0.0;
                for (tin, tout) in runs(o, d) {
                    let p0 = [o[0] + tin * d[0], o[1] + tin * d[1], o[2] + tin * d[2]];
                    let tr = trace_to_end(t, p0, d, &mut sc);
                    if tr.lost {
                        s.lost += 1;
                    }
                    let total: f64 = tr.chords.iter().map(|c| c.1).sum();
                    line_total += total;
                    s.max_err = s.max_err.max((total - (tout - tin)).abs());
                    for (c, l) in tr.chords {
                        if line_cell[c as usize] == 0.0 {
                            touched.push(c as usize);
                        }
                        line_cell[c as usize] += l;
                        if line_cell[c as usize] == 0.0 {
                            line_cell[c as usize] = f64::MIN_POSITIVE;
                        }
                    }
                }
                if line_total > 0.0 {
                    s.hits += 1;
                    s.line_sum += line_total;
                    s.line_sumsq += line_total * line_total;
                }
                for c in touched {
                    let l = line_cell[c];
                    line_cell[c] = 0.0;
                    s.sum[c] += l;
                    s.sumsq[c] += l * l;
                }
            }
            s
        })
        .collect();
    let mut tot = CellStats { sum: vec![0.0; nc], sumsq: vec![0.0; nc], hits: 0, line_sum: 0.0, line_sumsq: 0.0, n: 0, lost: 0, max_err: 0.0 };
    for p in parts {
        for c in 0..nc {
            tot.sum[c] += p.sum[c];
            tot.sumsq[c] += p.sumsq[c];
        }
        tot.n += p.n;
        tot.hits += p.hits;
        tot.line_sum += p.line_sum;
        tot.line_sumsq += p.line_sumsq;
        tot.lost += p.lost;
        tot.max_err = tot.max_err.max(p.max_err);
    }
    tot
}

/// Assert `mean chord == V / (pi rho^2)` within `k` sigma for every cell with volume `vols[c]`;
/// prints the largest |z| so the report can quote it.
pub fn assert_cell_means(st: &CellStats, vols: &[f64], rho: f64, k: f64) {
    let n = st.n as f64;
    let mut max_z: f64 = 0.0;
    for (c, &v) in vols.iter().enumerate() {
        let mean = st.sum[c] / n;
        let var = (st.sumsq[c] / n - mean * mean).max(0.0);
        let sigma = (var / n).sqrt();
        let want = v / (std::f64::consts::PI * rho * rho);
        let z = (mean - want).abs() / sigma.max(1e-18);
        max_z = max_z.max(z);
        assert!(z <= k, "cell {c}: mean chord {mean} vs {want} (sigma {sigma}, {z} sigma)");
    }
    eprintln!("cell means: {} cells, {} lines, max |z| = {max_z:.2}, max chord-sum error {:.2e}", vols.len(), st.n, st.max_err);
}

/// Cauchy: the mean chord of the lines that hit equals `4 V / S`, within `k` sigma.
pub fn assert_cauchy(st: &CellStats, v: f64, s: f64, k: f64) {
    let h = st.hits as f64;
    let mean = st.line_sum / h;
    let var = (st.line_sumsq / h - mean * mean).max(0.0);
    let sigma = (var / h).sqrt();
    let want = 4.0 * v / s;
    eprintln!("Cauchy: mean chord {mean:.6} vs 4V/S {want:.6} ({:.2} sigma, {} hits)", (mean - want).abs() / sigma, st.hits);
    assert!((mean - want).abs() <= k * sigma, "Cauchy mean chord {mean} vs 4V/S = {want} (sigma {sigma}, {} hits)", st.hits);
}

/// Merge per-polytope intervals into maximal runs (touching intervals join).
pub fn merge_runs(mut iv: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
    iv.retain(|(a, b)| b > a);
    iv.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut out: Vec<(f64, f64)> = Vec::new();
    for (a, b) in iv {
        match out.last_mut() {
            Some(last) if a <= last.1 + 1e-10 => last.1 = last.1.max(b),
            _ => out.push((a, b)),
        }
    }
    out
}

pub fn bbox(m: &dyn MeshRead) -> ([f64; 3], [f64; 3]) {
    let co = m.coords();
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for i in 0..m.vertex_count() {
        for (k, a) in [co.x, co.y, co.z].iter().enumerate() {
            lo[k] = lo[k].min(a[i]);
            hi[k] = hi[k].max(a[i]);
        }
    }
    (lo, hi)
}

pub fn shared_edge(m: &dyn MeshRead, c: usize) -> Vec<(FaceIx, FaceIx, [f64; 3], [f64; 3])> {
    let faces: Vec<FaceIx> = m.cell_faces(CellIx::new(c)).iter().map(|s| s.face()).collect();
    let co = m.coords();
    let mut out = Vec::new();
    for i in 0..faces.len() {
        for j in i + 1..faces.len() {
            let (a, b) = (m.face_vertices(faces[i]), m.face_vertices(faces[j]));
            let common: Vec<_> = a.iter().filter(|v| b.contains(v)).collect();
            if common.len() == 2 {
                let (p, q) = (common[0].get(), common[1].get());
                out.push((faces[i], faces[j], [co.x[p], co.y[p], co.z[p]], [co.x[q], co.y[q], co.z[q]]));
            }
        }
    }
    out
}

pub fn cell_vertices(m: &dyn MeshRead, c: usize) -> Vec<(usize, [f64; 3], Vec<FaceIx>)> {
    let co = m.coords();
    let faces: Vec<FaceIx> = m.cell_faces(CellIx::new(c)).iter().map(|s| s.face()).collect();
    let mut vs: Vec<usize> = faces.iter().flat_map(|&f| m.face_vertices(f).iter().map(|v| v.get())).collect();
    vs.sort_unstable();
    vs.dedup();
    vs.into_iter()
        .map(|v| (v, [co.x[v], co.y[v], co.z[v]], faces.iter().copied().filter(|&f| m.face_vertices(f).iter().any(|x| x.get() == v)).collect()))
        .collect()
}


pub fn ent(kind: EntityKind, n: u64) -> EntityId {
    EntityId::new(kind, n).unwrap()
}

pub fn vol(n: u64) -> Region {
    Region::Volume(ent(EntityKind::TopoVolume, n))
}

/// A closed shell: outward triangles, `front` inside, `back` outside, one B-rep face.
pub fn add_shell(f: &mut FacetInput, pts: &[[f64; 3]], tris: &[[u32; 3]], front: Region, back: Region, face: u64) {
    let base = f.vertices.len() as u32;
    f.vertices.extend_from_slice(pts);
    for t in tris {
        f.triangles.push([t[0] + base, t[1] + base, t[2] + base]);
        f.front.push(front);
        f.back.push(back);
        f.face.push(ent(EntityKind::TopoFace, face));
    }
}

pub fn sphere_facets(r: f64, level: u32) -> (FacetInput, SurfaceFixture) {
    let fx = icosphere(r, level);
    let mut f = FacetInput::default();
    add_shell(&mut f, &fx.points, &fx.oriented_tris(0), vol(1), Region::Exterior, 1);
    (f, fx)
}

pub fn nested_facets() -> FacetInput {
    let (outer, inner) = (icosphere(2.0, 2), icosphere(1.0, 2));
    let mut f = FacetInput::default();
    add_shell(&mut f, &outer.points, &outer.oriented_tris(0), vol(2), Region::Exterior, 1);
    add_shell(&mut f, &inner.points, &inner.oriented_tris(0), vol(1), vol(2), 2);
    f
}

pub fn tet_facets() -> FacetInput {
    let mut f = FacetInput::default();
    let pts = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    add_shell(&mut f, &pts, &[[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]], vol(1), Region::Exterior, 1);
    f
}

pub fn build(f: &FacetInput, opts: &BuildOptions) -> Result<Box<dyn Tracker>, RtError> {
    let spec = BoundarySpec::default();
    MeshTrackerBuilder.build(&BuildInput::Facets(f, &spec), opts).map(|b| b.0)
}

pub fn remove_triangle(f: &FacetInput, i: usize) -> FacetInput {
    let mut g = f.clone();
    g.triangles.remove(i);
    g.front.remove(i);
    g.back.remove(i);
    g.face.remove(i);
    g
}

