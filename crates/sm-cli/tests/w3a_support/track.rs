//! Row 13: the sm-rt VOLUME_MESH tracker built in process on a milestone mesh, checked against
//! a brute-force ray / model-face intersection, analytic axis chords, thread counts and the
//! `.smrt` cache round trip. No protocol field, op or `.smsh` section is involved.

use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_geom_api::Ray;
use sm_mesh_api::{FaceIx, MeshRead};
use sm_rt::MeshTrackerBuilder;
use sm_rt_api::{
    BoundarySpec, BuildInput, BuildOptions, CacheImage, ComplementPolicy, ContentHash, MeshInput, RegionRule, RtError,
    TraceOptions, TrackBundle, TrackerBuilder,
};

/// Rays per mesh and the LCG seed (plan row 13).
pub const N_RAYS: usize = 10_000;
pub const SEED: u64 = 0x57A3;
/// Chord tolerance.
pub const TOL: f64 = 1e-9;

struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// Analytic region of a point: 0 = V1, 1 = V2, 2 = V3, `None` outside (plan §4 part).
pub fn region_of(p: [f64; 3]) -> Option<usize> {
    let [x, y, z] = p;
    if (0.0..=4.0).contains(&z) {
        let in_sq = (0.0..=20.0).contains(&x) && (0.0..=20.0).contains(&y);
        if in_sq && !(x > 10.0 && y > 10.0) {
            return Some(0);
        }
        if x >= 10.0 && y >= 10.0 && x + y <= 30.0 {
            return Some(1);
        }
        return None;
    }
    if (4.0..=10.0).contains(&z) && (10.0..=20.0).contains(&x) && (10.0..=20.0).contains(&y) {
        return Some(2);
    }
    None
}

/// Distance of `p` from the part's boundary planes (for picking origins well inside).
fn margin(p: [f64; 3]) -> f64 {
    let [x, y, z] = p;
    let d = [x, 20.0 - x, y, 20.0 - y, (x - 10.0).abs(), (y - 10.0).abs(), z, (z - 4.0).abs(), 10.0 - z, (x + y - 30.0).abs() / 2f64.sqrt()];
    d.iter().fold(f64::INFINITY, |a, &b| a.min(b.abs()))
}

/// The milestone ray set: origins uniform inside the part (at least 1e-3 from its planes),
/// directions uniform on the sphere.
pub fn rays() -> Vec<Ray> {
    let mut g = Lcg(SEED);
    let mut out = Vec::with_capacity(N_RAYS);
    while out.len() < N_RAYS {
        let p = [20.0 * g.next(), 20.0 * g.next(), 10.0 * g.next()];
        let (u, w) = (g.next(), g.next());
        if region_of(p).is_none() || margin(p) < 1e-3 {
            continue;
        }
        let cz = 2.0 * u - 1.0;
        let s = (1.0 - cz * cz).max(0.0).sqrt();
        let ph = std::f64::consts::TAU * w;
        out.push(Ray { origin: Point3 { x: p[0], y: p[1], z: p[2] }, dir: Vector3 { x: s * ph.cos(), y: s * ph.sin(), z: cz } });
    }
    out
}

/// Planar polygons of the mesh faces classified on model faces.
struct Faces {
    pts: Vec<Vec<[f64; 3]>>,
    n: Vec<[f64; 3]>,
    c: Vec<[f64; 3]>,
    r: Vec<f64>,
}

fn model_faces(m: &dyn MeshRead) -> Faces {
    let mut f = Faces { pts: Vec::new(), n: Vec::new(), c: Vec::new(), r: Vec::new() };
    for i in 0..m.face_count() {
        let fi = FaceIx::new(i);
        if m.face_class(fi).entity.kind() != EntityKind::TopoFace {
            continue;
        }
        let pts: Vec<[f64; 3]> = m.face_vertices(fi).iter().map(|v| {
            let q = m.point(v.get());
            [q.x, q.y, q.z]
        }).collect();
        let k = pts.len() as f64;
        let c = [pts.iter().map(|p| p[0]).sum::<f64>() / k, pts.iter().map(|p| p[1]).sum::<f64>() / k, pts.iter().map(|p| p[2]).sum::<f64>() / k];
        let mut n = [0.0; 3];
        for j in 0..pts.len() {
            let (p, q) = (pts[j], pts[(j + 1) % pts.len()]);
            n[0] += (p[1] - q[1]) * (p[2] + q[2]);
            n[1] += (p[2] - q[2]) * (p[0] + q[0]);
            n[2] += (p[0] - q[0]) * (p[1] + q[1]);
        }
        let r = pts.iter().map(|p| ((p[0] - c[0]).powi(2) + (p[1] - c[1]).powi(2) + (p[2] - c[2]).powi(2)).sqrt()).fold(0.0, f64::max);
        f.pts.push(pts);
        f.n.push(n);
        f.c.push(c);
        f.r.push(r);
    }
    f
}

/// Crossing-number test of `q` in polygon `pts`, projected along the dominant axis of `n`.
fn inside(pts: &[[f64; 3]], n: [f64; 3], q: [f64; 3]) -> bool {
    let ax = if n[0].abs() >= n[1].abs() && n[0].abs() >= n[2].abs() { 0 } else if n[1].abs() >= n[2].abs() { 1 } else { 2 };
    let (a, b) = match ax {
        0 => (1, 2),
        1 => (2, 0),
        _ => (0, 1),
    };
    let mut c = false;
    let k = pts.len();
    for i in 0..k {
        let (p, r) = (pts[i], pts[(i + k - 1) % k]);
        if (p[b] > q[b]) != (r[b] > q[b]) {
            let x = (r[a] - p[a]) * (q[b] - p[b]) / (r[b] - p[b]) + p[a];
            if q[a] < x {
                c = !c;
            }
        }
    }
    c
}

/// Brute force: (volume index, chord) along a ray until it leaves the part.
fn brute(f: &Faces, o: [f64; 3], d: [f64; 3]) -> Vec<(usize, f64)> {
    let mut ts: Vec<f64> = Vec::new();
    for i in 0..f.pts.len() {
        let c = f.c[i];
        let w = [c[0] - o[0], c[1] - o[1], c[2] - o[2]];
        let along = w[0] * d[0] + w[1] * d[1] + w[2] * d[2];
        let perp2 = w[0] * w[0] + w[1] * w[1] + w[2] * w[2] - along * along;
        if perp2 > f.r[i] * f.r[i] * (1.0 + 1e-9) + 1e-18 || along < -f.r[i] {
            continue;
        }
        let n = f.n[i];
        let dn = n[0] * d[0] + n[1] * d[1] + n[2] * d[2];
        if dn == 0.0 {
            continue;
        }
        let t = (n[0] * w[0] + n[1] * w[1] + n[2] * w[2]) / dn;
        if t <= 1e-12 {
            continue;
        }
        let q = [o[0] + t * d[0], o[1] + t * d[1], o[2] + t * d[2]];
        if inside(&f.pts[i], n, q) {
            ts.push(t);
        }
    }
    ts.sort_by(f64::total_cmp);
    ts.dedup_by(|b, a| (*b - *a).abs() < 1e-9);
    let mut out: Vec<(usize, f64)> = Vec::new();
    let mut t0 = 0.0;
    for &t in &ts {
        let mid = (t0 + t) / 2.0;
        let Some(r) = region_of([o[0] + mid * d[0], o[1] + mid * d[1], o[2] + mid * d[2]]) else { break };
        match out.last_mut() {
            Some(last) if last.0 == r => last.1 += t - t0,
            _ => out.push((r, t - t0)),
        }
        t0 = t;
    }
    out
}

/// Tracker segments of ray `i`, merged per region and mapped to volume indices.
fn traced(b: &TrackBundle, i: usize, vol_of_region: &[Option<usize>]) -> Vec<(usize, f64)> {
    let mut out: Vec<(usize, f64)> = Vec::new();
    let Some(rg) = b.ray_range(i) else { return out };
    for s in rg {
        let v = vol_of_region.get(b.region[s] as usize).copied().flatten().unwrap_or(usize::MAX);
        match out.last_mut() {
            Some(last) if last.0 == v => last.1 += b.length[s],
            _ => out.push((v, b.length[s])),
        }
    }
    out
}

fn same(a: &[(usize, f64)], b: &[(usize, f64)]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.0 == y.0 && (x.1 - y.1).abs() <= TOL)
}

/// Row 13 on one mesh: problems (empty = pass) and facts for the report.
#[derive(Debug, Default)]
pub struct TrackCheck {
    pub problems: Vec<String>,
    pub facts: Vec<String>,
}

pub fn check(m: &dyn MeshRead, vols: &[EntityId; 3]) -> TrackCheck {
    let mut r = TrackCheck::default();
    let (rules, spec, comp) = (RegionRule::ByVolumeClassification, BoundarySpec::default(), ComplementPolicy::Outside);
    let input = BuildInput::Mesh(MeshInput::new(m, &rules, &spec, &comp));
    let opts = BuildOptions::default();
    let builder = MeshTrackerBuilder;
    let (t, report) = match builder.build(&input, &opts) {
        Ok(x) => x,
        Err(x) => {
            r.problems.push(format!("tracker build: {x}"));
            return r;
        }
    };
    r.facts.push(format!("build issues: {}", report.issues.len()));
    let vol_of_region: Vec<Option<usize>> =
        t.regions().0.iter().map(|ri| ri.entity.and_then(|e| vols.iter().position(|v| *v == e))).collect();
    if vol_of_region.iter().filter(|v| v.is_some()).count() != 3 {
        r.problems.push(format!("regions do not map to V1..V3: {:?}", t.regions().0.iter().map(|x| x.entity).collect::<Vec<_>>()));
    }
    let rs = rays();
    let topts = TraceOptions::default();
    let pool = |n: usize| rayon::ThreadPoolBuilder::new().num_threads(n).build().map_err(|x| x.to_string());
    let (b1, bn) = match (pool(1), pool(4)) {
        (Ok(p1), Ok(pn)) => (p1.install(|| t.trace_bundle(&rs, &topts)), pn.install(|| t.trace_bundle(&rs, &topts))),
        (Err(x), _) | (_, Err(x)) => {
            r.problems.push(format!("thread pool: {x}"));
            return r;
        }
    };
    let (b1, bn) = match (b1, bn) {
        (Ok(a), Ok(b)) => (a, b),
        (Err(x), _) | (_, Err(x)) => {
            r.problems.push(format!("trace_bundle: {x}"));
            return r;
        }
    };
    if b1.content_hash() != bn.content_hash() {
        r.problems.push(format!("bundle content hash differs for 1 vs 4 threads: {:?} vs {:?}", b1.content_hash(), bn.content_hash()));
    }
    // Brute force, rays split over a few scoped threads (each ray is independent).
    let faces = model_faces(m);
    let chunks = 8;
    let per = rs.len().div_ceil(chunks);
    let brute_all: Vec<Vec<(usize, f64)>> = std::thread::scope(|s| {
        let hs: Vec<_> = rs
            .chunks(per)
            .map(|ch| {
                let faces = &faces;
                s.spawn(move || {
                    ch.iter()
                        .map(|ray| {
                            let d = ray.dir;
                            brute(faces, [ray.origin.x, ray.origin.y, ray.origin.z], [d.x, d.y, d.z])
                        })
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        hs.into_iter().flat_map(|h| h.join().unwrap_or_default()).collect()
    });
    let mut bad = Vec::new();
    let mut max_dev = 0.0f64;
    for (i, want) in brute_all.iter().enumerate() {
        let got = traced(&b1, i, &vol_of_region);
        if got.len() == want.len() && got.iter().zip(want).all(|(a, b)| a.0 == b.0) {
            for (a, b) in got.iter().zip(want) {
                max_dev = max_dev.max((a.1 - b.1).abs());
            }
        }
        if !same(&got, want) {
            bad.push(i);
        }
    }
    r.facts.push(format!("{} rays, {} model-face polygons, max chord deviation {max_dev:e}", rs.len(), faces.pts.len()));
    if !bad.is_empty() {
        let i = bad[0];
        r.problems.push(format!(
            "{} of {} rays differ from the brute force (tol {TOL:e}); first ray {i}: tracker {:?} vs brute {:?}",
            bad.len(),
            rs.len(),
            traced(&b1, i, &vol_of_region),
            brute_all[i]
        ));
    }
    // Axis rays: up and down from an interior point, chords add up to the slab height.
    let axis = |x: f64, y: f64, z: f64, want_vol: usize, want: f64, r: &mut TrackCheck| {
        let mk = |dz: f64| Ray { origin: Point3 { x, y, z }, dir: Vector3 { x: 0.0, y: 0.0, z: dz } };
        match t.trace_bundle(&[mk(1.0), mk(-1.0)], &topts) {
            Ok(b) => {
                let mut tot = 0.0;
                for i in 0..2 {
                    for (v, l) in traced(&b, i, &vol_of_region) {
                        if v != want_vol {
                            r.problems.push(format!("axis ray at ({x}, {y}, {z}) enters volume index {v}"));
                        }
                        tot += l;
                    }
                }
                if (tot - want).abs() > TOL {
                    r.problems.push(format!("axis chord at ({x}, {y}): {tot} != {want}"));
                }
            }
            Err(x2) => r.problems.push(format!("axis trace: {x2}")),
        }
    };
    for (x, y) in [(16.3, 15.9), (19.1, 14.7), (14.6, 19.2), (18.4, 18.1), (12.9, 17.6)] {
        axis(x, y, 7.1, 2, 6.0, &mut r);
    }
    for (x, y) in [(5.3, 4.7), (15.2, 5.6), (4.4, 15.8), (2.1, 18.3), (9.7, 0.6)] {
        axis(x, y, 2.1, 0, 4.0, &mut r);
    }
    // The .smrt cache round trip.
    let Some(img) = t.export_cache() else {
        r.problems.push("export_cache returned None".into());
        return r;
    };
    let bytes = img.encode();
    match CacheImage::decode(&bytes) {
        Ok(back) => {
            match builder.load(&input, &opts, &back) {
                Ok((u, rep)) => {
                    if !rep.from_cache {
                        r.problems.push("restored tracker does not report from_cache".into());
                    }
                    if u.structure_hash() != t.structure_hash() {
                        r.problems.push("restored structure_hash differs".into());
                    }
                    match u.trace_bundle(&rs, &topts) {
                        Ok(bu) if bu.content_hash() == b1.content_hash() => {}
                        Ok(_) => r.problems.push("restored tracker gives another bundle hash".into()),
                        Err(x) => r.problems.push(format!("restored trace: {x}")),
                    }
                }
                Err(x) => r.problems.push(format!("load from cache: {x}")),
            }
            let mut stale = back.clone();
            stale.key.options_hash = ContentHash(stale.key.options_hash.0 ^ 1);
            match builder.load(&input, &opts, &stale) {
                Err(RtError::CacheStale(_)) => {}
                Err(x) => r.problems.push(format!("changed key: expected CacheStale, got {x}")),
                Ok(_) => r.problems.push("changed key: the cache was accepted".into()),
            }
        }
        Err(x) => r.problems.push(format!("CacheImage::decode: {x}")),
    }
    r.facts.push(format!("bundle hash {:?}, structure {:?}", b1.content_hash(), t.structure_hash()));
    r
}
