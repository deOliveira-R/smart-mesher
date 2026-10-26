//! Record-only throughput numbers (acceptance 14): `cargo run --release -p sm-rt --example bench_rt`.
//!
//! Prints step rate on `hex_block(N)` (default N = 100; pass another N as the first argument),
//! `locate` points/s, `trace_bundle` rays/s on an icosphere facet shell and the build time and
//! bytes per face, each for 1 and 4 threads. `--write-inputs DIR` writes `facets.bin` and
//! `rays.bin` (an icosphere and rays started inside it) for `trace_rays`.

use rayon::prelude::*;
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_geom_api::Ray;
use sm_mesh_api::SerialAlloc;
use sm_mesh_api::reference::hex_block;
use sm_rt::{MeshTracker, MeshTrackerBuilder};
use sm_rt_api::{BoundarySpec, BuildInput, BuildOptions, ComplementPolicy, FacetInput, MeshInput, RegionRule, Scratch, StepEnd, TraceOptions, Tracker};
use sm_testkit::surf3::{SplitMix64, icosphere};
use sm_topo_api::Region;
use std::time::Instant;

fn frame(r: &mut SplitMix64) -> [f64; 3] {
    let z = r.unit();
    let phi = std::f64::consts::PI * r.unit();
    let s = (1.0 - z * z).max(0.0).sqrt();
    [s * phi.cos(), s * phi.sin(), z]
}

fn cpu_model() -> String {
    std::fs::read_to_string("/proc/cpuinfo")
        .ok()
        .and_then(|s| s.lines().find(|l| l.starts_with("model name")).and_then(|l| l.split(':').nth(1)).map(|m| m.trim().to_owned()))
        .unwrap_or_else(|| "unknown".to_owned())
}

fn pool(n: usize) -> rayon::ThreadPool {
    rayon::ThreadPoolBuilder::new().num_threads(n).build().expect("pool")
}

fn sphere_facets(level: u32) -> (FacetInput, Vec<[f64; 3]>, Vec<[u32; 3]>) {
    let fx = icosphere(1.0, level);
    let tris = fx.oriented_tris(0);
    let mut f = FacetInput::default();
    f.vertices = fx.points.clone();
    f.triangles = tris.clone();
    let vol = Region::Volume(EntityId::new(EntityKind::TopoVolume, 1).expect("id"));
    f.front = vec![vol; tris.len()];
    f.back = vec![Region::Exterior; tris.len()];
    f.face = vec![EntityId::new(EntityKind::TopoFace, 1).expect("id"); tris.len()];
    (f, fx.points, tris)
}

fn write_inputs(dir: &str) {
    let (_, pts, tris) = sphere_facets(4);
    let mut b: Vec<u8> = Vec::new();
    b.extend((pts.len() as u32).to_le_bytes());
    b.extend((tris.len() as u32).to_le_bytes());
    for p in &pts {
        for c in p {
            b.extend(c.to_le_bytes());
        }
    }
    for t in &tris {
        for v in t {
            b.extend(v.to_le_bytes());
        }
    }
    for _ in &tris {
        b.extend(0i32.to_le_bytes());
        b.extend((-1i32).to_le_bytes());
    }
    std::fs::write(format!("{dir}/facets.bin"), b).expect("write facets");
    let n = 100_000u32;
    let mut r = SplitMix64(0x5eed);
    let mut b: Vec<u8> = Vec::new();
    b.extend(n.to_le_bytes());
    for _ in 0..n {
        let rad = 0.9 * ((r.unit() + 1.0) / 2.0).cbrt();
        let p = frame(&mut r);
        let o = [p[0] * rad, p[1] * rad, p[2] * rad];
        for c in o.iter().chain(frame(&mut r).iter()) {
            b.extend(c.to_le_bytes());
        }
    }
    for _ in 0..n {
        b.extend(0i32.to_le_bytes());
    }
    std::fs::write(format!("{dir}/rays.bin"), b).expect("write rays");
}

fn rate(label: &str, threads: usize, work: f64, secs: f64) {
    println!("{label:<34} threads={threads} {:>14.0} /s ({work:.0} in {secs:.3} s)", work / secs);
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().is_some_and(|a| a == "--write-inputs") {
        write_inputs(args.get(1).map_or(".", String::as_str));
        return;
    }
    let n: u32 = args.first().and_then(|a| a.parse().ok()).unwrap_or(100);
    println!("cpu: {}", cpu_model());
    let t0 = Instant::now();
    let mesh = hex_block([n; 3], [1.0; 3], &mut SerialAlloc::new()).expect("hex_block");
    println!("hex_block({n}) reference mesh built in {:.1} s", t0.elapsed().as_secs_f64());
    let (rules, comp, spec) = (RegionRule::ByVolumeClassification, ComplementPolicy::Outside, BoundarySpec::default());
    let input = BuildInput::Mesh(MeshInput::new(&mesh, &rules, &spec, &comp));
    let opts = BuildOptions::default();
    for threads in [1, 4] {
        let t0 = Instant::now();
        let (t, _) = pool(threads).install(|| MeshTrackerBuilder.build_concrete(&input, &opts)).expect("build");
        let secs = t0.elapsed().as_secs_f64();
        let info = t.info();
        println!("build hex_block({n}) threads={threads}: {secs:.2} s, {} cells, {} faces, {:.1} bytes/face ({} bytes)", info.cells, info.faces, info.memory_bytes as f64 / info.faces as f64, info.memory_bytes);
    }
    let (t, _) = MeshTrackerBuilder.build_concrete(&input, &opts).expect("build");
    let edge = f64::from(n);
    for threads in [1, 4] {
        let rays = 200_000usize;
        let t0 = Instant::now();
        let crossings: u64 = pool(threads).install(|| {
            (0..rays / 1000)
                .into_par_iter()
                .map(|ci| {
                    let mut r = SplitMix64(1000 + ci as u64);
                    let mut sc = Scratch::new();
                    let mut c = 0u64;
                    for _ in 0..1000 {
                        let o = Point3::new(edge * (r.unit() + 1.0) / 2.0, edge * (r.unit() + 1.0) / 2.0, edge * (r.unit() + 1.0) / 2.0);
                        let d = frame(&mut r);
                        let Ok(mut st) = t.init_state(o, Vector3::new(d[0], d[1], d[2]), None, &mut sc) else { continue };
                        for _ in 0..10_000 {
                            let Ok(s) = t.step(&mut st, f64::INFINITY, &mut sc) else { break };
                            c += 1;
                            if matches!(s.end, StepEnd::Boundary { .. } | StepEnd::Lost(_)) {
                                break;
                            }
                        }
                    }
                    c
                })
                .sum()
        });
        rate("step (hex-cell crossings)", threads, crossings as f64, t0.elapsed().as_secs_f64());
        let npts = 2_000_000usize;
        let t0 = Instant::now();
        let hits: usize = pool(threads).install(|| {
            (0..npts / 1000)
                .into_par_iter()
                .map(|ci| {
                    let mut r = SplitMix64(7000 + ci as u64);
                    let mut sc = Scratch::new();
                    let mut hit = 0;
                    for _ in 0..1000 {
                        let p = Point3::new(edge * (r.unit() + 1.0) / 2.0, edge * (r.unit() + 1.0) / 2.0, edge * (r.unit() + 1.0) / 2.0);
                        hit += usize::from(t.locate(p, None, None, &mut sc).is_ok_and(|l| !l.cell.is_none()));
                    }
                    hit
                })
                .sum()
        });
        rate("locate (no hint, BVH)", threads, npts as f64, t0.elapsed().as_secs_f64());
        assert!(hits + 100 >= npts, "points near the boundary may be outside, got {hits} of {npts}");
    }
    drop(t);
    drop(mesh);

    let (f, _, _) = sphere_facets(4);
    let fspec = BoundarySpec::default();
    let finput = BuildInput::Facets(&f, &fspec);
    for threads in [1, 4] {
        let t0 = Instant::now();
        let (t, _): (MeshTracker, _) = pool(threads).install(|| MeshTrackerBuilder.build_concrete(&finput, &opts)).expect("build");
        let secs = t0.elapsed().as_secs_f64();
        let info = t.info();
        println!("build icosphere(4) threads={threads}: {secs:.4} s, {} faces, {:.1} bytes/face", info.faces, info.memory_bytes as f64 / info.faces as f64);
    }
    let (t, _) = MeshTrackerBuilder.build_concrete(&finput, &opts).expect("build");
    let mut r = SplitMix64(99);
    let rays: Vec<Ray> = (0..1_000_000)
        .map(|_| {
            let rad = 0.9 * ((r.unit() + 1.0) / 2.0).cbrt();
            let p = frame(&mut r);
            let d = frame(&mut r);
            Ray { origin: Point3::new(p[0] * rad, p[1] * rad, p[2] * rad), dir: Vector3::new(d[0], d[1], d[2]) }
        })
        .collect();
    for threads in [1, 4] {
        let t0 = Instant::now();
        let b = pool(threads).install(|| t.trace_bundle(&rays, &TraceOptions::default())).expect("bundle");
        rate("trace_bundle (icosphere 4)", threads, rays.len() as f64, t0.elapsed().as_secs_f64());
        assert_eq!(b.ray_count(), rays.len());
    }
}
