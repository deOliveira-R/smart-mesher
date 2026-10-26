//! Record-only benchmark side of the `sm-rt-capi` harness.
//!
//! Usage: `trace_rays [facets.bin [rays.bin]]` (defaults: the two names in the working
//! directory). Formats, little-endian:
//!
//! * `smfacets v1`: `u32 nv, u32 nt`, `nv * 3` `f64` vertices, `nt * 3` `u32` triangle corners,
//!   `nt * 2` `i32` (forward, reverse) regions, `-1` = complement;
//! * `smrays v1`: `u32 n`, `n * 6` `f64` (origin, unit direction), `n` `i32` start regions.
//!
//! Builds a facet tracker, fires every ray to leakage with `fire`-equivalent stepping (a
//! `step` loop that ends at a model boundary, the loss of the particle or the domain bounds)
//! and prints one JSON object `{build_s, rays, rays_per_s, crossings_per_s, lost, threads,
//! cpu_model}`. Rays are processed in index order in fixed blocks; the counts do not depend on
//! the thread count.

use rayon::prelude::*;
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_rt::MeshTrackerBuilder;
use sm_rt_api::{BoundarySpec, BuildInput, BuildOptions, FacetInput, RecoveryPolicy, Scratch, StepEnd, Tracker, TrackerBuilder};
use sm_topo_api::Region;
use std::collections::BTreeMap;
use std::time::Instant;

struct Rd<'a>(&'a [u8], usize);

impl Rd<'_> {
    fn take(&mut self, n: usize) -> Result<&[u8], String> {
        let end = self.1.checked_add(n).filter(|&e| e <= self.0.len()).ok_or("file truncated")?;
        let s = &self.0[self.1..end];
        self.1 = end;
        Ok(s)
    }
    fn u32(&mut self) -> Result<u32, String> {
        let b = self.take(4)?;
        Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }
    fn f64(&mut self) -> Result<f64, String> {
        let b = self.take(8)?;
        Ok(f64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]]))
    }
}

fn region(r: i32) -> Result<Region, String> {
    if r == -1 {
        return Ok(Region::Exterior);
    }
    let serial = u64::try_from(r).map_err(|_| format!("bad region {r}"))? + 1;
    EntityId::new(EntityKind::TopoVolume, serial).map(Region::Volume).ok_or_else(|| format!("bad region {r}"))
}

fn read_facets(path: &str) -> Result<FacetInput, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    let mut r = Rd(&bytes, 0);
    let (nv, nt) = (r.u32()? as usize, r.u32()? as usize);
    let mut f = FacetInput::default();
    for _ in 0..nv {
        f.vertices.push([r.f64()?, r.f64()?, r.f64()?]);
    }
    for _ in 0..nt {
        f.triangles.push([r.u32()?, r.u32()?, r.u32()?]);
    }
    // One B-rep face per distinct (forward, reverse) pair, numbered in order of appearance.
    let mut faces: BTreeMap<(i32, i32), u64> = BTreeMap::new();
    for _ in 0..nt {
        let (fw, bw) = (r.u32()? as i32, r.u32()? as i32);
        f.front.push(region(fw)?);
        f.back.push(region(bw)?);
        let next = faces.len() as u64 + 1;
        let k = *faces.entry((fw, bw)).or_insert(next);
        f.face.push(EntityId::new(EntityKind::TopoFace, k).ok_or("too many faces")?);
    }
    Ok(f)
}

struct Ray {
    o: [f64; 3],
    d: [f64; 3],
    start: i32,
}

fn read_rays(path: &str) -> Result<Vec<Ray>, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    let mut r = Rd(&bytes, 0);
    let n = r.u32()? as usize;
    let mut v = Vec::with_capacity(n);
    for _ in 0..n {
        v.push(Ray { o: [r.f64()?, r.f64()?, r.f64()?], d: [r.f64()?, r.f64()?, r.f64()?], start: 0 });
    }
    for ray in &mut v {
        ray.start = r.u32()? as i32;
    }
    Ok(v)
}

fn cpu_model() -> String {
    std::fs::read_to_string("/proc/cpuinfo")
        .ok()
        .and_then(|s| s.lines().find(|l| l.starts_with("model name")).and_then(|l| l.split(':').nth(1)).map(|m| m.trim().to_owned()))
        .unwrap_or_else(|| "unknown".to_owned())
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let facets = read_facets(args.first().map_or("facets.bin", String::as_str))?;
    let rays = read_rays(args.get(1).map_or("rays.bin", String::as_str))?;
    let spec = BoundarySpec::default();
    let mut opts = BuildOptions::default();
    opts.recovery = RecoveryPolicy::Fail;
    let t0 = Instant::now();
    let (tracker, _) = MeshTrackerBuilder.build(&BuildInput::Facets(&facets, &spec), &opts).map_err(|e| e.to_string())?;
    let build_s = t0.elapsed().as_secs_f64();
    let tracker: &dyn Tracker = &*tracker;
    let t1 = Instant::now();
    let (crossings, lost): (u64, u64) = rays
        .par_chunks(256)
        .map_init(Scratch::new, |sc, chunk| {
            let (mut crossings, mut lost) = (0u64, 0u64);
            for ray in chunk {
                let Ok(mut st) = tracker.init_state(Point3::new(ray.o[0], ray.o[1], ray.o[2]), Vector3::new(ray.d[0], ray.d[1], ray.d[2]), None, sc) else {
                    lost += 1;
                    continue;
                };
                if st.cell.is_none() || (ray.start >= 0 && st.region.0 != ray.start as u32) {
                    lost += u64::from(ray.start >= 0);
                    continue;
                }
                for _ in 0..opts.max_events {
                    let Ok(s) = tracker.step(&mut st, f64::INFINITY, sc) else {
                        lost += 1;
                        break;
                    };
                    match s.end {
                        StepEnd::Lost(_) => {
                            lost += 1;
                            break;
                        }
                        StepEnd::Boundary { .. } => {
                            crossings += 1;
                            break;
                        }
                        StepEnd::RegionBoundary { to_cell, .. } => {
                            crossings += 1;
                            if to_cell.is_none() {
                                break;
                            }
                        }
                        StepEnd::Limit => break,
                        _ => crossings += 1,
                    }
                }
            }
            (crossings, lost)
        })
        .reduce(|| (0, 0), |a, b| (a.0 + b.0, a.1 + b.1));
    let secs = t1.elapsed().as_secs_f64().max(1e-12);
    println!(
        "{{\"build_s\": {build_s:.6}, \"rays\": {}, \"rays_per_s\": {:.1}, \"crossings_per_s\": {:.1}, \"lost\": {lost}, \"threads\": {}, \"cpu_model\": \"{}\"}}",
        rays.len(),
        rays.len() as f64 / secs,
        crossings as f64 / secs,
        rayon::current_num_threads(),
        cpu_model().replace('"', "'")
    );
    Ok(())
}
