//! Random voxel-shape fuzz: planar, CAD-like closed surfaces must tetrahedralize.
mod common;
use common::*;
use sm_scheme_tet::{SurfaceInput, TetOptions, tetrahedralize};
use sm_testkit::surf3;
use std::collections::BTreeMap;

/// Boundary surface of a voxel set, each unit quad subdivided `sub x sub` with random diagonals.
fn voxel_surface(occ: &[bool], n: usize, sub: usize, r: &mut surf3::SplitMix64) -> SurfaceInput {
    let at = |x: i64, y: i64, z: i64| -> bool {
        if x < 0 || y < 0 || z < 0 || x >= n as i64 || y >= n as i64 || z >= n as i64 { false } else { occ[(x as usize * n + y as usize) * n + z as usize] }
    };
    let mut pts: Vec<[f64; 3]> = Vec::new();
    let mut idx: BTreeMap<[i64; 3], u32> = BTreeMap::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();
    let s = sub as i64;
    let mut vid = |p: [i64; 3], pts: &mut Vec<[f64; 3]>| -> u32 {
        *idx.entry(p).or_insert_with(|| {
            pts.push([p[0] as f64 / s as f64, p[1] as f64 / s as f64, p[2] as f64 / s as f64]);
            (pts.len() - 1) as u32
        })
    };
    for x in 0..n as i64 {
        for y in 0..n as i64 {
            for z in 0..n as i64 {
                if !at(x, y, z) { continue; }
                for (axis, dir) in [(0, -1i64), (0, 1), (1, -1), (1, 1), (2, -1), (2, 1)] {
                    let mut nb = [x, y, z];
                    nb[axis] += dir;
                    if at(nb[0], nb[1], nb[2]) { continue; }
                    // Face of the voxel on side (axis, dir); (u,v) axes.
                    let (ua, va) = ((axis + 1) % 3, (axis + 2) % 3);
                    let base = [x * s, y * s, z * s];
                    for i in 0..s {
                        for j in 0..s {
                            let corner = |du: i64, dv: i64| -> [i64; 3] {
                                let mut p = base;
                                if dir == 1 { p[axis] += s; }
                                p[ua] += i + du;
                                p[va] += j + dv;
                                p
                            };
                            let (a, b, c, d) = (corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1));
                            let (a, b, c, d) = (vid(a, &mut pts), vid(b, &mut pts), vid(c, &mut pts), vid(d, &mut pts));
                            // (u,v,axis) is a right-handed frame: normal of (a,b,c) is +axis.
                            let mut quad = [a, b, c, d];
                            if dir == -1 { quad = [a, d, c, b]; }
                            if r.next_u64() & 1 == 0 {
                                tris.push([quad[0], quad[1], quad[2]]);
                                tris.push([quad[0], quad[2], quad[3]]);
                            } else {
                                tris.push([quad[0], quad[1], quad[3]]);
                                tris.push([quad[1], quad[2], quad[3]]);
                            }
                        }
                    }
                }
            }
        }
    }
    SurfaceInput { points: pts, tris }
}

/// Quick deterministic sweep (a few valid cases per configuration).
#[test]
fn voxel_fuzz_small() {
    // Debug builds run fewer cases (the exact predicates are an order of magnitude slower).
    sweep(if cfg!(debug_assertions) { 2 } else { 5 });
}

/// Larger sweep: `cargo test --release -- --ignored`.
#[test]
#[ignore]
fn voxel_fuzz_large() {
    sweep(60);
}

fn sweep(per_config: usize) {
    let mut ok = 0;
    let mut nonman = 0;
    let mut fails: Vec<String> = Vec::new();
    let mut slow = 0;
    let mut total_s = 0.0;
    for n in [3usize, 4] {
        for sub in [1usize, 2] {
            let mut valid = 0;
            for seed in 0..20000u64 {
                if valid >= per_config { break; }
                let mut r = surf3::SplitMix64(seed * 31 + n as u64 * 7 + sub as u64);
                let occ: Vec<bool> = (0..n * n * n).map(|_| r.next_u64() % 100 < 55).collect();
                if !occ.iter().any(|&b| b) { continue; }
                let inp = voxel_surface(&occ, n, sub, &mut r);
                let t0 = std::time::Instant::now();
                let res = tetrahedralize(&inp, &TetOptions::default());
                let el = t0.elapsed().as_secs_f64();
                total_s += el;
                if el > 1.0 { slow += 1; eprintln!("SLOW n={n} sub={sub} seed={seed}: {el:.1}s"); }
                match res {
                    Ok(m) => {
                        valid += 1;
                        let want = surf3::polyhedron_volume(&inp.points, &inp.tris);
                        let v = total_volume(&m);
                        if !all_exact_positive(&m) || check_boundary(&m, &inp.tris).is_err() || ((v - want) / want).abs() > 1e-9 {
                            fails.push(format!("INVALID n={n} sub={sub} seed={seed}"));
                        } else { ok += 1; }
                    }
                    Err(sm_scheme_tet::TetError::NonManifold { .. }) | Err(sm_scheme_tet::TetError::NotClosed { .. }) => nonman += 1,
                    Err(e) => { valid += 1; fails.push(format!("n={n} sub={sub} seed={seed}: {}", e.to_string().chars().take(90).collect::<String>())) },
                }
            }
        }
    }
    eprintln!("ok={ok} nonmanifold={nonman} slow={slow} total {total_s:.1}s");
    assert!(fails.is_empty(), "failures: {fails:?}");
}
