//! Acceptance 16: record-only numbers for the analytic tracker (no gate, wall clock only
//! printed). Run `cargo test --release -p sm-rt --test analytic_bench -- --ignored --nocapture`.
#[path = "common/csg.rs"]
mod csg;

use csg::*;
use sm_geom_api::Frame;
use sm_rt_api::{Scratch, StepEnd, Tracker};
use std::time::Instant;

fn segments(t: &dyn Tracker, lo: f64, hi: f64, n: usize, seed: &str) -> (u64, Scratch, f64) {
    let mut sc = Scratch::new();
    let mut r = rng(seed);
    let mut segs = 0u64;
    let start = Instant::now();
    for _ in 0..n {
        let o = p3(lo + (hi - lo) * (r.unit() + 1.0) / 2.0, lo + (hi - lo) * (r.unit() + 1.0) / 2.0, lo + (hi - lo) * (r.unit() + 1.0) / 2.0);
        let mut st = t.init_state(o, random_dir(&mut r), None, &mut sc).unwrap();
        for _ in 0..1000 {
            let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
            segs += 1;
            if matches!(s.end, StepEnd::Boundary { .. } | StepEnd::Lost(_)) {
                break;
            }
        }
    }
    (segs, sc, start.elapsed().as_secs_f64())
}

#[test]
#[ignore = "record-only benchmark"]
fn record_numbers() {
    let mut fx = Fx::new(10.0);
    fx.complement = sm_rt_api::ComplementPolicy::Outside;
    fx.bounds = None;
    let bx = fx.box_planes(p3(-1.26, -1.26, -50.0), p3(1.26, 1.26, 50.0));
    let f = Frame::at(p3(0.0, 0.0, 0.0));
    let cyl: Vec<_> = (1..=12).map(|k| fx.cylinder(f, 0.1 * f64::from(k))).collect();
    let b: Vec<_> = bx.iter().map(|&p| hs(p, true)).collect();
    let mut c0 = b.clone();
    c0.push(hs(cyl[0], true));
    fx.cell1("r0", &c0);
    for k in 1..12 {
        let mut h = b.clone();
        h.push(hs(cyl[k - 1], false));
        h.push(hs(cyl[k], true));
        fx.cell1(&format!("r{k}"), &h);
    }
    let mut h = b.clone();
    h.push(hs(cyl[11], false));
    fx.cell1("mod", &h);
    let t = fx.build();
    let (segs, sc, secs) = segments(&t, -1.0, 1.0, 20_000, "bench-pin");
    println!(
        "12-ring pin cell: {:.2e} segments/s, {:.2} surface evals per segment, slow_locates {} per 1e6 segments",
        segs as f64 / secs,
        sc.counters.surface_evals as f64 / segs as f64,
        sc.counters.slow_locates as f64 * 1e6 / segs as f64
    );
    let mut sc = Scratch::new();
    let mut r = rng("bench-locate");
    let pts: Vec<_> = (0..200_000).map(|_| p3(r.unit() * 1.2, r.unit() * 1.2, 0.0)).collect();
    let dir = v3(0.0, 0.0, 1.0);
    let s = Instant::now();
    for &p in &pts {
        let _ = t.locate(p, Some(dir), None, &mut sc).unwrap();
    }
    let fast = pts.len() as f64 / s.elapsed().as_secs_f64();
    let s = Instant::now();
    for &p in &pts {
        let _ = t.locate_generic(p, Some(dir), &mut sc).unwrap();
    }
    let slow = pts.len() as f64 / s.elapsed().as_secs_f64();
    println!("locate: {fast:.2e} points/s with the concentric fast path, {slow:.2e} without");

    let mut fx = Fx::new(40.0);
    let mut r = rng("bench-torus");
    let ft = random_frame(&mut r, p3(0.0, 0.0, 0.0));
    let tor = fx.torus(ft, 3.0, 1.0);
    fx.cell1("ring", &[hs(tor, true)]);
    let t = fx.build();
    let mut sc = Scratch::new();
    let n = 100_000;
    let (mut chords, start) = (0u64, Instant::now());
    for _ in 0..n {
        let d = random_dir(&mut r);
        let e = perp_to(&mut r, d);
        let tr = trace(&t, p3(0.0, 0.0, 0.0) + (e * (3.5 * r.unit().abs()) - d * 30.0), d, f64::INFINITY, &mut sc);
        chords += tr.chords().len() as u64;
    }
    println!("torus: {:.2e} lines/s ({chords} chords), including the trace harness", f64::from(n) / start.elapsed().as_secs_f64());
}
