//! Acceptance 3: mean-ratio optimisation, objective monotonicity, tet lattice quality.

#![allow(clippy::needless_range_loop)]

mod common;

use common::*;
use sm_quality::{Metric, Options, compute, summarize};
use sm_smooth::{Method, Objective, Schedule};
use sm_testkit::lattice;

fn shapes(f: &lattice::LatticeFixture, pts: &[[f64; 3]]) -> Vec<f64> {
    let mut m = f.mesh.clone();
    set_points(&mut m, pts);
    compute(&m, Metric::Shape, &Options::default()).expect("shape")
}

fn sum_beta(s: &[f64], p: f64) -> f64 {
    s.iter().map(|x| (1.0 / x).powf(p)).sum()
}

fn opt(p: f64) -> Method {
    Method::Optimize { objective: Objective::MeanRatio, p }
}

#[test]
fn a3_optimize_reaches_shape_099_on_the_jittered_hex_lattice() {
    let f = lattice::hex([6, 6, 6], [1.0; 3]).expect("lattice");
    let p = f.perturbed(3, 0.2).expect("perturb");
    let s0 = summarize(&shapes(&p, &pts_of(&p.mesh)));
    eprintln!("a3 hex initial min shape {}", s0.min);
    assert!(s0.min < 0.9, "precondition: min shape {}", s0.min);
    let o = run(&p, &req(opt(4.0), Schedule::Colored, 300, false));
    let s1 = summarize(&shapes(&p, &o.points));
    eprintln!("a3 hex final min shape {} mean {} after {} sweeps, rejected guard {} geom {}", s1.min, s1.mean, o.report.iterations_run, o.report.rejected_guard, o.report.rejected_geometry);
    assert!(s1.min >= 0.99, "min shape {}", s1.min);
}

#[test]
fn a3_colored_objective_never_increases() {
    // One sweep at a time, feeding the outcome back in (the classification carries the hints).
    let f = lattice::hex([6, 6, 6], [1.0; 3]).expect("lattice");
    let mut p = f.perturbed(3, 0.2).expect("perturb");
    let mut last = sum_beta(&shapes(&p, &pts_of(&p.mesh)), 4.0);
    for k in 1..=40u32 {
        let o = run(&p, &req(opt(4.0), Schedule::Colored, 1, false));
        p.mesh = applied(&p, &o);
        let now = sum_beta(&shapes(&p, &pts_of(&p.mesh)), 4.0);
        assert!(now <= last * (1.0 + 1e-12), "sweep {k}: {now} > {last}");
        last = now;
    }
}

#[test]
fn a3_condition_objective_reaches_max_condition_101() {
    let f = lattice::hex([6, 6, 6], [1.0; 3]).expect("lattice");
    let p = f.perturbed(3, 0.2).expect("perturb");
    let cond = |pts: &[[f64; 3]]| {
        let mut m = p.mesh.clone();
        set_points(&mut m, pts);
        summarize(&compute(&m, Metric::Condition, &Options::default()).expect("condition")).max
    };
    let c0 = cond(&pts_of(&p.mesh));
    let o = run(&p, &req(Method::Optimize { objective: Objective::Condition, p: 4.0 }, Schedule::Colored, 300, false));
    let c1 = cond(&o.points);
    eprintln!("a3 stretch hex condition: max {c0} -> {c1} in {} sweeps", o.report.iterations_run);
    assert!(c1 <= 1.01, "max condition {c1}");
    assert!((o.report.after.max_condition - c1).abs() < 1e-12);
}

#[test]
fn a3_tet_lattice_keeps_the_unperturbed_mean_shape() {
    let f = lattice::tet_kuhn([4, 4, 4], [1.0; 3]).expect("lattice");
    let ideal = summarize(&shapes(&f, &f.exact)).mean;
    let p = f.perturbed(5, 0.2).expect("perturb");
    let start = summarize(&shapes(&p, &pts_of(&p.mesh))).mean;
    eprintln!("a3 tet ideal mean {ideal} perturbed mean {start}");
    for (name, m) in [("smart", Method::SmartLaplacian { relax: 1.0 }), ("optimize", opt(4.0))] {
        let o = run(&p, &req(m, Schedule::Colored, 300, false));
        let mean = summarize(&shapes(&p, &o.points)).mean;
        eprintln!("a3 tet {name}: mean {mean} after {} sweeps", o.report.iterations_run);
        assert!(mean >= ideal - 1e-3, "{name}: {mean} < {ideal}");
    }
}
