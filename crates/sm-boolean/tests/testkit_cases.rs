//! The `sm_testkit::boolean` cases, run through the ops and compared with their analytic
//! expectations (stage-0 rule: use the testkit cases, not local copies).

mod common;

use common::rig::*;
use common::*;
use sm_boolean::ops::{Imprint, Intersect, Merge, Subtract, Unite};
use sm_testkit::boolean::{BooleanCase, Expected, Operand, Rule, cases};
use sm_topo_api::Dim;

/// Build `op` in the rig. `None` for operands this kernel does not build yet (NURBS, tilted axes).
fn build(r: &mut Rig, op: &Operand) -> Option<Solid> {
    match *op {
        Operand::Brick { min, size } => Some(r.brick(size, [min[0] + size[0] / 2.0, min[1] + size[1] / 2.0, min[2] + size[2] / 2.0])),
        Operand::Cylinder { base, axis, radius, height } => {
            if axis != [0.0, 0.0, 1.0] {
                return None;
            }
            Some(r.cylinder(height, radius, [base[0], base[1], base[2] + height / 2.0]))
        }
        Operand::Sphere { center, radius } => Some(r.sphere(radius, center)),
        Operand::NurbsCylinder { .. } => None,
    }
}

fn close(got: f64, want: f64, what: &str, case: &str) {
    assert!((got - want).abs() <= 1e-8 * want.abs().max(1.0), "{case} {what}: got {got}, want {want}");
}

fn check(r: &Rig, e: &Expected, case: &str) {
    let t = r.topo();
    assert_valid(t);
    let mut vols: Vec<f64> = t.cells(Dim::Volume).iter().map(|&v| volume_of(t, v)).collect();
    vols.sort_by(f64::total_cmp);
    assert_eq!(vols.len(), e.volumes.len(), "{case} {:?}: volume count {vols:?}", e.rule);
    for (g, w) in vols.iter().zip(&e.volumes) {
        close(*g, *w, "volume", case);
    }
    close(total_area(t), e.total_area, "area", case);
    if let Some(c) = e.counts {
        assert_eq!(&counts(t)[..3], &c, "{case} {:?} counts", e.rule);
    }
    if let Some(eu) = e.euler {
        assert_eq!(euler(t), eu, "{case} {:?} euler", e.rule);
    }
}

fn run_rule(c: &BooleanCase, e: &Expected) -> bool {
    let mut r = Rig::cc();
    let (Some(a), Some(b)) = (build(&mut r, &c.a), build(&mut r, &c.b)) else { return false };
    match e.rule {
        Rule::Unite => {
            r.run(&Unite { bodies: vec![a.body, b.body], keep: None, regularize: None, tol: None }).unwrap_or_else(|x| panic!("{} unite: {x}", c.name));
        }
        Rule::Intersect => {
            r.run(&Intersect { bodies: vec![a.body, b.body], keep: None, regularize: None, tol: None }).unwrap_or_else(|x| panic!("{} intersect: {x}", c.name));
        }
        Rule::Subtract => {
            r.run(&Subtract { targets: vec![a.body], tools: vec![b.body], keep: None, regularize: None, tol: None }).unwrap_or_else(|x| panic!("{} subtract: {x}", c.name));
        }
        Rule::ImprintMerge => {
            r.run(&Imprint { bodies: vec![a.body, b.body], merge: Some(true), tol: None }).unwrap_or_else(|x| panic!("{} imprint: {x}", c.name));
            r.run(&Merge { entities: None, kinds: None, tol: None, check_only: None }).unwrap_or_else(|x| panic!("{} merge: {x}", c.name));
        }
    }
    check(&r, e, c.name);
    true
}

#[test]
fn every_testkit_case_matches_its_analytic_expectation() {
    let mut ran = Vec::new();
    for c in cases() {
        for e in &c.expected {
            if run_rule(&c, e) {
                ran.push((c.name, e.rule));
            }
        }
    }
    // The cases with a closed form and buildable operands, rule by rule.
    for want in [
        ("overlapping_bricks", Rule::Unite),
        ("overlapping_bricks", Rule::Intersect),
        ("overlapping_bricks", Rule::Subtract),
        ("brick_minus_cylinder", Rule::Subtract),
        ("brick_with_cavity", Rule::Subtract),
        ("partial_overlap_bricks", Rule::ImprintMerge),
        ("brick_sphere_half", Rule::Unite),
        ("brick_sphere_half", Rule::Intersect),
        ("brick_sphere_half", Rule::Subtract),
    ] {
        assert!(ran.contains(&want), "{want:?} not run: {ran:?}");
    }
}
