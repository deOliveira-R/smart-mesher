//! Property tests (testkit strategies, pinned seed): invariance of every metric under rigid
//! motions, uniform scaling and mirroring, on randomly perturbed elements.
#![allow(missing_docs)]

mod common;
use common::*;
use proptest::prelude::*;
use sm_core::{Point3, Transform, Vector3};
use sm_mesh_api::CellShape;
use sm_quality::{Context, Metric, V, eval};
use sm_testkit::strategies::{config, proper_transform, reflection};

fn shapes() -> Vec<(CellShape, Vec<V>)> {
    ideal_elements()
}

/// Ideal element with each coordinate displaced by up to 0.12 (stays valid for unit elements).
fn perturbed(base: &[V], d: &[f64]) -> Vec<V> {
    base.iter().enumerate().map(|(i, p)| [p[0] + d[3 * i], p[1] + d[3 * i + 1], p[2] + d[3 * i + 2]]).collect()
}

fn map(t: &Transform, p: &[V]) -> Vec<V> {
    p.iter()
        .map(|q| {
            let r = t.apply_point(Point3::new(q[0], q[1], q[2]));
            [r.x, r.y, r.z]
        })
        .collect()
}

fn scaled(p: &[V], s: f64) -> Vec<V> {
    p.iter().map(|q| [q[0] * s, q[1] * s, q[2] * s]).collect()
}

fn close(a: f64, b: f64) -> bool {
    (a.is_nan() && b.is_nan()) || (a - b).abs() <= 1e-8 * (1.0 + a.abs().max(b.abs()))
}

fn ctx(shape: CellShape, p: &[V]) -> Context {
    Context { normal: None, ref_size: eval(Metric::Volume, shape, p, &Context::default()).unwrap().abs() * 1.3 + 0.1 }
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn rigid_motions_preserve_every_metric(t in proper_transform(), d in prop::collection::vec(-0.12f64..0.12, 24)) {
        for (shape, base) in shapes() {
            let p = perturbed(&base, &d);
            let q = map(&t, &p);
            let (c0, c1) = (ctx(shape, &p), ctx(shape, &q));
            for &m in Metric::ALL.iter().filter(|m| m.applies(shape) && **m != Metric::Starness) {
                let (a, b) = (eval(m, shape, &p, &c0).unwrap(), eval(m, shape, &q, &c1).unwrap());
                prop_assert!(close(a, b), "{m:?} on {shape:?}: {a} vs {b}");
            }
        }
    }

    #[test]
    fn uniform_scaling_only_changes_dimensional_metrics(e in -12.0f64..12.0, d in prop::collection::vec(-0.12f64..0.12, 24)) {
        for (shape, base) in shapes() {
            let s = 10f64.powf(e);
            let p = perturbed(&base, &d);
            let q = scaled(&p, s);
            let dim = if shape.dim() == sm_mesh_api::MeshDim::Two { 2 } else { 3 };
            let c0 = ctx(shape, &p);
            let c1 = Context { normal: None, ref_size: c0.ref_size * s.powi(dim) };
            for &m in Metric::ALL.iter().filter(|m| m.applies(shape)) {
                let (a, b) = (eval(m, shape, &p, &c0).unwrap(), eval(m, shape, &q, &c1).unwrap());
                let factor = match m {
                    Metric::Volume | Metric::Jacobian => s.powi(dim),
                    Metric::Dimension => s,
                    _ => 1.0,
                };
                prop_assert!(close(a * factor, b), "{m:?} on {shape:?}: {a} * {factor} vs {b}");
            }
        }
    }

    #[test]
    fn mirroring_with_the_reflect_permutation_preserves_metrics(r in reflection(), d in prop::collection::vec(-0.12f64..0.12, 24)) {
        for (shape, base) in shapes() {
            let Some(_) = shape.vertex_count() else { continue };
            if shape.dim() == sm_mesh_api::MeshDim::Two { continue } // 2-D: a mirror is a rotation about the normal, covered above
            let p = perturbed(&base, &d);
            let mirrored = map(&r, &p);
            // Without relabelling the cell is left-handed: every corner Jacobian flips sign.
            let sj = eval(Metric::ScaledJacobian, shape, &mirrored, &Context::default()).unwrap();
            // (The pyramid value is floored at 0, as in Verdict.)
            if shape == sm_mesh_api::CellShape::Pyramid {
                prop_assert!(sj == 0.0, "{shape:?}: sj {sj}");
            } else {
                prop_assert!(sj < 0.0, "{shape:?}: sj {sj}");
            }
            // The reflect permutation restores a right-handed element congruent to the original.
            let perm = shape.reflect_permutation();
            let fixed: Vec<V> = perm.iter().map(|&i| mirrored[i as usize]).collect();
            let (c0, c1) = (ctx(shape, &p), ctx(shape, &fixed));
            for &m in Metric::ALL.iter().filter(|m| m.applies(shape) && **m != Metric::Starness) {
                let (a, b) = (eval(m, shape, &p, &c0).unwrap(), eval(m, shape, &fixed, &c1).unwrap());
                prop_assert!(close(a, b), "{m:?} on {shape:?}: {a} vs {b}");
            }
        }
    }

    #[test]
    fn metrics_stay_inside_their_documented_bounds(d in prop::collection::vec(-0.12f64..0.12, 24)) {
        for (shape, base) in shapes() {
            let p = perturbed(&base, &d);
            let c = ctx(shape, &p);
            let v = |m| eval(m, shape, &p, &c).unwrap();
            for &m in Metric::ALL.iter().filter(|m| m.applies(shape)) {
                let x = v(m);
                prop_assert!(x.is_finite(), "{m:?} on {shape:?}");
                match m {
                    Metric::Shape | Metric::RelativeSizeSquared | Metric::ShapeAndSize | Metric::Shear | Metric::ShearAndSize
                    | Metric::Diagonal | Metric::Sphericity => prop_assert!((0.0..=1.0 + 1e-12).contains(&x), "{m:?} on {shape:?}: {x}"),
                    Metric::Skew => prop_assert!((0.0..=1.0 + 1e-12).contains(&x)),
                    Metric::ScaledJacobian if shape == sm_mesh_api::CellShape::Pyramid => prop_assert!((0.0..=1.0 + 1e-12).contains(&x), "pyramid sj {x}"),
                    Metric::AspectRatio | Metric::EdgeRatio | Metric::MaxEdgeRatio => prop_assert!(x >= 1.0 - 1e-12, "{m:?} {x}"),
                    Metric::Condition | Metric::MaxAspectFrobenius | Metric::MeanAspectFrobenius | Metric::AspectFrobenius => {
                        prop_assert!(x >= 1.0 - 1e-9, "{m:?} on {shape:?}: {x}");
                    }
                    Metric::MinAngle | Metric::MaxAngle => prop_assert!((0.0..=360.0).contains(&x)),
                    _ => {}
                }
            }
            // min <= max for angles; mean aspect <= max aspect.
            prop_assert!(v(Metric::MinAngle) <= v(Metric::MaxAngle) + 1e-12);
            if shape.vertex_count().is_some() && Metric::MeanAspectFrobenius.applies(shape) {
                prop_assert!(v(Metric::MeanAspectFrobenius) <= v(Metric::MaxAspectFrobenius) + 1e-12);
            }
        }
    }
}

#[test]
fn a_reflection_is_improper() {
    // Sanity: the strategy's transform type is the one the kernels see.
    let t = Transform::reflection(Vector3::new(1.0, 0.0, 0.0), 0.0).unwrap();
    assert!(t.is_improper());
}
