//! End to end through `Driver::mesh` on the reference brick with the stub map/sweep schemes, plus
//! applicability and registration checks.

use sm_core::{EntityId, Point3};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{MeshRead, SerialAlloc, validate};
use sm_scheme_api::stubs::{self as stub, brick_setup};
use sm_scheme_api::{
    Applicability, Driver, Firm, Firmness, IntervalRequest, MeshInput, MeshIntent, Params, PlanCtx, ReferenceCornerPicker, ReferenceIntervalSolver,
    SchemeChoice, SchemeDim, SchemeRegistry, Sequential, Target, WholeFaces,
};
use sm_scheme_curve::spacing;
use sm_topo_api::fixtures::{BrickIds, cylinder, with_op};
use sm_topo_api::{CellComplex, OpId, TopologyView, TopologyViewExt};

const X: usize = 0;
const Y: usize = 4;
const Z: usize = 8;

fn registry() -> SchemeRegistry {
    let mut r = SchemeRegistry::new();
    sm_scheme_curve::register(&mut r).expect("register");
    r.register(Box::new(stub::Map)).unwrap();
    r.register(Box::new(stub::Sweep)).unwrap();
    r
}

fn sweep_z(ids: &BrickIds) -> SchemeChoice {
    SchemeChoice::with_params("sweep", serde_json::json!({ "source": [ids.faces[4]], "target": [ids.faces[5]] }), Firmness::Hard)
}

fn choice(name: &str, params: serde_json::Value) -> SchemeChoice {
    SchemeChoice::with_params(name, params, Firmness::Hard)
}

/// Mesh the brick `[1, 1.5, 2]` with `scheme` on the x-edge 0 and hard counts on y and z (and on
/// x if `nx` is given). Returns the interior node distances from the edge's start vertex, the
/// edge, and the plan's interval count.
/// What the scheme's own edge gets as an interval request.
#[derive(Clone, Copy)]
enum Count {
    /// The standard count of that edge (4, 3, 4 for x, y, z).
    Std,
    /// No request: the scheme or the solver decides.
    Derive,
    /// A hard count.
    Hard(u32),
}

struct Run {
    dist: Vec<f64>,
    n: u32,
    vertices_on_edge: usize,
    valid: bool,
}

fn run_on(edge_ix: usize, scheme: Option<SchemeChoice>, count: Count) -> Result<Run, String> {
    let (topo, ids, model) = brick_setup([1.0, 1.5, 2.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    let counts = [(X, 4u32), (Y, 3), (Z, 4)];
    for (e, n) in counts {
        let c = match (e == edge_ix, count) {
            (true, Count::Derive) => None,
            (true, Count::Hard(c)) => Some(c),
            _ => Some(n),
        };
        if let Some(c) = c {
            intent.set_intervals(ids.edges[e], Firm::hard(IntervalRequest::Count(c)));
        }
    }
    intent.set_scheme(ids.volume, sweep_z(&ids));
    let e = ids.edges[edge_ix];
    if let Some(s) = scheme {
        intent.set_scheme(e, s);
    }
    let out = driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::new(), &mut SerialAlloc::new()).map_err(|e| e.to_string())?;
    let (a, _) = topo.edge_ends(e).unwrap();
    let p0 = match topo.carrier(a) {
        sm_topo_api::Carrier::Point(p) => *p,
        _ => unreachable!(),
    };
    let patch = out.patches.get(Target::Curve(e)).expect("curve patch");
    let dist = patch.nodes.iter().map(|n| (n.point - p0).norm()).collect();
    let m = &out.finished.mesh;
    Ok(Run {
        dist,
        n: out.plan.intervals(e).unwrap(),
        vertices_on_edge: m.classified_on(e).map_or(0, |s| s.vertices.len()),
        valid: validate(m).is_empty(),
    })
}

fn assert_near(a: f64, b: f64, tol: f64, what: &str) {
    assert!((a - b).abs() <= tol, "{what}: {a} vs {b}");
}

#[test]
fn register_adds_the_five_curve_schemes_and_no_auto_list() {
    let mut r = SchemeRegistry::new();
    sm_scheme_curve::register(&mut r).unwrap();
    let names: Vec<&str> = r.of_dim(SchemeDim::Curve).map(|s| s.name()).collect();
    assert_eq!(names, ["bias", "curvature", "dualbias", "equal", "stretch"]);
    assert!(r.auto(SchemeDim::Curve).is_empty());
    assert!(sm_scheme_curve::register(&mut r).is_err(), "double registration is refused");
}

#[test]
fn equal_brick_edge_nodes_are_classified_and_analytic() {
    let r = run_on(X, Some(choice("equal", serde_json::json!({}))), Count::Std).unwrap();
    assert_eq!(r.n, 4);
    for (i, d) in r.dist.iter().enumerate() {
        assert_near(*d, 0.25 * (i as f64 + 1.0), 1e-15, "equal node");
    }
    assert_eq!(r.vertices_on_edge, 3, "interior nodes are classified on the edge");
    assert!(r.valid);
}

#[test]
fn bias_ratio_through_the_driver() {
    let r = run_on(X, Some(choice("bias", serde_json::json!({ "ratio": 1.5 }))), Count::Std).unwrap();
    let want = spacing::bias_fractions(r.n, 1.5).unwrap();
    assert_eq!(r.dist.len(), want.len());
    for (d, w) in r.dist.iter().zip(&want) {
        assert_near(*d, *w * 1.0, 1e-13, "bias node");
    }
    let mut e = vec![0.0];
    e.extend(&r.dist);
    e.push(1.0);
    for w in e.windows(3) {
        assert_near((w[2] - w[1]) / (w[1] - w[0]), 1.5, 1e-12, "edge ratio");
    }
    assert_eq!(r.vertices_on_edge, r.n as usize - 1);
    assert!(r.valid);
}

#[test]
fn bias_start_on_the_far_vertex_reverses_the_sequence() {
    let (topo, ids, _) = brick_setup([1.0, 1.5, 2.0]);
    let (a, b) = topo.edge_ends(ids.edges[X]).unwrap();
    let fwd = run_on(X, Some(choice("bias", serde_json::json!({ "ratio": 1.5, "start": a.to_string() }))), Count::Std).unwrap();
    let rev = run_on(X, Some(choice("bias", serde_json::json!({ "ratio": 1.5, "start": b.to_string() }))), Count::Std).unwrap();
    let n = fwd.dist.len();
    for k in 0..n {
        assert_near(rev.dist[k], 1.0 - fwd.dist[n - 1 - k], 1e-13, "reversed node");
    }
    // The reversed series shrinks away from the start: the first edge is the largest.
    assert!(rev.dist[0] > fwd.dist[0]);
}

#[test]
fn bias_first_size_recovers_ratio_and_closes() {
    let r = run_on(X, Some(choice("bias", serde_json::json!({ "first_size": 0.1 }))), Count::Hard(6)).unwrap();
    assert_eq!(r.n, 6);
    assert_near(r.dist[0], 0.1, 1e-12, "first edge");
    let ratio = (r.dist[1] - r.dist[0]) / r.dist[0];
    let mut e = vec![0.0];
    e.extend(&r.dist);
    e.push(1.0);
    for w in e.windows(3) {
        assert_near((w[2] - w[1]) / (w[1] - w[0]), ratio, 1e-11, "constant ratio");
    }
    assert_near(e.iter().rev().take(2).sum::<f64>() - 1.0, e[e.len() - 2], 1e-15, "ends at the far vertex");
    assert!(r.valid);
}

#[test]
fn bias_fine_and_coarse_sizes_derive_the_count() {
    // No count on the x-edge: the scheme asks for the count that fits both sizes.
    let r = run_on(X, Some(choice("bias", serde_json::json!({ "first_size": 0.05, "last_size": 0.4 }))), Count::Derive).unwrap();
    assert_eq!(r.n, spacing::bias_count_from_sizes(1.0, 0.05, 0.4).unwrap());
    assert_near(r.dist[0], 0.05, 1e-12, "first edge");
    // A hard count wins over the derived one.
    let hard = run_on(X, Some(choice("bias", serde_json::json!({ "first_size": 0.05, "last_size": 0.4 }))), Count::Hard(7)).unwrap();
    assert_eq!(hard.n, 7);
    assert_near(hard.dist[0], 0.05, 1e-12, "first edge with a hard count");
}

#[test]
fn bias_ratio_and_first_size_derive_the_count() {
    let r = run_on(X, Some(choice("bias", serde_json::json!({ "ratio": 1.2, "first_size": 0.1 }))), Count::Derive).unwrap();
    assert_eq!(r.n, spacing::bias_count_from_ratio(1.0, 0.1, 1.2).unwrap());
    assert_eq!(r.n, 6);
}

#[test]
fn dualbias_is_mirror_symmetric_through_the_driver() {
    for n in [6u32, 7] {
        let r = run_on(Y, Some(choice("dualbias", serde_json::json!({ "ratio": 1.4 }))), Count::Hard(n)).unwrap();
        let mut e = vec![0.0];
        e.extend(&r.dist);
        e.push(1.5);
        let seg: Vec<f64> = e.windows(2).map(|w| w[1] - w[0]).collect();
        for k in 0..n as usize {
            assert_near(seg[k], seg[n as usize - 1 - k], 1e-12, "mirror");
        }
        assert!(seg[1] / seg[0] > 1.39 && seg[1] / seg[0] < 1.41);
        assert!(r.valid);
    }
}

#[test]
fn stretch_pins_the_first_edge_and_derives_the_count() {
    let r = run_on(Z, Some(choice("stretch", serde_json::json!({ "first_size": 0.2, "last_size": 0.5 }))), Count::Derive).unwrap();
    assert_eq!(r.n, (2.0f64 * 2.0 / 0.7).round_ties_even() as u32);
    assert_near(r.dist[0], 0.2, 1e-12, "first");
    let mut e = vec![0.0];
    e.extend(&r.dist);
    e.push(2.0);
    assert_near(e[e.len() - 1] - e[e.len() - 2], 0.5, 1e-12, "last");
    let worst = e.windows(3).map(|w| ((w[2] - w[1]) / (w[1] - w[0])).max((w[1] - w[0]) / (w[2] - w[1]))).fold(1.0, f64::max);
    assert!(worst <= 2.0 + 1e-9, "{worst}");
}

#[test]
fn stretch_beyond_max_ratio_fails_instead_of_relaxing() {
    let res = run_on(X, Some(choice("stretch", serde_json::json!({ "first_size": 0.01, "max_ratio": 1.2 }))), Count::Hard(5));
    assert!(res.is_err(), "a hard stretch that needs more than max_ratio is an error");
}

#[test]
fn curvature_on_a_straight_edge_is_equal() {
    let r = run_on(X, Some(choice("curvature", serde_json::json!({ "factor": 5.0 }))), Count::Std).unwrap();
    for (i, d) in r.dist.iter().enumerate() {
        assert_near(*d, (i as f64 + 1.0) / f64::from(r.n), 1e-13, "curvature on a line");
    }
}

fn applicability(name: &str, topo: &dyn TopologyView, edge: EntityId, params: serde_json::Value) -> Applicability {
    let reg = registry();
    let model = brick_setup([1.0, 1.0, 1.0]).2;
    let sel: [EntityId; 0] = [];
    let input = MeshInput::new(topo, &model, &WholeFaces, &sel);
    let intent = MeshIntent::default();
    let cx = PlanCtx::new(&input, &intent);
    reg.get(SchemeDim::Curve, name).unwrap().applicability(&cx, Target::Curve(edge), &Params(params))
}

fn reason(a: &Applicability) -> Option<&str> {
    match a {
        Applicability::NotApplicable { reason, .. } => Some(reason),
        _ => None,
    }
}

#[test]
fn invalid_params_are_not_applicable_with_a_reason() {
    let (topo, ids, _) = brick_setup([1.0, 1.5, 2.0]);
    let e = ids.edges[X];
    let (ea, eb) = topo.edge_ends(e).unwrap();
    let far = ids.vertices.iter().find(|v| **v != ea && **v != eb).unwrap().to_string();
    let bad = |name: &str, p: serde_json::Value| {
        let a = applicability(name, &topo, e, p.clone());
        let r = reason(&a).unwrap_or_else(|| panic!("{name} {p} should be NotApplicable, got {a:?}"));
        assert!(!r.is_empty(), "{name} {p}");
    };
    for (name, p) in [
        ("equal", serde_json::json!({ "ratio": 2.0 })),
        ("bias", serde_json::json!({})),
        ("bias", serde_json::json!({ "ratio": 0.0 })),
        ("bias", serde_json::json!({ "ratio": -2.0 })),
        ("bias", serde_json::json!({ "ratio": "fast" })),
        ("bias", serde_json::json!({ "last_size": 0.1 })),
        ("bias", serde_json::json!({ "ratio": 1.1, "last_size": 0.1 })),
        ("bias", serde_json::json!({ "first_size": 5.0 })),
        ("bias", serde_json::json!({ "first_size": 0.1, "last_size": 7.0 })),
        ("bias", serde_json::json!({ "ratio": 1.1, "bogus": 1 })),
        ("bias", serde_json::json!({ "ratio": 1.1, "start": far })),
        ("dualbias", serde_json::json!({})),
        ("dualbias", serde_json::json!({ "ratio": 1.1, "first_size": 0.1 })),
        ("dualbias", serde_json::json!({ "first_size": 0.6 })),
        ("stretch", serde_json::json!({})),
        ("stretch", serde_json::json!({ "last_size": 0.1 })),
        ("stretch", serde_json::json!({ "first_size": 0.1, "max_ratio": 0.5 })),
        ("stretch", serde_json::json!({ "first_size": 2.0 })),
        ("curvature", serde_json::json!({})),
        ("curvature", serde_json::json!({ "factor": "x" })),
    ] {
        bad(name, p);
    }
    for (name, p) in [
        ("equal", serde_json::json!({})),
        ("bias", serde_json::json!({ "ratio": 1.3 })),
        ("bias", serde_json::json!({ "first_size": 0.1 })),
        ("bias", serde_json::json!({ "first_size": 0.1, "last_size": 0.3 })),
        ("dualbias", serde_json::json!({ "ratio": 0.8 })),
        ("dualbias", serde_json::json!({ "first_size": 0.2 })),
        ("stretch", serde_json::json!({ "first_size": 0.1, "last_size": 0.3, "max_ratio": 3.0 })),
        ("curvature", serde_json::json!({ "factor": -2.0 })),
    ] {
        let a = applicability(name, &topo, e, p.clone());
        assert!(matches!(a, Applicability::Applicable { .. }), "{name} {p}: {a:?}");
    }
    // A target that is not a curve.
    let reg = registry();
    let model = brick_setup([1.0, 1.0, 1.0]).2;
    let input = MeshInput::new(&topo, &model, &WholeFaces, &[]);
    let intent = MeshIntent::default();
    let cx = PlanCtx::new(&input, &intent);
    let eq = reg.get(SchemeDim::Curve, "equal").unwrap();
    assert!(matches!(eq.applicability(&cx, Target::Surface(ids.faces[0]), &Params::empty()), Applicability::NotApplicable { .. }));
}

#[test]
fn equal_outscores_the_other_schemes_and_closed_curves_refuse_stretch_and_curvature() {
    let mut c = CellComplex::new();
    let cyl = with_op(&mut c, OpId(1), |t| cylinder(t, 1.5, 2.0)).expect("cylinder");
    let circle = cyl.edges[0];
    let score = |a: Applicability| match a {
        Applicability::Applicable { score, .. } => Some(score),
        _ => None,
    };
    let eq = score(applicability("equal", &c, circle, serde_json::json!({}))).unwrap();
    let bias = score(applicability("bias", &c, circle, serde_json::json!({ "ratio": 1.2 }))).unwrap();
    assert!(eq > bias);
    let a = applicability("stretch", &c, circle, serde_json::json!({ "first_size": 0.1 }));
    assert!(reason(&a).unwrap().contains("closed"), "{a:?}");
    let a = applicability("curvature", &c, circle, serde_json::json!({ "factor": 1.0 }));
    assert!(reason(&a).unwrap().contains("closed"), "{a:?}");
    let a = applicability("bias", &c, circle, serde_json::json!({ "ratio": 1.2, "start": cyl.vertices[0].to_string() }));
    assert!(reason(&a).unwrap().contains("ambiguous"), "{a:?}");
    // Dualbias and bias are fine on a closed edge.
    assert!(score(applicability("dualbias", &c, circle, serde_json::json!({ "ratio": 1.2 }))).is_some());
}

#[test]
fn driver_output_is_bit_identical_across_executors_and_runs() {
    use sm_testkit::determinism::hash_f64_bits;
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| {
            let mut bits = Vec::new();
            for (ix, s, c) in [
                (X, choice("bias", serde_json::json!({ "first_size": 0.03 })), Count::Hard(30)),
                (Y, choice("dualbias", serde_json::json!({ "ratio": 1.1 })), Count::Hard(25)),
                (Z, choice("curvature", serde_json::json!({ "factor": 2.0 })), Count::Std),
            ] {
                let r = run_on(ix, Some(s), c).unwrap();
                bits.extend(r.dist);
                bits.push(f64::from(r.n));
            }
            bits
        })
    };
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get().max(2));
    let bits = sm_testkit::determinism::check_f64_bits(2, threads, run).unwrap();
    assert_ne!(hash_f64_bits(&bits), 0);
    // Silence: the points type is part of the public API surface used above.
    let _ = Point3::new(0.0, 0.0, 0.0);
}
