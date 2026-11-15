//! W4-HEX-PYR: the hex-tet transition by pyramids (`transition: "pyramid"`), through the
//! driver with the real quad `map` schemes on the hex side and `trimesh` on the tet side.
//!
//! Layout: hex brick A = `[0,1]^3` (n x n x n hexes), tet brick B = `[1,2] x [0,1]^2`, merged on
//! the plane `x = 1`. The interface has n^2 quads, so n^2 pyramids of base edge s = 1/n and
//! height h = 0.6 s: volume `s^2 h / 3` each.

use sm_core::EntityId;
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::{CellIx, CellShape, MeshEntityKind, MeshRead, SerialAlloc, cell_volume, validate};
use sm_quality::{Metric, Options, compute};
use sm_scheme_api::stubs::Equal;
use sm_scheme_api::{
    Caps, Driver, Executor, Firm, Firmness, IntervalRequest, MeshInput, MeshIntent, Meshed, ReferenceCornerPicker, ReferenceIntervalSolver, SchemeChoice, SchemeError,
    SchemeRegistry, ScopedThreads, Sequential, SizingSpec, Target, WholeFaces,
};
use sm_scheme_tet::{NAME, TetParams, Transition};
use sm_topo_api::fixtures::{BrickIds, brick, merge_touching_bricks, two_bricks, with_op};
use sm_topo_api::{CellComplex, OpId, TopologyEdit};

type Out = Meshed<ReferenceMesh>;

fn registry() -> SchemeRegistry {
    let mut r = SchemeRegistry::new();
    r.register(Box::new(Equal)).unwrap();
    sm_scheme_map::register(&mut r).unwrap();
    sm_scheme_mapvol::register(&mut r).unwrap();
    sm_scheme_tri::register(&mut r).unwrap();
    sm_scheme_tet::register(&mut r).unwrap();
    r
}

fn setup() -> (CellComplex, BrickIds, BrickIds) {
    let mut topo = CellComplex::new();
    let (a, b) = with_op(&mut topo, OpId(1), |t| {
        let (a, b) = two_bricks(t)?;
        merge_touching_bricks(t, &a, &b)?;
        Ok((a, b))
    })
    .unwrap();
    (topo, a, b)
}

fn intent(a: &BrickIds, b: &BrickIds, n: u32, tet_params: serde_json::Value) -> MeshIntent {
    let mut intent = MeshIntent::default();
    intent.defaults.sizing = Some(SizingSpec::Constant { size: 1.0 / f64::from(n) });
    intent.set_scheme(a.volume, SchemeChoice::new("map", Firmness::Hard));
    intent.set_scheme(b.volume, SchemeChoice::with_params(NAME, tet_params, Firmness::Hard));
    for ids in [a, b] {
        for first in [0usize, 4, 8] {
            intent.set_intervals(ids.edges[first], Firm::hard(IntervalRequest::Count(n)));
        }
    }
    intent
}

fn run(topo: &CellComplex, vols: &[EntityId], intent: &MeshIntent, exec: &dyn Executor) -> Result<Out, SchemeError> {
    let r = registry();
    let model = sm_model_api::Model::new();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&r, &solver, &picker);
    let input = MeshInput::new(topo, &model, &WholeFaces, vols);
    driver.mesh(&input, intent, exec, ReferenceBuilder::new(), &mut SerialAlloc::new())
}

fn count_shape(m: &ReferenceMesh, s: CellShape) -> usize {
    (0..m.count(MeshEntityKind::Cell)).filter(|&c| m.cell_shape(CellIx(c as u32)) == s).count()
}

fn volume_of(m: &ReferenceMesh, s: CellShape) -> f64 {
    (0..m.count(MeshEntityKind::Cell)).filter(|&c| m.cell_shape(CellIx(c as u32)) == s).map(|c| cell_volume(m, CellIx(c as u32))).sum()
}

fn min_sj(m: &ReferenceMesh, s: CellShape) -> f64 {
    let all = compute(m, Metric::ScaledJacobian, &Options::default()).expect("quality");
    (0..m.count(MeshEntityKind::Cell)).filter(|&c| m.cell_shape(CellIx(c as u32)) == s).map(|c| all[c]).fold(f64::INFINITY, f64::min)
}

fn pyramid_params() -> serde_json::Value {
    serde_json::json!({"transition": "pyramid"})
}

#[test]
fn hex_tet_by_pyramids_is_conforming_with_analytic_volumes() {
    let n = 4u32;
    let (topo, a, b) = setup();
    let it = intent(&a, &b, n, pyramid_params());
    let out = run(&topo, &[a.volume, b.volume], &it, &Sequential).unwrap_or_else(|e| panic!("{e}"));
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    sm_testkit::check::mesh_valid(m).unwrap();
    sm_testkit::check::classification_complete(m).unwrap();
    let (hex, pyr, tet) = (count_shape(m, CellShape::Hex), count_shape(m, CellShape::Pyramid), count_shape(m, CellShape::Tet));
    assert_eq!(hex, (n * n * n) as usize, "the hex neighbour keeps all its hexes");
    assert_eq!(pyr, (n * n) as usize, "one pyramid per interface quad");
    assert!(tet > 0);
    // Pyramid volume s^2 h / 3, h = 0.6 s: every pyramid, to 1e-12.
    let s = 1.0 / f64::from(n);
    let want = s * s * 0.6 * s / 3.0;
    for c in 0..m.count(MeshEntityKind::Cell) {
        if m.cell_shape(CellIx(c as u32)) == CellShape::Pyramid {
            let v = cell_volume(m, CellIx(c as u32));
            assert!((v - want).abs() < 1e-12, "pyramid volume {v} vs {want}");
        }
    }
    // Total = the sum of the two boxes; the hex part is exact.
    let total = volume_of(m, CellShape::Hex) + volume_of(m, CellShape::Pyramid) + volume_of(m, CellShape::Tet);
    assert!((total - 2.0).abs() < 1e-12, "total {total}");
    assert!((volume_of(m, CellShape::Hex) - 1.0).abs() < 1e-12);
    // Signed pyramid scaled Jacobian is positive.
    let (pj, tj, hj) = (min_sj(m, CellShape::Pyramid), min_sj(m, CellShape::Tet), min_sj(m, CellShape::Hex));
    assert!(pj > 0.0 && tj > 0.0, "pyramid {pj}, tet {tj}");
    eprintln!("n={n}: hex {hex}, pyramid {pyr}, tet {tet}; min SJ hex {hj:.3}, pyramid {pj:.3}, tet {tj:.3}");
    // The plan advertises PYRAMID on the tet volume.
    let tp = out.plan.target(Target::Volume(b.volume)).unwrap();
    assert!(tp.caps.contains(Caps::PYRAMID));
}

#[test]
fn the_pyramid_count_is_n_squared() {
    for n in [2u32, 3, 5] {
        let (topo, a, b) = setup();
        let it = intent(&a, &b, n, pyramid_params());
        let out = run(&topo, &[a.volume, b.volume], &it, &Sequential).unwrap_or_else(|e| panic!("n={n}: {e}"));
        let m = &out.finished.mesh;
        assert!(validate(m).is_empty(), "n={n}: {:?}", validate(m));
        assert_eq!(count_shape(m, CellShape::Pyramid), (n * n) as usize);
        assert_eq!(count_shape(m, CellShape::Hex), (n * n * n) as usize);
        let total = volume_of(m, CellShape::Hex) + volume_of(m, CellShape::Pyramid) + volume_of(m, CellShape::Tet);
        assert!((total - 2.0).abs() < 1e-12, "n={n}: total {total}");
        eprintln!("n={n}: tets {}, min SJ pyramid {:.3}, tet {:.3}", count_shape(m, CellShape::Tet), min_sj(m, CellShape::Pyramid), min_sj(m, CellShape::Tet));
    }
}

#[test]
fn transition_none_keeps_todays_behaviour_and_conflict() {
    // Explicit "none" and absent params behave alike: a Hard TRI against a hex neighbour's Hard QUAD.
    let (topo, a, b) = setup();
    for params in [serde_json::json!({}), serde_json::json!({"transition": "none"})] {
        let it = intent(&a, &b, 3, params);
        let r = run(&topo, &[a.volume, b.volume], &it, &Sequential);
        assert!(r.is_err(), "a Hard TRI against a Hard QUAD must conflict");
    }
    // A lone tet volume with `pyramid` and no quads is today's mesh (no pyramids, same cells).
    let mut topo = CellComplex::new();
    let ids = with_op(&mut topo, OpId(1), |t| brick(t, sm_core::Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])).unwrap();
    let mk = |params: serde_json::Value| {
        let mut it = MeshIntent::default();
        it.defaults.sizing = Some(SizingSpec::Constant { size: 0.25 });
        it.set_scheme(ids.volume, SchemeChoice::with_params(NAME, params, Firmness::Hard));
        run(&topo, &[ids.volume], &it, &Sequential).unwrap_or_else(|e| panic!("{e}"))
    };
    let (plain, pyr) = (mk(serde_json::json!({})), mk(pyramid_params()));
    assert_eq!(sm_testkit::check::mesh_hash(&plain.finished.mesh), sm_testkit::check::mesh_hash(&pyr.finished.mesh));
    assert_eq!(count_shape(&pyr.finished.mesh, CellShape::Pyramid), 0);
}

#[test]
fn params_hash_is_unchanged_and_validation_is_typed() {
    // Defaults serialise to nothing new: existing plan hashes stay.
    assert_eq!(serde_json::to_string(&TetParams::default()).unwrap(), "{}");
    let p = TetParams { transition: Some(Transition::Pyramid), ..TetParams::default() };
    assert_eq!(serde_json::to_string(&p).unwrap(), r#"{"transition":"pyramid"}"#);
    let parse = |v: serde_json::Value| TetParams::parse(&sm_scheme_api::Params(v));
    assert!(parse(serde_json::json!({"transition": "pyramid", "pyramid_height": 0.5})).is_ok());
    assert!(parse(serde_json::json!({"pyramid_height": 0.5})).unwrap_err().contains("needs transition"));
    assert!(parse(serde_json::json!({"transition": "pyramid", "pyramid_height": 9.0})).unwrap_err().contains("pyramid_height"));
    assert!(parse(serde_json::json!({"transition": "wedge"})).unwrap_err().contains("tetmesh params"));
}

#[test]
fn deterministic_across_runs_and_threads() {
    let (topo, a, b) = setup();
    let it = intent(&a, &b, 3, pyramid_params());
    let h = |out: &Out| (sm_testkit::check::mesh_hash(&out.finished.mesh), format!("{:?}", out.finished.mesh.coords()));
    let one = run(&topo, &[a.volume, b.volume], &it, &Sequential).unwrap_or_else(|e| panic!("{e}"));
    let two = run(&topo, &[a.volume, b.volume], &it, &Sequential).unwrap();
    let thr = run(&topo, &[a.volume, b.volume], &it, &ScopedThreads(4)).unwrap();
    assert_eq!(h(&one), h(&two));
    assert_eq!(h(&one), h(&thr));
}

#[test]
fn a_thin_slab_fails_typed_and_may_fall_back_not_internal() {
    // Tet slab [1, 1.05] x [0,1]^2 next to the hex brick: the apex (h = 0.6 s) goes through the far face.
    let mut topo = CellComplex::new();
    let (a, b) = with_op(&mut topo, OpId(1), |t| {
        let a = brick(t, sm_core::Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])?;
        let b = brick(t, sm_core::Point3::new(1.0, 0.0, 0.0), [0.05, 1.0, 1.0])?;
        t.record_coincidence(a.faces[1], b.faces[0], sm_topo_api::Coincidence::CoincidentUnimprinted)?;
        merge_touching_bricks(t, &a, &b)?;
        Ok((a, b))
    })
    .unwrap();
    let mut it = intent(&a, &b, 4, pyramid_params());
    it.set_intervals(b.edges[0], Firm::hard(IntervalRequest::Count(1)));
    match run(&topo, &[a.volume, b.volume], &it, &Sequential) {
        Ok(_) => {} // a fallback or a shallower apex is fine
        Err(e) => {
            let text = format!("{e}");
            assert!(text.contains("(Geometry)") && text.contains("pyramid apex height"), "{text}");
            assert!(!text.contains("(Internal)"), "pyramid failure must not be Internal: {text}");
        }
    }
}
