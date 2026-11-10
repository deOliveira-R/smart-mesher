//! W4-HEX-STRAT: the `hex_only` and `tet` strategies and `mesh.info.volume_by_owner`, on the stub
//! schemes. (The real-geometry pin checks run through `standard()` in the server tests.)

mod common;

use common::*;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

use serde_json::{Value, json};
use sm_mesher::{AutoInput, AutoLists, AutoStrategy, MesherConfig, SchemeAssignment};
use sm_op_api::{DerivedCache, QuietControl, Session};
use sm_scheme_api::stubs;
use sm_scheme_api::{
    Applicability, Caps, GenCtx, GenerateFailure, IntervalCtx, IntervalSink, Params, Patch,
    PlanCtx, Requirements, Scheme, SchemeDim, SchemeError, SchemeRegistry, Target,
};

/// A stub scheme registered under another name (so the strategy lists find it).
struct Alias<S: Scheme> {
    name: &'static str,
    inner: S,
}

impl<S: Scheme> Scheme for Alias<S> {
    fn name(&self) -> &str {
        self.name
    }
    fn dim(&self) -> SchemeDim {
        self.inner.dim()
    }
    fn caps(&self, p: &Params) -> Caps {
        self.inner.caps(p)
    }
    fn applicability(&self, cx: &PlanCtx<'_>, t: Target, p: &Params) -> Applicability {
        self.inner.applicability(cx, t, p)
    }
    fn requirements(
        &self,
        cx: &PlanCtx<'_>,
        t: Target,
        p: &Params,
        out: &mut Requirements<'_>,
    ) -> Result<(), SchemeError> {
        self.inner.requirements(cx, t, p, out)
    }
    fn interval_constraints(
        &self,
        cx: &IntervalCtx<'_>,
        t: Target,
        p: &Params,
        out: &mut IntervalSink<'_>,
    ) -> Result<(), SchemeError> {
        self.inner.interval_constraints(cx, t, p, out)
    }
    fn generate(&self, cx: &GenCtx<'_>, t: Target, p: &Params) -> Result<Patch, GenerateFailure> {
        self.inner.generate(cx, t, p)
    }
}

/// A volume scheme that applies to nothing (stands for a volume no hex scheme can mesh).
struct Never(&'static str);

impl Scheme for Never {
    fn name(&self) -> &str {
        self.0
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::HEX
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::no("not meshable with hexes")
    }
    fn generate(&self, _cx: &GenCtx<'_>, t: Target, _p: &Params) -> Result<Patch, GenerateFailure> {
        Err(GenerateFailure::fallback(t, self.0, "never applies"))
    }
}

/// Stub schemes plus `trimesh` (a `tri` alias) and `tetmesh` (a lowest-score `sweep` alias, so
/// the stubs can mesh it); the real names the standard lists use.
fn registry(with_hex_volumes: bool) -> SchemeRegistry {
    let mut r = SchemeRegistry::new();
    let mut add = |s: Box<dyn Scheme>| r.register(s).expect("register");
    add(Box::new(stubs::Equal));
    add(Box::new(stubs::Map));
    add(Box::new(stubs::Submap));
    add(Box::new(stubs::Pave));
    add(Box::new(stubs::Tri));
    add(Box::new(Alias {
        name: "trimesh",
        inner: stubs::Tri,
    }));
    add(Box::new(Alias {
        name: "tetmesh",
        inner: stubs::Sweep,
    }));
    if !with_hex_volumes {
        // Registered under the list names, but they apply to nothing.
        add(Box::new(Never("map")));
        add(Box::new(Never("sweep")));
    } else {
        add(Box::new(stubs::Sweep));
        add(Box::new(stubs::MapVol));
        // The standard `map` volume scheme is `mapvol` in the stubs; register it as `map`.
        add(Box::new(Alias {
            name: "map",
            inner: stubs::MapVol,
        }));
    }
    r
}

fn session(with_hex_volumes: bool) -> Session {
    // The registry's own default strategies, plus nothing else.
    Session::new(engine_cfg(
        MesherConfig::new(registry(with_hex_volumes)),
        DerivedCache::default(),
    ))
}

fn bricks(s: &mut Session, sizes: &[[f64; 3]]) {
    for (i, sz) in sizes.iter().enumerate() {
        let op = sm_testkit_brick(3.0 * i as f64, *sz);
        run(s, &op).expect("brick");
    }
    let targets: Vec<String> = (1..=sizes.len())
        .map(|n| format!("TopoVolume#{n}"))
        .collect();
    run_json(s, "mesh.size", json!({"targets": targets, "size": 1.0})).expect("size");
}

fn sm_testkit_brick(x: f64, size: [f64; 3]) -> CreateBrick {
    CreateBrick {
        origin: [x, 0.0, 0.0],
        size,
    }
}

fn assignments(s: &Session) -> Vec<(String, String)> {
    let h = s.position().expect("head");
    let rec = s.timeline().get(h).expect("node").body.op.clone();
    rec.params["assignments"]
        .as_array()
        .expect("assignments")
        .iter()
        .map(|a| {
            (
                a["target"].as_str().expect("target").to_owned(),
                a["scheme"].as_str().expect("scheme").to_owned(),
            )
        })
        .collect()
}

fn auto(s: &mut Session, strategy: &str) -> Result<Vec<(String, String)>, String> {
    match run_json(s, "mesh.auto", json!({"strategy": strategy})) {
        Ok((_, d)) => {
            let tet = d
                .diagnostics
                .iter()
                .filter(|x| x.code == "mesh.auto_tet")
                .count();
            let mut a = assignments(s);
            a.push(("mesh.auto_tet".into(), tet.to_string()));
            Ok(a)
        }
        Err(e) => Err(e.to_string()),
    }
}

fn query(s: &Session, name: &str, input: Value) -> Value {
    let q = QuietControl::new();
    s.engine()
        .run_query(s.state(), name, &input, &q.control())
        .expect("query")
}

// 1: the registry lists the three names, ascending, with descriptions.
#[test]
fn auto_strategies_lists_default_hex_only_tet_ascending() {
    let s = session(true);
    let r = query(&s, "mesh.auto_strategies", json!({}));
    let rows = r["strategies"].as_array().expect("rows");
    let names: Vec<&str> = rows
        .iter()
        .map(|x| x["name"].as_str().expect("n"))
        .collect();
    assert_eq!(names, ["default", "hex_only", "tet"]);
    for x in rows {
        assert!(!x["description"].as_str().expect("d").is_empty());
    }
}

// 2: on the stub brick, `hex_only` assigns what `default` assigns.
#[test]
fn hex_only_equals_default_on_a_brick() {
    let mut a = session(true);
    bricks(&mut a, &[[2.0; 3]]);
    let da = auto(&mut a, "default").expect("default");
    let mut b = session(true);
    bricks(&mut b, &[[2.0; 3]]);
    let hb = auto(&mut b, "hex_only").expect("hex_only");
    assert_eq!(da, hb);
    assert!(
        da.iter()
            .all(|(t, s)| t == "mesh.auto_tet" || s != "tetmesh"),
        "{da:?}"
    );
}

// 3: with no hex volume scheme, `hex_only` fails naming the volume and the history is unchanged;
// `default` falls back to tets (with the warning) and does not.
#[test]
fn hex_only_fails_where_default_falls_back_to_tets() {
    let mut s = session(false);
    bricks(&mut s, &[[2.0; 3]]);
    let h = s.state().hash();
    let e = auto(&mut s, "hex_only").expect_err("no hex volume scheme");
    assert!(e.contains("TopoVolume#1"), "{e}");
    assert!(!e.contains("tetmesh"), "tetmesh was tried: {e}");
    assert!(
        e.contains("not meshable with hexes"),
        "no per-candidate reason: {e}"
    );
    assert_eq!(s.state().hash(), h, "a rejected op changes nothing");

    let d = auto(&mut s, "default").expect("default");
    assert!(
        d.contains(&("TopoVolume#1".into(), "tetmesh".into())),
        "{d:?}"
    );
    assert!(d.contains(&("mesh.auto_tet".into(), "1".into())), "{d:?}");
}

// 4: `tet` assigns tetmesh to every volume, trimesh to every face, and never warns.
#[test]
fn tet_assigns_tetmesh_and_trimesh_without_a_warning() {
    let mut s = session(true);
    bricks(&mut s, &[[2.0; 3]]);
    let a = auto(&mut s, "tet").expect("tet");
    assert!(
        a.contains(&("TopoVolume#1".into(), "tetmesh".into())),
        "{a:?}"
    );
    assert!(a.contains(&("mesh.auto_tet".into(), "0".into())), "{a:?}");
    let faces: Vec<_> = a
        .iter()
        .filter(|(t, _)| t.starts_with("TopoFace"))
        .collect();
    assert!(!faces.is_empty(), "no face assignments: {a:?}");
    assert!(faces.iter().all(|(_, sch)| sch == "trimesh"), "{faces:?}");
}

/// The real `curve`, `tri` and `tet` schemes (the `tet` strategy's names, no stubs).
fn real_tet_registry() -> SchemeRegistry {
    let mut r = SchemeRegistry::new();
    sm_scheme_curve::register(&mut r).expect("curve");
    sm_scheme_tri::register(&mut r).expect("tri");
    sm_scheme_tet::register(&mut r).expect("tet");
    r
}

// 4 (real geometry): `tet` on a brick meshes it all-tet, with no warning, and the mesh volume is
// the brick's. Faces follow tetmesh's boundary scheme (`tridelaunay`), not the surface list.
#[test]
fn tet_meshes_a_real_brick_with_all_tets_and_the_exact_volume() {
    let mut s = Session::new(engine_cfg(
        MesherConfig::new(real_tet_registry()),
        DerivedCache::default(),
    ));
    bricks(&mut s, &[[2.0, 3.0, 4.0]]);
    let a = auto(&mut s, "tet").expect("tet");
    assert!(
        a.contains(&("TopoVolume#1".into(), "tetmesh".into())),
        "{a:?}"
    );
    assert!(a.contains(&("mesh.auto_tet".into(), "0".into())), "{a:?}");
    let (_, d) = mesh_run(&mut s).expect("run");
    let mid = d.outputs["mesh"][0];
    let info = query(&s, "mesh.info", json!({"mesh": mid.to_string()}));
    let shapes = info["cell_shapes"].as_object().expect("cell_shapes");
    assert!(
        !shapes.is_empty() && shapes.keys().all(|k| k == "Tet"),
        "non-tet cells: {shapes:?}"
    );
    let v = info["volume"].as_f64().expect("volume");
    assert!((v - 24.0).abs() < 1e-12 * 24.0, "{v}");
}

#[test]
fn strategy_lists_are_as_documented() {
    let h = AutoLists::hex_only();
    assert_eq!(h.surface, ["map", "submap", "pave"]);
    assert_eq!(h.volume, ["map", "sweep"]);
    let t = AutoLists::tet();
    assert_eq!(
        (t.surface, t.volume),
        (vec!["trimesh".to_owned()], vec!["tetmesh".to_owned()])
    );
}

// 5: replay of a recorded `hex_only` history applies the stored assignments and reproduces every
// hash; the strategy is named in the record but never looked up. The replay engine swaps in a
// strategy of the same name that counts its calls (as `a8` in `auto.rs`).
struct Counting(Arc<AtomicUsize>);

impl AutoStrategy for Counting {
    fn name(&self) -> &'static str {
        "hex_only"
    }
    fn choose(&self, _input: &AutoInput<'_>) -> Result<Vec<SchemeAssignment>, SchemeError> {
        self.0.fetch_add(1, Ordering::SeqCst);
        Err(SchemeError::BadParams {
            scheme: sm_scheme_api::SchemeName::new("hex_only"),
            reason: "replay must not call a strategy".into(),
        })
    }
}

#[test]
fn replay_reproduces_the_recorded_hashes() {
    let mut s = session(true);
    bricks(&mut s, &[[2.0; 3]]);
    auto(&mut s, "hex_only").expect("auto");
    mesh_run(&mut s).expect("run");
    let head = s.position().expect("head");
    let recs: Vec<_> = s
        .timeline()
        .path_to_root(head)
        .expect("path")
        .into_iter()
        .map(|h| {
            let n = s.timeline().get(h).expect("node");
            (n.body.op.clone(), n.check.state)
        })
        .collect();
    assert!(recs.iter().any(|(r, _)| r.params["strategy"] == "hex_only"));

    let calls = Arc::new(AtomicUsize::new(0));
    let cfg = MesherConfig::new(registry(true)).with_strategy(Arc::new(Counting(calls.clone())));
    let eng = engine_cfg(cfg, DerivedCache::disabled());
    let q = QuietControl::new();
    let mut st = eng.registry().new_state();
    for (rec, want) in &recs {
        eng.apply_record(&mut st, rec, &q.control())
            .expect("replay");
        assert_eq!(st.hash(), *want);
    }
    assert_eq!(
        calls.load(Ordering::SeqCst),
        0,
        "replay called the strategy"
    );
}

// 6: volume_by_owner sums to the total and each brick's own volume.
#[test]
fn volume_by_owner_sums_to_the_total_and_matches_each_volume() {
    let mut s = session(true);
    bricks(&mut s, &[[2.0, 3.0, 4.0], [1.0, 2.0, 5.0]]);
    auto(&mut s, "hex_only").expect("auto");
    let (_, d) = run_json(
        &mut s,
        "mesh.run",
        json!({"targets": ["TopoVolume#1", "TopoVolume#2"]}),
    )
    .expect("run");
    let mid = d.outputs["mesh"][0];
    let info = query(&s, "mesh.info", json!({"mesh": mid.to_string()}));
    let by = info["volume_by_owner"].as_object().expect("map");
    let keys: Vec<&String> = by.keys().collect();
    assert_eq!(keys, ["TopoVolume#1", "TopoVolume#2"]);
    let v1 = by["TopoVolume#1"].as_f64().expect("f64");
    let v2 = by["TopoVolume#2"].as_f64().expect("f64");
    assert!((v1 - 24.0).abs() < 1e-12 * 24.0, "{v1}");
    assert!((v2 - 10.0).abs() < 1e-12 * 10.0, "{v2}");
    let total = info["volume"].as_f64().expect("f64");
    assert!((v1 + v2 - total).abs() < 1e-12 * total);
}

// 6 (determinism): equal for 1 and N threads, bit for bit.
fn info_with(threads: usize) -> String {
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(threads)
        .build()
        .expect("pool");
    pool.install(|| {
        let mut s = session(true);
        bricks(&mut s, &[[2.0, 3.0, 4.0], [1.0, 2.0, 5.0]]);
        auto(&mut s, "hex_only").expect("auto");
        let (_, d) = run_json(
            &mut s,
            "mesh.run",
            json!({"targets": ["TopoVolume#1", "TopoVolume#2"]}),
        )
        .expect("run");
        let mid = d.outputs["mesh"][0];
        query(&s, "mesh.info", json!({"mesh": mid.to_string()}))["volume_by_owner"].to_string()
    })
}

#[test]
fn volume_by_owner_is_bit_identical_across_threads_and_runs() {
    let a = info_with(1);
    assert_eq!(a, info_with(1));
    assert_eq!(a, info_with(4));
}
