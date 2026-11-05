//! W3b-RT-BRIDGE acceptance 5: the read-only query `rt.check_overlaps` over a session model.
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sm_layer_api::{CoreLayers, LayerResultExt, commit_topology, model_txn, topo_txn};
use sm_model_api::Domain;
use sm_op_api::{ApplyCtx, DerivedCache, Engine, Meta, OpError, QuietControl, Registry, Services, Session, TypedOp};
use sm_rt::RtPlugin;
use sm_topo_api::fixtures::cylinder;
use std::sync::Arc;

/// Two coaxial cylinders (radius 1 and 2, height 3) as two volumes `inner` and `outer`: they overlap.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct TwoCylinders {
    /// Make the second one wider than the first without touching it (radius 1 and 1.5 at once is
    /// still an overlap; `false` keeps the standard pair).
    #[serde(default)]
    solo: bool,
}

impl TypedOp for TwoCylinders {
    const NAME: &'static str = "test.two_cylinders";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let ((a, b), txn) = topo_txn(ctx, |topo, _| {
            let a = cylinder(topo, 1.0, 3.0).or_op()?;
            let b = if self.solo { None } else { Some(cylinder(topo, 2.0, 3.0).or_op()?) };
            Ok((a, b))
        })?;
        let vols: Vec<_> = std::iter::once(a.volume).chain(b.as_ref().map(|b| b.volume)).collect();
        commit_topology(ctx, &txn, &[("volume", &vols)])?;
        model_txn(
            ctx,
            |model, _| {
                let inner = model.add_color(1, "inner", Domain::Volume).map_err(|e| OpError::Failed(e.to_string()))?;
                model.color_volumes(inner, &[a.volume]).map_err(|e| OpError::Failed(e.to_string()))?;
                if let Some(b) = &b {
                    let outer = model.add_color(2, "outer", Domain::Volume).map_err(|e| OpError::Failed(e.to_string()))?;
                    model.color_volumes(outer, &[b.volume]).map_err(|e| OpError::Failed(e.to_string()))?;
                }
                Ok(())
            },
            |_, _| Vec::new(),
        )
    }
}

fn session() -> (Session, Arc<DerivedCache>) {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::default()).unwrap();
    reg.install(&RtPlugin).unwrap();
    reg.register::<TwoCylinders>().unwrap();
    let cache = Arc::new(DerivedCache::default());
    (Session::new(Engine::new(Arc::new(reg), Services::new(), Arc::clone(&cache)).unwrap()), cache)
}

fn model(solo: bool) -> (Session, Arc<DerivedCache>) {
    let (mut s, c) = session();
    let q = QuietControl::new();
    s.apply(&TwoCylinders { solo }, Meta::new("test", "", ""), &q.control()).expect("model");
    (s, c)
}

fn check(s: &Session, args: Value) -> Result<Value, OpError> {
    let q = QuietControl::new();
    s.engine().run_query(s.state(), "rt.check_overlaps", &args, &q.control()).map_err(|e| OpError::Failed(e.to_string()))
}

#[test]
fn rt_check_overlaps_is_in_the_catalog_with_its_schema() {
    let (s, _) = session();
    let reg = s.engine().registry();
    assert!(reg.query_names().contains(&"rt.check_overlaps"));
    let q = reg.query_catalog().into_iter().map(|q| q.expect("schema")).find(|q| q.name == "rt.check_overlaps").expect("listed");
    let text = serde_json::to_string(&q.schema).unwrap();
    for field in ["bodies", "samples", "rays", "seed", "x-entity-kind"] {
        assert!(text.contains(field), "input schema names {field}");
    }
    assert!(serde_json::to_string(&q.output_schema).unwrap().contains("findings"));
}

#[test]
fn two_overlapping_cylinders_report_csg_overlap_with_a_point_and_both_cells() {
    let (s, _) = model(false);
    let out = check(&s, json!({ "samples": 4000, "rays": 100 })).expect("rt.check_overlaps");
    assert_eq!(out["ok"], false);
    assert_eq!(out["seed"], 0);
    assert!(out["overlaps"].as_u64().unwrap() >= 1);
    let f = out["findings"].as_array().unwrap().iter().find(|f| f["code"] == "csg.overlap").expect("an overlap finding");
    let p = f["location"].as_array().unwrap();
    let (x, y, z) = (p[0].as_f64().unwrap(), p[1].as_f64().unwrap(), p[2].as_f64().unwrap());
    assert!(x.hypot(y) < 1.0 && (0.0..=3.0).contains(&z), "inside the smaller cylinder: {p:?}");
    let mut names: Vec<&str> = f["cells"].as_array().unwrap().iter().map(|c| c["name"].as_str().unwrap()).collect();
    names.sort_unstable();
    assert_eq!(names, ["inner", "outer"], "both cells, by color name");
    assert!(f["cells"][0]["volume"].is_string());
    assert!(f["message"].as_str().unwrap().contains("seed 0"));
}

#[test]
fn a_model_without_overlap_is_ok_and_bodies_restrict_the_check() {
    let (s, _) = model(true);
    let out = check(&s, json!({ "samples": 2000, "rays": 50 })).unwrap();
    assert_eq!(out["ok"], true);
    assert_eq!(out["overlaps"], 0);
    // `bodies` accepts a volume id: only the outer cylinder of the overlapping model is left, so no overlap.
    let (s, _) = model(false);
    let vols: Vec<String> = s.state().layer::<sm_layer_api::ModelLayer>().unwrap().topo().cells(sm_topo_api::Dim::Volume).iter().map(ToString::to_string).collect();
    let out = check(&s, json!({ "bodies": [vols[1]], "samples": 2000, "rays": 50 })).unwrap();
    assert_eq!(out["ok"], true);
    // Typed errors.
    assert!(check(&s, json!({ "bodies": [] })).is_err());
    assert!(check(&s, json!({ "bodies": ["TopoVolume#999"] })).is_err());
    assert!(check(&s, json!({ "samples": 20_000_000u32 })).is_err());
    assert!(check(&s, json!({ "bogus": 1 })).is_err());
}

#[test]
fn seed_defaults_to_zero_in_the_recorded_input_and_findings_are_reproducible() {
    let (s, cache) = model(false);
    let a = check(&s, json!({ "samples": 3000, "rays": 60 })).unwrap();
    let before = cache.stats();
    // A call with `seed: 0` is the same canonical input: same output, and it hits the cache entry
    // the first call stored (no second computation).
    let b = check(&s, json!({ "samples": 3000, "rays": 60, "seed": 0 })).unwrap();
    assert_eq!(a, b);
    assert_eq!(cache.stats().hits, before.hits + 1);
    assert_eq!(cache.stats().misses, before.misses);
    // Another seed is another input with its own echo, and (for these samples) another sample set.
    let c = check(&s, json!({ "samples": 3000, "rays": 60, "seed": 5 })).unwrap();
    assert_eq!(c["seed"], 5);
    assert_ne!(a["findings"], c["findings"]);
    // A fresh session reproduces the findings exactly.
    let (s2, _) = model(false);
    assert_eq!(a, check(&s2, json!({ "samples": 3000, "rays": 60 })).unwrap());
    // 1 thread and N threads give the same bytes.
    let h = sm_testkit::determinism::check(1, 4, |th| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(th).build().unwrap();
        pool.install(|| {
            let (s, _) = model(false);
            check(&s, json!({ "samples": 3000, "rays": 60, "seed": 7 })).unwrap().to_string()
        })
    })
    .unwrap();
    assert!(h.contains("csg.overlap"));
}
