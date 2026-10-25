//! Registry-level conventions (W2 plan §5 risk 6): every op of a distribution declares `OUTPUTS`
//! (or sets `NO_OUTPUTS` because it writes no ids), and every `EntityId` property of an op or query
//! carries `x-entity-kind`. The checker runs on the demo set, on the standard plugin set over the
//! stub schemes (always), and on `standard()` itself once the scheme crates are merged.
mod common;

use common::*;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sm_core::{ENTITY_ID_PATTERN, EntityId, EntityKind};
use sm_engine::WorkspaceBuilder;
use sm_op_api::{ApplyCtx, OpError, OpInfoOwned, QueryInfoOwned, TypedOp};
use sm_protocol::{CatalogResult, Endpoint, Request};
use sm_server::{Server, StandardError, distribution, try_standard};

fn is_entity_id(v: &Value) -> bool {
    let Some(o) = v.as_object() else { return false };
    if o.get("pattern").and_then(Value::as_str) == Some(ENTITY_ID_PATTERN) {
        return true;
    }
    if o.get("$ref").and_then(Value::as_str).is_some_and(|r| r.ends_with("/EntityId")) {
        return true;
    }
    ["anyOf", "oneOf", "allOf"].iter().any(|k| o.get(*k).and_then(Value::as_array).is_some_and(|a| a.iter().any(is_entity_id)))
}

/// True for an id, an array of ids, a map to ids, or an optional of these.
fn holds_ids(v: &Value) -> bool {
    if is_entity_id(v) {
        return true;
    }
    let Some(o) = v.as_object() else { return false };
    o.get("items").is_some_and(holds_ids)
        || o.get("additionalProperties").is_some_and(holds_ids)
        || ["anyOf", "oneOf", "allOf"].iter().any(|k| o.get(*k).and_then(Value::as_array).is_some_and(|a| a.iter().any(holds_ids)))
}

fn kind_annotation_ok(v: &Value) -> bool {
    v.get("x-entity-kind").and_then(Value::as_array).is_some_and(|a| {
        !a.is_empty() && a.iter().all(|k| k.as_str().is_some_and(|k| k.parse::<EntityKind>().is_ok()))
    })
}

fn walk_properties(node: &Value, path: &str, out: &mut Vec<String>) {
    let Some(o) = node.as_object() else { return };
    if let Some(props) = o.get("properties").and_then(Value::as_object) {
        for (k, p) in props {
            let here = format!("{path}.{k}");
            if holds_ids(p) && !kind_annotation_ok(p) {
                out.push(here.clone());
            }
            walk_properties(p, &here, out);
        }
    }
    for key in ["items", "additionalProperties"] {
        if let Some(i) = o.get(key) {
            walk_properties(i, path, out);
        }
    }
    for key in ["anyOf", "oneOf", "allOf"] {
        if let Some(a) = o.get(key).and_then(Value::as_array) {
            a.iter().for_each(|b| walk_properties(b, path, out));
        }
    }
    if let Some(defs) = o.get("$defs").and_then(Value::as_object) {
        for (k, d) in defs {
            walk_properties(d, &format!("{path}#{k}"), out);
        }
    }
}

/// Every violation of the conventions in a catalog, as readable lines.
fn violations(ops: &[OpInfoOwned], queries: &[QueryInfoOwned]) -> Vec<String> {
    let mut v = Vec::new();
    for op in ops {
        if op.outputs.is_empty() && !op.no_outputs {
            v.push(format!("op `{}` declares no OUTPUTS (set `NO_OUTPUTS = true` only if it writes no ids)", op.name));
        }
        let mut bad = Vec::new();
        walk_properties(op.schema.as_value(), "", &mut bad);
        v.extend(bad.into_iter().map(|p| format!("op `{}`: id property `{p}` lacks a valid x-entity-kind", op.name)));
    }
    for q in queries {
        let mut bad = Vec::new();
        walk_properties(q.schema.as_value(), "", &mut bad);
        v.extend(bad.into_iter().map(|p| format!("query `{}`: id property `{p}` lacks a valid x-entity-kind", q.name)));
    }
    v
}

fn catalog_of(builder: WorkspaceBuilder) -> CatalogResult {
    let mut s = Server::new(builder).expect("server");
    serde_json::from_value(s.call(Request::Catalog).expect("catalog")).expect("CatalogResult")
}

// ------------------------------------------------------------------ the checker itself

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Inner {
    #[schemars(extend("x-entity-kind" = ["TopoFace"]))]
    face: EntityId,
    loose: EntityId,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Probe {
    #[schemars(extend("x-entity-kind" = ["TopoVolume"]))]
    one: EntityId,
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoFace"]))]
    many: Vec<EntityId>,
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    maybe: Option<EntityId>,
    forgotten: EntityId,
    forgotten_list: Vec<EntityId>,
    #[schemars(extend("x-entity-kind" = ["Nonsense"]))]
    bad_kind: EntityId,
    #[schemars(extend("x-entity-kind" = []))]
    empty_kind: EntityId,
    nested: Inner,
    not_an_id: String,
}

impl TypedOp for Probe {
    const NAME: &'static str = "probe.run";
    fn apply(&self, _: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        Ok(())
    }
}

struct ProbePlugin;

impl sm_op_api::Plugin for ProbePlugin {
    fn id(&self) -> &str {
        "probe"
    }
    fn register(&self, r: &mut sm_op_api::Registry) -> Result<(), sm_op_api::RegistryError> {
        r.register::<Probe>().map(|_| ())
    }
}

#[test]
fn the_checker_flags_exactly_the_unannotated_id_properties() {
    let cat = catalog_of(WorkspaceBuilder::new().plugin(ProbePlugin));
    let probe: Vec<_> = cat.ops.iter().filter(|o| o.name == "probe.run").cloned().collect();
    let found = violations(&probe, &[]);
    let props: Vec<&str> = found.iter().filter(|l| l.contains("lacks")).map(String::as_str).collect();
    for (name, flagged) in [
        ("one", false),
        ("many", false),
        ("maybe", false),
        ("forgotten", true),
        ("forgotten_list", true),
        ("bad_kind", true),
        ("empty_kind", true),
        ("not_an_id", false),
    ] {
        let hit = props.iter().any(|l| l.contains(&format!("`.{name}`")));
        assert_eq!(hit, flagged, "property {name}: {found:?}");
    }
    assert!(props.iter().any(|l| l.contains("loose")), "nested struct properties are checked: {found:?}");
    assert!(!props.iter().any(|l| l.contains("`.face`") || l.contains(".face`")), "annotated nested id passes: {found:?}");
    assert!(found.iter().any(|l| l.contains("declares no OUTPUTS")), "{found:?}");
}

#[test]
fn the_demo_set_follows_the_conventions() {
    let cat = catalog_of(demo_builder());
    let v = violations(&cat.ops, &cat.queries);
    assert!(v.is_empty(), "{v:#?}");
    assert!(cat.ops.iter().any(|o| o.name == "demo.add_item"), "the demo ops are in the catalog");
}

// ------------------------------------------------------------------ the standard distribution

#[test]
fn the_standard_plugin_set_over_stub_schemes_follows_the_conventions() {
    let cat = catalog_of(distribution(sm_scheme_api::stubs::registry()));
    let v = violations(&cat.ops, &cat.queries);
    assert!(v.is_empty(), "{v:#?}");
    assert!(cat.ops.iter().any(|o| o.name == "core.batch"), "the skeleton op is listed");
}

#[test]
fn standard_follows_the_conventions_once_the_scheme_crates_are_merged() {
    match try_standard() {
        Ok(b) => {
            let cat = catalog_of(b);
            let v = violations(&cat.ops, &cat.queries);
            assert!(v.is_empty(), "{v:#?}");
        }
        // Skeleton scheme crates register nothing yet; the check goes live with the merge.
        Err(StandardError::MissingScheme { dim, scheme }) => {
            eprintln!("skipped: scheme `{scheme}` ({dim}) is not registered yet (skeleton scheme crates)");
        }
        Err(e) => panic!("standard distribution failed to assemble: {e}"),
    }
}

#[test]
fn standard_panics_exactly_when_try_standard_fails() {
    match try_standard() {
        Ok(_) => drop(sm_server::standard()),
        Err(_) => assert!(std::panic::catch_unwind(sm_server::standard).is_err()),
    }
}

#[test]
fn standard_lists_the_milestone_ops() {
    let cat = catalog_of(try_standard().expect("standard"));
    let names: Vec<&str> = cat.ops.iter().map(|o| o.name.as_str()).collect();
    for want in ["core.batch", "model.create_brick", "mesh.size", "mesh.scheme", "mesh.run", "fem.block_add", "fem.block_element_type", "mesh.intervals", "mesh.default_size"] {
        assert!(names.contains(&want), "missing op {want}: {names:?}");
    }
    let qn: Vec<&str> = cat.queries.iter().map(|q| q.name.as_str()).collect();
    for want in ["topo.related", "mesh.info"] {
        assert!(qn.contains(&want), "missing query {want}: {qn:?}");
    }
}
