//! `model.import_step {attributes: true}` (W3b-CAD-ATTR): STEP names become groups, solid-attached
//! colors become volume colors, face-only colors are ignored with a note. Runs on the real
//! `standard()` distribution.

use std::collections::BTreeMap;

use base64::Engine as _;
use base64::engine::general_purpose::STANDARD;
use serde_json::{Value, json};
use sm_core::EntityId;
use sm_layer_api::ModelLayer;
use sm_model_api::Domain;
use sm_op_api::Diagnostic;
use sm_engine::{DiagnosticPolicy, Verbosity};
use sm_server::{ApplyOptions, Workspace, standard};

fn data(name: &str) -> Vec<u8> {
    std::fs::read(format!("{}/tests/data/{name}", env!("CARGO_MANIFEST_DIR"))).expect("fixture")
}

fn ws() -> Workspace {
    standard().policy(DiagnosticPolicy { verbosity: Verbosity::Info, ..DiagnosticPolicy::default() }).memory().expect("workspace")
}

fn import(ws: &mut Workspace, bytes: &[u8], attributes: Option<bool>) -> (BTreeMap<String, Vec<EntityId>>, Vec<Diagnostic>) {
    let mut p = json!({"source": {"inline": STANDARD.encode(bytes)}});
    if let Some(a) = attributes {
        p["attributes"] = Value::Bool(a);
    }
    let d = ws.apply_json("model.import_step", p, &ApplyOptions::default()).unwrap_or_else(|e| panic!("import: {e}")).delta;
    (d.outputs.into_iter().collect(), d.diagnostics)
}

fn model(ws: &Workspace) -> &sm_model_api::Model {
    ws.state().layer::<ModelLayer>().expect("model").model()
}

/// `(group name, member volumes)` ascending by name.
fn groups(ws: &Workspace) -> Vec<(String, Vec<EntityId>)> {
    let mut v: Vec<_> = model(ws).groups().map(|g| (g.name.clone(), g.members.iter().copied().collect::<Vec<_>>())).collect();
    v.sort();
    v
}

fn color_names(ws: &Workspace, vols: &[EntityId]) -> Vec<Option<String>> {
    let m = model(ws);
    vols.iter().map(|v| m.volume_color(*v).and_then(|c| m.color(c)).map(|c| c.name.clone())).collect()
}

fn has(d: &[Diagnostic], code: &str) -> bool {
    d.iter().any(|x| x.code == code)
}

#[test]
fn groups_and_colors_equal_the_file() {
    let mut w = ws();
    let (out, diags) = import(&mut w, &data("named_colored.stp"), Some(true));
    let vols = &out["volumes"];
    assert_eq!(vols.len(), 4);
    // Names: fuel, clad, water from the solids; the fourth solid has no name, so its product name.
    let g = groups(&w);
    let names: Vec<&str> = g.iter().map(|(n, _)| n.as_str()).collect();
    assert_eq!(names, ["clad", "fuel", "part", "water"]);
    for (name, members) in &g {
        let k = match name.as_str() {
            "fuel" => 0,
            "clad" => 1,
            "water" => 2,
            _ => 3,
        };
        assert_eq!(members, &[vols[k]], "{name}");
    }
    // Colors: uo2 on solids 0 and 3, an unnamed blue by its RGB, none on the face-colored solid.
    assert_eq!(color_names(&w, vols), [Some("uo2".into()), Some("rgb_0000ff".into()), None, Some("uo2".into())]);
    let m = model(&w);
    assert_eq!(m.colors().count(), 2);
    assert!(m.colors().all(|c| c.domain == Domain::Volume));
    // The report.
    assert!(has(&diags, "step.attr_groups") && has(&diags, "step.attr_colors"), "{diags:?}");
    let ign = diags.iter().find(|d| d.code == "step.attr_ignored").expect("face-only note");
    assert!(ign.message.contains("1 face-only"), "{}", ign.message);
    assert!(!has(&diags, "step.attr_skipped"));
}

#[test]
fn face_only_colors_do_not_color_volumes() {
    let mut w = ws();
    let (out, _) = import(&mut w, &data("named_colored.stp"), Some(true));
    let water = out["volumes"][2];
    assert_eq!(model(&w).volume_color(water), None);
    assert!(model(&w).colors().all(|c| c.name != "face_green"));
    assert_eq!(model(&w).face_coloring().count(), 0);
}

#[test]
fn imports_without_the_option_are_unchanged() {
    let bytes = data("named_colored.stp");
    let (mut a, mut b, mut c) = (ws(), ws(), ws());
    let (_, da) = import(&mut a, &bytes, None);
    import(&mut b, &bytes, Some(false));
    import(&mut c, &bytes, Some(true));
    assert_eq!(a.state_hash(), b.state_hash());
    assert_ne!(a.state_hash(), c.state_hash());
    assert!(groups(&a).is_empty() && model(&a).colors().count() == 0);
    assert!(!has(&da, "step.attr_groups"));
    // The recorded op is the same JSON as before the option existed.
    let op = serde_json::to_value(sm_cadio::ImportStep { source: sm_cadio::Source::inline(&bytes), name: None, to_unit: None, assume_unit: None, weld_tol: None, solids: None, sheets: None, check: None, attributes: None }).expect("json");
    assert!(op.get("attributes").is_none());
    // An explicit `false` is recorded as such (the node hash covers the params, so it is not
    // normalised away); only its state is equal to the plain import's.
    let op = serde_json::to_value(sm_cadio::ImportStep { source: sm_cadio::Source::inline(&bytes), name: None, to_unit: None, assume_unit: None, weld_tol: None, solids: None, sheets: None, check: None, attributes: Some(false) }).expect("json");
    assert_eq!(op.get("attributes"), Some(&serde_json::json!(false)));
    // The cells are those of the plain import.
    let topo_a = a.state().layer::<ModelLayer>().expect("model").topo();
    let topo_c = c.state().layer::<ModelLayer>().expect("model").topo();
    use sm_topo_api::Dim;
    for dim in [Dim::Vertex, Dim::Edge, Dim::Face, Dim::Volume] {
        assert_eq!(topo_a.cells(dim), topo_c.cells(dim));
    }
}

#[test]
fn a_file_without_attributes_gets_none() {
    let mut w = ws();
    let (_, diags) = import(&mut w, &data("brick_2x3x4.stp"), Some(true));
    // The product name still names the group; there are no colors.
    assert_eq!(groups(&w).len(), 1);
    assert_eq!(model(&w).colors().count(), 0);
    assert!(has(&diags, "step.attr_groups"));
    assert!(!has(&diags, "step.attr_ignored"));
}

#[test]
fn a_second_import_reuses_colors_and_skips_taken_group_names() {
    let mut w = ws();
    let bytes = data("named_colored.stp");
    import(&mut w, &bytes, Some(true));
    let (out, diags) = import(&mut w, &bytes, Some(true));
    let m = model(&w);
    assert_eq!(m.colors().count(), 2, "colors are reused by name");
    assert_eq!(color_names(&w, &out["volumes"]), [Some("uo2".into()), Some("rgb_0000ff".into()), None, Some("uo2".into())]);
    assert_eq!(m.groups().count(), 4, "taken names are not duplicated");
    assert_eq!(diags.iter().filter(|d| d.code == "step.attr_skipped").count(), 4);
}

#[test]
fn attributes_are_deterministic_and_replay() {
    let bytes = data("named_colored.stp");
    let run = |threads: usize| {
        rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool").install(|| {
            let mut w = ws();
            import(&mut w, &bytes, Some(true));
            (w.state_hash(), groups(&w))
        })
    };
    let a = run(1);
    assert_eq!(a, run(1));
    assert_eq!(a, run(4));
}
