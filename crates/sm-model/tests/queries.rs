//! T1 acceptance: `topo.related`, `model.list`, `model.measure`, `model.delete`.

mod common;

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use common::*;
use core::f64::consts::PI;
use serde_json::json;
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{MeshLayer, ModelLayer};
use sm_model::{CreateBrick, CreateCylinder, CreateSphere, Delete, Measurement, ModelPlugin};
use sm_op_api::{DerivedCache, Engine, Registry, Services, Session};
use sm_topo_api::{CellComplex, Dim, TopologyView};

const TOPO_KINDS: [EntityKind; 5] = [EntityKind::TopoVertex, EntityKind::TopoEdge, EntityKind::TopoFace, EntityKind::TopoVolume, EntityKind::TopoBody];

/// A session whose `model` layer starts from a ready-made complex (ids are not in the entity
/// table, which `topo.related` does not need).
fn session_over(complex: CellComplex) -> Session {
    let mut reg = Registry::new();
    reg.register_layer_with::<ModelLayer>(move || ModelLayer::new(Box::new(complex.clone()))).expect("model layer");
    reg.install(&ModelPlugin).expect("plugin");
    Session::new(Engine::new(Arc::new(reg), Services::new(), Arc::new(DerivedCache::default())).expect("engine"))
}

fn ids(v: serde_json::Value) -> Vec<EntityId> {
    v.as_array().expect("array").iter().map(|x| x.as_str().expect("str").parse().expect("id")).collect()
}

fn related(s: &Session, kind: EntityKind, within: &[EntityId]) -> Vec<EntityId> {
    let within: Vec<String> = within.iter().map(ToString::to_string).collect();
    ids(query(s, "topo.related", json!({"kind": kind.name(), "within": within})).expect("topo.related"))
}

/// Brute force: the transitive closure of the direct boundary relation, by fixed-point
/// iteration over every cell (a different algorithm from the query's traversal).
fn descendants(t: &dyn TopologyView) -> BTreeMap<EntityId, BTreeSet<EntityId>> {
    let mut d: BTreeMap<EntityId, BTreeSet<EntityId>> = BTreeMap::new();
    for dim in Dim::ALL {
        for &c in t.cells(dim) {
            d.entry(c).or_default().extend(t.boundary(c).iter().map(|i| i.cell));
        }
    }
    loop {
        let mut changed = false;
        let snapshot = d.clone();
        for (c, set) in d.iter_mut() {
            let extra: Vec<EntityId> = set.iter().filter_map(|x| snapshot.get(x)).flatten().copied().collect();
            for x in extra {
                changed |= set.insert(x);
            }
            let _ = c;
        }
        if !changed {
            return d;
        }
    }
}

#[test]
fn topo_related_matches_brute_force_on_every_fixture() {
    for fx in sm_testkit::topo::all_topo_fixtures().expect("fixtures") {
        let desc = descendants(&fx.complex);
        let s = session_over(fx.complex.clone());
        let all: Vec<EntityId> = Dim::ALL.iter().flat_map(|&d| fx.complex.cells(d).to_vec()).collect();
        for &src in &all {
            for kind in TOPO_KINDS {
                let want: BTreeSet<EntityId> = all
                    .iter()
                    .copied()
                    .filter(|&t| t.kind() == kind && (t == src || desc[&src].contains(&t) || desc[&t].contains(&src)))
                    .collect();
                let got = related(&s, kind, &[src]);
                assert_eq!(got, want.iter().copied().collect::<Vec<_>>(), "{}: {src} -> {}", fx.name, kind.name());
                assert!(got.windows(2).all(|w| w[0] < w[1]), "sorted and distinct");
            }
        }
        // A set of sources gives the union.
        let vols = fx.complex.cells(Dim::Volume).to_vec();
        let got = related(&s, EntityKind::TopoFace, &vols);
        let want: BTreeSet<EntityId> = vols.iter().flat_map(|v| desc[v].iter().copied().filter(|c| c.kind() == EntityKind::TopoFace)).collect();
        assert_eq!(got, want.into_iter().collect::<Vec<_>>(), "{}", fx.name);
    }
}

#[test]
fn topo_related_named_relations_on_a_brick() {
    let mut s = session(topo_store);
    run(&mut s, &CreateBrick { x: 1.0, y: None, z: None, center: None }).expect("brick");
    let vol = id(EntityKind::TopoVolume, 1);
    assert_eq!(related(&s, EntityKind::TopoFace, &[vol]), range(EntityKind::TopoFace, 6), "volume -> faces");
    assert_eq!(related(&s, EntityKind::TopoVolume, &[id(EntityKind::TopoFace, 3)]), vec![vol], "face -> volumes");
    assert_eq!(related(&s, EntityKind::TopoVolume, &[id(EntityKind::TopoBody, 1)]), vec![vol], "body -> volumes");
    assert_eq!(related(&s, EntityKind::TopoEdge, &[id(EntityKind::TopoFace, 1)]).len(), 4, "face -> edges");
    assert_eq!(related(&s, EntityKind::TopoFace, &[id(EntityKind::TopoEdge, 1)]).len(), 2, "edge -> faces");
    assert_eq!(related(&s, EntityKind::TopoEdge, &[id(EntityKind::TopoVertex, 1)]).len(), 3, "vertex -> edges");
    assert_eq!(related(&s, EntityKind::TopoVertex, &[vol]), range(EntityKind::TopoVertex, 8), "volume -> vertices");
    assert_eq!(related(&s, EntityKind::TopoBody, &[id(EntityKind::TopoVertex, 1)]), vec![id(EntityKind::TopoBody, 1)], "vertex -> body");
    assert_eq!(related(&s, EntityKind::TopoFace, &[]), Vec::<EntityId>::new());
}

#[test]
fn topo_related_rejects_bad_input() {
    let mut s = session(cell_complex);
    run(&mut s, &CreateBrick { x: 1.0, y: None, z: None, center: None }).expect("brick");
    assert!(query(&s, "topo.related", json!({"kind": "Color", "within": ["TopoVolume#1"]})).is_err());
    assert!(query(&s, "topo.related", json!({"kind": "TopoFace", "within": ["TopoVolume#9"]})).is_err());
    assert!(query(&s, "topo.related", json!({"kind": "TopoFace", "within": ["Color#1"]})).is_err());
    assert!(query(&s, "topo.related", json!({"kind": "TopoFace"})).is_err());
}

#[test]
fn model_list_reports_topology_and_model_records() {
    let mut s = session(cell_complex);
    run(&mut s, &CreateBrick { x: 1.0, y: None, z: None, center: None }).expect("brick");
    run(&mut s, &CreateBrick { x: 1.0, y: None, z: None, center: Some([5.0, 0.0, 0.0]) }).expect("brick 2");
    let n = |k: &str| ids(query(&s, "model.list", json!({"kind": k})).expect("list")).len();
    assert_eq!((n("TopoVertex"), n("TopoEdge"), n("TopoFace"), n("TopoVolume"), n("TopoBody")), (16, 24, 12, 2, 2));
    assert_eq!((n("Color"), n("Group"), n("Boundary"), n("Prototype")), (0, 0, 0, 0));
    assert!(query(&s, "model.list", json!({"kind": "Mesh"})).is_err());
    let vols = ids(query(&s, "model.list", json!({"kind": "TopoVolume"})).expect("list"));
    assert_eq!(vols, range(EntityKind::TopoVolume, 2));
}

fn measure(s: &Session, e: EntityId) -> Measurement {
    serde_json::from_value(query(s, "model.measure", json!({"entity": e.to_string()})).expect("measure")).expect("decode")
}

fn near(a: f64, b: f64, rel: f64) {
    assert!((a - b).abs() <= rel * b.abs().max(1.0), "{a} vs {b}");
}

#[test]
fn measure_brick_edges_faces_volume_and_boxes() {
    let mut s = session(topo_store);
    run(&mut s, &CreateBrick { x: 2.0, y: Some(4.0), z: Some(6.0), center: Some([1.0, 0.0, -1.0]) }).expect("brick");
    let v = measure(&s, id(EntityKind::TopoVolume, 1));
    near(v.volume.expect("volume"), 48.0, 1e-12);
    near(v.area.expect("area"), 2.0 * (8.0 + 24.0 + 12.0), 1e-12);
    assert_eq!(v.bbox.lo, [0.0, -2.0, -4.0]);
    assert_eq!(v.bbox.hi, [2.0, 2.0, 2.0]);
    let c = v.centroid.expect("centroid");
    for (a, b) in c.iter().zip([1.0, 0.0, -1.0]) {
        assert!((a - b).abs() < 1e-12);
    }
    assert_eq!(measure(&s, id(EntityKind::TopoBody, 1)).volume, v.volume);
    // Face -x is 4 x 6; edge 1 runs along x (length 2); a vertex has a degenerate box.
    near(measure(&s, id(EntityKind::TopoFace, 1)).area.expect("area"), 24.0, 1e-12);
    near(measure(&s, id(EntityKind::TopoEdge, 1)).length.expect("length"), 2.0, 1e-12);
    let p = measure(&s, id(EntityKind::TopoVertex, 8));
    assert_eq!(p.bbox.lo, p.bbox.hi);
    assert_eq!(p.bbox.hi, [2.0, 2.0, 2.0]);
}

#[test]
fn measure_cylinder_and_sphere_match_closed_forms() {
    let (r, h) = (1.5, 4.0);
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = session(factory);
        run(&mut s, &CreateCylinder { height: h, radius: r, center: Some([1.0, 2.0, 3.0]) }).expect("cyl");
        let m = measure(&s, id(EntityKind::TopoVolume, 1));
        near(m.volume.expect("v"), PI * r * r * h, 1e-9);
        near(m.area.expect("a"), 2.0 * PI * r * r + 2.0 * PI * r * h, 1e-9);
        for k in 0..2 {
            assert!((m.bbox.lo[k] - ([1.0, 2.0][k] - r)).abs() < 1e-9, "{:?}", m.bbox);
            assert!((m.bbox.hi[k] - ([1.0, 2.0][k] + r)).abs() < 1e-9, "{:?}", m.bbox);
        }
        assert!((m.bbox.lo[2] - 1.0).abs() < 1e-12 && (m.bbox.hi[2] - 5.0).abs() < 1e-12);
        near(measure(&s, id(EntityKind::TopoFace, 3)).area.expect("lateral"), 2.0 * PI * r * h, 1e-9);
        near(measure(&s, id(EntityKind::TopoEdge, 1)).length.expect("circle"), 2.0 * PI * r, 1e-9);
        near(measure(&s, id(EntityKind::TopoEdge, 3)).length.expect("seam"), h, 1e-12);

        let mut s = session(factory);
        let r = 2.0;
        run(&mut s, &CreateSphere { radius: r, center: Some([1.0, 2.0, 3.0]) }).expect("sphere");
        let m = measure(&s, id(EntityKind::TopoVolume, 1));
        near(m.volume.expect("v"), 4.0 / 3.0 * PI * r * r * r, 1e-12);
        near(m.area.expect("a"), 4.0 * PI * r * r, 1e-12);
        assert_eq!(m.bbox.lo, [-1.0, 0.0, 1.0]);
        assert_eq!(m.bbox.hi, [3.0, 4.0, 5.0]);
        assert_eq!(m.centroid, Some([1.0, 2.0, 3.0]));
        // The meridian seam is a half circle.
        near(measure(&s, id(EntityKind::TopoEdge, 1)).length.expect("seam"), PI * r, 1e-9);
        assert_eq!(measure(&s, id(EntityKind::TopoEdge, 2)).length, Some(0.0), "pole edge");
    }
}

#[test]
fn measure_rejects_unknown_and_non_topology_ids() {
    let mut s = session(cell_complex);
    run(&mut s, &CreateBrick { x: 1.0, y: None, z: None, center: None }).expect("brick");
    assert!(query(&s, "model.measure", json!({"entity": "TopoVolume#7"})).is_err());
    assert!(query(&s, "model.measure", json!({"entity": "Color#1"})).is_err());
}

#[test]
fn delete_retires_ids_and_later_queries_do_not_list_them() {
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = session(factory);
        run(&mut s, &CreateBrick { x: 1.0, y: None, z: None, center: None }).expect("brick 1");
        run(&mut s, &CreateBrick { x: 1.0, y: None, z: None, center: Some([5.0, 0.0, 0.0]) }).expect("brick 2");
        let live_before = s.state().entity_count();
        let (_, d) = run(&mut s, &Delete { bodies: vec![id(EntityKind::TopoBody, 1)] }).expect("delete");
        assert_eq!(d.removed.len(), 28, "8 + 12 + 6 + 1 + 1 retired");
        assert!(d.created.is_empty());
        assert_eq!(s.state().entity_count(), live_before - 28);
        for k in TOPO_KINDS {
            let first = id(k, 1);
            assert!(!s.state().contains(first), "{first} is gone");
        }
        // The second brick is untouched; the first brick's ids are never listed again.
        let faces = related(&s, EntityKind::TopoFace, &[id(EntityKind::TopoVolume, 2)]);
        assert_eq!(faces.len(), 6);
        assert!(faces.iter().all(|f| f.serial() > 6));
        let list = ids(query(&s, "model.list", json!({"kind": "TopoVolume"})).expect("list"));
        assert_eq!(list, vec![id(EntityKind::TopoVolume, 2)]);
        assert!(query(&s, "topo.related", json!({"kind": "TopoFace", "within": ["TopoVolume#1"]})).is_err(), "retired source");
        // A retired body cannot be deleted again.
        let before = s.state().hash();
        assert!(run(&mut s, &Delete { bodies: vec![id(EntityKind::TopoBody, 1)] }).is_err());
        assert!(run(&mut s, &Delete { bodies: vec![] }).is_err());
        assert!(run(&mut s, &Delete { bodies: vec![id(EntityKind::TopoVolume, 2)] }).is_err(), "volumes are not bodies");
        assert_eq!(s.state().hash(), before);
        // Deleting the rest empties the topology.
        run(&mut s, &Delete { bodies: vec![id(EntityKind::TopoBody, 2)] }).expect("delete 2");
        assert_eq!(s.state().entity_count(), 0);
        assert!(model(&s).topo().cells(Dim::Volume).is_empty());
    }
}

#[test]
fn delete_marks_meshes_on_the_volume_stale() {
    let mut s = session(cell_complex);
    run(&mut s, &CreateBrick { x: 1.0, y: None, z: None, center: None }).expect("brick");
    run(&mut s, &FakeMesh { volume: id(EntityKind::TopoVolume, 1) }).expect("mesh");
    let layer = s.state().layer::<MeshLayer>().expect("meshes");
    assert_eq!(layer.len(), 1);
    assert!(layer.meshes().all(|(_, e)| !e.stale));
    run(&mut s, &Delete { bodies: vec![id(EntityKind::TopoBody, 1)] }).expect("delete");
    let layer = s.state().layer::<MeshLayer>().expect("meshes");
    assert_eq!(layer.len(), 1, "stale, not deleted");
    assert!(layer.meshes().all(|(_, e)| e.stale), "the mesh on the deleted volume is stale");
}

#[test]
fn every_declared_entity_id_has_a_kind_annotation_and_ops_declare_outputs() {
    let reg = registry(cell_complex);
    for name in reg.op_names() {
        if !name.starts_with("model.") {
            continue;
        }
        let schema = serde_json::to_value(reg.schema(name).expect("schema")).expect("json");
        check_kinds(name, &schema);
        if name != "model.delete" {
            assert!(!reg.op_outputs(name).is_empty(), "{name} declares OUTPUTS");
        }
    }
    for q in reg.query_catalog() {
        let q = q.expect("query info");
        check_kinds(&q.name, &serde_json::to_value(&q.schema).expect("json"));
    }
}

fn check_kinds(name: &str, schema: &serde_json::Value) {
    let Some(props) = schema["properties"].as_object() else { return };
    for (field, p) in props {
        let is_id = |v: &serde_json::Value| v["$ref"].as_str().is_some_and(|r| r.ends_with("/EntityId"));
        let id_typed = is_id(p) || is_id(&p["items"]);
        if id_typed {
            let kinds = p["x-entity-kind"].as_array().unwrap_or_else(|| panic!("{name}.{field} lacks x-entity-kind"));
            assert!(!kinds.is_empty());
        }
    }
}

#[test]
fn delete_keeps_the_model_tables_consistent() {
    use sm_core::EntityKind::{TopoBody, TopoFace};
    let mk = || {
        let mut s = session(topo_store);
        run(&mut s, &CreateBrick { x: 2.0, y: None, z: None, center: Some([1.0, 1.0, 1.0]) }).expect("brick A");
        run(&mut s, &CreateBrick { x: 2.0, y: None, z: None, center: Some([10.0, 1.0, 1.0]) }).expect("brick B");
        // Deck on A's +z face (z = 2); the helper colors every volume and every boundary face.
        run(&mut s, &MirrorDeck { face: id(TopoFace, 6), normal: [0.0, 0.0, 1.0], offset: 2.0 }).expect("deck");
        assert_eq!(model_errors(&s), Vec::<String>::new(), "valid before");
        s
    };
    // Deleting B leaves A, its color and its deck intact.
    let mut s = mk();
    run(&mut s, &Delete { bodies: vec![id(TopoBody, 2)] }).expect("delete B");
    assert_eq!(model_errors(&s), Vec::<String>::new(), "valid after deleting B");
    // Deleting A removes the deck's face; the rest stays valid.
    let mut s = mk();
    run(&mut s, &Delete { bodies: vec![id(TopoBody, 1)] }).expect("delete A");
    assert_eq!(model_errors(&s), Vec::<String>::new(), "valid after deleting A");
}
