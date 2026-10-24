//! The `fem.*` ops: roles, model-only transactions, warnings, errors, retirement, schemas.

mod common;

use common::*;
use sm_core::{EntityId, EntityKind};
use sm_fem::{
    BlockAdd, BlockElementType, BlockRemove, ElementType, FemLayer, FemPlugin, SidesetAdd,
};
use sm_layer_api::ModelLayer;
use sm_model_api::Domain;
use sm_op_api::{Cardinality, Registry, Severity, X_ENTITY_KIND, X_POSITIONAL};

fn add(block: u32, volumes: &[EntityId]) -> BlockAdd {
    BlockAdd {
        block,
        volumes: volumes.to_vec(),
        faces: vec![],
        name: None,
    }
}

fn two_bricks() -> (Rig, Vec<EntityId>, Vec<EntityId>) {
    let mut r = Rig::new();
    let d = r.apply(Bricks {
        origins: vec![[0.0; 3], [5.0, 0.0, 0.0]],
        size: [1.0; 3],
    });
    (r, role(&d, "volume"), role(&d, "faces"))
}

fn model(r: &Rig) -> &ModelLayer {
    r.state().layer::<ModelLayer>().expect("model")
}

#[test]
fn block_add_creates_the_color_and_outputs_it_without_a_topology_op() {
    let (mut r, v, _) = two_bricks();
    let ops_before = model(&r).ops_run();
    let d = r.apply(add(7, &v[..1]));
    let color = role(&d, "block");
    assert_eq!(color.len(), 1);
    assert_eq!(color[0].kind(), EntityKind::Color);
    assert_eq!(d.created, color, "only the color id is minted");
    assert!(d.diagnostics.is_empty());
    let m = model(&r);
    assert_eq!(m.ops_run(), ops_before, "model_txn opens no topology op");
    let row = m.model().color(color[0]).expect("row");
    assert_eq!(
        (row.label, row.name.as_str(), row.domain),
        (7, "block_7", Domain::Volume)
    );
    assert_eq!(m.model().volume_color(v[0]), Some(color[0]));
    assert_eq!(m.model().volume_color(v[1]), None);
}

#[test]
fn block_add_extends_an_existing_block_and_reports_the_same_color() {
    let (mut r, v, _) = two_bricks();
    let c = role(&r.apply(add(1, &v[..1])), "block");
    let d = r.apply(add(1, &v[1..]));
    assert_eq!(role(&d, "block"), c, "the surviving color is reported");
    assert!(d.created.is_empty());
    assert_eq!(model(&r).model().volumes_of_color(c[0]), v);
    // Adding the same volumes again changes nothing and warns about nothing.
    let d = r.apply(add(1, &v));
    assert!(d.diagnostics.is_empty());
}

#[test]
fn moving_a_volume_out_of_another_block_warns() {
    let (mut r, v, _) = two_bricks();
    r.apply(add(1, &v));
    let d = r.apply(add(2, &v[..1]));
    assert_eq!(d.diagnostics.len(), 1);
    let w = &d.diagnostics[0];
    assert_eq!(
        (w.severity, w.code.as_str()),
        (Severity::Warning, "block-move")
    );
    assert_eq!(w.entities, vec![v[0]]);
    assert!(
        w.message.contains("block 1") && w.message.contains("block 2"),
        "{}",
        w.message
    );
    let m = model(&r).model();
    assert_ne!(m.volume_color(v[0]), m.volume_color(v[1]));
}

#[test]
fn block_add_rejects_bad_input() {
    let (mut r, v, f) = two_bricks();
    r.apply(add(1, &v[..1]));
    let h0 = r.state().hash();
    // A face is not a volume.
    assert!(r.try_apply(add(2, &f[..1])).is_err());
    // Renaming an existing block, reusing a name, shell blocks.
    let rename = BlockAdd {
        name: Some("steel".into()),
        ..add(1, &[])
    };
    assert!(r.try_apply(rename).is_err());
    let dup = BlockAdd {
        name: Some("block_1".into()),
        ..add(2, &[])
    };
    assert!(r.try_apply(dup).is_err());
    let shell = BlockAdd {
        faces: f[..1].to_vec(),
        ..add(3, &[])
    };
    let e = r.try_apply(shell).expect_err("shell blocks");
    assert!(e.contains("shell"), "{e}");
    assert_eq!(r.state().hash(), h0, "failed ops leave the state untouched");
    // A name is honoured on creation.
    let named = BlockAdd {
        name: Some("steel".into()),
        ..add(5, &v[1..])
    };
    let d = r.apply(named);
    let c = role(&d, "block")[0];
    assert_eq!(model(&r).model().color(c).expect("row").name, "steel");
}

#[test]
fn block_element_type_is_stored_and_validated() {
    let (mut r, v, _) = two_bricks();
    let c = role(&r.apply(add(1, &v[..1])), "block")[0];
    assert!(
        r.state()
            .layer::<FemLayer>()
            .expect("fem")
            .element_type(c)
            .is_none()
    );
    r.apply(BlockElementType {
        block: 1,
        element_type: ElementType::Hex8,
    });
    assert_eq!(
        r.state().layer::<FemLayer>().expect("fem").element_type(c),
        Some(ElementType::Hex8)
    );
    for ho in [ElementType::Hex20, ElementType::Tet10, ElementType::Quad8] {
        let e = r
            .try_apply(BlockElementType {
                block: 1,
                element_type: ho,
            })
            .expect_err("higher order");
        assert!(e.contains("unsupported"), "{e}");
    }
    assert!(
        r.try_apply(BlockElementType {
            block: 9,
            element_type: ElementType::Hex8
        })
        .is_err()
    );
    // The names the file format uses.
    let j = serde_json::to_string(&[ElementType::Hex8, ElementType::Pyramid5, ElementType::Tri3])
        .expect("json");
    assert_eq!(j, r#"["HEX8","PYRAMID5","TRI3"]"#);
}

#[test]
fn block_remove_retires_the_color_and_its_element_type() {
    let (mut r, v, _) = two_bricks();
    let c = role(&r.apply(add(1, &[])), "block")[0];
    r.apply(BlockElementType {
        block: 1,
        element_type: ElementType::Tet4,
    });
    assert_eq!(
        r.state()
            .layer::<FemLayer>()
            .expect("fem")
            .element_types()
            .count(),
        1
    );
    let d = r.apply(BlockRemove { block: 1 });
    assert_eq!(d.removed, vec![c], "the color id is retired");
    assert!(!r.state().contains(c));
    assert!(model(&r).model().color(c).is_none());
    assert!(
        r.state()
            .layer::<FemLayer>()
            .expect("fem")
            .element_type(c)
            .is_none()
    );
    // A block that still holds volumes cannot go.
    r.apply(add(2, &v));
    let e = r
        .try_apply(BlockRemove { block: 2 })
        .expect_err("not empty");
    assert!(e.contains("move them"), "{e}");
    assert!(r.try_apply(BlockRemove { block: 42 }).is_err());
}

#[test]
fn sideset_add_creates_a_face_color_and_records_wrt() {
    let (mut r, v, f) = two_bricks();
    let d = r.apply(SidesetAdd {
        sideset: 3,
        faces: vec![f[1], f[0]],
        wrt: Some(v[0]),
        name: None,
    });
    let c = role(&d, "sideset");
    assert_eq!(c.len(), 1);
    assert_eq!(d.created, c);
    let row = model(&r).model().color(c[0]).expect("row");
    assert_eq!(
        (row.label, row.name.as_str(), row.domain),
        (3, "sideset_3", Domain::Face)
    );
    assert_eq!(model(&r).model().faces_of_color(c[0]), vec![f[0], f[1]]);
    let fem = r.state().layer::<FemLayer>().expect("fem");
    assert_eq!(
        fem.side_prefs().collect::<Vec<_>>(),
        vec![(f[0], v[0]), (f[1], v[0])]
    );
    // Re-adding one face without wrt clears its preference; moving it warns.
    let d = r.apply(SidesetAdd {
        sideset: 4,
        faces: vec![f[1]],
        wrt: None,
        name: Some("inlet".into()),
    });
    assert_eq!(d.diagnostics.len(), 1);
    assert_eq!(d.diagnostics[0].code, "sideset-move");
    let fem = r.state().layer::<FemLayer>().expect("fem");
    assert_eq!(fem.side_prefs().collect::<Vec<_>>(), vec![(f[0], v[0])]);
}

#[test]
fn sideset_add_rejects_bad_input() {
    let (mut r, v, f) = two_bricks();
    let h0 = r.state().hash();
    // wrt must own the face; faces must be faces.
    assert!(
        r.try_apply(SidesetAdd {
            sideset: 1,
            faces: vec![f[6]],
            wrt: Some(v[0]),
            name: None
        })
        .is_err()
    );
    assert!(
        r.try_apply(SidesetAdd {
            sideset: 1,
            faces: v.clone(),
            wrt: None,
            name: None
        })
        .is_err()
    );
    assert!(
        r.try_apply(SidesetAdd {
            sideset: 1,
            faces: vec![f[0]],
            wrt: Some(f[1]),
            name: None
        })
        .is_err()
    );
    assert_eq!(r.state().hash(), h0);
}

#[test]
fn retiring_entities_cleans_the_fem_layer() {
    let (mut r, v, f) = two_bricks();
    r.apply(add(1, &v));
    r.apply(BlockElementType {
        block: 1,
        element_type: ElementType::Hex8,
    });
    r.apply(SidesetAdd {
        sideset: 2,
        faces: vec![f[0], f[1]],
        wrt: Some(v[0]),
        name: None,
    });
    r.apply(SidesetAdd {
        sideset: 3,
        faces: vec![f[6]],
        wrt: Some(v[1]),
        name: None,
    });
    assert_eq!(
        r.state()
            .layer::<FemLayer>()
            .expect("fem")
            .side_prefs()
            .count(),
        3
    );
    let d = r.apply(KillVolume { volume: v[0] });
    assert!(
        d.touched_layers.iter().any(|l| l == "fem"),
        "{:?}",
        d.touched_layers
    );
    let fem = r.state().layer::<FemLayer>().expect("fem");
    assert_eq!(
        fem.side_prefs().collect::<Vec<_>>(),
        vec![(f[6], v[1])],
        "only the dead volume's faces go"
    );
    assert_eq!(fem.element_types().count(), 1, "the block color survives");
}

#[test]
fn fem_layer_digest_depends_on_content_and_empty_layers_are_invisible() {
    let run = |ty: ElementType| {
        let (mut r, v, _) = two_bricks();
        r.apply(add(1, &v));
        r.apply(BlockElementType {
            block: 1,
            element_type: ty,
        });
        r
    };
    let (a, b) = (run(ElementType::Hex8), run(ElementType::Tet4));
    assert_eq!(
        a.state().layer_hash("fem").expect("a"),
        run(ElementType::Hex8)
            .state()
            .layer_hash("fem")
            .expect("a2")
    );
    assert_ne!(
        a.state().layer_hash("fem").expect("a"),
        b.state().layer_hash("fem").expect("b")
    );
    assert_ne!(a.state().hash(), b.state().hash());
    // Installing the plugin does not change the hash of histories that never use it.
    let (mut with, mut without) = (Rig::new(), Rig::without_fem());
    for r in [&mut with, &mut without] {
        r.apply(Bricks {
            origins: vec![[0.0; 3]],
            size: [2.0; 3],
        });
    }
    assert_eq!(with.state().hash(), without.state().hash());
}

// ---- Registry-level conventions (plan §2 "Rules common to every task")

fn catalog() -> Vec<sm_op_api::OpInfo> {
    let mut reg = Registry::new();
    reg.install(&sm_layer_api::CoreLayers::default())
        .expect("layers");
    reg.install(&FemPlugin).expect("fem");
    reg.catalog()
        .into_iter()
        .map(|i| i.expect("info"))
        .filter(|i| i.name.starts_with("fem."))
        .collect()
}

#[test]
fn ops_declare_outputs_positional_and_entity_kinds() {
    let infos = catalog();
    let names: Vec<_> = infos.iter().map(|i| i.name.as_str()).collect();
    assert_eq!(
        names,
        vec![
            "fem.block_add",
            "fem.block_element_type",
            "fem.block_remove",
            "fem.sideset_add"
        ]
    );
    for i in &infos {
        assert!(i.tool_name.len() <= 64 && !i.tool_name.contains('.'));
        let v = serde_json::to_value(&i.schema).expect("schema json");
        let props = v["properties"].as_object().expect("properties");
        // Every EntityId property carries x-entity-kind.
        for (name, p) in props {
            let is_id = p.to_string().contains("EntityId")
                || p.get("$ref")
                    .is_some_and(|r| r.as_str().is_some_and(|r| r.contains("EntityId")));
            if is_id || ["volumes", "faces", "wrt"].contains(&name.as_str()) {
                assert!(p.get(X_ENTITY_KIND).is_some(), "{} .{name}: {p}", i.name);
            }
        }
        // Positional marker appears in the schema where declared.
        let pos = i.positional.as_deref().expect("positional");
        assert!(props[pos].get(X_POSITIONAL).is_some(), "{}", i.name);
    }
    let by = |n: &str| infos.iter().find(|i| i.name == n).expect("op");
    let out = &by("fem.block_add").outputs;
    assert_eq!(
        (out.len(), out[0].name, out[0].cardinality),
        (1, "block", Cardinality::One)
    );
    assert_eq!(out[0].kinds, &[EntityKind::Color]);
    let out = &by("fem.sideset_add").outputs;
    assert_eq!((out.len(), out[0].name), (1, "sideset"));
    assert!(by("fem.block_remove").outputs.is_empty());
    let v = serde_json::to_value(&by("fem.block_add").schema).expect("json");
    assert_eq!(
        v["properties"]["volumes"][X_ENTITY_KIND],
        serde_json::json!(["TopoVolume"])
    );
    let v = serde_json::to_value(&by("fem.sideset_add").schema).expect("json");
    assert_eq!(
        v["properties"]["faces"][X_ENTITY_KIND],
        serde_json::json!(["TopoFace"])
    );
    assert_eq!(
        v["properties"]["wrt"][X_ENTITY_KIND],
        serde_json::json!(["TopoVolume"])
    );
}
