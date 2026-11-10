//! W4-REGIONS-SPLIT: `lower_split` on a small meshed model (bricks with region tags), the typed
//! errors, and the no-split identity.

mod common;

use std::sync::Arc;

use common::*;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_fem::region::RegionSplit;
use sm_fem::{BlockAdd, FemLayer, SetOrigin, lower, lower_split};
use sm_layer_api::{LayerResultExt, MeshLayer, ModelLayer, model_txn};
use sm_mesh_api::SetMembers;
use sm_model_api::{Fill, InstanceRow, LatticeFrame, Layout, RegionTag, SlotIx, SubIndex};
use sm_op_api::{
    ApplyCtx, DerivedCache, Engine, Meta, OpError, QuietControl, Services, Session, TypedOp,
};

/// Defines frame `a` (2 x 1, pitch 5) and tags volume 0 as slot (0,0) ring 0 of 2, volume 1 as
/// slot (1,0) (test only).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct TagTwo {
    volumes: Vec<EntityId>,
}

impl TypedOp for TagTwo {
    const NAME: &'static str = "test.tag_two";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        model_txn(
            ctx,
            |m, _t| {
                let layout = Layout::Cartesian {
                    n: [2, 1, 1],
                    pitch: Vector3::new(5.0, 5.0, 1.0),
                };
                m.define_lattice_frame(
                    "a",
                    LatticeFrame::new(layout, Point3::new(0.0, 0.0, 0.0), None),
                )
                .or_op()?;
                let t0 = RegionTag::new(
                    [("a".to_string(), SlotIx::new(0, 0, 0))].into(),
                    Some(SubIndex::new(0, 2)),
                    None,
                );
                m.set_region_tag(self.volumes[0], t0).or_op()?;
                let t1 =
                    RegionTag::new([("a".to_string(), SlotIx::new(1, 0, 0))].into(), None, None);
                m.set_region_tag(self.volumes[1], t1).or_op()
            },
            |_, _| Vec::new(),
        )
    }
}

/// Adds a prototype and one instance of it (test only).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct AddInstance {
    volume: EntityId,
}

impl TypedOp for AddInstance {
    const NAME: &'static str = "test.add_instance";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        model_txn(
            ctx,
            |m, _t| {
                let p = m.add_prototype(&[self.volume]).or_op()?;
                m.add_instance(InstanceRow::new(Fill::Prototype(p), Transform::IDENTITY))
                    .or_op()?;
                Ok(())
            },
            |_, _| Vec::new(),
        )
    }
}

fn session() -> Session {
    let mut reg = registry(true);
    reg.register::<TagTwo>().expect("tag");
    reg.register::<AddInstance>().expect("instance");
    Session::new(
        Engine::new(
            Arc::new(reg),
            Services::new(),
            Arc::new(DerivedCache::default()),
        )
        .expect("engine"),
    )
}

fn apply<T: TypedOp>(s: &mut Session, op: T) -> sm_op_api::Delta {
    let q = QuietControl::new();
    s.apply(&op, Meta::new("test", "", ""), &q.control())
        .expect("op")
        .1
}

/// Two meshed bricks (8 cells each), blocks `fuel` and `water`, tags from `TagTwo`.
fn model(names: [&str; 2], second_in_block: bool) -> (Session, Vec<EntityId>) {
    let mut s = session();
    let d = apply(
        &mut s,
        Bricks {
            origins: vec![[0.0; 3], [5.0, 0.0, 0.0]],
            size: [2.0; 3],
        },
    );
    let v = role(&d, "volume");
    apply(
        &mut s,
        MeshIt {
            volumes: v.clone(),
            size: 1.0,
        },
    );
    let add = |block, name: &str, volumes| BlockAdd {
        block,
        volumes,
        faces: vec![],
        name: Some(name.into()),
    };
    apply(&mut s, add(1, names[0], vec![v[0]]));
    if second_in_block {
        apply(&mut s, add(2, names[1], vec![v[1]]));
    }
    apply(&mut s, TagTwo { volumes: v.clone() });
    (s, v)
}

fn split(s: &Session, sp: &RegionSplit) -> Result<sm_fem::Lowered, sm_fem::SplitError> {
    let st = s.state();
    let (model, fem, layer) = (
        st.layer::<ModelLayer>().expect("model"),
        st.layer::<FemLayer>().expect("fem"),
        st.layer::<MeshLayer>().expect("meshes"),
    );
    let (_, entry) = layer.meshes().next().expect("mesh");
    lower_split(model, entry, fem, layer.alloc(), sp)
}

fn cells(l: &sm_fem::Lowered, set: &str) -> Vec<u32> {
    let m = (*l.mesh).as_read();
    let SetMembers::Cells(c) = &m
        .set_by_name(set)
        .unwrap_or_else(|| panic!("set {set}"))
        .members
    else {
        panic!("cell set")
    };
    c.iter().map(|c| c.0).collect()
}

#[test]
fn region_sets_are_cell_sets_over_the_blocks_and_the_blocks_stay() {
    let (s, _) = model(["fuel", "water"], true);
    let all = RegionSplit {
        lattice: true,
        ring: true,
        sector: true,
    };
    let l = split(&s, &all).expect("split");
    let names: Vec<&str> = l.sets.iter().map(|x| x.name.as_str()).collect();
    assert_eq!(
        names,
        [
            "fuel",
            "water",
            "region/fuel@a[0,0]/r0",
            "region/water@a[1,0]"
        ]
    );
    assert_eq!(l.sets[2].origin, SetOrigin::Region);
    // The block sets are those of the plain lowering, cell for cell.
    let st = s.state();
    let (model, fem, layer) = (
        st.layer::<ModelLayer>().expect("model"),
        st.layer::<FemLayer>().expect("fem"),
        st.layer::<MeshLayer>().expect("meshes"),
    );
    let (_, entry) = layer.meshes().next().expect("mesh");
    let plain = lower(model, entry, fem, layer.alloc()).expect("lower");
    for b in ["fuel", "water"] {
        assert_eq!(cells(&l, b), cells(&plain, b), "{b}");
    }
    assert_eq!(cells(&l, "region/fuel@a[0,0]/r0"), cells(&l, "fuel"));
    assert_eq!(cells(&l, "region/water@a[1,0]"), cells(&l, "water"));
    assert_eq!(cells(&l, "fuel").len(), 8);
    let r = &l.regions;
    assert_eq!(
        r.iter().map(|x| x.name.as_str()).collect::<Vec<_>>(),
        ["fuel@a[0,0]/r0", "water@a[1,0]"]
    );
    assert_eq!(
        r[0].tags,
        [
            ("lat.a.i", 0),
            ("lat.a.j", 0),
            ("ring", 0),
            ("ring_count", 2)
        ]
        .map(|(k, v)| (k.to_string(), v))
    );
    assert_eq!(
        r[1].tags,
        [("lat.a.i", 1), ("lat.a.j", 0)].map(|(k, v)| (k.to_string(), v))
    );
    // Parts not asked for are not in the names or the tags; the untagged part is just the block.
    let l = split(
        &s,
        &RegionSplit {
            ring: true,
            ..RegionSplit::NONE
        },
    )
    .expect("ring only");
    assert_eq!(
        l.regions
            .iter()
            .map(|x| x.name.as_str())
            .collect::<Vec<_>>(),
        ["fuel/r0", "water"]
    );
    // An empty split is the plain lowering.
    let none = split(&s, &RegionSplit::NONE).expect("none");
    assert_eq!(
        none.sets.iter().map(|x| &x.name).collect::<Vec<_>>(),
        plain.sets.iter().map(|x| &x.name).collect::<Vec<_>>()
    );
    assert!(none.regions.is_empty());
}

#[test]
fn errors_are_typed() {
    let ring = RegionSplit {
        ring: true,
        ..RegionSplit::NONE
    };
    let code = |r: Result<sm_fem::Lowered, sm_fem::SplitError>| match r.err().expect("an error") {
        sm_fem::SplitError::Region(e) => e.code(),
        other => panic!("{other}"),
    };
    let (s, _) = model(["fuel", "water"], false);
    assert_eq!(code(split(&s, &ring)), "regions-need-blocks");
    let (s, _) = model(["u@235", "water"], true);
    assert_eq!(code(split(&s, &ring)), "region-name-reserved");
    let (mut s, v) = model(["fuel", "water"], true);
    apply(&mut s, AddInstance { volume: v[0] });
    assert_eq!(code(split(&s, &ring)), "unsupported-instances");
    // Without a split the instances do not matter to the lowering.
    assert!(split(&s, &RegionSplit::NONE).is_ok());
}
