//! W4-REGIONS-SPLIT acceptance 1: lattice frames, geometric tagging, `model.copy {lattice}`,
//! `model.region_clear` and the `model.region_tags` query.

mod common;

use common::*;
use serde_json::json;
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_model::{CopyXform, CreateBrick, CreatePrism, HexOrientSpec, LatticeDefine, LatticeTag, LayoutSpec, ModelCopy, Move, Outside, RegionClear, RegionTable};
use sm_model_api::{HexOrient, Layout};
use sm_op_api::Session;

const PITCH: f64 = 1.26;

fn vol(n: u64) -> EntityId {
    id(EntityKind::TopoVolume, n)
}

fn cart(n: u32, pitch: f64) -> LayoutSpec {
    LayoutSpec::Cartesian { n: vec![n, n, 1], pitch: vec![pitch, pitch, 1.0] }
}

fn define(s: &mut Session, name: &str, layout: LayoutSpec, parent: Option<&str>) {
    run(s, &LatticeDefine { name: name.into(), layout, origin: None, parent: parent.map(Into::into) }).expect("define");
}

fn copy(entities: Vec<EntityId>, count: u32, delta: [f64; 3], lattice: Option<&str>) -> ModelCopy {
    ModelCopy { entities, count: Some(count), xform: Some(CopyXform::Move { delta }), lattice: lattice.map(Into::into) }
}

/// A 17 x 17 lattice of bricks in two copy steps; returns every volume, ascending.
fn lattice17(s: &mut Session, lattice: Option<&str>) -> Vec<EntityId> {
    run(s, &CreateBrick { x: PITCH, y: Some(PITCH), z: Some(1.0), center: None }).expect("brick");
    let (_, d) = run(s, &copy(vec![vol(1)], 16, [PITCH, 0.0, 0.0], lattice)).expect("row");
    let mut row = vec![vol(1)];
    row.extend(d.outputs["volumes"].iter().copied());
    run(s, &copy(row, 16, [0.0, PITCH, 0.0], lattice)).expect("columns");
    model(s).topo().cells(sm_topo_api::Dim::Volume).to_vec()
}

fn table(s: &Session) -> RegionTable {
    serde_json::from_value(query(s, "model.region_tags", json!({})).expect("query")).expect("table")
}

#[test]
fn tag_17x17_matches_the_rounded_centroid_and_copy_lattice_gives_the_same_table() {
    both(|factory| {
        let mut s = session(factory);
        let vols = lattice17(&mut s, None);
        assert_eq!(vols.len(), 289);
        define(&mut s, "a", cart(17, PITCH), None);
        run(&mut s, &LatticeTag { lattice: "a".into(), volumes: vols.clone(), outside: None }).expect("tag");
        let t = table(&s);
        assert_eq!(t.tags.len(), 289);
        let mut seen = std::collections::BTreeSet::new();
        for row in &t.tags {
            let c = measure(&s, row.volume).centroid.expect("centroid");
            let want = [(c[0] / PITCH).round() as i32, (c[1] / PITCH).round() as i32, 0];
            assert_eq!(row.slots["a"], want, "volume {}", row.volume);
            assert!(seen.insert(want), "slot {want:?} appears twice");
        }
        assert_eq!(seen.len(), 289);
        assert!(seen.iter().all(|s| (0..17).contains(&s[0]) && (0..17).contains(&s[1]) && s[2] == 0));

        // The same lattice by `model.copy {lattice: "a"}` (two steps) gives the identical table.
        let mut c = session(factory);
        define(&mut c, "a", cart(17, PITCH), None);
        let vols_c = lattice17(&mut c, Some("a"));
        assert_eq!(vols_c, vols);
        assert_eq!(model(&c).model().region_tags(), model(&s).model().region_tags());
        assert_eq!(table(&c), t);
    });
}

#[test]
fn region_clear_leaves_floats_of_decks_and_the_state_hash_untouched() {
    // `region_clear` rebuilds the model without its region tables; a deck plane with awkward
    // floats must come back bit for bit (the hash covers it).
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &CreateBrick { x: 1.0, y: Some(1.0), z: Some(1.0), center: None }).expect("brick");
        run(&mut s, &MirrorDeck { face: id(EntityKind::TopoFace, 1), normal: [0.1, 0.7, 0.3], offset: 0.123_456_789_123_456_7 }).expect("deck");
        let before = s.state().hash();
        define(&mut s, "a", cart(2, 1.0), None);
        run(&mut s, &LatticeTag { lattice: "a".into(), volumes: vec![vol(1)], outside: Some(Outside::Skip) }).expect("tag");
        assert_ne!(s.state().hash(), before);
        run(&mut s, &RegionClear::default()).expect("clear");
        assert_eq!(s.state().hash(), before, "deck floats and the rest of the model survive the rebuild");
    });
}

#[test]
fn tagging_is_deterministic_across_runs() {
    let build = || {
        let mut s = session(cell_complex);
        define(&mut s, "a", cart(17, PITCH), None);
        lattice17(&mut s, Some("a"));
        s.state().hash()
    };
    assert_eq!(build(), build(), "same ops, same state hash (tags included)");
}

#[test]
fn hex_frames_tag_prisms_placed_at_their_slots_in_both_orientations() {
    for (orient, spec) in [(HexOrient::XRows, HexOrientSpec::XRows), (HexOrient::YRows, HexOrientSpec::YRows)] {
        both(|factory| {
            let mut s = session(factory);
            let layout = Layout::Hex { rings: 3, pitch: 2.0, orient, nz: 1, dz: 0.0 };
            run(&mut s, &CreatePrism { height: 1.0, sides: 6, radius: 0.5, minor: None, center: None }).expect("prism");
            define(&mut s, "h", LayoutSpec::Hex { rings: 3, pitch: 2.0, orient: spec, nz: None, dz: None }, None);
            let slots = layout.indices();
            assert_eq!(slots.len(), 19);
            let mut copies = Vec::new();
            for ix in &slots {
                let d: Vector3 = layout.offset(*ix);
                let (_, delta) = run(&mut s, &copy(vec![vol(1)], 1, [d.x, d.y, d.z], None)).expect("copy");
                copies.push((delta.outputs["volumes"][0], *ix));
            }
            let all: Vec<EntityId> = copies.iter().map(|c| c.0).collect();
            run(&mut s, &LatticeTag { lattice: "h".into(), volumes: all, outside: None }).expect("tag");
            for (v, ix) in copies {
                let tag = model(&s).model().region_tag(v).expect("tag");
                assert_eq!(tag.slots["h"], ix, "{orient:?} volume {v}");
            }
        });
    }
}

#[test]
fn locate_handles_both_hex_orientations_at_offsets() {
    // The model-api helper that tagging rests on, over every slot of 3 rings.
    for orient in [HexOrient::XRows, HexOrient::YRows] {
        let layout = Layout::Hex { rings: 3, pitch: 1.7, orient, nz: 1, dz: 1.0 };
        let f = sm_model_api::LatticeFrame::new(layout, Point3::new(0.3, -0.2, 0.0), None);
        for ix in layout.indices() {
            let d = layout.offset(ix);
            assert_eq!(f.locate(Point3::new(0.3 + d.x, -0.2 + d.y, 0.0)), Some(ix));
        }
    }
}

#[test]
fn ties_outside_and_unknown_frames_are_typed_errors_naming_the_volume() {
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &CreateBrick { x: 1.0, y: Some(1.0), z: Some(1.0), center: None }).expect("brick");
        // Centre exactly on the boundary between slots 0 and 1 (pitch 2, centre at x = 1).
        run(&mut s, &Move { entities: vec![vol(1)], delta: [1.0, 0.0, 0.0] }).expect("move");
        define(&mut s, "a", cart(3, 2.0), None);
        let tag = |outside| LatticeTag { lattice: "a".into(), volumes: vec![vol(1)], outside };
        let e = run(&mut s, &tag(None)).expect_err("tie").to_string();
        assert!(e.contains("slot boundary") && e.contains(&vol(1).to_string()), "{e}");
        run(&mut s, &tag(Some(Outside::Skip))).expect("skip");
        assert!(table(&s).tags.is_empty(), "a skipped volume stays untagged");
        // Outside the layout (3 slots from x = 0 to 4).
        run(&mut s, &Move { entities: vec![vol(1)], delta: [10.0, 0.0, 0.0] }).expect("move");
        let e = run(&mut s, &tag(None)).expect_err("outside").to_string();
        assert!(e.contains("outside the layout"), "{e}");
        let e = run(&mut s, &LatticeTag { lattice: "nope".into(), volumes: vec![vol(1)], outside: None }).expect_err("frame").to_string();
        assert!(e.contains("unknown lattice frame"), "{e}");
        let e = run(&mut s, &copy(vec![vol(1)], 1, [1.0, 0.0, 0.0], Some("nope"))).expect_err("copy frame").to_string();
        assert!(e.contains("unknown lattice frame"), "{e}");
        // A failed op leaves the model untouched.
        assert!(table(&s).tags.is_empty());
        // Bad frames: duplicate name, unknown parent, bad name.
        assert!(run(&mut s, &LatticeDefine { name: "a".into(), layout: cart(2, 1.0), origin: None, parent: None }).is_err());
        assert!(run(&mut s, &LatticeDefine { name: "b".into(), layout: cart(2, 1.0), origin: None, parent: Some("zz".into()) }).is_err());
        assert!(run(&mut s, &LatticeDefine { name: "b c".into(), layout: cart(2, 1.0), origin: None, parent: None }).is_err());
    });
}

#[test]
fn nested_frames_keep_their_own_slots_and_region_clear_restores_the_hash() {
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &CreateBrick { x: 1.0, y: Some(1.0), z: Some(1.0), center: None }).expect("brick");
        run(&mut s, &Move { entities: vec![vol(1)], delta: [5.0, 0.0, 0.0] }).expect("move");
        let before = s.state().hash();
        // core: 2 x 2 assemblies of pitch 8; assembly: 5 x 5 pins of pitch 1 whose slot (0,0,0)
        // is centred at x = 2.
        define(&mut s, "core", cart(2, 8.0), None);
        let asm = LatticeDefine { name: "asm".into(), layout: cart(5, 1.0), origin: Some([2.0, 0.0, 0.0]), parent: Some("core".into()) };
        run(&mut s, &asm).expect("asm");
        for f in ["core", "asm"] {
            run(&mut s, &LatticeTag { lattice: f.into(), volumes: vec![vol(1)], outside: None }).expect("tag");
        }
        let t = table(&s);
        assert_eq!(t.frames.len(), 2);
        assert_eq!(t.frames[0].name, "asm");
        assert_eq!(t.frames[1].name, "core");
        assert_eq!(t.tags[0].slots["core"], [1, 0, 0]);
        assert_eq!(t.tags[0].slots["asm"], [3, 0, 0]);
        assert_ne!(s.state().hash(), before);
        // Per-volume clear leaves the frames; a full clear restores the original state hash.
        run(&mut s, &RegionClear { volumes: Some(vec![vol(1)]) }).expect("clear one");
        assert!(table(&s).tags.is_empty() && table(&s).frames.len() == 2);
        run(&mut s, &RegionClear::default()).expect("clear all");
        assert_eq!(table(&s), RegionTable::default());
        assert_eq!(s.state().hash(), before, "empty tables hash as before");
    });
}

#[test]
fn two_value_frames_tag_bricks_at_any_z_height_not_only_around_z0() {
    // 2D frame: `n` and `pitch` have two values, so the z pitch defaults to 0 (one z slot).
    for h in [1.0, 100.0] {
        both(|factory| {
            let mut s = session(factory);
            // 3 x 3 bricks spanning z in [0, h] (centroid z = h/2), tagged in one go.
            run(&mut s, &CreateBrick { x: 1.0, y: Some(1.0), z: Some(h), center: None }).expect("brick");
            run(&mut s, &Move { entities: vec![vol(1)], delta: [0.0, 0.0, h / 2.0] }).expect("lift");
            let (_, d) = run(&mut s, &copy(vec![vol(1)], 2, [2.0, 0.0, 0.0], None)).expect("row");
            let mut row = vec![vol(1)];
            row.extend(d.outputs["volumes"].iter().copied());
            run(&mut s, &copy(row, 2, [0.0, 2.0, 0.0], None)).expect("cols");
            let vols = model(&s).topo().cells(sm_topo_api::Dim::Volume).to_vec();
            assert_eq!(vols.len(), 9);
            define(&mut s, "f", LayoutSpec::Cartesian { n: vec![3, 3], pitch: vec![2.0, 2.0] }, None);
            run(&mut s, &LatticeTag { lattice: "f".into(), volumes: vols, outside: None }).expect("tag");
            let t = table(&s);
            assert_eq!(t.tags.len(), 9, "h = {h}");
            let mut seen = std::collections::BTreeSet::new();
            for row in &t.tags {
                let c = measure(&s, row.volume).centroid.expect("centroid");
                assert!((c[2] - h / 2.0).abs() < 1e-9);
                let want = [(c[0] / 2.0).round() as i32, (c[1] / 2.0).round() as i32, 0];
                assert_eq!(row.slots["f"], want);
                assert!(seen.insert(want));
            }
        });
    }
}

#[test]
fn several_z_slots_demand_an_explicit_z_pitch() {
    both(|factory| {
        let mut s = session(factory);
        let hex = LayoutSpec::Hex { rings: 2, pitch: 2.0, orient: HexOrientSpec::XRows, nz: Some(3), dz: None };
        let e = run(&mut s, &LatticeDefine { name: "m".into(), layout: hex, origin: None, parent: None }).expect_err("dz").to_string();
        assert!(e.contains("dz"), "{e}");
        let cart3 = LayoutSpec::Cartesian { n: vec![2, 2, 2], pitch: vec![1.0, 1.0] };
        let e = run(&mut s, &LatticeDefine { name: "c".into(), layout: cart3, origin: None, parent: None }).expect_err("z pitch").to_string();
        assert!(e.contains("z slot"), "{e}");
    });
}
