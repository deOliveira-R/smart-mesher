//! Every public data type round-trips through JSON (feature `serde`), including maps whose keys
//! are not strings.
#![cfg(feature = "serde")]

mod common;

use common::*;
use sm_core::{Point3, Transform, Vector3};
use sm_model_api::*;
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::{CellComplex, OpId, TopologyEdit, TopologyView};
use std::collections::BTreeMap;

fn rt<T: serde::Serialize + serde::de::DeserializeOwned + PartialEq + core::fmt::Debug>(v: &T) {
    let s = serde_json::to_string(v).unwrap();
    let back: T = serde_json::from_str(&s).unwrap();
    assert_eq!(&back, v, "{s}");
}

#[test]
fn model_and_results_round_trip() {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])).unwrap();
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[b.volume]).unwrap();
    finish_boundaries(&mut m, &t, 7, true);
    let cap_color = m.add_color(9, "cap", Domain::Face).unwrap();
    with_op(&mut t, OpId(2), |t| {
        let w = fixtures::webcut_brick(t, &b, 0.5)?;
        t.kill_cascade(w.volumes[1])?;
        Ok(())
    })
    .unwrap();
    let plan = plan_cut(&CutSpec { deck: Deck::Mirror { plane: Plane::new(Vector3::new(0.0, 0.0, 1.0), 0.5).unwrap() }, keep: Keep::Negative, origin: Point3::new(0.0, 0.0, 0.5) }).unwrap();
    rt(&plan);
    m.apply_cut(&plan, &t.lineage().cut_outcome(OpId(2)), Some(cap_color), &t).unwrap();
    let vol = sm_topo_api::TopologyViewExt::volumes(&t)[0];
    let pin = m.add_prototype(&[vol]).unwrap();
    let lat = m
        .add_lattice(Lattice {
            layout: Layout::Hex { rings: 2, pitch: 1.0, orient: HexOrient::XRows, nz: 1, dz: 0.0 },
            origin: Transform::IDENTITY,
            outer: Fill::Prototype(pin),
            slots: BTreeMap::from([(SlotIx::new(1, 0, 0), Slot { fill: Fill::Void, recolor: ColorMap::default(), clip: None })]),
        })
        .unwrap();
    let inst = m.add_instance(InstanceRow { of: Fill::Lattice(lat), at: Transform::IDENTITY, recolor: ColorMap::default(), clip: None }).unwrap();
    let faces = sm_topo_api::TopologyViewExt::faces(&t).to_vec();
    m.split(PieceRef::Whole(vol), CutGeometry::Plane(Plane::new(Vector3::new(1.0, 0.0, 0.0), 0.5).unwrap()), &faces, &faces, &t).unwrap();
    m.add_path_override(PathOverride { prefix: [PathStep::Instance(inst)].into_iter().collect(), recolor: ColorMap::default() }).unwrap();
    rt(&m);
    rt(&validate(&m, &t, Tolerance::default()));
    rt(&unfold(&m, &t, &UnfoldSpec::default()).unwrap());
    rt(&detect_symmetry(&m, &t, Tolerance::default()).unwrap());
    rt(&flatten(&m).unwrap());
    rt(&m.deck_lookup(&t));
    rt(&Turns::new(5, 6).unwrap());
    rt(&Provenance::Detected(Verification::Rejected { stage: Stage::Faces, residual: 0.5, reason: "x".into() }));
}

#[test]
fn tampered_models_are_rejected_not_looped_on() {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])).unwrap();
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[b.volume]).unwrap();
    let all = b.faces.to_vec();
    m.split(PieceRef::Whole(b.volume), CutGeometry::Plane(Plane::new(Vector3::new(1.0, 0.0, 0.0), 0.5).unwrap()), &all, &all, &t).unwrap();
    let good = serde_json::to_value(&m).unwrap();
    assert_eq!(serde_json::from_value::<Model>(good.clone()).unwrap(), m);

    // A reset id allocator would reuse ids.
    let mut reset = good.clone();
    reset["ids"] = serde_json::json!({ "next": {} });
    let e = serde_json::from_value::<Model>(reset).unwrap_err().to_string();
    assert!(e.contains("reuse"), "{e}");

    // A split record whose child is itself a parent of its own parent.
    let mut cyc = good.clone();
    let splits = cyc["decomposition"]["splits"].as_array_mut().unwrap();
    let rec = splits[0][1].clone();
    let child = rec["children"][0].clone();
    splits.push(serde_json::json!([{ "Cell": child }, rec]));
    let e = serde_json::from_value::<Model>(cyc).unwrap_err().to_string();
    assert!(e.contains("decomposition"), "{e}");
}
