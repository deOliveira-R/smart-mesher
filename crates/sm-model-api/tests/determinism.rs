//! Determinism: the whole pipeline run twice and on several threads gives identical hashes.
//! (API crates do not depend on rayon, so the cross-thread check uses `std::thread`.)

mod common;

use common::*;
use sm_core::{Point3, Transform, Vector3};
use sm_model_api::*;
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::{CellComplex, OpId, TopologyEdit, TopologyView};
use std::collections::BTreeMap;

fn fnv(s: &str) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for b in s.bytes() {
        h ^= u64::from(b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// Cut, validate, unfold, detect, instance, flatten and decompose; hash everything observable.
fn pipeline() -> Vec<u64> {
    let mut out = Vec::new();
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])).unwrap();
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    let other = m.add_color(2, "other", Domain::Volume).unwrap();
    m.color_volumes(mat, &[b.volume]).unwrap();
    finish_boundaries(&mut m, &t, 7, true);
    let cap_color = m.add_color(9, "cap", Domain::Face).unwrap();
    with_op(&mut t, OpId(2), |t| {
        let w = fixtures::webcut_brick(t, &b, 0.5)?;
        t.kill_cascade(w.volumes[1])?;
        Ok(())
    })
    .unwrap();
    let plan = plan_cut(&CutSpec::new(Deck::Mirror { plane: Plane::new(Vector3::new(0.0, 0.0, 1.0), 0.5).unwrap() }, Keep::Negative, Point3::new(0.0, 0.0, 0.5))).unwrap();
    m.apply_cut(&plan, &t.lineage().cut_outcome(OpId(2)), Some(cap_color), &t).unwrap();
    out.push(fnv(&format!("{m:?}")));
    out.push(fnv(&format!("{:?}", validate(&m, &t, Tolerance::default()))));
    let u = unfold(&m, &t, &UnfoldSpec::default()).unwrap();
    out.push(fnv(&format!("{u:?}")));
    out.push(fnv(&format!("{:?}", detect_symmetry(&m, &t, Tolerance::default()).unwrap())));
    let look = m.deck_lookup(&t);
    out.push(fnv(&format!("{look:?}")));
    // Instancing.
    let vol = *t.volumes_for_test().first().unwrap();
    let pin = m.add_prototype(&[vol]).unwrap();
    let lat = m
        .add_lattice(Lattice::new(Layout::Hex { rings: 3, pitch: 1.0, orient: HexOrient::YRows, nz: 2, dz: 2.0 }, Transform::IDENTITY, Fill::Prototype(pin), BTreeMap::from([(SlotIx::new(1, -1, 1), Slot::new(Fill::Prototype(pin)).with_recolor(ColorMap { volume: BTreeMap::from([(mat, other)]), face: BTreeMap::new() }))])))
        .unwrap();
    m.add_instance(InstanceRow::new(Fill::Lattice(lat), Transform::translation(Vector3::new(1.0, 2.0, 3.0)))).unwrap();
    let f = flatten(&m).unwrap();
    out.push(f.fingerprint());
    out.push(f.fingerprint_quantized(1e-8));
    out.push(fnv(&format!("{f:?}")));
    // Decomposition.
    let faces = sm_topo_api::TopologyViewExt::faces(&t).to_vec();
    m.split(PieceRef::Whole(vol), CutGeometry::Plane(Plane::new(Vector3::new(1.0, 0.0, 0.0), 0.5).unwrap()), &faces, &faces, &t).unwrap();
    out.push(fnv(&format!("{:?}", m.decomposition())));
    out
}

trait VolumesForTest {
    fn volumes_for_test(&self) -> Vec<sm_core::EntityId>;
}
impl VolumesForTest for CellComplex {
    fn volumes_for_test(&self) -> Vec<sm_core::EntityId> {
        sm_topo_api::TopologyViewExt::volumes(self).to_vec()
    }
}

#[test]
fn two_runs_and_four_threads_agree() {
    let a = pipeline();
    let b = pipeline();
    assert_eq!(a, b);
    let hs: Vec<Vec<u64>> = std::thread::scope(|s| {
        let handles: Vec<_> = (0..4).map(|_| s.spawn(pipeline)).collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    });
    for h in hs {
        assert_eq!(h, a, "a thread produced a different result");
    }
}
