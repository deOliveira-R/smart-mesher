//! Determinism (CLAUDE.md): two runs, 1 thread vs N threads, concurrent runs, cold replay.

mod common;

use std::collections::BTreeMap;
use std::hash::Hasher;

use common::*;
use sm_fem::{
    BlockAdd, BlockElementType, ElementType, ExportFormat, ExportRequest, SidesetAdd, export,
};
use sm_op_api::{Hash256, QuietControl, Session};
use sm_testkit::determinism::{Fnv, check};

fn fnv(bytes: &[u8]) -> u64 {
    let mut h = Fnv::default();
    h.write(bytes);
    h.finish()
}

/// Everything observable: state hash, layer hashes, and the bytes of both exports.
type Observed = (String, BTreeMap<String, String>, u64, u64, usize);

fn scenario(rig: &mut Rig) -> Hash256 {
    let d = rig.apply(Bricks {
        origins: vec![[0.0; 3], [9.0, 0.0, 0.0]],
        size: [5.0; 3],
    });
    let (v, f) = (role(&d, "volume"), role(&d, "faces"));
    rig.apply(MeshIt {
        volumes: v.clone(),
        size: 1.0,
    });
    for (i, vol) in v.iter().enumerate() {
        let block = i as u32 + 1;
        rig.apply(BlockAdd {
            block,
            volumes: vec![*vol],
            faces: vec![],
            name: None,
        });
        rig.apply(BlockElementType {
            block,
            element_type: ElementType::Hex8,
        });
    }
    rig.apply(SidesetAdd {
        sideset: 1,
        faces: vec![f[1], f[7]],
        wrt: None,
        name: None,
    });
    let op = SidesetAdd {
        sideset: 2,
        faces: vec![f[0]],
        wrt: Some(v[0]),
        name: Some("inlet".into()),
    };
    rig.try_apply(op).expect("last op").0
}

fn observe(rig: &Rig, mesh: Option<sm_core::EntityId>) -> Observed {
    let st = rig.state();
    let mut smsh = Vec::new();
    let mut vtu = Vec::new();
    let req = |format| ExportRequest {
        format,
        mesh,
        provenance: Some("00".repeat(32)),
    };
    let rep = export(st, &req(ExportFormat::Smsh), &mut smsh).expect("smsh");
    export(st, &req(ExportFormat::Vtu), &mut vtu).expect("vtu");
    (
        st.hash().to_hex(),
        st.layer_hashes()
            .into_iter()
            .map(|(k, v)| (k, v.to_hex()))
            .collect(),
        fnv(&smsh),
        fnv(&vtu),
        rep.sets.len(),
    )
}

#[test]
fn state_and_exports_are_identical_across_threads_and_runs() {
    let seen = check(2, 4, |threads| {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .expect("pool");
        pool.install(|| {
            let mut rig = Rig::new();
            scenario(&mut rig);
            // Two meshes would be ambiguous; this state has one.
            observe(&rig, None)
        })
    })
    .expect("deterministic");
    assert_eq!(seen.4, 4, "two blocks and two sidesets");
    assert!(
        seen.1.contains_key("fem") && seen.1.contains_key("model") && seen.1.contains_key("meshes")
    );
}

#[test]
fn cold_replay_from_the_root_reproduces_the_state_and_the_exports() {
    let mut warm = Rig::new();
    scenario(&mut warm);
    let want = observe(&warm, None);

    // An engine with a disabled derived cache, replayed node by node from the root: any
    // mismatch with a recorded node check is a `Diverged` error.
    let mut cold = Rig {
        s: Session::new(engine(true)),
    };
    let head = scenario(&mut cold);
    let q = QuietControl::new();
    let replayed = cold
        .s
        .state_at_from_root(head, &q.control())
        .expect("no Diverged");
    assert_eq!(replayed.hash(), cold.state().hash());
    assert_eq!(
        replayed.hash().to_hex(),
        want.0,
        "same state hash as the warm run"
    );
    assert_eq!(
        replayed.layer_hash("fem").expect("fem"),
        warm.state().layer_hash("fem").expect("fem")
    );
    let (mut a, mut b) = (Vec::new(), Vec::new());
    export(&replayed, &ExportRequest::new(ExportFormat::Smsh), &mut a).expect("replayed");
    export(
        warm.state(),
        &ExportRequest::new(ExportFormat::Smsh),
        &mut b,
    )
    .expect("warm");
    assert_eq!(a, b, "byte-identical exports");
}

#[test]
fn undo_and_redo_restore_the_fem_layer() {
    let mut r = Rig::new();
    let d = r.apply(Bricks {
        origins: vec![[0.0; 3]],
        size: [2.0; 3],
    });
    let v = role(&d, "volume");
    r.apply(BlockAdd {
        block: 1,
        volumes: v,
        faces: vec![],
        name: None,
    });
    r.apply(BlockElementType {
        block: 1,
        element_type: ElementType::Hex8,
    });
    let h = r.state().hash();
    r.s.undo().expect("undo");
    assert_ne!(r.state().hash(), h);
    let fem = r.state().layer::<sm_fem::FemLayer>().expect("fem");
    assert_eq!(fem.element_types().count(), 0, "the declaration is undone");
    r.s.redo().expect("redo");
    assert_eq!(r.state().hash(), h);
}
