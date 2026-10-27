//! `mesh.flag_cells`: the cell set of failing cells, atomicity, undo/redo and replay.
#![allow(missing_docs)]

mod common;
use common::*;
use serde_json::json;
use sm_core::{EntityId, EntityKind};
use sm_diag::{Level, flagged_cells};
use sm_mesh_api::reference::ReferenceMesh;
use sm_mesh_api::{MeshRead, SerialAlloc, SetMembers};
use sm_op_api::QuietControl;
use sm_quality::{CellStatus, FvEntity, Measure as Ms, Options, Profile, compute, fv_compute, fv_geometry, rules, validity};
use std::collections::BTreeSet;

/// 3x3x3 block with the first interior vertex pushed hard: a few cells fail.
fn damaged_moves() -> Vec<(usize, [f64; 3])> {
    let m0 = block([3, 3, 3], [1.0; 3], &[], &mut SerialAlloc::new());
    let v = m0.classified_on(id(EntityKind::TopoVolume, 1)).unwrap().vertices[0].get();
    let p = m0.point(v);
    vec![(v, [p.x + 0.8, p.y + 0.7, p.z + 0.6])]
}

/// Cells graded at or above `min_rank` (1 warn, 2 fail), computed with plain loops.
fn expected(m: &ReferenceMesh, profile: Profile, min_rank: u8, include_invalid: bool) -> Vec<u32> {
    let mut flagged: BTreeSet<u32> = BTreeSet::new();
    let geom = fv_geometry(m).unwrap();
    for r in rules(profile) {
        let (values, face) = match r.measure {
            Ms::Verdict(metric) => match compute(m, metric, &Options::default()) {
                Ok(v) => (v, false),
                Err(_) => continue,
            },
            Ms::Fv(f) => {
                let c = fv_compute(m, &geom, &[f]).unwrap().remove(0);
                (c.values, c.entity == FvEntity::Face)
            }
            _ => continue,
        };
        let rank = |x: f64, boundary: bool| match r.grade(x, boundary) {
            sm_quality::Grade::Warn => 1,
            sm_quality::Grade::Fail => 2,
            _ => 0,
        };
        if face {
            for (f, &x) in values.iter().enumerate() {
                let boundary = m.face_right(sm_mesh_api::FaceIx::new(f)).is_none();
                if rank(x, boundary) >= min_rank {
                    flagged.insert(m.face_left(sm_mesh_api::FaceIx::new(f)).0);
                    if let Some(c) = m.face_right(sm_mesh_api::FaceIx::new(f)) {
                        flagged.insert(c.0);
                    }
                }
            }
        } else {
            for (c, x) in values.iter().enumerate() {
                if rank(*x, false) >= min_rank {
                    flagged.insert(c as u32);
                }
            }
        }
    }
    if include_invalid {
        let v = validity(m, &Options::default()).unwrap();
        for (c, s) in v.status.iter().enumerate() {
            if matches!(s, CellStatus::Inverted | CellStatus::Degenerate) {
                flagged.insert(c as u32);
            }
        }
    }
    flagged.into_iter().collect()
}

fn members_of(rig: &Rig, mesh: EntityId, set: EntityId) -> Vec<u32> {
    let layer = rig.state().layer::<sm_layer_api::MeshLayer>().unwrap();
    let m = layer.get(mesh).unwrap().mesh();
    let s = m.sets().iter().find(|s| s.id == set).expect("set in mesh");
    match &s.members {
        SetMembers::Cells(c) => c.iter().map(|c| c.0).collect(),
        other => panic!("not a cell set: {other:?}"),
    }
}

#[test]
fn flags_exactly_the_failing_cells() {
    let moves = damaged_moves();
    let reference = block([3, 3, 3], [1.0; 3], &moves, &mut SerialAlloc::new());
    for (profile, level, rank, invalid) in [
        (Profile::Generic, Level::Fail, 2u8, true),
        (Profile::Generic, Level::Fail, 2, false),
        (Profile::Generic, Level::Warn, 1, true),
        (Profile::OpenFoam, Level::Fail, 2, true),
    ] {
        let want = expected(&reference, profile, rank, invalid);
        let got: Vec<u32> = flagged_cells(&reference, profile, None, &Default::default(), level, invalid).unwrap().iter().map(|c| c.0).collect();
        assert_eq!(got, want, "{profile:?} {level:?} invalid={invalid}");
    }
    let n_fail = expected(&reference, Profile::Generic, 2, true).len();
    assert!(n_fail > 0 && n_fail < 27, "the damage fails some but not all cells: {n_fail}");
    assert!(expected(&reference, Profile::Generic, 1, true).len() >= n_fail);
}

#[test]
fn op_stores_a_live_set_and_is_atomic_and_replayable() {
    let moves = damaged_moves();
    let reference = block([3, 3, 3], [1.0; 3], &moves, &mut SerialAlloc::new());
    let want = expected(&reference, Profile::Generic, 2, true);
    let (mut rig, mesh) = Rig::with_block([3, 3, 3], &moves);
    let sets_before = rig.state().count_of(EntityKind::MeshSet);
    let hash0 = rig.state().hash();
    let (_, d) = rig.json("mesh.flag_cells", json!({"name": "bad", "profile": "generic"})).unwrap();
    let set = d.outputs["set"][0];
    assert_eq!(set.kind(), EntityKind::MeshSet);
    assert!(rig.state().contains(set));
    assert_eq!(rig.state().count_of(EntityKind::MeshSet), sets_before + 1);
    assert_ne!(rig.state().hash(), hash0);
    assert_eq!(members_of(&rig, mesh, set), want);
    assert!(d.created.contains(&set) && d.diagnostics.is_empty());
    let hash1 = rig.state().hash();
    // The `meshes` digest changed (sets are inside the topology hash), the model did not.
    assert_ne!(rig.state().layer_hash("meshes").unwrap(), {
        let (r2, _) = Rig::with_block([3, 3, 3], &moves);
        r2.state().layer_hash("meshes").unwrap()
    });

    // A second call with the same name fails and leaves the state alone.
    let e = rig.json("mesh.flag_cells", json!({"name": "bad"})).unwrap_err();
    assert!(e.contains("already has a set named"), "{e}");
    assert_eq!(rig.state().hash(), hash1);
    // Invalid names.
    for bad in ["", "has space", "x".repeat(65).as_str(), "slash/"] {
        assert!(rig.json("mesh.flag_cells", json!({"name": bad})).is_err(), "{bad:?}");
    }
    assert_eq!(rig.state().hash(), hash1);

    // Undo and redo.
    rig.s.undo().unwrap();
    assert!(!rig.state().contains(set) && rig.state().hash() == hash0);
    rig.s.redo().unwrap();
    assert_eq!(rig.state().hash(), hash1);
    assert_eq!(members_of(&rig, mesh, set), want);

    // mesh.delete retires the set id with the mesh.
    rig.json("mesh.delete", json!({"meshes": [mesh.to_string()]})).unwrap();
    assert!(!rig.state().contains(set) && !rig.state().contains(mesh));
}

#[test]
fn cold_replay_from_the_root_does_not_diverge() {
    let moves = damaged_moves();
    let run = |rig: &mut Rig| {
        rig.apply(CreateBrick { size: [3.0; 3] }).unwrap();
        rig.apply(InsertBlock { n: [3, 3, 3], h: [1.0; 3], moves: moves.clone() }).unwrap();
        rig.json("mesh.flag_cells", json!({"name": "bad"})).unwrap();
        rig.json("mesh.flag_cells", json!({"name": "warned", "level": "warn", "include_invalid": false})).unwrap().0
    };
    let mut warm = Rig::new();
    let h = run(&mut warm);
    let mut cold = Rig::cold();
    let head = run(&mut cold);
    assert_eq!(h, head);
    let q = QuietControl::new();
    let replayed = cold.s.state_at_from_root(head, &q.control()).expect("no Diverged");
    assert_eq!(replayed.hash(), cold.state().hash());
    assert_eq!(replayed.hash(), warm.state().hash());
}

#[test]
fn empty_selection_warns_and_adds_nothing() {
    let (mut rig, _) = Rig::with_block([3, 3, 3], &[]);
    let hash0 = rig.state().hash();
    let sets = rig.state().count_of(EntityKind::MeshSet);
    let (h, d) = rig.json("mesh.flag_cells", json!({"name": "none"})).unwrap();
    assert!(d.outputs.get("set").is_none_or(Vec::is_empty), "{:?}", d.outputs);
    assert_eq!(d.diagnostics.len(), 1);
    assert_eq!(d.diagnostics[0].code, "diag.nothing_to_flag");
    assert!(d.diagnostics[0].message.contains("graded fail or invalid"), "wire name, not Debug: {}", d.diagnostics[0].message);
    assert!(d.created.is_empty());
    assert_eq!(rig.state().count_of(EntityKind::MeshSet), sets);
    // The op is recorded (a no-op node) but the state hash is unchanged.
    let _ = h;
    assert_eq!(rig.state().hash(), hash0);
    // The name stays free.
    assert!(rig.json("mesh.flag_cells", json!({"name": "none"})).is_ok());
}

#[test]
fn mesh_defaults_to_the_only_mesh_and_lists_candidates() {
    let mut rig = Rig::new();
    rig.apply(CreateBrick { size: [1.0; 3] }).unwrap();
    let e = rig.json("mesh.flag_cells", json!({"name": "a"})).unwrap_err();
    assert!(e.contains("no mesh"), "{e}");
    rig.apply(InsertPair { kind: "sheared".into() }).unwrap();
    rig.apply(InsertPair { kind: "stretched".into() }).unwrap();
    let e = rig.json("mesh.flag_cells", json!({"name": "a"})).unwrap_err();
    assert!(e.contains("Mesh#1") && e.contains("Mesh#2"), "{e}");
    assert!(rig.json("mesh.flag_cells", json!({"name": "a", "mesh": "Mesh#2", "metrics": ["nope"]})).is_err());
    assert!(rig.json("mesh.flag_cells", json!({"name": "a", "mesh": "Mesh#2", "extra": 1})).is_err());
    let (_, d) = rig.json("mesh.flag_cells", json!({"name": "a", "mesh": "Mesh#2", "level": "warn", "metrics": ["interpolation_weight"]})).unwrap();
    // Stretched pair: weight 1/3 is above the 0.2 warning limit: nothing flagged.
    assert_eq!(d.diagnostics[0].code, "diag.nothing_to_flag");
}

#[test]
fn thresholds_override_the_limits() {
    let m = pair_mesh("sheared", &mut SerialAlloc::new());
    let metrics = [Ms::from_name("non_orthogonality").unwrap()];
    let none = flagged_cells(&m, Profile::Generic, Some(&metrics), &Default::default(), Level::Fail, true).unwrap();
    assert!(none.is_empty(), "45 degrees is within the generic limit");
    let t = std::collections::BTreeMap::from([("non_orthogonality".to_string(), sm_diag::ThresholdOverride { warn: None, fail: Some(40.0) })]);
    let both = flagged_cells(&m, Profile::Generic, Some(&metrics), &t, Level::Fail, true).unwrap();
    assert_eq!(both.iter().map(|c| c.0).collect::<Vec<_>>(), [0, 1], "the shared face reaches both its cells");
    let bad = std::collections::BTreeMap::from([("volume".to_string(), sm_diag::ThresholdOverride { warn: None, fail: Some(1.0) })]);
    assert!(flagged_cells(&m, Profile::Generic, Some(&metrics), &bad, Level::Fail, true).is_err());
    // Through the op.
    let (mut rig, _) = Rig::with_pair("sheared");
    let (_, d) = rig.json("mesh.flag_cells", json!({"name": "hot", "metrics": ["non_orthogonality"], "thresholds": {"non_orthogonality": {"fail": 40}}})).unwrap();
    assert_eq!(d.outputs["set"].len(), 1);
}

/// Cells sharing a face with a member, by a direct loop over the faces.
fn neighbours(m: &ReferenceMesh, members: &BTreeSet<u32>) -> BTreeSet<u32> {
    let mut out = members.clone();
    for f in 0..m.face_count() {
        let fi = sm_mesh_api::FaceIx::new(f);
        if let Some(r) = m.face_right(fi) {
            let l = m.face_left(fi);
            if members.contains(&l.0) {
                out.insert(r.0);
            }
            if members.contains(&r.0) {
                out.insert(l.0);
            }
        }
    }
    out
}

#[test]
fn grow_adds_exactly_the_face_neighbours() {
    let moves = damaged_moves();
    let reference = block([3, 3, 3], [1.0; 3], &moves, &mut SerialAlloc::new());
    let base = expected(&reference, Profile::Generic, 2, true);
    let base_set: BTreeSet<u32> = base.iter().copied().collect();
    let ring1 = neighbours(&reference, &base_set);
    assert!(ring1.len() > base_set.len() && ring1.len() < 27, "{} -> {}", base_set.len(), ring1.len());
    let ring2 = neighbours(&reference, &ring1);
    let (mut rig, mesh) = Rig::with_block([3, 3, 3], &moves);
    for (grow, want) in [(0u32, &base_set), (1, &ring1), (2, &ring2)] {
        let (_, d) = rig.json("mesh.flag_cells", json!({"name": format!("g{grow}"), "grow": grow})).unwrap();
        let got: BTreeSet<u32> = members_of(&rig, mesh, d.outputs["set"][0]).into_iter().collect();
        assert_eq!(&got, want, "grow {grow}");
    }
    assert!(rig.json("mesh.flag_cells", json!({"name": "too_far", "grow": 6})).unwrap_err().contains("grow"));
    // Growing an empty selection stays empty.
    let (mut clean, _) = Rig::with_block([3, 3, 3], &[]);
    let (_, d) = clean.json("mesh.flag_cells", json!({"name": "none", "grow": 3})).unwrap();
    assert_eq!(d.diagnostics[0].code, "diag.nothing_to_flag");
}
