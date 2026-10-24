//! T8 acceptance bullets: blocks, sidesets, deck lowering, export.

mod common;

use std::collections::BTreeMap;

use common::*;
use sm_core::EntityKind;
use sm_fem::{
    BlockAdd, BlockElementType, ElementType, ExportFormat, ExportRequest, FemError, FemLayer,
    SetOrigin, SidesetAdd, export, lower,
};
use sm_layer_api::{MeshLayer, ModelLayer};
use sm_mesh_api::{SetMembers, Side};
use sm_meshio::{SmshFile, SmshMesh};
use sm_model_api::{BoundaryKind, Deck};

fn smsh(r: &Rig) -> (Vec<u8>, sm_fem::ExportReport) {
    let mut buf = Vec::new();
    let rep = export(r.state(), &ExportRequest::new(ExportFormat::Smsh), &mut buf).expect("smsh");
    (buf, rep)
}

fn decode(buf: &[u8]) -> SmshMesh {
    SmshFile::parse(buf)
        .expect("parse")
        .decode()
        .expect("decode")
}

fn block_add(block: u32, volumes: Vec<sm_core::EntityId>) -> BlockAdd {
    BlockAdd {
        block,
        volumes,
        faces: vec![],
        name: None,
    }
}

/// A 10-brick meshed with size 1 and put in block 1.
fn meshed_brick(n: f64) -> (Rig, Vec<sm_core::EntityId>, Vec<sm_core::EntityId>) {
    let mut r = Rig::new();
    let d = r.apply(Bricks {
        origins: vec![[0.0; 3]],
        size: [n; 3],
    });
    let (v, f) = (role(&d, "volume"), role(&d, "faces"));
    r.apply(MeshIt {
        volumes: v.clone(),
        size: 1.0,
    });
    (r, v, f)
}

// ---- Stub-meshed brick plus block 1 gives a .smsh with cell set block_1 of label 1 (1000 cells)

#[test]
fn smsh_has_cell_set_block_1_with_label_1_and_1000_cells() {
    let (mut r, v, _) = meshed_brick(10.0);
    r.apply(block_add(1, v));
    let (buf, rep) = smsh(&r);
    assert_eq!(rep.sets.len(), 1);
    assert_eq!(rep.sets[0].name, "block_1");
    assert_eq!(rep.sets[0].origin, SetOrigin::Block);
    let m = decode(&buf);
    assert_eq!(m.cell_ids.len(), 1000);
    assert_eq!(m.sets.len(), 1);
    let s = &m.sets[0];
    assert_eq!((s.name.as_str(), s.label), ("block_1", Some(1)));
    let SetMembers::Cells(cells) = &s.members else {
        panic!("not a cell set")
    };
    assert_eq!(cells.len(), 1000);
    assert!(cells.iter().enumerate().all(|(i, c)| c.0 as usize == i));
    // The set id is minted from a clone of the layer allocator: the state is untouched.
    let layer = r.state().layer::<MeshLayer>().expect("meshes");
    assert!(
        layer
            .get(layer.meshes().next().expect("one").0)
            .expect("entry")
            .mesh()
            .sets()
            .is_empty(),
        "lowering never mutates the stored mesh"
    );
}

/// Raw appended arrays of a VTU: `Section/Name` to `(type, payload)`.
fn raw_arrays(bytes: &[u8]) -> BTreeMap<String, (String, Vec<u8>)> {
    let at = bytes
        .windows(13)
        .position(|w| w == b"<AppendedData")
        .expect("appended");
    let head = std::str::from_utf8(&bytes[..at]).expect("utf8 head");
    let body = &bytes[at + bytes[at..].iter().position(|b| *b == b'_').expect("marker") + 1..];
    let attr = |line: &str, key: &str| -> String {
        let p = line.find(&format!("{key}=\"")).expect("attr") + key.len() + 2;
        line[p..p + line[p..].find('"').expect("quote")].to_string()
    };
    let mut section = String::new();
    let mut out = BTreeMap::new();
    for line in head.lines().map(str::trim) {
        if let Some(s) = ["Points", "Cells", "PointData", "CellData"]
            .iter()
            .find(|s| line.starts_with(&format!("<{s}")))
        {
            section = (*s).to_string();
        } else if line.starts_with("<DataArray") {
            let off: usize = attr(line, "offset").parse().expect("offset");
            let len = u64::from_le_bytes(body[off..off + 8].try_into().expect("8")) as usize;
            out.insert(
                format!("{section}/{}", attr(line, "Name")),
                (attr(line, "type"), body[off + 8..off + 8 + len].to_vec()),
            );
        }
    }
    out
}

#[test]
fn vtu_has_set_block_1_array_all_ones() {
    let (mut r, v, _) = meshed_brick(10.0);
    r.apply(block_add(1, v));
    let mut buf = Vec::new();
    export(r.state(), &ExportRequest::new(ExportFormat::Vtu), &mut buf).expect("vtu");
    let a = raw_arrays(&buf);
    let (ty, flags) = &a["CellData/set:block_1"];
    assert_eq!(ty, "UInt8");
    assert_eq!(flags.len(), 1000);
    assert!(flags.iter().all(|f| *f == 1));
}

// ---- Two volumes in two blocks give disjoint sets that cover all cells

#[test]
fn two_volumes_two_blocks_are_disjoint_and_cover_all_cells() {
    let mut r = Rig::new();
    let d = r.apply(Bricks {
        origins: vec![[0.0; 3], [10.0, 0.0, 0.0]],
        size: [4.0; 3],
    });
    let v = role(&d, "volume");
    r.apply(MeshIt {
        volumes: v.clone(),
        size: 1.0,
    });
    r.apply(block_add(1, vec![v[0]]));
    r.apply(block_add(2, vec![v[1]]));
    let (buf, _) = smsh(&r);
    let m = decode(&buf);
    assert_eq!(m.cell_ids.len(), 128);
    let mut all = Vec::new();
    for s in &m.sets {
        let SetMembers::Cells(c) = &s.members else {
            panic!("cell set")
        };
        assert_eq!(c.len(), 64, "{}", s.name);
        all.extend(c.iter().map(|c| c.0));
    }
    assert_eq!(
        m.sets.iter().map(|s| s.label).collect::<Vec<_>>(),
        vec![Some(1), Some(2)]
    );
    all.sort_unstable();
    assert_eq!(all, (0..128).collect::<Vec<u32>>(), "disjoint and covering");
}

// ---- A TET4 block on hexes gives an error

#[test]
fn tet4_block_on_hexes_is_an_error_and_hex8_is_fine() {
    let (mut r, v, _) = meshed_brick(3.0);
    r.apply(block_add(1, v));
    r.apply(BlockElementType {
        block: 1,
        element_type: ElementType::Hex8,
    });
    let (_, rep) = smsh(&r);
    assert_eq!(rep.sets[0].len, 27);
    r.apply(BlockElementType {
        block: 1,
        element_type: ElementType::Tet4,
    });
    let mut sink = Vec::new();
    let err = export(
        r.state(),
        &ExportRequest::new(ExportFormat::Smsh),
        &mut sink,
    )
    .expect_err("tet4 on hexes");
    match &err {
        FemError::ElementMismatch {
            block,
            declared,
            cell,
            ..
        } => {
            assert_eq!(
                (block.as_str(), *declared, *cell),
                ("block_1", ElementType::Tet4, 0)
            );
        }
        e => panic!("wrong error {e:?}"),
    }
    assert_eq!(err.code(), "element-mismatch");
    assert!(sink.is_empty(), "nothing is written before the check");
}

/// The shape check runs in parallel above 2^14 cells and reports the same first cell.
#[test]
fn element_check_is_deterministic_on_the_parallel_path() {
    let (mut r, v, _) = meshed_brick(26.0);
    r.apply(block_add(1, v));
    r.apply(BlockElementType {
        block: 1,
        element_type: ElementType::Wedge6,
    });
    let mut sink = Vec::new();
    for threads in [1, 4] {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .expect("pool");
        let err = pool
            .install(|| {
                export(
                    r.state(),
                    &ExportRequest::new(ExportFormat::Smsh),
                    &mut sink,
                )
            })
            .expect_err("wedge on hexes");
        assert!(
            matches!(err, FemError::ElementMismatch { cell: 0, .. }),
            "{err:?}"
        );
    }
}

// ---- Sideset on the +x face gives 100 sides, all on Left of their boundary face

#[test]
fn sideset_on_plus_x_face_has_100_left_sides() {
    let (mut r, v, f) = meshed_brick(10.0);
    r.apply(block_add(1, v));
    let plus_x = f[1];
    let d = r.apply(SidesetAdd {
        sideset: 4,
        faces: vec![plus_x],
        wrt: None,
        name: None,
    });
    assert_eq!(role(&d, "sideset").len(), 1);
    let st = r.state();
    let (model, fem, layer) = (
        st.layer::<ModelLayer>().expect("model"),
        st.layer::<FemLayer>().expect("fem"),
        st.layer::<MeshLayer>().expect("meshes"),
    );
    let (_, entry) = layer.meshes().next().expect("mesh");
    let low = lower(model, entry, fem, layer.alloc()).expect("lower");
    let m = (*low.mesh).as_read();
    let set = m.set_by_name("sideset_4").expect("sideset");
    assert_eq!(set.label, Some(4));
    let SetMembers::Sides(sides) = &set.members else {
        panic!("side set")
    };
    assert_eq!(sides.len(), 100);
    for s in sides {
        assert_eq!(s.side(), Side::Left);
        assert!(
            m.face_right(s.face()).is_none(),
            "a boundary face has one cell"
        );
        for vtx in m.face_vertices(s.face()) {
            assert!((m.point(vtx.0 as usize).x - 10.0).abs() <= 1e-12);
        }
    }
    // And the file round trip.
    let (buf, rep) = smsh(&r);
    let sets: Vec<_> = rep
        .sets
        .iter()
        .map(|s| (s.name.as_str(), s.origin, s.len))
        .collect();
    assert_eq!(
        sets,
        vec![
            ("block_1", SetOrigin::Block, 1000),
            ("sideset_4", SetOrigin::Sideset, 100)
        ]
    );
    let file = decode(&buf);
    assert!(matches!(&file.sets[1].members, SetMembers::Sides(s) if s.len() == 100));
}

// ---- Interior faces: both sides, or the side of `wrt`

#[test]
fn interior_face_sideset_takes_both_sides_or_the_side_of_wrt() {
    let mut r = Rig::new();
    let d = r.apply(Row { n: 2 });
    let v = role(&d, "volume");
    r.apply(MeshIt {
        volumes: v.clone(),
        size: 0.5,
    });
    let shared = {
        let model = r.state().layer::<ModelLayer>().expect("model");
        let a = model.topo().volume_faces(v[0]);
        let b = model.topo().volume_faces(v[1]);
        let both: Vec<_> = a.iter().filter(|f| b.contains(f)).copied().collect();
        assert_eq!(both.len(), 1, "one interface face");
        both[0]
    };
    let count = |r: &Rig, set: &str| -> (usize, Vec<sm_core::EntityId>) {
        let st = r.state();
        let (model, fem, layer) = (
            st.layer::<ModelLayer>().expect("model"),
            st.layer::<FemLayer>().expect("fem"),
            st.layer::<MeshLayer>().expect("meshes"),
        );
        let (_, e) = layer.meshes().next().expect("mesh");
        let low = lower(model, e, fem, layer.alloc()).expect("lower");
        let m = (*low.mesh).as_read();
        let s = m.set_by_name(set).expect("set");
        let SetMembers::Sides(sides) = &s.members else {
            panic!("sides")
        };
        let owners = sides
            .iter()
            .map(|sf| {
                let c = if sf.side() == Side::Left {
                    m.face_left(sf.face())
                } else {
                    m.face_right(sf.face()).expect("right")
                };
                m.cell_class(c).entity
            })
            .collect();
        (sides.len(), owners)
    };
    r.apply(SidesetAdd {
        sideset: 1,
        faces: vec![shared],
        wrt: None,
        name: None,
    });
    let (n, owners) = count(&r, "sideset_1");
    assert_eq!(n, 8, "2x2 mesh faces, both sides");
    assert_eq!(owners.iter().filter(|o| **o == v[0]).count(), 4);
    assert_eq!(owners.iter().filter(|o| **o == v[1]).count(), 4);
    for (i, w) in v.iter().enumerate() {
        r.apply(SidesetAdd {
            sideset: 1,
            faces: vec![shared],
            wrt: Some(*w),
            name: None,
        });
        let (n, owners) = count(&r, "sideset_1");
        assert_eq!(n, 4, "volume {i}");
        assert!(owners.iter().all(|o| o == w));
    }
}

// ---- mirror_chamber meshed gives a deck face set on the mirror plane, and DeckRecord names it

#[test]
fn mirror_chamber_deck_face_set_lies_on_the_mirror_plane() {
    let mut r = Rig::new();
    r.apply(MirrorChamber {});
    // The brick's volume was replaced by the webcut: mesh the live one.
    let v = r
        .state()
        .layer::<ModelLayer>()
        .expect("model")
        .topo()
        .cells(sm_topo_api::Dim::Volume)
        .to_vec();
    assert_eq!(v.len(), 1);
    r.apply(MeshIt {
        volumes: v,
        size: 0.25,
    });
    let (buf, rep) = smsh(&r);
    let name = {
        let model = r.state().layer::<ModelLayer>().expect("model");
        let b = model.model().boundaries().next().expect("deck boundary");
        assert!(matches!(b.kind, BoundaryKind::Deck(Deck::Mirror { .. })));
        b.name.clone()
    };
    let set_name = format!("deck_{name}_a");
    let s = rep
        .sets
        .iter()
        .find(|s| s.name == set_name)
        .expect("deck set");
    assert_eq!(
        (s.origin, s.len),
        (SetOrigin::DeckFace, 16),
        "4x4 cap faces"
    );
    let file = SmshFile::parse(&buf).expect("parse");
    let hdr = file.header();
    assert_eq!(hdr.decks.len(), 1);
    let rec = &hdr.decks[0];
    assert_eq!(rec.name, name);
    assert_eq!(rec.face_set.as_deref(), Some(set_name.as_str()));
    assert_eq!(rec.mate_set, None, "a mirror is its own mate");
    assert!(rec.improper);
    let m = file.decode().expect("decode");
    let set = m
        .sets
        .iter()
        .find(|s| s.name == set_name)
        .expect("set in file");
    let SetMembers::Faces(faces) = &set.members else {
        panic!("face set")
    };
    assert_eq!(faces.len(), 16);
    for f in faces {
        for &vtx in m.face_loop(f.0 as usize) {
            assert!(
                (m.z[vtx as usize] - 0.5).abs() <= 1e-12,
                "z = {}",
                m.z[vtx as usize]
            );
        }
    }
    // The model's other colors lower as well: the block "mat" and the face color "cap".
    let names: Vec<_> = rep.sets.iter().map(|s| s.name.as_str()).collect();
    assert!(
        names.contains(&"mat") && names.contains(&"cap"),
        "{names:?}"
    );
}

// ---- Export is byte-identical over repeated runs and thread counts; no fem layer works

#[test]
fn export_is_byte_identical_across_runs_and_thread_counts() {
    let (mut r, v, f) = meshed_brick(6.0);
    r.apply(block_add(1, v));
    r.apply(SidesetAdd {
        sideset: 2,
        faces: vec![f[0], f[5]],
        wrt: None,
        name: None,
    });
    for format in [ExportFormat::Smsh, ExportFormat::Vtu] {
        let run = |threads: usize| -> Vec<u8> {
            let pool = rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .expect("pool");
            let mut buf = Vec::new();
            pool.install(|| export(r.state(), &ExportRequest::new(format), &mut buf))
                .expect("export");
            buf
        };
        let base = run(1);
        assert_eq!(base, run(1), "{format:?} repeat");
        assert_eq!(base, run(4), "{format:?} 4 threads");
        assert_eq!(base, run(0), "{format:?} default pool");
    }
}

#[test]
fn export_without_a_fem_layer_writes_no_sets() {
    let mut r = Rig::without_fem();
    assert!(r.state().layer::<FemLayer>().is_none());
    let d = r.apply(Bricks {
        origins: vec![[0.0; 3]],
        size: [3.0; 3],
    });
    r.apply(MeshIt {
        volumes: role(&d, "volume"),
        size: 1.0,
    });
    let (buf, rep) = smsh(&r);
    assert!(rep.sets.is_empty());
    let m = decode(&buf);
    assert_eq!(m.cell_ids.len(), 27);
    assert!(m.sets.is_empty() && m.header.decks.is_empty());
    let mut vtu = Vec::new();
    export(r.state(), &ExportRequest::new(ExportFormat::Vtu), &mut vtu).expect("vtu");
    assert!(!String::from_utf8_lossy(&vtu[..vtu.len().min(4000)]).contains("set:"));
}

// ---- Mesh selection rules

#[test]
fn mesh_selection_errors_are_typed() {
    let mut sink = Vec::new();
    let req = ExportRequest::new(ExportFormat::Smsh);
    let mut r = Rig::new();
    let e = export(r.state(), &req, &mut sink).expect_err("empty");
    assert!(
        matches!(e, FemError::NoMesh) && e.code() == "no-mesh",
        "{e:?}"
    );

    let d = r.apply(Bricks {
        origins: vec![[0.0; 3], [5.0, 0.0, 0.0]],
        size: [2.0; 3],
    });
    let v = role(&d, "volume");
    let m1 = role(
        &r.apply(MeshIt {
            volumes: vec![v[0]],
            size: 1.0,
        }),
        "mesh",
    )[0];
    let m2 = role(
        &r.apply(MeshIt {
            volumes: vec![v[1]],
            size: 1.0,
        }),
        "mesh",
    )[0];
    let e = export(r.state(), &req, &mut sink).expect_err("two meshes");
    assert!(
        matches!(e, FemError::AmbiguousMesh(2)) && e.code() == "ambiguous-mesh",
        "{e:?}"
    );
    for id in [m1, m2] {
        let rep = export(
            r.state(),
            &ExportRequest {
                mesh: Some(id),
                ..req.clone()
            },
            &mut sink,
        )
        .expect("named mesh");
        assert_eq!(rep.mesh, id);
    }
    let bogus = sm_core::EntityId::new(EntityKind::Mesh, 99).expect("id");
    let e = export(
        r.state(),
        &ExportRequest {
            mesh: Some(bogus),
            ..req.clone()
        },
        &mut sink,
    )
    .expect_err("unknown");
    assert!(matches!(e, FemError::UnknownMesh(_)), "{e:?}");

    // Deleting a volume marks its mesh stale: refused with a typed error.
    r.apply(KillVolume { volume: v[0] });
    let e = export(
        r.state(),
        &ExportRequest {
            mesh: Some(m1),
            ..req.clone()
        },
        &mut sink,
    )
    .expect_err("stale");
    assert!(
        matches!(e, FemError::StaleMesh(id) if id == m1) && e.code() == "stale-mesh",
        "{e:?}"
    );
    export(
        r.state(),
        &ExportRequest {
            mesh: Some(m2),
            ..req
        },
        &mut sink,
    )
    .expect("other mesh");
}

#[test]
fn nothing_to_lower_shares_the_stored_mesh() {
    let (r, _, _) = meshed_brick(2.0);
    let st = r.state();
    let (model, layer) = (
        st.layer::<ModelLayer>().expect("model"),
        st.layer::<MeshLayer>().expect("meshes"),
    );
    let (_, entry) = layer.meshes().next().expect("mesh");
    let low = lower(model, entry, &FemLayer::default(), layer.alloc()).expect("lower");
    assert!(std::sync::Arc::ptr_eq(&low.mesh, &entry.mesh));
    assert!(low.sets.is_empty() && low.decks.is_empty());
}

// ---- Review fixes: succession, fem-less model colors, pair decks, round trips, clashes

fn mirror_meshed(r: &mut Rig) {
    r.apply(MirrorChamber {});
    let v = r
        .state()
        .layer::<ModelLayer>()
        .expect("model")
        .topo()
        .cells(sm_topo_api::Dim::Volume)
        .to_vec();
    r.apply(MeshIt {
        volumes: v,
        size: 0.5,
    });
}

#[test]
fn sideset_wrt_follows_a_face_and_volume_split() {
    let mut r = Rig::new();
    let d = r.apply(Bricks {
        origins: vec![[0.0; 3]],
        size: [1.0; 3],
    });
    let (v, f) = (role(&d, "volume"), role(&d, "faces"));
    r.apply(SidesetAdd {
        sideset: 1,
        faces: vec![f[1]],
        wrt: Some(v[0]),
        name: None,
    });
    r.apply(Webcut {
        volume: v[0],
        faces: f.clone(),
    });
    {
        let st = r.state();
        let (model, fem) = (
            st.layer::<ModelLayer>().expect("model"),
            st.layer::<FemLayer>().expect("fem"),
        );
        let topo = model.topo();
        let colored: Vec<_> = model.model().face_coloring().map(|(f, _)| f).collect();
        assert_eq!(colored.len(), 2, "both halves of the +x face are colored");
        let prefs: Vec<_> = fem.side_prefs().collect();
        assert!(
            prefs
                .iter()
                .all(|(f, w)| colored.contains(f) && topo.is_alive(*w))
        );
        for face in colored {
            let w = fem.side_wrt(face);
            assert!(!w.is_empty(), "{face} lost its wrt");
            assert!(
                w.iter().any(|vol| topo.volume_faces(*vol).contains(&face)),
                "{face} borders none of {w:?}"
            );
        }
    }
    let vols = r
        .state()
        .layer::<ModelLayer>()
        .expect("model")
        .topo()
        .cells(sm_topo_api::Dim::Volume)
        .to_vec();
    assert_eq!(vols.len(), 2);
    r.apply(MeshIt {
        volumes: vols,
        size: 0.5,
    });
    let (buf, _) = smsh(&r);
    let m = decode(&buf);
    let set = m.sets.iter().find(|s| s.name == "sideset_1").expect("set");
    let SetMembers::Sides(sides) = &set.members else {
        panic!("sides")
    };
    assert_eq!(sides.len(), 4, "2x2 faces on x = 1, one side each");
    assert!(sides.iter().all(|s| s.side() == Side::Left));
}

#[test]
fn fem_less_export_still_lowers_the_models_colors_and_decks() {
    // Blocks, sidesets and decks are model data: the `fem` layer only adds declared element
    // types and `wrt`. So a state without it exports exactly the colors the model holds.
    let mut r = Rig::without_fem();
    mirror_meshed(&mut r);
    let (buf, rep) = smsh(&r);
    let names: Vec<_> = rep.sets.iter().map(|s| s.name.as_str()).collect();
    assert!(
        names.contains(&"mat") && names.contains(&"cap"),
        "{names:?}"
    );
    assert!(rep.sets.iter().any(|s| s.origin == SetOrigin::DeckFace));
    assert_eq!(decode(&buf).header.decks.len(), 1);
}

#[test]
fn translation_pair_deck_has_two_face_sets_and_a_mate() {
    let mut r = Rig::new();
    let d = r.apply(Bricks {
        origins: vec![[0.0; 3], [3.0, 0.0, 0.0]],
        size: [1.0; 3],
    });
    let (v, f) = (role(&d, "volume"), role(&d, "faces"));
    r.apply(PairDeck {
        a: f[1],
        b: f[6],
        vector: [2.0, 0.0, 0.0],
    });
    r.apply(MeshIt {
        volumes: v,
        size: 0.5,
    });
    let (buf, rep) = smsh(&r);
    for n in ["deck_pair_a", "deck_pair_b"] {
        let s = rep.sets.iter().find(|s| s.name == n).expect(n);
        assert_eq!((s.origin, s.len), (SetOrigin::DeckFace, 4));
    }
    let file = SmshFile::parse(&buf).expect("parse");
    let rec = &file.header().decks[0];
    assert_eq!(rec.name, "pair");
    assert_eq!(rec.face_set.as_deref(), Some("deck_pair_a"));
    assert_eq!(rec.mate_set.as_deref(), Some("deck_pair_b"));
    assert!(!rec.improper);
    let m = file.decode().expect("decode");
    for (n, x) in [("deck_pair_a", 1.0), ("deck_pair_b", 3.0)] {
        let set = m.sets.iter().find(|s| s.name == n).expect(n);
        let SetMembers::Faces(faces) = &set.members else {
            panic!("faces")
        };
        for fc in faces {
            for &vtx in m.face_loop(fc.0 as usize) {
                assert!((m.x[vtx as usize] - x).abs() <= 1e-12, "{n}");
            }
        }
    }
}

#[test]
fn sideset_round_trip_decodes_left_sides_on_the_face() {
    let (mut r, _, f) = meshed_brick(10.0);
    r.apply(SidesetAdd {
        sideset: 1,
        faces: vec![f[1]],
        wrt: None,
        name: None,
    });
    let (buf, _) = smsh(&r);
    let m = decode(&buf);
    let set = m.sets.iter().find(|s| s.name == "sideset_1").expect("set");
    assert_eq!(set.label, Some(1));
    let SetMembers::Sides(sides) = &set.members else {
        panic!("sides")
    };
    assert_eq!(sides.len(), 100);
    for s in sides {
        assert_eq!(s.side(), Side::Left);
        for &vtx in m.face_loop(s.face().0 as usize) {
            assert!((m.x[vtx as usize] - 10.0).abs() <= 1e-12);
        }
    }
}

#[test]
fn vtu_carries_cell_sets_only_not_side_or_face_sets() {
    // sm-meshio writes cell and vertex sets as indicators; side and face sets exist only in
    // `.smsh`. Pin that the lowering does not leak them into the VTU. (There is no VTU reader in
    // sm-meshio, so this parses the appended arrays by hand.)
    let mut r = Rig::new();
    mirror_meshed(&mut r);
    let mut buf = Vec::new();
    let rep = export(r.state(), &ExportRequest::new(ExportFormat::Vtu), &mut buf).expect("vtu");
    assert!(rep.sets.iter().any(|s| s.origin == SetOrigin::Sideset));
    assert!(rep.sets.iter().any(|s| s.origin == SetOrigin::DeckFace));
    let a = raw_arrays(&buf);
    let keys: Vec<_> = a.keys().map(String::as_str).collect();
    let (ty, flags) = &a["CellData/set:mat"];
    assert_eq!(ty, "UInt8");
    assert!(flags.iter().all(|f| *f == 1) && flags.len() == 4);
    assert!(keys.contains(&"CellData/region"), "{keys:?}");
    assert!(
        keys.iter()
            .all(|k| !k.contains("set:cap") && !k.contains("deck_")),
        "{keys:?}"
    );
}

#[test]
fn a_set_name_clash_is_a_mesh_error() {
    let mut r = Rig::new();
    mirror_meshed(&mut r);
    let deck = {
        let model = r.state().layer::<ModelLayer>().expect("model");
        model
            .model()
            .boundaries()
            .next()
            .expect("deck")
            .name
            .clone()
    };
    let v = r
        .state()
        .layer::<ModelLayer>()
        .expect("model")
        .topo()
        .cells(sm_topo_api::Dim::Volume)
        .to_vec();
    r.apply(BlockAdd {
        block: 2,
        volumes: v,
        faces: vec![],
        name: Some(format!("deck_{deck}_a")),
    });
    let mut sink = Vec::new();
    let e = export(r.state(), &ExportRequest::new(ExportFormat::Smsh), &mut sink)
        .expect_err("clash");
    assert!(matches!(e, FemError::Mesh(_)) && e.code() == "mesh", "{e:?}");
}
