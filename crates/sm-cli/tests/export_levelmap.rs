//! W4-B1-TIDY (EXPORT-SELECT): the `.smsh` level-map round trip of a refined mesh (`mesh.refine`,
//! then `session.export`, then read back with its map), and the loss note
//! `export.select_levelmap_dropped` when a selection cuts the mesh, through both `.smsh` paths
//! (`sm_fem::export` and the region-split writer `write_smsh_split`). Everything is exported
//! inline: nothing is written to disk.
#![allow(missing_docs)]

use serde_json::{Value, json};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::MeshLayer;
use sm_mesh_api::{CellIx, MeshCatalog};
use sm_meshio::{SmshFile, SmshMesh};
use sm_protocol::{Endpoint, ExportFormat, ExportResult, ExportSource, Request};
use sm_server::{Server, standard_with};

const TIME: &str = "2026-11-13T00:00:00Z";

/// Two 2 x 2 x 2 bricks of 8 hexes each, blocks `fuel` and `water`, then one `mesh.refine` level:
/// Mesh#1 (16 cells) is the coarse mesh and Mesh#2 (128 cells) the fine one.
const SCRIPT: &str = r#"
let a = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [1.0, 1.0, 1.0])
let b = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [3.0, 1.0, 1.0])
model.imprint(bodies: a.body | b.body)
model.merge(entities: a.body | b.body)
fem.block_add(block: 1, volumes: a.volume, name: "fuel")
fem.block_add(block: 2, volumes: b.volume, name: "water")
mesh.size(targets: a.volume | b.volume, size: 1.0)
mesh.scheme(targets: a.volume | b.volume, scheme: "map")
let m = mesh.run(targets: a.volume | b.volume)
mesh.refine(mesh: m.mesh[0])
"#;

fn mesh(n: u64) -> EntityId {
    EntityId::new(EntityKind::Mesh, n).expect("id")
}

fn server() -> Server {
    let mut s = Server::new(standard_with(Some("tester".into()), Some(TIME.into()))).expect("server");
    s.call(Request::ScriptRun { text: SCRIPT.into(), path: None, includes: Default::default() })
        .unwrap_or_else(|e| panic!("fixture script: {e} {}", e.data));
    s
}

/// Standard base64 (RFC 4648) decoder for the inline bytes.
fn unb64(s: &str) -> Vec<u8> {
    let val = |c: u8| match c {
        b'A'..=b'Z' => c - b'A',
        b'a'..=b'z' => c - b'a' + 26,
        b'0'..=b'9' => c - b'0' + 52,
        b'+' => 62,
        b'/' => 63,
        _ => panic!("bad base64 byte {c}"),
    };
    let mut out = Vec::new();
    let mut acc = 0u32;
    let mut bits = 0;
    for c in s.bytes().filter(|c| *c != b'=') {
        acc = (acc << 6) | u32::from(val(c));
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            out.push((acc >> bits) as u8);
            acc &= (1 << bits) - 1;
        }
    }
    out
}

/// Export `Mesh#n` as `.smsh` with `options`; the decoded file and the notes (`code`, `count`).
fn export(s: &mut Server, n: u64, options: Option<Value>) -> (SmshMesh, Vec<(String, u64)>) {
    let r = Request::Export {
        path: None,
        format: Some(ExportFormat::Smsh),
        mesh: Some(mesh(n)),
        provenance: false,
        inline: true,
        options,
        split_small_determinant: None,
        empty_patches: vec![],
        wedge_patches: vec![],
        split_warped_faces: false,
        source: ExportSource::Mesh,
    };
    let res: ExportResult = serde_json::from_value(s.call(r).unwrap_or_else(|e| panic!("export: {e} {}", e.data))).expect("ExportResult");
    let bytes = unb64(res.base64.as_deref().expect("inline"));
    let d = SmshFile::parse(&bytes).expect("parse").decode().expect("decode");
    (d, res.notes.iter().map(|n| (n.code.clone(), n.count)).collect())
}

fn region_sets(m: &SmshMesh) -> Vec<String> {
    m.sets.iter().filter_map(|s| s.name.strip_prefix("region/")).map(str::to_owned).collect()
}

/// `mesh.refine` then `.smsh`: the map reads back equal to the layer's own and is the analytic one
/// (8 fine cells per coarse cell, every coarse cell hit); it is the same from either mesh.
#[test]
fn a_refined_mesh_exports_its_level_map_and_it_reads_back() {
    let mut s = server();
    let (coarse, fine) = (mesh(1), mesh(2));
    let (live, n_coarse_faces) = {
        let layer = s.workspace().state().layer::<MeshLayer>().expect("layer");
        assert_eq!(layer.maps_from(coarse).len(), 0);
        assert_eq!(layer.maps_from(fine).len(), 1, "refine links the level to its parent");
        let map = &layer.maps_from(fine)[0].1;
        let live: Vec<u32> = (0..128).map(|i| map.coarse_of(CellIx::new(i)).get() as u32).collect();
        (live, layer.mesh(coarse).expect("coarse").face_count())
    };
    for side in [fine, coarse] {
        let (d, notes) = export(&mut s, side.serial(), None);
        assert!(notes.is_empty(), "{notes:?}");
        assert_eq!(d.level_maps.len(), 1, "the map is in the file of either side");
        let m = &d.level_maps[0];
        assert_eq!((m.fine, m.coarse), (fine, coarse));
        assert_eq!(m.coarse_of.len(), 128);
        let mut kids = [0usize; 16];
        for &c in &m.coarse_of {
            kids[c as usize] += 1;
        }
        assert_eq!(kids, [8; 16], "hex 1 to 8");
        assert_eq!(m.coarse_of, live, "the stored map is the layer's own");
        let fm = m.face_map.as_ref().expect("refine records the face map");
        // The fine grid is 8 x 4 x 4 cells: 9*16 + 8*5*4 + 8*4*5 = 464 faces; the coarse 4 x 2 x 2
        // grid has 5*4 + 4*3*2 + 4*2*3 = 68, each carried by 4 fine faces.
        assert_eq!((fm.len(), n_coarse_faces), (464, 68));
        // Every coarse face is carried by 4 fine faces.
        assert_eq!(fm.iter().filter(|&&f| f != u32::MAX).count(), 4 * n_coarse_faces);
    }
}

/// A selection that cuts the mesh drops the level maps with the note (`x1`); one that keeps every
/// cell keeps them. Both with and without the region split (the split goes through
/// `write_smsh_split`, the plain export through `sm_fem::export`).
#[test]
fn a_cutting_selection_drops_the_level_map_with_a_note_on_both_smsh_paths() {
    let mut s = server();
    let cut = json!({"blocks": ["fuel"]});
    let keep = json!({"blocks": "all"});
    for split in [false, true] {
        let with = |sel: &Value| {
            let mut o = json!({ "select": sel });
            if split {
                o["regions"] = json!({"split": ["ring"]});
            }
            Some(o)
        };
        let (d, notes) = export(&mut s, 2, with(&cut));
        let dropped: Vec<_> = notes.iter().filter(|(c, _)| c == "export.select_levelmap_dropped").collect();
        assert_eq!(dropped, [&("export.select_levelmap_dropped".to_owned(), 1)], "split {split}: {notes:?}");
        assert!(d.level_maps.is_empty(), "split {split}: a cut mesh is a new mesh, the maps name the whole one");
        assert_eq!(d.cell_ids.len(), 64, "split {split}: the fuel brick refined is 8 x 8 cells");
        if split {
            assert_eq!(region_sets(&d), ["fuel"], "the region sets of the kept cells stay, the others go");
        }

        let (d, notes) = export(&mut s, 2, with(&keep));
        assert!(notes.iter().all(|(c, _)| c != "export.select_levelmap_dropped"), "split {split}: {notes:?}");
        assert_eq!(d.level_maps.len(), 1, "split {split}: nothing cut, the map stays");
        assert_eq!(d.cell_ids.len(), 128);
        if split {
            assert_eq!(region_sets(&d), ["fuel", "water"]);
        }
    }
    // No selection with the split keeps the map too.
    let (d, _) = export(&mut s, 2, Some(json!({"regions": {"split": ["ring"]}})));
    assert_eq!(d.level_maps.len(), 1);
}
