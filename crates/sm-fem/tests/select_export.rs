//! `ExportRequest::select` through `sm_fem::export`: three unit bricks in a row, meshed at 0.5
//! (8 cells each, a 6 x 2 x 2 lattice of cells), one block per brick.
#![allow(missing_docs)]

mod common;

use common::*;
use sm_exodus::read_exodus;
use sm_fem::{BlockAdd, ExportFormat, ExportRequest, FemError, SidesetAdd, export};
use sm_layer_api::ModelLayer;
use sm_meshio::select::{Dimension, Pick, Select};
use sm_meshio::{SmshFile, SmshMesh};

/// Rows of three bricks meshed at 0.5, blocks 1..3 and a sideset on the free `+x` face of the
/// last brick (`sideset_9`) and on the free `-x` face of the first (`sideset_8`).
fn rig3() -> Rig {
    let mut r = Rig::new();
    let d = r.apply(Row { n: 3 });
    let v = role(&d, "volume");
    r.apply(MeshIt { volumes: v.clone(), size: 0.5 });
    for (i, vol) in v.iter().enumerate() {
        r.apply(BlockAdd {
            block: i as u32 + 1,
            volumes: vec![*vol],
            faces: vec![],
            name: None,
        });
    }
    let (first, last) = {
        let model = r.state().layer::<ModelLayer>().expect("model");
        let free = |vol| {
            let others: Vec<_> =
                v.iter().filter(|o| **o != vol).flat_map(|o| model.topo().volume_faces(*o).to_vec()).collect();
            model
                .topo()
                .volume_faces(vol)
                .to_vec()
                .into_iter()
                .filter(|f| !others.contains(f))
                .collect::<Vec<_>>()
        };
        (free(v[0]), free(v[2]))
    };
    // Any face of the end bricks that no other brick shares works; the first listed is enough.
    r.apply(SidesetAdd { sideset: 8, faces: vec![first[0]], wrt: None, name: None });
    r.apply(SidesetAdd { sideset: 9, faces: vec![last[0]], wrt: None, name: None });
    r
}

fn sel_blocks(names: &[&str]) -> Select {
    Select {
        blocks: Pick::Names(names.iter().map(|s| (*s).to_owned()).collect()),
        ..Select::default()
    }
}

fn run(r: &Rig, fmt: ExportFormat, sel: Option<Select>) -> Result<(Vec<u8>, sm_fem::ExportReport), FemError> {
    let mut req = ExportRequest::new(fmt);
    req.select = sel;
    let mut out = Vec::new();
    let rep = export(r.state(), &req, &mut out)?;
    Ok((out, rep))
}

fn decode(b: &[u8]) -> SmshMesh {
    SmshFile::parse(b).expect("parse").decode().expect("decode")
}

#[test]
fn two_blocks_of_three_write_their_cells_and_exactly_their_nodes() {
    let r = rig3();
    let all = decode(&run(&r, ExportFormat::Smsh, None).expect("all").0);
    assert_eq!((all.cell_ids.len(), all.vertex_ids.len()), (24, 7 * 3 * 3));
    let (bytes, rep) = run(&r, ExportFormat::Smsh, Some(sel_blocks(&["block_1", "block_2"]))).expect("select");
    let d = decode(&bytes);
    // 8 + 8 cells; a 4 x 2 x 2 lattice of cells has 5 x 3 x 3 nodes.
    assert_eq!(d.cell_ids.len(), 16);
    assert_eq!(d.vertex_ids.len(), 45);
    let set = |n: &str| d.sets.iter().find(|s| s.name == n).map(|s| s.members.len());
    assert_eq!((set("block_1"), set("block_2"), set("block_3")), (Some(8), Some(8), None));
    // The sideset on the dropped end brick is gone, the other one stays (4 sides of the face).
    assert_eq!((set("sideset_8"), set("sideset_9")), (Some(4), None));
    assert!(rep.sets.iter().all(|s| s.name != "block_3" && s.name != "sideset_9"));
    assert!(rep.notes.is_empty(), "{:?}", rep.notes);

    let x = read_exodus(&run(&r, ExportFormat::Exodus, Some(sel_blocks(&["block_1", "block_2"]))).expect("exo").0)
        .expect("read");
    assert_eq!((x.num_elem, x.num_nodes, x.blocks.len()), (16, 45, 2));
    // Analytic: the kept nodes span x in [0, 2].
    let xmax = x.coords[0].iter().cloned().fold(f64::MIN, f64::max);
    assert!((xmax - 2.0).abs() < 1e-12, "{xmax}");

    let vtu = run(&r, ExportFormat::Vtu, Some(sel_blocks(&["block_1", "block_2"]))).expect("vtu").0;
    let text = String::from_utf8_lossy(&vtu);
    assert!(text.contains("NumberOfPoints=\"45\" NumberOfCells=\"16\""), "{}", &text[..300.min(text.len())]);
}

#[test]
fn absent_select_and_a_keep_everything_select_write_the_same_bytes() {
    let r = rig3();
    for fmt in [ExportFormat::Smsh, ExportFormat::Vtu, ExportFormat::Exodus] {
        let plain = run(&r, fmt, None).expect("plain").0;
        let all = run(&r, fmt, Some(Select::default())).expect("all").0;
        let dim = Select { dimension: Dimension::Three, ..Select::default() };
        let three = run(&r, fmt, Some(dim)).expect("dimension").0;
        assert_eq!(plain, all, "{}", fmt.name());
        assert_eq!(plain, three, "{}", fmt.name());
    }
}

#[test]
fn a_named_empty_sideset_warns_and_an_unknown_name_is_a_bad_request() {
    let r = rig3();
    let sel = Select {
        blocks: Pick::Names(vec!["block_1".into(), "block_2".into()]),
        sidesets: Pick::Names(vec!["sideset_8".into(), "sideset_9".into()]),
        ..Select::default()
    };
    let (_, rep) = run(&r, ExportFormat::Smsh, Some(sel)).expect("select");
    assert_eq!(rep.notes.len(), 1, "{:?}", rep.notes);
    assert!(rep.notes[0].starts_with("export.empty_selection: sideset `sideset_9`"), "{}", rep.notes[0]);

    let bad = Select { blocks: Pick::Names(vec!["block_7".into()]), ..Select::default() };
    let e = run(&r, ExportFormat::Smsh, Some(bad)).expect_err("unknown");
    assert_eq!(e.code(), "bad-request");
    let msg = e.to_string();
    assert!(msg.contains("block_7") && msg.contains("block_1, block_2, block_3"), "{msg}");
}

#[test]
fn openfoam_refuses_select_and_the_selection_is_deterministic() {
    let r = rig3();
    let req = ExportRequest::new(ExportFormat::OpenFoam).select(sel_blocks(&["block_1"]));
    let e = sm_fem::export_files(r.state(), &req).expect_err("openfoam");
    assert_eq!(e.code(), "unsupported");
    let run1 = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool");
        pool.install(|| {
            [ExportFormat::Smsh, ExportFormat::Vtu, ExportFormat::Exodus]
                .map(|f| run(&r, f, Some(sel_blocks(&["block_3", "block_1"]))).expect("select").0)
        })
    };
    let base = run1(1);
    assert_eq!(base, run1(1));
    assert_eq!(base, run1(4));
}

#[test]
fn export_files_writes_the_selection_too() {
    let r = rig3();
    let req = ExportRequest::new(ExportFormat::Vtu).select(sel_blocks(&["block_2"]));
    let b = sm_fem::export_files(r.state(), &req).expect("files");
    assert_eq!(b.files[0].name, "mesh.vtu");
    let (bytes, _) = run(&r, ExportFormat::Vtu, Some(sel_blocks(&["block_2"]))).expect("stream");
    assert_eq!(b.files[0].bytes, bytes);
    assert!(b.sets.iter().any(|s| s.name == "block_2") && b.sets.iter().all(|s| s.name != "block_1"));
}
