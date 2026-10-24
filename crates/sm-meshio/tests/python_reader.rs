//! The ~100-line Python reference reader against files written by the Rust writer.

mod common;

use common::py::{python, run_json, tmp};
use common::*;
use sm_meshio::SmshWriter;
use sm_mesh_api::*;

#[test]
fn python_reader_agrees_with_the_writer() {
    let Some(py) = python() else {
        eprintln!("python3 not available: skipping");
        return;
    };
    let script = concat!(env!("CARGO_MANIFEST_DIR"), "/readers/smsh.py");
    for (name, m) in corpus() {
        let path = tmp(&format!("{name}.smsh"));
        SmshWriter::new(&m).provenance("abc").write_file(&path).unwrap();
        let j = run_json(&py, script, &path);
        let mi = &j["mesh"];
        assert_eq!(mi["n_cells"], m.cell_count(), "{name}");
        assert_eq!(mi["n_faces"], m.face_count(), "{name}");
        assert_eq!(mi["n_vertices"], m.vertex_count(), "{name}");
        // Distinct vertices per cell, discovered by walking the CSR arrays in Python.
        let want: Vec<usize> = (0..m.cell_count())
            .map(|c| {
                let mut v: Vec<u32> = m.cell_faces(CellIx::new(c)).iter().flat_map(|sf| m.face_vertices(sf.face()).iter().map(|v| v.0)).collect();
                v.sort_unstable();
                v.dedup();
                v.len()
            })
            .collect();
        let got: Vec<usize> = j["cell_vertex_counts"].as_array().unwrap().iter().map(|x| x.as_u64().unwrap() as usize).collect();
        assert_eq!(got, want, "{name}");
        let ids: Vec<u64> = j["cell_ids"].as_array().unwrap().iter().map(|x| x.as_u64().unwrap()).collect();
        assert_eq!(ids, (0..m.cell_count()).map(|c| m.cell_id(CellIx::new(c)).to_raw()).collect::<Vec<_>>(), "{name}");
        let cls: Vec<u64> = j["cell_class"].as_array().unwrap().iter().map(|x| x.as_u64().unwrap()).collect();
        assert_eq!(cls, (0..m.cell_count()).map(|c| m.cell_class(CellIx::new(c)).entity.to_raw()).collect::<Vec<_>>(), "{name}");
        assert_eq!(j["x_last"].as_f64().unwrap().to_bits(), m.coords().x.last().unwrap().to_bits(), "{name}");
        for s in m.sets() {
            let e = &j["sets"][&s.name];
            assert_eq!(e[0], serde_json::json!(s.label), "{name}/{}", s.name);
            assert_eq!(e[2], s.members.len(), "{name}/{}", s.name);
        }
    }
}

#[test]
fn python_reader_rejects_a_corrupted_file() {
    let Some(py) = python() else { return };
    let script = concat!(env!("CARGO_MANIFEST_DIR"), "/readers/smsh.py");
    let (_, m) = corpus().into_iter().find(|(n, _)| n == "hex_block").unwrap();
    let mut b = SmshWriter::new(&m).to_bytes().unwrap();
    let n = b.len();
    b[n - 5] ^= 0x10;
    let path = tmp("corrupt.smsh");
    std::fs::write(&path, b).unwrap();
    let out = std::process::Command::new(py).arg("-I").arg(script).arg(&path).output().unwrap();
    assert!(!out.status.success());
}
