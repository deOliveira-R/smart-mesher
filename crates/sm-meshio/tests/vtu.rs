//! VTU export: decoded with an independent Python decoder (and VTK itself when available).

mod common;

use common::py::{python, run_json, tmp};
use common::*;
use serde_json::Value;
use sm_core::EntityKind;
use sm_meshio::{VtuEncoding, VtuOptions, vtu_bytes};
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::*;

fn ints(v: &Value) -> Vec<i64> {
    v["values"].as_array().unwrap().iter().map(|x| x.as_i64().unwrap()).collect()
}

fn floats(v: &Value) -> Vec<f64> {
    v["values"].as_array().unwrap().iter().map(|x| x.as_f64().unwrap()).collect()
}

fn vtk_type(s: CellShape) -> i64 {
    match s {
        CellShape::Tet => 10,
        CellShape::Hex => 12,
        CellShape::Wedge => 13,
        CellShape::Pyramid => 14,
        CellShape::Polyhedron => 42,
        CellShape::Tri => 5,
        CellShape::Quad => 9,
        CellShape::Polygon => 7,
        _ => unreachable!(),
    }
}

/// Expected connectivity / faces streams, written independently of the exporter.
fn expected(m: &dyn MeshRead) -> (Vec<i64>, Vec<i64>, Vec<i64>, Vec<i64>) {
    let (mut conn, mut off, mut faces, mut foff) = (vec![], vec![], vec![], vec![]);
    for c in 0..m.cell_count() {
        let ci = CellIx::new(c);
        if m.cell_shape(ci) == CellShape::Polyhedron {
            let mut vs: Vec<i64> = m.cell_faces(ci).iter().flat_map(|sf| m.face_vertices(sf.face()).iter().map(|v| i64::from(v.0))).collect();
            vs.sort_unstable();
            vs.dedup();
            conn.extend(&vs);
            faces.push(m.cell_faces(ci).len() as i64);
            for sf in m.cell_faces(ci) {
                let mut l: Vec<i64> = m.face_vertices(sf.face()).iter().map(|v| i64::from(v.0)).collect();
                if sf.side() == Side::Right {
                    l.reverse();
                }
                faces.push(l.len() as i64);
                faces.extend(l);
            }
            foff.push(faces.len() as i64);
        } else {
            let k: Vec<i64> = m.cell_corners(ci).unwrap().iter().map(|v| i64::from(v.0)).collect();
            conn.extend(k);
            foff.push(-1);
        }
        off.push(conn.len() as i64);
    }
    (conn, off, faces, foff)
}

fn check_decoded(name: &str, m: &dyn MeshRead, j: &Value) {
    let s = &j["sections"];
    assert_eq!(j["np"], m.coords().len(), "{name}");
    assert_eq!(j["nc"], m.cell_count(), "{name}");
    let co = m.coords();
    let pts = floats(&s["Points"]["Points"]);
    for i in 0..co.len() {
        assert_eq!(pts[3 * i].to_bits(), co.x[i].to_bits(), "{name}");
        assert_eq!(pts[3 * i + 1].to_bits(), co.y[i].to_bits());
        assert_eq!(pts[3 * i + 2].to_bits(), co.z[i].to_bits());
    }
    let cells = &s["Cells"];
    let types: Vec<i64> = (0..m.cell_count()).map(|c| vtk_type(m.cell_shape(CellIx::new(c)))).collect();
    assert_eq!(ints(&cells["types"]), types, "{name}");
    let (conn, off, faces, foff) = expected(m);
    assert_eq!(ints(&cells["connectivity"]), conn, "{name}");
    assert_eq!(ints(&cells["offsets"]), off, "{name}");
    if types.contains(&42) {
        assert_eq!(ints(&cells["faces"]), faces, "{name}");
        // -1 is stored as the all-ones pattern of the index width; compare as signed.
        assert_eq!(ints(&cells["faceoffsets"]), foff, "{name}");
    } else {
        assert!(cells.get("faces").is_none(), "{name}: no faces stream without polyhedra");
    }
    let cd = &s["CellData"];
    let cls: Vec<i64> = (0..m.cell_count()).map(|c| m.cell_class(CellIx::new(c)).entity.serial() as i64).collect();
    assert_eq!(ints(&cd["class_entity"]), cls, "{name}");
    let pd = &s["PointData"];
    assert_eq!(pd["class_dim"]["values"].as_array().unwrap().len(), co.len());
}

#[test]
fn vtu_decodes_to_the_mesh_in_both_encodings() {
    let Some(py) = python() else {
        eprintln!("python3 not available: skipping");
        return;
    };
    let script = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/vtu_check.py");
    for (name, m) in corpus() {
        for (enc, ext) in [(VtuEncoding::AppendedRaw, "raw"), (VtuEncoding::AppendedBase64, "b64")] {
            let path = tmp(&format!("{name}.{ext}.vtu"));
            std::fs::write(&path, vtu_bytes(&m, &VtuOptions::with_encoding(enc)).unwrap()).unwrap();
            let j = run_json(&py, script, &path);
            check_decoded(&format!("{name}/{ext}"), &m, &j);
            if let Some(v) = j["vtk"].as_object() {
                // VTK's own reader sees the same cells, with positive-measure, outward-faced cells.
                assert_eq!(v["n_cells"], m.cell_count());
                let want: Vec<i64> = (0..m.cell_count()).map(|c| vtk_type(m.cell_shape(CellIx::new(c)))).collect();
                let got: Vec<i64> = v["types"].as_array().unwrap().iter().map(|x| x.as_i64().unwrap()).collect();
                assert_eq!(got, want, "{name}");
                for (c, ok) in v["outward"].as_array().unwrap().iter().enumerate() {
                    if m.dim() == MeshDim::Three {
                        assert_eq!(ok, &Value::Bool(true), "{name}/{ext}: cell {c} has an inward face in VTK's convention");
                    }
                }
                for c in 0..m.cell_count() {
                    let ci = CellIx::new(c);
                    let (measure, key) = if m.dim() == MeshDim::Three { (cell_volume(&m, ci), "volume") } else { (cell_volume(&m, ci), "area") };
                    let got = v[key][c].as_f64().unwrap();
                    // Planar-faced cells: VTK and the fan rule agree (to rounding).
                    if m.cell_shape(ci) != CellShape::Polyhedron || name != "dual_patch" {
                        assert!((got - measure).abs() <= 1e-9 * measure.abs().max(1.0), "{name}/{ext}: cell {c}: vtk {got} vs {measure}");
                    }
                }
            }
        }
    }
}

#[test]
fn cell_data_carries_regions_sets_tags_and_lineage() {
    let Some(py) = python() else { return };
    let script = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/vtu_check.py");
    let (_, m) = corpus().into_iter().find(|(n, _)| n == "decorated_block").unwrap();
    let path = tmp("decorated.vtu");
    std::fs::write(&path, vtu_bytes(&m, &VtuOptions::default()).unwrap()).unwrap();
    let j = run_json(&py, script, &path);
    let cd = &j["sections"]["CellData"];
    // fuel (label 7) is cells 0..6, clad (label 9) cells 6..12.
    let region = ints(&cd["region"]);
    assert_eq!(region, (0..12).map(|c| if c < 6 { 7 } else { 9 }).collect::<Vec<i64>>());
    assert_eq!(ints(&cd["set:fuel"]), (0..12).map(|c| i64::from(c < 6)).collect::<Vec<_>>());
    assert_eq!(ints(&cd["set:clad"]), (0..12).map(|c| i64::from(c >= 6)).collect::<Vec<_>>());
    assert_eq!(ints(&cd["tag:mat"]), (0..12).map(|c| c % 3).collect::<Vec<i64>>());
    assert!(cd.get("active").is_none(), "all cells active: no array");
    assert_eq!(ints(&cd["class_entity"]), vec![1; 12], "all cells on TopoVolume#1");
    let pd = &j["sections"]["PointData"];
    assert_eq!(ints(&pd["set:corner"]).iter().sum::<i64>(), 2);
    assert_eq!(ints(&pd["set:pts"]).iter().sum::<i64>(), 2);
    assert!((floats(&pd["tag:temp"])[4] - 301.0).abs() < 1e-15);
    // Vertex classification dimension: 12*... corners are 0, edges 1, faces 2, interior 3.
    let dims = ints(&pd["class_dim"]);
    assert!(dims.contains(&0) && dims.contains(&1) && dims.contains(&2) && dims.contains(&3));
    // Lineage: inactive parent shows up as an `active` array.
    let (_, l) = corpus().into_iter().find(|(n, _)| n == "lineage_pair").unwrap();
    std::fs::write(&path, vtu_bytes(&l, &VtuOptions::default()).unwrap()).unwrap();
    let j = run_json(&py, script, &path);
    assert_eq!(ints(&j["sections"]["CellData"]["active"]), vec![0, 1]);
    // Options switch the optional arrays off.
    let mut o = VtuOptions::default();
    o.set_indicators = false;
    o.tags = false;
    let (_, m) = corpus().into_iter().find(|(n, _)| n == "decorated_block").unwrap();
    std::fs::write(&path, vtu_bytes(&m, &o).unwrap()).unwrap();
    let j = run_json(&py, script, &path);
    let cd = j["sections"]["CellData"].as_object().unwrap();
    assert!(cd.contains_key("region") && !cd.contains_key("set:fuel") && !cd.contains_key("tag:mat"));
}

#[test]
fn names_are_xml_escaped() {
    let mut alloc = SerialAlloc::new();
    let mut m = hex_block([1, 1, 1], [1.0; 3], &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, "a<b&\"c\">", SetMembers::Cells(vec![CellIx(0)]), &mut alloc).unwrap();
    let b = vtu_bytes(&m, &VtuOptions::default()).unwrap();
    let at = b.windows(13).position(|w| w == b"<AppendedData").unwrap();
    let head = String::from_utf8_lossy(&b[..at]).into_owned();
    assert!(head.contains("set:a&lt;b&amp;&quot;c&quot;&gt;"), "{head}");
    let Some(py) = python() else { return };
    let path = tmp("escaped.vtu");
    std::fs::write(&path, &b).unwrap();
    let j = run_json(&py, concat!(env!("CARGO_MANIFEST_DIR"), "/tests/vtu_check.py"), &path);
    assert!(j["sections"]["CellData"].get("set:a<b&\"c\">").is_some());
}

#[test]
fn vtu_is_deterministic_across_runs_and_thread_counts() {
    use sm_testkit::determinism;
    for (name, m) in corpus() {
        for enc in [VtuEncoding::AppendedRaw, VtuEncoding::AppendedBase64] {
            let hash = |threads: usize| -> u64 {
                let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
                let b = pool.install(|| vtu_bytes(&m, &VtuOptions::with_encoding(enc)).unwrap());
                b.iter().fold(0xcbf2_9ce4_8422_2325u64, |h, x| (h ^ u64::from(*x)).wrapping_mul(0x0100_0000_01b3))
            };
            determinism::check(2, 4, hash).unwrap_or_else(|e| panic!("{name}: {e}"));
        }
    }
    let m = hex_block([40, 30, 20], [0.1, 0.2, 0.3], &mut SerialAlloc::new()).unwrap();
    let a = rayon::ThreadPoolBuilder::new().num_threads(1).build().unwrap().install(|| vtu_bytes(&m, &VtuOptions::default()).unwrap());
    let b = rayon::ThreadPoolBuilder::new().num_threads(6).build().unwrap().install(|| vtu_bytes(&m, &VtuOptions::default()).unwrap());
    assert!(a == b);
    let _ = EntityKind::Mesh;
}
