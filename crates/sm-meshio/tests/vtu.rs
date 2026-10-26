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
            // An explicitly chosen interpreter is a request for the VTK check: fail loudly.
            if std::env::var_os("SM_PYTHON").is_some() {
                assert!(j["vtk"].is_object(), "SM_PYTHON is set but vtk cannot be imported: {}", j["vtk_import_error"]);
            }
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

// ---- Hand-written fixed oracle -------------------------------------------------------------

use sm_core::Point3;
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use std::collections::BTreeMap;

/// One cell with the given points, classified on TopoVolume#1 / TopoFace#k / TopoVertex#k.
fn single_cell(points: &[[f64; 3]], add: impl FnOnce(&mut ReferenceBuilder, &[VertexIx]) -> CellIx) -> ReferenceMesh {
    let mut b = ReferenceBuilder::new();
    let vs: Vec<VertexIx> = points
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let v = b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap();
            b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, 1 + i as u64)).unwrap();
            v
        })
        .collect();
    let c = add(&mut b, &vs);
    b.set_class(MeshEntityRef::Cell(c), on(EntityKind::TopoVolume, 1)).unwrap();
    for f in 0..b.face_count() {
        b.set_class(MeshEntityRef::Face(FaceIx::new(f)), on(EntityKind::TopoFace, 1 + f as u64)).unwrap();
    }
    b.finish(&mut SerialAlloc::new()).unwrap().mesh
}

type RawArrays = BTreeMap<String, (String, Vec<u8>)>;

/// The appended-raw arrays of a VTU file, keyed `Section/Name`, as `(type, payload bytes)`.
/// Written without the exporter's helpers: header text scan plus the `UInt64` block headers.
fn raw_arrays(bytes: &[u8]) -> RawArrays {
    let at = bytes.windows(13).position(|w| w == b"<AppendedData").unwrap();
    let head = std::str::from_utf8(&bytes[..at]).unwrap();
    let body = &bytes[at + bytes[at..].iter().position(|b| *b == b'_').unwrap() + 1..];
    let attr = |line: &str, key: &str| -> String {
        let p = line.find(&format!("{key}=\"")).unwrap() + key.len() + 2;
        line[p..p + line[p..].find('"').unwrap()].to_string()
    };
    let mut section = String::new();
    let mut out = BTreeMap::new();
    for line in head.lines().map(str::trim) {
        if let Some(s) = ["Points", "Cells", "PointData", "CellData"].iter().find(|s| line.starts_with(&format!("<{s}"))) {
            section = (*s).to_string();
        } else if line.starts_with("<DataArray") {
            let off: usize = attr(line, "offset").parse().unwrap();
            let len = u64::from_le_bytes(body[off..off + 8].try_into().unwrap()) as usize;
            out.insert(format!("{section}/{}", attr(line, "Name")), (attr(line, "type"), body[off + 8..off + 8 + len].to_vec()));
        }
    }
    out
}

fn as_i32(a: &(String, Vec<u8>)) -> Vec<i32> {
    assert_eq!(a.0, "Int32");
    a.1.chunks(4).map(|c| i32::from_le_bytes(c.try_into().unwrap())).collect()
}

fn as_f64(a: &(String, Vec<u8>)) -> Vec<f64> {
    assert_eq!(a.0, "Float64");
    a.1.chunks(8).map(|c| f64::from_le_bytes(c.try_into().unwrap())).collect()
}

fn export(m: &ReferenceMesh) -> RawArrays {
    raw_arrays(&vtu_bytes(m, &VtuOptions::default()).unwrap())
}

#[test]
fn fixed_oracle_wedge() {
    let pts = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [0., 1., 1.]];
    let m = single_cell(&pts, |b, vs| b.add_cell_from_vertices(CellShape::Wedge, vs).unwrap().cell);
    let a = export(&m);
    assert_eq!(as_f64(&a["Points/Points"]), pts.iter().flatten().copied().collect::<Vec<f64>>());
    assert_eq!(a["Cells/types"].1, vec![13]);
    assert_eq!(as_i32(&a["Cells/connectivity"]), vec![0, 1, 2, 3, 4, 5]);
    assert_eq!(as_i32(&a["Cells/offsets"]), vec![6]);
    assert!(!a.contains_key("Cells/faces") && !a.contains_key("Cells/faceoffsets"));
}

#[test]
fn fixed_oracle_pyramid() {
    let pts = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0.5, 0.5, 1.]];
    let m = single_cell(&pts, |b, vs| b.add_cell_from_vertices(CellShape::Pyramid, vs).unwrap().cell);
    let a = export(&m);
    assert_eq!(as_f64(&a["Points/Points"]), pts.iter().flatten().copied().collect::<Vec<f64>>());
    assert_eq!(a["Cells/types"].1, vec![14]);
    assert_eq!(as_i32(&a["Cells/connectivity"]), vec![0, 1, 2, 3, 4]);
    assert_eq!(as_i32(&a["Cells/offsets"]), vec![5]);
    assert!(!a.contains_key("Cells/faces"));
}

/// Pentagonal prism: not a template shape, so it is a `VTK_POLYHEDRON`.
fn prism5() -> ([[f64; 3]; 10], ReferenceMesh) {
    let ring = [[0., 0.], [2., 0.], [3., 1.], [1., 3.], [-1., 1.]];
    let mut pts = [[0.0; 3]; 10];
    for i in 0..5 {
        pts[i] = [ring[i][0], ring[i][1], 0.0];
        pts[5 + i] = [ring[i][0], ring[i][1], 1.0];
    }
    let m = single_cell(&pts, |b, vs| {
        let v = |i: usize| vs[i];
        let bottom: Vec<VertexIx> = [4, 3, 2, 1, 0].map(v).to_vec();
        let top: Vec<VertexIx> = [5, 6, 7, 8, 9].map(v).to_vec();
        let sides: Vec<Vec<VertexIx>> = (0..5).map(|i| vec![v(i), v((i + 1) % 5), v(5 + (i + 1) % 5), v(5 + i)]).collect();
        let mut loops: Vec<&[VertexIx]> = vec![&bottom, &top];
        loops.extend(sides.iter().map(Vec::as_slice));
        b.add_polyhedron(&loops).unwrap().cell
    });
    (pts, m)
}

#[test]
fn fixed_oracle_polyhedron() {
    let (pts, m) = prism5();
    assert_eq!(m.cell_shape(CellIx(0)), CellShape::Polyhedron);
    let a = export(&m);
    assert_eq!(as_f64(&a["Points/Points"]), pts.iter().flatten().copied().collect::<Vec<f64>>());
    assert_eq!(a["Cells/types"].1, vec![42]);
    assert_eq!(as_i32(&a["Cells/connectivity"]), (0..10).collect::<Vec<i32>>());
    assert_eq!(as_i32(&a["Cells/offsets"]), vec![10]);
    // Seven faces, each `n v0 .. vn-1` with the loop outward (checked by hand against the
    // prism: bottom runs clockwise seen from +z, top and sides counter-clockwise from outside;
    // loops start at their smallest vertex).
    let faces = vec![
        7, //
        4, 0, 1, 6, 5, // side 0-1
        5, 0, 4, 3, 2, 1, // bottom (normal -z)
        4, 0, 5, 9, 4, // side 4-0
        4, 1, 2, 7, 6, // side 1-2
        4, 2, 3, 8, 7, // side 2-3
        4, 3, 4, 9, 8, // side 3-4
        5, 5, 6, 7, 8, 9, // top (normal +z)
    ];
    assert_eq!(as_i32(&a["Cells/faces"]), faces);
    assert_eq!(as_i32(&a["Cells/faceoffsets"]), vec![faces.len() as i32]);
}

/// A mesh whose tag columns are replaced (the wrapper edits only `tags`; every other call goes
/// to the real mesh), so a column length can disagree with the entity count.
struct WrongTags<'a> {
    inner: &'a ReferenceMesh,
    tags: Vec<TagColumn>,
}

impl MeshRead for WrongTags<'_> {
    fn mesh_id(&self) -> sm_core::EntityId {
        self.inner.mesh_id()
    }
    fn count(&self, k: MeshEntityKind) -> usize {
        self.inner.count(k)
    }
    fn id(&self, k: MeshEntityKind, ix: u32) -> sm_core::EntityId {
        self.inner.id(k, ix)
    }
    fn index_of(&self, id: sm_core::EntityId) -> Option<u32> {
        self.inner.index_of(id)
    }
    fn coords(&self) -> Coords<'_> {
        self.inner.coords()
    }
    fn columns(&self) -> TopologyColumns<'_> {
        self.inner.columns()
    }
    fn face_vertices(&self, f: FaceIx) -> &[VertexIx] {
        self.inner.face_vertices(f)
    }
    fn face_left(&self, f: FaceIx) -> CellIx {
        self.inner.face_left(f)
    }
    fn face_right(&self, f: FaceIx) -> Option<CellIx> {
        self.inner.face_right(f)
    }
    fn face_kind(&self, f: FaceIx) -> FaceKind {
        self.inner.face_kind(f)
    }
    fn dim(&self) -> MeshDim {
        self.inner.dim()
    }
    fn cell_faces(&self, c: CellIx) -> &[SignedFace] {
        self.inner.cell_faces(c)
    }
    fn cell_shape(&self, c: CellIx) -> CellShape {
        self.inner.cell_shape(c)
    }
    fn cell_corners(&self, c: CellIx) -> Option<&[VertexIx]> {
        self.inner.cell_corners(c)
    }
    fn high_order_cell(&self, c: CellIx) -> Option<&HighOrderCell> {
        self.inner.high_order_cell(c)
    }
    fn face_curve(&self, f: FaceIx) -> Option<FaceCurve<'_>> {
        self.inner.face_curve(f)
    }
    fn cell_parent(&self, c: CellIx) -> Option<CellIx> {
        self.inner.cell_parent(c)
    }
    fn cell_active(&self, c: CellIx) -> bool {
        self.inner.cell_active(c)
    }
    fn classification(&self, k: MeshEntityKind, ix: u32) -> Classification {
        self.inner.classification(k, ix)
    }
    fn classified_on(&self, model: sm_core::EntityId) -> Option<&ClassSet> {
        self.inner.classified_on(model)
    }
    fn classified_entities(&self) -> &[sm_core::EntityId] {
        self.inner.classified_entities()
    }
    fn edges(&self) -> Option<&EdgeTable> {
        self.inner.edges()
    }
    fn sets(&self) -> &[MeshSet] {
        self.inner.sets()
    }
    fn tags(&self) -> &[TagColumn] {
        &self.tags
    }
}

#[test]
fn tag_columns_of_the_wrong_length_are_an_error() {
    let m = hex_block([2, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let (nv, nc) = (m.vertex_count(), m.cell_count());
    for (target, n) in [(MeshEntityKind::Cell, nc), (MeshEntityKind::Vertex, nv)] {
        for values in [TagValues::I64(vec![1; n - 1]), TagValues::F64(vec![1.0; n + 1])] {
            let w = WrongTags { inner: &m, tags: vec![TagColumn::new("bad", target, values)] };
            let r = vtu_bytes(&w, &VtuOptions::default());
            assert!(matches!(r, Err(sm_meshio::MeshioError::Inconsistent(_))), "{target:?}: {r:?}");
        }
        // The right length passes.
        let w = WrongTags { inner: &m, tags: vec![TagColumn::new("good", target, TagValues::I64(vec![1; n]))] };
        vtu_bytes(&w, &VtuOptions::default()).unwrap();
    }
    // With tags switched off the column is not looked at.
    let w = WrongTags { inner: &m, tags: vec![TagColumn::new("bad", MeshEntityKind::Cell, TagValues::I64(vec![]))] };
    let mut o = VtuOptions::default();
    o.tags = false;
    vtu_bytes(&w, &o).unwrap();
}

// ---- Scope D: polyhedra decode to the fixture volumes -------------------------------------------

/// Cell volumes decoded from the raw VTU arrays alone: `1/3 sum p0 . Sf` over the outward faces
/// (`faces` stream for `VTK_POLYHEDRON`, the template faces for a hexahedron), with Newell area vectors.
fn volumes_from_vtu(m: &ReferenceMesh) -> (Vec<f64>, Vec<i32>, Vec<i32>) {
    let a = export(m);
    let pts = as_f64(&a["Points/Points"]);
    let p = |i: i32| [pts[3 * i as usize], pts[3 * i as usize + 1], pts[3 * i as usize + 2]];
    let types = &a["Cells/types"].1;
    let (conn, off) = (as_i32(&a["Cells/connectivity"]), as_i32(&a["Cells/offsets"]));
    let (faces, foff) = (a.get("Cells/faces").map(as_i32).unwrap_or_default(), a.get("Cells/faceoffsets").map(as_i32).unwrap_or_default());
    let third_p0_sf = |l: &[i32]| -> f64 {
        let mut s = [0.0; 3];
        for k in 0..l.len() {
            let (u, v) = (p(l[k]), p(l[(k + 1) % l.len()]));
            s[0] += (u[1] * v[2] - u[2] * v[1]) / 2.0;
            s[1] += (u[2] * v[0] - u[0] * v[2]) / 2.0;
            s[2] += (u[0] * v[1] - u[1] * v[0]) / 2.0;
        }
        let p0 = p(l[0]);
        (p0[0] * s[0] + p0[1] * s[1] + p0[2] * s[2]) / 3.0
    };
    let mut vols = vec![];
    let mut stream_at = 0usize;
    for c in 0..types.len() {
        let lo = if c == 0 { 0 } else { off[c - 1] as usize };
        let corners = &conn[lo..off[c] as usize];
        vols.push(match types[c] {
            42 => {
                let end = foff[c] as usize;
                let (mut at, mut v) = (stream_at, 0.0);
                let nfaces = faces[at];
                at += 1;
                for _ in 0..nfaces {
                    let n = faces[at] as usize;
                    v += third_p0_sf(&faces[at + 1..at + 1 + n]);
                    at += 1 + n;
                }
                assert_eq!(at, end, "face stream of cell {c} ends at its faceoffset");
                stream_at = end;
                v
            }
            12 => CellShape::Hex.local_faces().iter().map(|f| third_p0_sf(&f.iter().map(|i| corners[*i as usize]).collect::<Vec<_>>())).sum(),
            t => panic!("unexpected VTK type {t}"),
        });
    }
    (vols, types.iter().map(|t| i32::from(*t)).collect(), foff)
}

fn assert_volumes(name: &str, m: &ReferenceMesh, want: &[f64]) {
    let (got, _, _) = volumes_from_vtu(m);
    assert_eq!(got.len(), want.len(), "{name}");
    for (c, (g, w)) in got.iter().zip(want).enumerate() {
        assert!((g - w).abs() <= 1e-12 * w.abs(), "{name} cell {c}: {g} vs {w}");
    }
}

#[test]
fn polyhedra_decode_to_the_fixture_volumes() {
    use sm_testkit::meshes::{CellSpec, build_mesh, dual_patch, hexagonal_prism, nonconvex_poly, split_face_cube};
    for f in [dual_patch().unwrap(), hexagonal_prism(1.0, 2.0).unwrap(), nonconvex_poly().unwrap(), split_face_cube().unwrap()] {
        assert_volumes(f.name, &f.mesh, &f.cell_volumes);
    }
    assert_eq!(dual_patch().unwrap().cell_volumes[0], 1.5 * 3f64.sqrt());
    // The non-convex L prism is a polyhedron next to a hexahedron: -1 in `faceoffsets` for the hex.
    let nc = nonconvex_poly().unwrap();
    let (_, types, foff) = volumes_from_vtu(&nc.mesh);
    assert_eq!(types, [42, 12]);
    assert!(foff[0] > 0 && foff[1] == -1);

    // A hex stacked on the 7-face cube: the shared quad is a face of both, the hex comes second.
    let cube = split_face_cube().unwrap();
    let mut pts: Vec<[f64; 3]> = (0..8).map(|i| {
        let c = cube.mesh.point(i);
        [c.x, c.y, c.z]
    }).collect();
    pts.extend([[0., 0., 2.], [1., 0., 2.], [1., 1., 2.], [0., 1., 2.]]);
    let loops = vec![vec![0, 2, 1], vec![0, 3, 2], vec![4, 5, 6, 7], vec![0, 1, 5, 4], vec![1, 2, 6, 5], vec![2, 3, 7, 6], vec![3, 0, 4, 7]];
    let stacked = build_mesh(&pts, &[CellSpec::Poly(loops), CellSpec::Template(CellShape::Hex, vec![4, 5, 6, 7, 8, 9, 10, 11])], &mut SerialAlloc::new()).unwrap();
    assert_eq!((stacked.cell_shape(CellIx(0)), stacked.cell_shape(CellIx(1))), (CellShape::Polyhedron, CellShape::Hex));
    assert_eq!(stacked.face_count() - (0..stacked.face_count()).filter(|f| stacked.is_boundary_face(FaceIx::new(*f))).count(), 1);
    assert_volumes("stacked", &stacked, &[1.0, 1.0]);
    let (_, types, foff) = volumes_from_vtu(&stacked);
    assert_eq!(types, [42, 12]);
    assert!(foff[0] > 0 && foff[1] == -1);
    // The face stream lists outward loops: the cube's top face (shared with the hex) points up
    // (+z, out of the lower cell), so its loop is counter-clockwise seen from above: 4 5 6 7.
    let a = export(&stacked);
    let faces = as_i32(&a["Cells/faces"]);
    assert_eq!(faces[0], 7, "seven faces");
    let mut at = 1;
    let mut quad_loops = vec![];
    for _ in 0..7 {
        let n = faces[at] as usize;
        if n == 4 && faces[at + 1..at + 5].iter().all(|v| (4..8).contains(v)) {
            quad_loops.push(faces[at + 1..at + 5].to_vec());
        }
        at += 1 + n;
    }
    assert_eq!(quad_loops, [vec![4, 5, 6, 7]]);
}
