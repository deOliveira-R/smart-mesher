//! Hostile `.smsh` input: every index column is range-checked by the decoder (one test per
//! class, each changing a value but not the length), and stale hierarchy maps are refused by
//! the writer instead of panicking.

mod common;

use common::*;
use sm_core::EntityKind;
use sm_meshio::smsh::SmshFile;
use sm_meshio::{MeshioError, SmshWriter};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh, hex_block};
use sm_mesh_api::*;

const BIG: u64 = 1_000_000;

fn header_len(b: &[u8]) -> usize {
    u64::from_le_bytes(b[8..16].try_into().unwrap()) as usize
}

/// A copy of `good` with element `idx` of array `name` (elements of `size` bytes) set to `val`.
/// The CRC is left stale on purpose: `decode` does not check it, only `verify_checksums` does.
fn poke(good: &[u8], name: &str, size: usize, idx: usize, val: u64) -> Vec<u8> {
    let f = SmshFile::parse(good).unwrap();
    let a = f.header().array(name).unwrap_or_else(|| panic!("no array {name}")).clone();
    assert!((idx + 1) * size <= a.nbytes as usize, "{name}[{idx}] is outside the array");
    let at = 16 + header_len(good) + a.offset as usize + idx * size;
    let mut b = good.to_vec();
    b[at..at + size].copy_from_slice(&val.to_le_bytes()[..size]);
    b
}

/// Replace header text with text of the same length.
fn patch_header(good: &[u8], from: &str, to: &str) -> Vec<u8> {
    assert_eq!(from.len(), to.len());
    let hl = header_len(good);
    let text = std::str::from_utf8(&good[16..16 + hl]).unwrap();
    assert!(text.contains(from), "header has no {from}");
    let mut b = good.to_vec();
    b[16..16 + hl].copy_from_slice(text.replacen(from, to, 1).as_bytes());
    b
}

fn write(m: &dyn MeshRead) -> Vec<u8> {
    SmshWriter::new(m).to_bytes().unwrap()
}

fn set_array(good: &[u8], name: &str) -> String {
    SmshFile::parse(good).unwrap().header().sets.iter().find(|s| s.name == name).unwrap().array.clone()
}

/// The pristine file decodes; the damaged one is `Malformed` and the message names `needle`.
fn expect_malformed(good: &[u8], bad: &[u8], needle: &str) {
    SmshFile::parse(good).unwrap().decode().expect("the undamaged file must decode");
    let r = SmshFile::parse(bad).and_then(|f| f.decode());
    match r {
        Err(MeshioError::Malformed(msg)) => assert!(msg.contains(needle), "expected a message about '{needle}', got: {msg}"),
        other => panic!("expected Malformed about '{needle}', got {:?}", other.map(|_| "a decoded mesh")),
    }
}

fn decorated() -> Vec<u8> {
    write(&decorated_block())
}

#[test]
fn face_cell_columns_are_range_checked() {
    let g = decorated();
    expect_malformed(&g, &poke(&g, "face_left", 4, 0, BIG), "face_left");
    expect_malformed(&g, &poke(&g, "face_right", 4, 0, BIG), "face_right");
    expect_malformed(&g, &poke(&g, "cell_faces", 4, 0, u64::from(u32::MAX - 1)), "cell_faces");
    expect_malformed(&g, &poke(&g, "face_vertices", 4, 0, BIG), "vertex index");
    expect_malformed(&g, &poke(&g, "cell_corners", 4, 0, BIG), "vertex index");
}

#[test]
fn code_columns_outside_the_table_are_rejected() {
    let g = decorated();
    expect_malformed(&g, &poke(&g, "cell_shape", 1, 0, 8), "cell_shape");
    expect_malformed(&g, &poke(&g, "cell_shape", 1, 3, 255), "cell_shape");
    expect_malformed(&g, &poke(&g, "face_kind", 1, 0, 4), "face_kind");
    expect_malformed(&g, &poke(&g, "face_kind", 1, 2, 255), "face_kind");
    // The last valid codes pass the table check; the 1.1 `mesh.stats` then disagree with the
    // arrays, which is what the decoder reports.
    expect_malformed(&g, &poke(&g, "cell_shape", 1, 0, 7), "mesh.stats");
    expect_malformed(&g, &poke(&g, "face_kind", 1, 0, 3), "mesh.stats");
}

#[test]
fn tampered_stats_are_rejected() {
    let g = write(&sm_testkit::meshes::mixed_transition().unwrap().mesh);
    let f = SmshFile::parse(&g).unwrap();
    let st = f.header().mesh.stats.clone().expect("1.1 files carry stats");
    assert_eq!((st.n_hex, st.n_wedge, st.n_pyramid, st.n_tet), (1, 1, 1, 1));
    // Same-length edits of single digits in the header text.
    expect_malformed(&g, &patch_header(&g, "\"n_interior_faces\":3", "\"n_interior_faces\":4"), "mesh.stats");
    expect_malformed(&g, &patch_header(&g, "\"max_cell_faces\":6", "\"max_cell_faces\":7"), "mesh.stats");
    expect_malformed(&g, &patch_header(&g, "\"n_hex\":1", "\"n_hex\":2"), "mesh.stats");
}

#[test]
fn element_type_records_are_checked_by_the_decoder() {
    let m = hex_block([2, 2, 2], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let mut m = m;
    MeshEdit::add_set(&mut m, "b", SetMembers::Cells((0..8).map(CellIx).collect()), &mut SerialAlloc::new()).unwrap();
    let g = SmshWriter::new(&m).element_type("b", "HEX8").to_bytes().unwrap();
    SmshFile::parse(&g).unwrap().decode().unwrap();
    expect_malformed(&g, &patch_header(&g, "\"HEX8\"", "\"TET4\""), "element type");
    expect_malformed(&g, &patch_header(&g, "\"HEX8\"", "\"HEX9\""), "unknown element type");
    expect_malformed(&g, &patch_header(&g, "\"set\":\"b\"", "\"set\":\"c\""), "unknown set");
}

#[test]
fn cell_parent_is_range_checked() {
    let g = write(&lineage_pair());
    expect_malformed(&g, &poke(&g, "cell_parent", 4, 0, 7), "cell_parent");
}

#[test]
fn set_members_are_range_checked_by_kind() {
    let g = decorated();
    for set in ["corner", "wall", "fuel", "wall_sides", "pts"] {
        // A word past every limit fails whatever the kind.
        let arr = set_array(&g, set);
        expect_malformed(&g, &poke(&g, &arr, 4, 0, BIG), &arr);
    }
    // Tight limits: vertex sets stop at nv, point sets at np (> nv only with high-order nodes).
    let f = SmshFile::parse(&g).unwrap();
    let mi = &f.header().mesh;
    let arr = set_array(&g, "corner");
    expect_malformed(&g, &poke(&g, &arr, 4, 1, mi.n_vertices), &arr);
    let arr = set_array(&g, "pts");
    expect_malformed(&g, &poke(&g, &arr, 4, 1, mi.n_points), &arr);
    let arr = set_array(&g, "fuel");
    expect_malformed(&g, &poke(&g, &arr, 4, 0, mi.n_cells), &arr);
    // Sides are packed `face << 1 | side`: the side bit alone must not matter, the face must.
    let arr = set_array(&g, "wall_sides");
    expect_malformed(&g, &poke(&g, &arr, 4, 0, mi.n_faces << 1), &arr);
}

#[test]
fn shell_side_words_are_range_checked() {
    let mut alloc = SerialAlloc::new();
    let mut m = surface_strip();
    MeshEdit::add_set(&mut m, "top", SetMembers::ShellSides(vec![0, 2, 4]), &mut alloc).unwrap();
    let g = write(&m);
    let arr = set_array(&g, "top");
    // cell 3 does not exist (3 cells).
    expect_malformed(&g, &poke(&g, &arr, 4, 2, 6), &arr);
    expect_malformed(&g, &poke(&g, &arr, 4, 0, BIG), &arr);
}

#[test]
fn edge_table_indices_are_range_checked() {
    let g = decorated();
    expect_malformed(&g, &poke(&g, "edge_vertices", 4, 0, BIG), "edge_vertices");
    expect_malformed(&g, &poke(&g, "edge_face_edge", 4, 0, BIG), "edge_face_edge");
    let n_edges = SmshFile::parse(&g).unwrap().header().mesh.n_edges;
    expect_malformed(&g, &poke(&g, "edge_face_edge", 4, 1, n_edges), "edge_face_edge");
}

#[test]
fn high_order_records_are_range_checked() {
    let g = write(&ho_hex());
    expect_malformed(&g, &poke(&g, "ho_cell", 4, 0, 5), "ho_cell");
    expect_malformed(&g, &poke(&g, "ho_nodes", 4, 0, BIG), "ho_nodes");
    let np = SmshFile::parse(&g).unwrap().header().mesh.n_points;
    expect_malformed(&g, &poke(&g, "ho_nodes", 4, 8, np), "ho_nodes");
}

#[test]
fn face_curve_records_are_range_checked() {
    let mut m = ho_hex();
    let face = FaceIx(0);
    let mut nodes: Vec<PointIx> = m.face_vertices(face).iter().map(|v| PointIx(v.0)).collect();
    nodes.extend((8..13).map(PointIx));
    MeshEdit::set_face_curve(&mut m, face, Some((2, Basis::LagrangeEquispaced, nodes))).unwrap();
    let g = write(&m);
    expect_malformed(&g, &poke(&g, "fc_face", 4, 0, BIG), "fc_face");
    expect_malformed(&g, &poke(&g, "fc_nodes", 4, 4, BIG), "fc_nodes");
}

#[test]
fn tag_column_length_must_match_its_target() {
    let g = decorated();
    // Tag "mat" has one value per cell (12); retargeting it at faces (same text length) leaves a
    // column of the wrong length.
    expect_malformed(&g, &patch_header(&g, "\"target\":\"cell\"", "\"target\":\"face\""), "tag.");
}

#[test]
fn point_count_must_cover_the_vertices() {
    let g = write(&hex_block([4, 3, 2], [1.0; 3], &mut SerialAlloc::new()).unwrap());
    let f = SmshFile::parse(&g).unwrap();
    let nv = f.header().mesh.n_vertices;
    assert_eq!(nv, f.header().mesh.n_points);
    expect_malformed(&g, &patch_header(&g, &format!("\"n_points\":{nv}"), &format!("\"n_points\":{}", nv - 1)), "n_points");
}

// ---- Hierarchy maps ------------------------------------------------------------------------

/// A strip of `n` unit quads in z = 0.
fn strip(n: usize, alloc: &mut SerialAlloc) -> ReferenceMesh {
    let mut b = ReferenceBuilder::surface();
    let mut vs = Vec::new();
    for j in 0..2 {
        for i in 0..=n {
            let v = b.add_vertex(sm_core::Point3::new(i as f64, j as f64, 0.0)).unwrap();
            b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, 1 + vs.len() as u64)).unwrap();
            vs.push(v);
        }
    }
    for i in 0..n {
        let q = [vs[i], vs[i + 1], vs[n + 1 + i + 1], vs[n + 1 + i]];
        let c = b.add_polygon(&q).unwrap();
        b.set_class(MeshEntityRef::Cell(c.cell), on(EntityKind::TopoFace, 1)).unwrap();
    }
    for f in 0..b.face_count() {
        let fi = FaceIx::new(f);
        let c = if b.face_degree(fi) == 1 { on(EntityKind::TopoEdge, 1 + f as u64) } else { on(EntityKind::TopoFace, 1) };
        b.set_class(MeshEntityRef::Face(fi), c).unwrap();
    }
    b.finish(alloc).unwrap().mesh
}

fn centre(m: &dyn MeshRead, c: usize) -> sm_core::Point3 {
    cell_volume_centroid(m, CellIx::new(c)).1
}

struct Hierarchy {
    fine: ReferenceMesh,
    coarse: ReferenceMesh,
    lm: LevelMap,
    base: ReferenceMesh,
    ext: ReferenceMesh,
    xm: ExtrusionMap,
}

fn hierarchy() -> Hierarchy {
    let mut alloc = SerialAlloc::new();
    let fine = hex_block([4, 4, 2], [1.0; 3], &mut alloc).unwrap();
    let coarse = hex_block([2, 2, 1], [2.0; 3], &mut alloc).unwrap();
    let coarse_of: Vec<CellIx> = (0..fine.cell_count())
        .map(|c| {
            let p = centre(&fine, c);
            CellIx::new((p.x / 2.0).floor() as usize + 2 * (p.y / 2.0).floor() as usize)
        })
        .collect();
    let lm = LevelMap::new(&fine, &coarse, coarse_of, None).unwrap();
    let base = strip(2, &mut alloc);
    let ext = hex_block([2, 1, 3], [1.0; 3], &mut alloc).unwrap();
    let cell_of: Vec<(CellIx, u32)> = (0..ext.cell_count())
        .map(|c| {
            let p = centre(&ext, c);
            (CellIx::new(p.x.floor() as usize), p.z.floor() as u32)
        })
        .collect();
    let xm = ExtrusionMap::new(&base, &ext, vec![0.0, 1.0, 2.0, 3.0], cell_of).unwrap();
    Hierarchy { fine, coarse, lm, base, ext, xm }
}

/// Swap the two ids of a header field pair (same-length ids).
fn swap_ids(good: &[u8], a: (&str, &str), b: (&str, &str)) -> Vec<u8> {
    let (ta, tb) = (format!("\"{}\":\"{}\"", a.0, a.1), format!("\"{}\":\"{}\"", b.0, b.1));
    let tmp = patch_header(good, &ta, &format!("\"{}\":\"{}\"", a.0, "X".repeat(a.1.len())));
    let tmp = patch_header(&tmp, &tb, &format!("\"{}\":\"{}\"", b.0, "Y".repeat(b.1.len())));
    let tmp = patch_header(&tmp, &format!("\"{}\":\"{}\"", a.0, "X".repeat(a.1.len())), &format!("\"{}\":\"{}\"", a.0, b.1));
    patch_header(&tmp, &format!("\"{}\":\"{}\"", b.0, "Y".repeat(b.1.len())), &format!("\"{}\":\"{}\"", b.0, a.1))
}

#[test]
fn level_map_columns_are_checked_against_the_mesh_role() {
    let h = hierarchy();
    // Coarse file: coarse_of holds fine-cell -> coarse-cell values, all below nc.
    let g = SmshWriter::new(&h.coarse).level_map(&h.lm).to_bytes().unwrap();
    expect_malformed(&g, &poke(&g, "levelmap.0.coarse_of", 4, 0, BIG), "coarse_of");
    // Fine file: the length must equal nc. Relabel the coarse file so its mesh becomes the
    // "fine" one: 32 entries against 4 cells.
    let (fi, co) = (h.fine.mesh_id().to_string(), h.coarse.mesh_id().to_string());
    expect_malformed(&g, &swap_ids(&g, ("fine", &fi), ("coarse", &co)), "coarse_of");
    // A map that does not involve the file's mesh at all.
    let other = SmshWriter::new(&h.coarse).level_map(&h.lm).to_bytes().unwrap();
    let other = patch_header(&other, &format!("\"coarse\":\"{co}\""), &format!("\"coarse\":\"{}\"", co.replace('#', "#9").chars().take(co.len()).collect::<String>()));
    expect_malformed(&g, &other, "does not involve");
}

#[test]
fn extrusion_columns_are_checked_against_the_mesh_role() {
    let h = hierarchy();
    let on_ext = SmshWriter::new(&h.ext).extrusion(&h.xm).to_bytes().unwrap();
    let on_base = SmshWriter::new(&h.base).extrusion(&h.xm).to_bytes().unwrap();
    expect_malformed(&on_ext, &poke(&on_ext, "extrusion.0.cell_layer", 4, 0, 3), "cell_layer");
    expect_malformed(&on_base, &poke(&on_base, "extrusion.0.cell_base", 4, 0, BIG), "cell_base");
    // Length: relabel so the extruded file's mesh looks like the base (needs nc * layers entries).
    let (b, e) = (h.base.mesh_id().to_string(), h.ext.mesh_id().to_string());
    expect_malformed(&on_ext, &swap_ids(&on_ext, ("base", &b), ("extruded", &e)), "cell_base");
}

// ---- Stale maps in the writer -------------------------------------------------------------

#[test]
fn stale_level_map_is_an_error_not_a_panic() {
    let h = hierarchy();
    // A different mesh state under the same id: ids are drawn from a fresh allocator.
    let stale_fine = hex_block([4, 4, 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    assert_eq!(stale_fine.mesh_id(), h.fine.mesh_id());
    assert_ne!(stale_fine.cell_count(), h.fine.cell_count());
    let r = SmshWriter::new(&stale_fine).level_map(&h.lm).to_bytes();
    assert!(matches!(r, Err(MeshioError::Inconsistent(ref m)) if m.contains("stale")), "{r:?}");
    // The coarse side too.
    let mut a = SerialAlloc::new();
    let _ = hex_block([1, 1, 1], [1.0; 3], &mut a).unwrap();
    let stale_coarse = hex_block([3, 3, 1], [1.0; 3], &mut a).unwrap();
    assert_eq!(stale_coarse.mesh_id(), h.coarse.mesh_id());
    let r = SmshWriter::new(&stale_coarse).level_map(&h.lm).to_bytes();
    assert!(matches!(r, Err(MeshioError::Inconsistent(_))), "{r:?}");
    // The matching meshes still write.
    SmshWriter::new(&h.fine).level_map(&h.lm).to_bytes().unwrap();
    SmshWriter::new(&h.coarse).level_map(&h.lm).to_bytes().unwrap();
}

#[test]
fn stale_extrusion_map_is_an_error_not_a_panic() {
    let h = hierarchy();
    // Base id is the third mesh of the shared allocator, extruded the fourth.
    let mut a = SerialAlloc::new();
    let _ = hex_block([1, 1, 1], [1.0; 3], &mut a).unwrap();
    let _ = hex_block([1, 1, 1], [1.0; 3], &mut a).unwrap();
    let stale_base = strip(3, &mut a);
    let stale_ext = hex_block([3, 1, 3], [1.0; 3], &mut a).unwrap();
    assert_eq!((stale_base.mesh_id(), stale_ext.mesh_id()), (h.base.mesh_id(), h.ext.mesh_id()));
    // 3 base cells x 3 layers = 9 extruded cells against a 6-entry map: this used to index out of range.
    for r in [SmshWriter::new(&stale_base).extrusion(&h.xm).to_bytes(), SmshWriter::new(&stale_ext).extrusion(&h.xm).to_bytes()] {
        assert!(matches!(r, Err(MeshioError::Inconsistent(ref m)) if m.contains("stale")), "{r:?}");
    }
    SmshWriter::new(&h.base).extrusion(&h.xm).to_bytes().unwrap();
    SmshWriter::new(&h.ext).extrusion(&h.xm).to_bytes().unwrap();
}
