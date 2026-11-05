//! Acceptance tests 1-8 of the view queries (analytic).
#![allow(missing_docs)]

mod common;

use std::collections::BTreeMap;

use common::*;
use proptest::prelude::*;
use serde_json::{Value, json};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::MeshLayer;
use sm_mesh_api::{CellIx, FaceIx, MeshRead, cell_volume_centroid};
use sm_quality::{Metric, Options};
use sm_testkit::strategies::config;
use sm_view::{Clip, Skin, SkinOptions, ViewError, skin};

fn mesh_of(r: &Rig, id: EntityId) -> &dyn MeshRead {
    r.state().layer::<MeshLayer>().expect("layer").get(id).expect("mesh").mesh()
}

fn box_skin(clip: Option<Clip>, volumes: Option<Vec<EntityId>>) -> (Rig, Skin) {
    let (r, id) = Rig::with("block", [4, 2, 2], &[]);
    let s = skin(mesh_of(&r, id), &SkinOptions { clip, volumes, ..SkinOptions::default() }).expect("skin");
    (r, s)
}

fn vid(k: EntityKind, n: u64) -> EntityId {
    EntityId::new(k, n).expect("id")
}

fn owner_of(s: &Skin, t: usize) -> Option<EntityId> {
    match s.tri_owner[t] {
        0 => None,
        k => Some(s.owners[k as usize - 1]),
    }
}

/// Triangles whose three vertices all have `x == v`.
fn tris_at_x(s: &Skin, v: f64) -> Vec<usize> {
    (0..s.tri_count())
        .filter(|&t| (0..3).all(|k| s.positions[s.indices[t * 3 + k] as usize * 3] == v))
        .collect()
}

#[test]
fn acc1_box_skin() {
    let (_r, s) = box_skin(None, None);
    assert_eq!(s.tri_count(), 80);
    let faces: std::collections::BTreeSet<u32> = s.tri_face.iter().copied().collect();
    assert_eq!(faces.len(), 40);
    assert!(s.tri_owner.iter().all(|o| *o != 0));
    assert!((area(&s) - 40.0).abs() < 1e-12);
    assert_eq!((s.clipped_cells, s.kept_cells), (0, 16));
    assert_eq!(s.vertex_count(), 5 * 3 * 3 - 3);
    // Unique polygon edges of the 4x2x2 box surface: 2*(4*3 + 2*5 ... ) counted by Euler: V - E + F = 2.
    assert_eq!(s.vertex_count() as i64 - (s.outline_indices.len() / 2) as i64 + 40, 2);
}

#[test]
fn acc2_clip() {
    let c = Clip { origin: [2.0, 0.0, 0.0], normal: [1.0, 0.0, 0.0] };
    let (r, s) = box_skin(Some(c), None);
    assert_eq!((s.clipped_cells, s.kept_cells), (8, 8));
    assert_eq!(s.tri_count(), 48);
    assert!((area(&s) - 24.0).abs() < 1e-12);
    let cap = tris_at_x(&s, 2.0);
    assert_eq!(cap.len(), 8);
    let (_, id) = (0, r.state().layer::<MeshLayer>().unwrap().meshes().next().unwrap().0);
    let m = mesh_of(&r, id);
    for &t in &cap {
        let p = cell_volume_centroid(m, CellIx::new(s.tri_cell[t] as usize)).1;
        assert!(p.x < 2.0, "cap triangle shows a kept cell");
    }
    // Everything removed: empty skin, not an error.
    let (_, s) = box_skin(Some(Clip { origin: [-1.0, 0.0, 0.0], normal: [1.0, 0.0, 0.0] }), None);
    assert_eq!((s.tri_count(), s.clipped_cells, s.kept_cells), (0, 16, 0));
    assert_eq!(s.bbox, [[0.0; 3]; 2]);
    // A plane through the centroids at x = 0.5 keeps that column (dot = 0).
    let (_, s) = box_skin(Some(Clip { origin: [0.5, 0.0, 0.0], normal: [1.0, 0.0, 0.0] }), None);
    assert_eq!((s.clipped_cells, s.kept_cells), (12, 4));
}

#[test]
fn acc3_two_volumes() {
    let (r, id) = Rig::with("two_volumes", [0; 3], &[]);
    let m = mesh_of(&r, id);
    let both = skin(m, &SkinOptions::default()).expect("skin");
    // 80 boundary triangles plus the 2x2 cap shown from both sides: 4 quads x 2 sides x 2.
    assert_eq!(both.tri_count(), 80 + 16);
    let cap = tris_at_x(&both, 2.0);
    assert_eq!(cap.len(), 16);
    for &t in &cap {
        assert_eq!(owner_of(&both, t).map(|e| e.kind()), Some(EntityKind::TopoFace), "interface owner is the model face");
    }
    // Both sides' area is 4 each.
    let one = skin(m, &SkinOptions { volumes: Some(vec![vid(EntityKind::TopoVolume, 1)]), ..SkinOptions::default() }).expect("skin");
    assert_eq!(one.tri_count(), 48);
    let cap = tris_at_x(&one, 2.0);
    assert_eq!(cap.len(), 8);
    for &t in &cap {
        assert_eq!(owner_of(&one, t), Some(vid(EntityKind::TopoVolume, 1)));
    }
    let cap_area: f64 = {
        let mut only_cap = one.clone();
        let keep: Vec<usize> = cap.clone();
        only_cap.indices = keep.iter().flat_map(|&t| one.indices[t * 3..t * 3 + 3].to_vec()).collect();
        area(&only_cap)
    };
    assert!((cap_area - 4.0).abs() < 1e-12);
}

fn edge_use(s: &Skin) -> BTreeMap<(u32, u32), (u32, i32)> {
    let mut m: BTreeMap<(u32, u32), (u32, i32)> = BTreeMap::new();
    for t in s.indices.as_chunks::<3>().0 {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            let e = m.entry((a.min(b), a.max(b))).or_default();
            e.0 += 1;
            e.1 += if a < b { 1 } else { -1 };
        }
    }
    m
}

#[test]
fn acc4_polyhedral_skin() {
    let (r, id) = Rig::with("dual_patch", [0; 3], &[]);
    let m = mesh_of(&r, id);
    let s = skin(m, &SkinOptions::default()).expect("skin");
    // Watertight and consistently oriented: every edge used twice, once in each direction.
    for (e, (n, dir)) in edge_use(&s) {
        assert_eq!((n, dir), (2, 0), "edge {e:?}");
    }
    let fixture = sm_testkit::meshes::dual_patch().expect("fixture");
    assert!((area(&s) - fixture.area).abs() < 1e-12, "{} vs {}", area(&s), fixture.area);
    // Sum of boundary facet areas, independently.
    let boundary: f64 = (0..m.face_count())
        .filter(|&f| m.face_right(FaceIx::new(f)).is_none())
        .map(|f| {
            let a = sm_mesh_api::face_area_vector(m, FaceIx::new(f));
            (a.x * a.x + a.y * a.y + a.z * a.z).sqrt()
        })
        .sum();
    assert!((area(&s) - boundary).abs() < 1e-12);
    // 14 hexagonal caps are fanned from appended centroid vertices.
    let n_nodes = s.node_vertices as usize;
    assert_eq!(s.vertex_count() - n_nodes, 14);
    assert_eq!(s.vertex_node.len(), n_nodes);
    let max_idx = *s.indices.iter().max().expect("indices") as usize;
    assert_eq!(max_idx, s.vertex_count() - 1);
    for t in s.indices.as_chunks::<3>().0 {
        let fans = t.iter().filter(|&&i| i as usize >= n_nodes).count();
        assert!(fans <= 1, "a fan triangle has exactly one centroid vertex");
    }
}

#[test]
fn acc5_surface_mesh() {
    let (r, id) = Rig::with("quad_patch", [0; 3], &[]);
    let m = mesh_of(&r, id);
    let s = skin(m, &SkinOptions::default()).expect("skin");
    assert_eq!(s.tri_count(), 18);
    assert!(s.tri_face.iter().all(|f| *f == sm_view::NO_FACE));
    assert!((area(&s) - 9.0).abs() < 1e-12);
    // 3x3 grid: 24 unique polygon edges.
    assert_eq!(s.outline_indices.len() / 2, 24);
    // Clip removes polygons by centroid: x > 1.5 drops the third column.
    let c = Clip { origin: [1.5, 0.0, 0.0], normal: [1.0, 0.0, 0.0] };
    let s = skin(m, &SkinOptions { clip: Some(c), ..SkinOptions::default() }).expect("skin");
    assert_eq!((s.kept_cells, s.clipped_cells, s.tri_count()), (6, 3, 12));
}

#[test]
fn acc6_scalar() {
    // Interior vertex (1,1,1) of a 3x3x3 block moved by 0.3.
    let (r0, id0) = Rig::with("block", [3, 3, 3], &[]);
    let p = mesh_of(&r0, id0).point(21);
    assert_eq!((p.x, p.y, p.z), (1.0, 1.0, 1.0), "vertex numbering assumption");
    let (r, id) = Rig::with("block", [3, 3, 3], &[(21, [1.3, 1.0, 1.0])]);
    let reply = r.query("view.mesh_scalar", json!({"metric": "scaled_jacobian", "bad_below": 0.99})).expect("scalar");
    let m = mesh_of(&r, id);
    let col = sm_quality::compute(m, Metric::ScaledJacobian, &Options::default()).expect("compute");
    let got = b64_f32(&reply["values"]);
    assert_eq!(got.len(), col.len());
    for (g, c) in got.iter().zip(&col) {
        assert_eq!(g.to_bits(), (*c as f32).to_bits());
    }
    let wcell = reply["worst"][0]["cell"]["index"].as_u64().expect("index") as usize;
    let corners = m.cell_corners(CellIx::new(wcell)).expect("corners");
    assert!(corners.iter().any(|v| v.get() == 21), "worst cell touches the moved node");
    let bad = col.iter().filter(|v| **v < 0.99).count();
    assert!(bad > 0);
    assert_eq!(reply["bad_cells"].as_u64(), Some(bad as u64));
    let min = col.iter().copied().fold(f64::INFINITY, f64::min);
    let max = col.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let mean = col.iter().sum::<f64>() / col.len() as f64;
    assert_eq!(reply["min"].as_f64(), Some(min));
    assert_eq!(reply["max"].as_f64(), Some(max));
    assert!((reply["mean"].as_f64().expect("mean") - mean).abs() < 1e-14);
    assert_eq!(reply["worst"][0]["value"].as_f64(), Some(min));
    assert_eq!(reply["undefined_cells"].as_u64(), Some(0));
    assert_eq!(reply["worst_end"], "low");
    // An undistorted box is all ones.
    let (r, _) = Rig::with("block", [4, 2, 2], &[]);
    let reply = r.query("view.mesh_scalar", json!({"metric": "scaled_jacobian"})).expect("scalar");
    assert_eq!((reply["min"].as_f64(), reply["max"].as_f64(), reply["mean"].as_f64()), (Some(1.0), Some(1.0), Some(1.0)));
    // A high-is-worse metric lists the high end first.
    let reply = r.query("view.mesh_scalar", json!({"metric": "edge_ratio", "bad_above": 2.0, "worst": 3})).expect("scalar");
    assert_eq!(reply["worst"].as_array().map(Vec::len), Some(3));
    assert_eq!(reply["worst_end"], "high");
    assert_eq!(reply["bad_cells"].as_u64(), Some(0));
}

#[test]
fn acc6b_metric_undefined_for_shape_is_nan() {
    // Tet-only mesh: hex-only metric is unsupported (typed), a metric defined for tets works.
    let (r, _) = Rig::with("single_tet", [0; 3], &[]);
    let err = r.query("view.mesh_scalar", json!({"metric": "taper"})).expect_err("unsupported");
    assert!(err.contains("taper") && err.contains("not defined"), "{err}");
    let ok = r.query("view.mesh_scalar", json!({"metric": "radius_ratio"})).expect("tet metric");
    assert_eq!(ok["count"], 1);
    assert_eq!(ok["undefined_cells"], 0);
}

#[test]
fn acc7_typed_errors() {
    let (r, id) = Rig::with("block", [4, 2, 2], &[]);
    let err = r.query("view.mesh_scalar", json!({"metric": "nope"})).expect_err("unknown metric");
    assert!(err.contains("unknown metric `nope`") && err.contains("scaled_jacobian"), "{err}");
    // Library form: TooLarge carries the numbers and the hint.
    let e = skin(mesh_of(&r, id), &SkinOptions { max_tris: 10, ..SkinOptions::default() }).expect_err("too large");
    assert_eq!(e, ViewError::TooLarge { tris: 80, max: 10, hint: sm_view::TOO_LARGE_HINT });
    let err = r.query("view.mesh_skin", json!({"max_tris": 10})).expect_err("too large");
    assert!(err.contains("80") && err.contains("max_tris 10") && err.contains("clip"), "{err}");
    // Bad clip.
    let err = r.query("view.mesh_skin", json!({"clip": {"origin": [0, 0, 0], "normal": [0, 0, 0]}})).expect_err("bad clip");
    assert!(err.contains("bad clip plane"), "{err}");
    // Unknown mesh id.
    let err = r.query("view.mesh_skin", json!({"mesh": "Mesh#99"})).expect_err("unknown mesh");
    assert!(err.contains("Mesh#99"), "{err}");
    // Two meshes without `mesh`.
    let mut r = r;
    let d = r.apply(common::InsertMesh { kind: "block".into(), n: [1, 1, 1], moves: vec![] });
    let second = d.outputs["mesh"][0];
    let err = r.query("view.mesh_skin", json!({})).expect_err("ambiguous");
    assert!(err.contains("`mesh` is required") && err.contains(&id.to_string()) && err.contains(&second.to_string()), "{err}");
    let ok = r.query("view.mesh_skin", json!({"mesh": second.to_string()})).expect("named mesh");
    assert_eq!(ok["tri_count"], 12);
    // No mesh at all.
    let empty = Rig::new();
    let err = empty.query("view.mesh_skin", json!({})).expect_err("no mesh");
    assert!(err.contains("no mesh") && err.contains("mesh.run"), "{err}");
}

#[test]
fn scene_reports_model_meshes_and_colors() {
    let mut r = Rig::new();
    let d = r.apply(CreateBrick { size: [2.0, 3.0, 4.0] });
    let vol = d.outputs["volume"][0];
    let empty = r.query("view.scene", json!({})).expect("scene");
    assert_eq!(empty["meshes"], json!([]));
    assert_eq!(empty["bbox"], json!([[0.0, 0.0, 0.0], [2.0, 3.0, 4.0]]));
    assert_eq!(empty["volumes"][0]["color"], Value::Null);
    r.apply(ColorVolume { volume: vol, label: 14, name: "fuel".into() });
    let d = r.apply(InsertMesh { kind: "block".into(), n: [4, 2, 2], moves: vec![] });
    let mid = d.outputs["mesh"][0];
    let sc = r.query("view.scene", json!({})).expect("scene");
    let want = sm_view::palette_rgb(14);
    assert_eq!(sc["volumes"][0]["color"], json!(want));
    assert_eq!(sc["volumes"][0]["color_label"], 14);
    for b in sc["bodies"].as_array().expect("bodies") {
        assert_eq!(b["name"], "fuel");
        assert_eq!(b["color"], json!(want));
    }
    // Union of the brick [0,2]x[0,3]x[0,4] and the mesh [0,4]x[0,2]x[0,2].
    assert_eq!(sc["bbox"], json!([[0.0, 0.0, 0.0], [4.0, 3.0, 4.0]]));
    let m = &sc["meshes"][0];
    assert_eq!(m["id"], mid.to_string());
    let faces = mesh_of(&r, mid).face_count() as u64;
    assert_eq!((m["dim"].as_u64(), m["cells"].as_u64(), m["facets"].as_u64(), m["nodes"].as_u64()), (Some(3), Some(16), Some(faces), Some(45)));
    assert_eq!(m["bbox"], json!([[0.0, 0.0, 0.0], [4.0, 2.0, 2.0]]));
    assert_eq!(m["digest"].as_str().map(str::len), Some(64));
    assert_eq!(m["derived_from"], json!([]));
    // The digest changes when a coordinate moves.
    let (r2, _) = Rig::with("block", [4, 2, 2], &[(0, [-0.5, 0.0, 0.0])]);
    let (r1, _) = Rig::with("block", [4, 2, 2], &[]);
    let d1 = r1.query("view.scene", json!({})).expect("scene")["meshes"][0]["digest"].clone();
    let d2 = r2.query("view.scene", json!({})).expect("scene")["meshes"][0]["digest"].clone();
    assert_ne!(d1, d2);
}

#[test]
fn palette_is_a_fixed_label_rule() {
    let all: std::collections::BTreeSet<[u8; 3]> = (0..sm_view::PALETTE_LEN as u32).map(sm_view::palette_rgb).collect();
    assert_eq!(all.len(), sm_view::PALETTE_LEN);
    assert_eq!(sm_view::palette_rgb(3), sm_view::palette_rgb(3 + sm_view::PALETTE_LEN as u32));
}

#[test]
fn queries_are_registered_with_schemas() {
    let reg = registry();
    for q in ["view.scene", "view.mesh_skin", "view.mesh_scalar"] {
        assert!(reg.query_names().contains(&q), "{q}");
    }
}

fn pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool").install(f)
}

#[test]
fn acc8_determinism_bytes() {
    let (r, _) = Rig::with("block", [6, 4, 3], &[(30, [2.3, 1.1, 1.0])]);
    let go = |threads: usize| -> String {
        pool(threads, || {
            let mut out = String::new();
            for (name, input) in [
                ("view.scene", json!({})),
                ("view.mesh_skin", json!({})),
                ("view.mesh_skin", json!({"clip": {"origin": [3.0, 2.0, 1.0], "normal": [1.0, 0.5, 0.2]}})),
                ("view.mesh_scalar", json!({"metric": "scaled_jacobian", "bad_below": 0.99})),
                ("view.mesh_scalar", json!({"metric": "condition", "bad_above": 1.5})),
            ] {
                out += &serde_json::to_string(&r.query(name, input).expect("query")).expect("json");
            }
            out
        })
    };
    sm_testkit::determinism::check(2, 4, go).expect("byte identical for 1 and N threads");
    let (rp, _) = Rig::with("dual_patch", [0; 3], &[]);
    let go = |threads: usize| pool(threads, || serde_json::to_string(&rp.query("view.mesh_skin", json!({})).expect("q")).expect("json"));
    sm_testkit::determinism::check(2, 4, go).expect("polyhedral skin byte identical");
}

proptest! {
    #![proptest_config(config(48))]

    #[test]
    fn clip_partitions_cells_and_skin_faces(
        o in prop::array::uniform3(-1.0f64..5.0),
        n in prop::array::uniform3(-1.0f64..1.0),
    ) {
        prop_assume!(n.iter().map(|x| x * x).sum::<f64>() > 0.01);
        let (r, id) = Rig::with("block", [4, 2, 2], &[]);
        let m = mesh_of(&r, id);
        let s = skin(m, &SkinOptions { clip: Some(Clip { origin: o, normal: n }), ..SkinOptions::default() }).expect("skin");
        let removed = |c: usize| {
            let p = cell_volume_centroid(m, CellIx::new(c)).1;
            (p.x - o[0]) * n[0] + (p.y - o[1]) * n[1] + (p.z - o[2]) * n[2] > 0.0
        };
        let kept_n = (0..16).filter(|&c| !removed(c)).count() as u32;
        prop_assert_eq!(s.kept_cells, kept_n);
        prop_assert_eq!(s.kept_cells + s.clipped_cells, 16);
        for t in 0..s.tri_count() {
            let f = FaceIx::new(s.tri_face[t] as usize);
            let l = m.face_left(f).get();
            let rr = m.face_right(f).map(|c| c.get());
            let kl = !removed(l);
            let kr = rr.is_some_and(|c| !removed(c));
            prop_assert!(kl != kr, "face {} has exactly one kept side", f.get());
            let vis = s.tri_cell[t] as usize;
            prop_assert!(!removed(vis));
        }
    }
}
