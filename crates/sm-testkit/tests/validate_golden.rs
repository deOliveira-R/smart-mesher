//! `validate` output over the testkit meshes and tampered copies is byte-identical to the
//! recorded one (`UPDATE_GOLDEN=1` rewrites it), so the shared rule internals cannot drift.

use sm_core::Point3;
use sm_mesh_api::reference::ReferenceMesh;
use sm_mesh_api::{MeshEntityKind, MeshRead, validate};
use sm_testkit::broken::Tampered;
use sm_testkit::meshes::{all_mesh_fixtures, hex_block_2x2x2};

fn dump(out: &mut String, name: &str, m: &dyn MeshRead) {
    out.push_str(&format!("== {name}\n"));
    for v in validate(m) {
        out.push_str(&format!("{v}\n"));
    }
}

#[test]
fn validate_is_golden_over_testkit_meshes_and_tampered_copies() {
    let mut out = String::new();
    for f in all_mesh_fixtures().unwrap() {
        dump(&mut out, f.name, &f.mesh);
    }
    let m: ReferenceMesh = hex_block_2x2x2().unwrap().mesh;
    let nf = m.face_count();
    let interior = (0..nf).find(|&f| m.columns().face_right[f] != u32::MAX).unwrap();
    dump(&mut out, "swap", &Tampered::of(&m).swap_left_right(interior));
    dump(&mut out, "reverse", &Tampered::of(&m).reverse_loop(0));
    dump(&mut out, "move", &Tampered::of(&m).move_vertex(0, Point3::new(5.0, 5.0, 5.0)));
    dump(&mut out, "remove", &Tampered::of(&m).remove_cell_face(0, 0));
    dump(&mut out, "right", &Tampered::of(&m).set_face_right(0, 3));
    for (k, n) in [(MeshEntityKind::Vertex, 0), (MeshEntityKind::Face, 0), (MeshEntityKind::Cell, 0)] {
        dump(&mut out, &format!("declassify-{k:?}"), &Tampered::of(&m).declassify(k, n));
    }
    let path = format!("{}/tests/golden/validate_meshes.txt", env!("CARGO_MANIFEST_DIR"));
    if std::env::var_os("UPDATE_GOLDEN").is_some() {
        std::fs::write(&path, &out).unwrap();
    }
    assert_eq!(out, std::fs::read_to_string(&path).unwrap());
}
