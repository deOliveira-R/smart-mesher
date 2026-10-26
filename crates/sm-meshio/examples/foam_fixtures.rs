//! Write the OpenFOAM polyMesh of every `all_mesh_fixtures()` entry (plus a sets-bearing
//! 2x2x2 block and the non-convex polyhedron) as a case directory, for `checkMesh`.
//!
//! `cargo run -p sm-meshio --example foam_fixtures -- OUT` writes
//! `OUT/<name>/constant/polyMesh/*`, a minimal `OUT/<name>/system/controlDict`, and
//! `OUT/<name>/expected.json` (the `FoamReport`). `tools/conformance.sh` feeds the cases to
//! `checkMesh -allGeometry -allTopology`. Not run in worker CI.

use sm_meshio::foam::{FoamOptions, build_polymesh};
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::{CellIx, FaceIx, FaceKind, MeshEdit, MeshRead, SerialAlloc, SetMembers};
use sm_testkit::meshes::{all_mesh_fixtures, nonconvex_poly};
use std::path::Path;

const CONTROL_DICT: &str = "FoamFile\n{\n    version 2.0;\n    format ascii;\n    class dictionary;\n    object controlDict;\n}\n\napplication checkMesh;\nstartFrom startTime;\nstartTime 0;\nstopAt endTime;\nendTime 1;\ndeltaT 1;\nwriteControl timeStep;\nwriteInterval 1;\n";

fn write_case(out: &Path, name: &str, mesh: &dyn MeshRead) -> Result<(), Box<dyn std::error::Error>> {
    let b = build_polymesh(mesh, &FoamOptions::default())?;
    let case = out.join(name);
    b.mesh.write_dir(&case.join("constant/polyMesh"))?;
    std::fs::create_dir_all(case.join("system"))?;
    std::fs::write(case.join("system/controlDict"), CONTROL_DICT)?;
    std::fs::write(case.join("expected.json"), serde_json::to_string_pretty(&b.report)? + "\n")?;
    println!("{name}: {} points, {} cells, {} faces ({} internal), {} patches", b.report.n_points, b.report.n_cells, b.report.n_faces, b.report.n_internal_faces, b.report.patches.len());
    Ok(())
}

/// A 2x2x2 block with six side sets and two cell sets (zones).
fn sets_block() -> Result<sm_mesh_api::reference::ReferenceMesh, Box<dyn std::error::Error>> {
    let mut alloc = SerialAlloc::new();
    let mut m = hex_block([2, 2, 2], [0.5; 3], &mut alloc)?;
    for (i, name) in ["xmin", "xmax", "ymin", "ymax", "zmin", "zmax"].iter().enumerate() {
        let (axis, value) = (i / 2, (i % 2) as f64);
        let faces: Vec<FaceIx> = {
            let co = m.coords();
            let c = [co.x, co.y, co.z][axis];
            (0..m.face_count())
                .map(FaceIx::new)
                .filter(|f| m.face_kind(*f) == FaceKind::Boundary && m.face_vertices(*f).iter().all(|v| (c[v.get()] - value).abs() < 1e-12))
                .collect()
        };
        MeshEdit::add_set(&mut m, name, SetMembers::Faces(faces), &mut alloc)?;
    }
    MeshEdit::add_set(&mut m, "b1", SetMembers::Cells((0..4).map(CellIx::new).collect()), &mut alloc)?;
    MeshEdit::add_set(&mut m, "b2", SetMembers::Cells((4..8).map(CellIx::new).collect()), &mut alloc)?;
    Ok(m)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let out = std::env::args().nth(1).ok_or("usage: foam_fixtures OUT")?;
    let out = Path::new(&out);
    for f in all_mesh_fixtures()? {
        write_case(out, f.name, &f.mesh)?;
    }
    write_case(out, "nonconvex_poly", &nonconvex_poly()?.mesh)?;
    write_case(out, "hex_block_sets", &sets_block()?)?;
    Ok(())
}
