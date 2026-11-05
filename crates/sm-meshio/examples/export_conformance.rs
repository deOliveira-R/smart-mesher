//! Writes the DAGMC and Gmsh conformance inputs and their analytic expectations.
//!
//! `cargo run -p sm-meshio --example export_conformance -- OUTDIR` writes, per case, the file
//! and `<case>.expect.json` (read by `tools/conformance/check_h5m.py`, `check_msh4.py` and
//! `tools/conformance/dagmc/check_dagmc.py`). Until `smesh export` carries the formats
//! (W3b-EXPORT-WIRE) this is the stub the conformance driver calls.

use serde_json::{Value, json};
use sm_mesh_api::{CellIx, MeshRead, SetMembers};
use sm_meshio::dagmc::{DagmcOptions, Graveyard, export_h5m};
use sm_meshio::gmsh::{GmshOptions, write_msh4};
use sm_testkit::meshes::{MeshFixture, SetSpec, hex_block_two_volumes, mixed_transition, single_hex, single_pyramid, single_tet, single_wedge, tet_block_two_volumes, with_sets};
use std::path::Path;

fn cells_of(mesh: &dyn MeshRead, volume: u64) -> Vec<CellIx> {
    (0..mesh.cell_count()).map(CellIx::new).filter(|c| mesh.cell_class(*c).entity.serial() == volume).collect()
}

fn put(dir: &Path, name: &str, bytes: &[u8], expect: &Value) -> Result<(), Box<dyn std::error::Error>> {
    std::fs::write(dir.join(name), bytes)?;
    let stem = name.rsplit_once('.').map_or(name, |s| s.0);
    std::fs::write(dir.join(format!("{stem}.expect.json")), serde_json::to_string_pretty(expect)? + "\n")?;
    Ok(())
}

fn h5m_case(dir: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let base = tet_block_two_volumes([4, 4, 4], [0.25; 3], 2)?;
    let sets = [SetSpec::new("fuel", SetMembers::Cells(cells_of(&base.mesh, 1))), SetSpec::new("water", SetMembers::Cells(cells_of(&base.mesh, 2)))];
    let f = with_sets(tet_block_two_volumes([4, 4, 4], [0.25; 3], 2)?, &sets)?;
    let mut out = Vec::new();
    let o = DagmcOptions::with_graveyard(Graveyard::default());
    let r = export_h5m(&f.mesh, &o, &mut out)?;
    let expect = json!({
        "triangles": r.triangles, "surfaces": r.surfaces, "volumes": r.volumes, "groups": r.groups,
        "group_names": ["mat:Graveyard", "mat:fuel", "mat:water"],
        "boxes": {"1": {"lo": [0.0, 0.0, 0.0], "hi": [0.5, 1.0, 1.0]}, "2": {"lo": [0.5, 0.0, 0.0], "hi": [1.0, 1.0, 1.0]}},
        "graveyard": {"volume": 3, "inner": {"lo": [-0.05, -0.05, -0.05], "hi": [1.05, 1.05, 1.05]}, "outer": {"lo": [-0.105, -0.105, -0.105], "hi": [1.105, 1.105, 1.105]}},
        "model_volume": f.volume,
    });
    put(dir, "two_volume.h5m", &out, &expect)?;
    let h = hex_block_two_volumes([3, 2, 2], [0.5, 1.0, 2.0], 1)?;
    let mut out = Vec::new();
    let r = export_h5m(&h.mesh, &DagmcOptions::default(), &mut out)?;
    let expect = json!({"triangles": r.triangles, "surfaces": r.surfaces, "volumes": r.volumes, "groups": r.groups, "group_names": ["mat:volume_1", "mat:volume_2"],
        "boxes": {"1": {"lo": [0.0, 0.0, 0.0], "hi": [0.5, 2.0, 4.0]}, "2": {"lo": [0.5, 0.0, 0.0], "hi": [1.5, 2.0, 4.0]}}});
    put(dir, "hex_blocks.h5m", &out, &expect)?;
    Ok(())
}

fn msh_case(dir: &Path, name: &str, f: &MeshFixture, cells: Value, groups: Value) -> Result<(), Box<dyn std::error::Error>> {
    for binary in [false, true] {
        let mut out = Vec::new();
        let r = write_msh4(&f.mesh, &GmshOptions::default().with_binary(binary), &mut out)?;
        let expect = json!({"nodes": r.nodes, "volume": f.volume, "cells": cells, "physical_groups": groups, "binary": binary});
        put(dir, &format!("{name}{}.msh", if binary { "_bin" } else { "" }), &out, &expect)?;
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let dir = std::env::args().nth(1).ok_or("usage: export_conformance OUTDIR")?;
    let dir = Path::new(&dir);
    std::fs::create_dir_all(dir)?;
    h5m_case(dir)?;
    msh_case(dir, "single_hex", &single_hex()?, json!({"hexahedron": 1}), json!([]))?;
    msh_case(dir, "single_tet", &single_tet()?, json!({"tetra": 1}), json!([]))?;
    msh_case(dir, "single_wedge", &single_wedge()?, json!({"wedge": 1}), json!([]))?;
    msh_case(dir, "single_pyramid", &single_pyramid()?, json!({"pyramid": 1}), json!([]))?;
    msh_case(dir, "mixed", &mixed_transition()?, json!({"hexahedron": 1, "wedge": 1, "pyramid": 1, "tetra": 1}), json!([]))?;
    msh_case(dir, "hex_two_volume", &hex_block_two_volumes([2, 2, 2], [1.0; 3], 1)?, json!({"hexahedron": 8}), json!([[3, 1, "volume_1"], [3, 2, "volume_2"]]))?;
    println!("wrote the conformance inputs to {}", dir.display());
    Ok(())
}
