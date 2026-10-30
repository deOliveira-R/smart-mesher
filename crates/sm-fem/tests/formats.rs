//! D1 wiring: Exodus and OpenFOAM export, `export_files`, polyhedral element types, `.smsh`
//! element-type declarations.

mod common;

use common::*;
use sm_exodus::read_exodus;
use sm_fem::{
    BlockAdd, BlockElementType, ElementType, ExportFormat, ExportRequest, FemError, export,
    export_files,
};
use sm_meshio::SmshFile;
use sm_meshio::foam::{FoamFile, FoamOptions, build_polymesh, parse_polymesh};
use sm_testkit::meshes::{dual_patch, mixed_transition};

fn fixture_rig(name: &str) -> Rig {
    let mut r = Rig::new();
    r.apply(FixtureMesh { name: name.into() });
    r
}

fn exodus(r: &Rig) -> Vec<u8> {
    let mut buf = Vec::new();
    export(
        r.state(),
        &ExportRequest::new(ExportFormat::Exodus),
        &mut buf,
    )
    .expect("exodus");
    buf
}

fn files(r: &Rig, f: ExportFormat) -> sm_fem::ExportBundle {
    export_files(r.state(), &ExportRequest::new(f)).expect("bundle")
}

#[test]
fn exodus_of_mixed_transition_has_one_block_per_shape() {
    let r = fixture_rig("mixed_transition");
    let data = read_exodus(&exodus(&r)).expect("read");
    let mut types: Vec<(&str, usize)> = data
        .blocks
        .iter()
        .map(|b| (b.elem_type.as_str(), b.num_elem))
        .collect();
    types.sort();
    assert_eq!(
        types,
        vec![("HEX8", 1), ("PYRAMID5", 1), ("TET4", 1), ("WEDGE6", 1)]
    );
}

#[test]
fn exodus_of_dual_patch_has_nfaced_blocks() {
    let r = fixture_rig("dual_patch");
    let data = read_exodus(&exodus(&r)).expect("read");
    assert!(!data.blocks.is_empty());
    assert!(data.blocks.iter().all(|b| b.elem_type == "NFACED"));
    assert_eq!(data.blocks.iter().map(|b| b.num_elem).sum::<usize>(), 7);
}

#[test]
fn openfoam_files_parse_back_to_the_built_polymesh() {
    for name in ["mixed_transition", "dual_patch"] {
        let r = fixture_rig(name);
        let bundle = files(&r, ExportFormat::OpenFoam);
        let names: Vec<&str> = bundle.files.iter().map(|f| f.name.as_str()).collect();
        let mut sorted = names.clone();
        sorted.sort_unstable();
        assert_eq!(names, sorted, "{name}: sorted by name");
        for n in ["boundary", "faces", "neighbour", "owner", "points"] {
            assert!(
                names.contains(&format!("constant/polyMesh/{n}").as_str()),
                "{name}: {n} in {names:?}"
            );
        }
        let foam: Vec<FoamFile> = bundle
            .files
            .iter()
            .map(|f| FoamFile {
                name: f
                    .name
                    .strip_prefix("constant/polyMesh/")
                    .expect("prefix")
                    .into(),
                bytes: f.bytes.clone(),
            })
            .collect();
        let parsed = parse_polymesh(&foam).expect("parse");
        let fx = if name == "dual_patch" {
            dual_patch()
        } else {
            mixed_transition()
        }
        .expect("fixture");
        let built = build_polymesh(&fx.mesh, &FoamOptions::new()).expect("build");
        assert_eq!(parsed, built.mesh, "{name}");
    }
}

#[test]
fn bundle_and_exodus_bytes_are_identical_over_runs_and_threads() {
    for name in ["mixed_transition", "dual_patch"] {
        let r = fixture_rig(name);
        let run = |threads: usize| {
            let pool = rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .expect("pool");
            pool.install(|| {
                let b = files(&r, ExportFormat::OpenFoam);
                (b.files, b.notes, exodus(&r))
            })
        };
        let base = run(1);
        assert_eq!(base, run(1), "{name} repeat");
        assert_eq!(base, run(4), "{name} 4 threads");
        assert_eq!(base, run(0), "{name} default pool");
    }
}

#[test]
fn single_stream_formats_give_one_file_with_the_stream_bytes() {
    let r = fixture_rig("mixed_transition");
    let b = files(&r, ExportFormat::Exodus);
    assert_eq!(b.files.len(), 1);
    assert_eq!(b.files[0].name, "mesh.exo");
    assert_eq!(b.files[0].bytes, exodus(&r));
}

#[test]
fn openfoam_through_the_stream_api_is_multi_file() {
    let r = fixture_rig("mixed_transition");
    let mut sink = Vec::new();
    let e = export(
        r.state(),
        &ExportRequest::new(ExportFormat::OpenFoam),
        &mut sink,
    )
    .expect_err("multi file");
    assert!(matches!(e, FemError::MultiFile("openfoam")), "{e:?}");
    assert_eq!(e.code(), "multi-file-format");
    assert!(sink.is_empty());
    assert!(ExportFormat::OpenFoam.is_multi_file() && !ExportFormat::Exodus.is_multi_file());
    assert_eq!(ExportFormat::OpenFoam.name(), "openfoam");
    assert_eq!(ExportFormat::Exodus.name(), "exodus");
}

fn meshed_block() -> (Rig, Vec<sm_core::EntityId>) {
    let mut r = Rig::new();
    let d = r.apply(Bricks {
        origins: vec![[0.0; 3]],
        size: [2.0; 3],
    });
    let v = role(&d, "volume");
    r.apply(MeshIt {
        volumes: v.clone(),
        size: 1.0,
    });
    r.apply(BlockAdd {
        block: 1,
        volumes: v.clone(),
        faces: vec![],
        name: None,
    });
    (r, v)
}

#[test]
fn polyhedron_declared_on_a_hex_block_is_an_element_mismatch() {
    let (mut r, _) = meshed_block();
    r.apply(BlockElementType {
        block: 1,
        element_type: ElementType::Polyhedron,
    });
    for f in [ExportFormat::Smsh, ExportFormat::Exodus] {
        let mut sink = Vec::new();
        let e = export(r.state(), &ExportRequest::new(f), &mut sink).expect_err("mismatch");
        assert!(
            matches!(
                e,
                FemError::ElementMismatch {
                    declared: ElementType::Polyhedron,
                    ..
                }
            ),
            "{f:?}: {e:?}"
        );
    }
    assert_eq!(
        r.apply_code(BlockElementType {
            block: 1,
            element_type: ElementType::Polygon
        }),
        "unsupported"
    );
}

#[test]
fn smsh_header_lists_declared_element_types() {
    let (mut r, _) = meshed_block();
    let mut buf = Vec::new();
    export(r.state(), &ExportRequest::new(ExportFormat::Smsh), &mut buf).expect("smsh");
    let m = SmshFile::parse(&buf)
        .expect("parse")
        .decode()
        .expect("decode");
    assert!(m.element_types.is_empty(), "nothing declared");
    r.apply(BlockElementType {
        block: 1,
        element_type: ElementType::Hex8,
    });
    let mut buf = Vec::new();
    export(r.state(), &ExportRequest::new(ExportFormat::Smsh), &mut buf).expect("smsh");
    let m = SmshFile::parse(&buf)
        .expect("parse")
        .decode()
        .expect("decode");
    assert_eq!(
        m.element_types,
        vec![("block_1".to_string(), "HEX8".to_string())]
    );
}

#[test]
fn exodus_of_a_block_mesh_has_the_block_and_notes_are_lines() {
    let (r, _) = meshed_block();
    let data = read_exodus(&exodus(&r)).expect("read");
    assert_eq!(data.blocks.len(), 1);
    assert_eq!(data.blocks[0].elem_type, "HEX8");
    assert_eq!(data.blocks[0].num_elem, 8);
    let mut sink = Vec::new();
    let rep = export(r.state(), &ExportRequest::new(ExportFormat::Vtu), &mut sink).expect("vtu");
    assert!(
        rep.notes
            .iter()
            .all(|n| n.contains(": ") && n.contains(" x"))
    );
}
