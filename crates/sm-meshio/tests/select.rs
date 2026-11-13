//! Export selection: counts, byte identity with a mesh built from only the selection, warnings,
//! errors and the empty selection.
#![allow(missing_docs)]

#[path = "common/select_rig.rs"]
mod rig;

use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{
    CellIx, CellShape, FaceIx, MeshBuild, MeshEntityRef, MeshRead, SerialAlloc, SetMembers, VertexIx,
};
use sm_meshio::gmsh::{GmshOptions, write_msh4};
use sm_meshio::select::{Dimension, EMPTY_SELECTION, Pick, Roles, Select, SelectError, apply};
use sm_meshio::{VtuOptions, write_vtu};
use sm_testkit::meshes::{SetSpec, with_sets};

fn names(m: &dyn MeshRead) -> Vec<String> {
    m.sets().iter().map(|s| s.name.clone()).collect()
}

fn blocks(list: &[&str]) -> Select {
    Select {
        blocks: Pick::Names(list.iter().map(|s| (*s).to_owned()).collect()),
        ..Select::default()
    }
}

fn vtu(m: &dyn MeshRead) -> Vec<u8> {
    let mut out = Vec::new();
    write_vtu(m, &VtuOptions::default(), &mut out).expect("vtu");
    out
}

fn gmsh(m: &dyn MeshRead) -> Vec<u8> {
    let mut out = Vec::new();
    write_msh4(m, &GmshOptions::default(), &mut out).expect("gmsh");
    out
}

/// A mesh built from only `kept` cells of `host`: their nodes in original order, template cells,
/// the host's classes, then the row's sets (`names` blocks, plane sidesets except `skip`, `nx0`).
fn oracle_from(
    host: &dyn MeshRead,
    kept: &[usize],
    names: &[&str],
    skip: &[&str],
) -> sm_mesh_api::reference::ReferenceMesh {
    let mut verts: Vec<u32> = kept
        .iter()
        .flat_map(|&c| host.cell_corners(CellIx::new(c)).expect("hex").iter().map(|v| v.0))
        .collect();
    verts.sort_unstable();
    verts.dedup();
    let new_of = |v: VertexIx| VertexIx(verts.binary_search(&v.0).expect("kept") as u32);
    let mut b = ReferenceBuilder::new();
    for &v in &verts {
        let nv = b.add_vertex(host.point(v as usize)).expect("vertex");
        b.set_class(MeshEntityRef::Vertex(nv), host.vertex_class(VertexIx(v))).expect("class");
    }
    for &c in kept {
        let ci = CellIx::new(c);
        let corners: Vec<VertexIx> =
            host.cell_corners(ci).expect("hex").iter().map(|&v| new_of(v)).collect();
        let added = b.add_cell_from_vertices(CellShape::Hex, &corners).expect("cell");
        b.set_class(MeshEntityRef::Cell(added.cell), host.cell_class(ci)).expect("class");
    }
    for sf in kept.iter().flat_map(|&c| host.cell_faces(CellIx::new(c))) {
        let f = sf.face();
        let lp: Vec<VertexIx> = host.face_vertices(f).iter().map(|&v| new_of(v)).collect();
        let (nf, _) = b.find_face(&lp).expect("face");
        b.set_class(MeshEntityRef::Face(nf), host.face_class(f)).expect("class");
    }
    let mesh = b.finish(&mut SerialAlloc::new()).expect("finish").mesh;
    let mut specs = rig::block_sets(&[1, 2], names);
    specs.extend(
        rig::plane_sidesets(&mesh)
            .into_iter()
            .filter(|s| !skip.contains(&s.name.as_str())),
    );
    specs.push(rig::nodeset_x0(&mesh));
    with_sets(rig::fixture(mesh, kept.len()), &specs).expect("sets").mesh
}

#[test]
fn two_of_three_blocks_keep_exactly_their_cells_and_nodes() {
    let host = rig::row(&[1, 2, 3], &["A", "B", "C"]);
    let sel = apply(&host, &blocks(&["A", "B"]), &Roles::by_kind()).expect("select");
    let m = &sel.mesh;
    // Block sizes 1 + 1 cells; a row of 2 hexes has 3 x 4 = 12 nodes and 11 faces.
    assert_eq!(m.cell_count(), 2);
    assert_eq!(m.vertex_count(), 12);
    assert_eq!(m.face_count(), 11);
    // Unit cubes in a row: the bounding box of the kept nodes is 2 x 1 x 1 (volume 2).
    let ext = |f: fn(sm_core::Point3) -> f64| {
        let v: Vec<f64> = (0..m.vertex_count()).map(|i| f(m.point(i))).collect();
        v.iter().cloned().fold(f64::MIN, f64::max) - v.iter().cloned().fold(f64::MAX, f64::min)
    };
    assert_eq!([ext(|p| p.x), ext(|p| p.y), ext(|p| p.z)], [2.0, 1.0, 1.0]);
    // Sets: blocks A and B, the sidesets that still have faces, the nodeset.
    assert_eq!(names(m), ["A", "B", "xmin", "ymin", "ymax", "zmin", "zmax", "nx0"]);
    // xmax lives on the dropped cell; it was not named, so there is no report.
    assert!(sel.report.is_lossless());
    // The cut face of B is now a boundary face: 10 of 11 faces.
    let boundary = (0..m.face_count())
        .filter(|&f| m.face_right(FaceIx::new(f)).is_none())
        .count();
    assert_eq!(boundary, 10);
}

#[test]
fn the_selection_is_byte_identical_to_a_mesh_built_from_only_those_cells() {
    let host = rig::row(&[1, 2, 3], &["A", "B", "C"]);
    let sel = apply(&host, &blocks(&["A", "B"]), &Roles::by_kind()).expect("select");
    // The oracle is built from scratch with the reference builder from the kept cells' nodes (in
    // original order) and the host's classes; the plane x = 2 is a new boundary there, so its
    // sideset is left out (the selection has no `xmax` either).
    let oracle = oracle_from(&host, &[0, 1], &["A", "B"], &["xmax"]);
    assert_eq!(names(&sel.mesh), names(&oracle));
    assert_eq!(vtu(&sel.mesh), vtu(&oracle), "vtu");
    assert_eq!(gmsh(&sel.mesh), gmsh(&oracle), "gmsh");
    let smsh = |m: &dyn MeshRead| {
        let mut out = Vec::new();
        sm_meshio::SmshWriter::new(m).write_to(&mut out).expect("smsh");
        out
    };
    assert_eq!(smsh(&sel.mesh), smsh(&oracle), "smsh");
    let dagmc = |m: &dyn MeshRead| {
        let mut out = Vec::new();
        sm_meshio::dagmc::export_h5m(m, &sm_meshio::dagmc::DagmcOptions::default(), &mut out)
            .expect("dagmc");
        out
    };
    assert_eq!(dagmc(&sel.mesh), dagmc(&oracle), "dagmc");
}

#[test]
fn a_named_but_empty_sideset_warns_and_an_unknown_name_lists_the_choices() {
    let host = rig::row(&[1, 2, 3], &["A", "B", "C"]);
    let sel = Select {
        blocks: Pick::Names(vec!["A".into(), "B".into()]),
        sidesets: Pick::Names(vec!["xmin".into(), "xmax".into()]),
        ..Select::default()
    };
    let out = apply(&host, &sel, &Roles::by_kind()).expect("select");
    // Only the named sideset that has faces is written, plus the nodeset.
    assert_eq!(names(&out.mesh), ["A", "B", "xmin", "nx0"]);
    let item = out.report.get(EMPTY_SELECTION).expect("warning");
    assert!(item.detail.contains("xmax"), "{}", item.detail);
    assert_eq!(out.report.items.len(), 1);

    let bad = Select {
        sidesets: Pick::Names(vec!["nope".into()]),
        ..Select::default()
    };
    match apply(&host, &bad, &Roles::by_kind()) {
        Err(SelectError::UnknownSet {
            role,
            name,
            available,
        }) => {
            assert_eq!((role, name.as_str()), ("sideset", "nope"));
            assert_eq!(available, ["xmin", "xmax", "ymin", "ymax", "zmin", "zmax"]);
        }
        other => panic!("{other:?}"),
    }
    // A cell set is not a nodeset.
    let bad = Select {
        nodesets: Pick::Names(vec!["A".into()]),
        ..Select::default()
    };
    assert!(matches!(
        apply(&host, &bad, &Roles::by_kind()),
        Err(SelectError::UnknownSet {
            role: "nodeset",
            ..
        })
    ));
}

#[test]
fn the_default_selection_keeps_everything_and_dimension_is_checked() {
    let host = rig::row(&[1, 2, 3], &["A", "B", "C"]);
    let all = apply(&host, &Select::default(), &Roles::by_kind()).expect("select");
    assert_eq!(all.mesh.cell_count(), 3);
    assert_eq!(names(&all.mesh), names(&host));
    assert_eq!(vtu(&all.mesh), vtu(&host), "an all-selection writes the same VTU");
    let two = Select {
        dimension: Dimension::Two,
        ..Select::default()
    };
    assert!(matches!(
        apply(&host, &two, &Roles::by_kind()),
        Err(SelectError::Dimension { asked: 2, mesh: 3 })
    ));
    let three = Select {
        dimension: Dimension::Three,
        ..Select::default()
    };
    assert!(apply(&host, &three, &Roles::by_kind()).is_ok());
    let none = Select {
        blocks: Pick::Names(vec![]),
        ..Select::default()
    };
    assert!(matches!(
        apply(&host, &none, &Roles::by_kind()),
        Err(SelectError::Empty)
    ));
}

#[test]
fn companion_sets_follow_the_cells_and_keep_their_names() {
    // A cell set that is not a block (a region): restricted to what is kept, dropped when empty.
    let host = rig::row(&[1, 2, 3], &["A", "B", "C"]);
    let specs = [
        SetSpec::new("region/a", SetMembers::Cells(vec![CellIx::new(0)])),
        SetSpec::new("region/c", SetMembers::Cells(vec![CellIx::new(2)])),
    ];
    let host = with_sets(rig::fixture(host, 3), &specs).expect("sets").mesh;
    let roles = Roles::by_kind().companion("region/a").companion("region/c");
    let out = apply(&host, &blocks(&["A", "B"]), &roles).expect("select");
    assert!(names(&out.mesh).contains(&"region/a".to_owned()));
    assert!(!names(&out.mesh).contains(&"region/c".to_owned()));
    assert!(out.report.is_lossless(), "an unnamed companion going empty is not reported");
    // By kind alone the region sets are blocks, and unselected ones are dropped.
    let out = apply(&host, &blocks(&["A", "B"]), &Roles::by_kind()).expect("select");
    assert!(!names(&out.mesh).contains(&"region/a".to_owned()));
}

#[test]
fn selection_is_deterministic_across_threads() {
    let host = rig::row(&[1, 2, 3, 2, 1], &["A", "B", "C"]);
    let run = || {
        let s = apply(&host, &blocks(&["B", "A"]), &Roles::by_kind()).expect("select");
        (vtu(&s.mesh), gmsh(&s.mesh))
    };
    let a = run();
    assert_eq!(a, run());
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(1)
        .build()
        .expect("pool");
    assert_eq!(a, pool.install(run));
}
