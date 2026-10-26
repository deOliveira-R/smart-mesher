//! OpenFOAM polyMesh export: counts, orientation, ordering, patches and zones, golden bytes.

mod common;

use common::foam::*;
use common::wrap::Wrap;
use sm_core::EntityKind;
use sm_meshio::foam::*;
use sm_meshio::{LossSeverity, MeshioError};
use sm_mesh_api::reference::{ReferenceMesh, hex_block};
use sm_mesh_api::*;
use sm_testkit::meshes::*;

fn fixtures() -> Vec<MeshFixture> {
    let mut v = all_mesh_fixtures().unwrap();
    v.push(nonconvex_poly().unwrap());
    v
}

/// Add a face set (the borrow checker does not allow `faces_on(&m, ..)` inside the call).
fn add_faces(m: &mut ReferenceMesh, name: &str, faces: Vec<FaceIx>, alloc: &mut SerialAlloc) {
    MeshEdit::add_set(m, name, SetMembers::Faces(faces), alloc).unwrap();
}

fn build(m: &dyn MeshRead) -> FoamBuild {
    build_polymesh(m, &FoamOptions::default()).unwrap()
}

fn foam_err(r: sm_meshio::Result<FoamBuild>) -> FoamError {
    match r {
        Err(MeshioError::Foam(e)) => e,
        Err(e) => panic!("expected a FoamError, got {e}"),
        Ok(_) => panic!("expected an error"),
    }
}

// ---- 1. counts ----------------------------------------------------------------------------------

#[test]
fn counts_of_the_named_fixtures() {
    let cases: [(MeshFixture, [u64; 4]); 6] = [
        (hex_block_2x2x2().unwrap(), [27, 8, 36, 12]),
        (mixed_transition().unwrap(), [12, 4, 17, 3]),
        (dual_patch().unwrap(), [48, 7, 44, 12]),
        (hexagonal_prism(1.0, 2.0).unwrap(), [12, 1, 8, 0]),
        (single_hex().unwrap(), [8, 1, 6, 0]),
        (single_tet().unwrap(), [4, 1, 4, 0]),
    ];
    for (f, [np, nc, nf, ni]) in cases {
        let b = build(&f.mesh);
        let r = &b.report;
        assert_eq!((r.n_points, r.n_cells, r.n_faces, r.n_internal_faces), (np, nc, nf, ni), "{}", f.name);
        assert_eq!(nf - ni, f.boundary_faces as u64, "{}", f.name);
        assert_eq!(b.mesh.n_faces() as u64 - b.mesh.n_internal() as u64, f.boundary_faces as u64);
    }
}

#[test]
fn every_fixture_counts_its_entities() {
    for f in fixtures() {
        let m = &f.mesh;
        let b = build(m);
        assert_eq!(b.mesh.points.len(), m.count(MeshEntityKind::Vertex), "{}", f.name);
        assert_eq!(b.mesh.n_cells as usize, m.count(MeshEntityKind::Cell), "{}", f.name);
        assert_eq!(b.mesh.n_faces(), m.count(MeshEntityKind::Face), "{}", f.name);
        assert_eq!(b.report.n_points as usize, b.mesh.points.len());
    }
}

#[test]
fn shape_counts_follow_cell_shape() {
    let b = build(&mixed_transition().unwrap().mesh);
    let s = b.report.shape_counts;
    assert_eq!((s.hexahedra, s.prisms, s.pyramids, s.tetrahedra, s.polyhedra), (1, 1, 1, 1, 0));
    // The seven hexagonal prisms have 8 faces: polyhedra to checkMesh as well.
    let d = dual_patch().unwrap();
    assert!((0..7).all(|c| d.mesh.cell_shape(CellIx::new(c)) == CellShape::Polyhedron));
    let s = build(&d.mesh).report.shape_counts;
    assert_eq!((s.hexahedra, s.prisms, s.pyramids, s.tetrahedra, s.polyhedra), (0, 0, 0, 0, 7));
    // A true 6-node prism is five faces (2 tri + 3 quad), no wedge handling.
    let w = single_wedge().unwrap();
    let b = build(&w.mesh);
    let mut sizes: Vec<usize> = (0..5).map(|i| b.mesh.face(i).len()).collect();
    sizes.sort_unstable();
    assert_eq!(sizes, [3, 3, 4, 4, 4]);
    assert_eq!(b.report.shape_counts.prisms, 1);
    // Every fixture's census adds up to its cell count.
    for f in fixtures() {
        let s = build(&f.mesh).report.shape_counts;
        assert_eq!(s.hexahedra + s.prisms + s.pyramids + s.tetrahedra + s.polyhedra, f.mesh.cell_count() as u64, "{}", f.name);
    }
}

// ---- 2. round trip ------------------------------------------------------------------------------

#[test]
fn files_parse_back_to_the_same_mesh_bit_for_bit() {
    for f in fixtures() {
        let b = build(&f.mesh);
        let files = b.mesh.files().unwrap();
        let back = parse_polymesh(&files).unwrap();
        assert_eq!(back, b.mesh, "{}", f.name);
        for (p, q) in back.points.iter().zip(&b.mesh.points) {
            for d in 0..3 {
                assert_eq!(p[d].to_bits(), q[d].to_bits(), "{}", f.name);
            }
        }
        let names: Vec<&str> = files.iter().map(|f| f.name.as_str()).collect();
        assert_eq!(names, ["points", "faces", "owner", "neighbour", "boundary"], "no zones, no zone files");
    }
}

#[test]
fn awkward_scalars_survive_the_round_trip() {
    let mut b = build(&single_hex().unwrap().mesh);
    let vals = [0.1, -0.0, 1e-300, 5e-324, f64::MAX, 1.0 / 3.0, 123_456_789.123_456_79, -2.5e-7, 1e21, 1e16];
    for (i, v) in vals.iter().enumerate() {
        b.mesh.points[i % 8][i % 3] = *v;
    }
    let back = parse_polymesh(&b.mesh.files().unwrap()).unwrap();
    for (p, q) in back.points.iter().zip(&b.mesh.points) {
        for d in 0..3 {
            assert_eq!(p[d].to_bits(), q[d].to_bits());
        }
    }
    // NaN and infinity are refused, not written.
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut m = b.mesh.clone();
        m.points[2][1] = bad;
        assert!(matches!(m.files(), Err(MeshioError::Foam(FoamError::NonFinite { point: 2 }))));
    }
}

// ---- 3. orientation and geometry ------------------------------------------------------------------

#[test]
fn owner_outward_closure_and_volumes_match_the_fixtures() {
    for f in fixtures() {
        let b = build(&f.mesh);
        let stats = cell_stats(&b.mesh);
        assert_eq!(stats.len(), f.cell_volumes.len());
        for (c, (vol, resid, abs)) in stats.iter().enumerate() {
            assert!(*resid <= 1e-12 * abs, "{} cell {c}: closure {resid:e} of {abs:e}", f.name);
            assert!((vol - f.cell_volumes[c]).abs() <= 1e-12 * f.cell_volumes[c], "{} cell {c}: {vol} vs {}", f.name, f.cell_volumes[c]);
        }
    }
    let d = dual_patch().unwrap();
    let v = cell_stats(&build(&d.mesh).mesh);
    assert!(v.iter().all(|s| (s.0 - 3.0 * 3f64.sqrt() / 2.0).abs() <= 1e-12 * 2.6));
}

#[test]
fn checker_is_clean_on_every_fixture_and_the_input_validates() {
    for f in fixtures() {
        let issues = check_polymesh(&build(&f.mesh).mesh, 1e-9);
        assert!(issues.is_empty(), "{}: {issues:?}", f.name);
        let v = validate(&f.mesh);
        assert!(v.is_empty(), "{}: {v:?}", f.name);
    }
}

// ---- 4. ordering --------------------------------------------------------------------------------

fn check_ordering(name: &str, src: &dyn MeshRead, b: &FoamBuild, strict: bool) {
    let m = &b.mesh;
    let ni = m.n_internal();
    for i in 0..ni {
        assert!(m.owner[i] < m.neighbour[i], "{name}");
    }
    for i in 1..ni {
        let (a, c) = ((m.owner[i - 1], m.neighbour[i - 1]), (m.owner[i], m.neighbour[i]));
        assert!(if strict { a < c } else { a <= c }, "{name}: faces {} and {i}", i - 1);
    }
    let mut next = ni as u32;
    for p in &m.patches {
        assert_eq!(p.start_face, next, "{name}");
        next += p.n_faces;
    }
    assert_eq!(next as usize, m.n_faces());
    assert_eq!(m.patches.iter().map(|p| p.n_faces as usize).sum::<usize>(), m.n_faces() - ni);
    let mut seen = vec![false; src.face_count()];
    for &f in &b.face_of_foam {
        assert!(!std::mem::replace(&mut seen[f as usize], true), "{name}: face_of_foam repeats {f}");
    }
    assert!(seen.iter().all(|s| *s));
    assert_eq!(b.face_of_foam.len(), m.n_faces());
    for (i, &f) in b.face_of_foam.iter().enumerate() {
        let mut a: Vec<u32> = m.face(i).iter().map(|p| b.point_of_foam[*p as usize]).collect();
        let mut c: Vec<u32> = src.face_vertices(FaceIx(f)).iter().map(|v| v.0).collect();
        a.sort_unstable();
        c.sort_unstable();
        assert_eq!(a, c, "{name}: face {i}");
        assert_eq!(m.owner[i], src.face_left(FaceIx(f)).0, "{name}");
        if i < ni {
            assert_eq!(src.face_kind(FaceIx(f)), FaceKind::Interior);
            assert_eq!(Some(CellIx(m.neighbour[i])), src.face_right(FaceIx(f)), "{name}");
        } else {
            assert_ne!(src.face_kind(FaceIx(f)), FaceKind::Interior);
        }
    }
}

#[test]
fn ordering_rules_on_every_fixture() {
    for f in fixtures() {
        // No two cells of any fixture share two faces.
        check_ordering(f.name, &f.mesh, &build(&f.mesh), true);
    }
}

#[test]
fn interleaved_boundary_faces_are_moved_behind_the_interior_ones() {
    let m = hex_block([2, 2, 2], [0.5; 3], &mut SerialAlloc::new()).unwrap();
    // Natural order: cell 0 already has boundary faces between faces of later cells.
    let b = build(&m);
    assert!(b.mesh.n_internal() == 12);
    let natural_interleaved = (1..m.face_count()).any(|f| m.face_kind(FaceIx::new(f - 1)) == FaceKind::Boundary && m.face_kind(FaceIx::new(f)) == FaceKind::Interior);
    assert!(natural_interleaved, "the canonical order of the block interleaves boundary and interior faces");
    check_ordering("block", &m, &b, true);

    // Hand-built: put a boundary face of cell 0 between two of its interior faces.
    let int0: Vec<usize> = (0..m.face_count()).filter(|&f| m.face_left(FaceIx::new(f)) == CellIx(0) && m.face_kind(FaceIx::new(f)) == FaceKind::Interior).collect();
    assert_eq!(int0.len(), 3);
    let bnd0 = (0..m.face_count()).find(|&f| m.face_left(FaceIx::new(f)) == CellIx(0) && m.face_kind(FaceIx::new(f)) == FaceKind::Boundary).unwrap();
    let mut order: Vec<usize> = (0..m.face_count()).filter(|f| *f != bnd0).collect();
    let at = order.iter().position(|f| *f == int0[1]).unwrap();
    order.insert(at, bnd0);
    let w = Wrap::new(&m).permute(order);
    let k = |i: usize| w.face_kind(FaceIx::new(i));
    let first_int = (0..w.face_count()).find(|&i| k(i) == FaceKind::Interior).unwrap();
    let second_int = (first_int + 1..w.face_count()).find(|&i| k(i) == FaceKind::Interior).unwrap();
    assert_eq!(k(first_int + 1), FaceKind::Boundary, "boundary face between two interior faces of cell 0");
    assert_eq!(second_int, first_int + 2);
    let b = build(&w);
    check_ordering("interleaved", &w, &b, true);
    assert_eq!(b.mesh.n_internal(), 12);
    // The interior faces kept their relative (already sorted) order.
    let internal_src: Vec<usize> = b.face_of_foam[..12].iter().map(|f| w.old(*f as usize)).collect();
    let mut sorted = internal_src.clone();
    sorted.sort_unstable();
    assert_eq!(internal_src, sorted);
}

#[test]
fn unsorted_interior_faces_are_sorted_by_owner_then_neighbour() {
    let m = hex_block([2, 2, 2], [0.5; 3], &mut SerialAlloc::new()).unwrap();
    let mut order: Vec<usize> = (0..m.face_count()).collect();
    // Reverse the whole face list: now the interior faces are in descending order.
    order.reverse();
    let w = Wrap::new(&m).permute(order);
    let b = build(&w);
    check_ordering("reversed", &w, &b, true);
    assert_eq!(b.mesh.n_internal(), 12);
    // The geometry is unaffected by the reordering.
    for (c, (vol, resid, abs)) in cell_stats(&b.mesh).iter().enumerate() {
        assert!((vol - 0.125).abs() < 1e-15 && *resid < 1e-12 * abs, "cell {c}");
    }
    assert!(check_polymesh(&b.mesh, 1e-9).is_empty());
}

#[test]
fn an_interior_face_with_left_not_below_right_is_an_orientation_error() {
    let m = hex_block([2, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let i = (0..m.face_count()).find(|&f| m.face_kind(FaceIx::new(f)) == FaceKind::Interior).unwrap();
    let w = Wrap::new(&m).swap_cells(i);
    assert!(matches!(foam_err(build_polymesh(&w, &FoamOptions::default())), FoamError::Orientation { face } if face as usize == i));
}

// ---- 5. patches and zones -------------------------------------------------------------------------

#[test]
fn face_sets_become_patches_in_set_order() {
    let m = sets_block();
    let b = build(&m);
    let names: Vec<&str> = b.mesh.patches.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names, SIDES);
    for (k, p) in b.mesh.patches.iter().enumerate() {
        assert_eq!(p.n_faces, 4);
        assert_eq!(p.start_face as usize, 12 + 4 * k);
        assert_eq!(p.origin, PatchOrigin::Set(SIDES[k].into()));
        // Every face of the patch is a source face of that set.
        let set = m.set_by_name(SIDES[k]).unwrap();
        for i in p.start_face..p.start_face + p.n_faces {
            assert!(set.members.contains(b.face_of_foam[i as usize]));
        }
    }
    assert!(b.report.loss.get("foam.default_patch").is_none());
    assert!(check_polymesh(&b.mesh, 1e-9).is_empty());
    assert_eq!(b.report.patches.len(), 6);
}

#[test]
fn cell_sets_become_cell_zones() {
    let b = build(&sets_block());
    let z: Vec<(&str, &[u32])> = b.mesh.cell_zones.iter().map(|z| (z.name.as_str(), z.labels.as_slice())).collect();
    assert_eq!(z, [("b1", &[0u32, 1, 2, 3][..]), ("b2", &[4, 5, 6, 7][..])]);
    let files = b.mesh.files().unwrap();
    assert_eq!(files.last().unwrap().name, "cellZones");
    assert_eq!(parse_polymesh(&files).unwrap(), b.mesh);
    // Selection by name, and none.
    let only = build_polymesh(&sets_block(), &FoamOptions::new().cell_zones(ZoneSelect::Names(vec!["b2".into()]))).unwrap();
    assert_eq!(only.mesh.cell_zones.len(), 1);
    assert_eq!(only.mesh.cell_zones[0].labels, [4, 5, 6, 7]);
    let none = build_polymesh(&sets_block(), &FoamOptions::new().cell_zones(ZoneSelect::None)).unwrap();
    assert!(none.mesh.cell_zones.is_empty());
    assert_eq!(none.mesh.files().unwrap().len(), 5);
    let r = build_polymesh(&sets_block(), &FoamOptions::new().cell_zones(ZoneSelect::Names(vec!["nope".into()])));
    assert!(matches!(foam_err(r), FoamError::UnknownSet(n) if n == "nope"));
    // A face set name is not a cell set name.
    let r = build_polymesh(&sets_block(), &FoamOptions::new().cell_zones(ZoneSelect::Names(vec!["xmin".into()])));
    assert!(matches!(foam_err(r), FoamError::UnknownSet(_)));
}

#[test]
fn an_unlisted_boundary_face_lands_in_default_faces() {
    // The six side sets, but xmin lacks one face.
    let mut m = hex_block([2, 2, 2], [0.5; 3], &mut SerialAlloc::new()).unwrap();
    let mut alloc = SerialAlloc::new();
    for (i, n) in SIDES.iter().enumerate() {
        let mut f = faces_on(&m, i / 2, (i % 2) as f64);
        if i == 0 {
            f.pop();
        }
        MeshEdit::add_set(&mut m, n, SetMembers::Faces(f), &mut alloc).unwrap();
    }
    let b = build(&m);
    let l = b.report.loss.get("foam.default_patch").expect("default patch loss");
    assert_eq!((l.count, l.severity), (1, LossSeverity::Info));
    let last = b.mesh.patches.last().unwrap();
    assert_eq!((last.name.as_str(), last.patch_type, last.origin.clone(), last.n_faces), ("defaultFaces", FoamPatchType::Patch, PatchOrigin::Default, 1));
    assert_eq!(b.mesh.patches.len(), 7);
    assert_eq!(b.mesh.patches[0].n_faces, 3);
    check_ordering("default", &m, &b, true);
    // Renamed and retyped default patch.
    let b = build_polymesh(&m, &FoamOptions::new().default_patch("rest").default_patch_type(FoamPatchType::Wall)).unwrap();
    let last = b.mesh.patches.last().unwrap();
    assert_eq!((last.name.as_str(), last.patch_type), ("rest", FoamPatchType::Wall));
    // Without any set every boundary face is in the default patch.
    let bare = hex_block([2, 2, 2], [0.5; 3], &mut SerialAlloc::new()).unwrap();
    let b = build(&bare);
    assert_eq!(b.mesh.patches.len(), 1);
    assert_eq!(b.report.loss.get("foam.default_patch").unwrap().count, 24);
}

fn overlapping_block() -> ReferenceMesh {
    let mut m = hex_block([2, 2, 2], [0.5; 3], &mut SerialAlloc::new()).unwrap();
    let mut alloc = SerialAlloc::new();
    let a = faces_on(&m, 0, 0.0);
    let mut b = faces_on(&m, 1, 0.0);
    b.push(a[0]); // a face in both
    b.sort();
    MeshEdit::add_set(&mut m, "a", SetMembers::Faces(a), &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, "b", SetMembers::Faces(b), &mut alloc).unwrap();
    m
}

#[test]
fn overlapping_sets_report_in_sets_mode_and_fail_when_explicit() {
    let m = overlapping_block();
    let b = build(&m);
    let l = b.report.loss.get("foam.patch_overlap").unwrap();
    assert_eq!(l.count, 1);
    assert_eq!(b.mesh.patches[0].n_faces, 4);
    assert_eq!(b.mesh.patches[1].n_faces, 4, "the first set keeps the shared face, b loses it");
    let opts = FoamOptions::new().patches(PatchSource::Explicit(vec![PatchSpec::new("a"), PatchSpec::new("b")]));
    match foam_err(build_polymesh(&m, &opts)) {
        FoamError::PatchOverlap { a, b, .. } => assert_eq!((a.as_str(), b.as_str()), ("a", "b")),
        e => panic!("{e}"),
    }
    // Disjoint listing is fine, in listed order with new names and types.
    let m = sets_block();
    let opts = FoamOptions::new().patches(PatchSource::Explicit(vec![
        PatchSpec::new("xmax").named("inlet").of_type(FoamPatchType::Wall),
        PatchSpec::new("xmin"),
    ]));
    let b = build_polymesh(&m, &opts).unwrap();
    let p: Vec<(&str, FoamPatchType)> = b.mesh.patches.iter().map(|p| (p.name.as_str(), p.patch_type)).collect();
    assert_eq!(p, [("inlet", FoamPatchType::Wall), ("xmin", FoamPatchType::Patch), ("defaultFaces", FoamPatchType::Patch)]);
    assert_eq!(b.report.loss.get("foam.default_patch").unwrap().count, 16);
    assert!(matches!(foam_err(build_polymesh(&m, &FoamOptions::new().patches(PatchSource::Explicit(vec![PatchSpec::new("zzz")])))), FoamError::UnknownSet(_)));
    // A cell set cannot be a patch.
    assert!(matches!(foam_err(build_polymesh(&m, &FoamOptions::new().patches(PatchSource::Explicit(vec![PatchSpec::new("b1")])))), FoamError::IneligibleSet { .. }));
}

#[test]
fn classification_gives_one_patch_per_model_face() {
    let m = hex_block([2, 2, 2], [0.5; 3], &mut SerialAlloc::new()).unwrap();
    let b = build_polymesh(&m, &FoamOptions::new().patches(PatchSource::Classification)).unwrap();
    assert_eq!(b.mesh.patches.len(), 6);
    let serials: Vec<u64> = b.mesh.patches.iter().map(|p| p.name.strip_prefix("topo_face_").unwrap().parse().unwrap()).collect();
    assert!(serials.windows(2).all(|w| w[0] < w[1]), "ascending serial");
    for p in &b.mesh.patches {
        assert_eq!(p.n_faces, 4);
        let PatchOrigin::Classification(id) = &p.origin else { panic!() };
        assert_eq!(id.kind(), EntityKind::TopoFace);
        for i in p.start_face..p.start_face + p.n_faces {
            assert_eq!(m.face_class(FaceIx(b.face_of_foam[i as usize])).entity, *id);
        }
    }
    assert!(b.report.loss.get("foam.default_patch").is_none());
    // Sets are ignored in this mode.
    let b = build_polymesh(&sets_block(), &FoamOptions::new().patches(PatchSource::Classification)).unwrap();
    assert_eq!(b.mesh.patches.len(), 6);
    assert!(b.mesh.patches.iter().all(|p| p.name.starts_with("topo_face_")));
}

#[test]
fn non_conformal_and_partition_cut_faces_go_to_their_own_patches() {
    let m = hex_block([2, 2, 2], [0.5; 3], &mut SerialAlloc::new()).unwrap();
    let bnd: Vec<usize> = (0..m.face_count()).filter(|&f| m.face_kind(FaceIx::new(f)) == FaceKind::Boundary).collect();
    let w = Wrap::new(&m).kind(bnd[0], FaceKind::NonConformal).kind(bnd[1], FaceKind::NonConformal).kind(bnd[2], FaceKind::PartitionCut);
    let b = build(&w);
    let p: Vec<(&str, u32, PatchOrigin)> = b.mesh.patches.iter().map(|p| (p.name.as_str(), p.n_faces, p.origin.clone())).collect();
    assert_eq!(
        p,
        [("defaultFaces", 21, PatchOrigin::Default), ("nonConformal", 2, PatchOrigin::NonConformal), ("partitionCut", 1, PatchOrigin::PartitionCut)]
    );
    assert_eq!(b.report.loss.get("foam.nonconformal_as_patch").unwrap().count, 2);
    assert_eq!(b.report.loss.get("foam.partition_cut_as_patch").unwrap().count, 1);
    assert!(b.mesh.patches.iter().all(|p| p.patch_type == FoamPatchType::Patch));
    check_ordering("nc", &w, &b, true);
}

fn mirror_deck(set: &str) -> sm_meshio::DeckRecord {
    use sm_meshio::{DeckParams, DeckRecord};
    DeckRecord {
        name: "mirror_x".into(),
        params: DeckParams::Mirror { normal: [1.0, 0.0, 0.0], offset: 0.0 },
        matrix: [-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        translation: [0.0; 3],
        improper: true,
        order: Some(2),
        face_set: Some(set.into()),
        mate_set: None,
    }
}

#[test]
fn a_planar_mirror_set_becomes_a_symmetry_plane_and_a_bent_one_a_patch() {
    let m = sets_block();
    let b = build_polymesh(&m, &FoamOptions::new().deck(mirror_deck("xmin"))).unwrap();
    let t: Vec<(&str, FoamPatchType)> = b.mesh.patches.iter().map(|p| (p.name.as_str(), p.patch_type)).collect();
    assert_eq!(t[0], ("xmin", FoamPatchType::SymmetryPlane));
    assert!(t[1..].iter().all(|p| p.1 == FoamPatchType::Patch));
    assert!(b.report.loss.get("foam.symmetry_not_planar").is_none());
    // A set that spans two planes (one xmin face and one ymin face) is not planar.
    let mut m = hex_block([2, 2, 2], [0.5; 3], &mut SerialAlloc::new()).unwrap();
    let mut alloc = SerialAlloc::new();
    let bent = vec![faces_on(&m, 0, 0.0)[0], faces_on(&m, 1, 0.0)[0]];
    let mut bent = bent;
    bent.sort();
    MeshEdit::add_set(&mut m, "bent", SetMembers::Faces(bent), &mut alloc).unwrap();
    let b = build_polymesh(&m, &FoamOptions::new().deck(mirror_deck("bent"))).unwrap();
    assert_eq!(b.mesh.patches[0].patch_type, FoamPatchType::Patch);
    assert_eq!(b.report.loss.get("foam.symmetry_not_planar").unwrap().count, 2);
    // An explicit type wins over the deck.
    let opts = FoamOptions::new().deck(mirror_deck("xmin")).patches(PatchSource::Explicit(vec![PatchSpec::new("xmin").of_type(FoamPatchType::Wall)]));
    assert_eq!(build_polymesh(&sets_block(), &opts).unwrap().mesh.patches[0].patch_type, FoamPatchType::Wall);
}

#[test]
fn translation_decks_stay_plain_patches_with_a_loss_item() {
    use sm_meshio::{DeckParams, DeckRecord};
    let d = DeckRecord {
        name: "shift".into(),
        params: DeckParams::Translation { vector: [1.0, 0.0, 0.0] },
        matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        translation: [1.0, 0.0, 0.0],
        improper: false,
        order: None,
        face_set: Some("xmin".into()),
        mate_set: Some("xmax".into()),
    };
    let b = build_polymesh(&sets_block(), &FoamOptions::new().deck(d)).unwrap();
    assert!(b.mesh.patches.iter().all(|p| p.patch_type == FoamPatchType::Patch));
    let items: Vec<(u64, &str)> = b.report.loss.items.iter().filter(|i| i.code == "foam.deck_cyclic_not_emitted").map(|i| (i.count, i.detail.as_str())).collect();
    assert_eq!(items.len(), 2, "one item per patch of the pair: {items:?}");
    assert!(items.iter().all(|i| i.0 == 4));
}

#[test]
fn a_side_set_becomes_a_face_zone_with_flips() {
    let m = hex_block([2, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let int = (0..m.face_count()).map(FaceIx::new).find(|f| m.face_kind(*f) == FaceKind::Interior).unwrap();
    let bnd = (0..m.face_count()).map(FaceIx::new).filter(|f| m.face_kind(*f) == FaceKind::Boundary).take(2).collect::<Vec<_>>();
    let mut m = m;
    let mut alloc = SerialAlloc::new();
    // Sides: the interior face from its right cell, a boundary face from its left cell.
    let mut sides = vec![SignedFace::new(int, Side::Right), SignedFace::new(bnd[0], Side::Left), SignedFace::new(bnd[1], Side::Left)];
    sides.sort_by_key(|s| s.to_raw());
    MeshEdit::add_set(&mut m, "s", SetMembers::Sides(sides), &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, "plain", SetMembers::Faces(vec![int]), &mut alloc).unwrap();
    let b = build_polymesh(&m, &FoamOptions::new().face_zones(vec!["s".into(), "plain".into()])).unwrap();
    assert_eq!(b.mesh.face_zones.len(), 2);
    let foam_of = |f: FaceIx| b.face_of_foam.iter().position(|x| *x == f.0).unwrap() as u32;
    let s = &b.mesh.face_zones[0];
    assert_eq!(s.name, "s");
    let mut want = [(foam_of(int), true), (foam_of(bnd[0]), false), (foam_of(bnd[1]), false)];
    want.sort();
    assert_eq!(s.labels, want.iter().map(|w| w.0).collect::<Vec<_>>());
    assert_eq!(s.flips, want.iter().map(|w| w.1).collect::<Vec<_>>());
    assert!(s.flips.iter().any(|f| *f) && s.flips.iter().any(|f| !*f));
    let p = &b.mesh.face_zones[1];
    assert_eq!((p.name.as_str(), p.labels.as_slice(), p.flips.as_slice()), ("plain", &[foam_of(int)][..], &[false][..]));
    // Face zones round trip through the files.
    let files = b.mesh.files().unwrap();
    assert!(files.iter().any(|f| f.name == "faceZones"));
    assert_eq!(parse_polymesh(&files).unwrap(), b.mesh);
    // Opt-in only: the same mesh without the option has no faceZones, and the sets are not patches
    // (they hold interior faces) but reported as skipped.
    let b = build(&m);
    assert!(b.mesh.face_zones.is_empty());
    assert!(b.report.loss.get("foam.set_skipped").is_some());
    // Both sides of one face keep the left entry.
    let mut m2 = hex_block([2, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let both = vec![SignedFace::new(int, Side::Left), SignedFace::new(int, Side::Right)];
    MeshEdit::add_set(&mut m2, "both", SetMembers::Sides(both), &mut SerialAlloc::new()).unwrap();
    let b = build_polymesh(&m2, &FoamOptions::new().face_zones(vec!["both".into()])).unwrap();
    assert_eq!(b.mesh.face_zones[0].flips, [false]);
    assert_eq!(b.report.loss.get("foam.zone_both_sides").unwrap().count, 1);
    // Unknown and wrong-kind names.
    assert!(matches!(foam_err(build_polymesh(&m, &FoamOptions::new().face_zones(vec!["zz".into()]))), FoamError::UnknownSet(_)));
}

#[test]
fn names_are_sanitized_and_collisions_are_errors() {
    let mut m = hex_block([2, 2, 2], [0.5; 3], &mut SerialAlloc::new()).unwrap();
    let mut alloc = SerialAlloc::new();
    let f = faces_on(&m, 0, 0.0);
    add_faces(&mut m, "in let/1", f, &mut alloc);
    let b = build(&m);
    assert_eq!(b.mesh.patches[0].name, "in_let_1");
    assert!(b.report.loss.get("foam.name_sanitized").is_some());
    // Collision after sanitizing.
    let mut m = hex_block([2, 2, 2], [0.5; 3], &mut SerialAlloc::new()).unwrap();
    let (f0, f1) = (faces_on(&m, 0, 0.0), faces_on(&m, 0, 1.0));
    add_faces(&mut m, "a b", f0, &mut alloc);
    add_faces(&mut m, "a_b", f1, &mut alloc);
    assert!(matches!(foam_err(build_polymesh(&m, &FoamOptions::default())), FoamError::DuplicateName(n) if n == "a_b"));
    // A set named like the default patch collides with it when the default patch is used.
    let mut m = hex_block([2, 2, 2], [0.5; 3], &mut SerialAlloc::new()).unwrap();
    let f = faces_on(&m, 0, 0.0);
    add_faces(&mut m, "defaultFaces", f, &mut SerialAlloc::new());
    assert!(matches!(foam_err(build_polymesh(&m, &FoamOptions::default())), FoamError::DuplicateName(_)));
    // An empty default-patch name is bad.
    let mut mm = hex_block([2, 2, 2], [0.5; 3], &mut SerialAlloc::new()).unwrap();
    let f = vec![faces_on(&mm, 0, 0.0)[0]];
    add_faces(&mut mm, "x", f, &mut SerialAlloc::new());
    assert!(matches!(foam_err(build_polymesh(&mm, &FoamOptions::new().default_patch(""))), FoamError::BadName(_)));
}

#[test]
fn names_that_read_as_numbers_get_a_prefix() {
    let mut alloc = SerialAlloc::new();
    for (given, want) in [("1e5", "_1e5"), ("1", "_1"), ("+x", "_+x"), ("-x", "_-x"), (".x", "_.x"), ("3 inlet", "_3_inlet"), ("e5", "e5")] {
        let mut m = hex_block([2, 2, 2], [0.5; 3], &mut alloc).unwrap();
        let f = faces_on(&m, 0, 0.0);
        add_faces(&mut m, given, f, &mut alloc);
        let b = build(&m);
        assert_eq!(b.mesh.patches[0].name, want, "{given}");
    }
}

#[test]
fn out_of_range_columns_are_inconsistent_not_panics() {
    let m = hex_block([2, 2, 2], [0.5; 3], &mut SerialAlloc::new()).unwrap();
    let nv = m.vertex_count() as u32;
    let nc = m.cell_count() as u32;
    let o = FoamOptions::default();
    let inconsistent = |r: sm_meshio::Result<FoamBuild>| matches!(r, Err(MeshioError::Inconsistent(_)));
    assert!(inconsistent(build_polymesh(&Wrap::new(&m).corrupt_vertex(0, nv), &o)));
    assert!(inconsistent(build_polymesh(&Wrap::new(&m).corrupt_vertex(3, u32::MAX), &o)));
    assert!(inconsistent(build_polymesh(&Wrap::new(&m).corrupt_left(0, nc), &o)));
    assert!(inconsistent(build_polymesh(&Wrap::new(&m).corrupt_right(0, nc), &o)));
    assert!(build_polymesh(&Wrap::new(&m), &o).is_ok());
}

// ---- errors of the input mesh ----------------------------------------------------------------------

#[test]
fn unsupported_inputs_are_typed_errors() {
    let m = hex_block([2, 2, 2], [0.5; 3], &mut SerialAlloc::new()).unwrap();
    assert!(matches!(foam_err(build_polymesh(&Wrap::new(&m).dim2(), &FoamOptions::default())), FoamError::NotThreeDimensional));
    assert!(matches!(foam_err(build_polymesh(&Wrap::new(&m).inactive(&[0, 3]), &FoamOptions::default())), FoamError::InactiveCells { count: 2 }));
    assert!(matches!(foam_err(build_polymesh(&Wrap::new(&m).with_x(4, f64::NAN), &FoamOptions::default())), FoamError::NonFinite { point: 4 }));
}

#[test]
fn options_json_shape() {
    let o: FoamOptions = serde_json::from_str(
        r#"{"patches":{"explicit":[{"set":"xmin","name":"inlet","patch_type":"patch"},{"set":"ymin","patch_type":"symmetry_plane"}]},
            "cell_zones":{"names":["b1"]},"face_zones":["s"]}"#,
    )
    .unwrap();
    assert_eq!(o.patches, PatchSource::Explicit(vec![PatchSpec::new("xmin").named("inlet"), PatchSpec::new("ymin").of_type(FoamPatchType::SymmetryPlane)]));
    assert_eq!(o.cell_zones, ZoneSelect::Names(vec!["b1".into()]));
    assert_eq!(o.face_zones, ["s"]);
    assert_eq!(o.default_patch, "defaultFaces");
    for (t, want) in [("sets", PatchSource::Sets), ("classification", PatchSource::Classification)] {
        let o: FoamOptions = serde_json::from_str(&format!(r#"{{"patches":"{t}"}}"#)).unwrap();
        assert_eq!(o.patches, want);
    }
    assert_eq!(serde_json::from_str::<FoamOptions>("{}").unwrap(), FoamOptions::default());
    assert_eq!(serde_json::from_str::<FoamOptions>(r#"{"cell_zones":"none"}"#).unwrap().cell_zones, ZoneSelect::None);
    assert_eq!(serde_json::from_str::<FoamOptions>(r#"{"cell_zones":"all"}"#).unwrap().cell_zones, ZoneSelect::All);
    let json = serde_json::to_string(&FoamOptions::new().deck(mirror_deck("xmin"))).unwrap();
    assert_eq!(serde_json::from_str::<FoamOptions>(&json).unwrap().decks.len(), 1);
    assert_eq!(FoamPatchType::SymmetryPlane.as_str(), "symmetryPlane");
    assert_eq!(FoamPatchType::Wall.as_str(), "wall");
}

#[test]
fn high_order_cells_are_reported_not_written() {
    let b = build(&common::ho_hex());
    assert_eq!(b.report.loss.get("foam.high_order_dropped").unwrap().count, 1);
    assert_eq!(b.mesh.points.len(), 8, "extra nodes are never written");
    assert_eq!(b.report.n_points, 8);
}

// ---- 6. golden bytes -------------------------------------------------------------------------------

#[test]
fn single_hex_files_equal_the_hand_written_golden_bytes() {
    let b = build(&single_hex().unwrap().mesh);
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data/foam/single_hex");
    let files = b.mesh.files().unwrap();
    assert_eq!(files.len(), 5);
    for f in &files {
        let want = std::fs::read(dir.join(&f.name)).unwrap_or_else(|e| panic!("{}: {e}", f.name));
        assert_eq!(String::from_utf8_lossy(&f.bytes), String::from_utf8_lossy(&want), "{}", f.name);
    }
}

#[test]
fn long_zones_round_trip_through_the_block_layout() {
    // 4x4x4 block: a 64-cell zone and a faceZone over the 96 boundary faces; lists longer than the
    // inline threshold are written in OpenFOAM's multi-line layout.
    let mut m = hex_block([4, 4, 4], [0.25; 3], &mut SerialAlloc::new()).unwrap();
    let mut alloc = SerialAlloc::new();
    MeshEdit::add_set(&mut m, "all", SetMembers::Cells((0..64).map(CellIx::new).collect()), &mut alloc).unwrap();
    let bnd: Vec<FaceIx> = (0..m.face_count()).map(FaceIx::new).filter(|f| m.face_kind(*f) == FaceKind::Boundary).collect();
    assert_eq!(bnd.len(), 96);
    let sides: Vec<SignedFace> = bnd.iter().map(|f| SignedFace::new(*f, Side::Left)).collect();
    MeshEdit::add_set(&mut m, "skin", SetMembers::Sides(sides), &mut alloc).unwrap();
    let b = build_polymesh(&m, &FoamOptions::new().face_zones(vec!["skin".into()])).unwrap();
    assert_eq!((b.mesh.cell_zones[0].labels.len(), b.mesh.face_zones[0].labels.len()), (64, 96));
    let files = b.mesh.files().unwrap();
    let text = |n: &str| String::from_utf8(files.iter().find(|f| f.name == n).unwrap().bytes.clone()).unwrap();
    assert!(text("cellZones").contains(")\n;\n}"), "block layout, one semicolon");
    assert!(!text("cellZones").contains(";;") && !text("faceZones").contains(";;"));
    assert_eq!(parse_polymesh(&files).unwrap(), b.mesh);
    // Exactly 10 and 11 labels straddle the inline threshold.
    for n in [10usize, 11] {
        let mut m = hex_block([4, 4, 4], [0.25; 3], &mut SerialAlloc::new()).unwrap();
        MeshEdit::add_set(&mut m, "z", SetMembers::Cells((0..n).map(CellIx::new).collect()), &mut SerialAlloc::new()).unwrap();
        let b = build(&m);
        assert_eq!(parse_polymesh(&b.mesh.files().unwrap()).unwrap(), b.mesh, "{n} labels");
    }
}
