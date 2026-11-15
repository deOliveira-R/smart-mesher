//! MOD-REFINE-L on the pure pass (`sm_modify::refine_local`): the hand counts of the 3x3x3 block
//! with its centre refined, exact volume, conformity, planarity, the 2:1 balance (property test),
//! the level map, sets and tags.

use std::collections::BTreeSet;

use proptest::prelude::*;
use sm_mesh::MeshBuilder;
use sm_mesh_api::reference::hex_block_with_edges;
use sm_mesh_api::{
    CellIx, CellShape, FaceIx, FaceKind, LevelMap, MeshEdit, MeshRead, SerialAlloc, SetMembers, TagValues, VertexIx, cell_volume, validate,
};
use sm_modify::{LocalError, LocalRefined, refine_local};
use sm_quality::{Metric, Options, compute};
use sm_testkit::strategies::config;

fn block(n: [u32; 3]) -> sm_mesh_api::reference::ReferenceMesh {
    hex_block_with_edges(n, [1.0; 3], &mut SerialAlloc::new()).expect("block")
}

fn run(src: &dyn MeshRead, sel: &[bool], levels: u32, balance: bool) -> Result<LocalRefined<sm_mesh::Mesh>, LocalError> {
    refine_local(src, None, sel, levels, balance, false, MeshBuilder::new(), &mut SerialAlloc::new())
}

fn volume(m: &dyn MeshRead) -> f64 {
    (0..m.cell_count()).map(|c| cell_volume(m, CellIx::new(c))).sum()
}

fn centre(n: usize) -> Vec<bool> {
    let mut s = vec![false; n * n * n];
    s[(n / 2) * n * n + (n / 2) * n + n / 2] = true;
    s
}

/// Face loop sizes of a cell, sorted.
fn face_sizes(m: &dyn MeshRead, c: usize) -> Vec<usize> {
    let mut v: Vec<usize> = m.cell_faces(CellIx::new(c)).iter().map(|sf| m.face_vertices(sf.face()).len()).collect();
    v.sort_unstable();
    v
}

/// Largest distance of a face's vertices from its Newell plane.
fn nonplanarity(m: &dyn MeshRead, f: FaceIx) -> f64 {
    let l = m.face_vertices(f);
    let p = |v: VertexIx| {
        let q = m.point(v.get());
        [q.x, q.y, q.z]
    };
    let mut n = [0.0f64; 3];
    let mut cen = [0.0f64; 3];
    for i in 0..l.len() {
        let (a, b) = (p(l[i]), p(l[(i + 1) % l.len()]));
        n[0] += (a[1] - b[1]) * (a[2] + b[2]);
        n[1] += (a[2] - b[2]) * (a[0] + b[0]);
        n[2] += (a[0] - b[0]) * (a[1] + b[1]);
        for d in 0..3 {
            cen[d] += a[d] / l.len() as f64;
        }
    }
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    l.iter().map(|&v| {
        let a = p(v);
        ((a[0] - cen[0]) * n[0] + (a[1] - cen[1]) * n[1] + (a[2] - cen[2]) * n[2]).abs() / len
    }).fold(0.0, f64::max)
}

/// Level of a result cell: log8 of its parent's child count.
fn level_of(r: &LocalRefined<sm_mesh::Mesh>, nparents: usize) -> Vec<u32> {
    let mut count = vec![0u32; nparents];
    for p in &r.coarse_of {
        count[p.get()] += 1;
    }
    r.coarse_of.iter().map(|p| (count[p.get()] as f64).log(8.0).round() as u32).collect()
}

#[test]
fn centre_of_a_3x3x3_block_has_the_hand_counts() {
    let src = block([3, 3, 3]);
    let r = run(&src, &centre(3), 1, true).expect("refine");
    let m = &r.finished.mesh;
    assert_eq!(r.stats, {
        let mut s = r.stats;
        s.parents = 27;
        s
    });
    assert_eq!(m.vertex_count(), 64 + 19, "64 + 12 edge midpoints + 6 face centres + the cell centre");
    assert_eq!(m.cell_count(), 26 + 8);
    assert_eq!((r.stats.hexes, r.stats.polyhedra), (8 + 8, 6 + 12 + 0 + 0), "8 children + 8 corners; 6 face + 12 edge neighbours");
    // Per source cell: shapes and face-loop sizes.
    let mut face_nb = 0;
    let mut edge_nb = 0;
    let mut corner_nb = 0;
    for c in 0..m.cell_count() {
        let parent = r.coarse_of[c].get();
        if parent == 13 {
            assert_eq!(m.cell_shape(CellIx::new(c)), CellShape::Hex);
            continue;
        }
        let (i, j, k) = (parent % 3, (parent / 3) % 3, parent / 9);
        let ones = [i, j, k].iter().filter(|&&x| x == 1).count();
        let s = face_sizes(m, c);
        match ones {
            2 => {
                // face neighbour: 5 original + 4 split faces; 4 side faces gain the hanging midpoint
                face_nb += 1;
                assert_eq!(m.cell_shape(CellIx::new(c)), CellShape::Polyhedron);
                assert_eq!(s.len(), 9);
                assert_eq!(s.iter().filter(|&&n| n == 5).count(), 4);
                assert_eq!(s.iter().filter(|&&n| n == 4).count(), 5);
            }
            1 => {
                edge_nb += 1;
                assert_eq!(m.cell_shape(CellIx::new(c)), CellShape::Polyhedron);
                assert_eq!(s, vec![4, 4, 4, 4, 5, 5]);
            }
            _ => {
                corner_nb += 1;
                assert_eq!(m.cell_shape(CellIx::new(c)), CellShape::Hex, "corner neighbours are untouched hexes");
            }
        }
    }
    assert_eq!((face_nb, edge_nb, corner_nb), (6, 12, 8));
    assert!((volume(m) - 27.0).abs() < 1e-12);
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert!((0..m.face_count()).all(|f| m.face_kind(FaceIx::new(f)) != FaceKind::NonConformal));
    // 12 hanging edge midpoints and 6 face centres are shared by both sides: no face has one owner only inside.
    assert!((0..m.face_count()).all(|f| nonplanarity(m, FaceIx::new(f)) < 1e-12), "every face stays planar");
    // The pair is a valid LevelMap with a face map.
    LevelMap::new(m, &src, r.coarse_of.clone(), Some(r.face_map.clone())).expect("level map");
    // Quality: all hexes have scaled Jacobian 1.
    let sj = compute(m, Metric::ScaledJacobian, &Options::default()).expect("sj");
    let min = sj.iter().copied().filter(|x| x.is_finite()).fold(f64::INFINITY, f64::min);
    assert!((min - 1.0).abs() < 1e-12, "min scaled Jacobian {min}");
}

#[test]
fn child_volumes_and_sub_face_areas_are_exact() {
    let src = block([3, 3, 3]);
    let r = run(&src, &centre(3), 1, true).expect("refine");
    let m = &r.finished.mesh;
    for c in 0..m.cell_count() {
        if r.coarse_of[c].get() == 13 {
            assert!((cell_volume(m, CellIx::new(c)) - 0.125).abs() < 1e-14);
        } else {
            assert!((cell_volume(m, CellIx::new(c)) - 1.0).abs() < 1e-14, "neighbours keep their volume");
        }
    }
}

#[test]
fn two_levels_are_refused_without_balance_and_raise_neighbours_with_it() {
    let src = block([3, 3, 3]);
    let err = run(&src, &centre(3), 2, false).err().expect("jump 2");
    assert!(matches!(err, LocalError::Unbalanced { .. }), "{err}");
    let r = run(&src, &centre(3), 2, true).expect("balanced");
    let m = &r.finished.mesh;
    // Centre 64 hexes; the 6 face neighbours (jump 2 across a face) are raised to level 1 (8 each).
    // Edge and corner neighbours share no face with the centre and stay level 0 (their jump to the
    // raised face neighbours is 1).
    assert_eq!(r.stats.refined, 7);
    assert_eq!(r.stats.balanced, 6);
    assert!((volume(m) - 27.0).abs() < 1e-12);
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    let lv = level_of(&r, 27);
    for f in 0..m.face_count() {
        let fi = FaceIx::new(f);
        if let Some(rt) = m.face_right(fi) {
            let (a, b) = (m.face_left(fi).get(), rt.get());
            if r.coarse_of[a] != r.coarse_of[b] {
                assert!(lv[a].abs_diff(lv[b]) <= 1);
            }
        }
    }
}

#[test]
fn refining_a_corner_block_keeps_boundary_area_and_validates() {
    let src = block([2, 2, 2]);
    let mut sel = vec![false; 8];
    sel[0] = true;
    let r = run(&src, &sel, 1, true).expect("refine");
    let m = &r.finished.mesh;
    let area: f64 = (0..m.face_count())
        .map(|f| FaceIx::new(f))
        .filter(|&f| m.face_kind(f) == FaceKind::Boundary)
        .map(|f| sm_mesh_api::face_area_vector(m, f).norm())
        .sum();
    assert!((area - 24.0).abs() < 1e-12, "boundary area {area}");
    assert!((volume(m) - 8.0).abs() < 1e-12);
    assert!(validate(m).is_empty());
    // 7 hexes: 3 face neighbours + 3 edge neighbours (polyhedra) + 1 corner (a hex): 8 + 6 + 1.
    assert_eq!(m.cell_count(), 8 + 7);
    assert_eq!((r.stats.hexes, r.stats.polyhedra), (9, 6));
}

#[test]
fn a_whole_selection_is_a_uniform_refinement() {
    let src = block([2, 2, 2]);
    let r = run(&src, &[true; 8], 1, true).expect("refine");
    let m = &r.finished.mesh;
    assert_eq!(m.cell_count(), 64);
    assert_eq!(m.vertex_count(), 125);
    assert_eq!(r.stats.polyhedra, 0);
    assert!(validate(m).is_empty());
    assert!((volume(m) - 8.0).abs() < 1e-12);
}

#[test]
fn sets_and_tags_carry_over() {
    let mut alloc = SerialAlloc::new();
    let mut src = hex_block_with_edges([3, 3, 3], [1.0; 3], &mut alloc).expect("block");
    src.add_set("mid", SetMembers::Cells(vec![CellIx::new(13), CellIx::new(14)]), &mut alloc).expect("set");
    src.set_tag("w", sm_mesh_api::MeshEntityKind::Cell, TagValues::F64((0..27).map(f64::from).collect())).expect("tag");
    let r = run(&src, &centre(3), 1, true).expect("refine");
    let m = &r.finished.mesh;
    let mid = m.sets().iter().find(|s| s.name == "mid").expect("set");
    let SetMembers::Cells(cells) = &mid.members else { panic!("cells") };
    assert_eq!(cells.len(), 8 + 1, "the refined cell's 8 children plus the other member");
    let t = m.tags().iter().find(|t| t.name == "w").expect("tag");
    let TagValues::F64(v) = &t.values else { panic!("f64") };
    for (c, x) in v.iter().enumerate() {
        assert_eq!(*x, r.coarse_of[c].get() as f64);
    }
}

#[test]
fn non_hex_cells_that_would_be_refined_are_refused() {
    let src = sm_testkit::meshes::tet_block([1, 1, 1], [1.0; 3]).expect("tets").mesh;
    let err = run(&src, &[true, false, false, false, false, false], 1, true).err().expect("tet");
    assert!(matches!(err, LocalError::Unrefinable { .. }), "{err}");
}

#[test]
fn selection_must_be_non_empty() {
    let src = block([2, 2, 2]);
    assert!(matches!(run(&src, &[false; 8], 1, true), Err(LocalError::Empty(_))));
    assert!(matches!(run(&src, &[true; 8], 4, true), Err(LocalError::Levels(4))));
}

proptest! {
    #![proptest_config(config(48))]

    #[test]
    fn balanced_random_selections_validate_conserve_volume_and_have_no_jump_above_one(
        sel in proptest::collection::vec(0u8..4, 64), levels in 1u32..=2,
    ) {
        let src = block([4, 4, 4]);
        let mask: Vec<bool> = sel.iter().map(|&x| x == 0).collect();
        prop_assume!(mask.iter().any(|&b| b));
        let r = run(&src, &mask, levels, true).expect("refine");
        let m = &r.finished.mesh;
        prop_assert!((volume(m) - 64.0).abs() < 1e-11);
        prop_assert!(validate(m).is_empty(), "{:?}", validate(m));
        prop_assert!((0..m.face_count()).all(|f| m.face_kind(FaceIx::new(f)) != FaceKind::NonConformal));
        prop_assert!((0..m.face_count()).all(|f| nonplanarity(m, FaceIx::new(f)) < 1e-12));
        LevelMap::new(m, &src, r.coarse_of.clone(), Some(r.face_map.clone())).expect("level map");
        let lv = level_of(&r, 64);
        let mut seen = BTreeSet::new();
        for f in 0..m.face_count() {
            let fi = FaceIx::new(f);
            if let Some(rt) = m.face_right(fi) {
                let (a, b) = (m.face_left(fi).get(), rt.get());
                if r.coarse_of[a] != r.coarse_of[b] {
                    prop_assert!(lv[a].abs_diff(lv[b]) <= 1, "face {f}: levels {} {}", lv[a], lv[b]);
                    seen.insert((lv[a], lv[b]));
                }
            }
        }
        // The same call twice is bit-identical.
        let r2 = run(&src, &mask, levels, true).expect("again");
        prop_assert_eq!(sm_mesh_api::topology_hash(m), sm_mesh_api::topology_hash(&r2.finished.mesh));
        prop_assert_eq!(sm_mesh_api::geometry_hash(m), sm_mesh_api::geometry_hash(&r2.finished.mesh));
    }
}
