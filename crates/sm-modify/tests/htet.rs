//! MOD-HTET acceptance 1-5, 7-9 on the pure passes (`sm_modify::htet`, `sm_modify::qtri`):
//! Dompierre conformity by vertex ranks over all 8! orderings, shared-face relabelings, wedge and
//! pyramid counts, a 4x4x4 block, polyhedral closure, QTri areas, deck pairs and the inversion
//! refusal. Volumes and areas are analytic.

use std::collections::{BTreeMap, BTreeSet};

use sm_core::Point3;
use sm_mesh::MeshBuilder;
use sm_mesh_api::reference::{ReferenceMesh, hex_block_with_edges};
use sm_mesh_api::{
    CellIx, CellShape, FaceIx, FaceKind, LevelMap, MeshEdit, MeshRead, SerialAlloc, SetMembers, cell_volume, face_area_vector, validate,
    validate_classification,
};
use sm_modify::{Closure, Converted, HtetError, HtetOptions, QtriMode, hex_tets, htet, qtri};
use sm_testkit::meshes::{CellSpec, build_mesh, single_hex, single_pyramid, single_wedge};

type Out = Converted<sm_mesh::Mesh>;

fn run(src: &dyn MeshRead, o: &HtetOptions<'_>) -> Result<Out, HtetError> {
    htet(src, o, MeshBuilder::new(), &mut SerialAlloc::new())
}

fn volume(m: &dyn MeshRead) -> f64 {
    (0..m.cell_count()).map(|c| cell_volume(m, CellIx::new(c))).sum()
}

fn assert_valid(m: &dyn MeshRead) {
    let v = validate(m);
    assert!(v.is_empty(), "validate: {v:?}");
    assert!(validate_classification(m).is_empty());
}

fn check_map(src: &dyn MeshRead, r: &Out) {
    let m = &r.finished.mesh;
    let map = LevelMap::new(m, src, r.coarse_of.clone(), Some(r.face_map.clone())).expect("LevelMap::new accepts the maps");
    assert_eq!(map.coarse_len(), src.cell_count());
}

/// All permutations of `0..n` (Heap's algorithm), as rank arrays.
fn permutations(n: usize) -> Vec<Vec<u32>> {
    let mut out = Vec::new();
    let mut a: Vec<u32> = (0..n as u32).collect();
    let mut c = vec![0usize; n];
    out.push(a.clone());
    let mut i = 0;
    while i < n {
        if c[i] < i {
            if i % 2 == 0 {
                a.swap(0, i);
            } else {
                a.swap(c[i], i);
            }
            out.push(a.clone());
            c[i] += 1;
            i = 0;
        } else {
            c[i] = 0;
            i += 1;
        }
    }
    out
}

/// Acceptance 1: every one of the 8! rank orderings of the unit cube gives 5 or 6 tets that tile
/// it (volume 1 to 1e-14, every tet positive); no ordering needs a Steiner point. The kernel
/// (`hex_tets`) runs all 40320; the full pass (builder, validation, conformity) runs every 97th.
#[test]
fn a1_all_40320_orderings_of_a_unit_cube_give_five_or_six_positive_tets() {
    let cube: [[f64; 3]; 8] = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];
    let det = |q: &[u32; 4]| {
        let p = |i: usize| cube[q[i] as usize];
        let (a, b, c) = ([p(1)[0] - p(0)[0], p(1)[1] - p(0)[1], p(1)[2] - p(0)[2]], [p(2)[0] - p(0)[0], p(2)[1] - p(0)[1], p(2)[2] - p(0)[2]], [p(3)[0] - p(0)[0], p(3)[1] - p(0)[1], p(3)[2] - p(0)[2]]);
        a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])
    };
    let perms = permutations(8);
    assert_eq!(perms.len(), 40320);
    let (mut five, mut six) = (0usize, 0usize);
    for p in &perms {
        let rank: [u32; 8] = core::array::from_fn(|i| p[i]);
        let (tets, is5) = hex_tets(&rank);
        assert!(tets.len() == 5 || tets.len() == 6, "{} tets for ordering {p:?}", tets.len());
        assert_eq!(is5, tets.len() == 5);
        let mut vol = 0.0;
        for q in &tets {
            let d = det(q);
            assert!(d > 0.0, "non-positive tet {q:?} for {p:?}");
            vol += d / 6.0;
        }
        assert!((vol - 1.0).abs() < 1e-14, "volume {vol} for {p:?}");
        if is5 {
            five += 1;
        } else {
            six += 1;
        }
    }
    eprintln!("a1: of 40320 orderings {five} give 5 tets, {six} give 6; no Steiner point is ever needed");
    assert_eq!(five + six, 40320);
    assert!(five > 0 && six > 0);
    let fx = single_hex().expect("hex");
    for p in perms.iter().step_by(97) {
        let r = run(&fx.mesh, &HtetOptions { root: Some(p), ..HtetOptions::default() }).expect("htet");
        let m = &r.finished.mesh;
        assert!(m.cell_count() == 5 || m.cell_count() == 6);
        assert!((volume(m) - 1.0).abs() < 1e-14);
        assert_valid(m);
        assert_eq!(m.vertex_count(), 8, "no Steiner point");
    }
}

/// A 2x1x1 block with vertices relabeled by `rank`.
fn two_cubes() -> ReferenceMesh {
    hex_block_with_edges([2, 1, 1], [1.0; 3], &mut SerialAlloc::new()).expect("block")
}

/// Acceptance 2: two cubes sharing a quad under 1000 relabelings: both sides cut it along the
/// same diagonal (the interior faces are exactly the two triangles of the quad), the result validates.
mod relabel {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        #![proptest_config(sm_testkit::strategies::config(1000))]
        #[test]
        fn two_cubes_agree_on_the_shared_diagonal(rank in Just((0u32..12).collect::<Vec<_>>()).prop_shuffle()) {
            let src = two_cubes();
            let r = run(&src, &HtetOptions { root: Some(&rank), ..HtetOptions::default() }).unwrap();
            let m = &r.finished.mesh;
            prop_assert!(validate(m).is_empty());
            prop_assert!((volume(m) - 2.0).abs() < 1e-14);
            // Interior faces: the shared quad is cut in two triangles, plus the faces inside the two cells.
            let shared: Vec<FaceIx> = (0..m.face_count()).map(FaceIx::new)
                .filter(|&f| m.face_kind(f) == FaceKind::Interior && m.face_vertices(f).iter().all(|v| (m.point(v.get()).x - 1.0).abs() < 1e-12))
                .collect();
            prop_assert_eq!(shared.len(), 2);
            let area: f64 = shared.iter().map(|&f| face_area_vector(m, f).norm()).sum();
            prop_assert!((area - 1.0).abs() < 1e-14);
            // The boundary stays a closed 2-manifold of 10 quads cut into 20 tris.
            let bnd = (0..m.face_count()).filter(|&f| m.face_kind(FaceIx::new(f)) == FaceKind::Boundary).count();
            prop_assert_eq!(bnd, 20);
        }
    }
}

/// Acceptance 3: wedge gives 3 tets (right triangle, height 1: 0.5), pyramid 2 (height 1: 1/3).
#[test]
fn a3_wedge_gives_three_tets_and_pyramid_two() {
    let w = single_wedge().expect("wedge");
    let r = run(&w.mesh, &HtetOptions::default()).expect("wedge");
    assert_eq!(r.finished.mesh.cell_count(), 3);
    assert!((volume(&r.finished.mesh) - w.volume).abs() < 1e-14, "{} vs {}", volume(&r.finished.mesh), w.volume);
    assert_valid(&r.finished.mesh);
    check_map(&w.mesh, &r);
    let p = single_pyramid().expect("pyramid");
    let r = run(&p.mesh, &HtetOptions::default()).expect("pyramid");
    assert_eq!(r.finished.mesh.cell_count(), 2);
    assert!((volume(&r.finished.mesh) - p.volume).abs() < 1e-14);
    assert!((p.volume - 1.0 / 3.0).abs() < 1e-14, "square pyramid of height 1 is 1/3, fixture says {}", p.volume);
    assert_valid(&r.finished.mesh);
    check_map(&p.mesh, &r);
    // Every ordering of the 5 pyramid corners gives 2 tets, of the 6 wedge corners 3.
    for rank in permutations(5) {
        let r = run(&p.mesh, &HtetOptions { root: Some(&rank), ..HtetOptions::default() }).expect("pyramid");
        assert_eq!(r.finished.mesh.cell_count(), 2);
        assert!((volume(&r.finished.mesh) - p.volume).abs() < 1e-14);
    }
    for rank in permutations(6) {
        let r = run(&w.mesh, &HtetOptions { root: Some(&rank), ..HtetOptions::default() }).expect("wedge");
        assert_eq!(r.finished.mesh.cell_count(), 3);
        assert!((volume(&r.finished.mesh) - w.volume).abs() < 1e-14);
    }
}

/// Acceptance 4: a 4x4x4 block to tets: volume 64, validates, a side set keeps its area (each
/// boundary quad becomes 2 tris), blocks (cell sets) and the level map carry over.
#[test]
fn a4_block_4x4x4_to_tets_keeps_volume_sets_and_area() {
    let mut alloc = SerialAlloc::new();
    let mut src = hex_block_with_edges([4, 4, 4], [1.0; 3], &mut alloc).expect("block");
    let left: Vec<FaceIx> = (0..src.face_count())
        .map(FaceIx::new)
        .filter(|&f| src.face_kind(f) == FaceKind::Boundary && src.face_vertices(f).iter().all(|v| src.point(v.get()).x == 0.0))
        .collect();
    assert_eq!(left.len(), 16);
    src.add_set("xmin", SetMembers::Faces(left), &mut alloc).expect("set");
    let low: Vec<CellIx> = (0..src.cell_count()).map(CellIx::new).filter(|&c| cell_volume(&src, c) > 0.0 && src.cell_corners(c).expect("hex").iter().all(|v| src.point(v.get()).z <= 2.0)).collect();
    src.add_set("lower", SetMembers::Cells(low.clone()), &mut alloc).expect("set");
    let r = run(&src, &HtetOptions::default()).expect("htet");
    let m = &r.finished.mesh;
    assert_valid(m);
    assert!((volume(m) - 64.0).abs() < 1e-12, "volume {}", volume(m));
    assert!((0..m.cell_count()).all(|c| m.cell_shape(CellIx::new(c)) == CellShape::Tet));
    let SetMembers::Faces(fs) = &m.set_by_name("xmin").expect("set").members else { panic!("faces") };
    assert_eq!(fs.len(), 32);
    let area: f64 = fs.iter().map(|&f| face_area_vector(m, f).norm()).sum();
    assert!((area - 16.0).abs() < 1e-13, "area {area}");
    let SetMembers::Cells(cs) = &m.set_by_name("lower").expect("set").members else { panic!("cells") };
    let (five, six) = (r.stats.hex5, r.stats.hex6);
    assert_eq!(five + six, 64);
    assert!(cs.len() >= 32 * 5 && cs.len() <= 32 * 6);
    assert_eq!(m.cell_count(), 5 * five + 6 * six);
    check_map(&src, &r);
    eprintln!("a4: 64 hexes -> {} tets ({five} cut in 5, {six} in 6); volume {}", m.cell_count(), volume(m));
}

/// One quad cell in a 2D mesh, classified on a model face with its edges on model edges.
fn quad_mesh(pts: &[[f64; 3]; 4]) -> sm_mesh::Mesh {
    use sm_core::{EntityId, EntityKind};
    use sm_mesh_api::{Classification, MeshBuild, MeshEntityRef, VertexIx};
    let on = |k: EntityKind, n: u64| Classification::on(EntityId::new(k, n).expect("id"));
    let mut b = MeshBuilder::surface();
    let vs: Vec<VertexIx> = pts.iter().map(|p| b.add_vertex(Point3::new(p[0], p[1], p[2])).expect("vertex")).collect();
    for (i, v) in vs.iter().enumerate() {
        b.set_class(MeshEntityRef::Vertex(*v), on(EntityKind::TopoVertex, i as u64 + 1)).expect("class");
    }
    let added = b.add_cell_from_vertices(CellShape::Quad, &vs).expect("quad");
    b.set_class(MeshEntityRef::Cell(added.cell), on(EntityKind::TopoFace, 1)).expect("class");
    for (i, f) in added.faces.iter().enumerate() {
        b.set_class(MeshEntityRef::Face(f.face()), on(EntityKind::TopoEdge, i as u64 + 1)).expect("class");
    }
    b.finish(&mut SerialAlloc::new()).expect("finish").mesh
}

fn left_cell_only(src: &dyn MeshRead) -> Vec<bool> {
    (0..src.cell_count()).map(|c| src.cell_corners(CellIx::new(c)).expect("hex").iter().all(|v| src.point(v.get()).x <= 1.0)).collect()
}

/// Acceptance 5: only the left hex of a 2x1x1 block selected: the right cell has 7 faces and is a
/// polyhedron, the mesh validates, the interface area is unchanged; `whole` refuses naming the cell.
#[test]
fn a5_closure_turns_the_unselected_neighbour_into_a_polyhedron() {
    let src = two_cubes();
    let sel = left_cell_only(&src);
    assert_eq!(sel.iter().filter(|&&s| s).count(), 1);
    let r = run(&src, &HtetOptions { select: Some(&sel), ..HtetOptions::default() }).expect("htet");
    let m = &r.finished.mesh;
    assert_valid(m);
    let polys: Vec<usize> = (0..m.cell_count()).filter(|&c| m.cell_shape(CellIx::new(c)) == CellShape::Polyhedron).collect();
    assert_eq!(polys.len(), 1);
    assert_eq!(m.cell_faces(CellIx::new(polys[0])).len(), 7, "5 quads plus the two triangles of the cut face");
    assert!((volume(m) - 2.0).abs() < 1e-14);
    let iface: f64 = (0..m.face_count())
        .map(FaceIx::new)
        .filter(|&f| m.face_kind(f) == FaceKind::Interior && m.face_vertices(f).iter().all(|v| (m.point(v.get()).x - 1.0).abs() < 1e-12))
        .map(|f| face_area_vector(m, f).norm())
        .sum();
    assert!((iface - 1.0).abs() < 1e-14, "interface area {iface}");
    assert_eq!(r.stats.polyhedra, 1);
    check_map(&src, &r);
    let Err(e) = run(&src, &HtetOptions { select: Some(&sel), closure: Closure::Whole, ..HtetOptions::default() }) else { panic!("whole refuses") };
    assert!(matches!(e, HtetError::OpenClosure { neighbour: 1, selected: 0 }) || matches!(e, HtetError::OpenClosure { neighbour: 0, selected: 1 }), "{e}");
    assert!(e.to_string().contains("cell"), "{e}");
}

/// Selecting everything with `whole` is closed and fine.
#[test]
fn whole_accepts_a_closed_selection() {
    let src = two_cubes();
    let all = vec![true; 2];
    let r = run(&src, &HtetOptions { select: Some(&all), closure: Closure::Whole, ..HtetOptions::default() }).expect("closed");
    assert_valid(&r.finished.mesh);
}

/// Acceptance 7: QTri on a unit square and a parallelogram of area A.
#[test]
fn a7_qtri_two_split_and_four_split_areas() {
    for (pts, area) in [
        ([[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.]], 1.0),
        ([[0., 0., 0.], [3., 0., 0.], [4., 2., 0.], [1., 2., 0.]], 6.0),
    ] {
        let src = quad_mesh(&pts);
        let two = qtri(&src, QtriMode::Two, None, None, MeshBuilder::surface(), &mut SerialAlloc::new()).expect("2-split");
        let m = &two.finished.mesh;
        assert_eq!(m.cell_count(), 2);
        let a: f64 = (0..2).map(|c| cell_volume(m, CellIx::new(c))).sum();
        assert!((a - area).abs() < 1e-14, "2-split area {a}");
        assert!(validate(m).is_empty());
        let four = qtri(&src, QtriMode::Four, None, None, MeshBuilder::surface(), &mut SerialAlloc::new()).expect("4-split");
        let m = &four.finished.mesh;
        assert_eq!(m.cell_count(), 4);
        assert_eq!(m.vertex_count(), 5);
        // The four triangles around the centroid have A/4 each (the parallelogram's centroid).
        for c in 0..4 {
            let a = cell_volume(m, CellIx::new(c));
            assert!((a - area / 4.0).abs() < 1e-14, "4-split triangle {a} vs {}", area / 4.0);
        }
        assert!(validate(m).is_empty());
        check_map(&src, &four);
    }
}

/// Acceptance 8 (pure pass): a 2x2x2 block with a translation pairing x = 0 and x = 2, vertex
/// indices permuted so that plain-index ranks disagree across the pair. With the deck roots every
/// B-side triangle is the translate of an A-side one; without them they are not.
#[test]
fn a8_deck_roots_make_paired_faces_split_alike_under_permuted_indices() {
    let n = 3usize;
    let id = |i: usize, j: usize, k: usize| (k * n + j) * n + i;
    let base: Vec<[f64; 3]> = (0..n * n * n).map(|v| [(v % n) as f64, ((v / n) % n) as f64, (v / (n * n)) as f64]).collect();
    // A fixed pseudo-random relabeling of the 27 vertices.
    let mut perm: Vec<usize> = (0..27).collect();
    let mut s = 12345u64;
    for i in (1..27).rev() {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        perm.swap(i, (s >> 33) as usize % (i + 1));
    }
    let mut pts = vec![[0.0; 3]; 27];
    for (old, p) in base.iter().enumerate() {
        pts[perm[old]] = *p;
    }
    let mut cells = Vec::new();
    for k in 0..2 {
        for j in 0..2 {
            for i in 0..2 {
                let v = [id(i, j, k), id(i + 1, j, k), id(i + 1, j + 1, k), id(i, j + 1, k), id(i, j, k + 1), id(i + 1, j, k + 1), id(i + 1, j + 1, k + 1), id(i, j + 1, k + 1)];
                cells.push(CellSpec::Template(CellShape::Hex, v.iter().map(|&x| perm[x]).collect()));
            }
        }
    }
    let src = build_mesh(&pts, &cells, &mut SerialAlloc::new()).expect("mesh");
    // Deck: A = vertices with x = 0, B = the same shifted by (2, 0, 0).
    let find = |p: [f64; 3]| (0..27).find(|&v| { let q = src.point(v); [q.x, q.y, q.z].iter().zip(p).all(|(a, b)| (a - b).abs() < 1e-12) }).expect("vertex");
    let mut root: Vec<u32> = (0..27).collect();
    let mut a_to_b: BTreeMap<u32, u32> = BTreeMap::new();
    for v in 0..27 {
        let p = [src.point(v).x, src.point(v).y, src.point(v).z];
        if p[0] == 0.0 {
            let b = find([2.0, p[1], p[2]]);
            let r = v.min(b) as u32;
            root[v] = r;
            root[b] = r;
            a_to_b.insert(v as u32, b as u32);
        }
    }
    let side_tris = |m: &dyn MeshRead, x: f64| -> BTreeSet<Vec<u32>> {
        (0..m.face_count())
            .map(FaceIx::new)
            .filter(|&f| m.face_vertices(f).iter().all(|v| (m.point(v.get()).x - x).abs() < 1e-12))
            .map(|f| {
                let mut l: Vec<u32> = m.face_vertices(f).iter().map(|v| v.0).collect();
                l.sort_unstable();
                l
            })
            .collect()
    };
    let mismatch = |r: &Out| -> usize {
        let m = &r.finished.mesh;
        let (a, b) = (side_tris(m, 0.0), side_tris(m, 2.0));
        let mapped: BTreeSet<Vec<u32>> = a
            .iter()
            .map(|t| {
                let mut v: Vec<u32> = t.iter().map(|x| a_to_b[x]).collect();
                v.sort_unstable();
                v
            })
            .collect();
        mapped.symmetric_difference(&b).count()
    };
    let with = run(&src, &HtetOptions { root: Some(&root), ..HtetOptions::default() }).expect("with roots");
    assert_valid(&with.finished.mesh);
    assert_eq!(side_tris(&with.finished.mesh, 0.0).len(), 8, "4 quads, 2 tris each");
    assert_eq!(mismatch(&with), 0, "every B-side pair is the transform of the A-side pair");
    let without = run(&src, &HtetOptions::default()).expect("plain");
    assert_valid(&without.finished.mesh);
    let bad = mismatch(&without);
    eprintln!("a8: plain-index ranks leave {bad} unmatched face triangles across the pair; deck roots leave 0");
    assert!(bad > 0, "the permutation must make plain-index Dompierre disagree across the pair");
    let _ = Point3::new(0.0, 0.0, 0.0);
}

/// Acceptance 9: a thin hex with a saddle-twisted top face (height 0.05, two top corners raised by
/// 0.1): every corner Jacobian is positive (min scaled Jacobian > 0) yet under the ordering
/// `[1, 0, 2, 5, 4, 7, 3, 6]` the cut has a non-positive tet: refused with `htet.inverted_tet`
/// naming the cell. (Under the identity labeling the same cell cuts fine: the refusal depends on
/// which diagonals the labels pick, which is why it is a refusal and not a warning.)
#[test]
fn a9_a_folded_face_is_refused_with_the_cell_named() {
    let h = 0.05;
    let pts = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., h], [1., 0., h + 0.1], [1., 1., h], [0., 1., h + 0.1]];
    let cells = [CellSpec::Template(CellShape::Hex, (0..8).collect())];
    let mesh = build_mesh(&pts, &cells, &mut SerialAlloc::new()).expect("mesh");
    let sj = sm_quality::compute(&mesh, sm_quality::Metric::ScaledJacobian, &sm_quality::Options::default()).expect("sj");
    let min = sj.iter().copied().fold(f64::INFINITY, f64::min);
    assert!(min > 0.0, "the cell is valid: min scaled Jacobian {min}");
    let rank = [1u32, 0, 2, 5, 4, 7, 3, 6];
    let Err(e) = run(&mesh, &HtetOptions { root: Some(&rank), ..HtetOptions::default() }) else { panic!("refused") };
    eprintln!("a9: min scaled Jacobian {min:.4} > 0, refused: {e}");
    assert!(matches!(e, HtetError::InvertedTet { cell: 0, .. }), "{e}");
    assert!(e.to_string().contains("htet.inverted_tet") && e.to_string().contains("cell 0"));
    assert!(run(&mesh, &HtetOptions::default()).is_ok(), "the identity labeling cuts it");
}

/// The pass is deterministic for 1 and N threads (it has no parallel reduction; the builder is serial).
#[test]
fn a10_two_runs_and_one_thread_give_equal_hashes() {
    let src = hex_block_with_edges([3, 2, 2], [1.0, 0.5, 2.0], &mut SerialAlloc::new()).expect("block");
    let hash = |r: &Out| (sm_mesh_api::topology_hash(&r.finished.mesh), sm_mesh_api::geometry_hash(&r.finished.mesh));
    let a = hash(&run(&src, &HtetOptions::default()).expect("a"));
    assert_eq!(a, hash(&run(&src, &HtetOptions::default()).expect("b")));
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool");
    assert_eq!(a, one.install(|| hash(&run(&src, &HtetOptions::default()).expect("c"))));
}

mod props {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        #![proptest_config(sm_testkit::strategies::config(40))]
        #[test]
        fn random_relabelings_of_a_block_validate_and_keep_volume(
            n in prop::array::uniform3(1u32..4),
            seed in 0u64..1000,
        ) {
            let src = hex_block_with_edges(n, [1.0; 3], &mut SerialAlloc::new()).unwrap();
            let nv = src.vertex_count();
            let mut rank: Vec<u32> = (0..nv as u32).collect();
            let mut s = seed.wrapping_mul(2862933555777941757).wrapping_add(3037000493);
            for i in (1..nv).rev() {
                s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                rank.swap(i, (s >> 33) as usize % (i + 1));
            }
            let r = run(&src, &HtetOptions { root: Some(&rank), ..HtetOptions::default() }).unwrap();
            prop_assert!(validate(&r.finished.mesh).is_empty());
            let want = f64::from(n[0] * n[1] * n[2]);
            prop_assert!((volume(&r.finished.mesh) - want).abs() < 1e-12);
        }
    }
}
