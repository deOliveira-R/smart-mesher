//! MOD-REFINE-U acceptance 1-4, 6, 7 on the pure pass (`sm_modify::refine`): analytic counts,
//! volumes, similarity classes, level maps and set carry-over, over the reference fixtures.

use std::collections::{BTreeMap, BTreeSet};

use sm_core::Point3;
use sm_mesh::MeshBuilder;
use sm_mesh_api::reference::{ReferenceMesh, hex_block_with_edges};
use sm_mesh_api::{
    CellIx, CellShape, Classification, FaceIx, FaceKind, LevelMap, MeshEdit, MeshRead, SerialAlloc, SetMembers, TagValues,
    cell_volume, face_area_vector, geometry_hash, topology_hash, validate, validate_classification,
};
use sm_modify::{Refined, refine};
use sm_quality::{Metric, Options, compute};
use sm_testkit::meshes::{CellSpec, build_mesh, mixed_transition, tet_block};

fn run(src: &dyn MeshRead) -> Refined<sm_mesh::Mesh> {
    refine(src, None, MeshBuilder::new(), &mut SerialAlloc::new()).expect("refine")
}

fn total_volume(m: &dyn MeshRead) -> f64 {
    (0..m.cell_count()).map(|c| cell_volume(m, CellIx::new(c))).sum()
}

/// `(V, E, F)` of the boundary surface.
fn boundary_euler(m: &dyn MeshRead) -> (usize, usize, usize) {
    let (mut v, mut e, mut f) = (BTreeSet::new(), BTreeSet::new(), 0);
    for fi in 0..m.face_count() {
        let fi = FaceIx::new(fi);
        if m.face_kind(fi) != FaceKind::Boundary {
            continue;
        }
        f += 1;
        let l = m.face_vertices(fi);
        for i in 0..l.len() {
            v.insert(l[i]);
            let (a, b) = (l[i], l[(i + 1) % l.len()]);
            e.insert(if a < b { (a, b) } else { (b, a) });
        }
    }
    (v.len(), e.len(), f)
}

fn boundary_area(m: &dyn MeshRead) -> f64 {
    (0..m.face_count())
        .filter(|&f| m.face_kind(FaceIx::new(f)) == FaceKind::Boundary)
        .map(|f| face_area_vector(m, FaceIx::new(f)).norm())
        .sum()
}

fn assert_valid(m: &dyn MeshRead) {
    let v = validate(m);
    assert!(v.is_empty(), "validate: {v:?}");
    assert!(validate_classification(m).is_empty());
}

/// Children of every parent, ascending.
fn children(r: &Refined<sm_mesh::Mesh>, n_parents: usize) -> Vec<Vec<usize>> {
    let mut out = vec![Vec::new(); n_parents];
    for (k, p) in r.coarse_of.iter().enumerate() {
        out[p.get()].push(k);
    }
    out
}

#[test]
fn a1_hex_block_counts_and_euler() {
    for n in [1u32, 2, 3] {
        let src = hex_block_with_edges([n, n, n], [1.0; 3], &mut SerialAlloc::new()).expect("block");
        let r = run(&src);
        let m = &r.finished.mesh;
        let n = n as usize;
        assert_eq!(m.vertex_count(), (2 * n + 1).pow(3), "vertices n={n}");
        assert_eq!(m.cell_count(), 8 * n.pow(3), "cells n={n}");
        let (v, e, f) = boundary_euler(m);
        assert_eq!(v as isize - e as isize + f as isize, 2, "Euler characteristic of the boundary, n={n}");
        assert_eq!(f, 6 * (2 * n) * (2 * n));
        assert!((total_volume(m) - (n.pow(3)) as f64).abs() < 1e-12);
        assert_valid(m);
        assert!(m.edges().is_some(), "an edge table in gives an edge table out");
    }
}

#[test]
fn a2_hex_children_are_exactly_an_eighth_on_a_parallelepiped() {
    // Shear and scale a 2x2x2 block by an affine map: every parent is a parallelepiped.
    let mut alloc = SerialAlloc::new();
    let mut src = hex_block_with_edges([2, 2, 2], [1.0; 3], &mut alloc).expect("block");
    let a = [[1.5, 0.25, 0.0], [0.0, 0.75, 0.5], [0.125, 0.0, 2.0]];
    for i in 0..src.vertex_count() {
        let p = src.point(i);
        let q = Point3::new(
            a[0][0] * p.x + a[0][1] * p.y + a[0][2] * p.z + 1.0,
            a[1][0] * p.x + a[1][1] * p.y + a[1][2] * p.z - 2.0,
            a[2][0] * p.x + a[2][1] * p.y + a[2][2] * p.z,
        );
        src.set_point(sm_mesh_api::PointIx::new(i), q).expect("set");
    }
    let r = run(&src);
    let ch = children(&r, src.cell_count());
    for (p, kids) in ch.iter().enumerate() {
        assert_eq!(kids.len(), 8);
        let vp = cell_volume(&src, CellIx::new(p));
        for &k in kids {
            let vk = cell_volume(&r.finished.mesh, CellIx::new(k));
            assert!((vk - vp / 8.0).abs() <= 1e-14 * vp, "child {k} of {p}: {vk} vs {}", vp / 8.0);
        }
    }
    assert!((total_volume(&r.finished.mesh) - total_volume(&src)).abs() <= 1e-14 * total_volume(&src));
    // Affine invariance of the quality: the children keep the parents' scaled Jacobian.
    let sj = |m: &dyn MeshRead| -> (f64, f64) {
        let c = compute(m, Metric::ScaledJacobian, &Options::default()).expect("sj");
        (c.iter().copied().fold(f64::INFINITY, f64::min), c.iter().copied().fold(f64::NEG_INFINITY, f64::max))
    };
    let (p0, p1) = sj(&src);
    let (c0, c1) = sj(&r.finished.mesh);
    assert!((p0 - c0).abs() < 1e-12 && (p1 - c1).abs() < 1e-12, "parent {p0}..{p1}, children {c0}..{c1}");
}

#[test]
fn a2_tet_corner_children_are_an_eighth_and_the_octahedron_is_half() {
    let fx = tet_block([2, 2, 2], [1.0; 3]).expect("tet block");
    let src = &fx.mesh;
    let r = run(src);
    assert_eq!(r.finished.mesh.cell_count(), 8 * src.cell_count());
    for (p, kids) in children(&r, src.cell_count()).iter().enumerate() {
        assert_eq!(kids.len(), 8);
        let vp = cell_volume(src, CellIx::new(p));
        let mut v: Vec<f64> = kids.iter().map(|&k| cell_volume(&r.finished.mesh, CellIx::new(k))).collect();
        v.sort_by(f64::total_cmp);
        let octa: f64 = v.iter().sum::<f64>() - 4.0 * vp / 8.0;
        let eighths = v.iter().filter(|x| (**x - vp / 8.0).abs() <= 1e-14 * vp).count();
        assert!(eighths >= 4, "at least the four corner tets are exactly V/8: {v:?}");
        assert!((octa - vp / 2.0).abs() <= 1e-14 * vp, "octahedron {octa} vs {}", vp / 2.0);
        assert!(v.iter().all(|x| *x > 0.0));
    }
    assert!((total_volume(&r.finished.mesh) - fx.volume).abs() <= 1e-13);
    assert_valid(&r.finished.mesh);
    let (v, e, f) = boundary_euler(&r.finished.mesh);
    assert_eq!(v as isize - e as isize + f as isize, 2);
}

/// Sorted edge-length ratios of a tet (shortest = 1), rounded for grouping.
fn signature(m: &dyn MeshRead, c: usize) -> Vec<i64> {
    let k = m.cell_corners(CellIx::new(c)).expect("corners");
    let mut l = Vec::new();
    for i in 0..4 {
        for j in i + 1..4 {
            let (a, b) = (m.point(k[i].get()), m.point(k[j].get()));
            l.push(((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)).sqrt());
        }
    }
    l.sort_by(f64::total_cmp);
    let s = l[0];
    l.iter().map(|x| ((x / s) * 1e9).round() as i64).collect()
}

#[test]
fn a3_kuhn_tet_stays_in_at_most_three_similarity_classes() {
    let fx = tet_block([2, 2, 2], [1.0; 3]).expect("tet block");
    let mut cur: Box<dyn MeshRead> = Box::new(fx.mesh.clone());
    let mut classes_per_level = Vec::new();
    for level in 1..=3 {
        let r = run(cur.as_ref());
        let m = r.finished.mesh;
        let sigs: BTreeSet<Vec<i64>> = (0..m.cell_count()).map(|c| signature(&m, c)).collect();
        // Group signatures that agree to 1e-9 relative (rounding at the 1e9 grid can split a class).
        let mut groups: Vec<Vec<i64>> = Vec::new();
        for s in sigs {
            if !groups.iter().any(|g| g.iter().zip(&s).all(|(a, b)| (a - b).abs() <= 5)) {
                groups.push(s);
            }
        }
        classes_per_level.push(groups.len());
        assert!(groups.len() <= 3, "level {level}: {} classes", groups.len());
        cur = Box::new(m);
    }
    eprintln!("a3 similarity classes per level (Kuhn tet block, shortest-diagonal rule): {classes_per_level:?}");
}

/// A 2x2x(layers) wedge block over a triangulated square.
fn wedge_block(nl: usize) -> ReferenceMesh {
    let n = 2usize;
    let mut pts = Vec::new();
    for k in 0..=nl {
        for j in 0..=n {
            for i in 0..=n {
                pts.push([i as f64, j as f64, k as f64]);
            }
        }
    }
    let ix = |i: usize, j: usize, k: usize| (k * (n + 1) + j) * (n + 1) + i;
    let mut cells = Vec::new();
    for k in 0..nl {
        for j in 0..n {
            for i in 0..n {
                for tri in [[(0, 0), (1, 0), (1, 1)], [(0, 0), (1, 1), (0, 1)]] {
                    let b: Vec<usize> = tri.iter().map(|(a, c)| ix(i + a, j + c, k)).collect();
                    let t: Vec<usize> = tri.iter().map(|(a, c)| ix(i + a, j + c, k + 1)).collect();
                    cells.push(CellSpec::Template(CellShape::Wedge, [b, t].concat()));
                }
            }
        }
    }
    build_mesh(&pts, &cells, &mut SerialAlloc::new()).expect("wedge block")
}

/// A 2x2x2 cube whose eight sub-cubes are each cut into six pyramids from their centre.
fn pyramid_block() -> ReferenceMesh {
    let mut pts: Vec<[f64; 3]> = Vec::new();
    let ix = |i: usize, j: usize, k: usize| (k * 3 + j) * 3 + i;
    for k in 0..3 {
        for j in 0..3 {
            for i in 0..3 {
                pts.push([i as f64, j as f64, k as f64]);
            }
        }
    }
    let mut cells = Vec::new();
    // Face loops of a unit cube with corners numbered like the hex template, outward from the cube
    // (counter-clockwise seen from outside) and read as a pyramid base looking at the apex.
    let faces: [[usize; 4]; 6] = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]];
    for k in 0..2 {
        for j in 0..2 {
            for i in 0..2 {
                let corner = |t: usize| {
                    let (x, y) = [(0, 0), (1, 0), (1, 1), (0, 1)][t % 4];
                    ix(i + x, j + y, k + t / 4)
                };
                let centre = pts.len();
                pts.push([i as f64 + 0.5, j as f64 + 0.5, k as f64 + 0.5]);
                for f in &faces {
                    // A pyramid base is counter-clockwise seen from the apex (inside the cube).
                    let base: Vec<usize> = f.iter().rev().map(|&t| corner(t)).collect();
                    cells.push(CellSpec::Template(CellShape::Pyramid, [base, vec![centre]].concat()));
                }
            }
        }
    }
    build_mesh(&pts, &cells, &mut SerialAlloc::new()).expect("pyramid block")
}

fn refine_and_check(src: &ReferenceMesh, per_parent: usize, volume: f64) -> Refined<sm_mesh::Mesh> {
    assert_valid(src);
    let r = run(src);
    let m = &r.finished.mesh;
    assert_eq!(m.cell_count(), per_parent * src.cell_count());
    assert!((total_volume(m) - volume).abs() <= 1e-12 * volume, "volume {} vs {volume}", total_volume(m));
    assert!((total_volume(src) - volume).abs() <= 1e-12 * volume);
    assert_valid(m);
    assert!((boundary_area(m) - boundary_area(src)).abs() <= 1e-12 * boundary_area(src), "boundary area is unchanged");
    let (v, e, f) = boundary_euler(m);
    assert_eq!(v as isize - e as isize + f as isize, 2);
    r
}

#[test]
fn a4_wedge_block_refines_1_to_8() {
    let src = wedge_block(2);
    let r = refine_and_check(&src, 8, 8.0);
    assert!(children(&r, src.cell_count()).iter().all(|k| k.len() == 8));
    assert!((0..r.finished.mesh.cell_count()).all(|c| r.finished.mesh.cell_shape(CellIx::new(c)) == CellShape::Wedge));
    for (p, kids) in children(&r, src.cell_count()).iter().enumerate() {
        let vp = cell_volume(&src, CellIx::new(p));
        for &k in kids {
            assert!((cell_volume(&r.finished.mesh, CellIx::new(k)) - vp / 8.0).abs() <= 1e-14 * vp);
        }
    }
}

#[test]
fn a4_pyramid_block_refines_1_to_10() {
    let src = pyramid_block();
    assert_eq!(src.cell_count(), 48);
    let r = refine_and_check(&src, 10, 8.0);
    let m = &r.finished.mesh;
    let (mut pyr, mut tet) = (0, 0);
    for c in 0..m.cell_count() {
        match m.cell_shape(CellIx::new(c)) {
            CellShape::Pyramid => pyr += 1,
            CellShape::Tet => tet += 1,
            s => panic!("{s:?}"),
        }
    }
    assert_eq!((pyr, tet), (6 * 48, 4 * 48));
    // Child volumes: four corner pyramids and the top one are V/8, the inverted one V/8, tets V/16... check sums.
    for (p, kids) in children(&r, src.cell_count()).iter().enumerate() {
        assert_eq!(kids.len(), 10);
        let vp = cell_volume(&src, CellIx::new(p));
        let sum: f64 = kids.iter().map(|&k| cell_volume(m, CellIx::new(k))).sum();
        assert!((sum - vp).abs() <= 1e-14);
        let mut v: Vec<f64> = kids.iter().map(|&k| cell_volume(m, CellIx::new(k))).collect();
        v.sort_by(f64::total_cmp);
        // 4 tets of V/16, 6 pyramids: 5 of V/8 (corners and top) and the inverted one of V/8.
        for x in &v[..4] {
            assert!((x - vp / 16.0).abs() <= 1e-14, "tet {x} vs {}", vp / 16.0);
        }
        for x in &v[4..] {
            assert!((x - vp / 8.0).abs() <= 1e-14, "pyramid {x} vs {}", vp / 8.0);
        }
    }
}

#[test]
fn a4_mixed_transition_refines_conformally() {
    let fx = mixed_transition().expect("mixed");
    let n = fx.mesh.cell_count();
    assert_eq!(n, 4);
    // Every vertex of the fixture is a model vertex, so the edges along model edges need explicit
    // classes: give each one its own TopoEdge.
    let mut mesh = fx.mesh.clone();
    let mut explicit = Vec::new();
    loop {
        match mesh.clone().ensure_edges(&mut SerialAlloc::new(), &explicit) {
            Ok(()) => break,
            Err(sm_mesh_api::MeshError::UnclassifiedEdge(a, b)) => {
                let e = sm_core::EntityId::new(sm_core::EntityKind::TopoEdge, explicit.len() as u64 + 1).expect("id");
                explicit.push((a, b, Classification::on(e)));
            }
            Err(e) => panic!("{e}"),
        }
    }
    mesh.ensure_edges(&mut SerialAlloc::new(), &explicit).expect("edges");
    let r = run(&mesh);
    let m = &r.finished.mesh;
    assert_eq!(m.cell_count(), 8 + 8 + 10 + 8);
    assert!((total_volume(m) - fx.volume).abs() <= 1e-13);
    assert_valid(m);
    assert!((boundary_area(m) - fx.area).abs() <= 1e-12);
}

#[test]
fn a4_level_map_with_face_map_is_accepted_and_total() {
    let src = hex_block_with_edges([2, 2, 2], [1.0; 3], &mut SerialAlloc::new()).expect("block");
    let r = run(&src);
    let m = &r.finished.mesh;
    let map = LevelMap::new(m, &src, r.coarse_of.clone(), Some(r.face_map.clone())).expect("LevelMap::new accepts the face map");
    assert_eq!(map.fine_len(), 64);
    assert_eq!(map.coarse_len(), 8);
    for c in 0..8 {
        assert_eq!(map.fines_of(CellIx::new(c)).len(), 8);
    }
    // Every fine face on a parent face maps to it: four children per parent boundary or interior face.
    let mapped = r.face_map.iter().filter(|&&x| x != sm_mesh_api::NO_CELL).count();
    assert_eq!(mapped, 4 * src.face_count());
    // Restriction of the fine volumes gives the coarse volumes.
    let fv: Vec<f64> = (0..m.cell_count()).map(|c| cell_volume(m, CellIx::new(c))).collect();
    assert_eq!(map.restrict_sum(&fv).expect("restrict"), vec![1.0; 8]);
}

#[test]
fn a6_sets_tags_and_blocks_carry_over() {
    let mut alloc = SerialAlloc::new();
    let mut src = hex_block_with_edges([2, 2, 2], [1.0; 3], &mut alloc).expect("block");
    // A block of the four cells with x < 1, a side set on the face x = 0 (outward cell side) and
    // a node set on the same patch, plus a cell tag.
    let low: Vec<CellIx> = (0..src.cell_count()).filter(|&c| src.cell_corners(CellIx::new(c)).expect("hex").iter().all(|v| src.point(v.get()).x <= 1.0)).map(CellIx::new).collect();
    assert_eq!(low.len(), 4);
    src.add_set("low_x", SetMembers::Cells(low.clone()), &mut alloc).expect("block set");
    let left: Vec<FaceIx> = (0..src.face_count()).map(FaceIx::new).filter(|&f| src.face_kind(f) == FaceKind::Boundary && src.face_vertices(f).iter().all(|v| src.point(v.get()).x == 0.0)).collect();
    assert_eq!(left.len(), 4);
    src.add_set("xmin", SetMembers::Faces(left.clone()), &mut alloc).expect("face set");
    let sides: Vec<sm_mesh_api::SignedFace> = left.iter().map(|&f| sm_mesh_api::SignedFace::new(f, sm_mesh_api::Side::Left)).collect();
    src.add_set("xmin_sides", SetMembers::Sides(sides), &mut alloc).expect("side set");
    let nodes: Vec<sm_mesh_api::VertexIx> = (0..src.vertex_count()).map(sm_mesh_api::VertexIx::new).filter(|v| src.point(v.get()).x == 0.0).collect();
    assert_eq!(nodes.len(), 9);
    src.add_set("xmin_nodes", SetMembers::Vertices(nodes), &mut alloc).expect("node set");
    let mat: Vec<i64> = (0..src.cell_count() as i64).map(|c| 10 + c).collect();
    src.set_tag("mat", sm_mesh_api::MeshEntityKind::Cell, TagValues::I64(mat.clone())).expect("tag");

    let r = run(&src);
    let m = &r.finished.mesh;
    assert_valid(m);
    let set = |n: &str| m.set_by_name(n).unwrap_or_else(|| panic!("set {n}")).members.clone();
    // Block cell counts multiply by 8; the block is still the x < 1 half.
    let SetMembers::Cells(cells) = set("low_x") else { panic!("cells") };
    assert_eq!(cells.len(), 32);
    for c in &cells {
        assert!(m.cell_corners(*c).expect("corners").iter().all(|v| m.point(v.get()).x <= 1.0));
    }
    // The side set keeps its area (4 patches of 1 x 1 become 16 of 1/2 x 1/2): exactly 4.
    let SetMembers::Faces(fs) = set("xmin") else { panic!("faces") };
    assert_eq!(fs.len(), 16);
    let area: f64 = fs.iter().map(|&f| face_area_vector(m, f).norm()).sum();
    assert!((area - 4.0).abs() < 1e-14, "area {area}");
    let SetMembers::Sides(ss) = set("xmin_sides") else { panic!("sides") };
    assert_eq!(ss.len(), 16);
    for s in &ss {
        // The side's cell is a child of a block cell (x < 1).
        let cell = match s.side() {
            sm_mesh_api::Side::Left => m.face_left(s.face()),
            sm_mesh_api::Side::Right => m.face_right(s.face()).expect("right"),
        };
        assert!(cells.contains(&cell));
    }
    let SetMembers::Vertices(vs) = set("xmin_nodes") else { panic!("vertices") };
    assert_eq!(vs.len(), 25, "the 5 x 5 nodes of the refined patch");
    assert!(vs.iter().all(|v| m.point(v.get()).x == 0.0));
    // The cell tag is inherited.
    let t = m.tag(sm_mesh_api::MeshEntityKind::Cell, "mat").expect("tag").values.as_i64().expect("i64").to_vec();
    for (k, p) in r.coarse_of.iter().enumerate() {
        assert_eq!(t[k], mat[p.get()]);
    }
    // Classification: every boundary face is on the TopoFace of its parent face.
    assert_eq!(m.classified_entities().len(), src.classified_entities().len());
}

#[test]
fn a7_the_pass_is_deterministic_for_any_thread_count() {
    let fx = tet_block([3, 2, 2], [1.0; 3]).expect("tets");
    let hashes = |m: &sm_mesh::Mesh| (topology_hash(m), geometry_hash(m));
    let a = hashes(&run(&fx.mesh).finished.mesh);
    let b = hashes(&run(&fx.mesh).finished.mesh);
    assert_eq!(a, b);
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool");
    let c = one.install(|| hashes(&run(&fx.mesh).finished.mesh));
    let four = rayon::ThreadPoolBuilder::new().num_threads(4).build().expect("pool");
    let d = four.install(|| hashes(&run(&fx.mesh).finished.mesh));
    assert_eq!(a, c);
    assert_eq!(a, d);
    // The coarse map and face map are equally independent of threads.
    let r1 = one.install(|| run(&fx.mesh));
    let r4 = four.install(|| run(&fx.mesh));
    assert_eq!(r1.coarse_of, r4.coarse_of);
    assert_eq!(r1.face_map, r4.face_map);
}

#[test]
fn polyhedra_and_high_order_are_refused_with_a_reason() {
    let fx = sm_testkit::meshes::dual_patch().expect("dual patch");
    let Err(e) = refine(&fx.mesh, None, MeshBuilder::new(), &mut SerialAlloc::new()) else { panic!("refused") };
    assert!(e.to_string().contains("dualize again"), "{e}");
    // Atomic: the allocator is untouched on a refusal.
    let mut alloc = SerialAlloc::new();
    let before: BTreeMap<_, _> = alloc.state().collect();
    let _ = refine(&fx.mesh, None, MeshBuilder::new(), &mut alloc);
    assert_eq!(before, alloc.state().collect());
}

#[test]
fn high_order_meshes_are_refused() {
    let mut fx = sm_testkit::meshes::single_hex().expect("hex");
    fx.mesh.add_extra_node(Point3::new(0.5, 0.5, 0.5)).expect("extra node");
    let Err(e) = refine(&fx.mesh, None, MeshBuilder::new(), &mut SerialAlloc::new()) else { panic!("refused") };
    assert!(matches!(e, sm_modify::RefineError::HighOrder), "{e}");
}

mod props {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        #![proptest_config(sm_testkit::strategies::config(24))]
        #[test]
        fn hex_blocks_of_any_shape_refine_to_valid_meshes(
            n in prop::array::uniform3(1u32..4),
            h in prop::array::uniform3(0.25f64..2.0),
        ) {
            let src = hex_block_with_edges(n, h, &mut SerialAlloc::new()).unwrap();
            let r = run(&src);
            let m = &r.finished.mesh;
            let cells = 8 * (n[0] * n[1] * n[2]) as usize;
            prop_assert_eq!(m.cell_count(), cells);
            prop_assert!(validate(m).is_empty());
            let vol = (n[0] as f64 * h[0]) * (n[1] as f64 * h[1]) * (n[2] as f64 * h[2]);
            prop_assert!((total_volume(m) - vol).abs() <= 1e-12 * vol);
            let sj = compute(m, Metric::ScaledJacobian, &Options::default()).unwrap();
            prop_assert!(sj.iter().all(|x| (x - 1.0).abs() < 1e-12));
            let _: Classification = m.cell_class(CellIx::new(0));
        }
    }
}
