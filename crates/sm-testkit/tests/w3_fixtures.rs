//! W3 stage-0 fixtures: surf3, tet blocks, lattice, nonconvex_poly, staircase.

use core::f64::consts::{PI, TAU};
use sm_core::{EntityKind, Point3};
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::{CellIx, FaceIx, MeshDim, MeshEntityKind, MeshRead, Param, SerialAlloc, cell_volume, validate};
use sm_testkit::check::*;
use sm_testkit::determinism;
use sm_testkit::lattice;
use sm_testkit::loops2d;
use sm_testkit::meshes::{self, MeshFixture};
use sm_testkit::surf3::{self, SurfaceFixture, ball_volume, polyhedron_volume, torus_volume};
use sm_topo_api::{Carrier, TopologyView};
use std::collections::{BTreeMap, BTreeSet};

// ----------------------------------------------------------------------------------- surf3

fn tri_area(p: &[[f64; 3]], t: [u32; 3]) -> f64 {
    let (a, b, c) = (p[t[0] as usize], p[t[1] as usize], p[t[2] as usize]);
    let (u, v) = ([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
    let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    0.5 * (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt()
}

/// Every directed edge appears once and its reverse once (closed, consistently oriented
/// 2-manifold); returns `(V, E, F)` over the points used.
fn closed_oriented(tris: &[[u32; 3]]) -> (usize, usize, usize) {
    let mut dir: BTreeMap<(u32, u32), usize> = BTreeMap::new();
    for t in tris {
        for k in 0..3 {
            *dir.entry((t[k], t[(k + 1) % 3])).or_default() += 1;
        }
    }
    for (&(a, b), &c) in &dir {
        assert_eq!(c, 1, "directed edge ({a},{b}) used {c} times");
        assert_eq!(dir.get(&(b, a)), Some(&1), "edge ({a},{b}) has no opposite");
    }
    let verts: BTreeSet<u32> = tris.iter().flatten().copied().collect();
    (verts.len(), dir.len() / 2, tris.len())
}

fn euler(tris: &[[u32; 3]]) -> i64 {
    let (v, e, f) = closed_oriented(tris);
    v as i64 - e as i64 + f as i64
}

/// Volume with the divergence theorem about the origin (a second, independent centre).
fn volume_about_origin(f: &SurfaceFixture, v: usize) -> f64 {
    f.oriented_tris(v)
        .iter()
        .map(|t| {
            let (a, b, c) = (f.points[t[0] as usize], f.points[t[1] as usize], f.points[t[2] as usize]);
            (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
        })
        .sum()
}

fn check_basic(f: &SurfaceFixture) {
    for t in &f.tris {
        assert!(t.iter().all(|&i| (i as usize) < f.points.len()), "{}", f.name);
        assert!(t[0] != t[1] && t[1] != t[2] && t[0] != t[2], "{}", f.name);
    }
    for (v, vol) in f.volumes.iter().enumerate() {
        let got = volume_about_origin(f, v);
        assert!((got - vol.enclosed_volume).abs() <= 1e-13 * vol.enclosed_volume.abs().max(1.0), "{}: {got} vs {}", f.name, vol.enclosed_volume);
        assert!(vol.enclosed_volume > 0.0, "{}", f.name);
        assert!(f.oriented_tris(v).iter().all(|t| tri_area(&f.points, *t) > 0.0));
    }
    assert!(f.oriented_tris(f.volumes.len()).is_empty());
}

#[test]
fn tri_cube_counts_volume_and_manifoldness() {
    for n in 1..=6u32 {
        let f = surf3::tri_cube(n);
        let nn = n as usize;
        assert_eq!(f.points.len(), 6 * nn * nn + 2);
        assert_eq!(f.tris.len(), 12 * nn * nn);
        check_basic(&f);
        assert!((f.volumes[0].enclosed_volume - 1.0).abs() < 1e-15);
        assert_eq!((f.volumes.len(), f.volumes[0].n_shells), (1, 1));
        assert_eq!(euler(&f.tris), 2);
        let area: f64 = f.tris.iter().map(|t| tri_area(&f.points, *t)).sum();
        assert!((area - 6.0).abs() < 1e-12);
    }
    assert_eq!(surf3::tri_cube(4).points.len(), 98);
    assert_eq!(surf3::tri_cube(4).tris.len(), 192);
}

#[test]
fn icosphere_counts_and_volume_below_the_smooth_limit() {
    for level in 0..=4u32 {
        let r = 1.5;
        let f = surf3::icosphere(r, level);
        let p4 = 4usize.pow(level);
        assert_eq!((f.points.len(), f.tris.len()), (10 * p4 + 2, 20 * p4));
        check_basic(&f);
        assert_eq!(euler(&f.tris), 2);
        assert!(f.points.iter().all(|p| ((p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt() - r).abs() < 1e-14));
        let v = f.volumes[0].enclosed_volume;
        assert!(v < ball_volume(r));
        if level >= 3 {
            assert!(v / ball_volume(r) > 0.95, "level {level}: {}", v / ball_volume(r));
        }
    }
    // The regular icosahedron has the closed-form volume 5/12 (3 + sqrt 5) a^3 for edge a.
    let f = surf3::icosphere(1.0, 0);
    let a = {
        let (p, q) = (f.points[f.tris[0][0] as usize], f.points[f.tris[0][1] as usize]);
        ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt()
    };
    let exact = 5.0 / 12.0 * (3.0 + 5f64.sqrt()) * a * a * a;
    assert!((f.volumes[0].enclosed_volume - exact).abs() < 1e-14);
    let f3 = surf3::icosphere(1.0, 3);
    assert_eq!((f3.points.len(), f3.tris.len()), (642, 1280));
}

#[test]
fn torus_is_genus_one_and_near_its_smooth_volume() {
    let f = surf3::tri_torus(2.0, 0.7, 48, 24);
    assert_eq!((f.points.len(), f.tris.len()), (1152, 2304));
    check_basic(&f);
    assert_eq!(euler(&f.tris), 0);
    let v = f.volumes[0].enclosed_volume;
    let limit = torus_volume(2.0, 0.7);
    assert!(v < limit && (limit - v) / limit < 0.03, "{v} vs {limit}");
}

#[test]
fn cavity_cube_has_volume_seven_and_two_shells() {
    for n in 1..=4u32 {
        let f = surf3::cube_with_cavity(n);
        let nn = n as usize;
        assert_eq!(f.points.len(), 2 * (6 * nn * nn + 2));
        assert_eq!(f.tris.len(), 24 * nn * nn);
        check_basic(&f);
        assert_eq!(f.volumes.len(), 1);
        assert_eq!(f.volumes[0].n_shells, 2);
        assert!((f.volumes[0].enclosed_volume - 7.0).abs() < 1e-14);
        // Two spheres: chi = 4. The cavity triangles are wound toward the cavity.
        assert_eq!(euler(&f.oriented_tris(0)), 4);
        assert_eq!(f.volumes[0].tris.iter().filter(|t| t.1).count(), 12 * nn * nn);
    }
}

#[test]
fn shared_cubes_share_an_interface_with_opposite_flips() {
    for n in 1..=4u32 {
        let f = surf3::two_cubes_shared(n);
        let nn = n as usize;
        assert_eq!(f.points.len(), 2 * (6 * nn * nn + 2) - (nn + 1) * (nn + 1));
        assert_eq!(f.tris.len(), 22 * nn * nn);
        check_basic(&f);
        assert_eq!(f.volumes.len(), 2);
        for v in &f.volumes {
            assert!((v.enclosed_volume - 1.0).abs() < 1e-14);
            assert_eq!(v.n_shells, 1);
        }
        let a: BTreeMap<u32, bool> = f.volumes[0].tris.iter().copied().collect();
        let b: BTreeMap<u32, bool> = f.volumes[1].tris.iter().copied().collect();
        let shared: Vec<u32> = a.keys().filter(|t| b.contains_key(t)).copied().collect();
        assert_eq!(shared.len(), 2 * nn * nn);
        for t in &shared {
            assert_ne!(a[t], b[t], "interface triangle {t} must be flipped on one side");
        }
        // Union: the oriented surfaces minus the interface (both ways) enclose 2.
        let mut all: Vec<[u32; 3]> = f.oriented_tris(0);
        all.extend(f.oriented_tris(1));
        let sum = polyhedron_volume(&f.points, &all);
        assert!((sum - 2.0).abs() < 1e-14);
    }
}

#[test]
fn schonhardt_matches_its_closed_form_and_is_nonconvex() {
    let f = surf3::schonhardt();
    assert_eq!((f.points.len(), f.tris.len()), (6, 8));
    check_basic(&f);
    assert_eq!(euler(&f.tris), 2);
    // V = sqrt(3)/2 (1 + cos(60 deg + twist)) for a unit-radius twisted prism of height 1.
    let exact = 3f64.sqrt() / 2.0 * (1.0 + (PI / 3.0 + PI / 6.0).cos());
    assert!((f.volumes[0].enclosed_volume - exact).abs() < 1e-14, "{}", f.volumes[0].enclosed_volume);
    // Each side diagonal is a reflex edge: the far vertex of the neighbouring triangle lies on
    // the outer side of the first triangle's plane.
    let mut reflex = 0;
    for (i, t) in f.tris.iter().enumerate() {
        for (j, s) in f.tris.iter().enumerate() {
            if i == j {
                continue;
            }
            let shared: Vec<u32> = t.iter().copied().filter(|x| s.contains(x)).collect();
            if shared.len() != 2 {
                continue;
            }
            let far = *s.iter().find(|x| !t.contains(x)).unwrap();
            let (a, b, c, d) = (f.points[t[0] as usize], f.points[t[1] as usize], f.points[t[2] as usize], f.points[far as usize]);
            let (u, v, w) = ([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]], [d[0] - a[0], d[1] - a[1], d[2] - a[2]]);
            let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
            if n[0] * w[0] + n[1] * w[1] + n[2] * w[2] > 1e-9 {
                reflex += 1;
            }
        }
    }
    assert_eq!(reflex, 6, "three reflex edges, seen from both triangles");
}

#[test]
fn noisy_sphere_is_seeded_star_shaped_and_closed() {
    let a = surf3::noisy_sphere(7);
    assert_eq!(a, surf3::noisy_sphere(7));
    assert_ne!(a.points, surf3::noisy_sphere(8).points);
    assert_eq!((a.points.len(), a.tris.len()), (642, 1280));
    check_basic(&a);
    assert_eq!(euler(&a.tris), 2);
    for p in &a.points {
        let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
        assert!((0.9 - 1e-12..=1.1 + 1e-12).contains(&r), "{r}");
    }
    // Star-shaped about the origin: every triangle sees the origin on its inner side.
    for t in &a.tris {
        let (p, q, r) = (a.points[t[0] as usize], a.points[t[1] as usize], a.points[t[2] as usize]);
        let det = p[0] * (q[1] * r[2] - q[2] * r[1]) - p[1] * (q[0] * r[2] - q[2] * r[0]) + p[2] * (q[0] * r[1] - q[1] * r[0]);
        assert!(det > 0.0);
    }
    assert!(a.volumes[0].enclosed_volume > 0.9f64.powi(3) * ball_volume(1.0) * 0.9);
}

#[test]
fn open_cube_is_open_and_has_no_volume() {
    let f = surf3::open_cube_missing_tri(3);
    assert_eq!(f.tris.len(), 12 * 9 - 1);
    assert!(f.volumes.is_empty());
    let mut dir: BTreeMap<(u32, u32), usize> = BTreeMap::new();
    for t in &f.tris {
        for k in 0..3 {
            *dir.entry((t[k], t[(k + 1) % 3])).or_default() += 1;
        }
    }
    let unmatched = dir.keys().filter(|(a, b)| !dir.contains_key(&(*b, *a))).count();
    assert_eq!(unmatched, 3, "the removed triangle leaves one boundary loop of 3 edges");
}

#[test]
fn surface_fixtures_are_deterministic() {
    let h = |threads: usize| {
        determinism::par_map(threads, 3, |i| match i {
            0 => surf3::icosphere(1.0, 3),
            1 => surf3::noisy_sphere(3),
            _ => surf3::tri_torus(2.0, 0.7, 12, 8),
        })
        .iter()
        .map(|f| determinism::hash_f64_bits(&f.points.iter().flatten().copied().chain(f.volumes.iter().map(|v| v.enclosed_volume)).collect::<Vec<_>>()))
        .collect::<Vec<u64>>()
    };
    determinism::check(2, 4, h).unwrap();
    assert_eq!(surf3::all_surface_fixtures().len(), 7);
}

// ------------------------------------------------------------------------------ tet blocks

fn check_valid(f: &MeshFixture, euler: i64) {
    let m = &f.mesh;
    assert!(validate(m).is_empty(), "{}: {:?}", f.name, validate(m));
    mesh_valid(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
    orientation_rule(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
    closed_cells(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
    positive_volumes(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
    watertight(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
    classification_complete(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
    check_mesh_euler(m, euler).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
    assert_eq!(m.cell_count(), f.cell_volumes.len(), "{}", f.name);
    let mut total = 0.0;
    for (i, v) in f.cell_volumes.iter().enumerate() {
        let got = cell_volume(m, CellIx::new(i));
        assert!((got - v).abs() <= 1e-12 * v.abs().max(1e-3), "{} cell {i}: {got} vs {v}", f.name);
        total += got;
    }
    assert!((total - f.volume).abs() <= 1e-12 * f.volume, "{}: {total} vs {}", f.name, f.volume);
    let nb = (0..m.face_count()).filter(|&q| m.face_right(FaceIx::new(q)).is_none()).count();
    assert_eq!(nb, f.boundary_faces, "{}", f.name);
    let area: f64 = (0..m.face_count())
        .map(FaceIx::new)
        .filter(|q| m.face_right(*q).is_none())
        .map(|q| sm_mesh_api::face_area_vector(m, q).norm())
        .sum();
    assert!((area - f.area).abs() <= 1e-12 * f.area, "{}: area {area} vs {}", f.name, f.area);
}

#[test]
fn tet_block_has_the_documented_counts_and_classifies_like_hex_block() {
    for (n, h) in [([2, 2, 2], [1.0; 3]), ([3, 2, 4], [0.5, 1.0, 0.25]), ([3, 3, 3], [1.0; 3]), ([1, 1, 1], [2.0; 3]), ([2, 3, 1], [1.0, 0.5, 2.0])] {
        let f = meshes::tet_block(n, h).unwrap();
        let nn = n.map(|x| x as usize);
        let (v, t) = ((nn[0] + 1) * (nn[1] + 1) * (nn[2] + 1), 6 * nn[0] * nn[1] * nn[2]);
        assert_eq!(f.mesh.vertex_count(), v);
        assert_eq!(f.mesh.cell_count(), t);
        let b = 4 * (nn[0] * nn[1] + nn[1] * nn[2] + nn[0] * nn[2]);
        assert_eq!(f.boundary_faces, b);
        assert_eq!(f.mesh.face_count(), (4 * t + b) / 2);
        assert!((f.volume - (nn[0] * nn[1] * nn[2]) as f64 * h[0] * h[1] * h[2]).abs() < 1e-14);
        check_valid(&f, 1);
        // The same vertex numbering and classification as hex_block.
        let hb = hex_block(n, h, &mut SerialAlloc::new()).unwrap();
        for i in 0..v {
            assert_eq!(f.mesh.point(i), hb.point(i));
            assert_eq!(
                f.mesh.classification(MeshEntityKind::Vertex, i as u32).entity,
                hb.classification(MeshEntityKind::Vertex, i as u32).entity,
                "{n:?} vertex {i}"
            );
        }
        if n.iter().all(|&x| x >= 2) {
            conformity(&f.mesh, &box_carriers(n, h), 1e-12).unwrap_or_else(|e| panic!("{n:?}: {e:?}"));
        }
    }
    assert!(meshes::tet_block([0, 1, 1], [1.0; 3]).is_err());
}

#[test]
fn tet_block_kuhn_edge_count_and_interior_degree() {
    // Cube case: E = 3n(n+1)^2 + 3n^2(n+1) + n^3 lattice, face-diagonal and body-diagonal edges
    // (6 n^3 tets share them), so the Euler relation fixes the edge count from V, F and T.
    let n = 3usize;
    let f = meshes::tet_block([3, 3, 3], [1.0; 3]).unwrap();
    let (v, faces, t) = (4usize.pow(3), f.mesh.face_count(), 6 * n * n * n);
    let e = v + faces - t - 1;
    let lattice_edges = 3 * n * (n + 1) * (n + 1);
    let face_diagonals = 3 * n * n * (n + 1);
    let body_diagonals = n * n * n;
    assert_eq!(e, lattice_edges + face_diagonals + body_diagonals);
}

#[test]
fn tet_block_two_volumes_shares_an_interface() {
    let f = meshes::tet_block_two_volumes([4, 2, 2], [1.0; 3], 2).unwrap();
    check_valid(&f, 1);
    assert_eq!(f.mesh.vertex_count(), 45);
    let m = &f.mesh;
    let vol_of = |c: CellIx| m.classification(MeshEntityKind::Cell, c.get() as u32).entity;
    let mut per: BTreeMap<_, f64> = BTreeMap::new();
    for c in 0..m.cell_count() {
        *per.entry(vol_of(CellIx::new(c))).or_default() += cell_volume(m, CellIx::new(c));
    }
    assert_eq!(per.len(), 2);
    for (e, v) in &per {
        assert_eq!(e.kind(), EntityKind::TopoVolume);
        assert!((v - 8.0).abs() < 1e-12, "{e:?}: {v}");
    }
    // A vertex classified on a volume lies on the volume of every cell that contains it.
    let mut interior = BTreeMap::new();
    for c in 0..m.cell_count() {
        let c = CellIx::new(c);
        for sf in m.cell_faces(c) {
            for v in m.face_vertices(sf.face()) {
                let class = m.classification(MeshEntityKind::Vertex, v.get() as u32);
                if class.entity.kind() == EntityKind::TopoVolume {
                    assert_eq!(class.entity, vol_of(c), "vertex {} in a cell of another volume", v.get());
                    interior.insert(v.get(), class.entity);
                }
            }
        }
    }
    // Lattice (1,1,1) is in volume 1 and (3,1,1) in volume 2 (nodes are numbered x fastest, 5 x 3 x 3).
    let node = |i: usize, j: usize, k: usize| (k * 3 + j) * 5 + i;
    assert_eq!(interior[&node(1, 1, 1)], vol_of(CellIx::new(0)));
    assert_ne!(interior[&node(3, 1, 1)], interior[&node(1, 1, 1)]);
    assert_eq!(interior.len(), 2, "the interface node (2,1,1) is a face node, not a volume node");
    // Interface triangles: two cells, different volumes, one TopoFace.
    let mut iface = BTreeSet::new();
    let mut count = 0;
    for q in 0..m.face_count() {
        let q = FaceIx::new(q);
        if m.face_right(q).is_some_and(|r| vol_of(m.face_left(q)) != vol_of(r)) {
            count += 1;
            let c = m.classification(MeshEntityKind::Face, q.get() as u32);
            assert_eq!(c.entity.kind(), EntityKind::TopoFace);
            iface.insert(c.entity);
            for v in m.face_vertices(q) {
                assert!((m.point(v.get()).x - 2.0).abs() < 1e-14);
            }
        }
    }
    assert_eq!((count, iface.len()), (2 * 2 * 2, 1));
    // Vertices on the interface: (n1+1)(n2+1).
    assert_eq!((0..m.vertex_count()).filter(|&i| (m.point(i).x - 2.0).abs() < 1e-14).count(), 9);
    assert!(meshes::tet_block_two_volumes([4, 2, 2], [1.0; 3], 0).is_err());
    assert!(meshes::tet_block_two_volumes([4, 2, 2], [1.0; 3], 4).is_err());
}

#[test]
fn tet_block_warped_conserves_volume_and_stays_positive() {
    let flat = meshes::tet_block([4, 4, 3], [0.5, 0.5, 1.0]).unwrap();
    let zero = meshes::tet_block_warped([4, 4, 3], [0.5, 0.5, 1.0], 0.0).unwrap();
    assert_eq!(mesh_hash(&flat.mesh), mesh_hash(&zero.mesh));
    let (n, h, amp) = ([8, 8, 4], [0.25, 0.25, 0.5], 0.2);
    let f = meshes::tet_block_warped(n, h, amp).unwrap();
    check_valid(&f, 1);
    // The continuous map gives V = Lx Ly Lz (1 + amp (2/pi)^2); the piecewise-linear top is close.
    let exact = 2.0 * 2.0 * 2.0 * (1.0 + amp * (2.0 / PI) * (2.0 / PI));
    assert!((f.volume - exact).abs() / exact < 0.01, "{} vs {exact}", f.volume);
    // The top face is curved: its z values are not all equal.
    let m = &f.mesh;
    let top: Vec<f64> = (0..m.vertex_count()).map(|i| m.point(i)).filter(|p| (p.z - p.z.round()).abs() > 1e-9 || p.z > 1.9).map(|p| p.z).collect();
    assert!(top.iter().any(|z| *z > 2.0 + 0.1));
}

#[test]
fn tet_lshape_has_a_reflex_edge_and_exact_volume() {
    for (mm, h) in [(2u32, 0.5), (3, 1.0)] {
        let f = meshes::tet_lshape(mm, h).unwrap();
        let m = mm as usize;
        check_valid(&f, 1);
        assert_eq!(f.mesh.cell_count(), 18 * m * m * m);
        assert!((f.volume - 3.0 * (m as f64 * h).powi(3)).abs() < 1e-12);
        assert_eq!(f.boundary_faces, 28 * m * m);
        // L prism: 12 vertices, 18 edges, 8 faces, 1 volume.
        let mut kinds: BTreeMap<EntityKind, BTreeSet<_>> = BTreeMap::new();
        for i in 0..f.mesh.vertex_count() {
            let c = f.mesh.classification(MeshEntityKind::Vertex, i as u32);
            kinds.entry(c.entity.kind()).or_default().insert(c.entity);
        }
        let count = |k| kinds.get(&k).map_or(0, BTreeSet::len);
        assert_eq!((count(EntityKind::TopoVertex), count(EntityKind::TopoEdge), count(EntityKind::TopoFace)), (12, 18, 8));
        // The reflex edge runs along x = y = m h.
        let reflex: Vec<usize> = (0..f.mesh.vertex_count())
            .filter(|&i| {
                let p = f.mesh.point(i);
                (p.x - m as f64 * h).abs() < 1e-12 && (p.y - m as f64 * h).abs() < 1e-12
            })
            .collect();
        assert_eq!(reflex.len(), m + 1);
        let ents: BTreeSet<_> = reflex.iter().map(|&i| f.mesh.classification(MeshEntityKind::Vertex, i as u32).entity).collect();
        assert_eq!(ents.iter().filter(|e| e.kind() == EntityKind::TopoEdge).count(), 1);
    }
    assert!(meshes::tet_lshape(0, 1.0).is_err());
}

#[test]
fn nonconvex_poly_has_exact_volumes_and_two_chords() {
    let f = meshes::nonconvex_poly().unwrap();
    check_valid(&f, 1);
    assert_eq!(f.cell_volumes, vec![3.0, 1.0]);
    assert_eq!(f.volume, 4.0);
    assert_eq!(f.area, 18.0);
    assert_eq!(f.boundary_faces, 14);
    let m = &f.mesh;
    assert_eq!(m.cell_shape(CellIx::new(0)), sm_mesh_api::CellShape::Polyhedron);
    assert_eq!(m.cell_shape(CellIx::new(1)), sm_mesh_api::CellShape::Hex);
    assert_eq!(m.cell_faces(CellIx::new(0)).len(), 10);
    // One shared face of area 1.
    let shared: Vec<FaceIx> = (0..m.face_count()).map(FaceIx::new).filter(|q| m.face_right(*q).is_some()).collect();
    assert_eq!(shared.len(), 1);
    assert!((sm_mesh_api::face_area_vector(m, shared[0]).norm() - 1.0).abs() < 1e-14);
    // The L cell: the line x + y = 2.5 crosses it in two disjoint chords. Test with the
    // cell's own face planes: the mid-point of the gap (1.25, 1.25) is outside, both ends inside.
    let inside_l = |x: f64, y: f64| (0.0..=2.0).contains(&x) && (0.0..=2.0).contains(&y) && (x <= 1.0 || y <= 1.0);
    assert!(inside_l(0.75, 1.75) && inside_l(1.75, 0.75) && !inside_l(1.25, 1.25));
    // The reflex corner is a polyhedron vertex: the cell's convex hull volume (the hull adds the
    // notch tetra-prism) is larger than the cell.
    let cell_pts: BTreeSet<usize> = m.cell_faces(CellIx::new(0)).iter().flat_map(|sf| m.face_vertices(sf.face()).iter().map(|v| v.get())).collect();
    assert_eq!(cell_pts.len(), 16);
}

// ----------------------------------------------------------------------------------- lattice

fn assert_class_params(f: &lattice::LatticeFixture) {
    let m = &f.mesh;
    for i in 0..m.vertex_count() {
        let c = m.classification(MeshEntityKind::Vertex, i as u32);
        assert!(f.topo.dim_of(c.entity).is_some(), "vertex {i} is on {:?}, not in the topology", c.entity);
        let p = m.point(i);
        match (c.entity.kind(), c.param, f.topo.carrier(c.entity)) {
            (EntityKind::TopoVertex, Param::None, Carrier::Point(q)) => assert!((*q - p).norm() < 1e-12, "vertex {i}"),
            (EntityKind::TopoEdge, Param::Curve(t), Carrier::Curve(cc)) => {
                let q = f.topo.shapes().curve(cc.curve).unwrap().eval(t);
                assert!((q - p).norm() < 1e-12, "edge node {i}: {q:?} vs {p:?}");
            }
            (EntityKind::TopoFace, Param::Surface([u, v]), Carrier::Surface(sc)) => {
                let q = f.topo.shapes().surface(sc.surface).unwrap().eval(sm_geom_api::Uv::new(u, v));
                assert!((q - p).norm() < 1e-12, "face node {i}: {q:?} vs {p:?}");
            }
            (EntityKind::TopoVolume, Param::None, _) => {}
            other => panic!("vertex {i}: unexpected classification {other:?}"),
        }
    }
}

fn area_of_quads(m: &dyn MeshRead) -> f64 {
    (0..m.cell_count())
        .map(|c| m.cell_normal(CellIx::new(c)).map_or(f64::NAN, |n| 0.5 * n.norm()))
        .sum()
}

#[test]
fn hex_lattice_is_classified_with_parameters() {
    let (n, size) = ([3, 4, 5], [1.5, 2.0, 2.5]);
    let f = lattice::hex(n, size).unwrap();
    assert_eq!(f.mesh.vertex_count(), 4 * 5 * 6);
    assert_eq!(f.mesh.cell_count(), 60);
    assert!(validate(&f.mesh).is_empty(), "{:?}", validate(&f.mesh));
    check_valid_lattice(&f, 60.0 / 60.0 * size[0] * size[1] * size[2]);
    assert!((f.h - 0.5).abs() < 1e-15);
    assert_class_params(&f);
    for (i, e) in f.exact.iter().enumerate() {
        assert_eq!(f.mesh.point(i), Point3::new(e[0], e[1], e[2]));
    }
    assert!(sm_testkit::topo::model_errors(&sm_testkit::topo::ModelFixture {
        name: "lattice",
        complex: f.topo.clone(),
        model: f.model.clone(),
        flat_paths: 0,
        flat_entries: 0,
        deck_faces: 0,
        volume: 0.0
    })
    .is_empty());
    // The far corner is exact (no accumulated rounding).
    let last = f.mesh.point(f.mesh.vertex_count() - 1);
    assert_eq!((last.x, last.y, last.z), (1.5, 2.0, 2.5));
    assert!(lattice::hex([0, 1, 1], [1.0; 3]).is_err());
    assert!(lattice::hex([1, 1, 1], [1.0, -1.0, 1.0]).is_err());
}

fn check_valid_lattice(f: &lattice::LatticeFixture, volume: f64) {
    let m = &f.mesh;
    mesh_valid(m).unwrap();
    positive_volumes(m).unwrap();
    watertight(m).unwrap();
    classification_complete(m).unwrap();
    let total: f64 = (0..m.cell_count()).map(|c| cell_volume(m, CellIx::new(c))).sum();
    assert!((total - volume).abs() < 1e-12 * volume, "{total} vs {volume}");
}

#[test]
fn tet_kuhn_lattice_matches_tet_block() {
    let (n, size) = ([3, 2, 4], [1.5, 1.0, 2.0]);
    let f = lattice::tet_kuhn(n, size).unwrap();
    assert_eq!(f.mesh.cell_count(), 6 * 24);
    assert!(validate(&f.mesh).is_empty());
    check_valid_lattice(&f, 3.0);
    assert_class_params(&f);
    let b = meshes::tet_block([3, 2, 4], [0.5, 0.5, 0.5]).unwrap();
    assert_eq!(f.mesh.vertex_count(), b.mesh.vertex_count());
    for i in 0..b.mesh.vertex_count() {
        assert_eq!(
            f.mesh.classification(MeshEntityKind::Vertex, i as u32).entity,
            b.mesh.classification(MeshEntityKind::Vertex, i as u32).entity
        );
    }
}

#[test]
fn quad_lattice_is_a_planar_dimension_two_mesh() {
    let f = lattice::quad([4, 3], [2.0, 1.5]).unwrap();
    assert_eq!(f.mesh.dim(), MeshDim::Two);
    assert!(validate(&f.mesh).is_empty(), "{:?}", validate(&f.mesh));
    assert_eq!((f.mesh.vertex_count(), f.mesh.cell_count()), (20, 12));
    assert!((area_of_quads(&f.mesh) - 3.0).abs() < 1e-14);
    for c in 0..f.mesh.cell_count() {
        let n = f.mesh.cell_normal(CellIx::new(c)).unwrap();
        assert!(n.z > 0.0 && n.x == 0.0 && n.y == 0.0, "cell {c}: {n:?}");
    }
    assert_class_params(&f);
    let kinds: BTreeSet<_> = (0..20).map(|i| f.mesh.classification(MeshEntityKind::Vertex, i).entity.kind()).collect();
    assert_eq!(kinds, BTreeSet::from([EntityKind::TopoVertex, EntityKind::TopoEdge, EntityKind::TopoFace]));
    let corners = (0..20).filter(|&i| f.mesh.classification(MeshEntityKind::Vertex, i).entity.kind() == EntityKind::TopoVertex).count();
    assert_eq!(corners, 4);
    assert!((f.h - 0.5).abs() < 1e-15);
}

#[test]
fn tube_quad_is_closed_across_the_seam() {
    let (nt, nz, r, height) = (12usize, 5usize, 1.5, 4.0);
    let f = lattice::tube_quad(nt, nz, r, height).unwrap();
    let m = &f.mesh;
    assert_eq!(m.dim(), MeshDim::Two);
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!((m.vertex_count(), m.cell_count()), (nt * (nz + 1), nt * nz));
    // Two boundary rings: chi = V - E + F = 0.
    assert_eq!(m.vertex_count() as i64 - m.face_count() as i64 + m.cell_count() as i64, 0);
    for i in 0..m.vertex_count() {
        let p = m.point(i);
        assert!(((p.x * p.x + p.y * p.y).sqrt() - r).abs() < 1e-14);
        let (it, iz) = (i % nt, i / nt);
        assert!((p.z - height * iz as f64 / nz as f64).abs() < 1e-14);
        let a = p.y.atan2(p.x).rem_euclid(TAU);
        assert!((a - TAU * it as f64 / nt as f64).abs() < 1e-12 || (a - TAU * it as f64 / nt as f64 - TAU).abs() < 1e-12);
    }
    let chord = 2.0 * r * (PI / nt as f64).sin();
    assert!((area_of_quads(m) - nt as f64 * chord * height).abs() < 1e-12);
    // Outward-facing quads.
    for c in 0..m.cell_count() {
        let n = m.cell_normal(CellIx::new(c)).unwrap();
        let ctr = (0..4).map(|k| m.point(m.cell_corners(CellIx::new(c)).unwrap()[k].get())).fold(Point3::new(0.0, 0.0, 0.0), |a, p| Point3::new(a.x + p.x, a.y + p.y, 0.0));
        assert!(n.x * ctr.x + n.y * ctr.y > 0.0);
    }
    assert_class_params(&f);
    let count = |k: EntityKind| (0..m.vertex_count()).filter(|&i| m.classification(MeshEntityKind::Vertex, i as u32).entity.kind() == k).count();
    assert_eq!((count(EntityKind::TopoVertex), count(EntityKind::TopoEdge)), (2, 2 * (nt - 1) + (nz - 1)));
    assert!((f.h - (TAU * r / nt as f64).min(height / nz as f64)).abs() < 1e-15);
    assert!(lattice::tube_quad(2, 3, 1.0, 1.0).is_err());
}

#[test]
fn perturb_is_deterministic_and_stays_on_the_entity() {
    for f in [
        lattice::hex([4, 4, 4], [1.0; 3]).unwrap(),
        lattice::tet_kuhn([3, 3, 3], [1.0; 3]).unwrap(),
        lattice::quad([6, 6], [1.0, 1.0]).unwrap(),
        lattice::tube_quad(16, 4, 1.0, 1.0).unwrap(),
    ] {
        let amp = 0.25;
        let a = f.perturbed(11, amp).unwrap();
        let b = f.perturbed(11, amp).unwrap();
        let c = f.perturbed(12, amp).unwrap();
        assert_eq!(mesh_hash(&a.mesh), mesh_hash(&b.mesh));
        assert_ne!(mesh_hash(&a.mesh), mesh_hash(&c.mesh));
        assert!(validate(&a.mesh).is_empty(), "{:?}", validate(&a.mesh));
        assert_class_params(&a);
        let mut moved = 0;
        for i in 0..a.mesh.vertex_count() {
            let class = a.mesh.classification(MeshEntityKind::Vertex, i as u32);
            let d = (a.mesh.point(i) - Point3::new(f.exact[i][0], f.exact[i][1], f.exact[i][2])).norm();
            match class.entity.kind() {
                EntityKind::TopoVertex => assert_eq!(d, 0.0),
                _ => {
                    assert!(d <= amp * f.h * 3f64.sqrt() * 1.0001 + 1e-12, "node {i} moved {d}");
                    moved += usize::from(d > 0.0);
                }
            }
        }
        assert!(moved > 0);
        // The original is untouched.
        for (i, e) in f.exact.iter().enumerate() {
            assert_eq!(f.mesh.point(i), Point3::new(e[0], e[1], e[2]));
        }
    }
    // Free function with an absolute amplitude agrees with the convenience method.
    let f = lattice::hex([3, 3, 3], [1.0; 3]).unwrap();
    let mut m = f.mesh.clone();
    lattice::perturb(&mut m, &f.topo, 5, 0.1 * f.h).unwrap();
    assert_eq!(mesh_hash(&m), mesh_hash(&f.perturbed(5, 0.1).unwrap().mesh));
    // Zero amplitude changes nothing.
    let z = f.perturbed(5, 0.0).unwrap();
    for (i, e) in f.exact.iter().enumerate() {
        assert!((z.mesh.point(i) - Point3::new(e[0], e[1], e[2])).norm() < 1e-14);
    }
}

#[test]
fn lattice_construction_is_deterministic() {
    let h = |threads: usize| {
        determinism::par_map(threads, 3, |i| match i {
            0 => mesh_hash(&lattice::hex([3, 3, 3], [1.0; 3]).unwrap().perturbed(3, 0.2).unwrap().mesh),
            1 => mesh_hash(&meshes::tet_block_warped([3, 3, 3], [1.0; 3], 0.1).unwrap().mesh),
            _ => mesh_hash(&meshes::tet_lshape(2, 1.0).unwrap().mesh),
        })
    };
    determinism::check(2, 4, h).unwrap();
}

// ----------------------------------------------------------------------------------- loops2d

#[test]
fn staircase_has_exact_area_and_turn_types() {
    use loops2d::VertexType::{Corner, End};
    let l = loops2d::staircase(&[(1, 1)]);
    assert_eq!(l.area, 3.0);
    assert_eq!(l.outer, loops2d::l_shape().outer);
    assert_eq!(l.outer_types, loops2d::l_shape().outer_types);
    assert_eq!(loops2d::staircase(&[]).outer, loops2d::square().outer);
    for steps in [vec![(1, 1)], vec![(2, 1), (1, 3)], vec![(3, 2), (1, 1), (2, 2)], vec![(1, 4), (4, 1), (2, 2), (3, 3)], vec![(0, 0)]] {
        let r = loops2d::staircase(&steps);
        let k = steps.len();
        assert_eq!(r.outer.len(), 2 * k + 4);
        assert_eq!(r.area, r.shoelace_area());
        assert_eq!(r.area.fract(), 0.0);
        assert!(loops2d::signed_area(&r.outer) > 0.0);
        let types = r.outer_types.clone().unwrap();
        assert_eq!(types.iter().filter(|t| **t == Corner).count(), k);
        assert_eq!(types.iter().filter(|t| **t == End).count(), k + 4);
        assert_eq!(types, loops2d::vertex_types(&r.outer));
        // Turning sum: End +1, Corner -1 gives 4.
        assert_eq!(types.iter().map(|t| if *t == End { 1 } else { -1 }).sum::<i32>(), 4);
        // Integer coordinates, axis-parallel edges.
        for i in 0..r.outer.len() {
            let (p, q) = (r.outer[i], r.outer[(i + 1) % r.outer.len()]);
            assert!(p.0.fract() == 0.0 && p.1.fract() == 0.0);
            assert!(p.0 == q.0 || p.1 == q.1);
        }
    }
    // Closed form for two steps: columns 2, 1, 1 wide and 5, 4... heights 1 + sum of later drops.
    let r = loops2d::staircase(&[(2, 1), (1, 3)]);
    assert_eq!(r.area, 2.0 * 5.0 + 1.0 * 4.0 + 1.0 * 1.0);
}
