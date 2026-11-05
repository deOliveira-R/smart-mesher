//! W3b-0 fixtures: two-volume and nested hex meshes, `with_sets`, degenerate rays, tiled relief.

use sm_core::EntityKind;
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::{CellIx, FaceIx, MeshEntityKind, MeshRead, SerialAlloc, SetMembers, VertexIx, cell_volume, face_area_vector};
use sm_testkit::check::*;
use sm_testkit::meshes::{self, MeshFixture, SetSpec};
use sm_testkit::rays;
use sm_testkit::surf3::polyhedron_volume;
use sm_testkit::topo;
use std::collections::{BTreeMap, BTreeSet};

fn assert_valid(f: &MeshFixture) {
    let m = &f.mesh;
    mesh_valid(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
    orientation_rule(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
    closed_cells(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
    positive_volumes(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
    watertight(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
    classification_complete(m).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
    assert_eq!(m.cell_count(), f.cell_volumes.len());
    let total: f64 = (0..m.cell_count()).map(|c| cell_volume(m, CellIx::new(c))).sum();
    assert!((total - f.volume).abs() < 1e-12 * f.volume, "{}: {total} vs {}", f.name, f.volume);
    let boundary: Vec<FaceIx> = (0..m.face_count()).map(FaceIx::new).filter(|f| m.face_right(*f).is_none()).collect();
    assert_eq!(boundary.len(), f.boundary_faces);
    let area: f64 = boundary.iter().map(|f| face_area_vector(m, *f).norm()).sum();
    assert!((area - f.area).abs() < 1e-12 * f.area, "{}: area {area} vs {}", f.name, f.area);
    for (c, v) in f.cell_volumes.iter().enumerate() {
        assert!((cell_volume(m, CellIx::new(c)) - v).abs() < 1e-12 * v, "{} cell {c}", f.name);
    }
}

fn set_cells(f: &MeshFixture, name: &str) -> Vec<usize> {
    match &f.mesh.set_by_name(name).unwrap_or_else(|| panic!("no set {name}")).members {
        SetMembers::Cells(v) => v.iter().map(|c| c.get()).collect(),
        other => panic!("set {name} is {other:?}"),
    }
}

fn centroid(m: &dyn MeshRead, c: usize) -> [f64; 3] {
    let corners = m.cell_corners(CellIx::new(c)).expect("template cell");
    let p = m.coords();
    let mut s = [0.0; 3];
    for v in corners {
        s = [s[0] + p.x[v.get()], s[1] + p.y[v.get()], s[2] + p.z[v.get()]];
    }
    s.map(|x| x / corners.len() as f64)
}

fn volume_of_cell(m: &dyn MeshRead, c: usize) -> u64 {
    let cl = m.classification(MeshEntityKind::Cell, c as u32);
    assert_eq!(cl.entity.kind(), EntityKind::TopoVolume);
    cl.entity.serial()
}

#[test]
fn hex_block_two_volumes_has_the_documented_volumes_and_sets() {
    let (n, h, split) = ([4u32, 3, 2], [0.5, 1.0, 2.0], 1u32);
    let f = meshes::hex_block_two_volumes(n, h, split).unwrap();
    assert_valid(&f);
    let m = &f.mesh;
    let n1n2 = (n[1] * n[2]) as usize;
    assert_eq!(m.cell_count(), 24);
    assert_eq!(m.vertex_count(), 5 * 4 * 3);
    // Quad skin and the documented counts.
    assert_eq!(f.boundary_faces, 2 * (4 * 3 + 3 * 2 + 4 * 2));
    assert!((0..m.face_count()).all(|i| m.face_vertices(FaceIx::new(i)).len() == 4));
    assert_eq!(f.volume, 2.0 * 3.0 * 4.0);
    // Sets: volume_1 = cells with i < split, volume_2 the rest; volumes add up exactly.
    let (s1, s2) = (set_cells(&f, "volume_1"), set_cells(&f, "volume_2"));
    assert_eq!((s1.len(), s2.len()), (split as usize * n1n2, (4 - split) as usize * n1n2));
    let vol = |s: &[usize]| s.iter().map(|c| cell_volume(m, CellIx::new(*c))).sum::<f64>();
    assert!((vol(&s1) - 6.0).abs() < 1e-12 && (vol(&s2) - 18.0).abs() < 1e-12);
    assert!(s1.iter().all(|c| centroid(m, *c)[0] < split as f64 * h[0]));
    assert!(s2.iter().all(|c| centroid(m, *c)[0] > split as f64 * h[0]));
    // The sets agree with the TopoVolume classes.
    assert!(s1.iter().all(|c| volume_of_cell(m, *c) == 1) && s2.iter().all(|c| volume_of_cell(m, *c) == 2));
    // The interface: n1 n2 two-cell quads of area h1 h2 between the volumes, on a single TopoFace.
    let mut iface = BTreeSet::new();
    let mut classes = BTreeSet::new();
    for i in 0..m.face_count() {
        let fx = FaceIx::new(i);
        let Some(r) = m.face_right(fx) else { continue };
        if volume_of_cell(m, m.face_left(fx).get()) != volume_of_cell(m, r.get()) {
            assert!((face_area_vector(m, fx).norm() - h[1] * h[2]).abs() < 1e-12);
            let cl = m.classification(MeshEntityKind::Face, i as u32);
            assert_eq!(cl.entity.kind(), EntityKind::TopoFace);
            iface.insert(i);
            classes.insert(cl.entity.serial());
        }
    }
    assert_eq!(iface.len(), n1n2);
    assert_eq!(classes.len(), 1);
}

#[test]
fn hex_block_two_volumes_rejects_bad_input() {
    assert!(meshes::hex_block_two_volumes([4, 2, 2], [1.0; 3], 0).is_err());
    assert!(meshes::hex_block_two_volumes([4, 2, 2], [1.0; 3], 4).is_err());
    assert!(meshes::hex_block_two_volumes([4, 0, 2], [1.0; 3], 2).is_err());
    assert!(meshes::hex_block_two_volumes([4, 2, 2], [1.0, 0.0, 1.0], 2).is_err());
    assert!(meshes::hex_block_two_volumes([4, 2, 2], [1.0, f64::NAN, 1.0], 2).is_err());
}

#[test]
fn nested_shell_volumes_has_a_core_and_a_shell() {
    let f = meshes::nested_shell_volumes().unwrap();
    assert_valid(&f);
    let m = &f.mesh;
    assert_eq!((m.cell_count(), m.vertex_count(), f.boundary_faces), (27, 64, 54));
    assert_eq!(set_cells(&f, "volume_1"), vec![13]);
    let shell = set_cells(&f, "volume_2");
    assert_eq!(shell.len(), 26);
    assert!(!shell.contains(&13));
    assert_eq!(centroid(m, 13), [1.5, 1.5, 1.5]);
    assert!(shell.iter().all(|c| volume_of_cell(m, *c) == 2));
    // Six interface quads between core and shell, each on its own TopoFace; 12 TopoFaces in all.
    let mut iface = BTreeSet::new();
    let mut classes = BTreeSet::new();
    for i in 0..m.face_count() {
        let fx = FaceIx::new(i);
        let serial = m.classification(MeshEntityKind::Face, i as u32).entity.serial();
        if let Some(r) = m.face_right(fx)
            && volume_of_cell(m, m.face_left(fx).get()) != volume_of_cell(m, r.get())
        {
            iface.insert(serial);
        } else if m.face_right(fx).is_some() {
            continue;
        }
        classes.insert(serial);
    }
    assert_eq!(iface.len(), 6);
    assert_eq!(classes.len(), 12);
}

#[test]
fn with_sets_adds_sets_without_touching_the_mesh() {
    let base = MeshFixture {
        name: "hex_block",
        mesh: hex_block([2, 2, 2], [0.5; 3], &mut SerialAlloc::new()).unwrap(),
        cell_volumes: vec![0.125; 8],
        volume: 1.0,
        area: 6.0,
        boundary_faces: 24,
    };
    let specs = [
        SetSpec::new("low", SetMembers::Cells((0..4).map(CellIx::new).collect())).labeled(7),
        SetSpec::new("corner", SetMembers::Vertices(vec![VertexIx::new(0), VertexIx::new(26)])),
        SetSpec::new("patch", SetMembers::Faces(vec![FaceIx::new(0)])),
    ];
    let f = meshes::with_sets(base.clone(), &specs).unwrap();
    let m = &f.mesh;
    assert_eq!(m.sets().len(), 3);
    let low = m.set_by_name("low").unwrap();
    assert_eq!((low.members.len(), low.label, low.members.contains(3), low.members.contains(4)), (4, Some(7), true, false));
    assert_eq!(m.set_by_name("corner").unwrap().members.kind(), MeshEntityKind::Vertex);
    assert_eq!(m.set_by_name("patch").unwrap().members.kind(), MeshEntityKind::Face);
    assert!(m.sets().iter().all(|s| s.id.kind() == EntityKind::MeshSet));
    assert_eq!(m.sets().iter().map(|s| s.id).collect::<BTreeSet<_>>().len(), 3);
    // Same mesh otherwise.
    assert_eq!(base.mesh.coords().x, m.coords().x);
    assert_eq!(base.mesh.face_count(), m.face_count());
    assert_valid(&f);
    // A second call keeps the old sets and mints fresh ids.
    let g = meshes::with_sets(f.clone(), &[SetSpec::new("top", SetMembers::Cells(vec![CellIx::new(7)]))]).unwrap();
    assert_eq!(g.mesh.sets().len(), 4);
    assert_eq!(g.mesh.sets().iter().map(|s| s.id).collect::<BTreeSet<_>>().len(), 4);
    // Errors: duplicate name, out-of-range member, unsorted members, duplicate label.
    assert!(meshes::with_sets(f.clone(), &[SetSpec::new("low", SetMembers::Cells(vec![]))]).is_err());
    assert!(meshes::with_sets(base.clone(), &[SetSpec::new("x", SetMembers::Cells(vec![CellIx::new(8)]))]).is_err());
    assert!(meshes::with_sets(base, &[SetSpec::new("x", SetMembers::Cells(vec![CellIx::new(3), CellIx::new(1)]))]).is_err());
    assert!(meshes::with_sets(f, &[SetSpec::new("a", SetMembers::Cells(vec![CellIx::new(0)])).labeled(7)]).is_err());
    // Stacks on the volume sets of a two-volume fixture.
    let two = meshes::hex_block_two_volumes([3, 2, 2], [1.0; 3], 1).unwrap();
    let two = meshes::with_sets(two, &[SetSpec::new("first", SetMembers::Cells(vec![CellIx::new(0)]))]).unwrap();
    assert_eq!(two.mesh.sets().len(), 3);
}

// ------------------------------------------------------------------------------------ rays

#[test]
fn degenerate_grid_rays_hit_vertices_and_run_along_kuhn_edges() {
    let n = 3u32;
    let rs = rays::degenerate_grid(n);
    assert_eq!(rs.len(), 7 * 4 * 4 * 4);
    assert_eq!(rs, rays::degenerate_grid(n));
    assert_eq!(rays::degenerate_grid(0).len(), 7 * 8);
    // Edges of tet_block(n, 1) by lattice coordinates.
    let blk = meshes::tet_block([n; 3], [1.0; 3]).unwrap();
    let m = &blk.mesh;
    let c = m.coords();
    let pos = |v: VertexIx| [c.x[v.get()], c.y[v.get()], c.z[v.get()]].map(|x| x as i64);
    let mut edges: BTreeSet<([i64; 3], [i64; 3])> = BTreeSet::new();
    for cell in 0..m.cell_count() {
        let t = m.cell_corners(CellIx::new(cell)).unwrap();
        for a in t {
            for b in t {
                if a.get() < b.get() {
                    let (pa, pb) = (pos(*a), pos(*b));
                    edges.insert((pa.min(pb), pa.max(pb)));
                }
            }
        }
    }
    let dirs: BTreeSet<[i64; 3]> = edges.iter().map(|(a, b)| [b[0] - a[0], b[1] - a[1], b[2] - a[2]]).collect();
    assert_eq!(dirs, rays::KUHN_DIRECTIONS.iter().map(|d| d.map(i64::from)).collect());
    let (mut edge_rays, mut hits) = (0, BTreeSet::new());
    for (o, d) in &rs {
        let (od, dd) = ([o.x, o.y, o.z], [d.x, d.y, d.z]);
        assert!(od.iter().chain(&dd).all(|x| x.fract() == 0.0), "exact integers");
        let v = [od[0] + dd[0], od[1] + dd[1], od[2] + dd[2]].map(|x| x as i64);
        assert!(v.iter().all(|x| (0..=n as i64).contains(x)), "origin + d is a lattice vertex");
        hits.insert((v, dd.map(|x| x as i64)));
        let w = [v[0] + dd[0] as i64, v[1] + dd[1] as i64, v[2] + dd[2] as i64];
        if w.iter().all(|x| (0..=n as i64).contains(x)) {
            assert!(edges.contains(&(v.min(w), v.max(w))), "{v:?} -> {w:?} is a mesh edge");
            edge_rays += 1;
        }
    }
    // Every (vertex, direction) pair once; the ray continues along an edge unless it leaves the box.
    assert_eq!(hits.len(), rs.len());
    let inside: usize = rays::KUHN_DIRECTIONS.iter().map(|d| d.iter().map(|c| 4 - *c as usize).product::<usize>()).sum();
    assert_eq!(edge_rays, inside);
}

// ------------------------------------------------------------------------------------ relief

#[test]
fn tiled_relief_is_a_closed_cube_with_independent_dented_tiles() {
    let (n, k) = (16u32, 8u32);
    let fx = topo::tiled_relief(n);
    assert_eq!(fx.name, "tiled_relief");
    let flat = sm_testkit::surf3::tri_cube(n);
    let nn = n as usize;
    assert_eq!((fx.points.len(), fx.tris.len()), (6 * nn * nn + 2, 12 * nn * nn));
    assert_eq!(fx.tris, flat.tris);
    // Closed: every directed edge occurs once and so does its reverse.
    let mut dir: BTreeMap<(u32, u32), u32> = BTreeMap::new();
    for t in &fx.tris {
        for r in 0..3 {
            *dir.entry((t[r], t[(r + 1) % 3])).or_default() += 1;
        }
    }
    assert!(dir.iter().all(|(&(a, b), c)| *c == 1 && dir.get(&(b, a)) == Some(&1)));
    // Only top-face tile interiors moved, each down by less than depth * k / n.
    let lim = 0.9 * f64::from(k) / f64::from(n);
    let mut moved = 0;
    for (p, q) in fx.points.iter().zip(&flat.points) {
        if p != q {
            moved += 1;
            assert_eq!((p[0], p[1]), (q[0], q[1]));
            assert!(q[2] == 1.0 && p[2] < 1.0 && p[2] >= 1.0 - lim, "{p:?}");
            let (ix, iy) = ((p[0] * f64::from(n)).round() as u32, (p[1] * f64::from(n)).round() as u32);
            assert!(ix % k != 0 && iy % k != 0);
        }
    }
    assert_eq!(moved, ((n / k) * (k - 1)).pow(2) as usize);
    // Volume = 1 - dents; each top triangle (last 2 n^2) removes (projected area) x (mean depth).
    let mut dent = 0.0;
    for t in &fx.tris[10 * nn * nn..] {
        let p = t.map(|i| fx.points[i as usize]);
        let area = ((p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0])).abs() / 2.0;
        dent += area * (3.0 - p[0][2] - p[1][2] - p[2][2]) / 3.0;
    }
    let got = fx.volumes[0].enclosed_volume;
    assert!(dent > 0.0 && (got - (1.0 - dent)).abs() < 1e-12, "{got} vs {}", 1.0 - dent);
    assert_eq!(got, polyhedron_volume(&fx.points, &fx.oriented_tris(0)));
    // Rounding up to a multiple of 8, determinism, and depth 0 is the flat cube.
    assert_eq!(topo::tiled_relief(9).points.len(), 6 * 256 + 2);
    assert_eq!(topo::tiled_relief(n).points, fx.points);
    let flat0 = topo::tiled_relief_with(n, k, 2, 0.0);
    assert_eq!(flat0.points, flat.points);
    assert!((flat0.volumes[0].enclosed_volume - 1.0).abs() < 1e-12);
}
