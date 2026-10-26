//! W3a geometry fixtures against their analytic invariants.

use core::f64::consts::{PI, SQRT_2};
use sha2::{Digest, Sha256};
use sm_mesh_api::{CellIx, CellShape, FaceIx, MeshRead, SetMembers, cell_volume};
use sm_testkit::boolean::{self, Rule};
use sm_testkit::check::*;
use sm_testkit::determinism::check as det_check;
use sm_testkit::topo::{self, FaceFixture, SweepFixture};
use sm_testkit::{loops2d, step, w3a};
use sm_topo_api::{Carrier, Dim, TopologyView, TopologyViewExt, euler_characteristic};

fn close(got: f64, want: f64, rel: f64, what: &str) {
    assert!((got - want).abs() <= rel * want.abs().max(1.0), "{what}: {got} vs {want}");
}

fn face_area(f: &FaceFixture) -> f64 {
    f.complex.face_moments(f.face).unwrap().area
}

fn counts(v: &dyn TopologyView) -> [usize; 5] {
    Dim::ALL.map(|d| v.cells(d).len())
}

#[test]
fn planar_faces_of_every_region() {
    for r in loops2d::all_regions() {
        let f = topo::planar_face(&r, 0.5).unwrap();
        topo_valid(&f.complex).unwrap_or_else(|e| panic!("{}: {e:?}", r.name));
        assert_eq!(f.loop_count(), 1 + r.holes.len(), "{}", r.name);
        assert_eq!(f.euler, 1 - r.holes.len() as i64);
        assert_eq!(euler_characteristic(&f.complex), 1, "{}: combinatorial V - E + F", r.name);
        let n = r.vertex_count();
        assert_eq!(counts(&f.complex), [n, n, 1, 0, 0], "{}", r.name);
        close(face_area(&f), r.area, 1e-12, r.name);
        // Edge i of a loop joins vertex i and vertex i + 1.
        for (vs, es) in f.vertices.iter().zip(&f.edges) {
            assert_eq!(vs.len(), es.len());
        }
    }
}

#[test]
fn planar_face_rejects_degenerate_loops() {
    let mut r = loops2d::square();
    r.outer.truncate(2);
    assert!(topo::planar_face(&r, 0.0).is_err());
}

#[test]
fn annulus_and_sphere_zone() {
    let a = topo::annulus_face(0.5, 2.0).unwrap();
    topo_valid(&a.complex).unwrap();
    assert_eq!(counts(&a.complex), [2, 2, 1, 0, 0]);
    assert_eq!(a.loop_count(), 2);
    close(face_area(&a), PI * (4.0 - 0.25), 1e-9, "annulus");
    assert!(topo::annulus_face(2.0, 1.0).is_err());

    let z = topo::sphere_zone(2.0, [0.3, 1.9], [-0.4, 0.9]).unwrap();
    topo_valid(&z.complex).unwrap();
    assert_eq!(counts(&z.complex), [4, 4, 1, 0, 0]);
    close(z.area, 4.0 * 1.6 * (0.9f64.sin() + 0.4f64.sin()), 1e-15, "closed form");
    close(face_area(&z), z.area, 1e-9, "sphere zone");
    assert!(topo::sphere_zone(1.0, [0.0, 7.0], [0.0, 1.0]).is_err());
    assert!(topo::sphere_zone(1.0, [0.0, 1.0], [0.0, 1.6]).is_err());
}

fn check_sweep(s: &SweepFixture, rel: f64) {
    let f = &s.topo;
    let name = f.name;
    assert_eq!(counts(&f.complex), f.counts, "{name}");
    assert_eq!(euler_characteristic(&f.complex), f.euler, "{name}");
    topo_valid(&f.complex).unwrap_or_else(|e| panic!("{name}: {e:?}"));
    let v = f.complex.volume_moments(s.volume).unwrap().volume;
    close(v, f.volumes[0], rel, &format!("{name} volume"));
    let area: f64 = f.complex.faces().iter().map(|x| f.complex.face_moments(*x).unwrap().area).sum();
    close(area, f.area, rel, &format!("{name} area"));
    assert_eq!(f.complex.faces().len(), 2 + s.linking.len());
    for id in [s.source, s.target].iter().chain(&s.linking) {
        assert!(f.complex.faces().contains(id), "{name}: face id");
    }
}

#[test]
fn extruded_l_has_the_documented_counts_and_volume() {
    let s = topo::extruded_l(2.0, 1.0, 2.0).unwrap();
    check_sweep(&s, 1e-12);
    assert_eq!(s.topo.counts, [12, 18, 8, 1, 1]);
    assert_eq!(s.linking.len(), 6);
    close(s.topo.volumes[0], 6.0, 1e-15, "L volume");
    close(s.topo.area, 22.0, 1e-15, "L area");
    close(face_of(&s, s.source), 3.0, 1e-12, "source area");
    // The re-entrant column (index 3) is shared by exactly two linking faces.
    let col = s.columns[3];
    let users = s.linking.iter().filter(|f| s.topo.complex.loops(**f).any(|l| l.keys().any(|k| k.edge == col))).count();
    assert_eq!(users, 2);
    assert!(topo::extruded_l(1.0, 1.0, 1.0).is_err());
}

fn face_of(s: &SweepFixture, f: sm_core::EntityId) -> f64 {
    s.topo.complex.face_moments(f).unwrap().area
}

#[test]
fn regular_prism_hexagon() {
    let s = topo::regular_prism(6, 1.0, 3.0).unwrap();
    check_sweep(&s, 1e-12);
    close(s.topo.volumes[0], 7.794228634059948, 1e-14, "hexagon prism");
    close(face_of(&s, s.source), 3.0 * 3.0f64.sqrt() / 2.0, 1e-12, "hexagon");
    assert_eq!(s.topo.counts, [12, 18, 8, 1, 1]);
    assert!(topo::regular_prism(2, 1.0, 1.0).is_err());
}

#[test]
fn twisted_prism_volume_and_bilinear_faces() {
    let s = topo::twisted_prism(2.0, 3.0, PI / 2.0).unwrap();
    check_sweep(&s, 1e-7);
    close(s.topo.volumes[0], 8.0, 1e-15, "closed form");
    // A zero twist is the plain box: volume side^2 h, planar faces.
    let b = topo::twisted_prism(2.0, 3.0, 0.0).unwrap();
    close(b.topo.volumes[0], 12.0, 1e-15, "box");
    close(b.topo.area, 8.0 + 4.0 * 6.0, 1e-12, "box area");
    // The twist is a rotation of the target: its vertices are the rotated source vertices.
    let v: Vec<_> = s
        .topo
        .complex
        .cells(Dim::Vertex)
        .iter()
        .filter_map(|v| match s.topo.complex.carrier(*v) {
            Carrier::Point(p) => Some(*p),
            _ => None,
        })
        .collect();
    assert!(v.iter().any(|p| (p.x - 1.0).abs() < 1e-12 && (p.y - 1.0).abs() < 1e-12 && p.z == 3.0 || (p.x + 1.0).abs() < 1e-12 && (p.y - 1.0).abs() < 1e-12 && p.z == 3.0));
}

#[test]
fn annular_sector_volumes() {
    for sweep in [PI / 2.0, 1.5 * PI] {
        let s = topo::annular_sector(1.0, 2.0, 1.0, sweep).unwrap();
        check_sweep(&s, 1e-9);
        close(s.topo.volumes[0], sweep / 2.0 * 3.0, 1e-15, "sector volume");
        assert_eq!(s.linking.len(), 4);
        assert_eq!(s.topo.counts, [8, 12, 6, 1, 1]);
        close(face_of(&s, s.source), 1.0, 1e-12, "source rectangle");
    }
    assert!(topo::annular_sector(1.0, 2.0, 1.0, 7.0).is_err());
}

#[test]
fn hollow_cylinder_volume_and_seams() {
    let s = topo::hollow_cylinder(1.0, 2.0, 2.0).unwrap();
    check_sweep(&s, 1e-9);
    close(s.topo.volumes[0], 6.0 * PI, 1e-15, "6 pi");
    assert_eq!(s.topo.counts, [4, 6, 4, 1, 1]);
    assert_eq!(s.topo.complex.loops(s.source).count(), 2, "annulus source has two loops");
    close(face_of(&s, s.source), 3.0 * PI, 1e-9, "annulus");
    assert!(topo::hollow_cylinder(2.0, 1.0, 1.0).is_err());
}

#[test]
fn pin_assembly_flattens_and_has_exact_volumes() {
    let f = topo::pin_assembly(2, 1.26, 0.41, 0.475, 3.0).unwrap();
    topo_valid(&f.complex).unwrap();
    assert!(topo::model_errors(&f).is_empty(), "{:?}", topo::model_errors(&f));
    let flat = sm_model_api::flatten(&f.model).unwrap();
    assert_eq!(flat.paths.len(), f.flat_paths);
    assert_eq!(flat.entries.len(), f.flat_entries);
    assert_eq!(f.flat_paths, 36);
    let e = topo::pin_assembly_expect(2, 0.41, 0.475, 3.0);
    assert_eq!(e.pins, 36);
    let mut vols: Vec<f64> = f.complex.volumes().iter().map(|v| f.complex.volume_moments(*v).unwrap().volume).collect();
    vols.sort_by(f64::total_cmp);
    let mut want = [e.fuel_volume, e.clad_volume];
    want.sort_by(f64::total_cmp);
    for (g, w) in vols.iter().zip(want) {
        close(*g, w, 1e-9, "pin volume");
    }
    close(vols.iter().sum::<f64>(), f.volume, 1e-9, "pin total");
    close(e.total_fuel, 36.0 * PI * 0.41 * 0.41 * 3.0, 1e-14, "fuel total");
    assert!(topo::pin_assembly(2, 0.9, 0.41, 0.475, 3.0).is_err(), "pins overlap");
    assert!(topo::pin_assembly(0, 1.26, 0.41, 0.475, 3.0).is_err());
}

#[test]
fn instanced_core_places_n_copies() {
    for n in [1usize, 5, 27, 64] {
        let f = topo::instanced_core(n).unwrap();
        let flat = sm_model_api::flatten(&f.model).unwrap();
        assert_eq!(flat.paths.len(), n);
        assert_eq!(flat.entries.len(), 2 * n);
        assert_eq!(f.flat_paths, n);
    }
    let f = topo::instanced_core(8).unwrap();
    topo_valid(&f.complex).unwrap();
    assert!(topo::model_errors(&f).is_empty());
    let total: f64 = f.complex.volumes().iter().map(|v| f.complex.volume_moments(*v).unwrap().volume).sum();
    close(total, 8.0, 1e-9, "brick with inclusion");
    assert!(topo::instanced_core(0).is_err());
}

#[test]
fn boolean_cases_are_self_consistent() {
    let cases = boolean::cases();
    assert_eq!(cases.len(), 6);
    for c in &cases {
        // Inclusion-exclusion between the operations of one case.
        let get = |r| c.expect(r).map(|e| e.volumes.iter().sum::<f64>());
        if let (Some(u), Some(i)) = (get(Rule::Unite), get(Rule::Intersect)) {
            close(u, c.a.volume() + c.b.volume() - i, 1e-12, &format!("{} union", c.name));
            if let Some(s) = get(Rule::Subtract) {
                close(s, c.a.volume() - i, 1e-12, &format!("{} subtract", c.name));
            }
        }
        for e in &c.expected {
            if let (Some([v, ed, fc]), Some(chi)) = (e.counts, e.euler) {
                assert_eq!(v as i64 - ed as i64 + fc as i64, chi, "{}", c.name);
            }
        }
    }
    let by = |n: &str| cases.iter().find(|c| c.name == n).unwrap();
    let cav = by("brick_with_cavity");
    close(cav.expected[0].volumes[0], cav.a.volume() - cav.b.volume(), 1e-14, "cavity");
    close(cav.expected[0].total_area, cav.a.area() + cav.b.area(), 1e-14, "cavity area");
    let p = by("partial_overlap_bricks");
    close(p.expected[0].total_area, p.a.area() + p.b.area() - 2.0, 1e-14, "shared face counted once");
    let h = by("brick_sphere_half");
    close(h.expect(Rule::Unite).unwrap().total_area, h.a.area() - PI + 2.0 * PI, 1e-14, "half sphere");
    // The intersection curve of the crossing cylinders lies on both lateral surfaces.
    for k in 0..16 {
        for sign in [1.0, -1.0] {
            let p = boolean::crossing_cylinders_curve_point(k as f64 * 0.4, sign);
            close(p[0] * p[0] + p[1] * p[1], 1.0, 1e-14, "on the r = 1 cylinder");
            close(p[1] * p[1] + p[2] * p[2], 0.25, 1e-14, "on the r = 0.5 cylinder");
        }
    }
}

fn sha256_hex(b: &[u8]) -> String {
    Sha256::digest(b).iter().map(|x| format!("{x:02x}")).collect()
}

#[test]
fn step_files_are_pinned_and_describe_their_solids() {
    assert_eq!(sha256_hex(w3a::STEP), "252eb1e6377ebd4090287def170c3c6442b0f0c9f286308cf75b037606bb463d");
    for s in step::ALL {
        assert_eq!(sha256_hex(s.bytes), s.sha256, "{}", s.name);
        let text = std::str::from_utf8(s.bytes).unwrap();
        assert!(text.starts_with("ISO-10303-21;"), "{}", s.name);
        assert!(text.contains(s.marker), "{}: {}", s.name, s.marker);
        assert_eq!(text.matches("MANIFOLD_SOLID_BREP").count(), s.solids, "{}", s.name);
    }
    close(step::CYLINDER_R2_H5.volume, 20.0 * PI, 1e-15, "cylinder");
    close(step::BOX_FILLET1.volume, 1000.0 - (1.0 - PI / 4.0) * 10.0, 1e-15, "fillet box");
    close(step::CONE_R2_H3.volume, PI * 4.0 * 3.0 / 3.0, 1e-15, "cone");
    close(step::TORUS_R5_R1.volume, 2.0 * PI * PI * 5.0, 1e-15, "torus");
    close(step::SPHERE_R3.volume, 4.0 / 3.0 * PI * 27.0, 1e-15, "sphere");
    close(step::W3A_PART.volume, 2000.0, 0.0, "part");
}

#[test]
fn w3a_constants_follow_from_the_part_geometry() {
    // V1: L polygon area, V2: triangle, V3: box.
    let l = [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (10.0, 10.0), (10.0, 20.0), (0.0, 20.0)];
    let tri = [(10.0, 10.0), (20.0, 10.0), (10.0, 20.0)];
    let a1 = loops2d::signed_area(&l);
    let a2 = loops2d::signed_area(&tri);
    assert_eq!([a1 * 4.0, a2 * 4.0, 100.0 * 6.0], w3a::VOLUMES);
    assert_eq!(w3a::VOLUMES.iter().sum::<f64>(), w3a::TOTAL_VOLUME);
    let b = [2.0 * a1 + 4.0 * loops2d::perimeter(&l), 2.0 * a2 + 4.0 * loops2d::perimeter(&tri), 2.0 * 100.0 + 4.0 * 10.0 * 6.0];
    for (got, want) in b.iter().zip(w3a::SIDESET_AREAS) {
        close(*got, want, 1e-14, "boundary area");
    }
    close(w3a::SIDESET_AREAS[1], 180.0 + 40.0 * SQRT_2, 1e-14, "V2 area");
    let shared = w3a::SHARED_FACE_AREAS;
    close(shared[0], 10.0 * 4.0, 0.0, "V1|V2 y = 10");
    close(shared[1], 10.0 * 4.0, 0.0, "V1|V2 x = 10");
    close(shared[2], a2, 0.0, "V2|V3 triangle");
    close(w3a::SIDESET_AREAS.iter().sum::<f64>() - 2.0 * shared.iter().sum::<f64>(), w3a::OUTER_AREA, 1e-14, "outer area");
    assert_eq!(8 + 5 + 7 - 3, w3a::FACES);
}

#[test]
fn mixed_all_shapes_is_valid_and_has_its_sets() {
    let f = w3a::mixed_all_shapes().unwrap();
    let m = &f.mesh;
    assert_eq!(m.cell_count(), 5);
    for (i, shape) in w3a::SHAPES.iter().enumerate() {
        assert_eq!(m.cell_shape(CellIx::new(i)), *shape, "cell {i}");
        let got = cell_volume(m, CellIx::new(i));
        close(got, f.cell_volumes[i], 1e-12, &format!("cell {i}"));
    }
    assert!(w3a::SHAPES.contains(&CellShape::Polyhedron));
    close(f.volume, 1.0 + 0.5 + 1.0 / 3.0 + 5.0 / 24.0 + 1.25, 1e-14, "total");
    mesh_valid(m).unwrap();
    orientation_rule(m).unwrap();
    closed_cells(m).unwrap();
    positive_volumes(m).unwrap();
    watertight(m).unwrap();
    classification_complete(m).unwrap();
    check_mesh_euler(m, 1).unwrap();
    let area: f64 = (0..m.face_count()).map(FaceIx::new).filter(|f| m.face_right(*f).is_none()).map(|f| sm_mesh_api::face_area_vector(m, f).norm()).sum();
    close(area, f.area, 1e-12, "boundary area");
    // Sets: one per shape, the polyhedron's sides, the bottom faces.
    for (i, name) in w3a::CELL_SETS.iter().enumerate() {
        let s = m.set_by_name(name).unwrap();
        assert_eq!(s.members, SetMembers::Cells(vec![CellIx::new(i)]), "{name}");
    }
    let sides = m.set_by_name(w3a::SIDE_SET).unwrap();
    assert_eq!(sides.members.len(), 6, "7 polyhedron faces, one glued to the hex");
    let bottom = m.set_by_name(w3a::FACE_SET).unwrap();
    assert_eq!(bottom.members.len(), 3, "hex, wedge, polyhedron");
}

#[test]
fn fixtures_are_deterministic_across_runs_and_threads() {
    let build = |_: usize| {
        let mut h = Vec::new();
        for r in loops2d::all_regions() {
            h.push(topo_hash(&topo::planar_face(&r, 0.0).unwrap().complex));
        }
        h.push(topo_hash(&topo::annulus_face(1.0, 2.0).unwrap().complex));
        h.push(topo_hash(&topo::sphere_zone(1.0, [0.0, 1.0], [0.0, 1.0]).unwrap().complex));
        for s in [
            topo::extruded_l(2.0, 1.0, 2.0).unwrap(),
            topo::twisted_prism(2.0, 3.0, 1.0).unwrap(),
            topo::annular_sector(1.0, 2.0, 1.0, 1.0).unwrap(),
            topo::hollow_cylinder(1.0, 2.0, 2.0).unwrap(),
            topo::regular_prism(5, 1.0, 1.0).unwrap(),
        ] {
            h.push(topo_hash(&s.topo.complex));
            h.push(s.topo.area.to_bits());
        }
        h.push(topo_hash(&topo::pin_assembly(2, 1.26, 0.41, 0.475, 3.0).unwrap().complex));
        h.push(topo_hash(&topo::instanced_core(4).unwrap().complex));
        h.push(mesh_hash(&w3a::mixed_all_shapes().unwrap().mesh));
        h
    };
    det_check(2, 4, build).unwrap();
}
