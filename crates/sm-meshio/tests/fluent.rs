//! Fluent `.msh` export: an independent reader of the written text, orientation by signed cell
//! volumes and closure against the fixtures' analytic volumes, element-type labels, zones and
//! ids, option errors, determinism, and (gated by `SM_CONFORMANCE=1`) OpenFOAM
//! `fluent3DMeshToFoam` + `checkMesh` as the oracle (no Fluent licence).

mod common;

use sm_mesh_api::reference::hex_block;
use sm_mesh_api::*;
use sm_meshio::fluent::*;
use sm_testkit::meshes::{MeshFixture, dual_patch, hex_block_2x2x2, hexagonal_prism, mixed_transition, single_hex, single_pyramid, single_tet, single_wedge, split_face_cube, truncated_octahedron};
use std::collections::BTreeMap;

type V3 = [f64; 3];

fn write(mesh: &dyn MeshRead, o: &FluentOptions) -> (String, FluentReport) {
    let mut out = Vec::new();
    let r = write_fluent(mesh, o, &mut out).unwrap();
    assert_eq!(out.len() as u64, r.bytes);
    (String::from_utf8(out).unwrap(), r)
}

/// The parts of a written file, read back with a parser that shares no code with the writer.
#[derive(Default)]
struct Parsed {
    decl: [usize; 3],
    nodes: Vec<V3>,
    /// (zone id, first, last, element type) of every cell section; `types` is per cell.
    cell_secs: Vec<(u32, usize, usize, u32, bool)>,
    types: Vec<u32>,
    /// (zone id, first, last, bc, face type, node ids (0-based), c0, c1) per face, 1-based cells.
    faces: Vec<(u32, Vec<usize>, usize, usize)>,
    face_secs: Vec<(u32, usize, usize, u32, u32)>,
    zones: Vec<(u32, String, String)>,
}

fn h(s: &str) -> usize {
    usize::from_str_radix(s, 16).unwrap()
}

fn parse(text: &str) -> Parsed {
    let mut p = Parsed::default();
    let mut lines = text.lines().peekable();
    while let Some(l) = lines.next() {
        let head = |l: &str| -> Vec<String> {
            let inner = &l[l.find("(").unwrap() + 1..];
            let inner = &inner[inner.find('(').unwrap() + 1..];
            inner[..inner.find(')').unwrap()].split_whitespace().map(str::to_owned).collect()
        };
        if l.starts_with("(10 (") {
            let f = head(l);
            if h(&f[0]) == 0 {
                p.decl[0] = h(&f[2]);
                continue;
            }
            assert_eq!(f.len(), 5);
            assert_eq!((f[3].as_str(), f[4].as_str()), ("1", "3"));
            for _ in h(&f[1])..=h(&f[2]) {
                let v: Vec<f64> = lines.next().unwrap().split_whitespace().map(|x| x.parse().unwrap()).collect();
                assert_eq!(v.len(), 3);
                p.nodes.push([v[0], v[1], v[2]]);
            }
            assert_eq!(lines.next(), Some("))"));
        } else if l.starts_with("(12 (") {
            let f = head(l);
            if h(&f[0]) == 0 {
                p.decl[1] = h(&f[2]);
                continue;
            }
            let (first, last, et) = (h(&f[1]), h(&f[2]), h(&f[4]) as u32);
            assert_eq!(first, p.types.len() + 1, "cell zones tile 1..N");
            let mixed = et == 0;
            if mixed {
                assert!(l.ends_with('('), "{l}");
                let mut n = 0;
                while n < last - first + 1 {
                    let row = lines.next().unwrap();
                    for t in row.split_whitespace() {
                        p.types.push(h(t) as u32);
                        n += 1;
                    }
                }
                assert_eq!(lines.next(), Some("))"));
            } else {
                assert!(l.ends_with("))") && !l.ends_with("()"), "uniform zones have no body: {l}");
                p.types.extend(std::iter::repeat_n(et, last - first + 1));
            }
            p.cell_secs.push((h(&f[0]) as u32, first, last, et, mixed));
        } else if l.starts_with("(13 (") {
            let f = head(l);
            if h(&f[0]) == 0 {
                p.decl[2] = h(&f[2]);
                continue;
            }
            let (z, first, last, bc, ft) = (h(&f[0]) as u32, h(&f[1]), h(&f[2]), h(&f[3]) as u32, h(&f[4]) as u32);
            assert_eq!(first, p.faces.len() + 1, "face zones tile 1..N");
            for _ in first..=last {
                let t: Vec<usize> = lines.next().unwrap().split_whitespace().map(h).collect();
                let n = if ft == 0 { t[0] } else { ft as usize };
                let body = if ft == 0 { &t[1..] } else { &t[..] };
                assert_eq!(body.len(), n + 2, "one face per line, count prefix only in mixed zones");
                p.faces.push((z, body[..n].iter().map(|v| v - 1).collect(), body[n], body[n + 1]));
            }
            assert_eq!(lines.next(), Some("))"));
            p.face_secs.push((z, first, last, bc, ft));
        } else if l.starts_with("(39 (") {
            let f = head(l);
            assert_eq!(f.len(), 3, "3-field zone header");
            assert!(l.ends_with("())"));
            p.zones.push((f[0].parse().unwrap(), f[1].clone(), f[2].clone()));
        }
    }
    p
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Signed cell volumes and vector-area closure from the faces and the c0 / c1 rule alone: the
/// face loop's right-hand normal points INTO c0 and out of c1. A fan of the loop about its first
/// node gives both the area vector and the divergence-theorem volume.
fn volumes(p: &Parsed) -> (Vec<f64>, Vec<V3>) {
    let n = p.decl[1];
    let (mut vol, mut close) = (vec![0.0; n], vec![[0.0; 3]; n]);
    for (_, l, c0, c1) in &p.faces {
        let p0 = p.nodes[l[0]];
        let (mut det, mut area) = (0.0, [0.0; 3]);
        for w in l[1..].windows(2) {
            let (a, b) = (p.nodes[w[0]], p.nodes[w[1]]);
            det += dot(p0, cross(a, b)) / 6.0;
            let c = cross(sub(a, p0), sub(b, p0));
            for k in 0..3 {
                area[k] += c[k] / 2.0;
            }
        }
        // Outward from c0 is the reverse loop.
        vol[c0 - 1] -= det;
        for k in 0..3 {
            close[c0 - 1][k] -= area[k];
        }
        if *c1 != 0 {
            vol[c1 - 1] += det;
            for k in 0..3 {
                close[c1 - 1][k] += area[k];
            }
        }
    }
    (vol, close)
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-300)
}

/// Everything a conforming file satisfies for fixture `f` with the report `r`.
fn check_fixture(f: &MeshFixture, text: &str, r: &FluentReport) -> Parsed {
    let p = parse(text);
    assert_eq!(p.decl, [r.nodes as usize, r.cells as usize, r.faces as usize], "{}", f.name);
    assert_eq!(p.nodes.len(), p.decl[0]);
    assert_eq!(p.types.len(), p.decl[1]);
    assert_eq!(p.faces.len(), p.decl[2]);
    assert_eq!(p.decl[1], f.mesh.cell_count());
    let (vol, close) = volumes(&p);
    for n in 0..p.decl[1] {
        let want = f.cell_volumes[r.cell_of_fluent[n] as usize];
        assert!(vol[n] > 0.0, "{}: cell {n} has volume {} (orientation)", f.name, vol[n]);
        assert!(rel(vol[n], want) < 1e-12, "{}: cell {n} volume {} vs analytic {want}", f.name, vol[n]);
        let scale = want.cbrt().powi(2);
        assert!(close[n].iter().all(|x| x.abs() < 1e-12 * scale), "{}: cell {n} does not close: {:?}", f.name, close[n]);
    }
    // Boundary faces: one cell, c1 = 0, and the boundary area is the fixture's.
    let mut area = 0.0;
    for (_, l, c0, c1) in &p.faces {
        if *c1 == 0 {
            let p0 = p.nodes[l[0]];
            let mut a = [0.0; 3];
            for w in l[1..].windows(2) {
                let c = cross(sub(p.nodes[w[0]], p0), sub(p.nodes[w[1]], p0));
                for k in 0..3 {
                    a[k] += c[k] / 2.0;
                }
            }
            area += dot(a, a).sqrt();
            assert!(*c0 >= 1);
        }
    }
    assert!(rel(area, f.area) < 1e-12, "{}: boundary area {area} vs {}", f.name, f.area);
    // The same cell and face sets as the source, and unique zone ids over all sections.
    let mut ids: Vec<u32> = vec![1];
    ids.extend(p.cell_secs.iter().map(|s| s.0));
    ids.extend(p.face_secs.iter().map(|s| s.0));
    let mut sorted = ids.clone();
    sorted.sort_unstable();
    sorted.dedup();
    assert_eq!(sorted.len(), ids.len(), "{}: zone ids are unique", f.name);
    assert_eq!(p.zones.len(), ids.len() - 1, "{}: every zone has a section 39", f.name);
    p
}

fn all_fixtures() -> Vec<MeshFixture> {
    vec![
        single_hex().unwrap(),
        single_tet().unwrap(),
        single_wedge().unwrap(),
        single_pyramid().unwrap(),
        hexagonal_prism(1.0, 2.0).unwrap(),
        split_face_cube().unwrap(),
        hex_block_2x2x2().unwrap(),
        mixed_transition().unwrap(),
        dual_patch().unwrap(),
        truncated_octahedron().unwrap(),
    ]
}

#[test]
fn every_fixture_has_the_analytic_volumes_orientation_and_area() {
    for f in all_fixtures() {
        let (text, r) = write(&f.mesh, &FluentOptions::default());
        check_fixture(&f, &text, &r);
    }
}

#[test]
fn element_types_come_from_the_canonical_face_set_only() {
    let t = |f: MeshFixture| write(&f.mesh, &FluentOptions::default()).1.cells_by_type;
    assert_eq!(t(single_hex().unwrap()), BTreeMap::from([(4, 1)]));
    assert_eq!(t(single_tet().unwrap()), BTreeMap::from([(2, 1)]));
    assert_eq!(t(single_wedge().unwrap()), BTreeMap::from([(6, 1)]));
    assert_eq!(t(single_pyramid().unwrap()), BTreeMap::from([(5, 1)]));
    // A hexagonal prism and a cube with a split face are polyhedra, never hexes.
    assert_eq!(t(hexagonal_prism(1.0, 1.0).unwrap()), BTreeMap::from([(7, 1)]));
    assert_eq!(t(split_face_cube().unwrap()), BTreeMap::from([(7, 1)]));
    assert_eq!(t(dual_patch().unwrap()), BTreeMap::from([(7, 7)]));
    assert_eq!(t(mixed_transition().unwrap()), BTreeMap::from([(2, 1), (4, 1), (5, 1), (6, 1)]));
    assert_eq!(t(hex_block_2x2x2().unwrap()), BTreeMap::from([(4, 8)]));
}

#[test]
fn uniform_zones_have_no_body_and_mixed_zones_list_one_type_per_cell() {
    let f = mixed_transition().unwrap();
    let (text, _) = write(&f.mesh, &FluentOptions::default());
    // hex 4, wedge 6, pyramid 5, tet 2, in cell order (one zone, so no renumbering).
    assert!(text.contains("(12 (2 1 4 1 0)(\n4 6 5 2\n))\n"), "{text}");
    let (text, _) = write(&hex_block_2x2x2().unwrap().mesh, &FluentOptions::default());
    assert!(text.contains("(12 (2 1 8 1 4))\n"));
    assert!(text.contains("(13 (3 1 c 2 4)(\n"), "interior zone: 12 hex faces as quads, bc 2");
}

#[test]
fn faces_follow_the_right_hand_rule_toward_c0() {
    // Hand-checked: one hex, unit cube, boundary faces reversed with c1 = 0.
    let f = single_hex().unwrap();
    let (text, _) = write(&f.mesh, &FluentOptions::default());
    let p = parse(&text);
    assert!(p.faces.iter().all(|(_, _, c0, c1)| *c0 == 1 && *c1 == 0));
    let centre = [0.5; 3];
    for (_, l, _, _) in &p.faces {
        // Newell normal of the written loop points into the cell.
        let p0 = p.nodes[l[0]];
        let mut n = [0.0; 3];
        for w in l[1..].windows(2) {
            let c = cross(sub(p.nodes[w[0]], p0), sub(p.nodes[w[1]], p0));
            for k in 0..3 {
                n[k] += c[k];
            }
        }
        let mid = [0, 1, 2].map(|k| l.iter().map(|&v| p.nodes[v][k]).sum::<f64>() / l.len() as f64);
        assert!(dot(n, sub(centre, mid)) > 0.0);
    }
}

#[test]
fn a_file_with_every_face_reversed_is_caught() {
    let f = hex_block_2x2x2().unwrap();
    let (text, r) = write(&f.mesh, &FluentOptions::default());
    let mut p = check_fixture(&f, &text, &r);
    // Swap c0 and c1 on interior faces and reverse boundary loops: every orientation flips.
    for (_, l, c0, c1) in p.faces.iter_mut() {
        if *c1 != 0 {
            std::mem::swap(c0, c1);
        } else {
            l.reverse();
        }
    }
    let (vol, _) = volumes(&p);
    assert!(vol.iter().all(|v| *v < 0.0), "a reversed file has negative volumes: {vol:?}");
}

#[test]
fn cell_zones_boundary_zones_and_ids() {
    let m = common::foam::sets_block();
    let (text, r) = write(&m, &FluentOptions::default());
    let names = |z: &[FluentZone]| z.iter().map(|z| (z.id, z.name.clone(), z.kind, z.first, z.count)).collect::<Vec<_>>();
    assert_eq!(names(&r.cell_zones), vec![(2, "b1".into(), "fluid", 1, 4), (3, "b2".into(), "fluid", 5, 4)]);
    let fz = names(&r.face_zones);
    assert_eq!(fz[0], (4, "interior".into(), "interior", 1, 12));
    assert_eq!(fz.len(), 7);
    assert_eq!(fz[1], (5, "xmin".into(), "wall", 13, 4));
    assert_eq!(fz[6], (10, "zmax".into(), "wall", 33, 4));
    let p = parse(&text);
    assert_eq!(p.face_secs[0], (4, 1, 12, 2, 4));
    assert_eq!(p.zones.iter().map(|z| z.1.as_str()).collect::<Vec<_>>(), ["fluid", "fluid", "interior", "wall", "wall", "wall", "wall", "wall", "wall"]);
    // Cell zones are contiguous in the new numbering and keep the source order inside.
    assert_eq!(r.cell_of_fluent, (0..8).collect::<Vec<u32>>());

    // Boundary types are by set name; an unknown name is an error.
    let mut o = FluentOptions::default();
    o.boundary_types.insert("xmin".into(), FluentBc::PressureInlet);
    o.boundary_types.insert("xmax".into(), FluentBc::PressureOutlet);
    let (text, r) = write(&m, &o);
    let p = parse(&text);
    assert_eq!(p.face_secs[1].3, 4);
    assert_eq!(p.face_secs[2].3, 5);
    assert!(text.contains("(39 (5 pressure-inlet xmin)())\n") && text.contains("(39 (6 pressure-outlet xmax)())\n"));
    assert_eq!(r.face_zones[1].kind, "pressure-inlet");
    o.boundary_types.insert("nowhere".into(), FluentBc::Wall);
    let e = write_fluent(&m, &o, &mut Vec::new()).unwrap_err();
    assert!(matches!(e, FluentError::UnknownBoundary(ref n) if n == "nowhere"), "{e}");
    assert!(r.loss.get("fluent.boundary_type_default").is_some());
}

#[test]
fn cell_zones_renumber_cells_and_overlaps_and_loose_cells_are_reported() {
    // Zone "odd" takes the odd cells first, "low" then claims 0..4 but loses cells 1 and 3.
    let mut alloc = SerialAlloc::new();
    let mut m = hex_block([2, 2, 2], [0.5; 3], &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, "odd", SetMembers::Cells([1, 3, 5].map(CellIx::new).to_vec()), &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, "Low zone", SetMembers::Cells((0..4).map(CellIx::new).collect()), &mut alloc).unwrap();
    let (text, r) = write(&m, &FluentOptions::default());
    // odd: 1 3 5; low_zone: 0 2; fluid: 4 6 7.
    assert_eq!(r.cell_of_fluent, vec![1, 3, 5, 0, 2, 4, 6, 7]);
    let zs: Vec<_> = r.cell_zones.iter().map(|z| (z.name.as_str(), z.first, z.count)).collect();
    assert_eq!(zs, [("odd", 1, 3), ("low_zone", 4, 2), ("fluid", 6, 3)]);
    assert_eq!(r.loss.get("fluent.cell_zone_overlap").unwrap().count, 2);
    assert_eq!(r.loss.get("fluent.default_cell_zone").unwrap().count, 3);
    assert!(r.loss.items.iter().any(|i| i.code == "fluent.name_changed" && i.detail == "'Low_zone' -> 'low_zone'"));
    // Volumes follow the renumbering: every cell is 1/8, whatever the zone.
    let p = parse(&text);
    let (vol, close) = volumes(&p);
    assert!(vol.iter().all(|v| (v - 0.125).abs() < 1e-15));
    assert!(close.iter().all(|c| c.iter().all(|x| x.abs() < 1e-15)));
    // `cell_zones: none` gives the single zone `fluid`.
    let mut o = FluentOptions::default();
    o.cell_zones = sm_meshio::foam::ZoneSelect::None;
    let (_, r) = write(&m, &o);
    assert_eq!(r.cell_zones.len(), 1);
    assert_eq!(r.cell_of_fluent, (0..8).collect::<Vec<u32>>());
}

#[test]
fn names_are_lowercase_scheme_symbols_and_unique() {
    let mut alloc = SerialAlloc::new();
    let mut m = hex_block([2, 1, 1], [1.0; 3], &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, "Fuel/Pin 1", SetMembers::Cells(vec![CellIx::new(0)]), &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, "fuel_pin_1", SetMembers::Cells(vec![CellIx::new(1)]), &mut alloc).unwrap();
    let (text, r) = write(&m, &FluentOptions::default());
    let names: Vec<&str> = r.cell_zones.iter().map(|z| z.name.as_str()).collect();
    assert_eq!(names, ["fuel_pin_1", "fuel_pin_1_2"]);
    assert!(text.contains("(39 (2 fluid fuel_pin_1)())"));
    for z in r.cell_zones.iter().chain(&r.face_zones) {
        assert!(z.name.starts_with(|c: char| c.is_ascii_lowercase()) && z.name.chars().all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || "_.-".contains(c)), "{}", z.name);
    }
}

#[test]
fn reals_are_shortest_round_trip_floats() {
    let mut alloc = SerialAlloc::new();
    let m = hex_block([1, 1, 1], [1e-7, 0.1, 1e20], &mut alloc).unwrap();
    let (text, _) = write(&m, &FluentOptions::default());
    let p = parse(&text);
    assert!(p.nodes.iter().any(|n| n[0] == 1e-7 && n[1] == 0.1 && n[2] == 1e20));
    assert!(text.contains("1.0e-7") && !text.contains(" 1e-7") && !text.contains("\n1e-7"), "{text}");
}

#[test]
fn a_mesh_with_inactive_cells_or_no_cells_is_a_typed_error() {
    let mut alloc = SerialAlloc::new();
    let m = hex_block([1, 1, 1], [1.0; 3], &mut alloc).unwrap();
    let mut o = FluentOptions::default();
    o.cell_zones = sm_meshio::foam::ZoneSelect::Names(vec!["nope".into()]);
    let e = write_fluent(&m, &o, &mut Vec::new()).unwrap_err();
    assert!(matches!(e, FluentError::Mesh(_)), "{e}");
}

fn in_pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
}

#[test]
fn bytes_are_identical_across_runs_and_thread_counts() {
    // 20^3 hexes: more than one formatting chunk of nodes and of faces.
    let mut alloc = SerialAlloc::new();
    let m = hex_block([20, 20, 20], [0.05; 3], &mut alloc).unwrap();
    let go = || {
        let (t, r) = write(&m, &FluentOptions::default());
        (t, r.hash)
    };
    let a = in_pool(1, go);
    let b = in_pool(1, go);
    let c = in_pool(4, go);
    let d = in_pool(7, go);
    assert!(a.0.len() > 400_000);
    assert_eq!(a, b);
    assert_eq!(a, c);
    assert_eq!(a, d);
    let p = parse(&a.0);
    let (vol, _) = volumes(&p);
    assert!(vol.iter().all(|v| (v - 0.05f64.powi(3)).abs() < 1e-15));
    // Mixed fixtures too.
    let f = mixed_transition().unwrap();
    assert_eq!(in_pool(1, || write(&f.mesh, &FluentOptions::default()).0), in_pool(5, || write(&f.mesh, &FluentOptions::default()).0));
}

#[test]
fn options_have_a_stable_json_shape() {
    let o: FluentOptions = serde_json::from_str(r#"{"cell_zones":{"names":["b1"]},"boundary_types":{"inlet":"pressure-inlet","out":"outflow"}}"#).unwrap();
    assert_eq!(o.boundary_types["inlet"], FluentBc::PressureInlet);
    assert!(serde_json::from_str::<FluentOptions>(r#"{"binary":true}"#).is_err());
    for (b, code, word) in [(FluentBc::Wall, 3, "wall"), (FluentBc::Symmetry, 7, "symmetry"), (FluentBc::VelocityInlet, 10, "velocity-inlet"), (FluentBc::MassFlowInlet, 20, "mass-flow-inlet"), (FluentBc::Outflow, 36, "outflow")] {
        assert_eq!((b.code(), b.word()), (code, word));
        assert_eq!(serde_json::to_string(&b).unwrap(), format!("\"{word}\""));
    }
}

// ---- OpenFOAM as the oracle ----------------------------------------------------------------------

/// Removes its directory on drop. It lives under the cargo target dir, not `$TMPDIR`: Docker
/// Desktop on macOS does not share `/var/folders`.
struct Scratch(std::path::PathBuf);
impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn docker_ok(image: &str) -> bool {
    std::process::Command::new("docker").args(["image", "inspect", image]).output().is_ok_and(|o| o.status.success())
}

/// Signed cell volumes of an OpenFOAM `polyMesh` (owner positive, neighbour negative).
fn foam_volumes(m: &sm_meshio::foam::PolyMesh) -> Vec<f64> {
    let mut vol = vec![0.0; m.n_cells as usize];
    for f in 0..m.n_faces() {
        let l = m.face(f);
        let q = |i: usize| m.points[l[i] as usize];
        let mut det = 0.0;
        for i in 1..l.len() - 1 {
            det += dot(q(0), cross(q(i), q(i + 1))) / 6.0;
        }
        vol[m.owner[f] as usize] += det;
        if f < m.n_internal() {
            vol[m.neighbour[f] as usize] -= det;
        }
    }
    vol
}

/// `SM_CONFORMANCE=1`: `fluent3DMeshToFoam` + `checkMesh -allGeometry -allTopology` in the
/// OpenFOAM image on a hex brick, a mixed mesh and a polyhedral dual; the polyMesh read back has
/// our cell and face counts and our cell volumes to 1e-12 relative; a file with every face
/// reversed is caught by the same checkMesh.
#[test]
fn openfoam_reads_the_file_and_checkmesh_agrees() {
    if std::env::var("SM_CONFORMANCE").as_deref() != Ok("1") {
        eprintln!("skipped (set SM_CONFORMANCE=1)");
        return;
    }
    let image = std::env::var("SM_CONFORMANCE_IMAGE").unwrap_or_else(|_| "opencfd/openfoam-default:2506".into());
    if !docker_ok(&image) {
        assert_eq!(std::env::var("SM_CONFORMANCE_ALLOW_SKIP").as_deref(), Ok("1"), "docker image {image} is missing");
        eprintln!("skipped: no docker image {image}");
        return;
    }
    let scratch = Scratch(std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("sm-fluent-conf-{}", std::process::id())));
    let mut alloc = SerialAlloc::new();
    let brick = MeshFixture { name: "brick", mesh: hex_block([4, 3, 2], [0.25, 0.5, 0.125], &mut alloc).unwrap(), cell_volumes: vec![0.25 * 0.5 * 0.125; 24], volume: 0.0, area: 0.0, boundary_faces: 0 };
    let cases: Vec<MeshFixture> = vec![brick, mixed_transition().unwrap(), dual_patch().unwrap(), hexagonal_prism(1.0, 2.0).unwrap(), truncated_octahedron().unwrap()];
    let run = |case: &std::path::Path, name: &str, text: &str| -> String {
        let dir = case.join(name);
        std::fs::create_dir_all(dir.join("system")).unwrap();
        for (f, s) in [
            ("controlDict", include_str!("../../../tools/conformance/foam_system/controlDict")),
            ("fvSchemes", include_str!("../../../tools/conformance/foam_system/fvSchemes")),
            ("fvSolution", include_str!("../../../tools/conformance/foam_system/fvSolution")),
        ] {
            std::fs::write(dir.join("system").join(f), s).unwrap();
        }
        std::fs::write(dir.join("mesh.msh"), text).unwrap();
        let dir = dir.canonicalize().unwrap();
        let o = std::process::Command::new("docker")
            .args(["run", "--rm", "--user"])
            .arg(format!("{}:{}", unsafe_free_uid(), unsafe_free_gid()))
            .arg("-v")
            .arg(format!("{}:/case", dir.display()))
            .args(["-w", "/case", &image, "bash", "-lc"])
            .arg(". $(ls -d /usr/lib/openfoam/openfoam*/etc/bashrc | head -1) && fluent3DMeshToFoam -case /case /case/mesh.msh && checkMesh -case /case -allGeometry -allTopology")
            .output()
            .unwrap();
        format!("{}{}", String::from_utf8_lossy(&o.stdout), String::from_utf8_lossy(&o.stderr))
    };
    for f in &cases {
        let (text, r) = write(&f.mesh, &FluentOptions::default());
        let log = run(&scratch.0, f.name, &text);
        // checkMesh's small-determinant and concave-cell findings are quality heuristics of
        // tiny or prismatic meshes, not file defects (docs/research/fluent-msh.md, empirical
        // check); every other finding fails.
        let starred: Vec<&str> = log.lines().filter(|l| l.trim_start().starts_with("***")).collect();
        let heuristics = ["Cells with small determinant", "Concave cells"];
        assert!(starred.iter().all(|l| heuristics.iter().any(|h| l.contains(h))), "{}: checkMesh findings {starred:?}:\n{log}", f.name);
        assert!(log.contains("Boundary openness") && log.contains(" OK.\n    Max cell openness") && log.contains("Cell volumes OK."), "{}:\n{log}", f.name);
        assert!(!log.contains("incorrectly oriented") && !log.contains("negative"), "{}:\n{log}", f.name);
        // The zone files carry OpenFOAM's own `meta` entries, which our strict parser refuses;
        // the zone names are checked on the text, then the files are set aside.
        let pm = scratch.0.join(f.name).join("constant/polyMesh");
        let cz = std::fs::read_to_string(pm.join("cellZones")).unwrap();
        for z in &r.cell_zones {
            assert!(cz.contains(&z.name), "{}: cell zone {} is missing from OpenFOAM's cellZones:\n{cz}", f.name, z.name);
        }
        for z in ["cellZones", "faceZones"] {
            let _ = std::fs::remove_file(pm.join(z));
        }
        let foam = sm_meshio::foam::read_polymesh_dir(&pm).unwrap();
        assert_eq!(foam.n_cells as u64, r.cells, "{}", f.name);
        assert_eq!(foam.n_faces() as u64, r.faces, "{}", f.name);
        assert_eq!(foam.n_internal() as u64, r.interior_faces, "{}", f.name);
        let (mut a, mut b) = (foam_volumes(&foam), f.cell_volumes.clone());
        a.sort_by(f64::total_cmp);
        b.sort_by(f64::total_cmp);
        for (x, y) in a.iter().zip(&b) {
            assert!(rel(*x, *y) < 1e-12, "{}: volume {x} vs {y}", f.name);
        }
    }
    // Negative test: every face reversed (interior c0 <-> c1, boundary loops reversed).
    let f = &cases[0];
    let (text, _) = write(&f.mesh, &FluentOptions::default());
    let flipped: String = text
        .lines()
        .map(|l| {
            let t: Vec<&str> = l.split_whitespace().collect();
            if t.len() >= 5 && t.iter().all(|x| x.chars().all(|c| c.is_ascii_hexdigit())) && l.starts_with(|c: char| c.is_ascii_hexdigit()) {
                let (nodes, cells) = t.split_at(t.len() - 2);
                if cells[1] == "0" {
                    format!("{} {} {}", nodes.iter().rev().cloned().collect::<Vec<_>>().join(" "), cells[0], cells[1])
                } else {
                    format!("{} {} {}", nodes.join(" "), cells[1], cells[0])
                }
            } else {
                l.to_owned()
            }
        })
        .collect::<Vec<_>>()
        .join("\n");
    let log = run(&scratch.0, "flipped", &(flipped + "\n"));
    assert!(log.contains("incorrectly oriented") || log.contains("negative"), "a reversed file must fail checkMesh:\n{log}");
}

fn unsafe_free_uid() -> String {
    std::process::Command::new("id").arg("-u").output().map(|o| String::from_utf8_lossy(&o.stdout).trim().to_owned()).unwrap_or_else(|_| "1000".into())
}

fn unsafe_free_gid() -> String {
    std::process::Command::new("id").arg("-g").output().map(|o| String::from_utf8_lossy(&o.stdout).trim().to_owned()).unwrap_or_else(|_| "1000".into())
}
