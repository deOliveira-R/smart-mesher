//! Abaqus `.inp` export: analytic volumes from the written connectivity, the face numbering of
//! `*SURFACE` entries by outward normal, the meshio reader as oracle (skipped without meshio),
//! sets, names, polyhedra and pyramid policies, loss codes, determinism and property tests.

mod common;

use common::py::{python, run_json, tmp};
use proptest::prelude::*;
use sm_mesh_api::reference::ReferenceMesh;
use sm_mesh_api::*;
use sm_meshio::abaqus::*;
use sm_testkit::meshes::{CellSpec, MeshFixture, SetSpec, build_mesh, dual_patch, hex_block_two_volumes, hexagonal_prism, mixed_transition, single_hex, single_pyramid, single_tet, single_wedge, split_face_cube, tet_block_two_volumes, truncated_octahedron, with_sets};
use sm_testkit::strategies;
use std::collections::BTreeMap;

type V3 = [f64; 3];

fn write(mesh: &dyn MeshRead, o: &AbaqusOptions) -> (Vec<u8>, AbaqusReport) {
    let mut out = Vec::new();
    let r = write_abaqus(mesh, o, &mut out).unwrap();
    (out, r)
}

fn pyr(p: PyramidPolicy) -> AbaqusOptions {
    AbaqusOptions::default().with_pyramids(p)
}

/// A parsed deck, by the rules of the Abaqus input format (keyword lines, data lines).
#[derive(Default, Debug)]
struct Inp {
    nodes: Vec<V3>,
    /// (type, id, nodes 1-based)
    elems: Vec<(String, u32, Vec<u32>)>,
    elsets: BTreeMap<String, Vec<u32>>,
    nsets: BTreeMap<String, Vec<u32>>,
    surfaces: BTreeMap<String, Vec<(u32, u8)>>,
}

fn parse(bytes: &[u8]) -> Inp {
    let text = std::str::from_utf8(bytes).unwrap();
    let mut inp = Inp::default();
    let mut cur = ("".to_string(), "".to_string());
    let mut pending: Vec<u32> = Vec::new();
    for line in text.lines() {
        if line.starts_with("**") {
            continue;
        }
        if let Some(k) = line.strip_prefix('*') {
            let mut parts = k.split(',').map(str::trim);
            let kw = parts.next().unwrap().to_ascii_uppercase();
            let params: BTreeMap<String, String> = parts
                .map(|p| match p.split_once('=') {
                    Some((a, b)) => (a.trim().to_ascii_uppercase(), b.trim().to_string()),
                    None => (p.to_ascii_uppercase(), String::new()),
                })
                .collect();
            let name = params.get("TYPE").or(params.get("ELSET")).or(params.get("NSET")).or(params.get("NAME")).cloned().unwrap_or_default();
            if kw == "ELSET" {
                inp.elsets.entry(name.clone()).or_default();
            }
            if kw == "NSET" {
                inp.nsets.entry(name.clone()).or_default();
            }
            if kw == "SURFACE" {
                assert_eq!(params.get("TYPE").map(String::as_str), Some("ELEMENT"));
                inp.surfaces.entry(params["NAME"].clone()).or_default();
                cur = (kw, params["NAME"].clone());
            } else {
                cur = (kw, name);
            }
            pending.clear();
            continue;
        }
        let vals: Vec<&str> = line.split(',').map(str::trim).filter(|s| !s.is_empty()).collect();
        match cur.0.as_str() {
            "NODE" => {
                let id: usize = vals[0].parse().unwrap();
                assert_eq!(id, inp.nodes.len() + 1, "node ids are 1..n in order");
                inp.nodes.push([vals[1].parse().unwrap(), vals[2].parse().unwrap(), vals[3].parse().unwrap()]);
            }
            "ELEMENT" => {
                assert!(vals.len() <= 16, "at most 16 values per data line");
                let v: Vec<u32> = vals.iter().map(|s| s.parse().unwrap()).collect();
                inp.elems.push((cur.1.clone(), v[0], v[1..].to_vec()));
            }
            "ELSET" => {
                assert!(vals.len() <= 16);
                inp.elsets.get_mut(&cur.1).unwrap().extend(vals.iter().map(|s| s.parse::<u32>().unwrap()));
            }
            "NSET" => {
                assert!(vals.len() <= 16);
                inp.nsets.get_mut(&cur.1).unwrap().extend(vals.iter().map(|s| s.parse::<u32>().unwrap()));
            }
            "SURFACE" => {
                assert_eq!(vals.len(), 2);
                let s: u8 = vals[1].strip_prefix('S').unwrap().parse().unwrap();
                inp.surfaces.get_mut(&cur.1).unwrap().push((vals[0].parse().unwrap(), s));
            }
            "HEADING" => {}
            k => panic!("unexpected data under *{k}"),
        }
    }
    let _ = pending;
    inp
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
fn det(a: V3, b: V3, c: V3) -> f64 {
    dot(a, cross(b, c))
}

/// Signed volume of one element from its written nodes, with the smallest Jacobian determinant
/// (`C3D4`: det/6; `C3D8`: 2x2x2 Gauss over the trilinear map; `C3D6`: affine Jacobian at the
/// centroid and corners; `C3D5`: two tets over the base diagonal).
fn element_volume(ty: &str, p: &[V3]) -> (f64, f64) {
    match ty {
        "C3D4" => {
            let d = det(sub(p[1], p[0]), sub(p[2], p[0]), sub(p[3], p[0]));
            (d / 6.0, d)
        }
        "C3D8" => {
            const R: [[f64; 3]; 8] = [[-1., -1., -1.], [1., -1., -1.], [1., 1., -1.], [-1., 1., -1.], [-1., -1., 1.], [1., -1., 1.], [1., 1., 1.], [-1., 1., 1.]];
            let jac = |x: [f64; 3]| -> f64 {
                let mut j = [[0.0; 3]; 3];
                for (i, r) in R.iter().enumerate() {
                    let f = [1.0 + x[0] * r[0], 1.0 + x[1] * r[1], 1.0 + x[2] * r[2]];
                    let d = [r[0] * f[1] * f[2] / 8.0, f[0] * r[1] * f[2] / 8.0, f[0] * f[1] * r[2] / 8.0];
                    for a in 0..3 {
                        for b in 0..3 {
                            j[b][a] += d[b] * p[i][a];
                        }
                    }
                }
                det(j[0], j[1], j[2])
            };
            let g = 1.0 / 3.0_f64.sqrt();
            let (mut v, mut m) = (0.0, f64::INFINITY);
            for sx in [-g, g] {
                for sy in [-g, g] {
                    for sz in [-g, g] {
                        let d = jac([sx, sy, sz]);
                        v += d;
                        m = m.min(d);
                    }
                }
            }
            (v, m)
        }
        "C3D6" => {
            let jac = |xi: f64, eta: f64, ze: f64| -> f64 {
                let l = [1.0 - xi - eta, xi, eta];
                let dl = [[-1.0, -1.0], [1.0, 0.0], [0.0, 1.0]];
                let mut j = [[0.0; 3]; 3];
                for k in 0..3 {
                    for a in 0..3 {
                        let (b, t) = (p[k][a], p[k + 3][a]);
                        let mid = (1.0 - ze) / 2.0 * b + (1.0 + ze) / 2.0 * t;
                        j[0][a] += dl[k][0] * mid;
                        j[1][a] += dl[k][1] * mid;
                        j[2][a] += l[k] * (t - b) / 2.0;
                    }
                }
                det(j[0], j[1], j[2])
            };
            let mut m = f64::INFINITY;
            for (xi, eta) in [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)] {
                for ze in [-1.0, 1.0] {
                    m = m.min(jac(xi, eta, ze));
                }
            }
            (jac(1.0 / 3.0, 1.0 / 3.0, 0.0), m)
        }
        "C3D5" => {
            let d1 = det(sub(p[1], p[0]), sub(p[2], p[0]), sub(p[4], p[0]));
            let d2 = det(sub(p[2], p[0]), sub(p[3], p[0]), sub(p[4], p[0]));
            ((d1 + d2) / 6.0, d1.min(d2))
        }
        t => panic!("type {t}"),
    }
}

fn coords_of(inp: &Inp, nodes: &[u32]) -> Vec<V3> {
    nodes.iter().map(|&n| inp.nodes[n as usize - 1]).collect()
}

/// Total volume and the smallest determinant over the elements of a parsed deck. A collapsed
/// hexahedron (pyramid) has a zero determinant at the apex corners by construction, so the
/// minimum is only meaningful for the other types.
fn volume(inp: &Inp) -> (f64, f64) {
    let (mut total, mut min) = (0.0, f64::INFINITY);
    for (ty, _, nodes) in &inp.elems {
        let (v, m) = element_volume(ty, &coords_of(inp, nodes));
        total += v;
        min = min.min(m);
    }
    (total, min)
}

fn counts(inp: &Inp) -> BTreeMap<String, u64> {
    let mut c = BTreeMap::new();
    for (t, _, _) in &inp.elems {
        *c.entry(t.clone()).or_insert(0) += 1;
    }
    c
}

/// One analytic fixture per element type: type name, fixture, analytic volume.
fn unit_fixtures() -> Vec<(&'static str, MeshFixture, f64, AbaqusOptions)> {
    vec![
        ("C3D8", single_hex().unwrap(), 1.0, AbaqusOptions::default()),
        ("C3D6", single_wedge().unwrap(), 0.5, AbaqusOptions::default()),
        ("C3D4", single_tet().unwrap(), 1.0 / 6.0, AbaqusOptions::default()),
        ("C3D5", single_pyramid().unwrap(), 1.0 / 3.0, pyr(PyramidPolicy::C3d5)),
    ]
}

#[test]
fn each_type_has_positive_jacobians_and_analytic_volume() {
    for (ty, f, vol, o) in unit_fixtures() {
        let (bytes, r) = write(&f.mesh, &o);
        let inp = parse(&bytes);
        assert_eq!(inp.elems.len(), 1, "{ty}");
        assert_eq!(inp.elems[0].0, ty);
        assert_eq!(r.elements_by_type[ty], 1);
        let (v, m) = volume(&inp);
        assert!(m > 0.0, "{ty}: det {m}");
        assert!((v - vol).abs() <= 1e-14, "{ty}: {v} vs {vol}");
        assert_eq!(inp.nodes.len() as u64, r.nodes);
        assert_eq!(inp.nodes.len(), f.mesh.vertex_count());
    }
    // The collapsed-hexahedron pyramid has the same volume (positive; singular only at the apex).
    let f = single_pyramid().unwrap();
    let (bytes, r) = write(&f.mesh, &pyr(PyramidPolicy::DegenerateHex));
    let inp = parse(&bytes);
    assert_eq!(inp.elems[0].0, "C3D8");
    assert_eq!(inp.elems[0].2.len(), 8);
    assert_eq!(&inp.elems[0].2[4..], &[5, 5, 5, 5]);
    let (v, _) = volume(&inp);
    assert!((v - 1.0 / 3.0).abs() <= 1e-14, "{v}");
    assert_eq!(r.loss.get("abaqus.pyramid_degenerate_hex").unwrap().count, 1);
}

fn normalize(a: V3) -> V3 {
    let l = dot(a, a).sqrt();
    [a[0] / l, a[1] / l, a[2] / l]
}

/// Outward normal of every `Sn` of the unit fixtures, from the element definitions.
fn expected_normal(ty: &str, s: u8) -> V3 {
    let h = 0.5_f64.sqrt();
    let t = 3.0_f64.sqrt().recip();
    match (ty, s) {
        ("C3D8", 1) | ("C3D6", 1) | ("C3D4", 1) | ("C3D5", 1) => [0.0, 0.0, -1.0],
        ("C3D8", 2) | ("C3D6", 2) => [0.0, 0.0, 1.0],
        ("C3D8", 3) | ("C3D6", 3) | ("C3D4", 2) => [0.0, -1.0, 0.0],
        ("C3D8", 4) => [1.0, 0.0, 0.0],
        ("C3D8", 5) => [0.0, 1.0, 0.0],
        ("C3D8", 6) | ("C3D6", 5) | ("C3D4", 4) => [-1.0, 0.0, 0.0],
        ("C3D6", 4) => [h, h, 0.0],
        ("C3D4", 3) => [t, t, t],
        _ => panic!("no such face {ty} S{s}"),
    }
}

/// Pyramid side normals (base unit square, apex (0.5, 0.5, 1)), outward: y=0 side, x=1, y=1, x=0.
fn pyramid_side(k: usize) -> V3 {
    let n = [[0.0, -1.0, 0.5], [1.0, 0.0, 0.5], [0.0, 1.0, 0.5], [-1.0, 0.0, 0.5]][k];
    normalize(n)
}

/// Newell normal of the face loop (outward for the left cell).
fn face_normal(mesh: &dyn MeshRead, f: usize) -> V3 {
    let co = mesh.coords();
    let vs: Vec<V3> = mesh.face_vertices(FaceIx::new(f)).iter().map(|v| [co.x[v.get()], co.y[v.get()], co.z[v.get()]]).collect();
    let mut n = [0.0; 3];
    for i in 0..vs.len() {
        let (a, b) = (vs[i], vs[(i + 1) % vs.len()]);
        n[0] += (a[1] - b[1]) * (a[2] + b[2]);
        n[1] += (a[2] - b[2]) * (a[0] + b[0]);
        n[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    normalize(n)
}

#[test]
fn surface_face_numbers_have_the_documented_outward_normals() {
    let cases: Vec<(&str, MeshFixture, AbaqusOptions, bool)> = vec![
        ("C3D8", single_hex().unwrap(), AbaqusOptions::default(), false),
        ("C3D6", single_wedge().unwrap(), AbaqusOptions::default(), false),
        ("C3D4", single_tet().unwrap(), AbaqusOptions::default(), false),
        ("C3D5", single_pyramid().unwrap(), pyr(PyramidPolicy::C3d5), false),
        ("C3D8", single_pyramid().unwrap(), pyr(PyramidPolicy::DegenerateHex), true),
    ];
    for (ty, f, o, collapsed) in cases {
        let nf = f.mesh.face_count();
        let specs: Vec<SetSpec> = (0..nf).map(|i| SetSpec::new(&format!("f{i}"), SetMembers::Faces(vec![FaceIx::new(i)]))).collect();
        let f = with_sets(f, &specs).unwrap();
        let (bytes, r) = write(&f.mesh, &o);
        assert_eq!(r.surfaces as usize, nf, "{ty}");
        let inp = parse(&bytes);
        let mut seen = std::collections::BTreeSet::new();
        for i in 0..nf {
            let s = &inp.surfaces[&format!("f{i}")];
            assert_eq!(s.len(), 1);
            let (id, sn) = s[0];
            assert_eq!(id, 1);
            seen.insert(sn);
            let want = if ty == "C3D5" && sn >= 2 {
                pyramid_side(sn as usize - 2)
            } else if collapsed {
                match sn {
                    1 => [0.0, 0.0, -1.0],
                    3..=6 => pyramid_side(sn as usize - 3),
                    _ => panic!("collapsed pyramid face S{sn}"),
                }
            } else {
                expected_normal(ty, sn)
            };
            let got = face_normal(&f.mesh, i);
            for k in 0..3 {
                assert!((got[k] - want[k]).abs() <= 1e-14, "{ty} S{sn}: {got:?} vs {want:?}");
            }
        }
        // Every face got a distinct number.
        assert_eq!(seen.len(), nf, "{ty}: {seen:?}");
    }
}

fn two_volume_block() -> MeshFixture {
    hex_block_two_volumes([3, 2, 2], [0.5, 1.0, 0.25], 1).unwrap()
}

#[test]
fn sets_become_elsets_nsets_and_surfaces() {
    let f = two_volume_block();
    let nc = f.mesh.cell_count();
    assert_eq!(nc, 12);
    let (bytes, r) = write(&f.mesh, &AbaqusOptions::default());
    let inp = parse(&bytes);
    assert_eq!(counts(&inp), BTreeMap::from([("C3D8".to_string(), 12)]));
    // The fixture's cell sets (one per volume) become ELSETs with their cells + 1 as ids and an
    // NSET each (the cells' nodes).
    let sets: Vec<_> = f.mesh.sets().iter().filter(|s| matches!(s.members, SetMembers::Cells(_))).collect();
    assert_eq!(sets.len(), 2);
    for s in sets {
        let SetMembers::Cells(c) = &s.members else { unreachable!() };
        let want: Vec<u32> = c.iter().map(|c| c.0 + 1).collect();
        assert_eq!(inp.elsets[&s.name], want, "{}", s.name);
        let mut nodes: Vec<u32> = c.iter().flat_map(|c| f.mesh.cell_corners(*c).unwrap().iter().map(|v| v.0 + 1)).collect();
        nodes.sort_unstable();
        nodes.dedup();
        assert_eq!(inp.nsets[&s.name], nodes);
    }
    assert_eq!(r.elsets, 2);
    assert_eq!(r.nsets, 2);
    // Without nsets there are none.
    let (b2, r2) = write(&f.mesh, &AbaqusOptions::default().with_nsets(false));
    assert!(parse(&b2).nsets.is_empty() && r2.nsets == 0);
}

#[test]
fn boundary_surface_covers_the_boundary_area_and_vertex_sets_are_nsets() {
    let f = two_volume_block();
    // All one-cell faces as one face set, and two corner vertices.
    let bf: Vec<FaceIx> = (0..f.mesh.face_count()).map(FaceIx::new).filter(|&x| f.mesh.face_right(x).is_none()).collect();
    assert_eq!(bf.len(), f.boundary_faces);
    let f = with_sets(f, &[SetSpec::new("skin", SetMembers::Faces(bf.clone())), SetSpec::new("corners", SetMembers::Vertices(vec![VertexIx::new(0), VertexIx::new(5)]))]).unwrap();
    let (bytes, _) = write(&f.mesh, &AbaqusOptions::default());
    let inp = parse(&bytes);
    let surf = &inp.surfaces["skin"];
    assert_eq!(surf.len(), bf.len());
    // Area from the written element faces (hex face table) equals the analytic skin area.
    let faces: [&[usize]; 6] = [&[0, 1, 2, 3], &[4, 5, 6, 7], &[0, 1, 5, 4], &[1, 2, 6, 5], &[2, 3, 7, 6], &[3, 0, 4, 7]];
    let by_id: BTreeMap<u32, &Vec<u32>> = inp.elems.iter().map(|(_, id, n)| (*id, n)).collect();
    let mut area = 0.0;
    for (id, s) in surf {
        let p = coords_of(&inp, by_id[id]);
        let q: Vec<V3> = faces[*s as usize - 1].iter().map(|&i| p[i]).collect();
        let n = cross(sub(q[2], q[0]), sub(q[3], q[1]));
        area += 0.5 * dot(n, n).sqrt();
    }
    assert!((area - f.area).abs() <= 1e-12 * f.area, "{area} vs {}", f.area);
    assert_eq!(inp.nsets["corners"], vec![1, 6]);
    // The face set's NSET holds exactly the skin's nodes.
    let mut skin_nodes: Vec<u32> = bf.iter().flat_map(|x| f.mesh.face_vertices(*x).iter().map(|v| v.0 + 1)).collect();
    skin_nodes.sort_unstable();
    skin_nodes.dedup();
    assert_eq!(inp.nsets["skin"], skin_nodes);
}

#[test]
fn side_sets_pick_the_named_cell_and_interior_face_sets_use_the_left_cell() {
    let f = two_volume_block();
    // The interior face between volume 1 and 2.
    let x = (0..f.mesh.face_count()).map(FaceIx::new).find(|&x| f.mesh.face_right(x).is_some() && f.mesh.face_class(x).entity.kind() == sm_core::EntityKind::TopoFace).unwrap();
    let (l, rt) = (f.mesh.face_left(x), f.mesh.face_right(x).unwrap());
    let f = with_sets(
        f,
        &[SetSpec::new("left", SetMembers::Sides(vec![SignedFace::new(x, Side::Left)])), SetSpec::new("right", SetMembers::Sides(vec![SignedFace::new(x, Side::Right)])), SetSpec::new("both", SetMembers::Faces(vec![x]))],
    )
    .unwrap();
    let (bytes, r) = write(&f.mesh, &AbaqusOptions::default());
    let inp = parse(&bytes);
    assert_eq!(inp.surfaces["left"][0].0, l.0 + 1);
    assert_eq!(inp.surfaces["right"][0].0, rt.0 + 1);
    assert_eq!(inp.surfaces["both"][0].0, l.0 + 1);
    // The two sides are opposite faces of the two elements: the shared nodes agree.
    let by_id: BTreeMap<u32, &Vec<u32>> = inp.elems.iter().map(|(_, id, n)| (*id, n)).collect();
    let face = |(id, s): (u32, u8)| -> Vec<u32> {
        let faces: [&[usize]; 6] = [&[0, 1, 2, 3], &[4, 5, 6, 7], &[0, 1, 5, 4], &[1, 2, 6, 5], &[2, 3, 7, 6], &[3, 0, 4, 7]];
        let mut v: Vec<u32> = faces[s as usize - 1].iter().map(|&i| by_id[&id][i]).collect();
        v.sort_unstable();
        v
    };
    assert_eq!(face(inp.surfaces["left"][0]), face(inp.surfaces["right"][0]));
    assert_eq!(r.loss.get("abaqus.surface_interior_face").unwrap().count, 1);
}

fn sanity_loss_free(r: &AbaqusReport) {
    assert!(r.loss.items.iter().all(|i| i.severity == sm_meshio::LossSeverity::Info), "{:?}", r.loss);
}

#[test]
fn names_are_sanitized_made_unique_ignoring_case_and_reported() {
    let f = single_hex().unwrap();
    let long = "x".repeat(100);
    let f = with_sets(
        f,
        &[
            SetSpec::new("fuel pin/1", SetMembers::Cells(vec![CellIx::new(0)])),
            SetSpec::new("1st", SetMembers::Vertices(vec![VertexIx::new(0)])),
            SetSpec::new("Fuel", SetMembers::Vertices(vec![VertexIx::new(1)])),
            SetSpec::new("FUEL", SetMembers::Vertices(vec![VertexIx::new(2)])),
            SetSpec::new(&long, SetMembers::Vertices(vec![VertexIx::new(3)])),
            SetSpec::new("empty", SetMembers::Vertices(vec![])),
        ],
    )
    .unwrap();
    let (bytes, r) = write(&f.mesh, &AbaqusOptions::default());
    let inp = parse(&bytes);
    let mut names: Vec<String> = inp.elsets.keys().chain(inp.nsets.keys()).cloned().collect();
    names.sort();
    names.dedup();
    assert!(names.contains(&"fuel_pin_1".to_string()), "{names:?}");
    assert!(inp.nsets.contains_key("s_1st"));
    assert!(inp.nsets.contains_key("Fuel") && inp.nsets.contains_key("FUEL_2"), "{names:?}");
    assert!(inp.nsets.keys().all(|n| n.len() <= 80));
    // All names unique ignoring case over elsets+nsets (nsets of a cell set repeat its name).
    let up: Vec<String> = inp.nsets.keys().map(|n| n.to_ascii_uppercase()).collect();
    let mut u = up.clone();
    u.sort();
    u.dedup();
    assert_eq!(u.len(), up.len());
    assert_eq!(r.loss.get("abaqus.set_dropped").unwrap().count, 1, "the empty set");
    assert!(r.loss.items.iter().any(|i| i.code == "abaqus.name_changed" && i.detail.contains("fuel pin/1")));
}

#[test]
fn mixed_mesh_counts_per_type_and_volume() {
    let f = mixed_transition().unwrap();
    for p in [PyramidPolicy::C3d5, PyramidPolicy::DegenerateHex] {
        let (bytes, r) = write(&f.mesh, &pyr(p));
        let inp = parse(&bytes);
        assert_eq!(inp.elems.len(), 4);
        assert_eq!(counts(&inp), r.elements_by_type);
        let (v, _) = volume(&inp);
        assert!((v - f.volume).abs() <= 1e-14 * f.volume, "{p:?}: {v} vs {}", f.volume);
        // Ids are cell index + 1.
        assert_eq!(inp.elems.iter().map(|e| e.1).collect::<std::collections::BTreeSet<_>>().into_iter().collect::<Vec<_>>(), vec![1, 2, 3, 4]);
    }
    // The default refuses the pyramid, typed, and writes nothing.
    let mut out = Vec::new();
    assert!(matches!(write_abaqus(&f.mesh, &AbaqusOptions::default(), &mut out), Err(AbaqusError::PyramidsUnsupported { count: 1 })));
    assert!(out.is_empty());
    let (_, r) = write(&f.mesh, &pyr(PyramidPolicy::C3d5));
    assert_eq!(r.loss.get("abaqus.pyramid_type_unverified").unwrap().count, 1);
}

fn hybrid() -> ReferenceMesh {
    // The split-face cube (polyhedron) next to a hexahedron on its +x face.
    let mut p: Vec<[f64; 3]> = vec![[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];
    p.extend([[2., 0., 0.], [2., 1., 0.], [2., 0., 1.], [2., 1., 1.]]);
    let poly = vec![vec![0, 2, 1], vec![0, 3, 2], vec![4, 5, 6, 7], vec![0, 1, 5, 4], vec![1, 2, 6, 5], vec![2, 3, 7, 6], vec![3, 0, 4, 7]];
    build_mesh(&p, &[CellSpec::Poly(poly), CellSpec::Template(CellShape::Hex, vec![1, 8, 9, 2, 5, 10, 11, 6])], &mut SerialAlloc::new()).unwrap()
}

#[test]
fn polyhedra_policies() {
    let dp = dual_patch().unwrap();
    let mut out = Vec::new();
    match write_abaqus(&dp.mesh, &AbaqusOptions::default(), &mut out) {
        Err(AbaqusError::PolyhedraUnsupported { count }) => assert_eq!(count, 7),
        e => panic!("{e:?}"),
    }
    assert!(out.is_empty());
    // Decompose conserves the volume (1e-12) and every tet is positive.
    let o = AbaqusOptions::default().with_polyhedra(PolyPolicy::Decompose);
    for f in [dual_patch().unwrap(), hexagonal_prism(1.0, 2.0).unwrap(), split_face_cube().unwrap(), truncated_octahedron().unwrap()] {
        let (bytes, r) = write(&f.mesh, &o);
        let inp = parse(&bytes);
        let (v, min) = volume(&inp);
        assert!(min > 0.0, "{}: {min}", f.name);
        assert!((v - f.volume).abs() <= 1e-12 * f.volume, "{}: {v} vs {}", f.name, f.volume);
        assert_eq!(r.loss.get("abaqus.poly_decomposed").unwrap().count as usize, f.mesh.cell_count());
        assert_eq!(r.nodes as usize, f.mesh.vertex_count() + r.loss.get("abaqus.nodes_added").unwrap().count as usize);
        assert_eq!(r.elements_by_type.keys().collect::<Vec<_>>(), vec!["C3D4"]);
        // Decomposition tets are numbered after the last cell id.
        assert!(inp.elems.iter().all(|e| e.1 as usize > f.mesh.cell_count()));
    }
    // Next to a hexahedron across a quad it is refused, naming the face.
    let h = hybrid();
    assert!(matches!(write_abaqus(&h, &o, &mut Vec::new()), Err(AbaqusError::NonconformingDecomposition { .. })));
    // Drop reports the exact count and keeps the hexahedron.
    let o = AbaqusOptions::default().with_polyhedra(PolyPolicy::Drop);
    let (bytes, r) = write(&h, &o);
    let inp = parse(&bytes);
    assert_eq!(r.loss.get("abaqus.poly_dropped").unwrap().count, 1);
    assert_eq!(counts(&inp), BTreeMap::from([("C3D8".to_string(), 1)]));
    assert_eq!(inp.elems[0].1, 2, "ids keep the cell index");
    assert!((volume(&inp).0 - 1.0).abs() <= 1e-14);
    // Dropping everything is a typed error, not an empty deck.
    assert!(matches!(write_abaqus(&dual_patch().unwrap().mesh, &o, &mut Vec::new()), Err(AbaqusError::Unsupported(_))));
}

#[test]
fn decomposed_boundary_surface_area_equals_the_analytic_boundary() {
    let f = dual_patch().unwrap();
    let bf: Vec<FaceIx> = (0..f.mesh.face_count()).map(FaceIx::new).filter(|&x| f.mesh.face_right(x).is_none()).collect();
    assert_eq!(bf.len(), f.boundary_faces);
    let f = with_sets(f, &[SetSpec::new("skin", SetMembers::Faces(bf)), SetSpec::new("body", SetMembers::Cells((0..7).map(CellIx::new).collect()))]).unwrap();
    let (bytes, r) = write(&f.mesh, &AbaqusOptions::default().with_polyhedra(PolyPolicy::Decompose));
    let inp = parse(&bytes);
    let by_id: BTreeMap<u32, &Vec<u32>> = inp.elems.iter().map(|(_, id, n)| (*id, n)).collect();
    let mut area = 0.0;
    for (id, s) in &inp.surfaces["skin"] {
        assert_eq!(*s, 1, "decomposition tets expose their boundary facet as S1");
        let p = coords_of(&inp, by_id[id]);
        area += 0.5 * dot(cross(sub(p[1], p[0]), sub(p[2], p[0])), cross(sub(p[1], p[0]), sub(p[2], p[0]))).sqrt();
    }
    assert!((area - f.area).abs() <= 1e-12 * f.area, "{area} vs {}", f.area);
    // The ELSET of the polyhedra holds every decomposition tet.
    assert_eq!(inp.elsets["body"].len(), inp.elems.len());
    assert_eq!(r.elsets, 1);
}

#[test]
fn reals_are_shortest_round_trip_floats() {
    let p = [[0.0, 0.0, 0.0], [1e-7, 0.0, -0.0], [0.0, 0.1 + 0.2, 0.0], [0.0, 0.0, 1.5e300]];
    let m = build_mesh(&p, &[CellSpec::Template(CellShape::Tet, vec![0, 1, 2, 3])], &mut SerialAlloc::new()).unwrap();
    let (bytes, _) = write(&m, &AbaqusOptions::default());
    let text = String::from_utf8(bytes.clone()).unwrap();
    assert!(text.contains("1.0e-7") && text.contains("0.30000000000000004") && text.contains("1.5e300"), "{text}");
    let inp = parse(&bytes);
    for (a, b) in inp.nodes.iter().zip(p.iter()) {
        for k in 0..3 {
            assert_eq!(a[k].to_bits(), b[k].to_bits());
        }
    }
}

fn in_pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
}

#[test]
fn bytes_are_identical_across_runs_and_thread_counts() {
    // Large enough to span several parallel chunks.
    let big = hex_block_two_volumes([30, 20, 15], [0.5; 3], 7).unwrap();
    let m = mixed_transition().unwrap();
    let dp = dual_patch().unwrap();
    let o = AbaqusOptions::default();
    sm_testkit::determinism::check(2, 4, |t| in_pool(t, || write(&big.mesh, &o).1.hash)).unwrap();
    let o2 = pyr(PyramidPolicy::C3d5);
    sm_testkit::determinism::check(2, 4, |t| in_pool(t, || write(&m.mesh, &o2).1.hash)).unwrap();
    let o3 = AbaqusOptions::default().with_polyhedra(PolyPolicy::Decompose);
    sm_testkit::determinism::check(2, 4, |t| in_pool(t, || write(&dp.mesh, &o3).1.hash)).unwrap();
    assert_eq!(write(&big.mesh, &o).0, in_pool(1, || write(&big.mesh, &o).0));
}

#[test]
fn unsupported_inputs_are_typed_errors() {
    let f = two_volume_block();
    let w = common::wrap::Wrap::new(&f.mesh).dim2();
    assert!(matches!(write_abaqus(&w, &AbaqusOptions::default(), &mut Vec::new()), Err(AbaqusError::Unsupported(_))));
    let w = common::wrap::Wrap::new(&f.mesh).inactive(&[0]);
    assert!(matches!(write_abaqus(&w, &AbaqusOptions::default(), &mut Vec::new()), Err(AbaqusError::Unsupported(_))));
    // Options refuse unknown keys.
    assert!(serde_json::from_str::<AbaqusOptions>(r#"{"binary":true}"#).is_err());
    let o: AbaqusOptions = serde_json::from_str(r#"{"polyhedra":"decompose","pyramids":"degenerate_hex","nsets":false}"#).unwrap();
    assert_eq!((o.polyhedra, o.pyramids, o.nsets), (PolyPolicy::Decompose, PyramidPolicy::DegenerateHex, false));
    // Shapes that are plain are lossless apart from Info items.
    sanity_loss_free(&write(&single_hex().unwrap().mesh, &AbaqusOptions::default()).1);
}

/// A Python that can `import meshio`, else None (the check is skipped, loudly).
fn meshio_python() -> Option<String> {
    let py = python()?;
    let ok = std::process::Command::new(&py).args(["-I", "-c", "import meshio"]).output().ok().is_some_and(|o| o.status.success());
    ok.then_some(py)
}

#[test]
fn meshio_reads_the_decks_with_equal_counts_volumes_and_sets() {
    let Some(py) = meshio_python() else {
        eprintln!("meshio not importable (set SM_PYTHON): skipping the meshio oracle");
        return;
    };
    let script = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/abaqus_meshio.py");
    let block = two_volume_block();
    let tets = tet_block_two_volumes([3, 2, 2], [0.5, 1.0, 0.25], 2).unwrap();
    let mixed = mixed_transition().unwrap();
    let dp = dual_patch().unwrap();
    let wedge = single_wedge().unwrap();
    let cases: Vec<(&str, &MeshFixture, AbaqusOptions)> = vec![
        ("hex_block", &block, AbaqusOptions::default()),
        ("tet_block", &tets, AbaqusOptions::default()),
        ("wedge", &wedge, AbaqusOptions::default()),
        ("mixed", &mixed, pyr(PyramidPolicy::DegenerateHex)),
        ("dual", &dp, AbaqusOptions::default().with_polyhedra(PolyPolicy::Decompose)),
    ];
    let meshio_name = |t: &str| match t {
        "C3D4" => "tetra",
        "C3D6" => "wedge",
        "C3D8" => "hexahedron",
        t => panic!("{t}"),
    };
    for (name, f, o) in cases {
        let (bytes, r) = write(&f.mesh, &o);
        let path = tmp(&format!("abaqus_{name}.inp"));
        std::fs::write(&path, &bytes).unwrap();
        let j = run_json(&py, script, &path);
        let _ = std::fs::remove_file(&path);
        assert_eq!(j["points"].as_u64().unwrap(), r.nodes, "{name}");
        // Equal cell counts per type.
        let got: BTreeMap<String, u64> = j["counts"].as_object().unwrap().iter().map(|(k, v)| (k.clone(), v.as_u64().unwrap())).collect();
        let want: BTreeMap<String, u64> = r.elements_by_type.iter().map(|(k, v)| (meshio_name(k).to_string(), *v)).collect();
        assert_eq!(got, want, "{name}");
        // Volume read through meshio equals the analytic one.
        let vol: f64 = j["volume"].as_object().unwrap().values().map(|v| v.as_f64().unwrap()).sum();
        assert!((vol - f.volume).abs() <= 1e-12 * f.volume, "{name}: {vol} vs {}", f.volume);
        // Sets: element sets keep their sizes, node sets exist.
        for s in f.mesh.sets() {
            if let SetMembers::Cells(c) = &s.members {
                if r.elements_by_type.contains_key("C3D4") && name == "dual" {
                    continue;
                }
                assert_eq!(j["cell_sets"][&s.name].as_u64().unwrap() as usize, c.len(), "{name}/{}", s.name);
                assert!(j["point_sets"].get(&s.name).is_some(), "{name}/{}", s.name);
            }
        }
    }
}

proptest! {
    #![proptest_config(strategies::config(16))]

    #[test]
    fn random_blocks_have_the_analytic_volume_and_counts(n0 in 2u32..5, n1 in 1u32..4, n2 in 1u32..4, split in 1u32..4, h in 0.1f64..2.0) {
        let split = 1 + split % (n0 - 1);
        let hs = [h, h * 0.75, h * 1.5];
        for (fixture, per_cell, ty) in [(tet_block_two_volumes([n0, n1, n2], hs, split).unwrap(), 6usize, "C3D4"), (hex_block_two_volumes([n0, n1, n2], hs, split).unwrap(), 1, "C3D8")] {
            let (bytes, r) = write(&fixture.mesh, &AbaqusOptions::default());
            let inp = parse(&bytes);
            prop_assert_eq!(r.elements_by_type[ty] as usize, (n0 * n1 * n2) as usize * per_cell);
            prop_assert_eq!(inp.elems.len(), fixture.mesh.cell_count());
            let (v, m) = volume(&inp);
            prop_assert!(m > 0.0);
            prop_assert!((v - fixture.volume).abs() <= 1e-12 * fixture.volume);
            // Every cell set lists each of its elements once.
            let want: usize = fixture.mesh.sets().iter().map(|s| if let SetMembers::Cells(c) = &s.members { c.len() } else { 0 }).sum();
            let total: usize = inp.elsets.values().map(Vec::len).sum();
            prop_assert_eq!(total, want);
        }
    }
}
