//! CGNS export: the node tree against the format spec (names, labels, type tags, dtypes, shapes),
//! an independent model rebuilt from the tree, orientation and closure by signed cell volumes
//! against the fixtures' analytic volumes, the signed `NFACE_n` rule, BC face loops, blocks and
//! families, determinism, and (gated by `SM_CONFORMANCE=1`) the CGNS library itself as the
//! oracle: `cgnscheck`, and a C reader that links `libcgns` and dumps what the library reads.

mod common;

use sm_mesh_api::reference::hex_block;
use sm_mesh_api::*;
use sm_meshio::cgns::*;
use sm_meshio::foam::{FoamOptions, ZoneSelect, build_polymesh};
use sm_meshio::h5::read::read_file;
use sm_meshio::h5::{Dtype, Group, Obj};
use sm_testkit::meshes::{MeshFixture, dual_patch, hex_block_2x2x2, hexagonal_prism, mixed_transition, single_hex, single_pyramid, single_tet, single_wedge, split_face_cube, truncated_octahedron};
use std::collections::{BTreeMap, BTreeSet};

type V3 = [f64; 3];

fn layout_opts(layout: CgnsLayout) -> CgnsOptions {
    let mut o = CgnsOptions::default();
    o.layout = layout;
    o
}

fn write(mesh: &dyn MeshRead, o: &CgnsOptions) -> (Vec<u8>, CgnsReport) {
    let mut out = Vec::new();
    let r = write_cgns(mesh, o, &mut out).unwrap();
    assert_eq!(out.len() as u64, r.bytes);
    (out, r)
}

// ---- an independent model of the file ---------------------------------------------------------

struct Sec {
    name: String,
    etype: i64,
    first: usize,
    last: usize,
    conn: Vec<i64>,
    offs: Vec<i64>,
}

#[derive(Default)]
struct Model {
    points: Vec<V3>,
    nverts: usize,
    ncells: usize,
    secs: Vec<Sec>,
    bcs: Vec<(String, Vec<i64>)>,
    blocks: Vec<(String, Vec<i64>)>,
    families: BTreeMap<String, Option<String>>,
}

fn ints(d: &sm_meshio::h5::Dataset) -> Vec<i64> {
    match d.dtype {
        Dtype::I32 => d.as_i32().into_iter().map(i64::from).collect(),
        Dtype::I64 => d.as_i64(),
        ref t => panic!("not an integer dataset: {t:?}"),
    }
}

fn string(g: &Group, path: &str) -> String {
    let d = g.dataset(&format!("{path}/ data")).unwrap_or_else(|| panic!("{path} has no data"));
    assert_eq!(d.dtype, Dtype::U8);
    String::from_utf8(d.data.clone()).unwrap()
}

fn attr_str(g: &Group, key: &str) -> String {
    let a = g.attrs.iter().find(|a| a.name == key).unwrap_or_else(|| panic!("missing attribute {key}"));
    let n = a.data.iter().position(|&b| b == 0).unwrap_or(a.data.len());
    String::from_utf8(a.data[..n].to_vec()).unwrap()
}

fn group<'a>(g: &'a Group, path: &str) -> &'a Group {
    match g.find(path) {
        Some(Obj::Group(x)) => x,
        _ => panic!("no group {path}"),
    }
}

fn etype_of(word: &str) -> i64 {
    match word {
        "TRI_3" => 5,
        "QUAD_4" => 7,
        "TETRA_4" => 10,
        "PYRA_5" => 12,
        "PENTA_6" => 14,
        "HEXA_8" => 17,
        "NGON_n" => 22,
        "NFACE_n" => 23,
        w => panic!("{w}"),
    }
}

/// Walk the tree: every node has the four attributes, its `type` tag agrees with the dtype of its
/// ` data`, children are creation-ordered. Returns the nodes as `path -> (label, type)`.
fn check_tree(root: &Group) -> BTreeMap<String, (String, String)> {
    fn walk(g: &Group, path: &str, out: &mut BTreeMap<String, (String, String)>) {
        assert!(g.tracked, "{path}: groups track link creation order");
        for n in &g.order {
            assert!(g.children.contains_key(n), "{path}: order lists {n}");
        }
        assert_eq!(g.order.len(), g.children.len(), "{path}: every child has a creation index");
        for (name, o) in &g.children {
            let Obj::Group(c) = o else { continue };
            let p = if path.is_empty() { name.clone() } else { format!("{path}/{name}") };
            assert_eq!(attr_str(c, "name"), *name, "{p}");
            let (label, ty) = (attr_str(c, "label"), attr_str(c, "type"));
            let flags = c.attrs.iter().find(|a| a.name == "flags").unwrap();
            assert_eq!((flags.dtype.clone(), flags.dims.clone(), flags.data.clone()), (Dtype::I32, vec![1], 1i32.to_le_bytes().to_vec()), "{p}");
            let names: Vec<_> = c.attrs.iter().map(|a| (a.name.as_str(), a.dtype.clone())).collect();
            assert_eq!(names, [("name", Dtype::Str(33)), ("label", Dtype::Str(33)), ("type", Dtype::Str(3)), ("flags", Dtype::I32)], "{p}");
            match (ty.as_str(), c.dataset(" data")) {
                ("MT", None) => {}
                ("I4", Some(d)) => assert_eq!(d.dtype, Dtype::I32, "{p}"),
                ("I8", Some(d)) => assert_eq!(d.dtype, Dtype::I64, "{p}"),
                ("R4", Some(d)) => assert_eq!(d.dtype, Dtype::F32, "{p}"),
                ("R8", Some(d)) => assert_eq!(d.dtype, Dtype::F64, "{p}"),
                ("C1", Some(d)) => assert_eq!(d.dtype, Dtype::U8, "{p}"),
                other => panic!("{p}: type tag {other:?} does not match its data"),
            }
            out.insert(p.clone(), (label, ty));
            walk(c, &p, out);
        }
    }
    let mut out = BTreeMap::new();
    assert_eq!(attr_str(root, "label"), "Root Node of HDF5 File");
    assert_eq!(attr_str(root, "name"), "HDF5 MotherNode");
    assert_eq!(attr_str(root, "type"), "MT");
    walk(root, "", &mut out);
    out
}

fn model(bytes: &[u8]) -> Model {
    let root = read_file(bytes).unwrap();
    let tree = check_tree(&root);
    // Library version, base and zone.
    assert_eq!(tree["CGNSLibraryVersion"], ("CGNSLibraryVersion_t".into(), "R4".into()));
    assert_eq!(root.dataset("CGNSLibraryVersion/ data").unwrap().as_f32(), [4.0]);
    assert_eq!(tree["Base"].0, "CGNSBase_t");
    assert_eq!(ints(root.dataset("Base/ data").unwrap()), [3, 3]);
    assert_eq!(tree["Base/Zone"].0, "Zone_t");
    assert_eq!(string(&root, "Base/Zone/ZoneType"), "Unstructured");
    let zd = root.dataset("Base/Zone/ data").unwrap();
    assert_eq!(zd.dims, [3, 1]);
    let z = ints(zd);
    let mut m = Model { nverts: z[0] as usize, ncells: z[1] as usize, ..Model::default() };
    assert_eq!(z[2], 0);
    let col = |k: &str| {
        assert_eq!(tree[&format!("Base/Zone/GridCoordinates/Coordinate{k}")], ("DataArray_t".into(), "R8".into()));
        root.dataset(&format!("Base/Zone/GridCoordinates/Coordinate{k}/ data")).unwrap().as_f64()
    };
    let (x, y, zc) = (col("X"), col("Y"), col("Z"));
    m.points = (0..x.len()).map(|i| [x[i], y[i], zc[i]]).collect();
    assert_eq!(m.points.len(), m.nverts);
    let zone = group(&root, "Base/Zone");
    for name in &zone.order {
        let path = format!("Base/Zone/{name}");
        if !tree.contains_key(&path) {
            continue; // the zone's own ` data`
        }
        match tree[&path].0.as_str() {
            "Elements_t" => {
                let d = ints(root.dataset(&format!("{path}/ data")).unwrap());
                assert_eq!((d.len(), d[1]), (2, 0), "{path}: [type, 0]");
                let r = ints(root.dataset(&format!("{path}/ElementRange/ data")).unwrap());
                assert_eq!(tree[&format!("{path}/ElementRange")], ("IndexRange_t".into(), tree[&format!("{path}/ElementRange")].1.clone()));
                let conn = ints(root.dataset(&format!("{path}/ElementConnectivity/ data")).unwrap());
                let offs = root.dataset(&format!("{path}/ElementStartOffset/ data")).map(ints).unwrap_or_default();
                assert_eq!(tree[&format!("{path}/ElementConnectivity")].0, "DataArray_t");
                m.secs.push(Sec { name: name.clone(), etype: d[0], first: r[0] as usize, last: r[1] as usize, conn, offs });
            }
            "ZoneBC_t" => {
                for bc in &group(&root, &path).order {
                    let bp = format!("{path}/{bc}");
                    assert_eq!(tree[&bp].0, "BC_t");
                    assert_eq!(string(&root, &bp), "FamilySpecified");
                    assert_eq!(string(&root, &format!("{bp}/GridLocation")), "FaceCenter");
                    assert_eq!(string(&root, &format!("{bp}/FamilyName")), *bc);
                    let pl = root.dataset(&format!("{bp}/PointList/ data")).unwrap();
                    assert_eq!(tree[&format!("{bp}/PointList")].0, "IndexArray_t");
                    assert_eq!(pl.dims, [pl.dims[0], 1]);
                    m.bcs.push((bc.clone(), ints(pl)));
                }
            }
            "ZoneSubRegion_t" => {
                assert_eq!(ints(root.dataset(&format!("{path}/ data")).unwrap()), [3]);
                assert_eq!(string(&root, &format!("{path}/GridLocation")), "CellCenter");
                assert_eq!(string(&root, &format!("{path}/FamilyName")), *name);
                m.blocks.push((name.clone(), ints(root.dataset(&format!("{path}/PointList/ data")).unwrap())));
            }
            "ZoneType_t" | "GridCoordinates_t" => {}
            l => panic!("unexpected zone child {name}: {l}"),
        }
    }
    let base = group(&root, "Base");
    for n in base.order.iter().filter(|n| *n != " data") {
        if tree[&format!("Base/{n}")].0 == "Family_t" {
            let bc = root.dataset(&format!("Base/{n}/FamilyBC/ data")).map(|_| string(&root, &format!("Base/{n}/FamilyBC")));
            m.families.insert(n.clone(), bc);
        }
    }
    m
}

// ---- geometry from the model ----------------------------------------------------------------------

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-300)
}

/// Fan about the first vertex: (divergence-theorem volume term, area vector) of a loop.
fn fan(pts: &[V3], l: &[usize]) -> (f64, V3) {
    let p0 = pts[l[0]];
    let (mut det, mut area) = (0.0, [0.0; 3]);
    for w in l[1..].windows(2) {
        let (a, b) = (pts[w[0]], pts[w[1]]);
        det += dot(p0, cross(a, b)) / 6.0;
        let c = cross(sub(a, p0), sub(b, p0));
        for k in 0..3 {
            area[k] += c[k] / 2.0;
        }
    }
    (det, area)
}

/// CGNS faces of the standard cells, outward from the cell (1-based node numbers of the SIDS).
fn std_faces(etype: i64) -> &'static [&'static [usize]] {
    match etype {
        10 => &[&[1, 3, 2], &[1, 2, 4], &[2, 3, 4], &[3, 1, 4]],
        12 => &[&[1, 4, 3, 2], &[1, 2, 5], &[2, 3, 5], &[3, 4, 5], &[4, 1, 5]],
        14 => &[&[1, 3, 2], &[4, 5, 6], &[1, 2, 5, 4], &[2, 3, 6, 5], &[3, 1, 4, 6]],
        17 => &[&[1, 4, 3, 2], &[5, 6, 7, 8], &[1, 2, 6, 5], &[2, 3, 7, 6], &[3, 4, 8, 7], &[4, 1, 5, 8]],
        t => panic!("not a standard cell type {t}"),
    }
}

/// Face loops (0-based vertices) of an element id of a face section.
fn face_loop(m: &Model, id: usize) -> Vec<usize> {
    for s in &m.secs {
        if (s.first..=s.last).contains(&id) {
            let k = id - s.first;
            return match s.etype {
                22 => s.conn[s.offs[k] as usize..s.offs[k + 1] as usize].iter().map(|v| *v as usize - 1).collect(),
                5 => s.conn[3 * k..3 * k + 3].iter().map(|v| *v as usize - 1).collect(),
                7 => s.conn[4 * k..4 * k + 4].iter().map(|v| *v as usize - 1).collect(),
                t => panic!("element {id} is of type {t}, not a face"),
            };
        }
    }
    panic!("no section holds element {id}")
}

/// Every cell as its outward face loops, in element-id order, with its element id.
fn cells(m: &Model) -> Vec<(usize, Vec<Vec<usize>>)> {
    let mut out = Vec::new();
    for s in &m.secs {
        match s.etype {
            23 => {
                for k in 0..s.last - s.first + 1 {
                    let refs = &s.conn[s.offs[k] as usize..s.offs[k + 1] as usize];
                    let faces = refs
                        .iter()
                        .map(|&r| {
                            let mut l = face_loop(m, r.unsigned_abs() as usize);
                            if r < 0 {
                                l.reverse();
                            }
                            l
                        })
                        .collect();
                    out.push((s.first + k, faces));
                }
            }
            10 | 12 | 14 | 17 => {
                let w = match s.etype {
                    10 => 4,
                    12 => 5,
                    14 => 6,
                    _ => 8,
                };
                for k in 0..s.last - s.first + 1 {
                    let v = &s.conn[w * k..w * k + w];
                    out.push((s.first + k, std_faces(s.etype).iter().map(|f| f.iter().map(|&i| v[i - 1] as usize - 1).collect()).collect()));
                }
            }
            _ => {}
        }
    }
    out.sort_by_key(|c| c.0);
    out
}

/// Cyclic-rotation canonical form of a loop (rotated to its smallest vertex).
fn canon(l: &[usize]) -> Vec<usize> {
    let i = (0..l.len()).min_by_key(|&i| l[i]).unwrap();
    (0..l.len()).map(|k| l[(i + k) % l.len()]).collect()
}

/// Everything a conforming file satisfies for fixture `f` and report `r`; returns the model.
fn check_fixture(f: &MeshFixture, bytes: &[u8], r: &CgnsReport) -> Model {
    let m = model(bytes);
    check_model(f, &m, r);
    m
}

fn check_model(f: &MeshFixture, m: &Model, r: &CgnsReport) {
    assert_eq!((m.nverts as u64, m.ncells as u64), (r.nodes, r.cells), "{}", f.name);
    assert_eq!(m.ncells, f.mesh.cell_count(), "{}", f.name);
    let cs = cells(m);
    assert_eq!(cs.len(), m.ncells, "{}", f.name);
    // Element ids tile 1..N over all sections, in file order.
    let mut next = 1;
    for s in &m.secs {
        assert_eq!(s.first, next, "{}: section {} starts where the previous ended", f.name, s.name);
        next = s.last + 1;
    }
    let mut boundary = BTreeSet::new();
    for (n, (id, faces)) in cs.iter().enumerate() {
        let want = f.cell_volumes[r.cell_of_cgns[n] as usize];
        let (mut vol, mut close) = (0.0, [0.0; 3]);
        for l in faces {
            let (d, a) = fan(&m.points, l);
            vol += d;
            for k in 0..3 {
                close[k] += a[k];
            }
            boundary.insert(canon(l));
        }
        assert!(vol > 0.0, "{}: cell {id} has volume {vol} (orientation)", f.name);
        assert!(rel(vol, want) < 1e-12, "{}: cell {id} volume {vol} vs analytic {want}", f.name);
        let scale = want.cbrt().powi(2);
        assert!(close.iter().all(|x| x.abs() < 1e-12 * scale), "{}: cell {id} does not close: {close:?}", f.name);
    }
    // BC faces: each is exactly an outward loop of some cell, and the BCs tile the boundary.
    let mut area = 0.0;
    let mut seen = BTreeSet::new();
    for (name, ids) in &m.bcs {
        assert!(!ids.is_empty(), "{}: BC {name} is empty", f.name);
        for &id in ids {
            let l = face_loop(m, id as usize);
            assert!(boundary.contains(&canon(&l)), "{}: BC {name} face {id} is no outward cell face", f.name);
            assert!(seen.insert(id), "{}: face {id} is in two BCs", f.name);
            let (_, a) = fan(&m.points, &l);
            area += dot(a, a).sqrt();
        }
    }
    assert_eq!(seen.len(), f.boundary_faces, "{}: BCs cover every boundary face", f.name);
    assert!(rel(area, f.area) < 1e-12, "{}: boundary area {area} vs {}", f.name, f.area);
    // The report agrees with the file.
    assert_eq!(r.sections.len(), m.secs.len());
    for (a, b) in r.sections.iter().zip(&m.secs) {
        assert_eq!((a.name.as_str(), etype_of(a.kind), a.first as usize, a.last as usize), (b.name.as_str(), b.etype, b.first, b.last));
    }
    assert_eq!(r.bcs, m.bcs.iter().map(|b| b.0.clone()).collect::<Vec<_>>());
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

fn kinds(r: &CgnsReport) -> Vec<&'static str> {
    r.sections.iter().map(|s| s.kind).collect()
}

#[test]
fn every_fixture_has_the_analytic_volumes_orientation_and_area_in_both_layouts() {
    for f in all_fixtures() {
        for layout in [CgnsLayout::Auto, CgnsLayout::Ngon] {
            let o = layout_opts(layout);
            let (bytes, r) = write(&f.mesh, &o);
            check_fixture(&f, &bytes, &r);
            if layout == CgnsLayout::Ngon {
                assert_eq!(r.layout, CgnsLayout::Ngon);
                assert_eq!(&kinds(&r)[..2], ["NGON_n", "NFACE_n"], "{}", f.name);
                assert_eq!((r.sections[0].first, r.sections[0].last), (1, r.faces), "{}", f.name);
                assert_eq!((r.sections[1].first, r.sections[1].last), (r.faces + 1, r.faces + r.cells), "{}", f.name);
            }
        }
    }
}

#[test]
fn standard_cells_use_standard_sections_and_polyhedra_use_ngon() {
    let want: [(&str, MeshFixture, &str); 4] = [("single_hex", single_hex().unwrap(), "HEXA_8"), ("tet", single_tet().unwrap(), "TETRA_4"), ("wedge", single_wedge().unwrap(), "PENTA_6"), ("pyramid", single_pyramid().unwrap(), "PYRA_5")];
    for (n, f, word) in want {
        let (_, r) = write(&f.mesh, &CgnsOptions::default());
        assert_eq!(r.layout, CgnsLayout::Auto, "{n}");
        assert_eq!(r.sections[0].kind, word, "{n}");
        assert_eq!((r.sections[0].first, r.sections[0].last), (1, 1));
        assert!(r.sections[1..].iter().all(|s| matches!(s.kind, "TRI_3" | "QUAD_4")), "{n}");
        assert!(r.loss.get("cgns.layout_ngon").is_none());
    }
    // A mixed mesh: one section per type, cells first.
    let f = mixed_transition().unwrap();
    let (_, r) = write(&f.mesh, &CgnsOptions::default());
    let cell_sections: Vec<_> = r.sections.iter().filter(|s| !matches!(s.kind, "TRI_3" | "QUAD_4")).collect();
    let n: u64 = cell_sections.iter().map(|s| s.last - s.first + 1).sum();
    assert_eq!(n, r.cells);
    assert!(cell_sections.len() >= 2, "mixed mesh has several element types");
    // Polyhedra force NGON_n/NFACE_n and say so.
    for f in [dual_patch().unwrap(), truncated_octahedron().unwrap(), hexagonal_prism(1.0, 2.0).unwrap(), split_face_cube().unwrap()] {
        let (_, r) = write(&f.mesh, &CgnsOptions::default());
        assert_eq!(r.layout, CgnsLayout::Ngon, "{}", f.name);
        assert_eq!(kinds(&r), ["NGON_n", "NFACE_n"], "{}", f.name);
        assert!(r.loss.get("cgns.layout_ngon").is_some(), "{}", f.name);
        assert!(r.cells_by_type.contains_key("NFACE_n"));
    }
}

/// Face lists of a polyhedron of five or more faces survive the file: the NGON loops and the
/// NFACE references rebuild the cell's face set of the mesh the writer started from.
#[test]
fn polyhedra_round_trip_with_equal_face_lists() {
    for f in [truncated_octahedron().unwrap(), dual_patch().unwrap(), hexagonal_prism(1.0, 2.0).unwrap()] {
        let (bytes, r) = write(&f.mesh, &CgnsOptions::default());
        let m = model(&bytes);
        let poly = build_polymesh(&f.mesh, &FoamOptions::new().split_small_determinant(false).split_warped_faces(false)).unwrap().mesh;
        let mut big = 0;
        for (n, (_, faces)) in cells(&m).iter().enumerate() {
            let c = r.cell_of_cgns[n] as usize;
            // The polyMesh's own faces of cell `c` (owner loops as stored, neighbour loops reversed).
            let mut want: Vec<Vec<usize>> = Vec::new();
            for fi in 0..poly.n_faces() {
                let l: Vec<usize> = poly.face(fi).iter().map(|&v| v as usize).collect();
                if poly.owner[fi] as usize == c {
                    want.push(canon(&l));
                } else if fi < poly.n_internal() && poly.neighbour[fi] as usize == c {
                    want.push(canon(&l.iter().rev().copied().collect::<Vec<_>>()));
                }
            }
            let mut got: Vec<Vec<usize>> = faces.iter().map(|l| canon(l)).collect();
            got.sort();
            want.sort();
            assert_eq!(got, want, "{}: cell {n}", f.name);
            big += usize::from(faces.len() >= 5);
        }
        assert!(big > 0, "{}: has a polyhedron of five or more faces", f.name);
    }
}

#[test]
fn signed_nface_references_close_every_cell_and_a_reversed_file_is_caught() {
    let f = hex_block_2x2x2().unwrap();
    let o = layout_opts(CgnsLayout::Ngon);
    let (bytes, r) = write(&f.mesh, &o);
    let mut m = check_fixture(&f, &bytes, &r);
    let nface = m.secs.iter().position(|s| s.etype == 23).unwrap();
    // Interior faces are referenced twice with opposite signs, boundary faces once with +.
    let mut sign_sum: BTreeMap<i64, i64> = BTreeMap::new();
    let mut count: BTreeMap<i64, usize> = BTreeMap::new();
    for &x in &m.secs[nface].conn {
        *sign_sum.entry(x.abs()).or_default() += x.signum();
        *count.entry(x.abs()).or_default() += 1;
    }
    for (id, c) in &count {
        assert!(matches!((*c, sign_sum[id]), (2, 0) | (1, 1)), "face {id}: {c} references, sign sum {}", sign_sum[id]);
    }
    // Flip every sign: every cell now has negative volume.
    for x in m.secs[nface].conn.iter_mut() {
        *x = -*x;
    }
    for (_, faces) in cells(&m) {
        let vol: f64 = faces.iter().map(|l| fan(&m.points, l).0).sum();
        assert!(vol < 0.0, "a flipped file has negative volumes");
    }
}

#[test]
fn the_tree_has_the_spec_names_labels_and_shapes() {
    let f = hex_block_2x2x2().unwrap();
    let (bytes, _) = write(&f.mesh, &CgnsOptions::default());
    let root = read_file(&bytes).unwrap();
    let tree = check_tree(&root);
    let got: Vec<(&str, &str, &str)> = tree.iter().map(|(p, (l, t))| (p.as_str(), l.as_str(), t.as_str())).collect();
    // Section and BC names for a mesh without sets: one HEXA_8 section and one default BC.
    assert!(got.contains(&("Base/Zone/HEXA_8", "Elements_t", "I4")), "{got:?}");
    assert!(got.contains(&("Base/Zone/HEXA_8/ElementRange", "IndexRange_t", "I4")));
    assert!(got.contains(&("Base/Zone/HEXA_8/ElementConnectivity", "DataArray_t", "I4")));
    assert!(got.contains(&("Base/Zone/ZoneBC", "ZoneBC_t", "MT")));
    assert!(got.contains(&("Base/Zone/GridCoordinates", "GridCoordinates_t", "MT")));
    let root_names: Vec<&String> = root.order.iter().collect();
    assert_eq!(root_names, [" format", " hdf5version", "CGNSLibraryVersion", "Base"]);
    assert_eq!(root.dataset(" format").unwrap().data, b"IEEE_LITTLE_32\0");
    assert_eq!(root.dataset(" hdf5version").unwrap().data.len(), 33);
    // Element data dims: connectivity 8 per cell, boundary quads 4 per face.
    assert_eq!(root.dataset("Base/Zone/HEXA_8/ElementConnectivity/ data").unwrap().dims, [64]);
    let ngon = write(&f.mesh, &layout_opts(CgnsLayout::Ngon)).0;
    let root = read_file(&ngon).unwrap();
    assert_eq!(root.dataset("Base/Zone/NGON/ElementStartOffset/ data").unwrap().dims, [37]);
    assert_eq!(root.dataset("Base/Zone/NGON/ElementConnectivity/ data").unwrap().dims, [144]);
    assert_eq!(root.dataset("Base/Zone/NFACE/ElementStartOffset/ data").unwrap().dims, [9]);
    assert_eq!(root.dataset("Base/Zone/NFACE/ElementConnectivity/ data").unwrap().dims, [48]);
    assert_eq!(ints(root.dataset("Base/Zone/NFACE/ElementRange/ data").unwrap()), [37, 44]);
    assert_eq!(ints(root.dataset("Base/Zone/NGON/ElementRange/ data").unwrap()), [1, 36]);
}

#[test]
fn blocks_families_bcs_and_overlaps() {
    let m = common::foam::sets_block();
    for layout in [CgnsLayout::Auto, CgnsLayout::Ngon] {
        let mut o = layout_opts(layout);
        o.boundary_types.insert("xmin".into(), CgnsBc::Wall);
        o.boundary_types.insert("xmax".into(), CgnsBc::Outflow);
        let (bytes, r) = write(&m, &o);
        let model = model(&bytes);
        assert_eq!(r.blocks, ["b1", "b2"]);
        assert_eq!(r.bcs, ["xmin", "xmax", "ymin", "ymax", "zmin", "zmax"]);
        assert_eq!(model.families["xmin"].as_deref(), Some("BCWall"));
        assert_eq!(model.families["xmax"].as_deref(), Some("BCOutflow"));
        assert_eq!(model.families["ymin"].as_deref(), Some("UserDefined"));
        assert_eq!(model.families["b1"], None);
        assert!(r.loss.get("cgns.boundary_type_default").is_some());
        // Sub-regions hold the element ids of their cells: b1 is cells 0..4, b2 cells 4..8.
        let cell_ids: Vec<i64> = cells(&model).iter().map(|c| c.0 as i64).collect();
        assert_eq!(model.blocks.len(), 2);
        let mut b: Vec<i64> = model.blocks.iter().flat_map(|b| b.1.clone()).collect();
        b.sort_unstable();
        assert_eq!(b, cell_ids, "the blocks partition the cells");
        for (name, ids) in &model.blocks {
            // Every cell of the block is one of the source cells of that set.
            let want: Vec<u32> = if name == "b1" { (0..4).collect() } else { (4..8).collect() };
            let mut got: Vec<u32> = ids.iter().map(|id| r.cell_of_cgns[cell_ids.iter().position(|c| c == id).unwrap()]).collect();
            got.sort_unstable();
            assert_eq!(got, want, "{layout:?} {name}");
        }
        if layout == CgnsLayout::Auto {
            // One section per (block, type), cells of a block contiguous.
            assert_eq!(r.sections[0].name, "b1_HEXA_8");
            assert_eq!((r.sections[0].first, r.sections[0].last), (1, 4));
            assert_eq!(r.sections[1].name, "b2_HEXA_8");
            assert_eq!(r.cell_of_cgns, (0..8).collect::<Vec<u32>>());
        }
    }
    let mut o = CgnsOptions::default();
    o.boundary_types.insert("nowhere".into(), CgnsBc::Wall);
    let e = write_cgns(&m, &o, &mut Vec::new()).unwrap_err();
    assert!(matches!(e, CgnsError::UnknownBoundary(ref n) if n == "nowhere"), "{e}");
}

#[test]
fn cell_zone_overlaps_loose_cells_and_names() {
    let mut alloc = SerialAlloc::new();
    let mut m = hex_block([2, 2, 2], [0.5; 3], &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, "odd", SetMembers::Cells([1, 3, 5].map(CellIx::new).to_vec()), &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, "Low zone/1", SetMembers::Cells((0..4).map(CellIx::new).collect()), &mut alloc).unwrap();
    let (bytes, r) = write(&m, &CgnsOptions::default());
    // odd: 1 3 5; Low_zone_1: 0 2; loose: 4 6 7 (sections without a block prefix).
    assert_eq!(r.cell_of_cgns, vec![1, 3, 5, 0, 2, 4, 6, 7]);
    assert_eq!(r.blocks, ["odd", "Low_zone_1"]);
    let names: Vec<_> = r.sections.iter().take(3).map(|s| (s.name.as_str(), s.first, s.last)).collect();
    assert_eq!(names, [("odd_HEXA_8", 1, 3), ("Low_zone_1_HEXA_8", 4, 5), ("HEXA_8", 6, 8)]);
    assert_eq!(r.loss.get("cgns.cell_zone_overlap").unwrap().count, 2);
    assert!(r.loss.items.iter().any(|i| i.code == "cgns.name_sanitized" && i.detail.contains("'Low zone/1'")));
    let model = model(&bytes);
    assert_eq!(model.blocks[0], ("odd".to_string(), vec![1, 2, 3]));
    // Long and colliding names are cut to 32 bytes and made unique.
    let mut m = hex_block([2, 1, 1], [0.5; 3], &mut alloc).unwrap();
    let long = "a".repeat(40);
    MeshEdit::add_set(&mut m, &long, SetMembers::Cells(vec![CellIx::new(0)]), &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, &"a".repeat(33), SetMembers::Cells(vec![CellIx::new(1)]), &mut alloc).unwrap();
    let (_, r) = write(&m, &CgnsOptions::default());
    assert!(r.blocks.iter().all(|b| b.len() <= 32), "{:?}", r.blocks);
    assert_eq!(r.blocks.iter().collect::<BTreeSet<_>>().len(), 2);
    // `cell_zones: none` writes no blocks.
    let o = { let mut o = CgnsOptions::default(); o.cell_zones = ZoneSelect::None; o };
    let (_, r) = write(&m, &o);
    assert!(r.blocks.is_empty());
}

#[test]
fn sets_named_like_fixed_zone_children_are_renamed_not_merged() {
    let mut alloc = SerialAlloc::new();
    let reserved = ["NGON", "NFACE", "ZoneType", "ZoneBC", "GridCoordinates", "Base", "CGNSLibraryVersion"];
    for layout in [CgnsLayout::Auto, CgnsLayout::Ngon] {
        let mut m = hex_block([2, 2, 2], [0.5; 3], &mut alloc).unwrap();
        for (i, n) in reserved.iter().enumerate() {
            MeshEdit::add_set(&mut m, n, SetMembers::Cells(vec![CellIx::new(i)]), &mut alloc).unwrap();
        }
        let (bytes, r) = write(&m, &layout_opts(layout));
        let model = model(&bytes); // also checks ZoneType, GridCoordinates and ZoneBC stay intact
        assert_eq!(model.blocks.len(), reserved.len(), "{layout:?}: every sub-region survives");
        let got: Vec<&str> = model.blocks.iter().map(|b| b.0.as_str()).collect();
        for n in reserved {
            assert!(!got.contains(&n), "{layout:?}: {n} must be renamed: {got:?}");
            assert!(got.contains(&format!("{n}_2").as_str()), "{layout:?}: {got:?}");
        }
        assert_eq!(r.loss.items.iter().filter(|i| i.code == "cgns.name_changed").map(|i| i.count).sum::<u64>(), reserved.len() as u64);
        if layout == CgnsLayout::Ngon {
            let kinds: Vec<_> = model.secs.iter().map(|s| (s.name.as_str(), s.etype)).collect();
            assert!(kinds.iter().any(|k| k.0 == "NGON") && kinds.iter().any(|k| k.0 == "NFACE"), "{kinds:?}");
        }
    }
}

fn in_pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
}

#[test]
fn bytes_are_identical_across_runs_and_thread_counts() {
    let mut alloc = SerialAlloc::new();
    let m = hex_block([20, 20, 20], [0.05; 3], &mut alloc).unwrap();
    for layout in [CgnsLayout::Auto, CgnsLayout::Ngon] {
        let go = || {
            let (t, r) = write(&m, &layout_opts(layout));
            (t, r.hash)
        };
        let a = in_pool(1, go);
        assert_eq!(a, in_pool(1, go));
        assert_eq!(a, in_pool(4, go));
        assert_eq!(a, in_pool(7, go));
        assert!(a.0.len() > 400_000);
    }
    let f = mixed_transition().unwrap();
    assert_eq!(in_pool(1, || write(&f.mesh, &CgnsOptions::default()).0), in_pool(5, || write(&f.mesh, &CgnsOptions::default()).0));
    let f = truncated_octahedron().unwrap();
    assert_eq!(in_pool(1, || write(&f.mesh, &CgnsOptions::default()).0), in_pool(5, || write(&f.mesh, &CgnsOptions::default()).0));
}

#[test]
fn options_have_a_stable_json_shape() {
    let o: CgnsOptions = serde_json::from_str(r#"{"layout":"ngon","cell_zones":{"names":["b1"]},"boundary_types":{"inlet":"inflow","w":"wall","s":"symmetry-plane"}}"#).unwrap();
    assert_eq!(o.layout, CgnsLayout::Ngon);
    assert_eq!(o.boundary_types["inlet"], CgnsBc::Inflow);
    assert_eq!(o.boundary_types["s"], CgnsBc::SymmetryPlane);
    assert!(serde_json::from_str::<CgnsOptions>(r#"{"binary":true}"#).is_err());
    assert_eq!(serde_json::to_string(&CgnsOptions::default().layout).unwrap(), "\"auto\"");
    for (b, w) in [(CgnsBc::UserDefined, "UserDefined"), (CgnsBc::Wall, "BCWall"), (CgnsBc::Farfield, "BCFarfield")] {
        assert_eq!(b.word(), w);
    }
}

// ---- the CGNS library as the oracle -------------------------------------------------------------

/// Removes its directory on drop. Under the cargo target dir, not `$TMPDIR`: Docker Desktop on
/// macOS does not share `/var/folders`.
struct Scratch(std::path::PathBuf);
impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn docker_ok(image: &str) -> bool {
    std::process::Command::new("docker").args(["image", "inspect", image]).output().is_ok_and(|o| o.status.success())
}

fn sh(image: &str, dir: &std::path::Path, cmd: &str) -> String {
    let id = |flag: &str| std::process::Command::new("id").arg(flag).output().map(|o| String::from_utf8_lossy(&o.stdout).trim().to_owned()).unwrap_or_else(|_| "1000".into());
    let o = std::process::Command::new("docker")
        .args(["run", "--rm", "--user"])
        .arg(format!("{}:{}", id("-u"), id("-g")))
        .arg("-v")
        .arg(format!("{}:/work", dir.display()))
        .args(["-w", "/work", "--entrypoint", "sh", image, "-c", cmd])
        .output()
        .unwrap();
    format!("{}{}", String::from_utf8_lossy(&o.stdout), String::from_utf8_lossy(&o.stderr))
}

/// Parse the dump of `cgns_dump.c` (what `libcgns` read from the file) into a model.
fn parse_dump(text: &str) -> Model {
    let mut m = Model::default();
    let mut lines = text.lines();
    while let Some(l) = lines.next() {
        let t: Vec<&str> = l.split_whitespace().collect();
        match t.first().copied() {
            Some("Z") => {
                m.nverts = t[1].parse().unwrap();
                m.ncells = t[2].parse().unwrap();
            }
            Some("P") => m.points.push([t[1].parse().unwrap(), t[2].parse().unwrap(), t[3].parse().unwrap()]),
            Some("S") => {
                let nums = |l: &str| -> Vec<i64> { l.split_whitespace().skip(2).map(|x| x.parse().unwrap()).collect() };
                let conn = nums(lines.next().unwrap());
                let offs = nums(lines.next().unwrap());
                m.secs.push(Sec { name: t[1].into(), etype: t[2].parse().unwrap(), first: t[3].parse().unwrap(), last: t[4].parse().unwrap(), conn, offs });
            }
            Some("B") => m.bcs.push((t[1].into(), t[3..].iter().map(|x| x.parse().unwrap()).collect())),
            Some("R") => m.blocks.push((t[1].into(), t[3..].iter().map(|x| x.parse().unwrap()).collect())),
            Some("F") => {
                m.families.insert(t[1].into(), t.get(2).map(|s| s.to_string()));
            }
            _ => {}
        }
    }
    m
}

/// `SM_CONFORMANCE=1`: `cgnscheck` passes, and the CGNS library reads the same counts, sections,
/// coordinates and BCs we wrote; cell volumes recomputed from what the library read equal the
/// fixtures' analytic volumes to 1e-12 relative, on a hex brick, a tet mesh, a mixed mesh and a
/// polyhedral dual, in both layouts.
#[test]
fn cgns_library_reads_the_file_and_cgnscheck_passes() {
    if std::env::var("SM_CONFORMANCE").as_deref() != Ok("1") {
        eprintln!("skipped (set SM_CONFORMANCE=1)");
        return;
    }
    let image = std::env::var("SM_CONFORMANCE_IMAGE").unwrap_or_else(|_| "smesh-dagmc-conformance".into());
    if !docker_ok(&image) {
        assert_eq!(std::env::var("SM_CONFORMANCE_ALLOW_SKIP").as_deref(), Ok("1"), "docker image {image} is missing");
        eprintln!("skipped: no docker image {image}");
        return;
    }
    let scratch = Scratch(std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("sm-cgns-conf-{}", std::process::id())));
    std::fs::create_dir_all(&scratch.0).unwrap();
    let dir = scratch.0.canonicalize().unwrap();
    std::fs::write(dir.join("cgns_dump.c"), include_str!("cgns_dump.c")).unwrap();
    let out = sh(&image, &dir, "/opt/conda/bin/gcc -O1 -o cgns_dump cgns_dump.c -I/opt/conda/include -L/opt/conda/lib -lcgns -Wl,-rpath,/opt/conda/lib 2>&1 && echo built");
    assert!(out.trim_end().ends_with("built"), "image {image} needs conda-forge `cgns` and a compiler (rebuild tools/conformance/dagmc after ACR-EXPFMT):\n{out}");
    let mut alloc = SerialAlloc::new();
    let brick = MeshFixture { name: "brick", mesh: hex_block([4, 3, 2], [0.25, 0.5, 0.125], &mut alloc).unwrap(), cell_volumes: vec![0.25 * 0.5 * 0.125; 24], volume: 0.0, area: 2.0 * (1.0 * 1.5 + 1.0 * 0.25 + 1.5 * 0.25), boundary_faces: 2 * (12 + 8 + 6) };
    let mut cases: Vec<(String, MeshFixture, CgnsLayout)> = Vec::new();
    for f in [brick, single_tet().unwrap(), single_wedge().unwrap(), single_pyramid().unwrap(), mixed_transition().unwrap(), dual_patch().unwrap(), truncated_octahedron().unwrap(), hexagonal_prism(1.0, 2.0).unwrap()] {
        cases.push((format!("{}_auto", f.name), f.clone(), CgnsLayout::Auto));
        cases.push((format!("{}_ngon", f.name), f, CgnsLayout::Ngon));
    }
    let mut written = Vec::new();
    for (name, f, layout) in &cases {
        let (bytes, r) = write(&f.mesh, &layout_opts(*layout));
        std::fs::write(dir.join(format!("{name}.cgns")), &bytes).unwrap();
        written.push((name.clone(), r));
    }
    for ((name, f, _), (_, r)) in cases.iter().zip(&written) {
        let log = sh(&image, &dir, &format!("/opt/conda/bin/cgnscheck {name}.cgns 2>&1; echo \"exit $?\""));
        // `cgnscheck` exits 0 whatever it finds: the verdict is its summary, "N errors".
        let errors = log.lines().find_map(|l| l.strip_suffix(" errors").and_then(|n| n.trim().parse::<u32>().ok())).unwrap_or(0);
        assert!(log.contains("checking complete") && errors == 0 && !log.contains("ERROR"), "{name}: cgnscheck:\n{log}");
        // Only the library's own recommendations may remain: a zone family and a data class.
        for w in log.lines().filter(|l| l.starts_with("WARNING")) {
            assert!(w.starts_with("WARNING:No family name declared for zone") || w == "WARNING:dataclass not given", "{name}: {w}");
        }
        let dump = sh(&image, &dir, &format!("./cgns_dump {name}.cgns"));
        assert!(dump.starts_with("Z "), "{name}: library dump:\n{dump}");
        let lib = parse_dump(&dump);
        // What the library read is what we wrote.
        let ours = model(&std::fs::read(dir.join(format!("{name}.cgns"))).unwrap());
        assert_eq!((lib.nverts, lib.ncells), (ours.nverts, ours.ncells), "{name}");
        assert_eq!(lib.points, ours.points, "{name}: coordinates");
        assert_eq!(lib.secs.len(), ours.secs.len(), "{name}");
        for (a, b) in lib.secs.iter().zip(&ours.secs) {
            assert_eq!((&a.name, a.etype, a.first, a.last, &a.conn, &a.offs), (&b.name, b.etype, b.first, b.last, &b.conn, &b.offs), "{name}: section {}", b.name);
        }
        assert_eq!(lib.bcs, ours.bcs, "{name}: BCs");
        assert_eq!(lib.blocks, ours.blocks, "{name}: sub-regions");
        assert_eq!(lib.families, ours.families, "{name}: families");
        assert!(dump.lines().filter(|l| l.starts_with("L ")).all(|l| l.ends_with(" FaceCenter")), "{name}: BC grid location");
        // Analytic checks on the library's data alone.
        check_model(f, &lib, r);
    }
    // Negative test: a cell that references a face that does not exist is caught by the same
    // `cgnscheck` (so a clean verdict above is not vacuous).
    let mut root = read_file(&std::fs::read(dir.join("brick_ngon.cgns")).unwrap()).unwrap();
    let Some(Obj::Group(zone)) = root.children.get_mut("Base").and_then(|b| match b {
        Obj::Group(b) => b.children.get_mut("Zone"),
        _ => None,
    }) else {
        panic!("no zone")
    };
    let Some(Obj::Group(nface)) = zone.children.get_mut("NFACE") else { panic!("no NFACE") };
    let Some(Obj::Group(conn)) = nface.children.get_mut("ElementConnectivity") else { panic!("no connectivity") };
    let Some(Obj::Dataset(d)) = conn.children.get_mut(" data") else { panic!("no data") };
    d.data[..4].copy_from_slice(&99_999i32.to_le_bytes());
    std::fs::write(dir.join("broken.cgns"), sm_meshio::h5::write_file(root).unwrap()).unwrap();
    let log = sh(&image, &dir, "/opt/conda/bin/cgnscheck broken.cgns 2>&1");
    let errors = log.lines().find_map(|l| l.strip_suffix(" errors").and_then(|n| n.trim().parse::<u32>().ok())).unwrap_or(0);
    assert!(errors > 0 && log.contains("ERROR:"), "a dangling face reference must be reported:\n{log}");
}
