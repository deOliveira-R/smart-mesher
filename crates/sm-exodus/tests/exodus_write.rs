//! Writer acceptance: header bytes, golden CDL, blocks and sets, round trips, polyhedra,
//! determinism, errors.

mod common;

use common::*;
use sm_exodus::netcdf::{NcFile, NcFormat};
use sm_exodus::read::{ExodusData, read_exodus};
use sm_exodus::*;
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::*;
use sm_testkit::meshes;

const FORMATS: [NcFormat; 3] = [NcFormat::Classic, NcFormat::Offset64, NcFormat::Data64];

/// `single_hex` with node set `ns` = vertices {1,2,5,6} (the x = 1 face) and a side set on the
/// x = 1 face (Left). Values checked by hand against the task: `node_ns1 = 2,3,6,7`,
/// `elem_ss1 = 1`, `side_ss1 = 2`, `coordx = 0,1,1,0,0,1,1,0`, `eb_prop1 = 1`.
const HEX_CDL: &str = r#"netcdf file {
dimensions:
	time_step = UNLIMITED ; // (0 currently)
	len_string = 33 ;
	len_line = 81 ;
	four = 4 ;
	len_name = 33 ;
	num_dim = 3 ;
	num_nodes = 8 ;
	num_elem = 1 ;
	num_el_blk = 1 ;
	num_node_sets = 1 ;
	num_side_sets = 1 ;
	num_qa_rec = 1 ;
	num_el_in_blk1 = 1 ;
	num_nod_per_el1 = 8 ;
	num_nod_ns1 = 4 ;
	num_side_ss1 = 1 ;
variables:
	double time_whole(time_step) ;
	double coordx(num_nodes) ;
	double coordy(num_nodes) ;
	double coordz(num_nodes) ;
	char coor_names(num_dim, len_name) ;
	int eb_status(num_el_blk) ;
	int eb_prop1(num_el_blk) ;
		eb_prop1:name = "ID" ;
	char eb_names(num_el_blk, len_name) ;
	int connect1(num_el_in_blk1, num_nod_per_el1) ;
		connect1:elem_type = "HEX8" ;
	int ns_status(num_node_sets) ;
	int ns_prop1(num_node_sets) ;
		ns_prop1:name = "ID" ;
	char ns_names(num_node_sets, len_name) ;
	int node_ns1(num_nod_ns1) ;
	int ss_status(num_side_sets) ;
	int ss_prop1(num_side_sets) ;
		ss_prop1:name = "ID" ;
	char ss_names(num_side_sets, len_name) ;
	int elem_ss1(num_side_ss1) ;
	int side_ss1(num_side_ss1) ;
	int elem_num_map(num_elem) ;
	char qa_records(num_qa_rec, four, len_string) ;

// global attributes:
		:api_version = 8.11f ;
		:version = 8.11f ;
		:floating_point_word_size = 8 ;
		:file_size = 1 ;
		:maximum_name_length = 32 ;
		:int64_status = 0 ;
		:title = "smart-mesh" ;
data:

 coordx = 0, 1, 1, 0, 0, 1, 1, 0 ;

 coordy = 0, 0, 1, 1, 0, 0, 1, 1 ;

 coordz = 0, 0, 0, 0, 1, 1, 1, 1 ;

 coor_names =
  "x",
  "y",
  "z" ;

 eb_status = 1 ;

 eb_prop1 = 1 ;

 eb_names = "unassigned_HEX8" ;

 connect1 = 1, 2, 3, 4, 5, 6, 7, 8 ;

 ns_status = 1 ;

 ns_prop1 = 1 ;

 ns_names = "ns" ;

 node_ns1 = 2, 3, 6, 7 ;

 ss_status = 1 ;

 ss_prop1 = 1 ;

 ss_names = "ss" ;

 elem_ss1 = 1 ;

 side_ss1 = 2 ;

 elem_num_map = 1 ;

 qa_records =
  "smart-mesh",
  "@VERSION@",
  "1970/01/01",
  "00:00:00" ;
}
"#;

fn hex_with_sets() -> ReferenceMesh {
    let mut fx = meshes::single_hex().unwrap();
    let mut a = SerialAlloc::new();
    add_set(&mut fx.mesh, &mut a, "ns", None, verts(&[1, 2, 5, 6]));
    let f = face_with_vertices(&fx.mesh, &[1, 2, 5, 6]);
    add_set(&mut fx.mesh, &mut a, "ss", None, sides(&[left(f)]));
    fx.mesh
}

fn be32(b: &[u8], at: usize) -> u32 {
    u32::from_be_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]])
}

#[test]
fn acc1_header_bytes_single_hex() {
    let m = meshes::single_hex().unwrap().mesh;
    for (fmt, magic) in [(NcFormat::Classic, 1u8), (NcFormat::Offset64, 2), (NcFormat::Data64, 5)] {
        let b = exodus_bytes(&m, &ExodusOptions::default().format(fmt)).unwrap();
        assert_eq!(&b[..4], &[0x43, 0x44, 0x46, magic]);
        if fmt == NcFormat::Data64 {
            assert_eq!(&b[4..12], &[0; 8], "numrecs (8 bytes)");
            assert_eq!(be32(&b, 12), 0x0A, "NC_DIMENSION");
        } else {
            assert_eq!(be32(&b, 4), 0, "numrecs");
            assert_eq!(be32(&b, 8), 0x0A, "NC_DIMENSION");
        }
    }
    // Offset64 is the default flavour.
    assert_eq!(exodus_bytes(&m, &ExodusOptions::default()).unwrap()[3], 2);
}

#[test]
fn acc1_exodus_layout_is_aligned_and_ascending() {
    let m = hex_with_sets();
    for fmt in FORMATS {
        let opts = ExodusOptions::default().format(fmt);
        let p = plan(&m, &opts).unwrap();
        let nc = exodus_nc(&m, &opts, &p).unwrap();
        let lay = nc.layout().unwrap();
        let fixed: Vec<_> = lay.vars.iter().filter(|v| !v.is_record).collect();
        let mut prev = lay.header_size;
        for v in &fixed {
            assert_eq!(v.begin % 4, 0);
            assert_eq!(v.vsize % 4, 0);
            assert_eq!(v.begin, prev, "{}", v.name);
            prev = v.begin + v.vsize;
        }
        let bytes = exodus_bytes(&m, &opts).unwrap();
        let last = fixed.last().unwrap();
        assert_eq!(bytes.len() as u64, last.begin + last.vsize);
        // The record variable (time_whole) holds zero records and starts at end of file.
        let rec = lay.vars.iter().find(|v| v.is_record).unwrap();
        assert_eq!((rec.name.as_str(), rec.begin, rec.vsize), ("time_whole", bytes.len() as u64, 8));
    }
}

#[test]
fn acc2_golden_cdl_single_hex() {
    let m = hex_with_sets();
    let opts = ExodusOptions::default();
    let p = plan(&m, &opts).unwrap();
    let cdl = exodus_nc(&m, &opts, &p).unwrap().to_cdl();
    let golden = HEX_CDL.replace("@VERSION@", env!("CARGO_PKG_VERSION"));
    assert_eq!(cdl, golden);
    // The CDL of the file read back equals the CDL of the model.
    let bytes = exodus_bytes(&m, &opts).unwrap();
    assert_eq!(NcFile::read(&bytes).unwrap().to_cdl(), golden);
}

#[test]
fn acc2_simple_shapes() {
    let cases: [(meshes::MeshFixture, &str, usize, Vec<[f64; 3]>); 3] = [
        (meshes::single_tet().unwrap(), "TET4", 4, vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]),
        (
            meshes::single_wedge().unwrap(),
            "WEDGE6",
            6,
            vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [0., 1., 1.]],
        ),
        (
            meshes::single_pyramid().unwrap(),
            "PYRAMID5",
            5,
            vec![[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0.5, 0.5, 1.]],
        ),
    ];
    for (fx, ty, n, pts) in cases {
        let d = read_exodus(&exodus_bytes(&fx.mesh, &ExodusOptions::default()).unwrap()).unwrap();
        assert_eq!(d.blocks.len(), 1);
        assert_eq!(d.blocks[0].elem_type, ty);
        assert_eq!(d.blocks[0].name, format!("unassigned_{ty}"));
        assert_eq!(d.blocks[0].id, 1);
        assert_eq!(d.blocks[0].connectivity, (1..=n as i64).collect::<Vec<_>>());
        assert_eq!((d.num_nodes, d.num_elem, d.num_dim), (n, 1, 3));
        for (i, p) in pts.iter().enumerate() {
            assert_eq!(node_xyz(&d, i), *p);
        }
        assert!(d.sidesets.is_empty() && d.nodesets.is_empty());
    }
}

/// Element face centroid x of side `side` of element `elem` (1-based) read back from the file.
fn side_centroid(d: &ExodusData, elem: usize, side: usize) -> [f64; 3] {
    let mut start = 0;
    for b in &d.blocks {
        if elem - 1 < start + b.num_elem {
            let npe = b.connectivity.len() / b.num_elem;
            let row = &b.connectivity[(elem - 1 - start) * npe..(elem - start) * npe];
            let pts: Vec<[f64; 3]> = manual_side_nodes(&b.elem_type, side).iter().map(|n| node_xyz(d, (row[n - 1] - 1) as usize)).collect();
            return mean(&pts);
        }
        start += b.num_elem;
    }
    panic!("element {elem}");
}

#[test]
fn acc5_hex_block_sides_and_blocks() {
    let mut m = meshes::hex_block_2x2x2().unwrap().mesh;
    let mut a = SerialAlloc::new();
    // The four boundary faces on x = 1.
    let xs = m.coords().x.to_vec();
    let faces: Vec<SignedFace> = (0..m.face_count())
        .map(FaceIx::new)
        .filter(|f| m.face_right(*f).is_none() && m.face_vertices(*f).iter().all(|v| xs[v.get()] == 1.0))
        .map(left)
        .collect();
    assert_eq!(faces.len(), 4);
    add_set(&mut m, &mut a, "xmax", Some(10), sides(&faces));
    // Two disjoint block sets of 4 cells each, interleaved so the element numbering permutes.
    add_set(&mut m, &mut a, "even", Some(2), cells(&[0, 2, 4, 6]));
    add_set(&mut m, &mut a, "odd", Some(1), cells(&[1, 3, 5, 7]));
    let bytes = exodus_bytes(&m, &ExodusOptions::default()).unwrap();
    let d = read_exodus(&bytes).unwrap();
    assert_eq!(d.blocks.iter().map(|b| (b.id, b.name.as_str(), b.num_elem)).collect::<Vec<_>>(), [(1, "odd", 4), (2, "even", 4)]);
    assert_eq!(d.elem_num_map, vec![2, 4, 6, 8, 1, 3, 5, 7]);
    let mut sorted = d.elem_num_map.clone();
    sorted.sort_unstable();
    assert_eq!(sorted, (1..=8).collect::<Vec<_>>());
    let ss = &d.sidesets[0];
    assert_eq!((ss.id, ss.name.as_str(), ss.elems.len()), (10, "xmax", 4));
    assert!(ss.sides.iter().all(|s| *s == 2));
    for e in &ss.elems {
        assert_eq!(side_centroid(&d, *e as usize, 2)[0], 1.0);
    }
    // Each named element really is a cell owning an x = 1 boundary face.
    for (e, f) in ss.elems.iter().zip(&faces) {
        let cell = d.elem_num_map[*e as usize - 1] as u32 - 1;
        assert_eq!(m.face_left(f.face()).0, cell);
    }
}

#[test]
fn acc5_mixed_transition_blocks() {
    let mut m = meshes::mixed_transition().unwrap().mesh;
    let mut a = SerialAlloc::new();
    let by_shape = |m: &ReferenceMesh, s: CellShape| (0..m.cell_count()).find(|c| m.cell_shape(CellIx::new(*c)) == s).unwrap() as u32;
    for (i, (name, s)) in [("h", CellShape::Hex), ("w", CellShape::Wedge), ("p", CellShape::Pyramid), ("t", CellShape::Tet)].iter().enumerate() {
        let c = by_shape(&m, *s);
        add_set(&mut m, &mut a, name, Some(i as i64 + 1), cells(&[c]));
    }
    let rep = write_exodus(&m, &ExodusOptions::default(), &mut std::io::sink()).unwrap();
    let got: Vec<(i64, &str, usize)> = rep.blocks.iter().map(|b| (b.id, b.elem_type.as_str(), b.count)).collect();
    assert_eq!(got, [(1, "HEX8", 1), (2, "WEDGE6", 1), (3, "PYRAMID5", 1), (4, "TET4", 1)]);
    assert_eq!(rep.num_elem, 4);
    // Without sets: unassigned blocks by shape code (Tet, Hex, Wedge, Pyramid).
    let bare = meshes::mixed_transition().unwrap().mesh;
    let rep = write_exodus(&bare, &ExodusOptions::default(), &mut std::io::sink()).unwrap();
    let got: Vec<(i64, &str)> = rep.blocks.iter().map(|b| (b.id, b.name.as_str())).collect();
    assert_eq!(got, [(1, "unassigned_TET4"), (2, "unassigned_HEX8"), (3, "unassigned_WEDGE6"), (4, "unassigned_PYRAMID5")]);
}

#[test]
fn mixed_set_splits_by_shape() {
    let mut m = meshes::mixed_transition().unwrap().mesh;
    add_set(&mut m, &mut SerialAlloc::new(), "all", Some(5), cells(&[0, 1, 2, 3]));
    let rep = write_exodus(&m, &ExodusOptions::default(), &mut std::io::sink()).unwrap();
    let got: Vec<(i64, &str, &str)> = rep.blocks.iter().map(|b| (b.id, b.name.as_str(), b.elem_type.as_str())).collect();
    // Tet (lowest code) keeps label and name; Hex, Wedge, Pyramid get 6, 7, 8.
    assert_eq!(got, [(5, "all", "TET4"), (6, "all_HEX8", "HEX8"), (7, "all_WEDGE6", "WEDGE6"), (8, "all_PYRAMID5", "PYRAMID5")]);
}

#[test]
fn block_selection_options() {
    let mut m = meshes::hex_block_2x2x2().unwrap().mesh;
    let mut a = SerialAlloc::new();
    add_set(&mut m, &mut a, "lo", Some(3), cells(&[0, 1, 2, 3]));
    add_set(&mut m, &mut a, "hi", None, cells(&[4, 5, 6, 7]));
    let names = |o: &ExodusOptions| write_exodus(&m, o, &mut std::io::sink()).unwrap().blocks.iter().map(|b| (b.id, b.name.clone())).collect::<Vec<_>>();
    // Labelled: only "lo"; the rest is unassigned with a fresh id above the largest label.
    assert_eq!(names(&ExodusOptions::default()), [(3, "lo".to_string()), (4, "unassigned_HEX8".to_string())]);
    // Names: "hi" has no label, so it gets a fresh id.
    let o = ExodusOptions::default().blocks(BlockSelection::Names(vec!["hi".into()]));
    assert_eq!(names(&o), [(1, "hi".to_string()), (2, "unassigned_HEX8".to_string())]);
    let o = ExodusOptions::default().blocks(BlockSelection::None);
    assert_eq!(names(&o), [(1, "unassigned_HEX8".to_string())]);
    let o = ExodusOptions::default().blocks(BlockSelection::Names(vec!["nope".into()]));
    assert!(matches!(write_exodus(&m, &o, &mut std::io::sink()), Err(ExodusError::Mesh(_))));
}

#[test]
fn set_ids_and_deck_face_sets() {
    let mut m = meshes::hex_block_2x2x2().unwrap().mesh;
    let mut a = SerialAlloc::new();
    let b: Vec<FaceIx> = (0..m.face_count()).map(FaceIx::new).filter(|f| m.face_right(*f).is_none()).collect();
    add_set(&mut m, &mut a, "deck", Some(4), SetMembers::Faces(b[..3].to_vec()));
    add_set(&mut m, &mut a, "free", None, sides(&b[3..5].iter().map(|f| left(*f)).collect::<Vec<_>>()));
    add_set(&mut m, &mut a, "n1", Some(9), verts(&[0, 1]));
    add_set(&mut m, &mut a, "n2", None, verts(&[2, 3]));
    add_set(&mut m, &mut a, "empty", None, verts(&[]));
    let rep = write_exodus(&m, &ExodusOptions::default(), &mut std::io::sink()).unwrap();
    let ss: Vec<_> = rep.sidesets.iter().map(|s| (s.id, s.name.as_str(), s.count)).collect();
    assert_eq!(ss, [(4, "deck", 3), (5, "free", 2)]);
    let ns: Vec<_> = rep.nodesets.iter().map(|s| (s.id, s.name.as_str(), s.count)).collect();
    assert_eq!(ns, [(9, "n1", 2), (10, "n2", 2)]); // empty set not written
    let o = ExodusOptions::default().faces_as_sidesets(false);
    let rep = write_exodus(&m, &o, &mut std::io::sink()).unwrap();
    assert_eq!(rep.sidesets.len(), 1);
    assert_eq!(rep.sidesets[0].id, 1, "only candidate is unlabelled: first id above max label 0");
}

// ---- round trip (acceptance 6) -------------------------------------------------------------

fn boundary_faces(m: &dyn MeshRead) -> Vec<FaceIx> {
    (0..m.face_count()).map(FaceIx::new).filter(|f| m.face_right(*f).is_none()).collect()
}

fn decorate(m: &mut ReferenceMesh) {
    let mut a = SerialAlloc::new();
    let nv = m.vertex_count() as u32;
    let b = boundary_faces(m);
    add_set(m, &mut a, "vs", Some(7), verts(&[0, nv - 1]));
    add_set(m, &mut a, "bnd", Some(3), sides(&b.iter().map(|f| left(*f)).collect::<Vec<_>>()));
    add_set(m, &mut a, "deck", Some(4), SetMembers::Faces(vec![b[0]]));
    add_set(m, &mut a, "blk", Some(2), cells(&[0]));
}

fn rich_options(fmt: NcFormat) -> ExodusOptions {
    ExodusOptions::default()
        .format(fmt)
        .title("round trip")
        .write_node_num_map(true)
        .qa(vec![["code".into(), "1.2".into(), "2020/02/02".into(), "01:02:03".into()]])
        .info(vec!["first line".into(), "second line".into()])
        .provenance("abc123")
}

#[test]
fn acc6_round_trip_every_fixture_and_format() {
    for fx in meshes::all_mesh_fixtures().unwrap() {
        let mut m = fx.mesh.clone();
        decorate(&mut m);
        let mut decoded: Vec<ExodusData> = Vec::new();
        for fmt in FORMATS {
            let d = read_exodus(&exodus_bytes(&m, &rich_options(fmt)).unwrap()).unwrap();
            assert_eq!(d.format, fmt);
            assert_eq!(d.title, "round trip");
            assert_eq!(d.qa, vec![["code".to_string(), "1.2".into(), "2020/02/02".into(), "01:02:03".into()]]);
            assert_eq!(d.info, ["first line", "second line", "smart-mesh provenance: abc123"]);
            // Coordinates bit for bit.
            let nv = m.vertex_count();
            assert_eq!(d.num_nodes, nv);
            let c = m.coords();
            for (axis, col) in [&c.x[..nv], &c.y[..nv], &c.z[..nv]].iter().enumerate() {
                let got: Vec<u64> = d.coords[axis].iter().map(|x| x.to_bits()).collect();
                let want: Vec<u64> = col.iter().map(|x| x.to_bits()).collect();
                assert_eq!(got, want, "{} axis {axis}", fx.name);
            }
            assert_eq!(d.node_num_map, Some((1..=nv as i64).collect()));
            // Per-cell connectivity of template cells.
            let mut perm = d.elem_num_map.clone();
            perm.sort_unstable();
            assert_eq!(perm, (1..=m.cell_count() as i64).collect::<Vec<_>>());
            let mut e = 0usize;
            for b in &d.blocks {
                for r in 0..b.num_elem {
                    let cell = CellIx::new(d.elem_num_map[e] as usize - 1);
                    if let Some(corners) = m.cell_corners(cell).filter(|_| b.elem_type != "NFACED") {
                        let row = &b.connectivity[r * corners.len()..(r + 1) * corners.len()];
                        let want: Vec<i64> = corners.iter().map(|v| i64::from(v.0) + 1).collect();
                        assert_eq!(row, want, "{} cell {}", fx.name, cell.get());
                    }
                    e += 1;
                }
            }
            // Blocks: "blk" (id 2, cell 0) and the rest.
            let blk = d.blocks.iter().find(|b| b.id == 2).expect("block 2");
            assert_eq!((blk.name.as_str(), blk.num_elem), ("blk", 1));
            assert_eq!(d.elem_num_map[0], 1, "block 1 comes first or is the unassigned remainder");
            // Sets.
            assert_eq!(d.nodesets.len(), 1);
            assert_eq!((d.nodesets[0].id, d.nodesets[0].name.as_str()), (7, "vs"));
            assert_eq!(d.nodesets[0].nodes, vec![1, nv as i64]);
            let names: Vec<(i64, &str, usize)> = d.sidesets.iter().map(|s| (s.id, s.name.as_str(), s.elems.len())).collect();
            assert_eq!(names, [(3, "bnd", fx.boundary_faces), (4, "deck", 1)]);
            if d.face_blocks.len() > 1 {
                // Several NFACED blocks: the second face block has a nonzero global face base.
                check_poly(&m, &d);
            }
            decoded.push(d);
        }
        for d in &mut decoded {
            d.format = NcFormat::Classic;
        }
        assert_eq!(decoded[0], decoded[1], "{}", fx.name);
        assert_eq!(decoded[1], decoded[2], "{}", fx.name);
    }
}

#[test]
fn int64_ids_round_trip_in_data64() {
    let mut m = meshes::hex_block_2x2x2().unwrap().mesh;
    decorate(&mut m);
    let o64 = ExodusOptions::default().format(NcFormat::Data64).int64(true);
    let b64 = exodus_bytes(&m, &o64).unwrap();
    let mut d64 = read_exodus(&b64).unwrap();
    assert!(d64.int64);
    let d32 = read_exodus(&exodus_bytes(&m, &ExodusOptions::default().format(NcFormat::Data64)).unwrap()).unwrap();
    assert!(!d32.int64);
    d64.int64 = false;
    assert_eq!(d64, d32);
    // int64 needs Data64.
    let bad = ExodusOptions::default().int64(true);
    assert!(matches!(exodus_bytes(&m, &bad), Err(ExodusError::Unsupported(_))));
}

#[test]
fn qa_info_limits_and_defaults() {
    let m = meshes::single_hex().unwrap().mesh;
    let d = read_exodus(&exodus_bytes(&m, &ExodusOptions::default()).unwrap()).unwrap();
    assert_eq!(d.qa.len(), 1);
    assert_eq!(d.qa[0][0], "smart-mesh");
    assert_eq!(d.qa[0][2], "1970/01/01");
    assert!(d.info.is_empty());
    // A long info line is split at 80 bytes.
    let long = "x".repeat(170);
    let d = read_exodus(&exodus_bytes(&m, &ExodusOptions::default().info(vec![long])).unwrap()).unwrap();
    assert_eq!(d.info.iter().map(String::len).collect::<Vec<_>>(), [80, 80, 10]);
    // QA strings over 32 bytes are rejected.
    let q = ExodusOptions::default().qa(vec![["x".repeat(33), "a".into(), "b".into(), "c".into()]]);
    assert!(matches!(exodus_bytes(&m, &q), Err(ExodusError::NameTooLong { .. })));
    // No QA and no info: the dimensions are omitted (a zero-length dimension would mean "unlimited").
    let d = read_exodus(&exodus_bytes(&m, &ExodusOptions::default().qa(vec![])).unwrap()).unwrap();
    assert!(d.qa.is_empty() && d.info.is_empty());
}

#[test]
fn max_name_length_grows_to_the_longest_name() {
    let mut m = meshes::single_hex().unwrap().mesh;
    let name = "n".repeat(60);
    add_set(&mut m, &mut SerialAlloc::new(), &name, Some(1), cells(&[0]));
    let d = read_exodus(&exodus_bytes(&m, &ExodusOptions::default()).unwrap()).unwrap();
    assert_eq!(d.max_name_length, 60);
    assert_eq!(d.blocks[0].name, name);
    let o = ExodusOptions::default().max_name_length(300);
    assert!(matches!(exodus_bytes(&m, &o), Err(ExodusError::TooLarge { .. })));
}

// ---- polyhedra (acceptance 7) --------------------------------------------------------------

/// Check every NFACED element of a decoded file against the mesh: face order = `cell_faces`
/// order, loop orientation outward (reversed copies for Right entries), positive Newell dot.
fn check_poly(m: &ReferenceMesh, d: &ExodusData) {
    let mut faces: Vec<Vec<usize>> = Vec::new();
    for fb in &d.face_blocks {
        let mut at = 0;
        for n in &fb.entity_counts {
            faces.push(fb.connectivity[at..at + *n as usize].iter().map(|v| *v as usize - 1).collect());
            at += *n as usize;
        }
        assert_eq!(at, fb.connectivity.len());
        assert_eq!(fb.num_faces, fb.entity_counts.len());
    }
    let mut e = 0usize;
    let mut checked = 0;
    for b in d.blocks.iter() {
        let mut fa = 0usize;
        for r in 0..b.num_elem {
            let cell = CellIx::new(d.elem_num_map[e] as usize - 1);
            e += 1;
            if b.elem_type != "NFACED" {
                continue;
            }
            let n = b.entity_counts[r] as usize;
            let mine = m.cell_faces(cell);
            assert_eq!(n, mine.len());
            let nums = &b.face_connectivity[fa..fa + n];
            fa += n;
            let loops: Vec<&Vec<usize>> = nums.iter().map(|f| &faces[*f as usize - 1]).collect();
            let mut all: Vec<[f64; 3]> = Vec::new();
            for l in &loops {
                all.extend(l.iter().map(|v| node_xyz(d, *v)));
            }
            let cc = mean(&{
                let mut u: Vec<usize> = loops.iter().flat_map(|l| l.iter().copied()).collect();
                u.sort_unstable();
                u.dedup();
                u.iter().map(|v| node_xyz(d, *v)).collect::<Vec<_>>()
            });
            for (l, sf) in loops.iter().zip(mine) {
                let lv: Vec<VertexIx> = l.iter().map(|v| VertexIx::new(*v)).collect();
                let want = if sf.side() == Side::Left { LoopRelation::Same } else { LoopRelation::Reversed };
                assert_eq!(loop_relation(m.face_vertices(sf.face()), &lv), Some(want));
                let pts: Vec<[f64; 3]> = l.iter().map(|v| node_xyz(d, *v)).collect();
                assert!(dot(newell(&pts), sub(mean(&pts), cc)) > 0.0, "face copy not outward");
            }
            checked += 1;
        }
    }
    assert!(checked > 0);
}

#[test]
fn acc7_truncated_octahedron() {
    let m = meshes::truncated_octahedron().unwrap().mesh;
    let bytes = exodus_bytes(&m, &ExodusOptions::default()).unwrap();
    let d = read_exodus(&bytes).unwrap();
    assert_eq!(d.blocks.len(), 1);
    assert_eq!((d.blocks[0].elem_type.as_str(), d.blocks[0].num_elem), ("NFACED", 1));
    assert_eq!(d.blocks[0].entity_counts, [14]);
    assert_eq!(d.face_blocks.len(), 1);
    assert_eq!(d.face_blocks[0].num_faces, 14);
    let mut counts = d.face_blocks[0].entity_counts.clone();
    counts.sort_unstable();
    assert_eq!(counts, [vec![4; 6], vec![6; 8]].concat());
    check_poly(&m, &d);
    // libexodus dimension names (recalled from exodusII.h).
    let nc = NcFile::read(&bytes).unwrap();
    assert_eq!(nc.dim_len("num_fac_per_el1"), Some(14));
    assert_eq!(nc.dim_len("num_fa_in_blk1"), Some(14));
    let fb = nc.var_by_name("fbconn1").unwrap();
    assert!(fb.attrs.iter().any(|(n, v)| n == "elem_type" && v.as_text() == Some("NSIDED")));
    // Exodus spellings only.
    for needle in [&b"POLYHEDRON"[..], &b"POLYGON"[..]] {
        assert!(!bytes.windows(needle.len()).any(|w| w == needle));
    }
    let rep = write_exodus(&m, &ExodusOptions::default(), &mut std::io::sink()).unwrap();
    assert_eq!(rep.blocks[0].elem_type, "NFACED");
}

#[test]
fn acc7_prism_and_dual_patch() {
    let m = meshes::hexagonal_prism(1.0, 2.0).unwrap().mesh;
    let d = read_exodus(&exodus_bytes(&m, &ExodusOptions::default()).unwrap()).unwrap();
    assert_eq!(d.blocks[0].entity_counts, [8]);
    check_poly(&m, &d);
    // Seven prisms: interior faces are written once per cell, reversed for the Right copy.
    let m = meshes::dual_patch().unwrap().mesh;
    let d = read_exodus(&exodus_bytes(&m, &ExodusOptions::default()).unwrap()).unwrap();
    assert_eq!(d.blocks.len(), 1);
    assert_eq!(d.blocks[0].num_elem, 7);
    assert_eq!(d.face_blocks[0].num_faces, 7 * 8);
    check_poly(&m, &d);
    let m = meshes::split_face_cube().unwrap().mesh;
    let d = read_exodus(&exodus_bytes(&m, &ExodusOptions::default()).unwrap()).unwrap();
    assert_eq!(d.blocks[0].entity_counts, [7]);
    check_poly(&m, &d);
}

#[test]
fn acc7_sides_on_polyhedra_are_face_list_positions() {
    let mut m = meshes::truncated_octahedron().unwrap().mesh;
    let all = all_sides_of(&m, 0..1);
    add_set(&mut m, &mut SerialAlloc::new(), "all", Some(1), sides(&all));
    let d = read_exodus(&exodus_bytes(&m, &ExodusOptions::default()).unwrap()).unwrap();
    assert_eq!(d.sidesets[0].sides, (1..=14).collect::<Vec<i64>>());
}

fn hex_next_to_polyhedron() -> ReferenceMesh {
    let mut p: Vec<[f64; 3]> = vec![[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];
    p.extend([[2., 0., 0.], [2., 1., 0.], [2., 0., 1.], [2., 1., 1.]]);
    let poly = vec![vec![1, 5, 6, 2], vec![8, 9, 11, 10], vec![1, 2, 9, 8], vec![5, 10, 11], vec![5, 11, 6], vec![1, 8, 10, 5], vec![2, 6, 11, 9]];
    meshes::build_mesh(&p, &[meshes::CellSpec::Template(CellShape::Hex, (0..8).collect()), meshes::CellSpec::Poly(poly)], &mut SerialAlloc::new()).unwrap()
}

#[test]
fn acc7_mixed_template_and_nfaced_blocks() {
    let mut m = hex_next_to_polyhedron();
    assert_eq!(m.cell_shape(CellIx(1)), CellShape::Polyhedron);
    let shared = face_with_vertices(&m, &[1, 2, 5, 6]);
    assert_eq!(m.face_left(shared), CellIx(0));
    add_set(&mut m, &mut SerialAlloc::new(), "iface", Some(1), sides(&[left(shared), right(shared)]));
    let d = read_exodus(&exodus_bytes(&m, &ExodusOptions::default()).unwrap()).unwrap();
    let kinds: Vec<&str> = d.blocks.iter().map(|b| b.elem_type.as_str()).collect();
    assert_eq!(kinds, ["HEX8", "NFACED"]); // fresh ids in shape-code order: Hex (1) < Polyhedron (4)
    check_poly(&m, &d);
    // Left = hex side 2 (x = 1); Right = position of the shared face in the polyhedron's list.
    let pos = m.cell_faces(CellIx(1)).iter().position(|sf| sf.face() == shared).unwrap() as i64 + 1;
    assert_eq!(d.sidesets[0].elems, [1, 2]);
    assert_eq!(d.sidesets[0].sides, [2, pos]);
    // Disabling polyhedra is a typed error.
    let o = ExodusOptions::default().polyhedra(false);
    assert!(matches!(exodus_bytes(&m, &o), Err(ExodusError::Unsupported(_))));
}

// ---- dimension 2 ---------------------------------------------------------------------------

#[test]
fn quad_mesh_is_two_dimensional() {
    let fx = sm_testkit::lattice::quad([3, 2], [3.0, 2.0]).unwrap();
    let mut m = fx.mesh.clone();
    let b = boundary_faces(&m);
    add_set(&mut m, &mut SerialAlloc::new(), "edges", Some(1), sides(&b.iter().map(|f| left(*f)).collect::<Vec<_>>()));
    let bytes = exodus_bytes(&m, &ExodusOptions::default()).unwrap();
    let nc = NcFile::read(&bytes).unwrap();
    assert_eq!(nc.dim_len("num_dim"), Some(2));
    assert!(nc.var_by_name("coordz").is_none());
    let d = read_exodus(&bytes).unwrap();
    assert_eq!((d.num_dim, d.blocks[0].elem_type.as_str(), d.num_elem), (2, "QUAD4", 6));
    assert_eq!(d.coord_names, ["x", "y"]);
    // Each side names the edge (node s -> node s+1) of its element, as the face loop says.
    let ss = &d.sidesets[0];
    for (i, f) in b.iter().enumerate() {
        let e = ss.elems[i] as usize - 1;
        let conn = &d.blocks[0].connectivity[e * 4..e * 4 + 4];
        let nodes = manual_side_nodes("QUAD4", ss.sides[i] as usize);
        let got: Vec<i64> = nodes.iter().map(|n| conn[n - 1] - 1).collect();
        let want: Vec<i64> = m.face_vertices(*f).iter().map(|v| i64::from(v.0)).collect();
        assert_eq!(got, want);
    }
}

fn pentagon_and_triangle() -> ReferenceMesh {
    let mut b = ReferenceBuilder::surface();
    let face = sm_core::EntityId::new(sm_core::EntityKind::TopoFace, 1).unwrap();
    let pts = [[0., 0.], [2., 0.], [3., 1.], [1.5, 2.], [0., 1.], [3., 0.]];
    let vs: Vec<VertexIx> = pts.iter().map(|p| b.add_vertex(sm_core::Point3::new(p[0], p[1], 0.0)).unwrap()).collect();
    let edge = sm_core::EntityId::new(sm_core::EntityKind::TopoEdge, 1).unwrap();
    for (i, v) in vs.iter().enumerate() {
        let tv = sm_core::EntityId::new(sm_core::EntityKind::TopoVertex, i as u64 + 1).unwrap();
        b.set_class(MeshEntityRef::Vertex(*v), Classification::on(tv)).unwrap();
    }
    for cell in [&[0usize, 1, 2, 3, 4][..], &[1, 5, 2][..]] {
        let verts: Vec<VertexIx> = cell.iter().map(|i| vs[*i]).collect();
        let added = if verts.len() == 3 {
            b.add_cell_from_vertices(CellShape::Tri, &verts).unwrap()
        } else {
            b.add_polygon(&verts).unwrap()
        };
        b.set_class(MeshEntityRef::Cell(added.cell), Classification::on(face)).unwrap();
    }
    for f in 0..b.face_count() {
        let f = FaceIx::new(f);
        let class = if b.face_degree(f) == 1 { edge } else { face };
        b.set_class(MeshEntityRef::Face(f), Classification::on(class)).unwrap();
    }
    b.finish(&mut SerialAlloc::new()).unwrap().mesh
}

#[test]
fn polygon_mesh_writes_nsided_with_cycle_positions() {
    let mut m = pentagon_and_triangle();
    assert_eq!(m.cell_shape(CellIx(0)), CellShape::Polygon);
    let shared = face_with_vertices(&m, &[1, 2]);
    assert_eq!(m.face_left(shared), CellIx(0));
    add_set(&mut m, &mut SerialAlloc::new(), "iface", Some(1), sides(&[left(shared), right(shared)]));
    let bytes = exodus_bytes(&m, &ExodusOptions::default()).unwrap();
    let d = read_exodus(&bytes).unwrap();
    assert_eq!(d.num_dim, 2);
    let kinds: Vec<(&str, i64)> = d.blocks.iter().map(|b| (b.elem_type.as_str(), b.id)).collect();
    assert_eq!(kinds, [("TRI3", 1), ("NSIDED", 2)]);
    let nsided = &d.blocks[1];
    assert_eq!(nsided.entity_counts, [5]);
    assert_eq!(nsided.connectivity, [1, 2, 3, 4, 5]);
    // Pentagon edge 2->3 is side 2; the triangle (2,6,3) traverses 3->2 as its edge 3 (3->1).
    assert_eq!(d.sidesets[0].elems, [2, 1]);
    assert_eq!(d.sidesets[0].sides, [2, 3]);
    assert!(!bytes.windows(7).any(|w| w == b"POLYGON"));
}

// ---- errors (acceptance 10) ----------------------------------------------------------------

#[test]
fn acc10_errors_never_panic() {
    let base = meshes::hex_block_2x2x2().unwrap().mesh;
    let w = |m: &dyn MeshRead| exodus_bytes(m, &ExodusOptions::default());

    // Empty mesh: every cell inactive.
    let mut over = Over::new(&base);
    over.inactive = vec![true; base.cell_count()];
    assert!(matches!(w(&over), Err(ExodusError::EmptyMesh)));

    // Overlapping block sets.
    let mut m = base.clone();
    let mut a = SerialAlloc::new();
    add_set(&mut m, &mut a, "a", Some(1), cells(&[0, 1, 2]));
    add_set(&mut m, &mut a, "b", Some(2), cells(&[2, 3]));
    match w(&m) {
        Err(ExodusError::OverlappingBlocks { cell: 2, a, b }) => assert_eq!((a.as_str(), b.as_str()), ("a", "b")),
        other => panic!("{other:?}"),
    }

    // Labels 0 and negative; duplicate labels across cell sets cannot be attached (mesh API).
    for bad in [0, -4] {
        let mut m = base.clone();
        add_set(&mut m, &mut SerialAlloc::new(), "z", Some(bad), cells(&[0]));
        assert!(matches!(w(&m), Err(ExodusError::BadLabel { label, .. }) if label == bad));
    }
    let mut m = base.clone();
    add_set(&mut m, &mut SerialAlloc::new(), "v", Some(0), verts(&[0]));
    assert!(matches!(w(&m), Err(ExodusError::BadLabel { .. })), "node set labels are checked too");

    // 300-byte name.
    let mut m = base.clone();
    add_set(&mut m, &mut SerialAlloc::new(), &"q".repeat(300), Some(1), cells(&[0]));
    assert!(matches!(w(&m), Err(ExodusError::NameTooLong { name }) if name.len() == 300));

    // High order.
    let over = Over::new(&base).with_high_order();
    assert!(matches!(w(&over), Err(ExodusError::Unsupported(m)) if m.contains("high-order")));

    // Bad input to the reader.
    let good = w(&base).unwrap();
    assert!(matches!(read_exodus(b"\x89HDF\r\n\x1a\nxxxx"), Err(ExodusError::Unsupported(_))));
    for cut in [0, 3, 10, 100, good.len() / 2, good.len() - 1] {
        assert!(matches!(read_exodus(&good[..cut]), Err(ExodusError::Malformed(_))), "cut {cut}");
    }
    // Display never panics.
    let _ = format!("{}", ExodusError::EmptyMesh);
}

#[test]
fn inactive_cells_are_skipped_and_counted() {
    let mut m = meshes::hex_block_2x2x2().unwrap().mesh;
    let b = boundary_faces(&m);
    add_set(&mut m, &mut SerialAlloc::new(), "bnd", Some(1), sides(&b.iter().map(|f| left(*f)).collect::<Vec<_>>()));
    let on_cell0 = b.iter().filter(|f| m.face_left(**f) == CellIx(0)).count();
    assert_eq!(on_cell0, 3);
    let mut over = Over::new(&m);
    over.inactive[0] = true;
    let rep = write_exodus(&over, &ExodusOptions::default(), &mut std::io::sink()).unwrap();
    assert_eq!((rep.num_elem, rep.skipped_sides), (7, 3));
    assert_eq!(rep.sidesets[0].count, 24 - 3);
}

// ---- determinism (acceptance 8) ------------------------------------------------------------

fn in_pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
}

#[test]
fn acc8_determinism_with_and_without_sets() {
    let plain = meshes::hex_block_2x2x2().unwrap().mesh;
    let mut sets = plain.clone();
    decorate(&mut sets);
    let poly = meshes::dual_patch().unwrap().mesh;
    for (name, m) in [("plain", &plain), ("sets", &sets), ("poly", &poly)] {
        for fmt in FORMATS {
            let h = sm_testkit::determinism::check(2, 4, |t| in_pool(t, || fnv1a(&exodus_bytes(m, &rich_options(fmt)).unwrap()))).unwrap();
            assert_ne!(h, 0, "{name}");
        }
    }
}

#[test]
fn acc8_regression_hash_of_a_20_cubed_block() {
    let m = sm_mesh_api::reference::hex_block([20, 20, 20], [0.05; 3], &mut SerialAlloc::new()).unwrap();
    let h = sm_testkit::determinism::check(1, 4, |t| in_pool(t, || fnv1a(&exodus_bytes(&m, &ExodusOptions::default()).unwrap()))).unwrap();
    assert_eq!(h, REGRESSION_HASH_20_CUBED, "Exodus bytes of the 20^3 hex block changed: {h:#018x}");
}

/// FNV-1a 64 of the Offset64 file of `hex_block([20;3], [0.05;3])` with default options.
const REGRESSION_HASH_20_CUBED: u64 = 0x56a9_e4cc_4928_6397;
