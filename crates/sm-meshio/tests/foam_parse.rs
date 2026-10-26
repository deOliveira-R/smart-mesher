//! The polyMesh parser on hostile and unusual input, and the tar writer.

mod common;

use common::foam::sets_block;
use proptest::prelude::*;
use sm_meshio::foam::*;
use sm_meshio::MeshioError;
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::SerialAlloc;
use sm_testkit::meshes::{hex_block_2x2x2, single_hex};

// ---- helpers --------------------------------------------------------------------------------------

fn good() -> Vec<FoamFile> {
    build_polymesh(&single_hex().unwrap().mesh, &FoamOptions::default()).unwrap().mesh.files().unwrap()
}

fn replace(files: &[FoamFile], name: &str, from: &str, to: &str) -> Vec<FoamFile> {
    files
        .iter()
        .map(|f| {
            if f.name == name {
                let t = String::from_utf8(f.bytes.clone()).unwrap();
                assert!(t.contains(from), "{name} has no '{from}'");
                FoamFile { name: f.name.clone(), bytes: t.replacen(from, to, 1).into_bytes() }
            } else {
                f.clone()
            }
        })
        .collect()
}

/// Parse without panicking. Memory is bounded by construction (lists grow as tokens are read and
/// declared counts are only compared), which the huge-count tests exercise: a parser that sized
/// a buffer from `4000000000` would abort the test process.
fn parse_checked(files: &[FoamFile]) -> sm_meshio::Result<PolyMesh> {
    parse_polymesh(files)
}

fn kind(e: &MeshioError) -> &'static str {
    match e {
        MeshioError::Foam(FoamError::Parse { .. }) => "parse",
        MeshioError::Foam(FoamError::Unsupported(_)) => "unsupported",
        MeshioError::Foam(FoamError::Malformed(_)) => "malformed",
        _ => "other",
    }
}

// ---- 7. hostile parse --------------------------------------------------------------------------------

#[test]
fn truncated_files_are_errors() {
    let g = good();
    for f in &g {
        // Cut before the last closing bracket: always an error, never a panic.
        let last = f.bytes.iter().rposition(|b| *b == b')' || *b == b'}').unwrap();
        for cut in (0..=last).step_by(1) {
            let t: Vec<FoamFile> = g.iter().map(|x| if x.name == f.name { FoamFile { name: x.name.clone(), bytes: x.bytes[..cut].to_vec() } } else { x.clone() }).collect();
            let r = parse_checked(&t);
            assert!(r.is_err(), "{} cut at {cut} parsed", f.name);
        }
    }
    // A missing file is an error too.
    let mut g2 = good();
    g2.retain(|f| f.name != "neighbour");
    assert_eq!(kind(&parse_polymesh(&g2).unwrap_err()), "malformed");
}

#[test]
fn binary_format_is_unsupported() {
    for name in ["points", "faces", "owner", "neighbour", "boundary"] {
        let r = parse_polymesh(&replace(&good(), name, "format      ascii;", "format      binary;"));
        assert_eq!(kind(&r.unwrap_err()), "unsupported", "{name}");
    }
    let r = parse_polymesh(&replace(&good(), "faces", "class       faceList;", "class       faceCompactList;"));
    assert_eq!(kind(&r.unwrap_err()), "unsupported");
}

#[test]
fn negative_and_huge_counts_are_errors() {
    let g = good();
    for (name, from) in [("points", "8\n("), ("faces", "6\n("), ("owner", "6\n(")] {
        for count in ["-8", "-1", "99999999999", "4294967296", "18446744073709551615", "184467440737095516160", "1e9", "0x10"] {
            let t = replace(&g, name, from, &format!("{count}\n("));
            let r = parse_checked(&t);
            assert!(matches!(r, Err(MeshioError::Foam(_))), "{name} count {count}: {r:?}");
        }
    }
    // Uniform lists may not expand past the input size.
    let t = replace(&g, "owner", "6\n(\n0\n0\n0\n0\n0\n0\n)", "4000000000{0}");
    assert_eq!(kind(&parse_checked(&t).unwrap_err()), "malformed");
    // ... but a small one is fine: owner 6{0} is the same mesh as six zeros.
    let t = replace(&g, "owner", "6\n(\n0\n0\n0\n0\n0\n0\n)", "6{0}");
    assert_eq!(parse_checked(&t).unwrap(), parse_polymesh(&g).unwrap());
    // A count smaller or larger than the entries.
    let t = replace(&g, "owner", "6\n(", "7\n(");
    assert_eq!(kind(&parse_checked(&t).unwrap_err()), "malformed");
    let t = replace(&g, "owner", "6\n(", "5\n(");
    assert_eq!(kind(&parse_checked(&t).unwrap_err()), "malformed");
}

#[test]
fn indices_out_of_range_are_errors() {
    let g = good();
    let t = replace(&g, "faces", "4(0 1 5 4)", "4(0 1 5 8)");
    assert_eq!(kind(&parse_polymesh(&t).unwrap_err()), "malformed");
    let t = replace(&g, "faces", "4(0 1 5 4)", "4(0 1 5 4294967295)");
    assert_eq!(kind(&parse_polymesh(&t).unwrap_err()), "malformed");
    let t = replace(&g, "faces", "4(0 1 5 4)", "4(0 1 5 -1)");
    assert_eq!(kind(&parse_polymesh(&t).unwrap_err()), "malformed");
    let t = replace(&g, "faces", "4(0 1 5 4)", "4(0 1 5 99999999999)");
    assert_eq!(kind(&parse_polymesh(&t).unwrap_err()), "malformed");
    // A face that declares more points than it lists, or fewer than three.
    let t = replace(&g, "faces", "4(0 1 5 4)", "5(0 1 5 4)");
    assert_eq!(kind(&parse_polymesh(&t).unwrap_err()), "malformed");
    let t = replace(&g, "faces", "4(0 1 5 4)", "2(0 1)");
    assert_eq!(kind(&parse_polymesh(&t).unwrap_err()), "malformed");
    // Owner and face counts that disagree.
    let t = replace(&g, "owner", "6\n(\n0\n", "5\n(\n");
    assert!(parse_polymesh(&t).is_err());
    // A patch that runs past the faces.
    let t = replace(&g, "boundary", "nFaces          6;", "nFaces          7;");
    assert_eq!(kind(&parse_polymesh(&t).unwrap_err()), "malformed");
    // Cyclic patches are unsupported, not misread.
    let t = replace(&g, "boundary", "type            patch;", "type            cyclic;");
    assert_eq!(kind(&parse_polymesh(&t).unwrap_err()), "unsupported");
    // Non-finite coordinates.
    for v in ["nan", "inf", "-inf"] {
        let t = replace(&g, "points", "(0.0 0.0 0.0)", &format!("({v} 0.0 0.0)"));
        assert_eq!(kind(&parse_polymesh(&t).unwrap_err()), "malformed", "{v}");
    }
    let t = replace(&g, "points", "(0.0 0.0 0.0)", "(0.0 zero 0.0)");
    assert_eq!(kind(&parse_polymesh(&t).unwrap_err()), "parse");
}

#[test]
fn deep_nesting_and_junk_do_not_recurse_or_panic() {
    let g = good();
    let deep = format!("{}{}", "(".repeat(200_000), ")".repeat(200_000));
    for name in ["points", "faces", "owner", "neighbour", "boundary"] {
        let t: Vec<FoamFile> = g.iter().map(|f| if f.name == name { FoamFile { name: f.name.clone(), bytes: format!("{}{deep}", String::from_utf8_lossy(&f.bytes)).into_bytes() } } else { f.clone() }).collect();
        let _ = parse_checked(&t);
    }
    // Header values with brackets, unterminated strings and comments, stray bytes.
    for junk in ["FoamFile { a (( ; }", "\"unterminated", "/* never closed", "\u{0}\u{1}\u{2}", "\\", "FoamFile", "FoamFile {", "{{{{", ";;;;", "// only a comment"] {
        let t: Vec<FoamFile> = g.iter().map(|f| FoamFile { name: f.name.clone(), bytes: junk.as_bytes().to_vec() }).collect();
        assert!(parse_checked(&t).is_err(), "{junk}");
    }
    // Non UTF-8 bytes inside a token.
    let mut t = g.clone();
    t[0].bytes.splice(0..0, [0xff, 0xfe, b'x']);
    assert!(parse_checked(&t).is_err());
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 400, ..ProptestConfig::default() })]

    #[test]
    fn random_bytes_never_panic(data in proptest::collection::vec(any::<u8>(), 0..2048), which in 0usize..5) {
        let mut files = good();
        files[which].bytes = data;
        let _ = parse_checked(&files);
    }

    #[test]
    fn mutated_valid_files_never_panic(edits in proptest::collection::vec((any::<u16>(), any::<u8>(), 0usize..3), 1..8), which in 0usize..7) {
        let m = sets_block();
        let mut files = build_polymesh(&m, &FoamOptions::default()).unwrap().mesh.files().unwrap();
        let n = files.len();
        let f = &mut files[which % n];
        for (pos, byte, op) in edits {
            if f.bytes.is_empty() { break; }
            let i = pos as usize % f.bytes.len();
            match op {
                0 => f.bytes[i] = byte,
                1 => { f.bytes.remove(i); }
                _ => f.bytes.insert(i, byte),
            }
        }
        let _ = parse_checked(&files);
    }

    #[test]
    fn printable_garbage_in_every_slot(words in proptest::collection::vec("[0-9a-zA-Z(){};\"/*<>. +-]{0,12}", 0..60), which in 0usize..5) {
        let mut files = good();
        let header_end = files[which].bytes.windows(2).position(|w| w == b"}\n").unwrap() + 2;
        let mut bytes = files[which].bytes[..header_end].to_vec();
        bytes.extend_from_slice(words.join(" ").as_bytes());
        files[which].bytes = bytes;
        let _ = parse_checked(&files);
    }
}

// ---- spellings -----------------------------------------------------------------------------------------

#[test]
fn accepts_comments_and_the_list_spellings() {
    let hdr = |class: &str, obj: &str| format!("/* banner */\nFoamFile\n{{\n  version 2.0; // c\n  format ascii;\n  class {class};\n  object {obj};\n}}\n");
    let f = |name: &str, class: &str, body: &str| FoamFile { name: name.into(), bytes: format!("{}{body}\n// ***\n", hdr(class, name)).into_bytes() };
    // One tetrahedron with every spelling of the list syntax.
    let files = vec![
        f("points", "vectorField", "4 ((0 0 0) (1 0 0) (0 1 0) (0 0 1))"),
        f("faces", "faceList", "List<face> 4\n(\n3(0 2 1)\n3 (0 1 3)\n3(1 2 3)\n3(2 0 3)\n)"),
        f("owner", "labelList", "List<label> 4(0 0 0 0)"),
        f("neighbour", "labelList", "0()"),
        f("boundary", "polyBoundaryMesh", "1\n(\n    \"walls\"\n    {\n        type wall;\n        inGroups List<word> 1(wall);\n        nFaces 4;\n        startFace 0;\n    }\n)"),
    ];
    let m = parse_polymesh(&files).unwrap();
    assert_eq!((m.points.len(), m.n_faces(), m.n_internal(), m.n_cells), (4, 4, 0, 1));
    assert_eq!(m.face(1), [0, 1, 3]);
    assert_eq!(m.patches[0].name, "walls");
    assert_eq!(m.patches[0].patch_type, FoamPatchType::Wall);
    assert!(check_polymesh(&m, 1e-9).is_empty(), "{:?}", check_polymesh(&m, 1e-9));
    // A multi-line block list with the trailing semicolon style of OpenFOAM zone files.
    let mut files = files;
    files.push(f(
        "cellZones",
        "regIOobject",
        "1\n(\nz\n{\n    type cellZone;\n    cellLabels      List<label> \n1\n(\n0\n)\n;\n}\n)",
    ));
    files.push(f(
        "faceZones",
        "regIOobject",
        "1\n(\nfz\n{\n    type faceZone;\n    faceLabels List<label> 2(0 3);\n    flipMap List<bool> 2(true false);\n    cellZoneName none;\n}\n)",
    ));
    let m = parse_polymesh(&files).unwrap();
    assert_eq!((m.cell_zones[0].name.as_str(), m.cell_zones[0].labels.as_slice()), ("z", &[0][..]));
    assert_eq!((m.face_zones[0].labels.as_slice(), m.face_zones[0].flips.as_slice()), (&[0, 3][..], &[true, false][..]));
}

#[test]
fn read_dir_round_trips_and_write_dir_removes_stale_zone_files() {
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("foam_read_dir");
    let _ = std::fs::remove_dir_all(&dir);
    let with_zones = build_polymesh(&sets_block(), &FoamOptions::default()).unwrap();
    with_zones.mesh.write_dir(&dir).unwrap();
    assert!(dir.join("cellZones").is_file());
    assert_eq!(read_polymesh_dir(&dir).unwrap(), with_zones.mesh);
    // Same directory, a mesh without zones: the old cellZones file must go.
    let bare = build_polymesh(&hex_block_2x2x2().unwrap().mesh, &FoamOptions::default()).unwrap();
    bare.mesh.write_dir(&dir).unwrap();
    assert!(!dir.join("cellZones").exists() && !dir.join("faceZones").exists());
    assert_eq!(read_polymesh_dir(&dir).unwrap(), bare.mesh);
    // write_polymesh = build + write_dir, returning the report.
    let r = write_polymesh(&hex_block([2, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap(), &FoamOptions::default(), &dir).unwrap();
    assert_eq!((r.n_cells, r.n_faces, r.n_internal_faces), (2, 11, 1));
    assert_eq!(read_polymesh_dir(&dir).unwrap().n_cells, 2);
    assert!(matches!(read_polymesh_dir(&dir.join("nope")), Err(MeshioError::Io(_))));
}

// ---- 8. tar -----------------------------------------------------------------------------------------------

/// Minimal ustar reader: `(path, bytes)` of each entry, verifying checksums and metadata.
fn untar(b: &[u8]) -> Vec<(String, Vec<u8>)> {
    let oct = |s: &[u8]| u64::from_str_radix(std::str::from_utf8(s).unwrap().trim_matches(|c| c == '\0' || c == ' '), 8).unwrap();
    let text = |s: &[u8]| String::from_utf8(s.iter().copied().take_while(|c| *c != 0).collect()).unwrap();
    let (mut out, mut at) = (vec![], 0);
    while b[at..at + 512].iter().any(|c| *c != 0) {
        let h = &b[at..at + 512];
        let mut sum = 0u32;
        for (i, c) in h.iter().enumerate() {
            sum += if (148..156).contains(&i) { 32 } else { u32::from(*c) };
        }
        assert_eq!(oct(&h[148..156]) as u32, sum, "header checksum");
        assert_eq!(&h[257..263], b"ustar\0");
        assert_eq!((oct(&h[100..108]), oct(&h[108..116]), oct(&h[116..124]), oct(&h[136..148])), (0o644, 0, 0, 0), "mode, uid, gid, mtime");
        assert_eq!(h[156], b'0');
        let size = oct(&h[124..136]) as usize;
        let (prefix, name) = (text(&h[345..500]), text(&h[..100]));
        out.push((if prefix.is_empty() { name } else { format!("{prefix}/{name}") }, b[at + 512..at + 512 + size].to_vec()));
        at += 512 + size.div_ceil(512) * 512;
    }
    assert!(b[at..].iter().all(|c| *c == 0) && b.len() - at == 1024, "two zero end blocks");
    out
}

#[test]
fn tar_holds_the_files_with_valid_headers_and_is_deterministic() {
    let m = build_polymesh(&sets_block(), &FoamOptions::default()).unwrap().mesh;
    let mut a = Vec::new();
    m.write_tar(&mut a, "constant/polyMesh/").unwrap();
    let mut b = Vec::new();
    m.write_tar(&mut b, "constant/polyMesh/").unwrap();
    assert_eq!(a, b, "byte-identical across runs");
    assert_eq!(a.len() % 512, 0);
    let entries = untar(&a);
    let mut want: Vec<(String, Vec<u8>)> = m.files().unwrap().into_iter().map(|f| (format!("constant/polyMesh/{}", f.name), f.bytes)).collect();
    want.sort();
    assert_eq!(entries, want);
    assert_eq!(entries.len(), 6);
    // No prefix, and a long prefix that needs the ustar prefix field.
    let mut c = Vec::new();
    m.write_tar(&mut c, "").unwrap();
    assert_eq!(untar(&c).iter().map(|e| e.0.as_str()).collect::<Vec<_>>(), ["boundary", "cellZones", "faces", "neighbour", "owner", "points"]);
    let long = format!("{}/", "d".repeat(60) + "/" + &"e".repeat(60));
    let mut d = Vec::new();
    m.write_tar(&mut d, &long).unwrap();
    assert!(untar(&d).iter().all(|e| e.0.starts_with(&long)));
    let too_long = "x".repeat(300) + "/";
    assert!(matches!(m.write_tar(&mut Vec::new(), &too_long), Err(MeshioError::Unsupported(_))));
    // The system tar agrees when it exists.
    if std::process::Command::new("tar").arg("--version").output().is_ok_and(|o| o.status.success()) {
        let p = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("foam.tar");
        std::fs::write(&p, &a).unwrap();
        let o = std::process::Command::new("tar").arg("-tvf").arg(&p).output().unwrap();
        assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
        let l = String::from_utf8_lossy(&o.stdout);
        assert_eq!(l.lines().count(), 6, "{l}");
        assert!(l.lines().all(|x| x.starts_with("-rw-r--r--")) && l.contains("constant/polyMesh/points"), "{l}");
    }
}
