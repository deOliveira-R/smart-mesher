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

/// A uniform list of a big element multiplies the input: `20000{10000(0 ... 0)}` is 20 KB of text
/// but 800 MB of labels if expanded. Faces, points and the generic bound all refuse it.
#[test]
fn uniform_lists_of_big_elements_are_refused() {
    let g = good();
    let face = format!("10000({})", vec!["0"; 10_000].join(" "));
    let big_faces = replace(&g, "faces", "6\n(", &format!("20000{{{face}}}\n("));
    assert!(parse_checked(&big_faces).is_err());
    // A whole-file version of the same shape: the uniform form replacing the list entirely.
    let f = g.iter().find(|f| f.name == "faces").unwrap();
    let text = String::from_utf8(f.bytes.clone()).unwrap();
    let head = &text[..text.find("6\n(").unwrap()];
    let hostile = FoamFile { name: "faces".into(), bytes: format!("{head}20000{{{face}}}\n").into_bytes() };
    assert!(hostile.bytes.len() < 30_000);
    let files: Vec<FoamFile> = g.iter().map(|x| if x.name == "faces" { hostile.clone() } else { x.clone() }).collect();
    assert!(matches!(parse_checked(&files), Err(MeshioError::Foam(_))));
    // Points: 3 scalars per 8+ bytes, so a count near the input length is bounded by size too.
    let p = g.iter().find(|f| f.name == "points").unwrap();
    let text = String::from_utf8(p.bytes.clone()).unwrap();
    let head = &text[..text.find("8\n(").unwrap()];
    let n = head.len() / 2; // passes the plain count check, fails n * element size
    let files: Vec<FoamFile> = g.iter().map(|x| if x.name == "points" { FoamFile { name: "points".into(), bytes: format!("{head}{n}{{(0 0 0)}}\n").into_bytes() } } else { x.clone() }).collect();
    assert!(parse_checked(&files).is_err());
    // A small uniform point list still parses (and then fails the face index check, not the lexer).
    let files: Vec<FoamFile> = g.iter().map(|x| if x.name == "points" { FoamFile { name: "points".into(), bytes: format!("{head}8{{(0 0 0)}}\n").into_bytes() } } else { x.clone() }).collect();
    assert!(parse_checked(&files).is_ok());
}

fn with_file(files: &[FoamFile], name: &str, body: &str) -> Vec<FoamFile> {
    let head = String::from_utf8(files.iter().find(|f| f.name == "owner").unwrap().bytes.clone()).unwrap();
    let head = &head[..head.find("FoamFile").unwrap()];
    let mut v: Vec<FoamFile> = files.iter().filter(|f| f.name != name).cloned().collect();
    let text = format!("{head}FoamFile {{ version 2.0; format ascii; class regIOobject; object {name}; }}\n{body}\n");
    v.push(FoamFile { name: name.into(), bytes: text.into_bytes() });
    v
}

/// One uniform list is bounded by the file length, but a zone file holds one list per zone: the
/// total expansion must stay bounded too (it used to be zones x file length).
#[test]
fn many_uniform_zone_lists_share_one_budget() {
    let g = good();
    let zone = |i: usize| format!("z{i} {{ type cellZone; cellLabels List<label> 100{{0}}; }}");
    let few = |i: usize| format!("z{i} {{ type cellZone; cellLabels List<label> 5{{0}}; }}");
    let body = |n: usize| format!("{n}\n(\n{}\n)", (0..n).map(zone).collect::<Vec<_>>().join("\n"));
    // A few small uniform lists fit; 4000 zones x 100 labels (400k labels) in a ~230 KB file do not.
    let small = format!("10\n(\n{}\n)", (0..10).map(few).collect::<Vec<_>>().join("\n"));
    assert!(parse_checked(&with_file(&g, "cellZones", &small)).is_ok());
    let big = with_file(&g, "cellZones", &body(4000));
    let len = big.iter().find(|f| f.name == "cellZones").unwrap().bytes.len();
    assert!(4000 * 100 > len);
    assert_eq!(kind(&parse_checked(&big).unwrap_err()), "malformed");
    // faceZones: labels and flipMap draw on the same budget.
    let fz = |i: usize| format!("f{i} {{ type faceZone; faceLabels List<label> 100{{0}}; flipMap List<bool> 100{{0}}; }}");
    let body = format!("4000\n(\n{}\n)", (0..4000).map(fz).collect::<Vec<_>>().join("\n"));
    assert_eq!(kind(&parse_checked(&with_file(&g, "faceZones", &body)).unwrap_err()), "malformed");
    // flipMap N{0} on a legitimate zone stays accepted.
    let body = "1\n(\nf0 { type faceZone; faceLabels List<label> 2(0 1); flipMap List<bool> 2{0}; }\n)";
    assert!(parse_checked(&with_file(&g, "faceZones", body)).is_ok());
}

/// A tiny owner file with one huge label must not yield a mesh claiming billions of cells.
#[test]
fn huge_cell_label_is_refused() {
    let g = good();
    let t = String::from_utf8(g.iter().find(|f| f.name == "owner").unwrap().bytes.clone()).unwrap();
    let first = t.find("(\n").unwrap() + 2;
    let hostile = format!("{}4000000000{}", &t[..first], &t[first + t[first..].find('\n').unwrap()..]);
    let files = replace(&g, "owner", &t, &hostile);
    assert_eq!(kind(&parse_checked(&files).unwrap_err()), "malformed");
    // check_polymesh survives a hand-built PolyMesh with the same lie.
    let mut m = parse_checked(&g).unwrap();
    m.n_cells = u32::MAX;
    assert!(!check_polymesh(&m, 1e-9).is_empty());
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
    #![proptest_config(sm_testkit::strategies::config(400))]

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

// ---- allocation bound under an address-space limit ----------------------------------------------------

/// The parses that would allocate gigabytes if a declared count sized a buffer. Runs only inside
/// the child process of [`huge_counts_do_not_allocate_under_an_address_space_limit`].
#[test]
fn inner_huge_counts() {
    if std::env::var_os("SM_MESHIO_INNER").is_none() {
        return;
    }
    let g = good();
    for (name, from) in [("points", "8\n("), ("faces", "6\n("), ("owner", "6\n("), ("neighbour", "0\n("), ("boundary", "1\n(")] {
        for count in ["4000000000", "2147483647", "9999999999", "4294967295"] {
            let t = replace(&g, name, from, &format!("{count}\n("));
            assert!(parse_polymesh(&t).is_err(), "{name} {count}");
        }
    }
    let t = replace(&g, "owner", "6\n(\n0\n0\n0\n0\n0\n0\n)", "4000000000{0}");
    assert!(parse_polymesh(&t).is_err());
    // Uniform lists of big elements (the quadratic-expansion shape).
    let face = format!("10000({})", vec!["0"; 10_000].join(" "));
    let t = replace(&g, "faces", "6\n(", &format!("20000{{{face}}}\n("));
    assert!(parse_polymesh(&t).is_err());
    let t = replace(&g, "faces", "4(0 1 5 4)", "4000000000(0 1 5 4)");
    assert!(parse_polymesh(&t).is_err());
    let t = replace(&g, "boundary", "nFaces          6;", "nFaces          4294967295;");
    assert!(parse_polymesh(&t).is_err());
    // A file of 2 MB of data parses within the limit too (memory proportional to the input).
    let big = build_polymesh(&hex_block([30, 30, 30], [1.0; 3], &mut SerialAlloc::new()).unwrap(), &FoamOptions::default()).unwrap().mesh;
    assert_eq!(parse_polymesh(&big.files().unwrap()).unwrap(), big);
}

/// Child-process half of the enforcement probe: a 4 GiB reservation must fail under the limit.
#[test]
fn inner_limit_is_enforced() {
    if std::env::var_os("SM_MESHIO_INNER").is_none() {
        return;
    }
    let mut v: Vec<u8> = Vec::new();
    assert!(v.try_reserve_exact(4usize << 30).is_err(), "ulimit -v did not limit the address space");
}

/// Re-run `inner_huge_counts` in a child process whose address space is capped at 1 GiB. A parser
/// that allocated from the declared counts (tens of GB) cannot pass; one that grows with the
/// tokens read stays far below. Skipped without `sh`.
#[test]
fn huge_counts_do_not_allocate_under_an_address_space_limit() {
    if std::env::var_os("SM_MESHIO_INNER").is_some() {
        return;
    }
    if !std::process::Command::new("sh").arg("-c").arg("ulimit -v 1048576").status().is_ok_and(|s| s.success()) {
        // Linux only: macOS refuses `ulimit -v`, so this test gives no evidence there.
        eprintln!("cannot set an address space limit (Linux only): skipping");
        return;
    }
    // The limit must actually bite: a 4 GiB reservation fails inside the child.
    let exe0 = std::env::current_exe().unwrap();
    let probe = std::process::Command::new("sh")
        .arg("-c")
        .arg("ulimit -v 1048576 && exec \"$0\" --exact inner_limit_is_enforced --test-threads=1")
        .arg(&exe0)
        .env("SM_MESHIO_INNER", "1")
        .output()
        .unwrap();
    let ptext = format!("{}{}", String::from_utf8_lossy(&probe.stdout), String::from_utf8_lossy(&probe.stderr));
    assert!(probe.status.success() && ptext.contains("1 passed"), "address-space limit not enforced: {ptext}");
    let exe = std::env::current_exe().unwrap();
    let out = std::process::Command::new("sh")
        .arg("-c")
        .arg("ulimit -v 1048576 && exec \"$0\" --exact inner_huge_counts --test-threads=1")
        .arg(&exe)
        .env("SM_MESHIO_INNER", "1")
        .output()
        .unwrap();
    let text = format!("{}{}", String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
    assert!(out.status.success() && text.contains("1 passed"), "{text}");
}
