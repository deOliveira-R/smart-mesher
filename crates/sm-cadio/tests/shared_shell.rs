//! Many solid records over the same faces must neither multiply the decode work nor the memory:
//! every decoded solid, the first of a shared body included, is charged to one file-wide budget
//! that is checked from the entity graph before any geometry is decoded.

use sm_cadio::{NoteCode, NoteSeverity, ReadOptions, StepError, read_step};

/// The text of `fixture` split before its `ENDSEC;`, with the first solid's shell id and the
/// face list of that shell (`#a,#b,...`).
fn parts(fixture: &[u8]) -> (String, String, u64, String) {
    let text = String::from_utf8(fixture.to_vec()).expect("utf8");
    let at = text.find("MANIFOLD_SOLID_BREP(").expect("a solid");
    let rest = &text[at..];
    let hash = rest.find('#').expect("shell ref");
    let shell: u64 = rest[hash + 1..].chars().take_while(char::is_ascii_digit).collect::<String>().parse().expect("id");
    let rec = format!("#{shell} = CLOSED_SHELL('',(");
    let s0 = text.find(&rec).expect("the shell record") + rec.len();
    let faces = text[s0..s0 + text[s0..].find(')').expect("face list")].to_owned();
    let end = text.rfind("ENDSEC;").expect("endsec");
    (text[..end].to_owned(), text[end..].to_owned(), shell, faces)
}

/// `fixture` with `n` extra `MANIFOLD_SOLID_BREP`s over the shell of its first solid.
fn fan(fixture: &[u8], n: usize) -> Vec<u8> {
    let (mut s, tail, shell, _) = parts(fixture);
    for k in 0..n {
        s += &format!("#{}=MANIFOLD_SOLID_BREP('s{k}',#{shell});\n", 900_000 + k);
    }
    (s + &tail).into_bytes()
}

/// `fixture` with `n` extra solids, each over its own `CLOSED_SHELL` listing the faces of the
/// first solid's shell (about 100 bytes per copy).
fn shell_fan(fixture: &[u8], n: usize) -> Vec<u8> {
    let (mut s, tail, _, faces) = parts(fixture);
    for k in 0..n {
        let (a, b) = (1_000_000 + 2 * k, 1_000_001 + 2 * k);
        s += &format!("#{a}=CLOSED_SHELL('',({faces}));\n#{b}=MANIFOLD_SOLID_BREP('',#{a});\n");
    }
    (s + &tail).into_bytes()
}

#[test]
fn a_few_solids_over_one_shell_are_decoded_once_and_cloned() {
    let one = read_step(sm_testkit::step::BOX_FILLET1.bytes, &ReadOptions::default()).expect("one");
    let m = read_step(&fan(sm_testkit::step::BOX_FILLET1.bytes, 3), &ReadOptions::default()).expect("fan of 3");
    assert_eq!(m.solids.len(), 4);
    assert_eq!(m.solids[0], one.solids[0]);
    for (k, s) in m.solids[1..].iter().enumerate() {
        assert_eq!(s.id, 900_000 + k as u64);
        assert_eq!(s.name, format!("s{k}"));
        assert_eq!(s.faces, one.solids[0].faces);
        assert_eq!(s.edges, one.solids[0].edges);
    }
}

#[test]
fn a_few_distinct_shells_over_the_same_faces_still_read() {
    let one = read_step(sm_testkit::step::BOX_FILLET1.bytes, &ReadOptions::default()).expect("one");
    let m = read_step(&shell_fan(sm_testkit::step::BOX_FILLET1.bytes, 3), &ReadOptions::default()).expect("3 shells");
    assert_eq!(m.solids.len(), 4);
    for s in &m.solids[1..] {
        assert_eq!(s.faces, one.solids[0].faces);
    }
}

fn assert_refused_fast(bytes: &[u8], name: &str) {
    let t = std::time::Instant::now();
    let e = read_step(bytes, &ReadOptions::default()).expect_err(name);
    let dt = t.elapsed();
    eprintln!("{name}: {} bytes refused in {dt:?}: {e}", bytes.len());
    assert!(matches!(e, StepError::DecodeBudget { .. }), "{name}: {e:?}");
    assert_eq!(e.code(), "step.decode_budget");
    let msg = e.to_string();
    assert!(msg.contains("copies") && !msg.contains("assembly flattening"), "{msg}");
    // Refused from the entity graph before decoding: far below this even in a debug build.
    assert!(dt.as_secs() < 5, "{name}: {dt:?}");
}

#[test]
fn a_shared_shell_fan_is_refused_with_a_typed_error_and_quickly() {
    for fixture in [sm_testkit::step::BOX_FILLET1, sm_testkit::step::LOFT_BSPLINE] {
        assert_refused_fast(&fan(fixture.bytes, 4000), fixture.name);
    }
}

#[test]
fn box_fillet_with_4000_distinct_shells_over_its_faces_is_refused_before_decoding() {
    assert_refused_fast(&shell_fan(sm_testkit::step::BOX_FILLET1.bytes, 4000), "box_fillet1 + 4000 shells");
}

#[test]
fn loft_with_4000_distinct_shells_over_its_faces_is_refused_before_decoding() {
    assert_refused_fast(&shell_fan(sm_testkit::step::LOFT_BSPLINE.bytes, 4000), "loft_bspline + 4000 shells");
}

#[test]
fn a_closed_nurbs_face_that_cannot_be_split_is_kept_with_a_warning() {
    // The loft's lateral face also runs its bottom circle (#21) forward and back: still closed,
    // but no longer the plain seam configuration the splitter handles.
    let text = String::from_utf8(sm_testkit::step::LOFT_BSPLINE.bytes.to_vec()).expect("utf8");
    let lp = "#19 = EDGE_LOOP('',(#20,#78,#100,#127));";
    assert!(text.contains(lp));
    let text = text.replace(lp, "#19 = EDGE_LOOP('',(#20,#78,#100,#127,#900001,#900002));\n#900001 = ORIENTED_EDGE('',*,*,#21,.T.);\n#900002 = ORIENTED_EDGE('',*,*,#21,.F.);");
    let m = read_step(text.as_bytes(), &ReadOptions::default()).expect("read with a warning");
    assert!(m.notes.iter().any(|n| n.code == NoteCode::SplitClosedFace && n.severity == NoteSeverity::Warning && n.message.contains("left as is")), "{:?}", m.notes);
    assert_eq!(m.solids[0].faces.len(), 3, "the face is not split");
}
