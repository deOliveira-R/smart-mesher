//! Many solid records over one shell must neither multiply the decode work nor the memory:
//! the shell is decoded once and the copies are charged to one file-wide budget.

use sm_cadio::{ReadOptions, StepError, read_step};

/// `fixture` with `n` extra `MANIFOLD_SOLID_BREP`s over the shell of its first solid.
fn fan(fixture: &[u8], n: usize) -> Vec<u8> {
    let text = String::from_utf8(fixture.to_vec()).expect("utf8");
    let at = text.find("MANIFOLD_SOLID_BREP(").expect("a solid");
    let rest = &text[at..];
    let hash = rest.find('#').expect("shell ref");
    let digits: String = rest[hash + 1..].chars().take_while(char::is_ascii_digit).collect();
    let end = text.rfind("ENDSEC;").expect("endsec");
    let mut s = text[..end].to_owned();
    for k in 0..n {
        s += &format!("#{}=MANIFOLD_SOLID_BREP('s{k}',#{digits});\n", 900_000 + k);
    }
    s += &text[end..];
    s.into_bytes()
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
fn a_shared_shell_fan_is_refused_with_a_typed_error_and_quickly() {
    for fixture in [sm_testkit::step::BOX_FILLET1, sm_testkit::step::LOFT_BSPLINE] {
        let bytes = fan(fixture.bytes, 4000);
        let t = std::time::Instant::now();
        let e = read_step(&bytes, &ReadOptions::default()).expect_err(fixture.name);
        assert!(matches!(e, StepError::TooManyInstances { .. }), "{}: {e:?}", fixture.name);
        assert!(t.elapsed().as_secs() < 10, "{}: {:?}", fixture.name, t.elapsed());
    }
}
