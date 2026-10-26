//! Acceptance 11 (and the `line:col` half of 1): every malformed fixture gives its designated
//! `StepError`, with the offending STEP id or position in the message, and never panics.

mod common;

use common::*;
use sm_cadio::{ReadOptions, StepError, read_step};

fn err(name: &str) -> StepError {
    read_step(&bad(name), &ReadOptions::default()).expect_err(name)
}

#[test]
fn not_a_step_file() {
    let e = err("not_step.txt");
    assert!(matches!(e, StepError::Syntax { line: 1, col: 1, .. }), "{e:?}");
    assert!(e.to_string().contains("1:1"));
}

#[test]
fn truncated_file_without_endsec() {
    let e = err("truncated_no_endsec.stp");
    let StepError::Truncated { line, col, .. } = e else { panic!("{e:?}") };
    let text = String::from_utf8(bad("truncated_no_endsec.stp")).expect("utf8");
    let (want_line, want_col) = sm_cadio::part21::line_col(text.as_bytes(), text.len());
    assert_eq!((line, col), (want_line, want_col), "points at the end of the input");
}

#[test]
fn unterminated_string_points_at_its_opening_quote() {
    let e = err("unterminated_string.stp");
    let text = String::from_utf8(bad("unterminated_string.stp")).expect("utf8");
    let at = text.find("'core data").expect("quote");
    let (line, col) = sm_cadio::part21::line_col(text.as_bytes(), at);
    let StepError::Syntax { line: l, col: c, msg } = &e else { panic!("{e:?}") };
    assert_eq!((*l, *c), (line, col));
    assert!(msg.contains("unterminated string"), "{msg}");
}

#[test]
fn dangling_reference_names_both_entities() {
    let e = err("dangling_ref.stp");
    let StepError::MissingEntity { id, referenced_by } = &e else { panic!("{e:?}") };
    let msg = e.to_string();
    assert!(msg.contains(&format!("#{id}")) && msg.contains(&format!("#{referenced_by}")), "{msg}");
    let text = String::from_utf8(bad("dangling_ref.stp")).expect("utf8");
    assert!(!text.contains(&format!("#{id} =")), "the referenced entity really is absent");
    assert!(text.contains(&format!("#{referenced_by} = VERTEX_POINT")), "the referrer is a vertex");
}

#[test]
fn cyclic_reference() {
    let e = err("cyclic_ref.stp");
    let StepError::Cycle { id } = &e else { panic!("{e:?}") };
    let text = String::from_utf8(bad("cyclic_ref.stp")).expect("utf8");
    assert!(text.contains(&format!("#{id} = SURFACE_CURVE")), "{e}");
    assert!(e.to_string().contains(&format!("#{id}")));
}

#[test]
fn wrong_parameter_type() {
    let e = err("wrong_param_type.stp");
    let StepError::BadParam { id, entity, index, .. } = &e else { panic!("{e:?}") };
    assert_eq!((entity.as_str(), *index), ("ADVANCED_FACE", 3));
    assert!(e.to_string().contains(&format!("#{id}")));
}

#[test]
fn unsupported_geometry_is_named() {
    let e = err("unsupported_offset_surface.stp");
    let StepError::Unsupported { id, entity } = &e else { panic!("{e:?}") };
    assert_eq!(entity, "OFFSET_SURFACE");
    assert!(e.to_string().contains(&format!("#{id}")));
}

#[test]
fn zero_radius_cylinder() {
    let e = err("zero_radius_cylinder.stp");
    let StepError::BadParam { id, entity, index, msg } = &e else { panic!("{e:?}") };
    assert_eq!((entity.as_str(), *index), ("CYLINDRICAL_SURFACE", 2));
    assert!(msg.contains("radius"), "{msg}");
    assert!(e.to_string().contains(&format!("#{id}")));
}

#[test]
fn file_without_units() {
    let e = err("no_units.stp");
    assert!(matches!(e, StepError::Units(_)), "{e:?}");
    // The op-level default (assume_unit) reads it.
    let opts = ReadOptions { assume_unit: Some(sm_cadio::LengthUnit::Mm), ..ReadOptions::default() };
    let m = read_step(&bad("no_units.stp"), &opts).expect("with assume_unit");
    assert!(!m.solids[0].units.declared);
}

#[test]
fn open_shell_names_the_open_edges() {
    let e = err("open_shell.stp");
    let StepError::OpenShell { solid, issues } = &e else { panic!("{e:?}") };
    assert_eq!(issues.len(), 4, "the removed +z face leaves its four edges open: {issues:?}");
    assert!(issues.iter().all(|i| i.net_uses.abs() == 1));
    assert!(issues.windows(2).all(|w| w[0].edge < w[1].edge));
    let msg = e.to_string();
    assert!(msg.contains(&format!("#{solid}")) && msg.contains(&format!("edge #{}", issues[0].edge)), "{msg}");
}

#[test]
fn deep_nesting_is_too_deep_not_a_stack_overflow() {
    let mut s = String::from("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1 = A(");
    s.push_str(&"(".repeat(100_000));
    s.push_str(");\nENDSEC;\nEND-ISO-10303-21;\n");
    let e = read_step(s.as_bytes(), &ReadOptions::default()).expect_err("too deep");
    let StepError::TooDeep { line, col, max } = e else { panic!("{e:?}") };
    assert_eq!((line, max), (5, 256));
    assert!(col > 1);
}

#[test]
fn every_error_maps_to_an_invalid_op_error_with_a_step_code() {
    for (name, code) in [
        ("not_step.txt", "step.syntax"),
        ("truncated_no_endsec.stp", "step.truncated"),
        ("dangling_ref.stp", "step.missing_entity"),
        ("cyclic_ref.stp", "step.cycle"),
        ("wrong_param_type.stp", "step.bad_param"),
        ("unsupported_offset_surface.stp", "step.unsupported"),
        ("no_units.stp", "step.units"),
        ("open_shell.stp", "step.open_shell"),
    ] {
        let e = err(name);
        assert_eq!(e.code(), code, "{name}");
        let op: sm_op_api::OpError = e.into();
        let sm_op_api::OpError::Invalid(msg) = op else { panic!("{name}: not Invalid") };
        assert!(msg.contains(code), "{name}: {msg}");
    }
}
