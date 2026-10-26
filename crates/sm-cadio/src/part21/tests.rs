use super::*;

fn wrap(data: &str) -> Vec<u8> {
    format!("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('d'),'2;1');\nFILE_NAME('n','t',('a'),('o'),'p','s','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n").into_bytes()
}

fn first(data: &str) -> Entity {
    parse(&wrap(data)).expect("parse").entities.into_values().next().expect("one entity")
}

fn params(data: &str) -> Vec<Param> {
    match first(data) {
        Entity::Simple(r) => r.params,
        Entity::Complex(_) => panic!("complex"),
    }
}

#[test]
fn header_records_are_read() {
    let f = parse(&wrap("")).expect("parse");
    assert_eq!(f.header.description, ["d"]);
    assert_eq!(f.header.implementation_level, "2;1");
    assert_eq!(f.header.name, "n");
    assert_eq!(f.header.schemas, ["AUTOMOTIVE_DESIGN"]);
    assert!(f.entities.is_empty());
}

#[test]
fn comments_are_skipped_everywhere() {
    let p = params("/* c */ #1 /* c */ = /* c\nmulti */ A /* c */ ( /* c */ 1 /* c */ , /* c */ 2 /* c */ ) /* c */ ; /* c */");
    assert_eq!(p, [Param::Int(1), Param::Int(2)]);
}

#[test]
fn records_span_lines_and_forward_references_parse() {
    let f = parse(&wrap("#1 = A(\n  #2,\n  #3\n);\n#2 = B($);\n#3 = C(*);")).expect("parse");
    assert_eq!(f.entities.keys().copied().collect::<Vec<_>>(), [1, 2, 3]);
    let Entity::Simple(a) = &f.entities[&1] else { panic!() };
    assert_eq!(a.params, [Param::Ref(2), Param::Ref(3)]);
    assert_eq!(f.entities[&3].first().map(|r| r.params.clone()), Some(vec![Param::Derived]));
}

#[test]
fn doubled_quotes_and_escapes_decode() {
    assert_eq!(params("#1 = A('it''s');"), [Param::Str("it's".into())]);
    assert_eq!(params("#1 = A('\\X2\\00E9\\X0\\');"), [Param::Str("\u{e9}".into())]);
    assert_eq!(params("#1 = A('\\X2\\D83DDE00\\X0\\');"), [Param::Str("\u{1f600}".into())]);
    assert_eq!(params("#1 = A('\\X4\\0001F600\\X0\\');"), [Param::Str("\u{1f600}".into())]);
    assert_eq!(params("#1 = A('\\X\\E9');"), [Param::Str("\u{e9}".into())]);
    assert_eq!(params("#1 = A('a\\\\b');"), [Param::Str("a\\b".into())]);
    // \S\ is left raw.
    assert_eq!(params("#1 = A('\\S\\i');"), [Param::Str("\\S\\i".into())]);
    // A path ending in a backslash does not swallow the closing quote.
    assert_eq!(params("#1 = A('C:\\x\\');"), [Param::Str("C:\\x\\".into())]);
}

#[test]
fn reals_ints_and_enums() {
    let p = params("#1 = A(1., -.5, 1.E-3, 0.5E+2, 7, -7, +7, 1.0e3, .T., .MILLI., $, *);");
    let want = |s: &str| Param::Real(s.parse::<f64>().expect("rust parses it"));
    assert_eq!(p[0], want("1."));
    assert_eq!(p[1], want("-.5"));
    assert_eq!(p[2], want("1.E-3"));
    assert_eq!(p[3], want("0.5E+2"));
    assert_eq!(p[4], Param::Int(7));
    assert_eq!(p[5], Param::Int(-7));
    assert_eq!(p[6], Param::Int(7));
    assert_eq!(p[7], want("1.0e3"));
    assert_eq!(p[8], Param::Enum("T".into()));
    assert_eq!(p[9], Param::Enum("MILLI".into()));
    assert_eq!(p[10], Param::Null);
    assert_eq!(p[11], Param::Derived);
    // Bit-equality with the standard parser.
    let (Param::Real(a), Some(b)) = (&p[2], "1.E-3".parse::<f64>().ok()) else { panic!() };
    assert_eq!(a.to_bits(), b.to_bits());
    let (Param::Real(a), Some(b)) = (&params("#1 = A(-4.898587196589E-16);")[0], "-4.898587196589E-16".parse::<f64>().ok()) else { panic!() };
    assert_eq!(a.to_bits(), b.to_bits());
}

#[test]
fn nested_lists_and_typed_parameters() {
    let p = params("#1 = A(((1,2),(3)),LENGTH_MEASURE(25.4),X(1,2));");
    assert_eq!(p[0], Param::List(vec![Param::List(vec![Param::Int(1), Param::Int(2)]), Param::List(vec![Param::Int(3)])]));
    assert_eq!(p[1], Param::Typed("LENGTH_MEASURE".into(), Box::new(Param::Real(25.4))));
    assert_eq!(p[2], Param::Typed("X".into(), Box::new(Param::List(vec![Param::Int(1), Param::Int(2)]))));
    assert_eq!(params("#1 = A((),());"), [Param::List(vec![]), Param::List(vec![])]);
}

#[test]
fn keywords_are_upper_cased_and_offsets_recorded() {
    let bytes = wrap("#1 = abc_def(1);");
    let f = parse(&bytes).expect("parse");
    let r = f.entities[&1].first().expect("rec");
    assert_eq!(r.name, "ABC_DEF");
    assert_eq!(&bytes[r.offset..r.offset + 7], b"abc_def");
}

#[test]
fn complex_entities_including_the_occ_spaced_form() {
    let e = first("#426 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );");
    let Entity::Complex(v) = &e else { panic!("complex") };
    assert_eq!(v.iter().map(|r| r.name.as_str()).collect::<Vec<_>>(), ["LENGTH_UNIT", "NAMED_UNIT", "SI_UNIT"]);
    assert_eq!(v[2].params, [Param::Enum("MILLI".into()), Param::Enum("METRE".into())]);
    assert!(e.is("SI_UNIT") && !e.is("PLANE"));
    assert_eq!(e.display_name(), "LENGTH_UNIT+NAMED_UNIT+SI_UNIT");
}

#[test]
fn compact_complex_entity() {
    let e = first("#5=(A() B(1));");
    let Entity::Complex(v) = e else { panic!("complex") };
    assert_eq!(v.len(), 2);
    assert_eq!(v[1].params, [Param::Int(1)]);
}

#[test]
fn deep_nesting_is_rejected_without_overflowing_the_stack() {
    let mut s = String::from("#1 = A(");
    s.push_str(&"(".repeat(100_000));
    let err = parse(&wrap(&s)).expect_err("too deep");
    assert!(matches!(err, StepError::TooDeep { max: 256, .. }), "{err:?}");
    // Exactly at the limit still parses.
    let ok = format!("#1 = A({}{});", "(".repeat(255), ")".repeat(255));
    parse(&wrap(&ok)).expect("255 inner levels plus the record list is 256");
    let over = format!("#1 = A({}{});", "(".repeat(256), ")".repeat(256));
    assert!(matches!(parse(&wrap(&over)), Err(StepError::TooDeep { .. })));
}

#[test]
fn errors_carry_exact_line_and_column() {
    // Missing `;` after the first entity: the `#` of the next entity is at 9:1.
    let text = "ISO-10303-21;\nHEADER;\nFILE_SCHEMA(('X'));\nENDSEC;\nDATA;\n#1 = A(1);\n\n#2 = B(2)\n#3 = C(3);\nENDSEC;\nEND-ISO-10303-21;\n";
    let Err(StepError::Syntax { line, col, .. }) = parse(text.as_bytes()) else { panic!("syntax error expected") };
    assert_eq!((line, col), (9, 1));
    // A bad token inside a record.
    let text = "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1 = A(1, @);\nENDSEC;\nEND-ISO-10303-21;\n";
    let Err(StepError::Syntax { line, col, .. }) = parse(text.as_bytes()) else { panic!("syntax error expected") };
    assert_eq!((line, col), (5, 11));
    assert_eq!(line_col(b"ab\ncd", 4), (2, 2));
}

#[test]
fn truncation_and_non_step_input() {
    assert!(matches!(parse(b""), Err(StepError::Truncated { .. })));
    assert!(matches!(parse(b"hello"), Err(StepError::Syntax { line: 1, col: 1, .. })));
    let t = "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1 = A(1);\n";
    assert!(matches!(parse(t.as_bytes()), Err(StepError::Truncated { .. })));
    let t = "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1 = A(1";
    assert!(matches!(parse(t.as_bytes()), Err(StepError::Truncated { .. })));
    let t = "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1 = A('abc);\nENDSEC;\nEND-ISO-10303-21;\n";
    assert!(matches!(parse(t.as_bytes()), Err(StepError::Syntax { line: 5, col: 8, .. })));
}

#[test]
fn duplicate_ids_and_second_data_sections_are_rejected() {
    assert!(matches!(parse(&wrap("#1 = A(1);\n#1 = B(2);")), Err(StepError::Syntax { line: 9, col: 1, .. })));
    let t = "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n";
    assert!(matches!(parse(t.as_bytes()), Err(StepError::Syntax { line: 6, .. })));
}

#[test]
fn non_utf8_strings_fall_back_to_latin1() {
    let mut text = wrap("#1 = A('x');");
    let at = text.iter().position(|&c| c == b'x').expect("x");
    text[at] = 0xE9;
    let f = parse(&text).expect("parse");
    assert_eq!(f.entities[&1].first().map(|r| r.params.clone()), Some(vec![Param::Str("\u{e9}".into())]));
}
