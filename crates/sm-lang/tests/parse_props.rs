//! Property tests: the lexer and parser never panic, always terminate, produce in-bounds spans on
//! char boundaries, are deterministic, and accept every program a small generator can build.

use proptest::prelude::*;
use sm_lang::ast::*;
use sm_lang::lexer::{Tok, lex};
use sm_lang::{Diagnostic, parse};
use sm_testkit::strategies::config;

const FRAGMENTS: &[&str] = &[
    "let ", "set ", "alias ", "batch ", "if ", "else ", "for ", "in ", "include ", "print ", "assert ", "all ", "except ", "and ", "or ", "not ",
    "true", "false", "x", "y_1", "model.create_brick", "faces", "volumes", "Volume#3", "Volume#1..5", "Face#[1, 2..4]", "Node#", "Foo#2", "#", "\"str\"",
    "\"unterminated", "\\", "\\\n", "1", "2.5", "1e9", "1e999", "99999999999999999999", "..", "...", ".", ",", ";", ":", "=", "==", "!=", "<=", ">=",
    "(", ")", "[", "]", "{", "}", "+", "-", "*", "**", "/", "%", "|", "&", "!", "$", "@", "é", "\n", "\r\n", " ", "\t", "// c\n", "/*", "*/", "by ",
    "b.body[*]", "range(0, 3)",
];

fn spans_ok(src: &str, diags: &[Diagnostic]) {
    for d in diags {
        assert!(d.span.lo <= d.span.hi && d.span.hi as usize <= src.len(), "span out of bounds: {d:?}");
        assert!(src.is_char_boundary(d.span.lo as usize) && src.is_char_boundary(d.span.hi as usize), "span splits a char: {d:?}");
        for (s, _) in &d.notes {
            assert!(s.hi as usize <= src.len() && src.is_char_boundary(s.lo as usize) && src.is_char_boundary(s.hi as usize));
        }
    }
}

fn check_source(src: &str) {
    let mut ld = Vec::new();
    let toks = lex(0, src, &mut ld);
    assert_eq!(toks.last().map(|t| &t.tok), Some(&Tok::Eof));
    let mut prev = 0;
    for t in &toks {
        assert!(t.span.lo >= prev && t.span.lo <= t.span.hi && t.span.hi as usize <= src.len());
        prev = t.span.hi.min(t.span.lo.max(prev));
    }
    let a = parse(0, src);
    spans_ok(src, &a.diagnostics);
    let mut map = sm_lang::SourceMap::new();
    map.add("p", src);
    // Rendering must not panic either.
    let _ = sm_lang::diag::render_all(&a.diagnostics, &map);
    let b = parse(0, src);
    assert_eq!(a.script, b.script, "parse is deterministic");
    assert_eq!(a.diagnostics, b.diagnostics);
    // A tree with no errors must have no `Error` nodes; with errors, at least one diagnostic is an error.
    let has_err_node = format!("{:?}", a.script).contains("Error");
    if !a.has_errors() {
        assert!(!has_err_node || src.contains("Error"), "error node without a diagnostic");
    }
}

proptest! {
    #![proptest_config(config(2000))]

    #[test]
    fn arbitrary_text_never_panics(s in ".{0,200}") {
        check_source(&s);
    }

    #[test]
    fn token_soup_never_panics(parts in proptest::collection::vec(0..FRAGMENTS.len(), 0..60)) {
        let s: String = parts.iter().map(|&i| FRAGMENTS[i]).collect();
        check_source(&s);
    }

    #[test]
    fn truncation_of_valid_source_never_panics(cut in 0usize..400) {
        let src = VALID;
        let mut c = cut.min(src.len());
        while !src.is_char_boundary(c) { c -= 1; }
        check_source(&src[..c]);
    }

    #[test]
    fn mutated_valid_source_never_panics(pos in 0usize..400, ins in 0..FRAGMENTS.len(), del in 0usize..6) {
        let src = VALID;
        let mut p = pos.min(src.len());
        while !src.is_char_boundary(p) { p -= 1; }
        let mut e = (p + del).min(src.len());
        while !src.is_char_boundary(e) { e += 1; }
        let s = format!("{}{}{}", &src[..p], FRAGMENTS[ins], &src[e..]);
        check_source(&s);
    }
}

const VALID: &str = "let w = 10\nlet b = model.create_brick(x: w, y: 2 * w, z: [1, 2][0], name: \"s\")\nbatch {\n  intent.set_size(ids: faces in Volume#2 - Face#[1, 3..5], size: 0.5)\n}\nfor i in range(0, 3) { print i }\nif w > 1 and not false { print all volumes } else { print 1 }\n";

#[test]
fn the_seed_program_is_valid() {
    let out = parse(0, VALID);
    assert!(out.diagnostics.is_empty(), "{:?}", out.diagnostics);
}

// ---- generator of valid programs --------------------------------------------------------

fn expr_strategy() -> impl Strategy<Value = String> {
    let leaf = prop_oneof![
        (0i64..1000).prop_map(|n| n.to_string()),
        (0u32..100, 0u32..100).prop_map(|(a, b)| format!("{a}.{b}")),
        Just("true".to_owned()),
        "[a-z][a-z0-9_]{0,6}".prop_filter("not a keyword", |s| !["let", "set", "alias", "batch", "if", "else", "for", "in", "include", "print", "assert", "true", "false", "and", "or", "not", "all", "except"].contains(&s.as_str()) && sm_lang::kinds::kind_from_plural(s).is_none()),
        "[a-z ]{0,8}".prop_map(|s| format!("{s:?}")),
        (1u32..50).prop_map(|n| format!("Volume#{n}")),
        (1u32..20, 20u32..50).prop_map(|(a, b)| format!("Face#{a}..{b}")),
        Just("all faces".to_owned()),
    ];
    leaf.prop_recursive(4, 32, 4, |inner| {
        prop_oneof![
            (inner.clone(), inner.clone(), prop_oneof![Just("+"), Just("-"), Just("*"), Just("|"), Just("&"), Just("except"), Just("and"), Just("or"), Just("==")])
                .prop_map(|(a, b, op)| format!("({a} {op} {b})")),
            inner.clone().prop_map(|a| format!("-({a})")),
            proptest::collection::vec(inner.clone(), 0..4).prop_map(|v| format!("[{}]", v.join(", "))),
            inner.clone().prop_map(|a| format!("faces in ({a})")),
            (inner.clone(), inner).prop_map(|(a, b)| format!("min({a}, {b})")),
        ]
    })
}

proptest! {
    #![proptest_config(config(500))]

    #[test]
    fn generated_programs_parse_without_diagnostics(es in proptest::collection::vec(expr_strategy(), 1..6)) {
        let mut src = String::new();
        for (i, e) in es.iter().enumerate() {
            match i % 3 {
                0 => src.push_str(&format!("let v{i} = {e}\n")),
                1 => src.push_str(&format!("model.op{i}(a: {e}, b: [{e}], ...{{k: {e}}})\n")),
                _ => src.push_str(&format!("print {e}; assert {e}, {e}\n")),
            }
        }
        let out = parse(0, &src);
        prop_assert!(out.diagnostics.is_empty(), "{src}\n{:?}", out.diagnostics);
        prop_assert!(out.script.stmts.iter().all(|s| !matches!(s.kind, StmtKind::Error)));
    }
}
