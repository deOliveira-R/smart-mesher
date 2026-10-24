//! Parser behaviour: tree shapes, precedence, lexical rules, recovery.

use sm_lang::ast::*;
use sm_lang::{SourceMap, parse};

fn sx(e: &Expr) -> String {
    match &e.kind {
        ExprKind::Int(v) => v.to_string(),
        ExprKind::Num(v) => format!("{v:?}"),
        ExprKind::Str(s) => format!("{s:?}"),
        ExprKind::Bool(b) => b.to_string(),
        ExprKind::List(v) => format!("[{}]", v.iter().map(sx).collect::<Vec<_>>().join(" ")),
        ExprKind::Obj(v) => format!("{{{}}}", v.iter().map(|(k, e)| format!("{}:{}", k.name, sx(e))).collect::<Vec<_>>().join(" ")),
        ExprKind::Var(n) => n.clone(),
        ExprKind::Field(b, f) => format!("(. {} {})", sx(b), f.name),
        ExprKind::Index(b, Index::All) => format!("(idx {} *)", sx(b)),
        ExprKind::Index(b, Index::At(i)) => format!("(idx {} {})", sx(b), sx(i)),
        ExprKind::Call(c) => format!("(call {} {})", c.path.dotted(), c.args.iter().map(sxa).collect::<Vec<_>>().join(" ")),
        ExprKind::Unary(op, a) => format!("({} {})", if *op == UnOp::Neg { "neg" } else { "not" }, sx(a)),
        ExprKind::Binary(op, a, b) => format!("({:?} {} {})", op, sx(a), sx(b)),
        ExprKind::Entity(EntityExpr::Ids { kind, items, .. }) => {
            let it: Vec<String> = items
                .iter()
                .map(|i| match i {
                    IdItem::One(n) => sx(n),
                    IdItem::Range { lo, hi, step, .. } => format!("{}..{}/{}", sx(lo), sx(hi), step.as_ref().map_or("1".to_owned(), sx)),
                })
                .collect();
            format!("{}#[{}]", kind.name(), it.join(","))
        }
        ExprKind::Entity(EntityExpr::All { kind, .. }) => format!("all:{}", kind.name()),
        ExprKind::Entity(EntityExpr::In { kind, within, .. }) => format!("({}-in {})", kind.name(), sx(within)),
        ExprKind::Error => "<err>".into(),
    }
}

fn sxa(a: &Arg) -> String {
    match a {
        Arg::Named { name, value } => format!("{}={}", name.name, sx(value)),
        Arg::Spread(e) => format!("...{}", sx(e)),
        Arg::Pos(e) => sx(e),
    }
}

/// Parse `let _ = <expr>` and return the expression's s-expression.
fn e(src: &str) -> String {
    let out = parse(0, &format!("let _ = {src}"));
    assert!(out.diagnostics.is_empty(), "{src}: {:?}", out.diagnostics);
    match &out.script.stmts[0].kind {
        StmtKind::Let { value, .. } => sx(value),
        k => panic!("{k:?}"),
    }
}

#[test]
fn literals_and_numbers() {
    assert_eq!(e("42"), "42");
    assert_eq!(e("1_000"), "1000");
    assert_eq!(e("1.5"), "1.5");
    assert_eq!(e("2e-3"), "0.002");
    assert_eq!(e("1.5E+2"), "150.0");
    assert_eq!(e(r#""a\tb\u{41}\$""#), r#""a\tbA$""#);
    assert_eq!(e("true"), "true");
}

#[test]
fn dot_dot_is_not_part_of_a_number() {
    assert_eq!(e("Volume#1..5"), "TopoVolume#[1..5/1]");
    // `1.` is an int followed by a field access error, never a float.
    assert!(parse(0, "let _ = 1.").has_errors());
    assert!(parse(0, "let _ = .5").has_errors());
}

#[test]
fn precedence_and_associativity() {
    assert_eq!(e("1 + 2 * 3"), "(Add 1 (Mul 2 3))");
    assert_eq!(e("1 - 2 - 3"), "(Sub (Sub 1 2) 3)");
    assert_eq!(e("-2 ** 2"), "(neg (Pow 2 2))");
    assert_eq!(e("2 ** 3 ** 2"), "(Pow 2 (Pow 3 2))");
    assert_eq!(e("2 ** -1"), "(Pow 2 (neg 1))");
    assert_eq!(e("a | b & c"), "(Union a (Inter b c))");
    // `-`/`except` next to `|`/`&` must be parenthesized (see `mixing_difference_with_union_is_rejected`).
    assert_eq!(e("a & (b - c)"), "(Inter a (Sub b c))");
    assert_eq!(e("(a except b) | c"), "(Union (Except a b) c)");
    assert_eq!(e("a - b except c"), "(Except (Sub a b) c)");
    assert_eq!(e("not a and b or c"), "(Or (And (not a) b) c)");
    assert_eq!(e("1 + 2 == 3"), "(Eq (Add 1 2) 3)");
    assert_eq!(e("(1 + 2) * 3"), "(Mul (Add 1 2) 3)");
}

#[test]
fn entity_forms() {
    assert_eq!(e("Volume#3"), "TopoVolume#[3]");
    assert_eq!(e("Volume#1..9 by 2"), "TopoVolume#[1..9/2]");
    assert_eq!(e("Face#[1, 3, 5..7]"), "TopoFace#[1,3,5..7/1]");
    assert_eq!(e("MeshCell#4"), "MeshCell#[4]");
    assert_eq!(e("Node#2"), "MeshVertex#[2]");
    assert_eq!(e("all faces"), "all:TopoFace");
    assert_eq!(e("faces in Volume#2"), "(TopoFace-in TopoVolume#[2])");
    assert_eq!(e("(faces in Volume#2) - (faces in Volume#3)"), "(Sub (TopoFace-in TopoVolume#[2]) (TopoFace-in TopoVolume#[3]))");
    assert_eq!(e("all faces - faces in Volume#3"), "(Sub all:TopoFace (TopoFace-in TopoVolume#[3]))");
    assert_eq!(e("all faces in Volume#2"), "(TopoFace-in TopoVolume#[2])", "`all <plural> in X` is `<plural> in X`");
    assert_eq!(e("faces in (Volume#2 | Volume#3)"), "(TopoFace-in (Union TopoVolume#[2] TopoVolume#[3]))");
    assert_eq!(e("edges in b.body[0]"), "(TopoEdge-in (idx (. b body) 0))");
    assert_eq!(e("all volumes except Volume#2"), "(Except all:TopoVolume TopoVolume#[2])");
}

#[test]
fn plural_keywords_are_contextual() {
    // Usable as ordinary variable names; keywords only after `all` or before `in`.
    assert_eq!(e("faces + 1"), "(Add faces 1)");
    let out = parse(0, "let faces = 3\nfor volumes in range(2) { print volumes }");
    assert!(out.diagnostics.is_empty(), "{:?}", out.diagnostics);
}

#[test]
fn postfix_and_calls() {
    assert_eq!(e("b.body[0]"), "(idx (. b body) 0)");
    assert_eq!(e("b.body[*]"), "(idx (. b body) *)");
    assert_eq!(e("sin(pi / 2)"), "(call sin (Div pi 2))");
    assert_eq!(e("model.f(1, x: 2, ...p)"), "(call model.f 1 x=2 ...p)");
    assert_eq!(e("[1, 2, 3][-1]"), "(idx [1 2 3] (neg 1))");
    assert_eq!(e(r#"{a: 1, "b c": [2]}"#), "{a:1 b c:[2]}");
}

#[test]
fn newlines_comments_and_separators() {
    let src = "let a = 1; let b = 2 // trailing\n/* block /* nested */ still */ let c = 3 +\n  4\nmodel.f(\n  x: 1,\n  y: [1,\n     2],\n)\nlet d = 1 + \\\n 2\n";
    let out = parse(0, src);
    assert!(out.diagnostics.is_empty(), "{:?}", out.diagnostics);
    assert_eq!(out.script.stmts.len(), 5);
    // A newline outside brackets and after no operator ends the statement.
    let out = parse(0, "let a = 1\n+ 2");
    assert!(out.has_errors());
}

#[test]
fn statements() {
    let src = r#"
alias brick = model.create_brick
let b = brick(x: 1)
set b = 2
batch { model.a(x: 1); model.b(x: 2) }
if b == 2 { print "two" } else if b == 3 { print "three" } else { print "other" }
for i in range(3) { print i, i * 2 }
include "other.smesh"
assert b > 1, "b must be big"
help
help model.create_brick
"#;
    let out = parse(0, src);
    assert!(out.diagnostics.is_empty(), "{:?}", out.diagnostics);
    let kinds: Vec<&str> = out
        .script
        .stmts
        .iter()
        .map(|s| match &s.kind {
            StmtKind::Alias { .. } => "alias",
            StmtKind::Let { .. } => "let",
            StmtKind::Set { .. } => "set",
            StmtKind::Batch(b) => {
                assert_eq!(b.stmts.len(), 2);
                "batch"
            }
            StmtKind::If { elifs, .. } if !elifs.is_empty() => "if-elseif",
            StmtKind::For { .. } => "for",
            StmtKind::Include { .. } => "include",
            StmtKind::Assert { msg: Some(_), .. } => "assert",
            StmtKind::Help(None) => "help",
            StmtKind::Help(Some(_)) => "help-name",
            other => panic!("{other:?}"),
        })
        .collect();
    assert_eq!(kinds, ["alias", "let", "set", "batch", "if-elseif", "for", "include", "assert", "help", "help-name"]);
}

#[test]
fn spans_cover_exactly_the_source_text() {
    let src = "model.create_brick(x: 10, y: Volume#2)";
    let out = parse(0, src);
    let StmtKind::Call(c) = &out.script.stmts[0].kind else { panic!() };
    let text = |s: sm_lang::Span| &src[s.lo as usize..s.hi as usize];
    assert_eq!(text(out.script.stmts[0].span), src);
    assert_eq!(text(c.path.span), "model.create_brick");
    assert_eq!(text(c.args_span), "(x: 10, y: Volume#2)");
    let Arg::Named { value, .. } = &c.args[1] else { panic!() };
    assert_eq!(text(value.span), "Volume#2");
}

#[test]
fn recovery_reports_several_errors_and_keeps_good_statements() {
    let src = "let a = 1\nlet = 2\nmodel.f(x: , y: 2)\nlet b = 3\nmodel.g(\nlet c = 4\n";
    let out = parse(0, src);
    assert_eq!(out.diagnostics.len(), 3, "{:?}", out.diagnostics);
    let good: Vec<String> = out
        .script
        .stmts
        .iter()
        .filter_map(|s| match &s.kind {
            StmtKind::Let { name, .. } => Some(name.name.clone()),
            _ => None,
        })
        .collect();
    assert_eq!(good, ["a", "b", "c"], "statements around the broken ones survive");
    // The unclosed `(` is reported once and does not swallow the next statement.
    assert!(out.diagnostics.iter().any(|d| d.code == "unclosed-bracket"));
}

#[test]
fn error_limit_stops_the_flood() {
    let src = "let = 1\n".repeat(500);
    let out = parse(0, &src);
    assert_eq!(out.diagnostics.len(), 101);
    assert_eq!(out.diagnostics.last().unwrap().code, "too-many-errors");
}

#[test]
fn nesting_limit_is_an_error_not_a_stack_overflow() {
    let deep = format!("let _ = {}1{}", "(".repeat(5000), ")".repeat(5000));
    let out = parse(0, &deep);
    assert!(out.diagnostics.iter().any(|d| d.code == "too-deep"));
    let deep = format!("let _ = {}", "[".repeat(5000));
    assert!(parse(0, &deep).has_errors());
    let deep = format!("let _ = {}x", "-".repeat(5000));
    assert!(parse(0, &deep).has_errors());
    let deep = "if x { ".repeat(5000);
    assert!(parse(0, &deep).has_errors());
}

#[test]
fn diagnostics_render_with_positions() {
    let mut map = SourceMap::new();
    let text = "let a = 1\nlet b = @\n";
    let f = map.add("t.smesh", text);
    let out = parse(f, text);
    let r = out.diagnostics[0].render(&map);
    assert!(r.contains("t.smesh:2:9"), "{r}");
}

#[test]
fn crlf_and_unicode_do_not_break_spans() {
    let src = "let s = \"héllo\"\r\nlet t = 1\r\nlet é = 2\r\n";
    let out = parse(0, src);
    // `é` is not a valid identifier character: one error, and the other statements survive.
    assert_eq!(out.diagnostics.len(), 1, "{:?}", out.diagnostics);
    assert_eq!(out.diagnostics[0].code, "invalid-character");
    let mut map = SourceMap::new();
    map.add("u", src);
    let r = out.diagnostics[0].render(&map);
    assert!(r.contains("u:3:5"), "{r}");
}

#[test]
fn long_operator_and_postfix_chains_are_diagnostics_not_aborts() {
    // Left-leaning trees would otherwise be as deep as the chain is long and overflow on drop or recursion.
    for src in [
        format!("let x = {}1", "1 + ".repeat(100_000)),
        format!("let x = x{}", ".a".repeat(100_000)),
        format!("let x = x{}", "[0]".repeat(100_000)),
        format!("let x = {}b", "a | ".repeat(100_000)),
        format!("let x = {}b", "a and ".repeat(100_000)),
        format!("let x = {}b", "a ** ".repeat(100_000)),
        format!("let x = {}b", "not ".repeat(100_000)),
        format!("let x = {}b", "- ".repeat(100_000)),
    ] {
        let out = parse(0, &src);
        assert!(out.has_errors());
        assert!(out.diagnostics.iter().any(|d| d.code == "expression-too-long" || d.code == "too-deep"), "{:?}", out.diagnostics.first());
        drop(out);
    }
    // A moderately long chain is fine.
    let ok = format!("let x = {}1", "1 + ".repeat(200));
    assert!(parse(0, &ok).diagnostics.is_empty());
}

#[test]
fn oversized_sources_are_rejected_before_scanning() {
    let limits = sm_lang::ParseLimits { max_source_bytes: 10, ..Default::default() };
    let out = sm_lang::parse_with(0, "let x = 1000000", limits);
    assert_eq!(out.diagnostics[0].code, "source-too-large");
    assert!(out.script.stmts.is_empty());
}

#[test]
fn reserved_words_are_valid_name_segments_after_a_dot() {
    // `core.batch` is a registry op; ops and fields may use any word as a later segment.
    assert_eq!(e("core.batch(1)"), "(call core.batch 1)");
    assert_eq!(e("b.all[0]"), "(idx (. b all) 0)");
    assert_eq!(e("b.in.set"), "(. (. b in) set)");
    let out = parse(0, "core.batch(ops: [])\nmodel.set(x: 1)\nmodel.if.for(y: 2)\n");
    assert!(out.diagnostics.is_empty(), "{:?}", out.diagnostics);
    // But a statement still cannot start with a reserved word that is not a statement keyword.
    assert!(parse(0, "all.thing(1)").has_errors());
}

fn first_error(src: &str) -> sm_lang::Diagnostic {
    let out = parse(0, &format!("let _ = {src}"));
    out.diagnostics.into_iter().find(|d| d.is_error()).unwrap_or_else(|| panic!("`{src}` parsed without an error"))
}

#[test]
fn in_does_not_silently_swallow_a_following_set_operator() {
    for src in ["faces in Volume#1 | Volume#2", "faces in Volume#1 & Volume#2", "faces in Volume#1 - Volume#2", "faces in Volume#1 except Volume#2", "all faces in Volume#1 | Volume#2", "faces in Volume#2 - faces in Volume#3"] {
        assert_eq!(first_error(src).code, "ambiguous-in", "{src}");
    }
    // Both readings are available with parentheses.
    assert_eq!(e("faces in (Volume#1 | Volume#2)"), "(TopoFace-in (Union TopoVolume#[1] TopoVolume#[2]))");
    assert_eq!(e("(faces in Volume#1) | Face#2"), "(Union (TopoFace-in TopoVolume#[1]) TopoFace#[2])");
}

#[test]
fn mixing_difference_with_union_is_rejected() {
    for src in ["all faces - Face#1 | Face#2", "all volumes except Volume#1 | Volume#2", "a | b - c", "a & b except c", "a - b & c"] {
        assert_eq!(first_error(src).code, "ambiguous-set-expression", "{src}");
    }
    assert_eq!(e("(all volumes except Volume#1) | Volume#2"), "(Union (Except all:TopoVolume TopoVolume#[1]) TopoVolume#[2])");
    assert_eq!(e("all volumes except (Volume#1 | Volume#2)"), "(Except all:TopoVolume (Union TopoVolume#[1] TopoVolume#[2]))");
}

#[test]
fn id_serials_can_be_computed() {
    assert_eq!(e("Volume#[i]"), "TopoVolume#[i]");
    assert_eq!(e("Volume#[i + 1, 5]"), "TopoVolume#[(Add i 1),5]");
    assert_eq!(e("Volume#[a..b]"), "TopoVolume#[a..b/1]");
    assert_eq!(e("Volume#[a..n * 2 by 2]"), "TopoVolume#[a..(Mul n 2)/2]");
    assert_eq!(e("Volume#(k - 1)"), "TopoVolume#[(Sub k 1)]");
    // The bare form stays literal, so `Volume#3 - Volume#2` is a set difference.
    assert_eq!(e("Volume#3 - Volume#2"), "(Sub TopoVolume#[3] TopoVolume#[2])");
    let d = parse(0, "let _ = Volume#i").diagnostics;
    assert!(d.iter().any(|d| d.help.as_deref().is_some_and(|h| h.contains("Volume#[i]"))), "{d:?}");
}

#[test]
fn reserved_predicate_words_follow_entities_only() {
    for src in ["faces in Volume#1 where area > 2", "all faces with name", "volumes named \"fuel*\"", "Volume#1 where x"] {
        assert_eq!(first_error(src).code, "not-yet-supported", "{src}");
    }
    // Everywhere else they are ordinary names.
    let out = parse(0, "let where = 1\nlet named = where + 1\nprint with_x, named");
    assert!(out.diagnostics.is_empty(), "{:?}", out.diagnostics);
}

#[test]
fn else_may_start_the_next_line_and_chains_are_flat() {
    let out = parse(0, "if a {\n  print 1\n}\nelse if b {\n  print 2\n}\nelse {\n  print 3\n}\n");
    assert!(out.diagnostics.is_empty(), "{:?}", out.diagnostics);
    let StmtKind::If { elifs, els, .. } = &out.script.stmts[0].kind else { panic!() };
    assert_eq!((elifs.len(), els.is_some()), (1, true));
    // A 300-branch chain costs no nesting depth and is accepted.
    let mut src = String::from("if x == 0 { print 0 }");
    for i in 1..300 {
        src.push_str(&format!(" else if x == {i} {{ print {i} }}"));
    }
    src.push_str(" else { print -1 }");
    let out = parse(0, &src);
    assert!(out.diagnostics.is_empty(), "{:?}", out.diagnostics.iter().map(|d| d.code).collect::<Vec<_>>());
    let StmtKind::If { elifs, .. } = &out.script.stmts[0].kind else { panic!() };
    assert_eq!(elifs.len(), 299);
    // A stray `else` is still an error.
    assert!(parse(0, "print 1\nelse { print 2 }").has_errors());
}

#[test]
fn in_without_a_plural_kind_explains_membership() {
    for src in ["if Volume#2 in s { print 1 }", "let x = Volume#1 in Body#1", "print a in b"] {
        let out = parse(0, src);
        let d = out.diagnostics.iter().find(|d| d.is_error()).expect("error");
        assert!(d.help.as_deref().is_some_and(|h| h.contains("contains(")), "{src}: {d:?}");
    }
}
