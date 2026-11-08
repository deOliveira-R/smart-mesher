//! `smcsg`: exit codes 0/1/2 on hand-built decks (through the library entry point), the `--json`
//! golden of the overlap deck, and the real binary on files.
mod common;
use common::*;
use sm_csgio::cli::{LoadRequest, run};
use sm_csgio::{CsgIr, ImportError};

fn loader(req: &LoadRequest<'_>) -> Result<CsgIr, ImportError> {
    match req.deck.to_str().unwrap() {
        "ok" => Ok(deck_o()),
        "overlap" => Ok(deck_o_overlap()),
        "cyl" => Ok(deck_cyl(false)),
        "sph" => Ok(deck_sph()),
        "gap" => Ok(deck_o_gap()),
        "unsupported" => Err(ImportError::Unsupported { line: Some(3), what: "hex_lattice".into() }),
        "parse" => Err(ImportError::Parse { line: 2, col: 5, msg: "bad".into() }),
        other => panic!("unexpected deck {other}"),
    }
}

fn smcsg(args: &[&str]) -> (i32, String, String) {
    let args: Vec<String> = args.iter().map(|s| s.to_string()).collect();
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let code = run(&args, &loader, &mut out, &mut err);
    (code, String::from_utf8(out).unwrap(), String::from_utf8(err).unwrap())
}

#[test]
fn exit_0_for_a_clean_deck() {
    let (code, out, err) = smcsg(&["check", "ok"]);
    assert_eq!(code, 0, "{err}");
    assert!(out.starts_with("ok: 2 cells, 2 regions (material_1, material_2)"), "{out}");
    assert!(err.is_empty(), "a clean import prints no warnings: {err}");
}

#[test]
fn exit_1_for_an_overlap_and_for_gaps_only_when_strict() {
    let (code, out, _) = smcsg(&["check", "overlap"]);
    assert_eq!(code, 1);
    assert!(out.starts_with("overlap:") && out.contains("cells 1 and 2") || out.contains("cells 2 and 1"), "{out}");
    let (code, out, err) = smcsg(&["check", "gap"]);
    assert_eq!(code, 0, "{out}");
    assert!(err.lines().any(|l| l.starts_with("warning: import.gap")), "{err}");
    let (code, out, _) = smcsg(&["check", "gap", "--strict-gaps"]);
    assert_eq!(code, 1);
    assert!(out.starts_with("gaps:"), "{out}");
}

#[test]
fn curved_vacuum_boundaries_are_clean_even_with_strict_gaps() {
    for deck in ["cyl", "sph"] {
        let (code, out, err) = smcsg(&["check", deck, "--strict-gaps"]);
        assert_eq!(code, 0, "{deck}: {out} {err}");
        assert!(err.is_empty(), "{deck}: {err}");
    }
}

#[test]
fn exit_2_for_unreadable_decks_and_bad_usage() {
    for args in [vec!["check", "unsupported"], vec!["check", "parse"], vec!["check"], vec!["frobnicate", "ok"], vec!["check", "ok", "--format", "mcnp"], vec!["check", "ok", "--samples", "10"], vec!["check", "ok", "--nope"], vec!["check", "ok", "--names", "x=y"]] {
        let (code, _, err) = smcsg(&args);
        assert_eq!(code, 2, "{args:?}: {err}");
        assert!(err.starts_with("error:"), "{err}");
    }
    let (_, _, err) = smcsg(&["check", "unsupported"]);
    assert!(err.contains("line 3") && err.contains("hex_lattice"), "{err}");
}

#[test]
fn names_flag_renames_regions() {
    let (code, out, _) = smcsg(&["check", "ok", "--names", "1=fuel,2=water"]);
    assert_eq!(code, 0);
    assert!(out.contains("(fuel, water)"), "{out}");
}

#[test]
fn info_counts() {
    let (code, out, _) = smcsg(&["info", "ok", "--json"]);
    assert_eq!(code, 0);
    let v: serde_json::Value = serde_json::from_str(&out).unwrap();
    assert_eq!(v["surfaces"]["plane"], 6);
    assert_eq!(v["surfaces"]["sphere"], 1);
    assert_eq!(v["cells"], 2);
    assert_eq!(v["flat_cells"], 2);
    let (code, out, _) = smcsg(&["info", "ok"]);
    assert_eq!(code, 0);
    assert!(out.contains("surfaces: 7") && out.contains("flat: 2 cells"), "{out}");
}

#[test]
fn json_output_of_the_overlap_deck_matches_the_golden() {
    let (code, out, _) = smcsg(&["check", "overlap", "--json"]);
    assert_eq!(code, 1);
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/golden/deck_o_overlap.json");
    if std::env::var_os("SMCSG_BLESS").is_some() {
        std::fs::write(path, &out).unwrap();
    }
    let golden = std::fs::read_to_string(path).expect("golden missing: run once with SMCSG_BLESS=1");
    assert_eq!(out, golden);
    // Same bytes on a second run and on one thread.
    assert_eq!(smcsg(&["check", "overlap", "--json"]).1, out);
}

#[test]
fn the_binary_exits_2_on_a_deck_it_cannot_read() {
    let bin = env!("CARGO_BIN_EXE_smcsg");
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("smcsg-bin");
    std::fs::create_dir_all(&dir).unwrap();
    let deck = dir.join("geometry.xml");
    std::fs::write(&deck, "<geometry><cell id=\"1\" material=\"void\" region=\"-1\"/><surface id=\"1\" type=\"sphere\" coeffs=\"0 0 0 1\" boundary=\"periodic\"/></geometry>").unwrap();
    let run = |args: &[&std::ffi::OsStr]| std::process::Command::new(bin).args(args).output().unwrap();
    // A periodic boundary is unsupported: exit 2.
    let o = run(&["check".as_ref(), deck.as_os_str()]);
    assert_eq!(o.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&o.stderr).contains("unsupported"));
    // A missing file, a missing argument and an unknown format are exit 2 too.
    assert_eq!(run(&["check".as_ref(), dir.join("nope.xml").as_os_str()]).status.code(), Some(2));
    assert_eq!(run(&[]).status.code(), Some(2));
    assert_eq!(run(&["check".as_ref(), deck.as_os_str(), "--format".as_ref(), "mcnp".as_ref()]).status.code(), Some(2));
}
