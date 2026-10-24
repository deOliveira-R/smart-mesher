//! Golden tests for syntax diagnostics: `tests/golden/parse/*.smesh` -> `*.txt`.
//! Run with `UPDATE_GOLDEN=1` to regenerate the expected files after an intended change.

use std::fs;
use std::path::PathBuf;

use sm_lang::SourceMap;
use sm_lang::diag::render_all;

fn dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/golden/parse")
}

fn render(name: &str, text: &str) -> String {
    let mut map = SourceMap::new();
    let file = map.add(name, text);
    let out = sm_lang::parse(file, text);
    render_all(&out.diagnostics, &map)
}

#[test]
fn syntax_errors_match_golden_files() {
    let mut inputs: Vec<PathBuf> = fs::read_dir(dir()).unwrap().map(|e| e.unwrap().path()).filter(|p| p.extension().is_some_and(|e| e == "smesh")).collect();
    inputs.sort();
    assert!(inputs.len() >= 15, "golden inputs missing");
    let update = std::env::var_os("UPDATE_GOLDEN").is_some();
    let mut failures = Vec::new();
    for p in inputs {
        let text = fs::read_to_string(&p).unwrap();
        let name = p.file_name().unwrap().to_string_lossy().into_owned();
        let got = render(&name, &text);
        assert!(!got.is_empty(), "{name}: golden input must produce a diagnostic");
        let exp = p.with_extension("txt");
        if update {
            fs::write(&exp, &got).unwrap();
            continue;
        }
        let want = fs::read_to_string(&exp).unwrap_or_default();
        if want != got {
            failures.push(format!("--- {name}\nexpected:\n{want}\ngot:\n{got}"));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
