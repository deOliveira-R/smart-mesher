//! Running the Python checkers (skipped when no interpreter is available).
#![allow(dead_code)]

use std::path::{Path, PathBuf};
use std::process::Command;

/// Interpreter to use: `SM_PYTHON` or `python3`; `None` if it cannot run.
pub fn python() -> Option<String> {
    let py = std::env::var("SM_PYTHON").unwrap_or_else(|_| "python3".into());
    Command::new(&py).arg("--version").output().ok().filter(|o| o.status.success()).map(|_| py)
}

pub fn tmp(name: &str) -> PathBuf {
    let d = Path::new(env!("CARGO_TARGET_TMPDIR")).join("sm-meshio");
    std::fs::create_dir_all(&d).unwrap();
    d.join(name)
}

/// Run `python -I script file` and parse its stdout as JSON.
pub fn run_json(py: &str, script: &str, file: &Path) -> serde_json::Value {
    let out = Command::new(py).arg("-I").arg(script).arg(file).output().unwrap();
    assert!(out.status.success(), "{script} failed: {}", String::from_utf8_lossy(&out.stderr));
    serde_json::from_slice(&out.stdout).unwrap()
}
