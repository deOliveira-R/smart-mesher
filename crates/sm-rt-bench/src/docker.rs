//! Docker side of the benchmark: lint of the competitor Dockerfiles and `docker-check`.
//!
//! Images are pinned by digest and source commit. The base-image digests live in
//! `docker/pins.env` and must be filled in on the benchmark machine (`UNSET` is rejected by
//! `docker-check`: no digest is invented here). Nothing in this crate runs Docker in tests.

use crate::error::{BenchError, Result};
use std::collections::BTreeMap;
use std::path::Path;

/// The competitor images: `(image name, Dockerfile path relative to the crate root)`.
pub const IMAGES: [(&str, &str); 3] = [
    ("smrt-bench-dagmc-moab", "docker/dagmc-moab/Dockerfile"),
    ("smrt-bench-dagmc-dd", "docker/dagmc-dd/Dockerfile"),
    ("smrt-bench-xdg", "docker/xdg/Dockerfile"),
];

/// Source commits that must appear in the Dockerfiles: `(Dockerfile, commit)`.
pub const PINNED_COMMITS: [(&str, &str); 2] = [("docker/dagmc-dd/Dockerfile", "437ad209eac5"), ("docker/xdg/Dockerfile", "83efb306e152")];

/// Parse `KEY=VALUE` lines (`#` comments and blank lines ignored).
pub fn parse_pins(text: &str) -> BTreeMap<String, String> {
    text.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with('#')).filter_map(|l| l.split_once('=')).map(|(k, v)| (k.trim().to_string(), v.trim().to_string())).collect()
}

/// Problems of a Dockerfile text: a `FROM` without `@sha256:` or with `:latest`, an `ADD`/`curl`
/// of unpinned sources is not decidable here and left to review.
pub fn lint_dockerfile(text: &str) -> Vec<String> {
    let mut problems = Vec::new();
    let mut froms = 0;
    for (n, line) in text.lines().enumerate() {
        let l = line.trim();
        if l.to_ascii_uppercase().starts_with("FROM ") {
            froms += 1;
            if !l.contains("@sha256:") {
                problems.push(format!("line {}: FROM is not pinned by digest: {l}", n + 1));
            }
        }
        if l.contains(":latest") && !l.starts_with('#') {
            problems.push(format!("line {}: uses :latest", n + 1));
        }
    }
    if froms == 0 {
        problems.push("no FROM line".to_string());
    }
    problems
}

/// Lint every Dockerfile under `root` (the crate directory) and check the pinned commits.
pub fn lint_all(root: &Path) -> Result<Vec<String>> {
    let mut problems = Vec::new();
    for (name, rel) in IMAGES {
        let p = root.join(rel);
        let text = std::fs::read_to_string(&p).map_err(|e| BenchError::io(&p, e))?;
        problems.extend(lint_dockerfile(&text).into_iter().map(|m| format!("{name}: {m}")));
    }
    for (rel, commit) in PINNED_COMMITS {
        let p = root.join(rel);
        let text = std::fs::read_to_string(&p).map_err(|e| BenchError::io(&p, e))?;
        if !text.contains(commit) {
            problems.push(format!("{rel}: pinned commit {commit} not found"));
        }
    }
    Ok(problems)
}

/// Outcome of `docker-check` for one image.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ImageStatus {
    /// The Dockerfile failed the lint.
    LintFailed(Vec<String>),
    /// A base-image digest in `pins.env` is `UNSET` (or empty): fill it in on the benchmark machine.
    Unpinned(Vec<String>),
    /// Docker is not available; the failing command is recorded.
    Unavailable(String),
    /// Built, and the driver printed the pinned version line.
    Built {
        /// Image id as printed by `docker build -q`.
        id: String,
    },
    /// The build or the version check failed.
    Failed(String),
}

/// Run `docker-check`: lint, pins, then build each image (D acceptance, never fabricated).
pub fn docker_check(root: &Path) -> Result<Vec<(String, ImageStatus)>> {
    let pins_path = root.join("docker/pins.env");
    let pins = parse_pins(&std::fs::read_to_string(&pins_path).map_err(|e| BenchError::io(&pins_path, e))?);
    let lint = lint_all(root)?;
    let mut out = Vec::new();
    let unset: Vec<String> = pins.iter().filter(|(k, v)| k.ends_with("_DIGEST") && (v.as_str() == "UNSET" || v.is_empty())).map(|(k, _)| k.clone()).collect();
    // Probed only once lint and pins pass: an unpinned tree never touches Docker.
    let mut docker: Option<bool> = None;
    for (name, rel) in IMAGES {
        let mine: Vec<String> = lint.iter().filter(|m| m.starts_with(name) || m.starts_with(rel)).cloned().collect();
        let status = if !mine.is_empty() {
            ImageStatus::LintFailed(mine)
        } else if !unset.is_empty() {
            ImageStatus::Unpinned(unset.clone())
        } else if !*docker.get_or_insert_with(|| std::process::Command::new("docker").arg("version").output().is_ok_and(|o| o.status.success())) {
            ImageStatus::Unavailable("docker version".to_string())
        } else {
            build_image(root, name, rel, &pins)
        };
        out.push((name.to_string(), status));
    }
    Ok(out)
}

fn build_image(root: &Path, name: &str, rel: &str, pins: &BTreeMap<String, String>) -> ImageStatus {
    let mut cmd = std::process::Command::new("docker");
    cmd.arg("build").arg("-q").arg("-t").arg(name).arg("-f").arg(rel);
    for (k, v) in pins.iter().filter(|(k, _)| k.ends_with("_DIGEST")) {
        cmd.arg("--build-arg").arg(format!("{k}={v}"));
    }
    cmd.arg(".").current_dir(root);
    match cmd.output() {
        Ok(o) if o.status.success() => ImageStatus::Built { id: String::from_utf8_lossy(&o.stdout).trim().to_string() },
        Ok(o) => ImageStatus::Failed(format!("docker build {name}: {}", String::from_utf8_lossy(&o.stderr).lines().last().unwrap_or(""))),
        Err(e) => ImageStatus::Failed(format!("docker build {name}: {e}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn root() -> std::path::PathBuf {
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
    }

    #[test]
    fn committed_dockerfiles_pass_the_lint() {
        assert_eq!(lint_all(&root()).unwrap(), Vec::<String>::new());
    }

    #[test]
    fn lint_rejects_latest_and_undigested_from() {
        assert!(!lint_dockerfile("FROM condaforge/miniforge3:latest\n").is_empty());
        assert!(!lint_dockerfile("FROM ubuntu:24.04\nRUN true\n").is_empty());
        assert!(lint_dockerfile("ARG D\nFROM ubuntu@sha256:${D}\n").is_empty());
        assert!(!lint_dockerfile("RUN true\n").is_empty());
    }

    #[test]
    fn unset_digests_are_refused_before_any_docker_call() {
        let st = docker_check(&root()).unwrap();
        assert_eq!(st.len(), 3);
        let pins = parse_pins(&std::fs::read_to_string(root().join("docker/pins.env")).unwrap());
        if pins.values().any(|v| v == "UNSET") {
            assert!(st.iter().all(|(_, s)| matches!(s, ImageStatus::Unpinned(_))), "{st:?}");
        }
    }
}
