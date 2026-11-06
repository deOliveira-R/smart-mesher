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
    /// The directory the Dockerfile copies the driver from is missing: the build would fail at
    /// its `COPY` step. The command is recorded; no driver is faked.
    NoDriver(String),
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
        let driver = if name.contains("xdg") { "drivers/xdg" } else { "drivers/dagmc" };
        let status = if !mine.is_empty() {
            ImageStatus::LintFailed(mine)
        } else if !root.join(driver).join(if name.contains("xdg") { "xdg_driver.cpp" } else { "dagmc_driver.cpp" }).is_file() {
            ImageStatus::NoDriver(format!("`COPY {driver}/ /bench/` in {rel} would fail: {driver}/{} does not exist", if name.contains("xdg") { "xdg_driver.cpp" } else { "dagmc_driver.cpp" }))
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
        Ok(o) => {
            // The last line is a pointer to the build dashboard; the cause is in the error lines.
            let text = String::from_utf8_lossy(&o.stderr).to_string();
            let errors: Vec<&str> = text.lines().filter(|l| l.to_ascii_lowercase().contains("error")).collect();
            let why = if errors.is_empty() { text.lines().last().unwrap_or("").to_string() } else { errors[errors.len().saturating_sub(2)..].join(" | ") };
            ImageStatus::Failed(format!("`docker build -q -t {name} -f {rel} .` failed: {why}"))
        }
        Err(e) => ImageStatus::Failed(format!("docker build {name}: {e}")),
    }
}

/// Result of asking a competitor image to run a case.
#[derive(Clone, Debug, PartialEq)]
pub enum Competitor {
    /// Rows printed by the driver (one JSON `Row` per line) and, when the driver printed them,
    /// the tracks of every ray (region ids and chords, in ray order).
    Rows {
        /// The rows.
        rows: Vec<crate::result::Row>,
        /// Tracks of the W-track run, if the driver dumped them.
        tracks: Option<Vec<crate::truth::Track>>,
    },
    /// Not run: the failing command or the reason (reported as `partial`, never as numbers).
    Partial(String),
}

#[derive(serde::Deserialize)]
struct TrackJson {
    r: Vec<i32>,
    c: Vec<f64>,
}

#[derive(serde::Deserialize)]
struct TracksLine {
    tracks: Vec<TrackJson>,
}

/// Tracker name (`dagmc-moab`, `dagmc-dd`, `xdg`) to its image name.
pub fn image_of(tracker: &str) -> Option<&'static str> {
    IMAGES.iter().map(|(n, _)| *n).find(|n| n.strip_prefix("smrt-bench-") == Some(tracker))
}

fn last_line(b: &[u8]) -> String {
    String::from_utf8_lossy(b).lines().rev().find(|l| !l.trim().is_empty()).unwrap_or("").to_string()
}

/// Run `tracker`'s image on a case directory (`smcase.json`, `facets.bin`, `rays.bin`, ...,
/// written by `CaseData::write`, mounted read-only at `/data/<case>`): `docker run --rm -v
/// DIR:/data:ro -e SMRT_REPS=R -e SMRT_WARMUP=W IMAGE <case> <threads>...`. Each stdout line that
/// starts with `{"tracks"` is the track dump, every other line that starts with `{` is a `Row`.
/// Anything that stops the run (no Docker, digests `UNSET`, image not built, a failing
/// container, unparsable output) is [`Competitor::Partial`] with the command and its last line
/// of output.
pub fn run_competitor(root: &Path, tracker: &str, data_dir: &Path, case: &str, threads: &[usize], reps: usize, warmup: usize) -> Competitor {
    let Some(image) = image_of(tracker) else { return Competitor::Partial(format!("unknown competitor {tracker:?}")) };
    let probe = |args: &[&str]| std::process::Command::new("docker").args(args).output();
    match probe(&["version", "--format", "{{.Server.Version}}"]) {
        Ok(o) if o.status.success() => {}
        Ok(o) => return Competitor::Partial(format!("`docker version` failed: {}", last_line(&o.stderr))),
        Err(e) => return Competitor::Partial(format!("`docker version` failed: {e}")),
    }
    let pins = std::fs::read_to_string(root.join("docker/pins.env")).map(|t| parse_pins(&t)).unwrap_or_default();
    let unset: Vec<&String> = pins.iter().filter(|(k, v)| k.ends_with("_DIGEST") && (v.as_str() == "UNSET" || v.is_empty())).map(|(k, _)| k).collect();
    if !unset.is_empty() {
        return Competitor::Partial(format!("{image}: docker/pins.env has {unset:?} UNSET; pin the base digests, then `smrt-bench docker-check`"));
    }
    if !probe(&["image", "inspect", image]).is_ok_and(|o| o.status.success()) {
        return Competitor::Partial(format!("`docker image inspect {image}` failed: the image is not built (`smrt-bench docker-check` builds it)"));
    }
    let mut cmd = std::process::Command::new("docker");
    cmd.args(["run", "--rm", "-v"]).arg(format!("{}:/data:ro", data_dir.display())).arg("-e").arg(format!("SMRT_REPS={reps}")).arg("-e").arg(format!("SMRT_WARMUP={warmup}")).arg(image).arg(case);
    cmd.args(threads.iter().map(usize::to_string));
    let shown = format!("docker run --rm -v {}:/data:ro -e SMRT_REPS={reps} -e SMRT_WARMUP={warmup} {image} {case} {}", data_dir.display(), threads.iter().map(usize::to_string).collect::<Vec<_>>().join(" "));
    match cmd.output() {
        Ok(o) if o.status.success() => {
            let mut rows = Vec::new();
            let mut tracks = None;
            for l in String::from_utf8_lossy(&o.stdout).lines().filter(|l| l.starts_with('{')) {
                if l.starts_with("{\"tracks\"") {
                    match serde_json::from_str::<TracksLine>(l) {
                        Ok(t) => tracks = Some(t.tracks.into_iter().map(|t| crate::truth::Track { regions: t.r, chords: t.c }).collect()),
                        Err(e) => return Competitor::Partial(format!("`{shown}` printed a track dump that does not parse: {e}")),
                    }
                    continue;
                }
                match serde_json::from_str::<crate::result::Row>(l) {
                    Ok(mut r) => {
                        r.tracker = tracker.to_string();
                        rows.push(r);
                    }
                    Err(e) => return Competitor::Partial(format!("`{shown}` printed a line that is not a row: {e}")),
                }
            }
            if rows.is_empty() { Competitor::Partial(format!("`{shown}` printed no rows")) } else { Competitor::Rows { rows, tracks } }
        }
        Ok(o) => Competitor::Partial(format!("`{shown}` failed: {}", last_line(&o.stderr))),
        Err(e) => Competitor::Partial(format!("`{shown}` failed: {e}")),
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
        // A copy of the docker tree with the digests unset: docker-check must stop at the pins.
        let tmp = std::env::temp_dir().join(format!("smrt-bench-unset-{}", std::process::id()));
        std::fs::create_dir_all(tmp.join("docker")).unwrap();
        for (_, rel) in IMAGES {
            let to = tmp.join(rel);
            std::fs::create_dir_all(to.parent().unwrap()).unwrap();
            std::fs::copy(root().join(rel), to).unwrap();
        }
        std::fs::write(tmp.join("docker/pins.env"), "MINIFORGE_DIGEST=UNSET\nUBUNTU_DIGEST=UNSET\n").unwrap();
        // The dagmc driver is present, the xdg driver is not: its image stops at the missing COPY
        // source (with the command), the others at the pins.
        std::fs::create_dir_all(tmp.join("drivers/dagmc")).unwrap();
        std::fs::write(tmp.join("drivers/dagmc/dagmc_driver.cpp"), "").unwrap();
        let st = docker_check(&tmp).unwrap();
        assert_eq!(st.len(), 3);
        for (name, s) in &st {
            if name.contains("xdg") {
                assert!(matches!(s, ImageStatus::NoDriver(m) if m.contains("COPY drivers/xdg/")), "{s:?}");
            } else {
                assert!(matches!(s, ImageStatus::Unpinned(_)), "{name}: {s:?}");
            }
        }
        // And the committed pins are real digests (64 hex digits), none UNSET.
        let pins = parse_pins(&std::fs::read_to_string(root().join("docker/pins.env")).unwrap());
        for (k, v) in pins.iter().filter(|(k, _)| k.ends_with("_DIGEST")) {
            assert!(v.len() == 64 && v.bytes().all(|b| b.is_ascii_hexdigit()), "{k}={v}");
        }
        std::fs::remove_dir_all(&tmp).unwrap();
    }
}
