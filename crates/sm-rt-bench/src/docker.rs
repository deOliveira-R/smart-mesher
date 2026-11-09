//! Docker side of the benchmark: lint of the competitor Dockerfiles, `docker-check`, `docker-lock`,
//! image identity and the run of a competitor container.
//!
//! Images are pinned by base-image index digest and source commit. Every pin lives in
//! `docker/pins.env` (single source): the lint fails when a Dockerfile's `ARG` default differs, and
//! `docker-check` passes the pins as build arguments. `UNSET` digests are refused (no digest is
//! invented here). Conda environments are created from an explicit per-architecture lock when one
//! exists (`docker/locks/`), apt versions are recorded, not frozen. Builds and runs name their
//! platform explicitly and refuse an image of another architecture than the host's.

use crate::error::{BenchError, Result};
use crate::result::{DockerVm, ImageIdentity};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// The competitor images: `(image name, Dockerfile path relative to the crate root)`.
pub const IMAGES: [(&str, &str); 3] = [
    ("smrt-bench-dagmc-moab", "docker/dagmc-moab/Dockerfile"),
    ("smrt-bench-dagmc-dd", "docker/dagmc-dd/Dockerfile"),
    ("smrt-bench-xdg", "docker/xdg/Dockerfile"),
];

/// The XDG commit (12 hex digits) that `docker/pins.env` and `docker/xdg/Dockerfile` must carry.
/// A bump edits those two files and this constant; the lint and the pin tests fail for any pair
/// that disagrees (the tests compare `pins.env` with this constant, never with itself).
pub const XDG_PIN: &str = "a5d67e635b96";

/// Pins every Dockerfile must declare as `ARG KEY=<value of docker/pins.env>`:
/// `(Dockerfile, KEY)`. The values are read from `pins.env`, never repeated here.
pub const REQUIRED_PINS: [(&str, &str); 7] = [
    ("docker/dagmc-moab/Dockerfile", "DAGMC_VERSION"),
    ("docker/dagmc-moab/Dockerfile", "MOAB_VERSION"),
    ("docker/dagmc-dd/Dockerfile", "MOAB_VERSION"),
    ("docker/dagmc-dd/Dockerfile", "DAGMC_VERSION"),
    ("docker/dagmc-dd/Dockerfile", "DOUBLE_DOWN_COMMIT"),
    ("docker/xdg/Dockerfile", "XDG_COMMIT"),
    ("docker/xdg/Dockerfile", "MOAB_VERSION"),
];

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

/// The default of `ARG key=default` in a Dockerfile text (`None` when absent or without default).
pub fn arg_default(text: &str, key: &str) -> Option<String> {
    text.lines().map(str::trim).filter_map(|l| l.strip_prefix("ARG ")).find_map(|a| a.trim().strip_prefix(key)?.strip_prefix('=')).map(|v| v.trim().to_string())
}

/// True when the Dockerfile declares `ARG key` (with or without a default).
pub fn declares_arg(text: &str, key: &str) -> bool {
    text.lines().map(str::trim).filter_map(|l| l.strip_prefix("ARG ")).any(|a| {
        let a = a.trim();
        a == key || a.strip_prefix(key).is_some_and(|r| r.starts_with('='))
    })
}

/// True for a 12 digit lowercase hex source pin.
pub fn is_short_commit(s: &str) -> bool {
    s.len() == 12 && s.bytes().all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}

/// Lint every Dockerfile under `root` (the crate directory) and check them against `pins.env`:
/// every required pin is declared with the value of `pins.env`, every other declared pin default
/// equals it, and `*_COMMIT` pins are 12 hex digits.
pub fn lint_all(root: &Path) -> Result<Vec<String>> {
    let pins_path = root.join("docker/pins.env");
    let pins = parse_pins(&std::fs::read_to_string(&pins_path).map_err(|e| BenchError::io(&pins_path, e))?);
    let mut problems = Vec::new();
    for (k, v) in pins.iter().filter(|(k, _)| k.ends_with("_COMMIT")) {
        if !is_short_commit(v) {
            problems.push(format!("docker/pins.env: {k}={v} is not 12 hex digits"));
        }
    }
    for (name, rel) in IMAGES {
        let p = root.join(rel);
        let text = std::fs::read_to_string(&p).map_err(|e| BenchError::io(&p, e))?;
        problems.extend(lint_dockerfile(&text).into_iter().map(|m| format!("{name}: {m}")));
        for (k, v) in pins.iter().filter(|(k, _)| !k.ends_with("_DIGEST")) {
            if let Some(d) = arg_default(&text, k)
                && d != *v
            {
                problems.push(format!("{rel}: ARG {k}={d} differs from docker/pins.env ({v})"));
            }
        }
    }
    for (rel, key) in REQUIRED_PINS {
        let p = root.join(rel);
        let text = std::fs::read_to_string(&p).map_err(|e| BenchError::io(&p, e))?;
        match (pins.get(key), arg_default(&text, key)) {
            (None, _) => problems.push(format!("{rel}: {key} is not in docker/pins.env")),
            (Some(_), None) => problems.push(format!("{rel}: ARG {key}=... not declared (pin must be read from docker/pins.env)")),
            (Some(_), Some(_)) => {} // value equality is checked above
        }
    }
    Ok(problems)
}

/// A build or run platform: Docker's name and conda's.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Platform {
    /// `linux/arm64` or `linux/amd64`.
    pub docker: &'static str,
    /// `linux-aarch64` or `linux-x86_64`.
    pub conda: &'static str,
}

impl Platform {
    /// Platform of a Rust `ARCH` (`aarch64`, `x86_64`) or a Docker architecture (`arm64`, `amd64`).
    pub fn of_arch(arch: &str) -> Option<Self> {
        match arch {
            "aarch64" | "arm64" => Some(Self { docker: "linux/arm64", conda: "linux-aarch64" }),
            "x86_64" | "amd64" => Some(Self { docker: "linux/amd64", conda: "linux-x86_64" }),
            _ => None,
        }
    }

    /// Platform of a Docker `os/arch` string (`None` for a non-Linux or unknown one).
    pub fn of_docker(s: &str) -> Option<Self> {
        Self::of_arch(s.strip_prefix("linux/")?)
    }

    /// The platform of this machine (Linux containers on the host's architecture).
    ///
    /// # Errors
    /// [`BenchError::Unavailable`] on an architecture the images are not built for.
    pub fn host() -> Result<Self> {
        Self::of_arch(std::env::consts::ARCH).ok_or_else(|| BenchError::Unavailable(format!("no linux image platform for architecture {}", std::env::consts::ARCH)))
    }
}

/// Refuse an image whose platform is not the host's (a leftover amd64 image under qemu must not be
/// used for a benchmark row).
///
/// # Errors
/// The refusal message.
pub fn check_image_platform(image: &str, image_platform: &str, host: Platform) -> core::result::Result<(), String> {
    if image_platform == host.docker { Ok(()) } else { Err(format!("{image} is {image_platform} but this host runs {}: mixed-architecture runs are refused (rebuild with `smrt-bench docker-check`)", host.docker)) }
}

/// Refuse more threads than the Docker VM has CPUs; returns the CPU limit to pass (`--cpus`).
///
/// # Errors
/// The refusal message.
pub fn check_cpus(threads: &[usize], vm_ncpu: usize) -> core::result::Result<usize, String> {
    let want = threads.iter().copied().max().unwrap_or(1).max(1);
    if want > vm_ncpu { Err(format!("--threads asks for {want} CPUs but the Docker VM has {vm_ncpu} (raise it in Docker Desktop settings, or lower --threads); not clamped")) } else { Ok(want) }
}

/// The conda lock a build of image `key` (`dagmc-moab`, ...) on `platform` would use.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LockChoice {
    /// `docker/locks/<file>` exists and is passed as `--build-arg LOCKFILE=<file>`.
    Use {
        /// File name inside `docker/locks/`.
        file: String,
    },
    /// No lock for this platform: the build solves against conda-forge.
    Missing {
        /// What `docker-check` prints.
        note: String,
    },
}

/// Pick the lock of exactly `platform`; the lock of another architecture is never looked at.
pub fn select_lock(root: &Path, key: &str, platform: Platform) -> LockChoice {
    let file = format!("{key}.{}.txt", platform.conda);
    if root.join("docker/locks").join(&file).is_file() {
        LockChoice::Use { file }
    } else {
        LockChoice::Missing { note: format!("no lock for {}: will solve and write one (`smrt-bench docker-lock {key}` after the build)", platform.docker) }
    }
}

/// Short key of an image name (`smrt-bench-dagmc-moab` gives `dagmc-moab`).
pub fn key_of(name: &str) -> &str {
    name.strip_prefix("smrt-bench-").unwrap_or(name)
}

/// True for the conda based images (the xdg image is an apt image and has no conda lock).
fn has_conda(name: &str) -> bool {
    !key_of(name).starts_with("xdg")
}

/// The `docker build` argument vector (without the program name) for an image: explicit
/// `--platform`, the pins its Dockerfile declares, the lock of this platform when there is one.
pub fn build_args(root: &Path, name: &str, rel: &str, dockerfile: &str, pins: &BTreeMap<String, String>, platform: Platform) -> (Vec<String>, Option<LockChoice>) {
    let mut a: Vec<String> = ["build", "-q", "--platform", platform.docker, "-t", name, "-f", rel].iter().map(|s| (*s).to_string()).collect();
    for (k, v) in pins.iter().filter(|(k, _)| declares_arg(dockerfile, k)) {
        a.push("--build-arg".into());
        a.push(format!("{k}={v}"));
    }
    let lock = has_conda(name).then(|| select_lock(root, key_of(name), platform));
    if let Some(LockChoice::Use { file }) = &lock {
        a.push("--build-arg".into());
        a.push(format!("LOCKFILE={file}"));
    }
    a.push(".".into());
    (a, lock)
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
    /// Built, on the host platform, and the driver printed its version line.
    Built {
        /// Image id as printed by `docker build -q`.
        id: String,
        /// Identity read from the image.
        identity: Box<ImageIdentity>,
        /// The driver's `--version` line.
        driver_version: String,
        /// Whether the build used a conda lock, or why it did not.
        lock: String,
    },
    /// The build or the version check failed.
    Failed(String),
}

fn docker_ok(args: &[&str]) -> Option<String> {
    std::process::Command::new("docker").args(args).output().ok().filter(|o| o.status.success()).map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
}

/// Run `docker-check`: lint, pins, then build each image on the host platform (D acceptance,
/// never fabricated).
///
/// # Errors
/// Unreadable `pins.env`/Dockerfiles, or a host architecture without a linux platform.
pub fn docker_check(root: &Path) -> Result<Vec<(String, ImageStatus)>> {
    let pins_path = root.join("docker/pins.env");
    let pins = parse_pins(&std::fs::read_to_string(&pins_path).map_err(|e| BenchError::io(&pins_path, e))?);
    let lint = lint_all(root)?;
    let platform = Platform::host()?;
    let mut out = Vec::new();
    let unset: Vec<String> = pins.iter().filter(|(k, v)| k.ends_with("_DIGEST") && (v.as_str() == "UNSET" || v.is_empty())).map(|(k, _)| k.clone()).collect();
    // Probed only once lint and pins pass: an unpinned tree never touches Docker.
    let mut docker: Option<bool> = None;
    for (name, rel) in IMAGES {
        let mine: Vec<String> = lint.iter().filter(|m| m.starts_with(name) || m.starts_with(rel) || m.starts_with("docker/pins.env")).cloned().collect();
        let driver = if name.contains("xdg") { "drivers/xdg" } else { "drivers/dagmc" };
        let status = if !mine.is_empty() {
            ImageStatus::LintFailed(mine)
        } else if !root.join(driver).join(if name.contains("xdg") { "xdg_driver.cpp" } else { "dagmc_driver.cpp" }).is_file() {
            ImageStatus::NoDriver(format!("`COPY {driver}/ /bench/` in {rel} would fail: {driver}/{} does not exist", if name.contains("xdg") { "xdg_driver.cpp" } else { "dagmc_driver.cpp" }))
        } else if !unset.is_empty() {
            ImageStatus::Unpinned(unset.clone())
        } else if !*docker.get_or_insert_with(|| docker_ok(&["version"]).is_some()) {
            ImageStatus::Unavailable("docker version".to_string())
        } else {
            build_image(root, name, rel, &pins, platform)
        };
        out.push((name.to_string(), status));
    }
    // One MOAB for every row: each built image must report the pinned version.
    if let Some(want) = pins.get("MOAB_VERSION") {
        for (name, st) in &mut out {
            if let ImageStatus::Built { identity, .. } = st
                && let Some(why) = moab_mismatch(name, &identity.moab_version, want)
            {
                *st = ImageStatus::Failed(why);
            }
        }
    }
    Ok(out)
}

/// Why an image's MOAB does not match the pin: a different version, or no `moab=` line in its
/// `versions.txt` (an unchecked image is reported, never passed).
fn moab_mismatch(name: &str, have: &str, want: &str) -> Option<String> {
    if have.is_empty() {
        Some(format!("{name}: versions.txt lists no moab= line, so MOAB {want} (docker/pins.env) cannot be checked"))
    } else if have != want {
        Some(format!("{name} has MOAB {have} but docker/pins.env pins {want}"))
    } else {
        None
    }
}

fn build_image(root: &Path, name: &str, rel: &str, pins: &BTreeMap<String, String>, platform: Platform) -> ImageStatus {
    let text = match std::fs::read_to_string(root.join(rel)) {
        Ok(t) => t,
        Err(e) => return ImageStatus::Failed(format!("{rel}: {e}")),
    };
    let (args, lock) = build_args(root, name, rel, &text, pins, platform);
    let shown = format!("docker {}", args.join(" "));
    let lock_note = match &lock {
        Some(LockChoice::Use { file }) => format!("conda lock docker/locks/{file}"),
        Some(LockChoice::Missing { note }) => note.clone(),
        None => "no conda lock (apt image: versions recorded, not frozen)".to_string(),
    };
    match std::process::Command::new("docker").args(&args).current_dir(root).output() {
        Ok(o) if o.status.success() => {
            let id = String::from_utf8_lossy(&o.stdout).trim().to_string();
            let identity = match image_identity(name) {
                Ok(i) => i,
                Err(e) => return ImageStatus::Failed(format!("`{shown}` built {id} but its identity could not be read: {e}")),
            };
            if let Err(m) = check_image_platform(name, &identity.platform, platform) {
                return ImageStatus::Failed(m);
            }
            let driver = if name.contains("xdg") { "/bench/xdg_driver" } else { "/bench/dagmc_driver" };
            let driver_version = docker_ok(&["run", "--rm", "--entrypoint", driver, name, "--version"]).unwrap_or_default();
            if driver_version.is_empty() {
                return ImageStatus::Failed(format!("`docker run --rm --entrypoint {driver} {name} --version` printed nothing"));
            }
            ImageStatus::Built { id, identity: Box::new(identity), driver_version, lock: lock_note }
        }
        Ok(o) => {
            // The last line is a pointer to the build dashboard; the cause is in the error lines.
            let text = String::from_utf8_lossy(&o.stderr).to_string();
            let errors: Vec<&str> = text.lines().filter(|l| l.to_ascii_lowercase().contains("error")).collect();
            let why = if errors.is_empty() { text.lines().last().unwrap_or("").to_string() } else { errors[errors.len().saturating_sub(2)..].join(" | ") };
            ImageStatus::Failed(format!("`{shown}` failed: {why}"))
        }
        Err(e) => ImageStatus::Failed(format!("{shown}: {e}")),
    }
}

/// Split the dump of the identity files (`=== name` header lines, then the file) into sections.
pub fn parse_dump(dump: &str) -> BTreeMap<String, String> {
    let mut out: BTreeMap<String, String> = BTreeMap::new();
    let mut cur: Option<String> = None;
    for l in dump.lines() {
        if let Some(n) = l.strip_prefix("=== ") {
            cur = Some(n.trim().to_string());
            out.entry(n.trim().to_string()).or_default();
        } else if let Some(c) = &cur {
            let e = out.entry(c.clone()).or_default();
            e.push_str(l);
            e.push('\n');
        }
    }
    out
}

/// The explicit lock of a `conda list --explicit` output: from the `@EXPLICIT` line on (the
/// comment header is dropped), or `None` when there is none.
pub fn lock_text(explicit: &str) -> Option<String> {
    let at = explicit.lines().position(|l| l.trim() == "@EXPLICIT")?;
    let mut t: String = explicit.lines().skip(at).map(|l| format!("{l}\n")).collect();
    if !t.ends_with('\n') {
        t.push('\n');
    }
    Some(t)
}

/// Build an [`ImageIdentity`] from `docker image inspect --format '{{.Id}}|{{.Os}}/{{.Architecture}}|{{join .RepoDigests ","}}'`
/// and the dump of `/bench/{base-index-digest,source-commits,conda-explicit,versions}.txt`
/// (sections `=== file`).
///
/// # Errors
/// [`BenchError::Format`] when the inspect line is malformed.
pub fn identity_from(inspect: &str, dump: &str) -> Result<ImageIdentity> {
    let mut it = inspect.trim().splitn(3, '|');
    let (Some(id), Some(plat), Some(repo)) = (it.next(), it.next(), it.next()) else {
        return Err(BenchError::Format { what: "docker image inspect", why: format!("expected `id|os/arch|digests`, got {inspect:?}") });
    };
    let sec = parse_dump(dump);
    let get = |k: &str| sec.get(k).map_or("", String::as_str);
    let versions = get("versions.txt");
    let moab = versions.lines().find_map(|l| l.strip_prefix("moab=")).map(|v| v.split('=').next().unwrap_or("").trim().to_string()).unwrap_or_default();
    Ok(ImageIdentity {
        image_id: id.to_string(),
        repo_digest: repo.split(',').next().unwrap_or("").to_string(),
        base_index_digest: get("base-index-digest.txt").trim().to_string(),
        platform: plat.to_string(),
        source_commits: get("source-commits.txt").lines().map(str::trim).filter(|l| !l.is_empty()).map(str::to_string).collect(),
        conda_lock_sha256: lock_text(get("conda-explicit.txt")).map(|t| crate::rng::sha256_hex(t.as_bytes())).unwrap_or_default(),
        versions_sha256: if versions.is_empty() { String::new() } else { crate::rng::sha256_hex(versions.as_bytes()) },
        moab_version: moab,
    })
}

/// Read the identity of a built image.
///
/// # Errors
/// [`BenchError::Unavailable`] when Docker or the image does not answer. An image built before the
/// identity files existed yields an identity with id and platform only.
pub fn image_identity(image: &str) -> Result<ImageIdentity> {
    let inspect = docker_ok(&["image", "inspect", "--format", "{{.Id}}|{{.Os}}/{{.Architecture}}|{{join .RepoDigests \",\"}}", image]).ok_or_else(|| BenchError::Unavailable(format!("`docker image inspect {image}` failed")))?;
    let script = "for f in base-index-digest.txt source-commits.txt conda-explicit.txt versions.txt; do echo \"=== $f\"; cat /bench/$f 2>/dev/null || true; done";
    let dump = docker_ok(&["run", "--rm", "--entrypoint", "sh", image, "-c", script]).ok_or_else(|| BenchError::Unavailable(format!("`docker run --rm --entrypoint sh {image} -c '{script}'` failed")))?;
    identity_from(&inspect, &dump)
}

/// `smrt-bench docker-lock <image>`: write the explicit conda lock of a built image to
/// `docker/locks/<key>.<platform>.txt` (platform of the image itself) and return the path.
///
/// # Errors
/// [`BenchError::Usage`] for an image without conda, [`BenchError::Unavailable`] when the image or
/// its lock is unreadable, [`BenchError::Io`] on write.
pub fn docker_lock(root: &Path, which: &str) -> Result<PathBuf> {
    let key = key_of(which);
    let name = format!("smrt-bench-{key}");
    if !IMAGES.iter().any(|(n, _)| *n == name) || !has_conda(&name) {
        return Err(BenchError::Usage(format!("docker-lock: {which:?} is not a conda image (known: dagmc-moab, dagmc-dd)")));
    }
    let plat = docker_ok(&["image", "inspect", "--format", "{{.Os}}/{{.Architecture}}", &name]).ok_or_else(|| BenchError::Unavailable(format!("`docker image inspect {name}` failed: build it with `smrt-bench docker-check`")))?;
    let p = Platform::of_docker(&plat).ok_or_else(|| BenchError::Unavailable(format!("{name} is {plat}, not a linux image")))?;
    let text = docker_ok(&["run", "--rm", "--entrypoint", "cat", &name, "/bench/conda-explicit.txt"]).ok_or_else(|| BenchError::Unavailable(format!("`docker run --rm --entrypoint cat {name} /bench/conda-explicit.txt` failed")))?;
    let lock = lock_text(&text).ok_or_else(|| BenchError::Unavailable(format!("{name}: /bench/conda-explicit.txt has no @EXPLICIT line")))?;
    let path = root.join("docker/locks").join(format!("{key}.{}.txt", p.conda));
    std::fs::create_dir_all(root.join("docker/locks")).map_err(|e| BenchError::io(root.join("docker/locks"), e))?;
    std::fs::write(&path, lock).map_err(|e| BenchError::io(&path, e))?;
    Ok(path)
}

/// Parse `docker info --format '{{.NCPU}}|{{.MemTotal}}|{{.KernelVersion}}|{{.OperatingSystem}}'`,
/// the server version and the container's first CPU line into a [`DockerVm`].
pub fn parse_vm(info: &str, server_version: &str, container_cpu: &str) -> Option<DockerVm> {
    let mut it = info.trim().splitn(4, '|');
    let ncpu = it.next()?.trim().parse().ok()?;
    let mem_total_bytes = it.next()?.trim().parse().ok()?;
    let kernel = it.next()?.trim().to_string();
    let operating_system = it.next()?.trim().to_string();
    Some(DockerVm { ncpu, mem_total_bytes, kernel, server_version: server_version.trim().to_string(), operating_system, container_cpu: container_cpu.trim().to_string() })
}

/// Detect the Docker VM (`None` when Docker does not answer). `image` is any local image with
/// `sh` and `grep`, used to read the container's own `/proc/cpuinfo`.
pub fn detect_vm(image: Option<&str>) -> Option<DockerVm> {
    let info = docker_ok(&["info", "--format", "{{.NCPU}}|{{.MemTotal}}|{{.KernelVersion}}|{{.OperatingSystem}}"])?;
    let server = docker_ok(&["version", "--format", "{{.Server.Version}}"]).unwrap_or_default();
    let cpu = image
        .and_then(|i| docker_ok(&["run", "--rm", "--entrypoint", "sh", i, "-c", "grep -m1 -E 'model name|Model name|^Hardware|CPU part' /proc/cpuinfo || echo \"no model line in /proc/cpuinfo ($(uname -m))\""]))
        .unwrap_or_else(|| "not read (no image to run)".to_string());
    parse_vm(&info, &server, &cpu)
}

/// The first competitor image that exists locally (to read the container's CPU line).
pub fn any_local_image() -> Option<&'static str> {
    IMAGES.iter().map(|(n, _)| *n).find(|n| docker_ok(&["image", "inspect", "--format", "{{.Id}}", n]).is_some())
}

/// Result of asking a competitor image to run a case.
#[derive(Clone, Debug, PartialEq)]
pub enum Competitor {
    /// Rows printed by the driver (one JSON `Row` per line) and, when the driver printed them,
    /// the tracks of every ray (region ids and chords, in ray order).
    Rows {
        /// The rows.
        rows: Vec<crate::result::Row>,
        /// Id (`sha256:...`) of the image the container ran by; identity of the rows is read from it.
        image_id: String,
        /// Tracks of the W-track run, if the driver dumped them.
        tracks: Option<Vec<crate::truth::Track>>,
        /// Dumps of the other workloads (fires, flight tallies), keyed by workload name
        /// (`w-fire`, `w-fire-locate`, `w-flight`).
        dumps: BTreeMap<String, crate::adapter::Output>,
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

#[derive(serde::Deserialize)]
struct FiresLine {
    fires: Vec<(f64, i32)>,
}

#[derive(serde::Deserialize)]
struct WalkLine {
    walk: Vec<TrackJson>,
}

#[derive(serde::Deserialize)]
struct TallyLine {
    tally: Vec<f64>,
    tally_sq: Vec<f64>,
    blocks: usize,
}

/// What a competitor run does besides W-track (all additive; the default is W-track only).
#[derive(Clone, Debug, PartialEq)]
pub struct CompetitorOpts {
    /// Workloads to run, by driver name: `track`, `fire`, `fire-locate`, `flight`, `walk` (tet
    /// meshes: the xdg driver only, and the case needs `tets.bin`).
    pub workloads: Vec<String>,
    /// Histories of `flight`.
    pub histories: usize,
    /// Seed of the flight streams.
    pub seed: u64,
    /// Regions with cross sections (`flight` only; written to `sigma.txt` in the case directory).
    pub regions: Vec<crate::formats::RegionSpec>,
}

impl Default for CompetitorOpts {
    fn default() -> Self {
        Self { workloads: vec!["track".into()], histories: 10_000, seed: 0x5eed, regions: Vec::new() }
    }
}

/// `sigma.txt` of the flight driver: one `id sigma_t sigma_a` line per region (`{:?}` prints the
/// shortest decimal that round-trips, so the driver reads the same doubles).
pub fn sigma_text(regions: &[crate::formats::RegionSpec]) -> String {
    regions.iter().filter(|r| r.id >= 0).map(|r| format!("{} {:?} {:?}\n", r.id, r.sigma_t, r.sigma_a)).collect()
}

/// Parsed stdout of a driver: the rows, the track dump and the other dumps by workload.
pub type DriverOutput = (Vec<crate::result::Row>, Option<Vec<crate::truth::Track>>, BTreeMap<String, crate::adapter::Output>);

/// Parse the stdout of a driver: rows, the track dump and the dumps that follow a row (a `fires`,
/// `tally` or `walk` line (tet indices and chords per ray, kept as tracks) belongs to the workload of the row printed just before it).
pub fn parse_driver_output(tracker: &str, stdout: &str) -> core::result::Result<DriverOutput, String> {
    let mut rows: Vec<crate::result::Row> = Vec::new();
    let mut tracks = None;
    let mut dumps = BTreeMap::new();
    for l in stdout.lines().filter(|l| l.starts_with('{')) {
        let last = rows.last().map(|r| r.workload.clone());
        if l.starts_with("{\"tracks\"") {
            let t = serde_json::from_str::<TracksLine>(l).map_err(|e| format!("a track dump that does not parse: {e}"))?;
            tracks = Some(t.tracks.into_iter().map(|t| crate::truth::Track { regions: t.r, chords: t.c }).collect());
        } else if l.starts_with("{\"walk\"") {
            let t = serde_json::from_str::<WalkLine>(l).map_err(|e| format!("a walk dump that does not parse: {e}"))?;
            let w = last.ok_or("a walk dump before any row")?;
            let tracks = t.walk.into_iter().map(|t| crate::truth::Track { regions: t.r, chords: t.c }).collect();
            dumps.insert(w, crate::adapter::Output { tracks, ..Default::default() });
        } else if l.starts_with("{\"fires\"") {
            let f = serde_json::from_str::<FiresLine>(l).map_err(|e| format!("a fire dump that does not parse: {e}"))?;
            let w = last.ok_or("a fire dump before any row")?;
            dumps.insert(w, crate::adapter::Output { fires: f.fires, ..Default::default() });
        } else if l.starts_with("{\"tally\"") {
            let t = serde_json::from_str::<TallyLine>(l).map_err(|e| format!("a tally dump that does not parse: {e}"))?;
            let w = last.ok_or("a tally dump before any row")?;
            dumps.insert(w, crate::adapter::Output { tally: t.tally, tally_sq: t.tally_sq, tally_blocks: t.blocks, ..Default::default() });
        } else {
            let mut r = serde_json::from_str::<crate::result::Row>(l).map_err(|e| format!("a line that is not a row: {e}"))?;
            r.tracker = tracker.to_string();
            rows.push(r);
        }
    }
    Ok((rows, tracks, dumps))
}

/// Tracker name (`dagmc-moab`, `dagmc-dd`, `xdg`) to its image name.
pub fn image_of(tracker: &str) -> Option<&'static str> {
    IMAGES.iter().map(|(n, _)| *n).find(|n| n.strip_prefix("smrt-bench-") == Some(tracker))
}

fn last_line(b: &[u8]) -> String {
    String::from_utf8_lossy(b).lines().rev().find(|l| !l.trim().is_empty()).unwrap_or("").to_string()
}

/// Run `tracker`'s image on a case directory (`smcase.json`, `facets.bin`, `rays.bin`, ...,
/// written by `CaseData::write`, mounted read-only at `/data/<case>`): `docker run --rm --platform
/// linux/<host arch> --cpus N -v DIR:/data:ro -e SMRT_REPS=R -e SMRT_WARMUP=W IMAGE <case>
/// <threads>...`, with `N` the largest requested thread count (refused when the Docker VM has
/// fewer CPUs; never clamped). The cores are not pinned: Docker Desktop's VM does not allow it.
/// Each stdout line that starts with `{"tracks"` is the track dump, every other line that starts
/// with `{` is a `Row`. Anything that stops the run (no Docker, digests `UNSET`, image not built,
/// an image of another architecture, too many threads, a failing container, unparsable output) is
/// [`Competitor::Partial`] with the command and its last line of output.
pub fn run_competitor(root: &Path, tracker: &str, data_dir: &Path, case: &str, threads: &[usize], reps: usize, warmup: usize) -> Competitor {
    run_competitor_with(root, tracker, data_dir, case, threads, reps, warmup, &CompetitorOpts::default())
}

/// [`run_competitor`] with workloads: the container also gets `SMRT_WORKLOADS`, `SMRT_HISTORIES`
/// and `SMRT_SEED`, and `flight` needs `opts.regions` (written to `DIR/<case>/sigma.txt` before
/// the container starts). A `fires` or `tally` dump line belongs to the row printed before it.
#[allow(clippy::too_many_arguments)]
pub fn run_competitor_with(root: &Path, tracker: &str, data_dir: &Path, case: &str, threads: &[usize], reps: usize, warmup: usize, opts: &CompetitorOpts) -> Competitor {
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
    // The id is read once, here, and the container runs by id: a rebuild of the tag during the
    // run cannot swap the image under the rows.
    let Some(inspected) = docker_ok(&["image", "inspect", "--format", "{{.Id}}|{{.Os}}/{{.Architecture}}", image]) else {
        return Competitor::Partial(format!("`docker image inspect {image}` failed: the image is not built (`smrt-bench docker-check` builds it)"));
    };
    let (image_id, image_platform) = inspected.split_once('|').map(|(a, b)| (a.to_string(), b.to_string())).unwrap_or_default();
    let host = match Platform::host() {
        Ok(h) => h,
        Err(e) => return Competitor::Partial(format!("{image}: {e}")),
    };
    if !image_id.starts_with("sha256:") {
        return Competitor::Partial(format!("`docker image inspect {image}` printed no image id"));
    }
    if let Err(m) = check_image_platform(image, &image_platform, host) {
        return Competitor::Partial(m);
    }
    let Some(ncpu) = docker_ok(&["info", "--format", "{{.NCPU}}"]).and_then(|s| s.parse::<usize>().ok()) else {
        return Competitor::Partial("`docker info --format {{.NCPU}}` failed: cannot check --threads against the Docker VM".to_string());
    };
    let cpus = match check_cpus(threads, ncpu) {
        Ok(c) => c,
        Err(m) => return Competitor::Partial(format!("{image}: {m}")),
    };
    if opts.workloads.iter().any(|w| !["track", "fire", "fire-locate", "flight", "walk"].contains(&w.as_str())) {
        return Competitor::Partial(format!("{image}: unknown workload in {:?}", opts.workloads));
    }
    if opts.workloads.iter().any(|w| w == "flight") {
        let p = data_dir.join(case).join("sigma.txt");
        if let Err(e) = std::fs::write(&p, sigma_text(&opts.regions)) {
            return Competitor::Partial(format!("{image}: cannot write {}: {e}", p.display()));
        }
    }
    let envs = [("SMRT_REPS", reps.to_string()), ("SMRT_WARMUP", warmup.to_string()), ("SMRT_WORKLOADS", opts.workloads.join(",")), ("SMRT_HISTORIES", opts.histories.to_string()), ("SMRT_SEED", opts.seed.to_string())];
    let mut cmd = std::process::Command::new("docker");
    cmd.args(["run", "--rm", "--platform", host.docker, "--cpus", &cpus.to_string(), "-v"]).arg(format!("{}:/data:ro", data_dir.display()));
    let mut shown = format!("docker run --rm --platform {} --cpus {cpus} -v {}:/data:ro", host.docker, data_dir.display());
    for (k, v) in &envs {
        cmd.arg("-e").arg(format!("{k}={v}"));
        shown.push_str(&format!(" -e {k}={v}"));
    }
    cmd.arg(&image_id).arg(case);
    cmd.args(threads.iter().map(usize::to_string));
    shown.push_str(&format!(" {image_id} ({image}) {case} {}", threads.iter().map(usize::to_string).collect::<Vec<_>>().join(" ")));
    match cmd.output() {
        Ok(o) if o.status.success() => match parse_driver_output(tracker, &String::from_utf8_lossy(&o.stdout)) {
            Ok((rows, _, _)) if rows.is_empty() => Competitor::Partial(format!("`{shown}` printed no rows")),
            Ok((rows, tracks, dumps)) => Competitor::Rows { rows, image_id, tracks, dumps },
            Err(e) => Competitor::Partial(format!("`{shown}` printed {e}")),
        },
        Ok(o) => Competitor::Partial(format!("`{shown}` failed: {}", last_line(&o.stderr))),
        Err(e) => Competitor::Partial(format!("`{shown}` failed: {e}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn moab_pin_check_reports_a_missing_or_wrong_version() {
        assert_eq!(moab_mismatch("i", "5.6.0", "5.6.0"), None);
        assert!(moab_mismatch("i", "5.5.1", "5.6.0").unwrap().contains("5.5.1"));
        assert!(moab_mismatch("i", "", "5.6.0").unwrap().contains("no moab= line"));
    }

    fn root() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
    }

    fn scratch(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("smrt-bench-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn committed_dockerfiles_pass_the_lint() {
        assert_eq!(lint_all(&root()).unwrap(), Vec::<String>::new());
    }

    #[test]
    fn the_xdg_pin_is_the_one_in_pins_env_and_twelve_hex_digits() {
        let pins = parse_pins(&std::fs::read_to_string(root().join("docker/pins.env")).unwrap());
        let xdg = &pins["XDG_COMMIT"];
        assert!(is_short_commit(xdg), "{xdg}");
        assert_eq!(xdg, XDG_PIN);
        assert_eq!(pins["MOAB_VERSION"], "5.6.0");
        let text = std::fs::read_to_string(root().join("docker/xdg/Dockerfile")).unwrap();
        assert_eq!(arg_default(&text, "XDG_COMMIT").as_deref(), Some(xdg.as_str()));
        assert_eq!(arg_default(&text, "MOAB_VERSION").as_deref(), Some("5.6.0"));
    }

    #[test]
    fn lint_catches_a_dockerfile_that_drifts_from_pins_env() {
        let tmp = scratch("drift");
        for (_, rel) in IMAGES {
            let to = tmp.join(rel);
            std::fs::create_dir_all(to.parent().unwrap()).unwrap();
            std::fs::copy(root().join(rel), to).unwrap();
        }
        let pins = std::fs::read_to_string(root().join("docker/pins.env")).unwrap();
        std::fs::write(tmp.join("docker/pins.env"), &pins).unwrap();
        assert!(lint_all(&tmp).unwrap().is_empty());
        let current = format!("XDG_COMMIT={XDG_PIN}");
        assert!(pins.contains(&current), "pins.env does not hold XDG_PIN");
        std::fs::write(tmp.join("docker/pins.env"), pins.replace(&current, "XDG_COMMIT=fec80980ce29")).unwrap();
        let p = lint_all(&tmp).unwrap();
        assert!(p.iter().any(|m| m.contains(&format!("ARG XDG_COMMIT={XDG_PIN} differs from docker/pins.env (fec80980ce29)"))), "{p:?}");
        std::fs::write(tmp.join("docker/pins.env"), pins.replace(&current, "XDG_COMMIT=83efb30")).unwrap();
        assert!(lint_all(&tmp).unwrap().iter().any(|m| m.contains("not 12 hex digits")));
        std::fs::remove_dir_all(&tmp).unwrap();
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
        let tmp = scratch("unset");
        for (_, rel) in IMAGES {
            let to = tmp.join(rel);
            std::fs::create_dir_all(to.parent().unwrap()).unwrap();
            std::fs::copy(root().join(rel), to).unwrap();
        }
        let real = std::fs::read_to_string(root().join("docker/pins.env")).unwrap();
        let unset: String = real.lines().map(|l| if l.contains("_DIGEST=") { format!("{}=UNSET\n", l.split('=').next().unwrap_or("")) } else { format!("{l}\n") }).collect();
        std::fs::write(tmp.join("docker/pins.env"), unset).unwrap();
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
        let pins = parse_pins(&real);
        for (k, v) in pins.iter().filter(|(k, _)| k.ends_with("_DIGEST")) {
            assert!(v.len() == 64 && v.bytes().all(|b| b.is_ascii_hexdigit()), "{k}={v}");
        }
        std::fs::remove_dir_all(&tmp).unwrap();
    }

    #[test]
    fn the_lock_of_the_build_platform_is_chosen_and_the_other_is_never_read() {
        let tmp = scratch("locks");
        std::fs::create_dir_all(tmp.join("docker/locks")).unwrap();
        let arm = Platform::of_arch("aarch64").unwrap();
        let x86 = Platform::of_arch("x86_64").unwrap();
        assert_eq!((arm.docker, arm.conda, x86.docker, x86.conda), ("linux/arm64", "linux-aarch64", "linux/amd64", "linux-x86_64"));
        assert_eq!(Platform::of_docker("linux/amd64"), Some(x86));
        assert_eq!(Platform::of_docker("windows/amd64"), None);
        // Only an aarch64 lock: arm picks it, amd64 reports "will solve and write one", not an error.
        std::fs::write(tmp.join("docker/locks/dagmc-moab.linux-aarch64.txt"), "@EXPLICIT\n").unwrap();
        assert_eq!(select_lock(&tmp, "dagmc-moab", arm), LockChoice::Use { file: "dagmc-moab.linux-aarch64.txt".into() });
        match select_lock(&tmp, "dagmc-moab", x86) {
            LockChoice::Missing { note } => assert!(note.starts_with("no lock for linux/amd64: will solve and write one"), "{note}"),
            other => panic!("{other:?}"),
        }
        // A directory that only has the x86_64 lock: the aarch64 build does not use it.
        std::fs::rename(tmp.join("docker/locks/dagmc-moab.linux-aarch64.txt"), tmp.join("docker/locks/dagmc-moab.linux-x86_64.txt")).unwrap();
        assert!(matches!(select_lock(&tmp, "dagmc-moab", arm), LockChoice::Missing { .. }));
        assert_eq!(select_lock(&tmp, "dagmc-moab", x86), LockChoice::Use { file: "dagmc-moab.linux-x86_64.txt".into() });
        // The build command carries the platform and the lock of that platform only.
        let pins = parse_pins(&std::fs::read_to_string(root().join("docker/pins.env")).unwrap());
        let df = std::fs::read_to_string(root().join("docker/dagmc-moab/Dockerfile")).unwrap();
        let (a, l) = build_args(&tmp, "smrt-bench-dagmc-moab", "docker/dagmc-moab/Dockerfile", &df, &pins, x86);
        let a = a.join(" ");
        assert!(a.contains("--platform linux/amd64") && a.contains("LOCKFILE=dagmc-moab.linux-x86_64.txt") && a.contains("MINIFORGE_DIGEST="), "{a}");
        assert!(matches!(l, Some(LockChoice::Use { .. })));
        let (a, _) = build_args(&tmp, "smrt-bench-dagmc-moab", "docker/dagmc-moab/Dockerfile", &df, &pins, arm);
        let a = a.join(" ");
        assert!(a.contains("--platform linux/arm64") && !a.contains("LOCKFILE") && !a.contains("x86_64"), "{a}");
        // Only the pins a Dockerfile declares are passed; xdg has no conda lock.
        let dx = std::fs::read_to_string(root().join("docker/xdg/Dockerfile")).unwrap();
        let (ax, lx) = build_args(&tmp, "smrt-bench-xdg", "docker/xdg/Dockerfile", &dx, &pins, arm);
        let ax = ax.join(" ");
        assert!(ax.contains(&format!("XDG_COMMIT={XDG_PIN}")) && ax.contains("MOAB_VERSION=5.6.0") && ax.contains("UBUNTU_DIGEST=") && !ax.contains("MINIFORGE") && !ax.contains("LOCKFILE"), "{ax}");
        assert!(lx.is_none());
        std::fs::remove_dir_all(&tmp).unwrap();
    }

    #[test]
    fn mixed_architecture_images_and_too_many_threads_are_refused() {
        let arm = Platform::of_arch("arm64").unwrap();
        assert!(check_image_platform("img", "linux/arm64", arm).is_ok());
        let m = check_image_platform("img", "linux/amd64", arm).unwrap_err();
        assert!(m.contains("mixed-architecture") && m.contains("linux/amd64") && m.contains("linux/arm64"), "{m}");
        assert_eq!(check_cpus(&[1, 4], 10), Ok(4));
        assert_eq!(check_cpus(&[4], 4), Ok(4));
        let m = check_cpus(&[1, 16], 10).unwrap_err();
        assert!(m.contains("16") && m.contains("10") && m.contains("not clamped"), "{m}");
    }

    #[test]
    fn identity_is_read_from_the_inspect_line_and_the_recorded_files() {
        let dump = "=== base-index-digest.txt\n937d\n=== source-commits.txt\ndouble-down=437ad209eac5aaaa\nDAGMC=v3.2.4 (source build)\n=== conda-explicit.txt\n# header\n# platform: linux-aarch64\n@EXPLICIT\nhttps://x/moab-5.6.0.conda#abc\n=== versions.txt\nmoab=5.6.0=nompi_0\neigen=3.4.0=h0\n";
        let id = identity_from("sha256:feed|linux/arm64|repo@sha256:1,repo@sha256:2\n", dump).unwrap();
        assert_eq!((id.image_id.as_str(), id.platform.as_str(), id.repo_digest.as_str()), ("sha256:feed", "linux/arm64", "repo@sha256:1"));
        assert_eq!(id.base_index_digest, "937d");
        assert_eq!(id.source_commits, ["double-down=437ad209eac5aaaa", "DAGMC=v3.2.4 (source build)"]);
        assert_eq!(id.moab_version, "5.6.0");
        // The lock hash covers the lock without its comment header, so it equals the file docker-lock writes.
        assert_eq!(id.conda_lock_sha256, crate::rng::sha256_hex(b"@EXPLICIT\nhttps://x/moab-5.6.0.conda#abc\n"));
        assert_eq!(id.versions_sha256, crate::rng::sha256_hex(b"moab=5.6.0=nompi_0\neigen=3.4.0=h0\n"));
        // An apt image has no lock; a local image has no repo digest.
        let apt = identity_from("sha256:a|linux/arm64|", "=== conda-explicit.txt\nnot a conda image\n=== versions.txt\nmoab=5.6.0\nlibc6\t2.39\n").unwrap();
        assert_eq!((apt.conda_lock_sha256.as_str(), apt.repo_digest.as_str(), apt.moab_version.as_str()), ("", "", "5.6.0"));
        assert!(identity_from("garbage", "").is_err());
        assert_eq!(lock_text("# x\n@EXPLICIT\nu1\nu2").as_deref(), Some("@EXPLICIT\nu1\nu2\n"));
        assert_eq!(lock_text("no lock"), None);
    }

    #[test]
    fn the_docker_vm_block_is_parsed() {
        let vm = parse_vm("10|16745758720|7.0.14-linuxkit|Docker Desktop\n", "29.0.1\n", "CPU part\t: 0x022\n").unwrap();
        assert_eq!((vm.ncpu, vm.mem_total_bytes, vm.kernel.as_str(), vm.operating_system.as_str()), (10, 16_745_758_720, "7.0.14-linuxkit", "Docker Desktop"));
        assert_eq!((vm.server_version.as_str(), vm.container_cpu.as_str()), ("29.0.1", "CPU part\t: 0x022"));
        assert!(parse_vm("x|y", "", "").is_none());
    }

    #[test]
    fn docker_lock_refuses_an_image_without_conda() {
        assert!(matches!(docker_lock(&root(), "xdg"), Err(BenchError::Usage(_))));
        assert!(matches!(docker_lock(&root(), "smrt-bench-nothing"), Err(BenchError::Usage(_))));
    }
}
