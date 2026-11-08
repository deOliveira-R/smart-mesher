//! Command line of `smrt-bench`: `gen`, `run`, `report`, `self-test`, `docker-check`, `docker-lock`.
//!
//! Options are `--key value` pairs and bare flags; there is no argument-parser dependency.

use crate::adapter::{Adapter, WorkInput, Workload, stub_adapter};
use crate::cases::{self, Case, Scale};
use crate::error::{BenchError, Result};
use crate::formats::{Facets, Rays};
use crate::result::{BenchEnv, Env, ResultFile, write_result};
use crate::runner::{RunConfig, run_case};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

struct Args {
    cmd: String,
    opts: BTreeMap<String, String>,
    flags: Vec<String>,
    /// Bare words after the command (only `docker-lock IMAGE` takes one).
    positional: Vec<String>,
}

const FLAGS: [&str; 4] = ["check", "print", "dry-run", "help"];

fn parse(args: &[String]) -> Result<Args> {
    let cmd = args.first().cloned().ok_or_else(|| BenchError::Usage(usage()))?;
    let mut opts = BTreeMap::new();
    let mut flags = Vec::new();
    let mut positional = Vec::new();
    let mut i = 1;
    while i < args.len() {
        let a = &args[i];
        let Some(key) = a.strip_prefix("--") else {
            positional.push(a.clone());
            i += 1;
            continue;
        };
        if FLAGS.contains(&key) {
            flags.push(key.to_string());
            i += 1;
        } else {
            let v = args.get(i + 1).ok_or_else(|| BenchError::Usage(format!("--{key} needs a value")))?;
            opts.insert(key.to_string(), v.clone());
            i += 2;
        }
    }
    if cmd != "docker-lock" && !positional.is_empty() {
        return Err(BenchError::Usage(format!("unexpected argument {:?}", positional[0])));
    }
    Ok(Args { cmd, opts, flags, positional })
}

fn usage() -> String {
    "smrt-bench <gen|run|report|self-test|docker-check|docker-lock> [options]\n  gen    [--scale full|reduced|tiny] [--cases P1,H1] [--rays N] [--out DIR] [--check|--print] [--expected FILE] [--legacy-data DIR]\n  run    [--cases P1,H1] [--trackers stub,smrt,dagmc-moab,dagmc-dd,xdg] [--scale S] [--threads 1,4] [--reps 7] [--warmup 3] [--rays N] [--histories N] [--flight-cells N] [--out DIR] [--model STEP --sag X [--dry-run]]\n  report --results DIR [--out FILE]\n  self-test\n  docker-check   (build the competitor images for this host's platform, print their identity)\n  docker-lock IMAGE   (write docker/locks/<image>.<platform>.txt from a built conda image)".to_string()
}

fn crate_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn num<T: std::str::FromStr>(a: &Args, key: &str, default: T) -> Result<T> {
    match a.opts.get(key) {
        None => Ok(default),
        Some(v) => v.parse().map_err(|_| BenchError::Usage(format!("--{key}: cannot parse {v:?}"))),
    }
}

fn scale_of(a: &Args) -> Result<Scale> {
    let name = a.opts.get("scale").map_or("reduced", String::as_str);
    let mut s = Scale::parse(name).ok_or_else(|| BenchError::Usage(format!("unknown scale {name:?}")))?;
    if let Some(n) = a.opts.get("rays") {
        let n: usize = n.parse().map_err(|_| BenchError::Usage("--rays".into()))?;
        s.rays = n;
        s.f1_rays = n;
    }
    Ok(s)
}

fn cases_of(a: &Args) -> Result<Vec<Case>> {
    match a.opts.get("cases") {
        None => Ok(Case::ALL.to_vec()),
        Some(list) => list.split(',').map(|n| Case::parse(n.trim()).ok_or_else(|| BenchError::Usage(format!("unknown case {n:?}")))).collect(),
    }
}

fn git_sha() -> String {
    std::process::Command::new("git").args(["rev-parse", "HEAD"]).current_dir(crate_dir()).output().ok().filter(|o| o.status.success()).map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string()).unwrap_or_default()
}

/// Entry point: returns the process exit code (0 ok, 1 failure, 2 unavailable or unblessed).
pub fn main_with(args: &[String]) -> Result<i32> {
    let a = parse(args)?;
    if a.flags.iter().any(|f| f == "help") {
        println!("{}", usage());
        return Ok(0);
    }
    match a.cmd.as_str() {
        "gen" => cmd_gen(&a),
        "run" => run(&a),
        "report" => report(&a),
        "self-test" => {
            let checks = crate::selftest::self_test()?;
            for c in &checks {
                println!("{} {}: {}", if c.ok { "ok  " } else { "FAIL" }, c.name, c.detail);
            }
            Ok(i32::from(checks.iter().any(|c| !c.ok)))
        }
        "docker-check" => {
            let mut code = 0;
            for (name, st) in crate::docker::docker_check(&crate_dir())? {
                print_image_status(&name, &st);
                if !matches!(st, crate::docker::ImageStatus::Built { .. }) {
                    code = 2;
                }
            }
            Ok(code)
        }
        "docker-lock" => {
            let which = a.positional.first().ok_or_else(|| BenchError::Usage("docker-lock needs an image (dagmc-moab or dagmc-dd)".into()))?;
            let path = crate::docker::docker_lock(&crate_dir(), which)?;
            println!("wrote {}", path.display());
            Ok(0)
        }
        other => Err(BenchError::Usage(format!("unknown command {other:?}\n{}", usage()))),
    }
}

fn print_image_status(name: &str, st: &crate::docker::ImageStatus) {
    match st {
        crate::docker::ImageStatus::Built { id, identity, driver_version, lock } => {
            println!("{name}: built");
            println!("  id               {id}");
            println!("  platform         {}", identity.platform);
            println!("  base index       sha256:{}", identity.base_index_digest);
            println!("  source commits   {}", identity.source_commits.join("; "));
            println!("  moab             {}", if identity.moab_version.is_empty() { "not listed" } else { &identity.moab_version });
            println!("  conda lock       {}", if identity.conda_lock_sha256.is_empty() { "none (apt image)".to_string() } else { format!("sha256:{}", identity.conda_lock_sha256) });
            println!("  versions.txt     sha256:{}", identity.versions_sha256);
            println!("  build            {lock}");
            println!("  driver --version {driver_version}");
        }
        other => println!("{name}: {other:?}"),
    }
}

fn cmd_gen(a: &Args) -> Result<i32> {
    let scale = scale_of(a)?;
    let cases = cases_of(a)?;
    let expected_path = a.opts.get("expected").map_or_else(|| crate_dir().join("expected_sha256.txt"), PathBuf::from);
    let expected = if a.flags.iter().any(|f| f == "check") { Some(std::fs::read_to_string(&expected_path).map_err(|e| BenchError::io(&expected_path, e))?) } else { None };
    let out_dir = a.opts.get("out").map(PathBuf::from);
    let mut code = 0;
    for case in cases {
        let data = cases::build(case, &scale)?;
        if let Some(dir) = &out_dir {
            data.write(dir)?;
        }
        if a.flags.iter().any(|f| f == "print") {
            for l in cases::expected_lines(&scale, &data)? {
                println!("{l}");
            }
        }
        if let Some(text) = &expected {
            let rep = cases::check_against(text, &scale, &data)?;
            println!("{} {}: {} matched, {} mismatched, {} unblessed for this platform", scale.name, case.name(), rep.matched, rep.mismatched.len(), rep.unblessed.len());
            for m in &rep.mismatched {
                println!("  MISMATCH {m}");
            }
            for m in &rep.unblessed {
                println!("  unblessed {m}");
            }
            if !rep.ok() {
                code = 1;
            } else if !rep.unblessed.is_empty() && code == 0 {
                code = 2;
            }
        }
        if let Some(dir) = a.opts.get("legacy-data")
            && matches!(case, Case::C1 | Case::C2 | Case::C3)
        {
            verify_legacy(Path::new(dir), case, &data)?;
            println!("{}: equals the W3a files in {dir}", case.name());
        }
    }
    Ok(code)
}

fn verify_legacy(dir: &Path, case: Case, data: &cases::CaseData) -> Result<()> {
    let d = dir.join(case.name());
    let fpath = d.join("facets.bin");
    let rpath = d.join("rays.bin");
    let f = Facets::from_bytes(&std::fs::read(&fpath).map_err(|e| BenchError::io(&fpath, e))?)?;
    let r = Rays::from_bytes(&std::fs::read(&rpath).map_err(|e| BenchError::io(&rpath, e))?)?;
    if f != data.facets || r != data.rays.truncated(r.len()) || r.len() != data.rays.len() {
        return Err(BenchError::Check(format!("{} differs from the W3a files (use --rays {} to match the W3a count)", case.name(), r.len())));
    }
    Ok(())
}

/// Trackers `run --trackers` knows: the stub, sm-rt and the Docker competitors.
const TRACKERS: [&str; 5] = ["stub", "smrt", "dagmc-moab", "dagmc-dd", "xdg"];

fn run(a: &Args) -> Result<i32> {
    let scale = scale_of(a)?;
    let trackers: Vec<&str> = a.opts.get("trackers").map_or("stub", String::as_str).split(',').map(str::trim).collect();
    for t in &trackers {
        if !TRACKERS.contains(t) {
            return Err(BenchError::Usage(format!("unknown tracker {t:?} (known: {})", TRACKERS.join(", "))));
        }
    }
    let threads: Vec<usize> = a.opts.get("threads").map_or("1", String::as_str).split(',').map(|s| s.trim().parse().map_err(|_| BenchError::Usage("--threads".into()))).collect::<Result<_>>()?;
    let cfg = RunConfig { reps: num(a, "reps", 7)?, warmup: num(a, "warmup", 3)?, threads, histories: num(a, "histories", 10_000)?, ..RunConfig::default() };
    let out_dir = a.opts.get("out").map_or_else(|| crate_dir().join("results"), PathBuf::from);
    if let Some(model) = a.opts.get("model") {
        return run_model(a, Path::new(model), &cfg, &out_dir);
    }
    let flight_cells: usize = num(a, "flight-cells", 24)?;
    let mut code = 0;
    for case in cases_of(a)? {
        let data = cases::build(case, &scale)?;
        let env = Env::detect();
        let mut file = ResultFile::new(&data.case, &data.manifest.scale, env.clone());
        let mut failed = 0usize;
        let mut absorb = |f: ResultFile, ag: Vec<crate::compare::Agreement>| {
            failed += ag.iter().map(|g| g.failed).sum::<usize>() + f.rows.iter().filter(|r| r.workload == "w-flight").map(|r| r.agreement.failed).sum::<usize>();
            file.rows.extend(f.rows);
        };
        if trackers.contains(&"stub") {
            match stub_adapter(&data) {
                Ok(s) => {
                    let mut adapters: Vec<Box<dyn Adapter>> = vec![Box::new(s)];
                    let (f, ag) = run_case(&data, &mut adapters, "surface", &cfg, env.clone())?;
                    absorb(f, ag);
                }
                Err(e) => {
                    println!("{}: stub skipped ({e})", case.name());
                    code = code.max(2);
                }
            }
        }
        if trackers.contains(&"smrt") {
            let mut rows = crate::smrt::smrt_rows(&data, &scale, flight_cells);
            for n in &rows.notes {
                println!("{n}");
                file.partial.push(n.clone());
            }
            let geometric = RunConfig { workloads: vec![Workload::Track, Workload::Fire, Workload::FireLocate], ..cfg.clone() };
            let flight = RunConfig { workloads: vec![Workload::Flight], ..cfg.clone() };
            for (adapters, c) in [(&mut rows.geometric, &geometric), (&mut rows.exact_flight, &flight), (&mut rows.mesh_flight, &flight)] {
                if adapters.is_empty() {
                    continue;
                }
                let (f, ag) = run_case(&data, adapters, "surface", c, env.clone())?;
                absorb(f, ag);
            }
        }
        let competitors: Vec<&&str> = trackers.iter().filter(|t| crate::docker::image_of(t).is_some()).collect();
        if !competitors.is_empty() {
            let dir = std::env::temp_dir().join(format!("smrt-bench-case-{}", std::process::id()));
            data.write(&dir)?;
            // The facet-twin run of sm-rt is the reference every competitor holding the same
            // triangles is judged against; the analytic truth gives the geom_error column.
            let reference = crate::smrt::facets_adapter(&data).and_then(|mut a| {
                let w = WorkInput { rays: &data.rays, regions: &data.manifest.regions, threads: 1, histories: 0, seed: cfg.seed };
                a.run(Workload::Track, &w).map(|o| o.tracks)
            });
            let mut dumps: Vec<(String, Vec<crate::truth::Track>)> = Vec::new();
            for t in competitors {

                match crate::docker::run_competitor(&crate_dir(), t, &dir, &data.case, &cfg.threads, cfg.reps, cfg.warmup) {
                    crate::docker::Competitor::Rows { mut rows, tracks } => {
                        if let Some(tracks) = tracks {
                            let (geom, _) = crate::runner::geom_error(&data, &tracks, data.rays.len())?;
                            let agree = match &reference {
                                Ok(r) => Some(crate::compare::compare(&tracks, r, data.manifest.diag, data.manifest.chord_tol)?),
                                Err(_) => None,
                            };
                            for r in rows.iter_mut().filter(|r| r.workload == "w-track") {
                                r.geom_error = geom;
                                if let Some(a) = &agree {
                                    r.agreement = crate::result::AgreementSummary::from(a);
                                    failed += a.failed;
                                }
                            }
                            if let Err(e) = &reference {
                                file.partial.push(format!("{t}: agreement with the sm-rt facet twin not checked: {e}"));
                            }
                            dumps.push((t.to_string(), tracks));
                        }
                        file.rows.extend(rows);
                        // Identity of the image that produced these rows (outside `result_hash`).
                        match crate::docker::image_of(t).map(crate::docker::image_identity) {
                            Some(Ok(id)) => {
                                if file.image_digest.is_empty() {
                                    file.image_digest = id.image_id.clone();
                                }
                                file.images.insert(t.to_string(), id);
                            }
                            Some(Err(e)) => file.partial.push(format!("{t}: image identity not recorded: {e}")),
                            None => {}
                        }
                        file.notes.push(format!("{t}: cores not pinned (Docker Desktop VM does not allow it); container limited with --cpus {}", cfg.threads.iter().copied().max().unwrap_or(1)));
                    }
                    crate::docker::Competitor::Partial(why) => {
                        println!("{}: partial: {t}: {why}", case.name());
                        file.partial.push(format!("{t}: {why}"));
                        code = code.max(2);
                    }
                }
            }
            if let [(na, ta), (nb, tb)] = &dumps[..] {
                let equal = ta.iter().zip(tb).filter(|(x, y)| x.regions == y.regions).count();
                file.notes.push(format!("{na} and {nb}: {equal} of {} rays have equal region sequences", ta.len().min(tb.len())));
            }
            // Best effort: the temporary copy of the case files is not needed any more.
            let _ = std::fs::remove_dir_all(&dir);
        }
        file.git_sha = git_sha();
        file.seal()?;
        let path = write_result(&out_dir, &file)?;
        println!("{}: {} rows, {failed} disagreements, result_hash {} -> {}", case.name(), file.rows.len(), file.result_hash, path.display());
        if failed > 0 {
            code = 1;
        }
    }
    let env_path = out_dir.join("bench-env.json");
    if !env_path.exists() {
        let mut be = BenchEnv::detect();
        if trackers.iter().any(|t| crate::docker::image_of(t).is_some()) {
            be.docker = crate::docker::detect_vm(crate::docker::any_local_image());
        }
        std::fs::write(&env_path, serde_json::to_vec_pretty(&be)?).map_err(|e| BenchError::io(&env_path, e))?;
    }
    Ok(code)
}

fn run_model(a: &Args, model: &Path, cfg: &RunConfig, out_dir: &Path) -> Result<i32> {
    let bytes = std::fs::read(model).map_err(|e| BenchError::io(model, e))?;
    let sag: f64 = num(a, "sag", 1e-2)?;
    let rays: usize = num(a, "rays", 1000)?;
    let data = cases::build_model(&bytes, sag, rays)?;
    println!("model: {} triangles, {} rays, sag {sag}, threads {:?}", data.manifest.triangles, data.manifest.rays, cfg.threads);
    if a.flags.iter().any(|f| f == "dry-run") {
        println!("dry run: nothing written");
        return Ok(0);
    }
    // No tracker in this build can run a general model (stub: spheres and boxes only), so the
    // result has no rows; it records the case only. It is written outside the repository only.
    let mut file = ResultFile::new("model", "model", Env::detect());
    file.model_run = true;
    file.git_sha = git_sha();
    file.seal()?;
    let path = write_result(out_dir, &file)?;
    println!("wrote {}", path.display());
    Ok(0)
}

fn report(a: &Args) -> Result<i32> {
    let dir = a.opts.get("results").ok_or_else(|| BenchError::Usage("report needs --results DIR".into()))?;
    let mut files = Vec::new();
    let mut paths: Vec<PathBuf> = std::fs::read_dir(dir).map_err(|e| BenchError::io(dir, e))?.filter_map(|e| e.ok()).map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "json") && p.file_name().is_some_and(|n| n != "bench-env.json")).collect();
    paths.sort();
    for p in paths {
        let text = std::fs::read_to_string(&p).map_err(|e| BenchError::io(&p, e))?;
        files.push(ResultFile::from_json(&text)?);
    }
    let env_path = Path::new(dir).join("bench-env.json");
    let vm = std::fs::read_to_string(&env_path).ok().and_then(|t| serde_json::from_str::<BenchEnv>(&t).ok()).and_then(|b| b.docker);
    let md = crate::report::render_with_docker(&files, vm.as_ref())?;
    match a.opts.get("out") {
        Some(o) => std::fs::write(o, md).map_err(|e| BenchError::io(o, e))?,
        None => print!("{md}"),
    }
    Ok(0)
}
