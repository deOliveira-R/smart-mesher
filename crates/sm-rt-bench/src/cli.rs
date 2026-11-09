//! Command line of `smrt-bench`: `gen`, `run`, `report`, `self-test`, `docker-check`, `docker-lock`.
//!
//! Options are `--key value` pairs and bare flags; there is no argument-parser dependency.

use crate::adapter::{Adapter, WorkInput, Workload, stub_adapter};
use crate::cases::{self, Case, Scale};
use crate::error::{BenchError, Result};
use crate::formats::{Facets, Rays};
use crate::result::{AgreementSummary, BenchEnv, Env, ResultFile, write_result};
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
    "smrt-bench <gen|run|report|self-test|docker-check|docker-lock> [options]\n  gen    [--scale full|reduced|tiny] [--cases P1,H1] [--rays N] [--out DIR] [--check|--print] [--expected FILE] [--legacy-data DIR]\n  run    [--cases P1,H1] [--trackers stub,smrt,dagmc-moab,dagmc-dd,xdg] [--scale S] [--threads 1,4] [--reps 7] [--warmup 3] [--rays N] [--histories N] [--seed S] [--workloads track,fire,fire-locate,flight,walk (competitors; walk: tet cases, sm-rt and xdg)] [--flight-cells N] [--out DIR] [--model STEP --sag X [--dry-run]]\n  report --results DIR [--out FILE]\n  self-test\n  docker-check   (build the competitor images for this host's platform, print their identity)\n  docker-lock IMAGE   (write docker/locks/<image>.<platform>.txt from a built conda image)".to_string()
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
    let cfg = RunConfig { reps: num(a, "reps", 7)?, warmup: num(a, "warmup", 3)?, threads, histories: num(a, "histories", 10_000)?, seed: num(a, "seed", RunConfig::default().seed)?, ..RunConfig::default() };
    let workloads: Vec<&str> = a.opts.get("workloads").map_or("track,fire,fire-locate,flight,walk", String::as_str).split(',').map(str::trim).collect();
    for w in &workloads {
        if !["track", "fire", "fire-locate", "flight", "walk"].contains(w) {
            return Err(BenchError::Usage(format!("unknown workload {w:?} (known: track, fire, fire-locate, flight, walk)")));
        }
    }
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
            // W-walk: the tet mesh walked cell by cell (cases with a tet mesh), the reference of XDG's `segments`.
            if let Some(tets) = data.tets.as_ref().filter(|_| workloads.contains(&"walk")) {
                match crate::smrt::walk_adapter(tets) {
                    Ok(a) => {
                        let mut adapters: Vec<Box<dyn Adapter>> = vec![Box::new(a)];
                        let walk = RunConfig { workloads: vec![Workload::Walk], ..cfg.clone() };
                        let (f, ag) = run_case(&data, &mut adapters, "surface", &walk, env.clone())?;
                        absorb(f, ag);
                    }
                    Err(e) => {
                        println!("{}: smrt walk not run: {e}", case.name());
                        file.partial.push(format!("{}: smrt walk not run: {e}", case.name()));
                    }
                }
            }
        }
        let competitors: Vec<&&str> = trackers.iter().filter(|t| crate::docker::image_of(t).is_some()).collect();
        if !competitors.is_empty() {
            // Removed on every exit path (an early `?` included).
            struct RemoveOnDrop(PathBuf);
            impl Drop for RemoveOnDrop {
                fn drop(&mut self) {
                    let _ = std::fs::remove_dir_all(&self.0);
                }
            }
            let dir = std::env::temp_dir().join(format!("smrt-bench-case-{}", std::process::id()));
            let _guard = RemoveOnDrop(dir.clone());
            data.write(&dir)?;
            // The facet-twin run of sm-rt is the reference every competitor holding the same
            // triangles is judged against; the analytic truth gives the geom_error column.
            let reference = facet_reference(&data, &cfg, &workloads);
            let mut dumps: Vec<(String, Vec<crate::truth::Track>)> = Vec::new();
            for t in competitors {
                // `walk` is a tet-mesh workload only the xdg driver has; no other tracker is asked for it.
                let mine: Vec<String> = workloads.iter().filter(|w| **w != "walk" || (**t == *"xdg" && data.tets.is_some())).map(|w| w.to_string()).collect();
                if mine.is_empty() {
                    file.notes.push(format!("{t}: no requested workload applies to {}", case.name()));
                    continue;
                }
                let opts = crate::docker::CompetitorOpts { workloads: mine, histories: cfg.histories, seed: cfg.seed, regions: data.manifest.regions.clone() };
                match crate::docker::run_competitor_with(&crate_dir(), t, &dir, &data.case, &cfg.threads, cfg.reps, cfg.warmup, &opts) {
                    crate::docker::Competitor::Rows { mut rows, image_id, tracks, dumps: other } => {
                        if let Some(tracks) = tracks {
                            let (geom, _) = crate::runner::geom_error(&data, &tracks, data.rays.len())?;
                            let agree = match &reference.tracks {
                                Ok(r) => Some(crate::compare::compare(&tracks, r, data.manifest.diag, data.manifest.chord_tol)?),
                                Err(_) => None,
                            };
                            for r in rows.iter_mut().filter(|r| r.workload == "w-track") {
                                r.geom_error = geom;
                                if let Some(a) = &agree {
                                    r.agreement = crate::result::AgreementSummary::from(a);
                                }
                            }
                            if let Some(a) = &agree {
                                failed += a.failed;
                            }
                            if let Err(e) = &reference.tracks {
                                file.partial.push(format!("{t}: agreement with the sm-rt facet twin not checked: {e}"));
                            }
                            dumps.push((t.to_string(), tracks));
                        }
                        failed += judge_workloads(t, &data, &reference, &other, &mut rows, &mut file.notes, &mut file.partial)?;
                        file.rows.extend(rows);
                        // Identity of the image that produced these rows (outside `result_hash`).
                        // Read by the id the container ran by, not by tag (a rebuild cannot swap it).
                        match crate::docker::image_identity(&image_id) {
                            Ok(id) => {
                                if id.image_id != image_id {
                                    file.partial.push(format!("{t}: image id {} differs from the id the rows ran on ({image_id})", id.image_id));
                                }
                                if file.image_digest.is_empty() {
                                    file.image_digest = id.image_id.clone();
                                }
                                file.images.insert(t.to_string(), id);
                            }
                            Err(e) => file.partial.push(format!("{t}: image identity not recorded: {e}")),
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
            for (x, (na, ta)) in dumps.iter().enumerate() {
                for (nb, tb) in &dumps[x + 1..] {
                    let equal = ta.iter().zip(tb).filter(|(p, q)| p.regions == q.regions).count();
                    file.notes.push(format!("{na} and {nb}: {equal} of {} rays have equal region sequences", ta.len().min(tb.len())));
                }
            }
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

/// sm-rt's facet-twin outputs: what every competitor on the same triangles is judged against.
struct FacetReference {
    tracks: Result<Vec<crate::truth::Track>>,
    fire: Option<crate::adapter::Output>,
    flight: Option<crate::adapter::Output>,
    /// The sm-rt tet walk (cases with a tet mesh).
    walk: Option<crate::adapter::Output>,
    /// Why a reference output is missing.
    why: Vec<String>,
}

/// Run the sm-rt facets adapter on the workloads the competitors run (one thread; the outputs do
/// not depend on it).
fn facet_reference(data: &cases::CaseData, cfg: &RunConfig, workloads: &[&str]) -> FacetReference {
    let w = WorkInput { rays: &data.rays, regions: &data.manifest.regions, threads: 1, histories: cfg.histories, seed: cfg.seed };
    let mut adapter = match crate::smrt::facets_adapter(data) {
        Ok(a) => a,
        Err(e) => return FacetReference { tracks: Err(e), fire: None, flight: None, walk: None, why: Vec::new() },
    };
    let tracks = adapter.run(Workload::Track, &w).map(|o| o.tracks);
    let mut why = Vec::new();
    let mut keep = |what: &str, r: Result<crate::adapter::Output>| match r {
        Ok(o) => Some(o),
        Err(e) => {
            why.push(format!("{what}: {e}"));
            None
        }
    };
    let fire = if workloads.iter().any(|x| x.starts_with("fire")) { keep("fire", adapter.prepare(Workload::Fire, &w).and_then(|()| adapter.run(Workload::Fire, &w))) } else { None };
    let flight = if workloads.contains(&"flight") { keep("flight", adapter.run(Workload::Flight, &w)) } else { None };
    let walk = match data.tets.as_ref().filter(|_| workloads.contains(&"walk")) {
        Some(tets) => keep("walk", crate::smrt::walk_adapter(tets).and_then(|mut a| a.run(Workload::Walk, &w))),
        None => None,
    };
    FacetReference { tracks, fire, flight, walk, why }
}

/// Judge the fire and flight rows of one competitor against the facet reference: the dump of a
/// workload (printed for the first thread count) is compared once and its summary is put on every
/// row of that workload whose output hash equals the dumped row's; a row with another hash (a
/// different output across thread counts) is marked unstable. Returns the number of failed checks.
fn judge_workloads(tracker: &str, data: &cases::CaseData, reference: &FacetReference, dumps: &std::collections::BTreeMap<String, crate::adapter::Output>, rows: &mut [crate::result::Row], notes: &mut Vec<String>, partial: &mut Vec<String>) -> Result<usize> {
    let mut failed = 0;
    for (workload, out) in dumps {
        let Some(dumped_hash) = rows.iter().find(|r| &r.workload == workload).map(|r| r.determinism.hash.clone()) else { continue };
        let summary = match workload.as_str() {
            "w-fire" | "w-fire-locate" => match &reference.fire {
                Some(r) => {
                    let (s, bad) = crate::compare::compare_fires(&out.fires, &r.fires, data.manifest.diag, data.manifest.chord_tol)?;
                    if !bad.is_empty() {
                        notes.push(format!("{tracker} {workload}: failed rays (first {}): {:?}", bad.len().min(10), &bad[..bad.len().min(10)]));
                    }
                    notes.push(format!("{tracker} {workload}: {} of {} rays agree with the sm-rt facets fire ({} grazing, {} failed)", s.checked - s.failed, s.checked, s.grazing, s.failed));
                    Some(s)
                }
                None => {
                    partial.push(format!("{tracker} {workload}: sm-rt facets fire reference not available ({})", reference.why.join("; ")));
                    None
                }
            },
            "w-flight" => match &reference.flight {
                Some(r) => {
                    let s = crate::runner::compare_tallies_with(out, r, true);
                    if out.tally.len() != r.tally.len() {
                        notes.push(format!("{tracker} {workload}: tally lengths differ ({} against {})", out.tally.len(), r.tally.len()));
                    }
                    notes.push(format!("{tracker} {workload}: {} tally slots (collisions, track length, leaked, absorbed) checked against the sm-rt facets flight at 4 sigma, {} failed", s.checked, s.failed));
                    Some(s)
                }
                None => {
                    partial.push(format!("{tracker} {workload}: sm-rt facets flight reference not available ({})", reference.why.join("; ")));
                    None
                }
            },
            "w-walk" => match (&reference.walk, data.tets.as_ref()) {
                (Some(r), Some(tets)) => {
                    let diag = data.manifest.diag;
                    let a = crate::compare::compare(&out.tracks, &r.tracks, diag, 1e-9 * diag)?;
                    let (g, bad_truth) = crate::runner::walk_truth(tets, &data.rays, &out.tracks, diag)?;
                    if !a.listed.is_empty() {
                        notes.push(format!("{tracker} {workload}: rays that differ from the sm-rt walk (first {}): {:?}", a.listed.len().min(10), a.listed.iter().take(10).map(|d| d.ray).collect::<Vec<_>>()));
                    }
                    notes.push(format!(
                        "{tracker} {workload}: {} of {} rays walk the same tets as sm-rt with chords within 1e-9 diag ({} grazing chords dropped, {} sequence failures); chord sums equal the analytic cube chord within 1e-9 diag on {} of {} rays (max error {:.3e})",
                        a.checked - a.failed, a.checked, a.grazing, a.sequence_failures, g.tracks - bad_truth, g.tracks, g.max_chord_err
                    ));
                    for r in rows.iter_mut().filter(|r| &r.workload == workload) {
                        r.geom_error = g;
                    }
                    // Two checks per ray: against the sm-rt walk and against the analytic chord sum.
                    Some(AgreementSummary { checked: a.checked + g.tracks, failed: a.failed + bad_truth, grazing: a.grazing })
                }
                _ => {
                    partial.push(format!("{tracker} {workload}: sm-rt walk reference not available ({})", reference.why.join("; ")));
                    None
                }
            },
            _ => None,
        };
        if let Some(s) = &summary {
            failed += s.failed;
        }
        for r in rows.iter_mut().filter(|r| &r.workload == workload) {
            if r.determinism.hash == dumped_hash {
                if let Some(s) = &summary {
                    r.agreement = *s;
                }
            } else {
                r.determinism.stable = Some(false);
            }
        }
    }
    // Hash equality across thread counts for every workload (stable stays as the driver said it).
    for w in ["w-track", "w-fire", "w-fire-locate", "w-flight", "w-walk"] {
        let hashes: Vec<&String> = rows.iter().filter(|r| r.workload == w).map(|r| &r.determinism.hash).collect();
        if hashes.windows(2).any(|p| p[0] != p[1]) {
            for r in rows.iter_mut().filter(|r| r.workload == w) {
                r.determinism.stable = Some(false);
            }
            notes.push(format!("{tracker} {w}: output hash differs across thread counts"));
        }
    }
    Ok(failed)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cases::{Case, Scale, build};
    use crate::result::Row;

    /// What `judge_workloads` makes of a walk dump: a competitor that reproduces the sm-rt walk
    /// passes both checks of every ray, and one wrong tet or one wrong chord is counted.
    #[test]
    fn a_walk_dump_is_judged_against_the_sm_rt_walk_and_the_analytic_chord() {
        let data = build(Case::H1, &Scale::tiny()).unwrap();
        let reference = facet_reference(&data, &RunConfig::default(), &["walk"]);
        let walk = reference.walk.clone().expect("the sm-rt walk reference");
        let n = data.rays.len();
        let judge = |out: crate::adapter::Output| {
            let dumps = std::collections::BTreeMap::from([("w-walk".to_string(), out)]);
            let mut rows = vec![Row { workload: "w-walk".into(), ..Row::default() }];
            let (mut notes, mut partial) = (Vec::new(), Vec::new());
            let failed = judge_workloads("xdg", &data, &reference, &dumps, &mut rows, &mut notes, &mut partial).unwrap();
            assert!(partial.is_empty(), "{partial:?}");
            (failed, rows.remove(0), notes)
        };
        let (failed, row, notes) = judge(walk.clone());
        assert_eq!((failed, row.agreement.checked, row.agreement.failed), (0, 2 * n, 0));
        assert_eq!((row.geom_error.tracks, row.geom_error.max_chord_err <= 1e-9 * data.manifest.diag), (n, true));
        assert!(notes.iter().any(|m| m.contains(&format!("{n} of {n} rays walk the same tets"))), "{notes:?}");
        // One ray enters the wrong tet, one chord is off by 1e-6: both are found.
        let mut bad = walk;
        bad.tracks[0].regions[1] += 1;
        bad.tracks[1].chords[0] += 1e-6;
        let (failed, row, _) = judge(bad);
        assert!(failed >= 2, "{failed}");
        assert_eq!(row.agreement.failed, failed);
    }
}
