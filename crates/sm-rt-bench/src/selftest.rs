//! `self-test`: everything that needs no Docker. Generator determinism, the adapter on the stub
//! against analytic truth, the agreement checker, the schema round trip. `cargo test` calls
//! [`self_test`] directly.

use crate::adapter::{Adapter, Workload, stub_adapter};
use crate::cases::{Case, Scale, build};
use crate::error::Result;
use crate::result::{Env, ResultFile};
use crate::runner::{RunConfig, run_case};

/// One line of the self-test report.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Check {
    /// What was checked.
    pub name: String,
    /// Whether it passed.
    pub ok: bool,
    /// Evidence (counts, hashes).
    pub detail: String,
}

fn check(name: &str, ok: bool, detail: String) -> Check {
    Check { name: name.to_string(), ok, detail }
}

/// Run the checks at the `tiny` scale and return one [`Check`] per item.
pub fn self_test() -> Result<Vec<Check>> {
    let mut out = Vec::new();
    let tiny = Scale::tiny();
    // Generator determinism: two builds of each case give equal file bytes.
    for case in Case::ALL {
        let a = build(case, &tiny)?;
        let b = build(case, &tiny)?;
        let (fa, fb) = (a.files()?, b.files()?);
        out.push(check(&format!("gen {} twice", case.name()), fa == fb, format!("{} triangles, {} rays", a.manifest.triangles, a.manifest.rays)));
    }
    // Adapter on the stub: tracks equal analytic truth, outputs equal for 1 and 4 threads.
    for case in [Case::C2, Case::H1] {
        let data = build(case, &tiny)?;
        let mut adapters: Vec<Box<dyn Adapter>> = vec![Box::new(stub_adapter(&data)?)];
        let cfg = RunConfig { reps: 1, warmup: 0, threads: vec![1, 4], workloads: vec![Workload::Track, Workload::FireLocate, Workload::Flight], histories: 500, seed: 7 };
        let (file, agreements) = run_case(&data, &mut adapters, "surface", &cfg, Env::detect())?;
        let failed: usize = agreements.iter().map(|a| a.failed).sum();
        out.push(check(&format!("stub vs truth {}", case.name()), failed == 0, format!("{} tracks per thread count, {failed} disagreements", data.rays.len())));
        let stable = file.rows.iter().all(|r| r.determinism.stable == Some(true));
        out.push(check(&format!("stub determinism {}", case.name()), stable, format!("{} rows", file.rows.len())));
        let back = ResultFile::from_json(&serde_json::to_string(&file)?)?;
        out.push(check(&format!("schema round trip {}", case.name()), back == file && file.compute_hash()? == file.result_hash, file.result_hash.clone()));
    }
    Ok(out)
}
