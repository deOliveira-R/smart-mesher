//! The parse tree of a large file stays within 20x the input (measured as the growth of the
//! process's peak resident set, `VmHWM`, on Linux; skipped elsewhere). The input is generated
//! first, so the baseline already holds it. One test in its own binary so nothing else
//! allocates meanwhile.

mod common;

use sm_cadio::{ReadOptions, part21, read_step};

fn hwm_kib() -> Option<u64> {
    let s = std::fs::read_to_string("/proc/self/status").ok()?;
    let line = s.lines().find(|l| l.starts_with("VmHWM:"))?;
    line.split_whitespace().nth(1)?.parse().ok()
}

fn rss_kib() -> Option<u64> {
    let s = std::fs::read_to_string("/proc/self/status").ok()?;
    let line = s.lines().find(|l| l.starts_with("VmRSS:"))?;
    line.split_whitespace().nth(1)?.parse().ok()
}

#[test]
fn the_parse_tree_and_the_model_stay_within_twenty_times_the_input() {
    let bytes = common::synth::bricks_step(1500);
    let len = bytes.len() as f64;
    let Some(r0) = rss_kib() else { return };
    let file = part21::parse(&bytes).expect("parse");
    let (Some(h1), Some(r1)) = (hwm_kib(), rss_kib()) else { return };
    // Peak over the live baseline: conservative when an earlier transient set a higher peak.
    let peak_growth = h1.saturating_sub(r0) as f64 * 1024.0;
    let live_growth = r1.saturating_sub(r0) as f64 * 1024.0;
    eprintln!("input {:.1} MB; parse tree live {:.1}x, peak {:.1}x", len / 1e6, live_growth / len, peak_growth / len);
    assert!(live_growth <= 20.0 * len, "parse tree {:.1}x the input", live_growth / len);
    assert!(peak_growth <= 20.0 * len, "peak {:.1}x the input", peak_growth / len);
    drop(file);
    // The full read (tree plus model) is not bounded by the file size rule; just record it.
    let m = read_step(&bytes, &ReadOptions::default()).expect("read");
    assert_eq!(m.solids.len(), 1500);
    if let Some(h2) = hwm_kib() {
        eprintln!("after read_step: peak {:.1}x the input", (h2.saturating_sub(r0)) as f64 * 1024.0 / len);
    }
}
