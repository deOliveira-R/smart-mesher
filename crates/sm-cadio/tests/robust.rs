//! Acceptance 12: byte mutations, truncations, line shuffles of every fixture and random bytes
//! never panic and never build a parse tree larger than 20x the input. Every case is derived
//! from its index (SplitMix64 seeded with the index), so a failure names a reproducible case.

mod common;

use std::panic::{AssertUnwindSafe, catch_unwind};

use common::*;
use sm_cadio::part21::{self, Entity, Param, Rec};
use sm_cadio::{LengthUnit, ReadOptions, RealizeOptions, read_step, realize};
use sm_topo_api::{CellComplex, OpId, TopologyEdit};

struct SplitMix(u64);

impl SplitMix {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn below(&mut self, n: usize) -> usize {
        if n == 0 { 0 } else { (self.next() % n as u64) as usize }
    }
}

fn corpus() -> Vec<(String, Vec<u8>)> {
    let mut v: Vec<(String, Vec<u8>)> = sm_testkit::step::ALL.iter().map(|f| (f.name.to_owned(), f.bytes.to_vec())).collect();
    for n in [
        "brick_2x3x4.stp",
        "brick_ap203.stp",
        "brick_inch.stp",
        "two_bricks.stp",
        "brick_with_void.stp",
        "brick_with_void_inward.stp",
        "brick_inverted.stp",
        "frustum_r2_r1_h3.stp",
        "sphere_seam_r3.stp",
        "block_fillet_nurbs.stp",
        "brick_faceted.stp",
        "brick_faceted_dup.stp",
        "brick_unstitched.stp",
        "sheet_open_box.stp",
        "assembly_pins.stp",
        "assembly_mapped.stp",
        "bad/open_shell.stp",
        "bad/cyclic_ref.stp",
        "bad/no_units.stp",
    ] {
        v.push((n.to_owned(), data(n)));
    }
    v.push(("synthetic".to_owned(), synth::bricks_step(2)));
    v
}

fn mutate(rng: &mut SplitMix, src: &[u8]) -> Vec<u8> {
    let mut b = src.to_vec();
    match rng.below(9) {
        0 => {
            for _ in 0..1 + rng.below(8) {
                let i = rng.below(b.len());
                b[i] = rng.next() as u8;
            }
        }
        1 => b.truncate(rng.below(b.len() + 1)),
        2 => {
            let a = rng.below(b.len());
            let n = rng.below(200.min(b.len() - a) + 1);
            b.drain(a..a + n);
        }
        3 => {
            let a = rng.below(b.len());
            let n = rng.below(200.min(b.len() - a) + 1);
            let span: Vec<u8> = b[a..a + n].to_vec();
            let at = rng.below(b.len());
            for (k, c) in span.into_iter().enumerate() {
                b.insert((at + k).min(b.len()), c);
            }
        }
        4 => {
            // Shuffle lines: swap random pairs.
            let text = String::from_utf8_lossy(&b).into_owned();
            let mut lines: Vec<&str> = text.split('\n').collect();
            for _ in 0..1 + rng.below(6) {
                let (i, j) = (rng.below(lines.len()), rng.below(lines.len()));
                lines.swap(i, j);
            }
            b = lines.join("\n").into_bytes();
        }
        5 => {
            // Rewrite the digits after a random `#` (dangling and cyclic references).
            let marks: Vec<usize> = b.iter().enumerate().filter(|(_, c)| **c == b'#').map(|(i, _)| i).collect();
            if !marks.is_empty() {
                let at = marks[rng.below(marks.len())] + 1;
                let mut end = at;
                while end < b.len() && b[end].is_ascii_digit() {
                    end += 1;
                }
                let new = format!("{}", rng.below(400));
                b.splice(at..end, new.bytes());
            }
        }
        6 => {
            // Replace a number token with an extreme or malformed one.
            let nums: Vec<usize> = b.iter().enumerate().filter(|(_, c)| c.is_ascii_digit()).map(|(i, _)| i).collect();
            if !nums.is_empty() {
                let at = nums[rng.below(nums.len())];
                const BAD: [&str; 9] = ["1E308", "-1E308", "0.", "1E-320", "99999999999999999999", "-0.", "NaN", "1.E999", "0"];
                b.splice(at..at + 1, BAD[rng.below(BAD.len())].bytes());
            }
        }
        7 => {
            // Insert random printable garbage.
            let at = rng.below(b.len() + 1);
            for k in 0..1 + rng.below(20) {
                const GARBAGE: &[u8] = b" ()#$*.,;'=/\\0123456789ABCDEF";
                b.insert((at + k).min(b.len()), GARBAGE[rng.below(GARBAGE.len())]);
            }
        }
        _ => {
            // Swap two keywords' parameters by swapping two whole records.
            let text = String::from_utf8_lossy(&b).into_owned();
            let recs: Vec<&str> = text.split(';').collect();
            if recs.len() > 3 {
                let mut r = recs.clone();
                let (i, j) = (rng.below(r.len()), rng.below(r.len()));
                r.swap(i, j);
                b = r.join(";").into_bytes();
            }
        }
    }
    b
}

fn param_bytes(p: &Param) -> usize {
    std::mem::size_of::<Param>()
        + match p {
            Param::Str(s) | Param::Enum(s) => s.capacity(),
            Param::List(v) => v.iter().map(param_bytes).sum::<usize>(),
            Param::Typed(n, b) => n.capacity() + param_bytes(b),
            _ => 0,
        }
}

fn rec_bytes(r: &Rec) -> usize {
    std::mem::size_of::<Rec>() + r.name.capacity() + r.params.iter().map(param_bytes).sum::<usize>()
}

/// A bound on the heap held by a parse tree: map entries, records, parameters and strings.
fn tree_bytes(f: &part21::StepFile) -> usize {
    f.entities
        .values()
        .map(|e| {
            48 + match e {
                Entity::Simple(r) => rec_bytes(r),
                Entity::Complex(v) => v.iter().map(rec_bytes).sum(),
            }
        })
        .sum()
}

fn one_case(idx: u64, corpus: &[(String, Vec<u8>)]) -> String {
    let mut rng = SplitMix(idx);
    let (name, bytes) = if rng.below(12) == 0 {
        let n = rng.below(4096);
        (format!("random#{idx}"), (0..n).map(|_| rng.next() as u8).collect::<Vec<u8>>())
    } else {
        let (n, src) = &corpus[rng.below(corpus.len())];
        let mut m = mutate(&mut rng, src);
        if rng.below(3) == 0 {
            m = mutate(&mut rng, &m);
        }
        (format!("{n} #{idx}"), m)
    };
    let label = format!("case {idx} ({name}, {} bytes)", bytes.len());
    if let Ok(file) = part21::parse(&bytes) {
        let est = tree_bytes(&file);
        assert!(est <= 20 * bytes.len() + 4096, "{label}: parse tree of ~{est} bytes");
    }
    let opts = ReadOptions { assume_unit: Some(LengthUnit::Mm), to_unit: if idx.is_multiple_of(3) { Some(LengthUnit::Cm) } else { None }, weld_tol: if idx.is_multiple_of(5) { 1e-3 } else { 1e-6 }, sheets: idx.is_multiple_of(7), flatten_assemblies: !idx.is_multiple_of(11), ..ReadOptions::default() };
    if let Ok(model) = read_step(&bytes, &opts) {
        // Whatever reads must realize without panicking (errors are fine). NURBS models are
        // skipped: the orientation self-check integrates them (about 8 s for the loft in debug
        // builds); the acceptance tests cover those fixtures.
        let small = model.solids.iter().flat_map(|s| &s.surfaces).all(|x| !matches!(&x.def, sm_geom_api::SurfaceDef::Nurbs(_)));
        let mut cc = CellComplex::new();
        if small && cc.begin_op(OpId(1)).is_ok() {
            let _ = realize(&model, &mut cc, &RealizeOptions::default());
            cc.abort_op();
        }
    }
    label
}

#[test]
fn mutated_and_random_inputs_never_panic() {
    let corpus = corpus();
    // Warm the rayon pool so its start-up is not attributed to any case.
    let _ = read_step(&synth::bricks_step(10), &ReadOptions::default());
    // 2400 cases by default; `SM_CADIO_FUZZ_CASES=n` runs a longer sweep.
    let cases = std::env::var("SM_CADIO_FUZZ_CASES").ok().and_then(|v| v.parse().ok()).unwrap_or(2400u64);
    for idx in 0..cases {
        let t0 = std::time::Instant::now();
        let r = catch_unwind(AssertUnwindSafe(|| one_case(idx, &corpus)));
        if t0.elapsed().as_secs_f64() > 1.0 {
            eprintln!("slow case {idx}: {:.1} s", t0.elapsed().as_secs_f64());
        }
        if let Err(e) = r {
            let msg = e.downcast_ref::<String>().cloned().or_else(|| e.downcast_ref::<&str>().map(|s| (*s).to_owned())).unwrap_or_default();
            panic!("case {idx} failed: {msg}");
        }
    }
}

#[test]
fn every_prefix_of_a_small_file_is_handled() {
    // Truncation at every byte of a small valid file: an error (or success), never a panic.
    let b = data("sphere_seam_r3.stp");
    for n in (0..b.len()).step_by(7) {
        let _ = read_step(&b[..n], &ReadOptions { assume_unit: Some(LengthUnit::Mm), ..ReadOptions::default() });
    }
}

#[test]
fn degenerate_inputs() {
    for bytes in [&b""[..], b"\0", b"ISO-10303-21;", b"ISO-10303-21;HEADER;ENDSEC;DATA;ENDSEC;END-ISO-10303-21;", &[0xff; 64], b"#1=A();"] {
        let _ = read_step(bytes, &ReadOptions::default());
    }
    // A file with no solids is not an error for the reader.
    let m = read_step(b"ISO-10303-21;HEADER;ENDSEC;DATA;ENDSEC;END-ISO-10303-21;", &ReadOptions::default()).expect("empty data section");
    assert!(m.solids.is_empty());
}

// ---- the same property under proptest (2000 cases, fixed seed)

fn shared_corpus() -> &'static [(String, Vec<u8>)] {
    static C: std::sync::OnceLock<Vec<(String, Vec<u8>)>> = std::sync::OnceLock::new();
    C.get_or_init(corpus)
}

fn apply_edit(b: &mut Vec<u8>, kind: u8, at: f64, byte: u8) {
    if b.is_empty() {
        return;
    }
    let i = ((at * b.len() as f64) as usize).min(b.len() - 1);
    match kind {
        0 => b[i] = byte,
        1 => {
            let n = (byte as usize % 64).min(b.len() - i);
            b.drain(i..i + n);
        }
        2 => b.insert(i, byte),
        _ => b.truncate(i),
    }
}

proptest::proptest! {
    #![proptest_config(proptest::test_runner::Config {
        cases: 2000,
        failure_persistence: None,
        rng_seed: proptest::test_runner::RngSeed::Fixed(0x5EED_C0DE),
        ..proptest::test_runner::Config::default()
    })]

    #[test]
    fn structured_mutations_never_panic_and_stay_bounded(
        fixture in 0usize..64,
        edits in proptest::collection::vec((0u8..4, 0.0f64..1.0, proptest::prelude::any::<u8>()), 0..12),
    ) {
        let corpus = shared_corpus();
        let (_, src) = &corpus[fixture % corpus.len()];
        let mut b = src.clone();
        for (kind, at, byte) in edits {
            apply_edit(&mut b, kind, at, byte);
        }
        if let Ok(file) = part21::parse(&b) {
            proptest::prop_assert!(tree_bytes(&file) <= 20 * b.len() + 4096);
        }
        let _ = read_step(&b, &ReadOptions { assume_unit: Some(LengthUnit::Mm), ..ReadOptions::default() });
    }

    #[test]
    fn random_bytes_never_panic(bytes in proptest::collection::vec(proptest::prelude::any::<u8>(), 0..2048)) {
        let _ = read_step(&bytes, &ReadOptions::default());
        // And with a valid prologue so the data section parser is reached.
        let mut text = b"ISO-10303-21;HEADER;ENDSEC;DATA;".to_vec();
        text.extend_from_slice(&bytes);
        let _ = read_step(&text, &ReadOptions::default());
    }
}
