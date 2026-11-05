//! Agreement checker: pure functions comparing two sets of tracks (a competitor against a
//! reference, or any tracker against analytic truth).
//!
//! Rules (`DESIGN.md`, fairness rule 7):
//! * Per track the region sequences are equal, except that a chord shorter than
//!   `5e-9 * diag` (grazing) is dropped first. Dropped chords are counted and listed, never hidden;
//!   adjacent chords of one region left by a drop are merged.
//! * Chords agree within the tolerance given by the caller (`max(1e-9 diag, facet sag bound)`).
//! * Disagreements are listed with the ray index and both sequences; the runner writes the first
//!   [`MAX_LISTED`] to `disagreements.jsonl`.

use crate::error::{BenchError, Result};
use crate::truth::Track;
use serde::{Deserialize, Serialize};
use std::io::Write;
use std::path::Path;

/// Grazing threshold factor: chords shorter than `GRAZE_FACTOR * diag` are dropped.
pub const GRAZE_FACTOR: f64 = 5e-9;

/// Number of disagreements kept for `disagreements.jsonl`.
pub const MAX_LISTED: usize = 1000;

/// One disagreeing track.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Disagreement {
    /// Ray index.
    pub ray: usize,
    /// `region_sequence`, `length` (a track whose regions and chords differ in count: malformed input, counted as failed) or `chord` (a chord differs by more than the tolerance).
    pub kind: String,
    /// Sequence of the first tracker (the one being judged).
    pub a: Track,
    /// Sequence of the reference.
    pub b: Track,
}

/// A chord dropped as grazing.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Grazing {
    /// Ray index.
    pub ray: usize,
    /// `a` or `b`.
    pub side: String,
    /// Region of the dropped chord.
    pub region: i32,
    /// Its length.
    pub chord: f64,
}

/// Result of comparing two track sets.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Agreement {
    /// Tracks compared.
    pub checked: usize,
    /// Tracks that disagree.
    pub failed: usize,
    /// Chords dropped as grazing (both sides).
    pub grazing: usize,
    /// Largest absolute chord difference over agreeing sequences.
    pub max_chord_err: f64,
    /// Mean absolute chord difference over agreeing sequences.
    pub mean_chord_err: f64,
    /// Tracks whose region sequences differ.
    pub sequence_failures: usize,
    /// First [`MAX_LISTED`] disagreements.
    #[serde(skip)]
    pub listed: Vec<Disagreement>,
    /// Dropped grazing chords (first [`MAX_LISTED`]).
    #[serde(skip)]
    pub grazing_list: Vec<Grazing>,
}

impl Agreement {
    /// Disagreements per `1e5` tracks (the `geom_error` column).
    pub fn failures_per_1e5(&self) -> f64 {
        if self.checked == 0 { 0.0 } else { self.failed as f64 * 1e5 / self.checked as f64 }
    }
}

/// Drop chords shorter than `thr` and merge neighbours of equal region.
fn canonical(t: &Track, thr: f64, ray: usize, side: &str, dropped: &mut Vec<Grazing>) -> Track {
    let mut out = Track { regions: Vec::new(), chords: Vec::new() };
    for (r, c) in t.regions.iter().zip(&t.chords) {
        if *c < thr {
            dropped.push(Grazing { ray, side: side.to_string(), region: *r, chord: *c });
            continue;
        }
        if out.regions.last() == Some(r) {
            if let Some(l) = out.chords.last_mut() {
                *l += c;
            }
        } else {
            out.regions.push(*r);
            out.chords.push(*c);
        }
    }
    out
}

/// Compare `a` (judged) with `b` (reference), track by track. `diag` is the model's bounding box
/// diagonal and `chord_tol` the chord tolerance (cm).
pub fn compare(a: &[Track], b: &[Track], diag: f64, chord_tol: f64) -> Result<Agreement> {
    if a.len() != b.len() {
        return Err(BenchError::Tracker(format!("track counts differ: {} vs {}", a.len(), b.len())));
    }
    let thr = GRAZE_FACTOR * diag;
    let mut rep = Agreement { checked: a.len(), ..Agreement::default() };
    let (mut sum, mut count) = (0.0f64, 0usize);
    for (i, (ta, tb)) in a.iter().zip(b).enumerate() {
        // A malformed track (regions and chords of different length) would be silently truncated by zip.
        if ta.regions.len() != ta.chords.len() || tb.regions.len() != tb.chords.len() {
            rep.failed += 1;
            if rep.listed.len() < MAX_LISTED {
                rep.listed.push(Disagreement { ray: i, kind: "length".to_string(), a: ta.clone(), b: tb.clone() });
            }
            continue;
        }
        let mut dropped = Vec::new();
        let ca = canonical(ta, thr, i, "a", &mut dropped);
        let cb = canonical(tb, thr, i, "b", &mut dropped);
        rep.grazing += dropped.len();
        for g in dropped {
            if rep.grazing_list.len() < MAX_LISTED {
                rep.grazing_list.push(g);
            }
        }
        let kind = if ca.regions != cb.regions {
            Some("region_sequence")
        } else if ca.chords.iter().zip(&cb.chords).any(|(x, y)| (x - y).abs() > chord_tol) {
            Some("chord")
        } else {
            None
        };
        match kind {
            Some(k) => {
                rep.failed += 1;
                if k == "region_sequence" {
                    rep.sequence_failures += 1;
                }
                if rep.listed.len() < MAX_LISTED {
                    rep.listed.push(Disagreement { ray: i, kind: k.to_string(), a: ta.clone(), b: tb.clone() });
                }
            }
            None => {
                for (x, y) in ca.chords.iter().zip(&cb.chords) {
                    let e = (x - y).abs();
                    rep.max_chord_err = rep.max_chord_err.max(e);
                    sum += e;
                    count += 1;
                }
            }
        }
    }
    rep.mean_chord_err = if count == 0 { 0.0 } else { sum / count as f64 };
    Ok(rep)
}

/// Write `disagreements.jsonl` (one JSON object per line) with `create_new`: an existing file is
/// an error, results are never overwritten.
pub fn write_disagreements(path: &Path, list: &[Disagreement]) -> Result<()> {
    let mut f = std::fs::OpenOptions::new().write(true).create_new(true).open(path).map_err(|e| BenchError::io(path, e))?;
    for d in list.iter().take(MAX_LISTED) {
        let mut line = serde_json::to_vec(d)?;
        line.push(b'\n');
        f.write_all(&line).map_err(|e| BenchError::io(path, e))?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(r: &[i32], c: &[f64]) -> Track {
        Track { regions: r.to_vec(), chords: c.to_vec() }
    }

    #[test]
    fn identical_output_has_no_disagreement() {
        let a = vec![t(&[0, 1], &[1.0, 2.0]), t(&[2], &[3.0])];
        let rep = compare(&a, &a, 100.0, 1e-9).unwrap();
        assert_eq!((rep.checked, rep.failed, rep.grazing), (2, 0, 0));
        assert_eq!(rep.max_chord_err, 0.0);
    }

    #[test]
    fn a_perturbation_is_detected_located_and_written() {
        let a = vec![t(&[0, 1], &[1.0, 2.0]), t(&[0, 1], &[1.0, 2.0]), t(&[0, 1], &[1.0, 2.0])];
        let mut b = a.clone();
        b[1] = t(&[0, 1], &[1.0 + 1e-3, 2.0 - 1e-3]);
        let rep = compare(&b, &a, 100.0, 1e-6).unwrap();
        assert_eq!(rep.failed, 1);
        assert_eq!(rep.listed[0].ray, 1);
        assert_eq!(rep.listed[0].kind, "chord");
        let mut c = a.clone();
        c[2] = t(&[0, 2], &[1.0, 2.0]);
        let rep2 = compare(&c, &a, 100.0, 1e-6).unwrap();
        assert_eq!((rep2.failed, rep2.sequence_failures, rep2.listed[0].ray), (1, 1, 2));
        let dir = std::env::temp_dir().join(format!("smrt-bench-cmp-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("disagreements.jsonl");
        write_disagreements(&p, &rep.listed).unwrap();
        let text = std::fs::read_to_string(&p).unwrap();
        assert_eq!(text.lines().count(), 1);
        let back: Disagreement = serde_json::from_str(text.lines().next().unwrap()).unwrap();
        assert_eq!(back.ray, 1);
        assert!(write_disagreements(&p, &rep.listed).is_err(), "never overwritten");
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_grazing_chord_is_counted_not_flagged() {
        let diag = 100.0;
        let tiny = 0.5 * GRAZE_FACTOR * diag;
        let a = vec![t(&[0, 1, 0], &[1.0, tiny, 1.0])];
        let b = vec![t(&[0], &[2.0])];
        let rep = compare(&a, &b, diag, 1e-9).unwrap();
        assert_eq!((rep.failed, rep.grazing), (0, 1));
        assert_eq!(rep.grazing_list[0].region, 1);
        assert_eq!(rep.grazing_list[0].side, "a");
        // A chord just above the threshold is a real sequence difference.
        let a2 = vec![t(&[0, 1, 0], &[1.0, 2.0 * GRAZE_FACTOR * diag, 1.0])];
        assert_eq!(compare(&a2, &b, diag, 1e-9).unwrap().failed, 1);
    }

    #[test]
    fn a_malformed_track_is_a_length_disagreement() {
        let good = vec![t(&[0, 1], &[1.0, 2.0])];
        let bad = vec![t(&[0, 1], &[1.0])];
        for (a, b) in [(&bad, &good), (&good, &bad)] {
            let rep = compare(a, b, 100.0, 1e-9).unwrap();
            assert_eq!((rep.failed, rep.sequence_failures), (1, 0));
            assert_eq!(rep.listed[0].kind, "length");
        }
    }

    #[test]
    fn mismatched_counts_are_an_error() {
        assert!(compare(&[], &[t(&[0], &[1.0])], 1.0, 1.0).is_err());
    }
}
