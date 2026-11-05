//! Ray-tracing benchmark harness (not published): the honest tracking benchmark of sm-rt against
//! DAGMC (MOAB, double-down) and XDG.
//!
//! This crate is the **stub-only** half (task RTBENCH-A): cases, file formats, the agreement
//! checker, fairness rules as code, the result schema, `report`, the `self-test`, the
//! Dockerfiles. Runs of sm-rt and of the Docker competitors are RTBENCH-B.
//!
//! # Map
//! * [`cases`]: the case generator (`C1..C3` legacy, `P1`, `P2`, `H1`, `F1`) and `--model`.
//! * [`formats`]: `smfacets v1`, `smrays v1`, `smtets v1`, `smcase.json`.
//! * [`truth`]: analytic truth independent of every tracker.
//! * [`compare`]: the agreement checker. [`adapter`]: the tracker trait and the stub adapter.
//! * [`runner`]: ABAB interleaving and rows. [`result`]: `smbench-result v1`. [`report`]: tables.
//! * [`docker`]: Dockerfile lint and `docker-check`. [`selftest`]: everything without Docker.
//!
//! # Rules
//! Record-only: no wall-clock gate, no speedup claim, no number typed in by hand. Tests assert work
//! counters, hashes and analytic values.
#![forbid(unsafe_code)]

pub mod adapter;
pub mod bound;
pub mod cases;
pub mod cli;
pub mod compare;
pub mod docker;
pub mod error;
pub mod formats;
pub mod legacy;
pub mod parity;
pub mod report;
pub mod result;
pub mod rng;
pub mod runner;
pub mod selftest;
pub mod smrt;
pub mod truth;
pub mod twin;

pub use error::{BenchError, Result};
