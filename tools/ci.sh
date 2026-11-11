#!/usr/bin/env bash
# Workspace gate: same checks the orchestrator runs before merging.
# sm-py is excluded from --all-features: its `python` feature (pyo3 extension-module)
# cannot be linked by cargo test; it is gated by crates/sm-py/ci/py.sh. It still gets
# build/test/clippy/doc here with default features. tests/milestone_w3b.rs (sm-cli) builds the
# wheel into its own venv with maturin and compiles tools/w3b_driver.c against libsm_rt_capi; it
# skips the Python row with a printed reason only when maturin cannot be installed.
set -euo pipefail
cd "$(dirname "$0")/.."
cargo build --workspace --exclude sm-py --all-features
cargo build -p sm-py
# sm-rt-capi without the loader: smrt_open answers SMRT_ERR_UNSUPPORTED (-5), which only this
# configuration still tests (the workspace runs build the loader in). Own target directory: the
# cdylib is written to deps/libsm_rt_capi.<ext> whatever the features, so one directory shared by
# two feature sets would leave the other set's tests with the wrong library.
CAPI_STUBS_TARGET="${CAPI_STUBS_TARGET:-target/capi-stubs}"
CARGO_TARGET_DIR="$CAPI_STUBS_TARGET" cargo test -p sm-rt-capi --no-default-features --features stubs
cargo test --workspace --no-fail-fast --exclude sm-py --all-features
# Passing tests have their output captured; rerun with --nocapture so the CI log carries the
# flat_golden hash tuples (needed to record the bundle hashes of a new platform).
cargo test -p sm-rt --all-features --test flat_golden -- --nocapture
cargo test -p sm-py
RAYON_NUM_THREADS=1 cargo test --workspace --no-fail-fast --exclude sm-py --all-features
RAYON_NUM_THREADS=1 cargo test -p sm-py
cargo clippy --workspace --exclude sm-py --all-targets --all-features -- -D warnings
CARGO_TARGET_DIR="$CAPI_STUBS_TARGET" cargo clippy -p sm-rt-capi --no-default-features --features stubs --all-targets -- -D warnings
cargo clippy -p sm-py --all-targets -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --exclude sm-py --no-deps --all-features
RUSTDOCFLAGS="-D warnings" cargo doc -p sm-py --no-deps
python3 tools/graph.py > /dev/null
# tools/bench/x86_64.sh (the x86 bench session script) is linted where shellcheck exists
# (ubuntu runners ship it). Severity warning: shellcheck >= 0.10 adds info SC2329 for the step
# functions, which the script calls indirectly through its `step` runner ("$@").
if command -v shellcheck > /dev/null; then
    shellcheck -S warning tools/bench/x86_64.sh
else
    echo "ci.sh: shellcheck not installed, skipping the tools/bench/x86_64.sh lint"
fi
# tools/refcorpus: the unittest needs only python3; ruff and mypy run where installed (the
# mypy config imports openmc with ignores, so no OpenMC is needed).
python3 -I -B -m unittest discover -s tools/refcorpus -p 'test_*.py'
if command -v ruff > /dev/null; then
    ruff check --no-cache --config tools/refcorpus/ruff.toml tools/refcorpus
else
    echo "ci.sh: ruff not installed, skipping the tools/refcorpus lint"
fi
if command -v mypy > /dev/null; then
    (cd tools/refcorpus && mypy --cache-dir=/dev/null --config-file mypy.ini)
else
    echo "ci.sh: mypy not installed, skipping the tools/refcorpus type check"
fi
