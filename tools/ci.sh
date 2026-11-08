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
cargo test -p sm-py
RAYON_NUM_THREADS=1 cargo test --workspace --no-fail-fast --exclude sm-py --all-features
RAYON_NUM_THREADS=1 cargo test -p sm-py
cargo clippy --workspace --exclude sm-py --all-targets --all-features -- -D warnings
CARGO_TARGET_DIR="$CAPI_STUBS_TARGET" cargo clippy -p sm-rt-capi --no-default-features --features stubs --all-targets -- -D warnings
cargo clippy -p sm-py --all-targets -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --exclude sm-py --no-deps --all-features
RUSTDOCFLAGS="-D warnings" cargo doc -p sm-py --no-deps
python3 tools/graph.py > /dev/null
