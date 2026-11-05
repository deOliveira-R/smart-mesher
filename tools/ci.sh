#!/usr/bin/env bash
# Workspace gate: same checks the orchestrator runs before merging.
# sm-py is excluded from --all-features: its `python` feature (pyo3 extension-module)
# cannot be linked by cargo test; it is gated by crates/sm-py/ci/py.sh. It still gets
# build/test/clippy/doc here with default features.
set -euo pipefail
cd "$(dirname "$0")/.."
cargo build --workspace --exclude sm-py --all-features
cargo build -p sm-py
cargo test --workspace --exclude sm-py --all-features
cargo test -p sm-py
RAYON_NUM_THREADS=1 cargo test --workspace --exclude sm-py --all-features
RAYON_NUM_THREADS=1 cargo test -p sm-py
cargo clippy --workspace --exclude sm-py --all-targets --all-features -- -D warnings
cargo clippy -p sm-py --all-targets -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --exclude sm-py --no-deps --all-features
RUSTDOCFLAGS="-D warnings" cargo doc -p sm-py --no-deps
python3 tools/graph.py > /dev/null
