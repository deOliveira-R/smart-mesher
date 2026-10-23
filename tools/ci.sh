#!/usr/bin/env bash
# Workspace gate: same checks the orchestrator runs before merging.
set -euo pipefail
cd "$(dirname "$0")/.."
cargo build --workspace --all-features
cargo test --workspace --all-features
RAYON_NUM_THREADS=1 cargo test --workspace --all-features determinism
cargo clippy --workspace --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps --all-features
python3 tools/graph.py > /dev/null
