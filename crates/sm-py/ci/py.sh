#!/usr/bin/env bash
# Build the smartmesh extension with maturin and run the Python tests.
set -euo pipefail
cd "$(dirname "$0")/.."
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
python3 -m venv "$TMP/venv"
. "$TMP/venv/bin/activate"
pip install maturin pytest numpy
maturin develop --features python
python -c "import smartmesh; print(smartmesh.native_version())"
if [ -d tests ]; then pytest -q tests; fi
