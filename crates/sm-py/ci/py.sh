#!/usr/bin/env bash
# Build the smartmesh extension with maturin and run the Python tests.
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m venv .venv-ci
. .venv-ci/bin/activate
pip install maturin pytest numpy
maturin develop --features python
python -c "import smartmesh; print(smartmesh.native_version())"
if [ -d tests ]; then pytest -q tests; fi
