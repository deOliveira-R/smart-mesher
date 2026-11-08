#!/usr/bin/env bash
# Build the smartmesh wheel with maturin and install it into a venv (W3b milestone, plan 5.1).
#
#   tools/w3b_python.sh <venv-dir>
#
# The last stdout line is the venv's python. Exit 3 when maturin can be neither found nor
# installed (the milestone then skips its Python row with that reason). Everything else that
# goes wrong is a plain failure. Environment: PYTHON (interpreter used to create the venv,
# default python3), CARGO_TARGET_DIR (shared with the caller to avoid a second full build).
set -euo pipefail
VENV="${1:?usage: w3b_python.sh <venv-dir>}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PYTHON="${PYTHON:-python3}"

# A cached target/ (CI rust-cache) can restore a partial venv; rebuild unless it is whole.
if [ ! -x "$VENV/bin/python" ] || [ ! -f "$VENV/bin/activate" ]; then
  rm -rf "$VENV"
  "$PYTHON" -m venv "$VENV"
fi
# shellcheck disable=SC1091
. "$VENV/bin/activate"
if ! pip install --quiet numpy maturin >&2; then
  if ! command -v maturin >/dev/null 2>&1 || ! python -c 'import numpy' 2>/dev/null; then
    echo "maturin or numpy cannot be installed here (offline?)" >&2
    exit 3
  fi
fi
rm -rf "$VENV/wheels"
(cd "$ROOT/crates/sm-py" && maturin build --features python --out "$VENV/wheels" >&2)
pip install --quiet --no-deps --force-reinstall --no-index --find-links "$VENV/wheels" smartmesh >&2
echo "$VENV/bin/python"
