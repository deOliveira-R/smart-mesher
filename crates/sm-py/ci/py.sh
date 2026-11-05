#!/usr/bin/env bash
# Build the smartmesh extension with maturin and run everything on the Python side:
# stub drift check, pytest, mypy --strict, clippy with the `python` feature, and the wheel check
# in a fresh venv that has only numpy (acceptance 10).
#
# Per-platform wheels (manylinux_2_28 x86_64/aarch64, macOS arm64/x86_64, Windows x86_64) are a CI
# matrix over `maturin build --release --features python`; this script builds the host wheel.
# Environment: PYTHON (interpreter, default python3), PY_NUMPY_OLD (pin for the old-numpy wheel
# check, default "numpy==1.26.*", skipped with a message if it cannot be installed).
set -euo pipefail
cd "$(dirname "$0")/.."
CRATE="$(pwd)"
ROOT="$(cd ../.. && pwd)"
PYTHON="${PYTHON:-python3}"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# Binaries and data the tests need (a missing binary fails the tests, it never skips them).
(cd "$ROOT" && cargo build -p sm-cli)
export SMESH_BIN="$ROOT/target/debug/smesh"
export SM_TESTKIT_STEP="$ROOT/crates/sm-testkit/data/step/w3a_part.step"

"$PYTHON" -m venv "$TMP/venv"
. "$TMP/venv/bin/activate"
pip install --quiet maturin pytest numpy mypy
maturin develop --features python,test-hooks
python -c "import smartmesh; print('smartmesh', smartmesh.__version__)"

python tools/gen_stubs.py --check
pytest -q tests
mypy --strict python/smartmesh tests/typing_example.py
(cd "$ROOT" && cargo clippy -p sm-py --features python,test-hooks --all-targets -- -D warnings)

# Acceptance 10: the wheel (no test hooks) installs into a fresh venv with only numpy, and the
# slice runs from the installed wheel, not from the source tree.
maturin build --features python --out "$TMP/wheels"
VERSION="$(cd "$ROOT" && cargo metadata --no-deps --format-version 1 | "$PYTHON" -c \
  'import json,sys; print(next(p["version"] for p in json.load(sys.stdin)["packages"] if p["name"]=="sm-py"))')"
EXPECT_HASH="$("$SMESH_BIN" run tests/slice.smesh --memory --print-hash)"
check_wheel() {
  local pin="$1" venv="$TMP/wheel-venv-$2"
  "$PYTHON" -m venv "$venv"
  "$venv/bin/pip" install --quiet "$pin" || return 3
  "$venv/bin/pip" install --quiet --no-index --find-links "$TMP/wheels" smartmesh
  (cd "$TMP" && EXPECT_VERSION="$VERSION" EXPECT_HASH="$EXPECT_HASH" "$venv/bin/python" -W error "$CRATE/tests/wheel_check.py")
}
check_wheel "numpy" new
if check_wheel "${PY_NUMPY_OLD:-numpy==1.26.*}" old; then :; else
  rc=$?
  if [ "$rc" = 3 ]; then echo "old-numpy wheel check skipped: ${PY_NUMPY_OLD:-numpy==1.26.*} is not installable here"; else exit "$rc"; fi
fi
echo "py.sh: ok"
