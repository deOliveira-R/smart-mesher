#!/usr/bin/env bash
# Build the pinned Linux image and run a command in it with the repo mounted read-only.
#
#   tools/platform/run_container.sh [--platform linux/arm64|linux/amd64] [--base rust|ubuntu] \
#       [-- CMD...]
#
# Default CMD: `tools/platform/hashes.sh /out/hashes.txt --extras`; /out is bind-mounted to
# target/platform-out/<base>-<arch> in the checkout. Named volumes hold CARGO_HOME and the target
# dir (a host target/ is never mounted). Prints `platform linux-<arch> glibc <v>` and the image
# digests. Smoke only when run on a Mac (Docker Desktop VM).
set -euo pipefail
cd "$(dirname "$0")/../.."
platform=linux/arm64
base=rust
while [ $# -gt 0 ]; do
  case "$1" in
    --platform) platform="$2"; shift 2 ;;
    --base) base="$2"; shift 2 ;;
    --) shift; break ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
# shellcheck disable=SC1091
. tools/platform/pins.env
for v in RUST_DIGEST UBUNTU_DIGEST; do
  d="${!v:-}"
  if ! printf '%s' "$d" | grep -Eq '^[0-9a-f]{64}$'; then echo "pins.env: $v is unset or not a sha256 hex" >&2; exit 2; fi
done
# Lint: every FROM carries @sha256 (no :latest, no unpinned base).
if grep -E '^FROM ' tools/platform/Dockerfile | grep -vq '@sha256:'; then
  echo "Dockerfile: a FROM line is not pinned by @sha256" >&2; exit 2
fi
if grep -Eq ':latest' tools/platform/Dockerfile; then echo "Dockerfile: :latest is refused" >&2; exit 2; fi
arch="${platform#linux/}"
tag="smart-mesh-platform:${base}-${arch}"
docker build --platform "$platform" --target "base-$base" \
  --build-arg "RUST_DIGEST=$RUST_DIGEST" --build-arg "UBUNTU_DIGEST=$UBUNTU_DIGEST" \
  -t "$tag" tools/platform
out="$PWD/target/platform-out/$base-$arch"
mkdir -p "$out"
[ $# -gt 0 ] || set -- tools/platform/hashes.sh /out/hashes.txt --extras
docker run --rm --platform "$platform" \
  -v "$PWD:/src:ro" -v "$out:/out" \
  -v "sm-platform-cargo-$base-$arch:/cache/cargo" -v "sm-platform-target-$base-$arch:/cache/target" \
  -e CARGO_HOME=/cache/cargo -e CARGO_TARGET_DIR=/cache/target \
  -e PLATFORM_SCRATCH=/cache/target/platform-scratch \
  -e RUSTUP_TOOLCHAIN=1.99 -e RAYON_NUM_THREADS="${RAYON_NUM_THREADS:-}" \
  -e CARGO_PROFILE_DEV_DEBUG=line-tables-only -e CARGO_PROFILE_TEST_DEBUG=line-tables-only \
  -w /src "$tag" bash -c '
    echo "platform linux-$(uname -m) glibc $(getconf GNU_LIBC_VERSION | cut -d" " -f2)"
    echo "image '"$tag"' base digest '"$([ "$base" = rust ] && echo "$RUST_DIGEST" || echo "$UBUNTU_DIGEST")"'"
    [ -n "$RAYON_NUM_THREADS" ] || unset RAYON_NUM_THREADS
    exec "$@"' bash "$@"
