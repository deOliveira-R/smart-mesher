# Platform hashes: macOS arm64 against Linux arm64 (W3c-LINUX-HASH)

Smoke evidence. Everything below was run on one Mac (Apple Silicon, Docker Desktop VM, `linux/arm64`, no emulation) at commit `9ac71ca` on 2026-11-07/08 (first run) and rerun on the W3c-INT commit (base `8f62f64` plus the PLATFORM changes, 2026-11-08, with `tools/w3c_pin.smesh` added; table below). Linux x86_64 evidence comes from GitHub CI only. Plan: `plan.md` task W3c-LINUX-HASH; rulings C3, C11, C12 in `rulings.md`. No crate source was touched.

## Result in one paragraph

Every identity (state) hash of the milestone scripts is equal on macOS arm64 and on Linux arm64, for glibc 2.36 and glibc 2.39, with default threads and with `RAYON_NUM_THREADS=1`. That covers 7 scripts, 288 keys (239 for the six scripts of the first run, plus 49 for `w3c_pin`; 307 with the information-only `curved_probe`) (every timeline node, every branch tip, the four layer hashes, the state hash, the memory-run hash, the printed `h_*` hashes). No existing milestone hash moved (C3: nothing to stop). The only raw-bit hashes that differ are the `sm-rt` `flat_golden` bundle hashes (test-side ray generator, see below) and the platform tag inside `smcase.json`. The Linux arm64 `flat_golden` bundle hashes equal the Linux x86_64 values already recorded from GitHub CI. C12 therefore needs no near-tie fix and no portable libm on the evidence so far; `tools/platform/expected.txt` and `crates/sm-cli/tests/platform_pins.rs` (W3c-INT, below) now make Linux x86_64 CI enforce the identity hashes.

## Environment

| | macOS | Linux (container, rust base) | Linux (container, ubuntu base) |
|---|---|---|---|
| platform | darwin-arm64 | linux-aarch64 | linux-aarch64 |
| libc | Apple libm | glibc 2.36 | glibc 2.39 (GitHub runner's glibc) |
| image | none | `rust:1.99-bookworm@sha256:e6f2cff35d0f309acf8433b4f05878ed77092602c5065cc81801177da0bb3a9c` (index digest) | `ubuntu:24.04@sha256:fe81560990fae86dc4743202b2cb4d3df98d0bcf949173cecef15ce1127ef852` (index digest) plus rustup 1.99 (unpinned rustup installer) |
| toolchain | rust-toolchain.toml 1.99 | image's 1.99.1 via `RUSTUP_TOOLCHAIN=1.99.1` (printed as `rustc -V`) | rustup `--default-toolchain 1.99.1` |
| profile | dev, debug = line-tables-only | same | same |
| rustc / rustup (W3c-INT rerun) | rust-toolchain.toml 1.99 | rustc 1.99.1 (58cf8f9c3 2026-10-13), rustup 1.29.3 | same versions, recorded at image build in `/usr/local/share/toolchain-versions.txt` |

Digests were read with `docker buildx imagetools inspect` on 2026-11-07 (`tools/platform/pins.env`), not invented. apt package versions inside the images are recorded by the image build, not frozen (C13). The Dockerfile build prints two `InvalidDefaultArgInFrom` lint warnings (the digest is a build arg); the run lint in `run_container.sh` (every `FROM` has `@sha256:`, no `:latest`, digests are 64 hex) is what enforces pinning. The repo is mounted read-only; `CARGO_HOME` and the target dir are named volumes; `cargo --locked`.

Recipe: `tools/platform/run_container.sh [--platform linux/arm64] [--base rust|ubuntu] [-- CMD]`; hashes: `tools/platform/hashes.sh OUT [--extras]`; diff: `python3 tools/platform/compare.py A B`.

## Identity hashes (state hashes, quantized)

All rows: macOS = Linux (glibc 2.36) = Linux (glibc 2.39). Pins `--author it --fixed-time 2026-10-24T00:00:00Z`.

| script | key | value (equal on all) |
|---|---|---|
| W2 `milestone.smesh` | state | `1fc1366df0c3c526f2780a247c5c69e1a99b058eadadd693d2076b871a718657` |
| W2 `fork.smesh` | main, alt | `1fc1366df0c3c526f2780a247c5c69e1a99b058eadadd693d2076b871a718657` |
| W2 `fork.smesh` | coarse | `f5f2866af4e7f14369b8f9367c06d1dc52ce9b20e764bf81c690073452050164` |
| W3a `w3a_hybrid.smesh` (emitted) | state | `e88b84229b54a2fbf35214cebe8a1b2101dbace8cc48d30c135835e23c4954da` |
| W3a `w3a_tet.smesh` (emitted) | state | `64ce5956410bee6203725e8aea144fa1f1d53a07daa27136a5a97583a9b60773` |
| W3b `tools/w3b_milestone.smesh` | `h_model` | `66e4ac03a259d7e8411d7365470a6917770db7ae5fd57492aba72f04fe1e204e` |
| | `h_auto` | `fca39b7908faae617c67ba6ae5ee63f457225ac56982ddf21c5d955e40f469c4` |
| | `h_mesh` (= final state) | `6f6c19f042a4b23529cfa0d05ff6c4bbd212e9862fefebacf18617f682b7ea2f` |
| W3c `tools/w3c_pin.smesh` (through pin, 3x3 lattice, 432 hexes) | `h_model` | `f238f0340fc123ef799f241e7f9ff99e3d5d7ef848113f9a6c20f768d47b4e9d` |
| | `h_auto` | `1f4b0912569abb477044a9d2bf42f024b5936a30220888ea5bfd28f9234fdbe1` |
| | `h_mesh` (= final state) | `6953b909cfbaf576efe1828577f6daf73039b0fa4934677a99dab0172f1a36f6` |
| SUBMAP_GRADED (`tools/platform/submap_graded.smesh`) | topology / geometry | `c99bcaa8d93e9a5e` / `43ecc0a4b59b3715` |
| | state | `7b71fd6dd5c3dd7136a6a1b78a40c8fa3f851877e9f069a73f164f731b0c0b1f` |

The SUBMAP_GRADED values equal the literals in `crates/sm-cli/tests/milestone_w3b.rs:603-605`, and the platform script has the same ops (an added `mesh.info` query and prints).

Per-script key counts compared: `w2_milestone` 21, `w2_fork` 25, `w3a_hybrid` 63, `w3a_tet` 49, `w3b_milestone` 56, `w3b_submap_graded` 25, `w3c_pin` 49, `curved_probe` 19 (307 in all; 239 for the first six, 258 with `curved_probe`). Same key set on both sides for every script; `compare.py` treats a key present on one side only as a difference.

Controls:
- Same host twice, default threads: equal (macOS `mac1` vs `mac2`, 6 scripts).
- `RAYON_NUM_THREADS=1` against default: equal on macOS and on Linux (glibc 2.36).
- Negative control: one node-state key hand-edited; `compare.py` exits 1 and names the script (`w3a_tet`), the key (`node.003.state`) and the first differing node.
- Not run: `smesh hash --at` for `--at tet` as a separate rev (the tet script is its own file; branch tips of every script are compared).
- Not run: `milestone_w3b` test rows (including M6, deterministic only per libm) and `--platform linux/amd64`.

### Curved probe (information only)

`tools/platform/curved_probe.smesh`: cylinder, sphere, torus; `view.tessellate` counts; STEP export (5180 bytes) and re-import (`h_reimport`). Equal on macOS and both Linux images (state `aa4a9b9f...37dd2`, 2600 triangles, 1304 vertices, `h_reimport` `a304b519...26aa`). This is only the model and sm-tess count path: at `9ac71ca` `mesh.auto`, `tetmesh` and face `mesh.run` refuse curved entities ("no applicable scheme"), so no curved mesh hash exists yet. `tessellate` positions are `f32` raw bits and were not compared. The first hashed curved mesh is `w3c_pin.smesh` (INT rerun below): its 49 keys, including the layer hashes, `h_model`, `h_auto`, `h_mesh`, the cell count and the printed mesh volume (`14.288400000000104`, a float), are equal on macOS, Linux glibc 2.36 and glibc 2.39. The cylinder, imprint and merge path therefore does not expose the 1-ulp libm difference at these values. Only the identity keys are pinned (the volume is not).

`cos(TAU/3)` as computed by the host libm (`tools/platform/cos_probe.py`): macOS `0xbfdffffffffffffd`, glibc 2.36 and 2.39 `0xbfdffffffffffffc`. One ulp, the same case that made `sm-scheme-tet` `int7` pin ring coordinates (below).

## Raw-bit hashes

### sm-rt `flat_golden` (`(structure hash, bundle hash)`)

Structure hashes equal on every platform (macOS, Linux arm64 glibc 2.36 and 2.39, Linux x86_64 CI; they equal the constants at `flat_golden.rs:49-56`). Bundle hashes (raw bits, per platform by C11):

| case | structure hash | bundle, macOS arm64 | bundle, Linux arm64 (2.36 = 2.39) | bundle, Linux x86_64 (CI row in source at 9ac71ca) |
|---|---|---|---|---|
| hex_block6 | `0x9d2acd0945032943d69768d2b42ebe92` | `0xcd433b5fc77169cad96f4c57eebb8e17` | `0x21a475890c57ab277f541a53d45710dd` | `0x21a475890c57ab277f541a53d45710dd` |
| nonconvex_poly | `0x4af44037ed018b8228e09125dc004e3f` | `0x7f9e0cad2f60dd8a499b494fe2407b08` | `0xd88c1ac6430cdbf35c94b1be24b37627` | `0xd88c1ac6430cdbf35c94b1be24b37627` |
| mixed_transition | `0x5659f755d6b3ba0001f58b4a68cddb58` | `0xad87d0efcd2433934a62707dd4e1770c` | `0x02973ad67e28a8981126ca32545b815e` | `0x02973ad67e28a8981126ca32545b815e` |
| nested_facets | `0x156357c4b70cb571d51c1b58420b49aa` | `0xb87fb50821f41b2cb450a03bb5537d2d` | `0x754e51e2e9adde0dedc9bbcaa9c6652d` | `0x754e51e2e9adde0dedc9bbcaa9c6652d` |

- Linux arm64 equals Linux x86_64 on all four bundle hashes; only macOS differs. So the bundle hash separates Apple libm from glibc (the test's `random_frame` uses `phi.cos()`/`phi.sin()`, `crates/sm-rt/tests/common/mod.rs:53-57`), and arm64-against-x86_64 glibc did not matter for these four cases. This is consistent with the F4 hypothesis (tracker build libm-free on the flat path; ray generator libm-dependent); it is not a proof of the cause, because the rays were not swapped for a libm-free generator.
- The first red GitHub run (42412194333, main `d7ad022`) quoted the Linux hex_block6 bundle prefix `44718444...` in the planning draft; the row now in source is from PR #1 run 42448206605. The two were not reconciled here (different commits).

### sm-rt-bench `smrt-bench gen --scale tiny --print`

Raw `facets.bin` and `rays.bin` hashes are equal on macOS and Linux arm64 for every case printed (C1-C3, P1, P2, H1, F1; the `@macos-aarch64` keyed entries P1, P2, F1 match their Linux values byte for byte). Only `smcase.json` differs, because it embeds the platform tag. Linux arm64 lines (not blessed in `expected_sha256.txt`; blessing needs the orchestrator, `DESIGN.md:51-53`):

```
2fd43c38808591172c227a4c02002b05fadc328f228379bb7c78e354b225194e  tiny/C1/smcase.json@linux-aarch64
35cdbeb5aeb85192a3fc92f70b5dee94675838d7f89a87434df42d9e3f6fb784  tiny/C2/smcase.json@linux-aarch64
8643a511efa1e74618dfdd61c2897bd95717059363a52bc3d4b906cf6a36dfef  tiny/C3/smcase.json@linux-aarch64
ba1df4a36a51695c621fdc5fa8891efa0ed4914c8b0098de8aac8b19ca741812  tiny/P1/smcase.json@linux-aarch64
7f640109d1bc6b61cdbcea2ac2d2e77e1c18681cbed604e44da4a06336249737  tiny/P2/smcase.json@linux-aarch64
03b377e9ab8672b610e1593e9914ad670a0c19a95d873ed69b34054802183581  tiny/H1/smcase.json@linux-aarch64
e31ede03b6c876f5f81c60ea073c577b27504dc400df819ed4fbae4f2c5443dc  tiny/F1/smcase.json@linux-aarch64
```

Tiny scale only. The facet equality is an observation, not a ruling: the `@macos-aarch64` keys on P1/P2/F1 facets and rays in `expected_sha256.txt` stay as they are.

## What GitHub CI (Linux x86_64) already showed

- `sm-rt` `flat_golden`: structure hashes equal; bundle hashes differ from macOS. Rows are now recorded per platform (C11), the four x86_64 values above.
- `sm-scheme-tet` `int7`: the ring input was built with `cos`/`sin` of `TAU * i / 3 + rot`; glibc and macOS `cos(TAU/3)` differ by one ulp, which flipped whether the k = 3 tower restarts. The test now builds the ring from pinned bit patterns (`crates/sm-scheme-tet/tests/int7.rs`, `RING_BITS`). Confirmed here: macOS `0xbfdffffffffffffd`, glibc `0xbfdffffffffffffc`. Note this is a test input, not a library decision; but the library result depended on that 1-ulp input, so a restart-or-not decision in `sm-scheme-tet` is sensitive to near-ties of that size. The milestone scripts do not reach it (W3a tet is planar bricks; equal above).
- Earlier: `submap_graded_input_is_pinned` passes on Linux x86_64 against the macOS-blessed literals.
- Caveat: the gate was red from the W3b-1 merge until the `flat_golden` rows; crates sorted after `sm-rt` were not exercised on Linux during that period (`--no-fail-fast` fixes the masking).

## W3c-INT rerun (commit base 8f62f64 + PLATFORM changes)

`tools/platform/hashes.sh` on macOS arm64 and in `linux/arm64` containers (rust:1.99-bookworm glibc 2.36, ubuntu:24.04 glibc 2.39), `compare.py` mac against each Linux: all 8 scripts `equal` (`w2_milestone` 21, `w2_fork` 25, `w3a_hybrid` 63, `w3a_tet` 49, `w3b_milestone` 56, `w3b_submap_graded` 25, `w3c_pin` 49, `curved_probe` 19), `RESULT: all equal`, no `error.*` key (`hashes.sh` now exits 1 if one appears). The milestone hashes of C3 (W2 `1fc1366d...`, W3a A/B, W3b `h_model`/`h_auto`/`h_mesh`, SUBMAP_GRADED) are unchanged. `flat_golden` on Linux arm64 gives the same four (structure, bundle) tuples as the table above on both images.

`crates/sm-cli/tests/platform_pins.rs` (passes on macOS, in the glibc 2.36 container and in the glibc 2.39 container; each run does default threads and `RAYON_NUM_THREADS=1` and both equal `expected.txt`): 62 lines, 7 scripts (state, layer hashes, branch tips, memory-run hash, hash-like and count prints; not the per-node rows, floats or `curved_probe`). Refresh only after a ruling: `PLATFORM_PINS_EMIT=<file> cargo test -p sm-cli --test platform_pins`.

Follow-up fixes made in this rerun: `run_container.sh` pins `RUSTUP_TOOLCHAIN=1.99.1` and prints `rustc -V` / `rustup -V`; the ubuntu image installs toolchain 1.99.1 and records the versions; `collect.py` stores every error as `error.<step>` with one helper; `submap_graded.smesh` no longer claims `hashes.sh` checks its hash (it is pinned in `expected.txt`).

## C12 and what is left

- Nothing diverged, so no near-tie localisation was needed, and no per-platform state hash is proposed.
- Done at W3c-INT: `tools/platform/expected.txt` and `crates/sm-cli/tests/platform_pins.rs` (see the rerun section). Original instruction: rerun `run_container.sh` and `hashes.sh` on the INT commit.
- Closed after this run: `platform_pins.rs` passed on the Linux x86_64 GitHub gate (runs 42544784727, 42553841653), so the pinned state hashes hold there too. Background: glibc on x86_64 selects `sin`/`cos`/`exp`/`log`/`pow` variants at run time, which arm64 does not exercise; the `int7` ulp case shows near-tie decisions exist in `sm-scheme-tet`.
