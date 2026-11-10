# BENCH-X86: runbook for the three benchmark sessions

For the person at the x86_64 machine (ruling D6). Three sessions, on three different UTC days,
one command each, then send back one file per session. The script is `tools/bench/x86_64.sh`.

## What you need

- Bare-metal Linux x86_64. A VM is refused, because timing on shared hardware proves nothing.
  At least 16 physical cores and 64 GB RAM are recommended (the script warns below 32 GB).
- The same machine, the same settings and the same commit in all three sessions. The comparison
  across days is only valid then.
- Installed: `docker` (your user can run it without sudo), `git`, `rustup` (the repo pins Rust
  1.99 and rustup fetches it on the first build), a C compiler and linker (`build-essential`),
  `util-linux` (`lscpu`, `taskset`) and `systemd` (`systemd-detect-virt`).
- Network for the first stage of each session (cargo crates, base images, conda and git
  downloads for the competitor images). The timed runs are offline: the script starts every
  container with `--network none`. Plan for about 30 GB of free disk for images and build output.
- Before each session, as root:

  ```sh
  sudo cpupower frequency-set -g performance        # the script refuses anything else
  echo off | sudo tee /sys/devices/system/cpu/smt/control   # recommended; the script only warns
  ```

  Close other heavy jobs. Do not use the machine during the run.

## One-time setup

```sh
git clone <repo url> smart-mesh && cd smart-mesh
git checkout <COMMIT>          # the SHA the orchestrator gives you; the same in all three sessions
tools/bench/x86_64.sh --dry    # host check only; builds and runs nothing
```

`--dry` must end with `dry: host check done`. If it prints `refused: ...` fix that line first
(exit code 3). On a Mac or arm64 machine it prints `refused: not x86_64` and stops.

## The command, once per session

```sh
tools/bench/x86_64.sh
```

The checkout must have no local changes. The script writes to
`~/smrt-bench-sessions/session-<UTC date>/` and refuses to reuse a directory, so two runs on
one UTC day cannot mix. It does, in order: host self-check, `cargo build --release`,
`smrt-bench self-test`, `docker-check` (builds the three competitor images, pinned in
`crates/sm-rt-bench/docker/pins.env`), `gen --print` (the Linux hash lines), the run, and
`report`. The run is `--scale full --cases P1,H1,P2 --trackers smrt,dagmc-moab,dagmc-dd,xdg
--threads 1,<physical cores> --reps 7 --warmup 3`, pinned to one thread per physical core
(`taskset` for native code, `--cpuset-cpus` for containers, the same list, recorded in
`bench-env.json`).

If a stage fails the script says which; if an image does not build it stops (no competitor
row is possible then), and the session log is still the thing to send.

## How long

Not measured: this machine has not run it yet. Expect the first session to be dominated by the
image builds (conda environments and the XDG/MOAB compile) and the later two to skip most of
that, since Docker caches the images. The per-stage seconds land in `stages.txt`; please tell us
the numbers, and the runbook gets the real figures after session one. If the full-scale run is
longer than a working day, stop it (Ctrl-C), send `session.log`, and we will cut the scale.

## What to send back

One file per session, printed on the last line (`SEND BACK: ...`):

```
~/smrt-bench-sessions/smrt-bench-session-<UTC date>.tar.gz
```

It holds `results/` (the result JSON files and `bench-env.json`), `report.md` (this session on
its own), `gen-print.txt` (Linux hash lines for blessing), `docker-check.txt` (image identities),
`host.txt`, `commit.txt`, `stages.txt`, `session.log` and `SHA256SUMS`. It contains no model or
customer data. Also say anything unusual (a throttling warning, another job running, a reboot
between sessions).

## After the third session (orchestrator)

Unpack the three archives into one directory and run, from the same commit:

```sh
cargo run --release -p sm-rt-bench -- report --sessions s1/results,s2/results,s3/results --out table.md
```

`report --sessions` needs exactly three sessions on three UTC days from one machine and CPU, and
each must itself be a publishable machine (VERDICT.md "Three sessions"). A `-SMOKE` directory
is never a session.

## Notes for the reader of the table

- Polyhedral rows (`poly-*`) are a separately labelled table, outside the verdict (ruling D6).
- The `walk`/`delta`/`block` W-flight rows on H1 run on stair-stepped tets (L3 by the argument in
  `VERDICT.md`, open item): label them before publishing.
- This script runs the default build only. The labelled second sm-rt row at `target-cpu=native`
  (draft rt.md Q5) is not run here; it needs its own mode name or results directory (VERDICT.md).
- The Linux `gen --print` lines are unblessed until an orchestrator ruling; `gen-print.txt` is
  the input to that.
- `--smoke` runs the same pipeline on a host that fails the check (a VM, a laptop). The report
  then says SMOKE RESULT and the directory is `session-<date>-SMOKE`; use it to rehearse, never
  as a session.

## Exit codes of `x86_64.sh`

0 ok; 1 a stage failed or the run reported disagreements (still send the archive); 2 bad
arguments; 3 the host is refused, or the session directory exists, or the checkout is dirty.
