# Pre-registered verdict

Written before any published number exists, so the criteria cannot be tuned afterwards. The code
is `report::verdicts`, `report::statement` and `report::render_sessions`; this file and the golden
tests in `tests/verdict.rs` say the same thing. Changing a rule here means changing the code, the
tests and this file in one reviewed commit, and says so in the report.

## What a cell is

A cell is one (case, workload, sm-rt `mode`) judged against one baseline tracker (`dagmc-moab`,
`dagmc-dd` or `xdg`). Each sm-rt mode is its own cell: the layers L1 to L3 of `DESIGN.md` are never
averaged. The baseline for a cell is the best row of that tracker with the same case, workload and
thread count, across the tracker's modes (best effort for each side, rule c), among rows whose
agreement check passed.

## Criteria (all three must hold, at 1 thread and at the full core count)

1. **Agreement.** Challenger and baseline rows have `agreement.checked > 0` and no failed check in
   their tracker and mode (rule g). Otherwise the cell is `n/c`.
2. **Margin.** The median rate is higher by more than the larger of the two min-max spreads
   (`max - min` of the 7 repetitions). The baseline faster by more than that is a `loss`; anything
   else is `no claim`. Win at one thread count and loss at the other is `no claim`.
3. **Qualifiers.** `structure_bytes`, `build_s` and `peak_rss_bytes` are in the table. A win where
   sm-rt is more than 2x the baseline on any of them is printed `win (qualified: ...)` with both
   numbers, never as a bare win. A value of 0 (not measured) never qualifies.

A cell holds one sm-rt row per thread count. If a results set has two rows of one cell at the same
thread count (for example a second build, or the same run twice), the first in file-name order is
kept and the rest are ignored; `precision` is not a key. A labelled second build (such as the
`target-cpu=native` row) therefore needs its own `mode` name, or its own results directory, to be
judged.

"Full core count" is the largest thread count present for the cell's sm-rt rows. A cell without
rows at 1 thread and at a larger count is `incomplete`; so is a cell with no baseline row. W4-BENCH-X86
must run both the 1-thread and the physical-core-count rows.

## Statement

"sm-rt beats `<baseline>` in k of n cells [names] and does not beat it in the rest: [names with
their sign]". No headline number and no tracker ratio. The known loss (claim 9: the walk-only row in
a large homogeneous region) is a cell like any other and is printed as a loss.

## Polyhedral rows are outside the verdict (ruling D6)

A row whose `mode` or `workload` starts with `poly` (the L2b rows, W4-RT-ROWS names them
`poly-*`) is a different mesh. It renders in its own table, "Polyhedral rows (different mesh, same
geometry; never read by the verdict)", and is dropped before the verdict reads anything, as
challenger and as baseline. `tests/verdict.rs::a_polyhedral_row_cannot_change_a_verdict` proves it,
also through the three-session aggregate.

## L3 rows are outside the verdict

The W-fire rows of the jump group (`smrt` modes `walk` and `jump`, workloads `w-fire*`;
`report::is_l3`) run on H1's stair-stepped tets, while the baselines hold the facet twin
(icospheres): a cell of them would compare two geometries. They render in their own "L3 rows" table,
are dropped before the verdict reads anything (as challenger, and they cannot make another cell `n/c`),
and their own check (jump against walk on the same tets) stays in the agreement column. The reference
walk of the polyhedral group is the `poly-ref` mode, so it is polyhedral and outside the verdict, and
the `walk`/`w-flight` row of the tet-twin flights is the only row of that key. No cell may compare our
rows on a different geometry against theirs without this label.
`tests/verdict.rs::l3_fire_rows_are_outside_the_verdict` proves it.

Not covered by this rule (open for W4-BENCH-X86): the tet-twin `walk`/`delta`/`block` W-flight rows are also
on the stair-stepped tets (L3 by the same argument); a published table must label them or be reviewed first.

## Three sessions (`report::render_sessions`)

`report::render` still refuses mixed machines or UTC days (rule b). The aggregator takes three
sessions, each checked by the same rule on its own, and compares only the per-cell signs:

- exactly three sessions, on three different UTC days, from one machine and CPU;
- each session must itself be a publishable machine (not smoke);
- a cell is a published `win` only with `win` in all three, `loss` only with `loss` in all three,
  and `no claim` otherwise (a missing cell counts as `no claim`);
- no number from any session is printed.

## What counts as a publishable machine (Q9, D6)

`Env::smoke()` is false only for Linux x86_64 with `virt == "none"` (from
`systemd-detect-virt --vm`, falling back to the `hypervisor` flag of `/proc/cpuinfo`) and governor
`performance`. A VM, `powersave`, an undetected virtualisation (also every result file written before
the field existed), macOS and arm64 are smoke, and the banner lists the reasons. What is proven
depends on the source recorded in `virt_source`: `systemd-detect-virt` is the proof; the `cpuinfo`
fallback only sees the `hypervisor` flag, so a VM that hides it reads as `none` when
`systemd-detect-virt` is missing. A table published from a `cpuinfo` result is therefore only as good
as the host check of `tools/bench/x86_64.sh`, which must run `systemd-detect-virt` itself.
`bench-env.json` records `kernel`, `lscpu`, `smt`, `isolated_cores` and `cores` (the pinned core
list, taken from `SMRT_BENCH_CORES`) next to the existing `rustc`, `env` and `docker` blocks.

## No wall-clock gating

Every rule above is a function of recorded rows. Nothing in code or tests waits for, or fails on,
elapsed time.
