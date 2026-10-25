# W2 report: sm-cli (T12)

Status: **done**. Originally written before sm-server (T10) merged; since the integration wave the
flows in `tests/cli_full.rs` run against the real distribution and are no longer ignored (the
historical text below describes the state at T12 delivery). `tests/p0_rows.rs` adds P0 rows through `standard()`.
sm-cli no longer depends on sm-engine (it uses `sm_server::{FixedClock, ...}`).

Definition of done, all run on this tree: `cargo build --workspace`; `cargo test -p sm-cli
--all-features` (45 pass, 4 ignored at delivery) and with `RAYON_NUM_THREADS=1`; `cargo clippy -p sm-cli
--all-targets --all-features -- -D warnings`; `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-cli
--no-deps`.

## Summary

All subcommands of plan §4.3/§4.4 are implemented: `new`, `run`, `apply`, `query`, `log`, `status`,
`hash`, `checkout`, `fork`, `undo`, `redo`, `export`, `serve` (`--stdio`), `mcp`, `help [op]`, with
`--json`, `-p/--memory`, `--author`, `--fixed-time` as global flags. Each is one protocol request via
`run(args, connect, io)`; see `DESIGN.md`.

## Acceptance -> tests

| Acceptance bullet | Test(s) |
|---|---|
| Each command against an in-process server | `tests/commands.rs`: `new_creates_the_project_and_saves`, `run_sends_the_script_and_prints_its_output`, `run_print_hash_appends_the_state_hash_as_the_last_line`, `run_sends_included_files_read_next_to_the_script`, `apply_builds_the_request_and_prints_node_state_and_roles`, `query_defaults_to_null_input_and_parses_revisions`, `status_prints_head_node_and_branches_and_json_is_verbatim`, `log_asks_for_status_and_log_and_tags_branch_tips`, `hash_prints_the_state_hash_and_optionally_layers`, `checkout_fork_undo_redo_map_to_timeline_requests`, `export_infers_the_format_from_the_extension`, `export_flags_mesh_format_and_provenance`, `serve_stdio_answers_one_line_per_request_and_reports_parse_errors`, `mcp_runs_the_mcp_loop_over_stdio`, `help_*` (4). Real binary + real `standard()` server: `tests/binary.rs` (`help_lists_the_catalog_and_describes_an_op`, `hash_json_is_a_hash_result`, `serve_stdio_speaks_json_rpc_lines`, `mcp_initializes_over_stdio`). Full flows on the real server: `tests/cli_full.rs` (runs; not ignored). |
| Exit codes 0/1/2 | `no_arguments_is_a_usage_error_that_prints_usage`, `unknown_flag_and_unknown_command_are_exit_2`, `version_and_help_exit_0_without_connecting`, `server_errors_are_exit_1_with_the_contract_kind`, `project_and_memory_are_mutually_exclusive`, `apply_rejects_malformed_params_locally`, `fork_at_rejects_a_bad_hash`, `run_of_a_missing_script_is_a_usage_error`, `help_unknown_name_is_a_usage_error_with_suggestions`; binary: `version_and_usage_exit_codes`, `an_unknown_op_fails_with_exit_1_and_a_typed_error` |
| `--json` schemas | `json_errors_go_to_stdout_as_an_error_object`, `run_json_is_the_script_result`, `apply_json_is_the_apply_result_and_reads_params_from_stdin`, `log_json_is_one_object_with_head_branches_and_nodes`, `export_json_is_the_export_result_and_bad_inputs_are_usage_errors`, `help_json_is_the_catalog_or_one_entry`, `status_prints_...json_is_verbatim` (stdout parsed back into the typed `sm_protocol` structs) |
| `--fixed-time` makes node hashes reproducible | CLI half: `author_and_fixed_time_reach_the_connector_and_are_validated` (value reaches the builder, malformed values are exit 2). Node-hash half: `cli_full::fixed_time_and_author_make_node_hashes_reproducible` (runs against the real server) |
| Determinism | `binary::output_is_identical_across_runs_and_thread_counts` (`sm_testkit::determinism::check`, 1 vs 3 `RAYON_NUM_THREADS`, concurrent runs) |
| `tests/milestone.rs` marked ignored | **Not written, see deviation 1.** `tests/data/milestone.smesh` and `fork.smesh` are in place. |

## Deviations

1. **No `tests/milestone.rs`.** Plan T12 asks for an ignored one; the task instructions for this run
   forbid creating `tests/milestone*.rs` (the integration phase owns them). The instructions win.
   Substitute: `tests/cli_full.rs` (flows against the real server) and the §4.1/§4.2 scripts under `tests/data/`.
2. **Dependencies beyond the T12 list** (`sm-protocol`, `sm-server`, `sm-mcp`, `clap`): `sm-engine`
   (`FixedClock` and the builder methods; `WorkspaceBuilder` is the public argument of
   `Server::new`/`standard()`, and no Workspace is touched), `sm-op-api` (`Rev`, `Hash256`),
   `sm-core` (`EntityId` for `--mesh`), `serde` (generic `DeserializeOwned`), `serde_json`, clap's
   `derive` feature; dev: `sm-testkit`. `Cargo.lock` is updated for them (clap derive adds
   `clap_derive`/`syn` entries).
3. **Branch name.** Pushed to `claude/w2-sm-cli-0zulw0` (the session's designated branch), not the
   `claude/w2-sm-cli` named in the task text.
4. **`-p` is `session.open`.** `Server::open`/`from_workspace` do not exist yet; `session.open` is the
   protocol route and keeps the CLI off `Workspace`. The CLI sends `session.save` after changing
   commands (a no-op if autosave already persisted).
5. **`serve --ws`** exits 1 with a message: there is no `ws` feature in sm-server to forward to.
6. **Default project.** With neither `-p` nor `--memory`, the nearest enclosing directory with a
   `project.toml` is used, else memory (`new` and `help` never discover). The plan did not say.

7. **Fake endpoint instead of demo plugins.** The task text suggested `sm_server` with demo plugins.
   The skeleton `Server` rejects `apply`, `script.run` and `timeline.*` whatever plugins the builder
   carries, so demo plugins would not make a single extra command testable. `tests/commands.rs`
   therefore uses a recording `Endpoint`; the real server is exercised in `tests/binary.rs`.

## Known gaps / assumptions on other crates

- Everything not in `tests/binary.rs` runs against a fake until T10 merges. When it does, un-ignore
  `tests/cli_full.rs` and run the integration phase.
- **Assumption:** `session.open` on a server built with `--author`/`--fixed-time` keeps that builder's
  author and clock for the project's new nodes. If T10 rebuilds the workspace from a fresh builder,
  `--fixed-time -p dir` (plan §4.3 `smesh run ... -p $T/cli --fixed-time`) will not pin node
  metadata; `fixed_time_and_author_make_node_hashes_reproducible` will catch it.
- **Assumption:** `session.open{create:true}` creates the project on disk (the CLI also sends
  `session.save`).
- `log --json` is composed from two requests because `timeline.log` returns nodes only.
- The `include` pre-scan recognises lines starting with `include "path"` (LANGUAGE.md §4.5), not
  includes written in other layouts.
- `run` does not stream; output is printed after the script finishes.

## api_change_requests

1. **sm-protocol** (additive): re-export `sm_op_api::{Rev, Hash256, Head}` and `sm_core::EntityId`
   so that front ends need not depend on three contract crates to type their arguments. Not blocking.
2. **sm-server** (T10, not a frozen contract): `pub fn standard_with(author: Option<String>,
   fixed_time: Option<String>) -> WorkspaceBuilder` (or re-export `sm_engine::{FixedClock, Clock}`),
   so the CLI need not depend on sm-engine. Also: a `ws` cargo feature, which `smesh serve --ws` will
   forward to when it exists.

## Review fixes (fix0)

Fixed in sm-cli:
- **Must 1.** `run -p DIR` now sends `session.open{create:true}` (plan §4.3 runs `fork.smesh -p $T/fork`
  without `new`); every other `-p` command still sends `create:false`. Pinned by
  `run_under_p_creates_a_missing_project_and_others_do_not`; flow
  `cli_full::run_fork_script_in_a_fresh_project_without_new`.
- **Should 1.** For project sessions the command's output is buffered and `session.save` runs first, so a
  failed save prints only the `{error}` object (`failed_save_prints_only_the_error_object`). `serve`/`mcp`
  stream directly.
- **Should 3.** `--fixed-time` is range-checked (month, day per month and leap year, hour, minute, second).
- **Should 4/11.** `serve --ws ADDR` is a usage error (exit 2), checked before connecting.
- **Should 5.** Added the negative half `cli_full::unpinned_runs_differ_in_node_hashes_but_not_in_state`
  (runs against the real server).
- **Should 6/10.** `includes_of` keeps resolving against the top script's directory; the choice and reason
  (the server looks includes up by key as written) are in its rustdoc.
- **Should 7.** `log` truncates the node string by chars.
- **Should 9.** `new <dir> -p other` is a usage error (exit 2).
- **Should 13.** `tests/fake.rs` moved to `tests/common/mod.rs`.

Not done:
- **Must 2 (drop `sm-engine`).** DONE in w2-fu cli-p0 via sm-server's re-exports. Was: needs sm-server (T10) to expose the clock (`standard_with(author,
  fixed_time)` or a re-export of `FixedClock`/`Clock`); sm-cli may not edit sm-server. Route to T10, or the
  orchestrator accepts the deviation. Once it lands, only `connect_standard` changes.
- **Should 2/8.** `sm-op-api`, `sm-core`, `serde` stay until sm-protocol re-exports the types (ACR 1).
- **Should 12.** Implicit project discovery is unchanged; needs an orchestrator decision (default to memory?).
- **Should 14.** A `run --memory --print-hash` 1-vs-N-thread determinism check belongs to the integration
  phase (needs T10).

## Polish wave

| # | Item | Status |
|---|------|--------|
| 1, 6 | Direct `sm-engine` dependency / `sm_engine::FixedClock` in `connect_standard` | Obsolete: no `sm-engine` dependency exists. `connect_standard` now uses `sm_server::standard_with(author, fixed_time)` (adopted), replacing the hand-rolled `FixedClock` wiring. Pinning still covered by `fixed_time_and_author_make_node_hashes_reproducible`. |
| 2, 7 | `sm-op-api`, `sm-core` outside the T12 Deps list | Fixed: `src` takes `Rev`, `Hash256`, `EntityId` from `sm-protocol`; both crates are now dev-dependencies only (tests). `serde` stays: it is the `DeserializeOwned` bound in `typed` and `Serialize` in `help.rs`; sm-protocol does not re-export it. |
| 3 | `run_exports_and_hash_agree_with_print_hash` checked too little | Fixed (same test): last stdout line must be 64 lowercase hex and equal `state` of a `run --json` of the same pinned script. |
| 4 | `new <dir>` on an existing project | Fixed: exit 1, `already holds a project`, project untouched (`cli_full::new_on_an_existing_project_is_an_error`). Server-side `Open{create:true}` is unchanged (still opens an existing project); the check is in the CLI. |
| 5 | `run --print-hash --json` drops the flag silently | Fixed by documentation: help text of `--print-hash` says it is ignored with `--json` (`state` carries the hash). No behaviour change. |
| 8 | Implicit project discovery default | Not done (needs orchestrator): keep or default to memory. Unchanged. |
| 9 | 1-vs-N determinism of `run --memory --print-hash` | Fixed: `binary::output_is_identical_across_runs_and_thread_counts` now includes a pinned `run --memory --print-hash` of `milestone.smesh` (against the real server); binary tests now run in `CARGO_TARGET_TMPDIR` so exports do not land in the source tree. |
| 10 | Weak error assertions in `p0_rows.rs` | Fixed: `row4_unknown_scheme_is_a_typed_error` and `row2_auto_out_of_range_is_an_error` parse the `--json` error and assert `kind == script-error`, `data.error_code == op-failed` and the exact message (`no Volume/Surface/Curve scheme named \`nonesuch\``, `\`auto\` must be in 1..=10`); MCP side asserts the tool-error text names the Volume scheme. |
| 11 | Report says `cli_full.rs` is ignored | Fixed: stale mentions updated above. |
| contract requests: sm-protocol re-exports | Confirmed and adopted (see 2, 7). |
| contract requests: `standard_with` | Confirmed and adopted (see 1, 6). |
| contract requests: `session.open` keeps author/clock; `create:true` creates on disk | Confirmed by `fixed_time_and_author_make_node_hashes_reproducible` and `run_fork_script_in_a_fresh_project_without_new`; `Server::from_factory` is used. |
| contract requests: sm-server `ws` feature to forward | Not done: feature work, not a review defect. `serve --ws` stays a usage error (exit 2); sm-server now has `ws`, so a forwarding feature in sm-cli is a W3 item. |

| p2fix 1 | Flaky `output_is_identical_across_runs_and_thread_counts` | Fixed: each `smesh()` call in `tests/binary.rs` runs in its own `CARGO_TARGET_TMPDIR/<pid>-<n>` directory, so the concurrent determinism processes no longer race on `out.smsh.part`. |
| p2fix 2 | `new` guard used `exists()`, discovery `is_file()` | Fixed: both use `is_file()`. |
| p2fix 3 | sm-server `write_atomic` fixed `.part` temp name | Not done: outside sm-cli (orchestrator / sm-server owner); a unique temp name (pid + counter) would make concurrent exports robust. |

No state hash, export bytes or golden file changed.
