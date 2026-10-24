# sm-lang W2 report (T9)

Branch note: the task text names `claude/w2-sm-lang`; the session harness only permits `claude/w2-sm-lang-xkjnpe`, so the work is there. The orchestrator merges by SHA.

## Summary
* `session.export/save/hash/status` parsed and dispatched to `Host::session(&SessionCommand)`; `Event::Session`; `session-in-batch`, `session-failed`, `plan-session`, argument diagnostics.
* `x-entity-kind` checking (`wrong-kind` at the argument span) for single ids, `Option`, arrays, strings and sets.
* Static role checks from `OpInfo.outputs` (did-you-mean in batch and plan mode); bare `b.role` allowed in a batch for `One` roles, lowered to `$i.role[0]`.
* Kind-aware completion; `timeline.*` / `session.*` names and parameters complete; `help session.*`.
* `schema::is_batch_ref` / `ID_PATTERN` were already gone on main (W1 report is stale on this); nothing left to delete.
* LANGUAGE.md §4.4, §4.6-4.7, §6, §7 and DESIGN.md updated.

## Acceptance -> test (`crates/sm-lang/tests/w2.rs`)
| Acceptance | Test |
|---|---|
| Golden diagnostics for every new error | `new_diagnostics_match_golden_files` (`tests/golden/w2/*`: session-failed, session-in-batch, op-in-expression x2, invalid-argument, missing-argument, wrong-kind x3, unknown-name, unknown-field x2, plan-session) |
| `session.*` events reach a recording host with exact `SessionCommand` values | `session_commands_reach_the_host_with_exact_values`, `session_hash_and_status_are_expressions`, `plan_mode_warns_and_never_calls_the_host`, `session_in_batch_is_an_error_for_every_verb`, `bad_export_arguments_are_diagnosed_at_the_argument` |
| Face set for a `TopoVolume` param -> `wrong-kind` at the argument span | `a_face_given_for_a_volume_is_wrong_kind_at_the_argument` (span text asserted), `wrong_kind_covers_options_arrays_strings_and_sets`, `the_annotation_sits_where_the_checker_looks` |
| Bare `b.volume` for a `One` role equals the top-level form (batch bytes and state hash) | `a_bare_one_role_in_a_batch_equals_the_top_level_form`, `a_bare_many_role_in_a_batch_is_still_an_error` |
| `q.voluem` did-you-mean in batch and plan mode | `a_misspelled_role_gets_did_you_mean_in_batch_and_plan_mode` |
| Kind-aware completion; `timeline.*`/`session.*` completion | `completion_is_kind_aware`, `command_names_complete` |
| All existing tests pass | the whole `cargo test -p sm-lang --all-features` run |
| Determinism | `session_scripts_are_deterministic` plus the existing determinism suite (also run with `RAYON_NUM_THREADS=1`) |

## Deviations
* `Callable` gained `outputs` and `ParamInfo` gained `kinds` (public structs, not `#[non_exhaustive]`): additive for readers, breaking only for code that builds them with a struct literal (none in the workspace).
* `session.hash()`/`status()` return whatever JSON the host gives (not forced to a string); T10 owns the shape.
* Plan mode: an unknown `mesh:` placeholder is recorded as `None` (documented in §4.7).
* `Event::Session` was added (the enum is `#[non_exhaustive]`).

## Known gaps
* Role checks only cover ops that declare `OUTPUTS`; queries and `timeline.head()` placeholders are unchecked.
* `x-entity-kind` is checked for ids the language can see; ids inside nested object parameters follow the same path only when the annotation sits on the id property.

## api_change_requests
None.
