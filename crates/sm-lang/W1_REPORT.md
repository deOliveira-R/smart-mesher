# sm-lang: W1 report

Status: **done** (definition of done passes; see "Test summary"). Commit: the tip of `claude/w1-sm-lang-96bwd7`; the SHA is reported in the session's final message.

Branch: the session designated `claude/w1-sm-lang-96bwd7`, and the work was developed and pushed there. The task text names
`claude/w1-sm-lang`; the orchestrator should merge by SHA as CLAUDE.md prescribes, so the name does not matter, but it is not the name the task text gave.

## Summary

`crates/sm-lang` implements the smart-mesh command language as specified in `LANGUAGE.md` (design) and `DESIGN.md` (implementation):

* **Lexer and parser** with byte-exact spans, statement- and argument-level recovery (many errors per run, nothing executes if any exist), `Kind#serial` as a
  single token, bracket-aware newlines, hard limits (size, nesting, expression height, error count).
* **AST**, **values** (`Val` with symbolic batch outputs and plan placeholders), **expressions** (checked integer arithmetic, finite-only floats, 27 builtins and
  `pi`/`tau`, no randomness).
* **Entity lists**: typed ids (`Volume#3`, canonical `TopoVolume#3`), ranges with `by`, id lists, `all <kind>`, `<kind> in X` (through a host `Relations`
  hook), `| & - except`; sorted-set representation with data-parallel, thread-count independent set algebra and range resolution.
* **Lowering**: a call is looked up in the `sm-op-api` registry (ops and queries), arguments are converted under the parameter schema, checked, and become an
  `OpRecord` (canonical form via `Registry::decode` + `OpRecord::of`); `batch { }` becomes one `core.batch` with `$i.role`, `$i.role[k]`, `$i.role[*]` references
  and `$$` escapes exactly as the contract specifies; queries run immediately and are never recorded.
* **Schema checking** (own small JSON-Schema reader over what `schemars` 1 emits, including `$ref`/`$defs`, optionals, enums, bounds, id patterns, the batch schema),
  with issues mapped to the exact sub-expression, did-you-mean for unknown arguments, missing-argument messages carrying the parameter's description.
* **Help** generated from the schemas (`help`, `help <op>`, `help language`) and **completion** at a cursor (`Interpreter::complete`): op/query names and aliases at
  statement start, unused parameters (required first), schema enum values, `true/false`, kinds and live serials, variables, output roles of bound results.
* **Hosts**: `EngineHost` (Engine + State, applies via `Engine::apply_record`), `PlanHost` (validates and lowers without executing), `QueryRelations`.
* **Scripts**: `include` through `Host::load` (no I/O in the crate), `alias`, `let/set`, `if/else`, `for`, `print`, `assert`, `help`.

Deliberate scope decisions (details in `DESIGN.md`): no Cubit compatibility; no user functions; unknown arguments are errors even where the registry decoder would
ignore them; explicit op names only (no suffix resolution, `alias` instead).

## Test summary

`cargo test -p sm-lang`: **80 tests, all passing**, and identical under `RAYON_NUM_THREADS=1`.

| File | Tests | What |
|---|---|---|
| `parse.rs` | 17 | tree shapes, precedence, entity forms, newline/comment rules, spans, recovery, limits, long-chain DoS cases, reserved-word segments |
| `parse_props.rs` | 6 | proptest (pinned seeds): no panic / terminates / in-bounds char-boundary spans / deterministic for arbitrary text, token soup, truncations and mutations of a valid script; a generator of valid programs parses with zero diagnostics |
| `parse_golden.rs`, `run_golden.rs` | 1 + 1 | 17 syntax-error and 38 semantic-error golden files (`tests/golden/{parse,run}`), rendered diagnostics compared byte for byte (`UPDATE_GOLDEN=1` regenerates) |
| `kinds.rs` | 4 | alias table pinned, covers `EntityKind::ALL`, spellings unique and not reserved/builtin, id text round trip |
| `run.rs` | 12 | end-to-end scripts through the interpreter against demo ops, includes, limits, atomicity of failure, plan mode |
| `batch.rs` | 7 | `core.batch` lowering and reference forms, escapes, equals sequential state hash, snapshot semantics, loops unrolled, misuse, atomic failure, stale references |
| `schema.rs` | 7 | contract patterns pinned, checker against real op schemas and `Registry::batch_schema`, `QueryRelations` vs direct resolver, aliases, script statement == hand-built `OpRecord` |
| `entity.rs` | 4 | chunked set algebra vs `BTreeSet` oracle for chunk sizes {1,2,3,7,64,seq}, `live_range` vs filter, retired entities, clipping of huge ranges |
| `determinism.rs` | 3 | 200,000-entity script on 1/4/8 rayon workers + concurrent runs: hash of every record's `canonical_bytes` and of the state hash identical (via `sm_testkit::determinism::check`); `parse_many` parallel == sequential |
| `interp_fuzz.rs` | 4 | interpreter (execute and plan hosts) and completion never panic on token soup / arbitrary text, deterministic; `core.batch` callable directly |
| `help.rs`, `complete.rs` | 6 + 8 | generated help text, did-you-mean, completion contexts, totality over every prefix |

Other checks that pass: `cargo build` (workspace), `cargo clippy -p sm-lang --all-targets --all-features -- -D warnings`,
`RUSTDOCFLAGS="-D warnings" cargo doc -p sm-lang --no-deps`, `cargo bench -p sm-lang --no-run` (and the bench was run once, numbers in `DESIGN.md`).

Bugs the tests found in my own code, fixed: batch references to array parameters need the `[*]` splice form (bare `$i.role` is `[0]`); a quadratic `a.b.c(`
lookahead (400 ms for a 4,000-segment chain); long operator chains building trees deep enough to overflow the stack on drop.

`sm-testkit` use: `determinism::check`, `determinism::Fnv`, `strategies::config` (pinned seeds). sm-testkit has no op fixtures (it does not depend on `sm-op-api`), so
the demo layer, ops, queries and relations in `tests/common/mod.rs` follow the pattern of `sm-op-api/tests/api.rs`.

## Deviations from the contracts' intent

1. **Branch name** (see top).
2. **`Event::Batch.record` is the client form of the batch**, not the canonical stored record: `Engine::apply_record_canonical` is `pub(crate)` in sm-op-api, so a
   non-`Session` host cannot obtain the canonical `core.batch` record the node would store. Members are individually well-formed (current `version`, canonical
   values where the schema allows, e.g. ints written as floats for `number` fields) so the difference is limited to what the batch canonicalization drops/restores. See request 2.
3. **sm-lang rejects unknown arguments** although `Registry::decode`/canonical re-encode would silently drop them (a typo must not become a default).
4. **A plan host does not check existence of explicit ids** (earlier statements were not applied) and its op outputs are placeholders that can never match `REF_PATTERN`.
5. **`Cargo.lock` changed** (new dependencies: `rayon`, `serde_json` already present, `criterion`/`proptest`/`schemars`/`serde` as dev-dependencies). The orchestrator owns and regenerates it at merge.
6. Dependencies beyond the always-allowed list: none. Used: `thiserror`, `serde_json`, `rayon` (runtime); `proptest`, `criterion` (default features off), `schemars`, `serde`, `sm-testkit` (dev).
7. The two workspace-wide `Cargo.toml` rules were respected: only `crates/sm-lang` changed (plus `Cargo.lock`).

## Known gaps

* No user-defined functions, no `where`/attribute predicates in entity lists, no `else` on a line after `}`.
* A batch reference stored inside a list/object variable is not rebound to concrete ids after the batch applies (using it afterwards is `stale-reference`).
* Completion is lexical (no expression type inference); fields only for variables holding op results or objects.
* `kind in X` has no production resolver yet (needs the topology plugin, request 1); with a host without one the error is `no-relations`.
* No `Session`-backed host: it belongs in sm-engine/sm-cli because sm-lang must not manufacture `Meta` (author/time). `EngineHost` demonstrates the replay path.
* Transcendental builtins use platform libm, so *re-evaluating a script* on another platform can change the last bit of a float; replay of a recorded node never re-evaluates.
* Set union on two 1M-id sets does not gain from the parallel path (memory bound); intersection and difference do (about 1.5x on 4 cores).
* `Interpreter` is `Send` when its host is (callables are `Arc`ed), but it is not `Sync`.
* The schema checker interprets only what `schemars` 1 emits for op params; anything else passes and `Registry::decode` decides.

## api_change_requests

1. **crate: the topology plugin owner (sm-topo / sm-engine; not a frozen-contract edit)**
   * what: register a read-only query `topo.related` with input `{ "kind": "<EntityKind name>", "within": ["<EntityId>", ...] }` and output `["<EntityId>", ...]`:
     the entities of `kind` that are contained in or incident to any entity of `within`, in either hierarchy direction (volume to its faces, face to its volumes, body to
     its volumes, mesh set to its cells, ...).
   * why: `faces in Volume#2` needs adjacency, which lives in the topology/mesh layers, not in `State`'s entity table. sm-lang defines the `Relations` trait and
     `QueryRelations`, which calls a registered query of exactly this shape (the query name is a constructor argument).
   * proposed signature: `TypedQuery { const NAME = "topo.related"; struct Related { kind: String, within: Vec<EntityId> }; type Output = Vec<EntityId>; }` (the demo
     version is in `tests/common/mod.rs`).
2. **crate: sm-op-api (additive, open tier)**
   * what: make `Engine::apply_record_canonical` public.
   * why: a host that is not a `Session` (for example the headless runner, or sm-lang's `EngineHost`) cannot report the canonical `core.batch` record; it only has the
     client form. Reporting what a timeline would store matters for scripts that are saved or diffed.
   * proposed signature: `pub fn apply_record_canonical(&self, state: &mut State, rec: &OpRecord, ctl: &Control<'_>) -> Result<(Delta, Option<OpRecord>), ApplyError>`
     (it exists today as `pub(crate)`).
3. **crate: sm-engine / sm-cli (follow-up, not a contract change)**: a `Host` implementation over `Session` (`apply` = `Session::apply` with a `Meta` supplied by the
   caller, one node per record, `state()` = the current state, `relations()` from the topology adapter). `Host` is the only trait that needs implementing.

No change to a frozen contract is required for sm-lang to do its job.
