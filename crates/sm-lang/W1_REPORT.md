# sm-lang: W1 report

Status: **done**, revised by **fix0** and **fix1** (review findings; see the "fix0" and "fix1" sections, which supersede the older text below where they differ). Definition of done passes; see "Test summary". Commit: the tip of `claude/w1-sm-lang-96bwd7`; the SHA is reported in the session's final message.

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

`cargo test -p sm-lang`: **136 tests, all passing** (116 after fix0, 80 before; fix1 adds `tests/fix1.rs` with 20), and identical under `RAYON_NUM_THREADS=1`.

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
| `determinism.rs` | 3 | 200,000-entity script through `check(2, 4, ..)` (1 and 4 rayon workers, 4 concurrent calls) and again on 1 and 8 workers: hash of every record's `canonical_bytes` and of the state hash identical (via `sm_testkit::determinism::check`); `parse_many` parallel == sequential |
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

* No user-defined functions, no `where`/attribute predicates in entity lists (reserved and specified in `LANGUAGE.md` §5.5, rejected with `not-yet-supported`).
* (closed in fix1) References inside lists and objects are rebound after a batch.
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

No change to a frozen contract is required for sm-lang to do its job. fix0 adds the optional requests 4 to 6 below.

## fix0: review findings

Branch `w1/sm-lang-fix0-*`, started from `842517e`. Everything is inside `crates/sm-lang`. `LANGUAGE.md` is updated for every decision (sections named below).
Tests: `tests/fix0.rs` (27) plus additions to `parse.rs`, `batch.rs`, `kinds.rs`, `schema.rs` and new goldens; `cargo clippy -p sm-lang --all-targets -- -D warnings`,
`cargo doc` with `-D warnings`, `cargo check --benches` and `RAYON_NUM_THREADS=1 cargo test` pass.

### MUST FIX: language decisions (all binding from now on)

| # | Decision | Where |
|---|---|---|
| 1 | **Batch never changes meaning.** A bare `b.role` for a *scalar* id parameter inside a batch is `expected-one-entity` ("write `<result>.role[0]`"), because a batch cannot know the cardinality of a role and the same statement outside a batch fails when the role holds several ids. `b.role[0]` lowers to `$i.role[0]`. Test: `a_bare_role_for_a_single_id_parameter_means_the_same_inside_and_outside_a_batch` (batch.rs). Cost: scripts that relied on the bare form for single-id roles must add `[0]`. | LANGUAGE §4.4 |
| 2 | **`in` (option b).** `kind in X` followed by `\|`, `&`, `-`, `+` or `except` is the parse error `ambiguous-in`; the writer chooses `faces in (A \| B)` or `(faces in A) \| B`. An error can be relaxed into a meaning later; a meaning cannot be changed. `all faces in X` is accepted as `faces in X`. | §5.2 |
| 3 | **Outputs are ordered lists.** `b.role`, `b.created`, `b.removed` keep op order (`EntitySet` now has a sorted flag and an ordered flavour; set algebra always returns sorted sets; equality is set equality). `b.role[k]` outside a batch equals `$i.role[k]` inside, also for mixed-kind roles (test with a mixed-kind demo op). | §4.3 |
| 4 | **Computed ids**: `Volume#[i]`, `Volume#(i+1)`, `Volume#[a..b by s]`, `Volume#[i, j+1]`; the bare form stays a literal. Serials are validated when evaluated. | §5.2 |
| 5 | **Cubit vocabulary**: `Surface`/`surfaces` are `TopoFace`, `Curve`/`curves` are `TopoEdge`; the carriers are `GeomSurface`/`GeomCurve` (`geom_surfaces`/`geom_curves`). The canonical names `Surface`/`Curve` no longer reach the carriers in source. **Strings are wire form**: they are read canonical-first, so `"Surface#7"` in a string (for example from a query result) is still the carrier; only then are aliases tried. The residual risk is a user writing `"Surface#5"` inside quotes meaning a face; documented in §5.1. | §5.1 |
| 6 | **`except` / `-` (option b).** Mixing `-` or `except` with `\|` or `&` without parentheses is the parse error `ambiguous-set-expression` (needs a `paren` flag on `Expr`). `-`/`except` keep their tier. `all volumes except Volume#1 \| Volume#2` must now be written `... except (Volume#1 \| Volume#2)`. | §3 |
| 7 | **Timeline**: reserved non-registry namespace `timeline.*` (`branch`, `checkout`, `undo`, `redo`), dispatched to the new defaulted `Host::timeline(&TimelineCommand)`; `Event::Timeline`; an error inside a batch and in an expression; a `plan-timeline` warning under `PlanHost`. Not keywords, so no existing identifier stops working. Plugins must not register the `timeline.` prefix (test asserts the demo registry has none). | §4.6 |

### SHOULD FIX

Done: 1 (exact integer min/max and Int/Num comparison, `integer-precision` for `number` parameters above 2^53), 2 (analytic builtin tests, rules documented), 3 (plan-mode `for` over an unknown sequence checks the body once under a warning), 4 (enum values are strings: help and did-you-mean; decision: no bare enum identifiers in v1, a variable of the same name would win if ever added), 5 (= MUST 4), 6 (= MUST 5, plus the empty-selection note for carriers), 7 (`all faces in X`), 8 and 22 (`else` on the next line; flat `else if` chain, one diagnostic), 9 (`Options::max_value_len`, `value-too-long`, checked in `+`, `str`, query results; `range` keeps its own cap), 10 (`Val::List`, `Val::Obj` and `EntitySet` are `Arc`s; `lookup_var` and `.role` are O(1)), 11 and 24 (floats print as floats, ids with their preferred alias), 12 (snapshot explanation), 13 and 29 (`..` only in id specs, inclusive, documented), 16 (partial application documented; `Options::atomic` not added: `batch` is the atomic tool and a whole-run mode needs a decision about queries and `print` between ops), 17 (queries allowed in expressions, id-typed outputs become `Ents`; `Callable` now carries the query output schema), 18 and 15 (predicate grammar written in §5.5; `where`/`named`/`with` reserved after entity expressions with a targeted error), 19 (`batch-snapshot` warning), 20 (`contains(...)`, and `in` errors say a plural kind is needed), 21 (`+` on sets suggests `|`), 23 (`Options::allow_top_level_rebind`, `shadowed` warning), 25 (op failures point at the argument that supplied the entity, message de-duplicated, parameter description as help), 26 (`Diagnostic::to_json(&SourceMap)`, `RunReport::diagnostics_json`), 27 (`ID_PATTERN` pinned against `schemars::schema_for!(EntityId)`; `is_batch_ref` still reimplements the private contract function, covered by the pinned `REF_PATTERN` and the batch-vs-sequential state test), 28 (a schema property with `"x-positional": true` is the positional parameter and wins over the single-required-parameter rule), 30 (the three `expect()` calls and one `unreachable!` are gone; `internal-error` diagnostic instead).

Skipped or only partly done, with reasons:

* **14** schema-level kind checking: needs the `EntityId` schema to carry the kind (request 4). Until then the op's own error (`wrong-kind`) is localized to the argument (25) and the parameter description is shown as help.
* **15 (3)** selection by name: specified in §5.5 (`volumes named "x*"`) but not implemented; it needs a name store that the layers own.
* **Batch-vs-top-level `plan` for `if`**: only `for` got the plan-mode downgrade; an `if` whose condition is a plan placeholder is still a `plan-unresolved` error.
* **Completion** does not offer `timeline.*` names or the new `Volume#[` form specially; it still works on them lexically.

### Behaviour changes that break old scripts (all were error or ambiguous before)

`model.pick(p.body)` inside a batch (write `p.body[0]`); `faces in A | B`, `A - B | C`, `A except B | C` (parenthesize); `Surface#n` / `all surfaces` now select faces; printed output uses `Face#1`
spellings and `5.0` for floats; `ents(r.ids)` accepts entity sets as well as strings.

### More api_change_requests (optional, open tier)

4. **sm-core / sm-op-api**: let the `EntityId` JSON schema carry the expected kind (for example `x-entity-kind`), so sm-lang can reject `mesh.run(targets: faces in Volume#1)` at the argument span (SHOULD 14) and offer kind-aware completion.
5. **sm-op-api**: a first-class positional marker in op schemas (sm-lang already honours `"x-positional": true`), and per-role output cardinality in `OpInfo` (so a batch could accept a bare `b.role` where a role is known to hold exactly one id, MUST 1).
6. **sm-op-api**: export `batch::reference`/`REF_PATTERN` matching as a public function so `schema::is_batch_ref` can be deleted.

## fix1: review findings

Started from `e8a56be`. Everything is inside `crates/sm-lang`. New tests: `tests/fix1.rs` (20), plus changes to `fix0.rs`, `run.rs`, `determinism.rs` and one golden
(`op_failed_wrong_kind`, ids now in source spelling). Checks: `cargo test -p sm-lang` (also with `RAYON_NUM_THREADS=1`), `cargo clippy -p sm-lang --all-targets -- -D warnings`,
`RUSTDOCFLAGS="-D warnings" cargo doc -p sm-lang --no-deps`. `Cargo.lock` is unchanged.

### MUST FIX

| # | Decision | Where |
|---|---|---|
| 1 | **Strings are literal forever.** `"fuel_{i}"` prints `fuel_{i}`; any future interpolation uses a prefixed literal (`f"..."`), which is a syntax error today. A plain string containing `{name}` gets the `literal-braces` warning (help: `+ str(...)`). Pinned by `braces_in_a_string_are_literal_text_forever`. | LANGUAGE §2 |
| 2 | **`Surface#n`/`Curve#n` in a user-written string is the error `ambiguous-kind`** (help: write `Face#n` or `GeomSurface#n`): string-to-id parameters (single and array), `ents()` and `contains()`. New `kinds::parse_user_entity_text`; `parse_entity_text` (wire form, canonical-first) is unchanged and still used for query results, which stay canonical-first through `json_to_val`. Includes `"Surface#" + str(i)`. | §5.1 |
| 3 | **`TimelineRev` is gone**: `TimelineCommand::Checkout(sm_op_api::Rev)`. The interpreter parses the hash (`Hash256::from_str`); a malformed `node:` is `invalid-argument` at the argument span and never reaches the host. | §4.6 |
| 4 | **`${` in a string is a parse error** (`reserved-interpolation`, help: `\${` or `+ str(x)`); `\${` is a literal `${`. A lone `$` is unaffected (batch references still work). | §2 |

### SHOULD FIX

Done: 1 and 12a (a timeline command on the right of `let`/`set` is `op-in-expression`), 2a (a failing batch is mapped back to the member argument that wrote the bad reference, including the element in a
list literal; the batch span is a note; 2b is not possible, see below), 3 (negative-index and loop-over-batch-output messages say why, listed in §4.4), 4 (`ambiguous-in` help echoes the operator;
`expected-one-entity` in a batch names the written variable, `p.faces[0]`, via a `base` field on `Val::Ref`), 5 (line-start operator and `Volume#1..n` help), 6 (references inside lists and objects, at any depth, are rebound after a batch; a
selection past the end of a role reports the real reason, and the plan-mode and "different batch" cases have their own messages), 7 and 11 (`#[non_exhaustive]` on `TimelineCommand`, `Event`, `CandidateKind`, `Severity`;
`TimelineRev` is deleted), 8 (the words `fn def return while break continue import` are reserved: `reserved-word` as names; still valid after `.` and as quoted object keys), 9 (ids in `op-failed`, `query-failed` and batch messages are rewritten to
source spelling, `TopoFace#1` becomes `Face#1`, canonical `Surface#7` becomes `GeomSurface#7`; localization still uses the canonical text), 12c (`timeline.branch("x", at: "<hash>")`, `TimelineCommand::Branch { name, at }`), 13 (ids named by strings are checked for liveness like bare
ids: `entity-not-found` for `ents("Volume#9")` and `model.pick("Face#99")`; the "did you mean" for a dead carrier is unnecessary because `Surface#n` in a string is now an error), 14 (entity sets print in source form with compressed ranges,
`Face#[1..6] | Volume#2`; ordered lists print `[Body#1, Volume#1]`; empty is `<no entities>`), 15 (determinism wording fixed in `DESIGN.md` and here, and the 8-worker run is added to the test), 16 (operator echo, juxtaposition help
`Volume#1 | Volume#2` / `Volume#[1, 2]`), 17 (vocabulary note in §5.1; `hexes tets quads tris` reserved as `not-yet-supported` after `all` / before `in`), 18 (recorded in §5.5 as the first planned language feature).

Skipped or only partly done, with reasons:

* **2b** static role checks: `OpInfo` exposes no output role names. Still open (api_change_request 5, below).
* **10** (O(live) iteration) is a note for the orchestrator: `all <kind>` (`all_of` to `live_range`) and `State::entities_of` scan every serial ever allocated; an O(live) iterator has to come from `sm_op_api::State`.
* **12b** `timeline.head()`: a script still cannot read the current branch. It needs a return channel on `Host` (a value from `timeline`), which is best decided with the `sm-engine` implementor. Left open; documented in §4.6.
* **19** `schema::is_batch_ref` stays: `sm_op_api::batch::reference` is still private. Delete it once the contract exports it (request 6).

### Behaviour changes that break scripts (all were silently wrong or unspecified before)

`"Surface#n"` / `"Curve#n"` strings (error; write `Face#n` or `GeomSurface#n`); `${` inside a string (error); `fn def return while break continue import` as variable names (error);
`let x = timeline.undo()` (error); `ents("...")` and string ids that are not live (error); printed output of entity sets (`Face#[1..6]` instead of `{Face#1, ...}`); `all hexes`/`tets`/`quads`/`tris` (reserved).
Downstream: `Host::timeline` implementors get `Rev` (not a string) and must match `TimelineCommand` with a wildcard arm.

### More api_change_requests

5 (extended). **sm-op-api**: per-role output names in `OpInfo` (so sm-lang can reject `q.voluem` statically, with did-you-mean, in a batch and in plan mode), and optionally cardinality.
6. unchanged: export the batch reference matcher so `schema::is_batch_ref` can be deleted.
7. **sm-op-api State (orchestrator note, SHOULD 10)**: an O(live) entity iterator for `all <kind>`.

## polish0: review items

Started from `9ff1ee8`. Everything is inside `crates/sm-lang`; no frozen contract changed (contract changes are batched by the orchestrator). New tests: `tests/polish0.rs` (23),
plus changes in `tests/parse.rs` (chains). Checks: `cargo test -p sm-lang` (also `--all-features`, `RAYON_NUM_THREADS=1`, `--release`), `cargo clippy -p sm-lang --all-targets --all-features -- -D warnings`,
`RUSTDOCFLAGS="-D warnings" cargo doc -p sm-lang --no-deps`.

| # | Result | Where |
|---|---|---|
| 1 | **Done.** `set x = x + a [+ b ...]` on a list or string appends in place (`Arc::make_mut` / `push_str` on the binding itself; operands are evaluated and length-checked first, so a failing statement leaves `x` untouched). Flat `+` chains append in place too. The whole `polish0` test file, including 200,000 appends, runs in 0.2 s in release (the loop took 103 s before). Timing tests are bounded at 1 s (release) / 30 s (debug). No `push` builtin: builtins are pure functions here and the in-place path makes it unnecessary. | `interp.rs` `try_append`, `eval.rs` |
| 2 | **Done.** `null` is a keyword. `let null` / `for null` is `reserved-word`; `null` in an expression is `no-null` (help: leave the argument out); `key: null` as an object-literal value (also in a `...{}` spread) omits the field; duplicate keys are still checked. | LANGUAGE §2 |
| 3 | **Done.** `if c { a } else { b }` expression, `else` required (`missing-else`), `else if` chains, lazy. | §3 |
| 4 | **Done.** Block `let` hiding an outer binding: `shadowed` warning, help "use `set` to change the outer variable". Loop variables do not warn. | §3 |
| 5 | **Done (parser level).** `x = 2` help: `set x = 2` when an earlier `let`/`for` of the same source bound `x`, otherwise both `let y = 2` and `set y = 2`. The parser cannot see bindings from earlier inputs of an interactive session, so that case gets the two-way help. | `parser.rs` |
| 6 | **Done.** `reserved-prefix` error for an identifier glued to a string (`f"x"`); keywords (`print"hi"`) are unaffected. | lexer |
| 7 | **Done.** `literal-braces` is quiet after `\$` and for the `{` of an already reported `${`. | lexer |
| 8 | **Done.** `invalid-number` for `1.e5`, `1.`, `1_`, `1__0`, `1_.5`, `1e`, `1e+`, each with the fix as help. | lexer |
| 9 | **Done.** `==`/`!=` across types (except Int/Num) is `type-mismatch` with help (`ents(...)` / `str(...)`). Nested values and `contains(list, v)` keep the lenient structural equality. | `eval.rs` |
| 10 | **Done.** `ExprKind::Chain(op, Vec<Expr>)` for runs of one left-associative operator (all binary operators except `**` and comparisons); a parenthesized left operand stays separate. A 5000-term union is tested and 100,000-term chains parse. Nesting depth and operator-alternating chains still hit `too-deep` / `expression-too-long`. | `parser.rs`, `ast.rs` |
| 11 | **Done.** Defining an alias twice is `rebind` (`shadowed` warning in `allow_top_level_rebind` sessions); `alias-confusing` warning when the name is a registry namespace or an existing variable. | §4.1 |
| 12 | **Done.** `print b` / `str(b)` show roles, alphabetical (the op's own order is not kept in `Outcome`): `{body: Body#1, faces: Face#[1..6], volume: Volume#1}`. | §3 |
| 13 | **Decision: innermost body executions are charged** (an iteration during which no nested iteration started). Safe: every uncharged iteration has a charged descendant at most nesting-depth levels below. 1000 x 1000 works with the default 1,000,000 budget (tested); an empty inner loop still charges the outer body. Error text: "more than N loop-body executions". | §9 |
| 14 | **Done.** An `if` whose condition is, or depends on, a placeholder: `plan-unresolved` warning and every branch is checked once (a `plan-unresolved` error inside becomes a warning). `and`/`or` on placeholders now report `plan-unresolved` instead of `type-mismatch`. | §4.3 |
| 15 | **Done in sm-lang, no other crate touched.** `Host::timeline_head(&mut self) -> Result<TimelineHead, HostError>` is a defaulted method (default `timeline-unsupported`), so existing implementors compile unchanged. `timeline.head()` returns `{branch, node}` with absent fields omitted and can be used in expressions. **Request for the sm-engine `Host` implementor**: override `timeline_head` with the session's branch name and head node hash (64 lowercase hex). | §4.6 |
| 16 | **Done.** DESIGN.md "Known gaps" updated (and the robustness paragraph on the height limit). | DESIGN |
| 17 | **Decision: keep the one-space join**, documented with an example (`print "x =", 5` prints `x = 5`). | §3 |
| 18 | **Partly, by design.** Records and `Event::Query.params/result` are wire form on purpose (rewriting would change what is applied and hashed), and `diagnostics_json` messages are already in source spelling. New `sm_lang::respell_json(&Json)` returns a display copy with canonical ids rewritten; documented in §5.1 so the MCP layer applies it to what it shows. | §5.1 |
| 19 | **Done.** `a..b` inclusive range expression (loosest operator: `1..n + 1` is `1..(n + 1)`, a comparison needs parentheses); `range(a, b)` stays half-open; id specs keep their own `..`. | §3 |
| 20 | **Done (doc only).** `named` / `where` stay the first planned feature in §5.5. | §5.5 |

### Behaviour changes that break scripts

`null` as a name or value; `==`/`!=` across types; an alias defined twice; an identifier glued to a string literal; `1.`, `1_`, `1e` as numbers (parse errors of another kind before); printed op
results (were `<op result>`). The loop budget is more permissive for nested loops. Embedders: `ExprKind` has four new variants (`Chain`, `If`, `Range`, `Null`; the AST is public and not `#[non_exhaustive]`),
`Tok` has `Null`, `Host` has a defaulted `timeline_head`.

### Known limits left

* An `if` expression on its own line inside `(`, `[` or a call argument list is read as an unclosed bracket (a line starting with a statement keyword ends the bracket); start it on the line of the bracket or comma.
* Plan mode evaluates every branch of an unknown `if`, so a `set` of an outer variable inside a branch applies for all of them.
* The `x = 2` help cannot know about bindings from earlier inputs of an interactive session.
