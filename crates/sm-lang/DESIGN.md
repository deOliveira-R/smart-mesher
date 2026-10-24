# sm-lang: design

The smart-mesh command language (`smesh`): lexer, parser with recovery, entity-list resolution, schema-driven argument checking,
lowering to `sm-op-api` op records and `core.batch`, completion and help. The language itself is specified in
[`LANGUAGE.md`](LANGUAGE.md); this file is the implementation design.

## Positioning

sm-lang is **similar to** a Cubit journal front end in what users type (replayable commands, `all`/`except`/`in` entity vocabulary) and to
an MCP tool-call layer in what it produces (an op name plus a JSON object validated by the op's JSON schema); it **differs** in being a
real, small language (expressions, variables, loops, entity-set algebra) whose every call is *schema-driven*: there is no per-command
grammar, so the parser knows nothing about any op; argument checking, help text, completion and the `batch` lowering all read the schemas
published by the `sm_op_api::Registry`; **giving** one source of truth for GUI actions, Python, MCP and scripts (a statement *is* an `OpRecord`),
diagnostics with exact spans for scripts written by people or by Claude, and ops that can be added by a plugin without touching this crate.
Implementation-wise it is a recursive-descent parser plus a tree-walking interpreter over a schema-guided lowering, not a bytecode VM:
statements are dominated by op cost, and the interpreter's job is to be exact and diagnosable, not fast (a 2000-statement script lowers in about 13 ms).

## Data flow

```
text ──lex──► tokens ──parse──► AST (+ diagnostics, recovery)        parser.rs, lexer.rs, ast.rs
                                   │ (any syntax error: report all, run nothing)
                                   ▼
                     Interpreter::exec (statement by statement)       interp.rs
   eval ── pure expression semantics, builtins ─────────────────────── eval.rs, value.rs
   resolve entity lists against Host::state() (+ Relations) ────────── entity.rs
   build_params: evaluate args, positional rule, spreads
   conv: Val → JSON guided by the parameter schema (ids, refs, ints) ─ schema.rs (View)
   schema::check(_in_batch): validate; map issues to spans (locate)
   OpRecord::new + Registry::decode (canonical form, top level only)
                                   ▼
   Host::apply / query ── EngineHost (Engine::apply_record) | PlanHost (nothing runs) | a Session adapter (sm-engine)
```

| Module | Content |
|---|---|
| `span`, `diag` | `Span{file,lo,hi}`, `SourceMap`, `Diagnostic` (code, label, notes, help), rustc-style `render` (used by the goldens), edit distance with transpositions for did-you-mean |
| `lexer` | byte lexer, never panics, `Kind#` is one token, `1..5` is `Int .. Int`, errors become diagnostics + `Error` tokens |
| `parser`, `ast` | statement recovery (`sync`), argument-level recovery, bracket-aware newline handling, `Expr::height` bound, `parse_many` (rayon, ordered) |
| `kinds` | the frozen alias table (`KIND_TABLE`), tested to cover `EntityKind::ALL` |
| `entity` | `EntitySet` (an `Arc`ed id vector: a sorted set, or an ordered list for op outputs; set algebra always returns sorted sets), chunked parallel union/intersection/difference, `live_range`, `all_of`, the `Relations` hook |
| `value`, `eval` | `Val`, symbolic `Outcome`/`Ref`/`Placeholder`, checked arithmetic, builtins |
| `schema` | JSON-Schema reader/checker for what `schemars` 1 emits; `ParamInfo` for help/completion |
| `host` | `Host`, `EngineHost`, `PlanHost`, `QueryRelations` |
| `interp` | `Interpreter`, `Options`, `Event`, `RunReport` |
| `help`, `complete` | generated help; completion from a tolerant token scan |

## Key decisions

* **The registry is the grammar of calls.** `name(args)` is looked up in `Registry::catalog()`/`query_catalog()` (ops and queries share one
  namespace); the parser accepts any dotted name. A new op needs no change here.
* **One statement, one node; `batch` is `core.batch`.** Top-level statements execute one by one so entity lists see the state left by earlier
  statements (as in Cubit). A `batch { }` block is lowered without executing, against the block-entry state, using `$i.role` references: the same
  data a typed client would build, so a batch recorded from a script and from the GUI are the same node. References follow the contract's
  grammar exactly (`REF_PATTERN`, `[*]` only as an array element, `$$` escape for literals); a test applies the lowered batch and compares the
  state hash with applying the members one at a time.
* **Schema-guided conversion, then validation.** `conv` needs the schema to know that an entity set becomes an id string or an array, that an int
  must be written as a float for a `number` field, and where a batch reference is legal. `schema::check` then validates the result and maps each
  issue back to the sub-expression that produced it (descending into list and object literals and into `...{}` spreads).
* **The checker is conservative.** Anything it cannot interpret passes; `Registry::decode` (run on every top-level record) stays the authority, and
  the stored record is the canonical re-encoding `OpRecord::of(decoded)`. One deliberate stricter behaviour: unknown argument names are an error
  even where the registry's decoder would drop them silently.
* **No I/O, no `Meta`.** The language reads files only through `Host::load` and never builds a timeline `Meta` (author/time are not its business and
  time is non-deterministic input). A `Session`-backed host belongs in sm-engine/sm-cli.
* **Decisions that scripts depend on are made by parse errors where the alternative readings differ** (`in` before a set operator, `-`/`except` mixed with
  `|`/`&`): relaxing an error later is compatible, changing a meaning is not. `Expr::paren` exists only for this.
* **Timeline control is not an op.** `timeline.*` statements go to `Host::timeline`, never to `apply`; a session host maps them to `Session::branch/checkout/undo/redo`.
* **Op outputs are ordered lists** (`EntitySet` with `sorted == false`), so `b.role[k]` outside a batch is the entity `$i.role[k]` names inside one.
* **Plan mode** (`PlanHost`) validates and lowers without executing. Outputs that depend on execution are symbolic (`$plan.n.role` placeholders that
  can never match `REF_PATTERN`) with a `plan-unresolved` warning, and arithmetic on them is an error.
* **Adjacency is not on `State`.** `kind in X` calls a host `Relations` resolver; results are sorted, de-duplicated and filtered to live entities of the
  requested kind, so a resolver cannot affect determinism. `QueryRelations` adapts a registered query (shape in `W1_REPORT.md`).

## Determinism

* Output is a pure function of (text, registry, state, relations). Maps are `BTreeMap`, sets are sorted vectors (`EntitySet`), object keys sort;
  no `HashMap` iteration, no time, no RNG (the language has no random function).
* The only parallel code: set algebra on inputs of at least 65,536 ids, `live_range`/`all_of` over at least 65,536 serials, `parse_many`. The set
  algebra splits the left operand into fixed 16,384-element chunks and cuts the right operand at the same *values*, so the result is the same for
  any chunk size and thread count (tested against a `BTreeSet` oracle for chunk sizes 1, 2, 3, 7, 64 and sequential); ranges and `parse_many` use
  indexed parallel iterators, which collect in order.
* Tests: `tests/determinism.rs` runs a script over 200,000 entities through `sm_testkit::determinism::check(2, 4, ..)` (1 and 4 rayon workers, plus 4 concurrent calls) and
  separately on 1 and 8 workers and compares a hash of every record (`canonical_bytes` of the params) and of the final state; the whole suite is also
  run with `RAYON_NUM_THREADS=1`.
* Floats: results are exactly what `f64` arithmetic and libm give; they are written into records, so replay never re-evaluates the script. Transcendental
  functions may differ in the last bit across platforms (the same caveat as the rest of the project); re-running a script on another platform can
  therefore produce a different record, never a different replay of an existing one.

## Robustness

* The lexer and parser never panic and always terminate (proptest over arbitrary text, token soup, truncations and mutations of a valid script,
  with pinned seeds). Hostile input is bounded: source size 16 MiB, nesting 64, expression height 256 (a left-leaning tree as deep as the chain is long would overflow the stack on drop; since polish0 a run of one
  left-associative operator is a flat `ExprKind::Chain`, so generated scripts may chain thousands of terms and only nesting and operator-alternating chains hit the limit), 100 errors per source, 1,000,000 loop iterations, `range` capped. A
  linear-time lookahead for `a.b.c(` was needed to keep pathological `x.a.a.a...` chains linear (found by a test).
* Error recovery is statement-level (resynchronize at a separator outside brackets, or at a statement keyword at the start of a line, so one
  unclosed `(` does not swallow the rest of the file) and argument-level (skip to the next `,`/`)`), so one run reports many errors; if any exist,
  nothing executes.

## Performance (4-core container, criterion, `cargo bench -p sm-lang`)

| bench | result |
|---|---|
| parse a 2000-statement script (~90 KB) | about 3.0 ms, 30 MiB/s |
| lower the same script in plan mode (schema checks, decode gate) | about 13 ms |
| set algebra on two 1M-id sets, sequential / chunked parallel | intersection about 1.9 / 1.2 ms, difference about 2.2 / 1.3 ms; union is memory bound and does not gain (3.9 / 4.6 ms) |
| `all faces` over 1M allocated serials | about 9 ms |

Union gains nothing from chunking at this size; it is kept chunked only so that all three operations share one code path.

## W2 delta

* `session.*` is a second reserved command namespace beside `timeline.*`, sharing its argument reader (`command_args`) and event/plan-mode conventions; the host contract is the defaulted `Host::session`.
* Static checks read the registry's metadata instead of guessing: `x-entity-kind` (kept as a sibling of `$ref`, so `View` remembers the node as written) gives `wrong-kind`; `OpInfo.outputs` gives role did-you-mean in batch and plan
  mode and lets a bare `One` role lower to `$i.role[0]`. Ops that declare nothing behave exactly as in W1.
* Per-statement cost is unchanged (one extra map lookup per field access on a batch/plan result).

## Known gaps and non-goals (v1)

* No user-defined functions, no `where`/`named` predicates (reserved, `LANGUAGE.md` §5.5), no dictionaries of ops.
* A batch reference inside a list or object variable, at any depth, *is* rebound after the batch applies (fix1); what stays unavailable is `created`/`removed` and a
  selection past the end of a role (`LANGUAGE.md` §4.4).
* `Event::Batch.record` is the record the client built; the canonical stored form comes from the engine/session (see `W1_REPORT.md`).
* Completion is lexical: it does not infer the types of expressions beyond the enclosing call parameter; fields are offered only for variables that
  hold an op result or an object.
* `QueryRelations` and `Relations` define a convention that the topology plugin has to implement (`W1_REPORT.md`).
