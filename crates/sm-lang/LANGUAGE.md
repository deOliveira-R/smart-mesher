# The smart-mesh command language (`smesh`), v1

Status: design for W1 (revision `polish0`: the semantic decisions below are binding because scripts will depend on them), implemented by this crate. Binding facts come from the frozen contracts in
`sm-core` (entity ids) and `sm-op-api` (op/query registry, JSON schemas, `core.batch`).

A script is plain UTF-8 text, conventionally `*.smesh`. It is **not** Cubit-journal compatible and does
not try to be.

## 1. Positioning

| | Cubit journal | Gmsh `.geo` | OpenSCAD | **smesh** |
|---|---|---|---|---|
| Model | Verb-first free-form commands, one string per line; grammar per command | Imperative entity builders with hand-numbered ids (`Point(1) = {..}`) | Pure functional CSG expression tree; no mutable state, no ids | Statements that are **op invocations**: a registered op name plus named, schema-checked arguments |
| Variables | APREPRO (`{x = 3}` text substitution *inside* command strings) | `x = 3;` and macros | `x = 3;` (single assignment) | `let x = 3`, typed values, no text substitution; one expression grammar everywhere |
| Selecting entities | Id lists in the command text, `all`, `except`, `in`, with per-command parsing quirks | Hand-tracked integer ids | Not applicable (no model database) | One entity-list grammar (§5) used in every argument position: typed ids, ranges, `all faces`, `faces in Volume#2`, set algebra |
| Errors | Mostly "syntax error" for the whole line; commands applied before a failure stay applied | Line number only, parser aborts | Line/column, stops | Every diagnostic has a byte span, code, label, help; parse errors are recoverable and **nothing runs** if the script does not parse |
| Help / completion | Hand-written help pages, separate from the grammar | None | None | Generated from the op JSON schemas: help text, argument completion, and checking use one source |
| Machine use | Not designed for it | Not designed for it | Not designed for it | A call is the same data as an MCP tool call (`name` + JSON params); `...{json}` spreads a raw JSON object |
| History | Journal file is a log | Script is the model | Script is the model | Each op statement becomes one timeline node; `batch { }` becomes one atomic `core.batch` node |

smesh is **similar to** Cubit journals in being a replayable, command-per-step log and in the `all`/`except`/`in`
entity vocabulary users already know; **differs in** having no per-command grammar (one call syntax, one
expression language, schema-driven checking, deterministic evaluation) and no text-level macro
substitution; **giving** a language whose statements are 1:1 with the recorded op log and the MCP tools,
so a script, a GUI action, a Python call and a Claude tool call are interchangeable, and whose help,
completion and validation cannot drift from the ops because they are generated from the same schemas.

## 2. Lexical structure

* **Source**: UTF-8. Byte offsets are the span unit; `Span { file, lo, hi }`.
* **Whitespace**: space, tab, CR. A line feed is a statement separator, except that line feeds are ignored
  inside `( )` and `[ ]`, directly after a binary operator or `,` (unless the next line starts with a statement keyword, which then ends the
  dangling statement), and after `\` at end of line.
* **Separators**: newline or `;` (empty statements allowed). `else` may start the line after the closing `}` (Allman style) or follow it on the same line.
* **Comments**: `// to end of line`, and `/* block */` (nests; unterminated is an error). `#` is *not* a comment:
  it is the entity-id sigil (`Volume#3`), matching the `Kind#serial` text form of `sm-core`.
* **Identifiers**: `[A-Za-z_][A-Za-z0-9_]*`. Dotted names (`model.create_brick`) are parsed as identifier paths; after a `.` any word,
  reserved or not, is a valid segment (`core.batch`, `b.all`), because op and output names are not ours to choose.
* **Numbers**: `123`, `1_000`, `1.5`, `2e-3`, `1.5E+2`. Maximal munch: a `.` belongs to a number only when a digit follows it, so `1.` and `.5` are
  not floats (write `1.0`, `0.5`) and `1..5` lexes as `Int(1)` `..` `Int(5)`. Integers are `i64` (overflow is an error); anything with `.` or an
  exponent is `f64`. No `NaN`/`inf` literals. Malformed numbers get one targeted `invalid-number` error with the fix in its help: `1.e5` and `1.` (write `1.0e5`,
  `1.0`), `1_`, `1__0`, `1_.5` (a `_` separates digits, one at a time: `1_000`), `1e`, `1e+` (an exponent needs digits: `1e5`).
* **Entity references** are a single token: an identifier *immediately* followed by `#` and then a digit, `[` or `(` (`Volume#3`, `Volume#[1,2]`,
  `Volume#(n + 1)`) lexes as `EntityRef(kind-name)` and the parser then reads the id spec (§5.2). No spaces are allowed around `#`; one span
  covers `Volume#`, so entity diagnostics point at it. A `#` anywhere else is an error (`Volume#i` explains that a variable goes in brackets: `Volume#[i]`).
* **Strings**: `"..."`, escapes `\n \t \r \" \\ \$ \u{HEX}`; may not contain a raw newline. **A string literal is literal text, forever.**
  `{`, `}` and `$` mean nothing inside a plain string, so `let i = 3; print "fuel_{i}"` prints `fuel_{i}`. The language does not interpolate: build names with
  `"fuel_" + str(i)`. If interpolation is ever added it will use a **prefixed** literal (`f"fuel_{i}"`), which cannot change the meaning of a v1 script;
  `f"..."` is the error `reserved-prefix` today (an identifier directly followed by a string literal; help: `"..." + str(...)`), and the prefix is reserved for that. Two guards keep the decision honest:
  a plain string that contains `{name}` gets the warning `literal-braces` (help: `+ str(...)`), and an unescaped `${` is the parse error
  `reserved-interpolation` (help: write `\${` for a literal `${`, or use `+ str(x)`), so the APREPRO spelling `"pin_${i}"` can never silently mean
  something else later. A lone `$` (as in the batch reference text `"$0.body"`, or `"cost $5"`) is ordinary. `literal-braces` stays quiet for a `{name}` that follows
  an escaped `\$` (`"cost \${name}"`, the writer already said it is literal) and for the `{` of a `${` that was already reported as `reserved-interpolation`.
* **Punctuation / operators**: `( ) [ ] { } , ; : = . .. ... #`, `+ - * / % **`, `== != < <= > >=`, `| &`.
* **Reserved words**: `let set alias batch if else for in include print assert true false and or not all except null`, plus the **future words**
  `fn def return while break continue import` (user functions and loops are the obvious next features; using one as a name is the error `reserved-word`, so
  adding the syntax later cannot change a valid script: `return (x)` would already parse as a call). After a `.` they are still valid path segments
  (`b.return`), and quoted as object keys (`{ "return": 1 }`). `help`, `by` and the plural
  entity keywords (`faces`, `volumes`, ... §5) are *contextual*: they are keywords only directly after `all` or directly before `in`, so
  they remain usable as variable names elsewhere. `where`, `named` and `with` are contextual too, and reserved for entity predicates (§5.5):
  directly after an entity expression they are a `not-yet-supported` error, everywhere else they are ordinary names.
  The name `timeline` followed by `.` is a reserved namespace for timeline control (§4.6); it is not a registry namespace.
  The Cubit element-type words `hexes tets quads tris` are contextual and reserved as future plural selectors (§5.1): directly after `all` or before `in` they are a
  `not-yet-supported` error, elsewhere ordinary names.
* **`null` is a reserved word, and not a value.** The op-api params model has no null: an absent optional parameter is simply omitted. `let null = 1` is the error
  `reserved-word`; `null` in an expression is the error `no-null`, whose help says to leave the argument out. The one place it is accepted is as the **value of an
  object-literal field**, including an object literal spread into a call, where `key: null` means "omit this field": `model.create_brick(...{ x: 1, y: 1, z: 1, name: null })`
  is the call without `name`. Only the literal token `null` directly as the value counts; `{ name: if c { "a" } else { null } }` is an error. A duplicate key is still `duplicate-key`, null or not. After a `.` it is an ordinary path segment, like every reserved word.

## 3. Values and expressions (APREPRO replacement)

Values: `Bool`, `Int(i64)`, `Num(f64)`, `Str`, `List`, `Obj` (string keys), `Ents` (an ordered set of entity ids, §5),
and `Outcome` (the result of an op, §4). Expressions are evaluated eagerly, left to right, with no side
effects, so a script's lowered form is a pure function of (script text, registry, state).

Precedence, loosest to tightest: `a..b` (inclusive range); `or`; `and`; `not`; comparison (non-associative); `|` (set union);
`&` (set intersection); `+ - except`; `* / %`; unary `-`; `**` (right-associative); postfix `.field`,
`[index]`. `-` and `except` are difference on `Ents`; `+` concatenates strings and lists (on entity sets use `|`; `+` there is an error that says so).

**Mixing difference with union or intersection is a parse error unless parenthesized** (`ambiguous-set-expression`): `all faces - Face#1 | Face#2`
and `all volumes except Volume#1 | Volume#2` do not parse; write `(all faces - Face#1) | Face#2` or `all volumes except (Volume#1 | Volume#2)`.
The two readings differ (Python-style precedence keeps `Volume#2`; the English word `except` excludes it), so the language makes the writer choose.
Turning an error into a meaning later is compatible; changing a meaning is not. The same rule applies to `kind in X` (§5.2).

* Arithmetic: `Int op Int` is `Int` (checked, overflow is an error) except `/`, which is always `Num`;
  mixed arithmetic promotes to `Num`. Division or modulo by zero is an error (never `inf`/`NaN`). `%` is the **truncated remainder** (sign of
  the left operand, like C and Rust): `-7 % 3 == -1`, not Python's `2`. `**` with a negative integer exponent is a `Num` (`2 ** -1 == 0.5`);
  `2 ** 63` overflows.
* **`==` and `!=` between different types are an error** (`type-mismatch`), except `Int` with `Num`, which compare numerically: `Face#1 == "Face#1"` is a mistake, not
  `false`, and its help names the conversions (`ents("Face#1")` to compare entities, `str(Face#1)` to compare text). Same-type comparison is structural; a list or object
  that merely *contains* mixed types compares element by element and is unequal where the types differ (`[1] != ["a"]`), and `contains(list, v)` is unaffected.
* Comparison of an `Int` and a `Num` is **exact** (no rounding of the integer through `f64`): `9007199254740993 == 9007199254740992.0` is false.
  `min`/`max` of all-`Int` arguments are exact `Int`s. Passing an `Int` to a `number` parameter converts it to a float only when that is exact
  (|n| at most 2^53), else `integer-precision` (write it as a float if the rounding is intended).
* Builtins (pure, deterministic; names are not reserved but cannot be called as ops):
  `abs min max floor ceil round sqrt sin cos tan asin acos atan atan2 exp ln log10 pow int float
  len count range str ents`, constants `pi`, `tau`. `floor`/`ceil`/`round`/`int` return `Int` (checked); `round` is **half away from zero** (`round(-2.5) == -3`, Python rounds to even) and `int`
  truncates toward zero (`int(-0.5) == 0`); `contains(set, id)` / `contains(list, v)` / `contains(str, sub)` / `contains(obj, key)` is the membership test
  (there is no `x in set` operator: `in` needs a plural kind on its left); `ents("Volume#3")` /
  `ents([..])` turns id strings (for example from a query result) into an entity set (§5.1: `Surface#`/`Curve#` in a string is ambiguous), and every float result is checked finite (`non-finite`). There is deliberately **no random** function: any future
  randomness must be seeded from op parameters. `range(a, b[, step])` is half-open and capped (§9). Half-open is spelled `range` and nothing else: `..` exists **only inside id specs** (§5.2), where it is
  inclusive, and so is the **range expression** `a..b`: the list `a, a+1, ..., b` (empty when `b < a`; both ends must be integers, `range-too-long` beyond `max_range_len`),
  written for loops: `for i in 1..n { ... }` is `for i in range(1, n + 1)`. `range(a, b)` stays half-open. The range is the loosest operator, so `1..n + 1` is `1..(n + 1)` and
  a comparison needs parentheses (`(1..3) == [1, 2, 3]`); `1..2..3` is `chained-range`. Inside an id spec the `..` is the id range: `Volume#[1..n]` selects ids, it is not this list.
  Transcendental functions use the platform libm; the value written into an op record is what is replayed, so
  cross-platform last-digit differences affect only *re-evaluating the script*, never replay of a timeline.
* Lists `[1, 2, 3]`, objects `{ x: 1, "y z": 2 }`, indexing `l[0]`, `l[-1]` (negative from the end),
  `len(l)`; `count(set)` is the number of entities of an `Ents`.
* **Conditional expression**: `if cond { a } else { b }` is an expression (`let r = if shifted { 0.6 } else { 0.475 }`). The `else` is required (`missing-else`; use the
  `if` statement for a conditional action), `else if` chains, the condition must be a bool, and only the chosen branch is evaluated. Each branch is one expression in braces
  (an object result is `{ {a: 1} }`). A line that starts with `if` is always the statement; inside `(`, `[` or a call, start the `if` on the line of the bracket or comma (a
  new line starting with a statement keyword there reads as an unclosed bracket).
* **Long chains.** `a | b | c | ...`, `a + b + ...` and every other run of one left-associative operator is stored flat, so a generated script may chain thousands of terms
  (`Face#1 | Face#2 | ... | Face#5000` parses and runs). The expression-height limit (§9) applies to nesting and to mixed-operator chains, never to a run of one operator.
  A parenthesized operand stays its own unit. List and string `+` chains append in place.
* Variables: `let x = expr` binds once per scope (rebinding in the same scope is an error); `set x = expr` rebinds an
  existing binding of any enclosing scope. Names resolve lexically; there is no implicit global mutation. A `let` inside a block that hides a variable of an enclosing scope
  is allowed but gets the warning `shadowed` (help: use `set` to change the outer variable). `x = 2` is not an assignment: the error says `set x = 2` (or `let x = 2` for a new name).
  `set x = x + ...` on a list or string appends in place when nothing else holds the value, so building a list or string in a loop is linear (200,000 appends take milliseconds);
  the value is still copied if another variable holds it.
* Control flow: `if cond { } else { }` (statement), `for i in list_or_set { }` (also `for i in 1..n { }`). Loops are bounded (§9). No user functions in v1.
* `print e1, e2` emits text; the items are joined with **one space**, as in Python, with no trailing space (`print "x =", 5` prints `x = 5`; `print "a", ""` prints `a `). Write the
  separator you want yourself with `+`: `print "x=" + str(5)`. Printing (or `str()`) of an **op result** shows its roles, `{body: Body#1, faces: Face#[1..6], volume: Volume#1}`
  (alphabetical, not the op's order; a result from inside a batch shows only the named outputs). `assert cond[, "message"]` fails the script with a span if `cond` is false. Printing is meant to be pasted back:
  floats always read as floats (`5.0`, `1e300`, `1.5e-7`; shortest round-trip digits, never hundreds of digits). **Entity sets print in source form** with the
  preferred alias of each kind and consecutive serials compressed per kind: `Face#[1..6] | Volume#2` (a single id is `Face#5`; a 200,000-id range stays short). An
  **ordered op-output list** (§4.3) prints as a list in its own order, `[Body#1, Volume#1, Face#1]`, so it is visibly not a sorted set (a list whose order happens to be
  sorted is a sorted set and prints as one). The empty set has no source literal and prints as `<no entities>`, the one text that does not paste back. The recorded
  ops stay canonical. Op error messages are rewritten the same way (`Volume#1`, not `TopoVolume#1`), so an id copied from an error pastes into a script.
* Value sizes are bounded (`max_value_len`, §9) for strings and lists built by `+`, `str`, `range` and query results.

## 4. Statements and their meaning

```
script  := { stmt sep }
stmt    := 'let' IDENT '=' expr | 'set' IDENT '=' expr | 'alias' IDENT '=' dotted
         | call | 'batch' block | 'if' expr block ['else' (block | ifstmt)]
         | 'for' IDENT 'in' expr block | 'include' STRING
         | 'print' expr {',' expr} | 'assert' expr [',' expr] | 'help' [dotted]
call    := dotted '(' [ arg {',' arg} [','] ] ')'
arg     := IDENT ':' expr         // named argument
         | '...' expr             // spread an object (raw JSON params, or a variable)
         | expr                   // positional (shorthand, see below)
block   := '{' { stmt sep } '}'
dotted  := IDENT { '.' IDENT }
```

### 4.1 A call is an op or a query

`name(args)` where `name` is a dotted lowercase registry name (`model.create_brick`). Ops and queries share
one namespace in the registry, so the name decides: in `Registry::op_names()` it is an op, in
`query_names()` a query. Unknown names get a did-you-mean from both lists. Names are always written in full: a script's
meaning must not depend on which plugins happen to be installed (a replayable log, and an LLM caller, want explicit names).
The only sugar is `alias brick = model.create_brick`, a script-scope pure rewrite applied before lookup (an alias never shadows a
registry name, and its target must exist when the alias is defined). **An alias follows the rebind rule of `let`**: defining the same alias twice is the error `rebind`
(a session with `allow_top_level_rebind` gets the `shadowed` warning instead). An alias whose name equals a registry namespace (`alias model = ...`, easily mistaken for
`model.*`) or an existing variable gets the warning `alias-confusing`.

### 4.2 Arguments to params

1. Arguments are collected into one JSON object **in source order**; a later source overrides an earlier one, so
   `f(...base, x: 5)` overrides `x` from `base`, while `f(x: 1, ...{x: 2})` gives `x = 2`. A *named* argument written twice is
   `duplicate-argument`.
2. **One positional argument** is allowed when the op has exactly one *required* property: it binds to that property. A schema may name the
   positional parameter explicitly with `"x-positional": true` on one property (schemars: `#[schemars(extend("x-positional" = true))]`); that
   marker wins, so a later version that adds a required parameter does not break existing positional scripts. Any other positional use is
   `positional-ambiguous`. (A first-class marker in the `sm-op-api` contract is requested in `W1_REPORT.md`.)
3. Each value is converted to JSON under the guidance of the parameter's schema (§6): an `Ents` becomes an id string or an
   array of id strings; a string literal naming an entity (`"Volume#3"`) is normalized through the alias table;
   an `Int` is accepted for `number` and `integer`; a `Num` only for `number`.
4. The object is checked against the op's schema (§6); only then is `OpRecord::new(name, registry.op_version(name), params)` built
   (which canonicalizes: no nulls, `-0.0` folded). Every record, top-level or a `core.batch` member, carries its own current `version`.
   Top-level records are also decoded with `Registry::decode` as a last gate, so the statement is known to apply before it is sent
   (batch members cannot be decoded early because their params may hold `$` references).
5. A **query** is the same, but its record is never stored; its JSON result is the value of the call. Where the query's result schema says "entity id"
   or "array of entity ids" (the `EntityId` pattern), those strings become entity sets (`Ents`), also inside objects and lists, so
   `bc.sideset(faces: geom.find_faces(normal: [0, 0, 1]))` and `count(geom.find_faces(...))` work.
6. **Enum values are strings**: `scheme: "map"`. A bare `scheme: map` is an `unknown-variable` error whose help says to quote it. This is
   decided: bare identifiers are *not* enum values in v1, and if they ever are, a variable of the same name always wins (so existing scripts keep
   their meaning). A misspelled value gets a did-you-mean from the allowed values.
7. Unknown argument names are an error (`unknown-argument`, with a did-you-mean), even where the op's decoder would silently ignore
   them: a typo must not turn into a default.

An op call is a statement, or the whole right-hand side of `let`/`set`; inside a larger expression it is `op-in-expression` (ops change the
state). A **timeline command** (§4.6) is a statement only: it has no value, so `let x = timeline.undo()` is also `op-in-expression`. A **query** is read-only and deterministic for a given state, so it may also appear inside expressions (`assert model.count().count == 6`,
`intent.set_size(ids: topo.related(...), size: 1)`), but not inside a `batch` (`query-in-batch`).

### 4.3 Execution model, ids and results

Top-level op statements execute one at a time through a `Host` (an engine session in production; a plain
`Engine`+`State` in tests; a recording `PlanHost` for validation without execution). **One statement is
one timeline node.** Entity lists (§5) are resolved against the host's *current* state immediately before
the statement runs, so `all volumes` after a `create_brick` sees the new volume.

`let b = model.create_brick(x: 1, y: 1, z: 1)` binds an `Outcome`: `b.created`, `b.removed`, and one field per output
role the op allocated (`b.body`), each an `Ents`. `b.body[0]` is the first id, `b.body[*]` the whole set.

**Order.** `b.created`, `b.removed` and every `b.role` are **ordered id lists**: they keep the order in which the op reported the ids
(`Outcome::Concrete.created` is "ids created, in order"), so `b.created[0]` is the first entity the op allocated, and `b.role[k]` is the same entity
as `$i.role[k]` inside a `batch` (which indexes the op's own order), even when a role mixes kinds. Everything built from ranges, `all`, `kind in`
and the set operators (`| & - except`) is a **sorted set** (by kind, then serial); set algebra always returns a sorted set, whatever its inputs.
`[k]`, `for`, `print` and conversion to an array parameter follow the stored order; equality and `contains` ignore it.

**Partial application.** A script run is not transactional: statements that completed before a runtime error stay applied (the Cubit weakness §1
mentions applies to scripts too). Parse errors are different: nothing runs. Use `batch { }` where an all-or-nothing step is needed; whether a
whole-run atomic mode is worth adding is open (`W1_REPORT.md`).

**Plan mode** (`PlanHost`: validate and lower without executing) cannot know an op's outputs: the number of ids an op allocates is not
predictable from the state. There, a top-level `let b = op(...)` binds a *symbolic* outcome (the op is schema-checked, and its record is
collected). Using `b.role` as an argument produces a `plan-unresolved` warning and a placeholder `"$plan.<n>.<role>"` string that is
**not** valid in a record; plan output is for validation, completion and review, not for application. Wrap dependent
statements in `batch { }` to get a fully lowered, applicable form without executing anything. A `for` loop over an unknown sequence
(`for f in b.faces { ... }`) does not fail the plan: its body is checked **once** with a placeholder loop variable under a `plan-unresolved`
warning, and a `plan-unresolved` error inside that check is downgraded to a warning and ends it. An **`if` whose condition depends on a placeholder** (`if r.count > 3`)
gets the same treatment: the `plan-unresolved` error becomes a warning and **every branch is checked once** (a `then`, each `else if`, the `else`), so a plan may list the ops of
branches the real run will not take; a condition that is known takes its one branch as usual. An `if` *expression* on a placeholder yields a placeholder. A query used in an expression yields a placeholder too.

### 4.4 `batch { ... }` is `core.batch`

A `batch` block lowers to **one** `core.batch` record (one atomic timeline node, one undo step). Inside it:

* ops are collected, not executed; the batch index `i` counts **only emitted ops** (an unrolled `for` contributes one index per emitted op;
  `let`, `if`, `print`, `assert` contribute none), and each member record carries its own `version`; entity lists resolve against the state *at block entry* (documented snapshot
  semantics; explicit ids of entities created inside the block cannot exist yet: `entity-not-found` then says that ids created inside a block cannot be named and to use
  the op result). `all`, a range, or `kind in X` written **after an emitted op** of the same block gets a `batch-snapshot` warning, because the selection silently leaves
  out what the block created;
* `let b = op(...)` binds a **symbolic** outcome of batch index `i`. `b.role` becomes the reference string
  `"$i.role"`; `b.role[k]` becomes `"$i.role[k]"`; `b.role[*]` becomes `"$i.role[*]"`. The contract only accepts `[*]` as an **array element**,
  so the lowering is chosen by the schema of the parameter the value lands in. **Wrapping code in `batch` never changes what it means:** a scalar id
  parameter needs exactly one id, so a bare `b.role` there is `expected-one-entity` in a batch just as it is outside one when the role holds several ids
  (a batch cannot know how many, so it never silently takes the first); write `b.role[0]`, which lowers to `"$i.role[0]"`. **Exception (W2):** when the op *declares*
  the role with cardinality `One` (`OpInfo.outputs`), a bare `b.role` is that one id and lowers to `"$i.role[0]"` — byte-identical `core.batch` to the explicit form, and the same state
  as the top-level statement. Roles declared `Optional`/`Many`, and ops that declare no roles, keep the error. Array-of-id parameter, `b.role` or `b.role[*]` → `["$i.role[*]"]`; `[b.role[0]]` → `["$i.role[0]"]`; inside an array literal a bare `b.role` is
  spliced as `"$i.role[*]"`. `b.role[*]` for a scalar parameter is an error; Symbolic values may only be used as op arguments (or list elements of them);
  arithmetic or set algebra on them is `not-concrete`;
* a user string literal starting with `$` is emitted as `$$...` (the batch escape), so literals can never be mistaken
  for references; reference strings are validated against `sm_op_api::batch::REF_PATTERN`;
* two things work at top level but not in a batch, because the reference grammar (`REF_PATTERN`) cannot express them: `q.faces[-1]` (no negative index; write an
  explicit `[k]`) and `for f in q.faces` (a batch cannot know a role's cardinality; pass `q.faces` to an array parameter, or move the loop after the batch, where the
  output is concrete). Both are diagnosed with that reason;
* a misspelled output role (`q.voluem`) is `unknown-field` with did-you-mean **at the field**, in a batch and under `PlanHost`, when the op declares its output roles (`OUTPUTS`, W2).
  For an op that declares none the role is only checked when the batch is applied, and the failure is mapped back to the argument that wrote the reference;
* queries, `include`, and nested `batch` are errors inside a batch (queries are not recorded; nested batches are rejected by the contract);
  `let`, `set`, `if`, `for`, `print`, `assert` evaluate normally (so a `for` loop unrolls into the batch);
* a batch does not open a scope. When it has been applied, symbolic outcomes are re-bound to concrete `Ents` built from
  the batch `Delta.outputs` (`"<i>.<role>"`); `b.created`/`b.removed` are unavailable for them (`not-concrete`).
  Under `PlanHost` they stay symbolic. References held directly (`let f = p.faces`) and inside lists and objects, at any depth (`let l = [p.body[0]]`), are rebound.
  A selection past the end of a role (`p.body[5]`) cannot be rebound; using it afterwards is `stale-reference` with the reason ("the batch produced no output").

### 4.5 Scripts and includes

A script file is a statement list. `include "path"` asks the host's loader for the text of `path`
(`Host::load`; the language does no I/O itself), parses it as a separate source (so spans point into the right file), and runs it in the
**current scope**. Cycles and depth beyond 16 are errors. Parsing is all-or-nothing per source file: if any source fails to parse,
**no statement of the run executes**; every syntax error found by recovery is reported.

### 4.6 Timeline control

Fork, checkout, undo and redo are central to the project, and they are **statements of the language but not registry ops**: a fork is not itself a
timeline node, so recording it as one would be circular. They live in a reserved namespace, `timeline.*`, dispatched to
`Host::timeline(&TimelineCommand)` (default: error `timeline-failed`/`timeline-unsupported`), never to `Host::apply`:

```
timeline.branch("alt")               // fork: create a branch at the current position and attach to it   (Session::branch)
timeline.checkout("main")            // a branch by name                                                   (Session::checkout, Rev::Ref)
timeline.checkout(node: "<hash>")    // a node, detached                                                   (Rev::Node)
timeline.checkout(empty: true)       // the empty state                                                    (Rev::Empty)
timeline.undo()    timeline.redo()                                                                          (Session::undo / redo)
```

`timeline.branch("x", at: "<hash>")` forks at an earlier node instead of the current position (`Workspace::fork(.., at)`).
Revisions are the contract's `sm_op_api::Rev`: the interpreter parses the hash text, so a malformed `node:` or `at:` is `invalid-argument` at that argument and never
reaches the host. `TimelineCommand` is `#[non_exhaustive]` (new verbs such as `tag` or `log` will be added). One string may be positional (`branch(name)`, `checkout(ref)`).
They are statements only and have no value (`op-in-expression` inside an expression or on the right of `let`/`set`; `timeline-in-batch`
inside a batch: a batch is one timeline node). `timeline.head()` is the one command with a value: it returns the object `{branch, node}` (the attached branch, absent when detached; the current node hash, absent in the empty
state; the language has no null, so an absent field is missing) and may be used in expressions (`print timeline.head().branch`, `assert timeline.head().branch == "main"`). It
comes from the defaulted `Host::timeline_head` (default: `timeline-failed` / `timeline-unsupported`), so existing hosts compile unchanged; under `PlanHost` it is a placeholder. Every other command
becomes an `Event::Timeline`. Under `PlanHost` nothing happens and a `plan-timeline` warning says
later statements are checked against the state as it is. The namespace is shadowed by the language: a registry op named `timeline.x` would be unreachable,
so plugins must not register that prefix. **Variables are not rolled back** by `undo`/`checkout`: they keep the ids they hold, and entity lists are
resolved against the new current state when used (an id that no longer exists is `entity-not-found`). `help timeline.branch` documents each command.

### 4.7 Session commands

`session.*` mirrors `timeline.*`: statements of the language that are **not** registry ops (they are not recorded in the timeline), dispatched to `Host::session(&SessionCommand)` (default: `session-failed` /
`session-unsupported`). Every command becomes an `Event::Session { command, applied, result }`.

```
session.export("out.vtu")                                         // path is positional;         SessionCommand::Export
session.export("out.smsh", format: "smsh", mesh: m.mesh, provenance: true)
session.save()                                                    //                              SessionCommand::Save
let h = session.hash()      print session.status().branch         // values, usable in expressions SessionCommand::Hash / Status
```

* `export(path: string, format?: "smsh" | "vtu", mesh?: id, provenance?: bool)`. `path` is a non-empty string; `format` is validated here (`invalid-argument` with did-you-mean at the argument; when absent the
  host infers it from the extension); `mesh` is one id of kind `Mesh` (`wrong-kind` otherwise; when absent the host picks, or fails with its own `no-mesh` / `ambiguous-mesh`); `provenance` defaults to false.
  The language does no I/O and does not resolve paths: the host does.
* `export` and `save` are statements only (`op-in-expression` in an expression or on the right of `let`/`set`). `hash()` and `status()` have a value, the host's JSON converted like a query result
  (an object becomes an object, a string a string), and may be used in expressions.
* All four are errors inside a batch (`session-in-batch`: a batch is one timeline node and a session command is not an op).
* A host failure is `session-failed` carrying the host's code and message; the failing statement had no effect.
* Under `PlanHost` the host is never called: a `plan-session` warning, an `Event::Session` with `applied: false`, and `hash()`/`status()` give an unknown (placeholder) value. A `mesh:` that is itself unknown in a
  plan is recorded as `None` in the event.
* The namespace is reserved like `timeline.`: plugins must not register ops with the `session.` prefix.

## 5. Entity references and entity lists

All entity expressions evaluate to `Ents`: a duplicate-free set of `EntityId` ordered by `EntityId` (kind, then serial), so output never
depends on how a set was built.

### 5.1 Typed ids and the alias table (frozen by a test)

`Kind#serial` with **no spaces** around `#`. `Kind` is either the canonical `sm-core` kind name (`TopoVolume`, `MeshCell`, `Color`, ...)
or one of these aliases:

| alias | kind | plural keyword | | alias | kind | plural keyword |
|---|---|---|---|---|---|---|
| `Vertex` | `TopoVertex` | `vertices` | | `Node` | `MeshVertex` | `nodes` |
| `Edge`, **`Curve`** | `TopoEdge` | `edges`, **`curves`** | | `Element` | `MeshCell` | `elements` |
| `Face`, **`Surface`** | `TopoFace` | `faces`, **`surfaces`** | | `GeomCurve` | `Curve` (carrier) | `geom_curves` |
| `Volume` | `TopoVolume` | `volumes` | | `GeomSurface` | `Surface` (carrier) | `geom_surfaces` |
| `Body` | `TopoBody` | `bodies` | | | | |

**Cubit vocabulary wins.** In Cubit `surface` and `curve` are the topological faces and edges, and `sm-core` itself calls `TopoFace` "(surface)". So in
scripts `Surface#5`, `all surfaces` and `surfaces in Volume#2` are **faces**, and `Curve#n`/`curves` are **edges**. The geometry carriers (`sm-core`'s
`Surface` and `Curve` kinds) are spelled `GeomSurface`/`GeomCurve` and `geom_surfaces`/`geom_curves`; the canonical names `Surface`/`Curve` are *not* accepted
for them in source, so a carrier id can never be named by accident. An empty `geom_surfaces` selection says what the carriers are. This table is frozen by a test.

Every other kind name is also its own alias, and has a snake_case plural keyword (`Color`/`colors`, `Group`/`groups`,
`Boundary`/`boundaries`, `PartitionCell`/`partition_cells`, `Mesh`/`meshes`, `MeshFace`/`mesh_faces`, `Symmetry`/`symmetries`, ...; the full table
is `kinds::KIND_TABLE`, tested to cover `EntityKind::ALL`). Unknown kinds are `unknown-entity-kind` with a did-you-mean.

**Strings: `Surface#n` and `Curve#n` are ambiguous, so they are an error.** `sm-core` prints the geometry carriers as `Surface#7`/`Curve#7`, and the same text is
a face/edge in bare source. In a string a *user wrote* (a string argument for an id parameter, the argument of `ents()` and `contains()`) these two prefixes are the
error `ambiguous-kind` (help: write `Face#n` or `GeomSurface#n`), including an id built as `"Surface#" + str(i)`. Everything else is accepted: `"Face#3"`,
`"Volume#3"`, `"GeomSurface#3"` and the canonical names (`"TopoFace#3"`). This is an error today so that it can be relaxed into a meaning later. Ids
inside **query results** are wire form: where the result schema says "entity id" they become `Ents` read canonical-first, so a carrier id an op or query reported is
still the carrier. The canonical text `Surface#n` also appears in records and JSON events; those are not scripts. **JSON events stay in wire form on purpose**: `Event::Op.record`, the
`params` and `result` of `Event::Query` are exactly what the host applied, stored and hashed (rewriting a record would change its hash), so a geometry carrier shows there as
`Surface#7`, not `GeomSurface#7`, and a face as `TopoFace#1`. Messages are different: op, query and batch errors, and therefore `diagnostics_json`, are already in source
spelling. For a person or a model that reads events next to the script, `sm_lang::respell_json(&json)` returns a display copy with every canonical id in a string rewritten
(`TopoFace#1` to `Face#1`, `Surface#7` to `GeomSurface#7`); the MCP layer should apply it to what it shows, never to what it stores. Ids named by strings are checked for liveness like
bare ids: `ents("Volume#9")` and `model.pick("Face#99")` are `entity-not-found`.

**Vocabulary differs from Cubit.** In Cubit, `face`, `edge` and `node` used as mesh-entity words mean mesh faces, mesh edges and mesh nodes, and `hex`, `tet`,
`quad` and `tri` are element types. Here `Face` and `Edge` are **topology** (the boundary of a volume, a curve of the model), mesh faces and edges are `MeshFace` and
`MeshEdge`, mesh nodes are `Node`, cells are `Element`. `hexes`, `tets`, `quads` and `tris` are reserved as future element-type selectors (`all hexes`,
`hexes in Volume#1`), currently the error `not-yet-supported`.

### 5.2 Forms

```
Volume#3                  one id (an integer literal)
Volume#1..5               inclusive range (live members only; a warning if none)
Volume#1..9 by 2          stepped range (1, 3, 5, 7, 9)
Volume#[1, 3, 5..7]       list of ids and ranges
Volume#[i]  Volume#(i+1)  serials are expressions inside [...] or (...):  Volume#[a..b by 2]   Volume#[i, j + 1]
all faces                 every live entity of the kind
faces in Volume#2         entities of that kind related to the right operand (see 5.3); `all faces in Volume#2` is the same
a | b    a & b    a - b   union, intersection, difference ('-' may be written 'except')
(…)  variable names       parentheses and variables holding an Ents
```

**Computed ids** replace APREPRO's `volume {i}`: inside `[...]` or `(...)` a serial is any expression that evaluates to an integer from 0 to 2^56-1
(`bad-id-range` / `type-mismatch` otherwise); the bare `Volume#3` form stays a literal so that `Volume#3 - Volume#2` is a set difference. A single
computed id must be live like a literal one; in a range, live members are selected. `for i in range(1, n + 1) { mesh.run(targets: Volume#[i], ...) }` works.
`..` is inclusive and exists only here (see §3).

**`in` takes a postfix operand and must not be followed by a set operator.** `faces in Volume#1 | Volume#2` is an `ambiguous-in` parse error (it would
silently have been `(faces in Volume#1) | Volume#2`, a set of mixed kinds, while a Cubit user reads `faces in (Volume#1 | Volume#2)`). Write the
parentheses you mean: `faces in (Volume#1 | Volume#2)` or `(faces in Volume#1) | Volume#2`. `faces in Volume#2 - faces in Volume#3` needs them too.
`-`/`except` mixed with `|`/`&` is rejected in the same way (§3).

### 5.3 Liveness and relations

* A single explicit id (`Volume#3`) and the elements of an id list must be **live** in the host state (`State::contains`), else `entity-not-found` at that id's
  span. This is checked whenever the host executes (including inside a `batch`, against the block-entry state: a batch cannot name ids it creates;
  it uses references). A plan host (`executes() == false`) does not check explicit ids, because earlier statements were not applied.
* Ranges and `all` select live members only. An empty result is a *warning* (`empty-selection`), not an error: the op decides whether an empty
  set is acceptable.
* `kind in X` needs adjacency, which lives in the topology/mesh layers, not in `State`'s entity table. sm-lang therefore asks a host-supplied
  `Relations` resolver (`related(state, target_kind, within) -> ids`; "in" means contained-in or incident-to, in either hierarchy direction, as the layer defines
  it). `QueryRelations` adapts any registered query with input `{ "kind": "<Kind>", "within": ["<id>", ...] }` and output `["<id>", ...]`
  (the production adapter will point at the topology query). Without a resolver the error is `no-relations`. Results are sorted, de-duplicated and
  filtered to live entities, so a resolver cannot affect determinism.
* Resolution of large sets (`Volume#1..10000000`, set algebra over millions of ids) runs data-parallel with a fixed chunking and in-order
  concatenation, so results are bit-identical for any thread count.

### 5.4 Conversion to parameters

An `Ents` bound to an id parameter must contain **exactly one** entity (`expected-one-entity` with the count otherwise);
bound to an array of ids it becomes the array in stored order (id order for a sorted set, the op's order for an output list). It is a type error anywhere else.

### 5.5 Reserved: predicates and names (not implemented)

Selecting by attribute, name or location is the most common real-world selection (Cubit `with`, `name`, `at`). v1 has ids and relations only, but the grammar is
fixed now so scripts written today stay valid. After an entity expression, the contextual words `where` and `named` (and `with`) introduce a predicate:

```
faces in Volume#1 where area > 2                  // attribute / geometric predicate over the selected entities
faces in Volume#1 where normal == +z              // vector literals and axis names to be decided with the geometry layer
volumes named "fuel*"                              // selection by the name given with `name:` (glob)
all faces with name "inlet"                        // `with` is an alias of `where` for attribute-equality (reserved)
```

Name-based selection (`named`) and attribute predicates (`where`) are the **first language feature planned** once the geometry layer exists: for blocks, sidesets and sweep source/target
selection a script currently works only through queries plus `ents()`, which is the biggest readability gap against Cubit's `with name`/`at`.
Today each of them is a `not-yet-supported` parse error that points at the word. Elsewhere `where`, `named` and `with` remain ordinary variable names, and
adding predicates later cannot change the meaning of a valid v1 script. Until then a script that must find entities by property runs a query and passes
`ents(result)` (or the query result directly: §4.2.5).

## 6. Schema-driven checking

Each op's params schema is `Registry::schema(name)` (JSON Schema 2020-12, `$ref`/`$defs` resolved locally; `core.batch` uses `Registry::batch_schema()`).
The checker supports `type` (incl. type lists), `properties`, `required`, `additionalProperties: false`, `enum`, `const`, `items`/`prefixItems`,
`minItems`/`maxItems`, `minimum`/`maximum`/`exclusiveMin|Max`, `minLength`/`maxLength`, `pattern` (only the contract's id and batch-ref patterns are interpreted,
other patterns are left to the op's decoder), `anyOf`/`oneOf`/`allOf`. An entity-id parameter is recognized by the `EntityId` pattern. **`x-entity-kind`** (W2): a parameter annotated
with canonical kind names (`["TopoVolume"]`, on the property node, the `Option` wrapper or the array) rejects an id of any other kind with `wrong-kind` at the argument (`expected TopoVolume, found Face#1`), before the
op runs. Unknown kind names in a schema are ignored; unannotated parameters are unchanged. Issues map to spans:
the offending argument's value, descending into list and object literals to the exact element. Missing required arguments are reported at the call.
Unknown arguments suggest the closest property name. The checker never invents rules: anything it cannot interpret passes, and `Registry::decode` stays the authority.

## 7. Help and completion

* `help` lists every op and query grouped by namespace with a one-line summary; `help model.create_brick` prints a signature
  (`model.create_brick(x: number, y: number, z: number, name?: string)`), the description, one line per parameter (type, required/optional,
  default, enum values, description), and the version. All of it comes from the schema; ops written with doc comments are self-documenting.
* `Interpreter::complete(source, cursor) -> Completion` returns the replaced byte range and candidates for the token at the cursor from a tolerant lexical scan (it works on incomplete input):
  statement start → op/query names, aliases, keywords; inside `name(` → unused parameter names (required first) ; after `param:` → enum values,
  `true/false`, kind names for entity parameters (`Volume#` and live ids), variables; after `var.` → known output roles; after a kind and `#` → live serials;
  after `in` → kind keywords. A parameter annotated with `x-entity-kind` is offered only the matching kinds (`Volume#`, `volumes`) and not entity variables that hold other kinds; `timeline.*` and `session.*` names and parameters (including `format:` values) complete too. Each candidate has a label, insert text, kind, and detail (the schema description).

## 8. Determinism

Parse and lowering are pure and single-threaded except for (a) set resolution and (b) parsing several files, both with fixed-order results.
Output ordering never depends on hash iteration: maps are `BTreeMap`, sets are sorted vectors. The same (text, registry, state, relations) always
yields the same records; tests compare hashes across runs and thread counts.

## 9. Limits (denial-of-service guards, configurable in `Options`)

Source size 16 MiB, nesting depth 64, expression height 256 (nesting and chains that alternate operators; `expression-too-long`; a run of one operator such as `1 + 1 + ...` or `a | b | ...` is flat and unlimited, §3),
loop-body executions 1,000,000 per run (only **innermost** bodies are charged: an iteration during which no nested iteration started costs 1, an iteration that contained inner
iterations costs nothing itself, so a 1000 x 1000 nested loop costs 1,000,000 and works, an empty inner loop still charges the outer body, and total work stays within a factor of the
nesting depth of the budget), `range` length 1,000,000, string bytes / list length of any built value (`max_value_len`) 1,000,000 (checked in `+`, `str`, `range` and query
results, so `set l = l + l` in a loop stops at `value-too-long` instead of exhausting memory), include depth 16, errors reported per source 100. All are fields of `ParseLimits` / `Options`.
An `else if` chain is flat and costs no nesting depth. `Options::allow_top_level_rebind` lets an interactive session re-send `let w = 10` (a `shadowed` warning).

## 10. Examples

```
// parametric brick, then size and mesh it
let w = 10
let b = model.create_brick(x: w, y: w, z: 2 * w, name: "slab")
intent.set_size(ids: b.body, size: w / 10)

batch {                                 // one atomic node
    let p = model.create_brick(x: 1, y: 1, z: 1)
    intent.set_size(ids: p.body, size: 0.25)       // ids: ["$0.body[*]"] when `ids` is an array parameter
    intent.set_size(ids: [p.body[0]], size: 0.5)
    model.pick(p.body[0])                           // a scalar id parameter needs an index inside a batch
}

for i in range(0, 3) { model.create_brick(x: i + 1, y: 1, z: 1) }
for i in range(1, 4) { mesh.run(targets: Volume#[i], scheme: "map") }     // computed ids

mesh.run(targets: all volumes - Volume#2)
mesh.run(targets: (all volumes - Volume#2) | Volume#9)                   // parentheses required when mixing - with |
timeline.branch("coarse")                                                 // fork the history here
model.delete_body(...{ id: "Body#1" })           // raw JSON spread
```
