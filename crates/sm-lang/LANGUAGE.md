# The smart-mesh command language (`smesh`), v1

Status: design for W1, implemented by this crate. Binding facts come from the frozen contracts in
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
* **Separators**: newline or `;` (empty statements allowed).
* **Comments**: `// to end of line`, and `/* block */` (nests; unterminated is an error). `#` is *not* a comment:
  it is the entity-id sigil (`Volume#3`), matching the `Kind#serial` text form of `sm-core`.
* **Identifiers**: `[A-Za-z_][A-Za-z0-9_]*`. Dotted names (`model.create_brick`) are parsed as identifier paths.
* **Numbers**: `123`, `1_000`, `1.5`, `2e-3`, `1.5E+2`. Maximal munch: a `.` belongs to a number only when a digit follows it, so `1.` and `.5` are
  not floats (write `1.0`, `0.5`) and `1..5` lexes as `Int(1)` `..` `Int(5)`. Integers are `i64` (overflow is an error); anything with `.` or an
  exponent is `f64`. No `NaN`/`inf` literals.
* **Entity references** are a single token: an identifier *immediately* followed by `#` and then a digit or `[` (`Volume#3`, `Volume#[1,2]`)
  lexes as `EntityRef(kind-name)` and the parser then reads the id spec (`3`, `1..5`, `[1, 3, 5..7]`). No spaces are allowed around `#`; one span
  covers `Volume#`, so entity diagnostics point at it. A `#` anywhere else is an error.
* **Strings**: `"..."`, escapes `\n \t \r \" \\ \$ \u{HEX}`; may not contain a raw newline.
* **Punctuation / operators**: `( ) [ ] { } , ; : = . .. ... #`, `+ - * / % **`, `== != < <= > >=`, `| &`.
* **Reserved words**: `let set alias batch if else for in include print assert true false and or not all except`. `help`, `by` and the plural
  entity keywords (`faces`, `volumes`, ... §5) are *contextual*: they are keywords only directly after `all` or directly before `in`, so
  they remain usable as variable names elsewhere.
* There is no `null`: absent optional parameters are omitted (the op-api params model has no null).

## 3. Values and expressions (APREPRO replacement)

Values: `Bool`, `Int(i64)`, `Num(f64)`, `Str`, `List`, `Obj` (string keys), `Ents` (an ordered set of entity ids, §5),
and `Outcome` (the result of an op, §4). Expressions are evaluated eagerly, left to right, with no side
effects, so a script's lowered form is a pure function of (script text, registry, state).

Precedence, loosest to tightest: `or`; `and`; `not`; comparison (non-associative); `|` (set union);
`&` (set intersection); `+ - except`; `* / %`; unary `-`; `**` (right-associative); postfix `.field`,
`[index]`. `-` and `except` are difference on `Ents`; `+` also concatenates strings and lists.

* Arithmetic: `Int op Int` is `Int` (checked, overflow is an error) except `/`, which is always `Num`;
  mixed promotes to `Num`. Division or modulo by zero is an error (never `inf`/`NaN`).
* Builtins (pure, deterministic; names are not reserved but cannot be called as ops):
  `abs min max floor ceil round sqrt sin cos tan asin acos atan atan2 exp ln log10 pow int float
  len count range str ents`, constants `pi`, `tau`. `floor`/`ceil`/`round`/`int` return `Int` (checked), `ents("Volume#3")` /
  `ents([..])` turns id strings (for example from a query result) into an entity set, and every float result is checked finite (`non-finite`). There is deliberately **no random** function: any future
  randomness must be seeded from op parameters. `range(a, b[, step])` is half-open and capped (§9).
  Transcendental functions use the platform libm; the value written into an op record is what is replayed, so
  cross-platform last-digit differences affect only *re-evaluating the script*, never replay of a timeline.
* Lists `[1, 2, 3]`, objects `{ x: 1, "y z": 2 }`, indexing `l[0]`, `l[-1]` (negative from the end),
  `len(l)`; `count(set)` is the number of entities of an `Ents`.
* Variables: `let x = expr` binds once per scope (rebinding in the same scope is an error); `set x = expr` rebinds an
  existing binding of any enclosing scope. Names resolve lexically; there is no implicit global mutation.
* Control flow: `if cond { } else { }`, `for i in list_or_set { }`. Loops are bounded (§9). No user functions in v1.
* `print e1, e2` emits text; `assert cond[, "message"]` fails the script with a span if `cond` is false.

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
registry name, and its target must exist when the alias is defined).

### 4.2 Arguments to params

1. Arguments are collected into one JSON object **in source order**; a later source overrides an earlier one, so
   `f(...base, x: 5)` overrides `x` from `base`, while `f(x: 1, ...{x: 2})` gives `x = 2`. A *named* argument written twice is
   `duplicate-argument`.
2. **One positional argument** is allowed when the op has exactly one *required* property: it binds to that property.
   Any other positional use is `positional-ambiguous`.
3. Each value is converted to JSON under the guidance of the parameter's schema (§6): an `Ents` becomes an id string or an
   array of id strings; a string literal naming an entity (`"Volume#3"`) is normalized through the alias table;
   an `Int` is accepted for `number` and `integer`; a `Num` only for `number`.
4. The object is checked against the op's schema (§6); only then is `OpRecord::new(name, registry.op_version(name), params)` built
   (which canonicalizes: no nulls, `-0.0` folded). Every record, top-level or a `core.batch` member, carries its own current `version`.
   Top-level records are also decoded with `Registry::decode` as a last gate, so the statement is known to apply before it is sent
   (batch members cannot be decoded early because their params may hold `$` references).
5. A **query** is the same, but its record is never stored; its JSON result is the value of the call.
6. Unknown argument names are an error (`unknown-argument`, with a did-you-mean), even where the op's decoder would silently ignore
   them: a typo must not turn into a default.

An op or query call is a statement, or the whole right-hand side of `let`/`set`. Inside a larger expression it is `op-in-expression`:
expressions are pure, calls to the registry are not.

### 4.3 Execution model, ids and results

Top-level op statements execute one at a time through a `Host` (an engine session in production; a plain
`Engine`+`State` in tests; a recording `PlanHost` for validation without execution). **One statement is
one timeline node.** Entity lists (§5) are resolved against the host's *current* state immediately before
the statement runs, so `all volumes` after a `create_brick` sees the new volume.

`let b = model.create_brick(x: 1, y: 1, z: 1)` binds an `Outcome`: `b.created`, `b.removed`, and one field per output
role the op allocated (`b.body`), each an `Ents`. `b.body[0]` is the first id, `b.body[*]` the whole set.

**Plan mode** (`PlanHost`: validate and lower without executing) cannot know an op's outputs: the number of ids an op allocates is not
predictable from the state. There, a top-level `let b = op(...)` binds a *symbolic* outcome (the op is schema-checked, and its record is
collected). Using `b.role` as an argument produces a `plan-unresolved` warning and a placeholder `"$plan.<n>.<role>"` string that is
**not** valid in a record; plan output is for validation, completion and review, not for application. Wrap dependent
statements in `batch { }` to get a fully lowered, applicable form without executing anything.

### 4.4 `batch { ... }` is `core.batch`

A `batch` block lowers to **one** `core.batch` record (one atomic timeline node, one undo step). Inside it:

* ops are collected, not executed; the batch index `i` counts **only emitted ops** (an unrolled `for` contributes one index per emitted op;
  `let`, `if`, `print`, `assert` contribute none), and each member record carries its own `version`; entity lists resolve against the state *at block entry* (documented snapshot
  semantics; explicit ids of entities created inside the block cannot exist yet);
* `let b = op(...)` binds a **symbolic** outcome of batch index `i`. `b.role` becomes the reference string
  `"$i.role"`; `b.role[k]` becomes `"$i.role[k]"`; `b.role[*]` becomes `"$i.role[*]"`. The contract only accepts `[*]` as an **array element**,
  so the lowering is chosen by the schema of the parameter the value lands in: scalar id parameter, `b.role` → `"$i.role"` (the bare form, alias of `[0]`);
  array-of-id parameter, `b.role` or `b.role[*]` → `["$i.role[*]"]`; `[b.role[0]]` → `["$i.role[0]"]`; inside an array literal a bare `b.role` is
  spliced as `"$i.role[*]"`. `b.role[*]` for a scalar parameter is an error; Symbolic values may only be used as op arguments (or list elements of them);
  arithmetic or set algebra on them is `not-concrete`;
* a user string literal starting with `$` is emitted as `$$...` (the batch escape), so literals can never be mistaken
  for references; reference strings are validated against `sm_op_api::batch::REF_PATTERN`;
* queries, `include`, and nested `batch` are errors inside a batch (queries are not recorded; nested batches are rejected by the contract);
  `let`, `set`, `if`, `for`, `print`, `assert` evaluate normally (so a `for` loop unrolls into the batch);
* a batch does not open a scope. When it has been applied, symbolic outcomes are re-bound to concrete `Ents` built from
  the batch `Delta.outputs` (`"<i>.<role>"`); `b.created`/`b.removed` are unavailable for them (`not-concrete`).
  Under `PlanHost` they stay symbolic. Known gap: only variables that hold a reference directly (`let f = p.faces`) are rebound after the batch;
  a reference stored inside a list or object (`let l = [p.body]`) is not, and using it afterwards is `stale-reference`.

### 4.5 Scripts and includes

A script file is a statement list. `include "path"` asks the host's loader for the text of `path`
(`Host::load`; the language does no I/O itself), parses it as a separate source (so spans point into the right file), and runs it in the
**current scope**. Cycles and depth beyond 16 are errors. Parsing is all-or-nothing per source file: if any source fails to parse,
**no statement of the run executes**; every syntax error found by recovery is reported.

## 5. Entity references and entity lists

All entity expressions evaluate to `Ents`: a duplicate-free set of `EntityId` ordered by `EntityId` (kind, then serial), so output never
depends on how a set was built.

### 5.1 Typed ids and the alias table (frozen by a test)

`Kind#serial` with **no spaces** around `#`. `Kind` is either the canonical `sm-core` kind name (`TopoVolume`, `MeshCell`, `Color`, ...)
or one of these aliases:

| alias | kind | plural keyword | | alias | kind | plural keyword |
|---|---|---|---|---|---|---|
| `Vertex` | `TopoVertex` | `vertices` | | `Node` | `MeshVertex` | `nodes` |
| `Edge` | `TopoEdge` | `edges` | | `Element` | `MeshCell` | `elements` |
| `Face` | `TopoFace` | `faces` | | | | |
| `Volume` | `TopoVolume` | `volumes` | | | | |
| `Body` | `TopoBody` | `bodies` | | | | |

Every other kind name is also its own alias, and has a snake_case plural keyword (`Curve`/`curves`, `Color`/`colors`, `Group`/`groups`,
`Boundary`/`boundaries`, `PartitionCell`/`partition_cells`, `Mesh`/`meshes`, `MeshFace`/`mesh_faces`, `Symmetry`/`symmetries`, ...; the full table
is `kinds::KIND_TABLE`, tested to cover `EntityKind::ALL`). Unknown kinds are `unknown-entity-kind` with a did-you-mean.

### 5.2 Forms

```
Volume#3                  one id
Volume#1..5               inclusive range (live members only; a warning if none)
Volume#1..9 by 2          stepped range (1, 3, 5, 7, 9)
Volume#[1, 3, 5..7]       list of ids and ranges
all faces                 every live entity of the kind
faces in Volume#2         entities of that kind related to the right operand (see 5.3)
a | b    a & b    a - b   union, intersection, difference ('-' may be written 'except')
(…)  variable names       parentheses and variables holding an Ents
```

`faces in X` has the *tightest* binding on its right side (`faces in Volume#2 - faces in Volume#3` is
`(faces in Volume#2) - (faces in Volume#3)`); use parentheses for compound operands: `faces in (Volume#2 | Volume#3)`.

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
bound to an array of ids it becomes the array in id order. It is a type error anywhere else.

## 6. Schema-driven checking

Each op's params schema is `Registry::schema(name)` (JSON Schema 2020-12, `$ref`/`$defs` resolved locally; `core.batch` uses `Registry::batch_schema()`).
The checker supports `type` (incl. type lists), `properties`, `required`, `additionalProperties: false`, `enum`, `const`, `items`/`prefixItems`,
`minItems`/`maxItems`, `minimum`/`maximum`/`exclusiveMin|Max`, `minLength`/`maxLength`, `pattern` (only the contract's id and batch-ref patterns are interpreted,
other patterns are left to the op's decoder), `anyOf`/`oneOf`/`allOf`. An entity-id parameter is recognized by the `EntityId` pattern. Issues map to spans:
the offending argument's value, descending into list and object literals to the exact element. Missing required arguments are reported at the call.
Unknown arguments suggest the closest property name. The checker never invents rules: anything it cannot interpret passes, and `Registry::decode` stays the authority.

## 7. Help and completion

* `help` lists every op and query grouped by namespace with a one-line summary; `help model.create_brick` prints a signature
  (`model.create_brick(x: number, y: number, z: number, name?: string)`), the description, one line per parameter (type, required/optional,
  default, enum values, description), and the version. All of it comes from the schema; ops written with doc comments are self-documenting.
* `Interpreter::complete(source, cursor) -> Completion` returns the replaced byte range and candidates for the token at the cursor from a tolerant lexical scan (it works on incomplete input):
  statement start → op/query names, aliases, keywords; inside `name(` → unused parameter names (required first) ; after `param:` → enum values,
  `true/false`, kind names for entity parameters (`Volume#` and live ids), variables; after `var.` → known output roles; after a kind and `#` → live serials;
  after `in` → kind keywords. Each candidate has a label, insert text, kind, and detail (the schema description).

## 8. Determinism

Parse and lowering are pure and single-threaded except for (a) set resolution and (b) parsing several files, both with fixed-order results.
Output ordering never depends on hash iteration: maps are `BTreeMap`, sets are sorted vectors. The same (text, registry, state, relations) always
yields the same records; tests compare hashes across runs and thread counts.

## 9. Limits (denial-of-service guards, configurable in `Options`)

Source size 16 MiB, nesting depth 64, expression height 256 (nesting plus operator chains such as `1 + 1 + ...`; `expression-too-long`),
loop iterations 1,000,000 per run, `range` length 1,000,000, include depth 16, errors reported per source 100. All are fields of `ParseLimits` / `Options`.

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
}

for i in range(0, 3) { model.create_brick(x: i + 1, y: 1, z: 1) }

mesh.run(targets: all volumes - Volume#2)
model.delete_body(...{ id: "Body#1" })           // raw JSON spread
```
