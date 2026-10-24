# sm-topo: design

The production cellular non-manifold B-rep store. `TopoStore` implements the frozen
`sm-topo-api` traits (`TopologyView`, `TopologyEdit`), replacing the reference `CellComplex` where
size matters. Depends on `sm-core`, `sm-geom-api`, `sm-topo-api`, `smallvec`, `rayon`.

## Positioning

`TopoStore` is similar to the reference `CellComplex` in `sm-topo-api` (and, in spirit, to
CGAL's `Linear_cell_complex` and to Euler-operator B-rep kernels such as Parasolid, ACIS or
OpenCASCADE's `TopoDS`): one oriented incidence relation per rank, stable ids, atomic
operators, lineage. It differs in the storage and in the bookkeeping around each operator.
Every row lives in a paged CSR arena and is edited **in place by position** (a binary search
and a `memmove`), every edit is journaled as a **delta**, and the end-of-operation checks are
**incremental** (they look at the entries and edges the operation changed, not at whole
volumes). That gives operators whose cost depends on what they change rather than on the size
of the cells they touch: refining one volume to 10^5 faces costs about 45 µs per split instead
of a whole-volume rewrite per split, with **bit-identical observable behaviour** (ids,
lineage, validation results, `canonical_hash`) to the reference, which is what lets the op
layer swap it in without changing a stored project.

## Contract

Nothing in the API is extended or weakened. The observable behaviour is the reference's: the
same operators with the same atomicity (`Err` leaves the store unchanged, `abort_op` restores
the exact state including the hash), the same id minting (serial = slot + 1, never reused),
the same lineage records in the same order (the lineage log is hashed), the same
`end_op` normalization (orientation rule, ring compaction), the same `OpInvalid` payloads,
and the same canonical hash. `canonical_visit` / `canonical_hash` are overridden with the same
traversal specialised to the store's columns (`hash.rs`: slot-indexed column reads, shell rings
of long volume rows by binary search, the FNV fold inlined instead of one dynamic call per
word); the comparison helper of every differential test checks that the **word stream** (not
only the hash) equals the reference's, including polyline and NURBS pcurves. The operator
algorithms follow the reference step by step; what changed is the machinery under them.

## Storage

Per rank (`Dim`), structure of arrays, dense `slot = serial - 1`:

| Column | Type | Note |
|---|---|---|
| `state`, `born`, `flags` | `Vec<EntityState / OpId / CellFlags>` | |
| `carrier` | `Vec<Carrier>` | |
| `tol` | `Vec<f64>` | per-entity override, NaN = none (no map keyed on ids) |
| `down` | `Rows<Incidence>` | boundary rows, one CSR arena |
| `up` | `Rows<Incidence>` | coboundary rows (the transpose), a second arena |
| `aux` | `Rows<EntityId>` | face -> distinct edges, volume -> distinct faces (ascending) |
| `live` | `OnceLock<Vec<EntityId>>` | the ascending live list `cells(dim)` returns, see below |

Sparse data is dense as well: pcurves per face slot (sorted by `(edge, side)`), shell kinds per
volume slot (indexed by ring), coincidence records per face slot (plus the global sorted list),
radial rings per edge slot.

`Rows<T>` (`rows.rs`) is a paged CSR: one `Vec<T>`, a `(start, len, cap)` span per row, blocks
of power-of-two capacity. `splice(slot, pos, remove, insert)` is a `copy_within` when the block
has room and a relocation (amortized O(1)) when it does not; blocks of retired cells go to
per-class LIFO free lists. The layout depends only on the edit sequence, so it is
deterministic, and long split-heavy sessions do not leak (the reference appends a full copy of
a row to its arena on every write and compacts later).

## Edits and the journal

All mutation goes through a few primitives. Row edits are `splice_row(id, pos, remove, insert)`:
it saves the removed entries in the journal data buffer, applies the splice, relinks only the
removed and inserted entries into the coboundary rows, maintains the aux row, and appends a
`Splice` record (`pos`, where the removed entries were saved, how many were inserted). Undo
replays the delta backwards through the same path, so rollback needs no special cases and
leaves coboundary and aux rows consistent. Other mutations (state, carrier, flags, tolerance,
pcurve, shell kind, coincidence pair, model tolerance, geometry replacement) journal their old
value; large old values are boxed so a journal record is 48 bytes. `atomic` wraps each
operator: on error it rolls back to the mark taken before it, so a failing operator changes
nothing; `abort_op` rolls back to the mark taken at `begin_op`.

`set_row` (whole row, used where the reference rewrote a row) diffs against the old row and
splices only the differing middle. `replace_in_parents(old, f)` rewrites each parent row of a
retired or replaced cell. Short rows (edges, faces, small volumes) are rebuilt and normalized
as in the reference. **Long volume and body rows (>= 24 entries)** are edited by position:
the entries naming `old` (and any cell they are replaced by) are found per shell by binary
search on the `(ring, cell, side)` order, removed, normalized group-locally (sort, dedup,
`Left` + `Right` -> `Both`) and re-inserted at their sorted places. Group-local
normalization is exactly what the global normalization does to those groups, and
`paranoid` mode asserts the equality on every rewrite.

## End of operation

`end_op` does what the reference does, with incremental bookkeeping:

1. **Orientation rule.** Faces whose volume uses changed (tracked from the relinked entries,
   a subset of what the reference re-examines, but only faces whose uses changed can flip) and
   touched faces are normalized in ascending id order; each flip is logged as before.
2. **Shell compaction.** Touched volumes drop the kinds of emptied non-outer rings and
   renumber the rest. Ring groups are found by binary search.
3. **Local checks.** The reference's `check_cells` (private to `sm-topo-api`) is ported, with
   two incremental parts so the cost does not depend on volume size:
   - *Shell closure.* Instead of walking every face of every touched shell, the edges whose
     net face-use count may have changed (edges of faces added to or removed from a shell, edges
     added to or removed from a loop, edges whose `DEGENERATE` flag toggled) are recorded as
     the splices happen. At `end_op` the net use per `(volume, ring)` is computed edge-centric
     from the coboundary rows of those edges and their faces. Every committed state has closed
     shells, so a violation can only involve a recorded edge; the lowest violating edge per
     shell is what the reference reports too.
   - *Long rows.* For volume/body rows of >= 128 entries only the entries inserted in the
     operation (logged at splice time, filtered to those still present, in row order) are
     checked for bad or dead references and baffles in closed shells. Unchanged entries were
     valid when they were inserted, and every operator rewrites the parents of a cell it retires.
   `paranoid` mode (tests) also runs the full reference algorithms and asserts identical
   issue lists.
4. **Radial rings** of edges whose incidences or geometry changed are rebuilt. They are
   independent per edge, so with >= 256 dirty edges the rebuild runs on rayon and the results
   are written back by slot: the output is identical for any thread count.
5. Summary (`OpSummary::new`) and commit; the journal is cleared.

## Live lists

`cells(dim)` must return an ascending `&[EntityId]`. The list is a lazily built cache
(`OnceLock`, so `&self` readers are thread-safe): extended in place by `alloc`, edited in place
for the first 32 retirements of an operation, dropped (and rebuilt on the next read in
O(slots)) after more or on undo. Retiring a cell is therefore O(1) when nobody reads
the list in between, instead of an O(live) `Vec::remove`.

## Parallelism and determinism

- `end_op` radial rebuild (above) and `TopoStore::validate_par` (the same result as
  `sm_topo_api::validate`, computed over fixed chunks of cells and concatenated in order, then
  sorted by `(cell, rule)`) use rayon. All reductions are ordered collects.
- Operators are sequential: each touches a local neighbourhood. The canonical hash is the
  contract's sequential FNV-1a fold (byte-serial and order-dependent, so not parallelizable
  without changing the contract); at 10^5 faces it costs about 65 ms, dominated by the fold.
- No `HashMap`/`HashSet`, no time or thread dependent state. The determinism tests build the
  same history twice, with 1 and 4 rayon workers, and concurrently from several threads, and
  compare the canonical hash, every radial ring and both validators.

## Testing

- **Reference scenarios on the new store.** Every test file of `sm-topo-api/tests`
  (`reference`, `shells`, `bridge_chain`, `gaps3`, `gaps4`, `overlay`, `overlay_volume`,
  `freeze` with its hash goldens) is copied to `tests/ref_*.rs` with the name `CellComplex`
  rebound to `TopoStore`, so the original analytic and golden assertions run unchanged.
- **Differential tests** (`tests/differential.rs`, `tests/random.rs`, `tests/scale.rs`,
  `tests/props.rs`): the same history on both implementations; every `Result` (errors
  included), every id, the full boundary/coboundary/carrier/flags/override/radial/aux/pair rows
  of every cell ever created, lineage records and indexes, `validate`, `validate_geometry`,
  moments, and the canonical hash must be equal. Covered: the sm-testkit topology fixtures
  (rebuilt and compared with the testkit's own complexes, plus its checkers), the fixture
  catalogue, merge/unmerge, pole splits and `ChordAt`, the decomposition overlay over
  `Arc<TopoStore>` and `Arc<CellComplex>`, and a seeded random driver (~44 operator kinds,
  valid and invalid, 1-3 operators per op, random aborts, failing `end_op`s) with the
  incremental paths forced on small models by lowering the row thresholds.
- **Property tests** (`proptest`, fixed seed): random bricks against the closed forms, random
  refinements and webcuts against the reference.
- **Determinism** (`tests/determinism.rs`).

## Complexity (n = faces in the volume, k = entries of the rows touched)

| | reference | `TopoStore` |
|---|---|---|
| replace one face by two in its volume | O(n log n): rebuild, sort, relink, re-sort aux | O(log n) search + O(n) `memmove` |
| `end_op` shell/row checks of that volume | O(n) edge walks per touched volume | O(changed) |
| retire a cell | O(live) `Vec::remove` | O(1) (cache edit or lazy rebuild) |
| journal entry | whole old row span, rows grow by append | delta of the changed entries |
| `splice` growth | append a full copy | in place, amortized O(1) |

Measured with `benches/scaling.rs` (see `W1_REPORT.md` for the numbers): a brick's top face
refined by repeated splits; one volume ends with n + 6 faces.

## Known limits

- Row and arena indices are `u32` (about 4 billion entries per rank table); beyond that edits
  fail with `IdExhausted`.
- Long volume rows still pay an O(n) `memmove` per edit (about 16 bytes per face use; the aux
  row 8 bytes), which shows as mild super-linearity above 10^5 faces.
- `cells()` rebuilds in O(slots) when read after more than 32 retirements in one operation.
- `validate_par` walks whole shells (parallel, not incremental).
- `LineageLog` is the contract's type (BTreeMap indexes); its cost per record is not under
  this crate's control.
