# sm-scheme-mapvol

## Positioning

Similar to Cubit's `map` volume scheme (and to the Gordon-Hall transfinite maps used by every
block-structured mesher): a six-sided volume becomes one structured hexahedral block whose
interior is interpolated from its boundary. It differs in how it reaches the block. Cubit
discovers the block from the geometry; here the scheme sits inside the constraint-first driver
of `sm-scheme-api`. It *imposes* `QUAD | STRUCTURED` on its six faces (the driver then picks a
conforming face scheme, or reports why none exists), contributes the
three parallel-edge classes to the global interval system (so the integer interval solver sees
them with the face rows), and generates as a pure function of the solved plan: it reads the six
already generated face grids and returns one `Patch`. It orients the grids by node keys alone,
so the mesh does not depend on face order, loop start, face orientation or grid numbering. The
result is a hex mesh that is bit-identical for 1 and N threads, with canonical interior node
keys `(volume, 0..)` that the committer sorts without help from the scheme.

## Scheme

`"map"`, dimension `Volume`, caps `HEX | STRUCTURED | BLOCK_STRUCTURED`.

* **applicability** (score 100): the piece has exactly 6 faces, each with a single loop, every
  edge is used by exactly two of them, and `V - E + F = 2` over the faces. No embedded members.
  Whether the corners really form a hex block is only known after the corner picker ran; that
  is checked in the interval phase and in `generate` (below).
* **requirements**: `require_caps(face, QUAD | STRUCTURED, Hard)` for each face. No claims: map
  never writes a face mesh (the faces keep their own scheme, e.g. surface map).
* **interval_constraints**: from the solved `FaceLayout`s. Each logical side of each face is a
  set of coedge edges; the twelve distinct sets, each used by two faces, are the block edges.
  Union-find over "opposite sides of a face" gives three classes of four. One
  `IntervalSink::equal(first, other)` row per further member (sums over chains, so a logical side
  made of several edges is handled). Anything else (not 12 shared sides, not 3x4 classes) is
  `NotApplicable`. The planner propagates an `Err` from this phase (`planner.rs`: `?` on
  `interval_constraints`), so this is a backstop that aborts the run with a clear reason, not a
  fallback; the fallback-capable checks are `applicability` and `generate`.
* **generate**: `GenCtx::grid` of the six faces, then `block::assemble` and `block::patch`.

## Orientation (`block.rs`)

Grids are treated as unordered squares: their four corners are vertex nodes, and only keys and
positions are used (a copied face's grid follows its root's layout, not its own side numbering).

1. Corner vertices of each grid; the adjacency graph of the six corner cycles must be a cube
   (8 vertices of degree 3, 12 edges each used twice, the far corner consistent).
2. Frame: origin `V0` is the lowest vertex id; its three neighbours in id order are the i, j, k
   axes; if `det(a - V0, b - V0, c - V0) < 0` two axes swap. Right-handed `(i, j, k)` gives the
   `CellShape::Hex` template order (bottom ring counter-clockwise from +z, then the top ring) a
   positive Jacobian.
3. Every lattice face is matched to its grid by corner set; the grid's dihedral map (rotation or
   reflection) is read off the positions of three corners. Interval counts must agree between
   faces, shared boundary nodes must carry the same key from both faces, and no key may occupy
   two lattice slots; otherwise `Constraint`/`Unsupported` (the driver can fall back).

Interior node keys are `(volume, index)` with index `0..` in k, j, i order, so they are a pure
function of the lattice, and the frame depends only on vertex ids and positions.

## Interpolation

Boolean-sum (Gordon-Hall) transfinite interpolation with tensor parameters,
`X = U1 + U2 + U3 - U12 - U13 - U23 + U123`: face blends, edge blends and trilinear corners,
evaluated at lattice nodes only (so only the face/edge/corner *nodes* are read; no
interpolation of the boundary data itself). The parameters `xi_i, eta_j, zeta_k` are
arc-length fractions along the chord polylines of the four parallel lattice edges of each
direction, averaged, with exact 0 and 1 at the ends. Consequences, all tested:

* exact for affine maps with consistent spacing on parallel edges (a parallelepiped, with
  uniform or graded edges), up to rounding;
* the boundary is reproduced exactly (the interior is the only thing computed);
* uniform bricks give the integer lattice to rounding.

Not done (follow-ups): curved faces blend through arc-length chords of the discrete boundary
only, there is no re-projection onto the face surfaces (not needed: face nodes are given), and
no smoothing (map is a pure TFI; smoothing is a `MeshTransform`).

## Validation and failure

After interpolation every hexahedron must have a positive Jacobian determinant at all eight
corners (cheap, deterministic); otherwise `FailureKind::Quality`, which lets the driver fall
back. Missing nodes (a face that has no grid) is `Unsupported` (a face meshed by a
non-structured scheme), not `Internal`.

## Performance and determinism

Layout is flat: one `Vec<NodeKey>` and one `Vec<[f64; 3]>` over the `(ni+1)(nj+1)(nk+1)` lattice,
a dense `u32` local-index table, and elements pushed straight into the CSR `Elements` with
reserved capacity: no per-element allocation, no `BTreeMap` in the hex loop (boundary
references are listed once per lattice slot, not resolved per corner). Above 16k interior nodes
or cells, the interpolation (per k-slab, collected in slab order) and the Jacobian check
(first bad slab by `find_map_first`) run on rayon; both are order-independent, so output is
bit-identical for one and many threads (tested with a one-thread pool, `Sequential` and
`ScopedThreads`, and `sm_testkit::determinism::check`).

## Tests

`src/block_tests.rs`: synthetic grids (every dihedral map, face orders, vertex numberings, graded
affine blocks, mirror maps, folded interiors, bad inputs). `tests/volmap.rs`: through
`Driver::mesh` with the stub `equal` and surface `map` from `sm-scheme-api[stubs]`.
