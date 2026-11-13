# sm-modify

Mesh modification ops (W4 tasks MOD-HTET, MOD-REFINE-U, MOD-REFINE-L; see drafts/meshops.md): `mesh.htet` and `mesh.qtri` (hex to tet with polyhedral closure, quad to tri), uniform `mesh.refine` with `LevelMap`, and local refinement with polyhedral hanging faces. Region and tag edits are not here; they belong to REGIONS-* in sm-model/sm-boolean. The `modify` plugin is registered in `sm_server::standard()`.

## Pass shape (D1 core, built by MOD-REFINE-U)

Every op is a pure `MeshRead -> MeshBuild` pass: it reads the source entry, builds a NEW mesh through `MeshBuild` with new ids, inserts it with `insert_mesh` (`derived_from` set, `record_successors`), and keeps classification, sets and tags. The reusable parts live in `refine.rs`: node tables keyed by source edge / quad-face / cell index (never by hash order), a per-shape template (`template.rs`), classification of every new entity from the parent entity it lies on, and set / tag carry-over. Losses are reported as warning diagnostics with a code (`refine.set_dropped`, `refine.tag_dropped`). MOD-HTET and MOD-REFINE-L reuse the node table and the carry-over.

## `mesh.refine` (MOD-REFINE-U)

`mesh {mesh, levels?, keep_source?, project?}`. Each level is its own mesh entry (role `mesh`, coarsest first) linked fine-to-coarse by a `LevelMap` with a face map: `insert_mesh` first, then `MeshLayer::link` with `CatalogEdit` in scope. With `keep_source: false` the source goes through `retire_mesh` (its links drop) and level 1 is not linked. The `refine.report` info diagnostic carries counts, projection counters, total volume and minimum scaled Jacobian of parent and child.

Rules: tet 1 to 8 (four corner tets, the octahedron cut along its shortest diagonal, ties to the first of the three; the cut depends on coordinates only, so it needs no vertex ranks), hex 1 to 8, wedge 1 to 8 (triangle 1 to 4 times two layers), pyramid 1 to 6 pyramids + 4 tets (four corner pyramids, the top pyramid, the inverted centre pyramid and four tets at the base edges), tri 1 to 4, quad 1 to 4. A new node spans a set of parent corners and sits at their mean: edge midpoints, quad-face centres, hex centres. Polyhedra, high order, inactive and non-conformal cells are refused.

Classification: original vertices keep theirs; an edge node takes the class of its edge (the edge table of the source, or one derived with a vertex-rule fallback for edges on a model edge); a face node the face class; a cell node the cell class. A child face lying on a parent face takes that face's class (and `PartitionCut` kind), any other child face the cell's. If the source has an edge table so has the result (child half-edges of a parent edge keep its class).

Projection (`project`, default true): a new node classified on a `TopoEdge` or `TopoFace` is projected onto the carrier with `project(p, None)` (the `sm-smooth` rule: refused outside the curve range or the trimmed face). It is accepted unless an incident child cell would drop to a scaled Jacobian at or below 1e-12 (or worsen an already bad one); then the straight position stays and the counter `projection_rejected` says so. A projection that moves a node by rounding only (planar faces) keeps the straight bits.

Sets: cell sets become the children; face sets the child faces lying on the member faces; side sets the child sides whose cell descends from the member side; vertex sets the original vertices plus every new node all of whose spanned vertices are members. Cell tags are inherited; vertex tags take the mean (real) or the lowest spanned vertex's value (integer). Face and edge tags, shell-side and point sets are dropped with a warning.

## Measured (see `tests/`)

- Hex `n^3` block: `(2n+1)^3` vertices, `8n^3` cells, boundary Euler characteristic 2, scaled Jacobian unchanged (1); sheared parallelepipeds: each child exactly V/8 (1e-14), min and max scaled Jacobian equal the parent's (affine invariance).
- Tets: four corner children exactly V/8, the octahedron V/2; a Kuhn tet block stays in 1 similarity class for 3 levels (Freudenthal refinement is self-similar; the bound in the spec is 3).
- Pyramid: 10 children (6 pyramids, 4 tets), volumes V/8 and V/16; the 48-pyramid cube refines to 480 and validates. Wedge 8 children of V/8. Mixed hex/wedge/pyramid/tet fixture: conformal, boundary area unchanged.
- Axial cylinder `r = 1`, `h = 2`, 16 circle intervals, 240 hexes: 1920 and 15360 cells at levels 1 and 2, every lateral node at r = 1 to 1e-12, volume equal to the inscribed prism `(n/2) sin(2 pi/n) * 2` for `n = 32, 64` to 1e-12 (6.242890304516, 6.273096981092 against 6.283185307180), minimum scaled Jacobian stays 0.707107.
- W3c through pin cell (one cell, 48 hexes, 8 circle intervals): fuel volume 0.452548340 -> 0.489834793 (`pi r^2 h` = 0.502654825), relative error 9.97e-2 -> 2.55e-2 (the polygon deficit falls fourfold); total volume exact to 1e-12; cells 48 -> 384. The minimum scaled Jacobian falls 0.7071 -> 0.3827 with projection (0.7071 without): the worst cell has two consecutive boundary arcs at a corner, whose interior angle grows from 135 to 157.5 degrees when the octagon becomes a 16-gon. That is a property of the boundary-hugging cell, not of the refinement; smoothing cannot repair it.
