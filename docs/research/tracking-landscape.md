# Particle tracking landscape: competitors, our position, open questions

A living document. Update it whenever we learn something about a competitor or measure something of our own. Detailed, cited source reports sit next to it, and this page links to them instead of repeating them.

**Evidence tags:** [V] checked against source code or the paper's full text · [A] abstract or secondary source · [K] from memory, not yet checked · [M] measured by us · [D] design claim, not measured.

**Source reports**
- `dagmc-internals.md`: DAGMC, MOAB, double-down, XDG, OpenMC's DAGMC path. Line references at pinned commits.
- `delta-tracking.md`: Serpent, OpenMC delta-tracking PRs, MCATK, ORANGE/Celeritas, MCNP UM, Geant4, rendering literature.

---

## 1. The two tracking questions

| Mode | Question per step | Structure it needs |
|---|---|---|
| Surface tracking (ray tracing) | Where does the ray leave the current cell, and what is next? | Per-cell boundary (planes or a per-volume tree) plus adjacency |
| Delta (Woodcock) tracking | What material is at this point? (after a jump sampled from a majorant) | A disjoint partition (lattice, kd-tree or octree) with per-node majorants, walked by DDA; plus point location inside a leaf |

A BVH (overlapping boxes) suits point location but cannot produce majorant segments [V, PBRT-v4]. One iterator of segments `{t0, t1, cell_or_node, Σ_M}` serves both modes: surface tracking is the case where a segment is one cell and Σ_M = Σ_t. Switching mode along a flight is unbiased [V, delta-tracking.md §3].

## 2. Competitors

| Code | Geometry | Surface step | Point location | Delta tracking | Notes |
|---|---|---|---|---|---|
| DAGMC + MOAB | Faceted surfaces; volumes = sets of surfaces with senses | `ray_fire(V)` walks V's own OBB tree (V's bounding surfaces only), restarts from V's root every step, children in build order [V] | Stock DAGMC: linear loop of `point_in_volume` (box check + 1 ray) [V] | None [V] | 8.5–14.6 µs/ray [V]; zero lost particles on sealed models over 30 CPU-days [V]; `rand()` when no direction is given (non-deterministic) [V] |
| double-down (Embree) | Same | Embree BVH, mixed precision | Real global scene | None | Rays 5.5–6.5× faster, OpenMC 1.5–2× [V] |
| XDG | DAGMC lineage, plus tet/hex volume meshes | Walks volume elements face to face with Plücker tests [V] | Embree | None | **Active; GPU backend in progress.** Cell walking alone is not novel |
| OpenMC CSG | Analytic quadric surfaces, universes, lattices | Distance to every surface of the current cell | Universe hierarchy, lattice index | Unmerged PRs #3971, #4044 [V] | Still 3–6× faster than DAGMC with Embree [V] |
| OpenMC delta PRs | Same | | Restarts at the root (`exhaustive_find_cell`) after every flight [V] | Hybrid speedups: BEAVRS 1.6×, GCMR 1.8–3×, ARC-100 3.5×; the fusion model got slower [V] | Shows that point location is the deciding cost |
| Serpent | Universe CSG, STL, unstructured polyhedral (multiphysics) | | Universe search; STL uses a search mesh [A] | Global majorant vs energy; `set dt 0.9` switches to surface tracking [V, manual 2015] | Website blocked our fetches (HTTP 403): STL and polymesh details unverified |
| MCATK | Mesh | Surface tracking between mesh cells | | Delta tracking inside each cell, so track-length tallies survive (1.5–1.75×) [V] | |
| MCNP UM | Abaqus unstructured mesh | Element to element via setup-time neighbour lists [V] | | | |
| ORANGE / Celeritas | Surface-based CSG, GPU | Surface→volume connectivity [V] | BVH for the starting volume [V] | | |
| Geant4 | CSG, tessellated, tet | Safety distance skips intersection tests | Smartless voxels (default 2.0) [V] | | Safety concept is relevant to us |

## 3. Our position (claims ledger)

Each claim stays [D] until a benchmark makes it [M]. Remove or rewrite any claim that measurement refutes.

| # | Claim | Status | Against |
|---|---|---|---|
| 1 | Surface step tests only the current convex cell's planes, from the entered face; no tree, no restart from a root | [D] | DAGMC root restart, OpenMC all-surfaces-of-cell |
| 2 | Exact analytic faces (W3b ANALYTIC): one cylinder instead of thousands of facets; no faceting error | [D] | DAGMC, XDG |
| 3 | Exact adjacency from the B-rep and the mesh: no `make_watertight`, no `RayHistory`, no overlap tolerance, no leaks | [D] | DAGMC (robust only when sealed) |
| 4 | Safety distance to a convex cell's planes is exact and free; skip the intersection test when the flight is shorter | [D] | DAGMC's signed-distance-field prototype (Shriwise 2017, not shipped) [V] |
| 5 | Native instancing and lattices: O(1) element index, DDA across lattices | [D] | DAGMC flattens; OpenMC CSG has it |
| 6 | Hinted point location for delta tracking: start from the majorant node (lattice index, r² search in pin rings, short walk in meshed leaves) | [D] | OpenMC PRs restart at the root; DAGMC loops over volumes |
| 7 | Per-cell segments for MOC/Sn and mesh tallies come natively | [D] | DAGMC, OpenMC CSG (not designed for it) |
| 8 | Deterministic and bit-identical across thread counts; flat SoA layouts | [D] | DAGMC uses `rand()` |
| 9 | **Known weakness:** in a large homogeneous region a pure cell walk pays per cell, while DAGMC makes one O(log T) jump. Planned fix: a region-jump mode with an exact per-region boundary structure when per-cell segments are not needed | [D] | DAGMC wins here today |

## 4. Multiphysics: several meshes over one decomposed model

This is the advantage no tracking code above has, because none of them is a mesher.

- **Many meshes, one model.** smart-mesh keeps several meshes over the same decomposed geometry (ir.md: the `meshes` layer is plural). Examples:
  - a CFD mesh (boundary layers, hex-dominant or polyhedral);
  - neutronics meshes (a fine polyhedral mesh for MOC/Sn, a coarse one for CMFD/multigrid, and the tracking structure for Monte Carlo in surface or delta mode);
  - a structural mesh (second-order tets or hexes).
- **Shared classification.** Every mesh is classified to the same B-rep entities and colors. So each node, face and cell knows its geometric volume, face, edge or vertex.
- **Consistent mapping.** Transfers between meshes can be computed per geometric entity:
  - Volume fields (power, temperature, density) are mapped by overlap only within the same B-rep volume, so nothing is smeared across a material interface.
  - Surface fields (heat flux, pressure) are mapped on the same geometric face.
  - Conservation can be enforced region by region.
  - Today's coupled codes usually map by cell centroid or by mesh-to-mesh search without geometric identity [K, to verify: Cardinal/MOOSE transfers, Serpent multiphysics interface].
- **Tracking structures come from a mesh, not from a separate faceting.** The Monte Carlo tree (surface mode, delta-mode majorant partition, or both) is a derived layer built from whichever mesh suits the method. It stays consistent with the CFD and structural meshes, so feedback fields land on cells whose identity is known.
- **History and forks.** A timeline fork can hold a different meshing for one physics without touching the others, and the state hash says which meshes a result came from.
- **Open question:** Cubit can mesh a geometry several times only through copies, which loses the tie to the original entity [K, verify]. Check the inter-mesh mapping offered by Cubit, Gmsh, Salome/MEDCoupling and MOOSE (MultiApps transfers), and position against them.

## 5. Benchmark plan

- Compare per track: (segments per track × cost per segment) for us against (one ray fire per track) for DAGMC, with MOAB and with double-down/Embree, and against XDG `walk_elements`. Same geometry, same rays, same machine.
- Cases: sphere; pin cell; 17×17 assembly (large coolant region, the case for claim 9); ITER-like faceted part; TRISO compact (delta tracking).
- Report µs per ray, segments per ray, memory, build time, lost particles, and 1-thread vs N-thread bit identity.
- Harness: `crates/sm-rt-capi/bench/` (W3a, record only). DAGMC via `condaforge/miniforge3` + `mamba install -c conda-forge dagmc moab` (verified on aarch64).

## 6. Open questions

1. Serpent STL point-in-solid, its adaptive search mesh, and polyhedral-cell location (needs the wiki pages the fetcher could not reach).
2. When `find_volume` was added to MOAB and whether any DAGMC consumer builds the global tree.
3. Shriwise 2018 thesis contents beyond the 2017 SDF papers.
4. XDG's GPU backend: status and numbers.
5. Majorant memory at scale: the material-set count of a real depleted core (estimate 5×10⁴ sets, about 51 MB of majorant tables).
6. How coupled codes map fields between meshes today (§4).

## 7. Change log

- 2026-10-26: created from `dagmc-internals.md` and `delta-tracking.md`; added the multiphysics multi-mesh section (§4).
