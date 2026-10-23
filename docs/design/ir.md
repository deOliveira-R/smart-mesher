# smart-mesh IR: positioning and layer model (draft v1)

Evidence: `docs/research/mesh-ir-synthesis.md` (round 1, 18 fact-checked topics), `mesh-ir-critic.md`, `mesh-ir-addendum-r2.md` (round 2, 6 gap topics; its §7 holds the consolidated schema decisions).

## 1. Is there a format we should just adopt?

No single existing format covers polyhedra + high-order + non-conformal interfaces + materials/sets + CAD association + parallel layout + fast I/O.

- **CGNS** is the closest *interchange* format: a formal, versioned spec, NGON/NFACE polyhedra, Family indirection, and declared interfaces (`GridConnectivity_t`). It lacks per-entity CAD association, stable IDs and ownership, interface data for polyhedral or high-order faces (interpolants are linear-only), and high-order beyond a fixed list of element types. It also has no neutronics users, and PETSc rejects its polyhedra.
- **Exodus II** is mandatory as an *export* (MOOSE/Griffin, Cubit, Sierra). Its polyhedra are an add-on: no face orientation, and the library itself can't resolve sidesets on NFACED blocks.
- **Gmsh MSH 4.2** is the closest single file to our layer model (B-rep entities, classification, polytopes, partitions), but it is still an unreleased work in progress.
- Database precedents: **PETSc DMPlex** (Hasse diagram), **MOAB** (entities + sets + typed tags; DAGMC → OpenMC/MCNP), **STK** (parts + selector algebra + buckets), **Conduit Blueprint** (schema-described tables), **PUMI** (classification as the cross-layer key).
- The baseline for a whole *project* store is **Cubit `.cub5`**, plus **CGM** and **ITAPS iRel** (geometry↔mesh relations as a typed interface). These persist all the layers together but are closed (.cub5) or abandoned (ITAPS).

## 2. Positioning statement

> Our IR is conceptually similar to **Cubit's `.cub5` (itself HDF5) / Salome SMESH / ITAPS iRel** (geometry, regions and mesh persisted together, with typed relations between them) and to **DMPlex / MOAB** (the mesh as an entity–relation database), with a **CGNS-like** formal, versioned schema. It differs in:
> - **(A) Face-first topology.** Every cell, not only polyhedra, has stored, *oriented* cell→face and face→(left, right) relations (on-disk precedents: OpenFOAM owner/neighbour, CGNS ParentElements; we make them universal and add a single declared orientation rule). Polyhedra are the general case; tet and hex are derived shape tags with fast paths.
> - **(B) Explicit layers joined by typed foreign keys.** Classification (mesh→topology), selection (region→topology), pairing (interface→faces), ownership (part→entity). Precedents use name strings (CGNS Family), label conventions (DMPlex/MOAB) or bare integer tags (Gmsh).
> - **(C) Non-conformal interfaces as one first-class record.** Kind, side sets, transform, tolerance, match policy, provenance, imprint status, and conservation nature. It comes with a cached, symmetric overlap table valid for polyhedral and high-order faces. No surveyed format persists sub-face overlap polygons with areas and normals: HOPR persists a coarse-side mortar type, MFEM-NC/t8code persist refinement provenance, Nektar++/OpenFOAM-NCC/CGNS persist a declaration only.
> - **(D) Stable typed IDs**, with owner/ghost/global-id columns in the schema.
> - **(E) High-order as (shape, degree, basis)**, including curved faces on polyhedra. Every surveyed format uses a closed list of element types.
> - **(F) Schema, validator and converters shipped together from day one**, following openPMD (standard + validator + multiple backends). Every exporter emits a **loss report**.
> - **(G) Columnar physical layout:** CSR (compressed sparse row) offsets plus flat indices, SoA (structure-of-arrays) coordinates, partition-contiguous datasets. Backend: an abstract storage model with three physical profiles: a memory-mapped container (solver reads), Zarr v3 (archive/cloud, pure Rust) and HDF5 (HPC/MPI-IO interop). Details in addendum §7.1.
> - **(H) Provenance.** The IR records the op-timeline hash that produced it, so any mesh can be regenerated or forked.
>
> This gives: lossless polyhedral and DG-transport consumption (face adjacency, upwind graphs and interface weights come from the file instead of being re-detected); reproducible, inspectable non-conformity; remeshing that keeps materials, BCs and CAD links through classification; and exports to every surveyed format as declared lossy projections.

Like LLVM IR, the IR is the hub: the front ends (geometry ops, meshers, importers) write it, the back ends (exporters) lower it, and transformations (refine, conformize, partition, convert) are passes from IR to IR.

## 3. Imprint vs merge vs conformize (terms)

| Term | Layer | What changes |
|---|---|---|
| **Imprint** | L1 topology (geometric) | Faces are split so the overlapping regions of two volumes coincide exactly. Still two faces. |
| **Merge** | L1 topology (information) | Coincident entities become **one shared entity** with two sides, `(region_fwd, region_rev)`. Meshing then gives a conformal mesh automatically. |
| **Interface record** | L2/L7 | Two face sets are explicitly paired *without* sharing (non-conformal by design: sliding, mortar, different refinement). Stores kind, tolerance and transform. |
| **Conformize** | L5/L7 (mesh pass) | Physically rewrites mesh faces so a non-conformal interface becomes conformal. In 3D the coarse face is **replaced by the N fine faces** (the polyhedron gains faces). Inserting the hanging vertices into the coarse face's loop breaks Teton/OpenSn face matching. In 2D the two forms coincide. |

Topology stores the imprint/merge status for every coincident face pair (merged / imprinted-not-merged / coincident-unimprinted / overlapping). That makes "two faces touching but not shared" a **detectable, explicit state** instead of an accident.

## 4. How non-conformal meshes work elsewhere (summary)

Six patterns exist:
- **A.** Implicit unmerged faces + run-time geometric search: MOOSE mortar, Exodus/Sierra, ESI OpenFOAM AMI.
- **B.** Explicit declared pairs in the file: CGNS Abutting, OpenFOAM Foundation NCC patches, Gmsh periodic.
- **C.** AMR hanging-node trees: MFEM NCMesh, DMPlex trees, deal.II, libMesh, p4est.
- **D.** Precomputed intersections / cut faces: Fluent child faces, OpenFOAM NCC, STAR-CCM+, MEDCoupling, MOOSE mortar segment mesh in memory.
- **E.** Degenerate to polyhedra (the hanging node becomes a polygon vertex): PDT and OpenSn PWLD.
- **F.** Overlay, with no conformity needed: OpenMC unstructured tallies.

**How codes detect non-conformity:** almost always by declaration plus geometric proximity search (Cubit Find Overlap, Fluent auto-create, MOOSE k-d tree screening). Some infer it from unmatched faces (OpenFOAM checkMesh, libMesh find_neighbors, DUNE `conforming()`).

**Our IR stores all four representations** so any consumer can use what it understands:
1. **Interface records**, the declared level.
2. **Overlap tables**, derived and hash-versioned: sub-face polygons, areas, centroids, normals, both directions.
3. **An optional hanging-node hierarchy.**
4. **Detection** as a validator query that emits candidate records.

Exporters lower these per target:
- DG-SN transport: sub-faces plus upwind graphs.
- PWLD: conformize to polyhedra.
- MOOSE: two sidesets plus lower-dimensional blocks.
- OpenFOAM: NCC or AMI patches.
- CGNS: Abutting connectivity.
- DAGMC: needs a merged, watertight model.

## 5. Layers (v2, after user review 2026-10-23)

Physical volumes, their colors and their boundary kinds can't be separated. Symmetry is undefined until volumes are colored (a barycentre needs to know which material is where) and until each boundary is known to be a deck transformation or a response boundary. So the first layer is one **Model** layer with sub-layers, not three separate layers.

| Layer | Sub-layer | Content |
|---|---|---|
| **Model** (what the problem *is*; no discretization) | shape | exact curves/surfaces (B-rep + CSG quadrics) |
| | topology | non-manifold cell complex; imprint/merge status per coincident face pair |
| | coloring | colors on volumes and on faces/boundaries; abstract material IDs per volume ("material 1", "material 2"; exact partition), blocks/assemblies as named groupings. No physics. |
| | boundary kind | every boundary face is a **deck** boundary (mirror / translation / rotation, with pairing and transform), a **response** boundary (tag only), or an **interface** |
| | symmetry | deck group + fundamental domain, declared or detected; the full model is the **unfold** (derived, reversible) |
| | instancing | prototypes, instances, lattices. These are group actions on a fundamental piece, so they share the symmetry machinery. |
| **Decomposition** | pieces | virtual, reversible cuts/composites of the colored model into meshable **pieces** (sweep blocks, mappable chunks). Never changes coloring. Named to avoid "partition" (parallel parts) and "block" (Exodus material groups). |
| **Intent** | | sizes, intervals, schemes, layers per partition cell; reads boundary kind (deck faces get no boundary layers) |
| **Meshes** (plural) | mesh | face-first tables, classification onto partition/model entities, coordinates, (shape, degree, basis) |
| | hierarchy | several meshes over one model: fine + coarse levels (CMFD / algebraic multigrid) with inter-mesh maps; instanced view |
| | interfaces | non-conformal records + overlap tables; deck pairings **lowered** from the model via classification (declared once, never twice) |
| **Derived** | | tracking structure, geometry caches, sweep graphs, parallel distribution. Rebuildable, keyed by content hash. |
| **History** | | op timeline, provenance (cross-cutting) |

**Out of scope (confirmed):** material properties and response-function values. The mesher colors volumes and faces/boundaries; the consuming code resolves colors into properties.

### 5.1 Deck transformations

Deck transformations are topological information on boundaries, not response functions. They need no knowledge of the physics on other boundaries.

| Kind | Transform | Pairing | Meshing consequence |
|---|---|---|---|
| Mirror | reflection in a plane (improper, det = −1) | self-paired: the plane is fixed pointwise | boundary nodes stay on the plane (smoothing constraint); no boundary layers |
| Translation | rigid translation T | face set A ↔ face set B = T(A) | paired curves need equal intervals; mesh on B = T(mesh on A) |
| Rotation | rotation by 2π/n about an axis | A ↔ B = R(A) | same as translation, with R |

The name follows covering-space theory. The reduced model is the **fundamental domain** F, the deck group G acts on it, and the full model is ⋃ g·F over g ∈ G.

Workflows this enables:
- **Cut and keep a chamber.** Place a mirror plane (or a pair of periodic planes): the model is cut, the chosen chamber is kept, and the new faces are tagged as deck boundaries immediately.
- **Imported pre-cut geometry.** Tag existing faces as deck boundaries; tag the rest as response boundaries.
- **Undo symmetry.** Unfold to the full geometry, e.g. for a transient that breaks symmetry. The full model stays derived from F + G until you choose to edit it as a full model.
- **Mesh once, unfold.** Meshing F and unfolding gives an *exactly* symmetric full mesh for free.
- **Detection.** Candidate planes and axes come from colored invariants (per-color barycentres and inertia tensors), then get verified by mapping topology, geometry, colors and boundary kinds.

Constraints this puts on the interfaces:
1. Improper transforms reverse orientation, so unfolding must flip face loops and re-apply the left/right rule for reflected copies.
2. Paired deck boundaries become interval-matching constraints and copy-under-transform constraints for the scheme API.
3. The tracking structure treats a deck boundary as reflect-or-wrap on hit, so a reduced model can be ray-traced directly.

### 5.2 Tracking structure for Monte Carlo (derived)

DAGMC baseline (checked against DAGMC source): an OBB tree per surface over triangle **facets**, joined into one tree per volume (MOAB). The optional double-down backend uses Embree (single-precision traversal, double-precision Plücker test at the leaves). On crossing a surface, the next volume is read from the surface's two sense tags. An implicit complement covers the space outside the CAD model. Facets only: no exact surfaces.

We differ, because we know every level from geometry to mesh:
- **The hierarchy is the top of the tree.** Lattice/instance → volume → surface, with bounding volumes taken from the known structure (no rediscovery). Instances are traced in the prototype's frame, so one tree serves N copies.
- **Exact hits on analytic surfaces** (quadrics, tori), with facets only for freeform NURBS. This removes faceting error and cuts primitive count.
- **No tree search after a crossing.** Face sides (`left`/`right` in the model, face→cell in the mesh) give the next volume or cell directly ("mesh walking"). The tree is needed only for the first location and for re-entry.
- **Deck boundaries handled in the tracker** (reflect or wrap), so symmetric reduced models trace directly.
- The structure is a derived, content-hashed cache that can be exported. It is not a new geometry format.

Benchmarks against DAGMC and native CSG (OpenMC) are a W3 deliverable; we have no performance numbers yet.

### 5.3 Import/export direction

**Geometry in, meshes out.** We import CAD/geometry (below) and only export meshes. CGNS, Exodus, OpenFOAM, VTKHDF, DAGMC, etc. are export targets. Mesh import is deferred; it may come back later so our machinery (remeshing, tracking structures) can run on foreign meshes.

Geometry import:

- STEP AP203/214/242 (P1; what every CAD system exports), IGES, STL.
- SAT is feasible: `cq-acis` (MIT, Rust core, experimental) decodes SAT versions 105–700 and is a reference implementation.
- Parasolid x_t is to be researched.

## 6. Decisions from round 2 (addendum §7)

- **IDs:** `EntityId` = u64 (8-bit kind + 56-bit serial), never reused. Serials are assigned per operation by a deterministic canonical sort, so replay reproduces them exactly. Parents stay inactive after refinement, and repartitioning never touches IDs.
- **Orientation rule:** the face's vertex loop defines its normal (right-hand rule). `left` = the lower cell ID, and the normal points from left to right. Boundary faces have `right = none` with the normal pointing outward. With this rule, OpenFOAM owner/neighbour and CGNS NFACE signs come out with no flips.
- **Instancing and hierarchy:** `Prototype`, `Instance` and `Lattice` (nestable, hex/cartesian) live in topology/regions. The mesh layer has an instanced view plus a deterministic `flatten()`. A **coarse-mesh slot** (an independent mesh of the same topology plus a fine→coarse map) serves CMFD. A **CSG front end** (quadrics, universes, lattices) feeds the shape layer. Extruded structure (base mesh × layers) is first-class.
- **Polyhedral geometry rules:** a declared, versioned `face_geometry_rule` / `cell_geometry_rule`. The default is a triangle fan about the vertex mean (matches OpenSn's side-tet sum). The validator checks planarity, convexity and closedness.
- **Transport minimal subset:** coordinates; cell→face (signed) and face→vertex CSR; `face_left`/`face_right`; material per cell; boundary sets + BC types (vacuum, reflective, periodic, source, albedo, white); geometry rule id plus optional cached face/cell geometry. Sweep graphs are derived caches, never required.

## 7. Still open

- Mixed-material cells (volume fractions) vs "materials partition volumes exactly".
- Units, 2D/RZ/1D meshes, time series.
- How the Exodus writer gets netCDF-4 from Rust (link libnetcdf, or write the netCDF-4 layout through HDF5).
- A self-run billion-cell benchmark before freezing codecs and chunk sizes.
- User decisions: which transport code reads first and with which method; Monte Carlo export path (DAGMC vs MCNP UM vs Serpent); whether the transport code wants precomputed geometry/sweep caches.
