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

## 5. Layers

Your three layers map as: **geometry = L0 shape + L1 topology**, **partition = L2 regions (+ L3 attributes)**, **mesh = L5 mesh topology + L6 mesh geometry + L7 interfaces**. The research adds L4 (meshing intent), L8 (fields), L9 (distribution) and L10 (history/provenance). The full table is in `docs/graph/concepts.yaml` (`layers:`).

Note: SCOREC/PUMI's "partition model" means *parallel* parts. To avoid that clash, our material/color layer is called **Regions**.

All layers coexist and are linked by **classification**: every mesh entity points to the lowest-dimensional topology entity that contains it, with parametric coordinates. A validator enforces the cross-layer rules:
- classification closure
- materials partition the volumes exactly
- every topology face has two sides
- one declared orientation convention
- overlap reciprocity
- geometry-bound vs mesh-bound set lifetime, so remeshing keeps intent

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
