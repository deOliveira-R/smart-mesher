# Surface tracking vs delta (Woodcock) tracking: data structures, prior art, and a plan for sm-rt

Status: research note, 2026-10-26. Scope: what data structures each tracking mode needs, what Serpent, OpenMC, MCNP, Shift/ORANGE, Geant4 and the rendering literature do, and how `sm-rt` could support both modes from one hierarchy.

Verification legend:
- **[V]** checked this session against a primary source (manual text, source code, PR diff, or the paper's own text).
- **[A]** checked against an abstract, a repository record or a search-engine summary of the source, not the full text.
- **[K]** from background knowledge; citation details believed correct but **not fetched this session**.
- **[U]** unverified or inferred. Treat as a hypothesis.

---

## 0. Executive answer

1. **Different queries, overlapping structures.**
   - *Surface tracking* needs, per cell, the distance to exit along a ray and the neighbour across the exit face: exact planes/quadrics plus face adjacency. `sm-rt` already has this.
   - *Delta tracking* needs two other things: (a) a **majorant** Σ_M(E) valid over a region, and the distance at which the ray leaves that region; (b) **point location** at each tentative collision. It never needs distances to internal cell boundaries.
   - The rendering lesson (PBRT-v4, Yue et al.): the majorant structure should be a **disjoint spatial partition** (grid, kd-tree, octree, lattice) whose nodes carry bounds and a majorant. It is traversed by DDA and yields *majorant segments* `{t_min, t_max, Σ_M}`. An overlapping BVH is a good point-location index but a poor majorant-segment index, because segments must partition the ray.
2. **Surface tracking is a special case of segment-wise delta tracking.** If every segment is exactly one cell and Σ_M = Σ_t of that cell's material, no tentative collision is ever rejected, and the "segment iterator" is just the cell walk. So one abstraction, a majorant-segment iterator over a hierarchy, covers both modes. The hybrid policy picks, per node, whether to descend to cells (surface mode) or stop at a coarse node (delta mode).
3. **Where smart-mesh can beat existing codes.**
   - OpenMC's in-review delta tracking re-locates the particle from the root universe after every flight (`exhaustive_find_cell`) **[V]**.
   - Serpent uses one global, energy-dependent majorant per particle type. It falls back to surface tracking when the rejection probability exceeds 0.9 **[V]**.
   - smart-mesh knows the hierarchy (core → assembly → pin → regions), the exact topology, the material per cell, and a conforming mesh. That allows:
     - **local majorants** per hierarchy node and per energy band;
     - **point location that starts at the majorant node**, not at the root: O(1) lattice index, then a radial binary search in a pin or a small per-node BVH in meshed regions;
     - **track-length tallies kept** wherever the policy chose surface mode;
     - fully deterministic construction, since the majorant is a `max`, which is order-independent.
4. **Delta tracking is a Monte Carlo-only feature.** MOC, Sn and random ray need exact segment lengths per region, so surface tracking stays the core product of `sm-rt`. Delta tracking is an add-on for MC consumers.

---

## 1. Serpent

### 1.1 Delta tracking, majorant, and the switch to surface tracking

- **Mechanism.** Serpent combines surface-to-surface ray tracing with Woodcock delta tracking. Delta tracking is "a rejection sampling technique that enables the random walk to be continued over several material regions without stopping the neutron at each boundary surface" (Leppänen, INAC 2009) **[V]**.
- **Origin.** Woodcock et al., "Techniques used in the GEM code for Monte Carlo neutronics calculations in reactors and other systems of complex geometry", ANL-7050 (1965) **[K]**.
- **Implementation paper.** Leppänen, "Performance of Woodcock delta-tracking in lattice physics applications using the Serpent Monte Carlo reactor physics burnup calculation code", *Ann. Nucl. Energy* 37 (2010) 715–722, doi:10.1016/j.anucene.2010.01.011 **[A]**.
  - Its abstract says the method "performs well in LWR lattice applications, but really shows its efficiency when modeling HTGR particle fuels."
- **Global majorant.** Serpent 2 uses a single majorant per particle type, a function of energy only: the maximum total cross section over all materials on the unionized grid.
  - The Serpent wiki page "Delta- and surface-tracking" describes a uniform majorant (seen only through a search-engine summary; the page returned 403 to direct fetch) **[A]**.
  - OpenMC's re-implementation builds the majorant the same way (§2.3) **[V]**.
- **Threshold, from the Serpent 1 User's Manual (Leppänen, June 18, 2015, §5.11)** **[V]**:
  - Syntax: `set dt <thresh>`. Default **0.9** since version 1.1.1.
  - "0 = never, 1 = always" use delta tracking.
  - Footnote 4 gives the precise meaning: "the threshold parameter determines the highest rejection probability at which the method is used. If the probability is higher than the threshold value, the code switches to the conventional ray-tracing method."
  - So, with **P_rej = 1 − Σ_t(E)/Σ_M(E)**, Serpent uses delta tracking iff P_rej ≤ 0.9.
  - The main text says "probability of sampling a physical collision falls below a user-defined threshold". That wording contradicts the footnote and the 0/1 semantics. Treat the footnote as authoritative.
  - The test uses the material at the current position, and it is made at the start of each track (newer Serpent docs, per a search-engine summary) **[A]**.
- **Related options.**
  - A negative value of `dt` turns on an adaptive optimization of the threshold. Since 1.1.9 it adapts per material; −0.9 is recommended **[V]**.
  - `set blockdt <mats>` forbids delta tracking in the listed materials, for example a large outer reflector made of simple cells **[V]**.
  - Newer Serpent 2 adds `set forcedt` and separate neutron/photon thresholds. The docs note that delta tracking may still be used in CAD (STL) geometry even with the threshold at 0 **[A]**.
- **Unresolved-resonance probability tables.** Serpent offers three majorant strategies **[V]**:
  1. resample the tables and rebuild the material totals and the majorant at every energy change;
  2. use a precomputed majorant from the maximum table values;
  3. switch to surface tracking in the URR.
  - This is the "majorant must bound every stochastic xs realization" problem in miniature.
- **Estimators.**
  - Delta tracking "necessitates the use of the collision flux estimator". Track length "has a better efficiency in small and optically thin cells and in regions of low collision density" (INAC 2009) **[V]**.
  - `set tle 1` forces the track-length estimator and switches delta tracking off (manual §5.x) **[V]**.
  - Leppänen, "On the use of delta-tracking and the collision flux estimator in the Serpent 2 Monte Carlo particle transport code", *Ann. Nucl. Energy* 105 (2017) 161–167 explains the cell-flux CFE and shows its failure at very low collision density **[A]**.
  - Serpent's CFE also scores a fraction of the virtual collisions **[A]**.
- **Geometry simplicity.** The manual says "Neutron transport in Serpent, however, is primarily based on the delta-tracking method", which is why derived surface types (hexagonal prisms and similar) do not slow it down. Fewer surfaces make delta tracking faster (§3.3 footnote) **[V]**.
- **Varying composition.** "It is, in principle, possible to model continuously varying material compositions when the delta-tracking method is used" (§4.1 footnote) **[V]**. Serpent 2 later did this through its multiphysics interface (§1.4).

### 1.2 Point location in universe-based CSG

- **Cells are intersections only (Serpent 1).** A Serpent 1 cell is an intersection of half-spaces; there is no union operator: "the neutron is inside the cell, if and only if it is on the same side of each boundary surface" (Serpent 1 manual §3.3) **[V]**. Serpent 2 supports union and complement cell operators **[U]**.
- **Location after a tentative collision** walks down the universe nesting **[K/U]**:
  - in a universe, test the cells;
  - in a lattice, compute the index directly (O(1) for square/hex lattices) and transform to local coordinates;
  - recurse into the filling universe.
- **Why this is cheap in lattices.** Testing one point against a few cylinders and planes (a sign check) costs much less than intersecting a ray with them. That asymmetry is the main reason delta tracking wins in pin lattices.

### 1.3 TRISO and explicit particle fuel

- **`pbed` card.** `pbed <u0> <uf> "<file>"` reads explicit particle or pebble coordinates (x y z r u). The count is "unlimited, although memory or running time may become a limiting factor if the number exceeds several million" **[V]**.
- **Implicit model.** `disp` gives a statistical dispersion **[V]**.
- **Particle search.** A search mesh over the particle centres is used to find the containing particle **[U]**. It is not described in the 2015 manual; the method is in Leppänen, "Randomly dispersed particle fuel model in the PSG Monte Carlo neutron transport code", M&C+SNA 2007 **[K]**.
- **Why delta tracking wins here.** The TRISO case is the textbook win: thousands of sub-mm spheres per mean free path make surface crossings dominate. "The advantages of delta-tracking become pronounced in geometries containing regions that are small compared to the neutron mean-free-path" (INAC 2009) **[V]**.
  - OpenMC's PR reports ~5× on an HTGR compact at 15% packing fraction (§2.3) **[V]**.

### 1.4 Multiphysics interface, unstructured (OpenFOAM) mesh, and adaptive search mesh

- **Decoupled state points.** Serpent separates temperature/density state points from the geometry. Tuominen, Valtavirta, Peltola, Leppänen, "Coupling Serpent and OpenFOAM for neutronics–CFD multi-physics calculations", PHYSOR 2016 **[V]**:
  - "This is made possible by using rejection sampling for neutron path lengths and on-the-fly TMS temperature treatment for microscopic cross sections."
  - "The mesh based interface features a separate superimposed search mesh to speed up the cell search for finding correct state point information at each interaction point. The size of the search mesh has a considerable effect on both memory usage and required calculation time."
  - In other words: with delta tracking, the density/temperature field only has to be **queried at collision points**. No distances to field-cell boundaries are needed, but the majorant must bound the field (maximum density factor, maximum temperature).
- **TMS (target motion sampling).**
  - References: Viitanen & Leppänen, "Explicit treatment of thermal motion in continuous-energy Monte Carlo tracking routines", *Nucl. Sci. Eng.* 171 (2012) 165–173; and "Target motion sampling temperature treatment technique with elevated basis cross-section temperatures", *NSE* 177 (2014) **[K]**.
  - TMS samples target velocity at the collision and uses a **temperature majorant**: a cross section bounding all temperatures within the material's range over an energy window broadened for thermal motion. Rejection then handles the temperature dependence. TMS therefore *requires* rejection-based (delta) tracking **[K]**.
- **Unstructured-mesh geometry.** Leppänen & Aufiero, "Development of an unstructured mesh based geometry model in the Serpent 2 Monte Carlo code", PHYSOR 2014, JAEA-Conf 2014-003 **[A]** (full text not retrieved).
  - The current Serpent user guide (seen only through search-engine summaries; direct fetch returned 403) says **[A]**:
    - the mesh is read in OpenFOAM polyMesh format;
    - polyhedral cells are decomposed into tetrahedra before transport;
    - face/neighbour data speed up the search when a neutron exits a mesh cell;
    - the search mesh is "essentially equivalent" to the one used by the CAD (STL) type.
  - Internals **[U]**:
    - the search mesh is multi-level and adaptive: a coarse Cartesian mesh whose cells holding too many candidates are subdivided to user-given sizes per level;
    - each leaf lists candidate cells;
    - point location tests only those candidates.
- **CAD (STL) geometry.** Leppänen, "Methodology, applications and performance of the CAD-based geometry type in the Serpent 2 Monte Carlo code", *Ann. Nucl. Energy* 176 (2022) 109259 **[A]**. Earlier paper: "CAD-based geometry type in Serpent 2 – application in fusion neutronics", M&C 2015 (full ITER model) **[A]**.
  - Talamo, Gohar, Leppänen describe a mesh-adaptive search on STL models: *Ann. Nucl. Energy* (2018) and *Trans. ANS* 115 (2016) **[A]**.
  - The `-checkstl` utility samples points inside STL solids and fires rays. All rays must agree, otherwise the solid is not watertight. This implies a **ray-parity point-in-solid test** **[A]**.
  - The guide warns that "the STL search mesh stores pre-assigned cell information", which can hide geometry errors at high mesh resolution **[A]**.
  - So the search-mesh leaves that lie wholly inside one solid are pre-assigned and need no ray test **[A/U]**.
  - Tracking in STL solids is primarily delta tracking, which avoids triangle distance queries. Surface tracking (distance to the nearest triangle via the search mesh) is the fallback **[U]**.

### 1.5 Local or alternative majorant variants

- **Weighted delta tracking.** Serpent 2 has it ("Implementation of weighted delta tracking with scattering in the Serpent 2 …", VTT record) **[A]**. The idea: Carter, Cashwell, Taylor, *NSE* 48 (1972) **[K]**; Morgan & Kotlyar, "Weighted-delta-tracking for Monte Carlo particle transport", *Ann. Nucl. Energy* 85 (2015) **[K]**.
  - It allows a sampling cross section that is not a strict bound, at the cost of weight variance.
- **Negative-weight Woodcock.** Legrady et al., "Woodcock tracking with arbitrary sampling cross section using negative weights", *Ann. Nucl. Energy* 102 (2017) (GUARDYAN GPU code) **[K]**.
- **Regional majorants (Chinese work).** Guo & Chen, "Multi-regional delta-tracking method for neutron transport tracking in Monte Carlo criticality calculation", *Sustainability* 10(7) (2018) 2272 **[A]** (authors and venue verified; method details not retrieved, page returned 403).
  - A follow-up appeared in *Atomic Energy Science and Technology* (2019) **[A]**.
  - Presumably it uses per-region majorants and stops at region boundaries **[U]**.
- **No spatial majorants in Serpent itself.** I found no evidence that Serpent uses spatially local majorants. Its hybrid relies on the global majorant plus the threshold switch **[U]**.

---

## 2. Other neutronics codes

### 2.1 MCNP6 unstructured mesh (UM)

- **Design.** The UM is a mesh universe embedded in CSG. Abaqus-built meshes use first- and second-order tet, pentahedron and hex elements. Particles move element to element, and path-length tallies are scored per element (Martz et al., *Nuclear Technology* 2012, per a search-engine summary) **[A]**.
- **Neighbour lists.** Zieb & Martz, LA-UR-15-27000 (2015) **[V]**:
  - nearest-neighbour lists for elements were first built lazily during tracking (2012) and are now built at setup (2015);
  - there is "a more sophisticated method of selecting the intersecting face of an element";
  - the gap/overlap checks for non-contiguous meshes cost little;
  - UM beats CSG from roughly 16k–90k elements, depending on the problem.
- **Tracking mode.** MCNP UM is surface tracking element by element; it has no delta tracking **[V for the element walk; K for "no DT"]**.

### 2.2 Shift / ORANGE / Celeritas

- **ORANGE is surface tracking with a BVH/BIH for location** **[V]**, from Celeritas source `src/orange/univ/SimpleUnitTracker.hh`:
  - `initialize` uses "the BVH to locate a position that's inside" (a `find_volume_where` tree walk with exact sense evaluation at the leaves);
  - `cross_boundary` uses surface→volume connectivity: if the crossed surface has fewer than 3 neighbours, they are tested linearly; otherwise the tree is traversed;
  - `BIHBuilder.hh` follows Wächter & Keller's BIH (2006), splitting on bounding-box centres; volumes with infinite boxes are kept apart **[A]** (from a code-browser summary).
  - A separate `RectArrayTracker` handles rectilinear arrays (O(1) index) **[A]**.
- **Shift** uses ORANGE (and other geometries) with surface tracking on CPU and GPU (Hamilton & Evans, *Ann. Nucl. Energy* 2019) **[K]**.
- **Celeritas** is HEP transport over ORANGE or VecGeom. No geometric Woodcock tracking was found **[U]**.

### 2.3 OpenMC

There is no delta tracking in a released OpenMC as of this date. Two open PRs by `nuclearkevin` (GitHub user) exist:

- **#3971 "Delta Tracking"**: open, changes requested, last activity Oct 2026 **[V]**.
  - **Majorant:** unionize all nuclide grids (including URR probability tables at their maximum band). Take the per-material maximum with S(α,β) included, the maximum over the temperature grid, and the maximum density multiplier for distributed densities. The global majorant is the maximum over materials. Photons get a separate majorant.
  - **Tallies:** collision estimator only; track length is disallowed and surface filters are rejected. Multigroup, windowed multipole and NCrystal are unsupported.
  - **Benchmarks:** Jezebel and pincells show modest gains; HTGR compact ~5× faster; gas-cooled microreactor 0.66–0.71× (slower); BEAVRS 0.43–0.45× (slower); ARC-100 SFR 2.85–2.96× faster. Conclusion: strong thermal absorbers inflate the global majorant.
- **#4044 "Hybrid delta tracking"**: draft, Aug 2026 **[V]**.
  - **Two hybrids.** "Hybrid-in-cross-section" (Serpent's Eq. 9) and "hybrid-in-energy": delta tracking above 10 eV for neutrons and 100 keV for photons.
  - **Mode rule, from the diff:** `if (macro_xs().total / majorant()) > 1 - xs_threshold → delta`, with `xs_threshold = 0.9`. This equals Serpent's P_rej ≤ 0.9. The decision is re-made after every collision.
  - **Point location, from the diff:** after every move in delta mode the code resets all coordinate levels and calls `exhaustive_find_cell` from the root. That is a full root-down search per flight.
  - **Runtime speedups vs surface tracking:**
    - GCMR: 1.8× (cross-section rule), 3.0× (energy rule).
    - BEAVRS: 1.6× (both rules).
    - ARC-100: 3.5× (both rules).
    - "Octomak" (fusion): much slower with either rule, since point location is costly relative to neighbour-list-accelerated surface search.
  - **Verdict:** the energy rule was better.
- **Earlier discussion.** An OpenMC Discourse thread (#1461, "Delta tracking in OpenMC") mentions local majorants and weighted delta tracking as follow-ups **[A]**.

### 2.4 MCATK (LANL)

- **Paper.** Morgan, Trahan, Burke, Josey, Niemeyer, "Hybrid-delta tracking on a structured mesh in MCATK", arXiv:2306.07847 (2023) **[V]**.
- **Method.** **Surface tracking on mesh cells** with **delta tracking inside each cell**. There is one global microscopic majorant σ_M(E); the cell majorant is N_i·σ_M (cell number density times σ_M). Tentative collisions are accepted with Σ_t,i/(N_i σ_M).
- **Tallies.** Particles stop at every cell boundary, so **track-length tallies remain valid**.
- **Results.** 1.5–1.75× for k-eigenvalue and 1.2–1.6× for fixed source.
- **Lesson.** Local majorants on mesh cells let you keep both track-length tallies and rejection sampling inside cells whose material is mixed or varies. This is the "segment = cell" end of the spectrum in §0.

### 2.5 Geant4: voxels and safety

- **Smart voxels.** Per logical volume, Geant4 builds up to three levels of 1D slicings ("smart voxels") over the daughter volumes.
  - The `smartless` parameter defaults to 2.0 (`G4LogicalVolume.hh`: `G4double fSmartless = 2.0;`) and caps the average number of slices per daughter. `kMaxVoxelNodes = 1000` (`voxeldefs.hh`). Slice quality = Σ(volumes per node)/(non-empty nodes) (`G4SmartVoxelHeader.cc`) **[V]**.
- **Safety.** The isotropic **safety** is min(mother DistanceToOut, daughter distances) (`G4VoxelSafety.cc`) **[A]**.
  - If the next physics step is shorter than the safety, no boundary computation is needed. This is the standard trick for skipping intersection tests in surface tracking, and it is how condensed-history charged transport stays cheap **[K]**.
- **DAGMC analogue.** Shriwise, Davis, Jacobson, Wilson, "Particle tracking acceleration via signed distance fields in direct-accelerated geometry Monte Carlo", *Nucl. Eng. Technol.* 49(6) (2017) 1189–1198 **[A]**. Precomputed signed distance fields give a safety that skips OBB-tree ray fires.

### 2.6 DAGMC

- **Geometry queries.** Ray fire against triangulated surfaces through an OBB tree (MOAB). Point-in-volume uses ray parity. Surface tracking only **[K]**.
- **Reference.** Wilson et al., "Acceleration techniques for the direct use of CAD-based geometry in fusion neutronics analysis", *Fusion Eng. Des.* 85 (2010) **[K]**.
- **With OpenMC delta tracking.** OpenMC's delta-tracking PR works with DAGMC geometry, but boundary conditions on DAGMC surfaces are rejected **[V]**.

---

## 3. Volumetric rendering: what the graphics community knows

Rendering has handled spatially varying media with null collisions for a decade. The citations below are **[K]** unless marked otherwise.

- **Residual ratio tracking.** Novák, Selle, Jarosz, "Residual ratio tracking for estimating attenuation in participating media", *ACM TOG* 33(6) (SIGGRAPH Asia 2014).
  - *Ratio tracking* estimates transmittance as a product of (1 − Σ/Σ_M) over tentative collisions, with no Russian roulette.
  - *Residual* ratio tracking subtracts a *control* extinction Σ_c, integrates it analytically, and tracks only the residual.
  - Neutronics analogue: transmission estimates for point detectors and next-event estimators, and uncollided flux.
- **Decomposition tracking.** Kutz, Habel, Li, Novák, "Spectral and decomposition tracking for rendering heterogeneous volumes", *ACM TOG* 36(4) (SIGGRAPH 2017).
  - Decomposition: Σ = Σ_c + Σ_r. A collision from the constant part Σ_c (for example the node's *minorant*) is real without any cross-section lookup; only the residual is rejection-sampled.
  - Spectral tracking handles a vector of wavelengths. Neutron energy is a scalar, so that part is irrelevant here.
- **Null-scattering path integral.** Miller, Georgiev, Jarosz, "A null-scattering path integral formulation of light transport", *ACM TOG* 38(4) (SIGGRAPH 2019).
  - It puts real and null collisions into one path-space formulation. That allows MIS between tracking strategies and **non-bounding majorants**, made unbiased by weights.
  - This generalizes weighted delta tracking (Carter 1972; Legrady 2017).
- **Survey.** Novák, Georgiev, Hanika, Jarosz, "Monte Carlo methods for volumetric light transport simulation", *Computer Graphics Forum* 37(2) (Eurographics STAR 2018). Also the SIGGRAPH 2018 course "Monte Carlo methods for physically based volume rendering".
  - It surveys delta/ratio/residual tracking, majorant selection and spatial partitions. It states the key point plainly: efficiency is governed by how tight the majorant is.
- **Adaptive partitions for Woodcock tracking.**
  - Yue, Iwasaki, Chen, Dobashi, Nishita, "Unbiased, adaptive stochastic sampling for rendering inhomogeneous participating media", *ACM TOG* 29(6) (SIGGRAPH Asia 2010) **[A]**. It builds a **kd-tree with per-node majorants**, partitioned by a cost model of expected sampling cost.
  - "Toward optimal space partitioning for unbiased, adaptive free path sampling of inhomogeneous participating media", *CGF* 30(7) (Pacific Graphics 2011) 1911–1919 **[A]**. It gives an analytic estimate of sampling efficiency for any partition and uses it to choose the partition.
- **Low-resolution majorant grid.** Szirmay-Kalos, Tóth, Magdics, "Free path sampling in high resolution inhomogeneous participating media", *CGF* 30(1) (2011): a coarse majorant grid traversed by DDA.
- **PBRT-v4** (Pharr, Jakob, Humphreys, *Physically Based Rendering*, 4th ed., 2023; source `mmp/pbrt-v4`) **[V]**:
  - `MajorantGrid` stores bounds, resolution and one `Float` per voxel.
  - `GridMedium` and `RGBGridMedium` use **16×16×16** majorant grids; `NanoVDBMedium` uses **64×64×64** (`media.cpp`). Each voxel stores the maximum density over the voxel's bounds.
  - `DDAMajorantIterator` steps voxels by 3D DDA (Amanatides & Woo 1987 **[K]**) and returns `RayMajorantSegment{tMin, tMax, sigma_maj}`.
  - `SampleT_maj` exists in `media.h` **[V]**. Its behaviour below is **[K]**: it consumes segments, samples an exponential distance with the segment's Σ_M. If the sample passes `tMax`, it moves to the next segment and resamples. This is valid because of memorylessness.
  - Homogeneous media return a single segment, so no null collisions occur.
  - The book's exercises note two weaknesses: the fixed resolution is not adaptive (pointing to Yue 2011), and RGB majorants are loose because they ignore spectral variation. The second is exactly the energy-dependence problem neutronics has.
- **OpenVDB / NanoVDB.**
  - Museth, "VDB: High-resolution sparse volumes with dynamic topology", *ACM TOG* 32(3) (2013).
  - Museth, "Hierarchical digital differential analyzer for efficient ray-marching in OpenVDB", SIGGRAPH Talks 2014: DDA at each tree level, so empty or low-majorant tiles are skipped in one step.
  - NanoVDB (Museth, 2021) is the GPU-friendly flattened version.
- **Physics analogues.** Galtier et al., "Integral formulation of null-collision Monte Carlo algorithms", *JQSRT* 125 (2013) **[K]**. Charged-particle "integral approach" for energy-loss-dependent cross sections in Geant4 **[K/U]**.

### 3.1 The data-structure lesson

| Query | Surface tracking | Null-collision (delta) tracking |
|---|---|---|
| Per-step primitive | distance to exit the current cell (all faces/surfaces of the cell) | sample a distance against Σ_M of the current majorant region |
| Per-region data | exact planes or quadrics, face → neighbour adjacency | bounds (box or lattice cell) plus Σ_M(E) (and optionally a minorant Σ_c(E)) |
| Traversal | cell walk across faces (exact topology) | DDA over a **disjoint** partition (grid, kd-tree, octree, lattice); hierarchical DDA skips coarse nodes |
| Extra query | none, since the cell is known after crossing | **point location** at each tentative collision, then a material Σ_t(E) lookup |
| Region exit | the exact face crossing, which gives the neighbour | a cheap box or lattice-cell exit (no internal surfaces) |
| Estimator | track length (exact segment per cell) | collision; or a "tentative-collision path estimator" (score 1/Σ_M per tentative collision) **[U, own derivation, standard]** |
| Bit-exactness concerns | robust face-crossing predicates | majorant must strictly bound Σ (round up); RNG stream per history |

**Why disjoint?** Segments must partition the ray so that each length is sampled against exactly one Σ_M. In a BVH, sibling AABBs overlap, so the traversal would have to take, over each overlap interval, the maximum of the majorants of all overlapping nodes. That is doable but messy and loose.

- A BIH or BVH stays an excellent **point-location** index (it is what ORANGE uses).
- The **majorant** index should be a lattice, kd-tree or octree.
- The leaf candidate lists of that partition are themselves a point-location structure: Serpent's search mesh = PBRT's majorant grid + Geant4's smart voxels.

---

## 4. Point location for delta tracking

| Method | Cost per query | Notes |
|---|---|---|
| Root-down CSG search (OpenMC `exhaustive_find_cell`) **[V]** | Σ over levels of (cells tested × surfaces per cell) | Simple, but repeats work the hierarchy already knows; it is the reason "Octomak" is slow in #4044 |
| Lattice index | O(1): floor((x − x0)/pitch), or hex axial coordinates | Then transform to local coordinates; depth = number of nesting levels |
| Pin interior (concentric rings) | O(log k): binary search of r² against sorted radii² (plus an azimuthal sector by atan2 or a sign test) | Much cheaper than ray-cylinder intersection; this is why delta tracking is cheap in lattices |
| BVH/BIH over cells (ORANGE, sm-rt hint-less locate) **[V]** | O(log n) box tests + exact cell tests at leaves | Exact test on a polyhedral cell = f plane evaluations |
| Uniform or adaptive grid with candidate lists (Serpent search mesh) **[A]** | O(1) to O(levels) + candidate tests | Leaves fully inside a cell can be pre-assigned with zero tests (Serpent STL) |
| Jump-and-walk from the previous cell (mesh "visibility walk") **[K]** | O(cells crossed) | On a straight path from the last known cell this costs as much as surface tracking, so it gives nothing for DT flights. It is useful for *nearby* re-location (after a short move, or a small displacement with rounding) |
| Geant4 smart voxels **[V]** | O(levels) + daughters in the voxel | Per logical volume; tuned by `smartless` |

**Cost model** (per unit path length, energy fixed between collisions):

- Surface mode: `C_ST ≈ n_b · (C_cross + C_xs') + Σ̄_t · C_coll`
  - n_b: boundary crossings per cm.
  - C_cross: per-cell exit computation (f_cell plane tests ≈ 5f flops, quadric tests ≈ 20–40 flops each) + adjacency read.
  - C_xs': Σ_t lookup, paid only on a material change. In surface mode the cell is always known, so real collisions need no location.
- Delta mode in node *v*: `C_DT ≈ Σ_M,v · (C_rng + C_loc,v + C_xs) + n_v · C_node + Σ̄_t · C_coll`
  - C_loc,v: location below node v.
  - C_xs: Σ_t lookup for the located material; cacheable per flight because E is fixed.
  - n_v: node exits per cm (box or DDA step, a few flops).
- Delta wins iff `n_b · (C_cross + C_xs') > Σ_M,v · (C_rng + C_loc,v + C_xs) + n_v · C_node`. Location is paid at *every* tentative collision, real ones included.
  - **Wins:** TRISO, fine pin and ring discretizations, meshed regions with many small cells, high-energy neutrons (small Σ relative to cell size).
  - **Loses:** low-density regions under a high majorant (gas plenum, voids, coolant channels next to control absorbers, thermal neutrons near B₄C/Gd/Hf), where Σ_M − Σ̄ is large; and complex geometries where location is costly (fusion CSG).

---

## 5. When delta tracking wins or loses

- **Wins.**
  - Optically small regions relative to the mfp: TRISO (Leppänen 2010 abstract; OpenMC ~5× HTGR compact) **[A/V]**.
  - Fast spectra with a flat majorant (ARC-100 ~3× in OpenMC) **[V]**.
  - Geometries where surface distances are costly but point tests are cheap: derived surfaces, STL, many-faced polyhedra (Serpent manual §3.3) **[V]**.
  - Continuously varying density or temperature, where a field queried at collision points replaces field-cell crossings (Serpent multiphysics + TMS) **[V/K]**.
- **Loses.**
  - Localized strong absorbers inflate the global majorant: control rods, burnable absorbers (Serpent manual §5.11) **[V]**; BEAVRS 0.43× in OpenMC with pure delta tracking **[V]**.
  - Voids and gases under a dense majorant.
  - Costly point location (Octomak in #4044) **[V]**.
- **Tallies.**
  - Track length is unavailable under delta tracking; the collision estimator has higher variance in thin or low-collision regions (INAC 2009; Leppänen 2017) **[V/A]**.
  - Surface (current) tallies need explicit crossing detection, which OpenMC simply rejects **[V]**.
  - The tentative-collision estimator scores w/Σ_M at every tentative collision, real or virtual. Its expectation per segment is the segment length (flux-weighted). It is unbiased but noisier than true track length **[U, standard derivation]**. Serpent's "fraction of virtual collisions" scoring is a variant **[A]**.
- **MOC, Sn and random ray do not use it.** They need exact segment lengths per flat-source region, so surface tracking (exact cell walk) is mandatory for those consumers.

---

## 6. Recommendation for smart-mesh (`sm-rt`)

### 6.1 One hierarchy, two iterators

Build a single **tracking hierarchy** of disjoint spatial nodes:

```
Root (global box)
 └─ Lattice node (core map: square/hex; O(1) index, DDA traversal)
     └─ Lattice node (assembly: pin lattice)
         └─ Universe node (pin type; instanced)
             └─ Leaf: cells (exact planes/quadrics, face adjacency, material id)
 └─ Partition node (kd/octree over a meshed, non-lattice region; disjoint boxes)
     └─ Leaf: candidate cell list (+ "pre-assigned" flag if box ⊂ one cell)
```

**Every node stores:**
- `bounds`: AABB or lattice-cell frame;
- `mset: u32`, the id of the deduplicated **set of materials** present below the node (density/temperature-field max factors folded in);
- `kind`: lattice, universe or partition, with child indexing in CSR form;
- `mode_hint: u8`, precomputed policy bits (§6.4).

**Cells keep what `sm-rt` already has:** exact planes and analytic surfaces, face→neighbour CSR, material/region id. The existing BVH over cells stays as the **hint-less point locator inside partition leaves** and for global fallback.

**Two iterators over the same tree yield `MajorantSegment { t0, t1, sigma_maj, node }`:**
- **Surface iterator.** Descend to leaves and walk cells. Each segment is one cell with `sigma_maj = Σ_t(cell, E)`. Acceptance is 1, so only real collisions occur and the track-length estimator is valid. This is the current `sm-rt` walk, unchanged for MOC.
- **Delta iterator.** Stop at a node chosen by the policy and DDA across sibling lattice cells or partition boxes. `sigma_maj = M[mset][band(E)]`.

Sampling is PBRT's `SampleT_maj` loop:
1. sample s against `sigma_maj`;
2. if s passes `t1`, go to the next segment and resample;
3. otherwise, at the tentative collision, locate the cell (if the segment is a coarse node), look up Σ_t, and accept with probability Σ_t/Σ_M.

Because both modes produce the same segment type, mixing modes along one flight is trivial and unbiased. A segment's mode depends only on the node, never on history.

### 6.2 Point location starting at the majorant node (the main win over OpenMC and Serpent)

At a tentative collision inside node v, the point is known to lie in v. So:
- **Lattice:** O(1) index from the local coordinates.
- **Pin universe:** r² binary search over ring radii, plus an azimuthal sector by sign tests. Fall back to exact half-space tests only for non-concentric pins.
- **Partition leaf:** if pre-assigned, done. Otherwise, test the candidate list, or descend the per-leaf BVH.
- **Cache per flight** (E is constant): Σ_t for the last few materials, keyed by material id.

Expected C_loc ≈ (lattice depth × ~10 flops) + (log₂ rings × ~3 flops) for LWR pins. Compare a root-down CSG search (OpenMC #4044 **[V]**) or a global search mesh (Serpent). Jump-and-walk is used only for short re-locations, for example after boundary nudges.

### 6.3 Majorant storage: per material-set, per energy band

- **Band table.**
  - Define B energy bands: log-spaced, refined in the resolved-resonance range. Default B = 256 over 1e-5 eV–20 MeV, about 9 bands per lethargy unit.
  - For each distinct material set S, store `M[S][b] = max_{m∈S} max_{E∈band b} Σ_t,m(E) · f_m^max` as an f32 rounded up (`next_up` after conversion).
  - With lin-lin (or log-log, monotone between points) interpolation, the maximum over a band is attained at grid points inside the band or at the interpolated band edges, so this is a strict bound. Include the URR table maximum, the S(α,β) maximum and the TMS temperature maximum, as OpenMC's PR does **[V]**.
- **Memory arithmetic.** Take a 10⁶-cell model with ~2×10⁶ tree nodes. Naive per-node bands would cost 2×10⁶ × 256 × 4 B ≈ **2 GB**, which is too much. With deduplication:
  - per-node `mset` id: 2×10⁶ × 4 B = **8 MB**;
  - distinct sets: a depleted LWR core with ~5×10⁴ pins, each with distinct depleted rings, gives about 5×10⁴ pin-level sets plus ~10³ assembly/core sets plus a few hundred for meshed regions, so S ≈ 5×10⁴;
  - table: 5×10⁴ × 256 × 4 B ≈ **51 MB**. With u16 log-quantized entries relative to a per-set scale (rounded up), ≈ 26 MB.
  - Fresh or instanced cores (no depletion) give S ~ 10²–10³: **< 1 MB**.
  - If S turns out large, bound depleted fuel with one envelope curve per enrichment or assembly class. That gives a looser majorant and a tiny table.
  - **Caveat:** the set count S has not been measured on real models; these figures are estimates.
- **Exact option for small sets.** If |S| ≤ 8, compute Σ_M,v(E) = max_m Σ_t,m(E) once per flight on node entry, using the per-material lookups already cached for that flight. That is tight (no band looseness) and costs no extra memory.
- **Band looseness in resonances.** Within a band, U-238 peaks dominate. Mitigations: finer bands below ~10 keV; or the energy-split policy (§6.4, delta mode only above the resolved range), which OpenMC found to beat the cross-section rule **[V]**.
- **Minorant (decomposition tracking).** Optionally store `m[S][b] = min Σ_t`. Tentative collisions with ξ·Σ_M < m are real without a lookup (Kutz 2017). This is cheap to add but has a small payoff in neutronics. Defer it.

### 6.4 Hybrid policy (deterministic)

Precompute per node and per band, at build time, using the §4 cost model with Σ̄ ≈ the volume-weighted Σ_t of the node and n_b estimated from cell chord lengths (mean chord 4V/A):
- `delta_ok[v][b] = P_rej(v,b) ≤ τ` with τ = 0.9 (Serpent/OpenMC default), where P_rej(v,b) = 1 − Σ̄_t(v,b)/M[mset(v)][b];
- `delta_wins[v][b] = C_DT(v,b) < C_ST(v,b)`;
- the descent rule: at node v in band b, if both hold, emit a coarse segment; otherwise descend, and at the leaves emit cell segments in surface mode.

At runtime, refine with Serpent's local test on the *current* material: if Σ_t(current)/Σ_M < 1 − τ, use surface mode for this flight. This is a pure function of (node, material, E), never of thread or timing.

**Do not port Serpent's adaptive threshold optimization as-is.** It changes behaviour with run history. If you want it, update thresholds only between batches with a fixed-order reduction.

**User overrides:** `block_delta(materials | regions)` and `force_delta(...)`, matching Serpent's `blockdt`/`forcedt`.

**Tallies:** in surface segments use track length; in delta segments use the collision estimator or the tentative-collision path estimator. The tally for a region sums both. **Partly unverified:** the tentative-collision path estimator is my own derivation (marked [U] in §5); Serpent and OpenMC both apply one estimator for the whole run, so this mixing is not established practice. Validate it with tests. Surface-current tallies force surface mode in nodes that contain the tallied faces.

### 6.5 Per-step cost summary

| Event | Surface mode | Delta mode |
|---|---|---|
| Per boundary | f plane dots + quadric roots + CSR neighbour read (~50–200 flops, 1–2 cache lines) | only node exits: box slab test or DDA step (~10 flops) |
| Per tentative collision | none | 1 RNG + 1 log + location below node (~20–100 flops; one BVH descent of ~log n in meshed leaves) + cached Σ_t + 1 RNG |
| Per real collision | physics | physics (same) |
| Tally | track length per cell segment | collision estimator (or the tentative-collision path estimator) |

### 6.6 Determinism (binding per CLAUDE.md)

- **Construction.**
  - `max` and `min` over f64 values are exact and order-independent (NaN excluded: reject non-finite cross sections at build time), so rayon reductions over materials and grid points give bit-identical tables for any thread count.
  - Rounding to f32 is up-only.
  - Set deduplication uses a sorted material-id vector → `BTreeMap`/`IndexMap`, never `HashMap` iteration.
  - The kd/octree build uses deterministic split rules (median by sorted key, ties by cell id).
- **Runtime.**
  - Use a counter-based RNG per history (for example Philox keyed by (seed, batch, history)). Delta tracking consumes a variable number of random numbers, so streams must not be shared or interleaved.
  - The mode decision is a pure function of the state.
- **Safety net.** A majorant violation (Σ_t > Σ_M at a tentative collision) is a typed error in debug runs and a counted event in release runs. Never clamp silently. Optionally switch to weighted delta tracking (Carter 1972; Miller 2019) so that bands that are not strict bounds stay unbiased.
- **Test.** Run the same history set with `RAYON_NUM_THREADS=1` and with the default thread count, under the surface, delta and hybrid policies, and compare hashes of the tallies.
- **Analytic checks.**
  - Uncollided transmission through a homogeneous slab under all three modes.
  - A two-region slab with a known flux.
  - Acceptance-rate statistics: the measured acceptance should match Σ̄_t/Σ_M.

### 6.7 What to build first

1. `MajorantSegment` iterator trait in `sm-rt` with the surface implementation (a wrapper over today's walk).
2. Material-set dedup + band table builder (needs a total-xs provider; physics data stays outside `sm-rt` behind a trait).
3. Lattice/universe nodes with O(1) location and ring binary search.
4. Delta iterator + the hybrid policy + the determinism tests.
5. A partition (kd/octree) over meshed regions with candidate lists, reusing the existing BVH inside leaves.

---

## 7. Source list

**Fetched or checked this session:**
- Leppänen, *Serpent User's Manual*, 2015-06-18 (serpent.vtt.fi/download/Serpent_manual.pdf), §§3.3, 3.8.2, 4.x, 5.11 **[V]**.
- Leppänen, "On the use of the continuous-energy Monte Carlo method for lattice physics applications", INAC 2009 (sarjaweb.vtt.fi) **[V]**.
- Tuominen, Valtavirta, Peltola, Leppänen, PHYSOR 2016 (VTT OA repository) **[V]**.
- OpenMC PR #3971 and PR #4044 (description and diff) **[V]**.
- Morgan et al., arXiv:2306.07847 **[V]**.
- Zieb & Martz, LA-UR-15-27000 **[V]**.
- Celeritas `orange/univ/SimpleUnitTracker.hh` and `BIHBuilder.hh` **[V/A]**.
- pbrt-v4 `src/pbrt/media.{h,cpp}` and the pbr-book.org 4ed "Media" chapter **[V]**.
- Geant4 `G4LogicalVolume.hh`, `voxeldefs.hh`, `G4SmartVoxelHeader.cc` **[V]**.

**Abstract or summary only [A]:**
- Leppänen 2010 (*ANE* 37); Leppänen 2017 (*ANE* 105); Leppänen 2022 (*ANE* 176).
- Leppänen & Aufiero, PHYSOR 2014.
- Serpent wiki and docs pages (403 on direct fetch).
- Guo & Chen 2018.
- Yue 2010/2011.
- Shriwise et al. 2017.
- Martz 2012.

**Not fetched [K]:**
- Woodcock 1965; Carter et al. 1972; Morgan & Kotlyar 2015; Legrady 2017; Viitanen & Leppänen 2012/2014.
- Novák 2014/2018; Kutz 2017; Miller 2019; Szirmay-Kalos 2011.
- Museth 2013/2014/2021; Amanatides & Woo 1987.
- Wilson 2010; Hamilton & Evans 2019; Galtier 2013.
