# Candidate sm-rt-api: exact geometry and hierarchy first

## Baselines (sources)
- DAGMC `DagMC` API (raw header, fetched): `ray_fire(volume, xyz, uvw, next_surf, dist, RayHistory*, dist_limit, ray_orientation)`, `point_in_volume(volume, xyz, result, uvw, history)`, `find_volume`, `test_volume_boundary`, `closest_to_location`, `next_vol(surf, old_vol)`, `get_angle`, `setup_impl_compl`, `overlap_thickness`, `numerical_precision`. https://github.com/svalinn/DAGMC (src/dagmc/DagMC.hpp)
- double-down: double-precision Embree custom primitives over MOAB; README confirms double-precision intersection and bounding boxes. The single-precision-traversal claim in ir.md 5.2 was not verified here. https://github.com/pshriwise/double-down
- OpenMC: CSG surfaces/regions/cells, universes, rectangular and hex lattices; boundary types transmission, vacuum, reflective, periodic, white, albedo; DAGMC requires imprinted, merged, watertight models. https://docs.openmc.org/en/stable/usersguide/geometry.html
- Lost particles: forum threads and the KIT comparison (lost fractions about 4e-6 % McCAD, 2e-4 % MCNP6, 3e-4 % DAGMC) tie DAGMC losses to watertightness and overlaps; OpenMC's remedy is `check_watertight`/`check_overlaps`. https://openmc.discourse.group/t/issue-with-lost-particles-from-dagmc-geometry/2663 ; https://publikationen.bibliothek.kit.edu/220097316/3816633 ; MIT thesis on CSG vs DAGMC rates: https://dspace.mit.edu/handle/1721.1/157233 (abstract only read).

## Where this design differs
1. Exact hits on analytic surfaces (no faceting error, no watertightness requirement for them).
2. Hierarchy is the top of the structure; N copies share one prototype structure.
3. Next region is read from face sides; search only in `locate`.
4. Side-consistent crossings replace `RayHistory` and distance cutoffs.
5. Deck faces are applied inside `step`; response boundaries are reported (no RNG in the tracker).
6. Facet input is rejected, not repaired, when not watertight; findings are located by face id.
7. Cache: content-keyed, `.smsh`-compatible blob.

## Stage-0 ACRs (frozen crates)
- sm-geom-api: `CsgCell::ray_hits` hard-codes `TOL = 1e-9`; take a tolerance argument.
- sm-geom-api: `ParametricSurface::ray_hits` projects every hit for `uv`; add a `want_uv: bool` or a cheaper hook.
- sm-model-api: expose a hash of `Instancing` (`canonical_visit` exists on Model) as a free function.
- sm-mesh-api: a connectivity hash accessor on `MeshRead` for the cache key.
