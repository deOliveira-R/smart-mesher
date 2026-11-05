# Reference fixtures (external, third-party)

## dagmc_two_volume.h5m (W3b Q6, plan item 0.9)

A MOAB-written DAGMC geometry used as the reference for h5m export (EXPORT Tier A
compares the set and tag tables value by value).

- Source: https://github.com/svalinn/DAGMC, path `src/dagmc/tests/test_dagmc_impl.h5m`
  (here renamed `dagmc_two_volume.h5m`; bytes unchanged).
- Upstream commit fetched: `3b46ba3c9708bd2bd3923dac79ef8ea601faad64` (develop, 2026-11-04).
  The last upstream commit touching the file is `e78144810326eabed7d72a1adba6cb5bf6cfe070`
  (2018-10-09, "Update cmake for new macros").
- SHA-256: `52a13130e72fa74e80d1b07b3851ff2dc3b0d51b9891a615a094f2a432e2954d` (63140 bytes).
- Licence: Simplified BSD (2-clause), (c) 2003-2020 the DAGMC development team. It covers every
  file in the DAGMC repository except `pyne/` and `tally/lapack/`; this file is in neither.
  Redistribution in binary form requires the notice and disclaimer: see `LICENSE-DAGMC.txt`
  (verbatim copy of upstream `LICENSE.txt`).
- Treated as untrusted data: downloaded into an empty directory, read only by `h5dump`
  (conda-forge HDF5 in a container, input mounted read-only), never executed.
- `dagmc_two_volume.h5dump.txt`: full `h5dump` output (header and data), 968 lines.
  Opaque tag values (NAME, CATEGORY, EXTRA_NAME*) appear as hex bytes (NUL padded, 32 bytes).

### Contents (decoded from the dump)

- Root `/tstt`, `max_id` 181. 24 nodes, 36 `Edge2`, 36 `Tri3`.
- Geometry sets by CATEGORY: 24 Vertex, 36 Curve, 18 Surface, 2 Volume, 4 Group (84 sets).
  GEOM_DIMENSION is 0/1/2/3 on the first 80 (volume set handles 176 and 177).
- NAME tag on the 4 groups: `picked` (no members), `mat:Graveyard`, `mat:Steel`,
  `mat:Steel_comp`. EXTRA_NAME0 holds the same three `mat:` names, EXTRA_NAME1 the `mat_` forms.
- Membership as read from the set table: `mat:Graveyard` holds volume 177, `mat:Steel` holds
  volume 176, `mat:Steel_comp` holds volume 177. So volume 177 is in two material groups, a quirk
  of the upstream test file that matters to any reader of material groups.
- Graveyard: yes, the group `mat:Graveyard`.
- Other tags: FACETING_TOL 0.001, GEOMETRY_RESABS 1e-6, GEOM_SENSE_2, GEOM_SENSE_N_ENTS,
  GEOM_SENSE_N_SENSES (surface-volume sense), GLOBAL_ID, OBB, and the DIRICHLET_SET,
  NEUMANN_SET, MATERIAL_SET, EXTRA_NAME* tag definitions.

Other candidates checked: DAGMC `test_dagmc.h5m` and `test_geom.h5m` have no graveyard;
OpenMC's `dagmc.h5m` test files are Git LFS pointers in that repo (19 and 45 bytes).
