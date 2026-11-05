# DAGMC and Gmsh export conformance (W3b-EXPORT)

External readers judge the exporters, never the other way round: a fixture that fails
`smesh validate` is OUR bug and skips them. The driver hook `tools/conformance.sh --only dagmc|gmsh`
is a W3b-INT edit (the orchestrator owns `tools/conformance.sh`); this directory ships the pieces
and the stub invocation below, which the hook should run.

## Inputs

```
cargo run -p sm-meshio --example export_conformance -- "$OUT"
```

writes `two_volume.h5m` (two boxes, `mat:fuel`, `mat:water`, graveyard, with analytic boxes in
`two_volume.expect.json`), `hex_blocks.h5m` (hexahedra, so quads split to triangles), and the `.msh` files (ASCII and `_bin`) of
`single_{hex,tet,wedge,pyramid}`, `mixed` and `hex_two_volume`, each with `<case>.expect.json`.
(When `smesh export --format dagmc|gmsh` lands in W3b-EXPORT-WIRE the hook may call that instead.)

## Tier A (pip venv, always)

`h5py` and `gmsh==4.15.2` (plus the already pinned `meshio`, `numpy`) in the conformance venv;
`tools/conformance/requirements.txt` is the orchestrator's file and needs `h5py==3.16.0` and the
`gmsh==4.15.2` line enabled.

```
python tools/conformance/check_h5m.py out.json "$OUT/two_volume.h5m" \
    tools/conformance/fixtures/ref/dagmc_two_volume.h5m "$OUT/two_volume.expect.json" [--twin second_run.h5m]
python tools/conformance/check_msh4.py out.json "$OUT/mixed_bin.msh" "$OUT/mixed_bin.expect.json"
```

`check_h5m.py` opens the file with h5py (an independent HDF5 library), diffs the object tree,
dtypes, ranks and attributes against the committed MOAB reference (the subset
`crates/sm-meshio/H5M_FORMAT.md` promises), decodes the set, tag and handle tables and runs the same
invariants over the reference and our file (the reference passing fixes our sign convention), and
checks the analytic expectations. `check_msh4.py` asks Gmsh to open the file, compares counts,
physical groups, positive Jacobians and the analytic volume, re-saves, and reads it with meshio.

## Tier B (Docker, mandatory when the image is present)

```
docker build -t smesh-dagmc-conformance tools/conformance/dagmc
docker run --rm -v "$OUT:/in:ro" smesh-dagmc-conformance \
    python /opt/check_dagmc.py /in/two_volume.h5m /in/two_volume.expect.json
```

`mbconvert` (MOAB) converts the file to VTK and re-writes it as h5m; libdagmc (through
`probe.cpp`) builds the OBB tree, lists volumes, surfaces and `mat:` names, and answers
`point_in_volume` (1000 seeded points) and `ray_fire` (200 seeded rays per volume, plus the
graveyard wall) against the analytic boxes. Same skip rules as OpenFOAM: no docker or no image
is exit 3 unless `SM_CONFORMANCE_ALLOW_SKIP=1`. The conda-forge MOAB has no `pymoab`, so
`pydagmc.Model` cannot run in the image; the probe reports the same lists.
