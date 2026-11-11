# W4-REFCORPUS: reference models from public sources

Ruling D8: the user's work models cannot be shared, so the evidence that real models will work comes from public ones. This corpus is built by scripts in `tools/refcorpus/`; the scripts, the manifest (`tools/refcorpus/manifest.toml`) and the sha256 values are committed, the models are not (no committed file is over 100 kB).

## Rebuild

```sh
python3 -I tools/refcorpus/build.py                  # fetch (network), then build everything
python3 -I tools/refcorpus/build.py fetch            # the only step that uses the network
python3 -I tools/refcorpus/build.py build --offline  # build from the cache, no network
python3 -I tools/refcorpus/build.py --check          # recompute every sha256, exit 1 and name the file on a mismatch
python3 -I tools/refcorpus/build.py selftest         # corrupt a copy, require --check to fail naming it
python3 -I tools/refcorpus/build.py expected --smcsg target/debug/smcsg   # compare `expected` of every OpenMC deck with sm-csgio today
python3 -I tools/refcorpus/build.py bless [--only CAT|ID]   # maintainers: build twice, record sha256 only if both agree
```

- Output: `${SM_REFCORPUS_DIR:-$HOME/.cache/smart-mesh/refcorpus}`, one directory per entry id, plus `.cache/` (the pinned downloads). `build.py` refuses an output directory inside the repository.
- Tools: OpenMC 0.16.0 in a conda env (`SM_OPENMC_PYTHON`, default `~/opt/miniforge/envs/openmc/bin/python`), FreeCAD (`SM_FREECADCMD`, default the macOS app bundle, else `freecadcmd` on PATH), `smcsg` built with `cargo build -p sm-csgio --bin smcsg` (`--smcsg`). `build.py` itself is standard library only and runs under `python3 -I`.
- `expected` with a debug `smcsg` takes about three minutes (the BEAVRS core slice is flattened to the cell cap before it is refused, and the HTGR deck is 3.6 MB); `build`, `--check` and `selftest` take seconds to a minute. A full rebuild from an empty directory, network included, took 45 s on the Mac.
- Tests that need no external tool: `python3 -I -B -m unittest discover -s tools/refcorpus -p 'test_*.py'`. Lint: `ruff check` and `mypy` with the configs in `tools/refcorpus/`.
- Reproducibility: OpenMC decks call `openmc.reset_auto_ids()` and create objects in a fixed order (no cross-section path, no timestamps). STEP headers (`FILE_NAME` time, author, system) are rewritten to fixed strings in the file. `bless` builds every entry twice from a wiped entry directory and refuses to record a sha256 when the two differ. The sha256 of the generated entries is valid for the tool versions in `[meta]` (OpenMC 0.16.0, FreeCAD 1.1.4 revision 20260928); another version may produce other bytes, and `--check` then names the file. The downloaded entries (VTB) are byte copies of files at a pinned commit, so they are the same everywhere.
- Models are generated deterministically, so two machines with the same tool versions get the same sha256. The values were recorded on macOS arm64.

## Sources

| Source | Licence | Pin | Entries |
|---|---|---|---|
| smart-mesh (our scripts `gen_openmc.py`, `gen_step.py`; OpenMC 0.16.0, FreeCAD 1.1.4) | MIT (this repository) | the commit that contains the scripts | `openmc-*`, `step-*` |
| BEAVRS, `github.com/mit-crpg/BEAVRS` (MIT CRPG public benchmark; its OpenMC model builder under `models/openmc/`) | MIT (LICENSE of that repository) | `1ccae04dc3830a5977f08ed7f5f7e1e51ded0d22`; the 11 builder files are pinned by sha256 in `[[support]]` | `beavrs-*` |
| INL Virtual Test Bed, `github.com/idaholab/virtual_test_bed` | CC-BY-4.0 (repository licence) | `12c7e6dc46ed1fd9c7a5a1defc162eea028a4f37`; each file pinned by sha256 | `vtb-*` |
| MOOSE reactor module (optional) | LGPL-2.1 (MOOSE) | the VTB commit above (input), the executable from `SM_MOOSE_EXE` | `moose-sefor-mesh` |

Every OpenMC deck and STEP file is written by our scripts; the BEAVRS decks are produced by running the pinned BEAVRS builder (`gen_beavrs.py` follows the repository's own `extract-pin.ipynb`, `extract-assm.ipynb` and `make_beavrs.py --2d`). VTB files are copied verbatim and are not redistributed: only their path, commit and sha256 are committed.

## Entries

`expected` for an OpenMC deck is what `sm-csgio` does today, measured with `smcsg info --json` (`build.py expected`): `accepted`, or `<error variant>:<what>` of the first refusal in file order, without line numbers or ids (`unsupported:hex_lattice`). For dirty STEP, `expected` holds the `model.heal_check` codes of `drafts/vtopo.md` §T3 (`expected_basis` says where each comes from); for clean STEP it is empty (no findings).

| id | what | expected today | feeds |
|---|---|---|---|
| `openmc-pin` | PWR pin cell, reflective | accepted | CSG-ORACLE deck, CSG-CORE pin path |
| `openmc-assembly-17x17` | 17x17, 24 guide tubes + instrument tube, two universes | accepted | CSG-ORACLE 17x17 deck, RT-ROWS `reduced` case (deck sha256 from the manifest), POLY-MEASURE, REGIONS tags (D9) |
| `openmc-mini-3x3` | 3x3, eight pins + one guide tube | accepted | CSG-CORE lattice tags test, CSG-ORACLE (lattice index equals region tag, D9) |
| `openmc-pin-layered` | axial layers (plugs, fuel, plenum) by z planes | accepted | CSG-CORE, POLY-MEASURE |
| `openmc-hex-y-7ring` | hex lattice, orientation `y`, 7 rings, one universe | `unsupported:hex_lattice` | CSG-HEX (confirmed against OpenMC, C7), RT hex orientation |
| `openmc-hex-x-7ring` | hex lattice, orientation `x`, 7 rings, one universe | `unsupported:hex_lattice` | CSG-HEX, RT second orientation |
| `openmc-hex-y-mixed` | hex `y`, 7 rings, pins plus 7 guide tubes | `unsupported:hex_lattice` | CSG-HEX (mixed universes) |
| `openmc-triso-lattice` | five-layer sphere in a 4x4x4 regular lattice | accepted | BOOL-TOOLS (spheres), CSG-CORE `Unsupported` until BOOL-TOOLS |
| `openmc-cone-torus` | cone frustum, torus, sphere in a box | accepted | BOOL-TOOLS (cones and tori) |
| `beavrs-pin` | BEAVRS pin cell (1.6% fuel rod, active region) | accepted | CSG-ORACLE BEAVRS deck, benchmark |
| `beavrs-assembly` | BEAVRS 17x17 assembly 'Fuel 1.6% enr instr no BAs', full height | accepted (12043 flat cells) | CSG-ORACLE BEAVRS assembly, benchmark `reduced`, POLY-MEASURE |
| `beavrs-core-2d` | BEAVRS 2D core slice | `error:flat-model-has-more-than` (more than 2097152 flat cells) | CSG-CORE must keep lattices implicit; core-scale benchmark |
| `vtb-htgr-assembly` | VTB `htgr/assembly/{geometry,materials}.xml`: hex lattice (orientation `x`, 12 rings, 50 axial) | `unsupported:material-list` (a distributed-material cell, before the hex lattice) | CSG-HEX VTB deck (imports or fails naming the feature), CSG-CORE typed refusals |
| `vtb-startr-openmc` | VTB `microreactors/STARTR/OpenMC/{geometry,materials}.xml`: hex lattices | `unsupported:hex_lattice` | CSG-HEX VTB deck, ORACLE hex decks |
| `vtb-sefor-core-mesh` | VTB `sfr/sefor/Mesh/Mesh_Core-I-E_3D_450K.i`: MOOSE reactor-module hex patterns (SFR) | n/a (not an OpenMC deck) | reference for hex pitch, duct and pin layouts; input of the optional MOOSE entry |
| `step-clean-pin`, `-cladded-pin`, `-assembly-can`, `-grid-spacer` | clean STEP, 1 to 2 solids | none (no findings) | BOOL-HEAL-A (no false positives), STEP import in CADIO, POLY-MEASURE (tet meshes) |
| `step-dirty-notch` | unit brick minus a `1e-4` corner cube | `small_edge`, `small_face` | BOOL-HEAL-A thresholds (T3 acceptance 1) |
| `step-dirty-step` | two bricks offset `1e-4`, fused | `small_edge`, `sliver_face` | thresholds (T3 acceptance 2) |
| `step-dirty-gap` | two bodies `1e-5` apart | `gap_near` | thresholds (T3 acceptance 4), HEAL-TOL |
| `step-dirty-open-shell` | brick shell, one face removed | `open_shell` | thresholds, STEP-REPAIR (do open shells block import?) |
| `step-dirty-small-hole` | plate with a through hole of radius `1e-3` | `small_hole` | thresholds (T3 acceptance 3) |
| `step-dirty-fillet` | brick with one edge rounded, `r = 0.1` | `blend_candidate` | thresholds (T3 acceptance 6), VTOPO T6 trigger |
| `step-dirty-tol-edge` | unsewn brick faces, one face displaced `5e-5` | `gap_near`, `open_shell` (a design estimate: T3 has no fixture for this defect; BOOL-HEAL-A confirms or corrects it and updates the manifest) | HEAL-TOL |
| `moose-sefor-mesh` | optional: Exodus mesh from the SEFOR input with `--mesh-only` | not built without `SM_MOOSE_EXE` | reference meshes per os-arch (none on the Mac today: the `moose` env has no executable) |

Units of the STEP files are model units (the unit brick of the T3 fixtures); the OpenMC decks are in cm.

## What the corpus told us (for the W4a2 workers)

- Hex lattices are in all three public sources (OpenMC `orientation="x"` in the VTB HTGR assembly, VTB STARTR, our two orientations). Today's reader refuses `<hex_lattice>` at the first one it meets, so every hex deck's `expected` is `unsupported:hex_lattice`, except the HTGR assembly, which hits a distributed-material cell (`material="10 11 ..."`, one material per cell is the importer's rule) first. CSG-HEX should expect both: the HTGR deck needs the hex lattice and the material list.
- `beavrs-core-2d` flattens to more than the 2097152-cell cap of `smcsg`: any core-scale path must stay lattice-aware (instancing), not flatten.
- OpenMC hex-lattice orientation: the generator asserts with `Geometry.find`, for both orientations, that the leaf cell at the corner is `fuel pin inner`, that a point 7p along the vertex axis is inside the bounding `HexagonalPrism` of the same orientation, and that a point 7p along the flat axis is outside it, so the decks are consistent in OpenMC's own lookup.
- Hashes of the STEP files depend on the FreeCAD/OCC version; recompute with `bless` after upgrading FreeCAD and review the diff.
