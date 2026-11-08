# W3c rulings (user, 2026-11-07)

These answer `plan.md` section "Open questions for the user" (C1-C14) and override the plan and drafts where they differ.

## Standing principle
- **Cubit is a bar to surpass, not a spec to match.** We do not copy Cubit behaviour for its own sake; we pick the best design we can find and change it later if it proves unsuitable. The same holds for legacy transport codes: their conventions are not references.

## Answers
- **C1. Enclosed pin:** all-tet is accepted for now; the through pin is all-hex. O-grid all-hex and hex-tet transitions go to W4.
- **C2. B6:** a conformal tet fill with a wedge layer at the shared face (`{Tet, Wedge}`) is accepted as the `default` outcome; the B6 wording is amended.
- **C3. Census:** any move of an existing milestone hash (W2, W3a, W3b `h_*`, `SUBMAP_GRADED`) stops the task and goes to the user; nothing is re-blessed silently. (Orchestrator default, not objected to.)
- **C4. Round-pin milestone:** a new `tools/w3c_pin.smesh` and its test; the W3b milestone and its hashes stay as they are.
- **C5. Strategies:** `hex_only` and `tet` built-in strategies wait for W4. W3c-STRATEGIES is dropped.
- **C6. CSG formats: OpenMC only. MCNP is not a target** (export-controlled, legacy baggage). W3c-CSG-MCNP is dropped, the `mcnp` module is not pre-declared in sm-csgio, and nothing in the importer design is shaped after MCNP conventions. The OpenMC subset of `drafts/csg-import.md` stands (hex lattices and periodic boundaries refused for now, DAGMC universes refused).
- **C7. Reference decks:** we write our own OpenMC models. OpenMC is installed in the conda env `openmc` (conda-forge) on this machine, so OpenMC itself is the oracle: acceptance compares sm-csgio/sm-rt cell lookups and chords against OpenMC's own (`openmc.lib.find_cell`, geometry plots) on the same `geometry.xml`. Lattice and rotation conventions are confirmed that way instead of being returned `Unsupported`.
- **C8. CSG output:** in-process plus the `smcsg` CLI in W3c; no CSG section in `.smrt`; B-rep import of a CSG model goes to W4.
- **C9. Region names:** best design, not legacy convention. A region is named by the OpenMC material `name` when it is non-empty and unique; otherwise `material_<id>`; void cells are `void`; a user override map can rename. One region per material (R9). Gaps are warnings; a clean import has no warnings (as R10).
- **C10. Export warnings:** Python raises `ExportWarning` (a `UserWarning`) and fills `ExportResult.warnings`; the CLI prints every note on stderr as `note:` / `warning:` lines, stdout unchanged; ACR-EXP2 (note severity) approved.
- **C11. `flat_golden`:** bundle-hash constants keyed per os-arch (macOS values kept), structure hashes required equal on every platform.
- **C12. Platform pins:** milestone identity hashes pinned in `tools/platform/expected.txt` and enforced on Linux CI. On a divergence: fix the near-tie decision locally first; a portable libm only if that fails; raw-bit hashes may stay per platform.
- **C13. Benchmark:** W3c does pinning, identity and Docker hygiene (DAGMC W-fire/W-flight if time allows); XDG image and tet walk go to W4; apt versions are recorded rather than frozen by a snapshot mirror.
- **C14. Face-only `mesh.run` sizing:** keep the current behaviour (volume size not inherited, with the warning); revisit if it proves unsuitable.

## Process note
GitHub CI (Linux x86_64) has been red on `main` since the W3b-1 merge (2026-11-05): `flat_golden` (bundle hash differs on Linux) and sm-py `test_errors.py [<lambda>5]`. Local `tools/ci.sh` on macOS was green. From W3c on, a wave merges only with GitHub CI green on its pull request as well as the local run.
