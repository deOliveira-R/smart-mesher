# sm-modify

Mesh modification ops (W4 tasks MOD-HTET, MOD-REFINE-U, MOD-REFINE-L; see drafts/meshops.md): `mesh.htet` and `mesh.qtri` (hex to tet with polyhedral closure, quad to tri), uniform `mesh.refine` with `LevelMap`, and local refinement with polyhedral hanging faces. Region and tag edits are not here; they belong to REGIONS-* in sm-model/sm-boolean. The `modify` plugin is registered in `sm_server::standard()`; in this W4-0.2 skeleton it registers no op.
