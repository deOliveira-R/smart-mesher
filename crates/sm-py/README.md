# smartmesh (Python bindings)

`import smartmesh`: sessions, typed ops, queries, numpy mesh tables and an independent `.smsh`
reader. Package `smartmesh`, Python >= 3.10 (abi3), numpy >= 1.23 as the only dependency. Not
published on PyPI (Q7). Design: [DESIGN.md](DESIGN.md).

## Build and test

```sh
crates/sm-py/ci/py.sh        # venv, maturin develop, stub drift check, pytest, mypy --strict,
                             # clippy (python feature), wheel check in a fresh numpy-only venv
cargo test -p sm-py          # the Rust core (Host, tables, errors); no Python needed
```

For a hand-built development install: `maturin develop --features python,test-hooks` inside a
venv (the hooks add `_native._panic` for the tests; release wheels never carry them).

## Quickstart: the ARCHITECTURE section 8 slice

```python
import smartmesh as sm

s = sm.Session.memory()                              # or Session.open("proj", create=True)

# typed ops (signatures come from the catalog; ops.pyi types them for mypy)
b = s.model.create_brick(x=10.0, y=10.0, z=10.0)     # ApplyResult
vol = b.outputs["volume"]                            # list[EntityId], pass them back unchanged
s.ops.mesh.size(targets=vol, size=1.0)
s.ops.mesh.scheme(targets=vol, scheme="map")
s.ops.mesh.run(targets=vol)
s.fem.block_add(block=1, volumes=vol)
s.fem.block_element_type(block=1, element_type="HEX8")

# the same through the command language, or the generic entry point
# s.cmd('mesh.size(targets: TopoVolume#1, size: 1.0)')
# s.apply("mesh.size", targets=vol, size=1.0)

print(s.hash().state)          # equals `smesh run` and the MCP tool path

m = s.mesh()                   # read-only numpy tables named as in .smsh
m.points.shape                 # (1331, 3)
m.cells()["hex"][1].shape      # (1000, 8)

s.export("out.smsh")           # then the zero-copy path:
f = sm.smsh.open("out.smsh")   # mmap, read-only views
f.mesh().n_cells               # 1000
```

`s.ops.<family>.<op>(...)` reaches every op; `s.model` and `s.fem` are shortcuts for those
families (`s.mesh` is the mesh *table* accessor, so the mesh ops live under `s.ops.mesh`).
Queries: `s.query("mesh.info", mesh=id)` or `s.q.mesh.info(mesh=id)`.

Timeline: `s.undo()`, `s.redo()`, `s.checkout("main" | node_hash | "empty")`,
`s.branch("alt", at=node)`, `s.log()`, `s.status()`. Atomic groups:

```python
with s.batch() as b:
    brick = b.model.create_brick(x=2.0)
    b.ops.mesh.size(targets=[brick.ref("volume", 0)], size=1.0)   # later calls reference earlier outputs
```

Async code: `await asyncio.to_thread(s.apply, "mesh.run", targets=vol)`.
Use `multiprocessing` with `spawn`, not `fork` (a rayon pool does not survive `fork`).

## Errors

All derive from `SmartMeshError` (`.code`, `.kind`, `.message`, `.data`): `ParseError`,
`InvalidRequestError`, `MethodNotFoundError` (also `LookupError`), `InvalidParamsError` (also
`ValueError`), `InternalError`, `ApplicationError`, and `ScriptError` (an `ApplicationError` with
`.result`, the partial `ScriptResult`). Bad parameters from Python (NaN or inf, unknown id
spelling, wrong id kind, misspelled keyword, missing required keyword) raise
`InvalidParamsError` before any call, so the timeline never moves.
