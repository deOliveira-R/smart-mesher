"""`Session`: the typed Python API over the JSON protocol.

The layer is pure Python on a `Transport` (`request(method, params) -> Any`); the in-process
native host is one implementation, a WebSocket client could be another.
"""

from __future__ import annotations

import base64
import difflib
import json
import os
import re
import warnings
from functools import partial
from typing import TYPE_CHECKING, Any, Callable, Literal, Mapping, Protocol, Sequence, cast

import numpy as np
import numpy.typing as npt

from . import _native, errors
from .errors import ExportWarning, InvalidParamsError, SmartMeshError, invalid
from .ids import EntityId, coerce
from .mesh import Mesh
from .results import (
    ApplyResult,
    Catalog,
    ExportResult,
    InputRef,
    LogEntry,
    MeshInfo,
    ScriptResult,
    StateHash,
    Status,
)

if TYPE_CHECKING:
    from .ops import FemOps, ModelOps, Ops, Queries

_HEX64 = re.compile(r"^[0-9a-f]{64}$")


class Transport(Protocol):
    """What the typed layer needs: one JSON request call."""

    def request(self, method: str, params: Any) -> Any:
        """Send `method` with JSON-able `params`; return the JSON result or raise."""
        ...


def translate(fn: Callable[..., Any], *args: Any) -> Any:
    """Call a native function; a `NativeError` becomes the matching `SmartMeshError` subclass."""
    try:
        return fn(*args)
    except _native.NativeError as e:
        raise errors.from_native(*e.args) from None


class ClosedTransport:
    """Stands in for the transport of a closed session (the host is dropped, releasing its lock)."""

    def request(self, method: str, params: Any) -> Any:
        raise SmartMeshError("the session is closed", kind="closed")


class NativeTransport:
    """In-process transport over the native `Host` (every call releases the GIL)."""

    def __init__(self, host: _native.Host) -> None:
        self.host = host

    def request(self, method: str, params: Any) -> Any:
        try:
            text = json.dumps(params, allow_nan=False)
        except (ValueError, TypeError) as e:
            raise invalid(f"params are not JSON-serializable: {e}") from e
        return json.loads(translate(self.host.call, method, text))

    def stage(self, data: bytes) -> InputRef:
        """Stage bytes in the session's input store."""
        sha, n = translate(self.host.stage, data)
        return InputRef(sha, n)

    def mesh_ids(self) -> list[str]:
        """Ids of the meshes in the current state."""
        ids: list[str] = translate(self.host.mesh_ids)
        return ids

    def mesh_tables(self, mesh: str | None) -> dict[str, npt.NDArray[Any]]:
        """Snapshot tables of a mesh (owned copies)."""
        tables: dict[str, npt.NDArray[Any]] = translate(self.host.mesh_tables, mesh)
        return tables


# ---------------------------------------------------------------- parameter handling


def _plain(v: Any, where: str) -> Any:
    """JSON-able copy of `v`: ids to canonical text, finite floats only, numpy scalars unwrapped."""
    if v is None or isinstance(v, (bool, str, int)):
        return v
    if isinstance(v, float):
        if v != v or v in (float("inf"), float("-inf")):
            raise invalid(f"{where}: {v} is not a finite number")
        return v
    if isinstance(v, EntityId):
        return str(v)
    if isinstance(v, np.generic):
        return _plain(v.item(), where)
    if isinstance(v, np.ndarray):
        return _plain(v.tolist(), where)
    if isinstance(v, (list, tuple)):
        return [_plain(x, f"{where}[{i}]") for i, x in enumerate(v)]
    if isinstance(v, Mapping):
        out: dict[str, Any] = {}
        for k, x in v.items():
            if not isinstance(k, str):
                raise invalid(f"{where}: object keys must be strings, not {type(k).__name__}")
            out[k] = _plain(x, f"{where}.{k}")
        return out
    if isinstance(v, os.PathLike):
        return os.fspath(v)
    raise invalid(f"{where}: cannot send a {type(v).__name__}")


def _ids_of(v: Any, kinds: tuple[str, ...], where: str, in_batch: bool) -> Any:
    if isinstance(v, (list, tuple)):
        return [_ids_of(x, kinds, f"{where}[{i}]", in_batch) for i, x in enumerate(v)]
    if v is None:
        return None
    if isinstance(v, str) and in_batch and v.startswith("$"):
        return v
    if isinstance(v, (str, EntityId)):
        try:
            return coerce(v, kinds)
        except InvalidParamsError as e:
            raise invalid(f"{where}: {e.message}") from None
    raise invalid(f"{where}: expected an entity id, not {type(v).__name__}")


def prepare_params(name: str, schema: Mapping[str, Any] | None, params: Mapping[str, Any], *, in_batch: bool = False) -> dict[str, Any]:
    """Validate `params` against the op or query schema and return the JSON-able form.

    Raises `InvalidParamsError` before any call for: an unknown keyword, a missing required
    keyword, a non-finite float, an unparsable id and an id of the wrong kind.
    """
    out: dict[str, Any] = {}
    props: Mapping[str, Any] = (schema or {}).get("properties") or {}
    if schema and schema.get("additionalProperties") is False:
        for k in params:
            if k not in props:
                near = difflib.get_close_matches(k, list(props), n=1)
                hint = f" (did you mean `{near[0]}`?)" if near else ""
                raise invalid(f"{name}: unknown parameter `{k}`{hint}; valid: {sorted(props)}")
    if schema:
        missing = [r for r in schema.get("required") or [] if r not in params]
        if missing:
            raise invalid(f"{name}: missing required parameter(s) {missing}")
    for k, v in params.items():
        kinds = (props.get(k) or {}).get("x-entity-kind")
        if kinds:
            out[k] = _ids_of(v, tuple(kinds), f"{name}.{k}", in_batch)
        else:
            out[k] = _plain(v, f"{name}.{k}")
    return out


def rev_param(rev: Any) -> Any:
    """The wire form of a revision: `"empty"`, `{"node": hash}` or `{"ref": branch}`."""
    if rev is None or isinstance(rev, dict):
        return rev
    if isinstance(rev, str):
        if rev == "empty":
            return rev
        return {"node": rev} if _HEX64.match(rev) else {"ref": rev}
    raise invalid(f"not a revision: {rev!r}")


# ---------------------------------------------------------------- namespaces


class _Namespace:
    """`s.model.create_brick(...)`: ops (or queries) of one prefix, built from the live catalog."""

    def __init__(self, call: Callable[..., Any], names: Sequence[str], prefix: str) -> None:
        self._call = call
        self._names = frozenset(names)
        self._prefix = prefix

    def __getattr__(self, name: str) -> Callable[..., Any]:
        full = f"{self._prefix}.{name}"
        if name.startswith("_") or full not in self._names:
            raise AttributeError(f"no `{full}`")
        return partial(self._call, full)

    def __dir__(self) -> list[str]:
        p = self._prefix + "."
        return sorted(n[len(p) :] for n in self._names if n.startswith(p))


class _Prefixes:
    """`s.ops.<prefix>` (and `s.q.<prefix>` for queries): one `_Namespace` per name prefix."""

    def __init__(self, call: Callable[..., Any], names: Callable[[], Sequence[str]]) -> None:
        self._call = call
        self._names = names

    def __getattr__(self, prefix: str) -> _Namespace:
        names = self._names()
        if prefix.startswith("_") or not any(n.startswith(prefix + ".") for n in names):
            raise AttributeError(f"no op or query family `{prefix}`")
        return _Namespace(self._call, names, prefix)

    def __dir__(self) -> list[str]:
        return sorted({n.split(".", 1)[0] for n in self._names()})


class BatchStep:
    """A call collected by `Session.batch()`; later calls reference its outputs with `ref`."""

    def __init__(self, index: int, op: str) -> None:
        self.index = index
        self.op = op

    def ref(self, role: str, item: int | Literal["*"] | None = None) -> str:
        """Batch reference to an output role: `$i.role`, `$i.role[k]` or `$i.role[*]`."""
        suffix = "" if item is None else f"[{item}]"
        return f"${self.index}.{role}{suffix}"


class Batch:
    """Collects typed calls and applies them as one `core.batch` node on exit."""

    def __init__(self, session: Session) -> None:
        self._session = session
        self._ops: list[dict[str, Any]] = []
        self.result: ApplyResult | None = None
        self.ops = _Prefixes(self._collect, lambda: sorted(session.catalog().ops))

    def _collect(self, op: str, /, **params: Any) -> BatchStep:
        cat = self._session.catalog()
        info = cat.ops.get(op)
        if info is None:
            raise errors.MethodNotFoundError(f"unknown op `{op}`", kind="method-not-found")
        self._ops.append({"name": op, "version": info.version, "params": prepare_params(op, info.schema, params, in_batch=True)})
        return BatchStep(len(self._ops) - 1, op)

    def apply(self, op: str, /, **params: Any) -> BatchStep:
        """Collect `op` (same as `s.apply`, but deferred)."""
        return self._collect(op, **params)

    if not TYPE_CHECKING:

        def __getattr__(self, name: str) -> Any:
            if name.startswith("_"):
                raise AttributeError(name)
            return getattr(self.ops, name)

    def __enter__(self) -> Batch:
        return self

    def __exit__(self, et: Any, ev: Any, tb: Any) -> None:
        if et is None and self._ops:
            self.result = self._session.apply("core.batch", ops=self._ops)


# ---------------------------------------------------------------- the session


class Session:
    """A smart-mesh session (an in-memory or project timeline) with a typed API.

    Thread-safe: calls from several threads serialize on the host's lock and release the GIL
    while waiting. Use `asyncio.to_thread(s.apply, ...)` from async code.
    """

    if TYPE_CHECKING:
        model: ModelOps
        fem: FemOps
        ops: Ops
        q: Queries

    def __init__(self, transport: Transport) -> None:
        self._t = transport
        self._catalog: Catalog | None = None
        self._closed = False
        self.ops = cast("Ops", _Prefixes(self._apply_kw, lambda: sorted(self.catalog().ops)))
        self.q = cast("Queries", _Prefixes(self._query_kw, lambda: sorted(self.catalog().queries)))

    # -- construction

    @classmethod
    def memory(cls, *, author: str | None = None, fixed_time: str | None = None) -> Session:
        """An in-memory session; `author` and `fixed_time` (RFC 3339) make node hashes reproducible."""
        return cls(NativeTransport(translate(_native.Host.memory, author, fixed_time)))

    @classmethod
    def open(cls, path: str | os.PathLike[str], create: bool = False) -> Session:
        """Open the project directory `path`; with `create`, make it (named after the directory)."""
        p = os.fspath(path)
        return cls.create(p, os.path.basename(os.path.abspath(p))) if create else cls._open(p, None)

    @classmethod
    def create(cls, path: str | os.PathLike[str], name: str) -> Session:
        """Create a project named `name` in the new or empty directory `path`."""
        return cls._open(os.fspath(path), name)

    @classmethod
    def _open(cls, path: str, create: str | None) -> Session:
        return cls(NativeTransport(translate(_native.Host.open, path, create)))

    def close(self) -> None:
        """Save a project with pending changes and release the host. Idempotent."""
        if self._closed:
            return
        try:
            st = self.status()
            if st.project is not None and st.dirty:
                self.save()
        finally:
            self._closed = True
            self._t = ClosedTransport()

    def __enter__(self) -> Session:
        return self

    def __exit__(self, et: Any, ev: Any, tb: Any) -> None:
        self.close()

    # -- plumbing

    def _request(self, method: str, params: Any) -> Any:
        if self._closed:
            raise SmartMeshError("the session is closed", kind="closed")
        return self._t.request(method, params)

    def catalog(self, refresh: bool = False) -> Catalog:
        """The ops and queries with their JSON Schemas (cached after the first call)."""
        if self._catalog is None or refresh:
            self._catalog = Catalog.from_json(self._request("catalog", {}))
        return self._catalog

    # -- ops and queries

    def apply(self, op: str, /, **params: Any) -> ApplyResult:
        """Apply one op (keyword params) and return the new node, state hash and delta."""
        info = self.catalog().ops.get(op)
        wire = prepare_params(op, info.schema if info else None, params)
        return ApplyResult.from_json(self._request("apply", {"op": op, "params": wire}))

    def _apply_kw(self, op: str, /, **params: Any) -> ApplyResult:
        return self.apply(op, **params)

    def query(self, name: str, /, *, at: Any = None, **input: Any) -> Any:
        """Run a read-only query (optionally at a revision) and return its JSON value."""
        info = self.catalog().queries.get(name)
        req: dict[str, Any] = {"name": name, "input": prepare_params(name, info.schema if info else None, input)}
        if at is not None:
            req["at"] = rev_param(at)
        return self._request("query", req)

    def _query_kw(self, name: str, /, **input: Any) -> Any:
        return self.query(name, **input)

    if not TYPE_CHECKING:

        def __getattr__(self, name: str) -> Any:
            if name.startswith("_") or name in ("ops", "q"):
                raise AttributeError(name)
            return getattr(self.ops, name)

    def batch(self) -> Batch:
        """Context manager collecting typed calls into one atomic `core.batch` node."""
        return Batch(self)

    # -- scripts

    def cmd(self, text: str) -> ScriptResult:
        """Run command-language text; a failing statement raises `ScriptError` (partial state kept)."""
        return self.run_script(text)

    def run_script(self, text: str, *, path: str | None = None, includes: Mapping[str, str] | None = None) -> ScriptResult:
        """Run a script with an optional path (for diagnostics) and in-memory includes."""
        params: dict[str, Any] = {"text": text}
        if path is not None:
            params["path"] = path
        if includes:
            params["includes"] = dict(includes)
        return ScriptResult.from_json(self._request("script.run", params))

    # -- timeline

    def status(self) -> Status:
        """Where the timeline stands."""
        return Status.from_json(self._request("timeline.status", {}))

    def hash(self, at: Any = None) -> StateHash:
        """The state hash (and per-layer digests), at a revision if given."""
        return StateHash.from_json(self._request("session.hash", {} if at is None else {"at": rev_param(at)}))

    def undo(self) -> Status:
        """Step back one node."""
        return Status.from_json(self._request("timeline.undo", {}))

    def redo(self) -> Status:
        """Step forward one node."""
        return Status.from_json(self._request("timeline.redo", {}))

    def checkout(self, rev: Any) -> Status:
        """Move HEAD to a branch name, a node hash (64 hex) or `"empty"`."""
        return Status.from_json(self._request("timeline.checkout", {"rev": rev_param(rev)}))

    def branch(self, name: str, at: str | None = None) -> Status:
        """Create a branch (at the current node or at `at`) and switch to it."""
        params: dict[str, Any] = {"name": name}
        if at is not None:
            params["at"] = at
        return Status.from_json(self._request("timeline.branch", params))

    def log(self, limit: int = 100, from_: str | None = None) -> list[LogEntry]:
        """Timeline nodes, newest first."""
        params: dict[str, Any] = {"limit": limit}
        if from_ is not None:
            params["from"] = from_
        return [LogEntry.from_json(e) for e in self._request("timeline.log", params)]

    def save(self) -> Status:
        """Persist pending state (a project session)."""
        return Status.from_json(self._request("session.save", {}))

    # -- inputs, export, meshes

    def _native(self) -> NativeTransport:
        if self._closed:
            raise SmartMeshError("the session is closed", kind="closed")
        if not isinstance(self._t, NativeTransport):
            raise NotImplementedError("this transport has no in-process host")
        return self._t

    def stage_file(self, path: str | os.PathLike[str]) -> InputRef:
        """Stage a file's bytes in the session's content-addressed input store."""
        with open(path, "rb") as f:
            return self._native().stage(f.read())

    def import_step(self, path: str | os.PathLike[str], **kw: Any) -> ApplyResult:
        """Stage a STEP file, then apply `model.import_step` with `{sha256}` as its source."""
        ref = self.stage_file(path)
        return self.apply("model.import_step", source={"sha256": ref.sha256}, **kw)

    def import_step_inline(self, path: str | os.PathLike[str], **kw: Any) -> ApplyResult:
        """`model.import_step` with the bytes carried inside the op (`{inline}`)."""
        with open(path, "rb") as f:
            b64 = base64.b64encode(f.read()).decode("ascii")
        return self.apply("model.import_step", source={"inline": b64}, **kw)

    def export(
        self,
        path: str | os.PathLike[str] | None = None,
        *,
        format: str | None = None,
        mesh: EntityId | str | None = None,
        provenance: bool = False,
        inline: bool = False,
        options: Mapping[str, Any] | None = None,
        graveyard: bool | Mapping[str, float] | None = None,
        binary: bool | None = None,
        polyhedra: Literal["error", "decompose", "drop"] | None = None,
        split_small_determinant: bool | None = None,
        split_warped_faces: bool = False,
        empty_patches: Sequence[str] = (),
        wedge_patches: Sequence[str] = (),
    ) -> ExportResult:
        """Export a mesh to `path` (format inferred from the extension) or inline (`.data`).

        `options` is the per-format option object (`gmsh {binary, polyhedra}`, `dagmc {graveyard}`).
        `graveyard`, `binary` and `polyhedra` are sugar merged into it; naming a key in both places
        raises `ValueError` before any request. The OpenFOAM switches `split_small_determinant`,
        `split_warped_faces`, `empty_patches` and `wedge_patches` are top-level. The server validates
        per-format options and rejects options the format does not take (`SmartMeshError`).
        Every note with severity `warning` is also raised as an `ExportWarning`.
        """
        merged: dict[str, Any] = dict(options) if options is not None else {}
        for key, val in (("graveyard", graveyard), ("binary", binary), ("polyhedra", polyhedra)):
            if val is None:
                continue
            if key in merged:
                raise ValueError(f"export option `{key}` is given both in `options` and as a keyword")
            merged[key] = val
        params: dict[str, Any] = {}
        if path is not None:
            params["path"] = os.fspath(path)
        if format is not None:
            params["format"] = format
        if mesh is not None:
            params["mesh"] = coerce(mesh, ("Mesh",))
        if provenance:
            params["provenance"] = True
        if inline:
            params["inline"] = True
        if merged:
            params["options"] = merged
        if split_small_determinant is not None:
            params["split_small_determinant"] = split_small_determinant
        if split_warped_faces:
            params["split_warped_faces"] = True
        if empty_patches:
            params["empty_patches"] = list(empty_patches)
        if wedge_patches:
            params["wedge_patches"] = list(wedge_patches)
        result = ExportResult.from_json(self._request("session.export", params))
        for note in result.warnings:
            warnings.warn(f"{note.code}: {note.detail}", ExportWarning, stacklevel=2)
        return result

    def meshes(self) -> list[MeshInfo]:
        """Summaries of the meshes in the current state (the `mesh.info` query)."""
        return [MeshInfo.from_json(self._request("query", {"name": "mesh.info", "input": {"mesh": m}})) for m in self._native().mesh_ids()]

    def mesh(self, id: EntityId | str | None = None) -> Mesh:
        """Snapshot of a mesh's tables (the only mesh when `id` is None). Later ops never change it."""
        mid = None if id is None else coerce(id, ("Mesh",))
        tables = self._native().mesh_tables(mid)
        return Mesh(tables, mesh_id=None if mid is None else EntityId.parse(mid), source="session")
