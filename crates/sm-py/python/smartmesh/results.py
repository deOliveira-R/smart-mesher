"""Typed, frozen results built from the protocol JSON. Unknown fields are kept in `.extra`."""

from __future__ import annotations

import base64
import builtins
from dataclasses import dataclass, field
from typing import Any, Mapping

from .ids import EntityId


def _extra(d: Mapping[str, Any], known: tuple[str, ...]) -> dict[str, Any]:
    return {k: v for k, v in d.items() if k not in known}


def _ids(xs: Any) -> list[EntityId]:
    return [EntityId.parse(x) for x in xs or []]


@dataclass(frozen=True)
class Diagnostic:
    """A non-fatal message from an op."""

    severity: str
    code: str
    message: str
    entities: list[EntityId] = field(default_factory=list)
    extra: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_json(cls, d: Mapping[str, Any]) -> Diagnostic:
        return cls(
            str(d.get("severity", "")),
            str(d.get("code", "")),
            str(d.get("message", "")),
            _ids(d.get("entities")),
            _extra(d, ("severity", "code", "message", "entities")),
        )


@dataclass(frozen=True)
class ApplyResult:
    """Result of `apply`: the new timeline node, the state hash and what the op changed."""

    node: str
    state: str
    created: list[EntityId]
    removed: list[EntityId]
    outputs: dict[str, list[EntityId]]
    diagnostics: list[Diagnostic]
    raw: dict[str, Any]
    extra: dict[str, Any] = field(default_factory=dict)

    @property
    def retired(self) -> list[EntityId]:
        """Alias of `removed`."""
        return self.removed

    @classmethod
    def from_json(cls, d: Mapping[str, Any]) -> ApplyResult:
        delta = d.get("delta") or {}
        return cls(
            node=str(d.get("node", "")),
            state=str(d.get("state", "")),
            created=_ids(delta.get("created")),
            removed=_ids(delta.get("removed")),
            outputs={k: _ids(v) for k, v in (delta.get("outputs") or {}).items()},
            diagnostics=[Diagnostic.from_json(x) for x in delta.get("diagnostics") or []],
            raw=dict(d),
            extra=_extra(d, ("node", "state", "delta")),
        )


@dataclass(frozen=True)
class ScriptResult:
    """Result of `script.run`."""

    state: str
    output: str
    events: list[dict[str, Any]]
    diagnostics: list[dict[str, Any]]
    extra: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_json(cls, d: Mapping[str, Any]) -> ScriptResult:
        diags = d.get("diagnostics")
        return cls(
            str(d.get("state", "")),
            str(d.get("output", "")),
            list(d.get("events") or []),
            list(diags) if isinstance(diags, list) else [],
            _extra(d, ("state", "output", "events", "diagnostics", "rendered", "error_code")),
        )

    @classmethod
    def empty(cls) -> ScriptResult:
        return cls("", "", [], [])


@dataclass(frozen=True)
class Status:
    """Where the timeline stands."""

    head: str | dict[str, Any]
    position: str | None
    depth: int
    state: str
    nodes: int
    branches: dict[str, str]
    redo: int
    dirty: bool
    project: str | None
    extra: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_json(cls, d: Mapping[str, Any]) -> Status:
        known = ("head", "position", "depth", "state", "nodes", "branches", "redo", "dirty", "project")
        head = d.get("head")
        return cls(
            head=head if isinstance(head, str) else dict(head or {}),
            position=d.get("position"),
            depth=int(d.get("depth", 0)),
            state=str(d.get("state", "")),
            nodes=int(d.get("nodes", 0)),
            branches=dict(d.get("branches") or {}),
            redo=int(d.get("redo", 0)),
            dirty=bool(d.get("dirty", False)),
            project=d.get("project"),
            extra=_extra(d, known),
        )


@dataclass(frozen=True)
class StateHash:
    """The state hash (`state`, hex) and the per-layer digests."""

    state: str
    layers: dict[str, str]
    extra: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_json(cls, d: Mapping[str, Any]) -> StateHash:
        return cls(str(d.get("state", "")), dict(d.get("layers") or {}), _extra(d, ("state", "layers")))


@dataclass(frozen=True)
class LogEntry:
    """One timeline node."""

    node: str
    parent: str | None
    depth: int
    op: str | None
    author: str | None
    time: str | None
    message: str | None
    extra: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_json(cls, d: Mapping[str, Any]) -> LogEntry:
        known = ("node", "parent", "depth", "op", "author", "time", "message")
        return cls(
            str(d.get("node", "")),
            d.get("parent"),
            int(d.get("depth", 0)),
            d.get("op"),
            d.get("author"),
            d.get("time"),
            d.get("message"),
            _extra(d, known),
        )


@dataclass(frozen=True)
class InputRef:
    """A staged input: the sha256 of its bytes and its size."""

    sha256: str
    bytes: int


@dataclass(frozen=True)
class ExportedFile:
    """One file of a multi-file export."""

    name: str
    bytes: int
    sha256: str


@dataclass(frozen=True)
class ExportNote:
    """What an export format could not carry."""

    code: str
    count: int
    detail: str
    severity: str = "info"


@dataclass(frozen=True)
class ExportResult:
    """Result of `session.export`; `.data` holds the bytes for an inline export."""

    path: str | None
    format: str
    bytes: int
    sha256: str
    files: list[ExportedFile]
    notes: list[ExportNote]
    data: builtins.bytes | None
    extra: dict[str, Any] = field(default_factory=dict)

    @property
    def warnings(self) -> list[ExportNote]:
        """The notes with severity `warning` (the model should be fixed until there are none)."""
        return [n for n in self.notes if n.severity == "warning"]

    @classmethod
    def from_json(cls, d: Mapping[str, Any]) -> ExportResult:
        known = ("path", "format", "bytes", "sha256", "base64", "files", "notes")
        b64 = d.get("base64")
        return cls(
            path=d.get("path"),
            format=str(d.get("format", "")),
            bytes=int(d.get("bytes", 0)),
            sha256=str(d.get("sha256", "")),
            files=[ExportedFile(str(f["name"]), int(f["bytes"]), str(f["sha256"])) for f in d.get("files") or []],
            notes=[ExportNote(str(n["code"]), int(n["count"]), str(n["detail"]), str(n.get("severity", "info"))) for n in d.get("notes") or []],
            data=base64.b64decode(b64) if isinstance(b64, str) else None,
            extra=_extra(d, known),
        )


@dataclass(frozen=True)
class MeshInfo:
    """Result of the `mesh.info` query."""

    mesh: EntityId
    selection: list[EntityId]
    stale: bool
    vertices: int
    edges: int
    faces: int
    cells: int
    cell_shapes: dict[str, int]
    topology_hash: str
    geometry_hash: str
    volume: float
    extra: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_json(cls, d: Mapping[str, Any]) -> MeshInfo:
        known = ("mesh", "selection", "stale", "vertices", "edges", "faces", "cells", "cell_shapes", "topology_hash", "geometry_hash", "volume")
        return cls(
            EntityId.parse(d["mesh"]),
            _ids(d.get("selection")),
            bool(d.get("stale", False)),
            int(d.get("vertices", 0)),
            int(d.get("edges", 0)),
            int(d.get("faces", 0)),
            int(d.get("cells", 0)),
            dict(d.get("cell_shapes") or {}),
            str(d.get("topology_hash", "")),
            str(d.get("geometry_hash", "")),
            float(d.get("volume", 0.0)),
            _extra(d, known),
        )


@dataclass(frozen=True)
class OpInfo:
    """One op of the catalog (params schema, output roles)."""

    name: str
    version: int
    schema: dict[str, Any]
    outputs: list[dict[str, Any]]
    positional: str | None
    extra: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_json(cls, d: Mapping[str, Any]) -> OpInfo:
        return cls(
            str(d["name"]),
            int(d.get("version", 1)),
            dict(d.get("schema") or {}),
            list(d.get("outputs") or []),
            d.get("positional"),
            _extra(d, ("name", "version", "schema", "outputs", "positional")),
        )


@dataclass(frozen=True)
class QueryInfo:
    """One query of the catalog."""

    name: str
    schema: dict[str, Any]
    extra: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_json(cls, d: Mapping[str, Any]) -> QueryInfo:
        return cls(str(d["name"]), dict(d.get("schema") or {}), _extra(d, ("name", "schema")))


@dataclass(frozen=True)
class Catalog:
    """The ops and queries of a session, with their JSON Schemas."""

    protocol: int
    ops: dict[str, OpInfo]
    queries: dict[str, QueryInfo]
    batch_schema: dict[str, Any]
    raw: dict[str, Any]

    @classmethod
    def from_json(cls, d: Mapping[str, Any]) -> Catalog:
        return cls(
            int(d.get("protocol", 0)),
            {o["name"]: OpInfo.from_json(o) for o in d.get("ops") or []},
            {q["name"]: QueryInfo.from_json(q) for q in d.get("queries") or []},
            dict(d.get("batch_schema") or {}),
            dict(d),
        )
