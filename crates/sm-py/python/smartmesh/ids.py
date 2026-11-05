"""Entity ids: `Kind#serial` text, `kind << 56 | serial` raw."""

from __future__ import annotations

from functools import total_ordering
from typing import Any

from . import _native
from .errors import invalid

_SERIAL_BITS = 56


@total_ordering
class EntityId:
    """A stable entity id. Immutable, hashable, totally ordered by `raw`."""

    __slots__ = ("kind", "serial", "raw")
    kind: str
    serial: int
    raw: int

    def __init__(self, kind: str, serial: int, raw: int) -> None:
        object.__setattr__(self, "kind", kind)
        object.__setattr__(self, "serial", serial)
        object.__setattr__(self, "raw", raw)

    def __setattr__(self, name: str, value: Any) -> None:
        raise AttributeError("EntityId is immutable")

    @classmethod
    def parse(cls, text: str) -> EntityId:
        """Parse `Kind#serial`, canonical (`TopoVolume#1`) or command-language (`Volume#1`)."""
        parsed = _native.entity_parse(text) if isinstance(text, str) else None
        if parsed is None:
            raise invalid(f"not an entity id: {text!r} (expected `Kind#serial`)")
        return cls(*parsed)

    @classmethod
    def from_raw(cls, raw: int) -> EntityId:
        """Decode a raw `u64` id."""
        parsed = _native.entity_from_raw(raw) if isinstance(raw, int) else None
        if parsed is None:
            raise invalid(f"not a raw entity id: {raw!r}")
        return cls(parsed[0], parsed[1], raw)

    def __str__(self) -> str:
        return f"{self.kind}#{self.serial}"

    def __repr__(self) -> str:
        return f"EntityId({str(self)!r})"

    def __hash__(self) -> int:
        return hash(self.raw)

    def __eq__(self, other: object) -> bool:
        return isinstance(other, EntityId) and self.raw == other.raw

    def __lt__(self, other: object) -> bool:
        if not isinstance(other, EntityId):
            return NotImplemented
        return self.raw < other.raw


def coerce(value: EntityId | str, kinds: tuple[str, ...] | None = None) -> str:
    """Canonical id text of `value`; refuses an unknown spelling or (with `kinds`) a wrong kind."""
    eid = value if isinstance(value, EntityId) else EntityId.parse(value)
    if kinds is not None and eid.kind not in kinds:
        raise invalid(f"{eid} is a {eid.kind}; expected one of {list(kinds)}")
    return str(eid)


def all_kinds() -> list[str]:
    """Every entity kind name, in kind-byte order."""
    return [k for k, _ in _native.entity_kinds()]
