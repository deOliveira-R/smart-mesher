"""Exceptions: one class per JSON-RPC error code, carrying the contract `kind` string untouched."""

from __future__ import annotations

import json
from typing import TYPE_CHECKING, Any

from . import _native

if TYPE_CHECKING:
    from .results import ScriptResult

_CODES: dict[str, int] = dict(_native.protocol_codes())


class SmartMeshError(Exception):
    """Base class. `.code` is the JSON-RPC code, `.kind` the contract error string."""

    default_code: int = _CODES["INTERNAL_ERROR"]
    default_kind: str = "internal"

    def __init__(
        self,
        message: str,
        *,
        code: int | None = None,
        kind: str | None = None,
        data: Any = None,
    ) -> None:
        super().__init__(message)
        self.message = message
        self.code = self.default_code if code is None else code
        self.kind = self.default_kind if kind is None else kind
        self.data = data

    def __str__(self) -> str:
        return f"[{self.kind}] {self.message}"


class ParseError(SmartMeshError):
    """The request text was not valid JSON."""

    default_code = _CODES["PARSE_ERROR"]
    default_kind = "parse-error"


class InvalidRequestError(SmartMeshError):
    """The request was not a valid JSON-RPC request."""

    default_code = _CODES["INVALID_REQUEST"]
    default_kind = "invalid-request"


class MethodNotFoundError(SmartMeshError, LookupError):
    """Unknown protocol method, op or query."""

    default_code = _CODES["METHOD_NOT_FOUND"]
    default_kind = "method-not-found"


class InvalidParamsError(SmartMeshError, ValueError):
    """Bad parameters: raised by the Python layer before any call, or by the server."""

    default_code = _CODES["INVALID_PARAMS"]
    default_kind = "invalid-params"


class InternalError(SmartMeshError):
    """A bug or a caught panic in the native layer (never a `BaseException`)."""

    default_code = _CODES["INTERNAL_ERROR"]
    default_kind = "internal"


class ApplicationError(SmartMeshError):
    """An op, query or session command failed; `.kind` is the op's error code."""

    default_code = _CODES["APPLICATION_ERROR"]
    default_kind = "application"


class ScriptError(ApplicationError):
    """A script statement failed. Ops before it stay applied; `.result` holds the partial state."""

    default_kind = "script-error"

    def __init__(
        self,
        message: str,
        *,
        code: int | None = None,
        kind: str | None = None,
        data: Any = None,
    ) -> None:
        super().__init__(message, code=code, kind=kind, data=data)
        from .results import ScriptResult

        payload = data if isinstance(data, dict) else {}
        self.result: ScriptResult = ScriptResult.from_json(payload) if payload else ScriptResult.empty()
        self.rendered: str = str(payload.get("rendered", ""))
        self.error_code: str = str(payload.get("error_code", ""))


_CLASSES: dict[str, type[SmartMeshError]] = {
    c.__name__: c
    for c in (
        SmartMeshError,
        ParseError,
        InvalidRequestError,
        MethodNotFoundError,
        InvalidParamsError,
        InternalError,
        ApplicationError,
        ScriptError,
    )
}


def from_native(code: int, kind: str, message: str, data_json: str = "") -> SmartMeshError:
    """Build the exception for a protocol error; the class is chosen by the Rust side."""
    cls = _CLASSES[_native.error_class(code, kind)]
    data = json.loads(data_json) if data_json else None
    return cls(message, code=code, kind=kind, data=data)


def invalid(message: str) -> InvalidParamsError:
    """An `InvalidParamsError` raised by Python-side validation."""
    return InvalidParamsError(message)
