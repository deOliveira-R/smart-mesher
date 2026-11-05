"""smart-mesh Python bindings: sessions, typed ops, mesh tables and the `.smsh` reader.

>>> import smartmesh as sm
>>> s = sm.Session.memory()
>>> r = s.model.create_brick(x=10.0)
>>> s.hash().state == r.state
True
"""

from . import _native, smsh
from .errors import (
    ApplicationError,
    InternalError,
    InvalidParamsError,
    InvalidRequestError,
    MethodNotFoundError,
    ParseError,
    ScriptError,
    SmartMeshError,
)
from .ids import EntityId
from .mesh import Mesh
from .results import (
    ApplyResult,
    Catalog,
    Diagnostic,
    ExportResult,
    InputRef,
    LogEntry,
    MeshInfo,
    ScriptResult,
    StateHash,
    Status,
)
from .session import Batch, BatchStep, Session

__version__: str = _native.native_version()

__all__ = [
    "ApplicationError",
    "ApplyResult",
    "Batch",
    "BatchStep",
    "Catalog",
    "Diagnostic",
    "EntityId",
    "ExportResult",
    "InputRef",
    "InternalError",
    "InvalidParamsError",
    "InvalidRequestError",
    "LogEntry",
    "Mesh",
    "MeshInfo",
    "MethodNotFoundError",
    "ParseError",
    "ScriptError",
    "ScriptResult",
    "Session",
    "SmartMeshError",
    "StateHash",
    "Status",
    "smsh",
]
