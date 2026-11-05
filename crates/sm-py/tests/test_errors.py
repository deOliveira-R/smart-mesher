"""Acceptance 5: error mapping, partial scripts, Python-side validation, panic hooks."""

from __future__ import annotations

import pytest

import smartmesh as sm
from smartmesh import _native
from smartmesh.errors import SmartMeshError
from smartmesh.session import translate


def test_native_host_reports_bad_params_text_and_unknown_methods(s: sm.Session) -> None:
    host = s._t.host  # type: ignore[attr-defined]
    with pytest.raises(sm.InvalidParamsError):
        translate(host.call, "catalog", "{not json")
    with pytest.raises(sm.MethodNotFoundError):
        translate(host.call, "no.such.method", "{}")


def test_class_by_code() -> None:
    from smartmesh import errors

    cases = [
        (-32700, "parse-error", sm.ParseError),
        (-32600, "invalid-request", sm.InvalidRequestError),
        (-32601, "method-not-found", sm.MethodNotFoundError),
        (-32602, "invalid-params", sm.InvalidParamsError),
        (-32603, "internal", sm.InternalError),
        (-32000, "mesh-failed", sm.ApplicationError),
        (-32000, "script-error", sm.ScriptError),
    ]
    for code, kind, cls in cases:
        e = errors.from_native(code, kind, "m", "")
        assert type(e) is cls and e.code == code and e.kind == kind
    assert issubclass(sm.MethodNotFoundError, LookupError) and issubclass(sm.InvalidParamsError, ValueError)
    assert issubclass(sm.ScriptError, sm.ApplicationError)


def test_server_errors_reach_python_with_their_kind(s: sm.Session) -> None:
    with pytest.raises(sm.ApplicationError) as e1:
        s.apply("no.such_op", x=1)
    assert e1.value.kind == "unknown-op"
    with pytest.raises(sm.ApplicationError) as e2:
        s.ops.mesh.run(targets=["TopoVolume#7"])
    assert e2.value.code == -32000 and e2.value.kind not in ("", "internal")
    with pytest.raises(sm.ApplicationError) as e3:
        s.export(inline=True, format="smsh")
    assert e3.value.kind == "no-mesh"
    with pytest.raises(sm.ApplicationError) as e4:
        s.query("no.such_query")
    assert e4.value.kind.startswith("unknown")


def test_failing_script_statement_raises_with_the_partial_state(s: sm.Session) -> None:
    with pytest.raises(sm.ScriptError) as e:
        s.cmd("let b = model.create_brick(x: 3.0)\nmodel.create_brick(x: 4.0, bogus: 1)\nmodel.create_brick(x: 5.0)\n")
    err = e.value
    assert err.kind == "script-error" and err.error_code and err.rendered
    assert err.result.state == s.hash().state  # ops before the failure stay applied
    assert s.status().depth == 1
    assert len(err.result.events) >= 1


@pytest.mark.parametrize(
    "call",
    [
        lambda s: s.model.create_brick(x=float("nan")),
        lambda s: s.model.create_brick(x=float("inf")),
        lambda s: s.model.create_brick(x=1.0, center=[0.0, float("-inf"), 0.0]),
        lambda s: s.ops.mesh.run(targets=["Nonsense#1"]),
        lambda s: s.ops.mesh.run(targets=["TopoVolume1"]),
        lambda s: s.ops.mesh.run(targets=["TopoFace#1"]),  # wrong kind
        lambda s: s.ops.mesh.run(targets=[3]),
        lambda s: s.model.create_brick(),  # missing required
        lambda s: s.model.create_brick(x=1.0, widht=2.0),  # misspelled keyword
        lambda s: s.apply("model.create_brick", x=object()),
    ],
)
def test_python_side_validation_raises_before_any_call(s: sm.Session, call) -> None:  # type: ignore[no-untyped-def]
    s.model.create_brick(x=2.0)
    before, depth = s.hash().state, s.status().depth
    with pytest.raises(sm.InvalidParamsError):
        call(s)
    assert s.hash().state == before and s.status().depth == depth


def test_misspelled_op_attribute_is_an_attribute_error(s: sm.Session) -> None:
    with pytest.raises(AttributeError):
        s.model.create_brik  # noqa: B018
    with pytest.raises(AttributeError):
        s.nosuchfamily  # noqa: B018


def test_forced_panic_is_an_internal_error_and_the_session_stays_usable(s: sm.Session) -> None:
    if not hasattr(_native, "_panic"):
        pytest.skip("built without the test-hooks feature")
    with pytest.raises(sm.InternalError) as e:
        translate(_native._panic)
    assert e.value.kind == "internal" and "panic" in e.value.message
    assert s.model.create_brick(x=1.0).state == s.hash().state


def test_poisoned_lock_is_an_internal_error(s: sm.Session) -> None:
    host = s._t.host  # type: ignore[attr-defined]
    if not hasattr(host, "_panic_locked"):
        pytest.skip("built without the test-hooks feature")
    with pytest.raises(sm.InternalError):
        translate(host._panic_locked)
    with pytest.raises(sm.InternalError) as e:
        s.status()
    assert "poison" in e.value.message
    other = sm.Session.memory()  # a new session is unaffected
    assert other.status().depth == 0


def test_errors_are_exceptions_not_base_exceptions() -> None:
    assert issubclass(SmartMeshError, Exception)
