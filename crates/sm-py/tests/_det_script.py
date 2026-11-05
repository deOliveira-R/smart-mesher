"""Subprocess helper of the determinism test: run the slice, print the state hash and table digest."""

import json

import smartmesh as sm
from helpers import slice_typed, tables_digest

s = sm.Session.memory()
state = slice_typed(s)
print(json.dumps({"state": state, "tables": tables_digest(s.mesh())}))
