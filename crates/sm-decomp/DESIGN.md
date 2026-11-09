# sm-decomp

Cut planner for round cavities (W4 task HEX-DECOMP; see drafts/hexpin.md D2). It is a pure planner: given a volume that no hex scheme accepts, it returns an ordered `CutStep` list, each step holding the exact arguments of an existing op (`model.webcut_cylinder`, `model.webcut_plane`), with a cylinder plus a plane at every cavity end face. It is a library with no plugin and no registered ops, and the O-grid scheme does not consume it. This W4-0.2 skeleton is empty; the contents arrive with HEX-DECOMP.
