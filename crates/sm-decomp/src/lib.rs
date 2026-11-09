//! Cut planner for round cavities (W4 task HEX-DECOMP). A pure planner: given a volume that no hex scheme
//! accepts, it returns an ordered `CutStep` list, each step holding the exact arguments of an existing op
//! (`model.webcut_cylinder`, `model.webcut_plane`), with a cylinder plus a plane at every cavity end face.
//! A library: no plugin, no ops; the O-grid scheme does not consume it.
//! This skeleton (W4-0.2) holds no code yet.
