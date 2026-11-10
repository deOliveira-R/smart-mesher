//! Cut planner for round cavities (W4 task HEX-DECOMP). A pure planner: given a volume that no hex scheme
//! accepts, it returns an ordered [`CutStep`] list, each step holding the exact arguments of an existing op
//! (`model.webcut_cylinder`, `model.webcut_plane`), with a cylinder plus a plane at every cavity end face.
//! A library: no plugin, no ops; the O-grid scheme does not consume it.
//!
//! Entry points: [`plan_cuts`] reads a [`TopologyView`](sm_topo_api::TopologyView); [`plan_cuts_view`] is the
//! pure core on a plain [`VolumeView`] (used by the tests and by callers that hold no topology);
//! [`render_script`] writes the steps as command-language lines.
//!
//! First version: axis-aligned (all cavities share one axis direction), disjoint round cavities in a host
//! whose other faces are planes parallel or normal to that axis, or cylinders parallel to it. Anything else is
//! refused with a reason ([`DecompError::NoDecomposition`]); the planner never returns a cut it cannot justify.
//! The design is in `DESIGN.md`.

mod plan;
mod script;
mod view;

pub use plan::{
    CutKind, CutStep, DecompError, FaceKind, FaceView, PlaneArg, Role, Target, VolumeView,
    plan_cuts_view,
};
pub use script::render_script;
pub use view::{plan_cuts, volume_view};
