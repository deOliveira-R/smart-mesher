//! The ops and queries of the `boolean` plugin: `model.unite`, `model.subtract`,
//! `model.intersect`, `model.imprint`, `model.merge`, `model.merge_check` and the webcut family.
//!
//! Every op runs its library function inside one `topo_txn` and reports the surviving and new
//! cells under its declared output roles through `commit_topology`. A failing function rolls
//! the transaction back, so the state hash is unchanged.

use std::collections::BTreeSet;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_geom_api::{Frame, SurfaceDef};
use sm_layer_api::{MeshLayer, ModelLayer, commit_topology, topo_txn};
use sm_model_api::FaceRole;
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, QueryCtx, State, TypedOp, TypedQuery};
use sm_topo_api::{Carrier, Dim, TopologyView};

use crate::boolean::{BoolOptions, BoolRule, boolean_ordered};
use crate::error::BooleanError;
use crate::imprint::{ImprintOptions, imprint_bodies};
use crate::merge::{MergeKinds, MergeOptions, MergeReport, merge, merge_check};
use crate::webcut::{CutTool, KeepSide, WebcutOptions, webcut};

const BOOL_OUTPUTS: &[OutputRole] = &[
    OutputRole::new("bodies", &[EntityKind::TopoBody], Cardinality::Many, "The result bodies, one per result volume."),
    OutputRole::new("volumes", &[EntityKind::TopoVolume], Cardinality::Many, "The result volumes."),
    OutputRole::new("new_faces", &[EntityKind::TopoFace], Cardinality::Many, "Faces on the intersection surfaces, created by the operation."),
    OutputRole::new("faces", &[EntityKind::TopoFace], Cardinality::Many, "The remaining faces of the result volumes."),
];

const WEBCUT_OUTPUTS: &[OutputRole] = &[
    OutputRole::new("positive", &[EntityKind::TopoBody], Cardinality::Many, "Bodies on the side the tool normal points to."),
    OutputRole::new("negative", &[EntityKind::TopoBody], Cardinality::Many, "Bodies on the opposite side."),
    OutputRole::new("volumes", &[EntityKind::TopoVolume], Cardinality::Many, "The volumes of every piece."),
    OutputRole::new("cut_faces", &[EntityKind::TopoFace], Cardinality::Many, "The faces generated on the tool surface."),
    OutputRole::new("bodies", &[EntityKind::TopoBody], Cardinality::Many, "Pieces on neither side (empty for plane, surface and cylinder tools)."),
];

/// Refuse operands that have a mesh, are decomposed or are instanced.
fn guard_volumes(ctx: &ApplyCtx<'_>, volumes: &[EntityId]) -> Result<(), OpError> {
    let model_layer = ctx.layer::<ModelLayer>()?;
    let topo = model_layer.topo();
    let model = model_layer.model();
    let decomposed = model.decomposition().decomposed_volumes();
    let mesh = ctx.state().layer::<MeshLayer>();
    for &v in volumes {
        if decomposed.contains(&v) {
            return Err(OpError::Unsupported(format!("{v} is decomposed: undo the decomposition first")));
        }
        if model.instancing().prototypes.values().any(|p| p.volumes.contains(&v)) {
            return Err(OpError::Unsupported(format!("{v} is a prototype volume of an instance: instanced volumes cannot be edited")));
        }
        if let Some(m) = mesh {
            let mut cells = vec![v];
            cells.extend(topo.volume_faces(v).iter().copied());
            if let Some(mid) = cells.iter().find_map(|c| m.mesh_of(*c)) {
                return Err(OpError::Invalid(format!("{v} has a mesh ({mid}): mesh.delete first")));
            }
        }
    }
    Ok(())
}

fn volumes_of_bodies(ctx: &ApplyCtx<'_>, bodies: &[EntityId]) -> Result<Vec<EntityId>, OpError> {
    let topo = ctx.layer::<ModelLayer>()?.topo();
    crate::bodies::volumes_of(topo, bodies).map_err(OpError::from)
}

fn resolve_all(ctx: &ApplyCtx<'_>, ids: &[EntityId], kind: EntityKind) -> Result<(), OpError> {
    for &id in ids {
        ctx.resolve_kind(id, kind)?;
    }
    Ok(())
}

fn bool_options(keep: Option<bool>, regularize: Option<bool>, tol: Option<f64>) -> BoolOptions {
    BoolOptions { keep: keep.unwrap_or(false), regularize: regularize.unwrap_or(true), merge: true, max_gap: tol }
}

fn run_boolean(ctx: &mut ApplyCtx<'_>, rule: BoolRule, a: &[EntityId], b: &[EntityId], opts: BoolOptions) -> Result<(), OpError> {
    resolve_all(ctx, a, EntityKind::TopoBody)?;
    resolve_all(ctx, b, EntityKind::TopoBody)?;
    let mut all: Vec<EntityId> = a.to_vec();
    all.extend_from_slice(b);
    let vols = volumes_of_bodies(ctx, &all)?;
    guard_volumes(ctx, &vols)?;
    if let Some(t) = opts.max_gap {
        let max = ctx.layer::<ModelLayer>()?.tolerance().linear_max;
        if !(t.is_finite() && t >= 0.0 && t <= max) {
            return Err(OpError::Invalid(format!("`tol` {t:e} is outside [0, linear_max = {max:e}]")));
        }
    }
    // A tangent contact met after the arrangement started cannot be undone inside the kernel, but
    // the whole transaction can: roll back and retry with the offending operand fused last (a
    // bridge operand may cover the tangent line). Each retry defers one more operand, so at most
    // one retry per operand.
    let mut defer: Vec<usize> = Vec::new();
    let mut first_err: Option<OpError> = None;
    let (out, txn) = loop {
        let mut culprit: Option<usize> = None;
        let r = topo_txn(ctx, |topo, _model| boolean_ordered(topo, rule, a, b, &opts, &defer, &mut culprit).map_err(OpError::from));
        match (r, culprit) {
            (Err(e), Some(c)) if rule != BoolRule::Subtract && !defer.contains(&c) => {
                first_err.get_or_insert(e);
                defer.push(c);
            }
            // The first tangent contact is the informative one when the reordered run fails too.
            (Err(_), _) if first_err.is_some() => return Err(first_err.take().unwrap_or_else(|| OpError::Failed("boolean failed".into()))),
            (r, _) => break r?,
        }
    };
    commit_topology(ctx, &txn, &[("bodies", &out.bodies), ("volumes", &out.volumes), ("new_faces", &out.new_faces), ("faces", &out.faces)])
}

/// `model.unite {bodies, keep?, regularize?, tol?}`: the union of two or more bodies.
///
/// Bodies that neither overlap nor share a face give `Invalid("bodies do not touch")`.
/// `regularize` (default true) merges coplanar and co-surface fragments, `keep` leaves the inputs
/// unchanged (the kernel works on deep copies that share curve and surface handles with them),
/// `tol` is the weld tolerance (at most the model's `linear_max`).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Unite {
    /// The bodies to unite (at least two).
    #[schemars(extend("x-entity-kind" = ["TopoBody"]))]
    pub bodies: Vec<EntityId>,
    /// Keep the inputs (default false).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub keep: Option<bool>,
    /// Merge coplanar fragments of the result (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub regularize: Option<bool>,
    /// Merge tolerance.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tol: Option<f64>,
}

impl TypedOp for Unite {
    const NAME: &'static str = "model.unite";
    const OUTPUTS: &'static [OutputRole] = BOOL_OUTPUTS;
    const POSITIONAL: Option<&'static str> = Some("bodies");

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if self.bodies.len() < 2 {
            return Err(OpError::Invalid("`bodies` needs at least two bodies".into()));
        }
        run_boolean(ctx, BoolRule::Unite, &self.bodies[..1], &self.bodies[1..], bool_options(self.keep, self.regularize, self.tol))
    }
}

/// `model.subtract {targets, tools, keep?, regularize?, tol?}`: remove every tool from every
/// target. The tools are consumed unless `keep` is set.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Subtract {
    /// The bodies to cut.
    #[schemars(extend("x-entity-kind" = ["TopoBody"]))]
    pub targets: Vec<EntityId>,
    /// The bodies to remove.
    #[schemars(extend("x-entity-kind" = ["TopoBody"]))]
    pub tools: Vec<EntityId>,
    /// Keep the inputs (default false).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub keep: Option<bool>,
    /// Merge coplanar fragments of the result (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub regularize: Option<bool>,
    /// Merge tolerance.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tol: Option<f64>,
}

impl TypedOp for Subtract {
    const NAME: &'static str = "model.subtract";
    const OUTPUTS: &'static [OutputRole] = BOOL_OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if self.targets.is_empty() || self.tools.is_empty() {
            return Err(OpError::Invalid("`targets` and `tools` must not be empty".into()));
        }
        run_boolean(ctx, BoolRule::Subtract, &self.targets, &self.tools, bool_options(self.keep, self.regularize, self.tol))
    }
}

/// `model.intersect {bodies, keep?, regularize?, tol?}`: the common part of two or more bodies.
/// An empty result is an error (`EmptyResult`) and leaves the state unchanged.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Intersect {
    /// The bodies to intersect (at least two).
    #[schemars(extend("x-entity-kind" = ["TopoBody"]))]
    pub bodies: Vec<EntityId>,
    /// Keep the inputs (default false).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub keep: Option<bool>,
    /// Merge coplanar fragments of the result (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub regularize: Option<bool>,
    /// Merge tolerance.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tol: Option<f64>,
}

impl TypedOp for Intersect {
    const NAME: &'static str = "model.intersect";
    const OUTPUTS: &'static [OutputRole] = BOOL_OUTPUTS;
    const POSITIONAL: Option<&'static str> = Some("bodies");

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if self.bodies.len() < 2 {
            return Err(OpError::Invalid("`bodies` needs at least two bodies".into()));
        }
        run_boolean(ctx, BoolRule::Intersect, &self.bodies[..1], &self.bodies[1..], bool_options(self.keep, self.regularize, self.tol))
    }
}

/// `model.imprint {bodies, merge?, tol?}`: imprint the bodies on each other. Every overlapping or
/// intersecting face pair is split along the intersection; the cut edges are created once per
/// body. With `merge` the coincident cells are merged afterwards.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Imprint {
    /// The bodies to imprint (at least two).
    #[schemars(extend("x-entity-kind" = ["TopoBody"]))]
    pub bodies: Vec<EntityId>,
    /// Merge the coincident cells afterwards (default false).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub merge: Option<bool>,
    /// Merge tolerance (with `merge`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tol: Option<f64>,
}

impl TypedOp for Imprint {
    const NAME: &'static str = "model.imprint";
    const OUTPUTS: &'static [OutputRole] = &[
        OutputRole::new("faces", &[EntityKind::TopoFace], Cardinality::Many, "The faces the imprint produced (the children of split faces)."),
        OutputRole::new("edges", &[EntityKind::TopoEdge], Cardinality::Many, "The new imprint edges."),
    ];
    const POSITIONAL: Option<&'static str> = Some("bodies");

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if self.bodies.len() < 2 {
            return Err(OpError::Invalid("`bodies` needs at least two bodies".into()));
        }
        resolve_all(ctx, &self.bodies, EntityKind::TopoBody)?;
        let vols = volumes_of_bodies(ctx, &self.bodies)?;
        guard_volumes(ctx, &vols)?;
        let o = ImprintOptions { merge: self.merge.unwrap_or(false), tol: self.tol };
        let (out, txn) = topo_txn(ctx, |topo, _model| imprint_bodies(topo, &self.bodies, &o).map_err(OpError::from))?;
        commit_topology(ctx, &txn, &[("faces", &out.faces), ("edges", &out.edges)])
    }
}

/// The cell ranks `model.merge` can be limited to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum MergeKindName {
    /// Vertices.
    Vertex,
    /// Edges.
    Edge,
    /// Faces.
    Face,
}

fn kinds_of(list: &Option<Vec<MergeKindName>>) -> MergeKinds {
    match list {
        None => MergeKinds::default(),
        Some(l) => MergeKinds { vertex: l.contains(&MergeKindName::Vertex), edge: l.contains(&MergeKindName::Edge), face: l.contains(&MergeKindName::Face) },
    }
}

/// The interface faces of the model's boundary tables: never merged.
fn interface_faces(model: &sm_model_api::Model) -> Vec<EntityId> {
    let mut out = BTreeSet::new();
    for b in model.boundaries() {
        for (f, fb) in model.boundary_faces(b.id) {
            if matches!(fb.role, FaceRole::Interface { .. }) {
                out.insert(f);
            }
        }
    }
    out.into_iter().collect()
}

fn check_scope(topo: &dyn TopologyView, entities: &Option<Vec<EntityId>>) -> Result<Vec<EntityId>, OpError> {
    let scope = entities.clone().unwrap_or_default();
    for &e in &scope {
        match Dim::of_real_kind(e.kind()) {
            Some(_) if topo.is_alive(e) => {}
            Some(_) => return Err(OpError::UnknownEntity(e)),
            None => return Err(OpError::Invalid(format!("{e} is not a topology cell"))),
        }
    }
    Ok(scope)
}

/// `model.merge {entities?, kinds?, tol?, check_only?}`: merge coincident vertices, edges and
/// faces (default: all bodies). Interface faces of the model's boundary tables are skipped.
/// Survivors are reported by rank.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Merge {
    /// The cells to merge within (default: everything).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoVertex", "TopoEdge", "TopoFace", "TopoVolume", "TopoBody"]))]
    pub entities: Option<Vec<EntityId>>,
    /// Cell ranks to merge (default all three).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub kinds: Option<Vec<MergeKindName>>,
    /// Merge tolerance (at most `linear_max`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tol: Option<f64>,
    /// Only validate the arguments; change nothing (use `model.merge_check` for the report).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub check_only: Option<bool>,
}

impl TypedOp for Merge {
    const NAME: &'static str = "model.merge";
    const OUTPUTS: &'static [OutputRole] = &[
        OutputRole::new("vertices", &[EntityKind::TopoVertex], Cardinality::Many, "The surviving merged vertices."),
        OutputRole::new("edges", &[EntityKind::TopoEdge], Cardinality::Many, "The surviving merged edges."),
        OutputRole::new("faces", &[EntityKind::TopoFace], Cardinality::Many, "The surviving merged faces."),
    ];

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let layer = ctx.layer::<ModelLayer>()?;
        let scope = check_scope(layer.topo(), &self.entities)?;
        let skip = interface_faces(layer.model());
        let opts = MergeOptions { tol: self.tol, kinds: kinds_of(&self.kinds) };
        let max = layer.tolerance().linear_max;
        if let Some(t) = self.tol
            && !(t.is_finite() && t >= 0.0 && t <= max)
        {
            return Err(OpError::Invalid(format!("`tol` {t:e} is outside [0, linear_max = {max:e}]")));
        }
        if self.check_only.unwrap_or(false) {
            return Ok(());
        }
        let (rep, txn) = topo_txn(ctx, |topo, _model| merge(topo, &scope, &skip, &opts).map_err(OpError::from))?;
        let survivors = |kind: crate::merge::MergeKind| -> Vec<EntityId> {
            let s: BTreeSet<EntityId> = rep.merged.iter().filter(|m| m.kind == kind).map(|m| m.keep).collect();
            s.into_iter().collect()
        };
        let (v, e, f) = (survivors(crate::merge::MergeKind::Vertex), survivors(crate::merge::MergeKind::Edge), survivors(crate::merge::MergeKind::Face));
        commit_topology(ctx, &txn, &[("vertices", &v), ("edges", &e), ("faces", &f)])
    }
}

/// `model.merge_check {entities?, tol?}`: report what a merge would do, without changing
/// anything: every candidate pair with its status (`mergeable`, `near`, `overlapping`,
/// `conflict`, `skipped`), the measured gap, the overlap area where it applies, and a hint.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MergeCheck {
    /// The cells to examine (default: everything).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoVertex", "TopoEdge", "TopoFace", "TopoVolume", "TopoBody"]))]
    pub entities: Option<Vec<EntityId>>,
    /// Merge tolerance to examine against.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tol: Option<f64>,
}

impl TypedQuery for MergeCheck {
    const NAME: &'static str = "model.merge_check";
    type Output = MergeReport;

    fn run(&self, state: &State, _cx: &QueryCtx<'_>) -> Result<MergeReport, OpError> {
        let layer = state.layer::<ModelLayer>().ok_or_else(|| OpError::MissingLayer("model".to_owned()))?;
        let scope = check_scope(layer.topo(), &self.entities)?;
        let skip = interface_faces(layer.model());
        let rep = merge_check(layer.topo(), &scope, &skip, &MergeOptions { tol: self.tol, kinds: MergeKinds::default() });
        if let Some(e) = &rep.error {
            return Err(OpError::Invalid(e.clone()));
        }
        Ok(rep)
    }
}

// ---------------------------------------------------------------------------------------------
// Webcut
// ---------------------------------------------------------------------------------------------

/// A coordinate axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum PlaneAxis {
    /// The plane `x = offset`.
    X,
    /// The plane `y = offset`.
    Y,
    /// The plane `z = offset`.
    Z,
}

/// `{axis, offset}`: an axis-aligned plane, normal along the positive axis.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AxisPlane {
    /// Axis perpendicular to the plane.
    pub axis: PlaneAxis,
    /// Coordinate of the plane along the axis.
    pub offset: f64,
}

/// `{origin, normal}`: a plane through a point; the positive side is where the normal points.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PointNormalPlane {
    /// A point of the plane.
    pub origin: [f64; 3],
    /// The normal (need not be unit).
    pub normal: [f64; 3],
}

/// `{vertices}`: the plane through three vertices; the normal is `(v1 - v0) x (v2 - v0)`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct VerticesPlane {
    /// Three vertices; reversing their order flips the positive and negative sides.
    #[schemars(extend("x-entity-kind" = ["TopoVertex"]))]
    pub vertices: [EntityId; 3],
}

/// The ways to specify the plane of `model.webcut_plane`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum PlaneSpec {
    /// An axis-aligned plane.
    Axis(AxisPlane),
    /// A point and a normal.
    PointNormal(PointNormalPlane),
    /// Three vertices.
    Vertices(VerticesPlane),
}

fn finite3(name: &str, v: [f64; 3]) -> Result<Vector3, OpError> {
    if v.iter().all(|c| c.is_finite()) { Ok(Vector3::new(v[0], v[1], v[2])) } else { Err(OpError::Invalid(format!("`{name}` must be finite, got {v:?}"))) }
}

fn vertex_point(topo: &dyn TopologyView, v: EntityId) -> Result<Point3, OpError> {
    match topo.carrier(v) {
        Carrier::Point(p) if topo.is_alive(v) => Ok(*p),
        _ => Err(OpError::WrongKind { id: v, expected: EntityKind::TopoVertex }),
    }
}

fn webcut_run(ctx: &mut ApplyCtx<'_>, targets: &[EntityId], tool: CutTool, keep: Option<KeepSide>, merge_opt: Option<bool>, imprint: Option<bool>) -> Result<(), OpError> {
    if targets.is_empty() {
        return Err(OpError::Invalid("`targets` must not be empty".into()));
    }
    for &t in targets {
        ctx.require(t)?;
        if !matches!(t.kind(), EntityKind::TopoBody | EntityKind::TopoVolume) {
            return Err(OpError::WrongKind { id: t, expected: EntityKind::TopoBody });
        }
    }
    let vols = volumes_of_bodies(ctx, targets)?;
    guard_volumes(ctx, &vols)?;
    let o = WebcutOptions { merge: merge_opt.unwrap_or(true), imprint: imprint.unwrap_or(false), keep: keep.unwrap_or_default(), consume_tool: false };
    let (out, txn) = topo_txn(ctx, |topo, _model| webcut(topo, targets, &tool, &o).map_err(OpError::from))?;
    commit_topology(ctx, &txn, &[("positive", &out.positive), ("negative", &out.negative), ("volumes", &out.volumes), ("cut_faces", &out.cut_faces), ("bodies", &out.bodies)])
}

/// `model.webcut_plane {targets, plane, keep?, merge?, imprint?}`: split bodies or volumes with a
/// plane. The positive side is where the plane normal points; `keep` keeps one half only (a
/// section); `merge: false` gives every piece its own copy of the cut face (Cubit `nomerge`).
/// Every piece becomes its own body.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct WebcutPlane {
    /// The bodies or volumes to cut.
    #[schemars(extend("x-entity-kind" = ["TopoBody", "TopoVolume"]))]
    pub targets: Vec<EntityId>,
    /// The plane.
    pub plane: PlaneSpec,
    /// Which halves to keep (default both).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub keep: Option<KeepSide>,
    /// Share the cut face between the pieces (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub merge: Option<bool>,
    /// Imprint the pieces on the other bodies (not available yet).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub imprint: Option<bool>,
}

impl TypedOp for WebcutPlane {
    const NAME: &'static str = "model.webcut_plane";
    const OUTPUTS: &'static [OutputRole] = WEBCUT_OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let topo = ctx.layer::<ModelLayer>()?.topo();
        let (origin, normal) = match &self.plane {
            PlaneSpec::Axis(a) => {
                if !a.offset.is_finite() {
                    return Err(OpError::Invalid("`offset` must be finite".into()));
                }
                match a.axis {
                    PlaneAxis::X => (Point3::new(a.offset, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)),
                    PlaneAxis::Y => (Point3::new(0.0, a.offset, 0.0), Vector3::new(0.0, 1.0, 0.0)),
                    PlaneAxis::Z => (Point3::new(0.0, 0.0, a.offset), Vector3::new(0.0, 0.0, 1.0)),
                }
            }
            PlaneSpec::PointNormal(p) => {
                let o = finite3("origin", p.origin)?;
                (Point3::new(o.x, o.y, o.z), finite3("normal", p.normal)?)
            }
            PlaneSpec::Vertices(v) => {
                let p: Vec<Point3> = v.vertices.iter().map(|&x| vertex_point(topo, x)).collect::<Result<_, _>>()?;
                let n = (p[1] - p[0]).cross(p[2] - p[0]);
                (p[0], n)
            }
        };
        if normal.norm().is_nan() || normal.norm() <= 0.0 {
            return Err(OpError::Invalid("the plane is degenerate (zero normal)".into()));
        }
        webcut_run(ctx, &self.targets, CutTool::Plane { origin, normal }, self.keep, self.merge, self.imprint)
    }
}

/// `model.webcut_surface {targets, face, keep?, merge?, imprint?}`: split with the plane of a
/// planar face (its normal gives the positive side).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct WebcutSurface {
    /// The bodies or volumes to cut.
    #[schemars(extend("x-entity-kind" = ["TopoBody", "TopoVolume"]))]
    pub targets: Vec<EntityId>,
    /// A planar face giving the tool plane.
    #[schemars(extend("x-entity-kind" = ["TopoFace"]))]
    pub face: EntityId,
    /// Which halves to keep (default both).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub keep: Option<KeepSide>,
    /// Share the cut face between the pieces (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub merge: Option<bool>,
    /// Imprint the pieces on the other bodies (not available yet).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub imprint: Option<bool>,
}

impl TypedOp for WebcutSurface {
    const NAME: &'static str = "model.webcut_surface";
    const OUTPUTS: &'static [OutputRole] = WEBCUT_OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        ctx.resolve_kind(self.face, EntityKind::TopoFace)?;
        let layer = ctx.layer::<ModelLayer>()?;
        let topo = layer.topo();
        let tol = layer.tolerance().linear;
        let plane = topo.face_plane(self.face, tol.max(1e-9)).map_err(|e| OpError::Failed(e.to_string()))?;
        let Some((n, d)) = plane else {
            return Err(OpError::Unsupported(format!("{} is not planar: webcut_surface takes a planar face (extended quadric tools are not available yet)", self.face)));
        };
        let origin = Point3::new(n.x * d, n.y * d, n.z * d);
        webcut_run(ctx, &self.targets, CutTool::Plane { origin, normal: n }, self.keep, self.merge, self.imprint)
    }
}

/// `model.webcut_cylinder {targets, radius, axis, center, keep?, merge?, imprint?}`: split with
/// an infinite cylinder of the given radius about the line through `center` along `axis`. The
/// positive side is the cylinder's outside.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct WebcutCylinder {
    /// The bodies or volumes to cut.
    #[schemars(extend("x-entity-kind" = ["TopoBody", "TopoVolume"]))]
    pub targets: Vec<EntityId>,
    /// Radius.
    pub radius: f64,
    /// Axis direction (need not be unit).
    pub axis: [f64; 3],
    /// A point on the axis.
    pub center: [f64; 3],
    /// Which halves to keep (default both).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub keep: Option<KeepSide>,
    /// Share the cut face between the pieces (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub merge: Option<bool>,
    /// Imprint the pieces on the other bodies (not available yet).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub imprint: Option<bool>,
}

impl TypedOp for WebcutCylinder {
    const NAME: &'static str = "model.webcut_cylinder";
    const OUTPUTS: &'static [OutputRole] = WEBCUT_OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if !(self.radius.is_finite() && self.radius > 0.0) {
            return Err(OpError::Invalid("`radius` must be finite and positive".into()));
        }
        let axis = finite3("axis", self.axis)?;
        let c = finite3("center", self.center)?;
        if axis.norm().is_nan() || axis.norm() <= 0.0 {
            return Err(OpError::Invalid("`axis` must not be zero".into()));
        }
        let frame = Frame::from_z(Point3::new(c.x, c.y, c.z), axis).map_err(|e| OpError::Invalid(e.to_string()))?;
        let cyl = sm_geom_api::Cylinder::new(frame, self.radius).map_err(|e| OpError::Invalid(e.to_string()))?;
        webcut_run(ctx, &self.targets, CutTool::Surface { surface: SurfaceDef::Cylinder(cyl), window: None }, self.keep, self.merge, self.imprint)
    }
}

/// Map a library error to the op error with the documented code (kept for callers outside the
/// ops that run the library directly).
pub fn op_error(e: BooleanError) -> OpError {
    e.into()
}
