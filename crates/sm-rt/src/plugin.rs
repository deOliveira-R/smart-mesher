//! The `rt` plugin: read-only queries `rt.info`, `rt.check`, `rt.locate` and `rt.partition` over a mesh of the
//! `meshes` layer and the boundary tables of the `model` layer. No layers, no ops.
//!
//! Tracking is not part of the protocol or of `.smsh` in W3a: the queries build the tracker in
//! process and keep it in the session's derived cache, keyed by the exact content of the
//! `model` and `meshes` layers and the query parameters.

use crate::{MeshTracker, MeshTrackerBuilder};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_layer_api::{MeshLayer, ModelLayer};
use sm_mesh_api::{CellIx, FaceIx};
use sm_op_api::names::{MESHES, MODEL};
use sm_op_api::{OpError, Plugin, QueryCtx, Registry, RegistryError, State, TypedQuery};
use sm_rt_api::{
    BoundarySpec, BuildInput, BuildIssue, BuildOptions, BuildReport, CellId, ComplementPolicy, MeshInput, RegionRule, ResponseKind, ResponseMap, RtError, Scratch, Tracker, TrackerCaps,
};
use std::sync::Arc;

/// The `rt` plugin: read-only ray-tracing queries (`rt.info`, `rt.check`, `rt.locate`, `rt.partition`).
#[derive(Clone, Copy, Debug, Default)]
pub struct RtPlugin;

impl Plugin for RtPlugin {
    fn id(&self) -> &str {
        "rt"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        registry.register_query::<RtInfo>()?;
        registry.register_query::<RtCheck>()?;
        registry.register_query::<RtLocate>()?;
        registry.register_query::<RtPartition>()?;
        registry.register_query::<crate::RtCheckOverlaps>()?;
        Ok(())
    }
}

/// What a response boundary record means to the tracker (the model only tags such faces).
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ResponseSpec {
    /// Particles leave the problem.
    Vacuum,
    /// Cosine-law re-emission.
    White,
    /// Specular reflection.
    Reflective,
    /// Survival probability in `[0, 1]`.
    Albedo(f64),
}

impl ResponseSpec {
    fn kind(&self) -> ResponseKind {
        match *self {
            Self::Vacuum => ResponseKind::Vacuum,
            Self::White => ResponseKind::White,
            Self::Reflective => ResponseKind::Reflective,
            Self::Albedo(a) => ResponseKind::Albedo(a),
        }
    }
}

/// Meaning of one response boundary record, by the record's name.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ResponseBinding {
    /// Name of the boundary record (`model.list` kind `Boundary`).
    pub boundary: String,
    /// What it does.
    pub response: ResponseSpec,
}

/// A located finding.
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct IssueOut {
    /// Stable machine code, for example `mesh.nonplanar_face`.
    pub code: String,
    /// Human text.
    pub message: String,
    /// The entity concerned, as text.
    pub entity: Option<String>,
    /// Where.
    pub location: Option<[f64; 3]>,
    /// Gap magnitude of a leak.
    pub gap: Option<f64>,
    /// Suggested fix.
    pub suggestion: Option<String>,
}

fn issue_out(i: &BuildIssue) -> IssueOut {
    IssueOut {
        code: i.code.to_owned(),
        message: i.message.clone(),
        entity: i.entity.map(|e| e.to_string()),
        location: i.location.map(|p| [p.x, p.y, p.z]),
        gap: i.gap,
        suggestion: i.suggestion.clone(),
    }
}

fn layers(state: &State) -> Result<(&ModelLayer, &MeshLayer), OpError> {
    let model = state.layer::<ModelLayer>().ok_or_else(|| OpError::MissingLayer(MODEL.to_owned()))?;
    let meshes = state.layer::<MeshLayer>().ok_or_else(|| OpError::MissingLayer(MESHES.to_owned()))?;
    Ok((model, meshes))
}

/// Boundary conditions from the model's boundary tables (`BoundarySpec::add_model_face`).
fn boundary_spec(model: &ModelLayer, bindings: &[ResponseBinding]) -> Result<BoundarySpec, OpError> {
    let m = model.model();
    let mut responses = ResponseMap::new();
    for b in bindings {
        let row = m.boundaries().find(|r| r.name == b.boundary).ok_or_else(|| OpError::Invalid(format!("no boundary record named `{}`", b.boundary)))?;
        responses.insert(row.id, b.response.kind());
    }
    let mut spec = BoundarySpec::default();
    for row in m.boundaries() {
        if matches!(row.kind, sm_model_api::BoundaryKind::Response) {
            responses.entry(row.id).or_insert(ResponseKind::Vacuum);
        }
        for (face, fb) in m.boundary_faces(row.id) {
            spec.add_model_face(face, fb, &row.kind, &responses).map_err(|i| OpError::Invalid(format!("{}: {}", i.code, i.message)))?;
        }
    }
    Ok(spec)
}

fn rt_error(e: RtError) -> OpError {
    match e {
        RtError::Build(issues) => {
            let first = issues.first().map(|i| format!("{}: {}", i.code, i.message)).unwrap_or_default();
            OpError::Failed(format!("tracker input has {} issue(s); first: {first} (run rt.check)", issues.len()))
        }
        other => OpError::Failed(other.to_string()),
    }
}

struct Built {
    tracker: MeshTracker,
    report: BuildReport,
}

fn tracker_for(state: &State, cx: &QueryCtx<'_>, mesh: EntityId, bindings: &[ResponseBinding]) -> Result<Arc<Built>, OpError> {
    let (model, meshes) = layers(state)?;
    let entry = meshes.get(mesh).ok_or(OpError::UnknownEntity(mesh))?;
    let params = serde_json::to_value((mesh.to_string(), bindings)).map_err(|e| OpError::Invalid(e.to_string()))?;
    let key = state.cache_key("rt.tracker", &[MODEL, MESHES], &params)?;
    cx.derived().get_or_compute_sized("rt.tracker", key, || {
        let spec = boundary_spec(model, bindings)?;
        let (rules, complement) = (RegionRule::ByVolumeClassification, ComplementPolicy::Outside);
        let input = BuildInput::Mesh(MeshInput::new(entry.mesh(), &rules, &spec, &complement));
        let mut opts = BuildOptions::default();
        opts.recovery = sm_rt_api::RecoveryPolicy::Fail;
        let (tracker, report) = MeshTrackerBuilder.build_concrete(&input, &opts).map_err(rt_error)?;
        let bytes = tracker.info().memory_bytes;
        Ok((Built { tracker, report }, bytes))
    })
}

/// `rt.info {mesh, responses?}`: structure facts of the tracker built from a mesh.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RtInfo {
    /// The mesh to track in.
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: EntityId,
    /// Meaning of the model's response boundary records (default: vacuum).
    #[serde(default)]
    pub responses: Vec<ResponseBinding>,
}

/// One region of the tracker.
#[derive(Debug, Serialize, JsonSchema)]
pub struct RegionOut {
    /// Dense region id.
    pub id: u32,
    /// Name.
    pub name: String,
    /// The volume it comes from.
    pub entity: Option<String>,
}

/// Result of `rt.info`.
#[derive(Debug, Serialize, JsonSchema)]
pub struct RtInfoOut {
    /// Tracking cells (the mesh cells).
    pub cells: usize,
    /// Faces.
    pub faces: usize,
    /// Faces tracked exactly (planar).
    pub exact_faces: usize,
    /// Faces tracked through their fan triangles (non-planar).
    pub faceted_faces: usize,
    /// Cells flagged convex (convex fast path); 0 for trackers without the flag.
    pub convex_cells: usize,
    /// Regions.
    pub regions: Vec<RegionOut>,
    /// Model surfaces.
    pub surfaces: usize,
    /// Heap bytes of the built structure.
    pub memory_bytes: usize,
    /// Bytes per face.
    pub bytes_per_face: f64,
    /// Bounding box `[lo, hi]`.
    pub bounds: [[f64; 3]; 2],
    /// Capability names.
    pub caps: Vec<String>,
    /// Structure hash (hex).
    pub structure_hash: String,
    /// Cache key components (hex): topology, boundary, options.
    pub key: Option<[String; 3]>,
    /// Non-fatal findings of the build.
    pub issues: Vec<IssueOut>,
}

impl TypedQuery for RtInfo {
    const NAME: &'static str = "rt.info";
    type Output = RtInfoOut;

    fn run(&self, state: &State, cx: &QueryCtx<'_>) -> Result<RtInfoOut, OpError> {
        let b = tracker_for(state, cx, self.mesh, &self.responses)?;
        let t = &b.tracker;
        let info = t.info();
        let caps = [(TrackerCaps::VOLUME_MESH, "volume_mesh"), (TrackerCaps::EXACT_PLANAR, "exact_planar")]
            .into_iter()
            .filter(|(c, _)| t.caps().has(*c))
            .map(|(_, n)| n.to_owned())
            .collect();
        Ok(RtInfoOut {
            cells: info.cells,
            faces: info.faces,
            exact_faces: info.exact_faces,
            faceted_faces: info.faceted_faces,
            convex_cells: info.convex_cells,
            regions: t
                .regions()
                .0
                .iter()
                .enumerate()
                .map(|(i, r)| RegionOut { id: i as u32, name: r.name.clone(), entity: r.entity.map(|e| e.to_string()) })
                .collect(),
            surfaces: t.surfaces().0.len(),
            memory_bytes: info.memory_bytes,
            bytes_per_face: info.memory_bytes as f64 / info.faces.max(1) as f64,
            bounds: [[info.bounds.lo.x, info.bounds.lo.y, info.bounds.lo.z], [info.bounds.hi.x, info.bounds.hi.y, info.bounds.hi.z]],
            caps,
            structure_hash: info.structure_hash.to_string(),
            key: info.key.map(|k| [k.topology_hash.to_string(), k.boundary_hash.to_string(), k.options_hash.to_string()]),
            issues: b.report.issues.iter().map(issue_out).collect(),
        })
    }
}

/// `rt.check {mesh, responses?}`: can a tracker be built, and what does the build find?
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RtCheck {
    /// The mesh to check.
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: EntityId,
    /// Meaning of the model's response boundary records (default: vacuum).
    #[serde(default)]
    pub responses: Vec<ResponseBinding>,
}

/// Result of `rt.check`.
#[derive(Debug, Serialize, JsonSchema)]
pub struct RtCheckOut {
    /// True when a tracker builds (warnings may remain).
    pub ok: bool,
    /// Fatal findings (when `ok` is false) or warnings (when true), located.
    pub issues: Vec<IssueOut>,
    /// Why the build was refused, when it was.
    pub error: Option<String>,
}

impl TypedQuery for RtCheck {
    const NAME: &'static str = "rt.check";
    type Output = RtCheckOut;

    fn run(&self, state: &State, _cx: &QueryCtx<'_>) -> Result<RtCheckOut, OpError> {
        let (model, meshes) = layers(state)?;
        let entry = meshes.get(self.mesh).ok_or(OpError::UnknownEntity(self.mesh))?;
        let spec = match boundary_spec(model, &self.responses) {
            Ok(s) => s,
            // A bad boundary record is a finding of the check, not a failed query.
            Err(OpError::Invalid(m)) => {
                let (code, message) = m.split_once(": ").map_or(("boundary.invalid", m.as_str()), |(c, t)| (c, t));
                let issue = IssueOut { code: code.to_owned(), message: message.to_owned(), entity: None, location: None, gap: None, suggestion: None };
                return Ok(RtCheckOut { ok: false, issues: vec![issue], error: Some("boundary conditions are invalid".into()) });
            }
            Err(e) => return Err(e),
        };
        let (rules, complement) = (RegionRule::ByVolumeClassification, ComplementPolicy::Outside);
        let input = BuildInput::Mesh(MeshInput::new(entry.mesh(), &rules, &spec, &complement));
        Ok(match MeshTrackerBuilder.build_concrete(&input, &BuildOptions::default()) {
            Ok((_, report)) => RtCheckOut { ok: true, issues: report.issues.iter().map(issue_out).collect(), error: None },
            Err(RtError::Build(issues)) => RtCheckOut { ok: false, issues: issues.iter().map(issue_out).collect(), error: Some("tracker input has issues".into()) },
            Err(e) => RtCheckOut { ok: false, issues: Vec::new(), error: Some(e.to_string()) },
        })
    }
}

/// `rt.locate {mesh, points, dir?, responses?}`: the cell and region containing each point.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RtLocate {
    /// The mesh to locate in.
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: EntityId,
    /// Query points.
    pub points: Vec<[f64; 3]>,
    /// Direction deciding a point on a face (default: the deterministic hashed direction).
    #[serde(default)]
    pub dir: Option<[f64; 3]>,
    /// Meaning of the model's response boundary records (default: vacuum).
    #[serde(default)]
    pub responses: Vec<ResponseBinding>,
}

/// Where one point is.
#[derive(Debug, Serialize, JsonSchema)]
pub struct LocateOut {
    /// The mesh cell containing the point, `None` outside the mesh.
    pub cell: Option<String>,
    /// Its dense cell index.
    pub cell_index: Option<u32>,
    /// The region (volume) of the cell.
    pub region: Option<String>,
    /// The mesh face the point lies on (within the model tolerance), when it does.
    pub on_face: Option<String>,
}

impl TypedQuery for RtLocate {
    const NAME: &'static str = "rt.locate";
    type Output = Vec<LocateOut>;

    fn run(&self, state: &State, cx: &QueryCtx<'_>) -> Result<Vec<LocateOut>, OpError> {
        let b = tracker_for(state, cx, self.mesh, &self.responses)?;
        let (_, meshes) = layers(state)?;
        let entry = meshes.get(self.mesh).ok_or(OpError::UnknownEntity(self.mesh))?;
        let m = entry.mesh();
        let dir = match self.dir {
            Some(d) => {
                let v = sm_core::Vector3::new(d[0], d[1], d[2]);
                sm_rt_api::check_direction(v, b.tracker.info().tol.angular).map_err(rt_error)?;
                Some(v)
            }
            None => None,
        };
        let mut sc = Scratch::new();
        let mut out = Vec::with_capacity(self.points.len());
        for p in &self.points {
            let loc = b.tracker.locate(sm_core::Point3::new(p[0], p[1], p[2]), dir, None, &mut sc).map_err(rt_error)?;
            let cell = (!loc.cell.is_none() && loc.cell != CellId::COMPLEMENT).then_some(loc.cell.0);
            out.push(LocateOut {
                cell: cell.map(|c| m.cell_id(CellIx(c)).to_string()),
                cell_index: cell,
                region: b.tracker.regions().get(loc.region).and_then(|r| r.entity).map(|e| e.to_string()),
                on_face: loc.on_face.map(|f| m.face_id(FaceIx(f.0)).to_string()),
            });
        }
        Ok(out)
    }
}

/// `rt.partition {mesh, leaf_cells?, max_depth?}`: the structure of the majorant partition a
/// delta-tracking code would use over a mesh (read-only; no cross section is involved, the split
/// rule looks at regions and cell counts only).
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RtPartition {
    /// The mesh to partition.
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: EntityId,
    /// Stop splitting a node of at most this many candidate cells (default 8, at least 1).
    #[serde(default)]
    pub leaf_cells: Option<u32>,
    /// Deepest split (default 24, at most 48).
    #[serde(default)]
    pub max_depth: Option<u32>,
}

/// Cells (candidates) per leaf node.
#[derive(Clone, Debug, Serialize, JsonSchema, PartialEq)]
pub struct CellsPerLeaf {
    /// Fewest.
    pub min: u32,
    /// Most.
    pub max: u32,
    /// Mean.
    pub mean: f64,
}

/// Result of `rt.partition`.
#[derive(Clone, Debug, Serialize, JsonSchema, PartialEq)]
pub struct RtPartitionOut {
    /// Partition nodes (the length of a caller's majorant slice).
    pub nodes: usize,
    /// Leaf nodes (the ones that yield delta segments).
    pub leaves: usize,
    /// Longest root-to-leaf path, in edges.
    pub depth: u32,
    /// Leaves whose box lies inside one convex cell.
    pub preassigned_leaves: usize,
    /// Distinct material sets (regions of a node, deduplicated).
    pub sets: usize,
    /// Candidate cells per leaf.
    pub cells_per_leaf: CellsPerLeaf,
    /// Heap bytes of the partition arrays.
    pub memory_bytes: usize,
    /// Structure hash of the tracker (hex).
    pub structure_hash: String,
    /// Content hash of the partition itself (hex): arrays, sets and options.
    pub partition_hash: String,
}

impl TypedQuery for RtPartition {
    const NAME: &'static str = "rt.partition";
    type Output = RtPartitionOut;

    fn run(&self, state: &State, cx: &QueryCtx<'_>) -> Result<RtPartitionOut, OpError> {
        let (leaf_cells, max_depth) = (self.leaf_cells.unwrap_or(8), self.max_depth.unwrap_or(24));
        if leaf_cells == 0 {
            return Err(OpError::Invalid("leaf_cells must be at least 1".into()));
        }
        if max_depth > 48 {
            return Err(OpError::Invalid("max_depth must be at most 48".into()));
        }
        let (model, meshes) = layers(state)?;
        let entry = meshes.get(self.mesh).ok_or(OpError::UnknownEntity(self.mesh))?;
        // The canonical input has both parameters filled in, so omitting one equals passing its default.
        let params = serde_json::to_value((self.mesh.to_string(), leaf_cells, max_depth)).map_err(|e| OpError::Invalid(e.to_string()))?;
        let key = state.cache_key("rt.partition", &[MODEL, MESHES], &params)?;
        let out = cx.derived().get_or_compute_sized("rt.partition", key, || {
            let spec = boundary_spec(model, &[])?;
            let (rules, complement) = (RegionRule::ByVolumeClassification, ComplementPolicy::Outside);
            let input = BuildInput::Mesh(MeshInput::new(entry.mesh(), &rules, &spec, &complement));
            let mut opts = BuildOptions::default();
            opts.recovery = sm_rt_api::RecoveryPolicy::Fail;
            let mut p = sm_rt_api::PartitionOptions::default();
            p.enabled = true;
            p.leaf_cells = leaf_cells;
            p.max_depth = max_depth;
            let opts = opts.with_partition(p);
            let (tracker, _) = MeshTrackerBuilder.build_concrete(&input, &opts).map_err(rt_error)?;
            let part = tracker.partition().ok_or_else(|| OpError::Failed("the tracker has no partition".into()))?;
            let st = part.stats();
            let out = RtPartitionOut {
                nodes: st.nodes,
                leaves: st.leaves,
                depth: st.depth,
                preassigned_leaves: st.preassigned,
                sets: st.sets,
                cells_per_leaf: CellsPerLeaf { min: st.cells_min, max: st.cells_max, mean: st.cells_mean },
                memory_bytes: st.bytes,
                structure_hash: tracker.info().structure_hash.to_string(),
                partition_hash: part.content_hash().to_string(),
            };
            Ok::<_, OpError>((out, 256))
        })?;
        Ok((*out).clone())
    }
}
