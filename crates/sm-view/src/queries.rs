//! The three read-only queries of the `view` plugin: `view.scene`, `view.mesh_skin`,
//! `view.mesh_scalar`.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_layer_api::{MeshLayer, ModelLayer};
use sm_mesh_api::{CellIx, MeshRead, cell_volume_centroid, connectivity_hash, coordinate_hash};
use sm_op_api::{Hash256, OpError, QueryCtx, State, TypedQuery, names};
use sm_quality::{Measure, Metric, Options, Profile, Sense, rules};
use sm_topo_api::{Carrier, Dim};

use crate::b64;
use crate::error::ViewError;
use crate::skin::{Clip, SkinOptions, skin};

/// Wire version of the replies.
pub const SCHEMA_VERSION: u32 = 1;

/// Number of entries in the label palette.
pub const PALETTE_LEN: usize = 12;

const PALETTE: [[u8; 3]; PALETTE_LEN] = [
    [76, 120, 168],
    [245, 133, 24],
    [82, 162, 59],
    [228, 87, 86],
    [178, 121, 162],
    [157, 117, 93],
    [242, 142, 171],
    [118, 183, 178],
    [237, 201, 72],
    [186, 176, 172],
    [95, 90, 166],
    [160, 203, 232],
];

/// Display color of a model color label: `PALETTE[label % 12]`. Model colors are labels without
/// RGB, so the viewer derives one deterministically.
pub fn palette_rgb(label: u32) -> [u8; 3] {
    PALETTE[label as usize % PALETTE_LEN]
}

/// The mesh a request names, or the only mesh of the state.
fn resolve_mesh(layer: &MeshLayer, mesh: Option<EntityId>) -> Result<EntityId, ViewError> {
    match mesh {
        Some(id) => layer.get(id).map(|_| id).ok_or(ViewError::UnknownMesh(id)),
        None => {
            let ids: Vec<EntityId> = layer.meshes().map(|(id, _)| id).collect();
            match ids.as_slice() {
                [only] => Ok(*only),
                [] => Err(ViewError::NoMesh),
                _ => Err(ViewError::AmbiguousMesh(ids)),
            }
        }
    }
}

fn mesh_layer(state: &State) -> Result<&MeshLayer, OpError> {
    state.layer::<MeshLayer>().ok_or_else(|| OpError::MissingLayer(names::MESHES.to_owned()))
}

fn fin(x: f64) -> Option<f64> {
    x.is_finite().then_some(x)
}

// ---------------------------------------------------------------- view.scene

/// Input of `view.scene` (none).
#[derive(Clone, Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SceneQuery {}

/// A model volume with its display color.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct VolumeOut {
    /// The volume.
    #[schemars(extend("x-entity-kind" = ["TopoVolume"]))]
    pub id: EntityId,
    /// Label of its model color, if colored.
    pub color_label: Option<u32>,
    /// Palette color of the label, if colored.
    pub color: Option<[u8; 3]>,
}

/// A model body.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct BodyOut {
    /// The body.
    #[schemars(extend("x-entity-kind" = ["TopoBody"]))]
    pub id: EntityId,
    /// Its volumes, ascending.
    pub volumes: Vec<EntityId>,
    /// Palette color of its first colored volume.
    pub color: Option<[u8; 3]>,
    /// Label of that color.
    pub color_label: Option<u32>,
    /// Name of that color, if any.
    pub name: Option<String>,
}

/// One mesh of the state.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct MeshOut {
    /// The mesh.
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub id: EntityId,
    /// Display name (the id text; meshes have no names yet).
    pub name: String,
    /// Topological dimension, 2 or 3.
    pub dim: u8,
    /// Cell count.
    pub cells: u32,
    /// Facet count.
    pub facets: u32,
    /// Node count.
    pub nodes: u32,
    /// Bounding box of the nodes.
    pub bbox: [[f64; 3]; 2],
    /// Content digest (hex); the client refetches buffers only when it changes.
    pub digest: String,
    /// The mesh it was derived from, if any.
    pub derived_from: Vec<EntityId>,
}

/// Reply of `view.scene`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct SceneReply {
    /// Wire version, 1.
    pub schema_version: u32,
    /// Bounding box of the model vertices and the mesh nodes (zeros when empty).
    pub bbox: [[f64; 3]; 2],
    /// Model volumes with colors, ascending.
    pub volumes: Vec<VolumeOut>,
    /// Model bodies, ascending.
    pub bodies: Vec<BodyOut>,
    /// Meshes, ascending.
    pub meshes: Vec<MeshOut>,
}

fn mesh_bbox(m: &dyn MeshRead) -> Option<[[f64; 3]; 2]> {
    let co = m.coords();
    if co.is_empty() {
        return None;
    }
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for i in 0..co.len() {
        let p = [co.x[i], co.y[i], co.z[i]];
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    Some([lo, hi])
}

/// Content digest of a mesh: id, connectivity and coordinates (exact bits).
pub fn mesh_digest(m: &dyn MeshRead) -> Hash256 {
    let mut b = Vec::with_capacity(24);
    b.extend_from_slice(&m.mesh_id().to_raw().to_le_bytes());
    b.extend_from_slice(&connectivity_hash(m).to_le_bytes());
    b.extend_from_slice(&coordinate_hash(m).to_le_bytes());
    Hash256::of_domain("smartmesh.view.mesh.v1", &b)
}

fn union(acc: &mut Option<[[f64; 3]; 2]>, b: [[f64; 3]; 2]) {
    match acc {
        None => *acc = Some(b),
        Some(a) => {
            for k in 0..3 {
                a[0][k] = a[0][k].min(b[0][k]);
                a[1][k] = a[1][k].max(b[1][k]);
            }
        }
    }
}

impl TypedQuery for SceneQuery {
    const NAME: &'static str = "view.scene";
    type Output = SceneReply;

    fn run(&self, state: &State, _cx: &QueryCtx<'_>) -> Result<SceneReply, OpError> {
        let mut bbox: Option<[[f64; 3]; 2]> = None;
        let (mut volumes, mut bodies) = (Vec::new(), Vec::new());
        if let Some(ml) = state.layer::<ModelLayer>() {
            let topo = ml.topo();
            let model = ml.model();
            let color_of = |v: EntityId| {
                let row = model.live_volume_color(v, topo).and_then(|c| model.color(c));
                (row.map(|r| r.label), row.map(|r| r.name.clone()))
            };
            for &v in topo.cells(Dim::Volume) {
                let (label, _) = color_of(v);
                volumes.push(VolumeOut { id: v, color_label: label, color: label.map(palette_rgb) });
            }
            for &b in topo.cells(Dim::Body) {
                let mut vs: Vec<EntityId> = topo
                    .boundary(b)
                    .iter()
                    .map(|i| i.cell)
                    .filter(|c| c.kind() == sm_core::EntityKind::TopoVolume)
                    .collect();
                vs.sort();
                vs.dedup();
                let first = vs.iter().find_map(|v| {
                    let (l, n) = color_of(*v);
                    l.map(|l| (l, n))
                });
                bodies.push(BodyOut {
                    id: b,
                    volumes: vs,
                    color: first.as_ref().map(|(l, _)| palette_rgb(*l)),
                    color_label: first.as_ref().map(|(l, _)| *l),
                    name: first.and_then(|(_, n)| n),
                });
            }
            for &v in topo.cells(Dim::Vertex) {
                if let Carrier::Point(p) = topo.carrier(v) {
                    union(&mut bbox, [[p.x, p.y, p.z], [p.x, p.y, p.z]]);
                }
            }
        }
        let mut meshes = Vec::new();
        if let Some(layer) = state.layer::<MeshLayer>() {
            for (id, e) in layer.meshes() {
                let m = e.mesh();
                let bb = mesh_bbox(m);
                if let Some(b) = bb {
                    union(&mut bbox, b);
                }
                meshes.push(MeshOut {
                    id,
                    name: id.to_string(),
                    dim: m.dim().get(),
                    cells: m.cell_count() as u32,
                    facets: m.face_count() as u32,
                    nodes: m.coords().len() as u32,
                    bbox: bb.unwrap_or([[0.0; 3]; 2]),
                    digest: mesh_digest(m).to_hex(),
                    derived_from: e.derived_from.into_iter().collect(),
                });
            }
        }
        Ok(SceneReply { schema_version: SCHEMA_VERSION, bbox: bbox.unwrap_or([[0.0; 3]; 2]), volumes, bodies, meshes })
    }
}

// ------------------------------------------------------------ view.mesh_skin

/// Clip plane of `view.mesh_skin`.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ClipIn {
    /// A point on the plane.
    pub origin: [f64; 3],
    /// Plane normal; cells on the side it points to are removed.
    pub normal: [f64; 3],
}

fn default_true() -> bool {
    true
}
fn default_max_tris() -> u32 {
    SkinOptions::default().max_tris
}

/// Input of `view.mesh_skin`.
#[derive(Clone, Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SkinQuery {
    /// The mesh; the only mesh of the state when omitted.
    #[serde(default)]
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: Option<EntityId>,
    /// Keep only cells classified on these volumes.
    #[serde(default)]
    #[schemars(extend("x-entity-kind" = ["TopoVolume"]))]
    pub volumes: Option<Vec<EntityId>>,
    /// Clip plane (cell based, see DESIGN.md).
    #[serde(default)]
    pub clip: Option<ClipIn>,
    /// Include the polygon-edge line list (default true).
    #[serde(default = "default_true")]
    pub outline: bool,
    /// Refuse above this many triangles (default 2 000 000).
    #[serde(default = "default_max_tris")]
    pub max_tris: u32,
}

/// Reply of `view.mesh_skin`: the [`Skin`](crate::Skin) with base64 buffers.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct SkinReply {
    /// Wire version, 1.
    pub schema_version: u32,
    /// The mesh.
    pub mesh: EntityId,
    /// Triangle count.
    pub tri_count: u32,
    /// Vertex count.
    pub vertex_count: u32,
    /// Vertices that are mesh nodes (the rest are polygon centroids).
    pub node_vertices: u32,
    /// Base64 of `vertex_count * 3` little-endian `f32`.
    pub positions: String,
    /// Base64 of `tri_count * 3` little-endian `u32` vertex indices.
    pub indices: String,
    /// Base64 of `tri_count` little-endian `u32`: dense index of the visible cell.
    pub tri_cell: String,
    /// Base64 of `tri_count` little-endian `u32`: 0 = none, else 1 + index into `owners`.
    pub tri_owner: String,
    /// Topo entities owning triangles (ascending); `tri_owner` indexes this table.
    pub owners: Vec<EntityId>,
    /// Base64 of little-endian `u32` pairs: polygon edges as a line list (empty without `outline`).
    pub outline_indices: String,
    /// Bounding box of the skin.
    pub bbox: [[f64; 3]; 2],
    /// Cells removed by the clip plane.
    pub clipped_cells: u32,
    /// Cells remaining after volume filter and clip.
    pub kept_cells: u32,
}

impl SkinReply {
    /// Encode a skin.
    pub fn new(mesh: EntityId, s: &crate::Skin) -> Self {
        Self {
            schema_version: SCHEMA_VERSION,
            mesh,
            tri_count: s.tri_count() as u32,
            vertex_count: s.vertex_count() as u32,
            node_vertices: s.node_vertices,
            positions: b64::encode(&b64::f32_bytes(&s.positions.iter().map(|x| *x as f32).collect::<Vec<_>>())),
            indices: b64::encode(&b64::u32_bytes(&s.indices)),
            tri_cell: b64::encode(&b64::u32_bytes(&s.tri_cell)),
            tri_owner: b64::encode(&b64::u32_bytes(&s.tri_owner)),
            owners: s.owners.clone(),
            outline_indices: b64::encode(&b64::u32_bytes(&s.outline_indices)),
            bbox: s.bbox,
            clipped_cells: s.clipped_cells,
            kept_cells: s.kept_cells,
        }
    }
}

impl TypedQuery for SkinQuery {
    const NAME: &'static str = "view.mesh_skin";
    type Output = SkinReply;

    fn run(&self, state: &State, cx: &QueryCtx<'_>) -> Result<SkinReply, OpError> {
        let layer = mesh_layer(state)?;
        let id = resolve_mesh(layer, self.mesh)?;
        let entry = layer.get(id).ok_or(ViewError::UnknownMesh(id))?;
        let opts = SkinOptions {
            volumes: self.volumes.clone(),
            clip: self.clip.map(|c| Clip { origin: c.origin, normal: c.normal }),
            outline: self.outline,
            max_tris: self.max_tris,
        };
        cx.step(0, 1, "building skin")?;
        let s = skin(entry.mesh(), &opts)?;
        Ok(SkinReply::new(id, &s))
    }
}

// --------------------------------------------------------- view.mesh_scalar

fn default_worst() -> u32 {
    20
}

/// Input of `view.mesh_scalar`.
#[derive(Clone, Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ScalarQuery {
    /// The mesh; the only mesh of the state when omitted.
    #[serde(default)]
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: Option<EntityId>,
    /// An `sm-quality` cell metric name (`scaled_jacobian`, `aspect_ratio`, ...).
    pub metric: String,
    /// Count values below this as bad.
    #[serde(default)]
    pub bad_below: Option<f64>,
    /// Count values above this as bad.
    #[serde(default)]
    pub bad_above: Option<f64>,
    /// Number of worst cells to list, at most 20 (default 20).
    #[serde(default = "default_worst")]
    pub worst: u32,
}

/// A cell reference.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct CellRef {
    /// Stable id of the cell.
    pub id: EntityId,
    /// Dense index.
    pub index: u32,
}

/// One of the worst cells.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct WorstCell {
    /// The cell.
    pub cell: CellRef,
    /// Its metric value.
    pub value: f64,
    /// Its centroid.
    pub location: [f64; 3],
}

/// Reply of `view.mesh_scalar`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct ScalarReply {
    /// Wire version, 1.
    pub schema_version: u32,
    /// The mesh.
    pub mesh: EntityId,
    /// Wire name of the metric.
    pub metric: String,
    /// Number of cells (length of `values`).
    pub count: u32,
    /// Base64 of `count` little-endian `f32` (the `sm_quality::compute` value cast once; NaN where undefined).
    pub values: String,
    /// Smallest defined value (`null` if none).
    pub min: Option<f64>,
    /// Largest defined value.
    pub max: Option<f64>,
    /// Mean of the defined values.
    pub mean: Option<f64>,
    /// Cells where the metric is undefined (NaN).
    pub undefined_cells: u32,
    /// Cells below `bad_below` or above `bad_above` (0 when neither is given).
    pub bad_cells: u32,
    /// `low` or `high`: the end of the range `worst` lists.
    pub worst_end: String,
    /// Up to 20 worst cells, worst first, ties by cell index.
    pub worst: Vec<WorstCell>,
}

/// Valid metric names for `view.mesh_scalar`.
pub fn metric_names() -> Vec<String> {
    Metric::ALL.iter().map(|m| m.name().to_owned()).collect()
}

impl TypedQuery for ScalarQuery {
    const NAME: &'static str = "view.mesh_scalar";
    type Output = ScalarReply;

    fn run(&self, state: &State, cx: &QueryCtx<'_>) -> Result<ScalarReply, OpError> {
        let layer = mesh_layer(state)?;
        let id = resolve_mesh(layer, self.mesh)?;
        let entry = layer.get(id).ok_or(ViewError::UnknownMesh(id))?;
        let m = entry.mesh();
        let metric = Metric::from_name(&self.metric)
            .ok_or_else(|| ViewError::UnknownMetric { name: self.metric.clone(), valid: metric_names() })?;
        cx.step(0, 1, "computing metric")?;
        let col = sm_quality::compute(m, metric, &Options::default())
            .map_err(|_| ViewError::Unsupported(self.metric.clone()))?;
        let n = col.len();
        let values: Vec<f32> = col.iter().map(|v| *v as f32).collect();
        let (mut lo, mut hi, mut sum, mut cnt) = (f64::INFINITY, f64::NEG_INFINITY, 0.0f64, 0u32);
        let mut bad = 0u32;
        for &v in &col {
            if !v.is_finite() {
                continue;
            }
            lo = lo.min(v);
            hi = hi.max(v);
            sum += v;
            cnt += 1;
            if self.bad_below.is_some_and(|b| v < b) || self.bad_above.is_some_and(|b| v > b) {
                bad += 1;
            }
        }
        let high_is_worse = match (self.bad_below, self.bad_above) {
            (None, Some(_)) => true,
            (Some(_), _) => false,
            (None, None) => rules(Profile::Generic)
                .iter()
                .find(|r| r.measure == Measure::Verdict(metric))
                .is_some_and(|r| r.sense == Sense::HigherIsWorse),
        };
        let mut order: Vec<usize> = (0..n).filter(|&i| col[i].is_finite()).collect();
        order.sort_by(|&a, &b| {
            let o = col[a].total_cmp(&col[b]);
            let o = if high_is_worse { o.reverse() } else { o };
            o.then(a.cmp(&b))
        });
        let worst = order
            .into_iter()
            .take(self.worst.min(20) as usize)
            .map(|i| {
                let c = CellIx::new(i);
                let p = cell_volume_centroid(m, c).1;
                WorstCell {
                    cell: CellRef { id: m.cell_id(c), index: i as u32 },
                    value: col[i],
                    location: [p.x, p.y, p.z],
                }
            })
            .collect();
        Ok(ScalarReply {
            schema_version: SCHEMA_VERSION,
            mesh: id,
            metric: metric.name().to_owned(),
            count: n as u32,
            values: b64::encode(&b64::f32_bytes(&values)),
            min: fin(lo),
            max: fin(hi),
            mean: (cnt > 0).then(|| sum / f64::from(cnt)),
            undefined_cells: n as u32 - cnt,
            bad_cells: bad,
            worst_end: if high_is_worse { "high" } else { "low" }.to_owned(),
            worst,
        })
    }
}
