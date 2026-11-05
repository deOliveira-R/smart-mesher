//! The read-only query `rt.check_overlaps` (kind per ruling O6): the sampled overlap and gap
//! check over the model seen as CSG cells ([`crate::csg_world`]).

use crate::{BridgeError, CheckScope, CsgWorldOptions, MAX_FLAT_CELLS, check_world_overlaps, csg_world};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_layer_api::ModelLayer;
use sm_op_api::names::MODEL;
use sm_op_api::{OpError, QueryCtx, State, TypedQuery};

/// Default sample points of `rt.check_overlaps`.
const DEFAULT_SAMPLES: u32 = 10_000;
/// Default random chords of `rt.check_overlaps` (8 points each).
const DEFAULT_RAYS: u32 = 500;

/// `rt.check_overlaps {bodies?, samples?, rays?, seed?}`: sampled overlap and gap check of the
/// model seen as CSG cells (read-only; see [`csg_world`]). Stratified points and random chords are
/// classified against every cell; two matching cells are a `csg.overlap` (with the point and both
/// cells), none a `csg.gap`. The samples depend only on `seed` (default 0) and the cell geometry,
/// so the findings are the same for any thread count and every repeat. The effective seed is part
/// of the cached input and is echoed in the output.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RtCheckOverlaps {
    /// Bodies or volumes to check (default all).
    #[serde(default)]
    #[schemars(extend("x-entity-kind" = ["TopoBody", "TopoVolume"]))]
    pub bodies: Option<Vec<EntityId>>,
    /// Stratified sample points per leaf (default 10000, at most 10^7).
    #[serde(default)]
    pub samples: Option<u32>,
    /// Random chords per leaf, 8 points each (default 500, at most 10^6).
    #[serde(default)]
    pub rays: Option<u32>,
    /// RNG seed (default 0): never derived from time or threads.
    #[serde(default)]
    pub seed: Option<u64>,
}

/// A cell named by a finding.
#[derive(Clone, Debug, Serialize, JsonSchema, PartialEq)]
pub struct OverlapCellOut {
    /// The model volume.
    pub volume: String,
    /// The cell (region) name, the volume's color name.
    pub name: String,
    /// The placed copy (depth-first placement order) in a flattened world; 0 otherwise.
    pub copy: u32,
}

/// One located finding of `rt.check_overlaps`.
#[derive(Clone, Debug, Serialize, JsonSchema, PartialEq)]
pub struct OverlapFindingOut {
    /// `csg.overlap`, `csg.gap`, `csg.unbounded`, ...
    pub code: String,
    /// Human text.
    pub message: String,
    /// The sample point.
    pub location: Option<[f64; 3]>,
    /// The cells named (both for an overlap).
    pub cells: Vec<OverlapCellOut>,
    /// Suggested fix.
    pub suggestion: Option<String>,
}

/// Result of `rt.check_overlaps`.
#[derive(Clone, Debug, Serialize, JsonSchema, PartialEq)]
pub struct RtCheckOverlapsOut {
    /// True when no overlap was found (gaps are reported but do not fail the check).
    pub ok: bool,
    /// The effective seed.
    pub seed: u64,
    /// Sample points per leaf.
    pub samples: u32,
    /// Random chords per leaf.
    pub rays: u32,
    /// Whether placed copies were checked together or prototype leaves alone.
    pub scope: CheckScope,
    /// Cells of the world with every placed copy counted.
    pub cells: u64,
    /// Number of `csg.overlap` findings.
    pub overlaps: usize,
    /// The findings (at most 16 per code and leaf).
    pub findings: Vec<OverlapFindingOut>,
}

fn bridge_error(e: BridgeError) -> OpError {
    match e {
        BridgeError::Invalid(m) => OpError::Invalid(m),
        other => OpError::Failed(format!("csg.unsupported: {other}")),
    }
}

impl TypedQuery for RtCheckOverlaps {
    const NAME: &'static str = "rt.check_overlaps";
    type Output = RtCheckOverlapsOut;

    fn run(&self, state: &State, cx: &QueryCtx<'_>) -> Result<RtCheckOverlapsOut, OpError> {
        let (samples, rays, seed) = (self.samples.unwrap_or(DEFAULT_SAMPLES), self.rays.unwrap_or(DEFAULT_RAYS), self.seed.unwrap_or(0));
        if samples > 10_000_000 || rays > 1_000_000 {
            return Err(OpError::Invalid("samples is at most 10000000 and rays at most 1000000".into()));
        }
        let model = state.layer::<ModelLayer>().ok_or_else(|| OpError::MissingLayer(MODEL.to_owned()))?;
        // The canonical input has every default filled in: omitting `seed` equals `seed: 0`.
        let bodies: Option<Vec<String>> = self.bodies.as_ref().map(|b| b.iter().map(ToString::to_string).collect());
        let params = serde_json::to_value((bodies, samples, rays, seed)).map_err(|e| OpError::Invalid(e.to_string()))?;
        let key = state.cache_key("rt.check_overlaps", &[MODEL], &params)?;
        let out = cx.derived().get_or_compute_sized("rt.check_overlaps", key, || {
            let opts = CsgWorldOptions { bodies: self.bodies.clone(), complement: None };
            let world = csg_world(model.model(), model.topo(), &opts).map_err(bridge_error)?;
            let (scope, found) = check_world_overlaps(&world, samples as usize, rays as usize, seed, MAX_FLAT_CELLS).map_err(bridge_error)?;
            let findings: Vec<OverlapFindingOut> = found
                .iter()
                .map(|f| OverlapFindingOut {
                    code: f.issue.code.to_owned(),
                    message: f.issue.message.clone(),
                    location: f.issue.location.map(|p| [p.x, p.y, p.z]),
                    cells: f.cells.iter().map(|c| OverlapCellOut { volume: c.volume.to_string(), name: c.name.clone(), copy: c.copy }).collect(),
                    suggestion: f.issue.suggestion.clone(),
                })
                .collect();
            let overlaps = findings.iter().filter(|f| f.code == "csg.overlap").count();
            let out = RtCheckOverlapsOut { ok: overlaps == 0, seed, samples, rays, scope, cells: world.flat_cell_count(), overlaps, findings };
            let bytes = 256 + out.findings.len() * 256;
            Ok::<_, OpError>((out, bytes))
        })?;
        Ok((*out).clone())
    }
}
