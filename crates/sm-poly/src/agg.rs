//! Op `mesh.agglomerate`: a coarse polyhedral mesh over a fine one, linked by a `LevelMap`.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{MeshEntry, MeshLayer, ModelLayer, insert_mesh, retire_mesh};
use sm_mesh::MeshBuilder;
use sm_mesh_api::{CatalogEdit, CellIx, GeomTol, LevelMap, MeshDim, MeshEntityKind, MeshRead, SerialAlloc, validate, validate_geometry};
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, TypedOp};
use std::collections::BTreeMap;

use crate::coarsen;
use crate::error::PolyError;

/// How the fine cells are grouped into coarse cells.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum AggMode {
    /// Greedy pairwise merging down to about `cells` coarse cells (never fewer; fewer merges if
    /// the convex rule stops it). Never merges across model volumes.
    Target {
        /// Wanted number of coarse cells (at least 1).
        cells: usize,
    },
    /// Axis-aligned bins of the fine cell centroids: one coarse cell per (volume, bin, connected
    /// part). A group that fails the convex rule (when `convex` is on) is left as single cells.
    Blocks {
        /// Bin edge lengths (positive).
        size: [f64; 3],
        /// A bin corner (default the origin).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        origin: Option<[f64; 3]>,
    },
    /// CMFD pin mode: one coarse cell per pin (or `cells_per_group` of them), spanning every
    /// volume of the pin. A pin must be face-connected.
    PerPin {
        /// What a pin is.
        group: PinGroup,
        /// Coarse cells per pin (default 1). Above 1 the pin is split by greedy convex merging
        /// down to this count, within the pin.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        cells_per_group: Option<usize>,
    },
}

/// The grouping key of [`AggMode::PerPin`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum PinGroup {
    /// Every model volume is one pin.
    Volume,
    /// The lattice position of the volume (ruling D9 region tag: all frame slots, ignoring ring
    /// and sector, so a subdivided pin is one group). A volume without a lattice tag is its own
    /// group.
    Lattice,
}

/// `mesh.agglomerate`: build a coarse mesh by grouping the cells of a fine (typically
/// polyhedral) mesh and link the two with a fine-to-coarse `LevelMap` including its face map
/// (ruling D3, CMFD/multigrid levels).
///
/// The coarse mesh is a NEW mesh entry (output role `mesh`) over the same model, with the fine
/// mesh's points (a vertex collinear in every coarse face that holds it is dropped, so a
/// merged face has no hanging vertices). A coarse face is the union of the edge-adjacent coplanar
/// fine faces between the same two coarse cells and the same model entity; faces between cells of
/// one coarse cell vanish, so interface and boundary faces survive with their classification and
/// the total volume is preserved. A coarse cell takes the class of the model volume holding most
/// of its volume. Faces of different model entities (a fuel cap and a moderator face on one
/// plane) stay separate coarse faces, except in `per_pin` mode, where they merge into one face of
/// the class with the most area. Merge gate: the exact `sm-rt` convex rule on the merged cell (no separate
/// concavity number). `convex: false` lifts it for `blocks` and `per_pin` with one cell per pin
/// (which then still need a valid, star-shaped coarse cell); `target` and `per_pin` with several
/// cells per pin always merge greedily under the gate, so `convex: false` is refused there. Block groups that fail the rule are
/// left as single cells and reported as the warning `agglomerate.dissolved`.
///
/// With the default `keep_source` the fine mesh stays and the map is recorded in the mesh layer
/// (`link`); with `keep_source: false` the fine mesh is retired and, with it, the link (a map
/// needs both meshes). Cell sets and tags of the fine mesh are not carried over. The delta
/// records `source -> [source, new]` (`source -> [new]`).
///
/// Atomic. A non-3D mesh, a mesh with non-conformal faces or inactive cells, bad parameters, a
/// pin that is not face-connected, or a request that merges nothing fail with `OpError::Invalid`
/// or `OpError::Failed`; the state is unchanged.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshAgglomerate {
    /// The fine mesh.
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: EntityId,
    /// The grouping.
    pub mode: AggMode,
    /// Keep the fine mesh entry (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub keep_source: Option<bool>,
    /// Require every merged coarse cell to pass the convex rule (default true). `false` is only
    /// accepted for `blocks` and `per_pin` with one cell per pin; it is refused for `target` and for
    /// `per_pin` with `cells_per_group` above 1, whose greedy merging always applies the gate.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub convex: Option<bool>,
}

const OUTPUTS: &[OutputRole] = &[OutputRole::new("mesh", &[EntityKind::Mesh], Cardinality::One, "the new coarse mesh")];

fn refuse(e: PolyError) -> OpError {
    match e {
        PolyError::UnsupportedFaceKind { face } => OpError::Invalid(format!("face {face:?} is not an Interior or Boundary face; mesh.agglomerate needs a conformal mesh")),
        other => OpError::Failed(other.to_string()),
    }
}

/// Region (cell class entity) of every cell, densely numbered by first appearance.
fn regions(src: &dyn MeshRead) -> (Vec<u32>, usize) {
    let mut ids: BTreeMap<EntityId, u32> = BTreeMap::new();
    let v = (0..src.count(MeshEntityKind::Cell))
        .map(|c| {
            let n = ids.len() as u32;
            *ids.entry(src.cell_class(CellIx::new(c)).entity).or_insert(n)
        })
        .collect();
    (v, ids.len())
}

type PinKey = (Vec<(String, [i32; 3])>, Option<EntityId>);

fn pin_labels(src: &dyn MeshRead, model: &ModelLayer, group: PinGroup) -> (Vec<u64>, usize, usize) {
    let mut ids: BTreeMap<PinKey, u64> = BTreeMap::new();
    let mut tagged = 0usize;
    let lab = (0..src.count(MeshEntityKind::Cell))
        .map(|c| {
            let vol = src.cell_class(CellIx::new(c)).entity;
            let tag = (group == PinGroup::Lattice).then(|| model.model().region_tag(vol)).flatten().filter(|t| !t.slots.is_empty());
            let key: PinKey = match tag {
                Some(t) => {
                    tagged += 1;
                    (t.slots.iter().map(|(n, s)| (n.clone(), [s.i, s.j, s.k])).collect(), None)
                }
                None => (Vec::new(), Some(vol)),
            };
            let n = ids.len() as u64;
            *ids.entry(key).or_insert(n)
        })
        .collect();
    (lab, ids.len(), tagged)
}

fn group_count(g: &[u32]) -> usize {
    g.iter().copied().max().map_or(0, |m| m as usize + 1)
}

impl MeshAgglomerate {
    fn groups(&self, src: &dyn MeshRead, model: &ModelLayer) -> Result<(Vec<u32>, usize, usize), OpError> {
        let mut dissolved = 0;
        let convex = self.convex != Some(false);
        let gated = match &self.mode {
            AggMode::Target { .. } => true,
            AggMode::PerPin { cells_per_group, .. } => cells_per_group.unwrap_or(1) > 1,
            AggMode::Blocks { .. } => false,
        };
        if !convex && gated {
            return Err(OpError::Invalid("convex: false is not supported for mode target or per_pin with cells_per_group above 1: greedy merging always applies the convex gate".into()));
        }
        let nc = src.count(MeshEntityKind::Cell);
        let g = match &self.mode {
            AggMode::Target { cells } => {
                if *cells == 0 || *cells >= nc {
                    return Err(OpError::Invalid(format!("target {cells} must be between 1 and the {nc} fine cells (exclusive)")));
                }
                let (region, nr) = regions(src);
                coarsen::greedy(src, &region, &vec![1; nr], *cells)
            }
            AggMode::Blocks { size, origin } => {
                if !size.iter().all(|s| s.is_finite() && *s > 0.0) {
                    return Err(OpError::Invalid("block size must be finite and positive".into()));
                }
                let o = origin.unwrap_or([0.0; 3]);
                if !o.iter().all(|x| x.is_finite()) {
                    return Err(OpError::Invalid("block origin must be finite".into()));
                }
                let (region, _) = regions(src);
                let g = coarsen::group_blocks(src, &region, *size, o);
                if convex {
                    let (g, d) = coarsen::dissolve_nonconvex(src, &g, group_count(&g));
                    dissolved = d;
                    g
                } else {
                    g
                }
            }
            AggMode::PerPin { group, cells_per_group } => {
                let k = cells_per_group.unwrap_or(1);
                if k == 0 {
                    return Err(OpError::Invalid("cells_per_group must be at least 1".into()));
                }
                let (lab, np, tagged) = pin_labels(src, model, *group);
                if *group == PinGroup::Lattice && tagged == 0 {
                    return Err(OpError::Invalid("group: lattice needs volumes with a lattice tag (model.lattice_tag); none of the mesh's volumes has one".into()));
                }
                let g = coarsen::components(src, &lab);
                if group_count(&g) != np {
                    return Err(OpError::Failed("a pin group is not face-connected (it has more than one connected part)".into()));
                }
                if k == 1 { g } else { coarsen::greedy(src, &g, &vec![k; np], 0) }
            }
        };
        let n = group_count(&g);
        if n == nc {
            return Err(OpError::Failed("nothing to agglomerate: no group has more than one cell (the convex rule may forbid every merge)".into()));
        }
        Ok((g, n, dissolved))
    }
}

impl TypedOp for MeshAgglomerate {
    const NAME: &'static str = "mesh.agglomerate";
    const OUTPUTS: &'static [OutputRole] = OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let source = ctx.resolve_kind(self.mesh, EntityKind::Mesh)?;
        let (host, selection, plan, quanta, stale, alloc0) = {
            let layer = ctx.layer::<MeshLayer>()?;
            let e = layer.get(source).ok_or(OpError::UnknownEntity(source))?;
            (e.mesh.clone(), e.selection.clone(), e.plan.clone(), e.quanta, e.stale, layer.alloc().clone())
        };
        let model = ctx.layer::<ModelLayer>()?.clone();
        let src = host.as_read();
        if src.dim() != MeshDim::Three {
            return Err(OpError::Invalid(format!("{source} is not a 3D mesh")));
        }
        if (0..src.count(MeshEntityKind::Cell)).any(|c| !src.cell_active(CellIx::new(c))) {
            return Err(OpError::Invalid(format!("{source} has inactive cells")));
        }
        ctx.step(0, 3, "grouping")?;
        let (group_of, n, dissolved) = self.groups(src, &model)?;
        if dissolved > 0 {
            ctx.warn("agglomerate.dissolved", format!("{dissolved} block group(s) failed the convex rule and were left as single cells"), &[source]);
        }
        let report = {
            let mut kids = vec![0usize; n];
            for &x in &group_of {
                kids[x as usize] += 1;
            }
            let merged = kids.iter().filter(|&&k| k > 1).count();
            format!(
                "{{\"fine_cells\":{},\"coarse_cells\":{n},\"merged_groups\":{merged},\"single_cells\":{},\"largest_group\":{},\"dissolved_groups\":{dissolved}}}",
                group_of.len(),
                n - merged,
                kids.iter().copied().max().unwrap_or(0)
            )
        };
        ctx.info("agglomerate.report", report, &[source]);
        ctx.step(1, 3, "building")?;
        let mut alloc: SerialAlloc = alloc0.clone();
        let (fin, face_map) = coarsen::build_coarse(src, &group_of, n, matches!(self.mode, AggMode::PerPin { .. }), MeshBuilder::new(), &mut alloc).map_err(refuse)?;
        let coarse = fin.mesh;
        let v = validate(&coarse);
        if let Some(first) = v.first() {
            return Err(OpError::Failed(format!("the coarse mesh is invalid ({} violation(s)), first: {first:?}", v.len())));
        }
        let g = validate_geometry(&coarse, &GeomTol::default());
        if let Some(first) = g.first() {
            return Err(OpError::Failed(format!("the coarse mesh fails validate_geometry ({} violation(s)), first: {first:?}", g.len())));
        }
        if self.convex != Some(false) {
            let mut kids = vec![0usize; n];
            for &x in &group_of {
                kids[x as usize] += 1;
            }
            if let Some(c) = (0..n).find(|&c| kids[c] > 1 && !sm_quality::cell_convexity(&coarse, CellIx::new(c), sm_quality::RT_PLANARITY_TOL).convex) {
                return Err(OpError::Failed(format!("coarse cell {c} is not convex (set convex: false to accept it)")));
            }
        }
        let map = LevelMap::new(src, &coarse, group_of.iter().map(|&g| CellIx(g)).collect(), Some(face_map)).map_err(|e| OpError::Failed(e.to_string()))?;
        ctx.step(2, 3, "inserting")?;
        let new = coarse.mesh_id();
        let mut entry = MeshEntry::new(Box::new(coarse), selection, None, quanta);
        entry.plan = plan;
        entry.stale = stale;
        entry.derived_from = Some(source);
        insert_mesh(ctx, entry, &alloc0, alloc, "mesh")?;
        if self.keep_source == Some(false) {
            // The link would name a retired mesh, so none is recorded.
            ctx.record_successors(source, &[new])?;
            retire_mesh(ctx, source)?;
        } else {
            ctx.layer_mut::<MeshLayer>()?.link(map).map_err(|e| OpError::Failed(e.to_string()))?;
            ctx.record_successors(source, &[source, new])?;
        }
        Ok(())
    }
}
