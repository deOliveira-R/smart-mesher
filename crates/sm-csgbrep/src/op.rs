//! `model.import_openmc`: an OpenMC deck becomes B-rep volumes in one history node.

use std::collections::{BTreeMap, BTreeSet};

use base64::Engine as _;
use base64::engine::general_purpose::STANDARD;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_cadio::{MAX_INLINE_BYTES, Source, StepInputs, sha256_hex};
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_csgio::ir::BoundaryFlag;
use sm_fem::{BlockAdd, SidesetAdd};
use sm_geom_api::SurfaceDef;
use sm_layer_api::{commit_topology, model_txn, topo_txn};
use sm_model_api::{LatticeFrame, Layout, RegionTag, SlotIx};
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, TypedOp};

use crate::deck::{self, Placed};
use crate::error::CsgbrepError;
use crate::partition::{self, BuildOptions, OverlapPolicy};

/// Default cap on placed cells.
const DEFAULT_MAX_CELLS: usize = 20_000;

/// Output roles. `deck_volumes` and `deck_bodies` are the result; the rest are the roles of the
/// nested ops the import runs (they hold intermediate ids a later cut or union may have retired).
const OUTPUTS: &[OutputRole] = &[
    OutputRole::new("deck_volumes", &[EntityKind::TopoVolume], Cardinality::Many, "The final volumes of the deck, ascending."),
    OutputRole::new("deck_bodies", &[EntityKind::TopoBody], Cardinality::Many, "The final bodies of the deck, ascending."),
    OutputRole::new("block", &[EntityKind::Color], Cardinality::Many, "The volume colors that are the blocks, one per region name."),
    OutputRole::new("sideset", &[EntityKind::Color], Cardinality::Many, "The face colors that are the sidesets (vacuum, reflective, white)."),
];

/// What to do with a piece of space two cells own.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum OverlapMode {
    /// Fail naming both cells (default).
    #[default]
    Error,
    /// Keep the piece in the first cell (canonical order).
    FirstWins,
    /// As `first_wins`, and warn for each overlap.
    Report,
}

/// Options of `model.import_openmc`; every field is omitted from the hash when it holds the default.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct OpenmcOptions {
    /// Overlap policy (default `error`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub overlap: Option<OverlapMode>,
    /// Bounds of the model `[xmin, ymin, zmin, xmax, ymax, zmax]`; default: derived from the cells.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bounds: Option<[f64; 6]>,
    /// Region names that replace the derived ones, by material id.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub material_names: Option<BTreeMap<u32, String>>,
    /// Cap on placed cells (default 20000).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_cells: Option<usize>,
}

/// `model.import_openmc {geometry, materials?, options?}`: build the B-rep volumes of an OpenMC
/// `geometry.xml` (and `materials.xml`) by space partition.
///
/// `geometry` and `materials` are `{sha256}` or `{inline}` sources like `model.import_step`
/// (never a path). The deck is flattened as the tracker import does (universes, rectangular
/// lattices with their `outer`, transforms), the model box is cut by the plane and cylinder
/// surfaces (a quadric that is a plane or a cylinder included), each piece is classified by the
/// cell regions, and the pieces of one cell that touch are united. Every cut is conformal. Result
/// roles: `deck_volumes` and `deck_bodies` (ascending). Roles `volumes` and `bodies` are not
/// produced. One block per region name (numbered from 1 in name order), one sideset per boundary
/// kind in use (`vacuum` 1, `reflective` 2, `white` 3 when present). Each deck lattice becomes a
/// lattice frame `lat<id>` and each volume inside lattice elements gets a region tag with the
/// deck indices.
///
/// Refused with `csg.unsupported`, naming the deck surface: spheres, cones, tori, elliptic and
/// non-classifiable quadrics (until the sm-boolean tools exist), hex lattices and a structured
/// lattice `outer` (the reader refuses those). Gaps inside the bounds are dropped with an
/// `import.gap` warning; overlaps fail (`csg.overlap`) unless `options.overlap` says otherwise.
/// Diagnostics: `import.partition` (info: kernel cuts, calls, pieces, unions),
/// `import.reflective_curved`, `import.lattice_rotated`, `import.lattice_multi_placement`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ImportOpenmc {
    /// The `geometry.xml`.
    pub geometry: Source,
    /// The `materials.xml` (without it every material id is an unnamed material).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub materials: Option<Source>,
    /// Options.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub options: Option<OpenmcOptions>,
}

fn is_sha256_hex(s: &str) -> bool {
    s.len() == 64 && s.bytes().all(|c| c.is_ascii_digit() || (b'a'..=b'f').contains(&c))
}

/// Resolve a source to text: inline base64 (capped) or a hash read through [`StepInputs`] and
/// re-hashed.
fn load_text(ctx: &ApplyCtx<'_>, source: &Source, what: &str) -> Result<String, OpError> {
    let bytes: Vec<u8> = match source {
        Source::Inline(i) => {
            if i.inline.len() > MAX_INLINE_BYTES.div_ceil(3) * 4 {
                return Err(OpError::Invalid(format!("csg.inline: the inline {what} is over {MAX_INLINE_BYTES} bytes (stage the file and use sha256)")));
            }
            STANDARD.decode(i.inline.as_bytes()).map_err(|e| OpError::Invalid(format!("csg.inline: the inline {what} is not valid base64: {e}")))?
        }
        Source::Sha256(s) => {
            if !is_sha256_hex(&s.sha256) {
                return Err(OpError::Invalid(format!("csg.bad_sha256: `{}` is not 64 lowercase hex digits", s.sha256)));
            }
            let store = ctx.service::<StepInputs>()?;
            let Some(b) = store.0.get(&s.sha256) else {
                return Err(OpError::Invalid(format!("csg.input_missing: input blob {} not found; stage it first", s.sha256)));
            };
            let got = sha256_hex(&b);
            if got != s.sha256 {
                return Err(OpError::Invalid(format!("csg.input_mismatch: the staged blob for {} hashes to {got}", s.sha256)));
            }
            b.to_vec()
        }
    };
    String::from_utf8(bytes).map_err(|_| OpError::Invalid(format!("csg.utf8: the {what} is not UTF-8 text")))
}

/// Frame definitions in parents-first order and the region tags, written in one model-only
/// transaction after the last geometry edit.
fn write_frames_and_tags(ctx: &mut ApplyCtx<'_>, placed: &Placed, built: &partition::Built, bounds: ([f64; 3], [f64; 3])) -> Result<(), OpError> {
    if placed.lattices.is_empty() {
        return Ok(());
    }
    let by_id: BTreeMap<u32, &deck::LatticePlacement> = placed.lattices.iter().map(|l| (l.id, l)).collect();
    // Layout data per lattice from the deck: dims, pitch and lower left come from the placement
    // record, filled in by `Placed::lattice_geometry`.
    let mut frames: Vec<(String, LatticeFrame)> = Vec::new();
    let (lo, hi) = bounds;
    for l in &placed.lattices {
        let g = &l.geometry;
        let mut pitch = [0.0; 3];
        let mut centre = [0.0; 3];
        for a in 0..3 {
            if g.pitch[a].is_finite() {
                pitch[a] = g.pitch[a];
                centre[a] = g.lower_left[a] + 0.5 * g.pitch[a];
            } else {
                // One element along an unbounded axis: a finite pitch that keeps every point of the model in slot 0.
                pitch[a] = hi[a] - lo[a];
                centre[a] = 0.0;
            }
        }
        let mut origin = l.xf.apply_point(Point3::new(centre[0], centre[1], centre[2]));
        let o = [&mut origin.x, &mut origin.y, &mut origin.z];
        for (a, v) in o.into_iter().enumerate() {
            if !g.pitch[a].is_finite() {
                *v = 0.5 * (lo[a] + hi[a]);
            }
        }
        let layout = Layout::Cartesian { n: g.dims, pitch: Vector3::new(pitch[0], pitch[1], pitch[2]) };
        frames.push((format!("lat{}", l.id), LatticeFrame::new(layout, origin, l.parent.map(|p| format!("lat{p}")))));
    }
    // Parents first.
    let depth = |mut id: u32| {
        let mut d = 0;
        while let Some(p) = by_id.get(&id).and_then(|l| l.parent) {
            d += 1;
            id = p;
        }
        d
    };
    let mut order: Vec<usize> = (0..frames.len()).collect();
    order.sort_by_key(|&i| (depth(placed.lattices[i].id), placed.lattices[i].id));

    let mut tags: Vec<(EntityId, RegionTag)> = Vec::new();
    for v in &built.vols {
        let mut slots = BTreeMap::new();
        for h in &placed.cells[v.cell].origin.path {
            if let Some((lid, [i, j, k])) = h.element {
                slots.insert(format!("lat{lid}"), SlotIx::new(i, j, k));
            }
        }
        if !slots.is_empty() {
            tags.push((v.volume, RegionTag::new(slots, None, None)));
        }
    }
    tags.sort_by_key(|t| t.0);
    model_txn(
        ctx,
        |model, _topo| {
            for &i in &order {
                let (name, f) = &frames[i];
                model.define_lattice_frame(name, f.clone()).map_err(|e| OpError::Failed(format!("csg.frame: {e}")))?;
            }
            for (v, t) in tags {
                model.set_region_tag(v, t).map_err(|e| OpError::Failed(format!("csg.tag: {e}")))?;
            }
            Ok(())
        },
        |_, _| Vec::new(),
    )
}

impl TypedOp for ImportOpenmc {
    const NAME: &'static str = "model.import_openmc";
    const OUTPUTS: &'static [OutputRole] = OUTPUTS;
    /// Bump when the builder changes the cells it creates for the same deck.
    const IMPL_VERSION: u32 = 1;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let geometry = load_text(ctx, &self.geometry, "geometry.xml")?;
        let materials = self.materials.as_ref().map(|m| load_text(ctx, m, "materials.xml")).transpose()?;
        let ir = sm_csgio::openmc::import(&geometry, materials.as_deref()).map_err(CsgbrepError::from)?;
        let o = self.options.clone().unwrap_or_default();
        if let Some(b) = o.bounds
            && !(b.iter().all(|v| v.is_finite()) && b[0] < b[3] && b[1] < b[4] && b[2] < b[5])
        {
            return Err(OpError::Invalid(format!("csg.invalid: options.bounds {b:?} is not a finite box")));
        }
        let placed = deck::place(&ir, &o.material_names.clone().unwrap_or_default(), o.max_cells.unwrap_or(DEFAULT_MAX_CELLS))?;
        for s in &placed.surfaces {
            if s.flag == BoundaryFlag::Reflective && !matches!(s.def, SurfaceDef::Plane(_)) {
                ctx.warn("import.reflective_curved", format!("deck surface {} is reflective but not planar; reflection is exact on planes only", s.deck_id), &[]);
            }
        }
        for l in &placed.lattices {
            if l.rotated {
                ctx.warn("import.lattice_rotated", format!("lattice {} is placed with a rotation: its frame cannot describe it, the region tags come from the deck indices", l.id), &[]);
            }
            if l.placements > 1 {
                let msg = format!("lattice {} is placed {} times; its frame describes the first placement, the tags carry the deck indices", l.id, l.placements);
                if l.parents_differ {
                    ctx.warn("import.lattice_multi_placement", msg, &[]);
                } else {
                    ctx.info("import.lattice_multi_placement", msg, &[]);
                }
            }
        }
        let policy = match o.overlap.unwrap_or_default() {
            OverlapMode::Error => OverlapPolicy::Error,
            OverlapMode::FirstWins => OverlapPolicy::FirstWins,
            OverlapMode::Report => OverlapPolicy::Report,
        };
        let bopts = BuildOptions { overlap: policy, bounds: o.bounds.map(|b| ([b[0], b[1], b[2]], [b[3], b[4], b[5]])) };
        let (built, txn) = topo_txn(ctx, |topo, _model| partition::build(topo, &placed, &bopts).map_err(OpError::from))?;
        let volumes: Vec<EntityId> = built.vols.iter().map(|v| v.volume).collect();
        let bodies: Vec<EntityId> = built.vols.iter().map(|v| v.body).collect();
        commit_topology(ctx, &txn, &[("deck_volumes", &volumes), ("deck_bodies", &bodies)])?;
        let s = built.stats;
        ctx.info("import.partition", format!("{} kernel cuts in {} webcut calls, {} pieces, {} unions, {} volumes", s.cuts, s.cut_calls, s.pieces, s.unions, volumes.len()), &[]);
        for n in &built.notes {
            ctx.warn(n.code, n.message.clone(), &[]);
        }

        // Blocks: one per region name, numbered in name order.
        let mut by_name: BTreeMap<&str, Vec<EntityId>> = BTreeMap::new();
        for v in &built.vols {
            by_name.entry(placed.cells[v.cell].name.as_str()).or_default().push(v.volume);
        }
        for (n, (name, vols)) in by_name.iter().enumerate() {
            TypedOp::apply(&BlockAdd { block: n as u32 + 1, volumes: vols.clone(), faces: Vec::new(), name: Some((*name).to_owned()) }, ctx)?;
        }
        // Sidesets: vacuum 1, reflective 2, white 3, when present.
        for (k, name) in ["vacuum", "reflective", "white"].into_iter().enumerate() {
            if !built.boundary_faces[k].is_empty() {
                TypedOp::apply(&SidesetAdd { sideset: k as u32 + 1, faces: built.boundary_faces[k].clone(), wrt: None, name: Some(name.to_owned()) }, ctx)?;
            }
        }
        let names: BTreeSet<&str> = by_name.keys().copied().collect();
        ctx.info("import.regions", format!("{} region name(s): {}", names.len(), names.into_iter().collect::<Vec<_>>().join(", ")), &[]);
        write_frames_and_tags(ctx, &placed, &built, built.start)?;
        Ok(())
    }
}
