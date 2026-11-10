//! Lattice frames and region tags as ops (ruling D9): `model.lattice_define`,
//! `model.lattice_tag`, `model.region_clear` and the query `model.region_tags`.
//!
//! The tables live in the [`Model`] (`sm-model-api` `regions.rs`); these ops are the writers.
//! Tagging is geometric: a volume's slot in a frame is the slot nearest to its centroid
//! ([`LatticeFrame::locate`]), so it works for any construction (`model.copy` lattices, STEP
//! assemblies, imported decks). A centroid outside the layout, or within the model's linear
//! tolerance of a boundary between two slots, is an error naming the volume (or is skipped with
//! `outside: "skip"`).

use std::collections::{BTreeMap, BTreeSet};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_layer_api::{LayerResultExt, ModelLayer, model_txn};
use sm_model_api::{HexOrient, LatticeFrame, Layout, Model, RegionTag, SlotIx, SubIndex};
use sm_op_api::{ApplyCtx, OpError, QueryCtx, State, TypedOp, TypedQuery};
use sm_topo_api::TopologyView;

use crate::queries::volume_value;

/// The slot geometry of a frame, as op parameters.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum LayoutSpec {
    /// `n[0] x n[1] x n[2]` slots from index 0; slot `(i, j, k)` is centred at
    /// `origin + (i px, j py, k pz)`.
    Cartesian {
        /// Slot counts per axis (2 or 3 values; a missing z count is 1).
        n: Vec<u32>,
        /// Centre spacing per axis (2 or 3 values). A missing z pitch is 0, allowed only with
        /// one z slot (every z lands in it); with several z slots it is required.
        pitch: Vec<f64>,
    },
    /// Hex lattice in axial `(q, r)` coordinates, `rings` rings (1 = one slot), `nz` layers.
    Hex {
        /// Number of rings, at least 1.
        rings: u32,
        /// Centre-to-centre distance in the plane.
        pitch: f64,
        /// Row direction: `x_rows` or `y_rows`.
        orient: HexOrientSpec,
        /// Number of axial layers (default 1).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        nz: Option<u32>,
        /// Axial spacing. Defaults to 0 for a single layer; required when `nz` > 1.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        dz: Option<f64>,
    },
}

/// Hex row orientation as a parameter.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum HexOrientSpec {
    /// Rows along x.
    XRows,
    /// Rows along y.
    YRows,
}

fn axes(what: &str, v: &[f64], z_default: f64) -> Result<[f64; 3], OpError> {
    match v {
        [x, y] => Ok([*x, *y, z_default]),
        [x, y, z] => Ok([*x, *y, *z]),
        _ => Err(OpError::Invalid(format!("`{what}` needs 2 or 3 values, got {}", v.len()))),
    }
}

impl LayoutSpec {
    fn to_layout(&self) -> Result<Layout, OpError> {
        match self {
            LayoutSpec::Cartesian { n, pitch } => {
                let nf: Vec<f64> = n.iter().map(|&x| f64::from(x)).collect();
                let n = axes("n", &nf, 1.0)?.map(|x| x as u32);
                if pitch.len() == 2 && n[2] > 1 {
                    return Err(OpError::Invalid("`pitch` needs a z value when there is more than one z slot".into()));
                }
                let p = axes("pitch", pitch, 0.0)?;
                if p.iter().any(|x| !x.is_finite() || *x < 0.0) || p[0] <= 0.0 || p[1] <= 0.0 {
                    return Err(OpError::Invalid("`pitch` must be finite, with positive x and y".into()));
                }
                Ok(Layout::Cartesian { n, pitch: Vector3::new(p[0], p[1], p[2]) })
            }
            LayoutSpec::Hex { rings, pitch, orient, nz, dz } => {
                if !(pitch.is_finite() && *pitch > 0.0) {
                    return Err(OpError::Invalid("hex `pitch` must be positive".into()));
                }
                let orient = match orient {
                    HexOrientSpec::XRows => HexOrient::XRows,
                    HexOrientSpec::YRows => HexOrient::YRows,
                };
                let nz = nz.unwrap_or(1);
                if dz.is_none() && nz > 1 {
                    return Err(OpError::Invalid("hex `dz` is required when `nz` > 1".into()));
                }
                Ok(Layout::Hex { rings: *rings, pitch: *pitch, orient, nz, dz: dz.unwrap_or(0.0) })
            }
        }
    }

    fn from_layout(l: &Layout) -> Option<Self> {
        Some(match *l {
            Layout::Cartesian { n, pitch } => LayoutSpec::Cartesian { n: n.to_vec(), pitch: vec![pitch.x, pitch.y, pitch.z] },
            Layout::Hex { rings, pitch, orient, nz, dz } => LayoutSpec::Hex {
                rings,
                pitch,
                orient: if orient == HexOrient::XRows { HexOrientSpec::XRows } else { HexOrientSpec::YRows },
                nz: Some(nz),
                dz: Some(dz),
            },
            _ => return None,
        })
    }
}

/// `model.lattice_define {name, layout, origin?, parent?}`: define a lattice frame, the
/// provenance geometry that `model.lattice_tag` and `model.copy {lattice}` tag volumes against.
///
/// Slot `(0, 0, 0)` is centred at `origin` (default the world origin). `parent` names an
/// enclosing frame (core > assembly > pin) that must exist already. Names match
/// `[A-Za-z0-9_]+` because they appear in region names. A frame is provenance only: it makes no
/// copies and shares no memory.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct LatticeDefine {
    /// The frame name.
    pub name: String,
    /// The slot geometry.
    pub layout: LayoutSpec,
    /// Centre of slot `(0, 0, 0)` (default the origin).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub origin: Option<[f64; 3]>,
    /// The enclosing frame, if any.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub parent: Option<String>,
}

impl TypedOp for LatticeDefine {
    const NAME: &'static str = "model.lattice_define";
    const NO_OUTPUTS: bool = true;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let layout = self.layout.to_layout()?;
        let o = self.origin.unwrap_or([0.0; 3]);
        let frame = LatticeFrame::new(layout, Point3::new(o[0], o[1], o[2]), self.parent.clone());
        model_txn(ctx, |m, _t| m.define_lattice_frame(&self.name, frame).or_op(), |_, _| Vec::new())
    }
}

/// What `model.lattice_tag` does with a volume whose centroid has no unique slot.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Outside {
    /// Fail, naming the volume (default).
    #[default]
    Error,
    /// Leave the volume without a slot in this frame.
    Skip,
}

/// `model.lattice_tag {lattice, volumes, outside?}`: set each volume's slot in frame `lattice`
/// from its centroid. Other frames' slots and ring and sector indices of the volume are kept.
///
/// The slot is the one nearest to the centroid ([`LatticeFrame::locate`]). A centroid outside the
/// layout, or within the model's linear tolerance of the boundary between two slots, is an error
/// naming the volume (`outside: "skip"` leaves such volumes untagged instead). A full annulus
/// has its centroid on its axis, the pin centre, so it lands in the right slot.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct LatticeTag {
    /// The frame name.
    pub lattice: String,
    /// The volumes to tag.
    #[schemars(extend("x-entity-kind" = ["TopoVolume"]))]
    pub volumes: Vec<EntityId>,
    /// Policy for a centroid outside the layout or on a slot boundary (default `error`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub outside: Option<Outside>,
}

/// Why a point has no unique slot.
enum Miss {
    Outside,
    Boundary,
}

/// The slot of `p`, or why there is none: outside the layout, or within `tol` of a slot border.
fn locate_strict(frame: &LatticeFrame, p: Point3, tol: f64) -> Result<SlotIx, Miss> {
    let ix = frame.locate(p).ok_or(Miss::Outside)?;
    let t = tol;
    for (dx, dy, dz) in [(t, 0.0, 0.0), (-t, 0.0, 0.0), (0.0, t, 0.0), (0.0, -t, 0.0), (0.0, 0.0, t), (0.0, 0.0, -t), (t, t, 0.0), (t, -t, 0.0), (-t, t, 0.0), (-t, -t, 0.0)] {
        if frame.locate(Point3::new(p.x + dx, p.y + dy, p.z + dz)) != Some(ix) {
            return Err(Miss::Boundary);
        }
    }
    Ok(ix)
}

/// Tag `vols` in `frame_name` from their centroids (shared by `model.lattice_tag` and
/// `model.copy {lattice}`).
pub(crate) fn tag_volumes(m: &mut Model, topo: &dyn TopologyView, frame_name: &str, vols: &[EntityId], outside: Outside) -> Result<(), OpError> {
    let frame = m.lattice_frame(frame_name).cloned().ok_or_else(|| OpError::Invalid(format!("unknown lattice frame `{frame_name}` (model.lattice_define first)")))?;
    let tol = topo.tolerance().linear.max(1e-12);
    let mut slots: Vec<(EntityId, SlotIx)> = Vec::with_capacity(vols.len());
    for &v in vols {
        if v.kind() != EntityKind::TopoVolume {
            return Err(OpError::WrongKind { id: v, expected: EntityKind::TopoVolume });
        }
        if !topo.is_alive(v) {
            return Err(OpError::UnknownEntity(v));
        }
        let (_, c) = volume_value(topo, v)?;
        let c = c.ok_or_else(|| OpError::Failed(format!("lattice_tag: volume {v} has no centroid")))?;
        match locate_strict(&frame, c, tol) {
            Ok(ix) => slots.push((v, ix)),
            Err(miss) => {
                if outside == Outside::Skip {
                    continue;
                }
                let why = match miss {
                    Miss::Outside => "outside the layout",
                    Miss::Boundary => "within the model tolerance of a slot boundary",
                };
                return Err(OpError::Failed(format!("lattice_tag: centroid of volume {v} ({:.6}, {:.6}, {:.6}) is {why} of frame `{frame_name}`", c.x, c.y, c.z)));
            }
        }
    }
    for (v, ix) in slots {
        let mut tag = m.region_tag(v).cloned().unwrap_or_default();
        tag.slots.insert(frame_name.to_string(), ix);
        m.set_region_tag(v, tag).or_op()?;
    }
    Ok(())
}

impl TypedOp for LatticeTag {
    const NAME: &'static str = "model.lattice_tag";
    const NO_OUTPUTS: bool = true;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if self.volumes.is_empty() {
            return Err(OpError::Invalid("`volumes` is empty".into()));
        }
        for &v in &self.volumes {
            ctx.resolve_kind(v, EntityKind::TopoVolume)?;
        }
        let vols: BTreeSet<EntityId> = self.volumes.iter().copied().collect();
        let vols: Vec<EntityId> = vols.into_iter().collect();
        model_txn(ctx, |m, t| tag_volumes(m, t, &self.lattice, &vols, self.outside.unwrap_or_default()), |_, _| Vec::new())
    }
}

/// `model.region_clear {volumes?}`: drop region tags and lattice frames.
///
/// With `volumes`, only those volumes lose their tag (frames stay). Without, every tag and every
/// frame is dropped and the model's tables are as if no frame had ever been defined (an empty
/// model hashes as before).
#[derive(Debug, Default, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RegionClear {
    /// Volumes whose tags to drop (default: everything, frames included).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoVolume"]))]
    pub volumes: Option<Vec<EntityId>>,
}

/// A model without lattice frames and region tags: the persisted form with both tables removed.
fn without_regions(m: &Model) -> Result<Model, OpError> {
    let mut v = serde_json::to_value(m).map_err(|e| OpError::Failed(format!("region_clear: {e}")))?;
    if let Some(o) = v.as_object_mut() {
        o.remove("lattice_frames");
        o.remove("region_tags");
    }
    serde_json::from_value(v).map_err(|e| OpError::Failed(format!("region_clear: {e}")))
}

impl TypedOp for RegionClear {
    const NAME: &'static str = "model.region_clear";
    const NO_OUTPUTS: bool = true;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        match &self.volumes {
            Some(vols) => {
                for &v in vols {
                    ctx.resolve_kind(v, EntityKind::TopoVolume)?;
                }
                model_txn(
                    ctx,
                    |m, _t| {
                        for &v in vols {
                            m.clear_region_tag(v);
                        }
                        Ok(())
                    },
                    |_, _| Vec::new(),
                )
            }
            None => model_txn(
                ctx,
                |m, _t| {
                    if m.lattice_frames().is_empty() && m.region_tags().is_empty() {
                        return Ok(());
                    }
                    *m = without_regions(m)?;
                    Ok(())
                },
                |_, _| Vec::new(),
            ),
        }
    }
}

/// One lattice frame in the result of `model.region_tags`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct FrameRow {
    /// Frame name.
    pub name: String,
    /// Slot geometry.
    pub layout: LayoutSpec,
    /// Centre of slot `(0, 0, 0)`.
    pub origin: [f64; 3],
    /// Enclosing frame.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub parent: Option<String>,
}

/// One tagged volume in the result of `model.region_tags`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct TagRow {
    /// The volume.
    pub volume: EntityId,
    /// Slot `[i, j, k]` per frame name.
    pub slots: BTreeMap<String, [i32; 3]>,
    /// `[index, count]` of the ring (0 innermost).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ring: Option<[u32; 2]>,
    /// `[index, count]` of the sector.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sector: Option<[u32; 2]>,
}

/// The result of `model.region_tags`.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize, JsonSchema)]
pub struct RegionTable {
    /// Frames, by name.
    pub frames: Vec<FrameRow>,
    /// Tagged volumes, ascending.
    pub tags: Vec<TagRow>,
}

/// `model.region_tags {volumes?}`: the lattice frames and the region tags (of `volumes`, or of
/// every tagged volume). Read-only. Consumers read region indices from here, never from the
/// output order of an op.
#[derive(Debug, Default, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RegionTags {
    /// Only these volumes (default every tagged volume; an untagged volume is left out).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoVolume"]))]
    pub volumes: Option<Vec<EntityId>>,
}

fn row(v: EntityId, t: &RegionTag) -> TagRow {
    let sub = |s: Option<SubIndex>| s.map(|s| [s.index, s.count]);
    TagRow { volume: v, slots: t.slots.iter().map(|(k, ix)| (k.clone(), [ix.i, ix.j, ix.k])).collect(), ring: sub(t.ring), sector: sub(t.sector) }
}

impl TypedQuery for RegionTags {
    const NAME: &'static str = "model.region_tags";
    type Output = RegionTable;

    fn run(&self, state: &State, _cx: &QueryCtx<'_>) -> Result<RegionTable, OpError> {
        let layer = state.layer::<ModelLayer>().ok_or_else(|| OpError::MissingLayer(<ModelLayer as sm_op_api::LayerType>::NAME.to_owned()))?;
        let m = layer.model();
        let frames = m
            .lattice_frames()
            .iter()
            .map(|(name, f)| {
                let layout = LayoutSpec::from_layout(&f.layout).ok_or_else(|| OpError::Unsupported(format!("frame `{name}` has a layout this build cannot describe")))?;
                Ok(FrameRow { name: name.clone(), layout, origin: [f.origin.x, f.origin.y, f.origin.z], parent: f.parent.clone() })
            })
            .collect::<Result<Vec<_>, OpError>>()?;
        let tags = match &self.volumes {
            None => m.region_tags().iter().map(|(v, t)| row(*v, t)).collect(),
            Some(vs) => {
                let set: BTreeSet<EntityId> = vs.iter().copied().collect();
                set.into_iter().filter_map(|v| m.region_tag(v).map(|t| row(v, t))).collect()
            }
        };
        Ok(RegionTable { frames, tags })
    }
}
