//! The `fem.*` ops: blocks are volume colors, sidesets are face colors.
//!
//! Every op that edits the model tables goes through [`model_txn`], so it never opens a
//! topology operation and ids of colors are adopted (and retired) by the layer helpers.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{ModelLayer, model_txn};
use sm_model_api::{Domain, ModelError};
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, TypedOp};

use crate::element::ElementType;
use crate::layer::FemLayer;

fn invalid(e: ModelError) -> OpError {
    OpError::Invalid(e.to_string())
}

/// Resolve `ids` to live ids of `kind`, ascending and distinct.
fn resolve_all(
    ctx: &ApplyCtx<'_>,
    ids: &[EntityId],
    kind: EntityKind,
) -> Result<Vec<EntityId>, OpError> {
    let mut out = ids
        .iter()
        .map(|id| ctx.resolve_kind(*id, kind))
        .collect::<Result<Vec<_>, _>>()?;
    out.sort_unstable();
    out.dedup();
    Ok(out)
}

fn is_empty<T>(v: &[T]) -> bool {
    v.is_empty()
}

const SHELL_REFUSED: &str = "shell blocks (faces) are not supported: W2 meshes hold volume cells only";

const BLOCK_OUT: &[OutputRole] = &[OutputRole::new(
    "block",
    &[EntityKind::Color],
    Cardinality::One,
    "the volume color that is the block",
)];

const SIDESET_OUT: &[OutputRole] = &[OutputRole::new(
    "sideset",
    &[EntityKind::Color],
    Cardinality::One,
    "the face color that is the sideset",
)];

/// `fem.block_add`: create block `block` (a volume color labelled `block`) or extend it with
/// `volumes`.
///
/// The block is the volume's color, and a volume has exactly one color, so adding a volume to a
/// block moves it out of the block it was in (a `block-move` warning names the volume and the
/// old block). Colors are named `block_<n>` unless `name` is given; naming an existing block
/// differently is an error. `faces` (shell blocks) is reserved: W2 meshes hold volume cells
/// only, so a non-empty `faces` is refused.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct BlockAdd {
    /// The block number (the color label); unique among volume colors.
    pub block: u32,
    /// Volumes to put in the block.
    #[serde(default, skip_serializing_if = "is_empty")]
    #[schemars(extend("x-entity-kind" = ["TopoVolume"]))]
    pub volumes: Vec<EntityId>,
    /// Reserved for shell blocks; must be empty.
    #[serde(default, skip_serializing_if = "is_empty")]
    #[schemars(extend("x-entity-kind" = ["TopoFace"]))]
    pub faces: Vec<EntityId>,
    /// Name of the block (default `block_<n>`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
}

impl TypedOp for BlockAdd {
    const NAME: &'static str = "fem.block_add";
    const OUTPUTS: &'static [OutputRole] = BLOCK_OUT;
    const POSITIONAL: Option<&'static str> = Some("block");

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if !self.faces.is_empty() {
            return Err(OpError::Invalid(SHELL_REFUSED.into()));
        }
        let vols = resolve_all(ctx, &self.volumes, EntityKind::TopoVolume)?;
        let (label, name) = (self.block, self.name.clone());
        let (_, moved) = model_txn(
            ctx,
            |model, topo| {
                for v in &vols {
                    if !topo.is_alive(*v) {
                        return Err(OpError::UnknownEntity(*v));
                    }
                }
                let color = match model.color_by_label(Domain::Volume, label) {
                    Some(c) => {
                        if let (Some(want), Some(row)) = (&name, model.color(c))
                            && *want != row.name
                        {
                            return Err(OpError::Invalid(format!(
                                "block {label} is already named '{}', not '{want}'",
                                row.name
                            )));
                        }
                        c
                    }
                    None => {
                        let name = name.clone().unwrap_or_else(|| format!("block_{label}"));
                        model
                            .add_color(label, &name, Domain::Volume)
                            .map_err(invalid)?
                    }
                };
                let moved: Vec<(EntityId, u32)> = vols
                    .iter()
                    .filter_map(|v| {
                        let old = model.volume_color(*v).filter(|o| *o != color)?;
                        Some((*v, model.color(old).map_or(0, |r| r.label)))
                    })
                    .collect();
                model.color_volumes(color, &vols).map_err(invalid)?;
                Ok((color, moved))
            },
            |(color, _), _| vec![("block", vec![*color])],
        )?;
        for (v, old) in moved {
            ctx.warn(
                "block-move",
                format!("{v} moved from block {old} to block {label}"),
                &[v],
            );
        }
        Ok(())
    }
}

/// `fem.block_remove`: delete the empty block `block`.
///
/// Fails while volumes are still in the block: the model keeps an exact volume partition and has
/// no way to uncolor a volume, so move them to another block with `fem.block_add` first.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct BlockRemove {
    /// The block number.
    pub block: u32,
}

impl TypedOp for BlockRemove {
    const NAME: &'static str = "fem.block_remove";
    const OUTPUTS: &'static [OutputRole] = &[];
    const POSITIONAL: Option<&'static str> = Some("block");

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let label = self.block;
        model_txn(
            ctx,
            |model, _| {
                let c = model
                    .color_by_label(Domain::Volume, label)
                    .ok_or_else(|| OpError::Invalid(format!("no block {label}")))?;
                model.remove_color(c).map_err(|e| match e {
                    ModelError::ColorInUse(_) => OpError::Invalid(format!(
                        "block {label} still has volumes; move them to another block first"
                    )),
                    e => invalid(e),
                })
            },
            |_, _| Vec::new(),
        )
    }
}

/// `fem.block_element_type`: declare the element type of block `block`.
///
/// Higher-order types are refused until degree elevation exists, and so are the shell types
/// (`QUAD4`, `TRI3`): shell blocks are unsupported, so such a block could never pass the shape
/// check at export. The declaration is checked
/// against the cell shapes when the mesh is lowered (export), not here, because the block may be
/// meshed later.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct BlockElementType {
    /// The block number.
    pub block: u32,
    /// The element type.
    pub element_type: ElementType,
}

impl TypedOp for BlockElementType {
    const NAME: &'static str = "fem.block_element_type";
    const OUTPUTS: &'static [OutputRole] = &[];
    const POSITIONAL: Option<&'static str> = Some("block");

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if !self.element_type.is_linear() {
            return Err(OpError::Invalid(format!(
                "unsupported element type {}: higher-order elements need degree elevation",
                self.element_type
            )));
        }
        if matches!(self.element_type, ElementType::Quad4 | ElementType::Tri3) {
            return Err(OpError::Invalid(SHELL_REFUSED.into()));
        }
        let color = ctx
            .layer::<ModelLayer>()?
            .model()
            .color_by_label(Domain::Volume, self.block)
            .ok_or_else(|| OpError::Invalid(format!("no block {}", self.block)))?;
        ctx.layer_mut::<FemLayer>()?
            .set_element_type(color, self.element_type);
        Ok(())
    }
}

/// `fem.sideset_add`: create sideset `sideset` (a face color labelled `sideset`) or extend it
/// with `faces`.
///
/// A face is in one sideset (it has one color), so adding it elsewhere moves it (a
/// `sideset-move` warning). `wrt` names the volume whose side of an interior face the sideset
/// takes; every face must be a face of that volume. Without `wrt`, a boundary face lowers to its
/// only side and an interior face to both sides. Sidesets are named `sideset_<n>` unless `name`
/// is given.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SidesetAdd {
    /// The sideset number (the color label); unique among face colors.
    pub sideset: u32,
    /// Faces to put in the sideset.
    #[schemars(extend("x-entity-kind" = ["TopoFace"]))]
    pub faces: Vec<EntityId>,
    /// The volume whose side is meant.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoVolume"]))]
    pub wrt: Option<EntityId>,
    /// Name of the sideset (default `sideset_<n>`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
}

impl TypedOp for SidesetAdd {
    const NAME: &'static str = "fem.sideset_add";
    const OUTPUTS: &'static [OutputRole] = SIDESET_OUT;
    const POSITIONAL: Option<&'static str> = Some("sideset");

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        ctx.layer::<FemLayer>()?;
        let faces = resolve_all(ctx, &self.faces, EntityKind::TopoFace)?;
        let wrt = self
            .wrt
            .map(|v| ctx.resolve_kind(v, EntityKind::TopoVolume))
            .transpose()?;
        let (label, name) = (self.sideset, self.name.clone());
        let moved = model_txn(
            ctx,
            |model, topo| {
                for f in &faces {
                    if !topo.is_alive(*f) {
                        return Err(OpError::UnknownEntity(*f));
                    }
                }
                if let Some(v) = wrt {
                    let mut of_v = topo.volume_faces(v).to_vec();
                    of_v.sort_unstable();
                    if let Some(f) = faces.iter().find(|f| of_v.binary_search(f).is_err()) {
                        return Err(OpError::Invalid(format!("{f} is not a face of {v}")));
                    }
                }
                let color = match model.color_by_label(Domain::Face, label) {
                    Some(c) => {
                        if let (Some(want), Some(row)) = (&name, model.color(c))
                            && *want != row.name
                        {
                            return Err(OpError::Invalid(format!(
                                "sideset {label} is already named '{}', not '{want}'",
                                row.name
                            )));
                        }
                        c
                    }
                    None => {
                        let name = name.clone().unwrap_or_else(|| format!("sideset_{label}"));
                        model
                            .add_color(label, &name, Domain::Face)
                            .map_err(invalid)?
                    }
                };
                let moved: Vec<(EntityId, u32)> = faces
                    .iter()
                    .filter_map(|f| {
                        let old = model.face_color(*f).filter(|o| *o != color)?;
                        Some((*f, model.color(old).map_or(0, |r| r.label)))
                    })
                    .collect();
                model.color_faces(color, &faces).map_err(invalid)?;
                Ok((color, moved))
            },
            |(color, _), _| vec![("sideset", vec![*color])],
        )?
        .1;
        let fem = ctx.layer_mut::<FemLayer>()?;
        for f in &faces {
            fem.set_side_wrt(*f, wrt);
        }
        for (f, old) in moved {
            ctx.warn(
                "sideset-move",
                format!("{f} moved from sideset {old} to sideset {label}"),
                &[f],
            );
        }
        Ok(())
    }
}
