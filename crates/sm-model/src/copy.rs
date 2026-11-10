//! `model.copy`: deep copies of volumes or bodies, optionally transformed (the Cubit "repeat"
//! pattern).
//!
//! The downward closure of the selection is copied cell by cell (ascending ids, so the new ids
//! are deterministic) into fresh vertices, edges, faces and volumes, with their own curve and
//! surface handles, then moved by the composed transform. Sharing *inside* the selection is
//! preserved (two merged bricks copy as two volumes sharing one face); sharing with unselected
//! volumes is not, so copying one volume of a merged pair is legal. Meshes, colors, groups,
//! boundary kinds, decks and decomposition records are not copied (a later
//! `model.copy_attributes`), and a copy has no lineage link to its source beyond `Created`.

use std::collections::{BTreeMap, BTreeSet};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind, Transform};
use sm_geom_api::{CurveDef, PCurve, SurfaceDef};
use sm_layer_api::{ModelLayer, TopoHost, commit_topology, model_txn, topo_txn};
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, TypedOp};
use sm_topo_api::{Carrier, CellFlags, CoedgeKey, CoedgeSpec, CurveCarrier, FaceUseSpec, Sense, ShellKind, ShellSpec, Side, SurfaceCarrier, TopoError, TopologyView, TopologyViewExt};

use crate::region::{Outside, tag_volumes};
use crate::xform::{Xf, XformError, move_xf, reflect_xf, scale_xf, transform_cells};
use crate::xform2::{AxisSpec, RotationForm, resolve_rotation};

const MAX_COPIES: u32 = 4096;

const OUTPUTS: &[OutputRole] = &[
    OutputRole::new("volumes", &[EntityKind::TopoVolume], Cardinality::Many, "The new volumes, ascending."),
    OutputRole::new("bodies", &[EntityKind::TopoBody], Cardinality::Many, "The new bodies, ascending."),
];

/// The transform applied once per copy by `model.copy`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum CopyXform {
    /// Translate by `delta`.
    Move {
        /// Translation vector.
        delta: [f64; 3],
    },
    /// Rotate; exactly one of `axis`, `vertices`, `face` (see `model.rotate_about`).
    Rotate {
        /// Angle in degrees.
        angle_deg: f64,
        /// The axis direction.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        axis: Option<AxisSpec>,
        /// A point on the axis (or the pivot override).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        origin: Option<[f64; 3]>,
        /// Two vertices that define the axis.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        #[schemars(extend("x-entity-kind" = ["TopoVertex"]))]
        vertices: Option<[EntityId; 2]>,
        /// A face whose normal is the axis.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        #[schemars(extend("x-entity-kind" = ["TopoFace"]))]
        face: Option<EntityId>,
    },
    /// Uniform scale about `origin` (default the world origin).
    Scale {
        /// Scale factor (`> 0`).
        factor: f64,
        /// Fixed point.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        origin: Option<[f64; 3]>,
    },
    /// Mirror in the plane through `origin` with normal `normal`.
    Reflect {
        /// Plane normal (not necessarily unit).
        normal: [f64; 3],
        /// A point on the plane (default the origin).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        origin: Option<[f64; 3]>,
    },
}

/// `model.copy`: `count` copies of the selected volumes or bodies.
///
/// Copy `k` (1-based) is the source transformed by `xform` applied `k` times; without `xform`
/// the copies coincide with the source. Each selected body gets one new body holding its volumes'
/// copies; a volume selected on its own gets one new body per copy. Copies carry no names,
/// colors, groups, boundary kinds, decks, meshes or decomposition records (and no region tags,
/// except the lattice slots `lattice` assigns). Meshed and merged
/// volumes may be copied (the copy owns its cells). `count` is `1..=4096` (default 1).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ModelCopy {
    /// Volumes, or bodies standing for their members.
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoBody"]))]
    pub entities: Vec<EntityId>,
    /// Number of copies (default 1).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub count: Option<u32>,
    /// The per-copy transform (default none).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub xform: Option<CopyXform>,
    /// A lattice frame (`model.lattice_define`): after copying, the source and the copies are
    /// tagged with their slot in that frame from their centroids, as `model.lattice_tag` does
    /// (default none; copies carry no tag otherwise).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lattice: Option<String>,
}

fn bad(e: TopoError) -> OpError {
    OpError::Failed(format!("copy: {e}"))
}

/// Everything read from the source closure, so each copy builds without touching the source.
struct Snapshot {
    vertices: Vec<(EntityId, sm_core::Point3, CellFlags, Option<f64>)>,
    edges: Vec<EdgeRec>,
    faces: Vec<FaceRec>,
    volumes: Vec<VolumeRec>,
    curves: BTreeMap<EntityId, CurveDef>,
    surfaces: BTreeMap<EntityId, SurfaceDef>,
}

struct EdgeRec {
    id: EntityId,
    ends: (EntityId, EntityId),
    carrier: Option<CurveCarrier>,
    flags: CellFlags,
    tol: Option<f64>,
}

struct FaceRec {
    id: EntityId,
    loops: Vec<Vec<(EntityId, Side)>>,
    carrier: Option<SurfaceCarrier>,
    pcurves: Vec<(CoedgeKey, PCurve)>,
    flags: CellFlags,
    tol: Option<f64>,
}

struct VolumeRec {
    id: EntityId,
    shells: Vec<(ShellKind, Vec<FaceUseSpec>)>,
    flags: CellFlags,
    tol: Option<f64>,
}

fn sense_of(side: Side) -> Sense {
    if side == Side::Left { Sense::Forward } else { Sense::Reversed }
}

fn snapshot(topo: &dyn TopologyView, closure: &BTreeSet<EntityId>) -> Result<Snapshot, OpError> {
    let mut snap = Snapshot { vertices: Vec::new(), edges: Vec::new(), faces: Vec::new(), volumes: Vec::new(), curves: BTreeMap::new(), surfaces: BTreeMap::new() };
    for &c in closure {
        let (flags, tol) = (topo.flags(c), topo.tolerance_override(c));
        match c.kind() {
            EntityKind::TopoVertex => match topo.carrier(c) {
                Carrier::Point(p) => snap.vertices.push((c, *p, flags, tol)),
                _ => return Err(OpError::Failed(format!("copy: vertex {c} has no point"))),
            },
            EntityKind::TopoEdge => {
                let ends = topo.edge_ends(c).ok_or_else(|| OpError::Failed(format!("copy: edge {c} has no end vertices")))?;
                let carrier = match topo.carrier(c) {
                    Carrier::Curve(cc) => {
                        let def = topo.shapes().curve_def(cc.curve).ok_or_else(|| OpError::Failed(format!("copy: curve {} of {c} is not in the store", cc.curve)))?;
                        snap.curves.insert(cc.curve, def.clone());
                        Some(*cc)
                    }
                    Carrier::None => None,
                    other => return Err(OpError::Failed(format!("copy: edge {c} has an unsupported carrier {other:?}"))),
                };
                snap.edges.push(EdgeRec { id: c, ends, carrier, flags, tol });
            }
            EntityKind::TopoFace => {
                let loops: Vec<Vec<(EntityId, Side)>> = topo.loops(c).map(|l| l.entries.iter().map(|e| (e.cell, e.side)).collect()).collect();
                let carrier = match topo.carrier(c) {
                    Carrier::Surface(sc) => {
                        let def = topo.shapes().surface_def(sc.surface).ok_or_else(|| OpError::Failed(format!("copy: surface {} of {c} is not in the store", sc.surface)))?;
                        snap.surfaces.insert(sc.surface, def.clone());
                        Some(*sc)
                    }
                    Carrier::None => None,
                    other => return Err(OpError::Failed(format!("copy: face {c} has an unsupported carrier {other:?}"))),
                };
                let pcurves = topo.coedges(c).filter_map(|co| topo.pcurve(co.key).map(|p| (co.key, p.clone()))).collect();
                snap.faces.push(FaceRec { id: c, loops, carrier, pcurves, flags, tol });
            }
            EntityKind::TopoVolume => {
                let shells = topo
                    .shells(c)
                    .map(|s| (topo.shell_kind(c, s.ring).unwrap_or(ShellKind::Outer), s.entries.iter().map(|e| FaceUseSpec { face: e.cell, side: e.side }).collect()))
                    .collect();
                snap.volumes.push(VolumeRec { id: c, shells, flags, tol });
            }
            _ => {}
        }
    }
    Ok(snap)
}

/// Build one copy of the snapshot; returns the old -> new map of every cell.
fn build_copy(topo: &mut dyn TopoHost, snap: &Snapshot) -> Result<BTreeMap<EntityId, EntityId>, OpError> {
    let mut map: BTreeMap<EntityId, EntityId> = BTreeMap::new();
    let mut curve_map: BTreeMap<EntityId, EntityId> = BTreeMap::new();
    let mut surface_map: BTreeMap<EntityId, EntityId> = BTreeMap::new();
    for (&h, def) in &snap.curves {
        curve_map.insert(h, topo.shapes_mut().map_err(bad)?.add_curve(def.clone()).map_err(|e| OpError::Failed(format!("copy: {e}")))?);
    }
    for (&h, def) in &snap.surfaces {
        surface_map.insert(h, topo.shapes_mut().map_err(bad)?.add_surface(def.clone()).map_err(|e| OpError::Failed(format!("copy: {e}")))?);
    }
    let finish = |topo: &mut dyn TopoHost, new: EntityId, flags: CellFlags, tol: Option<f64>| -> Result<(), OpError> {
        if flags != CellFlags::NONE {
            topo.set_flags(new, flags).map_err(bad)?;
        }
        if let Some(t) = tol {
            topo.raise_tolerance(new, t).map_err(bad)?;
        }
        topo.record_created(new).map_err(bad)
    };
    for &(old, p, flags, tol) in &snap.vertices {
        let new = topo.make_vertex(p).map_err(bad)?;
        finish(topo, new, flags, tol)?;
        map.insert(old, new);
    }
    for e in &snap.edges {
        let carrier = e.carrier.map(|cc| CurveCarrier { curve: curve_map[&cc.curve], ..cc });
        let new = topo.make_edge(map[&e.ends.0], map[&e.ends.1], carrier).map_err(bad)?;
        finish(topo, new, e.flags, e.tol)?;
        map.insert(e.id, new);
    }
    for f in &snap.faces {
        let specs: Vec<Vec<CoedgeSpec>> = f.loops.iter().map(|l| l.iter().map(|&(e, side)| CoedgeSpec { edge: map[&e], sense: sense_of(side) }).collect()).collect();
        let refs: Vec<&[CoedgeSpec]> = specs.iter().map(Vec::as_slice).collect();
        let carrier = f.carrier.map(|sc| SurfaceCarrier { surface: surface_map[&sc.surface], ..sc });
        let new = topo.make_face(&refs, carrier).map_err(bad)?;
        for (k, pc) in &f.pcurves {
            topo.set_pcurve(CoedgeKey { face: new, edge: map[&k.edge], side: k.side }, Some(pc.clone())).map_err(bad)?;
        }
        finish(topo, new, f.flags, f.tol)?;
        map.insert(f.id, new);
    }
    for v in &snap.volumes {
        let uses: Vec<Vec<FaceUseSpec>> = v.shells.iter().map(|(_, u)| u.iter().map(|u| FaceUseSpec { face: map[&u.face], side: u.side }).collect()).collect();
        let shells: Vec<ShellSpec<'_>> = v.shells.iter().zip(&uses).map(|((kind, _), u)| ShellSpec { kind: *kind, uses: u }).collect();
        let new = topo.make_volume(&shells).map_err(bad)?;
        finish(topo, new, v.flags, v.tol)?;
        map.insert(v.id, new);
    }
    Ok(map)
}

/// The per-copy similarity (before powers).
fn resolve_xform(ctx: &ApplyCtx<'_>, x: &CopyXform) -> Result<Xf, OpError> {
    Ok(match x {
        CopyXform::Move { delta } => move_xf(*delta)?,
        CopyXform::Rotate { angle_deg, axis, origin, vertices, face } => resolve_rotation(ctx, &RotationForm { angle_deg: *angle_deg, axis: *axis, origin: *origin, vertices: *vertices, face: *face })?,
        CopyXform::Scale { factor, origin } => scale_xf(*factor, *origin)?,
        CopyXform::Reflect { normal, origin } => reflect_xf(*normal, *origin)?,
    })
}

/// `xf` applied `k` times.
fn power(xf: &Xf, k: u32) -> Xf {
    let mut t = Transform::IDENTITY;
    for _ in 0..k {
        t = xf.t.compose(&t);
    }
    Xf { t, scale: xf.scale.powi(k as i32), improper: xf.improper && k % 2 == 1 }
}

impl TypedOp for ModelCopy {
    const NAME: &'static str = "model.copy";
    const OUTPUTS: &'static [OutputRole] = OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let count = self.count.unwrap_or(1);
        if !(1..=MAX_COPIES).contains(&count) {
            return Err(OpError::Invalid(format!("`count` must be in 1..={MAX_COPIES}, got {count}")));
        }
        if self.entities.is_empty() {
            return Err(XformError::EmptySelection.into());
        }
        if let Some(name) = &self.lattice
            && ctx.layer::<ModelLayer>()?.model().lattice_frame(name).is_none()
        {
            return Err(OpError::Invalid(format!("unknown lattice frame `{name}` (model.lattice_define first)")));
        }
        let xf = self.xform.as_ref().map(|x| resolve_xform(ctx, x)).transpose()?;
        if let Some(xf) = &xf {
            xf.check_similarity()?;
        }
        // Groups of volumes that share one new body per copy: each selected body, then each
        // selected volume not already covered by a selected body.
        let (groups, snap) = {
            let topo = ctx.layer::<ModelLayer>()?.topo();
            let mut bodies: BTreeSet<EntityId> = BTreeSet::new();
            let mut loose: BTreeSet<EntityId> = BTreeSet::new();
            for &e in &self.entities {
                ctx.require(e)?;
                match e.kind() {
                    EntityKind::TopoBody => {
                        bodies.insert(e);
                    }
                    EntityKind::TopoVolume => {
                        loose.insert(e);
                    }
                    _ => return Err(OpError::WrongKind { id: e, expected: EntityKind::TopoVolume }),
                }
            }
            let mut groups: Vec<Vec<EntityId>> = Vec::new();
            let mut covered: BTreeSet<EntityId> = BTreeSet::new();
            for &b in &bodies {
                let vols: Vec<EntityId> = topo.boundary(b).iter().map(|i| i.cell).filter(|c| c.kind() == EntityKind::TopoVolume).collect();
                covered.extend(&vols);
                groups.push(vols);
            }
            for &v in loose.difference(&covered) {
                groups.push(vec![v]);
            }
            let mut closure: BTreeSet<EntityId> = BTreeSet::new();
            let mut stack: Vec<EntityId> = groups.iter().flatten().copied().collect();
            while let Some(c) = stack.pop() {
                if !closure.insert(c) {
                    continue;
                }
                if c.kind() != EntityKind::TopoVertex {
                    stack.extend(topo.boundary(c).iter().map(|i| i.cell));
                }
            }
            (groups, snapshot(topo, &closure)?)
        };
        let ((vols, bodies), txn) = topo_txn(ctx, |topo, _model| {
            let (mut vols, mut bodies) = (Vec::new(), Vec::new());
            for k in 1..=count {
                let map = build_copy(topo, &snap)?;
                if let Some(xf) = &xf {
                    let moved = power(xf, k);
                    moved.check_similarity()?;
                    let mut new_closure: Vec<EntityId> = map.values().copied().collect();
                    new_closure.sort_unstable();
                    transform_cells(topo, &new_closure, &moved)?;
                }
                for g in &groups {
                    let members: Vec<EntityId> = g.iter().map(|v| map[v]).collect();
                    let body = topo.make_body(&members).map_err(bad)?;
                    topo.record_created(body).map_err(bad)?;
                    vols.extend(members);
                    bodies.push(body);
                }
            }
            vols.sort_unstable();
            bodies.sort_unstable();
            Ok((vols, bodies))
        })?;
        commit_topology(ctx, &txn, &[("volumes", &vols), ("bodies", &bodies)])?;
        if let Some(name) = &self.lattice {
            let mut tagged: Vec<EntityId> = groups.iter().flatten().copied().chain(vols.iter().copied()).collect();
            tagged.sort_unstable();
            tagged.dedup();
            model_txn(ctx, |m, t| tag_volumes(m, t, name, &tagged, Outside::Error), |_, _| Vec::new())?;
        }
        Ok(())
    }
}
