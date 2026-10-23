//! Built-in copy schemes. Deck pairs (ir.md §5.1) and Cubit's copy mesh need them at every
//! dimension, and they are generic (a transform plus a committed source mesh), so the contract
//! ships them: [`CurveCopy`] (`"curve.copy"`) and [`SurfaceCopy`] (`"surface.copy"`). The
//! driver forces them on any entity with a [`Requirement::CopyOf`].

use crate::caps::Caps;
use crate::error::{FailureKind, SchemeError};
use crate::key::ShapeKey;
use crate::patch::{CurvePatch, FaceList, Grid, NewNode, NodeId, PatchRef, SurfacePatch};
use crate::plan::{Applicability, Plan, Requirement};
use crate::scheme::{CurveInput, CurveScheme, PlanCtx, SchemeCore, SurfaceInput, SurfaceScheme};
use sm_core::Transform;
use sm_mesh_api::Param;
use sm_topo_api::{Dim, Sense};
use std::collections::BTreeMap;

/// Name of the built-in curve copy scheme.
pub const CURVE_COPY: &str = "curve.copy";
/// Name of the built-in surface copy scheme.
pub const SURFACE_COPY: &str = "surface.copy";

fn copy_req(imposed: &[crate::plan::ChildRequirement], target: ShapeKey) -> Option<(ShapeKey, Transform, Sense)> {
    imposed.iter().find_map(|r| match &r.req {
        Requirement::CopyOf { source, transform, sense } if r.target == target => Some((*source, *transform, *sense)),
        _ => None,
    })
}

fn copy_plan(id: &str, cx: &PlanCtx<'_>) -> Result<Plan, SchemeError> {
    let t = cx.target();
    let (source, _, _) = copy_req(cx.imposed, t).ok_or_else(|| SchemeError::NotApplicable(format!("{t} has no copy source")))?;
    let mut p = Plan::single(id, t);
    p.after.push(source);
    Ok(p)
}

/// Copies the node distribution of a source curve (reversed for a `Reversed` pairing).
#[derive(Clone, Copy, Debug, Default)]
pub struct CurveCopy;

impl SchemeCore for CurveCopy {
    fn id(&self) -> &str {
        CURVE_COPY
    }
    fn dim(&self) -> Dim {
        Dim::Edge
    }
    fn caps(&self) -> Caps {
        Caps::INHERIT
    }
    fn applicability(&self, cx: &PlanCtx<'_>) -> Applicability {
        match copy_req(cx.imposed, cx.target()) {
            Some(_) => Applicability::yes(1.0, "copy source imposed"),
            None => Applicability::no("no copy source"),
        }
    }
    fn plan(&self, cx: &PlanCtx<'_>) -> Result<Plan, SchemeError> {
        copy_plan(CURVE_COPY, cx)
    }
}

impl CurveScheme for CurveCopy {
    fn apply(&self, input: &CurveInput<'_>) -> Result<CurvePatch, SchemeError> {
        let (source, _, sense) = copy_req(input.imposed, input.curve).ok_or_else(|| SchemeError::NotApplicable("no copy source".into()))?;
        let src = input.mesh.curve(source).ok_or_else(|| SchemeError::failed(FailureKind::Constraint, format!("copy source {source} is not meshed")))?;
        let n = src.fractions.len().saturating_sub(1);
        if n != input.intervals as usize {
            return Err(SchemeError::failed(FailureKind::Constraint, format!("copy source has {n} intervals, target {}", input.intervals)));
        }
        let inner = &src.fractions[1..n];
        let fractions = match sense {
            Sense::Forward => inner.to_vec(),
            Sense::Reversed => inner.iter().rev().map(|f| 1.0 - f).collect(),
        };
        Ok(CurvePatch { fractions })
    }
}

/// Copies a source face mesh under a transform; boundary nodes snap to the target's committed
/// boundary nodes, interior nodes are mapped and projected onto the target face. Proper
/// transforms reverse element loops (the image of the source normal is minus the target normal,
/// sign rule `n_B = -g(n_A)`); grids are transposed accordingly.
#[derive(Clone, Copy, Debug, Default)]
pub struct SurfaceCopy;

impl SchemeCore for SurfaceCopy {
    fn id(&self) -> &str {
        SURFACE_COPY
    }
    fn dim(&self) -> Dim {
        Dim::Face
    }
    fn caps(&self) -> Caps {
        Caps::INHERIT
    }
    fn applicability(&self, cx: &PlanCtx<'_>) -> Applicability {
        match copy_req(cx.imposed, cx.target()) {
            Some(_) => Applicability::yes(1.0, "copy source imposed"),
            None => Applicability::no("no copy source"),
        }
    }
    fn plan(&self, cx: &PlanCtx<'_>) -> Result<Plan, SchemeError> {
        copy_plan(SURFACE_COPY, cx)
    }
}

impl SurfaceScheme for SurfaceCopy {
    fn apply(&self, input: &SurfaceInput<'_>) -> Result<SurfacePatch, SchemeError> {
        let (source, t, _) = copy_req(input.imposed, input.face).ok_or_else(|| SchemeError::NotApplicable("no copy source".into()))?;
        let src = input.mesh.surface(source).ok_or_else(|| SchemeError::failed(FailureKind::Constraint, format!("copy source {source} is not meshed")))?;
        let reverse = !t.is_improper();
        let boundary: Vec<NodeId> = input.loops.iter().flat_map(|l| l.nodes.iter().copied()).collect();
        let mut map: BTreeMap<NodeId, PatchRef> = BTreeMap::new();
        let mut nodes: Vec<NewNode> = Vec::new();
        let base_tol = 16.0 * input.shape.linear_tolerance();
        let mut ids: Vec<NodeId> = src.faces.nodes.clone();
        ids.sort();
        ids.dedup();
        for n in ids {
            let q = t.apply_point(input.mesh.point(n));
            let tol = base_tol * (1.0 + (q - sm_core::Point3::new(0.0, 0.0, 0.0)).norm());
            if input.mesh.origin(n).on != source {
                let hit = boundary.iter().copied().find(|&b| (input.mesh.point(b) - q).norm() <= tol);
                let b = hit.ok_or_else(|| SchemeError::failed(FailureKind::Constraint, format!("copy: no boundary node of {} at the image of {n:?}", input.face)))?;
                map.insert(n, PatchRef::Old(b));
            } else {
                let (p, uv) = input.shape.snap_to_face(input.face, q)?;
                map.insert(n, PatchRef::New(nodes.len() as u32));
                nodes.push(NewNode { point: p, on: input.face, param: uv.map_or(Param::None, Param::Surface) });
            }
        }
        let get = |n: &NodeId| map.get(n).copied().ok_or_else(|| SchemeError::failed(FailureKind::Internal, "copy: unmapped node"));
        let mut faces = FaceList::new();
        let mut buf = Vec::with_capacity(8);
        for l in src.faces.iter() {
            buf.clear();
            for n in l {
                buf.push(get(n)?);
            }
            if reverse {
                buf.reverse();
            }
            faces.push(&buf);
        }
        let grid = match &src.grid {
            None => None,
            Some(g) if !reverse => Some(Grid { ni: g.ni, nj: g.nj, nodes: g.nodes.iter().map(get).collect::<Result<_, _>>()? }),
            Some(g) => {
                // Transpose: (i, j) -> (j, i) reverses the orientation of every grid quad.
                let mut out = Vec::with_capacity(g.nodes.len());
                for i in 0..=g.ni {
                    for j in 0..=g.nj {
                        out.push(get(&g.at(i, j))?);
                    }
                }
                Some(Grid { ni: g.nj, nj: g.ni, nodes: out })
            }
        };
        Ok(SurfacePatch { nodes, faces, grid })
    }
}
