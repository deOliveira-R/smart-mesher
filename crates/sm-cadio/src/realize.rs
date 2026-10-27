//! [`realize`]: put a [`BrepModel`] into a topology store.
//!
//! Creation order is fixed: all vertices (solids in order, then ascending STEP id), then all
//! edges, then all faces (pole edges are created with the face that needs them), then volumes
//! and bodies (one body per solid). Every distinct curve or surface gets one handle, added when
//! its first user is created.
//!
//! Mapping: `Face.sense` becomes the surface carrier sense, `Edge.sense` the curve carrier
//! sense, coedge senses become `CoedgeSpec` senses; faces are used on `Side::Left` (normals out
//! of the material) and void shells are `ShellKind::Void`.
//!
//! **Pole rule.** A loop where consecutive coedges are the same edge traversed oppositely (a
//! seam pair) and the vertex between them lies on a singularity of the face's surface (sphere
//! pole, cone apex) gets a `DEGENERATE` closed edge there with a `PCurve::Line2` pole pcurve
//! sweeping `u` by one period. The sweep direction is the one that gives positive face area.
//! Pcurves are set only for pole coedges.
//!
//! **Orientation self-check.** Each shell is first realized into a scratch
//! [`sm_topo_api::CellComplex`]; an outer shell with negative signed volume, or a void shell
//! with positive signed volume, has its loops and carrier senses flipped before the real run.

use sm_core::{EntityId, Point3};
use sm_geom_api::{Interval, Line2, ParamAxis, ParametricCurve, ParametricSurface, PCurve, SurfaceDef, Uv};
use sm_topo_api::{CellComplex, CellFlags, CoedgeKey, CoedgeSpec, CurveCarrier, FaceUseSpec, OpId, Sense, ShellKind, ShellSpec, Side, SurfaceCarrier, TopologyEdit, TopologyView};

use crate::error::StepError;
use crate::model::{BrepModel, Coedge, Solid, SolidKind};

/// Options of [`realize`].
#[derive(Clone, Debug, PartialEq)]
pub struct RealizeOptions {
    /// Check each shell's orientation in a scratch complex and flip inverted ones.
    pub fix_orientation: bool,
    /// A vertex within this distance of a surface singularity counts as at the pole.
    pub pole_tol: f64,
}

impl Default for RealizeOptions {
    fn default() -> Self {
        Self { fix_orientation: true, pole_tol: 1e-6 }
    }
}

/// A solid whose orientation was repaired.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Inverted {
    /// STEP id of the solid.
    pub solid: u64,
    /// Indices of the shells that were flipped.
    pub shells: Vec<usize>,
}

/// The cells [`realize`] created, in creation order.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Realized {
    /// One body per solid.
    pub bodies: Vec<EntityId>,
    /// One volume per solid.
    pub volumes: Vec<EntityId>,
    /// Faces.
    pub faces: Vec<EntityId>,
    /// Edges, including `DEGENERATE` pole edges (created with their face).
    pub edges: Vec<EntityId>,
    /// Vertices.
    pub vertices: Vec<EntityId>,
    /// Solids whose orientation was repaired.
    pub inverted: Vec<Inverted>,
    /// Number of pole edges inserted.
    pub poles: usize,
}

/// Create the cells of `m` in `topo`, inside an open operation.
///
/// # Errors
/// [`StepError::Topology`] when a store edit fails, [`StepError::GeometryMismatch`] when a pole
/// cannot be parametrized. Call it inside `begin_op`/`end_op` (as `topo_txn` does): the caller
/// aborts the operation on error, so no partial state survives.
pub fn realize<E: TopologyEdit + ?Sized>(m: &BrepModel, topo: &mut E, o: &RealizeOptions) -> Result<Realized, StepError> {
    let mut flips: Vec<Vec<bool>> = Vec::with_capacity(m.solids.len());
    let mut inverted = Vec::new();
    for s in &m.solids {
        let f = if o.fix_orientation && s.kind != SolidKind::Sheet { decide_flips(s, o)? } else { vec![false; s.shells.len()] };
        let shells: Vec<usize> = f.iter().enumerate().filter(|(_, x)| **x).map(|(i, _)| i).collect();
        if !shells.is_empty() {
            inverted.push(Inverted { solid: s.id, shells });
        }
        flips.push(f);
    }
    let refs: Vec<&Solid> = m.solids.iter().collect();
    let mut r = build(topo, &refs, &flips, false, o)?;
    r.inverted = inverted;
    Ok(r)
}

/// Which shells of `s` have their normals the wrong way round.
fn decide_flips(s: &Solid, o: &RealizeOptions) -> Result<Vec<bool>, StepError> {
    let mut scratch = CellComplex::new();
    scratch.begin_op(OpId(1))?;
    let flips = vec![false; s.shells.len()];
    let r = build(&mut scratch, &[s], std::slice::from_ref(&flips), true, o);
    let out = match r {
        Ok(r) => s
            .shells
            .iter()
            .enumerate()
            .map(|(i, sh)| {
                let vol = r.volumes.get(i).copied();
                let Some(m) = vol.and_then(|v| scratch.volume_moments(v).ok()) else { return false };
                // Outer shells enclose positive volume, voids negative (normals out of the material).
                let scale = bbox_scale(s, sh);
                if m.volume.abs() <= 1e-12 * scale * scale * scale {
                    return false;
                }
                match sh.kind {
                    ShellKind::Void => m.volume > 0.0,
                    _ => m.volume < 0.0,
                }
            })
            .collect(),
        Err(e) => {
            scratch.abort_op();
            return Err(e);
        }
    };
    scratch.abort_op();
    Ok(out)
}

fn bbox_scale(s: &Solid, sh: &crate::model::Shell) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for &f in &sh.faces {
        for lp in &s.faces[f].loops {
            for c in lp {
                for v in [s.edges[c.edge].start, s.edges[c.edge].end] {
                    let p = s.vertices[v].point;
                    for (k, x) in [p.x, p.y, p.z].into_iter().enumerate() {
                        lo[k] = lo[k].min(x);
                        hi[k] = hi[k].max(x);
                    }
                }
            }
        }
    }
    (0..3).map(|k| hi[k] - lo[k]).fold(0.0, f64::max).max(f64::MIN_POSITIVE)
}

/// One slot of a loop under construction: a real coedge or an inserted pole.
#[derive(Clone, Copy)]
struct Slot {
    edge: EntityId,
    sense: Sense,
    /// The model coedge for real slots.
    model: Option<Coedge>,
    /// Pole slots: the pole's position (vertex index and surface singularity).
    pole: Option<PoleSpec>,
}

#[derive(Clone, Copy)]
struct PoleSpec {
    at: f64,
}

fn build<E: TopologyEdit + ?Sized>(topo: &mut E, solids: &[&Solid], flips: &[Vec<bool>], per_shell_volumes: bool, o: &RealizeOptions) -> Result<Realized, StepError> {
    let mut out = Realized::default();
    // ---- vertices
    let mut vids: Vec<Vec<EntityId>> = Vec::with_capacity(solids.len());
    for s in solids {
        let mut v = Vec::with_capacity(s.vertices.len());
        for vx in &s.vertices {
            v.push(topo.make_vertex(vx.point)?);
        }
        out.vertices.extend_from_slice(&v);
        vids.push(v);
    }
    // ---- edges
    let mut eids: Vec<Vec<EntityId>> = Vec::with_capacity(solids.len());
    for s in solids {
        let mut curve_h: Vec<Option<EntityId>> = vec![None; s.curves.len()];
        let mut ev = Vec::with_capacity(s.edges.len());
        for e in s.edges.iter() {
            let h = match curve_h[e.curve] {
                Some(h) => h,
                None => {
                    let h = topo.shapes_mut()?.add_curve(s.curves[e.curve].def.clone()).map_err(|g| StepError::GeometryMismatch { id: s.curves[e.curve].id, msg: g.to_string() })?;
                    curve_h[e.curve] = Some(h);
                    h
                }
            };
            let sol_idx = eids.len();
            ev.push(topo.make_edge(vids[sol_idx][e.start], vids[sol_idx][e.end], Some(CurveCarrier { curve: h, range: e.range, sense: e.sense }))?);
        }
        out.edges.extend_from_slice(&ev);
        eids.push(ev);
    }
    // ---- faces
    let mut fids: Vec<Vec<EntityId>> = Vec::with_capacity(solids.len());
    for (k, s) in solids.iter().enumerate() {
        let mut surf_h: Vec<Option<EntityId>> = vec![None; s.surfaces.len()];
        let mut fv = Vec::with_capacity(s.faces.len());
        for f in &s.faces {
            let flip = flips[k][f.shell];
            let h = match surf_h[f.surface] {
                Some(h) => h,
                None => {
                    let h = topo.shapes_mut()?.add_surface(s.surfaces[f.surface].def.clone()).map_err(|g| StepError::GeometryMismatch { id: s.surfaces[f.surface].id, msg: g.to_string() })?;
                    surf_h[f.surface] = Some(h);
                    h
                }
            };
            let sense = if flip { f.sense.flip() } else { f.sense };
            let sdef = &s.surfaces[f.surface].def;
            let mut loops: Vec<Vec<Slot>> = Vec::with_capacity(f.loops.len());
            for lp in &f.loops {
                let mut slots: Vec<Slot> = lp.iter().map(|c| Slot { edge: eids[k][c.edge], sense: c.sense, model: Some(*c), pole: None }).collect();
                if flip {
                    slots.reverse();
                    for sl in &mut slots {
                        sl.sense = sl.sense.flip();
                        if let Some(m) = &mut sl.model {
                            m.sense = m.sense.flip();
                        }
                    }
                }
                loops.push(slots);
            }
            // Pole insertion needs the pole edge ids, so create them first and compute pcurves
            // afterwards from the final loop order.
            let mut pending: Vec<(EntityId, Side, PCurve)> = Vec::new();
            for slots in loops.iter_mut() {
                                let poles = insert_poles(topo, s, sdef, sense, slots, &vids[k], &mut out, o, f.id)?;
                pending.extend(poles);
            }
            let specs: Vec<Vec<CoedgeSpec>> = loops.iter().map(|l| l.iter().map(|sl| CoedgeSpec { edge: sl.edge, sense: sl.sense }).collect()).collect();
            let spec_refs: Vec<&[CoedgeSpec]> = specs.iter().map(Vec::as_slice).collect();
            let face = topo.make_face(&spec_refs, Some(SurfaceCarrier { surface: h, sense }))?;
            for (edge, side, pc) in pending {
                topo.set_pcurve(CoedgeKey { face, edge, side }, Some(pc))?;
            }
            fv.push(face);
        }
        out.faces.extend_from_slice(&fv);
        fids.push(fv);
    }
    // ---- volumes and bodies
    for (k, s) in solids.iter().enumerate() {
        if s.kind == SolidKind::Sheet {
            // A sheet is a body of free faces: no volume.
            if !per_shell_volumes {
                let b = topo.make_body(&fids[k])?;
                out.bodies.push(b);
            }
            continue;
        }
        let uses: Vec<Vec<FaceUseSpec>> = s.shells.iter().map(|sh| sh.faces.iter().map(|&f| FaceUseSpec { face: fids[k][f], side: Side::Left }).collect()).collect();
        if per_shell_volumes {
            for u in &uses {
                let v = topo.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: u }])?;
                out.volumes.push(v);
            }
        } else {
            let specs: Vec<ShellSpec<'_>> = s.shells.iter().zip(&uses).map(|(sh, u)| ShellSpec { kind: sh.kind, uses: u }).collect();
            let v = topo.make_volume(&specs)?;
            out.volumes.push(v);
            let b = topo.make_body(&[v])?;
            out.bodies.push(b);
        }
    }
    Ok(out)
}

/// First Gauss node of the face sampler (distance from the panel centre in `[-1, 1]` units).
const GAUSS_X0: f64 = 0.906_179_845_938_664;
const WALK_STEPS: usize = 32;

fn project_uv(surf: &dyn ParametricSurface, p: Point3, face: u64) -> Result<Uv, StepError> {
    surf.project(p, None).map(|r| r.uv).map_err(|e| StepError::GeometryMismatch { id: face, msg: format!("cannot parametrize a pole loop: {e}") })
}

/// The `u` reached after traversing coedge `c` of `s`, continuing from `prev` the way the face
/// sampler unwraps parameters (nearest period copy). With no `prev` it starts from the raw `u`
/// of the sampler's first Gauss node, so the branch matches what the sampler will see.
fn walk(surf: &dyn ParametricSurface, period_dom: sm_geom_api::Domain1, s: &Solid, c: Coedge, prev: Option<f64>, face: u64) -> Result<f64, StepError> {
    let e = &s.edges[c.edge];
    let curve: &dyn ParametricCurve = s.curves[e.curve].def.as_curve();
    let along = (e.sense == Sense::Forward) == (c.sense == Sense::Forward);
    let (lo, hi) = (e.range.lo, e.range.hi);
    let mut u = match prev {
        Some(u) => u,
        None => {
            let h = (hi - lo) / 16.0;
            let t = if along { lo + 0.5 * (1.0 - GAUSS_X0) * h } else { lo + (15.0 + 0.5 * (GAUSS_X0 + 1.0)) * h };
            project_uv(surf, curve.eval(t), face)?.u
        }
    };
    // Interior samples only: an end point on a pole has no meaningful `u`.
    for k in 0..WALK_STEPS {
        let f = (k as f64 + 0.5) / WALK_STEPS as f64;
        let t = if along { lo + (hi - lo) * f } else { hi - (hi - lo) * f };
        let raw = project_uv(surf, curve.eval(t), face)?.u;
        u = period_dom.nearest_to(raw, u);
    }
    Ok(u)
}

/// Insert `DEGENERATE` edges into `slots` at seam pairs that end on a surface pole. Returns the
/// pole pcurves to set once the face exists, keyed by `(edge, side)`.
#[allow(clippy::too_many_arguments)]
fn insert_poles<E: TopologyEdit + ?Sized>(
    topo: &mut E,
    s: &Solid,
    sdef: &SurfaceDef,
    carrier: Sense,
    slots: &mut Vec<Slot>,
    vids: &[EntityId],
    out: &mut Realized,
    o: &RealizeOptions,
    face_id: u64,
) -> Result<Vec<(EntityId, Side, PCurve)>, StepError> {
    let n = slots.len();
    if n < 2 {
        return Ok(Vec::new());
    }
    let Some(surf) = sdef.as_surface() else { return Ok(Vec::new()) };
    let dom = surf.domain();
    let Some(period) = dom.u.period else { return Ok(Vec::new()) };
    if dom.singularities.is_empty() {
        return Ok(Vec::new());
    }
    // Junctions: (index of the slot the pole follows, vertex index, v of the pole).
    let mut at_pole: Vec<(usize, usize, f64)> = Vec::new();
    for i in 0..n {
        let (Some(ma), Some(mb)) = (slots[i].model, slots[(i + 1) % n].model) else { continue };
        if ma.edge != mb.edge || ma.sense == mb.sense {
            continue;
        }
        let e = &s.edges[ma.edge];
        let junction = if ma.sense == Sense::Forward { e.end } else { e.start };
        let p = s.vertices[junction].point;
        if let Some(g) = dom.singularities.iter().find(|g| g.along == ParamAxis::V && (g.point - p).norm() <= o.pole_tol) {
            at_pole.push((i, junction, g.at));
        }
    }
    if at_pole.is_empty() {
        return Ok(Vec::new());
    }
    // Create the pole edges and build the new slot list.
    let mut new_slots: Vec<Slot> = Vec::with_capacity(n + at_pole.len());
    let mut pole_info: Vec<Option<f64>> = Vec::with_capacity(n + at_pole.len());
    for (i, sl) in slots.iter().enumerate() {
        new_slots.push(*sl);
        pole_info.push(None);
        if let Some(&(_, vi, at)) = at_pole.iter().find(|p| p.0 == i) {
            let edge = topo.make_edge(vids[vi], vids[vi], None)?;
            topo.set_flags(edge, CellFlags::DEGENERATE)?;
            out.edges.push(edge);
            out.poles += 1;
            new_slots.push(Slot { edge, sense: Sense::Forward, model: None, pole: Some(PoleSpec { at }) });
            pole_info.push(Some(at));
        }
    }
    // Walk the loop in the order the store will keep it (smallest `(edge, side)` first).
    let m = new_slots.len();
    let start = (0..m)
        .filter(|&i| new_slots[i].model.is_some())
        .min_by_key(|&i| (new_slots[i].edge, new_slots[i].sense.side()))
        .unwrap_or(0);
    let sigma = carrier.sign();
    let mut pending = Vec::new();
    let mut prev: Option<f64> = None;
    for step in 0..m {
        let i = (start + step) % m;
        if let (Some(c), None) = (new_slots[i].model, new_slots[i].pole) {
            prev = Some(walk(surf, dom.u, s, c, prev, face_id)?);
            continue;
        }
        let Some(spec) = new_slots[i].pole else { continue };
        let before = new_slots[(i + m - 1) % m].model.ok_or(StepError::GeometryMismatch { id: face_id, msg: "two pole edges in a row".into() })?;
        let e = &s.edges[before.edge];
        let mid = s.curves[e.curve].def.as_curve().eval(0.5 * (e.range.lo + e.range.hi));
        let v_mid = project_uv(surf, mid, face_id)?.v;
        let dv = spec.at - v_mid;
        if dv.is_nan() || dv.abs() <= 1e-12 {
            return Err(StepError::GeometryMismatch { id: face_id, msg: "the edge before a pole does not leave the pole".into() });
        }
        let u_in = prev.ok_or(StepError::GeometryMismatch { id: face_id, msg: "a pole loop must contain a curve edge".into() })?;
        let arrive = dv.signum();
        let sweep = -arrive * sigma * period;
        let (sense, origin_u) = if sweep > 0.0 { (Sense::Forward, u_in) } else { (Sense::Reversed, u_in - period) };
        new_slots[i].sense = sense;
        pending.push((new_slots[i].edge, sense.side(), PCurve::Line2(Line2 { origin: Uv::new(origin_u, spec.at), dir: Uv::new(1.0, 0.0), range: Interval::new(0.0, period) })));
        prev = Some(u_in + sweep);
    }
    *slots = new_slots;
    Ok(pending)
}
