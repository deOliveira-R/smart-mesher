//! Regularization of the result volumes: merge coplanar and co-surface fragments with
//! `join_faces`, dissolve degree-2 vertices with `join_edges`.
//!
//! `join_faces` and `join_edges` need both cells on the same carrier handle, but fragments of
//! different operands carry different handles for the same geometry (two bricks each own the
//! plane `z = 2`). The geometry is first compared within the model tolerance and, when it is the
//! same, the second cell is re-pointed at the first one's handle (`set_carrier`): this moves
//! nothing beyond the tolerance. A pair the joins refuse is left as it is.

use std::collections::BTreeSet;

use sm_core::{EntityId, Point3};
use sm_geom_api::{CurveDef, Interval, SurfaceDef};
use sm_topo_api::{Carrier, CurveCarrier, Dim, FaceSides, Sense, SurfaceCarrier, TopologyEdit, TopologyView, TopologyViewExt};

use crate::error::BooleanError;
use crate::geo;

/// True if the two surface definitions describe the same surface within `tol` (orientation is
/// not compared).
fn same_surface(a: &SurfaceDef, b: &SurfaceDef, tol: f64, ang: f64) -> bool {
    match (a, b) {
        (SurfaceDef::Plane(p), SurfaceDef::Plane(q)) => {
            let parallel = p.frame.z.cross(q.frame.z).norm() <= ang.max(1e-9);
            parallel && p.frame.z.dot(q.frame.origin - p.frame.origin).abs() <= tol
        }
        (SurfaceDef::Sphere(p), SurfaceDef::Sphere(q)) => geo::dist(p.frame.origin, q.frame.origin) <= tol && (p.radius - q.radius).abs() <= tol,
        (SurfaceDef::Cylinder(p), SurfaceDef::Cylinder(q)) => {
            let parallel = p.frame.z.cross(q.frame.z).norm() <= ang.max(1e-9);
            let d = q.frame.origin - p.frame.origin;
            let off = (d - p.frame.z * d.dot(p.frame.z)).norm();
            parallel && off <= tol && (p.radius - q.radius).abs() <= tol
        }
        (SurfaceDef::Cone(p), SurfaceDef::Cone(q)) => {
            let parallel = p.frame.z.cross(q.frame.z).norm() <= ang.max(1e-9);
            parallel && geo::dist(p.apex(), q.apex()) <= tol && (p.half_angle - q.half_angle).abs() <= 1e-9
        }
        _ => false,
    }
}

fn same_curve(a: &CurveDef, b: &CurveDef, tol: f64, ang: f64) -> bool {
    match (a, b) {
        (CurveDef::Line(p), CurveDef::Line(q)) => {
            let parallel = p.dir.cross(q.dir).norm() <= ang.max(1e-9);
            let d = q.origin - p.origin;
            let off = (d - p.dir * d.dot(p.dir)).norm();
            parallel && off <= tol
        }
        (CurveDef::Circle(p), CurveDef::Circle(q)) => {
            let parallel = p.frame.z.cross(q.frame.z).norm() <= ang.max(1e-9);
            parallel && geo::dist(p.frame.origin, q.frame.origin) <= tol && (p.radius - q.radius).abs() <= tol
        }
        _ => false,
    }
}

/// The sides of a face as a comparable key.
fn sides_key<V: TopologyView + ?Sized>(view: &V, f: EntityId) -> Option<FaceSides> {
    view.face_sides(f)
}

/// Re-point face `drop` at the surface handle of `keep` (same geometry), keeping its normal.
fn unify_surface<E: TopologyEdit + ?Sized>(topo: &mut E, keep: EntityId, drop: EntityId) -> Result<bool, BooleanError> {
    let (Carrier::Surface(sk), Carrier::Surface(sd)) = (topo.carrier(keep).clone(), topo.carrier(drop).clone()) else { return Ok(false) };
    if sk.surface == sd.surface {
        return Ok(sk.sense == sd.sense);
    }
    let Some(p) = crate::faceclass::interior_point(&*topo, drop)? else { return Ok(false) };
    let (Some((nk_surf, _)), Some((nd_surf, _))) = (surface_normal(&*topo, sk.surface, p), surface_normal(&*topo, sd.surface, p)) else { return Ok(false) };
    let n_drop = nd_surf * sd.sense.sign();
    let dot = n_drop.dot(nk_surf);
    if dot.abs() < 0.5 {
        return Ok(false);
    }
    let sense = if dot > 0.0 { Sense::Forward } else { Sense::Reversed };
    // The old pcurves belong to the old parametrization: drop them (the pcurve pass rebuilds).
    let keys: Vec<_> = topo.coedges(drop).map(|c| c.key).collect();
    topo.set_carrier(drop, Carrier::Surface(SurfaceCarrier { surface: sk.surface, sense }))?;
    for k in keys {
        if topo.pcurve(k).is_some() {
            topo.set_pcurve(k, None)?;
        }
    }
    Ok(true)
}

fn surface_normal<V: TopologyView + ?Sized>(view: &V, s: EntityId, p: Point3) -> Option<(sm_core::Vector3, Point3)> {
    let surf = view.shapes().surface(s)?;
    let pr = surf.project(p, None).ok()?;
    let n = geo::unit(surf.normal(pr.uv).ok()?)?;
    Some((n, pr.point))
}

/// Merge fragments of the same surface that meet along edges nothing else uses.
fn join_faces_pass<E: TopologyEdit + ?Sized>(topo: &mut E, volumes: &[EntityId], tried: &mut BTreeSet<(EntityId, EntityId)>) -> Result<bool, BooleanError> {
    let tol = *topo.tolerance();
    for &v in volumes {
        if !topo.is_alive(v) {
            continue;
        }
        let faces: Vec<EntityId> = topo.volume_faces(v).to_vec();
        let mut edges: BTreeSet<EntityId> = BTreeSet::new();
        for &f in &faces {
            edges.extend(topo.face_edges(f).iter().copied());
        }
        for e in edges {
            // The radial rings are only rebuilt at `end_op`: read the uses from the coboundary.
            let uses: Vec<EntityId> = topo.coboundary(e).iter().filter(|i| topo.dim_of(i.cell) == Some(Dim::Face)).map(|i| i.cell).collect();
            // Two uses by two distinct faces (a seam is two uses by one face).
            let [u0, u1] = uses[..] else { continue };
            let (f, g) = (u0.min(u1), u0.max(u1));
            if f == g || tried.contains(&(f, g)) {
                continue;
            }
            if !faces.contains(&f) || !faces.contains(&g) {
                continue;
            }
            tried.insert((f, g));
            if sides_key(&*topo, f) != sides_key(&*topo, g) {
                continue;
            }
            let (Some(sf), Some(sg)) = (crate::pieces::surface_def_of(&*topo, f), crate::pieces::surface_def_of(&*topo, g)) else { continue };
            if !same_surface(sf, sg, tol.linear, tol.angular) {
                continue;
            }
            if !unify_surface(topo, f, g)? {
                continue;
            }
            // Atomic: an error leaves the complex as it was.
            if topo.join_faces(f, g).is_ok() {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// Parameter range and sense that put `edge` on the curve `target` (same geometry).
fn repoint_range<E: TopologyEdit + ?Sized>(topo: &E, edge: EntityId, target_curve: EntityId) -> Option<CurveCarrier> {
    let (s, t) = topo.edge_ends(edge)?;
    let c = topo.shapes().curve(target_curve)?;
    let (ps, pt) = (geo::vertex_point(topo, s)?, geo::vertex_point(topo, t)?);
    let pm = geo::edge_sample(topo, edge, 0.5)?;
    let period = c.domain().period;
    let (ts, tm, te) = (c.project(ps, None).ok()?.t, c.project(pm, None).ok()?.t, c.project(pt, None).ok()?.t);
    match period {
        None => {
            if te >= ts {
                Some(CurveCarrier { curve: target_curve, range: Interval::new(ts, te), sense: Sense::Forward })
            } else {
                Some(CurveCarrier { curve: target_curve, range: Interval::new(te, ts), sense: Sense::Reversed })
            }
        }
        Some(p) => {
            let up = |x: f64, base: f64| x + p * ((base - x) / p).ceil();
            let (tm_f, te_f) = (up(tm, ts), up(te, up(tm, ts)));
            if te_f - ts < p {
                Some(CurveCarrier { curve: target_curve, range: Interval::new(ts, te_f), sense: Sense::Forward })
            } else {
                // Against the curve: go downwards from ts.
                let dn = |x: f64, base: f64| x - p * ((x - base) / p).ceil();
                let (tm_r, te_r) = (dn(tm, ts), dn(te, dn(tm, ts)));
                let _ = (tm_f, tm_r);
                Some(CurveCarrier { curve: target_curve, range: Interval::new(te_r, ts), sense: Sense::Reversed })
            }
        }
    }
}

fn join_edges_pass<E: TopologyEdit + ?Sized>(topo: &mut E, volumes: &[EntityId], tried: &mut BTreeSet<(EntityId, EntityId)>) -> Result<bool, BooleanError> {
    let tol = *topo.tolerance();
    let mut verts: BTreeSet<EntityId> = BTreeSet::new();
    for &v in volumes {
        if topo.is_alive(v) {
            verts.extend(topo.closure(v).into_iter().filter(|c| topo.dim_of(*c) == Some(Dim::Vertex)));
        }
    }
    for vx in verts {
        if !topo.is_alive(vx) {
            continue;
        }
        let es: Vec<EntityId> = topo.coboundary(vx).iter().filter(|i| topo.dim_of(i.cell) == Some(Dim::Edge)).map(|i| i.cell).collect();
        if es.len() != 2 || topo.coboundary(vx).len() != 2 {
            continue;
        }
        let (e1, e2) = (es[0].min(es[1]), es[0].max(es[1]));
        if e1 == e2 || tried.contains(&(e1, e2)) {
            continue;
        }
        tried.insert((e1, e2));
        if topo.edge_ends(e1).is_none_or(|(s, t)| s == t) || topo.edge_ends(e2).is_none_or(|(s, t)| s == t) {
            continue;
        }
        let (Carrier::Curve(c1), Carrier::Curve(c2)) = (topo.carrier(e1).clone(), topo.carrier(e2).clone()) else { continue };
        let (Some(d1), Some(d2)) = (topo.shapes().curve_def(c1.curve), topo.shapes().curve_def(c2.curve)) else { continue };
        if !same_curve(d1, d2, tol.linear, tol.angular) {
            continue;
        }
        if c1.curve != c2.curve {
            let Some(new) = repoint_range(&*topo, e2, c1.curve) else { continue };
            topo.set_carrier(e2, Carrier::Curve(new))?;
        }
        if topo.join_edges(e1, e2).is_ok() {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Regularize `volumes`: join fragments, then dissolve the vertices that no longer separate
/// anything. Pairs the topology refuses are skipped.
pub(crate) fn regularize<E: TopologyEdit + ?Sized>(topo: &mut E, volumes: &[EntityId]) -> Result<(), BooleanError> {
    let mut tried_f: BTreeSet<(EntityId, EntityId)> = BTreeSet::new();
    while join_faces_pass(topo, volumes, &mut tried_f)? {}
    let mut tried_e: BTreeSet<(EntityId, EntityId)> = BTreeSet::new();
    while join_edges_pass(topo, volumes, &mut tried_e)? {}
    Ok(())
}
