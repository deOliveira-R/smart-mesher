//! `model.heal_check`: a read-only scan of imported or hand-built geometry for the features that
//! force tiny mesh elements or break a mesh: short edges, small and sliver faces, small holes,
//! near gaps between bodies, open shells and blend (fillet) candidates.
//!
//! Every threshold is relative to the model tolerance and to the bounding-box diagonal, and every
//! finding is located (an entity id plus the measure that tripped it). Findings are sorted by
//! `(code, entity, value)`; per-entity work runs under rayon with ordered collection, so the
//! report is bit-identical for any thread count.
//!
//! ## Thresholds (tuned on the W4-REFCORPUS dirty STEP files)
//!
//! With `L = min(linear_max / 4, 1e-3 * diag)`, raised to at least `10 * linear`:
//! - `small_edge`: length `<= L`. The `1e-4` notch and step edges of the corpus are caught; the
//!   `2 pi 1e-3` circle of the small-hole plate is not (it is a hole, not a short edge).
//! - `small_face`: area `<= L^2`.
//! - `sliver_face`: a planar face, not small, with in-plane aspect `>= 100` and width `<= L`
//!   (width and aspect come from the central second moments, exact for rectangles).
//! - `small_hole`: an inner loop of diameter `<= 20 L`; the through bore counts once.
//! - `gap_near`: a `merge_check` pair with status `near` (gap above the cell tolerances and at
//!   most `linear_max`) whose two cells lie on no common face or neighbouring faces. The corners
//!   of a notch are close by construction and are not gaps.
//! - `open_shell`: edges used by a single face (not a baffle); one finding per connected chain.
//! - `blend_candidate`: a partial cylinder face with exactly two straight rails, each tangent
//!   (within `1e-3` rad) to its neighbour, an angular extent up to a quarter turn and radius up
//!   to `diag / 4`. Sphere, torus and cone blends are not detected.

use std::collections::{BTreeMap, BTreeSet};

use rayon::prelude::*;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{Aabb, EntityId, Point3, Vector3};
use sm_geom_api::{CurveDef, SurfaceDef};
use sm_topo_api::{CellFlags, Dim, Side, TopologyView, TopologyViewExt};

use crate::error::BooleanError;
use crate::geo;
use crate::merge::{MergeKind, MergeKinds, MergeOptions, MergeStatus, merge_check};

/// The kinds of finding. The declaration order is the alphabetical order of the code strings,
/// which is the report's sort order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum HealCode {
    /// A partial cylinder tangent to both neighbours (a fillet or round).
    BlendCandidate,
    /// Two cells of different bodies (or unsewn faces) closer than `linear_max` but not merged.
    GapNear,
    /// Edges used by one face only: the boundary of a hole in a shell.
    OpenShell,
    /// A thin planar face (long and narrow).
    SliverFace,
    /// An edge shorter than the small-length threshold.
    SmallEdge,
    /// A face of area below the small-area threshold.
    SmallFace,
    /// A hole (inner loop) narrower than the hole threshold.
    SmallHole,
}

impl HealCode {
    /// The stable code string.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::BlendCandidate => "blend_candidate",
            Self::GapNear => "gap_near",
            Self::OpenShell => "open_shell",
            Self::SliverFace => "sliver_face",
            Self::SmallEdge => "small_edge",
            Self::SmallFace => "small_face",
            Self::SmallHole => "small_hole",
        }
    }
}

/// One located finding.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct HealFinding {
    /// What was found.
    pub code: HealCode,
    /// The entity it is located at: the edge, face (the bore face for a hole) or lower id of a
    /// pair; the lowest edge for an open shell chain.
    pub entity: EntityId,
    /// All entities involved, ascending: the pair for `gap_near`, the chain for `open_shell`,
    /// the face and its two neighbours for `blend_candidate`, the hole's loop faces for
    /// `small_hole`; just `entity` otherwise.
    pub entities: Vec<EntityId>,
    /// The measure that tripped the threshold: edge length (`small_edge`), area (`small_face`),
    /// aspect ratio (`sliver_face`), diameter (`small_hole`), gap (`gap_near`), total free edge
    /// length (`open_shell`) or radius (`blend_candidate`).
    pub value: f64,
    /// A second measure where one helps: the width of a `sliver_face`, the angular extent of a
    /// `blend_candidate` in radians; `None` otherwise.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub extra: Option<f64>,
}

/// The thresholds a scan used, resolved from the options and the model.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct HealThresholds {
    /// Bounding-box diagonal of the scanned faces.
    pub diagonal: f64,
    /// Edges up to this length are `small_edge`.
    pub small_edge: f64,
    /// Faces up to this area are `small_face`.
    pub small_face_area: f64,
    /// Minimum aspect ratio of a `sliver_face`.
    pub sliver_aspect: f64,
    /// Maximum width of a `sliver_face`.
    pub sliver_width: f64,
    /// Holes up to this diameter are `small_hole`.
    pub hole_max_diameter: f64,
    /// Blends up to this radius are `blend_candidate`.
    pub blend_max_radius: f64,
}

/// Options of [`heal_check`]; every `None` takes the default derived from the model.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct HealOptions {
    /// Scan only the closure of these entities (default: the whole model).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub entities: Option<Vec<EntityId>>,
    /// Length at or below which an edge is `small_edge`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub small_edge: Option<f64>,
    /// Area at or below which a face is `small_face`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub small_face_area: Option<f64>,
    /// Aspect ratio at or above which a thin planar face is a `sliver_face`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sliver_aspect: Option<f64>,
    /// Width at or below which a thin planar face is a `sliver_face`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sliver_width: Option<f64>,
    /// Diameter at or below which a hole is `small_hole`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub hole_max_diameter: Option<f64>,
    /// Radius at or below which a tangent partial cylinder is `blend_candidate`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub blend_max_radius: Option<f64>,
}

/// The result of [`heal_check`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct HealReport {
    /// The thresholds used.
    pub thresholds: HealThresholds,
    /// Findings sorted by `(code, entity, value)`.
    pub findings: Vec<HealFinding>,
}

impl HealReport {
    /// The findings of one code.
    pub fn with_code(&self, code: HealCode) -> impl Iterator<Item = &HealFinding> {
        self.findings.iter().filter(move |f| f.code == code)
    }

    /// The distinct codes present, in code order.
    pub fn codes(&self) -> Vec<HealCode> {
        self.findings.iter().map(|f| f.code).collect::<BTreeSet<_>>().into_iter().collect()
    }
}

/// Quarter turn plus a small allowance: the largest angular extent of a blend.
const BLEND_MAX_EXTENT: f64 = core::f64::consts::FRAC_PI_2 + 0.01;
/// Largest angle between the normals of a blend and its neighbour at the rail.
const BLEND_TANGENT: f64 = 1e-3;
/// Holes are up to this many small lengths wide.
const HOLE_FACTOR: f64 = 20.0;
/// Samples per edge when measuring a hole.
const HOLE_SAMPLES: usize = 16;

struct Scope {
    edges: Vec<EntityId>,
    faces: Vec<EntityId>,
}

fn resolve_scope(view: &dyn TopologyView, entities: &Option<Vec<EntityId>>) -> Result<Scope, BooleanError> {
    let pick = |cells: &BTreeSet<EntityId>, d: Dim| cells.iter().copied().filter(|c| view.dim_of(*c) == Some(d) && view.is_alive(*c)).collect::<Vec<_>>();
    let mut cells: BTreeSet<EntityId> = BTreeSet::new();
    match entities {
        None => {
            for d in [Dim::Edge, Dim::Face] {
                cells.extend(view.cells(d).iter().copied());
            }
        }
        Some(list) => {
            for &id in list {
                if !view.is_alive(id) || view.dim_of(id).is_none() {
                    return Err(BooleanError::Invalid(format!("{id} is not a live topology cell")));
                }
                cells.extend(view.closure(id));
            }
        }
    }
    Ok(Scope { edges: pick(&cells, Dim::Edge), faces: pick(&cells, Dim::Face) })
}

fn positive(name: &str, v: Option<f64>) -> Result<Option<f64>, BooleanError> {
    match v {
        Some(x) if !(x.is_finite() && x > 0.0) => Err(BooleanError::Invalid(format!("`{name}` must be a positive finite number, got {x:e}"))),
        other => Ok(other),
    }
}

fn bounding_diagonal(view: &dyn TopologyView, faces: &[EntityId]) -> f64 {
    let boxes: Vec<Aabb> = faces.par_iter().map(|&f| geo::face_aabb(view, f, 0.0)).collect();
    let mut b = Aabb::empty();
    for fb in boxes.iter().filter(|b| !b.is_empty()) {
        b.extend(fb.lo);
        b.extend(fb.hi);
    }
    if b.is_empty() { 0.0 } else { geo::dist(b.lo, b.hi) }
}

/// Resolve the thresholds for a model of diagonal `diag`.
fn thresholds(view: &dyn TopologyView, diag: f64, o: &HealOptions) -> Result<HealThresholds, BooleanError> {
    let tol = view.tolerance();
    let l = (0.25 * tol.linear_max).min(1e-3 * diag).max(10.0 * tol.linear);
    Ok(HealThresholds {
        diagonal: diag,
        small_edge: positive("small_edge", o.small_edge)?.unwrap_or(l),
        small_face_area: positive("small_face_area", o.small_face_area)?.unwrap_or(l * l),
        sliver_aspect: positive("sliver_aspect", o.sliver_aspect)?.unwrap_or(100.0),
        sliver_width: positive("sliver_width", o.sliver_width)?.unwrap_or(l),
        hole_max_diameter: positive("hole_max_diameter", o.hole_max_diameter)?.unwrap_or(HOLE_FACTOR * l),
        blend_max_radius: positive("blend_max_radius", o.blend_max_radius)?.unwrap_or(0.25 * diag),
    })
}

/// Length of an edge: the curve length, or the chord without a curve.
fn edge_length(view: &dyn TopologyView, e: EntityId) -> Option<f64> {
    if let Some((c, cc)) = geo::edge_curve(view, e) {
        return c.length(cc.range, view.tolerance().linear).ok();
    }
    let (s, t) = view.edge_ends(e)?;
    Some(geo::dist(geo::vertex_point(view, s)?, geo::vertex_point(view, t)?))
}

/// The faces that use `cell` (an edge, or a vertex through its edges), ascending and distinct.
fn faces_around(view: &dyn TopologyView, cell: EntityId) -> BTreeSet<EntityId> {
    let face_users = |e: EntityId| view.coboundary(e).iter().filter(|i| view.dim_of(i.cell) == Some(Dim::Face)).map(|i| i.cell).collect::<Vec<_>>();
    match view.dim_of(cell) {
        Some(Dim::Face) => BTreeSet::from([cell]),
        Some(Dim::Edge) => face_users(cell).into_iter().collect(),
        Some(Dim::Vertex) => view.coboundary(cell).iter().filter(|i| view.dim_of(i.cell) == Some(Dim::Edge)).flat_map(|i| face_users(i.cell)).collect(),
        _ => BTreeSet::new(),
    }
}

/// `(aspect, width)` of a planar face from its central second moments; `None` if it is not
/// planar or degenerate.
fn planar_shape(view: &dyn TopologyView, f: EntityId) -> Option<(f64, f64)> {
    let (n, _) = view.face_plane(f, view.tolerance().linear).ok()??;
    let c = view.face_central(f).ok()?;
    if c.area.is_nan() || c.area <= 0.0 {
        return None;
    }
    let n = geo::unit(n)?;
    // An orthonormal basis (u, v) of the plane.
    let a = if n.x.abs() < 0.9 { Vector3::new(1.0, 0.0, 0.0) } else { Vector3::new(0.0, 1.0, 0.0) };
    let u = geo::unit(a - n * a.dot(n))?;
    let v = n.cross(u);
    let m = |p: Vector3, q: Vector3| -> f64 {
        let mut s = 0.0;
        for (i, pi) in [p.x, p.y, p.z].into_iter().enumerate() {
            for (j, qj) in [q.x, q.y, q.z].into_iter().enumerate() {
                s += pi * c.central[i][j] * qj;
            }
        }
        s / c.area
    };
    let (suu, svv, suv) = (m(u, u), m(v, v), m(u, v));
    let mean = 0.5 * (suu + svv);
    let disc = (0.25 * (suu - svv).powi(2) + suv * suv).sqrt();
    let (l1, l2) = (mean + disc, mean - disc);
    if l2.is_nan() || l2 <= 0.0 {
        return None;
    }
    // A rectangle l x w has variances l^2/12 and w^2/12.
    Some(((l1 / l2).sqrt(), (12.0 * l2).sqrt()))
}

/// Per-face findings: small, sliver, holes and blends.
fn face_findings(view: &dyn TopologyView, f: EntityId, th: &HealThresholds) -> (Vec<HealFinding>, Vec<(EntityId, HealFinding)>) {
    let mut out = Vec::new();
    let one = |code, value: f64, extra: Option<f64>| HealFinding { code, entity: f, entities: vec![f], value, extra };
    if let Ok(m) = view.face_moments(f) {
        if m.area <= th.small_face_area {
            out.push(one(HealCode::SmallFace, m.area, None));
        } else if let Some((aspect, width)) = planar_shape(view, f)
            && aspect >= th.sliver_aspect
            && width <= th.sliver_width
        {
            out.push(one(HealCode::SliverFace, aspect, Some(width)));
        }
        if let Some(b) = blend(view, f, m.area, th) {
            out.push(b);
        }
    }
    (out, holes(view, f, th))
}

/// `(key, finding)` per small inner loop of `f`; the key is the bore face across the loop, so
/// the two ends of a through hole collapse to one finding.
fn holes(view: &dyn TopologyView, f: EntityId, th: &HealThresholds) -> Vec<(EntityId, HealFinding)> {
    let mut out = Vec::new();
    for lp in view.loops(f).filter(|l| l.ring > 0) {
        let edges: Vec<EntityId> = lp.entries.iter().map(|e| e.cell).filter(|&e| !geo::is_degenerate(view, e)).collect();
        let Some(&first) = edges.first() else { continue };
        let pts: Vec<Point3> = edges.iter().flat_map(|&e| (0..HOLE_SAMPLES).filter_map(move |k| geo::edge_sample(view, e, k as f64 / HOLE_SAMPLES as f64))).collect();
        let mut d: f64 = 0.0;
        for (i, p) in pts.iter().enumerate() {
            for q in &pts[i + 1..] {
                d = d.max(geo::dist(*p, *q));
            }
        }
        if d > th.hole_max_diameter {
            continue;
        }
        let across = faces_around(view, first).into_iter().find(|&g| g != f);
        let key = across.unwrap_or(f);
        let mut ents = vec![f];
        ents.extend(across);
        ents.sort_unstable();
        out.push((key, HealFinding { code: HealCode::SmallHole, entity: key, entities: ents, value: d, extra: None }));
    }
    out
}

/// A blend finding for `f`, if it is a partial cylinder tangent to both neighbours.
fn blend(view: &dyn TopologyView, f: EntityId, area: f64, th: &HealThresholds) -> Option<HealFinding> {
    let SurfaceDef::Cylinder(cyl) = crate::pieces::surface_def_of(view, f)? else { return None };
    if cyl.radius > th.blend_max_radius {
        return None;
    }
    let axis = cyl.frame.z;
    let ang = view.tolerance().angular.max(1e-9);
    // The straight rails: lines along the axis shared with another face.
    let mut rails: Vec<(EntityId, EntityId)> = Vec::new();
    for &e in view.face_edges(f) {
        let Some(CurveDef::Line(line)) = geo::edge_curve(view, e).and_then(|(_, cc)| view.shapes().curve_def(cc.curve)) else { continue };
        if line.dir.cross(axis).norm() > ang.max(1e-9) {
            continue;
        }
        let others: Vec<EntityId> = faces_around(view, e).into_iter().filter(|&g| g != f).collect();
        let uses = view.coboundary(e).iter().filter(|i| view.dim_of(i.cell) == Some(Dim::Face)).count();
        if let [g] = others[..]
            && uses == 2
        {
            rails.push((e, g));
        }
    }
    let [(e1, g1), (e2, g2)] = rails[..] else { return None };
    // Both rails tangent to their neighbour: equal face-oriented normals at the rail midpoint.
    for (e, g) in [(e1, g1), (e2, g2)] {
        let p = geo::edge_sample(view, e, 0.5)?;
        let (nf, ..) = geo::face_normal_near(view, f, p)?;
        let (ng, ..) = geo::face_normal_near(view, g, p)?;
        if nf.dot(ng) < BLEND_TANGENT.cos() {
            return None;
        }
    }
    let rail_len = edge_length(view, e1)?;
    let extent = area / (cyl.radius * rail_len);
    if !(extent > 0.0 && extent <= BLEND_MAX_EXTENT) {
        return None;
    }
    let mut ents = vec![f, g1, g2];
    ents.sort_unstable();
    ents.dedup();
    Some(HealFinding { code: HealCode::BlendCandidate, entity: f, entities: ents, value: cyl.radius, extra: Some(extent) })
}

/// One finding per chain of free edges (edges used by a single non-baffle face).
fn open_shells(view: &dyn TopologyView, edges: &[EntityId]) -> Vec<HealFinding> {
    let free: Vec<EntityId> = edges
        .iter()
        .copied()
        .filter(|&e| !geo::is_degenerate(view, e))
        .filter(|&e| {
            let uses: Vec<_> = view.coboundary(e).iter().filter(|i| view.dim_of(i.cell) == Some(Dim::Face)).collect();
            matches!(uses[..], [u] if u.side != Side::Both)
        })
        .collect();
    // Union-find over vertices, deterministic by id order.
    let mut parent: BTreeMap<EntityId, EntityId> = BTreeMap::new();
    fn find(p: &mut BTreeMap<EntityId, EntityId>, x: EntityId) -> EntityId {
        let q = *p.entry(x).or_insert(x);
        if q == x {
            return x;
        }
        let r = find(p, q);
        p.insert(x, r);
        r
    }
    for &e in &free {
        if let Some((s, t)) = view.edge_ends(e) {
            let (rs, rt) = (find(&mut parent, s), find(&mut parent, t));
            if rs != rt {
                parent.insert(rs.max(rt), rs.min(rt));
            }
        }
    }
    let mut chains: BTreeMap<EntityId, Vec<EntityId>> = BTreeMap::new();
    for &e in &free {
        if let Some((s, _)) = view.edge_ends(e) {
            let root = find(&mut parent, s);
            chains.entry(root).or_default().push(e);
        }
    }
    chains
        .into_values()
        .map(|mut es| {
            es.sort_unstable();
            let value = es.iter().filter_map(|&e| edge_length(view, e)).sum();
            HealFinding { code: HealCode::OpenShell, entity: es[0], entities: es, value, extra: None }
        })
        .collect()
}

/// Near gaps between cells that share no face or neighbouring face.
fn near_gaps(view: &dyn TopologyView, entities: &Option<Vec<EntityId>>) -> Vec<HealFinding> {
    let rep = merge_check(view, entities.as_deref().unwrap_or(&[]), &[], &MergeOptions { tol: None, kinds: MergeKinds::default() });
    let near: Vec<_> = rep.pairs.iter().filter(|p| p.status == MergeStatus::Near).collect();
    // A pair on the same face or on neighbouring faces is a feature of that patch (the corner
    // of a notch), not a gap between bodies or between unsewn faces.
    let ring1 = |c: EntityId| -> BTreeSet<EntityId> {
        let own = faces_around(view, c);
        let mut all = own.clone();
        for &f in &own {
            for &e in view.face_edges(f) {
                all.extend(faces_around(view, e));
            }
        }
        all
    };
    let apart = |a: EntityId, b: EntityId| ring1(a).is_disjoint(&faces_around(view, b)) && ring1(b).is_disjoint(&faces_around(view, a));
    let near: Vec<_> = near.into_iter().filter(|p| apart(p.a, p.b)).collect();
    // Report at the highest rank: an edge pair covered by a near face pair, and a vertex pair
    // covered by a near edge pair, add nothing.
    let of_kind = |k: MergeKind| near.iter().filter(move |p| p.kind == k);
    let face_edges = |f: EntityId| view.face_edges(f).to_vec();
    let covered_edges: BTreeSet<(EntityId, EntityId)> = of_kind(MergeKind::Face)
        .flat_map(|p| {
            let (ea, eb) = (face_edges(p.a), face_edges(p.b));
            ea.into_iter().flat_map(move |x| eb.clone().into_iter().map(move |y| (x.min(y), x.max(y))))
        })
        .collect();
    let ends = |e: EntityId| view.edge_ends(e).map(|(s, t)| vec![s, t]).unwrap_or_default();
    let covered_vertices: BTreeSet<(EntityId, EntityId)> = of_kind(MergeKind::Edge)
        .flat_map(|p| {
            let (va, vb) = (ends(p.a), ends(p.b));
            va.into_iter().flat_map(move |x| vb.clone().into_iter().map(move |y| (x.min(y), x.max(y))))
        })
        .collect();
    near.iter()
        .filter(|p| match p.kind {
            MergeKind::Face => true,
            MergeKind::Edge => !covered_edges.contains(&(p.a, p.b)),
            MergeKind::Vertex => !covered_vertices.contains(&(p.a, p.b)),
        })
        .map(|p| HealFinding { code: HealCode::GapNear, entity: p.a, entities: vec![p.a, p.b], value: p.gap, extra: None })
        .collect()
}

/// Scan `view` for the findings of [`HealCode`].
///
/// # Errors
/// [`BooleanError::Invalid`] for a dead entity in `entities` or a non-positive threshold.
pub fn heal_check(view: &dyn TopologyView, o: &HealOptions) -> Result<HealReport, BooleanError> {
    let scope = resolve_scope(view, &o.entities)?;
    let th = thresholds(view, bounding_diagonal(view, &scope.faces), o)?;
    let mut findings: Vec<HealFinding> = Vec::new();
    // Short edges, in parallel with ordered collection.
    let edge_hits: Vec<Option<HealFinding>> = scope
        .edges
        .par_iter()
        .map(|&e| {
            if view.flags(e).contains(CellFlags::DEGENERATE) {
                return None;
            }
            let len = edge_length(view, e)?;
            (len <= th.small_edge).then(|| HealFinding { code: HealCode::SmallEdge, entity: e, entities: vec![e], value: len, extra: None })
        })
        .collect();
    findings.extend(edge_hits.into_iter().flatten());
    let per_face: Vec<_> = scope.faces.par_iter().map(|&f| face_findings(view, f, &th)).collect();
    let mut hole_by_key: BTreeMap<EntityId, HealFinding> = BTreeMap::new();
    for (fs, hs) in per_face {
        findings.extend(fs);
        for (key, h) in hs {
            // The two ends of a bore agree; keep the first (lowest face) and merge the loop faces.
            match hole_by_key.get_mut(&key) {
                None => {
                    hole_by_key.insert(key, h);
                }
                Some(prev) => {
                    prev.entities.extend(h.entities);
                    prev.entities.sort_unstable();
                    prev.entities.dedup();
                }
            }
        }
    }
    findings.extend(hole_by_key.into_values());
    findings.extend(open_shells(view, &scope.edges));
    findings.extend(near_gaps(view, &o.entities));
    findings.sort_by(|a, b| a.code.cmp(&b.code).then(a.entity.cmp(&b.entity)).then(a.value.total_cmp(&b.value)));
    Ok(HealReport { thresholds: th, findings })
}
