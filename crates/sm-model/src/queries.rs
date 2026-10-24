//! Read-only queries: `topo.related`, `model.list`, `model.measure`.

use core::f64::consts::{PI, TAU};
use std::collections::{BTreeMap, BTreeSet};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{Aabb, EntityId, EntityKind, Point3};
use sm_geom_api::{CurveDef, Interval, SurfaceDef};
use sm_layer_api::ModelLayer;
use sm_op_api::{OpError, QueryCtx, State, TypedQuery};
use sm_topo_api::{Carrier, CellFlags, Dim, QueryError, Side, TopologyView, TopologyViewExt};

fn model_layer(state: &State) -> Result<&ModelLayer, OpError> {
    state.layer::<ModelLayer>().ok_or_else(|| OpError::MissingLayer(<ModelLayer as sm_op_api::LayerType>::NAME.to_owned()))
}

fn topo_dim(kind: EntityKind) -> Result<Dim, OpError> {
    Dim::of_real_kind(kind).ok_or_else(|| OpError::Invalid(format!("`{}` is not a topology kind", kind.name())))
}

/// `topo.related {kind, within}`: the live cells of `kind` related to the cells `within`.
///
/// Related means reachable through the cell hierarchy in either direction: a lower kind is
/// found by descending the boundary rows (volume -> faces -> edges -> vertices; body ->
/// volumes), a higher kind by ascending the coboundary rows (vertex -> edges -> faces ->
/// volumes -> bodies). The same kind returns the live cells of `within` themselves. The result
/// is sorted by id and has no duplicates. `within` is checked against the topology, not the
/// entity table.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct TopoRelated {
    /// The kind to return (`TopoVertex` .. `TopoBody`).
    pub kind: EntityKind,
    /// Source cells.
    #[schemars(extend("x-entity-kind" = ["TopoVertex", "TopoEdge", "TopoFace", "TopoVolume", "TopoBody"]))]
    pub within: Vec<EntityId>,
}

impl TypedQuery for TopoRelated {
    const NAME: &'static str = "topo.related";
    type Output = Vec<EntityId>;

    fn run(&self, state: &State, _cx: &QueryCtx<'_>) -> Result<Vec<EntityId>, OpError> {
        let topo = model_layer(state)?.topo();
        let target = topo_dim(self.kind)?;
        let mut found: BTreeSet<EntityId> = BTreeSet::new();
        let mut seen: BTreeSet<EntityId> = BTreeSet::new();
        let mut stack: Vec<EntityId> = Vec::new();
        for &s in &self.within {
            let ds = Dim::of_real_kind(s.kind()).ok_or(OpError::WrongKind { id: s, expected: EntityKind::TopoVolume })?;
            if !topo.is_alive(s) {
                return Err(OpError::UnknownEntity(s));
            }
            if ds == target {
                found.insert(s);
            } else if seen.insert(s) {
                stack.push(s);
            }
        }
        let down = |d: Dim| d > target;
        while let Some(c) = stack.pop() {
            let dc = Dim::of_real_kind(c.kind()).ok_or(OpError::Failed(format!("{c} is not a topology cell")))?;
            let next = if down(dc) { topo.boundary(c) } else { topo.coboundary(c) };
            for inc in next {
                let n = inc.cell;
                let Some(dn) = Dim::of_real_kind(n.kind()) else { continue };
                if dn == target {
                    found.insert(n);
                } else if ((down(dc) && dn > target) || (!down(dc) && dn < target)) && seen.insert(n) {
                    stack.push(n);
                }
            }
        }
        Ok(found.into_iter().collect())
    }
}

/// `model.list {kind}`: every live entity of `kind` owned by the model layer, ascending.
///
/// Topology kinds come from the cell complex; `Color`, `Group`, `Boundary`, `Prototype`,
/// `Lattice`, `Instance`, `Cluster` and `PartitionCell` from the model tables.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ModelList {
    /// The kind to list.
    pub kind: EntityKind,
}

impl TypedQuery for ModelList {
    const NAME: &'static str = "model.list";
    type Output = Vec<EntityId>;

    fn run(&self, state: &State, _cx: &QueryCtx<'_>) -> Result<Vec<EntityId>, OpError> {
        let layer = model_layer(state)?;
        if let Some(d) = Dim::of_real_kind(self.kind) {
            return Ok(layer.topo().cells(d).to_vec());
        }
        let m = layer.model();
        let ins = m.instancing();
        let mut ids: Vec<EntityId> = match self.kind {
            EntityKind::Color => m.colors().map(|r| r.id).collect(),
            EntityKind::Group => m.groups().map(|r| r.id).collect(),
            EntityKind::Boundary => m.boundaries().map(|r| r.id).collect(),
            EntityKind::Prototype => ins.prototypes.keys().copied().collect(),
            EntityKind::Lattice => ins.lattices.keys().copied().collect(),
            EntityKind::Instance => ins.instances.keys().copied().collect(),
            EntityKind::Cluster => ins.clusters.keys().copied().collect(),
            EntityKind::PartitionCell => m.decomposition().cells().map(|p| p.id).collect(),
            other => return Err(OpError::Invalid(format!("`{}` is not owned by the model layer", other.name()))),
        };
        ids.sort_unstable();
        Ok(ids)
    }
}

/// An axis-aligned box.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct BBox {
    /// Minimum corner.
    pub lo: [f64; 3],
    /// Maximum corner.
    pub hi: [f64; 3],
}

/// The result of `model.measure`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Measurement {
    /// The measured entity.
    pub entity: EntityId,
    /// Arc length (edges).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub length: Option<f64>,
    /// Area: of a face; the boundary area of a volume or body. Faces internal to the measured
    /// set (shared by two member volumes, or a baffle) are not counted.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub area: Option<f64>,
    /// Volume (volumes, bodies).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub volume: Option<f64>,
    /// Centroid of the volume (volumes) or of the area (faces).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub centroid: Option<[f64; 3]>,
    /// Bounding box.
    pub bbox: BBox,
}

/// `model.measure {entity}`: length, area, volume, centroid and bounding box of a topology cell.
///
/// Face and volume integrals are `TopologyView::face_moments` / `volume_moments` (Gauss
/// quadrature, far below the model tolerance); a whole sphere (a single face with two pole edges
/// and a seam) has no quadrature in the view and uses the closed forms `4 pi r^2` and
/// `4/3 pi r^3`. Bounding boxes come from the edges: exact for lines, circles and ellipses
/// (analytic extremes), sampled (256 points per edge) for NURBS curves; a whole sphere adds
/// `centre +- r`. A face or volume bounded by a partial patch of a curved surface can have a
/// bounding box smaller than the true one (the patch interior is not sampled).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ModelMeasure {
    /// The cell to measure.
    #[schemars(extend("x-entity-kind" = ["TopoVertex", "TopoEdge", "TopoFace", "TopoVolume", "TopoBody"]))]
    pub entity: EntityId,
}

fn qerr(e: QueryError) -> OpError {
    OpError::Failed(format!("measure: {e}"))
}

fn arr(p: Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}

/// A whole sphere face: one loop, two degenerate coedges and every other edge used on both
/// sides. Returns the sphere.
fn whole_sphere(topo: &dyn TopologyView, f: EntityId) -> Option<(Point3, f64)> {
    let Carrier::Surface(sc) = topo.carrier(f) else { return None };
    let SurfaceDef::Sphere(s) = topo.shapes().surface_def(sc.surface)? else { return None };
    if topo.loops(f).count() != 1 {
        return None;
    }
    let mut degenerate = 0;
    let mut sides: std::collections::BTreeMap<EntityId, (bool, bool)> = Default::default();
    for co in topo.coedges(f) {
        if topo.flags(co.key.edge).contains(CellFlags::DEGENERATE) {
            degenerate += 1;
        } else {
            let e = sides.entry(co.key.edge).or_default();
            match co.key.side {
                Side::Left => e.0 = true,
                _ => e.1 = true,
            }
        }
    }
    (degenerate == 2 && !sides.is_empty() && sides.values().all(|&(l, r)| l && r)).then_some((s.frame.origin, s.radius))
}

/// Extreme angles of `c + a cos t x + b sin t y` along axis `i` inside `range`.
fn ellipse_extremes(frame: &sm_geom_api::Frame, a: f64, b: f64, range: Interval, bb: &mut Aabb) {
    let axes = [(frame.x.x, frame.y.x), (frame.x.y, frame.y.y), (frame.x.z, frame.y.z)];
    let eval = |t: f64| frame.origin + frame.x * (a * t.cos()) + frame.y * (b * t.sin());
    for (xi, yi) in axes {
        let t0 = (b * yi).atan2(a * xi);
        for t in [t0, t0 + PI] {
            let span = range.hi - range.lo;
            let dt = (t - range.lo).rem_euclid(TAU);
            if dt <= span {
                bb.extend(eval(range.lo + dt));
            }
        }
    }
}

fn edge_bbox(topo: &dyn TopologyView, e: EntityId) -> Aabb {
    let mut bb = Aabb::empty();
    for inc in topo.boundary(e) {
        if let Carrier::Point(p) = topo.carrier(inc.cell) {
            bb.extend(*p);
        }
    }
    if let Carrier::Curve(cc) = topo.carrier(e) {
        let shapes = topo.shapes();
        if let (Some(def), Some(curve)) = (shapes.curve_def(cc.curve), shapes.curve(cc.curve)) {
            match def {
                CurveDef::Line(_) => {}
                CurveDef::Circle(c) => ellipse_extremes(&c.frame, c.radius, c.radius, cc.range, &mut bb),
                CurveDef::Ellipse(el) => ellipse_extremes(&el.frame, el.a, el.b, cc.range, &mut bb),
                _ => {
                    const N: usize = 256;
                    for k in 0..=N {
                        bb.extend(curve.eval(cc.range.lo + (cc.range.hi - cc.range.lo) * (k as f64 / N as f64)));
                    }
                }
            }
        }
    }
    bb
}

fn face_bbox(topo: &dyn TopologyView, f: EntityId) -> Aabb {
    let mut bb = Aabb::empty();
    for &e in topo.face_edges(f) {
        bb = bb.union(&edge_bbox(topo, e));
    }
    if let Some((c, r)) = whole_sphere(topo, f) {
        bb = bb.union(&Aabb { lo: Point3::new(c.x - r, c.y - r, c.z - r), hi: Point3::new(c.x + r, c.y + r, c.z + r) });
    }
    bb
}

fn face_area(topo: &dyn TopologyView, f: EntityId) -> Result<(f64, Option<Point3>), OpError> {
    match topo.face_moments(f) {
        Ok(m) => Ok((m.area, m.centroid())),
        Err(QueryError::Unsupported(_)) if whole_sphere(topo, f).is_some() => {
            let (c, r) = whole_sphere(topo, f).ok_or_else(|| OpError::Failed("sphere vanished".into()))?;
            let sign = match topo.carrier(f) {
                Carrier::Surface(sc) => sc.sense.sign(),
                _ => 1.0,
            };
            Ok((sign * 4.0 * PI * r * r, Some(c)))
        }
        Err(e) => Err(qerr(e)),
    }
}

/// `(volume, centroid)` of a volume.
fn volume_value(topo: &dyn TopologyView, v: EntityId) -> Result<(f64, Option<Point3>), OpError> {
    match topo.volume_moments(v) {
        Ok(m) => Ok((m.volume, m.centroid())),
        Err(QueryError::Unsupported(_)) => {
            let faces = topo.volume_faces(v);
            if let [f] = faces
                && let Some((c, r)) = whole_sphere(topo, *f)
                && topo.boundary(v).iter().all(|i| i.side == Side::Left)
            {
                return Ok((4.0 / 3.0 * PI * r * r * r, Some(c)));
            }
            Err(OpError::Failed("measure: this volume has faces on a surface with poles that are not a whole sphere".into()))
        }
        Err(e) => Err(qerr(e)),
    }
}

impl TypedQuery for ModelMeasure {
    const NAME: &'static str = "model.measure";
    type Output = Measurement;

    fn run(&self, state: &State, _cx: &QueryCtx<'_>) -> Result<Measurement, OpError> {
        let topo = model_layer(state)?.topo();
        let e = self.entity;
        let dim = Dim::of_real_kind(e.kind()).ok_or(OpError::WrongKind { id: e, expected: EntityKind::TopoVolume })?;
        if !topo.is_alive(e) {
            return Err(OpError::UnknownEntity(e));
        }
        let mut out = Measurement { entity: e, length: None, area: None, volume: None, centroid: None, bbox: BBox { lo: [0.0; 3], hi: [0.0; 3] } };
        let bb = match dim {
            Dim::Vertex => match topo.carrier(e) {
                Carrier::Point(p) => Aabb::from_point(*p),
                _ => return Err(OpError::Failed(format!("{e} has no point"))),
            },
            Dim::Edge => {
                out.length = Some(match topo.carrier(e) {
                    Carrier::Curve(cc) => {
                        let curve = topo.shapes().curve(cc.curve).ok_or_else(|| OpError::Failed(format!("curve {} is not in the store", cc.curve)))?;
                        curve.length(cc.range, topo.tolerance().linear).map_err(|er| OpError::Failed(format!("measure: {er}")))?
                    }
                    _ => 0.0,
                });
                edge_bbox(topo, e)
            }
            Dim::Face => {
                let (a, c) = face_area(topo, e)?;
                out.area = Some(a);
                out.centroid = c.map(arr);
                face_bbox(topo, e)
            }
            Dim::Volume | Dim::Body => {
                let volumes: Vec<EntityId> = if dim == Dim::Volume {
                    vec![e]
                } else {
                    topo.boundary(e).iter().map(|i| i.cell).filter(|c| c.kind() == EntityKind::TopoVolume).collect()
                };
                let (mut vol, mut weighted, mut first) = (0.0, 0.0, [0.0; 3]);
                // Face uses over the member volumes (a `Both` use counts twice): a face with
                // exactly one use is on the boundary; an interface or baffle is internal.
                let mut uses: BTreeMap<EntityId, u32> = BTreeMap::new();
                for &v in &volumes {
                    let (vv, c) = volume_value(topo, v)?;
                    vol += vv;
                    // A volume without a centroid stays out of the centroid average.
                    if let Some(c) = c {
                        weighted += vv;
                        for (k, x) in arr(c).iter().enumerate() {
                            first[k] += vv * x;
                        }
                    }
                    for i in topo.boundary(v) {
                        *uses.entry(i.cell).or_default() += if i.side == Side::Both { 2 } else { 1 };
                    }
                }
                let mut area = 0.0;
                let mut bb = Aabb::empty();
                for (&f, &n) in &uses {
                    if n == 1 {
                        area += face_area(topo, f)?.0;
                        bb = bb.union(&face_bbox(topo, f));
                    }
                }
                out.volume = Some(vol);
                out.area = Some(area);
                if weighted.abs() > 0.0 {
                    out.centroid = Some(first.map(|x| x / weighted));
                }
                bb
            }
        };
        if bb.is_empty() {
            return Err(OpError::Failed(format!("{e} has no geometry to bound")));
        }
        out.bbox = BBox { lo: arr(bb.lo), hi: arr(bb.hi) };
        Ok(out)
    }
}
