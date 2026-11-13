//! `mesh.preserve_volume` (W4-HEX-VOLFIT, ruling D4): move the nodes of circles to the equal-area
//! radius so that a linear n-gon holds the area of its circle.
//!
//! A circle meshed with `n` nodes at angular gaps `g_i` bounds a polygon of area `r^2/2 sum sin g_i`.
//! Scaling its nodes radially by `f = sqrt(2 pi / sum sin g_i)` gives the polygon the area `pi r^2`
//! (for equal gaps `f^2 = (2 pi / n) / sin(2 pi / n)`). Where region boundaries (sector cuts) meet
//! the circle, each wedge between two of them is made exact on its own: see [`MeshPreserveVolume`].
//! The nodes of a cylinder face move with the circles, so a prism over the circle gets exactly the
//! volume of the cylinder. Interface nodes are shared, so both sides move together and the mesh
//! stays conformal.

use std::collections::BTreeMap;
use std::f64::consts::TAU;
use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_layer_api::{LayerResultExt, MeshLayer, ModelLayer};
use sm_mesh_api::{CellIx, MeshEntityKind, MeshRead, PointIx, VertexIx, cell_volume};
use sm_op_api::{ApplyCtx, OpError, TypedOp};
use sm_scheme_api::geom::{edge_curve, edge_point};
use sm_topo_api::TopologyView;

/// Relative tolerance for "a node lies on its model circle".
const ON_CIRCLE: f64 = 1e-6;
/// Relative tolerance for "two edges lie on the same circle" and for full coverage.
const SAME: f64 = 1e-7;

/// `mesh.preserve_volume`: move the nodes classified on circular edges (`curves`), or on circular
/// cylinder faces (`surfaces`), radially to the equal-area radius, so that the linear mesh of each
/// round region has the exact volume of the cylinder it approximates. Opt-in; recorded in the history
/// and replayed as an op (no strategy is consulted); deterministic; the node count and every id stay.
///
/// Give exactly one of `curves` (`TopoEdge` ids) and `surfaces` (`TopoFace` ids). Every circle is
/// moved to its own radius, so concentric ring cuts keep each ring's volume exact. A circle with no
/// region boundary on it is scaled by `f = sqrt(2 pi / sum sin g_i)` from the angular gaps `g_i`
/// of its nodes. Where sector cuts meet a circle (a vertex with an edge in the circle's plane), the
/// junction nodes get `f` and the nodes between two junctions share one radius, solved so that the
/// polygon of that wedge has the area of its circular sector; so each sector keeps its volume whatever
/// the node spacing (a wedge of one interval has no free node and is not exact). With `surfaces`, the
/// nodes of the face (axial layers, seams) take the scale of the circle node in their column; with
/// `curves` only the circle nodes move, which suits a single axial layer. A circle split into arcs
/// must be selected whole: pass every arc, or every face that bounds it.
///
/// The op refuses a selection whose nodes are not on the model circle (for example one already
/// preserved: it is not idempotent), a circle with fewer than 3 nodes or a gap of pi or more, a face
/// that is not a cylinder about its circle, and any move that inverts a cell. Only the circle and
/// face nodes move; the interior keeps its place, so the shift is about 5% of the radius at 8
/// nodes per circle and 1.3% at 16.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshPreserveVolume {
    /// The mesh.
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: EntityId,
    /// Circular edges to move (whole circles).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoEdge"]))]
    pub curves: Option<Vec<EntityId>>,
    /// Cylinder faces to move, with their boundary circles.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoFace"]))]
    pub surfaces: Option<Vec<EntityId>>,
}

/// One circle found in the selection.
#[derive(Debug, Clone)]
struct Circle {
    center: Point3,
    axis: Vector3,
    radius: f64,
    edges: Vec<EntityId>,
    coverage: f64,
}

/// What one circle did, for the report.
#[derive(Debug, Serialize)]
struct CircleReport {
    edges: Vec<EntityId>,
    radius: f64,
    nodes: usize,
    junctions: usize,
    max_scale: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    circles: Vec<CircleReport>,
    moved: usize,
}

fn bad(msg: impl Into<String>) -> OpError {
    OpError::Invalid(format!("mesh.preserve_volume: {}", msg.into()))
}

/// Circle through three points of the edge, checked against a fourth; `None` if not a circle.
fn circle_of(t: &dyn TopologyView, e: EntityId) -> Option<Circle> {
    let (curve, cc) = edge_curve(t, e).ok()?;
    let period = curve.domain().period?;
    let pt = |s: f64| edge_point(t, e, s).ok();
    let (a, b, c, d) = (pt(0.13)?, pt(0.41)?, pt(0.77)?, pt(0.29)?);
    let (u, v) = (b - a, c - a);
    let n = u.cross(v);
    let n2 = n.dot(n);
    if n2 < 1e-24 {
        return None;
    }
    let off = (v.cross(n) * u.dot(u) + n.cross(u) * v.dot(v)) * (0.5 / n2);
    let center = a + off;
    let radius = off.norm();
    let axis = n * (1.0 / n2.sqrt());
    let tol = SAME * radius.max(f64::MIN_POSITIVE);
    if radius <= 0.0
        || ((d - center).norm() - radius).abs() > tol
        || (d - center).dot(axis).abs() > tol
    {
        return None;
    }
    let coverage = (cc.range.hi - cc.range.lo).abs() / period;
    Some(Circle {
        center,
        axis,
        radius,
        edges: vec![e],
        coverage,
    })
}

fn same_circle(a: &Circle, b: &Circle) -> bool {
    let tol = SAME * a.radius;
    (a.center - b.center).norm() <= tol
        && (a.radius - b.radius).abs() <= tol
        && a.axis.cross(b.axis).norm() <= SAME
}

/// Group circular edges (ascending id) into whole circles.
fn group_circles(t: &dyn TopologyView, edges: &[EntityId]) -> Result<Vec<Circle>, OpError> {
    let mut out: Vec<Circle> = Vec::new();
    for &e in edges {
        let c = circle_of(t, e).ok_or_else(|| bad(format!("edge {e} is not a circle")))?;
        match out.iter_mut().find(|g| same_circle(g, &c)) {
            Some(g) => {
                g.edges.push(e);
                g.coverage += c.coverage;
            }
            None => out.push(c),
        }
    }
    for g in &out {
        if (g.coverage - 1.0).abs() > 1e-6 {
            return Err(bad(format!(
                "the selected edges {:?} cover {:.4} of their circle; select every arc of it \
                 (or every face that bounds it)",
                g.edges, g.coverage
            )));
        }
    }
    Ok(out)
}

/// In-plane angle of `p` about the circle, from a fixed reference direction.
fn frame_of(axis: Vector3) -> (Vector3, Vector3) {
    let h = if axis.x.abs() < 0.6 {
        Vector3::new(1.0, 0.0, 0.0)
    } else {
        Vector3::new(0.0, 1.0, 0.0)
    };
    let u = h - axis * h.dot(axis);
    let u = u * (1.0 / u.norm());
    (u, axis.cross(u))
}

/// A node of a circle: its mesh vertex, angle about the axis, and whether it is on a region boundary.
#[derive(Clone, Copy)]
struct Node {
    v: usize,
    theta: f64,
    junction: bool,
}

/// Radial scale `rho / r` per node (same order as `nodes`, sorted by angle).
///
/// Without junctions every node gets `f = sqrt(2 pi / sum sin g_i)`. With junctions (nodes where a
/// region boundary meets the circle) the junctions get that `f` and the nodes strictly between two
/// consecutive junctions share one radius, solved so that the polygon of the wedge has the area
/// `r^2 theta / 2` of its circular sector. A wedge of one interval has no free node and keeps `f`.
fn scales(nodes: &[Node], what: &str) -> Result<Vec<f64>, OpError> {
    let n = nodes.len();
    if n < 3 {
        return Err(bad(format!(
            "{what} has {n} node(s); at least 3 are needed"
        )));
    }
    let gap = |i: usize| {
        if i + 1 < n {
            nodes[i + 1].theta - nodes[i].theta
        } else {
            nodes[0].theta + TAU - nodes[i].theta
        }
    };
    let mut sum = 0.0;
    for i in 0..n {
        let g = gap(i);
        if g >= std::f64::consts::PI {
            return Err(bad(format!(
                "{what} has an angular gap of {g:.3} rad; refine it first"
            )));
        }
        sum += g.sin();
    }
    let f = (TAU / sum).sqrt();
    let mut out = vec![f; n];
    let Some(first) = nodes.iter().position(|x| x.junction) else {
        return Ok(out);
    };
    // Walk the wedges from junction to junction, cyclically.
    let mut start = first;
    loop {
        let mut m = 1;
        while !nodes[(start + m) % n].junction {
            m += 1;
        }
        if m >= 2 {
            let g: Vec<f64> = (0..m).map(|k| gap((start + k) % n)).collect();
            let theta: f64 = g.iter().sum();
            let inner: f64 = g[1..m - 1].iter().map(|x| x.sin()).sum();
            let b = (g[0].sin() + g[m - 1].sin()) * f;
            // inner rho^2 + b rho - theta = 0 in units of r (rho = scale).
            let rho = if inner.abs() < 1e-14 {
                theta / b
            } else {
                (-b + (b * b + 4.0 * inner * theta).sqrt()) / (2.0 * inner)
            };
            for k in 1..m {
                out[(start + k) % n] = rho;
            }
        }
        start = (start + m) % n;
        if start == first {
            break;
        }
    }
    Ok(out)
}

/// True if a region boundary (an edge in the plane of the circle) ends at topology vertex `tv`.
fn is_junction(t: &dyn TopologyView, tv: EntityId, own: &[EntityId], axis: Vector3) -> bool {
    t.coboundary(tv)
        .iter()
        .filter(|i| i.cell.kind() == EntityKind::TopoEdge && !own.contains(&i.cell))
        .any(|i| {
            let (Ok(p0), Ok(p1)) = (edge_point(t, i.cell, 0.0), edge_point(t, i.cell, 1.0)) else {
                return false;
            };
            let d = p1 - p0;
            let len = d.norm();
            len > 1e-12 && d.dot(axis).abs() <= 1e-9 * len
        })
}

/// A node's move: the axis line and the radial scale.
#[derive(Clone, Copy)]
struct Claim {
    center: Point3,
    axis: Vector3,
    s: f64,
}

/// One circle's nodes with their scales, for the lookup of column nodes by angle.
struct Table {
    radius: f64,
    center: Point3,
    axis: Vector3,
    u: Vector3,
    w: Vector3,
    rows: Vec<(f64, f64)>,
}

impl Table {
    fn angle(&self, d: Vector3) -> f64 {
        d.dot(self.w).atan2(d.dot(self.u))
    }
    /// Scale of the circle node at the angle of `d`, within 1e-9 rad.
    fn scale_at(&self, d: Vector3) -> Option<f64> {
        let th = self.angle(d);
        self.rows
            .iter()
            .find(|(a, _)| {
                let x = (a - th).rem_euclid(TAU);
                x < 1e-9 || TAU - x < 1e-9
            })
            .map(|r| r.1)
    }
}

/// New positions `(vertex, point)` and a report, for the selection. Pure.
fn plan(
    m: &dyn MeshRead,
    t: &dyn TopologyView,
    curves: &[EntityId],
    surfaces: &[EntityId],
) -> Result<(Vec<(usize, Point3)>, Report), OpError> {
    let mut edges: Vec<EntityId> = curves.to_vec();
    for &f in surfaces {
        if !t.is_alive(f) || f.kind() != EntityKind::TopoFace {
            return Err(bad(format!("{f} is not a live face")));
        }
        let circ: Vec<EntityId> = t
            .face_edges(f)
            .iter()
            .copied()
            .filter(|&e| circle_of(t, e).is_some())
            .collect();
        if circ.is_empty() {
            return Err(bad(format!("face {f} has no circular boundary edge")));
        }
        edges.extend(circ);
    }
    for &e in curves {
        if !t.is_alive(e) || e.kind() != EntityKind::TopoEdge {
            return Err(bad(format!("{e} is not a live edge")));
        }
    }
    edges.sort();
    edges.dedup();
    let circles = group_circles(t, &edges)?;

    // Mesh vertices by the model entity they are classified on.
    let mut on: BTreeMap<EntityId, Vec<usize>> = BTreeMap::new();
    for v in 0..m.count(MeshEntityKind::Vertex) {
        on.entry(m.vertex_class(VertexIx::new(v)).entity)
            .or_default()
            .push(v);
    }
    let nodes_of = |id: EntityId| on.get(&id).map_or(&[][..], Vec::as_slice);

    let mut claims: BTreeMap<usize, Claim> = BTreeMap::new();
    let mut report = Report {
        circles: Vec::new(),
        moved: 0,
    };
    let mut tables: Vec<Table> = Vec::new();
    for c in &circles {
        let mut vs: Vec<(usize, bool)> = Vec::new();
        for &e in &c.edges {
            vs.extend(nodes_of(e).iter().map(|&v| (v, false)));
            for inc in t.boundary(e) {
                if inc.cell.kind() == EntityKind::TopoVertex {
                    let j = is_junction(t, inc.cell, &c.edges, c.axis);
                    vs.extend(nodes_of(inc.cell).iter().map(|&v| (v, j)));
                }
            }
        }
        vs.sort_unstable();
        vs.dedup();
        let (u, w) = frame_of(c.axis);
        let mut nodes = Vec::with_capacity(vs.len());
        for &(v, junction) in &vs {
            let d = m.point(v) - c.center;
            let dev = ((d - c.axis * d.dot(c.axis)).norm() - c.radius)
                .abs()
                .max(d.dot(c.axis).abs());
            if dev > ON_CIRCLE * c.radius {
                return Err(bad(format!(
                    "node {v} of circle {:?} is {dev:.3e} off the model circle (radius {}); \
                     was the circle already preserved or smoothed off its curve?",
                    c.edges, c.radius
                )));
            }
            nodes.push(Node {
                v,
                theta: d.dot(w).atan2(d.dot(u)),
                junction,
            });
        }
        nodes.sort_by(|a, b| a.theta.total_cmp(&b.theta).then(a.v.cmp(&b.v)));
        let sc = scales(&nodes, &format!("circle {:?}", c.edges))?;
        for (nd, s) in nodes.iter().zip(&sc) {
            claims.insert(
                nd.v,
                Claim {
                    center: c.center,
                    axis: c.axis,
                    s: *s,
                },
            );
        }
        let smax = sc.iter().copied().fold(f64::MIN, f64::max);
        report.circles.push(CircleReport {
            edges: c.edges.clone(),
            radius: c.radius,
            nodes: nodes.len(),
            junctions: nodes.iter().filter(|x| x.junction).count(),
            max_scale: smax,
        });
        tables.push(Table {
            radius: c.radius,
            center: c.center,
            axis: c.axis,
            u,
            w,
            rows: nodes
                .iter()
                .zip(&sc)
                .map(|(nd, s)| (nd.theta, *s))
                .collect(),
        });
    }

    // Face nodes (axial layers, seams) take the scale of the circle node in their column.
    for &s in surfaces {
        let own: Vec<usize> = circles
            .iter()
            .enumerate()
            .filter(|(_, c)| t.face_edges(s).iter().any(|e| c.edges.contains(e)))
            .map(|(i, _)| i)
            .collect();
        let tab = &tables[own[0]];
        let mut ids: Vec<EntityId> = vec![s];
        for &e in t.face_edges(s) {
            ids.push(e);
            for inc in t.boundary(e) {
                if inc.cell.kind() == EntityKind::TopoVertex {
                    ids.push(inc.cell);
                }
            }
        }
        for id in ids {
            for &v in nodes_of(id) {
                if claims.contains_key(&v) {
                    continue;
                }
                let d = m.point(v) - tab.center;
                let rho = (d - tab.axis * d.dot(tab.axis)).norm();
                if (rho - tab.radius).abs() > ON_CIRCLE * tab.radius {
                    return Err(bad(format!(
                        "node {v} of face {s} is at distance {rho} from the axis, not the radius {}: \
                         the face is not a cylinder about its circle",
                        tab.radius
                    )));
                }
                let sc = tab.scale_at(d).ok_or_else(|| {
                    bad(format!("node {v} of face {s} is not in a column of a node of its circle; the face mesh is not a sweep of the circle"))
                })?;
                claims.insert(
                    v,
                    Claim {
                        center: tab.center,
                        axis: tab.axis,
                        s: sc,
                    },
                );
            }
        }
    }

    let mut out = Vec::with_capacity(claims.len());
    for (&v, c) in &claims {
        let p = m.point(v);
        let d = p - c.center;
        let radial = d - c.axis * d.dot(c.axis);
        out.push((v, p + radial * (c.s - 1.0)));
    }
    report.moved = out.len();
    Ok((out, report))
}

impl TypedOp for MeshPreserveVolume {
    const NAME: &'static str = "mesh.preserve_volume";
    const NO_OUTPUTS: bool = true;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        ctx.resolve_kind(self.mesh, EntityKind::Mesh)?;
        let (curves, surfaces) = match (&self.curves, &self.surfaces) {
            (Some(c), None) if !c.is_empty() => (c.as_slice(), &[][..]),
            (None, Some(s)) if !s.is_empty() => (&[][..], s.as_slice()),
            _ => {
                return Err(bad(
                    "give exactly one non-empty list, `curves` or `surfaces`",
                ));
            }
        };
        let (moves, report, host, alloc) = {
            let meshes = ctx.layer::<MeshLayer>()?;
            let entry = meshes
                .get(self.mesh)
                .ok_or(OpError::UnknownEntity(self.mesh))?;
            let model = ctx.layer::<ModelLayer>()?;
            let (moves, report) = plan(entry.mesh(), model.topo(), curves, surfaces)?;
            (
                moves,
                report,
                entry.mesh.clone_host(),
                meshes.alloc().clone(),
            )
        };
        let mut host = host;
        let before: Vec<f64> = (0..host.count(MeshEntityKind::Cell))
            .map(|c| cell_volume(&*host, CellIx::new(c)))
            .collect();
        for (v, p) in &moves {
            host.set_point(PointIx::new(*v), *p)
                .map_err(|e| bad(e.to_string()))?;
        }
        for (c, b) in before.iter().enumerate() {
            let a = cell_volume(&*host, CellIx::new(c));
            if *b > 0.0 && a <= 0.0 {
                return Err(bad(format!(
                    "the move would invert cell {c}; use more nodes on the circle or move fewer circles"
                )));
            }
        }
        ctx.layer_mut::<MeshLayer>()?
            .replace_mesh(self.mesh, Arc::from(host), alloc)
            .or_op()?;
        ctx.info(
            "preserve_volume.report",
            serde_json::to_string(&report).unwrap_or_default(),
            &[self.mesh],
        );
        Ok(())
    }
}
