//! The face chart: boundary loops lifted from the surface into a plane in which all exact
//! predicates run.
//!
//! The chart coordinates of a surface point `(u, v)` are
//!
//! ```text
//! w = (s (u - u0), v - v0)        s = -1 if the carrier sense is Reversed (loops stay CCW)
//! z = L^T w                       G = L L^T = J^T M J at the loop centroid (Cholesky)
//! q = scale * z + c               scale a power of two, c puts the loops into [1, 2]
//! ```
//!
//! `M` is the size metric at the loop centroid (`I / h^2` without a size field), so one chart
//! unit is about one target edge length. Because `scale` is a power of two and `c` a single
//! translation, all loop coordinates are plain doubles of similar magnitude; the kernel works on
//! exactly those doubles.
//!
//! Poles: a degenerate coedge (a collapsed iso-line) is replaced in the chart by a ring of
//! face-owned nodes around the pole, and the pole fan (`m` explicit triangles
//! `(pole, ring_i, ring_{i+1})`) is emitted outside the kernel, so no triangle holds the pole
//! node twice. See `DESIGN.md`.

use crate::cdt::Geometry2;
use crate::error::TriError;
use crate::util::pow2_ceil;
use core::f64::consts::TAU;
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{ParametricSurface, Uv};
use sm_scheme_api::{GenCtx, NodeKey, SizeField};
use sm_topo_api::{Carrier, CellFlags, Sense, TopologyViewExt};

/// One node of a lifted boundary loop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChartNode {
    /// The node's key. Pole-ring nodes are face-owned (`new` is true).
    pub key: NodeKey,
    /// Surface parameters on the loop's own (unwrapped) branch.
    pub uv: [f64; 2],
    /// Chart coordinates.
    pub q: [f64; 2],
    /// True for nodes this face creates (pole ring), false for nodes of earlier jobs.
    pub new: bool,
}

/// The fan around a pole: `pole` is joined to every consecutive pair of `path`.
#[derive(Clone, Debug, PartialEq)]
pub struct PoleFan {
    /// The pole's vertex node.
    pub pole: NodeKey,
    /// The ring from the node before the pole to the node after it, in loop order.
    pub path: Vec<NodeKey>,
}

/// A boundary loop in the chart.
#[derive(Clone, Debug, PartialEq)]
pub struct ChartLoop {
    /// Loop index (0 = outer).
    pub ring: u16,
    /// Nodes in loop order (the face lies on the left); the first is not repeated at the end.
    pub nodes: Vec<ChartNode>,
    /// Pole fans in this loop.
    pub fans: Vec<PoleFan>,
}

/// Newell plane frame of a face without a surface carrier.
#[derive(Clone, Copy, Debug)]
struct PlaneFrame {
    o: Point3,
    e1: Vector3,
    e2: Vector3,
}

/// The chart of one face, also the [`Geometry2`] of its refinement.
pub struct FaceChart<'a> {
    face: EntityId,
    surface: Option<&'a dyn ParametricSurface>,
    plane: Option<PlaneFrame>,
    flip: f64,
    uv0: [f64; 2],
    /// Upper triangular `L^T = [[l11, l21], [0, l22]]`.
    lt: [f64; 3],
    inv: [f64; 3],
    scale: f64,
    shift: [f64; 2],
    periodic_u: Option<f64>,
    isometric: bool,
    aniso: bool,
    size: Option<&'a dyn SizeField>,
    h0: f64,
    next_index: u32,
}

fn dot(a: Vector3, b: Vector3) -> f64 {
    a.dot(b)
}

impl<'a> FaceChart<'a> {
    /// Lift the boundary loops of `face` into a chart.
    ///
    /// Boundary uv come from [`GenCtx::loop_uvs`] (pcurves first, projection fallback with
    /// periodic unwrapping); a face without a surface carrier uses its Newell plane.
    ///
    /// # Errors
    /// [`TriError::Lift`] when a node cannot be placed, [`TriError::OpenLoop`] when a loop does
    /// not close, [`TriError::Unsupported`] for pole configurations other than a collapsed
    /// iso-line with matching neighbours, [`TriError::Degenerate`] for zero-size faces.
    pub fn lift(
        cx: &'a GenCtx<'_>,
        face: EntityId,
    ) -> Result<(FaceChart<'a>, Vec<ChartLoop>), TriError> {
        let topo = cx.input.topo;
        let loops = cx
            .loops(face)
            .map_err(|e| TriError::Degenerate(e.to_string()))?;
        if loops.is_empty() {
            return Err(TriError::Degenerate("face has no boundary nodes".into()));
        }
        let (surface, flip) = match topo.carrier(face) {
            Carrier::Surface(sc) => (
                topo.shapes().surface(sc.surface),
                if sc.sense == Sense::Reversed { -1.0 } else { 1.0 },
            ),
            _ => (None, 1.0),
        };
        // Positions of the boundary nodes (also for the default size).
        let mut pos: Vec<Vec<Point3>> = Vec::new();
        for lp in &loops {
            let mut v = Vec::with_capacity(lp.nodes.len());
            for k in &lp.nodes {
                v.push(cx.point(*k).ok_or(TriError::Lift { key: *k })?);
            }
            pos.push(v);
        }
        let h0 = {
            let (mut sum, mut n) = (0.0, 0usize);
            for v in &pos {
                for i in 0..v.len() {
                    sum += (v[(i + 1) % v.len()] - v[i]).norm();
                    n += 1;
                }
            }
            if n == 0 || !(sum > 0.0) {
                return Err(TriError::Degenerate("boundary has zero length".into()));
            }
            sum / n as f64
        };
        // Surface parameters of the loop nodes.
        let plane = if surface.is_none() {
            Some(newell_frame(cx, face, &pos)?)
        } else {
            None
        };
        let mut uvs: Vec<Vec<[f64; 2]>> = Vec::new();
        for (li, lp) in loops.iter().enumerate() {
            let v = if let Some(s) = surface {
                match cx.loop_uvs(face, lp) {
                    Some(v) => v,
                    None => project_loop(s, &pos[li], &lp.nodes)?,
                }
            } else if let Some(fr) = &plane {
                pos[li]
                    .iter()
                    .map(|p| {
                        let d = *p - fr.o;
                        [dot(d, fr.e1), dot(d, fr.e2)]
                    })
                    .collect()
            } else {
                return Err(TriError::Degenerate("no carrier".into()));
            };
            uvs.push(v);
        }
        // Poles: (loop, node index of the pole node, pole vertex node).
        let mut poles: Vec<Vec<(usize, NodeKey)>> = vec![Vec::new(); loops.len()];
        for (li, lp) in loops.iter().enumerate() {
            let Some(full) = topo.loops(face).find(|l| l.ring == lp.ring) else {
                continue;
            };
            let keys: Vec<_> = full.keys().collect();
            for (p, k) in keys.iter().enumerate() {
                if !topo.flags(k.edge).contains(CellFlags::DEGENERATE) {
                    continue;
                }
                let Some(vtx) = topo.edge_ends(k.edge).map(|e| e.0) else {
                    continue;
                };
                // The outgoing real coedge starts at the pole node.
                let out = (1..keys.len())
                    .map(|d| keys[(p + d) % keys.len()])
                    .find(|k2| !topo.flags(k2.edge).contains(CellFlags::DEGENERATE));
                let Some(out) = out else { continue };
                let Some(u) = lp.uses.iter().find(|u| u.coedge == out) else {
                    continue;
                };
                poles[li].push((u.first as usize, NodeKey::vertex(vtx)));
            }
            poles[li].sort_by_key(|p| p.0);
        }
        let period_u = surface.and_then(|s| s.domain().u.period);

        // Assemble nodes (pole nodes replaced by rings), uv still raw.
        let mut next_index = 0u32;
        let mut chart_loops: Vec<ChartLoop> = Vec::new();
        for (li, lp) in loops.iter().enumerate() {
            let n = lp.nodes.len();
            let uv = &uvs[li];
            let mut nodes: Vec<ChartNode> = Vec::with_capacity(n + 8);
            let mut fans: Vec<PoleFan> = Vec::new();
            for i in 0..n {
                if let Some((_, pole)) = poles[li].iter().find(|p| p.0 == i) {
                    if n < 3 {
                        return Err(TriError::Unsupported("pole loop with fewer than 3 nodes".into()));
                    }
                    let (ia, ib) = ((i + n - 1) % n, (i + 1) % n);
                    let (ka, kb) = (lp.nodes[ia], lp.nodes[ib]);
                    let (ua, ub) = (uv[ia], uv[ib]);
                    let tol = 1e-9 * (1.0 + ua[1].abs());
                    if (ua[1] - ub[1]).abs() > tol || (uv[i][1] - ua[1]).abs() <= tol {
                        return Err(TriError::Unsupported(
                            "pole: neighbours not on one iso-line of the collapsed edge".into(),
                        ));
                    }
                    let period = period_u.unwrap_or(TAU);
                    let span = ub[0] - ua[0];
                    let full = (span.abs() - period).abs() <= 1e-6 * period;
                    if (ka == kb) != full {
                        return Err(TriError::Unsupported(
                            "pole: ring neighbours do not close the revolution".into(),
                        ));
                    }
                    let m = if full {
                        6usize
                    } else {
                        ((6.0 * span.abs() / period).round() as usize).max(2)
                    };
                    let mut path = vec![ka];
                    for k in 1..m {
                        let u = ua[0] + span * k as f64 / m as f64;
                        let key = NodeKey::new(face, next_index);
                        next_index += 1;
                        nodes.push(ChartNode {
                            key,
                            uv: [u, ua[1]],
                            q: [0.0; 2],
                            new: true,
                        });
                        path.push(key);
                    }
                    path.push(kb);
                    fans.push(PoleFan { pole: *pole, path });
                    continue;
                }
                nodes.push(ChartNode {
                    key: lp.nodes[i],
                    uv: uv[i],
                    q: [0.0; 2],
                    new: false,
                });
            }
            // Closure: the implicit closing segment must be a regular step.
            check_closure(lp.ring, &nodes)?;
            chart_loops.push(ChartLoop {
                ring: lp.ring,
                nodes,
                fans,
            });
        }

        // The metric at the loop centroid.
        let outer = &chart_loops[0].nodes;
        let uv_c = {
            let n = outer.len() as f64;
            [
                outer.iter().map(|c| c.uv[0]).sum::<f64>() / n,
                outer.iter().map(|c| c.uv[1]).sum::<f64>() / n,
            ]
        };
        let eval = |uv: [f64; 2]| -> Point3 { eval_surface(surface, plane.as_ref(), uv) };
        let jet = |uv: [f64; 2]| -> (Vector3, Vector3) { jet_surface(surface, plane.as_ref(), uv) };
        let p_c = eval(uv_c);
        let m_c = match cx.size {
            Some(f) => f.metric(p_c).0,
            None => {
                let a = 1.0 / (h0 * h0);
                [a, 0.0, 0.0, a, 0.0, a]
            }
        };
        let gform = |uv: [f64; 2], m: &[f64; 6]| -> [f64; 3] {
            let (du, dv) = jet(uv);
            metric_form(du * flip, dv, m)
        };
        let g = gform(uv_c, &m_c);
        let (l11, l21, l22) = cholesky2(g)?;
        let lt = [l11, l21, l22];
        let inv = [1.0 / l11, -l21 / (l11 * l22), 1.0 / l22];

        // z for every node, bounding box, then the power-of-two scaling.
        let zmap = |uv: [f64; 2]| -> [f64; 2] {
            let w = [flip * (uv[0] - uv_c[0]), uv[1] - uv_c[1]];
            [lt[0] * w[0] + lt[1] * w[1], lt[2] * w[1]]
        };
        let (mut zmin, mut zmax) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for cl in &chart_loops {
            for nd in &cl.nodes {
                let z = zmap(nd.uv);
                if !(z[0].is_finite() && z[1].is_finite()) {
                    return Err(TriError::Lift { key: nd.key });
                }
                for k in 0..2 {
                    zmin[k] = zmin[k].min(z[k]);
                    zmax[k] = zmax[k].max(z[k]);
                }
            }
        }
        let ext = (zmax[0] - zmin[0]).max(zmax[1] - zmin[1]);
        if !(ext > 0.0 && ext.is_finite()) {
            return Err(TriError::Degenerate("face has zero extent in the chart".into()));
        }
        let scale = 1.0 / pow2_ceil(ext);
        let shift = [1.0 - zmin[0] * scale, 1.0 - zmin[1] * scale];
        for cl in &mut chart_loops {
            for nd in &mut cl.nodes {
                let z = zmap(nd.uv);
                nd.q = [z[0] * scale + shift[0], z[1] * scale + shift[1]];
            }
        }

        // Isometry and anisotropy, by sampling.
        let mut samples: Vec<[f64; 2]> = vec![uv_c];
        {
            let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
            for cl in &chart_loops {
                for nd in &cl.nodes {
                    for k in 0..2 {
                        lo[k] = lo[k].min(nd.uv[k]);
                        hi[k] = hi[k].max(nd.uv[k]);
                    }
                }
            }
            for &x in &[lo[0], hi[0]] {
                for &y in &[lo[1], hi[1]] {
                    samples.push([x, y]);
                }
            }
        }
        let unit = [1.0, 0.0, 0.0, 1.0, 0.0, 1.0];
        let g_ref = gform(uv_c, &unit);
        let mut iso_const = true;
        for s in &samples {
            // At a pole the form is singular on purpose; skip samples that collapse it.
            let gs = gform(*s, &unit);
            let nrm = g_ref[0].abs() + g_ref[1].abs() + g_ref[2].abs();
            if (0..3).any(|k| (gs[k] - g_ref[k]).abs() > 1e-9 * nrm) {
                iso_const = false;
            }
        }
        let mut aniso = false;
        let mut m_const = true;
        if let Some(f) = cx.size {
            let mut pts: Vec<Point3> = vec![p_c];
            for cl in &chart_loops {
                for nd in cl.nodes.iter().step_by((cl.nodes.len() / 8).max(1)) {
                    pts.push(eval(nd.uv));
                }
            }
            for p in &pts {
                let m = f.metric(*p).0;
                let tr = (m[0] + m[3] + m[5]) / 3.0;
                let nrm = m[0].abs() + m[3].abs() + m[5].abs() + 2.0 * (m[1].abs() + m[2].abs() + m[4].abs());
                if (m[0] - tr).abs() + (m[3] - tr).abs() + (m[5] - tr).abs()
                    + 2.0 * (m[1].abs() + m[2].abs() + m[4].abs())
                    > 1e-6 * nrm
                {
                    aniso = true;
                }
                if (0..6).any(|k| (m[k] - m_c[k]).abs() > 1e-9 * nrm) {
                    m_const = false;
                }
            }
        }
        let isometric = iso_const && (!aniso || m_const);
        let chart = FaceChart {
            face,
            surface,
            plane,
            flip,
            uv0: uv_c,
            lt,
            inv,
            scale,
            shift,
            periodic_u: period_u,
            isometric,
            aniso,
            size: cx.size,
            h0,
            next_index,
        };
        Ok((chart, chart_loops))
    }

    /// The face this chart belongs to.
    pub fn face(&self) -> EntityId {
        self.face
    }

    /// The default target size (mean boundary segment length) used without a size field.
    pub fn default_size(&self) -> f64 {
        self.h0
    }

    /// First free face-owned node index (after the pole-ring nodes).
    pub fn next_index(&self) -> u32 {
        self.next_index
    }

    /// True if the chart is an isometry of the surface up to a constant scale (plane,
    /// cylinder).
    pub fn is_isometric(&self) -> bool {
        self.isometric
    }

    /// True if the size field is anisotropic (quality is measured in the metric).
    pub fn is_anisotropic(&self) -> bool {
        self.aniso
    }

    /// Chart coordinates of surface parameters.
    pub fn to_q(&self, uv: [f64; 2]) -> [f64; 2] {
        let w = [self.flip * (uv[0] - self.uv0[0]), uv[1] - self.uv0[1]];
        let z = [self.lt[0] * w[0] + self.lt[1] * w[1], self.lt[2] * w[1]];
        [z[0] * self.scale + self.shift[0], z[1] * self.scale + self.shift[1]]
    }

    /// Surface parameters (on the loop branch, not wrapped) of chart point `q`.
    pub fn to_uv(&self, q: [f64; 2]) -> [f64; 2] {
        let z = [(q[0] - self.shift[0]) / self.scale, (q[1] - self.shift[1]) / self.scale];
        let w = [self.inv[0] * z[0] + self.inv[1] * z[1], self.inv[2] * z[1]];
        [self.uv0[0] + self.flip * w[0], self.uv0[1] + w[1]]
    }

    /// Surface parameters wrapped into the principal domain (what `Param::Surface` stores).
    pub fn wrap_uv(&self, uv: [f64; 2]) -> [f64; 2] {
        match self.surface {
            Some(s) => {
                let d = s.domain();
                [d.u.wrap(uv[0]), d.v.wrap(uv[1])]
            }
            None => uv,
        }
    }

    /// The surface point of parameters `uv` (not necessarily in the principal domain).
    pub fn point_at(&self, uv: [f64; 2]) -> Point3 {
        eval_surface(self.surface, self.plane.as_ref(), uv)
    }

    /// The surface point of chart point `q`.
    pub fn lift3(&self, q: [f64; 2]) -> Point3 {
        self.point_at(self.to_uv(q))
    }

    /// `d(point)/d(q)` as three rows `[dq1, dq2]` (x, y, z).
    pub fn jacobian(&self, q: [f64; 2]) -> [[f64; 2]; 3] {
        let uv = self.to_uv(q);
        let (du, dv) = jet_surface(self.surface, self.plane.as_ref(), uv);
        let (a, b) = (du * self.flip, dv);
        // J_q = [a b] * L^T^-1 / scale, L^T^-1 = [[i0, i1], [0, i2]].
        let c1 = a * (self.inv[0] / self.scale);
        let c2 = a * (self.inv[1] / self.scale) + b * (self.inv[2] / self.scale);
        [[c1.x, c2.x], [c1.y, c2.y], [c1.z, c2.z]]
    }

    /// The unit face normal (oriented with the face, carrier sense included) at surface
    /// parameters `uv`; `None` where the tangent plane is singular.
    pub fn normal_at(&self, uv: [f64; 2]) -> Option<Vector3> {
        let (du, dv) = jet_surface(self.surface, self.plane.as_ref(), uv);
        let n = du.cross(dv) * self.flip;
        let l = n.norm();
        (l > 1e-12 * du.norm().max(dv.norm()).powi(2)).then(|| n * (1.0 / l))
    }

    /// The surface parameters of the point `p` near `hint` (periodic parameters unwrapped
    /// against it).
    pub fn uv_near(&self, p: Point3, hint: [f64; 2]) -> Option<[f64; 2]> {
        if let Some(fr) = &self.plane {
            let d = p - fr.o;
            return Some([dot(d, fr.e1), dot(d, fr.e2)]);
        }
        let s = self.surface?;
        let pr = s.project(p, Some(Uv::new(hint[0], hint[1]))).ok()?;
        let d = s.domain();
        Some([d.u.nearest_to(pr.uv.u, hint[0]), d.v.nearest_to(pr.uv.v, hint[1])])
    }

    /// True if the face has a surface carrier (else it is planar by its Newell plane).
    pub fn has_surface(&self) -> bool {
        self.surface.is_some()
    }
}

impl Geometry2 for FaceChart<'_> {
    fn lift(&self, q: [f64; 2]) -> Point3 {
        self.lift3(q)
    }

    fn size(&self, p: Point3) -> f64 {
        match self.size {
            Some(f) => f.size(p),
            None => self.h0,
        }
    }

    fn is_isometric(&self) -> bool {
        self.isometric
    }

    fn chart_metric(&self, q: [f64; 2]) -> Option<[f64; 3]> {
        if self.isometric {
            return None;
        }
        let j = self.jacobian(q);
        let c1 = Vector3::new(j[0][0], j[1][0], j[2][0]);
        let c2 = Vector3::new(j[0][1], j[1][1], j[2][1]);
        if self.aniso {
            let m = self.size?.metric(self.lift3(q)).0;
            Some(metric_form(c1, c2, &m))
        } else {
            Some([dot(c1, c1), dot(c1, c2), dot(c2, c2)])
        }
    }

    fn metric_map(&self, p: Point3) -> Option<[[f64; 3]; 3]> {
        if !self.aniso {
            return None;
        }
        let m = self.size?.metric(p).0;
        // M = C C^T (lower Cholesky); |C^T e|^2 = e^T M e.
        let c00 = m[0].sqrt();
        if !(c00 > 0.0) {
            return None;
        }
        let c10 = m[1] / c00;
        let c20 = m[2] / c00;
        let d11 = m[3] - c10 * c10;
        if !(d11 > 0.0) {
            return None;
        }
        let c11 = d11.sqrt();
        let c21 = (m[4] - c20 * c10) / c11;
        let d22 = m[5] - c20 * c20 - c21 * c21;
        if !(d22 > 0.0) {
            return None;
        }
        let c22 = d22.sqrt();
        Some([[c00, c10, c20], [0.0, c11, c21], [0.0, 0.0, c22]])
    }
}

/// `[g11, g12, g22]` of the displacement `dq1 * a + dq2 * b` in the metric `m` (xx, xy, xz,
/// yy, yz, zz).
fn metric_form(a: Vector3, b: Vector3, m: &[f64; 6]) -> [f64; 3] {
    let mv = |v: Vector3| {
        Vector3::new(
            m[0] * v.x + m[1] * v.y + m[2] * v.z,
            m[1] * v.x + m[3] * v.y + m[4] * v.z,
            m[2] * v.x + m[4] * v.y + m[5] * v.z,
        )
    };
    let (ma, mb) = (mv(a), mv(b));
    [dot(a, ma), dot(a, mb), dot(b, mb)]
}

/// Cholesky of `[[g11, g12], [g12, g22]]` as `(l11, l21, l22)` with `G = L L^T`.
fn cholesky2(g: [f64; 3]) -> Result<(f64, f64, f64), TriError> {
    if !(g[0] > 0.0 && g[0].is_finite()) {
        return Err(TriError::Degenerate("surface metric is singular".into()));
    }
    let l11 = g[0].sqrt();
    let l21 = g[1] / l11;
    let d = g[2] - l21 * l21;
    if !(d > 0.0 && d.is_finite()) {
        return Err(TriError::Degenerate("surface metric is singular".into()));
    }
    Ok((l11, l21, d.sqrt()))
}

fn eval_surface(s: Option<&dyn ParametricSurface>, fr: Option<&PlaneFrame>, uv: [f64; 2]) -> Point3 {
    if let Some(s) = s {
        let d = s.domain();
        s.eval(Uv::new(d.u.wrap(uv[0]), d.v.wrap(uv[1])))
    } else if let Some(fr) = fr {
        fr.o + fr.e1 * uv[0] + fr.e2 * uv[1]
    } else {
        Point3::default()
    }
}

fn jet_surface(s: Option<&dyn ParametricSurface>, fr: Option<&PlaneFrame>, uv: [f64; 2]) -> (Vector3, Vector3) {
    if let Some(s) = s {
        let d = s.domain();
        let j = s.jet(Uv::new(d.u.wrap(uv[0]), d.v.wrap(uv[1])));
        (j.du, j.dv)
    } else if let Some(fr) = fr {
        (fr.e1, fr.e2)
    } else {
        (Vector3::default(), Vector3::default())
    }
}

/// Newell plane of the outline, with `(e1, e2, n)` right-handed.
fn newell_frame(cx: &GenCtx<'_>, face: EntityId, pos: &[Vec<Point3>]) -> Result<PlaneFrame, TriError> {
    let _ = face;
    let _ = cx;
    let pts = &pos[0];
    let mut n = Vector3::default();
    let mut c = Vector3::default();
    for i in 0..pts.len() {
        let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
        n = n + Vector3::new(
            (a.y - b.y) * (a.z + b.z),
            (a.z - b.z) * (a.x + b.x),
            (a.x - b.x) * (a.y + b.y),
        );
        c = c + Vector3::new(a.x, a.y, a.z);
    }
    let l = n.norm();
    if !(l > 0.0) {
        return Err(TriError::Degenerate("face without a surface has no normal".into()));
    }
    let n = n * (1.0 / l);
    let o = Point3::new(c.x / pts.len() as f64, c.y / pts.len() as f64, c.z / pts.len() as f64);
    // e1: the axis least aligned with n, orthogonalized.
    let ax = if n.x.abs() <= n.y.abs() && n.x.abs() <= n.z.abs() {
        Vector3::new(1.0, 0.0, 0.0)
    } else if n.y.abs() <= n.z.abs() {
        Vector3::new(0.0, 1.0, 0.0)
    } else {
        Vector3::new(0.0, 0.0, 1.0)
    };
    let e1 = ax - n * dot(ax, n);
    let e1 = e1 * (1.0 / e1.norm());
    let e2 = n.cross(e1);
    Ok(PlaneFrame { o, e1, e2 })
}

/// Projection fallback: project each node, unwrapping against the previous uv.
fn project_loop(
    s: &dyn ParametricSurface,
    pos: &[Point3],
    nodes: &[NodeKey],
) -> Result<Vec<[f64; 2]>, TriError> {
    let d = s.domain();
    if !d.singularities.is_empty() {
        return Err(TriError::Lift { key: nodes[0] });
    }
    let mut out: Vec<[f64; 2]> = Vec::with_capacity(pos.len());
    for (i, p) in pos.iter().enumerate() {
        let hint = out.last().map(|u| Uv::new(u[0], u[1]));
        let pr = s.project(*p, hint).map_err(|_| TriError::Lift { key: nodes[i] })?;
        let uv = match out.last() {
            Some(prev) => [d.u.nearest_to(pr.uv.u, prev[0]), d.v.nearest_to(pr.uv.v, prev[1])],
            None => [pr.uv.u, pr.uv.v],
        };
        out.push(uv);
    }
    Ok(out)
}

/// The closing step of the loop must not be longer than a few typical steps.
fn check_closure(ring: u16, nodes: &[ChartNode]) -> Result<(), TriError> {
    let n = nodes.len();
    if n < 3 {
        return Err(TriError::Degenerate("loop with fewer than 3 nodes".into()));
    }
    let step = |a: [f64; 2], b: [f64; 2]| ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt();
    let mut max_step = 0.0f64;
    let mut extent = [(f64::INFINITY, f64::NEG_INFINITY); 2];
    for i in 0..n {
        let (a, b) = (nodes[i].uv, nodes[(i + 1) % n].uv);
        if i + 1 < n {
            max_step = max_step.max(step(a, b));
        }
        for k in 0..2 {
            extent[k].0 = extent[k].0.min(a[k]);
            extent[k].1 = extent[k].1.max(a[k]);
        }
    }
    let ext = (extent[0].1 - extent[0].0).max(extent[1].1 - extent[1].0);
    let gap = step(nodes[n - 1].uv, nodes[0].uv);
    if gap > 4.0 * max_step + 1e-9 * ext {
        return Err(TriError::OpenLoop { ring, gap: gap / ext.max(f64::MIN_POSITIVE) });
    }
    Ok(())
}
