//! Preparation: template-cell CSR, in-entity adjacency, node selection, pins and motion modes.

use std::collections::{BTreeMap, BTreeSet};

use sm_core::{EntityId, EntityKind};
use sm_geom_api::{Domain1, Interval, ParametricCurve, ParametricSurface};
use sm_mesh_api::{
    CellIx, CellShape, Classification, FaceIx, MeshDim, MeshEntityKind, MeshRead, Param, VertexIx,
};
use sm_model_api::{BoundaryKind, Deck};
use sm_quality::Metric;
use sm_topo_api::{Carrier, TopologyView};

use crate::types::{Method, NodeSelect, Objective, SmoothError, SmoothInput, SmoothRequest};

/// A point.
pub(crate) type V = [f64; 3];

/// How a node may move.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Mode {
    /// Never moves.
    Pinned,
    /// Free in R^3 (volume node).
    Free,
    /// On a curve.
    Curve,
    /// On a surface.
    Surf,
}

/// The analytic carrier of an edge or face node.
pub(crate) enum Geo<'a> {
    Curve { curve: &'a dyn ParametricCurve, range: Interval, domain: Domain1 },
    Surf { surf: &'a dyn ParametricSurface, face: EntityId },
}

/// Everything the sweeps need, built once.
pub(crate) struct Prep<'a> {
    pub mesh: &'a dyn MeshRead,
    pub topo: &'a dyn TopologyView,
    pub n: usize,
    pub dim2: bool,
    /// Cell shape (`Polyhedron` cells have an empty corner range).
    pub shape: Vec<CellShape>,
    pub corner_off: Vec<u32>,
    pub corners: Vec<u32>,
    /// Classification entity of each cell (reference normal lookup of 2-D cells).
    pub cell_ent: Vec<EntityId>,
    pub v2c_off: Vec<u32>,
    pub v2c: Vec<u32>,
    /// In-entity neighbours, sorted and distinct.
    pub adj_off: Vec<u32>,
    pub adj: Vec<u32>,
    /// Initial mean in-entity edge length of each node (0 if isolated).
    pub ell: Vec<f64>,
    pub mode: Vec<Mode>,
    pub geo_ix: Vec<u32>,
    pub geos: Vec<Geo<'a>>,
    /// Movable nodes, ascending.
    pub movable: Vec<u32>,
    /// Stored parameter hint (`NaN` = none): `[t, _]` on curves, `[u, v]` on surfaces.
    pub hint0: Vec<[f64; 2]>,
    /// Classification of every vertex.
    pub vcls: Vec<Classification>,
    /// In-entity faces of each face node (3-D `CentroidArea` only).
    pub vf_off: Vec<u32>,
    pub vf: Vec<u32>,
    /// Winslow neighbours `[E, N, W, S, NE, NW, SW, SE]` of every eligible node (Winslow only).
    pub wnb: Vec<[u32; 8]>,
    pub pinned: BTreeMap<String, usize>,
    pub pinned_entities: BTreeMap<String, Vec<EntityId>>,
}

impl<'a> Prep<'a> {
    pub fn cell_corners(&self, c: usize) -> &[u32] {
        &self.corners[self.corner_off[c] as usize..self.corner_off[c + 1] as usize]
    }
    pub fn cells_of(&self, v: usize) -> &[u32] {
        &self.v2c[self.v2c_off[v] as usize..self.v2c_off[v + 1] as usize]
    }
    pub fn nbrs(&self, v: usize) -> &[u32] {
        &self.adj[self.adj_off[v] as usize..self.adj_off[v + 1] as usize]
    }
    pub fn faces_of(&self, v: usize) -> &[u32] {
        if self.vf_off.is_empty() {
            return &[];
        }
        &self.vf[self.vf_off[v] as usize..self.vf_off[v + 1] as usize]
    }
}

fn is_top_or_below(k: EntityKind) -> bool {
    matches!(k, EntityKind::TopoVolume | EntityKind::TopoFace | EntityKind::TopoEdge)
}

/// `e` and every cell below it (sorted, deduplicated).
fn closure(topo: &dyn TopologyView, e: EntityId) -> Vec<EntityId> {
    let mut out = BTreeSet::new();
    let mut stack = vec![e];
    while let Some(x) = stack.pop() {
        if out.insert(x) {
            stack.extend(topo.boundary(x).iter().map(|i| i.cell));
        }
    }
    out.into_iter().collect()
}

/// Every cell of kind `top` above `e`, meshed here or not.
fn top_ancestors(topo: &dyn TopologyView, e: EntityId, top: EntityKind) -> Vec<EntityId> {
    let mut out = BTreeSet::new();
    let mut seen = BTreeSet::new();
    let mut stack = vec![e];
    while let Some(x) = stack.pop() {
        for inc in topo.coboundary(x) {
            let c = inc.cell;
            if !seen.insert(c) {
                continue;
            }
            if c.kind() == top {
                out.insert(c);
            } else if matches!(c.kind(), EntityKind::TopoFace | EntityKind::TopoEdge) {
                stack.push(c);
            }
        }
    }
    out.into_iter().collect()
}

pub(crate) fn validate_request(req: &SmoothRequest) -> Result<(), SmoothError> {
    let bad = |s: String| Err(SmoothError::BadParams(s));
    if req.iterations == 0 {
        return bad("iterations must be at least 1".into());
    }
    if !(req.tolerance.is_finite() && req.tolerance >= 0.0) {
        return bad(format!("tolerance {} must be finite and >= 0", req.tolerance));
    }
    if !(req.guard_floor.is_finite() && (0.0..1.0).contains(&req.guard_floor)) {
        return bad(format!("guard_floor {} must be in [0, 1)", req.guard_floor));
    }
    match req.method {
        Method::Laplacian { relax }
        | Method::SmartLaplacian { relax }
        | Method::CentroidArea { relax }
        | Method::Winslow { relax } => {
            if !(relax.is_finite() && relax > 0.0 && relax < 2.0) {
                return bad(format!("relax {relax} must be in (0, 2)"));
            }
        }
        Method::Optimize { p, .. } => {
            if !(p.is_finite() && (1.0..=32.0).contains(&p)) {
                return bad(format!("p {p} must be in [1, 32]"));
            }
        }
    }
    Ok(())
}

/// The metric a method needs defined on every incident cell (besides the guard's).
fn needs_metric(m: &Method) -> Option<Metric> {
    match m {
        Method::SmartLaplacian { .. } | Method::Optimize { objective: Objective::MeanRatio, .. } => Some(Metric::Shape),
        Method::Optimize { objective: Objective::Condition, .. } => Some(Metric::Condition),
        _ => None,
    }
}

/// The eight Winslow neighbours `[E, N, W, S, NE, NW, SW, SE]` of a node with exactly four
/// incident quads whose wedges close into a cycle, in the orientation of the cells.
fn winslow_stencil(shape: &[CellShape], corner_off: &[u32], corners: &[u32], cells: &[u32], v: u32) -> Option<[u32; 8]> {
    if cells.len() != 4 {
        return None;
    }
    // (next, prev, opposite) of each quad about v.
    let mut w: [(u32, u32, u32); 4] = [(0, 0, 0); 4];
    for (k, &c) in cells.iter().enumerate() {
        let c = c as usize;
        if shape[c] != CellShape::Quad {
            return None;
        }
        let q = &corners[corner_off[c] as usize..corner_off[c + 1] as usize];
        let i = q.iter().position(|x| *x == v)?;
        w[k] = (q[(i + 1) % 4], q[(i + 3) % 4], q[(i + 2) % 4]);
    }
    let mut order = [0usize; 4];
    let mut used = [false; 4];
    used[0] = true;
    for k in 1..4 {
        let want = w[order[k - 1]].1;
        let j = (0..4).find(|&j| !used[j] && w[j].0 == want)?;
        used[j] = true;
        order[k] = j;
    }
    if w[order[3]].1 != w[order[0]].0 {
        return None;
    }
    let [a, b, c, d] = order.map(|i| w[i]);
    Some([a.0, b.0, c.0, d.0, a.2, b.2, c.2, d.2])
}

impl<'a> Prep<'a> {
    pub fn build(input: &SmoothInput<'a>, req: &SmoothRequest) -> Result<Self, SmoothError> {
        validate_request(req)?;
        let mesh = input.mesh;
        let topo = input.topo;
        let n = mesh.vertex_count();
        let nc = mesh.cell_count();
        let dim2 = mesh.dim() == MeshDim::Two;

        if mesh.coords().len() != n {
            return Err(SmoothError::Unsupported("high-order mesh (extra nodes)"));
        }
        let xyz = mesh.coords();
        if let Some(v) = (0..n).find(|&i| !(xyz.x[i].is_finite() && xyz.y[i].is_finite() && xyz.z[i].is_finite())) {
            return Err(SmoothError::NonFiniteInput(v));
        }
        // ---- cells ------------------------------------------------------------------------
        let mut shape = Vec::with_capacity(nc);
        let mut corner_off = Vec::with_capacity(nc + 1);
        let mut corners: Vec<u32> = Vec::with_capacity(nc * 8);
        let mut cell_ent = Vec::with_capacity(nc);
        let mut poly = vec![false; n];
        let cols = mesh.columns();
        corner_off.push(0u32);
        for c in 0..nc {
            let ci = CellIx::new(c);
            if mesh.high_order_cell(ci).is_some() {
                return Err(SmoothError::Unsupported("high-order mesh (high_order_cell)"));
            }
            if !mesh.cell_active(ci) {
                return Err(SmoothError::Unsupported("refined mesh (inactive cells)"));
            }
            let s = mesh.cell_shape(ci);
            match (s, mesh.cell_corners(ci)) {
                (CellShape::Polyhedron, _) | (_, None) => {
                    shape.push(CellShape::Polyhedron);
                    for sf in mesh.cell_faces(ci) {
                        for v in mesh.face_vertices(sf.face()) {
                            poly[v.get()] = true;
                        }
                    }
                }
                (s, Some(cs)) => {
                    shape.push(s);
                    corners.extend(cs.iter().map(|v| v.0));
                }
            }
            corner_off.push(corners.len() as u32);
            cell_ent.push(mesh.cell_class(ci).entity);
        }
        for f in 0..mesh.face_count() {
            if mesh.face_curve(FaceIx::new(f)).is_some() {
                return Err(SmoothError::Unsupported("high-order mesh (face_curve)"));
            }
        }
        let _ = cols;
        // ---- vertex -> cells ----------------------------------------------------------------
        let mut v2c_off = vec![0u32; n + 1];
        for &v in &corners {
            v2c_off[v as usize + 1] += 1;
        }
        for i in 0..n {
            v2c_off[i + 1] += v2c_off[i];
        }
        let mut v2c = vec![0u32; corners.len()];
        {
            let mut fill = v2c_off.clone();
            for c in 0..nc {
                for &v in &corners[corner_off[c] as usize..corner_off[c + 1] as usize] {
                    v2c[fill[v as usize] as usize] = c as u32;
                    fill[v as usize] += 1;
                }
            }
        }
        // ---- classification tables ------------------------------------------------------------
        let ents = mesh.classified_entities();
        let vcls: Vec<Classification> = (0..n).map(|i| mesh.vertex_class(VertexIx::new(i))).collect();
        let vei: Vec<u32> = vcls
            .iter()
            .map(|c| ents.binary_search(&c.entity).map_or(u32::MAX, |i| i as u32))
            .collect();
        let nf = mesh.face_count();
        let fei: Vec<u32> = (0..nf)
            .map(|f| {
                let e = mesh.face_class(FaceIx::new(f)).entity;
                ents.binary_search(&e).map_or(u32::MAX, |i| i as u32)
            })
            .collect();
        // Per entity: closure indices and (edges) the faces above, as sorted index lists.
        let mut closure_ix: Vec<Vec<u32>> = vec![Vec::new(); ents.len()];
        let mut cob_ix: Vec<Vec<u32>> = vec![Vec::new(); ents.len()];
        for (i, e) in ents.iter().enumerate() {
            match e.kind() {
                EntityKind::TopoFace | EntityKind::TopoEdge => {
                    let mut cl: Vec<u32> =
                        closure(topo, *e).iter().filter_map(|x| ents.binary_search(x).ok().map(|j| j as u32)).collect();
                    cl.sort_unstable();
                    closure_ix[i] = cl;
                    if e.kind() == EntityKind::TopoEdge {
                        let mut cb: Vec<u32> = topo
                            .coboundary(*e)
                            .iter()
                            .filter_map(|inc| ents.binary_search(&inc.cell).ok().map(|j| j as u32))
                            .collect();
                        cb.sort_unstable();
                        cob_ix[i] = cb;
                    }
                }
                _ => {}
            }
        }
        // ---- in-entity adjacency from the face loops -----------------------------------------
        let co = mesh.columns();
        let accept = |v: usize, w: usize, x: u32| -> bool {
            let ve = vei[v];
            if ve == u32::MAX {
                return false;
            }
            match ents[ve as usize].kind() {
                EntityKind::TopoVolume => true,
                EntityKind::TopoFace => {
                    x == ve && vei[w] != u32::MAX && closure_ix[ve as usize].binary_search(&vei[w]).is_ok()
                }
                EntityKind::TopoEdge => {
                    (x == ve || cob_ix[ve as usize].binary_search(&x).is_ok())
                        && vei[w] != u32::MAX
                        && closure_ix[ve as usize].binary_search(&vei[w]).is_ok()
                }
                _ => false,
            }
        };
        let for_each_pair = |emit: &mut dyn FnMut(usize, usize, u32)| {
            for (f, &x) in fei.iter().enumerate() {
                let lo = co.face_vertex_offsets[f] as usize;
                let hi = co.face_vertex_offsets[f + 1] as usize;
                let lp = &co.face_vertices[lo..hi];
                let m = lp.len();
                let pairs = if m == 2 { 1 } else { m };
                for i in 0..pairs {
                    emit(lp[i].get(), lp[(i + 1) % m].get(), x);
                }
            }
        };
        let mut cnt = vec![0u32; n + 1];
        for_each_pair(&mut |a, b, x| {
            if accept(a, b, x) {
                cnt[a + 1] += 1;
            }
            if accept(b, a, x) {
                cnt[b + 1] += 1;
            }
        });
        for i in 0..n {
            cnt[i + 1] += cnt[i];
        }
        let mut raw = vec![0u32; cnt[n] as usize];
        {
            let mut fill = cnt.clone();
            for_each_pair(&mut |a, b, x| {
                if accept(a, b, x) {
                    raw[fill[a] as usize] = b as u32;
                    fill[a] += 1;
                }
                if accept(b, a, x) {
                    raw[fill[b] as usize] = a as u32;
                    fill[b] += 1;
                }
            });
        }
        let mut adj_off = Vec::with_capacity(n + 1);
        let mut adj: Vec<u32> = Vec::with_capacity(raw.len() / 2 + 1);
        adj_off.push(0u32);
        for v in 0..n {
            let s = &mut raw[cnt[v] as usize..cnt[v + 1] as usize];
            s.sort_unstable();
            let mut last = u32::MAX;
            for &w in s.iter() {
                if w != last {
                    adj.push(w);
                    last = w;
                }
            }
            adj_off.push(adj.len() as u32);
        }
        drop(raw);
        let co_xyz = mesh.coords();
        let ell: Vec<f64> = (0..n)
            .map(|v| {
                let s = &adj[adj_off[v] as usize..adj_off[v + 1] as usize];
                if s.is_empty() {
                    return 0.0;
                }
                let p = co_xyz.get(v);
                let mut t = 0.0;
                for &w in s {
                    let q = co_xyz.get(w as usize);
                    t += ((p.x - q.x).powi(2) + (p.y - q.y).powi(2) + (p.z - q.z).powi(2)).sqrt();
                }
                t / s.len() as f64
            })
            .collect();

        // ---- pins from the model --------------------------------------------------------------
        let mut pin_ent: BTreeMap<EntityId, &'static str> = BTreeMap::new();
        let mut pin_faces: BTreeMap<&'static str, BTreeSet<EntityId>> = BTreeMap::new();
        for pass in ["deck", "interface"] {
            for row in input.model.boundaries() {
                let hit = match (&row.kind, pass) {
                    (BoundaryKind::Deck(Deck::Mirror { .. }), _) => false,
                    (BoundaryKind::Deck(_), "deck") => true,
                    (BoundaryKind::Interface(_), "interface") => true,
                    _ => false,
                };
                if !hit {
                    continue;
                }
                for (f, _) in input.model.boundary_faces(row.id) {
                    pin_faces.entry(pass).or_default().insert(f);
                    for e in closure(topo, f) {
                        if matches!(e.kind(), EntityKind::TopoFace | EntityKind::TopoEdge) {
                            pin_ent.entry(e).or_insert(pass);
                        }
                    }
                }
            }
        }

        // ---- selection ----------------------------------------------------------------------------
        let top_kind = if dim2 { EntityKind::TopoFace } else { EntityKind::TopoVolume };
        let mut selected = vec![false; n];
        match &req.nodes {
            NodeSelect::All => {
                // A face or edge node of a 3-D mesh moves only if every volume above it is meshed
                // here: a volume in another mesh entry would otherwise lose conformity.
                let mut conformal: BTreeMap<EntityId, bool> = BTreeMap::new();
                for v in 0..n {
                    let e = vcls[v].entity;
                    // In a surface mesh the face nodes are the interior nodes.
                    selected[v] = match e.kind() {
                        EntityKind::TopoFace if dim2 => true,
                        EntityKind::TopoEdge | EntityKind::TopoFace if req.fix_boundary => false,
                        EntityKind::TopoEdge | EntityKind::TopoFace if dim2 => true,
                        EntityKind::TopoEdge | EntityKind::TopoFace => *conformal
                            .entry(e)
                            .or_insert_with(|| top_ancestors(topo, e, top_kind).iter().all(|a| ents.binary_search(a).is_ok())),
                        _ => true,
                    };
                }
            }
            NodeSelect::Vertices(list) => {
                for v in list {
                    if v.get() >= n {
                        return Err(SmoothError::BadParams(format!("vertex {} is out of range ({n})", v.get())));
                    }
                    selected[v.get()] = true;
                }
            }
            NodeSelect::Entities(list) => {
                let mut tops: Vec<EntityId> = Vec::new();
                for &e in list {
                    if !is_top_or_below(e.kind()) {
                        return Err(SmoothError::BadParams(format!("{e} is not a TopoVolume, TopoFace or TopoEdge")));
                    }
                    if !topo.is_alive(e) {
                        return Err(SmoothError::UnknownEntity(e));
                    }
                    let Some(set) = mesh.classified_on(e) else {
                        return Err(SmoothError::UnknownEntity(e));
                    };
                    for v in &set.vertices {
                        selected[v.get()] = true;
                    }
                    if e.kind() == top_kind {
                        tops.push(e);
                    }
                }
                tops.sort_unstable();
                tops.dedup();
                if !req.fix_boundary && !tops.is_empty() {
                    for &e in ents {
                        let below = matches!(e.kind(), EntityKind::TopoEdge)
                            || (!dim2 && e.kind() == EntityKind::TopoFace);
                        if !below {
                            continue;
                        }
                        // 2-D: only faces meshed here count. 3-D: every volume above must be selected.
                        let mut anc = top_ancestors(topo, e, top_kind);
                        if dim2 {
                            anc.retain(|a| ents.binary_search(a).is_ok());
                        }
                        if !anc.is_empty()
                            && anc.iter().all(|a| tops.binary_search(a).is_ok())
                            && let Some(set) = mesh.classified_on(e)
                        {
                            for v in &set.vertices {
                                selected[v.get()] = true;
                            }
                        }
                    }
                }
            }
        }

        // ---- constraints and pins -------------------------------------------------------------------
        let mut geos: Vec<Geo<'a>> = Vec::new();
        let mut geo_of: BTreeMap<EntityId, Option<u32>> = BTreeMap::new();
        let mut mode = vec![Mode::Pinned; n];
        let mut geo_ix = vec![u32::MAX; n];
        let mut hint0 = vec![[f64::NAN; 2]; n];
        let mut movable = Vec::new();
        let mut pinned: BTreeMap<String, usize> = BTreeMap::new();
        let mut pinned_sets: BTreeMap<String, BTreeSet<EntityId>> = BTreeMap::new();
        let need_metric = needs_metric(&req.method);
        let winslow = matches!(req.method, Method::Winslow { .. });
        if winslow && !dim2 {
            return Err(SmoothError::Unsupported("winslow smoothing is 2-D (quad surface meshes) only"));
        }
        let mut wnb: Vec<[u32; 8]> = if winslow { vec![[0; 8]; n] } else { Vec::new() };
        for v in 0..n {
            if !selected[v] {
                continue;
            }
            let e = vcls[v].entity;
            let kind = e.kind();
            let adjn = adj_off[v + 1] - adj_off[v];
            let mut reason: Option<&'static str> = None;
            if kind == EntityKind::TopoVertex {
                reason = Some("vertex");
            } else if !is_top_or_below(kind) {
                reason = Some("unprojectable");
            } else if poly[v] {
                reason = Some("polyhedral");
            } else if let Some(r) = pin_ent.get(&e) {
                reason = Some(r);
            }
            let mut m = Mode::Free;
            if reason.is_none() && matches!(kind, EntityKind::TopoEdge | EntityKind::TopoFace) {
                let g = *geo_of.entry(e).or_insert_with(|| match topo.carrier(e) {
                    Carrier::Curve(cc) if kind == EntityKind::TopoEdge => {
                        topo.shapes().curve(cc.curve).map(|c| {
                            geos.push(Geo::Curve { curve: c, range: cc.range, domain: c.domain() });
                            (geos.len() - 1) as u32
                        })
                    }
                    Carrier::Surface(sc) if kind == EntityKind::TopoFace => {
                        topo.shapes().surface(sc.surface).map(|s| {
                            geos.push(Geo::Surf { surf: s, face: e });
                            (geos.len() - 1) as u32
                        })
                    }
                    _ => None,
                });
                match g {
                    Some(gi) => {
                        geo_ix[v] = gi;
                        m = if kind == EntityKind::TopoEdge { Mode::Curve } else { Mode::Surf };
                        hint0[v] = match vcls[v].param {
                            Param::Curve(t) if m == Mode::Curve && t.is_finite() => [t, 0.0],
                            Param::Surface([u, w]) if m == Mode::Surf && u.is_finite() && w.is_finite() => [u, w],
                            _ => [f64::NAN; 2],
                        };
                    }
                    None => reason = Some("unprojectable"),
                }
            }
            if reason.is_none() && adjn == 0 {
                reason = Some("isolated");
            }
            if reason.is_none()
                && let Some(metric) = need_metric
                && v2c[v2c_off[v] as usize..v2c_off[v + 1] as usize].iter().any(|&c| !metric.applies(shape[c as usize]))
            {
                reason = Some("unsupported_shape");
            }
            if reason.is_none() && winslow {
                match winslow_stencil(&shape, &corner_off, &corners, &v2c[v2c_off[v] as usize..v2c_off[v + 1] as usize], v as u32) {
                    Some(st) if m == Mode::Surf => wnb[v] = st,
                    _ => reason = Some("winslow_unstructured"),
                }
            }
            if let Some(r) = reason {
                *pinned.entry(r.to_owned()).or_default() += 1;
                let set = pinned_sets.entry(r.to_owned()).or_default();
                if matches!(r, "deck" | "interface") {
                    let faces = pin_faces.get(r);
                    if kind == EntityKind::TopoFace {
                        set.insert(e);
                    } else {
                        for inc in topo.coboundary(e) {
                            if faces.is_some_and(|s| s.contains(&inc.cell)) {
                                set.insert(inc.cell);
                            }
                        }
                    }
                } else {
                    set.insert(e);
                }
            } else {
                mode[v] = m;
                movable.push(v as u32);
            }
        }
        let pinned_entities = pinned_sets
            .into_iter()
            .map(|(k, s)| (k, s.into_iter().take(20).collect::<Vec<_>>()))
            .collect();

        // ---- in-entity faces of face nodes (3-D CentroidArea) ----------------------------------------
        let (mut vf_off, mut vf) = (Vec::new(), Vec::new());
        if !dim2 && matches!(req.method, Method::CentroidArea { .. }) {
            let mut c2 = vec![0u32; n + 1];
            let hit = |v: usize, f: usize| mode[v] == Mode::Surf && vei[v] == fei[f];
            for f in 0..nf {
                for v in &co.face_vertices[co.face_vertex_offsets[f] as usize..co.face_vertex_offsets[f + 1] as usize] {
                    if hit(v.get(), f) {
                        c2[v.get() + 1] += 1;
                    }
                }
            }
            for i in 0..n {
                c2[i + 1] += c2[i];
            }
            vf = vec![0u32; c2[n] as usize];
            let mut fill = c2.clone();
            for f in 0..nf {
                for v in &co.face_vertices[co.face_vertex_offsets[f] as usize..co.face_vertex_offsets[f + 1] as usize] {
                    if hit(v.get(), f) {
                        vf[fill[v.get()] as usize] = f as u32;
                        fill[v.get()] += 1;
                    }
                }
            }
            vf_off = c2;
        }
        let _ = MeshEntityKind::Vertex;
        Ok(Prep {
            mesh,
            topo,
            n,
            dim2,
            shape,
            corner_off,
            corners,
            cell_ent,
            v2c_off,
            v2c,
            adj_off,
            adj,
            ell,
            mode,
            geo_ix,
            geos,
            movable,
            hint0,
            vcls,
            vf_off,
            vf,
            wnb,
            pinned,
            pinned_entities,
        })
    }
}
