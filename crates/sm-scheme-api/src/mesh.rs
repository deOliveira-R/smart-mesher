//! [`CommittedMesh`]: the driver's scratch store of everything meshed so far (vertex nodes, curve
//! node chains, surface patches, volume cells), each node with its origin entity. Schemes read
//! it as their boundary mesh; only the driver commits to it, in a fixed order, and every commit
//! validates the whole patch before changing anything (all-or-nothing).

use crate::caps::Caps;
use crate::intent::SchemeId;
use crate::key::ShapeKey;
use crate::patch::{CellList, ClaimedSurface, FaceList, Grid, NewNode, NodeId, PatchRef, SurfacePatch, TransformPatch, VolumePatch};
use serde::{Deserialize, Serialize};
use sm_core::Point3;
use sm_mesh_api::{CellShape, Param};
use std::collections::{BTreeMap, BTreeSet};

/// Where a node lies.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct NodeOrigin {
    /// The entity.
    pub on: ShapeKey,
    /// Parameters on it.
    pub param: Param,
}

/// The node chain of a meshed curve.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CommittedCurve {
    /// Nodes from the curve's start vertex to its end vertex (`intervals + 1` entries).
    pub nodes: Vec<NodeId>,
    /// Arc-length fractions of `nodes` (`0` first, `1` last).
    pub fractions: Vec<f64>,
    /// The scheme that meshed it.
    pub scheme: SchemeId,
}

/// A meshed face.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CommittedSurface {
    /// Elements, oriented like the face.
    pub faces: FaceList<NodeId>,
    /// Logical grid, if structured.
    pub grid: Option<Grid<NodeId>>,
    /// The scheme that meshed it (the volume scheme for claimed faces).
    pub scheme: SchemeId,
    /// Capabilities of the result.
    pub caps: Caps,
}

/// Cells of one volume job (one volume, or an atomic group).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CommittedCells {
    /// The job's targets, ascending.
    pub targets: Vec<ShapeKey>,
    /// Shape per cell.
    pub shape: Vec<CellShape>,
    /// CSR offsets.
    pub off: Vec<u32>,
    /// Nodes in template order.
    pub nodes: Vec<NodeId>,
    /// Index into `targets` per cell.
    pub target: Vec<u32>,
    /// The scheme.
    pub scheme: SchemeId,
}

impl CommittedCells {
    /// Number of cells.
    pub fn len(&self) -> usize {
        self.shape.len()
    }
    /// True if empty.
    pub fn is_empty(&self) -> bool {
        self.shape.is_empty()
    }
    /// Nodes of cell `i`.
    pub fn cell(&self, i: usize) -> &[NodeId] {
        &self.nodes[self.off[i] as usize..self.off[i + 1] as usize]
    }
}

/// Everything meshed so far.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CommittedMesh {
    points: Vec<Point3>,
    origins: Vec<NodeOrigin>,
    vertices: BTreeMap<ShapeKey, NodeId>,
    curves: BTreeMap<ShapeKey, CommittedCurve>,
    surfaces: BTreeMap<ShapeKey, CommittedSurface>,
    volumes: BTreeMap<ShapeKey, CommittedCells>,
}

fn resolve(r: PatchRef, old: usize, base: usize, new: usize) -> Result<NodeId, String> {
    match r {
        PatchRef::Old(id) if id.get() < old => Ok(id),
        PatchRef::Old(id) => Err(format!("unknown node {id:?}")),
        PatchRef::New(i) if (i as usize) < new => Ok(NodeId((base + i as usize) as u32)),
        PatchRef::New(i) => Err(format!("new node {i} out of range ({new})")),
    }
}

fn map_faces(f: &FaceList<PatchRef>, old: usize, base: usize, new: usize) -> Result<FaceList<NodeId>, String> {
    if f.off.first() != Some(&0) || f.off.windows(2).any(|w| w[1] < w[0]) || f.off.last().map(|&l| l as usize) != Some(f.nodes.len()) {
        return Err("malformed face offsets".into());
    }
    let mut out = FaceList::new();
    let mut buf: Vec<NodeId> = Vec::with_capacity(8);
    for loop_ in f.iter() {
        if loop_.len() < 3 {
            return Err("face with fewer than 3 nodes".into());
        }
        buf.clear();
        for &r in loop_ {
            buf.push(resolve(r, old, base, new)?);
        }
        for i in 0..buf.len() {
            if buf[i + 1..].contains(&buf[i]) {
                return Err("face repeats a node".into());
            }
        }
        out.push(&buf);
    }
    Ok(out)
}

fn map_grid(g: &Option<Grid<PatchRef>>, old: usize, base: usize, new: usize) -> Result<Option<Grid<NodeId>>, String> {
    let Some(g) = g else { return Ok(None) };
    if g.nodes.len() != (g.ni as usize + 1) * (g.nj as usize + 1) {
        return Err("grid size does not match ni, nj".into());
    }
    let nodes = g.nodes.iter().map(|&r| resolve(r, old, base, new)).collect::<Result<Vec<_>, _>>()?;
    Ok(Some(Grid { ni: g.ni, nj: g.nj, nodes }))
}

impl CommittedMesh {
    /// Empty store.
    pub fn new() -> Self {
        Self::default()
    }
    /// Number of nodes.
    pub fn node_count(&self) -> usize {
        self.points.len()
    }
    /// Position of a node (panics on an id not from this store).
    pub fn point(&self, n: NodeId) -> Point3 {
        self.points[n.get()]
    }
    /// Origin of a node.
    pub fn origin(&self, n: NodeId) -> NodeOrigin {
        self.origins[n.get()]
    }
    /// All positions (SoA column).
    pub fn points(&self) -> &[Point3] {
        &self.points
    }
    /// All origins.
    pub fn origins(&self) -> &[NodeOrigin] {
        &self.origins
    }
    /// The node of a topology vertex.
    pub fn vertex(&self, v: ShapeKey) -> Option<NodeId> {
        self.vertices.get(&v).copied()
    }
    /// A meshed curve.
    pub fn curve(&self, c: ShapeKey) -> Option<&CommittedCurve> {
        self.curves.get(&c)
    }
    /// A meshed face.
    pub fn surface(&self, f: ShapeKey) -> Option<&CommittedSurface> {
        self.surfaces.get(&f)
    }
    /// All meshed faces, ascending.
    pub fn surfaces(&self) -> impl Iterator<Item = (&ShapeKey, &CommittedSurface)> {
        self.surfaces.iter()
    }
    /// All volume jobs, ascending by first target.
    pub fn volumes(&self) -> impl Iterator<Item = &CommittedCells> {
        self.volumes.values()
    }
    /// Total number of cells.
    pub fn cell_count(&self) -> usize {
        self.volumes.values().map(CommittedCells::len).sum()
    }

    /// Single-platform fingerprint (raw float bits), for determinism tests.
    pub fn fingerprint(&self) -> u64 {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        let mut w = |v: u64| {
            for b in v.to_le_bytes() {
                h ^= u64::from(b);
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        };
        for (p, o) in self.points.iter().zip(&self.origins) {
            w(p.x.to_bits());
            w(p.y.to_bits());
            w(p.z.to_bits());
            match o.on {
                ShapeKey::Cell(id) => w(id.to_raw()),
                ShapeKey::Piece(sm_model_api::PieceRef::Whole(id) | sm_model_api::PieceRef::Cell(id)) => w(id.to_raw() ^ 1 << 63),
                ShapeKey::PieceFace(pf) => w(pf.base().map_or(u64::MAX, |id| id.to_raw() ^ 1 << 62)),
            }
            match o.param {
                Param::None => w(0),
                Param::Curve(t) => w(t.to_bits()),
                Param::Surface([u, v]) => w(u.to_bits() ^ v.to_bits().rotate_left(17)),
                Param::Volume([u, v, x]) => w(u.to_bits() ^ v.to_bits().rotate_left(17) ^ x.to_bits().rotate_left(34)),
            }
        }
        for s in self.surfaces.values() {
            s.faces.nodes.iter().for_each(|n| w(u64::from(n.0)));
            s.faces.off.iter().for_each(|&n| w(u64::from(n)));
        }
        for c in self.volumes.values() {
            c.nodes.iter().for_each(|n| w(u64::from(n.0)));
        }
        h
    }

    fn push_node(&mut self, p: Point3, origin: NodeOrigin) -> NodeId {
        self.points.push(p);
        self.origins.push(origin);
        NodeId((self.points.len() - 1) as u32)
    }

    pub(crate) fn add_vertex(&mut self, v: ShapeKey, p: Point3) -> NodeId {
        if let Some(n) = self.vertices.get(&v) {
            return *n;
        }
        let n = self.push_node(p, NodeOrigin { on: v, param: Param::None });
        self.vertices.insert(v, n);
        n
    }

    pub(crate) fn commit_curve(&mut self, c: ShapeKey, ends: (NodeId, NodeId), interior: &[(f64, Point3, f64)], scheme: &SchemeId) -> Result<(), String> {
        if self.curves.contains_key(&c) {
            return Err(format!("{c} already meshed"));
        }
        let mut nodes = Vec::with_capacity(interior.len() + 2);
        let mut fractions = Vec::with_capacity(interior.len() + 2);
        nodes.push(ends.0);
        fractions.push(0.0);
        for &(f, p, t) in interior {
            nodes.push(self.push_node(p, NodeOrigin { on: c, param: Param::Curve(t) }));
            fractions.push(f);
        }
        nodes.push(ends.1);
        fractions.push(1.0);
        self.curves.insert(c, CommittedCurve { nodes, fractions, scheme: scheme.clone() });
        Ok(())
    }

    fn check_new_nodes(nodes: &[NewNode], allowed: &BTreeSet<ShapeKey>) -> Result<(), String> {
        for n in nodes {
            if !allowed.contains(&n.on) {
                return Err(format!("new node on {} which the plan does not own", n.on));
            }
            if !(n.point.x.is_finite() && n.point.y.is_finite() && n.point.z.is_finite()) {
                return Err("non-finite node".into());
            }
        }
        Ok(())
    }

    pub(crate) fn commit_surface(&mut self, f: ShapeKey, patch: &SurfacePatch, scheme: &SchemeId, caps: Caps) -> Result<(), String> {
        if self.surfaces.contains_key(&f) {
            return Err(format!("{f} already meshed"));
        }
        Self::check_new_nodes(&patch.nodes, &BTreeSet::from([f]))?;
        let (old, base, new) = (self.node_count(), self.node_count(), patch.nodes.len());
        let faces = map_faces(&patch.faces, old, base, new)?;
        let grid = map_grid(&patch.grid, old, base, new)?;
        if caps.contains(Caps::STRUCTURED) && grid.is_none() {
            return Err("a STRUCTURED scheme must return a grid".into());
        }
        for n in &patch.nodes {
            self.push_node(n.point, NodeOrigin { on: n.on, param: n.param });
        }
        self.surfaces.insert(f, CommittedSurface { faces, grid, scheme: scheme.clone(), caps });
        Ok(())
    }

    fn map_cells(cells: &CellList, ntargets: usize, old: usize, base: usize, new: usize) -> Result<(Vec<u32>, Vec<NodeId>), String> {
        if cells.off.len() != cells.shape.len() + 1 || cells.target.len() != cells.shape.len() || cells.off.first() != Some(&0) || cells.off.last().map(|&l| l as usize) != Some(cells.nodes.len()) {
            return Err("malformed cell list".into());
        }
        let mut nodes = Vec::with_capacity(cells.nodes.len());
        for i in 0..cells.len() {
            let c = cells.cell(i);
            if cells.shape[i].vertex_count() != Some(c.len()) {
                return Err(format!("cell {i}: {:?} with {} nodes (polyhedra are not supported in patches yet)", cells.shape[i], c.len()));
            }
            if cells.target[i] as usize >= ntargets {
                return Err(format!("cell {i}: target index out of range"));
            }
            for &r in c {
                nodes.push(resolve(r, old, base, new)?);
            }
        }
        Ok((cells.off.clone(), nodes))
    }

    pub(crate) fn commit_volume(&mut self, targets: &[ShapeKey], claims: &[ShapeKey], patch: &VolumePatch, scheme: &SchemeId, claimed_caps: Caps) -> Result<(), String> {
        let key = *targets.first().ok_or("plan without targets")?;
        if targets.iter().any(|t| self.volumes.values().any(|c| c.targets.contains(t))) {
            return Err("a target is already meshed".into());
        }
        let allowed: BTreeSet<ShapeKey> = targets.iter().chain(claims).copied().collect();
        Self::check_new_nodes(&patch.nodes, &allowed)?;
        let (old, base, new) = (self.node_count(), self.node_count(), patch.nodes.len());
        let (off, nodes) = Self::map_cells(&patch.cells, targets.len(), old, base, new)?;
        let mut surfs = Vec::with_capacity(patch.surfaces.len());
        for s in &patch.surfaces {
            if !claims.contains(&s.face) || self.surfaces.contains_key(&s.face) || surfs.iter().any(|(f, _, _)| *f == s.face) {
                return Err(format!("claimed surface {} is not claimed or already meshed", s.face));
            }
            surfs.push((s.face, map_faces(&s.faces, old, base, new)?, map_grid(&s.grid, old, base, new)?));
        }
        for n in &patch.nodes {
            self.push_node(n.point, NodeOrigin { on: n.on, param: n.param });
        }
        for (f, faces, grid) in surfs {
            self.surfaces.insert(f, CommittedSurface { faces, grid, scheme: scheme.clone(), caps: claimed_caps });
        }
        self.volumes.insert(key, CommittedCells { targets: targets.to_vec(), shape: patch.cells.shape.clone(), off, nodes, target: patch.cells.target.clone(), scheme: scheme.clone() });
        Ok(())
    }

    /// Apply a transform patch atomically: every target's cells and every listed surface are
    /// replaced, or nothing changes. Each existing volume job must lie wholly inside `targets`.
    pub fn apply_transform(&mut self, targets: &[ShapeKey], patch: &TransformPatch, scheme: &SchemeId, caps: Caps) -> Result<(), String> {
        let key = *targets.first().ok_or("transform without targets")?;
        let jobs: Vec<ShapeKey> = self.volumes.iter().filter(|(_, c)| c.targets.iter().any(|t| targets.contains(t))).map(|(k, _)| *k).collect();
        for j in &jobs {
            if self.volumes[j].targets.iter().any(|t| !targets.contains(t)) {
                return Err(format!("volume job {j} is only partly covered by the transform"));
            }
        }
        let allowed: BTreeSet<ShapeKey> = targets.iter().chain(patch.replace_surfaces.iter().map(|s| &s.face)).copied().collect();
        Self::check_new_nodes(&patch.volume.nodes, &allowed)?;
        let (old, base, new) = (self.node_count(), self.node_count(), patch.volume.nodes.len());
        let (off, nodes) = Self::map_cells(&patch.volume.cells, targets.len(), old, base, new)?;
        let mut surfs = Vec::new();
        for s in &patch.replace_surfaces {
            if !self.surfaces.contains_key(&s.face) {
                return Err(format!("{} has no mesh to replace", s.face));
            }
            surfs.push((s.face, map_faces(&s.faces, old, base, new)?, map_grid(&s.grid, old, base, new)?));
        }
        for n in &patch.volume.nodes {
            self.push_node(n.point, NodeOrigin { on: n.on, param: n.param });
        }
        for j in jobs {
            self.volumes.remove(&j);
        }
        for (f, faces, grid) in surfs {
            self.surfaces.insert(f, CommittedSurface { faces, grid, scheme: scheme.clone(), caps });
        }
        self.volumes.insert(key, CommittedCells { targets: targets.to_vec(), shape: patch.volume.cells.shape.clone(), off, nodes, target: patch.volume.cells.target.clone(), scheme: scheme.clone() });
        Ok(())
    }
}

impl From<&ClaimedSurface> for SurfacePatch {
    fn from(s: &ClaimedSurface) -> Self {
        SurfacePatch { nodes: Vec::new(), faces: s.faces.clone(), grid: s.grid.clone() }
    }
}
