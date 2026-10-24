//! Patches: the pure output of one scheme invocation, before commit.
//!
//! A [`Patch`] has a **local node table**: element entries are `u32` indices, where
//! `0..nodes.len()` name the nodes the patch creates and the remaining indices name entries
//! of `refs`, the canonical [`NodeKey`]s of nodes created by earlier jobs (boundary vertex and
//! edge nodes, source-face nodes). Commit resolves each reference once per patch instead of
//! once per element corner.

use crate::target::{NodeKey, Target};
use sm_core::{EntityId, Point3};
use sm_mesh_api::{CellShape, Param};
use std::collections::{BTreeMap, HashMap};

/// Shape of one patch element.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ElemShape {
    /// Two nodes (curve patches).
    Segment,
    /// Triangle.
    Tri,
    /// Quadrilateral.
    Quad,
    /// Polygon with three or more nodes.
    Polygon,
    /// Volume cell with a template shape (not `Polyhedron`); nodes in `sm-mesh-api` template
    /// order.
    Cell(CellShape),
    /// General polyhedron given by its outward-oriented face loops (see
    /// [`Elements::push_polyhedron`]).
    Polyhedron,
}

/// Elements in CSR form (one flat node array plus offsets; no per-element allocation).
/// Polyhedra additionally store a face CSR over the same node array.
#[derive(Clone, Debug, PartialEq)]
pub struct Elements {
    shapes: Vec<ElemShape>,
    offsets: Vec<u32>,
    nodes: Vec<u32>,
    /// Per element, the index into `face_ends` where its faces start (`len() + 1` entries).
    face_start: Vec<u32>,
    /// Absolute end offset (into `nodes`) of every polyhedron face.
    face_ends: Vec<u32>,
}

impl Default for Elements {
    fn default() -> Self {
        Self::new()
    }
}

impl Elements {
    /// Empty element block.
    pub fn new() -> Self {
        Self {
            shapes: Vec::new(),
            offsets: vec![0],
            nodes: Vec::new(),
            face_start: vec![0],
            face_ends: Vec::new(),
        }
    }

    /// Empty element block with room for `n` elements of `k` nodes.
    pub fn with_capacity(n: usize, k: usize) -> Self {
        let mut e = Self::new();
        e.shapes.reserve(n);
        e.offsets.reserve(n);
        e.face_start.reserve(n);
        e.nodes.reserve(n * k);
        e
    }

    /// Append an element given by local node indices. Surface elements are oriented with the
    /// face normal (right-hand rule); volume cells use the template order of their shape.
    /// For [`ElemShape::Polyhedron`] use [`Elements::push_polyhedron`].
    pub fn push(&mut self, shape: ElemShape, local: &[u32]) {
        self.shapes.push(shape);
        self.nodes.extend_from_slice(local);
        self.offsets.push(self.nodes.len() as u32);
        self.face_start.push(self.face_ends.len() as u32);
    }

    /// Append a polyhedron given by outward-oriented face loops of local node indices.
    pub fn push_polyhedron(&mut self, faces: &[&[u32]]) {
        for f in faces {
            self.nodes.extend_from_slice(f);
            self.face_ends.push(self.nodes.len() as u32);
        }
        self.shapes.push(ElemShape::Polyhedron);
        self.offsets.push(self.nodes.len() as u32);
        self.face_start.push(self.face_ends.len() as u32);
    }

    /// Number of elements.
    pub fn len(&self) -> usize {
        self.shapes.len()
    }

    /// True if there are no elements.
    pub fn is_empty(&self) -> bool {
        self.shapes.is_empty()
    }

    /// Element `i`: shape and local node indices (for a polyhedron: all its face loops
    /// concatenated), or `None` if `i >= len()`.
    pub fn get(&self, i: usize) -> Option<(ElemShape, &[u32])> {
        let shape = *self.shapes.get(i)?;
        let (a, b) = (
            *self.offsets.get(i)? as usize,
            *self.offsets.get(i + 1)? as usize,
        );
        Some((shape, self.nodes.get(a..b)?))
    }

    /// The face loops of polyhedron `i`, or `None` if `i` is out of range or not a polyhedron.
    pub fn polyhedron_faces(&self, i: usize) -> Option<impl Iterator<Item = &[u32]>> {
        if self.shapes.get(i) != Some(&ElemShape::Polyhedron) {
            return None;
        }
        let start = *self.offsets.get(i)? as usize;
        let (f0, f1) = (
            *self.face_start.get(i)? as usize,
            *self.face_start.get(i + 1)? as usize,
        );
        let ends = self.face_ends.get(f0..f1)?;
        let nodes = &self.nodes;
        let mut prev = start;
        Some(ends.iter().filter_map(move |&e| {
            let s = nodes.get(prev..e as usize);
            prev = e as usize;
            s
        }))
    }

    /// All elements in order.
    pub fn iter(&self) -> impl Iterator<Item = (ElemShape, &[u32])> + '_ {
        (0..self.len()).filter_map(move |i| self.get(i))
    }

    /// Every local node index of every element (flat).
    pub fn indices(&self) -> &[u32] {
        &self.nodes
    }

    pub(crate) fn map_indices(&mut self, f: impl Fn(u32) -> u32) {
        for x in &mut self.nodes {
            *x = f(*x);
        }
    }
}

/// A new node created by a patch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PatchNode {
    /// Canonical key (its owner must be the target entity, a group member, or a child the
    /// target claims).
    pub key: NodeKey,
    /// Position.
    pub point: Point3,
    /// Parameter on the owner (must fit the owner's kind, see [`Param::fits`]).
    pub param: Param,
}

/// Structured view of a mapped surface patch: `ni x nj` node keys, row-major (`j * ni + i`).
/// Row `j = 0` runs along logical side 0 of the face layout, column `i = ni - 1` along side 1;
/// row `nj - 1` and column `0` run along sides 2 and 3 reversed. Lets a volume scheme (sweep,
/// map) walk lines of a neighbouring mapped face without knowing its private numbering.
///
/// **Exception: copy members.** The grid of a face generated as a copy keeps the root's `ni`,
/// `nj` and order with the keys mapped through the correspondence, so it follows the *image of
/// the root's layout*, which need not be the member's own logical side 0. Consumers that need
/// the member's sides must go by keys ([`NodeGrid::line_from`], [`NodeGrid::position`]), not by
/// the side numbering.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NodeGrid {
    /// Nodes along `i`.
    pub ni: u32,
    /// Nodes along `j`.
    pub nj: u32,
    /// The keys (`ni * nj` of them).
    pub nodes: Vec<NodeKey>,
}

impl NodeGrid {
    /// A grid of `ni x nj` keys, row-major.
    pub fn new(ni: u32, nj: u32, nodes: Vec<NodeKey>) -> Self {
        Self { ni, nj, nodes }
    }

    /// Node `(i, j)`, or `None` if outside the grid.
    pub fn at(&self, i: u32, j: u32) -> Option<NodeKey> {
        if i >= self.ni || j >= self.nj {
            return None;
        }
        self.nodes
            .get(j as usize * self.ni as usize + i as usize)
            .copied()
    }

    /// Grid position `(i, j)` of `key`. The boundary ring is searched first (that is where
    /// sweeps and copies ask), then the interior.
    pub fn position(&self, key: NodeKey) -> Option<(u32, u32)> {
        let (ni, nj) = (self.ni, self.nj);
        if ni == 0 || nj == 0 || self.nodes.len() != ni as usize * nj as usize {
            return None;
        }
        let hit = |i: u32, j: u32| self.at(i, j) == Some(key);
        for i in 0..ni {
            if hit(i, 0) {
                return Some((i, 0));
            }
            if hit(i, nj - 1) {
                return Some((i, nj - 1));
            }
        }
        for j in 1..nj.saturating_sub(1) {
            if hit(0, j) {
                return Some((0, j));
            }
            if hit(ni - 1, j) {
                return Some((ni - 1, j));
            }
        }
        for j in 1..nj.saturating_sub(1) {
            for i in 1..ni.saturating_sub(1) {
                if hit(i, j) {
                    return Some((i, j));
                }
            }
        }
        None
    }

    /// The grid line that starts at boundary node `key` and runs across the grid
    /// (perpendicular to the boundary side `key` lies on), or `None` if `key` is not on the
    /// grid boundary or is a grid corner.
    pub fn line_from(&self, key: NodeKey) -> Option<Vec<NodeKey>> {
        let (i, j) = self.position(key)?;
        let (li, lj) = (self.ni - 1, self.nj - 1);
        let corner = (i == 0 || i == li) && (j == 0 || j == lj);
        if corner {
            return None;
        }
        if j == 0 {
            (0..self.nj).map(|q| self.at(i, q)).collect()
        } else if j == lj {
            (0..self.nj).rev().map(|q| self.at(i, q)).collect()
        } else if i == 0 {
            (0..self.ni).map(|q| self.at(q, j)).collect()
        } else if i == li {
            (0..self.ni).rev().map(|q| self.at(q, j)).collect()
        } else {
            None
        }
    }
}

/// What one scheme invocation produced. Pure data: nothing is written to the mesh until commit.
///
/// Element indices `i < nodes.len()` name `nodes[i]`; otherwise they name
/// `refs[i - nodes.len()]`.
#[derive(Clone, Debug, PartialEq)]
pub struct Patch {
    /// The target.
    pub target: Target,
    /// Nodes created by this patch (owned by the target entity, a group member, or a claimed
    /// child).
    pub nodes: Vec<PatchNode>,
    /// Nodes of earlier jobs that the elements reference, without duplicates.
    pub refs: Vec<NodeKey>,
    /// Elements, as local indices into `nodes` then `refs`.
    pub elements: Elements,
    /// Single-block structured view (map, `Caps::STRUCTURED`). A multi-block scheme
    /// (submap, `Caps::BLOCK_STRUCTURED` only) need not attach one; walk its strips with
    /// [`Patch::strip_from`].
    pub grid: Option<NodeGrid>,
    /// Per-element owning entity: for group patches the member piece of a cell, or the
    /// claimed face of a surface element. Empty means every element belongs to the target
    /// entity; otherwise one entry per element.
    pub owners: Vec<EntityId>,
}

impl Patch {
    /// An empty patch for `target`.
    pub fn new(target: Target) -> Self {
        Self {
            target,
            nodes: Vec::new(),
            refs: Vec::new(),
            elements: Elements::new(),
            grid: None,
            owners: Vec::new(),
        }
    }

    /// The canonical key of local index `local`.
    pub fn key(&self, local: u32) -> Option<NodeKey> {
        let l = local as usize;
        match self.nodes.get(l) {
            Some(n) => Some(n.key),
            None => self.refs.get(l - self.nodes.len()).copied(),
        }
    }

    /// Number of local indices (`nodes.len() + refs.len()`).
    pub fn local_len(&self) -> usize {
        self.nodes.len() + self.refs.len()
    }

    /// The quad strip that starts at boundary node `key` and runs across the patch to the
    /// opposite boundary, as the node line through the strip (`key` first): the analogue of
    /// [`NodeGrid::line_from`] for a multi-block (`Caps::BLOCK_STRUCTURED`) quad mesh.
    ///
    /// With a grid, [`NodeGrid::line_from`] answers (fast path). Otherwise the element table is
    /// walked: from `key` along its interior edge, then at each node along the edge opposite
    /// the incoming one (the edge sharing no quad with it), until a boundary node is reached.
    /// Where more than one edge qualifies (a re-entrant boundary corner, an irregular interior
    /// node) the neighbour with the smallest key is taken, so the walk is deterministic.
    /// `None` if the patch has a non-quad element, `key` is not a boundary node of the patch,
    /// it is a corner with no interior edge (as `line_from` refuses grid corners), or the walk
    /// does not close on the boundary. Builds its edge index per call: O(E log E).
    pub fn strip_from(&self, key: NodeKey) -> Option<Vec<NodeKey>> {
        if let Some(g) = &self.grid
            && let Some(line) = g.line_from(key)
        {
            return Some(line);
        }
        if self.elements.is_empty() || self.elements.iter().any(|(s, _)| s != ElemShape::Quad) {
            return None;
        }
        // Undirected edge -> number of quads using it; node -> neighbours; quad membership.
        let mut uses: BTreeMap<(u32, u32), u32> = BTreeMap::new();
        let mut nbrs: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
        for (_, q) in self.elements.iter() {
            for c in 0..4 {
                let (a, b) = (q[c], q[(c + 1) % 4]);
                let e = (a.min(b), a.max(b));
                let n = uses.entry(e).or_insert(0);
                *n += 1;
                if *n == 1 {
                    nbrs.entry(a).or_default().push(b);
                    nbrs.entry(b).or_default().push(a);
                }
            }
        }
        let edge = |a: u32, b: u32| (a.min(b), a.max(b));
        let on_boundary = |n: u32| nbrs.get(&n).is_some_and(|v| v.iter().any(|m| uses.get(&edge(n, *m)) == Some(&1)));
        // Quad-mates of the edge (a, b): the nodes adjacent to `b` in quads containing a and b.
        let mates = |a: u32, b: u32| -> Vec<u32> {
            let mut out = Vec::new();
            for (_, q) in self.elements.iter() {
                if let (Some(ia), Some(ib)) = (q.iter().position(|x| *x == a), q.iter().position(|x| *x == b))
                    && ((ia + 1) % 4 == ib || (ib + 1) % 4 == ia)
                {
                    out.push(q[(ib + 1) % 4]);
                    out.push(q[(ib + 3) % 4]);
                }
            }
            out
        };
        let start = (0..self.local_len() as u32).find(|i| self.key(*i) == Some(key))?;
        if !on_boundary(start) {
            return None;
        }
        let pick = |cands: Vec<u32>| cands.into_iter().filter_map(|c| Some((self.key(c)?, c))).min().map(|(_, c)| c);
        let interior: Vec<u32> = nbrs.get(&start)?.iter().copied().filter(|m| uses.get(&edge(start, *m)) == Some(&2)).collect();
        let mut prev = start;
        let mut cur = pick(interior)?;
        let mut line = vec![key, self.key(cur)?];
        for _ in 0..self.local_len() {
            if on_boundary(cur) {
                return Some(line);
            }
            let side = mates(prev, cur);
            let next: Vec<u32> = nbrs.get(&cur)?.iter().copied().filter(|m| *m != prev && !side.contains(m)).collect();
            let n = pick(next)?;
            prev = cur;
            cur = n;
            line.push(self.key(cur)?);
        }
        None
    }
}

const REF_BIT: u32 = 1 << 31;

/// Builds a [`Patch`] without tracking the local numbering by hand: new nodes and references
/// get builder indices, deduplicated references, and `finish` renumbers elements.
#[derive(Clone, Debug)]
pub struct PatchBuilder {
    target: Target,
    nodes: Vec<PatchNode>,
    refs: Vec<NodeKey>,
    ref_ix: BTreeMap<NodeKey, u32>,
    elements: Elements,
    grid: Option<NodeGrid>,
    owners: Vec<EntityId>,
    cur_owner: Option<EntityId>,
}

impl PatchBuilder {
    /// A builder for `target`.
    pub fn new(target: Target) -> Self {
        Self {
            target,
            nodes: Vec::new(),
            refs: Vec::new(),
            ref_ix: BTreeMap::new(),
            elements: Elements::new(),
            grid: None,
            owners: Vec::new(),
            cur_owner: None,
        }
    }

    /// Add a node the patch creates; returns its builder index.
    pub fn node(&mut self, key: NodeKey, point: Point3, param: Param) -> u32 {
        self.nodes.push(PatchNode { key, point, param });
        (self.nodes.len() - 1) as u32
    }

    /// Reference a node of an earlier job; returns its builder index (the same index for the
    /// same key).
    pub fn refer(&mut self, key: NodeKey) -> u32 {
        if let Some(&i) = self.ref_ix.get(&key) {
            return i;
        }
        let i = REF_BIT | self.refs.len() as u32;
        self.refs.push(key);
        self.ref_ix.insert(key, i);
        i
    }

    /// Append an element over builder indices.
    pub fn push(&mut self, shape: ElemShape, local: &[u32]) {
        self.elements.push(shape, local);
        if let Some(o) = self.cur_owner {
            self.owners.push(o);
        }
    }

    /// Append a polyhedron over builder indices (outward-oriented face loops).
    pub fn push_polyhedron(&mut self, faces: &[&[u32]]) {
        self.elements.push_polyhedron(faces);
        if let Some(o) = self.cur_owner {
            self.owners.push(o);
        }
    }

    /// Set the owning entity of the elements pushed from now on (group patches: the member
    /// piece; claimed faces: the face). Once used, set it before every element.
    pub fn owner(&mut self, piece: EntityId) {
        self.cur_owner = Some(piece);
    }

    /// Attach the structured view.
    pub fn grid(&mut self, g: NodeGrid) {
        self.grid = Some(g);
    }

    /// Finish: renumber references after the new nodes.
    pub fn finish(mut self) -> Patch {
        let n = self.nodes.len() as u32;
        self.elements.map_indices(|x| {
            if x & REF_BIT != 0 {
                n + (x & !REF_BIT)
            } else {
                x
            }
        });
        Patch {
            target: self.target,
            nodes: self.nodes,
            refs: self.refs,
            elements: self.elements,
            grid: self.grid,
            owners: self.owners,
        }
    }
}

/// All patches generated so far plus the node table in **commit order** (vertex nodes first,
/// then every patch's nodes in merge order). Points and parameters are dense
/// structure-of-arrays; the `NodeKey` index is lookup-only and never iterated, so the content
/// does not depend on hashing or scheduling.
#[derive(Clone, Default)]
pub struct PatchSet {
    patches: BTreeMap<Target, Patch>,
    order: Vec<Target>,
    keys: Vec<NodeKey>,
    points: Vec<Point3>,
    params: Vec<Param>,
    index: HashMap<NodeKey, u32>,
    vertex_nodes: usize,
}

impl core::fmt::Debug for PatchSet {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("PatchSet")
            .field("patches", &self.patches.len())
            .field("nodes", &self.keys.len())
            .finish()
    }
}

impl PatchSet {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    /// Add a driver-created vertex node. Returns false for a duplicate key.
    pub(crate) fn add_vertex_node(&mut self, key: NodeKey, point: Point3) -> bool {
        let ok = self.add_node(key, point, Param::None);
        if ok {
            self.vertex_nodes = self.keys.len();
        }
        ok
    }

    fn add_node(&mut self, key: NodeKey, point: Point3, param: Param) -> bool {
        if self.index.contains_key(&key) {
            return false;
        }
        self.index.insert(key, self.keys.len() as u32);
        self.keys.push(key);
        self.points.push(point);
        self.params.push(param);
        true
    }

    /// Merge a patch: its nodes are appended in order. Returns the duplicate key on error and
    /// leaves the set unchanged.
    pub(crate) fn insert(&mut self, patch: Patch) -> Result<(), NodeKey> {
        let mut keys: Vec<NodeKey> = Vec::with_capacity(patch.nodes.len());
        for n in &patch.nodes {
            if self.index.contains_key(&n.key) {
                return Err(n.key);
            }
            keys.push(n.key);
        }
        keys.sort_unstable();
        if let Some(w) = keys.windows(2).find(|w| w[0] == w[1]) {
            return Err(w[0]);
        }
        for n in &patch.nodes {
            self.add_node(n.key, n.point, n.param);
        }
        self.order.push(patch.target);
        self.patches.insert(patch.target, patch);
        Ok(())
    }

    /// The set without the patches of `targets` (their nodes are dropped). Consumes `self`,
    /// moving the remaining patches instead of cloning them.
    pub(crate) fn without(mut self, targets: &[Target]) -> PatchSet {
        let mut out = PatchSet::new();
        for i in 0..self.vertex_nodes {
            out.add_vertex_node(self.keys[i], self.points[i]);
        }
        for t in std::mem::take(&mut self.order) {
            if targets.contains(&t) {
                continue;
            }
            if let Some(p) = self.patches.remove(&t) {
                // Cannot fail: the keys were unique in `self`.
                let _ = out.insert(p);
            }
        }
        out
    }

    /// The patch of `t`.
    pub fn get(&self, t: Target) -> Option<&Patch> {
        self.patches.get(&t)
    }

    /// Dense commit-order index of a node.
    pub fn index_of(&self, k: NodeKey) -> Option<u32> {
        self.index.get(&k).copied()
    }

    /// Position of a node.
    pub fn point(&self, k: NodeKey) -> Option<Point3> {
        self.index_of(k)
            .and_then(|i| self.points.get(i as usize))
            .copied()
    }

    /// Position and parameter of a node.
    pub fn node(&self, k: NodeKey) -> Option<(Point3, Param)> {
        let i = self.index_of(k)? as usize;
        Some((*self.points.get(i)?, *self.params.get(i)?))
    }

    /// Every patch in target order.
    pub fn patches(&self) -> impl Iterator<Item = &Patch> {
        self.patches.values()
    }

    /// Every node in commit order.
    pub fn nodes(&self) -> impl Iterator<Item = (NodeKey, Point3, Param)> + '_ {
        (0..self.keys.len()).map(move |i| (self.keys[i], self.points[i], self.params[i]))
    }

    /// Number of nodes.
    pub fn node_count(&self) -> usize {
        self.keys.len()
    }

    /// The targets in merge order.
    pub fn merge_order(&self) -> &[Target] {
        &self.order
    }
}
