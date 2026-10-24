//! The scheme trait, its three phase contexts, input resolution and the registry.

use crate::caps::Caps;
use crate::error::{FailureKind, GenerateFailure, Result, SchemeError};
use crate::geom;
use crate::intent::{MeshIntent, Params, SchemeName};
use crate::patch::{ElemShape, NodeGrid, Patch, PatchBuilder, PatchSet};
use crate::plan::Plan;
use crate::problem::{IntervalSink, Requirements, Role, TargetPlan};
use crate::sizing::SizeField;
use crate::target::{NodeKey, SchemeDim, Target, piece_of};
use crate::vertex::FaceLayout;
use sm_core::{EntityId, Point3};
use sm_mesh_api::Param;
use sm_model_api::{Model, PieceFace, PieceRef};
use sm_topo_api::{CellFlags, CoedgeKey, Dim, Incidence, Side, TopologyView, TopologyViewExt};
use std::collections::BTreeMap;

/// Whether a scheme can mesh a target, with a score for automatic selection.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Applicability {
    /// Applicable. Higher scores win auto selection (ties: auto-list order, then name).
    /// Convention: 100 = exact fit (map on a 4-sided face), 50 = workable, 10 = last resort.
    Applicable {
        /// Score.
        score: u32,
        /// Why (shown in the plan).
        reason: String,
    },
    /// Not applicable.
    NotApplicable {
        /// Why not.
        reason: String,
    },
}

impl Applicability {
    /// Applicable with `score`.
    pub fn yes(score: u32, reason: impl Into<String>) -> Self {
        Applicability::Applicable {
            score,
            reason: reason.into(),
        }
    }
    /// Not applicable.
    pub fn no(reason: impl Into<String>) -> Self {
        Applicability::NotApplicable {
            reason: reason.into(),
        }
    }
}

/// How pieces of the decomposition layer are seen through the topology view, and how node
/// owners are classified. [`WholeFaces`] handles whole volumes and cells whose faces are all
/// whole model faces (including composites of whole volumes); [`DecompositionFaces`] also
/// resolves the cut and part faces of pieces split with `Model::split_realized`, through the
/// decomposition overlay.
pub trait PieceResolver: Send + Sync {
    /// Faces of the view bounding `piece`, ascending.
    fn faces(
        &self,
        topo: &dyn TopologyView,
        model: &Model,
        piece: PieceRef,
    ) -> Result<Vec<EntityId>>;
    /// The face of the view that realizes a piece face, if it is a single face.
    fn face_of(&self, topo: &dyn TopologyView, model: &Model, pf: &PieceFace) -> Option<EntityId>;
    /// The model or decomposition entity (`Topo*` or `Partition*`) a node owned by `owner` is
    /// classified on. Mesh entities may be classified on the virtual cells of the overlay
    /// (`PartitionVertex/Edge/Face`) and on pieces (`PartitionCell`) directly.
    fn classify(&self, owner: EntityId) -> Option<EntityId>;
}

/// Model and decomposition entity kinds a node owner can be classified on as itself.
fn classifiable(owner: EntityId) -> Option<EntityId> {
    sm_mesh_api::entity_dim(owner.kind()).map(|_| owner)
}

/// Resolver for whole volumes and for cells whose faces are all [`PieceFace::Whole`].
#[derive(Clone, Copy, Debug, Default)]
pub struct WholeFaces;

/// Resolver for every piece of the decomposition: whole volumes, cells, composites and pieces
/// split with `Model::split_realized`, whose cut and part faces resolve through
/// `Decomposition::piece_face_cells` to the cells of the view (the decomposition overlay the
/// split was realized in). Node owners are classified on themselves, virtual `Partition*`
/// cells included.
#[derive(Clone, Copy, Debug, Default)]
pub struct DecompositionFaces;

impl PieceResolver for DecompositionFaces {
    fn faces(
        &self,
        topo: &dyn TopologyView,
        model: &Model,
        piece: PieceRef,
    ) -> Result<Vec<EntityId>> {
        match piece {
            PieceRef::Whole(v) => Ok(topo.volume_faces(v).to_vec()),
            PieceRef::Cell(_) => {
                let d = model.decomposition();
                let mut out = Vec::new();
                for pf in d.piece_faces(piece, topo) {
                    let cells = d.piece_face_cells(&pf, topo).ok_or_else(|| {
                        SchemeError::Input(format!(
                            "piece {piece:?}: face {pf:?} has no cells in the view (realize the split with Model::split_realized and mesh through its overlay)"
                        ))
                    })?;
                    out.extend(cells);
                }
                out.sort_unstable();
                out.dedup();
                Ok(out)
            }
        }
    }

    fn face_of(&self, topo: &dyn TopologyView, model: &Model, pf: &PieceFace) -> Option<EntityId> {
        match model.decomposition().piece_face_cells(pf, topo)?.as_slice() {
            [one] => Some(*one),
            _ => None,
        }
    }

    fn classify(&self, owner: EntityId) -> Option<EntityId> {
        classifiable(owner)
    }
}

impl PieceResolver for WholeFaces {
    fn faces(
        &self,
        topo: &dyn TopologyView,
        model: &Model,
        piece: PieceRef,
    ) -> Result<Vec<EntityId>> {
        match piece {
            PieceRef::Whole(v) => Ok(topo.volume_faces(v).to_vec()),
            PieceRef::Cell(_) => {
                let mut out = Vec::new();
                for pf in model.decomposition().piece_faces(piece, topo) {
                    match pf {
                        PieceFace::Whole(f) => out.push(f),
                        other => {
                            return Err(SchemeError::Input(format!(
                                "piece {piece:?}: face {other:?} needs a decomposition-overlay resolver"
                            )));
                        }
                    }
                }
                out.sort_unstable();
                out.dedup();
                Ok(out)
            }
        }
    }

    fn face_of(
        &self,
        _topo: &dyn TopologyView,
        _model: &Model,
        pf: &PieceFace,
    ) -> Option<EntityId> {
        match pf {
            PieceFace::Whole(f) => Some(*f),
            _ => None,
        }
    }

    fn classify(&self, owner: EntityId) -> Option<EntityId> {
        classifiable(owner)
    }
}

/// Everything the driver meshes from: topology (possibly a decomposition overlay), the
/// model (colors, decks, pieces) and the selected pieces.
#[derive(Clone, Copy)]
pub struct MeshInput<'a> {
    /// The topology view.
    pub topo: &'a dyn TopologyView,
    /// The model.
    pub model: &'a Model,
    /// Piece resolution.
    pub pieces: &'a dyn PieceResolver,
    /// What to mesh: pieces (`TopoVolume` or `PartitionCell` ids), or faces of the view only
    /// (`TopoFace` / `PartitionFace` ids) for a surface-only mesh, committed into a
    /// dimension-2 builder. The two cannot be mixed.
    pub selection: &'a [EntityId],
}

impl MeshInput<'_> {
    /// Faces of a volume target (via the resolver).
    pub fn piece_faces(&self, t: Target) -> Result<Vec<EntityId>> {
        let p = t
            .piece()
            .ok_or_else(|| SchemeError::Input(format!("{t:?} is not a piece")))?;
        self.pieces.faces(self.topo, self.model, p)
    }

    /// Real (non-pole) edges of a face, ascending.
    pub fn face_edges(&self, f: EntityId) -> Vec<EntityId> {
        self.topo
            .face_edges(f)
            .iter()
            .copied()
            .filter(|e| !self.topo.flags(*e).contains(CellFlags::DEGENERATE))
            .collect()
    }

    /// Embedded (wire) edges of a face or piece (`Incidence::EMBEDDED` edge members of its
    /// boundary row in the view; pole edges skipped), ascending (W0f gaps 2). The planner gives
    /// each one a curve target, the parent's scheme must conform to it (see
    /// [`GenCtx::embedded`]).
    pub fn embedded_edges(&self, parent: EntityId) -> Vec<EntityId> {
        let t = self.topo;
        let mut v: Vec<EntityId> = t
            .boundary(parent)
            .iter()
            .filter(|i| i.ring == Incidence::EMBEDDED && t.dim_of(i.cell) == Some(Dim::Edge) && !t.flags(i.cell).contains(CellFlags::DEGENERATE))
            .map(|i| i.cell)
            .collect();
        v.sort_unstable();
        v.dedup();
        v
    }

    /// Embedded vertices of a face or piece (`Incidence::EMBEDDED` vertex members), ascending.
    /// Each gets a vertex node the parent's mesh must contain.
    pub fn embedded_vertices(&self, parent: EntityId) -> Vec<EntityId> {
        let t = self.topo;
        let mut v: Vec<EntityId> = t
            .boundary(parent)
            .iter()
            .filter(|i| i.ring == Incidence::EMBEDDED && t.dim_of(i.cell) == Some(Dim::Vertex))
            .map(|i| i.cell)
            .collect();
        v.sort_unstable();
        v.dedup();
        v
    }

    /// The piece of a volume-target id.
    pub fn piece(&self, id: EntityId) -> Option<PieceRef> {
        piece_of(id)
    }
}

/// Context of the applicability and requirements phases.
pub struct PlanCtx<'a> {
    /// The input.
    pub input: &'a MeshInput<'a>,
    /// The intent.
    pub intent: &'a MeshIntent,
}

/// Context of the interval phase: corner layouts and roles are solved.
pub struct IntervalCtx<'a> {
    /// The input.
    pub input: &'a MeshInput<'a>,
    /// Solved layouts, ascending by face.
    pub layouts: &'a [FaceLayout],
    /// Roles `(target, role)`.
    pub roles: &'a [(Target, Role)],
    /// The resolved targets, ascending (their `data` is what the scheme stored while
    /// planning).
    pub targets: &'a [TargetPlan],
}

impl IntervalCtx<'_> {
    /// Layout of `face`.
    pub fn layout(&self, face: EntityId) -> Option<&FaceLayout> {
        self.layouts
            .binary_search_by(|l| l.face.cmp(&face))
            .ok()
            .map(|i| &self.layouts[i])
    }
    /// Roles of `t`.
    pub fn roles_of(&self, t: Target) -> impl Iterator<Item = Role> + '_ {
        self.roles.iter().filter(move |r| r.0 == t).map(|r| r.1)
    }
    /// The plan row of `t`.
    pub fn target(&self, t: Target) -> Option<&TargetPlan> {
        self.targets
            .binary_search_by(|p| p.target.cmp(&t))
            .ok()
            .map(|i| &self.targets[i])
    }
    /// Scheme-private data stored for `t` ([`Requirements::set_data`]).
    pub fn data(&self, t: Target) -> Option<&Params> {
        self.target(t).map(|p| &p.data)
    }
}

/// One coedge's stretch of a [`BoundaryLoop`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BoundaryUse {
    /// The coedge (loop direction).
    pub coedge: CoedgeKey,
    /// Index into [`BoundaryLoop::nodes`] of the coedge's first node.
    pub first: u32,
    /// Number of segments (intervals) of the coedge.
    pub segments: u32,
}

/// A closed boundary loop of a face as node keys: what pave, tri* and polygon schemes need.
/// `nodes[i]` and `nodes[(i + 1) % len]` are consecutive (the last node connects to the
/// first); the face lies on the left.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BoundaryLoop {
    /// Loop index (0 = outer).
    pub ring: u16,
    /// The nodes in loop order, without repeating the first.
    pub nodes: Vec<NodeKey>,
    /// The coedges and where they start.
    pub uses: Vec<BoundaryUse>,
}

/// Context of the generation phase: the solved plan and every patch of earlier waves.
/// Read-only; a scheme returns its patch and never touches the mesh.
pub struct GenCtx<'a> {
    /// The input.
    pub input: &'a MeshInput<'a>,
    /// The plan.
    pub plan: &'a Plan,
    /// Patches of earlier waves. A target's patch here is the one left by every transform over
    /// it (W0f gaps 2: the last transform over a target is the writer its readers depend on, so
    /// a later wave sees the transformed patch).
    pub patches: &'a PatchSet,
    /// The size field, if any.
    pub size: Option<&'a dyn SizeField>,
}

impl GenCtx<'_> {
    /// Solved interval count of `edge`.
    pub fn intervals(&self, edge: EntityId) -> Option<u32> {
        self.plan.intervals(edge)
    }

    /// Scheme-private data stored for `t` ([`Requirements::set_data`]).
    pub fn data(&self, t: Target) -> Option<&Params> {
        self.plan.target(t).map(|p| &p.data)
    }

    /// Nodes of an edge from its start vertex to its end vertex: `[start, (e,1)..(e,n-1), end]`.
    pub fn edge_nodes(&self, edge: EntityId) -> Option<Vec<NodeKey>> {
        let (a, b) = self.input.topo.edge_ends(edge)?;
        let n = self.intervals(edge)?;
        let mut v = Vec::with_capacity(n as usize + 1);
        v.push(NodeKey::vertex(a));
        v.extend((1..n).map(|i| NodeKey::new(edge, i)));
        v.push(NodeKey::vertex(b));
        Some(v)
    }

    /// Nodes of a coedge in loop direction (start corner to end corner).
    pub fn coedge_nodes(&self, k: CoedgeKey) -> Option<Vec<NodeKey>> {
        let mut v = self.edge_nodes(k.edge)?;
        if k.side == Side::Right {
            v.reverse();
        }
        Some(v)
    }

    /// Nodes along logical side `side` of `face`'s layout, corner to corner.
    pub fn side_nodes(&self, face: EntityId, side: usize) -> Option<Vec<NodeKey>> {
        let s = self.plan.layout(face)?.sides.get(side)?;
        let mut out: Vec<NodeKey> = Vec::new();
        for k in &s.coedges {
            let n = self.coedge_nodes(*k)?;
            let skip = usize::from(!out.is_empty());
            out.extend_from_slice(&n[skip..]);
        }
        Some(out)
    }

    /// The boundary loops of `face` as node keys (pole coedges skipped), outer loop first.
    pub fn loops(&self, face: EntityId) -> Result<Vec<BoundaryLoop>> {
        let t = self.input.topo;
        let mut out = Vec::new();
        for lp in t.loops(face) {
            let mut bl = BoundaryLoop {
                ring: lp.ring,
                nodes: Vec::new(),
                uses: Vec::new(),
            };
            for k in lp
                .keys()
                .filter(|k| !t.flags(k.edge).contains(CellFlags::DEGENERATE))
            {
                let cn = self.coedge_nodes(k).ok_or_else(|| {
                    SchemeError::Input(format!("edge {} has no interval count", k.edge))
                })?;
                let first = bl.nodes.len() as u32;
                bl.nodes.extend_from_slice(&cn[..cn.len() - 1]);
                bl.uses.push(BoundaryUse {
                    coedge: k,
                    first,
                    segments: (cn.len() - 1) as u32,
                });
            }
            if !bl.nodes.is_empty() {
                out.push(bl);
            }
        }
        Ok(out)
    }

    /// The embedded members of a face or piece `parent` as node chains (W0f gaps 2): one chain
    /// per embedded edge, ascending by edge, `[start, (e,1)..(e,n-1), end]` in the edge's own
    /// direction ([`GenCtx::edge_nodes`]), then a one-node chain `[(v,0)]` for each embedded
    /// vertex that ends no embedded edge. Every key exists before the parent is generated (the
    /// parent's job reads the embedded curves). **Contract:** a scheme meshing a face or piece
    /// with embedded members must conform to them (every chain node is a node of its mesh,
    /// every chain segment an element edge, referenced through these keys) or report
    /// `NotApplicable`; the driver rejects a patch that does not reference every chain node.
    pub fn embedded(&self, parent: EntityId) -> Result<Vec<Vec<NodeKey>>> {
        let mut out: Vec<Vec<NodeKey>> = Vec::new();
        let mut ends: Vec<EntityId> = Vec::new();
        for e in self.input.embedded_edges(parent) {
            let chain = self.edge_nodes(e).ok_or_else(|| SchemeError::Input(format!("embedded edge {e} has no interval count")))?;
            if let Some((a, b)) = self.input.topo.edge_ends(e) {
                ends.extend([a, b]);
            }
            out.push(chain);
        }
        for v in self.input.embedded_vertices(parent) {
            if !ends.contains(&v) {
                out.push(vec![NodeKey::vertex(v)]);
            }
        }
        Ok(out)
    }

    /// The single-block structured view of an already generated face (map; see
    /// [`Patch::grid`]).
    pub fn grid(&self, face: EntityId) -> Option<&NodeGrid> {
        self.patches
            .get(Target::Surface(face))
            .and_then(|p| p.grid.as_ref())
    }

    /// The patch of an already generated face.
    pub fn surface_patch(&self, face: EntityId) -> Option<&Patch> {
        self.patches.get(Target::Surface(face))
    }

    /// The quad strip of an already generated face that starts at boundary node `key` (see
    /// [`Patch::strip_from`]); what a sweep walks across a `BLOCK_STRUCTURED` linking face.
    pub fn strip_from(&self, face: EntityId, key: NodeKey) -> Option<Vec<NodeKey>> {
        self.surface_patch(face)?.strip_from(key)
    }

    /// Position of a node of an earlier wave.
    pub fn point(&self, k: NodeKey) -> Option<Point3> {
        self.patches.point(k)
    }

    /// A valid curve patch for `edge` from `n - 1` increasing parameter fractions in `(0, 1)`
    /// measured from the edge's start vertex. Bias, dualbias and curvature schemes only
    /// compute fractions; positions, parameters and keys are the driver's business.
    pub fn curve_patch(&self, edge: EntityId, fractions: &[f64]) -> Result<Patch, GenerateFailure> {
        let target = Target::Curve(edge);
        let scheme = self
            .plan
            .target(target)
            .map_or("curve", |p| p.scheme.as_str());
        let fail = |kind: FailureKind, r: String| GenerateFailure::new(target, scheme, kind, r);
        let n = self.intervals(edge).ok_or_else(|| {
            fail(
                FailureKind::Internal,
                format!("edge {edge} has no interval count"),
            )
        })?;
        if fractions.len() != n.saturating_sub(1) as usize {
            return Err(fail(
                FailureKind::Internal,
                format!("{} fractions for {n} intervals", fractions.len()),
            ));
        }
        let mut prev = 0.0;
        for f in fractions {
            if !(f.is_finite() && *f > prev && *f < 1.0) {
                return Err(fail(
                    FailureKind::Geometry,
                    format!("fractions must increase strictly inside (0, 1); got {f} after {prev}"),
                ));
            }
            prev = *f;
        }
        let (a, b) = self
            .input
            .topo
            .edge_ends(edge)
            .ok_or_else(|| fail(FailureKind::Internal, "edge without ends".into()))?;
        let (c, cc) = geom::edge_curve(self.input.topo, edge)
            .map_err(|e| fail(FailureKind::Geometry, e.to_string()))?;
        let mut pb = PatchBuilder::new(target);
        let mut chain = Vec::with_capacity(n as usize + 1);
        chain.push(pb.refer(NodeKey::vertex(a)));
        for (i, f) in fractions.iter().enumerate() {
            let t = geom::edge_param(&cc, *f);
            chain.push(pb.node(NodeKey::new(edge, i as u32 + 1), c.eval(t), Param::Curve(t)));
        }
        chain.push(pb.refer(NodeKey::vertex(b)));
        for w in chain.windows(2) {
            pb.push(ElemShape::Segment, w);
        }
        Ok(pb.finish())
    }
}

/// A meshing scheme. Mostly declarative: it states what it needs (requirements, interval
/// equations) and only then generates, as a pure function of the solved plan.
///
/// Phases, all driven by the driver in a deterministic order:
/// 1. [`Scheme::applicability`] (auto selection, fallback chains).
/// 2. [`Scheme::requirements`]: child impositions, roles, claims, copies, corner rules.
/// 3. [`Scheme::interval_constraints`]: after corners/roles are solved.
/// 4. [`Scheme::generate`]: returns a [`Patch`]; may run concurrently with other targets of
///    the same wave, so it must be pure (no interior mutability that changes results).
pub trait Scheme: Send + Sync {
    /// Registered name (unique per dimension).
    fn name(&self) -> &str;
    /// Dimension.
    fn dim(&self) -> SchemeDim;
    /// What this scheme produces with `params`. Impositions are phrased over capabilities
    /// (see [`Caps`]), so a new structured scheme needs no edits in the schemes that impose.
    fn caps(&self, params: &Params) -> Caps;
    /// True if the scheme can mesh a [`Target::Group`] in one invocation.
    fn supports_groups(&self) -> bool {
        false
    }
    /// Can this scheme mesh `target`?
    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, params: &Params) -> Applicability;
    /// Contribute requirements (impositions on children, roles, claims, copies, corner rules).
    fn requirements(
        &self,
        cx: &PlanCtx<'_>,
        target: Target,
        params: &Params,
        out: &mut Requirements<'_>,
    ) -> Result<()> {
        let _ = (cx, target, params, out);
        Ok(())
    }
    /// Contribute interval constraints (opposite sides, sums, parity, layer counts).
    fn interval_constraints(
        &self,
        cx: &IntervalCtx<'_>,
        target: Target,
        params: &Params,
        out: &mut IntervalSink<'_>,
    ) -> Result<()> {
        let _ = (cx, target, params, out);
        Ok(())
    }
    /// Generate the patch of `target`.
    fn generate(
        &self,
        cx: &GenCtx<'_>,
        target: Target,
        params: &Params,
    ) -> Result<Patch, GenerateFailure>;
}

/// A **topology-preserving patch edit** applied to the patches of its targets atomically after
/// generation (smoothing, node snapping, reclassification): it may move nodes and change node
/// data but must keep the patch's nodes and elements (the driver re-checks every rewritten patch
/// with `check_patch`, which rejects nodes the target does not own). Element conversions (THex,
/// HTet, QTri, TQuad) are **not** `MeshTransform`s: they are IR -> IR mesh passes over
/// `sm_mesh_api::MeshRead -> MeshBuild` that re-verify deck pairs on the result (see
/// `DESIGN.md`).
pub trait MeshTransform: Send + Sync {
    /// Registered name.
    fn name(&self) -> &str;
    /// What the converted mesh provides.
    fn caps(&self, params: &Params) -> Caps;
    /// Rewrite `patches` (one per target, in target order) in place. The driver keeps the
    /// rewritten patches only if every transform of the plan succeeds.
    fn apply(
        &self,
        cx: &GenCtx<'_>,
        targets: &[Target],
        params: &Params,
        patches: &mut [Patch],
    ) -> Result<(), GenerateFailure>;
}

/// Schemes and transforms by name, plus fallback chains and automatic-selection lists.
#[derive(Default)]
pub struct SchemeRegistry {
    schemes: BTreeMap<SchemeDim, BTreeMap<String, Box<dyn Scheme>>>,
    transforms: BTreeMap<String, Box<dyn MeshTransform>>,
    fallbacks: BTreeMap<SchemeDim, BTreeMap<String, Vec<SchemeName>>>,
    auto: BTreeMap<SchemeDim, Vec<SchemeName>>,
}

impl SchemeRegistry {
    /// An empty registry.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a scheme. The name `copy` is reserved; duplicates are rejected.
    pub fn register(&mut self, s: Box<dyn Scheme>) -> Result<()> {
        let (dim, name) = (s.dim(), s.name().to_owned());
        let of_dim = self.schemes.entry(dim).or_default();
        if name == SchemeName::COPY || of_dim.contains_key(&name) {
            return Err(SchemeError::BadParams {
                scheme: SchemeName(name),
                reason: "name reserved or already registered".into(),
            });
        }
        of_dim.insert(name, s);
        Ok(())
    }

    /// Register a mesh transform (topology-preserving patch edit).
    pub fn register_transform(&mut self, t: Box<dyn MeshTransform>) -> Result<()> {
        let k = t.name().to_owned();
        if self.transforms.contains_key(&k) {
            return Err(SchemeError::BadParams {
                scheme: SchemeName(k),
                reason: "already registered".into(),
            });
        }
        self.transforms.insert(k, t);
        Ok(())
    }

    /// The scheme `name` of dimension `dim`.
    pub fn get(&self, dim: SchemeDim, name: &str) -> Option<&dyn Scheme> {
        self.schemes.get(&dim)?.get(name).map(|b| b.as_ref())
    }

    /// The mesh transform `name`.
    pub fn transform(&self, name: &str) -> Option<&dyn MeshTransform> {
        self.transforms.get(name).map(|b| b.as_ref())
    }

    /// Schemes of a dimension in name order.
    pub fn of_dim(&self, dim: SchemeDim) -> impl Iterator<Item = &dyn Scheme> {
        self.schemes
            .get(&dim)
            .into_iter()
            .flat_map(|m| m.values())
            .map(|v| v.as_ref())
    }

    /// Set the registry-level fallback chain of scheme `name`: tried, in order, after the
    /// last scheme of an intent chain that names `name` (Cubit's pave, then tridelaunay,
    /// then triadvance). Every name must be registered for `dim`.
    pub fn set_fallback(&mut self, dim: SchemeDim, name: &str, chain: &[&str]) -> Result<()> {
        for n in std::iter::once(&name).chain(chain.iter()) {
            if self.get(dim, n).is_none() {
                return Err(SchemeError::BadParams {
                    scheme: SchemeName::new(n),
                    reason: format!("no {dim:?} scheme of that name is registered"),
                });
            }
        }
        self.fallbacks.entry(dim).or_default().insert(
            name.to_owned(),
            chain.iter().map(|n| SchemeName::new(n)).collect(),
        );
        Ok(())
    }

    /// The fallback chain of `name` (empty if none).
    pub fn fallbacks(&self, dim: SchemeDim, name: &str) -> &[SchemeName] {
        self.fallbacks
            .get(&dim)
            .and_then(|m| m.get(name))
            .map_or(&[], Vec::as_slice)
    }

    /// Set the candidate list of automatic selection for `dim`, in priority order.
    /// `MeshIntent::defaults.auto` overrides it when non-empty.
    pub fn set_auto(&mut self, dim: SchemeDim, names: &[&str]) -> Result<()> {
        for n in names {
            if self.get(dim, n).is_none() {
                return Err(SchemeError::BadParams {
                    scheme: SchemeName::new(n),
                    reason: format!("no {dim:?} scheme of that name is registered"),
                });
            }
        }
        self.auto
            .insert(dim, names.iter().map(|n| SchemeName::new(n)).collect());
        Ok(())
    }

    /// The registry's automatic-selection list for `dim` (empty: every registered scheme).
    pub fn auto(&self, dim: SchemeDim) -> &[SchemeName] {
        self.auto.get(&dim).map_or(&[], Vec::as_slice)
    }
}
