//! The scheme trait, its three phase contexts, input resolution and the registry.

use crate::error::{GenerateFailure, Result, SchemeError};
use crate::intent::{MeshIntent, Params, SchemeName};
use crate::patch::{Patch, PatchSet};
use crate::plan::Plan;
use crate::problem::{IntervalSink, Requirements, Role};
use crate::sizing::SizeField;
use crate::target::{NodeKey, SchemeDim, Target, piece_of};
use crate::vertex::FaceLayout;
use sm_core::{EntityId, EntityKind, Point3};
use sm_model_api::{Model, PieceFace, PieceRef};
use sm_topo_api::{CellFlags, CoedgeKey, Side, TopologyView, TopologyViewExt};
use std::collections::BTreeMap;

/// Whether a scheme can mesh a target, with a score for automatic selection.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Applicability {
    /// Applicable. Higher scores win auto selection (ties: auto-policy order, then name).
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
        Applicability::Applicable { score, reason: reason.into() }
    }
    /// Not applicable.
    pub fn no(reason: impl Into<String>) -> Self {
        Applicability::NotApplicable { reason: reason.into() }
    }
}

/// How pieces of the decomposition layer are seen through the topology view, and how node
/// owners are classified. The default [`WholeFaces`] handles whole volumes and cells whose
/// faces are all whole model faces; a decomposition-overlay resolver realizes cut faces.
pub trait PieceResolver: Send + Sync {
    /// Faces of the view bounding `piece`, ascending.
    fn faces(&self, topo: &dyn TopologyView, model: &Model, piece: PieceRef) -> Result<Vec<EntityId>>;
    /// The face of the view that realizes a piece face, if it is a single face.
    fn face_of(&self, topo: &dyn TopologyView, model: &Model, pf: &PieceFace) -> Option<EntityId>;
    /// The model or piece entity (`Topo*` or `PartitionCell`) a node owned by `owner` is
    /// classified on (virtual overlay cells map to their host entity).
    fn classify(&self, owner: EntityId) -> Option<EntityId>;
}

/// Resolver for whole volumes and for cells whose faces are all [`PieceFace::Whole`].
#[derive(Clone, Copy, Debug, Default)]
pub struct WholeFaces;

impl PieceResolver for WholeFaces {
    fn faces(&self, topo: &dyn TopologyView, model: &Model, piece: PieceRef) -> Result<Vec<EntityId>> {
        match piece {
            PieceRef::Whole(v) => Ok(topo.volume_faces(v).to_vec()),
            PieceRef::Cell(_) => {
                let mut out = Vec::new();
                for pf in model.decomposition().piece_faces(piece, topo) {
                    match pf {
                        PieceFace::Whole(f) => out.push(f),
                        other => return Err(SchemeError::Input(format!("piece {piece:?}: face {other:?} needs a decomposition-overlay resolver"))),
                    }
                }
                out.sort_unstable();
                out.dedup();
                Ok(out)
            }
        }
    }

    fn face_of(&self, _topo: &dyn TopologyView, _model: &Model, pf: &PieceFace) -> Option<EntityId> {
        match pf {
            PieceFace::Whole(f) => Some(*f),
            _ => None,
        }
    }

    fn classify(&self, owner: EntityId) -> Option<EntityId> {
        match owner.kind() {
            EntityKind::TopoVertex | EntityKind::TopoEdge | EntityKind::TopoFace | EntityKind::TopoVolume | EntityKind::PartitionCell => Some(owner),
            _ => None,
        }
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
    /// Pieces to mesh (`TopoVolume` or `PartitionCell` ids).
    pub selection: &'a [EntityId],
}

impl MeshInput<'_> {
    /// Faces of a volume target (via the resolver).
    pub fn piece_faces(&self, t: Target) -> Result<Vec<EntityId>> {
        let p = t.piece().ok_or_else(|| SchemeError::Input(format!("{t:?} is not a piece")))?;
        self.pieces.faces(self.topo, self.model, p)
    }

    /// Real (non-pole) edges of a face, ascending.
    pub fn face_edges(&self, f: EntityId) -> Vec<EntityId> {
        self.topo.face_edges(f).iter().copied().filter(|e| !self.topo.flags(*e).contains(CellFlags::DEGENERATE)).collect()
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
}

impl IntervalCtx<'_> {
    /// Layout of `face`.
    pub fn layout(&self, face: EntityId) -> Option<&FaceLayout> {
        self.layouts.binary_search_by(|l| l.face.cmp(&face)).ok().map(|i| &self.layouts[i])
    }
    /// Roles of `t`.
    pub fn roles_of(&self, t: Target) -> impl Iterator<Item = Role> + '_ {
        self.roles.iter().filter(move |r| r.0 == t).map(|r| r.1)
    }
}

/// Context of the generation phase: the solved plan and every patch of earlier waves.
/// Read-only; a scheme returns its patch and never touches the mesh.
pub struct GenCtx<'a> {
    /// The input.
    pub input: &'a MeshInput<'a>,
    /// The plan.
    pub plan: &'a Plan,
    /// Patches of earlier waves.
    pub patches: &'a PatchSet,
    /// The size field, if any.
    pub size: Option<&'a dyn SizeField>,
}

impl GenCtx<'_> {
    /// Solved interval count of `edge`.
    pub fn intervals(&self, edge: EntityId) -> Option<u32> {
        self.plan.intervals(edge)
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

    /// Position of a node of an earlier wave.
    pub fn point(&self, k: NodeKey) -> Option<Point3> {
        self.patches.point(k)
    }
}

/// A meshing scheme. Mostly declarative: it states what it needs (requirements, interval
/// equations) and only then generates, as a pure function of the solved plan.
///
/// Phases, all driven by the driver in a deterministic order:
/// 1. [`Scheme::applicability`] (auto selection, fallback chains).
/// 2. [`Scheme::requirements`]: child impositions, roles, copies, corner rules.
/// 3. [`Scheme::interval_constraints`]: after corners/roles are solved.
/// 4. [`Scheme::generate`]: returns a [`Patch`]; may run concurrently with other targets of
///    the same wave, so it must be pure (no interior mutability that changes results).
pub trait Scheme: Send + Sync {
    /// Registered name (unique per dimension).
    fn name(&self) -> &str;
    /// Dimension.
    fn dim(&self) -> SchemeDim;
    /// True if the scheme can mesh a [`Target::Group`] in one invocation.
    fn supports_groups(&self) -> bool {
        false
    }
    /// Can this scheme mesh `target`?
    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, params: &Params) -> Applicability;
    /// Contribute requirements (impositions on children, roles, copies, corner rules).
    fn requirements(&self, cx: &PlanCtx<'_>, target: Target, params: &Params, out: &mut Requirements<'_>) -> Result<()> {
        let _ = (cx, target, params, out);
        Ok(())
    }
    /// Contribute interval constraints (opposite sides, sums, parity, layer counts).
    fn interval_constraints(&self, cx: &IntervalCtx<'_>, target: Target, params: &Params, out: &mut IntervalSink<'_>) -> Result<()> {
        let _ = (cx, target, params, out);
        Ok(())
    }
    /// Generate the patch of `target`.
    fn generate(&self, cx: &GenCtx<'_>, target: Target, params: &Params) -> Result<Patch, GenerateFailure>;
}

/// A conversion applied to the patches of a group atomically after generation (THex, HTet,
/// QTri). All member patches are rewritten together or not at all.
pub trait MeshTransform: Send + Sync {
    /// Registered name.
    fn name(&self) -> &str;
    /// Rewrite `patch` (the group patch). On error nothing is committed.
    fn apply(&self, cx: &GenCtx<'_>, group: Target, params: &Params, patch: &mut Patch) -> Result<(), GenerateFailure>;
}

/// Schemes and transforms by name.
#[derive(Default)]
pub struct SchemeRegistry {
    schemes: BTreeMap<(SchemeDim, String), Box<dyn Scheme>>,
    transforms: BTreeMap<String, Box<dyn MeshTransform>>,
}

impl SchemeRegistry {
    /// An empty registry.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a scheme. The name `copy` is reserved; duplicates are rejected.
    pub fn register(&mut self, s: Box<dyn Scheme>) -> Result<()> {
        let key = (s.dim(), s.name().to_owned());
        if key.1 == SchemeName::COPY || self.schemes.contains_key(&key) {
            return Err(SchemeError::BadParams { scheme: SchemeName(key.1), reason: "name reserved or already registered".into() });
        }
        self.schemes.insert(key, s);
        Ok(())
    }

    /// Register a conversion.
    pub fn register_transform(&mut self, t: Box<dyn MeshTransform>) -> Result<()> {
        let k = t.name().to_owned();
        if self.transforms.contains_key(&k) {
            return Err(SchemeError::BadParams { scheme: SchemeName(k), reason: "already registered".into() });
        }
        self.transforms.insert(k, t);
        Ok(())
    }

    /// The scheme `name` of dimension `dim`.
    pub fn get(&self, dim: SchemeDim, name: &str) -> Option<&dyn Scheme> {
        self.schemes.get(&(dim, name.to_owned())).map(|b| b.as_ref())
    }

    /// The conversion `name`.
    pub fn transform(&self, name: &str) -> Option<&dyn MeshTransform> {
        self.transforms.get(name).map(|b| b.as_ref())
    }

    /// Schemes of a dimension in name order.
    pub fn of_dim(&self, dim: SchemeDim) -> impl Iterator<Item = &dyn Scheme> {
        self.schemes.iter().filter(move |(k, _)| k.0 == dim).map(|(_, v)| v.as_ref())
    }
}
