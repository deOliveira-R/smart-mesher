//! `imprint_bodies`: split the faces of bodies along each other's intersections without sharing
//! anything. Each body keeps its own vertices and edges (new ones are created per body), so the
//! bodies stay independent until `merge` makes the coincident cells one.

use std::collections::{BTreeMap, BTreeSet};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_topo_api::{Coincidence, TopologyEdit, TopologyView, TopologyViewExt};

use crate::arrange;
use crate::bodies;
use crate::error::BooleanError;
use crate::faceclass;
use crate::merge::{MergeKinds, MergeOptions, MergeReport, merge};
use crate::pieces::{self, PairSpec, Other, Share};

/// Options of [`imprint_bodies`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct ImprintOptions {
    /// Merge the coincident cells after imprinting (Cubit `imprint ... merge`).
    pub merge: bool,
    /// Merge tolerance (see [`MergeOptions::tol`]); used only with `merge`.
    pub tol: Option<f64>,
}

/// What [`imprint_bodies`] produced.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ImprintOutcome {
    /// The live fragments of every face that was split, ascending.
    pub faces: Vec<EntityId>,
    /// The live edges created by the imprint, ascending.
    pub edges: Vec<EntityId>,
    /// The merge report when `merge` was requested.
    pub merge: Option<MergeReport>,
}

/// Imprint `bodies` (bodies or volumes, at least two) on each other.
///
/// Every pair of faces of different operands whose surfaces intersect or coincide is cut along
/// the intersection, in each operand separately: the cut edges and their vertices are created
/// once per operand, never shared, and a coplanar overlap becomes the other face's boundary
/// inside this one. Afterwards faces that coincide exactly are recorded
/// `Coincidence::ImprintedNotMerged`. With `o.merge` the coincident vertices, edges and faces are
/// then merged ([`merge`]).
///
/// Runs inside an open operation. The cells are created in a fixed order: operands ascending,
/// within an operand pairs ascending by `(face, face)`, within a pair along the curve parameter.
///
/// # Errors
/// `Unsupported` for operands that are not manifold solids or for geometry the face arrangement
/// cannot express; `Ssi` if two surfaces cannot be intersected; `Topo` if an edit is refused.
pub fn imprint_bodies<E: TopologyEdit + ?Sized>(topo: &mut E, bodies_in: &[EntityId], o: &ImprintOptions) -> Result<ImprintOutcome, BooleanError> {
    if bodies_in.len() < 2 {
        return Err(BooleanError::Invalid("imprint needs at least two bodies".into()));
    }
    // One group per operand.
    let mut groups: Vec<Vec<EntityId>> = Vec::new();
    for &b in bodies_in {
        let vols = bodies::volumes_of(&*topo, &[b])?;
        for &v in &vols {
            bodies::check_manifold(&*topo, v)?;
        }
        groups.push(vols);
    }
    let group_faces: Vec<Vec<EntityId>> = groups.iter().map(|g| bodies::faces_of_volumes(&*topo, g)).collect();
    // A face that belongs to two operands is already shared: leave it out.
    let mut owner: BTreeMap<EntityId, Vec<usize>> = BTreeMap::new();
    for (gi, fs) in group_faces.iter().enumerate() {
        for &f in fs {
            owner.entry(f).or_default().push(gi);
        }
    }
    let faces_for: Vec<Vec<EntityId>> = group_faces.iter().map(|fs| fs.iter().copied().filter(|f| owner[f].len() == 1).collect()).collect();
    let tol = *topo.tolerance();
    let mut pairs: Vec<PairSpec> = Vec::new();
    for i in 0..groups.len() {
        for j in (i + 1)..groups.len() {
            for (a, b) in pieces::candidate_pairs(&*topo, &faces_for[i], &faces_for[j], tol.linear_max) {
                pairs.push(PairSpec { a, b: Other::Face(b) });
            }
        }
    }
    let group_of = |f: EntityId| -> usize { owner.get(&f).and_then(|g| g.first()).copied().unwrap_or(0) };
    let computed = pieces::compute(&*topo, &pairs, Share::Private, &group_of, 10.0 * tol.linear_max)?;
    let mut out = ImprintOutcome::default();
    let mut fragments: BTreeMap<EntityId, Vec<EntityId>> = BTreeMap::new();
    for g in 0..groups.len() {
        let ps: Vec<&pieces::Piece> = computed.pieces.iter().filter(|p| p.group == g).collect();
        let applied = arrange::apply(topo, &ps, tol.linear, &faces_for[g])?;
        out.edges.extend(applied.new_edges.iter().copied());
        fragments.extend(applied.fragments);
    }
    // Coincident pairs: record the state of the fragments that now match exactly.
    let lin = tol.linear;
    let cache = faceclass::FaceCache::new();
    for &(a, b) in &computed.coincident {
        let fa = fragments.get(&a).cloned().unwrap_or_else(|| vec![a]);
        let fb = fragments.get(&b).cloned().unwrap_or_else(|| vec![b]);
        for &x in &fa {
            let Some(p) = faceclass::interior_point(&*topo, &cache, x)? else { continue };
            for &y in &fb {
                if crate::locate::on_face(&*topo, &cache, y, p, lin)? && same_boundary(&*topo, x, y, lin) {
                    topo.record_coincidence(x.min(y), x.max(y), Coincidence::ImprintedNotMerged)?;
                }
            }
        }
    }
    let mut faces: BTreeSet<EntityId> = BTreeSet::new();
    for v in fragments.values() {
        faces.extend(v.iter().copied());
    }
    let frag_list: Vec<EntityId> = faces.iter().copied().collect();
    crate::pcurves::fix_faces(topo, &frag_list)?;
    if o.merge {
        let mut scope: Vec<EntityId> = Vec::new();
        for g in &groups {
            scope.extend(g.iter().copied());
        }
        let opts = MergeOptions { tol: o.tol, kinds: MergeKinds::default() };
        out.merge = Some(merge(topo, &scope, &[], &opts)?);
    }
    out.faces = faces.into_iter().filter(|f| topo.is_alive(*f)).collect();
    out.edges.sort();
    out.edges.retain(|e| topo.is_alive(*e));
    Ok(out)
}

/// True if the vertex position sets of two faces match within `tol` (the faces are imprinted onto
/// each other).
fn same_boundary<V: TopologyView + ?Sized>(view: &V, x: EntityId, y: EntityId, tol: f64) -> bool {
    let pts = |f: EntityId| -> Vec<sm_core::Point3> {
        let mut v = Vec::new();
        for l in view.loops(f) {
            v.extend(l.entries.iter().filter_map(|e| view.edge_ends(e.cell).and_then(|(s, _)| crate::geo::vertex_point(view, s))));
        }
        v
    };
    let (a, b) = (pts(x), pts(y));
    a.len() == b.len() && a.iter().all(|p| b.iter().any(|q| crate::geo::dist(*p, *q) <= tol))
}
