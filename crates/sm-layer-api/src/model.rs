//! The `model` layer: topology, shape and model-semantics tables, plus the transaction helpers
//! every op that edits them goes through.

use std::sync::Arc;

use sm_core::{EntityId, EntityKind};
use sm_geom_api::ModelTolerance;
use sm_mesh_api::HashQuanta;
use sm_model_api::{IdAlloc, Model};
use sm_op_api::{ApplyCtx, Digest, LayerType, OpError, State, names};
use sm_topo_api::{CellComplex, Dim, HashWord, OpId, OpSummary, TopologyView};

use crate::error::{LayerError, LayerResultExt};
use crate::host::TopoHost;

/// The shared `model` layer (`names::MODEL`).
///
/// Holds the topology store behind [`TopoHost`] (the production `sm_topo::TopoStore`, or the
/// reference `CellComplex` in tests), the model-semantics tables ([`Model`]) and the count of
/// topology operations run. Cloning copies two `Arc`s. Ops never mutate it directly: they go
/// through [`topo_txn`] + [`commit_topology`] (topology edits) or [`model_txn`] (model-table
/// edits), which keep the entity table, the lineage and the model tables in step.
///
/// # Digest (version 1)
/// In order: the stream tag `1` (`Digest::u64`), the [`TopologyView::canonical_visit`] stream
/// (`HashWord::Int` as `Digest::u64`, `HashWord::Float { raw, q }` as
/// `Digest::f64_with(raw, q)`), the stream tag `2`, [`Model::canonical_visit`] under the
/// topology tolerance with the same word mapping, then `next_op` (`Digest::u64`). While a
/// [`topo_txn`] has not been committed (the `pending` flag) one more `Digest::u64(u64::MAX)`
/// marker follows, so a forgotten [`commit_topology`] changes the hash; committed states never
/// carry it. The identity hash is therefore quantized (W0c Q2) and the exact hash bit-for-bit.
/// Two hosts that emit the same word stream (the reference and the production store) give the
/// same layer hash.
///
/// # Known deviations
/// - Curve and surface store handles (`shapes_mut`) are shape handles, not entities: they are
///   never adopted into the entity table and never appear in a `Delta`.
/// - The topology operation id is `OpId(ops_run + 1)`: it counts committed topology
///   transactions of this layer, not entity-table serials.
/// - [`topo_txn`] and [`model_txn`] deep-clone the topology host / [`Model`] when the state is
///   shared with another state (copy-on-write at whole-store granularity); a persistent
///   structure is a known W3 item.
#[derive(Clone)]
pub struct ModelLayer {
    topo: Arc<dyn TopoHost>,
    model: Arc<Model>,
    /// Topology operations committed so far; the next one is `OpId(next_op + 1)`.
    next_op: u64,
    /// A `topo_txn` ran and `commit_topology` has not: ids are not adopted yet.
    pending: bool,
}

impl Default for ModelLayer {
    /// An empty layer over the reference [`CellComplex`]. Distributions that use another store
    /// build it with [`ModelLayer::new`] (see [`CoreLayers`](crate::CoreLayers)).
    fn default() -> Self {
        Self::new(Box::new(CellComplex::new()))
    }
}

impl ModelLayer {
    /// An empty layer over `topo`, which should hold no cells yet.
    pub fn new(topo: Box<dyn TopoHost>) -> Self {
        Self {
            topo: Arc::from(topo),
            model: Arc::new(Model::new()),
            next_op: 0,
            pending: false,
        }
    }

    /// A layer over an existing shared topology host and model tables, with no committed
    /// topology operation (`ops_run() == 0`). Neither store is cloned.
    pub fn from_parts(topo: Arc<dyn TopoHost>, model: Arc<Model>) -> Self {
        Self {
            topo,
            model,
            next_op: 0,
            pending: false,
        }
    }

    /// Read-only topology.
    pub fn topo(&self) -> &dyn TopologyView {
        (*self.topo).as_view()
    }

    /// The shared topology host, for readers that keep it alive beyond the borrow of the layer
    /// (the host is immutable while shared).
    pub fn topo_host(&self) -> &Arc<dyn TopoHost> {
        &self.topo
    }

    /// The model-semantics tables.
    pub fn model(&self) -> &Model {
        &self.model
    }

    /// The topology tolerance policy.
    pub fn tolerance(&self) -> ModelTolerance {
        *self.topo.tolerance()
    }

    /// The quantization steps for hashing derived data of this model: `linear * 2^-10` for
    /// lengths and `parametric * 2^-10` for parameters (W0c Q2).
    pub fn hash_quanta(&self) -> HashQuanta {
        let t = self.tolerance();
        HashQuanta::new(t.quantum(), t.param_quantum())
    }

    /// Topology operations committed so far.
    pub fn ops_run(&self) -> u64 {
        self.next_op
    }
}

/// Unique mutable access to a shared host: clones it first when other states still share it.
fn host_mut(a: &mut Arc<dyn TopoHost>) -> Result<&mut dyn TopoHost, LayerError> {
    if Arc::get_mut(a).is_none() {
        *a = Arc::from(a.clone_host());
    }
    match Arc::get_mut(a) {
        Some(h) => Ok(h),
        None => Err(LayerError::Txn(
            "topology host could not be made unique".into(),
        )),
    }
}

impl LayerType for ModelLayer {
    const NAME: &'static str = names::MODEL;

    fn digest(&self, d: &mut Digest) {
        fn fold(d: &mut Digest, w: HashWord) {
            match w {
                HashWord::Int(v) => {
                    d.u64(v);
                }
                HashWord::Float { raw, q } => {
                    d.f64_with(raw, q);
                }
            }
        }
        d.u64(1);
        self.topo.canonical_visit(&mut |w| fold(d, w));
        d.u64(2);
        let tol = *self.topo.tolerance();
        self.model.canonical_visit(&tol, &mut |w| fold(d, w));
        d.u64(self.next_op);
        if self.pending {
            // An uncommitted topo_txn is a bug in the op; make its state visibly different.
            d.u64(u64::MAX);
        }
    }

    fn is_empty(&self) -> bool {
        let shapes = self.topo.shapes();
        self.next_op == 0
            && !self.pending
            && Dim::ALL.iter().all(|&dim| self.topo.cells(dim).is_empty())
            && shapes.curve_count() == 0
            && shapes.surface_count() == 0
            && *self.model == Model::new()
    }

    /// The layer reacts to retirement through [`commit_topology`] (lineage), never through
    /// `on_retired`, so it is never copied for it.
    fn references_any(&self, _ids: &[EntityId]) -> bool {
        false
    }
}

/// The result of a successful [`topo_txn`], consumed by [`commit_topology`].
///
/// Must be passed to [`commit_topology`] exactly once; until then the entity table does not know
/// the new ids and further transactions are refused.
#[derive(Clone, Debug)]
#[non_exhaustive]
#[must_use = "pass the transaction to commit_topology"]
pub struct TopoTxn {
    /// What the topology store reports: cells created (every id allocated, including cells
    /// that were created and killed again inside the operation) and retired, lineage range.
    pub summary: OpSummary,
    /// Ids minted by the model tables inside the transaction (colors, groups, boundaries, ...),
    /// ascending by `(kind, serial)`.
    pub model_created: Vec<EntityId>,
    /// Ids of model-table records (colors, groups, boundaries, ...) that `f` removed, ascending.
    /// [`commit_topology`] retires them like the cells in `summary.retired`.
    pub model_removed: Vec<EntityId>,
}

/// Every id of the records the model tables hold (colors, groups, boundaries, prototypes,
/// lattices, instances, clusters, partition cells), ascending and distinct.
///
/// O(records log records) per call, and `model_txn`/`topo_txn` call it twice per transaction even
/// when nothing is removed; acceptable next to the copy-on-write clone (W3 persistence item).
fn live_model_ids(m: &Model) -> Vec<EntityId> {
    let ins = m.instancing();
    let mut v: Vec<EntityId> = m
        .colors()
        .map(|r| r.id)
        .chain(m.groups().map(|r| r.id))
        .chain(m.boundaries().map(|r| r.id))
        .chain(ins.prototypes.keys().copied())
        .chain(ins.lattices.keys().copied())
        .chain(ins.instances.keys().copied())
        .chain(ins.clusters.keys().copied())
        .chain(m.decomposition().cells().map(|p| p.id))
        .collect();
    v.sort_unstable();
    v.dedup();
    v
}

/// Ids that existed before (`before`, sorted) or were minted (`minted`) and are no longer in
/// the model, ascending.
fn removed_ids(before: &[EntityId], minted: &[EntityId], after: &Model) -> Vec<EntityId> {
    let now = live_model_ids(after);
    let mut out: Vec<EntityId> = before
        .iter()
        .chain(minted)
        .copied()
        .filter(|id| now.binary_search(id).is_err())
        .collect();
    out.sort_unstable();
    out.dedup();
    out
}

/// Ids minted between two allocator states, ascending by `(kind, serial)`.
fn minted(before: &IdAlloc, after: &IdAlloc) -> Vec<EntityId> {
    let mut out = Vec::new();
    for byte in 0..=u8::MAX {
        let Some(kind) = EntityKind::from_u8(byte) else {
            continue;
        };
        let (lo, hi) = (before.peek(kind), after.peek(kind));
        out.extend((lo..hi).filter_map(|s| EntityId::new(kind, s)));
    }
    out
}

fn pending_error() -> OpError {
    LayerError::Txn("a previous topo_txn was not passed to commit_topology".into()).into()
}

/// Run a topology transaction on the `model` layer.
///
/// Opens an operation on the topology store (`begin_op(OpId(ops_run + 1))`), runs `f` with the
/// store and the model tables, and closes it (`end_op`). On `Err` from `f` or from `end_op`
/// the store is rolled back (`abort_op`), the model tables are restored and the layer is
/// unchanged. Ids minted by `f` in the model tables are found by diffing the model's id
/// allocator.
///
/// The ids are **not** adopted into the state's entity table yet: call [`commit_topology`] with
/// the returned [`TopoTxn`] before doing anything else with the layer.
///
/// Curve and surface handles minted through `shapes_mut` are shape handles, not entities: they
/// are not adopted and do not appear in the [`Delta`](sm_op_api::Delta).
///
/// Copy-on-write cost: when the topology host or the model tables are shared with another state
/// (every state that was cloned from this one), the whole store is deep-cloned before `f` runs.
/// That is O(model size) per transaction on a shared state, a known W3 item.
pub fn topo_txn<R>(
    ctx: &mut ApplyCtx<'_>,
    f: impl FnOnce(&mut dyn TopoHost, &mut Model) -> Result<R, OpError>,
) -> Result<(R, TopoTxn), OpError> {
    let layer = ctx.layer_mut::<ModelLayer>()?;
    if layer.pending {
        return Err(pending_error());
    }
    let op = OpId(layer.next_op + 1);
    let saved = Arc::clone(&layer.model);
    let ids_before = saved.ids().clone();
    let live_before = live_model_ids(&saved);
    let ran = (|| {
        let host = host_mut(&mut layer.topo).or_op()?;
        host.begin_op(op).or_op()?;
        let model = Arc::make_mut(&mut layer.model);
        let r = match f(&mut *host, model) {
            Ok(r) => r,
            Err(e) => {
                host.abort_op();
                return Err(e);
            }
        };
        match host.end_op() {
            Ok(summary) => Ok((r, summary)),
            Err(e) => {
                host.abort_op();
                Err(LayerError::from(e).into())
            }
        }
    })();
    match ran {
        Ok((r, summary)) => {
            layer.next_op += 1;
            layer.pending = true;
            let model_created = minted(&ids_before, layer.model.ids());
            let model_removed = removed_ids(&live_before, &model_created, &layer.model);
            Ok((
                r,
                TopoTxn {
                    summary,
                    model_created,
                    model_removed,
                },
            ))
        }
        Err(e) => {
            layer.model = saved;
            Err(e)
        }
    }
}

/// Every id of `ids` (ascending, distinct kinds are consecutive per kind) must be what the
/// entity table would allocate next, in order; roles must name ids of `ids` or live ids, once,
/// and never an id in `retiring` (sorted: what the same transaction retires).
fn check_adoption(
    state: &State,
    ids: &[EntityId],
    retiring: &[EntityId],
    roles: &[(&str, &[EntityId])],
) -> Result<(), OpError> {
    let mut next: std::collections::BTreeMap<EntityKind, u64> = std::collections::BTreeMap::new();
    for &id in ids {
        let want = match next.get(&id.kind()) {
            Some(&n) => n,
            None => state
                .peek_next(id.kind())
                .ok_or(OpError::IdExhausted)?
                .serial(),
        };
        if id.serial() != want {
            let expected = EntityId::new(id.kind(), want).ok_or(OpError::IdExhausted)?;
            return Err(OpError::IdMismatch { expected, got: id });
        }
        next.insert(id.kind(), want + 1);
    }
    let mut seen = std::collections::BTreeSet::new();
    for (role, members) in roles {
        if role.is_empty() {
            return Err(LayerError::Txn("empty role name".into()).into());
        }
        for m in *members {
            if retiring.binary_search(m).is_ok() {
                return Err(LayerError::Txn(format!(
                    "role '{role}' lists {m}, which this transaction retires"
                ))
                .into());
            }
            // An id this transaction did not create must already be live (a surviving id).
            if ids.binary_search(m).is_err() && !state.contains(*m) {
                return Err(LayerError::Txn(format!(
                    "role '{role}' lists {m}, which is neither created by this transaction nor live"
                ))
                .into());
            }
            if !seen.insert(*m) {
                return Err(LayerError::Txn(format!(
                    "{m} is listed in more than one role (or twice)"
                ))
                .into());
            }
        }
    }
    Ok(())
}

/// Adopt `ids` (ascending), recording each listed id under its role. Created ids and surviving
/// role members are merged in ascending `(kind, serial)` order, and runs of the same kind of
/// item and role are issued as one call, so every role stays ascending overall. The caller has
/// run [`check_adoption`].
fn adopt_all(
    ctx: &mut ApplyCtx<'_>,
    ids: &[EntityId],
    roles: &[(&str, &[EntityId])],
) -> Result<(), OpError> {
    let role_of: std::collections::BTreeMap<EntityId, &str> = roles
        .iter()
        .flat_map(|(r, ms)| ms.iter().map(move |m| (*m, *r)))
        .collect();
    // (id, created, role), ascending by id; created ids without a role are included.
    let mut items: Vec<(EntityId, bool, Option<&str>)> =
        Vec::with_capacity(ids.len() + role_of.len());
    items.extend(ids.iter().map(|&i| (i, true, role_of.get(&i).copied())));
    items.extend(
        role_of
            .iter()
            .filter(|(m, _)| ids.binary_search(m).is_err())
            .map(|(&m, &r)| (m, false, Some(r))),
    );
    items.sort_unstable_by_key(|it| it.0);
    let mut buf: Vec<EntityId> = Vec::new();
    let mut start = 0;
    while start < items.len() {
        let (_, created, role) = items[start];
        let mut end = start + 1;
        while end < items.len() && items[end].1 == created && items[end].2 == role {
            end += 1;
        }
        buf.clear();
        buf.extend(items[start..end].iter().map(|it| it.0));
        match (created, role) {
            (true, Some(r)) => ctx.adopt_as(r, &buf)?,
            (true, None) => ctx.adopt(&buf)?,
            (false, Some(r)) => ctx.record_role(r, &buf)?,
            (false, None) => {}
        }
        start = end;
    }
    Ok(())
}

/// Finish a [`topo_txn`]: adopt its ids, retire the cells it retired, tell the model tables.
///
/// In order:
/// 1. every created id (`summary.created` plus `model_created`) is adopted in ascending
///    `(kind, serial)` order, so [`Delta::created`](sm_op_api::Delta::created) lists exactly
///    the new cells and model ids;
/// 2. ids listed in `roles` are recorded under their role in [`Delta::outputs`](sm_op_api::Delta::outputs)
///    (in adoption order, which is ascending id order, not the order of the slices; an id may
///    appear in at most one role). A listed id that this transaction did not create must be
///    live (a surviving id, e.g. a moved volume); it is recorded with
///    [`ApplyCtx::record_role`]; created and surviving members are merged, so each role is
///    ascending overall;
/// 3. successors from the operation's split and merge lineage are recorded
///    ([`ApplyCtx::record_successors`]), skipping successors retired again in the same
///    transaction;
/// 4. `summary.retired` and `model_removed` are retired in ascending `(kind, serial)` order
///    (`on_retired` reactions of other layers run when the op body returns). A `model_removed`
///    id the entity table never adopted is skipped, as in [`model_txn`].
///
/// A role may not list an id that this same transaction retires (`summary.retired` or
/// `model_removed`): that is a [`LayerError::Txn`].
///
/// The model tables are re-keyed through the operation's lineage ([`Model::apply_lineage`])
/// before any id is adopted. The adoption and role checks run before anything is changed; a
/// failing `apply_lineage` (a decomposed volume was edited) leaves the tables unchanged. On any
/// `Err` the op should fail as a whole (the engine then discards the scratch state).
pub fn commit_topology(
    ctx: &mut ApplyCtx<'_>,
    txn: &TopoTxn,
    roles: &[(&str, &[EntityId])],
) -> Result<(), OpError> {
    let mut ids: Vec<EntityId> = txn
        .summary
        .created
        .iter()
        .chain(&txn.model_created)
        .copied()
        .collect();
    ids.sort_unstable();
    let mut gone: Vec<EntityId> = txn
        .summary
        .retired
        .iter()
        .chain(&txn.model_removed)
        .copied()
        .collect();
    gone.sort_unstable();
    gone.dedup();
    check_adoption(ctx.state(), &ids, &gone, roles)?;
    let lineage = {
        let layer = ctx.layer_mut::<ModelLayer>()?;
        if !layer.pending {
            return Err(
                LayerError::Txn("commit_topology without a pending topo_txn".into()).into(),
            );
        }
        let lineage = layer.topo.lineage().cut_outcome(txn.summary.op).lineage;
        // `apply_lineage` is transactional: on error the model is unchanged.
        Arc::make_mut(&mut layer.model)
            .apply_lineage(&lineage, (*layer.topo).as_view())
            .map_err(LayerError::from)?;
        layer.pending = false;
        lineage
    };
    adopt_all(ctx, &ids, roles)?;
    for row in &lineage {
        let live: Vec<EntityId> = row
            .new
            .iter()
            .copied()
            .filter(|n| txn.summary.retired.binary_search(n).is_err())
            .collect();
        if !live.is_empty() {
            ctx.record_successors(row.old, &live)?;
        }
    }
    for id in gone {
        // Model records the entity table never adopted are skipped (as in `model_txn`); cells
        // the topology store retired must be live.
        if txn.model_removed.binary_search(&id).is_ok()
            && txn.summary.retired.binary_search(&id).is_err()
            && !ctx.state().contains(id)
        {
            continue;
        }
        ctx.retire(id)?;
    }
    Ok(())
}

/// A model-only transaction (coloring, groups, blocks, side sets): edits the model tables with
/// read access to the topology. No topology operation is opened and `ops_run` is unchanged.
///
/// Every id `f` mints in the model tables is adopted in ascending `(kind, serial)` order;
/// `roles(&result, &new_ids)` names the roles (an id in at most one role, recorded in
/// adoption order). Atomic: on `Err` from `f`, or from an invalid role list or adoption
/// mismatch, the model tables are restored and nothing is adopted.
///
/// Records `f` removed (colors, groups, boundaries, ...) are retired in ascending
/// `(kind, serial)` order after the adoption, like [`commit_topology`] does for cells; an id that
/// `f` both minted and removed is adopted and retired again. Roles may name surviving live ids
/// (see [`commit_topology`]).
///
/// Copy-on-write cost: a model shared with another state is deep-cloned before `f` runs (a known
/// W3 item).
pub fn model_txn<R>(
    ctx: &mut ApplyCtx<'_>,
    f: impl FnOnce(&mut Model, &dyn TopologyView) -> Result<R, OpError>,
    roles: impl FnOnce(&R, &[EntityId]) -> Vec<(&'static str, Vec<EntityId>)>,
) -> Result<R, OpError> {
    let (saved, ran) = {
        let layer = ctx.layer_mut::<ModelLayer>()?;
        if layer.pending {
            return Err(pending_error());
        }
        let saved = Arc::clone(&layer.model);
        let ids_before = saved.ids().clone();
        let live_before = live_model_ids(&saved);
        let ran = f(Arc::make_mut(&mut layer.model), (*layer.topo).as_view()).map(|r| {
            let new_ids = minted(&ids_before, layer.model.ids());
            let removed = removed_ids(&live_before, &new_ids, &layer.model);
            (r, new_ids, removed)
        });
        if ran.is_err() {
            layer.model = Arc::clone(&saved);
        }
        (saved, ran)
    };
    let (r, new_ids, removed) = ran?;
    let role_list = roles(&r, &new_ids);
    let role_refs: Vec<(&str, &[EntityId])> =
        role_list.iter().map(|(n, v)| (*n, v.as_slice())).collect();
    let adopted = check_adoption(ctx.state(), &new_ids, &removed, &role_refs)
        .and_then(|()| adopt_all(ctx, &new_ids, &role_refs))
        .and_then(|()| {
            for &id in &removed {
                // Ids the entity table never adopted are skipped (same policy as `commit_topology`).
                if ctx.state().contains(id) {
                    ctx.retire(id)?;
                }
            }
            Ok(())
        });
    match adopted {
        Ok(()) => Ok(r),
        Err(e) => {
            ctx.layer_mut::<ModelLayer>()?.model = saved;
            Err(e)
        }
    }
}
