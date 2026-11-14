//! The `model` layer: topology, shape and model-semantics tables, plus the transaction helpers
//! every op that edits them goes through.

use std::sync::Arc;

use sm_core::{EntityId, EntityKind};
use sm_geom_api::ModelTolerance;
use sm_mesh_api::HashQuanta;
use sm_model_api::{IdAlloc, Model};
use sm_op_api::{ApplyCtx, Digest, LayerType, OpError, State, names};
use sm_topo_api::{CellComplex, Dim, HashWord, OpId, OpSummary, Overlay, TopologyView};

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
/// **Overlay section (ACR-V1):** only when an overlay is set (`Some`), after `next_op` and before
/// the pending marker: the stream tag `3`, then the bytes of
/// `OverlayData::write_canonical(.., tolerance)` (`Digest::bytes`), which cover the host
/// revision, every recorded virtual op and the id counters. A layer that never had an overlay
/// digests exactly as before. An overlay with no ops still differs from none (its counters
/// are state), so `clear_overlay` restores the view but a timeline undo restores the hash.
///
/// # Virtual topology (ACR-V1)
/// The layer may hold one shared [`Overlay`] (`Option<Arc<Overlay>>`): the virtual topology
/// (composites, partitions) of the model, recorded and replayable. [`topo`](Self::topo) returns
/// the overlay's view when one is set, so meshing and querying code sees composites and
/// partitions without change; [`host`](Self::host) is the bare host that editing and exporting
/// code needs. With no overlay the two are the same object. [`topo_txn`] re-applies the
/// overlay to the edited host (`Overlay::rebase`) and refuses, rolling back, an edit that
/// breaks it ([`LayerError::OverlayRebase`]). The decomposition's realized splits
/// (`Model::split_realized`/`unsplit_realized`) take **this** overlay through
/// [`model_overlay_txn`] / [`ModelLayer::model_and_overlay_mut`], so their persisted cell ids
/// always resolve.
///
/// ## `topo()` call-site audit (ACR-V1)
/// *Editing / exporting* (switched to `host()`): `sm-boolean` (`guard_volumes`,
/// `volumes_of_bodies`, merge, merge_check, `WebcutPlane`, `WebcutSurface`, `subdivide`
/// guard and pin planning), `sm-model` (`xform::select`, `copy`, `xform2` anchor and rotation
/// frame), `sm-cadio` (`export_step`, the export query's alive check), `sm-csgbrep`
/// (`live_bodies`, `csg.copy` record), `sm-server` (`facets_of_model`, `source: model`
/// export of real geometry). *Meshing / querying* (keep `topo()`): `sm-mesher` (plan, auto,
/// census, intent, volfit, decompose queries), `sm-fem` lowering, `sm-smooth`, `sm-poly`,
/// `sm-tess` query, `sm-view`, `sm-rt` overlaps, `sm-diag` volume report, `sm-model` queries
/// (`topo.related`, entity listing, measure). `model_txn`'s closure gets the host view
/// (model tables are keyed on real cells); use [`model_overlay_txn`] for overlay access.
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
    /// The virtual topology over `topo` (ACR-V1), shared with the decomposition.
    overlay: Option<Arc<Overlay>>,
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
            overlay: None,
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
            overlay: None,
        }
    }

    /// Read-only topology as meshing and querying code sees it: the overlay's view when an
    /// overlay is set (virtual composites and partitions included), else the host.
    pub fn topo(&self) -> &dyn TopologyView {
        match &self.overlay {
            Some(o) => &**o,
            None => (*self.topo).as_view(),
        }
    }

    /// The bare host topology, without the overlay: what ops that edit or export real geometry
    /// must read. Same object as [`topo`](Self::topo) when no overlay is set. (A new method
    /// rather than [`topo_host`](Self::topo_host): that one returns the editable `Arc` for
    /// readers that keep the host alive, not a view.)
    pub fn host(&self) -> &dyn TopologyView {
        (*self.topo).as_view()
    }

    /// The virtual-topology overlay, if one was set.
    pub fn overlay(&self) -> Option<&Arc<Overlay>> {
        self.overlay.as_ref()
    }

    /// Set the overlay, replacing any previous one. It must have been built on this layer's
    /// current host: same revision and content hash, else [`LayerError::Overlay`].
    pub fn set_overlay(&mut self, overlay: Overlay) -> Result<(), LayerError> {
        let host = (*self.topo).as_view();
        if overlay.is_stale_against(host) || overlay.host_hash() != host.canonical_hash() {
            return Err(LayerError::Overlay(format!(
                "built on host revision {} (hash {:#x}), the layer's host is at revision {} (hash {:#x})",
                overlay.host().revision().0,
                overlay.host_hash(),
                host.revision().0,
                host.canonical_hash()
            )));
        }
        self.overlay = Some(Arc::new(overlay));
        Ok(())
    }

    /// Drop the overlay (and its id counters), returning it. The view returns to the host's;
    /// the digest returns to the no-overlay value.
    pub fn clear_overlay(&mut self) -> Option<Arc<Overlay>> {
        self.overlay.take()
    }

    fn overlay_or_new(&mut self) -> &mut Overlay {
        let host: Arc<dyn TopologyView> = self.topo.clone();
        Arc::make_mut(
            self.overlay
                .get_or_insert_with(|| Arc::new(Overlay::new(host))),
        )
    }

    /// Run `f` on the layer's overlay, creating an empty one over the current host on first
    /// use. A shared overlay (cloned state) is copied first. Not transactional on its own:
    /// use [`model_overlay_txn`] from an op. A caller that mints virtual ids through it must
    /// adopt them itself (`ApplyCtx::adopt`, ascending `(kind, serial)`): they are the
    /// `Overlay::data().next_serial` diff, ranks 0/1/2 = `PartitionVertex/Edge/Face`. It must
    /// also retire (`ApplyCtx::retire`) the virtual ids its edit removes (for example through
    /// `Overlay::remove`), or the entity table keeps dead ids.
    pub fn overlay_mut<R>(&mut self, f: impl FnOnce(&mut Overlay) -> R) -> R {
        f(self.overlay_or_new())
    }

    /// Like [`overlay_mut`](Self::overlay_mut), with the model tables as well: the shape
    /// `Model::split_realized(piece, plane, overlay)` and `unsplit_realized` need.
    pub fn model_and_overlay_mut<R>(&mut self, f: impl FnOnce(&mut Model, &mut Overlay) -> R) -> R {
        self.overlay_or_new();
        let model = Arc::make_mut(&mut self.model);
        let overlay = self.overlay.as_mut().map(Arc::make_mut);
        match overlay {
            Some(o) => f(model, o),
            None => unreachable!("overlay_or_new just set it"),
        }
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
        if let Some(o) = &self.overlay {
            let mut buf = Vec::new();
            o.data().write_canonical(&mut buf, &tol);
            d.u64(3);
            d.bytes(&buf);
        }
        if self.pending {
            // An uncommitted topo_txn is a bug in the op; make its state visibly different.
            d.u64(u64::MAX);
        }
    }

    fn is_empty(&self) -> bool {
        let shapes = self.topo.shapes();
        self.next_op == 0
            && !self.pending
            && self.overlay.is_none()
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

/// Ids the overlay minted between two `next_serial` states (rank 0, 1, 2 = vertex, edge, face).
/// Live virtual (partition) vertices, edges and faces of an overlay.
fn live_virtual(o: &Overlay) -> Vec<EntityId> {
    [Dim::Vertex, Dim::Edge, Dim::Face]
        .into_iter()
        .flat_map(|d| o.cells(d).iter().copied())
        .filter(|id| {
            matches!(
                id.kind(),
                EntityKind::PartitionVertex | EntityKind::PartitionEdge | EntityKind::PartitionFace
            )
        })
        .collect()
}

fn minted_virtual(before: [u64; 3], after: [u64; 3]) -> Vec<EntityId> {
    const KINDS: [EntityKind; 3] =
        [EntityKind::PartitionVertex, EntityKind::PartitionEdge, EntityKind::PartitionFace];
    let mut out = Vec::new();
    for (r, kind) in KINDS.into_iter().enumerate() {
        out.extend((before[r]..after[r]).filter_map(|s| EntityId::new(kind, s)));
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
    // With an overlay the host Arc is shared with it, so `host_mut` clones; keep the old one to
    // roll back to if the edit breaks the overlay.
    let saved_topo = layer.overlay.is_some().then(|| Arc::clone(&layer.topo));
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
    let ran = ran.and_then(|(r, summary)| {
        if let Some(o) = &layer.overlay {
            // Follow the edited host: re-apply the recorded virtual ops, or refuse the edit.
            let host: Arc<dyn TopologyView> = layer.topo.clone();
            let mut next = (**o).clone();
            next.rebase(host)
                .map_err(|e| OpError::from(LayerError::OverlayRebase(e)))?;
            layer.overlay = Some(Arc::new(next));
        }
        Ok((r, summary))
    });
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
            if let Some(t) = saved_topo {
                // The edit was applied and then refused (overlay rebase): put the host back.
                layer.topo = t;
            }
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
    model_txn_with(
        ctx,
        |layer| f(Arc::make_mut(&mut layer.model), (*layer.topo).as_view()),
        roles,
    )
}

/// [`model_txn`] for edits that also write the layer's [`Overlay`]: `f` gets the model tables
/// and the overlay (created empty over the host on first use). This is how
/// `Model::split_realized` / `unsplit_realized` are called, so that the cell ids they persist
/// resolve in THE layer's overlay (ACR-V1, vtopo finding 3). Same adoption, roles and
/// atomicity as [`model_txn`], and the virtual ids `f` drops from the overlay (live before or
/// minted here, dead after) are retired like removed records; on `Err` the overlay is restored too (to "none" if it did not
/// exist). No topology operation is opened and the host is untouched.
pub fn model_overlay_txn<R>(
    ctx: &mut ApplyCtx<'_>,
    f: impl FnOnce(&mut Model, &mut Overlay) -> Result<R, OpError>,
    roles: impl FnOnce(&R, &[EntityId]) -> Vec<(&'static str, Vec<EntityId>)>,
) -> Result<R, OpError> {
    model_txn_with(ctx, |layer| layer.model_and_overlay_mut(f), roles)
}

fn model_txn_with<R>(
    ctx: &mut ApplyCtx<'_>,
    f: impl FnOnce(&mut ModelLayer) -> Result<R, OpError>,
    roles: impl FnOnce(&R, &[EntityId]) -> Vec<(&'static str, Vec<EntityId>)>,
) -> Result<R, OpError> {
    let (saved, saved_overlay, ran) = {
        let layer = ctx.layer_mut::<ModelLayer>()?;
        if layer.pending {
            return Err(pending_error());
        }
        let saved = Arc::clone(&layer.model);
        let saved_overlay = layer.overlay.clone();
        let ids_before = saved.ids().clone();
        let live_before = live_model_ids(&saved);
        let serial_before = layer.overlay.as_ref().map_or([1; 3], |o| o.data().next_serial);
        let virtual_before = layer.overlay.as_deref().map(live_virtual).unwrap_or_default();
        let ran = f(layer).map(|r| {
            let mut new_ids = minted(&ids_before, layer.model.ids());
            // Only model-table ids can be "removed" (virtual ids live in the overlay).
            let mut removed = removed_ids(&live_before, &new_ids, &layer.model);
            // Virtual ids the overlay minted (cut face, loop cells, halves) are adopted too, so
            // the entity table stays in step with the overlay counters.
            let serial_after = layer.overlay.as_ref().map_or(serial_before, |o| o.data().next_serial);
            let minted_virt = minted_virtual(serial_before, serial_after);
            // Virtual ids that were live (or minted here) and are gone from the overlay are
            // retired, so the entity table and the Delta agree with the overlay.
            let alive = |id: &EntityId| layer.overlay.as_deref().is_some_and(|o| o.is_alive(*id));
            removed.extend(virtual_before.iter().chain(&minted_virt).filter(|id| !alive(id)));
            removed.sort_unstable();
            removed.dedup();
            new_ids.extend(minted_virt);
            new_ids.sort_unstable();
            (r, new_ids, removed)
        });
        if ran.is_err() {
            layer.model = Arc::clone(&saved);
            layer.overlay = saved_overlay.clone();
        }
        (saved, saved_overlay, ran)
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
            let layer = ctx.layer_mut::<ModelLayer>()?;
            layer.model = saved;
            layer.overlay = saved_overlay;
            Err(e)
        }
    }
}
