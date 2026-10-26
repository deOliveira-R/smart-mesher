//! The `meshes` layer: every mesh of the state, keyed by its `Mesh` id.

use std::collections::BTreeMap;
use std::sync::Arc;

use sm_core::{EntityId, EntityKind};
use sm_mesh_api::{
    HashQuanta, MeshEntityKind, MeshRead, SerialAlloc, SetMembers, TagValues, topology_hash,
};
use sm_op_api::{ApplyCtx, Digest, LayerType, OpError, names};
use sm_scheme_api::Plan;

use crate::error::LayerError;
use crate::host::MeshHost;

/// One mesh in the `meshes` layer.
#[derive(Clone)]
#[non_exhaustive]
pub struct MeshEntry {
    /// The mesh. Shared between states; edited copy-on-write by cloning the host.
    pub mesh: Arc<dyn MeshHost>,
    /// The model entities (volumes, faces) this mesh was generated for.
    pub selection: Vec<EntityId>,
    /// The plan that produced it, kept for provenance. **Not hashed**: it holds computed floats
    /// and is a deterministic function of hashed content.
    pub plan: Option<Arc<Plan>>,
    /// Quantization steps of the digest, fixed when the entry was inserted
    /// ([`ModelLayer::hash_quanta`](crate::ModelLayer::hash_quanta)).
    pub quanta: HashQuanta,
    /// The model entities of the selection changed or were deleted after meshing; the mesh no
    /// longer matches the model.
    pub stale: bool,
}

impl MeshEntry {
    /// A fresh (not stale) entry.
    pub fn new(
        mesh: Box<dyn MeshHost>,
        selection: Vec<EntityId>,
        plan: Option<Plan>,
        quanta: HashQuanta,
    ) -> Self {
        Self {
            mesh: Arc::from(mesh),
            selection,
            plan: plan.map(Arc::new),
            quanta,
            stale: false,
        }
    }

    /// Read access to the mesh.
    pub fn mesh(&self) -> &dyn MeshRead {
        (*self.mesh).as_read()
    }
}

/// The shared `meshes` layer (`names::MESHES`, depends on `model`).
///
/// Holds a `BTreeMap<EntityId, MeshEntry>` plus the [`SerialAlloc`] that mints every mesh-kind
/// id. The allocator is the single source of those ids: an op clones it
/// ([`alloc`](Self::alloc)), builds a mesh with the clone, and hands the advanced clone to
/// [`insert`](Self::insert); [`adopt_minted`] then adopts exactly the ids the clone handed out,
/// so the entity table and the allocator never disagree. Cloning the layer copies only `Arc`s.
///
/// # Digest (version 1)
/// Per entry, in id order: the id, the selection, `stale`, both quanta (bit patterns), the exact
/// integer [`topology_hash`] of the mesh, the geometry rule, then every point coordinate through
/// `Digest::f64_quantized(x, quanta.length)`, every classification parameter (vertices, faces,
/// cells, edge table) through `Digest::f64_quantized(v, quanta.param)` and every real tag value
/// scale-free (40-bit mantissa, relative step about 1e-12). Then the allocator counters. Identity
/// hashes are therefore platform independent; exact (cache-key) hashes are bit-for-bit.
#[derive(Clone, Default)]
pub struct MeshLayer {
    entries: BTreeMap<EntityId, MeshEntry>,
    alloc: SerialAlloc,
}

impl MeshLayer {
    /// Entries in id order.
    pub fn meshes(&self) -> impl Iterator<Item = (EntityId, &MeshEntry)> {
        self.entries.iter().map(|(k, v)| (*k, v))
    }

    /// One entry.
    pub fn get(&self, mesh: EntityId) -> Option<&MeshEntry> {
        self.entries.get(&mesh)
    }

    /// The lowest-id mesh whose selection contains `entity` (a volume or a face).
    pub fn mesh_of(&self, entity: EntityId) -> Option<EntityId> {
        self.entries
            .iter()
            .find(|(_, e)| e.selection.contains(&entity))
            .map(|(k, _)| *k)
    }

    /// The allocator for mesh-kind ids (clone it to build a mesh, then pass the advanced clone
    /// to [`insert`](Self::insert)).
    pub fn alloc(&self) -> &SerialAlloc {
        &self.alloc
    }

    /// Number of meshes.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// True when there is no mesh.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Add a mesh and advance the allocator to `alloc_after`.
    ///
    /// `id` must be a new `Mesh` id equal to the id of the mesh in `entry`, and `alloc_after`
    /// must not be behind the current allocator for any kind (serials are never reused).
    pub fn insert(
        &mut self,
        id: EntityId,
        entry: MeshEntry,
        alloc_after: SerialAlloc,
    ) -> Result<(), LayerError> {
        self.check_insert(id, &entry, &alloc_after)?;
        self.entries.insert(id, entry);
        self.alloc = alloc_after;
        Ok(())
    }

    /// The validation of [`insert`](Self::insert), without changing the layer.
    pub fn check_insert(
        &self,
        id: EntityId,
        entry: &MeshEntry,
        alloc_after: &SerialAlloc,
    ) -> Result<(), LayerError> {
        if id.kind() != EntityKind::Mesh {
            return Err(LayerError::BadMeshId(format!("{id} is not a Mesh id")));
        }
        if self.entries.contains_key(&id) {
            return Err(LayerError::BadMeshId(format!(
                "{id} is already in the layer"
            )));
        }
        let own = entry.mesh().mesh_id();
        if own != id {
            return Err(LayerError::BadMeshId(format!(
                "{id} given, the mesh says {own}"
            )));
        }
        for (kind, next) in self.alloc.state() {
            if alloc_after.next_serial(kind) < next {
                return Err(LayerError::AllocRegressed(kind));
            }
        }
        Ok(())
    }

    /// Replace the host of an existing entry and advance the allocator to `alloc_after`.
    ///
    /// Checked and atomic: the entry must exist, `host` must carry the same `Mesh` id, and
    /// `alloc_after` must not be behind the current allocator for any kind. Selection, plan,
    /// quanta and `stale` of the entry are unchanged. Ids the new host holds beyond the old
    /// one's must be adopted by the caller (see [`add_mesh_set`]).
    pub fn replace_mesh(
        &mut self,
        mesh: EntityId,
        host: Arc<dyn MeshHost>,
        alloc_after: SerialAlloc,
    ) -> Result<(), LayerError> {
        self.check_replace(mesh, &*host, &alloc_after)?;
        if let Some(e) = self.entries.get_mut(&mesh) {
            e.mesh = host;
        }
        self.alloc = alloc_after;
        Ok(())
    }

    /// The validation of [`replace_mesh`](Self::replace_mesh), without changing the layer.
    pub fn check_replace(
        &self,
        mesh: EntityId,
        host: &dyn MeshHost,
        alloc_after: &SerialAlloc,
    ) -> Result<(), LayerError> {
        if !self.entries.contains_key(&mesh) {
            return Err(LayerError::BadMeshId(format!("{mesh} is not in the layer")));
        }
        let own = host.as_read().mesh_id();
        if own != mesh {
            return Err(LayerError::BadMeshId(format!(
                "{mesh} given, the mesh says {own}"
            )));
        }
        for (kind, next) in self.alloc.state() {
            if alloc_after.next_serial(kind) < next {
                return Err(LayerError::AllocRegressed(kind));
            }
        }
        Ok(())
    }

    /// Remove a mesh (its ids stay allocated; retire them with `ApplyCtx::retire`).
    pub fn remove(&mut self, mesh: EntityId) -> Option<MeshEntry> {
        self.entries.remove(&mesh)
    }

    /// Mutable access to an entry (mark stale, replace the mesh host).
    pub fn get_mut(&mut self, mesh: EntityId) -> Option<&mut MeshEntry> {
        self.entries.get_mut(&mesh)
    }
}

/// Scale-free quantization of a real tag value: subnormals and zero code as 0, otherwise the
/// mantissa is rounded to 40 bits and packed with exponent and sign (the rule of
/// `sm_mesh_api::geometry_hash_quantized`). Non-finite values are handled by the caller.
fn scale_free_code(x: f64) -> i64 {
    let a = x.abs();
    if (a.to_bits() >> 52) == 0 {
        return 0;
    }
    let c = ((a.to_bits() + (1u64 << 12)) >> 13) as i64;
    if x < 0.0 { -c } else { c }
}

fn digest_entry(d: &mut Digest, id: EntityId, e: &MeshEntry) {
    let m = e.mesh();
    d.id(id);
    d.u64(e.selection.len() as u64);
    for s in &e.selection {
        d.id(*s);
    }
    d.u64(u64::from(e.stale));
    d.u64(e.quanta.length.to_bits());
    d.u64(e.quanta.param.to_bits());
    d.u64(topology_hash(m));
    d.str(m.geometry_rule());
    let co = m.coords();
    d.u64(co.len() as u64);
    for i in 0..co.len() {
        d.f64_quantized(co.x[i], e.quanta.length);
        d.f64_quantized(co.y[i], e.quanta.length);
        d.f64_quantized(co.z[i], e.quanta.length);
    }
    let param = |d: &mut Digest, c: sm_mesh_api::Classification| {
        let [tag, a, b, cc] = c.param_bits();
        d.u64(tag);
        for (k, bits) in [a, b, cc].into_iter().enumerate() {
            if (k as u64) < tag {
                d.f64_quantized(f64::from_bits(bits), e.quanta.param);
            }
        }
    };
    for kind in [
        MeshEntityKind::Vertex,
        MeshEntityKind::Face,
        MeshEntityKind::Cell,
    ] {
        for i in 0..m.count(kind) {
            param(d, m.classification(kind, i as u32));
        }
    }
    if let Some(t) = m.edges() {
        d.u64(t.class.len() as u64);
        for c in &t.class {
            param(d, *c);
        }
    }
    for t in m.tags() {
        if let TagValues::F64(v) = &t.values {
            d.str(&t.name);
            d.u64(t.target as u64);
            d.u64(v.len() as u64);
            for &x in v {
                d.f64_with(x, if x.is_finite() { scale_free_code(x) } else { 0 });
            }
        }
    }
}

impl LayerType for MeshLayer {
    const NAME: &'static str = names::MESHES;
    const DEPENDS_ON: &'static [&'static str] = &[names::MODEL];

    fn digest(&self, d: &mut Digest) {
        d.u64(self.entries.len() as u64);
        for (id, e) in &self.entries {
            digest_entry(d, *id, e);
        }
        for (kind, next) in self.alloc.state() {
            d.u64(kind as u64);
            d.u64(next);
        }
    }

    fn is_empty(&self) -> bool {
        self.entries.is_empty() && self.alloc.state().next().is_none()
    }

    /// Marks the entries whose selection names a retired entity as stale (they keep their
    /// `Mesh*` ids, which stay live until `mesh.delete`), and drops entries whose own `Mesh` id
    /// was retired. The ids are sorted once and binary-searched, so the cost is
    /// O((ids + entries x selection) log ids).
    ///
    /// Retiring a `Mesh` id directly with `ctx.retire` drops the entry here but leaves its
    /// child ids live, and they can no longer be cleaned up: always delete meshes through
    /// [`retire_mesh`].
    fn on_retired(&mut self, ids: &[EntityId]) {
        let set = SortedIds::new(ids);
        self.entries.retain(|k, _| !set.contains(*k));
        for e in self.entries.values_mut() {
            if !e.stale && e.selection.iter().any(|s| set.contains(*s)) {
                e.stale = true;
            }
        }
    }

    fn references_any(&self, ids: &[EntityId]) -> bool {
        let set = SortedIds::new(ids);
        self.entries.iter().any(|(k, e)| {
            set.contains(*k) || (!e.stale && e.selection.iter().any(|s| set.contains(*s)))
        })
    }
}

/// A sorted view of an id slice for repeated membership tests. Borrows the slice when it is
/// already ascending (`Delta.removed` is), otherwise sorts a copy.
struct SortedIds<'a>(std::borrow::Cow<'a, [EntityId]>);

impl<'a> SortedIds<'a> {
    fn new(ids: &'a [EntityId]) -> Self {
        if ids.is_sorted() {
            return Self(std::borrow::Cow::Borrowed(ids));
        }
        let mut v = ids.to_vec();
        v.sort_unstable();
        v.dedup();
        Self(std::borrow::Cow::Owned(v))
    }

    fn contains(&self, id: EntityId) -> bool {
        self.0.binary_search(&id).is_ok()
    }
}

/// Adopt the ids a mesh build minted, given the allocator before and after: for each kind in
/// ascending order, the `Mesh` ids with [`ApplyCtx::adopt_as`] under `mesh_role` and every other
/// kind as one [`ApplyCtx::adopt_range`]. Call it in the same op that inserts the entry, with
/// `before` = [`MeshLayer::alloc`] as read before building and `after` = the advanced clone.
pub fn adopt_minted(
    ctx: &mut ApplyCtx<'_>,
    before: &SerialAlloc,
    after: &SerialAlloc,
    mesh_role: &str,
) -> Result<(), OpError> {
    for (kind, next) in after.state() {
        let first = before.next_serial(kind);
        if next <= first {
            continue;
        }
        let n = next - first;
        if kind == EntityKind::Mesh {
            let ids: Vec<EntityId> = (first..next)
                .filter_map(|s| EntityId::new(kind, s))
                .collect();
            ctx.adopt_as(mesh_role, &ids)?;
        } else {
            let first_id = EntityId::new(kind, first).ok_or(OpError::IdExhausted)?;
            ctx.adopt_range(first_id, n)?;
        }
    }
    Ok(())
}

/// Insert a freshly built mesh: [`adopt_minted`] then [`MeshLayer::insert`] in one call, so an
/// entry can never be inserted without its ids being adopted.
///
/// `before` is [`MeshLayer::alloc`] as read before building, `after` the advanced clone; the
/// entry's mesh id is the id inserted. The `Mesh` id is recorded under `mesh_role`.
///
/// The ids minted between `before` and `after` (per kind, serials `before.next_serial(kind)..`
/// `after.next_serial(kind)`) must be exactly the ids the mesh holds (`Mesh`, vertex, edge, face,
/// cell and set ids): [`retire_mesh`] retires the mesh's own ids, so an id minted and discarded
/// by the builder would stay adopted forever. A mismatch is [`LayerError::MintedNotInMesh`].
///
/// Atomic on every `Err`: the entry is checked ([`MeshLayer::check_insert`]), `before` must equal
/// the layer's allocator ([`LayerError::AllocMismatch`]), the minted set must match the mesh,
/// and the entity table must be ready to allocate each advanced kind at `before`'s serial
/// ([`OpError::IdMismatch`]), all before any id is adopted. A refused insert therefore leaves
/// the state untouched even if the caller swallows the error.
pub fn insert_mesh(
    ctx: &mut ApplyCtx<'_>,
    entry: MeshEntry,
    before: &SerialAlloc,
    after: SerialAlloc,
    mesh_role: &str,
) -> Result<(), OpError> {
    let id = entry.mesh().mesh_id();
    {
        let layer = ctx.layer::<MeshLayer>()?;
        layer.check_insert(id, &entry, &after)?;
        if before.state().ne(layer.alloc().state()) {
            return Err(LayerError::AllocMismatch(
                "the allocator before the build is not the layer's".into(),
            )
            .into());
        }
    }
    check_minted(id, entry.mesh(), before, &after)?;
    for (kind, next) in after.state() {
        let first = before.next_serial(kind);
        if next <= first {
            continue;
        }
        let want = EntityId::new(kind, first).ok_or(OpError::IdExhausted)?;
        match ctx.state().peek_next(kind) {
            Some(got) if got == want => {}
            Some(got) => {
                return Err(OpError::IdMismatch {
                    expected: got,
                    got: want,
                });
            }
            None => return Err(OpError::IdExhausted),
        }
    }
    adopt_minted(ctx, before, &after, mesh_role)?;
    ctx.layer_mut::<MeshLayer>()?.insert(id, entry, after)?;
    Ok(())
}

/// Add a named set to the mesh `mesh` of the `meshes` layer and adopt its id under `role`.
///
/// Clones the host, calls [`MeshEdit::add_set`](sm_mesh_api::MeshEdit::add_set) with a clone of
/// the layer allocator, checks that exactly one `MeshSet` id (and nothing else) was minted and
/// that the entity table is ready to allocate it, adopts it with
/// [`ApplyCtx::adopt_as`] and stores the new host with [`MeshLayer::replace_mesh`]. Selection,
/// plan, quanta and `stale` of the entry are unchanged. Returns the new set id.
///
/// Atomic on every `Err` (unknown mesh, duplicate name, bad members, allocator or id mismatch):
/// nothing is adopted and the layer is untouched.
pub fn add_mesh_set(
    ctx: &mut ApplyCtx<'_>,
    mesh: EntityId,
    name: &str,
    members: SetMembers,
    role: &str,
) -> Result<EntityId, OpError> {
    let (mut host, before) = {
        let layer = ctx.layer::<MeshLayer>()?;
        let entry = layer.get(mesh).ok_or(OpError::UnknownEntity(mesh))?;
        (entry.mesh.clone_host(), layer.alloc().clone())
    };
    let mut after = before.clone();
    let id = host
        .add_set(name, members, &mut after)
        .map_err(LayerError::from)?;
    let first = before.next_serial(EntityKind::MeshSet);
    let minted_one = id.kind() == EntityKind::MeshSet
        && id.serial() == first
        && after.next_serial(EntityKind::MeshSet) == first + 1
        && after
            .state()
            .all(|(k, n)| k == EntityKind::MeshSet || n == before.next_serial(k));
    if !minted_one {
        return Err(LayerError::MintedNotInMesh(
            "add_set must mint exactly one MeshSet id".into(),
        )
        .into());
    }
    match ctx.state().peek_next(EntityKind::MeshSet) {
        Some(got) if got == id => {}
        Some(got) => {
            return Err(OpError::IdMismatch {
                expected: got,
                got: id,
            });
        }
        None => return Err(OpError::IdExhausted),
    }
    let host: Arc<dyn MeshHost> = Arc::from(host);
    ctx.layer::<MeshLayer>()?
        .check_replace(mesh, &*host, &after)?;
    ctx.adopt_as(role, &[id])?;
    ctx.layer_mut::<MeshLayer>()?
        .replace_mesh(mesh, host, after)?;
    Ok(id)
}

/// The minted set (per kind, `before..after`) must equal [`mesh_ids`]: same count, and every
/// mesh id inside its kind's minted range. One pass over the (sorted, distinct) mesh ids.
fn check_minted(
    mesh_id: EntityId,
    m: &dyn MeshRead,
    before: &SerialAlloc,
    after: &SerialAlloc,
) -> Result<(), LayerError> {
    let ranges: Vec<(EntityKind, u64, u64)> = after
        .state()
        .map(|(k, next)| (k, before.next_serial(k), next))
        .filter(|(_, first, next)| next > first)
        .collect();
    let minted: u64 = ranges.iter().map(|(_, f, n)| n - f).sum();
    let ids = mesh_ids(mesh_id, m);
    if minted != ids.len() as u64 {
        return Err(LayerError::MintedNotInMesh(format!(
            "{minted} ids were minted but the mesh holds {}",
            ids.len()
        )));
    }
    for id in &ids {
        let inside = ranges
            .iter()
            .any(|(k, f, n)| *k == id.kind() && (*f..*n).contains(&id.serial()));
        if !inside {
            return Err(LayerError::MintedNotInMesh(format!(
                "{id} of the mesh was not minted by this build"
            )));
        }
    }
    Ok(())
}

/// Every id that belongs to a mesh: the `Mesh` id, its vertices, edges, faces, cells and sets,
/// ascending by `(kind, serial)`.
fn mesh_ids(mesh_id: EntityId, m: &dyn MeshRead) -> Vec<EntityId> {
    let mut ids = vec![mesh_id];
    for kind in [
        MeshEntityKind::Vertex,
        MeshEntityKind::Edge,
        MeshEntityKind::Face,
        MeshEntityKind::Cell,
    ] {
        ids.extend((0..m.count(kind)).map(|i| m.id(kind, i as u32)));
    }
    ids.extend(m.sets().iter().map(|s| s.id));
    ids.sort_unstable();
    ids.dedup();
    ids
}

/// Delete a mesh: remove its [`MeshEntry`] and retire every id belonging to it (the `Mesh` id,
/// all vertex, edge, face and cell ids and all set ids) in ascending `(kind, serial)` order, so
/// the entity count returns to its pre-insert value and `on_retired` hooks of downstream layers
/// fire. Serials are never reused: the allocator keeps its counters.
///
/// Together with [`insert_mesh`]'s minted-set check this makes the lifecycle exact: the ids
/// adopted at insertion are precisely the ids retired here.
///
/// Atomic: fails with [`OpError::UnknownEntity`] when `mesh` is not in the layer, or when any of
/// its ids is no longer live (e.g. an op retired one cell of it), and then changes nothing.
///
/// This is the only valid way to delete a mesh. Calling `ctx.retire` on the `Mesh` id leaks
/// every child id.
pub fn retire_mesh(ctx: &mut ApplyCtx<'_>, mesh: EntityId) -> Result<(), OpError> {
    let ids = {
        let entry = ctx
            .layer::<MeshLayer>()?
            .get(mesh)
            .ok_or(OpError::UnknownEntity(mesh))?;
        mesh_ids(mesh, entry.mesh())
    };
    if let Some(&dead) = ids.iter().find(|i| !ctx.state().contains(**i)) {
        return Err(OpError::UnknownEntity(dead));
    }
    ctx.layer_mut::<MeshLayer>()?.remove(mesh);
    for id in ids {
        ctx.retire(id)?;
    }
    Ok(())
}
