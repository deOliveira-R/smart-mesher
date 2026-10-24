//! The `meshes` layer: every mesh of the state, keyed by its `Mesh` id.

use std::collections::BTreeMap;
use std::sync::Arc;

use sm_core::{EntityId, EntityKind};
use sm_mesh_api::{
    HashQuanta, MeshEntityKind, MeshRead, SerialAlloc, TagValues, topology_hash,
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
        Self { mesh: Arc::from(mesh), selection, plan: plan.map(Arc::new), quanta, stale: false }
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
        self.entries.iter().find(|(_, e)| e.selection.contains(&entity)).map(|(k, _)| *k)
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
        if id.kind() != EntityKind::Mesh {
            return Err(LayerError::BadMeshId(format!("{id} is not a Mesh id")));
        }
        if self.entries.contains_key(&id) {
            return Err(LayerError::BadMeshId(format!("{id} is already in the layer")));
        }
        let own = entry.mesh().mesh_id();
        if own != id {
            return Err(LayerError::BadMeshId(format!("{id} given, the mesh says {own}")));
        }
        for (kind, next) in self.alloc.state() {
            if alloc_after.next_serial(kind) < next {
                return Err(LayerError::AllocRegressed(kind));
            }
        }
        self.entries.insert(id, entry);
        self.alloc = alloc_after;
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
    for kind in [MeshEntityKind::Vertex, MeshEntityKind::Face, MeshEntityKind::Cell] {
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
    /// was retired.
    fn on_retired(&mut self, ids: &[EntityId]) {
        self.entries.retain(|k, _| !ids.contains(k));
        for e in self.entries.values_mut() {
            if !e.stale && e.selection.iter().any(|s| ids.contains(s)) {
                e.stale = true;
            }
        }
    }

    fn references_any(&self, ids: &[EntityId]) -> bool {
        self.entries.iter().any(|(k, e)| {
            ids.contains(k) || (!e.stale && e.selection.iter().any(|s| ids.contains(s)))
        })
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
            let ids: Vec<EntityId> = (first..next).filter_map(|s| EntityId::new(kind, s)).collect();
            ctx.adopt_as(mesh_role, &ids)?;
        } else {
            let first_id = EntityId::new(kind, first).ok_or(OpError::IdExhausted)?;
            ctx.adopt_range(first_id, n)?;
        }
    }
    Ok(())
}
