//! [`MeshTrackerBuilder`]: the `TrackerBuilder` implementation.

use crate::build::{Built, build_facets, build_mesh};
use crate::image::{import, make_info};
use crate::tracker::{MeshTracker, Parts};
use sm_mesh_api::{MeshRead, SetMembers, geometry_hash, topology_hash};
use std::sync::Arc;
use crate::world::WorldTracker;
use sm_rt_api::{
    BoundarySpec, BuildInput, BuildOptions, BuildReport, CacheImage, CacheKey, ComplementPolicy, ContentHash, ContentHasher, LeafInput, MeshInput, RegionRule, Result, RtError,
    Tracker, TrackerBuilder, TrackerKind,
};

/// Algorithm version recorded in cache keys; bump when the built structure changes.
pub const BUILDER_VERSION: u32 = 2;

/// Optional extras of a mesh or facet build beyond the contract's inputs.
///
/// Cell sets, tags and the shared mesh handle never change tracking results; tags enter the
/// structure hash and the cache key (so a tagged image is not mistaken for an untagged one).
#[derive(Clone, Default)]
pub struct BuildExtras {
    /// Shared handle to the source mesh; the built tracker's `Tracker::mesh` returns it.
    pub mesh: Option<Arc<dyn MeshRead + Send + Sync>>,
    /// Integer tags `(region name, key, value)`, attached to the named region in insertion order
    /// (`RegionInfo::tags`). A name no region carries is a build error (`region.tag_unknown`).
    pub tags: Vec<(String, String, i64)>,
}

impl BuildExtras {
    fn tag_hash(&self) -> ContentHash {
        if self.tags.is_empty() {
            return ContentHash(0);
        }
        let mut h = ContentHasher::new();
        h.write_u64(self.tags.len() as u64);
        for (r, k, v) in &self.tags {
            for s in [r, k] {
                h.write_u64(s.len() as u64);
                h.write_bytes(s.as_bytes());
            }
            h.write_u64(*v as u64);
        }
        h.finish()
    }
}

/// Builder of the mesh-native cell-walking tracker (`VOLUME_MESH | EXACT_PLANAR`).
///
/// Accepts [`BuildInput::Mesh`], [`BuildInput::Facets`] and (through the trait `build` or
/// [`MeshTrackerBuilder::build_world`]) [`BuildInput::Hier`], which gives a [`WorldTracker`];
/// `Brep` and `World` inputs are `RtError::Unsupported`.
#[derive(Clone, Copy, Debug, Default)]
pub struct MeshTrackerBuilder;

fn mesh_topology_hash(inp: &sm_rt_api::MeshInput<'_>) -> ContentHash {
    let m = inp.mesh;
    let mut h = ContentHasher::new();
    h.write_u64(topology_hash(m));
    h.write_u64(geometry_hash(m));
    match inp.regions {
        RegionRule::ByVolumeClassification => h.write_u64(1),
        RegionRule::ByCellSets(names) => {
            h.write_u64(2);
            for n in names {
                h.write_u64(n.len() as u64);
                h.write_bytes(n.as_bytes());
            }
        }
        _ => h.write_u64(0),
    }
    match inp.complement {
        ComplementPolicy::Outside => h.write_u64(0),
        ComplementPolicy::Region(n) => {
            h.write_u64(1);
            h.write_bytes(n.as_bytes());
        }
        _ => h.write_u64(9),
    }
    h.finish()
}

fn unsupported(input: &BuildInput<'_>) -> RtError {
    match input {
        BuildInput::Brep(_) => RtError::Unsupported("BuildInput::Brep (W3b)"),
        BuildInput::World(_) => RtError::Unsupported("BuildInput::World (W3b)"),
        BuildInput::Hier(_) => RtError::Unsupported("BuildInput::Hier needs MeshTrackerBuilder::build or build_world"),
        _ => RtError::Unsupported("unknown BuildInput variant"),
    }
}

impl MeshTrackerBuilder {
    /// Build the concrete tracker (the trait method boxes it).
    pub fn build_concrete(&self, input: &BuildInput<'_>, opts: &BuildOptions) -> Result<(MeshTracker, BuildReport)> {
        self.build_with(input, opts, &BuildExtras::default())
    }

    /// Build from a mesh held in an `Arc`: the tracker keeps the handle, so `Tracker::mesh()`
    /// returns `Some` (the Q2 reader path and tallies indexed by the mesh's own cell ids).
    pub fn build_arc(
        &self,
        mesh: Arc<dyn MeshRead + Send + Sync>,
        regions: &RegionRule,
        boundaries: &sm_rt_api::BoundarySpec,
        complement: &ComplementPolicy,
        opts: &BuildOptions,
    ) -> Result<(MeshTracker, BuildReport)> {
        let m: &dyn MeshRead = &*mesh;
        let input = BuildInput::Mesh(sm_rt_api::MeshInput::new(m, regions, boundaries, complement));
        self.build_with(&input, opts, &BuildExtras { mesh: Some(mesh.clone()), tags: Vec::new() })
    }

    /// Build with [`BuildExtras`] (shared mesh handle, region tags).
    pub fn build_with(&self, input: &BuildInput<'_>, opts: &BuildOptions, extras: &BuildExtras) -> Result<(MeshTracker, BuildReport)> {
        let mut key = self.cache_key(input, opts)?;
        key.hierarchy_hash = extras.tag_hash();
        build_flat(input, opts, extras, Some(key), true)
    }

    /// Build the hierarchical world tracker for a [`BuildInput::Hier`] input (see
    /// [`crate::WorldTracker`]). The trait `build` boxes the same value.
    pub fn build_world(&self, input: &BuildInput<'_>, opts: &BuildOptions) -> Result<(WorldTracker, BuildReport)> {
        let BuildInput::Hier(h) = input else { return Err(RtError::Unsupported("build_world needs BuildInput::Hier")) };
        let key = self.cache_key(input, opts)?;
        WorldTracker::build(h, opts, key)
    }
}

/// Build a prototype tracker for one leaf of a hierarchical world. The leaf's own boundary
/// conditions and complement policy do not apply inside a world: leaving the leaf is an event the
/// walker resolves (neighbour instance, or the world boundary), so the leaf is built with vacuum
/// boundaries and no complement.
pub(crate) fn build_proto(leaf: &LeafInput<'_>, opts: &BuildOptions) -> Result<(MeshTracker, BuildReport)> {
    // The world walker does not use a prototype's majorant partition (flights across worlds are
    // a later task), so prototypes are built without one and keep their structure unchanged.
    let mut plain = *opts;
    plain.partition = sm_rt_api::PartitionOptions::default();
    let opts = &plain;
    let vac = BoundarySpec::default();
    match leaf {
        LeafInput::Mesh(m) => {
            let outside = ComplementPolicy::Outside;
            let input = BuildInput::Mesh(MeshInput::new(m.mesh, m.regions, &vac, &outside));
            build_flat(&input, opts, &BuildExtras::default(), None, false)
        }
        LeafInput::Facets(f) => {
            let input = BuildInput::Facets(f, &vac);
            build_flat(&input, opts, &BuildExtras::default(), None, false)
        }
        LeafInput::Csg(_) => Err(RtError::Unsupported("CSG leaf (analytic tracker, W3b-RT-ANALYTIC)")),
        _ => Err(RtError::Unsupported("unknown leaf input variant")),
    }
}

fn build_flat(input: &BuildInput<'_>, opts: &BuildOptions, extras: &BuildExtras, key: Option<CacheKey>, with_complement: bool) -> Result<(MeshTracker, BuildReport)> {
    if opts.partition.region_jump && !opts.partition.enabled {
        return Err(RtError::Build(vec![sm_rt_api::BuildIssue::new("partition.region_jump", "region_jump needs PartitionOptions::enabled".to_owned())]));
    }
    {
        let (built, cell_sets) = match input {
            BuildInput::Mesh(m) => {
                let b = build_mesh(m, opts)?;
                let sets: Vec<(String, Vec<u32>)> = m
                    .mesh
                    .sets()
                    .iter()
                    .filter_map(|s| match &s.members {
                        SetMembers::Cells(c) => Some((s.name.clone(), c.iter().map(|x| x.0).collect())),
                        _ => None,
                    })
                    .collect();
                (b, sets)
            }
            BuildInput::Facets(f, b) => (build_facets(f, b, opts, with_complement)?, Vec::new()),
            other => return Err(unsupported(other)),
        };
        let Built { core, mut regions, surfaces, complement, kind, report } = built;
        for (name, k, v) in &extras.tags {
            let Some(r) = regions.0.iter_mut().find(|r| &r.name == name) else {
                return Err(RtError::Build(vec![sm_rt_api::BuildIssue::new("region.tag_unknown", format!("no region is named `{name}`"))]));
            };
            r.tags.push((k.clone(), *v));
        }
        let hash = MeshTracker::hash_structure(&core, &regions, &surfaces, kind, complement, &cell_sets);
        let info = make_info(&core, &regions, &surfaces, kind, opts.tol, opts.max_events, hash, key);
        let mut t = MeshTracker::assemble(Parts { core, regions, surfaces, kind, complement, cell_sets }, info, opts.block_size as usize, opts.recovery);
        if let Some(m) = &extras.mesh {
            t = t.with_mesh(m.clone());
        }
        if opts.partition.enabled {
            if kind == TrackerKind::Facets {
                return Err(RtError::Unsupported("majorant partition of a facet input"));
            }
            let params = crate::delta::Params { leaf_cells: opts.partition.leaf_cells, max_depth: opts.partition.max_depth, region_jump: opts.partition.region_jump };
            let part = crate::delta::Partition::build(&t.core, t.regions.0.len(), t.complement, opts.tol.linear, params)?;
            t = t.with_partition(part);
        }
        Ok((t, report))
    }
}

impl TrackerBuilder for MeshTrackerBuilder {
    fn builder_version(&self) -> u32 {
        BUILDER_VERSION
    }

    fn cache_key(&self, input: &BuildInput<'_>, opts: &BuildOptions) -> Result<CacheKey> {
        let (kind, topo, boundary) = match input {
            BuildInput::Mesh(m) => (TrackerKind::MeshWalk, mesh_topology_hash(m), m.boundaries.content_hash()),
            BuildInput::Facets(f, b) => (TrackerKind::Facets, f.content_hash(), b.content_hash()),
            BuildInput::Hier(h) => {
                return Ok(CacheKey::new(TrackerKind::World, BUILDER_VERSION, h.content_hash(), h.spec.content_hash(), h.boundaries.content_hash(), opts.content_hash()));
            }
            other => return Err(unsupported(other)),
        };
        Ok(CacheKey::new(kind, BUILDER_VERSION, topo, ContentHash(0), boundary, opts.content_hash()))
    }

    fn build(&self, input: &BuildInput<'_>, opts: &BuildOptions) -> Result<(Box<dyn Tracker>, BuildReport)> {
        if matches!(input, BuildInput::Hier(_)) {
            let (t, report) = self.build_world(input, opts)?;
            return Ok((Box::new(t), report));
        }
        let (t, report) = self.build_concrete(input, opts)?;
        Ok((Box::new(t), report))
    }

    fn load_image(&self, image: &CacheImage) -> Result<(Box<dyn Tracker>, BuildReport)> {
        let t = import(image)?;
        let mut report = BuildReport::default();
        report.from_cache = true;
        Ok((Box::new(t), report))
    }

    fn load(&self, input: &BuildInput<'_>, opts: &BuildOptions, image: &CacheImage) -> Result<(Box<dyn Tracker>, BuildReport)> {
        if matches!(input, BuildInput::Hier(_)) {
            return Err(RtError::Unsupported("hierarchical worlds have no cache image; rebuild"));
        }
        let key = self.cache_key(input, opts)?;
        image.check_key(&key)?;
        let t = import(image)?;
        let mut report = BuildReport::default();
        report.from_cache = true;
        Ok((Box::new(t), report))
    }
}

/// Restore a tracker from a cache image alone (no mesh, no staleness check against inputs):
/// what a `.smrt` sidecar opener such as the C ABI uses. The image is fully validated.
pub fn load_image(image: &CacheImage) -> Result<MeshTracker> {
    import(image)
}
