//! [`MeshTrackerBuilder`]: the `TrackerBuilder` implementation.

use crate::build::{Built, build_facets, build_mesh};
use crate::image::{import, make_info};
use crate::tracker::{MeshTracker, Parts};
use sm_mesh_api::{SetMembers, geometry_hash, topology_hash};
use sm_rt_api::{
    BuildInput, BuildOptions, BuildReport, CacheImage, CacheKey, ComplementPolicy, ContentHash, ContentHasher, RegionRule, Result, RtError, Tracker, TrackerBuilder,
    TrackerKind,
};

/// Algorithm version recorded in cache keys; bump when the built structure changes.
pub const BUILDER_VERSION: u32 = 1;

/// Builder of the mesh-native cell-walking tracker (`VOLUME_MESH | EXACT_PLANAR`).
///
/// Accepts [`BuildInput::Mesh`] and [`BuildInput::Facets`]; `Brep` and `World` inputs are
/// `RtError::Unsupported` (W3b).
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
        _ => RtError::Unsupported("unknown BuildInput variant"),
    }
}

impl MeshTrackerBuilder {
    /// Build the concrete tracker (the trait method boxes it).
    pub fn build_concrete(&self, input: &BuildInput<'_>, opts: &BuildOptions) -> Result<(MeshTracker, BuildReport)> {
        let key = self.cache_key(input, opts)?;
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
            BuildInput::Facets(f, b) => (build_facets(f, b, opts)?, Vec::new()),
            other => return Err(unsupported(other)),
        };
        let Built { core, regions, surfaces, complement, kind, report } = built;
        let hash = MeshTracker::hash_structure(&core, &regions, &surfaces, kind, complement, &cell_sets);
        let info = make_info(&core, &regions, &surfaces, kind, opts.tol, opts.max_events, hash, Some(key));
        let t = MeshTracker::assemble(Parts { core, regions, surfaces, kind, complement, cell_sets }, info, opts.block_size as usize, opts.recovery);
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
            other => return Err(unsupported(other)),
        };
        Ok(CacheKey::new(kind, BUILDER_VERSION, topo, ContentHash(0), boundary, opts.content_hash()))
    }

    fn build(&self, input: &BuildInput<'_>, opts: &BuildOptions) -> Result<(Box<dyn Tracker>, BuildReport)> {
        let (t, report) = self.build_concrete(input, opts)?;
        Ok((Box::new(t), report))
    }

    fn load(&self, input: &BuildInput<'_>, opts: &BuildOptions, image: &CacheImage) -> Result<(Box<dyn Tracker>, BuildReport)> {
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
