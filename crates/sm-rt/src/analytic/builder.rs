//! [`AnalyticTrackerBuilder`]: the `TrackerBuilder` of CSG leaves.

use super::tracker::{ANALYTIC_BUILDER_VERSION, AnalyticTracker, cache_key};
use sm_rt_api::{BoundarySpec, ComplementPolicy, ContentHash, BuildInput, BuildOptions, BuildReport, CacheImage, CacheKey, ComponentKind, CsgLeafInput, HierInput, LeafInput, Result, RtError, Tracker, TrackerBuilder};

/// Builds an [`AnalyticTracker`] from a [`BuildInput::Hier`] whose root is one `Csg` leaf.
///
/// Worlds that place or tile CSG leaves (`Union`, `Lattice`) are built by the world walker
/// (`MeshTrackerBuilder::build_world`, `CsgWorld::world_tracker`), not here (see `DESIGN.md`,
/// "Analytic leaf" and "World").
#[derive(Clone, Copy, Debug, Default)]
pub struct AnalyticTrackerBuilder;

/// Is the world one bare CSG leaf (what the analytic tracker builds)?
pub(crate) fn is_bare_csg(h: &HierInput<'_>) -> bool {
    match h.spec.components.get(h.spec.root.get()) {
        Some(ComponentKind::Leaf { input }) => matches!(h.leaves.get(*input), Some(LeafInput::Csg(_))),
        _ => false,
    }
}

/// Build the prototype tracker of one CSG leaf hosted by a world: vacuum boundaries and no
/// complement region (the world owns what lies between and around instances, as for mesh leaves).
pub(crate) fn build_proto_csg(leaf: &CsgLeafInput<'_>, opts: &BuildOptions) -> Result<(AnalyticTracker, BuildReport)> {
    if opts.partition.enabled {
        return Err(RtError::Unsupported("majorant partition of a CSG leaf"));
    }
    let outside = ComplementPolicy::Outside;
    let l = CsgLeafInput::new(leaf.geom, leaf.cells, leaf.bounds, &outside);
    let bnd = BoundarySpec::default();
    let key = cache_key(&l, ContentHash(0), &bnd, opts);
    AnalyticTracker::build(&l, &bnd, opts, key)
}

fn root_leaf<'a>(h: &HierInput<'a>) -> Result<CsgLeafInput<'a>> {
    // `validate` is an error for a malformed spec; its notes are not fatal here.
    h.spec.validate(h.leaves.len())?;
    match h.spec.components.get(h.spec.root.get()) {
        Some(ComponentKind::Leaf { input }) => match h.leaves.get(*input) {
            Some(LeafInput::Csg(c)) => Ok(*c),
            _ => Err(RtError::Unsupported("the root leaf is not a CSG leaf")),
        },
        _ => Err(RtError::Unsupported("placed or tiled CSG leaves (Union, Lattice) need the world walker to host non-mesh leaves")),
    }
}

impl AnalyticTrackerBuilder {
    /// Build the concrete tracker (the trait method boxes it).
    pub fn build_concrete(&self, input: &BuildInput<'_>, opts: &BuildOptions) -> Result<(AnalyticTracker, BuildReport)> {
        let BuildInput::Hier(h) = input else { return Err(RtError::Unsupported("the analytic tracker builds from BuildInput::Hier with a CSG leaf")) };
        let leaf = root_leaf(h)?;
        if opts.partition.enabled {
            return Err(RtError::Unsupported("majorant partition of a CSG leaf"));
        }
        let key = cache_key(&leaf, h.spec.content_hash(), h.boundaries, opts);
        AnalyticTracker::build(&leaf, h.boundaries, opts, key)
    }
}

impl TrackerBuilder for AnalyticTrackerBuilder {
    fn builder_version(&self) -> u32 {
        ANALYTIC_BUILDER_VERSION
    }

    fn cache_key(&self, input: &BuildInput<'_>, opts: &BuildOptions) -> Result<CacheKey> {
        let BuildInput::Hier(h) = input else { return Err(RtError::Unsupported("the analytic tracker builds from BuildInput::Hier with a CSG leaf")) };
        let leaf = root_leaf(h)?;
        Ok(cache_key(&leaf, h.spec.content_hash(), h.boundaries, opts))
    }

    fn build(&self, input: &BuildInput<'_>, opts: &BuildOptions) -> Result<(Box<dyn Tracker>, BuildReport)> {
        let (t, r) = self.build_concrete(input, opts)?;
        Ok((Box::new(t), r))
    }

    fn load_image(&self, _image: &CacheImage) -> Result<(Box<dyn Tracker>, BuildReport)> {
        Err(RtError::Unsupported("analytic tracker has no cache image"))
    }

    fn load(&self, _input: &BuildInput<'_>, _opts: &BuildOptions, _image: &CacheImage) -> Result<(Box<dyn Tracker>, BuildReport)> {
        Err(RtError::Unsupported("analytic tracker has no cache image"))
    }
}
