//! [`auto_size`]: the automatic element size of a selection.
//!
//! With `d` the diagonal of the bounding box of the selected geometry and `l_min` the length of
//! its shortest edge,
//!
//! ```text
//! auto_size(f) = clamp( 0.01 d 2^((f - 1) / 2),  l_min / 4,  d / 2 ),   f = 1..=10
//! ```
//!
//! (cubit-meshing-semantics §2): about `d/100` at `f = 1`, `d/25` at `f = 5`, `d/4.4` at
//! `f = 10`, never finer than a quarter of the shortest edge and never coarser than half the
//! diagonal (when the two bounds cross, the lower bound `l_min / 4` wins). The expression is non-decreasing in `f` (a clamp of an increasing function).
//!
//! Selection: volumes contribute their face edges, faces their edges, edges themselves;
//! vertices and other kinds are ignored; an empty selection means every live edge of the model.
//! The bounding box covers all selected vertices plus 17 uniformly spaced curve points per
//! edge (so a bulging arc counts), accumulated in ascending edge order. The box is approximate for curved edges: it is built from
//! the samples, so a bulge between two samples can lie slightly outside it (the diagonal, and so
//! the size, is then a hair small; the clamp bounds keep the result sane).

use sm_core::{EntityId, Point3};
use sm_scheme_api::geom;
use sm_topo_api::{Dim, TopologyView, TopologyViewExt};

/// Why [`auto_size`] failed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum SizingError {
    /// The factor is outside `1..=10`.
    #[error("auto size factor {factor} is outside 1..=10")]
    BadFactor {
        /// The rejected factor.
        factor: u32,
    },
    /// The selection holds no edge (or the model is empty).
    #[error("auto size: the selection has no edges")]
    NoGeometry,
    /// A target does not exist in the topology.
    #[error("auto size: {0} is not a live entity")]
    UnknownEntity(EntityId),
    /// Geometry could not be evaluated.
    #[error("auto size: {0}")]
    Geometry(String),
    /// The selected geometry is degenerate (zero extent or zero-length edge).
    #[error("auto size: degenerate geometry (diagonal {diagonal}, shortest edge {shortest})")]
    Degenerate {
        /// The bounding-box diagonal.
        diagonal: f64,
        /// The shortest edge length.
        shortest: f64,
    },
}

const SAMPLES: usize = 16;

/// A deterministic automatic element size for `targets`, non-decreasing in `factor` (1..=10);
/// see the [module docs](self) for the formula and the selection rules.
pub fn auto_size(
    view: &dyn TopologyView,
    targets: &[EntityId],
    factor: u32,
) -> Result<f64, SizingError> {
    if !(1..=10).contains(&factor) {
        return Err(SizingError::BadFactor { factor });
    }
    let mut edges: Vec<EntityId> = Vec::new();
    if targets.is_empty() {
        edges.extend_from_slice(view.cells(Dim::Edge));
    }
    for &t in targets {
        let dim = view.dim_of(t).filter(|_| view.is_alive(t)).ok_or(SizingError::UnknownEntity(t))?;
        match dim {
            Dim::Volume => {
                for &f in view.volume_faces(t) {
                    edges.extend_from_slice(view.face_edges(f));
                }
            }
            Dim::Face => edges.extend_from_slice(view.face_edges(t)),
            Dim::Edge => edges.push(t),
            _ => {}
        }
    }
    edges.sort_unstable();
    edges.dedup();
    edges.retain(|&e| view.is_alive(e));
    if edges.is_empty() {
        return Err(SizingError::NoGeometry);
    }
    let geo = |e: sm_scheme_api::SchemeError| SizingError::Geometry(e.to_string());
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    let mut grow = |p: Point3| {
        for (k, x) in [p.x, p.y, p.z].into_iter().enumerate() {
            lo[k] = lo[k].min(x);
            hi[k] = hi[k].max(x);
        }
    };
    let mut shortest = f64::INFINITY;
    for &e in &edges {
        shortest = shortest.min(geom::edge_length(view, e).map_err(geo)?);
        if let Some((a, b)) = view.edge_ends(e) {
            grow(geom::vertex_point(view, a).map_err(geo)?);
            grow(geom::vertex_point(view, b).map_err(geo)?);
        }
        for k in 1..SAMPLES {
            grow(geom::edge_point(view, e, k as f64 / SAMPLES as f64).map_err(geo)?);
        }
    }
    let diagonal = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt();
    if !(diagonal.is_finite() && diagonal > 0.0 && shortest.is_finite() && shortest > 0.0) {
        return Err(SizingError::Degenerate { diagonal, shortest });
    }
    let raw = 0.01 * diagonal * 2f64.powf((f64::from(factor) - 1.0) * 0.5);
    // A long closed or winding edge can have l_min/4 > d/2 (a full circle: 2*pi*r vs ~2.83*r);
    // f64::clamp would panic, so the floor l_min/4 wins.
    let lo = shortest / 4.0;
    let hi = (diagonal / 2.0).max(lo);
    Ok(raw.max(lo).min(hi))
}
