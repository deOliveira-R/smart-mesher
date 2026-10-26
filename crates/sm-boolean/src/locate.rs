//! Point location: is a point inside, on or outside a volume?
//!
//! [`locate_point`] is the reusable point-in-volume primitive (it serves ray tracing and
//! transport as well as the boolean classification). `On` is decided first, by the distance of
//! the point to every face of the volume. Otherwise a ray is cast along one of eight fixed
//! irrational directions and the signed crossings with the volume's faces are summed (a face
//! use contributes `+1` where the ray leaves the volume and `-1` where it enters). A direction
//! is ambiguous when the ray hits a face in a boundary position (edge or vertex), grazes a
//! surface, or the crossings do not add up to 0 or 1; the next direction is then tried. If all
//! eight are ambiguous the result is [`BooleanError::Ambiguous`].

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{Crossing, Interval, ModelTolerance, Ray, RayHit, Uv};
use sm_topo_api::{PointClass, Sense, TopologyView, TopologyViewExt};

use crate::error::BooleanError;
use crate::geo;

/// Where a point is relative to a volume.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum PointLocation {
    /// Strictly inside the volume.
    In,
    /// Within the model tolerance of the volume's boundary.
    On,
    /// Outside the volume.
    Out,
}

/// The eight probe directions, normalized at use. Components are irrational and mutually
/// unrelated so that no ray is parallel to an axis-aligned face or passes through the lattice
/// points where integer-coordinate geometry puts its edges.
const DIRECTIONS: [[f64; 3]; 8] = [
    [0.577_350_269_189_625_8, 0.707_106_781_186_547_5 + 0.131_578_947_368_421, 0.418_879_020_478_639_1],
    [-0.360_674_301_207_912_4, 0.822_474_487_139_158_9, 0.273_861_278_752_583_1],
    [0.264_575_131_106_459_1, -0.316_227_766_016_837_9, 0.912_870_929_175_276_8],
    [0.801_937_735_804_838_2, 0.230_940_107_675_850_3, -0.469_041_575_982_343_1],
    [-0.145_864_991_030_345_4, -0.755_928_946_018_454_6, 0.539_360_790_167_302_5],
    [0.935_414_346_693_485_6, -0.188_982_236_504_613_9, -0.118_322_414_258_213_5],
    [-0.627_647_368_240_493_8, 0.141_421_356_237_309_5, 0.693_889_411_277_062_3],
    [0.082_199_493_652_191_2, 0.968_245_836_551_854_2, -0.345_032_779_671_177_1],
];

fn direction(k: usize) -> Vector3 {
    let d = DIRECTIONS[k];
    let v = Vector3::new(d[0], d[1], d[2]);
    geo::unit(v).unwrap_or(Vector3::new(1.0, 0.0, 0.0))
}

/// One oriented boundary face of a region: the face and the side the region occupies
/// (`Forward`: the face normal points out of the region).
pub(crate) type Use = (EntityId, Sense);

/// All boundary uses of a volume (every shell).
pub(crate) fn volume_uses<V: TopologyView + ?Sized>(view: &V, volume: EntityId) -> Vec<Use> {
    view.face_uses(volume).map(|u| (u.face, u.sense)).collect()
}

/// Locate `p` against a region given by its boundary uses.
pub(crate) fn locate_in_uses<V: TopologyView + ?Sized>(view: &V, cache: &crate::faceclass::FaceCache, uses: &[Use], p: Point3, tol: &ModelTolerance) -> Result<PointLocation, BooleanError> {
    // 1. On: distance to every face.
    for &(f, _) in uses {
        if on_face(view, cache, f, p, tol.linear)? {
            return Ok(PointLocation::On);
        }
    }
    // 2. Ray casting.
    let mut bbox = sm_core::Aabb::from_point(p);
    for &(f, _) in uses {
        bbox = bbox.union(&geo::face_aabb(view, f, 0.0));
    }
    let diag = geo::dist(bbox.lo, bbox.hi);
    let reach = 2.0 * diag + 1.0;
    let mut last_ambiguous: Option<EntityId> = None;
    'dirs: for k in 0..DIRECTIONS.len() {
        let dir = direction(k);
        let ray = Ray { origin: p, dir };
        let mut sum = 0i64;
        for &(f, sense) in uses {
            let Some((surf, sc)) = geo::face_surface(view, f) else { return Err(BooleanError::Unsupported("a face of the volume has no surface")) };
            let mut hits: Vec<RayHit> = Vec::new();
            if surf.ray_hits_tol(&ray, Interval::new(0.0, reach), tol, &mut hits).is_err() {
                return Err(BooleanError::Unsupported("ray intersection is not available for a surface of the volume"));
            }
            for h in hits {
                let uv = match h.uv {
                    Some(uv) => uv,
                    None => match surf.project(h.point, None) {
                        Ok(pr) => pr.uv,
                        Err(_) => continue,
                    },
                };
                match crate::faceclass::classify_uv(view, cache, f, Uv::new(uv.u, uv.v), tol.linear)?.0 {
                    PointClass::Out => continue,
                    PointClass::On => {
                        last_ambiguous = Some(f);
                        continue 'dirs;
                    }
                    PointClass::In => {}
                }
                if h.crossing == Crossing::Tangent {
                    last_ambiguous = Some(f);
                    continue 'dirs;
                }
                // Face-oriented normal at the hit; the region's outward normal flips for `Reversed`.
                let n = match surf.normal(uv).ok().and_then(geo::unit) {
                    Some(n) => n * sc.sense.sign(),
                    None => {
                        last_ambiguous = Some(f);
                        continue 'dirs;
                    }
                };
                let outward = n * sense.sign();
                let c = dir.dot(outward);
                if c.abs() < 1e-9 {
                    last_ambiguous = Some(f);
                    continue 'dirs;
                }
                sum += if c > 0.0 { 1 } else { -1 };
            }
        }
        match sum {
            0 => return Ok(PointLocation::Out),
            1 => return Ok(PointLocation::In),
            _ => {
                last_ambiguous = last_ambiguous.or(uses.first().map(|u| u.0));
                continue;
            }
        }
    }
    let face = last_ambiguous.or_else(|| uses.first().map(|u| u.0));
    match face {
        Some(face) => Err(BooleanError::Ambiguous { face, point: p }),
        None => Ok(PointLocation::Out),
    }
}

/// True if `p` lies on face `f` within `tol` (on the surface and inside or on the boundary).
pub(crate) fn on_face<V: TopologyView + ?Sized>(view: &V, cache: &crate::faceclass::FaceCache, f: EntityId, p: Point3, tol: f64) -> Result<bool, BooleanError> {
    let Some((surf, _)) = geo::face_surface(view, f) else { return Ok(false) };
    let Ok(pr) = surf.project(p, None) else { return Ok(false) };
    if pr.distance > tol {
        return Ok(false);
    }
    Ok(matches!(crate::faceclass::classify_uv(view, cache, f, pr.uv, tol)?.0, PointClass::In | PointClass::On))
}

/// Where `p` is relative to `volume`: inside, on its boundary (within the model's linear
/// tolerance) or outside.
///
/// This is the point-in-volume primitive the booleans classify fragments with, public so that
/// ray tracing and transport can reuse it. It supports volumes with several shells (cavities)
/// and every surface whose `ParametricSurface::ray_hits_tol` is implemented (all analytic
/// kinds). `On` is decided first by the distance to the boundary faces; otherwise the signed
/// crossing count of a ray is used, with eight fixed irrational directions tried in order.
///
/// # Errors
/// [`BooleanError::Ambiguous`] if every direction hits an edge or vertex or grazes a surface
/// (perturb the point); [`BooleanError::Unsupported`] for a face without a surface or a surface
/// without ray support; `Query` if a face cannot be classified.
pub fn locate_point(view: &dyn TopologyView, volume: EntityId, p: Point3) -> Result<PointLocation, BooleanError> {
    if !view.is_alive(volume) || view.dim_of(volume) != Some(sm_topo_api::Dim::Volume) {
        return Err(BooleanError::Invalid(format!("{volume} is not a live volume")));
    }
    let uses = volume_uses(view, volume);
    let tol = *view.tolerance();
    locate_in_uses(view, &crate::faceclass::FaceCache::new(), &uses, p, &tol)
}
