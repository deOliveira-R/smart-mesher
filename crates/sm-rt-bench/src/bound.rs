//! The per-ray bound between a track on the exact model and a track on its facet twin.
//!
//! A facet twin is inscribed: at any point of a curved surface the facet plane lies inside it by
//! at most `sag` (the radial gap, recorded in the manifest and checked in `tests/cases.rs`).
//! A ray that crosses a curved surface at angle `theta` to the surface normal reaches the facet
//! plane `gap / cos(theta)` away from the exact crossing at most, so the crossing moves along
//! the ray by at most `sag / cos(theta)`:
//!
//! * vertical cylinder of radius `r`, ray with horizontal speed `rho = |d_xy|` at horizontal
//!   offset `b` from the axis: `cos(theta) = rho * sqrt(1 - b^2 / r^2)`;
//! * sphere of radius `r`, ray at offset `b` from the centre: `cos(theta) = sqrt(1 - b^2 / r^2)`.
//!
//! Every crossing of a curved surface adds its shift; a chord is bounded by the sum over all
//! crossings of the ray (a chord has two ends, the sum over the whole ray is a safe common
//! bound). Flat faces (planes, caps) are exact in both models. A ray whose `cos(theta)` is below
//! [`GRAZING_COS`] at some crossing is *grazing*: the twin may miss the chord altogether, so the
//! ray is counted and listed, not judged.

use crate::truth::{Scene, Shape, Track};

/// `cos(theta)` under which a crossing is grazing and the ray is not judged.
pub const GRAZING_COS: f64 = 0.05;

/// Bound of one ray.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayBound {
    /// Sum of the crossing shifts, cm.
    pub chord: f64,
    /// True when a crossing is below [`GRAZING_COS`].
    pub grazing: bool,
}

/// The bound of the ray `(o, d)` through `scene` for a twin of tolerance `sag`.
pub fn ray_bound(scene: &Scene, o: [f64; 3], d: [f64; 3], sag: f64) -> RayBound {
    let mut chord = 0.0;
    let mut grazing = false;
    for s in &scene.solids {
        let Some((t0, t1)) = s.interval(o, d) else { continue };
        let (cos, crossings) = match s.shape {
            Shape::Cyl { c, r, z0, z1 } => {
                let rho = d[0].hypot(d[1]);
                if rho < 1e-12 {
                    continue;
                }
                // Only the lateral surface is curved: count its crossings (not the caps).
                let at = |t: f64| [o[0] + t * d[0], o[1] + t * d[1], o[2] + t * d[2]];
                let on_side = |t: f64| {
                    let p = at(t);
                    let z_in = z0 + 1e-12 < p[2] && p[2] < z1 - 1e-12;
                    z_in && (((p[0] - c[0]).powi(2) + (p[1] - c[1]).powi(2)).sqrt() - r).abs() < 1e-9 * r.max(1.0)
                };
                let n = usize::from(on_side(t0)) + usize::from(on_side(t1));
                let b = ((o[0] - c[0]) * d[1] - (o[1] - c[1]) * d[0]).abs() / rho;
                (rho * (1.0 - (b / r).powi(2)).max(0.0).sqrt(), n)
            }
            Shape::Sphere { c, r } => {
                let q = [o[0] - c[0], o[1] - c[1], o[2] - c[2]];
                let cr = [q[1] * d[2] - q[2] * d[1], q[2] * d[0] - q[0] * d[2], q[0] * d[1] - q[1] * d[0]];
                let b = (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt();
                ((1.0 - (b / r).powi(2)).max(0.0).sqrt(), 2)
            }
            Shape::Box { .. } => continue,
        };
        if crossings == 0 {
            continue;
        }
        if cos < GRAZING_COS {
            grazing = true;
        }
        chord += crossings as f64 * sag / cos.max(1e-12);
    }
    RayBound { chord, grazing }
}

/// Outcome of [`compare_bounded`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BoundedReport {
    /// Rays judged.
    pub checked: usize,
    /// Rays skipped as grazing.
    pub grazing: usize,
    /// Judged rays whose region sequence differs or whose chord error exceeds its bound.
    pub failed: usize,
    /// Largest `chord error / bound` over judged chords.
    pub max_ratio: f64,
    /// First failing ray index.
    pub first_failed: Option<usize>,
}

/// Merge neighbours of equal region and drop chords shorter than `min`; returns the track and
/// the number of dropped chords.
fn shrink(t: &Track, min: f64) -> (Track, usize) {
    let mut out = Track { regions: Vec::new(), chords: Vec::new() };
    let mut dropped = 0;
    for (&r, &c) in t.regions.iter().zip(&t.chords) {
        if c < min {
            dropped += 1;
            continue;
        }
        if out.regions.last() == Some(&r) {
            if let Some(l) = out.chords.last_mut() {
                *l += c;
            }
        } else {
            out.regions.push(r);
            out.chords.push(c);
        }
    }
    (out, dropped)
}

/// Judge `twin` against `exact` ray by ray: the region sequences must be equal and every chord
/// must agree within `bound + 1e-9 * diag`; a ray that fails that may drop the chords that could
/// vanish in the twin (shorter than `2 * bound`, then `4 * bound`) from both sides, with the
/// tolerance widened to `bound * (1 + 2 * dropped) + 1e-9 * diag`.
pub fn compare_bounded(exact: &[Track], twin: &[Track], bounds: &[RayBound], diag: f64) -> BoundedReport {
    let mut rep = BoundedReport::default();
    for (i, ((a, b), rb)) in exact.iter().zip(twin).zip(bounds).enumerate() {
        if rb.grazing {
            rep.grazing += 1;
            continue;
        }
        rep.checked += 1;
        let direct = Track { regions: a.regions.clone(), chords: a.chords.clone() };
        let mut ok = false;
        // First without dropping anything: equal sequences, every chord within the bound. Only if
        // that fails may chords that could vanish in the twin (shorter than `2 * bound`, then
        // `4 * bound`) be dropped on both sides, with the tolerance widened by what was dropped.
        for (min, widen) in [(0.0, false), (2.0 * rb.chord, true), (4.0 * rb.chord, true)] {
            let (sa, da) = if widen { shrink(a, min) } else { (direct.clone(), 0) };
            let (sb, db) = if widen { shrink(b, min) } else { (Track { regions: b.regions.clone(), chords: b.chords.clone() }, 0) };
            let tol = rb.chord * (1.0 + 2.0 * (da + db) as f64) + 1e-9 * diag;
            if sa.regions != sb.regions {
                continue;
            }
            let worst = sa.chords.iter().zip(&sb.chords).map(|(x, y)| (x - y).abs() / tol).fold(0.0, f64::max);
            if worst <= 1.0 {
                rep.max_ratio = rep.max_ratio.max(worst);
                ok = true;
                break;
            }
        }
        if !ok {
            rep.failed += 1;
            rep.first_failed.get_or_insert(i);
        }
    }
    rep
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::truth::Solid;

    fn cyl_scene() -> Scene {
        Scene::new(vec![Solid { shape: Shape::Cyl { c: [0.0, 0.0], r: 1.0, z0: 0.0, z1: 10.0 }, region: 0, depth: 0 }])
    }

    #[test]
    fn a_horizontal_diameter_has_the_plain_sag_bound_per_crossing() {
        let b = ray_bound(&cyl_scene(), [-2.0, 0.0, 5.0], [1.0, 0.0, 0.0], 0.01);
        assert!((b.chord - 0.02).abs() < 1e-15 && !b.grazing, "{b:?}");
    }

    #[test]
    fn a_tangent_line_is_grazing_and_a_vertical_ray_is_not_bounded() {
        assert!(ray_bound(&cyl_scene(), [-2.0, 0.9999, 5.0], [1.0, 0.0, 0.0], 0.01).grazing);
        assert_eq!(ray_bound(&cyl_scene(), [0.0, 0.0, -1.0], [0.0, 0.0, 1.0], 0.01).chord, 0.0);
    }

    #[test]
    fn dropped_chords_widen_the_tolerance_and_a_large_error_fails() {
        let t = |r: &[i32], c: &[f64]| Track { regions: r.to_vec(), chords: c.to_vec() };
        let rb = [RayBound { chord: 0.01, grazing: false }];
        let exact = [t(&[0, 1, 0], &[1.0, 0.005, 1.0])];
        let twin = [t(&[0], &[2.005])];
        let rep = compare_bounded(&exact, &twin, &rb, 10.0);
        assert_eq!((rep.checked, rep.failed), (1, 0), "{rep:?}");
        let off = [t(&[0], &[2.5])];
        assert_eq!(compare_bounded(&exact, &off, &rb, 10.0).failed, 1);
    }
}
