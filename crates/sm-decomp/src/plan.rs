//! The planner core on plain data.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use std::f64::consts::TAU;

type V3 = [f64; 3];

fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}
fn axpy(a: V3, s: f64, b: V3) -> V3 {
    [a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2]]
}
/// Add 0.0 so that `-0.0` prints as `0.0` (stable text and hashes).
fn clean(a: V3) -> V3 {
    [a[0] + 0.0, a[1] + 0.0, a[2] + 0.0]
}

/// The surface class of one face of a [`VolumeView`].
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum FaceKind {
    /// A plane through `origin`; `normal` is a unit vector (either orientation).
    Plane {
        /// A point of the plane.
        origin: [f64; 3],
        /// Unit normal.
        normal: [f64; 3],
    },
    /// A cylinder about the line through `origin` along the unit vector `axis`.
    Cylinder {
        /// A point of the axis.
        origin: [f64; 3],
        /// Unit axis direction.
        axis: [f64; 3],
        /// Radius.
        radius: f64,
        /// True when the volume's outward normal on this face points away from the axis (a
        /// convex outer wall); false for a cavity wall (the volume surrounds the axis).
        outward: bool,
    },
    /// Any other surface (cone, sphere, torus, NURBS, a baffle): not decomposable here.
    Other,
}

/// One face of a [`VolumeView`].
#[derive(Debug, Clone, PartialEq)]
pub struct FaceView {
    /// The face id (for messages).
    pub id: EntityId,
    /// Its surface.
    pub kind: FaceKind,
    /// Positions of the face's vertices (all of them; order does not matter).
    pub points: Vec<[f64; 3]>,
    /// The face area, when known (checks that a cavity wall is a whole cylinder).
    pub area: Option<f64>,
}

/// What the planner needs to know about a volume.
#[derive(Debug, Clone, PartialEq)]
pub struct VolumeView {
    /// The volume.
    pub id: EntityId,
    /// Its faces (any order; the plan does not depend on it).
    pub faces: Vec<FaceView>,
}

/// Which output of an earlier step a later step cuts (the `WEBCUT_OUTPUTS` roles of `sm-boolean`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum Role {
    /// The pieces on the side the tool normal points to (the outside of a cylinder).
    Positive,
    /// The pieces on the other side.
    Negative,
}

impl Role {
    /// The role name as the ops spell it.
    pub fn as_str(self) -> &'static str {
        match self {
            Role::Positive => "positive",
            Role::Negative => "negative",
        }
    }
}

/// A cut target: an existing id, or an output of an earlier step.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum Target {
    /// An id that exists before the plan runs (the planned volume).
    Id {
        /// The id.
        id: EntityId,
    },
    /// The pieces of `role` produced by step `step` (an earlier step).
    Step {
        /// Index of the earlier step.
        step: usize,
        /// Which of its outputs.
        role: Role,
    },
}

/// The tool of a step: the arguments of one existing op.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "op")]
pub enum CutKind {
    /// `model.webcut_cylinder {radius, axis, center}`; the positive side is the outside.
    #[serde(rename = "model.webcut_cylinder")]
    Cylinder {
        /// Radius.
        radius: f64,
        /// Axis direction (unit).
        axis: [f64; 3],
        /// A point on the axis.
        center: [f64; 3],
    },
    /// `model.webcut_plane {plane: {origin, normal}}`; the positive side is where `normal` points.
    #[serde(rename = "model.webcut_plane")]
    Plane {
        /// The plane as `{origin, normal}`.
        plane: PlaneArg,
    },
}

/// A plane in the `{origin, normal}` form of `model.webcut_plane`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct PlaneArg {
    /// A point of the plane.
    pub origin: [f64; 3],
    /// Normal (unit).
    pub normal: [f64; 3],
}

/// One planned cut: apply `cut` to `targets`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct CutStep {
    /// Position in the plan (0-based); later steps may refer to it.
    pub step: usize,
    /// The op and its arguments.
    #[serde(flatten)]
    pub cut: CutKind,
    /// What to cut.
    pub targets: Vec<Target>,
}

/// Why no plan was made.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum DecompError {
    /// The id is not a live volume.
    #[error("{0} is not a live volume")]
    NotAVolume(EntityId),
    /// The geometry is outside what the planner handles; nothing was cut.
    #[error("no decomposition found for {volume}: {reason}")]
    NoDecomposition {
        /// The volume.
        volume: EntityId,
        /// What is unsupported.
        reason: String,
    },
}

/// One cavity: the whole-cylinder wall faces on one circle.
struct Cavity {
    center: V3,
    radius: f64,
    lo: f64,
    hi: f64,
    ends: Vec<f64>,
    /// Index of the first run on the same circle: runs of one circle share it.
    circle: usize,
}

fn span(points: &[V3], d: V3) -> Option<(f64, f64)> {
    let mut it = points.iter().map(|&p| dot(p, d));
    let first = it.next()?;
    Some(it.fold((first, first), |(a, b), s| (a.min(s), b.max(s))))
}

/// Plan the cuts that make `volume` sweepable along its cavity axis.
///
/// `tol` is the model length tolerance. Returns `Ok(vec![])` when nothing needs cutting (no cavity
/// wall, or every cavity already runs through the volume with a one-piece wall: the volume is
/// sweepable or fails for a reason cuts cannot fix). The order is fixed: cylinders by (centre,
/// radius), then planes ascending along the axis; each step cuts the `positive` output of the
/// previous one, so the plan does not depend on the face order of the view.
///
/// # Errors
/// [`DecompError::NoDecomposition`] with the reason when the volume has cavities with different
/// axes, overlapping or nested cavities (stepped bores), partial cylinder walls, or faces that are
/// neither planes nor cylinders parallel to the axis, or with stacked coaxial cavities whose solid
/// disc between them the kernel cannot cut out (for example blind holes from both faces).
pub fn plan_cuts_view(v: &VolumeView, tol: f64) -> Result<Vec<CutStep>, DecompError> {
    let no = |reason: String| DecompError::NoDecomposition {
        volume: v.id,
        reason,
    };
    // Cavity walls fix the axis.
    let mut axis: Option<V3> = None;
    for f in &v.faces {
        if let FaceKind::Cylinder {
            axis: a,
            outward: false,
            ..
        } = &f.kind
        {
            let a = *a;
            match axis {
                None => {
                    // The sign convention: the first non-negligible component is positive.
                    let s = a
                        .iter()
                        .find(|c| c.abs() > 1e-12)
                        .map_or(1.0, |c| c.signum());
                    axis = Some(clean([a[0] * s, a[1] * s, a[2] * s]));
                }
                Some(d) => {
                    if norm(cross(d, a)) > 1e-9 {
                        return Err(no(format!(
                            "cavity {} has a different axis from the other cavities ({a:?} against {d:?})",
                            f.id
                        )));
                    }
                }
            }
        }
    }
    let Some(d) = axis else { return Ok(Vec::new()) };

    // Extent along the axis, and the classification of every face.
    let all: Vec<V3> = v
        .faces
        .iter()
        .flat_map(|f| f.points.iter().copied())
        .collect();
    let Some((lo, hi)) = span(&all, d) else {
        return Err(no("the volume has no vertices".into()));
    };
    if hi - lo <= tol {
        return Err(no("the volume is flat along its cavity axis".into()));
    }
    let mut walls: Vec<(&FaceView, V3, f64, f64, f64)> = Vec::new(); // face, centre, radius, a, b
    let mut interior_planes: Vec<(&FaceView, f64)> = Vec::new();
    for f in &v.faces {
        match &f.kind {
            FaceKind::Other => {
                return Err(no(format!("face {} is not a plane or a cylinder", f.id)));
            }
            FaceKind::Plane { origin, normal } => {
                let c = dot(*normal, d).abs();
                if c > 1.0 - 1e-9 {
                    let s = dot(*origin, d);
                    if s > lo + tol && s < hi - tol {
                        interior_planes.push((f, s));
                    }
                } else if c > 1e-9 {
                    return Err(no(format!(
                        "face {} is a plane that is neither parallel nor normal to the cavity axis",
                        f.id
                    )));
                }
            }
            FaceKind::Cylinder {
                origin,
                axis: a,
                radius,
                outward,
            } => {
                if norm(cross(d, *a)) > 1e-9 {
                    return Err(no(format!(
                        "cylinder {} is not parallel to the cavity axis",
                        f.id
                    )));
                }
                if *outward {
                    continue;
                }
                let (a0, b0) = span(&f.points, d)
                    .ok_or_else(|| no(format!("cavity wall {} has no vertices", f.id)))?;
                if let Some(area) = f.area {
                    let whole = TAU * radius * (b0 - a0);
                    if (area.abs() - whole).abs() > 1e-6 * whole {
                        return Err(no(format!(
                            "cavity wall {} is not a whole cylinder (area {} against {whole}): a slot or a split wall",
                            f.id,
                            area.abs()
                        )));
                    }
                }
                let c = clean(axpy(*origin, -dot(*origin, d), d));
                walls.push((f, c, *radius, a0, b0));
            }
        }
    }

    // Group the walls into cavities (same circle), in a fixed order.
    walls.sort_by(|x, y| {
        let key = |w: &(&FaceView, V3, f64, f64, f64)| [w.1[0], w.1[1], w.1[2], w.2, w.3, w.4];
        let (a, b) = (key(x), key(y));
        a.iter()
            .zip(&b)
            .map(|(p, q)| p.total_cmp(q))
            .find(|o| o.is_ne())
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    // Contiguous walls on one circle form a run (one cavity); a gap larger than tol starts a new run.
    let mut cavities: Vec<Cavity> = Vec::new();
    for (_, c, r, a, b) in &walls {
        let same =
            |k: &Cavity| (k.radius - r).abs() <= tol && norm(axpy(k.center, -1.0, *c)) <= tol;
        match cavities.iter_mut().rev().find(|k| same(k) && *a <= k.hi + tol) {
            Some(k) => {
                k.lo = k.lo.min(*a);
                k.hi = k.hi.max(*b);
                k.ends.extend([*a, *b]);
            }
            None => {
                let circle = cavities.iter().position(same).unwrap_or(cavities.len());
                cavities.push(Cavity {
                    center: *c,
                    radius: *r,
                    lo: *a,
                    hi: *b,
                    ends: vec![*a, *b],
                    circle,
                });
            }
        }
    }
    // Disjoint footprints only: a nested or overlapping pair is a stepped bore.
    for (i, p) in cavities.iter().enumerate() {
        for q in &cavities[i + 1..] {
            if p.circle == q.circle {
                continue; // runs of one circle are stacked, not nested
            }
            if norm(axpy(p.center, -1.0, q.center)) < p.radius + q.radius + tol {
                return Err(no(format!(
                    "cavities of radius {} and {} overlap or nest (stepped or concentric bore)",
                    p.radius, q.radius
                )));
            }
        }
    }
    // An interior plane must close a cavity: at one of its ends and inside its circle.
    for (f, s) in &interior_planes {
        let closes = cavities.iter().any(|k| {
            k.ends.iter().any(|e| (e - s).abs() <= tol)
                && f.points
                    .iter()
                    .all(|p| norm(axpy(axpy(*p, -dot(*p, d), d), -1.0, k.center)) <= k.radius + tol)
        });
        if !closes {
            return Err(no(format!(
                "face {} is a plane inside the volume at {s} that does not close a round cavity (stepped geometry)",
                f.id
            )));
        }
    }

    // The plan. A cavity closed at both ends gets a cylinder cut first (it separates the caps from the
    // ring). Then a plane at every axial coordinate where a cavity wall starts or ends inside the
    // host, ascending. A blind cavity (open at one end) gets its cylinder right after its end plane,
    // on the piece beyond the cavity: the kernel refuses a cylinder cut that does not separate the
    // volume, and the solid slab over a filled cavity needs the disc cut out of it. A through cavity
    // needs nothing.
    let tol_eq = |a: f64, b: f64| (a - b).abs() <= tol;
    let enclosed = |k: &Cavity| k.lo > lo + tol && k.hi < hi - tol;
    let mut positions: Vec<f64> = cavities.iter().flat_map(|k| k.ends.iter().copied()).filter(|&s| s > lo + tol && s < hi - tol).collect();
    positions.sort_by(f64::total_cmp);
    positions.dedup_by(|b, a| (*b - *a).abs() <= tol);

    // Several runs on one circle leave solid discs between them. The kernel cuts a disc out only
    // when the cut face has a free edge, so the supported stacks are: enclosed runs only (one
    // cylinder over the whole host), or a blind-from-below run followed by enclosed runs (plane at
    // the blind end, then one cylinder on the slab above it). Anything else is refused.
    let runs_on = |c: usize| cavities.iter().filter(move |k| k.circle == c);
    let blind_low = |k: &Cavity| k.lo <= lo + tol && k.hi < hi - tol;
    let mut stacked_blind: Vec<usize> = Vec::new(); // circles planned from their blind-low end
    for k in &cavities {
        if runs_on(k.circle).count() < 2 || runs_on(k.circle).all(&enclosed) {
            continue;
        }
        let first = runs_on(k.circle).next().expect("a run");
        if blind_low(first) && runs_on(k.circle).skip(1).all(&enclosed) {
            if !stacked_blind.contains(&k.circle) {
                stacked_blind.push(k.circle);
            }
        } else {
            return Err(no(format!(
                "cavities of radius {} are stacked on one axis with a blind end that is not the lowest run: the solid disc between them cannot be cut out",
                k.radius
            )));
        }
    }
    let mut steps: Vec<CutStep> = Vec::new();
    let mut prev = Target::Id { id: v.id };
    fn add(steps: &mut Vec<CutStep>, cut: CutKind, target: Target) -> usize {
        let step = steps.len();
        steps.push(CutStep { step, cut, targets: vec![target] });
        step
    }
    let cylinder = |k: &Cavity| CutKind::Cylinder { radius: k.radius, axis: d, center: k.center };
    // One cylinder cut per circle: it already separates every disc stacked on that circle.
    let mut cut: Vec<usize> = Vec::new();
    for k in cavities.iter().filter(|k| enclosed(k) && !stacked_blind.contains(&k.circle)) {
        if cut.contains(&k.circle) {
            continue;
        }
        cut.push(k.circle);
        let i = add(&mut steps, cylinder(k), prev);
        prev = Target::Step { step: i, role: Role::Positive };
    }
    for s in positions {
        let plane = CutKind::Plane { plane: PlaneArg { origin: clean([d[0] * s, d[1] * s, d[2] * s]), normal: d } };
        let p = add(&mut steps, plane, prev);
        prev = Target::Step { step: p, role: Role::Positive };
        for k in &cavities {
            if cut.contains(&k.circle) {
                continue;
            }
            if k.lo <= lo + tol && k.hi < hi - tol && tol_eq(k.hi, s) {
                // Blind from the low end: the solid slab above the cavity.
                cut.push(k.circle);
                let i = add(&mut steps, cylinder(k), Target::Step { step: p, role: Role::Positive });
                prev = Target::Step { step: i, role: Role::Positive };
            } else if k.hi >= hi - tol && k.lo > lo + tol && tol_eq(k.lo, s) {
                // Blind from the high end: the solid slab under the cavity.
                cut.push(k.circle);
                add(&mut steps, cylinder(k), Target::Step { step: p, role: Role::Negative });
            }
        }
    }
    Ok(steps)
}

