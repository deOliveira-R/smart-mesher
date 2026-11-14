//! `model.subdivide_pin`: cut solid cylinders and coaxial annuli into radial rings and azimuthal
//! sectors, and record which piece is which in the model's region tags (ruling D9).
//!
//! The op is a fixed sequence of webcuts inside one transaction: rings first (outermost cut
//! first), then sectors (planes through the axis, one per pair of opposite rays). The order is
//! fixed so the minted ids are deterministic. Ring indices come from the cut sequence; sector
//! indices come from the angle of each piece's centroid, which lies strictly inside its wedge.

use std::f64::consts::TAU;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_geom_api::{Frame, SurfaceDef};
use sm_layer_api::{LayerResultExt, ModelLayer, commit_topology, model_txn, topo_txn};
use sm_model_api::SubIndex;
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, TypedOp};
use sm_topo_api::{Dim, TopologyView};

use crate::bodies;
use crate::pieces::surface_def_of;
use crate::webcut::{CutTool, WebcutOptions, webcut};

/// Azimuth (radians) of the seam of the cutting cylinders relative to the frame x axis.
///
/// The W4-REGIONS-CUT finding (a 0.475 cylinder cut at 0.418 failed with the default frame) was
/// not a seam coincidence: `faceclass::interior_point` could return a point on the wrong side of
/// an inscribed boundary polygon (fixed in W4-BOOL-TOOLS; see `tests/bool_tools.rs`). The turn is
/// kept because it moves the seam off the sector cuts (a seam vertex inside a sector arc would
/// force a non-uniform node spacing, see `sm-server/tests/regions_split.rs`) and because changing
/// it moves the hashes of every subdivided pin.
const SEAM_TURN: f64 = 1.0;

/// Largest ring or sector count per pin.
pub const MAX_SUBDIVISION: u32 = 1024;

const OUTPUTS: &[OutputRole] = &[
    OutputRole::new("volumes", &[EntityKind::TopoVolume], Cardinality::Many, "The pieces (read their ring and sector from the region tags)."),
    OutputRole::new("bodies", &[EntityKind::TopoBody], Cardinality::Many, "The bodies of the pieces, one per piece."),
];

/// How `rings.count` places the interior radii.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum RingMode {
    /// Every ring has the same area (`r_k^2` equally spaced).
    #[default]
    EqualArea,
    /// Every ring has the same radial width.
    EqualRadius,
}

/// `{count, mode?}`: `count` rings placed by `mode`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RingCount {
    /// Number of rings (1 to 1024); 1 cuts nothing.
    pub count: u32,
    /// Placement (default `equal_area`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mode: Option<RingMode>,
}

/// `{radii}`: the interior cut radii, strictly increasing, inside `(r_in, r_out)` of every volume.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RingRadii {
    /// The cut radii; `n` radii give `n + 1` rings.
    pub radii: Vec<f64>,
}

/// The ways to specify the rings of `model.subdivide_pin`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum RingSpec {
    /// A count and a placement mode.
    Count(RingCount),
    /// Explicit radii.
    Radii(RingRadii),
}

/// `{count, angle0_deg?}`: azimuthal sectors.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SectorSpec {
    /// Number of sectors: 1 (no cut) or even (a plane cuts both sides of the axis).
    pub count: u32,
    /// Start angle of sector 0 in degrees (default 0), counter-clockwise about `axis`, measured
    /// from the x axis of `Frame::from_z(center, axis)` (the world x axis for a z axis).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub angle0_deg: Option<f64>,
}

/// `model.subdivide_pin {volumes, axis, center, rings?, sectors?}`: cut each solid cylinder or
/// coaxial annulus into rings (0 innermost) and sectors (0 starting at `angle0`), and write the
/// `ring` and `sector` of each piece's region tag. Lattice slots of a tagged volume stay on every
/// piece. At least one of `rings` and `sectors` is required; a count of 1 cuts nothing and tags
/// the volume `{index 0, count 1}`.
///
/// The op is atomic. It fails with `Invalid` for bad counts or radii or a volume that is not a
/// coaxial pin, `Unsupported` for a volume that already carries a ring or sector tag, and
/// `Failed` when the kernel rejects one of the underlying cuts (this can happen for some radius
/// pairs); a failure leaves the state hash unchanged.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SubdividePin {
    /// The pins to cut.
    #[schemars(extend("x-entity-kind" = ["TopoVolume"]))]
    pub volumes: Vec<EntityId>,
    /// Axis direction (need not be unit).
    pub axis: [f64; 3],
    /// A point on the axis.
    pub center: [f64; 3],
    /// Radial subdivision.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rings: Option<RingSpec>,
    /// Azimuthal subdivision.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sectors: Option<SectorSpec>,
}

fn finite3(name: &str, v: [f64; 3]) -> Result<Vector3, OpError> {
    if v.iter().all(|c| c.is_finite()) { Ok(Vector3::new(v[0], v[1], v[2])) } else { Err(OpError::Invalid(format!("`{name}` must be finite, got {v:?}"))) }
}

fn check_count(what: &str, n: u32) -> Result<(), OpError> {
    if (1..=MAX_SUBDIVISION).contains(&n) { Ok(()) } else { Err(OpError::Invalid(format!("`{what}.count` must be in 1..={MAX_SUBDIVISION}, got {n}"))) }
}

/// `(r_in, r_out)` of a solid (`r_in = 0`) or annular pin; an error unless the volume is one
/// coaxial with the given axis.
fn pin_radii(topo: &dyn TopologyView, v: EntityId, c: Point3, a: Vector3, tol: f64) -> Result<(f64, f64), OpError> {
    let bad = |why: &str| OpError::Invalid(format!("{v} is not a coaxial cylinder or annulus about the given axis: {why}"));
    let mut radii: Vec<f64> = Vec::new();
    for &f in topo.volume_faces(v) {
        match surface_def_of(topo, f) {
            Some(SurfaceDef::Plane(p)) => {
                if p.frame.z.dot(a).abs() < 1.0 - 1e-9 {
                    return Err(bad("a face is a plane not perpendicular to the axis"));
                }
            }
            Some(SurfaceDef::Cylinder(cy)) => {
                if cy.frame.z.cross(a).norm() > 1e-9 || (cy.frame.origin - c).cross(a).norm() > tol {
                    return Err(bad("a cylindrical face has another axis"));
                }
                if !radii.iter().any(|r| (r - cy.radius).abs() <= tol) {
                    radii.push(cy.radius);
                }
            }
            _ => return Err(bad("a face is neither a plane nor a cylinder")),
        }
    }
    radii.sort_by(f64::total_cmp);
    match radii.as_slice() {
        [r] => Ok((0.0, *r)),
        [ri, ro] => Ok((*ri, *ro)),
        _ => Err(bad(&format!("it has {} distinct cylinder radii, expected 1 or 2", radii.len()))),
    }
}

/// Interior cut radii (ascending) of one pin.
fn ring_radii(spec: &RingSpec, r_in: f64, r_out: f64, tol: f64, v: EntityId) -> Result<Vec<f64>, OpError> {
    let radii: Vec<f64> = match spec {
        RingSpec::Count(rc) => {
            check_count("rings", rc.count)?;
            let n = f64::from(rc.count);
            (1..rc.count)
                .map(|k| {
                    let k = f64::from(k);
                    match rc.mode.unwrap_or_default() {
                        RingMode::EqualArea => (r_in * r_in + k * (r_out * r_out - r_in * r_in) / n).sqrt(),
                        RingMode::EqualRadius => r_in + k * (r_out - r_in) / n,
                    }
                })
                .collect()
        }
        RingSpec::Radii(rr) => {
            if rr.radii.is_empty() {
                return Err(OpError::Invalid("`rings.radii` must not be empty".into()));
            }
            if rr.radii.len() >= MAX_SUBDIVISION as usize {
                return Err(OpError::Invalid(format!("`rings.radii` has more than {} entries", MAX_SUBDIVISION - 1)));
            }
            rr.radii.clone()
        }
    };
    let mut prev = r_in;
    for &r in &radii {
        if !r.is_finite() || r <= prev + tol {
            return Err(OpError::Invalid(format!("ring radius {r} for {v} must increase strictly and exceed the inner radius {r_in} (previous {prev}, tolerance {tol:e})")));
        }
        prev = r;
    }
    if let Some(&last) = radii.last()
        && last >= r_out - tol
    {
        return Err(OpError::Invalid(format!("ring radius {last} for {v} is not inside (r_in, r_out) = ({r_in}, {r_out})")));
    }
    Ok(radii)
}

/// A piece: its volume, ring and sector.
type Piece = (EntityId, Option<SubIndex>, Option<SubIndex>);

fn one_volume(topo: &dyn TopologyView, found: &[EntityId], what: &str) -> Result<EntityId, OpError> {
    let vs = bodies::volumes_of(topo, found).map_err(OpError::from)?;
    match vs.as_slice() {
        [v] => Ok(*v),
        _ => Err(OpError::Failed(format!("subdivide_pin: the {what} cut gave {} pieces, expected 1", vs.len()))),
    }
}

/// Cut one pin; returns its pieces in (ring, sector) order.
#[allow(clippy::too_many_arguments)]
fn cut_pin(
    topo: &mut dyn sm_layer_api::TopoHost,
    v: EntityId,
    radii: &[f64],
    frame: &Frame,
    rings_requested: bool,
    sector_n: Option<u32>,
    angle0: f64,
) -> Result<Vec<Piece>, OpError> {
    let opts = WebcutOptions::default();
    let n_rings = radii.len() + 1;
    let mut ring_vols: Vec<EntityId> = vec![v; n_rings];
    let mut cur = v;
    // Outermost cut first: the outside of each cut is ring i + 1.
    for i in (0..radii.len()).rev() {
        // The tool frame's x reference is turned by `SEAM_TURN`: with the default frame, cutting
        // a 0.475 cylinder at 0.418 (and at 0.476) fails validation; the cause is not established
        // (see the W4-REGIONS-CUT report), the turned frame passed every case tried.
        let tool_frame = Frame::from_zx(frame.origin, frame.z, frame.x * SEAM_TURN.cos() + frame.y * SEAM_TURN.sin()).map_err(|e| OpError::Invalid(e.to_string()))?;
        let cyl = sm_geom_api::Cylinder::new(tool_frame, radii[i]).map_err(|e| OpError::Invalid(e.to_string()))?;
        let tool = CutTool::Surface { surface: SurfaceDef::Cylinder(cyl), window: None };
        let out = webcut(topo, &[cur], &tool, &opts).map_err(OpError::from)?;
        ring_vols[i + 1] = one_volume(&*topo, &out.positive, "ring")?;
        cur = one_volume(&*topo, &out.negative, "ring")?;
    }
    ring_vols[0] = cur;
    let mut pieces: Vec<Piece> = Vec::new();
    for (ri, &rv) in ring_vols.iter().enumerate() {
        let ring = rings_requested.then(|| SubIndex::new(ri as u32, n_rings as u32));
        let n_sec = sector_n.unwrap_or(0);
        if n_sec <= 1 {
            pieces.push((rv, ring, sector_n.map(|n| SubIndex::new(0, n))));
            continue;
        }
        let mut pcs = vec![rv];
        for j in 0..n_sec / 2 {
            let phi = angle0 + f64::from(j) * TAU / f64::from(n_sec);
            let d = frame.x * phi.cos() + frame.y * phi.sin();
            let tool = CutTool::Plane { origin: frame.origin, normal: frame.z.cross(d) };
            let out = webcut(topo, &pcs, &tool, &opts).map_err(OpError::from)?;
            let mut all = out.positive.clone();
            all.extend_from_slice(&out.negative);
            pcs = bodies::volumes_of(&*topo, &all).map_err(OpError::from)?;
        }
        if pcs.len() != n_sec as usize {
            return Err(OpError::Failed(format!("subdivide_pin: {v} gave {} sectors, expected {n_sec}", pcs.len())));
        }
        let mut by_sector: Vec<Option<EntityId>> = vec![None; n_sec as usize];
        for &p in &pcs {
            let m = topo.volume_moments(p).map_err(|e| OpError::Failed(e.to_string()))?;
            let cen = m.centroid().ok_or_else(|| OpError::Failed(format!("subdivide_pin: piece {p} has no volume")))?;
            let l = frame.to_local(cen);
            let frac = (l.y.atan2(l.x) - angle0).rem_euclid(TAU) / TAU * f64::from(n_sec);
            let s = (frac.floor() as usize).min(n_sec as usize - 1);
            if by_sector[s].replace(p).is_some() {
                return Err(OpError::Failed(format!("subdivide_pin: two pieces of {v} fall in sector {s}")));
            }
        }
        for (s, p) in by_sector.into_iter().enumerate() {
            let p = p.ok_or_else(|| OpError::Failed(format!("subdivide_pin: no piece of {v} in sector {s}")))?;
            pieces.push((p, ring, Some(SubIndex::new(s as u32, n_sec))));
        }
    }
    Ok(pieces)
}

impl TypedOp for SubdividePin {
    const NAME: &'static str = "model.subdivide_pin";
    const OUTPUTS: &'static [OutputRole] = OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if self.volumes.is_empty() {
            return Err(OpError::Invalid("`volumes` must not be empty".into()));
        }
        if self.rings.is_none() && self.sectors.is_none() {
            return Err(OpError::Invalid("give `rings`, `sectors` or both".into()));
        }
        let axis = finite3("axis", self.axis)?;
        let c = finite3("center", self.center)?;
        if axis.norm().is_nan() || axis.norm() <= 0.0 {
            return Err(OpError::Invalid("`axis` must not be zero".into()));
        }
        let a = axis * (1.0 / axis.norm());
        let center = Point3::new(c.x, c.y, c.z);
        let frame = Frame::from_z(center, a).map_err(|e| OpError::Invalid(e.to_string()))?;
        let mut sector_n = None;
        let mut angle0 = 0.0;
        if let Some(s) = &self.sectors {
            check_count("sectors", s.count)?;
            if s.count > 1 && s.count % 2 != 0 {
                return Err(OpError::Invalid(format!("`sectors.count` must be 1 or even (a plane cuts both sides of the axis), got {}", s.count)));
            }
            let deg = s.angle0_deg.unwrap_or(0.0);
            if !deg.is_finite() {
                return Err(OpError::Invalid("`sectors.angle0_deg` must be finite".into()));
            }
            angle0 = deg.to_radians();
            sector_n = Some(s.count);
        }
        let mut vols: Vec<EntityId> = Vec::new();
        for &v in &self.volumes {
            ctx.resolve_kind(v, EntityKind::TopoVolume)?;
            if !vols.contains(&v) {
                vols.push(v);
            }
        }
        crate::ops::guard_volumes(ctx, &vols)?;
        // Validate every pin and fix its cut radii before anything is edited.
        let mut plan: Vec<(EntityId, Vec<f64>)> = Vec::new();
        {
            let layer = ctx.layer::<ModelLayer>()?;
            let topo = layer.host();
            let tol = layer.tolerance().linear.max(1e-9);
            for &v in &vols {
                if let Some(t) = layer.model().region_tag(v)
                    && ((self.rings.is_some() && t.ring.is_some()) || (self.sectors.is_some() && t.sector.is_some()))
                {
                    return Err(OpError::Unsupported(format!("{v} already carries a ring or sector tag: it is a piece of a subdivided pin")));
                }
                let (r_in, r_out) = pin_radii(topo, v, center, a, tol)?;
                let radii = match &self.rings {
                    Some(spec) => ring_radii(spec, r_in, r_out, tol, v)?,
                    None => Vec::new(),
                };
                plan.push((v, radii));
            }
        }
        let rings_requested = self.rings.is_some();
        let ((pieces, body_ids), txn) = topo_txn(ctx, |topo, _model| {
            let mut pieces: Vec<Piece> = Vec::new();
            for (v, radii) in &plan {
                pieces.extend(cut_pin(topo, *v, radii, &frame, rings_requested, sector_n, angle0)?);
            }
            let mut body_ids: Vec<EntityId> = Vec::new();
            for (p, _, _) in &pieces {
                let body = topo.coboundary(*p).iter().map(|i| i.cell).find(|c| topo.dim_of(*c) == Some(Dim::Body));
                if let Some(b) = body {
                    body_ids.push(b);
                }
            }
            Ok((pieces, body_ids))
        })?;
        let volume_ids: Vec<EntityId> = pieces.iter().map(|p| p.0).collect();
        commit_topology(ctx, &txn, &[("volumes", &volume_ids), ("bodies", &body_ids)])?;
        // Lineage has carried the source tags (lattice slots) to every piece; add ring and sector.
        model_txn(
            ctx,
            |m, _t| {
                for (v, ring, sector) in &pieces {
                    let mut tag = m.region_tag(*v).cloned().unwrap_or_default();
                    if ring.is_some() {
                        tag.ring = *ring;
                    }
                    if sector.is_some() {
                        tag.sector = *sector;
                    }
                    m.set_region_tag(*v, tag).or_op()?;
                }
                Ok(())
            },
            |_, _| Vec::new(),
        )
    }
}
