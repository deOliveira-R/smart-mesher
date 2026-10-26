//! Safe conversions between the contract's types and the `#[repr(C)]` views.

use crate::status::Fail;
use crate::types::*;
use sm_core::{Point3, Vector3};
use sm_rt_api::{
    BoundaryHit, CellId, Closest, FaceAction, FaceId, Level, LostCause, LostInfo, Located, Peek, RegionId, ResponseKind, RtError, Step,
    StepEnd, SurfaceId, TrackState,
};

/// Id sentinels as the contract spells them.
const RAW_NONE: u32 = u32::MAX;
const RAW_COMPLEMENT: u32 = u32::MAX - 1;

/// Cell id to the C spelling.
pub fn cell_out(c: CellId) -> i64 {
    match c.0 {
        RAW_NONE => SMRT_NONE,
        RAW_COMPLEMENT => SMRT_COMPLEMENT,
        v => i64::from(v),
    }
}

/// Plain id (face, surface, component, instance) to the C spelling.
pub fn id_out(raw: u32) -> i64 {
    if raw == RAW_NONE { SMRT_NONE } else { i64::from(raw) }
}

/// Optional face to the C spelling.
pub fn face_out(f: Option<FaceId>) -> i64 {
    f.map_or(SMRT_NONE, |f| id_out(f.0))
}

/// Region id to the C spelling; the implicit-complement region is `SMRT_COMPLEMENT`.
pub fn region_out(r: RegionId, complement: Option<RegionId>) -> i64 {
    if r.is_none() {
        SMRT_NONE
    } else if Some(r) == complement {
        SMRT_COMPLEMENT
    } else {
        i64::from(r.0)
    }
}

fn unknown(kind: &'static str, v: i64) -> Fail {
    // The raw id of the error is only for the message; clamp it into u32.
    Fail::Rt(RtError::UnknownId { kind, id: u32::try_from(v).unwrap_or(u32::MAX) })
}

/// Region from the C spelling (`-1` none, `-2` complement, else a raw id).
pub fn region_in(v: i64, complement: Option<RegionId>) -> Result<RegionId, Fail> {
    match v {
        SMRT_NONE => Ok(RegionId::NONE),
        SMRT_COMPLEMENT => complement.ok_or_else(|| unknown("region", v)),
        v => match u32::try_from(v) {
            Ok(r) if r < RAW_COMPLEMENT => Ok(RegionId(r)),
            _ => Err(unknown("region", v)),
        },
    }
}

/// Optional cell hint from the C spelling (`-1` = no hint).
pub fn hint_in(v: i64) -> Result<Option<CellId>, Fail> {
    match v {
        SMRT_NONE => Ok(None),
        SMRT_COMPLEMENT => Ok(Some(CellId::COMPLEMENT)),
        v => match u32::try_from(v) {
            Ok(c) if c < RAW_COMPLEMENT => Ok(Some(CellId(c))),
            _ => Err(unknown("cell", v)),
        },
    }
}

/// Cell id from the C spelling (`-1` none, `-2` complement).
pub fn cell_in(v: i64) -> Result<CellId, Fail> {
    Ok(hint_in(v)?.unwrap_or(CellId::NONE))
}

/// Face id from the C spelling.
pub fn face_in(v: i64) -> Result<FaceId, Fail> {
    match u32::try_from(v) {
        Ok(f) if f < RAW_NONE => Ok(FaceId(f)),
        _ => Err(unknown("face", v)),
    }
}

/// Surface id from the C spelling.
pub fn surface_in(v: i64) -> Result<SurfaceId, Fail> {
    match u32::try_from(v) {
        Ok(s) if s < RAW_NONE => Ok(SurfaceId(s)),
        _ => Err(unknown("surface", v)),
    }
}

/// `[x, y, z]` to a point.
pub fn point(p: [f64; 3]) -> Point3 {
    Point3::new(p[0], p[1], p[2])
}

/// `[x, y, z]` to a vector.
pub fn vector(p: [f64; 3]) -> Vector3 {
    Vector3::new(p[0], p[1], p[2])
}

/// Vector to `[x, y, z]`.
pub fn arr(v: Vector3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

/// Point to `[x, y, z]`.
pub fn parr(p: Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}

fn level_out(l: &Level) -> SmrtLevel {
    SmrtLevel { component: id_out(l.component.0), instance: id_out(l.instance.0), lattice: l.lattice, pad: 0 }
}

/// First eight levels into the C array; returns `(levels_total, levels_len)`.
pub fn levels_out(src: &[Level], dst: &mut [SmrtLevel; SMRT_MAX_LEVELS]) -> (i32, i32) {
    let n = src.len().min(SMRT_MAX_LEVELS);
    for (d, s) in dst.iter_mut().zip(src.iter()) {
        *d = level_out(s);
    }
    for d in dst.iter_mut().skip(n) {
        *d = SmrtLevel::default();
    }
    (i32::try_from(src.len()).unwrap_or(i32::MAX), n as i32)
}

/// State to its C view.
pub fn state_out(st: &TrackState, comp: Option<RegionId>, out: &mut SmrtState) {
    out.pos = parr(st.pos);
    out.dir = arr(st.dir);
    out.cell = cell_out(st.cell);
    out.region = region_out(st.region, comp);
    out.entered_via = face_out(st.entered_via);
    (out.levels_total, out.levels_len) = levels_out(st.levels.as_slice(), &mut out.levels);
}

/// Located result to its C view.
pub fn located_out(l: &Located, comp: Option<RegionId>, out: &mut SmrtLocated) {
    out.cell = cell_out(l.cell);
    out.region = region_out(l.region, comp);
    out.on_face = face_out(l.on_face);
    (out.levels_total, out.levels_len) = levels_out(l.levels.as_slice(), &mut out.levels);
}

/// Closest answer to its C view.
pub fn closest_out(c: &Closest, out: &mut SmrtClosest) {
    out.dist = c.dist;
    out.surface = id_out(c.surface.0);
    out.point = parr(c.point);
}

fn lost_code(c: LostCause) -> i32 {
    match c {
        LostCause::NoCellContainsPoint => 1,
        LostCause::NoExitFace => 2,
        LostCause::InconsistentEntryFace => 3,
        LostCause::NonFinite => 4,
        LostCause::MaxStepsExceeded => 5,
        LostCause::RecoveryFailed => 6,
        _ => 6,
    }
}

fn lost_into(l: &LostInfo, out: &mut SmrtStep) {
    out.end = SMRT_END_LOST;
    out.lost_cause = lost_code(l.cause);
    out.lost_cell = cell_out(l.last_cell);
    out.lost_pos = parr(l.pos);
    out.lost_dir = arr(l.dir);
    out.face = face_out(l.last_face);
    out.surface = l.nearest_surface.map_or(SMRT_NONE, |s| id_out(s.0));
}

/// `(action code, albedo)` of a face action.
pub fn action_code(a: &FaceAction) -> (i32, f64) {
    match a {
        FaceAction::Respond(ResponseKind::Albedo(v)) => (SMRT_ACT_ALBEDO, *v),
        FaceAction::Respond(k) => (k.code() as i32, 0.0),
        FaceAction::Reflect => (SMRT_ACT_MIRROR, 0.0),
        FaceAction::Wrap { .. } => (SMRT_ACT_WRAP, 0.0),
        _ => (SMRT_ACT_NONE, 0.0),
    }
}

/// A `Step` into its C view (every field is overwritten).
pub fn step_out(s: &Step, comp: Option<RegionId>, out: &mut SmrtStep) {
    *out = SmrtStep {
        dist: s.dist,
        cell: cell_out(s.cell),
        region: region_out(s.region, comp),
        end: SMRT_END_LIMIT,
        action: SMRT_ACT_NONE,
        face: SMRT_NONE,
        surface: SMRT_NONE,
        to_region: SMRT_NONE,
        to_cell: SMRT_NONE,
        albedo: 0.0,
        lost_cause: SMRT_LOST_NONE,
        pad: 0,
        lost_cell: SMRT_NONE,
        lost_pos: [0.0; 3],
        lost_dir: [0.0; 3],
    };
    end_into(&s.end, comp, out);
}

/// Fill the event fields of `out` from a `StepEnd`.
pub fn end_into(e: &StepEnd, comp: Option<RegionId>, out: &mut SmrtStep) {
    match e {
        StepEnd::Limit => out.end = SMRT_END_LIMIT,
        StepEnd::CellFace { face, to } => {
            out.end = SMRT_END_CELL_FACE;
            out.face = id_out(face.0);
            out.to_cell = cell_out(*to);
        }
        StepEnd::SlotChange { to_cell } => {
            out.end = SMRT_END_SLOT_CHANGE;
            out.to_cell = cell_out(*to_cell);
        }
        StepEnd::RegionBoundary { surface, face, to_region, to_cell } => {
            out.end = SMRT_END_REGION_BOUNDARY;
            out.surface = id_out(surface.0);
            out.face = id_out(face.0);
            out.to_region = region_out(*to_region, comp);
            out.to_cell = cell_out(*to_cell);
        }
        StepEnd::Boundary { surface, face, action } => {
            out.end = SMRT_END_BOUNDARY;
            out.surface = id_out(surface.0);
            out.face = id_out(face.0);
            (out.action, out.albedo) = action_code(action);
        }
        StepEnd::Lost(l) => lost_into(l, out),
        // A variant added to the contract later: report it as a lost particle of unknown cause.
        _ => {
            out.end = SMRT_END_LOST;
            out.lost_cause = 6;
        }
    }
}

/// A fire result into the C view: `dist` is the total distance, `cell`/`region` those the
/// particle started in (the region is the one fired through).
pub fn fire_out(dist: f64, e: &StepEnd, cell: CellId, region: RegionId, comp: Option<RegionId>, out: &mut SmrtStep) {
    *out = SmrtStep {
        dist,
        cell: cell_out(cell),
        region: region_out(region, comp),
        end: SMRT_END_LIMIT,
        action: SMRT_ACT_NONE,
        face: SMRT_NONE,
        surface: SMRT_NONE,
        to_region: SMRT_NONE,
        to_cell: SMRT_NONE,
        albedo: 0.0,
        lost_cause: SMRT_LOST_NONE,
        pad: 0,
        lost_cell: SMRT_NONE,
        lost_pos: [0.0; 3],
        lost_dir: [0.0; 3],
    };
    end_into(e, comp, out);
}

fn hit_into(h: &BoundaryHit, comp: Option<RegionId>, out: &mut SmrtStep) {
    out.dist = h.dist;
    out.surface = id_out(h.surface.0);
    out.face = id_out(h.face.0);
    match &h.action {
        Some(a) => {
            out.end = SMRT_END_BOUNDARY;
            (out.action, out.albedo) = action_code(a);
        }
        None => {
            out.end = SMRT_END_REGION_BOUNDARY;
            out.to_region = region_out(h.to_region, comp);
        }
    }
}

/// A peek into the C view; `cell`/`region` are the particle's own. Returns true when a surface
/// (or a loss) was found, false for "nothing within the limit".
pub fn peek_out(p: &Peek, cell: CellId, region: RegionId, comp: Option<RegionId>, out: &mut SmrtStep) -> bool {
    *out = SmrtStep {
        cell: cell_out(cell),
        region: region_out(region, comp),
        face: SMRT_NONE,
        surface: SMRT_NONE,
        to_region: SMRT_NONE,
        to_cell: SMRT_NONE,
        lost_cell: SMRT_NONE,
        ..SmrtStep::default()
    };
    match p {
        Peek::Limit => {
            out.end = SMRT_END_LIMIT;
            false
        }
        Peek::Hit(h) => {
            hit_into(h, comp, out);
            true
        }
        Peek::Lost(l) => {
            lost_into(l, out);
            true
        }
        _ => {
            out.end = SMRT_END_LOST;
            out.lost_cause = 6;
            true
        }
    }
}

/// Response kind from the C spelling.
pub fn response_in(kind: i32, albedo: f64) -> Result<ResponseKind, Fail> {
    match kind {
        SMRT_RESP_VACUUM => Ok(ResponseKind::Vacuum),
        SMRT_RESP_WHITE => Ok(ResponseKind::White),
        SMRT_RESP_ALBEDO => Ok(ResponseKind::Albedo(albedo)),
        SMRT_RESP_REFLECTIVE => Ok(ResponseKind::Reflective),
        k => Err(Fail::bad(format!("unknown response kind {k}"))),
    }
}
