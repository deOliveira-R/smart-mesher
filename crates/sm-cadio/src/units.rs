//! Units and uncertainty of a shape representation context.

use crate::args::Ents;
use crate::error::StepError;
use crate::part21::{Entity, Param, StepFile};

/// A length unit as the file declares it.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct FileLength {
    pub name: String,
    /// Micrometres per unit.
    pub um: f64,
}

/// A plane angle unit as the file declares it.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct FileAngle {
    pub name: String,
    pub rad: f64,
}

/// What a context declares.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ContextUnits {
    pub length: Option<FileLength>,
    pub angle: FileAngle,
    /// Distance uncertainty in file length units.
    pub uncertainty: Option<f64>,
}

enum Unit {
    Length(FileLength),
    Angle(FileAngle),
    Other,
}

/// Decade exponent of an SI prefix.
fn prefix_exp(name: &str) -> Option<i32> {
    Some(match name {
        "EXA" => 18,
        "PETA" => 15,
        "TERA" => 12,
        "GIGA" => 9,
        "MEGA" => 6,
        "KILO" => 3,
        "HECTO" => 2,
        "DECA" => 1,
        "DECI" => -1,
        "CENTI" => -2,
        "MILLI" => -3,
        "MICRO" => -6,
        "NANO" => -9,
        "PICO" => -12,
        "FEMTO" => -15,
        "ATTO" => -18,
        _ => return None,
    })
}

fn prefix_name(exp: i32) -> &'static str {
    match exp {
        18 => "exa",
        15 => "peta",
        12 => "tera",
        9 => "giga",
        6 => "mega",
        3 => "kilo",
        2 => "hecto",
        1 => "deca",
        -1 => "deci",
        -2 => "centi",
        -3 => "milli",
        -6 => "micro",
        -9 => "nano",
        -12 => "pico",
        -15 => "femto",
        -18 => "atto",
        _ => "",
    }
}

fn unit(ents: Ents<'_>, id: u64, by: u64, depth: usize) -> Result<Unit, StepError> {
    if depth > 8 {
        return Err(StepError::Cycle { id });
    }
    let e = ents.get(id, by)?;
    let is_length = e.is("LENGTH_UNIT");
    let is_angle = e.is("PLANE_ANGLE_UNIT");
    if let Some(si) = e.rec("SI_UNIT") {
        let exp = match si.params.first() {
            Some(Param::Enum(p)) => prefix_exp(p).ok_or_else(|| StepError::BadParam { id, entity: "SI_UNIT".into(), index: 0, msg: format!("unknown prefix .{p}.") })?,
            _ => 0,
        };
        let base = match si.params.get(1) {
            Some(Param::Enum(n)) => n.as_str(),
            _ => "",
        };
        return Ok(match base {
            "METRE" => Unit::Length(FileLength { name: format!("{}metre", prefix_name(exp)), um: 10f64.powi(exp + 6) }),
            "RADIAN" => Unit::Angle(FileAngle { name: format!("{}radian", prefix_name(exp)), rad: 10f64.powi(exp) }),
            _ => Unit::Other,
        });
    }
    if let Some(cb) = e.rec("CONVERSION_BASED_UNIT") {
        let name = match cb.params.first() {
            Some(Param::Str(s)) => s.to_lowercase(),
            _ => String::new(),
        };
        let m = ents.get(match cb.params.get(1) {
            Some(Param::Ref(r)) => *r,
            _ => return Err(StepError::BadParam { id, entity: "CONVERSION_BASED_UNIT".into(), index: 1, msg: "expected a reference".into() }),
        }, id)?;
        // LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4), #unit) or the angle analogue.
        let rec = m.first().ok_or(StepError::Cycle { id })?;
        let value = match rec.params.first() {
            Some(Param::Typed(_, inner)) => match &**inner {
                Param::Real(v) => *v,
                Param::Int(v) => *v as f64,
                _ => return Err(StepError::BadParam { id, entity: rec.name.clone(), index: 0, msg: "expected a measure".into() }),
            },
            Some(Param::Real(v)) => *v,
            _ => return Err(StepError::BadParam { id, entity: rec.name.clone(), index: 0, msg: "expected a measure".into() }),
        };
        let base_id = match rec.params.get(1) {
            Some(Param::Ref(r)) => *r,
            _ => return Err(StepError::BadParam { id, entity: rec.name.clone(), index: 1, msg: "expected a unit reference".into() }),
        };
        let base = unit(ents, base_id, id, depth + 1)?;
        let length_like = is_length || rec.name.starts_with("LENGTH");
        let angle_like = is_angle || rec.name.starts_with("PLANE_ANGLE");
        return Ok(match base {
            Unit::Length(b) if length_like || !angle_like => Unit::Length(FileLength { name, um: value * b.um }),
            Unit::Angle(b) if angle_like || !length_like => Unit::Angle(FileAngle { name, rad: value * b.rad }),
            _ => Unit::Other,
        });
    }
    Ok(Unit::Other)
}

/// Units declared by the representation context `ctx`.
pub(crate) fn context_units(file: &StepFile, ctx: u64) -> Result<ContextUnits, StepError> {
    let ents = Ents::new(file);
    let e: &Entity = ents.get(ctx, ctx)?;
    let mut length = None;
    let mut angle = FileAngle { name: "radian".into(), rad: 1.0 };
    if let Some(r) = e.rec("GLOBAL_UNIT_ASSIGNED_CONTEXT")
        && let Some(Param::List(units)) = r.params.first()
    {
        for u in units {
            if let Param::Ref(uid) = u {
                match unit(ents, *uid, ctx, 0)? {
                    Unit::Length(l) => length = Some(l),
                    Unit::Angle(a) => angle = a,
                    Unit::Other => {}
                }
            }
        }
    }
    let mut uncertainty = None;
    if let Some(r) = e.rec("GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT")
        && let Some(Param::List(us)) = r.params.first()
    {
        for u in us {
            let Param::Ref(uid) = u else { continue };
            let ue = ents.get(*uid, ctx)?;
            let Some(rec) = ue.rec("UNCERTAINTY_MEASURE_WITH_UNIT") else { continue };
            let value = match rec.params.first() {
                Some(Param::Typed(_, inner)) => match &**inner {
                    Param::Real(v) => *v,
                    Param::Int(v) => *v as f64,
                    _ => continue,
                },
                Some(Param::Real(v)) => *v,
                _ => continue,
            };
            let factor = match rec.params.get(1) {
                Some(Param::Ref(r)) => match (unit(ents, *r, *uid, 0)?, &length) {
                    (Unit::Length(l), Some(fl)) if fl.um > 0.0 => l.um / fl.um,
                    _ => 1.0,
                },
                _ => 1.0,
            };
            if value.is_finite() && value >= 0.0 {
                uncertainty = Some(value * factor);
                break;
            }
        }
    }
    Ok(ContextUnits { length, angle, uncertainty })
}

/// The context id of the (first) representation listing `item` among its items.
pub(crate) fn contexts_by_item(file: &StepFile) -> std::collections::BTreeMap<u64, u64> {
    let mut out = std::collections::BTreeMap::new();
    for ent in file.entities.values() {
        let recs: &[crate::part21::Rec] = match ent {
            Entity::Simple(r) => std::slice::from_ref(r),
            Entity::Complex(v) => v,
        };
        for r in recs {
            if !r.name.ends_with("REPRESENTATION") || r.name == "DEFINITIONAL_REPRESENTATION" {
                continue;
            }
            let (Some(Param::List(items)), Some(Param::Ref(ctx))) = (r.params.get(1), r.params.get(2)) else { continue };
            for it in items {
                if let Param::Ref(i) = it {
                    out.entry(*i).or_insert(*ctx);
                }
            }
        }
    }
    out
}

/// A human name for a length factor.
pub(crate) fn describe_length(l: &FileLength) -> String {
    format!("{} ({} mm per unit)", l.name, l.um / 1000.0)
}

/// The angle unit assumed when no context declares one (radians).
pub(crate) fn context_units_default() -> FileAngle {
    FileAngle { name: "radian".into(), rad: 1.0 }
}
