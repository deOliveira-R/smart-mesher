//! Size fields: [`size_field`] builds a [`SizeField`] from a [`SizingSpec`].
//!
//! `Constant` is served by `sm_scheme_api::sizing::field_for`. Named functions
//! (`SizingSpec::Function { name, params }`), parameters as JSON numbers and `[x, y, z]` arrays:
//!
//! | name | parameters | size at `p` |
//! |---|---|---|
//! | `"linear"` | `p0`, `h0`, `p1`, `h1` | `h0 + (h1 - h0) t` |
//! | `"geometric"` | `p0`, `h0`, `p1`, `h1` (both `> 0`) | `h0 (h1 / h0)^t` |
//! | `"bias"` | `origin`, `axis`, `h0`, `ratio`, optional `h_min`, `h_max` | `h0 + (ratio - 1) s` |
//!
//! `t = clamp((p - p0) . (p1 - p0) / |p1 - p0|^2, 0, 1)`: the projection onto the segment, held
//! at the end values beyond it, so the size stays positive everywhere. For `"bias"`,
//! `s = max(0, (p - origin) . axis / |axis|)` is the distance along the axis: a geometric series
//! with first element `h0` and growth `ratio` has element size `h0 + (ratio - 1) x` at its start
//! position `x`, so the field is the continuous form of Cubit's bias. The result is clamped to
//! `[h_min, h_max]` (defaults `h0 / 1000` and unbounded), which keeps `ratio < 1` positive.

use serde_json::Value;
use sm_core::{Point3, Vector3};
use sm_scheme_api::{SchemeError, SchemeName, SizeField, SizingSpec};

/// Build the size field of a sizing spec (`None` for no field). `Constant` goes through
/// `sm_scheme_api::sizing::field_for`; functions are `"linear"`, `"geometric"` and `"bias"` (see
/// the [module docs](self)). An unknown name or bad parameters is [`SchemeError::BadParams`].
pub fn size_field(spec: &SizingSpec) -> Result<Option<Box<dyn SizeField>>, SchemeError> {
    match spec {
        SizingSpec::Constant { .. } => sm_scheme_api::sizing::field_for(Some(spec)),
        SizingSpec::Function { name, params } => {
            let bad = |reason: String| SchemeError::BadParams {
                scheme: SchemeName(format!("size_field:{name}")),
                reason,
            };
            let field: Box<dyn SizeField> = match name.as_str() {
                "linear" => Box::new(Interp::new(&params.0, false).map_err(bad)?),
                "geometric" => Box::new(Interp::new(&params.0, true).map_err(bad)?),
                "bias" => Box::new(Bias::new(&params.0).map_err(bad)?),
                other => return Err(bad(format!("unknown sizing function `{other}`"))),
            };
            Ok(Some(field))
        }
        // `SizingSpec` is non-exhaustive: a variant added later is reported, never ignored.
        #[allow(unreachable_patterns)]
        _ => Err(SchemeError::BadParams {
            scheme: SchemeName("size_field".into()),
            reason: "unsupported sizing spec".into(),
        }),
    }
}

fn num(p: &Value, key: &str) -> Result<f64, String> {
    p.get(key)
        .and_then(Value::as_f64)
        .filter(|x| x.is_finite())
        .ok_or_else(|| format!("`{key}` must be a finite number"))
}

fn pos(p: &Value, key: &str) -> Result<f64, String> {
    let x = num(p, key)?;
    if x > 0.0 {
        Ok(x)
    } else {
        Err(format!("`{key}` must be positive, got {x}"))
    }
}

fn point(p: &Value, key: &str) -> Result<Point3, String> {
    let a = p
        .get(key)
        .and_then(Value::as_array)
        .filter(|a| a.len() == 3)
        .ok_or_else(|| format!("`{key}` must be [x, y, z]"))?;
    let c: Vec<f64> = a
        .iter()
        .filter_map(Value::as_f64)
        .filter(|x| x.is_finite())
        .collect();
    if c.len() == 3 {
        Ok(Point3::new(c[0], c[1], c[2]))
    } else {
        Err(format!("`{key}` must be [x, y, z] of finite numbers"))
    }
}

/// Linear or geometric interpolation of the size between two points.
struct Interp {
    p0: Point3,
    d: Vector3,
    inv_len2: f64,
    h0: f64,
    h1: f64,
    geometric: bool,
}

impl Interp {
    fn new(p: &Value, geometric: bool) -> Result<Self, String> {
        let (p0, p1) = (point(p, "p0")?, point(p, "p1")?);
        let (h0, h1) = (pos(p, "h0")?, pos(p, "h1")?);
        let d = p1 - p0;
        let l2 = d.dot(d);
        if l2 <= 0.0 || !l2.is_finite() {
            return Err("`p0` and `p1` must differ".into());
        }
        Ok(Self {
            p0,
            d,
            inv_len2: 1.0 / l2,
            h0,
            h1,
            geometric,
        })
    }
}

impl SizeField for Interp {
    fn size(&self, p: Point3) -> f64 {
        let t = ((p - self.p0).dot(self.d) * self.inv_len2).clamp(0.0, 1.0);
        if self.geometric {
            self.h0 * (self.h1 / self.h0).powf(t)
        } else {
            self.h0 + (self.h1 - self.h0) * t
        }
    }
}

/// Distance-from-origin growth along an axis.
struct Bias {
    origin: Point3,
    axis: Vector3,
    h0: f64,
    ratio: f64,
    h_min: f64,
    h_max: f64,
}

impl Bias {
    fn new(p: &Value) -> Result<Self, String> {
        let origin = point(p, "origin")?;
        let a = point(p, "axis")?;
        let axis = Vector3::new(a.x, a.y, a.z);
        let n = axis.norm();
        if n <= 0.0 || !n.is_finite() {
            return Err("`axis` must be a nonzero vector".into());
        }
        let h0 = pos(p, "h0")?;
        let ratio = pos(p, "ratio")?;
        let h_min = if p.get("h_min").is_some() {
            pos(p, "h_min")?
        } else {
            h0 * 1e-3
        };
        let h_max = if p.get("h_max").is_some() {
            pos(p, "h_max")?
        } else {
            f64::INFINITY
        };
        if h_min > h_max {
            return Err("`h_min` exceeds `h_max`".into());
        }
        Ok(Self {
            origin,
            axis: axis * (1.0 / n),
            h0,
            ratio,
            h_min,
            h_max,
        })
    }
}

impl SizeField for Bias {
    fn size(&self, p: Point3) -> f64 {
        let s = (p - self.origin).dot(self.axis).max(0.0);
        (self.h0 + (self.ratio - 1.0) * s).clamp(self.h_min, self.h_max)
    }
}
