//! The sizing layer seam: size fields evaluated by the driver (interval goals) and by
//! unstructured schemes (pave, tetmesh).

use crate::error::{Result, SchemeError};
use crate::intent::SizingSpec;
use sm_core::Point3;

/// A symmetric 3x3 metric tensor (row-major upper triangle `xx, xy, xz, yy, yz, zz`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SymMat3(pub [f64; 6]);

/// A sizing function. Must be a pure function of the point (no interior mutability that
/// changes results), so concurrent schemes see the same sizes.
pub trait SizeField: Send + Sync {
    /// Isotropic target size at `p` (> 0).
    fn size(&self, p: Point3) -> f64;

    /// Anisotropic metric at `p`; the default is isotropic `I / size^2`.
    fn metric(&self, p: Point3) -> SymMat3 {
        let h = self.size(p);
        let a = 1.0 / (h * h);
        SymMat3([a, 0.0, 0.0, a, 0.0, a])
    }
}

/// A constant size.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConstantSize(pub f64);

impl SizeField for ConstantSize {
    fn size(&self, _p: Point3) -> f64 {
        self.0
    }
}

/// Build the size fields this crate knows (`Constant`); other functions come from the sizing
/// crate, which registers them with the driver.
pub fn builtin_field(spec: &SizingSpec) -> Option<Box<dyn SizeField>> {
    match spec {
        SizingSpec::Constant { size } if size.is_finite() && *size > 0.0 => {
            Some(Box::new(ConstantSize(*size)))
        }
        _ => None,
    }
}

/// The size field the driver builds for `spec` when it has no override: `None` without a spec,
/// the built-in field for `Constant`. A `Function` (or a non-positive constant) is a typed
/// error, never a silent fallback to one interval per edge: named functions need a
/// [`SizeField`] passed as `Driver::size`.
pub fn field_for(spec: Option<&SizingSpec>) -> Result<Option<Box<dyn SizeField>>> {
    match spec {
        None => Ok(None),
        Some(s @ SizingSpec::Constant { size }) => builtin_field(s)
            .map(Some)
            .ok_or_else(|| SchemeError::Input(format!("sizing: constant size {size} must be positive and finite"))),
        Some(SizingSpec::Function { name, .. }) => Err(SchemeError::Input(format!(
            "sizing function '{name}' needs a SizeField (set Driver::size)"
        ))),
    }
}
