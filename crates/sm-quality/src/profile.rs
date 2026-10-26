//! Quality profiles: which metrics to grade and with which limits.
//!
//! The numbers are data in one table per profile (`DESIGN.md`, "Finite-volume metrics"). The
//! OpenFOAM and Fluent profiles carry the tools' own limits; Generic and Transport are
//! provisional engineering guesses.

use crate::fv::FvMetric;
use crate::metric::Metric;

impl Metric {
    /// Stable snake_case name of the variant (the wire name); inverse of [`Metric::from_name`].
    pub const fn name(self) -> &'static str {
        match self {
            Metric::Volume => "volume",
            Metric::Jacobian => "jacobian",
            Metric::ScaledJacobian => "scaled_jacobian",
            Metric::AspectRatio => "aspect_ratio",
            Metric::AspectFrobenius => "aspect_frobenius",
            Metric::MaxAspectFrobenius => "max_aspect_frobenius",
            Metric::MeanAspectFrobenius => "mean_aspect_frobenius",
            Metric::AspectGamma => "aspect_gamma",
            Metric::RadiusRatio => "radius_ratio",
            Metric::CollapseRatio => "collapse_ratio",
            Metric::EdgeRatio => "edge_ratio",
            Metric::MaxEdgeRatio => "max_edge_ratio",
            Metric::Skew => "skew",
            Metric::Taper => "taper",
            Metric::Warpage => "warpage",
            Metric::Stretch => "stretch",
            Metric::Shear => "shear",
            Metric::Oddy => "oddy",
            Metric::Diagonal => "diagonal",
            Metric::Dimension => "dimension",
            Metric::Condition => "condition",
            Metric::Distortion => "distortion",
            Metric::Shape => "shape",
            Metric::RelativeSizeSquared => "relative_size_squared",
            Metric::ShapeAndSize => "shape_and_size",
            Metric::ShearAndSize => "shear_and_size",
            Metric::MinAngle => "min_angle",
            Metric::MaxAngle => "max_angle",
            Metric::Sphericity => "sphericity",
            Metric::Starness => "starness",
        }
    }

    /// The metric named `name` (inverse of [`Metric::name`]).
    pub fn from_name(name: &str) -> Option<Metric> {
        Metric::ALL.iter().copied().find(|m| m.name() == name)
    }
}

/// A named quality profile.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum Profile {
    /// General-purpose limits (provisional).
    Generic,
    /// OpenFOAM `checkMesh` limits.
    OpenFoam,
    /// Fluent mesh-check guidance (orthogonal quality only).
    Fluent,
    /// Planar convex faces for ray tracing and sweeps (provisional).
    Transport,
}

impl Profile {
    /// Every profile.
    pub const ALL: &'static [Profile] = &[Profile::Generic, Profile::OpenFoam, Profile::Fluent, Profile::Transport];

    /// Stable name: `generic`, `openfoam`, `fluent`, `transport`.
    pub const fn name(self) -> &'static str {
        match self {
            Profile::Generic => "generic",
            Profile::OpenFoam => "openfoam",
            Profile::Fluent => "fluent",
            Profile::Transport => "transport",
        }
    }

    /// The profile named `name` (inverse of [`Profile::name`]).
    pub fn from_name(name: &str) -> Option<Profile> {
        Profile::ALL.iter().copied().find(|p| p.name() == name)
    }
}

/// What a rule measures: a Verdict element metric or a finite-volume metric.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum Measure {
    /// A Verdict element metric (cell-indexed).
    Verdict(Metric),
    /// A finite-volume metric (face- or cell-indexed).
    Fv(FvMetric),
}

impl Measure {
    /// Wire name of the underlying metric.
    pub const fn name(self) -> &'static str {
        match self {
            Measure::Verdict(m) => m.name(),
            Measure::Fv(m) => m.name(),
        }
    }

    /// The measure with wire name `name`: a finite-volume metric or a Verdict metric (the
    /// namespaces do not overlap).
    pub fn from_name(name: &str) -> Option<Measure> {
        FvMetric::from_name(name).map(Measure::Fv).or_else(|| Metric::from_name(name).map(Measure::Verdict))
    }

    /// Unit of the values: `"deg"` for angles, `"volume"` for `Volume` and `Jacobian`, `"length"`
    /// for hex `Dimension`, `""` otherwise.
    pub const fn unit(self) -> &'static str {
        match self {
            Measure::Fv(m) => m.unit(),
            Measure::Verdict(Metric::MinAngle | Metric::MaxAngle) => "deg",
            Measure::Verdict(Metric::Volume | Metric::Jacobian) => "volume",
            Measure::Verdict(Metric::Dimension) => "length",
            Measure::Verdict(_) => "",
        }
    }
}

/// Which direction of a metric is bad.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum Sense {
    /// Small values are bad (a limit is a floor).
    LowerIsWorse,
    /// Large values are bad (a limit is a ceiling).
    HigherIsWorse,
}

/// Grade of one value against a [`Rule`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Grade {
    /// Within the warning limit.
    Ok,
    /// Past the warning limit, within the failure limit.
    Warn,
    /// Past the failure limit.
    Fail,
    /// The value is NaN or infinite (undefined for the entity).
    Undefined,
}

impl Grade {
    /// 0 for `Ok` and `Undefined`, 1 for `Warn`, 2 for `Fail`.
    pub const fn rank(self) -> u8 {
        match self {
            Grade::Ok | Grade::Undefined => 0,
            Grade::Warn => 1,
            Grade::Fail => 2,
        }
    }

    /// True if the grade is `Warn` or `Fail` and at least as bad as `level` (`Warn` or `Fail`).
    pub const fn reaches(self, level: Grade) -> bool {
        self.rank() >= level.rank() && self.rank() > 0
    }

    /// `"ok"`, `"warn"`, `"fail"` or `"undefined"`.
    pub const fn name(self) -> &'static str {
        match self {
            Grade::Ok => "ok",
            Grade::Warn => "warn",
            Grade::Fail => "fail",
            Grade::Undefined => "undefined",
        }
    }
}

/// One graded measure of a profile.
///
/// A value equal to a limit passes. A non-finite `fail` means the rule never fails (the
/// OpenFOAM flatness check only warns).
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct Rule {
    /// The graded measure.
    pub measure: Measure,
    /// Which direction is bad.
    pub sense: Sense,
    /// Warning limit, if any.
    pub warn: Option<f64>,
    /// Failure limit.
    pub fail: f64,
    /// Failure limit of boundary faces, where it differs from `fail`.
    pub fail_boundary: Option<f64>,
    /// Where the number comes from.
    pub note: &'static str,
}

impl Rule {
    /// Grade `v`; `boundary` selects `fail_boundary` (when present) for boundary faces.
    pub fn grade(&self, v: f64, boundary: bool) -> Grade {
        if !v.is_finite() {
            return Grade::Undefined;
        }
        let fail = if boundary { self.fail_boundary.unwrap_or(self.fail) } else { self.fail };
        let past = |limit: f64| match self.sense {
            Sense::LowerIsWorse => v < limit,
            Sense::HigherIsWorse => v > limit,
        };
        if past(fail) {
            Grade::Fail
        } else if self.warn.is_some_and(past) {
            Grade::Warn
        } else {
            Grade::Ok
        }
    }
}

const fn rule(measure: Measure, sense: Sense, warn: Option<f64>, fail: f64, fail_boundary: Option<f64>, note: &'static str) -> Rule {
    Rule { measure, sense, warn, fail, fail_boundary, note }
}

const fn v(m: Metric) -> Measure {
    Measure::Verdict(m)
}
const fn f(m: FvMetric) -> Measure {
    Measure::Fv(m)
}

const GENERIC: &[Rule] = &[
    rule(v(Metric::ScaledJacobian), Sense::LowerIsWorse, Some(0.2), 0.05, None, "provisional"),
    rule(v(Metric::EdgeRatio), Sense::HigherIsWorse, Some(10.0), 100.0, None, "provisional"),
    rule(f(FvMetric::NonOrthogonality), Sense::HigherIsWorse, Some(50.0), 70.0, None, "fail: checkMesh; warn provisional"),
    rule(f(FvMetric::Skewness), Sense::HigherIsWorse, Some(2.0), 4.0, Some(20.0), "fail: checkMesh; warn provisional"),
    rule(f(FvMetric::VolumeRatio), Sense::HigherIsWorse, Some(4.0), 20.0, None, "provisional"),
    rule(f(FvMetric::InterpolationWeight), Sense::LowerIsWorse, Some(0.2), 0.05, None, "fail: checkMesh; warn provisional"),
    rule(f(FvMetric::FaceFlatness), Sense::LowerIsWorse, Some(0.9), 0.8, None, "provisional"),
    rule(f(FvMetric::FaceConcavity), Sense::HigherIsWorse, Some(5.0), 10.0, None, "fail: checkMesh; warn provisional"),
];

const OPENFOAM: &[Rule] = &[
    rule(f(FvMetric::NonOrthogonality), Sense::HigherIsWorse, None, 70.0, None, "checkMesh checkFaceOrthogonality"),
    rule(f(FvMetric::Skewness), Sense::HigherIsWorse, None, 4.0, Some(20.0), "checkMesh checkFaceSkewness (recalled)"),
    rule(f(FvMetric::InterpolationWeight), Sense::LowerIsWorse, None, 0.05, None, "checkMesh checkFaceWeight"),
    rule(f(FvMetric::FaceFlatness), Sense::LowerIsWorse, Some(0.8), f64::NEG_INFINITY, None, "checkMesh checkFaceFlatness warns only"),
    rule(f(FvMetric::FaceConcavity), Sense::HigherIsWorse, None, 10.0, None, "checkMesh checkFaceAngles"),
    rule(f(FvMetric::CellDeterminant), Sense::LowerIsWorse, None, 1e-3, None, "checkMesh checkCellDeterminant"),
];

const FLUENT: &[Rule] = &[rule(
    f(FvMetric::OrthogonalQuality),
    Sense::LowerIsWorse,
    Some(0.1),
    0.01,
    None,
    "Fluent guidance: warn below 0.1, fail below 0.01",
)];

const TRANSPORT: &[Rule] = &[
    rule(f(FvMetric::FaceFlatness), Sense::LowerIsWorse, Some(0.9999), 0.999, None, "provisional, awaits ratification"),
    rule(f(FvMetric::FaceConcavity), Sense::HigherIsWorse, Some(0.1), 1.0, None, "provisional, awaits ratification"),
    rule(v(Metric::Starness), Sense::LowerIsWorse, Some(0.2), 0.0, None, "provisional, awaits ratification"),
    rule(v(Metric::EdgeRatio), Sense::HigherIsWorse, Some(20.0), 200.0, None, "provisional, awaits ratification"),
    rule(f(FvMetric::VolumeRatio), Sense::HigherIsWorse, Some(10.0), 100.0, None, "provisional, awaits ratification"),
];

/// The rules of `profile`, in grading order.
pub fn rules(profile: Profile) -> &'static [Rule] {
    match profile {
        Profile::Generic => GENERIC,
        Profile::OpenFoam => OPENFOAM,
        Profile::Fluent => FLUENT,
        Profile::Transport => TRANSPORT,
    }
}
