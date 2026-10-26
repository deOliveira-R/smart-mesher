//! The `"sweep"` volume scheme.

use crate::discover::{Columns, discover_columns};
use crate::emit::generate_prism;
use crate::fit::{best_effort_transform, fit_target_transform};
use crate::params::{SweepParams, TargetMode};
use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_scheme_api::{
    Applicability, Caps, CopyConstraint, CopyReason, Firmness, GenCtx, GenerateFailure,
    IntervalCtx, IntervalSink, MeshInput, Params, Patch, PlanCtx, Rel, Requirements, Role, Scheme,
    SchemeDim, SchemeError, SchemeName, Target, geom,
};
use sm_topo_api::{Side, TopologyView, TopologyViewExt};

/// Relative tolerance of the target copy test, times the source bounding-box diagonal. The
/// model tolerance is not reachable through `PlanCtx`, so this is a constant.
const COPY_TOL: f64 = 1e-9;

/// The `"sweep"` scheme: 1-to-1 volume sweep of a source face along columns to a target face.
///
/// See the crate docs and [`SweepParams`].
#[derive(Clone, Copy, Debug, Default)]
pub struct Sweep;

/// What planning learns about a sweep.
pub(crate) struct Analysis {
    pub columns: Columns,
    pub transform: Transform,
}

fn outward_normal(
    t: &dyn TopologyView,
    volume: EntityId,
    face: EntityId,
) -> Result<Vector3, String> {
    let p = t
        .face_outline(face)
        .first()
        .copied()
        .ok_or_else(|| format!("face {face} has no outline"))?;
    let n = geom::face_normal(t, face, p).map_err(|e| e.to_string())?;
    let side = geom::volume_face_uses(t, volume)
        .into_iter()
        .find(|(f, _)| *f == face)
        .map(|(_, s)| s);
    match side {
        Some(Side::Right) => Ok(-n),
        Some(Side::Both) => Err(format!("face {face} is a baffle of the volume")),
        _ => Ok(n),
    }
}

/// Source and target faces: from the parameters, else the first valid pair (lowest ids first).
fn find_columns(
    input: &MeshInput<'_>,
    target: Target,
    sp: &SweepParams,
) -> Result<Columns, String> {
    let t = input.topo;
    let faces = input.piece_faces(target).map_err(|e| e.to_string())?;
    let v = target.entity().ok_or("a sweep meshes a single piece")?;
    let one = [(v, faces.clone())];
    match (sp.source, sp.target) {
        (Some([s]), Some([g])) => discover_columns(t, &one, s, g).map_err(|e| e.to_string()),
        (None, None) => {
            let mut last = String::from("the piece has fewer than two faces");
            for (i, &s) in faces.iter().enumerate() {
                for &g in &faces[i + 1..] {
                    match discover_columns(t, &one, s, g) {
                        Ok(st) => return Ok(st),
                        Err(e) => last = e.to_string(),
                    }
                }
            }
            Err(format!("no pair of faces gives columns (last: {last})"))
        }
        _ => Err("give both source and target, or neither".into()),
    }
}

pub(crate) fn analyse(
    input: &MeshInput<'_>,
    target: Target,
    sp: &SweepParams,
) -> Result<Analysis, String> {
    let t = input.topo;
    let columns = find_columns(input, target, sp)?;
    let v = target.entity().ok_or("a sweep meshes a single piece")?;
    let vm = columns.vertex_map(t);
    let mut src: Vec<Point3> = Vec::with_capacity(vm.len());
    let mut dst: Vec<Point3> = Vec::with_capacity(vm.len());
    for (a, b) in &vm {
        src.push(geom::vertex_point(t, *a).map_err(|e| e.to_string())?);
        dst.push(geom::vertex_point(t, *b).map_err(|e| e.to_string())?);
    }
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for p in &src {
        for (a, x) in [p.x, p.y, p.z].into_iter().enumerate() {
            lo[a] = lo[a].min(x);
            hi[a] = hi[a].max(x);
        }
    }
    let diag = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt();
    let tol = COPY_TOL * diag;
    let n_s = outward_normal(t, v, columns.source)?;
    let n_t = outward_normal(t, v, columns.target)?;
    let strict = fit_target_transform(&src, &dst, n_s, n_t, tol);
    let transform = match (strict, sp.target_mode) {
        (Some(tr), _) => tr,
        (None, TargetMode::Copy) => {
            best_effort_transform(&src, &dst, n_s, n_t)
                .ok_or("no transform can be fitted to the vertex correspondence")?
                .0
        }
        (None, _) => {
            let resid = best_effort_transform(&src, &dst, n_s, n_t).map(|r| r.1);
            return Err(match resid {
                Some(r) => format!(
                    "the target is not an affine image of the source (residual {r:.3e}, tolerance {tol:.3e}); target_mode project is not implemented"
                ),
                None => "the target is not an affine image of the source".to_string(),
            });
        }
    };
    Ok(Analysis { columns, transform })
}

fn bad_input(reason: impl Into<String>) -> SchemeError {
    SchemeError::Input(reason.into())
}

fn columns_of(data: Option<&Params>) -> Result<Columns, SchemeError> {
    let d = data.ok_or_else(|| bad_input("no sweep data"))?;
    serde_json::from_value(d.0.clone()).map_err(|e| bad_input(e.to_string()))
}

/// Interval rows shared by every column: the same layer counts step by step.
fn column_equalities(st: &Columns, out: &mut IntervalSink<'_>) {
    let Some(first) = st.columns.first() else {
        return;
    };
    for c in st.columns.iter().skip(1) {
        for (a, b) in first.path.iter().zip(&c.path) {
            out.equal(&[a.0], &[b.0]);
        }
    }
}

impl Scheme for Sweep {
    fn name(&self) -> &str {
        crate::NAME
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::HEX | Caps::WEDGE | Caps::POLY
    }
    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, p: &Params) -> Applicability {
        if matches!(target, Target::Group(_)) {
            return Applicability::no("a sweep meshes one piece (multisweep is not supported)");
        }
        // Bad parameters are reported as `BadParams` when the requirements run.
        let Ok(sp) = SweepParams::from_params(p) else {
            return Applicability::yes(60, "parameters are checked when planning");
        };
        match analyse(cx.input, target, &sp) {
            Ok(_) => Applicability::yes(60, "1-to-1 sweepable"),
            Err(e) => Applicability::no(e),
        }
    }
    fn requirements(
        &self,
        cx: &PlanCtx<'_>,
        target: Target,
        p: &Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        let sp = SweepParams::from_params(p)?;
        let an = analyse(cx.input, target, &sp).map_err(|reason| SchemeError::NotApplicable {
            target,
            scheme: SchemeName::new(crate::NAME),
            reason,
        })?;
        let st = &an.columns;
        out.role(
            Target::Surface(st.source),
            Role::SweepSource { volume: target },
        );
        out.role(
            Target::Surface(st.target),
            Role::SweepTarget { volume: target },
        );
        for l in &st.linking {
            out.role(Target::Surface(*l), Role::SweepLinking { volume: target });
            out.require_caps(
                Target::Surface(*l),
                Caps::QUAD | Caps::BLOCK_STRUCTURED,
                Firmness::Hard,
            );
        }
        out.copy(
            CopyConstraint::new(
                Target::Surface(st.source),
                Target::Surface(st.target),
                an.transform,
                CopyReason::SweepTarget { volume: target },
            )
            .with_vertex_map(st.vertex_map(cx.input.topo)),
        );
        out.set_data(Params(
            serde_json::to_value(st).map_err(|e| bad_input(e.to_string()))?,
        ));
        Ok(())
    }
    fn interval_constraints(
        &self,
        cx: &IntervalCtx<'_>,
        target: Target,
        p: &Params,
        out: &mut IntervalSink<'_>,
    ) -> sm_scheme_api::Result<()> {
        let sp = SweepParams::from_params(p)?;
        let st = columns_of(cx.data(target))?;
        column_equalities(&st, out);
        if let (Some(n), Some(c)) = (sp.layers, st.columns.first()) {
            let terms: Vec<(EntityId, i32)> = c.path.iter().map(|(e, _)| (*e, 1)).collect();
            out.linear(&terms, Rel::Eq, i64::from(n));
        }
        Ok(())
    }
    fn generate(
        &self,
        cx: &GenCtx<'_>,
        target: Target,
        p: &Params,
    ) -> Result<Patch, GenerateFailure> {
        let sp = SweepParams::from_params(p).map_err(|e| {
            GenerateFailure::new(
                target,
                crate::NAME,
                sm_scheme_api::FailureKind::Constraint,
                e.to_string(),
            )
        })?;
        let st = columns_of(cx.data(target))
            .map_err(|e| GenerateFailure::fatal(target, crate::NAME, e.to_string()))?;
        generate_prism(cx, target, &st, &sp)
    }
}
