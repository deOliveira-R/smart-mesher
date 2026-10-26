//! The surface `submap` scheme: Cubit-style submapping of a single-loop face with more than
//! four logical sides (L, U, staircase) as a union of mapped rectangular blocks.
//!
//! The pure layers are [`lattice`] (the integer polygon the sides trace, exact validation,
//! Pick's theorem) and [`blocks`] (rectangular decomposition by cut lines). The scheme itself is
//! [`SurfaceSubmap`]; [`register`] adds it to a [`SchemeRegistry`]. See `DESIGN.md`.

pub mod blocks;
mod error;
mod generate;
pub mod lattice;
mod tfi;

pub use error::SubmapError;

use sm_scheme_api::geom;
use sm_scheme_api::{
    Applicability, Caps, FailureKind, Firmness, GenCtx, GenerateFailure, IntervalConstraint,
    IntervalCtx, IntervalSink, Params, Patch, PlanCtx, Requirements, Scheme, SchemeDim,
    SchemeError, SchemeRegistry, SideDir, Target, VertexRule, VertexType,
};

/// Registered name of the scheme.
pub const NAME: &str = "submap";

/// Register the surface submap scheme (`"submap"`, dimension [`SchemeDim::Surface`]).
///
/// The registry must not also hold the stub `Submap` of `sm_scheme_api::stubs`: both use this
/// name and the second registration is an error.
pub fn register(registry: &mut SchemeRegistry) -> sm_scheme_api::Result<()> {
    registry.register(Box::new(SurfaceSubmap))
}

/// Surface submapping: corner sum 4, two interval rows `sum(+i) = sum(-i)` and
/// `sum(+j) = sum(-j)` read from the logical sides' directions, interior as mapped rectangular
/// blocks. Produces `QUAD | BLOCK_STRUCTURED` (no single grid; consumers walk strips with
/// [`Patch::strip_from`]). Takes no parameters.
#[derive(Clone, Copy, Debug, Default)]
pub struct SurfaceSubmap;

fn params_empty(p: &Params) -> bool {
    p.0.is_null() || p.0.as_object().is_some_and(|o| o.is_empty())
}

impl Scheme for SurfaceSubmap {
    fn name(&self) -> &str {
        NAME
    }

    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }

    fn caps(&self, _params: &Params) -> Caps {
        Caps::QUAD | Caps::BLOCK_STRUCTURED
    }

    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, params: &Params) -> Applicability {
        let Target::Surface(f) = target else {
            return Applicability::no("submap meshes surfaces");
        };
        if !params_empty(params) {
            return Applicability::no("submap takes no parameters");
        }
        let topo = cx.input.topo;
        if geom::face_loops(topo, f).len() != 1 {
            return Applicability::no(
                "submap needs exactly one boundary loop (inner loops are deferred)",
            );
        }
        if !cx.input.embedded_edges(f).is_empty() || !cx.input.embedded_vertices(f).is_empty() {
            return Applicability::no("submap cannot conform to embedded edges or vertices");
        }
        let corners = match geom::corners(topo, f) {
            Ok(c) => c,
            Err(e) => return Applicability::no(e.to_string()),
        };
        if corners.len() < 4 {
            return Applicability::no(format!(
                "{} corners; submap needs at least four",
                corners.len()
            ));
        }
        // Types as the picker will see them: the firmest request wins (the later one on a tie),
        // else the nearest nominal angle.
        let mut sum = 0i32;
        for c in &corners {
            let mut ty = VertexType::nearest(c.angle);
            let mut firm: Option<Firmness> = None;
            for req in &cx.intent.corners {
                if req.corner == c.key && firm.is_none_or(|f| req.vertex_type.firmness >= f) {
                    ty = req.vertex_type.value;
                    firm = Some(req.vertex_type.firmness);
                }
            }
            if ty == VertexType::Reversal {
                return Applicability::no("reversal corners deferred");
            }
            sum += ty.submap_weight();
        }
        if sum == 4 {
            Applicability::yes(80, "corner sum 4")
        } else {
            Applicability::yes(20, "corner types need picking")
        }
    }

    fn requirements(
        &self,
        _cx: &PlanCtx<'_>,
        target: Target,
        _params: &Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        if let Target::Surface(f) = target {
            out.vertex_rule(f, VertexRule::Submap);
        }
        Ok(())
    }

    fn interval_constraints(
        &self,
        cx: &IntervalCtx<'_>,
        target: Target,
        _params: &Params,
        out: &mut IntervalSink<'_>,
    ) -> sm_scheme_api::Result<()> {
        let Target::Surface(f) = target else {
            return Ok(());
        };
        let layout = cx
            .layout(f)
            .ok_or_else(|| SchemeError::Input(format!("submap face {f} has no layout")))?;
        if layout.sides.len() < 4 {
            return Err(SchemeError::Input(format!(
                "submap face {f} has {} logical sides; at least four are needed",
                layout.sides.len()
            )));
        }
        use sm_scheme_api::SideDir::{MinusI, MinusJ, PlusI, PlusJ};
        let edges = |dir: SideDir| -> Vec<_> {
            layout
                .sides
                .iter()
                .filter(|s| s.dir == dir)
                .flat_map(|s| s.coedges.iter().map(|c| c.edge))
                .collect()
        };
        for (plus, minus) in [(PlusI, MinusI), (PlusJ, MinusJ)] {
            let (lhs, rhs) = (edges(plus), edges(minus));
            // An edge on both sides (a seam) cancels; an empty row is invalid.
            if matches!(IntervalConstraint::sum_equal(&lhs, &rhs), IntervalConstraint::Linear { terms, .. } if terms.is_empty())
            {
                continue;
            }
            out.equal(&lhs, &rhs);
        }
        Ok(())
    }

    fn generate(
        &self,
        cx: &GenCtx<'_>,
        target: Target,
        params: &Params,
    ) -> Result<Patch, GenerateFailure> {
        if !params_empty(params) {
            return Err(GenerateFailure::new(
                target,
                NAME,
                FailureKind::NotApplicable,
                "submap takes no parameters",
            ));
        }
        generate::run(cx, target)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn register_is_callable_once() {
        let mut reg = sm_scheme_api::SchemeRegistry::new();
        super::register(&mut reg).expect("register");
        assert!(super::register(&mut reg).is_err());
        assert!(
            reg.get(sm_scheme_api::SchemeDim::Surface, super::NAME)
                .is_some()
        );
    }
}
