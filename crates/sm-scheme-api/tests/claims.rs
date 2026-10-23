//! Impositions on claimed children are checked against what the claimer writes there, and
//! imposition firmness is honoured (soft = preference, hard = requirement).

mod stub_schemes;

use sm_scheme_api::{
    Applicability, Caps, Driver, FailureKind, Firmness, GenCtx, GenerateFailure, MeshInput,
    MeshIntent, Patch, PlanCtx, ReferenceCornerPicker, ReferenceIntervalSolver, Requirements,
    Scheme, SchemeChoice, SchemeDim, SchemeError, SchemeRegistry, Selection, Target, WholeFaces,
};
use stub_schemes::{brick_setup, registry, row_setup};

/// A volume scheme that claims the faces named in its `claim` list and imposes a soft
/// capability on them when `soft` is given.
struct ClaimTet;

impl Scheme for ClaimTet {
    fn name(&self) -> &str {
        "claimtet"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn caps(&self, _p: &sm_scheme_api::Params) -> Caps {
        Caps::TET | Caps::MESHES_CHILDREN
    }
    fn applicability(
        &self,
        _cx: &PlanCtx<'_>,
        _t: Target,
        _p: &sm_scheme_api::Params,
    ) -> Applicability {
        Applicability::yes(10, "test claimer")
    }
    fn requirements(
        &self,
        cx: &PlanCtx<'_>,
        target: Target,
        _p: &sm_scheme_api::Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        // Claim the faces shared with another selected volume.
        let others: Vec<Target> = cx
            .input
            .selection
            .iter()
            .map(|v| Target::Volume(*v))
            .filter(|t| *t != target)
            .collect();
        for f in cx.input.piece_faces(target)? {
            for o in &others {
                if cx.input.piece_faces(*o)?.contains(&f) {
                    out.claim(Target::Surface(f));
                }
            }
        }
        Ok(())
    }
    fn generate(
        &self,
        _cx: &GenCtx<'_>,
        target: Target,
        _p: &sm_scheme_api::Params,
    ) -> Result<Patch, GenerateFailure> {
        Err(GenerateFailure::new(
            target,
            "claimtet",
            FailureKind::Unsupported,
            "plan-level stub",
        ))
    }
}

/// A volume scheme that only prefers (soft) polygonal faces.
struct SoftPoly;

impl Scheme for SoftPoly {
    fn name(&self) -> &str {
        "softpoly"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn caps(&self, _p: &sm_scheme_api::Params) -> Caps {
        Caps::TET
    }
    fn applicability(
        &self,
        _cx: &PlanCtx<'_>,
        _t: Target,
        _p: &sm_scheme_api::Params,
    ) -> Applicability {
        Applicability::yes(10, "soft imposer")
    }
    fn requirements(
        &self,
        cx: &PlanCtx<'_>,
        target: Target,
        _p: &sm_scheme_api::Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        for f in cx.input.piece_faces(target)? {
            out.require_caps(Target::Surface(f), Caps::POLY, Firmness::Soft);
        }
        Ok(())
    }
    fn generate(
        &self,
        _cx: &GenCtx<'_>,
        target: Target,
        _p: &sm_scheme_api::Params,
    ) -> Result<Patch, GenerateFailure> {
        Err(GenerateFailure::new(
            target,
            "softpoly",
            FailureKind::Unsupported,
            "plan-level stub",
        ))
    }
}

fn reg() -> SchemeRegistry {
    let mut r = registry();
    r.register(Box::new(ClaimTet)).expect("claimtet");
    r.register(Box::new(SoftPoly)).expect("softpoly");
    r
}

#[test]
fn a_hard_imposition_on_a_claimed_face_is_a_conflict() {
    let (topo, ids, model) = row_setup(2);
    let reg = reg();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids[0].volume, ids[1].volume];
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let mut intent = MeshIntent::default();
    // A claims the shared face (tet-like caps); B is a hard mapvol that needs quad/structured there.
    intent.set_scheme(ids[0].volume, SchemeChoice::new("claimtet", Firmness::Hard));
    intent.set_scheme(ids[1].volume, SchemeChoice::new("mapvol", Firmness::Hard));
    match driver.plan(&input, &intent) {
        Err(SchemeError::ImpositionConflict { child, .. }) => {
            assert!(matches!(child, Target::Surface(_)))
        }
        other => panic!(
            "expected an imposition conflict on the claimed face, got {:?}",
            other.map(|_| ())
        ),
    }
    // Without the conflicting hard sweep-like requirement the claim stands.
    let mut ok = MeshIntent::default();
    ok.set_scheme(ids[0].volume, SchemeChoice::new("claimtet", Firmness::Hard));
    ok.set_scheme(ids[1].volume, SchemeChoice::new("tetvol", Firmness::Hard));
    let plan = driver.plan(&input, &ok).expect("plan");
    let shared = ids[0]
        .faces
        .iter()
        .find(|f| ids[1].faces.contains(f))
        .or_else(|| {
            ids[0].faces.iter().find(|f| {
                input
                    .piece_faces(Target::Volume(ids[1].volume))
                    .is_ok_and(|v| v.contains(f))
            })
        })
        .copied()
        .expect("shared face");
    assert_eq!(
        plan.claimed_by(Target::Surface(shared)),
        Some(Target::Volume(ids[0].volume))
    );
    assert!(matches!(
        plan.target(Target::Surface(shared)).map(|t| &t.selection),
        Some(Selection::Claimed { .. })
    ));
}

#[test]
fn soft_impositions_are_preferences() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = reg();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let mut intent = MeshIntent::default();
    intent.set_scheme(ids.volume, SchemeChoice::new("softpoly", Firmness::Hard));
    // No surface scheme is polygonal: the soft preference falls away instead of failing.
    let plan = driver
        .plan(&input, &intent)
        .expect("soft preference nobody meets");
    assert!(plan.target(Target::Surface(ids.faces[0])).is_some());
    // A hard intent on the face wins over the soft imposition.
    intent.set_scheme(ids.faces[0], SchemeChoice::new("tri", Firmness::Hard));
    let plan = driver
        .plan(&input, &intent)
        .expect("hard intent beats a soft imposition");
    assert_eq!(
        plan.target(Target::Surface(ids.faces[0]))
            .map(|t| t.scheme.as_str()),
        Some("tri")
    );
}
