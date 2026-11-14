//! The reserved volume scheme `polyhedron`: `mesh.run` makes tets and dualizes them in one step.
//!
//! `mesh.scheme(targets, "polyhedron", params: {dual?})` is lowered at run time: the intent the
//! driver sees names `tetmesh` instead (the other params go to it), and the run then does what
//! `mesh.to_poly(keep_source: false)` does on the tet mesh. One mesh entry results.

use serde_json::Value;
use sm_core::{EntityId, EntityKind};
use sm_op_api::OpError;
use sm_poly::DualPoints;
use sm_scheme_api::{MeshIntent, Params, SchemeChoice, SchemeName};

/// The reserved scheme name (volumes only).
pub const POLYHEDRON: &str = "polyhedron";
/// The scheme the tets come from.
pub(crate) const TET: &str = "tetmesh";

/// Split `polyhedron` params into the dual rule and the params left for `tetmesh`.
pub(crate) fn split_params(p: &Params) -> Result<(DualPoints, Params), OpError> {
    let mut v = p.0.clone();
    let mut dual = DualPoints::Barycentric;
    if let Value::Object(m) = &mut v {
        if let Some(d) = m.remove("dual") {
            dual = match d.as_str() {
                Some("barycentric") => DualPoints::Barycentric,
                Some("circumcentric") => DualPoints::Circumcentric,
                _ => {
                    return Err(OpError::Invalid(format!(
                        "polyhedron: `dual` is \"barycentric\" or \"circumcentric\", got {d}"
                    )));
                }
            };
        }
        if m.is_empty() {
            return Ok((dual, Params::empty()));
        }
    }
    Ok((dual, Params(v)))
}

/// The intent with every `polyhedron` choice replaced by `tetmesh`, and the dual rule, when the
/// volumes in `targets` run as polyhedra. `None`: no target uses the scheme. A mix of polyhedron
/// and other volumes, or two dual rules, in one run is refused.
pub(crate) fn lower(
    intent: &MeshIntent,
    targets: &[EntityId],
) -> Result<Option<(MeshIntent, DualPoints)>, OpError> {
    let is_poly =
        |c: &Option<SchemeChoice>| c.as_ref().is_some_and(|c| c.name.as_str() == POLYHEDRON);
    if !intent.entities.values().any(|e| is_poly(&e.scheme)) {
        return Ok(None);
    }
    let mut out = intent.clone();
    for e in out.entities.values_mut() {
        if let Some(c) = e.scheme.as_mut().filter(|c| c.name.as_str() == POLYHEDRON) {
            c.params = split_params(&c.params)?.1;
            c.name = SchemeName::new(TET);
        }
    }
    let mut dual: Option<DualPoints> = None;
    let (mut n_poly, mut n_vol) = (0usize, 0usize);
    for t in targets.iter().filter(|t| t.kind() == EntityKind::TopoVolume) {
        n_vol += 1;
        let Some(e) = intent.entities.get(t) else {
            continue;
        };
        if !is_poly(&e.scheme) {
            continue;
        }
        n_poly += 1;
        let d = split_params(&e.scheme.as_ref().map_or_else(Params::empty, |c| c.params.clone()))?.0;
        if dual.is_some_and(|x| x != d) {
            return Err(OpError::Invalid(
                "mesh.run: the polyhedron volumes of one run need the same `dual`; run them apart"
                    .into(),
            ));
        }
        dual = Some(d);
    }
    match dual {
        Some(d) if n_poly == n_vol => Ok(Some((out, d))),
        Some(_) => Err(OpError::Invalid(
            "mesh.run: polyhedron volumes and other volumes cannot share one run (one mesh \
             entry, one cell type); run them apart"
                .into(),
        )),
        None => Ok(None),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use sm_scheme_api::Firmness;

    fn vol(n: u64) -> EntityId {
        EntityId::new(EntityKind::TopoVolume, n).expect("id")
    }

    #[test]
    fn params_split_dual_from_tet_params() {
        let p = Params(json!({"dual": "circumcentric", "x": 1}));
        let (d, rest) = split_params(&p).expect("ok");
        assert_eq!(d, DualPoints::Circumcentric);
        assert_eq!(rest.0, json!({"x": 1}));
        let (d, rest) = split_params(&Params::empty()).expect("ok");
        assert_eq!(d, DualPoints::Barycentric);
        assert!(rest.is_empty());
        assert!(split_params(&Params(json!({"dual": "voronoi"}))).is_err());
        assert!(split_params(&Params(json!({"dual": 3}))).is_err());
    }

    #[test]
    fn lower_rewrites_scheme_and_refuses_mixes() {
        let mut i = MeshIntent::new();
        i.entity_mut(vol(1)).scheme = Some(SchemeChoice::with_params(
            POLYHEDRON,
            json!({"dual": "circumcentric"}),
            Firmness::Hard,
        ));
        let (low, d) = lower(&i, &[vol(1)]).expect("ok").expect("some");
        assert_eq!(d, DualPoints::Circumcentric);
        let c = low.entities[&vol(1)].scheme.as_ref().expect("scheme");
        assert_eq!(c.name.as_str(), TET);
        assert!(c.params.is_empty());
        // A second volume without the scheme in the same run is refused; apart it is plain.
        assert!(lower(&i, &[vol(1), vol(2)]).is_err());
        assert!(lower(&i, &[vol(2)]).expect("ok").is_none());
        // Two dual rules are refused.
        i.entity_mut(vol(2)).scheme = Some(SchemeChoice::new(POLYHEDRON, Firmness::Hard));
        assert!(lower(&i, &[vol(1), vol(2)]).is_err());
        assert!(lower(&MeshIntent::new(), &[vol(1)]).expect("ok").is_none());
    }
}
